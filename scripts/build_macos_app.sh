#!/usr/bin/env bash
# Build and package EBplus as a macOS app bundle and DMG.
#
# Defaults target Apple Silicon and the vendored OpenEB tree. Set
# EXTRA_HAL_PLUGIN_PATH to copy additional vendor HAL plugins, for example a
# silkyCam plugin directory, into the app bundle.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

APP_NAME="${APP_NAME:-EBplus}"
ARCH="${ARCH:-arm64}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
BUILD_ROOT="${BUILD_ROOT:-$REPO_ROOT/build}"
OPENEB_BUILD_DIR="${OPENEB_BUILD_DIR:-$BUILD_ROOT/openeb-macos-$ARCH}"
OPENEB_INSTALL_PREFIX="${OPENEB_INSTALL_PREFIX:-$BUILD_ROOT/openeb-install}"
APP_BUILD_DIR="${APP_BUILD_DIR:-$BUILD_ROOT/macos-$ARCH}"
DIST_DIR="${DIST_DIR:-$BUILD_ROOT/dist}"
OPENEB_HDF5_DISABLED="${OPENEB_HDF5_DISABLED:-ON}"
CREATE_DMG="${CREATE_DMG:-ON}"
CODESIGN="${CODESIGN:-ON}"

log() {
    printf '[macos] %s\n' "$*"
}

require_cmd() {
    if ! command -v "$1" >/dev/null 2>&1; then
        printf '[macos] Missing required command: %s\n' "$1" >&2
        exit 1
    fi
}

cmake_prefix_path() {
    local paths=("$@")
    local value=""
    local path
    for path in "${paths[@]}"; do
        [[ -d "$path" ]] || continue
        if [[ -n "$value" ]]; then
            value+=";"
        fi
        value+="$path"
    done
    printf '%s' "$value"
}

brew_prefix() {
    brew --prefix "$1" 2>/dev/null || true
}

is_macho() {
    [[ -f "$1" ]] && file "$1" | grep -q 'Mach-O'
}

add_rpath() {
    local binary="$1"
    local rpath="$2"
    otool -l "$binary" | grep -Fq "path $rpath " && return 0
    install_name_tool -add_rpath "$rpath" "$binary" 2>/dev/null || true
}

strip_absolute_rpaths() {
    local binary="$1"
    local rpath

    while IFS= read -r rpath; do
        [[ "$rpath" = /* ]] || continue
        install_name_tool -delete_rpath "$rpath" "$binary" 2>/dev/null || true
    done < <(otool -l "$binary" | awk '
        $1 == "cmd" && $2 == "LC_RPATH" { in_rpath = 1; next }
        in_rpath && $1 == "path" { print $2; in_rpath = 0 }
    ')
}

copy_rpath_dylib_dependencies() {
    local binary="$1"
    local frameworks_dir="$2"
    local dep base rpath

    while IFS= read -r dep; do
        dep="${dep%% (*}"
        dep="${dep#"${dep%%[![:space:]]*}"}"
        case "$dep" in
            @rpath/*.dylib)
                base="${dep##*/}"
                [[ -e "$frameworks_dir/$base" || -e "$(dirname "$binary")/$base" ]] && continue
                while IFS= read -r rpath; do
                    [[ -f "$rpath/$base" ]] || continue
                    copy_dylib "$rpath/$base" "$frameworks_dir" || true
                    break
                done < <({
                    otool -l "$binary" | awk '
                        $1 == "cmd" && $2 == "LC_RPATH" { in_rpath = 1; next }
                        in_rpath && $1 == "path" { print $2; in_rpath = 0 }
                    ' | grep '^/' || true
                    printf '%s\n' \
                        "$OPENEB_INSTALL_PREFIX/lib" \
                        "$BREW_PREFIX/lib" \
                        "$BREW_PREFIX/lib/gcc/current"
                } | awk '!seen[$0]++')
                ;;
        esac
    done < <(otool -L "$binary" | tail -n +2)
}

copy_dylib() {
    local source="$1"
    local frameworks_dir="$2"
    local base dest
    base="$(basename "$source")"
    dest="$frameworks_dir/$base"
    [[ -f "$source" ]] || return 1
    if [[ ! -e "$dest" ]]; then
        log "Copying dependency $base"
        cp -L "$source" "$dest"
        chmod u+w "$dest"
        install_name_tool -id "@rpath/$base" "$dest" 2>/dev/null || true
        return 0
    fi
    return 1
}

copy_framework() {
    local source_framework="$1"
    local frameworks_dir="$2"
    local framework_name destination

    [[ -d "$source_framework" ]] || return 1
    framework_name="$(basename "$source_framework")"
    destination="$frameworks_dir/$framework_name"
    [[ -e "$destination" ]] && return 1

    log "Copying Qt framework $framework_name"
    cp -R "$source_framework" "$frameworks_dir/"
}

fix_binary_dependencies() {
    local binary="$1"
    local frameworks_dir="$2"
    local dep base framework_path framework_name framework_relative

    while IFS= read -r dep; do
        dep="${dep%% (*}"
        dep="${dep#"${dep%%[![:space:]]*}"}"
        case "$dep" in
            ""|"$binary:"|/System/*|/usr/lib/*|@executable_path/*|@loader_path/*|@rpath/*)
                continue
                ;;
            *.framework/*)
                framework_path="${dep%%.framework/*}.framework"
                framework_name="$(basename "$framework_path")"
                framework_relative="${dep#"$framework_path"/}"
                if [[ "$dep" = /* && -d "$framework_path" ]]; then
                    copy_framework "$framework_path" "$frameworks_dir" || true
                fi
                if [[ -f "$frameworks_dir/$framework_name/$framework_relative" ]]; then
                    install_name_tool -change "$dep" \
                        "@rpath/$framework_name/$framework_relative" "$binary" 2>/dev/null || true
                fi
                continue
                ;;
        esac

        if [[ "$dep" = /* && -f "$dep" ]]; then
            base="$(basename "$dep")"
            copy_dylib "$dep" "$frameworks_dir" || true
            install_name_tool -change "$dep" "@rpath/$base" "$binary" 2>/dev/null || true
        fi
    done < <(otool -L "$binary" | tail -n +2)
}

fix_framework_install_name() {
    local binary="$1"
    local frameworks_dir="$2"
    local relative_path framework_name framework_relative

    [[ "$binary" == "$frameworks_dir/"*.framework/* ]] || return 0

    relative_path="${binary#"$frameworks_dir/"}"
    framework_name="${relative_path%%.framework/*}.framework"
    framework_relative="${relative_path#"$framework_name"/}"
    install_name_tool -id "@rpath/$framework_name/$framework_relative" "$binary" 2>/dev/null || true
}

fix_dylib_install_name() {
    local binary="$1"
    local frameworks_dir="$2"
    local base

    case "$binary" in
        "$frameworks_dir/"*.dylib)
            base="$(basename "$binary")"
            install_name_tool -id "@rpath/$base" "$binary" 2>/dev/null || true
            ;;
    esac
}

deploy_qt_runtime() {
    local app="$1"
    local frameworks_dir="$app/Contents/Frameworks"
    local plugins_source="$QT_PREFIX/share/qt/plugins"
    local qtsvg_imageformats_source="$QTSVG_PREFIX/share/qt/plugins/imageformats"
    local framework

    [[ -d "$QT_PREFIX/lib/QtCore.framework" ]] || {
        printf '[macos] Qt base frameworks not found under: %s\n' "$QT_PREFIX" >&2
        exit 1
    }
    [[ -f "$qtsvg_imageformats_source/libqsvg.dylib" ]] || {
        printf '[macos] Qt SVG image format plugin not found under: %s\n' \
            "$qtsvg_imageformats_source" >&2
        exit 1
    }
    [[ -d "$QTSVG_PREFIX/lib/QtSvg.framework" ]] || {
        printf '[macos] QtSvg framework not found under: %s\n' "$QTSVG_PREFIX" >&2
        exit 1
    }

    log "Deploying Qt frameworks and plugins"
    mkdir -p "$frameworks_dir" "$app/Contents/PlugIns"
    for framework in \
        "$QT_PREFIX/lib/QtCore.framework" \
        "$QT_PREFIX/lib/QtGui.framework" \
        "$QT_PREFIX/lib/QtWidgets.framework" \
        "$QT_PREFIX/lib/QtOpenGL.framework" \
        "$QT_PREFIX/lib/QtOpenGLWidgets.framework" \
        "$QTSVG_PREFIX/lib/QtSvg.framework"; do
        copy_framework "$framework" "$frameworks_dir" || true
    done

    if [[ -d "$plugins_source" ]]; then
        for framework in platforms imageformats iconengines styles; do
            [[ -d "$plugins_source/$framework" ]] || continue
            rsync -a "$plugins_source/$framework/" "$app/Contents/PlugIns/$framework/"
        done
    fi
    rsync -a "$qtsvg_imageformats_source/" "$app/Contents/PlugIns/imageformats/"

    [[ -d "$app/Contents/Frameworks/QtCore.framework" ]] || {
        printf '[macos] Qt runtime deployment did not copy QtCore.framework\n' >&2
        exit 1
    }
    [[ -f "$app/Contents/PlugIns/platforms/libqcocoa.dylib" ]] || {
        printf '[macos] Qt runtime deployment did not copy the Cocoa platform plugin\n' >&2
        exit 1
    }
    [[ -f "$app/Contents/PlugIns/imageformats/libqsvg.dylib" ]] || {
        printf '[macos] Qt runtime deployment did not copy the SVG image format plugin\n' >&2
        exit 1
    }
    [[ -d "$app/Contents/Frameworks/QtSvg.framework" ]] || {
        printf '[macos] Qt runtime deployment did not copy QtSvg.framework\n' >&2
        exit 1
    }
}

deploy_qt_framework_dependencies() {
    local app="$1"
    local frameworks_dir="$app/Contents/Frameworks"
    local before after file dep framework_name source_prefix source_framework round

    log "Resolving Qt framework dependencies"
    for round in $(seq 1 20); do
        before="$(find "$frameworks_dir" -maxdepth 1 -type d -name 'Qt*.framework' | wc -l | tr -d ' ')"
        while IFS= read -r -d '' file; do
            is_macho "$file" || continue
            while IFS= read -r dep; do
                dep="${dep%% (*}"
                dep="${dep#"${dep%%[![:space:]]*}"}"
                case "$dep" in
                    @rpath/Qt*.framework/*)
                        framework_name="${dep#@rpath/}"
                        framework_name="${framework_name%%/*}"
                        [[ -d "$frameworks_dir/$framework_name" ]] && continue
                        for source_prefix in "$QT_PREFIX" "$QTSVG_PREFIX"; do
                            source_framework="$source_prefix/lib/$framework_name"
                            if [[ -d "$source_framework" ]]; then
                                copy_framework "$source_framework" "$frameworks_dir" || true
                                break
                            fi
                        done
                        [[ -d "$frameworks_dir/$framework_name" ]] || {
                            printf '[macos] Missing Qt framework dependency: %s\n' \
                                "$framework_name" >&2
                            exit 1
                        }
                        ;;
                esac
            done < <(otool -L "$file" | tail -n +2)
        done < <(find "$app/Contents" -type f -print0)

        after="$(find "$frameworks_dir" -maxdepth 1 -type d -name 'Qt*.framework' | wc -l | tr -d ' ')"
        [[ "$after" = "$before" ]] && break
    done
}

deploy_non_qt_dependencies() {
    local app="$1"
    local frameworks_dir="$app/Contents/Frameworks"
    local plugin_dir="$app/Contents/PlugIns/metavision/hal/plugins"
    local resources_dir="$app/Contents/Resources/metavision/hal"
    local before after file round

    mkdir -p "$frameworks_dir" "$plugin_dir" "$resources_dir"

    log "Copying OpenEB runtime libraries"
    find "$OPENEB_INSTALL_PREFIX/lib" -maxdepth 1 -type f -name '*.dylib' -print0 |
        while IFS= read -r -d '' file; do
            copy_dylib "$file" "$frameworks_dir" || true
        done
    find "$OPENEB_INSTALL_PREFIX/lib" -maxdepth 1 -type l -name '*.dylib' -print0 |
        while IFS= read -r -d '' file; do
            local alias_path="$frameworks_dir/$(basename "$file")"
            [[ -e "$alias_path" || -L "$alias_path" ]] && continue
            log "Preserving OpenEB library alias $(basename "$file")"
            cp -P "$file" "$alias_path"
        done

    log "Copying OpenEB HAL plugins"
    if [[ -d "$OPENEB_INSTALL_PREFIX/lib/metavision/hal/plugins" ]]; then
        rsync -a "$OPENEB_INSTALL_PREFIX/lib/metavision/hal/plugins/" "$plugin_dir/"
    fi
    local extra_plugin_paths="${EXTRA_HAL_PLUGIN_PATHS:-${EXTRA_HAL_PLUGIN_PATH:-${MV_HAL_PLUGIN_PATH:-}}}"
    if [[ -n "$extra_plugin_paths" ]]; then
        local extra_path
        IFS=':' read -r -a extra_paths <<<"$extra_plugin_paths"
        for extra_path in "${extra_paths[@]}"; do
            [[ -d "$extra_path" ]] || continue
            [[ "$extra_path" == "$OPENEB_INSTALL_PREFIX/lib/metavision/hal/plugins" ]] && continue
            log "Copying extra HAL plugins from $extra_path"
            rsync -a "$extra_path/" "$plugin_dir/"
        done
    fi

    if [[ -d "$OPENEB_INSTALL_PREFIX/share/metavision/hal" ]]; then
        log "Copying OpenEB HAL resources"
        rsync -a "$OPENEB_INSTALL_PREFIX/share/metavision/hal/" "$resources_dir/"
    fi

    log "Rewriting non-Qt dylib dependencies"
    for round in $(seq 1 20); do
        before="$(find "$frameworks_dir" -type f -name '*.dylib' | wc -l | tr -d ' ')"
        while IFS= read -r -d '' file; do
            is_macho "$file" || continue
            copy_rpath_dylib_dependencies "$file" "$frameworks_dir"
            strip_absolute_rpaths "$file"
            case "$file" in
                "$app/Contents/MacOS/"*)
                    add_rpath "$file" "@executable_path/../Frameworks"
                    ;;
                "$plugin_dir/"*)
                    add_rpath "$file" "@loader_path"
                    add_rpath "$file" "@loader_path/../../../../Frameworks"
                    ;;
                "$app/Contents/PlugIns/"*)
                    add_rpath "$file" "@loader_path/../../Frameworks"
                    ;;
                "$frameworks_dir/"*)
                    add_rpath "$file" "@loader_path"
                    ;;
            esac
            fix_dylib_install_name "$file" "$frameworks_dir"
            fix_framework_install_name "$file" "$frameworks_dir"
            fix_binary_dependencies "$file" "$frameworks_dir"
        done < <(find "$app/Contents" -type f -print0)

        after="$(find "$frameworks_dir" -type f -name '*.dylib' | wc -l | tr -d ' ')"
        [[ "$after" = "$before" ]] && break
    done
}

copy_license_notices() {
    local app="$1"
    local licenses_dir="$app/Contents/Resources/licenses"
    local openeb_licenses="$OPENEB_INSTALL_PREFIX/share/metavision/licensing"

    mkdir -p "$licenses_dir"
    cp "$REPO_ROOT/LICENSE" "$licenses_dir/EBplus-MIT.txt"

    if [[ -f "$openeb_licenses/LICENSE_OPEN" ]]; then
        cp "$openeb_licenses/LICENSE_OPEN" "$licenses_dir/OpenEB-LICENSE_OPEN.txt"
    fi
    if [[ -f "$openeb_licenses/OPEN_SOURCE_3RDPARTY_NOTICES" ]]; then
        cp "$openeb_licenses/OPEN_SOURCE_3RDPARTY_NOTICES" \
            "$licenses_dir/OpenEB-OPEN_SOURCE_3RDPARTY_NOTICES.txt"
    fi
}

clean_deployed_bundle_contents() {
    local app="$1"

    log "Cleaning stale deployed bundle contents"
    rm -rf "$app/Contents/Frameworks"
    rm -rf "$app/Contents/PlugIns"
    rm -rf "$app/Contents/Resources/metavision"
}

ad_hoc_sign_app() {
    local app="$1"
    local file

    [[ "$CODESIGN" = "ON" ]] || return 0
    require_cmd codesign

    log "Applying ad-hoc code signatures"
    while IFS= read -r -d '' file; do
        is_macho "$file" || continue
        codesign --force --sign - "$file" >/dev/null 2>&1 || true
    done < <(find "$app/Contents" -type f -print0)

    codesign --force --deep --sign - "$app" >/dev/null
    codesign --verify --deep --strict "$app"
}

create_dmg() {
    local app="$1"
    local dmg_root="$DIST_DIR/dmg-root"
    local dmg_path="$DIST_DIR/$APP_NAME-macos-$ARCH.dmg"

    rm -rf "$dmg_root" "$dmg_path"
    mkdir -p "$dmg_root" "$DIST_DIR"
    cp -R "$app" "$dmg_root/"
    ln -s /Applications "$dmg_root/Applications"

    log "Creating $dmg_path"
    hdiutil create -volname "$APP_NAME" -srcfolder "$dmg_root" -ov -format UDZO "$dmg_path"
    log "DMG ready: $dmg_path"
}

require_cmd cmake
require_cmd file
require_cmd hdiutil
require_cmd install_name_tool
require_cmd otool
require_cmd rsync

BREW_PREFIX="$(brew --prefix 2>/dev/null || true)"
QT_PREFIX="${QT_PREFIX:-$(brew_prefix qtbase)}"
QTSVG_PREFIX="${QTSVG_PREFIX:-$(brew_prefix qtsvg)}"
OPENCV_PREFIX="${OPENCV_PREFIX:-$(brew_prefix opencv)}"
BOOST_PREFIX="${BOOST_PREFIX:-$(brew_prefix boost)}"
LIBUSB_PREFIX="${LIBUSB_PREFIX:-$(brew_prefix libusb)}"
PROTOBUF_PREFIX="${PROTOBUF_PREFIX:-$(brew_prefix protobuf)}"
ABSEIL_PREFIX="${ABSEIL_PREFIX:-$(brew_prefix abseil)}"

COMMON_PREFIX_PATH="$(cmake_prefix_path \
    "$BREW_PREFIX" \
    "$QT_PREFIX" \
    "$OPENCV_PREFIX" \
    "$BOOST_PREFIX" \
    "$LIBUSB_PREFIX" \
    "$PROTOBUF_PREFIX" \
    "$ABSEIL_PREFIX")"
COMMON_PREFIX_PATH="$OPENEB_INSTALL_PREFIX${COMMON_PREFIX_PATH:+;$COMMON_PREFIX_PATH}"

OPENEB_PREFIX_PATH="$(cmake_prefix_path \
    "$BREW_PREFIX" \
    "$OPENCV_PREFIX" \
    "$BOOST_PREFIX" \
    "$LIBUSB_PREFIX" \
    "$PROTOBUF_PREFIX" \
    "$ABSEIL_PREFIX")"

log "Configuring vendored OpenEB"
cmake -S "$REPO_ROOT/openeb" -B "$OPENEB_BUILD_DIR" \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DCMAKE_OSX_ARCHITECTURES="$ARCH" \
    -DCMAKE_INSTALL_PREFIX="$OPENEB_INSTALL_PREFIX" \
    -DTARGET_PLATFORM=macos \
    -DBUILD_TESTING=OFF \
    -DBUILD_SAMPLES=OFF \
    -DBUILD_APPS=OFF \
    -DCOMPILE_PYTHON3_BINDINGS=OFF \
    -DHDF5_DISABLED="$OPENEB_HDF5_DISABLED" \
    "-DMETAVISION_SELECTED_MODULES=base;core;stream" \
    -DCMAKE_PREFIX_PATH="$OPENEB_PREFIX_PATH"

log "Building and installing vendored OpenEB"
cmake --build "$OPENEB_BUILD_DIR" --target install --parallel

log "Configuring EBplus"
cmake -S "$REPO_ROOT" -B "$APP_BUILD_DIR" \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DCMAKE_OSX_ARCHITECTURES="$ARCH" \
    -DGUI_FOR_OPENEB_BUILD_TESTS=OFF \
    -DGUI_BUILD_TESTS=OFF \
    -DCMAKE_PREFIX_PATH="$COMMON_PREFIX_PATH"

log "Building EBplus.app"
cmake --build "$APP_BUILD_DIR" --target gui_for_openeb --parallel

APP_BUNDLE="$APP_BUILD_DIR/gui/$APP_NAME.app"
if [[ ! -d "$APP_BUNDLE" ]]; then
    printf '[macos] App bundle not found: %s\n' "$APP_BUNDLE" >&2
    exit 1
fi

clean_deployed_bundle_contents "$APP_BUNDLE"
deploy_qt_runtime "$APP_BUNDLE"
deploy_qt_framework_dependencies "$APP_BUNDLE"
deploy_non_qt_dependencies "$APP_BUNDLE"
copy_license_notices "$APP_BUNDLE"
ad_hoc_sign_app "$APP_BUNDLE"

mkdir -p "$DIST_DIR"
rm -rf "$DIST_DIR/$APP_NAME.app"
cp -R "$APP_BUNDLE" "$DIST_DIR/$APP_NAME.app"
log "App ready: $DIST_DIR/$APP_NAME.app"

if [[ "$CREATE_DMG" = "ON" ]]; then
    create_dmg "$DIST_DIR/$APP_NAME.app"
fi
