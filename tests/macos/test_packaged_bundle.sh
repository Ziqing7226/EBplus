#!/usr/bin/env bash
# Verifies the portable parts of a finished macOS app bundle.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
app_path="${1:-$repo_root/build/dist/EBplus.app}"

[[ -d "$app_path" ]] || {
    printf 'App bundle not found: %s\n' "$app_path" >&2
    exit 1
}

codesign --verify --deep --strict "$app_path"

for notice in \
    EBplus-MIT.txt \
    OpenEB-LICENSE_OPEN.txt \
    OpenEB-OPEN_SOURCE_3RDPARTY_NOTICES.txt; do
    [[ -f "$app_path/Contents/Resources/licenses/$notice" ]] || {
        printf 'Missing bundled license notice: %s\n' "$notice" >&2
        exit 1
    }
done

[[ -f "$app_path/Contents/PlugIns/imageformats/libqsvg.dylib" ]] || {
    printf 'Missing Qt SVG image format plugin: %s\n' \
        "$app_path/Contents/PlugIns/imageformats/libqsvg.dylib" >&2
    exit 1
}

[[ -f "$app_path/Contents/Frameworks/QtSvg.framework/Versions/A/QtSvg" ]] || {
    printf 'Missing Qt SVG framework: %s\n' \
        "$app_path/Contents/Frameworks/QtSvg.framework/Versions/A/QtSvg" >&2
    exit 1
}

otool -l "$app_path/Contents/PlugIns/imageformats/libqsvg.dylib" | \
    grep -Fq 'path @loader_path/../../Frameworks ' || {
    printf 'Qt SVG image format plugin is missing the bundled-framework rpath\n' >&2
    exit 1
}

while IFS= read -r -d '' binary; do
    file "$binary" | grep -q 'Mach-O' || continue
    if otool -L "$binary" | grep -Eq '^[[:space:]]*/(opt/homebrew|usr/local)/'; then
        printf 'Found non-portable dylib reference in: %s\n' "$binary" >&2
        exit 1
    fi
    if otool -l "$binary" | awk '
        $1 == "cmd" && $2 == "LC_RPATH" { in_rpath = 1; next }
        in_rpath && $1 == "path" { print $2; in_rpath = 0 }
    ' | grep -q '^/'; then
        printf 'Found absolute rpath in: %s\n' "$binary" >&2
        exit 1
    fi
    while IFS= read -r dependency; do
        case "$dependency" in
            @rpath/*.framework/*)
                dependency_path="$app_path/Contents/Frameworks/${dependency#@rpath/}"
                ;;
            @rpath/*)
                dependency_path="$app_path/Contents/Frameworks/${dependency##*/}"
                ;;
            *)
                continue
                ;;
        esac
        [[ -e "$dependency_path" || \
           -e "$(dirname "$binary")/${dependency##*/}" ]] || {
            printf 'Missing bundled rpath dependency for %s: %s\n' \
                "$binary" "$dependency" >&2
            exit 1
        }
    done < <(otool -L "$binary" | awk 'NR > 1 { print $1 }')
done < <(/usr/bin/find "$app_path/Contents" -type f -print0)
