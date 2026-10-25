#!/usr/bin/env bash
# Guards release requirements that are easy to miss in a local-only build.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
package_script="$repo_root/scripts/build_macos_app.sh"

require_contains() {
    local needle="$1"
    rg -Fq -- "$needle" "$package_script" || {
        printf 'Expected %s in %s\n' "$needle" "$package_script" >&2
        exit 1
    }
}

require_contains 'copy_license_notices()'
require_contains 'OPEN_SOURCE_3RDPARTY_NOTICES'
require_contains 'copy_license_notices "$APP_BUNDLE"'
require_contains 'deploy_qt_runtime()'
require_contains 'deploy_qt_framework_dependencies()'
require_contains 'copy_framework()'
require_contains 'QT_PREFIX="${QT_PREFIX:-$(brew_prefix qtbase)}"'
require_contains 'QTSVG_PREFIX="${QTSVG_PREFIX:-$(brew_prefix qtsvg)}"'
require_contains 'QtCore.framework'
require_contains '$QTSVG_PREFIX/lib/QtSvg.framework'
require_contains 'PlugIns/platforms/libqcocoa.dylib'
require_contains 'PlugIns/imageformats/libqsvg.dylib'
require_contains 'fix_framework_install_name()'
require_contains 'strip_absolute_rpaths()'
require_contains '-delete_rpath'
require_contains 'copy_rpath_dylib_dependencies()'
require_contains 'Preserving OpenEB library alias'
require_contains '@rpath/$framework_name/$framework_relative'
