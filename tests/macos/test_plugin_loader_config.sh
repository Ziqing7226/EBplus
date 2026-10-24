#!/usr/bin/env bash
# Verifies that a packaged macOS app loads HAL plugins only from its explicit path.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
main_cpp="$repo_root/gui/main.cpp"
resources_cpp="$repo_root/openeb/hal/cpp/src/utils/resources_folder.cpp"

require_contains() {
    local needle="$1"
    local file="$2"
    rg -Fq "$needle" "$file" || {
        printf 'Expected %s in %s\n' "$needle" "$file" >&2
        exit 1
    }
}

require_absent() {
    local needle="$1"
    local file="$2"
    ! rg -Fq "$needle" "$file" || {
        printf 'Did not expect %s in %s\n' "$needle" "$file" >&2
        exit 1
    }
}

require_contains 'set_env_if_unset("MV_HAL_PLUGIN_SEARCH_MODE", "PLUGIN_PATH_ONLY");' "$main_cpp"
require_absent 'get_env_plugin_install_path' "$resources_cpp"
require_absent '/opt/homebrew/lib/metavision/hal/plugins' "$main_cpp"
