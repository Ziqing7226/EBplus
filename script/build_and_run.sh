#!/usr/bin/env bash
# Project-local build/run entrypoint for the Codex macOS Run action.
set -euo pipefail

MODE="${1:-run}"
APP_NAME="EBplus"
BUNDLE_ID="org.ebplus.gui-for-openeb"

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
APP_BUNDLE="$ROOT_DIR/build/dist/$APP_NAME.app"
APP_BINARY="$APP_BUNDLE/Contents/MacOS/$APP_NAME"

usage() {
    echo "usage: $0 [run|--debug|--logs|--telemetry|--verify]" >&2
    exit 2
}

build_app() {
    CREATE_DMG=OFF "$ROOT_DIR/scripts/build_macos_app.sh"
}

open_app() {
    /usr/bin/open -n "$APP_BUNDLE"
}

pkill -x "$APP_NAME" >/dev/null 2>&1 || true

case "$MODE" in
    run)
        build_app
        open_app
        ;;
    --debug|debug)
        build_app
        lldb -- "$APP_BINARY"
        ;;
    --logs|logs)
        build_app
        open_app
        /usr/bin/log stream --info --style compact --predicate "process == \"$APP_NAME\""
        ;;
    --telemetry|telemetry)
        build_app
        open_app
        /usr/bin/log stream --info --style compact --predicate "subsystem == \"$BUNDLE_ID\" OR process == \"$APP_NAME\""
        ;;
    --verify|verify)
        build_app
        open_app
        sleep 2
        pgrep -x "$APP_NAME" >/dev/null
        ;;
    *)
        usage
        ;;
esac
