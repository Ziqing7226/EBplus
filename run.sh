#!/usr/bin/env bash
# run.sh — launcher for EBplus.
#
# Copy this script to run.local.sh and customise the env vars
# for your camera hardware.  The run.local.sh copy is git-ignored so you can
# keep your private HAL plugin path there.
#
# Common HAL plugin paths:
#   Prophesee (default openEB install): /usr/local/lib/metavision/hal/plugins
#   CenturyArks:                        /usr/lib/CenturyArks/hal/plugins
set -euo pipefail

# ---- resolve repo root + binary ----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$SCRIPT_DIR"
BIN="$REPO_ROOT/build/gui/gui_for_openeb"

if [[ ! -x "$BIN" ]]; then
    echo "[run.sh] Binary not found at: $BIN" >&2
    echo "[run.sh] Run 'cmake -B build && cmake --build build' first." >&2
    exit 1
fi

# ---- OpenEB runtime environment (customise for your camera) ----

# SDK shared libraries (needed if installed outside the standard linker path).
# Use ${VAR:+$VAR:} so an unset/empty variable does not produce a leading
# colon — ld.so treats an empty entry as the current working directory,
# which would let any lib*.so in the user's CWD shadow the real SDK library.
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}/usr/local/lib"

# HDF5 plugin path (for reading .hdf5 event files).
export HDF5_PLUGIN_PATH="${HDF5_PLUGIN_PATH:+$HDF5_PLUGIN_PATH:}/usr/local/lib/hdf5/plugin"

# HAL plugin path — camera driver .so files.
# Change this to match your camera vendor's install location.
#   - Prophesee:  /usr/local/lib/metavision/hal/plugins
#   - CenturyArks: /usr/lib/CenturyArks/hal/plugins
export MV_HAL_PLUGIN_PATH="${MV_HAL_PLUGIN_PATH:-/usr/local/lib/metavision/hal/plugins}"

# ---- Qt platform plugin + RHI backend ----
# Hard-won lessons from runtime testing on this host:
#   1. On Wayland sessions, older Qt/driver stacks rendered a black window
#      for QOpenGLWidget children under the native Wayland plugin; the XCB
#      plugin (via XWayland) is the reliable path → default QT_QPA_PLATFORM
#      to xcb. Any user override is respected.
#   2. Known cost of the xcb/XWayland path: while ANOTHER window is
#      fullscreen, animated views stutter until that window closes. Native
#      Wayland does not have this problem — try QT_QPA_PLATFORM=wayland
#      ./run.sh if you hit it (also printed to the terminal at launch).
#      Newer Qt/driver stacks no longer black-screen there.
#   3. Qt 6 may default to the Vulkan RHI backend on this GPU and produce a
#      black viewport; force OpenGL with QSG_RHI_BACKEND=opengl.
if [[ -n "${WAYLAND_DISPLAY:-}" && -z "${QT_QPA_PLATFORM:-}" ]]; then
    export QT_QPA_PLATFORM="xcb"
    echo "[run.sh] Wayland session: defaulting QT_QPA_PLATFORM=xcb (XWayland)." >&2
    echo "[run.sh] If animated views stutter while another window is fullscreen," >&2
    echo "[run.sh] relaunch with QT_QPA_PLATFORM=wayland." >&2
fi
export QSG_RHI_BACKEND="${QSG_RHI_BACKEND:-opengl}"

# Suppress harmless GTK/dconf warnings from Qt's GTK theme integration.
export QT_LOGGING_RULES="${QT_LOGGING_RULES:-*.warning=false}"

# ---- DL model self-check (non-blocking) ----
# DL-based algorithms (E2VID, FireNet, HyperE2VID, EvFlownet) load ONNX
# models from models/; missing models only disable those algorithms.
if [ ! -f "$REPO_ROOT/models/e2vid_lightweight.onnx" ]; then
    echo "[run.sh] DL models not configured - DL-based algorithms will be unavailable." >&2
    echo "[run.sh] Run scripts/setup_dl_models.sh to check / convert models." >&2
fi

# ---- launch ----
# Run from the repo root: the GUI's default model paths
# ("models/e2vid_lightweight.onnx", ...) are repo-relative and only resolve
# when the working directory is the repository root.
cd "$REPO_ROOT"
exec "$BIN" "$@"
