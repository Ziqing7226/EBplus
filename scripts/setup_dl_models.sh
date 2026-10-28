#!/usr/bin/env bash
# setup_dl_models.sh - one-click check/convert for the DL models EBplus
# loads (models/*.onnx). Reports what is present and missing and converts
# rpg_e2vid .pth.tar weights when possible. Missing models only disable the
# DL-based algorithms; the GUI works without them.
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$SCRIPT_DIR")"
MODELS="$REPO/models"

PYTHON="python3"
command -v python3 >/dev/null 2>&1 || PYTHON=""
[ -x "$REPO/.venv/bin/python" ] && PYTHON="$REPO/.venv/bin/python"

echo "=== EBplus DL model check ==="

ANY_MISSING=0
for m in e2vid_lightweight e2vid_plus firenet_plus hypere2vid evflownet; do
    if [ -f "$MODELS/$m.onnx" ]; then
        echo "[OK]      models/$m.onnx"
    else
        echo "[MISSING] models/$m.onnx"
        ANY_MISSING=1
    fi
done

if [ "$ANY_MISSING" = "0" ]; then
    echo "=== All models present. DL-based algorithms are ready. ==="
    exit 0
fi

echo
echo "=== Conversion ==="
if [ -z "$PYTHON" ]; then
    echo "[SKIP] Python 3 not found. Install Python 3, then re-run this script."
else
    if [ ! -f "$MODELS/e2vid_lightweight.onnx" ]; then
        if [ -f "$MODELS/E2VID_lightweight.pth.tar" ]; then
            echo "[RUN ] converting E2VID_lightweight.pth.tar -> e2vid_lightweight.onnx ..."
            "$PYTHON" "$REPO/models/convert_to_onnx.py" \
                --input "$MODELS/E2VID_lightweight.pth.tar" \
                --output "$MODELS/e2vid_lightweight.onnx"
        else
            echo "[SKIP] models/E2VID_lightweight.pth.tar not found - get the"
            echo "       rpg_e2vid weights, then re-run this script to convert them."
        fi
    fi
    if [ ! -f "$MODELS/hypere2vid.onnx" ]; then
        if [ -f "$MODELS/hypere2vid.pth.tar" ]; then
            echo "[RUN ] converting hypere2vid.pth.tar -> hypere2vid.onnx ..."
            "$PYTHON" "$REPO/models/convert_hypere2vid_to_onnx.py" \
                --input "$MODELS/hypere2vid.pth.tar" \
                --output "$MODELS/hypere2vid.onnx"
        else
            echo "[SKIP] models/hypere2vid.pth.tar not found."
        fi
    fi
fi

echo
echo "=== Summary ==="
echo "The GUI runs without the models; only DL-based algorithms"
echo "(E2VID, FireNet, HyperE2VID, EvFlownet) need them. Paths can also be"
echo "set per-algorithm in the Algorithms panel (model_path parameters)."
