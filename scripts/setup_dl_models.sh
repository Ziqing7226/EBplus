#!/usr/bin/env bash
# setup_dl_models.sh - one-click setup of the DL models EBplus loads
# (models/*.onnx): installs the python deps, clones the rpg_e2vid
# reference code (GPL-3.0, cloned on your side by license), downloads the
# E2VID_lightweight weights and converts them. Needs: Python 3, git, curl.
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
[ "$ANY_MISSING" = "0" ] && { echo "=== All models present. ==="; exit 0; }

if [ -z "$PYTHON" ]; then
    echo "[STOP] Python 3 not found - install it, then re-run this script."
    exit 1
fi

fail() { echo "[FAIL] $1 - fix it and re-run the script."; exit 1; }

if ! "$PYTHON" -c "import torch, onnx, numpy" >/dev/null 2>&1; then
    echo "[RUN ] installing python deps: torch onnx numpy ..."
    "$PYTHON" -m pip install --quiet torch onnx numpy || fail "pip install"
fi

if [ ! -d "$REPO/ref/rpg_e2vid/model" ]; then
    echo "[RUN ] cloning rpg_e2vid reference code into ref/rpg_e2vid ..."
    git clone --depth 1 https://github.com/uzh-rpg/rpg_e2vid "$REPO/ref/rpg_e2vid" || fail "git clone"
fi

if [ ! -f "$MODELS/e2vid_lightweight.onnx" ]; then
    if [ ! -f "$MODELS/E2VID_lightweight.pth.tar" ]; then
        echo "[RUN ] downloading E2VID_lightweight.pth.tar (~41 MB) ..."
        curl -L -o "$MODELS/E2VID_lightweight.pth.tar" \
            http://rpg.ifi.uzh.ch/data/E2VID/models/E2VID_lightweight.pth.tar || fail "download"
    fi
    echo "[RUN ] converting to models/e2vid_lightweight.onnx ..."
    "$PYTHON" "$REPO/models/convert_to_onnx.py" \
        --input "$MODELS/E2VID_lightweight.pth.tar" \
        --output "$MODELS/e2vid_lightweight.onnx" || fail "convert"
fi

if [ ! -f "$MODELS/hypere2vid.onnx" ] && [ -f "$MODELS/hypere2vid.pth.tar" ]; then
    echo "[RUN ] converting hypere2vid.pth.tar ..."
    "$PYTHON" "$REPO/models/convert_hypere2vid_to_onnx.py" \
        --input "$MODELS/hypere2vid.pth.tar" \
        --output "$MODELS/hypere2vid.onnx"
fi

# The other four models have no official direct download: put their weight
# files into models/ yourself and re-run - they convert automatically.
[ -f "$MODELS/e2vid_plus.onnx" ] || if [ -f "$MODELS/reconstruction_model.pth" ]; then
    echo "[RUN ] converting reconstruction_model.pth ..."
    "$PYTHON" "$REPO/models/convert_event_cnn_minimal_to_onnx.py" --model e2vid_plus \
        --input "$MODELS/reconstruction_model.pth" --output "$MODELS/e2vid_plus.onnx"
fi
[ -f "$MODELS/firenet_plus.onnx" ] || if [ -f "$MODELS/firenet_all_cts.pth" ]; then
    echo "[RUN ] converting firenet_all_cts.pth ..."
    "$PYTHON" "$REPO/models/convert_event_cnn_minimal_to_onnx.py" --model firenet_plus \
        --input "$MODELS/firenet_all_cts.pth" --output "$MODELS/firenet_plus.onnx"
fi
[ -f "$MODELS/evflownet.onnx" ] || if [ -f "$MODELS/flow_model.pth" ]; then
    echo "[RUN ] converting flow_model.pth ..."
    "$PYTHON" "$REPO/models/convert_event_cnn_minimal_to_onnx.py" --model evflownet \
        --input "$MODELS/flow_model.pth" --output "$MODELS/evflownet.onnx"
fi

echo "=== Fully automatic: e2vid_lightweight. The other four need their"
echo "=== weight files placed in models/ manually - see wiki/Algorithms.md"
echo "=== (reconstruction_model.pth / firenet_all_cts.pth / flow_model.pth /"
echo "=== model.pth); this script converts them once present."
