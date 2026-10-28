@echo off
setlocal
rem setup_dl_models.bat - one-click setup of the DL models EBplus loads
rem (models/*.onnx): installs the python deps, clones the rpg_e2vid
rem reference code (GPL-3.0, cloned on your side by license), downloads the
rem E2VID_lightweight weights and converts them. Needs: Python 3, git, curl
rem (all standard on updated Windows / macOS / Linux).

set "SCRIPT_DIR=%~dp0"
set "REPO=%SCRIPT_DIR%.."
set "MODELS=%REPO%\models"

set "PYTHON=python"
where python >nul 2>&1 || set "PYTHON="
if exist "%REPO%\.venv\Scripts\python.exe" set "PYTHON=%REPO%\.venv\Scripts\python.exe"

echo === EBplus DL model check ===
set ANY_MISSING=0
for %%M in (e2vid_lightweight e2vid_plus firenet_plus hypere2vid evflownet) do (
    if exist "%MODELS%\%%M.onnx" (
        echo [OK]      models\%%M.onnx
    ) else (
        echo [MISSING] models\%%M.onnx
        set ANY_MISSING=1
    )
)
if "%ANY_MISSING%"=="0" (
    echo === All models present. ===
    endlocal & exit /b 0
)

if "%PYTHON%"=="" (
    echo [STOP] Python 3 not found - install it, then re-run this script.
    endlocal & exit /b 1
)

"%PYTHON%" -c "import torch, onnx, numpy" >nul 2>&1
if errorlevel 1 (
    echo [RUN ] installing python deps: torch onnx numpy ...
    "%PYTHON%" -m pip install --quiet torch onnx numpy || goto :fail
)

if not exist "%REPO%\ref\rpg_e2vid\model" (
    echo [RUN ] cloning rpg_e2vid reference code into ref\rpg_e2vid ...
    git clone --depth 1 https://github.com/uzh-rpg/rpg_e2vid "%REPO%\ref\rpg_e2vid" || goto :fail
)

if not exist "%MODELS%\e2vid_lightweight.onnx" (
    if not exist "%MODELS%\E2VID_lightweight.pth.tar" (
        echo [RUN ] downloading E2VID_lightweight.pth.tar, about 41 MB ...
        curl -L -o "%MODELS%\E2VID_lightweight.pth.tar" http://rpg.ifi.uzh.ch/data/E2VID/models/E2VID_lightweight.pth.tar || goto :fail
    )
    echo [RUN ] converting to models\e2vid_lightweight.onnx ...
    "%PYTHON%" "%REPO%\models\convert_to_onnx.py" --input "%MODELS%\E2VID_lightweight.pth.tar" --output "%MODELS%\e2vid_lightweight.onnx" || goto :fail
)

if not exist "%MODELS%\hypere2vid.onnx" (
    if exist "%MODELS%\hypere2vid.pth.tar" (
        echo [RUN ] converting hypere2vid.pth.tar ...
        "%PYTHON%" "%REPO%\models\convert_hypere2vid_to_onnx.py" --input "%MODELS%\hypere2vid.pth.tar" --output "%MODELS%\hypere2vid.onnx"
    )
)

rem The other four models have no official direct download: put their weight
rem files into models\ yourself and re-run - they convert automatically.
if not exist "%MODELS%\e2vid_plus.onnx" (
    if exist "%MODELS%\reconstruction_model.pth" (
        echo [RUN ] converting reconstruction_model.pth ...
        "%PYTHON%" "%REPO%\models\convert_event_cnn_minimal_to_onnx.py" --model e2vid_plus --input "%MODELS%\reconstruction_model.pth" --output "%MODELS%\e2vid_plus.onnx"
    )
)
if not exist "%MODELS%\firenet_plus.onnx" (
    if exist "%MODELS%\firenet_all_cts.pth" (
        echo [RUN ] converting firenet_all_cts.pth ...
        "%PYTHON%" "%REPO%\models\convert_event_cnn_minimal_to_onnx.py" --model firenet_plus --input "%MODELS%\firenet_all_cts.pth" --output "%MODELS%\firenet_plus.onnx"
    )
)
if not exist "%MODELS%\evflownet.onnx" (
    if exist "%MODELS%\flow_model.pth" (
        echo [RUN ] converting flow_model.pth ...
        "%PYTHON%" "%REPO%\models\convert_event_cnn_minimal_to_onnx.py" --model evflownet --input "%MODELS%\flow_model.pth" --output "%MODELS%\evflownet.onnx"
    )
)

echo === Fully automatic: e2vid_lightweight. The other four need their
echo === weight files placed in models\ manually - see wiki/Algorithms.md
echo === ^(reconstruction_model.pth / firenet_all_cts.pth / flow_model.pth /
echo === model.pth^); this script converts them once present.
endlocal & exit /b 0

:fail
echo [FAIL] a step above failed - fix it and re-run the script.
endlocal & exit /b 1
