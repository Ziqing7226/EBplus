@echo off
setlocal EnableDelayedExpansion
rem setup_dl_models.bat - one-click check/convert for the DL models EBplus
rem loads (models/*.onnx). Reports what is present and missing, converts
rem rpg_e2vid .pth.tar weights when possible, and says exactly what to do
rem for the rest. Missing models only disable the DL-based algorithms; the
rem GUI works without them.

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
    echo === All models present. DL-based algorithms are ready. ===
    endlocal & exit /b 0
)

echo.
echo === Conversion ===
if "%PYTHON%"=="" (
    echo [SKIP] Python not found. Install Python 3, then re-run this script.
) else (
    if not exist "%MODELS%\e2vid_lightweight.onnx" (
        if exist "%MODELS%\E2VID_lightweight.pth.tar" (
            echo [RUN ] converting E2VID_lightweight.pth.tar -^> e2vid_lightweight.onnx ...
            "%PYTHON%" "%REPO%\models\convert_to_onnx.py" --input "%MODELS%\E2VID_lightweight.pth.tar" --output "%MODELS%\e2vid_lightweight.onnx"
        ) else (
            echo [SKIP] models\E2VID_lightweight.pth.tar not found - get the rpg_e2vid
            echo        weights, then re-run this script to convert them.
        )
    )
    if not exist "%MODELS%\hypere2vid.onnx" (
        if exist "%MODELS%\hypere2vid.pth.tar" (
            echo [RUN ] converting hypere2vid.pth.tar -^> hypere2vid.onnx ...
            "%PYTHON%" "%REPO%\models\convert_hypere2vid_to_onnx.py" --input "%MODELS%\hypere2vid.pth.tar" --output "%MODELS%\hypere2vid.onnx"
        ) else (
            echo [SKIP] models\hypere2vid.pth.tar not found.
        )
    )
)

echo.
echo === Summary ===
echo The GUI runs without the models; only DL-based algorithms
echo ^(E2VID, FireNet, HyperE2VID, EvFlownet^) need them. Paths can also be
echo set per-algorithm in the Algorithms panel ^(model_path parameters^).
endlocal
