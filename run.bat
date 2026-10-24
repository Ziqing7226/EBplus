@echo off
rem run.bat — launcher for EBplus on Windows.
rem
rem Copy this script to run.local.bat and customise the env vars for your
rem camera hardware. The run.local.bat copy is git-ignored so you can keep
rem your private HAL plugin path there.
rem
rem Common HAL plugin paths:
rem   Prophesee (default openEB install): C:\Program Files\Prophesee\lib\metavision\hal\plugins
rem   EBplus zip deployment:              <install dir>\metavision\hal\plugins

setlocal

rem ---- resolve repo root + binary ----
set "SCRIPT_DIR=%~dp0"
set "BIN=%SCRIPT_DIR%build\gui\gui_for_openeb.exe"

if not exist "%BIN%" (
    echo [run.bat] Binary not found at: %BIN% >&2
    echo [run.bat] Run: cmake -B build ^&^& cmake --build build --config Release >&2
    exit /b 1
)

rem ---- OpenEB runtime environment (customise for your camera) ----
if "%MV_HAL_PLUGIN_PATH%"=="" set "MV_HAL_PLUGIN_PATH=C:\Program Files\Prophesee\lib\metavision\hal\plugins"
if "%HDF5_PLUGIN_PATH%"=="" set "HDF5_PLUGIN_PATH=C:\Program Files\Prophesee\share\hdf5\plugin"

rem ---- Qt platform plugin + RHI backend ----
rem Qt 6 may default to the direct3d RHI backend and produce a black viewport
rem on some GPUs; force OpenGL (mirrors main.cpp / run.sh behaviour).
if "%QSG_RHI_BACKEND%"=="" set "QSG_RHI_BACKEND=opengl"

rem Qt DLLs may live in a vcpkg / aqtinstall layout — let the user prepend
rem extra PATH entries in run.local.bat before this line.
"%BIN%" %*
endlocal
