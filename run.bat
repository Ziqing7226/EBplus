@echo off
rem run.bat — launcher for EBplus on Windows.
rem
rem Copy this script to run.local.bat and customise the env vars for your
rem camera hardware. The run.local.bat copy is git-ignored so you can keep
rem your private HAL plugin path there.

setlocal

rem ---- resolve repo root + binary ----
set "SCRIPT_DIR=%~dp0"
set "BIN=%SCRIPT_DIR%build\gui\gui_for_openeb.exe"

if not exist "%BIN%" (
    echo [run.bat] Binary not found at: %BIN% >&2
    echo [run.bat] Run: cmake -B build ^&^& cmake --build build --config Release >&2
    exit /b 1
)

rem ---- OpenEB runtime environment ----
rem The vendored OpenEB build output IS the runtime: point the HAL plugin
rem and HDF5 ECF paths at it by default (no system-wide Prophesee install
rem required) and put its DLL directory on PATH so metavision_sdk_*.dll /
rem hdf5*.dll resolve. Override both in run.local.bat if you deliberately
rem use a system-wide Prophesee installation.
if "%MV_HAL_PLUGIN_PATH%"=="" set "MV_HAL_PLUGIN_PATH=%SCRIPT_DIR%build\openeb-install\lib\metavision\hal\plugins"
if "%HDF5_PLUGIN_PATH%"=="" set "HDF5_PLUGIN_PATH=%SCRIPT_DIR%build\openeb-install\lib\hdf5\plugin"
set "PATH=%SCRIPT_DIR%build\openeb-install\bin;%PATH%"

rem ---- Qt platform plugin + RHI backend ----
rem Qt 6 may default to the direct3d RHI backend and produce a black viewport
rem on some GPUs; force OpenGL (mirrors main.cpp / run.sh behaviour).
if "%QSG_RHI_BACKEND%"=="" set "QSG_RHI_BACKEND=opengl"

rem Qt DLLs may live in a vcpkg / aqtinstall layout — let the user prepend
rem extra PATH entries in run.local.bat before this line.
"%BIN%" %*
endlocal
