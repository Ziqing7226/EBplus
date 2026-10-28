@echo off
rem run.bat - the single launcher for EBplus on Windows.
rem
rem Works from a fresh clone + local build: it auto-discovers the binary and
rem every runtime DLL directory (vendored SDK, Qt, OpenCV, vcpkg ports).
rem Customisation: copy to run.local.bat (git-ignored) and edit the PATH
rem lines there if a dependency lives outside the layouts probed below.

setlocal

rem ---- resolve repo root + binary (single- and multi-config layouts) ----
set "SCRIPT_DIR=%~dp0"
set "BIN=%SCRIPT_DIR%build\gui\gui_for_openeb.exe"
if not exist "%BIN%" set "BIN=%SCRIPT_DIR%build\gui\Release\gui_for_openeb.exe"

if not exist "%BIN%" (
    echo [run.bat] Binary not found at: %BIN% >&2
    echo [run.bat] Run: cmake -B build ^&^& cmake --build build --config Release >&2
    exit /b 1
)

rem ---- OpenEB runtime environment ----
rem The vendored OpenEB build output IS the runtime: point the HAL plugin
rem and HDF5 ECF paths at it by default (no system-wide Prophesee install
rem required). Override both in run.local.bat if you deliberately use a
rem system-wide Prophesee installation.
if "%MV_HAL_PLUGIN_PATH%"=="" set "MV_HAL_PLUGIN_PATH=%SCRIPT_DIR%build\openeb-install\lib\metavision\hal\plugins"
if "%HDF5_PLUGIN_PATH%"=="" set "HDF5_PLUGIN_PATH=%SCRIPT_DIR%build\openeb-install\lib\hdf5\plugin"
set "PATH=%SCRIPT_DIR%build\openeb-install\bin;%PATH%"

rem ---- runtime DLL directories ----
rem The GUI imports Qt, OpenCV and the vcpkg runtime DLLs (hdf5, boost,
rem libusb, ...) directly; the vendored bin holds only the SDK DLLs, so every
rem dependency directory must be on PATH. Probe most-specific first, skip
rem silently when absent (fresh clone without the local deps layout).
rem 1. Local dev-dependency tree next to the repo (deps\, vcpkg\).
if exist "%SCRIPT_DIR%..\deps\Qt" (
    for /d %%D in ("%SCRIPT_DIR%..\deps\Qt\6.*") do if exist "%%D\msvc2022_64\bin" set "PATH=%%D\msvc2022_64\bin;%PATH%"
)
if exist "%SCRIPT_DIR%..\deps\opencv\build\x64\vc16\bin" set "PATH=%SCRIPT_DIR%..\deps\opencv\build\x64\vc16\bin;%PATH%"
if exist "%SCRIPT_DIR%..\vcpkg\installed\x64-windows\bin" set "PATH=%SCRIPT_DIR%..\vcpkg\installed\x64-windows\bin;%PATH%"
rem 2. Standard open-source install layouts: aqtinstall Qt, system vcpkg.
for /d %%D in ("%LOCALAPPDATA%\Qt\6.*") do if exist "%%D\msvc2022_64\bin" set "PATH=%%D\msvc2022_64\bin;%PATH%"
for /d %%D in ("C:\Qt\6.*") do if exist "%%D\msvc2022_64\bin" set "PATH=%%D\msvc2022_64\bin;%PATH%"
if defined VCPKG_ROOT if exist "%VCPKG_ROOT%\installed\x64-windows\bin" set "PATH=%VCPKG_ROOT%\installed\x64-windows\bin;%PATH%"

rem ---- Qt platform plugin + RHI backend ----
rem Qt 6 may default to the direct3d RHI backend and produce a black viewport
rem on some GPUs; force OpenGL (mirrors main.cpp / run.sh behaviour).
if "%QSG_RHI_BACKEND%"=="" set "QSG_RHI_BACKEND=opengl"

"%BIN%" %*
endlocal
