# Windows 11 Build Guide (x86_64, MSVC)

This guide builds the vendored `openeb/` SDK and the EBplus GUI on Windows 11
with the Microsoft Visual Studio toolchain — the same recipe the Windows CI
workflow (`.github/workflows/windows11.yml`) runs. OpenEB officially supports
MSVC on Windows (see `openeb/README.md`, "Compiling on Windows"); MinGW is
not supported.

## Environment

| Item         | Version                                  |
|--------------|------------------------------------------|
| OS           | Windows 11 x86_64                        |
| Compiler     | MSVC (Visual Studio 2022 Build Tools, VCTools workload) |
| CMake        | 4.x (minimum 3.16)                       |
| Qt           | 6.8.x (`win64_msvc2022_64`, `qtbase` + `qtsvg`) |
| OpenCV       | 4.10.x (official prebuilt package)       |
| Other deps   | vcpkg (`x64-windows`): dirent, boost (program_options/timer/chrono/thread), libusb, protobuf, hdf5, gtest |

Long paths must be enabled (OpenEB requirement): `gpedit.msc` → Computer
Configuration → Administrative Templates → System → Filesystem → *Enable
Win32 long paths*.

## 1. Toolchain

1. **Visual Studio 2022 Build Tools** with the "Desktop development with C++"
   workload (includes the Windows SDK).
2. **CMake** and (optionally) **Ninja**.
3. **Qt 6** via [aqtinstall](https://github.com/miurahr/aqtinstall):

   ```powershell
   pip install aqtinstall
   aqt install-qt windows desktop 6.8.3 win64_msvc2022_64 -O D:\deps\Qt --archives qtbase qtsvg
   ```

4. **OpenCV** — download the official prebuilt package
   <https://github.com/opencv/opencv/releases/download/4.10.0/opencv-4.10.0-windows.exe>
   and extract it (self-extracting archive) so that
   `<dir>\build\OpenCVConfig.cmake` exists.

## 2. vcpkg dependencies

```powershell
git clone https://github.com/microsoft/vcpkg.git D:\deps\vcpkg
D:\deps\vcpkg\bootstrap-vcpkg.bat -disableMetrics

# Boost note: only the components OpenEB needs (program_options, timer,
# chrono, thread) are listed — the full `boost` metapackage takes hours.
D:\deps\vcpkg\vcpkg.exe install --triplet x64-windows `
    dirent boost-program-options boost-timer boost-chrono boost-thread `
    libusb protobuf "hdf5[cpp,threadsafe,zlib]" gtest
```

## 3. Build vendored OpenEB

```powershell
cd <repo root>
cmake -S openeb -B build\openeb -A x64 `
    -DCMAKE_TOOLCHAIN_FILE=openeb/cmake/toolchains/vcpkg.cmake `
    -DVCPKG_DIRECTORY=D:\deps\vcpkg `
    -DCMAKE_BUILD_TYPE=Release `
    -DBUILD_TESTING=OFF -DBUILD_SAMPLES=OFF -DBUILD_APPS=OFF `
    -DCOMPILE_PYTHON3_BINDINGS=OFF `
    -DCMAKE_INSTALL_PREFIX=%CD%\build\openeb-install
cmake --build build\openeb --config Release --target install --parallel 8
```

## 4. Build the GUI

```powershell
cmake -S . -B build -A x64 `
    -DCMAKE_TOOLCHAIN_FILE=D:\deps\vcpkg\scripts\buildsystems\vcpkg.cmake `
    -DVCPKG_TARGET_TRIPLET=x64-windows `
    -DOpenCV_DIR=D:\deps\opencv\build `
    -DCMAKE_PREFIX_PATH=%CD%\build\openeb-install
cmake --build build --config Release --parallel 8
```

The binary lands at `build\gui\Release\gui_for_openeb.exe`. Launch it through
`run.bat` (sets `MV_HAL_PLUGIN_PATH`, `HDF5_PLUGIN_PATH` and the Qt RHI
backend, mirroring `run.sh` on Linux). Customise a `run.local.bat` copy for
your camera hardware — it is git-ignored.

## 5. Tests

```powershell
ctest --test-dir build -C Release --output-on-failure
```

Configure with `-DGUI_BUILD_TESTS=ON -DGUI_FOR_OPENEB_BUILD_TESTS=ON`
(both are ON by default) and make sure the `gtest` vcpkg port is installed.

## SilkyEvCam HAL plugin (optional, separate repo)

The closed-source CenturyArks firmware plugin lives in a separate repository
and is overlaid onto the vendored OpenEB tree before configuring it — never
committed into this repository:

```powershell
Copy-Item <test-repo>\src\silky_common.cpp      openeb\hal_psee_plugins\src\plugin\
Copy-Item <test-repo>\src\CMakeLists.plugin.txt openeb\hal_psee_plugins\src\plugin\CMakeLists.txt
Copy-Item <test-repo>\src\CMakeLists.lib.txt    openeb\hal_psee_plugins\lib\CMakeLists.txt
```

Then build as above; the `hal_plugin_silky_common` target is built and
installed together with the other HAL plugins.

## Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| `Unable to find a valid Visual Studio instance` (vcpkg) | Build Tools missing or not on PATH | Install the VCTools workload; run from a VS dev shell |
| `ssize_t` undeclared in `gui/davis/*` | POSIX type, not available on MSVC | Use `libusb_ssize_t` (fixed in this branch); report if it reappears |
| `OpenCV_DIR-NOTFOUND` | wrong path | must point at the directory containing `OpenCVConfig.cmake` (e.g. `...\opencv\build`) |
| LNK2019 `__imp_...` on Metavision libs | GUI built against a differently-configured OpenEB | rebuild OpenEB and the GUI with the same vcpkg triplet |
| Black viewport at startup | Qt picked the direct3d RHI backend | launch via `run.bat`, or `set QSG_RHI_BACKEND=opengl` |
| Huge boost build time | installed the `boost` metapackage | install the four component ports listed above instead |
