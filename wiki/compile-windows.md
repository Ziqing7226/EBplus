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
| OpenCV       | vcpkg `opencv4` (recommended — always ABI-matched to your compiler); the official 4.10.x prebuilt package also works when building with VS 2022 |
| Other deps   | vcpkg (`x64-windows`): dirent, boost (program_options/timer/chrono/thread + header-only components), libusb, protobuf, hdf5, gtest |

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

4. **OpenCV** — two options:
   - **vcpkg (recommended)**: install the `opencv4` port together with the other
     dependencies in step 2 — a source build always matches the compiler ABI,
     which matters because the prebuilt archives are vc16/vc17 builds whose own
     `OpenCVConfig.cmake` rejects a newer MSVC.
   - **Official prebuilt 4.10.x** (VS 2022 only): download
     <https://github.com/opencv/opencv/releases/download/4.10.0/opencv-4.10.0-windows.exe>
     and extract it so that `<dir>\build\OpenCVConfig.cmake` exists.

## 2. vcpkg dependencies

```powershell
git clone https://github.com/microsoft/vcpkg.git D:\deps\vcpkg
D:\deps\vcpkg\bootstrap-vcpkg.bat -disableMetrics

# Boost note: only the components OpenEB needs are listed — the full `boost`
# metapackage takes hours. opencv4 supplies OpenCV (see step 4).
D:\deps\vcpkg\vcpkg.exe install --triplet x64-windows `
    dirent boost-program-options boost-timer boost-chrono boost-thread `
    boost-property-tree boost-math boost-assign boost-algorithm `
    boost-preprocessor libusb protobuf "hdf5[cpp,threadsafe,zlib]" gtest opencv4
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
    -DCMAKE_PREFIX_PATH=%CD%\build\openeb-install
cmake --build build --config Release --parallel 8

:: With the prebuilt OpenCV instead of the vcpkg port, also pass:
::   -DOpenCV_DIR=D:\deps\opencv\build
```

The binary lands at `build\gui\Release\gui_for_openeb.exe`. Launch it through
`run.bat` (sets `MV_HAL_PLUGIN_PATH`, `HDF5_PLUGIN_PATH` and the Qt RHI
backend, mirroring `run.sh` on Linux). Customise a `run.local.bat` copy for
your camera hardware — it is git-ignored.

### Running a source build without a system-wide install

When Qt / OpenCV / the OpenEB SDK are not installed system-wide, the exe also
needs their DLL directories on `PATH` (the HAL plugin path is already handled
by `run.bat`). The easiest way is a private launcher: copy `run.bat` to
`run.local.bat` (git-ignored) and prepend the DLL directories before the exe
line, e.g.

```bat
rem runtime DLLs: Qt · vendored OpenEB SDK · OpenCV · vcpkg ports
set "PATH=D:\deps\Qt.8.3\msvc2022_64in;%CD%uild\openeb-installin;<opencv>dc16in;<vcpkg>\installedd-windowsin;%PATH%"
```

Adjust the paths to your machine (`<opencv>` is the prebuilt/vcpkg OpenCV
tree, `<vcpkg>` your vcpkg root). `run.local.bat` is git-ignored, so local
absolute paths never reach the repository.

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
| `Found OpenCV Windows Pack but it has no binaries compatible with your configuration` | prebuilt archive mismatched with the compiler | build OpenCV from the vcpkg `opencv4` port, or match the archive to your VS version |
| `OpenCV_DIR-NOTFOUND` | wrong path | must point at the directory containing `OpenCVConfig.cmake` (e.g. `...\opencv\build`) |
| LNK2019 `__imp_...` on Metavision libs | GUI built against a differently-configured OpenEB | rebuild OpenEB and the GUI with the same vcpkg triplet |
| Black viewport at startup | Qt picked the direct3d RHI backend | launch via `run.bat`, or `set QSG_RHI_BACKEND=opengl` |
| Huge boost build time | installed the `boost` metapackage | install the component ports listed above instead |
