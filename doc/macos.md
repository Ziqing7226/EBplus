# macOS Apple Silicon Build

This guide builds the vendored `openeb/` tree, packages `EBplus.app`, and creates a local DMG for Apple Silicon Macs.

## Development Branches

Upstream is [Ziqing7226/EBplus__more-than-gui-for-openeb](https://github.com/Ziqing7226/EBplus__more-than-gui-for-openeb),
whose development branch is `main` (there is no `master` branch). macOS
integration and synchronization pull requests live in
[xiangtianheng/EBplus--GUI_for_openEB](https://github.com/xiangtianheng/EBplus--GUI_for_openEB)
and target the fork's `main`, which intentionally includes macOS support.
Upstream `main` remains platform-neutral. Keep upstream updates and the macOS overlay
reviewable as described in `AGENTS.md`.

## Scope

- Target architecture: `arm64`
- OpenEB source: `openeb/` in this repository
- OpenEB modules: `base`, `core`, `stream`, HAL, and Prophesee HAL plugins
- Output app: `build/dist/EBplus.app`
- Output DMG: `build/dist/EBplus-macos-arm64.dmg`

The current package is suitable for local testing. It is ad-hoc signed, not Developer ID signed or notarized. It includes the EBplus MIT license and the OpenEB licensing notices in `Contents/Resources/licenses`.

## Dependencies

Install Xcode command line tools and Homebrew dependencies. The calibration
compatibility header supports OpenCV 4 and the OpenCV 5 module split:

```bash
xcode-select --install
brew install cmake qtbase qtsvg qtserialport ceres-solver opencv boost libusb protobuf abseil gcc
```

The packaging script reads Qt base frameworks and plugins from the `qtbase` prefix, and the Qt SVG runtime from the `qtsvg` prefix. EBplus links that runtime directly to render its activity-bar icons.

```bash
export QT_PREFIX="$(brew --prefix qtbase)"
export QTSVG_PREFIX="$(brew --prefix qtsvg)"
export QTSERIALPORT_PREFIX="$(brew --prefix qtserialport)"
```

## Build The DMG

```bash
./scripts/build_macos_app.sh
```

The script will:

- configure and install vendored OpenEB into `build/openeb-install`
- configure and build `EBplus.app` in `build/macos-arm64`
- copy the required Qt frameworks plus Cocoa, SVG/image format, icon, and style plugins
- copy OpenEB runtime dylibs, HAL plugins, and HAL resources into the app bundle
- copy EBplus, OpenEB, Ceres and Eigen licensing notices into the app bundle
- rewrite non-system dylib references to bundle-relative paths
- apply an ad-hoc signature
- create `build/dist/EBplus-macos-arm64.dmg`

Useful overrides:

```bash
ARCH=arm64 BUILD_TYPE=Release ./scripts/build_macos_app.sh
CREATE_DMG=OFF ./scripts/build_macos_app.sh
CMAKE_BUILD_PARALLEL_LEVEL=3 CREATE_DMG=OFF ./scripts/build_macos_app.sh
QT_PREFIX=/opt/homebrew/opt/qtbase ./scripts/build_macos_app.sh
QTSVG_PREFIX=/opt/homebrew/opt/qtsvg ./scripts/build_macos_app.sh
```

## Extra HAL Plugins

If you have a vendor plugin such as silkyCam installed outside the vendored OpenEB build, copy it into a local test bundle during packaging:

```bash
EXTRA_HAL_PLUGIN_PATHS="/path/to/silkycam/hal/plugins" ./scripts/build_macos_app.sh
```

Keep hardware-specific bundles separate from the generic release output, for example:

```bash
BUILD_ROOT=build/silky-local \
EXTRA_HAL_PLUGIN_PATHS="/path/to/silkycam/hal/plugins" \
./scripts/build_macos_app.sh
```

This creates `build/silky-local/dist/EBplus.app` and `build/silky-local/dist/EBplus-macos-arm64.dmg` for local hardware validation.

`EXTRA_HAL_PLUGIN_PATHS` is colon-separated, so multiple plugin directories are supported. If it is not set, the script also accepts `EXTRA_HAL_PLUGIN_PATH` or `MV_HAL_PLUGIN_PATH`. The repository and CI do not bundle silkyCam or another vendor's binary plugin by default; distribute one only when its source and redistribution terms are cleared.

At runtime, the macOS app defaults `MV_HAL_PLUGIN_PATH` to its bundled plugin directory and uses OpenEB's `PLUGIN_PATH_ONLY` mode to avoid loading the same plugin twice. A user-provided `MV_HAL_PLUGIN_PATH` still takes precedence.

## Local Run

Build without recreating the DMG and launch:

```bash
CREATE_DMG=OFF ./scripts/build_macos_app.sh
open build/dist/EBplus.app
```

There is also a Codex-friendly run wrapper:

```bash
./script/build_and_run.sh
```

It builds the app and opens `build/dist/EBplus.app`. Use
`./script/build_and_run.sh --verify` to also check that the `EBplus` process starts.

## Verification

Recommended checks after packaging:

```bash
codesign --verify --deep --strict --verbose=2 build/dist/EBplus.app
hdiutil verify build/dist/EBplus-macos-arm64.dmg
```

To audit for Homebrew or `/usr/local` dylib references inside the bundle:

```bash
while IFS= read -r -d "" f; do
  if file "$f" | grep -q "Mach-O"; then
    otool -L "$f" | grep -E "^[[:space:]]*/(opt/homebrew|usr/local)/" && echo "$f"
  fi
done < <(find build/dist/EBplus.app/Contents -type f \( -perm -111 -o -name "*.dylib" \) -print0)
```

No output means the bundle has no absolute Homebrew or `/usr/local` dylib dependency references.

## Tests

Run the project CTest suite after the package build has prepared the vendored OpenEB install prefix:

```bash
brew install googletest
CREATE_DMG=OFF ./scripts/build_macos_app.sh
cmake -S . -B build/macos-tests \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_OSX_ARCHITECTURES=arm64 \
  -DGUI_FOR_OPENEB_BUILD_TESTS=ON \
  -DGUI_BUILD_TESTS=ON \
  -DCMAKE_PREFIX_PATH="$PWD/build/openeb-install;$(brew --prefix);$(brew --prefix qtbase);$(brew --prefix qtsvg);$(brew --prefix qtserialport)"
cmake --build build/macos-tests --parallel
ctest --test-dir build/macos-tests --output-on-failure
```

The algorithm test targets keep `-Werror` and relax three existing AppleClang
warning categories on macOS. All GTest targets share a configurable 60-second
discovery timeout (`EBPLUS_GTEST_DISCOVERY_TIMEOUT`).

The suite includes synthetic DAVIS/DVXplorer protocol, AEDAT4/LZ4 and playback
tests. Real-recording tests use `EBPLUS_TEST_AEDAT4` and `EBPLUS_TEST_ALPDATA`
when those recordings are available; model inference requires separately
installed ONNX weights. CTest reports unavailable fixtures as skipped. Live
DAVIS/DVXplorer USB operation still requires hardware validation on macOS.

## Signing And Distribution

The macOS CI limits compilation to three jobs per runner and cancels older
runs of the same pull request or branch. Local builds default to the host's
logical CPU count; `CMAKE_BUILD_PARALLEL_LEVEL` overrides it.

The macOS CI checks pull requests into the fork's `main` and pushes to that
branch, while retaining `develop-MacOS` triggers for existing integration work.
Successful app jobs upload `EBplus-macos-arm64-app.zip`, created with
`ditto` so the bundle retains executable permissions and symbolic links after
download. The independent test job runs the complete CTest suite.

The default package uses ad-hoc signing:

```bash
codesign --force --deep --sign - build/dist/EBplus.app
```

This is enough for local smoke testing, but Gatekeeper will still reject the app as an unidentified developer build:

```bash
spctl -a -vv build/dist/EBplus.app
```

For external distribution, replace the ad-hoc signing step with Developer ID signing, enable the hardened runtime if needed by the final entitlement set, sign the DMG, and submit it for notarization.

## Notes

- HDF5 is not a supported feature of the current macOS package because `openeb/sdk/modules/stream/cpp/3rdparty/hdf5_ecf` is not populated in this checkout. The UI may still expose HDF5 actions inherited from the cross-platform application; enable and validate the dependency before distributing a package that advertises HDF5 support.
- The July 2026 macOS build was validated with a CenturyArks SilkyEvCam: one device source was discovered, connected as IMX636 v4.2 at 1280x720, and streamed at 30 fps. This is historical hardware evidence; repeat it after upstream synchronization. Other vendors still require their matching HAL plugin.
