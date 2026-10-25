# macOS Apple Silicon Build

This guide builds the vendored `openeb/` tree, packages `EBplus.app`, and creates a local DMG for Apple Silicon Macs.

## Scope

- Target architecture: `arm64`
- OpenEB source: `openeb/` in this repository
- OpenEB modules: `base`, `core`, `stream`, HAL, and Prophesee HAL plugins
- Output app: `build/dist/EBplus.app`
- Output DMG: `build/dist/EBplus-macos-arm64.dmg`

The current package is suitable for local testing. It is ad-hoc signed, not Developer ID signed or notarized. It includes the EBplus MIT license and the OpenEB licensing notices in `Contents/Resources/licenses`.

## Dependencies

Install Xcode command line tools and Homebrew dependencies:

```bash
xcode-select --install
brew install cmake qtbase qtsvg opencv boost libusb protobuf abseil hdf5 ripgrep
```

The packaging script reads Qt base frameworks and plugins from the `qtbase` prefix, and the Qt SVG runtime from the `qtsvg` prefix. EBplus links that runtime directly to render its activity-bar icons.

```bash
export QT_PREFIX="$(brew --prefix qtbase)"
export QTSVG_PREFIX="$(brew --prefix qtsvg)"
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
- copy EBplus and OpenEB licensing notices into the app bundle
- rewrite non-system dylib references to bundle-relative paths
- apply an ad-hoc signature
- create `build/dist/EBplus-macos-arm64.dmg`

Useful overrides:

```bash
ARCH=arm64 BUILD_TYPE=Release ./scripts/build_macos_app.sh
CREATE_DMG=OFF ./scripts/build_macos_app.sh
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

It builds the app, opens `build/dist/EBplus.app`, and checks that the `EBplus` process starts.

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
  -DCMAKE_PREFIX_PATH="$PWD/build/openeb-install;$(brew --prefix)"
cmake --build build/macos-tests --parallel
ctest --test-dir build/macos-tests --output-on-failure
```

AppleClang reports three existing header-only warnings more aggressively than the Linux toolchains. The algorithm test targets keep `-Werror` and relax only those three warning categories on macOS.

## Signing And Distribution

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
- The packaged app was validated with a CenturyArks SilkyEvCam: one device source was discovered, connected as IMX636 v4.2 at 1280x720, and streamed at 30 fps. Other vendors still require their matching HAL plugin.
