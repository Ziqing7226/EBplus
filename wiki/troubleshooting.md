# Troubleshooting — environment, cameras and DL models

Deployment pitfalls collected from real installs (3.2.0). Read the section
that matches your symptom; each section states the check first, the fix
second. See also [compile.md](compile.md), [compile-windows.md](compile-windows.md)
and [performance.md](performance.md).

---

## 1. Windows: camera not detected at all

**Check first** (Device Manager or PowerShell):

```powershell
Get-PnpDevice -PresentOnly | Where-Object {$_.InstanceId -match 'VID_31F7|VID_03FD|VID_04B4|VID_1FC9|VID_152A'}
```

| Symptom | Meaning |
|---|---|
| device listed with `Status: Error, CM_PROB_FAILED_INSTALL` | **no driver bound** — a WinUSB-class driver must be installed before ANY software (libusb) can see the device |
| device not listed at all | cable/port issue — try another port; a driver installed on one port does NOT carry to another port |
| device OK but EBplus shows nothing | plugin path — see section 2 |

### 1.1 Driver binding (the usual culprit)

EBplus talks to Prophesee-family cameras (Prophesee **and** CenturyArks
SilkyEvCam — one plugin covers both since 3.2.0) through **libusb, which on
Windows only enumerates devices bound to WinUSB / libusbK / libusb0**.
A freshly plugged camera with no driver shows exactly
`CM_PROB_FAILED_INSTALL`.

Options, in order of preference:

1. **Vendor driver installer.** The current CenturyArks plugin installer
   (V5.2.0 on the [download page](https://centuryarks.com/en/download/))
   does NOT contain a driver — it expects the binding to exist. Older
   installers shipped `wdi-simple.exe` (the libwdi command-line tool) and
   installed the binding themselves. Contact CenturyArks for their current
   Windows driver if in doubt.
2. **Zadig** (https://zadig.akeo.ie): Options → List All Devices → select
   the camera → driver `WinUSB` → Install Driver. 30 seconds, per port.
3. **A prepared driver package** (advanced): a minimal INF binding the
   inbox `WinUSB.sys` plus a catalog. Note Windows rejects *unsigned*
   packages ("third-party INF does not contain digital signature
   information"), and a self-signed catalog additionally requires the
   certificate in `LocalMachine\TrustedPublisher` **and** `LocalMachine\Root`.

### 1.2 The binding still fails even with the package staged

Real-world case (3.2.0 bring-up): the driver package was staged
successfully (`pnputil /add-driver` accepted it, `oemNNN.inf` published)
while the device kept reporting `CM_PROB_FAILED_INSTALL`, and
`UpdateDriverForPlugAndPlayDevices` returned "no more items". The
`setupapi.dev.log` (`C:\Windows\INF\setupapi.dev.log`) showed
`{Select Best Driver}` failing with `0xE0000228 (There are no compatible
drivers for this device)` on a Build Driver List that yielded **zero
candidates** despite identical hardware IDs.

Root cause of that case: the INF had been generated with **Unix (LF) line
endings**. Converting the INF to CRLF, rebuilding the catalog
(`makecat`), re-signing it and re-importing fixed the match. Lesson: the
Windows INF parser silently yields an empty model list for LF-only files
in decorated sections - always ship INFs with CRLF.

Note on signature failures during staging: `0x800b0109
(CERT_E_UNTRUSTEDROOT)` on an INF verify inside the store-import flow
appears when the package signature is Authenticode-only (self-signed
certificate); the store import can still succeed while device matching
stays broken. Read the FULL staged-package verification context in
`setupapi.dev.log` before concluding anything about signature policy.

### 1.3 Do NOT install two HAL plugins covering the same cameras

Two plugin DLLs that both register the same USB IDs (e.g. the EBplus stock
plugin **and** a vendor `hal_plugin_silky_common.dll`) will both enumerate
and open/claim every matching device — symptoms: duplicate devices, flaky
`Camera is busy`. Keep ONE plugin per HAL plugin directory.

### 1.4 iniVation cameras do not use the HAL plugin path

DAVIS/DVXplorer devices are served by EBplus' built-in libusb parsers
(`GUI_HAVE_DAVIS`). They will never appear through
`Camera::list_online_sources()`; the Devices panel shows them via the
direct libusb path. Their driver comes from the iniVation driver package.

---

## 2. HAL plugin not loading (all cameras "not detected", RAW discovery dead)

**Check**: with the GUI started from the same shell you plan to use,
`echo %MV_HAL_PLUGIN_PATH%` (Windows) / `echo $MV_HAL_PLUGIN_PATH` (Linux),
then confirm the directory exists and contains the plugin DLL/.so.

- `run.bat` / `run.sh` default the path to the **vendored build output**
  (`build/openeb-install/lib/metavision/hal/plugins`) since 3.2.0. Override
  in `run.local.bat` / your shell only if you deliberately use a
  system-wide Metavision install.
- **Windows 11 gotcha**: when `MV_HAL_PLUGIN_PATH` is unset/invalid, the
  HAL falls back to a **registry key only** (`HKLM\Software\Prophesee`)
  — there is no DLL-relative fallback. With a source build and no system
  install, an unset variable means zero plugins load.
- **Old `run.local.bat` copies** made before 3.2.0 predate the vendored
  defaults and the `build\openeb-install\bin` PATH prepend — re-copy it
  from the current `run.bat` or the exe may resolve stale DLLs.
- Linux: `run.sh` exports the same defaults; a missing `HDF5_PLUGIN_PATH`
  breaks .hdf5 reading, see section 3.

---

## 3. HDF5 files will not open

**Check 1 — what does the error say?**
`"HDF5 is not available"` = the build has no HDF5 (SDK built without
HDF5). `"Could not open file"` = reader ran but the file/plugin layer
failed.

**Check 2 — ECF codec.** Metavision event files are ECF-compressed.
READING uses `hdf5_ecf_codec.dll`/`.so` linked into the SDK (no plugin
path needed); **WRITING** (File Tools → Convert to HDF5) and third-party
tools (h5py) go through the HDF5 filter pipeline and need
`HDF5_PLUGIN_PATH` pointing at `build/openeb-install/lib/hdf5/plugin`
(`H5Zecf.dll` + the codec).

**Check 3 — DLL resolution (Windows).** The exe must resolve
`metavision_sdk_*.dll`, `hdf5.dll`, `hdf5_ecf_codec.dll`. Source builds:
always launch through `run.bat` (it prepends `build/openeb-install\bin`
to PATH), or verify with Process Explorer which DLLs the process loaded.
Stale DLLs from an old install masquerade as "file won't open".

**Check 4 — non-ASCII paths.** The reader passes narrow-byte paths to
`H5::H5File`; HDF5 2.x interprets them as UTF-8. Files under Chinese/
Japanese-named directories may fail — move to an ASCII path.

---

## 4. Deep-learning models (E2VID / FireNet / HyperE2VID)

The reconstruction algorithms E2VID, E2VID+, FireNet+, HyperE2VID run
neural inference on `.onnx` models. Three things must line up: the ONNX
Runtime library (build time), the model file (runtime), and the device
policy (panel).

### 4.1 Build time: ONNX Runtime is OPTIONAL

Without it the build succeeds but DL modes fall back to the heuristic
reconstruction (and the panel says so). The build looks for
`onnxruntime_cxx_api.h` + `onnxruntime` lib in `third_party/onnxruntime/`
and the standard system paths (see `algo/CMakeLists.txt`). Drop a prebuilt
CPU package there and reconfigure. **Intel GPUs**: OpenVINO is picked up
the same way (`third_party/openvino/runtime`); both runtimes consume the
same `.onnx` models.

### 4.2 Model files

- Get `.onnx` models from the rpg_e2vid release, or convert `.pth.tar`
  weights yourself: `models/convert_to_onnx.py` (needs a Python venv with
  torch CPU + onnx; exports dynamic H/W and the ConvLTM recurrent states
  in the layout the C++ side expects).
- Set the path per algorithm in the Algorithms panel:
  `model_path` (E2VID), `e2vid_plus_model_path`, `firenet_plus_model_path`,
  `hypere2vid_model_path`. Values persist in configs (ConfigManager).
- A wrong-architecture model fails at session load with a runtime error —
  match model family to algorithm.

### 4.3 Device policy and threads

- `e2vid_device`: `0=Auto, 1=CPU, 2=GPU`. **Auto means GPU only if a GPU
  provider is actually available** (CUDA for ONNX Runtime builds, Intel
  plugin for OpenVINO builds); a CPU-only ONNX Runtime silently runs on
  CPU — that is expected, not a bug.
- DL inference shares the machine with rendering and the SDK decode
  threads; `cv::setNumThreads` interacts with it. If the GUI stutters
  with DL enabled, prefer the CPU device with fewer threads over a GPU
  provider that swaps to system memory.
- Configs: device and model paths are restored from configs, but the
  runtime library must exist in the *deployed* environment — a config
  from a CUDA machine moves to a CPU-only machine and Auto falls back.

### 4.4 Windows DLL notes

`onnxruntime.dll` must be reachable at runtime (PATH or next to the exe);
a build-time link success does not copy the DLL. On Linux the build sets
RPATH, so no `LD_LIBRARY_PATH` edit is needed for ONNX Runtime itself.

---

## 5. Linux: udev permissions

Camera enumeration needs read/write access to the USB device node
(`/dev/bus/usb/...`). Install the rules from the vendored OpenEB
(`openeb/hal_psee_plugins/resources/rules/`) per the compile guide, or add
your user to `plugdev`. After editing rules:
`sudo udevadm control --reload && sudo udevadm trigger`, then re-plug.
CenturyArks (VID 31F7) is NOT covered by the shipped Cypress/EVK2 rules —
add the same `MODE="666"` line for `idVendor=="31f7"` if permissions block
enumeration.

---

## 6. Quick self-test probes

`benchmarks/` (build with `-DEBPLUS_BUILD_BENCHMARKS=ON`) contains
camera-free reproducers for most of the above:

```bash
bench_playback time <file.raw|hdf5>   # decode path, names the failing file
bench_index time <file.raw>           # index build + duration
bench_camera list                     # live enumeration through the plugins
bench_camera open <index>             # 2 s live streaming probe
```

If `bench_camera list` finds nothing while Device Manager is happy, the
plugin path (section 2) is wrong; if it finds the camera but the GUI does
not, check for dual plugins (section 1.3).
