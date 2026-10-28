# Camera Setup

EBplus talks to event cameras through the HAL plugins built from the vendored
OpenEB. Two camera families need nothing beyond the OS seeing the device
correctly; one (SilkyEvCam) needs a one-time USB driver installation on
Windows, provided by the camera vendor.

## SilkyEvCam (CenturyArks) — Windows: one-time driver install

SilkyEvCam cameras enumerate as a vendor-specific USB device
(`VID_31F7`). Windows only exposes them to applications once the vendor's
WinUSB driver is bound to the device. **The driver is provided by
CenturyArks and is not distributed with EBplus.**

1. Download the SilkyEvCam installer from the CenturyArks website
   (centuryarks.com → Download).
2. Run the installer and follow its steps — it installs the USB driver.
   Note: some plugin-only installers (e.g. v5.2.0) install just the HAL
   plugin and do **not** include the USB driver; use an installer that
   includes the driver installation (v3.0.x does).
3. Verify in Device Manager: the camera must appear without a warning icon
   (Properties → Status OK, no "Code 28").
4. Verify from EBplus: `build\benchmarks\Release\bench_camera.exe list`
   must print the camera, and the GUI's device list shows it.

If the HAL plugin for the camera is needed separately, it is available from
the vendor's installer or can be built from the vendor's published source —
EBplus does not ship a custom build of it.

## SilkyEvCam (CenturyArks) — Linux: udev rule

No kernel driver is needed; libusb talks to the device directly. Grant user
access with a udev rule:

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="31f7", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/99-centuryarks.rules
sudo udevadm control --reload-rules
# unplug and replug the camera
```

Verify with `lsusb | grep 31f7`.

## SilkyEvCam (CenturyArks) — macOS

No driver installation is required.

## Prophesee / iniVation

Both vendors' standard installers handle their own drivers and runtime
components; follow the respective vendor documentation. On Linux, install
the udev rules from the vendor package (the vendored OpenEB tree also ships
rules under `openeb/hal_psee_plugins/utils/`).
