# Camera Setup

## SilkyEvCam (CenturyArks)

- **Windows**: the camera needs the vendor's USB driver — download an
  installer that includes it from the
  [CenturyArks website](https://www.centuryarks.com/en/download/)
  (v3.0.x does; plugin-only installers such as v5.2.0 do not).
- **Plugin**: download the HAL plugin from the CenturyArks website, or
  build it from the plugin source published there.
- **Linux**: no driver; grant USB access with
  `echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="31f7", MODE="0666"' | sudo tee /etc/udev/rules.d/99-centuryarks.rules && sudo udevadm control --reload-rules`
- **macOS**: no driver.

## Prophesee / iniVation

Use the vendors' standard installers.
