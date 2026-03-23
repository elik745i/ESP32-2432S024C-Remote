# Release Notes

## v0.21.11

- Restored the main-screen `Power` entry on `ESP32-2432S024C` and `ESP32-3248S035`, while keeping actual hardware shutdown available only on boards that expose the power-off signal.
- Fixed Wi-Fi scan servicing so scans complete again on the non-S3 builds, shortened the status copy to `Searching...`, and added `Scan` / `Stop` behavior with automatic cancel-on-leave.
- Kept the on-screen keyboard pinned to the bottom on compact layouts and close the Wi-Fi password popup immediately after `Save`.
- Made the `Modules` screen non-blocking and cancelable, and added bundled module catalog and package fallback so module listing and installation still work on ESP32 boards without PSRAM when GitHub HTTPS fetches fail.
- Updated firmware version metadata, module manifest versions, and release documentation for the new firmware release.