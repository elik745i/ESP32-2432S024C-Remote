## v0.2.12

This release updates the ESP32-S3 pin map, adds configurable vibration intensity for message notifications, and improves submenu responsiveness and swipe-back behavior on the S3 build.

### ESP32-S3 updates

- Updated the documented ESP32-S3 battery sense pin to `GPIO10`.
- Added `GPIO2` vibration motor output for `ESP32-S3-3248S035-N16R8`.
- Added persisted vibration intensity selection with `Low`, `Medium`, and `High`.
- Tuned battery sensing for the current `470K / 220K` divider used on the S3 wiring.

### UI and responsiveness

- Increased the preferred LVGL draw buffer size on the S3 target.
- Added idle-time screen warmup for common submenus on the S3 build to reduce first-open lag.
- Fixed swipe-back on submenus so it no longer shows a transient blank/white screen on the S3 build.

### Firmware binaries

- `esp32-2432s024c-v0.2.12.bin`
- `esp32-3248s035-v0.2.12.bin`
- `esp32-s3-3248s035-n16r8-v0.2.12.bin`
