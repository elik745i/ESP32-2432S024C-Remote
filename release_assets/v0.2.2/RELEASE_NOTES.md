## v0.2.2

- Expanded `HC12 Config` from the PSRAM S3 build to all supported boards with board-specific UART and `SET` pin mappings.
- Added HC-12 support on `ESP32-3248S035` and `ESP32-2432S024C` using `RX=39`, `TX=1`, and `SET=22`.
- Relaxed runtime screen-build memory gates on the classic ESP32 boards so `Config`, `Info`, `Media`, and related screens stay accessible.
- Removed blocking UI work from the hottest touch paths by deferring HC-12 `SET` settling and screenshot capture outside LVGL click callbacks.
- Reduced loop yield latency for snappier touch, gesture, and animation response across the UI.

### Firmware Assets

- `esp32-2432s024c-v0.2.2.bin`
- `esp32-3248s035-v0.2.2.bin`
- `esp32-s3-3248s035-n16r8-v0.2.2.bin`
