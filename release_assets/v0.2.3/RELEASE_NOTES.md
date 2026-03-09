## v0.2.3

- Improved UI responsiveness by removing blocking work from several touch callback paths.
- Deferred HC-12 `SET` settling and screenshot capture so taps and gestures stay responsive.
- Reduced loop yield latency for smoother LVGL animation and faster touch reaction.
- Kept the HC-12 config available across all supported boards with board-specific wiring.
- Added the HC-12 wiring image to the README so the hardware setup is documented on GitHub.

### Firmware Assets

- `esp32-2432s024c-v0.2.3.bin`
- `esp32-3248s035-v0.2.3.bin`
- `esp32-s3-3248s035-n16r8-v0.2.3.bin`
