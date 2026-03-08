## v0.1.6

- Replaced the screensaver eye approximation with a direct port of the actual `esp32-eyes-main` eye geometry and preset behavior, scaled for all supported displays.
- Fixed OTA Update info cards so title and value text stack correctly without overlapping.
- Restored non-edge-only swipe-back behavior while suppressing swipe-back when the touch starts on sliders, switches, or scrollable controls.
- Kept the `ESP32-S3-3248S035-N16R8` on Espressif's stock `default_16MB.csv` OTA layout for stable boot and OTA behavior.

### Firmware Assets

- `esp32-2432s024c-v0.1.6.bin`
- `esp32-3248s035-v0.1.6.bin`
- `esp32-s3-3248s035-n16r8-v0.1.6.bin`
