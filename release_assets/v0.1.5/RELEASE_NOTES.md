## v0.1.5

- Fixed swipe-back behavior so it works from buttons and empty space again, while sliders, switches, and scrollable areas keep their own drag gestures.
- Fixed OTA Update info cards so title/value text no longer overlap.
- Switched the `ESP32-S3-3248S035-N16R8` build to Espressif's stock `default_16MB.csv` OTA partition layout for safer boot behavior.
- Kept the 4 MB ESP32 boards on the existing non-OTA flash layout and continued to gate unsupported firmware OTA paths clearly in the UI.

### Firmware Assets

- `esp32-2432s024c-v0.1.5.bin`
- `esp32-3248s035-v0.1.5.bin`
- `esp32-s3-3248s035-n16r8-v0.1.5.bin`
