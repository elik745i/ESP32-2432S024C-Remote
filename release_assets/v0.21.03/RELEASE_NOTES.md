# v0.21.03

## Highlights

- Improved UI responsiveness by giving touch, swipe, and animation phases higher scheduler priority
- Black button theme now shows a clear green active state for ON and pressed controls
- Radio Info now lazy-loads from cached values first instead of blocking the whole UI on entry
- Radio Config changes for channel, baud, mode, power, and E220 transfer mode now apply through a deferred queue, reducing missed taps and lag
- Discovery OFF now stops background radio discovery traffic and probe replies, and the discovery label is translated across supported UI languages
- Increased LVGL draw-buffer sizing on PSRAM-capable S3 builds for smoother rendering

## Included release assets

### ESP32-2432S024C

- `esp32-2432s024c-v0.21.03_bootloader.bin` at `0x1000`
- `esp32-2432s024c-v0.21.03_partitions.bin` at `0x8000`
- `esp32-2432s024c-v0.21.03_boot_app0.bin` at `0xE000`
- `esp32-2432s024c-v0.21.03.bin` at `0x10000`

### ESP32-3248S035

- `esp32-3248s035-v0.21.03_bootloader.bin` at `0x1000`
- `esp32-3248s035-v0.21.03_partitions.bin` at `0x8000`
- `esp32-3248s035-v0.21.03_boot_app0.bin` at `0xE000`
- `esp32-3248s035-v0.21.03.bin` at `0x10000`

### ESP32-S3-3248S035-N16R8

- `esp32-s3-3248s035-n16r8-v0.21.03_bootloader.bin` at `0x0000`
- `esp32-s3-3248s035-n16r8-v0.21.03_partitions.bin` at `0x8000`
- `esp32-s3-3248s035-n16r8-v0.21.03_boot_app0.bin` at `0xE000`
- `esp32-s3-3248s035-n16r8-v0.21.03.bin` at `0x10000`

## Notes

- `ESP32-2432S024C` and `ESP32-3248S035` should not be flashed with only the main firmware image when using Espressif flash tools
- `pio run -t upload` still handles the correct image layout automatically when flashing from source
