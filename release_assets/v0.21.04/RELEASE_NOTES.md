# v0.21.04

## Highlights

- Fixed horizontal swipe-back on `Config -> Style -> Main Screen Items`
- Repaired multilingual UI text encoding and regenerated the bundled LVGL font assets
- Expanded font generation to cover the missing `0x3000-0x30FF` punctuation and kana range
- Added radio terminal startup diagnostics for the active HC-12 / E220 pin mapping
- Enabled menu icon mode by default for first boot and reduced touch tick beep intensity

## Included release assets

### ESP32-2432S024C

- `esp32-2432s024c-v0.21.04_bootloader.bin` at `0x1000`
- `esp32-2432s024c-v0.21.04_partitions.bin` at `0x8000`
- `esp32-2432s024c-v0.21.04_boot_app0.bin` at `0xE000`
- `esp32-2432s024c-v0.21.04.bin` at `0x10000`

### ESP32-3248S035

- `esp32-3248s035-v0.21.04_bootloader.bin` at `0x1000`
- `esp32-3248s035-v0.21.04_partitions.bin` at `0x8000`
- `esp32-3248s035-v0.21.04_boot_app0.bin` at `0xE000`
- `esp32-3248s035-v0.21.04.bin` at `0x10000`

### ESP32-S3-3248S035-N16R8

- `esp32-s3-3248s035-n16r8-v0.21.04_bootloader.bin` at `0x0000`
- `esp32-s3-3248s035-n16r8-v0.21.04_partitions.bin` at `0x8000`
- `esp32-s3-3248s035-n16r8-v0.21.04_boot_app0.bin` at `0xE000`
- `esp32-s3-3248s035-n16r8-v0.21.04.bin` at `0x10000`

## Notes

- `ESP32-2432S024C` and `ESP32-3248S035` should not be flashed with only the main firmware image when using Espressif flash tools
- `pio run -t upload` still handles the correct image layout automatically when flashing from source
