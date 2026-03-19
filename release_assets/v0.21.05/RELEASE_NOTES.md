# v0.21.05

## Highlights

- Added a persisted `Play sound on start` option in `Config` and boot-time startup feedback
- Fixed swipe-back on `Config -> Style -> Main Screen Items`
- OTA firmware list now includes the newest release in the selectable list and uses brighter white bold text
- Menu click feedback is now a very short, consistent click and vibration across menus and controls
- Keyboard input now keeps the active text field visible while typing instead of dropping behind the keyboard
- Keyboard shift now cycles `one-shot uppercase -> caps lock -> lowercase`
- Added swipe-on-space keyboard toggle between English and localized layouts where a practical map is available
- Fixed Russian/localized keyboard crash by adding explicit LVGL control maps for custom keyboard layouts

## Included release assets

### ESP32-2432S024C

- `esp32-2432s024c-v0.21.05_bootloader.bin` at `0x1000`
- `esp32-2432s024c-v0.21.05_partitions.bin` at `0x8000`
- `esp32-2432s024c-v0.21.05_boot_app0.bin` at `0xE000`
- `esp32-2432s024c-v0.21.05.bin` at `0x10000`

### ESP32-3248S035

- `esp32-3248s035-v0.21.05_bootloader.bin` at `0x1000`
- `esp32-3248s035-v0.21.05_partitions.bin` at `0x8000`
- `esp32-3248s035-v0.21.05_boot_app0.bin` at `0xE000`
- `esp32-3248s035-v0.21.05.bin` at `0x10000`

### ESP32-S3-3248S035-N16R8

- `esp32-s3-3248s035-n16r8-v0.21.05_bootloader.bin` at `0x0000`
- `esp32-s3-3248s035-n16r8-v0.21.05_partitions.bin` at `0x8000`
- `esp32-s3-3248s035-n16r8-v0.21.05_boot_app0.bin` at `0xE000`
- `esp32-s3-3248s035-n16r8-v0.21.05.bin` at `0x10000`

## Notes

- `ESP32-2432S024C` and `ESP32-3248S035` should not be flashed with only the main firmware image when using Espressif flash tools
- Chinese, Japanese, and Korean keyboard input still falls back to English layout because a full IME is outside the current LVGL keyboard scope
