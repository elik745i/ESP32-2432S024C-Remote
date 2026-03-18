# v0.21.02

## Highlights

- Improved chat responsiveness on `HC-12` and `E220` by keeping radio delivery queue-driven and non-blocking
- Reduced half-duplex contention by sending radio ACKs earlier and removing avoidable blocking waits
- Fixed false failed-message `X` states so delivered messages are more likely to show the correct checkmark state
- Airplane mode chat prompt now appears only for conversations that require `Wi-Fi` or `MQTT`; radio chats no longer trigger it
- Release bundles now include full flash-tool image sets for all three supported boards

## Included release assets

### ESP32-2432S024C

- `esp32-2432s024c-v0.21.02_bootloader.bin` at `0x1000`
- `esp32-2432s024c-v0.21.02_partitions.bin` at `0x8000`
- `esp32-2432s024c-v0.21.02_boot_app0.bin` at `0xE000`
- `esp32-2432s024c-v0.21.02.bin` at `0x10000`

### ESP32-3248S035

- `esp32-3248s035-v0.21.02_bootloader.bin` at `0x1000`
- `esp32-3248s035-v0.21.02_partitions.bin` at `0x8000`
- `esp32-3248s035-v0.21.02_boot_app0.bin` at `0xE000`
- `esp32-3248s035-v0.21.02.bin` at `0x10000`

### ESP32-S3-3248S035-N16R8

- `esp32-s3-3248s035-n16r8-v0.21.02_bootloader.bin` at `0x0000`
- `esp32-s3-3248s035-n16r8-v0.21.02_partitions.bin` at `0x8000`
- `esp32-s3-3248s035-n16r8-v0.21.02_boot_app0.bin` at `0xE000`
- `esp32-s3-3248s035-n16r8-v0.21.02.bin` at `0x10000`

## Notes

- `ESP32-2432S024C` and `ESP32-3248S035` should not be flashed with only the main firmware image when using Espressif flash tools
- `pio run -t upload` still handles the correct image layout automatically when flashing from source
