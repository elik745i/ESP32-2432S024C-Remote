## v0.2.1

- Added persistent `last` and `best` game stats for `Snake`, `Tetris`, and PSRAM `Snake 3D`, plus persistent win counters for `Checkers`.
- Added an `HC12 Config` screen on the `ESP32-S3-3248S035-N16R8` build with `SET` mode control, a live UART terminal, and command sending for HC-12 radio modules.
- Reused the shared config keyboard behavior on the HC-12 screen so the command field stays visible while typing.
- Corrected the HC-12 UART mapping for module-side wiring `RXD -> GPIO4`, `TXD -> GPIO5`, and `SET -> GPIO3`.
- Reordered the `Config` menu so `HC12 Config` sits directly under `WiFi Config`, while `OTA Updates` is moved to the last menu button before `Device Name`.

### Firmware Assets

- `esp32-2432s024c-v0.2.1.bin`
- `esp32-3248s035-v0.2.1.bin`
- `esp32-s3-3248s035-n16r8-v0.2.1.bin`
