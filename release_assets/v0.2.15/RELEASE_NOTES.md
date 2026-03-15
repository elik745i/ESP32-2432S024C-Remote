## v0.2.15

This release refreshes the published binaries after the latest style timeout logic change and clarifies LoRa module support in the README.

### Style timeout behavior

- Refined `Screen Timeoff` and `Power Off` interaction in `Style`.
- `Screen Timeoff` now stays at the user-selected value unless it is shorter than `Power Off - 1 minute`.
- This keeps the screen-off and hardware power-off timers aligned without forcing a shorter screen timeout than the one already selected.

### Radio / LoRa documentation

- README now explicitly describes the `Ebyte E220-400T22D` path as the optional LoRa radio module on the S3 build.
- Existing `HC-12` and `E220` radio configuration, wiring, and transparent-mode chat/discovery notes remain unchanged.

### Firmware binaries

- `esp32-2432s024c-v0.2.15.bin`
- `esp32-3248s035-v0.2.15.bin`
- `esp32-s3-3248s035-n16r8-v0.2.15.bin`
