## v0.2.14

This release expands radio support, adds hardware power controls on the S3 build, and introduces a top-bar sound control for faster notification changes.

### Radio and chat

- Reworked `HC12 Config` into `Radio Config` with selectable `HC-12` or `Ebyte E220-400T22D` module support.
- Added persisted radio settings for both module types, including E220 transfer mode handling.
- Enabled encrypted radio chat and discovery over `HC-12` and `E220` when E220 is in `Transparent` mode.
- Added a warning popup when E220 is switched to `Fixed` mode because radio chat/discovery are disabled there.

### Power and device behavior

- Added an S3 home-screen `Power` button that sends the configured shutdown pulse on `GPIO21`.
- Added a persisted `Style -> Power Off` idle timeout with `2 min`, `5 min`, `15 min`, `30 min`, and `Never` options.
- Idle auto power-off now uses the same hardware shutdown signal path as the manual power button.

### Audio and top bar

- Added a top-bar sound status icon immediately left of the battery indicator.
- The unread-message indicator now sits left of the sound icon to keep the top bar ordered and readable.
- Tapping the sound icon opens a popup with a volume slider, vibration intensity selector, and one-touch vibration disable control.
- Volume, vibration intensity, and vibration enabled/disabled state are all persisted in memory.

### Firmware binaries

- `esp32-2432s024c-v0.2.14.bin`
- `esp32-3248s035-v0.2.14.bin`
- `esp32-s3-3248s035-n16r8-v0.2.14.bin`
