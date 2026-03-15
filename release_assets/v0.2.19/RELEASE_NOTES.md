## ESP32 Touch Remote v0.2.19

- Added radio pin-swap selectors in `Radio Config`.
- `HC-12` can now swap `RX/TX` pins from the UI and persist that setting.
- `E220` can now swap both `RX/TX` and `M0/M1` pins from the UI and persist those settings.
- `Radio Terminal` now shows module-specific example commands for quick testing.
- Refreshed runtime pin handling so terminal, AT/config mode, and radio transport use the selected pin mapping.
- Fixed `Screen Timeoff` persistence so slider changes are always saved and restored.
- Fixed the battery screen so the auto-calibration control visibly reflects `ON/OFF` state.
- Manual `FULL` calibration now reloads the stored anchor immediately after saving for consistent screen state.
