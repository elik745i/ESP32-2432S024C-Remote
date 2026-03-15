## ESP32 Touch Remote v0.2.17

- Added the `ESP32-S3-3248S035-N16R8` to `Ebyte E220-400T22D` schematic image and linked it from the README.
- Refreshed the tracked wiring reference image in `documents/SWAP.png`.
- Renamed `Train Battery` to `Battery` in the UI and README.
- Changed battery calibration so manual `FULL` maps the confirmed reading directly to the `4.20V` full anchor.
- Moved background battery auto-calibration behind the `Auto Calibration` button with a confirmation popup and persisted opt-in state.
- Kept manual `DISCHARGE` training working independently of background auto-calibration.
- Fixed double-tap sleep/screensaver so it suppresses accidental button clicks underneath the gesture.
- Fixed `Style -> Power Off` so the `Screen Timeoff` control redraws immediately after timeout capping changes.
- Reduced duplicate background sensor sampling and tightened LVGL idle scheduling to improve UI smoothness.
