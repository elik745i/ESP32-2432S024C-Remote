# Release Notes

## v0.21.12

- Added `Deep Sleep` under `Config -> Screen` for `ESP32-2432S024C` and `ESP32-3248S035`, with the same timeout flow as `Power Off`, RGB shutdown before sleep, and best-effort HC-12 sleep handling.
- Made `Radio Control -> Button Config` horizontally scrollable on `ESP32-2432S024C` and added a sideways swipe hint so the off-screen toggle and confirmation controls remain accessible.
- Normalized persisted remote-control keys, improved bundled module pack availability so installs work immediately during GitHub catalog refresh, and kept module action buttons usable while the catalog check runs.
- Updated `Receiver Test` to configure the receiver from the saved `Config -> Remote Control` radio profile and report the active HC-12 or E220 settings in the UI.