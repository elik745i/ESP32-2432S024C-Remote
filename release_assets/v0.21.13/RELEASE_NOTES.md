# Release Notes

## v0.21.13

- Added `Config Password` under `Config -> Screen`, defaulted to PIN `1111`, and require the current PIN before disabling either `Config Password` or `Screen Lock`.
- Removed the screenshot feature and cleaned up `Main Screen Items` so `Config` and `Power` stay available without separate visibility toggles.
- Fixed duplicate radio-control sends for gate and momentary actions, and keep the bundled Radio pack installed by default on `ESP32-2432S024C`.
- Hardened radio startup by auto-clearing invalid saved UART swaps and clarified the fixed HC-12 wiring shown on compact ESP32 boards.
