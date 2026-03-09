## v0.1.8

- Fixed AP-mode fallback so the device always brings up a discoverable default AP SSID/password when saved AP credentials are blank or invalid.
- Improved AP-mode networking, chat, and OTA status handling so AP-only usage reports clearer status and avoids STA-only assumptions.
- Added AP IP visibility to the LVGL Info screen and made the browser root open Wi-Fi setup first while AP mode is active.
- Restored legacy Wi-Fi web routes expected by the SD frontend so saved-network actions, including Forget, refresh correctly.
- Fixed the LVGL Wi-Fi config screen so forgetting a saved network clears the remembered entry immediately.
- Fixed LCD Wi-Fi scanning in AP mode and cleared stale discovered-network results once a connection starts.
- Reworked Snake and Tetris with slower starting speeds, gradual speed-up, start/replay overlays, persistent high scores, and larger touch controls.
- Updated Snake with wider arrow-button controls, a centered play/pause button, taller start/game-over overlays, an in-canvas pause overlay, and a 2x slower movement pace.
- Updated Tetris to match the same on-canvas score/best layout, keyboard-style control pad, centered play/pause button, taller start/game-over overlays, and in-canvas pause overlay behavior used by Snake.

### Firmware Assets

- `esp32-2432s024c-v0.1.8.bin`
- `esp32-3248s035-v0.1.8.bin`
- `esp32-s3-3248s035-n16r8-v0.1.8.bin`
