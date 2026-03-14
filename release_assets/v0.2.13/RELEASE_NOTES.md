## v0.2.13

This release adds configurable message tones, improves swipe-back rendering on chat screens, and trims UI/runtime overhead to improve touch responsiveness.

### Notifications and settings

- Added live-preview vibration intensity changes in `Config` on the ESP32-S3 build.
- Added a persisted `Message tone` dropdown with built-in beep and melody patterns for incoming chat messages.
- Message tone previews use the same notification audio path and follow the saved volume setting.
- Fixed the Style timezone selector so the saved GMT offset is restored correctly when reopening the screen and after reboot.

### UI and performance

- Reduced redundant sensor and Info-screen refresh work during active interaction.
- Adjusted LVGL handler scheduling to lower idle CPU load while preserving responsive touch and animation handling.
- Improved touch continuity by reusing the last sampled touch state between poll windows instead of reporting false releases.
- Fixed swipe-back from an open conversation so the previous chat-list view is previewed correctly instead of showing a white/blank background.

### Battery

- Added persisted battery self-calibration using observed full-charge plateaus and inferred low-battery cutoff events.

### Firmware binaries

- `esp32-2432s024c-v0.2.13.bin`
- `esp32-3248s035-v0.2.13.bin`
- `esp32-s3-3248s035-n16r8-v0.2.13.bin`
