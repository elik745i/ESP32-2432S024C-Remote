## v0.2.10

This release refines the new swipe-back interaction, hardens it across all supported boards, and adds a few small UX updates around notifications and audio control.

### UI and Navigation

- Swipe-back now tracks live under the finger instead of waiting to animate only after release.
- The previous screen is rendered under the active screen during swipe-back for a true interactive back gesture.
- Releasing the swipe now completes the back navigation once about 30% of the previous screen is visible, even on slower drags.
- Reorder drag and swipe-back now arbitrate by touch intent so horizontal swipes stay navigation gestures while long-press still enables menu reordering.
- Added a new flat black button theme for users who prefer a darker, lower-contrast control style.
- The top bar center can now show either the device name or internet-synced time, with a remembered GMT offset selector.
- `Style` now includes a persisted `3D Icons` switch so menu rows can swap between custom embedded icons and standard LVGL symbols without rebooting.
- `Config` now includes a persisted `Volume` slider alongside brightness so speaker output can be adjusted without opening the media screen.
- Speaker output now uses an exponential volume curve, which gives finer control at the low end while keeping full output at 100%.
- Incoming chat messages now play a short notification beep on the speaker output when media playback is idle.
- Home and config menu entries now use embedded custom icons instead of LVGL text symbols for a cleaner menu layout.
- `WiFi Config` and `MQTT Config` now use dedicated embedded config icons instead of generic symbols.
- Additional `Config` rows now have dedicated 3D icons, including `MQTT Controls` and `Screenshot`, and the airplane icon asset has been refreshed.

### Performance and Stability

- Replaced the temporary LVGL `prev_scr` preview approach that could freeze pointer handling during interactive swipe-back.
- Enabled LVGL snapshot support for swipe previews and routed those preview buffers through the existing PSRAM-first LVGL allocator on the `ESP32-S3-3248S035-N16R8` build to avoid extra SRAM pressure.
- Non-PSRAM boards now skip the full snapshot preview and use a lightweight fallback underlay during swipe-back so gesture-back no longer hangs or exposes a white background.

### Firmware Binaries

- `esp32-2432s024c-v0.2.10.bin`
- `esp32-3248s035-v0.2.10.bin`
- `esp32-s3-3248s035-n16r8-v0.2.10.bin`
