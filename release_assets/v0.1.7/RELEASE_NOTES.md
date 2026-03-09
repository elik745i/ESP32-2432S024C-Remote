## v0.1.7

- Switched the screensaver to a direct port of the actual `esp32-eyes-main` eye geometry and color, adapted to all supported display sizes.
- Fixed the sloped eyelid cut behavior in the screensaver so expressions no longer collapse or warp when the top edge is sliced.
- Fixed S3 OTA download handling by following GitHub release redirects and using the full OTA buffer size during streaming writes.
- Fixed S3 OTA apply/boot validation so successful updates boot into the new image instead of rolling back to the previous firmware.
- Corrected the runtime firmware version string so the OTA checker and Info screen report the actual released build.
- Moved the automatic OTA availability check to happen shortly after boot and again on first Wi-Fi connect, in addition to the existing periodic checks.

### Firmware Assets

- `esp32-2432s024c-v0.1.7.bin`
- `esp32-3248s035-v0.1.7.bin`
- `esp32-s3-3248s035-n16r8-v0.1.7.bin`
