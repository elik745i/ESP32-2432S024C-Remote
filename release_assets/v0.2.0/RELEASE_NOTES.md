## v0.2.0

- Consolidates the earlier `v0.1.x` public release line into the first `v0.2.x` milestone.
- Carries forward the initial multi-board remote firmware foundation from `v0.1.0` to `v0.1.2`.
- Added discovery-gated chat pairing with accept/reject confirmation on the target device.
- Added persistent peer-discovery control and stopped discovery-disabled devices from replying or appearing in scans.
- Added persistent OTA update checking, top-bar update indication, and the `Config -> OTA Updates` screen.
- Added board-correct OTA asset selection for multi-board releases.
- Switched the screensaver to the actual `esp32-eyes` geometry and behavior adapted to supported displays.
- Improved swipe-back behavior and blocked it correctly on sliders, switches, and scrollable controls.
- Fixed S3 OTA download/apply handling so successful updates boot into the new image reliably.
- Improved AP fallback, AP-only status handling, and Wi-Fi config behavior for saved-network actions and scanning.
- Reworked 2D `Snake` and `Tetris` with slower starts, progressive speed-up, larger touch controls, centered overlays, and persistent scores.
- Added `Snake 3D` for the PSRAM-equipped `ESP32-S3-3248S035-N16R8` build with a software-rendered 3D chase camera, solid snake geometry, and touch controls for planar and depth movement.
- Updated the 3D Snake renderer with a taller chase camera, visible play surface tiles, and improved solid-face cube rendering for the snake and food.
- Expanded `Checkers` with a rule-set picker and support for American Checkers, International Draughts, Russian Checkers, Pool Checkers, and Canadian/Sri Lankan variants.
- Fixed the `Checkers` multiplayer invite flow so the selected rule set is included in invitations and both devices stay synchronized.
- Refined the 2D `Snake` overlays with larger in-canvas start/game-over popups and corrected centered overlay positioning over the board.
- Corrected PSRAM usage reporting on the `Info` screen so low utilization shows a rounded percentage instead of truncating to `0%`.

### Firmware Assets

- `esp32-2432s024c-v0.2.0.bin`
- `esp32-3248s035-v0.2.0.bin`
- `esp32-s3-3248s035-n16r8-v0.2.0.bin`
