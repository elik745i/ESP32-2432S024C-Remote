# Release Notes

## v0.21.08

- Reworked the OTA success dialog into a full-screen update summary that can show release notes after the unit finishes updating.
- Stored and surfaced per-release OTA notes from GitHub releases so post-update confirmation includes what changed in that firmware.
- Improved touch responsiveness by throttling non-UI background work while the screen is actively being touched or animated.
- Reduced redraw churn in several live UI paths to make scrolling, taps, and media/status updates feel smoother.
- Improved swipe-back completion so transitions continue more naturally from the finger release instead of snapping to the previous screen.
