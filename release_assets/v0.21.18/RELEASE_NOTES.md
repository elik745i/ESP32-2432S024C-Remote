# Release Notes

## v0.21.18

- Fixed the top sound-popup vibration control so the action button can re-enable vibration after it was set to `Off`.
- Kept the `ESP32-S3-3248S035-N16R8` source build targeted at the correct PlatformIO environment and serialized that build path to avoid Windows linker artifact loss.
- Restored valid PlatformIO environment definitions so the S3 build and related board targets parse cleanly again.
