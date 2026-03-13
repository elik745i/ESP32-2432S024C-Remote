## v0.2.11

This release adds persisted UI language selection, packages the multilingual LVGL font generation needed for non-Latin labels, and refreshes the repository documentation with additional project photos.

### UI and Localization

- Added a new `Language` entry in `Config`, placed above `OTA Updates`.
- Added persisted display language selection for English, Russian, Chinese, French, Turkish, Italian, German, Japanese, and Korean.
- Main menu and config menu labels now switch immediately when the language is changed.
- Added an auto-generated LVGL multilingual font subset so Cyrillic, CJK, and Hangul UI labels render on-device.

### Build and Documentation

- Added an automatic pre-build font generation step to PlatformIO.
- Bundled local Noto CJK font sources used to generate the multilingual LVGL font file.
- Updated the README with the new language feature and additional hardware/project photos.
- Included newly added repository assets from the `documents`, `fonts`, `scripts`, and generated-source directories.

### Firmware Binaries

- `esp32-2432s024c-v0.2.11.bin`
- `esp32-3248s035-v0.2.11.bin`
- `esp32-s3-3248s035-n16r8-v0.2.11.bin`
