## v0.2.16

This release adds the manual battery-training screen, packages the new battery-action icon assets, and refreshes the published binaries for all supported boards.

### Battery training

- Added `Config -> Train Battery` with live calibration readout.
- `Reset` now clears stored battery anchors, shows charger instructions, and temporarily forces `Power Off` to `Never`.
- Added manual `FULL` and `DISCHARGE` actions so the device can capture full and empty anchors across real charge and cutoff cycles.
- `Auto Calibration` remains available on the same screen and exits manual training while keeping the background learning algorithm active.
- Manual battery-training state is stored in NVS so discharge training can survive reboot and smart-BMS cutoff recovery.

### UI updates

- Added custom 3D icons for the battery-training `FULL`, `DISCHARGE`, and `Auto Calibration` buttons.
- Reduced the home `Power` icon size by 20% so it better matches the rest of the menu icons.

### Firmware binaries

- `esp32-2432s024c-v0.2.16.bin`
- `esp32-3248s035-v0.2.16.bin`
- `esp32-s3-3248s035-n16r8-v0.2.16.bin`
