## ESP32 Touch Remote v0.2.20

- Fixed noise coming from SPK
- Fixed swipe action triggering button push.
- Fixed radio pin-swap persistence so swapped `HC-12` and `E220` pins survive reboot.
- Added clearer module-specific example commands in `Radio Terminal`.
- Added an OTA `Reflash` action to reinstall the current firmware on supported boards.
- Added an S3 battery ADC fallback path if `analogReadMilliVolts()` returns zero.
- Kept the battery calibration and screen-timeout persistence fixes from the previous refresh.
