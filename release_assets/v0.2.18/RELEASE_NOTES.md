## ESP32 Touch Remote v0.2.18

- Added a stronger software filter to battery voltage display readings.
- Battery display now uses median smoothing plus a slower follow filter so readings do not jump as quickly.
- Raw battery readings are still kept separately for calibration and training logic.
