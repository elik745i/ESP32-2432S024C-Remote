## ESP32 Touch Remote v0.2.18

- Added a stronger software filter to battery voltage display readings.
- Battery display now uses median smoothing plus a slower follow filter so readings do not jump as quickly.
- Raw battery readings are still kept separately for calibration and training logic.
- Improved S3 touch responsiveness by reducing the touch poll interval.
- Reduced UI lag by pausing LVGL screen warmup work briefly after user input.
- Moved large runtime registries and peer/discovery tables to PSRAM-backed storage to free internal SRAM.
- Added lazy allocation and null-safe guards for the moved UI and radio data structures.
