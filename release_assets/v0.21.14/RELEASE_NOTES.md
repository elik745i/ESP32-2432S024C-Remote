# Release Notes

## v0.21.14

- Deferred Wi-Fi, MQTT, OTA, and radio startup work so the UI becomes responsive earlier after boot.
- Kept the selected remote radio profile active in RAM for `Radio Control`, removed the send-time restore churn, and fixed HC-12 receiver-test/profile reapply handling.
- Added a `15 sec` timeout option to `Power Off` and `Deep Sleep` screen settings.
- Deep-sleep timeout now puts the active radio module to sleep first, works while charging, and is available on the `ESP32-S3-3248S035-N16R8` target too.