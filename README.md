# ESP32 Touch Remote

Firmware for Sunton-style ESP32 touch display boards with:
- local LVGL touch UI
- Wi-Fi AP/STA management
- SD-backed recovery/file manager
- SD media playback
- MQTT actions with Home Assistant discovery
- screenshot capture and SD health telemetry

Current firmware version: **`0.1.2`**

Currently supported boards:
- `ESP32-2432S024C` (`240x320`, `ILI9341`, `CST820`)
- `ESP32-3248S035` (`320x480`, `ST7796`, `GT911`)

![ESP32 Remote Render](3D_Models/render1.jpeg)

## Recent Changes

- Added multi-board PlatformIO environments for `ESP32-2432S024C` and `ESP32-3248S035`
- Added runtime board abstraction for display size, device strings, and touch controller selection
- Added `GT911` touch support for the `ESP32-3248S035`
- Updated UI layout sizing so it follows the actual display height instead of assuming `240x320`
- Changed button touch feedback so buttons no longer flash on initial touch during swipe gestures
- Buttons now visually react only after a confirmed tap/click is released
- Edge-swipe back navigation remains enabled on menu-style screens

## Supported Hardware

### `ESP32-2432S024C`

Display:
- Driver: `ILI9341`
- Resolution: `240x320`
- Rotation: `0`

Touch:
- Controller: `CST820`
- Bus: I2C
- Address: `0x15`

### `ESP32-3248S035`

Display:
- Driver: `ST7796`
- Resolution: `320x480`
- Rotation: `0`

Touch:
- Controller: `GT911`
- Bus: I2C
- Common addresses: `0x5D`, `0x14`

Board discovery references used for this support:
- https://homeding.github.io/boards/esp32/panel-3248S035.htm
- https://github.com/ardnew/ESP32-3248S035

## Shared Pin Mapping

These pins are used by both currently supported boards in this project.

### Display (TFT_eSPI build flags)

| Signal | GPIO |
|---|---|
| `TFT_MOSI` | 13 |
| `TFT_MISO` | 12 |
| `TFT_SCLK` | 14 |
| `TFT_CS` | 15 |
| `TFT_DC` | 2 |
| `TFT_BL` | 27 |
| `TFT_RST` | `-1` |

### Touch

| Signal | GPIO |
|---|---|
| `TOUCH_SDA` | 33 |
| `TOUCH_SCL` | 32 |
| `TOUCH_RST` | 25 |
| `TOUCH_IRQ` | 21 |

### SD Card

| Signal | GPIO |
|---|---|
| `SD_CS` | 5 |
| `SD_MOSI` | 23 |
| `SD_MISO` | 19 |
| `SD_SCK` | 18 |

Mount/recovery SPI speeds:
- `8 MHz`
- `4 MHz`
- `1 MHz`

### Audio / LEDs / Sensors

| Function | GPIO / Value |
|---|---|
| Audio DAC left (I2S port 0) | 26 |
| RGB R | 4 |
| RGB G | 17 |
| RGB B | 16 |
| RGB mode | active-low |
| Battery ADC | 35 |
| Light ADC | 34 |

Note: RGB logical R/G channels are swapped in software to match the board wiring used by this project.

## Default Runtime Properties

Defaults vary slightly by board for identity strings.

### Network defaults

- AP password: `12345678`
- Boot STA reconnect timeout: `12000 ms`

Board-specific defaults:

| Board | AP SSID | mDNS host |
|---|---|---|
| `ESP32-2432S024C` | `ESP32-2432S024C-FM` | `esp32-2432s024c` |
| `ESP32-3248S035` | `ESP32-3248S035-FM` | `esp32-3248s035` |

### MQTT defaults

- Enabled: `false`
- Broker: `homeassistant.local`
- Port: `1883`
- Discovery prefix: `homeassistant`
- Default button count: `4`
- Max button count: `12`
- Reconnect period: `5000 ms`

### SD health / recovery defaults

- Recovery retry delay: `80 ms`
- Min interval between remount attempts: `10000 ms`
- Background auto-retry period: `30000 ms`
- SD stats serial print period: `30000 ms`

### Battery / light behavior defaults

- Battery range: empty `3.30V`, full `4.20V`
- Battery calibration factor: `0.96`
- Battery samples: `16`
- Light samples: `8`
- Display idle timeout: `120000 ms`
- Light sleep check after idle: `20000 ms`

## UI and Gesture Notes

- Vertical swipe scrolls list-style screens
- Swipe right from the left screen edge navigates back on supported sub-screens
- Screenshot capture is available from the `Config` screen
- Button color feedback is intentionally delayed until a click is confirmed on release, to avoid false visual tap feedback during swipes

## SD Layout Used by Firmware

- `/web` for the primary static UI
- `/Screenshots` for JPEG screenshots and diagnostics

Recovery file APIs are intentionally restricted to `/web` and `/Screenshots`.

## HTTP Endpoints

### System / portal

| Method | Path | Purpose |
|---|---|---|
| `GET` | `/version` | Returns firmware version string |
| `GET` | `/` | Serves `/web/index.*` when available; otherwise redirects to setup or recovery flow |
| `GET` | `/wifi` | Wi-Fi setup page |
| `GET` | `/upload` | Fallback upload manager |
| `GET` | `/recovery` | Recovery browser |
| `GET` | `/generate_204`, `/fwlink`, `/hotspot-detect.html`, `/redirect` | Captive-portal helper redirects |
| `GET` | `/connecttest.txt`, `/ncsi.txt` | Captive checks |

### Wi-Fi / telemetry APIs

| Method | Path | Notes |
|---|---|---|
| `GET` | `/api/wifi/scan` | Returns scanned networks |
| `POST` | `/api/wifi/connect` | Params: `ssid`, `pass` |
| `GET` | `/api/telemetry` | Battery, light, and Wi-Fi snapshot |

### SD / file manager APIs

| Method | Path | Notes |
|---|---|---|
| `GET` | `/fs/list` | Optional `dir` query param |
| `POST` | `/fs/mkdir` | Param: `path` |
| `GET` | `/fs/download` | Query: `path` |
| `POST` | `/fs/delete` | Param: `path` |
| `POST` | `/fs/upload` | Multipart upload with `path` |
| `GET` | `/sd_info` | SD storage plus SD fault/recovery counters |

## SD Telemetry

`/sd_info` includes storage values (`total`, `used`, `free`) and SD health counters such as:
- `sd_faults`
- `sd_remount_attempts`
- `sd_remount_ok`
- `sd_remount_fail`
- `sd_try_8mhz`
- `sd_try_4mhz`
- `sd_try_1mhz`
- `sd_force_remounts`
- `sd_auto_retries`
- `sd_last_fault_ms`
- `sd_last_remount_ok_ms`
- `sd_last_remount_fail_ms`
- `sd_last_fault_at`

Serial logs also report SD recovery events:
- `[SD] I/O failure at ...`
- `[SD] remount ok @ ... Hz`
- `[SD] remount failed at all SPI speeds`
- `[SD][stats] ...`

## Media Support

Media browser/player accepts:

`.mp3`, `.wav`, `.flac`, `.aac`, `.m4a`, `.raw`, `.mpga`, `.mpeg`, `.wave`, `.adts`, `.m4b`, `.f4a`

## Build and Flash

### Requirements

- PlatformIO Core
- USB serial access to the ESP32 board

### Available environments

- `esp32-2432s024c`
- `esp32-3248s035`

### Switch boards

There are two ways to switch the target board.

1. Build or upload a specific environment directly:

```powershell
pio run -e esp32-2432s024c
pio run -e esp32-3248s035
```

```powershell
pio run -e esp32-2432s024c -t upload
pio run -e esp32-3248s035 -t upload
```

2. Change the default environment in [platformio.ini](platformio.ini):

```ini
[platformio]
default_envs = esp32-2432s024c
```

Set it to:
- `esp32-2432s024c` for the `ESP32-2432S024C`
- `esp32-3248s035` for the `ESP32-3248S035`

### Build

Build the current default environment:

```powershell
pio run
```

### Flash

Flash the current default environment:

```powershell
pio run -t upload
```

### Serial monitor

```powershell
pio device monitor -b 115200
```

## Partition Map

Configured via `partitions_3mb_no_ota.csv`.

See:
- [partitions_3mb_no_ota.csv](partitions_3mb_no_ota.csv)

## Project Structure

- [src/main.cpp](src/main.cpp) - firmware logic, UI, touch, routes, SD handling, MQTT
- [platformio.ini](platformio.ini) - multi-board build environments and display flags
- [partitions_3mb_no_ota.csv](partitions_3mb_no_ota.csv) - flash partition map
- [documents](documents) - board docs and datasheets
- [3D_Models](3D_Models) - enclosure and case files

## Release Binary

Release binaries are published in GitHub Releases for this repository.
