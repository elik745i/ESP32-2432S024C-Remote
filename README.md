# ESP32-2432S024C Remote

Firmware for the **ESP32-2432S024C** touch display board, focused on:
- local touch UI
- Wi-Fi AP/STA management
- SD-backed web/recovery file management
- audio playback
- MQTT actions with Home Assistant discovery
- screenshot capture and SD health telemetry

Current firmware version: **`0.1.0`**

## Highlights

- Touch UI with screens for Home, Wi-Fi, Media, Info, Games, and MQTT config/control
- Built-in AP + captive helper routes for initial setup
- Recovery browser (`/recovery`) and fallback upload page (`/upload`)
- SD card resilience logic with remount retries and serial/API counters
- JPEG screenshot capture by left-swipe gesture on menu-style screens
- Media browser/player from SD (`/` and subfolders)
- MQTT button actions, retained availability, and Home Assistant discovery payloads
- Battery and ambient-light monitoring with smoothing and periodic snapshots

## Hardware and Pin Mapping

### Display (TFT_eSPI build flags)

| Signal | GPIO |
|---|---|
| TFT_MOSI | 13 |
| TFT_MISO | 12 |
| TFT_SCLK | 14 |
| TFT_CS | 15 |
| TFT_DC | 2 |
| TFT_BL | 27 |
| TFT_RST | -1 (not used) |

Display config: ILI9341, 240x320, rotation `0`.

### Touch (CST820 over I2C)

| Signal | GPIO |
|---|---|
| TOUCH_SDA | 33 |
| TOUCH_SCL | 32 |
| TOUCH_RST | 25 |
| TOUCH_IRQ | 21 |

Touch I2C address: `0x15`

### SD Card (SPI/TF)

| Signal | GPIO |
|---|---|
| SD_CS | 5 |
| SD_MOSI | 23 |
| SD_MISO | 19 |
| SD_SCK | 18 |

Mount/recovery SPI speeds: `8 MHz`, `4 MHz`, `1 MHz`

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

Note: RGB logical R/G channels are swapped in software to match board wiring.

## Default Runtime Properties

### Network defaults

- AP SSID: `ESP32-2432S024C-FM`
- AP password: `12345678`
- mDNS host: `esp32-2432s024c` (`esp32-2432s024c.local` when STA connected)
- Boot STA reconnect timeout: `12000 ms`

### MQTT defaults

- Enabled: `false`
- Broker: `homeassistant.local`
- Port: `1883`
- Discovery prefix: `homeassistant`
- Default button count: `4` (max `12`)
- Reconnect period: `5000 ms`

### SD health/recovery defaults

- Recovery retry delay: `80 ms`
- Min interval between remount attempts: `10000 ms`
- Background auto-retry period: `30000 ms`
- SD stats serial print period: `30000 ms` (only when counters changed)

### Battery/light behavior defaults

- Battery range: empty `3.30V`, full `4.20V`
- Battery calibration factor: `0.96`
- Battery samples: `16`
- Light samples: `8`
- Display idle timeout: `15000 ms`
- Light sleep check after idle: `20000 ms`

## SD Layout Used by Firmware

- `/web` for primary static UI
- `/Screenshots` for JPEG screenshots and diagnostics

Recovery file APIs are intentionally restricted to `/web` and `/Screenshots`.

## HTTP Endpoints

### System / portal

| Method | Path | Purpose |
|---|---|---|
| GET | `/version` | Returns firmware version string |
| GET | `/` | Serves `/web/index.*` when available; otherwise redirects to setup/recovery flow |
| GET | `/wifi` | Wi-Fi setup page |
| GET | `/upload` | Fallback upload manager |
| GET | `/recovery` | Recovery browser |
| GET | `/generate_204`, `/fwlink`, `/hotspot-detect.html`, `/redirect` | Captive-portal helper redirects |
| GET | `/connecttest.txt`, `/ncsi.txt` | Captive checks |

### Wi-Fi / telemetry APIs

| Method | Path | Notes |
|---|---|---|
| GET | `/api/wifi/scan` | Returns scanned networks |
| POST | `/api/wifi/connect` | Params: `ssid`, `pass` |
| GET | `/api/telemetry` | Battery/light/Wi-Fi snapshot |

### SD / file manager APIs

| Method | Path | Notes |
|---|---|---|
| GET | `/fs/list` | Optional `dir` query param |
| POST | `/fs/mkdir` | Param: `path` |
| GET | `/fs/download` | Query: `path` |
| POST | `/fs/delete` | Param: `path` |
| POST | `/fs/upload` | Multipart upload with `path` |
| GET | `/sd_info` | SD storage + SD fault/recovery counters |

## SD Telemetry (`/sd_info`)

`/sd_info` includes storage values (`total`, `used`, `free`) and SD health counters:

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

Also watch serial logs:
- `[SD] I/O failure at ...`
- `[SD] remount ok @ ... Hz`
- `[SD] remount failed at all SPI speeds`
- `[SD][stats] ...`

## Media Support

Media browser accepts these file extensions:

`.mp3`, `.wav`, `.flac`, `.aac`, `.m4a`, `.raw`, `.mpga`, `.mpeg`, `.wave`, `.adts`, `.m4b`, `.f4a`

## Gesture Notes

- Vertical swipe: page scrolling in Home/Wi-Fi/Media lists
- Left swipe (menu screens): capture screenshot to `/Screenshots`

## Build and Flash

### Requirements

- PlatformIO Core
- USB serial access to the ESP32 board

### Build

```powershell
pio run
```

### Flash (via PlatformIO)

```powershell
pio run -t upload
```

### Serial Monitor

```powershell
pio device monitor -b 115200
```

## Partition Map

Configured via `partitions_2mb_ota.csv`:

- `nvs` 0x4000
- `otadata` 0x2000
- `phy_init` 0x1000
- `app0` 0x200000
- `app1` 0x1E0000
- `coredump` 0x10000

## Project Structure

- `src/main.cpp` - firmware logic, UI, routes, SD handling, MQTT
- `platformio.ini` - build environment and display flags
- `partitions_2mb_ota.csv` - flash partition map
- `documents/` - board docs/datasheets
- `3D_Models/` - enclosure/case files

## Release Binary

Release binaries are published in GitHub Releases for this repository.
