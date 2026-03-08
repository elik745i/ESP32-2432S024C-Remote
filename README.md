# ESP32 Touch Remote

Firmware for Sunton-style ESP32 touch display boards with an LVGL touch UI, Wi-Fi/AP management, SD-backed recovery tools, MQTT controls, and encrypted device-to-device chat.

Current firmware version: **`0.1.2`**

Supported boards:
- `ESP32-2432S024C` (`240x320`, `ILI9341`, `CST820`)
- `ESP32-3248S035` (`320x480`, `ST7796`, `GT911`)

![ESP32 Remote Render](3D_Models/render1.jpeg)

## Key Features

- Multi-board firmware with board-specific display and touch support selected at build time
- LVGL touch UI with swipe-back navigation, delayed click feedback, and global double-tap sleep
- LVGL touch/UI hot paths trimmed to reduce callback and off-screen refresh overhead
- Saved backlight brightness and RGB LED intensity controls in `Config`
- Brightness and RGB slider values are persisted in `Preferences`
- `WiFi Config` screen with current network info, manual scan, saved-network actions, and editable AP SSID/password
- `MQTT Config` screen for broker settings, status, and connection control
- SD recovery browser that can browse all rooted SD folders
- SD-backed chat history stored per contact under `/Conversations`
- Encrypted peer-to-peer LAN chat over UDP
- Encrypted global chat relay over MQTT
- On-device peer discovery, pairing, unpair, enable/disable, unread markers, and conversation actions

## Supported Hardware

### `ESP32-2432S024C`

Display:
- Driver: `ILI9341`
- Resolution: `240x320`

Touch:
- Controller: `CST820`
- Bus: I2C
- Address: `0x15`

### `ESP32-3248S035`

Display:
- Driver: `ST7796`
- Resolution: `320x480`

Touch:
- Controller: `GT911`
- Bus: I2C
- Common addresses: `0x5D`, `0x14`

Board references used while adding `ESP32-3248S035` support:
- https://homeding.github.io/boards/esp32/panel-3248S035.htm
- https://github.com/ardnew/ESP32-3248S035

## Shared Pin Mapping

### Display

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

### Audio, LEDs, Sensors

| Function | GPIO / Value |
|---|---|
| Audio DAC left (I2S port 0) | 26 |
| RGB R | 4 |
| RGB G | 17 |
| RGB B | 16 |
| RGB mode | active-low |
| Battery ADC | 35 |
| Light ADC | 34 |

Note: RGB logical red/green are swapped in software to match the wiring used by this project.

## Defaults

### Network

- AP password: `12345678`
- Boot STA reconnect timeout: `12000 ms`

Board-specific defaults:

| Board | AP SSID | mDNS host |
|---|---|---|
| `ESP32-2432S024C` | `ESP32-2432S024C-FM` | `esp32-2432s024c` |
| `ESP32-3248S035` | `ESP32-3248S035-FM` | `esp32-3248s035` |

### MQTT

- Enabled: `false`
- Broker: `homeassistant.local`
- Port: `1883`
- Discovery prefix: `homeassistant`
- Default button count: `4`
- Max button count: `12`
- Reconnect period: `5000 ms`

### Power / sensors

- Battery range: empty `3.30V`, full `4.20V`
- Battery calibration factor: `0.96`
- Battery samples: `16`
- Light samples: `8`
- Display idle timeout: `120000 ms`
- Light sleep check after idle: `20000 ms`

## UI Notes

- Buttons visually react on confirmed release, not on initial touch
- Horizontal swipe-back works on supported sub-screens and no longer has to start at the far left edge
- Single tap wakes the display, and the wake tap is consumed before normal UI interaction resumes
- Double-tap anywhere while awake turns the screen off
- Double-tap sleep is ignored while the on-screen keyboard is visible
- Charging sleep animation is limited to one cycle and cancels immediately on touch
- The first frame is rendered before the backlight fades in at boot
- Tapping the top-bar antenna icon opens `WiFi Config`
- Tapping the top-bar unread mail icon jumps directly into the first unread conversation
- The top bar can show device name, Wi-Fi state, MQTT state, unread mail, and battery

### Keyboard

- Shared on-screen keyboard is used across Wi-Fi, AP, MQTT, rename, and chat inputs
- Single shift tap enables one uppercase character
- Double shift tap locks uppercase until tapped again
- Tapping away or swiping while the keyboard is visible hides it first
- Double-tap sleep is disabled while the keyboard is open

### Config

- Screen backlight brightness slider with persistence
- RGB LED intensity slider with persistence
- Brightness and RGB values are restored after reboot
- Low-end brightness warning color on the slider
- Device rename field with persistence
- Device name is shown in the top bar and shortened there when needed

## Wi-Fi and AP

`Config -> WiFi Config` includes:
- Current/saved STA network at the top
- `Disconnect` and `Forget`
- Manual `Scan` button
- Discovered AP list below the scan area
- Password popup with inline eye toggle for secured networks
- `AP Config` section with saved AP SSID and password

Notes:
- AP SSID and password are stored in `Preferences`
- Wi-Fi scanning is on-demand from the screen; it does not auto-start on page open
- Some boards/USB adapters may require manual bootloader entry to flash:
  1. Hold `BOOT`
  2. Press and release `RST` or `EN`
  3. Release `BOOT` when upload starts

## MQTT

`Config -> MQTT Config` now contains only connection-related settings:
- enable switch
- broker
- port
- username
- password with inline eye toggle
- discovery prefix
- `Save`, `Connect`, `Discover`
- status window

`MQTT Controls` is the separate screen for MQTT action-button editing.

Notes:
- Turning MQTT off disconnects it immediately, saves the flag, and trims its buffer back down
- Pressing `Connect` auto-enables MQTT and updates the status window immediately
- MQTT connection and service work were moved away from direct UI handlers to reduce stalls
- All remotes use one fixed MQTT chat namespace in firmware

## Chat

### Overview

- LAN chat uses encrypted UDP peer-to-peer transport
- Global chat uses MQTT as an encrypted relay
- Devices must still be paired first; MQTT is not an open public room
- Trusted peers are stored in `Preferences`
- Conversation history is stored per contact under `/Conversations`
- Only a small recent window stays in RAM; older history is loaded from SD
- Contacts view shows all enabled paired peers, not just peers with existing history

### Chat flow

1. Open `Chat`
2. Select a paired contact
3. Conversation opens
4. Swipe back to return to contacts
5. Swipe back again to leave Chat

Conversation view includes:
- left/right aligned bubbles by sender
- roughly `75%` max bubble width
- unread indicator mail icon in the top bar
- unread green dot on contacts
- per-message send state
- 3-dot menu with `Clear` and `Clear for All`

Message state:
- pending outgoing message: airplane icon on the left
- delivered outgoing message: green check icon on the left
- outgoing messages only: trash icon on the right

Per-message delete:
- trash icon is shown only on messages you sent
- deleting a sent message removes it locally
- the same delete is also propagated to the paired device over LAN and MQTT

Conversation actions:
- `Clear` removes local conversation history only
- `Clear for All` removes the whole conversation on both paired devices

### Airplane mode behavior

- Airplane mode disables Wi-Fi/AP and MQTT
- Opening `Chat` while airplane mode is on shows a popup
- `Cancel` opens Chat read-only so old messages can still be read
- `Airplane Off` disables airplane mode and opens normal Chat
- If the user types a message while blocked and presses send, the popup is shown again
- Choosing `Cancel` keeps the draft and sends it later through the normal pending queue once radios are back

### Peer management

`Chat -> Peers` shows:
- `Scan` button at the top with `Scanning...` / `Done` state feedback
- `Paired Devices` section with `Enable` / `Disable` and `Unpair`
- `Discovered` section with `Pair`

Discovery behavior:
- peers auto-announce in the background on Wi-Fi
- `Scan` forces a fresh immediate discovery broadcast
- renaming a device updates peer names on other devices by public key and refreshes open conversations too

### LAN chat setup

1. Connect both devices to the same Wi-Fi network
2. Open `Chat -> Peers`
3. Wait for discovery or press `Scan`
4. Tap `Pair` on the discovered device
5. Return to `Chat`, choose the contact, and start messaging

### MQTT chat setup

1. Pair devices first
2. On both devices open `Config -> MQTT Config`
3. Enable MQTT
4. Enter the same broker settings
5. Save or press `Connect`
6. Wait for MQTT to connect
7. Use Chat normally

MQTT chat notes:
- payloads are end-to-end encrypted per trusted peer
- broker can relay messages but cannot read them
- paired peers can communicate globally as long as MQTT is connected

## SD Layout

- `/web` for the primary static UI
- `/Screenshots` for JPEG screenshots and diagnostics
- `/Conversations` for per-contact chat history

Recovery/file APIs can browse and manage any rooted SD path.

## HTTP Endpoints

### System / portal

| Method | Path | Purpose |
|---|---|---|
| `GET` | `/version` | Returns firmware version |
| `GET` | `/` | Serves `/web/index.*` or redirects into setup / recovery |
| `GET` | `/wifi` | Wi-Fi setup page |
| `GET` | `/upload` | Fallback upload manager |
| `GET` | `/recovery` | Recovery browser |
| `GET` | `/generate_204`, `/fwlink`, `/hotspot-detect.html`, `/redirect` | Captive-portal helpers |
| `GET` | `/connecttest.txt`, `/ncsi.txt` | Captive checks |

### Wi-Fi / telemetry

| Method | Path | Notes |
|---|---|---|
| `GET` | `/api/wifi/scan` | Returns scanned networks |
| `POST` | `/api/wifi/connect` | Params: `ssid`, `pass` |
| `GET` | `/api/telemetry` | Battery, light, Wi-Fi snapshot |

### Chat / peers

| Method | Path | Notes |
|---|---|---|
| `GET` | `/api/chat/messages` | Returns current conversation window |
| `POST` | `/api/chat/send` | Sends chat text to the selected peer |
| `GET` | `/api/chat/identity` | Returns device name, UDP port, public key |
| `GET` | `/api/chat/peers` | Returns trusted peers |
| `GET` | `/api/chat/discovery` | Returns discovered peers |
| `POST` | `/api/chat/discovery/pair` | Trusts a discovered peer |
| `POST` | `/api/chat/peers/add` | Adds a peer manually |
| `POST` | `/api/chat/peers/toggle` | Enables/disables a peer |
| `POST` | `/api/chat/peers/remove` | Removes a peer |

### SD / file manager

| Method | Path | Notes |
|---|---|---|
| `GET` | `/fs/list` | Optional `dir` query param |
| `POST` | `/fs/mkdir` | Param: `path` |
| `GET` | `/fs/download` | Query: `path` |
| `POST` | `/fs/delete` | Param: `path` |
| `POST` | `/fs/upload` | Multipart upload with `path` |
| `GET` | `/sd_info` | SD storage plus SD fault/recovery counters |

## SD Telemetry

`/sd_info` includes storage values and recovery counters such as:
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

Serial logs also report SD recovery events.

## Media Support

Supported audio extensions:
- `.mp3`
- `.wav`
- `.flac`
- `.aac`
- `.m4a`
- `.raw`
- `.mpga`
- `.mpeg`
- `.wave`
- `.adts`
- `.m4b`
- `.f4a`

## Build and Flash

### Requirements

- PlatformIO Core
- USB serial access to the ESP32 board

### Available environments

- `esp32-2432s024c`
- `esp32-3248s035`

### Switch boards

Run a specific environment directly:

```powershell
pio run -e esp32-2432s024c
pio run -e esp32-3248s035
pio run -e esp32-2432s024c -t upload
pio run -e esp32-3248s035 -t upload
```

Or change [platformio.ini](platformio.ini):

```ini
[platformio]
default_envs = esp32-2432s024c
; default_envs = esp32-3248s035
```

### Build

```powershell
pio run
```

### Flash

```powershell
pio run -t upload
```

### Serial monitor

```powershell
pio device monitor -b 115200
```

## Partition Map

Configured via [partitions_3mb_no_ota.csv](partitions_3mb_no_ota.csv).

## Project Structure

- [src/main.cpp](src/main.cpp) - firmware logic, UI, touch, SD, Wi-Fi, MQTT, chat, recovery
- [platformio.ini](platformio.ini) - multi-board PlatformIO environments and display flags
- [partitions_3mb_no_ota.csv](partitions_3mb_no_ota.csv) - flash partition map
- [documents](documents) - board docs and datasheets
- [3D_Models](3D_Models) - enclosure and case files

## Future UI Reference

For future UI experiments and LVGL demo ideas:
- https://github.com/lvgl/lvgl

## Release Binary

Release binaries are published in GitHub Releases for this repository.
