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
- Added asynchronous Wi-Fi scanning with a visible `Searching for access points...` loading state
- Renamed the Wi-Fi screen to `WiFi Config` and changed it to manual scan-on-demand
- Added saved-network controls (`Disconnect`, `Forget`) and editable `AP Config` on the same Wi-Fi screen
- Added in-device Wi-Fi password entry with on-screen keyboard for secured access points
- Added inline password visibility toggles to the Wi-Fi, AP, and MQTT password fields
- Updated Wi-Fi submenu back navigation so it returns to `Config`
- Added a `Config` brightness slider that controls real TFT backlight PWM
- Added a second `Config` slider for RGB LED intensity
- Brightness is saved in preferences and restored on the next boot
- The last `20%` of the brightness bar is highlighted red
- Boot now renders the first screen before enabling the backlight, then fades the backlight in
- Charging-before-sleep animation now runs a single cycle and cancels immediately on touch
- Wake touch handling now consumes the wake gesture fully before allowing normal menu taps
- Added global double-tap-to-sleep while keeping single-tap wake unchanged
- Shared on-screen keyboard now supports one-shot uppercase, caps lock on double-tap, and dismiss-on-tap-away/swipe
- Double-tap sleep is disabled while the on-screen keyboard is visible
- Added encrypted peer-to-peer chat over LAN using UDP and public-key cryptography
- Added encrypted global chat relay over MQTT using per-peer inbox topics
- Added automatic peer discovery plus a manual `Scan` action in `Chat -> Peers`
- Added peer pairing, enable/disable, and unpair flow directly in the on-device `Peers` screen
- Chat history is now stored on the SD card in `/Conversations`
- Each contact uses its own conversation text file, with friendlier filenames
- Chat now opens in contacts-first mode; all enabled paired peers stay in the contacts list even if history is cleared
- Selecting a contact opens its conversation, and conversation actions are available from a 3-dot menu
- Conversation menu supports `Clear` and `Clear for All`
- Swipe-back inside Chat now returns from a conversation to contacts before leaving Chat
- Chat now shows per-message delivery state: airplane while pending, green check after delivery, plus a per-message delete button
- Per-message delete is limited to outgoing messages only
- Airplane mode now blocks Chat send actions with a popup; `Cancel` opens Chat read-only and keeps the draft queued until airplane mode is turned off
- Added persistent device rename in `Config`
- Renamed devices are used in chat/discovery UI and old conversation display is normalized to current names
- Tapping the top-bar antenna icon now opens `WiFi Config`
- MQTT connect flow and several background services were staged to reduce UI stalls and keep touch/LVGL first priority
- Recovery browser access is no longer limited to fixed folders; it can browse all SD folders

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
- Swipe right on supported sub-screens navigates back when a clear horizontal gesture is detected, even when started away from the left edge
- Double-tap anywhere while the display is awake turns the screen off
- Double-tap sleep is ignored while the on-screen keyboard is open
- Screenshot capture is available from the `Config` screen
- Button color feedback is intentionally delayed until a click is confirmed on release, to avoid false visual tap feedback during swipes
- `Config -> WiFi Config` shows the saved/current STA network first and starts scanning only when `Scan` is pressed
- `WiFi Config` also includes editable AP SSID/password fields that are saved across reboots
- Tapping the top-bar antenna icon jumps directly to `WiFi Config`
- Tapping a secured AP opens an on-device password popup with keyboard support and password visibility toggle
- Tapping the keyboard `OK`/tick button in the Wi-Fi password popup acts the same as `Save`
- The AP password field and MQTT password field also include inline eye buttons
- The first touch while the display is asleep only wakes the screen; the next separate touch performs the menu action
- `Config` includes a brightness slider for screen backlight control, and its value is persisted across reboots
- `Config` also includes an RGB LED brightness slider, and its value is persisted across reboots
- `Config` also includes a device-name field; the saved name persists across reboots
- When editing the device name, the Config page scrolls the field above the on-screen keyboard
- Tapping away from the on-screen keyboard or swiping while it is open dismisses it first
- `Chat` now opens as a contacts list first, then switches to a single conversation after contact selection
- In conversation view, the contact name is shown in the top bar and swipe-back returns to the contacts list
- Chat message bubbles are left/right aligned by sender and capped to roughly `75%` width
- Outgoing messages show a left-side status badge: airplane while pending, green check after delivery
- Outgoing message rows include a trashcan button on the right to delete that one sent message from the UI, SD history, and pending outbox
- The conversation menu is opened from the 3-dot button; tapping away dismisses it
- If airplane mode is on, opening Chat shows a popup with `Cancel` and `Airplane Off`
- Choosing `Cancel` opens Chat in read-only mode; if a draft send is attempted while still blocked, the popup is shown again and the draft is retained until airplane mode is disabled
- `Chat -> Peers` shows paired devices and discovered devices in separate sections
- The `Scan` button forces an immediate discovery broadcast; devices are still discovered automatically in the background
- The `Scan` button shows inline state feedback: `Scanning...`, then `Done`, then back to `Scan`
- MQTT enable/disable now persists immediately from the on-screen switch and releases most MQTT buffer memory when turned off

### Future UI reference

For future UI experiments, widgets, and layout ideas, keep the upstream LVGL project/demo as a reference:
- https://github.com/lvgl/lvgl

## SD Layout Used by Firmware

- `/web` for the primary static UI
- `/Screenshots` for JPEG screenshots and diagnostics
- `/Conversations` for per-contact chat history logs

Recovery file APIs can browse and manage any rooted SD path.

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

### Chat / peer APIs

| Method | Path | Notes |
|---|---|---|
| `GET` | `/api/chat/messages` | Returns the currently loaded conversation window |
| `POST` | `/api/chat/send` | Sends chat text to the selected peer |
| `GET` | `/api/chat/identity` | Returns device name, UDP port, and public key |
| `GET` | `/api/chat/peers` | Returns trusted peers |
| `GET` | `/api/chat/discovery` | Returns discovered peers |
| `POST` | `/api/chat/discovery/pair` | Trusts a discovered peer |
| `POST` | `/api/chat/peers/add` | Adds a peer manually |
| `POST` | `/api/chat/peers/toggle` | Enables/disables a peer |
| `POST` | `/api/chat/peers/remove` | Removes a peer |

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

## Chat Overview

- LAN chat uses encrypted UDP peer-to-peer transport
- Global chat uses MQTT as a relay, but payloads are still encrypted per trusted peer
- MQTT chat is not a public room chat; devices must still be paired first
- Peer onboarding and management can be done on-device from `Chat -> Peers`
- Trusted peers are stored in preferences
- Recent chat history is reloaded from SD when a conversation is opened
- Only the recent in-memory window is kept in RAM; older history remains on SD
- Conversation history is stored per contact under `/Conversations`
- Outgoing messages show delivery state in the conversation UI
- Individual messages can be deleted from the conversation without clearing the whole thread
- Airplane mode can open Chat read-only, with drafts retained until radios are enabled again

## Chat Usage

### LAN chat

1. Connect both devices to the same Wi-Fi network.
2. Open `Chat -> Peers` on both devices.
3. Wait for auto discovery or tap `Scan` to force a discovery broadcast.
4. In the `Discovered` section, tap `Pair` for the other device.
5. Return to `Chat`, select the paired contact, and start messaging.

Notes:
- LAN chat uses encrypted UDP directly between paired devices.
- Paired devices remain available in the contacts list even after clearing conversation history.
- Delivered messages show a green check; messages still waiting for delivery show an airplane marker.

### MQTT chat

1. Pair the devices first using the LAN flow above.
2. On both devices, open `Config -> MQTT Config`.
3. Enable MQTT.
4. Enter the same broker settings on both devices.
5. Save the settings and wait for MQTT to connect.
6. When MQTT is connected, the top bar shows an MQTT icon next to the Wi-Fi indicator.
7. Open `Chat`, select the paired contact, and send messages normally.

Notes:
- MQTT is used as a global relay when direct LAN delivery is not enough.
- MQTT payloads are end-to-end encrypted per trusted peer using the same peer keys.
- The broker can relay messages but cannot read chat contents.
- The MQTT chat namespace is fixed in firmware for all remotes, so only broker settings must match.
- `Config -> MQTT Config` includes an inline eye toggle for the MQTT password field.

### Peer management

- `Paired Devices` section:
  - `Enable` / `Disable` temporarily controls whether that peer can be used
  - `Unpair` removes the trusted peer entry
- `Discovered` section:
  - `Pair` trusts that discovered device and adds it to saved peers
- Conversation menu in an open chat:
  - `Clear` removes only local history for that conversation
  - `Clear for All` removes local history and sends an encrypted delete request to the paired peer

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

If flashing does not start automatically, the board may not be entering the ESP32 bootloader on its own. On some USB ports, USB-serial adapters, or driver setups, you may need to do it manually:

1. Hold the `BOOT` button.
2. Press and release `RST` or `EN`.
3. Release `BOOT` when the upload begins.

If upload still fails, reconnect the USB cable, close any serial monitor, and try again.

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
