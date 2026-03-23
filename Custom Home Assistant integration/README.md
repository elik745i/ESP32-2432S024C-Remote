# MQTT Remote Buttons Remap

Custom Home Assistant integration for this firmware family.

Revision: `0.1.0`

This integration adds a Home Assistant setup interface for ESP32 remotes that already advertise MQTT buttons and switches through Home Assistant MQTT discovery.

The interface works like this:

1. Choose the MQTT remote device from a dropdown.
2. The integration entry is added immediately.
3. Open that integration entry and configure mappings from its options screen.
4. The integration reads that device's advertised MQTT buttons and toggle controls already registered in Home Assistant.
5. Under each remote button, pick the Home Assistant target entity from a dropdown.
6. Leave any control as `Unmapped` if you do not want to use it.
7. Press-type MQTT buttons trigger press-style Home Assistant entities.
8. Toggle-type MQTT buttons mirror on and off state to a compatible Home Assistant entity.

## What It Listens To

The firmware publishes button actions to:

- `esp32/remote/<HWID>/action`

Examples of payloads:

- `up`
- `down`
- `left`
- `right`
- `button_5`
- `light_on`
- `light_off`

The last two forms are produced by toggle-type MQTT buttons configured in the device UI, and they are what the integration uses to keep mapped stateful targets in sync.

## Installation

1. Copy the folder `custom_components/mqtt_remote_buttons_remap` into your Home Assistant config directory under `custom_components/`.
2. Restart Home Assistant.
3. Make sure the built-in MQTT integration is already configured and connected to the same broker as the ESP32 remote.
4. In Home Assistant, open `Settings -> Devices & services -> Add Integration`.
5. Search for `MQTT Remote Buttons Remap`.
6. Select the ESP32 remote device from the dropdown.
7. The integration entry is created right away.
8. Open the added integration entry and open its options.
9. The first options screen shows built-in instructions.
10. Continue to the mapping screen and assign only the buttons you want.
11. Leave the rest as `Unmapped`.

Final target path in Home Assistant:

```text
config/
  custom_components/
    mqtt_remote_buttons_remap/
      __init__.py
      config_flow.py
      const.py
      helpers.py
      manifest.json
      strings.json
```

## UI Behavior

After the integration entry is created, its options flow first shows an instructions screen and then shows one dropdown per advertised MQTT control.

- Source `button` entities from the ESP32 remote are treated as press actions.
- Source `switch` entities from the ESP32 remote are treated as on/off state actions.

Target dropdown behavior:

- Press actions offer Home Assistant entities from `button`, `input_button`, `scene`, and `script` domains.
- Toggle actions offer Home Assistant entities from `switch`, `input_boolean`, `light`, and `fan` domains.
- Every dropdown also includes `Unmapped`, so you can map only the controls you actually want.

This split is intentional. A Home Assistant `button` entity is stateless, so it cannot mirror an MQTT `ON/OFF` state. For actual on/off synchronization, the target must be a stateful entity such as a `switch`, `light`, or `input_boolean`.

## Synchronization Rules

- MQTT press payloads like `up` or `button_5` call the mapped target's press-style service.
- MQTT toggle payloads like `lamp_on` and `lamp_off` call `turn_on` and `turn_off` on the mapped target.
- The integration also fires a Home Assistant event named `mqtt_remote_buttons_remap_button_action` for every received payload.

## Example Flow

Example mapping result:

- Remote source `Up` -> `button.tv_volume_up`
- Remote source `Down` -> `button.tv_volume_down`
- Remote source `Lamp` -> `light.corner_lamp`

Behavior:

- Receiving `up` triggers `button.press` on `button.tv_volume_up`.
- Receiving `down` triggers `button.press` on `button.tv_volume_down`.
- Receiving `lamp_on` triggers `light.turn_on` on `light.corner_lamp`.
- Receiving `lamp_off` triggers `light.turn_off` on `light.corner_lamp`.

## Event Emitted

The integration fires this Home Assistant event for every received action:

- `mqtt_remote_buttons_remap_button_action`

Event data:

- `entry_id`
- `name`
- `hwid`
- `topic`
- `action`
- `payload`

You can use that event in automations if you want to build logic outside the direct mappings saved through the UI.

## Notes

- The remote firmware publishes to a shared action topic per device, not one topic per button.
- The integration derives the list of available source buttons from the MQTT entities already discovered in Home Assistant for that selected device.
- If a button or switch is renamed on the ESP32 and rediscovered by Home Assistant, reopen the integration options and review the mappings.
