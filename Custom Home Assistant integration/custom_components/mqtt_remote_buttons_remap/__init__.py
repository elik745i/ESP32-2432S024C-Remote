from __future__ import annotations

import logging
from typing import Any

from homeassistant.components import mqtt
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant, callback

from .const import CONF_ACTION_MAP, CONF_HWID, CONF_TOPIC, DOMAIN, EVENT_BUTTON_ACTION

_LOGGER = logging.getLogger(__name__)


async def async_setup(hass: HomeAssistant, config: dict[str, Any]) -> bool:
    return True


async def _async_process_message(hass: HomeAssistant, entry: ConfigEntry, topic: str, payload: str) -> None:
    action = payload.strip()
    if not action:
        return

    event_data = {
        "entry_id": entry.entry_id,
        "name": entry.title,
        "hwid": entry.data.get(CONF_HWID),
        "topic": topic,
        "action": action,
        "payload": payload,
    }
    hass.bus.async_fire(EVENT_BUTTON_ACTION, event_data)

    action_map = entry.data.get(CONF_ACTION_MAP, {})
    mapping = action_map.get(action)
    if not mapping:
        _LOGGER.debug("No mapping for entry '%s' action '%s'", entry.title, action)
        return

    service_ref = mapping.get("service", "")
    target_entity_id = mapping.get("target_entity_id")
    if not service_ref or not target_entity_id or "." not in service_ref:
        _LOGGER.warning("Skipping invalid mapping for entry '%s' action '%s'", entry.title, action)
        return

    service_domain, service_name = service_ref.split(".", 1)
    await hass.services.async_call(service_domain, service_name, {"entity_id": target_entity_id}, blocking=False)


async def async_setup_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    topic = entry.data[CONF_TOPIC]
    unsubscribers = hass.data.setdefault(DOMAIN, {})

    @callback
    def _message_received(message: mqtt.ReceiveMessage) -> None:
        hass.async_create_task(_async_process_message(hass, entry, message.topic, message.payload))

    unsubscribe = await mqtt.async_subscribe(hass, topic, _message_received, 0)
    unsubscribers[entry.entry_id] = unsubscribe
    _LOGGER.info("Subscribed '%s' to topic '%s'", entry.title, topic)
    return True


async def async_unload_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    unsubscribe = hass.data.get(DOMAIN, {}).pop(entry.entry_id, None)
    if unsubscribe is not None:
        unsubscribe()
    return True

