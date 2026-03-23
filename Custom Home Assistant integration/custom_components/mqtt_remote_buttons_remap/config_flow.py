from __future__ import annotations

from typing import Any

import voluptuous as vol

from homeassistant import config_entries
from homeassistant.config_entries import ConfigEntry
from homeassistant.const import CONF_DEVICE_ID
from homeassistant.data_entry_flow import FlowResult
from homeassistant.helpers.selector import SelectOptionDict, SelectSelector, SelectSelectorConfig

from .const import CONF_ACTION_MAP, CONF_HWID, CONF_SOURCE_MAP, CONF_TOPIC, DOMAIN
from .helpers import build_action_map, extract_hwid, list_remote_devices, list_remote_sources, list_target_entities, source_kind_label


def _select(options: list[tuple[str, str]]) -> SelectSelector:
    return SelectSelector(
        SelectSelectorConfig(
            options=[SelectOptionDict(value=value, label=label) for value, label in options],
            mode="dropdown",
        )
    )


def _source_field_key(source) -> str:
    return f"{source.name} [{source_kind_label(source.mode)}]"


def _build_mapping_schema(hass, sources, existing_map: dict[str, str]) -> vol.Schema:
    fields: dict[Any, Any] = {}
    for source in sources:
        choices = [("", "Unmapped")] + list_target_entities(hass, source.mode)
        fields[vol.Required(_source_field_key(source), default=existing_map.get(source.entity_id, ""))] = _select(choices)
    return vol.Schema(fields)


class MqttRemoteButtonsRemapConfigFlow(config_entries.ConfigFlow, domain=DOMAIN):
    VERSION = 1

    def __init__(self) -> None:
        self._selected_device_id: str | None = None
        self._selected_hwid: str | None = None
        self._selected_title: str | None = None

    async def async_step_user(self, user_input: dict[str, Any] | None = None) -> FlowResult:
        devices = list_remote_devices(self.hass)
        if not devices:
            return self.async_abort(reason="no_remote_devices")

        available_devices: list[tuple[str, str, str]] = []
        for device, label in devices:
            hwid = extract_hwid(device)
            if not hwid:
                continue
            if any(entry.unique_id == hwid for entry in self._async_current_entries()):
                continue
            available_devices.append((device.id, hwid, label))

        if not available_devices:
            return self.async_abort(reason="already_configured")

        if len(available_devices) == 1 and user_input is None:
            device_id, hwid, title = available_devices[0]
            await self.async_set_unique_id(hwid)
            self._abort_if_unique_id_configured()
            return self.async_create_entry(
                title=title,
                data={
                    CONF_DEVICE_ID: device_id,
                    CONF_HWID: hwid,
                    CONF_TOPIC: f"esp32/remote/{hwid}/action",
                    CONF_SOURCE_MAP: {},
                    CONF_ACTION_MAP: {},
                },
            )

        if user_input is not None:
            self._selected_device_id = user_input[CONF_DEVICE_ID]
            for device_id, hwid, label in available_devices:
                if device_id != self._selected_device_id:
                    continue
                self._selected_hwid = hwid
                self._selected_title = label
                break
            await self.async_set_unique_id(self._selected_hwid)
            self._abort_if_unique_id_configured()
            return self.async_create_entry(
                title=self._selected_title or self._selected_hwid or DOMAIN,
                data={
                    CONF_DEVICE_ID: self._selected_device_id,
                    CONF_HWID: self._selected_hwid,
                    CONF_TOPIC: f"esp32/remote/{self._selected_hwid}/action",
                    CONF_SOURCE_MAP: {},
                    CONF_ACTION_MAP: {},
                },
            )

        schema = vol.Schema({vol.Required(CONF_DEVICE_ID): _select([(device_id, label) for device_id, _hwid, label in available_devices])})
        return self.async_show_form(step_id="user", data_schema=schema)

    @staticmethod
    def async_get_options_flow(config_entry):
        return MqttRemoteButtonsRemapOptionsFlow(config_entry)

    async def async_step_reconfigure(self, user_input: dict[str, Any] | None = None) -> FlowResult:
        entry = self._get_reconfigure_entry()
        device_id = entry.data.get(CONF_DEVICE_ID)
        sources = list_remote_sources(self.hass, device_id)
        if not sources:
            return self.async_abort(reason="no_remote_buttons")

        existing_map = entry.data.get(CONF_SOURCE_MAP, {})
        if user_input is not None:
            source_map = {source.entity_id: user_input.get(_source_field_key(source), "") for source in sources}
            self.hass.config_entries.async_update_entry(
                entry,
                data={
                    **entry.data,
                    CONF_SOURCE_MAP: source_map,
                    CONF_ACTION_MAP: build_action_map(sources, source_map),
                },
            )
            await self.hass.config_entries.async_reload(entry.entry_id)
            return self.async_abort(reason="reconfigure_successful")

        return self.async_show_form(
            step_id="reconfigure",
            data_schema=_build_mapping_schema(self.hass, sources, existing_map),
        )


class MqttRemoteButtonsRemapOptionsFlow(config_entries.OptionsFlow):
    def __init__(self, config_entry) -> None:
        self.config_entry = config_entry
        self._sources = []

    async def async_step_init(self, user_input: dict[str, Any] | None = None) -> FlowResult:
        if user_input is not None:
            return await self.async_step_map_buttons()

        return self.async_show_form(step_id="init", data_schema=vol.Schema({}))

    async def async_step_map_buttons(self, user_input: dict[str, Any] | None = None) -> FlowResult:
        device_id = self.config_entry.data.get(CONF_DEVICE_ID)
        self._sources = list_remote_sources(self.hass, device_id)
        if not self._sources:
            return self.async_abort(reason="no_remote_buttons")

        existing_map = self.config_entry.data.get(CONF_SOURCE_MAP, {})
        if user_input is not None:
            source_map = {source.entity_id: user_input.get(_source_field_key(source), "") for source in self._sources}
            updated_data = {
                **self.config_entry.data,
                CONF_SOURCE_MAP: source_map,
                CONF_ACTION_MAP: build_action_map(self._sources, source_map),
            }
            self.hass.config_entries.async_update_entry(self.config_entry, data=updated_data)
            await self.hass.config_entries.async_reload(self.config_entry.entry_id)
            return self.async_create_entry(title="", data={})

        return self.async_show_form(step_id="map_buttons", data_schema=_build_mapping_schema(self.hass, self._sources, existing_map))

