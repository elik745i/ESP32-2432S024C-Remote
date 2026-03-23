from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from homeassistant.helpers import device_registry as dr
from homeassistant.helpers import entity_registry as er

from .const import (
    ACTION_MODE_PRESS,
    ACTION_MODE_STATE,
    BUTTON_TARGET_DOMAINS,
    STATE_TARGET_DOMAINS,
)


@dataclass(slots=True)
class RemoteSource:
    entity_id: str
    unique_id: str
    name: str
    source_domain: str
    base_action: str
    mode: str


def slugify_remote_action(name: str, fallback: str) -> str:
    lowered = name.lower()
    out: list[str] = []
    prev_underscore = False
    for char in lowered:
        if char.isalnum():
            out.append(char)
            prev_underscore = False
        elif char in {" ", "-", "_"}:
            if not prev_underscore:
                out.append("_")
                prev_underscore = True
    result = "".join(out).strip("_")
    return result or fallback


def button_service_for_domain(domain: str) -> str | None:
    if domain == "button":
        return "button.press"
    if domain == "input_button":
        return "input_button.press"
    if domain == "scene":
        return "scene.turn_on"
    if domain == "script":
        return "script.turn_on"
    return None


def state_services_for_domain(domain: str) -> tuple[str, str] | None:
    if domain in STATE_TARGET_DOMAINS:
        return (f"{domain}.turn_on", f"{domain}.turn_off")
    return None


def source_kind_label(mode: str) -> str:
    return "toggle" if mode == ACTION_MODE_STATE else "press"


def extract_hwid(device: dr.DeviceEntry) -> str | None:
    for identifier in device.identifiers:
        if len(identifier) < 2:
            continue
        identifier_value = str(identifier[1])
        if identifier_value.startswith("esp32_remote_"):
            return identifier_value.removeprefix("esp32_remote_")
    return None


def list_remote_devices(hass) -> list[tuple[dr.DeviceEntry, str]]:
    device_registry = dr.async_get(hass)
    results: list[tuple[dr.DeviceEntry, str]] = []
    for device in device_registry.devices.values():
        hwid = extract_hwid(device)
        if not hwid:
            continue
        name = device.name_by_user or device.name or f"ESP32 Remote {hwid}"
        results.append((device, f"{name} [{hwid}]"))
    results.sort(key=lambda item: item[1].lower())
    return results


def list_remote_sources(hass, device_id: str) -> list[RemoteSource]:
    entity_registry = er.async_get(hass)
    sources: list[RemoteSource] = []
    fallback_index = 1

    for entry in entity_registry.entities.values():
        if entry.device_id != device_id:
            continue
        if entry.platform != "mqtt":
            continue
        if entry.domain not in {"button", "switch"}:
            continue
        unique_id = entry.unique_id or ""
        if "_mqtt_control_" not in unique_id:
            continue

        display_name = entry.original_name or entry.name or entry.entity_id
        base_action = slugify_remote_action(display_name, f"button_{fallback_index}")
        fallback_index += 1
        mode = ACTION_MODE_STATE if entry.domain == "switch" else ACTION_MODE_PRESS
        sources.append(
            RemoteSource(
                entity_id=entry.entity_id,
                unique_id=unique_id,
                name=display_name,
                source_domain=entry.domain,
                base_action=base_action,
                mode=mode,
            )
        )

    sources.sort(key=lambda source: source.name.lower())
    return sources


def list_target_entities(hass, mode: str) -> list[tuple[str, str]]:
    entity_registry = er.async_get(hass)
    domains = BUTTON_TARGET_DOMAINS if mode == ACTION_MODE_PRESS else STATE_TARGET_DOMAINS
    entities: list[tuple[str, str]] = []
    for entry in entity_registry.entities.values():
        if entry.disabled_by is not None:
            continue
        if entry.domain not in domains:
            continue
        label = entry.original_name or entry.name or entry.entity_id
        entities.append((entry.entity_id, f"{label} ({entry.entity_id})"))
    entities.sort(key=lambda item: item[1].lower())
    return entities


def build_action_map(sources: list[RemoteSource], mapping_selection: dict[str, str]) -> dict[str, dict[str, Any]]:
    action_map: dict[str, dict[str, Any]] = {}
    for source in sources:
        target_entity_id = mapping_selection.get(source.entity_id, "")
        if not target_entity_id:
            continue
        target_domain = target_entity_id.split(".", 1)[0]
        if source.mode == ACTION_MODE_PRESS:
            service = button_service_for_domain(target_domain)
            if not service:
                continue
            action_map[source.base_action] = {
                "target_entity_id": target_entity_id,
                "service": service,
                "mode": ACTION_MODE_PRESS,
                "source_name": source.name,
            }
            continue

        services = state_services_for_domain(target_domain)
        if not services:
            continue
        service_on, service_off = services
        action_map[f"{source.base_action}_on"] = {
            "target_entity_id": target_entity_id,
            "service": service_on,
            "mode": ACTION_MODE_STATE,
            "source_name": source.name,
            "state": "on",
        }
        action_map[f"{source.base_action}_off"] = {
            "target_entity_id": target_entity_id,
            "service": service_off,
            "mode": ACTION_MODE_STATE,
            "source_name": source.name,
            "state": "off",
        }

    return action_map
