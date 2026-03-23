from __future__ import annotations

DOMAIN = "mqtt_remote_buttons_remap"
NAME = "MQTT Remote Buttons Remap"
VERSION = "0.1.0"

ACTION_MODE_PRESS = "press"
ACTION_MODE_STATE = "state"

BUTTON_TARGET_DOMAINS = {"button", "input_button", "scene", "script"}
STATE_TARGET_DOMAINS = {"switch", "input_boolean", "light", "fan"}

CONF_ACTION_MAP = "action_map"
CONF_DATA = "data"
CONF_DEVICE_ID = "device_id"
CONF_HWID = "hwid"
CONF_MAPPINGS = "mappings"
CONF_QOS = "qos"
CONF_SOURCE_MAP = "source_map"
CONF_TARGET = "target"
CONF_TOPIC = "topic"

DEFAULT_QOS = 0
EVENT_BUTTON_ACTION = "mqtt_remote_buttons_remap_button_action"
