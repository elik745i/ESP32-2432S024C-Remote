static constexpr int MAX_MQTT_DISCOVERED = 8;

WiFiClient mqttNetClient;
PubSubClient mqttClient(mqttNetClient);

struct MqttConfig {
    bool enabled = false;
    String broker = "homeassistant.local";
    uint16_t port = 1883;
    String username;
    String password;
    String discoveryPrefix = "homeassistant";
};

MqttConfig mqttCfg;
String mqttHwId;
String mqttClientId;
String mqttNodeId;
String mqttDeviceName;
String mqttActionTopic;
String mqttChatInboxTopic;
String mqttChatPresenceTopic;
String mqttStatusLine = "Disabled";
bool mqttDiscoveryPublished = false;
String mqttLastStatePayload;
unsigned long mqttLastReconnectMs = 0;
unsigned long mqttLastStatePublishMs = 0;
static unsigned long mqttNextReconnectMs = 0;
static unsigned long mqttReconnectIntervalMs = 5000UL;
static constexpr unsigned long MQTT_RECONNECT_MS = 5000UL;
static constexpr unsigned long MQTT_RECONNECT_MAX_MS = 60000UL;
static constexpr unsigned long MQTT_DISCOVERY_STALE_MS = 60000UL;
static constexpr unsigned long MQTT_STATE_PUBLISH_MS = 5000UL;
bool mqttConnectRequested = false;
static constexpr int MQTT_MAX_BUTTONS = 12;
static constexpr const char *MQTT_CHAT_NAMESPACE = "global";
static constexpr uint16_t MQTT_CLIENT_BUFFER_SIZE = 1400;
static constexpr uint16_t MQTT_CLIENT_BUFFER_IDLE = 256;
static constexpr uint32_t MQTT_MIN_FREE_HEAP_CONNECT = 36000U;
static constexpr uint32_t MQTT_MIN_LARGEST_8BIT_CONNECT = 8192U;
static constexpr uint32_t MQTT_MIN_FREE_HEAP_PUBLISH = 42000U;
static constexpr uint32_t MQTT_MIN_LARGEST_8BIT_PUBLISH = 14000U;
static constexpr unsigned long MQTT_SERVICE_INTERVAL_MS = 15UL;
static constexpr uint16_t MQTT_SOCKET_CONNECT_TIMEOUT_MS = 40U;
int mqttButtonCount = 0;
String mqttButtonNames[MQTT_MAX_BUTTONS];
bool mqttButtonCritical[MQTT_MAX_BUTTONS];
bool mqttButtonToggle[MQTT_MAX_BUTTONS];
bool mqttButtonState[MQTT_MAX_BUTTONS];
bool mqttButtonSelected[MQTT_MAX_BUTTONS];
bool mqttButtonSelectionMode = false;

struct MqttDiscoveredPeer {
    String name;
    String pubKeyHex;
    unsigned long lastSeenMs = 0;
};

static MqttDiscoveredPeer mqttDiscoveredPeers[MAX_MQTT_DISCOVERED];
static int mqttDiscoveredCount = 0;

static int mqttFindDiscoveredByPubKeyHex(const String &pubKeyHex)
{
    for (int i = 0; i < mqttDiscoveredCount; ++i) {
        if (mqttDiscoveredPeers[i].pubKeyHex.equalsIgnoreCase(pubKeyHex)) return i;
    }
    return -1;
}

static void mqttTouchDiscoveredPeer(const String &name, const String &pubKeyHex)
{
    if (pubKeyHex.isEmpty()) return;

    int idx = mqttFindDiscoveredByPubKeyHex(pubKeyHex);
    if (idx < 0) {
        if (mqttDiscoveredCount >= MAX_MQTT_DISCOVERED) {
            idx = 0;
            for (int i = 1; i < mqttDiscoveredCount; ++i) {
                if (mqttDiscoveredPeers[i].lastSeenMs < mqttDiscoveredPeers[idx].lastSeenMs) idx = i;
            }
        } else {
            idx = mqttDiscoveredCount++;
        }
    }

    mqttDiscoveredPeers[idx].name = name;
    mqttDiscoveredPeers[idx].pubKeyHex = pubKeyHex;
    mqttDiscoveredPeers[idx].lastSeenMs = millis();
}

static void mqttPruneDiscoveredPeers()
{
    const unsigned long now = millis();
    for (int i = 0; i < mqttDiscoveredCount; ) {
        if (static_cast<unsigned long>(now - mqttDiscoveredPeers[i].lastSeenMs) > MQTT_DISCOVERY_STALE_MS) {
            for (int j = i + 1; j < mqttDiscoveredCount; ++j) {
                mqttDiscoveredPeers[j - 1] = mqttDiscoveredPeers[j];
            }
            mqttDiscoveredCount--;
        } else {
            ++i;
        }
    }
}

String buildMqttHardwareId()
{
    uint64_t chip = ESP.getEfuseMac();
    char buf[13];
    snprintf(buf, sizeof(buf), "%04X%08X", static_cast<uint16_t>(chip >> 32), static_cast<uint32_t>(chip));
    return String(buf);
}

void mqttPublishPeerPresence()
{
    if (!mqttCfg.enabled || !mqttClient.connected()) return;
    if (mqttChatPresenceTopic.isEmpty()) return;

    JsonDocument doc;
    doc["kind"] = "peer_presence";
    doc["name"] = deviceShortNameValue();
    doc["sender_pub"] = p2pPublicKeyHex();

    String payload;
    serializeJson(doc, payload);

    mqttClient.publish(mqttChatPresenceTopic.c_str(), payload.c_str(), false);
}

void mqttBuildIdentity()
{
    mqttHwId = buildMqttHardwareId();
    mqttNodeId = "esp32_remote_" + mqttHwId;
    mqttClientId = "esp32-remote-" + mqttHwId;
    mqttDeviceName = "ESP32 Remote " + mqttHwId;
    mqttActionTopic = "esp32/remote/" + mqttHwId + "/action";
    mqttChatInboxTopic = "esp32/remote/chat/" + String(MQTT_CHAT_NAMESPACE) + "/inbox/" + p2pPublicKeyHex();
    mqttChatPresenceTopic = "esp32/remote/chat/" + String(MQTT_CHAT_NAMESPACE) + "/presence";
}

String mqttDefaultButtonName(int idx)
{
    if (idx == 0) return "up";
    if (idx == 1) return "down";
    if (idx == 2) return "left";
    if (idx == 3) return "right";
    return "button_" + String(idx + 1);
}

String mqttButtonPayloadForIndex(int idx)
{
    if (idx < 0 || idx >= MQTT_MAX_BUTTONS) return "button";
    String s = mqttButtonNames[idx];
    s.toLowerCase();
    String out;
    for (size_t i = 0; i < s.length(); i++) {
        char c = s[i];
        if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) out += c;
        else if (c == ' ' || c == '-' || c == '_') out += '_';
    }
    while (out.indexOf("__") >= 0) out.replace("__", "_");
    if (out.startsWith("_")) out.remove(0, 1);
    if (out.endsWith("_")) out.remove(out.length() - 1);
    if (out.isEmpty()) out = "button_" + String(idx + 1);
    return out;
}

void loadMqttConfig()
{
    mqttPrefs.begin("mqtt", true);
    mqttCfg.enabled = mqttPrefs.getBool("en", false);
    mqttCfg.broker = mqttPrefs.getString("broker", "homeassistant.local");
    mqttCfg.port = static_cast<uint16_t>(mqttPrefs.getUShort("port", 1883));
    mqttCfg.username = mqttPrefs.getString("user", "");
    mqttCfg.password = mqttPrefs.getString("pass", "");
    mqttCfg.discoveryPrefix = mqttPrefs.getString("disc", "homeassistant");
    mqttButtonCount = mqttPrefs.getInt("btn_count", 0);
    if (mqttButtonCount < 0) mqttButtonCount = 0;
    if (mqttButtonCount > MQTT_MAX_BUTTONS) mqttButtonCount = MQTT_MAX_BUTTONS;
    for (int i = 0; i < MQTT_MAX_BUTTONS; i++) {
        mqttButtonNames[i] = mqttPrefs.getString(("btn_" + String(i)).c_str(), "");
        if (mqttButtonNames[i].length() > 20) mqttButtonNames[i].remove(20);
        if (mqttButtonNames[i].isEmpty()) mqttButtonNames[i] = mqttDefaultButtonName(i);
        mqttButtonCritical[i] = mqttPrefs.getBool(("crit_" + String(i)).c_str(), false);
        mqttButtonToggle[i] = mqttPrefs.getBool(("tgl_" + String(i)).c_str(), false);
        mqttButtonState[i] = mqttPrefs.getBool(("state_" + String(i)).c_str(), false);
        mqttButtonSelected[i] = false;
    }
    mqttPrefs.end();
    if (mqttCfg.broker.isEmpty()) mqttCfg.broker = "homeassistant.local";
    if (mqttCfg.port == 0) mqttCfg.port = 1883;
    if (mqttCfg.discoveryPrefix.isEmpty()) mqttCfg.discoveryPrefix = "homeassistant";
    mqttChatInboxTopic = "esp32/remote/chat/" + String(MQTT_CHAT_NAMESPACE) + "/inbox/" + p2pPublicKeyHex();
}

void saveMqttConfig()
{
    mqttPrefs.begin("mqtt", false);
    mqttPrefs.putBool("en", mqttCfg.enabled);
    mqttPrefs.putString("broker", mqttCfg.broker);
    mqttPrefs.putUShort("port", mqttCfg.port);
    mqttPrefs.putString("user", mqttCfg.username);
    mqttPrefs.putString("pass", mqttCfg.password);
    mqttPrefs.putString("disc", mqttCfg.discoveryPrefix);
    mqttPrefs.putInt("btn_count", mqttButtonCount);
    for (int i = 0; i < MQTT_MAX_BUTTONS; i++) {
        mqttPrefs.putString(("btn_" + String(i)).c_str(), mqttButtonNames[i]);
        mqttPrefs.putBool(("crit_" + String(i)).c_str(), mqttButtonCritical[i]);
        mqttPrefs.putBool(("tgl_" + String(i)).c_str(), mqttButtonToggle[i]);
        mqttPrefs.putBool(("state_" + String(i)).c_str(), mqttButtonState[i]);
    }
    mqttPrefs.end();
}

static void mqttRefreshStatusLabelIfVisible()
{
    if (!lvglReady || !lvglMqttStatusLabel || !lv_obj_is_valid(lvglMqttStatusLabel)) return;
    lv_label_set_text_fmt(lvglMqttStatusLabel, "MQTT: %s", mqttStatusLine.c_str());
}

static bool mqttHasHeapHeadroom(uint32_t freeMin, uint32_t largestMin, const char *status)
{
    const uint32_t freeHeap = static_cast<uint32_t>(ESP.getFreeHeap());
    const uint32_t largest8 = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    if (freeHeap >= freeMin && largest8 >= largestMin) return true;
    mqttStatusLine = String(status ? status : "MQTT low memory") +
                     " (free=" + String(freeHeap / 1024U) + "KB" +
                     ", largest=" + String(largest8 / 1024U) + "KB" +
                     ", need=" + String(freeMin / 1024U) + "/" + String(largestMin / 1024U) + "KB)";
    Serial.printf("[MQTT] skipped low memory free=%lu largest=%lu need_free=%lu need_largest=%lu\n",
                  static_cast<unsigned long>(freeHeap),
                  static_cast<unsigned long>(largest8),
                  static_cast<unsigned long>(freeMin),
                  static_cast<unsigned long>(largestMin));
    return false;
}

static bool mqttPublishEncryptedPayload(const String &peerKey, const uint8_t *plain, size_t plainLen)
{
    if (!mqttCfg.enabled || !mqttClient.connected() || peerKey.isEmpty() || !plain || plainLen == 0) return false;
    if (plainLen > (P2P_MAX_PACKET - 64U)) return false;
    if (!mqttHasHeapHeadroom(MQTT_MIN_FREE_HEAP_PUBLISH, MQTT_MIN_LARGEST_8BIT_PUBLISH, "MQTT low memory")) return false;

    const int peerIdx = p2pFindPeerByPubKeyHex(peerKey);
    if (peerIdx < 0 || !p2pPeers[peerIdx].enabled) return false;

    unsigned char peerPk[P2P_PUBLIC_KEY_BYTES] = {0};
    if (!p2pHexToBytes(p2pPeers[peerIdx].pubKeyHex, peerPk, sizeof(peerPk))) return false;

    unsigned char nonce[P2P_NONCE_BYTES] = {0};
    unsigned char cipher[P2P_MAX_PACKET] = {0};
    randombytes_buf(nonce, sizeof(nonce));
    if (crypto_box_curve25519xchacha20poly1305_easy(cipher, plain, plainLen, nonce, peerPk, p2pSecretKey) != 0) return false;

    const size_t cipherLen = plainLen + P2P_MAC_BYTES;
    char senderPubHex[(P2P_PUBLIC_KEY_BYTES * 2U) + 1U] = {0};
    char nonceHex[(P2P_NONCE_BYTES * 2U) + 1U] = {0};
    char cipherHex[(P2P_MAX_PACKET * 2U) + 1U] = {0};
    char payload[(P2P_MAX_PACKET * 2U) + 180U] = {0};
    sodium_bin2hex(senderPubHex, sizeof(senderPubHex), p2pPublicKey, sizeof(p2pPublicKey));
    sodium_bin2hex(nonceHex, sizeof(nonceHex), nonce, sizeof(nonce));
    sodium_bin2hex(cipherHex, sizeof(cipherHex), cipher, cipherLen);
    const int written = snprintf(payload,
                                 sizeof(payload),
                                 "{\"sender_pub\":\"%s\",\"nonce\":\"%s\",\"cipher\":\"%s\"}",
                                 senderPubHex,
                                 nonceHex,
                                 cipherHex);
    if (written <= 0 || static_cast<size_t>(written) >= sizeof(payload)) return false;

    const String peerTopic = "esp32/remote/chat/" + String(MQTT_CHAT_NAMESPACE) + "/inbox/" + p2pPeers[peerIdx].pubKeyHex;
    return mqttClient.publish(peerTopic.c_str(), reinterpret_cast<const uint8_t *>(payload), static_cast<unsigned int>(written), false);
}

void mqttPublishAction(const char *action)
{
    if (!mqttCfg.enabled || !mqttClient.connected()) return;
    mqttClient.publish(mqttActionTopic.c_str(), action, false);
}

bool mqttPublishChatMessageWithId(const String &peerKey, const String &text, const String &messageId)
{
    if (!mqttCfg.enabled || !mqttClient.connected() || peerKey.isEmpty() || text.isEmpty() || messageId.isEmpty()) return false;
    JsonDocument plainDoc;
    plainDoc["kind"] = "chat";
    plainDoc["id"] = messageId;
    plainDoc["author"] = deviceShortNameValue();
    plainDoc["text"] = text.substring(0, P2P_MAX_CHAT_TEXT);
    char plain[P2P_MAX_PACKET] = {0};
    const size_t plainLen = serializeJson(plainDoc, plain, sizeof(plain));
    if (plainLen == 0) return false;
    return mqttPublishEncryptedPayload(peerKey, reinterpret_cast<const uint8_t *>(plain), plainLen);
}

bool mqttPublishConversationDelete()
{
    if (!mqttCfg.enabled || !mqttClient.connected() || currentChatPeerKey.isEmpty()) return false;
    JsonDocument plainDoc;
    plainDoc["kind"] = "delete_conversation";
    plainDoc["author"] = deviceShortNameValue();
    char plain[P2P_MAX_PACKET] = {0};
    const size_t plainLen = serializeJson(plainDoc, plain, sizeof(plain));
    if (plainLen == 0) return false;
    return mqttPublishEncryptedPayload(currentChatPeerKey, reinterpret_cast<const uint8_t *>(plain), plainLen);
}

bool mqttPublishMessageDelete(const String &peerKey, const String &messageId)
{
    if (!mqttCfg.enabled || !mqttClient.connected() || peerKey.isEmpty() || messageId.isEmpty()) return false;
    JsonDocument plainDoc;
    plainDoc["kind"] = "delete_message";
    plainDoc["id"] = messageId;
    plainDoc["author"] = deviceShortNameValue();
    char plain[P2P_MAX_PACKET] = {0};
    const size_t plainLen = serializeJson(plainDoc, plain, sizeof(plain));
    if (plainLen == 0) return false;
    return mqttPublishEncryptedPayload(peerKey, reinterpret_cast<const uint8_t *>(plain), plainLen);
}

bool mqttPublishChatAck(const String &peerKey, const String &messageId)
{
    if (!mqttCfg.enabled || !mqttClient.connected() || peerKey.isEmpty() || messageId.isEmpty()) return false;
    JsonDocument plainDoc;
    plainDoc["kind"] = "ack";
    plainDoc["ack_id"] = messageId;
    char plain[P2P_MAX_PACKET] = {0};
    const size_t plainLen = serializeJson(plainDoc, plain, sizeof(plain));
    if (plainLen == 0) return false;
    return mqttPublishEncryptedPayload(peerKey, reinterpret_cast<const uint8_t *>(plain), plainLen);
}

static String mqttAvailabilityTopic()
{
    return "esp32/remote/" + mqttHwId + "/availability";
}

static String mqttStateTopic()
{
    return "esp32/remote/" + mqttHwId + "/state";
}

String mqttButtonCommandTopic(int idx)
{
    return "esp32/remote/" + mqttHwId + "/control/" + String(idx) + "/set";
}

static String mqttButtonStateTopic(int idx)
{
    return "esp32/remote/" + mqttHwId + "/control/" + String(idx) + "/state";
}

static String mqttHaChatCommandTopic()
{
    return "esp32/remote/" + mqttHwId + "/chat/ha/set";
}

String mqttHaChatReceivedTopic()
{
    return "esp32/remote/" + mqttHwId + "/chat/ha/received";
}

String mqttHaChatReceivedCommandTopic()
{
    return "esp32/remote/" + mqttHwId + "/chat/ha/received/set";
}

static bool mqttParseBoolPayload(const String &payload, bool currentValue, bool &parsedValue)
{
    String normalized = payload;
    normalized.trim();
    normalized.toUpperCase();
    if (normalized == "ON" || normalized == "1" || normalized == "TRUE") {
        parsedValue = true;
        return true;
    }
    if (normalized == "OFF" || normalized == "0" || normalized == "FALSE") {
        parsedValue = false;
        return true;
    }
    if (normalized == "TOGGLE") {
        parsedValue = !currentValue;
        return true;
    }
    return false;
}

static String mqttDiscoveryDeviceName()
{
    const String name = deviceShortNameValue();
    return name.isEmpty() ? mqttDeviceName : name;
}

static void mqttFillDiscoveryDevice(JsonObject dev)
{
    JsonArray ids = dev["identifiers"].to<JsonArray>();
    ids.add(mqttNodeId);
    dev["name"] = mqttDiscoveryDeviceName();
    dev["model"] = DEVICE_MODEL;
    dev["manufacturer"] = "Elik";
    dev["sw_version"] = FW_VERSION;
    if (wifiConnectedSafe()) dev["configuration_url"] = "http://" + wifiIpSafe();
}

static bool mqttPublishDiscoveryConfig(const String &topic, JsonDocument &doc)
{
    String payload;
    serializeJson(doc, payload);
    return mqttClient.publish(topic.c_str(), payload.c_str(), true);
}

void mqttPublishStateIfNeeded(bool force)
{
    if (!mqttClient.connected()) return;

    const unsigned long now = millis();
    if (!force && static_cast<unsigned long>(now - mqttLastStatePublishMs) < MQTT_STATE_PUBLISH_MS) return;

    const bool wifiConnected = wifiConnectedSafe();
    const int32_t wifiRssi = wifiConnected ? WiFi.RSSI() : -127;
    const uint8_t wifiQuality = wifiConnected ? wifiQualityPercentFromRssi(wifiRssi) : 0;
    const bool unreadMessages = chatHasUnreadMessages();
    const uint32_t freeHeapKb = static_cast<uint32_t>(ESP.getFreeHeap() / 1024U);
    const uint32_t largest8Kb = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) / 1024U);

    JsonDocument doc;
    doc["device_name"] = mqttDiscoveryDeviceName();
    doc["firmware"] = FW_VERSION;
    doc["model"] = DEVICE_MODEL;
    doc["battery_percent"] = batteryPercent;
    doc["battery_voltage"] = roundf(batteryVoltage * 100.0f) / 100.0f;
    doc["battery_charging"] = batteryCharging;
    doc["wifi_connected"] = wifiConnected;
    doc["wifi_rssi"] = wifiRssi;
    doc["wifi_quality"] = wifiQuality;
    doc["wifi_ssid"] = wifiConnected ? wifiSsidSafe() : String("");
    doc["ip_address"] = wifiConnected ? wifiIpSafe() : String("");
    doc["illumination"] = lightPercent;
    doc["display_brightness"] = displayBrightnessPercent;
    doc["rgb_led_brightness"] = rgbLedPercent;
    doc["airplane_mode"] = airplaneModeEnabled;
    doc["ap_mode"] = apModeActive;
    doc["web_server"] = webServerEnabled;
    doc["messages_waiting"] = unreadMessages;
    doc["queued_messages"] = chatPendingCount;
    doc["paired_peers"] = p2pPeerCount;
    doc["radio_peers"] = hc12DiscoveredCount;
    doc["mqtt_peers"] = mqttDiscoveredCount;
    doc["status"] = uiStatusLine;
    doc["uptime_s"] = now / 1000UL;
    doc["free_heap_kb"] = freeHeapKb;
    doc["largest_block_kb"] = largest8Kb;

    String payload;
    serializeJson(doc, payload);
    if (!force && payload == mqttLastStatePayload) return;

    mqttLastStatePayload = payload;
    mqttLastStatePublishMs = now;
    mqttClient.publish(mqttStateTopic().c_str(), payload.c_str(), true);
}

void mqttPublishButtonState(int index)
{
    if (!mqttClient.connected() || index < 0 || index >= mqttButtonCount || !mqttButtonToggle[index]) return;
    mqttClient.publish(mqttButtonStateTopic(index).c_str(), mqttButtonState[index] ? "ON" : "OFF", true);
}

void mqttHandleButtonCommand(int index, const String &payload)
{
    if (index < 0 || index >= mqttButtonCount) return;

    if (mqttButtonToggle[index]) {
        bool nextState = mqttButtonState[index];
        if (!mqttParseBoolPayload(payload, mqttButtonState[index], nextState)) return;
        mqttButtonState[index] = nextState;
        saveMqttConfig();
        mqttPublishButtonState(index);

        const String baseAction = mqttButtonPayloadForIndex(index);
        const String action = baseAction + (nextState ? "_on" : "_off");
        mqttPublishAction(action.c_str());
        lvglStatusPush("Action: " + action);
    } else {
        const String action = mqttButtonPayloadForIndex(index);
        mqttPublishAction(action.c_str());
        lvglStatusPush("Action: " + action);
    }

    if (lvglReady) lvglRefreshMqttControlsUi();
    mqttPublishStateIfNeeded(true);
}

void mqttHandleHaChatCommand(const String &payload)
{
    String text = payload;
    text.trim();
    if (text.isEmpty()) return;
    if (text.length() > P2P_MAX_CHAT_TEXT) text.remove(P2P_MAX_CHAT_TEXT);

    const String peerKey = "ha_inbox";
    const String messageId = "ha_" + chatGenerateMessageId();
    chatStoreMessage(peerKey, "Home Assistant", text, false, CHAT_TRANSPORT_MQTT, messageId);
    uiStatusLine = "HA chat from Home Assistant";
    if (lvglReady) {
        lvglSyncStatusLine();
        lvglRefreshChatContactsUi();
        if (uiScreen == UI_CHAT && currentChatPeerKey == peerKey) lvglRefreshChatUi();
    }
    mqttPublishStateIfNeeded(true);
}

void mqttPublishDiscovery()
{
    if (!mqttClient.connected()) return;
    if (!mqttHasHeapHeadroom(MQTT_MIN_FREE_HEAP_PUBLISH, MQTT_MIN_LARGEST_8BIT_PUBLISH, "MQTT discovery deferred")) return;
    const String availTopic = mqttAvailabilityTopic();
    const String stateTopic = mqttStateTopic();
    const String deviceName = mqttDiscoveryDeviceName();
    String discoveryPrefixes[2];
    int prefixCount = 0;
    discoveryPrefixes[prefixCount++] = mqttCfg.discoveryPrefix.length() ? mqttCfg.discoveryPrefix : String("homeassistant");
    if (discoveryPrefixes[0] != "homeassistant") discoveryPrefixes[prefixCount++] = "homeassistant";

    for (int prefixIdx = 0; prefixIdx < prefixCount; ++prefixIdx) {
        const String discoveryBase = discoveryPrefixes[prefixIdx] + "/";
        auto publishSensor = [&](const char *component,
                                 const String &objId,
                                 const String &name,
                                 const char *valueTemplate,
                                 const char *unit,
                                 const char *deviceClass,
                                 const char *stateClass,
                                 const char *entityCategory) {
            JsonDocument doc;
            doc["name"] = name;
            doc["unique_id"] = mqttNodeId + "_" + objId;
            doc["state_topic"] = stateTopic;
            doc["availability_topic"] = availTopic;
            doc["value_template"] = valueTemplate;
            if (unit && unit[0] != '\0') doc["unit_of_measurement"] = unit;
            if (deviceClass && deviceClass[0] != '\0') doc["device_class"] = deviceClass;
            if (stateClass && stateClass[0] != '\0') doc["state_class"] = stateClass;
            if (entityCategory && entityCategory[0] != '\0') doc["entity_category"] = entityCategory;
            mqttFillDiscoveryDevice(doc["device"].to<JsonObject>());
            mqttPublishDiscoveryConfig(discoveryBase + component + "/" + mqttNodeId + "/" + objId + "/config", doc);
        };
        auto publishBinarySensor = [&](const String &objId,
                                       const String &name,
                                       const char *valueTemplate,
                                       const char *deviceClass,
                                       const char *entityCategory) {
            JsonDocument doc;
            doc["name"] = name;
            doc["unique_id"] = mqttNodeId + "_" + objId;
            doc["state_topic"] = stateTopic;
            doc["availability_topic"] = availTopic;
            doc["value_template"] = valueTemplate;
            doc["payload_on"] = "ON";
            doc["payload_off"] = "OFF";
            if (deviceClass && deviceClass[0] != '\0') doc["device_class"] = deviceClass;
            if (entityCategory && entityCategory[0] != '\0') doc["entity_category"] = entityCategory;
            mqttFillDiscoveryDevice(doc["device"].to<JsonObject>());
            mqttPublishDiscoveryConfig(discoveryBase + "binary_sensor/" + mqttNodeId + "/" + objId + "/config", doc);
        };

        publishSensor("sensor", "battery_percent", deviceName + " Battery", "{{ value_json.battery_percent }}", "%", "battery", "measurement", nullptr);
        publishSensor("sensor", "battery_voltage", deviceName + " Battery Voltage", "{{ value_json.battery_voltage }}", "V", "voltage", "measurement", "diagnostic");
        publishBinarySensor("battery_charging", deviceName + " Charging", "{{ 'ON' if value_json.battery_charging else 'OFF' }}", "battery_charging", "diagnostic");
        publishBinarySensor("wifi_connected", deviceName + " WiFi", "{{ 'ON' if value_json.wifi_connected else 'OFF' }}", "connectivity", "diagnostic");
        publishSensor("sensor", "wifi_quality", deviceName + " WiFi Quality", "{{ value_json.wifi_quality }}", "%", nullptr, "measurement", "diagnostic");
        publishSensor("sensor", "wifi_rssi", deviceName + " WiFi RSSI", "{{ value_json.wifi_rssi }}", "dBm", "signal_strength", "measurement", "diagnostic");
        publishSensor("sensor", "illumination", deviceName + " Illumination", "{{ value_json.illumination }}", "%", nullptr, "measurement", nullptr);
        publishSensor("sensor", "display_brightness", deviceName + " Display Brightness", "{{ value_json.display_brightness }}", "%", nullptr, "measurement", nullptr);
        publishSensor("sensor", "rgb_led_brightness", deviceName + " RGB LED Brightness", "{{ value_json.rgb_led_brightness }}", "%", nullptr, "measurement", nullptr);
        publishBinarySensor("messages_waiting", deviceName + " Messages Waiting", "{{ 'ON' if value_json.messages_waiting else 'OFF' }}", "problem", nullptr);
        publishSensor("sensor", "queued_messages", deviceName + " Queued Messages", "{{ value_json.queued_messages }}", nullptr, nullptr, "measurement", "diagnostic");
        publishSensor("sensor", "paired_peers", deviceName + " Paired Peers", "{{ value_json.paired_peers }}", nullptr, nullptr, "measurement", "diagnostic");
        publishSensor("sensor", "radio_peers", deviceName + " Radio Peers", "{{ value_json.radio_peers }}", nullptr, nullptr, "measurement", "diagnostic");
        publishSensor("sensor", "mqtt_peers", deviceName + " MQTT Peers", "{{ value_json.mqtt_peers }}", nullptr, nullptr, "measurement", "diagnostic");
        publishBinarySensor("airplane_mode", deviceName + " Airplane Mode", "{{ 'ON' if value_json.airplane_mode else 'OFF' }}", nullptr, nullptr);
        publishBinarySensor("ap_mode", deviceName + " AP Mode", "{{ 'ON' if value_json.ap_mode else 'OFF' }}", nullptr, nullptr);
        publishBinarySensor("web_server", deviceName + " Web Server", "{{ 'ON' if value_json.web_server else 'OFF' }}", nullptr, "diagnostic");
        publishSensor("sensor", "status", deviceName + " Status", "{{ value_json.status }}", nullptr, nullptr, nullptr, nullptr);
        publishSensor("sensor", "wifi_ssid", deviceName + " WiFi SSID", "{{ value_json.wifi_ssid }}", nullptr, nullptr, nullptr, "diagnostic");
        publishSensor("sensor", "ip_address", deviceName + " IP Address", "{{ value_json.ip_address }}", nullptr, nullptr, nullptr, "diagnostic");
        publishSensor("sensor", "uptime_s", deviceName + " Uptime", "{{ value_json.uptime_s }}", "s", nullptr, "total_increasing", "diagnostic");
        publishSensor("sensor", "free_heap_kb", deviceName + " Free Heap", "{{ value_json.free_heap_kb }}", "kB", nullptr, "measurement", "diagnostic");
        publishSensor("sensor", "largest_block_kb", deviceName + " Largest Free Block", "{{ value_json.largest_block_kb }}", "kB", nullptr, "measurement", "diagnostic");

        JsonDocument textDoc;
        textDoc["name"] = "Send Chat Message";
        textDoc["unique_id"] = mqttNodeId + "_ha_chat_send";
        textDoc["availability_topic"] = availTopic;
        textDoc["command_topic"] = mqttHaChatCommandTopic();
        textDoc["mode"] = "text";
        textDoc["max"] = P2P_MAX_CHAT_TEXT;
        textDoc["icon"] = "mdi:message-text-outline";
        mqttFillDiscoveryDevice(textDoc["device"].to<JsonObject>());
        mqttPublishDiscoveryConfig(discoveryBase + "text/" + mqttNodeId + "/ha_chat_send/config", textDoc);

        JsonDocument recvDoc;
        recvDoc["name"] = "Received Chat Messages";
        recvDoc["unique_id"] = mqttNodeId + "_ha_chat_received";
        recvDoc["availability_topic"] = availTopic;
        recvDoc["state_topic"] = mqttHaChatReceivedTopic();
        recvDoc["command_topic"] = mqttHaChatReceivedCommandTopic();
        recvDoc["mode"] = "text";
        recvDoc["max"] = P2P_MAX_CHAT_TEXT;
        recvDoc["icon"] = "mdi:message-reply-text-outline";
        mqttFillDiscoveryDevice(recvDoc["device"].to<JsonObject>());
        mqttPublishDiscoveryConfig(discoveryBase + "text/" + mqttNodeId + "/ha_chat_received/config", recvDoc);

        for (int i = mqttButtonCount; i < MQTT_MAX_BUTTONS; i++) {
            const String oldTopic = discoveryPrefixes[prefixIdx] + "/device_automation/" + mqttNodeId + "/" + String(i) + "/config";
            mqttClient.publish(oldTopic.c_str(), "", true);
            const String oldButtonTopic = discoveryPrefixes[prefixIdx] + "/button/" + mqttNodeId + "/mqtt_control_" + String(i) + "/config";
            mqttClient.publish(oldButtonTopic.c_str(), "", true);
            const String oldSwitchTopic = discoveryPrefixes[prefixIdx] + "/switch/" + mqttNodeId + "/mqtt_control_" + String(i) + "/config";
            mqttClient.publish(oldSwitchTopic.c_str(), "", true);
        }
        for (int i = 0; i < mqttButtonCount; i++) {
            const String action = mqttButtonPayloadForIndex(i);
            const String topic = discoveryPrefixes[prefixIdx] + "/device_automation/" + mqttNodeId + "/" + String(i) + "/config";
            JsonDocument doc;
            doc["automation_type"] = "trigger";
            doc["topic"] = mqttActionTopic;
            doc["payload"] = action;
            doc["type"] = "button_short_press";
            doc["subtype"] = "button_" + String(i + 1);
            doc["unique_id"] = mqttNodeId + "_" + String(i + 1) + "_" + action;
            doc["availability_topic"] = availTopic;
            mqttFillDiscoveryDevice(doc["device"].to<JsonObject>());
            mqttPublishDiscoveryConfig(topic, doc);
            const String buttonCfgTopic = discoveryPrefixes[prefixIdx] + "/button/" + mqttNodeId + "/mqtt_control_" + String(i) + "/config";
            const String switchCfgTopic = discoveryPrefixes[prefixIdx] + "/switch/" + mqttNodeId + "/mqtt_control_" + String(i) + "/config";
            mqttClient.publish(buttonCfgTopic.c_str(), "", true);
            mqttClient.publish(switchCfgTopic.c_str(), "", true);

            if (mqttButtonToggle[i]) {
                JsonDocument swDoc;
                swDoc["name"] = mqttButtonNames[i];
                swDoc["unique_id"] = mqttNodeId + "_mqtt_control_" + String(i);
                swDoc["availability_topic"] = availTopic;
                swDoc["command_topic"] = mqttButtonCommandTopic(i);
                swDoc["state_topic"] = mqttButtonStateTopic(i);
                swDoc["payload_on"] = "ON";
                swDoc["payload_off"] = "OFF";
                swDoc["state_on"] = "ON";
                swDoc["state_off"] = "OFF";
                swDoc["icon"] = mqttButtonState[i] ? "mdi:toggle-switch" : "mdi:toggle-switch-off-outline";
                mqttFillDiscoveryDevice(swDoc["device"].to<JsonObject>());
                mqttPublishDiscoveryConfig(switchCfgTopic, swDoc);
            } else {
                JsonDocument btnDoc;
                btnDoc["name"] = mqttButtonNames[i];
                btnDoc["unique_id"] = mqttNodeId + "_mqtt_control_" + String(i);
                btnDoc["availability_topic"] = availTopic;
                btnDoc["command_topic"] = mqttButtonCommandTopic(i);
                btnDoc["payload_press"] = "PRESS";
                btnDoc["icon"] = mqttButtonCritical[i] ? "mdi:alert-circle-outline" : "mdi:gesture-tap-button";
                mqttFillDiscoveryDevice(btnDoc["device"].to<JsonObject>());
                mqttPublishDiscoveryConfig(buttonCfgTopic, btnDoc);
            }
        }
    }
    mqttDiscoveryPublished = true;
    mqttStatusLine = "HA discovery published";
    for (int i = 0; i < mqttButtonCount; ++i) mqttPublishButtonState(i);
    mqttClient.publish(mqttHaChatReceivedTopic().c_str(), "", true);
    mqttPublishStateIfNeeded(true);
}

void lvglMqttPublishDiscoveryEvent(lv_event_t *e)
{
    (void)e;
    if (!mqttClient.connected()) {
        lvglStatusPush("MQTT not connected");
        return;
    }
    mqttPublishDiscovery();
    lv_label_set_text_fmt(lvglMqttStatusLabel, "MQTT: %s", mqttStatusLine.c_str());
}

void lvglMqttConnectEvent(lv_event_t *e)
{
    if (e) {
        lv_obj_t *btn = lv_event_get_current_target(e);
        if (btn && lv_obj_is_valid(btn)) {
            lv_obj_clear_state(btn, LV_STATE_CHECKED | LV_STATE_PRESSED);
        }
    }
    mqttCfg.enabled = true;
    saveMqttConfig();
    if (lvglMqttEnableSw) {
        lv_obj_add_state(lvglMqttEnableSw, LV_STATE_CHECKED);
        lv_obj_clear_state(lvglMqttEnableSw, 0);
    }
    mqttStatusLine = "Connecting...";
    if (lvglMqttStatusLabel) lv_label_set_text_fmt(lvglMqttStatusLabel, "MQTT: %s", mqttStatusLine.c_str());
    lvglStatusPush("MQTT connecting...");
    mqttConnectRequested = true;
    mqttLastReconnectMs = 0;
}

void lvglMqttEnableEvent(lv_event_t *e)
{
    (void)e;
    if (!lvglMqttEnableSw) return;
    mqttCfg.enabled = lv_obj_has_state(lvglMqttEnableSw, LV_STATE_CHECKED);
    saveMqttConfig();
    if (!mqttCfg.enabled) {
        if (mqttClient.connected()) mqttClient.disconnect();
        mqttTrimBufferForIdle();
        mqttStatusLine = "Disabled";
        lvglStatusPush("MQTT disabled");
    } else {
        mqttStatusLine = "Enabled";
        lvglStatusPush("MQTT enabled");
    }
    if (lvglReady) lvglRefreshTopIndicators();
}

void lvglMqttSaveEvent(lv_event_t *e)
{
    if (e) {
        lv_obj_t *btn = lv_event_get_current_target(e);
        if (btn && lv_obj_is_valid(btn)) {
            lv_obj_clear_state(btn, LV_STATE_CHECKED | LV_STATE_PRESSED);
        }
    }
    mqttCfg.enabled = lv_obj_has_state(lvglMqttEnableSw, LV_STATE_CHECKED);
    mqttCfg.broker = lv_textarea_get_text(lvglMqttBrokerTa);
    mqttCfg.username = lv_textarea_get_text(lvglMqttUserTa);
    mqttCfg.password = lv_textarea_get_text(lvglMqttPassTa);
    mqttCfg.discoveryPrefix = lv_textarea_get_text(lvglMqttDiscTa);
    mqttChatInboxTopic = "esp32/remote/chat/" + String(MQTT_CHAT_NAMESPACE) + "/inbox/" + p2pPublicKeyHex();

    int p = atoi(lv_textarea_get_text(lvglMqttPortTa));
    if (p <= 0 || p > 65535) p = 1883;
    mqttCfg.port = static_cast<uint16_t>(p);
    lv_textarea_set_text(lvglMqttPortTa, String(mqttCfg.port).c_str());

    saveMqttConfig();
    mqttStatusLine = mqttCfg.enabled ? "Enabled" : "Disabled";
    if (mqttCfg.enabled) {
        mqttConnectRequested = true;
        mqttLastReconnectMs = 0;
    }
    lv_label_set_text_fmt(lvglMqttStatusLabel, "MQTT: %s", mqttStatusLine.c_str());
    lvglStatusPush("MQTT settings saved");
}

static void syncMqttButtonConfigFromUi()
{
    if (!lvglMqttButtonConfigList || !lv_obj_is_valid(lvglMqttButtonConfigList)) return;
    const uint32_t childCount = lv_obj_get_child_cnt(lvglMqttButtonConfigList);
    for (uint32_t i = 0; i < childCount && i < static_cast<uint32_t>(mqttButtonCount); ++i) {
        lv_obj_t *row = lv_obj_get_child(lvglMqttButtonConfigList, i);
        if (!row) continue;
        lv_obj_t *selCb = lv_obj_get_child(row, 0);
        lv_obj_t *nameTa = lv_obj_get_child(row, 1);
        lv_obj_t *critCb = lv_obj_get_child(row, 2);
        lv_obj_t *toggleCb = lv_obj_get_child(row, 3);
        if (selCb) mqttButtonSelected[i] = lv_obj_has_state(selCb, LV_STATE_CHECKED);
        if (nameTa) {
            mqttButtonNames[i] = lv_textarea_get_text(nameTa);
            mqttButtonNames[i].trim();
            if (mqttButtonNames[i].length() > 20) mqttButtonNames[i].remove(20);
            if (mqttButtonNames[i].isEmpty()) mqttButtonNames[i] = mqttDefaultButtonName(static_cast<int>(i));
        }
        if (critCb) mqttButtonCritical[i] = lv_obj_has_state(critCb, LV_STATE_CHECKED);
        if (toggleCb) mqttButtonToggle[i] = lv_obj_has_state(toggleCb, LV_STATE_CHECKED);
    }
}

void lvglMqttButtonFieldChangedEvent(lv_event_t *e)
{
    (void)e;
    syncMqttButtonConfigFromUi();
    mqttDiscoveryPublished = false;
    saveMqttConfig();
    lvglRefreshMqttControlsUi();
}

void lvglMqttButtonAddEvent(lv_event_t *e)
{
    (void)e;
    syncMqttButtonConfigFromUi();
    if (mqttButtonCount >= MQTT_MAX_BUTTONS) {
        lvglStatusPush("MQTT button limit reached");
        return;
    }
    const int idx = mqttButtonCount++;
    mqttButtonNames[idx] = (idx > 0) ? mqttButtonNames[idx - 1] : (String("Button ") + String(idx + 1));
    mqttButtonCritical[idx] = false;
    mqttButtonToggle[idx] = false;
    mqttButtonState[idx] = false;
    mqttButtonSelected[idx] = false;
    mqttDiscoveryPublished = false;
    saveMqttConfig();
    lvglRefreshMqttButtonConfigUi();
    lvglRefreshMqttControlsUi();

    if (lvglMqttButtonConfigList && lv_obj_is_valid(lvglMqttButtonConfigList)) {
        lv_obj_t *row = lv_obj_get_child(lvglMqttButtonConfigList, mqttButtonCount - 1);
        if (row && lv_obj_is_valid(row)) {
            lv_obj_t *nameTa = lv_obj_get_child(row, 1);
            if (nameTa && lv_obj_is_valid(nameTa)) {
                lvglMqttBtnNameTa = nameTa;
                lv_obj_scroll_to_view_recursive(nameTa, LV_ANIM_ON);
                lv_obj_add_state(nameTa, LV_STATE_FOCUSED);
                lv_group_focus_obj(nameTa);
            }
        }
    }
}

void lvglMqttButtonSelectModeEvent(lv_event_t *e)
{
    (void)e;
    mqttButtonSelectionMode = !mqttButtonSelectionMode;
    if (!mqttButtonSelectionMode) {
        for (int i = 0; i < mqttButtonCount; ++i) mqttButtonSelected[i] = false;
    }
    lvglRefreshMqttButtonConfigUi();
}

void lvglMqttButtonDeletePromptEvent(lv_event_t *e)
{
    (void)e;
    syncMqttButtonConfigFromUi();
    int selectedCount = 0;
    for (int i = 0; i < mqttButtonCount; ++i) {
        if (mqttButtonSelected[i]) ++selectedCount;
    }
    if (selectedCount <= 0) {
        lvglStatusPush("Select MQTT buttons to delete");
        return;
    }
    static const char *btns[] = {"No", "Yes", ""};
    lv_obj_t *m = lv_msgbox_create(nullptr, "Delete Buttons", "Delete selected MQTT buttons?", btns, false);
    if (!m) return;
    lvglApplyMsgboxModalStyle(m);
    lv_obj_center(m);
    lv_obj_add_event_cb(m, lvglMqttButtonDeleteConfirmEvent, LV_EVENT_VALUE_CHANGED, nullptr);
}

void lvglMqttButtonDeleteConfirmEvent(lv_event_t *e)
{
    lv_obj_t *msgbox = lv_event_get_current_target(e);
    const char *txt = lv_msgbox_get_active_btn_text(msgbox);
    if (txt && strcmp(txt, "Yes") == 0) {
        syncMqttButtonConfigFromUi();
        int writeIdx = 0;
        for (int i = 0; i < mqttButtonCount; ++i) {
            if (mqttButtonSelected[i]) continue;
            if (writeIdx != i) {
                mqttButtonNames[writeIdx] = mqttButtonNames[i];
                mqttButtonCritical[writeIdx] = mqttButtonCritical[i];
                mqttButtonToggle[writeIdx] = mqttButtonToggle[i];
                mqttButtonState[writeIdx] = mqttButtonState[i];
            }
            mqttButtonSelected[writeIdx] = false;
            ++writeIdx;
        }
        for (int i = writeIdx; i < MQTT_MAX_BUTTONS; ++i) {
            mqttButtonNames[i] = mqttDefaultButtonName(i);
            mqttButtonCritical[i] = false;
            mqttButtonToggle[i] = false;
            mqttButtonState[i] = false;
            mqttButtonSelected[i] = false;
        }
        mqttButtonCount = writeIdx;
        mqttButtonSelectionMode = false;
        mqttDiscoveryPublished = false;
        saveMqttConfig();
        lvglRefreshMqttButtonConfigUi();
        lvglRefreshMqttControlsUi();
    }
    lv_msgbox_close(msgbox);
}

void lvglMqttControlConfirmEvent(lv_event_t *e)
{
    lv_obj_t *msgbox = lv_event_get_current_target(e);
    const char *txt = lv_msgbox_get_active_btn_text(msgbox);
    int idx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (txt && strcmp(txt, "Yes") == 0 && idx >= 0 && idx < mqttButtonCount) {
        mqttHandleButtonCommand(idx, mqttButtonToggle[idx] ? String("TOGGLE") : String("PRESS"));
    }
    lv_msgbox_close(msgbox);
}

void lvglMqttControlPressEvent(lv_event_t *e)
{
    const int idx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (idx < 0 || idx >= mqttButtonCount) return;
    if (mqttButtonCritical[idx]) {
        static const char *btns[] = {"No", "Yes", ""};
        lv_obj_t *m = lv_msgbox_create(nullptr, "Confirm", mqttButtonNames[idx].c_str(), btns, false);
        if (!m) return;
        lvglApplyMsgboxModalStyle(m);
        lv_obj_center(m);
        lv_obj_add_event_cb(m, lvglMqttControlConfirmEvent, LV_EVENT_VALUE_CHANGED, reinterpret_cast<void *>(static_cast<intptr_t>(idx)));
    } else {
        mqttHandleButtonCommand(idx, mqttButtonToggle[idx] ? String("TOGGLE") : String("PRESS"));
    }
}

void lvglRefreshMqttConfigUi()
{
    if (!lvglMqttEnableSw) return;
    const String displayStatus = !mqttCfg.enabled ? "Disabled"
                                : (mqttClient.connected() ? String("Connected")
                                                          : (mqttConnectRequested ? String("Connecting...") : mqttStatusLine));
    lv_obj_add_state(lvglMqttEnableSw, mqttCfg.enabled ? LV_STATE_CHECKED : 0);
    lv_obj_clear_state(lvglMqttEnableSw, mqttCfg.enabled ? 0 : LV_STATE_CHECKED);
    lv_textarea_set_text(lvglMqttBrokerTa, mqttCfg.broker.c_str());
    lv_textarea_set_text(lvglMqttPortTa, String(mqttCfg.port).c_str());
    lv_textarea_set_text(lvglMqttUserTa, mqttCfg.username.c_str());
    lv_textarea_set_text(lvglMqttPassTa, mqttCfg.password.c_str());
    lv_textarea_set_text(lvglMqttDiscTa, mqttCfg.discoveryPrefix.c_str());
    lv_label_set_text_fmt(lvglMqttStatusLabel, "MQTT: %s", displayStatus.c_str());
}

void lvglRefreshMqttButtonConfigUi()
{
    if (!lvglMqttButtonConfigList || !lv_obj_is_valid(lvglMqttButtonConfigList)) return;
    lv_obj_t *topRow = lv_obj_get_child(lv_obj_get_parent(lvglMqttButtonConfigList), 0);
    if (topRow) {
        lv_obj_t *addBtn = lv_obj_get_child(topRow, 0);
        lv_obj_t *selBtn = lv_obj_get_child(topRow, 1);
        lv_obj_t *delBtn = lv_obj_get_child(topRow, 2);
        if (addBtn) lvglRegisterStyledButton(addBtn, lv_color_hex(0x2F7A45), true);
        if (selBtn) lvglRegisterPersistentStateButtonColors(selBtn, lv_color_hex(0x2F6D86), lv_color_hex(0x8A5A25), mqttButtonSelectionMode, true);
        if (delBtn) lvglRegisterStyledButton(delBtn, lv_color_hex(0x8A3A3A), true);
    }

    lv_obj_clean(lvglMqttButtonConfigList);
    for (int i = 0; i < mqttButtonCount; ++i) {
        lv_obj_t *row = lv_obj_create(lvglMqttButtonConfigList);
        lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_style_bg_color(row, lv_color_hex(0x18222D), 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_radius(row, 10, 0);
        lv_obj_set_style_pad_all(row, 6, 0);
        lv_obj_set_style_pad_column(row, 6, 0);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *selCb = lv_checkbox_create(row);
        lv_obj_set_width(selCb, 24);
        if (mqttButtonSelected[i]) lv_obj_add_state(selCb, LV_STATE_CHECKED);
        if (mqttButtonSelectionMode) lv_obj_clear_flag(selCb, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(selCb, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_event_cb(selCb, lvglMqttButtonFieldChangedEvent, LV_EVENT_VALUE_CHANGED, reinterpret_cast<void *>(static_cast<intptr_t>(i)));

        lv_obj_t *nameTa = lv_textarea_create(row);
        lv_obj_set_width(nameTa, DISPLAY_WIDTH - 172);
        lv_obj_set_flex_grow(nameTa, 1);
        lvglConfigureCompactSingleLineTa(nameTa, 34);
        lv_textarea_set_max_length(nameTa, 20);
        lv_textarea_set_placeholder_text(nameTa, "Button name");
        lv_textarea_set_text(nameTa, mqttButtonNames[i].c_str());
        lv_obj_add_event_cb(nameTa, lvglTextAreaFocusEvent, LV_EVENT_FOCUSED, nullptr);
        lv_obj_add_event_cb(nameTa, lvglMqttButtonFieldChangedEvent, LV_EVENT_DEFOCUSED, reinterpret_cast<void *>(static_cast<intptr_t>(i)));
        lv_obj_add_event_cb(nameTa, lvglMqttButtonFieldChangedEvent, LV_EVENT_READY, reinterpret_cast<void *>(static_cast<intptr_t>(i)));

        lv_obj_t *critCb = lv_checkbox_create(row);
        lv_obj_set_width(critCb, 44);
        if (mqttButtonCritical[i]) lv_obj_add_state(critCb, LV_STATE_CHECKED);
        lv_obj_add_event_cb(critCb, lvglMqttButtonFieldChangedEvent, LV_EVENT_VALUE_CHANGED, reinterpret_cast<void *>(static_cast<intptr_t>(i)));

        lv_obj_t *toggleCb = lv_checkbox_create(row);
        lv_obj_set_width(toggleCb, 44);
        if (mqttButtonToggle[i]) lv_obj_add_state(toggleCb, LV_STATE_CHECKED);
        lv_obj_add_event_cb(toggleCb, lvglMqttButtonFieldChangedEvent, LV_EVENT_VALUE_CHANGED, reinterpret_cast<void *>(static_cast<intptr_t>(i)));
    }

    if (mqttButtonCount <= 0) {
        lv_obj_t *lbl = lv_label_create(lvglMqttButtonConfigList);
        lv_obj_set_width(lbl, lv_pct(100));
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_WRAP);
        lv_label_set_text(lbl, "Add the first MQTT button to show it in MQTT Controls");
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xB7C4D1), 0);
        lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
    }
}

void lvglRefreshMqttControlsUi()
{
    if (!lvglMqttCtrlList) return;
    lv_obj_clean(lvglMqttCtrlList);
    if (mqttButtonCount <= 0) {
        lv_obj_t *emptyCard = lv_obj_create(lvglMqttCtrlList);
        lv_obj_set_width(emptyCard, lv_pct(100));
        lv_obj_set_style_bg_color(emptyCard, lv_color_hex(0x16212C), 0);
        lv_obj_set_style_border_width(emptyCard, 0, 0);
        lv_obj_set_style_radius(emptyCard, 12, 0);
        lv_obj_set_style_pad_all(emptyCard, 12, 0);
        lv_obj_clear_flag(emptyCard, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *emptyLbl = lv_label_create(emptyCard);
        lv_obj_set_width(emptyLbl, lv_pct(100));
        lv_label_set_long_mode(emptyLbl, LV_LABEL_LONG_WRAP);
        lv_label_set_text(emptyLbl, "No MQTT buttons yet.\nOpen Button Config to add them.");
        lv_obj_set_style_text_color(emptyLbl, lv_color_hex(0xC8D3DD), 0);
        lv_obj_set_style_text_align(emptyLbl, LV_TEXT_ALIGN_CENTER, 0);
        return;
    }

    for (int i = 0; i < mqttButtonCount; i++) {
        String name = mqttButtonNames[i];
        String lbl = lvglSymbolText(mqttButtonToggle[i] ? LV_SYMBOL_POWER : LV_SYMBOL_LIST, name);
        if (mqttButtonCritical[i]) lbl += " (!)";
        const lv_color_t offColor = mqttButtonToggle[i]
                                        ? lv_color_hex(0x4C6174)
                                        : (mqttButtonCritical[i] ? lv_color_hex(0x9A5A2E) : lv_color_hex(0x2D6D8E));
        lv_obj_t *btn = lvglCreateMenuButton(
            lvglMqttCtrlList,
            lbl.c_str(),
            mqttButtonToggle[i] ? offColor : offColor,
            lvglMqttControlPressEvent,
            reinterpret_cast<void *>(static_cast<intptr_t>(i))
        );
        if (btn) {
            if (mqttButtonToggle[i]) {
                lvglRegisterPersistentStateButton(btn, offColor, mqttButtonState[i], true);
            }
            char key[16];
            snprintf(key, sizeof(key), "btn_%02d", i);
            lvglRegisterReorderableItem(btn, "ord_mctl", key);
        }
    }
    lvglApplySavedOrder(lvglMqttCtrlList, "ord_mctl");
}

bool mqttConnectNow()
{
    if (!mqttCfg.enabled) {
        mqttStatusLine = "Disabled";
        mqttConnectRequested = false;
        mqttRefreshStatusLabelIfVisible();
        if (lvglReady) lvglRefreshTopIndicators();
        return false;
    }
    if (networkSuspendedForAudio || !wifiConnectedSafe()) {
        mqttStatusLine = "No WiFi";
        mqttRefreshStatusLabelIfVisible();
        if (lvglReady) lvglRefreshTopIndicators();
        return false;
    }
    if (!mqttHasHeapHeadroom(MQTT_MIN_FREE_HEAP_CONNECT, MQTT_MIN_LARGEST_8BIT_CONNECT, "MQTT waiting for memory")) {
        mqttRefreshStatusLabelIfVisible();
        if (lvglReady) lvglRefreshTopIndicators();
        return false;
    }
    if (!mqttClient.setBufferSize(MQTT_CLIENT_BUFFER_SIZE)) {
        mqttStatusLine = "MQTT buffer alloc failed";
        mqttRefreshStatusLabelIfVisible();
        if (lvglReady) lvglRefreshTopIndicators();
        return false;
    }

    mqttNetClient.stop();
    // Keep connect attempts short so a bad broker does not block other tasks for long.
    mqttNetClient.setTimeout(MQTT_SOCKET_CONNECT_TIMEOUT_MS);
    if (!mqttNetClient.connect(mqttCfg.broker.c_str(), mqttCfg.port, MQTT_SOCKET_CONNECT_TIMEOUT_MS)) {
        mqttStatusLine = "Broker unreachable";
        mqttRefreshStatusLabelIfVisible();
        if (lvglReady) lvglRefreshTopIndicators();
        return false;
    }

    mqttClient.setServer(mqttCfg.broker.c_str(), mqttCfg.port);
    mqttClient.setCallback([](char *topic, uint8_t *payload, unsigned int length) {
        if (!topic || !payload || length == 0) return;
        if (length > MQTT_CLIENT_BUFFER_SIZE) return;
        if (ESP.getFreeHeap() < MQTT_MIN_FREE_HEAP_PUBLISH ||
            heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) < MQTT_MIN_LARGEST_8BIT_PUBLISH) return;

        const String topicStr = String(topic);
        if (topicStr == mqttHaChatCommandTopic()) {
            String text;
            text.reserve(length);
            for (unsigned int j = 0; j < length; ++j) text += static_cast<char>(payload[j]);
            mqttHandleHaChatCommand(text);
            return;
        }
        for (int i = 0; i < mqttButtonCount; ++i) {
            if (topicStr == mqttButtonCommandTopic(i)) {
                String cmd;
                cmd.reserve(length);
                for (unsigned int j = 0; j < length; ++j) cmd += static_cast<char>(payload[j]);
                mqttHandleButtonCommand(i, cmd);
                return;
            }
        }

        JsonDocument doc;
        if (deserializeJson(doc, payload, length) != DeserializationError::Ok) return;

        if (!mqttChatPresenceTopic.isEmpty() && strcmp(topic, mqttChatPresenceTopic.c_str()) == 0) {
            const String kind = String(static_cast<const char *>(doc["kind"] | ""));
            if (kind != "peer_presence") return;

            const String name = String(static_cast<const char *>(doc["name"] | ""));
            const String senderPubHex = String(static_cast<const char *>(doc["sender_pub"] | ""));
            if (senderPubHex.isEmpty()) return;
            if (senderPubHex.equalsIgnoreCase(p2pPublicKeyHex())) return;

            mqttTouchDiscoveredPeer(name, senderPubHex);

            if (lvglReady) {
                lvglRefreshChatPeerUi();
                lvglRefreshChatContactsUi();
                if (uiScreen == UI_CHAT) lvglRefreshChatUi();
            }
            return;
        }

        if (mqttChatInboxTopic.isEmpty() || strcmp(topic, mqttChatInboxTopic.c_str()) != 0) return;

        const String senderPubHex = String(static_cast<const char *>(doc["sender_pub"] | ""));
        const String nonceHex = String(static_cast<const char *>(doc["nonce"] | ""));
        const String cipherHex = String(static_cast<const char *>(doc["cipher"] | ""));
        const int peerIdx = p2pFindPeerByPubKeyHex(senderPubHex);
        if (peerIdx < 0 || !p2pPeers[peerIdx].enabled) return;

        unsigned char senderPk[P2P_PUBLIC_KEY_BYTES] = {0};
        unsigned char nonce[P2P_NONCE_BYTES] = {0};
        unsigned char cipher[P2P_MAX_PACKET] = {0};
        size_t cipherLen = 0;

        if (!p2pHexToBytes(senderPubHex, senderPk, sizeof(senderPk))) return;
        if (!p2pHexToBytes(nonceHex, nonce, sizeof(nonce))) return;
        if (sodium_hex2bin(cipher, sizeof(cipher), cipherHex.c_str(), cipherHex.length(), nullptr, &cipherLen, nullptr) != 0 ||
            cipherLen < P2P_MAC_BYTES) return;

        unsigned char plain[P2P_MAX_PACKET] = {0};
        if (crypto_box_curve25519xchacha20poly1305_open_easy(plain, cipher, cipherLen, nonce, senderPk, p2pSecretKey) != 0) return;

        JsonDocument plainDoc;
        if (deserializeJson(plainDoc, plain, cipherLen - P2P_MAC_BYTES) != DeserializationError::Ok) return;

        const String kind = String(static_cast<const char *>(plainDoc["kind"] | "chat"));
        const String author = String(static_cast<const char *>(plainDoc["author"] | "Remote"));

        p2pRefreshTrustedPeerIdentity(senderPubHex, author, p2pPeers[peerIdx].ip, p2pPeers[peerIdx].port);
        p2pTouchPeerSeen(peerIdx, p2pPeers[peerIdx].ip, p2pPeers[peerIdx].port);

        if (kind == "delete_conversation") {
            chatApplyConversationDeletion(senderPubHex, "Conversation deleted by " + author);
            return;
        }
        if (kind == "delete_message") {
            const String messageId = String(static_cast<const char *>(plainDoc["id"] | ""));
            if (!messageId.isEmpty()) chatDeleteMessageById(senderPubHex, messageId, "Message deleted by " + author);
            return;
        }
        if (kind == "ack") {
            const String ackId = String(static_cast<const char *>(plainDoc["ack_id"] | ""));
            if (!ackId.isEmpty()) chatAckOutgoingMessage(senderPubHex, ackId);
            return;
        }

        const String messageId = String(static_cast<const char *>(plainDoc["id"] | ""));
        const String text = String(static_cast<const char *>(plainDoc["text"] | ""));
        if (text.isEmpty()) return;
        const bool isDuplicate = !messageId.isEmpty() && chatHasLoggedMessageId(senderPubHex, messageId);

        if (!messageId.isEmpty()) mqttPublishChatAck(senderPubHex, messageId);

        wakeDisplayForIncomingNotification();

        if (checkersHandleIncomingChatPayload(senderPubHex, author, text, CHAT_TRANSPORT_MQTT, messageId)) {
            if (lvglReady) {
                lvglSyncStatusLine();
                if (uiScreen == UI_CHAT) lvglRefreshChatUi();
            }
        } else if (!isDuplicate) {
            chatStoreMessage(senderPubHex, author, text, false, CHAT_TRANSPORT_MQTT, messageId);
            uiStatusLine = "Global chat from " + author;
            if (lvglReady) {
                lvglSyncStatusLine();
                if (uiScreen == UI_CHAT) lvglRefreshChatUi();
            }
        }
    });

    const String availabilityTopic = "esp32/remote/" + mqttHwId + "/availability";
    bool ok = false;

    if (mqttCfg.username.length()) {
        ok = mqttClient.connect(mqttClientId.c_str(), mqttCfg.username.c_str(), mqttCfg.password.c_str(),
                                availabilityTopic.c_str(), 0, true, "offline");
    } else {
        ok = mqttClient.connect(mqttClientId.c_str(), availabilityTopic.c_str(), 0, true, "offline");
    }

    if (!ok) {
        mqttStatusLine = "Connect failed rc=" + String(mqttClient.state());
        mqttNetClient.stop();
        mqttRefreshStatusLabelIfVisible();
        if (lvglReady) lvglRefreshTopIndicators();
        return false;
    }

    mqttClient.publish(availabilityTopic.c_str(), "online", true);
    mqttClient.subscribe(mqttChatInboxTopic.c_str());
    if (!mqttChatPresenceTopic.isEmpty()) mqttClient.subscribe(mqttChatPresenceTopic.c_str());
    mqttClient.subscribe(mqttHaChatCommandTopic().c_str());
    for (int i = 0; i < MQTT_MAX_BUTTONS; ++i) mqttClient.subscribe(mqttButtonCommandTopic(i).c_str());
    mqttPublishPeerPresence();
    mqttDiscoveryPublished = false;
    mqttLastStatePayload = "";
    mqttLastStatePublishMs = 0;
    mqttStatusLine = "Connected";
    mqttConnectRequested = false;

    mqttRefreshStatusLabelIfVisible();
    if (lvglReady) lvglRefreshTopIndicators();
    return true;
}

void mqttTrimBufferForIdle()
{
    if (mqttClient.connected()) return;
    if (mqttClient.getBufferSize() > MQTT_CLIENT_BUFFER_IDLE) {
        mqttClient.setBufferSize(MQTT_CLIENT_BUFFER_IDLE);
    }
}

void mqttService()
{
    if (!mqttClient.connected() &&
        !mqttConnectRequested &&
        bootInteractiveUntilMs != 0 &&
        static_cast<long>(millis() - bootInteractiveUntilMs) < 0) {
        return;
    }
    if (networkSuspendedForAudio || airplaneModeEnabled) {
        const bool wasConnected = mqttClient.connected();
        if (wasConnected) mqttClient.disconnect();
        mqttTrimBufferForIdle();
        mqttConnectRequested = false;
        mqttStatusLine = "No WiFi";
        mqttRefreshStatusLabelIfVisible();
        if (wasConnected && lvglReady) lvglRefreshTopIndicators();
        return;
    }
    if (!mqttCfg.enabled) {
        const bool wasConnected = mqttClient.connected();
        if (wasConnected) mqttClient.disconnect();
        mqttTrimBufferForIdle();
        mqttConnectRequested = false;
        mqttStatusLine = "Disabled";
        mqttRefreshStatusLabelIfVisible();
        if (wasConnected && lvglReady) lvglRefreshTopIndicators();
        return;
    }
    if (!wifiConnectedSafe()) {
        const bool wasConnected = mqttClient.connected();
        if (wasConnected) mqttClient.disconnect();
        mqttTrimBufferForIdle();
        mqttConnectRequested = false;
        mqttStatusLine = "No WiFi";
        mqttRefreshStatusLabelIfVisible();
        if (wasConnected && lvglReady) lvglRefreshTopIndicators();
        return;
    }
    if (!mqttClient.connected()) {
        if (mqttConnectRequested || static_cast<long>(millis() - mqttNextReconnectMs) >= 0) {
            mqttConnectRequested = false;
            mqttLastReconnectMs = millis();
            bool connected = mqttConnectNow();
            if (connected) {
                mqttPublishDiscovery();
                mqttReconnectIntervalMs = MQTT_RECONNECT_MS;
                mqttNextReconnectMs = 0;
            } else {
                mqttNextReconnectMs = millis() + mqttReconnectIntervalMs;
                mqttReconnectIntervalMs = std::min(mqttReconnectIntervalMs * 2, MQTT_RECONNECT_MAX_MS);
            }
        }
        return;
    }
    mqttClient.loop();
    if (!mqttDiscoveryPublished) mqttPublishDiscovery();
    mqttPublishStateIfNeeded();
}
