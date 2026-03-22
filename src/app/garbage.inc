// Parking lot for symbols removed from active build after the unused-symbol sweep.
// Nothing in this file is compiled right now.
// If testing shows a hidden dependency, restore the symbol from here to its noted source area.

// From src/main.cpp: legacy chat notification constants
static constexpr uint16_t CHAT_NOTIFY_FREQ_SECONDARY = 1320;
static constexpr unsigned long CHAT_NOTIFY_BEEP1_MS = 70UL;
static constexpr unsigned long CHAT_NOTIFY_GAP_MS = 55UL;
static constexpr unsigned long CHAT_NOTIFY_BEEP2_MS = 95UL;

// From src/main.cpp: legacy battery calibration bounds
static constexpr float BATTERY_CAL_FACTOR_MIN = 0.75f;
static constexpr float BATTERY_CAL_FACTOR_MAX = 1.35f;

// From src/main.cpp: legacy HC12 constants
static constexpr int HC12_UART_NUM = 1;
static constexpr unsigned long HC12_BAUD = 9600UL;
static constexpr size_t HC12_RADIO_MAX_RX_LINE = HC12_RADIO_MAX_LINE + 16; // prefix + slack

// From src/main.cpp: legacy game layout constants
static constexpr int SNAKE_CELL = 14;
static constexpr int SNAKE_BOARD_X = 36;
static constexpr int SNAKE_BOARD_Y = 50;
static constexpr int TETRIS_CELL = 12;
static constexpr int TETRIS_BOARD_X = 60;
static constexpr int TETRIS_BOARD_Y = 50;

// From src/main.cpp: old service-slice interval constants
static constexpr unsigned long WIFI_SCAN_SERVICE_INTERVAL_MS = 25UL;
static constexpr unsigned long WIFI_CONN_SERVICE_INTERVAL_MS = 20UL;
static constexpr unsigned long SD_STATS_SERVICE_INTERVAL_MS = 100UL;
static constexpr unsigned long MDNS_SERVICE_INTERVAL_MS = 250UL;
static constexpr unsigned long P2P_SERVICE_INTERVAL_MS = 10UL;
static constexpr unsigned long CHAT_PENDING_SERVICE_INTERVAL_MS = 80UL;
static constexpr unsigned long RGB_SERVICE_INTERVAL_MS = 16UL;
static constexpr unsigned long CAR_TELEMETRY_SERVICE_INTERVAL_MS = 50UL;

// From src/main.cpp: unused HC12 helper
static bool hc12PeerRecentlySeen(const String &peerKey, unsigned long staleMs = HC12_DISCOVERY_STALE_MS)
{
    const int idx = hc12FindDiscoveredByPubKeyHex(peerKey);
    if (idx < 0) return false;
    return static_cast<unsigned long>(millis() - hc12DiscoveredPeers[idx].lastSeenMs) <= staleMs;
}

// From src/main.cpp: unused checkers control-message helper
static String checkersBuildDeclineText(const String &sessionId)
{
    return "@CHK_DECLINE|" + sessionId;
}

// From src/main.cpp: unused game navigation callbacks
void lvglSnakeBackEvent(lv_event_t *e)
{
    (void)e;
    lvglOpenScreen(UI_GAMES, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
}

void lvglTetrisBackEvent(lv_event_t *e)
{
    (void)e;
    lvglOpenScreen(UI_GAMES, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
}

// From src/main.cpp: unused SD route helper
bool pathUnderConversations(const String &path)
{
    String p = path;
    if (!p.startsWith("/")) p = "/" + p;
    String l = p;
    l.toLowerCase();
    return (l == "/conversations") || l.startsWith("/conversations/");
}

// From src/app/mqtt.inc: unused MQTT wrapper helpers
bool mqttPublishChatMessage(const String &text)
{
    if (currentChatPeerKey.isEmpty()) return false;
    const String messageId = chatGenerateMessageId();
    return mqttPublishChatMessageWithId(currentChatPeerKey, text, messageId);
}

void mqttPublishButtonAction(int index)
{
    if (index < 0 || index >= mqttButtonCount) return;
    String payload = mqttButtonPayloadForIndex(index);
    mqttPublishAction(payload.c_str());
}

String mqttHaChatStateTopic()
{
    return "esp32/remote/" + mqttHwId + "/chat/ha/state";
}

// From src/app/p2p_transport.inc: unused LAN chat wrapper
bool p2pSendChatMessage(const String &text)
{
    if (currentChatPeerKey.isEmpty()) return false;
    const String messageId = chatGenerateMessageId();
    return p2pSendChatMessageWithId(currentChatPeerKey, text, messageId);
}

// From src/app/ota_backend.inc: unused release lookup helper
static bool otaFetchReleaseInfoForTag(const String &tagName, String &binUrlOut, String &errorOut)
{
    binUrlOut = "";
    errorOut = "";
    if (tagName.isEmpty()) {
        errorOut = "missing_tag";
        return false;
    }

    int ghStatus = -1;
    const String url = String("https://api.github.com/repos/elik745i/ESP32-2432S024C-Remote/releases/tags/") + tagName;
    const String body = httpsGetText(url, &ghStatus);
    if (body.isEmpty()) {
        errorOut = "github_fetch_failed";
        return false;
    }

    JsonDocument filter;
    filter["assets"][0]["name"] = true;
    filter["assets"][0]["browser_download_url"] = true;

    JsonDocument parsed;
    const DeserializationError err = deserializeJson(parsed, body, DeserializationOption::Filter(filter));
    if (err) {
        errorOut = "json_parse_failed";
        return false;
    }

    binUrlOut = chooseLatestFirmwareBinUrl(parsed["assets"]);
    if (binUrlOut.isEmpty()) {
        errorOut = "release_asset_not_found";
        return false;
    }
    return true;
}
