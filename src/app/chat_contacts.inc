void chatReloadRecentMessagesFromSd(const String &peerKey)
{
    chatClearCache();
    if (peerKey.isEmpty()) return;
    if (!sdEnsureMounted()) return;
    if (!sdLock()) return;

    File f = SD.open(chatLogPathForPeer(peerKey), FILE_READ);
    if (f) {
        while (f.available()) {
            String line = f.readStringUntil('\n');
            line.trim();
            if (line.isEmpty()) continue;
            const int p1 = line.indexOf('\t');
            const int p2 = (p1 >= 0) ? line.indexOf('\t', p1 + 1) : -1;
            const int p3 = (p2 >= 0) ? line.indexOf('\t', p2 + 1) : -1;
            const int p4 = (p3 >= 0) ? line.indexOf('\t', p3 + 1) : -1;
            const int p5 = (p4 >= 0) ? line.indexOf('\t', p4 + 1) : -1;
            if (p1 <= 0 || p2 <= p1 || p3 <= p2 || p4 <= p3) continue;
            ChatMessage msg;
            msg.tsMs = static_cast<unsigned long>(strtoul(line.substring(0, p1).c_str(), nullptr, 10));
            msg.outgoing = line.substring(p1 + 1, p2) == "out";
            msg.transport = static_cast<ChatTransport>(atoi(line.substring(p2 + 1, p3).c_str()));
            msg.author = chatUnescapeLogField(line.substring(p3 + 1, p4));
            msg.text = chatUnescapeLogField((p5 > p4) ? line.substring(p4 + 1, p5) : line.substring(p4 + 1));
            msg.messageId = (p5 > p4) ? chatUnescapeLogField(line.substring(p5 + 1)) : String("");
            msg.failed = false;
            if (msg.text.isEmpty()) continue;
            if (chatMessageCount < MAX_CHAT_MESSAGES) {
                chatMessages[chatMessageCount++] = msg;
            } else {
                for (int i = 1; i < chatMessageCount; ++i) chatMessages[i - 1] = chatMessages[i];
                chatMessages[chatMessageCount - 1] = msg;
            }
        }
        f.close();
    }
    sdUnlock();
}

static bool p2pHexToBytes(const String &hex, unsigned char *out, size_t outLen)
{
    if (!out || outLen == 0) return false;
    size_t binLen = 0;
    return sodium_hex2bin(out, outLen, hex.c_str(), hex.length(), nullptr, &binLen, nullptr) == 0 && binLen == outLen;
}

static String p2pBytesToHex(const unsigned char *data, size_t len)
{
    if (!data || len == 0) return "";
    char buf[(P2P_PUBLIC_KEY_BYTES * 2) + 1] = {0};
    if (len > P2P_PUBLIC_KEY_BYTES) return "";
    sodium_bin2hex(buf, sizeof(buf), data, len);
    return String(buf);
}

String p2pPublicKeyHex()
{
    return p2pBytesToHex(p2pPublicKey, sizeof(p2pPublicKey));
}

static int p2pFindPeerByPubKeyHex(const String &pubKeyHex);

static int p2pFindPeerByPubKeyHex(const String &pubKeyHex)
{
    if (!p2pPeers) return -1;
    for (int i = 0; i < p2pPeerCount; ++i) {
        if (p2pPeers[i].pubKeyHex.equalsIgnoreCase(pubKeyHex)) return i;
    }
    return -1;
}

static int p2pFindDiscoveredByPubKeyHex(const String &pubKeyHex)
{
    if (!p2pDiscoveredPeers) return -1;
    for (int i = 0; i < p2pDiscoveredCount; ++i) {
        if (p2pDiscoveredPeers[i].pubKeyHex.equalsIgnoreCase(pubKeyHex)) return i;
    }
    return -1;
}

static void p2pDeduplicateTrustedPeers()
{
    for (int i = 0; i < p2pPeerCount; ++i) {
        for (int j = i + 1; j < p2pPeerCount; ) {
            const bool samePubKey = p2pPeers[i].pubKeyHex.equalsIgnoreCase(p2pPeers[j].pubKeyHex);
            const bool sameNamedEndpoint =
                !p2pPeers[i].name.isEmpty() &&
                p2pPeers[i].name.equalsIgnoreCase(p2pPeers[j].name) &&
                p2pPeers[i].ip != IPAddress((uint32_t)0) &&
                p2pPeers[i].ip == p2pPeers[j].ip;
            if (samePubKey || sameNamedEndpoint) {
                for (int k = j + 1; k < p2pPeerCount; ++k) {
                    p2pPeers[k - 1] = p2pPeers[k];
                }
                p2pPeerCount--;
            } else {
                ++j;
            }
        }
    }
}

static void p2pTouchPeerSeen(int idx, const IPAddress &ip, uint16_t port)
{
    if (!p2pPeers || idx < 0 || idx >= p2pPeerCount) return;
    p2pPeers[idx].ip = ip;
    p2pPeers[idx].port = port;
    p2pPeers[idx].lastSeenMs = millis();
}

static void p2pRefreshTrustedPeerIdentity(const String &pubKeyHex, const String &name, const IPAddress &ip, uint16_t port)
{
    const int idx = p2pFindPeerByPubKeyHex(pubKeyHex);
    if (idx < 0) return;

    bool changed = false;
    const String nextName = sanitizeDeviceShortName(name);
    if (!nextName.isEmpty() && p2pPeers[idx].name != nextName) {
        p2pPeers[idx].name = nextName;
        changed = true;
    }
    if (ip != IPAddress((uint32_t)0) && p2pPeers[idx].ip != ip) {
        p2pPeers[idx].ip = ip;
        changed = true;
    }
    const uint16_t nextPort = port ? port : P2P_UDP_PORT;
    if (p2pPeers[idx].port != nextPort) {
        p2pPeers[idx].port = nextPort;
        changed = true;
    }
    p2pPeers[idx].lastSeenMs = millis();

    if (changed) {
        saveP2pConfig();
        if (lvglReady) {
            if (uiScreen == UI_CHAT || uiScreen == UI_CHAT_PEERS) lvglRefreshChatContactsUi();
            if (uiScreen == UI_CHAT) lvglRefreshChatUi();
            if (uiScreen == UI_CHAT_PEERS) lvglRefreshChatPeerUi();
        }
    }
}

static void p2pTouchDiscoveredSeen(const String &name, const String &pubKeyHex, const IPAddress &ip, uint16_t port)
{
    if (pubKeyHex.isEmpty()) return;
    if (!p2pEnsureDiscoveredStorage()) return;
    int idx = p2pFindDiscoveredByPubKeyHex(pubKeyHex);
    if (idx < 0) {
        if (p2pDiscoveredCount >= MAX_P2P_DISCOVERED) {
            for (int i = 1; i < p2pDiscoveredCount; ++i) p2pDiscoveredPeers[i - 1] = p2pDiscoveredPeers[i];
            p2pDiscoveredCount = MAX_P2P_DISCOVERED - 1;
        }
        idx = p2pDiscoveredCount++;
    }
    p2pDiscoveredPeers[idx].name = name;
    p2pDiscoveredPeers[idx].pubKeyHex = pubKeyHex;
    p2pDiscoveredPeers[idx].ip = ip;
    p2pDiscoveredPeers[idx].port = port;
    p2pDiscoveredPeers[idx].trusted = p2pFindPeerByPubKeyHex(pubKeyHex) >= 0;
    p2pDiscoveredPeers[idx].lastSeenMs = millis();
    if (p2pDiscoveredPeers[idx].trusted) p2pRefreshTrustedPeerIdentity(pubKeyHex, name, ip, port);
    if (lvglReady && uiScreen == UI_CHAT_PEERS) lvglRefreshChatPeerUi();
}

void saveP2pConfig()
{
    if (!p2pPeers) return;
    p2pPrefs.begin("p2p", false);
    p2pPrefs.putString("pub", p2pPublicKeyHex());
    p2pPrefs.putBytes("sec", p2pSecretKey, sizeof(p2pSecretKey));
    p2pPrefs.putBool("discover_en", p2pDiscoveryEnabled);
    p2pPrefs.putInt("peer_count", p2pPeerCount);
    for (int i = 0; i < MAX_P2P_PEERS; ++i) {
        p2pPrefs.putString((String("peer_name_") + i).c_str(), (i < p2pPeerCount) ? p2pPeers[i].name : "");
        p2pPrefs.putString((String("peer_key_") + i).c_str(), (i < p2pPeerCount) ? p2pPeers[i].pubKeyHex : "");
        p2pPrefs.putString((String("peer_ip_") + i).c_str(), (i < p2pPeerCount) ? p2pPeers[i].ip.toString() : "");
        p2pPrefs.putUShort((String("peer_port_") + i).c_str(), (i < p2pPeerCount) ? p2pPeers[i].port : P2P_UDP_PORT);
        p2pPrefs.putBool((String("peer_en_") + i).c_str(), (i < p2pPeerCount) ? p2pPeers[i].enabled : false);
    }
    p2pPrefs.end();

    uiPrefs.begin("ui", false);
    uiPrefs.putBool("chat_disc", p2pDiscoveryEnabled);
    uiPrefs.end();
}

void loadP2pConfig()
{
    p2pPeerCount = 0;
    p2pDiscoveredCount = 0;
    p2pPrefs.begin("p2p", false);
    p2pDiscoveryEnabled = p2pPrefs.getBool("discover_en", p2pDiscoveryEnabled);

    String pubHex = p2pPrefs.getString("pub", "");
    size_t secLen = p2pPrefs.getBytesLength("sec");
    bool haveKeys = (pubHex.length() == (P2P_PUBLIC_KEY_BYTES * 2U)) && (secLen == sizeof(p2pSecretKey));
    if (haveKeys) {
        haveKeys = p2pHexToBytes(pubHex, p2pPublicKey, sizeof(p2pPublicKey));
        if (haveKeys) p2pPrefs.getBytes("sec", p2pSecretKey, sizeof(p2pSecretKey));
    }
    if (!haveKeys) {
        crypto_box_curve25519xchacha20poly1305_keypair(p2pPublicKey, p2pSecretKey);
    }

    p2pPeerCount = p2pPrefs.getInt("peer_count", 0);
    if (p2pPeerCount < 0) p2pPeerCount = 0;
    if (p2pPeerCount > MAX_P2P_PEERS) p2pPeerCount = MAX_P2P_PEERS;
    if (p2pPeerCount > 0 && !p2pEnsurePeerStorage()) {
        p2pPeerCount = 0;
        p2pPrefs.end();
        return;
    }
    for (int i = 0; i < p2pPeerCount; ++i) {
        p2pPeers[i].name = p2pPrefs.getString((String("peer_name_") + i).c_str(), "");
        p2pPeers[i].pubKeyHex = p2pPrefs.getString((String("peer_key_") + i).c_str(), "");
        p2pPeers[i].ip.fromString(p2pPrefs.getString((String("peer_ip_") + i).c_str(), ""));
        p2pPeers[i].port = p2pPrefs.getUShort((String("peer_port_") + i).c_str(), P2P_UDP_PORT);
        p2pPeers[i].enabled = p2pPrefs.getBool((String("peer_en_") + i).c_str(), true);
        p2pPeers[i].unread = false;
        p2pPeers[i].lastSeenMs = 0;
    }

    p2pDeduplicateTrustedPeers();

    p2pPrefs.end();
    saveP2pConfig();
}

void lvglRefreshWifiList();
void lvglRefreshMediaList();
void lvglRefreshInfoPanel(bool refreshIndicators = true);
static void queueInfoRefresh(bool includeRadio = false);
static void queueRadioInfoFetch(unsigned long delayMs = 20UL);
static void cancelRadioInfoFetch();
static void applyRadioInfoLabels();
static void queueHc12ConfigApply(Hc12ConfigApplyAction action, int target);
static void serviceDeferredHc12ConfigApply();
void lvglMediaPlayStopEvent(lv_event_t *e);
void lvglMediaPrevTrackEvent(lv_event_t *e);
void lvglMediaNextTrackEvent(lv_event_t *e);
void lvglMediaVolumeEvent(lv_event_t *e);
void lvglMediaVolumeStepEvent(lv_event_t *e);
void lvglHomeNavEvent(lv_event_t *e);
void lvglApModeEvent(lv_event_t *e);
void lvglWifiRescanEvent(lv_event_t *e);
void lvglWifiDisconnectEvent(lv_event_t *e);
void lvglWifiForgetEvent(lv_event_t *e);
void lvglWifiApSaveEvent(lv_event_t *e);
void lvglWifiWebServerToggleEvent(lv_event_t *e);
void lvglMediaRefreshEvent(lv_event_t *e);
static void mediaEnsureStorageReadyForUi();
void lvglOpenSnakeEvent(lv_event_t *e);
void lvglOpenTetrisEvent(lv_event_t *e);
void lvglOpenCheckersEvent(lv_event_t *e);
#if defined(BOARD_ESP32S3_3248S035_N16R8)
void lvglOpenSnake3dEvent(lv_event_t *e);
#endif
void lvglOpenMqttCfgEvent(lv_event_t *e);
void lvglOpenMqttCtrlEvent(lv_event_t *e);
void lvglOpenMqttButtonConfigScreenEvent(lv_event_t *e);
void lvglMqttButtonAddEvent(lv_event_t *e);
void lvglMqttButtonSelectModeEvent(lv_event_t *e);
void lvglMqttButtonDeletePromptEvent(lv_event_t *e);
void lvglOpenRemoteControlScreenEvent(lv_event_t *e);
void lvglOpenChatPeersEvent(lv_event_t *e);
void lvglChatDiscoveryToggleEvent(lv_event_t *e);
void lvglOpenStyleScreenEvent(lv_event_t *e);
void lvglOpenStyleHomeItemsScreenEvent(lv_event_t *e);
void lvglOpenLanguageScreenEvent(lv_event_t *e);
void lvglStyleScreensaverToggleEvent(lv_event_t *e);
void lvglStyleMenuIconsToggleEvent(lv_event_t *e);
void lvglStyleTouchVibrationToggleEvent(lv_event_t *e);
void lvglStyleTouchSoundToggleEvent(lv_event_t *e);
void lvglStyleHomeChatToggleEvent(lv_event_t *e);
void lvglStyleHomeRadioToggleEvent(lv_event_t *e);
void lvglStyleHomeMediaToggleEvent(lv_event_t *e);
void lvglStyleHomeInfoToggleEvent(lv_event_t *e);
void lvglStyleHomeGamesToggleEvent(lv_event_t *e);
void lvglStyleHomeMqttToggleEvent(lv_event_t *e);
void lvglStyleHomeAirplaneToggleEvent(lv_event_t *e);
void lvglStyleHomeApToggleEvent(lv_event_t *e);
void lvglStyleButtonFlatEvent(lv_event_t *e);
void lvglStyleButton3dEvent(lv_event_t *e);
void lvglStyleButtonBlackEvent(lv_event_t *e);
void lvglStyleTimezoneEvent(lv_event_t *e);
void lvglLanguageDropdownEvent(lv_event_t *e);
void lvglVibrationDropdownEvent(lv_event_t *e);
void lvglMessageToneDropdownEvent(lv_event_t *e);
void lvglSoundPopupBackdropEvent(lv_event_t *e);
void lvglSoundPopupVolumeEvent(lv_event_t *e);
void lvglSoundPopupVibrationEvent(lv_event_t *e);
void lvglSoundPopupDisableVibrationEvent(lv_event_t *e);
void lvglRadioModuleDropdownEvent(lv_event_t *e);
void lvglHc12PrevExtraEvent(lv_event_t *e);
void lvglHc12NextExtraEvent(lv_event_t *e);
void lvglRadioModeWarningEvent(lv_event_t *e);
void lvglStyleTopCenterNameEvent(lv_event_t *e);
void lvglStyleTopCenterTimeEvent(lv_event_t *e);
void lvglStyleTimeoutEvent(lv_event_t *e);
void lvglStylePowerOffEvent(lv_event_t *e);
void lvglStyleDeepSleepEvent(lv_event_t *e);
void lvglRefreshStyleUi();
void lvglRefreshStyleHomeItemsUi();
void lvglRefreshLanguageUi();
void lvglRefreshRemoteControlUi();
void lvglRefreshMqttButtonConfigUi();
void lvglRefreshLocalizedUi();
void lvglGestureBlockEvent(lv_event_t *e);
void screensaverSetActive(bool active);
void screensaverService();
void lvglAirplaneToggleEvent(lv_event_t *e);
void lvglChatAirplanePromptEvent(lv_event_t *e);
void lvglShowChatAirplanePrompt();
bool lvglChatPromptIfAirplaneBlocked();
void lvglPowerButtonEvent(lv_event_t *e);
void lvglPowerDialogEvent(lv_event_t *e);
void lvglStatusPush(const String &line);
void lvglBrightnessEvent(lv_event_t *e);
void lvglAdaptiveBrightnessEvent(lv_event_t *e);
void lvglConfigVolumeEvent(lv_event_t *e);
void lvglRgbLedEvent(lv_event_t *e);
static void lvglRefreshHomeButtonVisibility();
void lvglTextAreaFocusEvent(lv_event_t *e);
static void hc12InitIfNeeded();
static void hc12AppendTerminal(const char *text);
static void hc12SendLine(const String &line);
static void hc12Service();
static void hc12RefreshInfoSnapshot();
static bool hc12ReadConfigSelection();
static bool hc12ApplyChannel(int channel);
static bool hc12ApplyBaudIndex(int index);
static bool hc12ApplyModeIndex(int index);
static bool hc12ApplyPowerLevel(int level);
static bool e220ApplyTransferMode(bool fixedTransmission);
static void loadPersistedRadioSettings();
static void savePersistedRadioSettings();
static bool hc12FactoryReset();
static String hc12CompactResponse(String raw);
static String hc12FieldValue(const String &raw, const char *prefix);
static String e220QueryCommand(const char *line, unsigned long totalTimeoutMs = 220UL, unsigned long quietTimeoutMs = 48UL);
static String &hc12LogBuffer();
static lv_obj_t *hc12WrapObj();
static lv_obj_t *hc12SetBtnObj();
static lv_obj_t *hc12TerminalObj();
static lv_obj_t *hc12CmdTaObj();
static bool hc12SetIsAsserted();
void lvglWifiApShowToggleEvent(lv_event_t *e);
void lvglWifiPwdCancelEvent(lv_event_t *e);
void lvglWifiPwdConnectEvent(lv_event_t *e);
void lvglWifiPwdShowToggleEvent(lv_event_t *e);
void lvglMqttPassShowToggleEvent(lv_event_t *e);
void lvglMqttEnableEvent(lv_event_t *e);
void lvglOpenWifiPasswordDialog(const String &ssid);
void lvglMqttCountMinusEvent(lv_event_t *e);
void lvglMqttCountPlusEvent(lv_event_t *e);
void lvglMqttEditPrevEvent(lv_event_t *e);
void lvglMqttEditNextEvent(lv_event_t *e);
void lvglMqttApplyBtnEvent(lv_event_t *e);
void lvglMqttSaveEvent(lv_event_t *e);
void lvglMqttConnectEvent(lv_event_t *e);
void lvglMqttPublishDiscoveryEvent(lv_event_t *e);
void lvglRefreshMqttConfigUi();
void lvglRefreshMqttControlsUi();
void lvglRefreshMqttButtonConfigUi();
static bool lvglCanBuildScreen(UiScreen screen);
static void lvglWarmupScreensService(bool uiPriorityActive);
void lvglOtaPopupEvent(lv_event_t *e);
void lvglOtaAvailablePromptEvent(lv_event_t *e);
void lvglRefreshSnakeBoard();
void lvglRefreshTetrisBoard();
void lvglRefreshCheckersBoard();
#if defined(BOARD_ESP32S3_3248S035_N16R8)
void lvglRefreshSnake3dBoard();
#endif
void lvglSnakeBoardDrawEvent(lv_event_t *e);
void lvglSnakeRestartEvent(lv_event_t *e);
void lvglSnakeDirEvent(lv_event_t *e);
void lvglSnakePauseEvent(lv_event_t *e);
void lvglTetrisBoardDrawEvent(lv_event_t *e);
void lvglTetrisRestartEvent(lv_event_t *e);
void lvglTetrisPauseEvent(lv_event_t *e);
void lvglTetrisMoveLeftEvent(lv_event_t *e);
void lvglTetrisMoveRightEvent(lv_event_t *e);
void lvglTetrisRotateEvent(lv_event_t *e);
void lvglTetrisDropEvent(lv_event_t *e);
void lvglCheckersBoardDrawEvent(lv_event_t *e);
void lvglCheckersBoardEvent(lv_event_t *e);
void lvglCheckersEsp32Event(lv_event_t *e);
void lvglCheckersTagEvent(lv_event_t *e);
void lvglCheckersBackEvent(lv_event_t *e);
void lvglCheckersReplayEvent(lv_event_t *e);
void lvglCheckersInvitePlayEvent(lv_event_t *e);
void lvglCheckersPeerSelectEvent(lv_event_t *e);
void lvglCheckersVariantSelectEvent(lv_event_t *e);
void lvglChatPeerActionEvent(lv_event_t *e);
void checkersTick();
#if defined(BOARD_ESP32S3_3248S035_N16R8)
void lvglSnake3dBoardDrawEvent(lv_event_t *e);
void lvglSnake3dRestartEvent(lv_event_t *e);
void lvglSnake3dDirEvent(lv_event_t *e);
void lvglSnake3dPauseEvent(lv_event_t *e);
void snake3dTick();
#else
inline void snake3dTick() {}
#endif
static int checkersBoardSize();
static int checkersStartRows();
static bool checkersMenCaptureBackward();
static bool checkersFlyingKings();
static bool checkersPromotionContinuesCapture();

inline void lvglLoadScreen(lv_obj_t *target, lv_scr_load_anim_t anim)
{
    if (!target) return;
    lvglResetGestureTracking();
    lvglGestureBlocked = false;
    lvglReorderOwnsHorizontalGesture = false;
    if (lvglTouchIndev) lv_indev_reset(lvglTouchIndev, nullptr);
    lv_obj_update_layout(target);
    lv_obj_invalidate(target);
    if (anim == LV_SCR_LOAD_ANIM_NONE) {
        lv_scr_load(target);
        lv_refr_now(nullptr);
        lvglResetSwipeBackVisualState(true);
    } else {
        lvglResetSwipeBackVisualState(true);
        lv_scr_load_anim(target, anim, UI_ANIM_MS, 0, false);
    }
}

static void lvglPrepareScreenLeave(UiScreen screen)
{
    if (screen == UI_WIFI_LIST) stopWifiScan(false);
    if (screen == UI_CONFIG_MODULES) moduleStopCatalogRefresh(false);
    if (screen == UI_CONFIG_RADIO_BUTTONS) {
        syncRadioButtonConfigFromUi();
        savePersistedRadioControlButtons();
    }
    if (screen == UI_CONFIG_RADIO_RECEIVER_TEST) radioReceiverTestLeaveScreen();
    lv_obj_t *screenObj = lvglScreenForUi(screen);
    if (screenObj && lv_obj_is_valid(screenObj)) {
        if (lvglKeyboardFocusedTa && lv_obj_is_valid(lvglKeyboardFocusedTa) &&
            lv_obj_get_screen(lvglKeyboardFocusedTa) == screenObj) {
            lvglHideKeyboard();
        }

        lv_obj_t *activeObj = lv_indev_get_obj_act();
        if (activeObj && lv_obj_is_valid(activeObj) && lv_obj_get_screen(activeObj) == screenObj) {
            lv_obj_clear_state(activeObj, LV_STATE_PRESSED | LV_STATE_FOCUSED | LV_STATE_EDITED);
        }
    }

    if (lvglTouchIndev) lv_indev_reset(lvglTouchIndev, nullptr);
    lvglResetGestureTracking();
    lvglGestureBlocked = false;
    lvglReorderOwnsHorizontalGesture = false;
}

static void queueInfoRefresh(bool includeRadio)
{
    infoRefreshPending = true;
    infoRefreshPhase = 0;
    if (includeRadio) queueRadioInfoFetch();
}

static void queueRadioInfoFetch(unsigned long delayMs)
{
    radioInfoFetchPending = true;
    radioInfoFetchActive = false;
    radioInfoFetchStage = 0;
    radioInfoFetchDueMs = millis() + delayMs;
    radioInfoFetchNextMs = radioInfoFetchDueMs;
    hc12InfoValueText = tr(TXT_RADIO_INFO_LOADING);
    hc12InfoSubText = tr(TXT_RADIO_INFO_LOADING);
    radioInfoVersionText = tr(TXT_RADIO_INFO_LOADING);
    radioInfoBaudText = "--";
    radioInfoChannelText = "--";
    radioInfoFuModeText = "--";
    radioInfoPowerText = "--";
    radioInfoRawText = tr(TXT_RADIO_INFO_LOADING);
    applyRadioInfoLabels();
}

static void cancelRadioInfoFetch()
{
    if (radioInfoFetchActive) hc12ExitAtMode();
    radioInfoFetchPending = false;
    radioInfoFetchActive = false;
    radioInfoFetchStage = 0;
}

static void applyRadioInfoLabels()
{
    if (lvglInfoHc12ValueLabel) lvglLabelSetTextIfChanged(lvglInfoHc12ValueLabel, hc12InfoValueText);
    if (lvglInfoHc12SubLabel) lvglLabelSetTextIfChanged(lvglInfoHc12SubLabel, hc12InfoSubText);
    if (lvglHc12InfoVersionLabel) lvglLabelSetTextIfChanged(lvglHc12InfoVersionLabel, radioInfoVersionText);
    if (lvglHc12InfoBaudLabel) lvglLabelSetTextIfChanged(lvglHc12InfoBaudLabel, radioInfoBaudText);
    if (lvglHc12InfoChannelLabel) lvglLabelSetTextIfChanged(lvglHc12InfoChannelLabel, radioInfoChannelText);
    if (lvglHc12InfoFuModeLabel) lvglLabelSetTextIfChanged(lvglHc12InfoFuModeLabel, radioInfoFuModeText);
    if (lvglHc12InfoPowerLabel) lvglLabelSetTextIfChanged(lvglHc12InfoPowerLabel, radioInfoPowerText);
    if (lvglHc12InfoRawLabel) lvglLabelSetTextIfChanged(lvglHc12InfoRawLabel, radioInfoRawText);
}

static void serviceInfoRefresh()
{
    if (!infoRefreshPending) return;
    if (uiScreen != UI_INFO) {
        infoRefreshPending = false;
        return;
    }

    if (infoRefreshPhase == 0) {
        const bool connected = wifiConnectedSafe();
        const String ssid = connected ? wifiSsidSafe() : String("Disconnected");
        const String ip = connected ? wifiIpSafe() : String("-");
        const int rssi = connected ? static_cast<int>(wifiRssiSafe()) : -127;
        infoWifiQualityPercent = connected ? wifiQualityPercentFromRssi(rssi) : 0;
        infoWifiValueText = connected ? String(infoWifiQualityPercent) + "%" : String("Offline");
        if (connected) infoWifiSubText = ssid + "  |  " + ip + "  |  " + String(rssi) + "dBm";
        else if (apModeActive) infoWifiSubText = "AP " + savedApSsid + "  |  " + WiFi.softAPIP().toString() + "  |  Touch to Config > WiFi Config to connect";
        else infoWifiSubText = "AP " + savedApSsid + "  |  Touch to Config > WiFi Config to connect";

        infoLightPercentValue = lightPercent;
        infoLightValueText = String(lightPercent) + "%";
        infoLightSubText = String("Display ") + (displayAwake ? "awake" : "sleeping") +
                           "  |  Backlight " + String(displayBrightnessPercent) +
                           "%  |  raw " + String(lightRawAdc);
        infoRefreshPhase = 1;
        lvglRefreshInfoPanel();
        return;
    }

    const float tempC = temperatureRead();
    infoTempBarValue = 0;
    if (!isnan(tempC) && tempC > 0.0f) {
        float clamped = tempC;
        if (clamped < 0.0f) clamped = 0.0f;
        if (clamped > INFO_TEMP_BAR_MAX_C) clamped = INFO_TEMP_BAR_MAX_C;
        infoTempBarValue = static_cast<uint8_t>(clamped + 0.5f);
        infoTempValueText = String(static_cast<int>(tempC + 0.5f)) + "C";
    } else {
        infoTempValueText = "--";
    }
    infoTempSubText = String("ESP32 die temp  |  target range under ") + String(INFO_TEMP_WARN_C) + "C";

    const uint32_t freeHeap = static_cast<uint32_t>(ESP.getFreeHeap());
    const uint32_t heapTotal = static_cast<uint32_t>(ESP.getHeapSize());
    const uint32_t largest8 = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    infoSramPercentValue = heapTotal > 0U ? static_cast<uint8_t>((freeHeap * 100ULL) / heapTotal) : 0U;
    infoSramValueText = String(infoSramPercentValue) + "%";
    infoSramSubText = String("Free ") + String(static_cast<unsigned long long>(freeHeap / 1024ULL)) +
                      " / " + String(static_cast<unsigned long long>(heapTotal / 1024ULL)) +
                      " KB  |  Largest block " + String(static_cast<unsigned long long>(largest8 / 1024ULL)) + " KB";

    const uint32_t psramFree = boardHasUsablePsram()
                                   ? static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM))
                                   : 0U;
    const uint32_t psramTotal = boardHasUsablePsram()
                                    ? static_cast<uint32_t>(heap_caps_get_total_size(MALLOC_CAP_SPIRAM))
                                    : 0U;
    const uint32_t psramUsed = (psramTotal > psramFree) ? (psramTotal - psramFree) : 0U;
    infoPsramPercentValue = psramTotal > 0U ? static_cast<uint8_t>((psramUsed * 100ULL) / psramTotal) : 0U;
    infoPsramValueText = psramTotal > 0U ? String(infoPsramPercentValue) + "% used" : String("Off");
    infoPsramSubText = psramTotal > 0U
                           ? String("Used ") + String(static_cast<unsigned long long>(psramUsed / (1024ULL * 1024ULL))) +
                                 " / " + String(static_cast<unsigned long long>(psramTotal / (1024ULL * 1024ULL))) + " MB"
                           : String("PSRAM unavailable");

    uint64_t sdTotal = 0;
    uint64_t sdUsed = 0;
    if (sdMounted) {
        sdTotal = SD.totalBytes();
        sdUsed = SD.usedBytes();
    }
    infoSdPercentValue = (sdTotal > 0) ? static_cast<uint8_t>((sdUsed * 100ULL) / sdTotal) : 0U;
    infoSdValueText = sdTotal > 0 ? String(infoSdPercentValue) + "%" : String(sdMounted ? "--" : "Off");
    infoSdSubText = sdTotal > 0
                        ? String("Used ") + String(static_cast<unsigned long long>(sdUsed / (1024ULL * 1024ULL))) +
                              " / " + String(static_cast<unsigned long long>(sdTotal / (1024ULL * 1024ULL))) +
                              " MB  |  Free " + String(static_cast<unsigned long long>((sdTotal - sdUsed) / (1024ULL * 1024ULL))) + " MB"
                        : String(sdMounted ? "Capacity unavailable" : "Card offline");

    infoRefreshPending = false;
    infoRefreshPhase = 0;
    lvglRefreshInfoPanel(false);
}

static void serviceDeferredRadioInfoFetch()
{
    if (!radioInfoFetchPending) return;
    if (uiScreen != UI_INFO && uiScreen != UI_CONFIG_HC12_INFO) {
        cancelRadioInfoFetch();
        return;
    }

    const unsigned long now = millis();
    const unsigned long dueMs = radioInfoFetchActive ? radioInfoFetchNextMs : radioInfoFetchDueMs;
    if (static_cast<long>(now - dueMs) < 0) return;

    if (!radioInfoFetchActive) {
        hc12InitIfNeeded();
        hc12EnterAtMode();
        radioInfoFetchActive = true;
        radioInfoFetchStage = 1;
        radioInfoFetchNextMs = millis() + (radioModuleType == RADIO_MODULE_E220 ? 30UL : 0UL);
        return;
    }

    if (radioModuleType == RADIO_MODULE_E220) {
        static String modelRaw;
        static String fwRaw;
        static String uartRaw;
        static String channelRaw;
        static String rateRaw;
        static String powerRaw;
        static String transRaw;
        switch (radioInfoFetchStage) {
            case 1: modelRaw = e220QueryCommand("AT+DEVTYPE=?"); radioInfoFetchStage = 2; radioInfoFetchNextMs = millis() + 20UL; return;
            case 2: fwRaw = e220QueryCommand("AT+FWCODE=?"); radioInfoFetchStage = 3; radioInfoFetchNextMs = millis() + 20UL; return;
            case 3: uartRaw = e220QueryCommand("AT+UART=?"); radioInfoFetchStage = 4; radioInfoFetchNextMs = millis() + 20UL; return;
            case 4: channelRaw = e220QueryCommand("AT+CHANNEL=?"); radioInfoFetchStage = 5; radioInfoFetchNextMs = millis() + 20UL; return;
            case 5: rateRaw = e220QueryCommand("AT+RATE=?"); radioInfoFetchStage = 6; radioInfoFetchNextMs = millis() + 20UL; return;
            case 6: powerRaw = e220QueryCommand("AT+POWER=?"); radioInfoFetchStage = 7; radioInfoFetchNextMs = millis() + 20UL; return;
            case 7: transRaw = e220QueryCommand("AT+TRANS=?"); radioInfoFetchStage = 8; radioInfoFetchNextMs = millis(); return;
            default:
                hc12ExitAtMode();
                radioInfoFetchPending = false;
                radioInfoFetchActive = false;
                radioInfoFetchStage = 0;
                radioInfoVersionText = e220ValueAfterEquals(modelRaw);
                if (radioInfoVersionText.isEmpty()) radioInfoVersionText = "No Reply";
                const String fwVal = e220ValueAfterEquals(fwRaw);
                hc12InfoValueText = radioInfoVersionText;
                hc12InfoSubText = fwVal.isEmpty() ? String("No firmware response") : String("FW ") + fwVal;
                radioInfoBaudText = e220ValueAfterEquals(uartRaw); if (radioInfoBaudText.isEmpty()) radioInfoBaudText = "--";
                radioInfoChannelText = e220ValueAfterEquals(channelRaw); if (radioInfoChannelText.isEmpty()) radioInfoChannelText = "--";
                radioInfoFuModeText = e220ValueAfterEquals(transRaw); if (radioInfoFuModeText.isEmpty()) radioInfoFuModeText = "--";
                radioInfoPowerText = e220ValueAfterEquals(powerRaw); if (radioInfoPowerText.isEmpty()) radioInfoPowerText = "--";
                radioInfoRawText = String("UART ") + (radioInfoBaudText == "--" ? String("--") : radioInfoBaudText) +
                                   " | RATE " + (e220ValueAfterEquals(rateRaw).isEmpty() ? String("--") : e220ValueAfterEquals(rateRaw)) +
                                   " | TRANS " + (radioInfoFuModeText == "--" ? String("--") : radioInfoFuModeText);
                applyRadioInfoLabels();
                if (uiScreen == UI_INFO) lvglRefreshInfoPanel(false);
                return;
        }
    }

    static String versionRaw;
    static String baudRaw;
    static String channelRaw;
    static String fuRaw;
    static String powerRaw;
    static String summaryRaw;
    switch (radioInfoFetchStage) {
        case 1: versionRaw = hc12QueryCommand("AT+V"); radioInfoFetchStage = 2; radioInfoFetchNextMs = millis(); return;
        case 2: baudRaw = hc12QueryCommand("AT+RB"); radioInfoFetchStage = 3; radioInfoFetchNextMs = millis(); return;
        case 3: channelRaw = hc12QueryCommand("AT+RC"); radioInfoFetchStage = 4; radioInfoFetchNextMs = millis(); return;
        case 4: fuRaw = hc12QueryCommand("AT+RF"); radioInfoFetchStage = 5; radioInfoFetchNextMs = millis(); return;
        case 5: powerRaw = hc12QueryCommand("AT+RP"); radioInfoFetchStage = 6; radioInfoFetchNextMs = millis(); return;
        case 6: summaryRaw = hc12QueryCommand("AT+RX", 220UL, 40UL); radioInfoFetchStage = 7; radioInfoFetchNextMs = millis(); return;
        default:
            hc12ExitAtMode();
            radioInfoFetchPending = false;
            radioInfoFetchActive = false;
            radioInfoFetchStage = 0;
            radioInfoVersionText = hc12CompactResponse(versionRaw);
            if (radioInfoVersionText.isEmpty()) radioInfoVersionText = "No Reply";
            radioInfoBaudText = hc12FieldValue(baudRaw, "B");
            if (radioInfoBaudText != "--" && radioInfoBaudText.indexOf("baud") < 0 && radioInfoBaudText.indexOf("bps") < 0) radioInfoBaudText += " bps";
            radioInfoChannelText = hc12FieldValue(channelRaw, "RC");
            if (radioInfoChannelText != "--") radioInfoChannelText = "CH" + radioInfoChannelText;
            radioInfoFuModeText = hc12FieldValue(fuRaw, "FU");
            if (radioInfoFuModeText != "--") radioInfoFuModeText = "FU" + radioInfoFuModeText;
            radioInfoPowerText = hc12FieldValue(powerRaw, "RP:");
            if (radioInfoPowerText == "--") radioInfoPowerText = hc12FieldValue(powerRaw, "RP");
            radioInfoRawText = hc12CompactResponse(summaryRaw);
            if (radioInfoRawText.isEmpty()) radioInfoRawText = "No summary response";
            hc12InfoValueText = radioInfoVersionText;
            hc12InfoSubText = radioInfoRawText;
            applyRadioInfoLabels();
            if (uiScreen == UI_INFO) lvglRefreshInfoPanel(false);
            return;
    }
}

static void queueHc12ConfigApply(Hc12ConfigApplyAction action, int target)
{
    if (action == HC12_CFG_APPLY_NONE) return;

    if (hc12ConfigApplyPending == HC12_CFG_APPLY_NONE) {
        hc12CommittedChannel = hc12CurrentChannel;
        hc12CommittedBaudIndex = hc12CurrentBaudIndex;
        hc12CommittedModeIndex = hc12CurrentModeIndex;
        hc12CommittedPowerLevel = hc12CurrentPowerLevel;
        e220CommittedChannel = e220CurrentChannel;
        e220CommittedBaudIndex = e220CurrentBaudIndex;
        e220CommittedAirRateIndex = e220CurrentAirRateIndex;
        e220CommittedPowerIndex = e220CurrentPowerIndex;
        e220CommittedFixedTransmission = e220CurrentFixedTransmission;
    }

    hc12ConfigApplyPending = action;
    hc12ConfigApplyTarget = target;
    hc12ConfigApplyDueMs = millis() + 12UL;

    switch (action) {
        case HC12_CFG_APPLY_CHANNEL:
            if (radioModuleType == RADIO_MODULE_HC12) hc12CurrentChannel = constrain(target, HC12_MIN_CHANNEL, HC12_MAX_CHANNEL);
            else e220CurrentChannel = constrain(target, E220_MIN_CHANNEL, E220_MAX_CHANNEL);
            hc12ConfigStatusText = "Applying channel...";
            break;
        case HC12_CFG_APPLY_BAUD:
            if (radioModuleType == RADIO_MODULE_HC12) hc12CurrentBaudIndex = constrain(target, 0, static_cast<int>(sizeof(HC12_SUPPORTED_BAUDS) / sizeof(HC12_SUPPORTED_BAUDS[0])) - 1);
            else e220CurrentBaudIndex = constrain(target, 0, static_cast<int>(sizeof(E220_UART_BAUD_VALUES) / sizeof(E220_UART_BAUD_VALUES[0])) - 1);
            hc12ConfigStatusText = "Applying baud...";
            break;
        case HC12_CFG_APPLY_MODE:
            if (radioModuleType == RADIO_MODULE_HC12) hc12CurrentModeIndex = constrain(target, 0, 3);
            else e220CurrentAirRateIndex = constrain(target, 0, static_cast<int>(sizeof(E220_AIR_RATE_OPTIONS) / sizeof(E220_AIR_RATE_OPTIONS[0])) - 1);
            hc12ConfigStatusText = "Applying mode...";
            break;
        case HC12_CFG_APPLY_POWER:
            if (radioModuleType == RADIO_MODULE_HC12) hc12CurrentPowerLevel = constrain(target, 1, 8);
            else e220CurrentPowerIndex = constrain(target, 0, 3);
            hc12ConfigStatusText = "Applying power...";
            break;
        case HC12_CFG_APPLY_EXTRA:
            e220CurrentFixedTransmission = target != 0;
            hc12ConfigStatusText = "Applying transfer mode...";
            break;
        default:
            break;
    }

    if (lvglReady && uiScreen == UI_CONFIG_HC12) lvglRefreshHc12ConfigUi();
}

static void serviceDeferredHc12ConfigApply()
{
    if (hc12ConfigApplyPending == HC12_CFG_APPLY_NONE) return;
    if (lvglTouchDown || wakeTouchReleaseGuard || lvglSwipeTracking || lvglSwipeVisualAnimating) return;
    if ((uiDeferredFlags & UI_DEFERRED_HC12_SETTLE_PENDING) &&
        static_cast<int16_t>(static_cast<uint16_t>(millis()) - hc12SettleUntilTick) < 0) return;
    if (static_cast<long>(millis() - hc12ConfigApplyDueMs) < 0) return;

    const Hc12ConfigApplyAction action = hc12ConfigApplyPending;
    const int target = hc12ConfigApplyTarget;
    hc12ConfigApplyPending = HC12_CFG_APPLY_NONE;

    bool ok = false;
    switch (action) {
        case HC12_CFG_APPLY_CHANNEL: ok = hc12ApplyChannel(target); break;
        case HC12_CFG_APPLY_BAUD: ok = hc12ApplyBaudIndex(target); break;
        case HC12_CFG_APPLY_MODE: ok = hc12ApplyModeIndex(target); break;
        case HC12_CFG_APPLY_POWER: ok = hc12ApplyPowerLevel(target); break;
        case HC12_CFG_APPLY_EXTRA: ok = e220ApplyTransferMode(target != 0); break;
        default: return;
    }

    if (!ok) {
        hc12CurrentChannel = hc12CommittedChannel;
        hc12CurrentBaudIndex = hc12CommittedBaudIndex;
        hc12CurrentModeIndex = hc12CommittedModeIndex;
        hc12CurrentPowerLevel = hc12CommittedPowerLevel;
        e220CurrentChannel = e220CommittedChannel;
        e220CurrentBaudIndex = e220CommittedBaudIndex;
        e220CurrentAirRateIndex = e220CommittedAirRateIndex;
        e220CurrentPowerIndex = e220CommittedPowerIndex;
        e220CurrentFixedTransmission = e220CommittedFixedTransmission;
        if (hc12ConfigStatusText.indexOf("reverted") < 0) hc12ConfigStatusText += " | reverted";
    }

    if (lvglReady && uiScreen == UI_CONFIG_HC12) lvglRefreshHc12ConfigUi();
}

static bool uiScreenSupportsSwipeBack(UiScreen screen)
{
    switch (screen) {
        case UI_CHAT:
        case UI_CHAT_PEERS:
        case UI_WIFI_LIST:
        case UI_MEDIA:
        case UI_INFO:
        case UI_GAMES:
        case UI_RADIO_CONTROL:
        case UI_CONFIG:
        case UI_CONFIG_REMOTE_CONTROL:
        case UI_CONFIG_RADIO_BUTTONS:
        case UI_CONFIG_RADIO_RECEIVER_TEST:
        case UI_CONFIG_BATTERY:
        case UI_CONFIG_STYLE:
        case UI_CONFIG_STYLE_HOME_ITEMS:
        case UI_CONFIG_LANGUAGE:
        case UI_CONFIG_OTA:
        case UI_CONFIG_MODULES:
        case UI_CONFIG_HC12:
        case UI_CONFIG_HC12_TERMINAL:
        case UI_CONFIG_HC12_INFO:
        case UI_CONFIG_MQTT_CONFIG:
        case UI_CONFIG_MQTT_CONTROLS:
        case UI_CONFIG_MQTT_BUTTONS:
        case UI_GAME_SNAKE:
        case UI_GAME_TETRIS:
        case UI_GAME_CHECKERS:
        case UI_GAME_SNAKE3D:
            return true;
        default:
            return false;
    }
}

lv_obj_t *lvglScreenForUi(UiScreen screen)
{
    switch (screen) {
        case UI_HOME: return lvglScrHome;
        case UI_LOCK: return lvglScrLock;
        case UI_CHAT: return lvglScrChat;
        case UI_CHAT_PEERS: return lvglScrChatPeers;
        case UI_WIFI_LIST: return lvglScrWifi;
        case UI_MEDIA: return lvglScrMedia;
        case UI_INFO: return lvglScrInfo;
        case UI_GAMES: return lvglScrGames;
        case UI_RADIO_CONTROL: return lvglScrRadioControl;
        case UI_CONFIG: return lvglScrConfig;
        case UI_CONFIG_REMOTE_CONTROL: return lvglScrRemoteControl;
        case UI_CONFIG_RADIO_BUTTONS: return lvglScrRadioButtonConfig;
        case UI_CONFIG_RADIO_RECEIVER_TEST: return lvglScrRadioReceiverTest;
        case UI_CONFIG_BATTERY: return lvglScrBatteryTrain;
        case UI_CONFIG_STYLE: return lvglScrStyle;
        case UI_CONFIG_STYLE_HOME_ITEMS: return lvglScrStyleHomeItems;
        case UI_CONFIG_LANGUAGE: return lvglScrLanguage;
        case UI_CONFIG_OTA: return lvglScrOta;
        case UI_CONFIG_MODULES: return lvglScrModules;
        case UI_CONFIG_HC12: return lvglScrHc12;
        case UI_CONFIG_HC12_TERMINAL: return lvglScrHc12Terminal;
        case UI_CONFIG_HC12_INFO: return lvglScrHc12Info;
        case UI_SCREENSAVER: return lvglScrScreensaver;
        case UI_CONFIG_MQTT_CONFIG: return lvglScrMqttCfg;
        case UI_CONFIG_MQTT_CONTROLS: return lvglScrMqttCtrl;
        case UI_CONFIG_MQTT_BUTTONS: return lvglScrMqttButtonConfig;
        case UI_GAME_SNAKE: return lvglScrSnake;
        case UI_GAME_TETRIS: return lvglScrTetris;
        case UI_GAME_CHECKERS: return lvglScrCheckers;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
        case UI_GAME_SNAKE3D: return lvglScrSnake3d;
#endif
        default: return nullptr;
    }
}

UiScreen lvglUiForScreenObj(lv_obj_t *screenObj)
{
    if (!screenObj || !lv_obj_is_valid(screenObj)) return uiScreen;
    if (screenObj == lvglScrHome) return UI_HOME;
    if (screenObj == lvglScrLock) return UI_LOCK;
    if (screenObj == lvglScrChat) return UI_CHAT;
    if (screenObj == lvglScrChatPeers) return UI_CHAT_PEERS;
    if (screenObj == lvglScrWifi) return UI_WIFI_LIST;
    if (screenObj == lvglScrMedia) return UI_MEDIA;
    if (screenObj == lvglScrInfo) return UI_INFO;
    if (screenObj == lvglScrGames) return UI_GAMES;
    if (screenObj == lvglScrRadioControl) return UI_RADIO_CONTROL;
    if (screenObj == lvglScrConfig) return UI_CONFIG;
    if (screenObj == lvglScrRemoteControl) return UI_CONFIG_REMOTE_CONTROL;
    if (screenObj == lvglScrRadioButtonConfig) return UI_CONFIG_RADIO_BUTTONS;
    if (screenObj == lvglScrRadioReceiverTest) return UI_CONFIG_RADIO_RECEIVER_TEST;
    if (screenObj == lvglScrBatteryTrain) return UI_CONFIG_BATTERY;
    if (screenObj == lvglScrStyle) return UI_CONFIG_STYLE;
    if (screenObj == lvglScrStyleHomeItems) return UI_CONFIG_STYLE_HOME_ITEMS;
    if (screenObj == lvglScrLanguage) return UI_CONFIG_LANGUAGE;
    if (screenObj == lvglScrOta) return UI_CONFIG_OTA;
    if (screenObj == lvglScrModules) return UI_CONFIG_MODULES;
    if (screenObj == lvglScrHc12) return UI_CONFIG_HC12;
    if (screenObj == lvglScrHc12Terminal) return UI_CONFIG_HC12_TERMINAL;
    if (screenObj == lvglScrHc12Info) return UI_CONFIG_HC12_INFO;
    if (screenObj == lvglScrScreensaver) return UI_SCREENSAVER;
    if (screenObj == lvglScrMqttCfg) return UI_CONFIG_MQTT_CONFIG;
    if (screenObj == lvglScrMqttCtrl) return UI_CONFIG_MQTT_CONTROLS;
    if (screenObj == lvglScrMqttButtonConfig) return UI_CONFIG_MQTT_BUTTONS;
    if (screenObj == lvglScrSnake) return UI_GAME_SNAKE;
    if (screenObj == lvglScrTetris) return UI_GAME_TETRIS;
    if (screenObj == lvglScrCheckers) return UI_GAME_CHECKERS;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    if (screenObj == lvglScrSnake3d) return UI_GAME_SNAKE3D;
#endif
    return uiScreen;
}

void lvglEnsureScreenBuilt(UiScreen screen)
{
    if (lvglScreenForUi(screen)) return;
    if (!moduleInstalledForScreen(screen)) return;
    if (!lvglCanBuildScreen(screen)) return;

    auto makeSmallBtn = [](lv_obj_t *parent, const char *txt, int w, int h, lv_color_t col, lv_event_cb_t cb, void *ud = nullptr) -> lv_obj_t * {
        if (!parent) return nullptr;
        lv_obj_t *b = lv_btn_create(parent);
        if (!b) return nullptr;
        lv_obj_set_size(b, w, h);
        lv_obj_set_style_radius(b, 8, 0);
        lv_obj_set_style_border_width(b, 0, 0);
        lv_obj_add_event_cb(b, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(b, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
        lv_obj_t *l = lv_label_create(b);
        if (l) {
            lv_label_set_text(l, txt);
            lv_obj_center(l);
        }
        lvglRegisterStyledButton(b, col, true);
        return b;
    };

    switch (screen) {
        case UI_HOME: {
            lvglScrHome = lvglCreateScreenBase("", false);
            lv_obj_t *homeWrap = lv_obj_create(lvglScrHome);
            lv_obj_set_size(homeWrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(homeWrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(homeWrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(homeWrap, 0, 0);
            lv_obj_set_style_pad_left(homeWrap, 10, 0);
            lv_obj_set_style_pad_right(homeWrap, 10, 0);
            lv_obj_set_style_pad_top(homeWrap, 8, 0);
            lv_obj_set_style_pad_bottom(homeWrap, 8, 0);
            lv_obj_set_style_pad_row(homeWrap, 8, 0);
            lv_obj_set_flex_flow(homeWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scrollbar_mode(homeWrap, LV_SCROLLBAR_MODE_OFF);
            lvglStatusLabel = nullptr;
            lvglHomeChatBtn = lvglCreateMenuButton(homeWrap, tr(TXT_CHAT), lv_color_hex(0x7A4F2F), lvglHomeNavEvent, reinterpret_cast<void *>(static_cast<intptr_t>(UI_CHAT)));
            lvglHomeRadioBtn = lvglCreateMenuButton(homeWrap, tr(TXT_RADIO_CONTROL), lv_color_hex(0x7A5C2E), lvglHomeNavEvent, reinterpret_cast<void *>(static_cast<intptr_t>(UI_RADIO_CONTROL)));
            lvglHomeMediaBtn = lvglCreateMenuButton(homeWrap, tr(TXT_MEDIA), lv_color_hex(0x376B93), lvglHomeNavEvent, reinterpret_cast<void *>(static_cast<intptr_t>(UI_MEDIA)));
            lvglHomeInfoBtn = lvglCreateMenuButton(homeWrap, tr(TXT_INFO), lv_color_hex(0x7750A0), lvglHomeNavEvent, reinterpret_cast<void *>(static_cast<intptr_t>(UI_INFO)));
            lvglHomeGamesBtn = lvglCreateMenuButton(homeWrap, tr(TXT_GAMES), lv_color_hex(0x2B7D7D), lvglHomeNavEvent, reinterpret_cast<void *>(static_cast<intptr_t>(UI_GAMES)));
            lvglHomeMqttControlsBtn = lvglCreateMenuButton(
                homeWrap,
                tr(TXT_MQTT_CONTROLS),
                lv_color_hex(0x2D6D8E),
                lvglHomeNavEvent,
                reinterpret_cast<void *>(static_cast<intptr_t>(UI_CONFIG_MQTT_CONTROLS))
            );
            lvglHomeConfigBtn = lvglCreateMenuButton(homeWrap, tr(TXT_CONFIG), lv_color_hex(0x925A73), lvglHomeNavEvent, reinterpret_cast<void *>(static_cast<intptr_t>(UI_CONFIG)));
            lvglHomePowerBtn = lvglCreateMenuButton(homeWrap, tr(TXT_POWER), lv_color_hex(0xA33F3F), lvglPowerButtonEvent, nullptr);
            lvglAirplaneBtn = lvglCreateMenuButton(homeWrap, tr(TXT_AIRPLANE_OFF), lv_color_hex(0x8A5A25), lvglAirplaneToggleEvent, nullptr);
            if (lvglAirplaneBtn) {
                lvglAirplaneBtnLabel = lv_obj_get_child(lvglAirplaneBtn, 0);
                lvglApplyAirplaneButtonStyle();
            }
            lvglApModeBtn = lvglCreateMenuButton(homeWrap, tr(TXT_AP_MODE_OFF), lv_color_hex(0xA66A2A), lvglApModeEvent, nullptr);
            if (lvglApModeBtn) {
                lvglApModeBtnLabel = lv_obj_get_child(lvglApModeBtn, 0);
                lvglApplyApModeButtonStyle();
            }
            lvglRefreshPrimaryMenuButtonIcons();
            lvglRegisterReorderableItem(lvglHomeChatBtn, "ord_home", "chat");
            lvglRegisterReorderableItem(lvglHomeRadioBtn, "ord_home", "gate");
            lvglRegisterReorderableItem(lvglHomeMediaBtn, "ord_home", "media");
            lvglRegisterReorderableItem(lvglHomeInfoBtn, "ord_home", "info");
            lvglRegisterReorderableItem(lvglHomeGamesBtn, "ord_home", "games");
            lvglRegisterReorderableItem(lvglHomeMqttControlsBtn, "ord_home", "mqtt");
            lvglRegisterReorderableItem(lvglHomeConfigBtn, "ord_home", "config");
            lvglRegisterReorderableItem(lvglHomePowerBtn, "ord_home", "power");
            lvglRegisterReorderableItem(lvglAirplaneBtn, "ord_home", "air");
            lvglRegisterReorderableItem(lvglApModeBtn, "ord_home", "ap");
            lvglApplySavedOrder(homeWrap, "ord_home");
            lvglRefreshHomeButtonVisibility();
            break;
        }
        case UI_LOCK: {
            lvglScrLock = lvglCreateScreenBase("Screen Lock", false);
            lv_obj_t *wrap = lv_obj_create(lvglScrLock);
            lv_obj_set_size(wrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(wrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_color(wrap, lv_color_hex(0x101922), 0);
            lv_obj_set_style_border_width(wrap, 0, 0);
            lv_obj_set_style_pad_all(wrap, 14, 0);
            lv_obj_set_style_pad_row(wrap, 10, 0);
            lv_obj_set_flex_flow(wrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_flex_align(wrap, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
            lv_obj_clear_flag(wrap, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *title = lv_label_create(wrap);
            lv_label_set_text(title, "Enter PIN to unlock");
            lv_obj_set_style_text_color(title, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_style_text_font(title, LV_FONT_DEFAULT, 0);

            lvglScreenLockPinTa = lv_textarea_create(wrap);
            lv_obj_set_width(lvglScreenLockPinTa, min<lv_coord_t>(DISPLAY_WIDTH - 44, 150));
            lv_obj_set_height(lvglScreenLockPinTa, 30);
            lvglConfigureSingleLinePasswordTa(lvglScreenLockPinTa);
            lv_textarea_set_max_length(lvglScreenLockPinTa, 4);
            lv_textarea_set_accepted_chars(lvglScreenLockPinTa, "0123456789");
            lv_textarea_set_placeholder_text(lvglScreenLockPinTa, "0000");
            lv_obj_set_style_text_align(lvglScreenLockPinTa, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_add_event_cb(lvglScreenLockPinTa, lvglTextAreaFocusEvent, LV_EVENT_FOCUSED, nullptr);
            lv_obj_add_event_cb(lvglScreenLockPinTa, lvglScreenLockPinChangedEvent, LV_EVENT_VALUE_CHANGED, nullptr);

            lvglScreenLockStatusLabel = lv_label_create(wrap);
            lv_obj_set_width(lvglScreenLockStatusLabel, lv_pct(100));
            lv_label_set_long_mode(lvglScreenLockStatusLabel, LV_LABEL_LONG_WRAP);
            lv_obj_set_style_text_align(lvglScreenLockStatusLabel, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_set_style_text_color(lvglScreenLockStatusLabel, lv_color_hex(0xB7C4D1), 0);

            lvglScreenLockUnlockBtn = makeSmallBtn(wrap, "Unlock", min<int>(DISPLAY_WIDTH - 44, 140), 38, lv_color_hex(0x2F7A45), lvglScreenLockUnlockEvent, nullptr);
            if (lvglScreenLockUnlockBtn) lvglScreenLockUnlockBtnLabel = lv_obj_get_child(lvglScreenLockUnlockBtn, 0);

            lvglScreenLockResetBtn = makeSmallBtn(wrap, tr(TXT_FACTORY_RESET), min<int>(DISPLAY_WIDTH - 44, 168), 34, lv_color_hex(0x8E2D2D), lvglScreenLockFactoryResetEvent, nullptr);
            if (lvglScreenLockResetBtn) {
                lvglScreenLockResetBtnLabel = lv_obj_get_child(lvglScreenLockResetBtn, 0);
                lv_obj_add_flag(lvglScreenLockResetBtn, LV_OBJ_FLAG_HIDDEN);
            }

            lvglRefreshLockScreenUi();
            break;
        }
        case UI_CHAT: {
            lvglScrChat = lvglCreateScreenBase("Chat", true);
            lv_obj_t *chatOps = lv_obj_create(lvglScrChat);
            lv_obj_set_size(chatOps, lv_pct(100), 40);
            lv_obj_align(chatOps, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(chatOps, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(chatOps, 0, 0);
            lv_obj_set_style_pad_all(chatOps, 6, 0);
            lv_obj_set_style_pad_column(chatOps, 6, 0);
            lv_obj_set_flex_flow(chatOps, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(chatOps, LV_OBJ_FLAG_SCROLLABLE);
            lvglChatPeersBtn = makeSmallBtn(chatOps, lvglSymbolText(LV_SYMBOL_LIST, "Peers").c_str(), 82, 28, lv_color_hex(0x2F6D86), lvglOpenChatPeersEvent);

            lvglChatContactLabel = lv_label_create(chatOps);
            lv_obj_set_width(lvglChatContactLabel, lv_pct(100));
            lv_obj_set_flex_grow(lvglChatContactLabel, 1);
            lv_obj_set_style_text_align(lvglChatContactLabel, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_set_style_text_color(lvglChatContactLabel, lv_color_hex(0xC8D3DD), 0);
            lv_label_set_long_mode(lvglChatContactLabel, LV_LABEL_LONG_DOT);

            lvglChatMenuBtn = makeSmallBtn(chatOps, LV_SYMBOL_LIST, 34, 28, lv_color_hex(0x2F6D86), lvglToggleChatMenuEvent, nullptr);
            lvglChatDiscoveryBtn = makeSmallBtn(chatOps,
                                                lvglSymbolText(LV_SYMBOL_WIFI, tr(TXT_DISCOVERY)).c_str(),
                                                102,
                                                28,
                                                p2pDiscoveryEnabled ? lv_color_hex(0x3A7A3A) : lv_color_hex(0x4E5D6C),
                                                lvglChatDiscoveryToggleEvent);
            if (lvglChatDiscoveryBtn) {
                lvglApplyChatDiscoveryButtonStyle();
            }

            lvglChatMenuBackdrop = lv_obj_create(lvglScrChat);
            lv_obj_set_size(lvglChatMenuBackdrop, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(lvglChatMenuBackdrop, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(lvglChatMenuBackdrop, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(lvglChatMenuBackdrop, 0, 0);
            lv_obj_set_style_pad_all(lvglChatMenuBackdrop, 0, 0);
            lv_obj_add_flag(lvglChatMenuBackdrop, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(lvglChatMenuBackdrop, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_event_cb(
                lvglChatMenuBackdrop,
                [](lv_event_t *e) {
                    (void)e;
                    lvglHideChatMenu();
                },
                LV_EVENT_CLICKED,
                nullptr);

            lvglChatMenuPanel = lv_obj_create(lvglScrChat);
            lv_obj_set_size(lvglChatMenuPanel, lv_pct(50), LV_SIZE_CONTENT);
            lv_obj_align(lvglChatMenuPanel, LV_ALIGN_TOP_RIGHT, -8, UI_CONTENT_TOP_Y + 40);
            lv_obj_set_style_bg_color(lvglChatMenuPanel, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_color(lvglChatMenuPanel, lv_color_hex(0x536274), 0);
            lv_obj_set_style_border_width(lvglChatMenuPanel, 1, 0);
            lv_obj_set_style_radius(lvglChatMenuPanel, 12, 0);
            lv_obj_set_style_pad_all(lvglChatMenuPanel, 8, 0);
            lv_obj_set_style_pad_row(lvglChatMenuPanel, 6, 0);
            lv_obj_set_flex_flow(lvglChatMenuPanel, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_flex_align(lvglChatMenuPanel, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
            lv_obj_add_flag(lvglChatMenuPanel, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(lvglChatMenuPanel, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_t *chatClearBtn = makeSmallBtn(lvglChatMenuPanel, lvglSymbolText(LV_SYMBOL_TRASH, "Clear").c_str(), lv_pct(100), 32, lv_color_hex(0x8A3A3A), lvglDeleteChatConversationEvent, nullptr);
            if (chatClearBtn) lv_obj_set_width(chatClearBtn, lv_pct(100));
            lv_obj_t *chatClearAllBtn = makeSmallBtn(lvglChatMenuPanel, lvglSymbolText(LV_SYMBOL_TRASH, "Clear for All").c_str(), lv_pct(100), 32, lv_color_hex(0x944E2B), lvglDeleteChatConversationForAllEvent, nullptr);
            if (chatClearAllBtn) lv_obj_set_width(chatClearAllBtn, lv_pct(100));

            lvglChatContacts = lv_obj_create(lvglScrChat);
            lv_obj_set_size(lvglChatContacts, lv_pct(100), UI_CONTENT_H - 44);
            lv_obj_align(lvglChatContacts, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y + 44);
            lv_obj_set_style_bg_opa(lvglChatContacts, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(lvglChatContacts, 0, 0);
            lv_obj_set_style_pad_all(lvglChatContacts, 6, 0);
            lv_obj_set_style_pad_row(lvglChatContacts, 6, 0);
            lv_obj_set_style_pad_column(lvglChatContacts, 0, 0);
            lv_obj_set_flex_flow(lvglChatContacts, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scroll_dir(lvglChatContacts, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(lvglChatContacts, LV_SCROLLBAR_MODE_OFF);

            lvglChatList = lv_obj_create(lvglScrChat);
            lv_obj_set_size(lvglChatList, lv_pct(100), UI_CONTENT_H - 98);
            lv_obj_align(lvglChatList, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y + 44);
            lv_obj_set_style_bg_color(lvglChatList, lv_color_hex(0x111922), 0);
            lv_obj_set_style_border_width(lvglChatList, 0, 0);
            lv_obj_set_style_pad_all(lvglChatList, 8, 0);
            lv_obj_set_style_pad_row(lvglChatList, 6, 0);
            lv_obj_set_flex_flow(lvglChatList, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scroll_dir(lvglChatList, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(lvglChatList, LV_SCROLLBAR_MODE_OFF);

            lvglChatComposer = lv_obj_create(lvglScrChat);
            lv_obj_set_size(lvglChatComposer, lv_pct(100), 56);
            lv_obj_align(lvglChatComposer, LV_ALIGN_BOTTOM_MID, 0, 0);
            lv_obj_set_style_bg_color(lvglChatComposer, lv_color_hex(0x16212C), 0);
            lv_obj_set_style_border_width(lvglChatComposer, 0, 0);
            lv_obj_set_style_pad_all(lvglChatComposer, 8, 0);
            lv_obj_set_style_pad_column(lvglChatComposer, 8, 0);
            lv_obj_set_flex_flow(lvglChatComposer, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(lvglChatComposer, LV_OBJ_FLAG_SCROLLABLE);

            lvglChatInputTa = lv_textarea_create(lvglChatComposer);
            lv_obj_set_height(lvglChatInputTa, 38);
            lv_obj_set_width(lvglChatInputTa, DISPLAY_WIDTH - 96);
            lv_obj_set_flex_grow(lvglChatInputTa, 1);
            lv_textarea_set_one_line(lvglChatInputTa, true);
            lv_textarea_set_placeholder_text(lvglChatInputTa, "Message");
            lv_obj_add_event_cb(lvglChatInputTa, lvglTextAreaFocusEvent, LV_EVENT_FOCUSED, nullptr);

            makeSmallBtn(lvglChatComposer, lvglSymbolText(LV_SYMBOL_UPLOAD, "Send").c_str(), 82, 38, lv_color_hex(0x3A7A3A),
                         [](lv_event_t *e) {
                             (void)e;
                             if (!lvglChatInputTa) return;
                             if (lvglChatPromptIfAirplaneBlocked()) return;
                             const char *raw = lv_textarea_get_text(lvglChatInputTa);
                             String text = raw ? String(raw) : String("");
                             text.trim();
                             if (text.isEmpty()) {
                                 uiStatusLine = "Type a message first";
                                 lvglSyncStatusLine();
                                 return;
                             }
                            if (currentChatPeerKey.isEmpty()) {
                                uiStatusLine = "Select a contact first";
                                lvglSyncStatusLine();
                                return;
                            }
                            chatSendAndStoreMessage(currentChatPeerKey, text);
                             lv_textarea_set_text(lvglChatInputTa, "");
                             if (lvglKb) {
                                 lv_keyboard_set_textarea(lvglKb, nullptr);
                                 lv_obj_add_flag(lvglKb, LV_OBJ_FLAG_HIDDEN);
                             }
                             lvglSetChatKeyboardVisible(false);
                         });
            lvglRefreshChatLayout();
            lvglRefreshChatUi();
            break;
        }
        case UI_CHAT_PEERS: {
            lvglScrChatPeers = lvglCreateScreenBase("Chat Peers", false);
            static constexpr int16_t CHAT_PEERS_TOP_H = 96;

            lv_obj_t *peerTop = lv_obj_create(lvglScrChatPeers);
            lv_obj_set_size(peerTop, lv_pct(100), CHAT_PEERS_TOP_H);
            lv_obj_align(peerTop, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_color(peerTop, lv_color_hex(0x16212C), 0);
            lv_obj_set_style_border_width(peerTop, 0, 0);
            lv_obj_set_style_pad_left(peerTop, 8, 0);
            lv_obj_set_style_pad_right(peerTop, 8, 0);
            lv_obj_set_style_pad_top(peerTop, 6, 0);
            lv_obj_set_style_pad_bottom(peerTop, 6, 0);
            lv_obj_set_style_pad_row(peerTop, 6, 0);
            lv_obj_set_flex_flow(peerTop, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(peerTop, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *peerOps = lv_obj_create(peerTop);
            lv_obj_set_size(peerOps, lv_pct(100), 30);
            lv_obj_set_style_bg_opa(peerOps, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(peerOps, 0, 0);
            lv_obj_set_style_pad_all(peerOps, 0, 0);
            lv_obj_set_style_pad_column(peerOps, 6, 0);
            lv_obj_set_flex_flow(peerOps, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(peerOps, LV_OBJ_FLAG_SCROLLABLE);

            lvglChatPeerScanBtn = makeSmallBtn(
                peerOps,
                lvglSymbolText(LV_SYMBOL_REFRESH, tr(TXT_SCAN)).c_str(),
                74, 26,
                lv_color_hex(0x2F6D86),
                lvglChatPeerActionEvent,
                reinterpret_cast<void *>(static_cast<intptr_t>(0))
            );

            lvglChatPeerIdentityLabel = lv_label_create(peerTop);
            lv_obj_set_width(lvglChatPeerIdentityLabel, lv_pct(100));
            lv_label_set_long_mode(lvglChatPeerIdentityLabel, LV_LABEL_LONG_WRAP);
            lv_obj_set_style_text_color(lvglChatPeerIdentityLabel, lv_color_hex(0xC8D3DD), 0);
            lv_obj_set_style_text_line_space(lvglChatPeerIdentityLabel, 2, 0);

            lvglChatPeerList = lv_obj_create(lvglScrChatPeers);
            lv_obj_set_size(lvglChatPeerList, lv_pct(100), UI_CONTENT_H - CHAT_PEERS_TOP_H);
            lv_obj_align(lvglChatPeerList, LV_ALIGN_BOTTOM_MID, 0, 0);
            lv_obj_set_style_bg_color(lvglChatPeerList, lv_color_hex(0x111922), 0);
            lv_obj_set_style_border_width(lvglChatPeerList, 0, 0);
            lv_obj_set_style_pad_all(lvglChatPeerList, 8, 0);
            lv_obj_set_style_pad_row(lvglChatPeerList, 6, 0);
            lv_obj_set_flex_flow(lvglChatPeerList, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scroll_dir(lvglChatPeerList, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(lvglChatPeerList, LV_SCROLLBAR_MODE_OFF);

            lvglRefreshChatPeerUi();
            break;
        }
        case UI_WIFI_LIST: {
            lvglScrWifi = lvglCreateScreenBase("WiFi Config", true);
            lvglWifiList = lv_obj_create(lvglScrWifi);
            lv_obj_set_size(lvglWifiList, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(lvglWifiList, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(lvglWifiList, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(lvglWifiList, 0, 0);
            lv_obj_set_style_pad_all(lvglWifiList, 8, 0);
            lv_obj_set_style_pad_row(lvglWifiList, 6, 0);
            lv_obj_set_flex_flow(lvglWifiList, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scroll_dir(lvglWifiList, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(lvglWifiList, LV_SCROLLBAR_MODE_OFF);
            break;
        }
        case UI_MEDIA: {
            lvglScrMedia = lvglCreateScreenBase("Media", true);
            lvglMediaPlayerPanel = lv_obj_create(lvglScrMedia);
            lv_obj_set_size(lvglMediaPlayerPanel, lv_pct(100), MEDIA_PLAYER_PANEL_H);
            lv_obj_align(lvglMediaPlayerPanel, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(lvglMediaPlayerPanel, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(lvglMediaPlayerPanel, 0, 0);
            lv_obj_set_style_pad_left(lvglMediaPlayerPanel, 8, 0);
            lv_obj_set_style_pad_right(lvglMediaPlayerPanel, 8, 0);
            lv_obj_set_style_pad_top(lvglMediaPlayerPanel, 6, 0);
            lv_obj_set_style_pad_bottom(lvglMediaPlayerPanel, 6, 0);
            lv_obj_set_style_pad_row(lvglMediaPlayerPanel, 5, 0);
            lv_obj_set_flex_flow(lvglMediaPlayerPanel, LV_FLEX_FLOW_COLUMN);
            lv_obj_t *mediaCtrlRow = lv_obj_create(lvglMediaPlayerPanel);
            lv_obj_set_size(mediaCtrlRow, lv_pct(100), 28);
            lv_obj_set_style_bg_opa(mediaCtrlRow, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(mediaCtrlRow, 0, 0);
            lv_obj_set_style_pad_all(mediaCtrlRow, 0, 0);
            lv_obj_set_style_pad_column(mediaCtrlRow, 4, 0);
            lv_obj_set_flex_flow(mediaCtrlRow, LV_FLEX_FLOW_ROW);
            lv_obj_set_scrollbar_mode(mediaCtrlRow, LV_SCROLLBAR_MODE_OFF);
            makeSmallBtn(mediaCtrlRow, lvglSymbolText(LV_SYMBOL_REFRESH, "Refresh").c_str(), 82, 24, lv_color_hex(0x2F6D86), lvglMediaRefreshEvent);
            lvglMediaPrevBtn = makeSmallBtn(mediaCtrlRow, lvglSymbolText(LV_SYMBOL_PREV, "Prev").c_str(), 60, 24, lv_color_hex(0x355C3D), lvglMediaPrevTrackEvent);
            lvglMediaPlayBtn = makeSmallBtn(mediaCtrlRow, lvglSymbolText(LV_SYMBOL_PLAY, "Play").c_str(), 60, 24, lv_color_hex(0x7C3A3A), lvglMediaPlayStopEvent);
            lvglMediaNextBtn = makeSmallBtn(mediaCtrlRow, lvglSymbolText(LV_SYMBOL_NEXT, "Next").c_str(), 60, 24, lv_color_hex(0x355C3D), lvglMediaNextTrackEvent);
            if (lvglMediaPlayBtn) lvglMediaPlayBtnLabel = lv_obj_get_child(lvglMediaPlayBtn, 0);
            lvglMediaTrackLabel = lv_label_create(lvglMediaPlayerPanel);
            lv_obj_set_width(lvglMediaTrackLabel, lv_pct(100));
            lv_obj_set_style_text_color(lvglMediaTrackLabel, lv_color_hex(0xDDE6F0), 0);
            lv_label_set_long_mode(lvglMediaTrackLabel, LV_LABEL_LONG_CLIP);
            lv_label_set_text(lvglMediaTrackLabel, "Track: No track selected");
            lv_obj_t *mediaVolRow = lv_obj_create(lvglMediaPlayerPanel);
            lv_obj_set_size(mediaVolRow, lv_pct(100), 24);
            lv_obj_set_style_bg_opa(mediaVolRow, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(mediaVolRow, 0, 0);
            lv_obj_set_style_pad_all(mediaVolRow, 0, 0);
            lv_obj_set_style_pad_column(mediaVolRow, 4, 0);
            lv_obj_set_flex_flow(mediaVolRow, LV_FLEX_FLOW_ROW);
            makeSmallBtn(mediaVolRow, "-", 22, 22, lv_color_hex(0x4A5563), lvglMediaVolumeStepEvent, reinterpret_cast<void *>(static_cast<intptr_t>(-5)));
            lvglMediaVolSlider = lv_slider_create(mediaVolRow);
            lv_obj_set_size(lvglMediaVolSlider, 120, 20);
            lv_slider_set_range(lvglMediaVolSlider, 0, 100);
            lv_slider_set_value(lvglMediaVolSlider, mediaVolumePercent, LV_ANIM_OFF);
            lv_obj_add_event_cb(lvglMediaVolSlider, lvglMediaVolumeEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            makeSmallBtn(mediaVolRow, "+", 22, 22, lv_color_hex(0x4A5563), lvglMediaVolumeStepEvent, reinterpret_cast<void *>(static_cast<intptr_t>(5)));
            lvglMediaVolValueLabel = lv_label_create(mediaVolRow);
            lv_label_set_text(lvglMediaVolValueLabel, "50%");
            lv_obj_set_style_text_color(lvglMediaVolValueLabel, lv_color_hex(0xDDE6F0), 0);
            lvglMediaProgressBar = lv_bar_create(lvglMediaPlayerPanel);
            lv_obj_set_size(lvglMediaProgressBar, lv_pct(100), 8);
            lv_bar_set_range(lvglMediaProgressBar, 0, 1000);
            lv_bar_set_value(lvglMediaProgressBar, 0, LV_ANIM_OFF);
            lv_obj_set_style_bg_color(lvglMediaProgressBar, lv_color_hex(0x2A3340), 0);
            lv_obj_set_style_bg_color(lvglMediaProgressBar, lv_color_hex(0x52B788), LV_PART_INDICATOR);
            lvglMediaProgressLabel = lv_label_create(lvglMediaPlayerPanel);
            lv_obj_set_width(lvglMediaProgressLabel, lv_pct(100));
            lv_obj_set_style_text_align(lvglMediaProgressLabel, LV_TEXT_ALIGN_RIGHT, 0);
            lv_obj_set_style_text_color(lvglMediaProgressLabel, lv_color_hex(0xB7C4D1), 0);
            lv_label_set_text(lvglMediaProgressLabel, "00:00 / 00:00");
            lvglMediaList = lv_obj_create(lvglScrMedia);
            lv_obj_set_size(lvglMediaList, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(lvglMediaList, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_color(lvglMediaList, lv_color_hex(0x111922), 0);
            lv_obj_set_style_border_width(lvglMediaList, 0, 0);
            lv_obj_set_style_pad_all(lvglMediaList, 8, 0);
            lv_obj_set_style_pad_row(lvglMediaList, 6, 0);
            lv_obj_set_flex_flow(lvglMediaList, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scroll_dir(lvglMediaList, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(lvglMediaList, LV_SCROLLBAR_MODE_OFF);
            lv_obj_add_flag(lvglMediaPlayerPanel, LV_OBJ_FLAG_HIDDEN);
            lvglMediaPlayerVisible = false;
            lvglRefreshMediaLayout();
            lv_obj_t *lbl = lv_label_create(lvglMediaList);
            lv_label_set_text(lbl, "Open Media screen to scan SD");
            lv_obj_set_style_text_color(lbl, lv_color_hex(0xC8CED6), 0);
            break;
        }
        case UI_INFO: {
            lvglScrInfo = lvglCreateScreenBase(tr(TXT_INFO), true);
            lvglInfoList = lv_obj_create(lvglScrInfo);
            lvglInfoBatteryCard = nullptr;
            lvglInfoWifiCard = nullptr;
            lvglInfoRadioCard = nullptr;
            lvglInfoLightCard = nullptr;
            lvglInfoSdCard = nullptr;
            lv_obj_set_size(lvglInfoList, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(lvglInfoList, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(lvglInfoList, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(lvglInfoList, 0, 0);
            lv_obj_set_style_pad_all(lvglInfoList, 8, 0);
            lv_obj_set_style_pad_row(lvglInfoList, 6, 0);
            lv_obj_set_flex_flow(lvglInfoList, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scroll_dir(lvglInfoList, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(lvglInfoList, LV_SCROLLBAR_MODE_OFF);

            {
                lv_obj_t *tmp = nullptr;
                lvglInfoBatteryCard = lvglCreateInfoCard(lvglInfoList, LV_SYMBOL_BATTERY_FULL, tr(TXT_BATTERY), lv_color_hex(0x52B788),
                                                         &lvglInfoBatteryValueLabel, &lvglInfoBatterySubLabel, &tmp);
                lvglRegisterReorderableItem(lvglInfoBatteryCard, "ord_info", "battery");
            }
            {
                lv_obj_t *tmp = nullptr;
                lvglInfoWifiCard = lvglCreateInfoCard(lvglInfoList, LV_SYMBOL_WIFI, "WiFi Strength", lv_color_hex(0x4FC3F7),
                                                      &lvglInfoWifiValueLabel, &lvglInfoWifiSubLabel, &tmp);
                lvglRegisterReorderableItem(lvglInfoWifiCard, "ord_info", "wifi");
            }
            lvglInfoRadioCard = lvglCreateInfoCard(lvglInfoList, LV_SYMBOL_SETTINGS, tr(TXT_HC12_INFO), lv_color_hex(0x7A5C2E),
                                                   &lvglInfoHc12ValueLabel, &lvglInfoHc12SubLabel);
            lvglRegisterReorderableItem(lvglInfoRadioCard, "ord_info", "hc12");
            {
                lv_obj_t *tmp = nullptr;
                lvglInfoLightCard = lvglCreateInfoCard(lvglInfoList, LV_SYMBOL_EYE_OPEN, "Lighting", lv_color_hex(0xF4B942),
                                                       &lvglInfoLightValueLabel, &lvglInfoLightSubLabel, &tmp);
                lvglRegisterReorderableItem(lvglInfoLightCard, "ord_info", "light");
            }
            {
                lv_obj_t *tmp = nullptr;
                lvglInfoSdCard = lvglCreateInfoCard(lvglInfoList, LV_SYMBOL_SD_CARD, "SD Card", lv_color_hex(0xE59F45),
                                                    nullptr, nullptr, &tmp);
                lvglRegisterReorderableItem(lvglInfoSdCard, "ord_info", "sd");
            }
            {
                lv_obj_t *tmp = nullptr;
                lvglInfoSramCard = lvglCreateInfoCard(lvglInfoList, LV_SYMBOL_DIRECTORY, "SRAM", lv_color_hex(0x3FA7D6),
                                                      nullptr, nullptr, &tmp);
                lvglInfoPsramCard = lvglCreateInfoCard(lvglInfoList, LV_SYMBOL_DRIVE, "PSRAM", lv_color_hex(0x9B7CF2),
                                                       nullptr, nullptr, &tmp);
                lvglRegisterReorderableItem(lvglInfoSramCard, "ord_info", "sram");
                lvglRegisterReorderableItem(lvglInfoPsramCard, "ord_info", "psram");
            }
            {
                lv_obj_t *tmp = nullptr;
                lvglInfoCpuCard = lvglCreateInfoCard(lvglInfoList, LV_SYMBOL_CHARGE, "Processor Load", lv_color_hex(0x7BC96F),
                                                     nullptr, nullptr, &tmp);
                lvglInfoTempCard = lvglCreateInfoCard(lvglInfoList, LV_SYMBOL_WARNING, "Chip Temperature", lv_color_hex(0xF2C35E),
                                                      nullptr, nullptr, &tmp);
                lvglRegisterReorderableItem(lvglInfoCpuCard, "ord_info", "cpu");
                lvglRegisterReorderableItem(lvglInfoTempCard, "ord_info", "temp");
            }

            lv_obj_t *systemCard = lv_obj_create(lvglInfoList);
            lv_obj_set_width(systemCard, lv_pct(100));
            lv_obj_set_height(systemCard, LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(systemCard, lv_color_hex(0x16212C), 0);
            lv_obj_set_style_border_color(systemCard, lv_color_hex(0x536274), 0);
            lv_obj_set_style_border_width(systemCard, 1, 0);
            lv_obj_set_style_radius(systemCard, 12, 0);
            lv_obj_set_style_pad_all(systemCard, 10, 0);
            lv_obj_clear_flag(systemCard, LV_OBJ_FLAG_SCROLLABLE);

            lvglInfoSystemLabel = lv_label_create(systemCard);
            lv_obj_set_width(lvglInfoSystemLabel, lv_pct(100));
            lv_label_set_long_mode(lvglInfoSystemLabel, LV_LABEL_LONG_WRAP);
            lv_obj_set_style_text_color(lvglInfoSystemLabel, lv_color_hex(0xC8D3DD), 0);
            lvglRegisterReorderableItem(systemCard, "ord_info", "system");
            lvglApplySavedOrder(lvglInfoList, "ord_info");
            break;
        }
        case UI_GAMES: {
            lvglScrGames = lvglCreateScreenBase("Games", true);
            lv_obj_t *gamesWrap = lv_obj_create(lvglScrGames);
            lv_obj_set_size(gamesWrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(gamesWrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(gamesWrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(gamesWrap, 0, 0);
            lv_obj_set_style_pad_all(gamesWrap, 10, 0);
            lv_obj_set_style_pad_row(gamesWrap, 10, 0);
            lv_obj_set_flex_flow(gamesWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scrollbar_mode(gamesWrap, LV_SCROLLBAR_MODE_OFF);
            lv_obj_t *gamesSnakeBtn = lvglCreateMenuButton(gamesWrap, lvglSymbolText(LV_SYMBOL_PLAY, "Snake").c_str(), lv_color_hex(0x3A8F4B), lvglOpenSnakeEvent, nullptr);
            lv_obj_t *gamesTetrisBtn = lvglCreateMenuButton(gamesWrap, lvglSymbolText(LV_SYMBOL_PLAY, "Tetris").c_str(), lv_color_hex(0x376B93), lvglOpenTetrisEvent, nullptr);
            lv_obj_t *gamesCheckersBtn = lvglCreateMenuButton(gamesWrap, lvglSymbolText(LV_SYMBOL_PLAY, "Checkers").c_str(), lv_color_hex(0x8A5A25), lvglOpenCheckersEvent, nullptr);
#if defined(BOARD_ESP32S3_3248S035_N16R8)
            lv_obj_t *gamesSnake3dBtn = lvglCreateMenuButton(gamesWrap, lvglSymbolText(LV_SYMBOL_PLAY, "Snake 3D").c_str(), lv_color_hex(0x5A4CC7), lvglOpenSnake3dEvent, nullptr);
            lvglRegisterReorderableItem(gamesSnake3dBtn, "ord_games", "snake3d");
#endif
            lvglRegisterReorderableItem(gamesSnakeBtn, "ord_games", "snake");
            lvglRegisterReorderableItem(gamesTetrisBtn, "ord_games", "tetris");
            lvglRegisterReorderableItem(gamesCheckersBtn, "ord_games", "checkers");
            lvglApplySavedOrder(gamesWrap, "ord_games");
            break;
        }
        case UI_RADIO_CONTROL: {
            lvglScrRadioControl = lvglCreateScreenBase(tr(TXT_RADIO_CONTROL), true);
            lv_obj_t *wrap = lv_obj_create(lvglScrRadioControl);
            lv_obj_set_size(wrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(wrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(wrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(wrap, 0, 0);
            lv_obj_set_style_pad_all(wrap, 10, 0);
            lv_obj_set_style_pad_row(wrap, 10, 0);
            lv_obj_set_flex_flow(wrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scroll_dir(wrap, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(wrap, LV_SCROLLBAR_MODE_OFF);
            lvglRadioControlConfigBtn = lvglCreateMenuButton(wrap, tr(TXT_BUTTON_CONFIG), lv_color_hex(0x7A5C2E), lvglOpenRadioButtonConfigScreenEvent, nullptr);
            lvglRadioControlConfigBtnLabel = lvglRadioControlConfigBtn ? lv_obj_get_child(lvglRadioControlConfigBtn, 0) : nullptr;
            lvglRadioControlList = lv_obj_create(wrap);
            lv_obj_set_size(lvglRadioControlList, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(lvglRadioControlList, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(lvglRadioControlList, 0, 0);
            lv_obj_set_style_pad_all(lvglRadioControlList, 0, 0);
            lv_obj_set_style_pad_row(lvglRadioControlList, 10, 0);
            lv_obj_set_flex_flow(lvglRadioControlList, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(lvglRadioControlList, LV_OBJ_FLAG_SCROLLABLE);

            lvglRefreshRadioControlUi();
            break;
        }
        case UI_CONFIG_RADIO_BUTTONS: {
            lvglScrRadioButtonConfig = lvglCreateScreenBase(tr(TXT_BUTTON_CONFIG), true);
            lv_obj_t *wrap = lv_obj_create(lvglScrRadioButtonConfig);
            lv_obj_set_size(wrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(wrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(wrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(wrap, 0, 0);
            lv_obj_set_style_pad_all(wrap, 10, 0);
            lv_obj_set_style_pad_row(wrap, 10, 0);
            lv_obj_set_flex_flow(wrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scrollbar_mode(wrap, LV_SCROLLBAR_MODE_OFF);
#if defined(BOARD_ESP32_2432S024C)
            static constexpr lv_coord_t RADIO_BUTTON_CONFIG_WIDE_W = 360;
            lv_obj_set_scroll_dir(wrap, LV_DIR_HOR | LV_DIR_VER);
#else
            lv_obj_clear_flag(wrap, LV_OBJ_FLAG_SCROLLABLE);
#endif

            lv_obj_t *topRow = lv_obj_create(wrap);
#if defined(BOARD_ESP32_2432S024C)
            lv_obj_set_size(topRow, RADIO_BUTTON_CONFIG_WIDE_W, LV_SIZE_CONTENT);
#else
            lv_obj_set_size(topRow, lv_pct(100), LV_SIZE_CONTENT);
#endif
            lv_obj_set_style_bg_opa(topRow, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(topRow, 0, 0);
            lv_obj_set_style_pad_all(topRow, 0, 0);
            lv_obj_set_style_pad_column(topRow, 6, 0);
            lv_obj_set_flex_flow(topRow, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(topRow, LV_OBJ_FLAG_SCROLLABLE);

            lvglRadioButtonAddBtn = makeSmallBtn(topRow, "+", 44, 30, lv_color_hex(0x2F7A45), lvglRadioButtonAddEvent, nullptr);
            lvglRadioButtonSelBtn = makeSmallBtn(topRow, "Sel", 54, 30, lv_color_hex(0x2F6D86), lvglRadioButtonSelectModeEvent, nullptr);
            lvglRadioButtonDelBtn = makeSmallBtn(topRow, "Del", 54, 30, lv_color_hex(0x8A3A3A), lvglRadioButtonDeletePromptEvent, nullptr);

            lv_obj_t *headerRow = lv_obj_create(wrap);
#if defined(BOARD_ESP32_2432S024C)
            lv_obj_set_size(headerRow, RADIO_BUTTON_CONFIG_WIDE_W, LV_SIZE_CONTENT);
#else
            lv_obj_set_size(headerRow, lv_pct(100), LV_SIZE_CONTENT);
#endif
            lv_obj_set_style_bg_opa(headerRow, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(headerRow, 0, 0);
            lv_obj_set_style_pad_all(headerRow, 0, 0);
            lv_obj_set_style_pad_column(headerRow, 6, 0);
            lv_obj_set_flex_flow(headerRow, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(headerRow, LV_OBJ_FLAG_SCROLLABLE);

            auto makeHeaderLabel = [&](const char *txt, lv_coord_t w) {
                lv_obj_t *lbl = lv_label_create(headerRow);
                lv_obj_set_width(lbl, w);
                lv_label_set_text(lbl, txt);
                lv_obj_set_style_text_color(lbl, lv_color_hex(0xAFC3D7), 0);
                lv_obj_set_style_text_font(lbl, LV_FONT_DEFAULT, 0);
            };

            makeHeaderLabel("", 24);
            makeHeaderLabel("Name", 78);
            makeHeaderLabel("Dest.", 46);
            makeHeaderLabel("ON", 34);
            makeHeaderLabel("OFF", 34);
            makeHeaderLabel("Tgl", 36);
            makeHeaderLabel("Conf", 44);

            lvglRadioButtonConfigList = lv_obj_create(wrap);
#if defined(BOARD_ESP32_2432S024C)
            lv_obj_set_size(lvglRadioButtonConfigList, RADIO_BUTTON_CONFIG_WIDE_W, UI_CONTENT_H - 144);
#else
            lv_obj_set_size(lvglRadioButtonConfigList, lv_pct(100), UI_CONTENT_H - 126);
#endif
            lv_obj_set_style_bg_opa(lvglRadioButtonConfigList, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(lvglRadioButtonConfigList, 0, 0);
            lv_obj_set_style_pad_all(lvglRadioButtonConfigList, 0, 0);
            lv_obj_set_style_pad_row(lvglRadioButtonConfigList, 8, 0);
            lv_obj_set_flex_flow(lvglRadioButtonConfigList, LV_FLEX_FLOW_COLUMN);
#if defined(BOARD_ESP32_2432S024C)
            lv_obj_set_scroll_dir(lvglRadioButtonConfigList, LV_DIR_ALL);
#else
            lv_obj_set_scroll_dir(lvglRadioButtonConfigList, LV_DIR_VER);
#endif
            lv_obj_set_scrollbar_mode(lvglRadioButtonConfigList, LV_SCROLLBAR_MODE_OFF);

#if defined(BOARD_ESP32_2432S024C)
            lvglRadioReceiverTestBtn = makeSmallBtn(wrap, "Receiver Test", DISPLAY_WIDTH - 20, 32, lv_color_hex(0x7A5C2E), lvglOpenRadioReceiverTestScreenEvent, nullptr);
#else
            lvglRadioReceiverTestBtn = makeSmallBtn(wrap, "Receiver Test", DISPLAY_WIDTH - 20, 32, lv_color_hex(0x7A5C2E), lvglOpenRadioReceiverTestScreenEvent, nullptr);
#endif

            lvglRefreshRadioButtonConfigUi();
            break;
        }
        case UI_CONFIG_RADIO_RECEIVER_TEST: {
            lvglScrRadioReceiverTest = lvglCreateScreenBase("Receiver Test", true);
            lv_obj_t *wrap = lv_obj_create(lvglScrRadioReceiverTest);
            lv_obj_set_size(wrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(wrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(wrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(wrap, 0, 0);
            lv_obj_set_style_pad_all(wrap, 10, 0);
            lv_obj_set_style_pad_row(wrap, 8, 0);
            lv_obj_set_flex_flow(wrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(wrap, LV_OBJ_FLAG_SCROLLABLE);

            lvglRadioReceiverTestStatusLabel = lv_label_create(wrap);
            lv_obj_set_width(lvglRadioReceiverTestStatusLabel, lv_pct(100));
            lv_label_set_long_mode(lvglRadioReceiverTestStatusLabel, LV_LABEL_LONG_WRAP);
            lv_obj_set_style_text_color(lvglRadioReceiverTestStatusLabel, lv_color_hex(0xC8D3DD), 0);

            lvglRadioReceiverTestTa = lv_textarea_create(wrap);
            lv_obj_set_width(lvglRadioReceiverTestTa, lv_pct(100));
            lv_obj_set_height(lvglRadioReceiverTestTa, UI_CONTENT_H - 44);
            lv_textarea_set_one_line(lvglRadioReceiverTestTa, false);
            lv_textarea_set_placeholder_text(lvglRadioReceiverTestTa, "Listening for legacy HC-12 packets...");
            lv_obj_set_style_bg_color(lvglRadioReceiverTestTa, lv_color_hex(0x091118), 0);
            lv_obj_set_style_text_color(lvglRadioReceiverTestTa, lv_color_hex(0x82F6A2), 0);
            lv_obj_set_style_border_color(lvglRadioReceiverTestTa, lv_color_hex(0x2D495E), 0);
            lv_obj_set_style_border_width(lvglRadioReceiverTestTa, 1, 0);
            lv_obj_clear_flag(lvglRadioReceiverTestTa, LV_OBJ_FLAG_CLICKABLE);
            lv_textarea_set_cursor_click_pos(lvglRadioReceiverTestTa, false);

            lvglRefreshRadioReceiverTestUi();
            break;
        }
        case UI_CONFIG: {
            lvglScrConfig = lvglCreateScreenBase(tr(TXT_CONFIG), true);
            lvglConfigWrap = lv_obj_create(lvglScrConfig);
            lv_obj_set_size(lvglConfigWrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(lvglConfigWrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(lvglConfigWrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(lvglConfigWrap, 0, 0);
            lv_obj_set_style_pad_all(lvglConfigWrap, 10, 0);
            lv_obj_set_style_pad_row(lvglConfigWrap, 10, 0);
            lv_obj_set_flex_flow(lvglConfigWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scrollbar_mode(lvglConfigWrap, LV_SCROLLBAR_MODE_OFF);
            lvglConfigWifiBtn = lvglCreateMenuButton(lvglConfigWrap, tr(TXT_WIFI_CONFIG), lv_color_hex(0x3A8F4B), lvglHomeNavEvent, reinterpret_cast<void *>(static_cast<intptr_t>(UI_WIFI_LIST)));
            lvglConfigHc12Btn = lvglCreateMenuButton(lvglConfigWrap, tr(TXT_HC12_CONFIG), lv_color_hex(0x7A5C2E), lvglOpenHc12ScreenEvent, nullptr);
            lvglConfigRemoteBtn = lvglCreateMenuButton(lvglConfigWrap, tr(TXT_REMOTE_CONTROL), lv_color_hex(0x8A5A2D), lvglOpenRemoteControlScreenEvent, nullptr);
            lvglConfigStyleBtn = lvglCreateMenuButton(lvglConfigWrap, tr(TXT_SCREEN), lv_color_hex(0x2D6D8E), lvglOpenStyleScreenEvent, nullptr);
            lvglConfigBatteryBtn = lvglCreateMenuButton(lvglConfigWrap, tr(TXT_BATTERY), lv_color_hex(0x6C7E2C), lvglHomeNavEvent, reinterpret_cast<void *>(static_cast<intptr_t>(UI_CONFIG_BATTERY)));
            lvglConfigMqttBtn = lvglCreateMenuButton(lvglConfigWrap, tr(TXT_MQTT_CONFIG), lv_color_hex(0x6D4B9A), lvglOpenMqttCfgEvent, nullptr);
            lvglConfigLanguageBtn = lvglCreateMenuButton(lvglConfigWrap, tr(TXT_LANGUAGE), lv_color_hex(0x5A6FA8), lvglOpenLanguageScreenEvent, nullptr);
            lvglConfigOtaBtn = lvglCreateMenuButton(lvglConfigWrap, tr(TXT_OTA_UPDATES), lv_color_hex(0x2E6F95), lvglOpenOtaScreenEvent, nullptr);
            lvglConfigModulesBtn = lvglCreateMenuButton(lvglConfigWrap, "Modules", lv_color_hex(0x357A38), lvglOpenModulesScreenEvent, nullptr);
            lvglConfigFactoryResetBtn = lvglCreateMenuButton(
                lvglConfigWrap,
                tr(TXT_FACTORY_RESET),
                lv_color_hex(0x8E2D2D),
                lvglFactoryResetEvent,
                nullptr
            );
            lvglRefreshPrimaryMenuButtonIcons();

            lv_obj_t *nameWrap = lv_obj_create(lvglConfigWrap);
            lv_obj_set_size(nameWrap, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(nameWrap, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(nameWrap, 0, 0);
            lv_obj_set_style_radius(nameWrap, 12, 0);
            lv_obj_set_style_pad_all(nameWrap, 10, 0);
            lv_obj_set_style_pad_row(nameWrap, 6, 0);
            lv_obj_set_flex_flow(nameWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(nameWrap, LV_OBJ_FLAG_SCROLLABLE);

            lvglConfigDeviceNameHeader = lv_label_create(nameWrap);
            lv_label_set_text(lvglConfigDeviceNameHeader, tr(TXT_DEVICE_NAME));
            lv_obj_set_style_text_color(lvglConfigDeviceNameHeader, lv_color_hex(0xE5ECF3), 0);

            lv_obj_t *nameRow = lv_obj_create(nameWrap);
            lv_obj_set_size(nameRow, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(nameRow, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(nameRow, 0, 0);
            lv_obj_set_style_pad_all(nameRow, 0, 0);
            lv_obj_set_style_pad_column(nameRow, 8, 0);
            lv_obj_set_flex_flow(nameRow, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(nameRow, LV_OBJ_FLAG_SCROLLABLE);

            lvglConfigDeviceNameTa = lv_textarea_create(nameRow);
            lv_obj_set_width(lvglConfigDeviceNameTa, DISPLAY_WIDTH - 110);
            lv_obj_set_flex_grow(lvglConfigDeviceNameTa, 1);
            lvglConfigureCompactSingleLineTa(lvglConfigDeviceNameTa, 34);
            lv_textarea_set_placeholder_text(lvglConfigDeviceNameTa, "Device name");
            lv_textarea_set_max_length(lvglConfigDeviceNameTa, 24);
            lv_obj_add_event_cb(lvglConfigDeviceNameTa, lvglTextAreaFocusEvent, LV_EVENT_FOCUSED, nullptr);

            lv_obj_t *saveBtn = makeSmallBtn(nameRow, tr(TXT_SAVE), 64, 38, lv_color_hex(0x3A7A3A), lvglSaveDeviceNameEvent, nullptr);
            lvglConfigDeviceNameSaveBtnLabel = saveBtn ? lv_obj_get_child(saveBtn, 0) : nullptr;

            lv_obj_t *brightWrap = lv_obj_create(lvglConfigWrap);
            lv_obj_set_size(brightWrap, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(brightWrap, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(brightWrap, 0, 0);
            lv_obj_set_style_radius(brightWrap, 12, 0);
            lv_obj_set_style_pad_all(brightWrap, 10, 0);
            lv_obj_set_style_pad_row(brightWrap, 6, 0);
            lv_obj_set_flex_flow(brightWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(brightWrap, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *brightHdr = lv_obj_create(brightWrap);
            lv_obj_set_size(brightHdr, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(brightHdr, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(brightHdr, 0, 0);
            lv_obj_set_style_pad_all(brightHdr, 0, 0);
            lv_obj_set_style_pad_column(brightHdr, 8, 0);
            lv_obj_set_flex_flow(brightHdr, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(brightHdr, LV_OBJ_FLAG_SCROLLABLE);

            lvglConfigBrightnessHeader = lv_label_create(brightHdr);
            lv_label_set_text(lvglConfigBrightnessHeader, tr(TXT_BRIGHTNESS));
            lv_obj_set_style_text_color(lvglConfigBrightnessHeader, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_flex_grow(lvglConfigBrightnessHeader, 1);

            lvglBrightnessValueLabel = lv_label_create(brightHdr);
            lv_obj_set_style_text_color(lvglBrightnessValueLabel, lv_color_hex(0xB7C4D1), 0);

            lvglBrightnessSlider = lv_slider_create(brightWrap);
            lv_obj_set_width(lvglBrightnessSlider, lv_pct(100));
            lv_slider_set_range(lvglBrightnessSlider, 5, 100);
            lv_obj_add_event_cb(lvglBrightnessSlider, lvglBrightnessEvent, LV_EVENT_VALUE_CHANGED, nullptr);

            lv_obj_t *adaptiveRow = lv_obj_create(brightWrap);
            lv_obj_set_size(adaptiveRow, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(adaptiveRow, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(adaptiveRow, 0, 0);
            lv_obj_set_style_pad_all(adaptiveRow, 0, 0);
            lv_obj_set_style_pad_column(adaptiveRow, 8, 0);
            lv_obj_set_flex_flow(adaptiveRow, LV_FLEX_FLOW_ROW);
            lv_obj_set_flex_align(adaptiveRow, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
            lv_obj_clear_flag(adaptiveRow, LV_OBJ_FLAG_SCROLLABLE);

            lvglConfigAdaptiveBrightnessLabel = lv_label_create(adaptiveRow);
            lv_label_set_text(lvglConfigAdaptiveBrightnessLabel, tr(TXT_ADAPTIVE_BRIGHTNESS));
            lv_obj_set_style_text_color(lvglConfigAdaptiveBrightnessLabel, lv_color_hex(0xB7C4D1), 0);
            lv_obj_set_flex_grow(lvglConfigAdaptiveBrightnessLabel, 1);

            lvglAdaptiveBrightnessSw = lv_switch_create(adaptiveRow);
            lv_obj_add_event_cb(lvglAdaptiveBrightnessSw, lvglAdaptiveBrightnessEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglAdaptiveBrightnessSw, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglAdaptiveBrightnessSw, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglAdaptiveBrightnessSw, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);

            lv_obj_t *volWrap = lv_obj_create(lvglConfigWrap);
            lv_obj_set_size(volWrap, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(volWrap, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(volWrap, 0, 0);
            lv_obj_set_style_radius(volWrap, 12, 0);
            lv_obj_set_style_pad_all(volWrap, 10, 0);
            lv_obj_set_style_pad_row(volWrap, 6, 0);
            lv_obj_set_flex_flow(volWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(volWrap, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *volHdr = lv_obj_create(volWrap);
            lv_obj_set_size(volHdr, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(volHdr, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(volHdr, 0, 0);
            lv_obj_set_style_pad_all(volHdr, 0, 0);
            lv_obj_set_style_pad_column(volHdr, 8, 0);
            lv_obj_set_flex_flow(volHdr, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(volHdr, LV_OBJ_FLAG_SCROLLABLE);

            lvglConfigVolumeHeader = lv_label_create(volHdr);
            lv_label_set_text(lvglConfigVolumeHeader, tr(TXT_VOLUME));
            lv_obj_set_style_text_color(lvglConfigVolumeHeader, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_flex_grow(lvglConfigVolumeHeader, 1);

            lvglVolumeValueLabel = lv_label_create(volHdr);
            lv_obj_set_style_text_color(lvglVolumeValueLabel, lv_color_hex(0xB7C4D1), 0);

            lvglVolumeSlider = lv_slider_create(volWrap);
            lv_obj_set_width(lvglVolumeSlider, lv_pct(100));
            lv_slider_set_range(lvglVolumeSlider, 0, 100);
            lv_obj_add_event_cb(lvglVolumeSlider, lvglConfigVolumeEvent, LV_EVENT_VALUE_CHANGED, nullptr);

            lv_obj_t *vibrationWrap = lv_obj_create(lvglConfigWrap);
            lv_obj_set_size(vibrationWrap, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(vibrationWrap, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(vibrationWrap, 0, 0);
            lv_obj_set_style_radius(vibrationWrap, 12, 0);
            lv_obj_set_style_pad_all(vibrationWrap, 10, 0);
            lv_obj_set_style_pad_row(vibrationWrap, 6, 0);
            lv_obj_set_flex_flow(vibrationWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(vibrationWrap, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *vibrationHdr = lv_obj_create(vibrationWrap);
            lv_obj_set_size(vibrationHdr, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(vibrationHdr, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(vibrationHdr, 0, 0);
            lv_obj_set_style_pad_all(vibrationHdr, 0, 0);
            lv_obj_set_style_pad_column(vibrationHdr, 8, 0);
            lv_obj_set_flex_flow(vibrationHdr, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(vibrationHdr, LV_OBJ_FLAG_SCROLLABLE);

            lvglConfigVibrationHeader = lv_label_create(vibrationHdr);
            lv_label_set_text(lvglConfigVibrationHeader, "Vibration");
            lv_obj_set_style_text_color(lvglConfigVibrationHeader, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_flex_grow(lvglConfigVibrationHeader, 1);

            lvglVibrationDropdown = lv_dropdown_create(vibrationWrap);
            lv_obj_set_width(lvglVibrationDropdown, lv_pct(100));
            lv_dropdown_set_options(lvglVibrationDropdown, buildVibrationIntensityDropdownOptions().c_str());
            lv_obj_add_event_cb(lvglVibrationDropdown, lvglVibrationDropdownEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglVibrationDropdown, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglVibrationDropdown, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglVibrationDropdown, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);

            lv_obj_t *toneWrap = lv_obj_create(lvglConfigWrap);
            lv_obj_set_size(toneWrap, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(toneWrap, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(toneWrap, 0, 0);
            lv_obj_set_style_radius(toneWrap, 12, 0);
            lv_obj_set_style_pad_all(toneWrap, 10, 0);
            lv_obj_set_style_pad_row(toneWrap, 6, 0);
            lv_obj_set_flex_flow(toneWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(toneWrap, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *toneHdr = lv_obj_create(toneWrap);
            lv_obj_set_size(toneHdr, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(toneHdr, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(toneHdr, 0, 0);
            lv_obj_set_style_pad_all(toneHdr, 0, 0);
            lv_obj_set_style_pad_column(toneHdr, 8, 0);
            lv_obj_set_flex_flow(toneHdr, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(toneHdr, LV_OBJ_FLAG_SCROLLABLE);

            lvglConfigMessageToneHeader = lv_label_create(toneHdr);
            lv_label_set_text(lvglConfigMessageToneHeader, "Message tone");
            lv_obj_set_style_text_color(lvglConfigMessageToneHeader, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_flex_grow(lvglConfigMessageToneHeader, 1);

            lvglMessageToneDropdown = lv_dropdown_create(toneWrap);
            lv_obj_set_width(lvglMessageToneDropdown, lv_pct(100));
            lv_dropdown_set_options(lvglMessageToneDropdown, buildMessageBeepDropdownOptions().c_str());
            lv_obj_add_event_cb(lvglMessageToneDropdown, lvglMessageToneDropdownEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglMessageToneDropdown, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglMessageToneDropdown, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglMessageToneDropdown, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);

            lv_obj_t *startupSoundRow = lv_obj_create(toneWrap);
            lv_obj_set_size(startupSoundRow, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(startupSoundRow, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(startupSoundRow, 0, 0);
            lv_obj_set_style_pad_all(startupSoundRow, 0, 0);
            lv_obj_set_style_pad_column(startupSoundRow, 8, 0);
            lv_obj_set_flex_flow(startupSoundRow, LV_FLEX_FLOW_ROW);
            lv_obj_set_flex_align(startupSoundRow, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
            lv_obj_clear_flag(startupSoundRow, LV_OBJ_FLAG_SCROLLABLE);

            lvglConfigStartupSoundLabel = lv_label_create(startupSoundRow);
            lv_label_set_text(lvglConfigStartupSoundLabel, "Play sound on start");
            lv_obj_set_style_text_color(lvglConfigStartupSoundLabel, lv_color_hex(0xB7C4D1), 0);
            lv_obj_set_flex_grow(lvglConfigStartupSoundLabel, 1);

            lvglStartupSoundSw = lv_switch_create(startupSoundRow);
            lv_obj_add_event_cb(lvglStartupSoundSw, lvglStartupSoundToggleEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglStartupSoundSw, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglStartupSoundSw, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglStartupSoundSw, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);

            lv_obj_t *rgbWrap = lv_obj_create(lvglConfigWrap);
            lv_obj_set_size(rgbWrap, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(rgbWrap, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(rgbWrap, 0, 0);
            lv_obj_set_style_radius(rgbWrap, 12, 0);
            lv_obj_set_style_pad_all(rgbWrap, 10, 0);
            lv_obj_set_style_pad_row(rgbWrap, 6, 0);
            lv_obj_set_flex_flow(rgbWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(rgbWrap, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *rgbHdr = lv_obj_create(rgbWrap);
            lv_obj_set_size(rgbHdr, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(rgbHdr, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(rgbHdr, 0, 0);
            lv_obj_set_style_pad_all(rgbHdr, 0, 0);
            lv_obj_set_style_pad_column(rgbHdr, 8, 0);
            lv_obj_set_flex_flow(rgbHdr, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(rgbHdr, LV_OBJ_FLAG_SCROLLABLE);

            lvglConfigRgbHeader = lv_label_create(rgbHdr);
            lv_label_set_text(lvglConfigRgbHeader, tr(TXT_RGB_LED));
            lv_obj_set_style_text_color(lvglConfigRgbHeader, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_flex_grow(lvglConfigRgbHeader, 1);

            lv_obj_t *rgbValueLabel = lv_label_create(rgbHdr);
            lv_obj_set_style_text_color(rgbValueLabel, lv_color_hex(0xB7C4D1), 0);
            lv_label_set_text_fmt(rgbValueLabel, "%u%%", static_cast<unsigned int>(rgbLedPercent));

            lvglRgbLedSlider = lv_slider_create(rgbWrap);
            lv_obj_set_width(lvglRgbLedSlider, lv_pct(100));
            lv_slider_set_range(lvglRgbLedSlider, 0, 100);
            lv_obj_add_event_cb(lvglRgbLedSlider, lvglRgbLedEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lvglApplyConfigScreenControlStyles();
            lvglRegisterReorderableItem(lvglConfigWifiBtn, "ord_cfg", "wifi");
            lvglRegisterReorderableItem(lvglConfigHc12Btn, "ord_cfg", "hc12");
            lvglRegisterReorderableItem(lvglConfigRemoteBtn, "ord_cfg", "rctl");
            lvglRegisterReorderableItem(lvglConfigStyleBtn, "ord_cfg", "style");
            lvglRegisterReorderableItem(lvglConfigMqttBtn, "ord_cfg", "mqtt");
            lvglRegisterReorderableItem(lvglConfigLanguageBtn, "ord_cfg", "lang");
            lvglRegisterReorderableItem(lvglConfigOtaBtn, "ord_cfg", "ota");
            lvglRegisterReorderableItem(lvglConfigModulesBtn, "ord_cfg", "mods");
            lvglRegisterReorderableItem(lvglConfigFactoryResetBtn, "ord_cfg", "freset");            
            lvglRegisterReorderableItem(nameWrap, "ord_cfg", "name");
            lvglRegisterReorderableItem(brightWrap, "ord_cfg", "bright");
            lvglRegisterReorderableItem(volWrap, "ord_cfg", "vol");
            lvglRegisterReorderableItem(vibrationWrap, "ord_cfg", "vib");
            lvglRegisterReorderableItem(toneWrap, "ord_cfg", "tone");
            lvglRegisterReorderableItem(rgbWrap, "ord_cfg", "rgb");
            lvglApplySavedOrder(lvglConfigWrap, "ord_cfg");
            lvglRefreshConfigUi();
            break;
        }
        case UI_CONFIG_REMOTE_CONTROL: {
            lvglScrRemoteControl = lvglCreateScreenBase(tr(TXT_REMOTE_CONTROL), true);
            lv_obj_t *wrap = lv_obj_create(lvglScrRemoteControl);
            lv_obj_set_size(wrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(wrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(wrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(wrap, 0, 0);
            lv_obj_set_style_pad_all(wrap, 10, 0);
            lv_obj_set_style_pad_row(wrap, 10, 0);
            lv_obj_set_flex_flow(wrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scrollbar_mode(wrap, LV_SCROLLBAR_MODE_OFF);

            auto makeFieldCard = [&](const char *title, lv_obj_t **titleOut) -> lv_obj_t * {
                lv_obj_t *card = lv_obj_create(wrap);
                lv_obj_set_size(card, lv_pct(100), LV_SIZE_CONTENT);
                lv_obj_set_style_bg_color(card, lv_color_hex(0x18222D), 0);
                lv_obj_set_style_border_width(card, 0, 0);
                lv_obj_set_style_radius(card, 12, 0);
                lv_obj_set_style_pad_all(card, 10, 0);
                lv_obj_set_style_pad_row(card, 6, 0);
                lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
                lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_t *lbl = lv_label_create(card);
                lv_label_set_text(lbl, title);
                lv_obj_set_style_text_color(lbl, lv_color_hex(0xE5ECF3), 0);
                if (titleOut) *titleOut = lbl;
                return card;
            };

            lv_obj_t *moduleCard = makeFieldCard(tr(TXT_RADIO_MODULE), &lvglRemoteModuleLabel);
            lvglRemoteModuleDropdown = lv_dropdown_create(moduleCard);
            lv_obj_set_width(lvglRemoteModuleDropdown, lv_pct(100));
            lv_dropdown_set_options(lvglRemoteModuleDropdown, buildRadioModuleDropdownOptions().c_str());
            lv_obj_add_event_cb(lvglRemoteModuleDropdown, lvglRemoteControlModuleChangedEvent, LV_EVENT_VALUE_CHANGED, nullptr);

            lv_obj_t *channelCard = makeFieldCard(tr(TXT_CHANNEL), &lvglRemoteChannelLabel);
            lvglRemoteChannelTa = lv_textarea_create(channelCard);
            lv_obj_set_width(lvglRemoteChannelTa, lv_pct(100));
            lv_obj_set_height(lvglRemoteChannelTa, 38);
            lv_textarea_set_one_line(lvglRemoteChannelTa, true);
            lv_textarea_set_max_length(lvglRemoteChannelTa, 4);
            lv_textarea_set_placeholder_text(lvglRemoteChannelTa, "Channel");
            lv_obj_add_event_cb(lvglRemoteChannelTa, lvglTextAreaFocusEvent, LV_EVENT_FOCUSED, nullptr);

            lv_obj_t *baudCard = makeFieldCard(tr(TXT_BAUD_RATE), &lvglRemoteBaudLabel);
            lvglRemoteBaudDropdown = lv_dropdown_create(baudCard);
            lv_obj_set_width(lvglRemoteBaudDropdown, lv_pct(100));

            lv_obj_t *modeCard = makeFieldCard(tr(TXT_FU_MODE), &lvglRemoteModeLabel);
            lvglRemoteModeDropdown = lv_dropdown_create(modeCard);
            lv_obj_set_width(lvglRemoteModeDropdown, lv_pct(100));

            lv_obj_t *powerCard = makeFieldCard(tr(TXT_POWER), &lvglRemotePowerLabel);
            lvglRemotePowerDropdown = lv_dropdown_create(powerCard);
            lv_obj_set_width(lvglRemotePowerDropdown, lv_pct(100));

            lvglRemoteExtraCard = makeFieldCard(tr(TXT_TRANSFER_MODE), &lvglRemoteExtraLabel);
            lvglRemoteExtraDropdown = lv_dropdown_create(lvglRemoteExtraCard);
            lv_obj_set_width(lvglRemoteExtraDropdown, lv_pct(100));

            lv_obj_t *keyCard = makeFieldCard(tr(TXT_ENCRYPTION_KEY), &lvglRemoteKeyLabel);
            lvglRemoteKeyTa = lv_textarea_create(keyCard);
            lv_obj_set_width(lvglRemoteKeyTa, lv_pct(100));
            lv_obj_set_height(lvglRemoteKeyTa, 38);
            lv_textarea_set_one_line(lvglRemoteKeyTa, true);
            lv_textarea_set_max_length(lvglRemoteKeyTa, 24);
            lv_textarea_set_placeholder_text(lvglRemoteKeyTa, "16 hex chars");
            lv_obj_add_event_cb(lvglRemoteKeyTa, lvglTextAreaFocusEvent, LV_EVENT_FOCUSED, nullptr);

            lv_obj_t *remoteIdCard = makeFieldCard(tr(TXT_REMOTE_ID), &lvglRemoteIdLabel);
            lvglRemoteIdTa = lv_textarea_create(remoteIdCard);
            lv_obj_set_width(lvglRemoteIdTa, lv_pct(100));
            lv_obj_set_height(lvglRemoteIdTa, 38);
            lv_textarea_set_one_line(lvglRemoteIdTa, true);
            lv_textarea_set_max_length(lvglRemoteIdTa, 5);
            lv_textarea_set_accepted_chars(lvglRemoteIdTa, "0123456789");
            lv_textarea_set_placeholder_text(lvglRemoteIdTa, "1..65535");
            lv_obj_add_event_cb(lvglRemoteIdTa, lvglTextAreaFocusEvent, LV_EVENT_FOCUSED, nullptr);

            lv_obj_t *counterCard = makeFieldCard(tr(TXT_ROLLING_COUNTER), &lvglRemoteCounterLabel);
            lvglRemoteCounterTa = lv_textarea_create(counterCard);
            lv_obj_set_width(lvglRemoteCounterTa, lv_pct(100));
            lv_obj_set_height(lvglRemoteCounterTa, 38);
            lv_textarea_set_one_line(lvglRemoteCounterTa, true);
            lv_textarea_set_max_length(lvglRemoteCounterTa, 5);
            lv_textarea_set_accepted_chars(lvglRemoteCounterTa, "0123456789");
            lv_textarea_set_placeholder_text(lvglRemoteCounterTa, "1..65535");
            lv_obj_add_event_cb(lvglRemoteCounterTa, lvglTextAreaFocusEvent, LV_EVENT_FOCUSED, nullptr);

            lvglRemoteSaveBtn = lvglCreateMenuButton(wrap, tr(TXT_SAVE), lv_color_hex(0x2F7A45), lvglRemoteControlSaveEvent, nullptr);
            lvglRemoteSaveBtnLabel = lvglRemoteSaveBtn ? lv_obj_get_child(lvglRemoteSaveBtn, 0) : nullptr;

            lvglRemoteStatusLabel = lv_label_create(wrap);
            lv_obj_set_width(lvglRemoteStatusLabel, lv_pct(100));
            lv_label_set_long_mode(lvglRemoteStatusLabel, LV_LABEL_LONG_WRAP);
            lv_obj_set_style_text_color(lvglRemoteStatusLabel, lv_color_hex(0xC8D3DD), 0);

            lvglRefreshRemoteControlUi();
            break;
        }
        case UI_CONFIG_BATTERY: {
            lvglScrBatteryTrain = lvglCreateScreenBase(tr(TXT_BATTERY), false);
            lvglBatteryTrainStatusLabel = nullptr;
            lvglBatteryTrainCurrentLabel = nullptr;
            lvglBatteryTrainFullLabel = nullptr;
            lvglBatteryTrainEmptyLabel = nullptr;
            lvglBatteryTrainFactorLabel = nullptr;
            lvglBatteryTrainPowerLabel = nullptr;
            lvglBatteryTrainFullBtn = nullptr;
            lvglBatteryTrainDischargeBtn = nullptr;
            lvglBatteryTrainAutoBtn = nullptr;

            lv_obj_t *wrap = lv_obj_create(lvglScrBatteryTrain);
            lv_obj_set_size(wrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(wrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(wrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(wrap, 0, 0);
            lv_obj_set_style_pad_all(wrap, 10, 0);
            lv_obj_set_style_pad_row(wrap, 10, 0);
            lv_obj_set_flex_flow(wrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scrollbar_mode(wrap, LV_SCROLLBAR_MODE_OFF);

            lv_obj_t *stateCard = lv_obj_create(wrap);
            lv_obj_set_size(stateCard, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(stateCard, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(stateCard, 0, 0);
            lv_obj_set_style_radius(stateCard, 12, 0);
            lv_obj_set_style_pad_all(stateCard, 12, 0);
            lv_obj_set_style_pad_row(stateCard, 7, 0);
            lv_obj_set_flex_flow(stateCard, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(stateCard, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *stateTitle = lv_label_create(stateCard);
            lv_label_set_text(stateTitle, tr(TXT_STORED_CALIBRATION));
            lv_obj_set_style_text_color(stateTitle, lv_color_hex(0xE5ECF3), 0);

            auto makeBatteryInfoLabel = [&](lv_obj_t **out, lv_color_t color) {
                lv_obj_t *label = lv_label_create(stateCard);
                lv_obj_set_width(label, lv_pct(100));
                lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
                lv_obj_set_style_text_color(label, color, 0);
                if (out) *out = label;
            };

            makeBatteryInfoLabel(&lvglBatteryTrainStatusLabel, lv_color_hex(0xE5ECF3));
            makeBatteryInfoLabel(&lvglBatteryTrainCurrentLabel, lv_color_hex(0xB7C4D1));
            makeBatteryInfoLabel(&lvglBatteryTrainFullLabel, lv_color_hex(0xB7C4D1));
            makeBatteryInfoLabel(&lvglBatteryTrainEmptyLabel, lv_color_hex(0xB7C4D1));
            makeBatteryInfoLabel(&lvglBatteryTrainFactorLabel, lv_color_hex(0xB7C4D1));
            makeBatteryInfoLabel(&lvglBatteryTrainPowerLabel, lv_color_hex(0xB7C4D1));

            lv_obj_t *hintCard = lv_obj_create(wrap);
            lv_obj_set_size(hintCard, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(hintCard, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(hintCard, 0, 0);
            lv_obj_set_style_radius(hintCard, 12, 0);
            lv_obj_set_style_pad_all(hintCard, 12, 0);
            lv_obj_set_style_pad_row(hintCard, 8, 0);
            lv_obj_set_flex_flow(hintCard, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(hintCard, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *hint = lv_label_create(hintCard);
            lv_obj_set_width(hint, lv_pct(100));
            lv_label_set_long_mode(hint, LV_LABEL_LONG_WRAP);
            lv_obj_set_style_text_color(hint, lv_color_hex(0x9FB0C2), 0);
            lv_label_set_text(hint,
                              "Reset clears stored anchors and starts charge training. FULL stores the current top voltage and restores your power setting. "
                              "DISCHARGE forces Power Off to Never until the next low-battery cutoff is learned. Auto Calibration must be confirmed and only then enables the background learning algorithm.");

            makeSmallBtn(wrap, tr(TXT_RESET), DISPLAY_WIDTH - 20, 40, lv_color_hex(0x8D5F1E), lvglBatteryTrainResetEvent, nullptr);

            lv_obj_t *buttonRow = lv_obj_create(wrap);
            lv_obj_set_size(buttonRow, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(buttonRow, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(buttonRow, 0, 0);
            lv_obj_set_style_pad_all(buttonRow, 0, 0);
            lv_obj_set_style_pad_column(buttonRow, 10, 0);
            lv_obj_set_flex_flow(buttonRow, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(buttonRow, LV_OBJ_FLAG_SCROLLABLE);

            lvglBatteryTrainFullBtn = makeSmallBtn(buttonRow, "FULL", (DISPLAY_WIDTH - 30) / 2, 42, lv_color_hex(0x2F7A45), lvglBatteryTrainFullEvent, nullptr);
            lvglBatteryTrainDischargeBtn = makeSmallBtn(buttonRow, "DISCHARGE", (DISPLAY_WIDTH - 30) / 2, 42, lv_color_hex(0x8A3A2E), lvglBatteryTrainDischargeEvent, nullptr);
            lvglBatteryTrainAutoBtn = makeSmallBtn(wrap, tr(TXT_AUTO_CALIBRATION), DISPLAY_WIDTH - 20, 40, lv_color_hex(0x345D8A), lvglBatteryTrainAutoEvent, nullptr);
            lvglRefreshBatteryTrainButtonIcons();
            lvglRefreshBatteryTrainUi();
            break;
        }
        case UI_CONFIG_STYLE: {
            lvglScrStyle = lvglCreateScreenBase(tr(TXT_SCREEN), false);
            lvglStyleButtonFlatBtn = nullptr;
            lvglStyleButtonFlatBtnLabel = nullptr;
            lvglStyleButton3dBtn = nullptr;
            lvglStyleButton3dBtnLabel = nullptr;
            lvglStyleButtonBlackBtn = nullptr;
            lvglStyleButtonBlackBtnLabel = nullptr;
            lvglStyleMenuIconsSw = nullptr;
            lvglStyleTouchVibrationSw = nullptr;
            lvglStyleTouchSoundSw = nullptr;
            lvglStyleTimezoneDd = nullptr;
            lvglStyleTopCenterNameBtn = nullptr;
            lvglStyleTopCenterNameBtnLabel = nullptr;
            lvglStyleTopCenterTimeBtn = nullptr;
            lvglStyleTopCenterTimeBtnLabel = nullptr;
            lv_obj_t *wrap = lv_obj_create(lvglScrStyle);
            lv_obj_set_size(wrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(wrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(wrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(wrap, 0, 0);
            lv_obj_set_style_pad_all(wrap, 10, 0);
            lv_obj_set_style_pad_row(wrap, 10, 0);
            lv_obj_set_flex_flow(wrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scrollbar_mode(wrap, LV_SCROLLBAR_MODE_OFF);

            lvglStyleHomeItemsBtn = lvglCreateMenuButton(wrap, tr(TXT_MAIN_SCREEN_ITEMS), lv_color_hex(0x3A6D8F), lvglOpenStyleHomeItemsScreenEvent, nullptr);
            if (lvglStyleHomeItemsBtn) lvglStyleHomeItemsBtnLabel = lv_obj_get_child(lvglStyleHomeItemsBtn, 0);

            lvglStyleScreensaverRow = lv_obj_create(wrap);
            lv_obj_t *screensaverRow = lvglStyleScreensaverRow;
            lv_obj_set_size(screensaverRow, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(screensaverRow, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(screensaverRow, 0, 0);
            lv_obj_set_style_radius(screensaverRow, 12, 0);
            lv_obj_set_style_pad_all(screensaverRow, 12, 0);
            lv_obj_set_style_pad_column(screensaverRow, 8, 0);
            lv_obj_set_flex_flow(screensaverRow, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(screensaverRow, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *screensaverLbl = lv_label_create(screensaverRow);
            lv_label_set_text(screensaverLbl, tr(TXT_SCREENSAVER));
            lv_obj_set_style_text_color(screensaverLbl, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_flex_grow(screensaverLbl, 1);

            lvglStyleScreensaverSw = lv_switch_create(screensaverRow);
            lv_obj_add_event_cb(lvglStyleScreensaverSw, lvglStyleScreensaverToggleEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglStyleScreensaverSw, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglStyleScreensaverSw, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglStyleScreensaverSw, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);

            lvglStyleScreensaverTypeRow = lv_obj_create(wrap);
            lv_obj_set_size(lvglStyleScreensaverTypeRow, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(lvglStyleScreensaverTypeRow, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(lvglStyleScreensaverTypeRow, 0, 0);
            lv_obj_set_style_radius(lvglStyleScreensaverTypeRow, 12, 0);
            lv_obj_set_style_pad_all(lvglStyleScreensaverTypeRow, 12, 0);
            lv_obj_set_style_pad_column(lvglStyleScreensaverTypeRow, 8, 0);
            lv_obj_set_flex_flow(lvglStyleScreensaverTypeRow, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(lvglStyleScreensaverTypeRow, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *screensaverTypeLbl = lv_label_create(lvglStyleScreensaverTypeRow);
            lv_label_set_text(screensaverTypeLbl, "Screensaver type");
            lv_obj_set_style_text_color(screensaverTypeLbl, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_flex_grow(screensaverTypeLbl, 1);

            lvglStyleScreensaverTypeDd = lv_dropdown_create(lvglStyleScreensaverTypeRow);
            lv_dropdown_set_options(lvglStyleScreensaverTypeDd, buildScreensaverDropdownOptions().c_str());
            lv_obj_set_style_bg_color(lvglStyleScreensaverTypeDd, lv_color_hex(0x111922), 0);
            lv_obj_set_style_text_color(lvglStyleScreensaverTypeDd, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_style_border_color(lvglStyleScreensaverTypeDd, lv_color_hex(0x2F4658), 0);
            lv_obj_set_style_border_width(lvglStyleScreensaverTypeDd, 1, 0);
            lv_obj_set_style_radius(lvglStyleScreensaverTypeDd, 8, 0);
            lv_obj_add_event_cb(lvglStyleScreensaverTypeDd, lvglStyleScreensaverTypeEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglStyleScreensaverTypeDd, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglStyleScreensaverTypeDd, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglStyleScreensaverTypeDd, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);

            lv_obj_t *screenLockRow = lv_obj_create(wrap);
            lv_obj_set_size(screenLockRow, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(screenLockRow, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(screenLockRow, 0, 0);
            lv_obj_set_style_radius(screenLockRow, 12, 0);
            lv_obj_set_style_pad_all(screenLockRow, 12, 0);
            lv_obj_set_style_pad_column(screenLockRow, 8, 0);
            lv_obj_set_flex_flow(screenLockRow, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(screenLockRow, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *screenLockLbl = lv_label_create(screenLockRow);
            lv_label_set_text(screenLockLbl, "Screen Lock");
            lv_obj_set_style_text_color(screenLockLbl, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_flex_grow(screenLockLbl, 1);

            lvglStyleScreenLockSw = lv_switch_create(screenLockRow);
            lv_obj_add_event_cb(lvglStyleScreenLockSw, lvglStyleScreenLockToggleEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglStyleScreenLockSw, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglStyleScreenLockSw, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglStyleScreenLockSw, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);

            lv_obj_t *configLockRow = lv_obj_create(wrap);
            lv_obj_set_size(configLockRow, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(configLockRow, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(configLockRow, 0, 0);
            lv_obj_set_style_radius(configLockRow, 12, 0);
            lv_obj_set_style_pad_all(configLockRow, 12, 0);
            lv_obj_set_style_pad_column(configLockRow, 8, 0);
            lv_obj_set_flex_flow(configLockRow, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(configLockRow, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *configLockLbl = lv_label_create(configLockRow);
            lv_label_set_text(configLockLbl, "Config Password");
            lv_obj_set_style_text_color(configLockLbl, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_flex_grow(configLockLbl, 1);

            lvglStyleConfigLockSw = lv_switch_create(configLockRow);
            lv_obj_add_event_cb(lvglStyleConfigLockSw, lvglStyleConfigLockToggleEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglStyleConfigLockSw, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglStyleConfigLockSw, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglStyleConfigLockSw, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);

            lv_obj_t *menuIconsRow = lv_obj_create(wrap);
            lv_obj_set_size(menuIconsRow, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(menuIconsRow, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(menuIconsRow, 0, 0);
            lv_obj_set_style_radius(menuIconsRow, 12, 0);
            lv_obj_set_style_pad_all(menuIconsRow, 12, 0);
            lv_obj_set_style_pad_column(menuIconsRow, 8, 0);
            lv_obj_set_flex_flow(menuIconsRow, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(menuIconsRow, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *menuIconsLbl = lv_label_create(menuIconsRow);
            lv_label_set_text(menuIconsLbl, tr(TXT_3D_ICONS));
            lv_obj_set_style_text_color(menuIconsLbl, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_flex_grow(menuIconsLbl, 1);

            lvglStyleMenuIconsSw = lv_switch_create(menuIconsRow);
            lv_obj_add_event_cb(lvglStyleMenuIconsSw, lvglStyleMenuIconsToggleEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglStyleMenuIconsSw, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglStyleMenuIconsSw, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglStyleMenuIconsSw, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);

            lv_obj_t *touchVibrationRow = lv_obj_create(wrap);
            lv_obj_set_size(touchVibrationRow, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(touchVibrationRow, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(touchVibrationRow, 0, 0);
            lv_obj_set_style_radius(touchVibrationRow, 12, 0);
            lv_obj_set_style_pad_all(touchVibrationRow, 12, 0);
            lv_obj_set_style_pad_column(touchVibrationRow, 8, 0);
            lv_obj_set_flex_flow(touchVibrationRow, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(touchVibrationRow, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *touchVibrationLbl = lv_label_create(touchVibrationRow);
            lv_label_set_text(touchVibrationLbl, tr(TXT_TOUCH_VIBRATION));
            lv_obj_set_style_text_color(touchVibrationLbl, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_flex_grow(touchVibrationLbl, 1);

            lvglStyleTouchVibrationSw = lv_switch_create(touchVibrationRow);
            lv_obj_add_event_cb(lvglStyleTouchVibrationSw, lvglStyleTouchVibrationToggleEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglStyleTouchVibrationSw, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglStyleTouchVibrationSw, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglStyleTouchVibrationSw, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);

            lv_obj_t *touchSoundRow = lv_obj_create(wrap);
            lv_obj_set_size(touchSoundRow, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(touchSoundRow, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(touchSoundRow, 0, 0);
            lv_obj_set_style_radius(touchSoundRow, 12, 0);
            lv_obj_set_style_pad_all(touchSoundRow, 12, 0);
            lv_obj_set_style_pad_column(touchSoundRow, 8, 0);
            lv_obj_set_flex_flow(touchSoundRow, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(touchSoundRow, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *touchSoundLbl = lv_label_create(touchSoundRow);
            lv_label_set_text(touchSoundLbl, tr(TXT_TOUCH_TICK_SOUND));
            lv_obj_set_style_text_color(touchSoundLbl, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_flex_grow(touchSoundLbl, 1);

            lvglStyleTouchSoundSw = lv_switch_create(touchSoundRow);
            lv_obj_add_event_cb(lvglStyleTouchSoundSw, lvglStyleTouchSoundToggleEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglStyleTouchSoundSw, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglStyleTouchSoundSw, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglStyleTouchSoundSw, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);

            lv_obj_t *buttonStyleWrap = lv_obj_create(wrap);
            lv_obj_set_size(buttonStyleWrap, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(buttonStyleWrap, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(buttonStyleWrap, 0, 0);
            lv_obj_set_style_radius(buttonStyleWrap, 12, 0);
            lv_obj_set_style_pad_all(buttonStyleWrap, 10, 0);
            lv_obj_set_style_pad_row(buttonStyleWrap, 8, 0);
            lv_obj_set_flex_flow(buttonStyleWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(buttonStyleWrap, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *buttonStyleHdr = lv_label_create(buttonStyleWrap);
            lv_label_set_text(buttonStyleHdr, tr(TXT_BUTTON_STYLE));
            lv_obj_set_style_text_color(buttonStyleHdr, lv_color_hex(0xE5ECF3), 0);

            auto makeStyleRow = [&](lv_obj_t *parentWrap,
                                    const char *title,
                                    lv_color_t accent,
                                    lv_obj_t **btnOut,
                                    lv_obj_t **labelOut,
                                    lv_event_cb_t cb) {
                lv_obj_t *row = lv_obj_create(parentWrap);
                lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
                lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
                lv_obj_set_style_border_width(row, 0, 0);
                lv_obj_set_style_pad_all(row, 0, 0);
                lv_obj_set_style_pad_column(row, 8, 0);
                lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
                lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

                lv_obj_t *titleLbl = lv_label_create(row);
                lv_label_set_text(titleLbl, title);
                lv_obj_set_style_text_color(titleLbl, lv_color_hex(0xD7E0E8), 0);
                lv_obj_set_flex_grow(titleLbl, 1);

                lv_obj_t *btn = lv_btn_create(row);
                lv_obj_set_size(btn, 82, 28);
                lv_obj_set_style_radius(btn, 8, 0);
                lv_obj_set_style_border_width(btn, 0, 0);
                lv_obj_add_event_cb(btn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
                lv_obj_add_event_cb(btn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
                lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, nullptr);
                lv_obj_t *lbl = lv_label_create(btn);
                lv_label_set_text(lbl, tr(TXT_SELECT));
                lv_obj_center(lbl);
                lvglRegisterStyledButton(btn, accent, true);
                *btnOut = btn;
                *labelOut = lbl;
            };

            makeStyleRow(buttonStyleWrap,
                         tr(TXT_FLAT_BUTTONS),
                         lv_color_hex(uiButtonStyleFlatSelectorColor),
                         &lvglStyleButtonFlatBtn,
                         &lvglStyleButtonFlatBtnLabel,
                         lvglStyleButtonFlatEvent);
            makeStyleRow(buttonStyleWrap,
                         tr(TXT_3D_BUTTONS),
                         lv_color_hex(uiButtonStyle3dSelectorColor),
                         &lvglStyleButton3dBtn,
                         &lvglStyleButton3dBtnLabel,
                         lvglStyleButton3dEvent);
            makeStyleRow(buttonStyleWrap,
                         tr(TXT_BLACK_BUTTONS),
                         lv_color_hex(UI_BUTTON_BLACK_BODY_HEX),
                         &lvglStyleButtonBlackBtn,
                         &lvglStyleButtonBlackBtnLabel,
                         lvglStyleButtonBlackEvent);

            lv_obj_t *tzWrap = lv_obj_create(wrap);
            lv_obj_set_size(tzWrap, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(tzWrap, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(tzWrap, 0, 0);
            lv_obj_set_style_radius(tzWrap, 12, 0);
            lv_obj_set_style_pad_all(tzWrap, 10, 0);
            lv_obj_set_style_pad_row(tzWrap, 8, 0);
            lv_obj_set_flex_flow(tzWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(tzWrap, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *tzHdr = lv_label_create(tzWrap);
            lv_label_set_text(tzHdr, tr(TXT_TIMEZONE));
            lv_obj_set_style_text_color(tzHdr, lv_color_hex(0xE5ECF3), 0);

            lvglStyleTimezoneDd = lv_dropdown_create(tzWrap);
            lv_obj_set_width(lvglStyleTimezoneDd, lv_pct(100));
            lv_dropdown_set_options(lvglStyleTimezoneDd, buildGmtOffsetDropdownOptions().c_str());
            lv_obj_set_style_bg_color(lvglStyleTimezoneDd, lv_color_hex(0x111922), 0);
            lv_obj_set_style_text_color(lvglStyleTimezoneDd, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_style_border_color(lvglStyleTimezoneDd, lv_color_hex(0x2F4658), 0);
            lv_obj_set_style_border_width(lvglStyleTimezoneDd, 1, 0);
            lv_obj_set_style_radius(lvglStyleTimezoneDd, 8, 0);
            lv_obj_add_event_cb(lvglStyleTimezoneDd, lvglStyleTimezoneEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglStyleTimezoneDd, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglStyleTimezoneDd, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglStyleTimezoneDd, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);

            lv_obj_t *topCenterWrap = lv_obj_create(wrap);
            lv_obj_set_size(topCenterWrap, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(topCenterWrap, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(topCenterWrap, 0, 0);
            lv_obj_set_style_radius(topCenterWrap, 12, 0);
            lv_obj_set_style_pad_all(topCenterWrap, 10, 0);
            lv_obj_set_style_pad_row(topCenterWrap, 8, 0);
            lv_obj_set_flex_flow(topCenterWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(topCenterWrap, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *topCenterHdr = lv_label_create(topCenterWrap);
            lv_label_set_text(topCenterHdr, tr(TXT_TOP_BAR_CENTER));
            lv_obj_set_style_text_color(topCenterHdr, lv_color_hex(0xE5ECF3), 0);

            makeStyleRow(topCenterWrap,
                         tr(TXT_SHOW_DEVICE_NAME),
                         lv_color_hex(0x3F4A57),
                         &lvglStyleTopCenterNameBtn,
                         &lvglStyleTopCenterNameBtnLabel,
                         lvglStyleTopCenterNameEvent);
            makeStyleRow(topCenterWrap,
                         tr(TXT_SHOW_TIME),
                         lv_color_hex(0x3F4A57),
                         &lvglStyleTopCenterTimeBtn,
                         &lvglStyleTopCenterTimeBtnLabel,
                         lvglStyleTopCenterTimeEvent);

            lv_obj_t *timeWrap = lv_obj_create(wrap);
            lv_obj_set_size(timeWrap, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(timeWrap, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(timeWrap, 0, 0);
            lv_obj_set_style_radius(timeWrap, 12, 0);
            lv_obj_set_style_pad_all(timeWrap, 10, 0);
            lv_obj_set_style_pad_row(timeWrap, 6, 0);
            lv_obj_set_flex_flow(timeWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(timeWrap, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *timeHdr = lv_obj_create(timeWrap);
            lv_obj_set_size(timeHdr, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(timeHdr, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(timeHdr, 0, 0);
            lv_obj_set_style_pad_all(timeHdr, 0, 0);
            lv_obj_set_style_pad_column(timeHdr, 8, 0);
            lv_obj_set_flex_flow(timeHdr, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(timeHdr, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *timeLbl = lv_label_create(timeHdr);
            lv_label_set_text(timeLbl, tr(TXT_SCREEN_TIMEOFF));
            lv_obj_set_style_text_color(timeLbl, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_flex_grow(timeLbl, 1);

            lvglStyleTimeoutValueLabel = lv_label_create(timeHdr);
            lv_obj_set_style_text_color(lvglStyleTimeoutValueLabel, lv_color_hex(0xB7C4D1), 0);
            lv_label_set_text(lvglStyleTimeoutValueLabel, "--");

            lvglStyleTimeoutSlider = lv_slider_create(timeWrap);
            lv_obj_set_width(lvglStyleTimeoutSlider, lv_pct(100));
            lv_slider_set_range(lvglStyleTimeoutSlider,
                                static_cast<int32_t>(LCD_IDLE_TIMEOUT_MS_MIN / LCD_IDLE_TIMEOUT_STEP_MS),
                                static_cast<int32_t>(LCD_IDLE_TIMEOUT_MS_MAX / LCD_IDLE_TIMEOUT_STEP_MS));
            lv_obj_add_event_cb(lvglStyleTimeoutSlider, lvglStyleTimeoutEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglStyleTimeoutSlider, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglStyleTimeoutSlider, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglStyleTimeoutSlider, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);
            lvglApplyStyleScreenControlStyles();

            lv_obj_t *powerWrap = lv_obj_create(timeWrap);
            lv_obj_set_size(powerWrap, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(powerWrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(powerWrap, 0, 0);
            lv_obj_set_style_pad_all(powerWrap, 0, 0);
            lv_obj_set_style_pad_row(powerWrap, 6, 0);
            lv_obj_set_flex_flow(powerWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(powerWrap, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *powerHdr = lv_label_create(powerWrap);
            lv_label_set_text(powerHdr, tr(TXT_POWER_OFF));
            lv_obj_set_style_text_color(powerHdr, lv_color_hex(0xE5ECF3), 0);

            lvglStylePowerOffDropdown = lv_dropdown_create(powerWrap);
            lv_obj_set_width(lvglStylePowerOffDropdown, lv_pct(100));
            lv_dropdown_set_options(lvglStylePowerOffDropdown, buildPowerOffTimeoutDropdownOptions().c_str());
            lv_obj_set_style_bg_color(lvglStylePowerOffDropdown, lv_color_hex(0x111922), 0);
            lv_obj_set_style_text_color(lvglStylePowerOffDropdown, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_style_border_color(lvglStylePowerOffDropdown, lv_color_hex(0x2F4658), 0);
            lv_obj_set_style_border_width(lvglStylePowerOffDropdown, 1, 0);
            lv_obj_set_style_radius(lvglStylePowerOffDropdown, 8, 0);
            lv_obj_add_event_cb(lvglStylePowerOffDropdown, lvglStylePowerOffEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglStylePowerOffDropdown, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglStylePowerOffDropdown, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglStylePowerOffDropdown, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);
            if (POWER_OFF_SIGNAL_PIN < 0) lv_obj_add_flag(powerWrap, LV_OBJ_FLAG_HIDDEN);

            lv_obj_t *deepSleepWrap = lv_obj_create(timeWrap);
            lv_obj_set_size(deepSleepWrap, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(deepSleepWrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(deepSleepWrap, 0, 0);
            lv_obj_set_style_pad_all(deepSleepWrap, 0, 0);
            lv_obj_set_style_pad_row(deepSleepWrap, 6, 0);
            lv_obj_set_flex_flow(deepSleepWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(deepSleepWrap, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *deepSleepHdr = lv_label_create(deepSleepWrap);
            lv_label_set_text(deepSleepHdr, tr(TXT_DEEP_SLEEP));
            lv_obj_set_style_text_color(deepSleepHdr, lv_color_hex(0xE5ECF3), 0);

            lvglStyleDeepSleepDropdown = lv_dropdown_create(deepSleepWrap);
            lv_obj_set_width(lvglStyleDeepSleepDropdown, lv_pct(100));
            lv_dropdown_set_options(lvglStyleDeepSleepDropdown, buildDeepSleepTimeoutDropdownOptions().c_str());
            lv_obj_set_style_bg_color(lvglStyleDeepSleepDropdown, lv_color_hex(0x111922), 0);
            lv_obj_set_style_text_color(lvglStyleDeepSleepDropdown, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_style_border_color(lvglStyleDeepSleepDropdown, lv_color_hex(0x2F4658), 0);
            lv_obj_set_style_border_width(lvglStyleDeepSleepDropdown, 1, 0);
            lv_obj_set_style_radius(lvglStyleDeepSleepDropdown, 8, 0);
            lv_obj_add_event_cb(lvglStyleDeepSleepDropdown, lvglStyleDeepSleepEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglStyleDeepSleepDropdown, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglStyleDeepSleepDropdown, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglStyleDeepSleepDropdown, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);
            if (POWER_OFF_SIGNAL_PIN >= 0) lv_obj_add_flag(deepSleepWrap, LV_OBJ_FLAG_HIDDEN);

            lv_obj_t *hint = lv_label_create(timeWrap);
            lv_obj_set_width(hint, lv_pct(100));
            lv_label_set_long_mode(hint, LV_LABEL_LONG_WRAP);
            lv_obj_set_style_text_color(hint, lv_color_hex(0x9FB0C2), 0);
            lv_label_set_text(hint, tr(TXT_STYLE_HINT));
            lvglRefreshStyleUi();
            break;
        }
        case UI_CONFIG_STYLE_HOME_ITEMS: {
            lvglScrStyleHomeItems = lvglCreateScreenBase(tr(TXT_MAIN_SCREEN_ITEMS), false);
            lvglStyleHomeChatSw = nullptr;
            lvglStyleHomeRadioSw = nullptr;
            lvglStyleHomeMediaSw = nullptr;
            lvglStyleHomeInfoSw = nullptr;
            lvglStyleHomeGamesSw = nullptr;
            lvglStyleHomeMqttSw = nullptr;
            lvglStyleHomeAirplaneSw = nullptr;
            lvglStyleHomeApSw = nullptr;
            lvglStyleHomeChatLabel = nullptr;
            lvglStyleHomeRadioLabel = nullptr;
            lvglStyleHomeMediaLabel = nullptr;
            lvglStyleHomeInfoLabel = nullptr;
            lvglStyleHomeGamesLabel = nullptr;
            lvglStyleHomeMqttLabel = nullptr;
            lvglStyleHomeAirplaneLabel = nullptr;
            lvglStyleHomeApLabel = nullptr;

            lv_obj_t *wrap = lv_obj_create(lvglScrStyleHomeItems);
            lv_obj_set_size(wrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(wrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(wrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(wrap, 0, 0);
            lv_obj_set_style_pad_all(wrap, 10, 0);
            lv_obj_set_style_pad_row(wrap, 10, 0);
            lv_obj_set_flex_flow(wrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scrollbar_mode(wrap, LV_SCROLLBAR_MODE_OFF);

            auto makeHomeRow = [&](lv_obj_t **labelOut, lv_obj_t **switchOut, const char *labelText, lv_event_cb_t cb) {
                lv_obj_t *row = lv_obj_create(wrap);
                lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
                lv_obj_set_style_bg_color(row, lv_color_hex(0x18222D), 0);
                lv_obj_set_style_border_width(row, 0, 0);
                lv_obj_set_style_radius(row, 12, 0);
                lv_obj_set_style_pad_all(row, 12, 0);
                lv_obj_set_style_pad_column(row, 8, 0);
                lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
                lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

                lv_obj_t *lbl = lv_label_create(row);
                lv_label_set_text(lbl, labelText);
                lv_obj_set_style_text_color(lbl, lv_color_hex(0xE5ECF3), 0);
                lv_obj_set_flex_grow(lbl, 1);

                lv_obj_t *sw = lv_switch_create(row);
                lv_obj_add_event_cb(sw, cb, LV_EVENT_VALUE_CHANGED, nullptr);

                *labelOut = lbl;
                *switchOut = sw;
            };

            makeHomeRow(&lvglStyleHomeChatLabel, &lvglStyleHomeChatSw, tr(TXT_CHAT), lvglStyleHomeChatToggleEvent);
            makeHomeRow(&lvglStyleHomeRadioLabel, &lvglStyleHomeRadioSw, tr(TXT_RADIO_CONTROL), lvglStyleHomeRadioToggleEvent);
            makeHomeRow(&lvglStyleHomeMediaLabel, &lvglStyleHomeMediaSw, tr(TXT_MEDIA), lvglStyleHomeMediaToggleEvent);
            makeHomeRow(&lvglStyleHomeInfoLabel, &lvglStyleHomeInfoSw, tr(TXT_INFO), lvglStyleHomeInfoToggleEvent);
            makeHomeRow(&lvglStyleHomeGamesLabel, &lvglStyleHomeGamesSw, tr(TXT_GAMES), lvglStyleHomeGamesToggleEvent);
            makeHomeRow(&lvglStyleHomeMqttLabel, &lvglStyleHomeMqttSw, tr(TXT_MQTT_CONTROLS), lvglStyleHomeMqttToggleEvent);
            makeHomeRow(&lvglStyleHomeAirplaneLabel, &lvglStyleHomeAirplaneSw, tr(TXT_AIRPLANE_MODE), lvglStyleHomeAirplaneToggleEvent);
            makeHomeRow(&lvglStyleHomeApLabel, &lvglStyleHomeApSw, tr(TXT_AP_MODE), lvglStyleHomeApToggleEvent);

            lvglApplyStyleScreenControlStyles();
            lvglRefreshStyleHomeItemsUi();
            break;
        }
        case UI_CONFIG_LANGUAGE: {
            lvglScrLanguage = lvglCreateScreenBase(tr(TXT_LANGUAGE), false);
            lvglLanguageDropdown = nullptr;
            lvglLanguageInfoLabel = nullptr;

            lv_obj_t *wrap = lv_obj_create(lvglScrLanguage);
            lv_obj_set_size(wrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(wrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(wrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(wrap, 0, 0);
            lv_obj_set_style_pad_all(wrap, 10, 0);
            lv_obj_set_style_pad_row(wrap, 10, 0);
            lv_obj_set_flex_flow(wrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scrollbar_mode(wrap, LV_SCROLLBAR_MODE_OFF);

            lv_obj_t *langCard = lv_obj_create(wrap);
            lv_obj_set_size(langCard, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(langCard, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(langCard, 0, 0);
            lv_obj_set_style_radius(langCard, 12, 0);
            lv_obj_set_style_pad_all(langCard, 12, 0);
            lv_obj_set_style_pad_row(langCard, 8, 0);
            lv_obj_set_flex_flow(langCard, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(langCard, LV_OBJ_FLAG_SCROLLABLE);

            lvglLanguageInfoLabel = lv_label_create(langCard);
            lv_obj_set_width(lvglLanguageInfoLabel, lv_pct(100));
            lv_label_set_long_mode(lvglLanguageInfoLabel, LV_LABEL_LONG_WRAP);
            lv_obj_set_style_text_color(lvglLanguageInfoLabel, lv_color_hex(0xE5ECF3), 0);

            lvglLanguageDropdown = lv_dropdown_create(langCard);
            lv_obj_set_width(lvglLanguageDropdown, lv_pct(100));
            lv_dropdown_set_options(lvglLanguageDropdown, buildLanguageDropdownOptions().c_str());
            lv_obj_set_style_bg_color(lvglLanguageDropdown, lv_color_hex(0x111922), 0);
            lv_obj_set_style_text_color(lvglLanguageDropdown, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_style_border_color(lvglLanguageDropdown, lv_color_hex(0x2F4658), 0);
            lv_obj_set_style_border_width(lvglLanguageDropdown, 1, 0);
            lv_obj_set_style_radius(lvglLanguageDropdown, 8, 0);
            lv_obj_add_event_cb(lvglLanguageDropdown, lvglLanguageDropdownEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglLanguageDropdown, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglLanguageDropdown, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglLanguageDropdown, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);
            lvglRefreshLanguageUi();
            break;
        }
        case UI_CONFIG_OTA: {
            lvglScrOta = lvglCreateScreenBase(tr(TXT_OTA_UPDATES), false);
            lv_obj_t *wrap = lv_obj_create(lvglScrOta);
            lv_obj_set_size(wrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(wrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(wrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(wrap, 0, 0);
            lv_obj_set_style_pad_all(wrap, 10, 0);
            lv_obj_set_style_pad_row(wrap, 10, 0);
            lv_obj_set_flex_flow(wrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scroll_dir(wrap, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(wrap, LV_SCROLLBAR_MODE_OFF);

            auto makeInfoLine = [&](const char *title, lv_obj_t **valueOut) -> lv_obj_t * {
                lv_obj_t *card = lv_obj_create(wrap);
                lv_obj_set_width(card, lv_pct(100));
                lv_obj_set_height(card, LV_SIZE_CONTENT);
                lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
                lv_obj_set_flex_align(card, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
                lv_obj_set_style_bg_color(card, lv_color_hex(0x16212C), 0);
                lv_obj_set_style_border_color(card, lv_color_hex(0x536274), 0);
                lv_obj_set_style_border_width(card, 1, 0);
                lv_obj_set_style_radius(card, 12, 0);
                lv_obj_set_style_pad_all(card, 10, 0);
                lv_obj_set_style_pad_row(card, 4, 0);
                lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_t *label = lv_label_create(card);
                lv_label_set_text(label, title);
                lv_obj_set_style_text_color(label, lv_color_hex(0x9FB0C2), 0);
                lv_obj_t *value = lv_label_create(card);
                lv_obj_set_width(value, lv_pct(100));
                lv_label_set_long_mode(value, LV_LABEL_LONG_WRAP);
                lv_label_set_text(value, "--");
                lv_obj_set_style_text_color(value, lv_color_hex(0xF5F8FB), 0);
                *valueOut = value;
                return card;
            };

            lv_obj_t *otaCurrentCard = makeInfoLine(tr(TXT_OTA_CURRENT_VERSION), &lvglOtaCurrentLabel);
            lv_obj_t *otaLatestCard = makeInfoLine(tr(TXT_OTA_LATEST_VERSION), &lvglOtaLatestLabel);
            lv_obj_t *otaStatusCard = makeInfoLine(tr(TXT_OTA_STATUS), &lvglOtaStatusLabel);

            lvglOtaProgressBar = lv_bar_create(wrap);
            lv_obj_set_size(lvglOtaProgressBar, lv_pct(100), 14);
            lv_bar_set_range(lvglOtaProgressBar, 0, 100);
            lv_bar_set_value(lvglOtaProgressBar, 0, LV_ANIM_OFF);
            lv_obj_set_style_bg_color(lvglOtaProgressBar, lv_color_hex(0x2A3340), LV_PART_MAIN);
            lv_obj_set_style_bg_color(lvglOtaProgressBar, lv_color_hex(0x52B788), LV_PART_INDICATOR);

            lvglOtaProgressLabel = lv_label_create(wrap);
            lv_obj_set_width(lvglOtaProgressLabel, lv_pct(100));
            lv_obj_set_style_text_align(lvglOtaProgressLabel, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_set_style_text_color(lvglOtaProgressLabel, lv_color_hex(0xC8D3DD), 0);
            lv_label_set_text(lvglOtaProgressLabel, "");

            lvglOtaUpdateBtn = lvglCreateMenuButton(wrap, lvglSymbolText(LV_SYMBOL_UPLOAD, tr(TXT_OTA_UPDATE)).c_str(), lv_color_hex(0x3A8F4B), lvglOtaUpdateEvent, nullptr);
            if (lvglOtaUpdateBtn) lvglOtaUpdateBtnLabel = lv_obj_get_child(lvglOtaUpdateBtn, 0);
            lvglOtaReflashBtn = lvglCreateMenuButton(wrap, lvglSymbolText(LV_SYMBOL_REFRESH, tr(TXT_OTA_FLUSH_SELECTED)).c_str(), lv_color_hex(0x7A5C2E), lvglOtaReflashEvent, nullptr);
            if (lvglOtaReflashBtn) lvglOtaReflashBtnLabel = lv_obj_get_child(lvglOtaReflashBtn, 0);

            lvglOtaReleaseHeaderLabel = lv_label_create(wrap);
            lv_obj_set_width(lvglOtaReleaseHeaderLabel, lv_pct(100));
            lv_obj_set_style_text_color(lvglOtaReleaseHeaderLabel, lv_color_hex(0x9FB0C2), 0);
            lv_label_set_text(lvglOtaReleaseHeaderLabel, tr(TXT_OTA_PREVIOUS_RELEASES));

            for (uint8_t i = 0; i < OTA_RELEASE_LIST_MAX; ++i) {
                lvglOtaReleaseChecks[i] = lv_checkbox_create(wrap);
                lv_obj_set_width(lvglOtaReleaseChecks[i], lv_pct(100));
                lv_checkbox_set_text(lvglOtaReleaseChecks[i], "--");
                lv_obj_add_event_cb(lvglOtaReleaseChecks[i], lvglOtaReleaseSelectEvent, LV_EVENT_VALUE_CHANGED, reinterpret_cast<void *>(static_cast<intptr_t>(i)));
                lv_obj_add_event_cb(lvglOtaReleaseChecks[i], lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
                lv_obj_add_event_cb(lvglOtaReleaseChecks[i], lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
                lv_obj_add_event_cb(lvglOtaReleaseChecks[i], lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);
            }
            lvglRegisterReorderableItem(otaCurrentCard, "ord_ota", "cur");
            lvglRegisterReorderableItem(otaLatestCard, "ord_ota", "latest");
            lvglRegisterReorderableItem(otaStatusCard, "ord_ota", "status");
            lvglRegisterReorderableItem(lvglOtaProgressBar, "ord_ota", "bar");
            lvglRegisterReorderableItem(lvglOtaProgressLabel, "ord_ota", "pct");
            lvglRegisterReorderableItem(lvglOtaUpdateBtn, "ord_ota", "update");
            lvglRegisterReorderableItem(lvglOtaReflashBtn, "ord_ota", "reflash");
            lvglRegisterReorderableItem(lvglOtaReleaseHeaderLabel, "ord_ota", "rel_hdr");
            for (uint8_t i = 0; i < OTA_RELEASE_LIST_MAX; ++i) {
                const String relKey = String("rel_") + String(i);
                lvglRegisterReorderableItem(lvglOtaReleaseChecks[i], "ord_ota", relKey.c_str());
            }
            lvglApplySavedOrder(wrap, "ord_ota");
            lvglRefreshOtaUi();
            break;
        }
        case UI_CONFIG_MODULES: {
            lvglScrModules = lvglCreateScreenBase("Modules", false);
            lvglModulesWrap = lv_obj_create(lvglScrModules);
            lv_obj_set_size(lvglModulesWrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(lvglModulesWrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(lvglModulesWrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(lvglModulesWrap, 0, 0);
            lv_obj_set_style_pad_all(lvglModulesWrap, 10, 0);
            lv_obj_set_style_pad_row(lvglModulesWrap, 10, 0);
            lv_obj_set_flex_flow(lvglModulesWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scroll_dir(lvglModulesWrap, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(lvglModulesWrap, LV_SCROLLBAR_MODE_OFF);

            lvglModulesStatusLabel = lv_label_create(lvglModulesWrap);
            lv_obj_set_width(lvglModulesStatusLabel, lv_pct(100));
            lv_label_set_long_mode(lvglModulesStatusLabel, LV_LABEL_LONG_WRAP);
            lv_obj_set_style_text_color(lvglModulesStatusLabel, lv_color_hex(0x9FB0C2), 0);
            lv_label_set_text(lvglModulesStatusLabel, "Loading modules...");

            for (uint8_t i = 0; i < APP_MODULE_COUNT; ++i) {
                lv_obj_t *card = lv_obj_create(lvglModulesWrap);
                lv_obj_set_width(card, lv_pct(100));
                lv_obj_set_height(card, LV_SIZE_CONTENT);
                lv_obj_set_style_bg_color(card, lv_color_hex(0x16212C), 0);
                lv_obj_set_style_border_color(card, lv_color_hex(0x536274), 0);
                lv_obj_set_style_border_width(card, 1, 0);
                lv_obj_set_style_radius(card, 12, 0);
                lv_obj_set_style_pad_all(card, 12, 0);
                lv_obj_set_style_pad_row(card, 6, 0);
                lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
                lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

                lvglModulesNameLabels[i] = lv_label_create(card);
                lv_obj_set_width(lvglModulesNameLabels[i], lv_pct(100));
                lv_obj_set_style_text_color(lvglModulesNameLabels[i], lv_color_hex(0xF5F8FB), 0);

                lvglModulesDescLabels[i] = lv_label_create(card);
                lv_obj_set_width(lvglModulesDescLabels[i], lv_pct(100));
                lv_label_set_long_mode(lvglModulesDescLabels[i], LV_LABEL_LONG_WRAP);
                lv_obj_set_style_text_color(lvglModulesDescLabels[i], lv_color_hex(0xB9C7D5), 0);

                lvglModulesVersionLabels[i] = lv_label_create(card);
                lv_obj_set_width(lvglModulesVersionLabels[i], lv_pct(100));
                lv_label_set_long_mode(lvglModulesVersionLabels[i], LV_LABEL_LONG_WRAP);
                lv_obj_set_style_text_color(lvglModulesVersionLabels[i], lv_color_hex(0x8EA5BA), 0);

                lv_obj_t *btn = lv_btn_create(card);
                lv_obj_set_size(btn, lv_pct(100), 40);
                lv_obj_set_style_radius(btn, 10, 0);
                lv_obj_set_style_border_width(btn, 0, 0);
                lv_obj_add_event_cb(btn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
                lv_obj_add_event_cb(btn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
                lv_obj_add_event_cb(btn, lvglModuleActionEvent, LV_EVENT_CLICKED, reinterpret_cast<void *>(static_cast<intptr_t>(i)));
                lvglModulesActionBtns[i] = btn;

                lv_obj_t *lbl = lv_label_create(btn);
                lv_obj_center(lbl);
                lvglModulesActionLabels[i] = lbl;
            }
            lvglRefreshModulesUi();
            break;
        }
        case UI_CONFIG_HC12: {
            lvglScrHc12 = lvglCreateScreenBase("Radio Config", false);
            lv_obj_t *wrap = lv_obj_create(lvglScrHc12);
            lv_obj_set_size(wrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(wrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(wrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(wrap, 0, 0);
            lv_obj_set_style_pad_all(wrap, 8, 0);
            lv_obj_set_style_pad_row(wrap, 8, 0);
            lv_obj_set_flex_flow(wrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scroll_dir(wrap, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(wrap, LV_SCROLLBAR_MODE_OFF);

            auto makeSelectorRow = [&](const char *title,
                                       lv_color_t accent,
                                       lv_event_cb_t prevCb,
                                       lv_event_cb_t nextCb,
                                       lv_obj_t **titleOut,
                                       lv_obj_t **valueOut,
                                       lv_obj_t **subOut) -> lv_obj_t * {
                lv_obj_t *card = lv_obj_create(wrap);
                lv_obj_set_width(card, lv_pct(100));
                lv_obj_set_height(card, LV_SIZE_CONTENT);
                lv_obj_set_style_bg_color(card, lv_color_hex(0x16212C), 0);
                lv_obj_set_style_border_color(card, lv_color_hex(0x536274), 0);
                lv_obj_set_style_border_width(card, 1, 0);
                lv_obj_set_style_radius(card, 12, 0);
                lv_obj_set_style_pad_all(card, 10, 0);
                lv_obj_set_style_pad_row(card, 8, 0);
                lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
                lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

                lv_obj_t *titleLbl = lv_label_create(card);
                lv_label_set_text(titleLbl, title);
                lv_obj_set_style_text_color(titleLbl, lv_color_hex(0x9FB0C2), 0);
                if (titleOut) *titleOut = titleLbl;

                lv_obj_t *row = lv_obj_create(card);
                lv_obj_set_width(row, lv_pct(100));
                lv_obj_set_height(row, LV_SIZE_CONTENT);
                lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
                lv_obj_set_style_border_width(row, 0, 0);
                lv_obj_set_style_pad_all(row, 0, 0);
                lv_obj_set_style_pad_column(row, 8, 0);
                lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
                lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
                lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

                makeSmallBtn(row, LV_SYMBOL_LEFT, 44, 38, accent, prevCb, nullptr);

                lv_obj_t *center = lv_obj_create(row);
                lv_obj_set_flex_grow(center, 1);
                lv_obj_set_height(center, LV_SIZE_CONTENT);
                lv_obj_set_style_bg_color(center, lv_color_hex(0x101922), 0);
                lv_obj_set_style_border_color(center, lv_color_hex(0x2D495E), 0);
                lv_obj_set_style_border_width(center, 1, 0);
                lv_obj_set_style_radius(center, 10, 0);
                lv_obj_set_style_pad_all(center, 8, 0);
                lv_obj_set_style_pad_row(center, 2, 0);
                lv_obj_set_flex_flow(center, LV_FLEX_FLOW_COLUMN);
                lv_obj_set_flex_align(center, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
                lv_obj_clear_flag(center, LV_OBJ_FLAG_SCROLLABLE);

                lv_obj_t *valueLbl = lv_label_create(center);
                lv_obj_set_width(valueLbl, lv_pct(100));
                lv_obj_set_style_text_color(valueLbl, lv_color_hex(0xF5F8FB), 0);
                lv_obj_set_style_text_align(valueLbl, LV_TEXT_ALIGN_CENTER, 0);
                lv_label_set_long_mode(valueLbl, LV_LABEL_LONG_CLIP);
                lv_label_set_text(valueLbl, "--");
                *valueOut = valueLbl;

                lv_obj_t *subLbl = lv_label_create(center);
                lv_obj_set_width(subLbl, lv_pct(100));
                lv_obj_set_style_text_color(subLbl, lv_color_hex(0xA8BACB), 0);
                lv_obj_set_style_text_align(subLbl, LV_TEXT_ALIGN_CENTER, 0);
                lv_label_set_long_mode(subLbl, LV_LABEL_LONG_CLIP);
                lv_label_set_text(subLbl, "");
                *subOut = subLbl;

                makeSmallBtn(row, LV_SYMBOL_RIGHT, 44, 38, accent, nextCb, nullptr);
                return card;
            };

            lvglRadioModuleHeader = lv_label_create(wrap);
            lv_label_set_text(lvglRadioModuleHeader, tr(TXT_RADIO_MODULE));
            lv_obj_set_style_text_color(lvglRadioModuleHeader, lv_color_hex(0xE5ECF3), 0);

            lvglRadioModuleDropdown = lv_dropdown_create(wrap);
            lv_obj_set_width(lvglRadioModuleDropdown, lv_pct(100));
            lv_dropdown_set_options(lvglRadioModuleDropdown, buildRadioModuleDropdownOptions().c_str());
            lv_obj_set_style_bg_color(lvglRadioModuleDropdown, lv_color_hex(0x111922), 0);
            lv_obj_set_style_text_color(lvglRadioModuleDropdown, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_style_border_color(lvglRadioModuleDropdown, lv_color_hex(0x2F4658), 0);
            lv_obj_set_style_border_width(lvglRadioModuleDropdown, 1, 0);
            lv_obj_set_style_radius(lvglRadioModuleDropdown, 8, 0);
            lv_obj_add_event_cb(lvglRadioModuleDropdown, lvglRadioModuleDropdownEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglRadioModuleDropdown, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglRadioModuleDropdown, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglRadioModuleDropdown, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);

            lv_obj_t *hc12ChannelCard = makeSelectorRow(tr(TXT_CHANNEL), lv_color_hex(0x7A5C2E), lvglHc12PrevChannelEvent, lvglHc12NextChannelEvent,
                                                        &lvglRadioChannelTitleLabel, &lvglHc12ChannelValueLabel, &lvglHc12ChannelSubLabel);
            lv_obj_t *hc12BaudCard = makeSelectorRow(tr(TXT_BAUD_RATE), lv_color_hex(0x2F6D86), lvglHc12PrevBaudEvent, lvglHc12NextBaudEvent,
                                                    &lvglRadioBaudTitleLabel, &lvglHc12BaudValueLabel, &lvglHc12BaudSubLabel);
            lv_obj_t *hc12ModeCard = makeSelectorRow("Transmission Mode", lv_color_hex(0x355E8A), lvglHc12PrevModeEvent, lvglHc12NextModeEvent,
                                                    &lvglRadioModeTitleLabel, &lvglHc12ModeValueLabel, &lvglHc12ModeSubLabel);
            lv_obj_t *hc12PowerCard = makeSelectorRow("Transmission Power", lv_color_hex(0xA35757), lvglHc12PrevPowerEvent, lvglHc12NextPowerEvent,
                                                    &lvglRadioPowerTitleLabel, &lvglHc12PowerValueLabel, &lvglHc12PowerSubLabel);
            lvglRadioExtraCard = makeSelectorRow("Transfer Mode", lv_color_hex(0x5E7B35), lvglHc12PrevExtraEvent, lvglHc12NextExtraEvent,
                                                &lvglRadioExtraTitleLabel, &lvglRadioExtraValueLabel, &lvglRadioExtraSubLabel);

            /* Channel: keep top centered, make bottom scrolling */
            if (lvglHc12ChannelValueLabel) {
                lv_obj_set_width(lvglHc12ChannelValueLabel, lv_pct(100));
                lv_obj_set_style_text_align(lvglHc12ChannelValueLabel, LV_TEXT_ALIGN_CENTER, 0);
                lv_label_set_long_mode(lvglHc12ChannelValueLabel, LV_LABEL_LONG_CLIP);
            }
            if (lvglHc12ChannelSubLabel) {
                lv_obj_set_width(lvglHc12ChannelSubLabel, lv_pct(100));
                lv_obj_set_style_text_align(lvglHc12ChannelSubLabel, LV_TEXT_ALIGN_CENTER, 0);
                lv_label_set_long_mode(lvglHc12ChannelSubLabel, LV_LABEL_LONG_WRAP);
            }

            /* Transfer Mode: keep top centered, make bottom scrolling */
            if (lvglRadioExtraValueLabel) {
                lv_obj_set_width(lvglRadioExtraValueLabel, lv_pct(100));
                lv_obj_set_style_text_align(lvglRadioExtraValueLabel, LV_TEXT_ALIGN_CENTER, 0);
                lv_label_set_long_mode(lvglRadioExtraValueLabel, LV_LABEL_LONG_CLIP);
            }
            if (lvglRadioExtraSubLabel) {
                lv_obj_set_width(lvglRadioExtraSubLabel, lv_pct(100));
                lv_obj_set_style_text_align(lvglRadioExtraSubLabel, LV_TEXT_ALIGN_CENTER, 0);
                lv_label_set_long_mode(lvglRadioExtraSubLabel, LV_LABEL_LONG_WRAP);
            }

            lvglRadioPinSwapCard = makeSelectorRow("UART Pins", lv_color_hex(0x4E6786), lvglRadioPrevPinSwapEvent, lvglRadioNextPinSwapEvent,
                                                &lvglRadioPinSwapTitleLabel, &lvglRadioPinSwapValueLabel, &lvglRadioPinSwapSubLabel);
            lvglRadioModePinSwapCard = makeSelectorRow("Mode Pins", lv_color_hex(0x6A7D42), lvglRadioPrevModePinSwapEvent, lvglRadioNextModePinSwapEvent,
                                                    &lvglRadioModePinSwapTitleLabel, &lvglRadioModePinSwapValueLabel, &lvglRadioModePinSwapSubLabel);

            if (lvglRadioPinSwapSubLabel) {
                lv_obj_set_width(lvglRadioPinSwapSubLabel, lv_pct(100));
                lv_obj_set_style_text_align(lvglRadioPinSwapSubLabel, LV_TEXT_ALIGN_CENTER, 0);
                lv_label_set_long_mode(lvglRadioPinSwapSubLabel, LV_LABEL_LONG_WRAP);
            }

            lv_obj_t *hc12DefaultBtn = lvglCreateMenuButton(wrap, lvglSymbolText(LV_SYMBOL_REFRESH, "Default").c_str(), lv_color_hex(0xA35757), lvglHc12DefaultEvent, nullptr);
            lvglHc12ConfigStatusLabel = lv_label_create(wrap);
            if (lvglHc12ConfigStatusLabel) {
                lv_obj_set_width(lvglHc12ConfigStatusLabel, lv_pct(100));
                lv_label_set_long_mode(lvglHc12ConfigStatusLabel, LV_LABEL_LONG_WRAP);
                lv_obj_set_style_text_align(lvglHc12ConfigStatusLabel, LV_TEXT_ALIGN_CENTER, 0);
                lv_obj_set_style_text_color(lvglHc12ConfigStatusLabel, lv_color_hex(0xA8BACB), 0);
            }

            lv_obj_t *hc12TerminalBtn = lvglCreateMenuButton(wrap, lvglSymbolText(LV_SYMBOL_KEYBOARD, "Radio Terminal").c_str(), lv_color_hex(0x2F6D86), lvglOpenHc12TerminalEvent, nullptr);
            lv_obj_t *hc12InfoBtn = lvglCreateMenuButton(wrap, lvglSymbolText(LV_SYMBOL_LIST, tr(TXT_HC12_INFO)).c_str(), lv_color_hex(0x7A5C2E), lvglOpenHc12InfoEvent, nullptr);

            lv_obj_t *hintCard = lv_obj_create(wrap);
            lv_obj_set_width(hintCard, lv_pct(100));
            lv_obj_set_height(hintCard, LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(hintCard, lv_color_hex(0x16212C), 0);
            lv_obj_set_style_border_color(hintCard, lv_color_hex(0x536274), 0);
            lv_obj_set_style_border_width(hintCard, 1, 0);
            lv_obj_set_style_radius(hintCard, 12, 0);
            lv_obj_set_style_pad_all(hintCard, 10, 0);
            lv_obj_clear_flag(hintCard, LV_OBJ_FLAG_SCROLLABLE);

            lvglRadioHintLabel = lv_label_create(hintCard);
            lv_obj_set_width(lvglRadioHintLabel, lv_pct(100));
            lv_label_set_long_mode(lvglRadioHintLabel, LV_LABEL_LONG_WRAP);
            lv_obj_set_style_text_color(lvglRadioHintLabel, lv_color_hex(0xC8D3DD), 0);
            lv_label_set_text(lvglRadioHintLabel, "");
            if (lvglRadioModuleHeader) lvglRegisterReorderableItem(lvglRadioModuleHeader, "ord_hc12", "module_hdr");
            if (lvglRadioModuleDropdown) lvglRegisterReorderableItem(lvglRadioModuleDropdown, "ord_hc12", "module_dd");
            lvglRegisterReorderableItem(hc12ChannelCard, "ord_hc12", "chan");
            lvglRegisterReorderableItem(hc12BaudCard, "ord_hc12", "baud");
            lvglRegisterReorderableItem(hc12ModeCard, "ord_hc12", "mode");
            lvglRegisterReorderableItem(hc12PowerCard, "ord_hc12", "power");
            lvglRegisterReorderableItem(lvglRadioExtraCard, "ord_hc12", "extra");
            lvglRegisterReorderableItem(lvglRadioPinSwapCard, "ord_hc12", "pins");
            lvglRegisterReorderableItem(lvglRadioModePinSwapCard, "ord_hc12", "modepins");
            lvglRegisterReorderableItem(hc12DefaultBtn, "ord_hc12", "default");
            lvglRegisterReorderableItem(lvglHc12ConfigStatusLabel, "ord_hc12", "status");
            lvglRegisterReorderableItem(hc12TerminalBtn, "ord_hc12", "term");
            lvglRegisterReorderableItem(hc12InfoBtn, "ord_hc12", "info");
            lvglRegisterReorderableItem(hintCard, "ord_hc12", "hint");
            lvglApplySavedOrder(wrap, "ord_hc12");
            lvglRefreshHc12ConfigUi();
            break;
        }
        case UI_CONFIG_HC12_TERMINAL: {
            lvglScrHc12Terminal = lvglCreateScreenBase("Radio Terminal", false);
            lv_obj_t *wrap = lv_obj_create(lvglScrHc12Terminal);
            lv_obj_set_size(wrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(wrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_color(wrap, lv_color_hex(0x111922), 0);
            lv_obj_set_style_border_width(wrap, 0, 0);
            lv_obj_set_style_pad_all(wrap, 8, 0);
            lv_obj_set_style_pad_row(wrap, 8, 0);
            lv_obj_set_flex_flow(wrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scroll_dir(wrap, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(wrap, LV_SCROLLBAR_MODE_OFF);

            lv_obj_t *topRow = lv_obj_create(wrap);
            lv_obj_set_size(topRow, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(topRow, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(topRow, 0, 0);
            lv_obj_set_style_pad_all(topRow, 0, 0);
            lv_obj_set_style_pad_column(topRow, 8, 0);
            lv_obj_set_flex_flow(topRow, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(topRow, LV_OBJ_FLAG_SCROLLABLE);

            makeSmallBtn(topRow, "CFG: OFF", 96, 34, lv_color_hex(0x7A5C2E), lvglHc12ToggleSetEvent, nullptr);

            lvglHc12TerminalPinLabel = lv_label_create(topRow);
            lv_obj_set_flex_grow(lvglHc12TerminalPinLabel, 1);
            lv_obj_set_style_text_color(lvglHc12TerminalPinLabel, lv_color_hex(0xC9D7E3), 0);
            lv_label_set_text(lvglHc12TerminalPinLabel, "Radio serial pins");

            lv_obj_t *terminalTa = lv_textarea_create(wrap);
            lv_obj_set_width(terminalTa, lv_pct(100));
            lv_obj_set_height(terminalTa, UI_CONTENT_H - 108);
            lv_textarea_set_one_line(terminalTa, false);
            lv_textarea_set_placeholder_text(terminalTa, "HC-12 terminal output");
            lv_obj_set_style_bg_color(terminalTa, lv_color_hex(0x091118), 0);
            lv_obj_set_style_text_color(terminalTa, lv_color_hex(0x82F6A2), 0);
            lv_obj_set_style_border_color(terminalTa, lv_color_hex(0x2D495E), 0);
            lv_obj_set_style_border_width(terminalTa, 1, 0);
            lv_obj_clear_flag(terminalTa, LV_OBJ_FLAG_CLICKABLE);
            lv_textarea_set_cursor_click_pos(terminalTa, false);

            lv_obj_t *cmdRow = lv_obj_create(wrap);
            lv_obj_set_size(cmdRow, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(cmdRow, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(cmdRow, 0, 0);
            lv_obj_set_style_pad_all(cmdRow, 0, 0);
            lv_obj_set_style_pad_column(cmdRow, 8, 0);
            lv_obj_set_flex_flow(cmdRow, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(cmdRow, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *cmdTa = lv_textarea_create(cmdRow);
            lv_obj_set_height(cmdTa, 36);
            lv_obj_set_width(cmdTa, lv_pct(100));
            lv_obj_set_flex_grow(cmdTa, 1);
            lv_textarea_set_one_line(cmdTa, true);
            lv_textarea_set_placeholder_text(cmdTa, "AT or text to send");
            lv_obj_add_event_cb(cmdTa, lvglTextAreaFocusEvent, LV_EVENT_FOCUSED, nullptr);

            makeSmallBtn(cmdRow, "Send", 70, 36, lv_color_hex(0x2F6D86), lvglHc12SendEvent, nullptr);
            lvglHc12TerminalExamplesLabel = lv_label_create(wrap);
            lv_obj_set_width(lvglHc12TerminalExamplesLabel, lv_pct(100));
            lv_label_set_long_mode(lvglHc12TerminalExamplesLabel, LV_LABEL_LONG_WRAP);
            lv_obj_set_style_text_color(lvglHc12TerminalExamplesLabel, lv_color_hex(0xA8BACB), 0);
            lv_label_set_text(lvglHc12TerminalExamplesLabel, "");
            lvglRegisterReorderableItem(topRow, "ord_hct", "top");
            lvglRegisterReorderableItem(terminalTa, "ord_hct", "term");
            lvglRegisterReorderableItem(cmdRow, "ord_hct", "cmd");
            lvglRegisterReorderableItem(lvglHc12TerminalExamplesLabel, "ord_hct", "examples");
            lvglApplySavedOrder(wrap, "ord_hct");
            hc12InitIfNeeded();
            lvglRefreshHc12Ui();
            break;
        }
        case UI_CONFIG_HC12_INFO: {
            lvglScrHc12Info = lvglCreateScreenBase(tr(TXT_HC12_INFO), false);
            lv_obj_t *wrap = lv_obj_create(lvglScrHc12Info);
            lv_obj_set_size(wrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(wrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(wrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(wrap, 0, 0);
            lv_obj_set_style_pad_all(wrap, 8, 0);
            lv_obj_set_style_pad_row(wrap, 8, 0);
            lv_obj_set_flex_flow(wrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scroll_dir(wrap, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(wrap, LV_SCROLLBAR_MODE_OFF);

            auto makeInfoLine = [&](const char *title, lv_obj_t **valueOut) -> lv_obj_t * {
                lv_obj_t *card = lv_obj_create(wrap);
                lv_obj_set_width(card, lv_pct(100));
                lv_obj_set_height(card, LV_SIZE_CONTENT);
                lv_obj_set_style_bg_color(card, lv_color_hex(0x16212C), 0);
                lv_obj_set_style_border_color(card, lv_color_hex(0x536274), 0);
                lv_obj_set_style_border_width(card, 1, 0);
                lv_obj_set_style_radius(card, 12, 0);
                lv_obj_set_style_pad_all(card, 10, 0);
                lv_obj_set_style_pad_row(card, 4, 0);
                lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
                lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_t *label = lv_label_create(card);
                lv_label_set_text(label, title);
                lv_obj_set_style_text_color(label, lv_color_hex(0x9FB0C2), 0);
                lv_obj_t *value = lv_label_create(card);
                lv_obj_set_width(value, lv_pct(100));
                lv_label_set_long_mode(value, LV_LABEL_LONG_WRAP);
                lv_label_set_text(value, "--");
                lv_obj_set_style_text_color(value, lv_color_hex(0xF5F8FB), 0);
                *valueOut = value;
                return card;
            };

            lv_obj_t *hc12InfoVersionCard = makeInfoLine(tr(TXT_VERSION), &lvglHc12InfoVersionLabel);
            lv_obj_t *hc12InfoBaudCard = makeInfoLine(tr(TXT_BAUD_RATE), &lvglHc12InfoBaudLabel);
            lv_obj_t *hc12InfoChannelCard = makeInfoLine(tr(TXT_CHANNEL), &lvglHc12InfoChannelLabel);
            lv_obj_t *hc12InfoFuCard = makeInfoLine(tr(TXT_FU_MODE), &lvglHc12InfoFuModeLabel);
            lv_obj_t *hc12InfoPowerCard = makeInfoLine(tr(TXT_POWER), &lvglHc12InfoPowerLabel);
            lv_obj_t *hc12InfoRawCard = makeInfoLine(tr(TXT_RAW), &lvglHc12InfoRawLabel);
            lvglRegisterReorderableItem(hc12InfoVersionCard, "ord_hci", "ver");
            lvglRegisterReorderableItem(hc12InfoBaudCard, "ord_hci", "baud");
            lvglRegisterReorderableItem(hc12InfoChannelCard, "ord_hci", "chan");
            lvglRegisterReorderableItem(hc12InfoFuCard, "ord_hci", "fu");
            lvglRegisterReorderableItem(hc12InfoPowerCard, "ord_hci", "power");
            lvglRegisterReorderableItem(hc12InfoRawCard, "ord_hci", "raw");
            lvglApplySavedOrder(wrap, "ord_hci");
            applyRadioInfoLabels();
            break;
        }
        case UI_SCREENSAVER: {
            lvglScrScreensaver = lv_obj_create(nullptr);
            lv_obj_set_style_bg_color(lvglScrScreensaver, lv_color_hex(0x000000), 0);
            lv_obj_set_style_bg_opa(lvglScrScreensaver, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(lvglScrScreensaver, 0, 0);
            lv_obj_set_style_pad_all(lvglScrScreensaver, 0, 0);
            lv_obj_clear_flag(lvglScrScreensaver, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
            break;
        }
        case UI_CONFIG_MQTT_CONFIG: {
            lvglScrMqttCfg = lvglCreateScreenBase("MQTT Config", false);
            lvglMqttPassShowBtnLabel = nullptr;
            lvglMqttPassShowBtn = nullptr;
            lvglMqttStatusPanel = nullptr;
            lvglMqttCountLabel = nullptr;
            lvglMqttEditLabel = nullptr;
            lvglMqttBtnNameTa = nullptr;
            lvglMqttCriticalSw = nullptr;
            lv_obj_t *mqWrap = lv_obj_create(lvglScrMqttCfg);
            lv_obj_set_size(mqWrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(mqWrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_color(mqWrap, lv_color_hex(0x111922), 0);
            lv_obj_set_style_border_width(mqWrap, 0, 0);
            lv_obj_set_style_pad_all(mqWrap, 8, 0);
            lv_obj_clear_flag(mqWrap, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_scrollbar_mode(mqWrap, LV_SCROLLBAR_MODE_OFF);
            const lv_coord_t mqttStatusPanelH = 128;
            const lv_coord_t mqttLayoutGap = 6;
            lv_obj_t *formWrap = lv_obj_create(mqWrap);
            lv_obj_set_size(formWrap, lv_pct(100), UI_CONTENT_H - mqttStatusPanelH - mqttLayoutGap);
            lv_obj_align(formWrap, LV_ALIGN_TOP_MID, 0, 0);
            lv_obj_set_style_bg_opa(formWrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(formWrap, 0, 0);
            lv_obj_set_style_pad_all(formWrap, 0, 0);
            lv_obj_set_style_pad_row(formWrap, 6, 0);
            lv_obj_set_flex_flow(formWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scroll_dir(formWrap, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(formWrap, LV_SCROLLBAR_MODE_OFF);

            lv_obj_t *enableRow = lv_obj_create(formWrap);
            lv_obj_set_size(enableRow, lv_pct(100), 34);
            lv_obj_set_style_bg_opa(enableRow, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(enableRow, 0, 0);
            lv_obj_t *enableLbl = lv_label_create(enableRow);
            lv_label_set_text(enableLbl, "Enabled");
            lv_obj_align(enableLbl, LV_ALIGN_LEFT_MID, 0, 0);
            lvglMqttEnableSw = lv_switch_create(enableRow);
            lv_obj_align(lvglMqttEnableSw, LV_ALIGN_RIGHT_MID, 0, 0);
            lv_obj_add_event_cb(lvglMqttEnableSw, lvglMqttEnableEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            auto addTa = [&](const char *ph, bool pass, bool numeric) -> lv_obj_t * {
                lv_obj_t *ta = lv_textarea_create(formWrap);
                lv_obj_set_width(ta, lv_pct(100));
                lvglConfigureCompactSingleLineTa(ta, 34);
                lv_textarea_set_placeholder_text(ta, ph);
                if (pass) lvglConfigureSingleLinePasswordTa(ta);
                if (numeric) lv_textarea_set_accepted_chars(ta, "0123456789");
                lv_obj_add_event_cb(ta, lvglTextAreaFocusEvent, LV_EVENT_FOCUSED, nullptr);
                return ta;
            };
            lvglMqttBrokerTa = addTa("Broker", false, false);
            lvglMqttPortTa = addTa("Port", false, true);
            lvglMqttUserTa = addTa("Username", false, false);

            lv_obj_t *mqttPwdRow = lv_obj_create(formWrap);
            lv_obj_set_size(mqttPwdRow, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(mqttPwdRow, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(mqttPwdRow, 0, 0);
            lv_obj_set_style_pad_all(mqttPwdRow, 0, 0);
            lv_obj_set_style_pad_column(mqttPwdRow, 6, 0);
            lv_obj_set_flex_flow(mqttPwdRow, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(mqttPwdRow, LV_OBJ_FLAG_SCROLLABLE);

            lvglMqttPassTa = lv_textarea_create(mqttPwdRow);
            lv_obj_set_width(lvglMqttPassTa, lv_pct(100));
            lv_obj_set_height(lvglMqttPassTa, 28);
            lv_obj_set_flex_grow(lvglMqttPassTa, 1);
            lvglConfigureSingleLinePasswordTa(lvglMqttPassTa);
            lv_textarea_set_placeholder_text(lvglMqttPassTa, "Password");
            lv_obj_add_event_cb(lvglMqttPassTa, lvglTextAreaFocusEvent, LV_EVENT_FOCUSED, nullptr);

            lvglMqttPassShowBtn = lv_btn_create(mqttPwdRow);
            lv_obj_set_size(lvglMqttPassShowBtn, 28, 28);
            lv_obj_set_style_radius(lvglMqttPassShowBtn, 8, 0);
            lv_obj_set_style_border_width(lvglMqttPassShowBtn, 0, 0);
            lv_obj_add_event_cb(lvglMqttPassShowBtn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
            lv_obj_add_event_cb(lvglMqttPassShowBtn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
            lv_obj_add_event_cb(lvglMqttPassShowBtn, lvglMqttPassShowToggleEvent, LV_EVENT_CLICKED, nullptr);
            lvglMqttPassShowBtnLabel = lv_label_create(lvglMqttPassShowBtn);
            lv_label_set_text(lvglMqttPassShowBtnLabel, LV_SYMBOL_EYE_CLOSE);
            lv_obj_center(lvglMqttPassShowBtnLabel);
            lvglRegisterStyledButton(lvglMqttPassShowBtn, lv_color_hex(0x3F4A57), true);

            lvglMqttDiscTa = addTa("Discovery prefix", false, false);
            lv_obj_t *actRow = lv_obj_create(formWrap);
            lv_obj_set_size(actRow, lv_pct(100), 40);
            lv_obj_set_style_bg_opa(actRow, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(actRow, 0, 0);
            lv_obj_set_style_pad_all(actRow, 0, 0);
            lv_obj_set_style_pad_column(actRow, 6, 0);
            lv_obj_set_flex_flow(actRow, LV_FLEX_FLOW_ROW);
            lv_obj_set_flex_align(actRow, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
            lv_obj_clear_flag(actRow, LV_OBJ_FLAG_SCROLLABLE);
            makeSmallBtn(actRow, lvglSymbolText(LV_SYMBOL_SAVE, "Save").c_str(), 76, 30, lv_color_hex(0x3A7A3A), lvglMqttSaveEvent);
            makeSmallBtn(actRow, lvglSymbolText(LV_SYMBOL_POWER, "Connect").c_str(), 92, 30, lv_color_hex(0x2F6D86), lvglMqttConnectEvent);
            makeSmallBtn(actRow, lvglSymbolText(LV_SYMBOL_WIFI, "Discover").c_str(), 102, 30, lv_color_hex(0x2F6D86), lvglMqttPublishDiscoveryEvent);
            lvglRegisterReorderableItem(enableRow, "ord_mcfg", "enable");
            lvglRegisterReorderableItem(lvglMqttBrokerTa, "ord_mcfg", "broker");
            lvglRegisterReorderableItem(lvglMqttPortTa, "ord_mcfg", "port");
            lvglRegisterReorderableItem(lvglMqttUserTa, "ord_mcfg", "user");
            lvglRegisterReorderableItem(mqttPwdRow, "ord_mcfg", "pass");
            lvglRegisterReorderableItem(lvglMqttDiscTa, "ord_mcfg", "disc");
            lvglRegisterReorderableItem(actRow, "ord_mcfg", "act");
            lvglApplySavedOrder(formWrap, "ord_mcfg");

            lvglMqttStatusPanel = lv_obj_create(mqWrap);
            lv_obj_set_size(lvglMqttStatusPanel, lv_pct(100), mqttStatusPanelH);
            lv_obj_align(lvglMqttStatusPanel, LV_ALIGN_BOTTOM_MID, 0, 0);
            lv_obj_set_style_bg_color(lvglMqttStatusPanel, lv_color_hex(0x16212C), 0);
            lv_obj_set_style_border_width(lvglMqttStatusPanel, 0, 0);
            lv_obj_set_style_radius(lvglMqttStatusPanel, 12, 0);
            lv_obj_set_style_pad_all(lvglMqttStatusPanel, 8, 0);
            lv_obj_set_style_pad_row(lvglMqttStatusPanel, 6, 0);
            lv_obj_set_flex_flow(lvglMqttStatusPanel, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scroll_dir(lvglMqttStatusPanel, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(lvglMqttStatusPanel, LV_SCROLLBAR_MODE_OFF);
            lv_obj_clear_flag(lvglMqttStatusPanel, LV_OBJ_FLAG_CLICKABLE);

            lv_obj_t *statusHdr = lv_label_create(lvglMqttStatusPanel);
            lv_label_set_text(statusHdr, "Status");
            lv_obj_set_style_text_color(statusHdr, lv_color_hex(0xE5ECF3), 0);

            lvglMqttStatusLabel = lv_label_create(lvglMqttStatusPanel);
            lv_obj_set_width(lvglMqttStatusLabel, lv_pct(100));
            lv_label_set_long_mode(lvglMqttStatusLabel, LV_LABEL_LONG_WRAP);
            lv_obj_set_style_text_color(lvglMqttStatusLabel, lv_color_hex(0xC8D3DD), 0);
            lvglRefreshMqttConfigUi();
            break;
        }
        case UI_CONFIG_MQTT_CONTROLS: {
            lvglScrMqttCtrl = lvglCreateScreenBase("MQTT Controls", true);
            lv_obj_t *wrap = lv_obj_create(lvglScrMqttCtrl);
            lv_obj_set_size(wrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(wrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(wrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(wrap, 0, 0);
            lv_obj_set_style_pad_all(wrap, 10, 0);
            lv_obj_set_style_pad_row(wrap, 10, 0);
            lv_obj_set_flex_flow(wrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scroll_dir(wrap, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(wrap, LV_SCROLLBAR_MODE_OFF);

            lvglCreateMenuButton(wrap, tr(TXT_BUTTON_CONFIG), lv_color_hex(0x7A5C2E), lvglOpenMqttButtonConfigScreenEvent, nullptr);

            lvglMqttCtrlList = lv_obj_create(wrap);
            lv_obj_set_size(lvglMqttCtrlList, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(lvglMqttCtrlList, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(lvglMqttCtrlList, 0, 0);
            lv_obj_set_style_pad_all(lvglMqttCtrlList, 0, 0);
            lv_obj_set_style_pad_row(lvglMqttCtrlList, 10, 0);
            lv_obj_set_flex_flow(lvglMqttCtrlList, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(lvglMqttCtrlList, LV_OBJ_FLAG_SCROLLABLE);

            lvglRefreshMqttControlsUi();
            break;
        }
        case UI_CONFIG_MQTT_BUTTONS: {
            lvglScrMqttButtonConfig = lvglCreateScreenBase(tr(TXT_BUTTON_CONFIG), false);
            lv_obj_t *wrap = lv_obj_create(lvglScrMqttButtonConfig);
            lv_obj_set_size(wrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(wrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(wrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(wrap, 0, 0);
            lv_obj_set_style_pad_all(wrap, 10, 0);
            lv_obj_set_style_pad_row(wrap, 10, 0);
            lv_obj_set_flex_flow(wrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scrollbar_mode(wrap, LV_SCROLLBAR_MODE_OFF);
            lv_obj_clear_flag(wrap, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *topRow = lv_obj_create(wrap);
            lv_obj_set_size(topRow, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(topRow, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(topRow, 0, 0);
            lv_obj_set_style_pad_all(topRow, 0, 0);
            lv_obj_set_style_pad_column(topRow, 6, 0);
            lv_obj_set_flex_flow(topRow, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(topRow, LV_OBJ_FLAG_SCROLLABLE);

            makeSmallBtn(topRow, "+", 44, 30, lv_color_hex(0x2F7A45), lvglMqttButtonAddEvent, nullptr);
            makeSmallBtn(topRow, "Sel", 54, 30, lv_color_hex(0x2F6D86), lvglMqttButtonSelectModeEvent, nullptr);
            makeSmallBtn(topRow, "Del", 54, 30, lv_color_hex(0x8A3A3A), lvglMqttButtonDeletePromptEvent, nullptr);

            lv_obj_t *headerRow = lv_obj_create(wrap);
            lv_obj_set_size(headerRow, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(headerRow, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(headerRow, 0, 0);
            lv_obj_set_style_pad_all(headerRow, 0, 0);
            lv_obj_set_style_pad_column(headerRow, 6, 0);
            lv_obj_set_flex_flow(headerRow, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(headerRow, LV_OBJ_FLAG_SCROLLABLE);

            auto makeHeaderLabel = [&](const char *txt, lv_coord_t w) {
                lv_obj_t *lbl = lv_label_create(headerRow);
                lv_obj_set_width(lbl, w);
                lv_label_set_text(lbl, txt);
                lv_obj_set_style_text_color(lbl, lv_color_hex(0xAFC3D7), 0);
                lv_obj_set_style_text_font(lbl, LV_FONT_DEFAULT, 0);
            };

            makeHeaderLabel("", 24);
            makeHeaderLabel("Name", DISPLAY_WIDTH - 172);
            makeHeaderLabel("Conf", 44);
            makeHeaderLabel("Tgl", 44);

            lvglMqttButtonConfigList = lv_obj_create(wrap);
            lv_obj_set_size(lvglMqttButtonConfigList, lv_pct(100), UI_CONTENT_H - 90);
            lv_obj_set_style_bg_opa(lvglMqttButtonConfigList, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(lvglMqttButtonConfigList, 0, 0);
            lv_obj_set_style_pad_all(lvglMqttButtonConfigList, 0, 0);
            lv_obj_set_style_pad_row(lvglMqttButtonConfigList, 8, 0);
            lv_obj_set_flex_flow(lvglMqttButtonConfigList, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scroll_dir(lvglMqttButtonConfigList, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(lvglMqttButtonConfigList, LV_SCROLLBAR_MODE_OFF);

            lvglRefreshMqttButtonConfigUi();
            break;
        }
        case UI_GAME_SNAKE: {
            lvglScrSnake = lvglCreateScreenBase("Snake", false);
            const lv_coord_t snakeControlsHeight = 108;
            const lv_coord_t snakeBoardTop = UI_CONTENT_TOP_Y + 4;
            const lv_coord_t snakeBoardWidth = DISPLAY_WIDTH - 10;
            const lv_coord_t snakeBoardHeight = UI_CONTENT_H - snakeControlsHeight - 10;
            const lv_coord_t snakePadBtn = (DISPLAY_WIDTH >= 460) ? 58 : 42;
            const lv_coord_t snakeDirBtnWidth = snakePadBtn * 2;
            const lv_coord_t snakeDirBtnHeight = snakePadBtn - 2;
            const lv_coord_t snakeSideOffsetRequested = snakePadBtn + 28;
            const lv_coord_t snakePauseBtnWidth = snakeDirBtnWidth;
            const lv_coord_t snakePauseBtnHeight = snakeDirBtnHeight;
            const lv_coord_t snakeSideOffsetMinNoOverlap = ((snakePauseBtnWidth + snakeDirBtnWidth) / 2) + 4;
            const lv_coord_t snakeSideOffsetMaxFit = (DISPLAY_WIDTH - snakeDirBtnWidth) / 2 - 6;
            const lv_coord_t snakeSideOffset = min<lv_coord_t>(snakeSideOffsetMaxFit,
                                                                max<lv_coord_t>(snakeSideOffsetRequested,
                                                                                snakeSideOffsetMinNoOverlap));

            lvglSnakeBoardObj = lv_obj_create(lvglScrSnake);
            if (lvglSnakeBoardObj) {
                lv_obj_set_size(lvglSnakeBoardObj, snakeBoardWidth, snakeBoardHeight);
                lv_obj_align(lvglSnakeBoardObj, LV_ALIGN_TOP_MID, 0, snakeBoardTop);
                lv_obj_set_style_bg_color(lvglSnakeBoardObj, lv_color_hex(0x0C1218), 0);
                lv_obj_set_style_border_width(lvglSnakeBoardObj, 0, 0);
                lv_obj_set_style_radius(lvglSnakeBoardObj, 8, 0);
                lv_obj_clear_flag(lvglSnakeBoardObj, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_add_event_cb(lvglSnakeBoardObj, lvglSnakeBoardDrawEvent, LV_EVENT_DRAW_MAIN, nullptr);

                lvglSnakeScoreLabel = lv_label_create(lvglSnakeBoardObj);
                if (lvglSnakeScoreLabel) {
                    lv_obj_align(lvglSnakeScoreLabel, LV_ALIGN_TOP_LEFT, 10, 8);
                    lv_obj_set_style_text_color(lvglSnakeScoreLabel, lv_color_hex(0xEAF2F9), 0);
                }
                lvglSnakeBestLabel = lv_label_create(lvglSnakeBoardObj);
                if (lvglSnakeBestLabel) {
                    lv_obj_align(lvglSnakeBestLabel, LV_ALIGN_TOP_RIGHT, -10, 8);
                    lv_obj_set_style_text_color(lvglSnakeBestLabel, lv_color_hex(0xBBD2E6), 0);
                }

                lvglSnakeOverlay = lv_obj_create(lvglSnakeBoardObj);
                if (lvglSnakeOverlay) {
                    lv_obj_set_size(lvglSnakeOverlay, min<lv_coord_t>(snakeBoardWidth - 16, 324), min<lv_coord_t>(snakeBoardHeight - 16, 467));
                    lv_obj_center(lvglSnakeOverlay);
                    lv_obj_set_style_bg_color(lvglSnakeOverlay, lv_color_hex(0x12202B), 0);
                    lv_obj_set_style_bg_opa(lvglSnakeOverlay, LV_OPA_90, 0);
                    lv_obj_set_style_border_width(lvglSnakeOverlay, 1, 0);
                    lv_obj_set_style_border_color(lvglSnakeOverlay, lv_color_hex(0x3C566B), 0);
                    lv_obj_set_style_radius(lvglSnakeOverlay, 14, 0);
                    lv_obj_set_style_pad_all(lvglSnakeOverlay, 12, 0);
                    lv_obj_set_style_pad_row(lvglSnakeOverlay, 10, 0);
                    lv_obj_set_flex_flow(lvglSnakeOverlay, LV_FLEX_FLOW_COLUMN);
                    lv_obj_set_flex_align(lvglSnakeOverlay, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
                    lv_obj_clear_flag(lvglSnakeOverlay, LV_OBJ_FLAG_SCROLLABLE);
                    lv_obj_move_foreground(lvglSnakeOverlay);

                    lvglSnakeOverlayTitle = lv_label_create(lvglSnakeOverlay);
                    if (lvglSnakeOverlayTitle) lv_obj_set_style_text_color(lvglSnakeOverlayTitle, lv_color_hex(0xF2F7FB), 0);
                    lvglSnakeOverlaySubLabel = lv_label_create(lvglSnakeOverlay);
                    if (lvglSnakeOverlaySubLabel) {
                        lv_obj_set_width(lvglSnakeOverlaySubLabel, lv_pct(100));
                        lv_label_set_long_mode(lvglSnakeOverlaySubLabel, LV_LABEL_LONG_WRAP);
                        lv_obj_set_style_text_align(lvglSnakeOverlaySubLabel, LV_TEXT_ALIGN_CENTER, 0);
                        lv_obj_set_style_text_color(lvglSnakeOverlaySubLabel, lv_color_hex(0xB7C8D7), 0);
                    }
                    lvglSnakeOverlayBtn = makeSmallBtn(lvglSnakeOverlay, "Start", 120, 42, lv_color_hex(0x3A8F4B), lvglSnakeRestartEvent);
                    if (lvglSnakeOverlayBtn) lvglSnakeOverlayBtnLabel = lv_obj_get_child(lvglSnakeOverlayBtn, 0);
                }
            }

            lv_obj_t *snakeCtl = lv_obj_create(lvglScrSnake);
            lv_obj_set_size(snakeCtl, lv_pct(100), snakeControlsHeight);
            lv_obj_align(snakeCtl, LV_ALIGN_BOTTOM_MID, 0, 0);
            lv_obj_set_style_bg_color(snakeCtl, lv_color_hex(0x101922), 0);
            lv_obj_set_style_bg_opa(snakeCtl, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(snakeCtl, 0, 0);
            lv_obj_set_style_radius(snakeCtl, 0, 0);
            lv_obj_set_style_pad_all(snakeCtl, 0, 0);
            lv_obj_clear_flag(snakeCtl, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *upBtn = makeSmallBtn(snakeCtl, LV_SYMBOL_UP, snakeDirBtnWidth, snakeDirBtnHeight, lv_color_hex(0x2F6D86), lvglSnakeDirEvent, reinterpret_cast<void *>(static_cast<intptr_t>(0)));
            if (upBtn) lv_obj_align(upBtn, LV_ALIGN_TOP_MID, 0, 4);
            lv_obj_t *leftBtn = makeSmallBtn(snakeCtl, LV_SYMBOL_LEFT, snakeDirBtnWidth, snakeDirBtnHeight, lv_color_hex(0x2F6D86), lvglSnakeDirEvent, reinterpret_cast<void *>(static_cast<intptr_t>(3)));
            if (leftBtn) lv_obj_align(leftBtn, LV_ALIGN_CENTER, -snakeSideOffset, 6);
            lv_obj_t *pauseBtn = makeSmallBtn(snakeCtl, LV_SYMBOL_PLAY, snakePauseBtnWidth, snakePauseBtnHeight, lv_color_hex(0x3A8F4B), lvglSnakePauseEvent, nullptr);
            if (pauseBtn) {
                lv_obj_align(pauseBtn, LV_ALIGN_TOP_MID, -snakeSideOffset, 4);
                lvglSnakePauseBtnLabel = lv_obj_get_child(pauseBtn, 0);
            }
            lv_obj_t *rightBtn = makeSmallBtn(snakeCtl, LV_SYMBOL_RIGHT, snakeDirBtnWidth, snakeDirBtnHeight, lv_color_hex(0x2F6D86), lvglSnakeDirEvent, reinterpret_cast<void *>(static_cast<intptr_t>(1)));
            if (rightBtn) lv_obj_align(rightBtn, LV_ALIGN_CENTER, snakeSideOffset, 6);
            lv_obj_t *downBtn = makeSmallBtn(snakeCtl, LV_SYMBOL_DOWN, snakeDirBtnWidth, snakeDirBtnHeight, lv_color_hex(0x2F6D86), lvglSnakeDirEvent, reinterpret_cast<void *>(static_cast<intptr_t>(2)));
            if (downBtn) lv_obj_align(downBtn, LV_ALIGN_BOTTOM_MID, 0, -4);
            snakePrepareGame();
            lvglRefreshSnakeBoard();
            break;
        }
        case UI_GAME_TETRIS: {
            lvglScrTetris = lvglCreateScreenBase("Tetris", false);
            const lv_coord_t tetrisControlsHeight = 108;
            const lv_coord_t tetrisBoardTop = UI_CONTENT_TOP_Y + 4;
            const lv_coord_t tetrisBoardWidth = DISPLAY_WIDTH - 10;
            const lv_coord_t tetrisBoardHeight = UI_CONTENT_H - tetrisControlsHeight - 10;
            const lv_coord_t tetrisPadBtn = (DISPLAY_WIDTH >= 460) ? 58 : 42;
            const lv_coord_t tetrisDirBtnWidth = tetrisPadBtn * 2;
            const lv_coord_t tetrisDirBtnHeight = tetrisPadBtn - 2;
            const lv_coord_t tetrisPauseBtnWidth = tetrisDirBtnWidth;
            const lv_coord_t tetrisPauseBtnHeight = max<lv_coord_t>(18, tetrisDirBtnHeight - 5);
            const lv_coord_t tetrisSideOffsetRequested = tetrisPadBtn + 28;
            const lv_coord_t tetrisSideOffsetMinNoOverlap = ((tetrisPauseBtnWidth + tetrisDirBtnWidth) / 2) + 4;
            const lv_coord_t tetrisSideOffsetMaxFit = (DISPLAY_WIDTH - tetrisDirBtnWidth) / 2 - 6;
            const lv_coord_t tetrisSideOffset = min<lv_coord_t>(tetrisSideOffsetMaxFit,
                                                                 max<lv_coord_t>(tetrisSideOffsetRequested,
                                                                                 tetrisSideOffsetMinNoOverlap));

            lvglTetrisBoardObj = lv_obj_create(lvglScrTetris);
            if (lvglTetrisBoardObj) {
                lv_obj_set_size(lvglTetrisBoardObj, tetrisBoardWidth, tetrisBoardHeight);
                lv_obj_align(lvglTetrisBoardObj, LV_ALIGN_TOP_MID, 0, tetrisBoardTop);
                lv_obj_set_style_bg_color(lvglTetrisBoardObj, lv_color_hex(0x0C1218), 0);
                lv_obj_set_style_border_width(lvglTetrisBoardObj, 0, 0);
                lv_obj_set_style_radius(lvglTetrisBoardObj, 8, 0);
                lv_obj_clear_flag(lvglTetrisBoardObj, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_add_event_cb(lvglTetrisBoardObj, lvglTetrisBoardDrawEvent, LV_EVENT_DRAW_MAIN, nullptr);

                lvglTetrisScoreLabel = lv_label_create(lvglTetrisBoardObj);
                if (lvglTetrisScoreLabel) {
                    lv_obj_align(lvglTetrisScoreLabel, LV_ALIGN_TOP_LEFT, 10, 8);
                    lv_obj_set_style_text_color(lvglTetrisScoreLabel, lv_color_hex(0xEAF2F9), 0);
                }
                lvglTetrisBestLabel = lv_label_create(lvglTetrisBoardObj);
                if (lvglTetrisBestLabel) {
                    lv_obj_align(lvglTetrisBestLabel, LV_ALIGN_TOP_RIGHT, -10, 8);
                    lv_obj_set_style_text_color(lvglTetrisBestLabel, lv_color_hex(0xBBD2E6), 0);
                }

                lvglTetrisOverlay = lv_obj_create(lvglScrTetris);
                if (lvglTetrisOverlay) {
                    lv_obj_set_size(lvglTetrisOverlay, min<lv_coord_t>(tetrisBoardWidth - 16, 324), min<lv_coord_t>(tetrisBoardHeight - 16, 467));
                    lv_obj_center(lvglTetrisOverlay);
                    lv_obj_set_style_bg_color(lvglTetrisOverlay, lv_color_hex(0x12202B), 0);
                    lv_obj_set_style_bg_opa(lvglTetrisOverlay, LV_OPA_90, 0);
                    lv_obj_set_style_border_width(lvglTetrisOverlay, 1, 0);
                    lv_obj_set_style_border_color(lvglTetrisOverlay, lv_color_hex(0x3C566B), 0);
                    lv_obj_set_style_radius(lvglTetrisOverlay, 14, 0);
                    lv_obj_set_style_pad_all(lvglTetrisOverlay, 12, 0);
                    lv_obj_set_style_pad_row(lvglTetrisOverlay, 10, 0);
                    lv_obj_set_flex_flow(lvglTetrisOverlay, LV_FLEX_FLOW_COLUMN);
                    lv_obj_set_flex_align(lvglTetrisOverlay, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
                    lv_obj_clear_flag(lvglTetrisOverlay, LV_OBJ_FLAG_SCROLLABLE);
                    lv_obj_move_foreground(lvglTetrisOverlay);

                    lvglTetrisOverlayTitle = lv_label_create(lvglTetrisOverlay);
                    if (lvglTetrisOverlayTitle) lv_obj_set_style_text_color(lvglTetrisOverlayTitle, lv_color_hex(0xF2F7FB), 0);
                    lvglTetrisOverlaySubLabel = lv_label_create(lvglTetrisOverlay);
                    if (lvglTetrisOverlaySubLabel) {
                        lv_obj_set_width(lvglTetrisOverlaySubLabel, lv_pct(100));
                        lv_label_set_long_mode(lvglTetrisOverlaySubLabel, LV_LABEL_LONG_WRAP);
                        lv_obj_set_style_text_align(lvglTetrisOverlaySubLabel, LV_TEXT_ALIGN_CENTER, 0);
                        lv_obj_set_style_text_color(lvglTetrisOverlaySubLabel, lv_color_hex(0xB7C8D7), 0);
                    }
                    lvglTetrisOverlayBtn = makeSmallBtn(lvglTetrisOverlay, "Start", 120, 42, lv_color_hex(0x376B93), lvglTetrisRestartEvent);
                    if (lvglTetrisOverlayBtn) lvglTetrisOverlayBtnLabel = lv_obj_get_child(lvglTetrisOverlayBtn, 0);
                }
            }

            lv_obj_t *tCtl = lv_obj_create(lvglScrTetris);
            lv_obj_set_size(tCtl, lv_pct(100), tetrisControlsHeight);
            lv_obj_align(tCtl, LV_ALIGN_BOTTOM_MID, 0, 0);
            lv_obj_set_style_bg_color(tCtl, lv_color_hex(0x101922), 0);
            lv_obj_set_style_bg_opa(tCtl, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(tCtl, 0, 0);
            lv_obj_set_style_radius(tCtl, 0, 0);
            lv_obj_set_style_pad_all(tCtl, 0, 0);
            lv_obj_clear_flag(tCtl, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_t *upBtn = makeSmallBtn(tCtl, LV_SYMBOL_UP, tetrisDirBtnWidth, tetrisDirBtnHeight, lv_color_hex(0x376B93), lvglTetrisRotateEvent);
            if (upBtn) lv_obj_align(upBtn, LV_ALIGN_TOP_MID, 0, 4);
            lv_obj_t *leftBtn = makeSmallBtn(tCtl, LV_SYMBOL_LEFT, tetrisDirBtnWidth, tetrisDirBtnHeight, lv_color_hex(0x2F6D86), lvglTetrisMoveLeftEvent);
            if (leftBtn) lv_obj_align(leftBtn, LV_ALIGN_CENTER, -tetrisSideOffset, 6);
            lv_obj_t *pauseBtn = makeSmallBtn(tCtl, LV_SYMBOL_PLAY, tetrisPauseBtnWidth, tetrisPauseBtnHeight, lv_color_hex(0x376B93), lvglTetrisPauseEvent, nullptr);
            if (pauseBtn) {
                lv_obj_align(pauseBtn, LV_ALIGN_TOP_MID, -tetrisSideOffset, 4);
                lvglTetrisPauseBtnLabel = lv_obj_get_child(pauseBtn, 0);
            }
            lv_obj_t *rightBtn = makeSmallBtn(tCtl, LV_SYMBOL_RIGHT, tetrisDirBtnWidth, tetrisDirBtnHeight, lv_color_hex(0x2F6D86), lvglTetrisMoveRightEvent);
            if (rightBtn) lv_obj_align(rightBtn, LV_ALIGN_CENTER, tetrisSideOffset, 6);
            lv_obj_t *downBtn = makeSmallBtn(tCtl, LV_SYMBOL_DOWN, tetrisDirBtnWidth, tetrisDirBtnHeight, lv_color_hex(0x8A5A25), lvglTetrisDropEvent);
            if (downBtn) lv_obj_align(downBtn, LV_ALIGN_BOTTOM_MID, 0, -4);
            tetrisPrepareGame();
            lvglRefreshTetrisBoard();
            break;
        }
        case UI_GAME_CHECKERS: {
            lvglScrCheckers = lvglCreateScreenBase("Checkers", false);
            const lv_coord_t checkersBoardTop = UI_CONTENT_TOP_Y + 4;
            const lv_coord_t checkersBoardWidth = DISPLAY_WIDTH - 10;
            const lv_coord_t checkersBoardHeight = UI_CONTENT_H - CHECKERS_ACTION_BAR_H - 10;

            lvglCheckersBoardObj = lv_obj_create(lvglScrCheckers);
            if (lvglCheckersBoardObj) {
                lv_obj_set_size(lvglCheckersBoardObj, checkersBoardWidth, checkersBoardHeight);
                lv_obj_align(lvglCheckersBoardObj, LV_ALIGN_TOP_MID, 0, checkersBoardTop);
                lv_obj_set_style_bg_color(lvglCheckersBoardObj, lv_color_hex(0x0C1218), 0);
                lv_obj_set_style_border_width(lvglCheckersBoardObj, 0, 0);
                lv_obj_set_style_radius(lvglCheckersBoardObj, 8, 0);
                lv_obj_clear_flag(lvglCheckersBoardObj, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_add_event_cb(lvglCheckersBoardObj, lvglCheckersBoardDrawEvent, LV_EVENT_DRAW_MAIN, nullptr);
                lv_obj_add_event_cb(lvglCheckersBoardObj, lvglCheckersBoardEvent, LV_EVENT_CLICKED, nullptr);

                lvglCheckersModeLabel = lv_label_create(lvglCheckersBoardObj);
                if (lvglCheckersModeLabel) {
                    lv_obj_align(lvglCheckersModeLabel, LV_ALIGN_TOP_LEFT, 10, 8);
                    lv_obj_set_style_text_color(lvglCheckersModeLabel, lv_color_hex(0xEAF2F9), 0);
                }
                lvglCheckersTurnLabel = lv_label_create(lvglCheckersBoardObj);
                if (lvglCheckersTurnLabel) {
                    lv_obj_align(lvglCheckersTurnLabel, LV_ALIGN_TOP_RIGHT, -10, 8);
                    lv_obj_set_style_text_color(lvglCheckersTurnLabel, lv_color_hex(0xBBD2E6), 0);
                }
            }

            lvglCheckersOverlay = lv_obj_create(lvglScrCheckers);
            if (lvglCheckersOverlay) {
                lv_obj_set_size(lvglCheckersOverlay, min<lv_coord_t>(DISPLAY_WIDTH - 40, 190), min<lv_coord_t>(UI_CONTENT_H - 28, 150));
                lv_obj_align(lvglCheckersOverlay, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y + 10);
                lv_obj_set_style_bg_color(lvglCheckersOverlay, lv_color_hex(0x12202B), 0);
                lv_obj_set_style_bg_opa(lvglCheckersOverlay, LV_OPA_90, 0);
                lv_obj_set_style_border_width(lvglCheckersOverlay, 1, 0);
                lv_obj_set_style_border_color(lvglCheckersOverlay, lv_color_hex(0x3C566B), 0);
                lv_obj_set_style_radius(lvglCheckersOverlay, 14, 0);
                lv_obj_set_style_pad_all(lvglCheckersOverlay, 12, 0);
                lv_obj_set_style_pad_row(lvglCheckersOverlay, 10, 0);
                lv_obj_set_flex_flow(lvglCheckersOverlay, LV_FLEX_FLOW_COLUMN);
                lv_obj_set_flex_align(lvglCheckersOverlay, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
                lv_obj_clear_flag(lvglCheckersOverlay, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_move_foreground(lvglCheckersOverlay);

                lvglCheckersOverlayTitle = lv_label_create(lvglCheckersOverlay);
                if (lvglCheckersOverlayTitle) lv_obj_set_style_text_color(lvglCheckersOverlayTitle, lv_color_hex(0xF2F7FB), 0);
                lvglCheckersOverlaySubLabel = lv_label_create(lvglCheckersOverlay);
                if (lvglCheckersOverlaySubLabel) {
                    lv_obj_set_width(lvglCheckersOverlaySubLabel, lv_pct(100));
                    lv_label_set_long_mode(lvglCheckersOverlaySubLabel, LV_LABEL_LONG_WRAP);
                    lv_obj_set_style_text_align(lvglCheckersOverlaySubLabel, LV_TEXT_ALIGN_CENTER, 0);
                    lv_obj_set_style_text_color(lvglCheckersOverlaySubLabel, lv_color_hex(0xB7C8D7), 0);
                }
                lvglCheckersOverlayBtn = makeSmallBtn(lvglCheckersOverlay, "Replay", 120, 42, lv_color_hex(0x8A5A25), lvglCheckersReplayEvent);
                if (lvglCheckersOverlayBtn) lvglCheckersOverlayBtnLabel = lv_obj_get_child(lvglCheckersOverlayBtn, 0);
            }

            lvglCheckersPeerPopup = lv_obj_create(lvglScrCheckers);
            if (lvglCheckersPeerPopup) {
                lv_obj_set_size(lvglCheckersPeerPopup, DISPLAY_WIDTH - 28, min<lv_coord_t>(UI_CONTENT_H - 40, 180));
                lv_obj_align(lvglCheckersPeerPopup, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y + 8);
                lv_obj_set_style_bg_color(lvglCheckersPeerPopup, lv_color_hex(0x16212C), 0);
                lv_obj_set_style_border_color(lvglCheckersPeerPopup, lv_color_hex(0x536274), 0);
                lv_obj_set_style_border_width(lvglCheckersPeerPopup, 1, 0);
                lv_obj_set_style_radius(lvglCheckersPeerPopup, 12, 0);
                lv_obj_set_style_pad_all(lvglCheckersPeerPopup, 8, 0);
                lv_obj_set_style_pad_row(lvglCheckersPeerPopup, 6, 0);
                lv_obj_set_flex_flow(lvglCheckersPeerPopup, LV_FLEX_FLOW_COLUMN);
                lv_obj_clear_flag(lvglCheckersPeerPopup, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_add_flag(lvglCheckersPeerPopup, LV_OBJ_FLAG_HIDDEN);

                lvglCheckersPeerPopupTitle = lv_label_create(lvglCheckersPeerPopup);
                if (lvglCheckersPeerPopupTitle) {
                    lv_label_set_text(lvglCheckersPeerPopupTitle, "Choose Contact");
                    lv_obj_set_style_text_color(lvglCheckersPeerPopupTitle, lv_color_hex(0xEAF2F9), 0);
                }

                lvglCheckersPeerPopupList = lv_obj_create(lvglCheckersPeerPopup);
                if (lvglCheckersPeerPopupList) {
                    lv_obj_set_size(lvglCheckersPeerPopupList, lv_pct(100), min<lv_coord_t>(110, UI_CONTENT_H - 90));
                    lv_obj_set_style_bg_opa(lvglCheckersPeerPopupList, LV_OPA_TRANSP, 0);
                    lv_obj_set_style_border_width(lvglCheckersPeerPopupList, 0, 0);
                    lv_obj_set_style_pad_all(lvglCheckersPeerPopupList, 0, 0);
                    lv_obj_set_style_pad_row(lvglCheckersPeerPopupList, 6, 0);
                    lv_obj_set_flex_flow(lvglCheckersPeerPopupList, LV_FLEX_FLOW_COLUMN);
                    lv_obj_set_scroll_dir(lvglCheckersPeerPopupList, LV_DIR_VER);
                    lv_obj_set_scrollbar_mode(lvglCheckersPeerPopupList, LV_SCROLLBAR_MODE_OFF);
                }
            }

            lvglCheckersVariantPopup = lv_obj_create(lvglScrCheckers);
            if (lvglCheckersVariantPopup) {
                lv_obj_set_size(lvglCheckersVariantPopup, DISPLAY_WIDTH - 28, min<lv_coord_t>(UI_CONTENT_H - 40, 220));
                lv_obj_align(lvglCheckersVariantPopup, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y + 8);
                lv_obj_set_style_bg_color(lvglCheckersVariantPopup, lv_color_hex(0x16212C), 0);
                lv_obj_set_style_border_color(lvglCheckersVariantPopup, lv_color_hex(0x536274), 0);
                lv_obj_set_style_border_width(lvglCheckersVariantPopup, 1, 0);
                lv_obj_set_style_radius(lvglCheckersVariantPopup, 12, 0);
                lv_obj_set_style_pad_all(lvglCheckersVariantPopup, 8, 0);
                lv_obj_set_style_pad_row(lvglCheckersVariantPopup, 6, 0);
                lv_obj_set_flex_flow(lvglCheckersVariantPopup, LV_FLEX_FLOW_COLUMN);
                lv_obj_clear_flag(lvglCheckersVariantPopup, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_add_flag(lvglCheckersVariantPopup, LV_OBJ_FLAG_HIDDEN);

                lvglCheckersVariantPopupTitle = lv_label_create(lvglCheckersVariantPopup);
                if (lvglCheckersVariantPopupTitle) {
                    lv_label_set_text(lvglCheckersVariantPopupTitle, "Choose Rule Set");
                    lv_obj_set_style_text_color(lvglCheckersVariantPopupTitle, lv_color_hex(0xEAF2F9), 0);
                }

                lvglCheckersVariantPopupList = lv_obj_create(lvglCheckersVariantPopup);
                if (lvglCheckersVariantPopupList) {
                    lv_obj_set_size(lvglCheckersVariantPopupList, lv_pct(100), min<lv_coord_t>(148, UI_CONTENT_H - 110));
                    lv_obj_set_style_bg_opa(lvglCheckersVariantPopupList, LV_OPA_TRANSP, 0);
                    lv_obj_set_style_border_width(lvglCheckersVariantPopupList, 0, 0);
                    lv_obj_set_style_pad_all(lvglCheckersVariantPopupList, 0, 0);
                    lv_obj_set_style_pad_row(lvglCheckersVariantPopupList, 6, 0);
                    lv_obj_set_flex_flow(lvglCheckersVariantPopupList, LV_FLEX_FLOW_COLUMN);
                    lv_obj_set_scroll_dir(lvglCheckersVariantPopupList, LV_DIR_VER);
                    lv_obj_set_scrollbar_mode(lvglCheckersVariantPopupList, LV_SCROLLBAR_MODE_OFF);
                }
            }

            lv_obj_t *checkersBar = lv_obj_create(lvglScrCheckers);
            lv_obj_set_size(checkersBar, lv_pct(100), CHECKERS_ACTION_BAR_H);
            lv_obj_align(checkersBar, LV_ALIGN_BOTTOM_MID, 0, 0);
            lv_obj_set_style_bg_color(checkersBar, lv_color_hex(0x101922), 0);
            lv_obj_set_style_bg_opa(checkersBar, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(checkersBar, 0, 0);
            lv_obj_set_style_radius(checkersBar, 0, 0);
            lv_obj_set_style_pad_all(checkersBar, 6, 0);
            lv_obj_set_style_pad_column(checkersBar, 6, 0);
            lv_obj_set_flex_flow(checkersBar, LV_FLEX_FLOW_ROW);
            lv_obj_set_flex_align(checkersBar, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
            lv_obj_clear_flag(checkersBar, LV_OBJ_FLAG_SCROLLABLE);
            const lv_coord_t ctlW = max<lv_coord_t>(66, (DISPLAY_WIDTH - 28) / 3);
            makeSmallBtn(checkersBar, "ESP32", ctlW, 32, lv_color_hex(0x8A5A25), lvglCheckersEsp32Event, nullptr);
            makeSmallBtn(checkersBar, "Tag MP", ctlW, 32, lv_color_hex(0x2F6D86), lvglCheckersTagEvent, nullptr);
            makeSmallBtn(checkersBar, "Back", ctlW, 32, lv_color_hex(0x4E5D6C), lvglCheckersBackEvent, nullptr);

            checkersMode = CHECKERS_MODE_IDLE;
            checkersStarted = false;
            checkersGameOver = false;
            checkersPeerPopupOpen = false;
            checkersVariantPopupOpen = true;
            checkersWinnerSide = 0;
            checkersClearSelection();
            checkersClearSession();
            lvglRefreshCheckersBoard();
            break;
        }
#if defined(BOARD_ESP32S3_3248S035_N16R8)
        case UI_GAME_SNAKE3D: {
            lvglScrSnake3d = lvglCreateScreenBase("Snake 3D", false);
            const lv_coord_t snake3dControlsHeight = 138;
            const lv_coord_t snake3dBoardTop = UI_CONTENT_TOP_Y + 4;
            const lv_coord_t snake3dBoardWidth = DISPLAY_WIDTH - 10;
            const lv_coord_t snake3dBoardHeight = UI_CONTENT_H - snake3dControlsHeight - 10;
            const lv_coord_t snake3dPadBtn = (DISPLAY_WIDTH >= 460) ? 54 : 40;

            lvglSnake3dBoardObj = lv_obj_create(lvglScrSnake3d);
            if (lvglSnake3dBoardObj) {
                lv_obj_set_size(lvglSnake3dBoardObj, snake3dBoardWidth, snake3dBoardHeight);
                lv_obj_align(lvglSnake3dBoardObj, LV_ALIGN_TOP_MID, 0, snake3dBoardTop);
                lv_obj_set_style_bg_color(lvglSnake3dBoardObj, lv_color_hex(0x07111A), 0);
                lv_obj_set_style_border_width(lvglSnake3dBoardObj, 0, 0);
                lv_obj_set_style_radius(lvglSnake3dBoardObj, 8, 0);
                lv_obj_clear_flag(lvglSnake3dBoardObj, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_add_event_cb(lvglSnake3dBoardObj, lvglSnake3dBoardDrawEvent, LV_EVENT_DRAW_MAIN, nullptr);

                lvglSnake3dScoreLabel = lv_label_create(lvglSnake3dBoardObj);
                if (lvglSnake3dScoreLabel) {
                    lv_obj_align(lvglSnake3dScoreLabel, LV_ALIGN_TOP_LEFT, 10, 8);
                    lv_obj_set_style_text_color(lvglSnake3dScoreLabel, lv_color_hex(0xEAF2F9), 0);
                }
                lvglSnake3dBestLabel = lv_label_create(lvglSnake3dBoardObj);
                if (lvglSnake3dBestLabel) {
                    lv_obj_align(lvglSnake3dBestLabel, LV_ALIGN_TOP_RIGHT, -10, 8);
                    lv_obj_set_style_text_color(lvglSnake3dBestLabel, lv_color_hex(0xBBD2E6), 0);
                }

                lvglSnake3dOverlay = lv_obj_create(lvglSnake3dBoardObj);
                if (lvglSnake3dOverlay) {
                    lv_obj_set_size(lvglSnake3dOverlay, min<lv_coord_t>(snake3dBoardWidth - 16, 250), min<lv_coord_t>(snake3dBoardHeight - 16, 255));
                    lv_obj_center(lvglSnake3dOverlay);
                    lv_obj_set_style_bg_color(lvglSnake3dOverlay, lv_color_hex(0x102030), 0);
                    lv_obj_set_style_bg_opa(lvglSnake3dOverlay, LV_OPA_90, 0);
                    lv_obj_set_style_border_width(lvglSnake3dOverlay, 1, 0);
                    lv_obj_set_style_border_color(lvglSnake3dOverlay, lv_color_hex(0x48627A), 0);
                    lv_obj_set_style_radius(lvglSnake3dOverlay, 14, 0);
                    lv_obj_set_style_pad_all(lvglSnake3dOverlay, 12, 0);
                    lv_obj_set_style_pad_row(lvglSnake3dOverlay, 10, 0);
                    lv_obj_set_flex_flow(lvglSnake3dOverlay, LV_FLEX_FLOW_COLUMN);
                    lv_obj_set_flex_align(lvglSnake3dOverlay, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
                    lv_obj_clear_flag(lvglSnake3dOverlay, LV_OBJ_FLAG_SCROLLABLE);

                    lvglSnake3dOverlayTitle = lv_label_create(lvglSnake3dOverlay);
                    if (lvglSnake3dOverlayTitle) lv_obj_set_style_text_color(lvglSnake3dOverlayTitle, lv_color_hex(0xF4F7FB), 0);
                    lvglSnake3dOverlaySubLabel = lv_label_create(lvglSnake3dOverlay);
                    if (lvglSnake3dOverlaySubLabel) {
                        lv_obj_set_width(lvglSnake3dOverlaySubLabel, lv_pct(100));
                        lv_label_set_long_mode(lvglSnake3dOverlaySubLabel, LV_LABEL_LONG_WRAP);
                        lv_obj_set_style_text_align(lvglSnake3dOverlaySubLabel, LV_TEXT_ALIGN_CENTER, 0);
                        lv_obj_set_style_text_color(lvglSnake3dOverlaySubLabel, lv_color_hex(0xB7C8D7), 0);
                    }
                    lvglSnake3dOverlayBtn = makeSmallBtn(lvglSnake3dOverlay, "Start", 128, 42, lv_color_hex(0x5A4CC7), lvglSnake3dRestartEvent);
                    if (lvglSnake3dOverlayBtn) lvglSnake3dOverlayBtnLabel = lv_obj_get_child(lvglSnake3dOverlayBtn, 0);
                }
            }

            lv_obj_t *snake3dCtl = lv_obj_create(lvglScrSnake3d);
            lv_obj_set_size(snake3dCtl, lv_pct(100), snake3dControlsHeight);
            lv_obj_align(snake3dCtl, LV_ALIGN_BOTTOM_MID, 0, 0);
            lv_obj_set_style_bg_color(snake3dCtl, lv_color_hex(0x101922), 0);
            lv_obj_set_style_bg_opa(snake3dCtl, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(snake3dCtl, 0, 0);
            lv_obj_set_style_radius(snake3dCtl, 0, 0);
            lv_obj_set_style_pad_all(snake3dCtl, 0, 0);
            lv_obj_clear_flag(snake3dCtl, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *upBtn = makeSmallBtn(snake3dCtl, LV_SYMBOL_UP, snake3dPadBtn * 2, snake3dPadBtn - 2, lv_color_hex(0x2F6D86), lvglSnake3dDirEvent, reinterpret_cast<void *>(static_cast<intptr_t>(0)));
            if (upBtn) lv_obj_align(upBtn, LV_ALIGN_TOP_MID, 0, 4);
            lv_obj_t *leftBtn = makeSmallBtn(snake3dCtl, LV_SYMBOL_LEFT, snake3dPadBtn * 2, snake3dPadBtn - 2, lv_color_hex(0x2F6D86), lvglSnake3dDirEvent, reinterpret_cast<void *>(static_cast<intptr_t>(3)));
            if (leftBtn) lv_obj_align(leftBtn, LV_ALIGN_CENTER, -(snake3dPadBtn + 34), 8);
            lv_obj_t *pauseBtn = makeSmallBtn(snake3dCtl, LV_SYMBOL_PLAY, snake3dPadBtn * 2, snake3dPadBtn - 2, lv_color_hex(0x5A4CC7), lvglSnake3dPauseEvent, nullptr);
            if (pauseBtn) {
                lv_obj_align(pauseBtn, LV_ALIGN_CENTER, 0, 8);
                lvglSnake3dPauseBtnLabel = lv_obj_get_child(pauseBtn, 0);
            }
            lv_obj_t *rightBtn = makeSmallBtn(snake3dCtl, LV_SYMBOL_RIGHT, snake3dPadBtn * 2, snake3dPadBtn - 2, lv_color_hex(0x2F6D86), lvglSnake3dDirEvent, reinterpret_cast<void *>(static_cast<intptr_t>(1)));
            if (rightBtn) lv_obj_align(rightBtn, LV_ALIGN_CENTER, snake3dPadBtn + 34, 8);
            lv_obj_t *downBtn = makeSmallBtn(snake3dCtl, LV_SYMBOL_DOWN, snake3dPadBtn * 2, snake3dPadBtn - 2, lv_color_hex(0x2F6D86), lvglSnake3dDirEvent, reinterpret_cast<void *>(static_cast<intptr_t>(2)));
            if (downBtn) lv_obj_align(downBtn, LV_ALIGN_BOTTOM_MID, 0, -6);
            lv_obj_t *ascendBtn = makeSmallBtn(snake3dCtl, "Z+", snake3dPadBtn * 2, 28, lv_color_hex(0x355C9A), lvglSnake3dDirEvent, reinterpret_cast<void *>(static_cast<intptr_t>(4)));
            if (ascendBtn) lv_obj_align(ascendBtn, LV_ALIGN_TOP_LEFT, 8, 8);
            lv_obj_t *descendBtn = makeSmallBtn(snake3dCtl, "Z-", snake3dPadBtn * 2, 28, lv_color_hex(0x6C4CA0), lvglSnake3dDirEvent, reinterpret_cast<void *>(static_cast<intptr_t>(5)));
            if (descendBtn) lv_obj_align(descendBtn, LV_ALIGN_TOP_RIGHT, -8, 8);

            snake3dPrepareGame();
            lvglRefreshSnake3dBoard();
            break;
        }
#endif
        default:
            break;
    }
}

void lvglOpenScreen(UiScreen screen, lv_scr_load_anim_t anim)
{
    if (screenLockShouldGateUi() && screen != UI_LOCK) {
        screen = UI_LOCK;
        anim = LV_SCR_LOAD_ANIM_NONE;
    }
    if (!moduleInstalledForScreen(screen)) {
        uiStatusLine = "Install this module first";
        if (lvglReady) lvglSyncStatusLine();
        return;
    }
    const UiScreen prevScreen = uiScreen;
    lvglEnsureScreenBuilt(screen);
    lv_obj_t *target = lvglScreenForUi(screen);
    if (!target) return;
    if (prevScreen != screen) lvglPrepareScreenLeave(prevScreen);
    uiScreen = screen;
    if (prevScreen == UI_CHAT && screen != UI_CHAT) chatClearCache();
    if (screen == UI_LOCK) {
        lvglRefreshLockScreenUi();
    } else if (screen == UI_WIFI_LIST) {
        lvglRefreshWifiList();
    } else if (screen == UI_CHAT) {
        if (currentChatPeerKey.isEmpty()) chatClearCache();
        lvglRefreshChatLayout();
        lvglRefreshChatContactsUi();
        lvglRefreshChatUi();
    } else if (screen == UI_CHAT_PEERS) {
        lvglRefreshChatPeerUi();
    } else if (screen == UI_MEDIA) {
        lvglQueueMediaRefresh();
    } else if (screen == UI_INFO) {
        queueInfoRefresh(true);
        lvglRefreshInfoPanel();        
    } else if (screen == UI_RADIO_CONTROL) {
        lvglRefreshRadioControlUi();
    } else if (screen == UI_CONFIG) {
        lvglRefreshConfigUi();
    } else if (screen == UI_CONFIG_REMOTE_CONTROL) {
        lvglRefreshRemoteControlUi();
    } else if (screen == UI_CONFIG_RADIO_BUTTONS) {
        lvglRefreshRadioButtonConfigUi();
    } else if (screen == UI_CONFIG_RADIO_RECEIVER_TEST) {
        lvglRefreshRadioReceiverTestUi();
    } else if (screen == UI_CONFIG_BATTERY) {
        lvglRefreshBatteryTrainUi();
    } else if (screen == UI_CONFIG_STYLE) {
        lvglRefreshStyleUi();
    } else if (screen == UI_CONFIG_STYLE_HOME_ITEMS) {
        lvglRefreshStyleHomeItemsUi();
    } else if (screen == UI_CONFIG_LANGUAGE) {
        lvglRefreshLanguageUi();
    } else if (screen == UI_CONFIG_OTA) {
        lvglRefreshOtaUi();
    } else if (screen == UI_CONFIG_MODULES) {
        moduleStartCatalogRefresh();
    } else if (screen == UI_CONFIG_HC12) {
        hc12ReadConfigSelection();
        lvglRefreshHc12ConfigUi();
    } else if (screen == UI_CONFIG_HC12_TERMINAL) {
        lvglRefreshHc12Ui();
    } else if (screen == UI_CONFIG_HC12_INFO) {
        queueRadioInfoFetch();
        lvglRefreshHc12InfoUi();
    }
    lvglRefreshTopIndicators();
    lvglLoadScreen(target, anim);
}

static void lvglNavigateBackFromScreen(UiScreen sourceScreen, lv_scr_load_anim_t anim)
{
    uiScreen = sourceScreen;
    const UiScreen prev = sourceScreen;
    if (!lvglReady) return;
    switch (sourceScreen) {
        case UI_CHAT:
            if (!currentChatPeerKey.isEmpty()) {
                lvglHideChatMenu();
                currentChatPeerKey = "";
                chatClearCache();
                lvglRefreshChatLayout();
                lvglRefreshChatContactsUi();
                lvglRefreshChatUi();
            } else {
                lvglOpenScreen(UI_HOME, anim);
            }
            break;
        case UI_MEDIA:
        case UI_INFO:
        case UI_GAMES:
        case UI_RADIO_CONTROL:
        case UI_CONFIG:
        case UI_LOCK:
            lvglOpenScreen(UI_HOME, anim);
            break;
        case UI_CHAT_PEERS:
            lvglOpenScreen(UI_CHAT, anim);
            break;
        case UI_WIFI_LIST:
        case UI_CONFIG_REMOTE_CONTROL:
        case UI_CONFIG_BATTERY:
            lvglOpenScreen(UI_CONFIG, anim);
            break;
        case UI_CONFIG_RADIO_BUTTONS:
            lvglOpenScreen(UI_RADIO_CONTROL, anim);
            break;
        case UI_CONFIG_RADIO_RECEIVER_TEST:
            lvglOpenScreen(UI_CONFIG_RADIO_BUTTONS, anim);
            break;
        case UI_CONFIG_STYLE:
        case UI_CONFIG_LANGUAGE:
        case UI_CONFIG_MODULES:
            lvglOpenScreen(UI_CONFIG, anim);
            break;
        case UI_CONFIG_STYLE_HOME_ITEMS:
            lvglOpenScreen(UI_CONFIG_STYLE, anim);
            break;
        case UI_CONFIG_OTA:
        case UI_CONFIG_HC12:
            lvglOpenScreen(UI_CONFIG, anim);
            break;
        case UI_CONFIG_HC12_TERMINAL: {
            lvglHideKeyboard();

            lv_obj_t *terminalTa = hc12TerminalObj();
            lv_obj_t *cmdTa = hc12CmdTaObj();

            if (terminalTa && lv_obj_is_valid(terminalTa)) {
                lv_anim_del(terminalTa, nullptr);
                lv_obj_t *wrap = lv_obj_get_parent(terminalTa);
                if (wrap && lv_obj_is_valid(wrap)) {
                    lv_anim_del(wrap, nullptr);
                }
            }

            if (cmdTa && lv_obj_is_valid(cmdTa)) {
                lv_anim_del(cmdTa, nullptr);
            }

            lvglOpenScreen(UI_CONFIG_HC12, anim);
            break;
        }

        case UI_CONFIG_HC12_INFO:
            lvglOpenScreen(UI_CONFIG_HC12, anim);
            break;
        case UI_GAME_SNAKE:
        case UI_GAME_TETRIS:
        case UI_GAME_CHECKERS:
        case UI_GAME_SNAKE3D:
            lvglOpenScreen(UI_GAMES, anim);
            break;
        case UI_CONFIG_MQTT_CONFIG:
            lvglOpenScreen(UI_CONFIG, anim);
            break;
        case UI_CONFIG_MQTT_CONTROLS:
            lvglOpenScreen(UI_HOME, anim);
            break;
        case UI_CONFIG_MQTT_BUTTONS:
            lvglOpenScreen(UI_CONFIG_MQTT_CONTROLS, anim);
            break;
        default:
            break;
    }
    if (prev == UI_MEDIA && networkSuspendedForAudio && !mediaIsPlaying && !mediaPaused) {
        networkResumeAfterAudio();
    }
}

void lvglNavigateBackBySwipe(lv_scr_load_anim_t anim)
{
    UiScreen sourceScreen = lvglSwipeSourceScreen;
    if (!uiScreenSupportsSwipeBack(sourceScreen)) {
        const UiScreen activeScreen = lvglUiForScreenObj(lv_scr_act());
        sourceScreen = uiScreenSupportsSwipeBack(activeScreen) ? activeScreen : uiScreen;
    }
    lvglSwipeSourceScreen = UI_HOME;
    lvglNavigateBackFromScreen(sourceScreen, anim);
}

void lvglHomeNavEvent(lv_event_t *e)
{
    if (lvglClickSuppressed() || lvglReorderDrag.active) return;
    const UiScreen target = static_cast<UiScreen>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (target == UI_CONFIG && configLockIsConfigured()) {
        configLockPromptForEntry();
        return;
    }
    if (target == UI_CHAT) {
        lvglHideChatMenu();
        currentChatPeerKey = "";
        chatClearCache();
    } else if (target == UI_CONFIG_MQTT_CONTROLS) {
        lvglEnsureScreenBuilt(UI_CONFIG_MQTT_CONTROLS);
        lvglRefreshMqttControlsUi();
    }
    lvglOpenScreen(target, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglChatAirplanePromptEvent(lv_event_t *e)
{
    lv_obj_t *msgbox = lv_event_get_current_target(e);
    const char *txt = lv_msgbox_get_active_btn_text(msgbox);
    if (txt && strcmp(txt, "Airplane Off") == 0) {
        applyAirplaneMode(false, "chat_prompt");
    } else {
        uiStatusLine = "Airplane mode: WiFi chat unavailable";
        lvglSyncStatusLine();
    }
    lvglEnsureScreenBuilt(UI_CHAT);
    lvglRefreshChatLayout();
    lvglRefreshChatContactsUi();
    lvglRefreshChatUi();
    lvglOpenScreen(UI_CHAT, LV_SCR_LOAD_ANIM_MOVE_LEFT);
    lv_msgbox_close(msgbox);
}

void lvglShowChatAirplanePrompt()
{
    static const char *btns[] = {"Cancel", "Airplane Off", ""};
    lv_obj_t *m = lv_msgbox_create(nullptr, "WiFi Chat Unavailable",
                                   "Airplane mode is on.\nTurn it off to use WiFi chat.",
                                   btns, false);
    if (!m) return;
    lvglApplyMsgboxModalStyle(m);
    lv_obj_center(m);
    lv_obj_set_width(m, min<int16_t>(DISPLAY_WIDTH - 24, 260));
    lv_obj_add_event_cb(m, lvglChatAirplanePromptEvent, LV_EVENT_VALUE_CHANGED, nullptr);
}

void lvglRadioModeWarningEvent(lv_event_t *e)
{
    lv_obj_t *msgbox = lv_event_get_current_target(e);
    lv_msgbox_close(msgbox);
}

static void lvglApplyMarqueeLabel(lv_obj_t *label, lv_coord_t width = LV_PCT(100))
{
    if (!label) return;

    lv_obj_set_width(label, width);

    // Enable scrolling for long text
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);

    // IMPORTANT: reset text AFTER setting mode
    lv_label_set_text(label, "");

    // Align left so scroll starts from left edge
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_LEFT, 0);

    // Padding for readability
    lv_obj_set_style_pad_left(label, 2, 0);
    lv_obj_set_style_pad_right(label, 2, 0);

    // Ensure label itself is not treated as scroll container
    lv_obj_clear_flag(label, LV_OBJ_FLAG_SCROLLABLE);
}

static void lvglShowE220FixedModeWarning()
{
    static const char *btns[] = {"OK", ""};
    lv_obj_t *m = lv_msgbox_create(nullptr,
                                   "E220 Fixed Mode",
                                   "E220 Fixed mode disables current radio chat and discovery.\nUse Transparent mode for encrypted radio messaging.",
                                   btns,
                                   false);
    if (!m) return;
    lvglApplyMsgboxModalStyle(m);
    lv_obj_center(m);
    lv_obj_set_width(m, min<int16_t>(DISPLAY_WIDTH - 24, 270));
    lv_obj_add_event_cb(m, lvglRadioModeWarningEvent, LV_EVENT_VALUE_CHANGED, nullptr);
}

static void powerOffSignalPulse()
{
    if (POWER_OFF_SIGNAL_PIN < 0) return;
    pinMode(POWER_OFF_SIGNAL_PIN, OUTPUT);
    digitalWrite(POWER_OFF_SIGNAL_PIN, HIGH);
    delay(10);
    digitalWrite(POWER_OFF_SIGNAL_PIN, LOW);
    delay(POWER_OFF_PULSE_MS);
    digitalWrite(POWER_OFF_SIGNAL_PIN, HIGH);
    delay(POWER_OFF_GAP_MS);
    digitalWrite(POWER_OFF_SIGNAL_PIN, LOW);
    delay(POWER_OFF_PULSE_MS);
    digitalWrite(POWER_OFF_SIGNAL_PIN, HIGH);
}

static bool powerDialogSupportsManualPowerOff()
{
#if defined(BOARD_ESP32_2432S024C) || defined(BOARD_ESP32_3248S035)
    return true;
#else
    return POWER_OFF_SIGNAL_PIN >= 0;
#endif
}

void lvglPowerDialogEvent(lv_event_t *e)
{
    lv_obj_t *msgbox = lv_event_get_current_target(e);
    const char *txt = lv_msgbox_get_active_btn_text(msgbox);
    if (!txt) {
        lv_msgbox_close(msgbox);
        return;
    }

    if (strcmp(txt, tr(TXT_RESTART)) == 0) {
        lv_msgbox_close(msgbox);
        uiStatusLine = "Restarting...";
        if (lvglReady) lvglSyncStatusLine();
        delay(120);
        ESP.restart();
        return;
    }

    if (strcmp(txt, tr(TXT_POWER_OFF)) == 0) {
        lv_msgbox_close(msgbox);
        if (POWER_OFF_SIGNAL_PIN >= 0) {
            uiStatusLine = "Power-off signal sent";
            if (lvglReady) lvglSyncStatusLine();
            powerOffSignalPulse();
        } else if (powerDialogSupportsManualPowerOff()) {
            uiStatusLine = "Entering deep sleep...";
            if (lvglReady) lvglSyncStatusLine();
            delay(80);
            enterDeepSleep();
        }
        return;
    }

    lv_msgbox_close(msgbox);
}

void lvglPowerButtonEvent(lv_event_t *e)
{
    (void)e;
    if (lvglClickSuppressed() || lvglReorderDrag.active) return;
    static const char *btns[4];
    btns[0] = tr(TXT_CANCEL);
    btns[1] = tr(TXT_RESTART);
    btns[2] = powerDialogSupportsManualPowerOff() ? tr(TXT_POWER_OFF) : "";
    btns[3] = "";

    lv_obj_t *m = lv_msgbox_create(nullptr,
                                   tr(TXT_POWER),
                                   tr(TXT_POWER_ACTION_BODY),
                                   btns,
                                   false);
    if (!m) return;

    lvglApplyMsgboxModalStyle(m);
    lv_obj_center(m);
    const lv_coord_t powerPopupWidth =
#if defined(BOARD_ESP32_2432S024C)
        static_cast<lv_coord_t>(max<int16_t>(DISPLAY_WIDTH - 8, 220));
#else
        static_cast<lv_coord_t>(min<int16_t>(DISPLAY_WIDTH - 24, 270));
#endif
    lv_obj_set_width(m, powerPopupWidth);
    lv_obj_add_event_cb(m, lvglPowerDialogEvent, LV_EVENT_VALUE_CHANGED, nullptr);

    lv_obj_t *btnMatrix = lv_msgbox_get_btns(m);
    if (btnMatrix) {
        lv_btnmatrix_set_btn_ctrl(btnMatrix, 2, LV_BTNMATRIX_CTRL_CHECKABLE);
        lv_obj_set_width(btnMatrix, lv_pct(100));
        lv_obj_set_style_bg_color(btnMatrix, lv_color_hex(0xA33F3F), LV_PART_ITEMS | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(btnMatrix, lv_color_hex(0x7F2D2D), LV_PART_ITEMS | LV_STATE_PRESSED);
    }
}

bool lvglChatPromptIfAirplaneBlocked()
{
    if (!chatPeerBlockedByAirplaneMode(currentChatPeerKey)) return false;
    if (lvglChatInputTa && !currentChatPeerKey.isEmpty()) {
        String text = lv_textarea_get_text(lvglChatInputTa);
        text.trim();
        if (!text.isEmpty()) chatStageDeferredAirplaneMessage(currentChatPeerKey, text);
    }
    lvglShowChatAirplanePrompt();
    return true;
}

void lvglOpenChatPeersEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_CHAT_PEERS);
    lvglRefreshChatPeerUi();
    lvglOpenScreen(UI_CHAT_PEERS, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglChatDiscoveryToggleEvent(lv_event_t *e)
{
    (void)e;
    p2pDiscoveryEnabled = !p2pDiscoveryEnabled;
    saveP2pConfig();
    lvglApplyChatDiscoveryButtonStyle();
    uiStatusLine = p2pDiscoveryEnabled ? tr(TXT_PEER_DISCOVERY_ENABLED) : tr(TXT_PEER_DISCOVERY_DISABLED);
    lvglSyncStatusLine();
    if (!p2pDiscoveryEnabled) {
        hc12LastDiscoveryAnnounceMs = millis();
    } else {
        hc12LastDiscoveryAnnounceMs = 0;
        p2pLastDiscoverAnnounceMs = 0;
    }
    if (uiScreen == UI_CHAT_PEERS) lvglRefreshChatPeerUi();
}

void saveApModePref(bool enabled)
{
    wifiPrefs.begin("wifi", false);
    wifiPrefs.putBool("ap_mode", enabled);
    wifiPrefs.end();
}

void lvglApModeEvent(lv_event_t *e)
{
    (void)e;
    if (airplaneModeEnabled) {
        uiStatusLine = "Disable airplane mode first";
        lvglSyncStatusLine();
        return;
    }
    if (wifiSessionApMode) {
        wifiSessionApMode = false;
        saveApModePref(false);
        disableApWhenStaConnected("ui_ap_mode_off");
        if (wifiHasStaTarget()) {
            beginStaConnectAttempt("ui_ap_mode_off");
            uiStatusLine = "Trying saved WiFi";
        } else {
            uiStatusLine = "No saved WiFi";
        }
    } else {
        forceSessionApMode("ui_ap_mode");
        if (p2pDiscoveryEnabled) {
            hc12LastDiscoveryAnnounceMs = 0;
            hc12BroadcastDiscoveryFrame("probe");
        }
        uiStatusLine = "AP mode enabled";
    }
    lvglSyncStatusLine();
    lvglRefreshConfigUi();
    lvglRefreshAllButtonStyles();
    lvglRefreshWifiList();
}

void lvglWifiRescanEvent(lv_event_t *e)
{
    (void)e;
    if (wifiScanInProgress) stopWifiScan(true);
    else refreshWifiScan();
    lvglRefreshWifiList();
}

void lvglWifiDisconnectEvent(lv_event_t *e)
{
    (void)e;
    wifiForgetPendingUi = false;
    pendingSaveCreds = false;
    pendingSaveSsid = "";
    pendingSavePass = "";
    WiFi.disconnect(false, false);
    ensureApOnline("manual_disconnect");
    uiStatusLine = "WiFi disconnected";
    lvglSyncStatusLine();
    lvglRefreshWifiList();
}

void lvglWifiForgetEvent(lv_event_t *e)
{
    (void)e;
    wifiForgetPendingUi = true;
    pendingSaveCreds = false;
    pendingSaveSsid = "";
    pendingSavePass = "";
    clearStaCreds();
    WiFi.disconnect(false, false);
    ensureApOnline("manual_forget");
    uiStatusLine = "Saved WiFi forgotten";
    lvglSyncStatusLine();
    lvglRefreshWifiList();
}

void lvglWifiApSaveEvent(lv_event_t *e)
{
    (void)e;
    if (!lvglWifiApSsidTa || !lvglWifiApPassTa) return;
    String nextSsid = lv_textarea_get_text(lvglWifiApSsidTa);
    String nextPass = lv_textarea_get_text(lvglWifiApPassTa);
    nextSsid.trim();
    nextPass.trim();
    if (nextSsid.isEmpty()) {
        uiStatusLine = "AP name cannot be empty";
        lvglSyncStatusLine();
        return;
    }
    if (!nextPass.isEmpty() && nextPass.length() < 8) {
        uiStatusLine = "AP password must be 8+ chars";
        lvglSyncStatusLine();
        return;
    }
    saveApCreds(nextSsid, nextPass);
    if (apModeActive) {
        stopDnsForAp();
        WiFi.softAPdisconnect(true);
        apModeActive = false;
    }
    ensureApOnline("ap_config_saved");
    uiStatusLine = "AP config saved";
    lvglSyncStatusLine();
    lvglRefreshWifiList();
}

void lvglWifiWebServerToggleEvent(lv_event_t *e)
{
    (void)e;
    webServerEnabled = !webServerEnabled;
    persistWebServerEnabled();
    if (webServerEnabled) {
        ensureWebServerRuntime();
        uiStatusLine = "Web server enabled";
    } else {
        stopWebServerRuntime();
        uiStatusLine = "Web server disabled";
    }
    if (lvglReady) {
        lvglSyncStatusLine();
        lvglRefreshWifiList();
        lvglRefreshTopIndicators();
        if (uiScreen == UI_INFO) lvglRefreshInfoPanel();
    }
}

void lvglWifiEntryEvent(lv_event_t *e)
{
    const int idx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (idx < 0 || idx >= wifiCount) return;
    if (wifiEntries[idx].auth != WIFI_AUTH_OPEN) {
        lvglOpenWifiPasswordDialog(wifiEntries[idx].ssid);
        return;
    }
    startWifiConnect(wifiEntries[idx].ssid, "");
    uiStatusLine = "Connecting to open AP: " + wifiEntries[idx].ssid;
    lvglSyncStatusLine();
}

void lvglMediaRefreshEvent(lv_event_t *e)
{
    (void)e;
    mediaEnsureStorageReadyForUi();
    lvglQueueMediaRefresh();
}

void lvglMediaPrevPageEvent(lv_event_t *e)
{
    (void)e;
    if (mediaOffset <= 0) return;
    mediaOffset -= MEDIA_PAGE_SIZE;
    if (mediaOffset < 0) mediaOffset = 0;
    lvglQueueMediaRefresh();
}

void lvglMediaNextPageEvent(lv_event_t *e)
{
    (void)e;
    if (!mediaHasMore) return;
    mediaOffset += MEDIA_PAGE_SIZE;
    lvglQueueMediaRefresh();
}

void lvglMediaStopEvent(lv_event_t *e)
{
    (void)e;
    audioStopPlayback(true);
    uiStatusLine = "Playback stopped";
    lvglSyncStatusLine();
    lvglRefreshMediaPlayerUi();
}

void lvglMediaPlayStopEvent(lv_event_t *e)
{
    (void)e;
    if (mediaIsPlaying || mediaPaused) {
        audioStopPlayback(true);
        uiStatusLine = "Playback stopped";
        lvglSyncStatusLine();
        lvglRefreshMediaPlayerUi();
        return;
    }
    if (mediaSelectedSourcePath.isEmpty()) {
        uiStatusLine = "Select a track first";
        lvglSyncStatusLine();
        return;
    }
    if (!mediaStartTrack(mediaSelectedSourcePath, mediaSelectedTrackName)) {
        uiStatusLine = "Play failed: " + mediaSelectedTrackName;
        if (audioLastError == "decoder_mem") uiStatusLine += " (low RAM for decoder)";
        else if (audioLastError == "sd_access") uiStatusLine += " (SD access)";
        else if (audioLastError == "flac_mem") uiStatusLine += " (FLAC needs more RAM/PSRAM)";
        lvglSyncStatusLine();
        lvglRefreshMediaPlayerUi();
        return;
    }
    lvglRefreshMediaPlayerUi();
}

void lvglMediaPrevTrackEvent(lv_event_t *e)
{
    (void)e;
    String path = mediaFindAdjacentTrack(mediaSelectedSourcePath, false);
    if (path.isEmpty()) {
        uiStatusLine = "No previous track";
        lvglSyncStatusLine();
        return;
    }
    String name = mediaDisplayNameFromPath(path);
    if (!mediaStartTrack(path, name)) {
        uiStatusLine = "Play failed: " + name;
        if (audioLastError == "decoder_mem") uiStatusLine += " (low RAM for decoder)";
        else if (audioLastError == "sd_access") uiStatusLine += " (SD access)";
        else if (audioLastError == "flac_mem") uiStatusLine += " (FLAC needs more RAM/PSRAM)";
        lvglSyncStatusLine();
    }
    lvglRefreshMediaPlayerUi();
}

void lvglMediaNextTrackEvent(lv_event_t *e)
{
    (void)e;
    String path = mediaFindAdjacentTrack(mediaSelectedSourcePath, true);
    if (path.isEmpty()) {
        uiStatusLine = "No next track";
        lvglSyncStatusLine();
        return;
    }
    String name = mediaDisplayNameFromPath(path);
    if (!mediaStartTrack(path, name)) {
        uiStatusLine = "Play failed: " + name;
        if (audioLastError == "decoder_mem") uiStatusLine += " (low RAM for decoder)";
        else if (audioLastError == "sd_access") uiStatusLine += " (SD access)";
        else if (audioLastError == "flac_mem") uiStatusLine += " (FLAC needs more RAM/PSRAM)";
        lvglSyncStatusLine();
    }
    lvglRefreshMediaPlayerUi();
}

void lvglMediaVolumeEvent(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    if (!obj || obj != lvglMediaVolSlider) return;
    int v = lv_slider_get_value(obj);
    if (v < 0) v = 0;
    if (v > 100) v = 100;
    mediaVolumePercent = static_cast<uint8_t>(v);
    saveSoundPrefs();
    if (audioBackendReady && audio) audioSetVolumeImmediate(audioVolumeLevelFromPercent(mediaVolumePercent));
    lvglRefreshMediaPlayerUi();
    lvglRefreshConfigUi();
    lvglRefreshSoundPopupUi();
    lvglTopIndicatorStateValid = false;
    lvglRefreshTopIndicators();
}

void lvglMediaVolumeStepEvent(lv_event_t *e)
{
    const int step = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    int v = static_cast<int>(mediaVolumePercent) + step;
    if (v < 0) v = 0;
    if (v > 100) v = 100;
    mediaVolumePercent = static_cast<uint8_t>(v);
    if (lvglMediaVolSlider) lv_slider_set_value(lvglMediaVolSlider, static_cast<int32_t>(mediaVolumePercent), LV_ANIM_OFF);
    saveSoundPrefs();
    if (audioBackendReady && audio) audioSetVolumeImmediate(audioVolumeLevelFromPercent(mediaVolumePercent));
    lvglRefreshMediaPlayerUi();
    lvglRefreshConfigUi();
    lvglRefreshSoundPopupUi();
    lvglTopIndicatorStateValid = false;
    lvglRefreshTopIndicators();
}

void lvglRefreshMediaPlayerUi()
{
    if (!lvglMediaTrackLabel || !lvglMediaPlayBtnLabel || !lvglMediaProgressBar || !lvglMediaProgressLabel) return;
    if (uiScreen != UI_MEDIA) return;

    if (mediaSelectedTrackName.isEmpty() && !mediaNowPlaying.isEmpty()) mediaSelectedTrackName = mediaNowPlaying;
    const bool hasTrack = !mediaSelectedSourcePath.isEmpty();
    lvglSetMediaPlayerVisible(hasTrack);

    String trackText = hasTrack ? mediaSelectedTrackName : String("No track selected");
    if (trackText.length() > 34) trackText = trackText.substring(0, 31) + "...";
    lvglLabelSetTextIfChanged(lvglMediaTrackLabel, String("Track: ") + trackText);

    lvglLabelSetTextIfChanged(
        lvglMediaPlayBtnLabel,
        lvglSymbolText((mediaIsPlaying || mediaPaused) ? LV_SYMBOL_STOP : LV_SYMBOL_PLAY,
                       (mediaIsPlaying || mediaPaused) ? "Stop" : "Play"));

    if (lvglMediaVolSlider && lv_slider_get_value(lvglMediaVolSlider) != static_cast<int32_t>(mediaVolumePercent)) {
        lv_slider_set_value(lvglMediaVolSlider, static_cast<int32_t>(mediaVolumePercent), LV_ANIM_OFF);
    }
    if (lvglMediaVolValueLabel) lvglLabelSetTextIfChanged(lvglMediaVolValueLabel, String(mediaVolumePercent) + "%");

    if (lvglMediaPlayBtn) {
        if (hasTrack) lv_obj_clear_state(lvglMediaPlayBtn, LV_STATE_DISABLED);
        else lv_obj_add_state(lvglMediaPlayBtn, LV_STATE_DISABLED);
    }
    if (lvglMediaPrevBtn) {
        if (hasTrack) lv_obj_clear_state(lvglMediaPrevBtn, LV_STATE_DISABLED);
        else lv_obj_add_state(lvglMediaPrevBtn, LV_STATE_DISABLED);
    }
    if (lvglMediaNextBtn) {
        if (hasTrack) lv_obj_clear_state(lvglMediaNextBtn, LV_STATE_DISABLED);
        else lv_obj_add_state(lvglMediaNextBtn, LV_STATE_DISABLED);
    }

    uint32_t curSec = 0;
    uint32_t durSec = 0;
    if (audioBackendReady && audio && hasTrack) {
        curSec = audio->getAudioCurrentTime();
        durSec = audio->getAudioFileDuration();
        if (durSec > 0 && curSec > durSec) curSec = durSec;
    }
    const int32_t progress = (durSec > 0U) ? static_cast<int32_t>((static_cast<uint64_t>(curSec) * 1000ULL) / static_cast<uint64_t>(durSec)) : 0;
    lvglBarSetValueIfChanged(lvglMediaProgressBar, progress, LV_ANIM_OFF);

    char curBuf[8] = {0};
    char durBuf[8] = {0};
    mediaFormatSeconds(curSec, curBuf, sizeof(curBuf));
    mediaFormatSeconds(durSec, durBuf, sizeof(durBuf));
    lvglLabelSetTextIfChanged(lvglMediaProgressLabel, String(curBuf) + " / " + durBuf);
}

void lvglMediaEntryEvent(lv_event_t *e)
{
    const int idx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (idx == MEDIA_ENTRY_PARENT) {
        if (mediaCurrentDir != "/") {
            int slash = mediaCurrentDir.lastIndexOf('/');
            mediaCurrentDir = (slash <= 0) ? "/" : mediaCurrentDir.substring(0, slash);
            mediaOffset = 0;
            lvglQueueMediaRefresh();
        }
        return;
    }
    if (idx == MEDIA_ENTRY_PREV_PAGE) {
        lvglMediaPrevPageEvent(e);
        return;
    }
    if (idx == MEDIA_ENTRY_NEXT_PAGE) {
        lvglMediaNextPageEvent(e);
        return;
    }
    if (idx < 0 || idx >= mediaCount) return;

    if (mediaEntries[idx].isDir) {
        mediaCurrentDir = mediaEntries[idx].path;
        mediaOffset = 0;
        lvglQueueMediaRefresh();
        return;
    }

    mediaSelectedSourcePath = mediaEntries[idx].path;
    mediaSelectedTrackName = mediaEntries[idx].name;
    lvglRefreshMediaPlayerUi();

    if (!mediaStartTrack(mediaEntries[idx].path, mediaEntries[idx].name)) {
        uiStatusLine = "Play failed: " + mediaEntries[idx].name;
        if (audioLastError == "decoder_mem") uiStatusLine += " (low RAM for decoder)";
        else if (audioLastError == "sd_access") uiStatusLine += " (SD access)";
        else if (audioLastError == "flac_mem") uiStatusLine += " (FLAC needs more RAM/PSRAM)";
        lvglSyncStatusLine();
        lvglRefreshMediaPlayerUi();
        return;
    }
    lvglRefreshMediaPlayerUi();
}

void lvglRefreshChatUi()
{
    if (!lvglChatList) return;
    lv_obj_clean(lvglChatList);
    lvglChatEmptyLabel = nullptr;

    if (currentChatPeerKey.isEmpty()) {
        return;
    }

    if (chatMessageCount == 0) {
        lvglChatEmptyLabel = lv_label_create(lvglChatList);
        lv_label_set_text(lvglChatEmptyLabel, "No messages yet for this contact.");
        lv_obj_set_width(lvglChatEmptyLabel, lv_pct(100));
        lv_obj_set_style_text_color(lvglChatEmptyLabel, lv_color_hex(0xC8CED6), 0);
        lv_label_set_long_mode(lvglChatEmptyLabel, LV_LABEL_LONG_WRAP);
        return;
    }

    for (int i = 0; i < chatMessageCount; ++i) {
        const String authorLabel = chatDisplayAuthorForMessage(chatMessages[i]);
        String checkersInviteSessionId;
        const bool checkersInvite = checkersParseInviteText(chatMessages[i].text, checkersInviteSessionId);
        const String bodyText = checkersVisibleBodyFromRawText(chatMessages[i].text);
        if (bodyText.isEmpty()) continue;
        const bool pendingDelivery = chatMessages[i].outgoing &&
                                    chatMessagePendingForPeer(currentChatPeerKey, chatMessages[i].messageId);
        const bool failedDelivery = chatMessages[i].outgoing && chatMessages[i].failed && !pendingDelivery;
        const bool deliveredOutgoing = chatMessages[i].outgoing && !pendingDelivery && !failedDelivery;
        lv_obj_t *row = lv_obj_create(lvglChatList);
        lv_obj_set_width(row, lv_pct(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_set_style_pad_row(row, 0, 0);
        lv_obj_set_style_pad_column(row, 0, 0);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(
            row,
            chatMessages[i].outgoing ? LV_FLEX_ALIGN_END : LV_FLEX_ALIGN_START,
            LV_FLEX_ALIGN_START,
            LV_FLEX_ALIGN_START);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        if (chatMessages[i].outgoing) {
            lv_obj_t *statusBadge = lv_obj_create(row);
            lv_obj_set_size(statusBadge, 18, LV_SIZE_CONTENT);

            const bool failedDelivery = chatMessages[i].failed && !pendingDelivery;
            const lv_color_t badgeColor = pendingDelivery ? lv_color_hex(0x5A3A1E)
                                          : (failedDelivery ? lv_color_hex(0x6A1F2B)
                                                            : lv_color_hex(0x1E4D2B));
            const lv_color_t textColor = pendingDelivery ? lv_color_hex(0xFFD27A)
                                         : (failedDelivery ? lv_color_hex(0xFF9AA8)
                                                           : lv_color_hex(0x9BF0AE));
            const char *statusText = pendingDelivery ? LV_SYMBOL_UPLOAD
                                     : (failedDelivery ? LV_SYMBOL_CLOSE
                                                       : LV_SYMBOL_OK);

            lv_obj_set_style_bg_color(statusBadge, badgeColor, 0);
            lv_obj_set_style_border_width(statusBadge, 0, 0);
            lv_obj_set_style_radius(statusBadge, 9, 0);
            lv_obj_set_style_pad_left(statusBadge, 3, 0);
            lv_obj_set_style_pad_right(statusBadge, 3, 0);
            lv_obj_set_style_pad_top(statusBadge, pendingDelivery ? 4 : 2, 0);
            lv_obj_set_style_pad_bottom(statusBadge, pendingDelivery ? 4 : 2, 0);
            lv_obj_set_style_pad_row(statusBadge, 0, 0);
            lv_obj_clear_flag(statusBadge, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *statusLbl = lv_label_create(statusBadge);
            lv_label_set_text(statusLbl, statusText);
            lv_obj_set_style_text_color(statusLbl, textColor, 0);
            lv_obj_set_style_text_align(statusLbl, LV_TEXT_ALIGN_CENTER, 0);
        }

        lv_obj_t *bubble = lv_obj_create(row);
        lv_obj_set_width(bubble, lv_pct(75));
        lv_obj_set_height(bubble, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_color(bubble, chatMessages[i].outgoing ? lv_color_hex(0x254A33) : lv_color_hex(0x213246), 0);
        lv_obj_set_style_border_width(bubble, 0, 0);
        lv_obj_set_style_radius(bubble, 12, 0);
        lv_obj_set_style_pad_all(bubble, 8, 0);
        lv_obj_set_style_pad_row(bubble, 4, 0);
        lv_obj_set_flex_flow(bubble, LV_FLEX_FLOW_COLUMN);
        lv_obj_clear_flag(bubble, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *hdr = lv_label_create(bubble);
        lv_label_set_text_fmt(hdr, "%s", authorLabel.c_str());
        lv_obj_set_style_text_color(hdr, chatMessages[i].outgoing ? lv_color_hex(0xB6E3C6) : lv_color_hex(0xA9C8E8), 0);

        lv_obj_t *body = lv_label_create(bubble);
        lv_obj_set_width(body, lv_pct(100));
        lv_label_set_long_mode(body, LV_LABEL_LONG_WRAP);
        lv_label_set_text(body, bodyText.c_str());
        lv_obj_set_style_text_color(body, lv_color_hex(0xEDF2F7), 0);

        if (checkersInvite) {
            if (!chatMessages[i].outgoing) {
                lv_obj_t *playBtn = lv_btn_create(bubble);
                lv_obj_set_width(playBtn, lv_pct(100));
                lv_obj_set_height(playBtn, 28);
                lv_obj_set_style_radius(playBtn, 10, 0);
                lv_obj_set_style_border_width(playBtn, 0, 0);
                lv_obj_clear_flag(playBtn, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_add_event_cb(playBtn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
                lv_obj_add_event_cb(playBtn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
                lv_obj_add_event_cb(playBtn, lvglCheckersInvitePlayEvent, LV_EVENT_CLICKED, reinterpret_cast<void *>(static_cast<intptr_t>(i)));
                lv_obj_t *playLbl = lv_label_create(playBtn);
                lv_label_set_text(playLbl, "Play");
                lv_obj_center(playLbl);
                lvglRegisterStyledButton(playBtn, lv_color_hex(0x3A8F4B), true);
            } else {
                lv_obj_t *inviteState = lv_label_create(bubble);
                const bool activeInvite = (checkersMode == CHECKERS_MODE_TAG) &&
                                          (checkersSessionId == checkersInviteSessionId) &&
                                          (checkersPeerKey == currentChatPeerKey);
                if (activeInvite) {
                    lv_label_set_text(inviteState, checkersWaitingForRemote ? "Pending other player..." : "Game started");
                } else {
                    lv_label_set_text(inviteState, deliveredOutgoing ? "Invite sent" : "Sending invite...");
                }
                lv_obj_set_style_text_color(inviteState, lv_color_hex(0xB9D5E7), 0);
            }
        }

        if (chatMessages[i].outgoing) {
            lv_obj_t *delBtn = lv_btn_create(row);
            lv_obj_set_size(delBtn, 24, 24);
            lv_obj_set_style_border_width(delBtn, 0, 0);
            lv_obj_set_style_radius(delBtn, 12, 0);
            lv_obj_set_style_pad_all(delBtn, 0, 0);
            lv_obj_clear_flag(delBtn, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_event_cb(delBtn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
            lv_obj_add_event_cb(delBtn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
            lv_obj_add_event_cb(delBtn, lvglDeleteChatMessageEvent, LV_EVENT_CLICKED, reinterpret_cast<void *>(static_cast<intptr_t>(i)));
            lv_obj_t *delLbl = lv_label_create(delBtn);
            lv_label_set_text(delLbl, LV_SYMBOL_TRASH);
            lv_obj_set_style_text_color(delLbl, lv_color_hex(0xF5D2D2), 0);
            lv_obj_center(delLbl);
            lvglRegisterStyledButton(delBtn, lv_color_hex(0x7A2E2E), true);
        }
    }

    lv_obj_scroll_to_view_recursive(lv_obj_get_child(lvglChatList, lv_obj_get_child_cnt(lvglChatList) - 1), LV_ANIM_OFF);
}

void lvglToggleChatMenuEvent(lv_event_t *e)
{
    (void)e;
    if (!lvglChatMenuPanel || currentChatPeerKey.isEmpty()) return;
    if (lv_obj_has_flag(lvglChatMenuPanel, LV_OBJ_FLAG_HIDDEN)) {
        if (lvglChatMenuBackdrop) {
            lv_obj_clear_flag(lvglChatMenuBackdrop, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(lvglChatMenuBackdrop);
        }
        lv_obj_clear_flag(lvglChatMenuPanel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(lvglChatMenuPanel);
    } else {
        lvglHideChatMenu();
    }
}

void lvglDeleteChatMessageEvent(lv_event_t *e)
{
    const int idx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (currentChatPeerKey.isEmpty()) return;
    chatDeleteMessageAt(currentChatPeerKey, idx);
}

void lvglDeleteChatConversationEvent(lv_event_t *e)
{
    (void)e;
    if (currentChatPeerKey.isEmpty()) return;
    lvglHideChatMenu();
    chatApplyConversationDeletion(currentChatPeerKey, "Conversation deleted");
}

void lvglDeleteChatConversationForAllEvent(lv_event_t *e)
{
    (void)e;
    if (currentChatPeerKey.isEmpty()) return;
    const String peerKey = currentChatPeerKey;
    lvglHideChatMenu();
    const bool sentLan = p2pSendConversationDelete();
    const bool sentGlobal = mqttPublishConversationDelete();
    const bool sentRadio = hc12SendConversationDelete(peerKey);
    chatApplyConversationDeletion(peerKey, "Conversation deleted");
    uiStatusLine = sentGlobal ? "Conversation deleted for all"
                              : ((sentLan || sentRadio) ? "Conversation delete sent" : "Deleted locally only");
    lvglSyncStatusLine();
}

void lvglRefreshChatLayout()
{
    if (!lvglChatContactLabel || !lvglChatContacts || !lvglChatList || !lvglChatComposer || !lvglChatPeersBtn || !lvglChatMenuBtn || !lvglChatMenuPanel || !lvglChatMenuBackdrop) return;
    const bool showingConversation = !currentChatPeerKey.isEmpty();
    if (!showingConversation) {
        lv_label_set_text(lvglChatContactLabel, "");
        lv_obj_clear_flag(lvglChatPeersBtn, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(lvglChatMenuBtn, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(lvglChatMenuBackdrop, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(lvglChatMenuPanel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_size(lvglChatContacts, lv_pct(100), UI_CONTENT_H - 44);
        lv_obj_align(lvglChatContacts, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y + 44);
        lv_obj_set_flex_flow(lvglChatContacts, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_scroll_dir(lvglChatContacts, LV_DIR_VER);
        lv_obj_set_style_pad_all(lvglChatContacts, 6, 0);
        lv_obj_set_style_pad_row(lvglChatContacts, 6, 0);
        lv_obj_clear_flag(lvglChatContacts, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(lvglChatList, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(lvglChatComposer, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_label_set_text_fmt(lvglChatContactLabel, "%s", chatDisplayNameForPeerKey(currentChatPeerKey).c_str());
        lv_obj_add_flag(lvglChatPeersBtn, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(lvglChatMenuBtn, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(lvglChatContacts, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(lvglChatList, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(lvglChatComposer, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_size(lvglChatList, lv_pct(100), UI_CONTENT_H - 98);
        lv_obj_align(lvglChatList, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y + 44);
        lv_obj_align(lvglChatComposer, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_set_size(lvglChatMenuBackdrop, lv_pct(100), UI_CONTENT_H);
        lv_obj_align(lvglChatMenuBackdrop, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
        lv_obj_align(lvglChatMenuPanel, LV_ALIGN_TOP_RIGHT, -8, UI_CONTENT_TOP_Y + 40);
        if (!lv_obj_has_flag(lvglChatMenuBackdrop, LV_OBJ_FLAG_HIDDEN)) lv_obj_move_foreground(lvglChatMenuBackdrop);
        if (!lv_obj_has_flag(lvglChatMenuPanel, LV_OBJ_FLAG_HIDDEN)) lv_obj_move_foreground(lvglChatMenuPanel);
        if (lvglKeyboardVisible() && lvglKeyboardFocusedTa == lvglChatInputTa) {
            lvglSetChatKeyboardVisible(true);
            lvglKeyboardEnsureFocusedVisible(false);
        }
    }
}

void lvglSetChatKeyboardVisible(bool visible)
{
    if (!lvglChatComposer || !lvglChatList) return;
    if (currentChatPeerKey.isEmpty()) return;
    const lv_coord_t keyboardH = 120;
    const lv_coord_t composerH = 56;
    const lv_coord_t topSectionH = 44;

    if (visible) {
        lv_obj_align(lvglChatComposer, LV_ALIGN_BOTTOM_MID, 0, -keyboardH);
        lv_obj_set_size(lvglChatList, lv_pct(100), UI_CONTENT_H - topSectionH - composerH - keyboardH);
    } else {
        lv_obj_align(lvglChatComposer, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_set_size(lvglChatList, lv_pct(100), UI_CONTENT_H - topSectionH - composerH);
    }
}

void lvglRefreshChatContactsUi()
{
    lvglRefreshChatLayout();
    if (!lvglChatContacts) return;
    lv_obj_clean(lvglChatContacts);

    bool hasContacts = false;
    if (chatPeerHasHistory("ha_inbox")) hasContacts = true;
    for (int i = 0; i < p2pPeerCount; ++i) {
        if (!p2pPeers[i].enabled) continue;
        hasContacts = true;
        break;
    }

    if (!hasContacts) {
        lv_obj_t *lbl = lv_label_create(lvglChatContacts);
        lv_label_set_text(lbl, "No chats yet");
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xC8CED6), 0);
        lvglRegisterReorderableItem(lbl, "ord_ctc", "empty");
        lvglApplySavedOrder(lvglChatContacts, "ord_ctc");
        return;
    }

    auto makeContactBtn = [](lv_obj_t *parent, const char *txt, lv_color_t col, lv_event_cb_t cb, void *ud) -> lv_obj_t * {
        const bool conversationOpen = !currentChatPeerKey.isEmpty();
        lv_obj_t *b = lv_btn_create(parent);
        if (!b) return nullptr;
        lv_obj_set_size(b, conversationOpen ? LV_SIZE_CONTENT : lv_pct(100), conversationOpen ? 30 : 42);
        lv_obj_set_style_pad_left(b, 10, 0);
        lv_obj_set_style_pad_right(b, 10, 0);
        lv_obj_set_style_radius(b, 10, 0);
        lv_obj_set_style_border_width(b, 0, 0);
        lv_obj_add_event_cb(b, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(b, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
        lvglRegisterStyledButton(b, col, conversationOpen);
        return b;
    };

    if (chatPeerHasHistory("ha_inbox")) {
        lv_color_t col = currentChatPeerKey == "ha_inbox" ? lv_color_hex(0x3A7A3A) : lv_color_hex(0x5B4B8A);
        lv_obj_t *btn = makeContactBtn(
            lvglChatContacts,
            "Home Assistant",
            col,
            [](lv_event_t *e) {
                (void)e;
                chatScheduleOpenPeerConversation("ha_inbox");
            },
            nullptr);
        if (btn) {
            lvglRegisterReorderableItem(btn, "ord_ctc", "ha_inbox");
            lv_obj_t *nameLabel = lv_label_create(btn);
            if (nameLabel) {
                lv_label_set_text(nameLabel, "Home Assistant");
                lv_obj_align(nameLabel, LV_ALIGN_LEFT_MID, 10, 0);
                lvglRegisterStyledButton(btn, col, !currentChatPeerKey.isEmpty());
            }
            if (haChatUnread) {
                lv_obj_t *dot = lv_obj_create(btn);
                if (dot) {
                    lv_obj_set_size(dot, 10, 10);
                    lv_obj_align(dot, LV_ALIGN_RIGHT_MID, -10, 0);
                    lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
                    lv_obj_set_style_bg_color(dot, lv_color_hex(0x66DD77), 0);
                    lv_obj_set_style_border_width(dot, 0, 0);
                    lv_obj_clear_flag(dot, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
                }
            }
        }
    }

    for (int i = 0; i < p2pPeerCount; ++i) {
        if (!p2pPeers[i].enabled) continue;
        lv_color_t col = p2pPeers[i].pubKeyHex == currentChatPeerKey ? lv_color_hex(0x3A7A3A) : lv_color_hex(0x2F6D86);
        lv_obj_t *btn = makeContactBtn(
            lvglChatContacts,
            p2pPeers[i].name.isEmpty() ? "Peer" : p2pPeers[i].name.c_str(),
            col,
            [](lv_event_t *e) {
                const int rawIdx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
                if (rawIdx < 0 || rawIdx >= p2pPeerCount) return;
                chatScheduleOpenPeerConversation(p2pPeers[rawIdx].pubKeyHex);
            },
            reinterpret_cast<void *>(static_cast<intptr_t>(i)));
        if (!btn) continue;
        const String orderKey = lvglOrderTokenFromText("p", p2pPeers[i].pubKeyHex);
        lvglRegisterReorderableItem(btn, "ord_ctc", orderKey.c_str());

        lv_obj_t *nameLabel = lv_label_create(btn);
        if (nameLabel) {
            lv_label_set_text(nameLabel, p2pPeers[i].name.isEmpty() ? "Peer" : p2pPeers[i].name.c_str());
            lv_obj_align(nameLabel, LV_ALIGN_LEFT_MID, 10, 0);
            lvglRegisterStyledButton(btn, col, !currentChatPeerKey.isEmpty());
        }

        if (p2pPeers[i].unread) {
            lv_obj_t *dot = lv_obj_create(btn);
            if (dot) {
                lv_obj_set_size(dot, 10, 10);
                lv_obj_align(dot, LV_ALIGN_RIGHT_MID, -10, 0);
                lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
                lv_obj_set_style_bg_color(dot, lv_color_hex(0x66DD77), 0);
                lv_obj_set_style_border_width(dot, 0, 0);
                lv_obj_clear_flag(dot, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
            }
        }
    }

    lvglApplySavedOrder(lvglChatContacts, "ord_ctc");
}

void lvglSetChatPeerScanButtonStatus(const char *text, uint32_t revertDelayMs)
{
    if (!lvglChatPeerScanBtn) return;
    lv_obj_t *label = lv_obj_get_child(lvglChatPeerScanBtn, 0);
    if (!label) return;
    lv_label_set_text(label, lvglSymbolText(LV_SYMBOL_REFRESH, text ? text : tr(TXT_SCAN)).c_str());
    if (revertDelayMs == 0) {
        lvglChatPeerScanRevertAtMs = 0;
        return;
    }
    lvglChatPeerScanRevertAtMs = millis() + revertDelayMs;
    lv_timer_t *timer = lv_timer_create(
        [](lv_timer_t *timer) {
            lv_obj_t *btn = static_cast<lv_obj_t *>(timer ? timer->user_data : nullptr);
            if (btn) {
                lv_obj_t *lbl = lv_obj_get_child(btn, 0);
                if (lbl) lv_label_set_text(lbl, lvglSymbolText(LV_SYMBOL_REFRESH, tr(TXT_SCAN)).c_str());
            }
            lvglChatPeerScanRevertAtMs = 0;
            if (timer) lv_timer_del(timer);
        },
        revertDelayMs,
        lvglChatPeerScanBtn);
    if (timer) lv_timer_set_repeat_count(timer, 1);
}

static bool hc12SendRadioPayload(const char *payload, size_t len)
{
    hc12InitIfNeeded();
    if (hc12SetIsAsserted() || !radioModuleCanCarryChat() || !payload || len == 0 || len >= HC12_RADIO_MAX_LINE) return false;
    const size_t frameBytes = strlen(HC12_RADIO_FRAME_PREFIX) + len + 1U;
    const unsigned long baud = (radioModuleType == RADIO_MODULE_E220) ? e220RuntimeBaud() : hc12RuntimeBaud();
    const unsigned long bits = static_cast<unsigned long>(frameBytes) * 10UL;
    radioLastTxLeadMs = max<unsigned long>(1UL, ((bits * 1000UL) + (baud - 1UL)) / baud) + RADIO_TX_TIMING_SLOP_MS;
    Serial1.print(HC12_RADIO_FRAME_PREFIX);
    Serial1.write(reinterpret_cast<const uint8_t *>(payload), len);
    Serial1.write('\n');
    return true;
}

static bool hc12SendRadioDocument(JsonDocument &doc)
{
    hc12InitIfNeeded();
    if (hc12SetIsAsserted() || !radioModuleCanCarryChat()) return false;
    char payload[HC12_RADIO_MAX_LINE] = {0};
    const size_t len = serializeJson(doc, payload, sizeof(payload));
    if (len == 0 || len >= sizeof(payload)) return false;
    return hc12SendRadioPayload(payload, len);
}

static bool hc12SendRadioPing()
{
    JsonDocument doc;
    doc["kind"] = "rping";
    doc["device"] = deviceShortNameValue();
    doc["ms"] = millis();
    return hc12SendRadioDocument(doc);
}

static bool hc12SendRadioPong()
{
    JsonDocument doc;
    doc["kind"] = "rpong";
    doc["device"] = deviceShortNameValue();
    doc["ms"] = millis();
    return hc12SendRadioDocument(doc);
}

static bool hc12BroadcastDiscoveryFrame(const char *kind)
{
    if (!p2pDiscoveryEnabled) return false;
    JsonDocument doc;
    doc["kind"] = kind ? kind : "hello";
    doc["device"] = deviceShortNameValue();
    doc["public_key"] = p2pPublicKeyHex();
    return hc12SendRadioDocument(doc);
}

static void hc12DiscoveryService()
{
    if (!p2pDiscoveryEnabled) return;
    if (hc12SetIsAsserted() || !radioModuleCanCarryChat()) return;
    if (radioTxBusy()) return;   // ADD THIS LINE
    if (p2pPairRequestPending) return;
    if (!currentChatPeerKey.isEmpty() && p2pFindPeerByPubKeyHex(currentChatPeerKey) >= 0) return;

    static unsigned long nextDiscoveryMs = 0;
    const unsigned long now = millis();

    if (nextDiscoveryMs == 0) {
        nextDiscoveryMs = now + 1500UL + (unsigned long)random(0, 2500);
        return;
    }

    if ((long)(now - nextDiscoveryMs) < 0) return;

    hc12BroadcastDiscoveryFrame("hello");
    nextDiscoveryMs = now + HC12_DISCOVERY_INTERVAL_MS + (unsigned long)random(0, 2000);
}

static bool hc12SendEncryptedPayload(const String &peerKey, const uint8_t *plain, size_t plainLen)
{
    if (peerKey.isEmpty() || !plain || plainLen == 0 || plainLen > (P2P_MAX_PACKET - 64U)) return false;

    unsigned char peerPk[P2P_PUBLIC_KEY_BYTES] = {0};
    if (!p2pHexToBytes(peerKey, peerPk, sizeof(peerPk))) return false;

    if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
        hc12AppendTerminal((String("[RADIO] tx to peerKey=") + peerKey + "\n").c_str());
        hc12AppendTerminal((String("[RADIO] tx myPub=") + p2pPublicKeyHex() + "\n").c_str());
    }

    unsigned char nonce[P2P_NONCE_BYTES] = {0};
    unsigned char cipher[P2P_MAX_PACKET] = {0};
    randombytes_buf(nonce, sizeof(nonce));
    if (crypto_box_curve25519xchacha20poly1305_easy(cipher, plain, plainLen, nonce, peerPk, p2pSecretKey) != 0) return false;

    const size_t cipherLen = plainLen + P2P_MAC_BYTES;
    char senderPubHex[(P2P_PUBLIC_KEY_BYTES * 2U) + 1U] = {0};
    char nonceHex[(P2P_NONCE_BYTES * 2U) + 1U] = {0};
    char cipherHex[(P2P_MAX_PACKET * 2U) + 1U] = {0};
    char payload[HC12_RADIO_MAX_LINE] = {0};
    sodium_bin2hex(senderPubHex, sizeof(senderPubHex), p2pPublicKey, sizeof(p2pPublicKey));
    sodium_bin2hex(nonceHex, sizeof(nonceHex), nonce, sizeof(nonce));
    sodium_bin2hex(cipherHex, sizeof(cipherHex), cipher, cipherLen);
    const int written = snprintf(payload,
                                 sizeof(payload),
                                 "{\"sender_pub\":\"%s\",\"nonce\":\"%s\",\"cipher\":\"%s\"}",
                                 senderPubHex,
                                 nonceHex,
                                 cipherHex);

    if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
        hc12AppendTerminal((String("[RADIO] enc written=") + String(written) +
                            " max=" + String(sizeof(payload)) + "\n").c_str());
    }

    if (written <= 0 || static_cast<size_t>(written) >= sizeof(payload)) return false;
    return hc12SendRadioPayload(payload, static_cast<size_t>(written));
}

static bool hc12SendPairRequest(const String &name, const String &pubKeyHex)
{
    if (pubKeyHex.isEmpty()) return false;

    JsonDocument doc;
    doc["kind"] = "pair_request";
    doc["author"] = deviceShortNameValue();
    doc["name"] = name.isEmpty() ? String("Peer") : name;

    char plain[P2P_MAX_PACKET] = {0};
    const size_t plainLen = serializeJson(doc, plain, sizeof(plain));
    if (plainLen == 0) return false;

    return hc12SendEncryptedPayload(pubKeyHex, reinterpret_cast<const uint8_t *>(plain), plainLen);
}

static void hc12RestartWithCurrentPins(const String &statusText)
{
    Serial1.flush();
    Serial1.end();
    delay(10);

    uiDeferredFlags &= static_cast<uint8_t>(~(UI_DEFERRED_HC12_SETTLE_PENDING | UI_DEFERRED_HC12_TARGET_ASSERTED));

    delete hc12TerminalLog;
    hc12TerminalLog = nullptr;

    delete hc12RadioRxLine;
    hc12RadioRxLine = nullptr;

    if (!statusText.isEmpty()) hc12ConfigStatusText = statusText;

    hc12InitIfNeeded();
}

void lvglChatPeerActionEvent(lv_event_t *e)
{
    const int action = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));

    if (action == 0) {
        lvglSetChatPeerScanButtonStatus("Scanning...");
        p2pLastDiscoverAnnounceMs = 0;
        p2pSendDiscoveryProbe();
        hc12LastDiscoveryAnnounceMs = 0;
        hc12BroadcastDiscoveryFrame("probe");
        mqttPublishPeerPresence();
        uiStatusLine = "WiFi, radio and MQTT scan started";
        lvglSyncStatusLine();
        lvglRefreshChatPeerUi();
        lvglSetChatPeerScanButtonStatus("Done", 1500);
        return;
    }

    if (action >= 30000) {
        const int raw = action - 30000;
        const int idx = raw / 10;
        const int sub = raw % 10;
        if (idx < 0 || idx >= mqttDiscoveredCount) return;

        const String peerName = mqttDiscoveredPeers[idx].name.isEmpty() ? String("MQTT Peer")
                                                                        : mqttDiscoveredPeers[idx].name;
        const String peerKey = mqttDiscoveredPeers[idx].pubKeyHex;

        if (sub == 1) {
            const bool ok = p2pAddOrUpdateTrustedPeer(peerName, peerKey, IPAddress((uint32_t)0), 0);
            uiStatusLine = ok ? "MQTT peer paired" : "MQTT pair failed";
        } else if (sub == 2) {
            chatScheduleOpenPeerConversation(peerKey);
            return;
        } else if (sub == 3) {
            const int pidx = p2pFindPeerByPubKeyHex(peerKey);
            if (pidx >= 0) {
                const String removedKey = p2pPeers[pidx].pubKeyHex;

                for (int i = pidx + 1; i < p2pPeerCount; ++i) {
                    p2pPeers[i - 1] = p2pPeers[i];
                }
                p2pPeerCount--;
                saveP2pConfig();
                chatRemovePendingForPeer(removedKey);

                if (currentChatPeerKey == removedKey) {
                    chatSelectFirstEnabledPeer();
                    chatReloadRecentMessagesFromSd(currentChatPeerKey);
                }

                uiStatusLine = "MQTT peer unpaired";
            }
        }

        lvglSyncStatusLine();
        lvglRefreshChatPeerUi();
        lvglRefreshChatContactsUi();
        if (uiScreen == UI_CHAT) lvglRefreshChatUi();
        return;
    }

    if (action >= 20000) {
        const int raw = action - 20000;
        const int idx = raw / 10;
        const int sub = raw % 10;
        if (idx < 0 || idx >= hc12DiscoveredCount) return;

        const String peerName = hc12DiscoveredPeers[idx].name.isEmpty() ? String("Radio Peer")
                                                                        : hc12DiscoveredPeers[idx].name;
        const String peerKey = hc12DiscoveredPeers[idx].pubKeyHex;

        if (sub == 1) {
            const bool ok = hc12SendPairRequest(peerName, peerKey);
            uiStatusLine = ok ? "Radio pair request sent" : "Radio pair request failed";
        } else if (sub == 2) {
            chatScheduleOpenPeerConversation(peerKey);
            return;
        } else if (sub == 3) {
            const int pidx = p2pFindPeerByPubKeyHex(peerKey);
            if (pidx >= 0) {
                const String removedKey = p2pPeers[pidx].pubKeyHex;

                for (int i = pidx + 1; i < p2pPeerCount; ++i) {
                    p2pPeers[i - 1] = p2pPeers[i];
                }
                p2pPeerCount--;
                saveP2pConfig();
                chatRemovePendingForPeer(removedKey);

                hc12RestartWithCurrentPins("Radio restarted after unpair");
                hc12LastDiscoveryAnnounceMs = 0;
                p2pPairRequestPending = false;

                if (currentChatPeerKey == removedKey) {
                    chatSelectFirstEnabledPeer();
                    chatReloadRecentMessagesFromSd(currentChatPeerKey);
                }

                uiStatusLine = "Radio peer unpaired";
            }
        }

        lvglSyncStatusLine();
        lvglRefreshChatPeerUi();
        lvglRefreshChatContactsUi();
        if (uiScreen == UI_CHAT) lvglRefreshChatUi();
        return;
    }

    if (action >= 1000) {
        const int idx = (action / 10) - 100;
        const int sub = action % 10;
        if (idx < 0 || idx >= p2pPeerCount) return;

        if (sub == 2) {
            p2pPeers[idx].enabled = !p2pPeers[idx].enabled;
            saveP2pConfig();
            const int didx = p2pFindDiscoveredByPubKeyHex(p2pPeers[idx].pubKeyHex);
            if (didx >= 0) p2pDiscoveredPeers[didx].trusted = p2pPeers[idx].enabled;
            if (!p2pPeers[idx].enabled && currentChatPeerKey == p2pPeers[idx].pubKeyHex) {
                chatSelectFirstEnabledPeer();
                chatReloadRecentMessagesFromSd(currentChatPeerKey);
            }
            uiStatusLine = p2pPeers[idx].enabled ? "Peer enabled" : "Peer disabled";
        } else if (sub == 3) {
            const String removedKey = p2pPeers[idx].pubKeyHex;
            for (int i = idx + 1; i < p2pPeerCount; ++i) p2pPeers[i - 1] = p2pPeers[i];
            p2pPeerCount--;
            saveP2pConfig();
            const int didx = p2pFindDiscoveredByPubKeyHex(removedKey);
            if (didx >= 0) p2pDiscoveredPeers[didx].trusted = false;
            if (currentChatPeerKey == removedKey) {
                chatSelectFirstEnabledPeer();
                chatReloadRecentMessagesFromSd(currentChatPeerKey);
            }
            uiStatusLine = "Peer unpaired";
        }

        lvglSyncStatusLine();
        lvglRefreshChatPeerUi();
        lvglRefreshChatContactsUi();
        if (uiScreen == UI_CHAT) lvglRefreshChatUi();
        return;
    }

    const int idx = (action / 10) - 1;
    const int sub = action % 10;
    if (idx < 0 || idx >= p2pDiscoveredCount) return;

    if (sub == 1) {
        const bool ok = p2pSendPairRequest(
            p2pDiscoveredPeers[idx].name.isEmpty() ? String("Peer") : p2pDiscoveredPeers[idx].name,
            p2pDiscoveredPeers[idx].pubKeyHex,
            p2pDiscoveredPeers[idx].ip,
            p2pDiscoveredPeers[idx].port);
        uiStatusLine = ok ? "Pair request sent" : "Pair request failed";
    } else if (sub == 2) {
        const int pidx = p2pFindPeerByPubKeyHex(p2pDiscoveredPeers[idx].pubKeyHex);
        if (pidx >= 0) {
            p2pPeers[pidx].enabled = !p2pPeers[pidx].enabled;
            saveP2pConfig();
            p2pDiscoveredPeers[idx].trusted = p2pPeers[pidx].enabled;
            if (!p2pPeers[pidx].enabled && currentChatPeerKey == p2pPeers[pidx].pubKeyHex) {
                chatSelectFirstEnabledPeer();
                chatReloadRecentMessagesFromSd(currentChatPeerKey);
            }
            uiStatusLine = p2pPeers[pidx].enabled ? "Peer enabled" : "Peer disabled";
        }
    } else if (sub == 3) {
        const int pidx = p2pFindPeerByPubKeyHex(p2pDiscoveredPeers[idx].pubKeyHex);
        if (pidx >= 0) {
            const String removedKey = p2pPeers[pidx].pubKeyHex;

            for (int i = pidx + 1; i < p2pPeerCount; ++i) {
                p2pPeers[i - 1] = p2pPeers[i];
            }
            p2pPeerCount--;
            saveP2pConfig();
            chatRemovePendingForPeer(removedKey);

            if (currentChatPeerKey == removedKey) {
                chatSelectFirstEnabledPeer();
                chatReloadRecentMessagesFromSd(currentChatPeerKey);
            }
            uiStatusLine = "Peer removed";
        }
    }

    lvglSyncStatusLine();
    lvglRefreshChatPeerUi();
    lvglRefreshChatContactsUi();
    if (uiScreen == UI_CHAT) lvglRefreshChatUi();
}

void lvglRefreshChatPeerUi()
{
    const unsigned long nowMs = millis();
    const int MAX_RENDER = 12;

    if (lvglChatDiscoveryBtn) {
        lv_obj_t *label = lv_obj_get_child(lvglChatDiscoveryBtn, 0);
        if (label) lv_label_set_text(label, lvglSymbolText(LV_SYMBOL_WIFI, tr(TXT_DISCOVERY)).c_str());
        lvglApplyChatDiscoveryButtonStyle();
    }

    if (lvglChatPeerScanBtn) {
        lv_obj_t *scanLabel = lv_obj_get_child(lvglChatPeerScanBtn, 0);
        if (scanLabel && lvglChatPeerScanRevertAtMs == 0) {
            lv_label_set_text(scanLabel, lvglSymbolText(LV_SYMBOL_REFRESH, tr(TXT_SCAN)).c_str());
        }
    }

    if (lvglChatPeerIdentityLabel) {
        const String pub = p2pPublicKeyHex();
        String shortPub = pub;
        if (shortPub.length() > 20) shortPub = shortPub.substring(0, 20) + "...";
        lv_label_set_text_fmt(lvglChatPeerIdentityLabel, "Device: %s\nKey: %s", deviceShortNameValue().c_str(), shortPub.c_str());
    }

    if (!lvglChatPeerList) return;
    lv_obj_clean(lvglChatPeerList);

    auto makeSmallBtnLocal = [](lv_obj_t *parent, const char *txt, int w, int h, lv_color_t col, lv_event_cb_t cb, void *ud = nullptr) -> lv_obj_t * {
        lv_obj_t *b = lv_btn_create(parent);
        if (!b) return nullptr;
        lv_obj_set_size(b, w, h);
        lv_obj_set_style_radius(b, 8, 0);
        lv_obj_set_style_border_width(b, 0, 0);
        lv_obj_add_event_cb(b, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
        lv_obj_t *l = lv_label_create(b);
        if (l) {
            lv_label_set_text(l, txt);
            lv_obj_center(l);
        }
        lvglRegisterStyledButton(b, col, true);
        return b;
    };

    bool anyEntries = false;

    if (p2pPeerCount > 0) {
        lv_obj_t *section = lv_label_create(lvglChatPeerList);
        lv_obj_set_width(section, lv_pct(100));
        lv_label_set_text(section, "Paired Devices");
        lv_obj_set_style_text_color(section, lv_color_hex(0xE5ECF3), 0);
        lvglRegisterReorderableItem(section, "ord_cpr", "sec_p");

        for (int i = 0; i < p2pPeerCount; ++i) {
            const bool enabled = p2pPeers[i].enabled;
            anyEntries = true;

            lv_obj_t *card = lv_obj_create(lvglChatPeerList);
            lv_obj_set_width(card, lv_pct(100));
            lv_obj_set_height(card, LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(card, lv_color_hex(0x16212C), 0);
            lv_obj_set_style_border_width(card, 0, 0);
            lv_obj_set_style_radius(card, 12, 0);
            lv_obj_set_style_pad_all(card, 8, 0);
            lv_obj_set_style_pad_row(card, 4, 0);
            lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

            const String orderKey = lvglOrderTokenFromText("p", p2pPeers[i].pubKeyHex);
            lvglRegisterReorderableItem(card, "ord_cpr", orderKey.c_str());

            lv_obj_t *title = lv_label_create(card);
            lv_label_set_text_fmt(title, "%s  [%s]",
                                  p2pPeers[i].name.isEmpty() ? "Peer" : p2pPeers[i].name.c_str(),
                                  enabled ? "paired" : "disabled");
            lv_obj_set_style_text_color(title, lv_color_hex(0xE5ECF3), 0);

            lv_obj_t *info = lv_label_create(card);
            lv_obj_set_width(info, lv_pct(100));
            lv_label_set_long_mode(info, LV_LABEL_LONG_WRAP);
            String key = p2pPeers[i].pubKeyHex;
            if (key.length() > 16) key = key.substring(0, 16) + "...";
            lv_label_set_text_fmt(info, "%s:%u\n%s",
                                  p2pPeers[i].ip.toString().c_str(),
                                  static_cast<unsigned int>(p2pPeers[i].port),
                                  key.c_str());
            lv_obj_set_style_text_color(info, lv_color_hex(0xB7C4D1), 0);

            lv_obj_t *row = lv_obj_create(card);
            lv_obj_set_size(row, lv_pct(100), 30);
            lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(row, 0, 0);
            lv_obj_set_style_pad_all(row, 0, 0);
            lv_obj_set_style_pad_column(row, 6, 0);
            lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

            makeSmallBtnLocal(row, enabled ? "Disable" : "Enable", 64, 26, lv_color_hex(0x2F6D86), lvglChatPeerActionEvent,
                              reinterpret_cast<void *>(static_cast<intptr_t>(((i + 100) * 10) + 2)));
            makeSmallBtnLocal(row, "Unpair", 58, 26, lv_color_hex(0x8A3A3A), lvglChatPeerActionEvent,
                              reinterpret_cast<void *>(static_cast<intptr_t>(((i + 100) * 10) + 3)));
        }
    }

    bool anyDiscovered = false;
    for (int i = 0; i < p2pDiscoveredCount; ++i) {
        if (p2pFindPeerByPubKeyHex(p2pDiscoveredPeers[i].pubKeyHex) >= 0) continue;
        anyDiscovered = true;
        break;
    }

    if (anyDiscovered) {
        lv_obj_t *section = lv_label_create(lvglChatPeerList);
        lv_obj_set_width(section, lv_pct(100));
        lv_label_set_text(section, "Discovered");
        lv_obj_set_style_text_color(section, lv_color_hex(0xE5ECF3), 0);
        lvglRegisterReorderableItem(section, "ord_cpr", "sec_d");
    }

    for (int i = 0; i < p2pDiscoveredCount; ++i) {
        const int pidx = p2pFindPeerByPubKeyHex(p2pDiscoveredPeers[i].pubKeyHex);
        if (pidx >= 0) continue;
        anyEntries = true;

        lv_obj_t *card = lv_obj_create(lvglChatPeerList);
        lv_obj_set_width(card, lv_pct(100));
        lv_obj_set_height(card, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_color(card, lv_color_hex(0x16212C), 0);
        lv_obj_set_style_border_width(card, 0, 0);
        lv_obj_set_style_radius(card, 12, 0);
        lv_obj_set_style_pad_all(card, 8, 0);
        lv_obj_set_style_pad_row(card, 4, 0);
        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

        const String orderKey = lvglOrderTokenFromText("d", p2pDiscoveredPeers[i].pubKeyHex);
        lvglRegisterReorderableItem(card, "ord_cpr", orderKey.c_str());

        lv_obj_t *title = lv_label_create(card);
        lv_label_set_text_fmt(title, "%s - over WiFi", p2pDiscoveredPeers[i].name.c_str());
        lv_obj_set_style_text_color(title, lv_color_hex(0xE5ECF3), 0);

        lv_obj_t *info = lv_label_create(card);
        lv_obj_set_width(info, lv_pct(100));
        lv_label_set_long_mode(info, LV_LABEL_LONG_WRAP);
        String key = p2pDiscoveredPeers[i].pubKeyHex;
        if (key.length() > 16) key = key.substring(0, 16) + "...";
        lv_label_set_text_fmt(info, "%s:%u\n%s",
                              p2pDiscoveredPeers[i].ip.toString().c_str(),
                              static_cast<unsigned int>(p2pDiscoveredPeers[i].port),
                              key.c_str());
        lv_obj_set_style_text_color(info, lv_color_hex(0xB7C4D1), 0);

        lv_obj_t *row = lv_obj_create(card);
        lv_obj_set_size(row, lv_pct(100), 30);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_set_style_pad_column(row, 6, 0);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        makeSmallBtnLocal(row, "Pair", 54, 26, lv_color_hex(0x2F6D86), lvglChatPeerActionEvent,
                          reinterpret_cast<void *>(static_cast<intptr_t>(((i + 1) * 10) + 1)));
    }

    bool anyRadioDiscovered = false;
    for (int i = 0; i < hc12DiscoveredCount; ++i) {
        if (p2pFindPeerByPubKeyHex(hc12DiscoveredPeers[i].pubKeyHex) >= 0) continue;
        anyRadioDiscovered = true;
        break;
    }

    if (anyRadioDiscovered) {
        lv_obj_t *section = lv_label_create(lvglChatPeerList);
        lv_obj_set_width(section, lv_pct(100));
        lv_label_set_text(section, "HC-12 Radio");
        lv_obj_set_style_text_color(section, lv_color_hex(0xE5ECF3), 0);
        lvglRegisterReorderableItem(section, "ord_cpr", "sec_r");
    }

    for (int i = 0; i < hc12DiscoveredCount; ++i) {
        if (p2pFindPeerByPubKeyHex(hc12DiscoveredPeers[i].pubKeyHex) >= 0) continue;
        anyEntries = true;

        lv_obj_t *card = lv_obj_create(lvglChatPeerList);
        lv_obj_set_width(card, lv_pct(100));
        lv_obj_set_height(card, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_color(card, lv_color_hex(0x16212C), 0);
        lv_obj_set_style_border_width(card, 0, 0);
        lv_obj_set_style_radius(card, 12, 0);
        lv_obj_set_style_pad_all(card, 8, 0);
        lv_obj_set_style_pad_row(card, 4, 0);
        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

        const String orderKey = lvglOrderTokenFromText("r", hc12DiscoveredPeers[i].pubKeyHex);
        lvglRegisterReorderableItem(card, "ord_cpr", orderKey.c_str());

        lv_obj_t *title = lv_label_create(card);
        lv_label_set_text_fmt(title, "%s - over Radio",
                              hc12DiscoveredPeers[i].name.isEmpty() ? "Radio Peer" : hc12DiscoveredPeers[i].name.c_str());
        lv_obj_set_style_text_color(title, lv_color_hex(0xE5ECF3), 0);

        lv_obj_t *info = lv_label_create(card);
        lv_obj_set_width(info, lv_pct(100));
        lv_label_set_long_mode(info, LV_LABEL_LONG_WRAP);
        String key = hc12DiscoveredPeers[i].pubKeyHex;
        if (key.length() > 16) key = key.substring(0, 16) + "...";
        const unsigned long ageSec = static_cast<unsigned long>((millis() - hc12DiscoveredPeers[i].lastSeenMs) / 1000UL);
        lv_label_set_text_fmt(info, "HC-12 direct radio\nSeen %lus ago\n%s", ageSec, key.c_str());
        lv_obj_set_style_text_color(info, lv_color_hex(0xB7C4D1), 0);

        lv_obj_t *row = lv_obj_create(card);
        lv_obj_set_size(row, lv_pct(100), 30);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_set_style_pad_column(row, 6, 0);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        const bool trusted = p2pFindPeerByPubKeyHex(hc12DiscoveredPeers[i].pubKeyHex) >= 0;

        if (!trusted) {
            makeSmallBtnLocal(row, "Pair", 54, 26, lv_color_hex(0x2E7A4A), lvglChatPeerActionEvent,
                              reinterpret_cast<void *>(static_cast<intptr_t>(20000 + i * 10 + 1)));
        }

        makeSmallBtnLocal(row, "Open", 54, 26, lv_color_hex(0x7A5C2E), lvglChatPeerActionEvent,
                          reinterpret_cast<void *>(static_cast<intptr_t>(20000 + i * 10 + 2)));

        if (trusted) {
            makeSmallBtnLocal(row, "Unpair", 58, 26, lv_color_hex(0x8A3A3A), lvglChatPeerActionEvent,
                              reinterpret_cast<void *>(static_cast<intptr_t>(20000 + i * 10 + 3)));
        }
    }

    bool anyMqttDiscovered = false;
    for (int i = 0; i < mqttDiscoveredCount; ++i) {
        const int existing = p2pFindPeerByPubKeyHex(mqttDiscoveredPeers[i].pubKeyHex);
        if (existing >= 0) continue;
        anyMqttDiscovered = true;
        break;
    }

    if (anyMqttDiscovered) {
        lv_obj_t *section = lv_label_create(lvglChatPeerList);
        lv_obj_set_width(section, lv_pct(100));
        lv_label_set_text(section, "MQTT");
        lv_obj_set_style_text_color(section, lv_color_hex(0xE5ECF3), 0);
        lvglRegisterReorderableItem(section, "ord_cpr", "sec_m");
    }

    for (int i = 0; i < mqttDiscoveredCount && i < MAX_RENDER; ++i) {
        const int existing = p2pFindPeerByPubKeyHex(mqttDiscoveredPeers[i].pubKeyHex);
        if (existing >= 0) continue;
        anyEntries = true;

        lv_obj_t *card = lv_obj_create(lvglChatPeerList);
        lv_obj_set_width(card, lv_pct(100));
        lv_obj_set_height(card, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_color(card, lv_color_hex(0x16212C), 0);
        lv_obj_set_style_border_width(card, 0, 0);
        lv_obj_set_style_radius(card, 12, 0);
        lv_obj_set_style_pad_all(card, 8, 0);
        lv_obj_set_style_pad_row(card, 4, 0);
        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

        const String orderKey = lvglOrderTokenFromText("m", mqttDiscoveredPeers[i].pubKeyHex);
        lvglRegisterReorderableItem(card, "ord_cpr", orderKey.c_str());

        lv_obj_t *title = lv_label_create(card);
        lv_label_set_text_fmt(title, "%s - over MQTT",
                              mqttDiscoveredPeers[i].name.isEmpty() ? "MQTT Peer" : mqttDiscoveredPeers[i].name.c_str());
        lv_obj_set_style_text_color(title, lv_color_hex(0xE5ECF3), 0);

        lv_obj_t *info = lv_label_create(card);
        lv_obj_set_width(info, lv_pct(100));
        lv_label_set_long_mode(info, LV_LABEL_LONG_WRAP);
        String key = mqttDiscoveredPeers[i].pubKeyHex;
        if (key.length() > 16) key = key.substring(0, 16) + "...";
        const unsigned long ageSec = static_cast<unsigned long>((nowMs - mqttDiscoveredPeers[i].lastSeenMs) / 1000UL);
        lv_label_set_text_fmt(info, "MQTT peer presence\nSeen %lus ago\n%s", ageSec, key.c_str());
        lv_obj_set_style_text_color(info, lv_color_hex(0xB7C4D1), 0);

        lv_obj_t *row = lv_obj_create(card);
        lv_obj_set_size(row, lv_pct(100), 30);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_set_style_pad_column(row, 6, 0);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        const bool trusted = p2pFindPeerByPubKeyHex(mqttDiscoveredPeers[i].pubKeyHex) >= 0;

        if (!trusted) {
            makeSmallBtnLocal(row, "Pair", 54, 26, lv_color_hex(0x2E7A4A), lvglChatPeerActionEvent,
                              reinterpret_cast<void *>(static_cast<intptr_t>(30000 + i * 10 + 1)));
        }

        makeSmallBtnLocal(row, "Open", 54, 26, lv_color_hex(0x7A5C2E), lvglChatPeerActionEvent,
                          reinterpret_cast<void *>(static_cast<intptr_t>(30000 + i * 10 + 2)));

        if (trusted) {
            makeSmallBtnLocal(row, "Unpair", 58, 26, lv_color_hex(0x8A3A3A), lvglChatPeerActionEvent,
                              reinterpret_cast<void *>(static_cast<intptr_t>(30000 + i * 10 + 3)));
        }
    }

    if (!anyEntries) {
        lv_obj_t *lbl = lv_label_create(lvglChatPeerList);
        lv_obj_set_width(lbl, lv_pct(100));
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_WRAP);
        lv_label_set_text(lbl, "No paired or discovered peers yet.\nOpen this screen on another device on the same WiFi, same MQTT broker, or same HC-12 channel, then tap Scan.");
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xC8CED6), 0);
        lvglRegisterReorderableItem(lbl, "ord_cpr", "empty");
    }

    lvglApplySavedOrder(lvglChatPeerList, "ord_cpr");
}
