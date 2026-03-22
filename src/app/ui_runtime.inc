void appendUiSettings(JsonDocument &doc)
{
    uiPrefs.begin("ui", true);

    doc["RecordTelemetry"] = uiPrefs.getBool("record_tel", recordTelemetryEnabled) ? 1 : 0;
    doc["SystemSounds"] = uiPrefs.getBool("sys_snd", systemSoundsEnabled) ? 1 : 0;
    doc["SystemVolume"] = uiPrefs.getUChar("sys_vol", mediaVolumePercent);
    doc["VibrationEnabled"] = uiPrefs.getBool("vib_en", vibrationEnabled) ? 1 : 0;
    doc["TouchSoundEnabled"] = uiPrefs.getBool("touch_snd", touchSoundEnabled) ? 1 : 0;
    doc["StartupSoundEnabled"] = uiPrefs.getBool("boot_snd", startupSoundEnabled) ? 1 : 0;
    doc["TouchVibrationEnabled"] = uiPrefs.getBool("touch_vib", touchVibrationEnabled) ? 1 : 0;
    doc["DisplayBrightness"] = uiPrefs.getUChar("disp_bri", displayBrightnessPercent);
    doc["DeviceName"] = uiPrefs.getString("dev_name", deviceShortNameValue());
    doc["WsRebootOnDisconnect"] = uiPrefs.getBool("ws_reboot", wsRebootOnDisconnectEnabled) ? 1 : 0;
    doc["AirplaneMode"] = uiPrefs.getBool("airplane", airplaneModeEnabled) ? 1 : 0;
    doc["ChatDiscoveryEnabled"] = p2pDiscoveryEnabled ? 1 : 0;
    doc["ButtonStyle"] = uiPrefs.getUChar("btn_style", static_cast<uint8_t>(uiButtonStyleMode));
    doc["Menu3DIcons"] = uiPrefs.getBool("menu_3d_i", menuCustomIconsEnabled) ? 1 : 0;
    doc["DisplayLanguage"] = uiPrefs.getUChar("lang", static_cast<uint8_t>(uiLanguage));
    doc["VibrationIntensity"] = uiPrefs.getUChar("vib_int", static_cast<uint8_t>(vibrationIntensity));
    doc["RadioModule"] = uiPrefs.getUChar("radio_mod", static_cast<uint8_t>(radioModuleType));
    doc["Hc12SwapUart"] = uiPrefs.getBool("hc12_swap", hc12SwapUartPins) ? 1 : 0;
    doc["Hc12Channel"] = uiPrefs.getInt("hc12_ch", hc12CurrentChannel);
    doc["Hc12BaudIndex"] = uiPrefs.getInt("hc12_baud", hc12CurrentBaudIndex);
    doc["Hc12ModeIndex"] = uiPrefs.getInt("hc12_mode", hc12CurrentModeIndex);
    doc["Hc12PowerLevel"] = uiPrefs.getInt("hc12_pwr", hc12CurrentPowerLevel);
    doc["E220SwapUart"] = uiPrefs.getBool("e220_swap", e220SwapUartPins) ? 1 : 0;
    doc["E220SwapModePins"] = uiPrefs.getBool("e220_mswap", e220SwapModePins) ? 1 : 0;
    doc["E220Channel"] = uiPrefs.getInt("e220_ch", e220CurrentChannel);
    doc["E220BaudIndex"] = uiPrefs.getInt("e220_baud", e220CurrentBaudIndex);
    doc["E220AirRateIndex"] = uiPrefs.getInt("e220_rate", e220CurrentAirRateIndex);
    doc["E220PowerIndex"] = uiPrefs.getInt("e220_pwr", e220CurrentPowerIndex);
    doc["E220FixedMode"] = uiPrefs.getBool("e220_fix", e220CurrentFixedTransmission) ? 1 : 0;
    doc["TopBarCenter"] = uiPrefs.getUChar("top_mid", static_cast<uint8_t>(topBarCenterMode));
    doc["TopBarTimezone"] = uiPrefs.getInt("tz_gmt", static_cast<int>(topBarTimezoneGmtOffset));
    doc["PowerOffIdleMs"] = uiPrefs.getULong("pwr_idle", powerOffIdleTimeoutMs);
    doc["TelemetryMaxKB"] = uiPrefs.getUInt("tele_kb", telemetryMaxKB);
    doc["IndicatorsVisible"] = uiPrefs.getBool("ind_v", true) ? 1 : 0;
    doc["ImuVisible"] = uiPrefs.getBool("imu_v", true) ? 1 : 0;
    doc["MediaVisible"] = uiPrefs.getBool("med_v", true) ? 1 : 0;
    doc["PathVisible"] = uiPrefs.getBool("path_v", true) ? 1 : 0;
    doc["Model3DVisible"] = uiPrefs.getBool("mod_v", true) ? 1 : 0;
    doc["SerialVisible"] = uiPrefs.getBool("ser_v", true) ? 1 : 0;
    doc["HomeChatVisible"] = homeChatVisible ? 1 : 0;
    doc["HomeGateVisible"] = homeRadioVisible ? 1 : 0;
    doc["HomeMediaVisible"] = homeMediaVisible ? 1 : 0;
    doc["HomeInfoVisible"] = homeInfoVisible ? 1 : 0;
    doc["HomeGamesVisible"] = homeGamesVisible ? 1 : 0;
    doc["HomeMqttVisible"] = homeMqttVisible ? 1 : 0;
    doc["HomeConfigVisible"] = homeConfigVisible ? 1 : 0;
    doc["HomePowerVisible"] = homePowerVisible ? 1 : 0;
    doc["HomeAirplaneVisible"] = homeAirplaneVisible ? 1 : 0;
    doc["HomeApVisible"] = homeApVisible ? 1 : 0;
    doc["ScreenLockEnabled"] = uiPrefs.getBool("scr_lock", screenLockEnabled) ? 1 : 0;

    appendUiWidgetPositionIfPresent(doc, uiPrefs, "IndicatorsX", "IndicatorsY", "ind_x", "ind_y");
    appendUiWidgetPositionIfPresent(doc, uiPrefs, "ImuX", "ImuY", "imu_x", "imu_y");
    appendUiWidgetPositionIfPresent(doc, uiPrefs, "MediaX", "MediaY", "med_x", "med_y");
    appendUiWidgetPositionIfPresent(doc, uiPrefs, "PathX", "PathY", "path_x", "path_y");
    appendUiWidgetPositionIfPresent(doc, uiPrefs, "Model3DX", "Model3DY", "mod_x", "mod_y");
    appendUiWidgetPositionIfPresent(doc, uiPrefs, "SerialX", "SerialY", "ser_x", "ser_y");

    doc["ViewOverlapFx"] = uiPrefs.getBool("ovl_fx", true) ? 1 : 0;
    doc["ViewSnapFx"] = uiPrefs.getBool("snap_fx", true) ? 1 : 0;
    doc["ViewGravityFx"] = uiPrefs.getBool("grav_fx", true) ? 1 : 0;
    doc["ViewGravityStr"] = uiPrefs.getInt("grav_str", 55);
    doc["SerialLogRateMs"] = uiPrefs.getUInt("ser_rate", SERIAL_LOG_RATE_MS_DEFAULT);
    doc["SerialLogKeepLines"] = uiPrefs.getUInt("ser_keep", SERIAL_LOG_RING_SIZE);
    doc["MessageTone"] = uiPrefs.getUChar("msg_tone", static_cast<uint8_t>(messageBeepTone));

    uiPrefs.end();
}

void loadUiRuntimeConfig()
{
    uiPrefs.begin("ui", false);
    recordTelemetryEnabled = uiPrefs.getBool("record_tel", false);
    systemSoundsEnabled = uiPrefs.getBool("sys_snd", true);
    mediaVolumePercent = uiPrefs.getUChar("sys_vol", mediaVolumePercent);
    displayBrightnessPercent = static_cast<uint8_t>(constrain(uiPrefs.getUChar("disp_bri", displayBrightnessPercent), 5, 100));
    rgbLedPercent = static_cast<uint8_t>(constrain(uiPrefs.getUChar("rgb_led", rgbLedPercent), 0, 100));
    deviceShortName = sanitizeDeviceShortName(uiPrefs.getString("dev_name", DEVICE_SHORT_NAME));
    if (deviceShortName.isEmpty()) deviceShortName = DEVICE_SHORT_NAME;
    wsRebootOnDisconnectEnabled = uiPrefs.getBool("ws_reboot", false);
    webServerEnabled = uiPrefs.getBool("web_srv", true);
    airplaneModeEnabled = uiPrefs.getBool("airplane", false);
    menuCustomIconsEnabled = uiPrefs.getBool("menu_3d_i", true);
    uiLanguage = static_cast<UiLanguage>(constrain(uiPrefs.getUChar("lang", static_cast<uint8_t>(UI_LANG_ENGLISH)),
                                                   static_cast<uint8_t>(UI_LANG_ENGLISH),
                                                   static_cast<uint8_t>(UI_LANG_COUNT - 1)));
    vibrationIntensity = static_cast<VibrationIntensity>(constrain(uiPrefs.getUChar("vib_int", static_cast<uint8_t>(VIBRATION_INTENSITY_MEDIUM)),
                                                                   static_cast<uint8_t>(VIBRATION_INTENSITY_LOW),
                                                                   static_cast<uint8_t>(VIBRATION_INTENSITY_HIGH)));
    vibrationEnabled = uiPrefs.getBool("vib_en", true);
    touchSoundEnabled = uiPrefs.getBool("touch_snd", true);
    startupSoundEnabled = uiPrefs.getBool("boot_snd", true);
    touchVibrationEnabled = uiPrefs.getBool("touch_vib", true);
    p2pDiscoveryEnabled = uiPrefs.getBool("chat_disc", p2pDiscoveryEnabled);
    homeChatVisible = uiPrefs.getBool("home_chat", true);
    homeRadioVisible = uiPrefs.getBool("home_gate", true);
    homeMediaVisible = uiPrefs.getBool("home_media", true);
    homeInfoVisible = uiPrefs.getBool("home_info", true);
    homeGamesVisible = uiPrefs.getBool("home_games", true);
    homeMqttVisible = uiPrefs.getBool("home_mqtt", true);
    homeConfigVisible = uiPrefs.getBool("home_cfg", true);
    homePowerVisible = uiPrefs.getBool("home_power", true);
    homeAirplaneVisible = uiPrefs.getBool("home_air", true);
    homeApVisible = uiPrefs.getBool("home_ap", true);
    screenLockEnabled = uiPrefs.getBool("scr_lock", false);
    copyTextToBuf(screenLockPin, sizeof(screenLockPin), uiPrefs.getString("scr_pin", ""));
    if (strlen(screenLockPin) != 4) {
        screenLockEnabled = false;
        screenLockPin[0] = '\0';
    }
    screenLockUnlocked = !screenLockEnabled;
    screenLockFailedTotal = 0;
    screenLockBlockedUntilMs = 0;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    radioModuleType = static_cast<RadioModuleType>(constrain(uiPrefs.getUChar("radio_mod", static_cast<uint8_t>(RADIO_MODULE_HC12)),
                                                             static_cast<uint8_t>(RADIO_MODULE_HC12),
                                                             static_cast<uint8_t>(RADIO_MODULE_COUNT - 1)));
#else
    radioModuleType = RADIO_MODULE_HC12;
#endif
    loadPersistedRadioSettings();
    loadPersistedRemoteControlSettings();
    loadPersistedRadioControlButtons();
    gateRollingCount = static_cast<uint16_t>(constrain(uiPrefs.getUInt("gate_cnt", GATE_COUNT_INITIAL), 1U, 65535U));
    messageBeepTone = static_cast<MessageBeepTone>(constrain(uiPrefs.getUChar("msg_tone", static_cast<uint8_t>(MESSAGE_BEEP_DOUBLE_SHORT)),
                                                             static_cast<uint8_t>(MESSAGE_BEEP_SINGLE),
                                                             static_cast<uint8_t>(MESSAGE_BEEP_TONE_COUNT - 1)));
    const uint8_t rawButtonStyle = uiPrefs.getUChar("btn_style", static_cast<uint8_t>(UI_BUTTON_STYLE_3D));
    if (rawButtonStyle == static_cast<uint8_t>(UI_BUTTON_STYLE_FLAT)) uiButtonStyleMode = UI_BUTTON_STYLE_FLAT;
    else if (rawButtonStyle == static_cast<uint8_t>(UI_BUTTON_STYLE_BLACK)) uiButtonStyleMode = UI_BUTTON_STYLE_BLACK;
    else uiButtonStyleMode = UI_BUTTON_STYLE_3D;
    topBarCenterMode = uiPrefs.getUChar("top_mid", static_cast<uint8_t>(TOP_BAR_CENTER_NAME)) == static_cast<uint8_t>(TOP_BAR_CENTER_TIME)
                           ? TOP_BAR_CENTER_TIME
                           : TOP_BAR_CENTER_NAME;
    topBarTimezoneGmtOffset = static_cast<int8_t>(constrain(uiPrefs.getInt("tz_gmt", 0), static_cast<int>(TOP_BAR_GMT_MIN), static_cast<int>(TOP_BAR_GMT_MAX)));
    uiButtonStyleFlatSelectorColor = uiPrefs.getUInt("btn_flat_c", 0);
    uiButtonStyle3dSelectorColor = uiPrefs.getUInt("btn_3d_c", 0);
    if (uiButtonStyleFlatSelectorColor == 0) {
        uiButtonStyleFlatSelectorColor = lvglRandomStyleAccent(0x2E6F95);
        uiPrefs.putUInt("btn_flat_c", uiButtonStyleFlatSelectorColor);
    }
    if (uiButtonStyle3dSelectorColor == 0) {
        uiButtonStyle3dSelectorColor = lvglRandomStyleAccent(uiButtonStyleFlatSelectorColor);
        uiPrefs.putUInt("btn_3d_c", uiButtonStyle3dSelectorColor);
    }
    telemetryMaxKB = uiPrefs.getUInt("tele_kb", 512);
    serialLogWsMinIntervalMs = uiPrefs.getUInt("ser_rate", SERIAL_LOG_RATE_MS_DEFAULT);
    serialLogKeepLines = static_cast<size_t>(uiPrefs.getUInt("ser_keep", SERIAL_LOG_RING_SIZE));
    screensaverEnabled = uiPrefs.getBool("scrsvr_en", false);
    displayIdleTimeoutMs = clampIdleTimeoutMs(uiPrefs.getULong("disp_idle", LCD_IDLE_TIMEOUT_MS_DEFAULT));
    powerOffIdleTimeoutMs = POWER_OFF_IDLE_TIMEOUT_OPTIONS_MS[powerOffTimeoutOptionIndex(uiPrefs.getULong("pwr_idle", 0UL))];
    applyDisplayIdleTimeoutPowerOffCap(true);
    copyTextToBuf(otaPendingPopupVersion, sizeof(otaPendingPopupVersion), uiPrefs.getString("ota_popup", ""));
    otaPendingPopupNotes = uiPrefs.getString("ota_popup_notes", "");
    copyTextToBuf(otaLatestVersion, sizeof(otaLatestVersion), otaNormalizedVersion(uiPrefs.getString("ota_latest", "")));
    otaUpdateAvailable = uiPrefs.getBool("ota_avail", false);
    uiPrefs.end();
    if (!OTA_FIRMWARE_FLASH_SUPPORTED) {
        otaLatestVersion[0] = '\0';
        otaUpdateAvailable = false;
    }
    if (otaLatestVersion[0] != '\0' && otaCompareVersions(String(FW_VERSION), String(otaLatestVersion)) >= 0) {
        otaUpdateAvailable = false;
    }
    otaUpdatePromptPending = otaUpdateAvailable;
    otaPostUpdatePopupPending = otaPendingPopupVersion[0] != '\0' && otaNormalizedVersion(String(otaPendingPopupVersion)) == otaNormalizedVersion(String(FW_VERSION));
    audioSetVolumeImmediate(audioVolumeLevelFromPercent(mediaVolumePercent));
}

static void otaStorePopupVersion(const String &version, const String &notes = "")
{
    copyTextToBuf(otaPendingPopupVersion, sizeof(otaPendingPopupVersion), version);
    otaPendingPopupNotes = notes;
    uiPrefs.begin("ui", false);
    uiPrefs.putString("ota_popup", version);
    uiPrefs.putString("ota_popup_notes", otaPendingPopupNotes);
    uiPrefs.end();
    otaPostUpdatePopupPending = otaNormalizedVersion(version) == otaNormalizedVersion(String(FW_VERSION));
}

static void otaPersistAvailabilityState()
{
    uiPrefs.begin("ui", false);
    uiPrefs.putBool("ota_avail", otaUpdateAvailable);
    uiPrefs.putString("ota_latest", otaLatestVersion);
    uiPrefs.end();
}

static void otaClearPopupVersion()
{
    otaPendingPopupVersion[0] = '\0';
    otaPendingPopupNotes = "";
    otaPostUpdatePopupPending = false;
    uiPrefs.begin("ui", false);
    uiPrefs.remove("ota_popup");
    uiPrefs.remove("ota_popup_notes");
    uiPrefs.end();
}

static void otaCheckTask(void *param)
{
    (void)param;
    otaUiState = OTA_UI_CHECKING;
    otaSetStatus("Checking for updates...");
    String latestTag;
    String latestUrl;
    String error;
    const bool ok = otaFetchLatestReleaseInfo(latestTag, latestUrl, error);
    if (ok) {
        copyTextToBuf(otaLatestVersion, sizeof(otaLatestVersion), otaNormalizedVersion(latestTag));
        otaLatestBinUrl = latestUrl;
        otaUpdateAvailable = otaCompareVersions(String(FW_VERSION), String(otaLatestVersion)) < 0;
        otaUpdatePromptPending = otaUpdateAvailable;
        otaPersistAvailabilityState();
        otaUiState = otaUpdateAvailable ? OTA_UI_AVAILABLE : OTA_UI_UP_TO_DATE;
        otaSetStatus(otaUpdateAvailable ? "Update available" : "You are up to date");
        otaLastCheckMs = millis();
        otaNextAllowedCheckMs = millis() + OTA_CHECK_INTERVAL_MS;
    } else {
        otaUiState = OTA_UI_ERROR;
        otaSetStatus("Update check failed");
        otaNextAllowedCheckMs = millis() + OTA_RETRY_DELAY_MS;
        otaUpdatePromptPending = false;
    }
    if (lvglReady) {
        lvglRefreshTopIndicators();
        if (uiScreen == UI_CONFIG_OTA) lvglRefreshOtaUi();
    }
    otaCheckTaskHandle = nullptr;
    vTaskDelete(nullptr);
}

static void otaUpdateTask(void *param)
{
    (void)param;
    const String binUrl = otaLatestBinUrl;
    const String targetVersion = String(otaLatestVersion);
    String error;
    otaUiState = OTA_UI_DOWNLOADING;
    otaProgressPercent = 0;
    otaUpdatePromptPending = false;
    otaUpdateAvailable = false;
    otaPersistAvailabilityState();
    otaSetStatus("Starting update...");
    const bool ok = otaDownloadAndApplyFromUrl(binUrl, error);
    if (ok) {
        otaUiState = OTA_UI_DONE;
        otaSetStatus("Update complete. Rebooting...");
        if (targetVersion.length()) otaStorePopupVersion(targetVersion, otaPendingPopupNotes);
        rebootRequested = true;
        rebootRequestedAtMs = millis() + 300UL;
    } else {
        otaUiState = OTA_UI_ERROR;
        otaSetStatus("Update failed");
        uiStatusLine = "OTA failed: " + error;
        lvglSyncStatusLine();
    }
    if (lvglReady) {
        lvglRefreshTopIndicators();
        if (uiScreen == UI_CONFIG_OTA) lvglRefreshOtaUi();
    }
    otaUpdateTaskHandle = nullptr;
    vTaskDelete(nullptr);
}

void otaCheckService()
{
    if (!OTA_FIRMWARE_FLASH_SUPPORTED) return;
    if (otaCheckTaskHandle || otaUpdateTaskHandle) return;
    if (!wifiConnectedSafe()) return;
    const unsigned long now = millis();
    const bool due = otaCheckRequested ||
                     (otaBootCheckPending && now >= OTA_INITIAL_CHECK_DELAY_MS) ||
                     (otaNextAllowedCheckMs != 0 && static_cast<long>(now - otaNextAllowedCheckMs) >= 0);
    if (!due) return;
    otaCheckRequested = false;
    otaBootCheckPending = false;
    if (xTaskCreatePinnedToCore(otaCheckTask, "ota_check", 8192, nullptr, 1, &otaCheckTaskHandle, ARDUINO_RUNNING_CORE) != pdPASS) {
        otaCheckTaskHandle = nullptr;
        otaUiState = OTA_UI_ERROR;
        otaSetStatus("Check start failed");
        otaNextAllowedCheckMs = now + OTA_RETRY_DELAY_MS;
    }
}

void otaUpdateService()
{
    if (!lvglReady && !rebootRequested) return;
    if (uiScreen == UI_CONFIG_OTA) lvglRefreshOtaUi();
}

bool handleUiSettingMessage(const char *msg)
{
    if (!msg || !*msg) return false;
    const char *comma = strchr(msg, ',');
    if (!comma || comma == msg || !comma[1]) return false;

    const size_t keyLen = static_cast<size_t>(comma - msg);
    if (keyLen == 0 || keyLen >= 32) return false;

    char key[32];
    memcpy(key, msg, keyLen);
    key[keyLen] = '\0';

    const char *value = comma + 1;
    int parsed = 0;

    uiPrefs.begin("ui", false);
    bool handled = true;

    if (strcmp(key, "RecordTelemetry") == 0) {
        recordTelemetryEnabled = atoi(value) != 0;
        uiPrefs.putBool("record_tel", recordTelemetryEnabled);
    }
    else if (strcmp(key, "SystemSounds") == 0) {
        systemSoundsEnabled = atoi(value) != 0;
        uiPrefs.putBool("sys_snd", systemSoundsEnabled);
    }
    else if (strcmp(key, "SystemVolume") == 0 && parseIntMessageValue(value, parsed)) {
        mediaVolumePercent = static_cast<uint8_t>(max(0, min(100, parsed)));
        uiPrefs.putUChar("sys_vol", mediaVolumePercent);
        audioSetVolumeImmediate(audioVolumeLevelFromPercent(mediaVolumePercent));
        lvglRefreshMediaPlayerUi();
        lvglRefreshConfigUi();
        lvglRefreshSoundPopupUi();
        lvglTopIndicatorStateValid = false;
        if (lvglReady) lvglRefreshTopIndicators();
    }
    else if (strcmp(key, "DisplayBrightness") == 0 && parseIntMessageValue(value, parsed)) {
        displayBrightnessPercent = static_cast<uint8_t>(max(5, min(100, parsed)));
        uiPrefs.putUChar("disp_bri", displayBrightnessPercent);
        if (displayAwake) displayBacklightSet(displayBacklightLevelFromPercent(displayBrightnessPercent));
        lvglRefreshConfigUi();
    }
    else if (strcmp(key, "DeviceName") == 0) {
        deviceShortName = sanitizeDeviceShortName(String(value));
        if (deviceShortName.isEmpty()) deviceShortName = DEVICE_SHORT_NAME;
        uiPrefs.putString("dev_name", deviceShortName);
        lvglTopIndicatorStateValid = false;
        if (lvglReady) lvglRefreshTopIndicators();
        lvglRefreshConfigUi();
    }
    else if (strcmp(key, "WsRebootOnDisconnect") == 0) {
        wsRebootOnDisconnectEnabled = atoi(value) != 0;
        uiPrefs.putBool("ws_reboot", wsRebootOnDisconnectEnabled);
    }
    else if (strcmp(key, "AirplaneMode") == 0) {
        airplaneModeEnabled = atoi(value) != 0;
        uiPrefs.end();
        applyAirplaneMode(airplaneModeEnabled, "ws_setting");
        return true;
    }
    else if (strcmp(key, "ChatDiscoveryEnabled") == 0) {
        p2pDiscoveryEnabled = atoi(value) != 0;
        uiPrefs.putBool("chat_disc", p2pDiscoveryEnabled);
        uiPrefs.end();
        saveP2pConfig();
        if (!p2pDiscoveryEnabled) {
            hc12LastDiscoveryAnnounceMs = millis();
            p2pLastDiscoverAnnounceMs = millis();
        } else {
            hc12LastDiscoveryAnnounceMs = 0;
            p2pLastDiscoverAnnounceMs = 0;
        }
        if (lvglReady) {
            lvglApplyChatDiscoveryButtonStyle();
            if (uiScreen == UI_CHAT_PEERS) lvglRefreshChatPeerUi();
        }
        return true;
    }
    else if (strcmp(key, "ButtonStyle") == 0 && parseIntMessageValue(value, parsed)) {
        if (parsed == static_cast<int>(UI_BUTTON_STYLE_FLAT)) uiButtonStyleMode = UI_BUTTON_STYLE_FLAT;
        else if (parsed == static_cast<int>(UI_BUTTON_STYLE_BLACK)) uiButtonStyleMode = UI_BUTTON_STYLE_BLACK;
        else uiButtonStyleMode = UI_BUTTON_STYLE_3D;
        uiPrefs.putUChar("btn_style", static_cast<uint8_t>(uiButtonStyleMode));
        if (lvglReady) {
            lvglRefreshAllButtonStyles();
            lvglRefreshStyleUi();
        }
    }
    else if (strcmp(key, "Menu3DIcons") == 0) {
        menuCustomIconsEnabled = atoi(value) != 0;
        uiPrefs.putBool("menu_3d_i", menuCustomIconsEnabled);
        if (lvglReady) {
            lvglRefreshPrimaryMenuButtonIcons();
            lvglRefreshConfigUi();
            lvglRefreshStyleUi();
        }
    }
    else if (strcmp(key, "DisplayLanguage") == 0 && parseIntMessageValue(value, parsed)) {
        uiLanguage = static_cast<UiLanguage>(constrain(parsed, static_cast<int>(UI_LANG_ENGLISH), static_cast<int>(UI_LANG_COUNT - 1)));
        uiPrefs.putUChar("lang", static_cast<uint8_t>(uiLanguage));
        if (lvglReady) lvglRefreshLocalizedUi();
    }
    else if (strcmp(key, "RadioModule") == 0 && parseIntMessageValue(value, parsed)) {
#if defined(BOARD_ESP32S3_3248S035_N16R8)
        radioModuleType = static_cast<RadioModuleType>(constrain(parsed,
                                                                 static_cast<int>(RADIO_MODULE_HC12),
                                                                 static_cast<int>(RADIO_MODULE_COUNT - 1)));
#else
        radioModuleType = RADIO_MODULE_HC12;
#endif
        uiPrefs.putUChar("radio_mod", static_cast<uint8_t>(radioModuleType));
        if (lvglReady) {
            lvglRefreshConfigUi();
            lvglRefreshHc12ConfigUi();
            lvglRefreshHc12Ui();
            lvglRefreshHc12InfoUi();
        }
    }
    else if (strcmp(key, "VibrationEnabled") == 0) {
        vibrationEnabled = atoi(value) != 0;
        uiPrefs.putBool("vib_en", vibrationEnabled);
        if (!vibrationEnabled) {
#if defined(BOARD_ESP32S3_3248S035_N16R8)
            chatMessageVibrationStop(true);
#endif
        }
        if (lvglReady) {
            lvglRefreshConfigUi();
            lvglRefreshSoundPopupUi();
            lvglTopIndicatorStateValid = false;
            lvglRefreshTopIndicators();
        }
    }
    else if (strcmp(key, "TouchSoundEnabled") == 0) {
        touchSoundEnabled = atoi(value) != 0;
        uiPrefs.putBool("touch_snd", touchSoundEnabled);
        if (lvglReady) lvglRefreshStyleUi();
    }
    else if (strcmp(key, "StartupSoundEnabled") == 0) {
        startupSoundEnabled = atoi(value) != 0;
        uiPrefs.putBool("boot_snd", startupSoundEnabled);
        if (lvglReady) lvglRefreshConfigUi();
    }
    else if (strcmp(key, "TouchVibrationEnabled") == 0) {
        touchVibrationEnabled = atoi(value) != 0;
        uiPrefs.putBool("touch_vib", touchVibrationEnabled);
        if (lvglReady) lvglRefreshStyleUi();
    }
    else if (strcmp(key, "VibrationIntensity") == 0 && parseIntMessageValue(value, parsed)) {
        vibrationIntensity = static_cast<VibrationIntensity>(constrain(parsed,
                                                                       static_cast<int>(VIBRATION_INTENSITY_LOW),
                                                                       static_cast<int>(VIBRATION_INTENSITY_HIGH)));
        vibrationEnabled = true;
        uiPrefs.putUChar("vib_int", static_cast<uint8_t>(vibrationIntensity));
        uiPrefs.putBool("vib_en", vibrationEnabled);
        if (lvglReady) {
            lvglRefreshConfigUi();
            lvglRefreshSoundPopupUi();
            lvglTopIndicatorStateValid = false;
            lvglRefreshTopIndicators();
        }
    }
    else if (strcmp(key, "MessageTone") == 0 && parseIntMessageValue(value, parsed)) {
        messageBeepTone = static_cast<MessageBeepTone>(constrain(parsed,
                                                                 static_cast<int>(MESSAGE_BEEP_SINGLE),
                                                                 static_cast<int>(MESSAGE_BEEP_TONE_COUNT - 1)));
        uiPrefs.putUChar("msg_tone", static_cast<uint8_t>(messageBeepTone));
        if (lvglReady) lvglRefreshConfigUi();
    }
    else if (strcmp(key, "TopBarCenter") == 0 && parseIntMessageValue(value, parsed)) {
        topBarCenterMode = parsed == static_cast<int>(TOP_BAR_CENTER_TIME) ? TOP_BAR_CENTER_TIME : TOP_BAR_CENTER_NAME;
        uiPrefs.putUChar("top_mid", static_cast<uint8_t>(topBarCenterMode));
        if (topBarCenterMode == TOP_BAR_CENTER_TIME) syncInternetTimeIfNeeded(true);
        lvglTopIndicatorStateValid = false;
        if (lvglReady) {
            lvglRefreshTopIndicators();
            lvglRefreshStyleUi();
        }
    }
    else if (strcmp(key, "TopBarTimezone") == 0 && parseIntMessageValue(value, parsed)) {
        topBarTimezoneGmtOffset = static_cast<int8_t>(constrain(parsed, static_cast<int>(TOP_BAR_GMT_MIN), static_cast<int>(TOP_BAR_GMT_MAX)));
        uiPrefs.putInt("tz_gmt", static_cast<int>(topBarTimezoneGmtOffset));
        if (topBarCenterMode == TOP_BAR_CENTER_TIME) syncInternetTimeIfNeeded(true);
        lvglTopIndicatorStateValid = false;
        if (lvglReady) {
            lvglRefreshTopIndicators();
            lvglRefreshStyleUi();
        }
    }
    else if (strcmp(key, "TelemetryMaxKB") == 0 && parseIntMessageValue(value, parsed)) {
        telemetryMaxKB = static_cast<uint32_t>(max(32, min(8192, parsed)));
        uiPrefs.putUInt("tele_kb", telemetryMaxKB);
    }
    else if (strcmp(key, "IndicatorsVisible") == 0) uiPrefs.putBool("ind_v", atoi(value) != 0);
    else if (strcmp(key, "HomeChatVisible") == 0) {
        homeChatVisible = atoi(value) != 0;
        uiPrefs.putBool("home_chat", homeChatVisible);
        if (lvglReady) {
            lvglRefreshHomeButtonVisibility();
            lvglRefreshStyleHomeItemsUi();
        }
    }
    else if (strcmp(key, "HomeGateVisible") == 0) {
        homeRadioVisible = atoi(value) != 0;
        uiPrefs.putBool("home_gate", homeRadioVisible);
        if (lvglReady) {
            lvglRefreshHomeButtonVisibility();
            lvglRefreshStyleHomeItemsUi();
        }
    }
    else if (strcmp(key, "HomeMediaVisible") == 0) {
        homeMediaVisible = atoi(value) != 0;
        uiPrefs.putBool("home_media", homeMediaVisible);
        if (lvglReady) {
            lvglRefreshHomeButtonVisibility();
            lvglRefreshStyleHomeItemsUi();
        }
    }
    else if (strcmp(key, "HomeInfoVisible") == 0) {
        homeInfoVisible = atoi(value) != 0;
        uiPrefs.putBool("home_info", homeInfoVisible);
        if (lvglReady) {
            lvglRefreshHomeButtonVisibility();
            lvglRefreshStyleHomeItemsUi();
        }
    }
    else if (strcmp(key, "HomeGamesVisible") == 0) {
        homeGamesVisible = atoi(value) != 0;
        uiPrefs.putBool("home_games", homeGamesVisible);
        if (lvglReady) {
            lvglRefreshHomeButtonVisibility();
            lvglRefreshStyleHomeItemsUi();
        }
    }
    else if (strcmp(key, "HomeMqttVisible") == 0) {
        homeMqttVisible = atoi(value) != 0;
        uiPrefs.putBool("home_mqtt", homeMqttVisible);
        if (lvglReady) {
            lvglRefreshHomeButtonVisibility();
            lvglRefreshStyleHomeItemsUi();
        }
    }
    else if (strcmp(key, "HomeConfigVisible") == 0) {
        homeConfigVisible = atoi(value) != 0;
        uiPrefs.putBool("home_cfg", homeConfigVisible);
        if (lvglReady) {
            lvglRefreshHomeButtonVisibility();
            lvglRefreshStyleHomeItemsUi();
        }
    }
    else if (strcmp(key, "HomePowerVisible") == 0) {
        homePowerVisible = atoi(value) != 0;
        uiPrefs.putBool("home_power", homePowerVisible);
        if (lvglReady) {
            lvglRefreshHomeButtonVisibility();
            lvglRefreshStyleHomeItemsUi();
        }
    }
    else if (strcmp(key, "HomeAirplaneVisible") == 0) {
        homeAirplaneVisible = atoi(value) != 0;
        uiPrefs.putBool("home_air", homeAirplaneVisible);
        if (lvglReady) {
            lvglRefreshHomeButtonVisibility();
            lvglRefreshStyleHomeItemsUi();
        }
    }
    else if (strcmp(key, "HomeApVisible") == 0) {
        homeApVisible = atoi(value) != 0;
        uiPrefs.putBool("home_ap", homeApVisible);
        if (lvglReady) {
            lvglRefreshHomeButtonVisibility();
            lvglRefreshStyleHomeItemsUi();
        }
    }
    else if (strcmp(key, "IndicatorsX") == 0 && parseIntMessageValue(value, parsed)) uiPrefs.putInt("ind_x", parsed);
    else if (strcmp(key, "IndicatorsY") == 0 && parseIntMessageValue(value, parsed)) uiPrefs.putInt("ind_y", parsed);
    else if (strcmp(key, "ImuVisible") == 0) uiPrefs.putBool("imu_v", atoi(value) != 0);
    else if (strcmp(key, "ImuX") == 0 && parseIntMessageValue(value, parsed)) uiPrefs.putInt("imu_x", parsed);
    else if (strcmp(key, "ImuY") == 0 && parseIntMessageValue(value, parsed)) uiPrefs.putInt("imu_y", parsed);
    else if (strcmp(key, "MediaVisible") == 0) uiPrefs.putBool("med_v", atoi(value) != 0);
    else if (strcmp(key, "MediaX") == 0 && parseIntMessageValue(value, parsed)) uiPrefs.putInt("med_x", parsed);
    else if (strcmp(key, "MediaY") == 0 && parseIntMessageValue(value, parsed)) uiPrefs.putInt("med_y", parsed);
    else if (strcmp(key, "PathVisible") == 0) uiPrefs.putBool("path_v", atoi(value) != 0);
    else if (strcmp(key, "PathX") == 0 && parseIntMessageValue(value, parsed)) uiPrefs.putInt("path_x", parsed);
    else if (strcmp(key, "PathY") == 0 && parseIntMessageValue(value, parsed)) uiPrefs.putInt("path_y", parsed);
    else if (strcmp(key, "Model3DVisible") == 0) uiPrefs.putBool("mod_v", atoi(value) != 0);
    else if (strcmp(key, "Model3DX") == 0 && parseIntMessageValue(value, parsed)) uiPrefs.putInt("mod_x", parsed);
    else if (strcmp(key, "Model3DY") == 0 && parseIntMessageValue(value, parsed)) uiPrefs.putInt("mod_y", parsed);
    else if (strcmp(key, "SerialVisible") == 0) uiPrefs.putBool("ser_v", atoi(value) != 0);
    else if (strcmp(key, "SerialX") == 0 && parseIntMessageValue(value, parsed)) uiPrefs.putInt("ser_x", parsed);
    else if (strcmp(key, "SerialY") == 0 && parseIntMessageValue(value, parsed)) uiPrefs.putInt("ser_y", parsed);
    else if (strcmp(key, "ViewOverlapFx") == 0) uiPrefs.putBool("ovl_fx", atoi(value) != 0);
    else if (strcmp(key, "ViewSnapFx") == 0) uiPrefs.putBool("snap_fx", atoi(value) != 0);
    else if (strcmp(key, "ViewGravityFx") == 0) uiPrefs.putBool("grav_fx", atoi(value) != 0);
    else if (strcmp(key, "ViewGravityStr") == 0 && parseIntMessageValue(value, parsed)) uiPrefs.putInt("grav_str", max(0, min(100, parsed)));
    else if (strcmp(key, "SerialLogRateMs") == 0 && parseIntMessageValue(value, parsed)) {
        serialLogWsMinIntervalMs = static_cast<uint32_t>(max(0, min(1000, parsed)));
        uiPrefs.putUInt("ser_rate", serialLogWsMinIntervalMs);
    }
    else if (strcmp(key, "SerialLogKeepLines") == 0 && parseIntMessageValue(value, parsed)) {
        serialLogKeepLines = static_cast<size_t>(max(20, min(static_cast<int>(SERIAL_LOG_RING_SIZE), parsed)));
        uiPrefs.putUInt("ser_keep", static_cast<uint32_t>(serialLogKeepLines));
    }
    else handled = false;

    uiPrefs.end();
    return handled;
}

static void stopBluetoothRadio()
{
    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED) esp_bluedroid_disable();
    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_INITIALIZED) esp_bluedroid_deinit();
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) esp_bt_controller_disable();
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) esp_bt_controller_deinit();
}

void applyAirplaneMode(bool enabled, const char *reason)
{
    airplaneModeEnabled = enabled;

    uiPrefs.begin("ui", false);
    uiPrefs.putBool("airplane", airplaneModeEnabled);
    uiPrefs.end();

    if (enabled) {
        mqttClient.disconnect();
        if (mdnsStarted) {
            MDNS.end();
            mdnsStarted = false;
        }
        stopDnsForAp();
        if (webServerRunning) {
            wsCarInput.cleanupClients();
            server.end();
            webServerRunning = false;
        }
        WiFi.softAPdisconnect(true);
        apModeActive = false;
        WiFi.disconnect(true, false);
        wifiRuntimeManaged = false;
        stopWebServerRuntime();
        WiFi.mode(WIFI_OFF);
        stopBluetoothRadio();
        bootStaConnectInProgress = false;
        wifiStaGotIpPending = false;
        wifiStaDisconnectedPending = false;
        uiStatusLine = "Airplane mode enabled";
    } else {
        if (!networkSuspendedForAudio) {
            setupWifiAndServer();
            uiStatusLine = "Airplane mode disabled";
        } else {
            uiStatusLine = "Airplane off; network resumes after audio";
        }
        chatFlushDeferredAirplaneMessage();
    }

    lvglRefreshConfigUi();
    lvglSyncStatusLine();
    lvglRefreshTopIndicators();
    Serial.printf("[RADIO] airplane=%d reason=%s\n", airplaneModeEnabled ? 1 : 0, reason ? reason : "-");
}

static bool initSerialLogStorage()
{
    if (serialLogRing) return true;
    const size_t bytes = SERIAL_LOG_RING_SIZE * (SERIAL_LOG_LINE_MAX + 1);
    serialLogRing = static_cast<char *>(allocPreferPsram(bytes));
    if (!serialLogRing) return false;
    memset(serialLogRing, 0, bytes);
    serialLogHead = 0;
    serialLogCount = 0;
    Serial.printf("[SERIAL] log ring alloc bytes=%u psram=%d\n",
                  static_cast<unsigned int>(bytes),
                  boardHasUsablePsram() && esp_ptr_external_ram(serialLogRing) ? 1 : 0);
    return true;
}

static inline char *serialLogSlot(size_t idx)
{
    return serialLogRing + (idx * (SERIAL_LOG_LINE_MAX + 1));
}

static size_t serialLogActiveCapacity()
{
    return max<size_t>(20, min<size_t>(SERIAL_LOG_RING_SIZE, serialLogKeepLines));
}

static bool serialTerminalStreamingActive()
{
    if (!SERIAL_TERMINAL_TRANSFER_ENABLED) return false;
    if (serialTerminalStreamUntilMs == 0) return false;
    return static_cast<long>(serialTerminalStreamUntilMs - millis()) > 0;
}

static void markSerialTerminalStreamingActive(unsigned long durationMs = 180000UL)
{
    if (!SERIAL_TERMINAL_TRANSFER_ENABLED) return;
    serialTerminalStreamUntilMs = millis() + durationMs;
}

static void clearSerialLogBuffer()
{
    serialLogHead = 0;
    serialLogCount = 0;
    if (serialLogRing) {
        const size_t bytes = SERIAL_LOG_RING_SIZE * (SERIAL_LOG_LINE_MAX + 1);
        memset(serialLogRing, 0, bytes);
    }
}

static String serialLogUrlEncode(const char *text)
{
    static const char hex[] = "0123456789ABCDEF";
    if (!text) return "";
    String out;
    const size_t len = strlen(text);
    out.reserve(len * 3U + 1U);
    for (const uint8_t *p = reinterpret_cast<const uint8_t *>(text); *p; ++p) {
        const uint8_t c = *p;
        if ((c >= 'A' && c <= 'Z') ||
            (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') ||
            c == '-' || c == '_' || c == '.' || c == '~') {
            out += static_cast<char>(c);
        } else {
            out += '%';
            out += hex[(c >> 4) & 0x0F];
            out += hex[c & 0x0F];
        }
    }
    return out;
}

void serialLogPushLine(const char *line, bool sendWs)
{
    if (!SERIAL_TERMINAL_TRANSFER_ENABLED) return;
    if (!line) return;
    if (!initSerialLogStorage()) return;

    char tmp[SERIAL_LOG_LINE_MAX + 1];
    size_t out = 0;
    for (const char *p = line; *p && out < SERIAL_LOG_LINE_MAX; ++p) {
        if (*p == '\r') continue;
        tmp[out++] = *p;
    }
    while (out && (tmp[out - 1] == '\n' || tmp[out - 1] == '\r')) out--;
    tmp[out] = '\0';

    const size_t cap = serialLogActiveCapacity();
    char *slot = serialLogSlot(serialLogHead);
    memcpy(slot, tmp, out + 1U);
    serialLogHead = (serialLogHead + 1U) % cap;
    if (serialLogCount < cap) serialLogCount++;

    if (!sendWs || wsCarInput.count() == 0 || serialLogWsMinIntervalMs == 0 || !serialTerminalStreamingActive()) return;
    const uint32_t now = millis();
    if (static_cast<uint32_t>(now - serialLastWsPushMs) < serialLogWsMinIntervalMs) return;
    serialLastWsPushMs = now;
    wsCarInput.textAll(String("SERLOG,") + serialLogUrlEncode(tmp));
}

static void printSerialHelp()
{
    Serial.println("Commands: help, heap, wifi, sd, version, reboot, clear");
}

void executeSerialCommand(String input)
{
    input.trim();
    if (!input.length()) return;

    String cmd = input;
    cmd.toLowerCase();

    if (cmd == "help") {
        printSerialHelp();
        return;
    }
    if (cmd == "heap" || cmd == "debug") {
        Serial.printf("Free heap: %u bytes\n", static_cast<unsigned int>(ESP.getFreeHeap()));
        Serial.printf("Min free heap: %u bytes\n", static_cast<unsigned int>(ESP.getMinFreeHeap()));
        Serial.printf("Heap size: %u bytes\n", static_cast<unsigned int>(ESP.getHeapSize()));
        Serial.printf("Largest 8-bit block: %u bytes\n",
                      static_cast<unsigned int>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
        return;
    }
    if (cmd == "wifi") {
        const wl_status_t st = wifiStatusSafe();
        Serial.printf("WiFi.status(): %d\n", static_cast<int>(st));
        Serial.printf("STA connected: %s\n", wifiConnectedSafe() ? "yes" : "no");
        Serial.printf("STA SSID: %s\n", wifiSsidSafe().c_str());
        Serial.printf("STA IP: %s\n", wifiIpSafe().c_str());
        Serial.printf("STA RSSI: %d\n", static_cast<int>(wifiRssiSafe()));
        Serial.printf("AP active: %s\n", apModeActive ? "yes" : "no");
        if (apModeActive) Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
        return;
    }
    if (cmd == "sd") {
        const bool mounted = sdEnsureMounted();
        Serial.printf("SD mounted: %s\n", mounted ? "yes" : "no");
        if (mounted) {
            const uint64_t total = SD.totalBytes();
            const uint64_t used = SD.usedBytes();
            Serial.printf("SD total: %llu bytes\n", static_cast<unsigned long long>(total));
            Serial.printf("SD used: %llu bytes\n", static_cast<unsigned long long>(used));
        }
        return;
    }
    if (cmd == "version") {
        Serial.printf("Firmware: %s\n", FW_VERSION);
        return;
    }
    if (cmd == "clear") {
        clearSerialLogBuffer();
        Serial.println("[SERIAL] log cleared");
        return;
    }
    if (cmd == "reboot" || cmd == "reset") {
        Serial.println("[SERIAL] reboot requested");
        rebootRequested = true;
        rebootRequestedAtMs = millis();
        return;
    }

    Serial.printf("[UNKNOWN COMMAND] %s\n", input.c_str());
}

static String wifiDesiredStaSsid()
{
    return pendingSaveCreds ? pendingSaveSsid : savedStaSsid;
}

static String wifiDesiredStaPass()
{
    return pendingSaveCreds ? pendingSavePass : savedStaPass;
}

static bool wifiHasStaTarget()
{
    return !wifiDesiredStaSsid().isEmpty();
}

void registerWifiEvents()
{
    if (wifiEventsRegistered) return;
    wifiStaGotIpEventId = WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
        (void)event;
        (void)info;
        wifiStaGotIpPending = true;
    }, ARDUINO_EVENT_WIFI_STA_GOT_IP);
    wifiStaDisconnectedEventId = WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
        (void)event;
        wifiStaDisconnectReasonPending = info.wifi_sta_disconnected.reason;
        wifiStaDisconnectedPending = true;
    }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    wifiEventsRegistered = true;
}

#include "app/web_server.inc"

