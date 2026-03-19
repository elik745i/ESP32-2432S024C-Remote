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
    doc["HomeMediaVisible"] = homeMediaVisible ? 1 : 0;
    doc["HomeInfoVisible"] = homeInfoVisible ? 1 : 0;
    doc["HomeGamesVisible"] = homeGamesVisible ? 1 : 0;
    doc["HomeConfigVisible"] = homeConfigVisible ? 1 : 0;
    doc["HomePowerVisible"] = homePowerVisible ? 1 : 0;
    doc["HomeAirplaneVisible"] = homeAirplaneVisible ? 1 : 0;
    doc["HomeApVisible"] = homeApVisible ? 1 : 0;

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
    homeMediaVisible = uiPrefs.getBool("home_media", true);
    homeInfoVisible = uiPrefs.getBool("home_info", true);
    homeGamesVisible = uiPrefs.getBool("home_games", true);
    homeConfigVisible = uiPrefs.getBool("home_cfg", true);
    homePowerVisible = uiPrefs.getBool("home_power", true);
    homeAirplaneVisible = uiPrefs.getBool("home_air", true);
    homeApVisible = uiPrefs.getBool("home_ap", true);
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    radioModuleType = static_cast<RadioModuleType>(constrain(uiPrefs.getUChar("radio_mod", static_cast<uint8_t>(RADIO_MODULE_HC12)),
                                                             static_cast<uint8_t>(RADIO_MODULE_HC12),
                                                             static_cast<uint8_t>(RADIO_MODULE_COUNT - 1)));
#else
    radioModuleType = RADIO_MODULE_HC12;
#endif
    loadPersistedRadioSettings();
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

static void otaStorePopupVersion(const String &version)
{
    copyTextToBuf(otaPendingPopupVersion, sizeof(otaPendingPopupVersion), version);
    uiPrefs.begin("ui", false);
    uiPrefs.putString("ota_popup", version);
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
    otaPostUpdatePopupPending = false;
    uiPrefs.begin("ui", false);
    uiPrefs.remove("ota_popup");
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
        if (targetVersion.length()) otaStorePopupVersion(targetVersion);
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

static void beginDnsForAp(IPAddress ip)
{
    if (ip[0] == 0) return;
    if (!webServerEnabled) return;
    dnsServer.stop();
    dnsServer.start(53, "*", ip);
    dnsRunning = true;
}

static void stopDnsForAp()
{
    if (!dnsRunning) return;
    dnsServer.stop();
    dnsRunning = false;
}

static void stopWebServerRuntime()
{
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
}

static void ensureWebServerRuntime()
{
    if (!webServerEnabled || airplaneModeEnabled || networkSuspendedForAudio) return;
    if (!webRoutesRegistered) {
        setupWebRoutes();
        webRoutesRegistered = true;
    }
    if (!webServerRunning) {
        server.begin();
        webServerRunning = true;
    }
    if (apModeActive) beginDnsForAp(WiFi.softAPIP());
}

static void persistWebServerEnabled()
{
    uiPrefs.begin("ui", false);
    uiPrefs.putBool("web_srv", webServerEnabled);
    uiPrefs.end();
}

static void wifiEnsureRuntimeEnabled(const char *reason, wifi_mode_t mode, bool startWebServer)
{
    if (airplaneModeEnabled) return;
    wifiRuntimeManaged = true;
    registerWifiEvents();
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    WiFi.setSleep(false);
    if (WiFi.getMode() != mode) WiFi.mode(mode);
    if (startWebServer && webServerEnabled) ensureWebServerRuntime();
    Serial.printf("[WIFI] runtime enabled reason=%s mode=%d web=%d\n",
                  reason ? reason : "-",
                  static_cast<int>(mode),
                  (startWebServer && webServerEnabled) ? 1 : 0);
}

void ensureApOnline(const char *reason)
{
    if (networkSuspendedForAudio || airplaneModeEnabled) return;
    if (apModeActive) return;
    normalizeSavedApCreds();
    wifiEnsureRuntimeEnabled(reason ? reason : "ensure_ap", WIFI_AP_STA, true);
    const char *apSsid = savedApSsid.length() ? savedApSsid.c_str() : AP_SSID;
    const char *apPass = savedApPass.length() ? savedApPass.c_str() : AP_PASS;
    if (!WiFi.softAP(apSsid, apPass)) {
        Serial.printf("[WIFI] AP start failed reason=%s\n", reason ? reason : "-");
        return;
    }
    apModeActive = true;
    const IPAddress ip = WiFi.softAPIP();
    beginDnsForAp(ip);
    uiStatusLine = "AP ready: " + ip.toString();
    if (lvglReady) {
        lvglRefreshConfigUi();
        lvglRefreshAllButtonStyles();
    }
    Serial.printf("[WIFI] AP enabled reason=%s ip=%s\n",
                  reason ? reason : "-",
                  ip.toString().c_str());
}

void forceSessionApMode(const char *reason)
{
    wifiSessionApMode = true;
    saveApModePref(true);
    wifiForgetPendingUi = false;
    pendingSaveCreds = false;
    pendingSaveSsid = "";
    pendingSavePass = "";
    bootStaConnectInProgress = false;
    bootStaConnectStartedMs = 0;
    staLastConnectAttemptMs = 0;
    WiFi.disconnect(false, false);
    ensureApOnline(reason ? reason : "session_ap_mode");
}

void disableApWhenStaConnected(const char *reason)
{
    if (!apModeActive && WiFi.getMode() == WIFI_STA) return;
    stopDnsForAp();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    apModeActive = false;
    if (lvglReady) {
        lvglRefreshConfigUi();
        lvglRefreshAllButtonStyles();
    }
    Serial.printf("[WIFI] AP disabled reason=%s\n", reason ? reason : "-");
}

static void beginStaConnectAttempt(const char *reason)
{
    if (networkSuspendedForAudio || airplaneModeEnabled || wifiSessionApMode || !wifiHasStaTarget()) return;
    const String ssid = wifiDesiredStaSsid();
    const String pass = wifiDesiredStaPass();
    const wifi_mode_t mode = apModeActive ? WIFI_AP_STA : WIFI_STA;
    wifiEnsureRuntimeEnabled(reason ? reason : "sta_connect", mode, apModeActive);
    WiFi.begin(ssid.c_str(), pass.c_str());
    bootStaConnectInProgress = true;
    bootStaConnectStartedMs = millis();
    staLastConnectAttemptMs = bootStaConnectStartedMs;
    uiStatusLine = "Connecting to " + ssid;
    Serial.printf("[WIFI] STA connect start reason=%s ssid=%s ap=%d\n",
                  reason ? reason : "-",
                  ssid.c_str(),
                  apModeActive ? 1 : 0);
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
    mqttButtonCount = mqttPrefs.getInt("btn_count", 4);
    if (mqttButtonCount < 1) mqttButtonCount = 1;
    if (mqttButtonCount > MQTT_MAX_BUTTONS) mqttButtonCount = MQTT_MAX_BUTTONS;
    for (int i = 0; i < MQTT_MAX_BUTTONS; i++) {
        mqttButtonNames[i] = mqttPrefs.getString(("btn_" + String(i)).c_str(), "");
        if (mqttButtonNames[i].isEmpty()) mqttButtonNames[i] = mqttDefaultButtonName(i);
        mqttButtonCritical[i] = mqttPrefs.getBool(("crit_" + String(i)).c_str(), false);
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
    }
    mqttPrefs.end();
}

bool mqttConnectNow()
{
    if (!mqttCfg.enabled) {
        mqttStatusLine = "Disabled";
        mqttConnectRequested = false;
        if (lvglReady) lvglRefreshTopIndicators();
        return false;
    }
    if (networkSuspendedForAudio || !wifiConnectedSafe()) {
        mqttStatusLine = "No WiFi";
        if (lvglReady) lvglRefreshTopIndicators();
        return false;
    }
    if (!mqttHasHeapHeadroom(MQTT_MIN_FREE_HEAP_CONNECT, MQTT_MIN_LARGEST_8BIT_CONNECT, "MQTT waiting for memory")) {
        if (lvglReady) lvglRefreshTopIndicators();
        return false;
    }
    if (!mqttClient.setBufferSize(MQTT_CLIENT_BUFFER_SIZE)) {
        mqttStatusLine = "MQTT buffer alloc failed";
        if (lvglReady) lvglRefreshTopIndicators();
        return false;
    }

    mqttNetClient.stop();
    mqttNetClient.setTimeout(250);
    if (!mqttNetClient.connect(mqttCfg.broker.c_str(), mqttCfg.port, 250)) {
        mqttStatusLine = "Broker unreachable";
        if (lvglReady) lvglRefreshTopIndicators();
        return false;
    }

    mqttClient.setServer(mqttCfg.broker.c_str(), mqttCfg.port);
    mqttClient.setCallback([](char *topic, uint8_t *payload, unsigned int length) {
        if (!topic || !payload || length == 0) return;
        if (length > MQTT_CLIENT_BUFFER_SIZE) return;
        if (ESP.getFreeHeap() < MQTT_MIN_FREE_HEAP_PUBLISH ||
            heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) < MQTT_MIN_LARGEST_8BIT_PUBLISH) return;

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

        // ACK before SD/UI work so the sender can leave transmit mode sooner.
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
        if (lvglReady) lvglRefreshTopIndicators();
        return false;
    }

    mqttClient.publish(availabilityTopic.c_str(), "online", true);
    mqttClient.subscribe(mqttChatInboxTopic.c_str());
    if (!mqttChatPresenceTopic.isEmpty()) mqttClient.subscribe(mqttChatPresenceTopic.c_str());
    mqttPublishPeerPresence();
    mqttDiscoveryPublished = false;
    mqttStatusLine = "Connected";
    mqttConnectRequested = false;

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

bool mqttPublishChatMessage(const String &text)
{
    if (currentChatPeerKey.isEmpty()) return false;
    const String messageId = chatGenerateMessageId();
    return mqttPublishChatMessageWithId(currentChatPeerKey, text, messageId);
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

void mqttPublishButtonAction(int index)
{
    if (index < 0 || index >= mqttButtonCount) return;
    String payload = mqttButtonPayloadForIndex(index);
    mqttPublishAction(payload.c_str());
}

void mqttService()
{
    if (networkSuspendedForAudio || airplaneModeEnabled) {
        const bool wasConnected = mqttClient.connected();
        if (wasConnected) mqttClient.disconnect();
        mqttTrimBufferForIdle();
        mqttConnectRequested = false;
        mqttStatusLine = "No WiFi";
        if (wasConnected && lvglReady) lvglRefreshTopIndicators();
        return;
    }
    if (!mqttCfg.enabled) {
        const bool wasConnected = mqttClient.connected();
        if (wasConnected) mqttClient.disconnect();
        mqttTrimBufferForIdle();
        mqttConnectRequested = false;
        if (wasConnected && lvglReady) lvglRefreshTopIndicators();
        return;
    }
    if (!wifiConnectedSafe()) {
        const bool wasConnected = mqttClient.connected();
        if (wasConnected) mqttClient.disconnect();
        mqttTrimBufferForIdle();
        mqttConnectRequested = false;
        mqttStatusLine = "No WiFi";
        if (wasConnected && lvglReady) lvglRefreshTopIndicators();
        return;
    }
    if (!mqttClient.connected()) {
        if ((mqttConnectRequested || millis() - mqttLastReconnectMs >= MQTT_RECONNECT_MS)) {
            mqttLastReconnectMs = millis();
            if (mqttConnectNow()) mqttPublishDiscovery();
        }
        return;
    }
    mqttClient.loop();
    if (!mqttDiscoveryPublished) mqttPublishDiscovery();
}

void tryBootStaReconnect()
{
    if (airplaneModeEnabled) return;
    if (!wifiHasStaTarget()) {
        ensureApOnline("no_saved_sta");
        return;
    }
    apModeActive = false;
    stopDnsForAp();
    WiFi.mode(WIFI_STA);
    beginStaConnectAttempt("boot");
}

void startWifiConnect(const String &ssid, const String &pass)
{
    if (airplaneModeEnabled) {
        uiStatusLine = "Disable airplane mode first";
        lvglSyncStatusLine();
        return;
    }
    wifiSessionApMode = false;
    clearWifiScanResults();
    pendingSaveSsid = ssid;
    pendingSavePass = pass;
    pendingSaveCreds = true;
    ensureApOnline("manual_connect");
    beginStaConnectAttempt("manual_connect");
    lvglSyncStatusLine();
}

String urlDecode(const String &input)
{
    String s;
    char a = 0, b = 0;
    for (size_t i = 0; i < input.length(); i++) {
        if ((input[i] == '%') && (i + 2 < input.length()) && ((a = input[i + 1]) && (b = input[i + 2])) && isxdigit(a) && isxdigit(b)) {
            if (a >= 'a') a -= 'a' - 'A';
            if (a >= 'A') a = a - 'A' + 10;
            else a -= '0';
            if (b >= 'a') b -= 'a' - 'A';
            if (b >= 'A') b = b - 'A' + 10;
            else b -= '0';
            s += char(16 * a + b);
            i += 2;
        } else if (input[i] == '+') {
            s += ' ';
        } else {
            s += input[i];
        }
    }
    return s;
}

bool pathUnderWeb(const String &path)
{
    String p = path;
    if (!p.startsWith("/")) p = "/" + p;
    return (p == "/web") || p.startsWith("/web/");
}

