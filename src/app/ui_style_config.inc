void lvglBackToConfigEvent(lv_event_t *e)
{
    (void)e;
    lvglOpenScreen(UI_CONFIG, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
}

void lvglOpenStyleScreenEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_CONFIG_STYLE);
    lvglRefreshStyleUi();
    lvglOpenScreen(UI_CONFIG_STYLE, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglOpenStyleHomeItemsScreenEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_CONFIG_STYLE_HOME_ITEMS);
    lvglRefreshStyleHomeItemsUi();
    lvglOpenScreen(UI_CONFIG_STYLE_HOME_ITEMS, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglOpenLanguageScreenEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_CONFIG_LANGUAGE);
    lvglRefreshLanguageUi();
    lvglOpenScreen(UI_CONFIG_LANGUAGE, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglStyleScreensaverToggleEvent(lv_event_t *e)
{
    if (lvglStyleUiSyncing) return;
    lv_obj_t *target = e ? lv_event_get_target(e) : nullptr;
    screensaverEnabled = target && lv_obj_has_state(target, LV_STATE_CHECKED);
    uiPrefs.begin("ui", false);
    uiPrefs.putBool("scrsvr_en", screensaverEnabled);
    uiPrefs.end();
    if (!screensaverEnabled && screensaverActive) screensaverSetActive(false);
    lvglRefreshStyleUi();
}

void lvglStyleMenuIconsToggleEvent(lv_event_t *e)
{
    if (lvglStyleUiSyncing) return;
    lv_obj_t *target = e ? lv_event_get_target(e) : nullptr;
    menuCustomIconsEnabled = target && lv_obj_has_state(target, LV_STATE_CHECKED);
    uiPrefs.begin("ui", false);
    uiPrefs.putBool("menu_3d_i", menuCustomIconsEnabled);
    uiPrefs.end();
    lvglRefreshPrimaryMenuButtonIcons();
    lvglRefreshConfigUi();
    lvglRefreshStyleUi();
}

void lvglStyleTouchVibrationToggleEvent(lv_event_t *e)
{
    if (lvglStyleUiSyncing) return;
    lv_obj_t *target = e ? lv_event_get_target(e) : nullptr;
    touchVibrationEnabled = target && lv_obj_has_state(target, LV_STATE_CHECKED);
    saveSoundPrefs();
    lvglRefreshStyleUi();
}

void lvglStyleTouchSoundToggleEvent(lv_event_t *e)
{
    if (lvglStyleUiSyncing) return;
    lv_obj_t *target = e ? lv_event_get_target(e) : nullptr;
    touchSoundEnabled = target && lv_obj_has_state(target, LV_STATE_CHECKED);
    saveSoundPrefs();
    lvglRefreshStyleUi();
}

static void saveHomeScreenPrefs()
{
    uiPrefs.begin("ui", false);
    uiPrefs.putBool("home_chat", homeChatVisible);
    uiPrefs.putBool("home_gate", homeRadioVisible);
    uiPrefs.putBool("home_media", homeMediaVisible);
    uiPrefs.putBool("home_info", homeInfoVisible);
    uiPrefs.putBool("home_games", homeGamesVisible);
    uiPrefs.putBool("home_mqtt", homeMqttVisible);
    uiPrefs.putBool("home_cfg", homeConfigVisible);
    uiPrefs.putBool("home_power", homePowerVisible);
    uiPrefs.putBool("home_air", homeAirplaneVisible);
    uiPrefs.putBool("home_ap", homeApVisible);
    uiPrefs.end();
}

static void lvglHandleHomeVisibilityToggle(lv_event_t *e, bool &flag)
{
    if (lvglStyleUiSyncing) return;
    lv_obj_t *target = e ? lv_event_get_target(e) : nullptr;
    flag = target && lv_obj_has_state(target, LV_STATE_CHECKED);
    saveHomeScreenPrefs();
    lvglRefreshHomeButtonVisibility();
    lvglRefreshStyleHomeItemsUi();
}

void lvglStyleHomeChatToggleEvent(lv_event_t *e) { lvglHandleHomeVisibilityToggle(e, homeChatVisible); }
void lvglStyleHomeRadioToggleEvent(lv_event_t *e) { lvglHandleHomeVisibilityToggle(e, homeRadioVisible); }
void lvglStyleHomeMediaToggleEvent(lv_event_t *e) { lvglHandleHomeVisibilityToggle(e, homeMediaVisible); }
void lvglStyleHomeInfoToggleEvent(lv_event_t *e) { lvglHandleHomeVisibilityToggle(e, homeInfoVisible); }
void lvglStyleHomeGamesToggleEvent(lv_event_t *e) { lvglHandleHomeVisibilityToggle(e, homeGamesVisible); }
void lvglStyleHomeMqttToggleEvent(lv_event_t *e) { lvglHandleHomeVisibilityToggle(e, homeMqttVisible); }
void lvglStyleHomeConfigToggleEvent(lv_event_t *e) { lvglHandleHomeVisibilityToggle(e, homeConfigVisible); }
void lvglStyleHomePowerToggleEvent(lv_event_t *e) { lvglHandleHomeVisibilityToggle(e, homePowerVisible); }
void lvglStyleHomeAirplaneToggleEvent(lv_event_t *e) { lvglHandleHomeVisibilityToggle(e, homeAirplaneVisible); }
void lvglStyleHomeApToggleEvent(lv_event_t *e) { lvglHandleHomeVisibilityToggle(e, homeApVisible); }

void lvglStyleButtonFlatEvent(lv_event_t *e)
{
    (void)e;
    lvglSetButtonStyleMode(UI_BUTTON_STYLE_FLAT, true);
}

void lvglStyleButton3dEvent(lv_event_t *e)
{
    (void)e;
    lvglSetButtonStyleMode(UI_BUTTON_STYLE_3D, true);
}

void lvglStyleButtonBlackEvent(lv_event_t *e)
{
    (void)e;
    lvglSetButtonStyleMode(UI_BUTTON_STYLE_BLACK, true);
}

void lvglStyleTimezoneEvent(lv_event_t *e)
{
    (void)e;
    if (!lvglStyleTimezoneDd) return;
    const int selected = static_cast<int>(lv_dropdown_get_selected(lvglStyleTimezoneDd));
    topBarTimezoneGmtOffset = static_cast<int8_t>(constrain(TOP_BAR_GMT_MIN + selected, TOP_BAR_GMT_MIN, TOP_BAR_GMT_MAX));
    uiPrefs.begin("ui", false);
    uiPrefs.putInt("tz_gmt", static_cast<int>(topBarTimezoneGmtOffset));
    uiPrefs.end();
    syncInternetTimeIfNeeded(true);
    lvglTopIndicatorStateValid = false;
    lvglRefreshTopIndicators();
    lvglRefreshStyleUi();
}

void lvglRadioModuleDropdownEvent(lv_event_t *e)
{
    (void)e;
    if (!lvglRadioModuleDropdown) return;
    uint16_t selected = lv_dropdown_get_selected(lvglRadioModuleDropdown);
#if !defined(BOARD_ESP32S3_3248S035_N16R8)
    selected = 0;
#endif
    if (selected >= static_cast<uint16_t>(RADIO_MODULE_COUNT)) return;
    if (radioModuleType == static_cast<RadioModuleType>(selected)) return;
    radioModuleType = static_cast<RadioModuleType>(selected);
    uiPrefs.begin("ui", false);
    uiPrefs.putUChar("radio_mod", static_cast<uint8_t>(radioModuleType));
    uiPrefs.end();
    delete hc12TerminalLog;
    hc12TerminalLog = nullptr;
    delete hc12RadioRxLine;
    hc12RadioRxLine = nullptr;
    hc12ConfigStatusText = String("Active module: ") + radioModuleLabel(radioModuleType);
    hc12ReadConfigSelection();
    lvglRefreshHc12ConfigUi();
    lvglRefreshHc12Ui();
    lvglRefreshHc12InfoUi();
}

void lvglLanguageDropdownEvent(lv_event_t *e)
{
    (void)e;
    if (!lvglLanguageDropdown) return;
    const uint16_t selected = lv_dropdown_get_selected(lvglLanguageDropdown);
    if (selected >= static_cast<uint16_t>(UI_LANG_COUNT)) return;
    uiLanguage = static_cast<UiLanguage>(selected);
    uiPrefs.begin("ui", false);
    uiPrefs.putUChar("lang", static_cast<uint8_t>(uiLanguage));
    uiPrefs.end();
    lvglRefreshLocalizedUi();
    uiStatusLine = tr(TXT_LANGUAGE_SAVED);
    lvglSyncStatusLine();
}

void lvglVibrationDropdownEvent(lv_event_t *e)
{
    (void)e;
    if (!lvglVibrationDropdown) return;
    const uint16_t selected = lv_dropdown_get_selected(lvglVibrationDropdown);
    if (selected >= static_cast<uint16_t>(VIBRATION_INTENSITY_COUNT)) return;
    vibrationIntensity = static_cast<VibrationIntensity>(selected);
    vibrationEnabled = true;
    saveSoundPrefs();
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    chatMessageVibrationPreview();
#endif
    uiStatusLine = String("Vibration: ") + vibrationIntensityLabel(vibrationIntensity);
    lvglSyncStatusLine();
    lvglRefreshConfigUi();
    lvglRefreshSoundPopupUi();
    lvglTopIndicatorStateValid = false;
    lvglRefreshTopIndicators();
}

void lvglMessageToneDropdownEvent(lv_event_t *e)
{
    (void)e;
    if (!lvglMessageToneDropdown) return;
    const uint16_t selected = lv_dropdown_get_selected(lvglMessageToneDropdown);
    if (selected >= static_cast<uint16_t>(MESSAGE_BEEP_TONE_COUNT)) return;
    messageBeepTone = static_cast<MessageBeepTone>(selected);
    uiPrefs.begin("ui", false);
    uiPrefs.putUChar("msg_tone", static_cast<uint8_t>(messageBeepTone));
    uiPrefs.end();
    chatMessageBeepPreview();
    uiStatusLine = String("Message tone: ") + messageBeepToneLabel(messageBeepTone);
    lvglSyncStatusLine();
    lvglRefreshConfigUi();
}

void lvglStartupSoundToggleEvent(lv_event_t *e)
{
    lv_obj_t *target = e ? lv_event_get_target(e) : nullptr;
    startupSoundEnabled = target && lv_obj_has_state(target, LV_STATE_CHECKED);
    saveSoundPrefs();
    uiStatusLine = startupSoundEnabled ? "Startup sound enabled" : "Startup sound disabled";
    lvglSyncStatusLine();
    lvglRefreshConfigUi();
}

void lvglStyleTopCenterNameEvent(lv_event_t *e)
{
    (void)e;
    topBarCenterMode = TOP_BAR_CENTER_NAME;
    uiPrefs.begin("ui", false);
    uiPrefs.putUChar("top_mid", static_cast<uint8_t>(topBarCenterMode));
    uiPrefs.end();
    lvglTopIndicatorStateValid = false;
    lvglRefreshTopIndicators();
    lvglRefreshStyleUi();
}

void lvglStyleTopCenterTimeEvent(lv_event_t *e)
{
    (void)e;
    topBarCenterMode = TOP_BAR_CENTER_TIME;
    uiPrefs.begin("ui", false);
    uiPrefs.putUChar("top_mid", static_cast<uint8_t>(topBarCenterMode));
    uiPrefs.end();
    syncInternetTimeIfNeeded(true);
    lvglTopIndicatorStateValid = false;
    lvglRefreshTopIndicators();
    lvglRefreshStyleUi();
}

void lvglStyleTimeoutEvent(lv_event_t *e)
{
    (void)e;
    if (lvglStyleUiSyncing) return;
    if (!lvglStyleTimeoutSlider) return;
    const unsigned long stepValue = static_cast<unsigned long>(lv_slider_get_value(lvglStyleTimeoutSlider));
    displayIdleTimeoutMs = clampIdleTimeoutMs(stepValue * LCD_IDLE_TIMEOUT_STEP_MS);
    applyDisplayIdleTimeoutPowerOffCap(true);
    persistDisplayIdleTimeout();
    lvglRefreshStyleUi();
    lvglNextHandlerDueMs = 0;
}

void lvglStylePowerOffEvent(lv_event_t *e)
{
    (void)e;
    if (lvglStyleUiSyncing || !lvglStylePowerOffDropdown) return;
    const uint16_t selected = lv_dropdown_get_selected(lvglStylePowerOffDropdown);
    if (selected >= (sizeof(POWER_OFF_IDLE_TIMEOUT_OPTIONS_MS) / sizeof(POWER_OFF_IDLE_TIMEOUT_OPTIONS_MS[0]))) return;
    powerOffIdleTimeoutMs = POWER_OFF_IDLE_TIMEOUT_OPTIONS_MS[selected];
    uiPrefs.begin("ui", false);
    uiPrefs.putULong("pwr_idle", powerOffIdleTimeoutMs);
    uiPrefs.end();
    const bool timeoutChanged = applyDisplayIdleTimeoutPowerOffCap(true);
    lvglRefreshStyleUi();
    if (timeoutChanged) {
        if (lvglStyleTimeoutSlider) lv_obj_invalidate(lvglStyleTimeoutSlider);
        if (lvglStyleTimeoutValueLabel) lv_obj_invalidate(lvglStyleTimeoutValueLabel);
    }
    lvglNextHandlerDueMs = 0;
}

