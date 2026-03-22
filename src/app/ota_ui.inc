void lvglOtaUpdateEvent(lv_event_t *e)
{
    (void)e;
    if (!otaUpdateAvailable || otaLatestBinUrl.isEmpty()) {
        otaCheckRequested = true;
        uiStatusLine = "Checking for updates";
        lvglSyncStatusLine();
        return;
    }
    String releaseNotes;
    const String latestVersion = otaNormalizedVersion(String(otaLatestVersion));
    for (uint8_t i = 0; i < otaReleaseEntryCount; ++i) {
        if (otaNormalizedVersion(otaReleaseEntries[i].tag) == latestVersion) {
            releaseNotes = otaReleaseEntries[i].notes;
            break;
        }
    }
    otaStartUpdateTask(otaLatestBinUrl, String(otaLatestVersion), "Starting update...", releaseNotes);
}

void lvglOtaReleaseSelectEvent(lv_event_t *e)
{
    if (!e) return;
    lv_obj_t *target = lv_event_get_target(e);
    const int idx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (!target || idx < 0 || idx >= static_cast<int>(OTA_RELEASE_LIST_MAX)) return;

    const bool checked = lv_obj_has_state(target, LV_STATE_CHECKED);
    if (!checked) {
        if (otaSelectedReleaseIndex == idx) otaSelectedReleaseIndex = -1;
        return;
    }

    otaSelectedReleaseIndex = static_cast<int8_t>(idx);
    for (uint8_t i = 0; i < OTA_RELEASE_LIST_MAX; ++i) {
        if (!lvglOtaReleaseChecks[i] || lvglOtaReleaseChecks[i] == target) continue;
        lv_obj_clear_state(lvglOtaReleaseChecks[i], LV_STATE_CHECKED);
    }
}

void lvglOtaReflashEvent(lv_event_t *e)
{
    (void)e;
    const int idx = otaSelectedReleaseIndex;
    if (idx < 0 || idx >= otaReleaseEntryCount || otaReleaseEntries[idx].tag.isEmpty()) {
        otaUiState = OTA_UI_ERROR;
        otaSetStatus("Select one release first");
        uiStatusLine = "Select one release to flush";
        lvglSyncStatusLine();
        lvglRefreshOtaUi();
        return;
    }
    otaStartUpdateTask(otaReleaseEntries[idx].binUrl,
                       otaReleaseEntries[idx].tag,
                       String("Starting flush to ") + otaReleaseEntries[idx].tag + "...",
                       otaReleaseEntries[idx].notes);
}

static bool screenLockIsConfigured()
{
    return screenLockEnabled && strlen(screenLockPin) == 4;
}

static bool screenLockShouldGateUi()
{
    return screenLockIsConfigured() && !screenLockUnlocked;
}

static bool screenLockIsBlocked()
{
    return screenLockBlockedUntilMs != 0 && static_cast<long>(screenLockBlockedUntilMs - millis()) > 0;
}

static bool screenLockNumericTarget(lv_obj_t *ta)
{
    return ta && (ta == lvglScreenLockSetupPinTa || ta == lvglScreenLockPinTa);
}

static void screenLockSanitizeTextArea(lv_obj_t *ta)
{
    if (!ta) return;
    const char *raw = lv_textarea_get_text(ta);
    String filtered;
    if (raw) {
        for (size_t i = 0; raw[i] != '\0' && filtered.length() < 4; ++i) {
            if (raw[i] >= '0' && raw[i] <= '9') filtered += raw[i];
        }
    }
    if (filtered != String(raw ? raw : "")) {
        lv_textarea_set_text(ta, filtered.c_str());
        lv_textarea_set_cursor_pos(ta, LV_TEXTAREA_CURSOR_LAST);
    }
}

static void screenLockResetState()
{
    screenLockUnlocked = !screenLockIsConfigured();
    screenLockFailedTotal = 0;
    screenLockBlockedUntilMs = 0;
}

static void screenLockPersistConfig()
{
    uiPrefs.begin("ui", false);
    uiPrefs.putBool("scr_lock", screenLockEnabled);
    uiPrefs.putString("scr_pin", screenLockPin);
    uiPrefs.end();
}

static String screenLockBlockMessage()
{
    const unsigned long now = millis();
    if (screenLockBlockedUntilMs == 0 || static_cast<long>(screenLockBlockedUntilMs - now) <= 0) return "Enter PIN to unlock";
    const unsigned long remainingMs = screenLockBlockedUntilMs - now;
    const unsigned long totalSeconds = (remainingMs + 999UL) / 1000UL;
    const unsigned long minutes = totalSeconds / 60UL;
    const unsigned long seconds = totalSeconds % 60UL;
    char buf[96];
    snprintf(buf, sizeof(buf), "3+ failed attempts. Wait %lu:%02lu to try again.", minutes, seconds);
    return String(buf);
}

static void screenLockHideSetupModal(bool restoreToggle)
{
    if (lvglKb && lv_keyboard_get_textarea(lvglKb) == lvglScreenLockSetupPinTa) lvglHideKeyboard();
    if (lvglScreenLockModal) lv_obj_add_flag(lvglScreenLockModal, LV_OBJ_FLAG_HIDDEN);
    if (restoreToggle && lvglStyleScreenLockSw) lv_obj_clear_state(lvglStyleScreenLockSw, LV_STATE_CHECKED);
    if (lvglScreenLockSetupPinTa) lv_textarea_set_text(lvglScreenLockSetupPinTa, "");
    if (lvglScreenLockSetupStatusLabel) lv_label_set_text(lvglScreenLockSetupStatusLabel, "");
}

static void screenLockShowSetupModal()
{
    if (!lvglReady) return;
    if (!lvglScreenLockModal) {
        lvglScreenLockModal = lv_obj_create(lv_layer_top());
        if (!lvglScreenLockModal) return;
        lv_obj_set_size(lvglScreenLockModal, min<lv_coord_t>(DISPLAY_WIDTH - 20, 220), LV_SIZE_CONTENT);
        lv_obj_set_align(lvglScreenLockModal, LV_ALIGN_TOP_MID);
        lv_obj_set_y(lvglScreenLockModal, UI_CONTENT_TOP_Y + 6);
        lv_obj_set_style_bg_color(lvglScreenLockModal, lv_color_hex(0x16212C), 0);
        lv_obj_set_style_border_color(lvglScreenLockModal, lv_color_hex(0x5A6B7C), 0);
        lv_obj_set_style_border_width(lvglScreenLockModal, 1, 0);
        lv_obj_set_style_radius(lvglScreenLockModal, 12, 0);
        lv_obj_set_style_pad_all(lvglScreenLockModal, 10, 0);
        lv_obj_set_style_pad_row(lvglScreenLockModal, 8, 0);
        lv_obj_set_flex_flow(lvglScreenLockModal, LV_FLEX_FLOW_COLUMN);
        lv_obj_clear_flag(lvglScreenLockModal, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *title = lv_label_create(lvglScreenLockModal);
        lv_label_set_text(title, "Set 4-digit PIN");
        lv_obj_set_style_text_color(title, lv_color_hex(0xE5ECF3), 0);

        lvglScreenLockSetupPinTa = lv_textarea_create(lvglScreenLockModal);
        lv_obj_set_width(lvglScreenLockSetupPinTa, lv_pct(100));
        lv_obj_set_height(lvglScreenLockSetupPinTa, 36);
        lvglConfigureSingleLinePasswordTa(lvglScreenLockSetupPinTa);
        lv_textarea_set_max_length(lvglScreenLockSetupPinTa, 4);
        lv_textarea_set_placeholder_text(lvglScreenLockSetupPinTa, "0000");
        lv_obj_add_event_cb(lvglScreenLockSetupPinTa, lvglTextAreaFocusEvent, LV_EVENT_FOCUSED, nullptr);
        lv_obj_add_event_cb(lvglScreenLockSetupPinTa, lvglScreenLockSetupPinChangedEvent, LV_EVENT_VALUE_CHANGED, nullptr);

        lvglScreenLockSetupStatusLabel = lv_label_create(lvglScreenLockModal);
        lv_obj_set_width(lvglScreenLockSetupStatusLabel, lv_pct(100));
        lv_label_set_long_mode(lvglScreenLockSetupStatusLabel, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_color(lvglScreenLockSetupStatusLabel, lv_color_hex(0xD8B36A), 0);

        lv_obj_t *actions = lv_obj_create(lvglScreenLockModal);
        lv_obj_set_size(actions, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(actions, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(actions, 0, 0);
        lv_obj_set_style_pad_all(actions, 0, 0);
        lv_obj_set_style_pad_column(actions, 8, 0);
        lv_obj_set_flex_flow(actions, LV_FLEX_FLOW_ROW);
        lv_obj_clear_flag(actions, LV_OBJ_FLAG_SCROLLABLE);

        auto makeModalBtn = [&](lv_obj_t **slot, const char *txt, lv_color_t color, lv_event_cb_t cb) {
            lv_obj_t *btn = lv_btn_create(actions);
            if (!btn) return;
            lv_obj_set_size(btn, 88, 32);
            lv_obj_set_style_radius(btn, 8, 0);
            lv_obj_set_style_border_width(btn, 0, 0);
            lv_obj_add_event_cb(btn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
            lv_obj_add_event_cb(btn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
            lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, nullptr);
            lv_obj_t *lbl = lv_label_create(btn);
            lv_label_set_text(lbl, txt);
            lv_obj_center(lbl);
            lvglRegisterStyledButton(btn, color, true);
            *slot = btn;
        };

        makeModalBtn(&lvglScreenLockSetupCancelBtn, tr(TXT_CANCEL), lv_color_hex(0x4E5D6C), lvglStyleScreenLockToggleEvent);
        makeModalBtn(&lvglScreenLockSetupSaveBtn, tr(TXT_SAVE), lv_color_hex(0x2F7A45), lvglScreenLockUnlockEvent);
    }

    if (lvglScreenLockSetupPinTa) lv_textarea_set_text(lvglScreenLockSetupPinTa, "");
    if (lvglScreenLockSetupStatusLabel) lv_label_set_text(lvglScreenLockSetupStatusLabel, "");
    lv_obj_clear_flag(lvglScreenLockModal, LV_OBJ_FLAG_HIDDEN);
}

void lvglScreenLockSetupPinChangedEvent(lv_event_t *e)
{
    screenLockSanitizeTextArea(e ? lv_event_get_target(e) : nullptr);
}

void lvglScreenLockPinChangedEvent(lv_event_t *e)
{
    screenLockSanitizeTextArea(e ? lv_event_get_target(e) : nullptr);
}

void lvglStyleScreenLockToggleEvent(lv_event_t *e)
{
    lv_obj_t *target = e ? lv_event_get_target(e) : nullptr;
    if (target == lvglScreenLockSetupCancelBtn) {
        screenLockEnabled = false;
        screenLockPin[0] = '\0';
        screenLockPersistConfig();
        screenLockUnlocked = true;
        screenLockFailedTotal = 0;
        screenLockBlockedUntilMs = 0;
        screenLockHideSetupModal(true);
        lvglRefreshStyleUi();
        return;
    }
    if (!target || target != lvglStyleScreenLockSw) return;

    const bool enabled = lv_obj_has_state(target, LV_STATE_CHECKED);
    if (!enabled) {
        screenLockEnabled = false;
        screenLockPin[0] = '\0';
        screenLockPersistConfig();
        screenLockUnlocked = true;
        screenLockFailedTotal = 0;
        screenLockBlockedUntilMs = 0;
        screenLockHideSetupModal(false);
        lvglRefreshStyleUi();
        return;
    }

    screenLockShowSetupModal();
}

void lvglScreenLockUnlockEvent(lv_event_t *e)
{
    const lv_obj_t *target = e ? lv_event_get_target(e) : nullptr;
    if (target == lvglScreenLockSetupSaveBtn || (!e && lvglScreenLockSetupPinTa && lvglKb && lv_keyboard_get_textarea(lvglKb) == lvglScreenLockSetupPinTa)) {
        if (!lvglScreenLockSetupPinTa) return;
        screenLockSanitizeTextArea(lvglScreenLockSetupPinTa);
        String pin = lv_textarea_get_text(lvglScreenLockSetupPinTa);
        if (pin.length() != 4) {
            if (lvglScreenLockSetupStatusLabel) lv_label_set_text(lvglScreenLockSetupStatusLabel, "PIN must be exactly 4 digits.");
            return;
        }
        copyTextToBuf(screenLockPin, sizeof(screenLockPin), pin);
        screenLockEnabled = true;
        screenLockPersistConfig();
        screenLockUnlocked = true;
        screenLockFailedTotal = 0;
        screenLockBlockedUntilMs = 0;
        screenLockHideSetupModal(false);
        if (lvglStyleScreenLockSw) lv_obj_add_state(lvglStyleScreenLockSw, LV_STATE_CHECKED);
        uiStatusLine = "Screen lock enabled";
        lvglSyncStatusLine();
        lvglRefreshStyleUi();
        return;
    }

    if (!lvglScreenLockPinTa) return;
    screenLockSanitizeTextArea(lvglScreenLockPinTa);
    if (screenLockIsBlocked()) {
        lvglRefreshLockScreenUi();
        return;
    }

    const String entered = lv_textarea_get_text(lvglScreenLockPinTa);
    if (entered == String(screenLockPin)) {
        screenLockUnlocked = true;
        screenLockFailedTotal = 0;
        screenLockBlockedUntilMs = 0;
        lv_textarea_set_text(lvglScreenLockPinTa, "");
        uiStatusLine = "Screen unlocked";
        lvglSyncStatusLine();
        lvglOpenScreen(UI_HOME, LV_SCR_LOAD_ANIM_FADE_ON);
        return;
    }

    screenLockFailedTotal = static_cast<uint8_t>(min<int>(255, screenLockFailedTotal + 1));
    lv_textarea_set_text(lvglScreenLockPinTa, "");
    if (screenLockFailedTotal >= 3) {
        const unsigned long penaltyMinutes = static_cast<unsigned long>(screenLockFailedTotal - 2);
        screenLockBlockedUntilMs = millis() + (penaltyMinutes * 60000UL);
    }
    lvglRefreshLockScreenUi();
}

void lvglScreenLockFactoryResetEvent(lv_event_t *e)
{
    lvglFactoryResetEvent(e);
}

void lvglRefreshLockScreenUi()
{
    if (lvglScreenLockPinTa) {
        const bool blocked = screenLockIsBlocked();
        lv_textarea_set_password_mode(lvglScreenLockPinTa, true);
        if (blocked) lv_obj_add_state(lvglScreenLockPinTa, LV_STATE_DISABLED);
        else lv_obj_clear_state(lvglScreenLockPinTa, LV_STATE_DISABLED);
    }
    if (lvglScreenLockUnlockBtn && lvglScreenLockUnlockBtnLabel) {
        lvglLabelSetTextIfChanged(lvglScreenLockUnlockBtnLabel, "Unlock");
        if (screenLockIsBlocked()) lv_obj_add_state(lvglScreenLockUnlockBtn, LV_STATE_DISABLED);
        else lv_obj_clear_state(lvglScreenLockUnlockBtn, LV_STATE_DISABLED);
    }
    if (lvglScreenLockStatusLabel) {
        String status;
        if (screenLockIsBlocked()) {
            status = screenLockBlockMessage();
        } else if (screenLockFailedTotal == 0) {
            status = "Enter your 4-digit PIN.";
        } else if (screenLockFailedTotal >= 3) {
            status = "Try PIN again. Next wrong attempt increases the wait time.";
        } else {
            const int attemptsLeft = max<int>(0, 3 - screenLockFailedTotal);
            status = String("Wrong PIN. ") + attemptsLeft + " attempt" + (attemptsLeft == 1 ? "" : "s") + " left.";
        }
        lvglLabelSetTextIfChanged(lvglScreenLockStatusLabel, status);
    }
    if (lvglScreenLockResetBtn) {
        if (screenLockFailedTotal >= 3) lv_obj_clear_flag(lvglScreenLockResetBtn, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(lvglScreenLockResetBtn, LV_OBJ_FLAG_HIDDEN);
    }
    if (lvglScreenLockResetBtnLabel) lvglLabelSetTextIfChanged(lvglScreenLockResetBtnLabel, tr(TXT_FACTORY_RESET));
}

void lvglRefreshStyleUi()
{
    if (lvglStyleUiSyncing) return;
    lvglStyleUiSyncing = true;

    if (lvglStyleScreenLockSw) {
        const bool checked = lv_obj_has_state(lvglStyleScreenLockSw, LV_STATE_CHECKED);
        if (screenLockEnabled != checked) {
            if (screenLockEnabled) lv_obj_add_state(lvglStyleScreenLockSw, LV_STATE_CHECKED);
            else lv_obj_clear_state(lvglStyleScreenLockSw, LV_STATE_CHECKED);
        }
    }

    if (lvglStyleScreensaverRow) {
        if (moduleInstalled[APP_MODULE_SCREENSAVER]) {
            lv_obj_clear_flag(lvglStyleScreensaverRow, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(lvglStyleScreensaverTypeRow, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(lvglStyleScreensaverRow, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(lvglStyleScreensaverTypeRow, LV_OBJ_FLAG_HIDDEN);
        }
    }

    if (lvglStyleScreensaverSw) {
        const bool checked = lv_obj_has_state(lvglStyleScreensaverSw, LV_STATE_CHECKED);
        if (screensaverEnabled != checked) {
            if (screensaverEnabled) lv_obj_add_state(lvglStyleScreensaverSw, LV_STATE_CHECKED);
            else lv_obj_clear_state(lvglStyleScreensaverSw, LV_STATE_CHECKED);
        }
    }
    if (lvglStyleScreensaverTypeDd) {
        lv_dropdown_set_options(lvglStyleScreensaverTypeDd, buildScreensaverDropdownOptions().c_str());
        const uint16_t selected = static_cast<uint16_t>(screensaverMode);
        if (lv_dropdown_get_selected(lvglStyleScreensaverTypeDd) != selected) {
            lv_dropdown_set_selected(lvglStyleScreensaverTypeDd, selected);
        }
    }
    if (lvglStyleMenuIconsSw) {
        const bool checked = lv_obj_has_state(lvglStyleMenuIconsSw, LV_STATE_CHECKED);
        if (menuCustomIconsEnabled != checked) {
            if (menuCustomIconsEnabled) lv_obj_add_state(lvglStyleMenuIconsSw, LV_STATE_CHECKED);
            else lv_obj_clear_state(lvglStyleMenuIconsSw, LV_STATE_CHECKED);
        }
    }
    if (lvglStyleTouchVibrationSw) {
        const bool checked = lv_obj_has_state(lvglStyleTouchVibrationSw, LV_STATE_CHECKED);
        if (touchVibrationEnabled != checked) {
            if (touchVibrationEnabled) lv_obj_add_state(lvglStyleTouchVibrationSw, LV_STATE_CHECKED);
            else lv_obj_clear_state(lvglStyleTouchVibrationSw, LV_STATE_CHECKED);
        }
    }
    if (lvglStyleTouchSoundSw) {
        const bool checked = lv_obj_has_state(lvglStyleTouchSoundSw, LV_STATE_CHECKED);
        if (touchSoundEnabled != checked) {
            if (touchSoundEnabled) lv_obj_add_state(lvglStyleTouchSoundSw, LV_STATE_CHECKED);
            else lv_obj_clear_state(lvglStyleTouchSoundSw, LV_STATE_CHECKED);
        }
    }
    if (lvglStyleHomeItemsBtnLabel) {
        lvglLabelSetTextIfChanged(lvglStyleHomeItemsBtnLabel, tr(TXT_MAIN_SCREEN_ITEMS));
    }
    if (lvglStyleTimeoutSlider) {
        const int32_t sliderValue = static_cast<int32_t>(clampIdleTimeoutMs(displayIdleTimeoutMs) / LCD_IDLE_TIMEOUT_STEP_MS);
        if (lv_slider_get_value(lvglStyleTimeoutSlider) != sliderValue) {
            lv_slider_set_value(lvglStyleTimeoutSlider, sliderValue, LV_ANIM_OFF);
        }
    }
    if (lvglStyleTimeoutValueLabel) {
        lvglLabelSetTextIfChanged(lvglStyleTimeoutValueLabel, formatIdleTimeoutLabel(displayIdleTimeoutMs));
    }
    if (lvglStylePowerOffDropdown) {
        const uint16_t selected = static_cast<uint16_t>(powerOffTimeoutOptionIndex(powerOffIdleTimeoutMs));
        if (lv_dropdown_get_selected(lvglStylePowerOffDropdown) != selected) {
            lv_dropdown_set_selected(lvglStylePowerOffDropdown, selected);
        }
    }
    if (lvglStyleTimezoneDd) {
        lv_dropdown_set_options(lvglStyleTimezoneDd, buildGmtOffsetDropdownOptions().c_str());
        const uint16_t selected = static_cast<uint16_t>(topBarTimezoneGmtOffset - TOP_BAR_GMT_MIN);
        if (lv_dropdown_get_selected(lvglStyleTimezoneDd) != selected) {
            lv_dropdown_set_selected(lvglStyleTimezoneDd, selected);
        }
    }
    if (lvglStyleButtonFlatBtn && lvglStyleButtonFlatBtnLabel) {
        lvglLabelSetTextIfChanged(lvglStyleButtonFlatBtnLabel, uiButtonStyleMode == UI_BUTTON_STYLE_FLAT ? tr(TXT_SELECTED) : tr(TXT_SELECT));
        lvglApplyPersistentToggleButtonStyle(lvglStyleButtonFlatBtn,
                                             lvglStyleButtonFlatBtnLabel,
                                             uiButtonStyleMode == UI_BUTTON_STYLE_FLAT,
                                             lv_color_hex(uiButtonStyleFlatSelectorColor),
                                             lv_color_hex(uiButtonStyleFlatSelectorColor),
                                             true);
    }
    if (lvglStyleButton3dBtn && lvglStyleButton3dBtnLabel) {
        lvglLabelSetTextIfChanged(lvglStyleButton3dBtnLabel, uiButtonStyleMode == UI_BUTTON_STYLE_3D ? tr(TXT_SELECTED) : tr(TXT_SELECT));
        lvglApplyPersistentToggleButtonStyle(lvglStyleButton3dBtn,
                                             lvglStyleButton3dBtnLabel,
                                             uiButtonStyleMode == UI_BUTTON_STYLE_3D,
                                             lv_color_hex(uiButtonStyle3dSelectorColor),
                                             lv_color_hex(uiButtonStyle3dSelectorColor),
                                             true);
    }
    if (lvglStyleButtonBlackBtn && lvglStyleButtonBlackBtnLabel) {
        lvglLabelSetTextIfChanged(lvglStyleButtonBlackBtnLabel, uiButtonStyleMode == UI_BUTTON_STYLE_BLACK ? tr(TXT_SELECTED) : tr(TXT_SELECT));
        lvglApplyPersistentToggleButtonStyle(lvglStyleButtonBlackBtn,
                                             lvglStyleButtonBlackBtnLabel,
                                             uiButtonStyleMode == UI_BUTTON_STYLE_BLACK,
                                             lv_color_hex(UI_BUTTON_BLACK_BODY_HEX),
                                             lv_color_hex(UI_BUTTON_BLACK_BODY_HEX),
                                             true);
    }
    if (lvglStyleTopCenterNameBtn && lvglStyleTopCenterNameBtnLabel) {
        lvglLabelSetTextIfChanged(lvglStyleTopCenterNameBtnLabel, topBarCenterMode == TOP_BAR_CENTER_NAME ? tr(TXT_SELECTED) : tr(TXT_SELECT));
        lvglApplyPersistentToggleButtonStyle(lvglStyleTopCenterNameBtn,
                                             lvglStyleTopCenterNameBtnLabel,
                                             topBarCenterMode == TOP_BAR_CENTER_NAME,
                                             lv_color_hex(0x3F4A57),
                                             lv_color_hex(0x3F4A57),
                                             true);
    }
    if (lvglStyleTopCenterTimeBtn && lvglStyleTopCenterTimeBtnLabel) {
        lvglLabelSetTextIfChanged(lvglStyleTopCenterTimeBtnLabel, topBarCenterMode == TOP_BAR_CENTER_TIME ? tr(TXT_SELECTED) : tr(TXT_SELECT));
        lvglApplyPersistentToggleButtonStyle(lvglStyleTopCenterTimeBtn,
                                             lvglStyleTopCenterTimeBtnLabel,
                                             topBarCenterMode == TOP_BAR_CENTER_TIME,
                                             lv_color_hex(0x3F4A57),
                                             lv_color_hex(0x3F4A57),
                                             true);
    }
    lvglStyleUiSyncing = false;
}

void lvglRefreshStyleHomeItemsUi()
{
    if (lvglStyleUiSyncing) return;
    lvglStyleUiSyncing = true;

    if (lvglStyleHomeChatLabel) lvglLabelSetTextIfChanged(lvglStyleHomeChatLabel, tr(TXT_CHAT));
    if (lvglStyleHomeRadioLabel) lvglLabelSetTextIfChanged(lvglStyleHomeRadioLabel, tr(TXT_RADIO_CONTROL));
    if (lvglStyleHomeMediaLabel) lvglLabelSetTextIfChanged(lvglStyleHomeMediaLabel, tr(TXT_MEDIA));
    if (lvglStyleHomeInfoLabel) lvglLabelSetTextIfChanged(lvglStyleHomeInfoLabel, tr(TXT_INFO));
    if (lvglStyleHomeGamesLabel) lvglLabelSetTextIfChanged(lvglStyleHomeGamesLabel, tr(TXT_GAMES));
    if (lvglStyleHomeMqttLabel) lvglLabelSetTextIfChanged(lvglStyleHomeMqttLabel, tr(TXT_MQTT_CONTROLS));
    if (lvglStyleHomeConfigLabel) lvglLabelSetTextIfChanged(lvglStyleHomeConfigLabel, tr(TXT_CONFIG));
    if (lvglStyleHomePowerLabel) lvglLabelSetTextIfChanged(lvglStyleHomePowerLabel, tr(TXT_POWER));
    if (lvglStyleHomeAirplaneLabel) lvglLabelSetTextIfChanged(lvglStyleHomeAirplaneLabel, tr(TXT_AIRPLANE_MODE));
    if (lvglStyleHomeApLabel) lvglLabelSetTextIfChanged(lvglStyleHomeApLabel, tr(TXT_AP_MODE));

    auto syncSwitch = [](lv_obj_t *sw, bool enabled) {
        if (!sw) return;
        const bool checked = lv_obj_has_state(sw, LV_STATE_CHECKED);
        if (enabled != checked) {
            if (enabled) lv_obj_add_state(sw, LV_STATE_CHECKED);
            else lv_obj_clear_state(sw, LV_STATE_CHECKED);
        }
    };

    auto syncModuleRow = [&](lv_obj_t *label, lv_obj_t *sw, bool installed, bool enabled) {
        lv_obj_t *row = label ? lv_obj_get_parent(label) : (sw ? lv_obj_get_parent(sw) : nullptr);
        if (row) {
            if (installed) lv_obj_clear_flag(row, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_add_flag(row, LV_OBJ_FLAG_HIDDEN);
        }
        if (!sw) return;
        if (installed) lv_obj_clear_state(sw, LV_STATE_DISABLED);
        else lv_obj_add_state(sw, LV_STATE_DISABLED);
        syncSwitch(sw, installed && enabled);
    };

    syncModuleRow(lvglStyleHomeChatLabel, lvglStyleHomeChatSw, moduleInstalled[APP_MODULE_CHAT], homeChatVisible);
    syncModuleRow(lvglStyleHomeRadioLabel, lvglStyleHomeRadioSw, moduleInstalled[APP_MODULE_RADIO], homeRadioVisible);
    syncModuleRow(lvglStyleHomeMediaLabel, lvglStyleHomeMediaSw, moduleInstalled[APP_MODULE_MEDIA], homeMediaVisible);
    syncModuleRow(lvglStyleHomeInfoLabel, lvglStyleHomeInfoSw, moduleInstalled[APP_MODULE_INFO], homeInfoVisible);
    syncModuleRow(lvglStyleHomeGamesLabel, lvglStyleHomeGamesSw, moduleInstalled[APP_MODULE_GAMES], homeGamesVisible);
    syncModuleRow(lvglStyleHomeMqttLabel, lvglStyleHomeMqttSw, moduleInstalled[APP_MODULE_MQTT], homeMqttVisible);
    syncSwitch(lvglStyleHomeConfigSw, homeConfigVisible);
    syncSwitch(lvglStyleHomePowerSw, homePowerVisible);
    syncSwitch(lvglStyleHomeAirplaneSw, homeAirplaneVisible);
    syncSwitch(lvglStyleHomeApSw, homeApVisible);

    lvglStyleUiSyncing = false;
}

static void lvglRefreshBatteryTrainUi()
{
    if (!lvglBatteryTrainStatusLabel) return;

    lvglRefreshBatteryTrainButtonIcons();
    if (lvglBatteryTrainAutoBtn) {
        lvglRegisterStyledButton(lvglBatteryTrainAutoBtn,
                                 batteryTrainingState.autoCalibrationEnabled ? lv_color_hex(0x2F7A45) : lv_color_hex(0x345D8A),
                                 false);
    }

    String status = String("Mode: ") + batteryTrainingPhaseLabel();
    if (batteryTrainingState.active) {
        if (batteryTrainingState.phase == BATTERY_TRAIN_CHARGE) status += "  |  Charging session in progress";
        else if (batteryTrainingState.phase == BATTERY_TRAIN_DISCHARGE) status += "  |  Waiting for low-battery cutoff";
    } else {
        status += batteryTrainingState.autoCalibrationEnabled
                      ? "  |  Background learning enabled"
                      : "  |  Background learning disabled";
    }
    lvglLabelSetTextIfChanged(lvglBatteryTrainStatusLabel, status);

    char currentBuf[112];
    snprintf(currentBuf, sizeof(currentBuf), "Battery: %.3fV calibrated  |  Sense: %.3fV raw  |  %u%%",
             batteryVoltage, batteryRawVoltage, static_cast<unsigned>(batteryPercent));
    lvglLabelSetTextIfChanged(lvglBatteryTrainCurrentLabel, currentBuf);

    String fullText = "Stored FULL: ";
    fullText += batteryCalState.hasFullAnchor
                    ? String(batteryCalState.observedFullRawV, 3) + "V raw => 4.20V battery"
                    : String("Not learned");
    lvglLabelSetTextIfChanged(lvglBatteryTrainFullLabel, fullText);

    String emptyText = "Stored EMPTY: ";
    emptyText += batteryCalState.hasEmptyAnchor
                     ? String(batteryCalState.observedEmptyRawV, 3) + "V raw => 3.30V battery"
                     : String("Not learned");
    lvglLabelSetTextIfChanged(lvglBatteryTrainEmptyLabel, emptyText);

    lvglLabelSetTextIfChanged(lvglBatteryTrainFactorLabel, String("Effective factor: ") + String(batteryCalibrationFactor(), 4));

    String powerText = String("Power Off: ") + formatIdleTimeoutLabel(powerOffIdleTimeoutMs);
    if (batteryTrainingState.powerOverrideActive) {
        powerText += "  |  Saved ";
        powerText += formatIdleTimeoutLabel(batteryTrainingState.savedPowerOffMs);
    }
    lvglLabelSetTextIfChanged(lvglBatteryTrainPowerLabel, powerText);
}

static void lvglRefreshBatteryTrainButtonIcons()
{
    lvglSetMenuButtonIconMode(lvglBatteryTrainFullBtn, "FULL", LV_SYMBOL_OK, &img_full_small_icon, 6, 12);
    lvglSetMenuButtonIconMode(lvglBatteryTrainDischargeBtn, "DISCHARGE", LV_SYMBOL_DOWN, &img_disch_small_icon, 6, 12);
    lvglSetMenuButtonIconMode(lvglBatteryTrainAutoBtn,
                              batteryTrainingState.autoCalibrationEnabled ? "Auto Calibration: ON" : "Auto Calibration: OFF",
                              LV_SYMBOL_REFRESH,
                              &img_auto_small_icon,
                              6,
                              12);
}

void lvglBatteryTrainResetPromptEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    lv_obj_t *msgbox = lv_event_get_current_target(e);
    const char *btn = msgbox ? lv_msgbox_get_active_btn_text(msgbox) : nullptr;
    if (btn && strcmp(btn, "OK") == 0) {
        batteryCalibrationReset();
        batteryTrainingState.autoCalibrationEnabled = false;
        batteryTrainingStartCharge();
        uiStatusLine = "Battery training reset; connect charger and press FULL when charged";
        if (lvglReady) lvglSyncStatusLine();
        lvglRefreshBatteryTrainUi();
    }
    if (msgbox) lv_msgbox_close(msgbox);
}

void lvglFactoryResetEvent(lv_event_t *e)
{
    (void)e;

    static const char *btns[3];
    btns[0] = tr(TXT_CANCEL);
    btns[1] = tr(TXT_RESET);
    btns[2] = "";

    lv_obj_t *msgbox = lv_msgbox_create(
        nullptr,
        tr(TXT_FACTORY_RESET_TITLE),
        tr(TXT_FACTORY_RESET_BODY),
        btns,
        false
    );
    if (!msgbox) return;

    lvglApplyMsgboxModalStyle(msgbox);
    lv_obj_center(msgbox);
    lv_obj_set_width(msgbox, min<int16_t>(DISPLAY_WIDTH - 24, 280));
    lv_obj_add_event_cb(msgbox, lvglFactoryResetConfirmEvent, LV_EVENT_VALUE_CHANGED, nullptr);
}

void lvglFactoryResetConfirmEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;

    lv_obj_t *msgbox = lv_event_get_current_target(e);
    const char *btn = msgbox ? lv_msgbox_get_active_btn_text(msgbox) : nullptr;

    if (btn && strcmp(btn, tr(TXT_RESET)) == 0) {
        if (msgbox) lv_msgbox_close(msgbox);

        uiStatusLine = tr(TXT_FACTORY_RESET_DONE);
        lvglSyncStatusLine();

        factoryResetWipeStoredData();

        rebootRequested = true;
        rebootRequestedAtMs = millis();
        return;
    }

    if (msgbox) lv_msgbox_close(msgbox);
}

static void factoryResetClearNamespace(Preferences &prefs, const char *ns)
{
    if (!ns || !*ns) return;
    if (!prefs.begin(ns, false)) return;
    prefs.clear();
    prefs.end();
}

bool deletePathRecursive(const String &path, bool keepRoot = false)
{
    File f = SD.open(path);
    if (!f) return false;
    bool isDir = f.isDirectory();
    f.close();

    if (!isDir) return SD.remove(path);

    File dir = SD.open(path);
    if (!dir) return false;
    File entry = dir.openNextFile();
    while (entry) {
        String child = entry.name();
        if (!child.startsWith("/")) child = (path == "/") ? ("/" + child) : (path + "/" + child);
        entry.close();
        deletePathRecursive(child, false);
        entry = dir.openNextFile();
    }
    dir.close();
    if (keepRoot || path == "/") return true;
    return SD.rmdir(path);
}

static void factoryResetWipeSdData()
{
    if (!sdEnsureMounted()) return;

    if (SD.exists(CHAT_LOG_DIR)) {
        deletePathRecursive(CHAT_LOG_DIR, false);
    }

    if (SD.exists(CHAT_OUTBOX_PATH)) {
        SD.remove(CHAT_OUTBOX_PATH);
    }

    if (SD.exists("/recycle")) {
        deletePathRecursive("/recycle", false);
    }
}

static void factoryResetWipeStoredData()
{
    stopDnsForAp();
    stopWebServerRuntime();

    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);

    factoryResetClearNamespace(wifiPrefs, "wifi");
    factoryResetClearNamespace(uiPrefs, "ui");
    factoryResetClearNamespace(batteryPrefs, "battery");
    factoryResetClearNamespace(gamePrefs, "games");
    factoryResetClearNamespace(mqttPrefs, "mqtt");
    factoryResetClearNamespace(p2pPrefs, "p2p");

    factoryResetWipeSdData();
    screenLockEnabled = false;
    screenLockPin[0] = '\0';
    screenLockResetState();
}

void lvglBatteryTrainResetEvent(lv_event_t *e)
{
    (void)e;
    static const char *btns[] = {"Cancel", "OK", ""};
    lv_obj_t *msgbox = lv_msgbox_create(nullptr,
                                        "Battery Training",
                                        "Connect charger first. Press OK to clear stored FULL and EMPTY anchors, switch Power Off to Never, and start charge training.",
                                        btns,
                                        false);
    if (!msgbox) return;
    lvglApplyMsgboxModalStyle(msgbox);
    lv_obj_center(msgbox);
    lv_obj_add_event_cb(msgbox, lvglBatteryTrainResetPromptEvent, LV_EVENT_VALUE_CHANGED, nullptr);
}

void lvglBatteryTrainFullConfirmEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    lv_obj_t *msgbox = lv_event_get_current_target(e);
    const char *btn = msgbox ? lv_msgbox_get_active_btn_text(msgbox) : nullptr;
    if (btn && strcmp(btn, "FULL") == 0) {
        batteryTrainingState.autoCalibrationEnabled = false;
        batteryCalibrationForceFull(batteryRawVoltage);
        batteryCalibrationLoad();
        batteryVoltage = batteryCalibratedVoltageFromRaw(batteryRawVoltage);
        batteryPercent = batteryPercentFromVoltage(batteryVoltage);
        batteryFilterInitialized = true;
        chargePrevVoltage = batteryVoltage;
        lastChargeEvalMs = millis();
        batteryTrainingStopManual();
        uiStatusLine = "Stored FULL anchor and forced current reading to full scale";
        if (lvglReady) lvglSyncStatusLine();
        lvglRefreshTopIndicators();
        lvglRefreshBatteryTrainUi();
    }
    if (msgbox) lv_msgbox_close(msgbox);
}

void lvglBatteryTrainFullEvent(lv_event_t *e)
{
    (void)e;
    static const char *btns[] = {"Cancel", "FULL", ""};
    lv_obj_t *msgbox = lv_msgbox_create(nullptr,
                                        "Confirm FULL",
                                        "Store the current battery raw reading as the FULL anchor and recalibrate to the top of the curve?",
                                        btns,
                                        false);
    if (!msgbox) return;
    lvglApplyMsgboxModalStyle(msgbox);
    lv_obj_center(msgbox);
    lv_obj_add_event_cb(msgbox, lvglBatteryTrainFullConfirmEvent, LV_EVENT_VALUE_CHANGED, nullptr);
}

void lvglBatteryTrainDischargeEvent(lv_event_t *e)
{
    (void)e;
    batteryTrainingStartDischarge();
    uiStatusLine = "Discharge training armed until the next low-battery shutdown";
    if (lvglReady) lvglSyncStatusLine();
    lvglRefreshBatteryTrainUi();
}

void lvglBatteryTrainAutoConfirmEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    lv_obj_t *msgbox = lv_event_get_current_target(e);
    const char *btn = msgbox ? lv_msgbox_get_active_btn_text(msgbox) : nullptr;
    if (btn && strcmp(btn, "Enable") == 0) {
        batteryTrainingStopManual();
        batteryTrainingState.autoCalibrationEnabled = true;
        batteryTrainingSave();
        uiStatusLine = "Battery auto calibration enabled";
        if (lvglReady) lvglSyncStatusLine();
        lvglRefreshBatteryTrainUi();
    }
    if (msgbox) lv_msgbox_close(msgbox);
}

void lvglBatteryTrainAutoEvent(lv_event_t *e)
{
    (void)e;
    if (batteryTrainingState.autoCalibrationEnabled) {
        batteryTrainingStopManual();
        batteryTrainingState.autoCalibrationEnabled = false;
        batteryTrainingSave();
        uiStatusLine = "Battery auto calibration disabled";
        if (lvglReady) lvglSyncStatusLine();
        lvglRefreshBatteryTrainUi();
        return;
    }
    static const char *btns[] = {"Enable", "Cancel", ""};
    lv_obj_t *msgbox = lv_msgbox_create(nullptr,
                                        "Battery Auto Calibration",
                                        "Enable automatic charge-curve and low-battery learning in the background?",
                                        btns,
                                        true);
    if (!msgbox) return;
    lv_obj_center(msgbox);
    lv_obj_add_event_cb(msgbox, lvglBatteryTrainAutoConfirmEvent, LV_EVENT_VALUE_CHANGED, nullptr);
    lvglApplyMsgboxModalStyle(msgbox);
}

void lvglAirplaneToggleEvent(lv_event_t *e)
{
    (void)e;
    applyAirplaneMode(!airplaneModeEnabled, "lvgl_config");
}

void lvglBrightnessEvent(lv_event_t *e)
{
    (void)e;
    if (!lvglBrightnessSlider) return;
    displayBrightnessPercent = static_cast<uint8_t>(lv_slider_get_value(lvglBrightnessSlider));
    if (displayAwake) displayBacklightSet(displayBacklightLevelFromPercent(displayBrightnessPercent));
    uiPrefs.begin("ui", false);
    uiPrefs.putUChar("disp_bri", displayBrightnessPercent);
    uiPrefs.end();
    lvglRefreshConfigUi();
}

void lvglConfigVolumeEvent(lv_event_t *e)
{
    (void)e;
    if (!lvglVolumeSlider) return;
    mediaVolumePercent = static_cast<uint8_t>(lv_slider_get_value(lvglVolumeSlider));
    saveSoundPrefs();
    if (audioBackendReady && audio) audioSetVolumeImmediate(audioVolumeLevelFromPercent(mediaVolumePercent));
    lvglRefreshMediaPlayerUi();
    lvglRefreshConfigUi();
    lvglRefreshSoundPopupUi();
    lvglTopIndicatorStateValid = false;
    lvglRefreshTopIndicators();
}

void lvglSoundPopupBackdropEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (lv_event_get_target(e) != lvglSoundPopup) return;
    lvglHideSoundPopup();
}

void lvglSoundPopupVolumeEvent(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    if (!obj || obj != lvglSoundPopupVolumeSlider) return;
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

void lvglSoundPopupVibrationEvent(lv_event_t *e)
{
    (void)e;
    if (!lvglSoundPopupVibrationDropdown) return;
    const uint16_t selected = lv_dropdown_get_selected(lvglSoundPopupVibrationDropdown);
    if (selected >= static_cast<uint16_t>(VIBRATION_INTENSITY_COUNT)) return;
    vibrationIntensity = static_cast<VibrationIntensity>(selected);
    vibrationEnabled = true;
    saveSoundPrefs();
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    chatMessageVibrationPreview();
#endif
    lvglRefreshConfigUi();
    lvglRefreshSoundPopupUi();
    lvglTopIndicatorStateValid = false;
    lvglRefreshTopIndicators();
}

void lvglSoundPopupDisableVibrationEvent(lv_event_t *e)
{
    (void)e;
    vibrationEnabled = false;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    chatMessageVibrationStop(true);
#endif
    saveSoundPrefs();
    lvglRefreshConfigUi();
    lvglRefreshSoundPopupUi();
    lvglTopIndicatorStateValid = false;
    lvglRefreshTopIndicators();
}

void lvglRgbLedEvent(lv_event_t *e)
{
    (void)e;
    if (!lvglRgbLedSlider) return;
    rgbLedPercent = static_cast<uint8_t>(lv_slider_get_value(lvglRgbLedSlider));
    uiPrefs.begin("ui", false);
    uiPrefs.putUChar("rgb_led", rgbLedPercent);
    uiPrefs.end();
    rgbService();
    lvglRefreshConfigUi();
}

void lvglSaveDeviceNameEvent(lv_event_t *e)
{
    (void)e;
    if (!lvglConfigDeviceNameTa) return;
    String nextName = lv_textarea_get_text(lvglConfigDeviceNameTa);
    nextName = sanitizeDeviceShortName(nextName);
    if (nextName.isEmpty()) nextName = DEVICE_SHORT_NAME;
    deviceShortName = nextName;
    uiPrefs.begin("ui", false);
    uiPrefs.putString("dev_name", deviceShortName);
    uiPrefs.end();
    if (lvglKb) {
        lv_keyboard_set_textarea(lvglKb, nullptr);
        lv_obj_add_flag(lvglKb, LV_OBJ_FLAG_HIDDEN);
    }
    lvglSetConfigKeyboardVisible(false);
    lvglRefreshConfigUi();
    lvglRefreshChatPeerUi();
    if (uiScreen == UI_INFO) lvglRefreshInfoPanel();
    p2pLastDiscoverAnnounceMs = 0;
    p2pBroadcastDiscover();
    uiStatusLine = "Device name saved";
    lvglSyncStatusLine();
}

void lvglRefreshConfigUi()
{
    lvglRefreshPrimaryMenuButtonIcons();
    if (lvglAirplaneBtnLabel) {
        lvglLabelSetTextIfChanged(lvglAirplaneBtnLabel,
                                  menuCustomIconsEnabled ? (airplaneModeEnabled ? tr(TXT_AIRPLANE_ON) : tr(TXT_AIRPLANE_OFF))
                                                         : lvglSymbolText(LV_SYMBOL_CLOSE,
                                                                          airplaneModeEnabled ? tr(TXT_AIRPLANE_ON)
                                                                                              : tr(TXT_AIRPLANE_OFF)));
    }
    lvglApplyAirplaneButtonStyle();
    if (lvglApModeBtnLabel) {
        lvglLabelSetTextIfChanged(lvglApModeBtnLabel,
                                  menuCustomIconsEnabled ? (wifiSessionApMode ? tr(TXT_AP_MODE_ON) : tr(TXT_AP_MODE_OFF))
                                                         : lvglSymbolText(LV_SYMBOL_WIFI,
                                                                          wifiSessionApMode ? tr(TXT_AP_MODE_ON)
                                                                                            : tr(TXT_AP_MODE_OFF)));
    }
    lvglApplyApModeButtonStyle();
    if (lvglConfigDeviceNameHeader) lvglLabelSetTextIfChanged(lvglConfigDeviceNameHeader, tr(TXT_DEVICE_NAME));
    if (lvglConfigDeviceNameSaveBtnLabel) lvglLabelSetTextIfChanged(lvglConfigDeviceNameSaveBtnLabel, tr(TXT_SAVE));
    if (lvglConfigBrightnessHeader) lvglLabelSetTextIfChanged(lvglConfigBrightnessHeader, tr(TXT_BRIGHTNESS));
    if (lvglConfigVolumeHeader) lvglLabelSetTextIfChanged(lvglConfigVolumeHeader, tr(TXT_VOLUME));
    if (lvglConfigVibrationHeader) lvglLabelSetTextIfChanged(lvglConfigVibrationHeader, "Vibration");
    if (lvglConfigMessageToneHeader) lvglLabelSetTextIfChanged(lvglConfigMessageToneHeader, "Message tone");
    if (lvglConfigStartupSoundLabel) lvglLabelSetTextIfChanged(lvglConfigStartupSoundLabel, "Play sound on start");
    if (lvglConfigRgbHeader) lvglLabelSetTextIfChanged(lvglConfigRgbHeader, tr(TXT_RGB_LED));
    if (lvglConfigModulesBtn) lv_obj_clear_flag(lvglConfigModulesBtn, LV_OBJ_FLAG_HIDDEN);
    if (lvglConfigHc12Btn) {
        if (moduleInstalled[APP_MODULE_RADIO]) lv_obj_clear_flag(lvglConfigHc12Btn, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(lvglConfigHc12Btn, LV_OBJ_FLAG_HIDDEN);
    }
    if (lvglConfigRemoteBtn) {
        if (moduleInstalled[APP_MODULE_RADIO]) lv_obj_clear_flag(lvglConfigRemoteBtn, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(lvglConfigRemoteBtn, LV_OBJ_FLAG_HIDDEN);
    }
    if (lvglConfigMqttBtn) {
        if (moduleInstalled[APP_MODULE_MQTT]) lv_obj_clear_flag(lvglConfigMqttBtn, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(lvglConfigMqttBtn, LV_OBJ_FLAG_HIDDEN);
    }
    if (lvglConfigDeviceNameTa) {
        String current = lv_textarea_get_text(lvglConfigDeviceNameTa);
        if (current != deviceShortNameValue()) lv_textarea_set_text(lvglConfigDeviceNameTa, deviceShortNameValue().c_str());
    }
    if (lvglBrightnessSlider && lv_slider_get_value(lvglBrightnessSlider) != displayBrightnessPercent) {
        lv_slider_set_value(lvglBrightnessSlider, displayBrightnessPercent, LV_ANIM_OFF);
    }
    if (lvglBrightnessValueLabel) lvglLabelSetTextIfChanged(lvglBrightnessValueLabel, String(displayBrightnessPercent) + "%");
    if (lvglVolumeSlider && lv_slider_get_value(lvglVolumeSlider) != mediaVolumePercent) {
        lv_slider_set_value(lvglVolumeSlider, mediaVolumePercent, LV_ANIM_OFF);
    }
    if (lvglVolumeValueLabel) lvglLabelSetTextIfChanged(lvglVolumeValueLabel, String(mediaVolumePercent) + "%");
    if (lvglVibrationDropdown) {
        lv_dropdown_set_options(lvglVibrationDropdown, buildVibrationIntensityDropdownOptions().c_str());
        const uint16_t selected = static_cast<uint16_t>(vibrationIntensity);
        if (lv_dropdown_get_selected(lvglVibrationDropdown) != selected) lv_dropdown_set_selected(lvglVibrationDropdown, selected);
    }
    if (lvglMessageToneDropdown) {
        lv_dropdown_set_options(lvglMessageToneDropdown, buildMessageBeepDropdownOptions().c_str());
        const uint16_t selected = static_cast<uint16_t>(messageBeepTone);
        if (lv_dropdown_get_selected(lvglMessageToneDropdown) != selected) lv_dropdown_set_selected(lvglMessageToneDropdown, selected);
    }
    if (lvglStartupSoundSw) {
        if (startupSoundEnabled) lv_obj_add_state(lvglStartupSoundSw, LV_STATE_CHECKED);
        else lv_obj_clear_state(lvglStartupSoundSw, LV_STATE_CHECKED);
    }
    if (lvglRgbLedSlider && lv_slider_get_value(lvglRgbLedSlider) != rgbLedPercent) {
        lv_slider_set_value(lvglRgbLedSlider, rgbLedPercent, LV_ANIM_OFF);
    }
    lvglRefreshSoundPopupUi();
    if (lvglKeyboardVisible() && lvglKeyboardFocusedTa && lvglKeyboardFocusedTa != lvglChatInputTa) {
        lvglKeyboardRefreshLayout();
    }
}

void lvglRefreshModulesUi()
{
    if (lvglModulesStatusLabel) {
        const String text = modulesStatusText.isEmpty() ? String("Available modules are fetched from GitHub when this screen opens.") : modulesStatusText;
        lvglLabelSetTextIfChanged(lvglModulesStatusLabel, text);
    }

    for (uint8_t i = 0; i < APP_MODULE_COUNT; ++i) {
        lv_obj_t *card = nullptr;
        if (lvglModulesNameLabels[i]) card = lv_obj_get_parent(lvglModulesNameLabels[i]);

        const bool shouldShow = moduleAvailable[i] || moduleInstalled[i];
        if (card) {
            if (shouldShow) lv_obj_clear_flag(card, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_add_flag(card, LV_OBJ_FLAG_HIDDEN);
        }

        if (!shouldShow) continue;

        if (lvglModulesNameLabels[i]) lvglLabelSetTextIfChanged(lvglModulesNameLabels[i], APP_MODULE_TITLES[i]);
        if (lvglModulesDescLabels[i]) lvglLabelSetTextIfChanged(lvglModulesDescLabels[i], APP_MODULE_DESCRIPTIONS[i]);
        if (lvglModulesVersionLabels[i]) {
            String line = moduleInstalled[i] ? String("Installed") : String("Not installed");
            if (moduleAvailable[i]) {
                line += moduleRemoteVersion[i].isEmpty() ? " | GitHub package available" : String(" | GitHub: ") + moduleRemoteVersion[i];
            } else {
                line += " | local install";
            }
            lvglLabelSetTextIfChanged(lvglModulesVersionLabels[i], line);
        }
        if (!lvglModulesActionBtns[i] || !lvglModulesActionLabels[i]) continue;

        if (moduleInstalled[i]) {
            lv_obj_clear_state(lvglModulesActionBtns[i], LV_STATE_DISABLED);
            lvglLabelSetTextIfChanged(lvglModulesActionLabels[i], "Uninstall");
            lvglRegisterStyledButton(lvglModulesActionBtns[i], lv_color_hex(0xA33F3F), false);
        } else if (moduleAvailable[i]) {
            lv_obj_clear_state(lvglModulesActionBtns[i], LV_STATE_DISABLED);
            lvglLabelSetTextIfChanged(lvglModulesActionLabels[i], "Install");
            lvglRegisterStyledButton(lvglModulesActionBtns[i], lv_color_hex(0x3A8F4B), false);
        } else {
            lv_obj_add_state(lvglModulesActionBtns[i], LV_STATE_DISABLED);
            lvglLabelSetTextIfChanged(lvglModulesActionLabels[i], "Unavailable");
            lvglRegisterStyledButton(lvglModulesActionBtns[i], lv_color_hex(0x4E5D6C), false);
        }
    }
}

void lvglRefreshLanguageUi()
{
    if (lvglLanguageInfoLabel) lvglLabelSetTextIfChanged(lvglLanguageInfoLabel, tr(TXT_SELECT_DISPLAY_LANGUAGE));
    if (lvglLanguageDropdown) {
        lv_dropdown_set_options(lvglLanguageDropdown, buildLanguageDropdownOptions().c_str());
        const uint16_t selected = static_cast<uint16_t>(uiLanguage);
        if (lv_dropdown_get_selected(lvglLanguageDropdown) != selected) lv_dropdown_set_selected(lvglLanguageDropdown, selected);
    }
}

void lvglRefreshLocalizedUi()
{
    lvglRefreshConfigUi();
    lvglRefreshStyleUi();
    lvglRefreshLockScreenUi();
    lvglRefreshStyleHomeItemsUi();
    lvglRefreshRadioControlUi();
    lvglRefreshRemoteControlUi();
    lvglRefreshLanguageUi();
    lvglRefreshOtaUi();
    lvglTopIndicatorStateValid = false;
    if (lvglReady) lvglRefreshTopIndicators();
}

void lvglRefreshOtaUi()
{
    const int versionCmp =
        (otaLatestVersion[0] != '\0') ? otaCompareVersions(String(FW_VERSION), String(otaLatestVersion)) : 0;

    if (lvglOtaCurrentLabel) lvglLabelSetTextIfChanged(lvglOtaCurrentLabel, FW_VERSION);
    if (lvglOtaLatestLabel) {
        if (!OTA_FIRMWARE_FLASH_SUPPORTED) lvglLabelSetTextIfChanged(lvglOtaLatestLabel, "Unavailable on this board");
        else if (otaLatestVersion[0] != '\0' && versionCmp > 0) {
            lvglLabelSetTextIfChanged(lvglOtaLatestLabel, String(otaLatestVersion) + " (latest published)");
        } else if (otaLatestVersion[0] != '\0') {
            lvglLabelSetTextIfChanged(lvglOtaLatestLabel, otaLatestVersion);
        }
        else lvglLabelSetTextIfChanged(lvglOtaLatestLabel, otaUiState == OTA_UI_CHECKING ? "Checking..." : "Unknown");
    }
    if (lvglOtaStatusLabel) {
        if (!OTA_FIRMWARE_FLASH_SUPPORTED) lvglLabelSetTextIfChanged(lvglOtaStatusLabel, "Firmware image is too large for dual-slot OTA on the 4 MB layout.");
        else if (otaLatestVersion[0] != '\0' && versionCmp > 0) lvglLabelSetTextIfChanged(lvglOtaStatusLabel, "Current firmware is newer than the latest published GitHub release.");
        else if (otaStatusText.length()) lvglLabelSetTextIfChanged(lvglOtaStatusLabel, otaStatusText);
        else if (otaUpdateAvailable) lvglLabelSetTextIfChanged(lvglOtaStatusLabel, "Update available");
        else lvglLabelSetTextIfChanged(lvglOtaStatusLabel, "Idle");
    }
    const bool showProgress = otaUiState == OTA_UI_DOWNLOADING || otaUiState == OTA_UI_FINALIZING || otaUiState == OTA_UI_DONE;
    if (lvglOtaProgressBar) {
        if (showProgress) lv_obj_clear_flag(lvglOtaProgressBar, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(lvglOtaProgressBar, LV_OBJ_FLAG_HIDDEN);
        lv_bar_set_value(lvglOtaProgressBar, showProgress ? otaProgressPercent : 0, LV_ANIM_ON);
    }
    if (lvglOtaProgressLabel) {
        if (showProgress) {
            lv_obj_clear_flag(lvglOtaProgressLabel, LV_OBJ_FLAG_HIDDEN);
            lv_label_set_text_fmt(lvglOtaProgressLabel, "%u%%", static_cast<unsigned int>(otaProgressPercent));
        } else {
            lv_obj_add_flag(lvglOtaProgressLabel, LV_OBJ_FLAG_HIDDEN);
            lv_label_set_text(lvglOtaProgressLabel, "");
        }
    }
    if (lvglOtaUpdateBtn && lvglOtaUpdateBtnLabel) {
        lv_obj_clear_state(lvglOtaUpdateBtn, LV_STATE_CHECKED); // ensure no stuck flash state
        const bool busy = otaUiState == OTA_UI_CHECKING || otaUiState == OTA_UI_DOWNLOADING || otaUiState == OTA_UI_FINALIZING;
        if (busy || !OTA_FIRMWARE_FLASH_SUPPORTED) {
            lv_obj_add_flag(lvglOtaUpdateBtn, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_clear_flag(lvglOtaUpdateBtn, LV_OBJ_FLAG_HIDDEN);
            lvglLabelSetTextIfChanged(lvglOtaUpdateBtnLabel,
                                      lvglSymbolText(otaUpdateAvailable ? LV_SYMBOL_UPLOAD : LV_SYMBOL_REFRESH,
                                                     otaUpdateAvailable ? tr(TXT_OTA_UPDATE) : tr(TXT_OTA_CHECK_NOW)));
            const lv_color_t btnCol = otaUpdateAvailable ? lv_color_hex(0x3A8F4B) : lv_color_hex(0x2E6F95);
            lvglRegisterStyledButton(lvglOtaUpdateBtn, btnCol, false);
        }
    }
    if (lvglOtaReflashBtn && lvglOtaReflashBtnLabel) {
        lv_obj_clear_state(lvglOtaReflashBtn, LV_STATE_CHECKED); // ensure no stuck flash state
        const bool busy = otaUiState == OTA_UI_CHECKING || otaUiState == OTA_UI_DOWNLOADING || otaUiState == OTA_UI_FINALIZING;
        if (busy || !OTA_FIRMWARE_FLASH_SUPPORTED) {
            lv_obj_add_flag(lvglOtaReflashBtn, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_clear_flag(lvglOtaReflashBtn, LV_OBJ_FLAG_HIDDEN);
            lvglLabelSetTextIfChanged(lvglOtaReflashBtnLabel, lvglSymbolText(LV_SYMBOL_REFRESH, tr(TXT_OTA_FLUSH_SELECTED)));
            lvglRegisterStyledButton(lvglOtaReflashBtn, lv_color_hex(0x7A5C2E), false);
            if (otaSelectedReleaseIndex >= 0 && otaSelectedReleaseIndex < otaReleaseEntryCount) {
                lv_obj_clear_state(lvglOtaReflashBtn, LV_STATE_DISABLED);
            } else {
                lv_obj_add_state(lvglOtaReflashBtn, LV_STATE_DISABLED);
            }
        }
    }
    if (lvglOtaReleaseHeaderLabel) {
        lvglLabelSetTextIfChanged(lvglOtaReleaseHeaderLabel, "Published releases");
        const bool visible = OTA_FIRMWARE_FLASH_SUPPORTED && (otaUiState == OTA_UI_CHECKING || otaReleaseEntryCount > 0);
        if (visible) lv_obj_clear_flag(lvglOtaReleaseHeaderLabel, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(lvglOtaReleaseHeaderLabel, LV_OBJ_FLAG_HIDDEN);
    }
    for (uint8_t i = 0; i < OTA_RELEASE_LIST_MAX; ++i) {
        lv_obj_t *cb = lvglOtaReleaseChecks[i];
        if (!cb) continue;
        const bool busy = otaUiState == OTA_UI_DOWNLOADING || otaUiState == OTA_UI_FINALIZING;
        const bool available = i < otaReleaseEntryCount;
        if (!OTA_FIRMWARE_FLASH_SUPPORTED || (!available && otaUiState != OTA_UI_CHECKING)) {
            lv_obj_add_flag(cb, LV_OBJ_FLAG_HIDDEN);
            continue;
        }
        lv_obj_clear_flag(cb, LV_OBJ_FLAG_HIDDEN);
        if (available && otaSelectedReleaseIndex == static_cast<int8_t>(i)) lv_obj_add_state(cb, LV_STATE_CHECKED);
        else lv_obj_clear_state(cb, LV_STATE_CHECKED);
        if (busy) lv_obj_add_state(cb, LV_STATE_DISABLED);
        else lv_obj_clear_state(cb, LV_STATE_DISABLED);
        lv_obj_set_style_text_color(cb, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
        lv_obj_set_style_text_font(cb, &lv_font_montserrat_18, LV_PART_MAIN);
        lv_obj_set_style_text_opa(cb, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_text_color(cb, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
        lv_obj_set_style_border_color(cb, lv_color_hex(0xD9E2EC), LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(cb, lv_color_hex(0x52B788), LV_PART_INDICATOR | LV_STATE_CHECKED);
        if (available) {
            lv_checkbox_set_text(cb, otaReleaseEntries[i].tag.c_str());
        } else {
            lv_checkbox_set_text(cb, tr(TXT_OTA_SELECT_RELEASE));
        }
    }
}

