void lvglRefreshHc12Ui()
{
    const bool setAsserted = hc12SetIsAsserted();
    lv_obj_t *setBtn = hc12SetBtnObj();
    lv_obj_t *setBtnLabel = setBtn ? lv_obj_get_child(setBtn, 0) : nullptr;
    if (setBtnLabel) lv_label_set_text(setBtnLabel, setAsserted ? "CFG: ON" : "CFG: OFF");
    if (setBtn) {
        lvglRegisterStyledButton(setBtn, setAsserted ? lv_color_hex(0xC06C2B) : lv_color_hex(0x7A5C2E), true);
    }
    lv_obj_t *wrap = hc12WrapObj();
    lv_obj_t *topRow = wrap ? lv_obj_get_child(wrap, 0) : nullptr;
    lv_obj_t *pinLbl = lvglHc12TerminalPinLabel ? lvglHc12TerminalPinLabel : (topRow ? lv_obj_get_child(topRow, 1) : nullptr);
    if (pinLbl) {
        if (radioModuleType == RADIO_MODULE_HC12) {
            lv_label_set_text_fmt(pinLbl, "ESP RX %d  TX %d  |  SET %d  |  %lu baud",
                                  hc12ActiveRxPin(), hc12ActiveTxPin(), hc12ActiveSetPin(), hc12RuntimeBaud());
        } else {
            lv_label_set_text_fmt(pinLbl, "ESP RX %d  TX %d  |  M0 %d  M1 %d  |  %lu baud",
                                  e220ActiveRxPin(), e220ActiveTxPin(), e220ActiveM0Pin(), e220ActiveM1Pin(), e220RuntimeBaud());
        }
    }
    if (lvglHc12TerminalExamplesLabel) lv_label_set_text(lvglHc12TerminalExamplesLabel, hc12TerminalExampleCommands().c_str());
    lv_obj_t *terminalTa = hc12TerminalObj();
    if (terminalTa && lv_obj_is_valid(terminalTa)) {
        lv_textarea_set_text(terminalTa, hc12TerminalLog ? hc12TerminalLog->c_str() : "");
        lv_textarea_set_cursor_pos(terminalTa, LV_TEXTAREA_CURSOR_LAST);
        lv_textarea_set_placeholder_text(terminalTa, radioModuleType == RADIO_MODULE_HC12 ? "HC-12 terminal output" : "E220 terminal output");
    }

    lv_obj_t *cmdTa = hc12CmdTaObj();
    if (cmdTa && lv_obj_is_valid(cmdTa)) {
        lv_textarea_set_placeholder_text(cmdTa,
                                        radioModuleType == RADIO_MODULE_HC12 ? "AT, AT+RX, AT+RC..." : "AT, AT+CHANNEL=?, AT+UART=?...");
    }
}

void lvglRefreshHc12ConfigUi()
{
    if (lvglRadioModuleDropdown) {
        lv_dropdown_set_options(lvglRadioModuleDropdown, buildRadioModuleDropdownOptions().c_str());
        const uint16_t selected = static_cast<uint16_t>(radioModuleType);
        if (lv_dropdown_get_selected(lvglRadioModuleDropdown) != selected) lv_dropdown_set_selected(lvglRadioModuleDropdown, selected);
    }
    if (lvglRadioChannelTitleLabel) lv_label_set_text(lvglRadioChannelTitleLabel, tr(TXT_CHANNEL));
    if (lvglRadioBaudTitleLabel) lv_label_set_text(lvglRadioBaudTitleLabel, tr(TXT_BAUD_RATE));
    if (lvglRadioModeTitleLabel) lv_label_set_text(lvglRadioModeTitleLabel, radioModuleType == RADIO_MODULE_HC12 ? "Transmission Mode" : "Air Data Rate");
    if (lvglRadioPowerTitleLabel) lv_label_set_text(lvglRadioPowerTitleLabel, "Transmission Power");
    if (radioModuleType == RADIO_MODULE_HC12) {
        if (lvglRadioExtraCard) lv_obj_add_flag(lvglRadioExtraCard, LV_OBJ_FLAG_HIDDEN);
        if (lvglHc12ChannelValueLabel) lv_label_set_text_fmt(lvglHc12ChannelValueLabel, "CH%03d", hc12CurrentChannel);
        if (lvglHc12ChannelSubLabel) {
            String spacing = String((hc12CurrentChannel - 1) * 400UL) + " kHz offset";
            lv_label_set_text(lvglHc12ChannelSubLabel, spacing.c_str());
        }
        const int baudCount = static_cast<int>(sizeof(HC12_SUPPORTED_BAUDS) / sizeof(HC12_SUPPORTED_BAUDS[0]));
        const int baudIndex = constrain(hc12CurrentBaudIndex, 0, max(0, baudCount - 1));
        if (lvglHc12BaudValueLabel) lv_label_set_text_fmt(lvglHc12BaudValueLabel, "%lu", HC12_SUPPORTED_BAUDS[baudIndex]);
        if (lvglHc12BaudSubLabel) lv_label_set_text(lvglHc12BaudSubLabel, "bps");
        if (lvglHc12ModeValueLabel) lv_label_set_text(lvglHc12ModeValueLabel, HC12_MODE_LABELS[constrain(hc12CurrentModeIndex, 0, 3)]);
        if (lvglHc12ModeSubLabel) lv_label_set_text_fmt(lvglHc12ModeSubLabel, "FU%d", constrain(hc12CurrentModeIndex, 0, 3) + 1);
        const int powerLevel = constrain(hc12CurrentPowerLevel, 1, 8);
        if (lvglHc12PowerValueLabel) lv_label_set_text_fmt(lvglHc12PowerValueLabel, "P%d", powerLevel);
        if (lvglHc12PowerSubLabel) lv_label_set_text_fmt(lvglHc12PowerSubLabel, "%d dBm", HC12_POWER_DBM[powerLevel - 1]);
        if (lvglRadioPinSwapCard) lv_obj_clear_flag(lvglRadioPinSwapCard, LV_OBJ_FLAG_HIDDEN);
        if (lvglRadioPinSwapTitleLabel) lv_label_set_text(lvglRadioPinSwapTitleLabel, "UART Pins");
        if (lvglRadioPinSwapValueLabel) lv_label_set_text(lvglRadioPinSwapValueLabel, hc12SwapUartPins ? "Swapped" : "Normal");
        if (lvglRadioPinSwapSubLabel) lv_label_set_text_fmt(lvglRadioPinSwapSubLabel, "ESP RX %d  |  TX %d", hc12ActiveRxPin(), hc12ActiveTxPin());
        if (lvglRadioModePinSwapCard) lv_obj_add_flag(lvglRadioModePinSwapCard, LV_OBJ_FLAG_HIDDEN);
        if (lvglRadioHintLabel) lv_label_set_text_fmt(lvglRadioHintLabel,
                                                      "HC-12 RXD %d  |  TXD %d  |  SET %d  |  %lu baud\nLegacy gate controllers use CH100 + 2400 baud. Serial Terminal and Info use HC-12 AT mode.",
                                                      hc12ActiveTxPin(), hc12ActiveRxPin(), hc12ActiveSetPin(), hc12RuntimeBaud());
    } else {
        if (lvglRadioExtraCard) lv_obj_clear_flag(lvglRadioExtraCard, LV_OBJ_FLAG_HIDDEN);
        if (lvglHc12ChannelValueLabel) lv_label_set_text_fmt(lvglHc12ChannelValueLabel, "CH%02d", e220CurrentChannel);
        if (lvglHc12ChannelSubLabel) lv_label_set_text(lvglHc12ChannelSubLabel, "410 MHz base + channel");
        if (lvglHc12BaudValueLabel) lv_label_set_text_fmt(lvglHc12BaudValueLabel, "%lu", E220_UART_BAUD_VALUES[constrain(e220CurrentBaudIndex, 0, 7)]);
        if (lvglHc12BaudSubLabel) lv_label_set_text(lvglHc12BaudSubLabel, "bps");
        if (lvglHc12ModeValueLabel) lv_label_set_text(lvglHc12ModeValueLabel, E220_AIR_RATE_LABELS[constrain(e220CurrentAirRateIndex, 0, 7)]);
        if (lvglHc12ModeSubLabel) lv_label_set_text(lvglHc12ModeSubLabel, "air rate");
        if (lvglHc12PowerValueLabel) lv_label_set_text_fmt(lvglHc12PowerValueLabel, "P%d", e220CurrentPowerIndex);
        if (lvglHc12PowerSubLabel) lv_label_set_text_fmt(lvglHc12PowerSubLabel, "%d dBm",
                                                         E220_POWER_DBM[constrain(e220CurrentPowerIndex, 0, 3)]);
        if (lvglRadioExtraTitleLabel) lv_label_set_text(lvglRadioExtraTitleLabel, "Transfer Mode");
        if (lvglRadioExtraValueLabel) lv_label_set_text(lvglRadioExtraValueLabel, e220CurrentFixedTransmission ? "Fixed" : "Transparent");
        if (lvglRadioExtraSubLabel) lv_label_set_text(lvglRadioExtraSubLabel, e220CurrentFixedTransmission ? "packet addressing, chat off" : "packet streaming, chat on");
        if (lvglRadioPinSwapCard) lv_obj_clear_flag(lvglRadioPinSwapCard, LV_OBJ_FLAG_HIDDEN);
        if (lvglRadioPinSwapTitleLabel) lv_label_set_text(lvglRadioPinSwapTitleLabel, "UART Pins");
        if (lvglRadioPinSwapValueLabel) lv_label_set_text(lvglRadioPinSwapValueLabel, e220SwapUartPins ? "Swapped" : "Normal");
        if (lvglRadioPinSwapSubLabel) lv_label_set_text_fmt(lvglRadioPinSwapSubLabel, "ESP RX %d  |  TX %d", e220ActiveRxPin(), e220ActiveTxPin());
        if (lvglRadioModePinSwapCard) lv_obj_clear_flag(lvglRadioModePinSwapCard, LV_OBJ_FLAG_HIDDEN);
        if (lvglRadioModePinSwapTitleLabel) lv_label_set_text(lvglRadioModePinSwapTitleLabel, "Mode Pins");
        if (lvglRadioModePinSwapValueLabel) lv_label_set_text(lvglRadioModePinSwapValueLabel, e220SwapModePins ? "Swapped" : "Normal");
        if (lvglRadioModePinSwapSubLabel) lv_label_set_text_fmt(lvglRadioModePinSwapSubLabel, "M0 %d  |  M1 %d", e220ActiveM0Pin(), e220ActiveM1Pin());
        if (lvglRadioHintLabel) lv_label_set_text_fmt(lvglRadioHintLabel,
                                                      "E220 RX %d  |  TX %d  |  M0 %d  |  M1 %d  |  9600 baud in config mode\nEncrypted radio chat/discovery uses E220 Transparent mode; Fixed mode keeps config access but disables chat traffic.",
                                                      e220ActiveTxPin(), e220ActiveRxPin(), e220ActiveM0Pin(), e220ActiveM1Pin());
    }
    if (lvglHc12ConfigStatusLabel) lv_label_set_text(lvglHc12ConfigStatusLabel, hc12ConfigStatusText.c_str());
}

void lvglHc12PrevChannelEvent(lv_event_t *e)
{
    (void)e;
    const int currentChannel = radioModuleType == RADIO_MODULE_HC12 ? hc12CurrentChannel : e220CurrentChannel;
    const int minChannel = radioModuleType == RADIO_MODULE_HC12 ? HC12_MIN_CHANNEL : E220_MIN_CHANNEL;
    const int maxChannel = radioModuleType == RADIO_MODULE_HC12 ? HC12_MAX_CHANNEL : E220_MAX_CHANNEL;
    queueHc12ConfigApply(HC12_CFG_APPLY_CHANNEL, currentChannel <= minChannel ? maxChannel : (currentChannel - 1));
}

void lvglHc12NextChannelEvent(lv_event_t *e)
{
    (void)e;
    const int currentChannel = radioModuleType == RADIO_MODULE_HC12 ? hc12CurrentChannel : e220CurrentChannel;
    const int minChannel = radioModuleType == RADIO_MODULE_HC12 ? HC12_MIN_CHANNEL : E220_MIN_CHANNEL;
    const int maxChannel = radioModuleType == RADIO_MODULE_HC12 ? HC12_MAX_CHANNEL : E220_MAX_CHANNEL;
    queueHc12ConfigApply(HC12_CFG_APPLY_CHANNEL, currentChannel >= maxChannel ? minChannel : (currentChannel + 1));
}

void lvglHc12PrevBaudEvent(lv_event_t *e)
{
    (void)e;
    const int baudCount = radioModuleType == RADIO_MODULE_HC12
                              ? static_cast<int>(sizeof(HC12_SUPPORTED_BAUDS) / sizeof(HC12_SUPPORTED_BAUDS[0]))
                              : static_cast<int>(sizeof(E220_UART_BAUD_VALUES) / sizeof(E220_UART_BAUD_VALUES[0]));
    int next = (radioModuleType == RADIO_MODULE_HC12 ? hc12CurrentBaudIndex : e220CurrentBaudIndex) - 1;
    if (next < 0) next = baudCount - 1;
    queueHc12ConfigApply(HC12_CFG_APPLY_BAUD, next);
}

void lvglHc12NextBaudEvent(lv_event_t *e)
{
    (void)e;
    const int baudCount = radioModuleType == RADIO_MODULE_HC12
                              ? static_cast<int>(sizeof(HC12_SUPPORTED_BAUDS) / sizeof(HC12_SUPPORTED_BAUDS[0]))
                              : static_cast<int>(sizeof(E220_UART_BAUD_VALUES) / sizeof(E220_UART_BAUD_VALUES[0]));
    int next = (radioModuleType == RADIO_MODULE_HC12 ? hc12CurrentBaudIndex : e220CurrentBaudIndex) + 1;
    if (next >= baudCount) next = 0;
    queueHc12ConfigApply(HC12_CFG_APPLY_BAUD, next);
}

void lvglHc12PrevModeEvent(lv_event_t *e)
{
    (void)e;
    int maxIndex = radioModuleType == RADIO_MODULE_HC12 ? 3 : 7;
    int next = (radioModuleType == RADIO_MODULE_HC12 ? hc12CurrentModeIndex : e220CurrentAirRateIndex) - 1;
    if (next < 0) next = maxIndex;
    queueHc12ConfigApply(HC12_CFG_APPLY_MODE, next);
}

void lvglHc12NextModeEvent(lv_event_t *e)
{
    (void)e;
    int maxIndex = radioModuleType == RADIO_MODULE_HC12 ? 3 : 7;
    int next = (radioModuleType == RADIO_MODULE_HC12 ? hc12CurrentModeIndex : e220CurrentAirRateIndex) + 1;
    if (next > maxIndex) next = 0;
    queueHc12ConfigApply(HC12_CFG_APPLY_MODE, next);
}

void lvglHc12PrevPowerEvent(lv_event_t *e)
{
    (void)e;
    int next = (radioModuleType == RADIO_MODULE_HC12 ? hc12CurrentPowerLevel : e220CurrentPowerIndex) - 1;
    if (radioModuleType == RADIO_MODULE_HC12) {
        if (next < 1) next = 8;
    } else {
        if (next < 0) next = 3;
    }
    queueHc12ConfigApply(HC12_CFG_APPLY_POWER, next);
}

void lvglHc12NextPowerEvent(lv_event_t *e)
{
    (void)e;
    int next = (radioModuleType == RADIO_MODULE_HC12 ? hc12CurrentPowerLevel : e220CurrentPowerIndex) + 1;
    if (radioModuleType == RADIO_MODULE_HC12) {
        if (next > 8) next = 1;
    } else {
        if (next > 3) next = 0;
    }
    queueHc12ConfigApply(HC12_CFG_APPLY_POWER, next);
}

void lvglRadioPrevPinSwapEvent(lv_event_t *e)
{
    (void)e;
    if (radioModuleType == RADIO_MODULE_HC12) {
        hc12SwapUartPins = !hc12SwapUartPins;
        savePersistedRadioSettings();
        hc12RestartWithCurrentPins(hc12SwapUartPins ? "HC-12 UART pins swapped" : "HC-12 UART pins restored");
    } else {
        e220SwapUartPins = !e220SwapUartPins;
        savePersistedRadioSettings();
        hc12RestartWithCurrentPins(e220SwapUartPins ? "E220 UART pins swapped" : "E220 UART pins restored");
    }
    lvglRefreshHc12ConfigUi();
    lvglRefreshHc12Ui();
}

void lvglRadioNextPinSwapEvent(lv_event_t *e)
{
    lvglRadioPrevPinSwapEvent(e);
}

void lvglRadioPrevModePinSwapEvent(lv_event_t *e)
{
    (void)e;
    if (radioModuleType != RADIO_MODULE_E220) return;
    e220SwapModePins = !e220SwapModePins;
    savePersistedRadioSettings();
    hc12RestartWithCurrentPins(e220SwapModePins ? "E220 mode pins swapped" : "E220 mode pins restored");
    lvglRefreshHc12ConfigUi();
    lvglRefreshHc12Ui();
}

void lvglRadioNextModePinSwapEvent(lv_event_t *e)
{
    lvglRadioPrevModePinSwapEvent(e);
}

void lvglHc12PrevExtraEvent(lv_event_t *e)
{
    (void)e;
    if (radioModuleType != RADIO_MODULE_E220) return;
    queueHc12ConfigApply(HC12_CFG_APPLY_EXTRA, e220CurrentFixedTransmission ? 0 : 1);
}

void lvglHc12NextExtraEvent(lv_event_t *e)
{
    (void)e;
    if (radioModuleType != RADIO_MODULE_E220) return;
    queueHc12ConfigApply(HC12_CFG_APPLY_EXTRA, e220CurrentFixedTransmission ? 0 : 1);
}

void lvglHc12DefaultEvent(lv_event_t *e)
{
    (void)e;
    hc12FactoryReset();
    hc12ReadConfigSelection();
    lvglRefreshHc12ConfigUi();
}

void lvglRefreshHc12InfoUi()
{
    if (radioInfoFetchPending) {
        radioInfoVersionText = tr(TXT_RADIO_INFO_LOADING);
        radioInfoBaudText = "--";
        radioInfoChannelText = "--";
        radioInfoFuModeText = "--";
        radioInfoPowerText = "--";
        radioInfoRawText = tr(TXT_RADIO_INFO_LOADING);
    }
    applyRadioInfoLabels();
}
