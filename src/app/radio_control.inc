static constexpr uint32_t KEELOQ_NLF = 0x3A5C742EUL;
static constexpr uint16_t GATE_COUNT_INITIAL = 42000U;
static constexpr size_t REMOTE_KEY_HEX_LEN = 16U;
static constexpr int RADIO_CONTROL_MAX_BUTTONS = 12;

struct RadioControlButton {
    String name;
    uint16_t destinationId;
    int16_t onDataValue;
    int16_t offDataValue;
    bool requireConfirm;
    bool toggleEnabled;
    bool toggleState;
    bool pressActive;
    bool selected;
};

struct __attribute__((packed)) GateLegacyPacket {
    uint16_t device_id;
    uint16_t destination_id;
    uint32_t packet_id;
    uint8_t command;
    int16_t data;
};

static_assert(sizeof(GateLegacyPacket) == 11, "Legacy gate packet size mismatch");

struct RadioRuntimeSnapshot {
    RadioModuleType moduleType;
    int hc12Channel;
    int hc12BaudIndex;
    int hc12ModeIndex;
    int hc12PowerLevel;
    int e220Channel;
    int e220BaudIndex;
    int e220AirRateIndex;
    int e220PowerIndex;
    bool e220FixedTransmission;
};

static bool radioAppliedRuntimeValid = false;
static RadioRuntimeSnapshot radioAppliedRuntimeSnapshot = {};

static lv_obj_t *lvglRemotePowerLabel = nullptr;
static lv_obj_t *lvglRemotePowerDropdown = nullptr;
static lv_obj_t *lvglRemoteExtraCard = nullptr;
static lv_obj_t *lvglRemoteExtraLabel = nullptr;
static lv_obj_t *lvglRemoteExtraDropdown = nullptr;
static lv_obj_t *lvglRemoteKeyLabel = nullptr;
static lv_obj_t *lvglRemoteKeyTa = nullptr;
static lv_obj_t *lvglRemoteIdLabel = nullptr;
static lv_obj_t *lvglRemoteIdTa = nullptr;
static lv_obj_t *lvglRemoteCounterLabel = nullptr;
static lv_obj_t *lvglRemoteCounterTa = nullptr;
static lv_obj_t *lvglRemoteSaveBtn = nullptr;
static lv_obj_t *lvglRemoteSaveBtnLabel = nullptr;
static lv_obj_t *lvglRemoteStatusLabel = nullptr;
static lv_obj_t *lvglRadioControlConfigBtn = nullptr;
static lv_obj_t *lvglRadioControlConfigBtnLabel = nullptr;
static lv_obj_t *lvglRadioControlList = nullptr;
static lv_obj_t *lvglRadioButtonConfigList = nullptr;
static lv_obj_t *lvglRadioButtonAddBtn = nullptr;
static lv_obj_t *lvglRadioButtonSelBtn = nullptr;
static lv_obj_t *lvglRadioButtonDelBtn = nullptr;
static lv_obj_t *lvglRadioReceiverTestBtn = nullptr;
static lv_obj_t *lvglRadioReceiverTestTa = nullptr;
static lv_obj_t *lvglRadioReceiverTestStatusLabel = nullptr;

RadioModuleType remoteControlModuleType = RADIO_MODULE_HC12;
int remoteHc12Channel = HC12_MIN_CHANNEL;
int remoteHc12BaudIndex = 3;
int remoteHc12ModeIndex = 2;
int remoteHc12PowerLevel = 8;
int remoteE220Channel = 23;
int remoteE220BaudIndex = 3;
int remoteE220AirRateIndex = 2;
int remoteE220PowerIndex = 0;
bool remoteE220FixedTransmission = false;
String remoteControlKeyHex;
uint16_t remoteControlDeviceId = 0;
String remoteControlStatusText = "Enter a rolling-code key to enable remote sending";
RadioControlButton radioControlButtons[RADIO_CONTROL_MAX_BUTTONS] = {};
int radioControlButtonCount = 0;
bool radioButtonSelectionMode = false;
uint16_t gateRollingCount = GATE_COUNT_INITIAL;
static bool radioReceiverTestActive = false;
static bool radioReceiverTestRestoreValid = false;
static RadioRuntimeSnapshot radioReceiverTestRestoreSnapshot = {};
static String radioReceiverTestLog;
static uint8_t radioReceiverTestFrameBuf[3 + sizeof(GateLegacyPacket) + 1] = {};
static size_t radioReceiverTestFrameLen = 0;

struct RadioControlTxRequest {
    bool valid;
    int buttonIndex;
    uint16_t destinationId;
    int16_t payload;
    bool notifyOnSuccess;
    bool toggleStateValid;
    bool nextToggleState;
    bool clearPressActive;
    bool followUpValid;
    int16_t followUpPayload;
    bool followUpClearsPressActive;
};

static void savePersistedRadioControlButtons();
static void gatePersistRollingCount();
static bool gateSendLegacyPacket(uint16_t destinationId, int16_t payload, String &statusOut);

static portMUX_TYPE radioControlTxMux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t radioControlTxTaskHandle = nullptr;
static RadioControlTxRequest radioControlTxPending = {};
static RadioControlTxRequest radioControlTxActive = {};
static RadioControlTxRequest radioControlTxCompleted = {};
static bool radioControlTxBusy = false;
static bool radioControlTxCompletedReady = false;
static bool radioControlTxCompletedSuccess = false;
static char radioControlTxCompletedStatus[96] = {};

static bool remoteControlUsesHc12()
{
    return remoteControlModuleType == RADIO_MODULE_HC12;
}

static String remoteHexOnly(const String &raw)
{
    String cleaned;
    cleaned.reserve(raw.length());
    for (size_t i = 0; i < raw.length(); ++i) {
        const char c = raw[i];
        if (isxdigit(static_cast<unsigned char>(c))) cleaned += static_cast<char>(toupper(static_cast<unsigned char>(c)));
    }
    return cleaned;
}

static bool remoteParseKey(uint32_t &keyHigh, uint32_t &keyLow, String &statusOut)
{
    const String cleaned = remoteHexOnly(remoteControlKeyHex);
    if (cleaned.isEmpty()) {
        statusOut = "Enter a rolling-code key in Remote Control";
        return false;
    }
    if (cleaned.length() != REMOTE_KEY_HEX_LEN) {
        statusOut = "Rolling-code key must be 16 hex characters";
        return false;
    }
    keyHigh = strtoul(cleaned.substring(0, 8).c_str(), nullptr, 16);
    keyLow = strtoul(cleaned.substring(8, 16).c_str(), nullptr, 16);
    return true;
}

static bool remoteParseDeviceId(uint16_t &deviceIdOut, String &statusOut)
{
    if (remoteControlDeviceId == 0U) {
        statusOut = "Enter a remote ID in Remote Control";
        return false;
    }
    deviceIdOut = remoteControlDeviceId;
    return true;
}

static bool remoteParseRollingCounter(uint16_t &counterOut, String &statusOut)
{
    if (gateRollingCount == 0U) {
        statusOut = "Enter a rolling counter in Remote Control";
        return false;
    }
    counterOut = gateRollingCount;
    return true;
}

static String buildRemoteBaudDropdownOptions(bool hc12)
{
    String options;
    const size_t count = hc12 ? (sizeof(HC12_SUPPORTED_BAUDS) / sizeof(HC12_SUPPORTED_BAUDS[0]))
                              : (sizeof(E220_UART_BAUD_VALUES) / sizeof(E220_UART_BAUD_VALUES[0]));
    for (size_t i = 0; i < count; ++i) {
        if (i) options += "\n";
        options += String(hc12 ? HC12_SUPPORTED_BAUDS[i] : E220_UART_BAUD_VALUES[i]);
    }
    return options;
}

static String buildRemoteModeDropdownOptions(bool hc12)
{
    String options;
    if (hc12) {
        for (size_t i = 0; i < sizeof(HC12_MODE_LABELS) / sizeof(HC12_MODE_LABELS[0]); ++i) {
            if (i) options += "\n";
            options += String("FU") + String(i + 1) + " - " + HC12_MODE_LABELS[i];
        }
    } else {
        for (size_t i = 0; i < sizeof(E220_AIR_RATE_LABELS) / sizeof(E220_AIR_RATE_LABELS[0]); ++i) {
            if (i) options += "\n";
            options += E220_AIR_RATE_LABELS[i];
            options += " kbps";
        }
    }
    return options;
}

static String buildRemotePowerDropdownOptions(bool hc12)
{
    String options;
    if (hc12) {
        for (size_t i = 0; i < sizeof(HC12_POWER_DBM) / sizeof(HC12_POWER_DBM[0]); ++i) {
            if (i) options += "\n";
            options += "P";
            options += String(i + 1);
            options += " (";
            options += String(HC12_POWER_DBM[i]);
            options += " dBm)";
        }
    } else {
        for (size_t i = 0; i < sizeof(E220_POWER_DBM) / sizeof(E220_POWER_DBM[0]); ++i) {
            if (i) options += "\n";
            options += String(E220_POWER_DBM[i]) + " dBm";
        }
    }
    return options;
}

static String radioControlButtonDisplayName(int index)
{
    if (index < 0 || index >= radioControlButtonCount) return "Radio Button";
    String name = radioControlButtons[index].name;
    name.trim();
    if (!name.isEmpty()) return name;
    return String("Radio Button ") + String(index + 1);
}

static bool radioControlButtonUsesLocalToggle(const RadioControlButton &btn)
{
    return btn.toggleEnabled && btn.offDataValue >= 0;
}

static bool radioControlButtonUsesMomentary(const RadioControlButton &btn)
{
    return !btn.toggleEnabled && btn.offDataValue >= 0;
}

static bool radioControlButtonNeedsReleaseTx(const RadioControlButton &btn)
{
    return radioControlButtonUsesMomentary(btn) && btn.offDataValue != btn.onDataValue;
}

static bool radioControlTxInProgress()
{
    return radioControlTxBusy || radioControlTxPending.valid;
}

static bool radioControlModuleReadyForTx()
{
    if (radioReceiverTestActive) return false;
    if (hc12ConfigApplyPending != HC12_CFG_APPLY_NONE) return false;
    if (radioInfoFetchPending || radioInfoFetchActive) return false;
    if (uiDeferredFlags & UI_DEFERRED_HC12_SETTLE_PENDING) return false;
    return true;
}

static bool radioControlEnqueueTx(const RadioControlTxRequest &request, String &statusOut)
{
    if (!request.valid) {
        statusOut = "Radio request invalid";
        return false;
    }

    bool queued = false;
    portENTER_CRITICAL(&radioControlTxMux);
    if (!radioControlTxPending.valid) {
        radioControlTxPending = request;
        queued = true;
    }
    portEXIT_CRITICAL(&radioControlTxMux);

    if (!queued) {
        statusOut = "Radio busy, wait for queued command";
        return false;
    }

    statusOut = radioControlModuleReadyForTx() ? "Radio command queued" : "Radio queued, waiting for module";
    return true;
}

static void radioControlTxTask(void *param)
{
    (void)param;

    RadioControlTxRequest request = {};
    portENTER_CRITICAL(&radioControlTxMux);
    request = radioControlTxActive;
    portEXIT_CRITICAL(&radioControlTxMux);

    String status;
    const bool success = gateSendLegacyPacket(request.destinationId, request.payload, status);

    portENTER_CRITICAL(&radioControlTxMux);
    radioControlTxCompleted = request;
    radioControlTxCompletedSuccess = success;
    snprintf(radioControlTxCompletedStatus, sizeof(radioControlTxCompletedStatus), "%s", status.c_str());
    radioControlTxCompletedReady = true;
    radioControlTxBusy = false;
    radioControlTxTaskHandle = nullptr;
    portEXIT_CRITICAL(&radioControlTxMux);

    vTaskDelete(nullptr);
}

static void serviceRadioControlTxQueue()
{
    RadioControlTxRequest completed = {};
    bool completedReady = false;
    bool completedSuccess = false;
    char completedStatus[96] = {};

    portENTER_CRITICAL(&radioControlTxMux);
    if (radioControlTxCompletedReady) {
        completed = radioControlTxCompleted;
        completedSuccess = radioControlTxCompletedSuccess;
        snprintf(completedStatus, sizeof(completedStatus), "%s", radioControlTxCompletedStatus);
        radioControlTxCompletedReady = false;
        radioControlTxCompleted.valid = false;
        radioControlTxCompletedStatus[0] = '\0';
        completedReady = true;
    }
    portEXIT_CRITICAL(&radioControlTxMux);

    if (completedReady) {
        bool refreshRadioUi = false;
        if (completed.buttonIndex >= 0 && completed.buttonIndex < radioControlButtonCount) {
            RadioControlButton &btn = radioControlButtons[completed.buttonIndex];
            if (completed.clearPressActive) btn.pressActive = false;
            if (completedSuccess && completed.toggleStateValid) {
                btn.toggleState = completed.nextToggleState;
                refreshRadioUi = true;
            }
        }

        if (completedSuccess) {
            if (!displayAwake) displaySetAwake(true);
            lastUserActivityMs = millis();
            gatePersistRollingCount();
            if (completed.notifyOnSuccess) chatQueueIncomingMessageBeep();
        }

        if (completedSuccess && completed.followUpValid) {
            RadioControlTxRequest followUp = {};
            followUp.valid = true;
            followUp.buttonIndex = completed.buttonIndex;
            followUp.destinationId = completed.destinationId;
            followUp.payload = completed.followUpPayload;
            followUp.notifyOnSuccess = false;
            followUp.toggleStateValid = false;
            followUp.nextToggleState = false;
            followUp.clearPressActive = completed.followUpClearsPressActive;
            followUp.followUpValid = false;
            String queueStatus;
            if (!radioControlEnqueueTx(followUp, queueStatus) && completed.buttonIndex >= 0 && completed.buttonIndex < radioControlButtonCount) {
                radioControlButtons[completed.buttonIndex].pressActive = false;
            }
        }

        if (refreshRadioUi) {
            savePersistedRadioControlButtons();
            lvglRefreshRadioControlUi();
        }

        uiStatusLine = completedStatus;
        lvglSyncStatusLine();
    }

    if (radioControlTxBusy || radioControlTxTaskHandle != nullptr) return;
    if (!radioControlModuleReadyForTx()) return;

    RadioControlTxRequest next = {};
    bool havePending = false;
    portENTER_CRITICAL(&radioControlTxMux);
    if (radioControlTxPending.valid) {
        next = radioControlTxPending;
        radioControlTxPending.valid = false;
        radioControlTxActive = next;
        radioControlTxBusy = true;
        havePending = true;
    }
    portEXIT_CRITICAL(&radioControlTxMux);

    if (!havePending) return;

#if CONFIG_FREERTOS_UNICORE
    const BaseType_t workerCore = ARDUINO_RUNNING_CORE;
#else
    const BaseType_t workerCore = (ARDUINO_RUNNING_CORE == 0) ? 1 : 0;
#endif
    if (xTaskCreatePinnedToCore(radioControlTxTask,
                                "radio_btn_tx",
                                8192,
                                nullptr,
                                1,
                                &radioControlTxTaskHandle,
                                workerCore) != pdPASS) {
        portENTER_CRITICAL(&radioControlTxMux);
        radioControlTxBusy = false;
        radioControlTxActive.valid = false;
        radioControlTxTaskHandle = nullptr;
        portEXIT_CRITICAL(&radioControlTxMux);
        uiStatusLine = "Failed to start radio task";
        lvglSyncStatusLine();
    }
}

static bool remoteProgramActiveHc12(int currentBaudIndex,
                                    int targetChannel,
                                    int targetBaudIndex,
                                    int targetModeIndex,
                                    int targetPowerLevel,
                                    String &statusOut);
static bool remoteProgramActiveE220(int targetChannel,
                                    int targetBaudIndex,
                                    int targetAirRateIndex,
                                    int targetPowerIndex,
                                    bool targetFixedTransmission,
                                    String &statusOut);
static bool remoteApplySelectedProfile(String &statusOut);

static bool remoteApplySelectedHc12Profile(const RadioRuntimeSnapshot &targetRuntime, String &statusOut)
{
    int candidateBaudIndices[3] = {-1, -1, -1};
    size_t candidateCount = 0;
    const auto addCandidate = [&](int baudIndex) {
        const int bounded = constrain(baudIndex,
                                      0,
                                      static_cast<int>(sizeof(HC12_SUPPORTED_BAUDS) / sizeof(HC12_SUPPORTED_BAUDS[0])) - 1);
        for (size_t i = 0; i < candidateCount; ++i) {
            if (candidateBaudIndices[i] == bounded) return;
        }
        if (candidateCount < (sizeof(candidateBaudIndices) / sizeof(candidateBaudIndices[0]))) {
            candidateBaudIndices[candidateCount++] = bounded;
        }
    };

    if (radioAppliedRuntimeValid && radioAppliedRuntimeSnapshot.moduleType == RADIO_MODULE_HC12) {
        addCandidate(radioAppliedRuntimeSnapshot.hc12BaudIndex);
    }
    addCandidate(hc12CurrentBaudIndex);
    addCandidate(targetRuntime.hc12BaudIndex);

    String lastStatus;
    for (size_t i = 0; i < candidateCount; ++i) {
        if (remoteProgramActiveHc12(candidateBaudIndices[i],
                                    targetRuntime.hc12Channel,
                                    targetRuntime.hc12BaudIndex,
                                    targetRuntime.hc12ModeIndex,
                                    targetRuntime.hc12PowerLevel,
                                    lastStatus)) {
            statusOut = "";
            return true;
        }
    }

    statusOut = lastStatus;
    return false;
}

static void savePersistedRadioControlButtons()
{
    uiPrefs.begin("ui", false);
    uiPrefs.putUChar("rc_btn_count", static_cast<uint8_t>(radioControlButtonCount));
    for (int i = 0; i < RADIO_CONTROL_MAX_BUTTONS; ++i) {
        char key[24];
        snprintf(key, sizeof(key), "rc_btn_name_%02d", i);
        uiPrefs.putString(key, (i < radioControlButtonCount) ? radioControlButtons[i].name : "");
        snprintf(key, sizeof(key), "rc_btn_id_%02d", i);
        uiPrefs.putUInt(key, (i < radioControlButtonCount) ? radioControlButtons[i].destinationId : 0U);
        snprintf(key, sizeof(key), "rc_btn_on_%02d", i);
        uiPrefs.putInt(key, (i < radioControlButtonCount) ? radioControlButtons[i].onDataValue : -1);
        snprintf(key, sizeof(key), "rc_btn_off_%02d", i);
        uiPrefs.putInt(key, (i < radioControlButtonCount) ? radioControlButtons[i].offDataValue : -1);
        snprintf(key, sizeof(key), "rc_btn_cf_%02d", i);
        uiPrefs.putBool(key, (i < radioControlButtonCount) ? radioControlButtons[i].requireConfirm : false);
        snprintf(key, sizeof(key), "rc_btn_tg_%02d", i);
        uiPrefs.putBool(key,
                (i < radioControlButtonCount && radioControlButtons[i].offDataValue >= 0) ? radioControlButtons[i].toggleEnabled
                                                    : false);
        snprintf(key, sizeof(key), "rc_btn_ts_%02d", i);
        uiPrefs.putBool(key,
                        (i < radioControlButtonCount && radioControlButtonUsesLocalToggle(radioControlButtons[i])) ? radioControlButtons[i].toggleState
                                                                                                                   : false);
    }
    uiPrefs.end();
}

static void loadPersistedRadioControlButtons()
{
    radioControlButtonCount = constrain(uiPrefs.getUChar("rc_btn_count", 0), 0, RADIO_CONTROL_MAX_BUTTONS);
    for (int i = 0; i < RADIO_CONTROL_MAX_BUTTONS; ++i) {
        char key[24];
        radioControlButtons[i].onDataValue = -1;
        radioControlButtons[i].offDataValue = -1;
        snprintf(key, sizeof(key), "rc_btn_name_%02d", i);
        radioControlButtons[i].name = uiPrefs.getString(key, "");
        radioControlButtons[i].name.trim();
        if (radioControlButtons[i].name.length() > 20) radioControlButtons[i].name.remove(20);
        snprintf(key, sizeof(key), "rc_btn_id_%02d", i);
        radioControlButtons[i].destinationId = static_cast<uint16_t>(constrain(uiPrefs.getUInt(key, 0U), 0U, 65535U));
        snprintf(key, sizeof(key), "rc_btn_on_%02d", i);
        if (uiPrefs.isKey(key)) {
            radioControlButtons[i].onDataValue = constrain(uiPrefs.getInt(key, -1), -1, 99);
        } else {
            snprintf(key, sizeof(key), "rc_btn_dt_%02d", i);
            const int legacyValue = constrain(static_cast<int>(uiPrefs.getUChar(key, 0U)), 0, 99);
            radioControlButtons[i].onDataValue = legacyValue;
            radioControlButtons[i].offDataValue = (legacyValue == 1) ? 2 : -1;
        }
        snprintf(key, sizeof(key), "rc_btn_off_%02d", i);
        if (uiPrefs.isKey(key)) radioControlButtons[i].offDataValue = constrain(uiPrefs.getInt(key, -1), -1, 99);
        snprintf(key, sizeof(key), "rc_btn_cf_%02d", i);
        radioControlButtons[i].requireConfirm = uiPrefs.getBool(key, false);
        snprintf(key, sizeof(key), "rc_btn_tg_%02d", i);
        radioControlButtons[i].toggleEnabled = (radioControlButtons[i].offDataValue >= 0)
                                                   ? uiPrefs.getBool(key, true)
                                                   : false;
        snprintf(key, sizeof(key), "rc_btn_ts_%02d", i);
        radioControlButtons[i].toggleState = radioControlButtonUsesLocalToggle(radioControlButtons[i]) ? uiPrefs.getBool(key, false) : false;
        radioControlButtons[i].pressActive = false;
        radioControlButtons[i].selected = false;
    }
}

static void syncRadioButtonConfigFromUi()
{
    if (!lvglRadioButtonConfigList || !lv_obj_is_valid(lvglRadioButtonConfigList)) return;
    const uint32_t childCount = lv_obj_get_child_cnt(lvglRadioButtonConfigList);
    for (uint32_t i = 0; i < childCount && i < static_cast<uint32_t>(radioControlButtonCount); ++i) {
        lv_obj_t *row = lv_obj_get_child(lvglRadioButtonConfigList, i);
        if (!row) continue;
        lv_obj_t *selCb = lv_obj_get_child(row, 0);
        lv_obj_t *nameTa = lv_obj_get_child(row, 1);
        lv_obj_t *idTa = lv_obj_get_child(row, 2);
        lv_obj_t *onTa = lv_obj_get_child(row, 3);
        lv_obj_t *offTa = lv_obj_get_child(row, 4);
        lv_obj_t *toggleCb = lv_obj_get_child(row, 5);
        lv_obj_t *confirmCb = lv_obj_get_child(row, 6);
        if (selCb) radioControlButtons[i].selected = lv_obj_has_state(selCb, LV_STATE_CHECKED);
        if (nameTa) {
            radioControlButtons[i].name = lv_textarea_get_text(nameTa);
            radioControlButtons[i].name.trim();
            if (radioControlButtons[i].name.length() > 20) radioControlButtons[i].name.remove(20);
        }
        if (idTa) {
            const unsigned long idValue = strtoul(lv_textarea_get_text(idTa), nullptr, 10);
            radioControlButtons[i].destinationId = static_cast<uint16_t>(constrain(idValue, 0UL, 65535UL));
        }
        if (onTa) {
            const String raw = lv_textarea_get_text(onTa);
            radioControlButtons[i].onDataValue = raw.isEmpty() ? -1 : constrain(static_cast<int>(strtoul(raw.c_str(), nullptr, 10)), 0, 99);
        }
        if (offTa) {
            const String raw = lv_textarea_get_text(offTa);
            radioControlButtons[i].offDataValue = raw.isEmpty() ? -1 : constrain(static_cast<int>(strtoul(raw.c_str(), nullptr, 10)), 0, 99);
        }
        if (toggleCb) radioControlButtons[i].toggleEnabled = lv_obj_has_state(toggleCb, LV_STATE_CHECKED);
        if (confirmCb) radioControlButtons[i].requireConfirm = lv_obj_has_state(confirmCb, LV_STATE_CHECKED);
        if (radioControlButtons[i].offDataValue < 0) radioControlButtons[i].toggleEnabled = false;
        if (!radioControlButtonUsesLocalToggle(radioControlButtons[i])) radioControlButtons[i].toggleState = false;
        radioControlButtons[i].pressActive = false;
    }
}

static RadioRuntimeSnapshot captureRadioRuntimeSnapshot()
{
    RadioRuntimeSnapshot snap = {};
    snap.moduleType = radioModuleType;
    snap.hc12Channel = hc12CurrentChannel;
    snap.hc12BaudIndex = hc12CurrentBaudIndex;
    snap.hc12ModeIndex = hc12CurrentModeIndex;
    snap.hc12PowerLevel = hc12CurrentPowerLevel;
    snap.e220Channel = e220CurrentChannel;
    snap.e220BaudIndex = e220CurrentBaudIndex;
    snap.e220AirRateIndex = e220CurrentAirRateIndex;
    snap.e220PowerIndex = e220CurrentPowerIndex;
    snap.e220FixedTransmission = e220CurrentFixedTransmission;
    return snap;
}

static void restoreRadioRuntimeSnapshot(const RadioRuntimeSnapshot &snap)
{
    radioModuleType = snap.moduleType;
    hc12CurrentChannel = snap.hc12Channel;
    hc12CurrentBaudIndex = snap.hc12BaudIndex;
    hc12CurrentModeIndex = snap.hc12ModeIndex;
    hc12CurrentPowerLevel = snap.hc12PowerLevel;
    e220CurrentChannel = snap.e220Channel;
    e220CurrentBaudIndex = snap.e220BaudIndex;
    e220CurrentAirRateIndex = snap.e220AirRateIndex;
    e220CurrentPowerIndex = snap.e220PowerIndex;
    e220CurrentFixedTransmission = snap.e220FixedTransmission;
}

static RadioRuntimeSnapshot captureSelectedRemoteRuntimeSnapshot()
{
    RadioRuntimeSnapshot snap = {};
    snap.moduleType = remoteControlModuleType;
    snap.hc12Channel = constrain(remoteHc12Channel, HC12_MIN_CHANNEL, HC12_MAX_CHANNEL);
    snap.hc12BaudIndex = constrain(remoteHc12BaudIndex,
                                   0,
                                   static_cast<int>(sizeof(HC12_SUPPORTED_BAUDS) / sizeof(HC12_SUPPORTED_BAUDS[0])) - 1);
    snap.hc12ModeIndex = constrain(remoteHc12ModeIndex, 0, 3);
    snap.hc12PowerLevel = constrain(remoteHc12PowerLevel, 1, 8);
    snap.e220Channel = constrain(remoteE220Channel, E220_MIN_CHANNEL, E220_MAX_CHANNEL);
    snap.e220BaudIndex = constrain(remoteE220BaudIndex,
                                   0,
                                   static_cast<int>(sizeof(E220_UART_BAUD_VALUES) / sizeof(E220_UART_BAUD_VALUES[0])) - 1);
    snap.e220AirRateIndex = constrain(remoteE220AirRateIndex,
                                      0,
                                      static_cast<int>(sizeof(E220_AIR_RATE_OPTIONS) / sizeof(E220_AIR_RATE_OPTIONS[0])) - 1);
    snap.e220PowerIndex = constrain(remoteE220PowerIndex, 0, 3);
    snap.e220FixedTransmission = remoteE220FixedTransmission;
    return snap;
}

static bool radioRuntimeSnapshotsMatch(const RadioRuntimeSnapshot &lhs, const RadioRuntimeSnapshot &rhs)
{
    if (lhs.moduleType != rhs.moduleType) return false;

    if (lhs.moduleType == RADIO_MODULE_HC12) {
        return lhs.hc12Channel == rhs.hc12Channel &&
               lhs.hc12BaudIndex == rhs.hc12BaudIndex &&
               lhs.hc12ModeIndex == rhs.hc12ModeIndex &&
               lhs.hc12PowerLevel == rhs.hc12PowerLevel;
    }

    return lhs.e220Channel == rhs.e220Channel &&
           lhs.e220BaudIndex == rhs.e220BaudIndex &&
           lhs.e220AirRateIndex == rhs.e220AirRateIndex &&
           lhs.e220PowerIndex == rhs.e220PowerIndex &&
           lhs.e220FixedTransmission == rhs.e220FixedTransmission;
}

static void radioRememberAppliedRuntime(const RadioRuntimeSnapshot &snap, bool persist)
{
    radioAppliedRuntimeSnapshot = snap;
    radioAppliedRuntimeValid = true;
    (void)persist;
}

static void radioSyncAppliedRuntimeWithCurrentSettings()
{
    radioRememberAppliedRuntime(captureRadioRuntimeSnapshot(), false);
}

static uint32_t gateKeeLoqEncrypt(uint32_t data, uint32_t keyHigh, uint32_t keyLow)
{
    uint32_t x = data;
    for (uint32_t round = 0; round < 528U; ++round) {
        const int keyBitNo = static_cast<int>(round & 63U);
        const uint32_t keyBit = keyBitNo < 32 ? bitRead(keyLow, keyBitNo) : bitRead(keyHigh, keyBitNo - 32);
        const int index = 1 * bitRead(x, 1) + 2 * bitRead(x, 9) + 4 * bitRead(x, 20) + 8 * bitRead(x, 26) + 16 * bitRead(x, 31);
        const uint32_t bitVal = bitRead(x, 0) ^ bitRead(x, 16) ^ bitRead(KEELOQ_NLF, index) ^ keyBit;
        x = (x >> 1) ^ (bitVal << 31);
    }
    return x;
}

static uint32_t gateKeeLoqDecrypt(uint32_t data, uint32_t keyHigh, uint32_t keyLow)
{
    uint32_t x = data;
    for (int round = 527; round >= 0; --round) {
        const int keyBitNo = round & 63;
        const uint32_t keyBit = keyBitNo < 32 ? bitRead(keyLow, keyBitNo) : bitRead(keyHigh, keyBitNo - 32);
        const int index = 1 * bitRead(x, 0) + 2 * bitRead(x, 8) + 4 * bitRead(x, 19) + 8 * bitRead(x, 25) + 16 * bitRead(x, 30);
        const uint32_t bitVal = bitRead(x, 31) ^ bitRead(x, 15) ^ bitRead(KEELOQ_NLF, index) ^ keyBit;
        x = (x << 1) ^ bitVal;
    }
    return x;
}

static void gatePersistRollingCount()
{
    uiPrefs.begin("ui", false);
    uiPrefs.putUInt("gate_cnt", gateRollingCount);
    uiPrefs.end();
}

static void radioReceiverTestAppend(const String &line)
{
    radioReceiverTestLog += line;
    if (radioReceiverTestLog.length() > 2400) {
        const int trimTo = radioReceiverTestLog.length() - 2400;
        int newline = radioReceiverTestLog.indexOf('\n', trimTo);
        radioReceiverTestLog.remove(0, newline >= 0 ? (newline + 1) : trimTo);
    }
    if (lvglRadioReceiverTestTa && lv_obj_is_valid(lvglRadioReceiverTestTa)) {
        lv_textarea_set_text(lvglRadioReceiverTestTa, radioReceiverTestLog.c_str());
        lv_textarea_set_cursor_pos(lvglRadioReceiverTestTa, LV_TEXTAREA_CURSOR_LAST);
    }
}

static String radioReceiverTestDecodeCounter(const GateLegacyPacket &packet)
{
    uint32_t keyHigh = 0;
    uint32_t keyLow = 0;
    String status;
    if (!remoteParseKey(keyHigh, keyLow, status)) return String("");

    const uint32_t decoded = gateKeeLoqDecrypt(packet.packet_id, keyHigh, keyLow);
    const uint32_t decodedDeviceId = decoded % 100000UL;
    const uint32_t decodedCounter = (decoded / 100000UL) + 1UL;
    if (decodedDeviceId != static_cast<uint32_t>(packet.device_id)) return String("");
    if (decodedCounter == 0UL || decodedCounter > 65535UL) return String("");

    char decodedLine[64];
    snprintf(decodedLine,
             sizeof(decodedLine),
             " counter=%lu",
             static_cast<unsigned long>(decodedCounter));
    return String(decodedLine);
}

static String radioReceiverTestProfileSummary()
{
    if (remoteControlUsesHc12()) {
        const int baudIndex = constrain(remoteHc12BaudIndex,
                                        0,
                                        static_cast<int>(sizeof(HC12_SUPPORTED_BAUDS) / sizeof(HC12_SUPPORTED_BAUDS[0])) - 1);
        const int modeIndex = constrain(remoteHc12ModeIndex,
                                        0,
                                        static_cast<int>(sizeof(HC12_MODE_LABELS) / sizeof(HC12_MODE_LABELS[0])) - 1);
        return String("Listening on HC-12 CH")
               + String(constrain(remoteHc12Channel, HC12_MIN_CHANNEL, HC12_MAX_CHANNEL))
               + " @ "
               + String(HC12_SUPPORTED_BAUDS[baudIndex])
               + " baud / FU"
               + String(modeIndex + 1)
               + " / P"
               + String(constrain(remoteHc12PowerLevel, 1, 8));
    }

    const int baudIndex = constrain(remoteE220BaudIndex,
                                    0,
                                    static_cast<int>(sizeof(E220_UART_BAUD_VALUES) / sizeof(E220_UART_BAUD_VALUES[0])) - 1);
    const int airRateIndex = constrain(remoteE220AirRateIndex,
                                       0,
                                       static_cast<int>(sizeof(E220_AIR_RATE_LABELS) / sizeof(E220_AIR_RATE_LABELS[0])) - 1);
    const int powerIndex = constrain(remoteE220PowerIndex,
                                     0,
                                     static_cast<int>(sizeof(E220_POWER_DBM) / sizeof(E220_POWER_DBM[0])) - 1);
    return String("Listening on E220 CH")
           + String(constrain(remoteE220Channel, E220_MIN_CHANNEL, E220_MAX_CHANNEL))
           + " @ "
           + String(E220_UART_BAUD_VALUES[baudIndex])
           + " baud / "
           + String(E220_AIR_RATE_LABELS[airRateIndex])
           + " air / "
           + String(E220_POWER_DBM[powerIndex])
           + " dBm / "
           + (remoteE220FixedTransmission ? "fixed" : "transparent");
}

void lvglRefreshRadioReceiverTestUi()
{
    if (lvglRadioReceiverTestStatusLabel && lv_obj_is_valid(lvglRadioReceiverTestStatusLabel)) {
        const String status = radioReceiverTestActive ? radioReceiverTestProfileSummary() : "Receiver test inactive";
        lv_label_set_text(lvglRadioReceiverTestStatusLabel, status.c_str());
    }
    if (lvglRadioReceiverTestTa && lv_obj_is_valid(lvglRadioReceiverTestTa)) {
        lv_textarea_set_text(lvglRadioReceiverTestTa, radioReceiverTestLog.c_str());
        lv_textarea_set_cursor_pos(lvglRadioReceiverTestTa, LV_TEXTAREA_CURSOR_LAST);
    }
}

void radioReceiverTestEnterScreen()
{
    radioReceiverTestLog = "";
    radioReceiverTestFrameLen = 0;
    radioReceiverTestAppend("[RXTEST] Starting receiver test\n");
    radioReceiverTestRestoreSnapshot = captureRadioRuntimeSnapshot();
    radioReceiverTestRestoreValid = true;
    String status;
    const bool ok = remoteApplySelectedProfile(status);
    radioReceiverTestActive = ok;
    hc12RestartWithCurrentPins("");
    if (ok) {
        radioReceiverTestAppend(String("[RXTEST] ") + radioReceiverTestProfileSummary() + "\n");
        while (Serial1.available() > 0) Serial1.read();
    } else {
        radioReceiverTestAppend(String("[RXTEST] Receiver configure failed: ") + status + "\n");
    }
    lvglRefreshRadioReceiverTestUi();
}

void radioReceiverTestLeaveScreen()
{
    if (!radioReceiverTestRestoreValid) {
        radioReceiverTestActive = false;
        return;
    }
    radioReceiverTestAppend("[RXTEST] Restoring previous radio settings\n");
    String status;
    if (radioReceiverTestRestoreSnapshot.moduleType == RADIO_MODULE_HC12) {
        remoteApplySelectedHc12Profile(radioReceiverTestRestoreSnapshot, status);
    } else {
        remoteProgramActiveE220(radioReceiverTestRestoreSnapshot.e220Channel,
                                radioReceiverTestRestoreSnapshot.e220BaudIndex,
                                radioReceiverTestRestoreSnapshot.e220AirRateIndex,
                                radioReceiverTestRestoreSnapshot.e220PowerIndex,
                                radioReceiverTestRestoreSnapshot.e220FixedTransmission,
                                status);
    }
    radioRememberAppliedRuntime(radioReceiverTestRestoreSnapshot, false);
    restoreRadioRuntimeSnapshot(radioReceiverTestRestoreSnapshot);
    hc12RestartWithCurrentPins("");
    radioReceiverTestActive = false;
    radioReceiverTestRestoreValid = false;
    radioReceiverTestFrameLen = 0;
}

bool radioReceiverTestHandleByte(uint8_t ch)
{
    if (!radioReceiverTestActive) return false;

    if (radioReceiverTestFrameLen == 0) {
        if (ch != 0x06U) return true;
        radioReceiverTestFrameBuf[radioReceiverTestFrameLen++] = ch;
        return true;
    }
    if (radioReceiverTestFrameLen == 1) {
        if (ch != 0x85U) {
            radioReceiverTestFrameLen = 0;
            return true;
        }
        radioReceiverTestFrameBuf[radioReceiverTestFrameLen++] = ch;
        return true;
    }
    if (radioReceiverTestFrameLen == 2) {
        radioReceiverTestFrameBuf[radioReceiverTestFrameLen++] = ch;
        if (ch != sizeof(GateLegacyPacket)) {
            radioReceiverTestAppend(String("[RXTEST] Unexpected frame size: ") + String(ch) + "\n");
            radioReceiverTestFrameLen = 0;
        }
        return true;
    }

    const size_t expectedLen = 3U + static_cast<size_t>(radioReceiverTestFrameBuf[2]) + 1U;
    if (radioReceiverTestFrameLen < sizeof(radioReceiverTestFrameBuf)) {
        radioReceiverTestFrameBuf[radioReceiverTestFrameLen++] = ch;
    } else {
        radioReceiverTestAppend("[RXTEST] Frame buffer overflow\n");
        radioReceiverTestFrameLen = 0;
        return true;
    }

    if (radioReceiverTestFrameLen < expectedLen) return true;

    uint8_t checksum = radioReceiverTestFrameBuf[2];
    for (size_t i = 0; i < sizeof(GateLegacyPacket); ++i) checksum ^= radioReceiverTestFrameBuf[3 + i];
    const uint8_t receivedChecksum = radioReceiverTestFrameBuf[expectedLen - 1];
    if (checksum != receivedChecksum) {
        radioReceiverTestAppend(String("[RXTEST] Checksum mismatch calc=") + String(checksum, HEX) +
                                " rx=" + String(receivedChecksum, HEX) + "\n");
        radioReceiverTestFrameLen = 0;
        return true;
    }

    GateLegacyPacket packet = {};
    memcpy(&packet, &radioReceiverTestFrameBuf[3], sizeof(packet));
    const String decodedCounter = radioReceiverTestDecodeCounter(packet);
    char line[224];
    snprintf(line,
             sizeof(line),
             "[RXTEST] device=%u dest=%u packet=%08lX cmd=%u data=%d chk=%02X%s\n",
             packet.device_id,
             packet.destination_id,
             static_cast<unsigned long>(packet.packet_id),
             packet.command,
             packet.data,
             receivedChecksum,
             decodedCounter.c_str());
    radioReceiverTestAppend(line);
    radioReceiverTestFrameLen = 0;
    return true;
}

static void remoteDiagPrintPacketBytes(const uint8_t *data, size_t len)
{
    Serial.printf("[RCTL] TX bytes:");
    for (size_t i = 0; i < len; ++i) {
        Serial.printf(" %02X", data ? data[i] : 0U);
    }
    Serial.printf("\n");
}

static bool remoteProgramActiveHc12(int currentBaudIndex,
                                    int targetChannel,
                                    int targetBaudIndex,
                                    int targetModeIndex,
                                    int targetPowerLevel,
                                    String &statusOut)
{
    radioModuleType = RADIO_MODULE_HC12;
    hc12CurrentBaudIndex = constrain(currentBaudIndex, 0, static_cast<int>(sizeof(HC12_SUPPORTED_BAUDS) / sizeof(HC12_SUPPORTED_BAUDS[0])) - 1);
    Serial.printf("[RCTL] HC12 apply start currentBaud=%lu targetChannel=%d targetBaud=%lu targetMode=FU%d targetPower=P%d\n",
                  HC12_SUPPORTED_BAUDS[hc12CurrentBaudIndex],
                  constrain(targetChannel, HC12_MIN_CHANNEL, HC12_MAX_CHANNEL),
                  HC12_SUPPORTED_BAUDS[constrain(targetBaudIndex, 0, static_cast<int>(sizeof(HC12_SUPPORTED_BAUDS) / sizeof(HC12_SUPPORTED_BAUDS[0])) - 1)],
                  constrain(targetModeIndex, 0, 3) + 1,
                  constrain(targetPowerLevel, 1, 8));
    hc12RestartWithCurrentPins("");
    hc12EnterAtMode();

    char cmd[24];
    String resp;

    snprintf(cmd, sizeof(cmd), "AT+C%03d", constrain(targetChannel, HC12_MIN_CHANNEL, HC12_MAX_CHANNEL));
    resp = hc12QueryCommand(cmd);
    Serial.printf("[RCTL] HC12 cmd %s -> %s\n", cmd, resp.isEmpty() ? "<no reply>" : resp.c_str());
    if (!resp.startsWith("OK+C")) {
        hc12ExitAtMode();
        statusOut = resp.isEmpty() ? "Remote HC-12 channel change failed" : resp;
        return false;
    }

    snprintf(cmd, sizeof(cmd), "AT+FU%d", constrain(targetModeIndex, 0, 3) + 1);
    resp = hc12QueryCommand(cmd);
    Serial.printf("[RCTL] HC12 cmd %s -> %s\n", cmd, resp.isEmpty() ? "<no reply>" : resp.c_str());
    if (!resp.startsWith("OK+FU")) {
        hc12ExitAtMode();
        statusOut = resp.isEmpty() ? "Remote HC-12 mode change failed" : resp;
        return false;
    }

    snprintf(cmd, sizeof(cmd), "AT+P%d", constrain(targetPowerLevel, 1, 8));
    resp = hc12QueryCommand(cmd);
    Serial.printf("[RCTL] HC12 cmd %s -> %s\n", cmd, resp.isEmpty() ? "<no reply>" : resp.c_str());
    if (!resp.startsWith("OK+P")) {
        hc12ExitAtMode();
        statusOut = resp.isEmpty() ? "Remote HC-12 power change failed" : resp;
        return false;
    }

    snprintf(cmd,
             sizeof(cmd),
             "AT+B%lu",
             HC12_SUPPORTED_BAUDS[constrain(targetBaudIndex, 0, static_cast<int>(sizeof(HC12_SUPPORTED_BAUDS) / sizeof(HC12_SUPPORTED_BAUDS[0])) - 1)]);
    resp = hc12QueryCommand(cmd);
    Serial.printf("[RCTL] HC12 cmd %s -> %s\n", cmd, resp.isEmpty() ? "<no reply>" : resp.c_str());
    hc12ExitAtMode();
    if (!resp.startsWith("OK+B")) {
        statusOut = resp.isEmpty() ? "Remote HC-12 baud change failed" : resp;
        return false;
    }

    hc12CurrentChannel = constrain(targetChannel, HC12_MIN_CHANNEL, HC12_MAX_CHANNEL);
    hc12CurrentBaudIndex = constrain(targetBaudIndex, 0, static_cast<int>(sizeof(HC12_SUPPORTED_BAUDS) / sizeof(HC12_SUPPORTED_BAUDS[0])) - 1);
    hc12CurrentModeIndex = constrain(targetModeIndex, 0, 3);
    hc12CurrentPowerLevel = constrain(targetPowerLevel, 1, 8);
    hc12SerialReopen(hc12RuntimeBaud());
    delay(40);
    while (Serial1.available() > 0) Serial1.read();
    hc12EnterAtMode();
    const String verifyResp = hc12QueryCommand("AT", 220UL, 40UL);
    Serial.printf("[RCTL] HC12 verify AT @%lu -> %s\n",
                  hc12RuntimeBaud(),
                  verifyResp.isEmpty() ? "<no reply>" : verifyResp.c_str());
    hc12ExitAtMode();
    return true;
}

static bool remoteProgramActiveE220(int targetChannel,
                                    int targetBaudIndex,
                                    int targetAirRateIndex,
                                    int targetPowerIndex,
                                    bool targetFixedTransmission,
                                    String &statusOut)
{
    radioModuleType = RADIO_MODULE_E220;
    hc12RestartWithCurrentPins("");
    hc12EnterAtMode();

    String resp = e220QueryCommand((String("AT+CHANNEL=") + constrain(targetChannel, E220_MIN_CHANNEL, E220_MAX_CHANNEL)).c_str(), 220UL, 48UL);
    if (resp.indexOf("OK") < 0) {
        hc12ExitAtMode();
        statusOut = resp.isEmpty() ? "Remote E220 channel change failed" : resp;
        return false;
    }

    resp = e220QueryCommand((String("AT+RATE=") + E220_AIR_RATE_OPTIONS[constrain(targetAirRateIndex, 0, static_cast<int>(sizeof(E220_AIR_RATE_OPTIONS) / sizeof(E220_AIR_RATE_OPTIONS[0])) - 1)]).c_str(),
                            220UL,
                            48UL);
    if (resp.indexOf("OK") < 0) {
        hc12ExitAtMode();
        statusOut = resp.isEmpty() ? "Remote E220 air-rate change failed" : resp;
        return false;
    }

    resp = e220QueryCommand((String("AT+POWER=") + constrain(targetPowerIndex, 0, 3)).c_str(), 220UL, 48UL);
    if (resp.indexOf("OK") < 0) {
        hc12ExitAtMode();
        statusOut = resp.isEmpty() ? "Remote E220 power change failed" : resp;
        return false;
    }

    resp = e220QueryCommand((String("AT+TRANS=") + (targetFixedTransmission ? 1 : 0)).c_str(), 220UL, 48UL);
    if (resp.indexOf("OK") < 0) {
        hc12ExitAtMode();
        statusOut = resp.isEmpty() ? "Remote E220 transfer-mode change failed" : resp;
        return false;
    }

    resp = e220QueryCommand((String("AT+UART=") + E220_UART_BAUD_OPTIONS[constrain(targetBaudIndex, 0, static_cast<int>(sizeof(E220_UART_BAUD_OPTIONS) / sizeof(E220_UART_BAUD_OPTIONS[0])) - 1)] + ",0").c_str(),
                            220UL,
                            48UL);
    if (resp.indexOf("OK") < 0) {
        hc12ExitAtMode();
        statusOut = resp.isEmpty() ? "Remote E220 baud change failed" : resp;
        return false;
    }

    e220CurrentChannel = constrain(targetChannel, E220_MIN_CHANNEL, E220_MAX_CHANNEL);
    e220CurrentBaudIndex = constrain(targetBaudIndex, 0, static_cast<int>(sizeof(E220_UART_BAUD_VALUES) / sizeof(E220_UART_BAUD_VALUES[0])) - 1);
    e220CurrentAirRateIndex = constrain(targetAirRateIndex, 0, static_cast<int>(sizeof(E220_AIR_RATE_OPTIONS) / sizeof(E220_AIR_RATE_OPTIONS[0])) - 1);
    e220CurrentPowerIndex = constrain(targetPowerIndex, 0, 3);
    e220CurrentFixedTransmission = targetFixedTransmission;
    hc12ExitAtMode();
    delay(40);
    while (Serial1.available() > 0) Serial1.read();
    return true;
}

static bool remoteApplySelectedProfile(String &statusOut)
{
    const RadioRuntimeSnapshot targetRuntime = captureSelectedRemoteRuntimeSnapshot();
    if (radioAppliedRuntimeValid && radioRuntimeSnapshotsMatch(radioAppliedRuntimeSnapshot, targetRuntime)) {
        restoreRadioRuntimeSnapshot(targetRuntime);
        hc12RestartWithCurrentPins("");
        statusOut = "";
        return true;
    }

    bool ok = false;
    if (targetRuntime.moduleType == RADIO_MODULE_HC12) {
        ok = remoteApplySelectedHc12Profile(targetRuntime, statusOut);
    } else {
        ok = remoteProgramActiveE220(targetRuntime.e220Channel,
                                     targetRuntime.e220BaudIndex,
                                     targetRuntime.e220AirRateIndex,
                                     targetRuntime.e220PowerIndex,
                                     targetRuntime.e220FixedTransmission,
                                     statusOut);
    }

    if (ok) radioRememberAppliedRuntime(targetRuntime, false);
    return ok;
}

static bool gateSendLegacyPacket(uint16_t destinationId, int16_t payload, String &statusOut)
{
    uint32_t keyHigh = 0;
    uint32_t keyLow = 0;
    uint16_t remoteDeviceId = 0;
    uint16_t rollingCounter = 0;
    if (!remoteParseKey(keyHigh, keyLow, statusOut)) return false;
    if (!remoteParseDeviceId(remoteDeviceId, statusOut)) return false;
    if (!remoteParseRollingCounter(rollingCounter, statusOut)) return false;
    Serial.printf("[RCTL] send request module=%s remoteId=%u destId=%u payload=%d rolling=%u keyHigh=%08lX keyLow=%08lX\n",
                  remoteControlUsesHc12() ? "HC12" : "E220",
                  remoteDeviceId,
                  destinationId,
                  payload,
                  rollingCounter,
                  static_cast<unsigned long>(keyHigh),
                  static_cast<unsigned long>(keyLow));

    const RadioRuntimeSnapshot configSnapshot = captureRadioRuntimeSnapshot();
    const RadioRuntimeSnapshot targetRuntime = captureSelectedRemoteRuntimeSnapshot();
    String applyStatus;
    if (!remoteApplySelectedProfile(applyStatus)) {
        restoreRadioRuntimeSnapshot(configSnapshot);
        Serial.printf("[RCTL] remoteApplySelectedProfile FAILED: %s\n", applyStatus.c_str());
        statusOut = applyStatus;
        return false;
    }
    Serial.println("[RCTL] remoteApplySelectedProfile OK");

    while (Serial1.available() > 0) Serial1.read();

    if (targetRuntime.moduleType == RADIO_MODULE_HC12) {
        pinMode(hc12ActiveSetPin(), OUTPUT);
        digitalWrite(hc12ActiveSetPin(), LOW);
        delay(300);
        const String atResp = hc12QueryCommand("AT", 220UL, 40UL);
        Serial.printf("[RCTL] HC12 pre-send AT probe -> %s\n", atResp.isEmpty() ? "<no reply>" : atResp.c_str());
        digitalWrite(hc12ActiveSetPin(), HIGH);
        delay(60);
    }

    const uint16_t txRollingCounter = static_cast<uint16_t>(rollingCounter - 1U);
    const uint32_t rollingValue = static_cast<uint32_t>(txRollingCounter) * 100000UL + static_cast<uint32_t>(remoteDeviceId);
    const uint32_t encryptedPacketId = gateKeeLoqEncrypt(rollingValue, keyHigh, keyLow);
    GateLegacyPacket packet = {
        remoteDeviceId,
        destinationId,
        encryptedPacketId,
        0,
        payload
    };
    Serial.printf("[RCTL] packet fields device_id=%u destination_id=%u rollingValue=%lu packet_id=%08lX command=%u data=%d\n",
                  packet.device_id,
                  packet.destination_id,
                  static_cast<unsigned long>(rollingValue),
                  static_cast<unsigned long>(packet.packet_id),
                  packet.command,
                  packet.data);

    uint8_t checksum = static_cast<uint8_t>(sizeof(packet));
    uint8_t txBytes[3 + sizeof(packet) + 1] = {0x06, 0x85, static_cast<uint8_t>(sizeof(packet))};
    size_t txLen = 3;
    Serial1.write(static_cast<uint8_t>(0x06));
    Serial1.write(static_cast<uint8_t>(0x85));
    Serial1.write(static_cast<uint8_t>(sizeof(packet)));
    const uint8_t *raw = reinterpret_cast<const uint8_t *>(&packet);
    for (size_t i = 0; i < sizeof(packet); ++i) {
        checksum ^= raw[i];
        Serial1.write(raw[i]);
        txBytes[txLen++] = raw[i];
    }
    Serial1.write(checksum);
    txBytes[txLen++] = checksum;
    remoteDiagPrintPacketBytes(txBytes, txLen);
    Serial1.flush();
    delay(220);
    Serial.printf("[RCTL] packet sent checksum=%02X nextRolling=%u\n",
                  checksum,
                  static_cast<unsigned int>(txRollingCounter));

    gateRollingCount = txRollingCounter;
    gatePersistRollingCount();

    statusOut = "Gate command sent";
    return true;
}

void lvglRefreshRadioControlUi()
{
    if (lvglRadioControlConfigBtnLabel) lvglLabelSetTextIfChanged(lvglRadioControlConfigBtnLabel, tr(TXT_BUTTON_CONFIG));
    if (lvglRadioControlConfigBtn) lvglRegisterStyledButton(lvglRadioControlConfigBtn, lv_color_hex(0x7A5C2E), false);
    if (!lvglRadioControlList || !lv_obj_is_valid(lvglRadioControlList)) return;
    lv_obj_clean(lvglRadioControlList);
    if (radioControlButtonCount <= 0) {
        lv_obj_t *lbl = lv_label_create(lvglRadioControlList);
        lv_label_set_text(lbl, tr(TXT_NO_RADIO_BUTTONS));
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xC8D3DD), 0);
        return;
    }
    for (int i = 0; i < radioControlButtonCount; ++i) {
        const String title = radioControlButtonDisplayName(i);
        lv_obj_t *btn = lvglCreateMenuButton(lvglRadioControlList,
                                             title.c_str(),
                                             lv_color_hex(0x355F9A),
                                             lvglRadioControlButtonEvent,
                                             reinterpret_cast<void *>(static_cast<intptr_t>(i)));
        if (btn) {
            const bool localToggle = radioControlButtonUsesLocalToggle(radioControlButtons[i]);
            if (radioControlButtonUsesMomentary(radioControlButtons[i])) {
                lv_obj_add_event_cb(btn, lvglRadioControlButtonEvent, LV_EVENT_PRESSED, reinterpret_cast<void *>(static_cast<intptr_t>(i)));
                lv_obj_add_event_cb(btn, lvglRadioControlButtonEvent, LV_EVENT_RELEASED, reinterpret_cast<void *>(static_cast<intptr_t>(i)));
                lv_obj_add_event_cb(btn, lvglRadioControlButtonEvent, LV_EVENT_PRESS_LOST, reinterpret_cast<void *>(static_cast<intptr_t>(i)));
            }
            if (localToggle) {
                lvglRegisterPersistentStateButton(btn, lv_color_hex(0x355F9A), radioControlButtons[i].toggleState, false);
                lv_obj_clear_state(btn, LV_STATE_CHECKED);
            } else {
                lvglRegisterStyledButton(btn, lv_color_hex(0x355F9A), false);
                lv_obj_clear_state(btn, LV_STATE_CHECKED);
            }
            lvglRegisterReorderableItem(btn, "ord_rctl", String("btn_" + String(i)).c_str());
        }
    }
}

void lvglGateControlActionEvent(lv_event_t *e)
{
    (void)e;
}

void lvglRadioControlButtonEvent(lv_event_t *e)
{
    const lv_event_code_t code = lv_event_get_code(e);
    const int idx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (idx < 0 || idx >= radioControlButtonCount) return;

    syncRadioButtonConfigFromUi();
    RadioControlButton &btn = radioControlButtons[idx];
    if (btn.destinationId == 0U) {
        uiStatusLine = "Set destination ID first";
        lvglSyncStatusLine();
        return;
    }
    if (btn.onDataValue < 0) {
        uiStatusLine = "Set ON command first";
        lvglSyncStatusLine();
        return;
    }
    const bool toggleButton = radioControlButtonUsesLocalToggle(btn);
    const bool momentaryButton = radioControlButtonUsesMomentary(btn);
    if (code == LV_EVENT_PRESSED) {
        if (!momentaryButton || btn.pressActive) return;
        if (btn.requireConfirm) return;
        btn.pressActive = true;
        RadioControlTxRequest request = {};
        request.valid = true;
        request.buttonIndex = idx;
        request.destinationId = btn.destinationId;
        request.payload = btn.onDataValue;
        request.notifyOnSuccess = true;
        String status;
        const bool sent = radioControlEnqueueTx(request, status);
        if (!sent) btn.pressActive = false;
        uiStatusLine = status;
        lvglSyncStatusLine();
        return;
    }
    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        if (!momentaryButton || !btn.pressActive) return;
        if (!radioControlButtonNeedsReleaseTx(btn)) {
            btn.pressActive = false;
            return;
        }
        RadioControlTxRequest request = {};
        request.valid = true;
        request.buttonIndex = idx;
        request.destinationId = btn.destinationId;
        request.payload = btn.offDataValue;
        request.notifyOnSuccess = true;
        request.clearPressActive = true;
        String status;
        if (!radioControlEnqueueTx(request, status)) btn.pressActive = false;
        uiStatusLine = status;
        lvglSyncStatusLine();
        return;
    }
    if (code != LV_EVENT_CLICKED) return;

    const int16_t payload = toggleButton ? static_cast<int16_t>(btn.toggleState ? btn.offDataValue : btn.onDataValue)
                                         : static_cast<int16_t>(btn.onDataValue);
    if (btn.requireConfirm) {
        static const char *btns[] = {nullptr, nullptr, ""};
        btns[0] = tr(TXT_CANCEL);
        btns[1] = tr(TXT_SEND);
        lv_obj_t *m = lv_msgbox_create(nullptr, tr(TXT_CONFIRM), radioControlButtonDisplayName(idx).c_str(), btns, false);
        if (!m) return;
        lvglApplyMsgboxModalStyle(m);
        lv_obj_center(m);
        lv_obj_set_width(m, min<int16_t>(DISPLAY_WIDTH - 24, 260));
        lv_obj_add_event_cb(m, lvglRadioControlConfirmEvent, LV_EVENT_VALUE_CHANGED, reinterpret_cast<void *>(static_cast<intptr_t>(idx)));
        return;
    }

    if (momentaryButton) return;

    RadioControlTxRequest request = {};
    request.valid = true;
    request.buttonIndex = idx;
    request.destinationId = btn.destinationId;
    request.payload = payload;
    request.notifyOnSuccess = true;
    request.toggleStateValid = toggleButton;
    request.nextToggleState = toggleButton ? !btn.toggleState : false;
    String status;
    radioControlEnqueueTx(request, status);
    uiStatusLine = status;
    lvglSyncStatusLine();
}

void lvglRadioControlConfirmEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    lv_obj_t *msgbox = lv_event_get_current_target(e);
    const char *btn = msgbox ? lv_msgbox_get_active_btn_text(msgbox) : nullptr;
    const int idx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (btn && strcmp(btn, tr(TXT_SEND)) == 0 && idx >= 0 && idx < radioControlButtonCount) {
        syncRadioButtonConfigFromUi();
        RadioControlButton &buttonCfg = radioControlButtons[idx];
        if (buttonCfg.destinationId == 0U) {
            uiStatusLine = "Set destination ID first";
            lvglSyncStatusLine();
            if (msgbox) lv_msgbox_close(msgbox);
            return;
        }
        if (buttonCfg.onDataValue < 0) {
            uiStatusLine = "Set ON command first";
            lvglSyncStatusLine();
            if (msgbox) lv_msgbox_close(msgbox);
            return;
        }
        const bool toggleButton = radioControlButtonUsesLocalToggle(buttonCfg);
        const int16_t payload = toggleButton ? static_cast<int16_t>(buttonCfg.toggleState ? buttonCfg.offDataValue : buttonCfg.onDataValue)
                                             : static_cast<int16_t>(buttonCfg.onDataValue);
        RadioControlTxRequest request = {};
        request.valid = true;
        request.buttonIndex = idx;
        request.destinationId = buttonCfg.destinationId;
        request.payload = payload;
        request.notifyOnSuccess = true;
        request.toggleStateValid = toggleButton;
        request.nextToggleState = toggleButton ? !buttonCfg.toggleState : false;
        if (radioControlButtonUsesMomentary(buttonCfg)) {
            buttonCfg.pressActive = true;
            if (radioControlButtonNeedsReleaseTx(buttonCfg)) {
                request.followUpValid = true;
                request.followUpPayload = buttonCfg.offDataValue;
                request.followUpClearsPressActive = true;
            } else {
                request.clearPressActive = true;
            }
        }
        String status;
        if (!radioControlEnqueueTx(request, status) && radioControlButtonUsesMomentary(buttonCfg)) buttonCfg.pressActive = false;
        uiStatusLine = status;
        lvglSyncStatusLine();
    }
    if (msgbox) lv_msgbox_close(msgbox);
}

void lvglRefreshRadioButtonConfigUi()
{
    if (lvglRadioButtonAddBtn) lvglRegisterStyledButton(lvglRadioButtonAddBtn, lv_color_hex(0x2F7A45), true);
    if (lvglRadioButtonSelBtn) lvglRegisterPersistentStateButtonColors(lvglRadioButtonSelBtn, lv_color_hex(0x2F6D86), lv_color_hex(0x8A5A25), radioButtonSelectionMode, true);
    if (lvglRadioButtonDelBtn) lvglRegisterStyledButton(lvglRadioButtonDelBtn, lv_color_hex(0x8A3A3A), true);
    if (lvglRadioReceiverTestBtn) lvglRegisterStyledButton(lvglRadioReceiverTestBtn, lv_color_hex(0x7A5C2E), true);
    if (!lvglRadioButtonConfigList || !lv_obj_is_valid(lvglRadioButtonConfigList)) return;

    lv_obj_clean(lvglRadioButtonConfigList);
    for (int i = 0; i < radioControlButtonCount; ++i) {
        lv_obj_t *row = lv_obj_create(lvglRadioButtonConfigList);
        lv_obj_set_width(row, lv_pct(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_color(row, lv_color_hex(0x16212C), 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_radius(row, 12, 0);
        lv_obj_set_style_pad_all(row, 8, 0);
        lv_obj_set_style_pad_column(row, 6, 0);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *selCb = lv_checkbox_create(row);
        lv_checkbox_set_text(selCb, "");
        if (radioControlButtons[i].selected) lv_obj_add_state(selCb, LV_STATE_CHECKED);
        else lv_obj_clear_state(selCb, LV_STATE_CHECKED);
        if (radioButtonSelectionMode) lv_obj_clear_flag(selCb, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(selCb, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_event_cb(selCb, lvglRadioButtonFieldChangedEvent, LV_EVENT_VALUE_CHANGED, reinterpret_cast<void *>(static_cast<intptr_t>(i)));

        lv_obj_t *nameTa = lv_textarea_create(row);
        lv_obj_set_width(nameTa, 78);
        lvglConfigureCompactSingleLineTa(nameTa, 34);
        lv_textarea_set_max_length(nameTa, 20);
        lv_textarea_set_placeholder_text(nameTa, "Name");
        lv_textarea_set_text(nameTa, radioControlButtons[i].name.c_str());
        lv_obj_add_event_cb(nameTa, lvglTextAreaFocusEvent, LV_EVENT_FOCUSED, nullptr);
        lv_obj_add_event_cb(nameTa, lvglRadioButtonFieldChangedEvent, LV_EVENT_DEFOCUSED, reinterpret_cast<void *>(static_cast<intptr_t>(i)));
        lv_obj_add_event_cb(nameTa, lvglRadioButtonFieldChangedEvent, LV_EVENT_READY, reinterpret_cast<void *>(static_cast<intptr_t>(i)));

        lv_obj_t *idTa = lv_textarea_create(row);
        lv_obj_set_width(idTa, 46);
        lvglConfigureCompactSingleLineTa(idTa, 34);
        lv_textarea_set_max_length(idTa, 5);
        lv_textarea_set_accepted_chars(idTa, "0123456789");
        lv_textarea_set_placeholder_text(idTa, "ID");
        lv_textarea_set_text(idTa, radioControlButtons[i].destinationId == 0U ? "" : String(radioControlButtons[i].destinationId).c_str());
        lv_obj_add_event_cb(idTa, lvglTextAreaFocusEvent, LV_EVENT_FOCUSED, nullptr);
        lv_obj_add_event_cb(idTa, lvglRadioButtonFieldChangedEvent, LV_EVENT_DEFOCUSED, reinterpret_cast<void *>(static_cast<intptr_t>(i)));
        lv_obj_add_event_cb(idTa, lvglRadioButtonFieldChangedEvent, LV_EVENT_READY, reinterpret_cast<void *>(static_cast<intptr_t>(i)));

        lv_obj_t *onTa = lv_textarea_create(row);
        lv_obj_set_width(onTa, 34);
        lvglConfigureCompactSingleLineTa(onTa, 34);
        lv_textarea_set_max_length(onTa, 2);
        lv_textarea_set_accepted_chars(onTa, "0123456789");
        lv_textarea_set_placeholder_text(onTa, "On");
        lv_textarea_set_text(onTa, radioControlButtons[i].onDataValue < 0 ? "" : String(radioControlButtons[i].onDataValue).c_str());
        lv_obj_add_event_cb(onTa, lvglTextAreaFocusEvent, LV_EVENT_FOCUSED, nullptr);
        lv_obj_add_event_cb(onTa, lvglRadioButtonFieldChangedEvent, LV_EVENT_DEFOCUSED, reinterpret_cast<void *>(static_cast<intptr_t>(i)));
        lv_obj_add_event_cb(onTa, lvglRadioButtonFieldChangedEvent, LV_EVENT_READY, reinterpret_cast<void *>(static_cast<intptr_t>(i)));

        lv_obj_t *offTa = lv_textarea_create(row);
        lv_obj_set_width(offTa, 34);
        lvglConfigureCompactSingleLineTa(offTa, 34);
        lv_textarea_set_max_length(offTa, 2);
        lv_textarea_set_accepted_chars(offTa, "0123456789");
        lv_textarea_set_placeholder_text(offTa, "Off");
        lv_textarea_set_text(offTa, radioControlButtons[i].offDataValue < 0 ? "" : String(radioControlButtons[i].offDataValue).c_str());
        lv_obj_add_event_cb(offTa, lvglTextAreaFocusEvent, LV_EVENT_FOCUSED, nullptr);
        lv_obj_add_event_cb(offTa, lvglRadioButtonFieldChangedEvent, LV_EVENT_DEFOCUSED, reinterpret_cast<void *>(static_cast<intptr_t>(i)));
        lv_obj_add_event_cb(offTa, lvglRadioButtonFieldChangedEvent, LV_EVENT_READY, reinterpret_cast<void *>(static_cast<intptr_t>(i)));

        lv_obj_t *toggleCb = lv_checkbox_create(row);
        lv_obj_set_width(toggleCb, 36);
        if (radioControlButtons[i].toggleEnabled) lv_obj_add_state(toggleCb, LV_STATE_CHECKED);
        else lv_obj_clear_state(toggleCb, LV_STATE_CHECKED);
        lv_obj_add_event_cb(toggleCb, lvglRadioButtonFieldChangedEvent, LV_EVENT_VALUE_CHANGED, reinterpret_cast<void *>(static_cast<intptr_t>(i)));

        lv_obj_t *confirmCb = lv_checkbox_create(row);
        lv_checkbox_set_text(confirmCb, tr(TXT_CONFIRM));
        if (radioControlButtons[i].requireConfirm) lv_obj_add_state(confirmCb, LV_STATE_CHECKED);
        else lv_obj_clear_state(confirmCb, LV_STATE_CHECKED);
        lv_obj_add_event_cb(confirmCb, lvglRadioButtonFieldChangedEvent, LV_EVENT_VALUE_CHANGED, reinterpret_cast<void *>(static_cast<intptr_t>(i)));
    }

    if (radioControlButtonCount <= 0) {
        lv_obj_t *lbl = lv_label_create(lvglRadioButtonConfigList);
        lv_label_set_text(lbl, tr(TXT_ADD_FIRST_RADIO_BUTTON));
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xC8D3DD), 0);
    }
}

void lvglOpenRadioReceiverTestScreenEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_CONFIG_RADIO_RECEIVER_TEST);
    radioReceiverTestEnterScreen();
    lvglRefreshRadioReceiverTestUi();
    lvglOpenScreen(UI_CONFIG_RADIO_RECEIVER_TEST, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglRadioButtonFieldChangedEvent(lv_event_t *e)
{
    (void)e;
    syncRadioButtonConfigFromUi();
    savePersistedRadioControlButtons();
    lvglRefreshRadioControlUi();
}

void lvglRadioButtonAddEvent(lv_event_t *e)
{
    (void)e;
    syncRadioButtonConfigFromUi();
    if (radioControlButtonCount >= RADIO_CONTROL_MAX_BUTTONS) {
        uiStatusLine = tr(TXT_BUTTON_LIMIT_REACHED);
        lvglSyncStatusLine();
        return;
    }
    const int newIdx = radioControlButtonCount++;
    RadioControlButton &btn = radioControlButtons[newIdx];
    btn.name = (newIdx > 0) ? radioControlButtons[newIdx - 1].name : (String("Button ") + String(newIdx + 1));
    btn.destinationId = 0U;
    btn.onDataValue = -1;
    btn.offDataValue = -1;
    btn.requireConfirm = false;
    btn.toggleEnabled = false;
    btn.toggleState = false;
    btn.pressActive = false;
    btn.selected = false;
    savePersistedRadioControlButtons();
    lvglRefreshRadioButtonConfigUi();
    lvglRefreshRadioControlUi();
}

void lvglRadioButtonSelectModeEvent(lv_event_t *e)
{
    (void)e;
    radioButtonSelectionMode = !radioButtonSelectionMode;
    if (!radioButtonSelectionMode) {
        for (int i = 0; i < radioControlButtonCount; ++i) radioControlButtons[i].selected = false;
    }
    lvglRefreshRadioButtonConfigUi();
}

void lvglRadioButtonDeletePromptEvent(lv_event_t *e)
{
    (void)e;
    syncRadioButtonConfigFromUi();
    int selectedCount = 0;
    for (int i = 0; i < radioControlButtonCount; ++i) if (radioControlButtons[i].selected) selectedCount++;
    if (selectedCount <= 0) {
        uiStatusLine = tr(TXT_SELECT_BUTTONS_TO_DELETE);
        lvglSyncStatusLine();
        return;
    }
    static const char *btns[] = {nullptr, nullptr, ""};
    btns[0] = tr(TXT_CANCEL);
    btns[1] = tr(TXT_DELETE_BUTTONS);
    lv_obj_t *m = lv_msgbox_create(nullptr, tr(TXT_DELETE_BUTTONS), tr(TXT_DELETE_SELECTED_RADIO_BUTTONS), btns, false);
    if (!m) return;
    lvglApplyMsgboxModalStyle(m);
    lv_obj_center(m);
    lv_obj_set_width(m, min<int16_t>(DISPLAY_WIDTH - 24, 260));
    lv_obj_add_event_cb(m, lvglRadioButtonDeleteConfirmEvent, LV_EVENT_VALUE_CHANGED, nullptr);
}

void lvglRadioButtonDeleteConfirmEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    lv_obj_t *msgbox = lv_event_get_current_target(e);
    const char *btn = msgbox ? lv_msgbox_get_active_btn_text(msgbox) : nullptr;
    if (btn && strcmp(btn, tr(TXT_DELETE_BUTTONS)) == 0) {
        syncRadioButtonConfigFromUi();
        int writeIdx = 0;
        for (int i = 0; i < radioControlButtonCount; ++i) {
            if (radioControlButtons[i].selected) continue;
            if (writeIdx != i) radioControlButtons[writeIdx] = radioControlButtons[i];
            radioControlButtons[writeIdx].selected = false;
            writeIdx++;
        }
        radioControlButtonCount = writeIdx;
        radioButtonSelectionMode = false;
        savePersistedRadioControlButtons();
        lvglRefreshRadioButtonConfigUi();
        lvglRefreshRadioControlUi();
    }
    if (msgbox) lv_msgbox_close(msgbox);
}

void lvglRefreshRemoteControlUi()
{
    const bool useHc12 = remoteControlUsesHc12();
    if (lvglRemoteModuleLabel) lvglLabelSetTextIfChanged(lvglRemoteModuleLabel, tr(TXT_RADIO_MODULE));
    if (lvglRemoteChannelLabel) lvglLabelSetTextIfChanged(lvglRemoteChannelLabel, tr(TXT_CHANNEL));
    if (lvglRemoteBaudLabel) lvglLabelSetTextIfChanged(lvglRemoteBaudLabel, tr(TXT_BAUD_RATE));
    if (lvglRemoteModeLabel) lvglLabelSetTextIfChanged(lvglRemoteModeLabel, useHc12 ? tr(TXT_FU_MODE) : "Air Data Rate");
    if (lvglRemotePowerLabel) lvglLabelSetTextIfChanged(lvglRemotePowerLabel, tr(TXT_POWER));
    if (lvglRemoteExtraLabel) lvglLabelSetTextIfChanged(lvglRemoteExtraLabel, tr(TXT_TRANSFER_MODE));
    if (lvglRemoteKeyLabel) lvglLabelSetTextIfChanged(lvglRemoteKeyLabel, tr(TXT_ENCRYPTION_KEY));
    if (lvglRemoteIdLabel) lvglLabelSetTextIfChanged(lvglRemoteIdLabel, tr(TXT_REMOTE_ID));
    if (lvglRemoteCounterLabel) lvglLabelSetTextIfChanged(lvglRemoteCounterLabel, tr(TXT_ROLLING_COUNTER));
    if (lvglRemoteSaveBtnLabel) lvglLabelSetTextIfChanged(lvglRemoteSaveBtnLabel, tr(TXT_SAVE));
    if (lvglRemoteStatusLabel) lvglLabelSetTextIfChanged(lvglRemoteStatusLabel, remoteControlStatusText);

    if (lvglRemoteModuleDropdown) {
        lv_dropdown_set_options(lvglRemoteModuleDropdown, buildRadioModuleDropdownOptions().c_str());
        const uint16_t selected = static_cast<uint16_t>(remoteControlModuleType);
        if (lv_dropdown_get_selected(lvglRemoteModuleDropdown) != selected) lv_dropdown_set_selected(lvglRemoteModuleDropdown, selected);
    }
    if (lvglRemoteChannelTa && lvglKeyboardFocusedTa != lvglRemoteChannelTa) {
        const String channelText = String(useHc12 ? remoteHc12Channel : remoteE220Channel);
        if (String(lv_textarea_get_text(lvglRemoteChannelTa)) != channelText) lv_textarea_set_text(lvglRemoteChannelTa, channelText.c_str());
    }
    if (lvglRemoteBaudDropdown) {
        lv_dropdown_set_options(lvglRemoteBaudDropdown, buildRemoteBaudDropdownOptions(useHc12).c_str());
        const uint16_t selected = static_cast<uint16_t>(useHc12 ? remoteHc12BaudIndex : remoteE220BaudIndex);
        if (lv_dropdown_get_selected(lvglRemoteBaudDropdown) != selected) lv_dropdown_set_selected(lvglRemoteBaudDropdown, selected);
    }
    if (lvglRemoteModeDropdown) {
        lv_dropdown_set_options(lvglRemoteModeDropdown, buildRemoteModeDropdownOptions(useHc12).c_str());
        const uint16_t selected = static_cast<uint16_t>(useHc12 ? remoteHc12ModeIndex : remoteE220AirRateIndex);
        if (lv_dropdown_get_selected(lvglRemoteModeDropdown) != selected) lv_dropdown_set_selected(lvglRemoteModeDropdown, selected);
    }
    if (lvglRemotePowerDropdown) {
        lv_dropdown_set_options(lvglRemotePowerDropdown, buildRemotePowerDropdownOptions(useHc12).c_str());
        const uint16_t selected = static_cast<uint16_t>(useHc12 ? (remoteHc12PowerLevel - 1) : remoteE220PowerIndex);
        if (lv_dropdown_get_selected(lvglRemotePowerDropdown) != selected) lv_dropdown_set_selected(lvglRemotePowerDropdown, selected);
    }
    if (lvglRemoteExtraCard) {
        if (useHc12) lv_obj_add_flag(lvglRemoteExtraCard, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_clear_flag(lvglRemoteExtraCard, LV_OBJ_FLAG_HIDDEN);
    }
    if (lvglRemoteExtraDropdown) {
        lv_dropdown_set_options(lvglRemoteExtraDropdown, "Transparent\nFixed");
        const uint16_t selected = static_cast<uint16_t>(remoteE220FixedTransmission ? 1 : 0);
        if (lv_dropdown_get_selected(lvglRemoteExtraDropdown) != selected) lv_dropdown_set_selected(lvglRemoteExtraDropdown, selected);
    }
    if (lvglRemoteKeyTa && lvglKeyboardFocusedTa != lvglRemoteKeyTa) {
        if (String(lv_textarea_get_text(lvglRemoteKeyTa)) != remoteControlKeyHex) lv_textarea_set_text(lvglRemoteKeyTa, remoteControlKeyHex.c_str());
    }
    if (lvglRemoteIdTa && lvglKeyboardFocusedTa != lvglRemoteIdTa) {
        const String remoteIdText = remoteControlDeviceId == 0U ? String("") : String(remoteControlDeviceId);
        if (String(lv_textarea_get_text(lvglRemoteIdTa)) != remoteIdText) lv_textarea_set_text(lvglRemoteIdTa, remoteIdText.c_str());
    }
    if (lvglRemoteCounterTa && lvglKeyboardFocusedTa != lvglRemoteCounterTa) {
        const String counterText = gateRollingCount == 0U ? String("") : String(gateRollingCount);
        if (String(lv_textarea_get_text(lvglRemoteCounterTa)) != counterText) lv_textarea_set_text(lvglRemoteCounterTa, counterText.c_str());
    }
}

void lvglRemoteControlModuleChangedEvent(lv_event_t *e)
{
    (void)e;
    if (!lvglRemoteModuleDropdown) return;
    const uint16_t selected = lv_dropdown_get_selected(lvglRemoteModuleDropdown);
    remoteControlModuleType = static_cast<RadioModuleType>(constrain(static_cast<int>(selected),
                                                                     static_cast<int>(RADIO_MODULE_HC12),
                                                                     static_cast<int>(RADIO_MODULE_COUNT - 1)));
    lvglRefreshRemoteControlUi();
}

void lvglRemoteControlSaveEvent(lv_event_t *e)
{
    (void)e;
    if (!lvglRemoteChannelTa || !lvglRemoteBaudDropdown || !lvglRemoteModeDropdown || !lvglRemotePowerDropdown || !lvglRemoteKeyTa || !lvglRemoteIdTa || !lvglRemoteCounterTa) return;

    String keyText = lv_textarea_get_text(lvglRemoteKeyTa);
    keyText.trim();
    remoteControlKeyHex = remoteHexOnly(keyText);

    String remoteIdText = lv_textarea_get_text(lvglRemoteIdTa);
    remoteIdText.trim();
    const unsigned long remoteIdValue = strtoul(remoteIdText.c_str(), nullptr, 10);
    if (remoteIdText.isEmpty() || remoteIdValue == 0UL || remoteIdValue > 65535UL) {
        remoteControlStatusText = "Remote ID must be 1..65535";
        lvglRefreshRemoteControlUi();
        return;
    }
    remoteControlDeviceId = static_cast<uint16_t>(remoteIdValue);

    String rollingCounterText = lv_textarea_get_text(lvglRemoteCounterTa);
    rollingCounterText.trim();
    const unsigned long rollingCounterValue = strtoul(rollingCounterText.c_str(), nullptr, 10);
    if (rollingCounterText.isEmpty() || rollingCounterValue == 0UL || rollingCounterValue > 65535UL) {
        remoteControlStatusText = "Rolling counter must be 1..65535";
        lvglRefreshRemoteControlUi();
        return;
    }
    gateRollingCount = static_cast<uint16_t>(rollingCounterValue);

    String channelText = lv_textarea_get_text(lvglRemoteChannelTa);
    channelText.trim();
    const int channel = channelText.toInt();

    if (remoteControlUsesHc12()) {
        if (channel < HC12_MIN_CHANNEL || channel > HC12_MAX_CHANNEL) {
            remoteControlStatusText = "HC-12 channel must be 1..100";
            lvglRefreshRemoteControlUi();
            return;
        }
        remoteHc12Channel = channel;
        remoteHc12BaudIndex = lv_dropdown_get_selected(lvglRemoteBaudDropdown);
        remoteHc12ModeIndex = lv_dropdown_get_selected(lvglRemoteModeDropdown);
        remoteHc12PowerLevel = lv_dropdown_get_selected(lvglRemotePowerDropdown) + 1;
    } else {
        if (channel < E220_MIN_CHANNEL || channel > E220_MAX_CHANNEL) {
            remoteControlStatusText = "E220 channel must be 0..83";
            lvglRefreshRemoteControlUi();
            return;
        }
        remoteE220Channel = channel;
        remoteE220BaudIndex = lv_dropdown_get_selected(lvglRemoteBaudDropdown);
        remoteE220AirRateIndex = lv_dropdown_get_selected(lvglRemoteModeDropdown);
        remoteE220PowerIndex = lv_dropdown_get_selected(lvglRemotePowerDropdown);
        remoteE220FixedTransmission = lvglRemoteExtraDropdown ? (lv_dropdown_get_selected(lvglRemoteExtraDropdown) == 1) : false;
    }

    savePersistedRemoteControlSettings();
    remoteControlStatusText = remoteControlKeyHex.isEmpty() ? "Profile saved. Add a rolling-code key before sending."
                                                            : "Remote-control profile saved";
    uiStatusLine = remoteControlStatusText;
    lvglSyncStatusLine();
    lvglHideKeyboard();
    lvglRefreshRemoteControlUi();
}

static void loadPersistedRemoteControlSettings()
{
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    remoteControlModuleType = static_cast<RadioModuleType>(constrain(uiPrefs.getUChar("rctl_mod", static_cast<uint8_t>(RADIO_MODULE_HC12)),
                                                                     static_cast<uint8_t>(RADIO_MODULE_HC12),
                                                                     static_cast<uint8_t>(RADIO_MODULE_COUNT - 1)));
#else
    remoteControlModuleType = RADIO_MODULE_HC12;
#endif
    remoteHc12Channel = constrain(uiPrefs.getInt("rhc12_ch", HC12_MIN_CHANNEL), HC12_MIN_CHANNEL, HC12_MAX_CHANNEL);
    remoteHc12BaudIndex = constrain(uiPrefs.getInt("rhc12_baud", 3), 0, static_cast<int>(sizeof(HC12_SUPPORTED_BAUDS) / sizeof(HC12_SUPPORTED_BAUDS[0])) - 1);
    remoteHc12ModeIndex = constrain(uiPrefs.getInt("rhc12_mode", 2), 0, 3);
    remoteHc12PowerLevel = constrain(uiPrefs.getInt("rhc12_pwr", 8), 1, 8);
    remoteE220Channel = constrain(uiPrefs.getInt("re220_ch", 23), E220_MIN_CHANNEL, E220_MAX_CHANNEL);
    remoteE220BaudIndex = constrain(uiPrefs.getInt("re220_baud", 3), 0, static_cast<int>(sizeof(E220_UART_BAUD_VALUES) / sizeof(E220_UART_BAUD_VALUES[0])) - 1);
    remoteE220AirRateIndex = constrain(uiPrefs.getInt("re220_rate", 2), 0, static_cast<int>(sizeof(E220_AIR_RATE_OPTIONS) / sizeof(E220_AIR_RATE_OPTIONS[0])) - 1);
    remoteE220PowerIndex = constrain(uiPrefs.getInt("re220_pwr", 0), 0, 3);
    remoteE220FixedTransmission = uiPrefs.getBool("re220_fix", false);
    const String persistedKeyRaw = uiPrefs.getString("rctl_key", "");
    remoteControlKeyHex = remoteHexOnly(persistedKeyRaw);
    remoteControlDeviceId = static_cast<uint16_t>(constrain(uiPrefs.getUInt("rctl_id", 0U), 0U, 65535U));
    gateRollingCount = static_cast<uint16_t>(constrain(uiPrefs.getUInt("gate_cnt", GATE_COUNT_INITIAL), 0U, 65535U));
    if (remoteControlKeyHex != persistedKeyRaw) {
        uiPrefs.putString("rctl_key", remoteControlKeyHex);
    }
    if (remoteControlKeyHex.isEmpty()) remoteControlStatusText = "Enter a rolling-code key to enable remote sending";
    else if (remoteControlDeviceId == 0U) remoteControlStatusText = "Enter a remote ID to enable remote sending";
    else if (gateRollingCount == 0U) remoteControlStatusText = "Enter a rolling counter to enable remote sending";
    else remoteControlStatusText = "Remote-control profile loaded";
}

static void savePersistedRemoteControlSettings()
{
    remoteControlKeyHex = remoteHexOnly(remoteControlKeyHex);
    uiPrefs.begin("ui", false);
    uiPrefs.putUChar("rctl_mod", static_cast<uint8_t>(remoteControlModuleType));
    uiPrefs.putInt("rhc12_ch", remoteHc12Channel);
    uiPrefs.putInt("rhc12_baud", remoteHc12BaudIndex);
    uiPrefs.putInt("rhc12_mode", remoteHc12ModeIndex);
    uiPrefs.putInt("rhc12_pwr", remoteHc12PowerLevel);
    uiPrefs.putInt("re220_ch", remoteE220Channel);
    uiPrefs.putInt("re220_baud", remoteE220BaudIndex);
    uiPrefs.putInt("re220_rate", remoteE220AirRateIndex);
    uiPrefs.putInt("re220_pwr", remoteE220PowerIndex);
    uiPrefs.putBool("re220_fix", remoteE220FixedTransmission);
    uiPrefs.putString("rctl_key", remoteControlKeyHex);
    uiPrefs.putUInt("rctl_id", remoteControlDeviceId);
    uiPrefs.putUInt("gate_cnt", gateRollingCount);
    uiPrefs.end();
}

void lvglOpenRemoteControlScreenEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_CONFIG_REMOTE_CONTROL);
    lvglRefreshRemoteControlUi();
    lvglOpenScreen(UI_CONFIG_REMOTE_CONTROL, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglOpenRadioButtonConfigScreenEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_CONFIG_RADIO_BUTTONS);
    lvglRefreshRadioButtonConfigUi();
    lvglOpenScreen(UI_CONFIG_RADIO_BUTTONS, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

