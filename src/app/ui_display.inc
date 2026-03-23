#if defined(UI_DISPLAY_SECTION_TFT)
TFT_eSPI tft;
#elif defined(UI_DISPLAY_SECTION_STATE)
static lv_color_t *lvglDrawPixels = nullptr;
static lv_color_t *lvglDrawPixels2 = nullptr;
static uint16_t lvglBufLinesActive = 0;
static lv_disp_draw_buf_t lvglDrawBuf;
static lv_disp_drv_t lvglDispDrv;
static lv_indev_drv_t lvglIndevDrv;
static lv_indev_t *lvglTouchIndev = nullptr;
static bool lvglReady = false;
static lv_obj_t *lvglScrHome = nullptr;
static lv_obj_t *lvglScrLock = nullptr;
static lv_obj_t *lvglScrChat = nullptr;
static lv_obj_t *lvglScrChatPeers = nullptr;
static lv_obj_t *lvglScrWifi = nullptr;
static lv_obj_t *lvglScrMedia = nullptr;
static lv_obj_t *lvglScrInfo = nullptr;
static lv_obj_t *lvglScrGames = nullptr;
static lv_obj_t *lvglScrRadioControl = nullptr;
static lv_obj_t *lvglScrConfig = nullptr;
static lv_obj_t *lvglScrRemoteControl = nullptr;
static lv_obj_t *lvglScrRadioButtonConfig = nullptr;
static lv_obj_t *lvglScrRadioReceiverTest = nullptr;
static lv_obj_t *lvglScrStyle = nullptr;
static lv_obj_t *lvglScrStyleHomeItems = nullptr;
static lv_obj_t *lvglScrBatteryTrain = nullptr;
static lv_obj_t *lvglScrLanguage = nullptr;
static lv_obj_t *lvglScrOta = nullptr;
static lv_obj_t *lvglScrModules = nullptr;
static lv_obj_t *lvglScrScreensaver = nullptr;
static lv_obj_t *lvglScrHc12 = nullptr;
static lv_obj_t *lvglScrHc12Terminal = nullptr;
static lv_obj_t *lvglScrHc12Info = nullptr;
static lv_obj_t *lvglScrMqttCfg = nullptr;
static lv_obj_t *lvglScrMqttCtrl = nullptr;
static lv_obj_t *lvglScrMqttButtonConfig = nullptr;
static lv_obj_t *lvglScrSnake = nullptr;
static lv_obj_t *lvglScrTetris = nullptr;
static lv_obj_t *lvglScrCheckers = nullptr;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
static lv_obj_t *lvglScrSnake3d = nullptr;
#endif
static lv_obj_t *lvglStatusLabel = nullptr;
static lv_obj_t *lvglInfoList = nullptr;
static lv_obj_t *lvglInfoBatteryCard = nullptr;
static lv_obj_t *lvglInfoWifiCard = nullptr;
static lv_obj_t *lvglInfoRadioCard = nullptr;
static lv_obj_t *lvglInfoLightCard = nullptr;
static lv_obj_t *lvglInfoSdCard = nullptr;
static lv_obj_t *lvglInfoBatteryValueLabel = nullptr;
static lv_obj_t *lvglInfoBatterySubLabel = nullptr;
static lv_obj_t *lvglInfoWifiValueLabel = nullptr;
static lv_obj_t *lvglInfoWifiSubLabel = nullptr;
static lv_obj_t *lvglInfoHc12ValueLabel = nullptr;
static lv_obj_t *lvglInfoHc12SubLabel = nullptr;
static lv_obj_t *lvglInfoLightValueLabel = nullptr;
static lv_obj_t *lvglInfoLightSubLabel = nullptr;
static lv_obj_t *lvglHc12ChannelValueLabel = nullptr;
static lv_obj_t *lvglHc12ChannelSubLabel = nullptr;
static lv_obj_t *lvglHc12BaudValueLabel = nullptr;
static lv_obj_t *lvglHc12BaudSubLabel = nullptr;
static lv_obj_t *lvglHc12ModeValueLabel = nullptr;
static lv_obj_t *lvglHc12ModeSubLabel = nullptr;
static lv_obj_t *lvglHc12PowerValueLabel = nullptr;
static lv_obj_t *lvglHc12PowerSubLabel = nullptr;
static lv_obj_t *lvglRadioModuleDropdown = nullptr;
static lv_obj_t *lvglRadioModuleHeader = nullptr;
static lv_obj_t *lvglRadioChannelTitleLabel = nullptr;
static lv_obj_t *lvglRadioBaudTitleLabel = nullptr;
static lv_obj_t *lvglRadioModeTitleLabel = nullptr;
static lv_obj_t *lvglRadioPowerTitleLabel = nullptr;
static lv_obj_t *lvglRadioExtraCard = nullptr;
static lv_obj_t *lvglRadioExtraTitleLabel = nullptr;
static lv_obj_t *lvglRadioExtraValueLabel = nullptr;
static lv_obj_t *lvglRadioExtraSubLabel = nullptr;
static lv_obj_t *lvglRadioPinSwapCard = nullptr;
static lv_obj_t *lvglRadioPinSwapTitleLabel = nullptr;
static lv_obj_t *lvglRadioPinSwapValueLabel = nullptr;
static lv_obj_t *lvglRadioPinSwapSubLabel = nullptr;
static lv_obj_t *lvglRadioModePinSwapCard = nullptr;
static lv_obj_t *lvglRadioModePinSwapTitleLabel = nullptr;
static lv_obj_t *lvglRadioModePinSwapValueLabel = nullptr;
static lv_obj_t *lvglRadioModePinSwapSubLabel = nullptr;
static lv_obj_t *lvglRadioHintLabel = nullptr;
static lv_obj_t *lvglHc12ConfigStatusLabel = nullptr;
static lv_obj_t *lvglHc12TerminalPinLabel = nullptr;
static lv_obj_t *lvglHc12TerminalExamplesLabel = nullptr;
static lv_obj_t *lvglHc12InfoVersionLabel = nullptr;
static lv_obj_t *lvglHc12InfoBaudLabel = nullptr;
static lv_obj_t *lvglHc12InfoChannelLabel = nullptr;
static lv_obj_t *lvglHc12InfoFuModeLabel = nullptr;
static lv_obj_t *lvglHc12InfoPowerLabel = nullptr;
static lv_obj_t *lvglHc12InfoRawLabel = nullptr;
static lv_obj_t *lvglInfoCpuCard = nullptr;
static lv_obj_t *lvglInfoSramCard = nullptr;
static lv_obj_t *lvglInfoPsramCard = nullptr;
static lv_obj_t *lvglInfoTempCard = nullptr;
static lv_obj_t *lvglInfoSystemLabel = nullptr;
static lv_obj_t *lvglWifiList = nullptr;
static lv_obj_t *lvglWifiScanLabel = nullptr;
static lv_obj_t *lvglWifiApSsidTa = nullptr;
static lv_obj_t *lvglWifiApPassTa = nullptr;
static lv_obj_t *lvglWifiApPassShowBtnLabel = nullptr;
static lv_obj_t *lvglWifiApPassShowBtn = nullptr;
static lv_obj_t *lvglWifiWebServerBtn = nullptr;
static lv_obj_t *lvglWifiPwdModal = nullptr;
static lv_obj_t *lvglWifiPwdInputRow = nullptr;
static lv_obj_t *lvglWifiPwdTa = nullptr;
static lv_obj_t *lvglWifiPwdSsidLabel = nullptr;
static lv_obj_t *lvglWifiPwdStatusLabel = nullptr;
static lv_obj_t *lvglWifiPwdShowBtnLabel = nullptr;
static lv_obj_t *lvglWifiPwdShowBtn = nullptr;
static lv_obj_t *lvglWifiPwdActionRow = nullptr;
static lv_obj_t *lvglWifiPwdCancelBtn = nullptr;
static lv_obj_t *lvglWifiPwdSaveBtn = nullptr;
static lv_obj_t *lvglMediaPlayerPanel = nullptr;
static lv_obj_t *lvglMediaList = nullptr;
static lv_obj_t *lvglMediaTrackLabel = nullptr;
static lv_obj_t *lvglMediaPlayBtn = nullptr;
static lv_obj_t *lvglMediaPlayBtnLabel = nullptr;
static lv_obj_t *lvglMediaPrevBtn = nullptr;
static lv_obj_t *lvglMediaNextBtn = nullptr;
static lv_obj_t *lvglMediaVolSlider = nullptr;
static lv_obj_t *lvglMediaVolValueLabel = nullptr;
static lv_obj_t *lvglMediaProgressBar = nullptr;
static lv_obj_t *lvglMediaProgressLabel = nullptr;
static lv_obj_t *lvglMqttStatusLabel = nullptr;
static lv_obj_t *lvglMqttStatusPanel = nullptr;
static lv_obj_t *lvglMqttCountLabel = nullptr;
static lv_obj_t *lvglMqttEditLabel = nullptr;
static lv_obj_t *lvglMqttBrokerTa = nullptr;
static lv_obj_t *lvglMqttPortTa = nullptr;
static lv_obj_t *lvglMqttUserTa = nullptr;
static lv_obj_t *lvglMqttPassTa = nullptr;
static lv_obj_t *lvglMqttPassShowBtnLabel = nullptr;
static lv_obj_t *lvglMqttPassShowBtn = nullptr;
static lv_obj_t *lvglMqttDiscTa = nullptr;
static lv_obj_t *lvglMqttBtnNameTa = nullptr;
static lv_obj_t *lvglMqttEnableSw = nullptr;
static lv_obj_t *lvglMqttCriticalSw = nullptr;
static lv_obj_t *lvglMqttCtrlList = nullptr;
static lv_obj_t *lvglMqttButtonConfigList = nullptr;
static lv_obj_t *lvglChatList = nullptr;
static lv_obj_t *lvglChatInputTa = nullptr;
static lv_obj_t *lvglChatEmptyLabel = nullptr;
static lv_obj_t *lvglChatComposer = nullptr;
static lv_obj_t *lvglChatContacts = nullptr;
static lv_obj_t *lvglChatContactLabel = nullptr;
static lv_obj_t *lvglChatPeersBtn = nullptr;
static lv_obj_t *lvglChatDiscoveryBtn = nullptr;
static lv_obj_t *lvglChatMenuBtn = nullptr;
static lv_obj_t *lvglChatMenuBackdrop = nullptr;
static lv_obj_t *lvglChatMenuPanel = nullptr;
static lv_obj_t *lvglChatPeerList = nullptr;
static lv_obj_t *lvglChatPeerIdentityLabel = nullptr;
static lv_obj_t *lvglChatPeerScanBtn = nullptr;
static unsigned long lvglChatPeerScanRevertAtMs = 0;
static lv_obj_t *lvglAirplaneBtn = nullptr;
static lv_obj_t *lvglAirplaneBtnLabel = nullptr;
static lv_obj_t *lvglApModeBtn = nullptr;
static lv_obj_t *lvglApModeBtnLabel = nullptr;
static lv_obj_t *lvglHomePowerBtn = nullptr;
static lv_obj_t *lvglHomeRadioBtn = nullptr;
static lv_obj_t *lvglHomeChatBtn = nullptr;
static lv_obj_t *lvglHomeMediaBtn = nullptr;
static lv_obj_t *lvglHomeInfoBtn = nullptr;
static lv_obj_t *lvglHomeGamesBtn = nullptr;
static lv_obj_t *lvglHomeMqttControlsBtn = nullptr;
static lv_obj_t *lvglHomeConfigBtn = nullptr;
static lv_obj_t *lvglConfigWifiBtn = nullptr;
static lv_obj_t *lvglConfigHc12Btn = nullptr;
static lv_obj_t *lvglConfigRemoteBtn = nullptr;
static lv_obj_t *lvglConfigStyleBtn = nullptr;
static lv_obj_t *lvglConfigBatteryBtn = nullptr;
static lv_obj_t *lvglConfigMqttBtn = nullptr;
static lv_obj_t *lvglConfigScreenshotBtn = nullptr;
static lv_obj_t *lvglConfigLanguageBtn = nullptr;
static lv_obj_t *lvglConfigOtaBtn = nullptr;
static lv_obj_t *lvglConfigModulesBtn = nullptr;
static lv_obj_t *lvglConfigFactoryResetBtn = nullptr;
static lv_obj_t *lvglConfigWrap = nullptr;
static lv_obj_t *lvglModulesStatusLabel = nullptr;
static lv_obj_t *lvglModulesWrap = nullptr;
static lv_obj_t *lvglModulesNameLabels[MODULE_SLOT_COUNT] = {};
static lv_obj_t *lvglModulesDescLabels[MODULE_SLOT_COUNT] = {};
static lv_obj_t *lvglModulesVersionLabels[MODULE_SLOT_COUNT] = {};
static lv_obj_t *lvglModulesActionBtns[MODULE_SLOT_COUNT] = {};
static lv_obj_t *lvglModulesActionLabels[MODULE_SLOT_COUNT] = {};
static lv_obj_t *lvglStyleScreenLockSw = nullptr;
static lv_obj_t *lvglScreenLockModal = nullptr;
static lv_obj_t *lvglScreenLockSetupPinTa = nullptr;
static lv_obj_t *lvglScreenLockSetupStatusLabel = nullptr;
static lv_obj_t *lvglScreenLockSetupCancelBtn = nullptr;
static lv_obj_t *lvglScreenLockSetupSaveBtn = nullptr;
static lv_obj_t *lvglScreenLockPinTa = nullptr;
static lv_obj_t *lvglScreenLockStatusLabel = nullptr;
static lv_obj_t *lvglScreenLockUnlockBtn = nullptr;
static lv_obj_t *lvglScreenLockUnlockBtnLabel = nullptr;
static lv_obj_t *lvglScreenLockResetBtn = nullptr;
static lv_obj_t *lvglScreenLockResetBtnLabel = nullptr;
static lv_obj_t *lvglStyleScreensaverSw = nullptr;
static lv_obj_t *lvglStyleScreensaverRow = nullptr;
static lv_obj_t *lvglStyleScreensaverTypeRow = nullptr;
static lv_obj_t *lvglStyleScreensaverTypeDd = nullptr;
static lv_obj_t *lvglStyleMenuIconsSw = nullptr;
static lv_obj_t *lvglStyleTouchVibrationSw = nullptr;
static lv_obj_t *lvglStyleTouchSoundSw = nullptr;
static lv_obj_t *lvglStyleHomeItemsBtn = nullptr;
static lv_obj_t *lvglStyleHomeItemsBtnLabel = nullptr;
static lv_obj_t *lvglStyleButtonFlatBtn = nullptr;
static lv_obj_t *lvglStyleButtonFlatBtnLabel = nullptr;
static lv_obj_t *lvglStyleButton3dBtn = nullptr;
static lv_obj_t *lvglStyleButton3dBtnLabel = nullptr;
static lv_obj_t *lvglStyleButtonBlackBtn = nullptr;
static lv_obj_t *lvglStyleButtonBlackBtnLabel = nullptr;
static lv_obj_t *lvglStyleTimezoneDd = nullptr;
static lv_obj_t *lvglStyleTopCenterNameBtn = nullptr;
static lv_obj_t *lvglStyleTopCenterNameBtnLabel = nullptr;
static lv_obj_t *lvglStyleTopCenterTimeBtn = nullptr;
static lv_obj_t *lvglStyleTopCenterTimeBtnLabel = nullptr;
static lv_obj_t *lvglStyleTimeoutSlider = nullptr;
static lv_obj_t *lvglStyleTimeoutValueLabel = nullptr;
static lv_obj_t *lvglStylePowerOffDropdown = nullptr;
static lv_obj_t *lvglStyleDeepSleepDropdown = nullptr;
static bool lvglStyleUiSyncing = false;
static lv_obj_t *lvglStyleHomeChatSw = nullptr;
static lv_obj_t *lvglStyleHomeRadioSw = nullptr;
static lv_obj_t *lvglStyleHomeMediaSw = nullptr;
static lv_obj_t *lvglStyleHomeInfoSw = nullptr;
static lv_obj_t *lvglStyleHomeGamesSw = nullptr;
static lv_obj_t *lvglStyleHomeMqttSw = nullptr;
static lv_obj_t *lvglStyleHomeConfigSw = nullptr;
static lv_obj_t *lvglStyleHomePowerSw = nullptr;
static lv_obj_t *lvglStyleHomeAirplaneSw = nullptr;
static lv_obj_t *lvglStyleHomeApSw = nullptr;
static lv_obj_t *lvglStyleHomeChatLabel = nullptr;
static lv_obj_t *lvglStyleHomeRadioLabel = nullptr;
static lv_obj_t *lvglStyleHomeMediaLabel = nullptr;
static lv_obj_t *lvglStyleHomeInfoLabel = nullptr;
static lv_obj_t *lvglStyleHomeGamesLabel = nullptr;
static lv_obj_t *lvglStyleHomeMqttLabel = nullptr;
static lv_obj_t *lvglStyleHomeConfigLabel = nullptr;
static lv_obj_t *lvglStyleHomePowerLabel = nullptr;
static lv_obj_t *lvglStyleHomeAirplaneLabel = nullptr;
static lv_obj_t *lvglStyleHomeApLabel = nullptr;
static lv_obj_t *lvglOtaCurrentLabel = nullptr;
static lv_obj_t *lvglOtaLatestLabel = nullptr;
static lv_obj_t *lvglOtaStatusLabel = nullptr;
static lv_obj_t *lvglOtaUpdateBtn = nullptr;
static lv_obj_t *lvglOtaUpdateBtnLabel = nullptr;
static lv_obj_t *lvglOtaReflashBtn = nullptr;
static lv_obj_t *lvglOtaReflashBtnLabel = nullptr;
static lv_obj_t *lvglOtaProgressBar = nullptr;
static lv_obj_t *lvglOtaProgressLabel = nullptr;
static lv_obj_t *lvglOtaReleaseHeaderLabel = nullptr;
static lv_obj_t *lvglOtaReleaseChecks[OTA_RELEASE_LIST_MAX] = {};
static lv_obj_t *lvglRemoteModuleDropdown = nullptr;
static lv_obj_t *lvglRemoteModuleLabel = nullptr;
static lv_obj_t *lvglRemoteChannelLabel = nullptr;
static lv_obj_t *lvglRemoteChannelTa = nullptr;
static lv_obj_t *lvglRemoteBaudLabel = nullptr;
static lv_obj_t *lvglRemoteBaudDropdown = nullptr;
static lv_obj_t *lvglRemoteModeLabel = nullptr;
static lv_obj_t *lvglRemoteModeDropdown = nullptr;
static lv_obj_t *lvglBatteryTrainStatusLabel = nullptr;
static lv_obj_t *lvglBatteryTrainCurrentLabel = nullptr;
static lv_obj_t *lvglBatteryTrainFullLabel = nullptr;
static lv_obj_t *lvglBatteryTrainEmptyLabel = nullptr;
static lv_obj_t *lvglBatteryTrainFactorLabel = nullptr;
static lv_obj_t *lvglBatteryTrainPowerLabel = nullptr;
static lv_obj_t *lvglBatteryTrainFullBtn = nullptr;
static lv_obj_t *lvglBatteryTrainDischargeBtn = nullptr;
static lv_obj_t *lvglBatteryTrainAutoBtn = nullptr;
static lv_obj_t *lvglConfigDeviceNameTa = nullptr;
static lv_obj_t *lvglLanguageDropdown = nullptr;
static lv_obj_t *lvglLanguageInfoLabel = nullptr;
static lv_obj_t *lvglConfigDeviceNameHeader = nullptr;
static lv_obj_t *lvglConfigBrightnessHeader = nullptr;
static lv_obj_t *lvglConfigVolumeHeader = nullptr;
static lv_obj_t *lvglConfigRgbHeader = nullptr;
static lv_obj_t *lvglConfigVibrationHeader = nullptr;
static lv_obj_t *lvglConfigMessageToneHeader = nullptr;
static lv_obj_t *lvglConfigStartupSoundLabel = nullptr;
static lv_obj_t *lvglConfigDeviceNameSaveBtnLabel = nullptr;
static lv_obj_t *lvglBrightnessSlider = nullptr;
static lv_obj_t *lvglBrightnessValueLabel = nullptr;
static lv_obj_t *lvglVolumeSlider = nullptr;
static lv_obj_t *lvglVolumeValueLabel = nullptr;
static lv_obj_t *lvglRgbLedSlider = nullptr;
static lv_obj_t *lvglVibrationDropdown = nullptr;
static lv_obj_t *lvglMessageToneDropdown = nullptr;
static lv_obj_t *lvglStartupSoundSw = nullptr;
static lv_obj_t *lvglKb = nullptr;
static uint8_t lvglWarmupScreenIndex = 0;
static unsigned long lvglWarmupLastMs = 0;
static lv_obj_t *lvglSnakeScoreLabel = nullptr;
static lv_obj_t *lvglSnakeBestLabel = nullptr;
static lv_obj_t *lvglSnakePauseBtnLabel = nullptr;
static lv_obj_t *lvglTetrisScoreLabel = nullptr;
static lv_obj_t *lvglTetrisBestLabel = nullptr;
static lv_obj_t *lvglTetrisPauseBtnLabel = nullptr;
static lv_obj_t *lvglSnakeBoardObj = nullptr;
static lv_obj_t *lvglTetrisBoardObj = nullptr;
static lv_obj_t *lvglSnakeOverlay = nullptr;
static lv_obj_t *lvglSnakeOverlayTitle = nullptr;
static lv_obj_t *lvglSnakeOverlaySubLabel = nullptr;
static lv_obj_t *lvglSnakeOverlayBtn = nullptr;
static lv_obj_t *lvglSnakeOverlayBtnLabel = nullptr;
static lv_obj_t *lvglTetrisOverlay = nullptr;
static lv_obj_t *lvglTetrisOverlayTitle = nullptr;
static lv_obj_t *lvglTetrisOverlaySubLabel = nullptr;
static lv_obj_t *lvglTetrisOverlayBtn = nullptr;
static lv_obj_t *lvglTetrisOverlayBtnLabel = nullptr;
static lv_obj_t *lvglCheckersBoardObj = nullptr;
static lv_obj_t *lvglCheckersModeLabel = nullptr;
static lv_obj_t *lvglCheckersTurnLabel = nullptr;
static lv_obj_t *lvglCheckersOverlay = nullptr;
static lv_obj_t *lvglCheckersOverlayTitle = nullptr;
static lv_obj_t *lvglCheckersOverlaySubLabel = nullptr;
static lv_obj_t *lvglCheckersOverlayBtn = nullptr;
static lv_obj_t *lvglCheckersOverlayBtnLabel = nullptr;
static lv_obj_t *lvglCheckersPeerPopup = nullptr;
static lv_obj_t *lvglCheckersPeerPopupTitle = nullptr;
static lv_obj_t *lvglCheckersPeerPopupList = nullptr;
static lv_obj_t *lvglCheckersVariantPopup = nullptr;
static lv_obj_t *lvglCheckersVariantPopupTitle = nullptr;
static lv_obj_t *lvglCheckersVariantPopupList = nullptr;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
static lv_obj_t *lvglSnake3dBoardObj = nullptr;
static lv_obj_t *lvglSnake3dScoreLabel = nullptr;
static lv_obj_t *lvglSnake3dBestLabel = nullptr;
static lv_obj_t *lvglSnake3dPauseBtnLabel = nullptr;
static lv_obj_t *lvglSnake3dOverlay = nullptr;
static lv_obj_t *lvglSnake3dOverlayTitle = nullptr;
static lv_obj_t *lvglSnake3dOverlaySubLabel = nullptr;
static lv_obj_t *lvglSnake3dOverlayBtn = nullptr;
static lv_obj_t *lvglSnake3dOverlayBtnLabel = nullptr;
#endif
static lv_obj_t *lvglTopBarRoot = nullptr;
static lv_obj_t *lvglSoundPopup = nullptr;
static lv_obj_t *lvglSoundPopupCard = nullptr;
static lv_obj_t *lvglSoundPopupVolumeSlider = nullptr;
static lv_obj_t *lvglSoundPopupVolumeValueLabel = nullptr;
static lv_obj_t *lvglSoundPopupVolumeIcon = nullptr;
static lv_obj_t *lvglSoundPopupVibrationDropdown = nullptr;
static lv_obj_t *lvglSoundPopupVibrationDisableBtn = nullptr;
static constexpr uint8_t LVGL_MAX_TOP_INDICATORS = 16;
static lv_obj_t *lvglTopIndicators[LVGL_MAX_TOP_INDICATORS] = {};
static bool lvglKeyboardShiftOneShot = false;
static bool lvglKeyboardShiftLocked = false;
static unsigned long lvglKeyboardShiftLastPressMs = 0;
static lv_obj_t *lvglKeyboardFocusedTa = nullptr;
static lv_obj_t *lvglKeyboardPaddedContainer = nullptr;
static lv_coord_t lvglKeyboardPaddedContainerPrevBottom = 0;
static bool lvglKeyboardUseLocalized = false;
static bool lvglKeyboardSpaceSwipeActive = false;
static bool lvglKeyboardSpaceSwipeTriggered = false;
static lv_point_t lvglKeyboardSpaceSwipeStart = {0, 0};
static uint8_t lvglTopIndicatorCount = 0;
static int lvglMqttEditIndex = 0;
static String lvglWifiPendingSsid;
static bool lvglWifiPwdConnectPending = false;
static bool lvglTouchDown = false;
static int16_t lvglLastTouchX = DISPLAY_CENTER_X;
static int16_t lvglLastTouchY = DISPLAY_CENTER_Y;
static bool lvglSwipeTracking = false;
static int16_t lvglSwipeStartX = 0;
static int16_t lvglSwipeStartY = 0;
static int16_t lvglSwipeLastX = 0;
static int16_t lvglSwipeLastY = 0;
static unsigned long lvglSwipeStartMs = 0;
static bool lvglSwipeCandidate = false;
static bool lvglSwipeHorizontalLocked = false;
static bool lvglSwipePressCancelled = false;
static lv_obj_t *lvglSwipeVisualScreen = nullptr;
static lv_obj_t *lvglSwipePreviewImg = nullptr;
static lv_img_dsc_t *lvglSwipePreviewSnapshot = nullptr;
static bool lvglSwipePreviewUsesSnapshot = false;
static int16_t lvglSwipeVisualOffsetX = 0;
static bool lvglSwipeVisualActive = false;
static bool lvglSwipeVisualAnimating = false;
static bool lvglGestureBlocked = false;
static bool lvglReorderOwnsHorizontalGesture = false;
static unsigned long lvglClickSuppressUntilMs = 0;
static unsigned long lvglLastTapReleaseMs = 0;
static int16_t lvglLastTapReleaseX = 0;
static int16_t lvglLastTapReleaseY = 0;
static unsigned long lvglUiPriorityHoldUntilMs = 0;
static unsigned long lvglLastTickMs = 0;
static unsigned long lvglLastInfoRefreshMs = 0;
static unsigned long lvglLastStatusRefreshMs = 0;
static unsigned long lvglLastMediaPlayerRefreshMs = 0;
static bool lvglMediaPlayerVisible = false;
static bool p2pDiscoveryEnabled = true;
static bool haChatUnread = false;
static bool p2pPairPromptVisible = false;
static bool p2pPairRequestPending = false;
static int p2pPairRequestDiscoveredIdx = -1;
static String p2pPairRequestPeerKey;
static String p2pPairRequestPeerName;
static IPAddress p2pPairRequestPeerIp((uint32_t)0);
static uint16_t p2pPairRequestPeerPort = 0;
static lv_obj_t *lvglOtaPostUpdateModal = nullptr;
static lv_obj_t *lvglOtaPostUpdateCard = nullptr;
static lv_obj_t *lvglOtaPostUpdateTitleLabel = nullptr;
static lv_obj_t *lvglOtaPostUpdateVersionLabel = nullptr;
static lv_obj_t *lvglOtaPostUpdateNotesWrap = nullptr;
static lv_obj_t *lvglOtaPostUpdateNotesLabel = nullptr;
static lv_obj_t *lvglOtaPostUpdateOkBtn = nullptr;
static lv_obj_t *lvglOtaPostUpdateOkBtnLabel = nullptr;
static bool lvglOtaPostUpdatePopupVisible = false;
static bool lvglOtaUpdatePromptVisible = false;
struct LvglTopIndicatorState {
    uint8_t batteryPercent;
    uint8_t wifiBars;
    uint8_t soundMode;
    bool batteryCharging;
    bool batteryPulse;
    bool airplaneMode;
    bool wifiConnected;
    bool wifiConnecting;
    bool apVisible;
    bool mqttConnected;
    bool unreadMailVisible;
    bool otaVisible;
    char topName[9];
};
static LvglTopIndicatorState lvglLastTopIndicatorState = {};
static bool lvglTopIndicatorStateValid = false;
TopBarCenterMode topBarCenterMode = TOP_BAR_CENTER_NAME;
int8_t topBarTimezoneGmtOffset = 0;
unsigned long topBarTimeSyncLastAttemptMs = 0;
#elif defined(UI_DISPLAY_SECTION_CORE)
void touchInit()
{
    touchLastSampleDown = false;
    touchLastSampleX = 0;
    touchLastSampleY = 0;
    if (TOUCH_CONTROLLER == TOUCH_CTRL_GT911) gt911Init();
    else cstInit();
}

void touchTryRecoverBus(const char *reason)
{
    if (touchReadFailStreak < TOUCH_REINIT_FAIL_THRESHOLD) return;
    const unsigned long now = millis();
    if (static_cast<unsigned long>(now - touchLastRecoveryMs) < TOUCH_REINIT_MIN_INTERVAL_MS) return;
    touchLastRecoveryMs = now;
    if (VERBOSE_SERIAL_DEBUG) {
        Serial.printf("[TOUCH] recover reason=%s fail_streak=%u\n",
                      reason ? reason : "-",
                      static_cast<unsigned int>(touchReadFailStreak));
    }
    touchInit();
}

void rotatePointByOffset(int16_t &x, int16_t &y, uint8_t offset)
{
    const int16_t w = tft.width();
    const int16_t h = tft.height();
    const int16_t ox = x;
    const int16_t oy = y;
    switch (offset & 0x03) {
        case 1: x = h - 1 - oy; y = ox; break;
        case 2: x = w - 1 - ox; y = h - 1 - oy; break;
        case 3: x = oy; y = w - 1 - ox; break;
        default: break;
    }
}

bool readScreenTouch(int16_t &sx, int16_t &sy)
{
    const unsigned long now = millis();
    if (static_cast<unsigned long>(now - touchLastPollMs) < TOUCH_POLL_INTERVAL_MS) {
        sx = touchLastSampleX;
        sy = touchLastSampleY;
        return touchLastSampleDown;
    }
    touchLastPollMs = now;

    bool irqLow = true;
    if (TOUCH_USE_IRQ) {
        irqLow = (digitalRead(TOUCH_IRQ) == LOW);
        if (!irqLow) {
            touchLastSampleDown = false;
            sx = touchLastSampleX;
            sy = touchLastSampleY;
            return false;
        }
    } else {
        touchLastNoIrqPollMs = now;
    }

    int16_t x = 0;
    int16_t y = 0;
    if (TOUCH_CONTROLLER == TOUCH_CTRL_GT911) {
        uint8_t status = 0;
        if (!gt911ReadRegs(GT911_REG_STATUS, &status, 1)) {
            touchReadFailStreak++;
            touchTryRecoverBus("gt911_read_status");
            return false;
        }
        const uint8_t touchCount = status & 0x0F;
        if (((status & 0x80U) == 0U) || touchCount == 0U) {
            touchGhostLowStreak = 0;
            if (status & 0x80U) gt911WriteReg8(GT911_REG_STATUS, 0);
            touchLastSampleDown = false;
            return false;
        }

        uint8_t data[7] = {0, 0, 0, 0, 0, 0, 0};
        if (!gt911ReadRegs(GT911_REG_POINT1, data, sizeof(data))) {
            touchReadFailStreak++;
            gt911WriteReg8(GT911_REG_STATUS, 0);
            touchTryRecoverBus("gt911_read_point");
            return false;
        }
        gt911WriteReg8(GT911_REG_STATUS, 0);
        x = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | data[1]);
        y = static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) | data[3]);
    } else {
        uint8_t touched = 0;
        if (!cstReadRegs(0x02, &touched, 1)) {
            touchReadFailStreak++;
            touchTryRecoverBus("read_touch_count");
            return false;
        }
        if (touched == 0) {
            if (TOUCH_USE_IRQ && irqLow) {
                // IRQ low but controller reports no touch -> likely stuck IRQ line.
                touchGhostLowStreak++;
                if (touchGhostLowStreak >= 25 && static_cast<unsigned long>(millis() - touchLastInitMs) > 1000UL) touchInit();
            } else {
                touchGhostLowStreak = 0;
            }
            touchLastSampleDown = false;
            return false;
        }

        uint8_t data[4] = {0, 0, 0, 0};
        if (!cstReadRegs(0x03, data, sizeof(data))) {
            touchReadFailStreak++;
            touchTryRecoverBus("read_touch_xy");
            return false;
        }

        x = static_cast<int16_t>(((data[0] & 0x0F) << 8) | data[1]);
        y = static_cast<int16_t>(((data[2] & 0x0F) << 8) | data[3]);
    }

    x = constrain(x, 0, tft.width() - 1);
    y = constrain(y, 0, tft.height() - 1);
    rotatePointByOffset(x, y, TOUCH_ROTATION_OFFSET);
    sx = x;
    sy = y;
    touchLastSampleX = x;
    touchLastSampleY = y;
    touchLastSampleDown = true;
    touchReadFailStreak = 0;
    touchGhostLowStreak = 0;
    return true;
}

void lvglFlushCb(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    const int32_t w = area->x2 - area->x1 + 1;
    const int32_t h = area->y2 - area->y1 + 1;
    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors(reinterpret_cast<uint16_t *>(&color_p->full), static_cast<uint32_t>(w * h), true);
    tft.endWrite();
    lv_disp_flush_ready(disp);
}

static inline bool lvglKeyboardVisible()
{
    return lvglKb && !lv_obj_has_flag(lvglKb, LV_OBJ_FLAG_HIDDEN);
}

static inline void lvglResetGestureTracking()
{
    lvglSwipeTracking = false;
    lvglSwipeCandidate = false;
    lvglSwipeHorizontalLocked = false;
    lvglSwipePressCancelled = false;
}

static inline lv_coord_t lvglClampSwipeBackOffset(int dx)
{
    return static_cast<lv_coord_t>(constrain(dx, 0, DISPLAY_WIDTH));
}

static bool lvglGetSwipeBackPreviewTarget(UiScreen current, UiScreen &target)
{
    switch (current) {
        case UI_CHAT:
            target = currentChatPeerKey.isEmpty() ? UI_HOME : UI_CHAT;
            return true;
        case UI_MEDIA:
        case UI_INFO:
        case UI_GAMES:
        case UI_RADIO_CONTROL:
        case UI_CONFIG:
            target = UI_HOME;
            return true;
        case UI_CHAT_PEERS:
            target = UI_CHAT;
            return true;
        case UI_WIFI_LIST:
        case UI_CONFIG_REMOTE_CONTROL:
        case UI_CONFIG_BATTERY:
        case UI_CONFIG_STYLE:
        case UI_CONFIG_LANGUAGE:
        case UI_CONFIG_OTA:
        case UI_CONFIG_MODULES:
        case UI_CONFIG_HC12:
        case UI_CONFIG_MQTT_CONFIG:
        case UI_CONFIG_MQTT_BUTTONS:
            target = UI_CONFIG;
            return true;
        case UI_CONFIG_MQTT_CONTROLS:
            target = UI_HOME;
            return true;
        case UI_CONFIG_RADIO_BUTTONS:
            target = UI_RADIO_CONTROL;
            return true;
        case UI_CONFIG_RADIO_RECEIVER_TEST:
            target = UI_CONFIG_RADIO_BUTTONS;
            return true;
        case UI_CONFIG_STYLE_HOME_ITEMS:
            target = UI_CONFIG_STYLE;
            return true;
        case UI_CONFIG_HC12_TERMINAL:
        case UI_CONFIG_HC12_INFO:
            target = UI_CONFIG_HC12;
            return true;
        case UI_GAME_SNAKE:
        case UI_GAME_TETRIS:
        case UI_GAME_CHECKERS:
        case UI_GAME_SNAKE3D:
            target = UI_GAMES;
            return true;
        default:
            return false;
    }
}

static void lvglSetObjXAnim(void *obj, int32_t v)
{
    lv_obj_t *o = static_cast<lv_obj_t *>(obj);
    if (!o) return;
    if (!lv_obj_is_valid(o)) return;
    lv_obj_set_x(o, static_cast<lv_coord_t>(v));
}

static uint16_t lvglSwipeBackAnimDuration(lv_coord_t fromX, lv_coord_t toX, bool commitToPrevious = false)
{
    const uint32_t distance = static_cast<uint32_t>(abs(static_cast<int32_t>(toX) - static_cast<int32_t>(fromX)));
    if (distance == 0U) return 0;
    if (commitToPrevious) {
        const uint32_t scaled = (distance * SWIPE_BACK_COMMIT_MAX_MS) / max<int16_t>(DISPLAY_WIDTH, 1);
        return static_cast<uint16_t>(constrain(scaled,
                                               static_cast<uint32_t>(SWIPE_BACK_COMMIT_MIN_MS),
                                               static_cast<uint32_t>(SWIPE_BACK_COMMIT_MAX_MS)));
    }
    const uint32_t scaled = (distance * UI_ANIM_MS) / max<int16_t>(DISPLAY_WIDTH, 1);
    return static_cast<uint16_t>(constrain(scaled,
                                           static_cast<uint32_t>(SWIPE_BACK_SNAP_MIN_MS),
                                           static_cast<uint32_t>(UI_ANIM_MS)));
}

static void lvglReleaseSwipeBackPreview()
{
    if (lvglSwipePreviewImg && lv_obj_is_valid(lvglSwipePreviewImg)) {
        lv_anim_del(lvglSwipePreviewImg, nullptr);   // delete any anim on this object
        lv_obj_del(lvglSwipePreviewImg);
    }
    lvglSwipePreviewImg = nullptr;

    if (lvglSwipePreviewSnapshot) lv_snapshot_free(lvglSwipePreviewSnapshot);
    lvglSwipePreviewSnapshot = nullptr;
    lvglSwipePreviewUsesSnapshot = false;
}

static lv_img_dsc_t *lvglCreateChatSwipeBackSnapshot()
{
    if (uiScreen != UI_CHAT || currentChatPeerKey.isEmpty() || !lvglScrChat || !lv_obj_is_valid(lvglScrChat)) return nullptr;

    const String savedPeerKey = currentChatPeerKey;
    currentChatPeerKey = "";
    chatClearCache();
    lvglRefreshChatLayout();
    lvglRefreshChatContactsUi();
    lvglRefreshChatUi();
    lv_obj_update_layout(lvglScrChat);
    lv_img_dsc_t *snapshot = lv_snapshot_take(lvglScrChat, LV_IMG_CF_TRUE_COLOR);

    currentChatPeerKey = savedPeerKey;
    chatReloadRecentMessagesFromSd(currentChatPeerKey);
    lvglRefreshChatLayout();
    lvglRefreshChatContactsUi();
    lvglRefreshChatUi();
    lv_obj_update_layout(lvglScrChat);

    return snapshot;
}

static bool lvglEnsureSwipeBackPreviewMounted()
{
    if (lvglKeyboardVisible()) return false;
    if (lvglSwipePreviewImg && lv_obj_is_valid(lvglSwipePreviewImg) &&
        (lvglSwipePreviewUsesSnapshot ? (lvglSwipePreviewSnapshot != nullptr) : true)) {
        return true;
    }
    const UiScreen currentUi = uiScreenSupportsSwipeBack(lvglSwipeSourceScreen) ? lvglSwipeSourceScreen : uiScreen;
    UiScreen targetUi = UI_HOME;
    if (!lvglGetSwipeBackPreviewTarget(currentUi, targetUi)) return false;

    lvglReleaseSwipeBackPreview();
    if (boardHasUsablePsram()) {
        lv_obj_t *targetScreen = nullptr;
        if (currentUi == UI_CHAT && !currentChatPeerKey.isEmpty() && targetUi == UI_CHAT) {
            lvglSwipePreviewSnapshot = lvglCreateChatSwipeBackSnapshot();
        } else {
            lvglEnsureScreenBuilt(targetUi);
            targetScreen = lvglScreenForUi(targetUi);
            if (!targetScreen || !lv_obj_is_valid(targetScreen) || lv_scr_act() == targetScreen) return false;
            lv_obj_update_layout(targetScreen);
            lvglSwipePreviewSnapshot = lv_snapshot_take(targetScreen, LV_IMG_CF_TRUE_COLOR);
        }
        if (lvglSwipePreviewSnapshot) {
            lvglSwipePreviewImg = lv_img_create(lv_layer_top());
            if (!lvglSwipePreviewImg) {
                lvglReleaseSwipeBackPreview();
                return false;
            }
            lvglSwipePreviewUsesSnapshot = true;
            lv_obj_clear_flag(lvglSwipePreviewImg, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_flag(lvglSwipePreviewImg, LV_OBJ_FLAG_IGNORE_LAYOUT);
            lv_img_set_src(lvglSwipePreviewImg, lvglSwipePreviewSnapshot);
            lv_obj_set_pos(lvglSwipePreviewImg, static_cast<lv_coord_t>(lvglSwipeVisualOffsetX - DISPLAY_WIDTH), 0);
            lv_obj_move_background(lvglSwipePreviewImg);
            return true;
        }
        lvglReleaseSwipeBackPreview();
    }

    lvglEnsureScreenBuilt(targetUi);
    lv_obj_t *targetScreen = lvglScreenForUi(targetUi);
    if (!targetScreen || !lv_obj_is_valid(targetScreen) || lv_scr_act() == targetScreen) return false;

    if (boardHasUsablePsram()) {
        lv_obj_update_layout(targetScreen);
        lvglSwipePreviewSnapshot = lv_snapshot_take(targetScreen, LV_IMG_CF_TRUE_COLOR);
    }

    lvglSwipePreviewImg = lv_obj_create(lv_layer_top());
    if (!lvglSwipePreviewImg) return false;
    lv_obj_clear_flag(lvglSwipePreviewImg, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(lvglSwipePreviewImg, LV_OBJ_FLAG_IGNORE_LAYOUT);
    lv_obj_remove_style_all(lvglSwipePreviewImg);
    lv_obj_set_size(lvglSwipePreviewImg, DISPLAY_WIDTH, DISPLAY_HEIGHT);
    lv_obj_set_style_bg_color(lvglSwipePreviewImg, lv_color_hex(0x111922), 0);
    lv_obj_set_style_bg_grad_color(lvglSwipePreviewImg, lv_color_hex(0x243A52), 0);
    lv_obj_set_style_bg_grad_dir(lvglSwipePreviewImg, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(lvglSwipePreviewImg, LV_OPA_COVER, 0);
    lv_obj_set_pos(lvglSwipePreviewImg, static_cast<lv_coord_t>(lvglSwipeVisualOffsetX - DISPLAY_WIDTH), 0);
    lv_obj_move_background(lvglSwipePreviewImg);
    return true;
}

static void lvglResetSwipeBackVisualState(bool resetScreenX = true)
{
    if (lvglSwipeVisualScreen && lv_obj_is_valid(lvglSwipeVisualScreen)) {
        lv_anim_del(lvglSwipeVisualScreen, nullptr);
        if (resetScreenX) lv_obj_set_x(lvglSwipeVisualScreen, 0);
    }
    if (lvglSwipePreviewImg && lv_obj_is_valid(lvglSwipePreviewImg)) {
        lv_anim_del(lvglSwipePreviewImg, lvglSetObjXAnim);
    }
    lvglReleaseSwipeBackPreview();
    lvglSwipeVisualScreen = nullptr;
    lvglSwipeVisualOffsetX = 0;
    lvglSwipeVisualActive = false;
    lvglSwipeVisualAnimating = false;
}

static void lvglApplySwipeBackVisual(lv_coord_t offsetX)
{
    lv_obj_t *screen = lv_scr_act();
    if (!screen || !lv_obj_is_valid(screen)) return;
    if (lvglSwipeVisualScreen && lvglSwipeVisualScreen != screen) {
        lvglResetSwipeBackVisualState(true);
    }
    lvglSwipeVisualScreen = screen;
    lv_anim_del(screen, lvglSetObjXAnim);
    lvglSwipeVisualAnimating = false;
    lvglSwipeVisualOffsetX = offsetX;
    lvglSwipeVisualActive = offsetX > 0;
    lvglEnsureSwipeBackPreviewMounted();
    if (lvglSwipePreviewImg && lv_obj_is_valid(lvglSwipePreviewImg)) {
        lv_obj_set_x(lvglSwipePreviewImg, static_cast<lv_coord_t>(offsetX - DISPLAY_WIDTH));
    }
    lv_obj_set_x(screen, offsetX);
}

static void lvglSwipeBackVisualAnimReady(lv_anim_t *a)
{
    lv_obj_t *screen = a ? static_cast<lv_obj_t *>(a->var) : nullptr;
    const bool navigateBack = a && lv_anim_get_user_data(a) != nullptr;

    if (navigateBack) {
        // Commit the already-visible swipe preview without starting a second screen transition.
        lvglSwipeVisualAnimating = true;
        lvglNavigateBackBySwipe(LV_SCR_LOAD_ANIM_NONE);
        return;
    }

    if (screen && lv_obj_is_valid(screen)) {
        lv_anim_del(screen, nullptr);
        lv_obj_set_x(screen, 0);
    }
    lvglSwipeVisualScreen = nullptr;
    lvglSwipeVisualOffsetX = 0;
    lvglSwipeVisualActive = false;
    lvglSwipeVisualAnimating = false;
}

static void lvglAnimateSwipeBackVisual(lv_coord_t targetX, bool navigateBack)
{
    lv_obj_t *screen = lvglSwipeVisualScreen;
    if (!screen || !lv_obj_is_valid(screen)) {
        lvglResetSwipeBackVisualState(false);
        if (navigateBack) lvglNavigateBackBySwipe();
        return;
    }

    lv_anim_del(screen, lvglSetObjXAnim);
    const lv_coord_t startX = lvglSwipeVisualActive ? lvglSwipeVisualOffsetX : static_cast<lv_coord_t>(lv_obj_get_x(screen));
    lv_obj_t *previewImg = (lvglSwipePreviewImg && lv_obj_is_valid(lvglSwipePreviewImg)) ? lvglSwipePreviewImg : nullptr;
    const lv_coord_t previewStartX = previewImg ? static_cast<lv_coord_t>(lv_obj_get_x(previewImg)) : 0;
    const lv_coord_t previewTargetX = navigateBack ? 0 : static_cast<lv_coord_t>(-DISPLAY_WIDTH);
    if (startX == targetX) {
        lvglResetSwipeBackVisualState(true);
        if (navigateBack) lvglNavigateBackBySwipe();
        return;
    }

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, screen);
    lv_anim_set_exec_cb(&a, lvglSetObjXAnim);
    lv_anim_set_values(&a, startX, targetX);
    lv_anim_set_time(&a, lvglSwipeBackAnimDuration(startX, targetX, navigateBack));
    lv_anim_set_path_cb(&a, navigateBack ? lv_anim_path_ease_in_out : lv_anim_path_ease_out);
    lv_anim_set_ready_cb(&a, lvglSwipeBackVisualAnimReady);
    lv_anim_set_user_data(&a, reinterpret_cast<void *>(navigateBack ? 1 : 0));
    if (previewImg) {
        lv_anim_t previewAnim;
        lv_anim_init(&previewAnim);
        lv_anim_set_var(&previewAnim, previewImg);
        lv_anim_set_exec_cb(&previewAnim, lvglSetObjXAnim);
        lv_anim_set_values(&previewAnim, previewStartX, previewTargetX);
        lv_anim_set_time(&previewAnim, lvglSwipeBackAnimDuration(previewStartX, previewTargetX, navigateBack));
        lv_anim_set_path_cb(&previewAnim, navigateBack ? lv_anim_path_ease_in_out : lv_anim_path_ease_out);
        lv_anim_start(&previewAnim);
    }
    lvglSwipeVisualOffsetX = startX;
    lvglSwipeVisualActive = true;
    lvglSwipeVisualAnimating = true;
    lv_anim_start(&a);
}

static bool lvglTouchOwnsHorizontalGesture()
{
    if (lvglReorderOwnsHorizontalGesture) return true;
    if (uiScreen == UI_CONFIG_STYLE_HOME_ITEMS) return false;

    lv_obj_t *obj = lv_indev_get_obj_act();
    if (!obj || !lv_obj_is_valid(obj)) return false;

    for (lv_obj_t *cur = obj; cur && lv_obj_is_valid(cur); cur = lv_obj_get_parent(cur)) {
        if (lv_obj_check_type(cur, &lv_slider_class) || lv_obj_check_type(cur, &lv_switch_class)) {
            return true;
        }
        if (lv_obj_has_flag(cur, LV_OBJ_FLAG_SCROLLABLE)) {
            const lv_dir_t scrollDir = lv_obj_get_scroll_dir(cur);
            if ((scrollDir & LV_DIR_HOR) != 0) return true;
        }
    }
    return false;
}

void lvglGestureBlockEvent(lv_event_t *e)
{
    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED) lvglGestureBlocked = true;
    else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST || code == LV_EVENT_CLICKED) lvglGestureBlocked = false;
}

static inline bool lvglClickSuppressed()
{
    return static_cast<long>(lvglClickSuppressUntilMs - millis()) > 0;
}

static inline bool uiScreenKeepsDisplayAwake()
{
    return uiScreen == UI_GAME_SNAKE || uiScreen == UI_GAME_TETRIS || uiScreen == UI_GAME_CHECKERS || uiScreen == UI_GAME_SNAKE3D;
}

static inline bool uiScreenNeedsRealtimeMessaging()
{
    return uiScreen == UI_CHAT ||
           uiScreen == UI_CHAT_PEERS ||
           uiScreen == UI_GAME_CHECKERS ||
           uiScreen == UI_CONFIG_HC12 ||
           uiScreen == UI_CONFIG_HC12_TERMINAL ||
           uiScreen == UI_CONFIG_HC12_INFO ||
           uiScreen == UI_CONFIG_MQTT_CONFIG ||
           uiScreen == UI_CONFIG_MQTT_CONTROLS;
}

static inline bool uiPerformancePriorityActive(bool touchDown)
{
    const unsigned long now = millis();
    return displayAwake &&
           (touchDown ||
            wakeTouchReleaseGuard ||
            lvglSwipeTracking ||
            lvglSwipeVisualActive ||
            lvglSwipeVisualAnimating ||
            static_cast<long>(lvglUiPriorityHoldUntilMs - now) > 0 ||
            lv_anim_count_running() > 0U);
}

static inline void lvglSuppressClicksAfterGesture()
{
    lvglClickSuppressUntilMs = millis() + CLICK_SUPPRESS_AFTER_GESTURE_MS;
}

static void lvglClickFilterEvent(lv_event_t *e)
{
    if (!lvglClickSuppressed()) return;
    lv_event_stop_processing(e);
}

static void lvglFilteredClickFlashEvent(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    if (!obj || !lv_obj_is_valid(obj)) return;

    touchFeedbackQueue();
    lv_obj_add_state(obj, LV_STATE_CHECKED);

    lv_timer_t *timer = lv_timer_create(
        [](lv_timer_t *timer) {
            lv_obj_t *target = static_cast<lv_obj_t *>(timer ? timer->user_data : nullptr);
            if (target && lv_obj_is_valid(target)) {
                lv_obj_clear_state(target, LV_STATE_CHECKED);
            }
            if (timer) lv_timer_del(timer);
        },
        UI_BUTTON_CLICK_FLASH_MS,
        obj);

    if (timer) lv_timer_set_repeat_count(timer, 1);
}


void lvglTouchReadCb(lv_indev_drv_t *indev, lv_indev_data_t *data)
{
    (void)indev;
    const unsigned long now = millis();
    int16_t x = 0;
    int16_t y = 0;
    const bool rawDown = readScreenTouch(x, y);
    if (rawDown || wakeTouchReleaseGuard) lvglUiPriorityHoldUntilMs = now + UI_INTERACTION_COOLDOWN_MS;
    if (rawDown) {
        lvglLastTouchX = x;
        lvglLastTouchY = y;
    }
    if (screensaverActive) {
        if (rawDown && !wakeTouchReleaseGuard) {
            wakeTouchReleaseGuard = true;
            wakeTouchConfirmCount = WAKE_TOUCH_RELEASE_STABLE_POLLS;
            screensaverSetActive(false);
            lastUserActivityMs = millis();
        }
        if (wakeTouchReleaseGuard) {
            if (rawDown) {
                wakeTouchConfirmCount = WAKE_TOUCH_RELEASE_STABLE_POLLS;
            } else if (wakeTouchConfirmCount > 0) {
                wakeTouchConfirmCount--;
            } else {
                wakeTouchReleaseGuard = false;
            }
            lvglTouchDown = false;
            lvglResetGestureTracking();
            data->state = LV_INDEV_STATE_REL;
            data->point.x = lvglLastTouchX;
            data->point.y = lvglLastTouchY;
            return;
        }
    }
    if (!displayAwake) {
        if (rawDown && !wakeTouchReleaseGuard) {
            wakeTouchReleaseGuard = true;
            wakeTouchConfirmCount = WAKE_TOUCH_RELEASE_STABLE_POLLS;
            displaySetAwake(true);
            lastUserActivityMs = millis();
        }
        if (wakeTouchReleaseGuard) {
            if (rawDown) {
                wakeTouchConfirmCount = WAKE_TOUCH_RELEASE_STABLE_POLLS;
            } else if (wakeTouchConfirmCount > 0) {
                wakeTouchConfirmCount--;
            } else {
                wakeTouchReleaseGuard = false;
            }
            lvglTouchDown = false;
            lvglResetGestureTracking();
            data->state = LV_INDEV_STATE_REL;
            data->point.x = lvglLastTouchX;
            data->point.y = lvglLastTouchY;
            return;
        }
    }
    if (wakeTouchReleaseGuard) {
        if (rawDown) {
            wakeTouchConfirmCount = WAKE_TOUCH_RELEASE_STABLE_POLLS;
        } else if (wakeTouchConfirmCount > 0) {
            wakeTouchConfirmCount--;
        } else {
            wakeTouchReleaseGuard = false;
        }
        lvglTouchDown = false;
        lvglResetGestureTracking();
        data->state = LV_INDEV_STATE_REL;
        data->point.x = lvglLastTouchX;
        data->point.y = lvglLastTouchY;
        return;
    }
    lvglTouchDown = rawDown;
    if (lvglTouchDown) {
        if (!lvglSwipeTracking) {
            const unsigned long now = millis();
            if (displayAwake &&
                !uiScreenKeepsDisplayAwake() &&
                !lvglKeyboardVisible() &&
                lvglLastTapReleaseMs != 0 &&
                static_cast<unsigned long>(now - lvglLastTapReleaseMs) <= static_cast<unsigned long>(DOUBLE_TAP_MAX_GAP) &&
                abs(static_cast<int>(x) - static_cast<int>(lvglLastTapReleaseX)) <= DOUBLE_TAP_MAX_MOVE &&
                abs(static_cast<int>(y) - static_cast<int>(lvglLastTapReleaseY)) <= DOUBLE_TAP_MAX_MOVE) {
                lvglLastTapReleaseMs = 0;
                lvglTouchDown = false;
                lvglResetGestureTracking();
                lvglSuppressClicksAfterGesture();
                if (screensaverEnabled) screensaverSetActive(true);
                else displaySetAwake(false);
                wakeTouchReleaseGuard = true;
                wakeTouchConfirmCount = WAKE_TOUCH_RELEASE_STABLE_POLLS;
                data->state = LV_INDEV_STATE_REL;
                data->point.x = lvglLastTouchX;
                data->point.y = lvglLastTouchY;
                return;
            }
        }

        const bool swallowPressForSwipe =
            lvglSwipeCandidate &&
            lvglSwipeHorizontalLocked;

        if (swallowPressForSwipe) {
            data->state = LV_INDEV_STATE_REL;
            data->point.x = lvglLastTouchX;
            data->point.y = lvglLastTouchY;
            return;
        }

        data->state = LV_INDEV_STATE_PR;
        data->point.x = x;
        data->point.y = y;
    } else {
        data->state = LV_INDEV_STATE_REL;
        data->point.x = lvglLastTouchX;
        data->point.y = lvglLastTouchY;
    }
}

void lvglSyncStatusLine()
{
    if (!lvglStatusLabel) return;
    String next = "Status: " + uiStatusLine;
    const char *current = lv_label_get_text(lvglStatusLabel);
    if (current && next == current) return;
    lv_label_set_text(lvglStatusLabel, next.c_str());
}

static void lvglLabelSetTextIfChanged(lv_obj_t *label, const char *text)
{
    if (!label) return;
    const char *next = text ? text : "";
    const char *current = lv_label_get_text(label);
    if (current && strcmp(current, next) == 0) return;
    lv_label_set_text(label, next);
}

static void lvglLabelSetTextIfChanged(lv_obj_t *label, const String &text)
{
    lvglLabelSetTextIfChanged(label, text.c_str());
}

static void lvglBarSetValueIfChanged(lv_obj_t *bar, int32_t value, lv_anim_enable_t anim)
{
    if (!bar) return;
    if (lv_bar_get_value(bar) == value) return;
    lv_bar_set_value(bar, value, anim);
}

static void lvglApplyStyleScreenControlStyles()
{
    if (lvglStyleScreensaverSw) {
        lv_obj_set_style_bg_color(lvglStyleScreensaverSw, lv_color_hex(0x48515C), LV_PART_MAIN);
        lv_obj_set_style_bg_color(lvglStyleScreensaverSw, lv_color_hex(0x3A8F4B), LV_PART_INDICATOR | LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(lvglStyleScreensaverSw, lv_color_hex(0xDCE7F2), LV_PART_KNOB);
    }
    if (lvglStyleScreenLockSw) {
        lv_obj_set_style_bg_color(lvglStyleScreenLockSw, lv_color_hex(0x48515C), LV_PART_MAIN);
        lv_obj_set_style_bg_color(lvglStyleScreenLockSw, lv_color_hex(0x3A8F4B), LV_PART_INDICATOR | LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(lvglStyleScreenLockSw, lv_color_hex(0xDCE7F2), LV_PART_KNOB);
    }
    if (lvglStyleMenuIconsSw) {
        lv_obj_set_style_bg_color(lvglStyleMenuIconsSw, lv_color_hex(0x48515C), LV_PART_MAIN);
        lv_obj_set_style_bg_color(lvglStyleMenuIconsSw, lv_color_hex(0x3A8F4B), LV_PART_INDICATOR | LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(lvglStyleMenuIconsSw, lv_color_hex(0xDCE7F2), LV_PART_KNOB);
    }
    if (lvglStyleTouchVibrationSw) {
        lv_obj_set_style_bg_color(lvglStyleTouchVibrationSw, lv_color_hex(0x48515C), LV_PART_MAIN);
        lv_obj_set_style_bg_color(lvglStyleTouchVibrationSw, lv_color_hex(0x3A8F4B), LV_PART_INDICATOR | LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(lvglStyleTouchVibrationSw, lv_color_hex(0xDCE7F2), LV_PART_KNOB);
    }
    if (lvglStyleTouchSoundSw) {
        lv_obj_set_style_bg_color(lvglStyleTouchSoundSw, lv_color_hex(0x48515C), LV_PART_MAIN);
        lv_obj_set_style_bg_color(lvglStyleTouchSoundSw, lv_color_hex(0x3A8F4B), LV_PART_INDICATOR | LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(lvglStyleTouchSoundSw, lv_color_hex(0xDCE7F2), LV_PART_KNOB);
    }
    if (lvglStyleTimeoutSlider) {
        lv_obj_set_style_bg_color(lvglStyleTimeoutSlider, lv_color_hex(0x2A3340), LV_PART_MAIN);
        lv_obj_set_style_bg_color(lvglStyleTimeoutSlider, lv_color_hex(0x4FC3F7), LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(lvglStyleTimeoutSlider, lv_color_hex(0xE5ECF3), LV_PART_KNOB);
    }
}

static void lvglApplyConfigScreenControlStyles()
{
    if (lvglBrightnessSlider) {
        const lv_color_t normalCol = lv_color_hex(0x52B788);
        const lv_color_t highCol = lv_color_hex(0xC94B4B);
        lv_obj_set_style_bg_color(lvglBrightnessSlider, lv_color_hex(0x2A3340), LV_PART_MAIN);
        lv_obj_set_style_bg_color(lvglBrightnessSlider, normalCol, LV_PART_INDICATOR);
        lv_obj_set_style_bg_grad_color(lvglBrightnessSlider, highCol, LV_PART_INDICATOR);
        lv_obj_set_style_bg_grad_dir(lvglBrightnessSlider, LV_GRAD_DIR_HOR, LV_PART_INDICATOR);
        lv_obj_set_style_bg_main_stop(lvglBrightnessSlider, 204, LV_PART_INDICATOR);
        lv_obj_set_style_bg_grad_stop(lvglBrightnessSlider, 255, LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(lvglBrightnessSlider, lv_color_hex(0xE5ECF3), LV_PART_KNOB);
    }
    if (lvglVolumeSlider) {
        lv_obj_set_style_bg_color(lvglVolumeSlider, lv_color_hex(0x2A3340), LV_PART_MAIN);
        lv_obj_set_style_bg_color(lvglVolumeSlider, lv_color_hex(0x4FC3F7), LV_PART_INDICATOR);
        lv_obj_set_style_bg_grad_color(lvglVolumeSlider, lv_color_hex(0x80ED99), LV_PART_INDICATOR);
        lv_obj_set_style_bg_grad_dir(lvglVolumeSlider, LV_GRAD_DIR_HOR, LV_PART_INDICATOR);
        lv_obj_set_style_bg_main_stop(lvglVolumeSlider, 96, LV_PART_INDICATOR);
        lv_obj_set_style_bg_grad_stop(lvglVolumeSlider, 255, LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(lvglVolumeSlider, lv_color_hex(0xE5ECF3), LV_PART_KNOB);
    }
    if (lvglRgbLedSlider) {
        lv_obj_set_style_bg_color(lvglRgbLedSlider, lv_color_hex(0x2A3340), LV_PART_MAIN);
        lv_obj_set_style_bg_color(lvglRgbLedSlider, lv_color_hex(0x7C4DFF), LV_PART_INDICATOR);
        lv_obj_set_style_bg_grad_color(lvglRgbLedSlider, lv_color_hex(0xFF6B6B), LV_PART_INDICATOR);
        lv_obj_set_style_bg_grad_dir(lvglRgbLedSlider, LV_GRAD_DIR_HOR, LV_PART_INDICATOR);
        lv_obj_set_style_bg_main_stop(lvglRgbLedSlider, 96, LV_PART_INDICATOR);
        lv_obj_set_style_bg_grad_stop(lvglRgbLedSlider, 255, LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(lvglRgbLedSlider, lv_color_hex(0xE5ECF3), LV_PART_KNOB);
    }
    if (lvglVibrationDropdown) {
        lv_obj_set_style_bg_color(lvglVibrationDropdown, lv_color_hex(0x111922), 0);
        lv_obj_set_style_text_color(lvglVibrationDropdown, lv_color_hex(0xE5ECF3), 0);
        lv_obj_set_style_border_color(lvglVibrationDropdown, lv_color_hex(0x2F4658), 0);
        lv_obj_set_style_border_width(lvglVibrationDropdown, 1, 0);
        lv_obj_set_style_radius(lvglVibrationDropdown, 8, 0);
    }
    if (lvglMessageToneDropdown) {
        lv_obj_set_style_bg_color(lvglMessageToneDropdown, lv_color_hex(0x111922), 0);
        lv_obj_set_style_text_color(lvglMessageToneDropdown, lv_color_hex(0xE5ECF3), 0);
        lv_obj_set_style_border_color(lvglMessageToneDropdown, lv_color_hex(0x2F4658), 0);
        lv_obj_set_style_border_width(lvglMessageToneDropdown, 1, 0);
        lv_obj_set_style_radius(lvglMessageToneDropdown, 8, 0);
    }
    if (lvglStartupSoundSw) {
        lv_obj_set_style_bg_color(lvglStartupSoundSw, lv_color_hex(0x48515C), LV_PART_MAIN);
        lv_obj_set_style_bg_color(lvglStartupSoundSw, lv_color_hex(0x3A8F4B), LV_PART_INDICATOR | LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(lvglStartupSoundSw, lv_color_hex(0xDCE7F2), LV_PART_KNOB);
    }
}

void lvglRegisterTopIndicator(lv_obj_t *obj)
{
    if (!obj) return;
    if (lvglTopIndicatorCount >= LVGL_MAX_TOP_INDICATORS) return;
    lvglTopIndicators[lvglTopIndicatorCount++] = obj;
    lvglTopIndicatorStateValid = false;
}

wl_status_t wifiStatusSafe()
{
    if (networkSuspendedForAudio || airplaneModeEnabled) return WL_DISCONNECTED;
    return WiFi.status();
}

bool wifiConnectedSafe()
{
    return wifiStatusSafe() == WL_CONNECTED;
}

bool wifiLanAvailableSafe()
{
    if (networkSuspendedForAudio || airplaneModeEnabled) return false;
    return wifiConnectedSafe() || apModeActive;
}

int32_t wifiRssiSafe()
{
    if (!wifiConnectedSafe()) return -127;
    return WiFi.RSSI();
}

String wifiSsidSafe()
{
    if (!wifiConnectedSafe()) return "";
    return WiFi.SSID();
}

String wifiIpSafe()
{
    if (!wifiConnectedSafe()) return String("-");
    return WiFi.localIP().toString();
}

IPAddress wifiLanIpSafe()
{
    if (networkSuspendedForAudio || airplaneModeEnabled) return IPAddress((uint32_t)0);
    if (wifiConnectedSafe()) return WiFi.localIP();
    if (apModeActive) return WiFi.softAPIP();
    return IPAddress((uint32_t)0);
}

IPAddress wifiLanBroadcastIpSafe()
{
    IPAddress ip = wifiLanIpSafe();
    if (ip[0] == 0) return IPAddress((uint32_t)0);
    ip[3] = 255;
    return ip;
}

void lvglDrawRectSolid(lv_draw_ctx_t *drawCtx, int x, int y, int w, int h, lv_color_t color, lv_opa_t opa = LV_OPA_COVER)
{
    if (!drawCtx || w <= 0 || h <= 0) return;
    lv_draw_rect_dsc_t dsc;
    lv_draw_rect_dsc_init(&dsc);
    dsc.bg_opa = opa;
    dsc.bg_color = color;
    dsc.border_width = 0;
    dsc.radius = 1;
    lv_area_t a;
    a.x1 = static_cast<lv_coord_t>(x);
    a.y1 = static_cast<lv_coord_t>(y);
    a.x2 = static_cast<lv_coord_t>(x + w - 1);
    a.y2 = static_cast<lv_coord_t>(y + h - 1);
    lv_draw_rect(drawCtx, &dsc, &a);
}

void lvglDrawLineSeg(lv_draw_ctx_t *drawCtx, int x1, int y1, int x2, int y2, lv_color_t color, uint8_t width = 2, lv_opa_t opa = LV_OPA_COVER)
{
    if (!drawCtx) return;
    lv_draw_line_dsc_t dsc;
    lv_draw_line_dsc_init(&dsc);
    dsc.color = color;
    dsc.width = width;
    dsc.opa = opa;
    dsc.round_start = 1;
    dsc.round_end = 1;
    lv_point_t p1 = {static_cast<lv_coord_t>(x1), static_cast<lv_coord_t>(y1)};
    lv_point_t p2 = {static_cast<lv_coord_t>(x2), static_cast<lv_coord_t>(y2)};
    lv_draw_line(drawCtx, &dsc, &p1, &p2);
}

void lvglTopIndicatorsDrawEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_DRAW_MAIN) return;
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *drawCtx = lv_event_get_draw_ctx(e);
    if (!obj || !drawCtx) return;

    lv_area_t c;
    lv_obj_get_coords(obj, &c);
    const int ox = static_cast<int>(c.x1);
    const int oy = static_cast<int>(c.y1);
    const int ow = static_cast<int>(c.x2 - c.x1 + 1);
    const int topY = oy;
    const bool battPulse = batteryCharging && (((millis() / 280UL) & 0x1U) != 0U);

    const bool connected = wifiConnectedSafe();
    const bool connecting = !connected && !airplaneModeEnabled && bootStaConnectInProgress;
    const int rssi = connected ? wifiRssiSafe() : -127;
    const bool mqttConnected = mqttCfg.enabled && mqttClient.connected();
    int bars = 0;
    if (connected) {
        if (rssi >= -55) bars = 4;
        else if (rssi >= -67) bars = 3;
        else if (rssi >= -75) bars = 2;
        else bars = 1;
    }

    uint8_t batt = batteryPercent;
    if (batt > 100) batt = 100;
    int battFillW = (static_cast<int>(batt) * 30) / 100;
    if (batt > 0 && battFillW < 1) battFillW = 1;
    if (battPulse) battFillW = min(30, battFillW + 3);

    lv_color_t battCol = lv_color_hex(0x66BB6A);
    if (batt < 25) battCol = lv_color_hex(0xE45B5B);
    else if (batt < 55) battCol = lv_color_hex(0xF2C35E);
    if (batteryCharging) battCol = lv_color_hex(battPulse ? 0x81E8FF : 0x4FC3F7);

    static const int antHeights[4] = {6, 10, 16, 22};
    const int antX = ox + 10;
    if (airplaneModeEnabled) {
        lv_draw_img_dsc_t planeDsc;
        lv_draw_img_dsc_init(&planeDsc);
        planeDsc.opa = LV_OPA_COVER;
        const int planeW = static_cast<int>(img_airplane_mode_icon.header.w);
        const int planeH = static_cast<int>(img_airplane_mode_icon.header.h);
        const int planeX = antX + 2;
        const int planeY = topY + 4;
        lv_area_t planeArea = {
            static_cast<lv_coord_t>(planeX),
            static_cast<lv_coord_t>(planeY),
            static_cast<lv_coord_t>(planeX + planeW - 1),
            static_cast<lv_coord_t>(planeY + planeH - 1),
        };
        lv_draw_img(drawCtx, &planeDsc, &planeArea, &img_airplane_mode_icon);
    } else {
        const uint8_t animPhase = static_cast<uint8_t>((millis() / WIFI_CONNECT_BARS_ANIM_PERIOD_MS) & 0x03U);
        for (int i = 0; i < 4; i++) {
            const bool on = connected ? (i < bars) : (connecting && i == animPhase);
            lv_color_t col = on ? lv_color_hex(0x79E28A) : lv_color_hex(0x30404A);
            lvglDrawRectSolid(drawCtx, antX + (i * 8), topY + 23 - antHeights[i], 4, antHeights[i], col, on ? LV_OPA_COVER : LV_OPA_70);
        }
    }
    if (!connected && !airplaneModeEnabled) {
        if (connecting) {
            // Animated bars above already communicate the in-progress STA connection attempt.
        } else if (apModeActive) {
            const lv_color_t apCol = lv_color_hex(0xF2993A);
            const int ax = antX - 1;
            const int ay = topY + 4;
            lvglDrawRectSolid(drawCtx, ax + 0, ay + 4, 3, 14, apCol);
            lvglDrawRectSolid(drawCtx, ax + 10, ay + 4, 3, 14, apCol);
            lvglDrawRectSolid(drawCtx, ax + 2, ay + 1, 3, 5, apCol);
            lvglDrawRectSolid(drawCtx, ax + 8, ay + 1, 3, 5, apCol);
            lvglDrawRectSolid(drawCtx, ax + 4, ay + 0, 4, 3, apCol);
            lvglDrawRectSolid(drawCtx, ax + 3, ay + 8, 7, 3, apCol);

            const int px = ax + 18;
            lvglDrawRectSolid(drawCtx, px + 0, ay + 0, 3, 18, apCol);
            lvglDrawRectSolid(drawCtx, px + 3, ay + 0, 8, 3, apCol);
            lvglDrawRectSolid(drawCtx, px + 8, ay + 3, 3, 5, apCol);
            lvglDrawRectSolid(drawCtx, px + 3, ay + 8, 8, 3, apCol);
        }
    }

    if (mqttConnected) {
        const lv_color_t mqttCol = lv_color_hex(0x7FDBCA);
        const int mx = antX + 38;
        const int my = topY + 4;
        lvglDrawLineSeg(drawCtx, mx + 5, my + 4, mx + 12, my + 1, mqttCol, 2);
        lvglDrawLineSeg(drawCtx, mx + 5, my + 4, mx + 12, my + 15, mqttCol, 2);
        lvglDrawRectSolid(drawCtx, mx + 1, my + 1, 8, 8, mqttCol);
        lvglDrawRectSolid(drawCtx, mx + 9, my + 0, 8, 8, mqttCol, LV_OPA_90);
        lvglDrawRectSolid(drawCtx, mx + 9, my + 12, 8, 8, mqttCol, LV_OPA_90);
    }

    const bool conversationOpen = (uiScreen == UI_CHAT) && !currentChatPeerKey.isEmpty();
    const bool showUnreadMail = chatHasUnreadMessages() && !conversationOpen;
    const bool showOtaIcon = otaUpdateAvailable;
    const int battX = ox + ow - 54;
    const int soundX = battX - 28;
    const int mailX = soundX - 28;
    const int otaX = mailX - 28;
    if (showOtaIcon) {
        const lv_color_t otaCol = lv_color_hex(0x79E28A);
        const int otaY = topY + 4;
        lvglDrawRectSolid(drawCtx, otaX + 2, otaY + 1, 16, 12, otaCol, LV_OPA_30);
        lvglDrawLineSeg(drawCtx, otaX + 10, otaY + 2, otaX + 10, otaY + 12, otaCol, 2);
        lvglDrawLineSeg(drawCtx, otaX + 6, otaY + 8, otaX + 10, otaY + 12, otaCol, 2);
        lvglDrawLineSeg(drawCtx, otaX + 14, otaY + 8, otaX + 10, otaY + 12, otaCol, 2);
        lvglDrawRectSolid(drawCtx, otaX + 4, otaY + 14, 12, 2, otaCol);
    }
    {
        lv_draw_img_dsc_t soundDsc;
        lv_draw_img_dsc_init(&soundDsc);
        soundDsc.opa = LV_OPA_COVER;
        const lv_img_dsc_t *soundIcon = uiSoundModeIcon();
        if (soundIcon) {
            const int soundW = static_cast<int>(soundIcon->header.w);
            const int soundH = static_cast<int>(soundIcon->header.h);
            const int soundY = topY + 5;
            lv_area_t soundArea = {
                static_cast<lv_coord_t>(soundX),
                static_cast<lv_coord_t>(soundY),
                static_cast<lv_coord_t>(soundX + soundW - 1),
                static_cast<lv_coord_t>(soundY + soundH - 1),
            };
            lv_draw_img(drawCtx, &soundDsc, &soundArea, soundIcon);
        }
    }
    if (showUnreadMail) {
        const lv_color_t mailCol = lv_color_hex(0xF2C35E);
        const int mailY = topY + 5;
        lv_draw_rect_dsc_t mailFrame;
        lv_draw_rect_dsc_init(&mailFrame);
        mailFrame.bg_opa = LV_OPA_TRANSP;
        mailFrame.border_width = 1;
        mailFrame.border_color = mailCol;
        mailFrame.radius = 2;
        lv_area_t mailA;
        mailA.x1 = static_cast<lv_coord_t>(mailX);
        mailA.y1 = static_cast<lv_coord_t>(mailY);
        mailA.x2 = static_cast<lv_coord_t>(mailX + 19);
        mailA.y2 = static_cast<lv_coord_t>(mailY + 13);
        lv_draw_rect(drawCtx, &mailFrame, &mailA);
        lvglDrawLineSeg(drawCtx, mailX + 1, mailY + 1, mailX + 10, mailY + 7, mailCol, 1);
        lvglDrawLineSeg(drawCtx, mailX + 18, mailY + 1, mailX + 10, mailY + 7, mailCol, 1);
    }

    lv_draw_rect_dsc_t frame;
    lv_draw_rect_dsc_init(&frame);
    frame.bg_opa = LV_OPA_TRANSP;
    frame.border_width = 1;
    frame.border_color = lv_color_hex(0x8A97A4);
    frame.radius = 2;
    lv_area_t batA;
    batA.x1 = static_cast<lv_coord_t>(battX);
    batA.y1 = static_cast<lv_coord_t>(topY + 2);
    batA.x2 = static_cast<lv_coord_t>(battX + 35);
    batA.y2 = static_cast<lv_coord_t>(topY + 21);
    lv_draw_rect(drawCtx, &frame, &batA);
    lvglDrawRectSolid(drawCtx, battX + 36, topY + 8, 6, 8, batteryCharging ? battCol : lv_color_hex(0x8A97A4));
    if (battFillW > 0) lvglDrawRectSolid(drawCtx, battX + 3, topY + 5, battFillW, 14, battCol);

    String topName = topBarCenterText();
    const lv_font_t *topNameFont = lv_obj_get_style_text_font(obj, LV_PART_MAIN);
    if (!topNameFont) topNameFont = lv_theme_get_font_small(obj);
    const lv_coord_t topNameLineH = topNameFont ? static_cast<lv_coord_t>(lv_font_get_line_height(topNameFont)) : 16;
    const lv_coord_t topNameY = static_cast<lv_coord_t>(topY + ((UI_TOP_BAR_H - topNameLineH) / 2) + 1);
    lv_draw_label_dsc_t labelDsc;
    lv_draw_label_dsc_init(&labelDsc);
    labelDsc.color = lv_color_hex(0xC8D3DD);
    labelDsc.opa = LV_OPA_COVER;
    labelDsc.align = LV_TEXT_ALIGN_CENTER;
    labelDsc.font = topNameFont;
    lv_area_t nameA;
    nameA.x1 = static_cast<lv_coord_t>(ox + 56);
    nameA.y1 = topNameY;
    nameA.x2 = static_cast<lv_coord_t>(soundX - ((showUnreadMail ? 34 : 8) + (showOtaIcon ? 28 : 0)));
    nameA.y2 = static_cast<lv_coord_t>(topNameY + topNameLineH + 2);
    lv_draw_label(drawCtx, &labelDsc, &nameA, topName.c_str(), nullptr);
    lv_area_t nameABold = nameA;
    nameABold.x1 += 1;
    nameABold.x2 += 1;
    lv_draw_label(drawCtx, &labelDsc, &nameABold, topName.c_str(), nullptr);
    lv_area_t nameABold2 = nameA;
    nameABold2.y1 += 1;
    nameABold2.y2 += 1;
    lv_draw_label(drawCtx, &labelDsc, &nameABold2, topName.c_str(), nullptr);
}

void lvglTopIndicatorsTapEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (lvglClickSuppressed()) return;
    lv_obj_t *obj = lv_event_get_target(e);
    if (!obj || !displayAwake) return;
    if (screenLockShouldGateUi()) return;

    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) return;
    lv_point_t pt = {0, 0};
    lv_indev_get_point(indev, &pt);

    lv_area_t c;
    lv_obj_get_coords(obj, &c);
    const int localX = static_cast<int>(pt.x - c.x1);
    const int localY = static_cast<int>(pt.y - c.y1);
    if (localY < 0 || localY > (UI_TOP_BAR_H + 2)) return;

    const int antennaTapMaxX = 52;
    if (localX < 0 || localX > antennaTapMaxX) return;

    if (lvglKb && !lv_obj_has_flag(lvglKb, LV_OBJ_FLAG_HIDDEN)) lvglHideKeyboard();
    lvglEnsureScreenBuilt(UI_WIFI_LIST);
    lvglOpenScreen(UI_WIFI_LIST, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglTopUnreadTapEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (lvglClickSuppressed()) return;
    if (!displayAwake) return;
    if (screenLockShouldGateUi()) return;
    if (lvglKb && !lv_obj_has_flag(lvglKb, LV_OBJ_FLAG_HIDDEN)) lvglHideKeyboard();
    lv_async_call(lvglDeferredChatOpenFirstUnreadCallback, nullptr);
}

void lvglTopSoundTapEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (lvglClickSuppressed()) return;
    if (!displayAwake) return;
    if (screenLockShouldGateUi()) return;
    if (lvglKb && !lv_obj_has_flag(lvglKb, LV_OBJ_FLAG_HIDDEN)) lvglHideKeyboard();
    lvglShowSoundPopup();
}

void lvglTopOtaTapEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (lvglClickSuppressed()) return;
    if (!displayAwake || !otaUpdateAvailable) return;
    if (screenLockShouldGateUi()) return;
    if (lvglKb && !lv_obj_has_flag(lvglKb, LV_OBJ_FLAG_HIDDEN)) lvglHideKeyboard();
    lvglEnsureScreenBuilt(UI_CONFIG_OTA);
    lvglOpenScreen(UI_CONFIG_OTA, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglCreateTopIndicators(lv_obj_t *header, bool backToHome)
{
    if (!header) return;
    (void)backToHome;
    lv_obj_t *obj = lv_obj_create(header);
    if (!obj) return;
    lv_obj_set_size(obj, lv_pct(100), UI_TOP_BAR_H - 2);
    lv_obj_align(obj, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(obj, 0, 0);
    lv_obj_set_style_pad_all(obj, 0, 0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(obj, lvglTopIndicatorsDrawEvent, LV_EVENT_DRAW_MAIN, nullptr);
    lvglRegisterTopIndicator(obj);

    lv_obj_t *wifiTap = lv_obj_create(header);
    if (!wifiTap) return;
    lv_obj_set_size(wifiTap, 56, UI_TOP_BAR_H - 2);
    lv_obj_align(wifiTap, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_bg_opa(wifiTap, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(wifiTap, 0, 0);
    lv_obj_set_style_pad_all(wifiTap, 0, 0);
    lv_obj_clear_flag(wifiTap, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(wifiTap, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(wifiTap, lvglTopIndicatorsTapEvent, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *mailTap = lv_obj_create(header);
    if (!mailTap) return;
    lv_obj_set_size(mailTap, 30, UI_TOP_BAR_H - 2);
    lv_obj_align(mailTap, LV_ALIGN_TOP_RIGHT, -56, 0);
    lv_obj_set_style_bg_opa(mailTap, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(mailTap, 0, 0);
    lv_obj_set_style_pad_all(mailTap, 0, 0);
    lv_obj_clear_flag(mailTap, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(mailTap, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(mailTap, lvglTopUnreadTapEvent, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *soundTap = lv_obj_create(header);
    if (!soundTap) return;
    lv_obj_set_size(soundTap, 30, UI_TOP_BAR_H - 2);
    lv_obj_align(soundTap, LV_ALIGN_TOP_RIGHT, -56, 0);
    lv_obj_set_style_bg_opa(soundTap, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(soundTap, 0, 0);
    lv_obj_set_style_pad_all(soundTap, 0, 0);
    lv_obj_clear_flag(soundTap, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(soundTap, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(soundTap, lvglTopSoundTapEvent, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *otaTap = lv_obj_create(header);
    if (!otaTap) return;
    lv_obj_set_size(otaTap, 30, UI_TOP_BAR_H - 2);
    lv_obj_align(otaTap, LV_ALIGN_TOP_RIGHT, -112, 0);
    lv_obj_set_style_bg_opa(otaTap, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(otaTap, 0, 0);
    lv_obj_set_style_pad_all(otaTap, 0, 0);
    lv_obj_clear_flag(otaTap, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(otaTap, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(otaTap, lvglTopOtaTapEvent, LV_EVENT_CLICKED, nullptr);

    lv_obj_align(mailTap, LV_ALIGN_TOP_RIGHT, -84, 0);
}

void lvglEnsurePersistentTopBar()
{
    if (lvglTopBarRoot) return;
    lvglTopBarRoot = lv_obj_create(lv_layer_top());
    if (!lvglTopBarRoot) return;
    lv_obj_set_size(lvglTopBarRoot, lv_pct(100), UI_TOP_BAR_H);
    lv_obj_align(lvglTopBarRoot, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(lvglTopBarRoot, lv_color_hex(0x0E141C), 0);
    lv_obj_set_style_bg_opa(lvglTopBarRoot, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(lvglTopBarRoot, 0, 0);
    lv_obj_set_style_radius(lvglTopBarRoot, 0, 0);
    lv_obj_set_style_pad_all(lvglTopBarRoot, 0, 0);
    lv_obj_clear_flag(lvglTopBarRoot, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lvglTopIndicatorCount = 0;
    lvglCreateTopIndicators(lvglTopBarRoot, false);
}

void lvglRefreshTopIndicators()
{
    if (lvglTopIndicatorCount == 0) return;
    LvglTopIndicatorState next = {};
    next.batteryPercent = batteryPercent;
    next.batteryCharging = batteryCharging;
    next.batteryPulse = batteryCharging && (((millis() / 280UL) & 0x1U) != 0U);
    next.soundMode = uiSoundMode();
    next.airplaneMode = airplaneModeEnabled;
    next.wifiConnected = wifiConnectedSafe();
    next.wifiConnecting = !next.wifiConnected && !airplaneModeEnabled && bootStaConnectInProgress;
    next.apVisible = !next.wifiConnected && !airplaneModeEnabled && apModeActive;
    next.mqttConnected = mqttCfg.enabled && mqttClient.connected();
    next.otaVisible = otaUpdateAvailable;
    const bool conversationOpen = (uiScreen == UI_CHAT) && !currentChatPeerKey.isEmpty();
    next.unreadMailVisible = chatHasUnreadMessages() && !conversationOpen;
    if (next.wifiConnected) {
        const int rssi = wifiRssiSafe();
        if (rssi >= -55) next.wifiBars = 4;
        else if (rssi >= -67) next.wifiBars = 3;
        else if (rssi >= -75) next.wifiBars = 2;
        else next.wifiBars = 1;
    }
    String topName = topBarCenterText();
    snprintf(next.topName, sizeof(next.topName), "%s", topName.c_str());

    if (lvglTopIndicatorStateValid && memcmp(&lvglLastTopIndicatorState, &next, sizeof(next)) == 0) return;
    lvglLastTopIndicatorState = next;
    lvglTopIndicatorStateValid = true;
    for (uint8_t i = 0; i < lvglTopIndicatorCount; i++) {
        if (lvglTopIndicators[i]) lv_obj_invalidate(lvglTopIndicators[i]);
    }
}

static void lvglSoundPopupSetIcon()
{
    if (!lvglSoundPopupVolumeIcon) return;
    lv_img_set_src(lvglSoundPopupVolumeIcon, uiVolumeIcon());
}

static void lvglRefreshSoundPopupUi()
{
    if (!lvglSoundPopup || lv_obj_has_flag(lvglSoundPopup, LV_OBJ_FLAG_HIDDEN)) return;
    if (lvglSoundPopupVolumeSlider && lv_slider_get_value(lvglSoundPopupVolumeSlider) != mediaVolumePercent) {
        lv_slider_set_value(lvglSoundPopupVolumeSlider, mediaVolumePercent, LV_ANIM_OFF);
    }
    if (lvglSoundPopupVolumeValueLabel) lvglLabelSetTextIfChanged(lvglSoundPopupVolumeValueLabel, String(mediaVolumePercent) + "%");
    if (lvglSoundPopupVibrationDropdown) {
        lv_dropdown_set_options(lvglSoundPopupVibrationDropdown, buildVibrationIntensityDropdownOptions().c_str());
        const uint16_t selected = static_cast<uint16_t>(vibrationIntensity);
        if (lv_dropdown_get_selected(lvglSoundPopupVibrationDropdown) != selected) {
            lv_dropdown_set_selected(lvglSoundPopupVibrationDropdown, selected);
        }
        if (vibrationEnabled) lv_obj_clear_state(lvglSoundPopupVibrationDropdown, LV_STATE_DISABLED);
        else lv_obj_add_state(lvglSoundPopupVibrationDropdown, LV_STATE_DISABLED);
    }
    if (lvglSoundPopupVibrationDisableBtn) {
        lv_obj_t *label = lv_obj_get_child(lvglSoundPopupVibrationDisableBtn, 0);
        if (label) lvglLabelSetTextIfChanged(label, vibrationEnabled ? "X" : "Off");
        lvglRegisterStyledButton(lvglSoundPopupVibrationDisableBtn, vibrationEnabled ? lv_color_hex(0x8A3A3A) : lv_color_hex(0x586674), true);
    }
    lvglSoundPopupSetIcon();
}

static void lvglHideSoundPopup()
{
    if (!lvglSoundPopup) return;
    lv_obj_add_flag(lvglSoundPopup, LV_OBJ_FLAG_HIDDEN);
}

static void lvglHideOtaPostUpdatePopup()
{
    if (lvglOtaPostUpdateModal) lv_obj_add_flag(lvglOtaPostUpdateModal, LV_OBJ_FLAG_HIDDEN);
    lvglOtaPostUpdatePopupVisible = false;
}

static void lvglApplyMsgboxModalStyle(lv_obj_t *msgbox)
{
    if (!msgbox) return;
    lv_obj_t *backdrop = lv_obj_get_parent(msgbox);
    if (backdrop && backdrop != msgbox) {
        lv_obj_set_style_bg_color(backdrop, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(backdrop, LV_OPA_80, 0);
        lv_obj_set_style_border_width(backdrop, 0, 0);
    }
    lv_obj_set_style_bg_color(msgbox, lv_color_hex(0x121820), 0);
    lv_obj_set_style_bg_opa(msgbox, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(msgbox, lv_color_hex(0x384758), 0);
    lv_obj_set_style_border_width(msgbox, 1, 0);
    lv_obj_set_style_text_color(msgbox, lv_color_hex(0xE5ECF3), 0);
}

static void lvglShowOtaPostUpdatePopup()
{
    if (!lvglReady) return;
    if (!lvglOtaPostUpdateModal) {
        lvglOtaPostUpdateModal = lv_obj_create(lv_layer_top());
        if (!lvglOtaPostUpdateModal) return;
        lv_obj_set_size(lvglOtaPostUpdateModal, DISPLAY_WIDTH, DISPLAY_HEIGHT);
        lv_obj_center(lvglOtaPostUpdateModal);
        lv_obj_set_style_bg_color(lvglOtaPostUpdateModal, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(lvglOtaPostUpdateModal, LV_OPA_90, 0);
        lv_obj_set_style_border_width(lvglOtaPostUpdateModal, 0, 0);
        lv_obj_set_style_pad_all(lvglOtaPostUpdateModal, 0, 0);
        lv_obj_add_flag(lvglOtaPostUpdateModal, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(lvglOtaPostUpdateModal, LV_OBJ_FLAG_SCROLLABLE);

        lvglOtaPostUpdateCard = lv_obj_create(lvglOtaPostUpdateModal);
        if (!lvglOtaPostUpdateCard) return;
        lv_obj_set_size(lvglOtaPostUpdateCard, DISPLAY_WIDTH - 10, DISPLAY_HEIGHT - 10);
        lv_obj_center(lvglOtaPostUpdateCard);
        lv_obj_set_style_bg_color(lvglOtaPostUpdateCard, lv_color_hex(0x101923), 0);
        lv_obj_set_style_bg_grad_color(lvglOtaPostUpdateCard, lv_color_hex(0x1A2A38), 0);
        lv_obj_set_style_bg_grad_dir(lvglOtaPostUpdateCard, LV_GRAD_DIR_VER, 0);
        lv_obj_set_style_border_color(lvglOtaPostUpdateCard, lv_color_hex(0x4E667D), 0);
        lv_obj_set_style_border_width(lvglOtaPostUpdateCard, 1, 0);
        lv_obj_set_style_radius(lvglOtaPostUpdateCard, 14, 0);
        lv_obj_set_style_pad_left(lvglOtaPostUpdateCard, 12, 0);
        lv_obj_set_style_pad_right(lvglOtaPostUpdateCard, 12, 0);
        lv_obj_set_style_pad_top(lvglOtaPostUpdateCard, 12, 0);
        lv_obj_set_style_pad_bottom(lvglOtaPostUpdateCard, 12, 0);
        lv_obj_set_style_pad_row(lvglOtaPostUpdateCard, 10, 0);
        lv_obj_set_flex_flow(lvglOtaPostUpdateCard, LV_FLEX_FLOW_COLUMN);
        lv_obj_clear_flag(lvglOtaPostUpdateCard, LV_OBJ_FLAG_SCROLLABLE);

        lvglOtaPostUpdateTitleLabel = lv_label_create(lvglOtaPostUpdateCard);
        if (lvglOtaPostUpdateTitleLabel) {
            lv_label_set_text(lvglOtaPostUpdateTitleLabel, "Update Complete");
            lv_obj_set_style_text_color(lvglOtaPostUpdateTitleLabel, lv_color_hex(0xF2F7FB), 0);
            lv_obj_set_style_text_font(lvglOtaPostUpdateTitleLabel, LV_FONT_DEFAULT, 0);
        }

        lvglOtaPostUpdateVersionLabel = lv_label_create(lvglOtaPostUpdateCard);
        if (lvglOtaPostUpdateVersionLabel) {
            lv_obj_set_width(lvglOtaPostUpdateVersionLabel, lv_pct(100));
            lv_label_set_long_mode(lvglOtaPostUpdateVersionLabel, LV_LABEL_LONG_WRAP);
            lv_obj_set_style_text_color(lvglOtaPostUpdateVersionLabel, lv_color_hex(0x9FD7FF), 0);
        }

        lvglOtaPostUpdateNotesWrap = lv_obj_create(lvglOtaPostUpdateCard);
        if (lvglOtaPostUpdateNotesWrap) {
            lv_obj_set_width(lvglOtaPostUpdateNotesWrap, lv_pct(100));
            lv_obj_set_flex_grow(lvglOtaPostUpdateNotesWrap, 1);
            lv_obj_set_style_bg_color(lvglOtaPostUpdateNotesWrap, lv_color_hex(0x0B1218), 0);
            lv_obj_set_style_bg_opa(lvglOtaPostUpdateNotesWrap, LV_OPA_COVER, 0);
            lv_obj_set_style_border_color(lvglOtaPostUpdateNotesWrap, lv_color_hex(0x314353), 0);
            lv_obj_set_style_border_width(lvglOtaPostUpdateNotesWrap, 1, 0);
            lv_obj_set_style_radius(lvglOtaPostUpdateNotesWrap, 12, 0);
            lv_obj_set_style_pad_left(lvglOtaPostUpdateNotesWrap, 10, 0);
            lv_obj_set_style_pad_right(lvglOtaPostUpdateNotesWrap, 10, 0);
            lv_obj_set_style_pad_top(lvglOtaPostUpdateNotesWrap, 10, 0);
            lv_obj_set_style_pad_bottom(lvglOtaPostUpdateNotesWrap, 10, 0);
            lv_obj_set_scroll_dir(lvglOtaPostUpdateNotesWrap, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(lvglOtaPostUpdateNotesWrap, LV_SCROLLBAR_MODE_ACTIVE);
            lv_obj_add_event_cb(lvglOtaPostUpdateNotesWrap, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglOtaPostUpdateNotesWrap, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglOtaPostUpdateNotesWrap, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);

            lvglOtaPostUpdateNotesLabel = lv_label_create(lvglOtaPostUpdateNotesWrap);
            if (lvglOtaPostUpdateNotesLabel) {
                lv_obj_set_width(lvglOtaPostUpdateNotesLabel, lv_pct(100));
                lv_label_set_long_mode(lvglOtaPostUpdateNotesLabel, LV_LABEL_LONG_WRAP);
                lv_obj_set_style_text_color(lvglOtaPostUpdateNotesLabel, lv_color_hex(0xD6E2EC), 0);
                lv_obj_set_style_text_line_space(lvglOtaPostUpdateNotesLabel, 3, 0);
            }
        }

        lvglOtaPostUpdateOkBtn = lv_btn_create(lvglOtaPostUpdateCard);
        if (lvglOtaPostUpdateOkBtn) {
            lv_obj_set_size(lvglOtaPostUpdateOkBtn, lv_pct(100), 42);
            lv_obj_set_style_radius(lvglOtaPostUpdateOkBtn, 10, 0);
            lv_obj_set_style_border_width(lvglOtaPostUpdateOkBtn, 0, 0);
            lv_obj_add_event_cb(lvglOtaPostUpdateOkBtn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
            lv_obj_add_event_cb(lvglOtaPostUpdateOkBtn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
            lv_obj_add_event_cb(lvglOtaPostUpdateOkBtn, lvglOtaPopupEvent, LV_EVENT_CLICKED, nullptr);
            lvglRegisterStyledButton(lvglOtaPostUpdateOkBtn, lv_color_hex(0x2F7A53), false);

            lvglOtaPostUpdateOkBtnLabel = lv_label_create(lvglOtaPostUpdateOkBtn);
            if (lvglOtaPostUpdateOkBtnLabel) {
                lv_label_set_text(lvglOtaPostUpdateOkBtnLabel, "OK");
                lv_obj_center(lvglOtaPostUpdateOkBtnLabel);
            }
        }
    }

    if (lvglOtaPostUpdateVersionLabel) {
        const char *version = otaPendingPopupVersion[0] ? otaPendingPopupVersion : FW_VERSION;
        lv_label_set_text_fmt(lvglOtaPostUpdateVersionLabel, "Device updated to firmware %s", version);
    }
    if (lvglOtaPostUpdateNotesLabel) {
        String notes = otaPendingPopupNotes;
        notes.trim();
        if (notes.isEmpty()) notes = "Release notes unavailable for this update.";
        lvglLabelSetTextIfChanged(lvglOtaPostUpdateNotesLabel, notes);
    }
    if (lvglOtaPostUpdateNotesWrap && lv_obj_is_valid(lvglOtaPostUpdateNotesWrap)) {
        lv_obj_scroll_to_y(lvglOtaPostUpdateNotesWrap, 0, LV_ANIM_OFF);
    }
    lv_obj_clear_flag(lvglOtaPostUpdateModal, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(lvglOtaPostUpdateModal);
    lvglOtaPostUpdatePopupVisible = true;
}

static void lvglShowSoundPopup()
{
    if (!lvglReady) return;
    if (!lvglSoundPopup) {
        lvglSoundPopup = lv_obj_create(lv_layer_top());
        if (!lvglSoundPopup) return;
        lv_obj_set_size(lvglSoundPopup, DISPLAY_WIDTH, DISPLAY_HEIGHT);
        lv_obj_center(lvglSoundPopup);
        lv_obj_set_style_bg_color(lvglSoundPopup, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(lvglSoundPopup, LV_OPA_80, 0);
        lv_obj_set_style_border_width(lvglSoundPopup, 0, 0);
        lv_obj_set_style_pad_all(lvglSoundPopup, 0, 0);
        lv_obj_add_flag(lvglSoundPopup, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(lvglSoundPopup, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(lvglSoundPopup, lvglSoundPopupBackdropEvent, LV_EVENT_CLICKED, nullptr);

        lvglSoundPopupCard = lv_obj_create(lvglSoundPopup);
        if (!lvglSoundPopupCard) return;
        lv_obj_set_size(lvglSoundPopupCard, DISPLAY_WIDTH - 26, 118);
        lv_obj_align(lvglSoundPopupCard, LV_ALIGN_TOP_MID, 0, UI_TOP_BAR_H + 10);
        lv_obj_set_style_bg_color(lvglSoundPopupCard, lv_color_hex(0x16212C), 0);
        lv_obj_set_style_border_color(lvglSoundPopupCard, lv_color_hex(0x5A6B7C), 0);
        lv_obj_set_style_border_width(lvglSoundPopupCard, 1, 0);
        lv_obj_set_style_radius(lvglSoundPopupCard, 12, 0);
        lv_obj_set_style_pad_all(lvglSoundPopupCard, 10, 0);
        lv_obj_set_style_pad_row(lvglSoundPopupCard, 10, 0);
        lv_obj_set_flex_flow(lvglSoundPopupCard, LV_FLEX_FLOW_COLUMN);
        lv_obj_clear_flag(lvglSoundPopupCard, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *volumeRow = lv_obj_create(lvglSoundPopupCard);
        lv_obj_set_size(volumeRow, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(volumeRow, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(volumeRow, 0, 0);
        lv_obj_set_style_pad_all(volumeRow, 0, 0);
        lv_obj_set_style_pad_column(volumeRow, 8, 0);
        lv_obj_set_flex_flow(volumeRow, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(volumeRow, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(volumeRow, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *volumeLabel = lv_label_create(volumeRow);
        lv_label_set_text(volumeLabel, tr(TXT_VOLUME));
        lv_obj_set_style_text_color(volumeLabel, lv_color_hex(0xE5ECF3), 0);

        lvglSoundPopupVolumeSlider = lv_slider_create(volumeRow);
        lv_obj_set_width(lvglSoundPopupVolumeSlider, 122);
        lv_slider_set_range(lvglSoundPopupVolumeSlider, 0, 100);
        lv_obj_add_event_cb(lvglSoundPopupVolumeSlider, lvglSoundPopupVolumeEvent, LV_EVENT_VALUE_CHANGED, nullptr);

        lvglSoundPopupVolumeValueLabel = lv_label_create(volumeRow);
        lv_obj_set_width(lvglSoundPopupVolumeValueLabel, 38);
        lv_obj_set_style_text_color(lvglSoundPopupVolumeValueLabel, lv_color_hex(0xB7C4D1), 0);

        lvglSoundPopupVolumeIcon = lv_img_create(volumeRow);
        lv_obj_set_size(lvglSoundPopupVolumeIcon, 24, 24);

        lv_obj_t *vibrationRow = lv_obj_create(lvglSoundPopupCard);
        lv_obj_set_size(vibrationRow, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(vibrationRow, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(vibrationRow, 0, 0);
        lv_obj_set_style_pad_all(vibrationRow, 0, 0);
        lv_obj_set_style_pad_column(vibrationRow, 8, 0);
        lv_obj_set_flex_flow(vibrationRow, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(vibrationRow, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(vibrationRow, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *vibrationLabel = lv_label_create(vibrationRow);
        lv_label_set_text(vibrationLabel, "Vibration");
        lv_obj_set_style_text_color(vibrationLabel, lv_color_hex(0xE5ECF3), 0);

        lvglSoundPopupVibrationDropdown = lv_dropdown_create(vibrationRow);
        lv_obj_set_width(lvglSoundPopupVibrationDropdown, 126);
        lv_obj_set_flex_grow(lvglSoundPopupVibrationDropdown, 1);
        lv_obj_add_event_cb(lvglSoundPopupVibrationDropdown, lvglSoundPopupVibrationEvent, LV_EVENT_VALUE_CHANGED, nullptr);
        lv_obj_add_event_cb(lvglSoundPopupVibrationDropdown, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
        lv_obj_add_event_cb(lvglSoundPopupVibrationDropdown, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
        lv_obj_add_event_cb(lvglSoundPopupVibrationDropdown, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);

        lvglSoundPopupVibrationDisableBtn = lv_btn_create(vibrationRow);
        lv_obj_set_size(lvglSoundPopupVibrationDisableBtn, 42, 34);
        lv_obj_set_style_radius(lvglSoundPopupVibrationDisableBtn, 8, 0);
        lv_obj_set_style_border_width(lvglSoundPopupVibrationDisableBtn, 0, 0);
        lv_obj_add_event_cb(lvglSoundPopupVibrationDisableBtn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(lvglSoundPopupVibrationDisableBtn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(lvglSoundPopupVibrationDisableBtn, lvglSoundPopupDisableVibrationEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_t *disableLabel = lv_label_create(lvglSoundPopupVibrationDisableBtn);
        lv_label_set_text(disableLabel, "X");
        lv_obj_center(disableLabel);
    }

    lv_obj_clear_flag(lvglSoundPopup, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(lvglSoundPopup);
    lvglRefreshSoundPopupUi();
}

lv_obj_t *lvglCreateScreenBase(const char *title, bool backToHome = false)
{
    lv_obj_t *scr = lv_obj_create(nullptr);
    if (!scr) {
        Serial.printf("[LVGL] alloc failed: screen '%s'\n", title ? title : "?");
        return nullptr;
    }
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x111922), 0);
    lv_obj_set_style_bg_grad_color(scr, lv_color_hex(0x243A52), 0);
    lv_obj_set_style_bg_grad_dir(scr, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_width(scr, 0, 0);
    lv_obj_set_style_pad_all(scr, 0, 0);
    lv_obj_add_event_cb(
        scr,
        [](lv_event_t *e) {
            if (lvglClickSuppressed()) return;
            if (lv_event_get_target(e) != lv_event_get_current_target(e)) return;
            if (lvglKb && !lv_obj_has_flag(lvglKb, LV_OBJ_FLAG_HIDDEN)) lvglHideKeyboard();
        },
        LV_EVENT_CLICKED,
        nullptr);
    (void)title;
    (void)backToHome;
    return scr;
}

static const char *uiScreenName(UiScreen screen)
{
    switch (screen) {
        case UI_LOCK: return "Screen Lock";
        case UI_CHAT: return tr(TXT_CHAT);
        case UI_CHAT_PEERS: return tr(TXT_CHAT_PEERS);
        case UI_MEDIA: return tr(TXT_MEDIA);
        case UI_INFO: return tr(TXT_INFO);
        case UI_RADIO_CONTROL: return tr(TXT_RADIO_CONTROL);
        case UI_CONFIG: return tr(TXT_CONFIG);
        case UI_CONFIG_REMOTE_CONTROL: return tr(TXT_REMOTE_CONTROL);
        case UI_CONFIG_RADIO_BUTTONS: return tr(TXT_BUTTON_CONFIG);
        case UI_CONFIG_RADIO_RECEIVER_TEST: return "Receiver Test";
        case UI_CONFIG_BATTERY: return tr(TXT_BATTERY);
        case UI_CONFIG_STYLE: return tr(TXT_SCREEN);
        case UI_CONFIG_LANGUAGE: return tr(TXT_LANGUAGE);
        case UI_CONFIG_OTA: return tr(TXT_OTA_UPDATES);
        case UI_CONFIG_MODULES: return "Modules";
        case UI_CONFIG_HC12: return tr(TXT_HC12_CONFIG);
        case UI_CONFIG_HC12_TERMINAL: return tr(TXT_HC12_TERMINAL);
        case UI_CONFIG_HC12_INFO: return tr(TXT_HC12_INFO);
        case UI_CONFIG_MQTT_CONFIG: return tr(TXT_MQTT_CONFIG);
        case UI_CONFIG_MQTT_CONTROLS: return tr(TXT_MQTT_CONTROLS);
        case UI_CONFIG_MQTT_BUTTONS: return tr(TXT_BUTTON_CONFIG);
        case UI_GAME_CHECKERS: return tr(TXT_CHECKERS);
        case UI_GAME_SNAKE3D: return tr(TXT_SNAKE_3D);
        default: return tr(TXT_SCREEN);
    }
}

static bool lvglCanBuildScreen(UiScreen screen)
{
    uint32_t freeMin = 0;
    uint32_t largestMin = 0;

    switch (screen) {
        case UI_CHAT:
        case UI_CHAT_PEERS:
        case UI_CONFIG_MQTT_CONFIG:
        case UI_CONFIG_MQTT_CONTROLS:
        case UI_CONFIG_MQTT_BUTTONS:
            freeMin = 22000U;
            largestMin = 9000U;
            break;
        case UI_MEDIA:
        case UI_INFO:
        case UI_RADIO_CONTROL:
        case UI_CONFIG:
        case UI_CONFIG_REMOTE_CONTROL:
        case UI_CONFIG_RADIO_BUTTONS:
        case UI_CONFIG_RADIO_RECEIVER_TEST:
        case UI_CONFIG_BATTERY:
        case UI_CONFIG_STYLE:
        case UI_CONFIG_LANGUAGE:
        case UI_CONFIG_OTA:
        case UI_CONFIG_MODULES:
        case UI_CONFIG_HC12:
        case UI_CONFIG_HC12_TERMINAL:
        case UI_CONFIG_HC12_INFO:
            freeMin = 18000U;
            largestMin = 8000U;
            break;
        default:
            return true;
    }

    if (boardHasUsablePsram()) {
        freeMin = max<uint32_t>(24000U, freeMin / 2U);
        largestMin = max<uint32_t>(10000U, largestMin / 2U);
    }

    const uint32_t freeHeap = static_cast<uint32_t>(ESP.getFreeHeap());
    const uint32_t largest8 = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    if (freeHeap >= freeMin && largest8 >= largestMin) return true;

    uiStatusLine = String(uiScreenName(screen)) + " unavailable: low memory";
    if (lvglReady) lvglSyncStatusLine();
    Serial.printf("[LVGL] build blocked screen=%s free=%lu largest=%lu need=%lu/%lu\n",
                  uiScreenName(screen),
                  static_cast<unsigned long>(freeHeap),
                  static_cast<unsigned long>(largest8),
                  static_cast<unsigned long>(freeMin),
                  static_cast<unsigned long>(largestMin));
    return false;
}

static void lvglWarmupScreensService(bool uiPriorityActive)
{
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    if (!lvglReady || !displayAwake || uiPriorityActive || screensaverActive || lvglKeyboardVisible()) return;
    static const UiScreen warmupScreens[] = {
        UI_CONFIG,
        UI_CONFIG_BATTERY,
        UI_CONFIG_STYLE,
        UI_CONFIG_LANGUAGE,
        UI_CONFIG_OTA,
        UI_CONFIG_MODULES,
        UI_WIFI_LIST,
        UI_INFO,
        UI_MEDIA,
        UI_GAMES,
        UI_RADIO_CONTROL,
        UI_CONFIG_REMOTE_CONTROL,
        UI_CONFIG_RADIO_BUTTONS,
        UI_CONFIG_RADIO_RECEIVER_TEST,
        UI_CONFIG_HC12,
        UI_CONFIG_HC12_TERMINAL,
        UI_CONFIG_HC12_INFO,
        UI_CONFIG_MQTT_CONFIG,
        UI_CONFIG_MQTT_CONTROLS,
        UI_CONFIG_MQTT_BUTTONS
    };
    const unsigned long now = millis();
    if (static_cast<unsigned long>(now - lastUserActivityMs) < UI_WARMUP_IDLE_AFTER_INPUT_MS) return;
    if (static_cast<unsigned long>(now - lvglWarmupLastMs) < UI_WARMUP_INTERVAL_MS) return;
    lvglWarmupLastMs = now;

    const size_t warmupCount = sizeof(warmupScreens) / sizeof(warmupScreens[0]);
    for (size_t i = 0; i < warmupCount; ++i) {
        const UiScreen screen = warmupScreens[lvglWarmupScreenIndex];
        lvglWarmupScreenIndex = static_cast<uint8_t>((lvglWarmupScreenIndex + 1U) % warmupCount);
        if (lvglScreenForUi(screen)) continue;
        if (!lvglCanBuildScreen(screen)) return;
        lvglEnsureScreenBuilt(screen);
        return;
    }
#else
    (void)uiPriorityActive;
#endif
}

static GameBoardLayout gameBoardLayout(int cols, int rows)
{
    (void)cols;
    (void)rows;
    const lv_coord_t scoreHeight = (DISPLAY_HEIGHT >= 460) ? 52 : 44;
    const lv_coord_t controlsHeight = (DISPLAY_HEIGHT >= 460) ? 122 : 108;
    const lv_coord_t buttonHeight = (DISPLAY_HEIGHT >= 460) ? 50 : 42;
    const lv_coord_t boardTopOffset = scoreHeight + 8;
    GameBoardLayout layout = {};
    layout.width = max<lv_coord_t>(96, DISPLAY_WIDTH - 10);
    layout.height = max<lv_coord_t>(96, UI_CONTENT_H - scoreHeight - controlsHeight - 14);
    layout.scoreHeight = scoreHeight;
    layout.controlsHeight = controlsHeight;
    layout.boardTopOffset = boardTopOffset;
    layout.buttonWidth = max<lv_coord_t>(92, (DISPLAY_WIDTH - 22) / 2);
    layout.buttonHeight = buttonHeight;
    return layout;
}

// UI button styling, reorder, icon-mode, and theme logic moved out of main.cpp.
#include "app/ui_buttons.inc"

static lv_obj_t *lvglCreateInfoCard(lv_obj_t *parent, const char *symbol, const char *title, lv_color_t accent, lv_obj_t **valueOut, lv_obj_t **subOut, lv_obj_t **barOut = nullptr)
{
    if (valueOut) *valueOut = nullptr;
    if (subOut) *subOut = nullptr;
    if (barOut) *barOut = nullptr;
    if (!parent) return nullptr;

    lv_obj_t *card = lv_obj_create(parent);
    if (!card) return nullptr;
    lv_obj_set_width(card, lv_pct(100));
    lv_obj_set_height(card, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x16212C), 0);
    lv_obj_set_style_border_color(card, lv_color_mix(accent, lv_color_white(), LV_OPA_20), 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_radius(card, 12, 0);
    lv_obj_set_style_pad_all(card, 10, 0);
    lv_obj_set_style_pad_row(card, 4, 0);
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *top = lv_obj_create(card);
    lv_obj_set_size(top, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(top, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(top, 0, 0);
    lv_obj_set_style_pad_all(top, 0, 0);
    lv_obj_set_style_pad_column(top, 6, 0);
    lv_obj_set_flex_flow(top, LV_FLEX_FLOW_ROW);
    lv_obj_clear_flag(top, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *icon = lv_label_create(top);
    lv_label_set_text_fmt(icon, "%s", symbol ? symbol : "");
    lv_obj_set_style_text_color(icon, accent, 0);

    lv_obj_t *name = lv_label_create(top);
    lv_label_set_text(name, title ? title : "");
    lv_obj_set_style_text_color(name, lv_color_hex(0xE5ECF3), 0);
    lv_obj_set_flex_grow(name, 1);

    lv_obj_t *value = lv_label_create(top);
    lv_label_set_text(value, "--");
    lv_obj_set_style_text_color(value, lv_color_hex(0xF5F8FB), 0);

    lv_obj_t *sub = lv_label_create(card);
    lv_obj_set_width(sub, lv_pct(100));
    lv_label_set_long_mode(sub, LV_LABEL_LONG_WRAP);
    lv_label_set_text(sub, "");
    lv_obj_set_style_text_color(sub, lv_color_hex(0x9FB0C2), 0);

    lv_obj_t *bar = nullptr;
    if (barOut) {
        bar = lv_bar_create(card);
        lv_obj_set_size(bar, lv_pct(100), 10);
        lv_bar_set_range(bar, 0, 100);
        lv_bar_set_value(bar, 0, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(bar, lv_color_hex(0x2A3340), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_bg_color(bar, accent, LV_PART_INDICATOR);
        lv_obj_set_style_radius(bar, 6, LV_PART_MAIN);
        lv_obj_set_style_radius(bar, 6, LV_PART_INDICATOR);
    }

    if (valueOut) *valueOut = value;
    if (subOut) *subOut = sub;
    if (barOut) *barOut = bar;
    return card;
}

static void lvglSetInfoBarColor(lv_obj_t *bar, lv_color_t color)
{
    if (!bar) return;
    lv_obj_set_style_bg_color(bar, color, LV_PART_INDICATOR);
}

static lv_obj_t *lvglInfoCardValueLabel(lv_obj_t *card)
{
    lv_obj_t *top = card ? lv_obj_get_child(card, 0) : nullptr;
    return top ? lv_obj_get_child(top, 2) : nullptr;
}

static lv_obj_t *lvglInfoCardSubLabel(lv_obj_t *card)
{
    return card ? lv_obj_get_child(card, 1) : nullptr;
}

static lv_obj_t *lvglInfoCardBar(lv_obj_t *card)
{
    return card ? lv_obj_get_child(card, 2) : nullptr;
}

static lv_obj_t *lvglFindInfoCardByTitle(const char *title)
{
    if (!lvglInfoList || !title) return nullptr;
    const uint32_t childCount = lv_obj_get_child_cnt(lvglInfoList);
    for (uint32_t i = 0; i < childCount; ++i) {
        lv_obj_t *card = lv_obj_get_child(lvglInfoList, i);
        if (!card) continue;
        lv_obj_t *top = lv_obj_get_child(card, 0);
        if (!top) continue;
        lv_obj_t *name = lv_obj_get_child(top, 1);
        if (!name) continue;
        const char *text = lv_label_get_text(name);
        if (text && strcmp(text, title) == 0) return card;
    }
    return nullptr;
}

static String chatEscapeLogField(String text)
{
    text.replace("\\", "\\\\");
    text.replace("\t", "\\t");
    text.replace("\r", " ");
    text.replace("\n", "\\n");
    return text;
}

static String chatUnescapeLogField(String text)
{
    text.replace("\\n", "\n");
    text.replace("\\t", "\t");
    text.replace("\\\\", "\\");
    return text;
}

static String chatSafeFileToken(const String &raw)
{
    String out;
    for (size_t i = 0; i < raw.length(); ++i) {
        const char c = raw[i];
        if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')) out += c;
        else out += '_';
    }
    if (out.isEmpty()) out = "unknown";
    return out;
}

static String chatFriendlyLogFilenameForPeer(const String &peerKey)
{
    const String safeKey = chatSafeFileToken(peerKey);
    String shortKey = safeKey;
    if (shortKey.length() > 8) shortKey = shortKey.substring(0, 8);
    String name = chatSafeFileToken(chatDisplayNameForPeerKey(peerKey));
    if (name.length() > 24) name = name.substring(0, 24);
    return name + "_" + shortKey + ".txt";
}

static String chatLegacyLogPathForPeer(const String &peerKey)
{
    return String(CHAT_LOG_DIR) + "/" + chatSafeFileToken(peerKey) + ".txt";
}

static String chatLogPathForPeer(const String &peerKey)
{
    if (peerKey.isEmpty()) return "";
    const String safeKey = chatSafeFileToken(peerKey);
    String shortKey = safeKey;
    if (shortKey.length() > 8) shortKey = shortKey.substring(0, 8);
    const String friendlyPath = String(CHAT_LOG_DIR) + "/" + chatFriendlyLogFilenameForPeer(peerKey);
    const String legacyPath = chatLegacyLogPathForPeer(peerKey);
    const String keySuffix = "_" + shortKey + ".txt";

    String resolved = friendlyPath;
    if (SD.exists(friendlyPath)) {
        resolved = friendlyPath;
    } else if (SD.exists(legacyPath)) {
        resolved = legacyPath;
    } else {
        File dir = SD.open(CHAT_LOG_DIR, FILE_READ);
        if (dir) {
            File entry = dir.openNextFile();
            while (entry) {
                if (!entry.isDirectory()) {
                    String name = String(entry.name());
                    if (name.endsWith(keySuffix)) {
                        resolved = String(CHAT_LOG_DIR) + "/" + name;
                        entry.close();
                        break;
                    }
                }
                entry.close();
                entry = dir.openNextFile();
            }
            dir.close();
        }
    }
    return resolved;
}

static bool chatPeerHasHistory(const String &peerKey)
{
    if (peerKey.isEmpty()) return false;
    if (!sdEnsureMounted()) return false;
    if (!sdLock()) return false;
    const bool exists = SD.exists(chatLogPathForPeer(peerKey));
    sdUnlock();
    return exists;
}

static String chatDisplayNameForPeerKey(const String &peerKey)
{
    if (peerKey == "ha_inbox") return "Home Assistant";
    const int idx = p2pFindPeerByPubKeyHex(peerKey);
    if (idx >= 0 && !p2pPeers[idx].name.isEmpty()) return p2pPeers[idx].name;
    const int radioIdx = hc12FindDiscoveredByPubKeyHex(peerKey);
    if (radioIdx >= 0 && !hc12DiscoveredPeers[radioIdx].name.isEmpty()) return hc12DiscoveredPeers[radioIdx].name;
    String shortKey = peerKey;
    if (shortKey.length() > 12) shortKey = shortKey.substring(0, 12) + "...";
    return shortKey;
}

static String chatGenerateMessageId()
{
    char buf[33];
    snprintf(buf, sizeof(buf), "%08lx%08lx",
             static_cast<unsigned long>(millis()),
             static_cast<unsigned long>(esp_random()));
    return String(buf);
}

static bool gameControlMessageSeen(const String &messageId)
{
    if (messageId.isEmpty()) return false;
    for (uint8_t i = 0; i < gameControlMessageIdCount; ++i) {
        if (strncmp(gameControlMessageIds[i], messageId.c_str(), sizeof(gameControlMessageIds[i])) == 0) return true;
    }
    return false;
}

static void gameRememberControlMessageId(const String &messageId)
{
    if (messageId.isEmpty() || gameControlMessageSeen(messageId)) return;
    if (gameControlMessageIdCount < MAX_GAME_CONTROL_IDS) {
        strncpy(gameControlMessageIds[gameControlMessageIdCount], messageId.c_str(), sizeof(gameControlMessageIds[gameControlMessageIdCount]) - 1);
        gameControlMessageIds[gameControlMessageIdCount][sizeof(gameControlMessageIds[gameControlMessageIdCount]) - 1] = '\0';
        gameControlMessageIdCount++;
        return;
    }
    for (uint8_t i = 1; i < MAX_GAME_CONTROL_IDS; ++i) {
        memcpy(gameControlMessageIds[i - 1], gameControlMessageIds[i], sizeof(gameControlMessageIds[i - 1]));
    }
    strncpy(gameControlMessageIds[MAX_GAME_CONTROL_IDS - 1], messageId.c_str(), sizeof(gameControlMessageIds[MAX_GAME_CONTROL_IDS - 1]) - 1);
    gameControlMessageIds[MAX_GAME_CONTROL_IDS - 1][sizeof(gameControlMessageIds[MAX_GAME_CONTROL_IDS - 1]) - 1] = '\0';
}

// Checkers chat/control text helpers moved to app/games_checkers_text.inc.
#include "app/games_checkers_text.inc"

static bool chatSavePendingOutbox()
{
    if (!sdEnsureMounted(true)) return false;
    fsWriteBegin();
    bool ok = false;
    if (sdLock()) {
        if (!SD.exists(CHAT_LOG_DIR)) SD.mkdir(CHAT_LOG_DIR);
        if (chatPendingCount == 0) {
            SD.remove(CHAT_OUTBOX_PATH);
            ok = true;
        } else {
            SD.remove(CHAT_OUTBOX_PATH);
            File f = SD.open(CHAT_OUTBOX_PATH, FILE_WRITE);
            if (f) {
                ok = true;
                for (int i = 0; i < chatPendingCount; ++i) {
                    const PendingChatMessage &msg = chatPendingMessages[i];
                    const String line = chatEscapeLogField(msg.messageId) + "\t" +
                                        chatEscapeLogField(msg.peerKey) + "\t" +
                                        chatEscapeLogField(msg.author) + "\t" +
                                        chatEscapeLogField(msg.text) + "\t" +
                                        String(msg.createdMs) + "\n";
                    if (f.print(line) <= 0) ok = false;
                }
                f.close();
            } else {
                ok = false;
            }
        }
        sdUnlock();
    }
    fsWriteEnd();
    return ok;
}

static void chatLoadPendingOutbox()
{
    chatPendingCount = 0;
    chatPendingLoaded = true;
    if (!sdEnsureMounted()) return;
    if (!sdLock()) return;
    File f = SD.open(CHAT_OUTBOX_PATH, FILE_READ);
    if (f) {
        while (f.available() && chatPendingCount < MAX_CHAT_PENDING) {
            String line = f.readStringUntil('\n');
            line.trim();
            if (line.isEmpty()) continue;
            const int p1 = line.indexOf('\t');
            const int p2 = (p1 >= 0) ? line.indexOf('\t', p1 + 1) : -1;
            const int p3 = (p2 >= 0) ? line.indexOf('\t', p2 + 1) : -1;
            const int p4 = (p3 >= 0) ? line.indexOf('\t', p3 + 1) : -1;
            if (p1 <= 0 || p2 <= p1 || p3 <= p2 || p4 <= p3) continue;
            PendingChatMessage &msg = chatPendingMessages[chatPendingCount++];
            msg.messageId = chatUnescapeLogField(line.substring(0, p1));
            msg.peerKey = chatUnescapeLogField(line.substring(p1 + 1, p2));
            msg.author = chatUnescapeLogField(line.substring(p2 + 1, p3));
            msg.text = chatUnescapeLogField(line.substring(p3 + 1, p4));
            msg.createdMs = static_cast<unsigned long>(strtoul(line.substring(p4 + 1).c_str(), nullptr, 10));
            msg.lastAttemptMs = 0;
            msg.lastRadioAttemptMs = 0;
            msg.attempts = 0;
            msg.radioAttempts = 0;
        }
        f.close();
    }
    sdUnlock();
}

static int chatFindPendingIndex(const String &peerKey, const String &messageId)
{
    for (int i = 0; i < chatPendingCount; ++i) {
        if (chatPendingMessages[i].peerKey == peerKey && chatPendingMessages[i].messageId == messageId) return i;
    }
    return -1;
}

static void chatRememberRecentMessageId(const String &peerKey, const String &messageId)
{
    if (peerKey.isEmpty() || messageId.isEmpty()) return;
    for (int i = 0; i < CHAT_RECENT_MESSAGE_ID_CACHE; ++i) {
        if (chatRecentMessageIds[i].peerKey == peerKey &&
            chatRecentMessageIds[i].messageId == messageId) {
            return;
        }
    }
    chatRecentMessageIds[chatRecentMessageIdHead].peerKey = peerKey;
    chatRecentMessageIds[chatRecentMessageIdHead].messageId = messageId;
    chatRecentMessageIdHead = static_cast<uint8_t>((chatRecentMessageIdHead + 1U) % CHAT_RECENT_MESSAGE_ID_CACHE);
}

static bool chatRecentMessageIdSeen(const String &peerKey, const String &messageId)
{
    if (peerKey.isEmpty() || messageId.isEmpty()) return false;
    for (int i = 0; i < CHAT_RECENT_MESSAGE_ID_CACHE; ++i) {
        if (chatRecentMessageIds[i].peerKey == peerKey &&
            chatRecentMessageIds[i].messageId == messageId) {
            return true;
        }
    }
    return false;
}

static void chatSchedulePendingOutboxSave(unsigned long delayMs)
{
    chatPendingOutboxDirty = true;
    const unsigned long due = millis() + delayMs;
    if (chatPendingOutboxFlushAfterMs == 0 ||
        static_cast<long>(due - chatPendingOutboxFlushAfterMs) < 0) {
        chatPendingOutboxFlushAfterMs = due;
    }
}

static void chatPendingOutboxService()
{
    if (!chatPendingOutboxDirty) return;
    if (fsWriteBusy()) return;
    if (chatPendingOutboxFlushAfterMs != 0 &&
        static_cast<long>(millis() - chatPendingOutboxFlushAfterMs) < 0) {
        return;
    }
    if (chatSavePendingOutbox()) {
        chatPendingOutboxDirty = false;
        chatPendingOutboxFlushAfterMs = 0;
    } else {
        chatPendingOutboxFlushAfterMs = millis() + 1000UL;
    }
}

static bool chatPeerCanUseP2pTransport(const String &peerKey)
{
    if (!p2pReady || peerKey.isEmpty()) return false;
    const int peerIdx = p2pFindPeerByPubKeyHex(peerKey);
    if (peerIdx < 0 || !p2pPeers[peerIdx].enabled) return false;
    return p2pPeers[peerIdx].ip != IPAddress((uint32_t)0);
}

static bool chatPeerCanUseMqttTransport(const String &peerKey)
{
    if (peerKey.isEmpty()) return false;
    if (!mqttCfg.enabled || !mqttClient.connected()) return false;
    const int peerIdx = p2pFindPeerByPubKeyHex(peerKey);
    return peerIdx >= 0 && p2pPeers[peerIdx].enabled;
}

static bool chatPeerCanUseRadioTransport(const String &peerKey)
{
    if (peerKey.isEmpty() || !radioModuleCanCarryChat()) return false;
    if (hc12SetIsAsserted()) return false;
    const int peerIdx = p2pFindPeerByPubKeyHex(peerKey);
    return peerIdx >= 0 && p2pPeers[peerIdx].enabled;
}

static bool chatPeerUsesRadioPath(const String &peerKey)
{
    if (peerKey.isEmpty()) return false;
    if (chatPeerCanUseRadioTransport(peerKey)) return true;
    return hc12FindDiscoveredByPubKeyHex(peerKey) >= 0;
}

static bool chatPeerBlockedByAirplaneMode(const String &peerKey)
{
    if (peerKey == "ha_inbox") return false;
    if (!airplaneModeEnabled) return false;
    return !chatPeerUsesRadioPath(peerKey);
}

static void chatRemovePendingForPeer(const String &peerKey)
{
    if (peerKey.isEmpty()) return;

    bool changed = false;
    for (int i = 0; i < chatPendingCount; ) {
        if (chatPendingMessages[i].peerKey == peerKey) {
            for (int j = i + 1; j < chatPendingCount; ++j) {
                chatPendingMessages[j - 1] = chatPendingMessages[j];
            }
            chatPendingCount--;
            changed = true;
        } else {
            ++i;
        }
    }

    if (changed) {
        chatSchedulePendingOutboxSave();
    }
}

static bool chatConversationHasMessageId(const String &peerKey, const String &messageId)
{
    if (peerKey.isEmpty() || messageId.isEmpty()) return false;
    if (currentChatPeerKey != peerKey) return false;
    for (int i = 0; i < chatMessageCount; ++i) {
        if (chatMessages[i].messageId == messageId) return true;
    }
    return false;
}

static bool chatHasLoggedMessageId(const String &peerKey, const String &messageId)
{
    if (peerKey.isEmpty() || messageId.isEmpty()) return false;
    if (chatRecentMessageIdSeen(peerKey, messageId)) return true;
    if (!sdEnsureMounted()) return false;
    if (!sdLock()) return false;
    bool found = false;
    File f = SD.open(chatLogPathForPeer(peerKey), FILE_READ);
    if (f) {
        while (f.available() && !found) {
            String line = f.readStringUntil('\n');
            line.trim();
            if (line.isEmpty()) continue;
            const int p1 = line.indexOf('\t');
            const int p2 = (p1 >= 0) ? line.indexOf('\t', p1 + 1) : -1;
            const int p3 = (p2 >= 0) ? line.indexOf('\t', p2 + 1) : -1;
            const int p4 = (p3 >= 0) ? line.indexOf('\t', p3 + 1) : -1;
            const int p5 = (p4 >= 0) ? line.indexOf('\t', p4 + 1) : -1;
            if (p5 > p4) {
                const String loggedId = chatUnescapeLogField(line.substring(p5 + 1));
                if (loggedId == messageId) found = true;
            }
        }
        f.close();
    }
    sdUnlock();
    return found;
}

static void chatClearCache()
{
    for (int i = 0; i < chatMessageCount; ++i) {
        chatMessages[i].author = "";
        chatMessages[i].text = "";
        chatMessages[i].messageId = "";
    }
    chatMessageCount = 0;
}

static void chatAddMessage(const String &author, const String &text, bool outgoing, ChatTransport transport, const String &messageId = "")
{
    if (text.isEmpty()) return;
    if (chatMessageCount >= MAX_CHAT_MESSAGES) {
        for (int i = 1; i < chatMessageCount; ++i) chatMessages[i - 1] = chatMessages[i];
        chatMessageCount = MAX_CHAT_MESSAGES - 1;
    }
    chatMessages[chatMessageCount].author = author;
    chatMessages[chatMessageCount].text = text;
    chatMessages[chatMessageCount].messageId = messageId;
    chatMessages[chatMessageCount].outgoing = outgoing;
    chatMessages[chatMessageCount].failed = false;
    chatMessages[chatMessageCount].tsMs = millis();
    chatMessages[chatMessageCount].transport = transport;
    chatMessageCount++;
}

static bool chatAppendMessageToSd(const String &peerKey, ChatTransport transport, const String &author, const String &text, bool outgoing, unsigned long tsMs, const String &messageId = "")
{
    if (peerKey.isEmpty()) return false;
    if (!sdEnsureMounted(true)) return false;
    fsWriteBegin();
    bool ok = false;
    if (sdLock()) {
        if (!SD.exists(CHAT_LOG_DIR)) SD.mkdir(CHAT_LOG_DIR);
        String path = chatLogPathForPeer(peerKey);
        const String friendlyPath = String(CHAT_LOG_DIR) + "/" + chatFriendlyLogFilenameForPeer(peerKey);
        if (path != friendlyPath && !SD.exists(friendlyPath) && SD.exists(path)) {
            if (SD.rename(path, friendlyPath)) path = friendlyPath;
        }
        File f = SD.open(path, FILE_APPEND);
        if (f) {
            const String line = String(tsMs) + "\t" +
                                (outgoing ? "out" : "in") + "\t" +
                                String(static_cast<int>(transport)) + "\t" +
                                chatEscapeLogField(author) + "\t" +
                                chatEscapeLogField(text) + "\t" +
                                chatEscapeLogField(messageId) + "\n";
            ok = f.print(line) > 0;
            f.close();
        }
        sdUnlock();
    }
    fsWriteEnd();
    return ok;
}

static void chatStoreMessage(const String &peerKey, const String &author, const String &text, bool outgoing, ChatTransport transport, const String &messageId = "")
{
    const unsigned long nowMs = millis();
    if (!messageId.isEmpty() && !outgoing) {
        if (chatConversationHasMessageId(peerKey, messageId)) return;
        if (chatHasLoggedMessageId(peerKey, messageId)) return;
    }
    const bool stored = chatAppendMessageToSd(peerKey, transport, author, text, outgoing, nowMs, messageId);
    if (!outgoing && (stored || !messageId.isEmpty())) {
        chatQueueIncomingMessageBeep();
        chatQueueIncomingMessageVibration();
    }
    if (stored && !messageId.isEmpty()) chatRememberRecentMessageId(peerKey, messageId);
    const bool conversationOpen = (uiScreen == UI_CHAT) && (currentChatPeerKey == peerKey);
    if (conversationOpen && (stored || !messageId.isEmpty())) {
        chatSetPeerUnread(peerKey, false);
        chatAddMessage(author, text, outgoing, transport, messageId);
    } else if (!outgoing) {
        chatSetPeerUnread(peerKey, true);
    }
}

static bool chatQueueOutgoingMessage(const String &peerKey, const String &author, const String &text, const String &messageId)
{
    if (peerKey.isEmpty() || text.isEmpty() || messageId.isEmpty()) return false;
    const int existing = chatFindPendingIndex(peerKey, messageId);
    if (existing >= 0) return true;
    if (chatPendingCount >= MAX_CHAT_PENDING) return false;
    PendingChatMessage &msg = chatPendingMessages[chatPendingCount++];
    msg.peerKey = peerKey;
    msg.author = author;
    msg.text = text;
    msg.messageId = messageId;
    msg.createdMs = millis();
    msg.lastAttemptMs = 0;
    msg.lastRadioAttemptMs = 0;
    msg.attempts = 0;
    msg.radioAttempts = 0;
    return chatSavePendingOutbox();
}

static bool chatMessagePendingForPeer(const String &peerKey, const String &messageId)
{
    if (peerKey.isEmpty() || messageId.isEmpty()) return false;
    return chatFindPendingIndex(peerKey, messageId) >= 0;
}

static bool chatDeleteMessageAt(const String &peerKey, int index)
{
    if (peerKey.isEmpty() || index < 0 || index >= chatMessageCount) return false;
    ChatMessage target = chatMessages[index];

    if (!target.messageId.isEmpty()) {
        p2pSendMessageDelete(peerKey, target.messageId);
        mqttPublishMessageDelete(peerKey, target.messageId);
        hc12SendMessageDelete(peerKey, target.messageId);
        return chatDeleteMessageById(peerKey, target.messageId, "Message deleted");
    }

    bool ok = true;
    if (!sdEnsureMounted(true)) ok = false;
    if (ok) {
    fsWriteBegin();
        if (sdLock()) {
            String path = chatLogPathForPeer(peerKey);
            File src = SD.open(path, FILE_READ);
            if (src) {
                const String tmpPath = path + ".tmp";
                SD.remove(tmpPath);
                File dst = SD.open(tmpPath, FILE_WRITE);
                if (dst) {
                    bool removed = false;
                    while (src.available()) {
                        String line = src.readStringUntil('\n');
                        line.trim();
                        if (line.isEmpty()) continue;
                        const int p1 = line.indexOf('\t');
                        const int p2 = (p1 >= 0) ? line.indexOf('\t', p1 + 1) : -1;
                        const int p3 = (p2 >= 0) ? line.indexOf('\t', p2 + 1) : -1;
                        const int p4 = (p3 >= 0) ? line.indexOf('\t', p3 + 1) : -1;
                        const int p5 = (p4 >= 0) ? line.indexOf('\t', p4 + 1) : -1;
                        if (p1 <= 0 || p2 <= p1 || p3 <= p2 || p4 <= p3) continue;

                        const unsigned long tsMs = static_cast<unsigned long>(strtoul(line.substring(0, p1).c_str(), nullptr, 10));
                        const bool outgoing = line.substring(p1 + 1, p2) == "out";
                        const String author = chatUnescapeLogField(line.substring(p3 + 1, p4));
                        const String text = chatUnescapeLogField((p5 > p4) ? line.substring(p4 + 1, p5) : line.substring(p4 + 1));
                        const String messageId = (p5 > p4) ? chatUnescapeLogField(line.substring(p5 + 1)) : String("");

                        const bool idMatch = !target.messageId.isEmpty() && (messageId == target.messageId);
                        const bool fallbackMatch = target.messageId.isEmpty() &&
                                                   (tsMs == target.tsMs) &&
                                                   (outgoing == target.outgoing) &&
                                                   (author == target.author) &&
                                                   (text == target.text);
                        if (!removed && (idMatch || fallbackMatch)) {
                            removed = true;
                            continue;
                        }

                        if (dst.print(line + "\n") <= 0) ok = false;
                    }
                    dst.close();
                    src.close();
                    SD.remove(path);
                    if (!(ok && SD.rename(tmpPath, path))) {
                        SD.remove(tmpPath);
                        ok = false;
                    }
                } else {
                    src.close();
                    ok = false;
                }
            }
            sdUnlock();
        }
        fsWriteEnd();
    }

    for (int i = index + 1; i < chatMessageCount; ++i) chatMessages[i - 1] = chatMessages[i];
    chatMessageCount--;
    if (chatMessageCount >= 0 && chatMessageCount < MAX_CHAT_MESSAGES) {
        chatMessages[chatMessageCount].author = "";
        chatMessages[chatMessageCount].text = "";
        chatMessages[chatMessageCount].messageId = "";
    }
    if (lvglReady && uiScreen == UI_CHAT) lvglRefreshChatUi();
    uiStatusLine = "Message deleted";
    if (lvglReady) lvglSyncStatusLine();
    return true;
}

static bool chatDeleteMessageById(const String &peerKey, const String &messageId, const String &status)
{
    if (peerKey.isEmpty() || messageId.isEmpty()) return false;

    const int pendingIdx = chatFindPendingIndex(peerKey, messageId);
    if (pendingIdx >= 0) {
        for (int i = pendingIdx + 1; i < chatPendingCount; ++i) chatPendingMessages[i - 1] = chatPendingMessages[i];
        chatPendingCount--;
        chatSavePendingOutbox();
    }

    bool ok = true;
    if (!sdEnsureMounted(true)) ok = false;
    if (ok) {
    fsWriteBegin();
        if (sdLock()) {
            String path = chatLogPathForPeer(peerKey);
            File src = SD.open(path, FILE_READ);
            if (src) {
                const String tmpPath = path + ".tmp";
                SD.remove(tmpPath);
                File dst = SD.open(tmpPath, FILE_WRITE);
                if (dst) {
                    bool removed = false;
                    while (src.available()) {
                        String line = src.readStringUntil('\n');
                        line.trim();
                        if (line.isEmpty()) continue;
                        const int p1 = line.indexOf('\t');
                        const int p2 = (p1 >= 0) ? line.indexOf('\t', p1 + 1) : -1;
                        const int p3 = (p2 >= 0) ? line.indexOf('\t', p2 + 1) : -1;
                        const int p4 = (p3 >= 0) ? line.indexOf('\t', p3 + 1) : -1;
                        const int p5 = (p4 >= 0) ? line.indexOf('\t', p4 + 1) : -1;
                        if (p1 <= 0 || p2 <= p1 || p3 <= p2 || p4 <= p3) continue;
                        const String loggedId = (p5 > p4) ? chatUnescapeLogField(line.substring(p5 + 1)) : String("");
                        if (!removed && loggedId == messageId) {
                            removed = true;
                            continue;
                        }
                        if (removed && loggedId == messageId) continue;
                        if (dst.print(line + "\n") <= 0) ok = false;
                    }
                    dst.close();
                    src.close();
                    SD.remove(path);
                    if (!(ok && SD.rename(tmpPath, path))) {
                        SD.remove(tmpPath);
                        ok = false;
                    }
                } else {
                    src.close();
                    ok = false;
                }
            }
            sdUnlock();
        }
        fsWriteEnd();
    }

    for (int i = 0; i < chatMessageCount;) {
        if (chatMessages[i].messageId == messageId) {
            for (int j = i + 1; j < chatMessageCount; ++j) chatMessages[j - 1] = chatMessages[j];
            chatMessageCount--;
            if (chatMessageCount >= 0 && chatMessageCount < MAX_CHAT_MESSAGES) {
                chatMessages[chatMessageCount].author = "";
                chatMessages[chatMessageCount].text = "";
                chatMessages[chatMessageCount].messageId = "";
            }
            continue;
        }
        ++i;
    }
    if (lvglReady && uiScreen == UI_CHAT && currentChatPeerKey == peerKey) lvglRefreshChatUi();
    uiStatusLine = status;
    if (lvglReady) lvglSyncStatusLine();
    return true;
}

static bool chatSendRawReliableMessage(const String &peerKey, const String &text, bool storeVisible)
{
    if (peerKey.isEmpty() || text.isEmpty()) return false;
    if (chatPeerBlockedByAirplaneMode(peerKey)) return false;
    if (peerKey == "ha_inbox") {
        const String messageId = "dev_" + chatGenerateMessageId();
        const bool published = mqttClient.connected() ? mqttClient.publish(mqttHaChatReceivedTopic().c_str(), text.c_str(), true) : false;
        if (storeVisible) {
            chatStoreMessage(peerKey, "Me", text, true, CHAT_TRANSPORT_MQTT, messageId);
            if (lvglReady) {
                if (uiScreen == UI_CHAT) lvglRefreshChatUi();
                lvglRefreshChatContactsUi();
            }
        }
        return published || storeVisible;
    }
    const String messageId = chatGenerateMessageId();
    chatQueueOutgoingMessage(peerKey, deviceShortNameValue(), text, messageId);
    const bool sentLan = p2pSendChatMessageWithId(peerKey, text, messageId);
    const bool sentGlobal = mqttPublishChatMessageWithId(peerKey, text, messageId);
    const bool queuedRadio = chatPeerCanUseRadioTransport(peerKey);
    if (storeVisible) {
        const ChatTransport storedTransport = sentGlobal ? CHAT_TRANSPORT_MQTT
                                                         : (sentLan ? CHAT_TRANSPORT_WIFI
                                                                    : (queuedRadio ? radioSelectedChatTransport() : CHAT_TRANSPORT_WIFI));
        chatStoreMessage(peerKey, "Me", text, true, storedTransport, messageId);
        if (lvglReady) {
            if (uiScreen == UI_CHAT) lvglRefreshChatUi();
            lvglRefreshChatContactsUi();
        }
    }
    return sentLan || sentGlobal || queuedRadio || storeVisible;
}

static bool chatSendAndStoreMessage(const String &peerKey, const String &text)
{
    const bool ok = chatSendRawReliableMessage(peerKey, text, true);
    uiStatusLine = ok ? "Chat queued" : "Chat send failed";
    if (lvglReady) lvglSyncStatusLine();
    return ok;
}

static void chatStageDeferredAirplaneMessage(const String &peerKey, const String &text)
{
    if (peerKey.isEmpty() || text.isEmpty()) return;
    chatDeferredAirplanePeerKey = peerKey;
    chatDeferredAirplaneText = text;
}

static void chatFlushDeferredAirplaneMessage()
{
    if (airplaneModeEnabled) return;
    if (chatDeferredAirplanePeerKey.isEmpty() || chatDeferredAirplaneText.isEmpty()) return;
    const String peerKey = chatDeferredAirplanePeerKey;
    const String text = chatDeferredAirplaneText;
    chatDeferredAirplanePeerKey = "";
    chatDeferredAirplaneText = "";
    if (!chatSendAndStoreMessage(peerKey, text)) {
        chatDeferredAirplanePeerKey = peerKey;
        chatDeferredAirplaneText = text;
        return;
    }
    if (lvglChatInputTa && currentChatPeerKey == peerKey) {
        String currentText = lv_textarea_get_text(lvglChatInputTa);
        currentText.trim();
        if (currentText == text) lv_textarea_set_text(lvglChatInputTa, "");
    }
}

static void chatMarkOutgoingFailed(const String &peerKey, const String &messageId)
{
    if (peerKey.isEmpty() || messageId.isEmpty()) return;
    if (currentChatPeerKey != peerKey) return;

    for (int i = 0; i < chatMessageCount; ++i) {
        if (chatMessages[i].outgoing && chatMessages[i].messageId == messageId) {
            chatMessages[i].failed = true;
            break;
        }
    }
}

static bool radioTxBusy()
{
    return radioTxState.waitingAck;
}

static void radioTxClear()
{
    radioTxState.waitingAck = false;
    radioTxState.peerKey = "";
    radioTxState.messageId = "";
    radioTxState.sentAtMs = 0;
    radioTxState.guardUntilMs = 0;
    radioTxState.retries = 0;
}

static PendingChatMessage *chatFindPendingMessage(const String &peerKey, const String &messageId)
{
    for (int i = 0; i < chatPendingCount; ++i) {
        if (chatPendingMessages[i].peerKey == peerKey &&
            chatPendingMessages[i].messageId == messageId) {
            return &chatPendingMessages[i];
        }
    }
    return nullptr;
}

static PendingChatMessage *chatFindFirstPendingRadioMessage()
{
    for (int i = 0; i < chatPendingCount; ++i) {
        PendingChatMessage &msg = chatPendingMessages[i];
        if (msg.peerKey.isEmpty() || msg.messageId.isEmpty() || msg.text.isEmpty()) continue;

        const int peerIdx = p2pFindPeerByPubKeyHex(msg.peerKey);
        if (peerIdx < 0 || !p2pPeers[peerIdx].enabled) continue;

        return &msg;
    }
    return nullptr;
}

static bool chatAckOutgoingMessage(const String &peerKey, const String &messageId)
{
    const int idx = chatFindPendingIndex(peerKey, messageId);
    if (idx < 0) return false;

    if (radioTxState.waitingAck &&
        radioTxState.peerKey == peerKey &&
        radioTxState.messageId == messageId) {
        radioTxClear();
    }

    for (int i = idx + 1; i < chatPendingCount; ++i) {
        chatPendingMessages[i - 1] = chatPendingMessages[i];
    }
    chatPendingCount--;
    chatSchedulePendingOutboxSave();

    if (currentChatPeerKey == peerKey) {
        for (int i = 0; i < chatMessageCount; ++i) {
            if (chatMessages[i].outgoing && chatMessages[i].messageId == messageId) {
                chatMessages[i].failed = false;
                break;
            }
        }
    }

    if (lvglReady && uiScreen == UI_CHAT && currentChatPeerKey == peerKey) {
        lvglRefreshChatUi();
    }
    return true;
}

static void chatPendingService()
{
    if (!chatPendingLoaded) {
        if (sdMounted || sdEnsureMounted()) chatLoadPendingOutbox();
        else return;
    }

    const unsigned long now = millis();
    for (int i = 0; i < chatPendingCount; ) {
        PendingChatMessage &msg = chatPendingMessages[i];

        if (msg.peerKey.isEmpty() || msg.messageId.isEmpty() || msg.text.isEmpty()) {
            ++i;
            continue;
        }

        const int peerIdx = p2pFindPeerByPubKeyHex(msg.peerKey);
        if (peerIdx < 0 || !p2pPeers[peerIdx].enabled) {
            chatMarkOutgoingFailed(msg.peerKey, msg.messageId);
            for (int j = i + 1; j < chatPendingCount; ++j) chatPendingMessages[j - 1] = chatPendingMessages[j];
            chatPendingCount--;
            chatSchedulePendingOutboxSave();
            continue;
        }

        if (msg.lastAttemptMs != 0 &&
            static_cast<unsigned long>(now - msg.lastAttemptMs) < CHAT_RETRY_INTERVAL_MS) {
            ++i;
            continue;
        }

        bool attempted = false;
        if (p2pSendChatMessageWithId(msg.peerKey, msg.text, msg.messageId)) attempted = true;
        if (mqttPublishChatMessageWithId(msg.peerKey, msg.text, msg.messageId)) attempted = true;

        // DO NOT send radio here anymore.
        // Radio is now handled only by radioChatService().

        msg.lastAttemptMs = now;
        if (attempted && msg.attempts < 255) msg.attempts++;

        ++i;
    }
}

static bool hc12SetIsAsserted()
{
    if (!hc12TerminalLog) return false;
    if (radioModuleType == RADIO_MODULE_HC12) return digitalRead(hc12ActiveSetPin()) == LOW;
    return digitalRead(e220ActiveM0Pin()) == HIGH && digitalRead(e220ActiveM1Pin()) == HIGH;
}

static void radioChatService()
{
    if (!chatPendingLoaded) {
        if (sdMounted || sdEnsureMounted()) chatLoadPendingOutbox();
        else return;
    }

    if (!radioModuleCanCarryChat()) return;
    if (hc12SetIsAsserted()) return;

    const unsigned long now = millis();

    // 1) Waiting for ACK -> only retry current one
    if (radioTxState.waitingAck) {
        if ((long)(now - radioTxState.guardUntilMs) < 0) return;

        PendingChatMessage *msg = chatFindPendingMessage(radioTxState.peerKey, radioTxState.messageId);
        if (!msg) {
            radioTxClear();
            return;
        }

        if ((unsigned long)(now - radioTxState.sentAtMs) < RADIO_ACK_TIMEOUT_MS) return;

        if (radioTxState.retries >= RADIO_MAX_RETRIES) {
            if (!chatPeerCanUseP2pTransport(msg->peerKey) && !chatPeerCanUseMqttTransport(msg->peerKey)) {
                chatMarkOutgoingFailed(msg->peerKey, msg->messageId);
                const int idx = chatFindPendingIndex(msg->peerKey, msg->messageId);
                if (idx >= 0) {
                    for (int j = idx + 1; j < chatPendingCount; ++j) {
                        chatPendingMessages[j - 1] = chatPendingMessages[j];
                    }
                    chatPendingCount--;
                    chatSchedulePendingOutboxSave();
                }
            }
            radioTxClear();
            if (lvglReady && uiScreen == UI_CHAT) lvglRefreshChatUi();
            return;
        }

        if (msg->lastRadioAttemptMs != 0 &&
            static_cast<unsigned long>(now - msg->lastRadioAttemptMs) < CHAT_RETRY_INTERVAL_MS) {
            return;
        }

        if (hc12SendChatMessageWithId(msg->peerKey, msg->text, msg->messageId)) {
            const unsigned long txLeadMs = radioLastTxLeadMs;
            msg->lastRadioAttemptMs = now;
            if (msg->radioAttempts < 255) msg->radioAttempts++;
            radioTxState.sentAtMs = now + txLeadMs;
            radioTxState.guardUntilMs = now + txLeadMs + RADIO_POST_TX_GUARD_MS;
            radioTxState.retries++;
        }

        return;
    }

    // 2) No in-flight message -> send one only
    PendingChatMessage *msg = chatFindFirstPendingRadioMessage();
    if (!msg) return;

    if (msg->radioAttempts >= RADIO_MAX_RETRIES) {
        if (!chatPeerCanUseP2pTransport(msg->peerKey) && !chatPeerCanUseMqttTransport(msg->peerKey)) {
            chatMarkOutgoingFailed(msg->peerKey, msg->messageId);
            const int idx = chatFindPendingIndex(msg->peerKey, msg->messageId);
            if (idx >= 0) {
                for (int j = idx + 1; j < chatPendingCount; ++j) {
                    chatPendingMessages[j - 1] = chatPendingMessages[j];
                }
                chatPendingCount--;
                chatSchedulePendingOutboxSave();
            }
        }
        return;
    }

    if (hc12SendChatMessageWithId(msg->peerKey, msg->text, msg->messageId)) {
        const unsigned long txLeadMs = radioLastTxLeadMs;
        msg->lastRadioAttemptMs = now;
        if (msg->radioAttempts < 255) msg->radioAttempts++;

        radioTxState.waitingAck = true;
        radioTxState.peerKey = msg->peerKey;
        radioTxState.messageId = msg->messageId;
        radioTxState.sentAtMs = now + txLeadMs;
        radioTxState.guardUntilMs = now + txLeadMs + RADIO_POST_TX_GUARD_MS;
        radioTxState.retries = 0;
    }
}

static void chatSelectFirstEnabledPeer()
{
    currentChatPeerKey = "";
    if (chatPeerHasHistory("ha_inbox")) {
        currentChatPeerKey = "ha_inbox";
        return;
    }
    for (int i = 0; i < p2pPeerCount; ++i) {
        if (p2pPeers[i].enabled && chatPeerHasHistory(p2pPeers[i].pubKeyHex)) {
            currentChatPeerKey = p2pPeers[i].pubKeyHex;
            return;
        }
    }
    for (int i = 0; i < p2pPeerCount; ++i) {
        if (p2pPeers[i].enabled) {
            currentChatPeerKey = p2pPeers[i].pubKeyHex;
            break;
        }
    }
    if (!currentChatPeerKey.isEmpty()) return;
    for (int i = 0; i < hc12DiscoveredCount; ++i) {
        if (chatPeerHasHistory(hc12DiscoveredPeers[i].pubKeyHex)) {
            currentChatPeerKey = hc12DiscoveredPeers[i].pubKeyHex;
            return;
        }
    }
    if (hc12DiscoveredCount > 0) currentChatPeerKey = hc12DiscoveredPeers[0].pubKeyHex;
}

static bool chatOpenPeerConversation(const String &peerKey)
{
    if (peerKey.isEmpty()) return false;
    if (!chatPeerIsSelectable(peerKey)) return false;

    lvglHideChatMenu();
    currentChatPeerKey = peerKey;
    chatSetPeerUnread(currentChatPeerKey, false);
    chatReloadRecentMessagesFromSd(currentChatPeerKey);

    if (chatPeerBlockedByAirplaneMode(peerKey)) {
        lvglShowChatAirplanePrompt();
        return true;
    }

    lvglEnsureScreenBuilt(UI_CHAT);
    lvglRefreshChatLayout();
    lvglRefreshChatContactsUi();
    lvglRefreshChatUi();
    lvglOpenScreen(UI_CHAT, LV_SCR_LOAD_ANIM_MOVE_LEFT);
    lvglRefreshTopIndicators();
    return true;
}

static void lvglDeferredChatOpenPeerCallback(void *param)
{
    String *peerKey = static_cast<String *>(param);
    if (peerKey && !peerKey->isEmpty()) chatOpenPeerConversation(*peerKey);
    delete peerKey;
}

static void chatScheduleOpenPeerConversation(const String &peerKey)
{
    if (peerKey.isEmpty()) return;
    String *deferredKey = new String(peerKey);
    if (!deferredKey) return;
    lv_async_call(lvglDeferredChatOpenPeerCallback, deferredKey);
}

static void lvglDeferredChatOpenFirstUnreadCallback(void *param)
{
    (void)param;
    chatOpenFirstUnreadConversation();
}

static bool chatOpenFirstUnreadConversation()
{
    if (haChatUnread) return chatOpenPeerConversation("ha_inbox");
    for (int i = 0; i < p2pPeerCount; ++i) {
        if (p2pPeers[i].enabled && p2pPeers[i].unread) {
            return chatOpenPeerConversation(p2pPeers[i].pubKeyHex);
        }
    }
    for (int i = 0; i < hc12DiscoveredCount; ++i) {
        if (hc12DiscoveredPeers[i].unread) return chatOpenPeerConversation(hc12DiscoveredPeers[i].pubKeyHex);
    }
    return false;
}

static bool lvglKeyboardLocalizedLayoutAvailable(UiLanguage lang)
{
    switch (lang) {
        case UI_LANG_RUSSIAN:
        case UI_LANG_FRENCH:
        case UI_LANG_TURKISH:
        case UI_LANG_ITALIAN:
        case UI_LANG_GERMAN:
            return true;
        default:
            return false;
    }
}

static lv_obj_t *lvglKeyboardFindScrollableAncestor(lv_obj_t *obj)
{
    for (lv_obj_t *cur = obj; cur && lv_obj_is_valid(cur); cur = lv_obj_get_parent(cur)) {
        if (cur == lv_layer_top() || cur == lv_layer_sys()) continue;
        if (!lv_obj_has_flag(cur, LV_OBJ_FLAG_SCROLLABLE)) continue;
        if ((lv_obj_get_scroll_dir(cur) & LV_DIR_VER) == 0) continue;
        return cur;
    }
    return nullptr;
}

static bool lvglObjHasAncestor(lv_obj_t *obj, lv_obj_t *ancestor)
{
    if (!obj || !ancestor || !lv_obj_is_valid(obj) || !lv_obj_is_valid(ancestor)) return false;
    for (lv_obj_t *cur = obj; cur && lv_obj_is_valid(cur); cur = lv_obj_get_parent(cur)) {
        if (cur == ancestor) return true;
    }
    return false;
}

static void lvglKeyboardReleasePaddedContainer()
{
    if (lvglKeyboardPaddedContainer && lv_obj_is_valid(lvglKeyboardPaddedContainer)) {
        lv_obj_set_style_pad_bottom(lvglKeyboardPaddedContainer, lvglKeyboardPaddedContainerPrevBottom, 0);
    }
    lvglKeyboardPaddedContainer = nullptr;
    lvglKeyboardPaddedContainerPrevBottom = 0;
}

static void lvglPlaceKeyboardBottom(lv_coord_t height)
{
    if (!lvglKb || !lv_obj_is_valid(lvglKb)) return;
    lv_obj_t *topLayer = lv_layer_top();
    if (topLayer && lv_obj_get_parent(lvglKb) != topLayer) {
        lv_obj_set_parent(lvglKb, topLayer);
    }
    lv_obj_clear_flag(lvglKb, LV_OBJ_FLAG_IGNORE_LAYOUT);
    lv_obj_clear_flag(lvglKb, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_style_translate_x(lvglKb, 0, 0);
    lv_obj_set_style_translate_y(lvglKb, 0, 0);
    lv_obj_set_size(lvglKb, lv_pct(100), height);
    lv_obj_align(lvglKb, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_move_foreground(lvglKb);
}

static const char *const LVGL_KB_MAP_RU_LOWER[] = {
    "й","ц","у","к","е","н","г","ш","щ","з",LV_SYMBOL_BACKSPACE,"\n",
    "ф","ы","в","а","п","р","о","л","д","ж","э","\n",
    LV_SYMBOL_UP,"я","ч","с","м","и","т","ь","б","ю","ъ","\n",
    "1#"," ",".",LV_SYMBOL_OK,""
};
static const char *const LVGL_KB_MAP_RU_UPPER[] = {
    "Й","Ц","У","К","Е","Н","Г","Ш","Щ","З",LV_SYMBOL_BACKSPACE,"\n",
    "Ф","Ы","В","А","П","Р","О","Л","Д","Ж","Э","\n",
    LV_SYMBOL_UP,"Я","Ч","С","М","И","Т","Ь","Б","Ю","Ъ","\n",
    "1#"," ",".",LV_SYMBOL_OK,""
};
static const char *const LVGL_KB_MAP_TR_LOWER[] = {
    "q","w","e","r","t","y","u","ı","o","p",LV_SYMBOL_BACKSPACE,"\n",
    "a","s","d","f","g","h","j","k","l","ş","i","\n",
    LV_SYMBOL_UP,"z","x","c","v","b","n","m","ö","ç","ü","\n",
    "1#"," ",".",LV_SYMBOL_OK,""
};
static const char *const LVGL_KB_MAP_TR_UPPER[] = {
    "Q","W","E","R","T","Y","U","I","O","P",LV_SYMBOL_BACKSPACE,"\n",
    "A","S","D","F","G","H","J","K","L","Ş","İ","\n",
    LV_SYMBOL_UP,"Z","X","C","V","B","N","M","Ö","Ç","Ü","\n",
    "1#"," ",".",LV_SYMBOL_OK,""
};
static const char *const LVGL_KB_MAP_FR_LOWER[] = {
    "a","z","e","r","t","y","u","i","o","p",LV_SYMBOL_BACKSPACE,"\n",
    "q","s","d","f","g","h","j","k","l","m","é","\n",
    LV_SYMBOL_UP,"w","x","c","v","b","n","à","è","ù","ç","\n",
    "1#"," ",".",LV_SYMBOL_OK,""
};
static const char *const LVGL_KB_MAP_FR_UPPER[] = {
    "A","Z","E","R","T","Y","U","I","O","P",LV_SYMBOL_BACKSPACE,"\n",
    "Q","S","D","F","G","H","J","K","L","M","É","\n",
    LV_SYMBOL_UP,"W","X","C","V","B","N","À","È","Ù","Ç","\n",
    "1#"," ",".",LV_SYMBOL_OK,""
};
static const char *const LVGL_KB_MAP_IT_LOWER[] = {
    "q","w","e","r","t","y","u","i","o","p",LV_SYMBOL_BACKSPACE,"\n",
    "a","s","d","f","g","h","j","k","l","ò","à","\n",
    LV_SYMBOL_UP,"z","x","c","v","b","n","m","è","é","ù","\n",
    "1#"," ",".",LV_SYMBOL_OK,""
};
static const char *const LVGL_KB_MAP_IT_UPPER[] = {
    "Q","W","E","R","T","Y","U","I","O","P",LV_SYMBOL_BACKSPACE,"\n",
    "A","S","D","F","G","H","J","K","L","Ò","À","\n",
    LV_SYMBOL_UP,"Z","X","C","V","B","N","M","È","É","Ù","\n",
    "1#"," ",".",LV_SYMBOL_OK,""
};
static const char *const LVGL_KB_MAP_DE_LOWER[] = {
    "q","w","e","r","t","z","u","i","o","p",LV_SYMBOL_BACKSPACE,"\n",
    "a","s","d","f","g","h","j","k","l","ö","ä","\n",
    LV_SYMBOL_UP,"y","x","c","v","b","n","m","ü","ß",".","\n",
    "1#"," ",".",LV_SYMBOL_OK,""
};
static const char *const LVGL_KB_MAP_DE_UPPER[] = {
    "Q","W","E","R","T","Z","U","I","O","P",LV_SYMBOL_BACKSPACE,"\n",
    "A","S","D","F","G","H","J","K","L","Ö","Ä","\n",
    LV_SYMBOL_UP,"Y","X","C","V","B","N","M","Ü","ẞ",".","\n",
    "1#"," ",".",LV_SYMBOL_OK,""
};
static const lv_btnmatrix_ctrl_t LVGL_KB_CTRL_LOCALIZED[] = {
    3, 3, 3, 3, 3, 3, 3, 3, 3, 3, LV_BTNMATRIX_CTRL_CHECKED | 6,
    3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
    LV_BTNMATRIX_CTRL_CHECKED | 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
    LV_KEYBOARD_CTRL_BTN_FLAGS | 5, 20, 5, LV_KEYBOARD_CTRL_BTN_FLAGS | 6
};

static const char *const *lvglKeyboardLocalizedMap(bool upper)
{
    switch (uiLanguage) {
        case UI_LANG_RUSSIAN: return upper ? LVGL_KB_MAP_RU_UPPER : LVGL_KB_MAP_RU_LOWER;
        case UI_LANG_FRENCH: return upper ? LVGL_KB_MAP_FR_UPPER : LVGL_KB_MAP_FR_LOWER;
        case UI_LANG_TURKISH: return upper ? LVGL_KB_MAP_TR_UPPER : LVGL_KB_MAP_TR_LOWER;
        case UI_LANG_ITALIAN: return upper ? LVGL_KB_MAP_IT_UPPER : LVGL_KB_MAP_IT_LOWER;
        case UI_LANG_GERMAN: return upper ? LVGL_KB_MAP_DE_UPPER : LVGL_KB_MAP_DE_LOWER;
        default: return nullptr;
    }
}

static uint32_t lvglUtf8Codepoint(const char *text)
{
    if (!text || !*text) return 0;
    const uint8_t c0 = static_cast<uint8_t>(text[0]);
    if ((c0 & 0x80U) == 0) return c0;
    if ((c0 & 0xE0U) == 0xC0U && text[1]) {
        return (static_cast<uint32_t>(c0 & 0x1FU) << 6) |
               static_cast<uint32_t>(static_cast<uint8_t>(text[1]) & 0x3FU);
    }
    if ((c0 & 0xF0U) == 0xE0U && text[1] && text[2]) {
        return (static_cast<uint32_t>(c0 & 0x0FU) << 12) |
               (static_cast<uint32_t>(static_cast<uint8_t>(text[1]) & 0x3FU) << 6) |
               static_cast<uint32_t>(static_cast<uint8_t>(text[2]) & 0x3FU);
    }
    return 0;
}

static bool lvglKeyboardIsLetterKey(const char *txt)
{
    const uint32_t cp = lvglUtf8Codepoint(txt);
    if ((cp >= 'A' && cp <= 'Z') || (cp >= 'a' && cp <= 'z')) return true;
    if ((cp >= 0x00C0U && cp <= 0x017FU) || (cp >= 0x0400U && cp <= 0x04FFU)) return true;
    return false;
}

void lvglKeyboardApplyTextMode()
{
    if (!lvglKb) return;

    const bool useLocalized = lvglKeyboardUseLocalized && lvglKeyboardLocalizedLayoutAvailable(uiLanguage);
    if (useLocalized) {
        lv_keyboard_set_map(lvglKb, LV_KEYBOARD_MODE_USER_1, const_cast<const char **>(lvglKeyboardLocalizedMap(false)), LVGL_KB_CTRL_LOCALIZED);
        lv_keyboard_set_map(lvglKb, LV_KEYBOARD_MODE_USER_2, const_cast<const char **>(lvglKeyboardLocalizedMap(true)), LVGL_KB_CTRL_LOCALIZED);
        lv_keyboard_set_mode(lvglKb, (lvglKeyboardShiftOneShot || lvglKeyboardShiftLocked) ? LV_KEYBOARD_MODE_USER_2
                                                                                           : LV_KEYBOARD_MODE_USER_1);
    } else {
        lv_keyboard_set_mode(lvglKb, (lvglKeyboardShiftOneShot || lvglKeyboardShiftLocked) ? LV_KEYBOARD_MODE_TEXT_UPPER
                                                                                           : LV_KEYBOARD_MODE_TEXT_LOWER);
    }
}

void lvglKeyboardEnsureFocusedVisible(bool animated)
{
    if (!lvglKeyboardFocusedTa || !lv_obj_is_valid(lvglKeyboardFocusedTa)) return;
    lv_obj_scroll_to_view_recursive(lvglKeyboardFocusedTa, animated ? LV_ANIM_ON : LV_ANIM_OFF);
}

static void lvglSingleLinePasswordValueChangedEvent(lv_event_t *e)
{
    lv_obj_t *ta = e ? lv_event_get_target(e) : nullptr;
    if (!ta || !lv_obj_is_valid(ta)) return;
    lv_obj_scroll_to_y(ta, 0, LV_ANIM_OFF);
}

static void lvglCompactPasswordGeometryEvent(lv_event_t *e)
{
    lv_obj_t *ta = e ? lv_event_get_target(e) : nullptr;
    if (!ta || !lv_obj_is_valid(ta)) return;
    lvglConfigureCompactSingleLineTa(ta, lv_obj_get_height(ta) > 0 ? lv_obj_get_height(ta) : 34);
}

static void lvglCompactSingleLineTaContent(lv_obj_t *ta, lv_coord_t height)
{
    if (!ta || !lv_obj_is_valid(ta)) return;
    lv_obj_t *label = lv_obj_get_child(ta, 0);
    if (!label || !lv_obj_is_valid(label)) return;
    lv_obj_set_style_text_font(label, LV_FONT_DEFAULT, 0);
    lv_obj_set_style_pad_left(label, 0, 0);
    lv_obj_set_style_pad_right(label, 0, 0);
    lv_obj_set_style_pad_top(label, 0, 0);
    lv_obj_set_style_pad_bottom(label, 0, 0);
    lv_obj_set_style_text_line_space(label, 0, 0);
    lv_obj_set_style_min_height(label, 0, 0);
    lv_obj_set_style_max_height(label, height, 0);
}

static void lvglConfigureCompactSingleLineTa(lv_obj_t *ta, lv_coord_t height = 34)
{
    if (!ta || !lv_obj_is_valid(ta)) return;
    const lv_coord_t h = max<lv_coord_t>(28, height);
    const lv_style_selector_t selectors[] = {
        LV_PART_MAIN,
        LV_PART_MAIN | LV_STATE_ANY,
        LV_PART_MAIN | LV_STATE_DEFAULT,
        LV_PART_MAIN | LV_STATE_FOCUSED,
        LV_PART_MAIN | LV_STATE_EDITED,
        LV_PART_MAIN | LV_STATE_FOCUS_KEY,
        LV_PART_MAIN | LV_STATE_DISABLED,
        LV_PART_MAIN | (LV_STATE_FOCUSED | LV_STATE_EDITED),
        LV_PART_MAIN | (LV_STATE_FOCUSED | LV_STATE_DISABLED),
        LV_PART_MAIN | (LV_STATE_FOCUS_KEY | LV_STATE_EDITED),
    };
    lv_textarea_set_one_line(ta, true);
    lv_obj_set_height(ta, h);
    lv_obj_set_style_text_font(ta, LV_FONT_DEFAULT, LV_PART_MAIN | LV_STATE_ANY);
    lv_obj_set_scroll_dir(ta, LV_DIR_HOR);
    lv_obj_set_scrollbar_mode(ta, LV_SCROLLBAR_MODE_OFF);
    for (size_t i = 0; i < (sizeof(selectors) / sizeof(selectors[0])); ++i) {
        const lv_style_selector_t sel = selectors[i];
        lv_obj_set_style_min_height(ta, h, sel);
        lv_obj_set_style_max_height(ta, h, sel);
        lv_obj_set_style_pad_left(ta, 6, sel);
        lv_obj_set_style_pad_right(ta, 6, sel);
        lv_obj_set_style_pad_top(ta, 0, sel);
        lv_obj_set_style_pad_bottom(ta, 0, sel);
        lv_obj_set_style_text_line_space(ta, 0, sel);
    }
    lvglCompactSingleLineTaContent(ta, h);
    lv_obj_scroll_to_y(ta, 0, LV_ANIM_OFF);
}

static void lvglApplySingleLinePasswordTaState(lv_obj_t *ta)
{
    if (!ta || !lv_obj_is_valid(ta)) return;
    lvglConfigureCompactSingleLineTa(ta, lv_obj_get_height(ta) > 0 ? lv_obj_get_height(ta) : 34);
    lv_textarea_set_password_mode(ta, true);
    lv_obj_scroll_to_y(ta, 0, LV_ANIM_OFF);
}

static void lvglConfigureSingleLinePasswordTa(lv_obj_t *ta)
{
    if (!ta || !lv_obj_is_valid(ta)) return;
    const lv_coord_t h = lv_obj_get_height(ta);
    lvglApplySingleLinePasswordTaState(ta);
    // Keep geometry stable on focus: some themes adjust textarea metrics for focused state.
    if (h > 0) {
        lv_obj_set_height(ta, h);
        lv_obj_set_style_min_height(ta, h, LV_PART_MAIN | LV_STATE_ANY);
        lv_obj_set_style_max_height(ta, h, LV_PART_MAIN | LV_STATE_ANY);
        lv_obj_set_style_min_height(ta, h, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_max_height(ta, h, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_min_height(ta, h, LV_PART_MAIN | LV_STATE_FOCUSED);
        lv_obj_set_style_max_height(ta, h, LV_PART_MAIN | LV_STATE_FOCUSED);
        lv_obj_set_style_min_height(ta, h, LV_PART_MAIN | LV_STATE_EDITED);
        lv_obj_set_style_max_height(ta, h, LV_PART_MAIN | LV_STATE_EDITED);
        lv_obj_set_style_min_height(ta, h, LV_PART_MAIN | LV_STATE_FOCUS_KEY);
        lv_obj_set_style_max_height(ta, h, LV_PART_MAIN | LV_STATE_FOCUS_KEY);
        lv_obj_set_style_min_height(ta, h, LV_PART_MAIN | LV_STATE_DISABLED);
        lv_obj_set_style_max_height(ta, h, LV_PART_MAIN | LV_STATE_DISABLED);
        lv_obj_set_style_min_height(ta, h, LV_PART_MAIN | (LV_STATE_FOCUSED | LV_STATE_EDITED));
        lv_obj_set_style_max_height(ta, h, LV_PART_MAIN | (LV_STATE_FOCUSED | LV_STATE_EDITED));
        lv_obj_set_style_min_height(ta, h, LV_PART_MAIN | (LV_STATE_FOCUSED | LV_STATE_DISABLED));
        lv_obj_set_style_max_height(ta, h, LV_PART_MAIN | (LV_STATE_FOCUSED | LV_STATE_DISABLED));
        lv_obj_set_style_min_height(ta, h, LV_PART_MAIN | (LV_STATE_FOCUS_KEY | LV_STATE_EDITED));
        lv_obj_set_style_max_height(ta, h, LV_PART_MAIN | (LV_STATE_FOCUS_KEY | LV_STATE_EDITED));
    }
    lv_obj_add_event_cb(ta, lvglSingleLinePasswordValueChangedEvent, LV_EVENT_VALUE_CHANGED, nullptr);
    lv_obj_add_event_cb(ta, lvglCompactPasswordGeometryEvent, LV_EVENT_FOCUSED, nullptr);
    lv_obj_add_event_cb(ta, lvglCompactPasswordGeometryEvent, LV_EVENT_DEFOCUSED, nullptr);
}

void lvglKeyboardRefreshLayout()
{
    if (!lvglKeyboardVisible() || !lvglKeyboardFocusedTa || !lv_obj_is_valid(lvglKeyboardFocusedTa)) {
        lvglKeyboardReleasePaddedContainer();
        lvglRefreshWifiPasswordDialogLayout();
        return;
    }

    if (lvglKb && lv_obj_is_valid(lvglKb)) {
        const bool compactDisplay = (DISPLAY_WIDTH <= 240) || (DISPLAY_HEIGHT <= 320);
        const bool compactWifiPwd = compactDisplay && lvglWifiPwdTa && lvglKeyboardFocusedTa == lvglWifiPwdTa;
        lvglPlaceKeyboardBottom(compactWifiPwd ? 96 : 120);
    }

    if (lvglKeyboardFocusedTa == lvglChatInputTa) {
        lvglSetChatKeyboardVisible(true);
        lvglKeyboardEnsureFocusedVisible(false);
        lvglRefreshWifiPasswordDialogLayout();
        return;
    }

    if (lvglWifiPwdTa && lvglKeyboardFocusedTa == lvglWifiPwdTa) {
        lvglKeyboardReleasePaddedContainer();
        lvglSetConfigKeyboardVisible(false);
        lvglRefreshWifiPasswordDialogLayout();
        return;
    }

    const bool configManagedTarget =
        (lvglKeyboardFocusedTa == lvglConfigDeviceNameTa) ||
        (lvglKeyboardFocusedTa == lvglRemoteChannelTa) ||
        (lvglKeyboardFocusedTa == lvglRemoteKeyTa) ||
        (lvglKeyboardFocusedTa == lvglRemoteIdTa) ||
        (lvglKeyboardFocusedTa == lvglRemoteCounterTa) ||
        lvglObjHasAncestor(lvglKeyboardFocusedTa, lvglMqttButtonConfigList) ||
        (hc12CmdTaObj() && lvglKeyboardFocusedTa == hc12CmdTaObj());
    lvglSetConfigKeyboardVisible(configManagedTarget);

    lv_obj_t *scrollable = lvglKeyboardFindScrollableAncestor(lvglKeyboardFocusedTa);
    if (scrollable != lvglKeyboardPaddedContainer) {
        lvglKeyboardReleasePaddedContainer();
        if (scrollable && lv_obj_is_valid(scrollable)) {
            lvglKeyboardPaddedContainer = scrollable;
            lvglKeyboardPaddedContainerPrevBottom = lv_obj_get_style_pad_bottom(scrollable, LV_PART_MAIN);
            lv_obj_set_style_pad_bottom(scrollable, max<lv_coord_t>(lvglKeyboardPaddedContainerPrevBottom, 132), 0);
        }
    } else if (scrollable && lv_obj_is_valid(scrollable)) {
        lv_obj_set_style_pad_bottom(scrollable, max<lv_coord_t>(lvglKeyboardPaddedContainerPrevBottom, 132), 0);
    }

    lvglKeyboardEnsureFocusedVisible(false);
    lvglRefreshWifiPasswordDialogLayout();
}

void lvglStyleScreensaverTypeEvent(lv_event_t *e)
{
    if (lvglStyleUiSyncing) return;
    if (!lvglStyleScreensaverTypeDd) return;

    const uint16_t selected = lv_dropdown_get_selected(lvglStyleScreensaverTypeDd);
    if (selected >= static_cast<uint16_t>(SCREENSAVER_MODE_COUNT)) return;

    screensaverMode = static_cast<ScreensaverMode>(selected);
    uiPrefs.begin("ui", false);
    uiPrefs.putUChar("scrsvr_type", static_cast<uint8_t>(screensaverMode));
    uiPrefs.end();
}

#include "app/chat_contacts.inc"
#include "app/p2p_transport.inc"

static String hc12TerminalExampleCommands()
{
    if (radioModuleType == RADIO_MODULE_E220) {
        return "Examples:\nAT\nAT+CHANNEL=?\nAT+UART=?\nAT+POWER=?\nAT+TRANS=?";
    }
    return "Examples:\nAT\nAT+RX\nAT+RC\nAT+RB\nAT+RF\nAT+RP";
}

static lv_obj_t *hc12WrapObj()
{
    return lvglScrHc12Terminal ? lv_obj_get_child(lvglScrHc12Terminal, 0) : nullptr;
}

static lv_obj_t *hc12SetBtnObj()
{
    lv_obj_t *wrap = hc12WrapObj();
    lv_obj_t *topRow = wrap ? lv_obj_get_child(wrap, 0) : nullptr;
    return topRow ? lv_obj_get_child(topRow, 0) : nullptr;
}

static lv_obj_t *hc12TerminalObj()
{
    lv_obj_t *wrap = hc12WrapObj();
    if (!wrap || !lv_obj_is_valid(wrap)) return nullptr;

    lv_obj_t *obj = lv_obj_get_child(wrap, 1);
    return (obj && lv_obj_is_valid(obj)) ? obj : nullptr;
}

static lv_obj_t *hc12CmdTaObj()
{
    lv_obj_t *wrap = hc12WrapObj();
    if (!wrap || !lv_obj_is_valid(wrap)) return nullptr;

    lv_obj_t *cmdRow = lv_obj_get_child(wrap, 2);
    if (!cmdRow || !lv_obj_is_valid(cmdRow)) return nullptr;

    lv_obj_t *obj = lv_obj_get_child(cmdRow, 0);
    return (obj && lv_obj_is_valid(obj)) ? obj : nullptr;
}

static String &hc12LogBuffer()
{
    if (!hc12TerminalLog) hc12TerminalLog = new String();
    return *hc12TerminalLog;
}

static String &hc12RadioLineBuffer()
{
    if (!hc12RadioRxLine) hc12RadioRxLine = new String();
    return *hc12RadioRxLine;
}

static void hc12AppendTerminal(const char *text)
{
    if (!text || !*text) return;
    String &log = hc12LogBuffer();
    log += text;
    if (log.length() > HC12_TERMINAL_MAX_CHARS) {
        log.remove(0, log.length() - HC12_TERMINAL_MAX_CHARS);
        const int firstNewline = log.indexOf('\n');
        if (firstNewline > 0) log.remove(0, firstNewline + 1);
    }
    lv_obj_t *terminalTa = hc12TerminalObj();
    if (terminalTa && uiScreen == UI_CONFIG_HC12_TERMINAL) {
        lv_textarea_set_text(terminalTa, log.c_str());
        lv_textarea_set_cursor_pos(terminalTa, LV_TEXTAREA_CURSOR_LAST);
    }
}

static void hc12SendLine(const String &line)
{
    hc12InitIfNeeded();
    const bool atMode = hc12SetIsAsserted();
    while (Serial1.available() > 0) Serial1.read();
    String echoed = String("> ") + line + "\n";
    hc12AppendTerminal(echoed.c_str());
    if (radioModuleType == RADIO_MODULE_E220) {
        if (!atMode && line.startsWith("AT")) {
            hc12AppendTerminal("[E220] Enable CFG to use AT commands\n");
            return;
        }
        if (atMode) {
            const String response = e220QueryCommand(line.c_str(), 320UL, 72UL);
            hc12AppendTerminal((response.isEmpty() ? String("[E220] No reply\n") : (response + "\n")).c_str());
            return;
        }
    }
    Serial1.print(line);
    if (radioModuleType == RADIO_MODULE_E220) Serial1.print("\r\n");
    Serial1.flush();
}

static String hc12QueryCommand(const char *line, unsigned long totalTimeoutMs, unsigned long quietTimeoutMs)
{
    hc12InitIfNeeded();
    while (Serial1.available() > 0) Serial1.read();
    Serial1.print(line ? line : "");
    Serial1.flush();

    String response;
    unsigned long deadline = millis() + totalTimeoutMs;
    unsigned long quietDeadline = millis() + quietTimeoutMs;
    bool receivedAny = false;
    while (static_cast<long>(millis() - deadline) < 0) {
        bool gotByte = false;
        while (Serial1.available() > 0) {
            const int ch = Serial1.read();
            if (ch < 0) break;
            if (ch == '\r') continue;
            response += static_cast<char>(ch);
            gotByte = true;
            receivedAny = true;
        }
        if (gotByte) quietDeadline = millis() + quietTimeoutMs;
        if (receivedAny && static_cast<long>(millis() - quietDeadline) >= 0) break;
        delay(2);
    }
    response.trim();
    return response;
}

static String e220QueryCommand(const char *line, unsigned long totalTimeoutMs, unsigned long quietTimeoutMs)
{
    hc12InitIfNeeded();
    while (Serial1.available() > 0) Serial1.read();
    if (line && *line) {
        Serial1.print(line);
        Serial1.print("\r\n");
    }
    Serial1.flush();

    String response;
    unsigned long deadline = millis() + totalTimeoutMs;
    unsigned long quietDeadline = millis() + quietTimeoutMs;
    bool receivedAny = false;
    while (static_cast<long>(millis() - deadline) < 0) {
        bool gotByte = false;
        while (Serial1.available() > 0) {
            const int ch = Serial1.read();
            if (ch < 0) break;
            if (ch == '\r') continue;
            response += static_cast<char>(ch);
            gotByte = true;
            receivedAny = true;
        }
        if (gotByte) quietDeadline = millis() + quietTimeoutMs;
        if (receivedAny && static_cast<long>(millis() - quietDeadline) >= 0) break;
        delay(2);
    }
    response.trim();
    return response;
}

static void hc12EnterAtMode()
{
    hc12InitIfNeeded();

    while (Serial1.available() > 0) Serial1.read();

    if (radioModuleType == RADIO_MODULE_HC12) {
        digitalWrite(hc12ActiveSetPin(), LOW);
        delay(80);
        return;
    }

    digitalWrite(e220ActiveM0Pin(), HIGH);
    digitalWrite(e220ActiveM1Pin(), HIGH);
    delay(120);  // was 80

    hc12SerialReopen(E220_AT_BAUD);
    delay(30);   // extra settle after reopen
    while (Serial1.available() > 0) Serial1.read();
}

static void hc12ExitAtMode()
{
    while (Serial1.available() > 0) Serial1.read();

    if (radioModuleType == RADIO_MODULE_HC12) {
        digitalWrite(hc12ActiveSetPin(), HIGH);
        delay(40);
        uiDeferredFlags &= static_cast<uint8_t>(~(UI_DEFERRED_HC12_SETTLE_PENDING | UI_DEFERRED_HC12_TARGET_ASSERTED));
        return;
    }

    digitalWrite(e220ActiveM0Pin(), LOW);
    digitalWrite(e220ActiveM1Pin(), LOW);
    delay(80);   // was 40

    hc12SerialReopen(e220RuntimeBaud());
    delay(30);   // extra settle after reopen
    while (Serial1.available() > 0) Serial1.read();

    uiDeferredFlags &= static_cast<uint8_t>(~(UI_DEFERRED_HC12_SETTLE_PENDING | UI_DEFERRED_HC12_TARGET_ASSERTED));
}

static bool hc12TryEnterSleepMode()
{
    if (!moduleInstalled[APP_MODULE_RADIO] || radioModuleType != RADIO_MODULE_HC12) return false;
    hc12EnterAtMode();
    const String response = hc12QueryCommand("AT+SLEEP", 220UL, 48UL);
    const bool accepted = response.startsWith("OK");
    hc12ExitAtMode();
    return accepted;
}

static int e220FindBaudIndexFromCode(int code)
{
    for (int i = 0; i < static_cast<int>(sizeof(E220_UART_BAUD_OPTIONS) / sizeof(E220_UART_BAUD_OPTIONS[0])); ++i) {
        if (E220_UART_BAUD_OPTIONS[i] == code) return i;
    }
    return 3;
}

static int e220FindAirRateIndexFromCode(int code)
{
    for (int i = 0; i < static_cast<int>(sizeof(E220_AIR_RATE_OPTIONS) / sizeof(E220_AIR_RATE_OPTIONS[0])); ++i) {
        if (E220_AIR_RATE_OPTIONS[i] == code) return i;
    }
    return 2;
}

static String e220ValueAfterEquals(String raw)
{
    raw.trim();
    const int eq = raw.indexOf('=');
    if (eq < 0) return raw;
    String value = raw.substring(eq + 1);
    const int nl = value.indexOf('\n');
    if (nl >= 0) value.remove(nl);
    value.trim();
    return value;
}

static int hc12FindBaudIndex(unsigned long baud)
{
    for (int i = 0; i < static_cast<int>(sizeof(HC12_SUPPORTED_BAUDS) / sizeof(HC12_SUPPORTED_BAUDS[0])); ++i) {
        if (HC12_SUPPORTED_BAUDS[i] == baud) return i;
    }
    return 3;
}

static int hc12FindPowerLevelFromResponse(const String &raw)
{
    String value = hc12FieldValue(raw, "RP:");
    if (value == "--") value = hc12FieldValue(raw, "RP");
    value.replace("dBm", "");
    value.trim();
    const int dbm = value.toInt();
    for (int i = 0; i < static_cast<int>(sizeof(HC12_POWER_DBM) / sizeof(HC12_POWER_DBM[0])); ++i) {
        if (HC12_POWER_DBM[i] == dbm) return i + 1;
    }
    return hc12CurrentPowerLevel;
}

static void loadPersistedRadioSettings()
{
    hc12SwapUartPins = uiPrefs.getBool("hc12_swap", false);
    hc12CurrentChannel = constrain(uiPrefs.getInt("hc12_ch", HC12_MIN_CHANNEL), HC12_MIN_CHANNEL, HC12_MAX_CHANNEL);
    hc12CurrentBaudIndex = constrain(uiPrefs.getInt("hc12_baud", 3), 0, static_cast<int>(sizeof(HC12_SUPPORTED_BAUDS) / sizeof(HC12_SUPPORTED_BAUDS[0])) - 1);
    hc12CurrentModeIndex = constrain(uiPrefs.getInt("hc12_mode", 2), 0, 3);
    hc12CurrentPowerLevel = constrain(uiPrefs.getInt("hc12_pwr", 8), 1, 8);

    e220SwapUartPins = uiPrefs.getBool("e220_swap", false);
    e220SwapModePins = uiPrefs.getBool("e220_mswap", false);
    e220CurrentChannel = constrain(uiPrefs.getInt("e220_ch", 23), E220_MIN_CHANNEL, E220_MAX_CHANNEL);
    e220CurrentBaudIndex = constrain(uiPrefs.getInt("e220_baud", 3), 0, static_cast<int>(sizeof(E220_UART_BAUD_VALUES) / sizeof(E220_UART_BAUD_VALUES[0])) - 1);
    e220CurrentAirRateIndex = constrain(uiPrefs.getInt("e220_rate", 2), 0, static_cast<int>(sizeof(E220_AIR_RATE_OPTIONS) / sizeof(E220_AIR_RATE_OPTIONS[0])) - 1);
    e220CurrentPowerIndex = constrain(uiPrefs.getInt("e220_pwr", 0), 0, 3);
    e220CurrentFixedTransmission = uiPrefs.getBool("e220_fix", false);
}

static void savePersistedRadioSettings()
{
    uiPrefs.begin("ui", false);
    uiPrefs.putBool("hc12_swap", hc12SwapUartPins);
    uiPrefs.putInt("hc12_ch", hc12CurrentChannel);
    uiPrefs.putInt("hc12_baud", hc12CurrentBaudIndex);
    uiPrefs.putInt("hc12_mode", hc12CurrentModeIndex);
    uiPrefs.putInt("hc12_pwr", hc12CurrentPowerLevel);

    uiPrefs.putBool("e220_swap", e220SwapUartPins);
    uiPrefs.putBool("e220_mswap", e220SwapModePins);
    uiPrefs.putInt("e220_ch", e220CurrentChannel);
    uiPrefs.putInt("e220_baud", e220CurrentBaudIndex);
    uiPrefs.putInt("e220_rate", e220CurrentAirRateIndex);
    uiPrefs.putInt("e220_pwr", e220CurrentPowerIndex);
    uiPrefs.putBool("e220_fix", e220CurrentFixedTransmission);
    uiPrefs.end();
}

static bool hc12ReadConfigSelection()
{
    if (radioModuleType == RADIO_MODULE_E220) {
        hc12EnterAtMode();
        const String uartRaw = e220QueryCommand("AT+UART=?");
        const String channelRaw = e220QueryCommand("AT+CHANNEL=?");
        const String rateRaw = e220QueryCommand("AT+RATE=?");
        const String powerRaw = e220QueryCommand("AT+POWER=?");
        const String transRaw = e220QueryCommand("AT+TRANS=?");
        hc12ExitAtMode();

        bool ok = false;
        const String uartValue = e220ValueAfterEquals(uartRaw);
        if (uartValue.length() > 0) {
            const int code = uartValue.substring(0, uartValue.indexOf(',') >= 0 ? uartValue.indexOf(',') : uartValue.length()).toInt();
            e220CurrentBaudIndex = e220FindBaudIndexFromCode(code);
            ok = true;
        }
        const String channelValue = e220ValueAfterEquals(channelRaw);
        if (channelValue.length() > 0) {
            e220CurrentChannel = constrain(channelValue.toInt(), E220_MIN_CHANNEL, E220_MAX_CHANNEL);
            ok = true;
        }
        const String rateValue = e220ValueAfterEquals(rateRaw);
        if (rateValue.length() > 0) {
            e220CurrentAirRateIndex = e220FindAirRateIndexFromCode(rateValue.toInt());
            ok = true;
        }
        const String powerValue = e220ValueAfterEquals(powerRaw);
        if (powerValue.length() > 0) {
            e220CurrentPowerIndex = constrain(powerValue.toInt(), 0, 3);
            ok = true;
        }
        const String transValue = e220ValueAfterEquals(transRaw);
        if (transValue.length() > 0) {
            e220CurrentFixedTransmission = transValue.toInt() != 0;
            ok = true;
        }
        if (ok) savePersistedRadioSettings();
        hc12ConfigStatusText = ok ? "E220 settings loaded" : "E220 did not return settings";
        return ok;
    }

    hc12EnterAtMode();
    const String baudRaw = hc12QueryCommand("AT+RB");
    const String channelRaw = hc12QueryCommand("AT+RC");
    const String modeRaw = hc12QueryCommand("AT+RF");
    const String powerRaw = hc12QueryCommand("AT+RP");
    hc12ExitAtMode();

    bool ok = false;
    const String baudValue = hc12FieldValue(baudRaw, "B");
    if (baudValue != "--") {
        const unsigned long baud = strtoul(baudValue.c_str(), nullptr, 10);
        if (baud > 0UL) {
            hc12CurrentBaudIndex = hc12FindBaudIndex(baud);
            ok = true;
        }
    }

    const String channelValue = hc12FieldValue(channelRaw, "RC");
    if (channelValue != "--") {
        int channel = channelValue.toInt();
        if (channel >= HC12_MIN_CHANNEL && channel <= HC12_MAX_CHANNEL) {
            hc12CurrentChannel = channel;
            ok = true;
        }
    }

    const String modeValue = hc12FieldValue(modeRaw, "FU");
    if (modeValue != "--") {
        const int mode = modeValue.toInt();
        if (mode >= 1 && mode <= 4) {
            hc12CurrentModeIndex = mode - 1;
            ok = true;
        }
    }

    hc12CurrentPowerLevel = hc12FindPowerLevelFromResponse(powerRaw);
    ok = true;

    if (ok) savePersistedRadioSettings();
    hc12ConfigStatusText = ok ? "Module settings loaded" : "HC-12 did not return settings";
    return ok;
}

static bool hc12ApplyChannel(int channel)
{
    if (radioModuleType == RADIO_MODULE_E220) {
        channel = constrain(channel, E220_MIN_CHANNEL, E220_MAX_CHANNEL);
        hc12EnterAtMode();
        const String resp = e220QueryCommand((String("AT+CHANNEL=") + channel).c_str());
        hc12ExitAtMode();
        if (resp.indexOf("OK") >= 0) {
            e220CurrentChannel = channel;
            savePersistedRadioSettings();
            hc12ConfigStatusText = "Channel set to CH" + String(channel);
            return true;
        }
        hc12ConfigStatusText = resp.isEmpty() ? "Channel change failed" : resp;
        return false;
    }
    channel = constrain(channel, HC12_MIN_CHANNEL, HC12_MAX_CHANNEL);
    char cmd[12];
    snprintf(cmd, sizeof(cmd), "AT+C%03d", channel);
    hc12EnterAtMode();
    const String resp = hc12QueryCommand(cmd);
    hc12ExitAtMode();
    if (resp.startsWith("OK+C")) {
        hc12CurrentChannel = channel;
        savePersistedRadioSettings();
        hc12ConfigStatusText = "Channel set to CH" + String(channel);
        return true;
    }
    hc12ConfigStatusText = resp.isEmpty() ? "Channel change failed" : resp;
    return false;
}

static bool hc12ApplyBaudIndex(int index)
{
    if (radioModuleType == RADIO_MODULE_E220) {
        const int count = static_cast<int>(sizeof(E220_UART_BAUD_OPTIONS) / sizeof(E220_UART_BAUD_OPTIONS[0]));
        if (count <= 0) return false;

        index = constrain(index, 0, count - 1);

        hc12EnterAtMode();
        const String resp = e220QueryCommand((String("AT+UART=") + E220_UART_BAUD_OPTIONS[index] + ",0").c_str());
        const bool ok = resp.indexOf("OK") >= 0;

        if (ok) {
            e220CurrentBaudIndex = index;
            savePersistedRadioSettings();
            hc12ConfigStatusText = "UART baud set to " + String(E220_UART_BAUD_VALUES[index]) + " bps";
        } else {
            hc12ConfigStatusText = resp.isEmpty() ? "Baud change failed" : resp;
        }

        hc12ExitAtMode();

        if (ok) {
            hc12RestartWithCurrentPins(hc12ConfigStatusText);
            return true;
        }
        return false;
    }
    const int count = static_cast<int>(sizeof(HC12_SUPPORTED_BAUDS) / sizeof(HC12_SUPPORTED_BAUDS[0]));
    if (count <= 0) return false;
    index = constrain(index, 0, count - 1);
    char cmd[18];
    snprintf(cmd, sizeof(cmd), "AT+B%lu", HC12_SUPPORTED_BAUDS[index]);
    hc12EnterAtMode();
    const String resp = hc12QueryCommand(cmd);
    hc12ExitAtMode();
    if (resp.startsWith("OK+B")) {
        hc12CurrentBaudIndex = index;
        savePersistedRadioSettings();
        hc12ConfigStatusText = "Baud set to " + String(HC12_SUPPORTED_BAUDS[index]) + " bps";
        return true;
    }
    hc12ConfigStatusText = resp.isEmpty() ? "Baud change failed" : resp;
    return false;
}

static bool hc12ApplyModeIndex(int index)
{
    if (radioModuleType == RADIO_MODULE_E220) {
        const int count = static_cast<int>(sizeof(E220_AIR_RATE_OPTIONS) / sizeof(E220_AIR_RATE_OPTIONS[0]));
        index = constrain(index, 0, count - 1);
        hc12EnterAtMode();
        const String resp = e220QueryCommand((String("AT+RATE=") + E220_AIR_RATE_OPTIONS[index]).c_str());
        hc12ExitAtMode();
        if (resp.indexOf("OK") >= 0) {
            e220CurrentAirRateIndex = index;
            savePersistedRadioSettings();
            hc12ConfigStatusText = String("Air rate set to ") + E220_AIR_RATE_LABELS[index] + " bps";
            return true;
        }
        hc12ConfigStatusText = resp.isEmpty() ? "Air rate change failed" : resp;
        return false;
    }
    index = constrain(index, 0, 3);
    char cmd[12];
    snprintf(cmd, sizeof(cmd), "AT+FU%d", index + 1);
    hc12EnterAtMode();
    const String resp = hc12QueryCommand(cmd);
    hc12ExitAtMode();
    if (resp.startsWith("OK+FU")) {
        hc12CurrentModeIndex = index;
        savePersistedRadioSettings();
        hc12ConfigStatusText = String("Mode set to ") + HC12_MODE_LABELS[index];
        return true;
    }
    hc12ConfigStatusText = resp.isEmpty() ? "Mode change failed" : resp;
    return false;
}

static bool hc12ApplyPowerLevel(int level)
{
    if (radioModuleType == RADIO_MODULE_E220) {
        level = constrain(level, 0, 3);
        hc12EnterAtMode();
        const String resp = e220QueryCommand((String("AT+POWER=") + level).c_str());
        hc12ExitAtMode();
        if (resp.indexOf("OK") >= 0) {
            e220CurrentPowerIndex = level;
            savePersistedRadioSettings();
            hc12ConfigStatusText = String("Power set to ") + E220_POWER_DBM[level] + " dBm";
            return true;
        }
        hc12ConfigStatusText = resp.isEmpty() ? "Power change failed" : resp;
        return false;
    }
    level = constrain(level, 1, 8);
    char cmd[12];
    snprintf(cmd, sizeof(cmd), "AT+P%d", level);
    hc12EnterAtMode();
    const String resp = hc12QueryCommand(cmd);
    hc12ExitAtMode();
    if (resp.startsWith("OK+P")) {
        hc12CurrentPowerLevel = level;
        savePersistedRadioSettings();
        hc12ConfigStatusText = String("Power set to ") + HC12_POWER_DBM[level - 1] + " dBm";
        return true;
    }
    hc12ConfigStatusText = resp.isEmpty() ? "Power change failed" : resp;
    return false;
}

static bool e220ApplyTransferMode(bool fixedTransmission)
{
    hc12EnterAtMode();
    const String resp = e220QueryCommand((String("AT+TRANS=") + (fixedTransmission ? 1 : 0)).c_str());
    hc12ExitAtMode();
    if (resp.indexOf("OK") >= 0) {
        e220CurrentFixedTransmission = fixedTransmission;
        savePersistedRadioSettings();
        hc12ConfigStatusText = String("Transfer mode set to ") + (fixedTransmission ? "Fixed" : "Transparent");
        if (fixedTransmission && lvglReady) lvglShowE220FixedModeWarning();
        return true;
    }
    hc12ConfigStatusText = resp.isEmpty() ? "Transfer mode change failed" : resp;
    return false;
}

static bool hc12FactoryReset()
{
    if (radioModuleType == RADIO_MODULE_E220) {
        const bool prevSwapUartPins = e220SwapUartPins;
        const bool prevSwapModePins = e220SwapModePins;
        hc12EnterAtMode();
        const String resp = e220QueryCommand("AT+CFGTF", 420UL, 96UL);
        hc12ExitAtMode();
        bool resetConfirmed = resp.indexOf("OK") >= 0;
        if (!resetConfirmed) {
            e220SwapUartPins = false;
            e220SwapModePins = false;
            hc12RestartWithCurrentPins("");
            hc12EnterAtMode();
            resetConfirmed = e220QueryCommand("AT", 320UL, 72UL).indexOf("OK") >= 0;
            hc12ExitAtMode();
            if (!resetConfirmed) {
                e220SwapUartPins = prevSwapUartPins;
                e220SwapModePins = prevSwapModePins;
                hc12RestartWithCurrentPins("");
            }
        }
        if (resetConfirmed) {
            e220SwapUartPins = false;
            e220SwapModePins = false;
            e220CurrentChannel = 23;
            e220CurrentBaudIndex = 3;
            e220CurrentAirRateIndex = 2;
            e220CurrentPowerIndex = 0;
            e220CurrentFixedTransmission = false;
            savePersistedRadioSettings();
            hc12RestartWithCurrentPins("Factory defaults restored");
            return true;
        }
        hc12ConfigStatusText = resp.isEmpty() ? "Factory reset failed" : resp;
        return false;
    }
    hc12EnterAtMode();
    const String resp = hc12QueryCommand("AT+DEFAULT", 260UL, 48UL);
    hc12ExitAtMode();
    if (resp.startsWith("OK+DEFAULT")) {
        hc12SwapUartPins = false;
        hc12CurrentChannel = 1;
        hc12CurrentBaudIndex = hc12FindBaudIndex(9600UL);
        hc12CurrentModeIndex = 2;
        hc12CurrentPowerLevel = 8;
        savePersistedRadioSettings();
        hc12ConfigStatusText = "Factory defaults restored";
        return true;
    }
    hc12ConfigStatusText = resp.isEmpty() ? "Factory reset failed" : resp;
    return false;
}

static void hc12RefreshInfoSnapshot()
{
    if (radioModuleType == RADIO_MODULE_E220) {
        hc12InitIfNeeded();
        hc12InfoValueText = "Querying...";
        hc12InfoSubText = "Entering config mode...";

        hc12EnterAtMode();
        delay(120);  // let E220 fully settle in CFG mode

        const String modelRaw   = e220QueryCommand("AT+DEVTYPE=?");
        delay(30);
        const String fwRaw      = e220QueryCommand("AT+FWCODE=?");
        delay(30);
        const String uartRaw    = e220QueryCommand("AT+UART=?");
        delay(30);
        const String channelRaw = e220QueryCommand("AT+CHANNEL=?");
        delay(30);
        const String rateRaw    = e220QueryCommand("AT+RATE=?");
        delay(30);
        const String powerRaw   = e220QueryCommand("AT+POWER=?");
        delay(30);
        const String transRaw   = e220QueryCommand("AT+TRANS=?");

        hc12ExitAtMode();
        delay(80);   // let module return cleanly to normal mode

        hc12InfoValueText = e220ValueAfterEquals(modelRaw);
        if (hc12InfoValueText.isEmpty()) hc12InfoValueText = "No Reply";
        hc12InfoSubText = "FW " + e220ValueAfterEquals(fwRaw);

        if (lvglHc12InfoVersionLabel) lv_label_set_text(lvglHc12InfoVersionLabel, hc12InfoValueText.c_str());
        
        const String uartVal    = e220ValueAfterEquals(uartRaw);
        const String channelVal = e220ValueAfterEquals(channelRaw);
        const String transVal   = e220ValueAfterEquals(transRaw);
        const String powerVal   = e220ValueAfterEquals(powerRaw);
        const String rateVal    = e220ValueAfterEquals(rateRaw);

        if (lvglHc12InfoBaudLabel)    lv_label_set_text(lvglHc12InfoBaudLabel,    uartVal.isEmpty()    ? "--" : uartVal.c_str());
        if (lvglHc12InfoChannelLabel) lv_label_set_text(lvglHc12InfoChannelLabel, channelVal.isEmpty() ? "--" : channelVal.c_str());
        if (lvglHc12InfoFuModeLabel)  lv_label_set_text(lvglHc12InfoFuModeLabel,  transVal.isEmpty()   ? "--" : transVal.c_str());
        if (lvglHc12InfoPowerLabel)   lv_label_set_text(lvglHc12InfoPowerLabel,   powerVal.isEmpty()   ? "--" : powerVal.c_str());

        if (lvglHc12InfoRawLabel) {
            String raw = String("UART ") + (uartVal.isEmpty() ? "--" : uartVal) +
                        " | RATE " + (rateVal.isEmpty() ? "--" : rateVal) +
                        " | TRANS " + (transVal.isEmpty() ? "--" : transVal);
            lv_label_set_text(lvglHc12InfoRawLabel, raw.c_str());
        }

        if (lvglHc12InfoRawLabel) {
            String raw = String("UART ") + e220ValueAfterEquals(uartRaw) +
                         " | RATE " + e220ValueAfterEquals(rateRaw) +
                         " | TRANS " + e220ValueAfterEquals(transRaw);
            lv_label_set_text(lvglHc12InfoRawLabel, raw.c_str());
        }
        return;
    }

    // keep your HC-12 branch as-is
    hc12InitIfNeeded();
    hc12InfoValueText = "Querying.";
    hc12InfoSubText = "Entering AT mode.";

    hc12EnterAtMode();
    const String versionRaw = hc12QueryCommand("AT+V");
    const String configRaw = hc12QueryCommand("AT+RX");
    hc12ExitAtMode();

    String summary = versionRaw;
    if (!summary.isEmpty() && !configRaw.isEmpty()) summary += " | ";
    summary += configRaw;
    summary.replace("\n", " | ");
    while (summary.indexOf(" |  | ") >= 0) summary.replace(" |  | ", " | ");
    summary.trim();

    if (versionRaw.isEmpty() && configRaw.isEmpty()) {
        hc12InfoValueText = "No Reply";
        hc12InfoSubText = "No response from HC-12 in AT mode";
        return;
    }

    hc12InfoValueText = !versionRaw.isEmpty() ? versionRaw : String("Ready");
    hc12InfoSubText = summary.isEmpty() ? String("Query completed") : summary;
}

static String hc12CompactResponse(String raw)
{
    raw.replace("\n", " | ");
    while (raw.indexOf(" |  | ") >= 0) raw.replace(" |  | ", " | ");
    raw.trim();
    return raw;
}

static String hc12PayloadAfterOk(String raw)
{
    raw = hc12CompactResponse(raw);
    if (raw.startsWith("OK+")) raw.remove(0, 3);
    raw.trim();
    return raw;
}

static String hc12FieldValue(const String &raw, const char *prefix)
{
    String value = hc12PayloadAfterOk(raw);
    if (prefix && *prefix && value.startsWith(prefix)) value.remove(0, strlen(prefix));
    value.trim();
    return value.isEmpty() ? String("--") : value;
}

static ChatTransport radioSelectedChatTransport()
{
    return radioModuleType == RADIO_MODULE_E220 ? CHAT_TRANSPORT_E220 : CHAT_TRANSPORT_HC12;
}

static bool radioModuleCanCarryChat()
{
    if (radioModuleType == RADIO_MODULE_HC12) return true;
    if (radioModuleType == RADIO_MODULE_E220) return !e220CurrentFixedTransmission;
    return false;
}

static const char *radioModuleChatLabel()
{
    return radioModuleType == RADIO_MODULE_E220 ? "E220" : "HC-12";
}

bool hc12SendChatMessageWithId(const String &peerKey, const String &text, const String &messageId)
{
    if (peerKey.isEmpty() || text.isEmpty() || messageId.isEmpty()) return false;

    const int peerIdx = p2pFindPeerByPubKeyHex(peerKey);
    if (peerIdx < 0 || !p2pPeers[peerIdx].enabled) return false;

    JsonDocument doc;
    doc["kind"] = "chat";
    doc["id"] = messageId;
    doc["author"] = deviceShortNameValue();
    doc["text"] = text.substring(0, P2P_MAX_CHAT_TEXT);

    char plain[P2P_MAX_PACKET] = {0};
    const size_t plainLen = serializeJson(doc, plain, sizeof(plain));
    if (plainLen == 0) return false;

    return hc12SendEncryptedPayload(peerKey, reinterpret_cast<const uint8_t *>(plain), plainLen);
}

bool hc12SendChatAck(const String &peerKey, const String &messageId)
{
    if (peerKey.isEmpty() || messageId.isEmpty()) return false;

    const int peerIdx = p2pFindPeerByPubKeyHex(peerKey);
    if (peerIdx < 0 || !p2pPeers[peerIdx].enabled) return false;

    JsonDocument doc;
    doc["kind"] = "ack";
    doc["ack_id"] = messageId;

    char plain[P2P_MAX_PACKET] = {0};
    const size_t plainLen = serializeJson(doc, plain, sizeof(plain));
    if (plainLen == 0) return false;

    return hc12SendEncryptedPayload(peerKey, reinterpret_cast<const uint8_t *>(plain), plainLen);
}

static bool hc12SendPairResponse(const String &pubKeyHex, bool accepted)
{
    if (pubKeyHex.isEmpty()) return false;

    JsonDocument doc;
    doc["kind"] = accepted ? "pair_accept" : "pair_reject";
    doc["author"] = deviceShortNameValue();

    char plain[P2P_MAX_PACKET] = {0};
    const size_t plainLen = serializeJson(doc, plain, sizeof(plain));
    if (plainLen == 0) return false;

    return hc12SendEncryptedPayload(pubKeyHex, reinterpret_cast<const uint8_t *>(plain), plainLen);
}

bool hc12SendMessageDelete(const String &peerKey, const String &messageId)
{
    if (peerKey.isEmpty() || messageId.isEmpty()) return false;

    const int peerIdx = p2pFindPeerByPubKeyHex(peerKey);
    if (peerIdx < 0 || !p2pPeers[peerIdx].enabled) return false;

    JsonDocument doc;
    doc["kind"] = "delete_message";
    doc["id"] = messageId;
    doc["author"] = deviceShortNameValue();

    char plain[P2P_MAX_PACKET] = {0};
    const size_t plainLen = serializeJson(doc, plain, sizeof(plain));
    if (plainLen == 0) return false;

    return hc12SendEncryptedPayload(peerKey, reinterpret_cast<const uint8_t *>(plain), plainLen);
}

bool hc12SendConversationDelete(const String &peerKey)
{
    if (peerKey.isEmpty()) return false;

    const int peerIdx = p2pFindPeerByPubKeyHex(peerKey);
    if (peerIdx < 0 || !p2pPeers[peerIdx].enabled) return false;

    JsonDocument doc;
    doc["kind"] = "delete_conversation";
    doc["author"] = deviceShortNameValue();

    char plain[P2P_MAX_PACKET] = {0};
    const size_t plainLen = serializeJson(doc, plain, sizeof(plain));
    if (plainLen == 0) return false;

    return hc12SendEncryptedPayload(peerKey, reinterpret_cast<const uint8_t *>(plain), plainLen);
}

static void hc12HandleIncomingRadioLine(const String &line)
{
    if (!line.startsWith(HC12_RADIO_FRAME_PREFIX)) return;
    const ChatTransport radioTransport = radioSelectedChatTransport();

    JsonDocument outerDoc;
    const String body = line.substring(strlen(HC12_RADIO_FRAME_PREFIX));
    const DeserializationError err = deserializeJson(outerDoc, body);
    if (err != DeserializationError::Ok) {
        if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
            hc12AppendTerminal((String("[RADIO] outer JSON parse failed: ") + err.c_str() + "\n").c_str());
            hc12AppendTerminal((String("[RADIO] raw len=") + body.length() + "\n").c_str());
            hc12AppendTerminal((String("[RADIO] raw=") + body + "\n").c_str());
        }
        return;
    }

    const String kind = String(static_cast<const char *>(outerDoc["kind"] | ""));

    // 1) Plain discovery frames
    const String discoveryPubHex = String(static_cast<const char *>(outerDoc["public_key"] | ""));
    if (!kind.isEmpty() && !discoveryPubHex.isEmpty()) {
        if (discoveryPubHex.equalsIgnoreCase(p2pPublicKeyHex())) return;
        const String senderName = sanitizeDeviceShortName(String(static_cast<const char *>(outerDoc["device"] | "Peer")));
        hc12TouchDiscoveredPeer(senderName, discoveryPubHex);
        if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
            hc12AppendTerminal((String("[RADIO] DISCOVERY ") + kind + " from " + senderName + "\n").c_str());
            hc12AppendTerminal((String("[RADIO] discovery pub=") + discoveryPubHex + "\n").c_str());
            hc12AppendTerminal((String("[RADIO] my pub=") + p2pPublicKeyHex() + "\n").c_str());
        }
        if (kind == "probe") hc12BroadcastDiscoveryFrame("hello");
        return;
    }

    // 2) Plain ping/pong frames
    if (kind == "rping") {
        const String from = String(static_cast<const char *>(outerDoc["device"] | "Peer"));
        if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
            hc12AppendTerminal((String("[RADIO] PING from ") + from + "\n").c_str());
        }
        hc12SendRadioPong();
        return;
    }

    if (kind == "rpong") {
        const String from = String(static_cast<const char *>(outerDoc["device"] | "Peer"));
        if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
            hc12AppendTerminal((String("[RADIO] PONG from ") + from + "\n").c_str());
        }
        uiStatusLine = String("Radio link OK: ") + from;
        if (lvglReady) lvglSyncStatusLine();
        return;
    }

    // 3) Encrypted frames only below this point
    const String senderPubHex = String(static_cast<const char *>(outerDoc["sender_pub"] | ""));
    const String nonceHex = String(static_cast<const char *>(outerDoc["nonce"] | ""));
    const String cipherHex = String(static_cast<const char *>(outerDoc["cipher"] | ""));
    if (senderPubHex.isEmpty() || nonceHex.isEmpty() || cipherHex.isEmpty()) {
        if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
            hc12AppendTerminal("[RADIO] encrypted frame missing sender_pub/nonce/cipher\n");
        }
        return;
    }

    if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
        hc12AppendTerminal((String("[RADIO] rx sender_pub=") + senderPubHex + "\n").c_str());
        hc12AppendTerminal((String("[RADIO] rx myPub=") + p2pPublicKeyHex() + "\n").c_str());
    }

    if (senderPubHex.equalsIgnoreCase(p2pPublicKeyHex())) {
        if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
            hc12AppendTerminal("[RADIO] ignoring self frame\n");
        }
        return;
    }

    unsigned char senderPk[P2P_PUBLIC_KEY_BYTES] = {0};
    unsigned char nonce[P2P_NONCE_BYTES] = {0};
    unsigned char cipher[P2P_MAX_PACKET] = {0};
    size_t cipherLen = 0;

    if (!p2pHexToBytes(senderPubHex, senderPk, sizeof(senderPk))) {
        if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
            hc12AppendTerminal("[RADIO] sender_pub hex decode failed\n");
        }
        return;
    }

    if (!p2pHexToBytes(nonceHex, nonce, sizeof(nonce))) {
        if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
            hc12AppendTerminal("[RADIO] nonce hex decode failed\n");
        }
        return;
    }

    if (sodium_hex2bin(cipher, sizeof(cipher), cipherHex.c_str(), cipherHex.length(), nullptr, &cipherLen, nullptr) != 0) {
        if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
            hc12AppendTerminal("[RADIO] cipher hex decode failed\n");
        }
        return;
    }

    if (cipherLen < P2P_MAC_BYTES) {
        if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
            hc12AppendTerminal("[RADIO] cipher too short\n");
        }
        return;
    }

    unsigned char plain[P2P_MAX_PACKET] = {0};
    if (crypto_box_curve25519xchacha20poly1305_open_easy(plain, cipher, cipherLen, nonce, senderPk, p2pSecretKey) != 0) {
        if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
            hc12AppendTerminal("[RADIO] decrypt failed\n");
        }
        uiStatusLine = "Radio decrypt failed";
        if (lvglReady) lvglSyncStatusLine();
        return;
    }

    JsonDocument doc;
    if (deserializeJson(doc, plain, cipherLen - P2P_MAC_BYTES) != DeserializationError::Ok) {
        if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
            hc12AppendTerminal("[RADIO] decrypted JSON parse failed\n");
        }
        return;
    }

    const String author = sanitizeDeviceShortName(
        String(static_cast<const char *>(doc["author"] | chatDisplayNameForPeerKey(senderPubHex).c_str()))
    );

    const String innerKind = String(static_cast<const char *>(doc["kind"] | ""));
    if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
        hc12AppendTerminal((String("[RADIO] encrypted kind=") + innerKind + " from " + author + "\n").c_str());
    }

    if (innerKind == "chat") {
        hc12TouchDiscoveredPeer(author, senderPubHex);

        const String messageId = String(static_cast<const char *>(doc["id"] | ""));
        const String text = String(static_cast<const char *>(doc["text"] | ""));
        if (text.isEmpty()) {
            if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
                hc12AppendTerminal("[RADIO] chat payload empty\n");
            }
            return;
        }
        const bool isDuplicate = !messageId.isEmpty() && chatHasLoggedMessageId(senderPubHex, messageId);
        if (!messageId.isEmpty()) hc12SendChatAck(senderPubHex, messageId);
        wakeDisplayForIncomingNotification();
        if (checkersHandleIncomingChatPayload(senderPubHex, author, text, radioTransport, messageId)) {
            if (lvglReady) {
                lvglSyncStatusLine();
                if (uiScreen == UI_CHAT) lvglRefreshChatUi();
            }
        } else if (!isDuplicate) {
            chatStoreMessage(senderPubHex, author, text, false, radioTransport, messageId);
            uiStatusLine = String(radioModuleChatLabel()) + " chat from " + author;
            if (lvglReady) {
                lvglSyncStatusLine();
                if (uiScreen == UI_CHAT) lvglRefreshChatUi();
            }
        }
        return;
    }

    if (innerKind == "ack") {
        const String ackId = String(static_cast<const char *>(doc["ack_id"] | ""));
        if (!ackId.isEmpty()) chatAckOutgoingMessage(senderPubHex, ackId);
        return;
    }

    if (innerKind == "delete_message") {
        const String messageId = String(static_cast<const char *>(doc["id"] | ""));
        if (!messageId.isEmpty()) chatDeleteMessageById(senderPubHex, messageId, "Message deleted by " + author);
        return;
    }

    if (innerKind == "pair_request") {
        hc12TouchDiscoveredPeer(author, senderPubHex);

        p2pPairRequestDiscoveredIdx = hc12FindDiscoveredByPubKeyHex(senderPubHex);
        p2pPairRequestPeerKey = senderPubHex;
        p2pPairRequestPeerName = author.isEmpty() ? String("Peer") : author;
        p2pPairRequestPeerIp = IPAddress((uint32_t)0);
        p2pPairRequestPeerPort = 0;
        p2pPairRequestPending = true;

        if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
            hc12AppendTerminal((String("[RADIO] pair_request from ") + p2pPairRequestPeerName + "\n").c_str());
            hc12AppendTerminal((String("[RADIO] pending pair key=") + p2pPairRequestPeerKey + "\n").c_str());
        }

        uiStatusLine = String("Radio pair request from ") + p2pPairRequestPeerName;
        if (lvglReady) {
            lvglSyncStatusLine();
            lvglRefreshChatPeerUi();
            lvglRefreshChatContactsUi();
        }
        return;
    }

    if (innerKind == "pair_accept") {
        hc12TouchDiscoveredPeer(author, senderPubHex);

        if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
            hc12AppendTerminal((String("[RADIO] pair_accept from ") + author + "\n").c_str());
        }

        const bool ok = p2pAddOrUpdateTrustedPeer(
            author.isEmpty() ? String("Peer") : author,
            senderPubHex,
            IPAddress((uint32_t)0),
            0
        );
        if (ok) currentChatPeerKey = senderPubHex;
        uiStatusLine = ok ? "Radio peer paired" : "Radio pair accept failed";
        if (lvglReady) {
            lvglSyncStatusLine();
            lvglRefreshChatPeerUi();
            lvglRefreshChatContactsUi();
        }
        return;
    }

    if (innerKind == "pair_reject") {
        if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
            hc12AppendTerminal((String("[RADIO] pair_reject from ") + author + "\n").c_str());
        }
        uiStatusLine = String("Radio pair rejected by ") + author;
        if (lvglReady) lvglSyncStatusLine();
        return;
    }

    if (innerKind == "delete_conversation") {
        chatApplyConversationDeletion(senderPubHex, "Conversation deleted by " + author);
        return;
    }

    if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
        hc12AppendTerminal((String("[RADIO] unknown encrypted kind=") + innerKind + "\n").c_str());
    }
}

void lvglHc12ToggleSetEvent(lv_event_t *e)
{
    (void)e;
    hc12InitIfNeeded();
    const bool setAsserted = !hc12SetIsAsserted();

    if (radioModuleType == RADIO_MODULE_HC12) {
        digitalWrite(hc12ActiveSetPin(), setAsserted ? LOW : HIGH);
    } else {
        if (setAsserted) hc12EnterAtMode();
        else hc12ExitAtMode();
    }

    uiDeferredFlags |= UI_DEFERRED_HC12_SETTLE_PENDING;
    if (setAsserted) uiDeferredFlags |= UI_DEFERRED_HC12_TARGET_ASSERTED;
    else uiDeferredFlags &= static_cast<uint8_t>(~UI_DEFERRED_HC12_TARGET_ASSERTED);

    hc12SettleUntilTick = static_cast<uint16_t>(millis()) +
                          static_cast<uint16_t>(setAsserted ? 120U : 80U);

    lvglRefreshHc12Ui();
}

void lvglHc12SendEvent(lv_event_t *e)
{
    (void)e;
    lv_obj_t *cmdTa = hc12CmdTaObj();
    if (!cmdTa) return;
    if ((uiDeferredFlags & UI_DEFERRED_HC12_SETTLE_PENDING) &&
        static_cast<int16_t>(static_cast<uint16_t>(millis()) - hc12SettleUntilTick) < 0) {
        lvglStatusPush("Radio settling...");
        return;
    }
    String line = lv_textarea_get_text(cmdTa);
    line.trim();
    if (line.isEmpty()) return;

    if (!hc12SetIsAsserted() && line.equalsIgnoreCase("/rping")) {
        const bool ok = hc12SendRadioPing();
        hc12AppendTerminal(ok ? "[RADIO] PING sent\n" : "[RADIO] PING send failed\n");
        lv_textarea_set_text(cmdTa, "");
        return;
    }
    
    hc12SendLine(line);
    lv_textarea_set_text(cmdTa, "");
}

static void hc12Service()
{
    hc12InitIfNeeded();
    if ((uiDeferredFlags & UI_DEFERRED_HC12_SETTLE_PENDING) &&
        static_cast<int16_t>(static_cast<uint16_t>(millis()) - hc12SettleUntilTick) >= 0) {
        const bool setAsserted = (uiDeferredFlags & UI_DEFERRED_HC12_TARGET_ASSERTED) != 0;
        uiDeferredFlags &= static_cast<uint8_t>(~UI_DEFERRED_HC12_SETTLE_PENDING);
        if (radioModuleType == RADIO_MODULE_HC12) {
            hc12AppendTerminal(setAsserted ? "[HC12] SET asserted, AT mode requested\n"
                                           : "[HC12] SET released, normal radio mode\n");
        } else {
            hc12AppendTerminal(setAsserted ? "[E220] CFG mode asserted\n"
                                           : "[E220] Normal mode restored\n");
        }
    }
    hc12DiscoveryService();
    while (Serial1.available() > 0) {
        const int ch = Serial1.read();
        if (ch < 0) break;
        if (ch == '\r') continue;
        if (hc12SetIsAsserted()) {
            char buf[2] = {static_cast<char>(ch), '\0'};
            hc12AppendTerminal(buf);
            continue;
        }
        if (uiScreen == UI_CONFIG_RADIO_RECEIVER_TEST) {
            radioReceiverTestHandleByte(static_cast<uint8_t>(ch));
            continue;
        }
        if (ch == '\n') {
            String &rxLine = hc12RadioLineBuffer();
            String line = rxLine;
            rxLine = "";
            line.trim();
            if (line.isEmpty()) continue;
            if (line.startsWith(HC12_RADIO_FRAME_PREFIX) && radioModuleCanCarryChat()) {
                hc12HandleIncomingRadioLine(line);
            } else if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
                hc12AppendTerminal((line + "\n").c_str());
            }
            continue;
        }
        String &rxLine = hc12RadioLineBuffer();
        const size_t maxRxLine = HC12_RADIO_MAX_LINE + strlen(HC12_RADIO_FRAME_PREFIX) + 8;

        if (rxLine.length() < maxRxLine) {
            rxLine += static_cast<char>(ch);
        } else {
            rxLine = "";
            if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
                hc12AppendTerminal("[RADIO] RX line overflow, dropped\n");
            }
        }
    }
}

static void screenshotService(bool touchDown)
{
    if ((uiDeferredFlags & UI_DEFERRED_SCREENSHOT_PENDING) == 0 || (uiDeferredFlags & UI_DEFERRED_SCREENSHOT_BUSY) != 0) return;
    if (touchDown || fsWriteBusy()) return;

    uiDeferredFlags &= static_cast<uint8_t>(~UI_DEFERRED_SCREENSHOT_PENDING);
    uiDeferredFlags |= UI_DEFERRED_SCREENSHOT_BUSY;

    if (!sdEnsureMounted(true)) {
        uiDeferredFlags &= static_cast<uint8_t>(~UI_DEFERRED_SCREENSHOT_BUSY);
        lvglStatusPush("Screenshot failed: no SD");
        return;
    }

    fsWriteBegin();
    String path;
    String err;
    const bool ok = captureScreenToJpeg(path, err);
    fsWriteEnd();
    uiDeferredFlags &= static_cast<uint8_t>(~UI_DEFERRED_SCREENSHOT_BUSY);

    if (!ok) lvglStatusPush("Screenshot error: " + (err.length() ? err : String("unknown")));
    else lvglStatusPush("Saved: " + path);
}

void lvglWifiPwdCancelEvent(lv_event_t *e)
{
    (void)e;
    lvglHideKeyboard();
    if (lvglWifiPwdModal) lv_obj_add_flag(lvglWifiPwdModal, LV_OBJ_FLAG_HIDDEN);
    lvglWifiPwdConnectPending = false;
    lvglWifiPendingSsid = "";
}

void lvglWifiPwdConnectEvent(lv_event_t *e)
{
    (void)e;
    if (!lvglWifiPwdTa || lvglWifiPendingSsid.isEmpty()) return;
    const char *pass = lv_textarea_get_text(lvglWifiPwdTa);
    if (lvglWifiPwdStatusLabel) lv_label_set_text(lvglWifiPwdStatusLabel, "Saving and connecting...");
    lvglWifiPwdConnectPending = true;
    lvglHideKeyboard();
    if (lvglWifiPwdModal) lv_obj_add_flag(lvglWifiPwdModal, LV_OBJ_FLAG_HIDDEN);
    startWifiConnect(lvglWifiPendingSsid, pass ? String(pass) : String(""));
    uiStatusLine = "Connecting to: " + lvglWifiPendingSsid;
    lvglSyncStatusLine();
}

void lvglWifiApShowToggleEvent(lv_event_t *e)
{
    lv_obj_t *btn = lv_event_get_target(e);
    if (!btn || !lvglWifiApPassTa) return;
    const bool makeVisible = lv_textarea_get_password_mode(lvglWifiApPassTa);
    lv_textarea_set_password_mode(lvglWifiApPassTa, makeVisible ? false : true);
    lvglConfigureCompactSingleLineTa(lvglWifiApPassTa, lv_obj_get_height(lvglWifiApPassTa) > 0 ? lv_obj_get_height(lvglWifiApPassTa) : 34);
    if (lvglWifiApPassShowBtnLabel) lv_label_set_text(lvglWifiApPassShowBtnLabel, makeVisible ? LV_SYMBOL_EYE_OPEN : LV_SYMBOL_EYE_CLOSE);
}

void lvglWifiPwdShowToggleEvent(lv_event_t *e)
{
    lv_obj_t *btn = lv_event_get_target(e);
    if (!btn || !lvglWifiPwdTa) return;
    const bool makeVisible = lv_textarea_get_password_mode(lvglWifiPwdTa);
    lv_textarea_set_password_mode(lvglWifiPwdTa, makeVisible ? false : true);
    lvglConfigureCompactSingleLineTa(lvglWifiPwdTa, lv_obj_get_height(lvglWifiPwdTa) > 0 ? lv_obj_get_height(lvglWifiPwdTa) : 34);
    if (lvglWifiPwdShowBtnLabel) lv_label_set_text(lvglWifiPwdShowBtnLabel, makeVisible ? LV_SYMBOL_EYE_OPEN : LV_SYMBOL_EYE_CLOSE);
}

static void lvglRefreshWifiPasswordDialogLayout()
{
    if (!lvglWifiPwdModal || !lv_obj_is_valid(lvglWifiPwdModal)) return;

    const bool compactLayout = (DISPLAY_WIDTH <= 240) || (DISPLAY_HEIGHT <= 320);
    const lv_coord_t modalW = compactLayout ? static_cast<lv_coord_t>(DISPLAY_WIDTH - 10) : static_cast<lv_coord_t>(DISPLAY_WIDTH - 24);
    const lv_coord_t inputH = compactLayout ? 24 : 26;
    const lv_coord_t iconBtn = compactLayout ? 24 : 26;
    const lv_coord_t actionBtnW = compactLayout ? 68 : 84;
    const lv_coord_t actionBtnH = compactLayout ? 26 : 30;
    const lv_coord_t actionGap = compactLayout ? 4 : 8;
    const lv_coord_t modalPad = compactLayout ? 6 : 10;

    lv_obj_set_width(lvglWifiPwdModal, modalW);
    lv_obj_set_style_pad_all(lvglWifiPwdModal, modalPad, 0);
    lv_obj_set_style_pad_row(lvglWifiPwdModal, compactLayout ? 6 : 8, 0);

    if (lvglWifiPwdSsidLabel && lv_obj_is_valid(lvglWifiPwdSsidLabel)) {
        lv_obj_set_width(lvglWifiPwdSsidLabel, lv_pct(100));
        lv_label_set_long_mode(lvglWifiPwdSsidLabel, compactLayout ? LV_LABEL_LONG_DOT : LV_LABEL_LONG_WRAP);
    }

    if (lvglWifiPwdInputRow && lv_obj_is_valid(lvglWifiPwdInputRow)) {
        lv_obj_set_style_pad_column(lvglWifiPwdInputRow, compactLayout ? 4 : 6, 0);
    }

    if (lvglWifiPwdTa && lv_obj_is_valid(lvglWifiPwdTa)) {
        const lv_coord_t inputW = max<lv_coord_t>(96, modalW - (modalPad * 2) - iconBtn - (compactLayout ? 4 : 6));
        lv_obj_set_size(lvglWifiPwdTa, inputW, inputH);
        lvglConfigureCompactSingleLineTa(lvglWifiPwdTa, inputH);
    }

    if (lvglWifiPwdShowBtn && lv_obj_is_valid(lvglWifiPwdShowBtn)) {
        lv_obj_set_size(lvglWifiPwdShowBtn, iconBtn, inputH);
    }

    if (lvglWifiPwdActionRow && lv_obj_is_valid(lvglWifiPwdActionRow)) {
        lv_obj_set_style_pad_column(lvglWifiPwdActionRow, actionGap, 0);
    }
    if (lvglWifiPwdCancelBtn && lv_obj_is_valid(lvglWifiPwdCancelBtn)) {
        lv_obj_set_size(lvglWifiPwdCancelBtn, actionBtnW, actionBtnH);
    }
    if (lvglWifiPwdSaveBtn && lv_obj_is_valid(lvglWifiPwdSaveBtn)) {
        lv_obj_set_size(lvglWifiPwdSaveBtn, actionBtnW, actionBtnH);
    }

    lv_obj_update_layout(lvglWifiPwdModal);

    if (lvglKeyboardVisible()) {
        const lv_coord_t keyboardH = (lvglKb && lv_obj_is_valid(lvglKb)) ? lv_obj_get_height(lvglKb) : 120;
        lv_obj_set_align(lvglWifiPwdModal, LV_ALIGN_BOTTOM_MID);
        lv_obj_set_y(lvglWifiPwdModal, -keyboardH - 4);
        return;
    }

    const lv_coord_t modalH = lv_obj_get_height(lvglWifiPwdModal);
    const lv_coord_t topInset = UI_TOP_BAR_H + (compactLayout ? 4 : 10);
    lv_coord_t targetY = compactLayout ? (UI_TOP_BAR_H + 8) : 60;

    targetY = max<lv_coord_t>(topInset, targetY);
    targetY = min<lv_coord_t>(targetY, DISPLAY_HEIGHT - modalH - 6);

    lv_obj_set_align(lvglWifiPwdModal, LV_ALIGN_TOP_MID);
    lv_obj_set_y(lvglWifiPwdModal, targetY);
}

void lvglMqttPassShowToggleEvent(lv_event_t *e)
{
    lv_obj_t *btn = lv_event_get_target(e);
    if (!btn || !lvglMqttPassTa) return;
    const bool makeVisible = lv_textarea_get_password_mode(lvglMqttPassTa);
    lv_textarea_set_password_mode(lvglMqttPassTa, makeVisible ? false : true);
    lvglConfigureCompactSingleLineTa(lvglMqttPassTa, lv_obj_get_height(lvglMqttPassTa) > 0 ? lv_obj_get_height(lvglMqttPassTa) : 34);
    if (lvglMqttPassShowBtnLabel) lv_label_set_text(lvglMqttPassShowBtnLabel, makeVisible ? LV_SYMBOL_EYE_OPEN : LV_SYMBOL_EYE_CLOSE);
}

void lvglOpenWifiPasswordDialog(const String &ssid)
{
    if (!lvglReady) return;
    if (!lvglWifiPwdModal) {
        lvglWifiPwdModal = lv_obj_create(lv_layer_top());
        if (!lvglWifiPwdModal) return;
        lv_obj_set_size(lvglWifiPwdModal, DISPLAY_WIDTH - 24, LV_SIZE_CONTENT);
        lv_obj_set_y(lvglWifiPwdModal, 60);
        lv_obj_set_align(lvglWifiPwdModal, LV_ALIGN_TOP_MID);
        lv_obj_set_style_bg_color(lvglWifiPwdModal, lv_color_hex(0x16212C), 0);
        lv_obj_set_style_border_color(lvglWifiPwdModal, lv_color_hex(0x5A6B7C), 0);
        lv_obj_set_style_border_width(lvglWifiPwdModal, 1, 0);
        lv_obj_set_style_radius(lvglWifiPwdModal, 12, 0);
        lv_obj_set_style_pad_all(lvglWifiPwdModal, 10, 0);
        lv_obj_set_style_pad_row(lvglWifiPwdModal, 8, 0);
        lv_obj_set_flex_flow(lvglWifiPwdModal, LV_FLEX_FLOW_COLUMN);
        lv_obj_clear_flag(lvglWifiPwdModal, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *title = lv_label_create(lvglWifiPwdModal);
        lv_label_set_text(title, "Connect to WiFi");
        lv_obj_set_style_text_color(title, lv_color_hex(0xE5ECF3), 0);

        lvglWifiPwdSsidLabel = lv_label_create(lvglWifiPwdModal);
        lv_obj_set_width(lvglWifiPwdSsidLabel, lv_pct(100));
        lv_label_set_long_mode(lvglWifiPwdSsidLabel, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_color(lvglWifiPwdSsidLabel, lv_color_hex(0xB7C4D1), 0);

        lvglWifiPwdInputRow = lv_obj_create(lvglWifiPwdModal);
        lv_obj_set_size(lvglWifiPwdInputRow, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(lvglWifiPwdInputRow, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(lvglWifiPwdInputRow, 0, 0);
        lv_obj_set_style_pad_all(lvglWifiPwdInputRow, 0, 0);
        lv_obj_set_style_pad_column(lvglWifiPwdInputRow, 6, 0);
        lv_obj_set_flex_flow(lvglWifiPwdInputRow, LV_FLEX_FLOW_ROW);
        lv_obj_clear_flag(lvglWifiPwdInputRow, LV_OBJ_FLAG_SCROLLABLE);

        lvglWifiPwdTa = lv_textarea_create(lvglWifiPwdInputRow);
        lv_obj_set_width(lvglWifiPwdTa, 160);
        lv_obj_set_height(lvglWifiPwdTa, 26);
        lv_textarea_set_max_length(lvglWifiPwdTa, 16);
        lvglConfigureSingleLinePasswordTa(lvglWifiPwdTa);
        lv_textarea_set_placeholder_text(lvglWifiPwdTa, "Password");
        lv_obj_add_event_cb(lvglWifiPwdTa, lvglTextAreaFocusEvent, LV_EVENT_FOCUSED, nullptr);

        lvglWifiPwdShowBtn = lv_btn_create(lvglWifiPwdInputRow);
        lv_obj_set_size(lvglWifiPwdShowBtn, 26, 26);
        lv_obj_set_style_radius(lvglWifiPwdShowBtn, 8, 0);
        lv_obj_set_style_border_width(lvglWifiPwdShowBtn, 0, 0);
        lv_obj_add_event_cb(lvglWifiPwdShowBtn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(lvglWifiPwdShowBtn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(lvglWifiPwdShowBtn, lvglWifiPwdShowToggleEvent, LV_EVENT_CLICKED, nullptr);
        lvglWifiPwdShowBtnLabel = lv_label_create(lvglWifiPwdShowBtn);
        lv_label_set_text(lvglWifiPwdShowBtnLabel, LV_SYMBOL_EYE_OPEN);
        lv_obj_center(lvglWifiPwdShowBtnLabel);
        lvglRegisterStyledButton(lvglWifiPwdShowBtn, lv_color_hex(0x3F4A57), true);

        lvglWifiPwdStatusLabel = lv_label_create(lvglWifiPwdModal);
        lv_obj_set_width(lvglWifiPwdStatusLabel, lv_pct(100));
        lv_label_set_long_mode(lvglWifiPwdStatusLabel, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_color(lvglWifiPwdStatusLabel, lv_color_hex(0xD8B36A), 0);

        lvglWifiPwdActionRow = lv_obj_create(lvglWifiPwdModal);
        lv_obj_set_size(lvglWifiPwdActionRow, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(lvglWifiPwdActionRow, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(lvglWifiPwdActionRow, 0, 0);
        lv_obj_set_style_pad_all(lvglWifiPwdActionRow, 0, 0);
        lv_obj_set_style_pad_column(lvglWifiPwdActionRow, 8, 0);
        lv_obj_set_flex_flow(lvglWifiPwdActionRow, LV_FLEX_FLOW_ROW);
        lv_obj_clear_flag(lvglWifiPwdActionRow, LV_OBJ_FLAG_SCROLLABLE);

        auto makeActionBtn = [&](lv_obj_t **slot, const char *txt, lv_color_t color, lv_event_cb_t cb) {
            lv_obj_t *btn = lv_btn_create(lvglWifiPwdActionRow);
            if (!btn) return;
            lv_obj_set_size(btn, 84, 30);
            lv_obj_set_style_radius(btn, 8, 0);
            lv_obj_set_style_border_width(btn, 0, 0);
            lv_obj_add_event_cb(btn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
            lv_obj_add_event_cb(btn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
            lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, nullptr);
            lv_obj_t *lbl = lv_label_create(btn);
            if (lbl) {
                lv_label_set_text(lbl, txt);
                lv_obj_center(lbl);
            }
            lvglRegisterStyledButton(btn, color, true);
            if (slot) *slot = btn;
        };
        makeActionBtn(&lvglWifiPwdCancelBtn, "Cancel", lv_color_hex(0x4E5D6C), lvglWifiPwdCancelEvent);
        makeActionBtn(&lvglWifiPwdSaveBtn, "Save", lv_color_hex(0x357A38), lvglWifiPwdConnectEvent);
    }

    lvglWifiPendingSsid = ssid;
    lvglWifiPwdConnectPending = false;
    if (lvglWifiPwdSsidLabel) lv_label_set_text_fmt(lvglWifiPwdSsidLabel, "SSID: %s", ssid.c_str());
    if (lvglWifiPwdTa) {
        lv_textarea_set_text(lvglWifiPwdTa, "");
        lvglApplySingleLinePasswordTaState(lvglWifiPwdTa);
    }
    if (lvglWifiPwdShowBtnLabel) lv_label_set_text(lvglWifiPwdShowBtnLabel, LV_SYMBOL_EYE_OPEN);
    if (lvglWifiPwdStatusLabel) lv_label_set_text(lvglWifiPwdStatusLabel, "");
    lv_obj_move_foreground(lvglWifiPwdModal);
    lv_obj_clear_flag(lvglWifiPwdModal, LV_OBJ_FLAG_HIDDEN);
    lvglRefreshWifiPasswordDialogLayout();
    if (lvglWifiPwdTa) {
        lv_event_send(lvglWifiPwdTa, LV_EVENT_FOCUSED, nullptr);
    }
}

void lvglP2pPairPromptEvent(lv_event_t *e)
{
    lv_obj_t *msgbox = lv_event_get_current_target(e);
    const char *txt = lv_msgbox_get_active_btn_text(msgbox);
    const bool accepted = txt && strcmp(txt, "Accept") == 0;

    const bool hasStoredPeer = !p2pPairRequestPeerKey.isEmpty();
    const bool isRadioPeer =
        hasStoredPeer &&
        (p2pPairRequestPeerIp == IPAddress((uint32_t)0) || p2pPairRequestPeerPort == 0);

    bool ok = false;

    if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
        hc12AppendTerminal((String("[RADIO] pair prompt action=") + (accepted ? "Accept" : "Reject") + "\n").c_str());
        hc12AppendTerminal((String("[RADIO] pair prompt key=") + p2pPairRequestPeerKey + "\n").c_str());
        hc12AppendTerminal((String("[RADIO] pair prompt name=") + p2pPairRequestPeerName + "\n").c_str());
    }

    if (accepted) {
        if (hasStoredPeer) {
            ok = p2pAddOrUpdateTrustedPeer(
                p2pPairRequestPeerName.isEmpty() ? String("Peer") : p2pPairRequestPeerName,
                p2pPairRequestPeerKey,
                p2pPairRequestPeerIp,
                p2pPairRequestPeerPort
            );

            if (isRadioPeer) {
                if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
                    hc12AppendTerminal((String("[RADIO] sending pair response to key=") + p2pPairRequestPeerKey + "\n").c_str());
                }
                hc12SendPairResponse(p2pPairRequestPeerKey, ok);
            } else {
                p2pSendPairResponse(
                    p2pPairRequestPeerName,
                    p2pPairRequestPeerKey,
                    p2pPairRequestPeerIp,
                    p2pPairRequestPeerPort,
                    ok
                );
            }

            if (ok) currentChatPeerKey = p2pPairRequestPeerKey;
        }

        uiStatusLine = ok ? "Peer paired" : "Pair accept failed";
        if (lvglReady) {
            lvglSyncStatusLine();
            lvglRefreshChatPeerUi();
            lvglRefreshChatContactsUi();
        }
    } else {
        if (hasStoredPeer) {
            if (isRadioPeer) {
                if (uiScreen == UI_CONFIG_HC12_TERMINAL) {
                    hc12AppendTerminal((String("[RADIO] sending pair reject to key=") + p2pPairRequestPeerKey + "\n").c_str());
                }
                hc12SendPairResponse(p2pPairRequestPeerKey, false);
            } else {
                p2pSendPairResponse(
                    p2pPairRequestPeerName,
                    p2pPairRequestPeerKey,
                    p2pPairRequestPeerIp,
                    p2pPairRequestPeerPort,
                    false
                );
            }
        }

        uiStatusLine = "Pair request rejected";
        if (lvglReady) lvglSyncStatusLine();
    }

    p2pPairRequestPending = false;
    p2pPairPromptVisible = false;
    p2pPairRequestDiscoveredIdx = -1;
    p2pPairRequestPeerKey = "";
    p2pPairRequestPeerName = "";
    p2pPairRequestPeerIp = IPAddress((uint32_t)0);
    p2pPairRequestPeerPort = 0;

    lv_msgbox_close(msgbox);
}

void lvglOtaAvailablePromptEvent(lv_event_t *e)
{
    lv_obj_t *msgbox = lv_event_get_current_target(e);
    const char *txt = lv_msgbox_get_active_btn_text(msgbox);
    const bool updateNow = txt && strcmp(txt, "Update") == 0;
    otaUpdatePromptPending = false;
    lvglOtaUpdatePromptVisible = false;
    if (updateNow) {
        lvglEnsureScreenBuilt(UI_CONFIG_OTA);
        lvglRefreshOtaUi();
        lvglOpenScreen(UI_CONFIG_OTA, LV_SCR_LOAD_ANIM_MOVE_LEFT);
        lvglOtaUpdateEvent(nullptr);
    }
    lv_msgbox_close(msgbox);
}

void lvglOtaPopupEvent(lv_event_t *e)
{
    (void)e;
    otaClearPopupVersion();
    lvglHideOtaPostUpdatePopup();
}

void lvglScreenshotEvent(lv_event_t *e)
{
    (void)e;
    if (uiDeferredFlags & (UI_DEFERRED_SCREENSHOT_PENDING | UI_DEFERRED_SCREENSHOT_BUSY)) {
        lvglStatusPush("Screenshot already queued");
        return;
    }
    uiDeferredFlags |= UI_DEFERRED_SCREENSHOT_PENDING;
    lvglStatusPush("Screenshot queued");
}

// Snake, Snake 3D, and Tetris game UI/draw logic moved to dedicated game include files.

void lvglOpenMqttCfgEvent(lv_event_t *e)
{
    if (e) {
        lv_obj_t *btn = lv_event_get_current_target(e);
        if (btn && lv_obj_is_valid(btn)) {
            lv_obj_clear_state(btn, LV_STATE_CHECKED | LV_STATE_PRESSED);
        }
    }
    lvglEnsureScreenBuilt(UI_CONFIG_MQTT_CONFIG);
    lvglRefreshMqttConfigUi();
    lvglOpenScreen(UI_CONFIG_MQTT_CONFIG, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglOpenMqttCtrlEvent(lv_event_t *e)
{
    (void)e;
    if (lvglClickSuppressed() || lvglReorderDrag.active) return;
    lvglEnsureScreenBuilt(UI_CONFIG_MQTT_CONTROLS);
    lvglRefreshMqttControlsUi();
    lvglOpenScreen(UI_CONFIG_MQTT_CONTROLS, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglOpenMqttButtonConfigScreenEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_CONFIG_MQTT_BUTTONS);
    lvglRefreshMqttButtonConfigUi();
    lvglOpenScreen(UI_CONFIG_MQTT_BUTTONS, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglOpenOtaScreenEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_CONFIG_OTA);
    lvglRefreshOtaUi();
    lvglOpenScreen(UI_CONFIG_OTA, LV_SCR_LOAD_ANIM_MOVE_LEFT);
    otaCheckRequested = true;
}

void lvglOpenModulesScreenEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_CONFIG_MODULES);
    lvglOpenScreen(UI_CONFIG_MODULES, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglOpenHc12ScreenEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_CONFIG_HC12);
    lvglOpenScreen(UI_CONFIG_HC12, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglOpenHc12TerminalEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_CONFIG_HC12_TERMINAL);
    lvglRefreshHc12Ui();
    lvglOpenScreen(UI_CONFIG_HC12_TERMINAL, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglOpenHc12InfoEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_CONFIG_HC12_INFO);
    lvglOpenScreen(UI_CONFIG_HC12_INFO, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

#include "app/ui_style_config.inc"
#include "app/ota_ui.inc"
#include "app/screensaver.inc"
void lvglSetConfigKeyboardVisible(bool visible)
{
    const lv_coord_t keyboardPad = visible ? 132 : 10;

    if (lvglConfigWrap && lv_obj_is_valid(lvglConfigWrap)) {
        lv_obj_set_style_pad_bottom(lvglConfigWrap, keyboardPad, 0);
    }

    lv_obj_t *hc12TerminalTa = hc12TerminalObj();
    if (hc12TerminalTa && lv_obj_is_valid(hc12TerminalTa)) {
        lv_obj_t *hc12Wrap = lv_obj_get_parent(hc12TerminalTa);
        if (hc12Wrap && lv_obj_is_valid(hc12Wrap)) {
            lv_obj_set_style_pad_bottom(hc12Wrap, keyboardPad, 0);
        }

        lv_obj_set_height(hc12TerminalTa, visible ? (UI_CONTENT_H - 176) : (UI_CONTENT_H - 108));
    }

    if (visible) {
        if (lvglKeyboardFocusedTa && lv_obj_is_valid(lvglKeyboardFocusedTa)) {
            lv_obj_scroll_to_view_recursive(lvglKeyboardFocusedTa, LV_ANIM_ON);
        }
    } else if (uiScreen == UI_CONFIG_HC12_TERMINAL &&
               hc12TerminalTa && lv_obj_is_valid(hc12TerminalTa)) {
        lv_obj_t *hc12Wrap = lv_obj_get_parent(hc12TerminalTa);
        if (hc12Wrap && lv_obj_is_valid(hc12Wrap)) {
            lv_anim_del(hc12Wrap, nullptr);
            lv_obj_scroll_to_y(hc12Wrap, 0, LV_ANIM_OFF);
        }
    }
}

void lvglBuildUi()
{
    lvglEnsureScreenBuilt(UI_HOME);
    lvglRefreshTopIndicators();
    if (screenLockShouldGateUi()) {
        lvglEnsureScreenBuilt(UI_LOCK);
        uiScreen = UI_LOCK;
        lvglRefreshLockScreenUi();
        lv_scr_load(lvglScrLock);
    } else {
        uiScreen = UI_HOME;
        lv_scr_load(lvglScrHome);
    }
}

void lvglInitUi()
{
    static_assert(LV_COLOR_DEPTH == 16, "LVGL color depth must be 16 for TFT_eSPI flush path");
    lvglReady = false;
    if (!lvglEnsureUiRegistryStorage()) {
        Serial.println("[LVGL] registry alloc failed");
        return;
    }
    if (VERBOSE_SERIAL_DEBUG) {
        Serial.printf("[LVGL] cfg mem=%uB color_depth=%u free_heap=%u largest_8bit=%u\n",
                      static_cast<unsigned int>(LV_MEM_SIZE),
                      static_cast<unsigned int>(LV_COLOR_DEPTH),
                      static_cast<unsigned int>(ESP.getFreeHeap()),
                      static_cast<unsigned int>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
        Serial.println("[LVGL] step lv_init");
    }
    lv_init();
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[LVGL] step lv_init done");
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[LVGL] step read tft size");
    const uint32_t horRes = static_cast<uint32_t>(tft.width());
    const uint32_t verRes = static_cast<uint32_t>(tft.height());
    if (VERBOSE_SERIAL_DEBUG) {
        Serial.printf("[LVGL] tft size %ux%u\n",
                      static_cast<unsigned int>(horRes),
                      static_cast<unsigned int>(verRes));
    }
    if (horRes == 0 || verRes == 0) {
        Serial.println("[LVGL] invalid TFT resolution");
        return;
    }

    if (!lvglDrawPixels) {
        uint16_t lines = static_cast<uint16_t>((verRes + 9U) / 10U);
        if (boardHasUsablePsram()) {
            /* 20% buffer on PSRAM-capable boards for smoother regions and animation. */
            const uint16_t psramLines = static_cast<uint16_t>((verRes + 4U) / 5U);
            lines = max(lines, psramLines);
        }
        if (lines < LVGL_BUF_LINES_MIN) lines = LVGL_BUF_LINES_MIN;
        if (lines > LVGL_BUF_LINES_MAX) lines = LVGL_BUF_LINES_MAX;
        while (lines >= 2) {
            const size_t pxCount = static_cast<size_t>(horRes) * static_cast<size_t>(lines);
            const size_t allocBytes = pxCount * sizeof(lv_color_t);
            if (VERBOSE_SERIAL_DEBUG) {
                Serial.printf("[LVGL] alloc try lines=%u bytes=%u\n",
                              static_cast<unsigned int>(lines),
                              static_cast<unsigned int>(allocBytes));
            }
#if defined(BOARD_ESP32S3_3248S035_N16R8)
            lv_color_t *candidate = static_cast<lv_color_t *>(allocPreferPsram(allocBytes,
                MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
#else
            lv_color_t *candidate = static_cast<lv_color_t *>(malloc(allocBytes));
#endif
            if (candidate) {
                lvglDrawPixels = candidate;
                lvglBufLinesActive = lines;
                if (VERBOSE_SERIAL_DEBUG) {
                    Serial.printf("[LVGL] alloc ok lines=%u ptr=%p psram=%d\n",
                                  static_cast<unsigned int>(lines),
                                  static_cast<void *>(candidate),
                                  boardHasUsablePsram() && esp_ptr_external_ram(candidate) ? 1 : 0);
                }
                break;
            }
            if (VERBOSE_SERIAL_DEBUG) Serial.printf("[LVGL] alloc fail lines=%u\n", static_cast<unsigned int>(lines));
            lines /= 2;
        }
    }

    if (!lvglDrawPixels || lvglBufLinesActive == 0) {

        Serial.println("[LVGL] draw buffer allocation failed");
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.drawString("LVGL buffer alloc failed", 8, DISPLAY_CENTER_Y - 40, 2);
        return;
    }

    if (!lvglDrawPixels2 && boardHasUsablePsram()) {
        const size_t secondBufBytes = static_cast<size_t>(horRes) * static_cast<size_t>(lvglBufLinesActive) * sizeof(lv_color_t);
#if defined(BOARD_ESP32S3_3248S035_N16R8)
        lv_color_t *candidate2 = static_cast<lv_color_t *>(allocPreferPsram(secondBufBytes,
            MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
#else
        lv_color_t *candidate2 = static_cast<lv_color_t *>(malloc(secondBufBytes));
#endif
        if (candidate2) {
            lvglDrawPixels2 = candidate2;
            if (VERBOSE_SERIAL_DEBUG) {
                Serial.printf("[LVGL] alloc second buf ok lines=%u ptr=%p psram=%d\n",
                              static_cast<unsigned int>(lvglBufLinesActive),
                              static_cast<void *>(candidate2),
                              boardHasUsablePsram() && esp_ptr_external_ram(candidate2) ? 1 : 0);
            }
        } else if (VERBOSE_SERIAL_DEBUG) {
            Serial.printf("[LVGL] alloc second buf fail lines=%u\n", static_cast<unsigned int>(lvglBufLinesActive));
        }
    }

    if (VERBOSE_SERIAL_DEBUG) Serial.println("[LVGL] step draw_buf_init");
    lv_disp_draw_buf_init(
        &lvglDrawBuf,
        lvglDrawPixels,
        lvglDrawPixels2,
        static_cast<uint32_t>(horRes * static_cast<uint32_t>(lvglBufLinesActive))
    );
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[LVGL] step disp_drv_init");
    lv_disp_drv_init(&lvglDispDrv);
    lvglDispDrv.hor_res = static_cast<lv_coord_t>(horRes);
    lvglDispDrv.ver_res = static_cast<lv_coord_t>(verRes);
    lvglDispDrv.flush_cb = lvglFlushCb;
    lvglDispDrv.draw_buf = &lvglDrawBuf;
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[LVGL] step disp_drv_register");
    lv_disp_drv_register(&lvglDispDrv);

    if (VERBOSE_SERIAL_DEBUG) Serial.println("[LVGL] step indev_drv_init");
    lv_indev_drv_init(&lvglIndevDrv);
    lvglIndevDrv.type = LV_INDEV_TYPE_POINTER;
    lvglIndevDrv.read_cb = lvglTouchReadCb;
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[LVGL] step indev_drv_register");
    lvglTouchIndev = lv_indev_drv_register(&lvglIndevDrv);
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[LVGL] step ensure top bar");
    lvglEnsurePersistentTopBar();

    lvglLastTickMs = millis();
    lvglNextHandlerDueMs = 0;
    lvglLastInfoRefreshMs = 0;
    lvglLastStatusRefreshMs = 0;
    lvglLastMediaPlayerRefreshMs = 0;
    if (VERBOSE_SERIAL_DEBUG) {
        Serial.printf("[LVGL] init ok: %lux%lu, buf_lines=%u\n",
                      static_cast<unsigned long>(horRes),
                      static_cast<unsigned long>(verRes),
                      static_cast<unsigned int>(lvglBufLinesActive));
        Serial.printf("[LVGL] pre-ui free_heap=%u largest_8bit=%u\n",
                      static_cast<unsigned int>(ESP.getFreeHeap()),
                      static_cast<unsigned int>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
    }
    lvglBuildUi();
    if (VERBOSE_SERIAL_DEBUG) {
        Serial.printf("[LVGL] post-ui free_heap=%u largest_8bit=%u\n",
                      static_cast<unsigned int>(ESP.getFreeHeap()),
                      static_cast<unsigned int>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
    }
    lvglReady = true;
}

void lvglService()
{
    if (!lvglReady) return;
    if (!screensaverActive && otaPostUpdatePopupPending && !lvglOtaPostUpdatePopupVisible) {
        lvglShowOtaPostUpdatePopup();
    }
    if (!screensaverActive && otaUpdatePromptPending && !lvglOtaUpdatePromptVisible && !otaPostUpdatePopupPending &&
        otaUiState != OTA_UI_DOWNLOADING && otaUiState != OTA_UI_FINALIZING) {
        static const char *btns[] = {"Cancel", "Update", ""};
        String prompt = String("Firmware ") + (otaLatestVersion[0] ? otaLatestVersion : "?") + " is available.";
        lv_obj_t *m = lv_msgbox_create(nullptr, "Update Available", prompt.c_str(), btns, false);
        if (m) {
            lvglApplyMsgboxModalStyle(m);
            lv_obj_center(m);
            lv_obj_set_width(m, min<int16_t>(DISPLAY_WIDTH - 24, 280));
            lv_obj_add_event_cb(m, lvglOtaAvailablePromptEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lvglOtaUpdatePromptVisible = true;
        }
    }
    if (!screensaverActive && p2pPairRequestPending && !p2pPairPromptVisible) {
        static const char *btns[] = {"Reject", "Accept", ""};
        String prompt = "A device would like to pair.";
        if (p2pPairRequestDiscoveredIdx >= 0 && p2pPairRequestDiscoveredIdx < p2pDiscoveredCount) {
            const String &name = p2pDiscoveredPeers[p2pPairRequestDiscoveredIdx].name;
            if (!name.isEmpty()) prompt = name + " would like to pair.";
        }
        lv_obj_t *m = lv_msgbox_create(nullptr, "Pair Request", prompt.c_str(), btns, false);
        if (m) {
            lvglApplyMsgboxModalStyle(m);
            lv_obj_center(m);
            lv_obj_set_width(m, min<int16_t>(DISPLAY_WIDTH - 24, 260));
            lv_obj_add_event_cb(m, lvglP2pPairPromptEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            p2pPairPromptVisible = true;
        } else {
            p2pPairRequestPending = false;
            p2pPairRequestDiscoveredIdx = -1;
            p2pPairRequestPeerKey = "";
            p2pPairRequestPeerName = "";
            p2pPairRequestPeerIp = IPAddress((uint32_t)0);
            p2pPairRequestPeerPort = 0;
        }
    }
    if (lvglMediaRefreshPending) {
        lvglMediaRefreshPending = false;
        lvglRefreshMediaList();
    }

    const unsigned long now = millis();

    if (wifiScanInProgress && lvglWifiScanLabel && (now - wifiScanAnimLastMs) >= 320UL) {
        wifiScanAnimLastMs = now;
        wifiScanAnimPhase = static_cast<uint8_t>((wifiScanAnimPhase + 1U) & 0x03U);
        static const char *const kWifiScanFrames[] = {
            "Searching",
            "Searching.",
            "Searching..",
            "Searching..."
        };
        lv_label_set_text(lvglWifiScanLabel, kWifiScanFrames[wifiScanAnimPhase]);
    }

    unsigned long delta = now - lvglLastTickMs;
    if (delta > 0) {
        lv_tick_inc(delta);
        lvglLastTickMs = now;
    }

    const bool uiInteractive = uiPerformancePriorityActive(lvglTouchDown);
    if (!uiInteractive && lvglNextHandlerDueMs != 0 && static_cast<long>(now - lvglNextHandlerDueMs) < 0) return;

    const uint32_t nextHandlerInMs = lv_timer_handler();

    infoFpsFrameCount++;
    if (static_cast<long>(now - infoFpsLastMs) >= 1000) {
        infoFpsValue = infoFpsFrameCount;
        infoFpsFrameCount = 0;
        infoFpsLastMs = now;
    }

    if (uiInteractive) {
        lvglNextHandlerDueMs = now + 1UL;
    } else {
        uint32_t delayMs = nextHandlerInMs;
        const uint32_t maxDelayMs = displayAwake ? 8U : 12U;
        if (delayMs < 2U) delayMs = 2U;
        if (delayMs > maxDelayMs) delayMs = maxDelayMs;
        lvglNextHandlerDueMs = now + delayMs;
    }

    if (lvglTouchDown) {
        if (!lvglSwipeTracking) {
            const UiScreen activeScreen = lvglUiForScreenObj(lv_scr_act());
            if (activeScreen != uiScreen) uiScreen = activeScreen;
            const bool forceSwipeBackScreen = (uiScreen == UI_CONFIG_STYLE_HOME_ITEMS);
            lvglSwipeTracking = true;
            lvglSwipeStartX = lvglLastTouchX;
            lvglSwipeStartY = lvglLastTouchY;
            lvglSwipeLastX = lvglLastTouchX;
            lvglSwipeLastY = lvglLastTouchY;
            lvglSwipeStartMs = now;
            lvglSwipeSourceScreen = uiScreenSupportsSwipeBack(activeScreen) ? activeScreen : uiScreen;
            lvglSwipeCandidate = uiScreenSupportsSwipeBack(lvglSwipeSourceScreen) &&
                                 (forceSwipeBackScreen || !lvglGestureBlocked) &&
                                 (forceSwipeBackScreen || !lvglTouchOwnsHorizontalGesture());
            lvglSwipeHorizontalLocked = false;
            lvglSwipePressCancelled = false;
        } else {
            const bool forceSwipeBackScreen = (uiScreen == UI_CONFIG_STYLE_HOME_ITEMS);
            lvglSwipeLastX = lvglLastTouchX;
            lvglSwipeLastY = lvglLastTouchY;

            if (lvglGestureBlocked && !forceSwipeBackScreen) {
                lvglSwipeCandidate = false;
                lvglSwipeHorizontalLocked = false;
            }

            if (lvglSwipeCandidate) {
                const bool forceSwipeBackScreen = (uiScreen == UI_CONFIG_STYLE_HOME_ITEMS);
                const int swipeLockMinDx = forceSwipeBackScreen ? 18 : SWIPE_LOCK_MIN_DX;
                const int swipeBackMaxDy = forceSwipeBackScreen ? 72 : SWIPE_BACK_MAX_DY;
                const int dx = static_cast<int>(lvglSwipeLastX) - static_cast<int>(lvglSwipeStartX);
                const int dy = static_cast<int>(lvglSwipeLastY) - static_cast<int>(lvglSwipeStartY);
                const int absDx = abs(dx);
                const int absDy = abs(dy);

                if (!lvglSwipeHorizontalLocked) {
                    if (absDy >= SWIPE_CANCEL_VERTICAL_DY && absDy > absDx) {
                        lvglSwipeCandidate = false;
                    } else if (dx >= swipeLockMinDx && dx > ((absDy * 3) / 2)) {
                        lvglSwipeHorizontalLocked = true;
                        lvglSuppressClicksAfterGesture();

                        if (!lvglSwipePressCancelled && lvglTouchIndev) {
                            lvglSwipePressCancelled = true;
                            lv_indev_reset(lvglTouchIndev, nullptr);
                        }
                    }
                } else if (absDy > swipeBackMaxDy) {
                    lvglSwipeCandidate = false;
                    lvglSwipeHorizontalLocked = false;
                }

                if (lvglSwipeCandidate && lvglSwipeHorizontalLocked) {
                    lvglApplySwipeBackVisual(lvglClampSwipeBackOffset(dx));
                }
            }

            if ((!lvglSwipeCandidate || !lvglSwipeHorizontalLocked) &&
                lvglSwipeVisualActive && !lvglSwipeVisualAnimating) {
                lvglResetSwipeBackVisualState(true);
            }
        }
    } else if (lvglSwipeTracking) {
        lvglSwipeTracking = false;

        const int dx = static_cast<int>(lvglSwipeLastX) - static_cast<int>(lvglSwipeStartX);
        const int dy = static_cast<int>(lvglSwipeLastY) - static_cast<int>(lvglSwipeStartY);
        const bool forceSwipeBackScreen = (uiScreen == UI_CONFIG_STYLE_HOME_ITEMS);
        const int swipeLockMinDx = forceSwipeBackScreen ? 18 : SWIPE_LOCK_MIN_DX;
        const int swipeBackMinDx = forceSwipeBackScreen ? 24 : SWIPE_BACK_MIN_DX;
        const int swipeBackMaxDy = forceSwipeBackScreen ? 72 : SWIPE_BACK_MAX_DY;
        const unsigned long dt = static_cast<unsigned long>(now - lvglSwipeStartMs);
        const bool hadSwipeVisual = lvglSwipeVisualActive;
        const int completionDx = max<int>(1, (DISPLAY_WIDTH * SWIPE_BACK_COMPLETE_PERCENT) / 100);
        const bool passedCompletionThreshold = hadSwipeVisual && dx >= completionDx;

        const bool suppressClickFromSwipeAttempt =
            hadSwipeVisual ||
            lvglSwipePressCancelled ||
            (lvglSwipeCandidate && lvglSwipeHorizontalLocked) ||
            (dx >= swipeLockMinDx && dx > ((abs(dy) * 3) / 2));

        if (suppressClickFromSwipeAttempt) {
            lvglSuppressClicksAfterGesture();
        }

        const bool tapCandidate = displayAwake &&
                                  !lvglKeyboardVisible() &&
                                  abs(dx) <= DOUBLE_TAP_MAX_MOVE &&
                                  abs(dy) <= DOUBLE_TAP_MAX_MOVE &&
                                  dt <= DOUBLE_TAP_MAX_TAP_MS &&
                                  !lvglSwipePressCancelled;

        const bool swipeComplete =
            lvglSwipeCandidate &&
            lvglSwipeHorizontalLocked &&
            abs(dy) <= swipeBackMaxDy &&
            dx > ((abs(dy) * 3) / 2) &&
            ((dx >= swipeBackMinDx && dt <= SWIPE_BACK_MAX_MS) || passedCompletionThreshold);

        if (swipeComplete) {
            lvglSuppressClicksAfterGesture();
            lvglLastTapReleaseMs = 0;

            if (lvglKb && !lv_obj_has_flag(lvglKb, LV_OBJ_FLAG_HIDDEN)) {
                if (hadSwipeVisual) lvglAnimateSwipeBackVisual(0, false);
                lvglHideKeyboard();
            } else {
                if (hadSwipeVisual) {
                    lvglAnimateSwipeBackVisual(DISPLAY_WIDTH, true);
                } else {
                    lvglNavigateBackBySwipe();
                }
            }
        } else if (tapCandidate) {
            if (hadSwipeVisual) lvglAnimateSwipeBackVisual(0, false);
            lvglLastTapReleaseMs = now;
            lvglLastTapReleaseX = lvglLastTouchX;
            lvglLastTapReleaseY = lvglLastTouchY;
        } else {
            if (hadSwipeVisual) lvglAnimateSwipeBackVisual(0, false);
            if (abs(dx) >= swipeLockMinDx || abs(dy) >= SWIPE_CANCEL_VERTICAL_DY || lvglSwipePressCancelled) {
                lvglSuppressClicksAfterGesture();
            }
            lvglLastTapReleaseMs = 0;
        }

        lvglSwipeCandidate = false;
        lvglSwipeHorizontalLocked = false;
        lvglSwipePressCancelled = false;
        if (!swipeComplete) lvglSwipeSourceScreen = UI_HOME;
    }

    if (!uiInteractive && (now - lvglLastStatusRefreshMs) >= 1200UL) {
        lvglLastStatusRefreshMs = now;
        lvglSyncStatusLine();
        lvglRefreshTopIndicators();
    }

    if (!uiInteractive && uiScreen == UI_LOCK && (now - screenLockLastUiRefreshMs) >= 500UL) {
        screenLockLastUiRefreshMs = now;
        lvglRefreshLockScreenUi();
    }

    if (!uiInteractive && uiScreen == UI_MEDIA && (now - lvglLastMediaPlayerRefreshMs) >= 650UL) {
        lvglLastMediaPlayerRefreshMs = now;
        lvglRefreshMediaPlayerUi();
    }

    if (!uiInteractive && uiScreen == UI_INFO && (now - lvglLastInfoRefreshMs) >= 2500UL) {
        lvglLastInfoRefreshMs = now;
        queueInfoRefresh(false);
        lvglRefreshInfoPanel(false);
    }
}


void drawLargeBatteryFrame(int16_t x, int16_t y, int16_t w, int16_t h)
{
    tft.drawRoundRect(x, y, w, h, 8, TFT_WHITE);
    const int16_t tipW = 8;
    const int16_t tipH = h / 3;
    tft.fillRect(x + w, y + (h - tipH) / 2, tipW, tipH, TFT_WHITE);
}

void drawChargingCableDecor(int16_t bx, int16_t by, int16_t bw, int16_t bh, uint16_t col)
{
    // Start at battery bottom-center.
    int16_t x = bx + (bw / 2);
    int16_t y = by + bh + 2;
    // Thick (~10px) gentle wave extending near screen bottom.
    const int16_t yLimit = tft.height() - 8;
    int seg = 0;
    while (y < yLimit) {
        int16_t nx = x + ((seg % 2 == 0) ? 2 : -2);
        int16_t ny = y + 10;
        if (ny > yLimit) ny = yLimit;
        // Draw 10 parallel strokes for ~10px cable width.
        for (int o = -5; o <= 4; o++) {
            tft.drawLine(x + o, y, nx + o, ny, col);
        }
        x = nx;
        y = ny;
        seg++;
    }
}

uint16_t scaleColor565(uint16_t color, uint8_t level)
{
    uint8_t r = (color >> 11) & 0x1F;
    uint8_t g = (color >> 5) & 0x3F;
    uint8_t b = color & 0x1F;
    r = static_cast<uint8_t>((static_cast<uint16_t>(r) * level) / 255);
    g = static_cast<uint8_t>((static_cast<uint16_t>(g) * level) / 255);
    b = static_cast<uint8_t>((static_cast<uint16_t>(b) * level) / 255);
    return static_cast<uint16_t>((r << 11) | (g << 5) | b);
}

bool animateChargingBeforeSleep()
{
    displayBacklightSet(TFT_BL_LEVEL_ON);
    const int16_t w = min<int16_t>(220, max<int16_t>(140, DISPLAY_WIDTH - 90));
    const int16_t h = min<int16_t>(100, max<int16_t>(70, DISPLAY_HEIGHT / 5));
    const int16_t x = (DISPLAY_WIDTH - w) / 2;
    const int16_t titleY = max<int16_t>(36, DISPLAY_HEIGHT / 5);
    const int16_t y = min<int16_t>(DISPLAY_HEIGHT - h - 36, titleY + 40);
    tft.fillScreen(TFT_BLACK);
    drawLargeBatteryFrame(x, y, w, h);
    drawChargingCableDecor(x, y, w, h, TFT_DARKGREY);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawCentreString("Charging", DISPLAY_CENTER_X, titleY, 4);

    auto interruptedByTouch = [&]() -> bool {
        int16_t tx = 0;
        int16_t ty = 0;
        if (!readScreenTouch(tx, ty)) return false;
        lastUserActivityMs = millis();
        displaySetAwake(true);
        return true;
    };

    for (uint8_t cycle = 0; cycle < CHARGE_ANIM_CYCLES; cycle++) {
        for (int p = 0; p <= 100; p += 3) {
            if (interruptedByTouch()) return false;
            tft.fillRect(x + 2, y + 2, w - 4, h - 4, TFT_BLACK);
            int innerW = ((w - 4) * p) / 100;
            tft.fillRoundRect(x + 2, y + 2, innerW, h - 4, 5, TFT_GREEN);
            drawChargingCableDecor(x, y, w, h, scaleColor565(TFT_GREEN, 180));
            for (uint8_t waitStep = 0; waitStep < 11; ++waitStep) {
                if (interruptedByTouch()) return false;
                delay(5);
            }
        }
    }
    return true;
}

void displayBacklightInit()
{
    if (!DISPLAY_BACKLIGHT_PWM_SUPPORTED) return;
    ledcSetup(TFT_BL_LEDC_CHANNEL, TFT_BL_LEDC_FREQ, TFT_BL_LEDC_RES);
    ledcAttachPin(TFT_BL, TFT_BL_LEDC_CHANNEL);
    displayBacklightSet(TFT_BL_LEVEL_OFF);
}

uint8_t displayBacklightLevelFromPercent(uint8_t percent)
{
    percent = static_cast<uint8_t>(constrain(percent, 5, 100));
    return static_cast<uint8_t>((static_cast<uint16_t>(TFT_BL_LEVEL_ON) * percent) / 100U);
}

void displayBacklightSet(uint8_t level)
{
    if (!DISPLAY_BACKLIGHT_PWM_SUPPORTED) return;
    ledcWrite(TFT_BL_LEDC_CHANNEL, level);
}

void displayBacklightFadeIn(uint16_t durationMs)
{
    const uint8_t steps = 16;
    const uint16_t stepDelay = max<uint16_t>(8, durationMs / steps);
    const uint8_t target = displayBacklightLevelFromPercent(displayBrightnessPercent);
    for (uint8_t i = 0; i <= steps; i++) {
        const uint8_t level = static_cast<uint8_t>((static_cast<uint16_t>(target) * i) / steps);
        displayBacklightSet(level);
        delay(stepDelay);
    }
}

#endif
