#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <SD.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <lvgl.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Audio.h>
#include <JPEGENC.h>
#include <HTTPClient.h>
#include <Update.h>
#include <WiFiClientSecure.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_sleep.h>
#include <esp_heap_caps.h>
#include <driver/gpio.h>
#include <math.h>
#include <new>
#include <string.h>
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <sodium.h>

#include "generated/recovery_browser_asset.h"

enum TouchControllerType : uint8_t {
    TOUCH_CTRL_CST820 = 0,
    TOUCH_CTRL_GT911 = 1,
};

static constexpr int DISPLAY_WIDTH = TFT_WIDTH;
static constexpr int DISPLAY_HEIGHT = TFT_HEIGHT;
static constexpr int DISPLAY_CENTER_X = DISPLAY_WIDTH / 2;
static constexpr int DISPLAY_CENTER_Y = DISPLAY_HEIGHT / 2;

// Sunton ESP32 panels share these board-level pins across the supported variants.
static constexpr int TOUCH_SDA = 33;
static constexpr int TOUCH_SCL = 32;
static constexpr int TOUCH_RST = 25;
static constexpr int TOUCH_IRQ = 21;
static constexpr uint32_t TOUCH_I2C_HZ = 400000U;
static constexpr bool TOUCH_USE_IRQ = false;
static constexpr unsigned long TOUCH_POLL_INTERVAL_MS = 6UL;
static constexpr uint8_t WAKE_TOUCH_RELEASE_STABLE_POLLS = 3;
static constexpr uint16_t TOUCH_REINIT_FAIL_THRESHOLD = 40;
static constexpr unsigned long TOUCH_REINIT_MIN_INTERVAL_MS = 8000UL;
static constexpr uint8_t TFT_ROTATION = 0;

#if defined(BOARD_ESP32_3248S035)
static constexpr TouchControllerType TOUCH_CONTROLLER = TOUCH_CTRL_GT911;
static constexpr uint8_t TOUCH_ROTATION_OFFSET = 0;
static constexpr const char *DEVICE_MODEL = "ESP32-3248S035 Touch Remote";
static constexpr const char *DEVICE_SHORT_NAME = "ESP32-3248S035";
static constexpr const char *AP_SSID = "ESP32-3248S035-FM";
static constexpr const char *MDNS_HOST = "esp32-3248s035";
#else
static constexpr TouchControllerType TOUCH_CONTROLLER = TOUCH_CTRL_CST820;
static constexpr uint8_t TOUCH_ROTATION_OFFSET = 0;
static constexpr const char *DEVICE_MODEL = "ESP32-2432S024C Touch Remote";
static constexpr const char *DEVICE_SHORT_NAME = "ESP32-2432S024C";
static constexpr const char *AP_SSID = "ESP32-2432S024C-FM";
static constexpr const char *MDNS_HOST = "esp32-2432s024c";
#endif

static constexpr uint8_t CST820_ADDR = 0x15;
static constexpr uint8_t GT911_ADDR_PRIMARY = 0x5D;
static constexpr uint8_t GT911_ADDR_SECONDARY = 0x14;
static constexpr uint16_t GT911_REG_PRODUCT_ID = 0x8140;
static constexpr uint16_t GT911_REG_STATUS = 0x814E;
static constexpr uint16_t GT911_REG_POINT1 = 0x814F;

// User-provided TF card pins.
static constexpr int SD_CS = 5;
static constexpr int SD_MOSI = 23;
static constexpr int SD_SCK = 18;
static constexpr int SD_MISO = 19;
static constexpr uint32_t SD_SPI_FREQ_HZ = 8000000U;
static constexpr uint32_t SD_SPI_FREQ_RECOVERY_HZ = 4000000U;
static constexpr uint32_t SD_SPI_FREQ_SAFE_HZ = 1000000U;
static constexpr unsigned long SD_RECOVERY_RETRY_DELAY_MS = 80;
static constexpr unsigned long SD_RECOVERY_MIN_INTERVAL_MS = 10000;
static constexpr unsigned long SD_AUTORETRY_PERIOD_MS = 30000;
static constexpr unsigned long SD_STATS_LOG_PERIOD_MS = 30000;
static constexpr const char *CHAT_LOG_DIR = "/Conversations";
static constexpr const char *CHAT_OUTBOX_PATH = "/Conversations/.outbox.txt";
static constexpr unsigned long CHAT_RETRY_INTERVAL_MS = 5000UL;
static constexpr int MAX_CHAT_PENDING = 6;
static constexpr uint32_t SD_BOOT_PROBE_FREQ_HZ = SD_SPI_FREQ_SAFE_HZ;
static constexpr unsigned long BOOT_DEFER_SD_MS = 80UL;
static constexpr unsigned long BOOT_DEFER_WIFI_MS = 220UL;
static_assert(TOUCH_SDA != SD_CS && TOUCH_SDA != SD_MOSI && TOUCH_SDA != SD_SCK && TOUCH_SDA != SD_MISO,
              "Pin conflict: TOUCH_SDA overlaps SD SPI pin");
static_assert(TOUCH_SCL != SD_CS && TOUCH_SCL != SD_MOSI && TOUCH_SCL != SD_SCK && TOUCH_SCL != SD_MISO,
              "Pin conflict: TOUCH_SCL overlaps SD SPI pin");
static_assert(TOUCH_RST != SD_CS && TOUCH_RST != SD_MOSI && TOUCH_RST != SD_SCK && TOUCH_RST != SD_MISO,
              "Pin conflict: TOUCH_RST overlaps SD SPI pin");
static_assert(TOUCH_IRQ != SD_CS && TOUCH_IRQ != SD_MOSI && TOUCH_IRQ != SD_SCK && TOUCH_IRQ != SD_MISO,
              "Pin conflict: TOUCH_IRQ overlaps SD SPI pin");

// User-provided speaker pin (SC8002B related, reserved for I2S usage).
static constexpr int I2S_SPK_PIN = 26;
static constexpr uint8_t AUDIO_I2S_PORT = I2S_NUM_0;
static constexpr uint8_t AUDIO_VOLUME_TARGET = 21;
static constexpr bool AUDIO_FORCE_MONO_INTERNAL_DAC = true;
// ESP32-audioI2S uses LEFT_EN (value 2) for DAC2 on GPIO26.
// Keep TOUCH_RST on GPIO25 untouched (DAC1 / RIGHT_EN must stay disabled).
static constexpr uint8_t AUDIO_INTERNAL_DAC_CHANNEL = static_cast<uint8_t>(I2S_DAC_CHANNEL_LEFT_EN);
static_assert(I2S_DAC_CHANNEL_LEFT_EN == 2, "Unexpected DAC enum mapping: GPIO26 output would be unsafe");
static constexpr int AUDIO_INPUT_RAM_BUFFER_BYTES = 2048;
static constexpr uint32_t AUDIO_PREEMPTIVE_NET_SUSPEND_LARGEST_8BIT = 28000U;
static constexpr unsigned long AUDIO_NETWORK_RESUME_RETRY_MS = 2000UL;
static constexpr uint32_t AUDIO_INIT_MIN_FREE_HEAP = 70000U;
static constexpr uint32_t AUDIO_INIT_MIN_DMA_BLOCK = 12000U;
static constexpr unsigned long AUDIO_INIT_RETRY_MS = 1200UL;
static constexpr uint32_t AUDIO_FLAC_MIN_FREE_HEAP = 120000U;
static constexpr uint32_t AUDIO_FLAC_MIN_LARGEST_8BIT = 45000U;
static constexpr int RGB_PIN_R = 4;
static constexpr int RGB_PIN_G = 17;
static constexpr int RGB_PIN_B = 16;
static constexpr bool RGB_ACTIVE_LOW = true;
static constexpr uint8_t TFT_BL_LEDC_CHANNEL = 0;
static constexpr uint16_t TFT_BL_LEDC_FREQ = 5000;
static constexpr uint8_t TFT_BL_LEDC_RES = 8;
static constexpr uint8_t TFT_BL_LEVEL_ON = 255;
static constexpr uint8_t TFT_BL_LEVEL_OFF = 0;
static constexpr unsigned long LCD_IDLE_TIMEOUT_MS = 120000;
static constexpr unsigned long SENSOR_SAMPLE_PERIOD_MS = 2000;
static constexpr unsigned long LIGHT_SLEEP_AFTER_IDLE_MS = 20000;
static constexpr bool LIGHT_SLEEP_TIMER_FALLBACK = false;
static constexpr uint64_t LIGHT_SLEEP_TIMER_US = 5000000ULL;
static constexpr int BATTERY_ADC_PIN = 35;
static constexpr int LIGHT_ADC_PIN = 34;
static constexpr float BATTERY_DIVIDER_R_TOP = 220000.0f;
static constexpr float BATTERY_DIVIDER_R_BOTTOM = 100000.0f;
static constexpr float BATTERY_CAL_FACTOR = 0.96f;
static constexpr float BATTERY_EMPTY_V = 3.30f;
static constexpr float BATTERY_FULL_V = 4.20f;
static constexpr int BATTERY_ADC_SAMPLES = 16;
static constexpr float BATTERY_FILTER_ALPHA = 0.12f;
static constexpr unsigned long BATTERY_SNAPSHOT_PERIOD_MS = 30000;
static constexpr unsigned long BATTERY_SNAPSHOT_FORCE_MS = 300000;
static constexpr float BATTERY_SNAPSHOT_MIN_DELTA_V = 0.010f;
static constexpr float BATTERY_BOOT_CHARGE_DELTA_V = 0.015f;
static constexpr unsigned long CHARGE_DETECT_INTERVAL_MS = 4000;
static constexpr float CHARGE_RISE_THRESHOLD_V = 0.003f;
static constexpr int8_t CHARGE_SCORE_ON = 1;
static constexpr int8_t CHARGE_SCORE_MAX = 6;
static constexpr unsigned long CHARGE_HOLD_MS = 120000;
static constexpr bool CHARGE_LOG_TO_SERIAL = false;
static constexpr uint8_t CHARGE_ANIM_CYCLES = 1;
static constexpr int LIGHT_ADC_SAMPLES = 8;
static constexpr float LIGHT_FILTER_ALPHA = 0.20f;
static constexpr bool LIGHT_INVERT = true;
static constexpr int LIGHT_MIN_SPAN_RAW = 80;
static constexpr uint16_t LIGHT_RAW_CAL_MIN = 0;
static constexpr uint16_t LIGHT_RAW_CAL_MAX = 600;
static constexpr bool LIGHT_LOG_RAW_TO_SERIAL = false;

static constexpr const char *AP_PASS = "12345678";
static constexpr const char *FW_VERSION = "0.1.2";
static constexpr unsigned long STA_RETRY_INTERVAL_MS = 5000UL;
static constexpr bool SERIAL_TERMINAL_TRANSFER_ENABLED = false;
static constexpr size_t SERIAL_LOG_RING_SIZE = 200;
static constexpr size_t SERIAL_LOG_LINE_MAX = 192;
static constexpr uint32_t SERIAL_LOG_RATE_MS_DEFAULT = 40;
static constexpr uint32_t WS_TELEMETRY_MIN_FREE_HEAP = 50000U;

void serialLogPushLine(const char *line, bool sendWs = true);
void executeSerialCommand(String input);
HardwareSerial &serialHw = ::Serial;

class MirroredSerialPort {
public:
    void begin(unsigned long baud)
    {
        serialHw.begin(baud);
    }

    size_t println()
    {
        if (SERIAL_TERMINAL_TRANSFER_ENABLED) serialLogPushLine("", true);
        return serialHw.println();
    }

    size_t println(const char *text)
    {
        if (SERIAL_TERMINAL_TRANSFER_ENABLED) serialLogPushLine(text ? text : "", true);
        return serialHw.println(text ? text : "");
    }

    size_t println(const String &text)
    {
        if (SERIAL_TERMINAL_TRANSFER_ENABLED) serialLogPushLine(text.c_str(), true);
        return serialHw.println(text);
    }

    size_t printf(const char *fmt, ...)
    {
        if (!fmt) return 0;
        char buf[384];
        va_list ap;
        va_start(ap, fmt);
        const int n = vsnprintf(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        if (n <= 0) return 0;
        buf[sizeof(buf) - 1] = '\0';
        char logBuf[sizeof(buf)];
        strncpy(logBuf, buf, sizeof(logBuf));
        logBuf[sizeof(logBuf) - 1] = '\0';
        size_t len = strlen(logBuf);
        while (len && (logBuf[len - 1] == '\n' || logBuf[len - 1] == '\r')) {
            logBuf[--len] = '\0';
        }
        if (SERIAL_TERMINAL_TRANSFER_ENABLED) serialLogPushLine(logBuf, true);
        return serialHw.print(buf);
    }
};

MirroredSerialPort serialMirrorPort;
#define Serial serialMirrorPort

struct FsUploadCtx {
    String destPath;
    String tmpPath;
    File file;
    String error;
    bool started = false;
    size_t bytesWritten = 0;
};

struct OtaUploadCtx {
    bool started = false;
    bool valid = false;
    bool finished = false;
    size_t bytesWritten = 0;
    String error;
};

TFT_eSPI tft;
SPIClass sdSpi(HSPI);
AsyncWebServer server(80);
AsyncWebSocket wsCarInput("/CarInput");
DNSServer dnsServer;
Audio *audio = nullptr;
WiFiClient mqttNetClient;
PubSubClient mqttClient(mqttNetClient);
WiFiUDP p2pUdp;
void loadMediaEntries();
void refreshWifiScan();
void wifiScanService();
void startWifiConnect(const String &ssid, const String &pass);
void saveStaCreds(const String &ssid, const String &pass);
void saveApCreds(const String &ssid, const String &pass);
void tryBootStaReconnect();
void sampleTopIndicators();
const char *authName(wifi_auth_mode_t auth);
String mediaResolvePlaybackPath(const String &originalPath);
String mediaDisplayNameFromPath(const String &path);
uint8_t audioVolumeLevelFromPercent(uint8_t percent);
void mediaFormatSeconds(uint32_t sec, char *out, size_t outLen);
bool audioStartFile(const String &path);
void audioStopPlayback(bool smooth);
void audioSetVolumeImmediate(uint8_t v);
bool audioEnsureBackendReady(const char *reason);
bool mediaPathIsFlac(const String &path);
bool audioFlacSupportedNow(uint32_t *freeHeapOut = nullptr, uint32_t *largestOut = nullptr);
wl_status_t wifiStatusSafe();
bool wifiConnectedSafe();
int32_t wifiRssiSafe();
String wifiSsidSafe();
String wifiIpSafe();
void registerWifiEvents();
void ensureApOnline(const char *reason);
void disableApWhenStaConnected(const char *reason);
static void stopDnsForAp();
void wifiConnectionService();
bool mediaStartTrack(const String &sourcePath, const String &displayName);
String mediaFindAdjacentTrack(const String &sourcePath, bool nextDir);
void lvglSetMediaPlayerVisible(bool visible);
void lvglRefreshMediaLayout();
void lvglRefreshMediaPlayerUi();
void lvglHideKeyboard();
void networkSuspendForAudio();
bool networkResumeAfterAudio();
void lvglNavigateBackBySwipe();
bool captureScreenToJpeg(String &savedPathOut, String &errorOut);
String mqttDefaultButtonName(int idx);
String mqttButtonPayloadForIndex(int idx);
String mediaBuildListLabel(const String &name, bool isDir, size_t sizeBytes);
void displaySetAwake(bool awake);
void displayBacklightInit();
void displayBacklightSet(uint8_t level);
void displayBacklightFadeIn(uint16_t durationMs = 220);
uint8_t displayBacklightLevelFromPercent(uint8_t percent);
void snakeResetGame();
void tetrisResetGame();
void tetrisMove(int dx);
void setupCarInputSocket();
void serviceCarInputTelemetry();
String normalizeSdRoutePath(const String &rawPath, bool allowEmptyRoot = false);
bool sdShouldHideSystemEntry(const String &name);
String makeUniquePath(const String &path);
String recycleMetaPathFor(const String &recyclePath);
String httpsGetText(const String &url, int *statusOut = nullptr);
String chooseLatestFirmwareBinUrl(const JsonVariantConst &assets);
bool otaDownloadAndApplyFromUrl(const String &url, String &errorOut);
void tetrisRotate();
void tetrisDrop();
bool tetrisCellFor(int type, int rot, int i, int &ox, int &oy);
uint8_t wifiQualityPercentFromRssi(int32_t rssi);
void mqttPublishAction(const char *action);
void mqttPublishButtonAction(int index);
void mqttService();
bool mqttConnectNow();
void mqttPublishDiscovery();
void mqttTrimBufferForIdle();
bool mqttPublishChatMessage(const String &text);
bool mqttPublishChatMessageWithId(const String &peerKey, const String &text, const String &messageId);
bool mqttPublishConversationDelete();
bool mqttPublishChatAck(const String &peerKey, const String &messageId);
void loadMqttConfig();
void saveMqttConfig();
void appendUiSettings(JsonDocument &doc);
bool handleUiSettingMessage(const char *msg);
void loadUiRuntimeConfig();
void applyAirplaneMode(bool enabled, const char *reason);
void lvglRefreshConfigUi();
void lvglSaveDeviceNameEvent(lv_event_t *e);
void lvglSetConfigKeyboardVisible(bool visible);
void cpuLoadService(uint32_t loopStartUs);
void loadP2pConfig();
void saveP2pConfig();
void p2pEnsureUdp();
void p2pService();
bool p2pSendChatMessage(const String &text);
bool p2pSendChatMessageWithId(const String &peerKey, const String &text, const String &messageId);
bool p2pSendConversationDelete();
bool p2pSendChatAck(const String &peerKey, const String &messageId);
String p2pPublicKeyHex();
static int p2pFindPeerByPubKeyHex(const String &pubKeyHex);
void p2pBroadcastDiscover();
bool p2pAddOrUpdateTrustedPeer(const String &name, const String &pubKeyHex, const IPAddress &ip, uint16_t port);
void setupWifiAndServer();
void lvglRefreshChatUi();
void lvglRefreshChatPeerUi();
void lvglSetChatKeyboardVisible(bool visible);
void lvglSyncStatusLine();
void lvglRefreshTopIndicators();
void chatReloadRecentMessagesFromSd(const String &peerKey);
void lvglRefreshChatContactsUi();
static String chatDisplayNameForPeerKey(const String &peerKey);
static void chatClearCache();
static String chatGenerateMessageId();
static bool chatQueueOutgoingMessage(const String &peerKey, const String &author, const String &text, const String &messageId);
static bool chatAckOutgoingMessage(const String &peerKey, const String &messageId);
static void chatPendingService();
static bool mqttHasHeapHeadroom(uint32_t freeMin, uint32_t largestMin, const char *status);
static bool mqttPublishEncryptedPayload(const String &peerKey, const uint8_t *plain, size_t plainLen);
void lvglRefreshChatLayout();
void lvglSetChatPeerScanButtonStatus(const char *text, uint32_t revertDelayMs = 0);
void lvglToggleChatMenuEvent(lv_event_t *e);
void lvglDeleteChatConversationEvent(lv_event_t *e);
void lvglDeleteChatConversationForAllEvent(lv_event_t *e);
void lvglMediaPrevPageEvent(lv_event_t *e);
void lvglMediaNextPageEvent(lv_event_t *e);
void lvglQueueMediaRefresh();
void setupSd();
bool sdEnsureMounted(bool forceRemount = false);
void sdMarkFault(const char *where);
void sdStatsService();
extern bool sdMounted;
extern bool sdSpiReady;
SemaphoreHandle_t sdMutex = nullptr;
portMUX_TYPE fsWriteMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t fsWriteOps = 0;

struct SdStats {
    uint32_t faultCount = 0;
    uint32_t remountAttempts = 0;
    uint32_t remountSuccess = 0;
    uint32_t remountFailure = 0;
    uint32_t remountTryFast = 0;
    uint32_t remountTryRecovery = 0;
    uint32_t remountTrySafe = 0;
    uint32_t forceRemountCalls = 0;
    uint32_t autoRetryCalls = 0;
    unsigned long lastFaultMs = 0;
    unsigned long lastRemountOkMs = 0;
    unsigned long lastRemountFailMs = 0;
    char lastFaultWhere[48] = "";
};

portMUX_TYPE sdStatsMux = portMUX_INITIALIZER_UNLOCKED;
SdStats sdStats;
unsigned long sdStatsLastLogMs = 0;
uint32_t sdStatsLastLoggedFaultCount = 0;
uint32_t sdStatsLastLoggedRemountAttempts = 0;
uint32_t sdStatsLastLoggedRemountSuccess = 0;
uint32_t sdStatsLastLoggedRemountFailure = 0;

static constexpr uint16_t LVGL_BUF_LINES = 24;
static constexpr uint16_t UI_ANIM_MS = 72;
static constexpr uint16_t UI_BUTTON_CLICK_FLASH_MS = 90;
static constexpr lv_opa_t UI_BUTTON_CLICK_FLASH_MIX = 64;
static constexpr int16_t UI_TOP_BAR_H = 30;
static constexpr int16_t UI_CONTENT_TOP_Y = UI_TOP_BAR_H;
static constexpr int16_t UI_CONTENT_H = DISPLAY_HEIGHT - UI_CONTENT_TOP_Y;
static constexpr uint8_t INFO_TEMP_BAR_MAX_C = 100;
static constexpr uint8_t INFO_TEMP_WARN_C = 65;
static constexpr uint8_t INFO_TEMP_HOT_C = 80;
static constexpr int16_t SWIPE_EDGE_START_MAX_X = 28;
static constexpr int16_t SWIPE_BACK_MIN_DX = 52;
static constexpr int16_t SWIPE_BACK_MAX_DY = 30;
static constexpr int16_t SWIPE_LOCK_MIN_DX = 18;
static constexpr int16_t SWIPE_CANCEL_VERTICAL_DY = 18;
static constexpr unsigned long SWIPE_BACK_MAX_MS = 550;
static constexpr int16_t DOUBLE_TAP_MAX_MOVE = 12;
static constexpr int16_t DOUBLE_TAP_MAX_GAP = 350;
static constexpr unsigned long DOUBLE_TAP_MAX_TAP_MS = 240;
static lv_color_t *lvglDrawPixels = nullptr;
static uint16_t lvglBufLinesActive = 0;
static lv_disp_draw_buf_t lvglDrawBuf;
static lv_disp_drv_t lvglDispDrv;
static lv_indev_drv_t lvglIndevDrv;
static lv_indev_t *lvglTouchIndev = nullptr;
static bool lvglReady = false;
static lv_obj_t *lvglScrHome = nullptr;
static lv_obj_t *lvglScrChat = nullptr;
static lv_obj_t *lvglScrChatPeers = nullptr;
static lv_obj_t *lvglScrWifi = nullptr;
static lv_obj_t *lvglScrMedia = nullptr;
static lv_obj_t *lvglScrInfo = nullptr;
static lv_obj_t *lvglScrGames = nullptr;
static lv_obj_t *lvglScrConfig = nullptr;
static lv_obj_t *lvglScrMqttCfg = nullptr;
static lv_obj_t *lvglScrMqttCtrl = nullptr;
static lv_obj_t *lvglScrSnake = nullptr;
static lv_obj_t *lvglScrTetris = nullptr;
static lv_obj_t *lvglStatusLabel = nullptr;
static lv_obj_t *lvglInfoList = nullptr;
static lv_obj_t *lvglInfoBatteryValueLabel = nullptr;
static lv_obj_t *lvglInfoBatterySubLabel = nullptr;
static lv_obj_t *lvglInfoWifiValueLabel = nullptr;
static lv_obj_t *lvglInfoWifiSubLabel = nullptr;
static lv_obj_t *lvglInfoLightValueLabel = nullptr;
static lv_obj_t *lvglInfoLightSubLabel = nullptr;
static lv_obj_t *lvglInfoCpuValueLabel = nullptr;
static lv_obj_t *lvglInfoCpuSubLabel = nullptr;
static lv_obj_t *lvglInfoCpuBar = nullptr;
static lv_obj_t *lvglInfoTempValueLabel = nullptr;
static lv_obj_t *lvglInfoTempSubLabel = nullptr;
static lv_obj_t *lvglInfoTempBar = nullptr;
static lv_obj_t *lvglInfoSystemLabel = nullptr;
static lv_obj_t *lvglWifiList = nullptr;
static lv_obj_t *lvglWifiScanLabel = nullptr;
static lv_obj_t *lvglWifiApSsidTa = nullptr;
static lv_obj_t *lvglWifiApPassTa = nullptr;
static lv_obj_t *lvglWifiApPassShowBtnLabel = nullptr;
static lv_obj_t *lvglWifiApPassShowBtn = nullptr;
static lv_obj_t *lvglWifiPwdModal = nullptr;
static lv_obj_t *lvglWifiPwdTa = nullptr;
static lv_obj_t *lvglWifiPwdSsidLabel = nullptr;
static lv_obj_t *lvglWifiPwdStatusLabel = nullptr;
static lv_obj_t *lvglWifiPwdShowBtnLabel = nullptr;
static lv_obj_t *lvglWifiPwdShowBtn = nullptr;
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
static lv_obj_t *lvglMqttChatTa = nullptr;
static lv_obj_t *lvglMqttBtnNameTa = nullptr;
static lv_obj_t *lvglMqttEnableSw = nullptr;
static lv_obj_t *lvglMqttCriticalSw = nullptr;
static lv_obj_t *lvglMqttCtrlList = nullptr;
static lv_obj_t *lvglChatList = nullptr;
static lv_obj_t *lvglChatInputTa = nullptr;
static lv_obj_t *lvglChatEmptyLabel = nullptr;
static lv_obj_t *lvglChatComposer = nullptr;
static lv_obj_t *lvglChatContacts = nullptr;
static lv_obj_t *lvglChatContactLabel = nullptr;
static lv_obj_t *lvglChatPeersBtn = nullptr;
static lv_obj_t *lvglChatMenuBtn = nullptr;
static lv_obj_t *lvglChatMenuBackdrop = nullptr;
static lv_obj_t *lvglChatMenuPanel = nullptr;
static lv_obj_t *lvglChatPeerList = nullptr;
static lv_obj_t *lvglChatPeerIdentityLabel = nullptr;
static lv_obj_t *lvglChatPeerScanBtn = nullptr;
static lv_obj_t *lvglAirplaneBtn = nullptr;
static lv_obj_t *lvglAirplaneBtnLabel = nullptr;
static lv_obj_t *lvglConfigWrap = nullptr;
static lv_obj_t *lvglConfigDeviceNameTa = nullptr;
static lv_obj_t *lvglBrightnessSlider = nullptr;
static lv_obj_t *lvglBrightnessValueLabel = nullptr;
static lv_obj_t *lvglKb = nullptr;
static lv_obj_t *lvglSnakeScoreLabel = nullptr;
static lv_obj_t *lvglTetrisScoreLabel = nullptr;
static lv_obj_t *lvglSnakeBoardObj = nullptr;
static lv_obj_t *lvglTetrisBoardObj = nullptr;
static lv_obj_t *lvglTopBarRoot = nullptr;
static constexpr uint8_t LVGL_MAX_TOP_INDICATORS = 16;
static lv_obj_t *lvglTopIndicators[LVGL_MAX_TOP_INDICATORS] = {};
static bool lvglKeyboardShiftOneShot = false;
static bool lvglKeyboardShiftLocked = false;
static unsigned long lvglKeyboardShiftLastPressMs = 0;
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
static bool lvglSwipeBackPending = false;
static bool lvglSwipeCandidate = false;
static bool lvglSwipeHorizontalLocked = false;
static unsigned long lvglLastTapReleaseMs = 0;
static int16_t lvglLastTapReleaseX = 0;
static int16_t lvglLastTapReleaseY = 0;
static unsigned long lvglLastTickMs = 0;
static unsigned long lvglLastInfoRefreshMs = 0;
static unsigned long lvglLastStatusRefreshMs = 0;
static unsigned long lvglLastMediaPlayerRefreshMs = 0;
static bool lvglMediaPlayerVisible = false;

bool sdLock(TickType_t timeout = pdMS_TO_TICKS(3000))
{
    if (!sdMutex) return true;
    return xSemaphoreTake(sdMutex, timeout) == pdTRUE;
}

void sdUnlock()
{
    if (sdMutex) xSemaphoreGive(sdMutex);
}

void fsWriteBegin()
{
    portENTER_CRITICAL(&fsWriteMux);
    fsWriteOps++;
    portEXIT_CRITICAL(&fsWriteMux);
}

void fsWriteEnd()
{
    portENTER_CRITICAL(&fsWriteMux);
    if (fsWriteOps > 0) fsWriteOps--;
    portEXIT_CRITICAL(&fsWriteMux);
}

bool fsWriteBusy()
{
    portENTER_CRITICAL(&fsWriteMux);
    uint32_t n = fsWriteOps;
    portEXIT_CRITICAL(&fsWriteMux);
    return n != 0;
}

SdStats sdStatsSnapshot()
{
    SdStats snap;
    portENTER_CRITICAL(&sdStatsMux);
    snap = sdStats;
    portEXIT_CRITICAL(&sdStatsMux);
    return snap;
}

void sdStatsLogSnapshot(const SdStats &s, const char *reason)
{
    const char *tag = (reason && reason[0]) ? reason : "periodic";
    Serial.printf(
        "[SD][stats] reason=%s mounted=%d faults=%lu remount_attempts=%lu remount_ok=%lu remount_fail=%lu try_8m=%lu try_4m=%lu try_1m=%lu force=%lu auto=%lu last_fault_ms=%lu last_ok_ms=%lu last_fail_ms=%lu last_fault_at=%s\n",
        tag,
        sdMounted ? 1 : 0,
        static_cast<unsigned long>(s.faultCount),
        static_cast<unsigned long>(s.remountAttempts),
        static_cast<unsigned long>(s.remountSuccess),
        static_cast<unsigned long>(s.remountFailure),
        static_cast<unsigned long>(s.remountTryFast),
        static_cast<unsigned long>(s.remountTryRecovery),
        static_cast<unsigned long>(s.remountTrySafe),
        static_cast<unsigned long>(s.forceRemountCalls),
        static_cast<unsigned long>(s.autoRetryCalls),
        s.lastFaultMs,
        s.lastRemountOkMs,
        s.lastRemountFailMs,
        s.lastFaultWhere[0] ? s.lastFaultWhere : "-"
    );
}

void sdStatsService()
{
    if (static_cast<unsigned long>(millis() - sdStatsLastLogMs) < SD_STATS_LOG_PERIOD_MS) return;
    sdStatsLastLogMs = millis();
    SdStats snap = sdStatsSnapshot();
    bool changed = (snap.faultCount != sdStatsLastLoggedFaultCount) ||
                   (snap.remountAttempts != sdStatsLastLoggedRemountAttempts) ||
                   (snap.remountSuccess != sdStatsLastLoggedRemountSuccess) ||
                   (snap.remountFailure != sdStatsLastLoggedRemountFailure);
    if (!changed) return;
    sdStatsLastLoggedFaultCount = snap.faultCount;
    sdStatsLastLoggedRemountAttempts = snap.remountAttempts;
    sdStatsLastLoggedRemountSuccess = snap.remountSuccess;
    sdStatsLastLoggedRemountFailure = snap.remountFailure;
    sdStatsLogSnapshot(snap, "periodic");
}

bool sdMountAttemptAtFreq(uint32_t freqHz)
{
    if (!sdSpiReady) {
        sdSpi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
        sdSpiReady = true;
    }
    SD.end();
    delay(2);
    if (!SD.begin(SD_CS, sdSpi, freqHz)) return false;
    if (!SD.exists("/web")) SD.mkdir("/web");
    if (!SD.exists("/Screenshots")) SD.mkdir("/Screenshots");
    if (!SD.exists(CHAT_LOG_DIR)) SD.mkdir(CHAT_LOG_DIR);
    return true;
}

bool sdEnsureMountedLocked(bool forceRemount = false)
{
    if (sdMounted && !forceRemount) return true;

    const unsigned long now = millis();
    static unsigned long lastRecoveryAttemptMs = 0;
    if (!forceRemount && static_cast<unsigned long>(now - lastRecoveryAttemptMs) < SD_RECOVERY_MIN_INTERVAL_MS) {
        return sdMounted;
    }
    lastRecoveryAttemptMs = now;

    portENTER_CRITICAL(&sdStatsMux);
    sdStats.remountAttempts++;
    if (forceRemount) sdStats.forceRemountCalls++;
    portEXIT_CRITICAL(&sdStatsMux);

    static const uint32_t freqs[] = {SD_SPI_FREQ_HZ, SD_SPI_FREQ_RECOVERY_HZ, SD_SPI_FREQ_SAFE_HZ};
    for (size_t i = 0; i < (sizeof(freqs) / sizeof(freqs[0])); i++) {
        portENTER_CRITICAL(&sdStatsMux);
        if (freqs[i] == SD_SPI_FREQ_HZ) sdStats.remountTryFast++;
        else if (freqs[i] == SD_SPI_FREQ_RECOVERY_HZ) sdStats.remountTryRecovery++;
        else sdStats.remountTrySafe++;
        portEXIT_CRITICAL(&sdStatsMux);

        if (sdMountAttemptAtFreq(freqs[i])) {
            sdMounted = true;
            const unsigned long okMs = millis();
            portENTER_CRITICAL(&sdStatsMux);
            sdStats.remountSuccess++;
            sdStats.lastRemountOkMs = okMs;
            portEXIT_CRITICAL(&sdStatsMux);
            Serial.printf("[SD] remount ok @ %lu Hz\n", static_cast<unsigned long>(freqs[i]));
            return true;
        }
        delay(SD_RECOVERY_RETRY_DELAY_MS);
    }

    sdMounted = false;
    const unsigned long failMs = millis();
    portENTER_CRITICAL(&sdStatsMux);
    sdStats.remountFailure++;
    sdStats.lastRemountFailMs = failMs;
    portEXIT_CRITICAL(&sdStatsMux);
    Serial.println("[SD] remount failed at all SPI speeds");
    return false;
}

bool sdEnsureMounted(bool forceRemount)
{
    if (sdMounted && !forceRemount) return true;
    if (!sdMounted && !forceRemount) return false;
    if (fsWriteBusy()) return false;
    if (!sdLock()) return false;
    bool ok = sdEnsureMountedLocked(forceRemount);
    sdUnlock();
    return ok;
}

void sdMarkFault(const char *where)
{
    sdMounted = false;
    const unsigned long now = millis();
    portENTER_CRITICAL(&sdStatsMux);
    sdStats.faultCount++;
    sdStats.lastFaultMs = now;
    if (where && where[0]) {
        snprintf(sdStats.lastFaultWhere, sizeof(sdStats.lastFaultWhere), "%s", where);
    } else {
        sdStats.lastFaultWhere[0] = '\0';
    }
    portEXIT_CRITICAL(&sdStatsMux);
    if (where && where[0]) {
        Serial.printf("[SD] I/O failure at %s, scheduling remount\n", where);
    }
}

void audio_info(const char *info)
{
    Serial.println(String("[AUDIO] ") + info);
}

bool sdMounted = false;
bool sdSpiReady = false;
unsigned long sdLastAutoRetryMs = 0;
bool bootSdInitPending = true;
bool bootWifiInitPending = true;
bool wifiScanInProgress = false;
unsigned long wifiScanAnimLastMs = 0;
uint8_t wifiScanAnimPhase = 0;
unsigned long bootDeferredStartMs = 0;
bool mdnsStarted = false;
bool webRoutesRegistered = false;
bool webServerRunning = false;
bool dnsRunning = false;
bool networkSuspendedForAudio = false;
unsigned long networkResumeLastAttemptMs = 0;
uint32_t touchI2cHzActive = TOUCH_I2C_HZ;
uint8_t touchI2cAddrActive = 0;
uint16_t touchReadFailStreak = 0;
uint16_t touchGhostLowStreak = 0;
unsigned long touchLastInitMs = 0;
unsigned long touchLastNoIrqPollMs = 0;
unsigned long touchLastPollMs = 0;
unsigned long touchLastRecoveryMs = 0;
float batteryVoltage = 0.0f;
uint8_t batteryPercent = 0;
bool batteryCharging = false;
uint8_t lightPercent = 0;
unsigned long lastTopIndicatorRefreshMs = 0;
unsigned long lastLightLogMs = 0;
unsigned long lastUserActivityMs = 0;
unsigned long lastSensorSampleMs = 0;
unsigned long lastChargeEvalMs = 0;
unsigned long lastChargeSeenMs = 0;
bool displayAwake = true;
uint8_t displayBrightnessPercent = 100;
String deviceShortName = DEVICE_SHORT_NAME;
uint8_t cpuLoadPercent = 0;
bool batteryFilterInitialized = false;
bool lightFilterInitialized = false;
float lightPercentFiltered = 0.0f;
float chargePrevVoltage = 0.0f;
int8_t chargeTrendScore = 0;
uint8_t batteryIconAnimPercent = 0;
unsigned long batteryIconAnimLastMs = 0;
uint16_t lightRawAdc = 0;
uint16_t lightMinObserved = 4095;
uint16_t lightMaxObserved = 0;
uint8_t wakeTouchConfirmCount = 0;
bool wakeTouchReleaseGuard = false;

enum UiScreen : uint8_t {
    UI_HOME,
    UI_CHAT,
    UI_CHAT_PEERS,
    UI_WIFI_LIST,
    UI_MEDIA,
    UI_INFO,
    UI_GAMES,
    UI_CONFIG,
    UI_CONFIG_MQTT_CONFIG,
    UI_CONFIG_MQTT_CONTROLS,
    UI_GAME_SNAKE,
    UI_GAME_TETRIS
};

UiScreen uiScreen = UI_HOME;
String uiStatusLine = "Ready";

void lvglEnsureScreenBuilt(UiScreen screen);
lv_obj_t *lvglScreenForUi(UiScreen screen);
void lvglOpenScreen(UiScreen screen, lv_scr_load_anim_t anim);
static bool uiScreenSupportsSwipeBack(UiScreen screen);

struct WifiEntry {
    String ssid;
    int32_t rssi;
    wifi_auth_mode_t auth;
};

static constexpr int MAX_WIFI_RESULTS = 20;
WifiEntry wifiEntries[MAX_WIFI_RESULTS];
int wifiCount = 0;

enum ChatTransport : uint8_t {
    CHAT_TRANSPORT_WIFI = 0,
    CHAT_TRANSPORT_MQTT = 1,
};

struct ChatMessage {
    String author;
    String text;
    String messageId;
    bool outgoing;
    unsigned long tsMs;
    ChatTransport transport;
};

static constexpr int CHAT_PAGE_MESSAGES = 8;
static constexpr int MAX_CHAT_MESSAGES = CHAT_PAGE_MESSAGES * 2;
ChatMessage chatMessages[MAX_CHAT_MESSAGES];
int chatMessageCount = 0;
String currentChatPeerKey;
String chatDeferredAirplanePeerKey;
String chatDeferredAirplaneText;

struct PendingChatMessage {
    String peerKey;
    String author;
    String text;
    String messageId;
    unsigned long createdMs;
    unsigned long lastAttemptMs;
    uint8_t attempts;
};

PendingChatMessage chatPendingMessages[MAX_CHAT_PENDING];
int chatPendingCount = 0;
bool chatPendingLoaded = false;

static String sanitizeDeviceShortName(String name)
{
    name.trim();
    String out;
    out.reserve(name.length());
    for (size_t i = 0; i < name.length(); ++i) {
        const char c = name[i];
        if (c >= 32 && c <= 126) out += c;
    }
    out.trim();
    if (out.length() > 24) out.remove(24);
    return out;
}

static const String &deviceShortNameValue()
{
    return deviceShortName;
}

static String chatSafeFileToken(const String &raw);
static String chatFriendlyLogFilenameForPeer(const String &peerKey);
static String chatLegacyLogPathForPeer(const String &peerKey);
static String chatLogPathForPeer(const String &peerKey);
static bool chatHasUnreadMessages();
static void chatSetPeerUnread(const String &peerKey, bool unread);
static bool chatSendAndStoreMessage(const String &peerKey, const String &text);
static void chatStageDeferredAirplaneMessage(const String &peerKey, const String &text);
static void chatFlushDeferredAirplaneMessage();
static bool chatMessagePendingForPeer(const String &peerKey, const String &messageId);
static bool chatDeleteMessageAt(const String &peerKey, int index);
void lvglDeleteChatMessageEvent(lv_event_t *e);

static void lvglHideChatMenu()
{
    if (lvglChatMenuBackdrop) lv_obj_add_flag(lvglChatMenuBackdrop, LV_OBJ_FLAG_HIDDEN);
    if (lvglChatMenuPanel) lv_obj_add_flag(lvglChatMenuPanel, LV_OBJ_FLAG_HIDDEN);
}

static String chatDisplayAuthorForMessage(const ChatMessage &msg)
{
    if (msg.outgoing) return deviceShortNameValue();
    if (!currentChatPeerKey.isEmpty()) {
        const String peerName = chatDisplayNameForPeerKey(currentChatPeerKey);
        if (!peerName.isEmpty()) return peerName;
    }
    return msg.author;
}

static bool chatDeleteHistoryForPeer(const String &peerKey)
{
    if (peerKey.isEmpty()) return false;
    if (!sdEnsureMounted(true)) return false;

    const String safeKey = chatSafeFileToken(peerKey);
    String shortKey = safeKey;
    if (shortKey.length() > 8) shortKey = shortKey.substring(0, 8);
    const String keySuffix = "_" + shortKey + ".txt";
    const String resolvedPath = chatLogPathForPeer(peerKey);
    const String friendlyPath = String(CHAT_LOG_DIR) + "/" + chatFriendlyLogFilenameForPeer(peerKey);
    const String legacyPath = chatLegacyLogPathForPeer(peerKey);

    fsWriteBegin();
    bool removedAny = false;
    if (sdLock()) {
        auto tryRemove = [&](const String &path) {
            if (!path.isEmpty() && SD.exists(path) && SD.remove(path)) removedAny = true;
        };

        tryRemove(resolvedPath);
        if (friendlyPath != resolvedPath) tryRemove(friendlyPath);
        if (legacyPath != resolvedPath && legacyPath != friendlyPath) tryRemove(legacyPath);

        File dir = SD.open(CHAT_LOG_DIR, FILE_READ);
        if (dir) {
            File entry = dir.openNextFile();
            while (entry) {
                if (!entry.isDirectory()) {
                    String name = String(entry.name());
                    if (name.endsWith(keySuffix)) {
                        const String fullPath = String(CHAT_LOG_DIR) + "/" + name;
                        if (fullPath != resolvedPath && fullPath != friendlyPath && fullPath != legacyPath && SD.exists(fullPath) && SD.remove(fullPath)) {
                            removedAny = true;
                        }
                    }
                }
                entry.close();
                entry = dir.openNextFile();
            }
            dir.close();
        }
        sdUnlock();
    }
    fsWriteEnd();
    return removedAny;
}

static bool chatApplyConversationDeletion(const String &peerKey, const String &status)
{
    if (peerKey.isEmpty()) return false;
    const bool removed = chatDeleteHistoryForPeer(peerKey);
    chatSetPeerUnread(peerKey, false);
    if (currentChatPeerKey == peerKey) {
        currentChatPeerKey = "";
        chatClearCache();
    }
    if (lvglReady) {
        lvglRefreshChatLayout();
        lvglRefreshChatContactsUi();
        lvglRefreshChatUi();
    }
    uiStatusLine = status;
    if (lvglReady) lvglSyncStatusLine();
    return removed;
}

struct P2PPeer {
    String name;
    String pubKeyHex;
    IPAddress ip;
    uint16_t port;
    bool enabled;
    bool unread;
    unsigned long lastSeenMs;
};

static constexpr uint16_t P2P_UDP_PORT = 4210;
static constexpr uint8_t P2P_PKT_MAGIC_0 = 'P';
static constexpr uint8_t P2P_PKT_MAGIC_1 = '2';
static constexpr uint8_t P2P_PKT_VERSION = 1;
static constexpr uint8_t P2P_PKT_TYPE_CHAT = 1;
static constexpr uint8_t P2P_PKT_TYPE_DISCOVER = 2;
static constexpr int MAX_P2P_PEERS = 8;
static constexpr int MAX_P2P_DISCOVERED = 12;
static constexpr size_t P2P_PUBLIC_KEY_BYTES = crypto_box_curve25519xchacha20poly1305_PUBLICKEYBYTES;
static constexpr size_t P2P_SECRET_KEY_BYTES = crypto_box_curve25519xchacha20poly1305_SECRETKEYBYTES;
static constexpr size_t P2P_NONCE_BYTES = crypto_box_curve25519xchacha20poly1305_NONCEBYTES;
static constexpr size_t P2P_MAC_BYTES = crypto_box_curve25519xchacha20poly1305_MACBYTES;
static constexpr size_t P2P_MAX_CHAT_TEXT = 160;
static constexpr size_t P2P_MAX_PACKET = 512;
P2PPeer p2pPeers[MAX_P2P_PEERS];
int p2pPeerCount = 0;

struct P2PDiscoveredPeer {
    String name;
    String pubKeyHex;
    IPAddress ip;
    uint16_t port;
    bool trusted;
    unsigned long lastSeenMs;
};

P2PDiscoveredPeer p2pDiscoveredPeers[MAX_P2P_DISCOVERED];
int p2pDiscoveredCount = 0;

static bool chatHasUnreadMessages()
{
    for (int i = 0; i < p2pPeerCount; ++i) {
        if (p2pPeers[i].unread) return true;
    }
    return false;
}

static void chatSetPeerUnread(const String &peerKey, bool unread)
{
    const int idx = p2pFindPeerByPubKeyHex(peerKey);
    if (idx < 0) return;
    if (p2pPeers[idx].unread == unread) return;
    p2pPeers[idx].unread = unread;
    if (lvglReady) lvglRefreshTopIndicators();
}

static constexpr int SNAKE_COLS = 12;
static constexpr int SNAKE_ROWS = 14;
static constexpr int SNAKE_CELL = 14;
static constexpr int SNAKE_MAX_CELLS = SNAKE_COLS * SNAKE_ROWS;
static constexpr int SNAKE_BOARD_X = 36;
static constexpr int SNAKE_BOARD_Y = 50;
static constexpr unsigned long SNAKE_STEP_MS = 200;
int8_t snakeDir = 3; // 0 up,1 right,2 down,3 left
int8_t snakeNextDir = 3;
int snakeLen = 0;
int8_t snakeX[SNAKE_MAX_CELLS];
int8_t snakeY[SNAKE_MAX_CELLS];
int8_t snakeFoodX = 0;
int8_t snakeFoodY = 0;
bool snakeGameOver = false;
uint16_t snakeScore = 0;
unsigned long snakeLastStepMs = 0;

static constexpr int TETRIS_COLS = 10;
static constexpr int TETRIS_ROWS = 16;
static constexpr int TETRIS_CELL = 12;
static constexpr int TETRIS_BOARD_X = 60;
static constexpr int TETRIS_BOARD_Y = 50;
static constexpr unsigned long TETRIS_STEP_MS = 550;
uint8_t tetrisGrid[TETRIS_ROWS][TETRIS_COLS];
int8_t tetrisType = 0;
int8_t tetrisRot = 0;
int8_t tetrisX = 3;
int8_t tetrisY = 0;
bool tetrisGameOver = false;
uint16_t tetrisScore = 0;
unsigned long tetrisLastStepMs = 0;

struct MediaEntry {
    String name;
    String path;
    bool isDir;
    size_t size;
};

static constexpr int MAX_MEDIA_ENTRIES = 80;
static constexpr int MEDIA_PLAYER_PANEL_H = 108;
static constexpr int MEDIA_PAGE_SIZE = 12;
static constexpr int MEDIA_SCAN_YIELD_EVERY = 12;
static constexpr unsigned long MEDIA_SCAN_SLOW_MS = 250;
static constexpr int MEDIA_ENTRY_PARENT = -1;
static constexpr int MEDIA_ENTRY_PREV_PAGE = -2;
static constexpr int MEDIA_ENTRY_NEXT_PAGE = -3;
MediaEntry mediaEntries[MAX_MEDIA_ENTRIES];
int mediaCount = 0;
String mediaCurrentDir = "/";
int mediaOffset = 0;
bool mediaHasMore = false;
String mediaNowPlaying;
String mediaSelectedSourcePath;
String mediaSelectedTrackName;
String mediaPlaybackPath;
bool mediaIsPlaying = false;
bool mediaPaused = false;
uint8_t mediaVolumePercent = 50;
bool lvglMediaRefreshPending = false;
uint32_t mediaScanRuns = 0;
uint32_t mediaScanSlowRuns = 0;
bool rgbPersistR = false;
bool rgbPersistG = false;
bool rgbPersistB = false;
unsigned long rgbFlashUntilMs = 0;
uint8_t rgbPulseLevel = 32;
int8_t rgbPulseDir = 1;
unsigned long rgbPulseLastMs = 0;
uint8_t audioVolumeCurrent = 0;
bool audioBackendReady = false;
unsigned long audioLastInitAttemptMs = 0;
String audioLastError;

Preferences wifiPrefs;
Preferences batteryPrefs;
Preferences mqttPrefs;
Preferences uiPrefs;
Preferences p2pPrefs;
char *serialLogRing = nullptr;
size_t serialLogHead = 0;
size_t serialLogCount = 0;
uint32_t serialLastWsPushMs = 0;
uint32_t serialLogWsMinIntervalMs = SERIAL_LOG_RATE_MS_DEFAULT;
size_t serialLogKeepLines = SERIAL_LOG_RING_SIZE;
unsigned long serialTerminalStreamUntilMs = 0;
bool recordTelemetryEnabled = false;
bool systemSoundsEnabled = true;
bool wsRebootOnDisconnectEnabled = false;
bool airplaneModeEnabled = false;
bool p2pUdpStarted = false;
bool p2pReady = false;
unsigned char p2pPublicKey[P2P_PUBLIC_KEY_BYTES] = {0};
unsigned char p2pSecretKey[P2P_SECRET_KEY_BYTES] = {0};
unsigned long p2pLastDiscoverAnnounceMs = 0;
uint32_t telemetryMaxKB = 512;
String savedStaSsid;
String savedStaPass;
String savedApSsid = AP_SSID;
String savedApPass = AP_PASS;
String pendingSaveSsid;
String pendingSavePass;
bool bootStaConnectInProgress = false;
bool pendingSaveCreds = false;
unsigned long bootStaConnectStartedMs = 0;
static constexpr unsigned long BOOT_STA_TIMEOUT_MS = 12000;
static constexpr unsigned long CAR_INPUT_TELEMETRY_PERIOD_MS = 1000UL;
unsigned long staLastConnectAttemptMs = 0;
bool apModeActive = false;
bool wifiEventsRegistered = false;
volatile bool wifiStaGotIpPending = false;
volatile bool wifiStaDisconnectedPending = false;
volatile uint8_t wifiStaDisconnectReasonPending = 0;
wifi_event_id_t wifiStaGotIpEventId = 0;
wifi_event_id_t wifiStaDisconnectedEventId = 0;
unsigned long lastBatterySnapshotMs = 0;
float lastBatterySnapshotVoltage = -1.0f;
unsigned long lastCarInputTelemetryMs = 0;
bool rebootRequested = false;
unsigned long rebootRequestedAtMs = 0;
uint32_t cpuLoadPrevLoopStartUs = 0;
uint32_t cpuLoadPrevActiveUs = 0;
uint32_t cpuLoadAccumActiveUs = 0;
uint32_t cpuLoadAccumTotalUs = 0;
unsigned long cpuLoadWindowStartMs = 0;

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
String mqttStatusLine = "Disabled";
bool mqttDiscoveryPublished = false;
unsigned long mqttLastReconnectMs = 0;
static constexpr unsigned long MQTT_RECONNECT_MS = 5000;
bool mqttConnectRequested = false;
static constexpr int MQTT_MAX_BUTTONS = 12;
static constexpr const char *MQTT_CHAT_NAMESPACE = "global";
static constexpr uint16_t MQTT_CLIENT_BUFFER_SIZE = 1400;
static constexpr uint16_t MQTT_CLIENT_BUFFER_IDLE = 256;
static constexpr uint32_t MQTT_MIN_FREE_HEAP_CONNECT = 36000U;
static constexpr uint32_t MQTT_MIN_LARGEST_8BIT_CONNECT = 8192U;
static constexpr uint32_t MQTT_MIN_FREE_HEAP_PUBLISH = 42000U;
static constexpr uint32_t MQTT_MIN_LARGEST_8BIT_PUBLISH = 14000U;
int mqttButtonCount = 4;
String mqttButtonNames[MQTT_MAX_BUTTONS];
bool mqttButtonCritical[MQTT_MAX_BUTTONS];
File screenshotJpegFile;
String screenshotJpegPath;

static const char UPLOAD_FALLBACK_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>ESP32 Fallback Upload</title>
  <style>
    body{font-family:system-ui,Segoe UI,Arial;background:#10131a;color:#eef;margin:0;padding:18px}
    .card{max-width:760px;margin:0 auto;background:#171c26;border:1px solid #2f3b50;border-radius:12px;padding:16px}
    .row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
    .muted{color:#9fb0cc}
    input,button{background:#0f141d;color:#eef;border:1px solid #2f3b50;border-radius:8px;padding:8px}
    button{cursor:pointer}
    .a{background:#1e6fff;border-color:#1e6fff}
    .d{background:#c73a3a;border-color:#c73a3a}
    #list{margin-top:10px;border:1px solid #2f3b50;border-radius:10px;max-height:320px;overflow:auto}
    .it{display:grid;grid-template-columns:1fr 100px;gap:8px;padding:8px 10px;border-bottom:1px solid #232e40}
    .it:last-child{border-bottom:0}
  </style>
</head>
<body>
  <div class="card">
    <h2 style="margin:0 0 8px 0">Fallback Upload Manager</h2>
    <div class="muted">Firmware: %FIRMWARE_VERSION%</div>
    <p class="muted">Upload missing web files to <code>/web/</code> or open the recovery browser.</p>
    <div class="row">
      <a href="/recovery"><button>Open Recovery Browser</button></a>
      <button id="refreshBtn">Refresh List</button>
      <button id="clearBtn" class="d">Clear /web/</button>
    </div>
    <form id="uploadForm" style="margin-top:10px">
      <input id="uploadInput" type="file" multiple required>
      <button class="a" type="submit">Upload to /web/</button>
    </form>
    <div id="status" class="muted" style="margin-top:8px">Ready.</div>
    <div id="list"></div>
  </div>
<script>
const listEl = document.getElementById('list');
const statusEl = document.getElementById('status');
const uploadForm = document.getElementById('uploadForm');
const uploadInput = document.getElementById('uploadInput');
const show = (m,e=false)=>{ statusEl.textContent = m; statusEl.style.color = e ? '#ff9797' : '#9fb0cc'; };
function renderList(files){
  if(!files.length){ listEl.innerHTML = '<div class="it"><div>No files in /web/</div><div></div></div>'; return; }
  listEl.innerHTML = files.map(f=>{
    const kb = (Number(f.size)||0) >= 1024 ? ((Number(f.size)||0)/1024).toFixed(1)+' KB' : (Number(f.size)||0)+' B';
    return '<div class="it"><div>'+f.path+'</div><div>'+kb+'</div></div>';
  }).join('');
}
async function loadList(){
  try{
    const r = await fetch('/fs/list',{cache:'no-store'});
    const j = await r.json();
    const files = Array.isArray(j.files) ? j.files.filter(x=>x.path && x.path.startsWith('/web/')) : [];
    renderList(files);
  }catch(e){ show('List failed: ' + e, true); }
}
async function uploadOne(file){
  const fd = new FormData();
  fd.append('fsFile', file, file.name);
  const r = await fetch('/fs/upload?path=/web/', {method:'POST', body:fd});
  if(!r.ok) throw new Error(await r.text());
}
uploadForm.addEventListener('submit', async (e)=>{
  e.preventDefault();
  const files = Array.from(uploadInput.files || []);
  if(!files.length){ show('Select files first', true); return; }
  try{
    for(let i=0;i<files.length;i++){
      show('Uploading ' + files[i].name + ' (' + (i+1) + '/' + files.length + ')');
      await uploadOne(files[i]);
    }
    show('Upload complete');
    await loadList();
  }catch(err){
    show('Upload failed: ' + err.message, true);
  }
});
document.getElementById('refreshBtn').onclick = loadList;
document.getElementById('clearBtn').onclick = async ()=>{
  if(!confirm('Delete all contents of /web/?')) return;
  const body = new URLSearchParams({path:'/web'});
  const r = await fetch('/fs/delete',{method:'POST', body});
  if(!r.ok){ show('Failed to clear /web/', true); return; }
  show('Cleared /web/');
  loadList();
};
loadList();
</script>
</body>
</html>
)rawliteral";

static const char WIFI_SETUP_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>ESP32 WiFi Setup</title>
  <style>
    body{font-family:system-ui,Segoe UI,Arial;background:#10131a;color:#eef;margin:0;padding:18px}
    .card{max-width:760px;margin:0 auto;background:#171c26;border:1px solid #2f3b50;border-radius:12px;padding:16px}
    .row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
    .muted{color:#9fb0cc}
    input,button,select{background:#0f141d;color:#eef;border:1px solid #2f3b50;border-radius:8px;padding:8px}
    button{cursor:pointer}
    .a{background:#1e6fff;border-color:#1e6fff}
    #tele{margin-top:10px;padding:10px;border:1px solid #2f3b50;border-radius:10px}
  </style>
</head>
<body>
  <div class="card">
    <h2 style="margin:0 0 8px 0">WiFi Setup</h2>
    <div class="muted">Firmware: %FIRMWARE_VERSION%</div>
    <div class="muted">AP: %AP_SSID% | AP IP: %AP_IP%</div>
    <div id="tele" class="muted">Loading telemetry...</div>
    <div class="row" style="margin-top:10px">
      <button id="scanBtn" class="a">Scan WiFi</button>
      <select id="ssid" style="min-width:240px"></select>
    </div>
    <div class="row" style="margin-top:10px">
      <input id="pass" type="password" placeholder="Password">
      <button id="showBtn">Show</button>
      <button id="connBtn" class="a">Connect</button>
    </div>
    <div id="status" class="muted" style="margin-top:8px">Ready</div>
  </div>
<script>
const teleEl = document.getElementById('tele');
const statusEl = document.getElementById('status');
const ssidEl = document.getElementById('ssid');
const passEl = document.getElementById('pass');
const showBtn = document.getElementById('showBtn');
const show = (m,e=false)=>{ statusEl.textContent=m; statusEl.style.color=e?'#ff9797':'#9fb0cc'; };

async function refreshTelemetry(){
  try{
    const r=await fetch('/api/telemetry',{cache:'no-store'});
    const j=await r.json();
    teleEl.textContent =
      `Battery: ${j.battery_percent}% (${j.battery_voltage}V) | ` +
      `Light: ${j.light_percent}% | ` +
      `WiFi: ${j.wifi_connected ? ('Connected to '+j.wifi_ssid+' ('+j.wifi_rssi+' dBm)') : 'Not connected'}`;
  }catch(e){
    teleEl.textContent='Telemetry unavailable';
  }
}

async function scanWifi(){
  try{
    show('Scanning...');
    const r=await fetch('/api/wifi/scan',{cache:'no-store'});
    const j=await r.json();
    ssidEl.innerHTML='';
    (j.networks||[]).forEach(n=>{
      const o=document.createElement('option');
      o.value=n.ssid;
      o.textContent=`${n.ssid} (${n.rssi} dBm, ${n.auth})`;
      ssidEl.appendChild(o);
    });
    show(`Found ${j.networks ? j.networks.length : 0} networks`);
  }catch(e){
    show('Scan failed: '+e, true);
  }
}

async function connectWifi(){
  const ssid = ssidEl.value || '';
  const pass = passEl.value || '';
  if(!ssid){ show('Select SSID first', true); return; }
  try{
    show('Connecting...');
    const body = new URLSearchParams({ssid, pass});
    const r=await fetch('/api/wifi/connect',{method:'POST', body});
    if(!r.ok) throw new Error(await r.text());
    show('Connect request sent. Device will save credentials on success.');
  }catch(e){
    show('Connect failed: '+e.message, true);
  }
}

document.getElementById('scanBtn').onclick = scanWifi;
document.getElementById('connBtn').onclick = connectWifi;
showBtn.onclick = ()=>{ passEl.type = passEl.type === 'password' ? 'text' : 'password'; showBtn.textContent = passEl.type === 'password' ? 'Show' : 'Hide'; };
scanWifi();
refreshTelemetry();
setInterval(refreshTelemetry, 2000);
</script>
</body>
</html>
)rawliteral";

bool touchReadBytes(uint8_t addr, const uint8_t *prefix, size_t prefixLen, uint8_t *buf, uint8_t len)
{
    Wire.beginTransmission(addr);
    for (size_t i = 0; i < prefixLen; i++) {
        Wire.write(prefix[i]);
    }
    if (Wire.endTransmission(false) != 0) return false;

    const uint8_t got = Wire.requestFrom(addr, len);
    if (got != len) return false;
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = Wire.read();
    }
    return true;
}

bool touchWriteBytes(uint8_t addr, const uint8_t *data, size_t len)
{
    Wire.beginTransmission(addr);
    for (size_t i = 0; i < len; i++) {
        Wire.write(data[i]);
    }
    return Wire.endTransmission() == 0;
}

void cstWriteReg(uint8_t reg, uint8_t value)
{
    const uint8_t data[] = {reg, value};
    touchWriteBytes(CST820_ADDR, data, sizeof(data));
}

bool cstReadRegs(uint8_t startReg, uint8_t *buf, uint8_t len)
{
    return touchReadBytes(CST820_ADDR, &startReg, 1, buf, len);
}

bool gt911ReadRegs(uint16_t startReg, uint8_t *buf, uint8_t len)
{
    const uint8_t prefix[] = {
        static_cast<uint8_t>((startReg >> 8) & 0xFF),
        static_cast<uint8_t>(startReg & 0xFF),
    };
    return touchReadBytes(touchI2cAddrActive, prefix, sizeof(prefix), buf, len);
}

bool gt911WriteReg8(uint16_t reg, uint8_t value)
{
    const uint8_t data[] = {
        static_cast<uint8_t>((reg >> 8) & 0xFF),
        static_cast<uint8_t>(reg & 0xFF),
        value,
    };
    return touchWriteBytes(touchI2cAddrActive, data, sizeof(data));
}

bool gt911ProbeAddr(uint8_t addr, uint8_t *productIdOut = nullptr)
{
    touchI2cAddrActive = addr;
    uint8_t productId[4] = {0, 0, 0, 0};
    if (!gt911ReadRegs(GT911_REG_PRODUCT_ID, productId, sizeof(productId))) return false;

    bool useful = false;
    bool printable = true;
    for (uint8_t i = 0; i < sizeof(productId); i++) {
        const uint8_t c = productId[i];
        if (c != 0x00 && c != 0xFF) useful = true;
        if (c < 0x20 || c > 0x7E) printable = false;
    }
    if (!useful) return false;
    if (productIdOut) memcpy(productIdOut, productId, sizeof(productId));

    Serial.printf("[TOUCH] gt911 probe addr=0x%02X pid=%02X %02X %02X %02X text='%c%c%c%c'%s\n",
                  static_cast<unsigned int>(addr),
                  productId[0], productId[1], productId[2], productId[3],
                  printable ? productId[0] : '.',
                  printable ? productId[1] : '.',
                  printable ? productId[2] : '.',
                  printable ? productId[3] : '.',
                  printable ? "" : " nonprint");
    return true;
}

void touchBusBegin(uint32_t hz)
{
    touchI2cHzActive = hz;
    Wire.begin(TOUCH_SDA, TOUCH_SCL, touchI2cHzActive);
    Wire.setTimeOut(20);
}

void cstPulseReset()
{
    pinMode(TOUCH_RST, OUTPUT);
    digitalWrite(TOUCH_RST, LOW);
    delay(10);
    digitalWrite(TOUCH_RST, HIGH);
    delay(180);
}

void gt911PulseReset(uint8_t addr)
{
    pinMode(TOUCH_RST, OUTPUT);
    pinMode(TOUCH_IRQ, OUTPUT);
    // GT911 address latch: INT low -> 0x5D, INT high -> 0x14.
    digitalWrite(TOUCH_IRQ, (addr == GT911_ADDR_PRIMARY) ? LOW : HIGH);
    digitalWrite(TOUCH_RST, LOW);
    delay(10);
    digitalWrite(TOUCH_RST, HIGH);
    delay(8);
    pinMode(TOUCH_IRQ, TOUCH_USE_IRQ ? INPUT_PULLUP : INPUT);
    delay(50);
}

void cstInit()
{
    bool ok = false;
    pinMode(TOUCH_IRQ, TOUCH_USE_IRQ ? INPUT_PULLUP : INPUT);
    static const uint32_t speeds[] = {TOUCH_I2C_HZ, 100000U};
    for (uint8_t s = 0; s < (sizeof(speeds) / sizeof(speeds[0])) && !ok; s++) {
        touchBusBegin(speeds[s]);
        for (uint8_t attempt = 0; attempt < 3 && !ok; attempt++) {
            cstPulseReset();
            cstWriteReg(0xFE, 0xFF);
            delay(3);
            uint8_t probe = 0;
            ok = cstReadRegs(0x02, &probe, 1);
        }
    }
    touchI2cAddrActive = CST820_ADDR;
    touchReadFailStreak = 0;
    touchGhostLowStreak = 0;
    touchLastInitMs = millis();
    Serial.printf("[TOUCH] init ok=%d i2c=%lu irq=%d\n",
                  ok ? 1 : 0,
                  static_cast<unsigned long>(touchI2cHzActive),
                  digitalRead(TOUCH_IRQ));
}

void gt911Init()
{
    bool ok = false;
    uint8_t detectedPid[4] = {0, 0, 0, 0};
    touchI2cAddrActive = GT911_ADDR_PRIMARY;
    static const uint32_t speeds[] = {100000U, TOUCH_I2C_HZ};
    static const uint8_t addrs[] = {GT911_ADDR_PRIMARY, GT911_ADDR_SECONDARY};
    for (uint8_t s = 0; s < (sizeof(speeds) / sizeof(speeds[0])) && !ok; s++) {
        touchBusBegin(speeds[s]);
        // First try both addresses without relying on reset wiring.
        for (uint8_t i = 0; i < (sizeof(addrs) / sizeof(addrs[0])) && !ok; i++) {
            ok = gt911ProbeAddr(addrs[i], detectedPid);
        }
        // Fall back to explicit reset/address latch if direct probing failed.
        for (uint8_t i = 0; i < (sizeof(addrs) / sizeof(addrs[0])) && !ok; i++) {
            gt911PulseReset(addrs[i]);
            delay(10);
            ok = gt911ProbeAddr(addrs[i], detectedPid);
            if (!ok) {
                pinMode(TOUCH_RST, INPUT);
                pinMode(TOUCH_IRQ, TOUCH_USE_IRQ ? INPUT_PULLUP : INPUT);
            }
        }
    }
    if (!ok) touchI2cAddrActive = GT911_ADDR_PRIMARY;
    pinMode(TOUCH_RST, INPUT);
    pinMode(TOUCH_IRQ, TOUCH_USE_IRQ ? INPUT_PULLUP : INPUT);
    touchReadFailStreak = 0;
    touchGhostLowStreak = 0;
    touchLastInitMs = millis();
    Serial.printf("[TOUCH] gt911 init ok=%d addr=0x%02X i2c=%lu irq=%d pid=%02X %02X %02X %02X\n",
                  ok ? 1 : 0,
                  static_cast<unsigned int>(touchI2cAddrActive),
                  static_cast<unsigned long>(touchI2cHzActive),
                  digitalRead(TOUCH_IRQ),
                  detectedPid[0], detectedPid[1], detectedPid[2], detectedPid[3]);
}

void touchInit()
{
    if (TOUCH_CONTROLLER == TOUCH_CTRL_GT911) gt911Init();
    else cstInit();
}

void touchTryRecoverBus(const char *reason)
{
    if (touchReadFailStreak < TOUCH_REINIT_FAIL_THRESHOLD) return;
    const unsigned long now = millis();
    if (static_cast<unsigned long>(now - touchLastRecoveryMs) < TOUCH_REINIT_MIN_INTERVAL_MS) return;
    touchLastRecoveryMs = now;
    Serial.printf("[TOUCH] recover reason=%s fail_streak=%u\n",
                  reason ? reason : "-",
                  static_cast<unsigned int>(touchReadFailStreak));
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
    if (static_cast<unsigned long>(now - touchLastPollMs) < TOUCH_POLL_INTERVAL_MS) return false;
    touchLastPollMs = now;

    bool irqLow = true;
    if (TOUCH_USE_IRQ) {
        irqLow = (digitalRead(TOUCH_IRQ) == LOW);
        if (!irqLow) return false;
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

void lvglTouchReadCb(lv_indev_drv_t *indev, lv_indev_data_t *data)
{
    (void)indev;
    int16_t x = 0;
    int16_t y = 0;
    lvglTouchDown = readScreenTouch(x, y);
    if (!displayAwake) {
        if (lvglTouchDown && !wakeTouchReleaseGuard) {
            wakeTouchReleaseGuard = true;
            wakeTouchConfirmCount = WAKE_TOUCH_RELEASE_STABLE_POLLS;
            displaySetAwake(true);
            lastUserActivityMs = millis();
        }
        if (wakeTouchReleaseGuard) {
            if (lvglTouchDown) {
                wakeTouchConfirmCount = WAKE_TOUCH_RELEASE_STABLE_POLLS;
            } else if (wakeTouchConfirmCount > 0) {
                wakeTouchConfirmCount--;
            } else {
                wakeTouchReleaseGuard = false;
            }
            lvglSwipeTracking = false;
            lvglSwipeCandidate = false;
            lvglSwipeHorizontalLocked = false;
            data->state = LV_INDEV_STATE_REL;
            data->point.x = lvglLastTouchX;
            data->point.y = lvglLastTouchY;
            return;
        }
    }
    if (wakeTouchReleaseGuard) {
        if (lvglTouchDown) {
            wakeTouchConfirmCount = WAKE_TOUCH_RELEASE_STABLE_POLLS;
        } else if (wakeTouchConfirmCount > 0) {
            wakeTouchConfirmCount--;
        } else {
            wakeTouchReleaseGuard = false;
        }
        lvglSwipeTracking = false;
        lvglSwipeCandidate = false;
        lvglSwipeHorizontalLocked = false;
        data->state = LV_INDEV_STATE_REL;
        data->point.x = lvglLastTouchX;
        data->point.y = lvglLastTouchY;
        return;
    }
    if (lvglTouchDown) {
        if (!lvglSwipeTracking) {
            const unsigned long now = millis();
            if (displayAwake &&
                lvglLastTapReleaseMs != 0 &&
                static_cast<unsigned long>(now - lvglLastTapReleaseMs) <= static_cast<unsigned long>(DOUBLE_TAP_MAX_GAP) &&
                abs(static_cast<int>(x) - static_cast<int>(lvglLastTapReleaseX)) <= DOUBLE_TAP_MAX_MOVE &&
                abs(static_cast<int>(y) - static_cast<int>(lvglLastTapReleaseY)) <= DOUBLE_TAP_MAX_MOVE) {
                lvglLastTapReleaseMs = 0;
                lvglSwipeTracking = false;
                lvglSwipeCandidate = false;
                lvglSwipeHorizontalLocked = false;
                displaySetAwake(false);
                wakeTouchReleaseGuard = true;
                wakeTouchConfirmCount = WAKE_TOUCH_RELEASE_STABLE_POLLS;
                data->state = LV_INDEV_STATE_REL;
                data->point.x = lvglLastTouchX;
                data->point.y = lvglLastTouchY;
                return;
            }
            lvglSwipeTracking = true;
            lvglSwipeStartX = x;
            lvglSwipeStartY = y;
            lvglSwipeLastX = x;
            lvglSwipeLastY = y;
            lvglSwipeStartMs = millis();
            lvglSwipeCandidate = uiScreenSupportsSwipeBack(uiScreen);
            lvglSwipeHorizontalLocked = false;
        } else {
            lvglSwipeLastX = x;
            lvglSwipeLastY = y;
            if (lvglSwipeCandidate) {
                const int dx = static_cast<int>(lvglSwipeLastX) - static_cast<int>(lvglSwipeStartX);
                const int dy = static_cast<int>(lvglSwipeLastY) - static_cast<int>(lvglSwipeStartY);
                const int absDx = abs(dx);
                const int absDy = abs(dy);
                if (!lvglSwipeHorizontalLocked) {
                    if (absDy >= SWIPE_CANCEL_VERTICAL_DY && absDy > absDx) {
                        lvglSwipeCandidate = false;
                    } else if (dx >= SWIPE_LOCK_MIN_DX && dx > (absDy * 2)) {
                        lvglSwipeHorizontalLocked = true;
                    }
                } else if (absDy > SWIPE_BACK_MAX_DY) {
                    lvglSwipeCandidate = false;
                    lvglSwipeHorizontalLocked = false;
                }
            }
        }
        lvglLastTouchX = x;
        lvglLastTouchY = y;
        data->state = LV_INDEV_STATE_PR;
        data->point.x = x;
        data->point.y = y;
    } else {
        if (lvglSwipeTracking) {
            lvglSwipeTracking = false;
            const int dx = static_cast<int>(lvglSwipeLastX) - static_cast<int>(lvglSwipeStartX);
            const int dy = static_cast<int>(lvglSwipeLastY) - static_cast<int>(lvglSwipeStartY);
            const unsigned long dt = static_cast<unsigned long>(millis() - lvglSwipeStartMs);
            const bool tapCandidate = displayAwake &&
                                      abs(dx) <= DOUBLE_TAP_MAX_MOVE &&
                                      abs(dy) <= DOUBLE_TAP_MAX_MOVE &&
                                      dt <= DOUBLE_TAP_MAX_TAP_MS;
            if (lvglSwipeCandidate &&
                lvglSwipeHorizontalLocked &&
                dx >= SWIPE_BACK_MIN_DX &&
                abs(dy) <= SWIPE_BACK_MAX_DY &&
                dx > (abs(dy) * 2) &&
                dt <= SWIPE_BACK_MAX_MS) {
                lvglSwipeBackPending = true;
                lvglLastTapReleaseMs = 0;
            } else if (tapCandidate) {
                lvglLastTapReleaseMs = millis();
                lvglLastTapReleaseX = lvglLastTouchX;
                lvglLastTapReleaseY = lvglLastTouchY;
            } else {
                lvglLastTapReleaseMs = 0;
            }
            lvglSwipeCandidate = false;
            lvglSwipeHorizontalLocked = false;
        }
        data->state = LV_INDEV_STATE_REL;
        data->point.x = lvglLastTouchX;
        data->point.y = lvglLastTouchY;
    }
}

void lvglSyncStatusLine()
{
    if (!lvglStatusLabel) return;
    lv_label_set_text_fmt(lvglStatusLabel, "Status: %s", uiStatusLine.c_str());
}

void lvglRegisterTopIndicator(lv_obj_t *obj)
{
    if (!obj) return;
    if (lvglTopIndicatorCount >= LVGL_MAX_TOP_INDICATORS) return;
    lvglTopIndicators[lvglTopIndicatorCount++] = obj;
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
        const lv_color_t planeCol = lv_color_hex(0xF2C35E);
        const lv_color_t planeDim = lv_color_hex(0x5E6570);
        lvglDrawLineSeg(drawCtx, antX + 2, topY + 12, antX + 29, topY + 12, planeCol, 2);
        lvglDrawLineSeg(drawCtx, antX + 14, topY + 4, antX + 14, topY + 20, planeCol, 2);
        lvglDrawLineSeg(drawCtx, antX + 9, topY + 8, antX + 18, topY + 12, planeCol, 2);
        lvglDrawLineSeg(drawCtx, antX + 9, topY + 16, antX + 18, topY + 12, planeCol, 2);
        lvglDrawLineSeg(drawCtx, antX + 22, topY + 8, antX + 28, topY + 6, planeDim, 2);
        lvglDrawLineSeg(drawCtx, antX + 22, topY + 16, antX + 28, topY + 18, planeDim, 2);
    } else {
        for (int i = 0; i < 4; i++) {
            const bool on = connected && i < bars;
            lv_color_t col = on ? lv_color_hex(0x79E28A) : lv_color_hex(0x30404A);
            lvglDrawRectSolid(drawCtx, antX + (i * 8), topY + 23 - antHeights[i], 4, antHeights[i], col, on ? LV_OPA_COVER : LV_OPA_70);
        }
    }
    if (!connected && !airplaneModeEnabled) {
        if (apModeActive) {
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
        } else {
            const lv_color_t cross = lv_color_hex(0xE45B5B);
            lvglDrawLineSeg(drawCtx, antX - 2, topY + 2, antX + 31, topY + 23, cross, 2, LV_OPA_COVER);
            lvglDrawLineSeg(drawCtx, antX - 2, topY + 23, antX + 31, topY + 2, cross, 2, LV_OPA_COVER);
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
    const int battX = ox + ow - 54;
    if (showUnreadMail) {
        const lv_color_t mailCol = lv_color_hex(0xF2C35E);
        const int mailX = battX - 28;
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

    String topName = deviceShortNameValue();
    if (topName.length() > 8) topName.remove(8);
    lv_draw_label_dsc_t labelDsc;
    lv_draw_label_dsc_init(&labelDsc);
    labelDsc.color = lv_color_hex(0xC8D3DD);
    labelDsc.opa = LV_OPA_COVER;
    labelDsc.align = LV_TEXT_ALIGN_CENTER;
    lv_area_t nameA;
    nameA.x1 = static_cast<lv_coord_t>(ox + 56);
    nameA.y1 = static_cast<lv_coord_t>(topY + 1);
    nameA.x2 = static_cast<lv_coord_t>(battX - (showUnreadMail ? 34 : 8));
    nameA.y2 = static_cast<lv_coord_t>(topY + 24);
    lv_draw_label(drawCtx, &labelDsc, &nameA, topName.c_str(), nullptr);
    lv_area_t nameABold = nameA;
    nameABold.x1 += 1;
    nameABold.x2 += 1;
    lv_draw_label(drawCtx, &labelDsc, &nameABold, topName.c_str(), nullptr);
}

void lvglTopIndicatorsTapEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    lv_obj_t *obj = lv_event_get_target(e);
    if (!obj || !displayAwake) return;

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
    for (uint8_t i = 0; i < lvglTopIndicatorCount; i++) {
        if (lvglTopIndicators[i]) lv_obj_invalidate(lvglTopIndicators[i]);
    }
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
            if (lv_event_get_target(e) != lv_event_get_current_target(e)) return;
            if (lvglKb && !lv_obj_has_flag(lvglKb, LV_OBJ_FLAG_HIDDEN)) lvglHideKeyboard();
        },
        LV_EVENT_CLICKED,
        nullptr);
    (void)title;
    (void)backToHome;
    return scr;
}

lv_obj_t *lvglCreateMenuButton(lv_obj_t *parent, const char *txt, lv_color_t color, lv_event_cb_t cb, void *user)
{
    if (!parent) return nullptr;
    const bool compactList = (parent == lvglMediaList) || (parent == lvglWifiList) || (parent == lvglMqttCtrlList);
    lv_obj_t *btn = lv_btn_create(parent);
    if (!btn) {
        Serial.printf("[LVGL] alloc failed: menu button '%s'\n", txt ? txt : "?");
        return nullptr;
    }
    lv_obj_set_size(btn, lv_pct(100), compactList ? 38 : 44);
    lv_obj_set_style_radius(btn, 12, 0);
    lv_obj_set_style_border_width(btn, 0, 0);
    lv_obj_set_style_shadow_width(btn, compactList ? 0 : 6, 0);
    lv_obj_set_style_shadow_color(btn, lv_color_hex(0x000000), 0);
    lv_obj_set_style_shadow_opa(btn, compactList ? LV_OPA_TRANSP : LV_OPA_20, 0);
    const lv_color_t flashColor = lv_color_mix(lv_color_white(), color, UI_BUTTON_CLICK_FLASH_MIX);
    lv_obj_set_style_bg_color(btn, color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(btn, color, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
    lv_obj_add_event_cb(
        btn,
        [](lv_event_t *e) {
            lv_obj_t *obj = lv_event_get_target(e);
            if (!obj) return;
            lv_obj_add_state(obj, LV_STATE_CHECKED);
            lv_timer_t *timer = lv_timer_create(
                [](lv_timer_t *timer) {
                    lv_obj_t *target = static_cast<lv_obj_t *>(timer ? timer->user_data : nullptr);
                    if (target) lv_obj_clear_state(target, LV_STATE_CHECKED);
                    if (timer) lv_timer_del(timer);
                },
                UI_BUTTON_CLICK_FLASH_MS,
                obj);
            if (timer) lv_timer_set_repeat_count(timer, 1);
        },
        LV_EVENT_CLICKED,
        nullptr);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, user);
    lv_obj_t *lbl = lv_label_create(btn);
    if (lbl) {
        lv_label_set_text(lbl, txt);
        lv_obj_center(lbl);
    }
    return btn;
}

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
    const int idx = p2pFindPeerByPubKeyHex(peerKey);
    if (idx >= 0 && !p2pPeers[idx].name.isEmpty()) return p2pPeers[idx].name;
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
            msg.attempts = 0;
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

static bool chatHasLoggedMessageId(const String &peerKey, const String &messageId)
{
    if (peerKey.isEmpty() || messageId.isEmpty()) return false;
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
    if (!messageId.isEmpty() && !outgoing && chatHasLoggedMessageId(peerKey, messageId)) return;
    chatAppendMessageToSd(peerKey, transport, author, text, outgoing, nowMs, messageId);
    const bool conversationOpen = (uiScreen == UI_CHAT) && (currentChatPeerKey == peerKey);
    if (conversationOpen) {
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
    msg.attempts = 0;
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

    if (target.outgoing && !target.messageId.isEmpty()) {
        const int pendingIdx = chatFindPendingIndex(peerKey, target.messageId);
        if (pendingIdx >= 0) {
            for (int i = pendingIdx + 1; i < chatPendingCount; ++i) chatPendingMessages[i - 1] = chatPendingMessages[i];
            chatPendingCount--;
            chatSavePendingOutbox();
        }
    }

    if (!sdEnsureMounted(true)) return false;
    fsWriteBegin();
    bool ok = false;
    if (sdLock()) {
        String path = chatLogPathForPeer(peerKey);
        File src = SD.open(path, FILE_READ);
        if (src) {
            const String tmpPath = path + ".tmp";
            SD.remove(tmpPath);
            File dst = SD.open(tmpPath, FILE_WRITE);
            if (dst) {
                bool removed = false;
                ok = true;
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
                if (ok && SD.rename(tmpPath, path)) {
                    ok = true;
                } else {
                    SD.remove(tmpPath);
                    ok = false;
                }
            } else {
                src.close();
            }
        }
        sdUnlock();
    }
    fsWriteEnd();
    if (!ok) return false;

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

static bool chatSendAndStoreMessage(const String &peerKey, const String &text)
{
    if (peerKey.isEmpty() || text.isEmpty()) return false;
    const String messageId = chatGenerateMessageId();
    chatQueueOutgoingMessage(peerKey, deviceShortNameValue(), text, messageId);
    const bool sentLan = p2pSendChatMessageWithId(peerKey, text, messageId);
    const bool sentGlobal = mqttPublishChatMessageWithId(peerKey, text, messageId);
    chatStoreMessage(peerKey, "Me", text, true, sentGlobal ? CHAT_TRANSPORT_MQTT : CHAT_TRANSPORT_WIFI, messageId);
    uiStatusLine = sentGlobal ? "Chat sent globally" : (sentLan ? "Chat sent to peers" : "Chat saved locally");
    if (lvglReady) {
        lvglSyncStatusLine();
        if (uiScreen == UI_CHAT) lvglRefreshChatUi();
        lvglRefreshChatContactsUi();
    }
    return true;
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

static bool chatAckOutgoingMessage(const String &peerKey, const String &messageId)
{
    const int idx = chatFindPendingIndex(peerKey, messageId);
    if (idx < 0) return false;
    for (int i = idx + 1; i < chatPendingCount; ++i) chatPendingMessages[i - 1] = chatPendingMessages[i];
    chatPendingCount--;
    chatSavePendingOutbox();
    return true;
}

static void chatPendingService()
{
    if (!chatPendingLoaded) {
        if (sdMounted || sdEnsureMounted()) chatLoadPendingOutbox();
        else return;
    }

    const unsigned long now = millis();
    for (int i = 0; i < chatPendingCount; ++i) {
        PendingChatMessage &msg = chatPendingMessages[i];
        if (msg.peerKey.isEmpty() || msg.messageId.isEmpty() || msg.text.isEmpty()) continue;
        if (msg.lastAttemptMs != 0 && static_cast<unsigned long>(now - msg.lastAttemptMs) < CHAT_RETRY_INTERVAL_MS) continue;
        bool attempted = false;
        if (p2pSendChatMessageWithId(msg.peerKey, msg.text, msg.messageId)) attempted = true;
        if (mqttPublishChatMessageWithId(msg.peerKey, msg.text, msg.messageId)) attempted = true;
        if (attempted) {
            msg.lastAttemptMs = now;
            if (msg.attempts < 255) msg.attempts++;
        }
    }
}

static void chatSelectFirstEnabledPeer()
{
    currentChatPeerKey = "";
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
}

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
    for (int i = 0; i < p2pPeerCount; ++i) {
        if (p2pPeers[i].pubKeyHex.equalsIgnoreCase(pubKeyHex)) return i;
    }
    return -1;
}

static int p2pFindDiscoveredByPubKeyHex(const String &pubKeyHex)
{
    for (int i = 0; i < p2pDiscoveredCount; ++i) {
        if (p2pDiscoveredPeers[i].pubKeyHex.equalsIgnoreCase(pubKeyHex)) return i;
    }
    return -1;
}

static void p2pTouchPeerSeen(int idx, const IPAddress &ip, uint16_t port)
{
    if (idx < 0 || idx >= p2pPeerCount) return;
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
    p2pPrefs.begin("p2p", false);
    p2pPrefs.putString("pub", p2pPublicKeyHex());
    p2pPrefs.putBytes("sec", p2pSecretKey, sizeof(p2pSecretKey));
    p2pPrefs.putInt("peer_count", p2pPeerCount);
    for (int i = 0; i < MAX_P2P_PEERS; ++i) {
        p2pPrefs.putString((String("peer_name_") + i).c_str(), (i < p2pPeerCount) ? p2pPeers[i].name : "");
        p2pPrefs.putString((String("peer_key_") + i).c_str(), (i < p2pPeerCount) ? p2pPeers[i].pubKeyHex : "");
        p2pPrefs.putString((String("peer_ip_") + i).c_str(), (i < p2pPeerCount) ? p2pPeers[i].ip.toString() : "");
        p2pPrefs.putUShort((String("peer_port_") + i).c_str(), (i < p2pPeerCount) ? p2pPeers[i].port : P2P_UDP_PORT);
        p2pPrefs.putBool((String("peer_en_") + i).c_str(), (i < p2pPeerCount) ? p2pPeers[i].enabled : false);
    }
    p2pPrefs.end();
}

void loadP2pConfig()
{
    p2pPeerCount = 0;
    p2pPrefs.begin("p2p", false);

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
    for (int i = 0; i < p2pPeerCount; ++i) {
        p2pPeers[i].name = p2pPrefs.getString((String("peer_name_") + i).c_str(), "");
        p2pPeers[i].pubKeyHex = p2pPrefs.getString((String("peer_key_") + i).c_str(), "");
        p2pPeers[i].ip.fromString(p2pPrefs.getString((String("peer_ip_") + i).c_str(), ""));
        p2pPeers[i].port = p2pPrefs.getUShort((String("peer_port_") + i).c_str(), P2P_UDP_PORT);
        p2pPeers[i].enabled = p2pPrefs.getBool((String("peer_en_") + i).c_str(), true);
        p2pPeers[i].unread = false;
        p2pPeers[i].lastSeenMs = 0;
    }
    p2pPrefs.end();
    saveP2pConfig();
}

void lvglRefreshWifiList();
void lvglRefreshMediaList();
void lvglRefreshInfoPanel();
void lvglMediaPlayStopEvent(lv_event_t *e);
void lvglMediaPrevTrackEvent(lv_event_t *e);
void lvglMediaNextTrackEvent(lv_event_t *e);
void lvglMediaVolumeEvent(lv_event_t *e);
void lvglMediaVolumeStepEvent(lv_event_t *e);
void lvglHomeNavEvent(lv_event_t *e);
void lvglRecoveryHintEvent(lv_event_t *e);
void lvglWifiRescanEvent(lv_event_t *e);
void lvglWifiDisconnectEvent(lv_event_t *e);
void lvglWifiForgetEvent(lv_event_t *e);
void lvglWifiApSaveEvent(lv_event_t *e);
void lvglMediaRefreshEvent(lv_event_t *e);
void lvglOpenSnakeEvent(lv_event_t *e);
void lvglOpenTetrisEvent(lv_event_t *e);
void lvglOpenMqttCfgEvent(lv_event_t *e);
void lvglOpenMqttCtrlEvent(lv_event_t *e);
void lvglOpenChatPeersEvent(lv_event_t *e);
void lvglStyleHintEvent(lv_event_t *e);
void lvglAirplaneToggleEvent(lv_event_t *e);
void lvglChatAirplanePromptEvent(lv_event_t *e);
void lvglShowChatAirplanePrompt();
bool lvglChatPromptIfAirplaneBlocked();
void lvglScreenshotEvent(lv_event_t *e);
void lvglBrightnessEvent(lv_event_t *e);
void lvglTextAreaFocusEvent(lv_event_t *e);
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
void lvglRefreshSnakeBoard();
void lvglRefreshTetrisBoard();
void lvglSnakeBoardDrawEvent(lv_event_t *e);
void lvglSnakeRestartEvent(lv_event_t *e);
void lvglSnakeDirEvent(lv_event_t *e);
void lvglTetrisBoardDrawEvent(lv_event_t *e);
void lvglTetrisRestartEvent(lv_event_t *e);
void lvglTetrisMoveLeftEvent(lv_event_t *e);
void lvglTetrisMoveRightEvent(lv_event_t *e);
void lvglTetrisRotateEvent(lv_event_t *e);
void lvglTetrisDropEvent(lv_event_t *e);
void lvglChatPeerActionEvent(lv_event_t *e);

inline void lvglLoadScreen(lv_obj_t *target, lv_scr_load_anim_t anim)
{
    if (!target) return;
    lv_scr_load_anim(target, anim, UI_ANIM_MS, 0, false);
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
        case UI_CONFIG:
        case UI_CONFIG_MQTT_CONFIG:
        case UI_CONFIG_MQTT_CONTROLS:
        case UI_GAME_SNAKE:
        case UI_GAME_TETRIS:
            return true;
        default:
            return false;
    }
}

lv_obj_t *lvglScreenForUi(UiScreen screen)
{
    switch (screen) {
        case UI_HOME: return lvglScrHome;
        case UI_CHAT: return lvglScrChat;
        case UI_CHAT_PEERS: return lvglScrChatPeers;
        case UI_WIFI_LIST: return lvglScrWifi;
        case UI_MEDIA: return lvglScrMedia;
        case UI_INFO: return lvglScrInfo;
        case UI_GAMES: return lvglScrGames;
        case UI_CONFIG: return lvglScrConfig;
        case UI_CONFIG_MQTT_CONFIG: return lvglScrMqttCfg;
        case UI_CONFIG_MQTT_CONTROLS: return lvglScrMqttCtrl;
        case UI_GAME_SNAKE: return lvglScrSnake;
        case UI_GAME_TETRIS: return lvglScrTetris;
        default: return nullptr;
    }
}

void lvglEnsureScreenBuilt(UiScreen screen)
{
    if (lvglScreenForUi(screen)) return;

    auto makeSmallBtn = [](lv_obj_t *parent, const char *txt, int w, int h, lv_color_t col, lv_event_cb_t cb, void *ud = nullptr) -> lv_obj_t * {
        if (!parent) return nullptr;
        lv_obj_t *b = lv_btn_create(parent);
        if (!b) return nullptr;
        lv_obj_set_size(b, w, h);
        lv_obj_set_style_radius(b, 8, 0);
        lv_obj_set_style_border_width(b, 0, 0);
        const lv_color_t flashColor = lv_color_mix(lv_color_white(), col, UI_BUTTON_CLICK_FLASH_MIX);
        lv_obj_set_style_bg_color(b, col, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(b, col, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_color(b, flashColor, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(b, flashColor, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_add_event_cb(
            b,
            [](lv_event_t *e) {
                lv_obj_t *obj = lv_event_get_target(e);
                if (!obj) return;
                lv_obj_add_state(obj, LV_STATE_CHECKED);
                lv_timer_t *timer = lv_timer_create(
                    [](lv_timer_t *timer) {
                        lv_obj_t *target = static_cast<lv_obj_t *>(timer ? timer->user_data : nullptr);
                        if (target) lv_obj_clear_state(target, LV_STATE_CHECKED);
                        if (timer) lv_timer_del(timer);
                    },
                    UI_BUTTON_CLICK_FLASH_MS,
                    obj);
                if (timer) lv_timer_set_repeat_count(timer, 1);
            },
            LV_EVENT_CLICKED,
            nullptr);
        lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
        lv_obj_t *l = lv_label_create(b);
        if (l) {
            lv_label_set_text(l, txt);
            lv_obj_center(l);
        }
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
            lvglCreateMenuButton(homeWrap, "Chat", lv_color_hex(0x7A4F2F), lvglHomeNavEvent, reinterpret_cast<void *>(static_cast<intptr_t>(UI_CHAT)));
            lvglCreateMenuButton(homeWrap, "Media", lv_color_hex(0x376B93), lvglHomeNavEvent, reinterpret_cast<void *>(static_cast<intptr_t>(UI_MEDIA)));
            lvglCreateMenuButton(homeWrap, "Info", lv_color_hex(0x7750A0), lvglHomeNavEvent, reinterpret_cast<void *>(static_cast<intptr_t>(UI_INFO)));
            lvglCreateMenuButton(homeWrap, "Games", lv_color_hex(0x2B7D7D), lvglHomeNavEvent, reinterpret_cast<void *>(static_cast<intptr_t>(UI_GAMES)));
            lvglCreateMenuButton(homeWrap, "Config", lv_color_hex(0x925A73), lvglHomeNavEvent, reinterpret_cast<void *>(static_cast<intptr_t>(UI_CONFIG)));
            lvglCreateMenuButton(homeWrap, "Web Recovery", lv_color_hex(0xA66A2A), lvglRecoveryHintEvent, nullptr);
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
            lvglChatPeersBtn = makeSmallBtn(chatOps, "Peers", 62, 28, lv_color_hex(0x2F6D86), lvglOpenChatPeersEvent);

            lvglChatContactLabel = lv_label_create(chatOps);
            lv_obj_set_width(lvglChatContactLabel, lv_pct(100));
            lv_obj_set_flex_grow(lvglChatContactLabel, 1);
            lv_obj_set_style_text_align(lvglChatContactLabel, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_set_style_text_color(lvglChatContactLabel, lv_color_hex(0xC8D3DD), 0);
            lv_label_set_long_mode(lvglChatContactLabel, LV_LABEL_LONG_DOT);

            lvglChatMenuBtn = makeSmallBtn(chatOps, LV_SYMBOL_LIST, 34, 28, lv_color_hex(0x2F6D86), lvglToggleChatMenuEvent, nullptr);

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
            lv_obj_t *chatClearBtn = makeSmallBtn(lvglChatMenuPanel, "Clear", lv_pct(100), 32, lv_color_hex(0x8A3A3A), lvglDeleteChatConversationEvent, nullptr);
            if (chatClearBtn) lv_obj_set_width(chatClearBtn, lv_pct(100));
            lv_obj_t *chatClearAllBtn = makeSmallBtn(lvglChatMenuPanel, "Clear for All", lv_pct(100), 32, lv_color_hex(0x944E2B), lvglDeleteChatConversationForAllEvent, nullptr);
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

            makeSmallBtn(lvglChatComposer, "Send", 64, 38, lv_color_hex(0x3A7A3A),
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
            lv_obj_t *peerTop = lv_obj_create(lvglScrChatPeers);
            lv_obj_set_size(peerTop, lv_pct(100), 74);
            lv_obj_align(peerTop, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_color(peerTop, lv_color_hex(0x16212C), 0);
            lv_obj_set_style_border_width(peerTop, 0, 0);
            lv_obj_set_style_pad_all(peerTop, 8, 0);
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
            lvglChatPeerScanBtn = makeSmallBtn(peerOps, "Scan", 54, 26, lv_color_hex(0x2F6D86), lvglChatPeerActionEvent, reinterpret_cast<void *>(static_cast<intptr_t>(0)));

            lvglChatPeerIdentityLabel = lv_label_create(peerTop);
            lv_obj_set_width(lvglChatPeerIdentityLabel, lv_pct(100));
            lv_label_set_long_mode(lvglChatPeerIdentityLabel, LV_LABEL_LONG_WRAP);
            lv_obj_set_style_text_color(lvglChatPeerIdentityLabel, lv_color_hex(0xC8D3DD), 0);

            lvglChatPeerList = lv_obj_create(lvglScrChatPeers);
            lv_obj_set_size(lvglChatPeerList, lv_pct(100), UI_CONTENT_H - 74);
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
            makeSmallBtn(mediaCtrlRow, "Refresh", 58, 24, lv_color_hex(0x2F6D86), lvglMediaRefreshEvent);
            lvglMediaPrevBtn = makeSmallBtn(mediaCtrlRow, "Prev", 45, 24, lv_color_hex(0x355C3D), lvglMediaPrevTrackEvent);
            lvglMediaPlayBtn = makeSmallBtn(mediaCtrlRow, "Play", 45, 24, lv_color_hex(0x7C3A3A), lvglMediaPlayStopEvent);
            lvglMediaNextBtn = makeSmallBtn(mediaCtrlRow, "Next", 45, 24, lv_color_hex(0x355C3D), lvglMediaNextTrackEvent);
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
            lvglScrInfo = lvglCreateScreenBase("Info", true);
            lvglInfoList = lv_obj_create(lvglScrInfo);
            lv_obj_set_size(lvglInfoList, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(lvglInfoList, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(lvglInfoList, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(lvglInfoList, 0, 0);
            lv_obj_set_style_pad_all(lvglInfoList, 8, 0);
            lv_obj_set_style_pad_row(lvglInfoList, 6, 0);
            lv_obj_set_flex_flow(lvglInfoList, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scroll_dir(lvglInfoList, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(lvglInfoList, LV_SCROLLBAR_MODE_OFF);

            lvglCreateInfoCard(lvglInfoList, LV_SYMBOL_BATTERY_FULL, "Battery", lv_color_hex(0x52B788),
                               &lvglInfoBatteryValueLabel, &lvglInfoBatterySubLabel);
            lvglCreateInfoCard(lvglInfoList, LV_SYMBOL_WIFI, "WiFi Strength", lv_color_hex(0x4FC3F7),
                               &lvglInfoWifiValueLabel, &lvglInfoWifiSubLabel);
            lvglCreateInfoCard(lvglInfoList, LV_SYMBOL_EYE_OPEN, "Lighting", lv_color_hex(0xF4B942),
                               &lvglInfoLightValueLabel, &lvglInfoLightSubLabel);
            lvglCreateInfoCard(lvglInfoList, LV_SYMBOL_CHARGE, "Processor Load", lv_color_hex(0x7BC96F),
                               &lvglInfoCpuValueLabel, &lvglInfoCpuSubLabel, &lvglInfoCpuBar);
            lvglCreateInfoCard(lvglInfoList, LV_SYMBOL_WARNING, "Chip Temperature", lv_color_hex(0xF2C35E),
                               &lvglInfoTempValueLabel, &lvglInfoTempSubLabel, &lvglInfoTempBar);

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
            lvglCreateMenuButton(gamesWrap, "Snake", lv_color_hex(0x3A8F4B), lvglOpenSnakeEvent, nullptr);
            lvglCreateMenuButton(gamesWrap, "Tetris", lv_color_hex(0x376B93), lvglOpenTetrisEvent, nullptr);
            break;
        }
        case UI_CONFIG: {
            lvglScrConfig = lvglCreateScreenBase("Config", true);
            lvglConfigWrap = lv_obj_create(lvglScrConfig);
            lv_obj_set_size(lvglConfigWrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(lvglConfigWrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(lvglConfigWrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(lvglConfigWrap, 0, 0);
            lv_obj_set_style_pad_all(lvglConfigWrap, 10, 0);
            lv_obj_set_style_pad_row(lvglConfigWrap, 10, 0);
            lv_obj_set_flex_flow(lvglConfigWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scrollbar_mode(lvglConfigWrap, LV_SCROLLBAR_MODE_OFF);
            lvglCreateMenuButton(lvglConfigWrap, "WiFi Config", lv_color_hex(0x3A8F4B), lvglHomeNavEvent, reinterpret_cast<void *>(static_cast<intptr_t>(UI_WIFI_LIST)));
            lvglAirplaneBtn = lvglCreateMenuButton(lvglConfigWrap, "Airplane: OFF", lv_color_hex(0x8A5A25), lvglAirplaneToggleEvent, nullptr);
            if (lvglAirplaneBtn) lvglAirplaneBtnLabel = lv_obj_get_child(lvglAirplaneBtn, 0);
            lvglCreateMenuButton(lvglConfigWrap, "Style", lv_color_hex(0x2D6D8E), lvglStyleHintEvent, nullptr);
            lvglCreateMenuButton(lvglConfigWrap, "MQTT Config", lv_color_hex(0x6D4B9A), lvglOpenMqttCfgEvent, nullptr);
            lvglCreateMenuButton(lvglConfigWrap, "MQTT Controls", lv_color_hex(0x2D6D8E), lvglOpenMqttCtrlEvent, nullptr);
            lvglCreateMenuButton(lvglConfigWrap, "Screenshot", lv_color_hex(0x6B5B2A), lvglScreenshotEvent, nullptr);

            lv_obj_t *nameWrap = lv_obj_create(lvglConfigWrap);
            lv_obj_set_size(nameWrap, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(nameWrap, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(nameWrap, 0, 0);
            lv_obj_set_style_radius(nameWrap, 12, 0);
            lv_obj_set_style_pad_all(nameWrap, 10, 0);
            lv_obj_set_style_pad_row(nameWrap, 6, 0);
            lv_obj_set_flex_flow(nameWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(nameWrap, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *nameHdr = lv_label_create(nameWrap);
            lv_label_set_text(nameHdr, "Device Name");
            lv_obj_set_style_text_color(nameHdr, lv_color_hex(0xE5ECF3), 0);

            lv_obj_t *nameRow = lv_obj_create(nameWrap);
            lv_obj_set_size(nameRow, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(nameRow, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(nameRow, 0, 0);
            lv_obj_set_style_pad_all(nameRow, 0, 0);
            lv_obj_set_style_pad_column(nameRow, 8, 0);
            lv_obj_set_flex_flow(nameRow, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(nameRow, LV_OBJ_FLAG_SCROLLABLE);

            lvglConfigDeviceNameTa = lv_textarea_create(nameRow);
            lv_obj_set_height(lvglConfigDeviceNameTa, 38);
            lv_obj_set_width(lvglConfigDeviceNameTa, DISPLAY_WIDTH - 110);
            lv_obj_set_flex_grow(lvglConfigDeviceNameTa, 1);
            lv_textarea_set_one_line(lvglConfigDeviceNameTa, true);
            lv_textarea_set_placeholder_text(lvglConfigDeviceNameTa, "Device name");
            lv_textarea_set_max_length(lvglConfigDeviceNameTa, 24);
            lv_obj_add_event_cb(lvglConfigDeviceNameTa, lvglTextAreaFocusEvent, LV_EVENT_FOCUSED, nullptr);

            makeSmallBtn(nameRow, "Save", 64, 38, lv_color_hex(0x3A7A3A), lvglSaveDeviceNameEvent, nullptr);

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

            lv_obj_t *brightLbl = lv_label_create(brightHdr);
            lv_label_set_text(brightLbl, "Brightness");
            lv_obj_set_style_text_color(brightLbl, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_flex_grow(brightLbl, 1);

            lvglBrightnessValueLabel = lv_label_create(brightHdr);
            lv_obj_set_style_text_color(lvglBrightnessValueLabel, lv_color_hex(0xB7C4D1), 0);

            lvglBrightnessSlider = lv_slider_create(brightWrap);
            lv_obj_set_width(lvglBrightnessSlider, lv_pct(100));
            lv_slider_set_range(lvglBrightnessSlider, 5, 100);
            lv_obj_add_event_cb(lvglBrightnessSlider, lvglBrightnessEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lvglRefreshConfigUi();
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
                lv_textarea_set_one_line(ta, true);
                lv_textarea_set_placeholder_text(ta, ph);
                lv_textarea_set_password_mode(ta, pass);
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
            lv_obj_set_flex_grow(lvglMqttPassTa, 1);
            lv_textarea_set_one_line(lvglMqttPassTa, true);
            lv_textarea_set_placeholder_text(lvglMqttPassTa, "Password");
            lv_textarea_set_password_mode(lvglMqttPassTa, true);
            lv_obj_add_event_cb(lvglMqttPassTa, lvglTextAreaFocusEvent, LV_EVENT_FOCUSED, nullptr);

            lvglMqttPassShowBtn = lv_btn_create(mqttPwdRow);
            lv_obj_set_size(lvglMqttPassShowBtn, 34, 34);
            lv_obj_set_style_radius(lvglMqttPassShowBtn, 8, 0);
            lv_obj_set_style_border_width(lvglMqttPassShowBtn, 0, 0);
            lv_obj_set_style_bg_color(lvglMqttPassShowBtn, lv_color_hex(0x3F4A57), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(lvglMqttPassShowBtn, lv_color_hex(0x526274), LV_PART_MAIN | LV_STATE_PRESSED);
            lv_obj_set_style_shadow_width(lvglMqttPassShowBtn, 0, 0);
            lv_obj_add_event_cb(lvglMqttPassShowBtn, lvglMqttPassShowToggleEvent, LV_EVENT_CLICKED, nullptr);
            lvglMqttPassShowBtnLabel = lv_label_create(lvglMqttPassShowBtn);
            lv_label_set_text(lvglMqttPassShowBtnLabel, LV_SYMBOL_EYE_CLOSE);
            lv_obj_center(lvglMqttPassShowBtnLabel);

            lvglMqttDiscTa = addTa("Discovery prefix", false, false);
            lvglMqttChatTa = nullptr;
            lv_obj_t *actRow = lv_obj_create(formWrap);
            lv_obj_set_size(actRow, lv_pct(100), 40);
            lv_obj_set_style_bg_opa(actRow, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(actRow, 0, 0);
            lv_obj_set_style_pad_all(actRow, 0, 0);
            lv_obj_set_style_pad_column(actRow, 6, 0);
            lv_obj_set_flex_flow(actRow, LV_FLEX_FLOW_ROW);
            lv_obj_set_flex_align(actRow, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
            lv_obj_clear_flag(actRow, LV_OBJ_FLAG_SCROLLABLE);
            makeSmallBtn(actRow, "Save", 58, 30, lv_color_hex(0x3A7A3A), lvglMqttSaveEvent);
            makeSmallBtn(actRow, "Connect", 70, 30, lv_color_hex(0x2F6D86), lvglMqttConnectEvent);
            makeSmallBtn(actRow, "Discover", 82, 30, lv_color_hex(0x2F6D86), lvglMqttPublishDiscoveryEvent);

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
        case UI_CONFIG_MQTT_CONTROLS:
            lvglScrMqttCtrl = lvglCreateScreenBase("MQTT Controls", false);
            lvglMqttCtrlList = lv_obj_create(lvglScrMqttCtrl);
            lv_obj_set_size(lvglMqttCtrlList, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(lvglMqttCtrlList, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_color(lvglMqttCtrlList, lv_color_hex(0x111922), 0);
            lv_obj_set_style_border_width(lvglMqttCtrlList, 0, 0);
            lv_obj_set_style_pad_all(lvglMqttCtrlList, 8, 0);
            lv_obj_set_style_pad_row(lvglMqttCtrlList, 6, 0);
            lv_obj_set_flex_flow(lvglMqttCtrlList, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scroll_dir(lvglMqttCtrlList, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(lvglMqttCtrlList, LV_SCROLLBAR_MODE_OFF);
            lvglRefreshMqttControlsUi();
            break;
        case UI_GAME_SNAKE: {
            lvglScrSnake = lvglCreateScreenBase("Snake", false);
            lvglSnakeScoreLabel = lv_label_create(lvglScrSnake);
            if (lvglSnakeScoreLabel) lv_obj_align(lvglSnakeScoreLabel, LV_ALIGN_TOP_LEFT, 10, UI_CONTENT_TOP_Y + 6);
            lvglSnakeBoardObj = lv_obj_create(lvglScrSnake);
            if (lvglSnakeBoardObj) {
                lv_obj_set_size(lvglSnakeBoardObj, 150, 174);
                lv_obj_align(lvglSnakeBoardObj, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y + 30);
                lv_obj_set_style_bg_color(lvglSnakeBoardObj, lv_color_hex(0x0C1218), 0);
                lv_obj_set_style_border_width(lvglSnakeBoardObj, 0, 0);
                lv_obj_set_style_radius(lvglSnakeBoardObj, 4, 0);
                lv_obj_clear_flag(lvglSnakeBoardObj, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_add_event_cb(lvglSnakeBoardObj, lvglSnakeBoardDrawEvent, LV_EVENT_DRAW_MAIN, nullptr);
            }
            lv_obj_t *snakeCtl = lv_obj_create(lvglScrSnake);
            lv_obj_set_size(snakeCtl, lv_pct(100), 58);
            lv_obj_align(snakeCtl, LV_ALIGN_BOTTOM_MID, 0, 0);
            lv_obj_set_style_bg_opa(snakeCtl, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(snakeCtl, 0, 0);
            lv_obj_set_style_pad_all(snakeCtl, 6, 0);
            lv_obj_set_style_pad_column(snakeCtl, 6, 0);
            lv_obj_set_flex_flow(snakeCtl, LV_FLEX_FLOW_ROW_WRAP);
            makeSmallBtn(snakeCtl, "Restart", 70, 24, lv_color_hex(0x8A5A25), lvglSnakeRestartEvent);
            makeSmallBtn(snakeCtl, "U", 30, 24, lv_color_hex(0x2F6D86), lvglSnakeDirEvent, reinterpret_cast<void *>(static_cast<intptr_t>(0)));
            makeSmallBtn(snakeCtl, "D", 30, 24, lv_color_hex(0x2F6D86), lvglSnakeDirEvent, reinterpret_cast<void *>(static_cast<intptr_t>(2)));
            makeSmallBtn(snakeCtl, "L", 30, 24, lv_color_hex(0x2F6D86), lvglSnakeDirEvent, reinterpret_cast<void *>(static_cast<intptr_t>(3)));
            makeSmallBtn(snakeCtl, "R", 30, 24, lv_color_hex(0x2F6D86), lvglSnakeDirEvent, reinterpret_cast<void *>(static_cast<intptr_t>(1)));
            snakeResetGame();
            lvglRefreshSnakeBoard();
            break;
        }
        case UI_GAME_TETRIS: {
            lvglScrTetris = lvglCreateScreenBase("Tetris", false);
            lvglTetrisScoreLabel = lv_label_create(lvglScrTetris);
            if (lvglTetrisScoreLabel) lv_obj_align(lvglTetrisScoreLabel, LV_ALIGN_TOP_LEFT, 10, UI_CONTENT_TOP_Y + 6);
            lvglTetrisBoardObj = lv_obj_create(lvglScrTetris);
            if (lvglTetrisBoardObj) {
                lv_obj_set_size(lvglTetrisBoardObj, 110, 170);
                lv_obj_align(lvglTetrisBoardObj, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y + 28);
                lv_obj_set_style_bg_color(lvglTetrisBoardObj, lv_color_hex(0x0C1218), 0);
                lv_obj_set_style_border_width(lvglTetrisBoardObj, 0, 0);
                lv_obj_set_style_radius(lvglTetrisBoardObj, 4, 0);
                lv_obj_clear_flag(lvglTetrisBoardObj, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_add_event_cb(lvglTetrisBoardObj, lvglTetrisBoardDrawEvent, LV_EVENT_DRAW_MAIN, nullptr);
            }
            lv_obj_t *tCtl = lv_obj_create(lvglScrTetris);
            lv_obj_set_size(tCtl, lv_pct(100), 58);
            lv_obj_align(tCtl, LV_ALIGN_BOTTOM_MID, 0, 0);
            lv_obj_set_style_bg_opa(tCtl, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(tCtl, 0, 0);
            lv_obj_set_style_pad_all(tCtl, 6, 0);
            lv_obj_set_style_pad_column(tCtl, 6, 0);
            lv_obj_set_flex_flow(tCtl, LV_FLEX_FLOW_ROW_WRAP);
            makeSmallBtn(tCtl, "Restart", 70, 24, lv_color_hex(0x8A5A25), lvglTetrisRestartEvent);
            makeSmallBtn(tCtl, "L", 30, 24, lv_color_hex(0x2F6D86), lvglTetrisMoveLeftEvent);
            makeSmallBtn(tCtl, "R", 30, 24, lv_color_hex(0x2F6D86), lvglTetrisMoveRightEvent);
            makeSmallBtn(tCtl, "Rot", 40, 24, lv_color_hex(0x2F6D86), lvglTetrisRotateEvent);
            makeSmallBtn(tCtl, "Drop", 44, 24, lv_color_hex(0x2F6D86), lvglTetrisDropEvent);
            tetrisResetGame();
            lvglRefreshTetrisBoard();
            break;
        }
        default:
            break;
    }
}

void lvglOpenScreen(UiScreen screen, lv_scr_load_anim_t anim)
{
    const UiScreen prevScreen = uiScreen;
    lvglEnsureScreenBuilt(screen);
    lv_obj_t *target = lvglScreenForUi(screen);
    if (!target) return;
    uiScreen = screen;
    if (prevScreen == UI_CHAT && screen != UI_CHAT) chatClearCache();
    if (screen == UI_WIFI_LIST) {
        lvglRefreshWifiList();
    } else if (screen == UI_CHAT) {
        currentChatPeerKey = "";
        chatClearCache();
        lvglRefreshChatLayout();
        lvglRefreshChatContactsUi();
        lvglRefreshChatUi();
    } else if (screen == UI_CHAT_PEERS) {
        lvglRefreshChatPeerUi();
    } else if (screen == UI_MEDIA) {
        lvglQueueMediaRefresh();
    } else if (screen == UI_INFO) {
        lvglRefreshInfoPanel();
    } else if (screen == UI_CONFIG) {
        lvglRefreshConfigUi();
    }
    lvglRefreshTopIndicators();
    lvglLoadScreen(target, anim);
}

void lvglNavigateBackBySwipe()
{
    const UiScreen prev = uiScreen;
    if (!lvglReady) return;
    switch (uiScreen) {
        case UI_CHAT:
            if (!currentChatPeerKey.isEmpty()) {
                currentChatPeerKey = "";
                chatClearCache();
                lvglRefreshChatLayout();
                lvglRefreshChatContactsUi();
                lvglRefreshChatUi();
            } else {
                lvglOpenScreen(UI_HOME, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
            }
            break;
        case UI_MEDIA:
        case UI_INFO:
        case UI_GAMES:
        case UI_CONFIG:
            lvglOpenScreen(UI_HOME, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
            break;
        case UI_CHAT_PEERS:
            lvglOpenScreen(UI_CHAT, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
            break;
        case UI_WIFI_LIST:
            lvglOpenScreen(UI_CONFIG, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
            break;
        case UI_GAME_SNAKE:
        case UI_GAME_TETRIS:
            lvglOpenScreen(UI_GAMES, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
            break;
        case UI_CONFIG_MQTT_CONFIG:
        case UI_CONFIG_MQTT_CONTROLS:
            lvglOpenScreen(UI_CONFIG, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
            break;
        default:
            break;
    }
    if (prev == UI_MEDIA && networkSuspendedForAudio && !mediaIsPlaying && !mediaPaused) {
        networkResumeAfterAudio();
    }
}

void lvglHomeNavEvent(lv_event_t *e)
{
    const UiScreen target = static_cast<UiScreen>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (target == UI_CHAT && airplaneModeEnabled) {
        lvglShowChatAirplanePrompt();
        return;
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
        uiStatusLine = "Airplane mode: chat opened read-only";
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
    lv_obj_t *m = lv_msgbox_create(nullptr, "Chat Unavailable",
                                   "Airplane mode is on.\nTurn it off to use chat.",
                                   btns, false);
    if (!m) return;
    lv_obj_center(m);
    lv_obj_set_width(m, min<int16_t>(DISPLAY_WIDTH - 24, 260));
    lv_obj_add_event_cb(m, lvglChatAirplanePromptEvent, LV_EVENT_VALUE_CHANGED, nullptr);
}

bool lvglChatPromptIfAirplaneBlocked()
{
    if (!airplaneModeEnabled) return false;
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

void lvglRecoveryHintEvent(lv_event_t *e)
{
    (void)e;
    uiStatusLine = "Open /recovery in browser";
    lvglSyncStatusLine();
}

void lvglWifiRescanEvent(lv_event_t *e)
{
    (void)e;
    refreshWifiScan();
    lvglRefreshWifiList();
}

void lvglWifiDisconnectEvent(lv_event_t *e)
{
    (void)e;
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
    pendingSaveCreds = false;
    pendingSaveSsid = "";
    pendingSavePass = "";
    saveStaCreds("", "");
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
    if (audioBackendReady && audio) audioSetVolumeImmediate(audioVolumeLevelFromPercent(mediaVolumePercent));
    lvglRefreshMediaPlayerUi();
}

void lvglMediaVolumeStepEvent(lv_event_t *e)
{
    const int step = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    int v = static_cast<int>(mediaVolumePercent) + step;
    if (v < 0) v = 0;
    if (v > 100) v = 100;
    mediaVolumePercent = static_cast<uint8_t>(v);
    if (lvglMediaVolSlider) lv_slider_set_value(lvglMediaVolSlider, static_cast<int32_t>(mediaVolumePercent), LV_ANIM_OFF);
    if (audioBackendReady && audio) audioSetVolumeImmediate(audioVolumeLevelFromPercent(mediaVolumePercent));
    lvglRefreshMediaPlayerUi();
}

void lvglRefreshMediaPlayerUi()
{
    if (!lvglMediaTrackLabel || !lvglMediaPlayBtnLabel || !lvglMediaProgressBar || !lvglMediaProgressLabel) return;

    if (mediaSelectedTrackName.isEmpty() && !mediaNowPlaying.isEmpty()) mediaSelectedTrackName = mediaNowPlaying;
    const bool hasTrack = !mediaSelectedSourcePath.isEmpty();
    lvglSetMediaPlayerVisible(hasTrack);

    String trackText = hasTrack ? mediaSelectedTrackName : String("No track selected");
    if (trackText.length() > 34) trackText = trackText.substring(0, 31) + "...";
    lv_label_set_text_fmt(lvglMediaTrackLabel, "Track: %s", trackText.c_str());

    lv_label_set_text(lvglMediaPlayBtnLabel, (mediaIsPlaying || mediaPaused) ? "Stop" : "Play");

    if (lvglMediaVolSlider && lv_slider_get_value(lvglMediaVolSlider) != static_cast<int32_t>(mediaVolumePercent)) {
        lv_slider_set_value(lvglMediaVolSlider, static_cast<int32_t>(mediaVolumePercent), LV_ANIM_OFF);
    }
    if (lvglMediaVolValueLabel) lv_label_set_text_fmt(lvglMediaVolValueLabel, "%u%%", static_cast<unsigned int>(mediaVolumePercent));

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
    lv_bar_set_value(lvglMediaProgressBar, progress, LV_ANIM_OFF);

    char curBuf[8] = {0};
    char durBuf[8] = {0};
    mediaFormatSeconds(curSec, curBuf, sizeof(curBuf));
    mediaFormatSeconds(durSec, durBuf, sizeof(durBuf));
    lv_label_set_text_fmt(lvglMediaProgressLabel, "%s / %s", curBuf, durBuf);
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
        const bool pendingDelivery = chatMessages[i].outgoing &&
                                     chatMessagePendingForPeer(currentChatPeerKey, chatMessages[i].messageId);
        const bool deliveredOutgoing = chatMessages[i].outgoing && !pendingDelivery;
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
            lv_obj_set_style_bg_color(statusBadge, pendingDelivery ? lv_color_hex(0x5A3A1E) : lv_color_hex(0x1E4D2B), 0);
            lv_obj_set_style_border_width(statusBadge, 0, 0);
            lv_obj_set_style_radius(statusBadge, 9, 0);
            lv_obj_set_style_pad_left(statusBadge, 3, 0);
            lv_obj_set_style_pad_right(statusBadge, 3, 0);
            lv_obj_set_style_pad_top(statusBadge, pendingDelivery ? 4 : 2, 0);
            lv_obj_set_style_pad_bottom(statusBadge, pendingDelivery ? 4 : 2, 0);
            lv_obj_set_style_pad_row(statusBadge, 0, 0);
            lv_obj_clear_flag(statusBadge, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_t *statusLbl = lv_label_create(statusBadge);
            lv_label_set_text(statusLbl, pendingDelivery ? "A\nI\nR" : LV_SYMBOL_OK);
            lv_obj_set_style_text_color(statusLbl, pendingDelivery ? lv_color_hex(0xFFD27A) : lv_color_hex(0x9BF0AE), 0);
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
        lv_label_set_text(body, chatMessages[i].text.c_str());
        lv_obj_set_style_text_color(body, lv_color_hex(0xEDF2F7), 0);

        lv_obj_t *delBtn = lv_btn_create(row);
        lv_obj_set_size(delBtn, 24, 24);
        lv_obj_set_style_bg_color(delBtn, lv_color_hex(0x7A2E2E), 0);
        lv_obj_set_style_border_width(delBtn, 0, 0);
        lv_obj_set_style_radius(delBtn, 12, 0);
        lv_obj_set_style_pad_all(delBtn, 0, 0);
        lv_obj_clear_flag(delBtn, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(delBtn, lvglDeleteChatMessageEvent, LV_EVENT_CLICKED, reinterpret_cast<void *>(static_cast<intptr_t>(i)));
        lv_obj_t *delLbl = lv_label_create(delBtn);
        lv_label_set_text(delLbl, LV_SYMBOL_TRASH);
        lv_obj_set_style_text_color(delLbl, lv_color_hex(0xF5D2D2), 0);
        lv_obj_center(delLbl);
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
    chatApplyConversationDeletion(peerKey, "Conversation deleted");
    uiStatusLine = sentGlobal ? "Conversation deleted for all" : (sentLan ? "Conversation delete sent" : "Deleted locally only");
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
    for (int i = 0; i < p2pPeerCount; ++i) {
        if (!p2pPeers[i].enabled) continue;
        hasContacts = true;
        break;
    }

    if (!hasContacts) {
        lv_obj_t *lbl = lv_label_create(lvglChatContacts);
        lv_label_set_text(lbl, "No chats yet");
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xC8CED6), 0);
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
        lv_obj_set_style_bg_color(b, col, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(b, col, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
        return b;
    };

    for (int i = 0; i < p2pPeerCount; ++i) {
        if (!p2pPeers[i].enabled) continue;
        lv_color_t col = p2pPeers[i].pubKeyHex == currentChatPeerKey ? lv_color_hex(0x3A7A3A) : lv_color_hex(0x2F6D86);
        lv_obj_t *btn = makeContactBtn(
            lvglChatContacts,
            p2pPeers[i].name.isEmpty() ? "Peer" : p2pPeers[i].name.c_str(),
            col,
            [](lv_event_t *e) {
                const int idx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
                if (idx < 0 || idx >= p2pPeerCount) return;
                currentChatPeerKey = p2pPeers[idx].pubKeyHex;
                chatSetPeerUnread(currentChatPeerKey, false);
                chatReloadRecentMessagesFromSd(currentChatPeerKey);
                lvglRefreshChatLayout();
                lvglRefreshChatContactsUi();
                lvglRefreshChatUi();
                lvglRefreshTopIndicators();
            },
            reinterpret_cast<void *>(static_cast<intptr_t>(i)));
        if (!btn) continue;

        lv_obj_t *nameLabel = lv_label_create(btn);
        if (nameLabel) {
            lv_label_set_text(nameLabel, p2pPeers[i].name.isEmpty() ? "Peer" : p2pPeers[i].name.c_str());
            lv_obj_align(nameLabel, LV_ALIGN_LEFT_MID, 10, 0);
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
}

void lvglSetChatPeerScanButtonStatus(const char *text, uint32_t revertDelayMs)
{
    if (!lvglChatPeerScanBtn) return;
    lv_obj_t *label = lv_obj_get_child(lvglChatPeerScanBtn, 0);
    if (!label) return;
    lv_label_set_text(label, text ? text : "Scan");
    if (revertDelayMs == 0) return;
    lv_timer_t *timer = lv_timer_create(
        [](lv_timer_t *timer) {
            lv_obj_t *btn = static_cast<lv_obj_t *>(timer ? timer->user_data : nullptr);
            if (btn) {
                lv_obj_t *lbl = lv_obj_get_child(btn, 0);
                if (lbl) lv_label_set_text(lbl, "Scan");
            }
            if (timer) lv_timer_del(timer);
        },
        revertDelayMs,
        lvglChatPeerScanBtn);
    if (timer) lv_timer_set_repeat_count(timer, 1);
}

void lvglChatPeerActionEvent(lv_event_t *e)
{
    const int action = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (action == 0) {
        lvglSetChatPeerScanButtonStatus("Scanning...");
        p2pLastDiscoverAnnounceMs = 0;
        p2pBroadcastDiscover();
        uiStatusLine = "Peer scan started";
        lvglSyncStatusLine();
        lvglRefreshChatPeerUi();
        lvglSetChatPeerScanButtonStatus("Done", 1500);
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
        const bool ok = p2pAddOrUpdateTrustedPeer(
            p2pDiscoveredPeers[idx].name.isEmpty() ? String("Peer") : p2pDiscoveredPeers[idx].name,
            p2pDiscoveredPeers[idx].pubKeyHex,
            p2pDiscoveredPeers[idx].ip,
            p2pDiscoveredPeers[idx].port);
        if (ok && currentChatPeerKey.isEmpty()) currentChatPeerKey = p2pDiscoveredPeers[idx].pubKeyHex;
        uiStatusLine = ok ? "Peer paired" : "Pair failed";
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
            for (int i = pidx + 1; i < p2pPeerCount; ++i) p2pPeers[i - 1] = p2pPeers[i];
            p2pPeerCount--;
            saveP2pConfig();
            p2pDiscoveredPeers[idx].trusted = false;
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
        lv_obj_set_style_bg_color(b, col, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(b, col, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
        lv_obj_t *l = lv_label_create(b);
        if (l) {
            lv_label_set_text(l, txt);
            lv_obj_center(l);
        }
        return b;
    };

    bool anyEntries = false;

    if (p2pPeerCount > 0) {
        lv_obj_t *section = lv_label_create(lvglChatPeerList);
        lv_obj_set_width(section, lv_pct(100));
        lv_label_set_text(section, "Paired Devices");
        lv_obj_set_style_text_color(section, lv_color_hex(0xE5ECF3), 0);

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

            lv_obj_t *title = lv_label_create(card);
            lv_label_set_text_fmt(title, "%s  [%s]", p2pPeers[i].name.isEmpty() ? "Peer" : p2pPeers[i].name.c_str(), enabled ? "paired" : "disabled");
            lv_obj_set_style_text_color(title, lv_color_hex(0xE5ECF3), 0);

            lv_obj_t *info = lv_label_create(card);
            lv_obj_set_width(info, lv_pct(100));
            lv_label_set_long_mode(info, LV_LABEL_LONG_WRAP);
            String key = p2pPeers[i].pubKeyHex;
            if (key.length() > 16) key = key.substring(0, 16) + "...";
            lv_label_set_text_fmt(info, "%s:%u\n%s", p2pPeers[i].ip.toString().c_str(),
                                  static_cast<unsigned int>(p2pPeers[i].port), key.c_str());
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

        lv_obj_t *title = lv_label_create(card);
        lv_label_set_text_fmt(title, "%s  [discovered]", p2pDiscoveredPeers[i].name.c_str());
        lv_obj_set_style_text_color(title, lv_color_hex(0xE5ECF3), 0);

        lv_obj_t *info = lv_label_create(card);
        lv_obj_set_width(info, lv_pct(100));
        lv_label_set_long_mode(info, LV_LABEL_LONG_WRAP);
        String key = p2pDiscoveredPeers[i].pubKeyHex;
        if (key.length() > 16) key = key.substring(0, 16) + "...";
        lv_label_set_text_fmt(info, "%s:%u\n%s", p2pDiscoveredPeers[i].ip.toString().c_str(),
                              static_cast<unsigned int>(p2pDiscoveredPeers[i].port), key.c_str());
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

    if (!anyEntries) {
        lv_obj_t *lbl = lv_label_create(lvglChatPeerList);
        lv_obj_set_width(lbl, lv_pct(100));
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_WRAP);
        lv_label_set_text(lbl, "No paired or discovered peers yet.\nOpen this screen on another device on the same WiFi, then tap Scan.");
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xC8CED6), 0);
    }
}

void p2pEnsureUdp()
{
    if (!p2pReady) return;
    if (p2pUdpStarted) return;
    if (WiFi.getMode() == WIFI_OFF) return;
    if (p2pUdp.begin(P2P_UDP_PORT) == 1) {
        p2pUdpStarted = true;
        Serial.printf("[P2P] UDP listening on %u\n", static_cast<unsigned int>(P2P_UDP_PORT));
    }
}

static bool p2pSendPacketToPeer(const P2PPeer &peer, uint8_t type, const uint8_t *plain, size_t plainLen)
{
    if (!peer.enabled || plainLen == 0 || plainLen > (P2P_MAX_PACKET - 64)) return false;
    unsigned char peerPk[P2P_PUBLIC_KEY_BYTES] = {0};
    if (!p2pHexToBytes(peer.pubKeyHex, peerPk, sizeof(peerPk))) return false;

    unsigned char nonce[P2P_NONCE_BYTES] = {0};
    randombytes_buf(nonce, sizeof(nonce));
    const size_t cipherLen = plainLen + P2P_MAC_BYTES;
    unsigned char cipher[P2P_MAX_PACKET] = {0};
    if (crypto_box_curve25519xchacha20poly1305_easy(cipher, plain, plainLen, nonce, peerPk, p2pSecretKey) != 0) return false;

    uint8_t packet[P2P_MAX_PACKET] = {0};
    size_t pos = 0;
    packet[pos++] = P2P_PKT_MAGIC_0;
    packet[pos++] = P2P_PKT_MAGIC_1;
    packet[pos++] = P2P_PKT_VERSION;
    packet[pos++] = type;
    memcpy(packet + pos, p2pPublicKey, sizeof(p2pPublicKey));
    pos += sizeof(p2pPublicKey);
    memcpy(packet + pos, nonce, sizeof(nonce));
    pos += sizeof(nonce);
    packet[pos++] = static_cast<uint8_t>((cipherLen >> 8) & 0xFFU);
    packet[pos++] = static_cast<uint8_t>(cipherLen & 0xFFU);
    memcpy(packet + pos, cipher, cipherLen);
    pos += cipherLen;

    p2pEnsureUdp();
    if (!p2pUdpStarted) return false;
    if (!p2pUdp.beginPacket(peer.ip, peer.port ? peer.port : P2P_UDP_PORT)) return false;
    p2pUdp.write(packet, pos);
    return p2pUdp.endPacket() == 1;
}

bool p2pSendChatMessageWithId(const String &peerKey, const String &text, const String &messageId)
{
    if (!p2pReady) return false;
    if (peerKey.isEmpty() || text.isEmpty() || messageId.isEmpty()) return false;
    const int peerIdx = p2pFindPeerByPubKeyHex(peerKey);
    if (peerIdx < 0 || !p2pPeers[peerIdx].enabled || p2pPeers[peerIdx].ip == IPAddress((uint32_t)0)) return false;

    JsonDocument doc;
    doc["kind"] = "chat";
    doc["id"] = messageId;
    doc["author"] = deviceShortNameValue();
    doc["text"] = text.substring(0, P2P_MAX_CHAT_TEXT);
    char plain[P2P_MAX_PACKET] = {0};
    const size_t plainLen = serializeJson(doc, plain, sizeof(plain));
    if (plainLen == 0) return false;

    return p2pSendPacketToPeer(p2pPeers[peerIdx], P2P_PKT_TYPE_CHAT, reinterpret_cast<const uint8_t *>(plain), plainLen);
}

bool p2pSendChatMessage(const String &text)
{
    if (currentChatPeerKey.isEmpty()) return false;
    const String messageId = chatGenerateMessageId();
    return p2pSendChatMessageWithId(currentChatPeerKey, text, messageId);
}

bool p2pSendChatAck(const String &peerKey, const String &messageId)
{
    if (!p2pReady || peerKey.isEmpty() || messageId.isEmpty()) return false;
    const int peerIdx = p2pFindPeerByPubKeyHex(peerKey);
    if (peerIdx < 0 || !p2pPeers[peerIdx].enabled || p2pPeers[peerIdx].ip == IPAddress((uint32_t)0)) return false;

    JsonDocument doc;
    doc["kind"] = "ack";
    doc["ack_id"] = messageId;
    char plain[P2P_MAX_PACKET] = {0};
    const size_t plainLen = serializeJson(doc, plain, sizeof(plain));
    if (plainLen == 0) return false;

    return p2pSendPacketToPeer(p2pPeers[peerIdx], P2P_PKT_TYPE_CHAT, reinterpret_cast<const uint8_t *>(plain), plainLen);
}

bool p2pSendConversationDelete()
{
    if (!p2pReady || currentChatPeerKey.isEmpty()) return false;
    const int peerIdx = p2pFindPeerByPubKeyHex(currentChatPeerKey);
    if (peerIdx < 0 || !p2pPeers[peerIdx].enabled || p2pPeers[peerIdx].ip == IPAddress((uint32_t)0)) return false;

    JsonDocument doc;
    doc["kind"] = "delete_conversation";
    doc["author"] = deviceShortNameValue();
    char plain[P2P_MAX_PACKET] = {0};
    const size_t plainLen = serializeJson(doc, plain, sizeof(plain));
    if (plainLen == 0) return false;

    return p2pSendPacketToPeer(p2pPeers[peerIdx], P2P_PKT_TYPE_CHAT, reinterpret_cast<const uint8_t *>(plain), plainLen);
}

void p2pBroadcastDiscover()
{
    if (!p2pReady || !wifiConnectedSafe()) return;
    p2pEnsureUdp();
    if (!p2pUdpStarted) return;

    JsonDocument doc;
    doc["kind"] = "discover";
    doc["device"] = deviceShortNameValue();
    doc["public_key"] = p2pPublicKeyHex();
    doc["port"] = P2P_UDP_PORT;
    char payload[256] = {0};
    const size_t len = serializeJson(doc, payload, sizeof(payload));
    if (len == 0) return;

    IPAddress bcast = WiFi.localIP();
    bcast[3] = 255;
    if (p2pUdp.beginPacket(bcast, P2P_UDP_PORT)) {
        p2pUdp.write(reinterpret_cast<const uint8_t *>("\x50\x32\x01\x02"), 4);
        p2pUdp.write(reinterpret_cast<const uint8_t *>(payload), len);
        p2pUdp.endPacket();
    }
}

bool p2pAddOrUpdateTrustedPeer(const String &name, const String &pubKeyHex, const IPAddress &ip, uint16_t port)
{
    unsigned char testPk[P2P_PUBLIC_KEY_BYTES] = {0};
    if (name.isEmpty() || !p2pHexToBytes(pubKeyHex, testPk, sizeof(testPk))) return false;
    int idx = p2pFindPeerByPubKeyHex(pubKeyHex);
    if (idx < 0) {
        if (p2pPeerCount >= MAX_P2P_PEERS) return false;
        idx = p2pPeerCount++;
    }
    p2pPeers[idx].name = name;
    p2pPeers[idx].pubKeyHex = pubKeyHex;
    p2pPeers[idx].ip = ip;
    p2pPeers[idx].port = (port == 0) ? P2P_UDP_PORT : port;
    p2pPeers[idx].enabled = true;
    p2pPeers[idx].unread = false;
    p2pPeers[idx].lastSeenMs = millis();
    saveP2pConfig();
    p2pTouchDiscoveredSeen(name, pubKeyHex, ip, port);
    return true;
}

void p2pService()
{
    if (!p2pReady) return;
    p2pEnsureUdp();
    if (!p2pUdpStarted) return;

    const unsigned long now = millis();
    if (wifiConnectedSafe() && (now - p2pLastDiscoverAnnounceMs) >= 5000UL) {
        p2pLastDiscoverAnnounceMs = now;
        p2pBroadcastDiscover();
    }

    int packetLen = p2pUdp.parsePacket();
    while (packetLen > 0) {
        if (packetLen >= 4 && packetLen <= static_cast<int>(P2P_MAX_PACKET)) {
            uint8_t packet[P2P_MAX_PACKET] = {0};
            const int got = p2pUdp.read(packet, sizeof(packet));
            if (got >= 0) {
                size_t pos = 0;
                const bool headerOk = packet[pos++] == P2P_PKT_MAGIC_0 &&
                                      packet[pos++] == P2P_PKT_MAGIC_1 &&
                                      packet[pos++] == P2P_PKT_VERSION;
                const uint8_t type = packet[pos++];
                if (headerOk && type == P2P_PKT_TYPE_DISCOVER) {
                    JsonDocument doc;
                    if (deserializeJson(doc, packet + pos, got - static_cast<int>(pos)) == DeserializationError::Ok) {
                        const String kind = String(static_cast<const char *>(doc["kind"] | ""));
                        const String device = String(static_cast<const char *>(doc["device"] | ""));
                        const String pubKeyHex = String(static_cast<const char *>(doc["public_key"] | ""));
                        const uint16_t port = static_cast<uint16_t>(doc["port"] | P2P_UDP_PORT);
                        if (kind == "discover" && !pubKeyHex.equalsIgnoreCase(p2pPublicKeyHex())) {
                            p2pTouchDiscoveredSeen(device, pubKeyHex, p2pUdp.remoteIP(), port);
                        }
                    }
                } else if (headerOk && type == P2P_PKT_TYPE_CHAT &&
                           got >= static_cast<int>(4 + P2P_PUBLIC_KEY_BYTES + P2P_NONCE_BYTES + 2)) {
                    unsigned char senderPk[P2P_PUBLIC_KEY_BYTES] = {0};
                    unsigned char nonce[P2P_NONCE_BYTES] = {0};
                    memcpy(senderPk, packet + pos, sizeof(senderPk));
                    pos += sizeof(senderPk);
                    memcpy(nonce, packet + pos, sizeof(nonce));
                    pos += sizeof(nonce);
                    const size_t cipherLen = (static_cast<size_t>(packet[pos]) << 8) | static_cast<size_t>(packet[pos + 1]);
                    pos += 2;
                    if ((pos + cipherLen) <= static_cast<size_t>(got) && cipherLen >= P2P_MAC_BYTES) {
                        const String senderHex = p2pBytesToHex(senderPk, sizeof(senderPk));
                        const int peerIdx = p2pFindPeerByPubKeyHex(senderHex);
                        if (peerIdx >= 0 && p2pPeers[peerIdx].enabled) {
                            unsigned char plain[P2P_MAX_PACKET] = {0};
                            if (crypto_box_curve25519xchacha20poly1305_open_easy(
                                    plain,
                                    packet + pos,
                                    cipherLen,
                                    nonce,
                                    senderPk,
                                    p2pSecretKey) == 0) {
                                JsonDocument doc;
                                if (deserializeJson(doc, plain, cipherLen - P2P_MAC_BYTES) == DeserializationError::Ok) {
                                    const String kind = String(static_cast<const char *>(doc["kind"] | ""));
                                    const String author = String(static_cast<const char *>(doc["author"] | p2pPeers[peerIdx].name.c_str()));
                                    if (kind == "chat") {
                                        const String messageId = String(static_cast<const char *>(doc["id"] | ""));
                                        const String text = String(static_cast<const char *>(doc["text"] | ""));
                                        if (!text.isEmpty()) {
                                            p2pRefreshTrustedPeerIdentity(p2pPeers[peerIdx].pubKeyHex, author, p2pUdp.remoteIP(), p2pUdp.remotePort());
                                            p2pTouchPeerSeen(peerIdx, p2pUdp.remoteIP(), p2pUdp.remotePort());
                                            if (messageId.isEmpty() || !chatHasLoggedMessageId(p2pPeers[peerIdx].pubKeyHex, messageId)) {
                                                chatStoreMessage(p2pPeers[peerIdx].pubKeyHex, author, text, false, CHAT_TRANSPORT_WIFI, messageId);
                                                uiStatusLine = "Chat received from " + author;
                                                if (lvglReady) {
                                                    lvglSyncStatusLine();
                                                    if (uiScreen == UI_CHAT) lvglRefreshChatUi();
                                                }
                                            }
                                            if (!messageId.isEmpty()) p2pSendChatAck(p2pPeers[peerIdx].pubKeyHex, messageId);
                                        }
                                    } else if (kind == "ack") {
                                        const String ackId = String(static_cast<const char *>(doc["ack_id"] | ""));
                                        if (!ackId.isEmpty()) chatAckOutgoingMessage(p2pPeers[peerIdx].pubKeyHex, ackId);
                                    } else if (kind == "delete_conversation") {
                                        p2pRefreshTrustedPeerIdentity(p2pPeers[peerIdx].pubKeyHex, author, p2pUdp.remoteIP(), p2pUdp.remotePort());
                                        p2pTouchPeerSeen(peerIdx, p2pUdp.remoteIP(), p2pUdp.remotePort());
                                        chatApplyConversationDeletion(p2pPeers[peerIdx].pubKeyHex, "Conversation deleted by " + author);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } else {
            while (p2pUdp.available()) p2pUdp.read();
        }
        packetLen = p2pUdp.parsePacket();
    }
}

void lvglRefreshInfoPanel()
{
    if (!lvglInfoList) return;
    sampleTopIndicators();
    lvglRefreshTopIndicators();
    const bool connected = wifiConnectedSafe();
    const String ssid = connected ? wifiSsidSafe() : String("Disconnected");
    const String ip = connected ? wifiIpSafe() : String("-");
    const int rssi = connected ? static_cast<int>(wifiRssiSafe()) : -127;
    const uint8_t wifiQuality = connected ? wifiQualityPercentFromRssi(rssi) : 0;
    const float tempC = temperatureRead();
    uint8_t tempBarValue = 0;
    if (!isnan(tempC) && tempC > 0.0f) {
        float clamped = tempC;
        if (clamped < 0.0f) clamped = 0.0f;
        if (clamped > INFO_TEMP_BAR_MAX_C) clamped = INFO_TEMP_BAR_MAX_C;
        tempBarValue = static_cast<uint8_t>(clamped + 0.5f);
    }
    const uint32_t freeHeap = static_cast<uint32_t>(ESP.getFreeHeap());
    const uint32_t largest8 = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    char batteryBuf[40];
    const int batteryCentivolts = static_cast<int>(batteryVoltage * 100.0f + 0.5f);
    snprintf(batteryBuf, sizeof(batteryBuf), "%d.%02dV  |  %s",
             batteryCentivolts / 100,
             abs(batteryCentivolts % 100),
             batteryCharging ? "Charging" : "On battery");

    if (lvglInfoBatteryValueLabel) lv_label_set_text_fmt(lvglInfoBatteryValueLabel, "%u%%", batteryPercent);
    if (lvglInfoBatterySubLabel) {
        lv_label_set_text(lvglInfoBatterySubLabel, batteryBuf);
    }

    if (lvglInfoWifiValueLabel) lv_label_set_text_fmt(lvglInfoWifiValueLabel, connected ? "%u%%" : "Offline", wifiQuality);
    if (lvglInfoWifiSubLabel) {
        if (connected) lv_label_set_text_fmt(lvglInfoWifiSubLabel, "%s  |  %s  |  %ddBm", ssid.c_str(), ip.c_str(), rssi);
        else lv_label_set_text_fmt(lvglInfoWifiSubLabel, "AP %s  |  Touch to Config > WiFi Config to connect", savedApSsid.c_str());
    }

    if (lvglInfoLightValueLabel) lv_label_set_text_fmt(lvglInfoLightValueLabel, "%u%%", lightPercent);
    if (lvglInfoLightSubLabel) {
        lv_label_set_text_fmt(lvglInfoLightSubLabel, "Display %s  |  Backlight %u%%  |  raw %u",
                              displayAwake ? "awake" : "sleeping",
                              static_cast<unsigned int>(displayBrightnessPercent),
                              static_cast<unsigned int>(lightRawAdc));
    }

    if (lvglInfoCpuValueLabel) lv_label_set_text_fmt(lvglInfoCpuValueLabel, "%u%%", static_cast<unsigned int>(cpuLoadPercent));
    if (lvglInfoCpuSubLabel) {
        lv_label_set_text_fmt(lvglInfoCpuSubLabel, "Approx. main-loop load  |  Free heap %lu KB", static_cast<unsigned long>(freeHeap / 1024U));
    }
    if (lvglInfoCpuBar) {
        lv_bar_set_value(lvglInfoCpuBar, static_cast<int32_t>(cpuLoadPercent), LV_ANIM_OFF);
        lv_color_t cpuColor = lv_color_hex(0x52B788);
        if (cpuLoadPercent >= 85) cpuColor = lv_color_hex(0xD95C5C);
        else if (cpuLoadPercent >= 65) cpuColor = lv_color_hex(0xF2C35E);
        lvglSetInfoBarColor(lvglInfoCpuBar, cpuColor);
    }

    if (lvglInfoTempValueLabel) {
        if (!isnan(tempC) && tempC > 0.0f) {
            char tempBuf[16];
            const int tempTenths = static_cast<int>(tempC * 10.0f + 0.5f);
            snprintf(tempBuf, sizeof(tempBuf), "%d.%dC", tempTenths / 10, abs(tempTenths % 10));
            lv_label_set_text(lvglInfoTempValueLabel, tempBuf);
        } else {
            lv_label_set_text(lvglInfoTempValueLabel, "--");
        }
    }
    if (lvglInfoTempSubLabel) lv_label_set_text_fmt(lvglInfoTempSubLabel, "ESP32 die temp  |  target range under %uC", INFO_TEMP_WARN_C);
    if (lvglInfoTempBar) {
        lv_bar_set_value(lvglInfoTempBar, static_cast<int32_t>(tempBarValue), LV_ANIM_OFF);
        lv_color_t tempColor = lv_color_hex(0x4FC3F7);
        if (tempBarValue >= INFO_TEMP_HOT_C) tempColor = lv_color_hex(0xD95C5C);
        else if (tempBarValue >= INFO_TEMP_WARN_C) tempColor = lv_color_hex(0xF2C35E);
        lvglSetInfoBarColor(lvglInfoTempBar, tempColor);
    }

    if (lvglInfoSystemLabel) {
        lv_label_set_text_fmt(
            lvglInfoSystemLabel,
            "Model: %s\nFirmware: %s\nSoftAP: %s\nLargest 8-bit block: %lu KB\nSD: %s\nMedia: %s",
            deviceShortNameValue().c_str(),
            FW_VERSION,
            AP_SSID,
            static_cast<unsigned long>(largest8 / 1024U),
            sdMounted ? "mounted" : "offline",
            mediaIsPlaying ? mediaNowPlaying.c_str() : "stopped");
    }
}

void lvglRefreshWifiList()
{
    if (!lvglWifiList) return;
    lv_obj_clean(lvglWifiList);
    lvglWifiScanLabel = nullptr;
    lvglWifiApSsidTa = nullptr;
    lvglWifiApPassTa = nullptr;
    lvglWifiApPassShowBtnLabel = nullptr;
    lvglWifiApPassShowBtn = nullptr;

    auto makeCard = [&](const char *title) -> lv_obj_t * {
        lv_obj_t *card = lv_obj_create(lvglWifiList);
        if (!card) return nullptr;
        lv_obj_set_width(card, lv_pct(100));
        lv_obj_set_height(card, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_color(card, lv_color_hex(0x16212C), 0);
        lv_obj_set_style_border_width(card, 0, 0);
        lv_obj_set_style_radius(card, 12, 0);
        lv_obj_set_style_pad_all(card, 8, 0);
        lv_obj_set_style_pad_row(card, 6, 0);
        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_t *lbl = lv_label_create(card);
        if (lbl) {
            lv_label_set_text(lbl, title);
            lv_obj_set_style_text_color(lbl, lv_color_hex(0xE5ECF3), 0);
        }
        return card;
    };

    auto makeBtn = [](lv_obj_t *parent, const char *txt, int32_t w, int32_t h, lv_color_t col, lv_event_cb_t cb, void *ud = nullptr) -> lv_obj_t * {
        lv_obj_t *btn = lv_btn_create(parent);
        if (!btn) return nullptr;
        lv_obj_set_size(btn, w, h);
        lv_obj_set_style_radius(btn, 8, 0);
        lv_obj_set_style_border_width(btn, 0, 0);
        lv_obj_set_style_bg_color(btn, col, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(btn, col, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, ud);
        lv_obj_t *lbl = lv_label_create(btn);
        if (lbl) {
            lv_label_set_text(lbl, txt);
            lv_obj_center(lbl);
        }
        return btn;
    };

    lv_obj_t *staCard = makeCard("WiFi Config");
    if (staCard) {
        lv_obj_t *info = lv_label_create(staCard);
        lv_obj_set_width(info, lv_pct(100));
        lv_label_set_long_mode(info, LV_LABEL_LONG_WRAP);
        String staLine = savedStaSsid.isEmpty() ? String("Saved network: none") : ("Saved network: " + savedStaSsid);
        if (wifiConnectedSafe()) staLine += "\nConnected: " + wifiSsidSafe() + "  |  " + wifiIpSafe();
        else staLine += "\nStatus: disconnected";
        lv_label_set_text(info, staLine.c_str());
        lv_obj_set_style_text_color(info, lv_color_hex(0xB7C4D1), 0);

        lv_obj_t *row = lv_obj_create(staCard);
        lv_obj_set_size(row, lv_pct(100), 34);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_set_style_pad_column(row, 6, 0);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        makeBtn(row, "Disconnect", 82, 28, lv_color_hex(0x2F6D86), lvglWifiDisconnectEvent);
        makeBtn(row, "Forget", 64, 28, lv_color_hex(0x8A3A3A), lvglWifiForgetEvent);
    }

    makeBtn(lvglWifiList, "Scan", 92, 30, lv_color_hex(0x2F6D86), lvglWifiRescanEvent);

    if (wifiScanInProgress) {
        lvglWifiScanLabel = lv_label_create(lvglWifiList);
        if (lvglWifiScanLabel) {
            lv_label_set_text(lvglWifiScanLabel, "Searching for access points");
            lv_obj_set_width(lvglWifiScanLabel, lv_pct(100));
            lv_obj_set_style_text_align(lvglWifiScanLabel, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_set_style_text_color(lvglWifiScanLabel, lv_color_hex(0xC8CED6), 0);
        }
    } else if (wifiCount > 0) {
        lv_obj_t *sec = lv_label_create(lvglWifiList);
        lv_label_set_text(sec, "Discovered Networks");
        lv_obj_set_style_text_color(sec, lv_color_hex(0xE5ECF3), 0);
        for (int i = 0; i < wifiCount; i++) {
            char line[96];
            snprintf(line, sizeof(line), "%s (%ddBm) [%s]", wifiEntries[i].ssid.c_str(), static_cast<int>(wifiEntries[i].rssi), authName(wifiEntries[i].auth));
            lvglCreateMenuButton(
                lvglWifiList,
                line,
                (wifiEntries[i].auth == WIFI_AUTH_OPEN) ? lv_color_hex(0x357A38) : lv_color_hex(0x375A7A),
                lvglWifiEntryEvent,
                reinterpret_cast<void *>(static_cast<intptr_t>(i))
            );
        }
    } else {
        lv_obj_t *lbl = lv_label_create(lvglWifiList);
        lv_label_set_text(lbl, "No scan started yet. Tap Scan.");
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xC8CED6), 0);
    }

    lv_obj_t *apCard = makeCard("AP Config");
    if (apCard) {
        lv_obj_t *hint = lv_label_create(apCard);
        lv_obj_set_width(hint, lv_pct(100));
        lv_label_set_long_mode(hint, LV_LABEL_LONG_WRAP);
        lv_label_set_text_fmt(hint, "Current AP: %s", savedApSsid.c_str());
        lv_obj_set_style_text_color(hint, lv_color_hex(0xB7C4D1), 0);

        auto addTa = [&](lv_obj_t *parent, const char *ph, const char *value, bool pass) -> lv_obj_t * {
            lv_obj_t *ta = lv_textarea_create(parent ? parent : apCard);
            lv_obj_set_width(ta, lv_pct(100));
            lv_textarea_set_one_line(ta, true);
            lv_textarea_set_placeholder_text(ta, ph);
            lv_textarea_set_password_mode(ta, pass);
            lv_textarea_set_text(ta, value ? value : "");
            lv_obj_add_event_cb(ta, lvglTextAreaFocusEvent, LV_EVENT_FOCUSED, nullptr);
            return ta;
        };

        lvglWifiApSsidTa = addTa(apCard, "AP name", savedApSsid.c_str(), false);

        lv_obj_t *pwdRow = lv_obj_create(apCard);
        lv_obj_set_size(pwdRow, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(pwdRow, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(pwdRow, 0, 0);
        lv_obj_set_style_pad_all(pwdRow, 0, 0);
        lv_obj_set_style_pad_column(pwdRow, 6, 0);
        lv_obj_set_flex_flow(pwdRow, LV_FLEX_FLOW_ROW);
        lv_obj_clear_flag(pwdRow, LV_OBJ_FLAG_SCROLLABLE);

        lvglWifiApPassTa = addTa(pwdRow, "AP password", savedApPass.c_str(), true);
        lv_obj_set_width(lvglWifiApPassTa, lv_pct(100));
        lv_obj_set_flex_grow(lvglWifiApPassTa, 1);
        lv_obj_set_height(lvglWifiApPassTa, 38);

        lvglWifiApPassShowBtn = lv_btn_create(pwdRow);
        lv_obj_set_size(lvglWifiApPassShowBtn, 34, 34);
        lv_obj_set_style_radius(lvglWifiApPassShowBtn, 8, 0);
        lv_obj_set_style_border_width(lvglWifiApPassShowBtn, 0, 0);
        lv_obj_set_style_bg_color(lvglWifiApPassShowBtn, lv_color_hex(0x3F4A57), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(lvglWifiApPassShowBtn, lv_color_hex(0x526274), LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_shadow_width(lvglWifiApPassShowBtn, 0, 0);
        lv_obj_add_event_cb(lvglWifiApPassShowBtn, lvglWifiApShowToggleEvent, LV_EVENT_CLICKED, nullptr);
        lvglWifiApPassShowBtnLabel = lv_label_create(lvglWifiApPassShowBtn);
        lv_label_set_text(lvglWifiApPassShowBtnLabel, LV_SYMBOL_EYE_CLOSE);
        lv_obj_center(lvglWifiApPassShowBtnLabel);

        makeBtn(apCard, "Save AP Config", 120, 30, lv_color_hex(0x3A7A3A), lvglWifiApSaveEvent);
    }
}

void lvglQueueMediaRefresh()
{
    lvglMediaRefreshPending = true;
}

void lvglRefreshMediaLayout()
{
    if (!lvglMediaList || !lvglMediaPlayerPanel) return;
    if (lvglMediaPlayerVisible) {
        lv_obj_clear_flag(lvglMediaPlayerPanel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_size(lvglMediaPlayerPanel, lv_pct(100), MEDIA_PLAYER_PANEL_H);
        lv_obj_align(lvglMediaPlayerPanel, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
        lv_obj_set_size(lvglMediaList, lv_pct(100), UI_CONTENT_H - MEDIA_PLAYER_PANEL_H);
        lv_obj_align(lvglMediaList, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y + MEDIA_PLAYER_PANEL_H);
    } else {
        lv_obj_add_flag(lvglMediaPlayerPanel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_size(lvglMediaList, lv_pct(100), UI_CONTENT_H);
        lv_obj_align(lvglMediaList, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
    }
}

void lvglSetMediaPlayerVisible(bool visible)
{
    if (lvglMediaPlayerVisible == visible) return;
    lvglMediaPlayerVisible = visible;
    lvglRefreshMediaLayout();
}

void lvglRefreshMediaList()
{
    if (!lvglMediaList) return;
    lv_obj_clean(lvglMediaList);
    loadMediaEntries();

    if (mediaOffset > 0) {
        lvglCreateMenuButton(lvglMediaList, "<< Prev page", lv_color_hex(0x3F4A57), lvglMediaEntryEvent, reinterpret_cast<void *>(static_cast<intptr_t>(MEDIA_ENTRY_PREV_PAGE)));
    }
    if (mediaCurrentDir != "/") {
        lvglCreateMenuButton(lvglMediaList, ".. (parent)", lv_color_hex(0x4E5D6C), lvglMediaEntryEvent, reinterpret_cast<void *>(static_cast<intptr_t>(MEDIA_ENTRY_PARENT)));
    }
    for (int i = 0; i < mediaCount; i++) {
        String label = mediaBuildListLabel(mediaEntries[i].name, mediaEntries[i].isDir, mediaEntries[i].size);
        lvglCreateMenuButton(
            lvglMediaList,
            label.c_str(),
            mediaEntries[i].isDir ? lv_color_hex(0x2E7D9A) : lv_color_hex(0x355C3D),
            lvglMediaEntryEvent,
            reinterpret_cast<void *>(static_cast<intptr_t>(i))
        );
    }
    if (mediaHasMore) {
        lvglCreateMenuButton(lvglMediaList, "Next page >>", lv_color_hex(0x3F4A57), lvglMediaEntryEvent, reinterpret_cast<void *>(static_cast<intptr_t>(MEDIA_ENTRY_NEXT_PAGE)));
    }
    if (mediaCount == 0) {
        lv_obj_t *lbl = lv_label_create(lvglMediaList);
        lv_label_set_text(lbl, "No media entries");
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xC8CED6), 0);
    }
}

void lvglStatusPush(const String &line)
{
    uiStatusLine = line;
    lvglSyncStatusLine();
    if (lvglMqttStatusLabel) lv_label_set_text_fmt(lvglMqttStatusLabel, "MQTT: %s", mqttStatusLine.c_str());
}

void lvglHideKeyboard()
{
    if (lvglKb) {
        lv_keyboard_set_textarea(lvglKb, nullptr);
        lv_obj_add_flag(lvglKb, LV_OBJ_FLAG_HIDDEN);
    }
    lvglKeyboardShiftOneShot = false;
    lvglKeyboardShiftLocked = false;
    lvglKeyboardShiftLastPressMs = 0;
    lvglSetChatKeyboardVisible(false);
    lvglSetConfigKeyboardVisible(false);
}

void lvglTextAreaFocusEvent(lv_event_t *e)
{
    lv_obj_t *ta = lv_event_get_target(e);
    if (!ta) return;
    if (!lvglKb) {
        lvglKb = lv_keyboard_create(lv_layer_top());
        lv_obj_set_size(lvglKb, lv_pct(100), 120);
        lv_obj_align(lvglKb, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_add_flag(lvglKb, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_event_cb(
            lvglKb,
            [](lv_event_t *ke) {
                lv_event_code_t code = lv_event_get_code(ke);
                if (code == LV_EVENT_VALUE_CHANGED && lvglKb) {
                    const uint16_t btn = lv_btnmatrix_get_selected_btn(lvglKb);
                    const char *txt = lv_btnmatrix_get_btn_text(lvglKb, btn);
                    const bool isShiftKey = txt &&
                                            ((strcmp(txt, "ABC") == 0) ||
                                             (strcmp(txt, "Abc") == 0) ||
                                             (strcmp(txt, "abc") == 0) ||
                                             (strcmp(txt, LV_SYMBOL_UP) == 0));
                    if (isShiftKey) {
                        const unsigned long now = millis();
                        const bool doubleTap = (now - lvglKeyboardShiftLastPressMs) <= 450UL;
                        if (lvglKeyboardShiftLocked) {
                            lvglKeyboardShiftLocked = false;
                            lvglKeyboardShiftOneShot = false;
                            lvglKeyboardShiftLastPressMs = 0;
                            lv_keyboard_set_mode(lvglKb, LV_KEYBOARD_MODE_TEXT_LOWER);
                            return;
                        }
                        if (doubleTap) {
                            lvglKeyboardShiftLocked = true;
                            lvglKeyboardShiftOneShot = false;
                            lvglKeyboardShiftLastPressMs = 0;
                            lv_keyboard_set_mode(lvglKb, LV_KEYBOARD_MODE_TEXT_UPPER);
                            return;
                        }
                        lvglKeyboardShiftLocked = false;
                        lvglKeyboardShiftOneShot = true;
                        lvglKeyboardShiftLastPressMs = now;
                        lv_keyboard_set_mode(lvglKb, LV_KEYBOARD_MODE_TEXT_UPPER);
                        return;
                    }
                    if (lvglKeyboardShiftOneShot && txt && strlen(txt) == 1) {
                        const char c = txt[0];
                        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')) {
                            lvglKeyboardShiftOneShot = false;
                            lvglKeyboardShiftLocked = false;
                            lvglKeyboardShiftLastPressMs = 0;
                            lv_keyboard_set_mode(lvglKb, LV_KEYBOARD_MODE_TEXT_LOWER);
                        }
                    }
                    return;
                }
                if (code != LV_EVENT_READY && code != LV_EVENT_CANCEL) return;
                if (code == LV_EVENT_READY && lvglWifiPwdModal && !lv_obj_has_flag(lvglWifiPwdModal, LV_OBJ_FLAG_HIDDEN) &&
                    lvglWifiPwdTa && lv_keyboard_get_textarea(lvglKb) == lvglWifiPwdTa) {
                    lvglWifiPwdConnectEvent(nullptr);
                } else if (code == LV_EVENT_READY && lvglConfigDeviceNameTa && lv_keyboard_get_textarea(lvglKb) == lvglConfigDeviceNameTa) {
                    lvglSaveDeviceNameEvent(nullptr);
                } else if (code == LV_EVENT_READY && lvglChatInputTa && lv_keyboard_get_textarea(lvglKb) == lvglChatInputTa) {
                    if (lvglChatPromptIfAirplaneBlocked()) return;
                    const char *raw = lv_textarea_get_text(lvglChatInputTa);
                    String text = raw ? String(raw) : String("");
                    text.trim();
                    if (!text.isEmpty()) {
                        if (currentChatPeerKey.isEmpty()) {
                            uiStatusLine = "Select a contact first";
                            lvglSyncStatusLine();
                            return;
                        }
                        chatSendAndStoreMessage(currentChatPeerKey, text);
                        lv_textarea_set_text(lvglChatInputTa, "");
                        lvglSetChatKeyboardVisible(false);
                    }
                }
                lvglHideKeyboard();
            },
            LV_EVENT_ALL,
            nullptr
        );
    }
    lvglKeyboardShiftOneShot = false;
    lvglKeyboardShiftLocked = false;
    lvglKeyboardShiftLastPressMs = 0;
    lv_keyboard_set_mode(lvglKb, LV_KEYBOARD_MODE_TEXT_LOWER);
    lv_keyboard_set_textarea(lvglKb, ta);
    lv_obj_clear_flag(lvglKb, LV_OBJ_FLAG_HIDDEN);
    if (ta == lvglChatInputTa) lvglSetChatKeyboardVisible(true);
    if (ta == lvglConfigDeviceNameTa) lvglSetConfigKeyboardVisible(true);
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
    if (lvglWifiApPassShowBtnLabel) lv_label_set_text(lvglWifiApPassShowBtnLabel, makeVisible ? LV_SYMBOL_EYE_OPEN : LV_SYMBOL_EYE_CLOSE);
}

void lvglWifiPwdShowToggleEvent(lv_event_t *e)
{
    lv_obj_t *btn = lv_event_get_target(e);
    if (!btn || !lvglWifiPwdTa) return;
    const bool makeVisible = lv_textarea_get_password_mode(lvglWifiPwdTa);
    lv_textarea_set_password_mode(lvglWifiPwdTa, makeVisible ? false : true);
    if (lvglWifiPwdShowBtnLabel) lv_label_set_text(lvglWifiPwdShowBtnLabel, makeVisible ? LV_SYMBOL_EYE_OPEN : LV_SYMBOL_EYE_CLOSE);
}

void lvglMqttPassShowToggleEvent(lv_event_t *e)
{
    lv_obj_t *btn = lv_event_get_target(e);
    if (!btn || !lvglMqttPassTa) return;
    const bool makeVisible = lv_textarea_get_password_mode(lvglMqttPassTa);
    lv_textarea_set_password_mode(lvglMqttPassTa, makeVisible ? false : true);
    if (lvglMqttPassShowBtnLabel) lv_label_set_text(lvglMqttPassShowBtnLabel, makeVisible ? LV_SYMBOL_EYE_OPEN : LV_SYMBOL_EYE_CLOSE);
}

void lvglOpenWifiPasswordDialog(const String &ssid)
{
    if (!lvglReady) return;
    if (!lvglWifiPwdModal) {
        lvglWifiPwdModal = lv_obj_create(lv_layer_top());
        if (!lvglWifiPwdModal) return;
        lv_obj_set_size(lvglWifiPwdModal, DISPLAY_WIDTH - 24, 170);
        lv_obj_center(lvglWifiPwdModal);
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

        lv_obj_t *pwdRow = lv_obj_create(lvglWifiPwdModal);
        lv_obj_set_size(pwdRow, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(pwdRow, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(pwdRow, 0, 0);
        lv_obj_set_style_pad_all(pwdRow, 0, 0);
        lv_obj_set_style_pad_column(pwdRow, 6, 0);
        lv_obj_set_flex_flow(pwdRow, LV_FLEX_FLOW_ROW);
        lv_obj_clear_flag(pwdRow, LV_OBJ_FLAG_SCROLLABLE);

        lvglWifiPwdTa = lv_textarea_create(pwdRow);
        lv_obj_set_width(lvglWifiPwdTa, lv_pct(100));
        lv_obj_set_flex_grow(lvglWifiPwdTa, 1);
        lv_obj_set_height(lvglWifiPwdTa, 38);
        lv_textarea_set_one_line(lvglWifiPwdTa, true);
        lv_textarea_set_password_mode(lvglWifiPwdTa, true);
        lv_textarea_set_placeholder_text(lvglWifiPwdTa, "Password");
        lv_obj_add_event_cb(lvglWifiPwdTa, lvglTextAreaFocusEvent, LV_EVENT_FOCUSED, nullptr);

        lvglWifiPwdShowBtn = lv_btn_create(pwdRow);
        lv_obj_set_size(lvglWifiPwdShowBtn, 34, 34);
        lv_obj_set_style_radius(lvglWifiPwdShowBtn, 8, 0);
        lv_obj_set_style_border_width(lvglWifiPwdShowBtn, 0, 0);
        lv_obj_set_style_bg_color(lvglWifiPwdShowBtn, lv_color_hex(0x3F4A57), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(lvglWifiPwdShowBtn, lv_color_hex(0x526274), LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_shadow_width(lvglWifiPwdShowBtn, 0, 0);
        lv_obj_add_event_cb(lvglWifiPwdShowBtn, lvglWifiPwdShowToggleEvent, LV_EVENT_CLICKED, nullptr);
        lvglWifiPwdShowBtnLabel = lv_label_create(lvglWifiPwdShowBtn);
        lv_label_set_text(lvglWifiPwdShowBtnLabel, LV_SYMBOL_EYE_OPEN);
        lv_obj_center(lvglWifiPwdShowBtnLabel);

        lvglWifiPwdStatusLabel = lv_label_create(lvglWifiPwdModal);
        lv_obj_set_width(lvglWifiPwdStatusLabel, lv_pct(100));
        lv_label_set_long_mode(lvglWifiPwdStatusLabel, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_color(lvglWifiPwdStatusLabel, lv_color_hex(0xD8B36A), 0);

        lv_obj_t *row = lv_obj_create(lvglWifiPwdModal);
        lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_set_style_pad_column(row, 8, 0);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        auto makeActionBtn = [&](const char *txt, lv_color_t color, lv_event_cb_t cb) {
            lv_obj_t *btn = lv_btn_create(row);
            if (!btn) return;
            lv_obj_set_size(btn, 84, 30);
            lv_obj_set_style_radius(btn, 8, 0);
            lv_obj_set_style_border_width(btn, 0, 0);
            lv_obj_set_style_bg_color(btn, color, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(btn, color, LV_PART_MAIN | LV_STATE_PRESSED);
            lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, nullptr);
            lv_obj_t *lbl = lv_label_create(btn);
            if (lbl) {
                lv_label_set_text(lbl, txt);
                lv_obj_center(lbl);
            }
        };
        makeActionBtn("Cancel", lv_color_hex(0x4E5D6C), lvglWifiPwdCancelEvent);
        makeActionBtn("Save", lv_color_hex(0x357A38), lvglWifiPwdConnectEvent);
    }

    lvglWifiPendingSsid = ssid;
    lvglWifiPwdConnectPending = false;
    if (lvglWifiPwdSsidLabel) lv_label_set_text_fmt(lvglWifiPwdSsidLabel, "SSID: %s", ssid.c_str());
    if (lvglWifiPwdTa) {
        lv_textarea_set_text(lvglWifiPwdTa, "");
        lv_textarea_set_password_mode(lvglWifiPwdTa, true);
        lv_event_send(lvglWifiPwdTa, LV_EVENT_FOCUSED, nullptr);
    }
    if (lvglWifiPwdShowBtnLabel) lv_label_set_text(lvglWifiPwdShowBtnLabel, LV_SYMBOL_EYE_OPEN);
    if (lvglWifiPwdStatusLabel) lv_label_set_text(lvglWifiPwdStatusLabel, "");
    lv_obj_move_foreground(lvglWifiPwdModal);
    lv_obj_clear_flag(lvglWifiPwdModal, LV_OBJ_FLAG_HIDDEN);
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
    (void)e;
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
    (void)e;
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

void lvglMqttButtonCountDelta(int d)
{
    int next = mqttButtonCount + d;
    if (next < 1) next = 1;
    if (next > MQTT_MAX_BUTTONS) next = MQTT_MAX_BUTTONS;
    if (next == mqttButtonCount) return;
    if (next > mqttButtonCount) {
        for (int i = mqttButtonCount; i < next; i++) {
            mqttButtonNames[i] = mqttDefaultButtonName(i);
            mqttButtonCritical[i] = false;
        }
    }
    mqttButtonCount = next;
    if (lvglMqttEditIndex >= mqttButtonCount) lvglMqttEditIndex = mqttButtonCount - 1;
    mqttDiscoveryPublished = false;
    saveMqttConfig();
    lvglRefreshMqttConfigUi();
    lvglRefreshMqttControlsUi();
}

void lvglMqttCountMinusEvent(lv_event_t *e)
{
    (void)e;
    lvglMqttButtonCountDelta(-1);
}

void lvglMqttCountPlusEvent(lv_event_t *e)
{
    (void)e;
    lvglMqttButtonCountDelta(+1);
}

void lvglMqttEditPrevEvent(lv_event_t *e)
{
    (void)e;
    if (lvglMqttEditIndex > 0) lvglMqttEditIndex--;
    lvglRefreshMqttControlsUi();
}

void lvglMqttEditNextEvent(lv_event_t *e)
{
    (void)e;
    if (lvglMqttEditIndex + 1 < mqttButtonCount) lvglMqttEditIndex++;
    lvglRefreshMqttControlsUi();
}

void lvglMqttApplyBtnEvent(lv_event_t *e)
{
    (void)e;
    if (lvglMqttEditIndex < 0 || lvglMqttEditIndex >= mqttButtonCount) return;
    mqttButtonNames[lvglMqttEditIndex] = lv_textarea_get_text(lvglMqttBtnNameTa);
    if (mqttButtonNames[lvglMqttEditIndex].isEmpty()) mqttButtonNames[lvglMqttEditIndex] = mqttDefaultButtonName(lvglMqttEditIndex);
    mqttButtonCritical[lvglMqttEditIndex] = lv_obj_has_state(lvglMqttCriticalSw, LV_STATE_CHECKED);
    mqttDiscoveryPublished = false;
    saveMqttConfig();
    lvglRefreshMqttControlsUi();
}

void lvglMqttControlConfirmEvent(lv_event_t *e)
{
    lv_obj_t *msgbox = lv_event_get_current_target(e);
    const char *txt = lv_msgbox_get_active_btn_text(msgbox);
    int idx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (txt && strcmp(txt, "Yes") == 0 && idx >= 0 && idx < mqttButtonCount) {
        mqttPublishButtonAction(idx);
        lvglStatusPush("Action: " + mqttButtonPayloadForIndex(idx));
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
        lv_obj_center(m);
        lv_obj_add_event_cb(m, lvglMqttControlConfirmEvent, LV_EVENT_VALUE_CHANGED, reinterpret_cast<void *>(static_cast<intptr_t>(idx)));
    } else {
        mqttPublishButtonAction(idx);
        lvglStatusPush("Action: " + mqttButtonPayloadForIndex(idx));
    }
}

void lvglRefreshMqttConfigUi()
{
    if (!lvglMqttEnableSw) return;
    lv_obj_add_state(lvglMqttEnableSw, mqttCfg.enabled ? LV_STATE_CHECKED : 0);
    lv_obj_clear_state(lvglMqttEnableSw, mqttCfg.enabled ? 0 : LV_STATE_CHECKED);
    lv_textarea_set_text(lvglMqttBrokerTa, mqttCfg.broker.c_str());
    lv_textarea_set_text(lvglMqttPortTa, String(mqttCfg.port).c_str());
    lv_textarea_set_text(lvglMqttUserTa, mqttCfg.username.c_str());
    lv_textarea_set_text(lvglMqttPassTa, mqttCfg.password.c_str());
    lv_textarea_set_text(lvglMqttDiscTa, mqttCfg.discoveryPrefix.c_str());
    lv_label_set_text_fmt(lvglMqttStatusLabel, "MQTT: %s", mqttStatusLine.c_str());
}

void lvglRefreshMqttControlsUi()
{
    if (!lvglMqttCtrlList) return;
    lv_obj_clean(lvglMqttCtrlList);
    if (lvglMqttEditIndex < 0) lvglMqttEditIndex = 0;
    if (lvglMqttEditIndex >= mqttButtonCount) lvglMqttEditIndex = mqttButtonCount - 1;

    auto makeSmallBtnLocal = [](lv_obj_t *parent, const char *txt, int w, int h, lv_color_t col, lv_event_cb_t cb, void *ud = nullptr) -> lv_obj_t * {
        lv_obj_t *b = lv_btn_create(parent);
        if (!b) return nullptr;
        lv_obj_set_size(b, w, h);
        lv_obj_set_style_radius(b, 8, 0);
        lv_obj_set_style_border_width(b, 0, 0);
        lv_obj_set_style_bg_color(b, col, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(b, col, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
        lv_obj_t *l = lv_label_create(b);
        if (l) {
            lv_label_set_text(l, txt);
            lv_obj_center(l);
        }
        return b;
    };

    lv_obj_t *editCard = lv_obj_create(lvglMqttCtrlList);
    lv_obj_set_width(editCard, lv_pct(100));
    lv_obj_set_height(editCard, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_color(editCard, lv_color_hex(0x16212C), 0);
    lv_obj_set_style_border_width(editCard, 0, 0);
    lv_obj_set_style_radius(editCard, 12, 0);
    lv_obj_set_style_pad_all(editCard, 8, 0);
    lv_obj_set_style_pad_row(editCard, 6, 0);
    lv_obj_set_flex_flow(editCard, LV_FLEX_FLOW_COLUMN);
    lv_obj_clear_flag(editCard, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *countRow = lv_obj_create(editCard);
    lv_obj_set_size(countRow, lv_pct(100), 34);
    lv_obj_set_style_bg_opa(countRow, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(countRow, 0, 0);
    makeSmallBtnLocal(countRow, "-", 34, 26, lv_color_hex(0x6B4A2A), lvglMqttCountMinusEvent);
    lv_obj_align(lv_obj_get_child(countRow, 0), LV_ALIGN_LEFT_MID, 0, 0);
    makeSmallBtnLocal(countRow, "+", 34, 26, lv_color_hex(0x3A7A3A), lvglMqttCountPlusEvent);
    lv_obj_align(lv_obj_get_child(countRow, 1), LV_ALIGN_RIGHT_MID, 0, 0);
    lvglMqttCountLabel = lv_label_create(countRow);
    lv_obj_center(lvglMqttCountLabel);
    lv_label_set_text_fmt(lvglMqttCountLabel, "Buttons: %d", mqttButtonCount);

    lv_obj_t *editRow = lv_obj_create(editCard);
    lv_obj_set_size(editRow, lv_pct(100), 34);
    lv_obj_set_style_bg_opa(editRow, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(editRow, 0, 0);
    makeSmallBtnLocal(editRow, "<", 34, 26, lv_color_hex(0x2F6D86), lvglMqttEditPrevEvent);
    lv_obj_align(lv_obj_get_child(editRow, 0), LV_ALIGN_LEFT_MID, 0, 0);
    makeSmallBtnLocal(editRow, ">", 34, 26, lv_color_hex(0x2F6D86), lvglMqttEditNextEvent);
    lv_obj_align(lv_obj_get_child(editRow, 1), LV_ALIGN_RIGHT_MID, 0, 0);
    lvglMqttEditLabel = lv_label_create(editRow);
    lv_obj_center(lvglMqttEditLabel);
    lv_label_set_text_fmt(lvglMqttEditLabel, "Edit #%d", lvglMqttEditIndex + 1);

    lvglMqttBtnNameTa = lv_textarea_create(editCard);
    lv_obj_set_width(lvglMqttBtnNameTa, lv_pct(100));
    lv_textarea_set_one_line(lvglMqttBtnNameTa, true);
    lv_textarea_set_placeholder_text(lvglMqttBtnNameTa, "Button name");
    lv_obj_add_event_cb(lvglMqttBtnNameTa, lvglTextAreaFocusEvent, LV_EVENT_FOCUSED, nullptr);
    lv_textarea_set_text(lvglMqttBtnNameTa, mqttButtonNames[lvglMqttEditIndex].c_str());

    lv_obj_t *critRow = lv_obj_create(editCard);
    lv_obj_set_size(critRow, lv_pct(100), 34);
    lv_obj_set_style_bg_opa(critRow, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(critRow, 0, 0);
    lv_obj_t *critLbl = lv_label_create(critRow);
    lv_label_set_text(critLbl, "Critical");
    lv_obj_align(critLbl, LV_ALIGN_LEFT_MID, 0, 0);
    lvglMqttCriticalSw = lv_switch_create(critRow);
    lv_obj_align(lvglMqttCriticalSw, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_state(lvglMqttCriticalSw, mqttButtonCritical[lvglMqttEditIndex] ? LV_STATE_CHECKED : 0);
    lv_obj_clear_state(lvglMqttCriticalSw, mqttButtonCritical[lvglMqttEditIndex] ? 0 : LV_STATE_CHECKED);

    lv_obj_t *editActRow = lv_obj_create(editCard);
    lv_obj_set_size(editActRow, lv_pct(100), 36);
    lv_obj_set_style_bg_opa(editActRow, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(editActRow, 0, 0);
    lv_obj_set_style_pad_column(editActRow, 6, 0);
    lv_obj_set_flex_flow(editActRow, LV_FLEX_FLOW_ROW_WRAP);
    makeSmallBtnLocal(editActRow, "Apply Btn", 72, 30, lv_color_hex(0x6D4B9A), lvglMqttApplyBtnEvent);

    for (int i = 0; i < mqttButtonCount; i++) {
        String lbl = mqttButtonNames[i];
        if (mqttButtonCritical[i]) lbl += " (!)";
        lvglCreateMenuButton(
            lvglMqttCtrlList,
            lbl.c_str(),
            mqttButtonCritical[i] ? lv_color_hex(0x9A5A2E) : lv_color_hex(0x2D6D8E),
            lvglMqttControlPressEvent,
            reinterpret_cast<void *>(static_cast<intptr_t>(i))
        );
    }
}

void lvglScreenshotEvent(lv_event_t *e)
{
    (void)e;
    if (!sdEnsureMounted(true)) {
        lvglStatusPush("Screenshot failed: no SD");
        return;
    }
    fsWriteBegin();
    String path;
    String err;
    bool ok = captureScreenToJpeg(path, err);
    fsWriteEnd();
    if (!ok) lvglStatusPush("Screenshot error: " + (err.length() ? err : String("unknown")));
    else lvglStatusPush("Saved: " + path);
}

void lvglSnakeDirEvent(lv_event_t *e)
{
    const int d = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (d == 3 && snakeDir != 1) snakeNextDir = 3;
    else if (d == 1 && snakeDir != 3) snakeNextDir = 1;
    else if (d == 0 && snakeDir != 2) snakeNextDir = 0;
    else if (d == 2 && snakeDir != 0) snakeNextDir = 2;
}

void lvglSnakeBackEvent(lv_event_t *e)
{
    (void)e;
    lvglOpenScreen(UI_GAMES, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
}

void lvglSnakeRestartEvent(lv_event_t *e)
{
    (void)e;
    snakeResetGame();
    lvglRefreshSnakeBoard();
}

void lvglTetrisBackEvent(lv_event_t *e)
{
    (void)e;
    lvglOpenScreen(UI_GAMES, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
}

void lvglTetrisRestartEvent(lv_event_t *e)
{
    (void)e;
    tetrisResetGame();
    lvglRefreshTetrisBoard();
}

void lvglTetrisMoveLeftEvent(lv_event_t *e) { (void)e; tetrisMove(-1); }
void lvglTetrisMoveRightEvent(lv_event_t *e) { (void)e; tetrisMove(1); }
void lvglTetrisRotateEvent(lv_event_t *e) { (void)e; tetrisRotate(); }
void lvglTetrisDropEvent(lv_event_t *e) { (void)e; tetrisDrop(); }

void lvglDrawGridCell(lv_draw_ctx_t *drawCtx, int x, int y, int w, int h, lv_color_t color)
{
    lv_draw_rect_dsc_t dsc;
    lv_draw_rect_dsc_init(&dsc);
    dsc.bg_opa = LV_OPA_COVER;
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

void lvglSnakeBoardDrawEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_DRAW_MAIN) return;
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *drawCtx = lv_event_get_draw_ctx(e);
    if (!obj || !drawCtx) return;

    lv_area_t coords;
    lv_obj_get_coords(obj, &coords);
    const int gap = 1;
    const int width = static_cast<int>(coords.x2 - coords.x1 + 1);
    const int height = static_cast<int>(coords.y2 - coords.y1 + 1);
    const int cellW = (width - ((SNAKE_COLS - 1) * gap)) / SNAKE_COLS;
    const int cellH = (height - ((SNAKE_ROWS - 1) * gap)) / SNAKE_ROWS;
    if (cellW < 2 || cellH < 2) return;

    const lv_color_t bg = lv_color_hex(0x1B2430);
    const lv_color_t food = lv_color_hex(0xC62828);
    const lv_color_t head = lv_color_hex(0xC5E84A);
    const lv_color_t body = lv_color_hex(0x4CAF50);

    for (int y = 0; y < SNAKE_ROWS; y++) {
        for (int x = 0; x < SNAKE_COLS; x++) {
            const int px = static_cast<int>(coords.x1) + x * (cellW + gap);
            const int py = static_cast<int>(coords.y1) + y * (cellH + gap);
            lvglDrawGridCell(drawCtx, px, py, cellW, cellH, bg);
        }
    }

    if (snakeFoodX >= 0 && snakeFoodX < SNAKE_COLS && snakeFoodY >= 0 && snakeFoodY < SNAKE_ROWS) {
        const int fx = static_cast<int>(coords.x1) + snakeFoodX * (cellW + gap);
        const int fy = static_cast<int>(coords.y1) + snakeFoodY * (cellH + gap);
        lvglDrawGridCell(drawCtx, fx, fy, cellW, cellH, food);
    }
    for (int i = snakeLen - 1; i >= 0; i--) {
        const int sx = snakeX[i];
        const int sy = snakeY[i];
        if (sx < 0 || sx >= SNAKE_COLS || sy < 0 || sy >= SNAKE_ROWS) continue;
        const int px = static_cast<int>(coords.x1) + sx * (cellW + gap);
        const int py = static_cast<int>(coords.y1) + sy * (cellH + gap);
        lvglDrawGridCell(drawCtx, px, py, cellW, cellH, (i == 0) ? head : body);
    }
}

void lvglTetrisBoardDrawEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_DRAW_MAIN) return;
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *drawCtx = lv_event_get_draw_ctx(e);
    if (!obj || !drawCtx) return;

    static const lv_color_t cols[8] = {
        lv_color_hex(0x111922), lv_color_hex(0x26C6DA), lv_color_hex(0x1565C0), lv_color_hex(0xEF6C00),
        lv_color_hex(0xFDD835), lv_color_hex(0x43A047), lv_color_hex(0x8E24AA), lv_color_hex(0xD32F2F)
    };

    lv_area_t coords;
    lv_obj_get_coords(obj, &coords);
    const int gap = 1;
    const int width = static_cast<int>(coords.x2 - coords.x1 + 1);
    const int height = static_cast<int>(coords.y2 - coords.y1 + 1);
    const int cellW = (width - ((TETRIS_COLS - 1) * gap)) / TETRIS_COLS;
    const int cellH = (height - ((TETRIS_ROWS - 1) * gap)) / TETRIS_ROWS;
    if (cellW < 2 || cellH < 2) return;

    for (int y = 0; y < TETRIS_ROWS; y++) {
        for (int x = 0; x < TETRIS_COLS; x++) {
            const int px = static_cast<int>(coords.x1) + x * (cellW + gap);
            const int py = static_cast<int>(coords.y1) + y * (cellH + gap);
            lvglDrawGridCell(drawCtx, px, py, cellW, cellH, cols[tetrisGrid[y][x] & 0x07]);
        }
    }

    for (int i = 0; i < 4; i++) {
        int ox = 0, oy = 0;
        tetrisCellFor(tetrisType, tetrisRot, i, ox, oy);
        int gx = tetrisX + ox;
        int gy = tetrisY + oy;
        if (gx < 0 || gx >= TETRIS_COLS || gy < 0 || gy >= TETRIS_ROWS) continue;
        const int px = static_cast<int>(coords.x1) + gx * (cellW + gap);
        const int py = static_cast<int>(coords.y1) + gy * (cellH + gap);
        lvglDrawGridCell(drawCtx, px, py, cellW, cellH, cols[(tetrisType + 1) & 0x07]);
    }
}

void lvglRefreshSnakeBoard()
{
    if (lvglSnakeBoardObj) lv_obj_invalidate(lvglSnakeBoardObj);
    if (lvglSnakeScoreLabel) {
        lv_label_set_text_fmt(lvglSnakeScoreLabel, "Score: %u%s", snakeScore, snakeGameOver ? "  GAME OVER" : "");
    }
}

void lvglRefreshTetrisBoard()
{
    if (lvglTetrisBoardObj) lv_obj_invalidate(lvglTetrisBoardObj);
    if (lvglTetrisScoreLabel) {
        lv_label_set_text_fmt(lvglTetrisScoreLabel, "Score: %u%s", tetrisScore, tetrisGameOver ? "  GAME OVER" : "");
    }
}

void lvglOpenSnakeEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_GAME_SNAKE);
    snakeResetGame();
    lvglRefreshSnakeBoard();
    lvglOpenScreen(UI_GAME_SNAKE, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglOpenTetrisEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_GAME_TETRIS);
    tetrisResetGame();
    lvglRefreshTetrisBoard();
    lvglOpenScreen(UI_GAME_TETRIS, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglOpenMqttCfgEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_CONFIG_MQTT_CONFIG);
    lvglRefreshMqttConfigUi();
    lvglOpenScreen(UI_CONFIG_MQTT_CONFIG, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglOpenMqttCtrlEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_CONFIG_MQTT_CONTROLS);
    lvglRefreshMqttControlsUi();
    lvglOpenScreen(UI_CONFIG_MQTT_CONTROLS, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglBackToConfigEvent(lv_event_t *e)
{
    (void)e;
    lvglOpenScreen(UI_CONFIG, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
}

void lvglStyleHintEvent(lv_event_t *e)
{
    (void)e;
    lvglStatusPush("Style tuning now handled by LVGL theme");
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
    uiPrefs.putUChar("disp_bri", displayBrightnessPercent);
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
    if (lvglAirplaneBtnLabel) lv_label_set_text(lvglAirplaneBtnLabel, airplaneModeEnabled ? "Airplane: ON" : "Airplane: OFF");
    if (lvglConfigDeviceNameTa) {
        String current = lv_textarea_get_text(lvglConfigDeviceNameTa);
        if (current != deviceShortNameValue()) lv_textarea_set_text(lvglConfigDeviceNameTa, deviceShortNameValue().c_str());
    }
    if (lvglBrightnessSlider && lv_slider_get_value(lvglBrightnessSlider) != displayBrightnessPercent) {
        lv_slider_set_value(lvglBrightnessSlider, displayBrightnessPercent, LV_ANIM_OFF);
    }
    if (lvglBrightnessValueLabel) lv_label_set_text_fmt(lvglBrightnessValueLabel, "%u%%", static_cast<unsigned int>(displayBrightnessPercent));
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
}

void lvglSetConfigKeyboardVisible(bool visible)
{
    if (!lvglConfigWrap || !lvglConfigDeviceNameTa) return;
    lv_obj_set_style_pad_bottom(lvglConfigWrap, visible ? 132 : 10, 0);
    if (visible) lv_obj_scroll_to_view_recursive(lvglConfigDeviceNameTa, LV_ANIM_ON);
}

void lvglBuildUi()
{
    lvglEnsureScreenBuilt(UI_HOME);
    lvglRefreshTopIndicators();
    lv_scr_load(lvglScrHome);
}

void lvglInitUi()
{
    static_assert(LV_COLOR_DEPTH == 16, "LVGL color depth must be 16 for TFT_eSPI flush path");
    lvglReady = false;
    Serial.printf("[LVGL] cfg mem=%uB color_depth=%u free_heap=%u largest_8bit=%u\n",
                  static_cast<unsigned int>(LV_MEM_SIZE),
                  static_cast<unsigned int>(LV_COLOR_DEPTH),
                  static_cast<unsigned int>(ESP.getFreeHeap()),
                  static_cast<unsigned int>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
    lv_init();
    const uint32_t horRes = static_cast<uint32_t>(tft.width());
    const uint32_t verRes = static_cast<uint32_t>(tft.height());
    if (horRes == 0 || verRes == 0) {
        Serial.println("[LVGL] invalid TFT resolution");
        return;
    }

    if (!lvglDrawPixels) {
        uint16_t lines = LVGL_BUF_LINES;
        while (lines >= 2) {
            const size_t pxCount = static_cast<size_t>(horRes) * static_cast<size_t>(lines);
            lv_color_t *candidate = static_cast<lv_color_t *>(malloc(pxCount * sizeof(lv_color_t)));
            if (candidate) {
                lvglDrawPixels = candidate;
                lvglBufLinesActive = lines;
                break;
            }
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

    lv_disp_draw_buf_init(
        &lvglDrawBuf,
        lvglDrawPixels,
        nullptr,
        static_cast<uint32_t>(horRes * static_cast<uint32_t>(lvglBufLinesActive))
    );
    lv_disp_drv_init(&lvglDispDrv);
    lvglDispDrv.hor_res = static_cast<lv_coord_t>(horRes);
    lvglDispDrv.ver_res = static_cast<lv_coord_t>(verRes);
    lvglDispDrv.flush_cb = lvglFlushCb;
    lvglDispDrv.draw_buf = &lvglDrawBuf;
    lv_disp_drv_register(&lvglDispDrv);

    lv_indev_drv_init(&lvglIndevDrv);
    lvglIndevDrv.type = LV_INDEV_TYPE_POINTER;
    lvglIndevDrv.read_cb = lvglTouchReadCb;
    lvglTouchIndev = lv_indev_drv_register(&lvglIndevDrv);
    lvglEnsurePersistentTopBar();

    lvglLastTickMs = millis();
    lvglLastInfoRefreshMs = 0;
    lvglLastStatusRefreshMs = 0;
    lvglLastMediaPlayerRefreshMs = 0;
    Serial.printf("[LVGL] init ok: %lux%lu, buf_lines=%u\n",
                  static_cast<unsigned long>(horRes),
                  static_cast<unsigned long>(verRes),
                  static_cast<unsigned int>(lvglBufLinesActive));
    Serial.printf("[LVGL] pre-ui free_heap=%u largest_8bit=%u\n",
                  static_cast<unsigned int>(ESP.getFreeHeap()),
                  static_cast<unsigned int>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
    lvglBuildUi();
    Serial.printf("[LVGL] post-ui free_heap=%u largest_8bit=%u\n",
                  static_cast<unsigned int>(ESP.getFreeHeap()),
                  static_cast<unsigned int>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
    lvglReady = true;
}

void lvglService()
{
    if (!lvglReady) return;
    if (lvglSwipeBackPending) {
        lvglSwipeBackPending = false;
        if (lvglKb && !lv_obj_has_flag(lvglKb, LV_OBJ_FLAG_HIDDEN)) lvglHideKeyboard();
        else lvglNavigateBackBySwipe();
    }
    if (lvglMediaRefreshPending) {
        lvglMediaRefreshPending = false;
        lvglRefreshMediaList();
    }
    const unsigned long now = millis();
    if (wifiScanInProgress && lvglWifiScanLabel && (now - wifiScanAnimLastMs) >= 320UL) {
        wifiScanAnimLastMs = now;
        wifiScanAnimPhase = static_cast<uint8_t>((wifiScanAnimPhase + 1U) & 0x03U);
        char msg[48];
        snprintf(msg, sizeof(msg), "Searching for access points%.*s", wifiScanAnimPhase, "...");
        lv_label_set_text(lvglWifiScanLabel, msg);
    }
    unsigned long delta = now - lvglLastTickMs;
    if (delta > 0) {
        lv_tick_inc(delta);
        lvglLastTickMs = now;
    }
    lv_timer_handler();

    if ((now - lvglLastStatusRefreshMs) >= 600UL) {
        lvglLastStatusRefreshMs = now;
        lvglSyncStatusLine();
        lvglRefreshTopIndicators();
    }
    if ((now - lvglLastMediaPlayerRefreshMs) >= 250UL) {
        lvglLastMediaPlayerRefreshMs = now;
        lvglRefreshMediaPlayerUi();
    }
    if ((now - lvglLastInfoRefreshMs) >= 2000UL) {
        lvglLastInfoRefreshMs = now;
        lvglRefreshInfoPanel();
    }
}

float readBatteryVoltage()
{
    uint32_t mvSum = 0;
    for (int i = 0; i < BATTERY_ADC_SAMPLES; i++) {
        mvSum += analogReadMilliVolts(BATTERY_ADC_PIN);
    }
    const uint32_t mv = mvSum / BATTERY_ADC_SAMPLES;
    const float pinV = static_cast<float>(mv) / 1000.0f;
    const float ratio = (BATTERY_DIVIDER_R_TOP + BATTERY_DIVIDER_R_BOTTOM) / BATTERY_DIVIDER_R_BOTTOM;
    return pinV * ratio * BATTERY_CAL_FACTOR;
}

uint8_t batteryPercentFromVoltage(float vbat)
{
    float p = (vbat - BATTERY_EMPTY_V) * 100.0f / (BATTERY_FULL_V - BATTERY_EMPTY_V);
    if (p < 0.0f) p = 0.0f;
    if (p > 100.0f) p = 100.0f;
    return static_cast<uint8_t>(p + 0.5f);
}

void cpuLoadService(uint32_t loopStartUs)
{
    if (cpuLoadPrevLoopStartUs != 0U) {
        const uint32_t loopTotalUs = loopStartUs - cpuLoadPrevLoopStartUs;
        const uint32_t loopActiveUs = (cpuLoadPrevActiveUs > loopTotalUs) ? loopTotalUs : cpuLoadPrevActiveUs;
        cpuLoadAccumActiveUs += loopActiveUs;
        cpuLoadAccumTotalUs += loopTotalUs;
    }
    cpuLoadPrevLoopStartUs = loopStartUs;

    const unsigned long nowMs = millis();
    if (cpuLoadWindowStartMs == 0UL) cpuLoadWindowStartMs = nowMs;
    if ((nowMs - cpuLoadWindowStartMs) < 1000UL) return;

    uint8_t nextLoad = cpuLoadPercent;
    if (cpuLoadAccumTotalUs > 0U) {
        const uint32_t pct = (cpuLoadAccumActiveUs * 100U) / cpuLoadAccumTotalUs;
        nextLoad = static_cast<uint8_t>(pct > 100U ? 100U : pct);
    }
    cpuLoadPercent = nextLoad;
    cpuLoadAccumActiveUs = 0U;
    cpuLoadAccumTotalUs = 0U;
    cpuLoadWindowStartMs = nowMs;
}

uint16_t readLightRaw()
{
    uint32_t sum = 0;
    for (int i = 0; i < LIGHT_ADC_SAMPLES; i++) {
        sum += analogRead(LIGHT_ADC_PIN);
    }
    uint32_t raw = sum / LIGHT_ADC_SAMPLES;
    if (raw > 4095) raw = 4095;
    return static_cast<uint16_t>(raw);
}

uint8_t lightPercentFromRaw(uint16_t raw)
{
    // Track observed raw range to auto-calibrate high-impedance light dividers.
    if (raw < lightMinObserved) lightMinObserved = raw;
    if (raw > lightMaxObserved) lightMaxObserved = raw;

    int span = static_cast<int>(lightMaxObserved) - static_cast<int>(lightMinObserved);
    float p = 0.0f;
    if (span >= LIGHT_MIN_SPAN_RAW) {
        p = (static_cast<float>(raw - lightMinObserved) * 100.0f) / static_cast<float>(span);
    } else {
        // Fallback for very small-range/high-impedance circuits.
        int calSpan = static_cast<int>(LIGHT_RAW_CAL_MAX) - static_cast<int>(LIGHT_RAW_CAL_MIN);
        if (calSpan < 1) calSpan = 1;
        int shifted = static_cast<int>(raw) - static_cast<int>(LIGHT_RAW_CAL_MIN);
        if (shifted < 0) shifted = 0;
        if (shifted > calSpan) shifted = calSpan;
        p = (static_cast<float>(shifted) * 100.0f) / static_cast<float>(calSpan);
    }

    if (LIGHT_INVERT) p = 100.0f - p;
    if (p < 0.0f) p = 0.0f;
    if (p > 100.0f) p = 100.0f;
    return static_cast<uint8_t>(p + 0.5f);
}

void sampleTopIndicators()
{
    const float rawVoltage = readBatteryVoltage();
    if (!batteryFilterInitialized) {
        batteryVoltage = rawVoltage;
        batteryFilterInitialized = true;
    } else {
        batteryVoltage += BATTERY_FILTER_ALPHA * (rawVoltage - batteryVoltage);
    }
    batteryPercent = batteryPercentFromVoltage(batteryVoltage);

    const unsigned long now = millis();
    if (lastChargeEvalMs == 0) {
        lastChargeEvalMs = now;
        chargePrevVoltage = batteryVoltage;
    } else if (now - lastChargeEvalMs >= CHARGE_DETECT_INTERVAL_MS) {
        const float dv = batteryVoltage - chargePrevVoltage;
        chargePrevVoltage = batteryVoltage;
        lastChargeEvalMs = now;

        if (dv > CHARGE_RISE_THRESHOLD_V) {
            if (chargeTrendScore < CHARGE_SCORE_MAX) chargeTrendScore++;
        } else if (dv < -CHARGE_RISE_THRESHOLD_V) {
            if (chargeTrendScore > -CHARGE_SCORE_MAX) chargeTrendScore--;
        } else {
            if (chargeTrendScore > 0) chargeTrendScore--;
            else if (chargeTrendScore < 0) chargeTrendScore++;
        }
        if (chargeTrendScore >= CHARGE_SCORE_ON) lastChargeSeenMs = now;
        batteryCharging = chargeTrendScore >= CHARGE_SCORE_ON ||
                          (lastChargeSeenMs != 0 && (now - lastChargeSeenMs) <= CHARGE_HOLD_MS);

        if (CHARGE_LOG_TO_SERIAL) {
            Serial.printf("[CHG] V=%.3f dv=%.4f score=%d charging=%d\n",
                          batteryVoltage, dv, chargeTrendScore, batteryCharging ? 1 : 0);
        }
    }

    lightRawAdc = readLightRaw();
    const uint8_t rawLightPercent = lightPercentFromRaw(lightRawAdc);
    if (!lightFilterInitialized) {
        lightPercentFiltered = rawLightPercent;
        lightFilterInitialized = true;
    } else {
        lightPercentFiltered += LIGHT_FILTER_ALPHA * (static_cast<float>(rawLightPercent) - lightPercentFiltered);
    }
    lightPercent = static_cast<uint8_t>(lightPercentFiltered + 0.5f);

    if (LIGHT_LOG_RAW_TO_SERIAL && millis() - lastLightLogMs >= 2000) {
        lastLightLogMs = millis();
        Serial.printf("[LIGHT] raw=%u min=%u max=%u pct=%u\n",
                      lightRawAdc, lightMinObserved, lightMaxObserved, lightPercent);
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

const char *authName(wifi_auth_mode_t auth)
{
    return auth == WIFI_AUTH_OPEN ? "Open" : "Secured";
}

void refreshWifiScan()
{
    const int scanState = WiFi.scanComplete();
    if (scanState == WIFI_SCAN_RUNNING) return;
    WiFi.scanDelete();
    wifiCount = 0;
    wifiScanInProgress = true;
    wifiScanAnimLastMs = millis();
    wifiScanAnimPhase = 0;
    if (!WiFi.scanNetworks(true, true)) {
        wifiScanInProgress = false;
        uiStatusLine = "WiFi scan start failed";
        return;
    }
    uiStatusLine = "Searching for access points";
}

void wifiScanService()
{
    if (!wifiScanInProgress) return;
    const int n = WiFi.scanComplete();
    if (n == WIFI_SCAN_RUNNING) return;

    wifiScanInProgress = false;
    wifiCount = 0;
    if (n > 0) {
        for (int i = 0; i < n && wifiCount < MAX_WIFI_RESULTS; i++) {
            wifiEntries[wifiCount].ssid = WiFi.SSID(i);
            wifiEntries[wifiCount].rssi = WiFi.RSSI(i);
            wifiEntries[wifiCount].auth = WiFi.encryptionType(i);
            wifiCount++;
        }
        uiStatusLine = "Scan complete: " + String(wifiCount);
    } else if (n == 0) {
        uiStatusLine = "No access points found";
    } else {
        uiStatusLine = "WiFi scan failed";
    }
    WiFi.scanDelete();
    if (lvglReady) {
        if (uiScreen == UI_WIFI_LIST) lvglRefreshWifiList();
        lvglSyncStatusLine();
    }
}

bool snakeCellOccupied(int x, int y)
{
    for (int i = 0; i < snakeLen; i++) {
        if (snakeX[i] == x && snakeY[i] == y) return true;
    }
    return false;
}

void snakeSpawnFood()
{
    for (int tries = 0; tries < 300; tries++) {
        int fx = random(0, SNAKE_COLS);
        int fy = random(0, SNAKE_ROWS);
        if (!snakeCellOccupied(fx, fy)) {
            snakeFoodX = static_cast<int8_t>(fx);
            snakeFoodY = static_cast<int8_t>(fy);
            return;
        }
    }
    snakeFoodX = 0;
    snakeFoodY = 0;
}

void snakeResetGame()
{
    snakeLen = 4;
    snakeDir = 1;
    snakeNextDir = 1;
    snakeGameOver = false;
    snakeScore = 0;
    const int cx = SNAKE_COLS / 2;
    const int cy = SNAKE_ROWS / 2;
    for (int i = 0; i < snakeLen; i++) {
        snakeX[i] = static_cast<int8_t>(cx - i);
        snakeY[i] = static_cast<int8_t>(cy);
    }
    snakeSpawnFood();
    snakeLastStepMs = millis();
}

void snakeStep()
{
    if (snakeGameOver) return;
    snakeDir = snakeNextDir;
    int nx = snakeX[0];
    int ny = snakeY[0];
    if (snakeDir == 0) ny--;
    else if (snakeDir == 1) nx++;
    else if (snakeDir == 2) ny++;
    else nx--;

    if (nx < 0 || nx >= SNAKE_COLS || ny < 0 || ny >= SNAKE_ROWS) {
        snakeGameOver = true;
        return;
    }
    if (snakeCellOccupied(nx, ny)) {
        snakeGameOver = true;
        return;
    }

    bool ate = (nx == snakeFoodX && ny == snakeFoodY);
    int oldLen = snakeLen;
    if (ate && snakeLen < SNAKE_MAX_CELLS) snakeLen++;
    for (int i = snakeLen - 1; i > 0; i--) {
        snakeX[i] = snakeX[i - 1];
        snakeY[i] = snakeY[i - 1];
    }
    snakeX[0] = static_cast<int8_t>(nx);
    snakeY[0] = static_cast<int8_t>(ny);

    if (ate) {
        snakeScore += 10;
        snakeSpawnFood();
    } else if (oldLen < snakeLen) {
        snakeLen = oldLen;
    }
}

void snakeTick()
{
    if (uiScreen != UI_GAME_SNAKE) return;
    if (snakeGameOver) return;
    if (millis() - snakeLastStepMs < SNAKE_STEP_MS) return;
    snakeLastStepMs = millis();
    snakeStep();
    lvglRefreshSnakeBoard();
}

static const int8_t TETRIS_BASE[7][4][2] = {
    {{0, 1}, {1, 1}, {2, 1}, {3, 1}}, // I
    {{0, 0}, {0, 1}, {1, 1}, {2, 1}}, // J
    {{2, 0}, {0, 1}, {1, 1}, {2, 1}}, // L
    {{1, 0}, {2, 0}, {1, 1}, {2, 1}}, // O
    {{1, 0}, {2, 0}, {0, 1}, {1, 1}}, // S
    {{1, 0}, {0, 1}, {1, 1}, {2, 1}}, // T
    {{0, 0}, {1, 0}, {1, 1}, {2, 1}}  // Z
};

bool tetrisCellFor(int type, int rot, int i, int &ox, int &oy)
{
    int x = TETRIS_BASE[type][i][0];
    int y = TETRIS_BASE[type][i][1];
    rot &= 3;
    if (rot == 1) {
        int tx = x;
        x = 3 - y;
        y = tx;
    } else if (rot == 2) {
        x = 3 - x;
        y = 3 - y;
    } else if (rot == 3) {
        int tx = x;
        x = y;
        y = 3 - tx;
    }
    ox = x;
    oy = y;
    return true;
}

bool tetrisCanPlace(int px, int py, int type, int rot)
{
    for (int i = 0; i < 4; i++) {
        int ox = 0, oy = 0;
        tetrisCellFor(type, rot, i, ox, oy);
        int gx = px + ox;
        int gy = py + oy;
        if (gx < 0 || gx >= TETRIS_COLS || gy < 0 || gy >= TETRIS_ROWS) return false;
        if (tetrisGrid[gy][gx] != 0) return false;
    }
    return true;
}

void tetrisSpawnPiece()
{
    tetrisType = static_cast<int8_t>(random(0, 7));
    tetrisRot = 0;
    tetrisX = 3;
    tetrisY = 0;
    if (!tetrisCanPlace(tetrisX, tetrisY, tetrisType, tetrisRot)) tetrisGameOver = true;
}

void tetrisClearLines()
{
    for (int y = TETRIS_ROWS - 1; y >= 0; y--) {
        bool full = true;
        for (int x = 0; x < TETRIS_COLS; x++) {
            if (tetrisGrid[y][x] == 0) {
                full = false;
                break;
            }
        }
        if (!full) continue;
        for (int yy = y; yy > 0; yy--) {
            for (int x = 0; x < TETRIS_COLS; x++) tetrisGrid[yy][x] = tetrisGrid[yy - 1][x];
        }
        for (int x = 0; x < TETRIS_COLS; x++) tetrisGrid[0][x] = 0;
        tetrisScore += 100;
        y++;
    }
}

void tetrisLockPiece()
{
    for (int i = 0; i < 4; i++) {
        int ox = 0, oy = 0;
        tetrisCellFor(tetrisType, tetrisRot, i, ox, oy);
        int gx = tetrisX + ox;
        int gy = tetrisY + oy;
        if (gx >= 0 && gx < TETRIS_COLS && gy >= 0 && gy < TETRIS_ROWS) tetrisGrid[gy][gx] = static_cast<uint8_t>(tetrisType + 1);
    }
    tetrisClearLines();
    tetrisSpawnPiece();
}

void tetrisResetGame()
{
    for (int y = 0; y < TETRIS_ROWS; y++) {
        for (int x = 0; x < TETRIS_COLS; x++) tetrisGrid[y][x] = 0;
    }
    tetrisGameOver = false;
    tetrisScore = 0;
    tetrisSpawnPiece();
    tetrisLastStepMs = millis();
}

void tetrisMove(int dx)
{
    if (tetrisGameOver) return;
    if (tetrisCanPlace(tetrisX + dx, tetrisY, tetrisType, tetrisRot)) {
        tetrisX += static_cast<int8_t>(dx);
        lvglRefreshTetrisBoard();
    }
}

void tetrisRotate()
{
    if (tetrisGameOver) return;
    int nr = (tetrisRot + 1) & 3;
    if (tetrisCanPlace(tetrisX, tetrisY, tetrisType, nr)) tetrisRot = static_cast<int8_t>(nr);
    else if (tetrisCanPlace(tetrisX - 1, tetrisY, tetrisType, nr)) {
        tetrisX--;
        tetrisRot = static_cast<int8_t>(nr);
    } else if (tetrisCanPlace(tetrisX + 1, tetrisY, tetrisType, nr)) {
        tetrisX++;
        tetrisRot = static_cast<int8_t>(nr);
    }
    lvglRefreshTetrisBoard();
}

void tetrisDrop()
{
    if (tetrisGameOver) return;
    while (tetrisCanPlace(tetrisX, tetrisY + 1, tetrisType, tetrisRot)) tetrisY++;
    tetrisLockPiece();
    lvglRefreshTetrisBoard();
}

void tetrisStepDown()
{
    if (tetrisGameOver) return;
    if (tetrisCanPlace(tetrisX, tetrisY + 1, tetrisType, tetrisRot)) tetrisY++;
    else tetrisLockPiece();
}

void tetrisTick()
{
    if (uiScreen != UI_GAME_TETRIS) return;
    if (tetrisGameOver) return;
    if (millis() - tetrisLastStepMs < TETRIS_STEP_MS) return;
    tetrisLastStepMs = millis();
    tetrisStepDown();
    lvglRefreshTetrisBoard();
}

bool mediaAllowedExt(const String &name)
{
    String n = name;
    n.toLowerCase();
    return n.endsWith(".mp3") || n.endsWith(".wav") || n.endsWith(".flac") || n.endsWith(".aac") ||
           n.endsWith(".m4a") || n.endsWith(".raw") ||
           n.endsWith(".mpga") || n.endsWith(".mpeg") || n.endsWith(".wave") || n.endsWith(".adts") ||
           n.endsWith(".m4b") || n.endsWith(".f4a");
}

const char *mediaBaseNamePtr(const char *path)
{
    if (!path) return "";
    const char *slash = strrchr(path, '/');
    if (!slash) return path;
    return slash + 1;
}

bool mediaAllowedExtC(const char *name)
{
    if (!name || !name[0]) return false;
    const char *dot = strrchr(name, '.');
    if (!dot || !dot[1]) return false;

    char ext[8] = {0};
    int i = 0;
    for (const char *p = dot; *p && i < static_cast<int>(sizeof(ext) - 1); ++p, ++i) {
        char c = *p;
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
        ext[i] = c;
    }
    ext[i] = '\0';

    return strcmp(ext, ".mp3") == 0 || strcmp(ext, ".wav") == 0 || strcmp(ext, ".flac") == 0 ||
           strcmp(ext, ".aac") == 0 || strcmp(ext, ".m4a") == 0 || strcmp(ext, ".raw") == 0 ||
           strcmp(ext, ".mpga") == 0 || strcmp(ext, ".mpeg") == 0 || strcmp(ext, ".wave") == 0 ||
           strcmp(ext, ".adts") == 0 || strcmp(ext, ".m4b") == 0 || strcmp(ext, ".f4a") == 0;
}

String mediaBuildFullPath(const String &dir, const char *rawName)
{
    if (!rawName || !rawName[0]) return "";
    if (rawName[0] == '/') return String(rawName);

    String raw(rawName);
    if (dir == "/") return "/" + raw;

    String dirNoLead = dir;
    if (dirNoLead.startsWith("/")) dirNoLead.remove(0, 1);
    if (!dirNoLead.isEmpty() && (raw == dirNoLead || raw.startsWith(dirNoLead + "/"))) {
        return "/" + raw;
    }
    return dir + "/" + raw;
}

String mediaBuildListLabel(const String &name, bool isDir, size_t sizeBytes)
{
    String base = name;
    static constexpr int MEDIA_LABEL_MAX_CHARS = 34;
    if (base.length() > MEDIA_LABEL_MAX_CHARS) {
        base = base.substring(0, MEDIA_LABEL_MAX_CHARS - 3) + "...";
    }
    if (isDir) return "[DIR] " + base;

    const uint32_t kb = static_cast<uint32_t>((sizeBytes + 1023U) / 1024U);
    if (kb >= 10240U) return base + " (" + String(static_cast<unsigned long>(kb / 1024U)) + "MB)";
    return base + " (" + String(static_cast<unsigned long>(kb)) + "KB)";
}

void mediaNormalizeDir(String &dir)
{
    if (dir.isEmpty()) dir = "/";
    if (!dir.startsWith("/")) dir = "/" + dir;
    while (dir.length() > 1 && dir.endsWith("/")) dir.remove(dir.length() - 1);
}

String mediaDisplayNameFromPath(const String &path)
{
    const int slash = path.lastIndexOf('/');
    if (slash < 0) return path;
    return path.substring(slash + 1);
}

String mediaDirFromPath(const String &path)
{
    const int slash = path.lastIndexOf('/');
    if (slash <= 0) return "/";
    String dir = path.substring(0, slash);
    mediaNormalizeDir(dir);
    return dir;
}

uint8_t audioVolumeLevelFromPercent(uint8_t percent)
{
    if (percent > 100) percent = 100;
    return static_cast<uint8_t>((static_cast<uint16_t>(percent) * AUDIO_VOLUME_TARGET + 50U) / 100U);
}

void mediaFormatSeconds(uint32_t sec, char *out, size_t outLen)
{
    if (!out || outLen == 0) return;
    const uint32_t mins = sec / 60U;
    const uint32_t secs = sec % 60U;
    snprintf(out, outLen, "%02lu:%02lu", static_cast<unsigned long>(mins), static_cast<unsigned long>(secs));
}

String mediaFindAdjacentTrack(const String &sourcePath, bool nextDir)
{
    String dir = sourcePath.isEmpty() ? mediaCurrentDir : mediaDirFromPath(sourcePath);
    mediaNormalizeDir(dir);
    String currentKey = mediaDisplayNameFromPath(sourcePath);
    currentKey.toLowerCase();
    const bool hasCurrent = !sourcePath.isEmpty();

    if (!sdEnsureMounted() && !sdEnsureMounted(true)) return "";
    if (!sdLock()) return "";

    String firstPath;
    String firstKey;
    String lastPath;
    String lastKey;
    String bestPath;
    String bestKey;
    int scanned = 0;

    File folder = SD.open(dir);
    if (!folder || !folder.isDirectory()) {
        if (folder) folder.close();
        sdUnlock();
        return "";
    }

    File entry = folder.openNextFile();
    while (entry) {
        const char *rawName = entry.name();
        const char *baseName = mediaBaseNamePtr(rawName);
        if (!entry.isDirectory() && mediaAllowedExtC(baseName)) {
            String fullPath = mediaBuildFullPath(dir, rawName);
            if (!fullPath.isEmpty()) {
                String key(baseName);
                key.toLowerCase();

                if (firstPath.isEmpty() || key.compareTo(firstKey) < 0) {
                    firstPath = fullPath;
                    firstKey = key;
                }
                if (lastPath.isEmpty() || key.compareTo(lastKey) > 0) {
                    lastPath = fullPath;
                    lastKey = key;
                }

                if (hasCurrent) {
                    const int cmp = key.compareTo(currentKey);
                    if (nextDir) {
                        if (cmp > 0 && (bestPath.isEmpty() || key.compareTo(bestKey) < 0)) {
                            bestPath = fullPath;
                            bestKey = key;
                        }
                    } else {
                        if (cmp < 0 && (bestPath.isEmpty() || key.compareTo(bestKey) > 0)) {
                            bestPath = fullPath;
                            bestKey = key;
                        }
                    }
                }
            }
        }

        entry.close();
        entry = folder.openNextFile();
        scanned++;
        if ((scanned & 0x0F) == 0) delay(0);
    }
    if (entry) entry.close();
    folder.close();
    sdUnlock();

    if (!bestPath.isEmpty()) return bestPath;
    return nextDir ? firstPath : lastPath;
}

void rgbApplyNow(bool rOn, bool gOn, bool bOn)
{
    auto level = [](bool on) -> uint8_t {
        if (RGB_ACTIVE_LOW) return on ? LOW : HIGH;
        return on ? HIGH : LOW;
    };
    // Board LED channels are physically swapped: logical R<->G.
    digitalWrite(RGB_PIN_R, level(gOn));
    digitalWrite(RGB_PIN_G, level(rOn));
    digitalWrite(RGB_PIN_B, level(bOn));
}

void rgbSetPersistent(bool rOn, bool gOn, bool bOn)
{
    rgbPersistR = rOn;
    rgbPersistG = gOn;
    rgbPersistB = bOn;
    if (rgbFlashUntilMs == 0) rgbApplyNow(rOn, gOn, bOn);
}

void rgbFlashAcknowledge()
{
    rgbApplyNow(true, true, true);
    rgbFlashUntilMs = millis() + 150;
}

void rgbApplyPwm(uint8_t r, uint8_t g, uint8_t b)
{
    auto out = [](uint8_t val) -> uint8_t {
        return RGB_ACTIVE_LOW ? static_cast<uint8_t>(255 - val) : val;
    };
    // Board LED channels are physically swapped: logical R<->G.
    analogWrite(RGB_PIN_R, out(g));
    analogWrite(RGB_PIN_G, out(r));
    analogWrite(RGB_PIN_B, out(b));
}

void rgbService()
{
    if (!batteryCharging && batteryPercent < 5) {
        unsigned long now = millis();
        if (now - rgbPulseLastMs >= 20) {
            rgbPulseLastMs = now;
            int v = static_cast<int>(rgbPulseLevel) + (rgbPulseDir * 5);
            if (v >= 255) {
                v = 255;
                rgbPulseDir = -1;
            } else if (v <= 20) {
                v = 20;
                rgbPulseDir = 1;
            }
            rgbPulseLevel = static_cast<uint8_t>(v);
        }
        rgbApplyPwm(rgbPulseLevel, 0, 0);
        return;
    }
    if (batteryCharging) {
        if (batteryPercent >= 98) {
            // On this board revision, physical GREEN is on logical B channel.
            rgbApplyNow(false, false, true);
            return;
        }
        unsigned long now = millis();
        if (now - rgbPulseLastMs >= 20) {
            rgbPulseLastMs = now;
            int v = static_cast<int>(rgbPulseLevel) + (rgbPulseDir * 5);
            if (v >= 255) {
                v = 255;
                rgbPulseDir = -1;
            } else if (v <= 20) {
                v = 20;
                rgbPulseDir = 1;
            }
            rgbPulseLevel = static_cast<uint8_t>(v);
        }
        // On this board revision, physical GREEN is on logical B channel.
        rgbApplyPwm(0, 0, rgbPulseLevel);
        return;
    }
    if (!displayAwake) {
        rgbApplyNow(false, false, false);
        return;
    }
    if (rgbFlashUntilMs != 0) return;
    if (mediaIsPlaying && !mediaPaused) {
        unsigned long now = millis();
        if (now - rgbPulseLastMs >= 20) {
            rgbPulseLastMs = now;
            int v = static_cast<int>(rgbPulseLevel) + (rgbPulseDir * 5);
            if (v >= 255) {
                v = 255;
                rgbPulseDir = -1;
            } else if (v <= 20) {
                v = 20;
                rgbPulseDir = 1;
            }
            rgbPulseLevel = static_cast<uint8_t>(v);
        }
        rgbApplyPwm(0, rgbPulseLevel, 0);
        return;
    }
    rgbApplyNow(rgbPersistR, rgbPersistG, rgbPersistB);
}

void rgbRefreshByMediaState()
{
    if (mediaIsPlaying && !mediaPaused) {
        rgbSetPersistent(false, true, false); // Green: playing
    } else {
        rgbSetPersistent(true, false, false); // Red: idle/stopped
    }
}

bool audioEnsureBackendReady(const char *reason)
{
    if (audioBackendReady && audio) return true;
    const unsigned long now = millis();
    if (static_cast<unsigned long>(now - audioLastInitAttemptMs) < AUDIO_INIT_RETRY_MS) return false;
    audioLastInitAttemptMs = now;

    const uint32_t freeHeap = static_cast<uint32_t>(ESP.getFreeHeap());
    const uint32_t dmaLargest = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
    if (freeHeap < AUDIO_INIT_MIN_FREE_HEAP || dmaLargest < AUDIO_INIT_MIN_DMA_BLOCK) {
        Serial.printf("[AUDIO] init skipped: heap=%lu dma=%lu reason=%s\n",
                      static_cast<unsigned long>(freeHeap),
                      static_cast<unsigned long>(dmaLargest),
                      (reason && reason[0]) ? reason : "-");
        return false;
    }

    Audio *created = new (std::nothrow) Audio(true, AUDIO_INTERNAL_DAC_CHANNEL, AUDIO_I2S_PORT);
    if (!created) {
        Serial.printf("[AUDIO] init failed: new Audio null reason=%s\n", (reason && reason[0]) ? reason : "-");
        return false;
    }

    audio = created;
    audio->setBufsize(AUDIO_INPUT_RAM_BUFFER_BYTES, -1);
    audio->forceMono(AUDIO_FORCE_MONO_INTERNAL_DAC);
    audioBackendReady = true;
    audio->setVolume(audioVolumeLevelFromPercent(mediaVolumePercent));
    audioVolumeCurrent = audioVolumeLevelFromPercent(mediaVolumePercent);
    Serial.printf("[AUDIO] backend ready: internal_dac=1 channel=%u i2s_port=%u heap=%lu dma=%lu reason=%s\n",
                  static_cast<unsigned int>(AUDIO_INTERNAL_DAC_CHANNEL),
                  static_cast<unsigned int>(AUDIO_I2S_PORT),
                  static_cast<unsigned long>(freeHeap),
                  static_cast<unsigned long>(dmaLargest),
                  (reason && reason[0]) ? reason : "-");
    return true;
}

void audioSetVolumeImmediate(uint8_t v)
{
    if (!audio) return;
    audio->setVolume(v);
    audioVolumeCurrent = v;
}

String mediaAliasTargetExt(const String &path)
{
    String p = path;
    p.toLowerCase();
    if (p.endsWith(".mpga") || p.endsWith(".mpeg")) return ".mp3";
    if (p.endsWith(".wave")) return ".wav";
    if (p.endsWith(".adts")) return ".aac";
    if (p.endsWith(".m4b") || p.endsWith(".f4a")) return ".m4a";
    return "";
}

bool mediaPathIsFlac(const String &path)
{
    String n = path;
    n.toLowerCase();
    return n.endsWith(".flac");
}

bool audioFlacSupportedNow(uint32_t *freeHeapOut, uint32_t *largestOut)
{
    const uint32_t freeHeap = static_cast<uint32_t>(ESP.getFreeHeap());
    const uint32_t largest = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    if (freeHeapOut) *freeHeapOut = freeHeap;
    if (largestOut) *largestOut = largest;
    if (psramFound()) return true;
    return (freeHeap >= AUDIO_FLAC_MIN_FREE_HEAP) && (largest >= AUDIO_FLAC_MIN_LARGEST_8BIT);
}

bool copyFileToPath(const String &srcPath, const String &dstPath)
{
    for (int attempt = 0; attempt < 2; attempt++) {
        if (!sdEnsureMounted(attempt > 0)) return false;
        if (!sdLock()) return false;

        bool ok = true;
        File src = SD.open(srcPath, FILE_READ);
        if (!src || src.isDirectory()) {
            if (src) src.close();
            ok = false;
        }

        File dst;
        if (ok) {
            if (SD.exists(dstPath)) SD.remove(dstPath);
            dst = SD.open(dstPath, FILE_WRITE);
            if (!dst) ok = false;
        }

        if (ok) {
            uint8_t buf[1024];
            while (src.available()) {
                int n = src.read(buf, sizeof(buf));
                if (n <= 0) break;
                if (dst.write(buf, n) != static_cast<size_t>(n)) {
                    ok = false;
                    break;
                }
            }
            if (ok) {
                dst.flush();
                ok = (dst.getWriteError() == 0);
            }
        }

        if (src) src.close();
        if (dst) dst.close();
        if (!ok && SD.exists(dstPath)) SD.remove(dstPath);
        sdUnlock();

        if (ok) return true;
        sdMarkFault("copyFileToPath");
        delay(15);
    }
    return false;
}

String mediaResolvePlaybackPath(const String &originalPath)
{
    String aliasExt = mediaAliasTargetExt(originalPath);
    if (aliasExt.isEmpty()) return originalPath;

    String aliasPath = "/.__play_alias" + aliasExt;
    if (!copyFileToPath(originalPath, aliasPath)) return "";
    return aliasPath;
}

void audioStopPlayback(bool smooth = true)
{
    (void)smooth;
    if (audioBackendReady && audio) audio->stopSong();
    if (mediaPlaybackPath.startsWith("/.__play_alias")) {
        if (sdLock()) {
            SD.remove(mediaPlaybackPath);
            sdUnlock();
        }
        mediaPlaybackPath = "";
    }
    mediaIsPlaying = false;
    mediaPaused = false;
    rgbRefreshByMediaState();
}

bool audioStartFile(const String &path)
{
    audioLastError = "";
    if (mediaPathIsFlac(path)) {
        uint32_t freeHeap = 0;
        uint32_t largest = 0;
        if (!audioFlacSupportedNow(&freeHeap, &largest)) {
            audioLastError = "flac_mem";
            Serial.printf("[AUDIO] FLAC rejected: no_psram heap=%lu largest=%lu need_heap>=%lu need_largest>=%lu path=%s\n",
                          static_cast<unsigned long>(freeHeap),
                          static_cast<unsigned long>(largest),
                          static_cast<unsigned long>(AUDIO_FLAC_MIN_FREE_HEAP),
                          static_cast<unsigned long>(AUDIO_FLAC_MIN_LARGEST_8BIT),
                          path.c_str());
            return false;
        }
    }
    if (!sdEnsureMounted() && !sdEnsureMounted(true)) return false;
    if (!audioEnsureBackendReady("playback")) return false;
    if (!networkSuspendedForAudio && !psramFound()) {
        const uint32_t largestNow = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        if (largestNow < AUDIO_PREEMPTIVE_NET_SUSPEND_LARGEST_8BIT) {
            Serial.printf("[AUDIO] preemptive network suspend: no_psram largest=%lu < %lu\n",
                          static_cast<unsigned long>(largestNow),
                          static_cast<unsigned long>(AUDIO_PREEMPTIVE_NET_SUSPEND_LARGEST_8BIT));
            networkSuspendForAudio();
            delay(20);
        }
    }
    audio->stopSong();
    const uint32_t freeHeapBefore = static_cast<uint32_t>(ESP.getFreeHeap());
    const uint32_t largestBefore = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    if (!audio->connecttoFS(SD, path.c_str())) {
        bool fileExists = false;
        if (sdLock()) {
            fileExists = SD.exists(path);
            sdUnlock();
        }

        const uint32_t freeHeapFail = static_cast<uint32_t>(ESP.getFreeHeap());
        const uint32_t largestFail = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        Serial.printf("[AUDIO] connecttoFS failed path=%s exists=%d heap_before=%lu largest_before=%lu heap_fail=%lu largest_fail=%lu\n",
                      path.c_str(),
                      fileExists ? 1 : 0,
                      static_cast<unsigned long>(freeHeapBefore),
                      static_cast<unsigned long>(largestBefore),
                      static_cast<unsigned long>(freeHeapFail),
                      static_cast<unsigned long>(largestFail));

        // Decoder OOM (for example mp3/flac) can fail connecttoFS while SD remains healthy.
        // Only trigger SD fault/recovery when the file itself is not accessible.
        if (!fileExists) {
            audioLastError = "sd_access";
            sdMarkFault("audioStartFile");
            if (!sdEnsureMounted(true) || !audio->connecttoFS(SD, path.c_str())) return false;
        } else {
            if (!networkSuspendedForAudio) {
                Serial.println("[AUDIO] decoder allocation failed, retry with network suspended");
                networkSuspendForAudio();
                delay(60);
                audio->stopSong();
                if (audio->connecttoFS(SD, path.c_str())) {
                    Serial.printf("[AUDIO] retry ok after network suspend path=%s\n", path.c_str());
                } else {
                    audioLastError = "decoder_mem";
                    return false;
                }
            } else {
                audioLastError = "decoder_mem";
                return false;
            }
        }
    }
    audioSetVolumeImmediate(audioVolumeLevelFromPercent(mediaVolumePercent));
    mediaIsPlaying = true;
    mediaPaused = false;
    rgbRefreshByMediaState();
    return true;
}

bool mediaStartTrack(const String &sourcePath, const String &displayName)
{
    if (sourcePath.isEmpty()) return false;
    audioStopPlayback(false);

    String playPath = mediaResolvePlaybackPath(sourcePath);
    if (playPath.isEmpty()) playPath = sourcePath;
    if (!audioStartFile(playPath)) {
        if (audioLastError.isEmpty()) audioLastError = "start_failed";
        return false;
    }

    mediaSelectedSourcePath = sourcePath;
    mediaSelectedTrackName = displayName.isEmpty() ? mediaDisplayNameFromPath(sourcePath) : displayName;
    mediaPlaybackPath = playPath;
    mediaNowPlaying = mediaSelectedTrackName;
    uiStatusLine = "Playing: " + mediaSelectedTrackName;
    lvglSyncStatusLine();
    return true;
}

void audioSetPaused(bool pause)
{
    if (!audioBackendReady || !audio) return;
    if (!audio->isRunning()) return;
    if (mediaPaused != pause) audio->pauseResume();
    mediaPaused = pause;
    mediaIsPlaying = !pause;
    rgbRefreshByMediaState();
}

void audioService()
{
    if (!audioBackendReady || !audio) return;
    audio->loop();
}

void loadMediaEntries()
{
    mediaCount = 0;
    mediaHasMore = false;
    mediaNormalizeDir(mediaCurrentDir);
    mediaScanRuns++;
    const unsigned long scanStartMs = millis();
    int scanned = 0;
    int eligibleSeen = 0;

    if (!sdEnsureMounted() && !sdEnsureMounted(true)) {
        uiStatusLine = "SD unavailable";
        return;
    }
    if (!sdLock()) {
        uiStatusLine = "SD busy";
        return;
    }

    bool finished = false;
    for (int pass = 0; pass < 2 && !finished; pass++) {
        scanned = 0;
        eligibleSeen = 0;
        mediaCount = 0;
        mediaHasMore = false;

        File dir = SD.open(mediaCurrentDir);
        if (!dir || !dir.isDirectory()) {
            if (dir) dir.close();
            if (mediaCurrentDir != "/") {
                mediaCurrentDir = "/";
                mediaOffset = 0;
                continue;
            }
            uiStatusLine = "Invalid dir: " + mediaCurrentDir;
            sdUnlock();
            return;
        }

        File entry = dir.openNextFile();
        while (entry) {
            const char *rawName = entry.name();
            const char *baseName = mediaBaseNamePtr(rawName);
            const bool isDir = entry.isDirectory();
            const size_t size = isDir ? 0 : entry.size();
            const bool eligible = isDir || mediaAllowedExtC(baseName);

            if (eligible) {
                if (eligibleSeen < mediaOffset) {
                    eligibleSeen++;
                } else if (mediaCount < MEDIA_PAGE_SIZE && mediaCount < MAX_MEDIA_ENTRIES) {
                    String full = mediaBuildFullPath(mediaCurrentDir, rawName);
                    if (full.length() > 0) {
                        mediaEntries[mediaCount] = {String(baseName), full, isDir, size};
                        mediaCount++;
                    }
                    eligibleSeen++;
                } else {
                    mediaHasMore = true;
                    break;
                }
            }

            entry.close();
            entry = dir.openNextFile();
            scanned++;
            if ((scanned % MEDIA_SCAN_YIELD_EVERY) == 0) {
                if (mediaIsPlaying && !mediaPaused) audioService();
                delay(0);
            }
        }
        if (entry) entry.close();
        dir.close();

        if (mediaOffset > 0 && mediaCount == 0) {
            mediaOffset = 0;
            continue;
        }
        finished = true;
    }
    sdUnlock();

    const int page = (mediaOffset / MEDIA_PAGE_SIZE) + 1;
    uiStatusLine = "Media p" + String(page) + ": " + String(mediaCount) + " item(s)";
    if (mediaHasMore) uiStatusLine += " +";
    const unsigned long scanMs = static_cast<unsigned long>(millis() - scanStartMs);
    if (scanMs >= MEDIA_SCAN_SLOW_MS) {
        mediaScanSlowRuns++;
        Serial.printf("[MEDIA] slow scan dir=%s page=%d ms=%lu scanned=%d eligible=%d count=%d more=%d slow_runs=%lu/%lu\n",
                      mediaCurrentDir.c_str(),
                      page,
                      scanMs,
                      scanned,
                      eligibleSeen,
                      mediaCount,
                      mediaHasMore ? 1 : 0,
                      static_cast<unsigned long>(mediaScanSlowRuns),
                      static_cast<unsigned long>(mediaScanRuns));
    } else {
        Serial.printf("[MEDIA] scan dir=%s page=%d ms=%lu scanned=%d eligible=%d count=%d more=%d\n",
                      mediaCurrentDir.c_str(),
                      page,
                      scanMs,
                      scanned,
                      eligibleSeen,
                      mediaCount,
                      mediaHasMore ? 1 : 0);
    }
    rgbRefreshByMediaState();
}

void displaySetAwake(bool awake)
{
    displayAwake = awake;
    displayBacklightSet(awake ? displayBacklightLevelFromPercent(displayBrightnessPercent) : TFT_BL_LEVEL_OFF);
    if (awake) {
        wakeTouchConfirmCount = 0;
        rgbRefreshByMediaState();
        if (lvglReady) {
            // Charge animation is drawn directly to TFT; force LVGL to repaint active menu screen on wake.
            lv_obj_invalidate(lv_scr_act());
            lv_timer_handler();
            lvglRefreshInfoPanel();
            lvglRefreshTopIndicators();
        }
    } else {
        wakeTouchReleaseGuard = false;
        if (!batteryCharging) rgbApplyNow(false, false, false);
    }
}

bool canEnterLowPowerSleep(bool touchDownNow)
{
    if (touchDownNow) return false;
    if (wifiConnectedSafe()) return false; // requested: sleep only when not connected
    if (batteryCharging) return false;
    if (batteryPercent < 5) return false;
    if (fsWriteBusy()) return false;
    if (bootStaConnectInProgress) return false;
    if (mediaIsPlaying || mediaPaused) return false;
    if (displayAwake) return false;
    if (millis() - lastUserActivityMs < LIGHT_SLEEP_AFTER_IDLE_MS) return false;
    return true;
}

void enterLowPowerSleep()
{
    // Wake on CST820 IRQ low level touch signal when available.
    if (TOUCH_USE_IRQ) {
        gpio_wakeup_enable(static_cast<gpio_num_t>(TOUCH_IRQ), GPIO_INTR_LOW_LEVEL);
        esp_sleep_enable_gpio_wakeup();
    }
    // Optional safety wake in case touch IRQ is not wired on this board revision.
    if (LIGHT_SLEEP_TIMER_FALLBACK || !TOUCH_USE_IRQ) esp_sleep_enable_timer_wakeup(LIGHT_SLEEP_TIMER_US);
    esp_light_sleep_start();
}

void loadSavedStaCreds()
{
    wifiPrefs.begin("wifi", true);
    savedStaSsid = wifiPrefs.getString("sta_ssid", "");
    savedStaPass = wifiPrefs.getString("sta_pass", "");
    savedApSsid = wifiPrefs.getString("ap_ssid", AP_SSID);
    savedApPass = wifiPrefs.getString("ap_pass", AP_PASS);
    wifiPrefs.end();
    if (savedApSsid.isEmpty()) savedApSsid = AP_SSID;
}

bool loadBatterySnapshot(float &vbatOut)
{
    batteryPrefs.begin("battery", true);
    bool valid = batteryPrefs.getBool("valid", false);
    float v = batteryPrefs.getFloat("vbat", 0.0f);
    batteryPrefs.end();
    if (!valid || v <= 0.0f) return false;
    vbatOut = v;
    return true;
}

void saveBatterySnapshot(float vbat)
{
    batteryPrefs.begin("battery", false);
    batteryPrefs.putBool("valid", true);
    batteryPrefs.putFloat("vbat", vbat);
    batteryPrefs.end();
}

void saveStaCreds(const String &ssid, const String &pass)
{
    wifiPrefs.begin("wifi", false);
    wifiPrefs.putString("sta_ssid", ssid);
    wifiPrefs.putString("sta_pass", pass);
    wifiPrefs.end();
    savedStaSsid = ssid;
    savedStaPass = pass;
}

void saveApCreds(const String &ssid, const String &pass)
{
    wifiPrefs.begin("wifi", false);
    wifiPrefs.putString("ap_ssid", ssid);
    wifiPrefs.putString("ap_pass", pass);
    wifiPrefs.end();
    savedApSsid = ssid;
    savedApPass = pass;
}

static bool parseIntMessageValue(const char *value, int &out)
{
    if (!value || !*value) return false;
    char *end = nullptr;
    const long parsed = strtol(value, &end, 10);
    if (!end || *end != '\0') return false;
    if (parsed < INT32_MIN || parsed > INT32_MAX) return false;
    out = static_cast<int>(parsed);
    return true;
}

static void appendUiWidgetPositionIfPresent(JsonDocument &doc,
                                            Preferences &prefs,
                                            const char *jsonX,
                                            const char *jsonY,
                                            const char *prefX,
                                            const char *prefY)
{
    if (!prefs.isKey(prefX) || !prefs.isKey(prefY)) return;
    doc[jsonX] = prefs.getInt(prefX, 0);
    doc[jsonY] = prefs.getInt(prefY, 0);
}

void appendUiSettings(JsonDocument &doc)
{
    uiPrefs.begin("ui", true);

    doc["RecordTelemetry"] = uiPrefs.getBool("record_tel", recordTelemetryEnabled) ? 1 : 0;
    doc["SystemSounds"] = uiPrefs.getBool("sys_snd", systemSoundsEnabled) ? 1 : 0;
    doc["SystemVolume"] = uiPrefs.getUChar("sys_vol", mediaVolumePercent);
    doc["DisplayBrightness"] = uiPrefs.getUChar("disp_bri", displayBrightnessPercent);
    doc["DeviceName"] = uiPrefs.getString("dev_name", deviceShortNameValue());
    doc["WsRebootOnDisconnect"] = uiPrefs.getBool("ws_reboot", wsRebootOnDisconnectEnabled) ? 1 : 0;
    doc["AirplaneMode"] = uiPrefs.getBool("airplane", airplaneModeEnabled) ? 1 : 0;
    doc["TelemetryMaxKB"] = uiPrefs.getUInt("tele_kb", telemetryMaxKB);
    doc["IndicatorsVisible"] = uiPrefs.getBool("ind_v", true) ? 1 : 0;
    doc["ImuVisible"] = uiPrefs.getBool("imu_v", true) ? 1 : 0;
    doc["MediaVisible"] = uiPrefs.getBool("med_v", true) ? 1 : 0;
    doc["PathVisible"] = uiPrefs.getBool("path_v", true) ? 1 : 0;
    doc["Model3DVisible"] = uiPrefs.getBool("mod_v", true) ? 1 : 0;
    doc["SerialVisible"] = uiPrefs.getBool("ser_v", true) ? 1 : 0;

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

    uiPrefs.end();
}

void loadUiRuntimeConfig()
{
    uiPrefs.begin("ui", true);
    recordTelemetryEnabled = uiPrefs.getBool("record_tel", false);
    systemSoundsEnabled = uiPrefs.getBool("sys_snd", true);
    mediaVolumePercent = uiPrefs.getUChar("sys_vol", mediaVolumePercent);
    displayBrightnessPercent = static_cast<uint8_t>(constrain(uiPrefs.getUChar("disp_bri", displayBrightnessPercent), 5, 100));
    deviceShortName = sanitizeDeviceShortName(uiPrefs.getString("dev_name", DEVICE_SHORT_NAME));
    if (deviceShortName.isEmpty()) deviceShortName = DEVICE_SHORT_NAME;
    wsRebootOnDisconnectEnabled = uiPrefs.getBool("ws_reboot", false);
    airplaneModeEnabled = uiPrefs.getBool("airplane", false);
    telemetryMaxKB = uiPrefs.getUInt("tele_kb", 512);
    serialLogWsMinIntervalMs = uiPrefs.getUInt("ser_rate", SERIAL_LOG_RATE_MS_DEFAULT);
    serialLogKeepLines = static_cast<size_t>(uiPrefs.getUInt("ser_keep", SERIAL_LOG_RING_SIZE));
    uiPrefs.end();
    audioSetVolumeImmediate(mediaVolumePercent);
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
        audioSetVolumeImmediate(mediaVolumePercent);
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
    else if (strcmp(key, "TelemetryMaxKB") == 0 && parseIntMessageValue(value, parsed)) {
        telemetryMaxKB = static_cast<uint32_t>(max(32, min(8192, parsed)));
        uiPrefs.putUInt("tele_kb", telemetryMaxKB);
    }
    else if (strcmp(key, "IndicatorsVisible") == 0) uiPrefs.putBool("ind_v", atoi(value) != 0);
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
    serialLogRing = static_cast<char *>(heap_caps_malloc(bytes, MALLOC_CAP_8BIT));
    if (!serialLogRing) return false;
    memset(serialLogRing, 0, bytes);
    serialLogHead = 0;
    serialLogCount = 0;
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

void ensureApOnline(const char *reason)
{
    if (networkSuspendedForAudio || airplaneModeEnabled) return;
    if (apModeActive) return;
    if (WiFi.getMode() != WIFI_AP_STA) WiFi.mode(WIFI_AP_STA);
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
    Serial.printf("[WIFI] AP enabled reason=%s ip=%s\n",
                  reason ? reason : "-",
                  ip.toString().c_str());
}

void disableApWhenStaConnected(const char *reason)
{
    if (!apModeActive && WiFi.getMode() == WIFI_STA) return;
    stopDnsForAp();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    apModeActive = false;
    Serial.printf("[WIFI] AP disabled reason=%s\n", reason ? reason : "-");
}

static void beginStaConnectAttempt(const char *reason)
{
    if (networkSuspendedForAudio || airplaneModeEnabled || !wifiHasStaTarget()) return;
    const String ssid = wifiDesiredStaSsid();
    const String pass = wifiDesiredStaPass();
    const wifi_mode_t mode = apModeActive ? WIFI_AP_STA : WIFI_STA;
    if (WiFi.getMode() != mode) WiFi.mode(mode);
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

void mqttBuildIdentity()
{
    mqttHwId = buildMqttHardwareId();
    mqttNodeId = "esp32_remote_" + mqttHwId;
    mqttClientId = "esp32-remote-" + mqttHwId;
    mqttDeviceName = "ESP32 Remote " + mqttHwId;
    mqttActionTopic = "esp32/remote/" + mqttHwId + "/action";
    mqttChatInboxTopic = "esp32/remote/chat/" + String(MQTT_CHAT_NAMESPACE) + "/inbox/" + p2pPublicKeyHex();
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
        if (mqttChatInboxTopic.isEmpty() || strcmp(topic, mqttChatInboxTopic.c_str()) != 0) return;
        if (length > MQTT_CLIENT_BUFFER_SIZE) return;
        if (ESP.getFreeHeap() < MQTT_MIN_FREE_HEAP_PUBLISH ||
            heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) < MQTT_MIN_LARGEST_8BIT_PUBLISH) return;
        JsonDocument doc;
        if (deserializeJson(doc, payload, length) != DeserializationError::Ok) return;
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
        if (kind == "ack") {
            const String ackId = String(static_cast<const char *>(plainDoc["ack_id"] | ""));
            if (!ackId.isEmpty()) chatAckOutgoingMessage(senderPubHex, ackId);
            return;
        }
        const String messageId = String(static_cast<const char *>(plainDoc["id"] | ""));
        const String text = String(static_cast<const char *>(plainDoc["text"] | ""));
        if (text.isEmpty()) return;
        if (messageId.isEmpty() || !chatHasLoggedMessageId(senderPubHex, messageId)) {
            chatStoreMessage(senderPubHex, author, text, false, CHAT_TRANSPORT_MQTT, messageId);
            uiStatusLine = "Global chat from " + author;
            if (lvglReady) {
                lvglSyncStatusLine();
                if (uiScreen == UI_CHAT) lvglRefreshChatUi();
            }
        }
        if (!messageId.isEmpty()) mqttPublishChatAck(senderPubHex, messageId);
    });
    const String availabilityTopic = "esp32/remote/" + mqttHwId + "/availability";
    bool ok = false;
    if (mqttCfg.username.length()) {
        ok = mqttClient.connect(mqttClientId.c_str(), mqttCfg.username.c_str(), mqttCfg.password.c_str(), availabilityTopic.c_str(), 0, true, "offline");
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

void mqttPublishDiscovery()
{
    if (!mqttClient.connected()) return;
    if (!mqttHasHeapHeadroom(MQTT_MIN_FREE_HEAP_PUBLISH, MQTT_MIN_LARGEST_8BIT_PUBLISH, "MQTT discovery deferred")) return;
    const String availTopic = "esp32/remote/" + mqttHwId + "/availability";
    for (int i = mqttButtonCount; i < MQTT_MAX_BUTTONS; i++) {
        const String oldTopic = mqttCfg.discoveryPrefix + "/device_automation/" + mqttNodeId + "/" + String(i) + "/config";
        mqttClient.publish(oldTopic.c_str(), "", true);
    }
    for (int i = 0; i < mqttButtonCount; i++) {
        const String action = mqttButtonPayloadForIndex(i);
        const String topic = mqttCfg.discoveryPrefix + "/device_automation/" + mqttNodeId + "/" + String(i) + "/config";
        JsonDocument doc;
        doc["automation_type"] = "trigger";
        doc["topic"] = mqttActionTopic;
        doc["payload"] = action;
        doc["type"] = "button_short_press";
        doc["subtype"] = "button_" + String(i + 1);
        doc["unique_id"] = mqttNodeId + "_" + String(i + 1) + "_" + action;
        doc["availability_topic"] = availTopic;
        JsonObject dev = doc["device"].to<JsonObject>();
        JsonArray ids = dev["identifiers"].to<JsonArray>();
        ids.add(mqttNodeId);
        dev["name"] = mqttDeviceName;
        dev["model"] = DEVICE_MODEL;
        dev["manufacturer"] = "Elik";
        dev["sw_version"] = FW_VERSION;
        String payload;
        serializeJson(doc, payload);
        mqttClient.publish(topic.c_str(), payload.c_str(), true);
    }
    mqttDiscoveryPublished = true;
    mqttStatusLine = "HA discovery published";
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

bool pathUnderScreenshots(const String &path)
{
    String p = path;
    if (!p.startsWith("/")) p = "/" + p;
    String l = p;
    l.toLowerCase();
    return (l == "/screenshots") || l.startsWith("/screenshots/");
}

bool pathUnderConversations(const String &path)
{
    String p = path;
    if (!p.startsWith("/")) p = "/" + p;
    String l = p;
    l.toLowerCase();
    return (l == "/conversations") || l.startsWith("/conversations/");
}

bool pathAllowedInRecovery(const String &path)
{
    String p = path;
    if (!p.length()) return false;
    if (!p.startsWith("/")) p = "/" + p;
    return p.startsWith("/");
}

void ensureFolderExists(const String &fullPath)
{
    int lastSlash = fullPath.lastIndexOf('/');
    if (lastSlash <= 0) return;
    String folder = fullPath.substring(0, lastSlash);
    if (!SD.exists(folder)) SD.mkdir(folder);
}

void listSdFilesRecursive(const String &path, JsonArray out)
{
    File dir = SD.open(path);
    if (!dir) return;
    File entry = dir.openNextFile();
    while (entry) {
        String child = entry.name();
        bool isDir = entry.isDirectory();
        uint32_t size = isDir ? 0 : entry.size();
        if (!child.startsWith("/")) child = (path == "/") ? ("/" + child) : (path + "/" + child);

        JsonObject it = out.add<JsonObject>();
        it["path"] = child;
        it["size"] = size;

        entry.close();
        if (isDir) listSdFilesRecursive(child, out);
        entry = dir.openNextFile();
    }
    dir.close();
}

void listSdDirEntries(const String &path, JsonArray out)
{
    File dir = SD.open(path);
    if (!dir || !dir.isDirectory()) {
        if (dir) dir.close();
        return;
    }

    File entry = dir.openNextFile();
    while (entry) {
        String child = entry.name();
        bool isDir = entry.isDirectory();
        uint32_t size = isDir ? 0 : entry.size();
        if (!child.startsWith("/")) child = (path == "/") ? ("/" + child) : (path + "/" + child);
        String name = child;
        int slash = name.lastIndexOf('/');
        if (slash >= 0) name = name.substring(slash + 1);

        JsonObject it = out.add<JsonObject>();
        it["name"] = name;
        it["path"] = child;
        it["isDir"] = isDir;
        it["size"] = size;

        entry.close();
        entry = dir.openNextFile();
    }
    dir.close();
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

String normalizeSdRoutePath(const String &rawPath, bool allowEmptyRoot)
{
    String path = rawPath;
    if (path.isEmpty()) path = allowEmptyRoot ? String("/") : String("");
    if (!path.length()) return path;
    if (!path.startsWith("/")) path = "/" + path;
    while (path.indexOf("//") >= 0) path.replace("//", "/");
    while (path.endsWith("/") && path.length() > 1) path.remove(path.length() - 1);
    return path;
}

bool sdShouldHideSystemEntry(const String &name)
{
    return name.startsWith(".") ||
           name.endsWith(".index") ||
           name.endsWith(".path") ||
           name.endsWith(".bak") ||
           name.endsWith(".meta") ||
           name.equalsIgnoreCase("System Volume Information") ||
           name.startsWith("FOUND.") ||
           name == "Thumbs.db";
}

String makeUniquePath(const String &path)
{
    if (!sdEnsureMounted()) return path;
    if (!SD.exists(path)) return path;

    int slash = path.lastIndexOf('/');
    String dir = (slash > 0) ? path.substring(0, slash) : String("");
    String name = (slash >= 0) ? path.substring(slash + 1) : path;
    int dot = name.lastIndexOf('.');
    String base = (dot > 0) ? name.substring(0, dot) : name;
    String ext = (dot > 0) ? name.substring(dot) : String("");

    for (int n = 1; n < 1000; n++) {
        String candidate = dir + "/" + base + "_" + String(n) + ext;
        if (!SD.exists(candidate)) return candidate;
    }
    return path;
}

String recycleMetaPathFor(const String &recyclePath)
{
    return recyclePath + ".path";
}

String httpsGetText(const String &url, int *statusOut)
{
    if (statusOut) *statusOut = -1;
    if (!wifiConnectedSafe()) return "";

    WiFiClientSecure client;
    client.setInsecure();

    HTTPClient http;
    http.setConnectTimeout(6000);
    http.setTimeout(8000);
    if (!http.begin(client, url)) return "";

    http.addHeader("Accept", "application/vnd.github+json");
    http.addHeader("User-Agent", deviceShortNameValue());

    const int code = http.GET();
    if (statusOut) *statusOut = code;
    if (code <= 0) {
        http.end();
        return "";
    }

    String body;
    if (code >= 200 && code < 300) body = http.getString();
    http.end();
    return body;
}

String chooseLatestFirmwareBinUrl(const JsonVariantConst &assets)
{
    if (!assets.is<JsonArrayConst>()) return "";
    JsonArrayConst arr = assets.as<JsonArrayConst>();

    for (JsonObjectConst asset : arr) {
        const String name = asset["name"] | "";
        const String url = asset["browser_download_url"] | "";
        if (!url.isEmpty() && name.endsWith(".ino.bin")) return url;
    }
    for (JsonObjectConst asset : arr) {
        const String name = asset["name"] | "";
        const String url = asset["browser_download_url"] | "";
        if (url.isEmpty()) continue;
        const String low = String(name);
        if (low.endsWith(".bin") && low.indexOf("bootloader") < 0 && low.indexOf("partitions") < 0) return url;
    }
    return "";
}

bool otaDownloadAndApplyFromUrl(const String &url, String &errorOut)
{
    errorOut = "";
    if (!wifiConnectedSafe()) {
        errorOut = "no_wifi";
        return false;
    }
    if (!url.length()) {
        errorOut = "missing_url";
        return false;
    }

    WiFiClientSecure client;
    client.setInsecure();

    HTTPClient http;
    http.setConnectTimeout(7000);
    http.setTimeout(12000);
    if (!http.begin(client, url)) {
        errorOut = "http_begin_failed";
        return false;
    }

    const int code = http.GET();
    if (code < 200 || code >= 300) {
        errorOut = "http_" + String(code);
        http.end();
        return false;
    }

    const int totalLen = http.getSize();
    WiFiClient *stream = http.getStreamPtr();
    if (!stream) {
        errorOut = "no_stream";
        http.end();
        return false;
    }

    if (!Update.begin(totalLen > 0 ? static_cast<size_t>(totalLen) : UPDATE_SIZE_UNKNOWN)) {
        errorOut = "update_begin_failed";
        http.end();
        return false;
    }

    uint8_t buf[2048];
    int remaining = totalLen;
    while (http.connected() && (remaining > 0 || remaining == -1)) {
        const size_t avail = static_cast<size_t>(stream->available());
        if (!avail) {
            delay(1);
            continue;
        }
        const size_t toRead = avail > sizeof(buf) ? sizeof(buf) : avail;
        const int n = stream->readBytes(buf, toRead);
        if (n <= 0) {
            delay(1);
            continue;
        }
        if (Update.write(buf, static_cast<size_t>(n)) != static_cast<size_t>(n)) {
            errorOut = "update_write_failed";
            Update.abort();
            http.end();
            return false;
        }
        if (remaining > 0) remaining -= n;
        delay(0);
    }
    http.end();

    if (!Update.end(true)) {
        errorOut = "update_end_failed";
        return false;
    }
    return true;
}

String mimeFor(const String &path)
{
    if (path.endsWith(".html")) return "text/html";
    if (path.endsWith(".css")) return "text/css";
    if (path.endsWith(".js")) return "application/javascript";
    if (path.endsWith(".json")) return "application/json";
    if (path.endsWith(".png")) return "image/png";
    if (path.endsWith(".jpg") || path.endsWith(".jpeg")) return "image/jpeg";
    if (path.endsWith(".svg")) return "image/svg+xml";
    if (path.endsWith(".ico")) return "image/x-icon";
    if (path.endsWith(".txt")) return "text/plain";
    if (path.endsWith(".wav")) return "audio/wav";
    if (path.endsWith(".bin")) return "application/octet-stream";
    return "application/octet-stream";
}

void sendUploadFallbackPage(AsyncWebServerRequest *request)
{
    String html = FPSTR(UPLOAD_FALLBACK_HTML);
    html.replace("%FIRMWARE_VERSION%", FW_VERSION);
    request->send(200, "text/html", html);
}

void sendWifiSetupPage(AsyncWebServerRequest *request)
{
    String html = FPSTR(WIFI_SETUP_HTML);
    html.replace("%FIRMWARE_VERSION%", FW_VERSION);
    html.replace("%AP_SSID%", AP_SSID);
    html.replace("%AP_IP%", WiFi.softAPIP().toString());
    request->send(200, "text/html", html);
}

void sendRecoveryBrowserPage(AsyncWebServerRequest *request)
{
    AsyncWebServerResponse *res = request->beginResponse(
        200,
        "text/html",
        reinterpret_cast<const uint8_t *>(RECOVERY_BROWSER_HTML_GZ),
        RECOVERY_BROWSER_HTML_GZ_LEN
    );
    res->addHeader("Content-Encoding", "gzip");
    res->addHeader("Cache-Control", "public, max-age=300");
    request->send(res);
}

uint8_t wifiQualityPercentFromRssi(int32_t rssi)
{
    if (rssi <= -100) return 0;
    if (rssi >= -50) return 100;
    return static_cast<uint8_t>((rssi + 100) * 2);
}

uint32_t heapFragPercent(uint32_t freeBytes, uint32_t largestFreeBlock)
{
    if (freeBytes == 0 || largestFreeBlock >= freeBytes) return 0;
    return 100U - static_cast<uint32_t>((static_cast<uint64_t>(largestFreeBlock) * 100ULL) / freeBytes);
}

float readChipTemperatureC()
{
    const float tempC = temperatureRead();
    return isfinite(tempC) ? tempC : 0.0f;
}

void sendCarInputTelemetrySnapshot(AsyncWebSocketClient *client)
{
    sampleTopIndicators();

    if (ESP.getFreeHeap() < WS_TELEMETRY_MIN_FREE_HEAP) return;

    const bool staConnected = wifiConnectedSafe();
    const int wifiQuality = staConnected ? static_cast<int>(wifiQualityPercentFromRssi(wifiRssiSafe())) : 0;
    const unsigned long uptimeSecs = millis() / 1000UL;
    const float chipTempC = readChipTemperatureC();
    const uint32_t heapTotal = static_cast<uint32_t>(ESP.getHeapSize());
    const uint32_t heapFree = static_cast<uint32_t>(ESP.getFreeHeap());
    const uint32_t psramTotal = static_cast<uint32_t>(heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
    const uint32_t psramFree = static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    char msg[96];
    auto sendText = [&](const char *text) {
        if (client) client->text(text);
        else wsCarInput.textAll(text);
    };

    snprintf(msg, sizeof(msg), "BATT,%u,%.2f,%d", batteryPercent, batteryVoltage, wifiQuality);
    sendText(msg);

    snprintf(msg, sizeof(msg), "AIRPLANE,%s", airplaneModeEnabled ? "ON" : "OFF");
    sendText(msg);

    snprintf(msg, sizeof(msg), "CHARGE,%s", batteryCharging ? "YES" : "NO");
    sendText(msg);

    snprintf(msg, sizeof(msg), "STATS,%lu,%.1f", uptimeSecs, chipTempC);
    sendText(msg);

    snprintf(msg, sizeof(msg), "HEAP,%lu,%lu",
             static_cast<unsigned long>(heapTotal),
             static_cast<unsigned long>(heapFree));
    sendText(msg);

    snprintf(msg, sizeof(msg), "PSRAM,%lu,%lu",
             static_cast<unsigned long>(psramTotal),
             static_cast<unsigned long>(psramFree));
    sendText(msg);
}

void handleCarInputSocketEvent(AsyncWebSocket *serverRef,
                               AsyncWebSocketClient *client,
                               AwsEventType type,
                               void *arg,
                               uint8_t *data,
                               size_t len)
{
    (void)serverRef;
    if (!client) return;

    if (type == WS_EVT_CONNECT) {
        sendCarInputTelemetrySnapshot(client);
        return;
    }

    if (type != WS_EVT_DATA || !arg || !data || len == 0) return;

    AwsFrameInfo *info = reinterpret_cast<AwsFrameInfo *>(arg);
    if (!info || info->opcode != WS_TEXT || !info->final || info->index != 0 || info->len != len) return;

    char msg[80];
    const size_t copyLen = min(len, sizeof(msg) - 1U);
    memcpy(msg, data, copyLen);
    msg[copyLen] = '\0';

    if (strcmp(msg, "PING") == 0 || strcmp(msg, "ping") == 0) {
        client->text("PONG");
        return;
    }

    if (handleUiSettingMessage(msg)) {
        return;
    }
}

void setupCarInputSocket()
{
    wsCarInput.onEvent(handleCarInputSocketEvent);
    server.addHandler(&wsCarInput);
}

void serviceCarInputTelemetry()
{
    if (!webServerRunning) return;
    if (static_cast<unsigned long>(millis() - lastCarInputTelemetryMs) < CAR_INPUT_TELEMETRY_PERIOD_MS) return;
    lastCarInputTelemetryMs = millis();
    wsCarInput.cleanupClients();
    sendCarInputTelemetrySnapshot(nullptr);
}

bool captureScreenToJpeg(String &savedPathOut, String &errorOut)
{
    savedPathOut = "";
    errorOut = "";
    if (!sdEnsureMounted()) { errorOut = "no_sd"; return false; }
    if (mediaIsPlaying || mediaPaused) { errorOut = "media_busy"; return false; }
    if (!sdLock()) { errorOut = "sd_lock"; return false; }

    if (!SD.exists("/Screenshots") && !SD.mkdir("/Screenshots")) {
        sdUnlock();
        errorOut = "mkdir_failed";
        return false;
    }

    const time_t now = time(nullptr);
    const bool timeValid = now > 1700000000;
    struct tm tmNow = {};
    if (timeValid) localtime_r(&now, &tmNow);
    else {
        unsigned long sec = millis() / 1000UL;
        tmNow.tm_year = 70;
        tmNow.tm_mon = 0;
        tmNow.tm_mday = 1;
        tmNow.tm_hour = static_cast<int>((sec / 3600UL) % 24UL);
        tmNow.tm_min = static_cast<int>((sec / 60UL) % 60UL);
        tmNow.tm_sec = static_cast<int>(sec % 60UL);
    }

    char name[80];
    snprintf(
        name,
        sizeof(name),
        "/Screenshots/screenshot_%04d%02d%02d_%02d%02d%02d.jpg",
        tmNow.tm_year + 1900,
        tmNow.tm_mon + 1,
        tmNow.tm_mday,
        tmNow.tm_hour,
        tmNow.tm_min,
        tmNow.tm_sec
    );
    String path(name);
    if (SD.exists(path)) {
        for (int i = 1; i <= 999; i++) {
            snprintf(
                name,
                sizeof(name),
                "/Screenshots/screenshot_%04d%02d%02d_%02d%02d%02d_%03d.jpg",
                tmNow.tm_year + 1900,
                tmNow.tm_mon + 1,
                tmNow.tm_mday,
                tmNow.tm_hour,
                tmNow.tm_min,
                tmNow.tm_sec,
                i
            );
            if (!SD.exists(name)) {
                path = String(name);
                break;
            }
        }
    }
    if (path.isEmpty()) {
        sdUnlock();
        errorOut = "name_exhausted";
        return false;
    }

    auto jpegOpen = [](const char *filename) -> void * {
        (void)filename;
        screenshotJpegFile = SD.open(screenshotJpegPath, FILE_WRITE);
        if (!screenshotJpegFile) return nullptr;
        return static_cast<void *>(&screenshotJpegFile);
    };
    auto jpegClose = [](JPEGE_FILE *pFile) {
        File *f = reinterpret_cast<File *>(pFile ? pFile->fHandle : nullptr);
        if (!f || !(*f)) return;
        f->flush();
        f->close();
    };
    auto jpegRead = [](JPEGE_FILE *pFile, uint8_t *pBuf, int32_t len) -> int32_t {
        File *f = reinterpret_cast<File *>(pFile ? pFile->fHandle : nullptr);
        if (!f || !(*f) || !pBuf || len <= 0) return 0;
        return static_cast<int32_t>(f->read(pBuf, static_cast<size_t>(len)));
    };
    auto jpegWrite = [](JPEGE_FILE *pFile, uint8_t *pBuf, int32_t len) -> int32_t {
        File *f = reinterpret_cast<File *>(pFile ? pFile->fHandle : nullptr);
        if (!f || !(*f) || !pBuf || len <= 0) return 0;
        return static_cast<int32_t>(f->write(pBuf, static_cast<size_t>(len)));
    };
    auto jpegSeek = [](JPEGE_FILE *pFile, int32_t pos) -> int32_t {
        File *f = reinterpret_cast<File *>(pFile ? pFile->fHandle : nullptr);
        if (!f || !(*f) || pos < 0) return -1;
        return f->seek(static_cast<uint32_t>(pos)) ? static_cast<int32_t>(f->position()) : -1;
    };

    screenshotJpegPath = path;
    JPEGENC encoder;
    JPEGENCODE state = {};
    int rc = encoder.open(path.c_str(), jpegOpen, jpegClose, jpegRead, jpegWrite, jpegSeek);
    if (rc != JPEGE_SUCCESS) {
        sdMarkFault("captureScreenToJpeg/open");
        sdUnlock();
        errorOut = "jpeg_open";
        return false;
    }

    const int w = tft.width();
    const int h = tft.height();
    rc = encoder.encodeBegin(&state, w, h, JPEGE_PIXEL_RGB565, JPEGE_SUBSAMPLE_420, JPEGE_Q_HIGH);
    if (rc != JPEGE_SUCCESS || state.cx <= 0 || state.cy <= 0) {
        encoder.close();
        if (SD.exists(path)) SD.remove(path);
        sdUnlock();
        errorOut = "jpeg_begin";
        return false;
    }

    const int mcuW = state.cx;
    const int mcuH = state.cy;
    const size_t mcuBytes = static_cast<size_t>(mcuW) * static_cast<size_t>(mcuH) * 2U;
    uint8_t *mcuBuf = reinterpret_cast<uint8_t *>(malloc(mcuBytes));
    if (!mcuBuf) {
        encoder.close();
        if (SD.exists(path)) SD.remove(path);
        sdUnlock();
        errorOut = "oom_mcu";
        return false;
    }

    bool ok = true;
    const unsigned long startedMs = millis();
    for (int y = 0; y < h && ok; y += mcuH) {
        for (int x = 0; x < w && ok; x += mcuW) {
            const int validW = min(mcuW, w - x);
            const int validH = min(mcuH, h - y);
            for (int ry = 0; ry < validH; ry++) {
                uint16_t *dst = reinterpret_cast<uint16_t *>(mcuBuf + static_cast<size_t>(ry) * static_cast<size_t>(mcuW) * 2U);
                tft.readRect(x, y + ry, validW, 1, dst);
                if (validW < mcuW) {
                    const uint16_t edge = dst[validW - 1];
                    for (int xx = validW; xx < mcuW; xx++) dst[xx] = edge;
                }
            }
            if (validH < mcuH) {
                uint16_t *lastRow = reinterpret_cast<uint16_t *>(mcuBuf + static_cast<size_t>(validH - 1) * static_cast<size_t>(mcuW) * 2U);
                for (int ry = validH; ry < mcuH; ry++) {
                    uint16_t *dst = reinterpret_cast<uint16_t *>(mcuBuf + static_cast<size_t>(ry) * static_cast<size_t>(mcuW) * 2U);
                    memcpy(dst, lastRow, static_cast<size_t>(mcuW) * sizeof(uint16_t));
                }
            }
            rc = encoder.addMCU(&state, mcuBuf, mcuW * 2);
            if (rc != JPEGE_SUCCESS) {
                ok = false;
                errorOut = "jpeg_add_mcu";
                break;
            }
            if ((millis() - startedMs) > 6000UL) {
                ok = false;
                errorOut = "timeout";
                break;
            }
        }
        delay(0);
    }

    free(mcuBuf);
    const int outSize = encoder.close();
    screenshotJpegPath = "";
    sdUnlock();

    if (!ok || outSize < 128 || !SD.exists(path)) {
        sdMarkFault("captureScreenToJpeg/write");
        if (SD.exists(path)) SD.remove(path);
        if (errorOut.isEmpty()) errorOut = "encode_failed";
        return false;
    }

    File verify = SD.open(path, FILE_READ);
    const uint32_t sizeOnDisk = verify ? static_cast<uint32_t>(verify.size()) : 0U;
    if (verify) verify.close();
    if (sizeOnDisk < 128U) {
        SD.remove(path);
        errorOut = "verify_failed";
        return false;
    }

    savedPathOut = path;
    return true;
}

bool sendMaybeGz(AsyncWebServerRequest *request, const String &path)
{
    if (!sdEnsureMounted()) return false;
    String gzPath = path + ".gz";
    if (SD.exists(gzPath)) {
        AsyncWebServerResponse *res = request->beginResponse(SD, gzPath, mimeFor(path), false);
        res->addHeader("Content-Encoding", "gzip");
        request->send(res);
        return true;
    }
    if (SD.exists(path)) {
        request->send(SD, path, mimeFor(path), false);
        return true;
    }
    return false;
}

bool sdHasFileOrGz(const String &path)
{
    if (!sdEnsureMounted()) return false;
    return SD.exists(path) || SD.exists(path + ".gz");
}

bool hasPrimaryWebUi()
{
    // Accept fully gzipped or plain web bundles; only an index is required as entrypoint.
    return sdHasFileOrGz("/web/index.html") || sdHasFileOrGz("/web/index.htm");
}

String webIndexPath()
{
    if (sdHasFileOrGz("/web/index.html")) return "/web/index.html";
    if (sdHasFileOrGz("/web/index.htm")) return "/web/index.htm";
    return "";
}

void sendFileFromWebOr404(AsyncWebServerRequest *request, const String &urlPath)
{
    String path = urlPath;
    if (!path.startsWith("/web/") && path != "/web") path = "/web" + path;
    if (sendMaybeGz(request, path)) return;
    request->send(404, "text/plain", "Not found");
}

void setupWebRoutes()
{
    setupCarInputSocket();

    server.on("/version", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        doc["current"] = FW_VERSION;
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/set_ws_reboot_watchdog", HTTP_GET, [](AsyncWebServerRequest *request) {
        const bool enabled = request->hasParam("value") && request->getParam("value")->value().toInt() != 0;
        wsRebootOnDisconnectEnabled = enabled;
        uiPrefs.begin("ui", false);
        uiPrefs.putBool("ws_reboot", enabled);
        uiPrefs.end();
        JsonDocument doc;
        doc["ok"] = true;
        doc["value"] = enabled ? 1 : 0;
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/serial/logs", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        JsonArray lines = doc["lines"].to<JsonArray>();
        size_t count = 0;
        if (SERIAL_TERMINAL_TRANSFER_ENABLED) {
            markSerialTerminalStreamingActive();
            const size_t cap = serialLogActiveCapacity();
            count = min(serialLogCount, cap);
            const size_t start = (serialLogHead + cap - count) % cap;
            for (size_t i = 0; i < count; ++i) {
                const size_t idx = (start + i) % cap;
                if (serialLogRing) lines.add(serialLogSlot(idx));
            }
        }
        doc["count"] = static_cast<uint32_t>(count);
        doc["ok"] = true;
        doc["enabled"] = SERIAL_TERMINAL_TRANSFER_ENABLED ? 1 : 0;
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/serial/command", HTTP_ANY, [](AsyncWebServerRequest *request) {
        String cmd;
        if (request->hasParam("cmd", true)) cmd = request->getParam("cmd", true)->value();
        else if (request->hasParam("cmd")) cmd = request->getParam("cmd")->value();

        JsonDocument doc;
        if (!cmd.length()) {
            doc["ok"] = false;
            doc["error"] = "missing_cmd";
            String payload;
            serializeJson(doc, payload);
            request->send(400, "application/json", payload);
            return;
        }

        if (SERIAL_TERMINAL_TRANSFER_ENABLED) {
            markSerialTerminalStreamingActive();
            serialLogPushLine((String("> ") + cmd).c_str(), true);
            executeSerialCommand(cmd);
        }
        doc["ok"] = true;
        doc["enabled"] = SERIAL_TERMINAL_TRANSFER_ENABLED ? 1 : 0;
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/ota/upload", HTTP_POST,
              [](AsyncWebServerRequest *request) {
                  OtaUploadCtx *ctx = reinterpret_cast<OtaUploadCtx *>(request->_tempObject);
                  int code = 200;
                  String body = "OK";
                  if (!ctx) {
                      code = 500;
                      body = "OTA context missing";
                  } else if (ctx->error.length()) {
                      code = 500;
                      body = ctx->error;
                  } else if (!ctx->finished) {
                      code = 500;
                      body = "OTA incomplete";
                  } else {
                      body = "Update complete. Rebooting...";
                      rebootRequested = true;
                      rebootRequestedAtMs = millis();
                  }
                  request->send(code, "text/plain", body);
                  if (ctx) {
                      delete ctx;
                      request->_tempObject = nullptr;
                  }
              },
              [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
                  (void)filename;
                  OtaUploadCtx *ctx = reinterpret_cast<OtaUploadCtx *>(request->_tempObject);
                  if (!ctx) {
                      ctx = new OtaUploadCtx();
                      request->_tempObject = ctx;
                  }

                  if (index == 0) {
                      if (len < 1 || data[0] != 0xE9) {
                          ctx->error = "Invalid firmware format";
                          return;
                      }
                      ctx->valid = true;
                      ctx->started = true;
                      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
                          ctx->error = "OTA begin failed";
                          return;
                      }
                  }

                  if (!ctx->valid || ctx->error.length()) return;

                  if (len && Update.write(data, len) != len) {
                      ctx->error = "OTA write failed";
                      Update.abort();
                      return;
                  }
                  ctx->bytesWritten += len;

                  if (final) {
                      if (Update.end(true)) {
                          ctx->finished = true;
                      } else {
                          ctx->error = "OTA end failed";
                      }
                  }
              });

    server.on("/ota/latest", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        doc["ok"] = false;
        doc["repo"] = "elik745i/ESP32-2432S024C-Remote";

        int ghStatus = -1;
        const String body = httpsGetText("https://api.github.com/repos/elik745i/ESP32-2432S024C-Remote/releases/latest", &ghStatus);
        if (body.isEmpty()) {
            doc["error"] = "github_fetch_failed";
            doc["http_status"] = ghStatus;
            String payload;
            serializeJson(doc, payload);
            request->send(502, "application/json", payload);
            return;
        }

        JsonDocument filter;
        filter["tag_name"] = true;
        filter["assets"][0]["name"] = true;
        filter["assets"][0]["browser_download_url"] = true;

        JsonDocument parsed;
        const DeserializationError err =
            deserializeJson(parsed, body, DeserializationOption::Filter(filter));
        if (err) {
            doc["error"] = "json_parse_failed";
            doc["detail"] = err.c_str();
            doc["http_status"] = ghStatus;
            String payload;
            serializeJson(doc, payload);
            request->send(500, "application/json", payload);
            return;
        }

        const String tagName = parsed["tag_name"] | "";
        const String binUrl = chooseLatestFirmwareBinUrl(parsed["assets"]);
        if (tagName.isEmpty() || binUrl.isEmpty()) {
            doc["error"] = "release_asset_not_found";
            doc["http_status"] = ghStatus;
            String payload;
            serializeJson(doc, payload);
            request->send(500, "application/json", payload);
            return;
        }

        doc["ok"] = true;
        doc["tag_name"] = tagName;
        doc["bin_url"] = binUrl;
        doc["source"] = "device";
        doc["http_status"] = ghStatus;
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/ota/update_from_url", HTTP_ANY, [](AsyncWebServerRequest *request) {
        String url;
        if (request->hasParam("url", true)) url = request->getParam("url", true)->value();
        else if (request->hasParam("url", false)) url = request->getParam("url", false)->value();

        JsonDocument doc;
        if (url.isEmpty()) {
            doc["ok"] = false;
            doc["error"] = "missing_url";
            String payload;
            serializeJson(doc, payload);
            request->send(400, "application/json", payload);
            return;
        }

        String otaErr;
        const bool ok = otaDownloadAndApplyFromUrl(url, otaErr);
        doc["ok"] = ok;
        if (!ok) {
            doc["error"] = otaErr;
            String payload;
            serializeJson(doc, payload);
            request->send(500, "application/json", payload);
            return;
        }

        doc["rebooting"] = true;
        rebootRequested = true;
        rebootRequestedAtMs = millis();
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    // Captive portal helper routes.
    server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest *request) { request->redirect("/"); });
    server.on("/fwlink", HTTP_GET, [](AsyncWebServerRequest *request) { request->redirect("/"); });
    server.on("/hotspot-detect.html", HTTP_GET, [](AsyncWebServerRequest *request) { request->redirect("/"); });
    server.on("/redirect", HTTP_GET, [](AsyncWebServerRequest *request) { request->redirect("/"); });
    server.on("/connecttest.txt", HTTP_GET, [](AsyncWebServerRequest *request) { request->send(200, "text/plain", "OK"); });
    server.on("/ncsi.txt", HTTP_GET, [](AsyncWebServerRequest *request) { request->send(200, "text/plain", "Microsoft NCSI"); });

    server.on("/wifi", HTTP_GET, [](AsyncWebServerRequest *request) {
        sendWifiSetupPage(request);
    });

    server.on("/api/wifi/scan", HTTP_GET, [](AsyncWebServerRequest *request) {
        (void)request;
        JsonDocument doc;
        JsonArray arr = doc["networks"].to<JsonArray>();
        WiFi.scanDelete();
        int n = WiFi.scanNetworks(false, true);
        for (int i = 0; i < n; i++) {
            JsonObject it = arr.add<JsonObject>();
            it["ssid"] = WiFi.SSID(i);
            it["rssi"] = WiFi.RSSI(i);
            it["auth"] = authName(WiFi.encryptionType(i));
        }
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/api/wifi/connect", HTTP_POST, [](AsyncWebServerRequest *request) {
        String ssid;
        String pass;
        if (request->hasParam("ssid", true)) ssid = urlDecode(request->getParam("ssid", true)->value());
        else if (request->hasParam("ssid", false)) ssid = urlDecode(request->getParam("ssid", false)->value());
        if (request->hasParam("pass", true)) pass = urlDecode(request->getParam("pass", true)->value());
        else if (request->hasParam("pass", false)) pass = urlDecode(request->getParam("pass", false)->value());
        if (ssid.isEmpty()) {
            request->send(400, "text/plain", "missing_ssid");
            return;
        }
        startWifiConnect(ssid, pass);
        request->send(200, "text/plain", "connecting");
    });

    server.on("/api/telemetry", HTTP_GET, [](AsyncWebServerRequest *request) {
        sampleTopIndicators();
        JsonDocument doc;
        doc["battery_percent"] = batteryPercent;
        doc["battery_voltage"] = String(batteryVoltage, 2);
        doc["battery_charging"] = batteryCharging;
        doc["light_percent"] = lightPercent;
        bool connected = (WiFi.status() == WL_CONNECTED);
        doc["wifi_connected"] = connected;
        doc["wifi_ssid"] = connected ? WiFi.SSID() : "";
        doc["wifi_rssi"] = connected ? WiFi.RSSI() : 0;
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/api/chat/messages", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        JsonArray arr = doc["messages"].to<JsonArray>();
        for (int i = 0; i < chatMessageCount; ++i) {
            JsonObject msg = arr.add<JsonObject>();
            msg["author"] = chatDisplayAuthorForMessage(chatMessages[i]);
            msg["text"] = chatMessages[i].text;
            msg["outgoing"] = chatMessages[i].outgoing;
            msg["ts_ms"] = static_cast<uint64_t>(chatMessages[i].tsMs);
        }
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/api/chat/identity", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        doc["device"] = deviceShortNameValue();
        doc["port"] = P2P_UDP_PORT;
        doc["public_key"] = p2pPublicKeyHex();
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/api/chat/peers", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        JsonArray arr = doc["peers"].to<JsonArray>();
        for (int i = 0; i < p2pPeerCount; ++i) {
            JsonObject peer = arr.add<JsonObject>();
            peer["name"] = p2pPeers[i].name;
            peer["public_key"] = p2pPeers[i].pubKeyHex;
            peer["ip"] = p2pPeers[i].ip.toString();
            peer["port"] = p2pPeers[i].port;
            peer["enabled"] = p2pPeers[i].enabled;
            peer["last_seen_ms"] = static_cast<uint64_t>(p2pPeers[i].lastSeenMs);
        }
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/api/chat/discovery", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        JsonArray arr = doc["discovered"].to<JsonArray>();
        for (int i = 0; i < p2pDiscoveredCount; ++i) {
            JsonObject peer = arr.add<JsonObject>();
            peer["name"] = p2pDiscoveredPeers[i].name;
            peer["public_key"] = p2pDiscoveredPeers[i].pubKeyHex;
            peer["ip"] = p2pDiscoveredPeers[i].ip.toString();
            peer["port"] = p2pDiscoveredPeers[i].port;
            peer["trusted"] = p2pDiscoveredPeers[i].trusted;
            peer["last_seen_ms"] = static_cast<uint64_t>(p2pDiscoveredPeers[i].lastSeenMs);
        }
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/api/chat/discovery/pair", HTTP_POST, [](AsyncWebServerRequest *request) {
        String pubKey;
        if (request->hasParam("public_key", true)) pubKey = urlDecode(request->getParam("public_key", true)->value());
        else if (request->hasParam("public_key", false)) pubKey = urlDecode(request->getParam("public_key", false)->value());
        pubKey.trim();

        JsonDocument doc;
        const int idx = p2pFindDiscoveredByPubKeyHex(pubKey);
        if (idx < 0) {
            doc["ok"] = false;
            doc["error"] = "peer_not_discovered";
            String payload;
            serializeJson(doc, payload);
            request->send(404, "application/json", payload);
            return;
        }

        const bool ok = p2pAddOrUpdateTrustedPeer(
            p2pDiscoveredPeers[idx].name.isEmpty() ? String("Peer") : p2pDiscoveredPeers[idx].name,
            p2pDiscoveredPeers[idx].pubKeyHex,
            p2pDiscoveredPeers[idx].ip,
            p2pDiscoveredPeers[idx].port);
        p2pDiscoveredPeers[idx].trusted = ok;
        doc["ok"] = ok;
        if (!ok) doc["error"] = "pair_failed";
        String payload;
        serializeJson(doc, payload);
        request->send(ok ? 200 : 409, "application/json", payload);
    });

    server.on("/api/chat/peers/add", HTTP_POST, [](AsyncWebServerRequest *request) {
        String name;
        String pubKey;
        String ipText;
        uint16_t port = P2P_UDP_PORT;
        if (request->hasParam("name", true)) name = urlDecode(request->getParam("name", true)->value());
        else if (request->hasParam("name", false)) name = urlDecode(request->getParam("name", false)->value());
        if (request->hasParam("public_key", true)) pubKey = urlDecode(request->getParam("public_key", true)->value());
        else if (request->hasParam("public_key", false)) pubKey = urlDecode(request->getParam("public_key", false)->value());
        if (request->hasParam("ip", true)) ipText = urlDecode(request->getParam("ip", true)->value());
        else if (request->hasParam("ip", false)) ipText = urlDecode(request->getParam("ip", false)->value());
        if (request->hasParam("port", true)) port = static_cast<uint16_t>(request->getParam("port", true)->value().toInt());
        else if (request->hasParam("port", false)) port = static_cast<uint16_t>(request->getParam("port", false)->value().toInt());
        name.trim();
        pubKey.trim();

        JsonDocument doc;
        unsigned char testPk[P2P_PUBLIC_KEY_BYTES] = {0};
        IPAddress ip;
        if (name.isEmpty() || !p2pHexToBytes(pubKey, testPk, sizeof(testPk)) || !ip.fromString(ipText)) {
            doc["ok"] = false;
            doc["error"] = "invalid_peer";
            String payload;
            serializeJson(doc, payload);
            request->send(400, "application/json", payload);
            return;
        }

        const bool ok = p2pAddOrUpdateTrustedPeer(name, pubKey, ip, port);
        doc["ok"] = ok;
        if (!ok) doc["error"] = "peer_list_full";
        String payload;
        serializeJson(doc, payload);
        request->send(ok ? 200 : 409, "application/json", payload);
    });

    server.on("/api/chat/peers/toggle", HTTP_POST, [](AsyncWebServerRequest *request) {
        String pubKey;
        bool enabled = true;
        if (request->hasParam("public_key", true)) pubKey = urlDecode(request->getParam("public_key", true)->value());
        else if (request->hasParam("public_key", false)) pubKey = urlDecode(request->getParam("public_key", false)->value());
        if (request->hasParam("enabled", true)) enabled = request->getParam("enabled", true)->value().toInt() != 0;
        else if (request->hasParam("enabled", false)) enabled = request->getParam("enabled", false)->value().toInt() != 0;
        pubKey.trim();

        JsonDocument doc;
        const int idx = p2pFindPeerByPubKeyHex(pubKey);
        if (idx < 0) {
            doc["ok"] = false;
            doc["error"] = "peer_not_found";
            String payload;
            serializeJson(doc, payload);
            request->send(404, "application/json", payload);
            return;
        }
        p2pPeers[idx].enabled = enabled;
        saveP2pConfig();
        const int didx = p2pFindDiscoveredByPubKeyHex(pubKey);
        if (didx >= 0) p2pDiscoveredPeers[didx].trusted = enabled;
        doc["ok"] = true;
        doc["enabled"] = enabled;
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/api/chat/peers/remove", HTTP_POST, [](AsyncWebServerRequest *request) {
        String pubKey;
        if (request->hasParam("public_key", true)) pubKey = urlDecode(request->getParam("public_key", true)->value());
        else if (request->hasParam("public_key", false)) pubKey = urlDecode(request->getParam("public_key", false)->value());
        pubKey.trim();

        JsonDocument doc;
        const int idx = p2pFindPeerByPubKeyHex(pubKey);
        if (idx < 0) {
            doc["ok"] = false;
            doc["error"] = "peer_not_found";
            String payload;
            serializeJson(doc, payload);
            request->send(404, "application/json", payload);
            return;
        }
        for (int i = idx + 1; i < p2pPeerCount; ++i) p2pPeers[i - 1] = p2pPeers[i];
        p2pPeerCount--;
        saveP2pConfig();
        const int didx = p2pFindDiscoveredByPubKeyHex(pubKey);
        if (didx >= 0) p2pDiscoveredPeers[didx].trusted = false;
        doc["ok"] = true;
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/api/chat/send", HTTP_POST, [](AsyncWebServerRequest *request) {
        String text;
        if (request->hasParam("text", true)) text = urlDecode(request->getParam("text", true)->value());
        else if (request->hasParam("text", false)) text = urlDecode(request->getParam("text", false)->value());
        text.trim();

        JsonDocument doc;
        if (text.isEmpty()) {
            doc["ok"] = false;
            doc["error"] = "missing_text";
            String payload;
            serializeJson(doc, payload);
            request->send(400, "application/json", payload);
            return;
        }

        if (currentChatPeerKey.isEmpty()) {
            doc["ok"] = false;
            doc["error"] = "missing_peer";
            String payload;
            serializeJson(doc, payload);
            request->send(400, "application/json", payload);
            return;
        }

        const String messageId = chatGenerateMessageId();
        chatQueueOutgoingMessage(currentChatPeerKey, deviceShortNameValue(), text, messageId);
        const bool sentLan = p2pSendChatMessageWithId(currentChatPeerKey, text, messageId);
        const bool sentGlobal = mqttPublishChatMessageWithId(currentChatPeerKey, text, messageId);
        chatStoreMessage(currentChatPeerKey, "Me", text, true, sentGlobal ? CHAT_TRANSPORT_MQTT : CHAT_TRANSPORT_WIFI, messageId);
        if (lvglReady) lvglRefreshChatUi();
        uiStatusLine = sentGlobal ? "Chat sent globally" : (sentLan ? "Chat sent to peers" : "Chat saved locally");
        if (lvglReady) lvglSyncStatusLine();

        doc["ok"] = true;
        doc["transport"] = sentGlobal ? "mqtt_global" : (sentLan ? "p2p_udp_encrypted" : "local_only");
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/list_sd_files", HTTP_GET, [](AsyncWebServerRequest *request) {
        String path = "/";
        if (request->hasParam("path")) path = urlDecode(request->getParam("path")->value());
        path = normalizeSdRoutePath(path, true);
        const bool showSystem = request->hasParam("showSystem") && request->getParam("showSystem")->value().toInt() != 0;
        int start = request->hasParam("start") ? request->getParam("start")->value().toInt() : 0;
        int count = request->hasParam("count") ? request->getParam("count")->value().toInt() : 20;
        if (start < 0) start = 0;
        if (count < 1) count = 20;
        if (count > 128) count = 128;

        JsonDocument doc;
        JsonArray arr = doc.to<JsonArray>();

        if (!sdEnsureMounted() || !sdLock()) {
            String payload;
            serializeJson(arr, payload);
            request->send(200, "application/json", payload);
            return;
        }

        if (!SD.exists(path)) {
            sdUnlock();
            String payload;
            serializeJson(arr, payload);
            request->send(200, "application/json", payload);
            return;
        }

        File dir = SD.open(path);
        if (!dir || !dir.isDirectory()) {
            if (dir) dir.close();
            sdUnlock();
            String payload;
            serializeJson(arr, payload);
            request->send(200, "application/json", payload);
            return;
        }

        int visibleIndex = 0;
        File entry = dir.openNextFile();
        while (entry) {
            String child = entry.name();
            bool isDir = entry.isDirectory();
            uint32_t size = isDir ? 0U : static_cast<uint32_t>(entry.size());
            if (!child.startsWith("/")) child = (path == "/") ? ("/" + child) : (path + "/" + child);
            String name = child;
            int slash = name.lastIndexOf('/');
            if (slash >= 0) name = name.substring(slash + 1);

            const bool hidden = !showSystem && sdShouldHideSystemEntry(name);
            if (!hidden) {
                if (visibleIndex >= start && arr.size() < static_cast<size_t>(count)) {
                    JsonObject obj = arr.add<JsonObject>();
                    obj["name"] = name;
                    obj["isFolder"] = isDir;
                    obj["size"] = size;
                    if (isDir) obj["type"] = "folder";
                    else if (name.endsWith(".js")) obj["type"] = "js";
                    else if (name.endsWith(".html") || name.endsWith(".htm")) obj["type"] = "html";
                    else if (name.endsWith(".txt")) obj["type"] = "txt";
                    else obj["type"] = "default";
                    time_t lastWrite = entry.getLastWrite();
                    if (lastWrite > 0) obj["date"] = static_cast<uint64_t>(lastWrite) * 1000ULL;
                }
                visibleIndex++;
                if (arr.size() >= static_cast<size_t>(count)) {
                    entry.close();
                    break;
                }
            }

            entry.close();
            entry = dir.openNextFile();
        }
        dir.close();
        sdUnlock();

        String payload;
        serializeJson(arr, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/sd_reindex", HTTP_POST, [](AsyncWebServerRequest *request) {
        String path = "/";
        if (request->hasParam("path", true)) path = urlDecode(request->getParam("path", true)->value());
        else if (request->hasParam("path", false)) path = urlDecode(request->getParam("path", false)->value());
        path = normalizeSdRoutePath(path, true);

        JsonDocument doc;
        doc["ok"] = true;
        doc["status"] = "started";
        doc["path"] = path;

        String payload;
        serializeJson(doc, payload);
        request->send(202, "application/json", payload);
    });

    server.on("/sd_reindex_status", HTTP_GET, [](AsyncWebServerRequest *request) {
        (void)request;
        JsonDocument doc;
        doc["pending"] = false;
        doc["path"] = "";
        doc["count"] = 0;
        doc["total"] = 0;
        doc["counting"] = false;

        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/reboot", HTTP_POST, [](AsyncWebServerRequest *request) {
        rebootRequested = true;
        rebootRequestedAtMs = millis();
        request->send(200, "application/json", "{\"ok\":true,\"status\":\"restarting\"}");
    });

    server.on("/download_sd", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!request->hasParam("path")) {
            request->send(400, "application/json", "{\"ok\":false,\"error\":\"missing_path\"}");
            return;
        }
        String path = normalizeSdRoutePath(urlDecode(request->getParam("path")->value()), false);
        if (!path.length() || !pathAllowedInRecovery(path)) {
            request->send(400, "application/json", "{\"ok\":false,\"error\":\"path_not_allowed\"}");
            return;
        }
        if (!sdEnsureMounted()) {
            request->send(503, "application/json", "{\"ok\":false,\"error\":\"no_sd\"}");
            return;
        }
        if (!SD.exists(path)) {
            request->send(404, "application/json", "{\"ok\":false,\"error\":\"not_found\"}");
            return;
        }
        request->send(SD, path, mimeFor(path), true);
    });

    server.on("/delete_sd", HTTP_POST, [](AsyncWebServerRequest *request) {
        String path;
        if (request->hasParam("path", true)) path = urlDecode(request->getParam("path", true)->value());
        else if (request->hasParam("path", false)) path = urlDecode(request->getParam("path", false)->value());
        path = normalizeSdRoutePath(path, false);

        JsonDocument doc;
        int code = 200;

        if (!path.length()) {
            doc["ok"] = false;
            doc["error"] = "missing_path";
            code = 400;
        } else if (!pathAllowedInRecovery(path) || path == "/recycle") {
            doc["ok"] = false;
            doc["error"] = "path_not_allowed";
            code = 400;
        } else if (mediaIsPlaying || mediaPaused) {
            doc["ok"] = false;
            doc["error"] = "media_busy";
            code = 409;
        } else if (fsWriteBusy()) {
            doc["ok"] = false;
            doc["error"] = "fs_busy";
            code = 409;
        } else if (!sdEnsureMounted()) {
            doc["ok"] = false;
            doc["error"] = "no_sd";
            code = 503;
        } else {
            fsWriteBegin();
            if (!sdLock()) {
                doc["ok"] = false;
                doc["error"] = "sd_lock_failed";
                code = 500;
            } else if (!SD.exists(path)) {
                doc["ok"] = false;
                doc["error"] = "not_found";
                code = 404;
                sdUnlock();
            } else {
                if (!SD.exists("/recycle")) SD.mkdir("/recycle");
                String filename = path.substring(path.lastIndexOf('/') + 1);
                String recyclePath = makeUniquePath("/recycle/" + filename);
                String metaPath = recycleMetaPathFor(recyclePath);
                File meta = SD.open(metaPath, FILE_WRITE);
                if (meta) {
                    meta.print(path);
                    meta.close();
                }
                const bool ok = SD.rename(path, recyclePath);
                if (!ok && SD.exists(metaPath)) SD.remove(metaPath);
                sdUnlock();

                doc["ok"] = ok;
                doc["path"] = path;
                if (ok) doc["recyclePath"] = recyclePath;
                else {
                    doc["error"] = "move_failed";
                    code = 500;
                }
            }
            fsWriteEnd();
        }

        String payload;
        serializeJson(doc, payload);
        request->send(code, "application/json", payload);
    });

    server.on("/permadelete_sd", HTTP_POST, [](AsyncWebServerRequest *request) {
        String path;
        if (request->hasParam("path", true)) path = urlDecode(request->getParam("path", true)->value());
        else if (request->hasParam("path", false)) path = urlDecode(request->getParam("path", false)->value());
        path = normalizeSdRoutePath(path, false);

        JsonDocument doc;
        int code = 200;

        if (!path.length()) {
            doc["ok"] = false;
            doc["error"] = "missing_path";
            code = 400;
        } else if (!pathAllowedInRecovery(path)) {
            doc["ok"] = false;
            doc["error"] = "path_not_allowed";
            code = 400;
        } else if (mediaIsPlaying || mediaPaused) {
            doc["ok"] = false;
            doc["error"] = "media_busy";
            code = 409;
        } else if (fsWriteBusy()) {
            doc["ok"] = false;
            doc["error"] = "fs_busy";
            code = 409;
        } else if (!sdEnsureMounted()) {
            doc["ok"] = false;
            doc["error"] = "no_sd";
            code = 503;
        } else {
            fsWriteBegin();
            if (!sdLock()) {
                doc["ok"] = false;
                doc["error"] = "sd_lock_failed";
                code = 500;
            } else if (!SD.exists(path)) {
                doc["ok"] = false;
                doc["error"] = "not_found";
                code = 404;
                sdUnlock();
            } else {
                const bool ok = deletePathRecursive(path, false);
                if (path.startsWith("/recycle/")) {
                    String metaPath = recycleMetaPathFor(path);
                    if (SD.exists(metaPath)) SD.remove(metaPath);
                }
                sdUnlock();
                doc["ok"] = ok;
                doc["path"] = path;
                if (!ok) {
                    doc["error"] = "delete_failed";
                    code = 500;
                }
            }
            fsWriteEnd();
        }

        String payload;
        serializeJson(doc, payload);
        request->send(code, "application/json", payload);
    });

    server.on("/recover_sd", HTTP_GET, [](AsyncWebServerRequest *request) {
        String name = request->hasParam("name") ? urlDecode(request->getParam("name")->value()) : String("");
        JsonDocument doc;
        int code = 200;

        if (!name.length()) {
            doc["ok"] = false;
            doc["error"] = "missing_name";
            code = 400;
        } else if (!sdEnsureMounted()) {
            doc["ok"] = false;
            doc["error"] = "no_sd";
            code = 503;
        } else if (mediaIsPlaying || mediaPaused) {
            doc["ok"] = false;
            doc["error"] = "media_busy";
            code = 409;
        } else if (fsWriteBusy()) {
            doc["ok"] = false;
            doc["error"] = "fs_busy";
            code = 409;
        } else {
            fsWriteBegin();
            if (!sdLock()) {
                doc["ok"] = false;
                doc["error"] = "sd_lock_failed";
                code = 500;
            } else {
                String recyclePath = "/recycle/" + name;
                String metaPath = recycleMetaPathFor(recyclePath);
                String dst = "/" + name;

                if (!SD.exists(recyclePath)) {
                    doc["ok"] = false;
                    doc["error"] = "not_found";
                    code = 404;
                } else {
                    if (SD.exists(metaPath)) {
                        File meta = SD.open(metaPath, FILE_READ);
                        if (meta) {
                            String orig = meta.readString();
                            meta.close();
                            orig.trim();
                            if (orig.startsWith("/")) dst = orig;
                        }
                    }
                    ensureFolderExists(dst);
                    if (SD.exists(dst)) {
                        doc["ok"] = false;
                        doc["error"] = "dest_exists";
                        doc["dest"] = dst;
                        code = 409;
                    } else {
                        const bool ok = SD.rename(recyclePath, dst);
                        if (ok && SD.exists(metaPath)) SD.remove(metaPath);
                        doc["ok"] = ok;
                        doc["path"] = dst;
                        if (!ok) {
                            doc["error"] = "recover_failed";
                            code = 500;
                        }
                    }
                }
                sdUnlock();
            }
            fsWriteEnd();
        }

        String payload;
        serializeJson(doc, payload);
        request->send(code, "application/json", payload);
    });

    server.on("/create_file", HTTP_POST, [](AsyncWebServerRequest *request) {
        String path;
        if (request->hasParam("path", true)) path = urlDecode(request->getParam("path", true)->value());
        else if (request->hasParam("path", false)) path = urlDecode(request->getParam("path", false)->value());
        path = normalizeSdRoutePath(path, false);

        JsonDocument doc;
        int code = 200;

        if (!path.length()) {
            doc["ok"] = false;
            doc["error"] = "missing_path";
            code = 400;
        } else if (!pathAllowedInRecovery(path)) {
            doc["ok"] = false;
            doc["error"] = "path_not_allowed";
            code = 400;
        } else if (mediaIsPlaying || mediaPaused) {
            doc["ok"] = false;
            doc["error"] = "media_busy";
            code = 409;
        } else if (fsWriteBusy()) {
            doc["ok"] = false;
            doc["error"] = "fs_busy";
            code = 409;
        } else if (!sdEnsureMounted()) {
            doc["ok"] = false;
            doc["error"] = "no_sd";
            code = 503;
        } else {
            fsWriteBegin();
            if (!sdLock()) {
                doc["ok"] = false;
                doc["error"] = "sd_lock_failed";
                code = 500;
            } else {
                String finalPath = makeUniquePath(path);
                ensureFolderExists(finalPath);
                File file = SD.open(finalPath, FILE_WRITE);
                const bool ok = static_cast<bool>(file);
                if (file) file.close();
                sdUnlock();
                doc["ok"] = ok;
                doc["path"] = finalPath;
                if (!ok) {
                    doc["error"] = "create_failed";
                    code = 500;
                }
            }
            fsWriteEnd();
        }

        String payload;
        serializeJson(doc, payload);
        request->send(code, "application/json", payload);
    });

    server.on("/create_folder", HTTP_POST, [](AsyncWebServerRequest *request) {
        String path;
        if (request->hasParam("path", true)) path = urlDecode(request->getParam("path", true)->value());
        else if (request->hasParam("path", false)) path = urlDecode(request->getParam("path", false)->value());
        path = normalizeSdRoutePath(path, false);

        JsonDocument doc;
        int code = 200;

        if (!path.length()) {
            doc["ok"] = false;
            doc["error"] = "missing_path";
            code = 400;
        } else if (!pathAllowedInRecovery(path)) {
            doc["ok"] = false;
            doc["error"] = "path_not_allowed";
            code = 400;
        } else if (mediaIsPlaying || mediaPaused) {
            doc["ok"] = false;
            doc["error"] = "media_busy";
            code = 409;
        } else if (fsWriteBusy()) {
            doc["ok"] = false;
            doc["error"] = "fs_busy";
            code = 409;
        } else if (!sdEnsureMounted()) {
            doc["ok"] = false;
            doc["error"] = "no_sd";
            code = 503;
        } else {
            fsWriteBegin();
            if (!sdLock()) {
                doc["ok"] = false;
                doc["error"] = "sd_lock_failed";
                code = 500;
            } else {
                String finalPath = makeUniquePath(path);
                const bool ok = SD.mkdir(finalPath);
                sdUnlock();
                doc["ok"] = ok;
                doc["path"] = finalPath;
                if (!ok) {
                    doc["error"] = "mkdir_failed";
                    code = 500;
                }
            }
            fsWriteEnd();
        }

        String payload;
        serializeJson(doc, payload);
        request->send(code, "application/json", payload);
    });

    server.on("/upload_sd", HTTP_POST,
              [](AsyncWebServerRequest *request) {
                  FsUploadCtx *ctx = reinterpret_cast<FsUploadCtx *>(request->_tempObject);
                  JsonDocument doc;
                  int code = 200;
                  if (ctx) {
                      if (ctx->file) ctx->file.close();
                      if (ctx->started) {
                          if (!ctx->tmpPath.isEmpty() && sdEnsureMounted() && sdLock()) {
                              if (ctx->error.length()) {
                                  if (SD.exists(ctx->tmpPath)) SD.remove(ctx->tmpPath);
                              } else {
                                  if (SD.exists(ctx->destPath)) {
                                      if (!SD.exists("/recycle")) SD.mkdir("/recycle");
                                      String recyclePath = makeUniquePath("/recycle/" + ctx->destPath.substring(ctx->destPath.lastIndexOf('/') + 1));
                                      String metaPath = recycleMetaPathFor(recyclePath);
                                      File meta = SD.open(metaPath, FILE_WRITE);
                                      if (meta) {
                                          meta.print(ctx->destPath);
                                          meta.close();
                                      }
                                      if (!SD.rename(ctx->destPath, recyclePath) && SD.exists(metaPath)) SD.remove(metaPath);
                                  }
                                  if (!SD.rename(ctx->tmpPath, ctx->destPath)) {
                                      if (SD.exists(ctx->tmpPath)) SD.remove(ctx->tmpPath);
                                      ctx->error = "rename_failed";
                                  }
                              }
                              sdUnlock();
                          } else if (ctx->tmpPath.length()) {
                              ctx->error = "sd_lock_failed";
                          }
                          fsWriteEnd();
                      }
                      if (ctx->error.length()) {
                          code = 400;
                          doc["ok"] = false;
                          doc["error"] = ctx->error;
                      } else {
                          doc["ok"] = true;
                          doc["path"] = ctx->destPath;
                          doc["bytes"] = static_cast<uint32_t>(ctx->bytesWritten);
                      }
                      delete ctx;
                      request->_tempObject = nullptr;
                  } else {
                      code = 500;
                      doc["ok"] = false;
                      doc["error"] = "unknown";
                  }
                  String payload;
                  serializeJson(doc, payload);
                  request->send(code, "application/json", payload);
              },
              [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
                  FsUploadCtx *ctx = reinterpret_cast<FsUploadCtx *>(request->_tempObject);
                  if (!ctx) {
                      ctx = new FsUploadCtx();
                      request->_tempObject = ctx;
                  }

                  if (index == 0) {
                      if (mediaIsPlaying || mediaPaused) {
                          ctx->error = "media_busy";
                          return;
                      }
                      if (!sdEnsureMounted()) {
                          ctx->error = "no_sd";
                          return;
                      }
                      if (fsWriteBusy()) {
                          ctx->error = "fs_busy";
                          return;
                      }
                      fsWriteBegin();
                      ctx->started = true;

                      String destPath;
                      if (request->hasParam("path", true)) destPath = urlDecode(request->getParam("path", true)->value());
                      else if (request->hasParam("path", false)) destPath = urlDecode(request->getParam("path", false)->value());
                      else destPath = "/" + filename;
                      destPath = normalizeSdRoutePath(destPath, false);
                      ctx->destPath = destPath;

                      if (!ctx->destPath.length() || !pathAllowedInRecovery(ctx->destPath)) {
                          ctx->error = "path_not_allowed";
                          return;
                      }

                      ctx->tmpPath = ctx->destPath + ".upload_tmp";
                      ensureFolderExists(ctx->destPath);
                      if (!sdLock()) {
                          ctx->error = "sd_lock_failed";
                          return;
                      }
                      if (SD.exists(ctx->tmpPath)) SD.remove(ctx->tmpPath);
                      ctx->file = SD.open(ctx->tmpPath, FILE_WRITE);
                      sdUnlock();
                      if (!ctx->file) {
                          ctx->error = "open_failed";
                          return;
                      }
                  }

                  if (!ctx->error.length() && ctx->file && len) {
                      const size_t written = ctx->file.write(data, len);
                      if (written != len) {
                          ctx->error = "write_failed";
                      } else {
                          ctx->bytesWritten += written;
                      }
                  }

                  if (final && ctx->file) {
                      ctx->file.flush();
                      ctx->file.close();
                  }
              });

    server.on("/getsettings", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        doc["darkMode"] = 0;
        doc["horizontalScreen"] = 0;
        doc["holdBucket"] = 0;
        doc["holdAux"] = 0;
        doc["BluepadEnabled"] = 0;
        doc["GamepadEnabled"] = 0;
        doc["ModelRotX"] = 0;
        doc["ModelRotY"] = 0;
        doc["ModelRotZ"] = 0;
        doc["ModelDirX"] = 1;
        doc["ModelDirY"] = 1;
        doc["ModelDirZ"] = 1;
        doc["ModelAxisX"] = "x";
        doc["ModelAxisY"] = "y";
        doc["ModelAxisZ"] = "z";
        appendUiSettings(doc);

        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/get_mqtt", HTTP_GET, [](AsyncWebServerRequest *request) {
        loadMqttConfig();
        JsonDocument doc;
        doc["enable"] = mqttCfg.enabled;
        doc["host"] = mqttCfg.broker;
        doc["port"] = mqttCfg.port;
        doc["user"] = mqttCfg.username;
        doc["pass"] = mqttCfg.password;
        doc["topic_prefix"] = mqttCfg.discoveryPrefix;
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/set_mqtt", HTTP_POST,
              [](AsyncWebServerRequest *request) {
                  String *body = reinterpret_cast<String *>(request->_tempObject);
                  if (!body) {
                      request->send(400, "text/plain", "Invalid JSON");
                      return;
                  }

                  JsonDocument doc;
                  const DeserializationError err = deserializeJson(doc, *body);
                  delete body;
                  request->_tempObject = nullptr;

                  if (err || !doc.is<JsonObject>()) {
                      request->send(400, "text/plain", "Invalid JSON");
                      return;
                  }

                  JsonObject obj = doc.as<JsonObject>();
                  mqttCfg.enabled = obj["enable"] | false;
                  mqttCfg.broker = String(static_cast<const char *>(obj["host"] | ""));
                  mqttCfg.port = static_cast<uint16_t>(obj["port"] | 1883);
                  mqttCfg.username = String(static_cast<const char *>(obj["user"] | ""));
                  mqttCfg.password = String(static_cast<const char *>(obj["pass"] | ""));
                  mqttCfg.discoveryPrefix = String(static_cast<const char *>(obj["topic_prefix"] | "homeassistant"));
                  if (mqttCfg.port == 0) mqttCfg.port = 1883;
                  if (mqttCfg.discoveryPrefix.isEmpty()) mqttCfg.discoveryPrefix = "homeassistant";
                  mqttChatInboxTopic = "esp32/remote/chat/" + String(MQTT_CHAT_NAMESPACE) + "/inbox/" + p2pPublicKeyHex();
                  saveMqttConfig();
                  mqttDiscoveryPublished = false;
                  mqttLastReconnectMs = 0;
                  if (mqttClient.connected()) mqttClient.disconnect();
                  if (mqttCfg.enabled && wifiConnectedSafe() && !networkSuspendedForAudio) mqttConnectNow();
                  else mqttStatusLine = mqttCfg.enabled ? "Enabled" : "Disabled";
                  request->send(200, "text/plain", "OK");
              },
              nullptr,
              [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
                  String *body = reinterpret_cast<String *>(request->_tempObject);
                  if (index == 0) {
                      body = new String();
                      body->reserve(total + 1);
                      request->_tempObject = body;
                  }
                  if (body && len) body->concat(reinterpret_cast<const char *>(data), len);
              });

    server.on("/mqtt_test", HTTP_POST, [](AsyncWebServerRequest *request) {
        loadMqttConfig();
        if (mqttCfg.broker.isEmpty() || mqttCfg.port == 0) {
            request->send(200, "text/plain", "Invalid host/port");
            return;
        }
        if (!wifiConnectedSafe()) {
            request->send(200, "text/plain", "No WiFi");
            return;
        }

        WiFiClient testNetClient;
        PubSubClient testClient(testNetClient);
        testNetClient.setTimeout(2000);
        testClient.setServer(mqttCfg.broker.c_str(), mqttCfg.port);
        testClient.setSocketTimeout(2);
        String clientId = "MiniExcoTest-" + String(static_cast<uint32_t>(millis()));

        bool connected = false;
        yield();
        if (mqttCfg.username.length()) {
            connected = testClient.connect(clientId.c_str(), mqttCfg.username.c_str(), mqttCfg.password.c_str());
        } else {
            connected = testClient.connect(clientId.c_str());
        }
        yield();
        if (connected) testClient.disconnect();

        request->send(200, "text/plain", connected ? "MQTT connection OK" : "MQTT connection failed");
    });

    server.on("/mqtt_status", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        const bool connected = mqttClient.connected();
        doc["connected"] = connected;
        if (!connected && !mqttStatusLine.isEmpty() && mqttStatusLine != "Connected") {
            doc["last_error"] = mqttStatusLine;
        }
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/mqtt_discovery", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (mqttClient.connected()) {
            mqttPublishDiscovery();
            request->send(200, "text/plain", "Discovery published");
        } else {
            request->send(200, "text/plain", "MQTT not connected");
        }
    });

    server.on("/get_keymap", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        doc["forward"] = "w";
        doc["backward"] = "s";
        doc["left"] = "a";
        doc["right"] = "d";
        doc["stop"] = " ";
        doc["arm_up"] = " ";
        doc["arm_down"] = " ";
        doc["bucket_up"] = "e";
        doc["bucket_down"] = "q";
        doc["aux_up"] = "r";
        doc["aux_down"] = "f";
        doc["light_toggle"] = "l";
        doc["beacon"] = "b";
        doc["emergency"] = "x";
        doc["horn"] = "h";

        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/camera_status", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        doc["enabled"] = false;
        doc["initialized"] = false;
        doc["boot_restore_pending"] = false;

        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/heap_report", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;

        const uint32_t heapFree = static_cast<uint32_t>(ESP.getFreeHeap());
        const uint32_t heapTotal = static_cast<uint32_t>(ESP.getHeapSize());
        const uint32_t heapLargest = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        const uint32_t heapMinFree = static_cast<uint32_t>(ESP.getMinFreeHeap());

        const uint32_t internalFree = static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        const uint32_t internalTotal = static_cast<uint32_t>(heap_caps_get_total_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        const uint32_t internalLargest = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        const uint32_t internalMinFree = static_cast<uint32_t>(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

        const uint32_t psramFree = static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        const uint32_t psramTotal = static_cast<uint32_t>(heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
        const uint32_t psramLargest = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
        const uint32_t psramMinFree = static_cast<uint32_t>(heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM));

        JsonObject heap = doc["heap"].to<JsonObject>();
        heap["free"] = heapFree;
        heap["total"] = heapTotal;
        heap["largest_free_block"] = heapLargest;
        heap["min_free"] = heapMinFree;
        heap["frag_pct"] = heapFragPercent(heapFree, heapLargest);

        JsonObject internalHeap = doc["internal_heap"].to<JsonObject>();
        internalHeap["free"] = internalFree;
        internalHeap["total"] = internalTotal;
        internalHeap["largest_free_block"] = internalLargest;
        internalHeap["min_free"] = internalMinFree;
        internalHeap["frag_pct"] = heapFragPercent(internalFree, internalLargest);

        JsonObject psram = doc["psram"].to<JsonObject>();
        psram["free"] = psramFree;
        psram["total"] = psramTotal;
        psram["largest_free_block"] = psramLargest;
        psram["min_free"] = psramMinFree;
        psram["frag_pct"] = heapFragPercent(psramFree, psramLargest);

        doc["tasks"].to<JsonArray>();

        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/upload", HTTP_GET, [](AsyncWebServerRequest *request) {
        sendUploadFallbackPage(request);
    });

    server.on("/recovery", HTTP_GET, [](AsyncWebServerRequest *request) {
        sendRecoveryBrowserPage(request);
    });

    server.on("/fs/list", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        JsonObject root = doc.to<JsonObject>();

        String dirPath;
        if (request->hasParam("dir")) dirPath = urlDecode(request->getParam("dir")->value());
        if (!sdEnsureMounted()) {
            root["ok"] = false;
            root["error"] = "no_sd";
        } else if (!sdLock()) {
            root["ok"] = false;
            root["error"] = "sd_lock_failed";
        } else {
            if (dirPath.length()) {
                if (!dirPath.startsWith("/")) dirPath = "/" + dirPath;
                while (dirPath.endsWith("/") && dirPath.length() > 1) dirPath.remove(dirPath.length() - 1);
                if (dirPath != "/" && !pathAllowedInRecovery(dirPath)) {
                    root["ok"] = false;
                    root["error"] = "path_not_allowed";
                } else {
                    JsonArray entries = root["entries"].to<JsonArray>();
                    listSdDirEntries(dirPath, entries);
                    root["ok"] = true;
                    root["dir"] = dirPath;
                }
            } else {
                JsonArray files = root["files"].to<JsonArray>();
                if (SD.exists("/web")) listSdFilesRecursive("/web", files);
                root["ok"] = true;
            }
            sdUnlock();
        }

        String payload;
        serializeJson(doc, payload);
        request->send(root["ok"] == false ? 400 : 200, "application/json", payload);
    });

    server.on("/fs/mkdir", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (mediaIsPlaying || mediaPaused) {
            request->send(409, "application/json", "{\"ok\":false,\"error\":\"media_busy\"}");
            return;
        }
        if (fsWriteBusy()) {
            request->send(409, "application/json", "{\"ok\":false,\"error\":\"fs_busy\"}");
            return;
        }
        fsWriteBegin();
        String path = "/web/new_folder";
        if (request->hasParam("path", true)) path = urlDecode(request->getParam("path", true)->value());
        else if (request->hasParam("path", false)) path = urlDecode(request->getParam("path", false)->value());
        if (!path.startsWith("/")) path = "/" + path;
        while (path.endsWith("/") && path.length() > 1) path.remove(path.length() - 1);

        JsonDocument doc;
        JsonObject root = doc.to<JsonObject>();
        int code = 200;

        if (!sdEnsureMounted()) {
            root["ok"] = false;
            root["error"] = "no_sd";
            code = 503;
        } else if (!pathAllowedInRecovery(path)) {
            root["ok"] = false;
            root["error"] = "path_not_allowed";
            code = 400;
        } else if (!sdLock()) {
            root["ok"] = false;
            root["error"] = "sd_lock_failed";
            code = 500;
        } else {
            bool ok = SD.exists(path) || SD.mkdir(path);
            if (!ok) sdMarkFault("fs_mkdir");
            root["ok"] = ok;
            root["path"] = path;
            if (!ok) {
                root["error"] = "mkdir_failed";
                code = 500;
            }
            sdUnlock();
        }

        String payload;
        serializeJson(doc, payload);
        request->send(code, "application/json", payload);
        fsWriteEnd();
    });

    server.on("/fs/download", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!request->hasParam("path")) {
            request->send(400, "text/plain", "missing_path");
            return;
        }
        String path = urlDecode(request->getParam("path")->value());
        if (!path.startsWith("/")) path = "/" + path;
        if (!pathAllowedInRecovery(path)) {
            request->send(400, "text/plain", "path_not_allowed");
            return;
        }
        if (!sdEnsureMounted()) {
            request->send(503, "text/plain", "no_sd");
            return;
        }
        if (fsWriteBusy()) {
            request->send(409, "text/plain", "fs_busy");
            return;
        }
        if (!SD.exists(path)) {
            request->send(404, "text/plain", "not_found");
            return;
        }
        request->send(SD, path, mimeFor(path), true);
    });

    server.on("/fs/delete", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (mediaIsPlaying || mediaPaused) {
            request->send(409, "application/json", "{\"ok\":false,\"error\":\"media_busy\"}");
            return;
        }
        if (fsWriteBusy()) {
            request->send(409, "application/json", "{\"ok\":false,\"error\":\"fs_busy\"}");
            return;
        }
        fsWriteBegin();
        String path = "/web";
        if (request->hasParam("path", true)) path = urlDecode(request->getParam("path", true)->value());
        else if (request->hasParam("path", false)) path = urlDecode(request->getParam("path", false)->value());
        if (!path.startsWith("/")) path = "/" + path;
        while (path.endsWith("/") && path.length() > 1) path.remove(path.length() - 1);

        JsonDocument doc;
        JsonObject root = doc.to<JsonObject>();
        int code = 200;

        if (!sdEnsureMounted()) {
            root["ok"] = false;
            root["error"] = "no_sd";
            code = 503;
        } else if (!pathAllowedInRecovery(path)) {
            root["ok"] = false;
            root["error"] = "path_not_allowed";
            code = 400;
        } else if (!sdLock()) {
            root["ok"] = false;
            root["error"] = "sd_lock_failed";
            code = 500;
        } else {
            bool exists = SD.exists(path);
            String lowPath = path;
            lowPath.toLowerCase();
            bool keepRoot = (path == "/web") || (lowPath == "/screenshots");
            bool ok = exists ? deletePathRecursive(path, keepRoot) : true;
            if (path == "/web" && !SD.exists("/web")) SD.mkdir("/web");
            if (pathUnderScreenshots(path) && !SD.exists("/Screenshots")) SD.mkdir("/Screenshots");
            if (!ok) sdMarkFault("fs_delete");
            root["ok"] = ok;
            if (!ok) {
                root["error"] = "delete_failed";
                code = 500;
            }
            sdUnlock();
        }

        String payload;
        serializeJson(doc, payload);
        request->send(code, "application/json", payload);
        fsWriteEnd();
    });

    server.on("/fs/upload", HTTP_POST,
              [](AsyncWebServerRequest *request) {
                  FsUploadCtx *ctx = reinterpret_cast<FsUploadCtx *>(request->_tempObject);
                  int code = 200;
                  String body = "ok";
                  if (ctx) {
                      if (ctx->file) ctx->file.close();
                      if (ctx->started) {
                          if (!ctx->tmpPath.isEmpty() && sdEnsureMounted() && sdLock()) {
                              if (ctx->error.length()) {
                                  if (SD.exists(ctx->tmpPath)) SD.remove(ctx->tmpPath);
                              } else {
                                  if (SD.exists(ctx->destPath)) SD.remove(ctx->destPath);
                                  if (!SD.rename(ctx->tmpPath, ctx->destPath)) {
                                      sdMarkFault("fs_upload_rename");
                                      body = "rename_failed";
                                      code = 500;
                                      if (SD.exists(ctx->tmpPath)) SD.remove(ctx->tmpPath);
                                  }
                              }
                              sdUnlock();
                          } else if (ctx->tmpPath.length()) {
                              body = "sd_lock_failed";
                              code = 500;
                          }
                          fsWriteEnd();
                      }
                      if (ctx->error.length()) {
                          code = 400;
                          body = ctx->error;
                      }
                      delete ctx;
                      request->_tempObject = nullptr;
                  }
                  request->send(code, "text/plain", body);
              },
              [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
                  FsUploadCtx *ctx = reinterpret_cast<FsUploadCtx *>(request->_tempObject);
                  if (!ctx) {
                      ctx = new FsUploadCtx();
                      request->_tempObject = ctx;
                  }

                  if (index == 0) {
                      if (mediaIsPlaying || mediaPaused) {
                          ctx->error = "media_busy";
                          return;
                      }
                      if (!sdEnsureMounted()) {
                          ctx->error = "no_sd";
                          return;
                      }
                      if (fsWriteBusy()) {
                          ctx->error = "fs_busy";
                          return;
                      }
                      fsWriteBegin();
                      ctx->started = true;
                      String basePath = "/web/";
                      if (request->hasParam("path", true)) basePath = urlDecode(request->getParam("path", true)->value());
                      else if (request->hasParam("path", false)) basePath = urlDecode(request->getParam("path", false)->value());
                      if (!basePath.startsWith("/")) basePath = "/" + basePath;
                      if (!basePath.endsWith("/")) basePath += "/";
                      ctx->destPath = basePath + filename;
                      ctx->tmpPath = ctx->destPath + ".upload_tmp";

                      if (!pathAllowedInRecovery(ctx->destPath)) {
                          ctx->error = "path_not_allowed";
                          return;
                      }

                      ensureFolderExists(ctx->destPath);
                      if (!sdLock()) {
                          ctx->error = "sd_lock_failed";
                          return;
                      }
                      if (SD.exists(ctx->tmpPath)) SD.remove(ctx->tmpPath);
                      ctx->file = SD.open(ctx->tmpPath, FILE_WRITE);
                      sdUnlock();
                      if (!ctx->file) {
                          sdMarkFault("fs_upload_open");
                          ctx->error = "open_failed";
                          return;
                      }
                  }

                  if (!ctx->error.length() && ctx->file && len) {
                      if (ctx->file.write(data, len) != len) {
                          sdMarkFault("fs_upload_write");
                          ctx->error = "write_failed";
                      }
                  }

                  if (final && ctx->file) {
                      ctx->file.flush();
                      ctx->file.close();
                  }
              });

    server.on("/sd_info", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        JsonObject root = doc.to<JsonObject>();
        SdStats s = sdStatsSnapshot();
        root["sd_faults"] = s.faultCount;
        root["sd_remount_attempts"] = s.remountAttempts;
        root["sd_remount_ok"] = s.remountSuccess;
        root["sd_remount_fail"] = s.remountFailure;
        root["sd_try_8mhz"] = s.remountTryFast;
        root["sd_try_4mhz"] = s.remountTryRecovery;
        root["sd_try_1mhz"] = s.remountTrySafe;
        root["sd_force_remounts"] = s.forceRemountCalls;
        root["sd_auto_retries"] = s.autoRetryCalls;
        root["sd_last_fault_ms"] = s.lastFaultMs;
        root["sd_last_remount_ok_ms"] = s.lastRemountOkMs;
        root["sd_last_remount_fail_ms"] = s.lastRemountFailMs;
        root["sd_last_fault_at"] = s.lastFaultWhere[0] ? String(s.lastFaultWhere) : String("");
        if (!sdEnsureMounted()) {
            root["ok"] = false;
            root["error"] = "no_sd";
            String payload;
            serializeJson(doc, payload);
            request->send(503, "application/json", payload);
            return;
        }
        uint64_t total = SD.totalBytes();
        uint64_t used = SD.usedBytes();
        root["ok"] = true;
        root["total"] = total;
        root["used"] = used;
        root["free"] = (total >= used) ? (total - used) : 0;
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        const bool staConnected = (WiFi.status() == WL_CONNECTED);
        if (sdEnsureMounted() && hasPrimaryWebUi()) {
            String idx = webIndexPath();
            if (idx.length() && sendMaybeGz(request, idx)) return;
            // If core UI is expected but index still failed, keep behavior deterministic.
            request->send(500, "text/plain", "Web UI gate passed but index missing");
            return;
        }
        // Connected: go to recovery file manager when SD UI is missing.
        if (staConnected) {
            request->redirect("/recovery");
            return;
        }
        // Not connected: WiFi setup captive page.
        sendWifiSetupPage(request);
    });

    server.onNotFound([](AsyncWebServerRequest *request) {
        const bool staConnected = (WiFi.status() == WL_CONNECTED);
        if (request->url().startsWith("/api/")) {
            request->send(404, "application/json", "{\"ok\":false,\"error\":\"not_found\"}");
            return;
        }
        if (sdEnsureMounted() && hasPrimaryWebUi()) {
            sendFileFromWebOr404(request, request->url());
            return;
        }
        if (staConnected) {
            // Connected without SD UI -> fallback manager flow.
            if (request->url() == "/upload" || request->url() == "/recovery") return;
            request->redirect("/upload");
            return;
        }
        request->redirect("/wifi");
    });
}

void wifiConnectionService()
{
    if (networkSuspendedForAudio || airplaneModeEnabled) return;

    if (wifiStaGotIpPending) {
        wifiStaGotIpPending = false;
        bootStaConnectInProgress = false;
        const String ssid = wifiSsidSafe();
        const String ip = wifiIpSafe();
        disableApWhenStaConnected("sta_got_ip");
        if (pendingSaveCreds && ssid == pendingSaveSsid) {
            saveStaCreds(pendingSaveSsid, pendingSavePass);
            pendingSaveCreds = false;
            pendingSaveSsid = "";
            pendingSavePass = "";
        }
        uiStatusLine = "Connected: " + ssid;
        if (lvglWifiPwdConnectPending && ssid == lvglWifiPendingSsid) {
            lvglWifiPwdConnectPending = false;
            if (lvglWifiPwdStatusLabel) lv_label_set_text(lvglWifiPwdStatusLabel, "Connected");
            lvglWifiPwdCancelEvent(nullptr);
            lvglOpenScreen(UI_HOME, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
        }
        Serial.printf("[WIFI] STA connected ssid=%s ip=%s rssi=%d\n",
                      ssid.c_str(),
                      ip.c_str(),
                      static_cast<int>(wifiRssiSafe()));
    }

    if (wifiStaDisconnectedPending) {
        const uint8_t reason = wifiStaDisconnectReasonPending;
        wifiStaDisconnectedPending = false;
        bootStaConnectInProgress = false;
        ensureApOnline("sta_disconnected");
        uiStatusLine = "WiFi disconnected";
        if (lvglWifiPwdConnectPending) {
            lvglWifiPwdConnectPending = false;
            if (lvglWifiPwdStatusLabel) {
                lv_label_set_text_fmt(lvglWifiPwdStatusLabel, "Connection failed: %s",
                                      WiFi.disconnectReasonName(static_cast<wifi_err_reason_t>(reason)));
            }
            if (lvglWifiPwdTa) {
                lv_textarea_set_text(lvglWifiPwdTa, "");
                lv_event_send(lvglWifiPwdTa, LV_EVENT_FOCUSED, nullptr);
            }
        }
        Serial.printf("[WIFI] STA disconnected reason=%u (%s)\n",
                      static_cast<unsigned int>(reason),
                      WiFi.disconnectReasonName(static_cast<wifi_err_reason_t>(reason)));
    }

    if (wifiConnectedSafe()) return;
    if (!wifiHasStaTarget()) {
        if (!apModeActive) ensureApOnline("no_sta_target");
        return;
    }

    const unsigned long now = millis();
    if (bootStaConnectInProgress) {
        if (static_cast<unsigned long>(now - bootStaConnectStartedMs) >= BOOT_STA_TIMEOUT_MS) {
            bootStaConnectInProgress = false;
            WiFi.disconnect(false, false);
            ensureApOnline("sta_timeout");
            uiStatusLine = "Saved STA reconnect failed";
            if (lvglWifiPwdConnectPending) {
                lvglWifiPwdConnectPending = false;
                if (lvglWifiPwdStatusLabel) lv_label_set_text(lvglWifiPwdStatusLabel, "Connection timed out");
                if (lvglWifiPwdTa) {
                    lv_textarea_set_text(lvglWifiPwdTa, "");
                    lv_event_send(lvglWifiPwdTa, LV_EVENT_FOCUSED, nullptr);
                }
            }
            Serial.printf("[WIFI] STA connect timeout ssid=%s\n", wifiDesiredStaSsid().c_str());
            staLastConnectAttemptMs = now;
        }
        return;
    }

    ensureApOnline("sta_retry_pending");
    if (staLastConnectAttemptMs == 0 || static_cast<unsigned long>(now - staLastConnectAttemptMs) >= STA_RETRY_INTERVAL_MS) {
        beginStaConnectAttempt("retry");
    }
}

void refreshMdnsState()
{
    if (airplaneModeEnabled) {
        if (mdnsStarted) {
            MDNS.end();
            mdnsStarted = false;
        }
        return;
    }
    if (networkSuspendedForAudio) {
        if (mdnsStarted) {
            MDNS.end();
            mdnsStarted = false;
        }
        return;
    }
    if (wifiConnectedSafe()) {
        if (!mdnsStarted) {
            if (MDNS.begin(MDNS_HOST)) {
                MDNS.addService("http", "tcp", 80);
                mdnsStarted = true;
                uiStatusLine = "mDNS active: " + String(MDNS_HOST) + ".local";
            }
        }
    } else if (mdnsStarted) {
        MDNS.end();
        mdnsStarted = false;
    }
}

void networkSuspendForAudio()
{
    if (networkSuspendedForAudio) return;

    mqttClient.disconnect();
    if (mdnsStarted) {
        MDNS.end();
        mdnsStarted = false;
    }
    if (dnsRunning) {
        dnsServer.stop();
        dnsRunning = false;
    }
    if (webServerRunning) {
        wsCarInput.cleanupClients();
        server.end();
        webServerRunning = false;
    }

    WiFi.softAPdisconnect(true);
    apModeActive = false;
    stopDnsForAp();
    WiFi.disconnect(true, false);
    WiFi.mode(WIFI_OFF);
    delay(40);
    networkSuspendedForAudio = true;
    bootStaConnectInProgress = false;
    wifiStaGotIpPending = false;
    wifiStaDisconnectedPending = false;
    networkResumeLastAttemptMs = 0;
    Serial.printf("[AUDIO] network suspended for playback, free_heap=%lu largest=%lu\n",
                  static_cast<unsigned long>(ESP.getFreeHeap()),
                  static_cast<unsigned long>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
}

bool networkResumeAfterAudio()
{
    if (!networkSuspendedForAudio) return true;
    if (mediaIsPlaying || mediaPaused) return false;
    if (uiScreen == UI_MEDIA) return false;
    if (airplaneModeEnabled) return false;

    WiFi.mode(WIFI_OFF);
    delay(30);
    WiFi.mode(WIFI_STA);
    if (wifiHasStaTarget()) {
        beginStaConnectAttempt("audio_resume");
    } else {
        ensureApOnline("audio_resume_no_sta");
    }
    if (wifiHasStaTarget() || apModeActive) {
        networkSuspendedForAudio = false;
        Serial.printf("[AUDIO] network resume requested free_heap=%lu largest=%lu\n",
                      static_cast<unsigned long>(ESP.getFreeHeap()),
                      static_cast<unsigned long>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
        return true;
    }
    Serial.printf("[AUDIO] network resume deferred free_heap=%lu largest=%lu\n",
                  static_cast<unsigned long>(ESP.getFreeHeap()),
                  static_cast<unsigned long>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
    return false;
}

void setupWifiAndServer()
{
    if (airplaneModeEnabled) {
        WiFi.mode(WIFI_OFF);
        apModeActive = false;
        stopDnsForAp();
        return;
    }
    registerWifiEvents();
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    WiFi.mode(WIFI_STA);
    apModeActive = false;
    stopDnsForAp();

    if (!webRoutesRegistered) {
        setupWebRoutes();
        webRoutesRegistered = true;
    }
    if (!webServerRunning) {
        server.begin();
        webServerRunning = true;
    }

    loadSavedStaCreds();
    tryBootStaReconnect();
}

void setupSd()
{
    if (!sdLock(pdMS_TO_TICKS(40))) {
        Serial.println("SD boot probe skipped: lock timeout");
        return;
    }
    const bool ok = sdMountAttemptAtFreq(SD_BOOT_PROBE_FREQ_HZ);
    sdMounted = ok;
    sdUnlock();
    if (ok) Serial.printf("SD mounted (boot probe @ %lu Hz)\n", static_cast<unsigned long>(SD_BOOT_PROBE_FREQ_HZ));
    else Serial.printf("SD mount deferred (boot probe failed @ %lu Hz)\n", static_cast<unsigned long>(SD_BOOT_PROBE_FREQ_HZ));
    if (ok) chatLoadPendingOutbox();
}

void setup()
{
    sdMutex = xSemaphoreCreateMutex();
    displayBacklightInit();
    pinMode(RGB_PIN_R, OUTPUT);
    pinMode(RGB_PIN_G, OUTPUT);
    pinMode(RGB_PIN_B, OUTPUT);
    pinMode(TOUCH_IRQ, TOUCH_USE_IRQ ? INPUT_PULLUP : INPUT);
    rgbApplyNow(false, false, false);
    analogReadResolution(12);
    analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
    analogSetPinAttenuation(LIGHT_ADC_PIN, ADC_11db);
    randomSeed(static_cast<unsigned long>(esp_random()));
    p2pReady = sodium_init() >= 0;
    if (p2pReady) loadP2pConfig();
    loadUiRuntimeConfig();
    mqttBuildIdentity();
    loadMqttConfig();
    if (mqttCfg.enabled) mqttStatusLine = "Enabled";
    else mqttStatusLine = "Disabled";
    sampleTopIndicators();
    float prevBootV = 0.0f;
    if (loadBatterySnapshot(prevBootV)) {
        if ((batteryVoltage - prevBootV) >= BATTERY_BOOT_CHARGE_DELTA_V) {
            chargeTrendScore = CHARGE_SCORE_ON;
            batteryCharging = true;
            lastChargeSeenMs = millis();
        }
    }
    lastBatterySnapshotVoltage = batteryVoltage;
    lastBatterySnapshotMs = millis();

    Serial.begin(115200);
    Serial.println();
    Serial.printf("%s UI + fallback file manager\n", deviceShortNameValue().c_str());
    Serial.printf("[P2P] ready=%d pub=%s\n", p2pReady ? 1 : 0, p2pReady ? p2pPublicKeyHex().c_str() : "-");

    tft.init();
    tft.setRotation(TFT_ROTATION);
    tft.setTextFont(2);
    tft.fillScreen(TFT_BLACK);

    Serial.println("[BOOT] touch pre-init");
    touchInit();
    lvglInitUi();
    if (lvglReady) {
        lv_timer_handler();
        delay(20);
    }
    displayBacklightFadeIn();
    Serial.println("[BOOT] SD/network init deferred to loop");
    bootDeferredStartMs = millis();
    bootSdInitPending = true;
    bootWifiInitPending = true;
    delay(20);
    sdStatsLogSnapshot(sdStatsSnapshot(), "boot");
    Serial.println("[AUDIO] backend init deferred until playback");
    rgbRefreshByMediaState();
    lastUserActivityMs = millis();
    lastSensorSampleMs = millis();

    Serial.printf("Audio decoder ready, internal DAC2 on GPIO %d only (GPIO25 kept for TOUCH_RST)\n", I2S_SPK_PIN);
    Serial.printf("[TOUCH] ctrl=%s mode=%s irq_pin=%d i2c=(sda=%d,scl=%d,rst=%d)\n",
                  TOUCH_CONTROLLER == TOUCH_CTRL_GT911 ? "gt911" : "cst820",
                  TOUCH_USE_IRQ ? "irq+poll" : "poll-only",
                  TOUCH_IRQ, TOUCH_SDA, TOUCH_SCL, TOUCH_RST);
}

void loop()
{
    const uint32_t loopStartUs = micros();
    cpuLoadService(loopStartUs);
    lvglService();
    bool isDown = lvglReady ? lvglTouchDown : false;

    const unsigned long now = millis();
    if (bootSdInitPending && static_cast<unsigned long>(now - bootDeferredStartMs) >= BOOT_DEFER_SD_MS) {
        bootSdInitPending = false;
        setupSd();
        sdStatsLogSnapshot(sdStatsSnapshot(), "boot_deferred_sd");
    }
    if (bootWifiInitPending && static_cast<unsigned long>(now - bootDeferredStartMs) >= BOOT_DEFER_WIFI_MS) {
        bootWifiInitPending = false;
        setupWifiAndServer();
    }

    if (dnsRunning) dnsServer.processNextRequest();
    wifiScanService();
    wifiConnectionService();
    sdStatsService();
    const bool allowSdAutoRetry = (uiScreen == UI_MEDIA) || !displayAwake;
    if (!sdMounted && allowSdAutoRetry && !isDown && !fsWriteBusy() &&
        static_cast<unsigned long>(millis() - sdLastAutoRetryMs) >= SD_AUTORETRY_PERIOD_MS) {
        sdLastAutoRetryMs = millis();
        portENTER_CRITICAL(&sdStatsMux);
        sdStats.autoRetryCalls++;
        portEXIT_CRITICAL(&sdStatsMux);
        sdEnsureMounted(true);
    }
    refreshMdnsState();
    if (p2pReady) p2pService();
    mqttService();
    chatPendingService();
    if (rgbFlashUntilMs != 0 && static_cast<long>(millis() - rgbFlashUntilMs) >= 0) {
        rgbFlashUntilMs = 0;
        rgbApplyNow(rgbPersistR, rgbPersistG, rgbPersistB);
    }
    rgbService();
    bool wasRunning = mediaIsPlaying || mediaPaused;
    // Keep decoder fed continuously; sparse servicing causes audible clicks.
    audioService();
    const bool decoderRunning = audioBackendReady && audio && audio->isRunning();
    if (wasRunning && !decoderRunning && mediaIsPlaying && !mediaPaused) {
        mediaIsPlaying = false;
        uiStatusLine = "Track finished";
        rgbRefreshByMediaState();
        lvglSyncStatusLine();
        lvglRefreshMediaPlayerUi();
    }

    if (networkSuspendedForAudio && !mediaIsPlaying && !mediaPaused && uiScreen != UI_MEDIA) {
        if (static_cast<unsigned long>(millis() - networkResumeLastAttemptMs) >= AUDIO_NETWORK_RESUME_RETRY_MS) {
            networkResumeLastAttemptMs = millis();
            networkResumeAfterAudio();
        }
    }

    if (millis() - lastSensorSampleMs >= SENSOR_SAMPLE_PERIOD_MS) {
        lastSensorSampleMs = millis();
        sampleTopIndicators();
    }

    serviceCarInputTelemetry();

    if (rebootRequested && static_cast<unsigned long>(millis() - rebootRequestedAtMs) >= 150UL) {
        delay(20);
        ESP.restart();
    }

    if (!displayAwake && (millis() - lastUserActivityMs >= LCD_IDLE_TIMEOUT_MS)) {
        // Keep state as-is while sleeping.
    } else if (displayAwake && (millis() - lastUserActivityMs >= LCD_IDLE_TIMEOUT_MS)) {
        bool allowSleep = true;
        if (batteryCharging) allowSleep = animateChargingBeforeSleep();
        if (allowSleep) displaySetAwake(false);
    }

    if (!displayAwake) {
        wakeTouchConfirmCount = 0;
    }

    if (displayAwake && isDown) lastUserActivityMs = millis();

    snakeTick();
    tetrisTick();

    static wl_status_t lastWiFi = WL_DISCONNECTED;
    wl_status_t cur = wifiStatusSafe();

    if (cur != lastWiFi) {
        if (cur == WL_CONNECTED) {
            bootStaConnectInProgress = false;
            const String ssid = wifiSsidSafe();
            uiStatusLine = "Connected: " + ssid;
            if (pendingSaveCreds && ssid == pendingSaveSsid) {
                saveStaCreds(pendingSaveSsid, pendingSavePass);
                pendingSaveCreds = false;
                pendingSaveSsid = "";
                pendingSavePass = "";
                uiStatusLine = "Connected+Saved: " + ssid;
            }
            if (uiScreen == UI_MEDIA) lvglQueueMediaRefresh();
        } else {
            if (lastWiFi == WL_CONNECTED) uiStatusLine = "WiFi disconnected";
        }
        if (lvglReady) {
            lvglSyncStatusLine();
            lvglRefreshInfoPanel();
            lvglRefreshWifiList();
            if (lvglMqttStatusLabel) lv_label_set_text_fmt(lvglMqttStatusLabel, "MQTT: %s", mqttStatusLine.c_str());
        }
        lastWiFi = cur;
    }

    if (lvglReady && displayAwake && millis() - lastTopIndicatorRefreshMs >= 1500) {
        lastTopIndicatorRefreshMs = millis();
        lvglRefreshInfoPanel();
    }

    // Service audio again after UI/network work to reduce underruns.
    audioService();

    if (millis() - lastBatterySnapshotMs >= BATTERY_SNAPSHOT_PERIOD_MS) {
        sampleTopIndicators();
        const float dv = (lastBatterySnapshotVoltage < 0.0f) ? 999.0f : fabsf(batteryVoltage - lastBatterySnapshotVoltage);
        const bool dueByDelta = dv >= BATTERY_SNAPSHOT_MIN_DELTA_V;
        const bool dueByForce = (millis() - lastBatterySnapshotMs) >= BATTERY_SNAPSHOT_FORCE_MS;
        if (dueByDelta || dueByForce) {
            saveBatterySnapshot(batteryVoltage);
            lastBatterySnapshotVoltage = batteryVoltage;
            lastBatterySnapshotMs = millis();
        }
    }

    if (canEnterLowPowerSleep(isDown)) enterLowPowerSleep();
    cpuLoadPrevActiveUs = micros() - loopStartUs;
    delay(1);
}
