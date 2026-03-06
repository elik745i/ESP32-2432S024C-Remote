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
#include <Wire.h>
#include <Audio.h>
#include <JPEGENC.h>
#include <HTTPClient.h>
#include <Update.h>
#include <WiFiClientSecure.h>
#include <esp_sleep.h>
#include <esp_heap_caps.h>
#include <driver/gpio.h>
#include <math.h>
#include <new>
#include <string.h>
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ESP32-2432S024C (CST820 capacitive touch).
static constexpr int TOUCH_SDA = 33;
static constexpr int TOUCH_SCL = 32;
static constexpr int TOUCH_RST = 25;
static constexpr int TOUCH_IRQ = 21;
static constexpr int TOUCH_ADDR = 0x15;
static constexpr uint32_t TOUCH_I2C_HZ = 400000U;
static constexpr bool TOUCH_USE_IRQ = false;
static constexpr unsigned long TOUCH_POLL_INTERVAL_MS = 12UL;
static constexpr uint16_t TOUCH_REINIT_FAIL_THRESHOLD = 40;
static constexpr unsigned long TOUCH_REINIT_MIN_INTERVAL_MS = 8000UL;
static constexpr uint8_t TFT_ROTATION = 0;
static constexpr uint8_t TOUCH_ROTATION_OFFSET = 0;

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
static constexpr int AUDIO_INPUT_RAM_BUFFER_BYTES = 4096;
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
static constexpr bool CHARGE_LOG_TO_SERIAL = true;
static constexpr uint8_t CHARGE_ANIM_CYCLES = 2;
static constexpr int LIGHT_ADC_SAMPLES = 8;
static constexpr float LIGHT_FILTER_ALPHA = 0.20f;
static constexpr bool LIGHT_INVERT = true;
static constexpr int LIGHT_MIN_SPAN_RAW = 80;
static constexpr uint16_t LIGHT_RAW_CAL_MIN = 0;
static constexpr uint16_t LIGHT_RAW_CAL_MAX = 600;
static constexpr bool LIGHT_LOG_RAW_TO_SERIAL = true;

static constexpr const char *AP_SSID = "ESP32-2432S024C-FM";
static constexpr const char *AP_PASS = "12345678";
static constexpr const char *FW_VERSION = "0.1.1";
static constexpr const char *MDNS_HOST = "esp32-2432s024c";
static constexpr unsigned long STA_RETRY_INTERVAL_MS = 5000UL;

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
void loadMediaEntries();
void refreshWifiScan();
void startWifiConnect(const String &ssid, const String &pass);
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
void wifiConnectionService();
bool mediaStartTrack(const String &sourcePath, const String &displayName);
String mediaFindAdjacentTrack(const String &sourcePath, bool nextDir);
void lvglSetMediaPlayerVisible(bool visible);
void lvglRefreshMediaLayout();
void lvglRefreshMediaPlayerUi();
void networkSuspendForAudio();
bool networkResumeAfterAudio();
void lvglNavigateBackBySwipe();
bool captureScreenToJpeg(String &savedPathOut, String &errorOut);
String mqttDefaultButtonName(int idx);
String mqttButtonPayloadForIndex(int idx);
String mediaBuildListLabel(const String &name, bool isDir, size_t sizeBytes);
void displaySetAwake(bool awake);
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
void mqttPublishAction(const char *action);
void mqttPublishButtonAction(int index);
void mqttService();
bool mqttConnectNow();
void mqttPublishDiscovery();
void loadMqttConfig();
void saveMqttConfig();
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

static constexpr uint16_t LVGL_BUF_LINES = 8;
static constexpr uint16_t UI_ANIM_MS = 140;
static constexpr int16_t UI_TOP_BAR_H = 30;
static constexpr int16_t UI_CONTENT_TOP_Y = UI_TOP_BAR_H;
static constexpr int16_t UI_CONTENT_H = 320 - UI_CONTENT_TOP_Y;
static constexpr int16_t SWIPE_EDGE_START_MAX_X = 24;
static constexpr int16_t SWIPE_BACK_MIN_DX = 56;
static constexpr int16_t SWIPE_BACK_MAX_DY = 44;
static constexpr unsigned long SWIPE_BACK_MAX_MS = 700;
static lv_color_t *lvglDrawPixels = nullptr;
static uint16_t lvglBufLinesActive = 0;
static lv_disp_draw_buf_t lvglDrawBuf;
static lv_disp_drv_t lvglDispDrv;
static lv_indev_drv_t lvglIndevDrv;
static lv_indev_t *lvglTouchIndev = nullptr;
static bool lvglReady = false;
static lv_obj_t *lvglScrHome = nullptr;
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
static lv_obj_t *lvglInfoLabel = nullptr;
static lv_obj_t *lvglWifiList = nullptr;
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
static lv_obj_t *lvglMqttCountLabel = nullptr;
static lv_obj_t *lvglMqttEditLabel = nullptr;
static lv_obj_t *lvglMqttBrokerTa = nullptr;
static lv_obj_t *lvglMqttPortTa = nullptr;
static lv_obj_t *lvglMqttUserTa = nullptr;
static lv_obj_t *lvglMqttPassTa = nullptr;
static lv_obj_t *lvglMqttDiscTa = nullptr;
static lv_obj_t *lvglMqttBtnNameTa = nullptr;
static lv_obj_t *lvglMqttEnableSw = nullptr;
static lv_obj_t *lvglMqttCriticalSw = nullptr;
static lv_obj_t *lvglMqttCtrlList = nullptr;
static lv_obj_t *lvglKb = nullptr;
static lv_obj_t *lvglSnakeScoreLabel = nullptr;
static lv_obj_t *lvglTetrisScoreLabel = nullptr;
static lv_obj_t *lvglSnakeBoardObj = nullptr;
static lv_obj_t *lvglTetrisBoardObj = nullptr;
static lv_obj_t *lvglTopBarRoot = nullptr;
static constexpr uint8_t LVGL_MAX_TOP_INDICATORS = 16;
static lv_obj_t *lvglTopIndicators[LVGL_MAX_TOP_INDICATORS] = {};
static uint8_t lvglTopIndicatorCount = 0;
static int lvglMqttEditIndex = 0;
static bool lvglTouchDown = false;
static int16_t lvglLastTouchX = 120;
static int16_t lvglLastTouchY = 160;
static bool lvglSwipeTracking = false;
static int16_t lvglSwipeStartX = 0;
static int16_t lvglSwipeStartY = 0;
static int16_t lvglSwipeLastX = 0;
static int16_t lvglSwipeLastY = 0;
static unsigned long lvglSwipeStartMs = 0;
static bool lvglSwipeBackPending = false;
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
unsigned long bootDeferredStartMs = 0;
bool mdnsStarted = false;
bool webRoutesRegistered = false;
bool webServerRunning = false;
bool dnsRunning = false;
bool networkSuspendedForAudio = false;
unsigned long networkResumeLastAttemptMs = 0;
uint32_t touchI2cHzActive = TOUCH_I2C_HZ;
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

struct WifiEntry {
    String ssid;
    int32_t rssi;
    wifi_auth_mode_t auth;
};

static constexpr int MAX_WIFI_RESULTS = 20;
WifiEntry wifiEntries[MAX_WIFI_RESULTS];
int wifiCount = 0;

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
String savedStaSsid;
String savedStaPass;
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
String mqttStatusLine = "Disabled";
bool mqttDiscoveryPublished = false;
unsigned long mqttLastReconnectMs = 0;
static constexpr unsigned long MQTT_RECONNECT_MS = 5000;
static constexpr int MQTT_MAX_BUTTONS = 12;
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

static const char RECOVERY_BROWSER_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Recovery Browser</title>
  <style>
    body{font-family:system-ui,Segoe UI,Arial;background:#10131a;color:#eef;margin:0;padding:18px}
    .card{max-width:980px;margin:0 auto;background:#171c26;border:1px solid #2f3b50;border-radius:12px;padding:14px}
    .row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
    .path{font-family:Consolas,monospace;color:#bde0ff}
    #list{margin-top:10px;border:1px solid #2f3b50;border-radius:10px;max-height:62vh;overflow:auto}
    .it{display:grid;grid-template-columns:28px 1fr 110px 180px;gap:8px;align-items:center;padding:8px 10px;border-bottom:1px solid #232e40}
    .it:last-child{border-bottom:0}
    .muted{color:#9fb0cc}
    input,button{background:#0f141d;color:#eef;border:1px solid #2f3b50;border-radius:8px;padding:7px 10px}
    button{cursor:pointer}
    .a{background:#1e6fff;border-color:#1e6fff}
    .d{background:#c73a3a;border-color:#c73a3a}
  </style>
</head>
<body>
  <div class="card">
    <div class="row" style="justify-content:space-between">
      <strong>Recovery Browser</strong><span>%FIRMWARE_VERSION%</span>
    </div>
    <div class="row" style="margin-top:8px">
      <button id="upBtn">Up</button><button id="refreshBtn">Refresh</button><span class="path" id="pathLbl">/web</span>
    </div>
    <div class="row" style="margin-top:8px">
      <input id="folderName" placeholder="new-folder-name">
      <button id="mkBtn" class="a">Create Folder</button>
      <input id="uploadInput" type="file" multiple>
      <button id="upFileBtn" class="a">Upload</button>
    </div>
    <div id="status" class="muted" style="margin-top:8px">Ready.</div>
    <div id="list"></div>
  </div>
<script>
let cur = '/web';
const listEl = document.getElementById('list');
const statusEl = document.getElementById('status');
const pathLbl = document.getElementById('pathLbl');
const esc = (s)=>String(s).replace(/[&<>"']/g,m=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[m]));
const show = (m,e=false)=>{ statusEl.textContent = m; statusEl.style.color = e ? '#ff9797' : '#9fb0cc'; };
const human = (n)=>{ n=Number(n)||0; if(n<1024) return n+' B'; if(n<1048576) return (n/1024).toFixed(1)+' KB'; return (n/1048576).toFixed(1)+' MB'; };
const loadDir = ()=>{
  pathLbl.textContent = cur;
  return fetch('/fs/list?dir='+encodeURIComponent(cur),{cache:'no-store'})
    .then(r => r.json().then(j => ({r,j})))
    .then(({r,j})=>{
      if(!r.ok || (j && j.ok===false)) throw new Error((j&&j.error)||('HTTP '+r.status));
      const arr = Array.isArray(j.entries) ? j.entries : [];
      if(!arr.length){ listEl.innerHTML = '<div class="it"><div></div><div class="muted">Empty folder</div><div></div><div></div></div>'; return; }
      arr.sort((a,b)=>(b.isDir-a.isDir)||String(a.name).localeCompare(String(b.name)));
      listEl.innerHTML = arr.map(it=>{
        const icon = it.isDir ? '&#128193;' : '&#128196;';
        const action = it.isDir
          ? '<button data-open="'+esc(it.path)+'">Open</button> <button class="d" data-del="'+esc(it.path)+'">Delete</button>'
          : '<a href="/fs/download?path='+encodeURIComponent(it.path)+'"><button>Download</button></a> <button class="d" data-del="'+esc(it.path)+'">Delete</button>';
        return '<div class="it"><div>'+icon+'</div><div>'+esc(it.name)+'</div><div class="muted">'+(it.isDir?'--':human(it.size))+'</div><div>'+action+'</div></div>';
      }).join('');
    })
    .catch(e=>show(e.message||'List failed',true));
};
const doDelete = (path)=>{
  if(!confirm('Delete '+path+' ?')) return;
  const body = new URLSearchParams({path});
  return fetch('/fs/delete',{method:'POST',body})
    .then(r=>r.text().then(t=>({r,t})))
    .then(({r,t})=>{ if(!r.ok) throw new Error(t||('HTTP '+r.status)); show('Deleted: '+path); return loadDir(); })
    .catch(e=>show(e.message||'Delete failed',true));
};
const doMkdir = ()=>{
  const n = (document.getElementById('folderName').value||'').trim();
  if(!n){ show('Enter folder name', true); return; }
  const p = (cur.endsWith('/') ? cur : cur + '/') + n;
  const body = new URLSearchParams({path:p});
  return fetch('/fs/mkdir',{method:'POST',body})
    .then(r=>r.text().then(t=>({r,t})))
    .then(({r,t})=>{ if(!r.ok) throw new Error(t||('HTTP '+r.status)); show('Created: '+p); document.getElementById('folderName').value=''; return loadDir(); })
    .catch(e=>show(e.message||'Create failed',true));
};
const uploadFiles = (files)=>{
  let p = Promise.resolve();
  files.forEach((f,i)=>{
    p = p.then(()=>{
      const fd = new FormData();
      fd.append('fsFile', f, f.name);
      const url = '/fs/upload?path=' + encodeURIComponent(cur.endsWith('/') ? cur : (cur + '/'));
      return fetch(url, {method:'POST', body:fd}).then(r=>r.text().then(t=>({r,t}))).then(({r,t})=>{
        if(!r.ok) throw new Error(t||('HTTP '+r.status));
        show('Uploaded '+f.name+' ('+(i+1)+'/'+files.length+')');
      });
    });
  });
  return p.then(()=>loadDir()).catch(e=>show(e.message||'Upload failed', true));
};
document.getElementById('refreshBtn').onclick = loadDir;
document.getElementById('mkBtn').onclick = doMkdir;
document.getElementById('upFileBtn').onclick = ()=>uploadFiles(Array.from(document.getElementById('uploadInput').files || []));
document.getElementById('upBtn').onclick = ()=>{
  let p = String(cur||'/web').replace(/\/+$/,'');
  if(!p.length) p='/web';
  if(p==='/web'){ cur='/'; loadDir(); return; }
  if(p==='/'){ show('Already at root'); return; }
  const i = p.lastIndexOf('/');
  cur = (i <= 0) ? '/' : p.substring(0, i);
  if(!cur.length) cur='/';
  loadDir();
};
listEl.addEventListener('click',(e)=>{
  const o = e.target && e.target.getAttribute ? e.target.getAttribute('data-open') : null;
  const d = e.target && e.target.getAttribute ? e.target.getAttribute('data-del') : null;
  if(o){ cur=o; loadDir(); }
  if(d){ doDelete(d); }
});
loadDir();
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

void cstWriteReg(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(TOUCH_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

bool cstReadRegs(uint8_t startReg, uint8_t *buf, uint8_t len)
{
    Wire.beginTransmission(TOUCH_ADDR);
    Wire.write(startReg);
    if (Wire.endTransmission(false) != 0) return false;

    const uint8_t got = Wire.requestFrom((uint8_t)TOUCH_ADDR, (uint8_t)len);
    if (got != len) return false;

    for (uint8_t i = 0; i < len; i++) {
        buf[i] = Wire.read();
    }
    return true;
}

void cstBusBegin(uint32_t hz)
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

void cstInit()
{
    bool ok = false;
    pinMode(TOUCH_IRQ, TOUCH_USE_IRQ ? INPUT_PULLUP : INPUT);
    static const uint32_t speeds[] = {TOUCH_I2C_HZ, 100000U};
    for (uint8_t s = 0; s < (sizeof(speeds) / sizeof(speeds[0])) && !ok; s++) {
        cstBusBegin(speeds[s]);
        for (uint8_t attempt = 0; attempt < 3 && !ok; attempt++) {
            cstPulseReset();
            cstWriteReg(0xFE, 0xFF);
            delay(3);
            uint8_t probe = 0;
            ok = cstReadRegs(0x02, &probe, 1);
        }
    }
    touchReadFailStreak = 0;
    touchGhostLowStreak = 0;
    touchLastInitMs = millis();
    Serial.printf("[TOUCH] init ok=%d i2c=%lu irq=%d\n",
                  ok ? 1 : 0,
                  static_cast<unsigned long>(touchI2cHzActive),
                  digitalRead(TOUCH_IRQ));
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
    cstInit();
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
            if (touchGhostLowStreak >= 25 && static_cast<unsigned long>(millis() - touchLastInitMs) > 1000UL) cstInit();
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

    int16_t x = static_cast<int16_t>(((data[0] & 0x0F) << 8) | data[1]);
    int16_t y = static_cast<int16_t>(((data[2] & 0x0F) << 8) | data[3]);
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
            displaySetAwake(true);
            lastUserActivityMs = millis();
        }
        if (wakeTouchReleaseGuard) {
            if (!lvglTouchDown) wakeTouchReleaseGuard = false;
            lvglSwipeTracking = false;
            data->state = LV_INDEV_STATE_REL;
            data->point.x = lvglLastTouchX;
            data->point.y = lvglLastTouchY;
            return;
        }
    }
    if (wakeTouchReleaseGuard) {
        if (!lvglTouchDown) wakeTouchReleaseGuard = false;
        lvglSwipeTracking = false;
        data->state = LV_INDEV_STATE_REL;
        data->point.x = lvglLastTouchX;
        data->point.y = lvglLastTouchY;
        return;
    }
    if (lvglTouchDown) {
        if (!lvglSwipeTracking) {
            lvglSwipeTracking = true;
            lvglSwipeStartX = x;
            lvglSwipeStartY = y;
            lvglSwipeLastX = x;
            lvglSwipeLastY = y;
            lvglSwipeStartMs = millis();
        } else {
            lvglSwipeLastX = x;
            lvglSwipeLastY = y;
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
            const bool edgeStart = (lvglSwipeStartX <= SWIPE_EDGE_START_MAX_X);
            if (edgeStart && dx >= SWIPE_BACK_MIN_DX && abs(dy) <= SWIPE_BACK_MAX_DY && dt <= SWIPE_BACK_MAX_MS) {
                lvglSwipeBackPending = true;
            }
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
    if (networkSuspendedForAudio) return WL_DISCONNECTED;
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

    uint8_t light = lightPercent;
    if (light > 100) light = 100;
    int lightFillW = (static_cast<int>(light) * 32) / 100;
    if (light > 0 && lightFillW < 1) lightFillW = 1;
    lv_color_t lightCol = lv_color_hex(0x8D7A3A);
    if (light >= 33) lightCol = lv_color_hex(0xF4B942);
    if (light >= 66) lightCol = lv_color_hex(0xFFD166);

    static const int antHeights[4] = {6, 10, 16, 22};
    const int antX = ox + 10;
    for (int i = 0; i < 4; i++) {
        const bool on = connected && i < bars;
        lv_color_t col = on ? lv_color_hex(0x79E28A) : lv_color_hex(0x30404A);
        lvglDrawRectSolid(drawCtx, antX + (i * 8), topY + 23 - antHeights[i], 4, antHeights[i], col, on ? LV_OPA_COVER : LV_OPA_70);
    }
    if (!connected) {
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

    lv_draw_rect_dsc_t frame;
    lv_draw_rect_dsc_init(&frame);
    frame.bg_opa = LV_OPA_TRANSP;
    frame.border_width = 1;
    frame.border_color = lv_color_hex(0x8A97A4);
    frame.radius = 2;
    const int battX = ox + ow - 54;
    lv_area_t batA;
    batA.x1 = static_cast<lv_coord_t>(battX);
    batA.y1 = static_cast<lv_coord_t>(topY + 2);
    batA.x2 = static_cast<lv_coord_t>(battX + 35);
    batA.y2 = static_cast<lv_coord_t>(topY + 21);
    lv_draw_rect(drawCtx, &frame, &batA);
    lvglDrawRectSolid(drawCtx, battX + 36, topY + 8, 6, 8, batteryCharging ? battCol : lv_color_hex(0x8A97A4));
    if (battFillW > 0) lvglDrawRectSolid(drawCtx, battX + 3, topY + 5, battFillW, 14, battCol);

    const int lightX = ox + (ow / 2) - 16;
    lvglDrawRectSolid(drawCtx, lightX + 10, topY + 0, 12, 12, lightCol);
    lvglDrawRectSolid(drawCtx, lightX + 0, topY + 18, 32, 8, lv_color_hex(0x2D3843));
    if (lightFillW > 0) lvglDrawRectSolid(drawCtx, lightX + 0, topY + 18, lightFillW, 8, lightCol);
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
    lv_obj_set_style_bg_color(btn, color, 0);
    lv_obj_set_style_border_width(btn, 0, 0);
    lv_obj_set_style_shadow_width(btn, compactList ? 0 : 14, 0);
    lv_obj_set_style_shadow_color(btn, lv_color_hex(0x000000), 0);
    lv_obj_set_style_shadow_opa(btn, compactList ? LV_OPA_TRANSP : LV_OPA_40, 0);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, user);
    lv_obj_t *lbl = lv_label_create(btn);
    if (lbl) {
        lv_label_set_text(lbl, txt);
        lv_obj_center(lbl);
    }
    return btn;
}

void lvglRefreshWifiList();
void lvglRefreshMediaList();
void lvglMediaPlayStopEvent(lv_event_t *e);
void lvglMediaPrevTrackEvent(lv_event_t *e);
void lvglMediaNextTrackEvent(lv_event_t *e);
void lvglMediaVolumeEvent(lv_event_t *e);
void lvglMediaVolumeStepEvent(lv_event_t *e);
void lvglRefreshMqttConfigUi();
void lvglRefreshMqttControlsUi();
void lvglRefreshSnakeBoard();
void lvglRefreshTetrisBoard();

inline void lvglLoadScreen(lv_obj_t *target, lv_scr_load_anim_t anim)
{
    if (!target) return;
    lv_scr_load_anim(target, anim, UI_ANIM_MS, 0, false);
}

void lvglNavigateBackBySwipe()
{
    const UiScreen prev = uiScreen;
    if (!lvglReady) return;
    switch (uiScreen) {
        case UI_WIFI_LIST:
        case UI_MEDIA:
        case UI_INFO:
        case UI_GAMES:
        case UI_CONFIG:
            uiScreen = UI_HOME;
            lvglLoadScreen(lvglScrHome, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
            break;
        case UI_GAME_SNAKE:
        case UI_GAME_TETRIS:
            uiScreen = UI_GAMES;
            lvglLoadScreen(lvglScrGames, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
            break;
        case UI_CONFIG_MQTT_CONFIG:
        case UI_CONFIG_MQTT_CONTROLS:
            uiScreen = UI_CONFIG;
            lvglLoadScreen(lvglScrConfig, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
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
    lv_obj_t *target = reinterpret_cast<lv_obj_t *>(lv_event_get_user_data(e));
    if (!target) return;
    if (target == lvglScrWifi) {
        uiScreen = UI_WIFI_LIST;
        refreshWifiScan();
        lvglRefreshWifiList();
    }
    else if (target == lvglScrMedia) {
        uiScreen = UI_MEDIA;
        lvglQueueMediaRefresh();
    }
    else if (target == lvglScrInfo) uiScreen = UI_INFO;
    else if (target == lvglScrGames) uiScreen = UI_GAMES;
    else if (target == lvglScrConfig) uiScreen = UI_CONFIG;
    else uiScreen = UI_HOME;
    lvglLoadScreen(target, LV_SCR_LOAD_ANIM_MOVE_LEFT);
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

void lvglWifiEntryEvent(lv_event_t *e)
{
    const int idx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (idx < 0 || idx >= wifiCount) return;
    if (wifiEntries[idx].auth != WIFI_AUTH_OPEN) {
        uiStatusLine = "Secured AP: connect via /wifi";
        lvglSyncStatusLine();
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

void lvglRefreshInfoPanel()
{
    if (!lvglInfoLabel) return;
    sampleTopIndicators();
    lvglRefreshTopIndicators();
    bool connected = wifiConnectedSafe();
    String ssid = connected ? wifiSsidSafe() : String("disconnected");
    String ip = connected ? wifiIpSafe() : String("-");
    const int rssi = connected ? static_cast<int>(wifiRssiSafe()) : 0;
    char buf[400];
    snprintf(
        buf,
        sizeof(buf),
        "FW: %s\nAP: %s\nSTA: %s\nIP: %s\nRSSI: %d\nBattery: %u%% (%.2fV)\nCharging: %s\nLight: %u%%\nSD mounted: %s\nMedia: %s",
        FW_VERSION,
        AP_SSID,
        ssid.c_str(),
        ip.c_str(),
        rssi,
        batteryPercent,
        batteryVoltage,
        batteryCharging ? "yes" : "no",
        lightPercent,
        sdMounted ? "yes" : "no",
        mediaIsPlaying ? mediaNowPlaying.c_str() : "stopped"
    );
    lv_label_set_text(lvglInfoLabel, buf);
}

void lvglRefreshWifiList()
{
    if (!lvglWifiList) return;
    lv_obj_clean(lvglWifiList);

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
    if (wifiCount == 0) {
        lv_obj_t *lbl = lv_label_create(lvglWifiList);
        lv_label_set_text(lbl, "No scan yet. Tap Rescan.");
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xC8CED6), 0);
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
                if (code != LV_EVENT_READY && code != LV_EVENT_CANCEL) return;
                if (lvglKb) {
                    lv_keyboard_set_textarea(lvglKb, nullptr);
                    lv_obj_add_flag(lvglKb, LV_OBJ_FLAG_HIDDEN);
                }
            },
            LV_EVENT_ALL,
            nullptr
        );
    }
    lv_keyboard_set_textarea(lvglKb, ta);
    lv_obj_clear_flag(lvglKb, LV_OBJ_FLAG_HIDDEN);
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
    mqttConnectNow();
    lv_label_set_text_fmt(lvglMqttStatusLabel, "MQTT: %s", mqttStatusLine.c_str());
}

void lvglMqttSaveEvent(lv_event_t *e)
{
    (void)e;
    mqttCfg.enabled = lv_obj_has_state(lvglMqttEnableSw, LV_STATE_CHECKED);
    mqttCfg.broker = lv_textarea_get_text(lvglMqttBrokerTa);
    mqttCfg.username = lv_textarea_get_text(lvglMqttUserTa);
    mqttCfg.password = lv_textarea_get_text(lvglMqttPassTa);
    mqttCfg.discoveryPrefix = lv_textarea_get_text(lvglMqttDiscTa);

    int p = atoi(lv_textarea_get_text(lvglMqttPortTa));
    if (p <= 0 || p > 65535) p = 1883;
    mqttCfg.port = static_cast<uint16_t>(p);
    lv_textarea_set_text(lvglMqttPortTa, String(mqttCfg.port).c_str());

    saveMqttConfig();
    mqttStatusLine = mqttCfg.enabled ? "Enabled" : "Disabled";
    if (mqttCfg.enabled) mqttConnectNow();
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
    lvglRefreshMqttConfigUi();
}

void lvglMqttEditNextEvent(lv_event_t *e)
{
    (void)e;
    if (lvglMqttEditIndex + 1 < mqttButtonCount) lvglMqttEditIndex++;
    lvglRefreshMqttConfigUi();
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
    lvglRefreshMqttConfigUi();
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
    if (!lvglMqttCountLabel) return;
    lv_label_set_text_fmt(lvglMqttCountLabel, "Buttons: %d", mqttButtonCount);
    if (lvglMqttEditIndex < 0) lvglMqttEditIndex = 0;
    if (lvglMqttEditIndex >= mqttButtonCount) lvglMqttEditIndex = mqttButtonCount - 1;
    lv_label_set_text_fmt(lvglMqttEditLabel, "Edit #%d", lvglMqttEditIndex + 1);
    lv_obj_add_state(lvglMqttEnableSw, mqttCfg.enabled ? LV_STATE_CHECKED : 0);
    lv_obj_clear_state(lvglMqttEnableSw, mqttCfg.enabled ? 0 : LV_STATE_CHECKED);
    lv_textarea_set_text(lvglMqttBrokerTa, mqttCfg.broker.c_str());
    lv_textarea_set_text(lvglMqttPortTa, String(mqttCfg.port).c_str());
    lv_textarea_set_text(lvglMqttUserTa, mqttCfg.username.c_str());
    lv_textarea_set_text(lvglMqttPassTa, mqttCfg.password.c_str());
    lv_textarea_set_text(lvglMqttDiscTa, mqttCfg.discoveryPrefix.c_str());
    lv_textarea_set_text(lvglMqttBtnNameTa, mqttButtonNames[lvglMqttEditIndex].c_str());
    lv_obj_add_state(lvglMqttCriticalSw, mqttButtonCritical[lvglMqttEditIndex] ? LV_STATE_CHECKED : 0);
    lv_obj_clear_state(lvglMqttCriticalSw, mqttButtonCritical[lvglMqttEditIndex] ? 0 : LV_STATE_CHECKED);
    lv_label_set_text_fmt(lvglMqttStatusLabel, "MQTT: %s", mqttStatusLine.c_str());
}

void lvglRefreshMqttControlsUi()
{
    if (!lvglMqttCtrlList) return;
    lv_obj_clean(lvglMqttCtrlList);
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
    uiScreen = UI_GAMES;
    lvglLoadScreen(lvglScrGames, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
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
    uiScreen = UI_GAMES;
    lvglLoadScreen(lvglScrGames, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
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
    snakeResetGame();
    uiScreen = UI_GAME_SNAKE;
    lvglRefreshSnakeBoard();
    lvglLoadScreen(lvglScrSnake, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglOpenTetrisEvent(lv_event_t *e)
{
    (void)e;
    tetrisResetGame();
    uiScreen = UI_GAME_TETRIS;
    lvglRefreshTetrisBoard();
    lvglLoadScreen(lvglScrTetris, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglOpenMqttCfgEvent(lv_event_t *e)
{
    (void)e;
    uiScreen = UI_CONFIG_MQTT_CONFIG;
    lvglRefreshMqttConfigUi();
    lvglLoadScreen(lvglScrMqttCfg, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglOpenMqttCtrlEvent(lv_event_t *e)
{
    (void)e;
    uiScreen = UI_CONFIG_MQTT_CONTROLS;
    lvglRefreshMqttControlsUi();
    lvglLoadScreen(lvglScrMqttCtrl, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglBackToConfigEvent(lv_event_t *e)
{
    (void)e;
    uiScreen = UI_CONFIG;
    lvglLoadScreen(lvglScrConfig, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
}

void lvglStyleHintEvent(lv_event_t *e)
{
    (void)e;
    lvglStatusPush("Style tuning now handled by LVGL theme");
}

void lvglBuildUi()
{
    auto makeSmallBtn = [](lv_obj_t *parent, const char *txt, int w, int h, lv_color_t col, lv_event_cb_t cb, void *ud = nullptr) -> lv_obj_t * {
        if (!parent) return nullptr;
        lv_obj_t *b = lv_btn_create(parent);
        if (!b) {
            Serial.printf("[LVGL] alloc failed: small button '%s'\n", txt ? txt : "?");
            return nullptr;
        }
        lv_obj_set_size(b, w, h);
        lv_obj_set_style_radius(b, 8, 0);
        lv_obj_set_style_bg_color(b, col, 0);
        lv_obj_set_style_border_width(b, 0, 0);
        lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
        lv_obj_t *l = lv_label_create(b);
        if (!l) return b;
        lv_label_set_text(l, txt);
        lv_obj_center(l);
        return b;
    };

    lvglScrWifi = lvglCreateScreenBase("WiFi", true);
    lv_obj_t *wifiOps = lv_obj_create(lvglScrWifi);
    lv_obj_set_size(wifiOps, lv_pct(100), 50);
    lv_obj_align(wifiOps, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
    lv_obj_set_style_bg_opa(wifiOps, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(wifiOps, 0, 0);
    lv_obj_set_style_pad_all(wifiOps, 8, 0);
    lv_obj_set_style_pad_column(wifiOps, 8, 0);
    lv_obj_set_flex_flow(wifiOps, LV_FLEX_FLOW_ROW);
    lv_obj_set_scrollbar_mode(wifiOps, LV_SCROLLBAR_MODE_OFF);
    lvglCreateMenuButton(wifiOps, "Rescan", lv_color_hex(0x2F6D86), lvglWifiRescanEvent, nullptr);
    lvglWifiList = lv_obj_create(lvglScrWifi);
    lv_obj_set_size(lvglWifiList, lv_pct(100), 240);
    lv_obj_align(lvglWifiList, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(lvglWifiList, lv_color_hex(0x111922), 0);
    lv_obj_set_style_border_width(lvglWifiList, 0, 0);
    lv_obj_set_style_pad_all(lvglWifiList, 8, 0);
    lv_obj_set_style_pad_row(lvglWifiList, 6, 0);
    lv_obj_set_flex_flow(lvglWifiList, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(lvglWifiList, LV_DIR_VER);

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
    lv_obj_add_flag(lvglMediaPlayerPanel, LV_OBJ_FLAG_HIDDEN);
    lvglMediaPlayerVisible = false;
    lvglRefreshMediaLayout();

    lvglScrInfo = lvglCreateScreenBase("Info", true);
    lvglInfoLabel = lv_label_create(lvglScrInfo);
    lv_obj_set_width(lvglInfoLabel, lv_pct(94));
    lv_obj_align(lvglInfoLabel, LV_ALIGN_TOP_LEFT, 10, UI_CONTENT_TOP_Y + 8);
    lv_obj_set_style_text_color(lvglInfoLabel, lv_color_hex(0xE5ECF3), 0);
    lv_label_set_long_mode(lvglInfoLabel, LV_LABEL_LONG_WRAP);

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
    } else {
        Serial.println("[LVGL] alloc failed: snake board");
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
    } else {
        Serial.println("[LVGL] alloc failed: tetris board");
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

    lvglScrGames = lvglCreateScreenBase("Games", true);
    lv_obj_t *gamesWrap = lv_obj_create(lvglScrGames);
    lv_obj_set_size(gamesWrap, lv_pct(100), UI_CONTENT_H);
    lv_obj_align(gamesWrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
    lv_obj_set_style_bg_opa(gamesWrap, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(gamesWrap, 0, 0);
    lv_obj_set_style_pad_all(gamesWrap, 10, 0);
    lv_obj_set_style_pad_row(gamesWrap, 10, 0);
    lv_obj_set_flex_flow(gamesWrap, LV_FLEX_FLOW_COLUMN);
    lvglCreateMenuButton(gamesWrap, "Snake", lv_color_hex(0x3A8F4B), lvglOpenSnakeEvent, nullptr);
    lvglCreateMenuButton(gamesWrap, "Tetris", lv_color_hex(0x376B93), lvglOpenTetrisEvent, nullptr);

    lvglScrMqttCfg = lvglCreateScreenBase("MQTT Config", false);
    lv_obj_t *mqWrap = lv_obj_create(lvglScrMqttCfg);
    lv_obj_set_size(mqWrap, lv_pct(100), UI_CONTENT_H);
    lv_obj_align(mqWrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
    lv_obj_set_style_bg_color(mqWrap, lv_color_hex(0x111922), 0);
    lv_obj_set_style_border_width(mqWrap, 0, 0);
    lv_obj_set_style_pad_all(mqWrap, 8, 0);
    lv_obj_set_style_pad_row(mqWrap, 6, 0);
    lv_obj_set_flex_flow(mqWrap, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(mqWrap, LV_DIR_VER);
    lv_obj_t *enableRow = lv_obj_create(mqWrap);
    lv_obj_set_size(enableRow, lv_pct(100), 34);
    lv_obj_set_style_bg_opa(enableRow, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(enableRow, 0, 0);
    lv_obj_t *enableLbl = lv_label_create(enableRow);
    lv_label_set_text(enableLbl, "Enabled");
    lv_obj_align(enableLbl, LV_ALIGN_LEFT_MID, 0, 0);
    lvglMqttEnableSw = lv_switch_create(enableRow);
    lv_obj_align(lvglMqttEnableSw, LV_ALIGN_RIGHT_MID, 0, 0);
    auto addTa = [&](const char *ph, bool pass, bool numeric) -> lv_obj_t * {
        lv_obj_t *ta = lv_textarea_create(mqWrap);
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
    lvglMqttPassTa = addTa("Password", true, false);
    lvglMqttDiscTa = addTa("Discovery prefix", false, false);
    lv_obj_t *countRow = lv_obj_create(mqWrap);
    lv_obj_set_size(countRow, lv_pct(100), 34);
    lv_obj_set_style_bg_opa(countRow, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(countRow, 0, 0);
    makeSmallBtn(countRow, "-", 34, 26, lv_color_hex(0x6B4A2A), lvglMqttCountMinusEvent);
    lv_obj_align(lv_obj_get_child(countRow, 0), LV_ALIGN_LEFT_MID, 0, 0);
    makeSmallBtn(countRow, "+", 34, 26, lv_color_hex(0x3A7A3A), lvglMqttCountPlusEvent);
    lv_obj_align(lv_obj_get_child(countRow, 1), LV_ALIGN_RIGHT_MID, 0, 0);
    lvglMqttCountLabel = lv_label_create(countRow);
    lv_obj_center(lvglMqttCountLabel);
    lv_obj_t *editRow = lv_obj_create(mqWrap);
    lv_obj_set_size(editRow, lv_pct(100), 34);
    lv_obj_set_style_bg_opa(editRow, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(editRow, 0, 0);
    makeSmallBtn(editRow, "<", 34, 26, lv_color_hex(0x2F6D86), lvglMqttEditPrevEvent);
    lv_obj_align(lv_obj_get_child(editRow, 0), LV_ALIGN_LEFT_MID, 0, 0);
    makeSmallBtn(editRow, ">", 34, 26, lv_color_hex(0x2F6D86), lvglMqttEditNextEvent);
    lv_obj_align(lv_obj_get_child(editRow, 1), LV_ALIGN_RIGHT_MID, 0, 0);
    lvglMqttEditLabel = lv_label_create(editRow);
    lv_obj_center(lvglMqttEditLabel);
    lvglMqttBtnNameTa = addTa("Button name", false, false);
    lv_obj_t *critRow = lv_obj_create(mqWrap);
    lv_obj_set_size(critRow, lv_pct(100), 34);
    lv_obj_set_style_bg_opa(critRow, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(critRow, 0, 0);
    lv_obj_t *critLbl = lv_label_create(critRow);
    lv_label_set_text(critLbl, "Critical");
    lv_obj_align(critLbl, LV_ALIGN_LEFT_MID, 0, 0);
    lvglMqttCriticalSw = lv_switch_create(critRow);
    lv_obj_align(lvglMqttCriticalSw, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_t *actRow = lv_obj_create(mqWrap);
    lv_obj_set_size(actRow, lv_pct(100), 40);
    lv_obj_set_style_bg_opa(actRow, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(actRow, 0, 0);
    lv_obj_set_style_pad_column(actRow, 6, 0);
    lv_obj_set_flex_flow(actRow, LV_FLEX_FLOW_ROW_WRAP);
    makeSmallBtn(actRow, "Apply Btn", 72, 30, lv_color_hex(0x6D4B9A), lvglMqttApplyBtnEvent);
    makeSmallBtn(actRow, "Save", 56, 30, lv_color_hex(0x3A7A3A), lvglMqttSaveEvent);
    makeSmallBtn(actRow, "Connect", 70, 30, lv_color_hex(0x2F6D86), lvglMqttConnectEvent);
    makeSmallBtn(actRow, "Discover", 76, 30, lv_color_hex(0x2F6D86), lvglMqttPublishDiscoveryEvent);
    lvglMqttStatusLabel = lv_label_create(mqWrap);
    lv_obj_set_width(lvglMqttStatusLabel, lv_pct(100));
    lv_label_set_long_mode(lvglMqttStatusLabel, LV_LABEL_LONG_WRAP);

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

    lvglScrConfig = lvglCreateScreenBase("Config", true);
    lv_obj_t *cfgWrap = lv_obj_create(lvglScrConfig);
    lv_obj_set_size(cfgWrap, lv_pct(100), UI_CONTENT_H);
    lv_obj_align(cfgWrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
    lv_obj_set_style_bg_opa(cfgWrap, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(cfgWrap, 0, 0);
    lv_obj_set_style_pad_all(cfgWrap, 10, 0);
    lv_obj_set_style_pad_row(cfgWrap, 10, 0);
    lv_obj_set_flex_flow(cfgWrap, LV_FLEX_FLOW_COLUMN);
    lvglCreateMenuButton(cfgWrap, "Style", lv_color_hex(0x2D6D8E), lvglStyleHintEvent, nullptr);
    lvglCreateMenuButton(cfgWrap, "MQTT Config", lv_color_hex(0x6D4B9A), lvglOpenMqttCfgEvent, nullptr);
    lvglCreateMenuButton(cfgWrap, "MQTT Controls", lv_color_hex(0x2D6D8E), lvglOpenMqttCtrlEvent, nullptr);
    lvglCreateMenuButton(cfgWrap, "Screenshot", lv_color_hex(0x6B5B2A), lvglScreenshotEvent, nullptr);

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
    lvglCreateMenuButton(homeWrap, "WiFi", lv_color_hex(0x3A8F4B), lvglHomeNavEvent, lvglScrWifi);
    lvglCreateMenuButton(homeWrap, "Media", lv_color_hex(0x376B93), lvglHomeNavEvent, lvglScrMedia);
    lvglCreateMenuButton(homeWrap, "Info", lv_color_hex(0x7750A0), lvglHomeNavEvent, lvglScrInfo);
    lvglCreateMenuButton(homeWrap, "Games", lv_color_hex(0x2B7D7D), lvglHomeNavEvent, lvglScrGames);
    lvglCreateMenuButton(homeWrap, "Config", lv_color_hex(0x925A73), lvglHomeNavEvent, lvglScrConfig);
    lvglCreateMenuButton(homeWrap, "Web Recovery", lv_color_hex(0xA66A2A), lvglRecoveryHintEvent, nullptr);

    if (lvglMediaList) {
        lv_obj_t *lbl = lv_label_create(lvglMediaList);
        lv_label_set_text(lbl, "Open Media screen to scan SD");
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xC8CED6), 0);
    }
    lvglRefreshMediaPlayerUi();
    lvglRefreshInfoPanel();
    lvglRefreshMqttConfigUi();
    lvglRefreshMqttControlsUi();
    lvglRefreshTopIndicators();
    snakeResetGame();
    tetrisResetGame();
    lvglRefreshSnakeBoard();
    lvglRefreshTetrisBoard();
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
        tft.drawString("LVGL buffer alloc failed", 8, 120, 2);
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
        lvglNavigateBackBySwipe();
    }
    if (lvglMediaRefreshPending) {
        lvglMediaRefreshPending = false;
        lvglRefreshMediaList();
    }
    const unsigned long now = millis();
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

void animateChargingBeforeSleep()
{
    digitalWrite(TFT_BL, HIGH);
    const int16_t x = 40, y = 110, w = 150, h = 70;
    tft.fillScreen(TFT_BLACK);
    drawLargeBatteryFrame(x, y, w, h);
    drawChargingCableDecor(x, y, w, h, TFT_DARKGREY);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawCentreString("Charging", 120, 70, 4);
    for (uint8_t cycle = 0; cycle < CHARGE_ANIM_CYCLES; cycle++) {
        for (int p = 0; p <= 100; p += 3) {
            tft.fillRect(x + 2, y + 2, w - 4, h - 4, TFT_BLACK);
            int innerW = ((w - 4) * p) / 100;
            tft.fillRoundRect(x + 2, y + 2, innerW, h - 4, 5, TFT_GREEN);
            drawChargingCableDecor(x, y, w, h, scaleColor565(TFT_GREEN, 180));
            delay(55);
        }
    }
}

const char *authName(wifi_auth_mode_t auth)
{
    return auth == WIFI_AUTH_OPEN ? "Open" : "Secured";
}

void refreshWifiScan()
{
    WiFi.scanDelete();
    wifiCount = 0;
    int n = WiFi.scanNetworks(false, true);
    if (n <= 0) {
        uiStatusLine = "No networks found";
        return;
    }
    for (int i = 0; i < n && wifiCount < MAX_WIFI_RESULTS; i++) {
        wifiEntries[wifiCount].ssid = WiFi.SSID(i);
        wifiEntries[wifiCount].rssi = WiFi.RSSI(i);
        wifiEntries[wifiCount].auth = WiFi.encryptionType(i);
        wifiCount++;
    }
    uiStatusLine = "Scan complete: " + String(wifiCount);
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
    digitalWrite(TFT_BL, awake ? HIGH : LOW);
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
    wifiPrefs.end();
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
    if (networkSuspendedForAudio) return;
    if (apModeActive) return;
    if (WiFi.getMode() != WIFI_AP_STA) WiFi.mode(WIFI_AP_STA);
    if (!WiFi.softAP(AP_SSID, AP_PASS)) {
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
    if (networkSuspendedForAudio || !wifiHasStaTarget()) return;
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
        return false;
    }
    if (networkSuspendedForAudio || !wifiConnectedSafe()) {
        mqttStatusLine = "No WiFi";
        return false;
    }
    mqttClient.setServer(mqttCfg.broker.c_str(), mqttCfg.port);
    const String availabilityTopic = "esp32/remote/" + mqttHwId + "/availability";
    bool ok = false;
    if (mqttCfg.username.length()) {
        ok = mqttClient.connect(mqttClientId.c_str(), mqttCfg.username.c_str(), mqttCfg.password.c_str(), availabilityTopic.c_str(), 0, true, "offline");
    } else {
        ok = mqttClient.connect(mqttClientId.c_str(), availabilityTopic.c_str(), 0, true, "offline");
    }

    if (!ok) {
        mqttStatusLine = "Connect failed rc=" + String(mqttClient.state());
        return false;
    }

    mqttClient.publish(availabilityTopic.c_str(), "online", true);
    mqttDiscoveryPublished = false;
    mqttStatusLine = "Connected";
    return true;
}

void mqttPublishDiscovery()
{
    if (!mqttClient.connected()) return;
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
        dev["model"] = "ESP32-2432S024C Touch Remote";
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

void mqttPublishButtonAction(int index)
{
    if (index < 0 || index >= mqttButtonCount) return;
    String payload = mqttButtonPayloadForIndex(index);
    mqttPublishAction(payload.c_str());
}

void mqttService()
{
    if (networkSuspendedForAudio) {
        if (mqttClient.connected()) mqttClient.disconnect();
        mqttStatusLine = "No WiFi";
        return;
    }
    if (!mqttCfg.enabled) {
        if (mqttClient.connected()) mqttClient.disconnect();
        return;
    }
    if (!wifiConnectedSafe()) {
        if (mqttClient.connected()) mqttClient.disconnect();
        mqttStatusLine = "No WiFi";
        return;
    }
    if (!mqttClient.connected()) {
        if (millis() - mqttLastReconnectMs >= MQTT_RECONNECT_MS) {
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
    pendingSaveSsid = ssid;
    pendingSavePass = pass;
    pendingSaveCreds = true;
    ensureApOnline("manual_connect");
    beginStaConnectAttempt("manual_connect");
    uiScreen = UI_HOME;
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

bool pathAllowedInRecovery(const String &path)
{
    return pathUnderWeb(path) || pathUnderScreenshots(path);
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
    http.addHeader("User-Agent", "ESP32-2432S024C");

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
    String html = FPSTR(RECOVERY_BROWSER_HTML);
    html.replace("%FIRMWARE_VERSION%", FW_VERSION);
    request->send(200, "text/html", html);
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
        doc["repo"] = "elik745i/miniexco.v2";

        int ghStatus = -1;
        const String body = httpsGetText("https://api.github.com/repos/elik745i/miniexco.v2/releases/latest", &ghStatus);
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
        doc["RecordTelemetry"] = 0;
        doc["SystemSounds"] = 1;
        doc["BluepadEnabled"] = 0;
        doc["GamepadEnabled"] = 0;
        doc["SystemVolume"] = mediaVolumePercent;
        doc["WsRebootOnDisconnect"] = 0;
        doc["SerialLogRateMs"] = 40;
        doc["SerialLogKeepLines"] = 200;
        doc["IndicatorsVisible"] = 1;
        doc["ImuVisible"] = 1;
        doc["MediaVisible"] = 1;
        doc["PathVisible"] = 1;
        doc["Model3DVisible"] = 1;
        doc["SerialVisible"] = 1;
        doc["ModelRotX"] = 0;
        doc["ModelRotY"] = 0;
        doc["ModelRotZ"] = 0;
        doc["ModelDirX"] = 1;
        doc["ModelDirY"] = 1;
        doc["ModelDirZ"] = 1;
        doc["ModelAxisX"] = "x";
        doc["ModelAxisY"] = "y";
        doc["ModelAxisZ"] = "z";
        doc["ViewOverlapFx"] = 1;
        doc["ViewSnapFx"] = 1;
        doc["ViewGravityFx"] = 1;
        doc["ViewGravityStr"] = 55;

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
    if (networkSuspendedForAudio) return;

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
}

void setup()
{
    sdMutex = xSemaphoreCreateMutex();
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
    pinMode(RGB_PIN_R, OUTPUT);
    pinMode(RGB_PIN_G, OUTPUT);
    pinMode(RGB_PIN_B, OUTPUT);
    pinMode(TOUCH_IRQ, TOUCH_USE_IRQ ? INPUT_PULLUP : INPUT);
    rgbApplyNow(false, false, false);
    analogReadResolution(12);
    analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
    analogSetPinAttenuation(LIGHT_ADC_PIN, ADC_11db);
    randomSeed(static_cast<unsigned long>(esp_random()));
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
    Serial.println("ESP32-2432S024C UI + fallback file manager");

    tft.init();
    tft.setRotation(TFT_ROTATION);
    tft.setTextFont(2);

    Serial.println("[BOOT] touch pre-init");
    cstInit();
    lvglInitUi();
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
    Serial.printf("[TOUCH] mode=%s irq_pin=%d i2c=(sda=%d,scl=%d,rst=%d)\n",
                  TOUCH_USE_IRQ ? "irq+poll" : "poll-only",
                  TOUCH_IRQ, TOUCH_SDA, TOUCH_SCL, TOUCH_RST);
}

void loop()
{
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
    mqttService();
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
        if (batteryCharging) animateChargingBeforeSleep();
        displaySetAwake(false);
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
    delay(1);
}
