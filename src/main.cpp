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
#include <HTTPClient.h>
#include <Update.h>
#include <WiFiClientSecure.h>
#include <esp_ota_ops.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <driver/gpio.h>
#include <math.h>
#include <new>
#include <string.h>
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <sodium.h>

#include "generated/img_airplane_mode_icon.h"
#include "generated/img_chat_small_icon.h"
#include "generated/img_config_small_icon.h"
#include "generated/img_games_small_icon.h"
#include "generated/img_info_small_icon.h"
#include "generated/img_ap_small_icon.h"
#include "generated/img_music_small_icon.h"
#include "generated/img_mqtt_controls_small_icon.h"
#include "generated/img_mqtt_conf_small_icon.h"
#include "generated/img_mqtt_small_icon.h"
#include "generated/img_ota_small_icon.h"
#include "generated/img_auto_small_icon.h"
#include "generated/img_disch_small_icon.h"
#include "generated/img_full_small_icon.h"
#include "generated/img_power_small_icon.h"
#include "generated/img_radio_small_icon.h"
#include "generated/img_silent_small_icon.h"
#include "generated/img_speaker_small_icon.h"
#include "generated/img_styles_small_icon.h"
#include "generated/img_vibro_small_icon.h"
#include "generated/img_wifi_small_icon.h"
#include "generated/recovery_browser_asset.h"

enum TouchControllerType : uint8_t {
    TOUCH_CTRL_CST820 = 0,
    TOUCH_CTRL_GT911 = 1,
};

enum UiButtonStyleMode : uint8_t {
    UI_BUTTON_STYLE_FLAT = 0,
    UI_BUTTON_STYLE_3D = 1,
    UI_BUTTON_STYLE_BLACK = 2,
};

enum TopBarCenterMode : uint8_t {
    TOP_BAR_CENTER_NAME = 0,
    TOP_BAR_CENTER_TIME = 1,
};

// App-wide hardware/runtime constants moved to app/constants.inc.
#include "app/constants.inc"

struct RadioTxState {
    bool waitingAck = false;
    String peerKey;
    String messageId;
    unsigned long sentAtMs = 0;
    unsigned long guardUntilMs = 0;
    uint8_t retries = 0;
};

static RadioTxState radioTxState;

struct ChatRecentMessageIdEntry {
    String peerKey;
    String messageId;
};

static ChatRecentMessageIdEntry chatRecentMessageIds[CHAT_RECENT_MESSAGE_ID_CACHE];
static uint8_t chatRecentMessageIdHead = 0;
static bool chatPendingOutboxDirty = false;
static unsigned long chatPendingOutboxFlushAfterMs = 0;
static unsigned long radioLastTxLeadMs = 0;

#if defined(BOARD_ESP32S3_3248S035_N16R8)
static constexpr bool AUDIO_BACKEND_SUPPORTED = false;
static constexpr int I2S_SPK_PIN = 18;
#else
static constexpr bool AUDIO_BACKEND_SUPPORTED = true;
static constexpr int I2S_SPK_PIN = 26;
#endif
static constexpr uint8_t AUDIO_I2S_PORT = I2S_NUM_0;
static constexpr uint8_t AUDIO_VOLUME_TARGET = 21;
static constexpr bool AUDIO_FORCE_MONO_INTERNAL_DAC = true;
static constexpr float AUDIO_VOLUME_CURVE_EXPONENT = 2.0f;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
static constexpr uint8_t VIBRATION_LEDC_CHANNEL = 6;
static constexpr uint16_t VIBRATION_PWM_HZ = 220;
static constexpr uint8_t VIBRATION_PWM_RES_BITS = 8;
static constexpr unsigned long TOUCH_VIBRATION_PULSE_MS = 5UL;
static constexpr uint32_t TOUCH_VIBRATION_DUTY = 220U;
#endif
static constexpr uint8_t CHAT_NOTIFY_LEDC_CHANNEL = 7;
static constexpr uint16_t CHAT_NOTIFY_FREQ_PRIMARY = 1760;
static constexpr uint16_t TOUCH_TICK_FREQ = 3200;
static constexpr uint32_t TOUCH_TICK_DUTY = 64U;
static constexpr unsigned long TOUCH_TICK_MS = 4UL;
// Legacy unused chat notification pattern constants were moved to app/garbage.inc:
// CHAT_NOTIFY_FREQ_SECONDARY, CHAT_NOTIFY_BEEP1_MS, CHAT_NOTIFY_GAP_MS, CHAT_NOTIFY_BEEP2_MS
static constexpr unsigned long CHAT_NOTIFY_REPEAT_GAP_MS = 140UL;
static constexpr uint8_t CHAT_NOTIFY_QUEUE_MAX = 3;
#if !defined(BOARD_ESP32S3_3248S035_N16R8)
// ESP32-audioI2S uses LEFT_EN (value 2) for DAC2 on GPIO26.
// Keep TOUCH_RST on GPIO25 untouched (DAC1 / RIGHT_EN must stay disabled).
static constexpr uint8_t AUDIO_INTERNAL_DAC_CHANNEL = static_cast<uint8_t>(I2S_DAC_CHANNEL_LEFT_EN);
static_assert(I2S_DAC_CHANNEL_LEFT_EN == 2, "Unexpected DAC enum mapping: GPIO26 output would be unsafe");
#endif
static constexpr int AUDIO_INPUT_RAM_BUFFER_BYTES = 2048;
static constexpr uint32_t AUDIO_PREEMPTIVE_NET_SUSPEND_LARGEST_8BIT = 28000U;
static constexpr unsigned long AUDIO_NETWORK_RESUME_RETRY_MS = 2000UL;
static constexpr uint32_t AUDIO_INIT_MIN_FREE_HEAP = 70000U;
static constexpr uint32_t AUDIO_INIT_MIN_DMA_BLOCK = 12000U;
static constexpr unsigned long AUDIO_INIT_RETRY_MS = 1200UL;
static constexpr uint32_t AUDIO_FLAC_MIN_FREE_HEAP = 120000U;
static constexpr uint32_t AUDIO_FLAC_MIN_LARGEST_8BIT = 45000U;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
static constexpr int RGB_PIN_R = 36;
static constexpr int RGB_PIN_G = 35;
static constexpr int RGB_PIN_B = 37;
static constexpr bool RGB_OUTPUT_SUPPORTED = false;
#else
static constexpr int RGB_PIN_R = 4;
static constexpr int RGB_PIN_G = 17;
static constexpr int RGB_PIN_B = 16;
static constexpr bool RGB_OUTPUT_SUPPORTED = true;
#endif
static constexpr bool RGB_ACTIVE_LOW = true;
static constexpr uint8_t TFT_BL_LEDC_CHANNEL = 0;
static constexpr uint16_t TFT_BL_LEDC_FREQ = 5000;
static constexpr uint8_t TFT_BL_LEDC_RES = 8;
static constexpr uint8_t TFT_BL_LEVEL_ON = 255;
static constexpr uint8_t TFT_BL_LEVEL_OFF = 0;
static constexpr bool DISPLAY_BACKLIGHT_PWM_SUPPORTED = true;
static constexpr unsigned long LCD_IDLE_TIMEOUT_MS_DEFAULT = 120000UL;
static constexpr unsigned long LCD_IDLE_TIMEOUT_MS_MIN = 15000UL;
static constexpr unsigned long LCD_IDLE_TIMEOUT_MS_MAX = 600000UL;
static constexpr unsigned long LCD_IDLE_TIMEOUT_STEP_MS = 15000UL;
static constexpr unsigned long ADAPTIVE_BRIGHTNESS_IDLE_DIM_DELAY_MS = 5000UL;
static constexpr uint8_t ADAPTIVE_BRIGHTNESS_IDLE_DIM_PERCENT = 50U;
static constexpr uint8_t ADAPTIVE_BRIGHTNESS_IDLE_TRIGGER_PERCENT = 80U;
static constexpr uint8_t ADAPTIVE_BRIGHTNESS_SENSOR_MIN_FACTOR_PERCENT = 35U;
static constexpr unsigned long SENSOR_SAMPLE_PERIOD_MS = 2000;
static constexpr unsigned long TOP_INDICATOR_REFRESH_MS = 1500;
static constexpr unsigned long TOP_INDICATOR_WIFI_CONNECT_ANIM_MS = 220;
static constexpr unsigned long WIFI_CONNECT_BARS_ANIM_PERIOD_MS = 240;
static constexpr unsigned long LIGHT_SLEEP_AFTER_IDLE_MS = 20000;
static constexpr bool LIGHT_SLEEP_TIMER_FALLBACK = false;
static constexpr uint64_t LIGHT_SLEEP_TIMER_US = 5000000ULL;
static constexpr unsigned long DEEP_SLEEP_WAKE_TIMER_US = 0ULL;
static constexpr unsigned long SCREENSAVER_POSE_MIN_MS = 1400UL;
static constexpr unsigned long SCREENSAVER_POSE_MAX_MS = 3200UL;
static constexpr unsigned long SCREENSAVER_BLINK_MS = 160UL;
static constexpr uint32_t SCREENSAVER_EYE_COLOR_RGB = 0x19E0C3UL;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
static constexpr int BATTERY_ADC_PIN = 10;
static constexpr int LIGHT_ADC_PIN = 6;
static constexpr int VIBRATION_MOTOR_PIN = 2;
static constexpr float BATTERY_DIVIDER_R_TOP = 470000.0f;
static constexpr float BATTERY_DIVIDER_R_BOTTOM = 220000.0f;
#else
static constexpr int BATTERY_ADC_PIN = 35;
static constexpr int LIGHT_ADC_PIN = 34;
static constexpr float BATTERY_DIVIDER_R_TOP = 220000.0f;
static constexpr float BATTERY_DIVIDER_R_BOTTOM = 100000.0f;
#endif
static constexpr float BATTERY_CAL_FACTOR = 0.96f;
static constexpr float BATTERY_EMPTY_V = 3.30f;
static constexpr float BATTERY_FULL_V = 4.20f;
// Legacy unused battery calibration bound constants were moved to app/garbage.inc:
// BATTERY_CAL_FACTOR_MIN, BATTERY_CAL_FACTOR_MAX
static constexpr float BATTERY_CAL_MIN_SPAN_V = 0.55f;
static constexpr float BATTERY_CAL_BLEND_ALPHA = 0.30f;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
static constexpr int BATTERY_ADC_SAMPLES = 32;
#else
static constexpr int BATTERY_ADC_SAMPLES = 16;
#endif
static constexpr int BATTERY_ADC_SETTLE_READS = 3;
static constexpr unsigned int BATTERY_ADC_SETTLE_US = 250U;
static constexpr float BATTERY_ADC_FALLBACK_REF_V = 3.30f;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
static constexpr uint8_t BATTERY_MEDIAN_WINDOW = 7;
#else
static constexpr uint8_t BATTERY_MEDIAN_WINDOW = 5;
#endif
static constexpr float BATTERY_FILTER_ALPHA_RISE = 0.08f;
static constexpr float BATTERY_FILTER_ALPHA_FALL = 0.05f;
static constexpr float BATTERY_FILTER_FAST_ALPHA = 0.30f;
static constexpr float BATTERY_FILTER_FAST_DELTA_V = 0.12f;
static constexpr unsigned long BATTERY_SNAPSHOT_PERIOD_MS = 30000;
static constexpr unsigned long BATTERY_SNAPSHOT_FORCE_MS = 300000;
static constexpr float BATTERY_SNAPSHOT_MIN_DELTA_V = 0.010f;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
static constexpr float BATTERY_BOOT_CHARGE_DELTA_V = 0.030f;
#else
static constexpr float BATTERY_BOOT_CHARGE_DELTA_V = 0.015f;
#endif
static constexpr float BATTERY_BOOT_EMPTY_INFER_MAX_V = 3.55f;
static constexpr unsigned long BATTERY_BOOT_EMPTY_MIN_UPTIME_MS = 120000UL;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
static constexpr unsigned long CHARGE_DETECT_INTERVAL_MS = 5000;
static constexpr float CHARGE_RISE_THRESHOLD_V = 0.008f;
static constexpr int8_t CHARGE_SCORE_ON = 3;
static constexpr float CHARGE_FILTER_ALPHA = 0.18f;
static constexpr unsigned long CHARGE_HOLD_MS = 30000;
#else
static constexpr unsigned long CHARGE_DETECT_INTERVAL_MS = 4000;
static constexpr float CHARGE_RISE_THRESHOLD_V = 0.003f;
static constexpr int8_t CHARGE_SCORE_ON = 1;
static constexpr float CHARGE_FILTER_ALPHA = 0.35f;
static constexpr unsigned long CHARGE_HOLD_MS = 120000;
#endif
static constexpr int8_t CHARGE_SCORE_MAX = 6;
static constexpr bool CHARGE_LOG_TO_SERIAL = false;
static constexpr uint8_t CHARGE_ANIM_CYCLES = 1;
static constexpr unsigned long CHARGE_CAL_SESSION_MIN_MS = 20UL * 60UL * 1000UL;
static constexpr float CHARGE_CAL_SESSION_MIN_RISE_V = 0.18f;
static constexpr float CHARGE_CAL_FULL_MIN_RAW_V = 3.65f;
static constexpr float CHARGE_CAL_PLATEAU_BAND_V = 0.015f;
static constexpr unsigned long CHARGE_CAL_PLATEAU_HOLD_MS = 5UL * 60UL * 1000UL;
static constexpr int LIGHT_ADC_SAMPLES = 8;
static constexpr float LIGHT_FILTER_ALPHA = 0.20f;
static constexpr bool LIGHT_INVERT = true;
static constexpr int LIGHT_MIN_SPAN_RAW = 80;
static constexpr uint16_t LIGHT_RAW_CAL_MIN = 0;
static constexpr uint16_t LIGHT_RAW_CAL_MAX = 600;
static constexpr bool LIGHT_LOG_RAW_TO_SERIAL = false;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
static constexpr unsigned long VIBRATION_PULSE_MS = 500UL;
static constexpr unsigned long VIBRATION_GAP_MS = 180UL;
static constexpr uint8_t VIBRATION_QUEUE_MAX = 4;
#endif

static constexpr const char *AP_PASS = "12345678";
static constexpr const char *FW_VERSION = "0.21.14";
static constexpr uint8_t MODULE_SLOT_COUNT = 8U;
static constexpr const char *MODULES_MANIFEST_URL = "https://raw.githubusercontent.com/elik745i/ESP32-2432S024C-Remote/main/modules_manifest.json";
static constexpr bool VERBOSE_SERIAL_DEBUG = false;
static constexpr unsigned long OTA_CHECK_INTERVAL_MS = 6UL * 60UL * 60UL * 1000UL;
static constexpr unsigned long OTA_INITIAL_CHECK_DELAY_MS = 5000UL;
static constexpr unsigned long OTA_RETRY_DELAY_MS = 10UL * 60UL * 1000UL;
static constexpr size_t OTA_VERSION_TEXT_MAX = 32;
static constexpr size_t OTA_DOWNLOAD_BUF_SIZE = 2048U;
static constexpr uint8_t OTA_RELEASE_LIST_MAX = 6U;
static constexpr size_t OTA_RELEASE_NOTES_STORE_MAX = 3072U;
static constexpr const char *APP_MODULE_IDS[MODULE_SLOT_COUNT] = {
    "chat", "radio", "media", "info", "games", "mqtt", "screensaver", "webserver"
};
static constexpr const char *APP_MODULE_TITLES[MODULE_SLOT_COUNT] = {
    "Chat Pack", "Radio Pack", "Media Pack", "Info Pack", "Games Pack", "MQTT Pack", "Screensaver Pack", "WebServer Pack"
};
static constexpr const char *APP_MODULE_DESCRIPTIONS[MODULE_SLOT_COUNT] = {
    "Peer chat and contacts",
    "Radio control and remote tools",
    "Player and media browser",
    "System info and diagnostics",
    "Snake, Tetris, and Checkers",
    "MQTT config and controls",
    "Screensaver toggle and presets",
    "Web server control and remote APIs"
};
static constexpr unsigned long STA_RETRY_INTERVAL_MS = 5000UL;
static constexpr bool SERIAL_TERMINAL_TRANSFER_ENABLED = false;
static constexpr size_t SERIAL_LOG_RING_SIZE = 200;
static constexpr size_t SERIAL_LOG_LINE_MAX = 192;
static constexpr uint32_t SERIAL_LOG_RATE_MS_DEFAULT = 40;
static constexpr uint32_t WS_TELEMETRY_MIN_FREE_HEAP = 50000U;
static constexpr int8_t TOP_BAR_GMT_MIN = -12;
static constexpr int8_t TOP_BAR_GMT_MAX = 14;
static constexpr unsigned long NTP_SYNC_RETRY_MS = 15UL * 60UL * 1000UL;
static constexpr unsigned long NTP_SYNC_REFRESH_MS = 6UL * 60UL * 60UL * 1000UL;
static constexpr const char *NTP_SERVER_1 = "pool.ntp.org";
static constexpr const char *NTP_SERVER_2 = "time.nist.gov";
static constexpr const char *NTP_SERVER_3 = "time.google.com";
static constexpr unsigned long POWER_OFF_IDLE_TIMEOUT_OPTIONS_MS[] = {
    15UL * 1000UL,
    2UL * 60UL * 1000UL,
    5UL * 60UL * 1000UL,
    15UL * 60UL * 1000UL,
    30UL * 60UL * 1000UL,
    0UL
};

// Legacy unused HC12 constants were moved to app/garbage.inc:
// HC12_UART_NUM, HC12_BAUD, HC12_RADIO_MAX_RX_LINE
#if defined(BOARD_ESP32S3_3248S035_N16R8)
static constexpr int HC12_RX_PIN_DEFAULT = 5;
static constexpr int HC12_TX_PIN_DEFAULT = 4;
static constexpr int HC12_SET_PIN_DEFAULT = 3;
static constexpr int E220_RX_PIN_DEFAULT = 4;
static constexpr int E220_TX_PIN_DEFAULT = 5;
static constexpr int E220_M0_PIN_DEFAULT = 3;
static constexpr int E220_M1_PIN_DEFAULT = 13;
static constexpr int POWER_OFF_SIGNAL_PIN = 21;
#else
static constexpr int HC12_RX_PIN_DEFAULT = 39;
static constexpr int HC12_TX_PIN_DEFAULT = 1;
static constexpr int HC12_SET_PIN_DEFAULT = 22;
static constexpr int E220_RX_PIN_DEFAULT = -1;
static constexpr int E220_TX_PIN_DEFAULT = -1;
static constexpr int E220_M0_PIN_DEFAULT = -1;
static constexpr int E220_M1_PIN_DEFAULT = -1;
static constexpr int POWER_OFF_SIGNAL_PIN = -1;
#endif
static constexpr unsigned long E220_AT_BAUD = 9600UL;
static constexpr unsigned long POWER_OFF_PULSE_MS = 100UL;
static constexpr unsigned long POWER_OFF_GAP_MS = 100UL;
static constexpr size_t HC12_TERMINAL_MAX_CHARS = 2800;
static constexpr unsigned long HC12_SUPPORTED_BAUDS[] = {1200UL, 2400UL, 4800UL, 9600UL, 19200UL, 38400UL, 57600UL, 115200UL};
static const char *const HC12_MODE_LABELS[] = {"Raw Mod", "Fast", "Norm", "LoRa"};
static constexpr int8_t HC12_POWER_DBM[] = {-1, 2, 5, 8, 11, 14, 17, 20};
static constexpr int HC12_MIN_CHANNEL = 1;
static constexpr int HC12_MAX_CHANNEL = 100;
static constexpr int E220_MIN_CHANNEL = 0;
static constexpr int E220_MAX_CHANNEL = 83;
static constexpr uint8_t E220_UART_BAUD_OPTIONS[] = {0, 1, 2, 3, 4, 5, 6, 7};
static constexpr unsigned long E220_UART_BAUD_VALUES[] = {1200UL, 2400UL, 4800UL, 9600UL, 19200UL, 38400UL, 57600UL, 115200UL};
static constexpr uint8_t E220_AIR_RATE_OPTIONS[] = {0, 1, 2, 3, 4, 5, 6, 7};
static const char *const E220_AIR_RATE_LABELS[] = {"2.4k", "2.4k", "2.4k", "4.8k", "9.6k", "19.2k", "38.4k", "62.5k"};
static constexpr int8_t E220_POWER_DBM[] = {22, 17, 13, 10};
static constexpr char HC12_RADIO_FRAME_PREFIX[] = "@RMT|";
static constexpr size_t HC12_RADIO_MAX_LINE = 1216;   // payload only

static constexpr int MAX_HC12_DISCOVERED = 8;
static constexpr unsigned long HC12_DISCOVERY_INTERVAL_MS = 8000UL;
static constexpr unsigned long HC12_DISCOVERY_STALE_MS = 45000UL;
static constexpr uint8_t UI_DEFERRED_HC12_SETTLE_PENDING = 0x04;
static constexpr uint8_t UI_DEFERRED_HC12_TARGET_ASSERTED = 0x08;
static constexpr const char *MODULES_STATUS_CHECKING = "Checking...";

static constexpr uint16_t CHAT_NOTIFY_FADE_IN_MS = 12;
static constexpr uint16_t CHAT_NOTIFY_FADE_OUT_MS = 12;

void serialLogPushLine(const char *line, bool sendWs = true);
void executeSerialCommand(String input);
decltype(::Serial) &serialHw = ::Serial;

static inline bool boardHasUsablePsram()
{
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    return psramFound();
#else
    return false;
#endif
}

static void *allocPreferPsram(size_t bytes, uint32_t fallbackCaps = MALLOC_CAP_8BIT)
{
    if (bytes == 0) return nullptr;
    if (boardHasUsablePsram()) {
        void *ptr = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (ptr) return ptr;
    }
    return heap_caps_malloc(bytes, fallbackCaps);
}

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

struct AppModuleCatalogEntry {
    const char *id;
    const char *title;
    const char *description;
};

struct OtaReleaseEntry {
    String tag;
    String binUrl;
    String notes;
};

struct BatterySnapshot {
    bool valid = false;
    float rawV = 0.0f;
    bool charging = false;
    uint32_t uptimeMs = 0;
};

struct BatteryCalibrationState {
    float observedFullRawV = 0.0f;
    float observedEmptyRawV = 0.0f;
    bool hasFullAnchor = false;
    bool hasEmptyAnchor = false;
};

enum BatteryTrainingPhase : uint8_t {
    BATTERY_TRAIN_IDLE = 0,
    BATTERY_TRAIN_CHARGE,
    BATTERY_TRAIN_DISCHARGE,
};

struct BatteryTrainingState {
    bool active = false;
    bool powerOverrideActive = false;
    bool autoCalibrationEnabled = false;
    BatteryTrainingPhase phase = BATTERY_TRAIN_IDLE;
    unsigned long savedPowerOffMs = 0;
};

// TFT display object moved to app/ui_display.inc.
#define UI_DISPLAY_SECTION_TFT
#include "app/ui_display.inc"
#undef UI_DISPLAY_SECTION_TFT
#if defined(BOARD_ESP32S3_3248S035_N16R8)
SPIClass sdSpi(FSPI);
#else
SPIClass sdSpi(HSPI);
#endif
AsyncWebServer server(80);
AsyncWebSocket wsCarInput("/CarInput");
DNSServer dnsServer;
Audio *audio = nullptr;
WiFiUDP p2pUdp;
void loadMediaEntries();
void refreshWifiScan();
void wifiScanService();
void stopWifiScan(bool refreshUi);
void startWifiConnect(const String &ssid, const String &pass);
void saveStaCreds(const String &ssid, const String &pass);
void clearStaCreds();
void saveApCreds(const String &ssid, const String &pass);
static String wifiDesiredStaSsid();
static String wifiDesiredStaPass();
void clearWifiScanResults();
static bool normalizeSavedApCreds();
static void snakeMaybeStoreHighScore(bool persist = false);
static void tetrisMaybeStoreHighScore(bool persist = false);
#if defined(BOARD_ESP32S3_3248S035_N16R8)
static void snake3dMaybeStoreHighScore(bool persist = false);
#endif
static void saveGameValue(const char *key, uint16_t value);
void tryBootStaReconnect();
void sampleTopIndicators();
float batteryCalibrationFactor();
const char *authName(wifi_auth_mode_t auth);
String mediaResolvePlaybackPath(const String &originalPath);
String mediaDisplayNameFromPath(const String &path);
uint8_t audioVolumeLevelFromPercent(uint8_t percent);
void mediaFormatSeconds(uint32_t sec, char *out, size_t outLen);
bool audioStartFile(const String &path);
void audioStopPlayback(bool smooth);
void audioSetVolumeImmediate(uint8_t v);
bool audioEnsureBackendReady(const char *reason);
void chatQueueIncomingMessageBeep();
void chatMessageBeepService();
void chatQueueIncomingMessageVibration();
void chatMessageVibrationService();
static void chatMessageBeepPreview();
static void chatMessageVibrationPreview();
static void touchFeedbackQueue();
static void touchFeedbackBeepService();
#if defined(BOARD_ESP32S3_3248S035_N16R8)
static void chatMessageVibrationStop(bool clearQueue);
static void touchFeedbackVibrationService();
#endif
bool mediaPathIsFlac(const String &path);
bool audioFlacSupportedNow(uint32_t *freeHeapOut = nullptr, uint32_t *largestOut = nullptr);
wl_status_t wifiStatusSafe();
bool wifiConnectedSafe();
bool wifiLanAvailableSafe();
int32_t wifiRssiSafe();
String wifiSsidSafe();
String wifiIpSafe();
IPAddress wifiLanIpSafe();
IPAddress wifiLanBroadcastIpSafe();
void registerWifiEvents();
static void wifiEnsureRuntimeEnabled(const char *reason, wifi_mode_t mode, bool startWebServer);
void ensureApOnline(const char *reason);
void forceSessionApMode(const char *reason);
void disableApWhenStaConnected(const char *reason);
static bool wifiHasStaTarget();
static void beginStaConnectAttempt(const char *reason);
static void stopDnsForAp();
static void stopWebServerRuntime();
static void ensureWebServerRuntime();
static void persistWebServerEnabled();
void wifiConnectionService();
bool mediaStartTrack(const String &sourcePath, const String &displayName);
String mediaFindAdjacentTrack(const String &sourcePath, bool nextDir);
void lvglSetMediaPlayerVisible(bool visible);
void lvglRefreshMediaLayout();
void lvglRefreshMediaPlayerUi();
void lvglHideKeyboard();
void networkSuspendForAudio();
bool networkResumeAfterAudio();
void lvglNavigateBackBySwipe(lv_scr_load_anim_t anim = LV_SCR_LOAD_ANIM_MOVE_RIGHT);
String mqttDefaultButtonName(int idx);
String mqttButtonPayloadForIndex(int idx);
String mediaBuildListLabel(const String &name, bool isDir, size_t sizeBytes);
void displaySetAwake(bool awake);
static void wakeDisplayForIncomingNotification();
void displayBacklightInit();
void displayBacklightSet(uint8_t level);
void displayBacklightFadeIn(uint16_t durationMs = 220);
uint8_t displayBacklightLevelFromPercent(uint8_t percent);
uint8_t displayEffectiveBrightnessPercent();
void displayApplyBacklightPolicy(bool force = false);
bool animateChargingBeforeSleep();
void snakeResetGame();
void snakePrepareGame();
void snakeStartGame();
#if defined(BOARD_ESP32S3_3248S035_N16R8)
void snake3dResetGame();
void snake3dPrepareGame();
void snake3dStartGame();
#endif
void tetrisResetGame();
void tetrisPrepareGame();
void tetrisStartGame();
void tetrisMove(int dx);
void setupCarInputSocket();
void serviceCarInputTelemetry();
String normalizeSdRoutePath(const String &rawPath, bool allowEmptyRoot = false);
bool sdShouldHideSystemEntry(const String &name);
String makeUniquePath(const String &path);
String recycleMetaPathFor(const String &recyclePath);
void saveBatterySnapshot(float rawV, bool charging, uint32_t uptimeMs);
bool hasPrimaryWebUi();
String apLandingContinueUrl();
void sendWifiSetupPage(AsyncWebServerRequest *request);
void sendApWifiLandingPage(AsyncWebServerRequest *request);
String httpsGetText(const String &url, int *statusOut = nullptr);
String chooseLatestFirmwareBinUrl(const JsonVariantConst &assets);
bool otaDownloadAndApplyFromUrl(const String &url, String &errorOut);
void otaCheckService();
void otaUpdateService();
static bool otaFetchLatestReleaseInfo(String &tagNameOut, String &binUrlOut, String &errorOut);
static bool otaFetchReleaseCatalog(String &latestTagOut, String &latestUrlOut, String &errorOut);
static void otaSetStatus(const String &text);
static void otaClearPopupVersion();
static void otaCheckTask(void *param);
static void otaUpdateTask(void *param);
// Legacy unused OTA helper was moved to app/garbage.inc:
// otaFetchReleaseInfoForTag(const String&, String&, String&)
static bool otaStartUpdateTask(const String &binUrl, const String &targetVersion, const String &statusText, const String &releaseNotes = "");
void lvglOpenOtaScreenEvent(lv_event_t *e);
void lvglOpenHc12ScreenEvent(lv_event_t *e);
void lvglOpenRemoteControlScreenEvent(lv_event_t *e);
void lvglOpenRadioButtonConfigScreenEvent(lv_event_t *e);
void lvglOpenRadioReceiverTestScreenEvent(lv_event_t *e);
void lvglOpenHc12TerminalEvent(lv_event_t *e);
void lvglOpenHc12InfoEvent(lv_event_t *e);
void lvglTextAreaFocusEvent(lv_event_t *e);
void lvglGateControlActionEvent(lv_event_t *e);
void lvglRefreshRadioControlUi();
void lvglRefreshRemoteControlUi();
void lvglRemoteControlSaveEvent(lv_event_t *e);
void lvglRemoteControlModuleChangedEvent(lv_event_t *e);
void lvglRefreshRadioButtonConfigUi();
void lvglRefreshRadioReceiverTestUi();
void lvglRadioButtonAddEvent(lv_event_t *e);
void lvglRadioButtonSelectModeEvent(lv_event_t *e);
void lvglRadioButtonDeletePromptEvent(lv_event_t *e);
void lvglRadioButtonDeleteConfirmEvent(lv_event_t *e);
void lvglRadioButtonFieldChangedEvent(lv_event_t *e);
void lvglRadioControlButtonEvent(lv_event_t *e);
void lvglRadioControlConfirmEvent(lv_event_t *e);
bool radioReceiverTestHandleByte(uint8_t ch);
void radioReceiverTestEnterScreen();
void radioReceiverTestLeaveScreen();
void lvglHc12PrevChannelEvent(lv_event_t *e);
void lvglHc12NextChannelEvent(lv_event_t *e);
void lvglHc12PrevBaudEvent(lv_event_t *e);
void lvglHc12NextBaudEvent(lv_event_t *e);
void lvglHc12PrevModeEvent(lv_event_t *e);
void lvglHc12NextModeEvent(lv_event_t *e);
void lvglHc12PrevPowerEvent(lv_event_t *e);
void lvglHc12NextPowerEvent(lv_event_t *e);
void lvglRadioPrevPinSwapEvent(lv_event_t *e);
void lvglRadioNextPinSwapEvent(lv_event_t *e);
void lvglRadioPrevModePinSwapEvent(lv_event_t *e);
void lvglRadioNextModePinSwapEvent(lv_event_t *e);
void lvglHc12DefaultEvent(lv_event_t *e);
void lvglOtaUpdateEvent(lv_event_t *e);
void lvglOtaReflashEvent(lv_event_t *e);
void lvglOtaReleaseSelectEvent(lv_event_t *e);
void lvglRefreshOtaUi();
void lvglRefreshHc12Ui();
void lvglRefreshHc12ConfigUi();
void lvglRefreshHc12InfoUi();
void loadGamePrefs();
void screensaverSetActive(bool active);
void screensaverService();
void tetrisRotate();
void tetrisDrop();
bool tetrisCellFor(int type, int rot, int i, int &ox, int &oy);
uint8_t wifiQualityPercentFromRssi(int32_t rssi);
void mqttPublishAction(const char *action);
void mqttPublishButtonState(int index);
void mqttHandleButtonCommand(int index, const String &payload);
String mqttButtonCommandTopic(int idx);
void mqttHandleHaChatCommand(const String &payload);
String mqttHaChatReceivedTopic();
String mqttHaChatReceivedCommandTopic();
void mqttService();
bool mqttConnectNow();
void mqttPublishDiscovery();
void mqttPublishStateIfNeeded(bool force = false);
void mqttTrimBufferForIdle();
// Legacy unused MQTT wrappers were moved to app/garbage.inc:
// mqttPublishButtonAction(int), mqttHaChatStateTopic(), mqttPublishChatMessage(const String&)
bool mqttPublishChatMessageWithId(const String &peerKey, const String &text, const String &messageId);
void mqttPublishPeerPresence();
bool mqttPublishMessageDelete(const String &peerKey, const String &messageId);
bool mqttPublishConversationDelete();
bool mqttPublishChatAck(const String &peerKey, const String &messageId);
void loadMqttConfig();
void saveMqttConfig();
void appendUiSettings(JsonDocument &doc);
bool handleUiSettingMessage(const char *msg);
void loadUiRuntimeConfig();
void applyAirplaneMode(bool enabled, const char *reason);
void lvglRefreshConfigUi();
void lvglRefreshAllButtonStyles();
void lvglSetButtonStyleMode(UiButtonStyleMode mode, bool persist);
void lvglRefreshStyleUi();
void lvglRefreshLockScreenUi();
void lvglRefreshMqttControlsUi();
void lvglRefreshMqttButtonConfigUi();
void lvglMqttButtonAddEvent(lv_event_t *e);
void lvglMqttButtonSelectModeEvent(lv_event_t *e);
void lvglMqttButtonDeletePromptEvent(lv_event_t *e);
void lvglMqttButtonDeleteConfirmEvent(lv_event_t *e);
void lvglMqttButtonFieldChangedEvent(lv_event_t *e);
void lvglSoundPopupBackdropEvent(lv_event_t *e);
void lvglSoundPopupVolumeEvent(lv_event_t *e);
void lvglSoundPopupVibrationEvent(lv_event_t *e);
void lvglSoundPopupDisableVibrationEvent(lv_event_t *e);
void lvglApplyAirplaneButtonStyle();
void lvglApplyApModeButtonStyle();
void lvglApplyChatDiscoveryButtonStyle();
lv_obj_t *lvglCreateMenuButton(lv_obj_t *parent, const char *txt, lv_color_t color, lv_event_cb_t cb, void *user = nullptr);
void lvglApplyWifiWebServerButtonStyle();
void lvglFactoryResetEvent(lv_event_t *e);
void lvglFactoryResetConfirmEvent(lv_event_t *e);
void lvglStyleScreenLockToggleEvent(lv_event_t *e);
void lvglStyleConfigLockToggleEvent(lv_event_t *e);
void lvglScreenLockUnlockEvent(lv_event_t *e);
void lvglScreenLockFactoryResetEvent(lv_event_t *e);
void lvglScreenLockPinChangedEvent(lv_event_t *e);
void lvglProtectedPinSubmitEvent(lv_event_t *e);
void lvglOpenMqttButtonConfigScreenEvent(lv_event_t *e);
enum ChatTransport : uint8_t;
static bool p2pHexToBytes(const String &hex, unsigned char *out, size_t outLen);
static void p2pRefreshTrustedPeerIdentity(const String &pubKeyHex, const String &name, const IPAddress &ip, uint16_t port);
static void p2pTouchPeerSeen(int idx, const IPAddress &ip, uint16_t port);
static bool chatHasLoggedMessageId(const String &peerKey, const String &messageId);
static void chatStoreMessage(const String &peerKey, const String &author, const String &text, bool outgoing, ChatTransport transport, const String &messageId);
void lvglStatusPush(const String &line);
static String lvglSymbolText(const char *symbol, const String &text);
static void lvglConfigureCompactSingleLineTa(lv_obj_t *ta, lv_coord_t height);
static void lvglRegisterReorderableItem(lv_obj_t *obj, const char *prefKey, const char *itemKey);
static void lvglApplySavedOrder(lv_obj_t *parent, const char *prefKey);
static void factoryResetWipeStoredData();
static void factoryResetClearNamespace(Preferences &prefs, const char *ns);
static void factoryResetWipeSdData();
static bool screenLockIsConfigured();
static bool screenLockShouldGateUi();
static bool screenLockIsBlocked();
static bool configLockIsConfigured();
static void configLockPromptForEntry();
static bool screenLockNumericTarget(lv_obj_t *ta);
static void screenLockSanitizeTextArea(lv_obj_t *ta);
static void screenLockResetState();
static void screenLockPersistConfig();
static void screenLockShowSetupModal();
static void screenLockHideSetupModal(bool restoreToggle);
static String screenLockBlockMessage();
static void lvglAttachMenuButtonImage(lv_obj_t *btn, const lv_img_dsc_t *imgSrc, lv_coord_t xOffset, lv_coord_t labelShiftX);
static void lvglSetButtonImageZoom(lv_obj_t *btn, uint16_t zoom, lv_coord_t xOffset = 8, lv_coord_t labelShiftX = 10);
static void lvglRefreshPrimaryMenuButtonIcons();
static void lvglLabelSetTextIfChanged(lv_obj_t *label, const char *text);
static void lvglLabelSetTextIfChanged(lv_obj_t *label, const String &text);
static void lvglApplyPersistentToggleButtonStyle(lv_obj_t *btn,
                                                 lv_obj_t *label,
                                                 bool enabled,
                                                 lv_color_t offBodyCol,
                                                 lv_color_t onBodyCol,
                                                 bool compact);
static void lvglRegisterStyledButton(lv_obj_t *btn, lv_color_t baseColor, bool compact);
static void lvglRegisterPersistentStateButtonColors(lv_obj_t *btn, lv_color_t offBodyCol, lv_color_t onBodyCol, bool enabled, bool compact);
static void lvglRegisterPersistentStateButton(lv_obj_t *btn, lv_color_t offBodyCol, bool enabled, bool compact);
static String topBarCenterText();
static bool internetTimeValid();
static void syncInternetTimeIfNeeded(bool force = false);
static String buildGmtOffsetDropdownOptions();
static void batteryCalibrationLearnFull(float rawV);
static void batteryCalibrationSetFull(float rawV);
static void batteryCalibrationForceFull(float rawV);
static void batteryCalibrationLoad();
static float batteryCalibratedVoltageFromRaw(float rawV);
static float batteryFilterSample(float calibratedV);
uint8_t batteryPercentFromVoltage(float vbat);
static void batteryCalibrationReset();
static void batteryTrainingLoad();
static void batteryTrainingSave();
static void batteryTrainingRestorePowerOff();
static void batteryTrainingStartCharge();
static void batteryTrainingStartDischarge();
static void batteryTrainingStopManual();
static const char *batteryTrainingPhaseLabel();
static void lvglRefreshBatteryTrainButtonIcons();
static void lvglRefreshBatteryTrainUi();
void enterDeepSleep();
void lvglBatteryTrainResetEvent(lv_event_t *e);
void lvglBatteryTrainResetPromptEvent(lv_event_t *e);
void lvglBatteryTrainFullConfirmEvent(lv_event_t *e);
void lvglBatteryTrainFullEvent(lv_event_t *e);
void lvglBatteryTrainDischargeEvent(lv_event_t *e);
void lvglBatteryTrainAutoConfirmEvent(lv_event_t *e);
void lvglBatteryTrainAutoEvent(lv_event_t *e);
void lvglHc12ToggleSetEvent(lv_event_t *e);
void lvglHc12SendEvent(lv_event_t *e);
void lvglSaveDeviceNameEvent(lv_event_t *e);
void lvglStartupSoundToggleEvent(lv_event_t *e);
void lvglSetConfigKeyboardVisible(bool visible);
void lvglShowChatAirplanePrompt();
void lvglKeyboardEnsureFocusedVisible(bool animated);
void lvglKeyboardRefreshLayout();
void lvglKeyboardApplyTextMode();
static void lvglRefreshWifiPasswordDialogLayout();
static lv_obj_t *hc12CmdTaObj();
void cpuLoadService(uint32_t loopStartUs);
void rgbService();
void loadP2pConfig();
void saveP2pConfig();
void p2pEnsureUdp();
void p2pService();
// Legacy unused P2P wrapper was moved to app/garbage.inc:
// p2pSendChatMessage(const String&)
bool p2pSendChatMessageWithId(const String &peerKey, const String &text, const String &messageId);
bool p2pSendMessageDelete(const String &peerKey, const String &messageId);
bool p2pSendConversationDelete();
bool p2pSendChatAck(const String &peerKey, const String &messageId);
void playStartupFeedbackIfEnabled();
static bool hc12BroadcastDiscoveryFrame(const char *kind);
bool hc12SendChatMessageWithId(const String &peerKey, const String &text, const String &messageId);
bool hc12SendMessageDelete(const String &peerKey, const String &messageId);
bool hc12SendConversationDelete(const String &peerKey);
bool hc12SendChatAck(const String &peerKey, const String &messageId);
static String hc12QueryCommand(const char *line, unsigned long totalTimeoutMs = 180UL, unsigned long quietTimeoutMs = 32UL);
static String e220QueryCommand(const char *line, unsigned long totalTimeoutMs, unsigned long quietTimeoutMs);
static void hc12EnterAtMode();
static void hc12ExitAtMode();
static void radioSyncAppliedRuntimeWithCurrentSettings();
static String e220ValueAfterEquals(String raw);
static bool infoRefreshPending = false;
static bool radioInfoFetchPending = false;
static unsigned long radioInfoFetchDueMs = 0;
static uint8_t infoRefreshPhase = 0;
static uint8_t radioInfoFetchStage = 0;
static bool radioInfoFetchActive = false;
static unsigned long radioInfoFetchNextMs = 0;
static void lvglAirplaneButtonDrawEvent(lv_event_t *e);
String p2pPublicKeyHex();
static int p2pFindPeerByPubKeyHex(const String &pubKeyHex);
void p2pBroadcastDiscover();
void p2pSendDiscoveryProbe();
void p2pSendDiscoveryAnnounceTo(const IPAddress &ip, uint16_t port);
bool p2pSendPairRequest(const String &name, const String &pubKeyHex, const IPAddress &ip, uint16_t port);
void p2pSendPairResponse(const String &name, const String &pubKeyHex, const IPAddress &ip, uint16_t port, bool accepted);
bool p2pAddOrUpdateTrustedPeer(const String &name, const String &pubKeyHex, const IPAddress &ip, uint16_t port);
void setupWifiAndServer();
void setupWebRoutes();
void lvglRefreshChatUi();
void lvglRefreshChatPeerUi();
void lvglSetChatKeyboardVisible(bool visible);
void lvglSyncStatusLine();
void lvglRefreshTopIndicators();
void chatReloadRecentMessagesFromSd(const String &peerKey);
void lvglRefreshChatContactsUi();
static bool chatPeerHasHistory(const String &peerKey);
static String chatDisplayNameForPeerKey(const String &peerKey);
static bool chatPeerIsSelectable(const String &peerKey);
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

#if defined(BOARD_ESP32S3_3248S035_N16R8)
static constexpr uint16_t LVGL_BUF_LINES_MIN = 80;
static constexpr uint16_t LVGL_BUF_LINES_MAX = 160;
static constexpr uint16_t UI_ANIM_MS = 88;
#else
static constexpr uint16_t LVGL_BUF_LINES_MIN = 32;
static constexpr uint16_t LVGL_BUF_LINES_MAX = 48;
static constexpr uint16_t UI_ANIM_MS = 72;
#endif
static constexpr uint16_t UI_BUTTON_CLICK_FLASH_MS = 90;
static constexpr lv_opa_t UI_BUTTON_CLICK_FLASH_MIX = 64;
static constexpr int16_t UI_TOP_BAR_H = 30;
static constexpr int16_t UI_CONTENT_TOP_Y = UI_TOP_BAR_H;
static constexpr int16_t UI_CONTENT_H = DISPLAY_HEIGHT - UI_CONTENT_TOP_Y;
static constexpr uint8_t INFO_TEMP_BAR_MAX_C = 100;
static constexpr uint8_t INFO_TEMP_WARN_C = 65;
static constexpr uint8_t INFO_TEMP_HOT_C = 80;
static constexpr int16_t SWIPE_BACK_MIN_DX = 34;
static constexpr int16_t SWIPE_BACK_MAX_DY = 42;
static constexpr int16_t SWIPE_LOCK_MIN_DX = 12;
static constexpr int16_t SWIPE_CANCEL_VERTICAL_DY = 24;
static constexpr uint16_t SWIPE_BACK_SNAP_MIN_MS = 22;
static constexpr uint16_t SWIPE_BACK_COMMIT_MIN_MS = 120;
static constexpr uint16_t SWIPE_BACK_COMMIT_MAX_MS = 220;
static constexpr uint8_t SWIPE_BACK_COMPLETE_PERCENT = 10;
static constexpr unsigned long SWIPE_BACK_MAX_MS = 700;
static constexpr int16_t DOUBLE_TAP_MAX_MOVE = 12;
static constexpr int16_t DOUBLE_TAP_MAX_GAP = 350;
static constexpr unsigned long DOUBLE_TAP_MAX_TAP_MS = 240;
static constexpr unsigned long CLICK_SUPPRESS_AFTER_GESTURE_MS = 220UL;
static constexpr unsigned long UI_INTERACTION_COOLDOWN_MS = 180UL;
static constexpr unsigned long UI_WARMUP_INTERVAL_MS = 420UL;
static constexpr unsigned long UI_WARMUP_IDLE_AFTER_INPUT_MS = 3500UL;
// LVGL display state and screen handles moved to app/ui_display.inc.
#define UI_DISPLAY_SECTION_STATE
#include "app/ui_display.inc"
#undef UI_DISPLAY_SECTION_STATE

static void chatLoadPendingOutbox();

// SD card runtime/recovery logic moved to app/sd_card.inc.
#include "app/sd_card.inc"

void audio_info(const char *info)
{
    Serial.println(String("[AUDIO] ") + info);
}

bool bootWifiInitPending = true;
bool bootRadioInitPending = false;
bool bootRadioProbePending = false;
bool bootStartupFeedbackPending = false;
bool wifiRuntimeManaged = true;
bool wifiScanInProgress = false;
unsigned long wifiScanStartedMs = 0;
bool wifiScanFallbackUsed = false;
unsigned long wifiScanAnimLastMs = 0;
uint8_t wifiScanAnimPhase = 0;
unsigned long bootDeferredStartMs = 0;
unsigned long bootInteractiveUntilMs = 0;
unsigned long bootRadioInitDueMs = 0;
unsigned long bootRadioProbeDueMs = 0;
unsigned long bootStartupFeedbackDueMs = 0;
bool mdnsStarted = false;
bool webRoutesRegistered = false;
bool webServerRunning = false;
bool dnsRunning = false;
bool webServerEnabled = true;
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
bool touchLastSampleDown = false;
int16_t touchLastSampleX = 0;
int16_t touchLastSampleY = 0;
float batteryVoltage = 0.0f;
float batteryRawVoltage = 0.0f;
float batteryVoltageMedianWindow[BATTERY_MEDIAN_WINDOW] = {0.0f};
uint8_t batteryVoltageMedianIndex = 0;
uint8_t batteryVoltageMedianCount = 0;
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
bool adaptiveBrightnessEnabled = false;
uint8_t displayAppliedBrightnessPercent = 0;
unsigned long displayIdleTimeoutMs = LCD_IDLE_TIMEOUT_MS_DEFAULT;
unsigned long powerOffIdleTimeoutMs = 0;
unsigned long deepSleepIdleTimeoutMs = 0;
bool screensaverEnabled = false;
bool screensaverActive = false;
uint8_t rgbLedPercent = 100;
String deviceShortName = DEVICE_SHORT_NAME;
uint8_t cpuLoadPercent = 0;
uint16_t infoFpsValue = 0;
uint16_t infoFpsFrameCount = 0;
unsigned long infoFpsLastMs = 0;
bool batteryFilterInitialized = false;
bool lightFilterInitialized = false;
float lightPercentFiltered = 0.0f;
float chargePrevVoltage = 0.0f;
float chargeFilteredVoltage = 0.0f;
bool chargeFilterInitialized = false;
int8_t chargeTrendScore = 0;
bool chargeSessionActive = false;
unsigned long chargeSessionStartMs = 0;
unsigned long chargePlateauStartMs = 0;
float chargeSessionStartRawVoltage = 0.0f;
float chargeSessionMaxRawVoltage = 0.0f;
uint8_t batteryIconAnimPercent = 0;
unsigned long batteryIconAnimLastMs = 0;
uint16_t lightRawAdc = 0;
uint16_t lightMinObserved = 4095;
uint16_t lightMaxObserved = 0;
uint8_t wakeTouchConfirmCount = 0;
bool wakeTouchReleaseGuard = false;
unsigned long screensaverLastPoseMs = 0;
unsigned long screensaverNextPoseDelayMs = SCREENSAVER_POSE_MIN_MS;

enum UiScreen : uint8_t {
    UI_HOME,
    UI_LOCK,
    UI_CHAT,
    UI_CHAT_PEERS,
    UI_WIFI_LIST,
    UI_MEDIA,
    UI_INFO,
    UI_GAMES,
    UI_RADIO_CONTROL,
    UI_CONFIG,
    UI_CONFIG_REMOTE_CONTROL,
    UI_CONFIG_RADIO_BUTTONS,
    UI_CONFIG_RADIO_RECEIVER_TEST,
    UI_CONFIG_BATTERY,
    UI_CONFIG_STYLE,
    UI_CONFIG_STYLE_HOME_ITEMS,
    UI_CONFIG_LANGUAGE,
    UI_CONFIG_OTA,
    UI_CONFIG_MODULES,
    UI_SCREENSAVER,
    UI_CONFIG_HC12,
    UI_CONFIG_HC12_TERMINAL,
    UI_CONFIG_HC12_INFO,
    UI_CONFIG_MQTT_CONFIG,
    UI_CONFIG_MQTT_CONTROLS,
    UI_CONFIG_MQTT_BUTTONS,
    UI_GAME_SNAKE,
    UI_GAME_TETRIS,
    UI_GAME_CHECKERS,
    UI_GAME_SNAKE3D
};

enum AppModuleId : uint8_t {
    APP_MODULE_CHAT = 0,
    APP_MODULE_RADIO,
    APP_MODULE_MEDIA,
    APP_MODULE_INFO,
    APP_MODULE_GAMES,
    APP_MODULE_MQTT,
    APP_MODULE_SCREENSAVER,
    APP_MODULE_WEBSERVER,
    APP_MODULE_COUNT
};

enum UiLanguage : uint8_t {
    UI_LANG_ENGLISH = 0,
    UI_LANG_RUSSIAN,
    UI_LANG_CHINESE,
    UI_LANG_FRENCH,
    UI_LANG_TURKISH,
    UI_LANG_ITALIAN,
    UI_LANG_GERMAN,
    UI_LANG_JAPANESE,
    UI_LANG_KOREAN,
    UI_LANG_COUNT
};

enum UiTextId : uint8_t {
    TXT_CHAT = 0,
    TXT_CHAT_PEERS,
    TXT_MEDIA,
    TXT_INFO,
    TXT_GAMES,
    TXT_RADIO_CONTROL,
    TXT_GATE_01,
    TXT_GATE_01_LIGHT,
    TXT_GATE_02,
    TXT_GATE_02_LIGHT,
    TXT_CONFIG,
    TXT_REMOTE_CONTROL,
    TXT_BUTTON_CONFIG,
    TXT_CONFIRM,
    TXT_NO_RADIO_BUTTONS,
    TXT_ADD_FIRST_RADIO_BUTTON,
    TXT_BUTTON_LIMIT_REACHED,
    TXT_SELECT_BUTTONS_TO_DELETE,
    TXT_DELETE_BUTTONS,
    TXT_DELETE_SELECTED_RADIO_BUTTONS,
    TXT_SEND,
    TXT_ENCRYPTION_KEY,
    TXT_REMOTE_ID,
    TXT_ROLLING_COUNTER,
    TXT_TRANSFER_MODE,
    TXT_AIRPLANE_ON,
    TXT_AIRPLANE_OFF,
    TXT_AP_MODE_ON,
    TXT_AP_MODE_OFF,
    TXT_WIFI_CONFIG,
    TXT_HC12_CONFIG,
    TXT_STYLE,
    TXT_MQTT_CONFIG,
    TXT_MQTT_CONTROLS,
    TXT_LANGUAGE,
    TXT_OTA_UPDATES,
    TXT_OTA_CURRENT_VERSION,
    TXT_OTA_LATEST_VERSION,
    TXT_OTA_STATUS,
    TXT_OTA_CHECK_NOW,
    TXT_OTA_UPDATE,
    TXT_OTA_FLUSH_SELECTED,
    TXT_OTA_PREVIOUS_RELEASES,
    TXT_OTA_SELECT_RELEASE,
    TXT_DEVICE_NAME,
    TXT_SAVE,
    TXT_BRIGHTNESS,
    TXT_VOLUME,
    TXT_RGB_LED,
    TXT_VIBRATION,
    TXT_LOW,
    TXT_MEDIUM,
    TXT_HIGH,
    TXT_SELECT_DISPLAY_LANGUAGE,
    TXT_LANGUAGE_SAVED,
    TXT_HC12_TERMINAL,
    TXT_HC12_INFO,
    TXT_CHECKERS,
    TXT_SNAKE_3D,
    TXT_FACTORY_RESET,
    TXT_FACTORY_RESET_TITLE,
    TXT_FACTORY_RESET_BODY,
    TXT_RESET,
    TXT_CANCEL,
    TXT_RESTART,
    TXT_POWER,
    TXT_POWER_OFF,
    TXT_POWER_ACTION_BODY,    
    TXT_FACTORY_RESET_DONE,    
    TXT_SCREEN,
    TXT_BATTERY,
    TXT_ADAPTIVE_BRIGHTNESS,
    TXT_RADIO_MODULE,
    TXT_RADIO_INFO_LOADING,
    TXT_SCREENSAVER,
    TXT_3D_ICONS,
    TXT_TOUCH_VIBRATION,
    TXT_TOUCH_TICK_SOUND,
    TXT_MAIN_SCREEN_ITEMS,
    TXT_AIRPLANE_MODE,
    TXT_AP_MODE,
    TXT_BUTTON_STYLE,
    TXT_FLAT_BUTTONS,
    TXT_3D_BUTTONS,
    TXT_BLACK_BUTTONS,
    TXT_TIMEZONE,
    TXT_TOP_BAR_CENTER,
    TXT_SHOW_DEVICE_NAME,
    TXT_SHOW_TIME,
    TXT_SCREEN_TIMEOFF,
    TXT_DEEP_SLEEP,
    TXT_STYLE_HINT,
    TXT_SELECT,
    TXT_SELECTED,
    TXT_STORED_CALIBRATION,
    TXT_AUTO_CALIBRATION,
    TXT_VERSION,
    TXT_BAUD_RATE,
    TXT_CHANNEL,
    TXT_FU_MODE,
    TXT_RAW,
    TXT_DISCOVERY,
    TXT_SCAN,
    TXT_PEER_DISCOVERY_ENABLED,
    TXT_PEER_DISCOVERY_DISABLED
};

enum Hc12ConfigApplyAction : uint8_t;

UiScreen uiScreen = UI_HOME;
static UiScreen lvglSwipeSourceScreen = UI_HOME;
UiScreen screensaverReturnScreen = UI_HOME;
String uiStatusLine = "Ready";
UiLanguage uiLanguage = UI_LANG_ENGLISH;

enum VibrationIntensity : uint8_t {
    VIBRATION_INTENSITY_LOW = 0,
    VIBRATION_INTENSITY_MEDIUM,
    VIBRATION_INTENSITY_HIGH,
    VIBRATION_INTENSITY_COUNT
};

enum RadioModuleType : uint8_t {
    RADIO_MODULE_HC12 = 0,
    RADIO_MODULE_E220,
    RADIO_MODULE_COUNT
};

enum MessageBeepTone : uint8_t {
    MESSAGE_BEEP_SINGLE = 0,
    MESSAGE_BEEP_DOUBLE_SHORT,
    MESSAGE_BEEP_ASCEND,
    MESSAGE_BEEP_DESCEND,
    MESSAGE_BEEP_DOORBELL,
    MESSAGE_BEEP_WESTMINSTER,
    MESSAGE_BEEP_FUR_ELISE,
    MESSAGE_BEEP_ODE_TO_JOY,
    MESSAGE_BEEP_TONE_COUNT
};

enum ScreensaverMode : uint8_t {
    SCREENSAVER_MODE_EYES = 0,
    SCREENSAVER_MODE_COUNT
};

VibrationIntensity vibrationIntensity = VIBRATION_INTENSITY_MEDIUM;
RadioModuleType radioModuleType = RADIO_MODULE_HC12;
MessageBeepTone messageBeepTone = MESSAGE_BEEP_DOUBLE_SHORT;
ScreensaverMode screensaverMode = SCREENSAVER_MODE_EYES;
bool vibrationEnabled = true;

static const char *tr(UiTextId id);
static String buildLanguageDropdownOptions();
static String buildVibrationIntensityDropdownOptions();
static String buildRadioModuleDropdownOptions();
static String buildMessageBeepDropdownOptions();
static String buildScreensaverDropdownOptions();
static const char *vibrationIntensityLabel(VibrationIntensity intensity);
static const char *radioModuleLabel(RadioModuleType module);
static const char *messageBeepToneLabel(MessageBeepTone tone);
static const char *screensaverModeLabel(ScreensaverMode mode);
static void saveSoundPrefs();
static uint8_t uiSoundMode();
static const lv_img_dsc_t *uiSoundModeIcon();
static const lv_img_dsc_t *uiVolumeIcon();
static void lvglRefreshSoundPopupUi();
static void lvglShowSoundPopup();
static void lvglHideSoundPopup();
static void lvglOtaPopupEvent(lv_event_t *e);
void lvglOpenModulesScreenEvent(lv_event_t *e);
void lvglModuleActionEvent(lv_event_t *e);
void lvglRefreshModulesUi();
bool moduleRefreshCatalog();
void moduleStartCatalogRefresh();
void moduleStopCatalogRefresh(bool preserveStatus = true);
void moduleCatalogService();
bool moduleCatalogCheckInProgress();
static bool moduleInstalledForScreen(UiScreen screen);
static bool moduleNeedsP2pRuntime();
static void lvglShowOtaPostUpdatePopup();
static void lvglHideOtaPostUpdatePopup();
static void lvglApplyMsgboxModalStyle(lv_obj_t *msgbox);

void lvglEnsureScreenBuilt(UiScreen screen);
lv_obj_t *lvglScreenForUi(UiScreen screen);
UiScreen lvglUiForScreenObj(lv_obj_t *screenObj);
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
    CHAT_TRANSPORT_HC12 = 2,
    CHAT_TRANSPORT_E220 = 3,
};

static ChatTransport radioSelectedChatTransport();
static bool radioModuleCanCarryChat();
static const char *radioModuleChatLabel();

struct ChatMessage {
    String author;
    String text;
    String messageId;
    bool outgoing;
    bool failed;
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

struct Hc12DiscoveredPeer {
    String name;
    String pubKeyHex;
    bool unread;
    unsigned long lastSeenMs;
};

Hc12DiscoveredPeer *hc12DiscoveredPeers = nullptr;
int hc12DiscoveredCount = 0;

struct PendingChatMessage {
    String peerKey;
    String author;
    String text;
    String messageId;
    unsigned long createdMs;
    unsigned long lastAttemptMs;
    unsigned long lastRadioAttemptMs;
    uint8_t attempts;
    uint8_t radioAttempts;
};

PendingChatMessage chatPendingMessages[MAX_CHAT_PENDING];
int chatPendingCount = 0;
bool chatPendingLoaded = false;
static constexpr int MAX_GAME_CONTROL_IDS = 4;
char gameControlMessageIds[MAX_GAME_CONTROL_IDS][17] = {};
uint8_t gameControlMessageIdCount = 0;

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

static void copyTextToBuf(char *dst, size_t dstSize, const String &value)
{
    if (!dst || dstSize == 0) return;
    strncpy(dst, value.c_str(), dstSize - 1);
    dst[dstSize - 1] = '\0';
}

static void copyTextToBuf(char *dst, size_t dstSize, const char *value)
{
    if (!dst || dstSize == 0) return;
    strncpy(dst, value ? value : "", dstSize - 1);
    dst[dstSize - 1] = '\0';
}

static String otaNormalizedVersion(const String &raw)
{
    String value = raw;
    value.trim();
    while (value.startsWith("v") || value.startsWith("V")) value.remove(0, 1);
    return value;
}

static int otaCompareVersions(const String &lhsRaw, const String &rhsRaw)
{
    const String lhs = otaNormalizedVersion(lhsRaw);
    const String rhs = otaNormalizedVersion(rhsRaw);
    int li = 0;
    int ri = 0;
    while (li < lhs.length() || ri < rhs.length()) {
        long lv = 0;
        long rv = 0;
        while (li < lhs.length() && lhs[li] != '.') {
            if (lhs[li] >= '0' && lhs[li] <= '9') lv = (lv * 10L) + (lhs[li] - '0');
            li++;
        }
        while (ri < rhs.length() && rhs[ri] != '.') {
            if (rhs[ri] >= '0' && rhs[ri] <= '9') rv = (rv * 10L) + (rhs[ri] - '0');
            ri++;
        }
        if (lv < rv) return -1;
        if (lv > rv) return 1;
        li++;
        ri++;
    }
    return 0;
}

static unsigned long clampIdleTimeoutMs(unsigned long ms)
{
    if (ms < LCD_IDLE_TIMEOUT_MS_MIN) ms = LCD_IDLE_TIMEOUT_MS_MIN;
    if (ms > LCD_IDLE_TIMEOUT_MS_MAX) ms = LCD_IDLE_TIMEOUT_MS_MAX;
    const unsigned long snapped = ((ms + (LCD_IDLE_TIMEOUT_STEP_MS / 2UL)) / LCD_IDLE_TIMEOUT_STEP_MS) * LCD_IDLE_TIMEOUT_STEP_MS;
    if (snapped < LCD_IDLE_TIMEOUT_MS_MIN) return LCD_IDLE_TIMEOUT_MS_MIN;
    if (snapped > LCD_IDLE_TIMEOUT_MS_MAX) return LCD_IDLE_TIMEOUT_MS_MAX;
    return snapped;
}

extern Preferences uiPrefs;

static String formatIdleTimeoutLabel(unsigned long ms)
{
    const unsigned long totalSec = ms / 1000UL;
    const unsigned long mins = totalSec / 60UL;
    const unsigned long secs = totalSec % 60UL;
    if (mins == 0UL) return String(secs) + " sec";
    if (secs == 0UL) return String(mins) + " min";
    return String(mins) + " min " + String(secs) + " sec";
}

static String buildPowerOffTimeoutDropdownOptions()
{
    return String("15 sec\n2 min\n5 min\n15 min\n30 min\nNever");
}

static String buildDeepSleepTimeoutDropdownOptions()
{
    return buildPowerOffTimeoutDropdownOptions();
}

static int powerOffTimeoutOptionIndex(unsigned long ms)
{
    for (int i = 0; i < static_cast<int>(sizeof(POWER_OFF_IDLE_TIMEOUT_OPTIONS_MS) / sizeof(POWER_OFF_IDLE_TIMEOUT_OPTIONS_MS[0])); ++i) {
        if (POWER_OFF_IDLE_TIMEOUT_OPTIONS_MS[i] == ms) return i;
    }
    return static_cast<int>(sizeof(POWER_OFF_IDLE_TIMEOUT_OPTIONS_MS) / sizeof(POWER_OFF_IDLE_TIMEOUT_OPTIONS_MS[0])) - 1;
}

static int deepSleepTimeoutOptionIndex(unsigned long ms)
{
    for (int i = 0; i < static_cast<int>(sizeof(POWER_OFF_IDLE_TIMEOUT_OPTIONS_MS) / sizeof(POWER_OFF_IDLE_TIMEOUT_OPTIONS_MS[0])); ++i) {
        if (POWER_OFF_IDLE_TIMEOUT_OPTIONS_MS[i] == ms) return i;
    }
    return static_cast<int>(sizeof(POWER_OFF_IDLE_TIMEOUT_OPTIONS_MS) / sizeof(POWER_OFF_IDLE_TIMEOUT_OPTIONS_MS[0])) - 1;
}

static bool applyDisplayIdleTimeoutPowerOffCap(bool persist)
{
    unsigned long cappedTimeoutMs = displayIdleTimeoutMs;
    unsigned long terminalIdleTimeoutMs = 0;
    if (powerOffIdleTimeoutMs >= 120000UL) terminalIdleTimeoutMs = powerOffIdleTimeoutMs;
    if (deepSleepIdleTimeoutMs >= 120000UL) {
        if (terminalIdleTimeoutMs == 0 || deepSleepIdleTimeoutMs < terminalIdleTimeoutMs) {
            terminalIdleTimeoutMs = deepSleepIdleTimeoutMs;
        }
    }
    if (terminalIdleTimeoutMs >= 120000UL) {
        const unsigned long minimumTimeoutMs = clampIdleTimeoutMs(terminalIdleTimeoutMs - 60000UL);
        if (displayIdleTimeoutMs < minimumTimeoutMs) cappedTimeoutMs = minimumTimeoutMs;
    }
    if (displayIdleTimeoutMs == cappedTimeoutMs) return false;
    displayIdleTimeoutMs = cappedTimeoutMs;
    if (persist) {
        uiPrefs.begin("ui", false);
        uiPrefs.putULong("disp_idle", displayIdleTimeoutMs);
        uiPrefs.end();
    }
    return true;
}

static void persistDisplayIdleTimeout()
{
    uiPrefs.begin("ui", false);
    uiPrefs.putULong("disp_idle", displayIdleTimeoutMs);
    uiPrefs.end();
}

static String chatSafeFileToken(const String &raw);
static String chatFriendlyLogFilenameForPeer(const String &peerKey);
static String chatLegacyLogPathForPeer(const String &peerKey);
static String chatLogPathForPeer(const String &peerKey);
static bool chatSavePendingOutbox();
static void chatRememberRecentMessageId(const String &peerKey, const String &messageId);
static bool chatRecentMessageIdSeen(const String &peerKey, const String &messageId);
static void chatSchedulePendingOutboxSave(unsigned long delayMs = 200UL);
static void chatPendingOutboxService();
static void chatLoadPendingOutbox();
static bool chatPeerCanUseP2pTransport(const String &peerKey);
static bool chatPeerCanUseMqttTransport(const String &peerKey);
static bool chatPeerCanUseRadioTransport(const String &peerKey);
static bool chatPeerUsesRadioPath(const String &peerKey);
static bool chatPeerBlockedByAirplaneMode(const String &peerKey);
static bool chatHasUnreadMessages();
static void chatSetPeerUnread(const String &peerKey, bool unread);
static bool chatSendRawReliableMessage(const String &peerKey, const String &text, bool storeVisible);
static bool chatSendAndStoreMessage(const String &peerKey, const String &text);
static void chatStageDeferredAirplaneMessage(const String &peerKey, const String &text);
static void chatFlushDeferredAirplaneMessage();
static bool chatMessagePendingForPeer(const String &peerKey, const String &messageId);
static bool chatOpenPeerConversation(const String &peerKey);
static bool chatOpenFirstUnreadConversation();
static void chatScheduleOpenPeerConversation(const String &peerKey);
static void lvglDeferredChatOpenFirstUnreadCallback(void *param);
static bool chatDeleteMessageById(const String &peerKey, const String &messageId, const String &status);
static bool chatDeleteMessageAt(const String &peerKey, int index);
static void chatLoadPendingOutbox();
static unsigned long e220RuntimeBaud();
static unsigned long hc12RuntimeBaud();
static void hc12SerialReopen(unsigned long baud);
static void hc12InitIfNeeded();
static void hc12RestartWithCurrentPins(const String &statusText);
static void loadPersistedRemoteControlSettings();
static void loadPersistedRadioControlButtons();
static void savePersistedRemoteControlSettings();
static bool hc12SetIsAsserted();
static bool checkersHandleIncomingChatPayload(const String &peerKey,
                                              const String &author,
                                              const String &text,
                                              ChatTransport transport,
                                              const String &messageId);
static void checkersClearSelection();
static void checkersClearSession();
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
#if defined(BOARD_ESP32S3_3248S035_N16R8)
static constexpr int MAX_P2P_DISCOVERED = 12;
#else
static constexpr int MAX_P2P_DISCOVERED = 6;
#endif
static constexpr size_t P2P_PUBLIC_KEY_BYTES = crypto_box_curve25519xchacha20poly1305_PUBLICKEYBYTES;
static constexpr size_t P2P_SECRET_KEY_BYTES = crypto_box_curve25519xchacha20poly1305_SECRETKEYBYTES;
static constexpr size_t P2P_NONCE_BYTES = crypto_box_curve25519xchacha20poly1305_NONCEBYTES;
static constexpr size_t P2P_MAC_BYTES = crypto_box_curve25519xchacha20poly1305_MACBYTES;
static constexpr size_t P2P_MAX_CHAT_TEXT = 160;
static constexpr size_t P2P_MAX_PACKET = 512;
P2PPeer *p2pPeers = nullptr;
int p2pPeerCount = 0;

struct P2PDiscoveredPeer {
    String name;
    String pubKeyHex;
    IPAddress ip;
    uint16_t port;
    bool trusted;
    unsigned long lastSeenMs;
};

P2PDiscoveredPeer *p2pDiscoveredPeers = nullptr;
int p2pDiscoveredCount = 0;

static bool p2pEnsurePeerStorage()
{
    if (p2pPeers) return true;
    p2pPeers = static_cast<P2PPeer *>(allocPreferPsram(sizeof(P2PPeer) * MAX_P2P_PEERS));
    if (!p2pPeers) return false;
    memset(p2pPeers, 0, sizeof(P2PPeer) * MAX_P2P_PEERS);
    return true;
}

static bool p2pEnsureDiscoveredStorage()
{
    if (p2pDiscoveredPeers) return true;
    p2pDiscoveredPeers = static_cast<P2PDiscoveredPeer *>(allocPreferPsram(sizeof(P2PDiscoveredPeer) * MAX_P2P_DISCOVERED));
    if (!p2pDiscoveredPeers) return false;
    memset(p2pDiscoveredPeers, 0, sizeof(P2PDiscoveredPeer) * MAX_P2P_DISCOVERED);
    return true;
}

static bool hc12EnsureDiscoveredStorage()
{
    if (hc12DiscoveredPeers) return true;
    hc12DiscoveredPeers = static_cast<Hc12DiscoveredPeer *>(allocPreferPsram(sizeof(Hc12DiscoveredPeer) * MAX_HC12_DISCOVERED));
    if (hc12DiscoveredPeers) memset(hc12DiscoveredPeers, 0, sizeof(Hc12DiscoveredPeer) * MAX_HC12_DISCOVERED);
    return hc12DiscoveredPeers != nullptr;
}

extern bool hc12SwapUartPins;
extern bool e220SwapUartPins;
extern bool e220SwapModePins;

static int hc12ActiveRxPin()
{
    return hc12SwapUartPins ? HC12_TX_PIN_DEFAULT : HC12_RX_PIN_DEFAULT;
}

static int hc12ActiveTxPin()
{
    return hc12SwapUartPins ? HC12_RX_PIN_DEFAULT : HC12_TX_PIN_DEFAULT;
}

static int hc12ActiveSetPin()
{
    return HC12_SET_PIN_DEFAULT;
}

static int e220ActiveRxPin()
{
    return e220SwapUartPins ? E220_TX_PIN_DEFAULT : E220_RX_PIN_DEFAULT;
}

static int e220ActiveTxPin()
{
    return e220SwapUartPins ? E220_RX_PIN_DEFAULT : E220_TX_PIN_DEFAULT;
}

static int e220ActiveM0Pin()
{
    return e220SwapModePins ? E220_M1_PIN_DEFAULT : E220_M0_PIN_DEFAULT;
}

static int e220ActiveM1Pin()
{
    return e220SwapModePins ? E220_M0_PIN_DEFAULT : E220_M1_PIN_DEFAULT;
}

static void hc12PruneDiscoveredPeers()
{
    const unsigned long now = millis();
    for (int i = 0; i < hc12DiscoveredCount; ) {
        if (static_cast<unsigned long>(now - hc12DiscoveredPeers[i].lastSeenMs) > HC12_DISCOVERY_STALE_MS) {
            for (int j = i + 1; j < hc12DiscoveredCount; ++j) {
                hc12DiscoveredPeers[j - 1] = hc12DiscoveredPeers[j];
            }
            hc12DiscoveredCount--;
        } else {
            ++i;
        }
    }
}

static int hc12FindDiscoveredByPubKeyHex(const String &pubKeyHex)
{
    if (!hc12DiscoveredPeers) return -1;
    for (int i = 0; i < hc12DiscoveredCount; ++i) {
        if (hc12DiscoveredPeers[i].pubKeyHex.equalsIgnoreCase(pubKeyHex)) return i;
    }
    return -1;
}

// Legacy unused HC12 discovery helper moved to app/garbage.inc:
// hc12PeerRecentlySeen(const String&, unsigned long)

static void hc12TouchDiscoveredPeer(const String &name, const String &pubKeyHex)
{
    if (pubKeyHex.isEmpty() || pubKeyHex.equalsIgnoreCase(p2pPublicKeyHex())) return;
    if (!hc12EnsureDiscoveredStorage()) return;
    int idx = hc12FindDiscoveredByPubKeyHex(pubKeyHex);
    if (idx < 0) {
        if (hc12DiscoveredCount >= MAX_HC12_DISCOVERED) {
            idx = 0;
            for (int i = 1; i < hc12DiscoveredCount; ++i) {
                if (hc12DiscoveredPeers[i].lastSeenMs < hc12DiscoveredPeers[idx].lastSeenMs) idx = i;
            }
        } else {
            idx = hc12DiscoveredCount++;
        }
        hc12DiscoveredPeers[idx].unread = false;
    }
    if (!name.isEmpty()) hc12DiscoveredPeers[idx].name = sanitizeDeviceShortName(name);
    hc12DiscoveredPeers[idx].pubKeyHex = pubKeyHex;
    hc12DiscoveredPeers[idx].lastSeenMs = millis();
    if (lvglReady) {
        if (uiScreen == UI_CHAT || uiScreen == UI_CHAT_PEERS) lvglRefreshChatContactsUi();
        if (uiScreen == UI_CHAT_PEERS) lvglRefreshChatPeerUi();
    }
}

static bool chatPeerIsSelectable(const String &peerKey)
{
    if (peerKey.isEmpty()) return false;
    if (peerKey == "ha_inbox") return chatPeerHasHistory(peerKey);
    const int idx = p2pFindPeerByPubKeyHex(peerKey);
    if (idx >= 0 && p2pPeers[idx].enabled) return true;
    return hc12FindDiscoveredByPubKeyHex(peerKey) >= 0 || chatPeerHasHistory(peerKey);
}

static bool chatHasUnreadMessages()
{
    if (haChatUnread) return true;
    for (int i = 0; i < p2pPeerCount; ++i) {
        if (p2pPeers[i].unread) return true;
    }
    for (int i = 0; i < hc12DiscoveredCount; ++i) {
        if (hc12DiscoveredPeers[i].unread) return true;
    }
    return false;
}

static void chatSetPeerUnread(const String &peerKey, bool unread)
{
    if (peerKey == "ha_inbox") {
        if (haChatUnread != unread) {
            haChatUnread = unread;
            if (lvglReady) {
                lvglRefreshTopIndicators();
                if (uiScreen == UI_CHAT || uiScreen == UI_CHAT_PEERS) lvglRefreshChatContactsUi();
                if (uiScreen == UI_CHAT_PEERS) lvglRefreshChatPeerUi();
            }
        }
        return;
    }
    const int idx = p2pFindPeerByPubKeyHex(peerKey);
    const int radioIdx = hc12FindDiscoveredByPubKeyHex(peerKey);
    bool changed = false;
    if (idx >= 0) {
        if (p2pPeers[idx].unread != unread) {
            p2pPeers[idx].unread = unread;
            changed = true;
        }
    }
    if (radioIdx >= 0 && hc12DiscoveredPeers[radioIdx].unread != unread) {
        hc12DiscoveredPeers[radioIdx].unread = unread;
        changed = true;
    }
    if (changed && lvglReady) {
        lvglRefreshTopIndicators();
        if (uiScreen == UI_CHAT || uiScreen == UI_CHAT_PEERS) lvglRefreshChatContactsUi();
        if (uiScreen == UI_CHAT_PEERS) lvglRefreshChatPeerUi();
    }
}

// Snake, Snake 3D, Tetris, and Checkers state/constants moved to app/games_state.inc.
#include "app/games_state.inc"

struct GameBoardLayout {
    lv_coord_t width;
    lv_coord_t height;
    lv_coord_t scoreHeight;
    lv_coord_t controlsHeight;
    lv_coord_t boardTopOffset;
    lv_coord_t buttonWidth;
    lv_coord_t buttonHeight;
};

struct MediaEntry {
    String name;
    String path;
    bool isDir;
    size_t size;
};

static constexpr int MAX_MEDIA_ENTRIES = 68;
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
bool chatMessageBeepPinAttached = false;
uint8_t chatMessageBeepPhase = 0;
uint8_t chatMessageBeepQueue = 0;
unsigned long chatMessageBeepDeadlineMs = 0;
bool touchSoundEnabled = true;
bool startupSoundEnabled = true;
bool touchFeedbackBeepActive = false;
unsigned long touchFeedbackBeepDeadlineMs = 0;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
bool chatMessageVibrationPinAttached = false;
bool chatMessageVibrationPinOutput = false;
uint8_t chatMessageVibrationPhase = 0;
uint8_t chatMessageVibrationQueue = 0;
unsigned long chatMessageVibrationDeadlineMs = 0;
bool touchVibrationEnabled = true;
bool touchFeedbackVibrationActive = false;
unsigned long touchFeedbackVibrationDeadlineMs = 0;
#else
bool touchVibrationEnabled = true;
#endif

Preferences wifiPrefs;
Preferences batteryPrefs;
Preferences mqttPrefs;
Preferences uiPrefs;
Preferences p2pPrefs;
Preferences gamePrefs;
String *hc12TerminalLog = nullptr;
String hc12InfoValueText = "--";
String hc12InfoSubText = "Open Info to query module";
String radioInfoVersionText = "--";
String radioInfoBaudText = "--";
String radioInfoChannelText = "--";
String radioInfoFuModeText = "--";
String radioInfoPowerText = "--";
String radioInfoRawText = "--";
String infoWifiValueText = "--";
String infoWifiSubText = "Loading...";
uint8_t infoWifiQualityPercent = 0;
String infoLightValueText = "--";
String infoLightSubText = "Loading...";
uint8_t infoLightPercentValue = 0;
String infoSdValueText = "--";
String infoSdSubText = "Loading...";
uint8_t infoSdPercentValue = 0;
String infoTempValueText = "--";
String infoTempSubText = "Loading...";
uint8_t infoTempBarValue = 0;
String infoSramValueText = "--";
String infoSramSubText = "Loading...";
uint8_t infoSramPercentValue = 0;
String infoPsramValueText = "--";
String infoPsramSubText = "Loading...";
uint8_t infoPsramPercentValue = 0;
bool hc12SwapUartPins = false;
int hc12CurrentChannel = HC12_MIN_CHANNEL;
int hc12CurrentBaudIndex = 3;
int hc12CurrentModeIndex = 2;
int hc12CurrentPowerLevel = 8;
bool e220SwapUartPins = false;
bool e220SwapModePins = false;
int e220CurrentChannel = 23;
int e220CurrentBaudIndex = 3;
int e220CurrentAirRateIndex = 2;
int e220CurrentPowerIndex = 0;
bool e220CurrentFixedTransmission = false;
String hc12ConfigStatusText = "Read current settings from module";
String *hc12RadioRxLine = nullptr;
unsigned long hc12LastDiscoveryAnnounceMs = 0;
char *serialLogRing = nullptr;
size_t serialLogHead = 0;
size_t serialLogCount = 0;
uint32_t serialLastWsPushMs = 0;
uint32_t serialLogWsMinIntervalMs = SERIAL_LOG_RATE_MS_DEFAULT;
size_t serialLogKeepLines = SERIAL_LOG_RING_SIZE;
unsigned long serialTerminalStreamUntilMs = 0;
uint16_t hc12SettleUntilTick = 0;
enum Hc12ConfigApplyAction : uint8_t {
    HC12_CFG_APPLY_NONE = 0,
    HC12_CFG_APPLY_CHANNEL,
    HC12_CFG_APPLY_BAUD,
    HC12_CFG_APPLY_MODE,
    HC12_CFG_APPLY_POWER,
    HC12_CFG_APPLY_EXTRA
};
Hc12ConfigApplyAction hc12ConfigApplyPending = HC12_CFG_APPLY_NONE;
int hc12ConfigApplyTarget = 0;
unsigned long hc12ConfigApplyDueMs = 0;
int hc12CommittedChannel = HC12_MIN_CHANNEL;
int hc12CommittedBaudIndex = 3;
int hc12CommittedModeIndex = 2;
int hc12CommittedPowerLevel = 8;
int e220CommittedChannel = 23;
int e220CommittedBaudIndex = 3;
int e220CommittedAirRateIndex = 2;
int e220CommittedPowerIndex = 0;
bool e220CommittedFixedTransmission = false;
bool recordTelemetryEnabled = false;
bool systemSoundsEnabled = true;
bool wsRebootOnDisconnectEnabled = false;
bool airplaneModeEnabled = false;
bool menuCustomIconsEnabled = true;
bool homeChatVisible = true;
bool homeRadioVisible = true;
bool homeMediaVisible = true;
bool homeInfoVisible = true;
bool homeGamesVisible = true;
bool homeMqttVisible = true;
bool homeConfigVisible = true;
bool homePowerVisible = true;
bool homeAirplaneVisible = true;
bool homeApVisible = true;
bool moduleInstalled[APP_MODULE_COUNT] = {};
bool moduleAvailable[APP_MODULE_COUNT] = {};
String moduleRemoteVersion[APP_MODULE_COUNT];
String moduleRemoteNotes[APP_MODULE_COUNT];
String moduleRemotePackageUrl[APP_MODULE_COUNT];
String modulesStatusText;
volatile bool moduleCatalogCancelRequested = false;
volatile bool moduleCatalogUiRefreshPending = false;
volatile bool moduleCatalogRestartPending = false;
uint32_t moduleCatalogRequestToken = 0;
bool screenLockEnabled = false;
bool screenLockUnlocked = true;
char screenLockPin[5] = "";
bool configLockEnabled = true;
char configLockPin[5] = "1111";
uint8_t screenLockFailedTotal = 0;
unsigned long screenLockBlockedUntilMs = 0;
unsigned long screenLockLastUiRefreshMs = 0;
UiButtonStyleMode uiButtonStyleMode = UI_BUTTON_STYLE_3D;
uint32_t uiButtonStyleFlatSelectorColor = 0;
uint32_t uiButtonStyle3dSelectorColor = 0;
uint8_t uiDeferredFlags = 0;
enum OtaUiState : uint8_t {
    OTA_UI_IDLE = 0,
    OTA_UI_CHECKING,
    OTA_UI_AVAILABLE,
    OTA_UI_UP_TO_DATE,
    OTA_UI_DOWNLOADING,
    OTA_UI_FINALIZING,
    OTA_UI_DONE,
    OTA_UI_ERROR
};
volatile OtaUiState otaUiState = OTA_UI_IDLE;
volatile uint8_t otaProgressPercent = 0;
bool otaUpdateAvailable = false;
bool otaCheckRequested = false;
bool otaBootCheckPending = true;
bool otaPostUpdatePopupPending = false;
bool otaUpdatePromptPending = false;
unsigned long otaLastCheckMs = 0;
unsigned long otaNextAllowedCheckMs = 0;
char otaLatestVersion[OTA_VERSION_TEXT_MAX] = "";
char otaPendingPopupVersion[OTA_VERSION_TEXT_MAX] = "";
String otaPendingPopupNotes;
String otaStatusText;
String otaLatestBinUrl;
OtaReleaseEntry otaReleaseEntries[OTA_RELEASE_LIST_MAX];
uint8_t otaReleaseEntryCount = 0;
int8_t otaSelectedReleaseIndex = -1;
TaskHandle_t moduleCatalogTaskHandle = nullptr;
TaskHandle_t otaCheckTaskHandle = nullptr;
TaskHandle_t otaUpdateTaskHandle = nullptr;
bool p2pUdpStarted = false;
bool p2pReady = false;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
static int p2pUdpLastParseError = 0;
static unsigned long p2pUdpLastParseErrorMs = 0;
#endif
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
bool wifiSessionApMode = false;
bool wifiForgetPendingUi = false;
bool wifiEventsRegistered = false;
volatile bool wifiStaGotIpPending = false;
volatile bool wifiStaDisconnectedPending = false;
volatile uint8_t wifiStaDisconnectReasonPending = 0;
wifi_event_id_t wifiStaGotIpEventId = 0;
wifi_event_id_t wifiStaDisconnectedEventId = 0;
unsigned long lastBatterySnapshotMs = 0;
float lastBatterySnapshotVoltage = -1.0f;
BatteryCalibrationState batteryCalState;
BatteryTrainingState batteryTrainingState;
unsigned long lvglNextHandlerDueMs = 0;
unsigned long lastCarInputTelemetryMs = 0;
bool rebootRequested = false;
unsigned long rebootRequestedAtMs = 0;
uint32_t cpuLoadPrevLoopStartUs = 0;
uint32_t cpuLoadPrevActiveUs = 0;
uint32_t cpuLoadAccumActiveUs = 0;
uint32_t cpuLoadAccumTotalUs = 0;
unsigned long cpuLoadWindowStartMs = 0;

static constexpr unsigned long BG_SERVICE_INTERVAL_MS = 12UL;
static constexpr unsigned long BG_SERVICE_INTERVAL_UI_PRIORITY_MS = 36UL;
// Legacy unused per-service interval constants were moved to app/garbage.inc:
// WIFI_SCAN_SERVICE_INTERVAL_MS, WIFI_CONN_SERVICE_INTERVAL_MS, SD_STATS_SERVICE_INTERVAL_MS,
// MDNS_SERVICE_INTERVAL_MS, P2P_SERVICE_INTERVAL_MS, CHAT_PENDING_SERVICE_INTERVAL_MS,
// RGB_SERVICE_INTERVAL_MS, CAR_TELEMETRY_SERVICE_INTERVAL_MS
#include "app/mqtt.inc"
#include "app/web_html.inc"

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

    if (VERBOSE_SERIAL_DEBUG) {
        Serial.printf("[TOUCH] gt911 probe addr=0x%02X pid=%02X %02X %02X %02X text='%c%c%c%c'%s\n",
                      static_cast<unsigned int>(addr),
                      productId[0], productId[1], productId[2], productId[3],
                      printable ? productId[0] : '.',
                      printable ? productId[1] : '.',
                      printable ? productId[2] : '.',
                      printable ? productId[3] : '.',
                      printable ? "" : " nonprint");
    }
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
    if (VERBOSE_SERIAL_DEBUG) {
        Serial.printf("[TOUCH] init ok=%d i2c=%lu irq=%d\n",
                      ok ? 1 : 0,
                      static_cast<unsigned long>(touchI2cHzActive),
                      digitalRead(TOUCH_IRQ));
    }
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
    if (VERBOSE_SERIAL_DEBUG) {
        Serial.printf("[TOUCH] gt911 init ok=%d addr=0x%02X i2c=%lu irq=%d pid=%02X %02X %02X %02X\n",
                      ok ? 1 : 0,
                      static_cast<unsigned int>(touchI2cAddrActive),
                      static_cast<unsigned long>(touchI2cHzActive),
                      digitalRead(TOUCH_IRQ),
                      detectedPid[0], detectedPid[1], detectedPid[2], detectedPid[3]);
    }
}

// LVGL/TFT core runtime, touch handling, and display helpers moved to app/ui_display.inc.
#define UI_DISPLAY_SECTION_CORE
#include "app/ui_display.inc"
#undef UI_DISPLAY_SECTION_CORE

#include "app/battery.inc"

// TFT display helper functions moved to app/ui_display.inc (UI_DISPLAY_SECTION_CORE).
#include "app/wifi_scan.inc"

#include "app/games_snake.inc"
#include "app/games_snake3d.inc"

#include "app/radio_hc12_ui.inc"

#include "app/games_tetris.inc"

#include "app/games_checkers.inc"

#include "app/media_audio.inc"

void saveApCreds(const String &ssid, const String &pass)
{
    savedApSsid = ssid;
    savedApPass = pass;
    normalizeSavedApCreds();
    wifiPrefs.begin("wifi", false);
    wifiPrefs.putString("ap_ssid", savedApSsid);
    wifiPrefs.putString("ap_pass", savedApPass);
    wifiPrefs.end();
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

#include "app/ui_runtime.inc"

String httpsGetText(const String &url, int *statusOut)
{
    if (statusOut) *statusOut = -1;
    if (!wifiConnectedSafe()) return "";

    WiFiClientSecure client;
    client.setInsecure();
    client.setHandshakeTimeout(12);

    HTTPClient http;
    http.setConnectTimeout(6000);
    http.setTimeout(8000);
    http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    if (!http.begin(client, url)) return "";

    http.addHeader("Accept", url.indexOf("raw.githubusercontent.com") >= 0 ? "application/json" : "application/vnd.github+json");
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

    const String boardNeedle = String(OTA_ASSET_NAME);

    for (JsonObjectConst asset : arr) {
        const String name = asset["name"] | "";
        const String url = asset["browser_download_url"] | "";
        if (url.isEmpty()) continue;
        const String low = name;
        if (!low.endsWith(".bin") || low.indexOf("bootloader") >= 0 || low.indexOf("partitions") >= 0) continue;
        if (low.indexOf(boardNeedle) >= 0) return url;
    }
    for (JsonObjectConst asset : arr) {
        const String name = asset["name"] | "";
        const String url = asset["browser_download_url"] | "";
        if (url.isEmpty()) continue;
        const String low = name;
        if (!low.endsWith(".bin") || low.indexOf("bootloader") >= 0 || low.indexOf("partitions") >= 0) continue;
        if (low.endsWith(".ino.bin")) return url;
    }
    for (JsonObjectConst asset : arr) {
        const String name = asset["name"] | "";
        const String url = asset["browser_download_url"] | "";
        if (url.isEmpty()) continue;
        const String low = name;
        if (low.endsWith(".bin") && low.indexOf("bootloader") < 0 && low.indexOf("partitions") < 0) return url;
    }
    return "";
}

#include "app/ota_backend.inc"
#include "app/web_routes.inc"
void setup()
{
    sdMutex = xSemaphoreCreateMutex();
    Serial.begin(115200);
    otaFinalizePendingBootImage();
    if (VERBOSE_SERIAL_DEBUG) {
        Serial.println();
        Serial.printf("[BOOT] setup start board=%s\n", deviceShortNameValue().c_str());
        Serial.println("[BOOT] step displayBacklightInit");
    }
    displayBacklightInit();
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step rgb pinMode");
    if (RGB_OUTPUT_SUPPORTED) {
        pinMode(RGB_PIN_R, OUTPUT);
        pinMode(RGB_PIN_G, OUTPUT);
        pinMode(RGB_PIN_B, OUTPUT);
    } else if (VERBOSE_SERIAL_DEBUG) {
        Serial.println("[RGB] disabled on this board; configured pins overlap ESP32-S3 octal PSRAM");
    }
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step touch irq pinMode");
    pinMode(TOUCH_IRQ, TOUCH_USE_IRQ ? INPUT_PULLUP : INPUT);
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step rgbApplyNow");
    rgbApplyNow(false, false, false);
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step analogReadResolution");
    analogReadResolution(12);
    pinMode(BATTERY_ADC_PIN, INPUT);
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step battery attenuation");
    analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
    if (LIGHT_ADC_PIN >= 0) {
        if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step light attenuation");
        analogSetPinAttenuation(LIGHT_ADC_PIN, ADC_11db);
    }
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step vibration pin init");
    pinMode(VIBRATION_MOTOR_PIN, OUTPUT);
    digitalWrite(VIBRATION_MOTOR_PIN, LOW);
    if (POWER_OFF_SIGNAL_PIN >= 0) {
        if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step power signal pin init");
        pinMode(POWER_OFF_SIGNAL_PIN, OUTPUT);
        digitalWrite(POWER_OFF_SIGNAL_PIN, HIGH);
    }
#endif
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step randomSeed");
    randomSeed(static_cast<unsigned long>(esp_random()));
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step loadUiRuntimeConfig");
    loadUiRuntimeConfig();
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step sodium_init");
    if (moduleNeedsP2pRuntime()) {
        p2pReady = sodium_init() >= 0;
        if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step loadP2pConfig");
        if (p2pReady) loadP2pConfig();
    } else {
        p2pReady = false;
    }
    batteryCalibrationLoad();
    batteryTrainingLoad();
    loadGamePrefs();
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step mqttBuildIdentity");
    mqttBuildIdentity();
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step loadMqttConfig");
    loadMqttConfig();
    if (mqttCfg.enabled) mqttStatusLine = "Enabled";
    else mqttStatusLine = "Disabled";
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step sampleTopIndicators");
    sampleTopIndicators();
    BatterySnapshot prevBatterySnapshot;
    if (loadBatterySnapshot(prevBatterySnapshot)) {
        const float prevBootV = batteryCalibratedVoltageFromRaw(prevBatterySnapshot.rawV);
        if ((batteryVoltage - prevBootV) >= BATTERY_BOOT_CHARGE_DELTA_V) {
            chargeTrendScore = CHARGE_SCORE_ON;
            batteryCharging = true;
            lastChargeSeenMs = millis();
        }
        const esp_reset_reason_t resetReason = esp_reset_reason();
        const bool inferEmptyFromPreviousCycle =
            resetReason == ESP_RST_POWERON &&
            !prevBatterySnapshot.charging &&
            prevBatterySnapshot.uptimeMs >= BATTERY_BOOT_EMPTY_MIN_UPTIME_MS &&
            prevBatterySnapshot.rawV <= BATTERY_BOOT_EMPTY_INFER_MAX_V;
        const bool allowEmptyLearning =
            batteryTrainingState.autoCalibrationEnabled ||
            (batteryTrainingState.active && batteryTrainingState.phase == BATTERY_TRAIN_DISCHARGE);
        if (allowEmptyLearning && inferEmptyFromPreviousCycle) {
            batteryCalibrationLearnEmpty(prevBatterySnapshot.rawV);
            if (batteryTrainingState.active && batteryTrainingState.phase == BATTERY_TRAIN_DISCHARGE) {
                batteryTrainingStopManual();
            }
        }
    }
    lastBatterySnapshotVoltage = batteryVoltage;
    lastBatterySnapshotMs = millis();

    if (VERBOSE_SERIAL_DEBUG) {
        Serial.printf("%s UI + fallback file manager\n", deviceShortNameValue().c_str());
        Serial.printf("[P2P] ready=%d pub=%s\n", p2pReady ? 1 : 0, p2pReady ? p2pPublicKeyHex().c_str() : "-");
    }

    tft.init();
    tft.setRotation(TFT_ROTATION);
    tft.setTextFont(2);
    tft.fillScreen(TFT_BLACK);

    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] touch pre-init");
    touchInit();
    lvglInitUi();
    if (lvglReady) {
        if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step lv_timer_handler");
        lv_timer_handler();
        if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step lv_timer_handler done");
        delay(20);
    }
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step displayBacklightFadeIn");
    displayBacklightFadeIn();
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step displayBacklightFadeIn done");
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] SD/network init deferred to loop");
    bootDeferredStartMs = millis();
    bootInteractiveUntilMs = bootDeferredStartMs + BOOT_INTERACTIVE_GRACE_MS;
    bootSdInitPending = true;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] SD boot init enabled on ESP32-S3 after RGB/PSRAM fix");
#endif
    bootWifiInitPending = true;
    bootStartupFeedbackPending = true;
    bootStartupFeedbackDueMs = bootDeferredStartMs + BOOT_DEFER_STARTUP_FEEDBACK_MS;
    bootRadioInitPending = moduleInstalled[APP_MODULE_RADIO];
    bootRadioInitDueMs = bootDeferredStartMs + BOOT_DEFER_RADIO_INIT_MS;
    bootRadioProbePending = moduleInstalled[APP_MODULE_RADIO] && radioModuleType == RADIO_MODULE_E220;
    bootRadioProbeDueMs = bootDeferredStartMs + BOOT_DEFER_RADIO_PROBE_MS;
    wifiRuntimeManaged = true;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] WiFi boot init enabled on ESP32-S3 after RGB/PSRAM fix");
#endif
    delay(20);
    sdStatsLogSnapshot(sdStatsSnapshot(), "boot");
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[AUDIO] backend init deferred until playback");
    rgbRefreshByMediaState();
    lastUserActivityMs = millis();
    lastSensorSampleMs = millis();

#if defined(BOARD_ESP32S3_3248S035_N16R8)
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step speaker pin quiet");
    //pinMode(I2S_SPK_PIN, OUTPUT);
    //digitalWrite(I2S_SPK_PIN, LOW);
    ledcDetachPin(I2S_SPK_PIN);
    pinMode(I2S_SPK_PIN, INPUT);
    gpio_set_pull_mode((gpio_num_t)I2S_SPK_PIN, GPIO_FLOATING);

    gpio_set_drive_capability((gpio_num_t)I2S_SPK_PIN, GPIO_DRIVE_CAP_0);
    if (VERBOSE_SERIAL_DEBUG) {
        Serial.printf("Audio output is disabled on this ESP32-S3 build; GPIO %d is reserved until external I2S pinout is wired in firmware\n", I2S_SPK_PIN);
    }
#else
    if (VERBOSE_SERIAL_DEBUG) {
        Serial.printf("Audio decoder ready, internal DAC2 on GPIO %d only (GPIO25 kept for TOUCH_RST)\n", I2S_SPK_PIN);
    }
#endif
    if (VERBOSE_SERIAL_DEBUG) {
        Serial.printf("[TOUCH] ctrl=%s mode=%s irq_pin=%d i2c=(sda=%d,scl=%d,rst=%d)\n",
                      TOUCH_CONTROLLER == TOUCH_CTRL_GT911 ? "gt911" : "cst820",
                      TOUCH_USE_IRQ ? "irq+poll" : "poll-only",
                      TOUCH_IRQ, TOUCH_SDA, TOUCH_SCL, TOUCH_RST);
    }
}

static inline void cpuBoostForUi(bool uiPriorityActive)
{
#if defined(ESP32)
    if (uiPriorityActive) {
        if (getCpuFrequencyMhz() < 240) {
            setCpuFrequencyMhz(240);
        }
    } else {
        if (getCpuFrequencyMhz() > 80) {
            setCpuFrequencyMhz(80);
        }
    }
#endif
}

void loop()
{
    const uint32_t loopStartUs = micros();
    cpuLoadService(loopStartUs);
    lvglService();
    screensaverService();
    bool isDown = lvglReady ? lvglTouchDown : false;
    const bool uiPriorityActive = uiPerformancePriorityActive(isDown);
    cpuBoostForUi(uiPriorityActive);
    lvglWarmupScreensService(uiPriorityActive);
    const bool realtimeMessaging = uiScreenNeedsRealtimeMessaging();

    const unsigned long now = millis();
    const bool modulesRuntimeActive = moduleNeedsRuntime();
    const bool bootInteractiveGraceActive = bootInteractiveUntilMs != 0 && static_cast<long>(now - bootInteractiveUntilMs) < 0;

    if (bootSdInitPending && static_cast<unsigned long>(now - bootDeferredStartMs) >= BOOT_DEFER_SD_MS) {
        bootSdInitPending = false;
        setupSd();
        sdStatsLogSnapshot(sdStatsSnapshot(), "boot_deferred_sd");
    }
    if (bootWifiInitPending && static_cast<unsigned long>(now - bootDeferredStartMs) >= BOOT_DEFER_WIFI_MS) {
        bootWifiInitPending = false;
        setupWifiAndServer();
    }
    if (bootStartupFeedbackPending && static_cast<long>(now - bootStartupFeedbackDueMs) >= 0 && !isDown && !uiPriorityActive) {
        bootStartupFeedbackPending = false;
        playStartupFeedbackIfEnabled();
    }
    if (bootRadioInitPending && static_cast<long>(now - bootRadioInitDueMs) >= 0 && !isDown && !uiPriorityActive) {
        bootRadioInitPending = false;
        hc12InitIfNeeded();
    }
    if (bootRadioProbePending && !bootRadioInitPending && static_cast<long>(now - bootRadioProbeDueMs) >= 0 && !isDown && !uiPriorityActive) {
        bootRadioProbePending = false;
        hc12ReadConfigSelection();
    }

    static unsigned long lastServiceSliceMs = 0;
    static uint8_t serviceSlicePhase = 0;
    const unsigned long bgSliceIntervalMs =
        uiPriorityActive ? BG_SERVICE_INTERVAL_UI_PRIORITY_MS : BG_SERVICE_INTERVAL_MS;
    const bool deferNetworkWork = uiPriorityActive;
    if (static_cast<unsigned long>(now - lastServiceSliceMs) >= bgSliceIntervalMs) {
        lastServiceSliceMs = now;
        serviceSlicePhase = static_cast<uint8_t>((serviceSlicePhase + 1U) % 5U);
        if (dnsRunning) dnsServer.processNextRequest();
        rgbService();

        if (!modulesRuntimeActive) {
            // Minimal runtime path for smooth UI when no modules are installed.
            if (serviceSlicePhase == 0 && !deferNetworkWork) {
                wifiConnectionService();
            }
            if (!uiPriorityActive || uiScreen == UI_WIFI_LIST) {
                wifiScanService();
            }
            if (!uiPriorityActive || uiScreen == UI_INFO) serviceInfoRefresh();
            if (!uiPriorityActive) {
                if (!bootInteractiveGraceActive) refreshMdnsState();
                if (!bootInteractiveGraceActive || uiScreen == UI_CONFIG_OTA) otaCheckService();
            }
        } else {
            switch (serviceSlicePhase) {
                case 0:
                    if (!deferNetworkWork) {
                        wifiConnectionService();
                        if (moduleInstalled[APP_MODULE_MQTT] && (!uiPriorityActive || realtimeMessaging) && !bootInteractiveGraceActive) mqttService();
                    }
                    break;

                case 1:
                    if (moduleNeedsP2pRuntime() && p2pReady && (!uiPriorityActive || realtimeMessaging)) p2pService();
                    break;

                case 2:
                    if (!uiPriorityActive || uiScreen == UI_WIFI_LIST) wifiScanService();
                    if (moduleInstalled[APP_MODULE_CHAT] && (!uiPriorityActive || realtimeMessaging)) {
                        chatPendingService();   // keep for WiFi/MQTT generic outbox maintenance
                    }
                    break;

                case 3:
                    if (!uiPriorityActive || !sdMounted) sdStatsService();
                    if (!uiPriorityActive) serviceCarInputTelemetry();
                    if (!uiPriorityActive || uiScreen == UI_INFO) serviceInfoRefresh();

                    if (!radioControlTxInProgress() && (!uiPriorityActive || uiScreen == UI_INFO || uiScreen == UI_CONFIG_HC12_INFO)) {
                        serviceDeferredRadioInfoFetch();
                    }
                    break;

                case 4:
                    if (moduleInstalled[APP_MODULE_MQTT] && !uiPriorityActive) mqttPruneDiscoveredPeers();
                    if (moduleInstalled[APP_MODULE_RADIO] && !uiPriorityActive) hc12PruneDiscoveredPeers();

                    if (!uiPriorityActive && !bootInteractiveGraceActive) refreshMdnsState();
                    if ((!uiPriorityActive || uiScreen == UI_CONFIG_OTA) && (!bootInteractiveGraceActive || uiScreen == UI_CONFIG_OTA)) otaCheckService();
                    break;
            }
        }
    }
    moduleCatalogService();
    otaUpdateService();
    if (modulesRuntimeActive) {
        if (moduleInstalled[APP_MODULE_RADIO] && !bootRadioInitPending) serviceRadioControlTxQueue();
        if (moduleInstalled[APP_MODULE_RADIO] && !bootRadioInitPending && !radioControlTxInProgress()) hc12Service();
        if (moduleInstalled[APP_MODULE_RADIO] && !bootRadioInitPending && !radioControlTxInProgress() && (!uiPriorityActive || uiScreen == UI_CONFIG_HC12) && hc12ConfigApplyPending != HC12_CFG_APPLY_NONE) {
            serviceDeferredHc12ConfigApply();
        }
        if (moduleInstalled[APP_MODULE_RADIO] && !bootRadioInitPending && !radioControlTxInProgress() && moduleInstalled[APP_MODULE_CHAT]) radioChatService();
        if (moduleInstalled[APP_MODULE_CHAT] && (!uiPriorityActive || realtimeMessaging || chatPendingCount > 0)) {
            chatPendingService();
        }
        if (moduleInstalled[APP_MODULE_CHAT]) chatPendingOutboxService();
    }
    const bool allowSdAutoRetry = (moduleInstalled[APP_MODULE_MEDIA] && uiScreen == UI_MEDIA) || !displayAwake;
    if (!sdMounted && allowSdAutoRetry && !isDown && !fsWriteBusy() &&
        static_cast<unsigned long>(millis() - sdLastAutoRetryMs) >= SD_AUTORETRY_PERIOD_MS) {
        sdLastAutoRetryMs = millis();
        portENTER_CRITICAL(&sdStatsMux);
        sdStats.autoRetryCalls++;
        portEXIT_CRITICAL(&sdStatsMux);
        sdEnsureMounted(true);
    }
    if (rgbFlashUntilMs != 0 && static_cast<long>(millis() - rgbFlashUntilMs) >= 0) {
        rgbFlashUntilMs = 0;
        rgbApplyNow(rgbPersistR, rgbPersistG, rgbPersistB);
    }
    bool wasRunning = mediaIsPlaying || mediaPaused;
    // Keep decoder fed continuously; sparse servicing causes audible clicks.
    if (moduleInstalled[APP_MODULE_MEDIA] || mediaIsPlaying || mediaPaused) audioService();
    chatMessageBeepService();
    chatMessageVibrationService();
    touchFeedbackBeepService();
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    touchFeedbackVibrationService();
#endif
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

    const unsigned long sensorPeriodMs = uiPriorityActive ? (SENSOR_SAMPLE_PERIOD_MS * 2UL) : SENSOR_SAMPLE_PERIOD_MS;
    if (!isDown && millis() - lastSensorSampleMs >= sensorPeriodMs) {
        lastSensorSampleMs = millis();
        sampleTopIndicators();
    }
    static unsigned long lastBatteryTrainUiRefreshMs = 0;
    if (uiScreen == UI_CONFIG_BATTERY && static_cast<unsigned long>(millis() - lastBatteryTrainUiRefreshMs) >= 1000UL) {
        lastBatteryTrainUiRefreshMs = millis();
        lvglRefreshBatteryTrainUi();
    }

    if (rebootRequested && static_cast<unsigned long>(millis() - rebootRequestedAtMs) >= 150UL) {
        delay(20);
        ESP.restart();
    }

    if (uiScreenKeepsDisplayAwake()) {
        lastUserActivityMs = millis();
        if (screensaverActive) screensaverSetActive(false);
        if (!displayAwake) displaySetAwake(true);
    } else if (deepSleepIdleTimeoutMs > 0 &&
               !isDown &&
               !mediaIsPlaying &&
               !mediaPaused &&
               !wifiConnectedSafe() &&
               !fsWriteBusy() &&
               !bootStaConnectInProgress &&
               (millis() - lastUserActivityMs >= deepSleepIdleTimeoutMs) &&
               (powerOffIdleTimeoutMs == 0 ||
                POWER_OFF_SIGNAL_PIN < 0 ||
                deepSleepIdleTimeoutMs <= powerOffIdleTimeoutMs)) {
        uiStatusLine = "Idle deep sleep";
        if (lvglReady) lvglSyncStatusLine();
        enterDeepSleep();
    } else if (powerOffIdleTimeoutMs > 0 &&
               POWER_OFF_SIGNAL_PIN >= 0 &&
               !isDown &&
               !batteryCharging &&
               !mediaIsPlaying &&
               !mediaPaused &&
               (millis() - lastUserActivityMs >= powerOffIdleTimeoutMs)) {
        uiStatusLine = "Idle power-off";
        if (lvglReady) lvglSyncStatusLine();
        powerOffSignalPulse();
        lastUserActivityMs = millis();
    } else if (!displayAwake && (millis() - lastUserActivityMs >= displayIdleTimeoutMs)) {
        // Keep state as-is while sleeping.
    } else if (!screensaverActive && displayAwake && (millis() - lastUserActivityMs >= displayIdleTimeoutMs)) {
        bool allowSleep = true;
        if (batteryCharging) allowSleep = animateChargingBeforeSleep();
        if (allowSleep) {
            if (screensaverEnabled) screensaverSetActive(true);
            else displaySetAwake(false);
        }
    }

    if (!displayAwake) {
        wakeTouchConfirmCount = 0;
    }

    if (displayAwake && isDown) lastUserActivityMs = millis();
    displayApplyBacklightPolicy(false);

    if (!bootInteractiveGraceActive && topBarCenterMode == TOP_BAR_CENTER_TIME) syncInternetTimeIfNeeded(false);

    if (moduleInstalled[APP_MODULE_GAMES]) {
        snakeTick();
        tetrisTick();
        tetrisAnimationService();
        checkersTick();
        snake3dTick();
    }

    static wl_status_t lastWiFi = WL_DISCONNECTED;
    wl_status_t cur = wifiStatusSafe();

        if (cur != lastWiFi) {
        if (cur == WL_CONNECTED) {
            bootStaConnectInProgress = false;
            const String ssid = wifiSsidSafe();
            uiStatusLine = "Connected: " + ssid;
            if (OTA_FIRMWARE_FLASH_SUPPORTED && otaLastCheckMs == 0 && !otaCheckTaskHandle && !otaUpdateTaskHandle) {
                otaCheckRequested = true;
            }
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
            if (uiScreen == UI_INFO) lvglRefreshInfoPanel(false);
            if (uiScreen == UI_WIFI_LIST) lvglRefreshWifiList();
            if (lvglMqttStatusLabel) lv_label_set_text_fmt(lvglMqttStatusLabel, "MQTT: %s", mqttStatusLine.c_str());
        }
        lastWiFi = cur;
    }

    unsigned long topIndicatorRefreshMs =
        bootStaConnectInProgress ? TOP_INDICATOR_WIFI_CONNECT_ANIM_MS : TOP_INDICATOR_REFRESH_MS;
    if (uiPriorityActive) topIndicatorRefreshMs *= 2UL;
    if (lvglReady && displayAwake && millis() - lastTopIndicatorRefreshMs >= topIndicatorRefreshMs) {
        lastTopIndicatorRefreshMs = millis();
        lvglRefreshTopIndicators();
        if (uiScreen == UI_INFO && !uiPriorityActive) lvglRefreshInfoPanel(false);
    }

    // Service audio again after UI/network work to reduce underruns.
    audioService();

    if (!isDown && millis() - lastBatterySnapshotMs >= BATTERY_SNAPSHOT_PERIOD_MS) {
        sampleTopIndicators();
        const float dv = (lastBatterySnapshotVoltage < 0.0f) ? 999.0f : fabsf(batteryVoltage - lastBatterySnapshotVoltage);
        const bool dueByDelta = dv >= BATTERY_SNAPSHOT_MIN_DELTA_V;
        const bool dueByForce = (millis() - lastBatterySnapshotMs) >= BATTERY_SNAPSHOT_FORCE_MS;
        if (dueByDelta || dueByForce) {
            saveBatterySnapshot(batteryRawVoltage, batteryCharging, millis());
            lastBatterySnapshotVoltage = batteryVoltage;
            lastBatterySnapshotMs = millis();
        }
    }

    if (canEnterLowPowerSleep(isDown)) enterLowPowerSleep();
    cpuLoadPrevActiveUs = micros() - loopStartUs;

    // Lower CPU load when idle (no UI interaction and only base/INFO module runtime).
    if (!uiPriorityActive && !modulesRuntimeActive) {
        delay(1);
    } else {
        delay(0);
    }
}

