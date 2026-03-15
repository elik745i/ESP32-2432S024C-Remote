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
#include <esp_ota_ops.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
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
#include "generated/img_screenshot_small_icon.h"
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

static constexpr uint32_t UI_BUTTON_BLACK_BODY_HEX = 0x1A1A1A;
static constexpr uint32_t UI_BUTTON_BLACK_PRESSED_HEX = 0x131313;
static constexpr uint32_t UI_BUTTON_BLACK_FLASH_HEX = 0x2B2B2B;
static constexpr uint32_t UI_BUTTON_BLACK_BORDER_HEX = 0x343434;
static constexpr uint32_t UI_BUTTON_BLACK_BORDER_ACTIVE_HEX = 0x666666;

static constexpr int DISPLAY_WIDTH = TFT_WIDTH;
static constexpr int DISPLAY_HEIGHT = TFT_HEIGHT;
static constexpr int DISPLAY_CENTER_X = DISPLAY_WIDTH / 2;
static constexpr int DISPLAY_CENTER_Y = DISPLAY_HEIGHT / 2;

static constexpr uint32_t TOUCH_I2C_HZ = 400000U;
static constexpr bool TOUCH_USE_IRQ = false;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
static constexpr unsigned long TOUCH_POLL_INTERVAL_MS = 3UL;
#else
static constexpr unsigned long TOUCH_POLL_INTERVAL_MS = 6UL;
#endif
static constexpr uint8_t WAKE_TOUCH_RELEASE_STABLE_POLLS = 3;
static constexpr uint16_t TOUCH_REINIT_FAIL_THRESHOLD = 40;
static constexpr unsigned long TOUCH_REINIT_MIN_INTERVAL_MS = 8000UL;
static constexpr uint8_t TFT_ROTATION = 0;

#if defined(BOARD_ESP32S3_3248S035_N16R8)
static constexpr int TOUCH_SDA = 16;
static constexpr int TOUCH_SCL = 15;
static constexpr int TOUCH_RST = 17;
static constexpr int TOUCH_IRQ = 42;
static constexpr TouchControllerType TOUCH_CONTROLLER = TOUCH_CTRL_GT911;
static constexpr uint8_t TOUCH_ROTATION_OFFSET = 0;
static constexpr const char *DEVICE_MODEL = "ESP32-S3-3248S035 N16R8 Touch Remote";
static constexpr const char *DEVICE_SHORT_NAME = "ESP32-S3-3248S035";
static constexpr const char *AP_SSID = "ESP32-S3-3248S035-FM";
static constexpr const char *MDNS_HOST = "esp32-s3-3248s035";
static constexpr const char *OTA_ASSET_NAME = "esp32-s3-3248s035-n16r8";
static constexpr bool OTA_FIRMWARE_FLASH_SUPPORTED = true;
#elif defined(BOARD_ESP32_3248S035)
static constexpr int TOUCH_SDA = 33;
static constexpr int TOUCH_SCL = 32;
static constexpr int TOUCH_RST = 25;
static constexpr int TOUCH_IRQ = 21;
static constexpr TouchControllerType TOUCH_CONTROLLER = TOUCH_CTRL_GT911;
static constexpr uint8_t TOUCH_ROTATION_OFFSET = 0;
static constexpr const char *DEVICE_MODEL = "ESP32-3248S035 Touch Remote";
static constexpr const char *DEVICE_SHORT_NAME = "ESP32-3248S035";
static constexpr const char *AP_SSID = "ESP32-3248S035-FM";
static constexpr const char *MDNS_HOST = "esp32-3248s035";
static constexpr const char *OTA_ASSET_NAME = "esp32-3248s035";
static constexpr bool OTA_FIRMWARE_FLASH_SUPPORTED = false;
#else
static constexpr int TOUCH_SDA = 33;
static constexpr int TOUCH_SCL = 32;
static constexpr int TOUCH_RST = 25;
static constexpr int TOUCH_IRQ = 21;
static constexpr TouchControllerType TOUCH_CONTROLLER = TOUCH_CTRL_CST820;
static constexpr uint8_t TOUCH_ROTATION_OFFSET = 0;
static constexpr const char *DEVICE_MODEL = "ESP32-2432S024C Touch Remote";
static constexpr const char *DEVICE_SHORT_NAME = "ESP32-2432S024C";
static constexpr const char *AP_SSID = "ESP32-2432S024C-FM";
static constexpr const char *MDNS_HOST = "esp32-2432s024c";
static constexpr const char *OTA_ASSET_NAME = "esp32-2432s024c";
static constexpr bool OTA_FIRMWARE_FLASH_SUPPORTED = false;
#endif

static constexpr uint8_t CST820_ADDR = 0x15;
static constexpr uint8_t GT911_ADDR_PRIMARY = 0x5D;
static constexpr uint8_t GT911_ADDR_SECONDARY = 0x14;
static constexpr uint16_t GT911_REG_PRODUCT_ID = 0x8140;
static constexpr uint16_t GT911_REG_STATUS = 0x814E;
static constexpr uint16_t GT911_REG_POINT1 = 0x814F;

#if defined(BOARD_ESP32S3_3248S035_N16R8)
static constexpr int SD_CS = 38;
static constexpr int SD_MOSI = 1;
static constexpr int SD_SCK = 39;
static constexpr int SD_MISO = 40;
#else
static constexpr int SD_CS = 5;
static constexpr int SD_MOSI = 23;
static constexpr int SD_SCK = 18;
static constexpr int SD_MISO = 19;
#endif
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
#endif
static constexpr uint8_t CHAT_NOTIFY_LEDC_CHANNEL = 7;
static constexpr uint16_t CHAT_NOTIFY_FREQ_PRIMARY = 1760;
static constexpr uint16_t CHAT_NOTIFY_FREQ_SECONDARY = 1320;
static constexpr unsigned long CHAT_NOTIFY_BEEP1_MS = 70UL;
static constexpr unsigned long CHAT_NOTIFY_GAP_MS = 55UL;
static constexpr unsigned long CHAT_NOTIFY_BEEP2_MS = 95UL;
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
static constexpr unsigned long SENSOR_SAMPLE_PERIOD_MS = 2000;
static constexpr unsigned long TOP_INDICATOR_REFRESH_MS = 1500;
static constexpr unsigned long TOP_INDICATOR_WIFI_CONNECT_ANIM_MS = 220;
static constexpr unsigned long WIFI_CONNECT_BARS_ANIM_PERIOD_MS = 240;
static constexpr unsigned long LIGHT_SLEEP_AFTER_IDLE_MS = 20000;
static constexpr bool LIGHT_SLEEP_TIMER_FALLBACK = false;
static constexpr uint64_t LIGHT_SLEEP_TIMER_US = 5000000ULL;
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
static constexpr float BATTERY_CAL_FACTOR_MIN = 0.75f;
static constexpr float BATTERY_CAL_FACTOR_MAX = 1.35f;
static constexpr float BATTERY_CAL_MIN_SPAN_V = 0.55f;
static constexpr float BATTERY_CAL_BLEND_ALPHA = 0.30f;
static constexpr int BATTERY_ADC_SAMPLES = 16;
static constexpr int BATTERY_ADC_SETTLE_READS = 3;
static constexpr unsigned int BATTERY_ADC_SETTLE_US = 250U;
static constexpr uint8_t BATTERY_MEDIAN_WINDOW = 5;
static constexpr float BATTERY_FILTER_ALPHA_RISE = 0.08f;
static constexpr float BATTERY_FILTER_ALPHA_FALL = 0.05f;
static constexpr float BATTERY_FILTER_FAST_ALPHA = 0.30f;
static constexpr float BATTERY_FILTER_FAST_DELTA_V = 0.12f;
static constexpr unsigned long BATTERY_SNAPSHOT_PERIOD_MS = 30000;
static constexpr unsigned long BATTERY_SNAPSHOT_FORCE_MS = 300000;
static constexpr float BATTERY_SNAPSHOT_MIN_DELTA_V = 0.010f;
static constexpr float BATTERY_BOOT_CHARGE_DELTA_V = 0.015f;
static constexpr float BATTERY_BOOT_EMPTY_INFER_MAX_V = 3.55f;
static constexpr unsigned long BATTERY_BOOT_EMPTY_MIN_UPTIME_MS = 120000UL;
static constexpr unsigned long CHARGE_DETECT_INTERVAL_MS = 4000;
static constexpr float CHARGE_RISE_THRESHOLD_V = 0.003f;
static constexpr int8_t CHARGE_SCORE_ON = 1;
static constexpr int8_t CHARGE_SCORE_MAX = 6;
static constexpr unsigned long CHARGE_HOLD_MS = 120000;
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
static constexpr const char *FW_VERSION = "0.2.19";
static constexpr bool VERBOSE_SERIAL_DEBUG = false;
static constexpr unsigned long OTA_CHECK_INTERVAL_MS = 6UL * 60UL * 60UL * 1000UL;
static constexpr unsigned long OTA_INITIAL_CHECK_DELAY_MS = 5000UL;
static constexpr unsigned long OTA_RETRY_DELAY_MS = 10UL * 60UL * 1000UL;
static constexpr size_t OTA_VERSION_TEXT_MAX = 32;
static constexpr size_t OTA_DOWNLOAD_BUF_SIZE = 2048U;
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
    2UL * 60UL * 1000UL,
    5UL * 60UL * 1000UL,
    15UL * 60UL * 1000UL,
    30UL * 60UL * 1000UL,
    0UL
};

static constexpr int HC12_UART_NUM = 1;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
static constexpr int HC12_RX_PIN_DEFAULT = 5;
static constexpr int HC12_TX_PIN_DEFAULT = 4;
static constexpr int HC12_SET_PIN_DEFAULT = 3;
static constexpr int E220_RX_PIN_DEFAULT = 5;
static constexpr int E220_TX_PIN_DEFAULT = 4;
static constexpr int E220_M0_PIN_DEFAULT = 11;
static constexpr int E220_M1_PIN_DEFAULT = 3;
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
static constexpr unsigned long HC12_BAUD = 9600UL;
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
static constexpr size_t HC12_RADIO_MAX_LINE = 1216;
static constexpr int MAX_HC12_DISCOVERED = 8;
static constexpr unsigned long HC12_DISCOVERY_INTERVAL_MS = 8000UL;
static constexpr unsigned long HC12_DISCOVERY_STALE_MS = 45000UL;
static constexpr uint8_t UI_DEFERRED_SCREENSHOT_PENDING = 0x01;
static constexpr uint8_t UI_DEFERRED_SCREENSHOT_BUSY = 0x02;
static constexpr uint8_t UI_DEFERRED_HC12_SETTLE_PENDING = 0x04;
static constexpr uint8_t UI_DEFERRED_HC12_TARGET_ASSERTED = 0x08;

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

TFT_eSPI tft;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
SPIClass sdSpi(FSPI);
#else
SPIClass sdSpi(HSPI);
#endif
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
#if defined(BOARD_ESP32S3_3248S035_N16R8)
static void chatMessageVibrationStop(bool clearQueue);
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
bool captureScreenToJpeg(String &savedPathOut, String &errorOut);
String mqttDefaultButtonName(int idx);
String mqttButtonPayloadForIndex(int idx);
String mediaBuildListLabel(const String &name, bool isDir, size_t sizeBytes);
void displaySetAwake(bool awake);
static void wakeDisplayForIncomingNotification();
void displayBacklightInit();
void displayBacklightSet(uint8_t level);
void displayBacklightFadeIn(uint16_t durationMs = 220);
uint8_t displayBacklightLevelFromPercent(uint8_t percent);
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
static void otaSetStatus(const String &text);
static void otaClearPopupVersion();
static void otaCheckTask(void *param);
static void otaUpdateTask(void *param);
void lvglOpenOtaScreenEvent(lv_event_t *e);
void lvglOpenHc12ScreenEvent(lv_event_t *e);
void lvglOpenHc12TerminalEvent(lv_event_t *e);
void lvglOpenHc12InfoEvent(lv_event_t *e);
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
void mqttPublishButtonAction(int index);
void mqttService();
bool mqttConnectNow();
void mqttPublishDiscovery();
void mqttTrimBufferForIdle();
bool mqttPublishChatMessage(const String &text);
bool mqttPublishChatMessageWithId(const String &peerKey, const String &text, const String &messageId);
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
void lvglSoundPopupBackdropEvent(lv_event_t *e);
void lvglSoundPopupVolumeEvent(lv_event_t *e);
void lvglSoundPopupVibrationEvent(lv_event_t *e);
void lvglSoundPopupDisableVibrationEvent(lv_event_t *e);
void lvglApplyAirplaneButtonStyle();
void lvglApplyApModeButtonStyle();
void lvglApplyChatDiscoveryButtonStyle();
void lvglApplyWifiWebServerButtonStyle();
static void lvglAttachMenuButtonImage(lv_obj_t *btn, const lv_img_dsc_t *imgSrc, lv_coord_t xOffset, lv_coord_t labelShiftX);
static void lvglSetButtonImageZoom(lv_obj_t *btn, uint16_t zoom, lv_coord_t xOffset = 8, lv_coord_t labelShiftX = 10);
static void lvglRefreshPrimaryMenuButtonIcons();
static void lvglRegisterStyledButton(lv_obj_t *btn, lv_color_t baseColor, bool compact);
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
void lvglSetConfigKeyboardVisible(bool visible);
void lvglShowChatAirplanePrompt();
void cpuLoadService(uint32_t loopStartUs);
void rgbService();
void loadP2pConfig();
void saveP2pConfig();
void p2pEnsureUdp();
void p2pService();
bool p2pSendChatMessage(const String &text);
bool p2pSendChatMessageWithId(const String &peerKey, const String &text, const String &messageId);
bool p2pSendMessageDelete(const String &peerKey, const String &messageId);
bool p2pSendConversationDelete();
bool p2pSendChatAck(const String &peerKey, const String &messageId);
static bool hc12BroadcastDiscoveryFrame(const char *kind);
bool hc12SendChatMessageWithId(const String &peerKey, const String &text, const String &messageId);
bool hc12SendMessageDelete(const String &peerKey, const String &messageId);
bool hc12SendConversationDelete(const String &peerKey);
bool hc12SendChatAck(const String &peerKey, const String &messageId);
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
static constexpr uint16_t LVGL_BUF_LINES_MIN = 64;
static constexpr uint16_t LVGL_BUF_LINES_MAX = 120;
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
static constexpr int16_t SWIPE_BACK_MIN_DX = 52;
static constexpr int16_t SWIPE_BACK_MAX_DY = 30;
static constexpr int16_t SWIPE_LOCK_MIN_DX = 18;
static constexpr int16_t SWIPE_CANCEL_VERTICAL_DY = 18;
static constexpr uint16_t SWIPE_BACK_SNAP_MIN_MS = 36;
static constexpr uint8_t SWIPE_BACK_COMPLETE_PERCENT = 30;
static constexpr unsigned long SWIPE_BACK_MAX_MS = 550;
static constexpr int16_t DOUBLE_TAP_MAX_MOVE = 12;
static constexpr int16_t DOUBLE_TAP_MAX_GAP = 350;
static constexpr unsigned long DOUBLE_TAP_MAX_TAP_MS = 240;
static constexpr unsigned long CLICK_SUPPRESS_AFTER_GESTURE_MS = 180UL;
static constexpr unsigned long UI_WARMUP_INTERVAL_MS = 220UL;
static constexpr unsigned long UI_WARMUP_IDLE_AFTER_INPUT_MS = 1800UL;
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
static lv_obj_t *lvglScrStyle = nullptr;
static lv_obj_t *lvglScrBatteryTrain = nullptr;
static lv_obj_t *lvglScrLanguage = nullptr;
static lv_obj_t *lvglScrOta = nullptr;
static lv_obj_t *lvglScrScreensaver = nullptr;
static lv_obj_t *lvglScrHc12 = nullptr;
static lv_obj_t *lvglScrHc12Terminal = nullptr;
static lv_obj_t *lvglScrHc12Info = nullptr;
static lv_obj_t *lvglScrMqttCfg = nullptr;
static lv_obj_t *lvglScrMqttCtrl = nullptr;
static lv_obj_t *lvglScrSnake = nullptr;
static lv_obj_t *lvglScrTetris = nullptr;
static lv_obj_t *lvglScrCheckers = nullptr;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
static lv_obj_t *lvglScrSnake3d = nullptr;
#endif
static lv_obj_t *lvglStatusLabel = nullptr;
static lv_obj_t *lvglInfoList = nullptr;
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
static lv_obj_t *lvglChatDiscoveryBtn = nullptr;
static lv_obj_t *lvglChatMenuBtn = nullptr;
static lv_obj_t *lvglChatMenuBackdrop = nullptr;
static lv_obj_t *lvglChatMenuPanel = nullptr;
static lv_obj_t *lvglChatPeerList = nullptr;
static lv_obj_t *lvglChatPeerIdentityLabel = nullptr;
static lv_obj_t *lvglChatPeerScanBtn = nullptr;
static lv_obj_t *lvglAirplaneBtn = nullptr;
static lv_obj_t *lvglAirplaneBtnLabel = nullptr;
static lv_obj_t *lvglApModeBtn = nullptr;
static lv_obj_t *lvglApModeBtnLabel = nullptr;
static lv_obj_t *lvglHomePowerBtn = nullptr;
static lv_obj_t *lvglHomeChatBtn = nullptr;
static lv_obj_t *lvglHomeMediaBtn = nullptr;
static lv_obj_t *lvglHomeInfoBtn = nullptr;
static lv_obj_t *lvglHomeGamesBtn = nullptr;
static lv_obj_t *lvglHomeConfigBtn = nullptr;
static lv_obj_t *lvglConfigWifiBtn = nullptr;
static lv_obj_t *lvglConfigHc12Btn = nullptr;
static lv_obj_t *lvglConfigStyleBtn = nullptr;
static lv_obj_t *lvglConfigBatteryBtn = nullptr;
static lv_obj_t *lvglConfigMqttBtn = nullptr;
static lv_obj_t *lvglConfigMqttControlsBtn = nullptr;
static lv_obj_t *lvglConfigScreenshotBtn = nullptr;
static lv_obj_t *lvglConfigLanguageBtn = nullptr;
static lv_obj_t *lvglConfigOtaBtn = nullptr;
static lv_obj_t *lvglConfigWrap = nullptr;
static lv_obj_t *lvglStyleScreensaverSw = nullptr;
static lv_obj_t *lvglStyleMenuIconsSw = nullptr;
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
static bool lvglStyleUiSyncing = false;
static lv_obj_t *lvglOtaCurrentLabel = nullptr;
static lv_obj_t *lvglOtaLatestLabel = nullptr;
static lv_obj_t *lvglOtaStatusLabel = nullptr;
static lv_obj_t *lvglOtaUpdateBtn = nullptr;
static lv_obj_t *lvglOtaUpdateBtnLabel = nullptr;
static lv_obj_t *lvglOtaProgressBar = nullptr;
static lv_obj_t *lvglOtaProgressLabel = nullptr;
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
static lv_obj_t *lvglConfigDeviceNameSaveBtnLabel = nullptr;
static lv_obj_t *lvglBrightnessSlider = nullptr;
static lv_obj_t *lvglBrightnessValueLabel = nullptr;
static lv_obj_t *lvglVolumeSlider = nullptr;
static lv_obj_t *lvglVolumeValueLabel = nullptr;
static lv_obj_t *lvglRgbLedSlider = nullptr;
static lv_obj_t *lvglVibrationDropdown = nullptr;
static lv_obj_t *lvglMessageToneDropdown = nullptr;
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
static unsigned long lvglLastTickMs = 0;
static unsigned long lvglLastInfoRefreshMs = 0;
static unsigned long lvglLastStatusRefreshMs = 0;
static unsigned long lvglLastMediaPlayerRefreshMs = 0;
static bool lvglMediaPlayerVisible = false;
static bool p2pDiscoveryEnabled = true;
static bool p2pPairPromptVisible = false;
static bool p2pPairRequestPending = false;
static int p2pPairRequestDiscoveredIdx = -1;
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
bool wifiRuntimeManaged = true;
bool wifiScanInProgress = false;
unsigned long wifiScanAnimLastMs = 0;
uint8_t wifiScanAnimPhase = 0;
unsigned long bootDeferredStartMs = 0;
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
unsigned long displayIdleTimeoutMs = LCD_IDLE_TIMEOUT_MS_DEFAULT;
unsigned long powerOffIdleTimeoutMs = 0;
bool screensaverEnabled = false;
bool screensaverActive = false;
uint8_t rgbLedPercent = 100;
String deviceShortName = DEVICE_SHORT_NAME;
uint8_t cpuLoadPercent = 0;
bool batteryFilterInitialized = false;
bool lightFilterInitialized = false;
float lightPercentFiltered = 0.0f;
float chargePrevVoltage = 0.0f;
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
    UI_CHAT,
    UI_CHAT_PEERS,
    UI_WIFI_LIST,
    UI_MEDIA,
    UI_INFO,
    UI_GAMES,
    UI_CONFIG,
    UI_CONFIG_BATTERY,
    UI_CONFIG_STYLE,
    UI_CONFIG_LANGUAGE,
    UI_CONFIG_OTA,
    UI_SCREENSAVER,
    UI_CONFIG_HC12,
    UI_CONFIG_HC12_TERMINAL,
    UI_CONFIG_HC12_INFO,
    UI_CONFIG_MQTT_CONFIG,
    UI_CONFIG_MQTT_CONTROLS,
    UI_GAME_SNAKE,
    UI_GAME_TETRIS,
    UI_GAME_CHECKERS,
    UI_GAME_SNAKE3D
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
    TXT_CONFIG,
    TXT_AIRPLANE_ON,
    TXT_AIRPLANE_OFF,
    TXT_AP_MODE_ON,
    TXT_AP_MODE_OFF,
    TXT_WIFI_CONFIG,
    TXT_HC12_CONFIG,
    TXT_STYLE,
    TXT_MQTT_CONFIG,
    TXT_MQTT_CONTROLS,
    TXT_SCREENSHOT,
    TXT_LANGUAGE,
    TXT_OTA_UPDATES,
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
    TXT_SCREEN
};

UiScreen uiScreen = UI_HOME;
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

VibrationIntensity vibrationIntensity = VIBRATION_INTENSITY_MEDIUM;
RadioModuleType radioModuleType = RADIO_MODULE_HC12;
MessageBeepTone messageBeepTone = MESSAGE_BEEP_DOUBLE_SHORT;
bool vibrationEnabled = true;

static const char *tr(UiTextId id);
static String buildLanguageDropdownOptions();
static String buildVibrationIntensityDropdownOptions();
static String buildRadioModuleDropdownOptions();
static String buildMessageBeepDropdownOptions();
static const char *vibrationIntensityLabel(VibrationIntensity intensity);
static const char *radioModuleLabel(RadioModuleType module);
static const char *messageBeepToneLabel(MessageBeepTone tone);
static void saveSoundPrefs();
static uint8_t uiSoundMode();
static const lv_img_dsc_t *uiSoundModeIcon();
static const lv_img_dsc_t *uiVolumeIcon();
static void lvglRefreshSoundPopupUi();
static void lvglShowSoundPopup();
static void lvglHideSoundPopup();
static void lvglApplyMsgboxModalStyle(lv_obj_t *msgbox);

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
    uint8_t attempts;
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
    return String("2 min\n5 min\n15 min\n30 min\nNever");
}

static int powerOffTimeoutOptionIndex(unsigned long ms)
{
    for (int i = 0; i < static_cast<int>(sizeof(POWER_OFF_IDLE_TIMEOUT_OPTIONS_MS) / sizeof(POWER_OFF_IDLE_TIMEOUT_OPTIONS_MS[0])); ++i) {
        if (POWER_OFF_IDLE_TIMEOUT_OPTIONS_MS[i] == ms) return i;
    }
    return static_cast<int>(sizeof(POWER_OFF_IDLE_TIMEOUT_OPTIONS_MS) / sizeof(POWER_OFF_IDLE_TIMEOUT_OPTIONS_MS[0])) - 1;
}

static bool applyDisplayIdleTimeoutPowerOffCap(bool persist)
{
    unsigned long cappedTimeoutMs = displayIdleTimeoutMs;
    if (powerOffIdleTimeoutMs >= 120000UL) {
        const unsigned long minimumTimeoutMs = clampIdleTimeoutMs(powerOffIdleTimeoutMs - 60000UL);
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

static int hc12FindDiscoveredByPubKeyHex(const String &pubKeyHex)
{
    if (!hc12DiscoveredPeers) return -1;
    for (int i = 0; i < hc12DiscoveredCount; ++i) {
        if (hc12DiscoveredPeers[i].pubKeyHex.equalsIgnoreCase(pubKeyHex)) return i;
    }
    return -1;
}

static bool hc12PeerRecentlySeen(const String &peerKey, unsigned long staleMs = HC12_DISCOVERY_STALE_MS)
{
    const int idx = hc12FindDiscoveredByPubKeyHex(peerKey);
    if (idx < 0) return false;
    return static_cast<unsigned long>(millis() - hc12DiscoveredPeers[idx].lastSeenMs) <= staleMs;
}

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
    const int idx = p2pFindPeerByPubKeyHex(peerKey);
    if (idx >= 0 && p2pPeers[idx].enabled) return true;
    return hc12FindDiscoveredByPubKeyHex(peerKey) >= 0 || chatPeerHasHistory(peerKey);
}

static bool chatHasUnreadMessages()
{
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

static constexpr int SNAKE_COLS = 12;
static constexpr int SNAKE_ROWS = 14;
static constexpr int SNAKE_CELL = 14;
static constexpr int SNAKE_MAX_CELLS = SNAKE_COLS * SNAKE_ROWS;
static constexpr int SNAKE_BOARD_X = 36;
static constexpr int SNAKE_BOARD_Y = 50;
static constexpr unsigned long SNAKE_STEP_MS_START = 840;
static constexpr unsigned long SNAKE_STEP_MS_MIN = 440;
static constexpr unsigned long SNAKE_STEP_MS_DELTA_PER_FOOD = 10;
int8_t snakeDir = 3; // 0 up,1 right,2 down,3 left
int8_t snakeNextDir = 3;
int snakeLen = 0;
int8_t snakeX[SNAKE_MAX_CELLS];
int8_t snakeY[SNAKE_MAX_CELLS];
int8_t snakeFoodX = 0;
int8_t snakeFoodY = 0;
bool snakeStarted = false;
bool snakePaused = false;
bool snakeGameOver = false;
uint16_t snakeScore = 0;
uint16_t snakeHighScore = 0;
uint16_t snakeLastScore = 0;
unsigned long snakeLastStepMs = 0;

#if defined(BOARD_ESP32S3_3248S035_N16R8)
static constexpr int SNAKE3D_COLS = 5;
static constexpr int SNAKE3D_ROWS = 5;
static constexpr int SNAKE3D_LAYERS = 6;
static constexpr int SNAKE3D_GROUND_SCALE = 4;
static constexpr int SNAKE3D_GROUND_TILE_SPAN = 2;
static constexpr int SNAKE3D_GROUND_COLS = (SNAKE3D_COLS * SNAKE3D_GROUND_SCALE) / SNAKE3D_GROUND_TILE_SPAN;
static constexpr int SNAKE3D_GROUND_ROWS = (SNAKE3D_ROWS * SNAKE3D_GROUND_SCALE) / SNAKE3D_GROUND_TILE_SPAN;
static constexpr int SNAKE3D_MAX_CELLS = SNAKE3D_COLS * SNAKE3D_ROWS * SNAKE3D_LAYERS;
static constexpr unsigned long SNAKE3D_STEP_MS_START = 720;
static constexpr unsigned long SNAKE3D_STEP_MS_MIN = 280;
static constexpr unsigned long SNAKE3D_STEP_MS_DELTA_PER_FOOD = 12;
int8_t snake3dDir = 1; // 0 up,1 right,2 down,3 left,4 climb,5 dive
int8_t snake3dNextDir = 1;
int8_t snake3dViewDir = 1;
int snake3dLen = 0;
int8_t snake3dX[SNAKE3D_MAX_CELLS];
int8_t snake3dY[SNAKE3D_MAX_CELLS];
int8_t snake3dZ[SNAKE3D_MAX_CELLS];
int8_t snake3dFoodX = 0;
int8_t snake3dFoodY = 0;
int8_t snake3dFoodZ = 0;
bool snake3dStarted = false;
bool snake3dPaused = false;
bool snake3dGameOver = false;
uint16_t snake3dScore = 0;
uint16_t snake3dHighScore = 0;
uint16_t snake3dLastScore = 0;
unsigned long snake3dLastStepMs = 0;
#endif

static constexpr int TETRIS_COLS = 10;
static constexpr int TETRIS_ROWS = 16;
static constexpr int TETRIS_CELL = 12;
static constexpr int TETRIS_BOARD_X = 60;
static constexpr int TETRIS_BOARD_Y = 50;
static constexpr unsigned long TETRIS_STEP_MS_START = 900;
static constexpr unsigned long TETRIS_STEP_MS_MIN = 420;
static constexpr unsigned long TETRIS_STEP_MS_DELTA_PER_100_SCORE = 10;
static constexpr unsigned long TETRIS_DROP_ANIM_MS_MIN = 70UL;
static constexpr unsigned long TETRIS_DROP_ANIM_MS_PER_ROW = 24UL;
static constexpr unsigned long TETRIS_DROP_ANIM_MS_MAX = 260UL;
static constexpr uint8_t TETRIS_DROP_ANIM_ACTIVE = 0x01;
static constexpr uint8_t TETRIS_DROP_LOCK_PENDING = 0x02;
uint8_t tetrisGrid[TETRIS_ROWS][TETRIS_COLS];
int8_t tetrisType = 0;
int8_t tetrisRot = 0;
int8_t tetrisX = 3;
int8_t tetrisY = 0;
bool tetrisStarted = false;
bool tetrisPaused = false;
bool tetrisGameOver = false;
uint16_t tetrisScore = 0;
uint16_t tetrisHighScore = 0;
uint16_t tetrisLastScore = 0;
unsigned long tetrisLastStepMs = 0;
uint8_t tetrisDropAnimFlags = 0;
int8_t tetrisAnimFromY = 0;
uint16_t tetrisAnimStartTick = 0;
uint16_t tetrisAnimDurationMs = 0;

static constexpr int CHECKERS_MAX_BOARD_SIZE = 12;
static constexpr int CHECKERS_MAX_HINT_MOVES = 32;
static constexpr int CHECKERS_ACTION_BAR_H = 46;
static constexpr int CHECKERS_CONTROL_MSG_MARKERS = 4;
static const char *const CHECKERS_CONTROL_PREFIXES[CHECKERS_CONTROL_MSG_MARKERS] = {
    "@CHK_INVITE|",
    "@CHK_ACCEPT|",
    "@CHK_MOVE|",
    "@CHK_DECLINE|"
};

enum CheckersMode : uint8_t {
    CHECKERS_MODE_IDLE = 0,
    CHECKERS_MODE_ESP32,
    CHECKERS_MODE_TAG
};

enum CheckersVariant : uint8_t {
    CHECKERS_VARIANT_AMERICAN = 0,
    CHECKERS_VARIANT_INTERNATIONAL,
    CHECKERS_VARIANT_RUSSIAN,
    CHECKERS_VARIANT_POOL,
    CHECKERS_VARIANT_CANADIAN
};

static const char *checkersVariantName(CheckersVariant variant);

struct CheckersMove {
    int8_t fromX;
    int8_t fromY;
    int8_t toX;
    int8_t toY;
    int8_t captureX;
    int8_t captureY;
    bool capture;
};

static constexpr int CHECKERS_MAX_MOVES = 128;
int8_t checkersBoard[CHECKERS_MAX_BOARD_SIZE][CHECKERS_MAX_BOARD_SIZE] = {};
CheckersMode checkersMode = CHECKERS_MODE_IDLE;
CheckersVariant checkersVariant = CHECKERS_VARIANT_AMERICAN;
String checkersSessionId;
String checkersPeerKey;
int8_t checkersLocalSide = 0;
int8_t checkersTurn = 1;
int8_t checkersSelectedX = -1;
int8_t checkersSelectedY = -1;
int8_t checkersForcedX = -1;
int8_t checkersForcedY = -1;
bool checkersStarted = false;
bool checkersWaitingForRemote = false;
bool checkersPeerPopupOpen = false;
bool checkersVariantPopupOpen = true;
bool checkersGameOver = false;
int8_t checkersWinnerSide = 0;
uint16_t checkersLocalWins = 0;
uint16_t checkersRemoteWins = 0;
unsigned long checkersAiDueMs = 0;

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
#if defined(BOARD_ESP32S3_3248S035_N16R8)
bool chatMessageVibrationPinAttached = false;
bool chatMessageVibrationPinOutput = false;
uint8_t chatMessageVibrationPhase = 0;
uint8_t chatMessageVibrationQueue = 0;
unsigned long chatMessageVibrationDeadlineMs = 0;
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
bool recordTelemetryEnabled = false;
bool systemSoundsEnabled = true;
bool wsRebootOnDisconnectEnabled = false;
bool airplaneModeEnabled = false;
bool menuCustomIconsEnabled = false;
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
String otaStatusText;
String otaLatestBinUrl;
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
static constexpr unsigned long DNS_SERVICE_INTERVAL_MS = 8UL;
static constexpr unsigned long WIFI_SCAN_SERVICE_INTERVAL_MS = 25UL;
static constexpr unsigned long WIFI_CONN_SERVICE_INTERVAL_MS = 20UL;
static constexpr unsigned long SD_STATS_SERVICE_INTERVAL_MS = 100UL;
static constexpr unsigned long MDNS_SERVICE_INTERVAL_MS = 250UL;
static constexpr unsigned long P2P_SERVICE_INTERVAL_MS = 10UL;
static constexpr unsigned long MQTT_SERVICE_INTERVAL_MS = 15UL;
static constexpr unsigned long CHAT_PENDING_SERVICE_INTERVAL_MS = 80UL;
static constexpr unsigned long RGB_SERVICE_INTERVAL_MS = 16UL;
static constexpr unsigned long CAR_TELEMETRY_SERVICE_INTERVAL_MS = 50UL;
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

static const char AP_WIFI_LANDING_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>ESP32 AP Setup</title>
  <style>
    :root{
      --bg:#050b14;
      --panel:#1d1d1f;
      --panel-2:#20242b;
      --line:#5b6573;
      --text:#f4f7fb;
      --muted:#9db0c5;
      --accent:#5ea2ff;
      --accent-2:#7cb342;
      --danger:#ff8a8a;
      --tab:#efefef;
      --tab-text:#101317;
    }
    *{box-sizing:border-box}
    body{
      margin:0;
      min-height:100vh;
      font-family:Segoe UI,Arial,sans-serif;
      color:var(--text);
      background:
        radial-gradient(circle at top left, rgba(94,162,255,.18), transparent 28%),
        radial-gradient(circle at bottom right, rgba(255,177,66,.12), transparent 24%),
        linear-gradient(180deg, #02060d 0%, #061120 100%);
      display:flex;
      align-items:flex-start;
      justify-content:center;
      padding:18px 12px;
    }
    .shell{
      width:min(980px,100%);
      background:linear-gradient(180deg, rgba(40,44,50,.96), rgba(24,26,29,.98));
      border:4px solid rgba(255,255,255,.92);
      border-radius:22px;
      box-shadow:0 22px 80px rgba(0,0,0,.48);
      overflow:hidden;
    }
    .frame{
      margin:14px;
      padding:18px 18px 24px;
      border-radius:18px;
      background:linear-gradient(180deg, rgba(30,31,34,.98), rgba(27,28,31,.95));
      min-height:calc(100vh - 72px);
      position:relative;
    }
    .topbar{
      display:flex;
      align-items:center;
      justify-content:space-between;
      gap:12px;
      margin-bottom:16px;
    }
    .title{
      font-size:30px;
      font-weight:800;
      color:#89bcff;
      display:flex;
      align-items:center;
      gap:10px;
    }
    .title small{
      display:block;
      font-size:13px;
      font-weight:600;
      color:var(--muted);
      margin-top:4px;
    }
    .close{
      width:42px;
      height:42px;
      border:none;
      border-radius:999px;
      background:#18212d;
      color:#fff;
      font-size:30px;
      cursor:pointer;
      display:flex;
      align-items:center;
      justify-content:center;
    }
    .close:hover{background:#223042}
    h2{
      font-size:22px;
      margin:4px 0 8px;
    }
    .divider{
      height:1px;
      background:rgba(255,255,255,.72);
      margin:18px 0 28px;
    }
    .lead{
      color:var(--muted);
      margin-bottom:16px;
      line-height:1.45;
    }
    .panel-grid{
      display:grid;
      grid-template-columns:repeat(2,minmax(0,1fr));
      gap:18px;
      align-items:start;
    }
    .card{
      background:rgba(27,30,35,.84);
      border:1px solid rgba(140,154,173,.25);
      border-radius:14px;
      padding:18px;
    }
    .card h3{
      margin:0 0 16px;
      font-size:16px;
    }
    .meta{
      display:grid;
      gap:10px;
      color:var(--muted);
      margin-bottom:14px;
    }
    .meta strong{
      color:#fff;
      margin-right:8px;
    }
    .field{
      display:grid;
      gap:8px;
      margin-bottom:12px;
    }
    label{
      font-weight:700;
    }
    select,input,button{
      font:inherit;
    }
    select,input{
      width:100%;
      border-radius:8px;
      border:1px solid #aeb6c1;
      padding:10px 12px;
      background:#f3f5f7;
      color:#111;
    }
    .row{
      display:flex;
      gap:10px;
      align-items:center;
      flex-wrap:wrap;
    }
    .row > *{
      flex:1 1 auto;
    }
    .btn{
      border:none;
      border-radius:8px;
      padding:12px 14px;
      background:#d9dde2;
      color:#111;
      cursor:pointer;
      font-weight:700;
    }
    .btn.primary{background:var(--accent);color:#fff}
    .btn.secondary{background:#d7d7d7}
    .btn.success{background:var(--accent-2);color:#fff}
    .btn:disabled{opacity:.6;cursor:wait}
    .hint{
      color:#7fb0ff;
      font-size:13px;
      margin-top:6px;
      line-height:1.45;
    }
    .status{
      margin-top:12px;
      min-height:22px;
      color:var(--muted);
      font-weight:600;
    }
    .status.error{color:var(--danger)}
    .saved{
      min-height:132px;
      border-left:1px solid rgba(255,255,255,.26);
      padding-left:18px;
    }
    .saved .item{
      padding:8px 0;
      border-bottom:1px solid rgba(255,255,255,.08);
      color:#dfe7ef;
    }
    .footer{
      display:flex;
      justify-content:space-between;
      gap:12px;
      align-items:center;
      margin-top:22px;
      flex-wrap:wrap;
    }
    .telemetry{
      color:var(--muted);
      font-size:14px;
    }
    .actions{
      display:flex;
      gap:10px;
      flex-wrap:wrap;
    }
    @media (max-width: 820px){
      body{padding:10px}
      .frame{min-height:auto;padding:16px}
      .title{font-size:24px}
      .panel-grid{grid-template-columns:1fr}
      .saved{border-left:0;padding-left:0}
      .close{width:38px;height:38px;font-size:26px}
      .footer{align-items:stretch}
      .actions{width:100%}
      .actions .btn{flex:1 1 auto}
    }
  </style>
</head>
<body>
  <div class="shell">
    <div class="frame">
      <div class="topbar">
        <div>
          <div class="title">Settings</div>
          <small style="color:#a8b7ca">Firmware %FIRMWARE_VERSION%</small>
        </div>
        <button id="closeBtn" class="close" title="Close">&times;</button>
      </div>

      <h2>Wi-Fi Setup</h2>
      <div class="lead">AP mode is active. Configure infrastructure Wi-Fi first, then continue to the main web UI or the embedded recovery browser.</div>
      <div class="divider"></div>

      <div class="panel-grid">
        <div class="card">
          <h3>Access Point (AP) Details</h3>
          <div class="meta">
            <div><strong>AP Name:</strong> %AP_SSID%</div>
            <div><strong>AP IP:</strong> %AP_IP%</div>
            <div><strong>Portal:</strong> http://%AP_IP%/</div>
          </div>
          <div class="hint">This page is always shown first while the device is in AP mode. Closing it will continue to %CONTINUE_LABEL%.</div>
        </div>

        <div class="card saved">
          <h3>Saved Network</h3>
          <div id="savedNetwork" class="item">Loading...</div>
          <button id="continueBtn" class="btn secondary" style="margin-top:14px;">Continue</button>
        </div>
      </div>

      <div class="panel-grid" style="margin-top:18px;">
        <div class="card">
          <h3>Select Wi-Fi</h3>
          <div class="field">
            <label for="ssid">Network</label>
            <select id="ssid"></select>
          </div>
          <div class="field">
            <label for="pass">Password</label>
            <div class="row">
              <input id="pass" type="password" placeholder="Enter network password">
              <button id="showBtn" class="btn" style="flex:0 0 auto;">Show</button>
            </div>
          </div>
          <div class="actions">
            <button id="scanBtn" class="btn secondary">Scan Wi-Fi Networks</button>
            <button id="connBtn" class="btn primary">Connect</button>
          </div>
          <div id="status" class="status">Ready</div>
        </div>

        <div class="card">
          <h3>Device Telemetry</h3>
          <div id="telemetry" class="telemetry">Loading telemetry...</div>
          <div class="hint" style="margin-top:18px;">If no SD frontend is present, Continue opens the embedded recovery file browser so you can upload the missing web files.</div>
        </div>
      </div>

      <div class="footer">
        <div id="continueHint" class="telemetry">%CONTINUE_HINT%</div>
        <div class="actions">
          <button id="footerContinueBtn" class="btn success">Close and Continue</button>
        </div>
      </div>
    </div>
  </div>
<script>
const continueUrl = "%CONTINUE_URL%";
const teleEl = document.getElementById('telemetry');
const statusEl = document.getElementById('status');
const ssidEl = document.getElementById('ssid');
const passEl = document.getElementById('pass');
const showBtn = document.getElementById('showBtn');
const savedEl = document.getElementById('savedNetwork');
const actionButtons = [document.getElementById('scanBtn'), document.getElementById('connBtn')];

function showStatus(message, isError){
  statusEl.textContent = message;
  statusEl.classList.toggle('error', !!isError);
}

function continueToUi(){
  if (!continueUrl) return;
  window.location.assign(continueUrl);
}

async function refreshTelemetry(){
  try{
    const r = await fetch('/api/telemetry', {cache:'no-store'});
    const j = await r.json();
    const wifiLine = j.wifi_connected ? ('Connected to ' + j.wifi_ssid + ' (' + j.wifi_rssi + ' dBm)') : 'Not connected to infrastructure Wi-Fi';
    teleEl.textContent =
      'Battery: ' + j.battery_percent + '% (' + j.battery_voltage + 'V) | ' +
      'Light: ' + j.light_percent + '% | ' +
      'Wi-Fi: ' + wifiLine;
    savedEl.textContent = j.wifi_connected ? ('Connected: ' + j.wifi_ssid) : 'Saved network will appear after successful STA connection.';
  }catch(e){
    teleEl.textContent = 'Telemetry unavailable';
    savedEl.textContent = 'Unable to query current network state.';
  }
}

async function scanWifi(){
  try{
    showStatus('Scanning...', false);
    actionButtons.forEach(btn => btn.disabled = true);
    const r = await fetch('/api/wifi/scan', {cache:'no-store'});
    const j = await r.json();
    ssidEl.innerHTML = '';
    const list = Array.isArray(j.networks) ? j.networks : [];
    list.forEach(n => {
      const opt = document.createElement('option');
      opt.value = n.ssid || '';
      opt.textContent = (n.ssid || '<hidden>') + ' (' + (n.rssi ?? '?') + ' dBm, ' + (n.auth || 'unknown') + ')';
      ssidEl.appendChild(opt);
    });
    if (!list.length) {
      const opt = document.createElement('option');
      opt.value = '';
      opt.textContent = 'No networks found';
      ssidEl.appendChild(opt);
    }
    showStatus('Found ' + list.length + ' network(s)', false);
  }catch(e){
    showStatus('Scan failed', true);
  }finally{
    actionButtons.forEach(btn => btn.disabled = false);
  }
}

async function connectWifi(){
  const ssid = ssidEl.value || '';
  const pass = passEl.value || '';
  if(!ssid){
    showStatus('Select SSID first', true);
    return;
  }
  try{
    actionButtons.forEach(btn => btn.disabled = true);
    showStatus('Connecting...', false);
    const body = new URLSearchParams({ssid, pass});
    const r = await fetch('/api/wifi/connect', {method:'POST', body});
    if(!r.ok) throw new Error(await r.text());
    showStatus('Connect request sent. The device will try to join ' + ssid + '.', false);
    setTimeout(refreshTelemetry, 1500);
  }catch(e){
    showStatus('Connect failed: ' + e.message, true);
  }finally{
    actionButtons.forEach(btn => btn.disabled = false);
  }
}

document.getElementById('scanBtn').onclick = scanWifi;
document.getElementById('connBtn').onclick = connectWifi;
document.getElementById('closeBtn').onclick = continueToUi;
document.getElementById('continueBtn').onclick = continueToUi;
document.getElementById('footerContinueBtn').onclick = continueToUi;
showBtn.onclick = () => {
  passEl.type = passEl.type === 'password' ? 'text' : 'password';
  showBtn.textContent = passEl.type === 'password' ? 'Show' : 'Hide';
};

scanWifi();
refreshTelemetry();
setInterval(refreshTelemetry, 2500);
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
        case UI_CONFIG:
            target = UI_HOME;
            return true;
        case UI_CHAT_PEERS:
            target = UI_CHAT;
            return true;
        case UI_WIFI_LIST:
        case UI_CONFIG_BATTERY:
        case UI_CONFIG_STYLE:
        case UI_CONFIG_LANGUAGE:
        case UI_CONFIG_OTA:
        case UI_CONFIG_HC12:
        case UI_CONFIG_MQTT_CONFIG:
        case UI_CONFIG_MQTT_CONTROLS:
            target = UI_CONFIG;
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
    if (!obj) return;
    lv_obj_set_x(static_cast<lv_obj_t *>(obj), static_cast<lv_coord_t>(v));
}

static uint16_t lvglSwipeBackAnimDuration(lv_coord_t fromX, lv_coord_t toX)
{
    const uint32_t distance = static_cast<uint32_t>(abs(static_cast<int32_t>(toX) - static_cast<int32_t>(fromX)));
    if (distance == 0U) return 0;
    const uint32_t scaled = (distance * UI_ANIM_MS) / max<int16_t>(DISPLAY_WIDTH, 1);
    return static_cast<uint16_t>(constrain(scaled, static_cast<uint32_t>(SWIPE_BACK_SNAP_MIN_MS), static_cast<uint32_t>(UI_ANIM_MS)));
}

static void lvglReleaseSwipeBackPreview()
{
    if (lvglSwipePreviewImg && lv_obj_is_valid(lvglSwipePreviewImg)) lv_obj_del(lvglSwipePreviewImg);
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
    UiScreen targetUi = UI_HOME;
    if (!lvglGetSwipeBackPreviewTarget(uiScreen, targetUi)) return false;

    lvglReleaseSwipeBackPreview();
    if (boardHasUsablePsram()) {
        lv_obj_t *targetScreen = nullptr;
        if (uiScreen == UI_CHAT && !currentChatPeerKey.isEmpty() && targetUi == UI_CHAT) {
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
        lv_anim_del(lvglSwipeVisualScreen, lvglSetObjXAnim);
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
    if (screen && lv_obj_is_valid(screen)) lv_obj_set_x(screen, 0);
    lvglSwipeVisualScreen = nullptr;
    lvglSwipeVisualOffsetX = 0;
    lvglSwipeVisualActive = false;
    lvglSwipeVisualAnimating = false;
    if (navigateBack) lvglNavigateBackBySwipe(LV_SCR_LOAD_ANIM_NONE);
}

static void lvglAnimateSwipeBackVisual(lv_coord_t targetX, bool navigateBack)
{
    lv_obj_t *screen = lvglSwipeVisualScreen;
    if (!screen || !lv_obj_is_valid(screen)) {
        lvglResetSwipeBackVisualState(false);
        if (navigateBack) lvglNavigateBackBySwipe(LV_SCR_LOAD_ANIM_NONE);
        return;
    }

    lv_anim_del(screen, lvglSetObjXAnim);
    const lv_coord_t startX = lvglSwipeVisualActive ? lvglSwipeVisualOffsetX : static_cast<lv_coord_t>(lv_obj_get_x(screen));
    lv_obj_t *previewImg = (lvglSwipePreviewImg && lv_obj_is_valid(lvglSwipePreviewImg)) ? lvglSwipePreviewImg : nullptr;
    const lv_coord_t previewStartX = previewImg ? static_cast<lv_coord_t>(lv_obj_get_x(previewImg)) : 0;
    const lv_coord_t previewTargetX = navigateBack ? 0 : static_cast<lv_coord_t>(-DISPLAY_WIDTH);
    if (startX == targetX) {
        lvglResetSwipeBackVisualState(true);
        if (navigateBack) lvglNavigateBackBySwipe(LV_SCR_LOAD_ANIM_NONE);
        return;
    }

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, screen);
    lv_anim_set_exec_cb(&a, lvglSetObjXAnim);
    lv_anim_set_values(&a, startX, targetX);
    lv_anim_set_time(&a, lvglSwipeBackAnimDuration(startX, targetX));
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_ready_cb(&a, lvglSwipeBackVisualAnimReady);
    lv_anim_set_user_data(&a, reinterpret_cast<void *>(navigateBack ? 1 : 0));
    if (previewImg) {
        lv_anim_t previewAnim;
        lv_anim_init(&previewAnim);
        lv_anim_set_var(&previewAnim, previewImg);
        lv_anim_set_exec_cb(&previewAnim, lvglSetObjXAnim);
        lv_anim_set_values(&previewAnim, previewStartX, previewTargetX);
        lv_anim_set_time(&previewAnim, lvglSwipeBackAnimDuration(previewStartX, previewTargetX));
        lv_anim_set_path_cb(&previewAnim, lv_anim_path_ease_out);
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
    return displayAwake && (touchDown || lv_anim_count_running() > 0U);
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
}


void lvglTouchReadCb(lv_indev_drv_t *indev, lv_indev_data_t *data)
{
    (void)indev;
    int16_t x = 0;
    int16_t y = 0;
    const bool rawDown = readScreenTouch(x, y);
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
    if (lvglStyleMenuIconsSw) {
        lv_obj_set_style_bg_color(lvglStyleMenuIconsSw, lv_color_hex(0x48515C), LV_PART_MAIN);
        lv_obj_set_style_bg_color(lvglStyleMenuIconsSw, lv_color_hex(0x3A8F4B), LV_PART_INDICATOR | LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(lvglStyleMenuIconsSw, lv_color_hex(0xDCE7F2), LV_PART_KNOB);
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
    if (lvglKb && !lv_obj_has_flag(lvglKb, LV_OBJ_FLAG_HIDDEN)) lvglHideKeyboard();
    lv_async_call(lvglDeferredChatOpenFirstUnreadCallback, nullptr);
}

void lvglTopSoundTapEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (lvglClickSuppressed()) return;
    if (!displayAwake) return;
    if (lvglKb && !lv_obj_has_flag(lvglKb, LV_OBJ_FLAG_HIDDEN)) lvglHideKeyboard();
    lvglShowSoundPopup();
}

void lvglTopOtaTapEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (lvglClickSuppressed()) return;
    if (!displayAwake || !otaUpdateAvailable) return;
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
        case UI_CHAT: return tr(TXT_CHAT);
        case UI_CHAT_PEERS: return tr(TXT_CHAT_PEERS);
        case UI_MEDIA: return tr(TXT_MEDIA);
        case UI_INFO: return tr(TXT_INFO);
        case UI_CONFIG: return tr(TXT_CONFIG);
        case UI_CONFIG_BATTERY: return "Battery";
        case UI_CONFIG_STYLE: return tr(TXT_STYLE);
        case UI_CONFIG_LANGUAGE: return tr(TXT_LANGUAGE);
        case UI_CONFIG_OTA: return tr(TXT_OTA_UPDATES);
        case UI_CONFIG_HC12: return tr(TXT_HC12_CONFIG);
        case UI_CONFIG_HC12_TERMINAL: return tr(TXT_HC12_TERMINAL);
        case UI_CONFIG_HC12_INFO: return tr(TXT_HC12_INFO);
        case UI_CONFIG_MQTT_CONFIG: return tr(TXT_MQTT_CONFIG);
        case UI_CONFIG_MQTT_CONTROLS: return tr(TXT_MQTT_CONTROLS);
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
            freeMin = 22000U;
            largestMin = 9000U;
            break;
        case UI_MEDIA:
        case UI_INFO:
        case UI_CONFIG:
        case UI_CONFIG_BATTERY:
        case UI_CONFIG_STYLE:
        case UI_CONFIG_LANGUAGE:
        case UI_CONFIG_OTA:
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
        UI_WIFI_LIST,
        UI_INFO,
        UI_MEDIA,
        UI_GAMES,
        UI_CONFIG_HC12,
        UI_CONFIG_HC12_TERMINAL,
        UI_CONFIG_HC12_INFO,
        UI_CONFIG_MQTT_CONFIG,
        UI_CONFIG_MQTT_CONTROLS
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

struct LvglStyledButtonEntry {
    lv_obj_t *obj;
    lv_color_t baseColor;
    uint8_t flags;
};

static constexpr uint8_t LVGL_STYLED_BUTTON_FLAG_COMPACT = 0x01;
static constexpr size_t LVGL_STYLED_BUTTON_CAPACITY = 256;
static LvglStyledButtonEntry *lvglStyledButtons = nullptr;
static constexpr size_t LVGL_REORDER_ITEM_CAPACITY = 192;
static constexpr unsigned long UI_REORDER_HOLD_MS = 2000UL;
static constexpr lv_coord_t UI_REORDER_AUTO_SCROLL_MARGIN = 26;
static constexpr lv_coord_t UI_REORDER_AUTO_SCROLL_STEP = 10;
static constexpr lv_coord_t UI_REORDER_SWAP_HYSTERESIS = 10;

struct LvglReorderItemEntry {
    lv_obj_t *obj;
    lv_obj_t *parent;
    char prefKey[16];
    char itemKey[16];
};

struct UiReorderDragState {
    lv_obj_t *obj;
    lv_obj_t *parent;
    unsigned long pressedAtMs;
    lv_point_t pressPoint;
    lv_coord_t fingerOffsetY;
    lv_coord_t originalWidth;
    lv_coord_t originalHeight;
    bool active;
    bool cancelled;
    bool parentOverflowTemporarilyEnabled;
};

static LvglReorderItemEntry *lvglReorderItems = nullptr;
static UiReorderDragState lvglReorderDrag = {};

static bool lvglEnsureUiRegistryStorage()
{
    if (!lvglStyledButtons) {
        lvglStyledButtons = static_cast<LvglStyledButtonEntry *>(allocPreferPsram(sizeof(LvglStyledButtonEntry) * LVGL_STYLED_BUTTON_CAPACITY));
        if (!lvglStyledButtons) return false;
        memset(lvglStyledButtons, 0, sizeof(LvglStyledButtonEntry) * LVGL_STYLED_BUTTON_CAPACITY);
    }
    if (!lvglReorderItems) {
        lvglReorderItems = static_cast<LvglReorderItemEntry *>(allocPreferPsram(sizeof(LvglReorderItemEntry) * LVGL_REORDER_ITEM_CAPACITY));
        if (!lvglReorderItems) return false;
        memset(lvglReorderItems, 0, sizeof(LvglReorderItemEntry) * LVGL_REORDER_ITEM_CAPACITY);
    }
    return true;
}

static void lvglApplyReorderDragVisual(lv_obj_t *obj, bool active)
{
    if (!obj || !lv_obj_is_valid(obj)) return;

    const lv_coord_t outlineWidth = active ? 2 : 0;
    const lv_coord_t shadowWidth = active ? 26 : 0;
    const lv_coord_t shadowOfs = active ? 7 : 0;
    const lv_opa_t shadowOpa = active ? LV_OPA_70 : LV_OPA_TRANSP;

    lv_obj_set_style_transform_zoom(obj, 256, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_transform_zoom(obj, 256, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_transform_width(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_transform_width(obj, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_transform_height(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_transform_height(obj, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(obj, outlineWidth, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(obj, outlineWidth, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_pad(obj, active ? 1 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_pad(obj, active ? 1 : 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_color(obj, lv_color_hex(0xE7F2FF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_color(obj, lv_color_hex(0xE7F2FF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_opa(obj, active ? LV_OPA_60 : LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_opa(obj, active ? LV_OPA_60 : LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_shadow_width(obj, shadowWidth, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(obj, shadowWidth, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_shadow_ofs_x(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_x(obj, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_shadow_ofs_y(obj, shadowOfs, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_y(obj, shadowOfs, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_shadow_color(obj, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(obj, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_shadow_opa(obj, shadowOpa, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(obj, shadowOpa, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_invalidate(obj);
}

static uint32_t lvglRandomStyleAccent(uint32_t avoid = 0)
{
    static const uint32_t palette[] = {
        0x3A8F4B, 0x2F6D86, 0x8A5A25, 0x925A73, 0x5A4CC7, 0x2B7D7D, 0xA35757, 0x2E6F95
    };
    const size_t count = sizeof(palette) / sizeof(palette[0]);
    uint32_t chosen = palette[random(0, static_cast<int>(count))];
    if (chosen == avoid) chosen = palette[(random(0, static_cast<int>(count - 1)) + 1U) % count];
    return chosen;
}

static uint32_t lvglFnv1a32(const char *text)
{
    uint32_t hash = 2166136261UL;
    if (!text) return hash;
    while (*text) {
        hash ^= static_cast<uint8_t>(*text++);
        hash *= 16777619UL;
    }
    return hash;
}

static String lvglOrderTokenFromText(const char *prefix, const String &text)
{
    char buf[48];
    snprintf(buf, sizeof(buf), "%s%08lx", prefix ? prefix : "", static_cast<unsigned long>(lvglFnv1a32(text.c_str())));
    return String(buf);
}

static String lvglSymbolText(const char *symbol, const char *text)
{
    if (!text || !*text) return symbol ? String(symbol) : String("");
    if (!symbol || !*symbol) return String(text);
    return String(symbol) + " " + text;
}

static String lvglSymbolText(const char *symbol, const String &text)
{
    return lvglSymbolText(symbol, text.c_str());
}

static lv_color_t lvglActiveToggleGreen(bool compact)
{
    return compact ? lv_color_hex(0x2B5A32) : lv_color_hex(0x28552F);
}

static LvglReorderItemEntry *lvglFindReorderItemEntry(lv_obj_t *obj)
{
    if (!obj || !lvglReorderItems) return nullptr;
    for (size_t i = 0; i < LVGL_REORDER_ITEM_CAPACITY; ++i) {
        if (lvglReorderItems[i].obj == obj) return &lvglReorderItems[i];
    }
    return nullptr;
}

static const LvglReorderItemEntry *lvglFindReorderItemEntryConst(const lv_obj_t *obj)
{
    if (!obj || !lvglReorderItems) return nullptr;
    for (size_t i = 0; i < LVGL_REORDER_ITEM_CAPACITY; ++i) {
        if (lvglReorderItems[i].obj == obj) return &lvglReorderItems[i];
    }
    return nullptr;
}

static void lvglPersistReorderForParent(lv_obj_t *parent, const char *prefKey)
{
    if (!parent || !prefKey || !*prefKey) return;
    String order;
    const uint32_t childCount = lv_obj_get_child_cnt(parent);
    for (uint32_t i = 0; i < childCount; ++i) {
        lv_obj_t *child = lv_obj_get_child(parent, i);
        const LvglReorderItemEntry *entry = lvglFindReorderItemEntryConst(child);
        if (!entry || entry->parent != parent || strcmp(entry->prefKey, prefKey) != 0 || entry->itemKey[0] == '\0') continue;
        if (!order.isEmpty()) order += '|';
        order += entry->itemKey;
    }
    uiPrefs.begin("ui", false);
    if (order.isEmpty()) uiPrefs.remove(prefKey);
    else uiPrefs.putString(prefKey, order);
    uiPrefs.end();
}

static void lvglApplySavedOrder(lv_obj_t *parent, const char *prefKey)
{
    if (!parent || !prefKey || !*prefKey || !lvglReorderItems) return;
    uiPrefs.begin("ui", true);
    String order = uiPrefs.getString(prefKey, "");
    uiPrefs.end();
    if (order.isEmpty()) return;

    int nextIndex = 0;
    int start = 0;
    while (start < static_cast<int>(order.length())) {
        int sep = order.indexOf('|', start);
        if (sep < 0) sep = order.length();
        const String token = order.substring(start, sep);
        start = sep + 1;
        if (token.isEmpty()) continue;

        for (size_t i = 0; i < LVGL_REORDER_ITEM_CAPACITY; ++i) {
            LvglReorderItemEntry &entry = lvglReorderItems[i];
            if (!entry.obj || !lv_obj_is_valid(entry.obj)) continue;
            if (entry.parent != parent || strcmp(entry.prefKey, prefKey) != 0) continue;
            if (strcmp(entry.itemKey, token.c_str()) != 0) continue;
            lv_obj_move_to_index(entry.obj, nextIndex++);
            break;
        }
    }
}

static void lvglEndReorderDrag(bool persistOrder)
{
    if (!lvglReorderDrag.obj || !lv_obj_is_valid(lvglReorderDrag.obj)) {
        lvglReorderDrag = {};
        lvglGestureBlocked = false;
        lvglReorderOwnsHorizontalGesture = false;
        return;
    }

    lv_obj_t *obj = lvglReorderDrag.obj;
    lv_obj_t *parent = lvglReorderDrag.parent;
    const LvglReorderItemEntry *entry = lvglFindReorderItemEntryConst(obj);
    const bool wasActive = lvglReorderDrag.active;
    if (lvglReorderDrag.originalWidth > 0 && lvglReorderDrag.originalHeight > 0) {
        lv_obj_set_size(obj, lvglReorderDrag.originalWidth, lvglReorderDrag.originalHeight);
    }
    lv_obj_set_style_translate_y(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_translate_y(obj, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lvglApplyReorderDragVisual(obj, false);
    if (lvglReorderDrag.parentOverflowTemporarilyEnabled && parent && lv_obj_is_valid(parent)) {
        lv_obj_clear_flag(parent, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
    }

    if (persistOrder && parent && lv_obj_is_valid(parent) && entry) {
        lvglPersistReorderForParent(parent, entry->prefKey);
    }

    lvglReorderDrag = {};
    lvglGestureBlocked = false;
    lvglReorderOwnsHorizontalGesture = false;
    if (wasActive) lvglSuppressClicksAfterGesture();
    lvglRefreshAllButtonStyles();
}

static void lvglReorderItemDeleteEvent(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    if (!obj || !lvglReorderItems) return;
    for (size_t i = 0; i < LVGL_REORDER_ITEM_CAPACITY; ++i) {
        if (lvglReorderItems[i].obj == obj) {
            lvglReorderItems[i].obj = nullptr;
            lvglReorderItems[i].parent = nullptr;
            lvglReorderItems[i].prefKey[0] = '\0';
            lvglReorderItems[i].itemKey[0] = '\0';
        }
    }
    if (lvglReorderDrag.obj == obj) lvglEndReorderDrag(false);
}

static void lvglReorderItemEvent(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    LvglReorderItemEntry *entry = lvglFindReorderItemEntry(obj);
    if (!obj || !entry || !entry->parent || !lv_obj_is_valid(entry->parent)) return;

    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED) {
        lv_indev_t *indev = lv_indev_get_act();
        lv_point_t pt = {0, 0};
        if (indev) lv_indev_get_point(indev, &pt);
        lvglReorderDrag.obj = obj;
        lvglReorderDrag.parent = entry->parent;
        lvglReorderDrag.pressedAtMs = millis();
        lvglReorderDrag.pressPoint = pt;
        lvglReorderDrag.fingerOffsetY = 0;
        lvglReorderDrag.originalWidth = lv_obj_get_width(obj);
        lvglReorderDrag.originalHeight = lv_obj_get_height(obj);
        lvglReorderDrag.active = false;
        lvglReorderDrag.cancelled = false;
        lvglReorderDrag.parentOverflowTemporarilyEnabled = false;
        return;
    }

    if (code == LV_EVENT_PRESSING) {
        if (lvglReorderDrag.obj != obj || lvglReorderDrag.parent != entry->parent) return;
        lv_indev_t *indev = lv_indev_get_act();
        if (!indev) return;
        lv_point_t pt = {0, 0};
        lv_indev_get_point(indev, &pt);

        if (!lvglReorderDrag.active) {
            if (!lvglReorderDrag.cancelled) {
                const int dx = static_cast<int>(pt.x) - static_cast<int>(lvglReorderDrag.pressPoint.x);
                const int dy = static_cast<int>(pt.y) - static_cast<int>(lvglReorderDrag.pressPoint.y);
                const int absDx = abs(dx);
                const int absDy = abs(dy);
                if ((absDx >= SWIPE_LOCK_MIN_DX && absDx > (absDy * 2)) ||
                    (absDy >= SWIPE_CANCEL_VERTICAL_DY && absDy > absDx)) {
                    lvglReorderDrag.cancelled = true;
                }
            }
            if (lvglReorderDrag.cancelled) return;
            if (static_cast<unsigned long>(millis() - lvglReorderDrag.pressedAtMs) < UI_REORDER_HOLD_MS) return;
            lv_area_t startArea;
            lv_obj_get_coords(obj, &startArea);
            lvglReorderDrag.fingerOffsetY = static_cast<lv_coord_t>(pt.y - ((startArea.y1 + startArea.y2) / 2));
            lvglReorderDrag.active = true;
            lvglGestureBlocked = true;
            lvglReorderOwnsHorizontalGesture = true;
            lvglResetGestureTracking();
            if (!lv_obj_has_flag(entry->parent, LV_OBJ_FLAG_OVERFLOW_VISIBLE)) {
                lv_obj_add_flag(entry->parent, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
                lvglReorderDrag.parentOverflowTemporarilyEnabled = true;
            }
            const lv_coord_t dragWidth = static_cast<lv_coord_t>(lvglReorderDrag.originalWidth + max<lv_coord_t>(8, lvglReorderDrag.originalWidth / 8));
            const lv_coord_t dragHeight = static_cast<lv_coord_t>(lvglReorderDrag.originalHeight + max<lv_coord_t>(4, lvglReorderDrag.originalHeight / 8));
            lv_obj_set_size(obj, dragWidth, dragHeight);
            lvglApplyReorderDragVisual(obj, true);
        }

        if (lv_obj_has_flag(entry->parent, LV_OBJ_FLAG_SCROLLABLE)) {
            lv_area_t parentArea;
            lv_obj_get_coords(entry->parent, &parentArea);
            if (pt.y < (parentArea.y1 + UI_REORDER_AUTO_SCROLL_MARGIN)) {
                lv_obj_scroll_by(entry->parent, 0, -UI_REORDER_AUTO_SCROLL_STEP, LV_ANIM_OFF);
            } else if (pt.y > (parentArea.y2 - UI_REORDER_AUTO_SCROLL_MARGIN)) {
                lv_obj_scroll_by(entry->parent, 0, UI_REORDER_AUTO_SCROLL_STEP, LV_ANIM_OFF);
            }
        }

        lv_obj_set_style_translate_y(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_translate_y(obj, 0, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_update_layout(entry->parent);

        lv_area_t objArea;
        lv_obj_get_coords(obj, &objArea);
        const lv_coord_t baseCenterY = static_cast<lv_coord_t>((objArea.y1 + objArea.y2) / 2);
        const lv_coord_t desiredCenterY = static_cast<lv_coord_t>(pt.y - lvglReorderDrag.fingerOffsetY);
        const lv_coord_t deltaY = desiredCenterY - baseCenterY;

        int32_t targetIndex = lv_obj_get_index(obj);
        const uint32_t childCount = lv_obj_get_child_cnt(entry->parent);
        while (targetIndex > 0) {
            lv_obj_t *prevChild = lv_obj_get_child(entry->parent, targetIndex - 1);
            if (!prevChild || prevChild == obj) break;
            lv_area_t prevArea;
            lv_obj_get_coords(prevChild, &prevArea);
            const lv_coord_t prevCenterY = static_cast<lv_coord_t>((prevArea.y1 + prevArea.y2) / 2);
            const lv_coord_t prevHalfH = static_cast<lv_coord_t>((prevArea.y2 - prevArea.y1 + 1) / 2);
            const lv_coord_t swapThreshold = max<lv_coord_t>(UI_REORDER_SWAP_HYSTERESIS, prevHalfH / 3);
            if (desiredCenterY >= (prevCenterY - swapThreshold)) break;
            targetIndex--;
        }
        while (targetIndex < static_cast<int32_t>(childCount) - 1) {
            lv_obj_t *nextChild = lv_obj_get_child(entry->parent, targetIndex + 1);
            if (!nextChild || nextChild == obj) break;
            lv_area_t nextArea;
            lv_obj_get_coords(nextChild, &nextArea);
            const lv_coord_t nextCenterY = static_cast<lv_coord_t>((nextArea.y1 + nextArea.y2) / 2);
            const lv_coord_t nextHalfH = static_cast<lv_coord_t>((nextArea.y2 - nextArea.y1 + 1) / 2);
            const lv_coord_t swapThreshold = max<lv_coord_t>(UI_REORDER_SWAP_HYSTERESIS, nextHalfH / 3);
            if (desiredCenterY <= (nextCenterY + swapThreshold)) break;
            targetIndex++;
        }

        if (lv_obj_get_index(obj) != targetIndex) {
            lv_obj_move_to_index(obj, targetIndex);
            lv_obj_update_layout(entry->parent);
            lv_obj_get_coords(obj, &objArea);
        }

        const lv_coord_t refreshedCenterY = static_cast<lv_coord_t>((objArea.y1 + objArea.y2) / 2);
        const lv_coord_t refreshedDeltaY = desiredCenterY - refreshedCenterY;
        lv_obj_set_style_translate_y(obj, refreshedDeltaY, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_translate_y(obj, refreshedDeltaY, LV_PART_MAIN | LV_STATE_PRESSED);
        return;
    }

    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        if (lvglReorderDrag.obj == obj) {
            const bool persistOrder = lvglReorderDrag.active;
            if (lvglReorderDrag.active) lvglEndReorderDrag(persistOrder);
            else {
                lvglReorderDrag = {};
                lvglGestureBlocked = false;
                lvglReorderOwnsHorizontalGesture = false;
            }
        } else {
            lvglReorderOwnsHorizontalGesture = false;
        }
    }
}

static void lvglRegisterReorderableItem(lv_obj_t *obj, const char *prefKey, const char *itemKey)
{
    if (!obj || !prefKey || !*prefKey || !itemKey || !*itemKey) return;
    if (!lvglEnsureUiRegistryStorage()) return;
    lv_obj_t *parent = lv_obj_get_parent(obj);
    if (!parent) return;

    LvglReorderItemEntry *entry = lvglFindReorderItemEntry(obj);
    bool isNew = false;
    if (!entry) {
        for (size_t i = 0; i < LVGL_REORDER_ITEM_CAPACITY; ++i) {
            if (!lvglReorderItems[i].obj || !lv_obj_is_valid(lvglReorderItems[i].obj)) {
                entry = &lvglReorderItems[i];
                isNew = true;
                break;
            }
        }
    }
    if (!entry) return;

    entry->obj = obj;
    entry->parent = parent;
    snprintf(entry->prefKey, sizeof(entry->prefKey), "%s", prefKey);
    snprintf(entry->itemKey, sizeof(entry->itemKey), "%s", itemKey);

    if (!lv_obj_has_flag(obj, LV_OBJ_FLAG_CLICKABLE)) lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);

    if (isNew) {
        lv_obj_add_event_cb(obj, lvglReorderItemEvent, LV_EVENT_PRESSED, nullptr);
        lv_obj_add_event_cb(obj, lvglReorderItemEvent, LV_EVENT_PRESSING, nullptr);
        lv_obj_add_event_cb(obj, lvglReorderItemEvent, LV_EVENT_RELEASED, nullptr);
        lv_obj_add_event_cb(obj, lvglReorderItemEvent, LV_EVENT_PRESS_LOST, nullptr);
        lv_obj_add_event_cb(obj, lvglReorderItemDeleteEvent, LV_EVENT_DELETE, nullptr);
    }
}

static void lvglStyledButtonDeleteEvent(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    if (!obj || !lvglStyledButtons) return;
    for (size_t i = 0; i < LVGL_STYLED_BUTTON_CAPACITY; ++i) {
        if (lvglStyledButtons[i].obj == obj) lvglStyledButtons[i].obj = nullptr;
    }
}

static LvglStyledButtonEntry *lvglFindStyledButtonEntry(lv_obj_t *obj)
{
    if (!obj || !lvglStyledButtons) return nullptr;
    for (size_t i = 0; i < LVGL_STYLED_BUTTON_CAPACITY; ++i) {
        if (lvglStyledButtons[i].obj == obj) return &lvglStyledButtons[i];
    }
    return nullptr;
}

static const LvglStyledButtonEntry *lvglFindStyledButtonEntryConst(const lv_obj_t *obj)
{
    if (!obj || !lvglStyledButtons) return nullptr;
    for (size_t i = 0; i < LVGL_STYLED_BUTTON_CAPACITY; ++i) {
        if (lvglStyledButtons[i].obj == obj) return &lvglStyledButtons[i];
    }
    return nullptr;
}

static inline bool lvglBlackButtonThemeActive()
{
    return uiButtonStyleMode == UI_BUTTON_STYLE_BLACK;
}

static bool internetTimeValid()
{
    return time(nullptr) > 1700000000;
}

static void syncInternetTimeIfNeeded(bool force)
{
    if (airplaneModeEnabled || !wifiConnectedSafe()) return;
    const unsigned long nowMs = millis();
    const unsigned long minInterval = internetTimeValid() ? NTP_SYNC_REFRESH_MS : NTP_SYNC_RETRY_MS;
    if (!force && static_cast<unsigned long>(nowMs - topBarTimeSyncLastAttemptMs) < minInterval) return;
    configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2, NTP_SERVER_3);
    topBarTimeSyncLastAttemptMs = nowMs;
}

static String buildGmtOffsetDropdownOptions()
{
    String options;
    for (int offset = TOP_BAR_GMT_MIN; offset <= TOP_BAR_GMT_MAX; ++offset) {
        if (!options.isEmpty()) options += '\n';
        options += "GMT";
        if (offset >= 0) options += '+';
        options += String(offset);
    }
    return options;
}

static const char *uiLanguageOptionName(UiLanguage lang)
{
    switch (lang) {
        case UI_LANG_RUSSIAN: return "Russian";
        case UI_LANG_CHINESE: return "Chinese";
        case UI_LANG_FRENCH: return "French";
        case UI_LANG_TURKISH: return "Turkish";
        case UI_LANG_ITALIAN: return "Italian";
        case UI_LANG_GERMAN: return "German";
        case UI_LANG_JAPANESE: return "Japanese";
        case UI_LANG_KOREAN: return "Korean";
        case UI_LANG_ENGLISH:
        default: return "English";
    }
}

static const char *vibrationIntensityLabel(VibrationIntensity intensity)
{
    switch (uiLanguage) {
        case UI_LANG_FRENCH:
            switch (intensity) {
                case VIBRATION_INTENSITY_LOW: return "Faible";
                case VIBRATION_INTENSITY_MEDIUM: return "Moyenne";
                case VIBRATION_INTENSITY_HIGH: return "Elevee";
                default: return "Moyenne";
            }
        case UI_LANG_TURKISH:
            switch (intensity) {
                case VIBRATION_INTENSITY_LOW: return "Dusuk";
                case VIBRATION_INTENSITY_MEDIUM: return "Orta";
                case VIBRATION_INTENSITY_HIGH: return "Yuksek";
                default: return "Orta";
            }
        case UI_LANG_ITALIAN:
            switch (intensity) {
                case VIBRATION_INTENSITY_LOW: return "Bassa";
                case VIBRATION_INTENSITY_MEDIUM: return "Media";
                case VIBRATION_INTENSITY_HIGH: return "Alta";
                default: return "Media";
            }
        case UI_LANG_GERMAN:
            switch (intensity) {
                case VIBRATION_INTENSITY_LOW: return "Niedrig";
                case VIBRATION_INTENSITY_MEDIUM: return "Mittel";
                case VIBRATION_INTENSITY_HIGH: return "Hoch";
                default: return "Mittel";
            }
        default:
            break;
    }
    switch (intensity) {
        case VIBRATION_INTENSITY_LOW: return "Low";
        case VIBRATION_INTENSITY_MEDIUM: return "Medium";
        case VIBRATION_INTENSITY_HIGH: return "High";
        default: return "Medium";
    }
}

static String buildLanguageDropdownOptions()
{
    String options;
    for (uint8_t i = 0; i < static_cast<uint8_t>(UI_LANG_COUNT); ++i) {
        if (!options.isEmpty()) options += '\n';
        options += uiLanguageOptionName(static_cast<UiLanguage>(i));
    }
    return options;
}

static const char *radioModuleLabel(RadioModuleType module)
{
    switch (module) {
        case RADIO_MODULE_E220: return "Ebyte E220-400T22D";
        case RADIO_MODULE_HC12:
        default: return "HC-12";
    }
}

static String buildRadioModuleDropdownOptions()
{
    String options = radioModuleLabel(RADIO_MODULE_HC12);
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    options += "\n";
    options += radioModuleLabel(RADIO_MODULE_E220);
#endif
    return options;
}

static String buildVibrationIntensityDropdownOptions()
{
    String options;
    for (int i = 0; i < static_cast<int>(VIBRATION_INTENSITY_COUNT); ++i) {
        if (i > 0) options += "\n";
        options += vibrationIntensityLabel(static_cast<VibrationIntensity>(i));
    }
    return options;
}

static void saveSoundPrefs()
{
    uiPrefs.begin("ui", false);
    uiPrefs.putUChar("sys_vol", mediaVolumePercent);
    uiPrefs.putBool("vib_en", vibrationEnabled);
    uiPrefs.putUChar("vib_int", static_cast<uint8_t>(vibrationIntensity));
    uiPrefs.end();
}

static uint8_t uiSoundMode()
{
    if (mediaVolumePercent > 0) return 2;
    if (vibrationEnabled) return 1;
    return 0;
}

static const lv_img_dsc_t *uiSoundModeIcon()
{
    switch (uiSoundMode()) {
        case 0: return &img_silent_small_icon;
        case 1: return &img_vibro_small_icon;
        case 2:
        default: return &img_speaker_small_icon;
    }
}

static const lv_img_dsc_t *uiVolumeIcon()
{
    return mediaVolumePercent > 0 ? &img_speaker_small_icon : &img_silent_small_icon;
}

static const char *messageBeepToneLabel(MessageBeepTone tone)
{
    switch (tone) {
        case MESSAGE_BEEP_SINGLE: return "One Beep";
        case MESSAGE_BEEP_DOUBLE_SHORT: return "Two Short";
        case MESSAGE_BEEP_ASCEND: return "Ascend";
        case MESSAGE_BEEP_DESCEND: return "Descend";
        case MESSAGE_BEEP_DOORBELL: return "Doorbell";
        case MESSAGE_BEEP_WESTMINSTER: return "Westminster";
        case MESSAGE_BEEP_FUR_ELISE: return "Fur Elise";
        case MESSAGE_BEEP_ODE_TO_JOY: return "Ode to Joy";
        default: return "Two Short";
    }
}

static String buildMessageBeepDropdownOptions()
{
    String options;
    for (int i = 0; i < static_cast<int>(MESSAGE_BEEP_TONE_COUNT); ++i) {
        if (i > 0) options += "\n";
        options += messageBeepToneLabel(static_cast<MessageBeepTone>(i));
    }
    return options;
}

struct ChatBeepStep {
    uint16_t freq;
    uint16_t durationMs;
};

struct ChatBeepPattern {
    const ChatBeepStep *steps;
    uint8_t count;
};

static constexpr ChatBeepStep CHAT_BEEP_PATTERN_SINGLE[] = {
    {1760, 140}
};
static constexpr ChatBeepStep CHAT_BEEP_PATTERN_DOUBLE_SHORT[] = {
    {1760, 70}, {0, 55}, {1320, 95}
};
static constexpr ChatBeepStep CHAT_BEEP_PATTERN_ASCEND[] = {
    {988, 70}, {0, 35}, {1319, 70}, {0, 35}, {1760, 110}
};
static constexpr ChatBeepStep CHAT_BEEP_PATTERN_DESCEND[] = {
    {1760, 70}, {0, 35}, {1319, 70}, {0, 35}, {988, 110}
};
static constexpr ChatBeepStep CHAT_BEEP_PATTERN_DOORBELL[] = {
    {1047, 85}, {0, 40}, {1568, 150}
};
static constexpr ChatBeepStep CHAT_BEEP_PATTERN_WESTMINSTER[] = {
    {1568, 90}, {0, 30}, {1175, 90}, {0, 30}, {1319, 90}, {0, 30}, {988, 160}
};
static constexpr ChatBeepStep CHAT_BEEP_PATTERN_FUR_ELISE[] = {
    {1319, 90}, {0, 25}, {1245, 90}, {0, 25}, {1319, 90}, {0, 25}, {1245, 90}, {0, 25}, {1319, 120}
};
static constexpr ChatBeepStep CHAT_BEEP_PATTERN_ODE_TO_JOY[] = {
    {1319, 85}, {0, 25}, {1319, 85}, {0, 25}, {1480, 85}, {0, 25}, {1568, 85}, {0, 25}, {1568, 120}
};

static ChatBeepPattern chatMessageBeepPattern(MessageBeepTone tone)
{
    switch (tone) {
        case MESSAGE_BEEP_SINGLE: return {CHAT_BEEP_PATTERN_SINGLE, static_cast<uint8_t>(sizeof(CHAT_BEEP_PATTERN_SINGLE) / sizeof(CHAT_BEEP_PATTERN_SINGLE[0]))};
        case MESSAGE_BEEP_DOUBLE_SHORT: return {CHAT_BEEP_PATTERN_DOUBLE_SHORT, static_cast<uint8_t>(sizeof(CHAT_BEEP_PATTERN_DOUBLE_SHORT) / sizeof(CHAT_BEEP_PATTERN_DOUBLE_SHORT[0]))};
        case MESSAGE_BEEP_ASCEND: return {CHAT_BEEP_PATTERN_ASCEND, static_cast<uint8_t>(sizeof(CHAT_BEEP_PATTERN_ASCEND) / sizeof(CHAT_BEEP_PATTERN_ASCEND[0]))};
        case MESSAGE_BEEP_DESCEND: return {CHAT_BEEP_PATTERN_DESCEND, static_cast<uint8_t>(sizeof(CHAT_BEEP_PATTERN_DESCEND) / sizeof(CHAT_BEEP_PATTERN_DESCEND[0]))};
        case MESSAGE_BEEP_DOORBELL: return {CHAT_BEEP_PATTERN_DOORBELL, static_cast<uint8_t>(sizeof(CHAT_BEEP_PATTERN_DOORBELL) / sizeof(CHAT_BEEP_PATTERN_DOORBELL[0]))};
        case MESSAGE_BEEP_WESTMINSTER: return {CHAT_BEEP_PATTERN_WESTMINSTER, static_cast<uint8_t>(sizeof(CHAT_BEEP_PATTERN_WESTMINSTER) / sizeof(CHAT_BEEP_PATTERN_WESTMINSTER[0]))};
        case MESSAGE_BEEP_FUR_ELISE: return {CHAT_BEEP_PATTERN_FUR_ELISE, static_cast<uint8_t>(sizeof(CHAT_BEEP_PATTERN_FUR_ELISE) / sizeof(CHAT_BEEP_PATTERN_FUR_ELISE[0]))};
        case MESSAGE_BEEP_ODE_TO_JOY: return {CHAT_BEEP_PATTERN_ODE_TO_JOY, static_cast<uint8_t>(sizeof(CHAT_BEEP_PATTERN_ODE_TO_JOY) / sizeof(CHAT_BEEP_PATTERN_ODE_TO_JOY[0]))};
        default: return {CHAT_BEEP_PATTERN_DOUBLE_SHORT, static_cast<uint8_t>(sizeof(CHAT_BEEP_PATTERN_DOUBLE_SHORT) / sizeof(CHAT_BEEP_PATTERN_DOUBLE_SHORT[0]))};
    }
}

static const char *tr(UiTextId id)
{
    switch (uiLanguage) {
        case UI_LANG_RUSSIAN:
            switch (id) {
                case TXT_CHAT: return "Чат";
                case TXT_CHAT_PEERS: return "Контакты чата";
                case TXT_MEDIA: return "Медиа";
                case TXT_INFO: return "Инфо";
                case TXT_GAMES: return "Игры";
                case TXT_CONFIG: return "Настройки";
                case TXT_AIRPLANE_ON: return "Авиарежим: ВКЛ";
                case TXT_AIRPLANE_OFF: return "Авиарежим: ВЫКЛ";
                case TXT_AP_MODE_ON: return "AP режим: ВКЛ";
                case TXT_AP_MODE_OFF: return "AP режим: ВЫКЛ";
                case TXT_WIFI_CONFIG: return "WiFi";
                case TXT_HC12_CONFIG: return "HC12";
                case TXT_STYLE: return "Стиль";
                case TXT_MQTT_CONFIG: return "MQTT";
                case TXT_MQTT_CONTROLS: return "MQTT кнопки";
                case TXT_SCREENSHOT: return "Скриншот";
                case TXT_LANGUAGE: return "Язык";
                case TXT_OTA_UPDATES: return "OTA обновления";
                case TXT_DEVICE_NAME: return "Имя устройства";
                case TXT_SAVE: return "Сохранить";
                case TXT_BRIGHTNESS: return "Яркость";
                case TXT_VOLUME: return "Громкость";
                case TXT_RGB_LED: return "RGB LED";
                case TXT_SELECT_DISPLAY_LANGUAGE: return "Выберите язык экрана";
                case TXT_LANGUAGE_SAVED: return "Язык сохранен";
                case TXT_HC12_TERMINAL: return "Терминал HC12";
                case TXT_HC12_INFO: return "Инфо HC12";
                case TXT_CHECKERS: return "Шашки";
                case TXT_SNAKE_3D: return "Змейка 3D";
                case TXT_SCREEN:
                default: return "Экран";
            }
        case UI_LANG_CHINESE:
            switch (id) {
                case TXT_CHAT: return "聊天";
                case TXT_CHAT_PEERS: return "聊天联系人";
                case TXT_MEDIA: return "媒体";
                case TXT_INFO: return "信息";
                case TXT_GAMES: return "游戏";
                case TXT_CONFIG: return "设置";
                case TXT_AIRPLANE_ON: return "飞行模式: 开";
                case TXT_AIRPLANE_OFF: return "飞行模式: 关";
                case TXT_AP_MODE_ON: return "AP模式: 开";
                case TXT_AP_MODE_OFF: return "AP模式: 关";
                case TXT_WIFI_CONFIG: return "WiFi设置";
                case TXT_HC12_CONFIG: return "HC12设置";
                case TXT_STYLE: return "样式";
                case TXT_MQTT_CONFIG: return "MQTT设置";
                case TXT_MQTT_CONTROLS: return "MQTT控制";
                case TXT_SCREENSHOT: return "截图";
                case TXT_LANGUAGE: return "语言";
                case TXT_OTA_UPDATES: return "OTA更新";
                case TXT_DEVICE_NAME: return "设备名称";
                case TXT_SAVE: return "保存";
                case TXT_BRIGHTNESS: return "亮度";
                case TXT_VOLUME: return "音量";
                case TXT_RGB_LED: return "RGB 灯";
                case TXT_SELECT_DISPLAY_LANGUAGE: return "选择显示语言";
                case TXT_LANGUAGE_SAVED: return "语言已保存";
                case TXT_HC12_TERMINAL: return "HC12终端";
                case TXT_HC12_INFO: return "HC12信息";
                case TXT_CHECKERS: return "跳棋";
                case TXT_SNAKE_3D: return "3D贪吃蛇";
                case TXT_SCREEN:
                default: return "界面";
            }
        case UI_LANG_FRENCH:
            switch (id) {
                case TXT_CHAT: return "Chat";
                case TXT_CHAT_PEERS: return "Contacts chat";
                case TXT_MEDIA: return "Media";
                case TXT_INFO: return "Infos";
                case TXT_GAMES: return "Jeux";
                case TXT_CONFIG: return "Config";
                case TXT_AIRPLANE_ON: return "Mode avion: ON";
                case TXT_AIRPLANE_OFF: return "Mode avion: OFF";
                case TXT_AP_MODE_ON: return "Mode AP: ON";
                case TXT_AP_MODE_OFF: return "Mode AP: OFF";
                case TXT_WIFI_CONFIG: return "Config WiFi";
                case TXT_HC12_CONFIG: return "Config Radio";
                case TXT_STYLE: return "Style";
                case TXT_MQTT_CONFIG: return "Config MQTT";
                case TXT_MQTT_CONTROLS: return "Commandes MQTT";
                case TXT_SCREENSHOT: return "Capture";
                case TXT_LANGUAGE: return "Langue";
                case TXT_OTA_UPDATES: return "Mises a jour OTA";
                case TXT_DEVICE_NAME: return "Nom appareil";
                case TXT_SAVE: return "Enregistrer";
                case TXT_BRIGHTNESS: return "Luminosite";
                case TXT_VOLUME: return "Volume";
                case TXT_RGB_LED: return "LED RGB";
                case TXT_SELECT_DISPLAY_LANGUAGE: return "Choisir la langue";
                case TXT_LANGUAGE_SAVED: return "Langue enregistree";
                case TXT_HC12_TERMINAL: return "Terminal Radio";
                case TXT_HC12_INFO: return "Infos Radio";
                case TXT_CHECKERS: return "Dames";
                case TXT_SNAKE_3D: return "Snake 3D";
                case TXT_SCREEN:
                default: return "Ecran";
            }
        case UI_LANG_TURKISH:
            switch (id) {
                case TXT_CHAT: return "Sohbet";
                case TXT_CHAT_PEERS: return "Sohbet Kisileri";
                case TXT_MEDIA: return "Medya";
                case TXT_INFO: return "Bilgi";
                case TXT_GAMES: return "Oyunlar";
                case TXT_CONFIG: return "Ayarlar";
                case TXT_AIRPLANE_ON: return "Ucak modu: Acik";
                case TXT_AIRPLANE_OFF: return "Ucak modu: Kapali";
                case TXT_AP_MODE_ON: return "AP modu: Acik";
                case TXT_AP_MODE_OFF: return "AP modu: Kapali";
                case TXT_WIFI_CONFIG: return "WiFi Ayari";
                case TXT_HC12_CONFIG: return "Radyo Ayari";
                case TXT_STYLE: return "Stil";
                case TXT_MQTT_CONFIG: return "MQTT Ayari";
                case TXT_MQTT_CONTROLS: return "MQTT Kontroller";
                case TXT_SCREENSHOT: return "Ekran Goruntusu";
                case TXT_LANGUAGE: return "Dil";
                case TXT_OTA_UPDATES: return "OTA Guncelleme";
                case TXT_DEVICE_NAME: return "Cihaz Adi";
                case TXT_SAVE: return "Kaydet";
                case TXT_BRIGHTNESS: return "Parlaklik";
                case TXT_VOLUME: return "Ses";
                case TXT_RGB_LED: return "RGB LED";
                case TXT_SELECT_DISPLAY_LANGUAGE: return "Gosterim dilini sec";
                case TXT_LANGUAGE_SAVED: return "Dil kaydedildi";
                case TXT_HC12_TERMINAL: return "Radyo Terminal";
                case TXT_HC12_INFO: return "Radyo Bilgi";
                case TXT_CHECKERS: return "Dama";
                case TXT_SNAKE_3D: return "Yilan 3D";
                case TXT_SCREEN:
                default: return "Ekran";
            }
        case UI_LANG_ITALIAN:
            switch (id) {
                case TXT_CHAT: return "Chat";
                case TXT_CHAT_PEERS: return "Contatti chat";
                case TXT_MEDIA: return "Media";
                case TXT_INFO: return "Info";
                case TXT_GAMES: return "Giochi";
                case TXT_CONFIG: return "Config";
                case TXT_AIRPLANE_ON: return "Modalita aereo: ON";
                case TXT_AIRPLANE_OFF: return "Modalita aereo: OFF";
                case TXT_AP_MODE_ON: return "Modalita AP: ON";
                case TXT_AP_MODE_OFF: return "Modalita AP: OFF";
                case TXT_WIFI_CONFIG: return "Config WiFi";
                case TXT_HC12_CONFIG: return "Config Radio";
                case TXT_STYLE: return "Stile";
                case TXT_MQTT_CONFIG: return "Config MQTT";
                case TXT_MQTT_CONTROLS: return "Controlli MQTT";
                case TXT_SCREENSHOT: return "Screenshot";
                case TXT_LANGUAGE: return "Lingua";
                case TXT_OTA_UPDATES: return "Aggiornamenti OTA";
                case TXT_DEVICE_NAME: return "Nome dispositivo";
                case TXT_SAVE: return "Salva";
                case TXT_BRIGHTNESS: return "Luminosita";
                case TXT_VOLUME: return "Volume";
                case TXT_RGB_LED: return "LED RGB";
                case TXT_SELECT_DISPLAY_LANGUAGE: return "Seleziona lingua display";
                case TXT_LANGUAGE_SAVED: return "Lingua salvata";
                case TXT_HC12_TERMINAL: return "Terminale Radio";
                case TXT_HC12_INFO: return "Info Radio";
                case TXT_CHECKERS: return "Dama";
                case TXT_SNAKE_3D: return "Snake 3D";
                case TXT_SCREEN:
                default: return "Schermo";
            }
        case UI_LANG_GERMAN:
            switch (id) {
                case TXT_CHAT: return "Chat";
                case TXT_CHAT_PEERS: return "Chat Kontakte";
                case TXT_MEDIA: return "Medien";
                case TXT_INFO: return "Info";
                case TXT_GAMES: return "Spiele";
                case TXT_CONFIG: return "Konfig";
                case TXT_AIRPLANE_ON: return "Flugmodus: EIN";
                case TXT_AIRPLANE_OFF: return "Flugmodus: AUS";
                case TXT_AP_MODE_ON: return "AP Modus: EIN";
                case TXT_AP_MODE_OFF: return "AP Modus: AUS";
                case TXT_WIFI_CONFIG: return "WLAN Konfig";
                case TXT_HC12_CONFIG: return "Radio Konfig";
                case TXT_STYLE: return "Stil";
                case TXT_MQTT_CONFIG: return "MQTT Konfig";
                case TXT_MQTT_CONTROLS: return "MQTT Steuerung";
                case TXT_SCREENSHOT: return "Screenshot";
                case TXT_LANGUAGE: return "Sprache";
                case TXT_OTA_UPDATES: return "OTA Updates";
                case TXT_DEVICE_NAME: return "Geratename";
                case TXT_SAVE: return "Speichern";
                case TXT_BRIGHTNESS: return "Helligkeit";
                case TXT_VOLUME: return "Lautstarke";
                case TXT_RGB_LED: return "RGB LED";
                case TXT_SELECT_DISPLAY_LANGUAGE: return "Displaysprache wahlen";
                case TXT_LANGUAGE_SAVED: return "Sprache gespeichert";
                case TXT_HC12_TERMINAL: return "Radio Terminal";
                case TXT_HC12_INFO: return "Radio Info";
                case TXT_CHECKERS: return "Dame";
                case TXT_SNAKE_3D: return "Snake 3D";
                case TXT_SCREEN:
                default: return "Bildschirm";
            }
        case UI_LANG_JAPANESE:
            switch (id) {
                case TXT_CHAT: return "チャット";
                case TXT_CHAT_PEERS: return "チャット相手";
                case TXT_MEDIA: return "メディア";
                case TXT_INFO: return "情報";
                case TXT_GAMES: return "ゲーム";
                case TXT_CONFIG: return "設定";
                case TXT_AIRPLANE_ON: return "機内モード: ON";
                case TXT_AIRPLANE_OFF: return "機内モード: OFF";
                case TXT_AP_MODE_ON: return "APモード: ON";
                case TXT_AP_MODE_OFF: return "APモード: OFF";
                case TXT_WIFI_CONFIG: return "WiFi設定";
                case TXT_HC12_CONFIG: return "HC12設定";
                case TXT_STYLE: return "スタイル";
                case TXT_MQTT_CONFIG: return "MQTT設定";
                case TXT_MQTT_CONTROLS: return "MQTT操作";
                case TXT_SCREENSHOT: return "スクリーンショット";
                case TXT_LANGUAGE: return "言語";
                case TXT_OTA_UPDATES: return "OTA更新";
                case TXT_DEVICE_NAME: return "デバイス名";
                case TXT_SAVE: return "保存";
                case TXT_BRIGHTNESS: return "明るさ";
                case TXT_VOLUME: return "音量";
                case TXT_RGB_LED: return "RGB LED";
                case TXT_SELECT_DISPLAY_LANGUAGE: return "表示言語を選択";
                case TXT_LANGUAGE_SAVED: return "言語を保存しました";
                case TXT_HC12_TERMINAL: return "HC12端末";
                case TXT_HC12_INFO: return "HC12情報";
                case TXT_CHECKERS: return "チェッカー";
                case TXT_SNAKE_3D: return "スネーク3D";
                case TXT_SCREEN:
                default: return "画面";
            }
        case UI_LANG_KOREAN:
            switch (id) {
                case TXT_CHAT: return "채팅";
                case TXT_CHAT_PEERS: return "채팅 연락처";
                case TXT_MEDIA: return "미디어";
                case TXT_INFO: return "정보";
                case TXT_GAMES: return "게임";
                case TXT_CONFIG: return "설정";
                case TXT_AIRPLANE_ON: return "비행기 모드: 켬";
                case TXT_AIRPLANE_OFF: return "비행기 모드: 끔";
                case TXT_AP_MODE_ON: return "AP 모드: 켬";
                case TXT_AP_MODE_OFF: return "AP 모드: 끔";
                case TXT_WIFI_CONFIG: return "WiFi 설정";
                case TXT_HC12_CONFIG: return "HC12 설정";
                case TXT_STYLE: return "스타일";
                case TXT_MQTT_CONFIG: return "MQTT 설정";
                case TXT_MQTT_CONTROLS: return "MQTT 제어";
                case TXT_SCREENSHOT: return "스크린샷";
                case TXT_LANGUAGE: return "언어";
                case TXT_OTA_UPDATES: return "OTA 업데이트";
                case TXT_DEVICE_NAME: return "장치 이름";
                case TXT_SAVE: return "저장";
                case TXT_BRIGHTNESS: return "밝기";
                case TXT_VOLUME: return "볼륨";
                case TXT_RGB_LED: return "RGB LED";
                case TXT_SELECT_DISPLAY_LANGUAGE: return "표시 언어 선택";
                case TXT_LANGUAGE_SAVED: return "언어가 저장되었습니다";
                case TXT_HC12_TERMINAL: return "HC12 터미널";
                case TXT_HC12_INFO: return "HC12 정보";
                case TXT_CHECKERS: return "체커";
                case TXT_SNAKE_3D: return "스네이크 3D";
                case TXT_SCREEN:
                default: return "화면";
            }
        case UI_LANG_ENGLISH:
        default:
            switch (id) {
                case TXT_CHAT: return "Chat";
                case TXT_CHAT_PEERS: return "Chat Peers";
                case TXT_MEDIA: return "Media";
                case TXT_INFO: return "Info";
                case TXT_GAMES: return "Games";
                case TXT_CONFIG: return "Config";
                case TXT_AIRPLANE_ON: return "Airplane: ON";
                case TXT_AIRPLANE_OFF: return "Airplane: OFF";
                case TXT_AP_MODE_ON: return "AP Mode: ON";
                case TXT_AP_MODE_OFF: return "AP Mode: OFF";
                case TXT_WIFI_CONFIG: return "WiFi Config";
                case TXT_HC12_CONFIG: return "Radio Config";
                case TXT_STYLE: return "Style";
                case TXT_MQTT_CONFIG: return "MQTT Config";
                case TXT_MQTT_CONTROLS: return "MQTT Controls";
                case TXT_SCREENSHOT: return "Screenshot";
                case TXT_LANGUAGE: return "Language";
                case TXT_OTA_UPDATES: return "OTA Updates";
                case TXT_DEVICE_NAME: return "Device Name";
                case TXT_SAVE: return "Save";
                case TXT_BRIGHTNESS: return "Brightness";
                case TXT_VOLUME: return "Volume";
                case TXT_RGB_LED: return "RGB LED";
                case TXT_SELECT_DISPLAY_LANGUAGE: return "Select display language";
                case TXT_LANGUAGE_SAVED: return "Language saved";
                case TXT_HC12_TERMINAL: return "Radio Terminal";
                case TXT_HC12_INFO: return "Radio Info";
                case TXT_CHECKERS: return "Checkers";
                case TXT_SNAKE_3D: return "Snake 3D";
                case TXT_SCREEN:
                default: return "Screen";
            }
    }
}

static String topBarCenterText()
{
    if (topBarCenterMode == TOP_BAR_CENTER_TIME) {
        if (!internetTimeValid()) return deviceShortNameValue();
        const time_t now = time(nullptr) + (static_cast<long>(topBarTimezoneGmtOffset) * 3600L);
        struct tm tmNow = {};
        gmtime_r(&now, &tmNow);
        char buf[9];
        snprintf(buf, sizeof(buf), "%02d:%02d", tmNow.tm_hour, tmNow.tm_min);
        return String(buf);
    }

    String topName = deviceShortNameValue();
    if (topName.length() > 8) topName.remove(8);
    return topName;
}

static void lvglSetMenuButtonIconMode(lv_obj_t *btn,
                                      const String &plainText,
                                      const char *symbol,
                                      const lv_img_dsc_t *customImg,
                                      lv_coord_t xOffset = 8,
                                      lv_coord_t labelShiftX = 10)
{
    if (!btn || !lv_obj_is_valid(btn)) return;
    lv_obj_t *label = lv_obj_get_child(btn, 0);
    if (!label) return;

    while (lv_obj_get_child_cnt(btn) > 1) {
        lv_obj_t *child = lv_obj_get_child(btn, lv_obj_get_child_cnt(btn) - 1);
        if (!child) break;
        lv_obj_del(child);
    }

    const String labelText = (!menuCustomIconsEnabled || !customImg) ? lvglSymbolText(symbol, plainText) : plainText;
    lv_label_set_text(label, labelText.c_str());
    lv_obj_center(label);

    if (menuCustomIconsEnabled && customImg) {
        lvglAttachMenuButtonImage(btn, customImg, xOffset, labelShiftX);
    }
}

static void lvglRefreshPrimaryMenuButtonIcons()
{
    lvglSetMenuButtonIconMode(lvglHomeChatBtn, tr(TXT_CHAT), LV_SYMBOL_BELL, &img_chat_small_icon);
    lvglSetMenuButtonIconMode(lvglHomeMediaBtn, tr(TXT_MEDIA), LV_SYMBOL_AUDIO, &img_music_small_icon, 8, 12);
    lvglSetMenuButtonIconMode(lvglHomeInfoBtn, tr(TXT_INFO), LV_SYMBOL_WARNING, &img_info_small_icon);
    lvglSetMenuButtonIconMode(lvglHomeGamesBtn, tr(TXT_GAMES), LV_SYMBOL_PLAY, &img_games_small_icon);
    lvglSetMenuButtonIconMode(lvglHomeConfigBtn, tr(TXT_CONFIG), LV_SYMBOL_SETTINGS, &img_config_small_icon);
    lvglSetMenuButtonIconMode(lvglHomePowerBtn, "Power", LV_SYMBOL_POWER, &img_power_small_icon);
    if (menuCustomIconsEnabled) lvglSetButtonImageZoom(lvglHomePowerBtn, 205, 10, 12);

    lvglSetMenuButtonIconMode(lvglAirplaneBtn,
                              airplaneModeEnabled ? tr(TXT_AIRPLANE_ON) : tr(TXT_AIRPLANE_OFF),
                              LV_SYMBOL_CLOSE,
                              &img_airplane_mode_icon);
    lvglSetMenuButtonIconMode(lvglApModeBtn,
                              wifiSessionApMode ? tr(TXT_AP_MODE_ON) : tr(TXT_AP_MODE_OFF),
                              LV_SYMBOL_WIFI,
                              &img_ap_small_icon);
    lvglAirplaneBtnLabel = lvglAirplaneBtn ? lv_obj_get_child(lvglAirplaneBtn, 0) : nullptr;
    lvglApModeBtnLabel = lvglApModeBtn ? lv_obj_get_child(lvglApModeBtn, 0) : nullptr;

    lvglSetMenuButtonIconMode(lvglConfigWifiBtn, tr(TXT_WIFI_CONFIG), LV_SYMBOL_WIFI, &img_wifi_small_icon, 8, 12);
    lvglSetMenuButtonIconMode(lvglConfigHc12Btn, tr(TXT_HC12_CONFIG), LV_SYMBOL_SETTINGS, &img_radio_small_icon, 8, 12);
    lvglSetMenuButtonIconMode(lvglConfigStyleBtn, tr(TXT_STYLE), LV_SYMBOL_SETTINGS, &img_styles_small_icon, 8, 12);
    lvglSetMenuButtonIconMode(lvglConfigMqttBtn, tr(TXT_MQTT_CONFIG), LV_SYMBOL_SETTINGS, &img_mqtt_conf_small_icon, 8, 12);
    lvglSetMenuButtonIconMode(lvglConfigMqttControlsBtn, tr(TXT_MQTT_CONTROLS), LV_SYMBOL_LIST, &img_mqtt_controls_small_icon, 8, 12);
    lvglSetMenuButtonIconMode(lvglConfigScreenshotBtn, tr(TXT_SCREENSHOT), LV_SYMBOL_IMAGE, &img_screenshot_small_icon, 8, 12);
    lvglSetMenuButtonIconMode(lvglConfigLanguageBtn, tr(TXT_LANGUAGE), LV_SYMBOL_EDIT, &img_styles_small_icon, 8, 12);
    lvglSetMenuButtonIconMode(lvglConfigOtaBtn, tr(TXT_OTA_UPDATES), LV_SYMBOL_UPLOAD, &img_ota_small_icon, 8, 12);
    lvglRefreshBatteryTrainButtonIcons();
}

static void lvglApplyMomentaryButtonStyle(lv_obj_t *btn, lv_obj_t *label, lv_color_t bodyCol, bool compact)
{
    if (!btn) return;

    if (lvglBlackButtonThemeActive()) {
        const lv_color_t blackBody = lv_color_hex(UI_BUTTON_BLACK_BODY_HEX);
        const lv_color_t pressedCol = lv_color_hex(UI_BUTTON_BLACK_PRESSED_HEX);
        const lv_color_t flashColor = lv_color_hex(UI_BUTTON_BLACK_FLASH_HEX);
        lv_obj_set_style_bg_color(btn, blackBody, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_color(btn, blackBody, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(btn, pressedCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, pressedCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_grad_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_border_width(btn, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_color(btn, lv_color_hex(UI_BUTTON_BLACK_BORDER_HEX), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_opa(btn, static_cast<lv_opa_t>(84), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(btn, 1, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_border_color(btn, lv_color_hex(UI_BUTTON_BLACK_BORDER_ACTIVE_HEX), LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_shadow_width(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(btn, 0, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_translate_y(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_translate_y(btn, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    } else {
        const lv_color_t flashColor = lv_color_mix(lv_color_white(), bodyCol, UI_BUTTON_CLICK_FLASH_MIX);
        if (uiButtonStyleMode == UI_BUTTON_STYLE_FLAT) {
        const lv_color_t pressedCol = lv_color_mix(lv_color_black(), bodyCol, compact ? 32 : 44);
        lv_obj_set_style_bg_color(btn, bodyCol, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_color(btn, bodyCol, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(btn, pressedCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, pressedCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_grad_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_border_width(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(btn, compact ? 0 : 6, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_ofs_x(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_ofs_y(btn, compact ? 0 : 2, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_opa(btn, compact ? LV_OPA_TRANSP : LV_OPA_20, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(btn, 0, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_translate_y(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_translate_y(btn, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    } else {
        const lv_color_t topCol = lv_color_mix(lv_color_white(), bodyCol, compact ? 70 : 92);
        const lv_color_t bottomCol = lv_color_mix(lv_color_black(), bodyCol, compact ? 80 : 104);
        const lv_color_t pressedTopCol = lv_color_mix(lv_color_black(), bodyCol, compact ? 88 : 116);
        const lv_color_t pressedBottomCol = lv_color_mix(lv_color_white(), bodyCol, compact ? 56 : 76);
        const lv_color_t borderCol = lv_color_mix(lv_color_black(), bodyCol, compact ? 76 : 108);
        lv_obj_set_style_bg_color(btn, topCol, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_color(btn, bottomCol, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_dir(btn, LV_GRAD_DIR_VER, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(btn, pressedTopCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, pressedBottomCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_dir(btn, LV_GRAD_DIR_VER, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_grad_color(btn, lv_color_mix(lv_color_white(), bottomCol, UI_BUTTON_CLICK_FLASH_MIX / 2),
                                       LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, lv_color_mix(lv_color_white(), bottomCol, UI_BUTTON_CLICK_FLASH_MIX / 2),
                                       LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_border_width(btn, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_color(btn, borderCol, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_opa(btn, LV_OPA_50, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(btn, 1, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_border_color(btn, borderCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_shadow_width(btn, compact ? 5 : 12, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_spread(btn, compact ? 0 : 1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_ofs_x(btn, compact ? 1 : 3, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_ofs_y(btn, compact ? 2 : 4, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_color(btn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_opa(btn, compact ? LV_OPA_30 : LV_OPA_40, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(btn, compact ? 1 : 3, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_shadow_ofs_x(btn, 1, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_shadow_ofs_y(btn, 1, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_shadow_opa(btn, compact ? LV_OPA_10 : LV_OPA_20, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_translate_y(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_translate_y(btn, 1, LV_PART_MAIN | LV_STATE_PRESSED);
    }
    }

    if (label) {
        lv_obj_set_style_text_color(label, lv_color_hex(0xF7FBFF), 0);
        lv_obj_set_style_text_font(label, LV_FONT_DEFAULT, 0);
    }
    lv_obj_invalidate(btn);
}

static void lvglRegisterStyledButton(lv_obj_t *btn, lv_color_t baseColor, bool compact)
{
    if (!btn) return;
    if (!lvglEnsureUiRegistryStorage()) return;
    LvglStyledButtonEntry *entry = lvglFindStyledButtonEntry(btn);
    if (!entry) {
        for (size_t i = 0; i < LVGL_STYLED_BUTTON_CAPACITY; ++i) {
            if (!lvglStyledButtons[i].obj || !lv_obj_is_valid(lvglStyledButtons[i].obj)) {
                entry = &lvglStyledButtons[i];
                entry->obj = btn;
                lv_obj_add_event_cb(btn, lvglAirplaneButtonDrawEvent, LV_EVENT_DRAW_MAIN, nullptr);
                lv_obj_add_event_cb(btn, lvglStyledButtonDeleteEvent, LV_EVENT_DELETE, nullptr);
                break;
            }
        }
    }
    if (!entry) return;
    entry->baseColor = baseColor;
    entry->flags = compact ? LVGL_STYLED_BUTTON_FLAG_COMPACT : 0;
    lvglApplyMomentaryButtonStyle(btn, lv_obj_get_child(btn, 0), baseColor, compact);
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
    lv_obj_add_event_cb(btn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(btn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, user);
    lv_obj_t *lbl = lv_label_create(btn);
    if (lbl) {
        lv_label_set_text(lbl, txt);
        lv_obj_center(lbl);
    }
    lvglRegisterStyledButton(btn, color, compactList);
    return btn;
}

static void lvglAttachMenuButtonImage(lv_obj_t *btn, const lv_img_dsc_t *imgSrc, lv_coord_t xOffset = 8, lv_coord_t labelShiftX = 10)
{
    if (!btn || !imgSrc) return;
    lv_obj_t *img = lv_img_create(btn);
    if (!img) return;
    lv_img_set_src(img, imgSrc);
    lv_obj_clear_flag(img, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_align(img, LV_ALIGN_LEFT_MID, xOffset, 0);

    lv_obj_t *label = lv_obj_get_child(btn, 0);
    if (label) lv_obj_align(label, LV_ALIGN_CENTER, labelShiftX, 0);
}

static void lvglSetButtonImageZoom(lv_obj_t *btn, uint16_t zoom, lv_coord_t xOffset, lv_coord_t labelShiftX)
{
    if (!btn || lv_obj_get_child_cnt(btn) < 2) return;
    lv_obj_t *img = lv_obj_get_child(btn, 1);
    if (!img) return;
    lv_img_set_zoom(img, zoom);
    lv_obj_align(img, LV_ALIGN_LEFT_MID, xOffset, 0);

    lv_obj_t *label = lv_obj_get_child(btn, 0);
    if (label) lv_obj_align(label, LV_ALIGN_CENTER, labelShiftX, 0);
}

static void lvglApplyPersistentToggleButtonStyle(lv_obj_t *btn,
                                                 lv_obj_t *label,
                                                 bool enabled,
                                                 lv_color_t offBodyCol,
                                                 lv_color_t onBodyCol,
                                                 bool compact)
{
    if (!btn) return;

    const lv_color_t bodyCol = enabled ? lvglActiveToggleGreen(compact) : offBodyCol;
    const lv_color_t flashColor = lv_color_mix(lv_color_white(), bodyCol, UI_BUTTON_CLICK_FLASH_MIX);
    if (lvglBlackButtonThemeActive()) {
        const lv_color_t blackBody = lv_color_hex(UI_BUTTON_BLACK_BODY_HEX);
        const lv_color_t pressedCol = lv_color_hex(UI_BUTTON_BLACK_PRESSED_HEX);
        const lv_color_t flashCol = lv_color_hex(UI_BUTTON_BLACK_FLASH_HEX);
        const lv_color_t borderCol = lv_color_hex(enabled ? UI_BUTTON_BLACK_BORDER_ACTIVE_HEX : UI_BUTTON_BLACK_BORDER_HEX);
        lv_obj_set_style_bg_color(btn, blackBody, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_color(btn, blackBody, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(btn, pressedCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, pressedCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_color(btn, flashCol, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_grad_color(btn, flashCol, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(btn, flashCol, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, flashCol, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_border_width(btn, enabled ? 2 : 1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_color(btn, borderCol, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_opa(btn, enabled ? static_cast<lv_opa_t>(120) : static_cast<lv_opa_t>(84), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(btn, enabled ? 2 : 1, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_border_color(btn, borderCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_shadow_width(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(btn, 0, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_translate_y(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_translate_y(btn, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    } else if (uiButtonStyleMode == UI_BUTTON_STYLE_FLAT) {
        const lv_color_t pressedCol = lv_color_mix(lv_color_black(), bodyCol, compact ? 40 : 56);
        const lv_color_t borderCol = enabled
                                         ? lv_color_mix(lv_color_white(), bodyCol, compact ? 108 : 132)
                                         : lv_color_mix(lv_color_black(), bodyCol, compact ? 84 : 108);
        lv_obj_set_style_bg_color(btn, bodyCol, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_color(btn, bodyCol, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(btn, pressedCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, pressedCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_grad_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_border_width(btn, enabled ? 2 : 1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_color(btn, borderCol, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_opa(btn, enabled ? LV_OPA_90 : static_cast<lv_opa_t>(72), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(btn, enabled ? 2 : 1, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_border_color(btn, borderCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_shadow_width(btn, enabled ? 0 : (compact ? 0 : 4), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_ofs_x(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_ofs_y(btn, compact ? 0 : 1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_opa(btn, compact ? LV_OPA_TRANSP : static_cast<lv_opa_t>(18), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(btn, 0, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_translate_y(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_translate_y(btn, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    } else {
        const lv_color_t lightCol = lv_color_mix(lv_color_white(), bodyCol, compact ? 56 : 76);
        const lv_color_t darkCol = lv_color_mix(lv_color_black(), bodyCol, compact ? 92 : 118);
        const lv_color_t topCol = enabled ? darkCol : lightCol;
        const lv_color_t bottomCol = enabled ? lightCol : darkCol;
        const lv_color_t pressedTopCol = lv_color_mix(lv_color_black(), topCol, compact ? 64 : 82);
        const lv_color_t pressedBottomCol = lv_color_mix(lv_color_black(), bottomCol, compact ? 52 : 68);
        const lv_color_t borderCol = enabled ? lv_color_mix(lv_color_black(), bodyCol, 104) : lv_color_mix(lv_color_white(), bodyCol, 88);
        const lv_color_t flashBottomCol = lv_color_mix(lv_color_white(), bottomCol, UI_BUTTON_CLICK_FLASH_MIX / 2);

        lv_obj_set_style_bg_color(btn, topCol, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_color(btn, bottomCol, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_dir(btn, LV_GRAD_DIR_VER, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_main_stop(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_stop(btn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

        lv_obj_set_style_bg_color(btn, pressedTopCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, pressedBottomCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_dir(btn, LV_GRAD_DIR_VER, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_main_stop(btn, 0, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_stop(btn, 255, LV_PART_MAIN | LV_STATE_PRESSED);

        lv_obj_set_style_bg_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_grad_color(btn, flashBottomCol, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_grad_dir(btn, LV_GRAD_DIR_VER, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, flashBottomCol, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_dir(btn, LV_GRAD_DIR_VER, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);

        lv_obj_set_style_border_width(btn, enabled ? 3 : 1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_color(btn, borderCol, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_opa(btn, enabled ? static_cast<lv_opa_t>(76) : static_cast<lv_opa_t>(72), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(btn, enabled ? 2 : 1, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_border_color(btn, borderCol, LV_PART_MAIN | LV_STATE_PRESSED);

        lv_obj_set_style_shadow_width(btn, compact ? (enabled ? 1 : 6) : (enabled ? 1 : 12), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_spread(btn, enabled ? 0 : 1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_ofs_x(btn, compact ? (enabled ? 0 : 2) : (enabled ? 0 : 4), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_ofs_y(btn, compact ? (enabled ? 0 : 2) : (enabled ? 0 : 4), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_color(btn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_opa(btn,
                                    compact ? (enabled ? static_cast<lv_opa_t>(6) : static_cast<lv_opa_t>(28))
                                            : (enabled ? static_cast<lv_opa_t>(8) : LV_OPA_40),
                                    LV_PART_MAIN | LV_STATE_DEFAULT);

        lv_obj_set_style_shadow_width(btn, compact ? 1 : 2, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_shadow_ofs_x(btn, enabled ? 0 : 1, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_shadow_ofs_y(btn, enabled ? 0 : 1, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_shadow_opa(btn, compact ? static_cast<lv_opa_t>(6) : static_cast<lv_opa_t>(10), LV_PART_MAIN | LV_STATE_PRESSED);

        lv_obj_set_style_translate_y(btn, enabled ? 2 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_translate_y(btn, enabled ? 3 : 1, LV_PART_MAIN | LV_STATE_PRESSED);
    }

    if (label) {
        lv_obj_set_style_text_color(label, lv_color_hex(0xF7FBFF), 0);
        lv_obj_set_style_text_font(label, LV_FONT_DEFAULT, 0);
    }

    lv_obj_invalidate(btn);
}

static void lvglAirplaneButtonDrawEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_DRAW_MAIN) return;
    if (uiButtonStyleMode != UI_BUTTON_STYLE_3D) return;
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *drawCtx = lv_event_get_draw_ctx(e);
    if (!obj || !drawCtx) return;

    lv_color_t bodyCol = lv_color_hex(0x4E5D6C);
    bool sunken = lv_obj_has_state(obj, LV_STATE_PRESSED);
    bool compact = false;
    if (obj == lvglAirplaneBtn) {
        bodyCol = airplaneModeEnabled ? lvglActiveToggleGreen(false) : lv_color_hex(0x98632E);
        sunken = airplaneModeEnabled || sunken;
    } else if (obj == lvglApModeBtn) {
        bodyCol = wifiSessionApMode ? lvglActiveToggleGreen(false) : lv_color_hex(0xA66A2A);
        sunken = wifiSessionApMode || sunken;
    } else if (obj == lvglChatDiscoveryBtn) {
        bodyCol = p2pDiscoveryEnabled ? lvglActiveToggleGreen(true) : lv_color_hex(0x4E5D6C);
        sunken = p2pDiscoveryEnabled || sunken;
        compact = true;
    } else if (obj == lvglWifiWebServerBtn) {
        bodyCol = webServerEnabled ? lvglActiveToggleGreen(true) : lv_color_hex(0x6B3A3A);
        sunken = webServerEnabled || sunken;
        compact = true;
    } else if (obj == lvglStyleButtonFlatBtn) {
        bodyCol = (uiButtonStyleMode == UI_BUTTON_STYLE_FLAT) ? lvglActiveToggleGreen(true) : lv_color_hex(uiButtonStyleFlatSelectorColor);
        sunken = (uiButtonStyleMode == UI_BUTTON_STYLE_FLAT) || sunken;
        compact = true;
    } else if (obj == lvglStyleButton3dBtn) {
        bodyCol = (uiButtonStyleMode == UI_BUTTON_STYLE_3D) ? lvglActiveToggleGreen(true) : lv_color_hex(uiButtonStyle3dSelectorColor);
        sunken = (uiButtonStyleMode == UI_BUTTON_STYLE_3D) || sunken;
        compact = true;
    } else {
        const LvglStyledButtonEntry *entry = lvglFindStyledButtonEntryConst(obj);
        if (!entry) return;
        bodyCol = entry->baseColor;
        compact = (entry->flags & LVGL_STYLED_BUTTON_FLAG_COMPACT) != 0;
    }

    lv_area_t a;
    lv_obj_get_coords(obj, &a);
    const int inset = compact ? 3 : 4;
    const int x1 = static_cast<int>(a.x1) + inset;
    const int y1 = static_cast<int>(a.y1) + inset;
    const int x2 = static_cast<int>(a.x2) - inset;
    const int y2 = static_cast<int>(a.y2) - inset;
    if (x2 <= x1 || y2 <= y1) return;

    const lv_color_t lightCol = lv_color_mix(lv_color_white(), bodyCol, compact ? 56 : 76);
    const lv_color_t darkCol = lv_color_mix(lv_color_black(), bodyCol, compact ? 92 : 118);
    const lv_opa_t strongOpa = sunken ? (compact ? static_cast<lv_opa_t>(56) : static_cast<lv_opa_t>(72))
                                      : (compact ? static_cast<lv_opa_t>(72) : LV_OPA_90);
    const lv_opa_t softOpa = sunken ? (compact ? static_cast<lv_opa_t>(34) : static_cast<lv_opa_t>(46))
                                    : (compact ? static_cast<lv_opa_t>(44) : LV_OPA_60);

    const lv_color_t topCol = sunken ? darkCol : lightCol;
    const lv_color_t leftCol = sunken ? darkCol : lightCol;
    const lv_color_t bottomCol = sunken ? lightCol : darkCol;
    const lv_color_t rightCol = sunken ? lightCol : darkCol;

    lvglDrawLineSeg(drawCtx, x1 + 3, y1, x2 - 3, y1, topCol, 1, strongOpa);
    lvglDrawLineSeg(drawCtx, x1, y1 + 3, x1, y2 - 3, leftCol, 1, strongOpa);
    lvglDrawLineSeg(drawCtx, x1 + 3, y2, x2 - 3, y2, bottomCol, 1, strongOpa);
    lvglDrawLineSeg(drawCtx, x2, y1 + 3, x2, y2 - 3, rightCol, 1, strongOpa);

    lvglDrawLineSeg(drawCtx, x1 + 4, y1 + 1, x2 - 4, y1 + 1, topCol, 1, softOpa);
    lvglDrawLineSeg(drawCtx, x1 + 1, y1 + 4, x1 + 1, y2 - 4, leftCol, 1, softOpa);
    lvglDrawLineSeg(drawCtx, x1 + 4, y2 - 1, x2 - 4, y2 - 1, bottomCol, 1, softOpa);
    lvglDrawLineSeg(drawCtx, x2 - 1, y1 + 4, x2 - 1, y2 - 4, rightCol, 1, softOpa);
}

void lvglApplyAirplaneButtonStyle()
{
    lvglApplyPersistentToggleButtonStyle(
        lvglAirplaneBtn, lvglAirplaneBtnLabel, airplaneModeEnabled, lv_color_hex(0x98632E), lv_color_hex(0x3B78B6), false);
}

void lvglApplyApModeButtonStyle()
{
    lvglApplyPersistentToggleButtonStyle(
        lvglApModeBtn, lvglApModeBtnLabel, wifiSessionApMode, lv_color_hex(0xA66A2A), lv_color_hex(0xA66A2A), false);
}

void lvglApplyChatDiscoveryButtonStyle()
{
    lv_obj_t *label = lvglChatDiscoveryBtn ? lv_obj_get_child(lvglChatDiscoveryBtn, 0) : nullptr;
    lvglApplyPersistentToggleButtonStyle(
        lvglChatDiscoveryBtn, label, p2pDiscoveryEnabled, lv_color_hex(0x4E5D6C), lv_color_hex(0x3A7A3A), true);
}

void lvglApplyWifiWebServerButtonStyle()
{
    lv_obj_t *label = lvglWifiWebServerBtn ? lv_obj_get_child(lvglWifiWebServerBtn, 0) : nullptr;
    lvglApplyPersistentToggleButtonStyle(
        lvglWifiWebServerBtn, label, webServerEnabled, lv_color_hex(0x6B3A3A), lv_color_hex(0x357A38), true);
}

void lvglRefreshAllButtonStyles()
{
    if (!lvglStyledButtons) return;
    for (size_t i = 0; i < LVGL_STYLED_BUTTON_CAPACITY; ++i) {
        LvglStyledButtonEntry &entry = lvglStyledButtons[i];
        if (!entry.obj || !lv_obj_is_valid(entry.obj)) {
            entry.obj = nullptr;
            continue;
        }
        const bool compact = (entry.flags & LVGL_STYLED_BUTTON_FLAG_COMPACT) != 0;
        lvglApplyMomentaryButtonStyle(entry.obj, lv_obj_get_child(entry.obj, 0), entry.baseColor, compact);
    }
    lvglApplyAirplaneButtonStyle();
    lvglApplyApModeButtonStyle();
    lvglApplyChatDiscoveryButtonStyle();
    lvglApplyWifiWebServerButtonStyle();
    if (lvglStyleButtonFlatBtn) {
        lvglApplyPersistentToggleButtonStyle(lvglStyleButtonFlatBtn,
                                             lvglStyleButtonFlatBtnLabel,
                                             uiButtonStyleMode == UI_BUTTON_STYLE_FLAT,
                                             lv_color_hex(uiButtonStyleFlatSelectorColor),
                                             lv_color_hex(uiButtonStyleFlatSelectorColor),
                                             true);
    }
    if (lvglStyleButton3dBtn) {
        lvglApplyPersistentToggleButtonStyle(lvglStyleButton3dBtn,
                                             lvglStyleButton3dBtnLabel,
                                             uiButtonStyleMode == UI_BUTTON_STYLE_3D,
                                             lv_color_hex(uiButtonStyle3dSelectorColor),
                                             lv_color_hex(uiButtonStyle3dSelectorColor),
                                             true);
    }
    if (lvglStyleButtonBlackBtn) {
        lvglApplyPersistentToggleButtonStyle(lvglStyleButtonBlackBtn,
                                             lvglStyleButtonBlackBtnLabel,
                                             uiButtonStyleMode == UI_BUTTON_STYLE_BLACK,
                                             lv_color_hex(UI_BUTTON_BLACK_BODY_HEX),
                                             lv_color_hex(UI_BUTTON_BLACK_BODY_HEX),
                                             true);
    }
    if (lvglStyleTimezoneDd) {
        const uint16_t selected = static_cast<uint16_t>(topBarTimezoneGmtOffset - TOP_BAR_GMT_MIN);
        if (lv_dropdown_get_selected(lvglStyleTimezoneDd) != selected) {
            lv_dropdown_set_selected(lvglStyleTimezoneDd, selected);
        }
    }
    if (lvglStyleTopCenterNameBtn) {
        lvglApplyPersistentToggleButtonStyle(lvglStyleTopCenterNameBtn,
                                             lvglStyleTopCenterNameBtnLabel,
                                             topBarCenterMode == TOP_BAR_CENTER_NAME,
                                             lv_color_hex(0x3F4A57),
                                             lv_color_hex(0x3F4A57),
                                             true);
    }
    if (lvglStyleTopCenterTimeBtn) {
        lvglApplyPersistentToggleButtonStyle(lvglStyleTopCenterTimeBtn,
                                             lvglStyleTopCenterTimeBtnLabel,
                                             topBarCenterMode == TOP_BAR_CENTER_TIME,
                                             lv_color_hex(0x3F4A57),
                                             lv_color_hex(0x3F4A57),
                                             true);
    }
}

void lvglSetButtonStyleMode(UiButtonStyleMode mode, bool persist)
{
    if (mode != UI_BUTTON_STYLE_FLAT && mode != UI_BUTTON_STYLE_3D && mode != UI_BUTTON_STYLE_BLACK) {
        mode = UI_BUTTON_STYLE_3D;
    }
    uiButtonStyleMode = mode;
    if (persist) {
        uiPrefs.begin("ui", false);
        uiPrefs.putUChar("btn_style", static_cast<uint8_t>(uiButtonStyleMode));
        uiPrefs.end();
    }
    lvglRefreshAllButtonStyles();
    lvglRefreshStyleUi();
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

static bool checkersRawTextStartsWith(const String &text, const char *prefix)
{
    return prefix && text.startsWith(prefix);
}

static bool checkersRawTextHasControlPrefix(const String &text)
{
    for (int i = 0; i < CHECKERS_CONTROL_MSG_MARKERS; ++i) {
        if (checkersRawTextStartsWith(text, CHECKERS_CONTROL_PREFIXES[i])) return true;
    }
    return false;
}

static String checkersVisibleBodyFromRawText(const String &text)
{
    if (!checkersRawTextHasControlPrefix(text)) return text;
    const int nl = text.indexOf('\n');
    if (nl < 0 || nl >= (text.length() - 1)) return "";
    return text.substring(nl + 1);
}

static bool checkersParseInviteText(const String &text, String &sessionId, CheckersVariant *variantOut = nullptr)
{
    if (!checkersRawTextStartsWith(text, "@CHK_INVITE|")) return false;
    const int lineEnd = text.indexOf('\n');
    const String marker = (lineEnd >= 0) ? text.substring(0, lineEnd) : text;
    const int firstSep = marker.indexOf('|');
    const int secondSep = (firstSep >= 0) ? marker.indexOf('|', firstSep + 1) : -1;
    if (firstSep < 0 || firstSep >= (marker.length() - 1)) return false;
    sessionId = (secondSep > firstSep) ? marker.substring(firstSep + 1, secondSep) : marker.substring(firstSep + 1);
    sessionId.trim();
    if (variantOut) {
        int rawVariant = static_cast<int>(CHECKERS_VARIANT_AMERICAN);
        if (secondSep > firstSep && secondSep < (marker.length() - 1)) rawVariant = marker.substring(secondSep + 1).toInt();
        if (rawVariant < static_cast<int>(CHECKERS_VARIANT_AMERICAN) || rawVariant > static_cast<int>(CHECKERS_VARIANT_CANADIAN)) {
            rawVariant = static_cast<int>(CHECKERS_VARIANT_AMERICAN);
        }
        *variantOut = static_cast<CheckersVariant>(rawVariant);
    }
    return !sessionId.isEmpty();
}

static bool checkersParseAcceptText(const String &text, String &sessionId)
{
    if (!checkersRawTextStartsWith(text, "@CHK_ACCEPT|")) return false;
    sessionId = text.substring(String("@CHK_ACCEPT|").length());
    sessionId.trim();
    return !sessionId.isEmpty();
}

static bool checkersParseDeclineText(const String &text, String &sessionId)
{
    if (!checkersRawTextStartsWith(text, "@CHK_DECLINE|")) return false;
    sessionId = text.substring(String("@CHK_DECLINE|").length());
    sessionId.trim();
    return !sessionId.isEmpty();
}

static bool checkersParseMoveText(const String &text, String &sessionId, CheckersMove &move)
{
    if (!checkersRawTextStartsWith(text, "@CHK_MOVE|")) return false;
    int cursor = String("@CHK_MOVE|").length();
    int nextSep = text.indexOf('|', cursor);
    if (nextSep <= cursor) return false;
    sessionId = text.substring(cursor, nextSep);
    sessionId.trim();
    if (sessionId.isEmpty()) return false;

    int values[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; ++i) {
        cursor = nextSep + 1;
        nextSep = (i == 3) ? -1 : text.indexOf('|', cursor);
        const String token = (nextSep >= 0) ? text.substring(cursor, nextSep) : text.substring(cursor);
        if (token.isEmpty()) return false;
        values[i] = token.toInt();
    }

    move.fromX = static_cast<int8_t>(values[0]);
    move.fromY = static_cast<int8_t>(values[1]);
    move.toX = static_cast<int8_t>(values[2]);
    move.toY = static_cast<int8_t>(values[3]);
    move.capture = false;
    move.captureX = -1;
    move.captureY = -1;
    return true;
}

static String checkersBuildInviteText(const String &sessionId, const String &name)
{
    return "@CHK_INVITE|" + sessionId + "|" + String(static_cast<int>(checkersVariant)) +
           "\nCheckers invite from " + name + "\n" + checkersVariantName(checkersVariant) + "\nTap Play to join.";
}

static String checkersBuildAcceptText(const String &sessionId)
{
    return "@CHK_ACCEPT|" + sessionId;
}

static String checkersBuildDeclineText(const String &sessionId)
{
    return "@CHK_DECLINE|" + sessionId;
}

static String checkersBuildMoveText(const String &sessionId, const CheckersMove &move)
{
    return "@CHK_MOVE|" + sessionId + "|" +
           String(move.fromX) + "|" +
           String(move.fromY) + "|" +
           String(move.toX) + "|" +
           String(move.toY);
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
    if (!messageId.isEmpty() && !outgoing) {
        if (chatConversationHasMessageId(peerKey, messageId)) return;
        if (chatHasLoggedMessageId(peerKey, messageId)) return;
    }
    const bool stored = chatAppendMessageToSd(peerKey, transport, author, text, outgoing, nowMs, messageId);
    const bool conversationOpen = (uiScreen == UI_CHAT) && (currentChatPeerKey == peerKey);
    if (conversationOpen && (stored || !messageId.isEmpty())) {
        chatSetPeerUnread(peerKey, false);
        chatAddMessage(author, text, outgoing, transport, messageId);
    } else if (!outgoing) {
        chatSetPeerUnread(peerKey, true);
        chatQueueIncomingMessageBeep();
        chatQueueIncomingMessageVibration();
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
    const String messageId = chatGenerateMessageId();
    chatQueueOutgoingMessage(peerKey, deviceShortNameValue(), text, messageId);
    const bool sentLan = p2pSendChatMessageWithId(peerKey, text, messageId);
    const bool sentGlobal = mqttPublishChatMessageWithId(peerKey, text, messageId);
    const bool sentRadio = hc12SendChatMessageWithId(peerKey, text, messageId);
    if (storeVisible) {
        const ChatTransport storedTransport = sentGlobal ? CHAT_TRANSPORT_MQTT
                                                         : (sentLan ? CHAT_TRANSPORT_WIFI
                                                                    : (sentRadio ? radioSelectedChatTransport() : CHAT_TRANSPORT_WIFI));
        chatStoreMessage(peerKey, "Me", text, true, storedTransport, messageId);
        if (lvglReady) {
            if (uiScreen == UI_CHAT) lvglRefreshChatUi();
            lvglRefreshChatContactsUi();
        }
    }
    return sentLan || sentGlobal || sentRadio || storeVisible;
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

static bool chatAckOutgoingMessage(const String &peerKey, const String &messageId)
{
    const int idx = chatFindPendingIndex(peerKey, messageId);
    if (idx < 0) return false;
    for (int i = idx + 1; i < chatPendingCount; ++i) chatPendingMessages[i - 1] = chatPendingMessages[i];
    chatPendingCount--;
    chatSavePendingOutbox();
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
    for (int i = 0; i < chatPendingCount; ++i) {
        PendingChatMessage &msg = chatPendingMessages[i];
        if (msg.peerKey.isEmpty() || msg.messageId.isEmpty() || msg.text.isEmpty()) continue;
        if (msg.lastAttemptMs != 0 && static_cast<unsigned long>(now - msg.lastAttemptMs) < CHAT_RETRY_INTERVAL_MS) continue;
        bool attempted = false;
        if (p2pSendChatMessageWithId(msg.peerKey, msg.text, msg.messageId)) attempted = true;
        if (mqttPublishChatMessageWithId(msg.peerKey, msg.text, msg.messageId)) attempted = true;
        if (hc12SendChatMessageWithId(msg.peerKey, msg.text, msg.messageId)) attempted = true;
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

    if (airplaneModeEnabled) {
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
    if (!p2pPeers) return -1;
    for (int i = 0; i < p2pPeerCount; ++i) {
        if (p2pPeers[i].pubKeyHex.equalsIgnoreCase(pubKeyHex)) return i;
    }
    return -1;
}

static int p2pFindDiscoveredByPubKeyHex(const String &pubKeyHex)
{
    if (!p2pDiscoveredPeers) return -1;
    for (int i = 0; i < p2pDiscoveredCount; ++i) {
        if (p2pDiscoveredPeers[i].pubKeyHex.equalsIgnoreCase(pubKeyHex)) return i;
    }
    return -1;
}

static void p2pTouchPeerSeen(int idx, const IPAddress &ip, uint16_t port)
{
    if (!p2pPeers || idx < 0 || idx >= p2pPeerCount) return;
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
    if (!p2pEnsureDiscoveredStorage()) return;
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
    if (!p2pPeers) return;
    p2pPrefs.begin("p2p", false);
    p2pPrefs.putString("pub", p2pPublicKeyHex());
    p2pPrefs.putBytes("sec", p2pSecretKey, sizeof(p2pSecretKey));
    p2pPrefs.putBool("discover_en", p2pDiscoveryEnabled);
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
    p2pDiscoveredCount = 0;
    p2pPrefs.begin("p2p", false);
    p2pDiscoveryEnabled = p2pPrefs.getBool("discover_en", true);

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
    if (p2pPeerCount > 0 && !p2pEnsurePeerStorage()) {
        p2pPeerCount = 0;
        p2pPrefs.end();
        return;
    }
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
void lvglRefreshInfoPanel(bool refreshIndicators = true);
void lvglMediaPlayStopEvent(lv_event_t *e);
void lvglMediaPrevTrackEvent(lv_event_t *e);
void lvglMediaNextTrackEvent(lv_event_t *e);
void lvglMediaVolumeEvent(lv_event_t *e);
void lvglMediaVolumeStepEvent(lv_event_t *e);
void lvglHomeNavEvent(lv_event_t *e);
void lvglApModeEvent(lv_event_t *e);
void lvglWifiRescanEvent(lv_event_t *e);
void lvglWifiDisconnectEvent(lv_event_t *e);
void lvglWifiForgetEvent(lv_event_t *e);
void lvglWifiApSaveEvent(lv_event_t *e);
void lvglWifiWebServerToggleEvent(lv_event_t *e);
void lvglMediaRefreshEvent(lv_event_t *e);
static void mediaEnsureStorageReadyForUi();
void lvglOpenSnakeEvent(lv_event_t *e);
void lvglOpenTetrisEvent(lv_event_t *e);
void lvglOpenCheckersEvent(lv_event_t *e);
#if defined(BOARD_ESP32S3_3248S035_N16R8)
void lvglOpenSnake3dEvent(lv_event_t *e);
#endif
void lvglOpenMqttCfgEvent(lv_event_t *e);
void lvglOpenMqttCtrlEvent(lv_event_t *e);
void lvglOpenChatPeersEvent(lv_event_t *e);
void lvglChatDiscoveryToggleEvent(lv_event_t *e);
void lvglOpenStyleScreenEvent(lv_event_t *e);
void lvglOpenLanguageScreenEvent(lv_event_t *e);
void lvglStyleScreensaverToggleEvent(lv_event_t *e);
void lvglStyleMenuIconsToggleEvent(lv_event_t *e);
void lvglStyleButtonFlatEvent(lv_event_t *e);
void lvglStyleButton3dEvent(lv_event_t *e);
void lvglStyleButtonBlackEvent(lv_event_t *e);
void lvglStyleTimezoneEvent(lv_event_t *e);
void lvglLanguageDropdownEvent(lv_event_t *e);
void lvglVibrationDropdownEvent(lv_event_t *e);
void lvglMessageToneDropdownEvent(lv_event_t *e);
void lvglSoundPopupBackdropEvent(lv_event_t *e);
void lvglSoundPopupVolumeEvent(lv_event_t *e);
void lvglSoundPopupVibrationEvent(lv_event_t *e);
void lvglSoundPopupDisableVibrationEvent(lv_event_t *e);
void lvglRadioModuleDropdownEvent(lv_event_t *e);
void lvglHc12PrevExtraEvent(lv_event_t *e);
void lvglHc12NextExtraEvent(lv_event_t *e);
void lvglRadioModeWarningEvent(lv_event_t *e);
void lvglStyleTopCenterNameEvent(lv_event_t *e);
void lvglStyleTopCenterTimeEvent(lv_event_t *e);
void lvglStyleTimeoutEvent(lv_event_t *e);
void lvglStylePowerOffEvent(lv_event_t *e);
void lvglRefreshStyleUi();
void lvglRefreshLanguageUi();
void lvglRefreshLocalizedUi();
void lvglGestureBlockEvent(lv_event_t *e);
void screensaverSetActive(bool active);
void screensaverService();
void lvglAirplaneToggleEvent(lv_event_t *e);
void lvglChatAirplanePromptEvent(lv_event_t *e);
void lvglShowChatAirplanePrompt();
bool lvglChatPromptIfAirplaneBlocked();
void lvglPowerButtonEvent(lv_event_t *e);
void lvglPowerConfirmEvent(lv_event_t *e);
void lvglScreenshotEvent(lv_event_t *e);
void lvglStatusPush(const String &line);
void lvglBrightnessEvent(lv_event_t *e);
void lvglConfigVolumeEvent(lv_event_t *e);
void lvglRgbLedEvent(lv_event_t *e);
void lvglTextAreaFocusEvent(lv_event_t *e);
static void hc12InitIfNeeded();
static void hc12AppendTerminal(const char *text);
static void hc12SendLine(const String &line);
static void hc12Service();
static void hc12RefreshInfoSnapshot();
static bool hc12ReadConfigSelection();
static bool hc12ApplyChannel(int channel);
static bool hc12ApplyBaudIndex(int index);
static bool hc12ApplyModeIndex(int index);
static bool hc12ApplyPowerLevel(int level);
static void loadPersistedRadioSettings();
static void savePersistedRadioSettings();
static bool hc12FactoryReset();
static String hc12CompactResponse(String raw);
static String hc12FieldValue(const String &raw, const char *prefix);
static String &hc12LogBuffer();
static lv_obj_t *hc12WrapObj();
static lv_obj_t *hc12SetBtnObj();
static lv_obj_t *hc12TerminalObj();
static lv_obj_t *hc12CmdTaObj();
static bool hc12SetIsAsserted();
static void screenshotService(bool touchDown);
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
static bool lvglCanBuildScreen(UiScreen screen);
static void lvglWarmupScreensService(bool uiPriorityActive);
void lvglOtaPopupEvent(lv_event_t *e);
void lvglOtaAvailablePromptEvent(lv_event_t *e);
void lvglRefreshSnakeBoard();
void lvglRefreshTetrisBoard();
void lvglRefreshCheckersBoard();
#if defined(BOARD_ESP32S3_3248S035_N16R8)
void lvglRefreshSnake3dBoard();
#endif
void lvglSnakeBoardDrawEvent(lv_event_t *e);
void lvglSnakeRestartEvent(lv_event_t *e);
void lvglSnakeDirEvent(lv_event_t *e);
void lvglSnakePauseEvent(lv_event_t *e);
void lvglTetrisBoardDrawEvent(lv_event_t *e);
void lvglTetrisRestartEvent(lv_event_t *e);
void lvglTetrisPauseEvent(lv_event_t *e);
void lvglTetrisMoveLeftEvent(lv_event_t *e);
void lvglTetrisMoveRightEvent(lv_event_t *e);
void lvglTetrisRotateEvent(lv_event_t *e);
void lvglTetrisDropEvent(lv_event_t *e);
void lvglCheckersBoardDrawEvent(lv_event_t *e);
void lvglCheckersBoardEvent(lv_event_t *e);
void lvglCheckersEsp32Event(lv_event_t *e);
void lvglCheckersTagEvent(lv_event_t *e);
void lvglCheckersBackEvent(lv_event_t *e);
void lvglCheckersReplayEvent(lv_event_t *e);
void lvglCheckersInvitePlayEvent(lv_event_t *e);
void lvglCheckersPeerSelectEvent(lv_event_t *e);
void lvglCheckersVariantSelectEvent(lv_event_t *e);
void lvglChatPeerActionEvent(lv_event_t *e);
void checkersTick();
#if defined(BOARD_ESP32S3_3248S035_N16R8)
void lvglSnake3dBoardDrawEvent(lv_event_t *e);
void lvglSnake3dRestartEvent(lv_event_t *e);
void lvglSnake3dDirEvent(lv_event_t *e);
void lvglSnake3dPauseEvent(lv_event_t *e);
void snake3dTick();
#else
inline void snake3dTick() {}
#endif
static int checkersBoardSize();
static int checkersStartRows();
static bool checkersMenCaptureBackward();
static bool checkersFlyingKings();
static bool checkersPromotionContinuesCapture();

inline void lvglLoadScreen(lv_obj_t *target, lv_scr_load_anim_t anim)
{
    if (!target) return;
    lvglResetSwipeBackVisualState(true);
    lvglResetGestureTracking();
    lvglGestureBlocked = false;
    lvglReorderOwnsHorizontalGesture = false;
    if (lvglTouchIndev) lv_indev_reset(lvglTouchIndev, nullptr);
    lv_obj_update_layout(target);
    lv_obj_invalidate(target);
    if (anim == LV_SCR_LOAD_ANIM_NONE) {
        lv_scr_load(target);
        lv_refr_now(nullptr);
    } else {
        lv_scr_load_anim(target, anim, UI_ANIM_MS, 0, false);
    }
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
        case UI_CONFIG_BATTERY:
        case UI_CONFIG_STYLE:
        case UI_CONFIG_LANGUAGE:
        case UI_CONFIG_OTA:
        case UI_CONFIG_HC12:
        case UI_CONFIG_HC12_TERMINAL:
        case UI_CONFIG_HC12_INFO:
        case UI_CONFIG_MQTT_CONFIG:
        case UI_CONFIG_MQTT_CONTROLS:
        case UI_GAME_SNAKE:
        case UI_GAME_TETRIS:
        case UI_GAME_CHECKERS:
        case UI_GAME_SNAKE3D:
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
        case UI_CONFIG_BATTERY: return lvglScrBatteryTrain;
        case UI_CONFIG_STYLE: return lvglScrStyle;
        case UI_CONFIG_LANGUAGE: return lvglScrLanguage;
        case UI_CONFIG_OTA: return lvglScrOta;
        case UI_CONFIG_HC12: return lvglScrHc12;
        case UI_CONFIG_HC12_TERMINAL: return lvglScrHc12Terminal;
        case UI_CONFIG_HC12_INFO: return lvglScrHc12Info;
        case UI_SCREENSAVER: return lvglScrScreensaver;
        case UI_CONFIG_MQTT_CONFIG: return lvglScrMqttCfg;
        case UI_CONFIG_MQTT_CONTROLS: return lvglScrMqttCtrl;
        case UI_GAME_SNAKE: return lvglScrSnake;
        case UI_GAME_TETRIS: return lvglScrTetris;
        case UI_GAME_CHECKERS: return lvglScrCheckers;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
        case UI_GAME_SNAKE3D: return lvglScrSnake3d;
#endif
        default: return nullptr;
    }
}

void lvglEnsureScreenBuilt(UiScreen screen)
{
    if (lvglScreenForUi(screen)) return;
    if (!lvglCanBuildScreen(screen)) return;

    auto makeSmallBtn = [](lv_obj_t *parent, const char *txt, int w, int h, lv_color_t col, lv_event_cb_t cb, void *ud = nullptr) -> lv_obj_t * {
        if (!parent) return nullptr;
        lv_obj_t *b = lv_btn_create(parent);
        if (!b) return nullptr;
        lv_obj_set_size(b, w, h);
        lv_obj_set_style_radius(b, 8, 0);
        lv_obj_set_style_border_width(b, 0, 0);
        lv_obj_add_event_cb(b, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(b, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
        lv_obj_t *l = lv_label_create(b);
        if (l) {
            lv_label_set_text(l, txt);
            lv_obj_center(l);
        }
        lvglRegisterStyledButton(b, col, true);
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
            lvglHomeChatBtn = lvglCreateMenuButton(homeWrap, tr(TXT_CHAT), lv_color_hex(0x7A4F2F), lvglHomeNavEvent, reinterpret_cast<void *>(static_cast<intptr_t>(UI_CHAT)));
            lvglHomeMediaBtn = lvglCreateMenuButton(homeWrap, tr(TXT_MEDIA), lv_color_hex(0x376B93), lvglHomeNavEvent, reinterpret_cast<void *>(static_cast<intptr_t>(UI_MEDIA)));
            lvglHomeInfoBtn = lvglCreateMenuButton(homeWrap, tr(TXT_INFO), lv_color_hex(0x7750A0), lvglHomeNavEvent, reinterpret_cast<void *>(static_cast<intptr_t>(UI_INFO)));
            lvglHomeGamesBtn = lvglCreateMenuButton(homeWrap, tr(TXT_GAMES), lv_color_hex(0x2B7D7D), lvglHomeNavEvent, reinterpret_cast<void *>(static_cast<intptr_t>(UI_GAMES)));
            lvglHomeConfigBtn = lvglCreateMenuButton(homeWrap, tr(TXT_CONFIG), lv_color_hex(0x925A73), lvglHomeNavEvent, reinterpret_cast<void *>(static_cast<intptr_t>(UI_CONFIG)));
#if defined(BOARD_ESP32S3_3248S035_N16R8)
            lvglHomePowerBtn = lvglCreateMenuButton(homeWrap, "Power", lv_color_hex(0xA33F3F), lvglPowerButtonEvent, nullptr);
#endif
            lvglAirplaneBtn = lvglCreateMenuButton(homeWrap, tr(TXT_AIRPLANE_OFF), lv_color_hex(0x8A5A25), lvglAirplaneToggleEvent, nullptr);
            if (lvglAirplaneBtn) {
                lvglAirplaneBtnLabel = lv_obj_get_child(lvglAirplaneBtn, 0);
                lvglApplyAirplaneButtonStyle();
            }
            lvglApModeBtn = lvglCreateMenuButton(homeWrap, tr(TXT_AP_MODE_OFF), lv_color_hex(0xA66A2A), lvglApModeEvent, nullptr);
            if (lvglApModeBtn) {
                lvglApModeBtnLabel = lv_obj_get_child(lvglApModeBtn, 0);
                lvglApplyApModeButtonStyle();
            }
            lvglRefreshPrimaryMenuButtonIcons();
            lvglRegisterReorderableItem(lvglHomeChatBtn, "ord_home", "chat");
            lvglRegisterReorderableItem(lvglHomeMediaBtn, "ord_home", "media");
            lvglRegisterReorderableItem(lvglHomeInfoBtn, "ord_home", "info");
            lvglRegisterReorderableItem(lvglHomeGamesBtn, "ord_home", "games");
            lvglRegisterReorderableItem(lvglHomeConfigBtn, "ord_home", "config");
#if defined(BOARD_ESP32S3_3248S035_N16R8)
            lvglRegisterReorderableItem(lvglHomePowerBtn, "ord_home", "power");
#endif
            lvglRegisterReorderableItem(lvglAirplaneBtn, "ord_home", "air");
            lvglRegisterReorderableItem(lvglApModeBtn, "ord_home", "ap");
            lvglApplySavedOrder(homeWrap, "ord_home");
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
            lvglChatPeersBtn = makeSmallBtn(chatOps, lvglSymbolText(LV_SYMBOL_LIST, "Peers").c_str(), 82, 28, lv_color_hex(0x2F6D86), lvglOpenChatPeersEvent);

            lvglChatContactLabel = lv_label_create(chatOps);
            lv_obj_set_width(lvglChatContactLabel, lv_pct(100));
            lv_obj_set_flex_grow(lvglChatContactLabel, 1);
            lv_obj_set_style_text_align(lvglChatContactLabel, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_set_style_text_color(lvglChatContactLabel, lv_color_hex(0xC8D3DD), 0);
            lv_label_set_long_mode(lvglChatContactLabel, LV_LABEL_LONG_DOT);

            lvglChatMenuBtn = makeSmallBtn(chatOps, LV_SYMBOL_LIST, 34, 28, lv_color_hex(0x2F6D86), lvglToggleChatMenuEvent, nullptr);
            lvglChatDiscoveryBtn = makeSmallBtn(chatOps,
                                                lvglSymbolText(LV_SYMBOL_WIFI, "Discovery").c_str(),
                                                102,
                                                28,
                                                p2pDiscoveryEnabled ? lv_color_hex(0x3A7A3A) : lv_color_hex(0x4E5D6C),
                                                lvglChatDiscoveryToggleEvent);
            if (lvglChatDiscoveryBtn) {
                lvglApplyChatDiscoveryButtonStyle();
            }

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
            lv_obj_t *chatClearBtn = makeSmallBtn(lvglChatMenuPanel, lvglSymbolText(LV_SYMBOL_TRASH, "Clear").c_str(), lv_pct(100), 32, lv_color_hex(0x8A3A3A), lvglDeleteChatConversationEvent, nullptr);
            if (chatClearBtn) lv_obj_set_width(chatClearBtn, lv_pct(100));
            lv_obj_t *chatClearAllBtn = makeSmallBtn(lvglChatMenuPanel, lvglSymbolText(LV_SYMBOL_TRASH, "Clear for All").c_str(), lv_pct(100), 32, lv_color_hex(0x944E2B), lvglDeleteChatConversationForAllEvent, nullptr);
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

            makeSmallBtn(lvglChatComposer, lvglSymbolText(LV_SYMBOL_UPLOAD, "Send").c_str(), 82, 38, lv_color_hex(0x3A7A3A),
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
            lvglChatPeerScanBtn = makeSmallBtn(peerOps, lvglSymbolText(LV_SYMBOL_REFRESH, "Scan").c_str(), 74, 26, lv_color_hex(0x2F6D86), lvglChatPeerActionEvent, reinterpret_cast<void *>(static_cast<intptr_t>(0)));

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
            makeSmallBtn(mediaCtrlRow, lvglSymbolText(LV_SYMBOL_REFRESH, "Refresh").c_str(), 82, 24, lv_color_hex(0x2F6D86), lvglMediaRefreshEvent);
            lvglMediaPrevBtn = makeSmallBtn(mediaCtrlRow, lvglSymbolText(LV_SYMBOL_PREV, "Prev").c_str(), 60, 24, lv_color_hex(0x355C3D), lvglMediaPrevTrackEvent);
            lvglMediaPlayBtn = makeSmallBtn(mediaCtrlRow, lvglSymbolText(LV_SYMBOL_PLAY, "Play").c_str(), 60, 24, lv_color_hex(0x7C3A3A), lvglMediaPlayStopEvent);
            lvglMediaNextBtn = makeSmallBtn(mediaCtrlRow, lvglSymbolText(LV_SYMBOL_NEXT, "Next").c_str(), 60, 24, lv_color_hex(0x355C3D), lvglMediaNextTrackEvent);
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

            {
                lv_obj_t *tmp = nullptr;
                lv_obj_t *card = lvglCreateInfoCard(lvglInfoList, LV_SYMBOL_BATTERY_FULL, "Battery", lv_color_hex(0x52B788),
                                                    &lvglInfoBatteryValueLabel, &lvglInfoBatterySubLabel, &tmp);
                lvglRegisterReorderableItem(card, "ord_info", "battery");
            }
            {
                lv_obj_t *tmp = nullptr;
                lv_obj_t *card = lvglCreateInfoCard(lvglInfoList, LV_SYMBOL_WIFI, "WiFi Strength", lv_color_hex(0x4FC3F7),
                                                    &lvglInfoWifiValueLabel, &lvglInfoWifiSubLabel, &tmp);
                lvglRegisterReorderableItem(card, "ord_info", "wifi");
            }
            lv_obj_t *hc12InfoCard = lvglCreateInfoCard(lvglInfoList, LV_SYMBOL_SETTINGS, "Radio Info", lv_color_hex(0x7A5C2E),
                                                        &lvglInfoHc12ValueLabel, &lvglInfoHc12SubLabel);
            lvglRegisterReorderableItem(hc12InfoCard, "ord_info", "hc12");
            {
                lv_obj_t *tmp = nullptr;
                lv_obj_t *card = lvglCreateInfoCard(lvglInfoList, LV_SYMBOL_EYE_OPEN, "Lighting", lv_color_hex(0xF4B942),
                                                    &lvglInfoLightValueLabel, &lvglInfoLightSubLabel, &tmp);
                lvglRegisterReorderableItem(card, "ord_info", "light");
            }
            {
                lv_obj_t *tmp = nullptr;
                lv_obj_t *card = lvglCreateInfoCard(lvglInfoList, LV_SYMBOL_SD_CARD, "SD Card", lv_color_hex(0xE59F45),
                                                    nullptr, nullptr, &tmp);
                lvglRegisterReorderableItem(card, "ord_info", "sd");
            }
            {
                lv_obj_t *tmp = nullptr;
                lvglInfoSramCard = lvglCreateInfoCard(lvglInfoList, LV_SYMBOL_DIRECTORY, "SRAM", lv_color_hex(0x3FA7D6),
                                                      nullptr, nullptr, &tmp);
                lvglInfoPsramCard = lvglCreateInfoCard(lvglInfoList, LV_SYMBOL_DRIVE, "PSRAM", lv_color_hex(0x9B7CF2),
                                                       nullptr, nullptr, &tmp);
                lvglRegisterReorderableItem(lvglInfoSramCard, "ord_info", "sram");
                lvglRegisterReorderableItem(lvglInfoPsramCard, "ord_info", "psram");
            }
            {
                lv_obj_t *tmp = nullptr;
                lvglInfoCpuCard = lvglCreateInfoCard(lvglInfoList, LV_SYMBOL_CHARGE, "Processor Load", lv_color_hex(0x7BC96F),
                                                     nullptr, nullptr, &tmp);
                lvglInfoTempCard = lvglCreateInfoCard(lvglInfoList, LV_SYMBOL_WARNING, "Chip Temperature", lv_color_hex(0xF2C35E),
                                                      nullptr, nullptr, &tmp);
                lvglRegisterReorderableItem(lvglInfoCpuCard, "ord_info", "cpu");
                lvglRegisterReorderableItem(lvglInfoTempCard, "ord_info", "temp");
            }

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
            lvglRegisterReorderableItem(systemCard, "ord_info", "system");
            lvglApplySavedOrder(lvglInfoList, "ord_info");
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
            lv_obj_t *gamesSnakeBtn = lvglCreateMenuButton(gamesWrap, lvglSymbolText(LV_SYMBOL_PLAY, "Snake").c_str(), lv_color_hex(0x3A8F4B), lvglOpenSnakeEvent, nullptr);
            lv_obj_t *gamesTetrisBtn = lvglCreateMenuButton(gamesWrap, lvglSymbolText(LV_SYMBOL_PLAY, "Tetris").c_str(), lv_color_hex(0x376B93), lvglOpenTetrisEvent, nullptr);
            lv_obj_t *gamesCheckersBtn = lvglCreateMenuButton(gamesWrap, lvglSymbolText(LV_SYMBOL_PLAY, "Checkers").c_str(), lv_color_hex(0x8A5A25), lvglOpenCheckersEvent, nullptr);
#if defined(BOARD_ESP32S3_3248S035_N16R8)
            lv_obj_t *gamesSnake3dBtn = lvglCreateMenuButton(gamesWrap, lvglSymbolText(LV_SYMBOL_PLAY, "Snake 3D").c_str(), lv_color_hex(0x5A4CC7), lvglOpenSnake3dEvent, nullptr);
            lvglRegisterReorderableItem(gamesSnake3dBtn, "ord_games", "snake3d");
#endif
            lvglRegisterReorderableItem(gamesSnakeBtn, "ord_games", "snake");
            lvglRegisterReorderableItem(gamesTetrisBtn, "ord_games", "tetris");
            lvglRegisterReorderableItem(gamesCheckersBtn, "ord_games", "checkers");
            lvglApplySavedOrder(gamesWrap, "ord_games");
            break;
        }
        case UI_CONFIG: {
            lvglScrConfig = lvglCreateScreenBase(tr(TXT_CONFIG), true);
            lvglConfigWrap = lv_obj_create(lvglScrConfig);
            lv_obj_set_size(lvglConfigWrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(lvglConfigWrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(lvglConfigWrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(lvglConfigWrap, 0, 0);
            lv_obj_set_style_pad_all(lvglConfigWrap, 10, 0);
            lv_obj_set_style_pad_row(lvglConfigWrap, 10, 0);
            lv_obj_set_flex_flow(lvglConfigWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scrollbar_mode(lvglConfigWrap, LV_SCROLLBAR_MODE_OFF);
            lvglConfigWifiBtn = lvglCreateMenuButton(lvglConfigWrap, tr(TXT_WIFI_CONFIG), lv_color_hex(0x3A8F4B), lvglHomeNavEvent, reinterpret_cast<void *>(static_cast<intptr_t>(UI_WIFI_LIST)));
            lvglConfigHc12Btn = lvglCreateMenuButton(lvglConfigWrap, tr(TXT_HC12_CONFIG), lv_color_hex(0x7A5C2E), lvglOpenHc12ScreenEvent, nullptr);
            lvglConfigStyleBtn = lvglCreateMenuButton(lvglConfigWrap, tr(TXT_STYLE), lv_color_hex(0x2D6D8E), lvglOpenStyleScreenEvent, nullptr);
            lvglConfigBatteryBtn = lvglCreateMenuButton(lvglConfigWrap, "Battery", lv_color_hex(0x6C7E2C), lvglHomeNavEvent, reinterpret_cast<void *>(static_cast<intptr_t>(UI_CONFIG_BATTERY)));
            lvglConfigMqttBtn = lvglCreateMenuButton(lvglConfigWrap, tr(TXT_MQTT_CONFIG), lv_color_hex(0x6D4B9A), lvglOpenMqttCfgEvent, nullptr);
            lvglConfigMqttControlsBtn = lvglCreateMenuButton(lvglConfigWrap, tr(TXT_MQTT_CONTROLS), lv_color_hex(0x2D6D8E), lvglOpenMqttCtrlEvent, nullptr);
            lvglConfigScreenshotBtn = lvglCreateMenuButton(lvglConfigWrap, tr(TXT_SCREENSHOT), lv_color_hex(0x6B5B2A), lvglScreenshotEvent, nullptr);
            lvglConfigLanguageBtn = lvglCreateMenuButton(lvglConfigWrap, tr(TXT_LANGUAGE), lv_color_hex(0x5A6FA8), lvglOpenLanguageScreenEvent, nullptr);
            lvglConfigOtaBtn = lvglCreateMenuButton(lvglConfigWrap, tr(TXT_OTA_UPDATES), lv_color_hex(0x2E6F95), lvglOpenOtaScreenEvent, nullptr);
            lvglRefreshPrimaryMenuButtonIcons();

            lv_obj_t *nameWrap = lv_obj_create(lvglConfigWrap);
            lv_obj_set_size(nameWrap, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(nameWrap, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(nameWrap, 0, 0);
            lv_obj_set_style_radius(nameWrap, 12, 0);
            lv_obj_set_style_pad_all(nameWrap, 10, 0);
            lv_obj_set_style_pad_row(nameWrap, 6, 0);
            lv_obj_set_flex_flow(nameWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(nameWrap, LV_OBJ_FLAG_SCROLLABLE);

            lvglConfigDeviceNameHeader = lv_label_create(nameWrap);
            lv_label_set_text(lvglConfigDeviceNameHeader, tr(TXT_DEVICE_NAME));
            lv_obj_set_style_text_color(lvglConfigDeviceNameHeader, lv_color_hex(0xE5ECF3), 0);

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

            lv_obj_t *saveBtn = makeSmallBtn(nameRow, tr(TXT_SAVE), 64, 38, lv_color_hex(0x3A7A3A), lvglSaveDeviceNameEvent, nullptr);
            lvglConfigDeviceNameSaveBtnLabel = saveBtn ? lv_obj_get_child(saveBtn, 0) : nullptr;

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

            lvglConfigBrightnessHeader = lv_label_create(brightHdr);
            lv_label_set_text(lvglConfigBrightnessHeader, tr(TXT_BRIGHTNESS));
            lv_obj_set_style_text_color(lvglConfigBrightnessHeader, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_flex_grow(lvglConfigBrightnessHeader, 1);

            lvglBrightnessValueLabel = lv_label_create(brightHdr);
            lv_obj_set_style_text_color(lvglBrightnessValueLabel, lv_color_hex(0xB7C4D1), 0);

            lvglBrightnessSlider = lv_slider_create(brightWrap);
            lv_obj_set_width(lvglBrightnessSlider, lv_pct(100));
            lv_slider_set_range(lvglBrightnessSlider, 5, 100);
            lv_obj_add_event_cb(lvglBrightnessSlider, lvglBrightnessEvent, LV_EVENT_VALUE_CHANGED, nullptr);

            lv_obj_t *volWrap = lv_obj_create(lvglConfigWrap);
            lv_obj_set_size(volWrap, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(volWrap, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(volWrap, 0, 0);
            lv_obj_set_style_radius(volWrap, 12, 0);
            lv_obj_set_style_pad_all(volWrap, 10, 0);
            lv_obj_set_style_pad_row(volWrap, 6, 0);
            lv_obj_set_flex_flow(volWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(volWrap, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *volHdr = lv_obj_create(volWrap);
            lv_obj_set_size(volHdr, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(volHdr, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(volHdr, 0, 0);
            lv_obj_set_style_pad_all(volHdr, 0, 0);
            lv_obj_set_style_pad_column(volHdr, 8, 0);
            lv_obj_set_flex_flow(volHdr, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(volHdr, LV_OBJ_FLAG_SCROLLABLE);

            lvglConfigVolumeHeader = lv_label_create(volHdr);
            lv_label_set_text(lvglConfigVolumeHeader, tr(TXT_VOLUME));
            lv_obj_set_style_text_color(lvglConfigVolumeHeader, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_flex_grow(lvglConfigVolumeHeader, 1);

            lvglVolumeValueLabel = lv_label_create(volHdr);
            lv_obj_set_style_text_color(lvglVolumeValueLabel, lv_color_hex(0xB7C4D1), 0);

            lvglVolumeSlider = lv_slider_create(volWrap);
            lv_obj_set_width(lvglVolumeSlider, lv_pct(100));
            lv_slider_set_range(lvglVolumeSlider, 0, 100);
            lv_obj_add_event_cb(lvglVolumeSlider, lvglConfigVolumeEvent, LV_EVENT_VALUE_CHANGED, nullptr);

            lv_obj_t *vibrationWrap = lv_obj_create(lvglConfigWrap);
            lv_obj_set_size(vibrationWrap, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(vibrationWrap, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(vibrationWrap, 0, 0);
            lv_obj_set_style_radius(vibrationWrap, 12, 0);
            lv_obj_set_style_pad_all(vibrationWrap, 10, 0);
            lv_obj_set_style_pad_row(vibrationWrap, 6, 0);
            lv_obj_set_flex_flow(vibrationWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(vibrationWrap, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *vibrationHdr = lv_obj_create(vibrationWrap);
            lv_obj_set_size(vibrationHdr, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(vibrationHdr, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(vibrationHdr, 0, 0);
            lv_obj_set_style_pad_all(vibrationHdr, 0, 0);
            lv_obj_set_style_pad_column(vibrationHdr, 8, 0);
            lv_obj_set_flex_flow(vibrationHdr, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(vibrationHdr, LV_OBJ_FLAG_SCROLLABLE);

            lvglConfigVibrationHeader = lv_label_create(vibrationHdr);
            lv_label_set_text(lvglConfigVibrationHeader, "Vibration");
            lv_obj_set_style_text_color(lvglConfigVibrationHeader, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_flex_grow(lvglConfigVibrationHeader, 1);

            lvglVibrationDropdown = lv_dropdown_create(vibrationWrap);
            lv_obj_set_width(lvglVibrationDropdown, lv_pct(100));
            lv_dropdown_set_options(lvglVibrationDropdown, buildVibrationIntensityDropdownOptions().c_str());
            lv_obj_add_event_cb(lvglVibrationDropdown, lvglVibrationDropdownEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglVibrationDropdown, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglVibrationDropdown, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglVibrationDropdown, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);

            lv_obj_t *toneWrap = lv_obj_create(lvglConfigWrap);
            lv_obj_set_size(toneWrap, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(toneWrap, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(toneWrap, 0, 0);
            lv_obj_set_style_radius(toneWrap, 12, 0);
            lv_obj_set_style_pad_all(toneWrap, 10, 0);
            lv_obj_set_style_pad_row(toneWrap, 6, 0);
            lv_obj_set_flex_flow(toneWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(toneWrap, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *toneHdr = lv_obj_create(toneWrap);
            lv_obj_set_size(toneHdr, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(toneHdr, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(toneHdr, 0, 0);
            lv_obj_set_style_pad_all(toneHdr, 0, 0);
            lv_obj_set_style_pad_column(toneHdr, 8, 0);
            lv_obj_set_flex_flow(toneHdr, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(toneHdr, LV_OBJ_FLAG_SCROLLABLE);

            lvglConfigMessageToneHeader = lv_label_create(toneHdr);
            lv_label_set_text(lvglConfigMessageToneHeader, "Message tone");
            lv_obj_set_style_text_color(lvglConfigMessageToneHeader, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_flex_grow(lvglConfigMessageToneHeader, 1);

            lvglMessageToneDropdown = lv_dropdown_create(toneWrap);
            lv_obj_set_width(lvglMessageToneDropdown, lv_pct(100));
            lv_dropdown_set_options(lvglMessageToneDropdown, buildMessageBeepDropdownOptions().c_str());
            lv_obj_add_event_cb(lvglMessageToneDropdown, lvglMessageToneDropdownEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglMessageToneDropdown, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglMessageToneDropdown, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglMessageToneDropdown, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);

            lv_obj_t *rgbWrap = lv_obj_create(lvglConfigWrap);
            lv_obj_set_size(rgbWrap, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(rgbWrap, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(rgbWrap, 0, 0);
            lv_obj_set_style_radius(rgbWrap, 12, 0);
            lv_obj_set_style_pad_all(rgbWrap, 10, 0);
            lv_obj_set_style_pad_row(rgbWrap, 6, 0);
            lv_obj_set_flex_flow(rgbWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(rgbWrap, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *rgbHdr = lv_obj_create(rgbWrap);
            lv_obj_set_size(rgbHdr, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(rgbHdr, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(rgbHdr, 0, 0);
            lv_obj_set_style_pad_all(rgbHdr, 0, 0);
            lv_obj_set_style_pad_column(rgbHdr, 8, 0);
            lv_obj_set_flex_flow(rgbHdr, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(rgbHdr, LV_OBJ_FLAG_SCROLLABLE);

            lvglConfigRgbHeader = lv_label_create(rgbHdr);
            lv_label_set_text(lvglConfigRgbHeader, tr(TXT_RGB_LED));
            lv_obj_set_style_text_color(lvglConfigRgbHeader, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_flex_grow(lvglConfigRgbHeader, 1);

            lv_obj_t *rgbValueLabel = lv_label_create(rgbHdr);
            lv_obj_set_style_text_color(rgbValueLabel, lv_color_hex(0xB7C4D1), 0);
            lv_label_set_text_fmt(rgbValueLabel, "%u%%", static_cast<unsigned int>(rgbLedPercent));

            lvglRgbLedSlider = lv_slider_create(rgbWrap);
            lv_obj_set_width(lvglRgbLedSlider, lv_pct(100));
            lv_slider_set_range(lvglRgbLedSlider, 0, 100);
            lv_obj_add_event_cb(lvglRgbLedSlider, lvglRgbLedEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lvglApplyConfigScreenControlStyles();
            lvglRegisterReorderableItem(lvglConfigWifiBtn, "ord_cfg", "wifi");
            lvglRegisterReorderableItem(lvglConfigHc12Btn, "ord_cfg", "hc12");
            lvglRegisterReorderableItem(lvglConfigStyleBtn, "ord_cfg", "style");
            lvglRegisterReorderableItem(lvglConfigMqttBtn, "ord_cfg", "mqtt");
            lvglRegisterReorderableItem(lvglConfigMqttControlsBtn, "ord_cfg", "mctl");
            lvglRegisterReorderableItem(lvglConfigScreenshotBtn, "ord_cfg", "shot");
            lvglRegisterReorderableItem(lvglConfigLanguageBtn, "ord_cfg", "lang");
            lvglRegisterReorderableItem(lvglConfigOtaBtn, "ord_cfg", "ota");
            lvglRegisterReorderableItem(nameWrap, "ord_cfg", "name");
            lvglRegisterReorderableItem(brightWrap, "ord_cfg", "bright");
            lvglRegisterReorderableItem(volWrap, "ord_cfg", "vol");
            lvglRegisterReorderableItem(vibrationWrap, "ord_cfg", "vib");
            lvglRegisterReorderableItem(toneWrap, "ord_cfg", "tone");
            lvglRegisterReorderableItem(rgbWrap, "ord_cfg", "rgb");
            lvglApplySavedOrder(lvglConfigWrap, "ord_cfg");
            lvglRefreshConfigUi();
            break;
        }
        case UI_CONFIG_BATTERY: {
            lvglScrBatteryTrain = lvglCreateScreenBase("Battery", false);
            lvglBatteryTrainStatusLabel = nullptr;
            lvglBatteryTrainCurrentLabel = nullptr;
            lvglBatteryTrainFullLabel = nullptr;
            lvglBatteryTrainEmptyLabel = nullptr;
            lvglBatteryTrainFactorLabel = nullptr;
            lvglBatteryTrainPowerLabel = nullptr;
            lvglBatteryTrainFullBtn = nullptr;
            lvglBatteryTrainDischargeBtn = nullptr;
            lvglBatteryTrainAutoBtn = nullptr;

            lv_obj_t *wrap = lv_obj_create(lvglScrBatteryTrain);
            lv_obj_set_size(wrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(wrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(wrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(wrap, 0, 0);
            lv_obj_set_style_pad_all(wrap, 10, 0);
            lv_obj_set_style_pad_row(wrap, 10, 0);
            lv_obj_set_flex_flow(wrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scrollbar_mode(wrap, LV_SCROLLBAR_MODE_OFF);

            lv_obj_t *stateCard = lv_obj_create(wrap);
            lv_obj_set_size(stateCard, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(stateCard, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(stateCard, 0, 0);
            lv_obj_set_style_radius(stateCard, 12, 0);
            lv_obj_set_style_pad_all(stateCard, 12, 0);
            lv_obj_set_style_pad_row(stateCard, 7, 0);
            lv_obj_set_flex_flow(stateCard, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(stateCard, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *stateTitle = lv_label_create(stateCard);
            lv_label_set_text(stateTitle, "Stored Calibration");
            lv_obj_set_style_text_color(stateTitle, lv_color_hex(0xE5ECF3), 0);

            auto makeBatteryInfoLabel = [&](lv_obj_t **out, lv_color_t color) {
                lv_obj_t *label = lv_label_create(stateCard);
                lv_obj_set_width(label, lv_pct(100));
                lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
                lv_obj_set_style_text_color(label, color, 0);
                if (out) *out = label;
            };

            makeBatteryInfoLabel(&lvglBatteryTrainStatusLabel, lv_color_hex(0xE5ECF3));
            makeBatteryInfoLabel(&lvglBatteryTrainCurrentLabel, lv_color_hex(0xB7C4D1));
            makeBatteryInfoLabel(&lvglBatteryTrainFullLabel, lv_color_hex(0xB7C4D1));
            makeBatteryInfoLabel(&lvglBatteryTrainEmptyLabel, lv_color_hex(0xB7C4D1));
            makeBatteryInfoLabel(&lvglBatteryTrainFactorLabel, lv_color_hex(0xB7C4D1));
            makeBatteryInfoLabel(&lvglBatteryTrainPowerLabel, lv_color_hex(0xB7C4D1));

            lv_obj_t *hintCard = lv_obj_create(wrap);
            lv_obj_set_size(hintCard, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(hintCard, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(hintCard, 0, 0);
            lv_obj_set_style_radius(hintCard, 12, 0);
            lv_obj_set_style_pad_all(hintCard, 12, 0);
            lv_obj_set_style_pad_row(hintCard, 8, 0);
            lv_obj_set_flex_flow(hintCard, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(hintCard, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *hint = lv_label_create(hintCard);
            lv_obj_set_width(hint, lv_pct(100));
            lv_label_set_long_mode(hint, LV_LABEL_LONG_WRAP);
            lv_obj_set_style_text_color(hint, lv_color_hex(0x9FB0C2), 0);
            lv_label_set_text(hint,
                              "Reset clears stored anchors and starts charge training. FULL stores the current top voltage and restores your power setting. "
                              "DISCHARGE forces Power Off to Never until the next low-battery cutoff is learned. Auto Calibration must be confirmed and only then enables the background learning algorithm.");

            makeSmallBtn(wrap, "Reset", DISPLAY_WIDTH - 20, 40, lv_color_hex(0x8D5F1E), lvglBatteryTrainResetEvent, nullptr);

            lv_obj_t *buttonRow = lv_obj_create(wrap);
            lv_obj_set_size(buttonRow, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(buttonRow, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(buttonRow, 0, 0);
            lv_obj_set_style_pad_all(buttonRow, 0, 0);
            lv_obj_set_style_pad_column(buttonRow, 10, 0);
            lv_obj_set_flex_flow(buttonRow, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(buttonRow, LV_OBJ_FLAG_SCROLLABLE);

            lvglBatteryTrainFullBtn = makeSmallBtn(buttonRow, "FULL", (DISPLAY_WIDTH - 30) / 2, 42, lv_color_hex(0x2F7A45), lvglBatteryTrainFullEvent, nullptr);
            lvglBatteryTrainDischargeBtn = makeSmallBtn(buttonRow, "DISCHARGE", (DISPLAY_WIDTH - 30) / 2, 42, lv_color_hex(0x8A3A2E), lvglBatteryTrainDischargeEvent, nullptr);
            lvglBatteryTrainAutoBtn = makeSmallBtn(wrap, "Auto Calibration", DISPLAY_WIDTH - 20, 40, lv_color_hex(0x345D8A), lvglBatteryTrainAutoEvent, nullptr);
            lvglRefreshBatteryTrainButtonIcons();
            lvglRefreshBatteryTrainUi();
            break;
        }
        case UI_CONFIG_STYLE: {
            lvglScrStyle = lvglCreateScreenBase("Style", false);
            lvglStyleButtonFlatBtn = nullptr;
            lvglStyleButtonFlatBtnLabel = nullptr;
            lvglStyleButton3dBtn = nullptr;
            lvglStyleButton3dBtnLabel = nullptr;
            lvglStyleButtonBlackBtn = nullptr;
            lvglStyleButtonBlackBtnLabel = nullptr;
            lvglStyleMenuIconsSw = nullptr;
            lvglStyleTimezoneDd = nullptr;
            lvglStyleTopCenterNameBtn = nullptr;
            lvglStyleTopCenterNameBtnLabel = nullptr;
            lvglStyleTopCenterTimeBtn = nullptr;
            lvglStyleTopCenterTimeBtnLabel = nullptr;
            lv_obj_t *wrap = lv_obj_create(lvglScrStyle);
            lv_obj_set_size(wrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(wrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(wrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(wrap, 0, 0);
            lv_obj_set_style_pad_all(wrap, 10, 0);
            lv_obj_set_style_pad_row(wrap, 10, 0);
            lv_obj_set_flex_flow(wrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scrollbar_mode(wrap, LV_SCROLLBAR_MODE_OFF);

            lv_obj_t *screensaverRow = lv_obj_create(wrap);
            lv_obj_set_size(screensaverRow, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(screensaverRow, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(screensaverRow, 0, 0);
            lv_obj_set_style_radius(screensaverRow, 12, 0);
            lv_obj_set_style_pad_all(screensaverRow, 12, 0);
            lv_obj_set_style_pad_column(screensaverRow, 8, 0);
            lv_obj_set_flex_flow(screensaverRow, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(screensaverRow, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *screensaverLbl = lv_label_create(screensaverRow);
            lv_label_set_text(screensaverLbl, "Screensaver");
            lv_obj_set_style_text_color(screensaverLbl, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_flex_grow(screensaverLbl, 1);

            lvglStyleScreensaverSw = lv_switch_create(screensaverRow);
            lv_obj_add_event_cb(lvglStyleScreensaverSw, lvglStyleScreensaverToggleEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglStyleScreensaverSw, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglStyleScreensaverSw, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglStyleScreensaverSw, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);

            lv_obj_t *menuIconsRow = lv_obj_create(wrap);
            lv_obj_set_size(menuIconsRow, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(menuIconsRow, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(menuIconsRow, 0, 0);
            lv_obj_set_style_radius(menuIconsRow, 12, 0);
            lv_obj_set_style_pad_all(menuIconsRow, 12, 0);
            lv_obj_set_style_pad_column(menuIconsRow, 8, 0);
            lv_obj_set_flex_flow(menuIconsRow, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(menuIconsRow, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *menuIconsLbl = lv_label_create(menuIconsRow);
            lv_label_set_text(menuIconsLbl, "3D Icons");
            lv_obj_set_style_text_color(menuIconsLbl, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_flex_grow(menuIconsLbl, 1);

            lvglStyleMenuIconsSw = lv_switch_create(menuIconsRow);
            lv_obj_add_event_cb(lvglStyleMenuIconsSw, lvglStyleMenuIconsToggleEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglStyleMenuIconsSw, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglStyleMenuIconsSw, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglStyleMenuIconsSw, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);

            lv_obj_t *buttonStyleWrap = lv_obj_create(wrap);
            lv_obj_set_size(buttonStyleWrap, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(buttonStyleWrap, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(buttonStyleWrap, 0, 0);
            lv_obj_set_style_radius(buttonStyleWrap, 12, 0);
            lv_obj_set_style_pad_all(buttonStyleWrap, 10, 0);
            lv_obj_set_style_pad_row(buttonStyleWrap, 8, 0);
            lv_obj_set_flex_flow(buttonStyleWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(buttonStyleWrap, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *buttonStyleHdr = lv_label_create(buttonStyleWrap);
            lv_label_set_text(buttonStyleHdr, "Button Style");
            lv_obj_set_style_text_color(buttonStyleHdr, lv_color_hex(0xE5ECF3), 0);

            auto makeStyleRow = [&](lv_obj_t *parentWrap,
                                    const char *title,
                                    lv_color_t accent,
                                    lv_obj_t **btnOut,
                                    lv_obj_t **labelOut,
                                    lv_event_cb_t cb) {
                lv_obj_t *row = lv_obj_create(parentWrap);
                lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
                lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
                lv_obj_set_style_border_width(row, 0, 0);
                lv_obj_set_style_pad_all(row, 0, 0);
                lv_obj_set_style_pad_column(row, 8, 0);
                lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
                lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

                lv_obj_t *titleLbl = lv_label_create(row);
                lv_label_set_text(titleLbl, title);
                lv_obj_set_style_text_color(titleLbl, lv_color_hex(0xD7E0E8), 0);
                lv_obj_set_flex_grow(titleLbl, 1);

                lv_obj_t *btn = lv_btn_create(row);
                lv_obj_set_size(btn, 82, 28);
                lv_obj_set_style_radius(btn, 8, 0);
                lv_obj_set_style_border_width(btn, 0, 0);
                lv_obj_add_event_cb(btn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
                lv_obj_add_event_cb(btn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
                lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, nullptr);
                lv_obj_t *lbl = lv_label_create(btn);
                lv_label_set_text(lbl, "Select");
                lv_obj_center(lbl);
                lvglRegisterStyledButton(btn, accent, true);
                *btnOut = btn;
                *labelOut = lbl;
            };

            makeStyleRow(buttonStyleWrap,
                         "Flat Buttons",
                         lv_color_hex(uiButtonStyleFlatSelectorColor),
                         &lvglStyleButtonFlatBtn,
                         &lvglStyleButtonFlatBtnLabel,
                         lvglStyleButtonFlatEvent);
            makeStyleRow(buttonStyleWrap,
                         "3D Buttons",
                         lv_color_hex(uiButtonStyle3dSelectorColor),
                         &lvglStyleButton3dBtn,
                         &lvglStyleButton3dBtnLabel,
                         lvglStyleButton3dEvent);
            makeStyleRow(buttonStyleWrap,
                         "Black Buttons",
                         lv_color_hex(UI_BUTTON_BLACK_BODY_HEX),
                         &lvglStyleButtonBlackBtn,
                         &lvglStyleButtonBlackBtnLabel,
                         lvglStyleButtonBlackEvent);

            lv_obj_t *tzWrap = lv_obj_create(wrap);
            lv_obj_set_size(tzWrap, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(tzWrap, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(tzWrap, 0, 0);
            lv_obj_set_style_radius(tzWrap, 12, 0);
            lv_obj_set_style_pad_all(tzWrap, 10, 0);
            lv_obj_set_style_pad_row(tzWrap, 8, 0);
            lv_obj_set_flex_flow(tzWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(tzWrap, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *tzHdr = lv_label_create(tzWrap);
            lv_label_set_text(tzHdr, "Timezone");
            lv_obj_set_style_text_color(tzHdr, lv_color_hex(0xE5ECF3), 0);

            lvglStyleTimezoneDd = lv_dropdown_create(tzWrap);
            lv_obj_set_width(lvglStyleTimezoneDd, lv_pct(100));
            lv_dropdown_set_options(lvglStyleTimezoneDd, buildGmtOffsetDropdownOptions().c_str());
            lv_obj_set_style_bg_color(lvglStyleTimezoneDd, lv_color_hex(0x111922), 0);
            lv_obj_set_style_text_color(lvglStyleTimezoneDd, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_style_border_color(lvglStyleTimezoneDd, lv_color_hex(0x2F4658), 0);
            lv_obj_set_style_border_width(lvglStyleTimezoneDd, 1, 0);
            lv_obj_set_style_radius(lvglStyleTimezoneDd, 8, 0);
            lv_obj_add_event_cb(lvglStyleTimezoneDd, lvglStyleTimezoneEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglStyleTimezoneDd, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglStyleTimezoneDd, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglStyleTimezoneDd, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);

            lv_obj_t *topCenterWrap = lv_obj_create(wrap);
            lv_obj_set_size(topCenterWrap, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(topCenterWrap, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(topCenterWrap, 0, 0);
            lv_obj_set_style_radius(topCenterWrap, 12, 0);
            lv_obj_set_style_pad_all(topCenterWrap, 10, 0);
            lv_obj_set_style_pad_row(topCenterWrap, 8, 0);
            lv_obj_set_flex_flow(topCenterWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(topCenterWrap, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *topCenterHdr = lv_label_create(topCenterWrap);
            lv_label_set_text(topCenterHdr, "Top Bar Center");
            lv_obj_set_style_text_color(topCenterHdr, lv_color_hex(0xE5ECF3), 0);

            makeStyleRow(topCenterWrap,
                         "Show Device Name",
                         lv_color_hex(0x3F4A57),
                         &lvglStyleTopCenterNameBtn,
                         &lvglStyleTopCenterNameBtnLabel,
                         lvglStyleTopCenterNameEvent);
            makeStyleRow(topCenterWrap,
                         "Show Time",
                         lv_color_hex(0x3F4A57),
                         &lvglStyleTopCenterTimeBtn,
                         &lvglStyleTopCenterTimeBtnLabel,
                         lvglStyleTopCenterTimeEvent);

            lv_obj_t *timeWrap = lv_obj_create(wrap);
            lv_obj_set_size(timeWrap, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(timeWrap, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(timeWrap, 0, 0);
            lv_obj_set_style_radius(timeWrap, 12, 0);
            lv_obj_set_style_pad_all(timeWrap, 10, 0);
            lv_obj_set_style_pad_row(timeWrap, 6, 0);
            lv_obj_set_flex_flow(timeWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(timeWrap, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *timeHdr = lv_obj_create(timeWrap);
            lv_obj_set_size(timeHdr, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(timeHdr, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(timeHdr, 0, 0);
            lv_obj_set_style_pad_all(timeHdr, 0, 0);
            lv_obj_set_style_pad_column(timeHdr, 8, 0);
            lv_obj_set_flex_flow(timeHdr, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(timeHdr, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *timeLbl = lv_label_create(timeHdr);
            lv_label_set_text(timeLbl, "Screen Timeoff");
            lv_obj_set_style_text_color(timeLbl, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_flex_grow(timeLbl, 1);

            lvglStyleTimeoutValueLabel = lv_label_create(timeHdr);
            lv_obj_set_style_text_color(lvglStyleTimeoutValueLabel, lv_color_hex(0xB7C4D1), 0);
            lv_label_set_text(lvglStyleTimeoutValueLabel, "--");

            lvglStyleTimeoutSlider = lv_slider_create(timeWrap);
            lv_obj_set_width(lvglStyleTimeoutSlider, lv_pct(100));
            lv_slider_set_range(lvglStyleTimeoutSlider,
                                static_cast<int32_t>(LCD_IDLE_TIMEOUT_MS_MIN / LCD_IDLE_TIMEOUT_STEP_MS),
                                static_cast<int32_t>(LCD_IDLE_TIMEOUT_MS_MAX / LCD_IDLE_TIMEOUT_STEP_MS));
            lv_obj_add_event_cb(lvglStyleTimeoutSlider, lvglStyleTimeoutEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglStyleTimeoutSlider, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglStyleTimeoutSlider, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglStyleTimeoutSlider, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);
            lvglApplyStyleScreenControlStyles();

            lv_obj_t *powerWrap = lv_obj_create(timeWrap);
            lv_obj_set_size(powerWrap, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(powerWrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(powerWrap, 0, 0);
            lv_obj_set_style_pad_all(powerWrap, 0, 0);
            lv_obj_set_style_pad_row(powerWrap, 6, 0);
            lv_obj_set_flex_flow(powerWrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(powerWrap, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *powerHdr = lv_label_create(powerWrap);
            lv_label_set_text(powerHdr, "Power Off");
            lv_obj_set_style_text_color(powerHdr, lv_color_hex(0xE5ECF3), 0);

            lvglStylePowerOffDropdown = lv_dropdown_create(powerWrap);
            lv_obj_set_width(lvglStylePowerOffDropdown, lv_pct(100));
            lv_dropdown_set_options(lvglStylePowerOffDropdown, buildPowerOffTimeoutDropdownOptions().c_str());
            lv_obj_set_style_bg_color(lvglStylePowerOffDropdown, lv_color_hex(0x111922), 0);
            lv_obj_set_style_text_color(lvglStylePowerOffDropdown, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_style_border_color(lvglStylePowerOffDropdown, lv_color_hex(0x2F4658), 0);
            lv_obj_set_style_border_width(lvglStylePowerOffDropdown, 1, 0);
            lv_obj_set_style_radius(lvglStylePowerOffDropdown, 8, 0);
            lv_obj_add_event_cb(lvglStylePowerOffDropdown, lvglStylePowerOffEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglStylePowerOffDropdown, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglStylePowerOffDropdown, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglStylePowerOffDropdown, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);

            lv_obj_t *hint = lv_label_create(timeWrap);
            lv_obj_set_width(hint, lv_pct(100));
            lv_label_set_long_mode(hint, LV_LABEL_LONG_WRAP);
            lv_obj_set_style_text_color(hint, lv_color_hex(0x9FB0C2), 0);
            lv_label_set_text(hint, "Screen Timeoff controls sleep/screensaver. Power Off can send a hardware shutdown signal after longer inactivity to save battery.");
            lvglRefreshStyleUi();
            break;
        }
        case UI_CONFIG_LANGUAGE: {
            lvglScrLanguage = lvglCreateScreenBase(tr(TXT_LANGUAGE), false);
            lvglLanguageDropdown = nullptr;
            lvglLanguageInfoLabel = nullptr;

            lv_obj_t *wrap = lv_obj_create(lvglScrLanguage);
            lv_obj_set_size(wrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(wrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(wrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(wrap, 0, 0);
            lv_obj_set_style_pad_all(wrap, 10, 0);
            lv_obj_set_style_pad_row(wrap, 10, 0);
            lv_obj_set_flex_flow(wrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scrollbar_mode(wrap, LV_SCROLLBAR_MODE_OFF);

            lv_obj_t *langCard = lv_obj_create(wrap);
            lv_obj_set_size(langCard, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(langCard, lv_color_hex(0x18222D), 0);
            lv_obj_set_style_border_width(langCard, 0, 0);
            lv_obj_set_style_radius(langCard, 12, 0);
            lv_obj_set_style_pad_all(langCard, 12, 0);
            lv_obj_set_style_pad_row(langCard, 8, 0);
            lv_obj_set_flex_flow(langCard, LV_FLEX_FLOW_COLUMN);
            lv_obj_clear_flag(langCard, LV_OBJ_FLAG_SCROLLABLE);

            lvglLanguageInfoLabel = lv_label_create(langCard);
            lv_obj_set_width(lvglLanguageInfoLabel, lv_pct(100));
            lv_label_set_long_mode(lvglLanguageInfoLabel, LV_LABEL_LONG_WRAP);
            lv_obj_set_style_text_color(lvglLanguageInfoLabel, lv_color_hex(0xE5ECF3), 0);

            lvglLanguageDropdown = lv_dropdown_create(langCard);
            lv_obj_set_width(lvglLanguageDropdown, lv_pct(100));
            lv_dropdown_set_options(lvglLanguageDropdown, buildLanguageDropdownOptions().c_str());
            lv_obj_set_style_bg_color(lvglLanguageDropdown, lv_color_hex(0x111922), 0);
            lv_obj_set_style_text_color(lvglLanguageDropdown, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_style_border_color(lvglLanguageDropdown, lv_color_hex(0x2F4658), 0);
            lv_obj_set_style_border_width(lvglLanguageDropdown, 1, 0);
            lv_obj_set_style_radius(lvglLanguageDropdown, 8, 0);
            lv_obj_add_event_cb(lvglLanguageDropdown, lvglLanguageDropdownEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglLanguageDropdown, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglLanguageDropdown, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglLanguageDropdown, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);
            lvglRefreshLanguageUi();
            break;
        }
        case UI_CONFIG_OTA: {
            lvglScrOta = lvglCreateScreenBase(tr(TXT_OTA_UPDATES), false);
            lv_obj_t *wrap = lv_obj_create(lvglScrOta);
            lv_obj_set_size(wrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(wrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(wrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(wrap, 0, 0);
            lv_obj_set_style_pad_all(wrap, 10, 0);
            lv_obj_set_style_pad_row(wrap, 10, 0);
            lv_obj_set_flex_flow(wrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scrollbar_mode(wrap, LV_SCROLLBAR_MODE_OFF);

            auto makeInfoLine = [&](const char *title, lv_obj_t **valueOut) -> lv_obj_t * {
                lv_obj_t *card = lv_obj_create(wrap);
                lv_obj_set_width(card, lv_pct(100));
                lv_obj_set_height(card, LV_SIZE_CONTENT);
                lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
                lv_obj_set_flex_align(card, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
                lv_obj_set_style_bg_color(card, lv_color_hex(0x16212C), 0);
                lv_obj_set_style_border_color(card, lv_color_hex(0x536274), 0);
                lv_obj_set_style_border_width(card, 1, 0);
                lv_obj_set_style_radius(card, 12, 0);
                lv_obj_set_style_pad_all(card, 10, 0);
                lv_obj_set_style_pad_row(card, 4, 0);
                lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_t *label = lv_label_create(card);
                lv_label_set_text(label, title);
                lv_obj_set_style_text_color(label, lv_color_hex(0x9FB0C2), 0);
                lv_obj_t *value = lv_label_create(card);
                lv_obj_set_width(value, lv_pct(100));
                lv_label_set_long_mode(value, LV_LABEL_LONG_WRAP);
                lv_label_set_text(value, "--");
                lv_obj_set_style_text_color(value, lv_color_hex(0xF5F8FB), 0);
                *valueOut = value;
                return card;
            };

            lv_obj_t *otaCurrentCard = makeInfoLine("Current version", &lvglOtaCurrentLabel);
            lv_obj_t *otaLatestCard = makeInfoLine("Latest version", &lvglOtaLatestLabel);
            lv_obj_t *otaStatusCard = makeInfoLine("Status", &lvglOtaStatusLabel);

            lvglOtaProgressBar = lv_bar_create(wrap);
            lv_obj_set_size(lvglOtaProgressBar, lv_pct(100), 14);
            lv_bar_set_range(lvglOtaProgressBar, 0, 100);
            lv_bar_set_value(lvglOtaProgressBar, 0, LV_ANIM_OFF);
            lv_obj_set_style_bg_color(lvglOtaProgressBar, lv_color_hex(0x2A3340), LV_PART_MAIN);
            lv_obj_set_style_bg_color(lvglOtaProgressBar, lv_color_hex(0x52B788), LV_PART_INDICATOR);

            lvglOtaProgressLabel = lv_label_create(wrap);
            lv_obj_set_width(lvglOtaProgressLabel, lv_pct(100));
            lv_obj_set_style_text_align(lvglOtaProgressLabel, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_set_style_text_color(lvglOtaProgressLabel, lv_color_hex(0xC8D3DD), 0);
            lv_label_set_text(lvglOtaProgressLabel, "");

            lvglOtaUpdateBtn = lvglCreateMenuButton(wrap, lvglSymbolText(LV_SYMBOL_UPLOAD, "Update").c_str(), lv_color_hex(0x3A8F4B), lvglOtaUpdateEvent, nullptr);
            if (lvglOtaUpdateBtn) lvglOtaUpdateBtnLabel = lv_obj_get_child(lvglOtaUpdateBtn, 0);
            lvglRegisterReorderableItem(otaCurrentCard, "ord_ota", "cur");
            lvglRegisterReorderableItem(otaLatestCard, "ord_ota", "latest");
            lvglRegisterReorderableItem(otaStatusCard, "ord_ota", "status");
            lvglRegisterReorderableItem(lvglOtaProgressBar, "ord_ota", "bar");
            lvglRegisterReorderableItem(lvglOtaProgressLabel, "ord_ota", "pct");
            lvglRegisterReorderableItem(lvglOtaUpdateBtn, "ord_ota", "update");
            lvglApplySavedOrder(wrap, "ord_ota");
            lvglRefreshOtaUi();
            break;
        }
        case UI_CONFIG_HC12: {
            lvglScrHc12 = lvglCreateScreenBase("Radio Config", false);
            lv_obj_t *wrap = lv_obj_create(lvglScrHc12);
            lv_obj_set_size(wrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(wrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(wrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(wrap, 0, 0);
            lv_obj_set_style_pad_all(wrap, 8, 0);
            lv_obj_set_style_pad_row(wrap, 8, 0);
            lv_obj_set_flex_flow(wrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scroll_dir(wrap, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(wrap, LV_SCROLLBAR_MODE_OFF);

            auto makeSelectorRow = [&](const char *title,
                                       lv_color_t accent,
                                       lv_event_cb_t prevCb,
                                       lv_event_cb_t nextCb,
                                       lv_obj_t **titleOut,
                                       lv_obj_t **valueOut,
                                       lv_obj_t **subOut) -> lv_obj_t * {
                lv_obj_t *card = lv_obj_create(wrap);
                lv_obj_set_width(card, lv_pct(100));
                lv_obj_set_height(card, LV_SIZE_CONTENT);
                lv_obj_set_style_bg_color(card, lv_color_hex(0x16212C), 0);
                lv_obj_set_style_border_color(card, lv_color_hex(0x536274), 0);
                lv_obj_set_style_border_width(card, 1, 0);
                lv_obj_set_style_radius(card, 12, 0);
                lv_obj_set_style_pad_all(card, 10, 0);
                lv_obj_set_style_pad_row(card, 8, 0);
                lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
                lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

                lv_obj_t *titleLbl = lv_label_create(card);
                lv_label_set_text(titleLbl, title);
                lv_obj_set_style_text_color(titleLbl, lv_color_hex(0x9FB0C2), 0);
                if (titleOut) *titleOut = titleLbl;

                lv_obj_t *row = lv_obj_create(card);
                lv_obj_set_width(row, lv_pct(100));
                lv_obj_set_height(row, LV_SIZE_CONTENT);
                lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
                lv_obj_set_style_border_width(row, 0, 0);
                lv_obj_set_style_pad_all(row, 0, 0);
                lv_obj_set_style_pad_column(row, 8, 0);
                lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
                lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
                lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

                makeSmallBtn(row, LV_SYMBOL_LEFT, 44, 38, accent, prevCb, nullptr);

                lv_obj_t *center = lv_obj_create(row);
                lv_obj_set_flex_grow(center, 1);
                lv_obj_set_height(center, LV_SIZE_CONTENT);
                lv_obj_set_style_bg_color(center, lv_color_hex(0x101922), 0);
                lv_obj_set_style_border_color(center, lv_color_hex(0x2D495E), 0);
                lv_obj_set_style_border_width(center, 1, 0);
                lv_obj_set_style_radius(center, 10, 0);
                lv_obj_set_style_pad_all(center, 8, 0);
                lv_obj_set_style_pad_row(center, 2, 0);
                lv_obj_set_flex_flow(center, LV_FLEX_FLOW_COLUMN);
                lv_obj_set_flex_align(center, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
                lv_obj_clear_flag(center, LV_OBJ_FLAG_SCROLLABLE);

                lv_obj_t *valueLbl = lv_label_create(center);
                lv_obj_set_style_text_color(valueLbl, lv_color_hex(0xF5F8FB), 0);
                lv_label_set_text(valueLbl, "--");
                *valueOut = valueLbl;

                lv_obj_t *subLbl = lv_label_create(center);
                lv_obj_set_style_text_color(subLbl, lv_color_hex(0xA8BACB), 0);
                lv_label_set_text(subLbl, "");
                *subOut = subLbl;

                makeSmallBtn(row, LV_SYMBOL_RIGHT, 44, 38, accent, nextCb, nullptr);
                return card;
            };

            lvglRadioModuleHeader = lv_label_create(wrap);
            lv_label_set_text(lvglRadioModuleHeader, "Radio Module");
            lv_obj_set_style_text_color(lvglRadioModuleHeader, lv_color_hex(0xE5ECF3), 0);

            lvglRadioModuleDropdown = lv_dropdown_create(wrap);
            lv_obj_set_width(lvglRadioModuleDropdown, lv_pct(100));
            lv_dropdown_set_options(lvglRadioModuleDropdown, buildRadioModuleDropdownOptions().c_str());
            lv_obj_set_style_bg_color(lvglRadioModuleDropdown, lv_color_hex(0x111922), 0);
            lv_obj_set_style_text_color(lvglRadioModuleDropdown, lv_color_hex(0xE5ECF3), 0);
            lv_obj_set_style_border_color(lvglRadioModuleDropdown, lv_color_hex(0x2F4658), 0);
            lv_obj_set_style_border_width(lvglRadioModuleDropdown, 1, 0);
            lv_obj_set_style_radius(lvglRadioModuleDropdown, 8, 0);
            lv_obj_add_event_cb(lvglRadioModuleDropdown, lvglRadioModuleDropdownEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lv_obj_add_event_cb(lvglRadioModuleDropdown, lvglGestureBlockEvent, LV_EVENT_PRESSED, nullptr);
            lv_obj_add_event_cb(lvglRadioModuleDropdown, lvglGestureBlockEvent, LV_EVENT_RELEASED, nullptr);
            lv_obj_add_event_cb(lvglRadioModuleDropdown, lvglGestureBlockEvent, LV_EVENT_PRESS_LOST, nullptr);

            lv_obj_t *hc12ChannelCard = makeSelectorRow("Channel", lv_color_hex(0x7A5C2E), lvglHc12PrevChannelEvent, lvglHc12NextChannelEvent,
                                                        &lvglRadioChannelTitleLabel, &lvglHc12ChannelValueLabel, &lvglHc12ChannelSubLabel);
            lv_obj_t *hc12BaudCard = makeSelectorRow("Baud Rate", lv_color_hex(0x2F6D86), lvglHc12PrevBaudEvent, lvglHc12NextBaudEvent,
                                                     &lvglRadioBaudTitleLabel, &lvglHc12BaudValueLabel, &lvglHc12BaudSubLabel);
            lv_obj_t *hc12ModeCard = makeSelectorRow("Transmission Mode", lv_color_hex(0x355E8A), lvglHc12PrevModeEvent, lvglHc12NextModeEvent,
                                                     &lvglRadioModeTitleLabel, &lvglHc12ModeValueLabel, &lvglHc12ModeSubLabel);
            lv_obj_t *hc12PowerCard = makeSelectorRow("Transmission Power", lv_color_hex(0xA35757), lvglHc12PrevPowerEvent, lvglHc12NextPowerEvent,
                                                      &lvglRadioPowerTitleLabel, &lvglHc12PowerValueLabel, &lvglHc12PowerSubLabel);
            lvglRadioExtraCard = makeSelectorRow("Transfer Mode", lv_color_hex(0x5E7B35), lvglHc12PrevExtraEvent, lvglHc12NextExtraEvent,
                                                 &lvglRadioExtraTitleLabel, &lvglRadioExtraValueLabel, &lvglRadioExtraSubLabel);
            lvglRadioPinSwapCard = makeSelectorRow("UART Pins", lv_color_hex(0x4E6786), lvglRadioPrevPinSwapEvent, lvglRadioNextPinSwapEvent,
                                                   &lvglRadioPinSwapTitleLabel, &lvglRadioPinSwapValueLabel, &lvglRadioPinSwapSubLabel);
            lvglRadioModePinSwapCard = makeSelectorRow("Mode Pins", lv_color_hex(0x6A7D42), lvglRadioPrevModePinSwapEvent, lvglRadioNextModePinSwapEvent,
                                                       &lvglRadioModePinSwapTitleLabel, &lvglRadioModePinSwapValueLabel, &lvglRadioModePinSwapSubLabel);

            lv_obj_t *hc12DefaultBtn = lvglCreateMenuButton(wrap, lvglSymbolText(LV_SYMBOL_REFRESH, "Default").c_str(), lv_color_hex(0xA35757), lvglHc12DefaultEvent, nullptr);
            lvglHc12ConfigStatusLabel = lv_label_create(wrap);
            if (lvglHc12ConfigStatusLabel) {
                lv_obj_set_width(lvglHc12ConfigStatusLabel, lv_pct(100));
                lv_label_set_long_mode(lvglHc12ConfigStatusLabel, LV_LABEL_LONG_WRAP);
                lv_obj_set_style_text_align(lvglHc12ConfigStatusLabel, LV_TEXT_ALIGN_CENTER, 0);
                lv_obj_set_style_text_color(lvglHc12ConfigStatusLabel, lv_color_hex(0xA8BACB), 0);
            }

            lv_obj_t *hc12TerminalBtn = lvglCreateMenuButton(wrap, lvglSymbolText(LV_SYMBOL_KEYBOARD, "Radio Terminal").c_str(), lv_color_hex(0x2F6D86), lvglOpenHc12TerminalEvent, nullptr);
            lv_obj_t *hc12InfoBtn = lvglCreateMenuButton(wrap, lvglSymbolText(LV_SYMBOL_LIST, "Radio Info").c_str(), lv_color_hex(0x7A5C2E), lvglOpenHc12InfoEvent, nullptr);

            lv_obj_t *hintCard = lv_obj_create(wrap);
            lv_obj_set_width(hintCard, lv_pct(100));
            lv_obj_set_height(hintCard, LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(hintCard, lv_color_hex(0x16212C), 0);
            lv_obj_set_style_border_color(hintCard, lv_color_hex(0x536274), 0);
            lv_obj_set_style_border_width(hintCard, 1, 0);
            lv_obj_set_style_radius(hintCard, 12, 0);
            lv_obj_set_style_pad_all(hintCard, 10, 0);
            lv_obj_clear_flag(hintCard, LV_OBJ_FLAG_SCROLLABLE);

            lvglRadioHintLabel = lv_label_create(hintCard);
            lv_obj_set_width(lvglRadioHintLabel, lv_pct(100));
            lv_label_set_long_mode(lvglRadioHintLabel, LV_LABEL_LONG_WRAP);
            lv_obj_set_style_text_color(lvglRadioHintLabel, lv_color_hex(0xC8D3DD), 0);
            lv_label_set_text(lvglRadioHintLabel, "");
            if (lvglRadioModuleHeader) lvglRegisterReorderableItem(lvglRadioModuleHeader, "ord_hc12", "module_hdr");
            if (lvglRadioModuleDropdown) lvglRegisterReorderableItem(lvglRadioModuleDropdown, "ord_hc12", "module_dd");
            lvglRegisterReorderableItem(hc12ChannelCard, "ord_hc12", "chan");
            lvglRegisterReorderableItem(hc12BaudCard, "ord_hc12", "baud");
            lvglRegisterReorderableItem(hc12ModeCard, "ord_hc12", "mode");
            lvglRegisterReorderableItem(hc12PowerCard, "ord_hc12", "power");
            lvglRegisterReorderableItem(lvglRadioExtraCard, "ord_hc12", "extra");
            lvglRegisterReorderableItem(lvglRadioPinSwapCard, "ord_hc12", "pins");
            lvglRegisterReorderableItem(lvglRadioModePinSwapCard, "ord_hc12", "modepins");
            lvglRegisterReorderableItem(hc12DefaultBtn, "ord_hc12", "default");
            lvglRegisterReorderableItem(lvglHc12ConfigStatusLabel, "ord_hc12", "status");
            lvglRegisterReorderableItem(hc12TerminalBtn, "ord_hc12", "term");
            lvglRegisterReorderableItem(hc12InfoBtn, "ord_hc12", "info");
            lvglRegisterReorderableItem(hintCard, "ord_hc12", "hint");
            lvglApplySavedOrder(wrap, "ord_hc12");
            lvglRefreshHc12ConfigUi();
            break;
        }
        case UI_CONFIG_HC12_TERMINAL: {
            lvglScrHc12Terminal = lvglCreateScreenBase("Radio Terminal", false);
            lv_obj_t *wrap = lv_obj_create(lvglScrHc12Terminal);
            lv_obj_set_size(wrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(wrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_color(wrap, lv_color_hex(0x111922), 0);
            lv_obj_set_style_border_width(wrap, 0, 0);
            lv_obj_set_style_pad_all(wrap, 8, 0);
            lv_obj_set_style_pad_row(wrap, 8, 0);
            lv_obj_set_flex_flow(wrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scroll_dir(wrap, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(wrap, LV_SCROLLBAR_MODE_OFF);

            lv_obj_t *topRow = lv_obj_create(wrap);
            lv_obj_set_size(topRow, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(topRow, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(topRow, 0, 0);
            lv_obj_set_style_pad_all(topRow, 0, 0);
            lv_obj_set_style_pad_column(topRow, 8, 0);
            lv_obj_set_flex_flow(topRow, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(topRow, LV_OBJ_FLAG_SCROLLABLE);

            makeSmallBtn(topRow, "CFG: OFF", 96, 34, lv_color_hex(0x7A5C2E), lvglHc12ToggleSetEvent, nullptr);

            lvglHc12TerminalPinLabel = lv_label_create(topRow);
            lv_obj_set_flex_grow(lvglHc12TerminalPinLabel, 1);
            lv_obj_set_style_text_color(lvglHc12TerminalPinLabel, lv_color_hex(0xC9D7E3), 0);
            lv_label_set_text(lvglHc12TerminalPinLabel, "Radio serial pins");

            lv_obj_t *terminalTa = lv_textarea_create(wrap);
            lv_obj_set_width(terminalTa, lv_pct(100));
            lv_obj_set_height(terminalTa, UI_CONTENT_H - 108);
            lv_textarea_set_one_line(terminalTa, false);
            lv_textarea_set_placeholder_text(terminalTa, "HC-12 terminal output");
            lv_obj_set_style_bg_color(terminalTa, lv_color_hex(0x091118), 0);
            lv_obj_set_style_text_color(terminalTa, lv_color_hex(0x82F6A2), 0);
            lv_obj_set_style_border_color(terminalTa, lv_color_hex(0x2D495E), 0);
            lv_obj_set_style_border_width(terminalTa, 1, 0);
            lv_obj_clear_flag(terminalTa, LV_OBJ_FLAG_CLICKABLE);
            lv_textarea_set_cursor_click_pos(terminalTa, false);

            lv_obj_t *cmdRow = lv_obj_create(wrap);
            lv_obj_set_size(cmdRow, lv_pct(100), LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(cmdRow, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(cmdRow, 0, 0);
            lv_obj_set_style_pad_all(cmdRow, 0, 0);
            lv_obj_set_style_pad_column(cmdRow, 8, 0);
            lv_obj_set_flex_flow(cmdRow, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(cmdRow, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *cmdTa = lv_textarea_create(cmdRow);
            lv_obj_set_height(cmdTa, 36);
            lv_obj_set_width(cmdTa, lv_pct(100));
            lv_obj_set_flex_grow(cmdTa, 1);
            lv_textarea_set_one_line(cmdTa, true);
            lv_textarea_set_placeholder_text(cmdTa, "AT or text to send");
            lv_obj_add_event_cb(cmdTa, lvglTextAreaFocusEvent, LV_EVENT_FOCUSED, nullptr);

            makeSmallBtn(cmdRow, "Send", 70, 36, lv_color_hex(0x2F6D86), lvglHc12SendEvent, nullptr);
            lvglHc12TerminalExamplesLabel = lv_label_create(wrap);
            lv_obj_set_width(lvglHc12TerminalExamplesLabel, lv_pct(100));
            lv_label_set_long_mode(lvglHc12TerminalExamplesLabel, LV_LABEL_LONG_WRAP);
            lv_obj_set_style_text_color(lvglHc12TerminalExamplesLabel, lv_color_hex(0xA8BACB), 0);
            lv_label_set_text(lvglHc12TerminalExamplesLabel, "");
            lvglRegisterReorderableItem(topRow, "ord_hct", "top");
            lvglRegisterReorderableItem(terminalTa, "ord_hct", "term");
            lvglRegisterReorderableItem(cmdRow, "ord_hct", "cmd");
            lvglRegisterReorderableItem(lvglHc12TerminalExamplesLabel, "ord_hct", "examples");
            lvglApplySavedOrder(wrap, "ord_hct");
            hc12InitIfNeeded();
            lvglRefreshHc12Ui();
            break;
        }
        case UI_CONFIG_HC12_INFO: {
            lvglScrHc12Info = lvglCreateScreenBase("Radio Info", false);
            lv_obj_t *wrap = lv_obj_create(lvglScrHc12Info);
            lv_obj_set_size(wrap, lv_pct(100), UI_CONTENT_H);
            lv_obj_align(wrap, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
            lv_obj_set_style_bg_opa(wrap, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(wrap, 0, 0);
            lv_obj_set_style_pad_all(wrap, 8, 0);
            lv_obj_set_style_pad_row(wrap, 8, 0);
            lv_obj_set_flex_flow(wrap, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_scroll_dir(wrap, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(wrap, LV_SCROLLBAR_MODE_OFF);

            auto makeInfoLine = [&](const char *title, lv_obj_t **valueOut) -> lv_obj_t * {
                lv_obj_t *card = lv_obj_create(wrap);
                lv_obj_set_width(card, lv_pct(100));
                lv_obj_set_height(card, LV_SIZE_CONTENT);
                lv_obj_set_style_bg_color(card, lv_color_hex(0x16212C), 0);
                lv_obj_set_style_border_color(card, lv_color_hex(0x536274), 0);
                lv_obj_set_style_border_width(card, 1, 0);
                lv_obj_set_style_radius(card, 12, 0);
                lv_obj_set_style_pad_all(card, 10, 0);
                lv_obj_set_style_pad_row(card, 4, 0);
                lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
                lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_t *label = lv_label_create(card);
                lv_label_set_text(label, title);
                lv_obj_set_style_text_color(label, lv_color_hex(0x9FB0C2), 0);
                lv_obj_t *value = lv_label_create(card);
                lv_obj_set_width(value, lv_pct(100));
                lv_label_set_long_mode(value, LV_LABEL_LONG_WRAP);
                lv_label_set_text(value, "--");
                lv_obj_set_style_text_color(value, lv_color_hex(0xF5F8FB), 0);
                *valueOut = value;
                return card;
            };

            lv_obj_t *hc12InfoVersionCard = makeInfoLine("Version", &lvglHc12InfoVersionLabel);
            lv_obj_t *hc12InfoBaudCard = makeInfoLine("Baud Rate", &lvglHc12InfoBaudLabel);
            lv_obj_t *hc12InfoChannelCard = makeInfoLine("Channel", &lvglHc12InfoChannelLabel);
            lv_obj_t *hc12InfoFuCard = makeInfoLine("FU Mode", &lvglHc12InfoFuModeLabel);
            lv_obj_t *hc12InfoPowerCard = makeInfoLine("Power", &lvglHc12InfoPowerLabel);
            lv_obj_t *hc12InfoRawCard = makeInfoLine("Raw", &lvglHc12InfoRawLabel);
            lvglRegisterReorderableItem(hc12InfoVersionCard, "ord_hci", "ver");
            lvglRegisterReorderableItem(hc12InfoBaudCard, "ord_hci", "baud");
            lvglRegisterReorderableItem(hc12InfoChannelCard, "ord_hci", "chan");
            lvglRegisterReorderableItem(hc12InfoFuCard, "ord_hci", "fu");
            lvglRegisterReorderableItem(hc12InfoPowerCard, "ord_hci", "power");
            lvglRegisterReorderableItem(hc12InfoRawCard, "ord_hci", "raw");
            lvglApplySavedOrder(wrap, "ord_hci");
            break;
        }
        case UI_SCREENSAVER: {
            lvglScrScreensaver = lv_obj_create(nullptr);
            lv_obj_set_style_bg_color(lvglScrScreensaver, lv_color_hex(0x000000), 0);
            lv_obj_set_style_bg_opa(lvglScrScreensaver, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(lvglScrScreensaver, 0, 0);
            lv_obj_set_style_pad_all(lvglScrScreensaver, 0, 0);
            lv_obj_clear_flag(lvglScrScreensaver, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
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
            lv_obj_add_event_cb(lvglMqttPassShowBtn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
            lv_obj_add_event_cb(lvglMqttPassShowBtn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
            lv_obj_add_event_cb(lvglMqttPassShowBtn, lvglMqttPassShowToggleEvent, LV_EVENT_CLICKED, nullptr);
            lvglMqttPassShowBtnLabel = lv_label_create(lvglMqttPassShowBtn);
            lv_label_set_text(lvglMqttPassShowBtnLabel, LV_SYMBOL_EYE_CLOSE);
            lv_obj_center(lvglMqttPassShowBtnLabel);
            lvglRegisterStyledButton(lvglMqttPassShowBtn, lv_color_hex(0x3F4A57), true);

            lvglMqttDiscTa = addTa("Discovery prefix", false, false);
            lv_obj_t *actRow = lv_obj_create(formWrap);
            lv_obj_set_size(actRow, lv_pct(100), 40);
            lv_obj_set_style_bg_opa(actRow, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(actRow, 0, 0);
            lv_obj_set_style_pad_all(actRow, 0, 0);
            lv_obj_set_style_pad_column(actRow, 6, 0);
            lv_obj_set_flex_flow(actRow, LV_FLEX_FLOW_ROW);
            lv_obj_set_flex_align(actRow, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
            lv_obj_clear_flag(actRow, LV_OBJ_FLAG_SCROLLABLE);
            makeSmallBtn(actRow, lvglSymbolText(LV_SYMBOL_SAVE, "Save").c_str(), 76, 30, lv_color_hex(0x3A7A3A), lvglMqttSaveEvent);
            makeSmallBtn(actRow, lvglSymbolText(LV_SYMBOL_POWER, "Connect").c_str(), 92, 30, lv_color_hex(0x2F6D86), lvglMqttConnectEvent);
            makeSmallBtn(actRow, lvglSymbolText(LV_SYMBOL_WIFI, "Discover").c_str(), 102, 30, lv_color_hex(0x2F6D86), lvglMqttPublishDiscoveryEvent);
            lvglRegisterReorderableItem(enableRow, "ord_mcfg", "enable");
            lvglRegisterReorderableItem(lvglMqttBrokerTa, "ord_mcfg", "broker");
            lvglRegisterReorderableItem(lvglMqttPortTa, "ord_mcfg", "port");
            lvglRegisterReorderableItem(lvglMqttUserTa, "ord_mcfg", "user");
            lvglRegisterReorderableItem(mqttPwdRow, "ord_mcfg", "pass");
            lvglRegisterReorderableItem(lvglMqttDiscTa, "ord_mcfg", "disc");
            lvglRegisterReorderableItem(actRow, "ord_mcfg", "act");
            lvglApplySavedOrder(formWrap, "ord_mcfg");

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
            const lv_coord_t snakeControlsHeight = 108;
            const lv_coord_t snakeBoardTop = UI_CONTENT_TOP_Y + 4;
            const lv_coord_t snakeBoardWidth = DISPLAY_WIDTH - 10;
            const lv_coord_t snakeBoardHeight = UI_CONTENT_H - snakeControlsHeight - 10;
            const lv_coord_t snakePadBtn = (DISPLAY_WIDTH >= 460) ? 58 : 42;
            const lv_coord_t snakeDirBtnWidth = snakePadBtn * 2;
            const lv_coord_t snakeDirBtnHeight = snakePadBtn - 2;
            const lv_coord_t snakeSideOffsetRequested = snakePadBtn + 28;
            const lv_coord_t snakePauseBtnWidth = snakeDirBtnWidth;
            const lv_coord_t snakePauseBtnHeight = snakeDirBtnHeight;
            const lv_coord_t snakeSideOffsetMinNoOverlap = ((snakePauseBtnWidth + snakeDirBtnWidth) / 2) + 4;
            const lv_coord_t snakeSideOffsetMaxFit = (DISPLAY_WIDTH - snakeDirBtnWidth) / 2 - 6;
            const lv_coord_t snakeSideOffset = min<lv_coord_t>(snakeSideOffsetMaxFit,
                                                                max<lv_coord_t>(snakeSideOffsetRequested,
                                                                                snakeSideOffsetMinNoOverlap));

            lvglSnakeBoardObj = lv_obj_create(lvglScrSnake);
            if (lvglSnakeBoardObj) {
                lv_obj_set_size(lvglSnakeBoardObj, snakeBoardWidth, snakeBoardHeight);
                lv_obj_align(lvglSnakeBoardObj, LV_ALIGN_TOP_MID, 0, snakeBoardTop);
                lv_obj_set_style_bg_color(lvglSnakeBoardObj, lv_color_hex(0x0C1218), 0);
                lv_obj_set_style_border_width(lvglSnakeBoardObj, 0, 0);
                lv_obj_set_style_radius(lvglSnakeBoardObj, 8, 0);
                lv_obj_clear_flag(lvglSnakeBoardObj, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_add_event_cb(lvglSnakeBoardObj, lvglSnakeBoardDrawEvent, LV_EVENT_DRAW_MAIN, nullptr);

                lvglSnakeScoreLabel = lv_label_create(lvglSnakeBoardObj);
                if (lvglSnakeScoreLabel) {
                    lv_obj_align(lvglSnakeScoreLabel, LV_ALIGN_TOP_LEFT, 10, 8);
                    lv_obj_set_style_text_color(lvglSnakeScoreLabel, lv_color_hex(0xEAF2F9), 0);
                }
                lvglSnakeBestLabel = lv_label_create(lvglSnakeBoardObj);
                if (lvglSnakeBestLabel) {
                    lv_obj_align(lvglSnakeBestLabel, LV_ALIGN_TOP_RIGHT, -10, 8);
                    lv_obj_set_style_text_color(lvglSnakeBestLabel, lv_color_hex(0xBBD2E6), 0);
                }

                lvglSnakeOverlay = lv_obj_create(lvglSnakeBoardObj);
                if (lvglSnakeOverlay) {
                    lv_obj_set_size(lvglSnakeOverlay, min<lv_coord_t>(snakeBoardWidth - 16, 324), min<lv_coord_t>(snakeBoardHeight - 16, 467));
                    lv_obj_center(lvglSnakeOverlay);
                    lv_obj_set_style_bg_color(lvglSnakeOverlay, lv_color_hex(0x12202B), 0);
                    lv_obj_set_style_bg_opa(lvglSnakeOverlay, LV_OPA_90, 0);
                    lv_obj_set_style_border_width(lvglSnakeOverlay, 1, 0);
                    lv_obj_set_style_border_color(lvglSnakeOverlay, lv_color_hex(0x3C566B), 0);
                    lv_obj_set_style_radius(lvglSnakeOverlay, 14, 0);
                    lv_obj_set_style_pad_all(lvglSnakeOverlay, 12, 0);
                    lv_obj_set_style_pad_row(lvglSnakeOverlay, 10, 0);
                    lv_obj_set_flex_flow(lvglSnakeOverlay, LV_FLEX_FLOW_COLUMN);
                    lv_obj_set_flex_align(lvglSnakeOverlay, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
                    lv_obj_clear_flag(lvglSnakeOverlay, LV_OBJ_FLAG_SCROLLABLE);
                    lv_obj_move_foreground(lvglSnakeOverlay);

                    lvglSnakeOverlayTitle = lv_label_create(lvglSnakeOverlay);
                    if (lvglSnakeOverlayTitle) lv_obj_set_style_text_color(lvglSnakeOverlayTitle, lv_color_hex(0xF2F7FB), 0);
                    lvglSnakeOverlaySubLabel = lv_label_create(lvglSnakeOverlay);
                    if (lvglSnakeOverlaySubLabel) {
                        lv_obj_set_width(lvglSnakeOverlaySubLabel, lv_pct(100));
                        lv_label_set_long_mode(lvglSnakeOverlaySubLabel, LV_LABEL_LONG_WRAP);
                        lv_obj_set_style_text_align(lvglSnakeOverlaySubLabel, LV_TEXT_ALIGN_CENTER, 0);
                        lv_obj_set_style_text_color(lvglSnakeOverlaySubLabel, lv_color_hex(0xB7C8D7), 0);
                    }
                    lvglSnakeOverlayBtn = makeSmallBtn(lvglSnakeOverlay, "Start", 120, 42, lv_color_hex(0x3A8F4B), lvglSnakeRestartEvent);
                    if (lvglSnakeOverlayBtn) lvglSnakeOverlayBtnLabel = lv_obj_get_child(lvglSnakeOverlayBtn, 0);
                }
            }

            lv_obj_t *snakeCtl = lv_obj_create(lvglScrSnake);
            lv_obj_set_size(snakeCtl, lv_pct(100), snakeControlsHeight);
            lv_obj_align(snakeCtl, LV_ALIGN_BOTTOM_MID, 0, 0);
            lv_obj_set_style_bg_color(snakeCtl, lv_color_hex(0x101922), 0);
            lv_obj_set_style_bg_opa(snakeCtl, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(snakeCtl, 0, 0);
            lv_obj_set_style_radius(snakeCtl, 0, 0);
            lv_obj_set_style_pad_all(snakeCtl, 0, 0);
            lv_obj_clear_flag(snakeCtl, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *upBtn = makeSmallBtn(snakeCtl, LV_SYMBOL_UP, snakeDirBtnWidth, snakeDirBtnHeight, lv_color_hex(0x2F6D86), lvglSnakeDirEvent, reinterpret_cast<void *>(static_cast<intptr_t>(0)));
            if (upBtn) lv_obj_align(upBtn, LV_ALIGN_TOP_MID, 0, 4);
            lv_obj_t *leftBtn = makeSmallBtn(snakeCtl, LV_SYMBOL_LEFT, snakeDirBtnWidth, snakeDirBtnHeight, lv_color_hex(0x2F6D86), lvglSnakeDirEvent, reinterpret_cast<void *>(static_cast<intptr_t>(3)));
            if (leftBtn) lv_obj_align(leftBtn, LV_ALIGN_CENTER, -snakeSideOffset, 6);
            lv_obj_t *pauseBtn = makeSmallBtn(snakeCtl, LV_SYMBOL_PLAY, snakePauseBtnWidth, snakePauseBtnHeight, lv_color_hex(0x3A8F4B), lvglSnakePauseEvent, nullptr);
            if (pauseBtn) {
                lv_obj_align(pauseBtn, LV_ALIGN_TOP_MID, -snakeSideOffset, 4);
                lvglSnakePauseBtnLabel = lv_obj_get_child(pauseBtn, 0);
            }
            lv_obj_t *rightBtn = makeSmallBtn(snakeCtl, LV_SYMBOL_RIGHT, snakeDirBtnWidth, snakeDirBtnHeight, lv_color_hex(0x2F6D86), lvglSnakeDirEvent, reinterpret_cast<void *>(static_cast<intptr_t>(1)));
            if (rightBtn) lv_obj_align(rightBtn, LV_ALIGN_CENTER, snakeSideOffset, 6);
            lv_obj_t *downBtn = makeSmallBtn(snakeCtl, LV_SYMBOL_DOWN, snakeDirBtnWidth, snakeDirBtnHeight, lv_color_hex(0x2F6D86), lvglSnakeDirEvent, reinterpret_cast<void *>(static_cast<intptr_t>(2)));
            if (downBtn) lv_obj_align(downBtn, LV_ALIGN_BOTTOM_MID, 0, -4);
            snakePrepareGame();
            lvglRefreshSnakeBoard();
            break;
        }
        case UI_GAME_TETRIS: {
            lvglScrTetris = lvglCreateScreenBase("Tetris", false);
            const lv_coord_t tetrisControlsHeight = 108;
            const lv_coord_t tetrisBoardTop = UI_CONTENT_TOP_Y + 4;
            const lv_coord_t tetrisBoardWidth = DISPLAY_WIDTH - 10;
            const lv_coord_t tetrisBoardHeight = UI_CONTENT_H - tetrisControlsHeight - 10;
            const lv_coord_t tetrisPadBtn = (DISPLAY_WIDTH >= 460) ? 58 : 42;
            const lv_coord_t tetrisDirBtnWidth = tetrisPadBtn * 2;
            const lv_coord_t tetrisDirBtnHeight = tetrisPadBtn - 2;
            const lv_coord_t tetrisPauseBtnWidth = tetrisDirBtnWidth;
            const lv_coord_t tetrisPauseBtnHeight = max<lv_coord_t>(18, tetrisDirBtnHeight - 5);
            const lv_coord_t tetrisSideOffsetRequested = tetrisPadBtn + 28;
            const lv_coord_t tetrisSideOffsetMinNoOverlap = ((tetrisPauseBtnWidth + tetrisDirBtnWidth) / 2) + 4;
            const lv_coord_t tetrisSideOffsetMaxFit = (DISPLAY_WIDTH - tetrisDirBtnWidth) / 2 - 6;
            const lv_coord_t tetrisSideOffset = min<lv_coord_t>(tetrisSideOffsetMaxFit,
                                                                 max<lv_coord_t>(tetrisSideOffsetRequested,
                                                                                 tetrisSideOffsetMinNoOverlap));

            lvglTetrisBoardObj = lv_obj_create(lvglScrTetris);
            if (lvglTetrisBoardObj) {
                lv_obj_set_size(lvglTetrisBoardObj, tetrisBoardWidth, tetrisBoardHeight);
                lv_obj_align(lvglTetrisBoardObj, LV_ALIGN_TOP_MID, 0, tetrisBoardTop);
                lv_obj_set_style_bg_color(lvglTetrisBoardObj, lv_color_hex(0x0C1218), 0);
                lv_obj_set_style_border_width(lvglTetrisBoardObj, 0, 0);
                lv_obj_set_style_radius(lvglTetrisBoardObj, 8, 0);
                lv_obj_clear_flag(lvglTetrisBoardObj, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_add_event_cb(lvglTetrisBoardObj, lvglTetrisBoardDrawEvent, LV_EVENT_DRAW_MAIN, nullptr);

                lvglTetrisScoreLabel = lv_label_create(lvglTetrisBoardObj);
                if (lvglTetrisScoreLabel) {
                    lv_obj_align(lvglTetrisScoreLabel, LV_ALIGN_TOP_LEFT, 10, 8);
                    lv_obj_set_style_text_color(lvglTetrisScoreLabel, lv_color_hex(0xEAF2F9), 0);
                }
                lvglTetrisBestLabel = lv_label_create(lvglTetrisBoardObj);
                if (lvglTetrisBestLabel) {
                    lv_obj_align(lvglTetrisBestLabel, LV_ALIGN_TOP_RIGHT, -10, 8);
                    lv_obj_set_style_text_color(lvglTetrisBestLabel, lv_color_hex(0xBBD2E6), 0);
                }

                lvglTetrisOverlay = lv_obj_create(lvglScrTetris);
                if (lvglTetrisOverlay) {
                    lv_obj_set_size(lvglTetrisOverlay, min<lv_coord_t>(tetrisBoardWidth - 16, 324), min<lv_coord_t>(tetrisBoardHeight - 16, 467));
                    lv_obj_center(lvglTetrisOverlay);
                    lv_obj_set_style_bg_color(lvglTetrisOverlay, lv_color_hex(0x12202B), 0);
                    lv_obj_set_style_bg_opa(lvglTetrisOverlay, LV_OPA_90, 0);
                    lv_obj_set_style_border_width(lvglTetrisOverlay, 1, 0);
                    lv_obj_set_style_border_color(lvglTetrisOverlay, lv_color_hex(0x3C566B), 0);
                    lv_obj_set_style_radius(lvglTetrisOverlay, 14, 0);
                    lv_obj_set_style_pad_all(lvglTetrisOverlay, 12, 0);
                    lv_obj_set_style_pad_row(lvglTetrisOverlay, 10, 0);
                    lv_obj_set_flex_flow(lvglTetrisOverlay, LV_FLEX_FLOW_COLUMN);
                    lv_obj_set_flex_align(lvglTetrisOverlay, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
                    lv_obj_clear_flag(lvglTetrisOverlay, LV_OBJ_FLAG_SCROLLABLE);
                    lv_obj_move_foreground(lvglTetrisOverlay);

                    lvglTetrisOverlayTitle = lv_label_create(lvglTetrisOverlay);
                    if (lvglTetrisOverlayTitle) lv_obj_set_style_text_color(lvglTetrisOverlayTitle, lv_color_hex(0xF2F7FB), 0);
                    lvglTetrisOverlaySubLabel = lv_label_create(lvglTetrisOverlay);
                    if (lvglTetrisOverlaySubLabel) {
                        lv_obj_set_width(lvglTetrisOverlaySubLabel, lv_pct(100));
                        lv_label_set_long_mode(lvglTetrisOverlaySubLabel, LV_LABEL_LONG_WRAP);
                        lv_obj_set_style_text_align(lvglTetrisOverlaySubLabel, LV_TEXT_ALIGN_CENTER, 0);
                        lv_obj_set_style_text_color(lvglTetrisOverlaySubLabel, lv_color_hex(0xB7C8D7), 0);
                    }
                    lvglTetrisOverlayBtn = makeSmallBtn(lvglTetrisOverlay, "Start", 120, 42, lv_color_hex(0x376B93), lvglTetrisRestartEvent);
                    if (lvglTetrisOverlayBtn) lvglTetrisOverlayBtnLabel = lv_obj_get_child(lvglTetrisOverlayBtn, 0);
                }
            }

            lv_obj_t *tCtl = lv_obj_create(lvglScrTetris);
            lv_obj_set_size(tCtl, lv_pct(100), tetrisControlsHeight);
            lv_obj_align(tCtl, LV_ALIGN_BOTTOM_MID, 0, 0);
            lv_obj_set_style_bg_color(tCtl, lv_color_hex(0x101922), 0);
            lv_obj_set_style_bg_opa(tCtl, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(tCtl, 0, 0);
            lv_obj_set_style_radius(tCtl, 0, 0);
            lv_obj_set_style_pad_all(tCtl, 0, 0);
            lv_obj_clear_flag(tCtl, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_t *upBtn = makeSmallBtn(tCtl, LV_SYMBOL_UP, tetrisDirBtnWidth, tetrisDirBtnHeight, lv_color_hex(0x376B93), lvglTetrisRotateEvent);
            if (upBtn) lv_obj_align(upBtn, LV_ALIGN_TOP_MID, 0, 4);
            lv_obj_t *leftBtn = makeSmallBtn(tCtl, LV_SYMBOL_LEFT, tetrisDirBtnWidth, tetrisDirBtnHeight, lv_color_hex(0x2F6D86), lvglTetrisMoveLeftEvent);
            if (leftBtn) lv_obj_align(leftBtn, LV_ALIGN_CENTER, -tetrisSideOffset, 6);
            lv_obj_t *pauseBtn = makeSmallBtn(tCtl, LV_SYMBOL_PLAY, tetrisPauseBtnWidth, tetrisPauseBtnHeight, lv_color_hex(0x376B93), lvglTetrisPauseEvent, nullptr);
            if (pauseBtn) {
                lv_obj_align(pauseBtn, LV_ALIGN_TOP_MID, -tetrisSideOffset, 4);
                lvglTetrisPauseBtnLabel = lv_obj_get_child(pauseBtn, 0);
            }
            lv_obj_t *rightBtn = makeSmallBtn(tCtl, LV_SYMBOL_RIGHT, tetrisDirBtnWidth, tetrisDirBtnHeight, lv_color_hex(0x2F6D86), lvglTetrisMoveRightEvent);
            if (rightBtn) lv_obj_align(rightBtn, LV_ALIGN_CENTER, tetrisSideOffset, 6);
            lv_obj_t *downBtn = makeSmallBtn(tCtl, LV_SYMBOL_DOWN, tetrisDirBtnWidth, tetrisDirBtnHeight, lv_color_hex(0x8A5A25), lvglTetrisDropEvent);
            if (downBtn) lv_obj_align(downBtn, LV_ALIGN_BOTTOM_MID, 0, -4);
            tetrisPrepareGame();
            lvglRefreshTetrisBoard();
            break;
        }
        case UI_GAME_CHECKERS: {
            lvglScrCheckers = lvglCreateScreenBase("Checkers", false);
            const lv_coord_t checkersBoardTop = UI_CONTENT_TOP_Y + 4;
            const lv_coord_t checkersBoardWidth = DISPLAY_WIDTH - 10;
            const lv_coord_t checkersBoardHeight = UI_CONTENT_H - CHECKERS_ACTION_BAR_H - 10;

            lvglCheckersBoardObj = lv_obj_create(lvglScrCheckers);
            if (lvglCheckersBoardObj) {
                lv_obj_set_size(lvglCheckersBoardObj, checkersBoardWidth, checkersBoardHeight);
                lv_obj_align(lvglCheckersBoardObj, LV_ALIGN_TOP_MID, 0, checkersBoardTop);
                lv_obj_set_style_bg_color(lvglCheckersBoardObj, lv_color_hex(0x0C1218), 0);
                lv_obj_set_style_border_width(lvglCheckersBoardObj, 0, 0);
                lv_obj_set_style_radius(lvglCheckersBoardObj, 8, 0);
                lv_obj_clear_flag(lvglCheckersBoardObj, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_add_event_cb(lvglCheckersBoardObj, lvglCheckersBoardDrawEvent, LV_EVENT_DRAW_MAIN, nullptr);
                lv_obj_add_event_cb(lvglCheckersBoardObj, lvglCheckersBoardEvent, LV_EVENT_CLICKED, nullptr);

                lvglCheckersModeLabel = lv_label_create(lvglCheckersBoardObj);
                if (lvglCheckersModeLabel) {
                    lv_obj_align(lvglCheckersModeLabel, LV_ALIGN_TOP_LEFT, 10, 8);
                    lv_obj_set_style_text_color(lvglCheckersModeLabel, lv_color_hex(0xEAF2F9), 0);
                }
                lvglCheckersTurnLabel = lv_label_create(lvglCheckersBoardObj);
                if (lvglCheckersTurnLabel) {
                    lv_obj_align(lvglCheckersTurnLabel, LV_ALIGN_TOP_RIGHT, -10, 8);
                    lv_obj_set_style_text_color(lvglCheckersTurnLabel, lv_color_hex(0xBBD2E6), 0);
                }
            }

            lvglCheckersOverlay = lv_obj_create(lvglScrCheckers);
            if (lvglCheckersOverlay) {
                lv_obj_set_size(lvglCheckersOverlay, min<lv_coord_t>(DISPLAY_WIDTH - 40, 190), min<lv_coord_t>(UI_CONTENT_H - 28, 150));
                lv_obj_align(lvglCheckersOverlay, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y + 10);
                lv_obj_set_style_bg_color(lvglCheckersOverlay, lv_color_hex(0x12202B), 0);
                lv_obj_set_style_bg_opa(lvglCheckersOverlay, LV_OPA_90, 0);
                lv_obj_set_style_border_width(lvglCheckersOverlay, 1, 0);
                lv_obj_set_style_border_color(lvglCheckersOverlay, lv_color_hex(0x3C566B), 0);
                lv_obj_set_style_radius(lvglCheckersOverlay, 14, 0);
                lv_obj_set_style_pad_all(lvglCheckersOverlay, 12, 0);
                lv_obj_set_style_pad_row(lvglCheckersOverlay, 10, 0);
                lv_obj_set_flex_flow(lvglCheckersOverlay, LV_FLEX_FLOW_COLUMN);
                lv_obj_set_flex_align(lvglCheckersOverlay, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
                lv_obj_clear_flag(lvglCheckersOverlay, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_move_foreground(lvglCheckersOverlay);

                lvglCheckersOverlayTitle = lv_label_create(lvglCheckersOverlay);
                if (lvglCheckersOverlayTitle) lv_obj_set_style_text_color(lvglCheckersOverlayTitle, lv_color_hex(0xF2F7FB), 0);
                lvglCheckersOverlaySubLabel = lv_label_create(lvglCheckersOverlay);
                if (lvglCheckersOverlaySubLabel) {
                    lv_obj_set_width(lvglCheckersOverlaySubLabel, lv_pct(100));
                    lv_label_set_long_mode(lvglCheckersOverlaySubLabel, LV_LABEL_LONG_WRAP);
                    lv_obj_set_style_text_align(lvglCheckersOverlaySubLabel, LV_TEXT_ALIGN_CENTER, 0);
                    lv_obj_set_style_text_color(lvglCheckersOverlaySubLabel, lv_color_hex(0xB7C8D7), 0);
                }
                lvglCheckersOverlayBtn = makeSmallBtn(lvglCheckersOverlay, "Replay", 120, 42, lv_color_hex(0x8A5A25), lvglCheckersReplayEvent);
                if (lvglCheckersOverlayBtn) lvglCheckersOverlayBtnLabel = lv_obj_get_child(lvglCheckersOverlayBtn, 0);
            }

            lvglCheckersPeerPopup = lv_obj_create(lvglScrCheckers);
            if (lvglCheckersPeerPopup) {
                lv_obj_set_size(lvglCheckersPeerPopup, DISPLAY_WIDTH - 28, min<lv_coord_t>(UI_CONTENT_H - 40, 180));
                lv_obj_align(lvglCheckersPeerPopup, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y + 8);
                lv_obj_set_style_bg_color(lvglCheckersPeerPopup, lv_color_hex(0x16212C), 0);
                lv_obj_set_style_border_color(lvglCheckersPeerPopup, lv_color_hex(0x536274), 0);
                lv_obj_set_style_border_width(lvglCheckersPeerPopup, 1, 0);
                lv_obj_set_style_radius(lvglCheckersPeerPopup, 12, 0);
                lv_obj_set_style_pad_all(lvglCheckersPeerPopup, 8, 0);
                lv_obj_set_style_pad_row(lvglCheckersPeerPopup, 6, 0);
                lv_obj_set_flex_flow(lvglCheckersPeerPopup, LV_FLEX_FLOW_COLUMN);
                lv_obj_clear_flag(lvglCheckersPeerPopup, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_add_flag(lvglCheckersPeerPopup, LV_OBJ_FLAG_HIDDEN);

                lvglCheckersPeerPopupTitle = lv_label_create(lvglCheckersPeerPopup);
                if (lvglCheckersPeerPopupTitle) {
                    lv_label_set_text(lvglCheckersPeerPopupTitle, "Choose Contact");
                    lv_obj_set_style_text_color(lvglCheckersPeerPopupTitle, lv_color_hex(0xEAF2F9), 0);
                }

                lvglCheckersPeerPopupList = lv_obj_create(lvglCheckersPeerPopup);
                if (lvglCheckersPeerPopupList) {
                    lv_obj_set_size(lvglCheckersPeerPopupList, lv_pct(100), min<lv_coord_t>(110, UI_CONTENT_H - 90));
                    lv_obj_set_style_bg_opa(lvglCheckersPeerPopupList, LV_OPA_TRANSP, 0);
                    lv_obj_set_style_border_width(lvglCheckersPeerPopupList, 0, 0);
                    lv_obj_set_style_pad_all(lvglCheckersPeerPopupList, 0, 0);
                    lv_obj_set_style_pad_row(lvglCheckersPeerPopupList, 6, 0);
                    lv_obj_set_flex_flow(lvglCheckersPeerPopupList, LV_FLEX_FLOW_COLUMN);
                    lv_obj_set_scroll_dir(lvglCheckersPeerPopupList, LV_DIR_VER);
                    lv_obj_set_scrollbar_mode(lvglCheckersPeerPopupList, LV_SCROLLBAR_MODE_OFF);
                }
            }

            lvglCheckersVariantPopup = lv_obj_create(lvglScrCheckers);
            if (lvglCheckersVariantPopup) {
                lv_obj_set_size(lvglCheckersVariantPopup, DISPLAY_WIDTH - 28, min<lv_coord_t>(UI_CONTENT_H - 40, 220));
                lv_obj_align(lvglCheckersVariantPopup, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y + 8);
                lv_obj_set_style_bg_color(lvglCheckersVariantPopup, lv_color_hex(0x16212C), 0);
                lv_obj_set_style_border_color(lvglCheckersVariantPopup, lv_color_hex(0x536274), 0);
                lv_obj_set_style_border_width(lvglCheckersVariantPopup, 1, 0);
                lv_obj_set_style_radius(lvglCheckersVariantPopup, 12, 0);
                lv_obj_set_style_pad_all(lvglCheckersVariantPopup, 8, 0);
                lv_obj_set_style_pad_row(lvglCheckersVariantPopup, 6, 0);
                lv_obj_set_flex_flow(lvglCheckersVariantPopup, LV_FLEX_FLOW_COLUMN);
                lv_obj_clear_flag(lvglCheckersVariantPopup, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_add_flag(lvglCheckersVariantPopup, LV_OBJ_FLAG_HIDDEN);

                lvglCheckersVariantPopupTitle = lv_label_create(lvglCheckersVariantPopup);
                if (lvglCheckersVariantPopupTitle) {
                    lv_label_set_text(lvglCheckersVariantPopupTitle, "Choose Rule Set");
                    lv_obj_set_style_text_color(lvglCheckersVariantPopupTitle, lv_color_hex(0xEAF2F9), 0);
                }

                lvglCheckersVariantPopupList = lv_obj_create(lvglCheckersVariantPopup);
                if (lvglCheckersVariantPopupList) {
                    lv_obj_set_size(lvglCheckersVariantPopupList, lv_pct(100), min<lv_coord_t>(148, UI_CONTENT_H - 110));
                    lv_obj_set_style_bg_opa(lvglCheckersVariantPopupList, LV_OPA_TRANSP, 0);
                    lv_obj_set_style_border_width(lvglCheckersVariantPopupList, 0, 0);
                    lv_obj_set_style_pad_all(lvglCheckersVariantPopupList, 0, 0);
                    lv_obj_set_style_pad_row(lvglCheckersVariantPopupList, 6, 0);
                    lv_obj_set_flex_flow(lvglCheckersVariantPopupList, LV_FLEX_FLOW_COLUMN);
                    lv_obj_set_scroll_dir(lvglCheckersVariantPopupList, LV_DIR_VER);
                    lv_obj_set_scrollbar_mode(lvglCheckersVariantPopupList, LV_SCROLLBAR_MODE_OFF);
                }
            }

            lv_obj_t *checkersBar = lv_obj_create(lvglScrCheckers);
            lv_obj_set_size(checkersBar, lv_pct(100), CHECKERS_ACTION_BAR_H);
            lv_obj_align(checkersBar, LV_ALIGN_BOTTOM_MID, 0, 0);
            lv_obj_set_style_bg_color(checkersBar, lv_color_hex(0x101922), 0);
            lv_obj_set_style_bg_opa(checkersBar, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(checkersBar, 0, 0);
            lv_obj_set_style_radius(checkersBar, 0, 0);
            lv_obj_set_style_pad_all(checkersBar, 6, 0);
            lv_obj_set_style_pad_column(checkersBar, 6, 0);
            lv_obj_set_flex_flow(checkersBar, LV_FLEX_FLOW_ROW);
            lv_obj_set_flex_align(checkersBar, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
            lv_obj_clear_flag(checkersBar, LV_OBJ_FLAG_SCROLLABLE);
            const lv_coord_t ctlW = max<lv_coord_t>(66, (DISPLAY_WIDTH - 28) / 3);
            makeSmallBtn(checkersBar, "ESP32", ctlW, 32, lv_color_hex(0x8A5A25), lvglCheckersEsp32Event, nullptr);
            makeSmallBtn(checkersBar, "Tag MP", ctlW, 32, lv_color_hex(0x2F6D86), lvglCheckersTagEvent, nullptr);
            makeSmallBtn(checkersBar, "Back", ctlW, 32, lv_color_hex(0x4E5D6C), lvglCheckersBackEvent, nullptr);

            checkersMode = CHECKERS_MODE_IDLE;
            checkersStarted = false;
            checkersGameOver = false;
            checkersPeerPopupOpen = false;
            checkersVariantPopupOpen = true;
            checkersWinnerSide = 0;
            checkersClearSelection();
            checkersClearSession();
            lvglRefreshCheckersBoard();
            break;
        }
#if defined(BOARD_ESP32S3_3248S035_N16R8)
        case UI_GAME_SNAKE3D: {
            lvglScrSnake3d = lvglCreateScreenBase("Snake 3D", false);
            const lv_coord_t snake3dControlsHeight = 138;
            const lv_coord_t snake3dBoardTop = UI_CONTENT_TOP_Y + 4;
            const lv_coord_t snake3dBoardWidth = DISPLAY_WIDTH - 10;
            const lv_coord_t snake3dBoardHeight = UI_CONTENT_H - snake3dControlsHeight - 10;
            const lv_coord_t snake3dPadBtn = (DISPLAY_WIDTH >= 460) ? 54 : 40;

            lvglSnake3dBoardObj = lv_obj_create(lvglScrSnake3d);
            if (lvglSnake3dBoardObj) {
                lv_obj_set_size(lvglSnake3dBoardObj, snake3dBoardWidth, snake3dBoardHeight);
                lv_obj_align(lvglSnake3dBoardObj, LV_ALIGN_TOP_MID, 0, snake3dBoardTop);
                lv_obj_set_style_bg_color(lvglSnake3dBoardObj, lv_color_hex(0x07111A), 0);
                lv_obj_set_style_border_width(lvglSnake3dBoardObj, 0, 0);
                lv_obj_set_style_radius(lvglSnake3dBoardObj, 8, 0);
                lv_obj_clear_flag(lvglSnake3dBoardObj, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_add_event_cb(lvglSnake3dBoardObj, lvglSnake3dBoardDrawEvent, LV_EVENT_DRAW_MAIN, nullptr);

                lvglSnake3dScoreLabel = lv_label_create(lvglSnake3dBoardObj);
                if (lvglSnake3dScoreLabel) {
                    lv_obj_align(lvglSnake3dScoreLabel, LV_ALIGN_TOP_LEFT, 10, 8);
                    lv_obj_set_style_text_color(lvglSnake3dScoreLabel, lv_color_hex(0xEAF2F9), 0);
                }
                lvglSnake3dBestLabel = lv_label_create(lvglSnake3dBoardObj);
                if (lvglSnake3dBestLabel) {
                    lv_obj_align(lvglSnake3dBestLabel, LV_ALIGN_TOP_RIGHT, -10, 8);
                    lv_obj_set_style_text_color(lvglSnake3dBestLabel, lv_color_hex(0xBBD2E6), 0);
                }

                lvglSnake3dOverlay = lv_obj_create(lvglSnake3dBoardObj);
                if (lvglSnake3dOverlay) {
                    lv_obj_set_size(lvglSnake3dOverlay, min<lv_coord_t>(snake3dBoardWidth - 16, 250), min<lv_coord_t>(snake3dBoardHeight - 16, 255));
                    lv_obj_center(lvglSnake3dOverlay);
                    lv_obj_set_style_bg_color(lvglSnake3dOverlay, lv_color_hex(0x102030), 0);
                    lv_obj_set_style_bg_opa(lvglSnake3dOverlay, LV_OPA_90, 0);
                    lv_obj_set_style_border_width(lvglSnake3dOverlay, 1, 0);
                    lv_obj_set_style_border_color(lvglSnake3dOverlay, lv_color_hex(0x48627A), 0);
                    lv_obj_set_style_radius(lvglSnake3dOverlay, 14, 0);
                    lv_obj_set_style_pad_all(lvglSnake3dOverlay, 12, 0);
                    lv_obj_set_style_pad_row(lvglSnake3dOverlay, 10, 0);
                    lv_obj_set_flex_flow(lvglSnake3dOverlay, LV_FLEX_FLOW_COLUMN);
                    lv_obj_set_flex_align(lvglSnake3dOverlay, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
                    lv_obj_clear_flag(lvglSnake3dOverlay, LV_OBJ_FLAG_SCROLLABLE);

                    lvglSnake3dOverlayTitle = lv_label_create(lvglSnake3dOverlay);
                    if (lvglSnake3dOverlayTitle) lv_obj_set_style_text_color(lvglSnake3dOverlayTitle, lv_color_hex(0xF4F7FB), 0);
                    lvglSnake3dOverlaySubLabel = lv_label_create(lvglSnake3dOverlay);
                    if (lvglSnake3dOverlaySubLabel) {
                        lv_obj_set_width(lvglSnake3dOverlaySubLabel, lv_pct(100));
                        lv_label_set_long_mode(lvglSnake3dOverlaySubLabel, LV_LABEL_LONG_WRAP);
                        lv_obj_set_style_text_align(lvglSnake3dOverlaySubLabel, LV_TEXT_ALIGN_CENTER, 0);
                        lv_obj_set_style_text_color(lvglSnake3dOverlaySubLabel, lv_color_hex(0xB7C8D7), 0);
                    }
                    lvglSnake3dOverlayBtn = makeSmallBtn(lvglSnake3dOverlay, "Start", 128, 42, lv_color_hex(0x5A4CC7), lvglSnake3dRestartEvent);
                    if (lvglSnake3dOverlayBtn) lvglSnake3dOverlayBtnLabel = lv_obj_get_child(lvglSnake3dOverlayBtn, 0);
                }
            }

            lv_obj_t *snake3dCtl = lv_obj_create(lvglScrSnake3d);
            lv_obj_set_size(snake3dCtl, lv_pct(100), snake3dControlsHeight);
            lv_obj_align(snake3dCtl, LV_ALIGN_BOTTOM_MID, 0, 0);
            lv_obj_set_style_bg_color(snake3dCtl, lv_color_hex(0x101922), 0);
            lv_obj_set_style_bg_opa(snake3dCtl, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(snake3dCtl, 0, 0);
            lv_obj_set_style_radius(snake3dCtl, 0, 0);
            lv_obj_set_style_pad_all(snake3dCtl, 0, 0);
            lv_obj_clear_flag(snake3dCtl, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *upBtn = makeSmallBtn(snake3dCtl, LV_SYMBOL_UP, snake3dPadBtn * 2, snake3dPadBtn - 2, lv_color_hex(0x2F6D86), lvglSnake3dDirEvent, reinterpret_cast<void *>(static_cast<intptr_t>(0)));
            if (upBtn) lv_obj_align(upBtn, LV_ALIGN_TOP_MID, 0, 4);
            lv_obj_t *leftBtn = makeSmallBtn(snake3dCtl, LV_SYMBOL_LEFT, snake3dPadBtn * 2, snake3dPadBtn - 2, lv_color_hex(0x2F6D86), lvglSnake3dDirEvent, reinterpret_cast<void *>(static_cast<intptr_t>(3)));
            if (leftBtn) lv_obj_align(leftBtn, LV_ALIGN_CENTER, -(snake3dPadBtn + 34), 8);
            lv_obj_t *pauseBtn = makeSmallBtn(snake3dCtl, LV_SYMBOL_PLAY, snake3dPadBtn * 2, snake3dPadBtn - 2, lv_color_hex(0x5A4CC7), lvglSnake3dPauseEvent, nullptr);
            if (pauseBtn) {
                lv_obj_align(pauseBtn, LV_ALIGN_CENTER, 0, 8);
                lvglSnake3dPauseBtnLabel = lv_obj_get_child(pauseBtn, 0);
            }
            lv_obj_t *rightBtn = makeSmallBtn(snake3dCtl, LV_SYMBOL_RIGHT, snake3dPadBtn * 2, snake3dPadBtn - 2, lv_color_hex(0x2F6D86), lvglSnake3dDirEvent, reinterpret_cast<void *>(static_cast<intptr_t>(1)));
            if (rightBtn) lv_obj_align(rightBtn, LV_ALIGN_CENTER, snake3dPadBtn + 34, 8);
            lv_obj_t *downBtn = makeSmallBtn(snake3dCtl, LV_SYMBOL_DOWN, snake3dPadBtn * 2, snake3dPadBtn - 2, lv_color_hex(0x2F6D86), lvglSnake3dDirEvent, reinterpret_cast<void *>(static_cast<intptr_t>(2)));
            if (downBtn) lv_obj_align(downBtn, LV_ALIGN_BOTTOM_MID, 0, -6);
            lv_obj_t *ascendBtn = makeSmallBtn(snake3dCtl, "Z+", snake3dPadBtn * 2, 28, lv_color_hex(0x355C9A), lvglSnake3dDirEvent, reinterpret_cast<void *>(static_cast<intptr_t>(4)));
            if (ascendBtn) lv_obj_align(ascendBtn, LV_ALIGN_TOP_LEFT, 8, 8);
            lv_obj_t *descendBtn = makeSmallBtn(snake3dCtl, "Z-", snake3dPadBtn * 2, 28, lv_color_hex(0x6C4CA0), lvglSnake3dDirEvent, reinterpret_cast<void *>(static_cast<intptr_t>(5)));
            if (descendBtn) lv_obj_align(descendBtn, LV_ALIGN_TOP_RIGHT, -8, 8);

            snake3dPrepareGame();
            lvglRefreshSnake3dBoard();
            break;
        }
#endif
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
        if (currentChatPeerKey.isEmpty()) chatClearCache();
        lvglRefreshChatLayout();
        lvglRefreshChatContactsUi();
        lvglRefreshChatUi();
    } else if (screen == UI_CHAT_PEERS) {
        lvglRefreshChatPeerUi();
    } else if (screen == UI_MEDIA) {
        lvglQueueMediaRefresh();
    } else if (screen == UI_INFO) {
        hc12RefreshInfoSnapshot();
        lvglRefreshInfoPanel();
    } else if (screen == UI_CONFIG) {
        lvglRefreshConfigUi();
    } else if (screen == UI_CONFIG_BATTERY) {
        lvglRefreshBatteryTrainUi();
    } else if (screen == UI_CONFIG_STYLE) {
        lvglRefreshStyleUi();
    } else if (screen == UI_CONFIG_LANGUAGE) {
        lvglRefreshLanguageUi();
    } else if (screen == UI_CONFIG_OTA) {
        lvglRefreshOtaUi();
    } else if (screen == UI_CONFIG_HC12) {
        hc12ReadConfigSelection();
        lvglRefreshHc12ConfigUi();
    } else if (screen == UI_CONFIG_HC12_TERMINAL) {
        lvglRefreshHc12Ui();
    } else if (screen == UI_CONFIG_HC12_INFO) {
        lvglRefreshHc12InfoUi();
    }
    lvglRefreshTopIndicators();
    lvglLoadScreen(target, anim);
}

void lvglNavigateBackBySwipe(lv_scr_load_anim_t anim)
{
    const UiScreen prev = uiScreen;
    if (!lvglReady) return;
    switch (uiScreen) {
        case UI_CHAT:
            if (!currentChatPeerKey.isEmpty()) {
                lvglHideChatMenu();
                currentChatPeerKey = "";
                chatClearCache();
                lvglRefreshChatLayout();
                lvglRefreshChatContactsUi();
                lvglRefreshChatUi();
            } else {
                lvglOpenScreen(UI_HOME, anim);
            }
            break;
        case UI_MEDIA:
        case UI_INFO:
        case UI_GAMES:
        case UI_CONFIG:
            lvglOpenScreen(UI_HOME, anim);
            break;
        case UI_CHAT_PEERS:
            lvglOpenScreen(UI_CHAT, anim);
            break;
        case UI_WIFI_LIST:
        case UI_CONFIG_BATTERY:
            lvglOpenScreen(UI_CONFIG, anim);
            break;
        case UI_CONFIG_STYLE:
        case UI_CONFIG_LANGUAGE:
            lvglOpenScreen(UI_CONFIG, anim);
            break;
        case UI_CONFIG_OTA:
        case UI_CONFIG_HC12:
            lvglOpenScreen(UI_CONFIG, anim);
            break;
        case UI_CONFIG_HC12_TERMINAL:
        case UI_CONFIG_HC12_INFO:
            lvglOpenScreen(UI_CONFIG_HC12, anim);
            break;
        case UI_GAME_SNAKE:
        case UI_GAME_TETRIS:
        case UI_GAME_CHECKERS:
        case UI_GAME_SNAKE3D:
            lvglOpenScreen(UI_GAMES, anim);
            break;
        case UI_CONFIG_MQTT_CONFIG:
        case UI_CONFIG_MQTT_CONTROLS:
            lvglOpenScreen(UI_CONFIG, anim);
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
    if (target == UI_CHAT) {
        lvglHideChatMenu();
        currentChatPeerKey = "";
        chatClearCache();
    }
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
    lvglApplyMsgboxModalStyle(m);
    lv_obj_center(m);
    lv_obj_set_width(m, min<int16_t>(DISPLAY_WIDTH - 24, 260));
    lv_obj_add_event_cb(m, lvglChatAirplanePromptEvent, LV_EVENT_VALUE_CHANGED, nullptr);
}

void lvglRadioModeWarningEvent(lv_event_t *e)
{
    lv_obj_t *msgbox = lv_event_get_current_target(e);
    lv_msgbox_close(msgbox);
}

static void lvglShowE220FixedModeWarning()
{
    static const char *btns[] = {"OK", ""};
    lv_obj_t *m = lv_msgbox_create(nullptr,
                                   "E220 Fixed Mode",
                                   "E220 Fixed mode disables current radio chat and discovery.\nUse Transparent mode for encrypted radio messaging.",
                                   btns,
                                   false);
    if (!m) return;
    lvglApplyMsgboxModalStyle(m);
    lv_obj_center(m);
    lv_obj_set_width(m, min<int16_t>(DISPLAY_WIDTH - 24, 270));
    lv_obj_add_event_cb(m, lvglRadioModeWarningEvent, LV_EVENT_VALUE_CHANGED, nullptr);
}

static void powerOffSignalPulse()
{
    if (POWER_OFF_SIGNAL_PIN < 0) return;
    pinMode(POWER_OFF_SIGNAL_PIN, OUTPUT);
    digitalWrite(POWER_OFF_SIGNAL_PIN, HIGH);
    delay(10);
    digitalWrite(POWER_OFF_SIGNAL_PIN, LOW);
    delay(POWER_OFF_PULSE_MS);
    digitalWrite(POWER_OFF_SIGNAL_PIN, HIGH);
    delay(POWER_OFF_GAP_MS);
    digitalWrite(POWER_OFF_SIGNAL_PIN, LOW);
    delay(POWER_OFF_PULSE_MS);
    digitalWrite(POWER_OFF_SIGNAL_PIN, HIGH);
}

void lvglPowerConfirmEvent(lv_event_t *e)
{
    lv_obj_t *msgbox = lv_event_get_current_target(e);
    const char *txt = lv_msgbox_get_active_btn_text(msgbox);
    lv_msgbox_close(msgbox);
    if (!txt) return;
    if (strcmp(txt, "Power Off") != 0) return;
    uiStatusLine = "Power-off signal sent";
    if (lvglReady) lvglSyncStatusLine();
    powerOffSignalPulse();
}

void lvglPowerButtonEvent(lv_event_t *e)
{
    (void)e;
    if (POWER_OFF_SIGNAL_PIN < 0) {
        lvglStatusPush("Power signal unavailable");
        return;
    }
    static const char *btns[] = {"Cancel", "Power Off", ""};
    lv_obj_t *m = lv_msgbox_create(nullptr,
                                   "Power Off",
                                   "Send power-off signal on GPIO21?",
                                   btns,
                                   false);
    if (!m) return;
    lvglApplyMsgboxModalStyle(m);
    lv_obj_center(m);
    lv_obj_set_width(m, min<int16_t>(DISPLAY_WIDTH - 24, 270));
    lv_obj_add_event_cb(m, lvglPowerConfirmEvent, LV_EVENT_VALUE_CHANGED, nullptr);
    lv_obj_t *btnMatrix = lv_msgbox_get_btns(m);
    if (btnMatrix) {
        lv_btnmatrix_set_btn_ctrl(btnMatrix, 1, LV_BTNMATRIX_CTRL_CHECKABLE);
        lv_obj_set_style_bg_color(btnMatrix, lv_color_hex(0xA33F3F), LV_PART_ITEMS | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(btnMatrix, lv_color_hex(0x7F2D2D), LV_PART_ITEMS | LV_STATE_PRESSED);
    }
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

void lvglChatDiscoveryToggleEvent(lv_event_t *e)
{
    (void)e;
    p2pDiscoveryEnabled = !p2pDiscoveryEnabled;
    saveP2pConfig();
    lvglApplyChatDiscoveryButtonStyle();
    uiStatusLine = p2pDiscoveryEnabled ? "Peer discovery enabled" : "Peer discovery disabled";
    lvglSyncStatusLine();
}

void lvglApModeEvent(lv_event_t *e)
{
    (void)e;
    if (airplaneModeEnabled) {
        uiStatusLine = "Disable airplane mode first";
        lvglSyncStatusLine();
        return;
    }
    if (wifiSessionApMode) {
        wifiSessionApMode = false;
        disableApWhenStaConnected("ui_ap_mode_off");
        if (wifiHasStaTarget()) {
            beginStaConnectAttempt("ui_ap_mode_off");
            uiStatusLine = "Trying saved WiFi";
        } else {
            uiStatusLine = "No saved WiFi";
        }
    } else {
        forceSessionApMode("ui_ap_mode");
        uiStatusLine = "AP mode enabled";
    }
    lvglSyncStatusLine();
    lvglRefreshConfigUi();
    lvglRefreshAllButtonStyles();
    lvglRefreshWifiList();
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
    wifiForgetPendingUi = false;
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
    wifiForgetPendingUi = true;
    pendingSaveCreds = false;
    pendingSaveSsid = "";
    pendingSavePass = "";
    clearStaCreds();
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

void lvglWifiWebServerToggleEvent(lv_event_t *e)
{
    (void)e;
    webServerEnabled = !webServerEnabled;
    persistWebServerEnabled();
    if (webServerEnabled) {
        ensureWebServerRuntime();
        uiStatusLine = "Web server enabled";
    } else {
        stopWebServerRuntime();
        uiStatusLine = "Web server disabled";
    }
    if (lvglReady) {
        lvglSyncStatusLine();
        lvglRefreshWifiList();
        lvglRefreshTopIndicators();
        if (uiScreen == UI_INFO) lvglRefreshInfoPanel();
    }
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
    mediaEnsureStorageReadyForUi();
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
    saveSoundPrefs();
    if (audioBackendReady && audio) audioSetVolumeImmediate(audioVolumeLevelFromPercent(mediaVolumePercent));
    lvglRefreshMediaPlayerUi();
    lvglRefreshConfigUi();
    lvglRefreshSoundPopupUi();
    lvglTopIndicatorStateValid = false;
    lvglRefreshTopIndicators();
}

void lvglMediaVolumeStepEvent(lv_event_t *e)
{
    const int step = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    int v = static_cast<int>(mediaVolumePercent) + step;
    if (v < 0) v = 0;
    if (v > 100) v = 100;
    mediaVolumePercent = static_cast<uint8_t>(v);
    if (lvglMediaVolSlider) lv_slider_set_value(lvglMediaVolSlider, static_cast<int32_t>(mediaVolumePercent), LV_ANIM_OFF);
    saveSoundPrefs();
    if (audioBackendReady && audio) audioSetVolumeImmediate(audioVolumeLevelFromPercent(mediaVolumePercent));
    lvglRefreshMediaPlayerUi();
    lvglRefreshConfigUi();
    lvglRefreshSoundPopupUi();
    lvglTopIndicatorStateValid = false;
    lvglRefreshTopIndicators();
}

void lvglRefreshMediaPlayerUi()
{
    if (!lvglMediaTrackLabel || !lvglMediaPlayBtnLabel || !lvglMediaProgressBar || !lvglMediaProgressLabel) return;
    if (uiScreen != UI_MEDIA) return;

    if (mediaSelectedTrackName.isEmpty() && !mediaNowPlaying.isEmpty()) mediaSelectedTrackName = mediaNowPlaying;
    const bool hasTrack = !mediaSelectedSourcePath.isEmpty();
    lvglSetMediaPlayerVisible(hasTrack);

    String trackText = hasTrack ? mediaSelectedTrackName : String("No track selected");
    if (trackText.length() > 34) trackText = trackText.substring(0, 31) + "...";
    lv_label_set_text_fmt(lvglMediaTrackLabel, "Track: %s", trackText.c_str());

    lv_label_set_text(lvglMediaPlayBtnLabel, lvglSymbolText((mediaIsPlaying || mediaPaused) ? LV_SYMBOL_STOP : LV_SYMBOL_PLAY,
                                                            (mediaIsPlaying || mediaPaused) ? "Stop" : "Play").c_str());

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
        String checkersInviteSessionId;
        const bool checkersInvite = checkersParseInviteText(chatMessages[i].text, checkersInviteSessionId);
        const String bodyText = checkersVisibleBodyFromRawText(chatMessages[i].text);
        if (bodyText.isEmpty()) continue;
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
            lv_label_set_text(statusLbl, pendingDelivery ? LV_SYMBOL_UPLOAD : LV_SYMBOL_OK);
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
        lv_label_set_text(body, bodyText.c_str());
        lv_obj_set_style_text_color(body, lv_color_hex(0xEDF2F7), 0);

        if (checkersInvite) {
            if (!chatMessages[i].outgoing) {
                lv_obj_t *playBtn = lv_btn_create(bubble);
                lv_obj_set_width(playBtn, lv_pct(100));
                lv_obj_set_height(playBtn, 28);
                lv_obj_set_style_radius(playBtn, 10, 0);
                lv_obj_set_style_border_width(playBtn, 0, 0);
                lv_obj_clear_flag(playBtn, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_add_event_cb(playBtn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
                lv_obj_add_event_cb(playBtn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
                lv_obj_add_event_cb(playBtn, lvglCheckersInvitePlayEvent, LV_EVENT_CLICKED, reinterpret_cast<void *>(static_cast<intptr_t>(i)));
                lv_obj_t *playLbl = lv_label_create(playBtn);
                lv_label_set_text(playLbl, "Play");
                lv_obj_center(playLbl);
                lvglRegisterStyledButton(playBtn, lv_color_hex(0x3A8F4B), true);
            } else {
                lv_obj_t *inviteState = lv_label_create(bubble);
                const bool activeInvite = (checkersMode == CHECKERS_MODE_TAG) &&
                                          (checkersSessionId == checkersInviteSessionId) &&
                                          (checkersPeerKey == currentChatPeerKey);
                if (activeInvite) {
                    lv_label_set_text(inviteState, checkersWaitingForRemote ? "Pending other player..." : "Game started");
                } else {
                    lv_label_set_text(inviteState, deliveredOutgoing ? "Invite sent" : "Sending invite...");
                }
                lv_obj_set_style_text_color(inviteState, lv_color_hex(0xB9D5E7), 0);
            }
        }

        if (chatMessages[i].outgoing) {
            lv_obj_t *delBtn = lv_btn_create(row);
            lv_obj_set_size(delBtn, 24, 24);
            lv_obj_set_style_border_width(delBtn, 0, 0);
            lv_obj_set_style_radius(delBtn, 12, 0);
            lv_obj_set_style_pad_all(delBtn, 0, 0);
            lv_obj_clear_flag(delBtn, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_event_cb(delBtn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
            lv_obj_add_event_cb(delBtn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
            lv_obj_add_event_cb(delBtn, lvglDeleteChatMessageEvent, LV_EVENT_CLICKED, reinterpret_cast<void *>(static_cast<intptr_t>(i)));
            lv_obj_t *delLbl = lv_label_create(delBtn);
            lv_label_set_text(delLbl, LV_SYMBOL_TRASH);
            lv_obj_set_style_text_color(delLbl, lv_color_hex(0xF5D2D2), 0);
            lv_obj_center(delLbl);
            lvglRegisterStyledButton(delBtn, lv_color_hex(0x7A2E2E), true);
        }
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
    const bool sentRadio = hc12SendConversationDelete(peerKey);
    chatApplyConversationDeletion(peerKey, "Conversation deleted");
    uiStatusLine = sentGlobal ? "Conversation deleted for all"
                              : ((sentLan || sentRadio) ? "Conversation delete sent" : "Deleted locally only");
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
        lv_obj_add_flag(lvglChatMenuBackdrop, LV_OBJ_FLAG_HIDDEN);
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
        for (int i = 0; i < hc12DiscoveredCount; ++i) {
            if (p2pFindPeerByPubKeyHex(hc12DiscoveredPeers[i].pubKeyHex) >= 0) continue;
            hasContacts = true;
            break;
        }
    }

    if (!hasContacts) {
        lv_obj_t *lbl = lv_label_create(lvglChatContacts);
        lv_label_set_text(lbl, "No chats yet");
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xC8CED6), 0);
        lvglRegisterReorderableItem(lbl, "ord_ctc", "empty");
        lvglApplySavedOrder(lvglChatContacts, "ord_ctc");
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
        lv_obj_add_event_cb(b, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(b, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
        lvglRegisterStyledButton(b, col, conversationOpen);
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
                const int rawIdx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
                if (rawIdx >= 10000) {
                    const int radioIdx = rawIdx - 10000;
                    if (radioIdx < 0 || radioIdx >= hc12DiscoveredCount) return;
                    chatScheduleOpenPeerConversation(hc12DiscoveredPeers[radioIdx].pubKeyHex);
                    return;
                }
                if (rawIdx < 0 || rawIdx >= p2pPeerCount) return;
                chatScheduleOpenPeerConversation(p2pPeers[rawIdx].pubKeyHex);
            },
            reinterpret_cast<void *>(static_cast<intptr_t>(i)));
        if (!btn) continue;
        const String orderKey = lvglOrderTokenFromText("p", p2pPeers[i].pubKeyHex);
        lvglRegisterReorderableItem(btn, "ord_ctc", orderKey.c_str());

        lv_obj_t *nameLabel = lv_label_create(btn);
        if (nameLabel) {
            lv_label_set_text(nameLabel, p2pPeers[i].name.isEmpty() ? "Peer" : p2pPeers[i].name.c_str());
            lv_obj_align(nameLabel, LV_ALIGN_LEFT_MID, 10, 0);
            lvglRegisterStyledButton(btn, col, !currentChatPeerKey.isEmpty());
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

    for (int i = 0; i < hc12DiscoveredCount; ++i) {
        if (p2pFindPeerByPubKeyHex(hc12DiscoveredPeers[i].pubKeyHex) >= 0) continue;
        lv_color_t col = hc12DiscoveredPeers[i].pubKeyHex == currentChatPeerKey ? lv_color_hex(0x946226) : lv_color_hex(0x7A5C2E);
        const String title = (hc12DiscoveredPeers[i].name.isEmpty() ? String("Radio Peer") : hc12DiscoveredPeers[i].name) + "  [Radio]";
        lv_obj_t *btn = makeContactBtn(
            lvglChatContacts,
            title.c_str(),
            col,
            [](lv_event_t *e) {
                const int radioIdx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e))) - 10000;
                if (radioIdx < 0 || radioIdx >= hc12DiscoveredCount) return;
                chatScheduleOpenPeerConversation(hc12DiscoveredPeers[radioIdx].pubKeyHex);
            },
            reinterpret_cast<void *>(static_cast<intptr_t>(10000 + i)));
        if (!btn) continue;
        const String orderKey = lvglOrderTokenFromText("r", hc12DiscoveredPeers[i].pubKeyHex);
        lvglRegisterReorderableItem(btn, "ord_ctc", orderKey.c_str());

        lv_obj_t *nameLabel = lv_label_create(btn);
        if (nameLabel) {
            lv_label_set_text(nameLabel, title.c_str());
            lv_obj_align(nameLabel, LV_ALIGN_LEFT_MID, 10, 0);
            lvglRegisterStyledButton(btn, col, !currentChatPeerKey.isEmpty());
        }

        if (hc12DiscoveredPeers[i].unread) {
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
    lvglApplySavedOrder(lvglChatContacts, "ord_ctc");
}

void lvglSetChatPeerScanButtonStatus(const char *text, uint32_t revertDelayMs)
{
    if (!lvglChatPeerScanBtn) return;
    lv_obj_t *label = lv_obj_get_child(lvglChatPeerScanBtn, 0);
    if (!label) return;
    lv_label_set_text(label, lvglSymbolText(LV_SYMBOL_REFRESH, text ? text : "Scan").c_str());
    if (revertDelayMs == 0) return;
    lv_timer_t *timer = lv_timer_create(
        [](lv_timer_t *timer) {
            lv_obj_t *btn = static_cast<lv_obj_t *>(timer ? timer->user_data : nullptr);
            if (btn) {
                lv_obj_t *lbl = lv_obj_get_child(btn, 0);
                if (lbl) lv_label_set_text(lbl, lvglSymbolText(LV_SYMBOL_REFRESH, "Scan").c_str());
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
        p2pSendDiscoveryProbe();
        hc12LastDiscoveryAnnounceMs = 0;
        hc12BroadcastDiscoveryFrame("probe");
        uiStatusLine = "WiFi and radio scan started";
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
        const bool ok = p2pSendPairRequest(
            p2pDiscoveredPeers[idx].name.isEmpty() ? String("Peer") : p2pDiscoveredPeers[idx].name,
            p2pDiscoveredPeers[idx].pubKeyHex,
            p2pDiscoveredPeers[idx].ip,
            p2pDiscoveredPeers[idx].port);
        uiStatusLine = ok ? "Pair request sent" : "Pair request failed";
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
    if (lvglChatDiscoveryBtn) {
        lv_obj_t *label = lv_obj_get_child(lvglChatDiscoveryBtn, 0);
        if (label) lv_label_set_text(label, lvglSymbolText(LV_SYMBOL_WIFI, "Discovery").c_str());
        lvglApplyChatDiscoveryButtonStyle();
    }
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
        lv_obj_add_event_cb(b, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(b, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
        lv_obj_t *l = lv_label_create(b);
        if (l) {
            lv_label_set_text(l, txt);
            lv_obj_center(l);
        }
        lvglRegisterStyledButton(b, col, true);
        return b;
    };

    bool anyEntries = false;

    if (p2pPeerCount > 0) {
        lv_obj_t *section = lv_label_create(lvglChatPeerList);
        lv_obj_set_width(section, lv_pct(100));
        lv_label_set_text(section, "Paired Devices");
        lv_obj_set_style_text_color(section, lv_color_hex(0xE5ECF3), 0);
        lvglRegisterReorderableItem(section, "ord_cpr", "sec_p");

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
            const String orderKey = lvglOrderTokenFromText("p", p2pPeers[i].pubKeyHex);
            lvglRegisterReorderableItem(card, "ord_cpr", orderKey.c_str());

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
        lvglRegisterReorderableItem(section, "ord_cpr", "sec_d");
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
        const String orderKey = lvglOrderTokenFromText("d", p2pDiscoveredPeers[i].pubKeyHex);
        lvglRegisterReorderableItem(card, "ord_cpr", orderKey.c_str());

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

    bool anyRadioDiscovered = false;
    for (int i = 0; i < hc12DiscoveredCount; ++i) {
        if (p2pFindPeerByPubKeyHex(hc12DiscoveredPeers[i].pubKeyHex) >= 0) continue;
        anyRadioDiscovered = true;
        break;
    }

    if (anyRadioDiscovered) {
        lv_obj_t *section = lv_label_create(lvglChatPeerList);
        lv_obj_set_width(section, lv_pct(100));
        lv_label_set_text(section, "HC-12 Radio");
        lv_obj_set_style_text_color(section, lv_color_hex(0xE5ECF3), 0);
        lvglRegisterReorderableItem(section, "ord_cpr", "sec_r");
    }

    for (int i = 0; i < hc12DiscoveredCount; ++i) {
        if (p2pFindPeerByPubKeyHex(hc12DiscoveredPeers[i].pubKeyHex) >= 0) continue;
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
        const String orderKey = lvglOrderTokenFromText("r", hc12DiscoveredPeers[i].pubKeyHex);
        lvglRegisterReorderableItem(card, "ord_cpr", orderKey.c_str());

        lv_obj_t *title = lv_label_create(card);
        lv_label_set_text_fmt(title, "%s  [radio]",
                              hc12DiscoveredPeers[i].name.isEmpty() ? "Radio Peer" : hc12DiscoveredPeers[i].name.c_str());
        lv_obj_set_style_text_color(title, lv_color_hex(0xE5ECF3), 0);

        lv_obj_t *info = lv_label_create(card);
        lv_obj_set_width(info, lv_pct(100));
        lv_label_set_long_mode(info, LV_LABEL_LONG_WRAP);
        String key = hc12DiscoveredPeers[i].pubKeyHex;
        if (key.length() > 16) key = key.substring(0, 16) + "...";
        const unsigned long ageSec = static_cast<unsigned long>((millis() - hc12DiscoveredPeers[i].lastSeenMs) / 1000UL);
        lv_label_set_text_fmt(info, "HC-12 direct radio\nSeen %lus ago\n%s", ageSec, key.c_str());
        lv_obj_set_style_text_color(info, lv_color_hex(0xB7C4D1), 0);

        lv_obj_t *row = lv_obj_create(card);
        lv_obj_set_size(row, lv_pct(100), 30);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_set_style_pad_column(row, 6, 0);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        makeSmallBtnLocal(row, "Open", 54, 26, lv_color_hex(0x7A5C2E), [](lv_event_t *e) {
            const int radioIdx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e))) - 20000;
            if (radioIdx < 0 || radioIdx >= hc12DiscoveredCount) return;
            chatScheduleOpenPeerConversation(hc12DiscoveredPeers[radioIdx].pubKeyHex);
        }, reinterpret_cast<void *>(static_cast<intptr_t>(20000 + i)));
    }

    if (!anyEntries) {
        lv_obj_t *lbl = lv_label_create(lvglChatPeerList);
        lv_obj_set_width(lbl, lv_pct(100));
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_WRAP);
        lv_label_set_text(lbl, "No paired or discovered peers yet.\nOpen this screen on another device on the same WiFi or same HC-12 channel, then tap Scan.");
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xC8CED6), 0);
        lvglRegisterReorderableItem(lbl, "ord_cpr", "empty");
    }
    lvglApplySavedOrder(lvglChatPeerList, "ord_cpr");
}

void p2pEnsureUdp()
{
    if (!p2pReady) return;
    if (p2pUdpStarted) return;
    if (WiFi.getMode() == WIFI_OFF) return;
    if (!wifiLanAvailableSafe()) return;
    if (p2pUdp.begin(P2P_UDP_PORT) == 1) {
        p2pUdpStarted = true;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
        p2pUdpLastParseError = 0;
        p2pUdpLastParseErrorMs = 0;
#endif
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

bool p2pSendMessageDelete(const String &peerKey, const String &messageId)
{
    if (!p2pReady || peerKey.isEmpty() || messageId.isEmpty()) return false;
    const int peerIdx = p2pFindPeerByPubKeyHex(peerKey);
    if (peerIdx < 0 || !p2pPeers[peerIdx].enabled || p2pPeers[peerIdx].ip == IPAddress((uint32_t)0)) return false;

    JsonDocument doc;
    doc["kind"] = "delete_message";
    doc["id"] = messageId;
    doc["author"] = deviceShortNameValue();
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
    if (!p2pReady || !wifiLanAvailableSafe()) return;
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

    IPAddress bcast = wifiLanBroadcastIpSafe();
    if (bcast[0] == 0) return;
    if (p2pUdp.beginPacket(bcast, P2P_UDP_PORT)) {
        p2pUdp.write(reinterpret_cast<const uint8_t *>("\x50\x32\x01\x02"), 4);
        p2pUdp.write(reinterpret_cast<const uint8_t *>(payload), len);
        p2pUdp.endPacket();
    }
}

void p2pSendDiscoveryProbe()
{
    if (!p2pReady || !wifiLanAvailableSafe()) return;
    p2pEnsureUdp();
    if (!p2pUdpStarted) return;

    JsonDocument doc;
    doc["kind"] = "discover_probe";
    doc["device"] = deviceShortNameValue();
    doc["public_key"] = p2pPublicKeyHex();
    doc["port"] = P2P_UDP_PORT;
    char payload[256] = {0};
    const size_t len = serializeJson(doc, payload, sizeof(payload));
    if (len == 0) return;

    IPAddress bcast = wifiLanBroadcastIpSafe();
    if (bcast[0] == 0) return;
    if (p2pUdp.beginPacket(bcast, P2P_UDP_PORT)) {
        p2pUdp.write(reinterpret_cast<const uint8_t *>("\x50\x32\x01\x02"), 4);
        p2pUdp.write(reinterpret_cast<const uint8_t *>(payload), len);
        p2pUdp.endPacket();
    }
}

void p2pSendDiscoveryAnnounceTo(const IPAddress &ip, uint16_t port)
{
    if (!p2pReady || !wifiLanAvailableSafe() || !p2pDiscoveryEnabled) return;
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

    if (p2pUdp.beginPacket(ip, port ? port : P2P_UDP_PORT)) {
        p2pUdp.write(reinterpret_cast<const uint8_t *>("\x50\x32\x01\x02"), 4);
        p2pUdp.write(reinterpret_cast<const uint8_t *>(payload), len);
        p2pUdp.endPacket();
    }
}

bool p2pSendPairRequest(const String &name, const String &pubKeyHex, const IPAddress &ip, uint16_t port)
{
    if (!p2pReady || !wifiLanAvailableSafe() || pubKeyHex.isEmpty()) return false;
    p2pEnsureUdp();
    if (!p2pUdpStarted) return false;

    JsonDocument doc;
    doc["kind"] = "pair_request";
    doc["device"] = deviceShortNameValue();
    doc["public_key"] = p2pPublicKeyHex();
    doc["port"] = P2P_UDP_PORT;
    char payload[256] = {0};
    const size_t len = serializeJson(doc, payload, sizeof(payload));
    if (len == 0) return false;

    p2pTouchDiscoveredSeen(name, pubKeyHex, ip, port);
    if (!p2pUdp.beginPacket(ip, port ? port : P2P_UDP_PORT)) return false;
    p2pUdp.write(reinterpret_cast<const uint8_t *>("\x50\x32\x01\x02"), 4);
    p2pUdp.write(reinterpret_cast<const uint8_t *>(payload), len);
    return p2pUdp.endPacket() == 1;
}

void p2pSendPairResponse(const String &name, const String &pubKeyHex, const IPAddress &ip, uint16_t port, bool accepted)
{
    if (!p2pReady || !wifiLanAvailableSafe() || pubKeyHex.isEmpty()) return;
    p2pEnsureUdp();
    if (!p2pUdpStarted) return;

    JsonDocument doc;
    doc["kind"] = accepted ? "pair_accept" : "pair_reject";
    doc["device"] = deviceShortNameValue();
    doc["public_key"] = p2pPublicKeyHex();
    doc["port"] = P2P_UDP_PORT;
    char payload[256] = {0};
    const size_t len = serializeJson(doc, payload, sizeof(payload));
    if (len == 0) return;

    p2pTouchDiscoveredSeen(name, pubKeyHex, ip, port);
    if (p2pUdp.beginPacket(ip, port ? port : P2P_UDP_PORT)) {
        p2pUdp.write(reinterpret_cast<const uint8_t *>("\x50\x32\x01\x02"), 4);
        p2pUdp.write(reinterpret_cast<const uint8_t *>(payload), len);
        p2pUdp.endPacket();
    }
}

bool p2pAddOrUpdateTrustedPeer(const String &name, const String &pubKeyHex, const IPAddress &ip, uint16_t port)
{
    unsigned char testPk[P2P_PUBLIC_KEY_BYTES] = {0};
    if (name.isEmpty() || !p2pHexToBytes(pubKeyHex, testPk, sizeof(testPk))) return false;
    if (!p2pEnsurePeerStorage()) return false;
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
    if (!wifiLanAvailableSafe()) {
        if (p2pUdpStarted) {
            p2pUdp.stop();
            p2pUdpStarted = false;
        }
        return;
    }
    p2pEnsureUdp();
    if (!p2pUdpStarted) return;

    const unsigned long now = millis();
    if (p2pDiscoveryEnabled && wifiLanAvailableSafe() && (now - p2pLastDiscoverAnnounceMs) >= 5000UL) {
        p2pLastDiscoverAnnounceMs = now;
        p2pBroadcastDiscover();
    }

    int packetLen = p2pUdp.parsePacket();
    if (packetLen < 0) {
        const unsigned long nowMs = millis();
#if defined(BOARD_ESP32S3_3248S035_N16R8)
        if (p2pUdpLastParseError != packetLen ||
            static_cast<unsigned long>(nowMs - p2pUdpLastParseErrorMs) >= 3000UL) {
            Serial.printf("[P2P] UDP parsePacket error=%d mode=%d sta=%d ap=%d\n",
                          packetLen,
                          static_cast<int>(WiFi.getMode()),
                          wifiConnectedSafe() ? 1 : 0,
                          apModeActive ? 1 : 0);
            p2pUdpLastParseError = packetLen;
            p2pUdpLastParseErrorMs = nowMs;
        }
#else
        (void)nowMs;
#endif
        p2pUdp.stop();
        p2pUdpStarted = false;
        delay(1);
        return;
    }
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
                        } else if (kind == "discover_probe" && !pubKeyHex.equalsIgnoreCase(p2pPublicKeyHex())) {
                            if (p2pDiscoveryEnabled) p2pSendDiscoveryAnnounceTo(p2pUdp.remoteIP(), port);
                        } else if (kind == "pair_request" && !pubKeyHex.equalsIgnoreCase(p2pPublicKeyHex())) {
                            p2pTouchDiscoveredSeen(device, pubKeyHex, p2pUdp.remoteIP(), port);
                            if (p2pDiscoveryEnabled) {
                                const int requestIdx = p2pFindDiscoveredByPubKeyHex(pubKeyHex);
                                p2pPairRequestPending = true;
                                p2pPairPromptVisible = false;
                                p2pPairRequestDiscoveredIdx = requestIdx;
                                wakeDisplayForIncomingNotification();
                            }
                        } else if (kind == "pair_accept" && !pubKeyHex.equalsIgnoreCase(p2pPublicKeyHex())) {
                            const bool ok = p2pAddOrUpdateTrustedPeer(device.isEmpty() ? String("Peer") : device,
                                                                      pubKeyHex,
                                                                      p2pUdp.remoteIP(),
                                                                      port);
                            if (ok && currentChatPeerKey.isEmpty()) currentChatPeerKey = pubKeyHex;
                            uiStatusLine = ok ? "Peer paired" : "Pair accept failed";
                            if (lvglReady) {
                                lvglSyncStatusLine();
                                lvglRefreshChatPeerUi();
                                lvglRefreshChatContactsUi();
                            }
                        } else if (kind == "pair_reject" && !pubKeyHex.equalsIgnoreCase(p2pPublicKeyHex())) {
                            uiStatusLine = (device.isEmpty() ? String("Peer") : device) + " rejected pairing";
                            if (lvglReady) lvglSyncStatusLine();
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
                                            wakeDisplayForIncomingNotification();
                                            p2pRefreshTrustedPeerIdentity(p2pPeers[peerIdx].pubKeyHex, author, p2pUdp.remoteIP(), p2pUdp.remotePort());
                                            p2pTouchPeerSeen(peerIdx, p2pUdp.remoteIP(), p2pUdp.remotePort());
                                            if (checkersHandleIncomingChatPayload(p2pPeers[peerIdx].pubKeyHex,
                                                                                  author,
                                                                                  text,
                                                                                  CHAT_TRANSPORT_WIFI,
                                                                                  messageId)) {
                                                if (lvglReady) {
                                                    lvglSyncStatusLine();
                                                    if (uiScreen == UI_CHAT) lvglRefreshChatUi();
                                                }
                                            } else if (messageId.isEmpty() || !chatHasLoggedMessageId(p2pPeers[peerIdx].pubKeyHex, messageId)) {
                                                chatStoreMessage(p2pPeers[peerIdx].pubKeyHex, author, text, false, CHAT_TRANSPORT_WIFI, messageId);
                                                chatQueueIncomingMessageBeep();
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
                                    } else if (kind == "delete_message") {
                                        const String messageId = String(static_cast<const char *>(doc["id"] | ""));
                                        if (!messageId.isEmpty()) {
                                            p2pRefreshTrustedPeerIdentity(p2pPeers[peerIdx].pubKeyHex, author, p2pUdp.remoteIP(), p2pUdp.remotePort());
                                            p2pTouchPeerSeen(peerIdx, p2pUdp.remoteIP(), p2pUdp.remotePort());
                                            chatDeleteMessageById(p2pPeers[peerIdx].pubKeyHex, messageId, "Message deleted by " + author);
                                        }
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
        if (packetLen < 0) {
            p2pUdp.stop();
            p2pUdpStarted = false;
            delay(1);
            return;
        }
        yield();
    }
}

void lvglRefreshInfoPanel(bool refreshIndicators)
{
    if (!lvglInfoList) return;
    if (uiScreen != UI_INFO) return;
    if (refreshIndicators) {
        sampleTopIndicators();
        lvglRefreshTopIndicators();
    }
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
    const uint32_t heapTotal = static_cast<uint32_t>(ESP.getHeapSize());
    const uint32_t largest8 = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    const uint32_t psramFree = boardHasUsablePsram()
                                   ? static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM))
                                   : 0U;
    const uint32_t psramTotal = boardHasUsablePsram()
                                    ? static_cast<uint32_t>(heap_caps_get_total_size(MALLOC_CAP_SPIRAM))
                                    : 0U;
    char batteryBuf[40];
    const int batteryCentivolts = static_cast<int>(batteryVoltage * 100.0f + 0.5f);
    snprintf(batteryBuf, sizeof(batteryBuf), "%d.%02dV  |  %s",
             batteryCentivolts / 100,
             abs(batteryCentivolts % 100),
             batteryCharging ? "Charging" : "On battery");
    lv_obj_t *batteryCard = lvglFindInfoCardByTitle("Battery");
    lv_obj_t *wifiCard = lvglFindInfoCardByTitle("WiFi Strength");
    lv_obj_t *lightCard = lvglFindInfoCardByTitle("Lighting");

    if (lvglInfoBatteryValueLabel) lvglLabelSetTextIfChanged(lvglInfoBatteryValueLabel, String(batteryPercent) + "%");
    if (lvglInfoBatterySubLabel) {
        lvglLabelSetTextIfChanged(lvglInfoBatterySubLabel, batteryBuf);
    }
    if (lv_obj_t *batteryBar = lvglInfoCardBar(batteryCard)) {
        lvglBarSetValueIfChanged(batteryBar, static_cast<int32_t>(batteryPercent), LV_ANIM_ON);
        lv_color_t batteryColor = lv_color_hex(0x52B788);
        if (batteryPercent <= 20) batteryColor = lv_color_hex(0xD95C5C);
        else if (batteryPercent <= 45) batteryColor = lv_color_hex(0xF2C35E);
        lvglSetInfoBarColor(batteryBar, batteryColor);
    }

    if (lvglInfoWifiValueLabel) lvglLabelSetTextIfChanged(lvglInfoWifiValueLabel, connected ? String(wifiQuality) + "%" : String("Offline"));
    if (lvglInfoWifiSubLabel) {
        if (connected) lvglLabelSetTextIfChanged(lvglInfoWifiSubLabel, ssid + "  |  " + ip + "  |  " + String(rssi) + "dBm");
        else if (apModeActive) lvglLabelSetTextIfChanged(lvglInfoWifiSubLabel, "AP " + savedApSsid + "  |  " + WiFi.softAPIP().toString() + "  |  Touch to Config > WiFi Config to connect");
        else lvglLabelSetTextIfChanged(lvglInfoWifiSubLabel, "AP " + savedApSsid + "  |  Touch to Config > WiFi Config to connect");
    }
    if (lv_obj_t *wifiBar = lvglInfoCardBar(wifiCard)) {
        lvglBarSetValueIfChanged(wifiBar, static_cast<int32_t>(connected ? wifiQuality : 0U), LV_ANIM_ON);
        lv_color_t wifiColor = lv_color_hex(0x4A5563);
        if (connected) {
            wifiColor = lv_color_hex(0x4FC3F7);
            if (wifiQuality <= 25) wifiColor = lv_color_hex(0xD95C5C);
            else if (wifiQuality <= 55) wifiColor = lv_color_hex(0xF2C35E);
        }
        lvglSetInfoBarColor(wifiBar, wifiColor);
    }

    if (lvglInfoHc12ValueLabel) lvglLabelSetTextIfChanged(lvglInfoHc12ValueLabel, hc12InfoValueText);
    if (lvglInfoHc12SubLabel) lvglLabelSetTextIfChanged(lvglInfoHc12SubLabel, hc12InfoSubText);

    if (lvglInfoLightValueLabel) lvglLabelSetTextIfChanged(lvglInfoLightValueLabel, String(lightPercent) + "%");
    if (lvglInfoLightSubLabel) {
        lvglLabelSetTextIfChanged(lvglInfoLightSubLabel,
                                  String("Display ") + (displayAwake ? "awake" : "sleeping") +
                                  "  |  Backlight " + String(displayBrightnessPercent) +
                                  "%  |  raw " + String(lightRawAdc));
    }
    if (lv_obj_t *lightBar = lvglInfoCardBar(lightCard)) {
        lvglBarSetValueIfChanged(lightBar, static_cast<int32_t>(lightPercent), LV_ANIM_ON);
        lv_color_t lightColor = lv_color_hex(0x4A5563);
        if (lightPercent >= 75) lightColor = lv_color_hex(0xF4B942);
        else if (lightPercent >= 35) lightColor = lv_color_hex(0xF2C35E);
        else lightColor = lv_color_hex(0x6AAEE6);
        lvglSetInfoBarColor(lightBar, lightColor);
    }

    lv_obj_t *sdCard = lvglFindInfoCardByTitle("SD Card");
    uint64_t sdTotal = 0;
    uint64_t sdUsed = 0;
    if (sdMounted) {
        sdTotal = SD.totalBytes();
        sdUsed = SD.usedBytes();
    }
    const uint8_t sdPct = (sdTotal > 0) ? static_cast<uint8_t>((sdUsed * 100ULL) / sdTotal) : 0U;
    if (lv_obj_t *sdValue = lvglInfoCardValueLabel(sdCard)) {
        if (sdTotal > 0) lvglLabelSetTextIfChanged(sdValue, String(sdPct) + "%");
        else lvglLabelSetTextIfChanged(sdValue, sdMounted ? "--" : "Off");
    }
    if (lv_obj_t *sdSub = lvglInfoCardSubLabel(sdCard)) {
        if (sdTotal > 0) {
            lvglLabelSetTextIfChanged(sdSub,
                                      "Used " + String(static_cast<unsigned long long>(sdUsed / (1024ULL * 1024ULL))) +
                                      " / " + String(static_cast<unsigned long long>(sdTotal / (1024ULL * 1024ULL))) +
                                      " MB  |  Free " + String(static_cast<unsigned long long>((sdTotal - sdUsed) / (1024ULL * 1024ULL))) + " MB");
        } else {
            lvglLabelSetTextIfChanged(sdSub, sdMounted ? "Capacity unavailable" : "Card offline");
        }
    }
    if (lv_obj_t *sdBar = lvglInfoCardBar(sdCard)) {
        lvglBarSetValueIfChanged(sdBar, static_cast<int32_t>(sdTotal > 0 ? sdPct : 0U), LV_ANIM_ON);
        lv_color_t sdColor = lv_color_hex(0xE59F45);
        if (sdPct >= 90) sdColor = lv_color_hex(0xD95C5C);
        else if (sdPct >= 75) sdColor = lv_color_hex(0xF2C35E);
        lvglSetInfoBarColor(sdBar, sdTotal > 0 ? sdColor : lv_color_hex(0x3A4150));
    }

    const uint32_t sramUsed = (heapTotal >= freeHeap) ? (heapTotal - freeHeap) : 0U;
    const uint8_t sramPct = (heapTotal > 0) ? static_cast<uint8_t>((static_cast<uint64_t>(sramUsed) * 100ULL) / heapTotal) : 0U;
    if (lv_obj_t *sramValue = lvglInfoCardValueLabel(lvglInfoSramCard)) {
        lvglLabelSetTextIfChanged(sramValue, String(sramPct) + "%");
    }
    if (lv_obj_t *sramSub = lvglInfoCardSubLabel(lvglInfoSramCard)) {
        lvglLabelSetTextIfChanged(sramSub,
                                  "Used " + String(static_cast<unsigned long>(sramUsed / 1024U)) +
                                  " / " + String(static_cast<unsigned long>(heapTotal / 1024U)) +
                                  " KB  |  Free " + String(static_cast<unsigned long>(freeHeap / 1024U)) + " KB");
    }
    if (lv_obj_t *sramBar = lvglInfoCardBar(lvglInfoSramCard)) {
        lvglBarSetValueIfChanged(sramBar, static_cast<int32_t>(sramPct), LV_ANIM_ON);
        lv_color_t sramColor = lv_color_hex(0x4FC3F7);
        if (sramPct >= 85) sramColor = lv_color_hex(0xD95C5C);
        else if (sramPct >= 65) sramColor = lv_color_hex(0xF2C35E);
        lvglSetInfoBarColor(sramBar, sramColor);
    }

    const uint32_t psramUsed = (psramTotal >= psramFree) ? (psramTotal - psramFree) : 0U;
    const uint32_t psramPctTenths = (psramTotal > 0)
                                        ? static_cast<uint32_t>((static_cast<uint64_t>(psramUsed) * 1000ULL + (psramTotal / 2ULL)) / psramTotal)
                                        : 0U;
    const uint8_t psramPct = static_cast<uint8_t>(min<uint32_t>(100U, (psramPctTenths + 5U) / 10U));
    if (lv_obj_t *psramValue = lvglInfoCardValueLabel(lvglInfoPsramCard)) {
        if (psramTotal > 0) {
            if (psramPctTenths < 100U) {
                lvglLabelSetTextIfChanged(psramValue,
                                          String(static_cast<unsigned int>(psramPctTenths / 10U)) + "." +
                                          String(static_cast<unsigned int>(psramPctTenths % 10U)) + "%");
            } else {
                lvglLabelSetTextIfChanged(psramValue, String(static_cast<unsigned int>(psramPct)) + "%");
            }
        }
        else lvglLabelSetTextIfChanged(psramValue, "--");
    }
    if (lv_obj_t *psramSub = lvglInfoCardSubLabel(lvglInfoPsramCard)) {
        if (psramTotal > 0) {
            lvglLabelSetTextIfChanged(psramSub,
                                      "Used " + String(static_cast<unsigned long>(psramUsed / 1024U)) +
                                      " / " + String(static_cast<unsigned long>(psramTotal / 1024U)) +
                                      " KB  |  Free " + String(static_cast<unsigned long>(psramFree / 1024U)) + " KB");
        } else {
            lvglLabelSetTextIfChanged(psramSub, "Not available on this board");
        }
    }
    if (lv_obj_t *psramBar = lvglInfoCardBar(lvglInfoPsramCard)) {
        lvglBarSetValueIfChanged(psramBar, static_cast<int32_t>(psramTotal > 0 ? psramPct : 0U), LV_ANIM_ON);
        lv_color_t psramColor = lv_color_hex(0x9B7CF2);
        if (psramPct >= 85) psramColor = lv_color_hex(0xD95C5C);
        else if (psramPct >= 65) psramColor = lv_color_hex(0xF2C35E);
        lvglSetInfoBarColor(psramBar, psramTotal > 0 ? psramColor : lv_color_hex(0x3A4150));
    }

    if (lv_obj_t *cpuValue = lvglInfoCardValueLabel(lvglInfoCpuCard)) lv_label_set_text_fmt(cpuValue, "%u%%", static_cast<unsigned int>(cpuLoadPercent));
    if (lv_obj_t *cpuSub = lvglInfoCardSubLabel(lvglInfoCpuCard)) {
        if (psramTotal > 0) {
            lv_label_set_text_fmt(cpuSub,
                                  "Approx. main-loop load  |  SRAM %lu KB  |  PSRAM %lu KB",
                                  static_cast<unsigned long>(freeHeap / 1024U),
                                  static_cast<unsigned long>(psramFree / 1024U));
        } else {
            lv_label_set_text_fmt(cpuSub, "Approx. main-loop load  |  SRAM %lu KB",
                                  static_cast<unsigned long>(freeHeap / 1024U));
        }
    }
    if (lv_obj_t *cpuBar = lvglInfoCardBar(lvglInfoCpuCard)) {
        lv_bar_set_value(cpuBar, static_cast<int32_t>(cpuLoadPercent), LV_ANIM_OFF);
        lv_color_t cpuColor = lv_color_hex(0x52B788);
        if (cpuLoadPercent >= 85) cpuColor = lv_color_hex(0xD95C5C);
        else if (cpuLoadPercent >= 65) cpuColor = lv_color_hex(0xF2C35E);
        lvglSetInfoBarColor(cpuBar, cpuColor);
    }

    if (lv_obj_t *tempValue = lvglInfoCardValueLabel(lvglInfoTempCard)) {
        if (!isnan(tempC) && tempC > 0.0f) {
            char tempBuf[16];
            const int tempTenths = static_cast<int>(tempC * 10.0f + 0.5f);
            snprintf(tempBuf, sizeof(tempBuf), "%d.%dC", tempTenths / 10, abs(tempTenths % 10));
            lv_label_set_text(tempValue, tempBuf);
        } else {
            lv_label_set_text(tempValue, "--");
        }
    }
    if (lv_obj_t *tempSub = lvglInfoCardSubLabel(lvglInfoTempCard)) lv_label_set_text_fmt(tempSub, "ESP32 die temp  |  target range under %uC", INFO_TEMP_WARN_C);
    if (lv_obj_t *tempBar = lvglInfoCardBar(lvglInfoTempCard)) {
        lv_bar_set_value(tempBar, static_cast<int32_t>(tempBarValue), LV_ANIM_OFF);
        lv_color_t tempColor = lv_color_hex(0x4FC3F7);
        if (tempBarValue >= INFO_TEMP_HOT_C) tempColor = lv_color_hex(0xD95C5C);
        else if (tempBarValue >= INFO_TEMP_WARN_C) tempColor = lv_color_hex(0xF2C35E);
        lvglSetInfoBarColor(tempBar, tempColor);
    }

    if (lvglInfoSystemLabel) {
        if (psramTotal > 0) {
            lv_label_set_text_fmt(
                lvglInfoSystemLabel,
                "Model: %s\nDevice name: %s\nFirmware: %s\nSoftAP: %s\nLargest 8-bit block: %lu KB\nPSRAM: %lu / %lu KB free\nSD: %s\nMedia: %s",
                DEVICE_MODEL,
                deviceShortNameValue().c_str(),
                FW_VERSION,
                AP_SSID,
                static_cast<unsigned long>(largest8 / 1024U),
                static_cast<unsigned long>(psramFree / 1024U),
                static_cast<unsigned long>(psramTotal / 1024U),
                sdMounted ? "mounted" : "offline",
                mediaIsPlaying ? mediaNowPlaying.c_str() : "stopped");
        } else {
            lv_label_set_text_fmt(
                lvglInfoSystemLabel,
                "Model: %s\nDevice name: %s\nFirmware: %s\nSoftAP: %s\nLargest 8-bit block: %lu KB\nSD: %s\nMedia: %s",
                DEVICE_MODEL,
                deviceShortNameValue().c_str(),
                FW_VERSION,
                AP_SSID,
                static_cast<unsigned long>(largest8 / 1024U),
                sdMounted ? "mounted" : "offline",
                mediaIsPlaying ? mediaNowPlaying.c_str() : "stopped");
        }
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
    lvglWifiWebServerBtn = nullptr;

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
        lv_obj_add_event_cb(btn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(btn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, ud);
        lv_obj_t *lbl = lv_label_create(btn);
        if (lbl) {
            lv_label_set_text(lbl, txt);
            lv_obj_center(lbl);
        }
        lvglRegisterStyledButton(btn, col, true);
        return btn;
    };

    lv_obj_t *staCard = makeCard("WiFi Config");
    if (staCard) {
        lvglRegisterReorderableItem(staCard, "ord_wifi", "sta");
        lv_obj_t *info = lv_label_create(staCard);
        lv_obj_set_width(info, lv_pct(100));
        lv_label_set_long_mode(info, LV_LABEL_LONG_WRAP);
        const String desiredStaSsid = wifiDesiredStaSsid();
        String staLine = desiredStaSsid.isEmpty() ? String("Saved network: none") : ("Saved network: " + desiredStaSsid);
        if (wifiForgetPendingUi) staLine += "\nStatus: forgetting saved WiFi...";
        else if (wifiConnectedSafe()) staLine += "\nConnected: " + wifiSsidSafe() + "  |  " + wifiIpSafe();
        else if (bootStaConnectInProgress) staLine += "\nStatus: connecting...";
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
        if (wifiConnectedSafe() || bootStaConnectInProgress || wifiForgetPendingUi) {
            makeBtn(row, lvglSymbolText(LV_SYMBOL_CLOSE, "Disconnect").c_str(), 102, 28, lv_color_hex(0x2F6D86), lvglWifiDisconnectEvent);
        }
        if (!desiredStaSsid.isEmpty() || pendingSaveCreds || wifiForgetPendingUi) {
            makeBtn(row, lvglSymbolText(LV_SYMBOL_TRASH, "Forget").c_str(), 82, 28, lv_color_hex(0x8A3A3A), lvglWifiForgetEvent);
        }

        lvglWifiWebServerBtn = makeBtn(
            staCard,
            lvglSymbolText(LV_SYMBOL_DIRECTORY, webServerEnabled ? "Web Server: ON" : "Web Server: OFF").c_str(),
            164,
            30,
            webServerEnabled ? lv_color_hex(0x357A38) : lv_color_hex(0x6B3A3A),
            lvglWifiWebServerToggleEvent
        );
        if (lvglWifiWebServerBtn) {
            lvglApplyWifiWebServerButtonStyle();
        }
    }

    lv_obj_t *scanBtn = makeBtn(lvglWifiList, lvglSymbolText(LV_SYMBOL_REFRESH, "Scan").c_str(), 108, 30, lv_color_hex(0x2F6D86), lvglWifiRescanEvent);
    if (scanBtn) lvglRegisterReorderableItem(scanBtn, "ord_wifi", "scan");

    if (wifiScanInProgress) {
        lvglWifiScanLabel = lv_label_create(lvglWifiList);
        if (lvglWifiScanLabel) {
            lv_label_set_text(lvglWifiScanLabel, "Searching for access points");
            lv_obj_set_width(lvglWifiScanLabel, lv_pct(100));
            lv_obj_set_style_text_align(lvglWifiScanLabel, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_set_style_text_color(lvglWifiScanLabel, lv_color_hex(0xC8CED6), 0);
            lvglRegisterReorderableItem(lvglWifiScanLabel, "ord_wifi", "scaning");
        }
    } else if (wifiCount > 0) {
        lv_obj_t *sec = lv_label_create(lvglWifiList);
        lv_label_set_text(sec, "Discovered Networks");
        lv_obj_set_style_text_color(sec, lv_color_hex(0xE5ECF3), 0);
        lvglRegisterReorderableItem(sec, "ord_wifi", "hdr");
        for (int i = 0; i < wifiCount; i++) {
            char line[96];
            snprintf(line, sizeof(line), "%s (%ddBm) [%s]", wifiEntries[i].ssid.c_str(), static_cast<int>(wifiEntries[i].rssi), authName(wifiEntries[i].auth));
            lv_obj_t *networkBtn = lvglCreateMenuButton(
                lvglWifiList,
                lvglSymbolText(LV_SYMBOL_WIFI, line).c_str(),
                (wifiEntries[i].auth == WIFI_AUTH_OPEN) ? lv_color_hex(0x357A38) : lv_color_hex(0x375A7A),
                lvglWifiEntryEvent,
                reinterpret_cast<void *>(static_cast<intptr_t>(i))
            );
            if (networkBtn) {
                const String orderKey = lvglOrderTokenFromText("n", wifiEntries[i].ssid + "|" + String(static_cast<int>(wifiEntries[i].auth)));
                lvglRegisterReorderableItem(networkBtn, "ord_wifi", orderKey.c_str());
            }
        }
    } else {
        lv_obj_t *lbl = lv_label_create(lvglWifiList);
        lv_label_set_text(lbl, "No scan started yet. Tap Scan.");
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xC8CED6), 0);
        lvglRegisterReorderableItem(lbl, "ord_wifi", "empty");
    }

    lv_obj_t *apCard = makeCard("AP Config");
    if (apCard) {
        lvglRegisterReorderableItem(apCard, "ord_wifi", "ap");
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
        lv_obj_add_event_cb(lvglWifiApPassShowBtn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(lvglWifiApPassShowBtn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(lvglWifiApPassShowBtn, lvglWifiApShowToggleEvent, LV_EVENT_CLICKED, nullptr);
        lvglWifiApPassShowBtnLabel = lv_label_create(lvglWifiApPassShowBtn);
        lv_label_set_text(lvglWifiApPassShowBtnLabel, LV_SYMBOL_EYE_CLOSE);
        lv_obj_center(lvglWifiApPassShowBtnLabel);
        lvglRegisterStyledButton(lvglWifiApPassShowBtn, lv_color_hex(0x3F4A57), true);

        makeBtn(apCard, lvglSymbolText(LV_SYMBOL_SAVE, "Save AP Config").c_str(), 144, 30, lv_color_hex(0x3A7A3A), lvglWifiApSaveEvent);
    }
    lvglApplySavedOrder(lvglWifiList, "ord_wifi");
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
    mediaEnsureStorageReadyForUi();
    loadMediaEntries();

    if (mediaOffset > 0) {
        lv_obj_t *btn = lvglCreateMenuButton(lvglMediaList, lvglSymbolText(LV_SYMBOL_PREV, "Prev page").c_str(), lv_color_hex(0x3F4A57), lvglMediaEntryEvent, reinterpret_cast<void *>(static_cast<intptr_t>(MEDIA_ENTRY_PREV_PAGE)));
        if (btn) lvglRegisterReorderableItem(btn, "ord_media", "pg_prev");
    }
    if (mediaCurrentDir != "/") {
        lv_obj_t *btn = lvglCreateMenuButton(lvglMediaList, lvglSymbolText(LV_SYMBOL_DIRECTORY, "Parent").c_str(), lv_color_hex(0x4E5D6C), lvglMediaEntryEvent, reinterpret_cast<void *>(static_cast<intptr_t>(MEDIA_ENTRY_PARENT)));
        if (btn) lvglRegisterReorderableItem(btn, "ord_media", "parent");
    }
    for (int i = 0; i < mediaCount; i++) {
        String label = lvglSymbolText(mediaEntries[i].isDir ? LV_SYMBOL_DIRECTORY : LV_SYMBOL_FILE,
                                      mediaBuildListLabel(mediaEntries[i].name, mediaEntries[i].isDir, mediaEntries[i].size));
        lv_obj_t *entryBtn = lvglCreateMenuButton(
            lvglMediaList,
            label.c_str(),
            mediaEntries[i].isDir ? lv_color_hex(0x2E7D9A) : lv_color_hex(0x355C3D),
            lvglMediaEntryEvent,
            reinterpret_cast<void *>(static_cast<intptr_t>(i))
        );
        if (entryBtn) {
            const String orderKey = lvglOrderTokenFromText("m", mediaEntries[i].path);
            lvglRegisterReorderableItem(entryBtn, "ord_media", orderKey.c_str());
        }
    }
    if (mediaHasMore) {
        lv_obj_t *btn = lvglCreateMenuButton(lvglMediaList, lvglSymbolText(LV_SYMBOL_NEXT, "Next page").c_str(), lv_color_hex(0x3F4A57), lvglMediaEntryEvent, reinterpret_cast<void *>(static_cast<intptr_t>(MEDIA_ENTRY_NEXT_PAGE)));
        if (btn) lvglRegisterReorderableItem(btn, "ord_media", "pg_next");
    }
    if (mediaCount == 0) {
        lv_obj_t *lbl = lv_label_create(lvglMediaList);
        lv_label_set_text(lbl, "No media entries");
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xC8CED6), 0);
        lvglRegisterReorderableItem(lbl, "ord_media", "empty");
    }
    lvglApplySavedOrder(lvglMediaList, "ord_media");
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
                } else if (code == LV_EVENT_READY && hc12CmdTaObj() && lv_keyboard_get_textarea(lvglKb) == hc12CmdTaObj()) {
                    lvglHc12SendEvent(nullptr);
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
    if (ta == lvglConfigDeviceNameTa || ta == hc12CmdTaObj()) lvglSetConfigKeyboardVisible(true);
}

static void hc12InitIfNeeded()
{
    if (radioModuleType == RADIO_MODULE_E220 && (e220ActiveRxPin() < 0 || e220ActiveTxPin() < 0 || e220ActiveM0Pin() < 0 || e220ActiveM1Pin() < 0)) {
        hc12ConfigStatusText = "E220 is only wired on the ESP32-S3 build";
        return;
    }
    if (hc12TerminalLog) return;
    if (radioModuleType == RADIO_MODULE_HC12) {
        pinMode(hc12ActiveSetPin(), OUTPUT);
        digitalWrite(hc12ActiveSetPin(), HIGH);
        Serial1.begin(HC12_BAUD, SERIAL_8N1, hc12ActiveRxPin(), hc12ActiveTxPin());
    } else {
        pinMode(e220ActiveM0Pin(), OUTPUT);
        pinMode(e220ActiveM1Pin(), OUTPUT);
        digitalWrite(e220ActiveM0Pin(), LOW);
        digitalWrite(e220ActiveM1Pin(), LOW);
        Serial1.begin(E220_AT_BAUD, SERIAL_8N1, e220ActiveRxPin(), e220ActiveTxPin());
    }
    String banner = String("[RADIO] Serial1 ready on ESP RX") +
                    (radioModuleType == RADIO_MODULE_HC12 ? hc12ActiveRxPin() : e220ActiveRxPin()) +
                    " TX" +
                    (radioModuleType == RADIO_MODULE_HC12 ? hc12ActiveTxPin() : e220ActiveTxPin()) + "\n";
    if (radioModuleType == RADIO_MODULE_HC12) {
        banner += String("[HC12] HC12 RXD->GPIO") + hc12ActiveTxPin() + " TXD->GPIO" + hc12ActiveRxPin() + " SET->GPIO" + hc12ActiveSetPin() + "\n";
    } else {
        banner += String("[E220] RX->GPIO") + e220ActiveTxPin() + " TX->GPIO" + e220ActiveRxPin() + " M0->GPIO" + e220ActiveM0Pin() + " M1->GPIO" + e220ActiveM1Pin() + "\n";
    }
    hc12AppendTerminal(banner.c_str());
}

static String hc12TerminalExampleCommands()
{
    if (radioModuleType == RADIO_MODULE_E220) {
        return "Examples: AT, AT+CHANNEL=?, AT+UART=?, AT+POWER=?, AT+TRANS=?";
    }
    return "Examples: AT, AT+RX, AT+RC, AT+RB, AT+RF, AT+RP";
}

static void hc12RestartWithCurrentPins(const String &statusText)
{
    Serial1.flush();
    Serial1.end();
    uiDeferredFlags &= static_cast<uint8_t>(~(UI_DEFERRED_HC12_SETTLE_PENDING | UI_DEFERRED_HC12_TARGET_ASSERTED));
    delete hc12TerminalLog;
    hc12TerminalLog = nullptr;
    delete hc12RadioRxLine;
    hc12RadioRxLine = nullptr;
    if (!statusText.isEmpty()) hc12ConfigStatusText = statusText;
    hc12InitIfNeeded();
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
    return wrap ? lv_obj_get_child(wrap, 1) : nullptr;
}

static lv_obj_t *hc12CmdTaObj()
{
    lv_obj_t *wrap = hc12WrapObj();
    lv_obj_t *cmdRow = wrap ? lv_obj_get_child(wrap, 2) : nullptr;
    return cmdRow ? lv_obj_get_child(cmdRow, 0) : nullptr;
}

static bool hc12SetIsAsserted()
{
    if (!hc12TerminalLog) return false;
    if (radioModuleType == RADIO_MODULE_HC12) return digitalRead(hc12ActiveSetPin()) == LOW;
    return digitalRead(e220ActiveM0Pin()) == HIGH && digitalRead(e220ActiveM1Pin()) == HIGH;
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
    while (Serial1.available() > 0) Serial1.read();
    Serial1.print(line);
    if (radioModuleType == RADIO_MODULE_E220) Serial1.print("\r\n");
    Serial1.flush();
    String echoed = String("> ") + line + "\n";
    hc12AppendTerminal(echoed.c_str());
}

static String hc12QueryCommand(const char *line, unsigned long totalTimeoutMs = 180UL, unsigned long quietTimeoutMs = 32UL)
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

static String e220QueryCommand(const char *line, unsigned long totalTimeoutMs = 220UL, unsigned long quietTimeoutMs = 48UL)
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
    } else {
        digitalWrite(e220ActiveM0Pin(), HIGH);
        digitalWrite(e220ActiveM1Pin(), HIGH);
    }
    delay(80);
}

static void hc12ExitAtMode()
{
    if (radioModuleType == RADIO_MODULE_HC12) {
        digitalWrite(hc12ActiveSetPin(), HIGH);
    } else {
        digitalWrite(e220ActiveM0Pin(), LOW);
        digitalWrite(e220ActiveM1Pin(), LOW);
    }
    delay(40);
    uiDeferredFlags &= static_cast<uint8_t>(~(UI_DEFERRED_HC12_SETTLE_PENDING | UI_DEFERRED_HC12_TARGET_ASSERTED));
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
        hc12ExitAtMode();
        if (resp.indexOf("OK") >= 0) {
            e220CurrentBaudIndex = index;
            savePersistedRadioSettings();
            hc12ConfigStatusText = "UART baud set to " + String(E220_UART_BAUD_VALUES[index]) + " bps";
            return true;
        }
        hc12ConfigStatusText = resp.isEmpty() ? "Baud change failed" : resp;
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
        hc12EnterAtMode();
        const String resp = e220QueryCommand("AT+CFGTF");
        hc12ExitAtMode();
        if (resp.indexOf("OK") >= 0) {
            e220SwapUartPins = false;
            e220SwapModePins = false;
            e220CurrentChannel = 23;
            e220CurrentBaudIndex = 3;
            e220CurrentAirRateIndex = 2;
            e220CurrentPowerIndex = 0;
            e220CurrentFixedTransmission = false;
            savePersistedRadioSettings();
            hc12ConfigStatusText = "Factory defaults restored";
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
        const String modelRaw = e220QueryCommand("AT+DEVTYPE=?");
        const String fwRaw = e220QueryCommand("AT+FWCODE=?");
        const String uartRaw = e220QueryCommand("AT+UART=?");
        const String channelRaw = e220QueryCommand("AT+CHANNEL=?");
        const String rateRaw = e220QueryCommand("AT+RATE=?");
        const String powerRaw = e220QueryCommand("AT+POWER=?");
        const String transRaw = e220QueryCommand("AT+TRANS=?");
        hc12ExitAtMode();
        hc12InfoValueText = e220ValueAfterEquals(modelRaw);
        if (hc12InfoValueText.isEmpty()) hc12InfoValueText = "No Reply";
        hc12InfoSubText = "FW " + e220ValueAfterEquals(fwRaw);
        if (lvglHc12InfoVersionLabel) lv_label_set_text(lvglHc12InfoVersionLabel, hc12InfoValueText.c_str());
        if (lvglHc12InfoBaudLabel) lv_label_set_text(lvglHc12InfoBaudLabel, e220ValueAfterEquals(uartRaw).c_str());
        if (lvglHc12InfoChannelLabel) lv_label_set_text(lvglHc12InfoChannelLabel, e220ValueAfterEquals(channelRaw).c_str());
        if (lvglHc12InfoFuModeLabel) lv_label_set_text(lvglHc12InfoFuModeLabel, e220ValueAfterEquals(transRaw).c_str());
        if (lvglHc12InfoPowerLabel) lv_label_set_text(lvglHc12InfoPowerLabel, e220ValueAfterEquals(powerRaw).c_str());
        if (lvglHc12InfoRawLabel) {
            String raw = String("UART ") + e220ValueAfterEquals(uartRaw) + " | RATE " + e220ValueAfterEquals(rateRaw) +
                         " | TRANS " + e220ValueAfterEquals(transRaw);
            lv_label_set_text(lvglHc12InfoRawLabel, raw.c_str());
        }
        return;
    }
    hc12InitIfNeeded();
    hc12InfoValueText = "Querying...";
    hc12InfoSubText = "Entering AT mode...";

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

static bool hc12SendRadioDocument(JsonDocument &doc)
{
    hc12InitIfNeeded();
    if (hc12SetIsAsserted() || !radioModuleCanCarryChat()) return false;
    char payload[HC12_RADIO_MAX_LINE] = {0};
    const size_t len = serializeJson(doc, payload, sizeof(payload));
    if (len == 0 || len >= sizeof(payload)) return false;
    Serial1.print(HC12_RADIO_FRAME_PREFIX);
    Serial1.write(reinterpret_cast<const uint8_t *>(payload), len);
    Serial1.write('\n');
    Serial1.flush();
    return true;
}

static bool hc12SendRadioPayload(const char *payload, size_t len)
{
    hc12InitIfNeeded();
    if (hc12SetIsAsserted() || !radioModuleCanCarryChat() || !payload || len == 0 || len >= HC12_RADIO_MAX_LINE) return false;
    Serial1.print(HC12_RADIO_FRAME_PREFIX);
    Serial1.write(reinterpret_cast<const uint8_t *>(payload), len);
    Serial1.write('\n');
    Serial1.flush();
    return true;
}

static bool hc12SendEncryptedPayload(const String &peerKey, const uint8_t *plain, size_t plainLen)
{
    if (peerKey.isEmpty() || !plain || plainLen == 0 || plainLen > (P2P_MAX_PACKET - 64U)) return false;

    unsigned char peerPk[P2P_PUBLIC_KEY_BYTES] = {0};
    if (!p2pHexToBytes(peerKey, peerPk, sizeof(peerPk))) return false;

    unsigned char nonce[P2P_NONCE_BYTES] = {0};
    unsigned char cipher[P2P_MAX_PACKET] = {0};
    randombytes_buf(nonce, sizeof(nonce));
    if (crypto_box_curve25519xchacha20poly1305_easy(cipher, plain, plainLen, nonce, peerPk, p2pSecretKey) != 0) return false;

    const size_t cipherLen = plainLen + P2P_MAC_BYTES;
    char senderPubHex[(P2P_PUBLIC_KEY_BYTES * 2U) + 1U] = {0};
    char nonceHex[(P2P_NONCE_BYTES * 2U) + 1U] = {0};
    char cipherHex[(P2P_MAX_PACKET * 2U) + 1U] = {0};
    char payload[HC12_RADIO_MAX_LINE] = {0};
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
    return hc12SendRadioPayload(payload, static_cast<size_t>(written));
}

static bool hc12BroadcastDiscoveryFrame(const char *kind)
{
    JsonDocument doc;
    doc["kind"] = kind ? kind : "hello";
    doc["device"] = deviceShortNameValue();
    doc["public_key"] = p2pPublicKeyHex();
    return hc12SendRadioDocument(doc);
}

static void hc12DiscoveryService()
{
    if (hc12SetIsAsserted() || !radioModuleCanCarryChat()) return;
    const unsigned long now = millis();
    if (static_cast<unsigned long>(now - hc12LastDiscoveryAnnounceMs) < HC12_DISCOVERY_INTERVAL_MS) return;
    hc12LastDiscoveryAnnounceMs = now;
    hc12BroadcastDiscoveryFrame("hello");
}

bool hc12SendChatMessageWithId(const String &peerKey, const String &text, const String &messageId)
{
    if (peerKey.isEmpty() || text.isEmpty() || messageId.isEmpty()) return false;
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
    JsonDocument doc;
    doc["kind"] = "ack";
    doc["ack_id"] = messageId;
    char plain[P2P_MAX_PACKET] = {0};
    const size_t plainLen = serializeJson(doc, plain, sizeof(plain));
    if (plainLen == 0) return false;
    return hc12SendEncryptedPayload(peerKey, reinterpret_cast<const uint8_t *>(plain), plainLen);
}

bool hc12SendMessageDelete(const String &peerKey, const String &messageId)
{
    if (peerKey.isEmpty() || messageId.isEmpty()) return false;
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
    if (deserializeJson(outerDoc, line.substring(strlen(HC12_RADIO_FRAME_PREFIX))) != DeserializationError::Ok) return;

    const String discoveryKind = String(static_cast<const char *>(outerDoc["kind"] | ""));
    const String discoveryPubHex = String(static_cast<const char *>(outerDoc["public_key"] | ""));
    if (!discoveryKind.isEmpty() && !discoveryPubHex.isEmpty()) {
        if (discoveryPubHex.equalsIgnoreCase(p2pPublicKeyHex())) return;
        const String senderName = sanitizeDeviceShortName(String(static_cast<const char *>(outerDoc["device"] | "Peer")));
        hc12TouchDiscoveredPeer(senderName, discoveryPubHex);
        if (discoveryKind == "probe") hc12BroadcastDiscoveryFrame("hello");
        return;
    }

    const String senderPubHex = String(static_cast<const char *>(outerDoc["sender_pub"] | ""));
    const String nonceHex = String(static_cast<const char *>(outerDoc["nonce"] | ""));
    const String cipherHex = String(static_cast<const char *>(outerDoc["cipher"] | ""));
    if (senderPubHex.isEmpty() || nonceHex.isEmpty() || cipherHex.isEmpty() || senderPubHex.equalsIgnoreCase(p2pPublicKeyHex())) return;

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

    JsonDocument doc;
    if (deserializeJson(doc, plain, cipherLen - P2P_MAC_BYTES) != DeserializationError::Ok) return;

    const String author = sanitizeDeviceShortName(String(static_cast<const char *>(doc["author"] | chatDisplayNameForPeerKey(senderPubHex).c_str())));
    hc12TouchDiscoveredPeer(author, senderPubHex);
    const String kind = String(static_cast<const char *>(doc["kind"] | ""));
    if (kind == "chat") {
        const String messageId = String(static_cast<const char *>(doc["id"] | ""));
        const String text = String(static_cast<const char *>(doc["text"] | ""));
        if (text.isEmpty()) return;
        wakeDisplayForIncomingNotification();
        if (checkersHandleIncomingChatPayload(senderPubHex, author, text, radioTransport, messageId)) {
            if (lvglReady) {
                lvglSyncStatusLine();
                if (uiScreen == UI_CHAT) lvglRefreshChatUi();
            }
        } else if (messageId.isEmpty() || !chatHasLoggedMessageId(senderPubHex, messageId)) {
            chatStoreMessage(senderPubHex, author, text, false, radioTransport, messageId);
            chatQueueIncomingMessageBeep();
            uiStatusLine = String(radioModuleChatLabel()) + " chat from " + author;
            if (lvglReady) {
                lvglSyncStatusLine();
                if (uiScreen == UI_CHAT) lvglRefreshChatUi();
            }
        }
        if (!messageId.isEmpty()) hc12SendChatAck(senderPubHex, messageId);
        return;
    }

    if (kind == "ack") {
        const String ackId = String(static_cast<const char *>(doc["ack_id"] | ""));
        if (!ackId.isEmpty()) chatAckOutgoingMessage(senderPubHex, ackId);
        return;
    }

    if (kind == "delete_message") {
        const String messageId = String(static_cast<const char *>(doc["id"] | ""));
        if (!messageId.isEmpty()) chatDeleteMessageById(senderPubHex, messageId, "Message deleted by " + author);
        return;
    }

    if (kind == "delete_conversation") chatApplyConversationDeletion(senderPubHex, "Conversation deleted by " + author);
}

void lvglHc12ToggleSetEvent(lv_event_t *e)
{
    (void)e;
    hc12InitIfNeeded();
    const bool setAsserted = !hc12SetIsAsserted();
    if (radioModuleType == RADIO_MODULE_HC12) {
        digitalWrite(hc12ActiveSetPin(), setAsserted ? LOW : HIGH);
    } else {
        digitalWrite(e220ActiveM0Pin(), setAsserted ? HIGH : LOW);
        digitalWrite(e220ActiveM1Pin(), setAsserted ? HIGH : LOW);
    }
    uiDeferredFlags |= UI_DEFERRED_HC12_SETTLE_PENDING;
    if (setAsserted) uiDeferredFlags |= UI_DEFERRED_HC12_TARGET_ASSERTED;
    else uiDeferredFlags &= static_cast<uint8_t>(~UI_DEFERRED_HC12_TARGET_ASSERTED);
    hc12SettleUntilTick = static_cast<uint16_t>(millis()) + static_cast<uint16_t>(setAsserted ? 80U : 40U);
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
        if (rxLine.length() < (HC12_RADIO_MAX_LINE - 1)) {
            rxLine += static_cast<char>(ch);
        } else {
            rxLine = "";
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
            lv_obj_add_event_cb(btn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
            lv_obj_add_event_cb(btn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
            lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, nullptr);
            lv_obj_t *lbl = lv_label_create(btn);
            if (lbl) {
                lv_label_set_text(lbl, txt);
                lv_obj_center(lbl);
            }
            lvglRegisterStyledButton(btn, color, true);
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

void lvglP2pPairPromptEvent(lv_event_t *e)
{
    lv_obj_t *msgbox = lv_event_get_current_target(e);
    const char *txt = lv_msgbox_get_active_btn_text(msgbox);
    const bool accepted = txt && strcmp(txt, "Accept") == 0;
    const int discoveredIdx = p2pPairRequestDiscoveredIdx;
    const bool requestValid = discoveredIdx >= 0 && discoveredIdx < p2pDiscoveredCount;
    if (accepted) {
        bool ok = false;
        if (requestValid) {
            const P2PDiscoveredPeer &peer = p2pDiscoveredPeers[discoveredIdx];
            ok = p2pAddOrUpdateTrustedPeer(
                peer.name.isEmpty() ? String("Peer") : peer.name,
                peer.pubKeyHex,
                peer.ip,
                peer.port);
            p2pSendPairResponse(peer.name, peer.pubKeyHex, peer.ip, peer.port, ok);
            if (ok && currentChatPeerKey.isEmpty()) currentChatPeerKey = peer.pubKeyHex;
        }
        uiStatusLine = ok ? "Peer paired" : "Pair accept failed";
        if (lvglReady) {
            lvglSyncStatusLine();
            lvglRefreshChatPeerUi();
            lvglRefreshChatContactsUi();
        }
    } else {
        if (requestValid) {
            const P2PDiscoveredPeer &peer = p2pDiscoveredPeers[discoveredIdx];
            p2pSendPairResponse(peer.name, peer.pubKeyHex, peer.ip, peer.port, false);
        }
        uiStatusLine = "Pair request rejected";
        if (lvglReady) lvglSyncStatusLine();
    }
    p2pPairRequestPending = false;
    p2pPairPromptVisible = false;
    p2pPairRequestDiscoveredIdx = -1;
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
    lv_obj_t *msgbox = lv_event_get_current_target(e);
    otaClearPopupVersion();
    lvglOtaPostUpdatePopupVisible = false;
    lv_msgbox_close(msgbox);
}

void lvglMqttControlPressEvent(lv_event_t *e)
{
    const int idx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (idx < 0 || idx >= mqttButtonCount) return;
    if (mqttButtonCritical[idx]) {
        static const char *btns[] = {"No", "Yes", ""};
        lv_obj_t *m = lv_msgbox_create(nullptr, "Confirm", mqttButtonNames[idx].c_str(), btns, false);
        if (!m) return;
        lvglApplyMsgboxModalStyle(m);
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
        lv_obj_add_event_cb(b, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(b, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
        lv_obj_t *l = lv_label_create(b);
        if (l) {
            lv_label_set_text(l, txt);
            lv_obj_center(l);
        }
        lvglRegisterStyledButton(b, col, true);
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
    lvglRegisterReorderableItem(editCard, "ord_mctl", "edit");

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
    makeSmallBtnLocal(editActRow, lvglSymbolText(LV_SYMBOL_OK, "Apply Btn").c_str(), 94, 30, lv_color_hex(0x6D4B9A), lvglMqttApplyBtnEvent);

    for (int i = 0; i < mqttButtonCount; i++) {
        String lbl = lvglSymbolText(LV_SYMBOL_LIST, mqttButtonNames[i]);
        if (mqttButtonCritical[i]) lbl += " (!)";
        lv_obj_t *btn = lvglCreateMenuButton(
            lvglMqttCtrlList,
            lbl.c_str(),
            mqttButtonCritical[i] ? lv_color_hex(0x9A5A2E) : lv_color_hex(0x2D6D8E),
            lvglMqttControlPressEvent,
            reinterpret_cast<void *>(static_cast<intptr_t>(i))
        );
        if (btn) {
            char key[16];
            snprintf(key, sizeof(key), "btn_%02d", i);
            lvglRegisterReorderableItem(btn, "ord_mctl", key);
        }
    }
    lvglApplySavedOrder(lvglMqttCtrlList, "ord_mctl");
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

void lvglSnakeDirEvent(lv_event_t *e)
{
    if (!snakeStarted || snakePaused || snakeGameOver) return;
    const int d = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (d == 3 && snakeDir != 1) snakeNextDir = 3;
    else if (d == 1 && snakeDir != 3) snakeNextDir = 1;
    else if (d == 0 && snakeDir != 2) snakeNextDir = 0;
    else if (d == 2 && snakeDir != 0) snakeNextDir = 2;
}

void lvglSnakePauseEvent(lv_event_t *e)
{
    (void)e;
    if (snakeGameOver) return;
    if (!snakeStarted) {
        snakeStartGame();
    } else {
        snakePaused = !snakePaused;
        if (!snakePaused) snakeLastStepMs = millis();
    }
    lvglRefreshSnakeBoard();
}

void lvglSnakeBackEvent(lv_event_t *e)
{
    (void)e;
    lvglOpenScreen(UI_GAMES, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
}

void lvglSnakeRestartEvent(lv_event_t *e)
{
    (void)e;
    snakeStartGame();
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
    tetrisStartGame();
    lvglRefreshTetrisBoard();
}

void lvglTetrisPauseEvent(lv_event_t *e)
{
    (void)e;
    if (tetrisGameOver) return;
    if (tetrisDropAnimFlags & TETRIS_DROP_ANIM_ACTIVE) return;
    if (!tetrisStarted) {
        tetrisStartGame();
    } else {
        tetrisPaused = !tetrisPaused;
        if (!tetrisPaused) tetrisLastStepMs = millis();
    }
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

    const bool dropAnimating = (tetrisDropAnimFlags & TETRIS_DROP_ANIM_ACTIVE) != 0;
    const int drawPieceBaseY = dropAnimating ? static_cast<int>(tetrisAnimFromY) : static_cast<int>(tetrisY);
    int animOffsetPx = 0;
    if (dropAnimating && tetrisAnimDurationMs > 0) {
        const uint16_t elapsedTick = static_cast<uint16_t>(millis()) - tetrisAnimStartTick;
        const unsigned long elapsed = min<unsigned long>(elapsedTick, tetrisAnimDurationMs);
        const int totalRows = static_cast<int>(tetrisY - tetrisAnimFromY);
        const int stepPx = cellH + gap;
        animOffsetPx = static_cast<int>((static_cast<uint64_t>(stepPx * totalRows) * elapsed) / tetrisAnimDurationMs);
    }

    for (int i = 0; i < 4; i++) {
        int ox = 0, oy = 0;
        tetrisCellFor(tetrisType, tetrisRot, i, ox, oy);
        int gx = tetrisX + ox;
        int gy = drawPieceBaseY + oy;
        if (gx < 0 || gx >= TETRIS_COLS || gy < 0 || gy >= TETRIS_ROWS) continue;
        const int px = static_cast<int>(coords.x1) + gx * (cellW + gap);
        const int py = static_cast<int>(coords.y1) + gy * (cellH + gap) + animOffsetPx;
        lvglDrawGridCell(drawCtx, px, py, cellW, cellH, cols[(tetrisType + 1) & 0x07]);
    }
}

void lvglRefreshSnakeBoard()
{
    if (lvglSnakeBoardObj) lv_obj_invalidate(lvglSnakeBoardObj);
    if (lvglSnakeScoreLabel) {
        lv_label_set_text_fmt(lvglSnakeScoreLabel, "Score %u", snakeScore);
    }
    if (lvglSnakeBestLabel) {
        lv_label_set_text_fmt(lvglSnakeBestLabel, "Best %u", snakeHighScore);
    }
    if (lvglSnakeOverlay) {
        const bool showOverlay = !snakeStarted || snakeGameOver || snakePaused;
        const bool pauseOverlay = snakePaused && snakeStarted && !snakeGameOver;
        lv_coord_t overlayW = pauseOverlay ? min<lv_coord_t>(DISPLAY_WIDTH - 64, 140) : min<lv_coord_t>(DISPLAY_WIDTH - 20, 324);
        lv_coord_t overlayH = pauseOverlay ? 76 : min<lv_coord_t>(UI_CONTENT_H - 8, 467);
        if (lvglSnakeBoardObj) {
            lv_area_t boardCoords;
            lv_obj_get_coords(lvglSnakeBoardObj, &boardCoords);
            const lv_coord_t boardWidth = static_cast<lv_coord_t>(boardCoords.x2 - boardCoords.x1 + 1);
            const lv_coord_t boardHeight = static_cast<lv_coord_t>(boardCoords.y2 - boardCoords.y1 + 1);
            overlayW = min<lv_coord_t>(overlayW, max<lv_coord_t>(120, boardWidth - 16));
            overlayH = min<lv_coord_t>(overlayH, max<lv_coord_t>(137, boardHeight - 16));
        }
        lv_obj_set_size(lvglSnakeOverlay, overlayW, overlayH);
        lv_obj_center(lvglSnakeOverlay);
        lv_obj_move_foreground(lvglSnakeOverlay);
        if (showOverlay) lv_obj_clear_flag(lvglSnakeOverlay, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(lvglSnakeOverlay, LV_OBJ_FLAG_HIDDEN);
        if (lvglSnakeOverlayTitle) {
            if (pauseOverlay) lv_label_set_text(lvglSnakeOverlayTitle, "PAUSE");
            else lv_label_set_text(lvglSnakeOverlayTitle, snakeGameOver ? "Game Over" : "Snake");
        }
        if (lvglSnakeOverlaySubLabel) {
            String msg = pauseOverlay ? String("Tap center button to resume")
                                      : (snakeGameOver ? ("Score " + String(snakeScore) + "\nBest " + String(snakeHighScore))
                                                       : ("Tap Start when ready\nBest " + String(snakeHighScore) + "\nLast " + String(snakeLastScore)));
            lv_label_set_text(lvglSnakeOverlaySubLabel, msg.c_str());
        }
        if (lvglSnakeOverlayBtn) {
            if (pauseOverlay) lv_obj_add_flag(lvglSnakeOverlayBtn, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_clear_flag(lvglSnakeOverlayBtn, LV_OBJ_FLAG_HIDDEN);
        }
        if (lvglSnakeOverlayBtnLabel && !pauseOverlay) lv_label_set_text(lvglSnakeOverlayBtnLabel, snakeGameOver ? "Replay" : "Start");
    }
    if (lvglSnakePauseBtnLabel) lv_label_set_text(lvglSnakePauseBtnLabel, (!snakeStarted || snakePaused) ? LV_SYMBOL_PLAY : LV_SYMBOL_PAUSE);
}

#if defined(BOARD_ESP32S3_3248S035_N16R8)
struct Snake3dVec {
    float x;
    float y;
    float z;
};

struct Snake3dCamera {
    Snake3dVec pos;
    Snake3dVec forward;
    Snake3dVec right;
    Snake3dVec up;
};

static Snake3dVec snake3dVecSub(const Snake3dVec &a, const Snake3dVec &b)
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

static Snake3dVec snake3dVecAdd(const Snake3dVec &a, const Snake3dVec &b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

static Snake3dVec snake3dVecScale(const Snake3dVec &v, float s)
{
    return {v.x * s, v.y * s, v.z * s};
}

static float snake3dVecDot(const Snake3dVec &a, const Snake3dVec &b)
{
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

static Snake3dVec snake3dVecCross(const Snake3dVec &a, const Snake3dVec &b)
{
    return {
        (a.y * b.z) - (a.z * b.y),
        (a.z * b.x) - (a.x * b.z),
        (a.x * b.y) - (a.y * b.x)
    };
}

static Snake3dVec snake3dVecNormalize(const Snake3dVec &v)
{
    const float len = sqrtf((v.x * v.x) + (v.y * v.y) + (v.z * v.z));
    if (len <= 0.0001f) return {0.0f, 0.0f, 1.0f};
    return {v.x / len, v.y / len, v.z / len};
}

static lv_color_t snake3dMixColor(lv_color_t a, lv_color_t b, uint8_t mix)
{
    return lv_color_mix(a, b, mix);
}

static void snake3dFillQuad(lv_draw_ctx_t *drawCtx, const lv_point_t pts[4], lv_color_t color, lv_opa_t opa = LV_OPA_COVER)
{
    if (!drawCtx) return;
    int minY = pts[0].y;
    int maxY = pts[0].y;
    for (int i = 1; i < 4; ++i) {
        if (pts[i].y < minY) minY = pts[i].y;
        if (pts[i].y > maxY) maxY = pts[i].y;
    }
    for (int y = minY; y <= maxY; ++y) {
        int hits[8];
        int hitCount = 0;
        for (int i = 0; i < 4; ++i) {
            const lv_point_t a = pts[i];
            const lv_point_t b = pts[(i + 1) & 3];
            if (a.y == b.y) continue;
            const int yMin = min(a.y, b.y);
            const int yMax = max(a.y, b.y);
            if (y < yMin || y >= yMax) continue;
            const float t = static_cast<float>(y - a.y) / static_cast<float>(b.y - a.y);
            hits[hitCount++] = a.x + static_cast<int>((b.x - a.x) * t);
        }
        if (hitCount < 2) continue;
        if (hits[1] < hits[0]) {
            const int tmp = hits[0];
            hits[0] = hits[1];
            hits[1] = tmp;
        }
        lvglDrawLineSeg(drawCtx, hits[0], y, hits[1], y, color, 1, opa);
    }
}

static Snake3dVec snake3dGridToWorld(float gx, float gy, float gz, float spacing)
{
    Snake3dVec v = {};
    v.x = (gx - ((SNAKE3D_COLS - 1) * 0.5f)) * spacing;
    v.y = gz * (spacing * 0.78f);
    v.z = (((SNAKE3D_ROWS - 1) * 0.5f) - gy) * spacing;
    return v;
}

static Snake3dVec snake3dForwardForDir(int8_t dir)
{
    switch (dir) {
        case 0: return {0.0f, 0.0f, 1.0f};
        case 1: return {1.0f, 0.0f, 0.0f};
        case 2: return {0.0f, 0.0f, -1.0f};
        case 3: return {-1.0f, 0.0f, 0.0f};
        default: return snake3dForwardForDir(snake3dViewDir);
    }
}

static Snake3dCamera snake3dBuildCamera(float spacing)
{
    const Snake3dVec head = snake3dGridToWorld(static_cast<float>(snake3dX[0]),
                                               static_cast<float>(snake3dY[0]),
                                               static_cast<float>(snake3dZ[0]),
                                               spacing);
    const Snake3dVec globalUp = {0.0f, 1.0f, 0.0f};
    const Snake3dVec planarForward = snake3dForwardForDir(snake3dViewDir);
    const Snake3dVec camPos = snake3dVecAdd(head,
                                            snake3dVecAdd(snake3dVecScale(planarForward, -spacing * 5.4f),
                                                          {0.0f, spacing * 3.9f, 0.0f}));
    const Snake3dVec target = snake3dVecAdd(head,
                                            snake3dVecAdd(snake3dVecScale(planarForward, spacing * 2.8f),
                                                          {0.0f, spacing * 0.15f, 0.0f}));
    Snake3dCamera cam = {};
    cam.pos = camPos;
    cam.forward = snake3dVecNormalize(snake3dVecSub(target, camPos));
    cam.right = snake3dVecNormalize(snake3dVecCross(cam.forward, globalUp));
    if (fabsf(cam.right.x) < 0.001f && fabsf(cam.right.y) < 0.001f && fabsf(cam.right.z) < 0.001f) cam.right = {1.0f, 0.0f, 0.0f};
    cam.up = snake3dVecNormalize(snake3dVecCross(cam.right, cam.forward));
    return cam;
}

static bool snake3dProjectPoint(const Snake3dVec &v, const Snake3dCamera &cam, int cx, int cy, float focal, int &sx, int &sy, float &depth)
{
    const Snake3dVec rel = snake3dVecSub(v, cam.pos);
    const float rx = snake3dVecDot(rel, cam.right);
    const float ry = snake3dVecDot(rel, cam.up);
    depth = snake3dVecDot(rel, cam.forward);
    if (depth <= 4.0f) return false;
    const float scale = focal / depth;
    sx = cx + static_cast<int>(rx * scale);
    sy = cy - static_cast<int>(ry * scale);
    return true;
}

static void snake3dDrawCube(lv_draw_ctx_t *drawCtx, int cx, int cy, float focal, const Snake3dCamera &cam, const Snake3dVec &center, float halfSize, lv_color_t color, uint8_t width)
{
    static const uint8_t faces[6][4] = {
        {0, 1, 2, 3},
        {4, 5, 6, 7},
        {0, 1, 5, 4},
        {1, 2, 6, 5},
        {2, 3, 7, 6},
        {3, 0, 4, 7}
    };
    const float offs[8][3] = {
        {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
        {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}
    };
    Snake3dVec verts[8] = {};
    int px[8] = {};
    int py[8] = {};
    float pz[8] = {};
    bool ok[8] = {};
    for (int i = 0; i < 8; ++i) {
        verts[i] = {
            center.x + (offs[i][0] * halfSize),
            center.y + (offs[i][1] * halfSize),
            center.z + (offs[i][2] * halfSize)
        };
        ok[i] = snake3dProjectPoint(verts[i], cam, cx, cy, focal, px[i], py[i], pz[i]);
    }

    struct FaceInfo {
        int idx;
        float depth;
    };
    FaceInfo visible[6];
    int visibleCount = 0;
    for (int i = 0; i < 6; ++i) {
        const uint8_t a = faces[i][0];
        const uint8_t b = faces[i][1];
        const uint8_t c = faces[i][2];
        const uint8_t d = faces[i][3];
        if (!(ok[a] && ok[b] && ok[c] && ok[d])) continue;
        const Snake3dVec ab = snake3dVecSub(verts[b], verts[a]);
        const Snake3dVec ac = snake3dVecSub(verts[c], verts[a]);
        const Snake3dVec normal = snake3dVecCross(ab, ac);
        const Snake3dVec toCam = snake3dVecSub(cam.pos, verts[a]);
        if (snake3dVecDot(normal, toCam) <= 0.0f) continue;
        visible[visibleCount++] = {i, (pz[a] + pz[b] + pz[c] + pz[d]) * 0.25f};
    }

    for (int i = 0; i < visibleCount - 1; ++i) {
        for (int j = i + 1; j < visibleCount; ++j) {
            if (visible[j].depth > visible[i].depth) {
                const FaceInfo tmp = visible[i];
                visible[i] = visible[j];
                visible[j] = tmp;
            }
        }
    }

    for (int i = 0; i < visibleCount; ++i) {
        const uint8_t *face = faces[visible[i].idx];
        lv_point_t quad[4] = {
            {static_cast<lv_coord_t>(px[face[0]]), static_cast<lv_coord_t>(py[face[0]])},
            {static_cast<lv_coord_t>(px[face[1]]), static_cast<lv_coord_t>(py[face[1]])},
            {static_cast<lv_coord_t>(px[face[2]]), static_cast<lv_coord_t>(py[face[2]])},
            {static_cast<lv_coord_t>(px[face[3]]), static_cast<lv_coord_t>(py[face[3]])}
        };
        const uint8_t shade = static_cast<uint8_t>(50 + (i * 35));
        const lv_color_t faceColor = snake3dMixColor(color, lv_color_hex(0x081018), shade);
        snake3dFillQuad(drawCtx, quad, faceColor, LV_OPA_90);
        for (int e = 0; e < 4; ++e) {
            const int n = (e + 1) & 3;
            lvglDrawLineSeg(drawCtx, quad[e].x, quad[e].y, quad[n].x, quad[n].y, snake3dMixColor(color, lv_color_white(), 80), width, LV_OPA_COVER);
        }
    }
}

static void snake3dDrawFloorTile(lv_draw_ctx_t *drawCtx, int cx, int cy, float focal, const Snake3dCamera &cam,
                                 int gx, int gy, int planeCols, int planeRows, float tileSpacing)
{
    const float floorY = -tileSpacing * 0.11f;
    const float tileInset = tileSpacing * 0.05f;
    const float left = (static_cast<float>(gx) - (planeCols * 0.5f)) * tileSpacing + tileInset;
    const float right = left + tileSpacing - (tileInset * 2.0f);
    const float back = (((planeRows * 0.5f) - static_cast<float>(gy)) * tileSpacing) - tileInset;
    const float front = back - tileSpacing + (tileInset * 2.0f);
    Snake3dVec corners[4] = {
        {left, floorY, back},
        {right, floorY, back},
        {right, floorY, front},
        {left, floorY, front}
    };
    int px[4] = {};
    int py[4] = {};
    bool ok[4] = {};
    for (int i = 0; i < 4; ++i) {
        float depth = 0.0f;
        ok[i] = snake3dProjectPoint(corners[i], cam, cx, cy, focal, px[i], py[i], depth);
    }
    if (!(ok[0] && ok[1] && ok[2] && ok[3])) return;

    static const lv_color_t kGroundPalette[4] = {
        lv_color_hex(0x0D6E6E),
        lv_color_hex(0x1D4ED8),
        lv_color_hex(0x7C3AED),
        lv_color_hex(0x0F766E)
    };
    const uint8_t colorIdx = static_cast<uint8_t>((gx + (gy * 3)) & 0x03);
    const lv_color_t fill = kGroundPalette[colorIdx];
    const lv_color_t edge = snake3dMixColor(fill, lv_color_white(), 90);
    const lv_point_t quad[4] = {
        {static_cast<lv_coord_t>(px[0]), static_cast<lv_coord_t>(py[0])},
        {static_cast<lv_coord_t>(px[1]), static_cast<lv_coord_t>(py[1])},
        {static_cast<lv_coord_t>(px[2]), static_cast<lv_coord_t>(py[2])},
        {static_cast<lv_coord_t>(px[3]), static_cast<lv_coord_t>(py[3])}
    };
    snake3dFillQuad(drawCtx, quad, fill, LV_OPA_60);
    lvglDrawLineSeg(drawCtx, px[0], py[0], px[1], py[1], edge, 1, LV_OPA_COVER);
    lvglDrawLineSeg(drawCtx, px[1], py[1], px[2], py[2], edge, 1, LV_OPA_COVER);
    lvglDrawLineSeg(drawCtx, px[2], py[2], px[3], py[3], edge, 1, LV_OPA_COVER);
    lvglDrawLineSeg(drawCtx, px[3], py[3], px[0], py[0], edge, 1, LV_OPA_COVER);
}

void lvglSnake3dBoardDrawEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_DRAW_MAIN) return;
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *drawCtx = lv_event_get_draw_ctx(e);
    if (!obj || !drawCtx) return;

    lv_area_t coords;
    lv_obj_get_coords(obj, &coords);
    const int width = static_cast<int>(coords.x2 - coords.x1 + 1);
    const int height = static_cast<int>(coords.y2 - coords.y1 + 1);
    const int centerX = static_cast<int>(coords.x1) + (width / 2);
    const int centerY = static_cast<int>(coords.y1) + (height / 2) + 30;
    const float spacing = static_cast<float>(min(width, height)) * 0.16f;
    const float groundTileSpacing = spacing * static_cast<float>(SNAKE3D_GROUND_TILE_SPAN);
    const float cubeHalf = spacing * 0.34f;
    const float focal = static_cast<float>(min(width, height)) * 1.35f;
    const Snake3dCamera cam = snake3dBuildCamera(spacing);

    for (int gy = 0; gy < SNAKE3D_GROUND_ROWS; ++gy) {
        for (int gx = 0; gx < SNAKE3D_GROUND_COLS; ++gx) {
            snake3dDrawFloorTile(drawCtx, centerX, centerY, focal, cam, gx, gy,
                                 SNAKE3D_GROUND_COLS, SNAKE3D_GROUND_ROWS, groundTileSpacing);
        }
    }

    for (int i = snake3dLen - 1; i >= 0; --i) {
        const Snake3dVec center = snake3dGridToWorld(static_cast<float>(snake3dX[i]),
                                                     static_cast<float>(snake3dY[i]),
                                                     static_cast<float>(snake3dZ[i]),
                                                     spacing);
        const lv_color_t color = (i == 0) ? lv_color_hex(0xD7FF7A) : lv_color_hex(0x58D06D);
        snake3dDrawCube(drawCtx, centerX, centerY, focal, cam, center, cubeHalf, color, (i == 0) ? 3 : 2);
    }

    const Snake3dVec foodCenter = snake3dGridToWorld(static_cast<float>(snake3dFoodX),
                                                     static_cast<float>(snake3dFoodY),
                                                     static_cast<float>(snake3dFoodZ),
                                                     spacing);
    snake3dDrawCube(drawCtx, centerX, centerY, focal, cam, foodCenter, cubeHalf * 0.8f, lv_color_hex(0xFF6C62), 2);
}

void lvglSnake3dDirEvent(lv_event_t *e)
{
    const int d = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (!snake3dStarted || snake3dPaused || snake3dGameOver) return;
    if ((d == 0 && snake3dDir != 2) || (d == 2 && snake3dDir != 0) ||
        (d == 1 && snake3dDir != 3) || (d == 3 && snake3dDir != 1) ||
        (d == 4 && snake3dDir != 5) || (d == 5 && snake3dDir != 4)) {
        snake3dNextDir = static_cast<int8_t>(d);
        if (d >= 0 && d <= 3) snake3dViewDir = static_cast<int8_t>(d);
    }
}

void lvglSnake3dPauseEvent(lv_event_t *e)
{
    (void)e;
    if (snake3dGameOver) return;
    if (!snake3dStarted) {
        snake3dStartGame();
    } else {
        snake3dPaused = !snake3dPaused;
        if (!snake3dPaused) snake3dLastStepMs = millis();
    }
    lvglRefreshSnake3dBoard();
}

void lvglSnake3dRestartEvent(lv_event_t *e)
{
    (void)e;
    snake3dStartGame();
    lvglRefreshSnake3dBoard();
}

void lvglRefreshSnake3dBoard()
{
    if (lvglSnake3dBoardObj) lv_obj_invalidate(lvglSnake3dBoardObj);
    if (lvglSnake3dScoreLabel) lv_label_set_text_fmt(lvglSnake3dScoreLabel, "Score %u", snake3dScore);
    if (lvglSnake3dBestLabel) lv_label_set_text_fmt(lvglSnake3dBestLabel, "Best %u", snake3dHighScore);
    if (lvglSnake3dPauseBtnLabel) lv_label_set_text(lvglSnake3dPauseBtnLabel, (!snake3dStarted || snake3dPaused) ? LV_SYMBOL_PLAY : LV_SYMBOL_PAUSE);

    if (!lvglSnake3dOverlay) return;
    const bool showOverlay = !snake3dStarted || snake3dPaused || snake3dGameOver;
    const bool pauseOverlay = snake3dPaused && snake3dStarted && !snake3dGameOver;
    lv_coord_t overlayW = pauseOverlay ? min<lv_coord_t>(DISPLAY_WIDTH - 64, 150) : min<lv_coord_t>(DISPLAY_WIDTH - 24, 250);
    lv_coord_t overlayH = pauseOverlay ? 84 : min<lv_coord_t>(UI_CONTENT_H - 24, 255);
    if (lvglSnake3dBoardObj) {
        lv_area_t boardCoords;
        lv_obj_get_coords(lvglSnake3dBoardObj, &boardCoords);
        const lv_coord_t boardWidth = static_cast<lv_coord_t>(boardCoords.x2 - boardCoords.x1 + 1);
        const lv_coord_t boardHeight = static_cast<lv_coord_t>(boardCoords.y2 - boardCoords.y1 + 1);
        overlayW = min<lv_coord_t>(overlayW, max<lv_coord_t>(140, boardWidth - 16));
        overlayH = min<lv_coord_t>(overlayH, max<lv_coord_t>(84, boardHeight - 16));
    }
    lv_obj_set_size(lvglSnake3dOverlay, overlayW, overlayH);
    lv_obj_center(lvglSnake3dOverlay);
    lv_obj_move_foreground(lvglSnake3dOverlay);
    if (showOverlay) lv_obj_clear_flag(lvglSnake3dOverlay, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(lvglSnake3dOverlay, LV_OBJ_FLAG_HIDDEN);

    if (lvglSnake3dOverlayTitle) {
        if (pauseOverlay) lv_label_set_text(lvglSnake3dOverlayTitle, "PAUSE");
        else lv_label_set_text(lvglSnake3dOverlayTitle, snake3dGameOver ? "Game Over" : "Snake 3D");
    }
        if (lvglSnake3dOverlaySubLabel) {
            String msg = pauseOverlay ? String("Resume to keep climbing the 3D grid")
                                      : (snake3dGameOver ? ("Score " + String(snake3dScore) + "\nBest " + String(snake3dHighScore))
                                                     : ("Chase camera 3D snake\nBest " + String(snake3dHighScore) + "  Last " + String(snake3dLastScore)));
        lv_label_set_text(lvglSnake3dOverlaySubLabel, msg.c_str());
    }
    if (lvglSnake3dOverlayBtn) {
        if (pauseOverlay) lv_obj_add_flag(lvglSnake3dOverlayBtn, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_clear_flag(lvglSnake3dOverlayBtn, LV_OBJ_FLAG_HIDDEN);
    }
    if (lvglSnake3dOverlayBtnLabel && !pauseOverlay) lv_label_set_text(lvglSnake3dOverlayBtnLabel, snake3dGameOver ? "Replay" : "Start");
}
#endif

void lvglRefreshTetrisBoard()
{
    if (lvglTetrisBoardObj) lv_obj_invalidate(lvglTetrisBoardObj);
    if (lvglTetrisScoreLabel) {
        lv_label_set_text_fmt(lvglTetrisScoreLabel, "Score %u", tetrisScore);
    }
    if (lvglTetrisBestLabel) {
        lv_label_set_text_fmt(lvglTetrisBestLabel, "Best %u", tetrisHighScore);
    }
    if (lvglTetrisOverlay) {
        const bool showOverlay = !tetrisStarted || tetrisGameOver || tetrisPaused;
        const bool pauseOverlay = tetrisPaused && tetrisStarted && !tetrisGameOver;
        lv_coord_t overlayW = pauseOverlay ? min<lv_coord_t>(DISPLAY_WIDTH - 64, 140) : min<lv_coord_t>(DISPLAY_WIDTH - 20, 324);
        lv_coord_t overlayH = pauseOverlay ? 76 : min<lv_coord_t>(UI_CONTENT_H - 8, 467);
        if (lvglTetrisBoardObj) {
            lv_area_t boardCoords;
            lv_obj_get_coords(lvglTetrisBoardObj, &boardCoords);
            const lv_coord_t boardWidth = static_cast<lv_coord_t>(boardCoords.x2 - boardCoords.x1 + 1);
            const lv_coord_t boardHeight = static_cast<lv_coord_t>(boardCoords.y2 - boardCoords.y1 + 1);
            overlayW = min<lv_coord_t>(overlayW, max<lv_coord_t>(120, boardWidth - 16));
            overlayH = min<lv_coord_t>(overlayH, max<lv_coord_t>(137, boardHeight - 16));
        }
        lv_obj_set_size(lvglTetrisOverlay, overlayW, overlayH);
        lv_obj_center(lvglTetrisOverlay);
        lv_obj_move_foreground(lvglTetrisOverlay);
        if (showOverlay) lv_obj_clear_flag(lvglTetrisOverlay, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(lvglTetrisOverlay, LV_OBJ_FLAG_HIDDEN);
        if (lvglTetrisOverlayTitle) {
            if (pauseOverlay) lv_label_set_text(lvglTetrisOverlayTitle, "PAUSE");
            else lv_label_set_text(lvglTetrisOverlayTitle, tetrisGameOver ? "Game Over" : "Tetris");
        }
        if (lvglTetrisOverlaySubLabel) {
            String msg = pauseOverlay ? String("Tap center button to resume")
                                      : (tetrisGameOver ? ("Score " + String(tetrisScore) + "\nBest " + String(tetrisHighScore))
                                                       : ("Tap Start when ready\nBest " + String(tetrisHighScore) + "\nLast " + String(tetrisLastScore)));
            lv_label_set_text(lvglTetrisOverlaySubLabel, msg.c_str());
        }
        if (lvglTetrisOverlayBtn) {
            if (pauseOverlay) lv_obj_add_flag(lvglTetrisOverlayBtn, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_clear_flag(lvglTetrisOverlayBtn, LV_OBJ_FLAG_HIDDEN);
        }
        if (lvglTetrisOverlayBtnLabel && !pauseOverlay) lv_label_set_text(lvglTetrisOverlayBtnLabel, tetrisGameOver ? "Replay" : "Start");
    }
    if (lvglTetrisPauseBtnLabel) lv_label_set_text(lvglTetrisPauseBtnLabel, (!tetrisStarted || tetrisPaused) ? LV_SYMBOL_PLAY : LV_SYMBOL_PAUSE);
}

void lvglOpenSnakeEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_GAME_SNAKE);
    snakePrepareGame();
    if (screensaverActive) screensaverSetActive(false);
    if (!displayAwake) displaySetAwake(true);
    lastUserActivityMs = millis();
    lvglRefreshSnakeBoard();
    lvglOpenScreen(UI_GAME_SNAKE, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

#if defined(BOARD_ESP32S3_3248S035_N16R8)
void lvglOpenSnake3dEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_GAME_SNAKE3D);
    snake3dPrepareGame();
    if (screensaverActive) screensaverSetActive(false);
    if (!displayAwake) displaySetAwake(true);
    lastUserActivityMs = millis();
    lvglRefreshSnake3dBoard();
    lvglOpenScreen(UI_GAME_SNAKE3D, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}
#endif

void lvglOpenTetrisEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_GAME_TETRIS);
    tetrisPrepareGame();
    if (screensaverActive) screensaverSetActive(false);
    if (!displayAwake) displaySetAwake(true);
    lastUserActivityMs = millis();
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

void lvglOpenOtaScreenEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_CONFIG_OTA);
    lvglRefreshOtaUi();
    lvglOpenScreen(UI_CONFIG_OTA, LV_SCR_LOAD_ANIM_MOVE_LEFT);
    otaCheckRequested = true;
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

void lvglOtaUpdateEvent(lv_event_t *e)
{
    (void)e;
    if (otaUpdateTaskHandle || otaUiState == OTA_UI_DOWNLOADING || otaUiState == OTA_UI_FINALIZING) return;
    if (!OTA_FIRMWARE_FLASH_SUPPORTED) {
        otaUiState = OTA_UI_ERROR;
        otaSetStatus("This board build does not have an OTA flash slot");
        lvglRefreshOtaUi();
        uiStatusLine = "OTA unsupported on current flash layout";
        lvglSyncStatusLine();
        return;
    }
    if (!wifiConnectedSafe()) {
        otaUiState = OTA_UI_ERROR;
        otaSetStatus(apModeActive ? "Not connected to update server (AP mode only)" : "Not connected to update server");
        lvglRefreshOtaUi();
        uiStatusLine = apModeActive ? "OTA needs internet WiFi, not AP-only mode" : "Connect WiFi before OTA";
        lvglSyncStatusLine();
        return;
    }
    if (!otaUpdateAvailable || otaLatestBinUrl.isEmpty()) {
        otaCheckRequested = true;
        uiStatusLine = "Checking for updates";
        lvglSyncStatusLine();
        return;
    }
    otaUiState = OTA_UI_DOWNLOADING;
    otaProgressPercent = 0;
    otaSetStatus("Starting update...");
    if (xTaskCreatePinnedToCore(otaUpdateTask, "ota_update", 12288, nullptr, 1, &otaUpdateTaskHandle, ARDUINO_RUNNING_CORE) != pdPASS) {
        otaUpdateTaskHandle = nullptr;
        otaUiState = OTA_UI_ERROR;
        otaSetStatus("Update start failed");
    }
    lvglRefreshOtaUi();
}

void lvglRefreshStyleUi()
{
    if (lvglStyleUiSyncing) return;
    lvglStyleUiSyncing = true;

    if (lvglStyleScreensaverSw) {
        const bool checked = lv_obj_has_state(lvglStyleScreensaverSw, LV_STATE_CHECKED);
        if (screensaverEnabled != checked) {
            if (screensaverEnabled) lv_obj_add_state(lvglStyleScreensaverSw, LV_STATE_CHECKED);
            else lv_obj_clear_state(lvglStyleScreensaverSw, LV_STATE_CHECKED);
        }
    }
    if (lvglStyleMenuIconsSw) {
        const bool checked = lv_obj_has_state(lvglStyleMenuIconsSw, LV_STATE_CHECKED);
        if (menuCustomIconsEnabled != checked) {
            if (menuCustomIconsEnabled) lv_obj_add_state(lvglStyleMenuIconsSw, LV_STATE_CHECKED);
            else lv_obj_clear_state(lvglStyleMenuIconsSw, LV_STATE_CHECKED);
        }
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
        lvglLabelSetTextIfChanged(lvglStyleButtonFlatBtnLabel, uiButtonStyleMode == UI_BUTTON_STYLE_FLAT ? "Selected" : "Select");
        lvglApplyPersistentToggleButtonStyle(lvglStyleButtonFlatBtn,
                                             lvglStyleButtonFlatBtnLabel,
                                             uiButtonStyleMode == UI_BUTTON_STYLE_FLAT,
                                             lv_color_hex(uiButtonStyleFlatSelectorColor),
                                             lv_color_hex(uiButtonStyleFlatSelectorColor),
                                             true);
    }
    if (lvglStyleButton3dBtn && lvglStyleButton3dBtnLabel) {
        lvglLabelSetTextIfChanged(lvglStyleButton3dBtnLabel, uiButtonStyleMode == UI_BUTTON_STYLE_3D ? "Selected" : "Select");
        lvglApplyPersistentToggleButtonStyle(lvglStyleButton3dBtn,
                                             lvglStyleButton3dBtnLabel,
                                             uiButtonStyleMode == UI_BUTTON_STYLE_3D,
                                             lv_color_hex(uiButtonStyle3dSelectorColor),
                                             lv_color_hex(uiButtonStyle3dSelectorColor),
                                             true);
    }
    if (lvglStyleButtonBlackBtn && lvglStyleButtonBlackBtnLabel) {
        lvglLabelSetTextIfChanged(lvglStyleButtonBlackBtnLabel, uiButtonStyleMode == UI_BUTTON_STYLE_BLACK ? "Selected" : "Select");
        lvglApplyPersistentToggleButtonStyle(lvglStyleButtonBlackBtn,
                                             lvglStyleButtonBlackBtnLabel,
                                             uiButtonStyleMode == UI_BUTTON_STYLE_BLACK,
                                             lv_color_hex(UI_BUTTON_BLACK_BODY_HEX),
                                             lv_color_hex(UI_BUTTON_BLACK_BODY_HEX),
                                             true);
    }
    if (lvglStyleTopCenterNameBtn && lvglStyleTopCenterNameBtnLabel) {
        lvglLabelSetTextIfChanged(lvglStyleTopCenterNameBtnLabel, topBarCenterMode == TOP_BAR_CENTER_NAME ? "Selected" : "Select");
        lvglApplyPersistentToggleButtonStyle(lvglStyleTopCenterNameBtn,
                                             lvglStyleTopCenterNameBtnLabel,
                                             topBarCenterMode == TOP_BAR_CENTER_NAME,
                                             lv_color_hex(0x3F4A57),
                                             lv_color_hex(0x3F4A57),
                                             true);
    }
    if (lvglStyleTopCenterTimeBtn && lvglStyleTopCenterTimeBtnLabel) {
        lvglLabelSetTextIfChanged(lvglStyleTopCenterTimeBtnLabel, topBarCenterMode == TOP_BAR_CENTER_TIME ? "Selected" : "Select");
        lvglApplyPersistentToggleButtonStyle(lvglStyleTopCenterTimeBtn,
                                             lvglStyleTopCenterTimeBtnLabel,
                                             topBarCenterMode == TOP_BAR_CENTER_TIME,
                                             lv_color_hex(0x3F4A57),
                                             lv_color_hex(0x3F4A57),
                                             true);
    }
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
    if (lvglConfigRgbHeader) lvglLabelSetTextIfChanged(lvglConfigRgbHeader, tr(TXT_RGB_LED));
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
    if (lvglRgbLedSlider && lv_slider_get_value(lvglRgbLedSlider) != rgbLedPercent) {
        lv_slider_set_value(lvglRgbLedSlider, rgbLedPercent, LV_ANIM_OFF);
    }
    lvglRefreshSoundPopupUi();
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
    lvglRefreshLanguageUi();
    lvglTopIndicatorStateValid = false;
    if (lvglReady) lvglRefreshTopIndicators();
}

void lvglRefreshOtaUi()
{
    if (lvglOtaCurrentLabel) lvglLabelSetTextIfChanged(lvglOtaCurrentLabel, FW_VERSION);
    if (lvglOtaLatestLabel) {
        if (!OTA_FIRMWARE_FLASH_SUPPORTED) lvglLabelSetTextIfChanged(lvglOtaLatestLabel, "Unavailable on this board");
        else if (otaLatestVersion[0] != '\0') lvglLabelSetTextIfChanged(lvglOtaLatestLabel, otaLatestVersion);
        else lvglLabelSetTextIfChanged(lvglOtaLatestLabel, otaUiState == OTA_UI_CHECKING ? "Checking..." : "Unknown");
    }
    if (lvglOtaStatusLabel) {
        if (!OTA_FIRMWARE_FLASH_SUPPORTED) lvglLabelSetTextIfChanged(lvglOtaStatusLabel, "Firmware image is too large for dual-slot OTA on the 4 MB layout.");
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
        const bool busy = otaUiState == OTA_UI_CHECKING || otaUiState == OTA_UI_DOWNLOADING || otaUiState == OTA_UI_FINALIZING;
        if (busy || !OTA_FIRMWARE_FLASH_SUPPORTED) {
            lv_obj_add_flag(lvglOtaUpdateBtn, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_clear_flag(lvglOtaUpdateBtn, LV_OBJ_FLAG_HIDDEN);
            lvglLabelSetTextIfChanged(lvglOtaUpdateBtnLabel,
                                      lvglSymbolText(otaUpdateAvailable ? LV_SYMBOL_UPLOAD : LV_SYMBOL_REFRESH,
                                                     otaUpdateAvailable ? "Update" : "Check Now"));
            const lv_color_t btnCol = otaUpdateAvailable ? lv_color_hex(0x3A8F4B) : lv_color_hex(0x2E6F95);
            lvglRegisterStyledButton(lvglOtaUpdateBtn, btnCol, false);
        }
    }
}

struct ScreensaverEyePreset {
    int16_t offsetX;
    int16_t offsetY;
    int16_t height;
    int16_t width;
    float slopeTop;
    float slopeBottom;
    int16_t radiusTop;
    int16_t radiusBottom;
    int16_t inverseRadiusTop;
    int16_t inverseRadiusBottom;
    int16_t inverseOffsetTop;
    int16_t inverseOffsetBottom;
    bool pupilsVisible;
};

struct ScreensaverPose {
    const ScreensaverEyePreset *preset;
    float faceScale;
    float lookX;
    float lookY;
};

static constexpr ScreensaverEyePreset SCREENSAVER_PRESET_NORMAL = {0, 0, 40, 40, 0.0f, 0.0f, 8, 8, 0, 0, 0, 0, true};
static constexpr ScreensaverEyePreset SCREENSAVER_PRESET_HAPPY = {0, 0, 10, 40, 0.0f, 0.0f, 10, 0, 0, 0, 0, 0, true};
static constexpr ScreensaverEyePreset SCREENSAVER_PRESET_GLEE = {0, 0, 8, 40, 0.0f, 0.0f, 8, 0, 0, 5, 0, 0, true};
static constexpr ScreensaverEyePreset SCREENSAVER_PRESET_SAD = {0, 0, 15, 40, -0.5f, 0.0f, 1, 10, 0, 0, 0, 0, true};
static constexpr ScreensaverEyePreset SCREENSAVER_PRESET_FOCUSED = {0, 0, 14, 40, 0.2f, 0.0f, 3, 1, 0, 0, 0, 0, true};
static constexpr ScreensaverEyePreset SCREENSAVER_PRESET_SURPRISED = {-2, 0, 45, 45, 0.0f, 0.0f, 16, 16, 0, 0, 0, 0, true};
static constexpr ScreensaverEyePreset SCREENSAVER_PRESET_SKEPTIC_ALT = {0, -6, 26, 40, 0.3f, 0.0f, 1, 10, 0, 0, 0, 0, true};
static constexpr ScreensaverEyePreset SCREENSAVER_PRESET_SLEEPY = {0, -2, 14, 40, -0.5f, -0.5f, 3, 3, 0, 0, 0, 0, false};
static constexpr ScreensaverEyePreset SCREENSAVER_PRESET_SUSPICIOUS = {0, 0, 22, 40, 0.0f, 0.0f, 8, 3, 0, 0, 0, 0, true};
static constexpr ScreensaverEyePreset SCREENSAVER_PRESET_BLINK = {0, 0, 5, 40, 0.0f, 0.0f, 3, 3, 0, 0, 0, 0, false};

static const ScreensaverEyePreset *const SCREENSAVER_PRESETS[] = {
    &SCREENSAVER_PRESET_NORMAL,
    &SCREENSAVER_PRESET_HAPPY,
    &SCREENSAVER_PRESET_GLEE,
    &SCREENSAVER_PRESET_SAD,
    &SCREENSAVER_PRESET_FOCUSED,
    &SCREENSAVER_PRESET_SURPRISED,
    &SCREENSAVER_PRESET_SKEPTIC_ALT,
    &SCREENSAVER_PRESET_SLEEPY,
    &SCREENSAVER_PRESET_SUSPICIOUS,
};

static ScreensaverPose screensaverRandomPose(bool blinkOnly)
{
    const float scale = min(DISPLAY_WIDTH / 128.0f, DISPLAY_HEIGHT / 64.0f) * 0.9f;
    const ScreensaverEyePreset *preset = blinkOnly
                                             ? &SCREENSAVER_PRESET_BLINK
                                             : SCREENSAVER_PRESETS[random(0, static_cast<int>(sizeof(SCREENSAVER_PRESETS) / sizeof(SCREENSAVER_PRESETS[0])))];
    ScreensaverPose pose = {
        preset,
        scale,
        blinkOnly ? 0.0f : static_cast<float>(random(-50, 51)) / 100.0f,
        blinkOnly ? 0.0f : static_cast<float>(random(-40, 41)) / 100.0f,
    };
    return pose;
}

enum ScreensaverCornerType : uint8_t { SC_T_R, SC_T_L, SC_B_L, SC_B_R };

static void screensaverFillRect(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint16_t color)
{
    const int32_t l = min(x0, x1);
    const int32_t r = max(x0, x1);
    const int32_t t = min(y0, y1);
    const int32_t b = max(y0, y1);
    const int32_t w = max<int32_t>(0, r - l);
    const int32_t h = max<int32_t>(0, b - t);
    if (w <= 0 || h <= 0) return;
    tft.fillRect(l, t, w, h, color);
}

static inline uint16_t screensaverDrawColor(uint8_t drawValue, uint16_t eyeColor)
{
    return drawValue ? eyeColor : TFT_BLACK;
}

static void screensaverFillRectTriangle(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint8_t drawValue, uint16_t eyeColor)
{
    tft.fillTriangle(x0, y0, x1, y1, x1, y0, screensaverDrawColor(drawValue, eyeColor));
}

static void screensaverFillEllipseCorner(ScreensaverCornerType corner, int16_t x0, int16_t y0, int32_t rx, int32_t ry, uint16_t color)
{
    if (rx < 2 || ry < 2) return;
    int32_t x, y;
    const int32_t rx2 = rx * rx;
    const int32_t ry2 = ry * ry;
    const int32_t fx2 = 4 * rx2;
    const int32_t fy2 = 4 * ry2;
    int32_t s;

    auto hline = [&](int32_t sx, int32_t sy, int32_t len) {
        if (len <= 0) return;
        tft.drawFastHLine(static_cast<int16_t>(sx), static_cast<int16_t>(sy), static_cast<int16_t>(len), color);
    };

    if (corner == SC_T_R) {
        for (x = 0, y = ry, s = 2 * ry2 + rx2 * (1 - 2 * ry); ry2 * x <= rx2 * y; x++) {
            hline(x0, y0 - y, x);
            if (s >= 0) { s += fx2 * (1 - y); y--; }
            s += ry2 * ((4 * x) + 6);
        }
        for (x = rx, y = 0, s = 2 * rx2 + ry2 * (1 - 2 * rx); rx2 * y <= ry2 * x; y++) {
            hline(x0, y0 - y, x);
            if (s >= 0) { s += fy2 * (1 - x); x--; }
            s += rx2 * ((4 * y) + 6);
        }
    } else if (corner == SC_B_R) {
        for (x = 0, y = ry, s = 2 * ry2 + rx2 * (1 - 2 * ry); ry2 * x <= rx2 * y; x++) {
            hline(x0, y0 + y - 1, x);
            if (s >= 0) { s += fx2 * (1 - y); y--; }
            s += ry2 * ((4 * x) + 6);
        }
        for (x = rx, y = 0, s = 2 * rx2 + ry2 * (1 - 2 * rx); rx2 * y <= ry2 * x; y++) {
            hline(x0, y0 + y - 1, x);
            if (s >= 0) { s += fy2 * (1 - x); x--; }
            s += rx2 * ((4 * y) + 6);
        }
    } else if (corner == SC_T_L) {
        for (x = 0, y = ry, s = 2 * ry2 + rx2 * (1 - 2 * ry); ry2 * x <= rx2 * y; x++) {
            hline(x0 - x, y0 - y, x);
            if (s >= 0) { s += fx2 * (1 - y); y--; }
            s += ry2 * ((4 * x) + 6);
        }
        for (x = rx, y = 0, s = 2 * rx2 + ry2 * (1 - 2 * rx); rx2 * y <= ry2 * x; y++) {
            hline(x0 - x, y0 - y, x);
            if (s >= 0) { s += fy2 * (1 - x); x--; }
            s += rx2 * ((4 * y) + 6);
        }
    } else {
        for (x = 0, y = ry, s = 2 * ry2 + rx2 * (1 - 2 * ry); ry2 * x <= rx2 * y; x++) {
            hline(x0 - x, y0 + y - 1, x);
            if (s >= 0) { s += fx2 * (1 - y); y--; }
            s += ry2 * ((4 * x) + 6);
        }
        for (x = rx, y = 0, s = 2 * rx2 + ry2 * (1 - 2 * rx); rx2 * y <= ry2 * x; y++) {
            hline(x0 - x, y0 + y, x);
            if (s >= 0) { s += fy2 * (1 - x); x--; }
            s += rx2 * ((4 * y) + 6);
        }
    }
}

static void screensaverDrawEyeExact(int16_t centerX, int16_t centerY, const ScreensaverEyePreset &srcPreset,
                                    bool mirrored, float scaleX, float scaleY, uint16_t color)
{
    ScreensaverEyePreset config = srcPreset;
    config.offsetX = mirrored ? -config.offsetX : config.offsetX;
    config.offsetY = -config.offsetY;
    config.slopeTop = mirrored ? config.slopeTop : -config.slopeTop;
    config.slopeBottom = mirrored ? config.slopeBottom : -config.slopeBottom;

    const int32_t offsetX = lroundf(config.offsetX * scaleX);
    const int32_t offsetY = lroundf(config.offsetY * scaleY);
    const int32_t width = max<int32_t>(2, lroundf(config.width * scaleX));
    const int32_t height = max<int32_t>(2, lroundf(config.height * scaleY));
    int32_t radiusTop = max<int32_t>(0, lroundf(config.radiusTop * min(scaleX, scaleY)));
    int32_t radiusBottom = max<int32_t>(0, lroundf(config.radiusBottom * min(scaleX, scaleY)));

    int32_t deltaYTop = lroundf(height * config.slopeTop / 2.0f);
    int32_t deltaYBottom = lroundf(height * config.slopeBottom / 2.0f);
    const int32_t totalHeight = height + deltaYTop - deltaYBottom;
    if (radiusBottom > 0 && radiusTop > 0 && totalHeight - 1 < radiusBottom + radiusTop) {
        const int32_t correctedTop = static_cast<int32_t>(static_cast<float>(radiusTop) * (totalHeight - 1) / max<int32_t>(1, radiusBottom + radiusTop));
        const int32_t correctedBottom = static_cast<int32_t>(static_cast<float>(radiusBottom) * (totalHeight - 1) / max<int32_t>(1, radiusBottom + radiusTop));
        radiusTop = correctedTop;
        radiusBottom = correctedBottom;
    }

    const int32_t TLcY = centerY + offsetY - height / 2 + radiusTop - deltaYTop;
    const int32_t TLcX = centerX + offsetX - width / 2 + radiusTop;
    const int32_t TRcY = centerY + offsetY - height / 2 + radiusTop + deltaYTop;
    const int32_t TRcX = centerX + offsetX + width / 2 - radiusTop;
    const int32_t BLcY = centerY + offsetY + height / 2 - radiusBottom - deltaYBottom;
    const int32_t BLcX = centerX + offsetX - width / 2 + radiusBottom;
    const int32_t BRcY = centerY + offsetY + height / 2 - radiusBottom + deltaYBottom;
    const int32_t BRcX = centerX + offsetX + width / 2 - radiusBottom;

    const int32_t minCX = min(TLcX, BLcX);
    const int32_t maxCX = max(TRcX, BRcX);
    const int32_t minCY = min(TLcY, TRcY);
    const int32_t maxCY = max(BLcY, BRcY);

    screensaverFillRect(minCX, minCY, maxCX, maxCY, color);
    screensaverFillRect(TRcX, TRcY, BRcX + radiusBottom, BRcY, color);
    screensaverFillRect(TLcX - radiusTop, TLcY, BLcX, BLcY, color);
    screensaverFillRect(TLcX, TLcY - radiusTop, TRcX, TRcY, color);
    screensaverFillRect(BLcX, BLcY, BRcX, BRcY + radiusBottom, color);

    if (config.slopeTop > 0) {
        screensaverFillRectTriangle(TLcX, TLcY - radiusTop, TRcX, TRcY - radiusTop, 0, color);
        screensaverFillRectTriangle(TRcX, TRcY - radiusTop, TLcX, TLcY - radiusTop, 1, color);
    } else if (config.slopeTop < 0) {
        screensaverFillRectTriangle(TRcX, TRcY - radiusTop, TLcX, TLcY - radiusTop, 0, color);
        screensaverFillRectTriangle(TLcX, TLcY - radiusTop, TRcX, TRcY - radiusTop, 1, color);
    }
    if (config.slopeBottom > 0) {
        screensaverFillRectTriangle(BRcX + radiusBottom, BRcY + radiusBottom, BLcX - radiusBottom, BLcY + radiusBottom, 0, color);
        screensaverFillRectTriangle(BLcX - radiusBottom, BLcY + radiusBottom, BRcX + radiusBottom, BRcY + radiusBottom, 1, color);
    } else if (config.slopeBottom < 0) {
        screensaverFillRectTriangle(BLcX - radiusBottom, BLcY + radiusBottom, BRcX + radiusBottom, BRcY + radiusBottom, 0, color);
        screensaverFillRectTriangle(BRcX + radiusBottom, BRcY + radiusBottom, BLcX - radiusBottom, BLcY + radiusBottom, 1, color);
    }

    if (radiusTop > 0) {
        screensaverFillEllipseCorner(SC_T_L, TLcX, TLcY, radiusTop, radiusTop, color);
        screensaverFillEllipseCorner(SC_T_R, TRcX, TRcY, radiusTop, radiusTop, color);
    }
    if (radiusBottom > 0) {
        screensaverFillEllipseCorner(SC_B_L, BLcX, BLcY, radiusBottom, radiusBottom, color);
        screensaverFillEllipseCorner(SC_B_R, BRcX, BRcY, radiusBottom, radiusBottom, color);
    }
}

static void screensaverApplyPose(const ScreensaverPose &pose)
{
    const ScreensaverEyePreset &preset = *pose.preset;
    const float eyeScale = pose.faceScale;
    const int16_t eyeSize = max<int16_t>(18, lroundf(40.0f * eyeScale));
    const int16_t interDistance = max<int16_t>(4, lroundf(4.0f * eyeScale));
    const int16_t centerX = DISPLAY_WIDTH / 2 + lroundf(-25.0f * pose.lookX * eyeScale);
    const int16_t centerY = DISPLAY_HEIGHT / 2 + lroundf(20.0f * pose.lookY * eyeScale);
    const float scaleYY = 1.0f - fabsf(pose.lookY) * 0.4f;
    const float leftScaleY = (1.0f + pose.lookX * 0.2f) * scaleYY;
    const float rightScaleY = (1.0f - pose.lookX * 0.2f) * scaleYY;
    const uint16_t eyeColor = tft.color565(
        static_cast<uint8_t>((SCREENSAVER_EYE_COLOR_RGB >> 16) & 0xFF),
        static_cast<uint8_t>((SCREENSAVER_EYE_COLOR_RGB >> 8) & 0xFF),
        static_cast<uint8_t>(SCREENSAVER_EYE_COLOR_RGB & 0xFF));

    tft.fillScreen(TFT_BLACK);
    screensaverDrawEyeExact(centerX - eyeSize / 2 - interDistance, centerY, preset, true, eyeScale, eyeScale * leftScaleY, eyeColor);
    screensaverDrawEyeExact(centerX + eyeSize / 2 + interDistance, centerY, preset, false, eyeScale, eyeScale * rightScaleY, eyeColor);
}

void screensaverSetActive(bool active)
{
    if (screensaverActive == active) return;
    screensaverActive = active;
    if (active) {
        screensaverReturnScreen = uiScreen;
        lvglEnsureScreenBuilt(UI_SCREENSAVER);
        if (lvglTopBarRoot) lv_obj_add_flag(lvglTopBarRoot, LV_OBJ_FLAG_HIDDEN);
        screensaverLastPoseMs = 0;
        screensaverNextPoseDelayMs = SCREENSAVER_POSE_MIN_MS;
        screensaverApplyPose(screensaverRandomPose(false));
        lvglOpenScreen(UI_SCREENSAVER, LV_SCR_LOAD_ANIM_FADE_ON);
    } else {
        if (lvglTopBarRoot) lv_obj_clear_flag(lvglTopBarRoot, LV_OBJ_FLAG_HIDDEN);
        lvglOpenScreen(screensaverReturnScreen, LV_SCR_LOAD_ANIM_FADE_ON);
        lvglSuppressClicksAfterGesture();
    }
}

void screensaverService()
{
    if (!screensaverActive || !lvglReady) return;
    const unsigned long now = millis();
    if (screensaverLastPoseMs != 0 && static_cast<unsigned long>(now - screensaverLastPoseMs) < screensaverNextPoseDelayMs) return;
    const bool blinkNow = random(0, 100) < 24;
    screensaverApplyPose(screensaverRandomPose(blinkNow));
    screensaverLastPoseMs = now;
    screensaverNextPoseDelayMs = blinkNow ? SCREENSAVER_BLINK_MS : static_cast<unsigned long>(random(static_cast<long>(SCREENSAVER_POSE_MIN_MS), static_cast<long>(SCREENSAVER_POSE_MAX_MS + 1UL)));
}

void lvglSetConfigKeyboardVisible(bool visible)
{
    const lv_coord_t keyboardPad = visible ? 132 : 10;
    if (lvglConfigWrap) lv_obj_set_style_pad_bottom(lvglConfigWrap, keyboardPad, 0);
    lv_obj_t *hc12TerminalTa = hc12TerminalObj();
    if (hc12TerminalTa) {
        lv_obj_t *hc12Wrap = lv_obj_get_parent(hc12TerminalTa);
        if (hc12Wrap) lv_obj_set_style_pad_bottom(hc12Wrap, keyboardPad, 0);
    }

    if (hc12TerminalTa) {
        lv_obj_set_height(hc12TerminalTa, visible ? (UI_CONTENT_H - 176) : (UI_CONTENT_H - 108));
    }

    if (visible) {
        lv_obj_t *hc12CmdTa = hc12CmdTaObj();
        if (uiScreen == UI_CONFIG_HC12_TERMINAL && hc12CmdTa) lv_obj_scroll_to_view_recursive(hc12CmdTa, LV_ANIM_ON);
        else if (lvglConfigDeviceNameTa) lv_obj_scroll_to_view_recursive(lvglConfigDeviceNameTa, LV_ANIM_ON);
    } else if (uiScreen == UI_CONFIG_HC12_TERMINAL && hc12TerminalTa) {
        lv_obj_t *hc12Wrap = lv_obj_get_parent(hc12TerminalTa);
        if (hc12Wrap) lv_obj_scroll_to_y(hc12Wrap, 0, LV_ANIM_ON);
    }
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

    if (VERBOSE_SERIAL_DEBUG) Serial.println("[LVGL] step draw_buf_init");
    lv_disp_draw_buf_init(
        &lvglDrawBuf,
        lvglDrawPixels,
        nullptr,
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
        static const char *btns[] = {"OK", ""};
        String prompt = String("Device updated to FW Version ") + FW_VERSION;
        lv_obj_t *m = lv_msgbox_create(nullptr, "Update Complete", prompt.c_str(), btns, false);
        if (m) {
            lvglApplyMsgboxModalStyle(m);
            lv_obj_center(m);
            lv_obj_set_width(m, min<int16_t>(DISPLAY_WIDTH - 24, 280));
            lv_obj_add_event_cb(m, lvglOtaPopupEvent, LV_EVENT_VALUE_CHANGED, nullptr);
            lvglOtaPostUpdatePopupVisible = true;
        }
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
        char msg[48];
        snprintf(msg, sizeof(msg), "Searching for access points%.*s", wifiScanAnimPhase, "...");
        lv_label_set_text(lvglWifiScanLabel, msg);
    }
    unsigned long delta = now - lvglLastTickMs;
    if (delta > 0) {
        lv_tick_inc(delta);
        lvglLastTickMs = now;
    }
    const bool uiInteractive = lvglTouchDown || wakeTouchReleaseGuard || lvglSwipeTracking || lv_anim_count_running() > 0U;
    if (!uiInteractive && lvglNextHandlerDueMs != 0 && static_cast<long>(now - lvglNextHandlerDueMs) < 0) return;
    const uint32_t nextHandlerInMs = lv_timer_handler();
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
            lvglSwipeTracking = true;
            lvglSwipeStartX = lvglLastTouchX;
            lvglSwipeStartY = lvglLastTouchY;
            lvglSwipeLastX = lvglLastTouchX;
            lvglSwipeLastY = lvglLastTouchY;
            lvglSwipeStartMs = now;
            lvglSwipeCandidate = uiScreenSupportsSwipeBack(uiScreen) &&
                                 !lvglGestureBlocked &&
                                 !lvglTouchOwnsHorizontalGesture();
            lvglSwipeHorizontalLocked = false;
        } else if (lvglSwipeTracking) {
            lvglSwipeLastX = lvglLastTouchX;
            lvglSwipeLastY = lvglLastTouchY;
            if (lvglGestureBlocked) {
                lvglSwipeCandidate = false;
                lvglSwipeHorizontalLocked = false;
            }
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
                if (lvglSwipeCandidate && lvglSwipeHorizontalLocked) {
                    lvglApplySwipeBackVisual(lvglClampSwipeBackOffset(dx));
                }
            }
            if ((!lvglSwipeCandidate || !lvglSwipeHorizontalLocked) && lvglSwipeVisualActive && !lvglSwipeVisualAnimating) {
                lvglResetSwipeBackVisualState(true);
            }
        }
    } else if (lvglSwipeTracking) {
        lvglSwipeTracking = false;
        const int dx = static_cast<int>(lvglSwipeLastX) - static_cast<int>(lvglSwipeStartX);
        const int dy = static_cast<int>(lvglSwipeLastY) - static_cast<int>(lvglSwipeStartY);
        const unsigned long dt = static_cast<unsigned long>(now - lvglSwipeStartMs);
        const bool hadSwipeVisual = lvglSwipeVisualActive;
        const int completionDx = max<int>(1, (DISPLAY_WIDTH * SWIPE_BACK_COMPLETE_PERCENT) / 100);
        const bool passedCompletionThreshold = hadSwipeVisual && dx >= completionDx;
        const bool tapCandidate = displayAwake &&
                                  !lvglKeyboardVisible() &&
                                  abs(dx) <= DOUBLE_TAP_MAX_MOVE &&
                                  abs(dy) <= DOUBLE_TAP_MAX_MOVE &&
                                  dt <= DOUBLE_TAP_MAX_TAP_MS;
        const bool swipeComplete =
            lvglSwipeCandidate &&
            lvglSwipeHorizontalLocked &&
            abs(dy) <= SWIPE_BACK_MAX_DY &&
            dx > (abs(dy) * 2) &&
            ((dx >= SWIPE_BACK_MIN_DX && dt <= SWIPE_BACK_MAX_MS) || passedCompletionThreshold);
        if (swipeComplete) {
            lvglSuppressClicksAfterGesture();
            lvglLastTapReleaseMs = 0;
            if (lvglKb && !lv_obj_has_flag(lvglKb, LV_OBJ_FLAG_HIDDEN)) {
                if (hadSwipeVisual) lvglAnimateSwipeBackVisual(0, false);
                lvglHideKeyboard();
            } else if (hadSwipeVisual) {
                lvglAnimateSwipeBackVisual(DISPLAY_WIDTH, true);
            } else {
                lvglNavigateBackBySwipe();
            }
        } else if (tapCandidate) {
            if (hadSwipeVisual) lvglAnimateSwipeBackVisual(0, false);
            lvglLastTapReleaseMs = now;
            lvglLastTapReleaseX = lvglLastTouchX;
            lvglLastTapReleaseY = lvglLastTouchY;
        } else {
            if (hadSwipeVisual) lvglAnimateSwipeBackVisual(0, false);
            if (abs(dx) >= SWIPE_LOCK_MIN_DX || abs(dy) >= SWIPE_CANCEL_VERTICAL_DY) lvglSuppressClicksAfterGesture();
            lvglLastTapReleaseMs = 0;
        }
        lvglSwipeCandidate = false;
        lvglSwipeHorizontalLocked = false;
    }

    if ((now - lvglLastStatusRefreshMs) >= 900UL) {
        lvglLastStatusRefreshMs = now;
        lvglSyncStatusLine();
        lvglRefreshTopIndicators();
    }
    if (uiScreen == UI_MEDIA && (now - lvglLastMediaPlayerRefreshMs) >= 400UL) {
        lvglLastMediaPlayerRefreshMs = now;
        lvglRefreshMediaPlayerUi();
    }
    if (uiScreen == UI_INFO && !lvglTouchDown && (now - lvglLastInfoRefreshMs) >= 2500UL) {
        lvglLastInfoRefreshMs = now;
        lvglRefreshInfoPanel();
    }
}

static void batteryCalibrationLoad()
{
    batteryPrefs.begin("battery", true);
    batteryCalState.observedFullRawV = batteryPrefs.getFloat("cal_full", 0.0f);
    batteryCalState.observedEmptyRawV = batteryPrefs.getFloat("cal_empty", 0.0f);
    batteryPrefs.end();
    batteryCalState.hasFullAnchor = batteryCalState.observedFullRawV > 0.0f;
    batteryCalState.hasEmptyAnchor = batteryCalState.observedEmptyRawV > 0.0f;
}

static void batteryCalibrationSave()
{
    batteryPrefs.begin("battery", false);
    if (batteryCalState.hasFullAnchor) batteryPrefs.putFloat("cal_full", batteryCalState.observedFullRawV);
    else batteryPrefs.remove("cal_full");
    if (batteryCalState.hasEmptyAnchor) batteryPrefs.putFloat("cal_empty", batteryCalState.observedEmptyRawV);
    else batteryPrefs.remove("cal_empty");
    batteryPrefs.end();
}

static void batteryCalibrationReset()
{
    batteryCalState.hasFullAnchor = false;
    batteryCalState.hasEmptyAnchor = false;
    batteryCalState.observedFullRawV = 0.0f;
    batteryCalState.observedEmptyRawV = 0.0f;
    batteryCalibrationSave();
}

static void batteryTrainingLoad()
{
    batteryPrefs.begin("battery", true);
    batteryTrainingState.active = batteryPrefs.getBool("train_on", false);
    batteryTrainingState.powerOverrideActive = batteryPrefs.getBool("train_ovr", false);
    batteryTrainingState.autoCalibrationEnabled = batteryPrefs.getBool("auto_en", false);
    batteryTrainingState.phase = static_cast<BatteryTrainingPhase>(batteryPrefs.getUChar("train_ph", static_cast<uint8_t>(BATTERY_TRAIN_IDLE)));
    if (batteryTrainingState.phase > BATTERY_TRAIN_DISCHARGE) batteryTrainingState.phase = BATTERY_TRAIN_IDLE;
    batteryTrainingState.savedPowerOffMs = batteryPrefs.getULong("train_pwr", 0UL);
    batteryPrefs.end();
    if (!batteryTrainingState.active) batteryTrainingState.phase = BATTERY_TRAIN_IDLE;
    if (batteryTrainingState.powerOverrideActive) powerOffIdleTimeoutMs = 0UL;
}

static void batteryTrainingSave()
{
    batteryPrefs.begin("battery", false);
    batteryPrefs.putBool("train_on", batteryTrainingState.active);
    batteryPrefs.putBool("train_ovr", batteryTrainingState.powerOverrideActive);
    batteryPrefs.putBool("auto_en", batteryTrainingState.autoCalibrationEnabled);
    batteryPrefs.putUChar("train_ph", static_cast<uint8_t>(batteryTrainingState.phase));
    batteryPrefs.putULong("train_pwr", batteryTrainingState.savedPowerOffMs);
    batteryPrefs.end();
}

static const char *batteryTrainingPhaseLabel()
{
    if (!batteryTrainingState.active) {
        return batteryTrainingState.autoCalibrationEnabled ? "Auto Calibration" : "Manual Calibration";
    }
    switch (batteryTrainingState.phase) {
        case BATTERY_TRAIN_CHARGE: return "Charge Training";
        case BATTERY_TRAIN_DISCHARGE: return "Discharge Training";
        default: return "Manual Calibration";
    }
}

static void batteryTrainingRestorePowerOff()
{
    if (!batteryTrainingState.powerOverrideActive) return;
    powerOffIdleTimeoutMs = POWER_OFF_IDLE_TIMEOUT_OPTIONS_MS[powerOffTimeoutOptionIndex(batteryTrainingState.savedPowerOffMs)];
    batteryTrainingState.powerOverrideActive = false;
    uiPrefs.begin("ui", false);
    uiPrefs.putULong("pwr_idle", powerOffIdleTimeoutMs);
    uiPrefs.end();
    applyDisplayIdleTimeoutPowerOffCap(true);
    batteryTrainingSave();
    if (lvglReady) lvglRefreshStyleUi();
}

static void batteryTrainingStartCharge()
{
    if (!batteryTrainingState.powerOverrideActive) batteryTrainingState.savedPowerOffMs = powerOffIdleTimeoutMs;
    powerOffIdleTimeoutMs = 0UL;
    batteryTrainingState.active = true;
    batteryTrainingState.autoCalibrationEnabled = false;
    batteryTrainingState.phase = BATTERY_TRAIN_CHARGE;
    batteryTrainingState.powerOverrideActive = true;
    uiPrefs.begin("ui", false);
    uiPrefs.putULong("pwr_idle", powerOffIdleTimeoutMs);
    uiPrefs.end();
    batteryTrainingSave();
    if (lvglReady) {
        lvglRefreshStyleUi();
        lvglRefreshBatteryTrainUi();
    }
}

static void batteryTrainingStartDischarge()
{
    if (!batteryTrainingState.powerOverrideActive) batteryTrainingState.savedPowerOffMs = powerOffIdleTimeoutMs;
    powerOffIdleTimeoutMs = 0UL;
    batteryTrainingState.active = true;
    batteryTrainingState.autoCalibrationEnabled = false;
    batteryTrainingState.phase = BATTERY_TRAIN_DISCHARGE;
    batteryTrainingState.powerOverrideActive = true;
    uiPrefs.begin("ui", false);
    uiPrefs.putULong("pwr_idle", powerOffIdleTimeoutMs);
    uiPrefs.end();
    batteryTrainingSave();
    if (lvglReady) {
        lvglRefreshStyleUi();
        lvglRefreshBatteryTrainUi();
    }
}

static void batteryTrainingStopManual()
{
    batteryTrainingRestorePowerOff();
    batteryTrainingState.active = false;
    batteryTrainingState.phase = BATTERY_TRAIN_IDLE;
    batteryTrainingSave();
    if (lvglReady) lvglRefreshBatteryTrainUi();
}

static float batteryBlendObserved(float current, float observed)
{
    if (current <= 0.0f) return observed;
    return current + BATTERY_CAL_BLEND_ALPHA * (observed - current);
}

static void batteryCalibrationLearnFull(float rawV)
{
    if (rawV <= 0.0f) return;
    const float next = batteryBlendObserved(batteryCalState.observedFullRawV, rawV);
    if (batteryCalState.hasFullAnchor && fabsf(next - batteryCalState.observedFullRawV) < 0.003f) return;
    batteryCalState.observedFullRawV = next;
    batteryCalState.hasFullAnchor = true;
    batteryCalibrationSave();
}

static void batteryCalibrationSetFull(float rawV)
{
    if (rawV <= 0.0f) return;
    batteryCalState.observedFullRawV = rawV;
    batteryCalState.hasFullAnchor = true;
    batteryCalibrationSave();
}

static void batteryCalibrationForceFull(float rawV)
{
    if (rawV <= 0.0f) return;
    batteryCalState.observedFullRawV = rawV;
    batteryCalState.hasFullAnchor = true;

    // Rebuild the empty anchor onto the same scale so manual FULL maps the
    // current reading to 4.20V immediately instead of being diluted by older data.
    const float scale = BATTERY_FULL_V / rawV;
    if (scale > 0.0f && batteryCalState.hasEmptyAnchor) {
        batteryCalState.observedEmptyRawV = BATTERY_EMPTY_V / scale;
    }
    batteryCalibrationSave();
}

static void batteryCalibrationLearnEmpty(float rawV)
{
    if (rawV <= 0.0f) return;
    const float next = batteryBlendObserved(batteryCalState.observedEmptyRawV, rawV);
    if (batteryCalState.hasEmptyAnchor && fabsf(next - batteryCalState.observedEmptyRawV) < 0.003f) return;
    batteryCalState.observedEmptyRawV = next;
    batteryCalState.hasEmptyAnchor = true;
    batteryCalibrationSave();
}

static float batteryCalibratedVoltageFromRaw(float rawV)
{
    if (rawV <= 0.0f) return 0.0f;

    if (batteryCalState.hasFullAnchor && batteryCalState.hasEmptyAnchor) {
        const float observedSpan = batteryCalState.observedFullRawV - batteryCalState.observedEmptyRawV;
        if (observedSpan >= BATTERY_CAL_MIN_SPAN_V) {
            const float ratio = (rawV - batteryCalState.observedEmptyRawV) / observedSpan;
            return BATTERY_EMPTY_V + ratio * (BATTERY_FULL_V - BATTERY_EMPTY_V);
        }
    }

    if (batteryCalState.hasFullAnchor && batteryCalState.observedFullRawV > 0.0f) {
        return rawV * (BATTERY_FULL_V / batteryCalState.observedFullRawV);
    }

    if (batteryCalState.hasEmptyAnchor && batteryCalState.observedEmptyRawV > 0.0f) {
        return rawV * (BATTERY_EMPTY_V / batteryCalState.observedEmptyRawV);
    }

    return rawV * BATTERY_CAL_FACTOR;
}

static float batteryFilterSample(float calibratedV)
{
    if (calibratedV <= 0.0f) return calibratedV;

    batteryVoltageMedianWindow[batteryVoltageMedianIndex] = calibratedV;
    batteryVoltageMedianIndex = static_cast<uint8_t>((batteryVoltageMedianIndex + 1U) % BATTERY_MEDIAN_WINDOW);
    if (batteryVoltageMedianCount < BATTERY_MEDIAN_WINDOW) batteryVoltageMedianCount++;

    float sorted[BATTERY_MEDIAN_WINDOW];
    for (uint8_t i = 0; i < batteryVoltageMedianCount; ++i) sorted[i] = batteryVoltageMedianWindow[i];
    for (uint8_t i = 1; i < batteryVoltageMedianCount; ++i) {
        const float key = sorted[i];
        int8_t j = static_cast<int8_t>(i) - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            --j;
        }
        sorted[j + 1] = key;
    }
    const float median = sorted[batteryVoltageMedianCount / 2U];

    if (!batteryFilterInitialized) {
        batteryVoltage = median;
        batteryFilterInitialized = true;
        return batteryVoltage;
    }

    const float delta = median - batteryVoltage;
    float alpha = delta >= 0.0f ? BATTERY_FILTER_ALPHA_RISE : BATTERY_FILTER_ALPHA_FALL;
    if (fabsf(delta) >= BATTERY_FILTER_FAST_DELTA_V) alpha = BATTERY_FILTER_FAST_ALPHA;
    batteryVoltage += alpha * delta;
    return batteryVoltage;
}

float batteryCalibrationFactor()
{
    if (batteryRawVoltage > 0.0f) {
        const float calibrated = batteryCalibratedVoltageFromRaw(batteryRawVoltage);
        if (calibrated > 0.0f) return calibrated / batteryRawVoltage;
    }
    if (batteryCalState.hasFullAnchor && batteryCalState.observedFullRawV > 0.0f) {
        return BATTERY_FULL_V / batteryCalState.observedFullRawV;
    }
    if (batteryCalState.hasEmptyAnchor && batteryCalState.observedEmptyRawV > 0.0f) {
        return BATTERY_EMPTY_V / batteryCalState.observedEmptyRawV;
    }
    return BATTERY_CAL_FACTOR;
}

float readBatteryVoltage()
{
    uint32_t mvSum = 0;
    for (int i = 0; i < BATTERY_ADC_SAMPLES; i++) {
        for (int settle = 0; settle < BATTERY_ADC_SETTLE_READS; ++settle) {
            (void)analogReadMilliVolts(BATTERY_ADC_PIN);
            delayMicroseconds(BATTERY_ADC_SETTLE_US);
        }
        mvSum += analogReadMilliVolts(BATTERY_ADC_PIN);
        delayMicroseconds(BATTERY_ADC_SETTLE_US);
    }
    const uint32_t mv = mvSum / BATTERY_ADC_SAMPLES;
    const float pinV = static_cast<float>(mv) / 1000.0f;
    const float ratio = (BATTERY_DIVIDER_R_TOP + BATTERY_DIVIDER_R_BOTTOM) / BATTERY_DIVIDER_R_BOTTOM;
    return pinV * ratio;
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
    if (LIGHT_ADC_PIN < 0) return 0;
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
    const unsigned long now = millis();
    batteryRawVoltage = readBatteryVoltage();
    const float rawVoltage = batteryCalibratedVoltageFromRaw(batteryRawVoltage);
    batteryVoltage = batteryFilterSample(rawVoltage);
    batteryPercent = batteryPercentFromVoltage(batteryVoltage);

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
            Serial.printf("[CHG] raw=%.3f V=%.3f dv=%.4f score=%d charging=%d cal=%.4f\n",
                          batteryRawVoltage, batteryVoltage, dv, chargeTrendScore, batteryCharging ? 1 : 0,
                          batteryCalibrationFactor());
        }
    }

    if (batteryCharging) {
        if (!chargeSessionActive) {
            chargeSessionActive = true;
            chargeSessionStartMs = now;
            chargePlateauStartMs = 0;
            chargeSessionStartRawVoltage = batteryRawVoltage;
            chargeSessionMaxRawVoltage = batteryRawVoltage;
        } else {
            if (batteryRawVoltage > chargeSessionMaxRawVoltage) chargeSessionMaxRawVoltage = batteryRawVoltage;
        }

        if ((chargeSessionMaxRawVoltage - batteryRawVoltage) <= CHARGE_CAL_PLATEAU_BAND_V) {
            if (chargePlateauStartMs == 0) chargePlateauStartMs = now;
        } else {
            chargePlateauStartMs = 0;
        }

        const bool sessionLongEnough = (now - chargeSessionStartMs) >= CHARGE_CAL_SESSION_MIN_MS;
        const bool sessionRoseEnough = (chargeSessionMaxRawVoltage - chargeSessionStartRawVoltage) >= CHARGE_CAL_SESSION_MIN_RISE_V;
        const bool plateauLongEnough = chargePlateauStartMs != 0 && (now - chargePlateauStartMs) >= CHARGE_CAL_PLATEAU_HOLD_MS;
        if (batteryTrainingState.autoCalibrationEnabled &&
            sessionLongEnough && sessionRoseEnough && plateauLongEnough &&
            chargeSessionMaxRawVoltage >= CHARGE_CAL_FULL_MIN_RAW_V) {
            batteryCalibrationLearnFull(chargeSessionMaxRawVoltage);
        }
    } else if (chargeSessionActive) {
        chargeSessionActive = false;
        chargeSessionStartMs = 0;
        chargePlateauStartMs = 0;
        chargeSessionStartRawVoltage = 0.0f;
        chargeSessionMaxRawVoltage = 0.0f;
    }

    if (LIGHT_ADC_PIN >= 0) {
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
    } else {
        lightRawAdc = 0;
        lightPercent = 0;
        lightPercentFiltered = 0.0f;
        lightFilterInitialized = true;
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

const char *authName(wifi_auth_mode_t auth)
{
    return auth == WIFI_AUTH_OPEN ? "Open" : "Secured";
}

void refreshWifiScan()
{
    const wifi_mode_t scanMode = apModeActive ? WIFI_AP_STA : WIFI_STA;
    wifiEnsureRuntimeEnabled("scan", scanMode, apModeActive);
    const int scanState = WiFi.scanComplete();
    if (scanState == WIFI_SCAN_RUNNING) {
        uiStatusLine = "Searching for access points";
        if (lvglReady) lvglSyncStatusLine();
        return;
    }
    WiFi.scanDelete();
    wifiCount = 0;
    wifiScanInProgress = true;
    wifiScanAnimLastMs = millis();
    wifiScanAnimPhase = 0;
    const int scanStart = WiFi.scanNetworks(true, true);
    if (scanStart == WIFI_SCAN_FAILED) {
        wifiScanInProgress = false;
        uiStatusLine = "WiFi scan start failed";
        if (lvglReady) lvglSyncStatusLine();
        return;
    }
    uiStatusLine = "Searching for access points";
    if (lvglReady) lvglSyncStatusLine();
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

void clearWifiScanResults()
{
    wifiScanInProgress = false;
    wifiCount = 0;
    WiFi.scanDelete();
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
    snakeStarted = true;
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

void snakePrepareGame()
{
    snakeResetGame();
    snakeStarted = false;
    snakePaused = false;
    snakeGameOver = false;
}

void snakeStartGame()
{
    snakeResetGame();
    snakeStarted = true;
    snakePaused = false;
    snakeLastStepMs = millis();
}

static unsigned long snakeCurrentStepMs()
{
    const unsigned long foods = static_cast<unsigned long>(snakeScore / 10U);
    const unsigned long reduction = foods * SNAKE_STEP_MS_DELTA_PER_FOOD;
    if (reduction >= (SNAKE_STEP_MS_START - SNAKE_STEP_MS_MIN)) return SNAKE_STEP_MS_MIN;
    return SNAKE_STEP_MS_START - reduction;
}

void snakeStep()
{
    if (!snakeStarted || snakePaused || snakeGameOver) return;
    snakeDir = snakeNextDir;
    int nx = snakeX[0];
    int ny = snakeY[0];
    if (snakeDir == 0) ny--;
    else if (snakeDir == 1) nx++;
    else if (snakeDir == 2) ny++;
    else nx--;

    if (nx < 0 || nx >= SNAKE_COLS || ny < 0 || ny >= SNAKE_ROWS) {
        snakeGameOver = true;
        snakeStarted = false;
        snakeLastScore = snakeScore;
        saveGameValue("snake_last", snakeLastScore);
        snakeMaybeStoreHighScore(true);
        return;
    }
    if (snakeCellOccupied(nx, ny)) {
        snakeGameOver = true;
        snakeStarted = false;
        snakeLastScore = snakeScore;
        saveGameValue("snake_last", snakeLastScore);
        snakeMaybeStoreHighScore(true);
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
        snakeMaybeStoreHighScore();
        snakeSpawnFood();
    } else if (oldLen < snakeLen) {
        snakeLen = oldLen;
    }
}

void snakeTick()
{
    if (uiScreen != UI_GAME_SNAKE) return;
    if (!snakeStarted || snakePaused || snakeGameOver) return;
    if (millis() - snakeLastStepMs < snakeCurrentStepMs()) return;
    snakeLastStepMs = millis();
    snakeStep();
    lvglRefreshSnakeBoard();
}

#if defined(BOARD_ESP32S3_3248S035_N16R8)
static bool snake3dCellOccupied(int x, int y, int z)
{
    for (int i = 0; i < snake3dLen; ++i) {
        if (snake3dX[i] == x && snake3dY[i] == y && snake3dZ[i] == z) return true;
    }
    return false;
}

static void snake3dSpawnFood()
{
    for (int tries = 0; tries < 400; ++tries) {
        const int fx = random(0, SNAKE3D_COLS);
        const int fy = random(0, SNAKE3D_ROWS);
        const int fz = random(0, SNAKE3D_LAYERS);
        if (!snake3dCellOccupied(fx, fy, fz)) {
            snake3dFoodX = static_cast<int8_t>(fx);
            snake3dFoodY = static_cast<int8_t>(fy);
            snake3dFoodZ = static_cast<int8_t>(fz);
            return;
        }
    }
    snake3dFoodX = 0;
    snake3dFoodY = 0;
    snake3dFoodZ = 0;
}

void snake3dResetGame()
{
    snake3dLen = 4;
    snake3dDir = 1;
    snake3dNextDir = 1;
    snake3dViewDir = 1;
    snake3dStarted = true;
    snake3dPaused = false;
    snake3dGameOver = false;
    snake3dScore = 0;
    const int cy = SNAKE3D_ROWS / 2;
    const int cz = SNAKE3D_LAYERS / 2;
    for (int i = 0; i < snake3dLen; ++i) {
        snake3dX[i] = static_cast<int8_t>((SNAKE3D_COLS - 2) - i);
        snake3dY[i] = static_cast<int8_t>(cy);
        snake3dZ[i] = static_cast<int8_t>(cz);
    }
    snake3dSpawnFood();
    snake3dLastStepMs = millis();
}

void snake3dPrepareGame()
{
    snake3dResetGame();
    snake3dStarted = false;
    snake3dPaused = false;
    snake3dGameOver = false;
}

void snake3dStartGame()
{
    snake3dResetGame();
    snake3dStarted = true;
    snake3dPaused = false;
    snake3dLastStepMs = millis();
}

static unsigned long snake3dCurrentStepMs()
{
    const unsigned long foods = static_cast<unsigned long>(snake3dScore / 10U);
    const unsigned long reduction = foods * SNAKE3D_STEP_MS_DELTA_PER_FOOD;
    if (reduction >= (SNAKE3D_STEP_MS_START - SNAKE3D_STEP_MS_MIN)) return SNAKE3D_STEP_MS_MIN;
    return SNAKE3D_STEP_MS_START - reduction;
}

static void snake3dStep()
{
    if (!snake3dStarted || snake3dPaused || snake3dGameOver) return;
    snake3dDir = snake3dNextDir;
    int nx = snake3dX[0];
    int ny = snake3dY[0];
    int nz = snake3dZ[0];
    if (snake3dDir == 0) ny--;
    else if (snake3dDir == 1) nx++;
    else if (snake3dDir == 2) ny++;
    else if (snake3dDir == 3) nx--;
    else if (snake3dDir == 4) nz++;
    else nz--;

    if (nx < 0 || nx >= SNAKE3D_COLS || ny < 0 || ny >= SNAKE3D_ROWS || nz < 0 || nz >= SNAKE3D_LAYERS || snake3dCellOccupied(nx, ny, nz)) {
        snake3dGameOver = true;
        snake3dStarted = false;
        snake3dLastScore = snake3dScore;
        saveGameValue("snake3d_last", snake3dLastScore);
        snake3dMaybeStoreHighScore(true);
        return;
    }

    const bool ate = (nx == snake3dFoodX && ny == snake3dFoodY && nz == snake3dFoodZ);
    const int oldLen = snake3dLen;
    if (ate && snake3dLen < SNAKE3D_MAX_CELLS) snake3dLen++;
    for (int i = snake3dLen - 1; i > 0; --i) {
        snake3dX[i] = snake3dX[i - 1];
        snake3dY[i] = snake3dY[i - 1];
        snake3dZ[i] = snake3dZ[i - 1];
    }
    snake3dX[0] = static_cast<int8_t>(nx);
    snake3dY[0] = static_cast<int8_t>(ny);
    snake3dZ[0] = static_cast<int8_t>(nz);

    if (ate) {
        snake3dScore += 10;
        snake3dMaybeStoreHighScore(false);
        snake3dSpawnFood();
    } else if (oldLen < snake3dLen) {
        snake3dLen = oldLen;
    }
}

void snake3dTick()
{
    if (uiScreen != UI_GAME_SNAKE3D) return;
    const unsigned long now = millis();
    if (!snake3dStarted || snake3dPaused || snake3dGameOver) return;
    if (now - snake3dLastStepMs < snake3dCurrentStepMs()) return;
    snake3dLastStepMs = now;
    snake3dStep();
    lvglRefreshSnake3dBoard();
}
#endif

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
            lv_label_set_text_fmt(pinLbl, "HC-12 RXD %d  TXD %d  SET %d  9600 baud", hc12ActiveTxPin(), hc12ActiveRxPin(), hc12ActiveSetPin());
        } else {
            lv_label_set_text_fmt(pinLbl, "E220 RX %d  TX %d  M0 %d  M1 %d  9600 baud", e220ActiveTxPin(), e220ActiveRxPin(), e220ActiveM0Pin(), e220ActiveM1Pin());
        }
    }
    if (lvglHc12TerminalExamplesLabel) lv_label_set_text(lvglHc12TerminalExamplesLabel, hc12TerminalExampleCommands().c_str());
    lv_obj_t *terminalTa = hc12TerminalObj();
    if (terminalTa) {
        lv_textarea_set_text(terminalTa, hc12TerminalLog ? hc12TerminalLog->c_str() : "");
        lv_textarea_set_cursor_pos(terminalTa, LV_TEXTAREA_CURSOR_LAST);
        lv_textarea_set_placeholder_text(terminalTa, radioModuleType == RADIO_MODULE_HC12 ? "HC-12 terminal output" : "E220 terminal output");
    }
    lv_obj_t *cmdTa = hc12CmdTaObj();
    if (cmdTa) {
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
    if (lvglRadioChannelTitleLabel) lv_label_set_text(lvglRadioChannelTitleLabel, "Channel");
    if (lvglRadioBaudTitleLabel) lv_label_set_text(lvglRadioBaudTitleLabel, radioModuleType == RADIO_MODULE_HC12 ? "Baud Rate" : "UART Baud");
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
                                                      "HC-12 RXD %d  |  TXD %d  |  SET %d  |  9600 baud\nSerial Terminal and Info use HC-12 AT mode. Radio chat/discovery is only active while HC-12 is selected.",
                                                      hc12ActiveTxPin(), hc12ActiveRxPin(), hc12ActiveSetPin());
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
    if (hc12ApplyChannel(currentChannel <= minChannel ? maxChannel : (currentChannel - 1))) {
        lvglRefreshHc12ConfigUi();
    } else {
        lvglRefreshHc12ConfigUi();
    }
}

void lvglHc12NextChannelEvent(lv_event_t *e)
{
    (void)e;
    const int currentChannel = radioModuleType == RADIO_MODULE_HC12 ? hc12CurrentChannel : e220CurrentChannel;
    const int minChannel = radioModuleType == RADIO_MODULE_HC12 ? HC12_MIN_CHANNEL : E220_MIN_CHANNEL;
    const int maxChannel = radioModuleType == RADIO_MODULE_HC12 ? HC12_MAX_CHANNEL : E220_MAX_CHANNEL;
    if (hc12ApplyChannel(currentChannel >= maxChannel ? minChannel : (currentChannel + 1))) {
        lvglRefreshHc12ConfigUi();
    } else {
        lvglRefreshHc12ConfigUi();
    }
}

void lvglHc12PrevBaudEvent(lv_event_t *e)
{
    (void)e;
    const int baudCount = radioModuleType == RADIO_MODULE_HC12
                              ? static_cast<int>(sizeof(HC12_SUPPORTED_BAUDS) / sizeof(HC12_SUPPORTED_BAUDS[0]))
                              : static_cast<int>(sizeof(E220_UART_BAUD_VALUES) / sizeof(E220_UART_BAUD_VALUES[0]));
    int next = (radioModuleType == RADIO_MODULE_HC12 ? hc12CurrentBaudIndex : e220CurrentBaudIndex) - 1;
    if (next < 0) next = baudCount - 1;
    hc12ApplyBaudIndex(next);
    lvglRefreshHc12ConfigUi();
}

void lvglHc12NextBaudEvent(lv_event_t *e)
{
    (void)e;
    const int baudCount = radioModuleType == RADIO_MODULE_HC12
                              ? static_cast<int>(sizeof(HC12_SUPPORTED_BAUDS) / sizeof(HC12_SUPPORTED_BAUDS[0]))
                              : static_cast<int>(sizeof(E220_UART_BAUD_VALUES) / sizeof(E220_UART_BAUD_VALUES[0]));
    int next = (radioModuleType == RADIO_MODULE_HC12 ? hc12CurrentBaudIndex : e220CurrentBaudIndex) + 1;
    if (next >= baudCount) next = 0;
    hc12ApplyBaudIndex(next);
    lvglRefreshHc12ConfigUi();
}

void lvglHc12PrevModeEvent(lv_event_t *e)
{
    (void)e;
    int maxIndex = radioModuleType == RADIO_MODULE_HC12 ? 3 : 7;
    int next = (radioModuleType == RADIO_MODULE_HC12 ? hc12CurrentModeIndex : e220CurrentAirRateIndex) - 1;
    if (next < 0) next = maxIndex;
    hc12ApplyModeIndex(next);
    lvglRefreshHc12ConfigUi();
}

void lvglHc12NextModeEvent(lv_event_t *e)
{
    (void)e;
    int maxIndex = radioModuleType == RADIO_MODULE_HC12 ? 3 : 7;
    int next = (radioModuleType == RADIO_MODULE_HC12 ? hc12CurrentModeIndex : e220CurrentAirRateIndex) + 1;
    if (next > maxIndex) next = 0;
    hc12ApplyModeIndex(next);
    lvglRefreshHc12ConfigUi();
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
    hc12ApplyPowerLevel(next);
    lvglRefreshHc12ConfigUi();
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
    hc12ApplyPowerLevel(next);
    lvglRefreshHc12ConfigUi();
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
    e220ApplyTransferMode(!e220CurrentFixedTransmission);
    lvglRefreshHc12ConfigUi();
}

void lvglHc12NextExtraEvent(lv_event_t *e)
{
    (void)e;
    if (radioModuleType != RADIO_MODULE_E220) return;
    e220ApplyTransferMode(!e220CurrentFixedTransmission);
    lvglRefreshHc12ConfigUi();
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
    String versionRaw;
    String baudRaw;
    String channelRaw;
    String fuRaw;
    String powerRaw;
    String summaryRaw;

    hc12InitIfNeeded();
    if (radioModuleType == RADIO_MODULE_E220) {
        hc12RefreshInfoSnapshot();
        return;
    }
    while (Serial1.available() > 0) Serial1.read();
    digitalWrite(hc12ActiveSetPin(), LOW);
    delay(80);

    versionRaw = hc12QueryCommand("AT+V");
    baudRaw = hc12QueryCommand("AT+RB");
    channelRaw = hc12QueryCommand("AT+RC");
    fuRaw = hc12QueryCommand("AT+RF");
    powerRaw = hc12QueryCommand("AT+RP");
    summaryRaw = hc12QueryCommand("AT+RX", 220UL, 40UL);

    digitalWrite(hc12ActiveSetPin(), HIGH);
    delay(40);
    uiDeferredFlags &= static_cast<uint8_t>(~(UI_DEFERRED_HC12_SETTLE_PENDING | UI_DEFERRED_HC12_TARGET_ASSERTED));

    String version = hc12CompactResponse(versionRaw);
    if (version.isEmpty()) version = "No reply";

    String baud = hc12FieldValue(baudRaw, "B");
    if (baud != "--" && baud.indexOf("baud") < 0 && baud.indexOf("bps") < 0) baud += " bps";

    String channel = hc12FieldValue(channelRaw, "RC");
    if (channel != "--") channel = "CH" + channel;

    String fuMode = hc12FieldValue(fuRaw, "FU");
    if (fuMode != "--") fuMode = "FU" + fuMode;

    String power = hc12FieldValue(powerRaw, "RP:");
    if (power == "--") power = hc12FieldValue(powerRaw, "RP");

    String raw = hc12CompactResponse(summaryRaw);
    if (raw.isEmpty()) raw = "No summary response";

    hc12InfoValueText = version;
    hc12InfoSubText = raw;

    if (lvglHc12InfoVersionLabel) lv_label_set_text(lvglHc12InfoVersionLabel, version.c_str());
    if (lvglHc12InfoBaudLabel) lv_label_set_text(lvglHc12InfoBaudLabel, baud.c_str());
    if (lvglHc12InfoChannelLabel) lv_label_set_text(lvglHc12InfoChannelLabel, channel.c_str());
    if (lvglHc12InfoFuModeLabel) lv_label_set_text(lvglHc12InfoFuModeLabel, fuMode.c_str());
    if (lvglHc12InfoPowerLabel) lv_label_set_text(lvglHc12InfoPowerLabel, power.c_str());
    if (lvglHc12InfoRawLabel) lv_label_set_text(lvglHc12InfoRawLabel, raw.c_str());
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
    tetrisDropAnimFlags = 0;
    tetrisAnimDurationMs = 0;
    tetrisType = static_cast<int8_t>(random(0, 7));
    tetrisRot = 0;
    tetrisX = 3;
    tetrisY = 0;
    if (!tetrisCanPlace(tetrisX, tetrisY, tetrisType, tetrisRot)) {
        tetrisGameOver = true;
        tetrisStarted = false;
        tetrisLastScore = tetrisScore;
        saveGameValue("tetris_last", tetrisLastScore);
        tetrisMaybeStoreHighScore(true);
    }
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
        tetrisMaybeStoreHighScore();
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
    tetrisDropAnimFlags = 0;
    tetrisAnimDurationMs = 0;
    tetrisStarted = true;
    tetrisPaused = false;
    tetrisGameOver = false;
    tetrisScore = 0;
    tetrisSpawnPiece();
    tetrisLastStepMs = millis();
}

void tetrisPrepareGame()
{
    tetrisResetGame();
    tetrisStarted = false;
    tetrisPaused = false;
    tetrisGameOver = false;
}

void tetrisStartGame()
{
    tetrisResetGame();
    tetrisStarted = true;
    tetrisPaused = false;
    tetrisLastStepMs = millis();
}

static unsigned long tetrisCurrentStepMs()
{
    const unsigned long level = static_cast<unsigned long>(tetrisScore / 100U);
    const unsigned long reduction = level * TETRIS_STEP_MS_DELTA_PER_100_SCORE;
    if (reduction >= (TETRIS_STEP_MS_START - TETRIS_STEP_MS_MIN)) return TETRIS_STEP_MS_MIN;
    return TETRIS_STEP_MS_START - reduction;
}

void tetrisMove(int dx)
{
    if (!tetrisStarted || tetrisPaused || tetrisGameOver || (tetrisDropAnimFlags & TETRIS_DROP_ANIM_ACTIVE)) return;
    if (tetrisCanPlace(tetrisX + dx, tetrisY, tetrisType, tetrisRot)) {
        tetrisX += static_cast<int8_t>(dx);
        lvglRefreshTetrisBoard();
    }
}

void tetrisRotate()
{
    if (!tetrisStarted || tetrisPaused || tetrisGameOver || (tetrisDropAnimFlags & TETRIS_DROP_ANIM_ACTIVE)) return;
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
    if (!tetrisStarted || tetrisPaused || tetrisGameOver || (tetrisDropAnimFlags & TETRIS_DROP_ANIM_ACTIVE)) return;
    const int startY = tetrisY;
    while (tetrisCanPlace(tetrisX, tetrisY + 1, tetrisType, tetrisRot)) tetrisY++;
    if (tetrisY == startY) {
        tetrisLockPiece();
    } else {
        tetrisAnimFromY = static_cast<int8_t>(startY);
        tetrisAnimStartTick = static_cast<uint16_t>(millis());
        const unsigned long travelMs = static_cast<unsigned long>(tetrisY - startY) * TETRIS_DROP_ANIM_MS_PER_ROW;
        tetrisAnimDurationMs = min<unsigned long>(TETRIS_DROP_ANIM_MS_MAX,
                                                  max<unsigned long>(TETRIS_DROP_ANIM_MS_MIN, travelMs));
        tetrisDropAnimFlags = TETRIS_DROP_ANIM_ACTIVE | TETRIS_DROP_LOCK_PENDING;
    }
    lvglRefreshTetrisBoard();
}

void tetrisStepDown()
{
    if (!tetrisStarted || tetrisPaused || tetrisGameOver || (tetrisDropAnimFlags & TETRIS_DROP_ANIM_ACTIVE)) return;
    if (tetrisCanPlace(tetrisX, tetrisY + 1, tetrisType, tetrisRot)) tetrisY++;
    else tetrisLockPiece();
}

void tetrisAnimationService()
{
    if (uiScreen != UI_GAME_TETRIS || (tetrisDropAnimFlags & TETRIS_DROP_ANIM_ACTIVE) == 0) return;
    const uint16_t elapsedTick = static_cast<uint16_t>(millis()) - tetrisAnimStartTick;
    if (elapsedTick >= tetrisAnimDurationMs) {
        const bool lockPending = (tetrisDropAnimFlags & TETRIS_DROP_LOCK_PENDING) != 0;
        tetrisDropAnimFlags = 0;
        tetrisAnimDurationMs = 0;
        if (lockPending) tetrisLockPiece();
        lvglRefreshTetrisBoard();
        return;
    }
    if (lvglTetrisBoardObj) lv_obj_invalidate(lvglTetrisBoardObj);
}

void tetrisTick()
{
    if (uiScreen != UI_GAME_TETRIS) return;
    if (!tetrisStarted || tetrisPaused || tetrisGameOver || (tetrisDropAnimFlags & TETRIS_DROP_ANIM_ACTIVE)) return;
    if (millis() - tetrisLastStepMs < tetrisCurrentStepMs()) return;
    tetrisLastStepMs = millis();
    tetrisStepDown();
    lvglRefreshTetrisBoard();
}

static bool checkersInBounds(int x, int y)
{
    const int boardSize = checkersBoardSize();
    return x >= 0 && x < boardSize && y >= 0 && y < boardSize;
}

static int8_t checkersPieceSide(int8_t piece)
{
    if (piece > 0) return 1;
    if (piece < 0) return -1;
    return 0;
}

static bool checkersIsKing(int8_t piece)
{
    return piece == 2 || piece == -2;
}

static bool checkersIsDarkSquare(int x, int y)
{
    return ((x + y) & 1) == 1;
}

static int checkersBoardSize()
{
    switch (checkersVariant) {
        case CHECKERS_VARIANT_INTERNATIONAL:
            return 10;
        case CHECKERS_VARIANT_CANADIAN:
            return 12;
        default:
            return 8;
    }
}

static int checkersStartRows()
{
    switch (checkersVariant) {
        case CHECKERS_VARIANT_INTERNATIONAL:
            return 4;
        case CHECKERS_VARIANT_CANADIAN:
            return 5;
        default:
            return 3;
    }
}

static bool checkersMenCaptureBackward()
{
    return checkersVariant != CHECKERS_VARIANT_AMERICAN;
}

static bool checkersFlyingKings()
{
    return checkersVariant != CHECKERS_VARIANT_AMERICAN;
}

static bool checkersPromotionContinuesCapture()
{
    return checkersVariant == CHECKERS_VARIANT_RUSSIAN;
}

static const char *checkersVariantName(CheckersVariant variant)
{
    switch (variant) {
        case CHECKERS_VARIANT_AMERICAN: return "American Checkers";
        case CHECKERS_VARIANT_INTERNATIONAL: return "International Draughts";
        case CHECKERS_VARIANT_RUSSIAN: return "Russian Checkers";
        case CHECKERS_VARIANT_POOL: return "Pool Checkers";
        case CHECKERS_VARIANT_CANADIAN: return "Canadian/Sri Lankan";
        default: return "Checkers";
    }
}

static void checkersClearSelection()
{
    checkersSelectedX = -1;
    checkersSelectedY = -1;
    checkersForcedX = -1;
    checkersForcedY = -1;
}

static void checkersClearSession()
{
    checkersSessionId = "";
    checkersPeerKey = "";
    checkersLocalSide = 0;
    checkersWaitingForRemote = false;
    checkersAiDueMs = 0;
}

static String checkersPeerDisplayName()
{
    if (checkersPeerKey.isEmpty()) return "peer";
    const String name = chatDisplayNameForPeerKey(checkersPeerKey);
    return name.isEmpty() ? String("peer") : name;
}

static void checkersResetBoard()
{
    const int boardSize = checkersBoardSize();
    const int startRows = checkersStartRows();
    for (int y = 0; y < CHECKERS_MAX_BOARD_SIZE; ++y) {
        for (int x = 0; x < CHECKERS_MAX_BOARD_SIZE; ++x) {
            checkersBoard[y][x] = 0;
            if (x >= boardSize || y >= boardSize) continue;
            if (!checkersIsDarkSquare(x, y)) continue;
            if (y < startRows) checkersBoard[y][x] = -1;
            else if (y >= (boardSize - startRows)) checkersBoard[y][x] = 1;
        }
    }
    checkersTurn = 1;
    checkersStarted = true;
    checkersGameOver = false;
    checkersWinnerSide = 0;
    checkersAiDueMs = 0;
    checkersVariantPopupOpen = false;
    checkersClearSelection();
}

static int checkersCollectPieceMoves(int x, int y, bool captureOnly, CheckersMove *out, int maxOut)
{
    if (!checkersInBounds(x, y)) return 0;
    const int8_t piece = checkersBoard[y][x];
    if (piece == 0) return 0;
    const int8_t side = checkersPieceSide(piece);
    const bool king = checkersIsKing(piece);
    const int dirs[4][2] = {{-1, -1}, {1, -1}, {-1, 1}, {1, 1}};
    int count = 0;

    for (int i = 0; i < 4; ++i) {
        const int dx = dirs[i][0];
        const int dy = dirs[i][1];
        if (!king && !checkersMenCaptureBackward()) {
            if (side == 1 && dy != -1) continue;
            if (side == -1 && dy != 1) continue;
        }

        if (king && checkersFlyingKings()) {
            int step = 1;
            int capX = -1;
            int capY = -1;
            while (true) {
                const int toX = x + (dx * step);
                const int toY = y + (dy * step);
                if (!checkersInBounds(toX, toY)) break;
                const int8_t target = checkersBoard[toY][toX];
                if (!captureOnly) {
                    if (target != 0) break;
                    if (out && count < maxOut) {
                        out[count] = {static_cast<int8_t>(x), static_cast<int8_t>(y), static_cast<int8_t>(toX), static_cast<int8_t>(toY), -1, -1, false};
                    }
                    count++;
                    step++;
                    continue;
                }

                if (target == 0) {
                    if (capX >= 0) {
                        if (out && count < maxOut) {
                            out[count] = {static_cast<int8_t>(x), static_cast<int8_t>(y), static_cast<int8_t>(toX), static_cast<int8_t>(toY), static_cast<int8_t>(capX), static_cast<int8_t>(capY), true};
                        }
                        count++;
                    }
                    step++;
                    continue;
                }

                if (checkersPieceSide(target) == side || capX >= 0) break;
                capX = toX;
                capY = toY;
                step++;
                continue;
            }
            continue;
        }

        if (captureOnly) {
            const int midX = x + dx;
            const int midY = y + dy;
            const int toX = x + (dx * 2);
            const int toY = y + (dy * 2);
            if (!checkersInBounds(midX, midY) || !checkersInBounds(toX, toY)) continue;
            const int8_t midPiece = checkersBoard[midY][midX];
            if (midPiece == 0 || checkersPieceSide(midPiece) == side || checkersBoard[toY][toX] != 0) continue;
            if (out && count < maxOut) {
                out[count] = {static_cast<int8_t>(x), static_cast<int8_t>(y), static_cast<int8_t>(toX), static_cast<int8_t>(toY), static_cast<int8_t>(midX), static_cast<int8_t>(midY), true};
            }
            count++;
        } else {
            if (!king) {
                if (side == 1 && dy != -1) continue;
                if (side == -1 && dy != 1) continue;
            }
            const int toX = x + dx;
            const int toY = y + dy;
            if (!checkersInBounds(toX, toY) || checkersBoard[toY][toX] != 0) continue;
            if (out && count < maxOut) {
                out[count] = {static_cast<int8_t>(x), static_cast<int8_t>(y), static_cast<int8_t>(toX), static_cast<int8_t>(toY), -1, -1, false};
            }
            count++;
        }
    }

    return count;
}

static bool checkersSideHasCapture(int8_t side)
{
    if (checkersForcedX >= 0 && checkersForcedY >= 0) {
        return checkersCollectPieceMoves(checkersForcedX, checkersForcedY, true, nullptr, 0) > 0;
    }

    const int boardSize = checkersBoardSize();
    for (int y = 0; y < boardSize; ++y) {
        for (int x = 0; x < boardSize; ++x) {
            if (checkersPieceSide(checkersBoard[y][x]) != side) continue;
            if (checkersCollectPieceMoves(x, y, true, nullptr, 0) > 0) return true;
        }
    }
    return false;
}

static int checkersCollectLegalMovesForPiece(int x, int y, CheckersMove *out, int maxOut)
{
    if (!checkersInBounds(x, y)) return 0;
    if (checkersPieceSide(checkersBoard[y][x]) != checkersTurn) return 0;
    if (checkersForcedX >= 0 && checkersForcedY >= 0 && (x != checkersForcedX || y != checkersForcedY)) return 0;
    const bool requireCapture = checkersSideHasCapture(checkersTurn);
    return checkersCollectPieceMoves(x, y, requireCapture, out, maxOut);
}

static int checkersCollectLegalMovesForTurn(CheckersMove *out, int maxOut)
{
    int count = 0;
    if (checkersForcedX >= 0 && checkersForcedY >= 0) {
        return checkersCollectLegalMovesForPiece(checkersForcedX, checkersForcedY, out, maxOut);
    }

    const bool requireCapture = checkersSideHasCapture(checkersTurn);
    const int boardSize = checkersBoardSize();
    for (int y = 0; y < boardSize; ++y) {
        for (int x = 0; x < boardSize; ++x) {
            if (checkersPieceSide(checkersBoard[y][x]) != checkersTurn) continue;
            CheckersMove local[CHECKERS_MAX_HINT_MOVES];
            const int localCount = checkersCollectPieceMoves(x, y, requireCapture, local, CHECKERS_MAX_HINT_MOVES);
            for (int i = 0; i < localCount; ++i) {
                if (out && count < maxOut) out[count] = local[i];
                count++;
            }
        }
    }
    return count;
}

static void checkersUpdateHintMoves()
{
    if (checkersSelectedX < 0 || checkersSelectedY < 0) return;
    CheckersMove moves[CHECKERS_MAX_HINT_MOVES];
    const int moveCount = checkersCollectLegalMovesForPiece(checkersSelectedX, checkersSelectedY, moves, CHECKERS_MAX_HINT_MOVES);
    if (moveCount <= 0 && checkersForcedX < 0) {
        checkersSelectedX = -1;
        checkersSelectedY = -1;
    }
}

static String checkersModeLabelText()
{
    const String variant = String(checkersVariantName(checkersVariant));
    const String stats = " W " + String(checkersLocalWins) + ":" + String(checkersRemoteWins);
    switch (checkersMode) {
        case CHECKERS_MODE_ESP32:
            return variant + " / ESP32" + stats;
        case CHECKERS_MODE_TAG:
            if (!checkersPeerKey.isEmpty()) return variant + " / Tag MP: " + checkersPeerDisplayName() + stats;
            return variant + " / Tag Multiplayer" + stats;
        default:
            return variant + stats;
    }
}

static String checkersTurnLabelText()
{
    if (checkersGameOver) {
        if (checkersMode == CHECKERS_MODE_ESP32) return (checkersWinnerSide == 1) ? "You win" : "ESP32 wins";
        if (checkersMode == CHECKERS_MODE_TAG) return (checkersWinnerSide == checkersLocalSide) ? "You win" : (checkersPeerDisplayName() + " wins");
        return (checkersWinnerSide == 1) ? "Red wins" : "Blue wins";
    }
    if (!checkersStarted) return "Pick a mode";
    if (checkersWaitingForRemote) return "Pending";
    if (checkersMode == CHECKERS_MODE_ESP32) {
        return (checkersTurn == 1) ? "Your move" : "ESP32";
    }
    if (checkersMode == CHECKERS_MODE_TAG) {
        if (checkersTurn == checkersLocalSide) return "Your move";
        return checkersPeerDisplayName();
    }
    return (checkersTurn == 1) ? "Red" : "Blue";
}

static void checkersFinishGame(int8_t winnerSide)
{
    checkersGameOver = true;
    checkersWinnerSide = winnerSide;
    checkersAiDueMs = 0;
    checkersWaitingForRemote = false;
    if ((checkersMode == CHECKERS_MODE_ESP32 || checkersMode == CHECKERS_MODE_TAG) && checkersLocalSide != 0) {
        if (winnerSide == checkersLocalSide) {
            checkersLocalWins++;
            saveGameValue("checkers_local_wins", checkersLocalWins);
        } else if (winnerSide == -checkersLocalSide) {
            checkersRemoteWins++;
            saveGameValue("checkers_remote_wins", checkersRemoteWins);
        }
    }
    checkersClearSelection();
}

static void checkersEvaluateGameOver()
{
    CheckersMove legal[CHECKERS_MAX_MOVES];
    if (checkersCollectLegalMovesForTurn(legal, CHECKERS_MAX_MOVES) > 0) return;

    if (checkersMode == CHECKERS_MODE_ESP32) {
        checkersFinishGame(static_cast<int8_t>(-checkersTurn));
    } else if (checkersMode == CHECKERS_MODE_TAG) {
        checkersFinishGame(static_cast<int8_t>(-checkersTurn));
    } else {
        checkersFinishGame(static_cast<int8_t>(-checkersTurn));
    }
}

static bool checkersApplyMove(const CheckersMove &move, bool broadcastMove)
{
    const int boardSize = checkersBoardSize();
    CheckersMove legal[CHECKERS_MAX_MOVES];
    const int legalCount = checkersCollectLegalMovesForPiece(move.fromX, move.fromY, legal, CHECKERS_MAX_MOVES);
    int match = -1;
    for (int i = 0; i < legalCount; ++i) {
        if (legal[i].toX == move.toX && legal[i].toY == move.toY) {
            match = i;
            break;
        }
    }
    if (match < 0) return false;

    const CheckersMove applied = legal[match];
    int8_t piece = checkersBoard[applied.fromY][applied.fromX];
    checkersBoard[applied.fromY][applied.fromX] = 0;
    checkersBoard[applied.toY][applied.toX] = piece;
    if (applied.capture) checkersBoard[applied.captureY][applied.captureX] = 0;
    bool promoted = false;
    if (piece == 1 && applied.toY == 0) {
        piece = 2;
        promoted = true;
    } else if (piece == -1 && applied.toY == (boardSize - 1)) {
        piece = -2;
        promoted = true;
    }
    checkersBoard[applied.toY][applied.toX] = piece;

    if (broadcastMove && checkersMode == CHECKERS_MODE_TAG && !checkersPeerKey.isEmpty() && !checkersSessionId.isEmpty()) {
        chatSendRawReliableMessage(checkersPeerKey, checkersBuildMoveText(checkersSessionId, applied), false);
    }

    const bool allowFollowUpCapture = applied.capture && (!promoted || checkersPromotionContinuesCapture());
    if (allowFollowUpCapture && checkersCollectPieceMoves(applied.toX, applied.toY, true, nullptr, 0) > 0) {
        checkersForcedX = applied.toX;
        checkersForcedY = applied.toY;
        if (checkersTurn == checkersLocalSide) {
            checkersSelectedX = applied.toX;
            checkersSelectedY = applied.toY;
        } else {
            checkersSelectedX = -1;
            checkersSelectedY = -1;
        }
    } else {
        checkersClearSelection();
        checkersTurn = static_cast<int8_t>(-checkersTurn);
        checkersEvaluateGameOver();
    }

    checkersUpdateHintMoves();
    if (!checkersGameOver && checkersMode == CHECKERS_MODE_ESP32 && checkersTurn == -1) {
        checkersAiDueMs = millis() + 360UL;
    } else if (checkersMode != CHECKERS_MODE_ESP32 || checkersTurn != -1) {
        checkersAiDueMs = 0;
    }
    return true;
}

static void checkersStartEsp32Game()
{
    checkersClearSession();
    checkersMode = CHECKERS_MODE_ESP32;
    checkersLocalSide = 1;
    checkersPeerPopupOpen = false;
    checkersResetBoard();
    checkersUpdateHintMoves();
    uiStatusLine = String(checkersVariantName(checkersVariant)) + " vs ESP32";
    if (lvglReady) lvglSyncStatusLine();
}

static void checkersStartPendingInvite(const String &peerKey)
{
    if (peerKey.isEmpty()) return;
    checkersMode = CHECKERS_MODE_TAG;
    checkersPeerKey = peerKey;
    checkersSessionId = chatGenerateMessageId();
    checkersLocalSide = 1;
    checkersWaitingForRemote = true;
    checkersPeerPopupOpen = false;
    checkersResetBoard();
    checkersUpdateHintMoves();
    chatSendRawReliableMessage(peerKey, checkersBuildInviteText(checkersSessionId, deviceShortNameValue()), true);
    uiStatusLine = String(checkersVariantName(checkersVariant)) + " invite sent to " + checkersPeerDisplayName();
    if (lvglReady) lvglSyncStatusLine();
}

static void checkersStartAcceptedInvite(const String &peerKey, const String &sessionId, CheckersVariant variant)
{
    if (peerKey.isEmpty() || sessionId.isEmpty()) return;
    checkersVariant = variant;
    checkersMode = CHECKERS_MODE_TAG;
    checkersPeerKey = peerKey;
    checkersSessionId = sessionId;
    checkersLocalSide = -1;
    checkersWaitingForRemote = false;
    checkersPeerPopupOpen = false;
    checkersResetBoard();
    checkersUpdateHintMoves();
    chatSendRawReliableMessage(peerKey, checkersBuildAcceptText(sessionId), false);
    uiStatusLine = String(checkersVariantName(checkersVariant)) + " started with " + checkersPeerDisplayName();
    if (lvglReady) lvglSyncStatusLine();
}

static void checkersOpenPeerPopup()
{
    checkersPeerPopupOpen = true;
    if (lvglReady && uiScreen == UI_GAME_CHECKERS) lvglRefreshCheckersBoard();
}

static void checkersClosePeerPopup()
{
    checkersPeerPopupOpen = false;
    if (lvglCheckersPeerPopup) lv_obj_add_flag(lvglCheckersPeerPopup, LV_OBJ_FLAG_HIDDEN);
}

static bool checkersHandleIncomingChatPayload(const String &peerKey,
                                              const String &author,
                                              const String &text,
                                              ChatTransport transport,
                                              const String &messageId)
{
    String sessionId;
    if (checkersParseInviteText(text, sessionId)) {
        if (messageId.isEmpty() || !chatHasLoggedMessageId(peerKey, messageId)) {
            chatStoreMessage(peerKey, author, text, false, transport, messageId);
        }
        uiStatusLine = "Checkers invite from " + author;
        return true;
    }

    if (checkersParseAcceptText(text, sessionId)) {
        if (gameControlMessageSeen(messageId)) return true;
        gameRememberControlMessageId(messageId);
        if (checkersMode == CHECKERS_MODE_TAG &&
            checkersWaitingForRemote &&
            checkersSessionId == sessionId &&
            checkersPeerKey == peerKey) {
            checkersWaitingForRemote = false;
            uiStatusLine = author + " joined Checkers";
            if (lvglReady && uiScreen == UI_GAME_CHECKERS) lvglRefreshCheckersBoard();
        }
        return true;
    }

    if (checkersParseDeclineText(text, sessionId)) {
        if (gameControlMessageSeen(messageId)) return true;
        gameRememberControlMessageId(messageId);
        if (checkersMode == CHECKERS_MODE_TAG &&
            checkersWaitingForRemote &&
            checkersSessionId == sessionId &&
            checkersPeerKey == peerKey) {
            checkersWaitingForRemote = false;
            checkersStarted = false;
            checkersMode = CHECKERS_MODE_IDLE;
            checkersClearSession();
            uiStatusLine = author + " declined Checkers";
            if (lvglReady && uiScreen == UI_GAME_CHECKERS) lvglRefreshCheckersBoard();
        }
        return true;
    }

    CheckersMove remoteMove = {};
    if (checkersParseMoveText(text, sessionId, remoteMove)) {
        if (gameControlMessageSeen(messageId)) return true;
        gameRememberControlMessageId(messageId);
        if (checkersMode == CHECKERS_MODE_TAG &&
            !checkersWaitingForRemote &&
            checkersSessionId == sessionId &&
            checkersPeerKey == peerKey &&
            checkersTurn != checkersLocalSide) {
            if (checkersApplyMove(remoteMove, false)) {
                uiStatusLine = "Checkers move from " + author;
                if (lvglReady && uiScreen == UI_GAME_CHECKERS) lvglRefreshCheckersBoard();
            }
        }
        return true;
    }

    return false;
}

static bool checkersLocalTurnActive()
{
    if (!checkersStarted || checkersGameOver || checkersWaitingForRemote || checkersVariantPopupOpen) return false;
    if (checkersMode == CHECKERS_MODE_ESP32) return checkersTurn == 1;
    if (checkersMode == CHECKERS_MODE_TAG) return checkersTurn == checkersLocalSide;
    return false;
}

static void checkersHandleBoardTap(int cellX, int cellY)
{
    if (!checkersLocalTurnActive()) return;
    if (!checkersInBounds(cellX, cellY) || !checkersIsDarkSquare(cellX, cellY)) return;

    if (checkersSelectedX >= 0 && checkersSelectedY >= 0) {
        CheckersMove hintMoves[CHECKERS_MAX_HINT_MOVES];
        const int hintMoveCount = checkersCollectLegalMovesForPiece(checkersSelectedX, checkersSelectedY, hintMoves, CHECKERS_MAX_HINT_MOVES);
        CheckersMove chosen = {};
        bool found = false;
        for (int i = 0; i < hintMoveCount; ++i) {
            if (hintMoves[i].toX == cellX && hintMoves[i].toY == cellY) {
                chosen = hintMoves[i];
                found = true;
                break;
            }
        }
        if (found) {
            if (checkersApplyMove(chosen, checkersMode == CHECKERS_MODE_TAG)) {
                lvglRefreshCheckersBoard();
            }
            return;
        }
    }

    if (checkersPieceSide(checkersBoard[cellY][cellX]) == checkersTurn) {
        checkersSelectedX = static_cast<int8_t>(cellX);
        checkersSelectedY = static_cast<int8_t>(cellY);
        checkersUpdateHintMoves();
        CheckersMove hintMoves[CHECKERS_MAX_HINT_MOVES];
        const int hintMoveCount = checkersCollectLegalMovesForPiece(checkersSelectedX, checkersSelectedY, hintMoves, CHECKERS_MAX_HINT_MOVES);
        if (hintMoveCount <= 0) {
            if (checkersForcedX < 0) checkersSelectedX = -1;
            if (checkersForcedY < 0) checkersSelectedY = -1;
        }
        lvglRefreshCheckersBoard();
        return;
    }

    if (checkersForcedX < 0) {
        checkersSelectedX = -1;
        checkersSelectedY = -1;
        lvglRefreshCheckersBoard();
    }
}

void lvglCheckersBoardEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (lvglClickSuppressed()) return;
    lv_obj_t *obj = lv_event_get_target(e);
    if (!obj) return;
    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) return;
    lv_point_t pt = {0, 0};
    lv_indev_get_point(indev, &pt);
    lv_area_t coords;
    lv_obj_get_coords(obj, &coords);
    const int boardSize = checkersBoardSize();
    const int width = static_cast<int>(coords.x2 - coords.x1 + 1);
    const int height = static_cast<int>(coords.y2 - coords.y1 + 1);
    const int cellW = width / boardSize;
    const int cellH = height / boardSize;
    if (cellW <= 0 || cellH <= 0) return;
    const int localX = pt.x - coords.x1;
    const int localY = pt.y - coords.y1;
    const int cellX = localX / cellW;
    const int cellY = localY / cellH;
    checkersHandleBoardTap(cellX, cellY);
}

static void checkersDrawPiece(lv_draw_ctx_t *drawCtx, int x, int y, int w, int h, lv_color_t fill, bool king, bool selected)
{
    lv_draw_rect_dsc_t dsc;
    lv_draw_rect_dsc_init(&dsc);
    dsc.bg_opa = LV_OPA_COVER;
    dsc.bg_color = fill;
    dsc.radius = LV_RADIUS_CIRCLE;
    dsc.border_width = selected ? 3 : 2;
    dsc.border_color = selected ? lv_color_hex(0xFFF1A8) : lv_color_hex(0xD6DEE8);
    const int inset = max(2, min(w, h) / 8);
    lv_area_t area = {
        static_cast<lv_coord_t>(x + inset),
        static_cast<lv_coord_t>(y + inset),
        static_cast<lv_coord_t>(x + w - inset),
        static_cast<lv_coord_t>(y + h - inset)
    };
    lv_draw_rect(drawCtx, &dsc, &area);

    if (!king) return;
    lv_draw_rect_dsc_t crownDsc;
    lv_draw_rect_dsc_init(&crownDsc);
    crownDsc.bg_opa = LV_OPA_TRANSP;
    crownDsc.radius = LV_RADIUS_CIRCLE;
    crownDsc.border_width = 2;
    crownDsc.border_color = lv_color_hex(0xFFE08A);
    const int crownInset = inset + max(2, min(w, h) / 7);
    lv_area_t crownArea = {
        static_cast<lv_coord_t>(x + crownInset),
        static_cast<lv_coord_t>(y + crownInset),
        static_cast<lv_coord_t>(x + w - crownInset),
        static_cast<lv_coord_t>(y + h - crownInset)
    };
    lv_draw_rect(drawCtx, &crownDsc, &crownArea);
}

void lvglCheckersBoardDrawEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_DRAW_MAIN) return;
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *drawCtx = lv_event_get_draw_ctx(e);
    if (!obj || !drawCtx) return;

    lv_area_t coords;
    lv_obj_get_coords(obj, &coords);
    const int boardSize = checkersBoardSize();
    const int width = static_cast<int>(coords.x2 - coords.x1 + 1);
    const int height = static_cast<int>(coords.y2 - coords.y1 + 1);
    const int cellW = width / boardSize;
    const int cellH = height / boardSize;
    if (cellW < 5 || cellH < 5) return;
    CheckersMove hintMoves[CHECKERS_MAX_HINT_MOVES];
    const int hintMoveCount = (checkersSelectedX >= 0 && checkersSelectedY >= 0)
                                  ? checkersCollectLegalMovesForPiece(checkersSelectedX, checkersSelectedY, hintMoves, CHECKERS_MAX_HINT_MOVES)
                                  : 0;

    for (int y = 0; y < boardSize; ++y) {
        for (int x = 0; x < boardSize; ++x) {
            const int px = static_cast<int>(coords.x1) + (x * cellW);
            const int py = static_cast<int>(coords.y1) + (y * cellH);
            lv_color_t square = checkersIsDarkSquare(x, y) ? lv_color_hex(0x314355) : lv_color_hex(0xD5C7AF);
            if (x == checkersSelectedX && y == checkersSelectedY) square = lv_color_hex(0x5B7244);
            for (int i = 0; i < hintMoveCount; ++i) {
                if (hintMoves[i].toX == x && hintMoves[i].toY == y) {
                    square = hintMoves[i].capture ? lv_color_hex(0x7D4A3B) : lv_color_hex(0x4B657A);
                    break;
                }
            }
            lvglDrawGridCell(drawCtx, px, py, cellW - 1, cellH - 1, square);

            const int8_t piece = checkersBoard[y][x];
            if (piece == 0) continue;
            checkersDrawPiece(drawCtx,
                              px + 1,
                              py + 1,
                              cellW - 2,
                              cellH - 2,
                              piece > 0 ? lv_color_hex(0xD94A4A) : lv_color_hex(0x2D3F8E),
                              checkersIsKing(piece),
                              x == checkersSelectedX && y == checkersSelectedY);
        }
    }
}

static void checkersRefreshPeerPopup()
{
    if (!lvglCheckersPeerPopup || !lvglCheckersPeerPopupList) return;
    if (!checkersPeerPopupOpen) {
        lv_obj_add_flag(lvglCheckersPeerPopup, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    lv_obj_clear_flag(lvglCheckersPeerPopup, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(lvglCheckersPeerPopup);
    lv_obj_clean(lvglCheckersPeerPopupList);

    if (lvglCheckersPeerPopupTitle) lv_label_set_text(lvglCheckersPeerPopupTitle, "Choose Contact");

    bool havePeers = false;
    for (int i = 0; i < p2pPeerCount; ++i) {
        if (!p2pPeers[i].enabled) continue;
        havePeers = true;
        lv_obj_t *btn = lv_btn_create(lvglCheckersPeerPopupList);
        lv_obj_set_width(btn, lv_pct(100));
        lv_obj_set_height(btn, 34);
        lv_obj_set_style_radius(btn, 10, 0);
        lv_obj_set_style_border_width(btn, 0, 0);
        lv_obj_add_event_cb(btn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(btn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(btn, lvglCheckersPeerSelectEvent, LV_EVENT_CLICKED, reinterpret_cast<void *>(static_cast<intptr_t>(i)));
        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, p2pPeers[i].name.isEmpty() ? "Peer" : p2pPeers[i].name.c_str());
        lv_obj_center(lbl);
        lvglRegisterStyledButton(btn, lv_color_hex(0x2F6D86), true);
        const String orderKey = lvglOrderTokenFromText("p", p2pPeers[i].pubKeyHex);
        lvglRegisterReorderableItem(btn, "ord_ckp", orderKey.c_str());
    }

    if (!havePeers) {
        lv_obj_t *lbl = lv_label_create(lvglCheckersPeerPopupList);
        lv_obj_set_width(lbl, lv_pct(100));
        lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_WRAP);
        lv_label_set_text(lbl, "No paired contacts.\nOpen Chat Peers first and pair another device.");
        lvglRegisterReorderableItem(lbl, "ord_ckp", "empty");
    }
    lvglApplySavedOrder(lvglCheckersPeerPopupList, "ord_ckp");
}

static void checkersRefreshVariantPopup()
{
    if (!lvglCheckersVariantPopup || !lvglCheckersVariantPopupList) return;
    if (!checkersVariantPopupOpen) {
        lv_obj_add_flag(lvglCheckersVariantPopup, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    lv_obj_clear_flag(lvglCheckersVariantPopup, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(lvglCheckersVariantPopup);
    lv_obj_clean(lvglCheckersVariantPopupList);

    if (lvglCheckersVariantPopupTitle) lv_label_set_text(lvglCheckersVariantPopupTitle, "Choose Rule Set");

    static const CheckersVariant variants[] = {
        CHECKERS_VARIANT_AMERICAN,
        CHECKERS_VARIANT_INTERNATIONAL,
        CHECKERS_VARIANT_RUSSIAN,
        CHECKERS_VARIANT_POOL,
        CHECKERS_VARIANT_CANADIAN
    };

    for (size_t i = 0; i < (sizeof(variants) / sizeof(variants[0])); ++i) {
        lv_obj_t *btn = lv_btn_create(lvglCheckersVariantPopupList);
        if (!btn) continue;
        lv_obj_set_width(btn, lv_pct(100));
        lv_obj_set_height(btn, 28);
        lv_obj_set_style_radius(btn, 10, 0);
        lv_obj_set_style_border_width(btn, 0, 0);
        lv_obj_add_event_cb(btn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(btn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(btn,
                            lvglCheckersVariantSelectEvent,
                            LV_EVENT_CLICKED,
                            reinterpret_cast<void *>(static_cast<intptr_t>(variants[i])));
        lv_obj_t *lbl = lv_label_create(btn);
        if (!lbl) continue;
        lv_label_set_text(lbl, checkersVariantName(variants[i]));
        lv_obj_center(lbl);
        lvglRegisterStyledButton(btn, variants[i] == checkersVariant ? lv_color_hex(0x3A7A3A) : lv_color_hex(0x2F4658), true);
        char orderKey[16];
        snprintf(orderKey, sizeof(orderKey), "v%d", static_cast<int>(variants[i]));
        lvglRegisterReorderableItem(btn, "ord_ckv", orderKey);
    }
    lvglApplySavedOrder(lvglCheckersVariantPopupList, "ord_ckv");
}

void lvglRefreshCheckersBoard()
{
    checkersUpdateHintMoves();
    if (lvglCheckersBoardObj) lv_obj_invalidate(lvglCheckersBoardObj);
    if (lvglCheckersModeLabel) lv_label_set_text(lvglCheckersModeLabel, checkersModeLabelText().c_str());
    if (lvglCheckersTurnLabel) lv_label_set_text(lvglCheckersTurnLabel, checkersTurnLabelText().c_str());

    if (lvglCheckersOverlay) {
        bool showOverlay = false;
        String title = checkersVariantName(checkersVariant);
        String msg = "";
        bool showBtn = false;
        String btnText = "Replay";

        if (checkersGameOver) {
            showOverlay = true;
            title = checkersTurnLabelText();
            msg = (checkersMode == CHECKERS_MODE_TAG && !checkersPeerKey.isEmpty())
                      ? ("Match finished with " + checkersPeerDisplayName())
                      : "Tap Replay to start again.";
            showBtn = true;
        } else if (checkersWaitingForRemote) {
            showOverlay = true;
            title = "Tag Multiplayer";
            msg = "Invitation sent to " + checkersPeerDisplayName() + "\nWaiting for Play...";
        } else if (!checkersStarted) {
            showOverlay = true;
            title = checkersVariantPopupOpen ? "Select Checkers Type" : String(checkersVariantName(checkersVariant));
            msg = checkersVariantPopupOpen
                      ? "Pick a rule set, then start ESP32 or Tag MP."
                      : ("Tap ESP32 for solo or Tag MP to invite a contact.\nCurrent: " + String(checkersVariantName(checkersVariant)));
        }

        if (showOverlay) lv_obj_clear_flag(lvglCheckersOverlay, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(lvglCheckersOverlay, LV_OBJ_FLAG_HIDDEN);
        if (lvglCheckersOverlayTitle) lv_label_set_text(lvglCheckersOverlayTitle, title.c_str());
        if (lvglCheckersOverlaySubLabel) lv_label_set_text(lvglCheckersOverlaySubLabel, msg.c_str());
        if (lvglCheckersOverlayBtn) {
            if (showBtn) lv_obj_clear_flag(lvglCheckersOverlayBtn, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_add_flag(lvglCheckersOverlayBtn, LV_OBJ_FLAG_HIDDEN);
        }
        if (lvglCheckersOverlayBtnLabel) lv_label_set_text(lvglCheckersOverlayBtnLabel, btnText.c_str());
    }

    checkersRefreshPeerPopup();
    checkersRefreshVariantPopup();
}

void lvglCheckersEsp32Event(lv_event_t *e)
{
    (void)e;
    if (checkersVariantPopupOpen) return;
    checkersStartEsp32Game();
    lvglRefreshCheckersBoard();
}

void lvglCheckersTagEvent(lv_event_t *e)
{
    (void)e;
    if (checkersVariantPopupOpen) return;
    if (checkersPeerPopupOpen) checkersClosePeerPopup();
    else checkersOpenPeerPopup();
    lvglRefreshCheckersBoard();
}

void lvglCheckersBackEvent(lv_event_t *e)
{
    (void)e;
    checkersClosePeerPopup();
    lvglOpenScreen(UI_GAMES, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
}

void lvglCheckersReplayEvent(lv_event_t *e)
{
    (void)e;
    if (checkersMode == CHECKERS_MODE_ESP32) {
        checkersStartEsp32Game();
    } else if (checkersMode == CHECKERS_MODE_TAG && !checkersPeerKey.isEmpty()) {
        checkersStartPendingInvite(checkersPeerKey);
    } else {
        checkersStarted = false;
        checkersGameOver = false;
    }
    lvglRefreshCheckersBoard();
}

void lvglCheckersPeerSelectEvent(lv_event_t *e)
{
    const int idx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (idx < 0 || idx >= p2pPeerCount || !p2pPeers[idx].enabled) return;
    checkersStartPendingInvite(p2pPeers[idx].pubKeyHex);
    lvglRefreshCheckersBoard();
}

void lvglCheckersVariantSelectEvent(lv_event_t *e)
{
    const int idx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (idx < static_cast<int>(CHECKERS_VARIANT_AMERICAN) || idx > static_cast<int>(CHECKERS_VARIANT_CANADIAN)) return;
    checkersVariant = static_cast<CheckersVariant>(idx);
    checkersVariantPopupOpen = false;
    checkersPeerPopupOpen = false;
    checkersStarted = false;
    checkersGameOver = false;
    checkersWinnerSide = 0;
    checkersMode = CHECKERS_MODE_IDLE;
    checkersClearSelection();
    checkersClearSession();
    lvglRefreshCheckersBoard();
}

void lvglCheckersInvitePlayEvent(lv_event_t *e)
{
    const int idx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (idx < 0 || idx >= chatMessageCount) return;
    String sessionId;
    CheckersVariant inviteVariant = CHECKERS_VARIANT_AMERICAN;
    if (!checkersParseInviteText(chatMessages[idx].text, sessionId, &inviteVariant)) return;
    const String peerKey = currentChatPeerKey;
    if (peerKey.isEmpty()) return;
    lvglEnsureScreenBuilt(UI_GAME_CHECKERS);
    checkersStartAcceptedInvite(peerKey, sessionId, inviteVariant);
    if (screensaverActive) screensaverSetActive(false);
    if (!displayAwake) displaySetAwake(true);
    lastUserActivityMs = millis();
    lvglRefreshCheckersBoard();
    lvglOpenScreen(UI_GAME_CHECKERS, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglOpenCheckersEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_GAME_CHECKERS);
    checkersMode = CHECKERS_MODE_IDLE;
    checkersPeerPopupOpen = false;
    checkersVariantPopupOpen = true;
    checkersStarted = false;
    checkersGameOver = false;
    checkersWinnerSide = 0;
    checkersClearSession();
    checkersClearSelection();
    if (screensaverActive) screensaverSetActive(false);
    if (!displayAwake) displaySetAwake(true);
    lastUserActivityMs = millis();
    lvglRefreshCheckersBoard();
    lvglOpenScreen(UI_GAME_CHECKERS, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

static bool checkersChooseAiMove(CheckersMove &bestMove)
{
    const int boardSize = checkersBoardSize();
    CheckersMove legal[CHECKERS_MAX_MOVES];
    const int legalCount = checkersCollectLegalMovesForTurn(legal, CHECKERS_MAX_MOVES);
    if (legalCount <= 0) return false;

    int bestScore = -100000;
    int bestIdx = 0;
    for (int i = 0; i < legalCount; ++i) {
        int score = legal[i].capture ? 100 : 10;
        const int8_t piece = checkersBoard[legal[i].fromY][legal[i].fromX];
        if (piece == -1 && legal[i].toY == (boardSize - 1)) score += 25;
        if (checkersIsKing(piece)) score += 12;
        score += static_cast<int>(esp_random() & 0x0F);
        if (score > bestScore) {
            bestScore = score;
            bestIdx = i;
        }
    }
    bestMove = legal[bestIdx];
    return true;
}

void checkersTick()
{
    if (uiScreen != UI_GAME_CHECKERS) return;
    if (checkersMode != CHECKERS_MODE_ESP32 || !checkersStarted || checkersGameOver || checkersWaitingForRemote) return;
    if (checkersTurn != -1) return;
    if (millis() < checkersAiDueMs) return;

    CheckersMove aiMove = {};
    if (!checkersChooseAiMove(aiMove)) {
        checkersFinishGame(1);
        lvglRefreshCheckersBoard();
        return;
    }

    checkersApplyMove(aiMove, false);
    if (!checkersGameOver && checkersTurn == -1) checkersAiDueMs = millis() + 260UL;
    lvglRefreshCheckersBoard();
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
    if (percent == 0) return 0;
    const float normalized = static_cast<float>(percent) / 100.0f;
    const float curved = powf(normalized, AUDIO_VOLUME_CURVE_EXPONENT);
    const long mapped = lroundf(curved * static_cast<float>(AUDIO_VOLUME_TARGET));
    return static_cast<uint8_t>(constrain(mapped, 1L, static_cast<long>(AUDIO_VOLUME_TARGET)));
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
    if (!RGB_OUTPUT_SUPPORTED) return;
    auto level = [](bool on) -> uint8_t {
        const uint8_t scaled = on ? static_cast<uint8_t>((static_cast<uint16_t>(255U) * rgbLedPercent) / 100U) : 0U;
        return RGB_ACTIVE_LOW ? static_cast<uint8_t>(255U - scaled) : scaled;
    };
    // Board LED channels are physically swapped: logical R<->G.
    analogWrite(RGB_PIN_R, level(gOn));
    analogWrite(RGB_PIN_G, level(rOn));
    analogWrite(RGB_PIN_B, level(bOn));
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
    if (!RGB_OUTPUT_SUPPORTED) return;
    auto out = [](uint8_t val) -> uint8_t {
        const uint8_t scaled = static_cast<uint8_t>((static_cast<uint16_t>(val) * rgbLedPercent) / 100U);
        return RGB_ACTIVE_LOW ? static_cast<uint8_t>(255U - scaled) : scaled;
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

static inline bool chatMessageBeepCanPlay()
{
    return !mediaIsPlaying && !mediaPaused;
}

static void chatMessageBeepStop(bool clearQueue)
{
    ledcWriteTone(CHAT_NOTIFY_LEDC_CHANNEL, 0);
    ledcWrite(CHAT_NOTIFY_LEDC_CHANNEL, 0);
    if (chatMessageBeepPinAttached) {
        ledcDetachPin(I2S_SPK_PIN);
        pinMode(I2S_SPK_PIN, OUTPUT);
        digitalWrite(I2S_SPK_PIN, LOW);
        chatMessageBeepPinAttached = false;
    }
    chatMessageBeepPhase = 0;
    chatMessageBeepDeadlineMs = 0;
    if (clearQueue) chatMessageBeepQueue = 0;
}

static bool chatMessageBeepEnsurePinAttached()
{
    if (chatMessageBeepPinAttached) return true;
    ledcSetup(CHAT_NOTIFY_LEDC_CHANNEL, CHAT_NOTIFY_FREQ_PRIMARY, 10);
    ledcAttachPin(I2S_SPK_PIN, CHAT_NOTIFY_LEDC_CHANNEL);
    chatMessageBeepPinAttached = true;
    return true;
}

static inline uint32_t chatMessageBeepDuty()
{
    if (mediaVolumePercent == 0) return 0;
    const float normalized = static_cast<float>(mediaVolumePercent) / 100.0f;
    const float curved = powf(normalized, AUDIO_VOLUME_CURVE_EXPONENT);
    const long mapped = lroundf(curved * 1023.0f);
    return static_cast<uint32_t>(constrain(mapped, 1L, 1023L));
}

static void chatMessageBeepStartStep(uint8_t stepIndex)
{
    const ChatBeepPattern pattern = chatMessageBeepPattern(messageBeepTone);
    if (stepIndex >= pattern.count || pattern.count == 0) {
        if (chatMessageBeepQueue > 0) chatMessageBeepQueue--;
        chatMessageBeepStop(false);
        chatMessageBeepDeadlineMs = millis() + CHAT_NOTIFY_REPEAT_GAP_MS;
        return;
    }
    if (!chatMessageBeepEnsurePinAttached()) return;
    const ChatBeepStep &step = pattern.steps[stepIndex];
    if (step.freq == 0) {
        ledcWriteTone(CHAT_NOTIFY_LEDC_CHANNEL, 0);
        ledcWrite(CHAT_NOTIFY_LEDC_CHANNEL, 0);
    } else {
        ledcWriteTone(CHAT_NOTIFY_LEDC_CHANNEL, step.freq);
        ledcWrite(CHAT_NOTIFY_LEDC_CHANNEL, chatMessageBeepDuty());
    }
    chatMessageBeepPhase = static_cast<uint8_t>(stepIndex + 1U);
    chatMessageBeepDeadlineMs = millis() + step.durationMs;
}

static void chatMessageBeepPreview()
{
    if (!chatMessageBeepCanPlay()) return;
    chatMessageBeepStop(true);
    chatMessageBeepQueue = 1;
    chatMessageBeepDeadlineMs = 0;
    chatMessageBeepStartStep(0);
}

void chatQueueIncomingMessageBeep()
{
    if (!chatMessageBeepCanPlay()) return;
    if (chatMessageBeepQueue < CHAT_NOTIFY_QUEUE_MAX) chatMessageBeepQueue++;
}

void chatQueueIncomingMessageVibration()
{
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    if (!vibrationEnabled) return;
    if (chatMessageVibrationQueue < VIBRATION_QUEUE_MAX) chatMessageVibrationQueue++;
#endif
}

#if defined(BOARD_ESP32S3_3248S035_N16R8)
static uint32_t chatMessageVibrationDuty()
{
    if (!vibrationEnabled) return 0;
    switch (vibrationIntensity) {
        case VIBRATION_INTENSITY_LOW: return 96;
        case VIBRATION_INTENSITY_MEDIUM: return 168;
        case VIBRATION_INTENSITY_HIGH:
        default: return 255;
    }
}

static bool chatMessageVibrationEnsurePinAttached()
{
    if (chatMessageVibrationPinAttached) return true;
    ledcSetup(VIBRATION_LEDC_CHANNEL, VIBRATION_PWM_HZ, VIBRATION_PWM_RES_BITS);
    ledcAttachPin(VIBRATION_MOTOR_PIN, VIBRATION_LEDC_CHANNEL);
    chatMessageVibrationPinAttached = true;
    return true;
}

static void chatMessageVibrationStop(bool clearQueue)
{
    ledcWrite(VIBRATION_LEDC_CHANNEL, 0);
    if (chatMessageVibrationPinAttached) {
        ledcDetachPin(VIBRATION_MOTOR_PIN);
        pinMode(VIBRATION_MOTOR_PIN, OUTPUT);
        digitalWrite(VIBRATION_MOTOR_PIN, LOW);
        chatMessageVibrationPinAttached = false;
    }
    chatMessageVibrationPinOutput = false;
    chatMessageVibrationPhase = 0;
    chatMessageVibrationDeadlineMs = 0;
    if (clearQueue) chatMessageVibrationQueue = 0;
}

static void chatMessageVibrationStartOutput()
{
    if (!chatMessageVibrationEnsurePinAttached()) return;
    ledcWrite(VIBRATION_LEDC_CHANNEL, chatMessageVibrationDuty());
    chatMessageVibrationPinOutput = true;
}

static void chatMessageVibrationPreview()
{
    if (!vibrationEnabled) {
        chatMessageVibrationStop(true);
        return;
    }
    chatMessageVibrationStop(true);
    chatMessageVibrationQueue = 1;
    chatMessageVibrationDeadlineMs = 0;
    chatMessageVibrationService();
}
#endif

void chatMessageBeepService()
{
    if (!chatMessageBeepCanPlay()) {
        if (chatMessageBeepPhase != 0 || chatMessageBeepPinAttached) chatMessageBeepStop(true);
        return;
    }

    const unsigned long now = millis();
    if (chatMessageBeepPhase == 0) {
        if (chatMessageBeepQueue == 0) return;
        if (chatMessageBeepDeadlineMs != 0 && static_cast<long>(now - chatMessageBeepDeadlineMs) < 0) return;
        chatMessageBeepStartStep(0);
        return;
    }

    if (static_cast<long>(now - chatMessageBeepDeadlineMs) < 0) return;
    chatMessageBeepStartStep(chatMessageBeepPhase);
}

void chatMessageVibrationService()
{
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    const unsigned long now = millis();
    if (chatMessageVibrationPhase == 0) {
        if (chatMessageVibrationQueue == 0) return;
        if (chatMessageVibrationDeadlineMs != 0 && static_cast<long>(now - chatMessageVibrationDeadlineMs) < 0) return;
        chatMessageVibrationStartOutput();
        chatMessageVibrationPhase = 1;
        chatMessageVibrationDeadlineMs = now + VIBRATION_PULSE_MS;
        return;
    }

    if (static_cast<long>(now - chatMessageVibrationDeadlineMs) < 0) return;

    switch (chatMessageVibrationPhase) {
        case 1:
            ledcWrite(VIBRATION_LEDC_CHANNEL, 0);
            chatMessageVibrationPhase = 2;
            chatMessageVibrationDeadlineMs = now + VIBRATION_GAP_MS;
            break;
        case 2:
            ledcWrite(VIBRATION_LEDC_CHANNEL, chatMessageVibrationDuty());
            chatMessageVibrationPhase = 3;
            chatMessageVibrationDeadlineMs = now + VIBRATION_PULSE_MS;
            break;
        case 3:
            if (chatMessageVibrationQueue > 0) chatMessageVibrationQueue--;
            chatMessageVibrationStop(false);
            break;
        default:
            chatMessageVibrationStop(true);
            break;
    }
#endif
}

bool audioEnsureBackendReady(const char *reason)
{
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    Serial.printf("[AUDIO] backend unavailable on this build: GPIO %d mapped but internal DAC mode is ESP32-only reason=%s\n",
                  I2S_SPK_PIN,
                  (reason && reason[0]) ? reason : "-");
    return false;
#else
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
#endif
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
    uint8_t *buf = reinterpret_cast<uint8_t *>(allocPreferPsram(1024));
    if (!buf) return false;
    for (int attempt = 0; attempt < 2; attempt++) {
        if (!sdEnsureMounted(attempt > 0)) {
            free(buf);
            return false;
        }
        if (!sdLock()) {
            free(buf);
            return false;
        }

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
            while (src.available()) {
                int n = src.read(buf, 1024);
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

        if (ok) {
            free(buf);
            return true;
        }
        sdMarkFault("copyFileToPath");
        delay(15);
    }
    free(buf);
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

static void mediaEnsureStorageReadyForUi()
{
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    if (!sdMounted) {
        setupSd();
    } else {
        sdEnsureMounted();
    }
#else
    sdEnsureMounted();
#endif
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
            lvglNextHandlerDueMs = 0;
            lv_timer_handler();
            lvglRefreshInfoPanel(false);
            lvglRefreshTopIndicators();
        }
    } else {
        wakeTouchReleaseGuard = false;
        if (!batteryCharging) rgbApplyNow(false, false, false);
    }
}

static void wakeDisplayForIncomingNotification()
{
    if (displayAwake && !screensaverActive) return;
    if (screensaverActive) screensaverSetActive(false);
    if (!displayAwake) displaySetAwake(true);
    lastUserActivityMs = millis();
}

bool canEnterLowPowerSleep(bool touchDownNow)
{
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    (void)touchDownNow;
    return false;
#else
    if (touchDownNow) return false;
    if (uiScreenKeepsDisplayAwake()) return false;
    if (wifiConnectedSafe()) return false; // requested: sleep only when not connected
    if (batteryCharging) return false;
    if (batteryPercent < 5) return false;
    if (fsWriteBusy()) return false;
    if (bootStaConnectInProgress) return false;
    if (mediaIsPlaying || mediaPaused) return false;
    if (displayAwake) return false;
    if (millis() - lastUserActivityMs < LIGHT_SLEEP_AFTER_IDLE_MS) return false;
    return true;
#endif
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

static bool normalizeSavedApCreds()
{
    bool changed = false;
    savedApSsid.trim();
    savedApPass.trim();
    if (savedApSsid.isEmpty()) {
        savedApSsid = AP_SSID;
        changed = true;
    }
    if (savedApPass.isEmpty() || savedApPass.length() < 8) {
        savedApPass = AP_PASS;
        changed = true;
    }
    return changed;
}

void loadSavedStaCreds()
{
    wifiPrefs.begin("wifi", true);
    savedStaSsid = wifiPrefs.getString("sta_ssid", "");
    savedStaPass = wifiPrefs.getString("sta_pass", "");
    savedApSsid = wifiPrefs.getString("ap_ssid", AP_SSID);
    savedApPass = wifiPrefs.getString("ap_pass", AP_PASS);
    wifiPrefs.end();
    if (normalizeSavedApCreds()) {
        wifiPrefs.begin("wifi", false);
        wifiPrefs.putString("ap_ssid", savedApSsid);
        wifiPrefs.putString("ap_pass", savedApPass);
        wifiPrefs.end();
    }
}

bool loadBatterySnapshot(BatterySnapshot &snapshotOut)
{
    batteryPrefs.begin("battery", true);
    bool valid = batteryPrefs.getBool("valid", false);
    float rawV = batteryPrefs.getFloat("raw_v", 0.0f);
    if (rawV <= 0.0f) rawV = batteryPrefs.getFloat("vbat", 0.0f);
    bool charging = batteryPrefs.getBool("charging", false);
    uint32_t uptimeMs = batteryPrefs.getULong("uptime_ms", 0UL);
    batteryPrefs.end();
    if (!valid || rawV <= 0.0f) return false;
    snapshotOut.valid = true;
    snapshotOut.rawV = rawV;
    snapshotOut.charging = charging;
    snapshotOut.uptimeMs = uptimeMs;
    return true;
}

void saveBatterySnapshot(float rawV, bool charging, uint32_t uptimeMs)
{
    batteryPrefs.begin("battery", false);
    batteryPrefs.putBool("valid", true);
    batteryPrefs.putFloat("raw_v", rawV);
    batteryPrefs.putFloat("vbat", batteryCalibratedVoltageFromRaw(rawV));
    batteryPrefs.putBool("charging", charging);
    batteryPrefs.putULong("uptime_ms", uptimeMs);
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

void clearStaCreds()
{
    wifiPrefs.begin("wifi", false);
    wifiPrefs.remove("sta_ssid");
    wifiPrefs.remove("sta_pass");
    wifiPrefs.end();
    savedStaSsid = "";
    savedStaPass = "";
}

void loadGamePrefs()
{
    gamePrefs.begin("games", true);
    snakeHighScore = static_cast<uint16_t>(gamePrefs.getUInt("snake_best", 0));
    snakeLastScore = static_cast<uint16_t>(gamePrefs.getUInt("snake_last", 0));
    tetrisHighScore = static_cast<uint16_t>(gamePrefs.getUInt("tetris_best", 0));
    tetrisLastScore = static_cast<uint16_t>(gamePrefs.getUInt("tetris_last", 0));
    checkersLocalWins = static_cast<uint16_t>(gamePrefs.getUInt("checkers_local_wins", 0));
    checkersRemoteWins = static_cast<uint16_t>(gamePrefs.getUInt("checkers_remote_wins", 0));
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    snake3dHighScore = static_cast<uint16_t>(gamePrefs.getUInt("snake3d_best", 0));
    snake3dLastScore = static_cast<uint16_t>(gamePrefs.getUInt("snake3d_last", 0));
#endif
    gamePrefs.end();
}

static void saveGameHighScore(const char *key, uint16_t score)
{
    if (!key) return;
    gamePrefs.begin("games", false);
    gamePrefs.putUInt(key, score);
    gamePrefs.end();
}

static void saveGameValue(const char *key, uint16_t value)
{
    if (!key) return;
    gamePrefs.begin("games", false);
    gamePrefs.putUInt(key, value);
    gamePrefs.end();
}

static void snakeMaybeStoreHighScore(bool persist)
{
    if (snakeScore <= snakeHighScore) return;
    snakeHighScore = snakeScore;
    if (persist) saveGameHighScore("snake_best", snakeHighScore);
}

static void tetrisMaybeStoreHighScore(bool persist)
{
    if (tetrisScore <= tetrisHighScore) return;
    tetrisHighScore = tetrisScore;
    if (persist) saveGameHighScore("tetris_best", tetrisHighScore);
}

#if defined(BOARD_ESP32S3_3248S035_N16R8)
static void snake3dMaybeStoreHighScore(bool persist)
{
    if (snake3dScore <= snake3dHighScore) return;
    snake3dHighScore = snake3dScore;
    if (persist) saveGameHighScore("snake3d_best", snake3dHighScore);
}
#endif

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

void appendUiSettings(JsonDocument &doc)
{
    uiPrefs.begin("ui", true);

    doc["RecordTelemetry"] = uiPrefs.getBool("record_tel", recordTelemetryEnabled) ? 1 : 0;
    doc["SystemSounds"] = uiPrefs.getBool("sys_snd", systemSoundsEnabled) ? 1 : 0;
    doc["SystemVolume"] = uiPrefs.getUChar("sys_vol", mediaVolumePercent);
    doc["VibrationEnabled"] = uiPrefs.getBool("vib_en", vibrationEnabled) ? 1 : 0;
    doc["DisplayBrightness"] = uiPrefs.getUChar("disp_bri", displayBrightnessPercent);
    doc["DeviceName"] = uiPrefs.getString("dev_name", deviceShortNameValue());
    doc["WsRebootOnDisconnect"] = uiPrefs.getBool("ws_reboot", wsRebootOnDisconnectEnabled) ? 1 : 0;
    doc["AirplaneMode"] = uiPrefs.getBool("airplane", airplaneModeEnabled) ? 1 : 0;
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
    menuCustomIconsEnabled = uiPrefs.getBool("menu_3d_i", false);
    uiLanguage = static_cast<UiLanguage>(constrain(uiPrefs.getUChar("lang", static_cast<uint8_t>(UI_LANG_ENGLISH)),
                                                   static_cast<uint8_t>(UI_LANG_ENGLISH),
                                                   static_cast<uint8_t>(UI_LANG_COUNT - 1)));
    vibrationIntensity = static_cast<VibrationIntensity>(constrain(uiPrefs.getUChar("vib_int", static_cast<uint8_t>(VIBRATION_INTENSITY_MEDIUM)),
                                                                   static_cast<uint8_t>(VIBRATION_INTENSITY_LOW),
                                                                   static_cast<uint8_t>(VIBRATION_INTENSITY_HIGH)));
    vibrationEnabled = uiPrefs.getBool("vib_en", true);
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
        wakeDisplayForIncomingNotification();
        if (checkersHandleIncomingChatPayload(senderPubHex, author, text, CHAT_TRANSPORT_MQTT, messageId)) {
            if (lvglReady) {
                lvglSyncStatusLine();
                if (uiScreen == UI_CHAT) lvglRefreshChatUi();
            }
        } else if (messageId.isEmpty() || !chatHasLoggedMessageId(senderPubHex, messageId)) {
            chatStoreMessage(senderPubHex, author, text, false, CHAT_TRANSPORT_MQTT, messageId);
            chatQueueIncomingMessageBeep();
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

static bool otaFetchLatestReleaseInfo(String &tagNameOut, String &binUrlOut, String &errorOut)
{
    tagNameOut = "";
    binUrlOut = "";
    errorOut = "";

    int ghStatus = -1;
    const String body = httpsGetText("https://api.github.com/repos/elik745i/ESP32-2432S024C-Remote/releases/latest", &ghStatus);
    if (body.isEmpty()) {
        errorOut = "github_fetch_failed";
        return false;
    }

    JsonDocument filter;
    filter["tag_name"] = true;
    filter["assets"][0]["name"] = true;
    filter["assets"][0]["browser_download_url"] = true;

    JsonDocument parsed;
    const DeserializationError err = deserializeJson(parsed, body, DeserializationOption::Filter(filter));
    if (err) {
        errorOut = "json_parse_failed";
        return false;
    }

    tagNameOut = String(static_cast<const char *>(parsed["tag_name"] | ""));
    binUrlOut = chooseLatestFirmwareBinUrl(parsed["assets"]);
    if (tagNameOut.isEmpty() || binUrlOut.isEmpty()) {
        errorOut = "release_asset_not_found";
        return false;
    }
    return true;
}

static void otaSetStatus(const String &text)
{
    otaStatusText = text;
}

static void otaFinalizePendingBootImage()
{
#if defined(ESP_ARDUINO_VERSION_MAJOR)
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (!running) return;

    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    if (esp_ota_get_state_partition(running, &state) != ESP_OK) return;

    if (state == ESP_OTA_IMG_PENDING_VERIFY) {
        const esp_err_t markErr = esp_ota_mark_app_valid_cancel_rollback();
        if (markErr == ESP_OK) {
            Serial.printf("[OTA] marked running image valid: %s @ 0x%08lx\n",
                          running->label,
                          static_cast<unsigned long>(running->address));
        } else {
            Serial.printf("[OTA] mark app valid failed: %s\n", esp_err_to_name(markErr));
        }
    }
#endif
}

bool otaDownloadAndApplyFromUrl(const String &url, String &errorOut)
{
    errorOut = "";
    if (!OTA_FIRMWARE_FLASH_SUPPORTED) {
        errorOut = "ota_partition_unsupported";
        return false;
    }
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
    http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
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

    uint8_t *buf = reinterpret_cast<uint8_t *>(allocPreferPsram(OTA_DOWNLOAD_BUF_SIZE));
    if (!buf) {
        errorOut = "oom_buffer";
        Update.abort();
        http.end();
        return false;
    }
    int remaining = totalLen;
    size_t writtenTotal = 0;
    otaUiState = OTA_UI_DOWNLOADING;
    otaProgressPercent = 0;
    otaSetStatus(totalLen > 0 ? "Downloading update..." : "Downloading update...");
    while (http.connected() && (remaining > 0 || remaining == -1)) {
        const size_t avail = static_cast<size_t>(stream->available());
        if (!avail) {
            delay(1);
            continue;
        }
        const size_t toRead = avail > OTA_DOWNLOAD_BUF_SIZE ? OTA_DOWNLOAD_BUF_SIZE : avail;
        const int n = stream->readBytes(buf, toRead);
        if (n <= 0) {
            delay(1);
            continue;
        }
        if (Update.write(buf, static_cast<size_t>(n)) != static_cast<size_t>(n)) {
            errorOut = "update_write_failed";
            Update.abort();
            http.end();
            free(buf);
            return false;
        }
        writtenTotal += static_cast<size_t>(n);
        if (remaining > 0) remaining -= n;
        if (totalLen > 0) {
            const size_t pct = min<size_t>(100U, (writtenTotal * 100U) / static_cast<size_t>(totalLen));
            otaProgressPercent = static_cast<uint8_t>(pct);
        }
        delay(0);
    }
    http.end();
    free(buf);

    otaUiState = OTA_UI_FINALIZING;
    otaProgressPercent = 100;
    otaSetStatus("Finalizing update...");
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

String apLandingContinueUrl()
{
    if (sdEnsureMounted() && hasPrimaryWebUi()) return "/?skip_ap_portal=1";
    return "/recovery";
}

void sendApWifiLandingPage(AsyncWebServerRequest *request)
{
    const String continueUrl = apLandingContinueUrl();
    const bool hasFrontend = (continueUrl == "/?skip_ap_portal=1");
    String html = FPSTR(AP_WIFI_LANDING_HTML);
    html.replace("%FIRMWARE_VERSION%", FW_VERSION);
    html.replace("%AP_SSID%", savedApSsid.length() ? savedApSsid : String(AP_SSID));
    html.replace("%AP_IP%", WiFi.softAPIP().toString());
    html.replace("%CONTINUE_URL%", continueUrl);
    html.replace("%CONTINUE_LABEL%", hasFrontend ? "the main web UI" : "the recovery browser");
    html.replace("%CONTINUE_HINT%", hasFrontend ? "Close to open the SD web frontend." : "Close to open the embedded recovery browser.");
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
    uint8_t *mcuBuf = reinterpret_cast<uint8_t *>(allocPreferPsram(mcuBytes));
    if (!mcuBuf) {
        encoder.close();
        if (SD.exists(path)) SD.remove(path);
        sdUnlock();
        errorOut = "oom_mcu";
        return false;
    }
    Serial.printf("[JPEG] MCU buffer bytes=%u psram=%d\n",
                  static_cast<unsigned int>(mcuBytes),
                  boardHasUsablePsram() && esp_ptr_external_ram(mcuBuf) ? 1 : 0);

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
        String tagName;
        String binUrl;
        String error;
        if (!otaFetchLatestReleaseInfo(tagName, binUrl, error)) {
            doc["error"] = error;
            String payload;
            serializeJson(doc, payload);
            request->send(500, "application/json", payload);
            return;
        }

        doc["ok"] = true;
        doc["tag_name"] = tagName;
        doc["bin_url"] = binUrl;
        doc["source"] = "device";
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
        if (apModeActive) {
            sendApWifiLandingPage(request);
            return;
        }
        sendWifiSetupPage(request);
    });

    server.on("/listwifi", HTTP_GET, [](AsyncWebServerRequest *request) {
        (void)request;
        JsonDocument doc;
        JsonArray arr = doc.to<JsonArray>();
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

    server.on("/savewifi", HTTP_POST, [](AsyncWebServerRequest *request) {
        String ssid;
        String pass;
        if (request->hasParam("ssid", true)) ssid = urlDecode(request->getParam("ssid", true)->value());
        else if (request->hasParam("ssid", false)) ssid = urlDecode(request->getParam("ssid", false)->value());
        if (request->hasParam("password", true)) pass = urlDecode(request->getParam("password", true)->value());
        else if (request->hasParam("password", false)) pass = urlDecode(request->getParam("password", false)->value());
        if (ssid.isEmpty()) {
            request->send(400, "text/plain", "missing_ssid");
            return;
        }
        startWifiConnect(ssid, pass);
        request->send(200, "application/json", "{\"ok\":true,\"status\":\"connecting\"}");
    });

    server.on("/list_saved_wifi", HTTP_GET, [](AsyncWebServerRequest *request) {
        (void)request;
        JsonDocument doc;
        JsonArray arr = doc.to<JsonArray>();
        const String ssid = wifiDesiredStaSsid();
        if (ssid.length()) {
            JsonObject it = arr.add<JsonObject>();
            it["ssid"] = ssid;
            it["preferred"] = true;
            it["retry"] = 3;
            it["autoReconnect"] = true;
        }
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/delete_saved_wifi", HTTP_POST, [](AsyncWebServerRequest *request) {
        String ssid;
        if (request->hasParam("ssid", true)) ssid = urlDecode(request->getParam("ssid", true)->value());
        else if (request->hasParam("ssid", false)) ssid = urlDecode(request->getParam("ssid", false)->value());

        const String current = wifiDesiredStaSsid();
        if (current.isEmpty() || (ssid.length() && !ssid.equalsIgnoreCase(current))) {
            request->send(404, "application/json", "{\"ok\":false,\"error\":\"not_found\"}");
            return;
        }

        pendingSaveCreds = false;
        pendingSaveSsid = "";
        pendingSavePass = "";
        clearStaCreds();
        wifiForgetPendingUi = false;
        WiFi.disconnect(false, false);
        ensureApOnline("web_delete_saved_wifi");
        request->send(200, "application/json", "{\"ok\":true,\"removed\":true}");
    });

    server.on("/connect_saved_wifi", HTTP_ANY, [](AsyncWebServerRequest *request) {
        String ssid;
        if (request->hasParam("ssid", true)) ssid = urlDecode(request->getParam("ssid", true)->value());
        else if (request->hasParam("ssid", false)) ssid = urlDecode(request->getParam("ssid", false)->value());
        if (ssid.isEmpty()) ssid = wifiDesiredStaSsid();
        if (ssid.isEmpty() || !ssid.equalsIgnoreCase(wifiDesiredStaSsid())) {
            request->send(404, "text/plain", "saved_wifi_not_found");
            return;
        }
        startWifiConnect(wifiDesiredStaSsid(), wifiDesiredStaPass());
        request->send(200, "text/plain", "connecting");
    });

    server.on("/wifi_try_connect", HTTP_POST, [](AsyncWebServerRequest *request) {
        String ssid;
        if (request->hasParam("ssid", true)) ssid = urlDecode(request->getParam("ssid", true)->value());
        else if (request->hasParam("ssid", false)) ssid = urlDecode(request->getParam("ssid", false)->value());
        if (ssid.isEmpty() || !ssid.equalsIgnoreCase(wifiDesiredStaSsid())) {
            request->send(404, "text/plain", "saved_wifi_not_found");
            return;
        }
        startWifiConnect(wifiDesiredStaSsid(), wifiDesiredStaPass());
        request->send(200, "text/plain", "connecting");
    });

    server.on("/update_wifi_password", HTTP_POST, [](AsyncWebServerRequest *request) {
        String ssid;
        String pass;
        if (request->hasParam("ssid", true)) ssid = urlDecode(request->getParam("ssid", true)->value());
        else if (request->hasParam("ssid", false)) ssid = urlDecode(request->getParam("ssid", false)->value());
        if (request->hasParam("password", true)) pass = urlDecode(request->getParam("password", true)->value());
        else if (request->hasParam("password", false)) pass = urlDecode(request->getParam("password", false)->value());
        const String current = wifiDesiredStaSsid();
        if (current.isEmpty() || !ssid.equalsIgnoreCase(current)) {
            request->send(404, "text/plain", "saved_wifi_not_found");
            return;
        }
        saveStaCreds(current, pass);
        pendingSaveCreds = false;
        pendingSaveSsid = "";
        pendingSavePass = "";
        request->send(200, "text/plain", "ok");
    });

    server.on("/wifi_set_autoreconnect", HTTP_POST, [](AsyncWebServerRequest *request) {
        (void)request;
        request->send(200, "text/plain", "ok");
    });

    server.on("/update_retry_count", HTTP_GET, [](AsyncWebServerRequest *request) {
        (void)request;
        request->send(200, "text/plain", "ok");
    });

    server.on("/wifi_set_priority", HTTP_POST, [](AsyncWebServerRequest *request) {
        String ssid;
        if (request->hasParam("ssid", true)) ssid = urlDecode(request->getParam("ssid", true)->value());
        else if (request->hasParam("ssid", false)) ssid = urlDecode(request->getParam("ssid", false)->value());
        JsonDocument doc;
        doc["ok"] = ssid.length() && ssid.equalsIgnoreCase(wifiDesiredStaSsid());
        String payload;
        serializeJson(doc, payload);
        request->send(doc["ok"] ? 200 : 404, "application/json", payload);
    });

    server.on("/set_ap_password", HTTP_POST, [](AsyncWebServerRequest *request) {
        String pass;
        if (request->hasParam("ap_pass", true)) pass = urlDecode(request->getParam("ap_pass", true)->value());
        else if (request->hasParam("ap_pass", false)) pass = urlDecode(request->getParam("ap_pass", false)->value());
        pass.trim();

        JsonDocument doc;
        if (pass.isEmpty() || pass.length() < 8 || pass.length() > 63) {
            doc["ok"] = false;
            doc["err"] = "len";
            String payload;
            serializeJson(doc, payload);
            request->send(400, "application/json", payload);
            return;
        }

        saveApCreds(savedApSsid.length() ? savedApSsid : String(AP_SSID), pass);
        doc["ok"] = true;
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/get_ap_password", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        doc["ok"] = true;
        doc["open"] = savedApPass.isEmpty();
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
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
        JsonArray radioArr = doc["radio_discovered"].to<JsonArray>();
        for (int i = 0; i < hc12DiscoveredCount; ++i) {
            JsonObject peer = radioArr.add<JsonObject>();
            peer["name"] = hc12DiscoveredPeers[i].name;
            peer["public_key"] = hc12DiscoveredPeers[i].pubKeyHex;
            peer["last_seen_ms"] = static_cast<uint64_t>(hc12DiscoveredPeers[i].lastSeenMs);
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

        const bool ok = chatSendAndStoreMessage(currentChatPeerKey, text);
        doc["ok"] = ok;
        doc["transport"] = "auto";
        String payload;
        serializeJson(doc, payload);
        request->send(ok ? 200 : 409, "application/json", payload);
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
        const bool bypassApPortal = request->hasParam("skip_ap_portal");
        if (apModeActive && !bypassApPortal) {
            sendApWifiLandingPage(request);
            return;
        }
        if (sdEnsureMounted() && hasPrimaryWebUi()) {
            String idx = webIndexPath();
            if (idx.length() && sendMaybeGz(request, idx)) return;
            // If core UI is expected but index still failed, keep behavior deterministic.
            request->send(500, "text/plain", "Web UI gate passed but index missing");
            return;
        }
        // Connected: go to recovery file manager when SD UI is missing.
        if (staConnected || (apModeActive && bypassApPortal)) {
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
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    if (!wifiRuntimeManaged) return;
    if (!bootWifiInitPending && WiFi.getMode() == WIFI_OFF) return;
#endif

    if (wifiStaGotIpPending) {
        wifiStaGotIpPending = false;
        bootStaConnectInProgress = false;
        const String ssid = wifiSsidSafe();
        const String ip = wifiIpSafe();
        clearWifiScanResults();
        wifiForgetPendingUi = false;
        disableApWhenStaConnected("sta_got_ip");
        if (pendingSaveCreds && ssid == pendingSaveSsid) {
            saveStaCreds(pendingSaveSsid, pendingSavePass);
            pendingSaveCreds = false;
            pendingSaveSsid = "";
            pendingSavePass = "";
        }
        uiStatusLine = "Connected: " + ssid;
        syncInternetTimeIfNeeded(true);
        lvglTopIndicatorStateValid = false;
        if (lvglReady) lvglRefreshTopIndicators();
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
        wifiForgetPendingUi = false;
        ensureApOnline("sta_disconnected");
        uiStatusLine = wifiSessionApMode ? "AP mode enabled" : "WiFi disconnected";
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
    if (wifiSessionApMode) {
        if (!apModeActive) ensureApOnline("session_ap_mode");
        return;
    }
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
    if (airplaneModeEnabled || networkSuspendedForAudio || !webServerEnabled) {
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
    stopWebServerRuntime();

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
        wifiRuntimeManaged = false;
        WiFi.mode(WIFI_OFF);
        apModeActive = false;
        stopWebServerRuntime();
        return;
    }
    wifiRuntimeManaged = true;
    registerWifiEvents();
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    WiFi.setSleep(false);
    WiFi.mode(WIFI_STA);
    apModeActive = false;
    wifiSessionApMode = false;
    stopDnsForAp();
    ensureWebServerRuntime();

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
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step sodium_init");
    p2pReady = sodium_init() >= 0;
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step loadP2pConfig");
    if (p2pReady) loadP2pConfig();
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step loadUiRuntimeConfig");
    loadUiRuntimeConfig();
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
    bootSdInitPending = true;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] SD boot init enabled on ESP32-S3 after RGB/PSRAM fix");
#endif
    bootWifiInitPending = true;
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
    pinMode(I2S_SPK_PIN, OUTPUT);
    digitalWrite(I2S_SPK_PIN, LOW);
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

void loop()
{
    const uint32_t loopStartUs = micros();
    cpuLoadService(loopStartUs);
    lvglService();
    screensaverService();
    bool isDown = lvglReady ? lvglTouchDown : false;
    const bool uiPriorityActive = uiPerformancePriorityActive(isDown);
    lvglWarmupScreensService(uiPriorityActive);
    const bool realtimeMessaging = uiScreenNeedsRealtimeMessaging();

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

    static unsigned long lastServiceSliceMs = 0;
    static uint8_t serviceSlicePhase = 0;
    const unsigned long bgSliceIntervalMs = uiPriorityActive ? (DNS_SERVICE_INTERVAL_MS * 3UL) : DNS_SERVICE_INTERVAL_MS;
    if (static_cast<unsigned long>(now - lastServiceSliceMs) >= bgSliceIntervalMs) {
        lastServiceSliceMs = now;
        serviceSlicePhase = static_cast<uint8_t>((serviceSlicePhase + 1U) % 5U);
        if (dnsRunning) dnsServer.processNextRequest();
        rgbService();
        switch (serviceSlicePhase) {
            case 0:
                wifiConnectionService();
                if (!uiPriorityActive || realtimeMessaging) mqttService();
                break;
            case 1:
                if (p2pReady && (!uiPriorityActive || realtimeMessaging)) p2pService();
                break;
            case 2:
                if (!uiPriorityActive || uiScreen == UI_WIFI_LIST) wifiScanService();
                if (!uiPriorityActive || realtimeMessaging) chatPendingService();
                break;
            case 3:
                if (!uiPriorityActive || uiScreen == UI_INFO || !sdMounted) sdStatsService();
                if (!uiPriorityActive) serviceCarInputTelemetry();
                break;
            case 4:
                if (!uiPriorityActive) refreshMdnsState();
                if (!uiPriorityActive || uiScreen == UI_CONFIG_OTA) otaCheckService();
                break;
        }
    }
    otaUpdateService();
    if (!uiPriorityActive || realtimeMessaging || Serial1.available() > 0 ||
        (uiDeferredFlags & UI_DEFERRED_HC12_SETTLE_PENDING) != 0) {
        hc12Service();
    }
    screenshotService(isDown);
    const bool allowSdAutoRetry = (uiScreen == UI_MEDIA) || !displayAwake;
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
    audioService();
    chatMessageBeepService();
    chatMessageVibrationService();
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

    if (topBarCenterMode == TOP_BAR_CENTER_TIME) syncInternetTimeIfNeeded(false);

    snakeTick();
    tetrisTick();
    tetrisAnimationService();
    checkersTick();
    snake3dTick();

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
    delay(0);
}
