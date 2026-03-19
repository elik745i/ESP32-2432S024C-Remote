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

static constexpr unsigned long RADIO_ACK_TIMEOUT_MS = 1500UL;
static constexpr uint8_t RADIO_MAX_RETRIES = 4;
static constexpr unsigned long RADIO_POST_TX_GUARD_MS = 25UL;
static constexpr unsigned long RADIO_TX_TIMING_SLOP_MS = 4UL;
static constexpr int CHAT_RECENT_MESSAGE_ID_CACHE = 32;

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
static constexpr uint16_t CHAT_NOTIFY_FREQ_SECONDARY = 1320;
static constexpr uint16_t TOUCH_TICK_FREQ = 3200;
static constexpr uint32_t TOUCH_TICK_DUTY = 64U;
static constexpr unsigned long TOUCH_TICK_MS = 4UL;
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
static constexpr float BATTERY_ADC_FALLBACK_REF_V = 3.30f;
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
static constexpr const char *FW_VERSION = "0.21.05";
static constexpr bool VERBOSE_SERIAL_DEBUG = false;
static constexpr unsigned long OTA_CHECK_INTERVAL_MS = 6UL * 60UL * 60UL * 1000UL;
static constexpr unsigned long OTA_INITIAL_CHECK_DELAY_MS = 5000UL;
static constexpr unsigned long OTA_RETRY_DELAY_MS = 10UL * 60UL * 1000UL;
static constexpr size_t OTA_VERSION_TEXT_MAX = 32;
static constexpr size_t OTA_DOWNLOAD_BUF_SIZE = 2048U;
static constexpr uint8_t OTA_RELEASE_LIST_MAX = 6U;
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
static constexpr size_t HC12_RADIO_MAX_LINE = 1216;   // payload only
static constexpr size_t HC12_RADIO_MAX_RX_LINE = HC12_RADIO_MAX_LINE + 16; // prefix + slack
static constexpr int MAX_HC12_DISCOVERED = 8;
static constexpr int MAX_MQTT_DISCOVERED = 8;
static constexpr unsigned long HC12_DISCOVERY_INTERVAL_MS = 8000UL;
static constexpr unsigned long HC12_DISCOVERY_STALE_MS = 45000UL;
static constexpr uint8_t UI_DEFERRED_SCREENSHOT_PENDING = 0x01;
static constexpr uint8_t UI_DEFERRED_SCREENSHOT_BUSY = 0x02;
static constexpr uint8_t UI_DEFERRED_HC12_SETTLE_PENDING = 0x04;
static constexpr uint8_t UI_DEFERRED_HC12_TARGET_ASSERTED = 0x08;

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

struct OtaReleaseEntry {
    String tag;
    String binUrl;
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
static bool otaFetchReleaseCatalog(String &latestTagOut, String &latestUrlOut, String &errorOut);
static void otaSetStatus(const String &text);
static void otaClearPopupVersion();
static void otaCheckTask(void *param);
static void otaUpdateTask(void *param);
static bool otaFetchReleaseInfoForTag(const String &tagName, String &binUrlOut, String &errorOut);
static bool otaStartUpdateTask(const String &binUrl, const String &targetVersion, const String &statusText);
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
void mqttPublishButtonAction(int index);
void mqttService();
bool mqttConnectNow();
void mqttPublishDiscovery();
void mqttTrimBufferForIdle();
bool mqttPublishChatMessage(const String &text);
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
void lvglSoundPopupBackdropEvent(lv_event_t *e);
void lvglSoundPopupVolumeEvent(lv_event_t *e);
void lvglSoundPopupVibrationEvent(lv_event_t *e);
void lvglSoundPopupDisableVibrationEvent(lv_event_t *e);
void lvglApplyAirplaneButtonStyle();
void lvglApplyApModeButtonStyle();
void lvglApplyChatDiscoveryButtonStyle();
void lvglApplyWifiWebServerButtonStyle();
void lvglFactoryResetEvent(lv_event_t *e);
void lvglFactoryResetConfirmEvent(lv_event_t *e);
static void factoryResetWipeStoredData();
static void factoryResetClearNamespace(Preferences &prefs, const char *ns);
static void factoryResetWipeSdData();
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
void lvglStartupSoundToggleEvent(lv_event_t *e);
void lvglSetConfigKeyboardVisible(bool visible);
void lvglShowChatAirplanePrompt();
void lvglKeyboardEnsureFocusedVisible(bool animated);
void lvglKeyboardRefreshLayout();
void lvglKeyboardApplyTextMode();
static lv_obj_t *hc12CmdTaObj();
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
static constexpr uint8_t SWIPE_BACK_COMPLETE_PERCENT = 10;
static constexpr unsigned long SWIPE_BACK_MAX_MS = 700;
static constexpr int16_t DOUBLE_TAP_MAX_MOVE = 12;
static constexpr int16_t DOUBLE_TAP_MAX_GAP = 350;
static constexpr unsigned long DOUBLE_TAP_MAX_TAP_MS = 240;
static constexpr unsigned long CLICK_SUPPRESS_AFTER_GESTURE_MS = 220UL;
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
static lv_obj_t *lvglScrStyleHomeItems = nullptr;
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
static unsigned long lvglChatPeerScanRevertAtMs = 0;
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
static lv_obj_t *lvglConfigFactoryResetBtn = nullptr;
static lv_obj_t *lvglConfigWrap = nullptr;
static lv_obj_t *lvglStyleScreensaverSw = nullptr;
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
static bool lvglStyleUiSyncing = false;
static lv_obj_t *lvglStyleHomeChatSw = nullptr;
static lv_obj_t *lvglStyleHomeMediaSw = nullptr;
static lv_obj_t *lvglStyleHomeInfoSw = nullptr;
static lv_obj_t *lvglStyleHomeGamesSw = nullptr;
static lv_obj_t *lvglStyleHomeConfigSw = nullptr;
static lv_obj_t *lvglStyleHomePowerSw = nullptr;
static lv_obj_t *lvglStyleHomeAirplaneSw = nullptr;
static lv_obj_t *lvglStyleHomeApSw = nullptr;
static lv_obj_t *lvglStyleHomeChatLabel = nullptr;
static lv_obj_t *lvglStyleHomeMediaLabel = nullptr;
static lv_obj_t *lvglStyleHomeInfoLabel = nullptr;
static lv_obj_t *lvglStyleHomeGamesLabel = nullptr;
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
static unsigned long lvglLastTickMs = 0;
static unsigned long lvglLastInfoRefreshMs = 0;
static unsigned long lvglLastStatusRefreshMs = 0;
static unsigned long lvglLastMediaPlayerRefreshMs = 0;
static bool lvglMediaPlayerVisible = false;
static bool p2pDiscoveryEnabled = true;
static bool p2pPairPromptVisible = false;
static bool p2pPairRequestPending = false;
static int p2pPairRequestDiscoveredIdx = -1;
static String p2pPairRequestPeerKey;
static String p2pPairRequestPeerName;
static IPAddress p2pPairRequestPeerIp((uint32_t)0);
static uint16_t p2pPairRequestPeerPort = 0;
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
    UI_CONFIG_STYLE_HOME_ITEMS,
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
static bool chatSavePendingOutbox();
static void chatRememberRecentMessageId(const String &peerKey, const String &messageId);
static bool chatRecentMessageIdSeen(const String &peerKey, const String &messageId);
static void chatSchedulePendingOutboxSave(unsigned long delayMs = 200UL);
static void chatPendingOutboxService();
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
static unsigned long e220RuntimeBaud();
static void hc12SerialReopen(unsigned long baud);
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
bool homeMediaVisible = true;
bool homeInfoVisible = true;
bool homeGamesVisible = true;
bool homeConfigVisible = true;
bool homePowerVisible = true;
bool homeAirplaneVisible = true;
bool homeApVisible = true;
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
OtaReleaseEntry otaReleaseEntries[OTA_RELEASE_LIST_MAX];
uint8_t otaReleaseEntryCount = 0;
int8_t otaSelectedReleaseIndex = -1;
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
String mqttChatPresenceTopic;
String mqttStatusLine = "Disabled";
bool mqttDiscoveryPublished = false;
unsigned long mqttLastReconnectMs = 0;
static constexpr unsigned long MQTT_RECONNECT_MS = 5000;
static constexpr unsigned long MQTT_DISCOVERY_STALE_MS = 60000UL;
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


struct MqttDiscoveredPeer {
    String name;
    String pubKeyHex;
    unsigned long lastSeenMs = 0;
};

static MqttDiscoveredPeer mqttDiscoveredPeers[MAX_MQTT_DISCOVERED];
static int mqttDiscoveredCount = 0;

static int mqttFindDiscoveredByPubKeyHex(const String &pubKeyHex)
{
    for (int i = 0; i < mqttDiscoveredCount; ++i) {
        if (mqttDiscoveredPeers[i].pubKeyHex.equalsIgnoreCase(pubKeyHex)) return i;
    }
    return -1;
}

static void mqttTouchDiscoveredPeer(const String &name, const String &pubKeyHex)
{
    if (pubKeyHex.isEmpty()) return;

    int idx = mqttFindDiscoveredByPubKeyHex(pubKeyHex);
    if (idx < 0) {
        if (mqttDiscoveredCount >= MAX_MQTT_DISCOVERED) {
            idx = 0;
            for (int i = 1; i < mqttDiscoveredCount; ++i) {
                if (mqttDiscoveredPeers[i].lastSeenMs < mqttDiscoveredPeers[idx].lastSeenMs) idx = i;
            }
        } else {
            idx = mqttDiscoveredCount++;
        }
    }

    mqttDiscoveredPeers[idx].name = name;
    mqttDiscoveredPeers[idx].pubKeyHex = pubKeyHex;
    mqttDiscoveredPeers[idx].lastSeenMs = millis();
}

static void mqttPruneDiscoveredPeers()
{
    const unsigned long now = millis();
    for (int i = 0; i < mqttDiscoveredCount; ) {
        if (static_cast<unsigned long>(now - mqttDiscoveredPeers[i].lastSeenMs) > MQTT_DISCOVERY_STALE_MS) {
            for (int j = i + 1; j < mqttDiscoveredCount; ++j) {
                mqttDiscoveredPeers[j - 1] = mqttDiscoveredPeers[j];
            }
            mqttDiscoveredCount--;
        } else {
            ++i;
        }
    }
}

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

static uint16_t lvglSwipeBackAnimDuration(lv_coord_t fromX, lv_coord_t toX)
{
    const uint32_t distance = static_cast<uint32_t>(abs(static_cast<int32_t>(toX) - static_cast<int32_t>(fromX)));
    if (distance == 0U) return 0;
    const uint32_t scaled = (distance * UI_ANIM_MS) / max<int16_t>(DISPLAY_WIDTH, 1);
    return static_cast<uint16_t>(constrain(scaled, static_cast<uint32_t>(SWIPE_BACK_SNAP_MIN_MS), static_cast<uint32_t>(UI_ANIM_MS)));
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

    if (screen && lv_obj_is_valid(screen)) {
        lv_anim_del(screen, nullptr);
        lv_obj_set_x(screen, 0);
    }

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
    return displayAwake &&
           (touchDown ||
            wakeTouchReleaseGuard ||
            lvglSwipeTracking ||
            lvglSwipeVisualActive ||
            lvglSwipeVisualAnimating ||
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
        case UI_CONFIG_BATTERY: return tr(TXT_BATTERY);
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
    uiPrefs.putBool("touch_snd", touchSoundEnabled);
    uiPrefs.putBool("boot_snd", startupSoundEnabled);
    uiPrefs.putBool("touch_vib", touchVibrationEnabled);
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

#include "app/ui_text.inc"
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
        lv_anim_del(child, nullptr);
        lv_obj_del(child);
    }

    const String labelText = (!menuCustomIconsEnabled && symbol && *symbol) ? lvglSymbolText(symbol, plainText) : plainText;
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
    lvglSetMenuButtonIconMode(lvglHomePowerBtn, tr(TXT_POWER), LV_SYMBOL_POWER, &img_power_small_icon);
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
    lvglSetMenuButtonIconMode(
        lvglConfigFactoryResetBtn,
        tr(TXT_FACTORY_RESET),
        LV_SYMBOL_WARNING,
        &img_power_small_icon,
        8,
        12
    );
    lvglRefreshBatteryTrainButtonIcons();
}

static void lvglSetHomeButtonVisibleState(lv_obj_t *btn, bool visible)
{
    if (!btn) return;
    if (visible) lv_obj_clear_flag(btn, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(btn, LV_OBJ_FLAG_HIDDEN);
}

static void lvglRefreshHomeButtonVisibility()
{
    lvglSetHomeButtonVisibleState(lvglHomeChatBtn, homeChatVisible);
    lvglSetHomeButtonVisibleState(lvglHomeMediaBtn, homeMediaVisible);
    lvglSetHomeButtonVisibleState(lvglHomeInfoBtn, homeInfoVisible);
    lvglSetHomeButtonVisibleState(lvglHomeGamesBtn, homeGamesVisible);
    lvglSetHomeButtonVisibleState(lvglHomeConfigBtn, homeConfigVisible);
    lvglSetHomeButtonVisibleState(lvglHomePowerBtn, homePowerVisible);
    lvglSetHomeButtonVisibleState(lvglAirplaneBtn, homeAirplaneVisible);
    lvglSetHomeButtonVisibleState(lvglApModeBtn, homeApVisible);
}

static void lvglApplyMomentaryButtonStyle(lv_obj_t *btn, lv_obj_t *label, lv_color_t bodyCol, bool compact)
{
    if (!btn) return;

    if (lvglBlackButtonThemeActive()) {
        const lv_color_t blackBody = lv_color_hex(UI_BUTTON_BLACK_BODY_HEX);
        const lv_color_t activeGreen = lvglActiveToggleGreen(compact);
        const lv_color_t pressedCol = lv_color_mix(lv_color_black(), activeGreen, compact ? 44 : 58);
        const lv_color_t flashColor = lv_color_mix(lv_color_white(), activeGreen, compact ? 70 : 86);
        lv_obj_set_style_bg_color(btn, blackBody, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_color(btn, blackBody, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(btn, activeGreen, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, activeGreen, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_grad_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(btn, pressedCol, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, pressedCol, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_border_width(btn, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_color(btn, lv_color_hex(UI_BUTTON_BLACK_BORDER_HEX), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_opa(btn, static_cast<lv_opa_t>(84), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(btn, 1, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_border_color(btn, lv_color_mix(lv_color_white(), activeGreen, 104), LV_PART_MAIN | LV_STATE_PRESSED);
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
        const lv_color_t blackPressed = lv_color_hex(UI_BUTTON_BLACK_PRESSED_HEX);
        const lv_color_t blackFlash = lv_color_hex(UI_BUTTON_BLACK_FLASH_HEX);
        const lv_color_t activeBody = enabled ? bodyCol : blackBody;
        const lv_color_t activePressed = enabled ? lv_color_mix(lv_color_black(), bodyCol, compact ? 72 : 92) : blackPressed;
        const lv_color_t activeFlash = enabled ? flashColor : blackFlash;
        const lv_color_t borderCol = lv_color_hex(enabled ? UI_BUTTON_BLACK_BORDER_ACTIVE_HEX : UI_BUTTON_BLACK_BORDER_HEX);
        lv_obj_set_style_bg_color(btn, activeBody, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_color(btn, activeBody, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(btn, activePressed, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, activePressed, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_color(btn, activeFlash, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_grad_color(btn, activeFlash, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(btn, activeFlash, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, activeFlash, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
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
        lvglApModeBtn, lvglApModeBtnLabel, wifiSessionApMode, lv_color_hex(0xA66A2A), lv_color_hex(0x3A7A3A), false);
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
        if (lv_obj_has_flag(cur, LV_OBJ_FLAG_SCROLLABLE)) return cur;
    }
    return nullptr;
}

static void lvglKeyboardReleasePaddedContainer()
{
    if (lvglKeyboardPaddedContainer && lv_obj_is_valid(lvglKeyboardPaddedContainer)) {
        lv_obj_set_style_pad_bottom(lvglKeyboardPaddedContainer, lvglKeyboardPaddedContainerPrevBottom, 0);
    }
    lvglKeyboardPaddedContainer = nullptr;
    lvglKeyboardPaddedContainerPrevBottom = 0;
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

void lvglKeyboardRefreshLayout()
{
    if (!lvglKeyboardVisible() || !lvglKeyboardFocusedTa || !lv_obj_is_valid(lvglKeyboardFocusedTa)) {
        lvglKeyboardReleasePaddedContainer();
        return;
    }

    if (lvglKeyboardFocusedTa == lvglChatInputTa) {
        lvglSetChatKeyboardVisible(true);
        lvglKeyboardEnsureFocusedVisible(false);
        return;
    }

    const bool configManagedTarget =
        (lvglKeyboardFocusedTa == lvglConfigDeviceNameTa) ||
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
            const bool forceSwipeBackScreen = (uiScreen == UI_CONFIG_STYLE_HOME_ITEMS);
            lvglSwipeTracking = true;
            lvglSwipeStartX = lvglLastTouchX;
            lvglSwipeStartY = lvglLastTouchY;
            lvglSwipeLastX = lvglLastTouchX;
            lvglSwipeLastY = lvglLastTouchY;
            lvglSwipeStartMs = now;
            lvglSwipeCandidate = uiScreenSupportsSwipeBack(uiScreen) &&
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
            if (abs(dx) >= swipeLockMinDx || abs(dy) >= SWIPE_CANCEL_VERTICAL_DY || lvglSwipePressCancelled) {
                lvglSuppressClicksAfterGesture();
            }
            lvglLastTapReleaseMs = 0;
        }

        lvglSwipeCandidate = false;
        lvglSwipeHorizontalLocked = false;
        lvglSwipePressCancelled = false;
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
        queueInfoRefresh(false);
        lvglRefreshInfoPanel(false);
    }
}

#include "app/battery.inc"

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

#include "app/wifi_scan.inc"

#include "app/games_snake.inc"

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

#include "app/ota_backend.inc"
#include "app/web_routes.inc"
void setupWifiAndServer()
{
    if (airplaneModeEnabled) {
        wifiRuntimeManaged = false;
        stopWebServerRuntime();
        WiFi.mode(WIFI_OFF);
        apModeActive = false;
        return;
    }

    wifiRuntimeManaged = true;
    registerWifiEvents();
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    WiFi.setSleep(false);
    stopDnsForAp();

    /* AP-mode boot preference */
    if (wifiSessionApMode) {
        Serial.println("Booting in saved AP mode");
        ensureApOnline("boot_pref");
        return;
    }

    /* normal STA boot path */
    WiFi.mode(WIFI_STA);
    apModeActive = false;
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
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step sodium_init");
    p2pReady = sodium_init() >= 0;
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step loadP2pConfig");
    if (p2pReady) loadP2pConfig();
    if (VERBOSE_SERIAL_DEBUG) Serial.println("[BOOT] step loadUiRuntimeConfig");
    loadUiRuntimeConfig();
    if (radioModuleType == RADIO_MODULE_E220) {
        pinMode(e220ActiveM0Pin(), OUTPUT);
        pinMode(e220ActiveM1Pin(), OUTPUT);
        digitalWrite(e220ActiveM0Pin(), LOW);
        digitalWrite(e220ActiveM1Pin(), LOW);
        delay(120);
        hc12SerialReopen(e220RuntimeBaud());
        delay(30);
    }    
    hc12InitIfNeeded();
    if (radioModuleType == RADIO_MODULE_E220) {
        hc12ReadConfigSelection();
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
    playStartupFeedbackIfEnabled();
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
                if (!uiPriorityActive || realtimeMessaging) {
                    chatPendingService();   // keep for WiFi/MQTT generic outbox maintenance
                }
                break;

            case 3:
                if (!uiPriorityActive || !sdMounted) sdStatsService();
                if (!uiPriorityActive) serviceCarInputTelemetry();
                if (!uiPriorityActive || uiScreen == UI_INFO) serviceInfoRefresh();

                if (!uiPriorityActive || uiScreen == UI_INFO || uiScreen == UI_CONFIG_HC12_INFO) {
                    serviceDeferredRadioInfoFetch();
                }
                break;

            case 4:
                if (!uiPriorityActive) mqttPruneDiscoveredPeers();
                if (!uiPriorityActive) hc12PruneDiscoveredPeers();

                if (!uiPriorityActive) refreshMdnsState();
                if (!uiPriorityActive || uiScreen == UI_CONFIG_OTA) otaCheckService();
                break;
        }
    }
    otaUpdateService();
    hc12Service();
    if ((!uiPriorityActive || uiScreen == UI_CONFIG_HC12) && hc12ConfigApplyPending != HC12_CFG_APPLY_NONE) {
        serviceDeferredHc12ConfigApply();
    }
    radioChatService();
    if (!uiPriorityActive || realtimeMessaging || chatPendingCount > 0) {
        chatPendingService();
    }
    chatPendingOutboxService();
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
