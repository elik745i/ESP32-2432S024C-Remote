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
static constexpr unsigned long BOOT_DEFER_WIFI_MS = 1200UL;
static constexpr unsigned long BOOT_INTERACTIVE_GRACE_MS = 2200UL;
static constexpr unsigned long BOOT_DEFER_STARTUP_FEEDBACK_MS = 450UL;
static constexpr unsigned long BOOT_DEFER_RADIO_INIT_MS = 900UL;
static constexpr unsigned long BOOT_DEFER_RADIO_PROBE_MS = 1600UL;
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
