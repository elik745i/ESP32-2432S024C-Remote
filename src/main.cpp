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
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>
#include <Audio.h>
#include <JPEGENC.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <math.h>
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ESP32-2432S024C (CST820 capacitive touch).
static constexpr int TOUCH_SDA = 33;
static constexpr int TOUCH_SCL = 32;
static constexpr int TOUCH_RST = 25;
static constexpr int TOUCH_IRQ = 21;
static constexpr int TOUCH_ADDR = 0x15;
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

// User-provided speaker pin (SC8002B related, reserved for I2S usage).
static constexpr int I2S_SPK_PIN = 26;
static constexpr uint8_t AUDIO_I2S_PORT = I2S_NUM_0;
static constexpr uint8_t AUDIO_VOLUME_TARGET = 21;
static constexpr bool AUDIO_FORCE_MONO_INTERNAL_DAC = true;
static constexpr int RGB_PIN_R = 4;
static constexpr int RGB_PIN_G = 17;
static constexpr int RGB_PIN_B = 16;
static constexpr bool RGB_ACTIVE_LOW = true;
static constexpr unsigned long LCD_IDLE_TIMEOUT_MS = 15000;
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
static constexpr unsigned long LOW_BATT_POPUP_PERIOD_MS = 30000;
static constexpr unsigned long FULL_BATT_POPUP_PERIOD_MS = 30000;
static constexpr int LIGHT_ADC_SAMPLES = 8;
static constexpr float LIGHT_FILTER_ALPHA = 0.20f;
static constexpr bool LIGHT_INVERT = true;
static constexpr int LIGHT_MIN_SPAN_RAW = 80;
static constexpr uint16_t LIGHT_RAW_CAL_MIN = 0;
static constexpr uint16_t LIGHT_RAW_CAL_MAX = 600;
static constexpr bool LIGHT_LOG_RAW_TO_SERIAL = true;

static constexpr const char *AP_SSID = "ESP32-2432S024C-FM";
static constexpr const char *AP_PASS = "12345678";
static constexpr const char *FW_VERSION = "0.1.0";
static constexpr const char *MDNS_HOST = "esp32-2432s024c";

struct UiRect {
    int16_t x;
    int16_t y;
    int16_t w;
    int16_t h;
};

struct FsUploadCtx {
    String destPath;
    String tmpPath;
    File file;
    String error;
    bool started = false;
};

TFT_eSPI tft;
SPIClass sdSpi(VSPI);
AsyncWebServer server(80);
DNSServer dnsServer;
Audio audio(true, I2S_DAC_CHANNEL_LEFT_EN, AUDIO_I2S_PORT);
WiFiClient mqttNetClient;
PubSubClient mqttClient(mqttNetClient);
void redrawUi();
bool applyWifiListScrollDelta(int deltaPages);
bool applyMediaListScrollDelta(int deltaPages);
bool applyHomeMenuScrollDelta(int deltaPages);
void drawUiGameSnake();
void drawUiGameTetris();
void mqttPublishAction(const char *action);
void mqttPublishButtonAction(int index);
void mqttService();
void popupService();
void renderTransientPopupOverlay();
bool mqttConnectNow();
void mqttPublishDiscovery();
void loadMqttConfig();
void saveMqttConfig();
void handleHorizontalSwipeGesture(int direction);
bool isMenuScreenForScreenshot();
void triggerScreenshotCapture();
void screenshotService();
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

bool touchWasDown = false;
bool touchTracking = false;
bool touchGestureConsumed = false;
int16_t touchStartX = 0;
int16_t touchStartY = 0;
int16_t touchLastX = 0;
int16_t touchLastY = 0;
bool sdMounted = false;
bool sdSpiReady = false;
unsigned long sdLastAutoRetryMs = 0;
bool mdnsStarted = false;
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
bool ignoreTouchUntilRelease = false;
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
static constexpr int TOUCH_SWIPE_MIN_DELTA = 32;
static constexpr int TOUCH_SWIPE_DOMINANCE_PX = 10;
static constexpr int TOUCH_HSWIPE_MIN_DELTA = 48;
static constexpr uint8_t SWIPE_ANIM_STEPS = 8;
static constexpr uint8_t SWIPE_ANIM_FRAME_MS = 8;
static constexpr int WIFI_LIST_VIEW_X = 8;
static constexpr int WIFI_LIST_VIEW_Y = 52;
static constexpr int WIFI_LIST_VIEW_W = 224;
static constexpr int WIFI_LIST_VIEW_H = 205;
static constexpr int MEDIA_LIST_VIEW_X = 8;
static constexpr int MEDIA_LIST_VIEW_Y = 50;
static constexpr int MEDIA_LIST_VIEW_W = 224;
static constexpr int MEDIA_LIST_VIEW_H = 152;
static constexpr int HOME_MENU_X = 20;
static constexpr int HOME_MENU_Y = 58;
static constexpr int HOME_MENU_W = 200;
static constexpr int HOME_MENU_BUTTON_H = 50;
static constexpr int HOME_MENU_GAP = 10;
static constexpr int HOME_MENU_PER_PAGE = 4;
static constexpr int HOME_MENU_VIEW_W = HOME_MENU_W;
static constexpr int HOME_MENU_VIEW_H = (HOME_MENU_PER_PAGE * HOME_MENU_BUTTON_H) + ((HOME_MENU_PER_PAGE - 1) * HOME_MENU_GAP);
static constexpr uint16_t BTN_GLOSS_GREEN = 0x3EC8;
static constexpr uint16_t BTN_GLOSS_PURPLE = 0x681F;
static constexpr uint16_t BTN_GLOSS_ORANGE = 0xFD20;
static constexpr uint16_t BTN_GLOSS_BLUE = 0x2D7F;
static constexpr uint16_t BTN_GLOSS_CYAN = 0x2D7C;
static constexpr uint16_t BTN_GLOSS_PINK = 0xF16F;

enum UiScreen : uint8_t {
    UI_HOME,
    UI_WIFI_LIST,
    UI_KEYBOARD,
    UI_MEDIA,
    UI_INFO,
    UI_GAMES,
    UI_CONFIG,
    UI_CONFIG_STYLE,
    UI_CONFIG_MQTT,
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
int wifiPage = 0;
int homeMenuPage = 0;

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

String selectedSsid;
String typedPassword;
bool kbUpper = false;
bool kbShowPassword = false;

struct MediaEntry {
    String name;
    String path;
    bool isDir;
    size_t size;
};

static constexpr int MAX_MEDIA_ENTRIES = 80;
MediaEntry mediaEntries[MAX_MEDIA_ENTRIES];
int mediaCount = 0;
int mediaPage = 0;
int mediaSelected = -1;
String mediaCurrentDir = "/";
String mediaNowPlaying;
String mediaPlaybackPath;
bool mediaIsPlaying = false;
bool mediaPaused = false;
bool rgbPersistR = false;
bool rgbPersistG = false;
bool rgbPersistB = false;
unsigned long rgbFlashUntilMs = 0;
uint8_t rgbPulseLevel = 32;
int8_t rgbPulseDir = 1;
unsigned long rgbPulseLastMs = 0;
uint8_t audioVolumeCurrent = 0;

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
unsigned long lastBatterySnapshotMs = 0;
float lastBatterySnapshotVoltage = -1.0f;
unsigned long lastLowBatteryPopupMs = 0;
unsigned long lastFullBatteryPopupMs = 0;

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
int mqttControlPage = 0;
int mqttEditButtonIndex = 0;
bool mqttConfirmVisible = false;
int mqttConfirmButtonIndex = -1;
bool uiPopupActive = false;
String uiPopupTitle;
String uiPopupLine;
uint16_t uiPopupColor = TFT_DARKGREY;
unsigned long uiPopupUntilMs = 0;
bool screenshotPending = false;
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

bool hitRect(const UiRect &r, int16_t x, int16_t y)
{
    return x >= r.x && x < (r.x + r.w) && y >= r.y && y < (r.y + r.h);
}

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

void cstInit()
{
    Wire.begin(TOUCH_SDA, TOUCH_SCL, 400000);
    pinMode(TOUCH_RST, OUTPUT);
    digitalWrite(TOUCH_RST, LOW);
    delay(10);
    digitalWrite(TOUCH_RST, HIGH);
    delay(300);
    cstWriteReg(0xFE, 0xFF);
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
    uint8_t touched = 0;
    if (!cstReadRegs(0x02, &touched, 1) || touched == 0) return false;

    uint8_t data[4] = {0, 0, 0, 0};
    if (!cstReadRegs(0x03, data, sizeof(data))) return false;

    int16_t x = static_cast<int16_t>(((data[0] & 0x0F) << 8) | data[1]);
    int16_t y = static_cast<int16_t>(((data[2] & 0x0F) << 8) | data[3]);
    x = constrain(x, 0, tft.width() - 1);
    y = constrain(y, 0, tft.height() - 1);
    rotatePointByOffset(x, y, TOUCH_ROTATION_OFFSET);
    sx = x;
    sy = y;
    return true;
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

int wifiBarsFromRssi(int32_t rssi)
{
    if (rssi >= -55) return 4;
    if (rssi >= -67) return 3;
    if (rssi >= -75) return 2;
    if (rssi >= -85) return 1;
    return 0;
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

void drawWifiIndicator(int16_t x, int16_t y)
{
    const bool connected = (WiFi.status() == WL_CONNECTED);
    const int bars = connected ? wifiBarsFromRssi(WiFi.RSSI()) : 0;

    const int barW = 6;
    const int gap = 3;
    const int baseY = y + 12;
    for (int i = 0; i < 4; i++) {
        const int h = 4 + i * 2;
        const int bx = x + i * (barW + gap);
        const int by = baseY - h;
        tft.drawRect(bx, by, barW, h, TFT_DARKGREY);
        if (i < bars) {
            uint16_t c = TFT_GREEN;
            if (bars <= 2) c = TFT_YELLOW;
            if (bars <= 1) c = TFT_RED;
            tft.fillRect(bx + 1, by + 1, barW - 2, h - 2, c);
        }
    }

    if (!connected) {
        tft.drawLine(x + 2, y + 2, x + 34, y + 14, TFT_RED);
        tft.drawLine(x + 34, y + 2, x + 2, y + 14, TFT_RED);
    }
}

void drawBatteryIndicator(int16_t x, int16_t y)
{
    const int bodyW = 20;
    const int bodyH = 10;
    const int tipW = 2;
    const int tipH = 4;

    tft.drawRect(x, y, bodyW, bodyH, TFT_WHITE);
    tft.fillRect(x + bodyW, y + ((bodyH - tipH) / 2), tipW, tipH, TFT_WHITE);

    int shownPercent = batteryPercent;
    if (shownPercent < 0) shownPercent = 0;
    if (shownPercent > 100) shownPercent = 100;

    int fillPct = shownPercent;
    if (batteryCharging) {
        unsigned long now = millis();
        if (now - batteryIconAnimLastMs >= 2) {
            batteryIconAnimLastMs = now;
            batteryIconAnimPercent = static_cast<uint8_t>(batteryIconAnimPercent + 12);
            if (batteryIconAnimPercent > shownPercent) batteryIconAnimPercent = 0;
        }
        fillPct = batteryIconAnimPercent;
    } else {
        batteryIconAnimPercent = 0;
    }

    int fillW = ((bodyW - 2) * fillPct) / 100;
    uint16_t fillCol = TFT_GREEN;
    if (batteryPercent <= 30) fillCol = TFT_RED;
    else if (batteryPercent <= 60) fillCol = TFT_YELLOW;
    if (fillW > 0) tft.fillRect(x + 1, y + 1, fillW, bodyH - 2, fillCol);

    String pct = String(batteryPercent) + "%";
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(pct, x + bodyW + tipW + 4, y - 2, 2);
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

void animateLowBatteryWarning()
{
    digitalWrite(TFT_BL, HIGH);
    const int16_t x = 40, y = 110, w = 150, h = 70;
    const int pct = 5;
    const int innerW = ((w - 4) * pct) / 100;
    tft.fillScreen(TFT_BLACK);
    drawLargeBatteryFrame(x, y, w, h);
    for (int cycle = 0; cycle < 2; cycle++) {
        for (int level = 40; level <= 255; level += 12) {
            tft.fillRect(x + 2, y + 2, w - 4, h - 4, TFT_BLACK);
            tft.fillRoundRect(x + 2, y + 2, innerW, h - 4, 5, scaleColor565(TFT_RED, static_cast<uint8_t>(level)));
            tft.fillRect(24, 70, 192, 24, TFT_BLACK);
            tft.setTextColor(scaleColor565(TFT_RED, static_cast<uint8_t>(level)), TFT_BLACK);
            tft.drawCentreString("LOW BATTERY", 120, 70, 4);
            delay(35);
        }
        for (int level = 255; level >= 40; level -= 12) {
            tft.fillRect(x + 2, y + 2, w - 4, h - 4, TFT_BLACK);
            tft.fillRoundRect(x + 2, y + 2, innerW, h - 4, 5, scaleColor565(TFT_RED, static_cast<uint8_t>(level)));
            tft.fillRect(24, 70, 192, 24, TFT_BLACK);
            tft.setTextColor(scaleColor565(TFT_RED, static_cast<uint8_t>(level)), TFT_BLACK);
            tft.drawCentreString("LOW BATTERY", 120, 70, 4);
            delay(35);
        }
    }
}

void animateFullBatteryNotification()
{
    digitalWrite(TFT_BL, HIGH);
    const int16_t x = 40, y = 110, w = 150, h = 70;
    tft.fillScreen(TFT_BLACK);
    drawLargeBatteryFrame(x, y, w, h);
    drawChargingCableDecor(x, y, w, h, TFT_DARKGREY);
    for (int i = 0; i < 2; i++) {
        for (int level = 40; level <= 255; level += 12) {
            tft.fillRect(x + 2, y + 2, w - 4, h - 4, TFT_BLACK);
            int innerW = (w - 4); // full
            tft.fillRoundRect(x + 2, y + 2, innerW, h - 4, 5, scaleColor565(TFT_GREEN, static_cast<uint8_t>(level)));
            drawChargingCableDecor(x, y, w, h, scaleColor565(TFT_GREEN, static_cast<uint8_t>(level)));
            tft.fillRect(24, 70, 192, 24, TFT_BLACK);
            tft.setTextColor(scaleColor565(TFT_GREEN, static_cast<uint8_t>(level)), TFT_BLACK);
            tft.drawCentreString("BATTERY FULL", 120, 70, 4);
            delay(35);
        }
        for (int level = 255; level >= 40; level -= 12) {
            tft.fillRect(x + 2, y + 2, w - 4, h - 4, TFT_BLACK);
            int innerW = (w - 4); // full
            tft.fillRoundRect(x + 2, y + 2, innerW, h - 4, 5, scaleColor565(TFT_GREEN, static_cast<uint8_t>(level)));
            drawChargingCableDecor(x, y, w, h, scaleColor565(TFT_GREEN, static_cast<uint8_t>(level)));
            tft.fillRect(24, 70, 192, 24, TFT_BLACK);
            tft.setTextColor(scaleColor565(TFT_GREEN, static_cast<uint8_t>(level)), TFT_BLACK);
            tft.drawCentreString("BATTERY FULL", 120, 70, 4);
            delay(35);
        }
    }
}

void drawLightIndicator(int16_t x, int16_t y)
{
    const uint16_t bulbColor = (lightPercent > 60) ? TFT_YELLOW : TFT_LIGHTGREY;
    tft.drawCircle(x + 5, y + 6, 4, bulbColor);
    tft.fillRect(x + 4, y + 10, 3, 4, bulbColor);
    String txt = "L" + String(lightPercent) + "%";
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(txt, x + 12, y + 1, 2);
}

void drawTopStatus(const String &line1, const String &line2 = "")
{
    (void)line1;
    (void)line2;
    sampleTopIndicators();
    tft.fillRect(0, 0, tft.width(), 44, TFT_BLACK);
    drawWifiIndicator(6, 5);
    drawLightIndicator(95, 7);
    drawBatteryIndicator(tft.width() - 54, 8);
}

void drawMenuButton(const UiRect &r, const String &label, uint16_t fill = TFT_DARKCYAN, bool pressed = false)
{
    UiRect b = r;
    if (pressed) b.y += 2;
    const int radius = (b.h / 2) < 9 ? (b.h / 2) : 9;
    uint16_t face = pressed ? scaleColor565(fill, 180) : fill;
    uint16_t topGloss = scaleColor565(TFT_WHITE, pressed ? 70 : 110);
    uint16_t lowerShade = scaleColor565(TFT_BLACK, pressed ? 110 : 170);
    uint16_t rim = scaleColor565(TFT_WHITE, pressed ? 80 : 130);

    if (!pressed) tft.fillRoundRect(b.x + 2, b.y + 4, b.w, b.h, radius, scaleColor565(TFT_BLACK, 120));
    tft.fillRoundRect(b.x, b.y, b.w, b.h, radius, face);
    tft.drawRoundRect(b.x, b.y, b.w, b.h, radius, rim);
    tft.fillRoundRect(b.x + 4, b.y + 3, b.w - 8, (b.h / 2) - 2, (radius > 3 ? radius - 3 : 1), topGloss);
    tft.drawFastHLine(b.x + 6, b.y + b.h - 4, b.w - 12, lowerShade);
    tft.setTextColor(TFT_WHITE, face);
    tft.drawCentreString(label, b.x + (b.w / 2), b.y + (b.h / 2) - 8, 2);
}

void animateUiButtonPress(const UiRect &r, const String &label, uint16_t fill)
{
    drawMenuButton(r, label, fill, true);
    delay(65);
}

const char *authName(wifi_auth_mode_t auth)
{
    return auth == WIFI_AUTH_OPEN ? "Open" : "Secured";
}

void refreshWifiScan()
{
    WiFi.scanDelete();
    wifiCount = 0;
    wifiPage = 0;
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

struct HomeMenuItem {
    const char *label;
    uint16_t color;
    UiScreen target;
};

static const HomeMenuItem HOME_MENU_ITEMS[] = {
    {"WiFi Setup", BTN_GLOSS_GREEN, UI_WIFI_LIST},
    {"Web Recovery", BTN_GLOSS_ORANGE, UI_HOME},
    {"MediaPlayer", BTN_GLOSS_BLUE, UI_MEDIA},
    {"Info", BTN_GLOSS_PURPLE, UI_INFO},
    {"Games", BTN_GLOSS_CYAN, UI_GAMES},
    {"Configuration", BTN_GLOSS_PINK, UI_CONFIG},
};

int homeMenuItemCount()
{
    return static_cast<int>(sizeof(HOME_MENU_ITEMS) / sizeof(HOME_MENU_ITEMS[0]));
}

int homeMenuPageCount()
{
    const int n = homeMenuItemCount();
    return (n + HOME_MENU_PER_PAGE - 1) / HOME_MENU_PER_PAGE;
}

void drawHomeButton3D(const UiRect &r, const String &label, uint16_t base, bool pressed = false)
{
    UiRect b = r;
    if (pressed) b.y += 2;
    const int radius = b.h / 2;
    uint16_t face = pressed ? scaleColor565(base, 180) : base;
    uint16_t topGloss = scaleColor565(TFT_WHITE, pressed ? 70 : 110);
    uint16_t lowerShade = scaleColor565(TFT_BLACK, pressed ? 110 : 170);
    uint16_t rim = scaleColor565(TFT_WHITE, pressed ? 80 : 130);

    if (!pressed) tft.fillRoundRect(b.x + 2, b.y + 4, b.w, b.h, radius, scaleColor565(TFT_BLACK, 120));
    tft.fillRoundRect(b.x, b.y, b.w, b.h, radius, face);
    tft.drawRoundRect(b.x, b.y, b.w, b.h, radius, rim);
    tft.fillRoundRect(b.x + 4, b.y + 3, b.w - 8, (b.h / 2) - 2, radius - 3, topGloss);
    tft.drawFastHLine(b.x + 6, b.y + b.h - 4, b.w - 12, lowerShade);
    tft.setTextColor(TFT_WHITE, face);
    tft.drawCentreString(label, b.x + (b.w / 2), b.y + (b.h / 2) - 8, 2);
}

void drawHomeButton3DToSprite(TFT_eSprite &spr, const UiRect &r, const String &label, uint16_t base)
{
    const int radius = r.h / 2;
    uint16_t topGloss = scaleColor565(TFT_WHITE, 110);
    uint16_t lowerShade = scaleColor565(TFT_BLACK, 170);
    uint16_t rim = scaleColor565(TFT_WHITE, 130);
    spr.fillRoundRect(r.x + 2, r.y + 4, r.w, r.h, radius, scaleColor565(TFT_BLACK, 120));
    spr.fillRoundRect(r.x, r.y, r.w, r.h, radius, base);
    spr.drawRoundRect(r.x, r.y, r.w, r.h, radius, rim);
    spr.fillRoundRect(r.x + 4, r.y + 3, r.w - 8, (r.h / 2) - 2, radius - 3, topGloss);
    spr.drawFastHLine(r.x + 6, r.y + r.h - 4, r.w - 12, lowerShade);
    spr.setTextColor(TFT_WHITE, base);
    spr.drawCentreString(label, r.x + (r.w / 2), r.y + (r.h / 2) - 8, 2);
}

void drawHomeMenuPageContent(int page, int16_t yOffset, int pressedSlot = -1)
{
    const int start = page * HOME_MENU_PER_PAGE;
    for (int i = 0; i < HOME_MENU_PER_PAGE; i++) {
        int idx = start + i;
        UiRect row{HOME_MENU_X, static_cast<int16_t>(HOME_MENU_Y + i * (HOME_MENU_BUTTON_H + HOME_MENU_GAP) + yOffset), HOME_MENU_W, HOME_MENU_BUTTON_H};
        if (row.y + row.h < HOME_MENU_Y || row.y > (HOME_MENU_Y + HOME_MENU_VIEW_H - 1)) continue;
        if (idx < homeMenuItemCount()) {
            drawHomeButton3D(row, HOME_MENU_ITEMS[idx].label, HOME_MENU_ITEMS[idx].color, pressedSlot == i);
        } else {
            tft.fillRoundRect(row.x, row.y, row.w, row.h, 9, TFT_BLACK);
        }
    }
}

void drawHomeMenuPageContentToSprite(TFT_eSprite &spr, int page, int16_t yOffset)
{
    const int start = page * HOME_MENU_PER_PAGE;
    for (int i = 0; i < HOME_MENU_PER_PAGE; i++) {
        int idx = start + i;
        UiRect row{0, static_cast<int16_t>(i * (HOME_MENU_BUTTON_H + HOME_MENU_GAP) + yOffset), HOME_MENU_W, HOME_MENU_BUTTON_H};
        if (row.y + row.h < 0 || row.y > (HOME_MENU_VIEW_H - 1)) continue;
        if (idx < homeMenuItemCount()) {
            drawHomeButton3DToSprite(spr, row, HOME_MENU_ITEMS[idx].label, HOME_MENU_ITEMS[idx].color);
        }
    }
}

void drawUiHomeChrome()
{
    tft.fillScreen(TFT_BLACK);
    const bool staConnected = (WiFi.status() == WL_CONNECTED);
    drawTopStatus("WiFi Manager", "STA: " + (staConnected ? WiFi.localIP().toString() : String("not connected")));
    const int pages = homeMenuPageCount();
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.drawRightString(String(homeMenuPage + 1) + "/" + String(pages), 232, 300, 2);
}

void drawUiHome()
{
    drawUiHomeChrome();
    drawHomeMenuPageContent(homeMenuPage, 0);
}

void animateHomeMenuTransition(int oldPage, int newPage)
{
    if (oldPage == newPage) return;
    const int dir = (newPage > oldPage) ? 1 : -1;
    const int travel = HOME_MENU_VIEW_H;
    drawUiHomeChrome();

    TFT_eSprite spr(&tft);
    spr.setColorDepth(16);
    bool spriteReady = (spr.createSprite(HOME_MENU_VIEW_W, HOME_MENU_VIEW_H) != nullptr);

    if (spriteReady) {
        for (int step = 1; step <= SWIPE_ANIM_STEPS; step++) {
            int p = (travel * step * step) / (SWIPE_ANIM_STEPS * SWIPE_ANIM_STEPS);
            int oldOffset = (dir > 0) ? -p : p;
            int newOffset = (dir > 0) ? (travel - p) : (-travel + p);
            spr.fillSprite(TFT_BLACK);
            drawHomeMenuPageContentToSprite(spr, oldPage, oldOffset);
            drawHomeMenuPageContentToSprite(spr, newPage, newOffset);
            spr.pushSprite(HOME_MENU_X, HOME_MENU_Y);
            delay(SWIPE_ANIM_FRAME_MS);
        }
        spr.deleteSprite();
    } else {
        tft.startWrite();
        for (int step = 1; step <= SWIPE_ANIM_STEPS; step++) {
            int p = (travel * step * step) / (SWIPE_ANIM_STEPS * SWIPE_ANIM_STEPS);
            int oldOffset = (dir > 0) ? -p : p;
            int newOffset = (dir > 0) ? (travel - p) : (-travel + p);
            tft.fillRect(HOME_MENU_X, HOME_MENU_Y, HOME_MENU_VIEW_W, HOME_MENU_VIEW_H, TFT_BLACK);
            drawHomeMenuPageContent(oldPage, oldOffset);
            drawHomeMenuPageContent(newPage, newOffset);
            delay(SWIPE_ANIM_FRAME_MS);
        }
        tft.endWrite();
    }
}

void drawUiInfo()
{
    tft.fillScreen(TFT_BLACK);
    const bool staConnected = (WiFi.status() == WL_CONNECTED);
    const String ipLine = staConnected
                              ? ("STA IP: " + WiFi.localIP().toString())
                              : ("AP IP: " + WiFi.softAPIP().toString());
    const String apIpLine = "AP IP: " + WiFi.softAPIP().toString();

    drawTopStatus("Info", "Device details");
    drawMenuButton({8, 268, 52, 44}, "Back", TFT_DARKGREY);

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(ipLine, 8, 64, 2);
    tft.drawString(apIpLine, 8, 94, 2);
    tft.drawString("FW " + String(FW_VERSION), 8, 124, 2);
}

void drawUiGames()
{
    tft.fillScreen(TFT_BLACK);
    drawTopStatus("Games", "Select game");
    drawMenuButton({8, 268, 52, 44}, "Back", TFT_DARKGREY);
    drawMenuButton({20, 92, 200, 52}, "Snake", BTN_GLOSS_GREEN);
    drawMenuButton({20, 158, 200, 52}, "Tetris", BTN_GLOSS_BLUE);
}

void drawUiConfig()
{
    tft.fillScreen(TFT_BLACK);
    drawTopStatus("Configuration", "Select submenu");
    drawMenuButton({8, 268, 52, 44}, "Back", TFT_DARKGREY);
    drawMenuButton({20, 92, 200, 52}, "Style", BTN_GLOSS_CYAN);
    drawMenuButton({20, 158, 200, 52}, "MQTT", BTN_GLOSS_BLUE);
}

void drawUiConfigStyle()
{
    tft.fillScreen(TFT_BLACK);
    drawTopStatus("Configuration", "Style");
    drawMenuButton({8, 268, 52, 44}, "Back", TFT_DARKGREY);
    drawMenuButton({20, 96, 200, 48}, "Theme: Glossy", BTN_GLOSS_PURPLE);
    drawMenuButton({20, 154, 200, 48}, "Button FX: ON", BTN_GLOSS_GREEN);
    drawMenuButton({20, 212, 200, 48}, "Swipe FX: ON", BTN_GLOSS_CYAN);
}

void drawUiConfigMqtt()
{
    tft.fillScreen(TFT_BLACK);
    drawTopStatus("Configuration", "MQTT");
    drawMenuButton({8, 268, 52, 44}, "Back", TFT_DARKGREY);
    drawMenuButton({20, 102, 200, 60}, "Config", BTN_GLOSS_CYAN);
    drawMenuButton({20, 176, 200, 60}, "Controls", BTN_GLOSS_BLUE);
}

void drawUiConfigMqttConfig()
{
    tft.fillScreen(TFT_BLACK);
    drawTopStatus("MQTT", "Config");
    String hw = mqttHwId;
    if (hw.length() > 12) hw = hw.substring(hw.length() - 12);
    String brokerLine = mqttCfg.broker + ":" + String(mqttCfg.port);
    if (brokerLine.length() > 22) brokerLine = brokerLine.substring(0, 22) + "...";
    String btnName = mqttButtonNames[mqttEditButtonIndex];
    if (btnName.length() > 14) btnName = btnName.substring(0, 14) + "...";
    String crit = mqttButtonCritical[mqttEditButtonIndex] ? "ON" : "OFF";
    String st = mqttStatusLine;
    if (st.length() > 20) st = st.substring(0, 20) + "...";

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("HW: " + hw, 8, 50, 2);
    tft.drawString("Broker: " + brokerLine, 8, 68, 2);
    tft.drawString("Btns: " + String(mqttButtonCount), 8, 86, 2);
    tft.drawString("Edit #" + String(mqttEditButtonIndex + 1) + ": " + btnName, 8, 104, 2);
    tft.drawString("Crit #" + String(mqttEditButtonIndex + 1) + ": " + crit, 8, 122, 2);
    tft.drawString("Status: " + st, 8, 140, 2);
    drawMenuButton({8, 268, 52, 44}, "Back", TFT_DARKGREY);
    drawMenuButton({64, 268, 84, 44}, mqttCfg.enabled ? "Enabled" : "Disabled", mqttCfg.enabled ? BTN_GLOSS_GREEN : BTN_GLOSS_ORANGE);
    drawMenuButton({152, 268, 80, 44}, "Connect", BTN_GLOSS_BLUE);
    drawMenuButton({20, 158, 200, 32}, "Discover", BTN_GLOSS_CYAN);
    drawMenuButton({20, 194, 96, 32}, "Count -", BTN_GLOSS_ORANGE);
    drawMenuButton({124, 194, 96, 32}, "Count +", BTN_GLOSS_GREEN);
    drawMenuButton({20, 230, 48, 32}, "Prev", BTN_GLOSS_BLUE);
    drawMenuButton({72, 230, 48, 32}, "Next", BTN_GLOSS_BLUE);
    drawMenuButton({124, 230, 48, 32}, "Name+", BTN_GLOSS_PURPLE);
    drawMenuButton({176, 230, 44, 32}, "Crit", mqttButtonCritical[mqttEditButtonIndex] ? BTN_GLOSS_ORANGE : BTN_GLOSS_CYAN);
}

void drawUiConfigMqttControls()
{
    tft.fillScreen(TFT_BLACK);
    drawTopStatus("MQTT", "Controls");
    const int perPage = 4;
    int pages = (mqttButtonCount + perPage - 1) / perPage;
    if (pages < 1) pages = 1;
    if (mqttControlPage >= pages) mqttControlPage = pages - 1;
    const int start = mqttControlPage * perPage;
    for (int i = 0; i < perPage; i++) {
        int idx = start + i;
        int col = i % 2;
        int row = i / 2;
        UiRect r{static_cast<int16_t>(20 + col * 106), static_cast<int16_t>(84 + row * 74), 96, 60};
        if (idx < mqttButtonCount) {
            String lbl = mqttButtonNames[idx];
            if (lbl.length() > 10) lbl = lbl.substring(0, 10) + "...";
            drawMenuButton(r, lbl, BTN_GLOSS_BLUE);
        } else {
            tft.fillRoundRect(r.x, r.y, r.w, r.h, 9, TFT_BLACK);
        }
    }
    drawMenuButton({8, 268, 52, 44}, "Back", TFT_DARKGREY);
    drawMenuButton({64, 268, 84, 44}, "<", BTN_GLOSS_CYAN);
    drawMenuButton({152, 268, 80, 44}, ">", BTN_GLOSS_CYAN);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawCentreString(String(mqttControlPage + 1) + "/" + String(pages), 120, 52, 2);

    if (mqttConfirmVisible && mqttConfirmButtonIndex >= 0 && mqttConfirmButtonIndex < mqttButtonCount) {
        tft.fillRoundRect(26, 122, 188, 108, 10, TFT_DARKGREY);
        tft.drawRoundRect(26, 122, 188, 108, 10, TFT_WHITE);
        String lbl = mqttButtonNames[mqttConfirmButtonIndex];
        if (lbl.length() > 14) lbl = lbl.substring(0, 14) + "...";
        tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
        tft.drawCentreString("Confirm action", 120, 136, 2);
        tft.drawCentreString(lbl, 120, 156, 2);
        drawMenuButton({52, 186, 64, 34}, "No", BTN_GLOSS_BLUE);
        drawMenuButton({124, 186, 64, 34}, "Yes", BTN_GLOSS_ORANGE);
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
    drawUiGameSnake();
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
        drawUiGameTetris();
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
    drawUiGameTetris();
}

void tetrisDrop()
{
    if (tetrisGameOver) return;
    while (tetrisCanPlace(tetrisX, tetrisY + 1, tetrisType, tetrisRot)) tetrisY++;
    tetrisLockPiece();
    drawUiGameTetris();
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
    drawUiGameTetris();
}

void drawUiGameSnake()
{
    tft.fillScreen(TFT_BLACK);
    drawTopStatus("Snake", snakeGameOver ? "Game Over" : "Play");
    tft.drawRect(SNAKE_BOARD_X - 1, SNAKE_BOARD_Y - 1, SNAKE_COLS * SNAKE_CELL + 2, SNAKE_ROWS * SNAKE_CELL + 2, TFT_DARKGREY);
    tft.fillRect(SNAKE_BOARD_X + snakeFoodX * SNAKE_CELL + 3, SNAKE_BOARD_Y + snakeFoodY * SNAKE_CELL + 3, SNAKE_CELL - 6, SNAKE_CELL - 6, TFT_RED);
    for (int i = snakeLen - 1; i >= 0; i--) {
        uint16_t col = (i == 0) ? TFT_GREENYELLOW : TFT_GREEN;
        tft.fillRect(SNAKE_BOARD_X + snakeX[i] * SNAKE_CELL + 1, SNAKE_BOARD_Y + snakeY[i] * SNAKE_CELL + 1, SNAKE_CELL - 2, SNAKE_CELL - 2, col);
    }
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("Score: " + String(snakeScore), 8, 246, 2);
    drawMenuButton({8, 268, 52, 44}, "Back", TFT_DARKGREY);
    drawMenuButton({64, 268, 76, 44}, "Restart", BTN_GLOSS_ORANGE);
    drawMenuButton({148, 268, 40, 44}, "L", BTN_GLOSS_BLUE);
    drawMenuButton({192, 268, 40, 44}, "R", BTN_GLOSS_BLUE);
    drawMenuButton({148, 222, 40, 40}, "U", BTN_GLOSS_CYAN);
    drawMenuButton({192, 222, 40, 40}, "D", BTN_GLOSS_CYAN);
}

void drawUiGameTetris()
{
    static const uint16_t pieceColors[8] = {TFT_BLACK, TFT_CYAN, TFT_BLUE, TFT_ORANGE, TFT_YELLOW, TFT_GREEN, TFT_MAGENTA, TFT_RED};
    tft.fillScreen(TFT_BLACK);
    drawTopStatus("Tetris", tetrisGameOver ? "Game Over" : "Play");
    tft.drawRect(TETRIS_BOARD_X - 1, TETRIS_BOARD_Y - 1, TETRIS_COLS * TETRIS_CELL + 2, TETRIS_ROWS * TETRIS_CELL + 2, TFT_DARKGREY);

    for (int y = 0; y < TETRIS_ROWS; y++) {
        for (int x = 0; x < TETRIS_COLS; x++) {
            uint8_t c = tetrisGrid[y][x];
            if (!c) continue;
            tft.fillRect(TETRIS_BOARD_X + x * TETRIS_CELL + 1, TETRIS_BOARD_Y + y * TETRIS_CELL + 1, TETRIS_CELL - 2, TETRIS_CELL - 2, pieceColors[c]);
        }
    }
    if (!tetrisGameOver) {
        for (int i = 0; i < 4; i++) {
            int ox = 0, oy = 0;
            tetrisCellFor(tetrisType, tetrisRot, i, ox, oy);
            int gx = tetrisX + ox;
            int gy = tetrisY + oy;
            if (gx >= 0 && gx < TETRIS_COLS && gy >= 0 && gy < TETRIS_ROWS) {
                tft.fillRect(TETRIS_BOARD_X + gx * TETRIS_CELL + 1, TETRIS_BOARD_Y + gy * TETRIS_CELL + 1, TETRIS_CELL - 2, TETRIS_CELL - 2,
                             pieceColors[tetrisType + 1]);
            }
        }
    }

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("Score: " + String(tetrisScore), 8, 246, 2);
    drawMenuButton({8, 268, 52, 44}, "Back", TFT_DARKGREY);
    drawMenuButton({64, 268, 76, 44}, "Restart", BTN_GLOSS_ORANGE);
    drawMenuButton({148, 268, 40, 44}, "L", BTN_GLOSS_BLUE);
    drawMenuButton({192, 268, 40, 44}, "R", BTN_GLOSS_BLUE);
    drawMenuButton({148, 222, 40, 40}, "Rot", BTN_GLOSS_CYAN);
    drawMenuButton({192, 222, 40, 40}, "Drop", BTN_GLOSS_CYAN);
}

void drawWifiListPageContent(int page, int16_t yOffset)
{
    const int perPage = 5;
    const int start = page * perPage;
    for (int i = 0; i < perPage; i++) {
        int idx = start + i;
        UiRect row{8, static_cast<int16_t>(52 + i * 42 + yOffset), 224, 36};
        if (row.y + row.h < WIFI_LIST_VIEW_Y || row.y > (WIFI_LIST_VIEW_Y + WIFI_LIST_VIEW_H - 1)) continue;
        uint16_t col = (idx < wifiCount) ? TFT_DARKGREEN : TFT_DARKGREY;
        tft.fillRoundRect(row.x, row.y, row.w, row.h, 6, col);
        tft.drawRoundRect(row.x, row.y, row.w, row.h, 6, TFT_WHITE);
        if (idx < wifiCount) {
            String left = wifiEntries[idx].ssid;
            if (left.length() > 16) left = left.substring(0, 16) + "...";
            String right = String(wifiEntries[idx].rssi) + " " + authName(wifiEntries[idx].auth);
            tft.setTextColor(TFT_WHITE, col);
            tft.drawString(left, row.x + 6, row.y + 6, 2);
            tft.drawRightString(right, row.x + row.w - 6, row.y + 6, 2);
        }
    }
}

void drawWifiListPageContentToSprite(TFT_eSprite &spr, int page, int16_t yOffset)
{
    const int perPage = 5;
    const int start = page * perPage;
    for (int i = 0; i < perPage; i++) {
        int idx = start + i;
        UiRect row{0, static_cast<int16_t>(i * 42 + yOffset), WIFI_LIST_VIEW_W, 36};
        if (row.y + row.h < 0 || row.y > (WIFI_LIST_VIEW_H - 1)) continue;
        uint16_t col = (idx < wifiCount) ? TFT_DARKGREEN : TFT_DARKGREY;
        spr.fillRoundRect(row.x, row.y, row.w, row.h, 6, col);
        spr.drawRoundRect(row.x, row.y, row.w, row.h, 6, TFT_WHITE);
        if (idx < wifiCount) {
            String left = wifiEntries[idx].ssid;
            if (left.length() > 16) left = left.substring(0, 16) + "...";
            String right = String(wifiEntries[idx].rssi) + " " + authName(wifiEntries[idx].auth);
            spr.setTextColor(TFT_WHITE, col);
            spr.drawString(left, row.x + 6, row.y + 6, 2);
            spr.drawRightString(right, row.x + row.w - 6, row.y + 6, 2);
        }
    }
}

void drawUiWifiListChrome()
{
    tft.fillScreen(TFT_BLACK);
    drawTopStatus("Select WiFi", uiStatusLine);
    drawMenuButton({8, 268, 52, 44}, "Back", TFT_DARKGREY);
    drawMenuButton({66, 268, 70, 44}, "Rescan", TFT_DARKCYAN);
    drawMenuButton({142, 268, 42, 44}, "<", TFT_DARKCYAN);
    drawMenuButton({190, 268, 42, 44}, ">", TFT_DARKCYAN);
}

void drawUiWifiList()
{
    drawUiWifiListChrome();
    drawWifiListPageContent(wifiPage, 0);
}

void animateWifiPageTransition(int oldPage, int newPage)
{
    if (oldPage == newPage) return;
    const int dir = (newPage > oldPage) ? 1 : -1;
    const int travel = 210;
    drawUiWifiListChrome();
    TFT_eSprite spr(&tft);
    bool spriteReady = false;
    spr.setColorDepth(16);
    spriteReady = (spr.createSprite(WIFI_LIST_VIEW_W, WIFI_LIST_VIEW_H) != nullptr);

    if (spriteReady) {
        for (int step = 1; step <= SWIPE_ANIM_STEPS; step++) {
            int p = (travel * step * step) / (SWIPE_ANIM_STEPS * SWIPE_ANIM_STEPS);
            int oldOffset = (dir > 0) ? -p : p;
            int newOffset = (dir > 0) ? (travel - p) : (-travel + p);
            spr.fillSprite(TFT_BLACK);
            drawWifiListPageContentToSprite(spr, oldPage, oldOffset);
            drawWifiListPageContentToSprite(spr, newPage, newOffset);
            spr.pushSprite(WIFI_LIST_VIEW_X, WIFI_LIST_VIEW_Y);
            delay(SWIPE_ANIM_FRAME_MS);
        }
        spr.deleteSprite();
    } else {
        tft.startWrite();
        for (int step = 1; step <= SWIPE_ANIM_STEPS; step++) {
            int p = (travel * step * step) / (SWIPE_ANIM_STEPS * SWIPE_ANIM_STEPS);
            int oldOffset = (dir > 0) ? -p : p;
            int newOffset = (dir > 0) ? (travel - p) : (-travel + p);
            tft.fillRect(WIFI_LIST_VIEW_X, WIFI_LIST_VIEW_Y, WIFI_LIST_VIEW_W, WIFI_LIST_VIEW_H, TFT_BLACK);
            drawWifiListPageContent(oldPage, oldOffset);
            drawWifiListPageContent(newPage, newOffset);
            delay(SWIPE_ANIM_FRAME_MS);
        }
        tft.endWrite();
    }
}

void drawUiKeyboard()
{
    static const char *rowsLower[4] = {"1234567890", "qwertyuiop", "asdfghjkl_", "zxcvbnm.-@"};
    static const char *rowsUpper[4] = {"1234567890", "QWERTYUIOP", "ASDFGHJKL_", "ZXCVBNM.-@"};
    const char **rows = kbUpper ? rowsUpper : rowsLower;

    tft.fillScreen(TFT_BLACK);
    drawTopStatus("Password", selectedSsid);

    String shown;
    if (kbShowPassword) {
        shown = typedPassword;
    } else {
        shown.reserve(typedPassword.length());
        for (size_t i = 0; i < typedPassword.length(); i++) shown += '*';
    }
    if (shown.length() > 26) shown = "..." + shown.substring(shown.length() - 26);
    tft.fillRoundRect(8, 50, 224, 30, 5, TFT_NAVY);
    tft.drawRoundRect(8, 50, 224, 30, 5, TFT_WHITE);
    tft.setTextColor(TFT_WHITE, TFT_NAVY);
    tft.drawString(shown, 12, 58, 2);

    const int keyW = 21, keyH = 32, gap = 2, x0 = 6, y0 = 88;
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 10; c++) {
            UiRect k{static_cast<int16_t>(x0 + c * (keyW + gap)), static_cast<int16_t>(y0 + r * (keyH + gap)), keyW, keyH};
            tft.fillRoundRect(k.x, k.y, k.w, k.h, 4, TFT_DARKGREY);
            tft.drawRoundRect(k.x, k.y, k.w, k.h, 4, TFT_WHITE);
            String ch(rows[r][c]);
            tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
            tft.drawCentreString(ch, k.x + (k.w / 2), k.y + 8, 2);
        }
    }

    drawMenuButton({6, 226, 56, 36}, "Back", TFT_DARKGREY);
    drawMenuButton({66, 226, 50, 36}, "Aa", TFT_DARKCYAN);
    drawMenuButton({120, 226, 56, 36}, "Space", TFT_DARKCYAN);
    drawMenuButton({180, 226, 52, 36}, "Del", TFT_RED);

    drawMenuButton({6, 268, 70, 44}, kbShowPassword ? "Hide" : "Show", TFT_DARKCYAN);
    drawMenuButton({82, 268, 70, 44}, "Clear", TFT_RED);
    drawMenuButton({158, 268, 74, 44}, "Connect", TFT_DARKGREEN);
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

void mediaNormalizeDir(String &dir)
{
    if (dir.isEmpty()) dir = "/";
    if (!dir.startsWith("/")) dir = "/" + dir;
    while (dir.length() > 1 && dir.endsWith("/")) dir.remove(dir.length() - 1);
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

void audioSetVolumeImmediate(uint8_t v)
{
    audio.setVolume(v);
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
    audio.stopSong();
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
    if (!sdEnsureMounted(true)) return false;
    audio.stopSong();
    if (!audio.connecttoFS(SD, path.c_str())) {
        sdMarkFault("audioStartFile");
        if (!sdEnsureMounted(true) || !audio.connecttoFS(SD, path.c_str())) return false;
    }
    audioSetVolumeImmediate(AUDIO_VOLUME_TARGET);
    mediaIsPlaying = true;
    mediaPaused = false;
    rgbRefreshByMediaState();
    return true;
}

void audioSetPaused(bool pause)
{
    if (!audio.isRunning()) return;
    if (mediaPaused != pause) audio.pauseResume();
    mediaPaused = pause;
    mediaIsPlaying = !pause;
    rgbRefreshByMediaState();
}

void audioService()
{
    audio.loop();
}

void loadMediaEntries()
{
    mediaCount = 0;
    mediaPage = 0;
    mediaSelected = -1;
    mediaNormalizeDir(mediaCurrentDir);

    if (!sdEnsureMounted(true)) {
        uiStatusLine = "SD unavailable";
        return;
    }
    if (!sdLock()) {
        uiStatusLine = "SD busy";
        return;
    }

    File dir = SD.open(mediaCurrentDir);
    if (!dir || !dir.isDirectory()) {
        uiStatusLine = "Invalid dir: " + mediaCurrentDir;
        if (dir) dir.close();
        sdUnlock();
        return;
    }

    File entry = dir.openNextFile();
    while (entry && mediaCount < MAX_MEDIA_ENTRIES) {
        String full = entry.name();
        bool isDir = entry.isDirectory();
        size_t size = isDir ? 0 : entry.size();
        if (!full.startsWith("/")) {
            full = (mediaCurrentDir == "/") ? ("/" + full) : (mediaCurrentDir + "/" + full);
        }
        String name = full;
        int slash = name.lastIndexOf('/');
        if (slash >= 0) name = name.substring(slash + 1);

        if (isDir || mediaAllowedExt(name)) {
            mediaEntries[mediaCount] = {name, full, isDir, size};
            mediaCount++;
        }
        entry.close();
        entry = dir.openNextFile();
    }
    dir.close();
    sdUnlock();
    uiStatusLine = "Media entries: " + String(mediaCount);
    rgbRefreshByMediaState();
}

void drawMediaPageContent(int page, int16_t yOffset)
{
    const int perPage = 4;
    int start = page * perPage;
    for (int i = 0; i < perPage; i++) {
        int idx = start + i;
        UiRect row{8, static_cast<int16_t>(50 + i * 38 + yOffset), 224, 32};
        if (row.y + row.h < MEDIA_LIST_VIEW_Y || row.y > (MEDIA_LIST_VIEW_Y + MEDIA_LIST_VIEW_H - 1)) continue;
        uint16_t col = (idx < mediaCount) ? (idx == mediaSelected ? TFT_BLUE : TFT_DARKGREY) : TFT_BLACK;
        tft.fillRoundRect(row.x, row.y, row.w, row.h, 5, col);
        tft.drawRoundRect(row.x, row.y, row.w, row.h, 5, TFT_WHITE);
        if (idx < mediaCount) {
            String tag = mediaEntries[idx].isDir ? "[DIR] " : "      ";
            String nm = mediaEntries[idx].name;
            if (nm.length() > 16) nm = nm.substring(0, 16) + "...";
            tft.setTextColor(TFT_WHITE, col);
            tft.drawString(tag + nm, row.x + 4, row.y + 8, 2);
            if (!mediaEntries[idx].isDir) {
                String sz = String(static_cast<int>(mediaEntries[idx].size / 1024)) + "KB";
                tft.drawRightString(sz, row.x + row.w - 4, row.y + 8, 2);
            }
        }
    }
}

void drawMediaPageContentToSprite(TFT_eSprite &spr, int page, int16_t yOffset)
{
    const int perPage = 4;
    int start = page * perPage;
    for (int i = 0; i < perPage; i++) {
        int idx = start + i;
        UiRect row{0, static_cast<int16_t>(i * 38 + yOffset), MEDIA_LIST_VIEW_W, 32};
        if (row.y + row.h < 0 || row.y > (MEDIA_LIST_VIEW_H - 1)) continue;
        uint16_t col = (idx < mediaCount) ? (idx == mediaSelected ? TFT_BLUE : TFT_DARKGREY) : TFT_BLACK;
        spr.fillRoundRect(row.x, row.y, row.w, row.h, 5, col);
        spr.drawRoundRect(row.x, row.y, row.w, row.h, 5, TFT_WHITE);
        if (idx < mediaCount) {
            String tag = mediaEntries[idx].isDir ? "[DIR] " : "      ";
            String nm = mediaEntries[idx].name;
            if (nm.length() > 16) nm = nm.substring(0, 16) + "...";
            spr.setTextColor(TFT_WHITE, col);
            spr.drawString(tag + nm, row.x + 4, row.y + 8, 2);
            if (!mediaEntries[idx].isDir) {
                String sz = String(static_cast<int>(mediaEntries[idx].size / 1024)) + "KB";
                spr.drawRightString(sz, row.x + row.w - 4, row.y + 8, 2);
            }
        }
    }
}

void drawUiMediaChrome()
{
    tft.fillScreen(TFT_BLACK);
    drawTopStatus("MediaPlayer", mediaCurrentDir);
    drawMenuButton({8, 242, 44, 32}, "Up", TFT_DARKCYAN);
    drawMenuButton({56, 242, 44, 32}, "Ref", TFT_DARKCYAN);
    drawMenuButton({104, 242, 40, 32}, "<", TFT_DARKCYAN);
    drawMenuButton({148, 242, 40, 32}, ">", TFT_DARKCYAN);
    drawMenuButton({192, 242, 40, 32}, "Home", TFT_DARKGREY);

    drawMenuButton({8, 278, 52, 34}, "Prev", TFT_MAROON);
    const char *playLabel = mediaIsPlaying ? "Stop" : "Play";
    uint16_t playColor = mediaIsPlaying ? TFT_RED : TFT_DARKGREEN;
    drawMenuButton({64, 278, 108, 34}, playLabel, playColor);
    drawMenuButton({176, 278, 56, 34}, "Next", TFT_MAROON);

    String state = "STOPPED";
    uint16_t stateColor = TFT_RED;
    if (mediaIsPlaying) {
        state = "PLAYING";
        stateColor = TFT_GREEN;
    }
    tft.setTextColor(stateColor, TFT_BLACK);
    tft.drawString("STATE: " + state, 8, 206, 2);

    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    String now = mediaNowPlaying.isEmpty() ? "No track selected" : mediaNowPlaying;
    if (now.length() > 27) now = "..." + now.substring(now.length() - 27);
    tft.drawString(now, 8, 224, 2);
}

void drawUiMedia()
{
    drawUiMediaChrome();
    drawMediaPageContent(mediaPage, 0);
}

void animateMediaPageTransition(int oldPage, int newPage)
{
    if (oldPage == newPage) return;
    const int dir = (newPage > oldPage) ? 1 : -1;
    const int travel = 152;
    drawUiMediaChrome();
    TFT_eSprite spr(&tft);
    bool spriteReady = false;
    spr.setColorDepth(16);
    spriteReady = (spr.createSprite(MEDIA_LIST_VIEW_W, MEDIA_LIST_VIEW_H) != nullptr);

    if (spriteReady) {
        for (int step = 1; step <= SWIPE_ANIM_STEPS; step++) {
            int p = (travel * step * step) / (SWIPE_ANIM_STEPS * SWIPE_ANIM_STEPS);
            int oldOffset = (dir > 0) ? -p : p;
            int newOffset = (dir > 0) ? (travel - p) : (-travel + p);
            spr.fillSprite(TFT_BLACK);
            drawMediaPageContentToSprite(spr, oldPage, oldOffset);
            drawMediaPageContentToSprite(spr, newPage, newOffset);
            spr.pushSprite(MEDIA_LIST_VIEW_X, MEDIA_LIST_VIEW_Y);
            delay(SWIPE_ANIM_FRAME_MS);
        }
        spr.deleteSprite();
    } else {
        tft.startWrite();
        for (int step = 1; step <= SWIPE_ANIM_STEPS; step++) {
            int p = (travel * step * step) / (SWIPE_ANIM_STEPS * SWIPE_ANIM_STEPS);
            int oldOffset = (dir > 0) ? -p : p;
            int newOffset = (dir > 0) ? (travel - p) : (-travel + p);
            tft.fillRect(MEDIA_LIST_VIEW_X, MEDIA_LIST_VIEW_Y, MEDIA_LIST_VIEW_W, MEDIA_LIST_VIEW_H, TFT_BLACK);
            drawMediaPageContent(oldPage, oldOffset);
            drawMediaPageContent(newPage, newOffset);
            delay(SWIPE_ANIM_FRAME_MS);
        }
        tft.endWrite();
    }
}

void drawTopForCurrentScreen()
{
    if (uiScreen == UI_HOME) {
        const String sta = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : String("not connected");
        drawTopStatus("WiFi Manager", "STA: " + sta);
        return;
    }
    if (uiScreen == UI_WIFI_LIST) {
        drawTopStatus("Select WiFi", uiStatusLine);
        return;
    }
    if (uiScreen == UI_KEYBOARD) {
        drawTopStatus("Password", selectedSsid);
        return;
    }
    if (uiScreen == UI_INFO) {
        drawTopStatus("Info", "Device details");
        return;
    }
    if (uiScreen == UI_GAMES) {
        drawTopStatus("Games", "Menu placeholder");
        return;
    }
    if (uiScreen == UI_CONFIG) {
        drawTopStatus("Configuration", "Select submenu");
        return;
    }
    if (uiScreen == UI_CONFIG_STYLE) {
        drawTopStatus("Configuration", "Style");
        return;
    }
    if (uiScreen == UI_CONFIG_MQTT) {
        drawTopStatus("Configuration", "MQTT");
        return;
    }
    if (uiScreen == UI_CONFIG_MQTT_CONFIG) {
        drawTopStatus("MQTT", "Config");
        return;
    }
    if (uiScreen == UI_CONFIG_MQTT_CONTROLS) {
        drawTopStatus("MQTT", "Controls");
        return;
    }
    if (uiScreen == UI_GAME_SNAKE) {
        drawTopStatus("Snake", snakeGameOver ? "Game Over" : "Play");
        return;
    }
    if (uiScreen == UI_GAME_TETRIS) {
        drawTopStatus("Tetris", tetrisGameOver ? "Game Over" : "Play");
        return;
    }
    drawTopStatus("MediaPlayer", mediaCurrentDir);
}

void displaySetAwake(bool awake)
{
    displayAwake = awake;
    digitalWrite(TFT_BL, awake ? HIGH : LOW);
    if (awake) {
        rgbRefreshByMediaState();
        redrawUi();
    } else {
        if (!batteryCharging) rgbApplyNow(false, false, false);
    }
}

bool canEnterLowPowerSleep(bool touchDownNow)
{
    if (touchDownNow) return false;
    if (WiFi.status() == WL_CONNECTED) return false; // requested: sleep only when not connected
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
    gpio_wakeup_enable(static_cast<gpio_num_t>(TOUCH_IRQ), GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();
    // Optional safety wake in case touch IRQ is not wired on this board revision.
    if (LIGHT_SLEEP_TIMER_FALLBACK) esp_sleep_enable_timer_wakeup(LIGHT_SLEEP_TIMER_US);
    esp_light_sleep_start();
}

void redrawUi()
{
    if (uiScreen == UI_HOME) drawUiHome();
    else if (uiScreen == UI_WIFI_LIST) drawUiWifiList();
    else if (uiScreen == UI_KEYBOARD) drawUiKeyboard();
    else if (uiScreen == UI_MEDIA) drawUiMedia();
    else if (uiScreen == UI_INFO) drawUiInfo();
    else if (uiScreen == UI_GAMES) drawUiGames();
    else if (uiScreen == UI_CONFIG) drawUiConfig();
    else if (uiScreen == UI_CONFIG_STYLE) drawUiConfigStyle();
    else if (uiScreen == UI_CONFIG_MQTT) drawUiConfigMqtt();
    else if (uiScreen == UI_CONFIG_MQTT_CONFIG) drawUiConfigMqttConfig();
    else if (uiScreen == UI_CONFIG_MQTT_CONTROLS) drawUiConfigMqttControls();
    else if (uiScreen == UI_GAME_SNAKE) drawUiGameSnake();
    else drawUiGameTetris();
    if (uiPopupActive) renderTransientPopupOverlay();
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
    if (WiFi.status() != WL_CONNECTED) {
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
    if (!mqttCfg.enabled) {
        if (mqttClient.connected()) mqttClient.disconnect();
        return;
    }
    if (WiFi.status() != WL_CONNECTED) {
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
    if (savedStaSsid.isEmpty()) {
        uiStatusLine = "AP ready (no saved STA)";
        return;
    }
    uiStatusLine = "Boot reconnect: " + savedStaSsid;
    WiFi.begin(savedStaSsid.c_str(), savedStaPass.c_str());
    bootStaConnectInProgress = true;
    bootStaConnectStartedMs = millis();
}

void startWifiConnect(const String &ssid, const String &pass)
{
    uiStatusLine = "Connecting to " + ssid;
    pendingSaveSsid = ssid;
    pendingSavePass = pass;
    pendingSaveCreds = true;
    bootStaConnectInProgress = false;
    WiFi.begin(ssid.c_str(), pass.c_str());
    uiScreen = UI_HOME;
    redrawUi();
}

int homeMenuIndexAtPoint(int16_t x, int16_t y, int &slotOut)
{
    slotOut = -1;
    if (x < HOME_MENU_X || x >= (HOME_MENU_X + HOME_MENU_W)) return -1;
    for (int i = 0; i < HOME_MENU_PER_PAGE; i++) {
        UiRect row{HOME_MENU_X, static_cast<int16_t>(HOME_MENU_Y + i * (HOME_MENU_BUTTON_H + HOME_MENU_GAP)), HOME_MENU_W, HOME_MENU_BUTTON_H};
        if (hitRect(row, x, y)) {
            slotOut = i;
            int idx = homeMenuPage * HOME_MENU_PER_PAGE + i;
            if (idx < homeMenuItemCount()) return idx;
            return -1;
        }
    }
    return -1;
}

void animateHomeButtonPress(int pageSlot, int itemIndex)
{
    if (pageSlot < 0 || pageSlot >= HOME_MENU_PER_PAGE) return;
    if (itemIndex < 0 || itemIndex >= homeMenuItemCount()) return;
    drawUiHomeChrome();
    drawHomeMenuPageContent(homeMenuPage, 0, pageSlot);
    delay(70);
    drawUiHomeChrome();
    drawHomeMenuPageContent(homeMenuPage, 0, -1);
    delay(18);
}

void handleTouchHome(int16_t x, int16_t y)
{
    int slot = -1;
    int idx = homeMenuIndexAtPoint(x, y, slot);
    if (idx < 0) return;
    animateHomeButtonPress(slot, idx);

    if (strcmp(HOME_MENU_ITEMS[idx].label, "Web Recovery") == 0) {
        uiStatusLine = "Open /upload in browser";
        redrawUi();
        return;
    }

    uiScreen = HOME_MENU_ITEMS[idx].target;
    if (uiScreen == UI_WIFI_LIST) {
        refreshWifiScan();
    } else if (uiScreen == UI_MEDIA) {
        uiScreen = UI_MEDIA;
        loadMediaEntries();
    }
    redrawUi();
}

void handleTouchInfo(int16_t x, int16_t y)
{
    if (hitRect({8, 268, 52, 44}, x, y)) {
        animateUiButtonPress({8, 268, 52, 44}, "Back", TFT_DARKGREY);
        uiScreen = UI_HOME;
        redrawUi();
    }
}

void handleTouchGames(int16_t x, int16_t y)
{
    if (hitRect({8, 268, 52, 44}, x, y)) {
        animateUiButtonPress({8, 268, 52, 44}, "Back", TFT_DARKGREY);
        uiScreen = UI_HOME;
        redrawUi();
        return;
    }
    if (hitRect({20, 92, 200, 52}, x, y)) {
        animateUiButtonPress({20, 92, 200, 52}, "Snake", BTN_GLOSS_GREEN);
        uiScreen = UI_GAME_SNAKE;
        snakeResetGame();
        redrawUi();
        return;
    }
    if (hitRect({20, 158, 200, 52}, x, y)) {
        animateUiButtonPress({20, 158, 200, 52}, "Tetris", BTN_GLOSS_BLUE);
        uiScreen = UI_GAME_TETRIS;
        tetrisResetGame();
        redrawUi();
        return;
    }
}

void handleTouchGameSnake(int16_t x, int16_t y)
{
    if (hitRect({8, 268, 52, 44}, x, y)) {
        animateUiButtonPress({8, 268, 52, 44}, "Back", TFT_DARKGREY);
        uiScreen = UI_GAMES;
        redrawUi();
        return;
    }
    if (hitRect({64, 268, 76, 44}, x, y)) {
        animateUiButtonPress({64, 268, 76, 44}, "Restart", BTN_GLOSS_ORANGE);
        snakeResetGame();
        redrawUi();
        return;
    }
    if (hitRect({148, 268, 40, 44}, x, y) && snakeDir != 1) snakeNextDir = 3;
    else if (hitRect({192, 268, 40, 44}, x, y) && snakeDir != 3) snakeNextDir = 1;
    else if (hitRect({148, 222, 40, 40}, x, y) && snakeDir != 2) snakeNextDir = 0;
    else if (hitRect({192, 222, 40, 40}, x, y) && snakeDir != 0) snakeNextDir = 2;
}

void handleTouchGameTetris(int16_t x, int16_t y)
{
    if (hitRect({8, 268, 52, 44}, x, y)) {
        animateUiButtonPress({8, 268, 52, 44}, "Back", TFT_DARKGREY);
        uiScreen = UI_GAMES;
        redrawUi();
        return;
    }
    if (hitRect({64, 268, 76, 44}, x, y)) {
        animateUiButtonPress({64, 268, 76, 44}, "Restart", BTN_GLOSS_ORANGE);
        tetrisResetGame();
        redrawUi();
        return;
    }
    if (hitRect({148, 268, 40, 44}, x, y)) {
        tetrisMove(-1);
        return;
    }
    if (hitRect({192, 268, 40, 44}, x, y)) {
        tetrisMove(1);
        return;
    }
    if (hitRect({148, 222, 40, 40}, x, y)) {
        tetrisRotate();
        return;
    }
    if (hitRect({192, 222, 40, 40}, x, y)) {
        tetrisDrop();
        return;
    }
}

void handleTouchConfig(int16_t x, int16_t y)
{
    if (hitRect({8, 268, 52, 44}, x, y)) {
        animateUiButtonPress({8, 268, 52, 44}, "Back", TFT_DARKGREY);
        uiScreen = UI_HOME;
        redrawUi();
        return;
    }
    if (hitRect({20, 92, 200, 52}, x, y)) {
        animateUiButtonPress({20, 92, 200, 52}, "Style", BTN_GLOSS_CYAN);
        uiScreen = UI_CONFIG_STYLE;
        redrawUi();
        return;
    }
    if (hitRect({20, 158, 200, 52}, x, y)) {
        animateUiButtonPress({20, 158, 200, 52}, "MQTT", BTN_GLOSS_BLUE);
        uiScreen = UI_CONFIG_MQTT;
        redrawUi();
        return;
    }
}

void handleTouchConfigStyle(int16_t x, int16_t y)
{
    if (hitRect({8, 268, 52, 44}, x, y)) {
        animateUiButtonPress({8, 268, 52, 44}, "Back", TFT_DARKGREY);
        uiScreen = UI_CONFIG;
        redrawUi();
    }
}

void handleTouchConfigMqtt(int16_t x, int16_t y)
{
    if (hitRect({8, 268, 52, 44}, x, y)) {
        animateUiButtonPress({8, 268, 52, 44}, "Back", TFT_DARKGREY);
        uiScreen = UI_CONFIG;
        redrawUi();
        return;
    }
    if (hitRect({20, 102, 200, 60}, x, y)) {
        animateUiButtonPress({20, 102, 200, 60}, "Config", BTN_GLOSS_CYAN);
        uiScreen = UI_CONFIG_MQTT_CONFIG;
        redrawUi();
        return;
    }
    if (hitRect({20, 176, 200, 60}, x, y)) {
        animateUiButtonPress({20, 176, 200, 60}, "Controls", BTN_GLOSS_BLUE);
        uiScreen = UI_CONFIG_MQTT_CONTROLS;
        mqttConfirmVisible = false;
        mqttConfirmButtonIndex = -1;
        redrawUi();
        return;
    }
}

void handleTouchConfigMqttConfig(int16_t x, int16_t y)
{
    if (hitRect({8, 268, 52, 44}, x, y)) {
        animateUiButtonPress({8, 268, 52, 44}, "Back", TFT_DARKGREY);
        uiScreen = UI_CONFIG_MQTT;
        redrawUi();
        return;
    }
    if (hitRect({64, 268, 84, 44}, x, y)) {
        animateUiButtonPress({64, 268, 84, 44}, mqttCfg.enabled ? "Enabled" : "Disabled",
                             mqttCfg.enabled ? BTN_GLOSS_GREEN : BTN_GLOSS_ORANGE);
        mqttCfg.enabled = !mqttCfg.enabled;
        saveMqttConfig();
        if (!mqttCfg.enabled) {
            if (mqttClient.connected()) mqttClient.disconnect();
            mqttStatusLine = "Disabled";
        } else {
            mqttStatusLine = "Enabled";
            mqttConnectNow();
        }
        redrawUi();
        return;
    }
    if (hitRect({152, 268, 80, 44}, x, y)) {
        animateUiButtonPress({152, 268, 80, 44}, "Connect", BTN_GLOSS_BLUE);
        mqttConnectNow();
        redrawUi();
        return;
    }
    if (hitRect({20, 158, 200, 32}, x, y)) {
        animateUiButtonPress({20, 158, 200, 32}, "Discover", BTN_GLOSS_CYAN);
        if (mqttClient.connected()) mqttPublishDiscovery();
        else mqttStatusLine = "Not connected";
        redrawUi();
        return;
    }
    if (hitRect({20, 194, 96, 32}, x, y)) {
        if (mqttButtonCount > 1) {
            mqttButtonCount--;
            if (mqttEditButtonIndex >= mqttButtonCount) mqttEditButtonIndex = mqttButtonCount - 1;
            if (mqttConfirmButtonIndex >= mqttButtonCount) {
                mqttConfirmButtonIndex = -1;
                mqttConfirmVisible = false;
            }
            mqttDiscoveryPublished = false;
            saveMqttConfig();
            mqttStatusLine = "Button count: " + String(mqttButtonCount);
        }
        redrawUi();
        return;
    }
    if (hitRect({124, 194, 96, 32}, x, y)) {
        if (mqttButtonCount < MQTT_MAX_BUTTONS) {
            mqttButtonNames[mqttButtonCount] = mqttDefaultButtonName(mqttButtonCount);
            mqttButtonCritical[mqttButtonCount] = false;
            mqttButtonCount++;
            mqttDiscoveryPublished = false;
            saveMqttConfig();
            mqttStatusLine = "Button count: " + String(mqttButtonCount);
        }
        redrawUi();
        return;
    }
    if (hitRect({20, 230, 48, 32}, x, y)) {
        if (mqttEditButtonIndex > 0) mqttEditButtonIndex--;
        redrawUi();
        return;
    }
    if (hitRect({72, 230, 48, 32}, x, y)) {
        if (mqttEditButtonIndex + 1 < mqttButtonCount) mqttEditButtonIndex++;
        redrawUi();
        return;
    }
    if (hitRect({124, 230, 48, 32}, x, y)) {
        static const char *presets[] = {"up", "down", "left", "right", "ok", "back", "play", "pause", "power", "menu", "vol_up", "vol_down",
                                        "ch_up", "ch_down"};
        const int n = static_cast<int>(sizeof(presets) / sizeof(presets[0]));
        int cur = -1;
        for (int i = 0; i < n; i++) {
            if (mqttButtonNames[mqttEditButtonIndex].equalsIgnoreCase(presets[i])) {
                cur = i;
                break;
            }
        }
        cur = (cur + 1) % n;
        mqttButtonNames[mqttEditButtonIndex] = presets[cur];
        mqttDiscoveryPublished = false;
        saveMqttConfig();
        mqttStatusLine = "Name #" + String(mqttEditButtonIndex + 1) + ": " + mqttButtonNames[mqttEditButtonIndex];
        redrawUi();
        return;
    }
    if (hitRect({176, 230, 44, 32}, x, y)) {
        mqttButtonCritical[mqttEditButtonIndex] = !mqttButtonCritical[mqttEditButtonIndex];
        saveMqttConfig();
        mqttStatusLine = "Critical #" + String(mqttEditButtonIndex + 1) + ": " + String(mqttButtonCritical[mqttEditButtonIndex] ? "ON" : "OFF");
        redrawUi();
        return;
    }
}

void handleTouchConfigMqttControls(int16_t x, int16_t y)
{
    if (mqttConfirmVisible) {
        if (hitRect({52, 186, 64, 34}, x, y)) {
            mqttConfirmVisible = false;
            mqttConfirmButtonIndex = -1;
            mqttStatusLine = "Canceled";
            redrawUi();
            return;
        }
        if (hitRect({124, 186, 64, 34}, x, y)) {
            const int idx = mqttConfirmButtonIndex;
            mqttConfirmVisible = false;
            mqttConfirmButtonIndex = -1;
            mqttPublishButtonAction(idx);
            mqttStatusLine = "Action: " + mqttButtonPayloadForIndex(idx);
            redrawUi();
            return;
        }
        return;
    }
    if (hitRect({8, 268, 52, 44}, x, y)) {
        animateUiButtonPress({8, 268, 52, 44}, "Back", TFT_DARKGREY);
        uiScreen = UI_CONFIG_MQTT;
        redrawUi();
        return;
    }
    const int perPage = 4;
    int pages = (mqttButtonCount + perPage - 1) / perPage;
    if (pages < 1) pages = 1;
    if (hitRect({64, 268, 84, 44}, x, y)) {
        if (mqttControlPage > 0) mqttControlPage--;
        redrawUi();
        return;
    }
    if (hitRect({152, 268, 80, 44}, x, y)) {
        if (mqttControlPage + 1 < pages) mqttControlPage++;
        redrawUi();
        return;
    }
    int start = mqttControlPage * perPage;
    for (int i = 0; i < perPage; i++) {
        int idx = start + i;
        if (idx >= mqttButtonCount) continue;
        int col = i % 2;
        int row = i / 2;
        UiRect r{static_cast<int16_t>(20 + col * 106), static_cast<int16_t>(84 + row * 74), 96, 60};
        if (hitRect(r, x, y)) {
            String label = mqttButtonNames[idx];
            if (label.length() > 10) label = label.substring(0, 10) + "...";
            animateUiButtonPress(r, label, BTN_GLOSS_BLUE);
            if (mqttButtonCritical[idx]) {
                mqttConfirmVisible = true;
                mqttConfirmButtonIndex = idx;
                mqttStatusLine = "Confirm required";
            } else {
                mqttPublishButtonAction(idx);
                mqttStatusLine = "Action: " + mqttButtonPayloadForIndex(idx);
            }
            redrawUi();
            return;
        }
    }
}

void handleTouchWifiList(int16_t x, int16_t y)
{
    const int perPage = 5;
    for (int i = 0; i < perPage; i++) {
        UiRect row{8, static_cast<int16_t>(52 + i * 42), 224, 36};
        if (hitRect(row, x, y)) {
            int idx = wifiPage * perPage + i;
            if (idx < wifiCount) {
                selectedSsid = wifiEntries[idx].ssid;
                if (wifiEntries[idx].auth == WIFI_AUTH_OPEN) {
                    typedPassword = "";
                    startWifiConnect(selectedSsid, typedPassword);
                } else {
                    typedPassword = "";
                    kbUpper = false;
                    kbShowPassword = false;
                    uiScreen = UI_KEYBOARD;
                    redrawUi();
                }
            }
            return;
        }
    }

    if (hitRect({8, 268, 52, 44}, x, y)) {
        animateUiButtonPress({8, 268, 52, 44}, "Back", TFT_DARKGREY);
        uiScreen = UI_HOME;
        redrawUi();
        return;
    }
    if (hitRect({66, 268, 70, 44}, x, y)) {
        refreshWifiScan();
        redrawUi();
        return;
    }
    if (hitRect({142, 268, 42, 44}, x, y)) {
        applyWifiListScrollDelta(-1);
        return;
    }
    if (hitRect({190, 268, 42, 44}, x, y)) {
        applyWifiListScrollDelta(+1);
    }
}

void handleTouchKeyboard(int16_t x, int16_t y)
{
    static const char *rowsLower[4] = {"1234567890", "qwertyuiop", "asdfghjkl_", "zxcvbnm.-@"};
    static const char *rowsUpper[4] = {"1234567890", "QWERTYUIOP", "ASDFGHJKL_", "ZXCVBNM.-@"};
    const char **rows = kbUpper ? rowsUpper : rowsLower;

    const int keyW = 21, keyH = 32, gap = 2, x0 = 6, y0 = 88;
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 10; c++) {
            UiRect k{static_cast<int16_t>(x0 + c * (keyW + gap)), static_cast<int16_t>(y0 + r * (keyH + gap)), keyW, keyH};
            if (hitRect(k, x, y)) {
                typedPassword += rows[r][c];
                drawUiKeyboard();
                return;
            }
        }
    }

    if (hitRect({6, 226, 56, 36}, x, y)) { animateUiButtonPress({6, 226, 56, 36}, "Back", TFT_DARKGREY); uiScreen = UI_WIFI_LIST; redrawUi(); return; }
    if (hitRect({66, 226, 50, 36}, x, y)) { kbUpper = !kbUpper; drawUiKeyboard(); return; }
    if (hitRect({120, 226, 56, 36}, x, y)) { typedPassword += ' '; drawUiKeyboard(); return; }
    if (hitRect({180, 226, 52, 36}, x, y)) {
        if (typedPassword.length()) typedPassword.remove(typedPassword.length() - 1);
        drawUiKeyboard();
        return;
    }
    if (hitRect({6, 268, 70, 44}, x, y)) { kbShowPassword = !kbShowPassword; drawUiKeyboard(); return; }
    if (hitRect({82, 268, 70, 44}, x, y)) { typedPassword = ""; drawUiKeyboard(); return; }
    if (hitRect({158, 268, 74, 44}, x, y)) { startWifiConnect(selectedSsid, typedPassword); }
}

void mediaStartSelected()
{
    if (fsWriteBusy()) {
        uiStatusLine = "Storage busy";
        return;
    }
    if (mediaSelected < 0 || mediaSelected >= mediaCount) return;
    if (mediaEntries[mediaSelected].isDir) return;
    mediaNowPlaying = mediaEntries[mediaSelected].path;

    String playPath = mediaResolvePlaybackPath(mediaEntries[mediaSelected].path);
    if (playPath.isEmpty()) {
        uiStatusLine = "Alias remap failed";
        Serial.println("[Media] alias remap failed: " + mediaEntries[mediaSelected].path);
        return;
    }
    mediaPlaybackPath = playPath;

    if (audioStartFile(playPath)) {
        uiStatusLine = "Playing: " + mediaEntries[mediaSelected].name;
        Serial.println("[Media] play: " + mediaEntries[mediaSelected].path + " -> " + playPath);
    } else {
        audioStopPlayback();
        uiStatusLine = "Decode/open failed";
        Serial.println("[Media] decode/open failed");
    }
}

void mediaSelectRelative(int delta)
{
    if (mediaCount == 0) return;
    int idx = mediaSelected;
    if (idx < 0) idx = 0;
    idx += delta;
    if (idx < 0) idx = mediaCount - 1;
    if (idx >= mediaCount) idx = 0;
    mediaSelected = idx;
    if (!mediaEntries[mediaSelected].isDir) mediaNowPlaying = mediaEntries[mediaSelected].path;
    rgbRefreshByMediaState();
}

bool mediaSelectPlayableRelative(int delta)
{
    if (mediaCount == 0) return false;
    int idx = mediaSelected;
    for (int tries = 0; tries < mediaCount; tries++) {
        idx += delta;
        if (idx < 0) idx = mediaCount - 1;
        if (idx >= mediaCount) idx = 0;
        if (!mediaEntries[idx].isDir) {
            mediaSelected = idx;
            mediaNowPlaying = mediaEntries[idx].path;
            rgbRefreshByMediaState();
            return true;
        }
    }
    return false;
}

void mediaGoParentDir()
{
    if (mediaCurrentDir == "/") return;
    int slash = mediaCurrentDir.lastIndexOf('/');
    if (slash <= 0) mediaCurrentDir = "/";
    else mediaCurrentDir = mediaCurrentDir.substring(0, slash);
    loadMediaEntries();
}

void handleTouchMedia(int16_t x, int16_t y)
{
    const int perPage = 4;
    for (int i = 0; i < perPage; i++) {
        UiRect row{8, static_cast<int16_t>(50 + i * 38), 224, 32};
        if (hitRect(row, x, y)) {
            int idx = mediaPage * perPage + i;
            if (idx < mediaCount) {
                mediaSelected = idx;
                if (mediaEntries[idx].isDir) {
                    mediaCurrentDir = mediaEntries[idx].path;
                    loadMediaEntries();
                } else {
                    mediaNowPlaying = mediaEntries[idx].path;
                    mediaStartSelected();
                }
                rgbRefreshByMediaState();
            }
            redrawUi();
            return;
        }
    }

    if (hitRect({8, 242, 44, 32}, x, y)) { mediaGoParentDir(); redrawUi(); return; }
    if (hitRect({56, 242, 44, 32}, x, y)) { loadMediaEntries(); redrawUi(); return; }
    if (hitRect({104, 242, 40, 32}, x, y)) {
        applyMediaListScrollDelta(-1);
        return;
    }
    if (hitRect({148, 242, 40, 32}, x, y)) {
        applyMediaListScrollDelta(+1);
        return;
    }
    if (hitRect({192, 242, 40, 32}, x, y)) { animateUiButtonPress({192, 242, 40, 32}, "Home", TFT_DARKGREY); uiScreen = UI_HOME; redrawUi(); return; }

    if (hitRect({8, 278, 52, 34}, x, y)) {
        if (mediaSelectPlayableRelative(-1)) {
            mediaStartSelected();
        } else {
            uiStatusLine = "No playable files";
        }
        redrawUi();
        return;
    }
    if (hitRect({64, 278, 108, 34}, x, y)) {
        rgbFlashAcknowledge();
        if (!mediaIsPlaying) {
            if (mediaSelected < 0 || mediaEntries[mediaSelected].isDir) {
                if (!mediaSelectPlayableRelative(+1)) {
                    uiStatusLine = "No playable files";
                    redrawUi();
                    return;
                }
            }
            mediaStartSelected();
        }
        else {
            audioStopPlayback(true);
            uiStatusLine = "Stopped";
            Serial.println("[Media] stop");
        }
        redrawUi();
        return;
    }
    if (hitRect({176, 278, 56, 34}, x, y)) {
        if (mediaSelectPlayableRelative(+1)) {
            mediaStartSelected();
        } else {
            uiStatusLine = "No playable files";
        }
        redrawUi();
        return;
    }
}

bool applyWifiListScrollDelta(int deltaPages)
{
    const int perPage = 5;
    const int pages = (wifiCount + perPage - 1) / perPage;
    const int maxPage = (pages > 0) ? (pages - 1) : 0;
    const int oldPage = wifiPage;
    int nextPage = wifiPage + deltaPages;
    if (nextPage < 0) nextPage = 0;
    if (nextPage > maxPage) nextPage = maxPage;
    if (nextPage == oldPage) return false;
    animateWifiPageTransition(oldPage, nextPage);
    wifiPage = nextPage;
    drawUiWifiList();
    return true;
}

bool applyHomeMenuScrollDelta(int deltaPages)
{
    const int pages = homeMenuPageCount();
    const int maxPage = (pages > 0) ? (pages - 1) : 0;
    const int oldPage = homeMenuPage;
    int nextPage = homeMenuPage + deltaPages;
    if (nextPage < 0) nextPage = 0;
    if (nextPage > maxPage) nextPage = maxPage;
    if (nextPage == oldPage) return false;
    animateHomeMenuTransition(oldPage, nextPage);
    homeMenuPage = nextPage;
    drawUiHome();
    return true;
}

bool applyMediaListScrollDelta(int deltaPages)
{
    const int perPage = 4;
    const int pages = (mediaCount + perPage - 1) / perPage;
    const int maxPage = (pages > 0) ? (pages - 1) : 0;
    const int oldPage = mediaPage;
    int nextPage = mediaPage + deltaPages;
    if (nextPage < 0) nextPage = 0;
    if (nextPage > maxPage) nextPage = maxPage;
    if (nextPage == oldPage) return false;
    animateMediaPageTransition(oldPage, nextPage);
    mediaPage = nextPage;
    drawUiMedia();
    return true;
}

void handleVerticalSwipeGesture(int direction)
{
    // direction: +1 = swipe up, -1 = swipe down
    if (uiScreen == UI_HOME) applyHomeMenuScrollDelta(direction);
    else if (uiScreen == UI_WIFI_LIST) applyWifiListScrollDelta(direction);
    else if (uiScreen == UI_MEDIA) applyMediaListScrollDelta(direction);
}

void handleHorizontalSwipeGesture(int direction)
{
    // direction: -1 = left swipe, +1 = right swipe
    if (direction < 0 && isMenuScreenForScreenshot()) triggerScreenshotCapture();
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

void renderTransientPopupOverlay()
{
    if (!uiPopupActive) return;
    tft.fillRoundRect(18, 118, 204, 86, 10, uiPopupColor);
    tft.drawRoundRect(18, 118, 204, 86, 10, TFT_WHITE);
    tft.setTextColor(TFT_WHITE, uiPopupColor);
    String t = uiPopupTitle;
    String l = uiPopupLine;
    if (t.length() > 18) t = t.substring(0, 18) + "...";
    if (l.length() > 22) l = l.substring(0, 22) + "...";
    tft.drawCentreString(t, 120, 136, 2);
    tft.drawCentreString(l, 120, 160, 2);
}

void showTransientPopup(const String &title, const String &line, uint16_t color = TFT_DARKGREY, unsigned long ms = 900)
{
    uiPopupTitle = title;
    uiPopupLine = line;
    uiPopupColor = color;
    uiPopupActive = true;
    uiPopupUntilMs = millis() + ms;
    renderTransientPopupOverlay();
}

void popupService()
{
    if (!uiPopupActive) return;
    if (static_cast<long>(millis() - uiPopupUntilMs) >= 0) {
        uiPopupActive = false;
        redrawUi();
    }
}

bool isMenuScreenForScreenshot()
{
    return uiScreen != UI_GAME_SNAKE && uiScreen != UI_GAME_TETRIS;
}

bool captureScreenToSdBmp(String &savedPathOut, String &errorOut)
{
    savedPathOut = "";
    errorOut = "";
    if (!sdEnsureMounted()) { errorOut = "no_sd"; return false; }
    if (mediaIsPlaying || mediaPaused) { errorOut = "media_busy"; return false; }
    if (!sdLock()) { errorOut = "sd_lock"; return false; }

    const int w = tft.width();
    const int h = tft.height();
    const int rowBytes = (w * 3 + 3) & ~3;
    const uint32_t pixelDataSize = static_cast<uint32_t>(rowBytes * h);
    const uint32_t fileSize = 54 + pixelDataSize;

    if (!SD.exists("/Screenshots") && !SD.mkdir("/Screenshots")) {
        sdUnlock();
        errorOut = "mkdir_failed";
        return false;
    }

    String path;
    for (int i = 1; i <= 9999; i++) {
        char name[40];
        snprintf(name, sizeof(name), "/Screenshots/shot_%04d.bmp", i);
        if (!SD.exists(name)) {
            path = String(name);
            break;
        }
    }
    if (path.isEmpty()) {
        sdUnlock();
        errorOut = "name_exhausted";
        return false;
    }

    File f = SD.open(path, FILE_WRITE);
    if (!f) {
        sdMarkFault("captureScreenToSdBmp/open");
        sdUnlock();
        errorOut = "open_failed";
        return false;
    }

    auto w8 = [&](uint8_t v) { f.write(&v, 1); };
    auto w16 = [&](uint16_t v) {
        w8(static_cast<uint8_t>(v & 0xFF));
        w8(static_cast<uint8_t>((v >> 8) & 0xFF));
    };
    auto w32 = [&](uint32_t v) {
        w8(static_cast<uint8_t>(v & 0xFF));
        w8(static_cast<uint8_t>((v >> 8) & 0xFF));
        w8(static_cast<uint8_t>((v >> 16) & 0xFF));
        w8(static_cast<uint8_t>((v >> 24) & 0xFF));
    };

    // BMP header (54 bytes)
    w8('B');
    w8('M');
    w32(fileSize);
    w16(0);
    w16(0);
    w32(54);
    w32(40);
    w32(static_cast<uint32_t>(w));
    w32(static_cast<uint32_t>(h));
    w16(1);
    w16(24);
    w32(0);
    w32(pixelDataSize);
    w32(2835);
    w32(2835);
    w32(0);
    w32(0);

    static constexpr int SHOT_ROWS_PER_CHUNK = 8;
    uint16_t *row565 = reinterpret_cast<uint16_t *>(malloc(static_cast<size_t>(w) * sizeof(uint16_t)));
    uint8_t *chunkBgr = reinterpret_cast<uint8_t *>(malloc(static_cast<size_t>(rowBytes) * SHOT_ROWS_PER_CHUNK));
    if (!row565 || !chunkBgr) {
        if (row565) free(row565);
        if (chunkBgr) free(chunkBgr);
        f.close();
        sdUnlock();
        errorOut = "oom";
        return false;
    }

    const unsigned long startedMs = millis();
    bool writeFailed = false;
    int chunkRows = 0;
    int chunkOffset = 0;
    for (int y = h - 1; y >= 0; y--) {
        if (millis() - startedMs > 4000UL) {
            errorOut = "timeout";
            writeFailed = true;
            break;
        }
        tft.readRect(0, y, w, 1, row565);
        int o = chunkOffset;
        for (int x = 0; x < w; x++) {
            uint16_t c = row565[x];
            uint8_t r = static_cast<uint8_t>(((c >> 11) & 0x1F) * 255 / 31);
            uint8_t g = static_cast<uint8_t>(((c >> 5) & 0x3F) * 255 / 63);
            uint8_t b = static_cast<uint8_t>((c & 0x1F) * 255 / 31);
            chunkBgr[o++] = b;
            chunkBgr[o++] = g;
            chunkBgr[o++] = r;
        }
        while (o < (chunkOffset + rowBytes)) chunkBgr[o++] = 0;
        chunkRows++;
        chunkOffset += rowBytes;
        bool flushNow = (chunkRows >= SHOT_ROWS_PER_CHUNK) || (y == 0);
        if (flushNow) {
            size_t target = static_cast<size_t>(chunkOffset);
            size_t wr = f.write(chunkBgr, target);
            if (wr != target) {
                errorOut = "write_error";
                writeFailed = true;
                break;
            }
            chunkRows = 0;
            chunkOffset = 0;
        }
        if ((y & 0x0F) == 0) delay(0);
    }

    bool hadWriteErr = f.getWriteError() != 0;
    if (!writeFailed) f.flush();
    free(row565);
    free(chunkBgr);
    f.close();
    sdUnlock();
    if (writeFailed || hadWriteErr) {
        sdMarkFault("captureScreenToSdBmp/write");
        if (errorOut.isEmpty()) errorOut = hadWriteErr ? "write_error" : "verify_failed";
        if (SD.exists(path)) SD.remove(path);
        return false;
    }
    savedPathOut = path;
    return true;
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

bool writeScreenshotDiagnostic(String &savedPathOut, String &errorOut)
{
    savedPathOut = "";
    errorOut = "";
    if (!sdEnsureMounted()) { errorOut = "no_sd"; return false; }
    if (!sdLock()) { errorOut = "sd_lock"; return false; }

    if (!SD.exists("/Screenshots") && !SD.mkdir("/Screenshots")) {
        sdUnlock();
        errorOut = "mkdir_failed";
        return false;
    }

    String path;
    for (int i = 1; i <= 9999; i++) {
        char name[44];
        snprintf(name, sizeof(name), "/Screenshots/diag_%04d.txt", i);
        if (!SD.exists(name)) {
            path = String(name);
            break;
        }
    }
    if (path.isEmpty()) {
        sdUnlock();
        errorOut = "name_exhausted";
        return false;
    }

    File f = SD.open(path, FILE_WRITE);
    if (!f) {
        sdMarkFault("writeScreenshotDiagnostic/open");
        sdUnlock();
        errorOut = "open_failed";
        return false;
    }

    f.println("type=screenshot_diagnostic");
    f.println("millis=" + String(millis()));
    f.println("ui_screen=" + String(static_cast<int>(uiScreen)));
    f.println("wifi_connected=" + String(WiFi.status() == WL_CONNECTED ? 1 : 0));
    f.println("free_heap=" + String(ESP.getFreeHeap()));
    f.flush();

    bool hadErr = f.getWriteError() != 0;
    uint32_t sz = static_cast<uint32_t>(f.size());
    f.close();
    sdUnlock();

    if (hadErr || sz == 0 || !SD.exists(path)) {
        sdMarkFault("writeScreenshotDiagnostic/write");
        errorOut = hadErr ? "write_error" : "verify_failed";
        if (SD.exists(path)) SD.remove(path);
        return false;
    }

    savedPathOut = path;
    return true;
}

void triggerScreenshotCapture()
{
    if (!sdEnsureMounted(true)) {
        showTransientPopup("Screenshot", "SD card not found", TFT_RED, 1100);
        return;
    }
    if (fsWriteBusy()) {
        showTransientPopup("Screenshot", "Storage busy", TFT_RED, 900);
        return;
    }
    // Queue capture so popup lifecycle stays non-blocking and never gets stuck.
    screenshotPending = true;
    showTransientPopup("Screenshot", "Taking screenshot...", TFT_DARKCYAN, 260);
}

void screenshotService()
{
    if (!screenshotPending) return;
    // Wait until transient "taking" popup expires before running capture work.
    if (uiPopupActive) return;
    screenshotPending = false;
    if (!sdEnsureMounted() || fsWriteBusy()) {
        showTransientPopup("Screenshot failed", "Storage busy/error", TFT_RED, 1000);
        return;
    }
    fsWriteBegin();
    String path;
    String err;
    bool ok = captureScreenToJpeg(path, err);
    if (!ok && err != "media_busy" && err != "sd_lock") {
        sdEnsureMounted(true);
        ok = captureScreenToJpeg(path, err);
    }
    fsWriteEnd();
    if (ok) {
        String name = path;
        int slash = name.lastIndexOf('/');
        if (slash >= 0) name = name.substring(slash + 1);
        showTransientPopup("Screenshot saved", name, TFT_DARKGREEN, 1400);
        Serial.println("[SHOT] jpeg saved: " + path);
    } else {
        String msg = "Error: " + (err.isEmpty() ? String("unknown") : err);
        showTransientPopup("Screenshot failed", msg, TFT_RED, 1200);
        Serial.println("[SHOT] jpeg failed: " + msg);
    }
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
    String path = "/web" + urlPath;
    if (sendMaybeGz(request, path)) return;
    request->send(404, "text/plain", "Not found");
}

void setupWebRoutes()
{
    server.on("/version", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", FW_VERSION);
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

void refreshMdnsState()
{
    if (WiFi.status() == WL_CONNECTED) {
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

void setupWifiAndServer()
{
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID, AP_PASS);
    IPAddress ip = WiFi.softAPIP();
    Serial.printf("AP: %s  IP: %s\n", AP_SSID, ip.toString().c_str());
    dnsServer.start(53, "*", ip);
    uiStatusLine = "AP ready: " + ip.toString();

    setupWebRoutes();
    server.begin();

    loadSavedStaCreds();
    tryBootStaReconnect();
}

void setupSd()
{
    if (sdEnsureMounted(true)) Serial.println("SD mounted");
    else Serial.println("SD mount failed");
}

void setup()
{
    sdMutex = xSemaphoreCreateMutex();
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
    pinMode(RGB_PIN_R, OUTPUT);
    pinMode(RGB_PIN_G, OUTPUT);
    pinMode(RGB_PIN_B, OUTPUT);
    pinMode(TOUCH_IRQ, INPUT_PULLUP);
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

    cstInit();
    setupSd();
    sdStatsLogSnapshot(sdStatsSnapshot(), "boot");
    audioSetVolumeImmediate(0);
    audio.forceMono(AUDIO_FORCE_MONO_INTERNAL_DAC);
    // Keep EQ flat for internal DAC path to avoid unnecessary attenuation.
    audio.setTone(0, 0, 0);
    setupWifiAndServer();
    rgbRefreshByMediaState();
    redrawUi();
    lastUserActivityMs = millis();
    lastSensorSampleMs = millis();
    lastLowBatteryPopupMs = millis();
    lastFullBatteryPopupMs = millis();

    Serial.printf("Audio decoder ready, DAC-left on GPIO %d\n", I2S_SPK_PIN);
}

void loop()
{
    bool needMediaRedraw = false;
    dnsServer.processNextRequest();
    sdStatsService();
    if (!sdMounted && !fsWriteBusy() && static_cast<unsigned long>(millis() - sdLastAutoRetryMs) >= SD_AUTORETRY_PERIOD_MS) {
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
    if (wasRunning && !audio.isRunning() && mediaIsPlaying && !mediaPaused) {
        mediaIsPlaying = false;
        uiStatusLine = "Track finished";
        rgbRefreshByMediaState();
        needMediaRedraw = true;
    }

    int16_t x = 0, y = 0;
    const bool isDown = readScreenTouch(x, y);

    if (millis() - lastSensorSampleMs >= SENSOR_SAMPLE_PERIOD_MS) {
        lastSensorSampleMs = millis();
        sampleTopIndicators();
    }

    if (!displayAwake && (millis() - lastUserActivityMs >= LCD_IDLE_TIMEOUT_MS)) {
        // Keep state as-is while sleeping.
    } else if (displayAwake && (millis() - lastUserActivityMs >= LCD_IDLE_TIMEOUT_MS)) {
        if (batteryCharging) animateChargingBeforeSleep();
        displaySetAwake(false);
    }

    if (!displayAwake) {
        if (isDown && digitalRead(TOUCH_IRQ) == LOW) {
            if (wakeTouchConfirmCount < 2) wakeTouchConfirmCount++;
        } else {
            wakeTouchConfirmCount = 0;
        }
        if (wakeTouchConfirmCount >= 2) {
            wakeTouchConfirmCount = 0;
            displaySetAwake(true);
            lastUserActivityMs = millis();
            ignoreTouchUntilRelease = true;
        }
    }

    if (ignoreTouchUntilRelease) {
        if (!isDown) {
            ignoreTouchUntilRelease = false;
            touchTracking = false;
            touchGestureConsumed = false;
        }
    } else if (displayAwake) {
        if (isDown && !touchWasDown) {
            touchTracking = true;
            touchGestureConsumed = false;
            touchStartX = x;
            touchStartY = y;
            touchLastX = x;
            touchLastY = y;
            lastUserActivityMs = millis();
        } else if (isDown && touchTracking) {
            touchLastX = x;
            touchLastY = y;
            if (!touchGestureConsumed) {
                const int dx = static_cast<int>(x - touchStartX);
                const int dy = static_cast<int>(y - touchStartY);
                const int adx = abs(dx);
                const int ady = abs(dy);
                if (adx >= TOUCH_HSWIPE_MIN_DELTA && adx > (ady + TOUCH_SWIPE_DOMINANCE_PX)) {
                    touchGestureConsumed = true;
                    lastUserActivityMs = millis();
                    handleHorizontalSwipeGesture((dx < 0) ? -1 : +1);
                }
                else if (ady >= TOUCH_SWIPE_MIN_DELTA && ady > (adx + TOUCH_SWIPE_DOMINANCE_PX)) {
                    touchGestureConsumed = true;
                    lastUserActivityMs = millis();
                    handleVerticalSwipeGesture((dy < 0) ? +1 : -1);
                }
            }
        } else if (!isDown && touchWasDown && touchTracking) {
            if (!touchGestureConsumed) {
                if (uiScreen == UI_HOME) handleTouchHome(touchLastX, touchLastY);
                else if (uiScreen == UI_WIFI_LIST) handleTouchWifiList(touchLastX, touchLastY);
                else if (uiScreen == UI_KEYBOARD) handleTouchKeyboard(touchLastX, touchLastY);
                else if (uiScreen == UI_MEDIA) handleTouchMedia(touchLastX, touchLastY);
                else if (uiScreen == UI_INFO) handleTouchInfo(touchLastX, touchLastY);
                else if (uiScreen == UI_GAMES) handleTouchGames(touchLastX, touchLastY);
                else if (uiScreen == UI_CONFIG) handleTouchConfig(touchLastX, touchLastY);
                else if (uiScreen == UI_CONFIG_STYLE) handleTouchConfigStyle(touchLastX, touchLastY);
                else if (uiScreen == UI_CONFIG_MQTT) handleTouchConfigMqtt(touchLastX, touchLastY);
                else if (uiScreen == UI_CONFIG_MQTT_CONFIG) handleTouchConfigMqttConfig(touchLastX, touchLastY);
                else if (uiScreen == UI_CONFIG_MQTT_CONTROLS) handleTouchConfigMqttControls(touchLastX, touchLastY);
                else if (uiScreen == UI_GAME_SNAKE) handleTouchGameSnake(touchLastX, touchLastY);
                else handleTouchGameTetris(touchLastX, touchLastY);
            }
            touchTracking = false;
            touchGestureConsumed = false;
        } else if (!isDown && !touchWasDown) {
            touchTracking = false;
            touchGestureConsumed = false;
        }
    }

    snakeTick();
    tetrisTick();
    popupService();
    screenshotService();

    static wl_status_t lastWiFi = WL_IDLE_STATUS;
    wl_status_t cur = WiFi.status();

    if (bootStaConnectInProgress && cur != WL_CONNECTED) {
        if (millis() - bootStaConnectStartedMs > BOOT_STA_TIMEOUT_MS) {
            bootStaConnectInProgress = false;
            WiFi.disconnect();
            uiStatusLine = "Saved STA reconnect failed";
            if (uiScreen == UI_HOME) redrawUi();
        }
    }

    if (cur != lastWiFi) {
        if (cur == WL_CONNECTED) {
            bootStaConnectInProgress = false;
            uiStatusLine = "Connected: " + WiFi.SSID();
            if (pendingSaveCreds && WiFi.SSID() == pendingSaveSsid) {
                saveStaCreds(pendingSaveSsid, pendingSavePass);
                pendingSaveCreds = false;
                pendingSaveSsid = "";
                pendingSavePass = "";
                uiStatusLine = "Connected+Saved: " + WiFi.SSID();
            }
            if (uiScreen == UI_MEDIA) loadMediaEntries();
        } else {
            if (lastWiFi == WL_CONNECTED) uiStatusLine = "WiFi disconnected";
        }
        redrawUi();
        lastWiFi = cur;
    }

    if (!displayAwake && !isDown && !batteryCharging && batteryPercent < 5 &&
        (millis() - lastLowBatteryPopupMs >= LOW_BATT_POPUP_PERIOD_MS)) {
        lastLowBatteryPopupMs = millis();
        animateLowBatteryWarning();
        digitalWrite(TFT_BL, LOW);
    }
    else if (!displayAwake && !isDown && batteryCharging && batteryPercent >= 98 &&
             (millis() - lastFullBatteryPopupMs >= FULL_BATT_POPUP_PERIOD_MS)) {
        lastFullBatteryPopupMs = millis();
        animateFullBatteryNotification();
        digitalWrite(TFT_BL, LOW);
    }

    if (displayAwake && millis() - lastTopIndicatorRefreshMs >= 1500) {
        lastTopIndicatorRefreshMs = millis();
        drawTopForCurrentScreen();
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

    if (needMediaRedraw && uiScreen == UI_MEDIA) redrawUi();
    if (canEnterLowPowerSleep(isDown)) enterLowPowerSleep();
    touchWasDown = isDown;
    delay(1);
}
