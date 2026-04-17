#ifdef BOARD_JC4880P443C_I_W

#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_lcd_mipi_dsi.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_ldo_regulator.h>
#include <esp_log.h>

namespace {

constexpr int BUILTIN_LED_PIN = 26;
constexpr int BOOT_BUTTON_PIN = 35;
constexpr int LCD_RESET_PIN = 5;
constexpr int TOUCH_RESET_PIN = 22;
constexpr int TOUCH_INT_PIN = 21;
constexpr int LCD_H_RES = 480;
constexpr int LCD_V_RES = 800;
constexpr int LCD_MIPI_DSI_LANE_NUM = 2;
constexpr int MIPI_DSI_PHY_PWR_LDO_CHAN = 3;
constexpr int MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV = 2500;

static const char *TAG = "jc4880p443";

struct PanelInitCmd {
    uint8_t cmd;
    const uint8_t *data;
    size_t len;
    uint16_t delayMs;
};

const uint8_t CMD_FF_13[] = {0x77, 0x01, 0x00, 0x00, 0x13};
const uint8_t CMD_EF_08[] = {0x08};
const uint8_t CMD_FF_10[] = {0x77, 0x01, 0x00, 0x00, 0x10};
const uint8_t CMD_C0[] = {0x63, 0x00};
const uint8_t CMD_C1[] = {0x0D, 0x02};
const uint8_t CMD_C2[] = {0x10, 0x08};
const uint8_t CMD_CC[] = {0x10};
const uint8_t CMD_B0_GAMMA[] = {0x80, 0x09, 0x53, 0x0C, 0xD0, 0x07, 0x0C, 0x09, 0x09, 0x28, 0x06, 0xD4, 0x13, 0x69, 0x2B, 0x71};
const uint8_t CMD_B1_GAMMA[] = {0x80, 0x94, 0x5A, 0x10, 0xD3, 0x06, 0x0A, 0x08, 0x08, 0x25, 0x03, 0xD3, 0x12, 0x66, 0x6A, 0x0D};
const uint8_t CMD_FF_11[] = {0x77, 0x01, 0x00, 0x00, 0x11};
const uint8_t CMD_B0[] = {0x5D};
const uint8_t CMD_B1[] = {0x58};
const uint8_t CMD_B2[] = {0x87};
const uint8_t CMD_B3[] = {0x80};
const uint8_t CMD_B5[] = {0x4E};
const uint8_t CMD_B7[] = {0x85};
const uint8_t CMD_B8[] = {0x21};
const uint8_t CMD_B9[] = {0x10, 0x1F};
const uint8_t CMD_BB[] = {0x03};
const uint8_t CMD_BC[] = {0x00};
const uint8_t CMD_C1_78[] = {0x78};
const uint8_t CMD_C2_78[] = {0x78};
const uint8_t CMD_D0[] = {0x88};
const uint8_t CMD_E0[] = {0x00, 0x3A, 0x02};
const uint8_t CMD_E1[] = {0x04, 0xA0, 0x00, 0xA0, 0x05, 0xA0, 0x00, 0xA0, 0x00, 0x40, 0x40};
const uint8_t CMD_E2[] = {0x30, 0x00, 0x40, 0x40, 0x32, 0xA0, 0x00, 0xA0, 0x00, 0xA0, 0x00, 0xA0, 0x00};
const uint8_t CMD_E3[] = {0x00, 0x00, 0x33, 0x33};
const uint8_t CMD_E4[] = {0x44, 0x44};
const uint8_t CMD_E5[] = {0x09, 0x2E, 0xA0, 0xA0, 0x0B, 0x30, 0xA0, 0xA0, 0x05, 0x2A, 0xA0, 0xA0, 0x07, 0x2C, 0xA0, 0xA0};
const uint8_t CMD_E6[] = {0x00, 0x00, 0x33, 0x33};
const uint8_t CMD_E7[] = {0x44, 0x44};
const uint8_t CMD_E8[] = {0x08, 0x2D, 0xA0, 0xA0, 0x0A, 0x2F, 0xA0, 0xA0, 0x04, 0x29, 0xA0, 0xA0, 0x06, 0x2B, 0xA0, 0xA0};
const uint8_t CMD_EB[] = {0x00, 0x00, 0x4E, 0x4E, 0x00, 0x00, 0x00};
const uint8_t CMD_EC[] = {0x08, 0x01};
const uint8_t CMD_ED[] = {0xB0, 0x2B, 0x98, 0xA4, 0x56, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF7, 0x65, 0x4A, 0x89, 0xB2, 0x0B};
const uint8_t CMD_EF[] = {0x08, 0x08, 0x08, 0x45, 0x3F, 0x54};
const uint8_t CMD_FF_00[] = {0x77, 0x01, 0x00, 0x00, 0x00};
const uint8_t CMD_NOP[] = {0x00};

const PanelInitCmd PANEL_INIT_CMDS[] = {
    {0xFF, CMD_FF_13, sizeof(CMD_FF_13), 0},
    {0xEF, CMD_EF_08, sizeof(CMD_EF_08), 0},
    {0xFF, CMD_FF_10, sizeof(CMD_FF_10), 0},
    {0xC0, CMD_C0, sizeof(CMD_C0), 0},
    {0xC1, CMD_C1, sizeof(CMD_C1), 0},
    {0xC2, CMD_C2, sizeof(CMD_C2), 0},
    {0xCC, CMD_CC, sizeof(CMD_CC), 0},
    {0xB0, CMD_B0_GAMMA, sizeof(CMD_B0_GAMMA), 0},
    {0xB1, CMD_B1_GAMMA, sizeof(CMD_B1_GAMMA), 0},
    {0xFF, CMD_FF_11, sizeof(CMD_FF_11), 0},
    {0xB0, CMD_B0, sizeof(CMD_B0), 0},
    {0xB1, CMD_B1, sizeof(CMD_B1), 0},
    {0xB2, CMD_B2, sizeof(CMD_B2), 0},
    {0xB3, CMD_B3, sizeof(CMD_B3), 0},
    {0xB5, CMD_B5, sizeof(CMD_B5), 0},
    {0xB7, CMD_B7, sizeof(CMD_B7), 0},
    {0xB8, CMD_B8, sizeof(CMD_B8), 0},
    {0xB9, CMD_B9, sizeof(CMD_B9), 0},
    {0xBB, CMD_BB, sizeof(CMD_BB), 0},
    {0xBC, CMD_BC, sizeof(CMD_BC), 0},
    {0xC1, CMD_C1_78, sizeof(CMD_C1_78), 0},
    {0xC2, CMD_C2_78, sizeof(CMD_C2_78), 0},
    {0xD0, CMD_D0, sizeof(CMD_D0), 0},
    {0xE0, CMD_E0, sizeof(CMD_E0), 0},
    {0xE1, CMD_E1, sizeof(CMD_E1), 0},
    {0xE2, CMD_E2, sizeof(CMD_E2), 0},
    {0xE3, CMD_E3, sizeof(CMD_E3), 0},
    {0xE4, CMD_E4, sizeof(CMD_E4), 0},
    {0xE5, CMD_E5, sizeof(CMD_E5), 0},
    {0xE6, CMD_E6, sizeof(CMD_E6), 0},
    {0xE7, CMD_E7, sizeof(CMD_E7), 0},
    {0xE8, CMD_E8, sizeof(CMD_E8), 0},
    {0xEB, CMD_EB, sizeof(CMD_EB), 0},
    {0xEC, CMD_EC, sizeof(CMD_EC), 0},
    {0xED, CMD_ED, sizeof(CMD_ED), 0},
    {0xEF, CMD_EF, sizeof(CMD_EF), 0},
    {0xFF, CMD_FF_00, sizeof(CMD_FF_00), 0},
    {0x11, CMD_NOP, sizeof(CMD_NOP), 120},
    {0x29, CMD_NOP, sizeof(CMD_NOP), 20},
};

esp_lcd_panel_io_handle_t gPanelIo = nullptr;
esp_lcd_panel_handle_t gPanel = nullptr;
esp_lcd_dsi_bus_handle_t gDsiBus = nullptr;
esp_ldo_channel_handle_t gPhyLdo = nullptr;
bool gPanelReady = false;

bool gBacklightOn = true;
bool gLedOn = false;
bool gLastButtonPressed = false;
unsigned long gLastHeartbeatMs = 0;
unsigned long gLastLogMs = 0;

void writeOutput(int pin, bool level)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, level ? HIGH : LOW);
}

void setBacklight(bool on)
{
    gBacklightOn = on;
    writeOutput(TFT_BL, on);
}

void pulseTouchReset()
{
    pinMode(TOUCH_RESET_PIN, OUTPUT);
    digitalWrite(TOUCH_RESET_PIN, LOW);
    delay(5);
    digitalWrite(TOUCH_RESET_PIN, HIGH);
}

void pulsePanelReset()
{
    pinMode(LCD_RESET_PIN, OUTPUT);
    digitalWrite(LCD_RESET_PIN, LOW);
    delay(20);
    digitalWrite(LCD_RESET_PIN, HIGH);
    delay(120);
}

bool checkOk(esp_err_t err, const char *step)
{
    if (err == ESP_OK) return true;
    Serial.printf("[JC4880P443C_I_W] %s failed: %s\n", step, esp_err_to_name(err));
    return false;
}

bool enableDsiPhyPower()
{
    if (gPhyLdo) return true;

    esp_ldo_channel_config_t ldoCfg = {};
    ldoCfg.chan_id = MIPI_DSI_PHY_PWR_LDO_CHAN;
    ldoCfg.voltage_mv = MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV;
    return checkOk(esp_ldo_acquire_channel(&ldoCfg, &gPhyLdo), "esp_ldo_acquire_channel");
}

bool sendPanelInitSequence()
{
    for (const PanelInitCmd &entry : PANEL_INIT_CMDS) {
        if (!checkOk(esp_lcd_panel_io_tx_param(gPanelIo, entry.cmd, entry.data, entry.len), "esp_lcd_panel_io_tx_param")) {
            return false;
        }
        if (entry.delayMs > 0) {
            delay(entry.delayMs);
        }
    }
    return true;
}

bool initializePanel()
{
    if (gPanelReady) return true;
    if (!enableDsiPhyPower()) return false;

    esp_lcd_dsi_bus_config_t busConfig = {};
    busConfig.bus_id = 0;
    busConfig.num_data_lanes = LCD_MIPI_DSI_LANE_NUM;
    busConfig.phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT;
    busConfig.lane_bit_rate_mbps = 500;
    if (!checkOk(esp_lcd_new_dsi_bus(&busConfig, &gDsiBus), "esp_lcd_new_dsi_bus")) return false;

    esp_lcd_dbi_io_config_t dbiConfig = {};
    dbiConfig.virtual_channel = 0;
    dbiConfig.lcd_cmd_bits = 8;
    dbiConfig.lcd_param_bits = 8;
    if (!checkOk(esp_lcd_new_panel_io_dbi(gDsiBus, &dbiConfig, &gPanelIo), "esp_lcd_new_panel_io_dbi")) return false;

    esp_lcd_dpi_panel_config_t dpiConfig = {};
    dpiConfig.virtual_channel = 0;
    dpiConfig.dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT;
    dpiConfig.dpi_clock_freq_mhz = 34;
    dpiConfig.pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565;
    dpiConfig.in_color_format = LCD_COLOR_FMT_RGB565;
    dpiConfig.out_color_format = LCD_COLOR_FMT_RGB565;
    dpiConfig.num_fbs = 1;
    dpiConfig.video_timing.h_size = LCD_H_RES;
    dpiConfig.video_timing.v_size = LCD_V_RES;
    dpiConfig.video_timing.hsync_pulse_width = 12;
    dpiConfig.video_timing.hsync_back_porch = 42;
    dpiConfig.video_timing.hsync_front_porch = 42;
    dpiConfig.video_timing.vsync_pulse_width = 2;
    dpiConfig.video_timing.vsync_back_porch = 8;
    dpiConfig.video_timing.vsync_front_porch = 166;
    dpiConfig.flags.use_dma2d = 1;
    if (!checkOk(esp_lcd_new_panel_dpi(gDsiBus, &dpiConfig, &gPanel), "esp_lcd_new_panel_dpi")) return false;

    pulsePanelReset();
    if (!sendPanelInitSequence()) return false;
    if (!checkOk(esp_lcd_panel_init(gPanel), "esp_lcd_panel_init")) return false;
    if (!checkOk(esp_lcd_dpi_panel_set_pattern(gPanel, MIPI_DSI_PATTERN_BAR_VERTICAL), "esp_lcd_dpi_panel_set_pattern")) return false;

    gPanelReady = true;
    return true;
}

void printBoardInfo()
{
    Serial.println();
    Serial.println("[JC4880P443C_I_W] P4 bring-up target");
    Serial.printf("[JC4880P443C_I_W] flash: %u bytes\n", ESP.getFlashChipSize());
    Serial.printf("[JC4880P443C_I_W] psram: %u bytes\n", ESP.getPsramSize());
    Serial.printf("[JC4880P443C_I_W] free heap: %u bytes\n", ESP.getFreeHeap());
    Serial.printf("[JC4880P443C_I_W] display: %dx%d MIPI-DSI ST7701\n", TFT_WIDTH, TFT_HEIGHT);
    Serial.printf("[JC4880P443C_I_W] pins: BL=%d LED=%d LCD_RST=%d TOUCH_RST=%d TOUCH_INT=%d BOOT=%d\n",
                  TFT_BL, BUILTIN_LED_PIN, LCD_RESET_PIN, TOUCH_RESET_PIN, TOUCH_INT_PIN, BOOT_BUTTON_PIN);
    Serial.println("[JC4880P443C_I_W] expected display: vertical color bars");
    Serial.println("[JC4880P443C_I_W] press BOOT to toggle backlight");
}

} // namespace

void setup()
{
    Serial.begin(115200);
    delay(250);

    pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
    pinMode(TOUCH_INT_PIN, INPUT_PULLUP);

    pulseTouchReset();
    setBacklight(true);
    writeOutput(BUILTIN_LED_PIN, LOW);

    printBoardInfo();
    if (initializePanel()) {
        Serial.println("[JC4880P443C_I_W] panel init OK");
    } else {
        Serial.println("[JC4880P443C_I_W] panel init FAILED");
    }
}

void loop()
{
    const unsigned long now = millis();
    const bool buttonPressed = digitalRead(BOOT_BUTTON_PIN) == LOW;

    if (buttonPressed && !gLastButtonPressed) {
        setBacklight(!gBacklightOn);
        Serial.printf("[JC4880P443C_I_W] backlight %s\n", gBacklightOn ? "ON" : "OFF");
    }
    gLastButtonPressed = buttonPressed;

    if (now - gLastHeartbeatMs >= 500UL) {
        gLastHeartbeatMs = now;
        gLedOn = !gLedOn;
        digitalWrite(BUILTIN_LED_PIN, gLedOn ? HIGH : LOW);
    }

    if (now - gLastLogMs >= 2000UL) {
        gLastLogMs = now;
        Serial.printf("[JC4880P443C_I_W] heap=%u touch_int=%d button=%d backlight=%d panel=%d\n",
                      ESP.getFreeHeap(), digitalRead(TOUCH_INT_PIN), buttonPressed ? 1 : 0,
                      gBacklightOn ? 1 : 0, gPanelReady ? 1 : 0);
    }
}

#endif