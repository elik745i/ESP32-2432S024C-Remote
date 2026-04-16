#include "display_compat.h"

#if defined(BOARD_UEDX24320028E_WB_A) || defined(BOARD_UEDX32480035E_WB_A)

#include <algorithm>
#include <vector>
#include <SPI.h>
#include <Wire.h>

namespace {

#ifndef LCD_PIN_SCK_OVERRIDE
#define LCD_PIN_SCK_OVERRIDE 40
#endif
#ifndef LCD_PIN_MOSI_OVERRIDE
#define LCD_PIN_MOSI_OVERRIDE 45
#endif
#ifndef LCD_PIN_CS_OVERRIDE
#define LCD_PIN_CS_OVERRIDE 42
#endif
#ifndef LCD_PIN_DC_OVERRIDE
#define LCD_PIN_DC_OVERRIDE 41
#endif
#ifndef LCD_PIN_RST_OVERRIDE
#define LCD_PIN_RST_OVERRIDE -1
#endif

constexpr int LCD_PIN_SCK = LCD_PIN_SCK_OVERRIDE;
constexpr int LCD_PIN_MOSI = LCD_PIN_MOSI_OVERRIDE;
constexpr int LCD_PIN_CS = LCD_PIN_CS_OVERRIDE;
constexpr int LCD_PIN_DC = LCD_PIN_DC_OVERRIDE;
constexpr int LCD_PIN_RST = LCD_PIN_RST_OVERRIDE;
#ifndef LCD_PIN_IM0_OVERRIDE
#if defined(BOARD_UEDX32480035E_WB_A)
#define LCD_PIN_IM0_OVERRIDE 47
#else
#define LCD_PIN_IM0_OVERRIDE -1
#endif
#endif

#ifndef LCD_PIN_IM1_OVERRIDE
#if defined(BOARD_UEDX32480035E_WB_A)
#define LCD_PIN_IM1_OVERRIDE 48
#else
#define LCD_PIN_IM1_OVERRIDE -1
#endif
#endif

constexpr int LCD_PIN_IM0 = LCD_PIN_IM0_OVERRIDE;
constexpr int LCD_PIN_IM1 = LCD_PIN_IM1_OVERRIDE;
constexpr int TOUCH_PIN_SDA = 1;
constexpr int TOUCH_PIN_SCL = 3;
constexpr int TOUCH_PIN_RST = 2;
constexpr int TOUCH_PIN_INT = 4;
constexpr uint8_t TOUCH_ADDR = 0x2E;
#if defined(BOARD_UEDX32480035E_WB_A)
constexpr uint32_t LCD_SPI_HZ = 80000000UL;
#else
constexpr uint32_t LCD_SPI_HZ = 40000000UL;
#endif

constexpr int LCD_IM0_LEVEL = HIGH;
constexpr int LCD_IM1_LEVEL = HIGH;

constexpr uint8_t CMD_SWRESET = 0x01;
constexpr uint8_t CMD_SLPOUT = 0x11;
constexpr uint8_t CMD_DISPON = 0x29;
constexpr uint8_t CMD_CASET = 0x2A;
constexpr uint8_t CMD_RASET = 0x2B;
constexpr uint8_t CMD_RAMWR = 0x2C;
constexpr uint8_t CMD_MADCTL = 0x36;
constexpr uint8_t CMD_COLMOD = 0x3A;

constexpr uint8_t MADCTL_MY = 0x80;
constexpr uint8_t MADCTL_MX = 0x40;
constexpr uint8_t MADCTL_MV = 0x20;
constexpr uint8_t MADCTL_BGR = 0x08;

struct PanelInitCmd {
    uint8_t cmd;
    const uint8_t *data;
    uint8_t len;
    uint16_t delayMs;
};

const uint8_t CMD_86[] = {0x98};
const uint8_t CMD_89[] = {0x03};
const uint8_t CMD_8B[] = {0x80};
const uint8_t CMD_8D[] = {0x33};
const uint8_t CMD_8E[] = {0x0F};
const uint8_t CMD_E8[] = {0x12, 0x00};
const uint8_t CMD_C3[] = {0x1D};
const uint8_t CMD_C4[] = {0x1D};
const uint8_t CMD_C9[] = {0x0F};
const uint8_t CMD_FF[] = {0x62};
const uint8_t CMD_99[] = {0x3E};
const uint8_t CMD_9D[] = {0x4B};
const uint8_t CMD_98[] = {0x3E};
const uint8_t CMD_9C[] = {0x4B};
const uint8_t CMD_F0[] = {0x49, 0x0B, 0x09, 0x08, 0x06, 0x2E};
const uint8_t CMD_F2[] = {0x49, 0x0B, 0x09, 0x08, 0x06, 0x2E};
const uint8_t CMD_F1[] = {0x45, 0x92, 0x93, 0x2B, 0x31, 0x6F};
const uint8_t CMD_F3[] = {0x45, 0x92, 0x93, 0x2B, 0x31, 0x6F};
const uint8_t CMD_35[] = {0x00};
const uint8_t CMD_3A[] = {0x55};
const uint8_t CMD_ST_F0_C3[] = {0xC3};
const uint8_t CMD_ST_F0_96[] = {0x96};
const uint8_t CMD_ST_B4[] = {0x01};
const uint8_t CMD_ST_B7[] = {0xC6};
const uint8_t CMD_ST_C0[] = {0x80, 0x04};
const uint8_t CMD_ST_C1[] = {0x13};
const uint8_t CMD_ST_C5_A7[] = {0xA7};
const uint8_t CMD_ST_C5_16[] = {0x16};
const uint8_t CMD_ST_E8[] = {0x40, 0x8A, 0x00, 0x00, 0x29, 0x19, 0xA5, 0x33};
const uint8_t CMD_ST_E0[] = {0xF0, 0x19, 0x20, 0x10, 0x11, 0x0A, 0x46, 0x44, 0x57, 0x09, 0x1A, 0x1B, 0x2A, 0x2D};
const uint8_t CMD_ST_E1[] = {0xF0, 0x12, 0x1A, 0x0A, 0x0C, 0x18, 0x45, 0x44, 0x56, 0x3F, 0x15, 0x11, 0x24, 0x26};
const uint8_t CMD_ST_F0_3C[] = {0x3C};
const uint8_t CMD_ST_F0_69[] = {0x69};

#if defined(BOARD_UEDX32480035E_WB_A)
const PanelInitCmd PANEL_INIT_CMDS[] = {
    {CMD_SLPOUT, nullptr, 0, 120},
    {CMD_COLMOD, CMD_3A, sizeof(CMD_3A), 0},
    {0xF0, CMD_ST_F0_C3, sizeof(CMD_ST_F0_C3), 0},
    {0xF0, CMD_ST_F0_96, sizeof(CMD_ST_F0_96), 0},
    {0xB4, CMD_ST_B4, sizeof(CMD_ST_B4), 0},
    {0xB7, CMD_ST_B7, sizeof(CMD_ST_B7), 0},
    {0xC0, CMD_ST_C0, sizeof(CMD_ST_C0), 0},
    {0xC1, CMD_ST_C1, sizeof(CMD_ST_C1), 0},
    {0xC5, CMD_ST_C5_A7, sizeof(CMD_ST_C5_A7), 0},
    {0xC5, CMD_ST_C5_16, sizeof(CMD_ST_C5_16), 0},
    {0xE8, CMD_ST_E8, sizeof(CMD_ST_E8), 0},
    {0xE0, CMD_ST_E0, sizeof(CMD_ST_E0), 0},
    {0xE1, CMD_ST_E1, sizeof(CMD_ST_E1), 0},
    {0xF0, CMD_ST_F0_3C, sizeof(CMD_ST_F0_3C), 0},
    {0xF0, CMD_ST_F0_69, sizeof(CMD_ST_F0_69), 0},
    {0x21, nullptr, 0, 0},
    {CMD_DISPON, nullptr, 0, 50},
    {CMD_RAMWR, nullptr, 0, 0},
};
#else
const PanelInitCmd PANEL_INIT_CMDS[] = {
    {0xFE, nullptr, 0, 0},
    {0xFE, nullptr, 0, 0},
    {0xEF, nullptr, 0, 0},
    {0x86, CMD_86, sizeof(CMD_86), 0},
    {0x89, CMD_89, sizeof(CMD_89), 0},
    {0x8B, CMD_8B, sizeof(CMD_8B), 0},
    {0x8D, CMD_8D, sizeof(CMD_8D), 0},
    {0x8E, CMD_8E, sizeof(CMD_8E), 0},
    {0xE8, CMD_E8, sizeof(CMD_E8), 0},
    {0xC3, CMD_C3, sizeof(CMD_C3), 0},
    {0xC4, CMD_C4, sizeof(CMD_C4), 0},
    {0xC9, CMD_C9, sizeof(CMD_C9), 0},
    {0xFF, CMD_FF, sizeof(CMD_FF), 0},
    {0x99, CMD_99, sizeof(CMD_99), 0},
    {0x9D, CMD_9D, sizeof(CMD_9D), 0},
    {0x98, CMD_98, sizeof(CMD_98), 0},
    {0x9C, CMD_9C, sizeof(CMD_9C), 0},
    {0xF0, CMD_F0, sizeof(CMD_F0), 0},
    {0xF2, CMD_F2, sizeof(CMD_F2), 0},
    {0xF1, CMD_F1, sizeof(CMD_F1), 0},
    {0xF3, CMD_F3, sizeof(CMD_F3), 0},
    {0x35, CMD_35, sizeof(CMD_35), 0},
    {CMD_COLMOD, CMD_3A, sizeof(CMD_3A), 0},
    {CMD_SLPOUT, nullptr, 0, 120},
    {CMD_DISPON, nullptr, 0, 20},
    {CMD_RAMWR, nullptr, 0, 0},
};
#endif

SPIClass gSpi(FSPI);
bool gReady = false;
bool gTouchReady = false;
bool gWriteActive = false;
uint8_t gRotation = 0;
std::vector<uint16_t> gColorBuffer;
TwoWire &gTouchWire = Wire;

uint16_t swap16(uint16_t value)
{
    return static_cast<uint16_t>((value << 8) | (value >> 8));
}

void beginTransaction()
{
    if (gWriteActive) return;
    gSpi.beginTransaction(SPISettings(LCD_SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(LCD_PIN_CS, LOW);
    gWriteActive = true;
}

void endTransaction()
{
    if (!gWriteActive) return;
    digitalWrite(LCD_PIN_CS, HIGH);
    gSpi.endTransaction();
    gWriteActive = false;
}

void writeCommandLocked(uint8_t cmd)
{
    digitalWrite(LCD_PIN_DC, LOW);
    gSpi.transfer(cmd);
}

void writeDataLocked(const uint8_t *data, size_t len)
{
    if (!data || len == 0) return;
    digitalWrite(LCD_PIN_DC, HIGH);
    gSpi.writeBytes(data, len);
}

void writeCommand(uint8_t cmd, const uint8_t *data = nullptr, size_t len = 0)
{
    beginTransaction();
    writeCommandLocked(cmd);
    writeDataLocked(data, len);
    endTransaction();
}

uint8_t rotationMadctl(uint8_t rotation)
{
    switch (rotation & 0x03) {
        case 1: return static_cast<uint8_t>(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
        case 2: return static_cast<uint8_t>(MADCTL_MY | MADCTL_BGR);
        case 3: return static_cast<uint8_t>(MADCTL_MV | MADCTL_BGR);
        default: return static_cast<uint8_t>(MADCTL_MX | MADCTL_BGR);
    }
}

void applyRotation()
{
    const uint8_t madctl = rotationMadctl(gRotation);
    writeCommand(CMD_MADCTL, &madctl, 1);
}

bool ensureTouchReady()
{
    if (gTouchReady) return true;
    pinMode(TOUCH_PIN_RST, OUTPUT);
    digitalWrite(TOUCH_PIN_RST, LOW);
    delay(5);
    digitalWrite(TOUCH_PIN_RST, HIGH);
    delay(50);
    pinMode(TOUCH_PIN_INT, INPUT_PULLUP);
    gTouchWire.begin(TOUCH_PIN_SDA, TOUCH_PIN_SCL, 400000U);
    gTouchWire.setTimeOut(20);
    gTouchReady = true;
    return true;
}

bool touchReadFrame(uint8_t *buf, size_t len)
{
    if (!ensureTouchReady() || !buf || len == 0) return false;
    gTouchWire.beginTransmission(TOUCH_ADDR);
    gTouchWire.write(static_cast<uint8_t>(0x00));
    if (gTouchWire.endTransmission(false) != 0) return false;
    const size_t got = gTouchWire.requestFrom(static_cast<int>(TOUCH_ADDR), static_cast<int>(len));
    if (got != len) return false;
    for (size_t i = 0; i < len; ++i) buf[i] = static_cast<uint8_t>(gTouchWire.read());
    return true;
}

void transformTouchByRotation(int16_t &x, int16_t &y)
{
    const int16_t rawX = x;
    const int16_t rawY = y;
    switch (gRotation & 0x03) {
        case 1:
            x = rawY;
            y = static_cast<int16_t>(TFT_WIDTH - 1 - rawX);
            break;
        case 2:
            x = static_cast<int16_t>(TFT_WIDTH - 1 - rawX);
            y = static_cast<int16_t>(TFT_HEIGHT - 1 - rawY);
            break;
        case 3:
            x = static_cast<int16_t>(TFT_HEIGHT - 1 - rawY);
            y = rawX;
            break;
        default:
            break;
    }
}

bool ensurePanelReady()
{
    if (gReady) return true;

    Serial.println("[PANEL] begin");
    if (LCD_PIN_IM0 >= 0) {
        pinMode(LCD_PIN_IM0, OUTPUT);
        digitalWrite(LCD_PIN_IM0, LCD_IM0_LEVEL);
    }
    if (LCD_PIN_IM1 >= 0) {
        pinMode(LCD_PIN_IM1, OUTPUT);
        digitalWrite(LCD_PIN_IM1, LCD_IM1_LEVEL);
    }
    if (LCD_PIN_IM0 >= 0 && LCD_PIN_IM1 >= 0) {
        Serial.printf("[PANEL] straps im0=%d im1=%d\n", digitalRead(LCD_PIN_IM0), digitalRead(LCD_PIN_IM1));
    }

    pinMode(LCD_PIN_CS, OUTPUT);
    pinMode(LCD_PIN_DC, OUTPUT);
    digitalWrite(LCD_PIN_CS, HIGH);
    digitalWrite(LCD_PIN_DC, HIGH);
    if (LCD_PIN_RST >= 0) {
        pinMode(LCD_PIN_RST, OUTPUT);
        digitalWrite(LCD_PIN_RST, HIGH);
        digitalWrite(LCD_PIN_RST, LOW);
        delay(20);
        digitalWrite(LCD_PIN_RST, HIGH);
        delay(120);
    }

    gSpi.begin(LCD_PIN_SCK, -1, LCD_PIN_MOSI, -1);
    Serial.printf("[PANEL] spi begin sck=%d mosi=%d cs=%d dc=%d rst=%d\n",
                  LCD_PIN_SCK,
                  LCD_PIN_MOSI,
                  LCD_PIN_CS,
                  LCD_PIN_DC,
                  LCD_PIN_RST);
    delay(20);
    for (const PanelInitCmd &cmd : PANEL_INIT_CMDS) {
        writeCommand(cmd.cmd, cmd.data, cmd.len);
        if (cmd.delayMs) delay(cmd.delayMs);
    }
    Serial.println("[PANEL] init sequence done");
    applyRotation();
    Serial.printf("[PANEL] rotation applied=%u\n", static_cast<unsigned int>(gRotation));
    Serial.println("[PANEL] backlight delegated to pwm controller");
    gReady = true;
    Serial.println("[PANEL] ready");
    return true;
}

void normalizeRect(int32_t &x, int32_t &y, int32_t &w, int32_t &h, int32_t maxW, int32_t maxH)
{
    if (w <= 0 || h <= 0) {
        w = 0;
        h = 0;
        return;
    }
    if (x < 0) {
        w += x;
        x = 0;
    }
    if (y < 0) {
        h += y;
        y = 0;
    }
    if (x >= maxW || y >= maxH) {
        w = 0;
        h = 0;
        return;
    }
    if ((x + w) > maxW) w = maxW - x;
    if ((y + h) > maxH) h = maxH - y;
}

bool drawBitmapHost565(int32_t x, int32_t y, int32_t w, int32_t h, const uint16_t *data, bool swapBytes)
{
    if (!ensurePanelReady() || !data || w <= 0 || h <= 0) return false;
    const size_t count = static_cast<size_t>(w) * static_cast<size_t>(h);
    gColorBuffer.resize(count);
    if (swapBytes) {
        for (size_t i = 0; i < count; ++i) gColorBuffer[i] = swap16(data[i]);
    } else {
        memcpy(gColorBuffer.data(), data, count * sizeof(uint16_t));
    }

    const uint8_t colData[] = {
        static_cast<uint8_t>((x >> 8) & 0xFF),
        static_cast<uint8_t>(x & 0xFF),
        static_cast<uint8_t>(((x + w - 1) >> 8) & 0xFF),
        static_cast<uint8_t>((x + w - 1) & 0xFF),
    };
    const uint8_t rowData[] = {
        static_cast<uint8_t>((y >> 8) & 0xFF),
        static_cast<uint8_t>(y & 0xFF),
        static_cast<uint8_t>(((y + h - 1) >> 8) & 0xFF),
        static_cast<uint8_t>((y + h - 1) & 0xFF),
    };

    beginTransaction();
    writeCommandLocked(CMD_CASET);
    writeDataLocked(colData, sizeof(colData));
    writeCommandLocked(CMD_RASET);
    writeDataLocked(rowData, sizeof(rowData));
    writeCommandLocked(CMD_RAMWR);
    writeDataLocked(reinterpret_cast<const uint8_t *>(gColorBuffer.data()), count * sizeof(uint16_t));
    endTransaction();
    return true;
}

void drawSolidRect(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t color)
{
    if (!ensurePanelReady()) return;
    const int32_t maxW = (gRotation & 0x01) ? TFT_HEIGHT : TFT_WIDTH;
    const int32_t maxH = (gRotation & 0x01) ? TFT_WIDTH : TFT_HEIGHT;
    normalizeRect(x, y, w, h, maxW, maxH);
    if (w <= 0 || h <= 0) return;

    std::vector<uint16_t> row(static_cast<size_t>(w), swap16(color));
    for (int32_t yy = 0; yy < h; ++yy) {
        (void)drawBitmapHost565(x, y + yy, w, 1, row.data(), false);
    }
}

void drawPixel(int32_t x, int32_t y, uint16_t color)
{
    drawSolidRect(x, y, 1, 1, color);
}

} // namespace

void TFT_eSPI::init()
{
    (void)ensurePanelReady();
}

void TFT_eSPI::setRotation(uint8_t rotation)
{
    gRotation = rotation & 0x03;
    if (!ensurePanelReady()) return;
    applyRotation();
}

void TFT_eSPI::setTextFont(int font)
{
    (void)font;
}

void TFT_eSPI::setTextColor(uint16_t fg, uint16_t bg)
{
    _textFg = fg;
    _textBg = bg;
}

int16_t TFT_eSPI::width() const
{
    return (gRotation & 0x01) ? TFT_HEIGHT : TFT_WIDTH;
}

int16_t TFT_eSPI::height() const
{
    return (gRotation & 0x01) ? TFT_WIDTH : TFT_HEIGHT;
}

void TFT_eSPI::startWrite() { beginTransaction(); }
void TFT_eSPI::endWrite() { endTransaction(); }

void TFT_eSPI::setAddrWindow(int32_t x, int32_t y, int32_t w, int32_t h)
{
    _windowX = x;
    _windowY = y;
    _windowW = w;
    _windowH = h;
}

void TFT_eSPI::pushColors(uint16_t *data, uint32_t len, bool swap)
{
    if (!data || len == 0) return;
    int32_t w = _windowW;
    int32_t h = _windowH;
    if (w <= 0 || h <= 0) return;
    const uint32_t expected = static_cast<uint32_t>(w) * static_cast<uint32_t>(h);
    if (expected != len) {
        h = static_cast<int32_t>(len / static_cast<uint32_t>(std::max<int32_t>(w, 1)));
        if (h <= 0) h = 1;
    }
    (void)drawBitmapHost565(_windowX, _windowY, w, h, data, swap);
}

void TFT_eSPI::fillScreen(uint16_t color)
{
    drawSolidRect(0, 0, width(), height(), color);
}

void TFT_eSPI::fillRect(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t color)
{
    drawSolidRect(x, y, w, h, color);
}

void TFT_eSPI::fillRoundRect(int32_t x, int32_t y, int32_t w, int32_t h, int32_t r, uint16_t color)
{
    (void)r;
    drawSolidRect(x, y, w, h, color);
}

void TFT_eSPI::drawFastHLine(int32_t x, int32_t y, int32_t w, uint16_t color)
{
    drawSolidRect(x, y, w, 1, color);
}

void TFT_eSPI::drawLine(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint16_t color)
{
    int32_t dx = abs(x1 - x0);
    int32_t sx = x0 < x1 ? 1 : -1;
    int32_t dy = -abs(y1 - y0);
    int32_t sy = y0 < y1 ? 1 : -1;
    int32_t err = dx + dy;

    while (true) {
        drawPixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        const int32_t e2 = err << 1;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void TFT_eSPI::drawRoundRect(int32_t x, int32_t y, int32_t w, int32_t h, int32_t r, uint16_t color)
{
    (void)r;
    drawLine(x, y, x + w - 1, y, color);
    drawLine(x, y + h - 1, x + w - 1, y + h - 1, color);
    drawLine(x, y, x, y + h - 1, color);
    drawLine(x + w - 1, y, x + w - 1, y + h - 1, color);
}

void TFT_eSPI::fillTriangle(int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint16_t color)
{
    if (y0 > y1) { std::swap(y0, y1); std::swap(x0, x1); }
    if (y1 > y2) { std::swap(y1, y2); std::swap(x1, x2); }
    if (y0 > y1) { std::swap(y0, y1); std::swap(x0, x1); }

    auto edgeInterpolate = [](int32_t x0i, int32_t y0i, int32_t x1i, int32_t y1i, int32_t y) -> int32_t {
        if (y1i == y0i) return x0i;
        return x0i + (x1i - x0i) * (y - y0i) / (y1i - y0i);
    };

    for (int32_t y = y0; y <= y2; ++y) {
        int32_t xa = edgeInterpolate(x0, y0, x2, y2, y);
        int32_t xb = (y < y1) ? edgeInterpolate(x0, y0, x1, y1, y) : edgeInterpolate(x1, y1, x2, y2, y);
        if (xa > xb) std::swap(xa, xb);
        drawSolidRect(xa, y, xb - xa + 1, 1, color);
    }
}

uint16_t TFT_eSPI::color565(uint8_t r, uint8_t g, uint8_t b) const
{
    return static_cast<uint16_t>(((r & 0xF8U) << 8) | ((g & 0xFCU) << 3) | (b >> 3));
}

int16_t TFT_eSPI::drawString(const char *str, int32_t x, int32_t y, int font)
{
    (void)str;
    (void)x;
    (void)y;
    (void)font;
    return 0;
}

int16_t TFT_eSPI::drawCentreString(const char *str, int32_t x, int32_t y, int font)
{
    return drawString(str, x, y, font);
}

bool displayCompatTouchRead(int16_t &x, int16_t &y)
{
    uint8_t data[15] = {0};
    if (!ensureTouchReady() || !touchReadFrame(data, sizeof(data))) return false;
    if (data[2] == 0) return false;
    x = static_cast<int16_t>(((static_cast<uint16_t>(data[3] & 0x0F) << 8) | data[4]));
    y = static_cast<int16_t>(((static_cast<uint16_t>(data[5] & 0x0F) << 8) | data[6]));
    transformTouchByRotation(x, y);
    return true;
}

bool displayCompatPanelReady()
{
    return ensurePanelReady();
}

#endif
