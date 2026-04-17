#pragma once

#if defined(BOARD_UEDX24320028E_WB_A) || defined(BOARD_UEDX32480035E_WB_A) || defined(BOARD_JC4880P443C_I_W)

#include <Arduino.h>
#include <stdint.h>

static constexpr uint16_t TFT_BLACK = 0x0000;
static constexpr uint16_t TFT_WHITE = 0xFFFF;
static constexpr uint16_t TFT_RED   = 0xF800;
static constexpr uint16_t TFT_GREEN = 0x07E0;
static constexpr uint16_t TFT_DARKGREY = 0x7BEF;

class TFT_eSPI {
public:
    TFT_eSPI() = default;

    void init();
    void setRotation(uint8_t rotation);
    void setTextFont(int font);
    void setTextColor(uint16_t fg, uint16_t bg = TFT_BLACK);

    int16_t width() const;
    int16_t height() const;

    void startWrite();
    void endWrite();
    void setAddrWindow(int32_t x, int32_t y, int32_t w, int32_t h);
    void pushColors(uint16_t *data, uint32_t len, bool swap);

    void fillScreen(uint16_t color);
    void fillRect(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t color);
    void fillRoundRect(int32_t x, int32_t y, int32_t w, int32_t h, int32_t r, uint16_t color);
    void drawFastHLine(int32_t x, int32_t y, int32_t w, uint16_t color);
    void drawLine(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint16_t color);
    void drawRoundRect(int32_t x, int32_t y, int32_t w, int32_t h, int32_t r, uint16_t color);
    void fillTriangle(int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint16_t color);

    uint16_t color565(uint8_t r, uint8_t g, uint8_t b) const;
    int16_t drawString(const char *str, int32_t x, int32_t y, int font = 1);
    int16_t drawCentreString(const char *str, int32_t x, int32_t y, int font = 1);

private:
    int32_t _windowX = 0;
    int32_t _windowY = 0;
    int32_t _windowW = 0;
    int32_t _windowH = 0;
    uint16_t _textFg = TFT_WHITE;
    uint16_t _textBg = TFT_BLACK;
};

bool displayCompatTouchRead(int16_t &x, int16_t &y);
bool displayCompatPanelReady();

#else

#include <TFT_eSPI.h>

#endif
