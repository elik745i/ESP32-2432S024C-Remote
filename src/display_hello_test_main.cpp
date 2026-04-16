#ifdef DISPLAY_HELLO_TEST_BUILD

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "display_compat.h"

#ifndef TFT_BL
#define TFT_BL 13
#endif

#ifndef RGB_LED_PIN
#define RGB_LED_PIN 0
#endif

namespace {

TFT_eSPI tft;
Adafruit_NeoPixel pixel(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
constexpr int LCD_PIN_IM0 = 47;
constexpr int LCD_PIN_IM1 = 48;

constexpr uint8_t FONT_SCALE = 6;
constexpr uint8_t GLYPH_W = 5;
constexpr uint8_t GLYPH_H = 7;

const uint8_t GLYPH_H_BITMAP[GLYPH_H] = {
    0b10001,
    0b10001,
    0b10001,
    0b11111,
    0b10001,
    0b10001,
    0b10001,
};

const uint8_t GLYPH_E_BITMAP[GLYPH_H] = {
    0b11111,
    0b10000,
    0b10000,
    0b11110,
    0b10000,
    0b10000,
    0b11111,
};

const uint8_t GLYPH_L_BITMAP[GLYPH_H] = {
    0b10000,
    0b10000,
    0b10000,
    0b10000,
    0b10000,
    0b10000,
    0b11111,
};

const uint8_t GLYPH_O_BITMAP[GLYPH_H] = {
    0b01110,
    0b10001,
    0b10001,
    0b10001,
    0b10001,
    0b10001,
    0b01110,
};

const uint8_t *glyphFor(char ch)
{
    switch (ch) {
        case 'H': return GLYPH_H_BITMAP;
        case 'E': return GLYPH_E_BITMAP;
        case 'L': return GLYPH_L_BITMAP;
        case 'O': return GLYPH_O_BITMAP;
        default: return nullptr;
    }
}

void setBacklight(bool on)
{
    digitalWrite(TFT_BL, on ? HIGH : LOW);
}

void configureDisplayStraps()
{
    pinMode(LCD_PIN_IM0, OUTPUT);
    pinMode(LCD_PIN_IM1, OUTPUT);
    digitalWrite(LCD_PIN_IM0, HIGH);
    digitalWrite(LCD_PIN_IM1, HIGH);
}

void setRgb(uint8_t red, uint8_t green, uint8_t blue)
{
    pixel.setPixelColor(0, pixel.Color(red, green, blue));
    pixel.show();
}

void drawGlyph(int16_t x, int16_t y, char ch, uint16_t color)
{
    const uint8_t *glyph = glyphFor(ch);
    if (!glyph) return;

    for (uint8_t row = 0; row < GLYPH_H; ++row) {
        for (uint8_t col = 0; col < GLYPH_W; ++col) {
            const bool pixelOn = (glyph[row] & (1U << (GLYPH_W - 1 - col))) != 0;
            if (!pixelOn) continue;
            tft.fillRect(x + (col * FONT_SCALE), y + (row * FONT_SCALE), FONT_SCALE, FONT_SCALE, color);
        }
    }
}

void drawHello(int16_t x, int16_t y, uint16_t color)
{
    const char *text = "HELLO";
    const int16_t advance = static_cast<int16_t>((GLYPH_W + 1) * FONT_SCALE);
    while (*text) {
        drawGlyph(x, y, *text, color);
        x = static_cast<int16_t>(x + advance);
        ++text;
    }
}

void drawTestPattern()
{
    const int16_t width = tft.width();
    const int16_t height = tft.height();
    const int16_t stripeH = height / 4;

    tft.fillScreen(TFT_BLACK);
    tft.fillRect(0, 0, width, stripeH, TFT_RED);
    tft.fillRect(0, stripeH, width, stripeH, TFT_GREEN);
    tft.fillRect(0, stripeH * 2, width, stripeH, TFT_WHITE);
    tft.fillRect(0, stripeH * 3, width, height - (stripeH * 3), TFT_BLACK);

    tft.drawRoundRect(4, 4, width - 8, height - 8, 0, TFT_WHITE);
    tft.drawFastHLine(0, height / 2, width, TFT_BLACK);

    const int16_t helloWidth = static_cast<int16_t>((5 * GLYPH_W + 4) * FONT_SCALE + (4 * FONT_SCALE));
    const int16_t helloHeight = static_cast<int16_t>(GLYPH_H * FONT_SCALE);
    const int16_t helloX = static_cast<int16_t>((width - helloWidth) / 2);
    const int16_t helloY = static_cast<int16_t>((height - helloHeight) / 2);

    tft.fillRect(helloX - 10, helloY - 10, helloWidth + 20, helloHeight + 20, TFT_BLACK);
    drawHello(helloX, helloY, TFT_WHITE);
}

} // namespace

void setup()
{
    pinMode(TFT_BL, OUTPUT);
    setBacklight(true);
    configureDisplayStraps();

    Serial.begin(115200);
    delay(200);

    pixel.begin();
    pixel.clear();
    pixel.show();
    setRgb(0, 0, 32);

    Serial.println();
    Serial.println("[TEST] display hello test starting");
    Serial.printf("[TEST] backlight pin: GPIO%d\n", TFT_BL);
    Serial.printf("[TEST] rgb pixel pin: GPIO%d\n", RGB_LED_PIN);
    Serial.printf("[TEST] display straps: IM0=GPIO%d IM1=GPIO%d\n", LCD_PIN_IM0, LCD_PIN_IM1);

    tft.init();
    tft.setRotation(1);
    drawTestPattern();
    setRgb(0, 32, 0);

    Serial.println("[TEST] panel init returned");
    Serial.println("[TEST] expected screen: red/green/white/black bands and HELLO in center");
}

void loop()
{
    static unsigned long lastMs = 0;
    if (millis() - lastMs < 2000UL) return;
    lastMs = millis();
    static bool toggle = false;
    toggle = !toggle;
    setRgb(toggle ? 32 : 0, toggle ? 32 : 0, 0);
    Serial.println("[TEST] running");
}

#endif