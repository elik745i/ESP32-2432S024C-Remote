#ifdef BACKLIGHT_TEST_BUILD

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#ifndef TFT_BL
#define TFT_BL 13
#endif

#ifndef RGB_LED_PIN
#define RGB_LED_PIN 0
#endif

namespace {

Adafruit_NeoPixel pixel(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
constexpr int LCD_PIN_IM0 = 47;
constexpr int LCD_PIN_IM1 = 48;

void setRgb(uint8_t red, uint8_t green, uint8_t blue)
{
    pixel.setPixelColor(0, pixel.Color(red, green, blue));
    pixel.show();
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

    Serial.println();
    Serial.println("[TEST] UEDX backlight test sketch starting");
    Serial.printf("[TEST] Backlight pin: GPIO%d\n", TFT_BL);
    Serial.printf("[TEST] RGB pixel pin: GPIO%d\n", RGB_LED_PIN);
    Serial.printf("[TEST] display straps: IM0=GPIO%d IM1=GPIO%d\n", LCD_PIN_IM0, LCD_PIN_IM1);
}

void loop()
{
    Serial.println("[TEST] backlight GPIO HIGH, RGB RED");
    setBacklight(true);
    setRgb(64, 0, 0);
    delay(2000);

    Serial.println("[TEST] backlight GPIO LOW, RGB BLUE");
    setBacklight(false);
    setRgb(0, 0, 64);
    delay(2000);
}

#endif