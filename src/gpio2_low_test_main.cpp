#include <Arduino.h>
#include <driver/gpio.h>

static constexpr gpio_num_t TEST_PIN = GPIO_NUM_2;
static constexpr unsigned long REPORT_MS = 1000UL;

static void clampPinLow()
{
    gpio_reset_pin(TEST_PIN);
    gpio_set_direction(TEST_PIN, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(TEST_PIN, GPIO_PULLDOWN_ONLY);
    gpio_set_level(TEST_PIN, 0);
    pinMode(static_cast<uint8_t>(TEST_PIN), OUTPUT);
    digitalWrite(static_cast<uint8_t>(TEST_PIN), LOW);
}

void setup()
{
    Serial.begin(115200);
    delay(250);
    clampPinLow();
    Serial.println("GPIO2 low clamp test starting");
}

void loop()
{
    static unsigned long lastReport = 0;

    clampPinLow();
    delay(10);

    const unsigned long now = millis();
    if (now - lastReport >= REPORT_MS) {
        lastReport = now;
        Serial.printf("GPIO2 level=%d\n", gpio_get_level(TEST_PIN));
    }
}
