// Bench test for two chained 1x8 WS2812B sticks (16 LEDs) on GPIO18.
// Lights one pixel at a time, in chain order 0..15: pixels 0-7 red (first
// stick), 8-15 blue (second stick), so you can see each stick's direction and
// which one is first. Then all 16 dim white for 2s. Repeats. Brightness and
// current are capped hard - safe to run from the ESP32's USB 5V.
#include <FastLED.h>

#define LED_PIN 18
#define NUM_LEDS 16

CRGB leds[NUM_LEDS];

void setup() {
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(30);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 150);
}

void loop() {
  for (int i = 0; i < NUM_LEDS; i++) {
    FastLED.clear();
    leds[i] = i < 8 ? CRGB::Red : CRGB::Blue;
    FastLED.show();
    delay(500);
  }
  fill_solid(leds, NUM_LEDS, CRGB::White);
  FastLED.show();
  delay(2000);
}
