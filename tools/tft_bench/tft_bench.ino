// Bench test for a 1.8" ST7735R 128x160 TFT on hardware VSPI.
// Pins match the planned front-panel wiring:
//   CS=5  RESET=17(TX2)  DC/A0=16(RX2)  MOSI=23  SCK=18
// Cycles: red/green/blue/white fills, a border rect, and text - confirms the
// panel is alive and not mirrored/rotated before any real firmware work.
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

#define TFT_CS   5
#define TFT_RST  17
#define TFT_DC   16

Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  Serial.begin(115200);
  tft.initR(INITR_BLACKTAB);  // most common 1.8" ST7735R variant; try INITR_GREENTAB if colors/offset look wrong
  tft.setRotation(0);
}

void loop() {
  tft.fillScreen(ST77XX_RED);   delay(800);
  tft.fillScreen(ST77XX_GREEN); delay(800);
  tft.fillScreen(ST77XX_BLUE);  delay(800);
  tft.fillScreen(ST77XX_WHITE); delay(800);

  tft.fillScreen(ST77XX_BLACK);
  tft.drawRect(0, 0, tft.width(), tft.height(), ST77XX_WHITE);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.setCursor(10, 60);
  tft.print("CD PLAYER");
  tft.setTextSize(1);
  tft.setCursor(10, 90);
  tft.printf("%dx%d OK", tft.width(), tft.height());
  delay(3000);
}
