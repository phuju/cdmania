// CD player front panel - ESP32 WROOM-32 DevKit + ST7735R 1.8" TFT (160x128
// landscape, SPI), USB serial only.
//
// Talks to cd_player.py on the host. Sends: STOP, PLAY_BUTTON (pause toggle, or
// "play" from STANDBY), NEXT/PREV (track skip), POT:<0-100> (volume), PONG.
// Receives: STANDBY:, PLAY:, PLAY_STATUS:, VU:<64 levels>, LR:<left>,<right>, PING.
//
// STOP never ejects: the host locks the drive at startup and the disc is lifted
// out by hand. The STOP button (GPIO14), HOME during playback, and a long-press
// on PLAY/PAUSE all send the same rate-limited STOP.
//
// Forked from DiscStation's DiscStation.ino with the burn/rip menus, Wi-Fi and
// TCP remote removed. Originally drove an SSD1306 OLED + two WS2812B LED
// sticks for stereo meters; the OLED and (fried) LEDs were replaced by this
// one color TFT, which now draws the stereo meters on-screen instead.

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include "esp_task_wdt.h"
#include <Update.h>
#include "mbedtls/base64.h"

// Injected by scripts/build-firmware.sh as -DFW_VERSION=x.y.z; ad-hoc builds report "dev".
#define FW_STR_(x) #x
#define FW_STR(x) FW_STR_(x)
#ifdef FW_VERSION
  #define FW_VERSION_STR FW_STR(FW_VERSION)
#else
  #define FW_VERSION_STR "dev"
#endif

#define SCREEN_WIDTH  160
#define SCREEN_HEIGHT 128
#define TFT_CS   5
#define TFT_DC   16   // labeled RX2 on some 30-pin boards
#define TFT_RST  17   // labeled TX2 on some 30-pin boards

#define BTN_STOP_PIN      14
#define BTN_HOME_PIN      13
#define BTN_PLAYPAUSE_PIN 15
#define ENC_CLK_PIN       32
#define ENC_DT_PIN        33
#define ENC_SW_PIN         4  // encoder click

#define DEBOUNCE_MS        50
#define LONG_PRESS_MS      1000
#define ENC_CLICK_GUARD_MS 200   // rotation vibration can bounce SW low; ignore clicks this soon after a tick
#define STOP_MSG_MS        3000  // how long "Stopping..." shows before returning to STANDBY
#define STOP_COOLDOWN_MS   750   // drop STOP triggers this soon after the last: caps a noisy/faulty
                                 // button to a trickle instead of a flood (seen during bring-up)
#define IDLE_BLANK_MS      15000 // no input this long in STANDBY (or paused) -> spinning-disc screensaver
#define SAVER_FRAME_MS     90
#define PING_TIMEOUT_MS    30000 // no message from the host this long -> DISCONNECTED screen

#define VU_BARS            64    // must match the host's VU_BARS
#define VU_TIMEOUT_MS      1200  // no VU:/LR: frame this long -> fall back to the text screen / dark meters
#define VU_ENTRY_DELAY_MS  10000 // text screen this long after playback starts, then bars+meters
#define VU_RESUME_DELAY_MS 7000  // ...and this long after any input during PLAY
#define VU_TOP_MARGIN      10    // px of headroom above the tallest bar/meter
#define VIZ_FRAME_MS       33    // bars+meters redraw at ~30fps, independent of when host frames arrive
#define BAR_RELEASE_S      0.12f // bar ease time constant, both rise and fall - higher = calmer/smoother
#define PEAK_HOLD_S        0.28f // a peak cap hangs this long before falling
#define PEAK_GRAVITY       200.0f // px/s^2 the cap accelerates downward once released

// Stereo meter columns, to the right of the spectrum bars (see drawPlayVisualizer).
#define METER_W     8
#define METER_GAP   2
#define METER_BLOCK (METER_W * 2 + METER_GAP)
#define SEGMENT_LIT_H 3   // LED-block look: lit px, then a gap, repeating up the meter column
#define SEGMENT_GAP_H 1

Adafruit_ST7735 display(TFT_CS, TFT_DC, TFT_RST);
// Off-screen RAM frame for drawPlayVisualizer(): drawing ~150 small shapes
// directly to the panel each frame meant ~150 separate SPI address-window
// commands/frame, slow enough to fall behind the 33ms budget and show a
// part-drawn frame (the strobe). Composing in RAM first (cheap) and pushing
// the finished frame in one drawRGBBitmap() burst (one address-window, one
// write) fixes both the speed and the tearing - the panel only ever receives
// a complete frame.
GFXcanvas16 vizCanvas(SCREEN_WIDTH, SCREEN_HEIGHT);

enum UiState { UI_STOPPING, UI_PLAY, UI_STANDBY, UI_DISCONNECTED };
UiState uiState = UI_STANDBY;

String line1 = "";   // PLAY screen: track line
String line2 = "";
bool displayOk = false;
int displayRotation = 1;   // 1=landscape, 3=landscape upside-down; flipped by the encoder in STANDBY

unsigned long returnToStandbyAt = 0;
unsigned long lastStopSentAt = 0;
unsigned long lastMsgTime = 0;
unsigned long lastInputTime = 0;   // last input/screen change; drives the idle screensaver
bool displayBlank = false;         // true = screensaver running; any input restores the UI
unsigned long lastSaverFrame = 0;
int saverStep = 0;
// The text screen gets exactly one full clear when first entered (set true
// below); a volume tick then only redraws the one line that changed, not the
// whole panel. (The bars screen doesn't need this - see vizCanvas above.)
bool playTextInited = false;

int playVolume = 50;
bool playSeekMode = false;   // false: encoder = volume; true: encoder = track skip
bool isPaused = false;       // the screensaver may only take over PLAY while paused

// Spectrum visualizer: the host streams "VU:v0,v1,...". While frames keep
// arriving PLAY shows bars; VU_TIMEOUT_MS after they stop it falls back to text.
uint8_t vuLevel[VU_BARS];    // latest host targets 0-63
float barH[VU_BARS];           // smoothed bar height in px
unsigned long lastVizAt = 0;
float peakH[VU_BARS];          // peak cap height in px above the baseline
float peakVel[VU_BARS];        // its current fall speed
float peakHold[VU_BARS];       // seconds left to hang before it starts falling
bool visualizerActive = false;
unsigned long lastVuAt = 0;
unsigned long vuSuppressUntil = 0;   // bars withheld (text shown) until millis() reaches this
bool vuSeenReal = false;   // has this PLAY session ever seen a nonzero bar? Decided once per session,
                           // deliberately separate from visualizerActive/lastVuAt (stream liveness,
                           // which must also count legitimate all-zero frames) - merging the two
                           // made bars and screensaver flicker on quiet frames.

// Stereo level meters: drawn as two columns on the visualizer screen (see
// drawPlayVisualizer). LR: frames carry 0-63 per channel at ~30fps; if they
// stop arriving the columns are treated as silent (same VU_TIMEOUT_MS as the
// spectrum bars, reusing lastLrAt), no separate clear/redraw lifecycle needed.
// ponytail: only shown alongside the bars screen, not the plain PLAY text
// screen (the old physical LEDs were separate hardware and showed immediately);
// move this into drawPlay() too if you want them visible the whole time.
uint8_t lrLevel[2];
unsigned long lastLrAt = 0;

// Standard fixed-point HSV->RGB565 (hue 0-359, sat/val 0-255). Adafruit_GFX has
// no HSV helper, and FastLED's CHSV went with the dead LED code in Phase 12 -
// needed now for the neon per-bar hue sweep and glow brightness falloff below.
uint16_t hsv565(uint16_t hue, uint8_t sat, uint8_t val) {
  uint8_t region = hue / 60;
  uint8_t remainder = (hue % 60) * 255 / 60;
  uint8_t p = (val * (255 - sat)) / 255;
  uint8_t q = (val * (255 - ((uint16_t)sat * remainder) / 255)) / 255;
  uint8_t t = (val * (255 - ((uint16_t)sat * (255 - remainder)) / 255)) / 255;
  uint8_t r, g, b;
  switch (region) {
    case 0: r = val; g = t;   b = p;   break;
    case 1: r = q;   g = val; b = p;   break;
    case 2: r = p;   g = val; b = t;   break;
    case 3: r = p;   g = q;   b = val; break;
    case 4: r = t;   g = p;   b = val; break;
    default: r = val; g = p;  b = q;   break;
  }
  return display.color565(r, g, b);
}

// Purple (low) -> light blue (high) gradient, same endpoints as the old LED
// meters, scaled by `value` (0-255) for a brighter-top/dimmer-base glow falloff.
uint16_t meterColor(uint8_t level63, uint8_t value) {
  int t = (int)level63 * 255 / 63;
  uint8_t r = 150 + ((70 - 150) * t) / 255;
  uint8_t g = (190 * t) / 255;
  uint8_t b = 255;
  return display.color565((r * value) / 255, (g * value) / 255, (b * value) / 255);
}

// --- Buttons: one debounced press/release tracker each ---
struct Btn { uint8_t pin; bool down; unsigned long downAt, lastRelease; };
Btn stopBtn{BTN_STOP_PIN}, homeBtn{BTN_HOME_PIN}, playBtn{BTN_PLAYPAUSE_PIN}, encBtn{ENC_SW_PIN};

// True once on release; *heldMs = how long it was held. allowPress=false ignores a new press.
bool btnReleased(Btn& b, unsigned long* heldMs = nullptr, bool allowPress = true) {
  bool low = digitalRead(b.pin) == LOW;
  if (low && !b.down && allowPress && millis() - b.lastRelease > DEBOUNCE_MS) {
    b.down = true;
    b.downAt = millis();
  }
  if (!low && b.down) {
    b.down = false;
    b.lastRelease = millis();
    if (heldMs) *heldMs = millis() - b.downAt;
    return true;
  }
  return false;
}

// --- Rotary encoder: Ben Buxton's full-step quadrature state machine ---
// Reads both pins on every change and only commits a tick after a complete valid
// 4-transition sequence, so contact bounce needs no time debounce.
#define ENC_R_START     0x0
#define ENC_R_CW_FINAL  0x1
#define ENC_R_CW_BEGIN  0x2
#define ENC_R_CW_NEXT   0x3
#define ENC_R_CCW_BEGIN 0x4
#define ENC_R_CCW_FINAL 0x5
#define ENC_R_CCW_NEXT  0x6
#define ENC_DIR_CW  0x10
#define ENC_DIR_CCW 0x20

const uint8_t ENC_TTABLE[7][4] = {
  {ENC_R_START,    ENC_R_CW_BEGIN,  ENC_R_CCW_BEGIN, ENC_R_START},
  {ENC_R_CW_NEXT,  ENC_R_START,     ENC_R_CW_FINAL,  ENC_R_START | ENC_DIR_CW},
  {ENC_R_CW_NEXT,  ENC_R_CW_BEGIN,  ENC_R_START,     ENC_R_START},
  {ENC_R_CW_NEXT,  ENC_R_CW_BEGIN,  ENC_R_CW_FINAL,  ENC_R_START},
  {ENC_R_CCW_NEXT, ENC_R_START,     ENC_R_CCW_BEGIN, ENC_R_START},
  {ENC_R_CCW_NEXT, ENC_R_CCW_FINAL, ENC_R_START,     ENC_R_START | ENC_DIR_CCW},
  {ENC_R_CCW_NEXT, ENC_R_CCW_FINAL, ENC_R_CCW_BEGIN, ENC_R_START},
};

volatile uint8_t encState = ENC_R_START;
volatile int16_t encTicks = 0;             // whole detents waiting for loop() to drain
volatile uint32_t encLastActivityMs = 0;   // every raw pin transition, not just committed ticks

void IRAM_ATTR encoderISR() {
  encLastActivityMs = millis();
  uint8_t pinState = (digitalRead(ENC_DT_PIN) << 1) | digitalRead(ENC_CLK_PIN);
  encState = ENC_TTABLE[encState & 0xF][pinState];
  uint8_t dir = encState & 0x30;
  // Sign flipped from the table so clockwise = increase (next track, volume up).
  if (dir == ENC_DIR_CW) encTicks--;
  else if (dir == ENC_DIR_CCW) encTicks++;
}

// --- Drawing ---
void printUpper(String value) {
  value.toUpperCase();
  display.print(value);
}

void drawHeader() {
  display.setRotation(displayRotation);
  display.setTextSize(1);
  display.setTextColor(ST77XX_WHITE);
  // corner ticks
  display.drawLine(0, 0, 5, 0, ST77XX_WHITE);
  display.drawLine(0, 0, 0, 5, ST77XX_WHITE);
  display.drawLine(154, 0, 159, 0, ST77XX_WHITE);
  display.drawLine(159, 0, 159, 5, ST77XX_WHITE);
  display.drawLine(0, 127, 5, 127, ST77XX_WHITE);
  display.drawLine(0, 122, 0, 127, ST77XX_WHITE);
  display.drawLine(154, 127, 159, 127, ST77XX_WHITE);
  display.drawLine(159, 122, 159, 127, ST77XX_WHITE);
  display.setCursor(6, 2);
  display.print("CD PLAYER");
  display.drawLine(8, 13, 151, 13, ST77XX_WHITE);
}

void drawStopping() {
  uiState = UI_STOPPING;
  playTextInited = false;
  if (!displayOk) return;
  display.fillScreen(ST77XX_BLACK);
  drawHeader();
  display.setCursor(4, 40);
  display.print("STOPPING...");
}

void drawStandby() {
  uiState = UI_STANDBY;
  returnToStandbyAt = 0;
  lastInputTime = millis();
  displayBlank = false;
  playTextInited = false;
  if (!displayOk) return;
  display.fillScreen(ST77XX_BLACK);
  drawHeader();
  display.setCursor(4, 40);
  display.print("STANDBY // READY");
  display.setCursor(4, 58);
  display.print("INSERT DISC");
}

void drawDisconnected() {
  uiState = UI_DISCONNECTED;
  returnToStandbyAt = 0;
  displayBlank = false;
  playTextInited = false;
  if (!displayOk) return;
  display.fillScreen(ST77XX_BLACK);
  drawHeader();
  display.setCursor(4, 40);
  display.print("DISCONNECTED // LINK");
  display.setCursor(4, 58);
  display.print("CHECK USB");
}

// Spectrum bars (left) + stereo L/R level meters (right columns). 64 thin bars
// (1px wide, 2px pitch) with a peak cap on each that jumps to the bar's
// highest point, hangs briefly, then falls under gravity until the bar catches
// it; the meter columns use the old LEDs' purple->light-blue gradient. No
// header. Falls back to drawPlay() once VU: frames stop (see VU_TIMEOUT_MS).
void drawPlayVisualizer() {
  uiState = UI_PLAY;
  returnToStandbyAt = 0;
  if (!displayOk) return;
  playTextInited = false;
  const int maxH = SCREEN_HEIGHT - VU_TOP_MARGIN;
  const int pitch = 2;
  unsigned long now = millis();
  float dt = min((now - lastVizAt) / 1000.0f, 0.1f);
  lastVizAt = now;
  float k = 1.0f - expf(-dt / BAR_RELEASE_S);
  // Composed entirely in RAM (vizCanvas), so clearing + redrawing everything
  // every frame is free, unlike on the live panel - no stray-pixel bookkeeping
  // needed, and nothing is ever visible here until the one blit below.
  vizCanvas.fillScreen(ST77XX_BLACK);
  for (int i = 0; i < VU_BARS; i++) {
    float t = vuLevel[i] * maxH / 63.0f;
    barH[i] += (t - barH[i]) * k;   // symmetric ease, rise and fall alike - no more instant snap on rise
    if (barH[i] >= peakH[i]) {
      peakH[i] = barH[i];
      peakVel[i] = 0;
      peakHold[i] = PEAK_HOLD_S;
    } else if (peakHold[i] > 0) {
      peakHold[i] -= dt;
    } else {
      peakVel[i] += PEAK_GRAVITY * dt;
      peakH[i] -= peakVel[i] * dt;
      if (peakH[i] < barH[i]) peakH[i] = barH[i];
    }
    int h = (int)(barH[i] + 0.5f);
    int x = i * pitch;
    // Neon hue sweep across the 64 bands: magenta (bass, i=0) -> blue -> cyan
    // (treble, i=63), matching the meters' purple->blue family.
    uint16_t hue = 320 - (i * 130) / (VU_BARS - 1);
    if (h > 0) {
      // Vertical "tube glow": brightest at the top of the lit bar, fading
      // toward its base - per-pixel instead of one flat drawFastVLine, still
      // O(h) and RAM-only, so cheap even at 64 bars/frame.
      for (int ry = 0; ry < h; ry++) {
        uint8_t val = 255 - (h > 1 ? (ry * 140) / (h - 1) : 0);
        vizCanvas.drawPixel(x, SCREEN_HEIGHT - h + ry, hsv565(hue, 255, val));
      }
      // Horizontal halo bleed into the always-empty 1px gap column, scaled by
      // this bar's height - gives the glow real spread with no blur math.
      uint8_t glowVal = (uint8_t)(h * 160L / maxH);
      if (glowVal > 0) vizCanvas.drawFastVLine(x + 1, SCREEN_HEIGHT - h, h, hsv565(hue, 180, glowVal));
    }
    if (peakH[i] >= 1) vizCanvas.drawPixel(x, SCREEN_HEIGHT - (int)(peakH[i] + 0.5f) - 2, ST77XX_WHITE);
  }

  if ((long)(millis() - lastLrAt) >= VU_TIMEOUT_MS) { lrLevel[0] = 0; lrLevel[1] = 0; }
  int lx = SCREEN_WIDTH - METER_BLOCK, rx = lx + METER_W + METER_GAP;
  for (int ch = 0; ch < 2; ch++) {
    int h = lrLevel[ch] * maxH / 63;
    if (h <= 0) continue;
    int x = ch == 0 ? lx : rx;
    // Same brighter-top/dimmer-base glow falloff as the bars, plus a regular
    // gap every few rows so the fill reads as stacked LED blocks, not one
    // smooth bar - closer to the physical LED sticks this screen replaced.
    for (int ry = 0; ry < h; ry++) {
      if (ry % (SEGMENT_LIT_H + SEGMENT_GAP_H) >= SEGMENT_LIT_H) continue;
      uint8_t val = 255 - (h > 1 ? (ry * 140) / (h - 1) : 0);
      vizCanvas.drawFastHLine(x, SCREEN_HEIGHT - h + ry, METER_W, meterColor(lrLevel[ch], val));
    }
  }
  // One address-window + one continuous burst to the real panel - the only
  // point this frame becomes visible, and always as a complete image.
  display.drawRGBBitmap(0, 0, vizCanvas.getBuffer(), SCREEN_WIDTH, SCREEN_HEIGHT);
}

// Erases just one text row (so a volume tick redraws one line, not the whole screen).
void clearLine(int y) {
  display.fillRect(0, y - 2, SCREEN_WIDTH, 12, ST77XX_BLACK);
}

void drawPlay() {
  uiState = UI_PLAY;
  returnToStandbyAt = 0;
  lastInputTime = millis();   // fresh content gets a full IDLE_BLANK_MS before any screensaver
  if (!displayOk) return;
  // One full clear+header on first entry to this screen; every call (first
  // time or not) only erases+redraws the four lines below, never the whole
  // panel - a volume tick used to call this on every detent and flash the
  // entire screen each time.
  if (!playTextInited) {
    display.fillScreen(ST77XX_BLACK);
    drawHeader();
    playTextInited = true;
  }
  // No outer startWrite()/endWrite() here: fillRect()/print() each manage their
  // own SPI transaction internally (see Adafruit_SPITFT::startWrite - it's not
  // reentrant), so wrapping them again nests transactions and hangs the task.
  clearLine(28);
  display.setCursor(4, 28);
  printUpper(line1);
  clearLine(44);
  display.setCursor(4, 44);
  printUpper(line2);
  clearLine(64);
  display.setCursor(4, 64);
  if (playSeekMode) {
    display.print("TURN // SKIP TRACK");
  } else {
    display.print("VOL // ");
    display.print(playVolume);
    display.print("%");
  }
  clearLine(80);
  display.setCursor(4, 80);
  display.print("CLICK // ");
  display.print(playSeekMode ? "VOL MODE" : "SKIP MODE");
}

// 24-step sine table x64: sin(i*15deg)*64. Drives the disc screensaver.
const int8_t SIN24[24] = {
  0, 17, 32, 45, 55, 62, 64, 62, 55, 45, 32, 17,
  0, -17, -32, -45, -55, -62, -64, -62, -55, -45, -32, -17
};

// Spinning-disc screensaver frame.
void drawDiscSaver(int step) {
  if (!displayOk) return;
  const int cx = 80, cy = 64, R = 45;
  // No per-frame fillScreen(): the area outside the circle is static background,
  // cleared once when the screensaver starts (see loop()); the solid white
  // circle below already overwrites the previous frame's grooves on its own.
  display.fillCircle(cx, cy, R, ST77XX_WHITE);                 // vinyl body

  for (int cl = 0; cl < 2; cl++) {                              // 2 groove clusters, 180 apart
    int base = step + cl * 12;
    for (int g = 0; g < 4; g++) {                               // nested "sound wave" grooves
      for (int t = 0; t < 3; t++) {                             // 3px-thick stroke
        int r = 22 + g * 6 + t;
        int px = -200, py = -200;
        for (int a = 0; a <= 4; a++) {                          // ~60deg arc, 4 chords
          int i = (base + a) % 24;
          int x = cx + r * SIN24[(i + 6) % 24] / 64;
          int y = cy + r * SIN24[i] / 64;
          if (px > -200) display.drawLine(px, py, x, y, ST77XX_BLACK);
          px = x; py = y;
        }
      }
    }
  }

  display.fillCircle(cx, cy, 18, ST77XX_BLACK);                 // concentric label
  display.fillCircle(cx, cy, 15, ST77XX_WHITE);
  display.fillCircle(cx, cy, 11, ST77XX_BLACK);
  display.fillCircle(cx, cy, 7,  ST77XX_WHITE);
  display.fillCircle(cx, cy, 3,  ST77XX_BLACK);                 // center hole
}

// Redraws the real UI over a running screensaver. True if it woke the screen.
bool wakeDisplay() {
  if (!displayBlank || !displayOk) return false;
  displayBlank = false;
  switch (uiState) {
    case UI_STOPPING: drawStopping(); break;
    case UI_PLAY:
      if (vuSeenReal && visualizerActive && (long)(millis() - lastVuAt) < VU_TIMEOUT_MS &&
          (long)(millis() - vuSuppressUntil) >= 0) drawPlayVisualizer();
      else drawPlay();
      break;
    case UI_STANDBY: drawStandby(); break;
    case UI_DISCONNECTED: drawDisconnected(); break;
  }
  return true;
}

// --- Over-the-air update, pushed by the host over serial (stop-and-wait, one ACK per chunk) ---
bool otaActive = false;
size_t otaSize = 0, otaDone = 0;

void drawOtaProgress(int pct) {
  if (!displayOk) return;
  display.fillScreen(ST77XX_BLACK);
  drawHeader();
  display.setCursor(4, 30);
  display.print("UPDATING FIRMWARE");
  display.drawRect(8, 55, 144, 16, ST77XX_WHITE);
  display.fillRect(10, 57, (140 * pct) / 100, 12, ST77XX_WHITE);
  display.setCursor(4, 80);
  display.print("DO NOT POWER OFF");
}

void otaAbort(const char* why) {
  Update.abort();
  otaActive = false;
  Serial.print("OTA:ERR ");
  Serial.println(why);
  drawStandby();
}

// The image is MD5- and header-checked by Update before the slot switch, so a bad transfer never boots.
void handleOta(const String& msg) {
  lastMsgTime = millis();
  lastInputTime = millis();
  if (msg.startsWith("OTA_BEGIN:")) {
    int bar = msg.indexOf('|');
    otaSize = msg.substring(10, bar).toInt();
    String md5 = msg.substring(bar + 1);
    if (bar < 0 || otaSize == 0 || md5.length() != 32) { Serial.println("OTA:ERR bad request"); return; }
    displayBlank = false;
    if (!Update.begin(otaSize)) { Serial.println("OTA:ERR no space"); return; }
    Update.setMD5(md5.c_str());
    otaActive = true;
    otaDone = 0;
    drawOtaProgress(0);
    Serial.println("OTA:READY");
  } else if (!otaActive) {
    Serial.println("OTA:ERR not started");
  } else if (msg.startsWith("OTA_DATA:")) {
    uint8_t buf[512];
    size_t n = 0;
    if (mbedtls_base64_decode(buf, sizeof(buf), &n, (const uint8_t*)msg.c_str() + 9, msg.length() - 9) != 0 ||
        Update.write(buf, n) != n) { otaAbort("write failed"); return; }
    int before = otaDone * 100 / otaSize;
    otaDone += n;
    int pct = otaDone * 100 / otaSize;
    if (pct / 5 != before / 5) drawOtaProgress(pct);
    Serial.println("OTA:ACK");
  } else if (msg == "OTA_END") {
    if (otaDone == otaSize && Update.end(true)) {
      Serial.println("OTA:DONE");
      delay(500);
      ESP.restart();
    }
    otaAbort("verify failed");
  }
}

// --- Host messages ---
void parseMessage(String msg) {
  msg.trim();

  if (msg == "VERSION") {
    Serial.print("VERSION:");
    Serial.println(FW_VERSION_STR);
    return;
  }

  if (msg.startsWith("OTA_")) {
    handleOta(msg);
    return;
  }

  if (msg == "PING") {
    lastMsgTime = millis();
    Serial.println("PONG");
    if (uiState == UI_DISCONNECTED) drawStandby();
    return;
  }

  if (msg.startsWith("VU:")) {
    // Arrives ~30x/sec: no wakeDisplay()/RCV echo (would draw stale bars / spam the log).
    lastMsgTime = millis();
    bool anyNonzero = false;
    const char* p = msg.c_str() + 3;   // one char per bar: level = char - '0'
    for (int i = 0; i < VU_BARS; i++) {
      int c = *p ? *p++ - '0' : 0;   // constrain() is a macro: never pass it *p++
      uint8_t v = (uint8_t)constrain(c, 0, 63);
      vuLevel[i] = v;
      if (v) anyNonzero = true;
    }
    if (anyNonzero) vuSeenReal = true;
    // Liveness (visualizerActive/lastVuAt) updates on EVERY frame, zero or not:
    // real music produces all-zero frames, and treating those as "stream died"
    // flickers bars and screensaver.
    visualizerActive = true;
    lastVuAt = millis();
    lastInputTime = millis();
    // Only redraw bars once a real (nonzero) frame has been seen, so an
    // all-silent stream can't keep dragging the screen back from the screensaver.
    if (vuSeenReal && uiState == UI_PLAY && (long)(millis() - vuSuppressUntil) >= 0) displayBlank = false;
    return;
  }

  if (msg.startsWith("LR:")) {
    // ~30x/sec like VU: no wake, no echo, and it does not count as user activity.
    // Just stores the levels - drawPlayVisualizer()'s own 30fps loop paints them.
    lastMsgTime = millis();
    int comma = msg.indexOf(',');
    if (comma > 3) {
      lrLevel[0] = (uint8_t)constrain(msg.substring(3, comma).toInt(), 0, 63);
      lrLevel[1] = (uint8_t)constrain(msg.substring(comma + 1).toInt(), 0, 63);
      lastLrAt = millis();
    }
    return;
  }

  wakeDisplay();
  lastMsgTime = millis();
  Serial.print("RCV:");
  Serial.println(msg);

  if (msg.startsWith("STANDBY:")) {
    drawStandby();

  } else if (msg.startsWith("PLAY:")) {
    line1 = msg.substring(5);
    line2 = "Playing disc";
    playSeekMode = false;          // always start in volume mode
    isPaused = false;
    vuSuppressUntil = millis() + VU_ENTRY_DELAY_MS;   // text first, bars after a beat
    lastVuAt = 0;
    visualizerActive = false;
    vuSeenReal = false;
    memset(barH, 0, sizeof(barH));
    memset(peakH, 0, sizeof(peakH));
    memset(peakVel, 0, sizeof(peakVel));
    memset(peakHold, 0, sizeof(peakHold));
    Serial.print("POT:");          // push the last-used volume so playback starts at it
    Serial.println(playVolume);
    drawPlay();

  } else if (msg.startsWith("PLAY_STATUS:")) {
    line1 = msg.substring(12);
    if (line1.length() > 20) line1 = line1.substring(0, 20);
    if (line1 == "PLAYING") isPaused = false;
    else if (line1 == "PAUSED") isPaused = true;
    drawPlay();
  }
}

// Drains everything buffered on the link. Only the newest VU:/LR: frame in a batch
// is rendered - drawing each one made the display lag behind real time.
void drainAndDispatch() {
  String pendingVu = "", pendingLr = "";
  while (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    if (msg.length() == 0) continue;
    if (msg.startsWith("VU:")) pendingVu = msg;
    else if (msg.startsWith("LR:")) pendingLr = msg;
    else parseMessage(msg);
  }
  if (pendingVu.length() > 0) parseMessage(pendingVu);
  if (pendingLr.length() > 0) parseMessage(pendingLr);
}

// --- Input handlers ---
// Every input stamps activity. True if it only woke the screensaver (swallow the action).
bool inputWakesOnly() {
  bool woke = wakeDisplay();
  lastInputTime = millis();
  return woke;
}

// Shared by every STOP trigger; rate-limited by STOP_COOLDOWN_MS.
void sendStop() {
  if ((long)(millis() - lastStopSentAt) < STOP_COOLDOWN_MS) return;
  lastStopSentAt = millis();
  Serial.println("STOP");
  returnToStandbyAt = millis() + STOP_MSG_MS;
  drawStopping();
}

void handleStopButton() {
  if (inputWakesOnly()) return;
  if (uiState == UI_PLAY || uiState == UI_STANDBY || uiState == UI_DISCONNECTED) sendStop();
}

// HOME/BACK: STOP during PLAY, otherwise back to STANDBY.
void handleHomeButton() {
  if (inputWakesOnly()) return;
  if (uiState == UI_PLAY) sendStop();
  else if (uiState != UI_STANDBY) drawStandby();
}

// PLAY/PAUSE: in PLAY, short = pause toggle, long = STOP. From STANDBY a short
// press sends PLAY_BUTTON too - the host re-probes the drive and plays, which
// is how you resume after STOP without removing the disc.
void handlePlayPauseButton(bool longPress) {
  if (inputWakesOnly()) return;
  if (uiState == UI_PLAY) {
    vuSuppressUntil = millis() + VU_RESUME_DELAY_MS;   // any PLAY input -> text for a beat
    if (longPress) sendStop();
    else Serial.println("PLAY_BUTTON");
  } else if (uiState == UI_STANDBY && !longPress) {
    Serial.println("PLAY_BUTTON");
  }
}

// Encoder click: in PLAY swaps rotation between volume and track skip.
void handleEncoderClick() {
  if (inputWakesOnly()) return;
  if (uiState == UI_STOPPING || uiState == UI_DISCONNECTED) {
    drawStandby();   // manual escape if the host never answered
  } else if (uiState == UI_PLAY) {
    vuSuppressUntil = millis() + VU_RESUME_DELAY_MS;
    playSeekMode = !playSeekMode;
    drawPlay();
  }
}

// Encoder rotation, dir = +1 clockwise / -1 counter-clockwise. STANDBY flips the
// display; PLAY adjusts volume or skips tracks depending on playSeekMode.
void handleEncoder(int dir) {
  if (inputWakesOnly()) return;
  if (uiState == UI_STANDBY) {
    displayRotation = dir > 0 ? 3 : 1;
    drawStandby();
  } else if (uiState == UI_PLAY) {
    vuSuppressUntil = millis() + VU_RESUME_DELAY_MS;
    if (playSeekMode) {
      Serial.println(dir > 0 ? "NEXT" : "PREV");
    } else {
      playVolume = constrain(playVolume + 5 * dir, 0, 100);
      Serial.print("POT:");
      Serial.println(playVolume);
      drawPlay();
    }
  }
}

void setup() {
  setCpuFrequencyMhz(160);
  Serial.setRxBufferSize(1024);   // a 64-value VU: frame is ~200 bytes; the 256-byte default overflows
  Serial.begin(115200);
  esp_task_wdt_add(NULL);
  pinMode(BTN_STOP_PIN, INPUT_PULLUP);
  pinMode(BTN_HOME_PIN, INPUT_PULLUP);
  pinMode(BTN_PLAYPAUSE_PIN, INPUT_PULLUP);
  pinMode(ENC_SW_PIN, INPUT_PULLUP);
  pinMode(ENC_CLK_PIN, INPUT_PULLUP);
  pinMode(ENC_DT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT_PIN), encoderISR, CHANGE);

  display.initR(INITR_BLACKTAB);   // try INITR_GREENTAB if colors/offset look wrong on your panel
  // 27MHz caused an intermittent power-on brownout loop on this board (already
  // marginal from an earlier wiring short) - back to the library's own default.
  display.setSPISpeed(16000000);
  displayOk = true;
  Serial.println("Display OK");

  drawStandby();
  lastMsgTime = millis();
  Serial.println("CDPLAYER_READY");
}

void loop() {
  esp_task_wdt_reset();
  if (Serial.available()) drainAndDispatch();

  if (returnToStandbyAt != 0 && (long)(millis() - returnToStandbyAt) >= 0) drawStandby();

  // Mid-update the screen belongs to the progress bar; give up if the host goes silent.
  if (otaActive) {
    if ((long)(millis() - lastMsgTime) >= 15000) otaAbort("timeout");
    delay(2);
    return;
  }

  // Encoder rotation: drain ticks accumulated by the ISR.
  noInterrupts();
  int16_t ticks = encTicks;
  encTicks = 0;
  interrupts();
  while (ticks > 0) { handleEncoder(+1); ticks--; }
  while (ticks < 0) { handleEncoder(-1); ticks++; }

  unsigned long held = 0;
  if (btnReleased(encBtn, nullptr, millis() - encLastActivityMs > ENC_CLICK_GUARD_MS)) handleEncoderClick();
  if (btnReleased(stopBtn)) handleStopButton();
  if (btnReleased(homeBtn)) handleHomeButton();
  if (btnReleased(playBtn, &held)) handlePlayPauseButton(held >= LONG_PRESS_MS);

  // Host stopped sending VU: (paused/stopped): back to the text screen.
  if (visualizerActive && (long)(millis() - lastVuAt) >= VU_TIMEOUT_MS) {
    visualizerActive = false;
    if (uiState == UI_PLAY && !displayBlank) drawPlay();
  }

  // Idle screensaver: STANDBY, or PLAY while paused - never over live playback.
  if (displayOk && !displayBlank &&
      (uiState == UI_STANDBY || (uiState == UI_PLAY && isPaused)) &&
      (long)(millis() - lastInputTime) >= IDLE_BLANK_MS) {
    displayBlank = true;
    saverStep = 0;
    lastSaverFrame = 0;
    playTextInited = false;
    if (displayOk) display.fillScreen(ST77XX_BLACK);   // one-time clear; drawDiscSaver() never clears per-frame
  }
  if (displayBlank && displayOk && (long)(millis() - lastSaverFrame) >= SAVER_FRAME_MS) {
    lastSaverFrame = millis();
    drawDiscSaver(saverStep);
    saverStep = (saverStep + 1) % 24;
  }

  if (uiState != UI_DISCONNECTED && (long)(millis() - lastMsgTime) >= PING_TIMEOUT_MS) drawDisconnected();

  if (vuSeenReal && visualizerActive && uiState == UI_PLAY && !displayBlank &&
      (long)(millis() - vuSuppressUntil) >= 0 && millis() - lastVizAt >= VIZ_FRAME_MS) drawPlayVisualizer();

  delay(4);
}
