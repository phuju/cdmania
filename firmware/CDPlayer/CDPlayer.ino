// CD player front panel - ESP32 WROOM-32 DevKit + SSD1306 OLED, USB serial only.
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
// TCP remote removed.

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_task_wdt.h"
#include <FastLED.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define I2C_ADDRESS   0x3C

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

#define VU_BARS            64    // must match the host's VU_BARS (128px / 64 = 2px pitch: 1px bar + 1px gap)
#define VU_TIMEOUT_MS      1200  // no VU: frame this long -> fall back to the text screen
#define VU_ENTRY_DELAY_MS  10000 // text screen this long after playback starts, then bars
#define VU_RESUME_DELAY_MS 7000  // ...and this long after any input during PLAY
#define VU_TOP_MARGIN      16    // px of headroom above the tallest bar
#define VIZ_FRAME_MS       33    // bars redraw at ~30fps, independent of when host frames arrive
#define BAR_RELEASE_S      0.06f // bar fall time constant (attack is instant)
#define PEAK_HOLD_S        0.28f // a peak cap hangs this long before falling
#define PEAK_GRAVITY       200.0f // px/s^2 the cap accelerates downward once released

// Stereo level meters: two chained 1x8 WS2812B sticks (LEFT = pixels 0-7, RIGHT = 8-15).
// Powered from the ESP32's USB 5V, so brightness and current are capped hard.
#define LED_PIN            18
#define LED_STICK          8
#define LED_BRIGHTNESS     3     // of 255
#define LED_MAX_MA         150

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

enum UiState { UI_STOPPING, UI_PLAY, UI_STANDBY, UI_DISCONNECTED };
UiState uiState = UI_STANDBY;

String line1 = "";   // PLAY screen: track line
String line2 = "";
bool displayOk = false;
int displayRotation = 0;   // flipped by turning the encoder in STANDBY (upside-down mounting)

unsigned long returnToStandbyAt = 0;
unsigned long lastStopSentAt = 0;
unsigned long lastMsgTime = 0;
unsigned long lastInputTime = 0;   // last input/screen change; drives the idle screensaver
bool displayBlank = false;         // true = screensaver running; any input restores the UI
unsigned long lastSaverFrame = 0;
int saverStep = 0;

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

// --- LED stereo meters ---
// LR: frames carry 0-63 per channel; the host only sends them while audio plays,
// so the meters also clear when frames stop (paused/stopped) or the UI leaves PLAY.
CRGB leds[2 * LED_STICK];
uint8_t lrLevel[2];
bool ledsLit = false;
unsigned long lastLrAt = 0;

void clearMeters() {
  if (!ledsLit) return;
  ledsLit = false;
  FastLED.clear();
  FastLED.show();
}

void drawMeters() {
  ledsLit = true;
  FastLED.clear();
  for (int ch = 0; ch < 2; ch++) {
    int n = (lrLevel[ch] * LED_STICK + 31) / 63;   // 0-63 -> 0-8 LEDs lit
    for (int row = 0; row < n && row < LED_STICK; row++) {
      // Both bars grow upward. The right stick is mounted the other way round, so its
      // pixels run top-to-bottom: pixel 8 is the top row, pixel 15 the bottom.
      int px = ch == 0 ? row : 2 * LED_STICK - 1 - row;
      // Smooth RGB blend from purple (bottom) to light blue (top): mixing the two
      // colours directly gives a softer transition than stepping through hues.
      leds[px] = blend(CRGB(150, 0, 255), CRGB(70, 190, 255), row * 255 / (LED_STICK - 1));
    }
  }
  FastLED.show();
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
  display.setTextColor(SSD1306_WHITE);
  // corner ticks
  display.drawLine(0, 0, 4, 0, SSD1306_WHITE);
  display.drawLine(0, 0, 0, 4, SSD1306_WHITE);
  display.drawLine(123, 0, 127, 0, SSD1306_WHITE);
  display.drawLine(127, 0, 127, 4, SSD1306_WHITE);
  display.drawLine(0, 63, 4, 63, SSD1306_WHITE);
  display.drawLine(0, 59, 0, 63, SSD1306_WHITE);
  display.drawLine(123, 63, 127, 63, SSD1306_WHITE);
  display.drawLine(127, 59, 127, 63, SSD1306_WHITE);
  display.setCursor(4, 0);
  display.print("CD PLAYER");
  display.drawLine(6, 10, 121, 10, SSD1306_WHITE);
}

void drawStopping() {
  uiState = UI_STOPPING;
  if (!displayOk) return;
  display.clearDisplay();
  drawHeader();
  display.setCursor(2, 20);
  display.print("STOPPING...");
  display.display();
}

void drawStandby() {
  uiState = UI_STANDBY;
  returnToStandbyAt = 0;
  lastInputTime = millis();
  displayBlank = false;
  if (!displayOk) return;
  display.clearDisplay();
  drawHeader();
  display.setCursor(2, 25);
  display.print("STANDBY // READY");
  display.setCursor(2, 40);
  display.print("INSERT DISC");
  display.display();
}

void drawDisconnected() {
  uiState = UI_DISCONNECTED;
  returnToStandbyAt = 0;
  displayBlank = false;
  if (!displayOk) return;
  display.clearDisplay();
  drawHeader();
  display.setCursor(2, 25);
  display.print("DISCONNECTED // LINK");
  display.setCursor(2, 40);
  display.print("CHECK USB");
  display.display();
}

// Full-width spectrum: 64 thin bars (1px wide, 2px pitch) with a peak cap on each that jumps to
// the bar's highest point, hangs briefly, then falls under gravity until the bar catches it.
// No header. Falls back to drawPlay() once VU: frames stop (see VU_TIMEOUT_MS in loop()).
void drawPlayVisualizer() {
  uiState = UI_PLAY;
  returnToStandbyAt = 0;
  if (!displayOk) return;
  display.clearDisplay();
  const int maxH = SCREEN_HEIGHT - VU_TOP_MARGIN;
  const int pitch = SCREEN_WIDTH / VU_BARS;
  unsigned long now = millis();
  float dt = min((now - lastVizAt) / 1000.0f, 0.1f);
  lastVizAt = now;
  float k = 1.0f - expf(-dt / BAR_RELEASE_S);
  for (int i = 0; i < VU_BARS; i++) {
    float t = vuLevel[i] * maxH / 63.0f;
    barH[i] = t > barH[i] ? t : barH[i] + (t - barH[i]) * k;
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
    if (h > 0) display.drawFastVLine(x, SCREEN_HEIGHT - h, h, SSD1306_WHITE);
    if (peakH[i] >= 1) display.drawPixel(x, SCREEN_HEIGHT - (int)(peakH[i] + 0.5f) - 2, SSD1306_WHITE);   // 1px gap above the bar
  }
  display.display();
}

void drawPlay() {
  uiState = UI_PLAY;
  returnToStandbyAt = 0;
  lastInputTime = millis();   // fresh content gets a full IDLE_BLANK_MS before any screensaver
  if (!displayOk) return;
  display.clearDisplay();
  drawHeader();
  display.setCursor(2, 16);
  printUpper(line1);
  display.setCursor(2, 30);
  printUpper(line2);
  display.setCursor(2, 44);
  if (playSeekMode) {
    display.print("TURN // SKIP TRACK");
  } else {
    display.print("VOL // ");
    display.print(playVolume);
    display.print("%");
  }
  display.setCursor(2, 56);
  display.print("CLICK // ");
  display.print(playSeekMode ? "VOL MODE" : "SKIP MODE");
  display.display();
}

// 24-step sine table x64: sin(i*15deg)*64. Drives the disc screensaver.
const int8_t SIN24[24] = {
  0, 17, 32, 45, 55, 62, 64, 62, 55, 45, 32, 17,
  0, -17, -32, -45, -55, -62, -64, -62, -55, -45, -32, -17
};

// Spinning-disc screensaver frame.
void drawDiscSaver(int step) {
  if (!displayOk) return;
  const int cx = 64, cy = 32, R = 30;
  display.clearDisplay();
  display.fillCircle(cx, cy, R, SSD1306_WHITE);                 // vinyl body

  for (int cl = 0; cl < 2; cl++) {                              // 2 groove clusters, 180 apart
    int base = step + cl * 12;
    for (int g = 0; g < 3; g++) {                               // 3 nested "sound wave" grooves
      for (int t = 0; t < 3; t++) {                             // 3px-thick stroke
        int r = 16 + g * 5 + t;
        int px = -100, py = -100;
        for (int a = 0; a <= 4; a++) {                          // ~60deg arc, 4 chords
          int i = (base + a) % 24;
          int x = cx + r * SIN24[(i + 6) % 24] / 64;
          int y = cy + r * SIN24[i] / 64;
          if (px > -100) display.drawLine(px, py, x, y, SSD1306_BLACK);
          px = x; py = y;
        }
      }
    }
  }

  display.fillCircle(cx, cy, 13, SSD1306_BLACK);                // concentric label
  display.fillCircle(cx, cy, 11, SSD1306_WHITE);
  display.fillCircle(cx, cy, 8,  SSD1306_BLACK);
  display.fillCircle(cx, cy, 5,  SSD1306_WHITE);
  display.fillCircle(cx, cy, 2,  SSD1306_BLACK);                // center hole
  display.display();
}

// Redraws the real UI over a running screensaver. True if it woke the screen.
bool wakeDisplay() {
  if (!displayBlank || !displayOk) return false;
  display.ssd1306_command(0xAF);
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

// --- Host messages ---
void parseMessage(String msg) {
  msg.trim();

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
    // ~15x/sec like VU: no wake, no echo, and it does not count as user activity.
    lastMsgTime = millis();
    int comma = msg.indexOf(',');
    if (comma > 3) {
      lrLevel[0] = (uint8_t)constrain(msg.substring(3, comma).toInt(), 0, 63);
      lrLevel[1] = (uint8_t)constrain(msg.substring(comma + 1).toInt(), 0, 63);
      lastLrAt = millis();
      if (uiState == UI_PLAY) drawMeters();
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
    displayRotation = dir > 0 ? 2 : 0;
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
  setCpuFrequencyMhz(160);  // 240 -> 160: about half the CPU power, plenty for I2C at 400kHz
  Serial.setRxBufferSize(1024);   // a 64-value VU: frame is ~200 bytes; the 256-byte default overflows
  Serial.begin(115200);
  esp_task_wdt_add(NULL);
  Wire.begin(21, 22);
  Wire.setClock(400000);   // 800kHz left a stuck column on the OLED
  pinMode(BTN_STOP_PIN, INPUT_PULLUP);
  pinMode(BTN_HOME_PIN, INPUT_PULLUP);
  pinMode(BTN_PLAYPAUSE_PIN, INPUT_PULLUP);
  pinMode(ENC_SW_PIN, INPUT_PULLUP);
  pinMode(ENC_CLK_PIN, INPUT_PULLUP);
  pinMode(ENC_DT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT_PIN), encoderISR, CHANGE);

  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, 2 * LED_STICK);
  FastLED.setBrightness(LED_BRIGHTNESS);
  FastLED.setDither(BINARY_DITHER);   // at this brightness the blend only exists via temporal dithering
  FastLED.setMaxPowerInVoltsAndMilliamps(5, LED_MAX_MA);
  FastLED.clear(true);

  displayOk = display.begin(SSD1306_SWITCHCAPVCC, I2C_ADDRESS);
  if (displayOk) {
    Serial.println("Display OK");
  } else {
    Serial.println("Display FAILED");
    delay(3000);
  }

  drawStandby();
  lastMsgTime = millis();
  Serial.println("CDPLAYER_READY");
}

void loop() {
  esp_task_wdt_reset();
  if (Serial.available()) drainAndDispatch();

  if (returnToStandbyAt != 0 && (long)(millis() - returnToStandbyAt) >= 0) drawStandby();

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

  // Refresh every loop while lit: FastLED's temporal dithering only blends the dim
  // colours if show() runs far more often than the ~15fps LR: frames.
  if (ledsLit) FastLED.show();

  // Meters go dark when LR: frames stop (paused/stopped) or the UI leaves PLAY.
  if (ledsLit && (uiState != UI_PLAY || (long)(millis() - lastLrAt) >= VU_TIMEOUT_MS)) clearMeters();

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
