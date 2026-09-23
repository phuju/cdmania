// CD Player front-panel firmware - ESP32 WROOM-32 DevKit + SSD1306 OLED.
//
// Forked from the DiscStation project's arduino/DiscStation/DiscStation.ino
// (not that file - this is a separate, dedicated sketch for a playback-only
// CD player). Kept from the original: OLED drawing primitives, the VU
// spectrum visualizer, the Wi-Fi/OTA connection lifecycle, the rotary
// encoder's quadrature decoder, and the screensaver. Stripped: the
// BURN/PLAY/RIP home menu, burn-mode/speed pickers, burn/rip progress UI,
// the IP/QR web-setup screen, and the TCP remote-control transport (nothing
// in this project uses a web/mobile remote - USB serial only).
//
// Button roles: EJECT (GPIO14) and a long-press on PLAY/PAUSE both send a
// single "STOP" - never a real eject. This drive is locked against ejection
// by the host at startup (SCSI PREVENT/ALLOW MEDIUM REMOVAL); the disc is
// meant to be lifted out by hand once stopped, not pushed out through the
// slot. See ~/Desktop/cd-player-project-findings.md for why.

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_task_wdt.h"
#include <WiFi.h>
#include "esp_mac.h"
#include <WiFiManager.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <ArduinoOTA.h>

// Optional build-time Wi-Fi override for power users: copy secrets.h.example
// to secrets.h (git-ignored) and set WIFI_SSID / WIFI_PASS / OTA_PASSWORD.
// If WIFI_SSID is defined the runtime setup portal is skipped entirely.
#if __has_include("secrets.h")
  #include "secrets.h"
#endif

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define I2C_ADDRESS   0x3C

#define BTN_STOP_PIN      14  // was EJECT - now always just STOPs, never ejects
#define BTN_HOME_PIN      13  // also carries the 10s Wi-Fi-reset hold
#define BTN_PLAYPAUSE_PIN 15
#define ENC_CLK_PIN       32
#define ENC_DT_PIN        33
#define ENC_SW_PIN         4  // encoder's click switch

#define DEBOUNCE_MS      50
#define LONG_PRESS_MS    1000
#define ENC_CLICK_GUARD_MS 200  // ignore new SW presses this soon after a rotation tick (rotation vibration can bounce SW low)
#define STOP_MSG_MS      3000   // how long "Stopping..." stays on screen before returning to STANDBY
#define STOP_COOLDOWN_MS  750   // ignore a new STOP trigger this soon after the last one - caps a
                                 // noisy/faulty button or wire to a low rate instead of a flood; a
                                 // real press is never anywhere near this fast
#define STANDBY_BLANK_MS 60000
#define IDLE_BLANK_MS    15000   // no input this long on STANDBY/PLAY -> spinning-disc screensaver
                                 // (this power bank's no-load auto-shutoff trips at ~30s idle; keep this
                                 // well under that so the screensaver's current draw beats it there)
#define SAVER_FRAME_MS   90      // screensaver frame interval (~11fps)
#define PING_TIMEOUT_MS  30000

#define VU_BARS       16   // must match the host's VU: band count
#define VU_TIMEOUT_MS 1200 // no VU: update this long -> host isn't sending (paused/stopped), fall back to text
#define VU_ENTRY_DELAY_MS  10000  // stay on the text status screen this long after entering PLAY
#define VU_RESUME_DELAY_MS  7000  // ...and this long after any input while already in PLAY

#define WIFI_RESET_HOLD_MS      10000  // hold HOME/BACK this long on STANDBY to wipe Wi-Fi creds
#define WIFI_CONNECT_TIMEOUT_MS 18000  // give a stored-creds join this long before falling to the portal
#define WIFI_RETRY_MS           15000  // if a live link drops, force a re-join after this

WiFiManager wm;
Preferences prefs;
bool wifiConnected = false;
bool wifiInitDone = false;
bool portalActive = false;
bool otaEnabled = false;
bool credsJustSaved = false;
bool wifiResetArmed = false;
unsigned long wifiDropAt = 0;
String apName = "CDPlayer";
String mdnsHost = "cdplayer";

void drawSetup();       // fwd decls (defined with the other draw* below)
void drawWifiReset();
void drawPlayVisualizer();

String deviceSuffix() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);  // factory MAC from eFuse - valid before the Wi-Fi driver starts
  char buf[5];
  snprintf(buf, sizeof(buf), "%02X%02X", mac[4], mac[5]);
  return String(buf);
}

void loadCreds(String& ssid, String& pass) {
  prefs.begin("cdplayer", true);
  ssid = prefs.getString("ssid", "");
  pass = prefs.getString("pass", "");
  prefs.end();
}

void saveCreds(const String& ssid, const String& pass) {
  prefs.begin("cdplayer", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();
}

void clearCreds() {
  prefs.begin("cdplayer", false);
  prefs.clear();
  prefs.end();
  WiFi.disconnect(true, true);   // also wipe the ESP's own persisted creds
}

void wmSaveCallback() {
  saveCreds(wm.getWiFiSSID(), wm.getWiFiPass());
  credsJustSaved = true;         // loop() reboots cleanly into STA mode
}

void onWifiUp() {
  portalActive = false;
  wifiConnected = true;
  wifiDropAt = 0;
  WiFi.setSleep(WIFI_PS_MIN_MODEM);
  MDNS.begin(mdnsHost.c_str());   // gives OTA a resolvable <mdnsHost>.local name
#ifdef OTA_PASSWORD
  ArduinoOTA.setHostname(mdnsHost.c_str());
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.begin();
  otaEnabled = true;
#endif
  Serial.print("WiFi OK "); Serial.println(WiFi.localIP());
}

void startPortal() {
  Serial.print("WiFi: setup portal '"); Serial.print(apName); Serial.println("'");
  portalActive = true;
  WiFi.mode(WIFI_AP_STA);
  wm.setConfigPortalBlocking(false);
  wm.setConfigPortalTimeout(0);
  wm.setSaveConfigCallback(wmSaveCallback);
  wm.startConfigPortal(apName.c_str());   // open AP at 192.168.4.1
  drawSetup();
}

void initWiFi() {
  apName   = "CDPlayer-" + deviceSuffix();
  mdnsHost = "cdplayer-" + deviceSuffix();
  mdnsHost.toLowerCase();

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(WIFI_PS_MIN_MODEM);

  String ssid, pass;
#ifdef WIFI_SSID
  ssid = WIFI_SSID;
  pass = WIFI_PASS;
#else
  loadCreds(ssid, pass);
#endif

  if (ssid.length() > 0) {
    Serial.print("WiFi "); Serial.print(ssid); Serial.print("...");
    WiFi.begin(ssid.c_str(), pass.c_str());
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_CONNECT_TIMEOUT_MS) {
      esp_task_wdt_reset();
      delay(200);
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" OK");
    onWifiUp();
  } else {
    if (ssid.length() > 0) Serial.println(" fail (2.4GHz band / wrong password?)");
    startPortal();
  }
}

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

enum UiState {
  UI_STATUS,
  UI_PLAY,
  UI_STANDBY,
  UI_DISCONNECTED,
  UI_SETUP
};

UiState uiState = UI_STATUS;

String line1 = "Status: READY";
String line2 = "";
String line3 = "";

bool displayOk = false;
unsigned long returnToHomeAt = 0;
unsigned long lastStopSentAt = 0;   // STOP_COOLDOWN_MS gate, see sendStop()
unsigned long playStatusAt = 0;
bool playStatusTemp = false;

// Encoder click (SW) - short/long press state
unsigned long encClickDownAt = 0;
bool encClickDown = false;
unsigned long encClickLastDebounce = 0;

// STOP button (physically the old EJECT button) - single press, no long-press timer needed
bool stopBtnDown = false;
unsigned long stopBtnLastDebounce = 0;

// HOME/BACK button - single press; also carries the 10s Wi-Fi-reset hold
unsigned long homeDownAt = 0;
bool homeDown = false;
unsigned long homeLastDebounce = 0;

// PLAY/PAUSE button - short = pause/resume, long = stop
unsigned long playpauseDownAt = 0;
bool playpauseDown = false;
unsigned long playpauseLastDebounce = 0;

int playVolume = 50;
bool playSeekMode = false;   // false = encoder rotate adjusts volume (default); true = track skip
unsigned long standbyStartTime = 0;
unsigned long lastMsgTime = 0;
unsigned long lastInputTime = 0;   // last button press / screen change; drives the idle screensaver
bool displayBlank = false;          // true = real UI hidden, disc screensaver running; any input restores it
unsigned long lastSaverFrame = 0;
int saverStep = 0;
bool isPaused = false;   // drives the screensaver during UI_PLAY - never blank while actively playing

// Spectrum visualizer: host streams real playback levels as "VU:v0,v1,...".
// While they keep arriving, PLAY shows full-screen bars instead of the usual
// status text; VU_TIMEOUT_MS after they stop (paused/stopped host-side) it
// falls back to the normal drawPlay() screen.
uint8_t vuLevel[VU_BARS];
bool visualizerActive = false;
unsigned long lastVuAt = 0;
unsigned long vuSuppressUntil = 0;   // bars withheld (text screen shown instead) until millis() reaches this
bool vuSeenReal = false;   // has this PLAY session ever gotten a VU: frame with a nonzero bar -
                           // a session-level fact, decided once and left alone. Deliberately
                           // separate from visualizerActive/lastVuAt, which track "is the
                           // stream still alive" per frame (including legitimate all-zero
                           // frames during quiet music) - conflating the two previously caused
                           // the bars/screensaver to flicker whenever a frame read all zero.
int displayRotation = 0;

// --- Rotary encoder decode ---
// Ben Buxton's full-step quadrature state machine. Reads BOTH pins on every
// change and only commits a tick after a complete, valid 4-transition
// sequence - contact bounce just walks between non-committing states instead
// of producing a spurious tick, so no time debounce is needed.
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
volatile int16_t encTicks = 0;   // whole detents ready for loop() to drain
volatile uint32_t encLastActivityMs = 0;   // refreshed on every raw pin transition, not just
                                            // committed ticks - guards handleSelectPress's SW
                                            // read against bounce/crosstalk from a spin in progress

void IRAM_ATTR encoderISR() {
  encLastActivityMs = millis();
  uint8_t pinState = (digitalRead(ENC_DT_PIN) << 1) | digitalRead(ENC_CLK_PIN);
  encState = ENC_TTABLE[encState & 0xF][pinState];
  uint8_t dir = encState & 0x30;
  // Flipped from the table's literal CW/CCW so the tick sign matches this
  // encoder's physical wiring: turning the knob clockwise should increase
  // (next track, volume up), not decrease.
  if (dir == ENC_DIR_CW) encTicks--;
  else if (dir == ENC_DIR_CCW) encTicks++;
}

void printUpper(String value) {
  value.toUpperCase();
  display.print(value);
}

void drawChrome() {
  display.setRotation(displayRotation);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.drawLine(0, 0, 4, 0, SSD1306_WHITE);
  display.drawLine(0, 0, 0, 4, SSD1306_WHITE);
  display.drawLine(123, 0, 127, 0, SSD1306_WHITE);
  display.drawLine(127, 0, 127, 4, SSD1306_WHITE);
  display.drawLine(0, 63, 4, 63, SSD1306_WHITE);
  display.drawLine(0, 59, 0, 63, SSD1306_WHITE);
  display.drawLine(123, 63, 127, 63, SSD1306_WHITE);
  display.drawLine(127, 59, 127, 63, SSD1306_WHITE);
}

void drawHeader() {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  drawChrome();
  display.setCursor(4, 0);
  display.print("CD PLAYER");
  display.drawLine(6, 10, 121, 10, SSD1306_WHITE);
}

void drawStatus() {
  uiState = UI_STATUS;
  if (!displayOk) return;

  display.clearDisplay();
  drawHeader();

  display.setCursor(2, 20);
  printUpper(line1);
  display.setCursor(2, 34);
  printUpper(line2);
  display.setCursor(2, 48);
  printUpper(line3);

  display.display();
}

void drawStandby() {
  // While the Wi-Fi setup portal is up and the player is otherwise idle,
  // the standby screen doubles as the setup instructions.
  if (portalActive) { drawSetup(); return; }
  uiState = UI_STANDBY;
  returnToHomeAt = 0;
  standbyStartTime = millis();
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

void drawSetup() {
  uiState = UI_SETUP;
  returnToHomeAt = 0;
  displayBlank = false;
  lastInputTime = millis();
  if (!displayOk) return;

  display.clearDisplay();
  drawHeader();
  display.setCursor(2, 16);
  display.print("WIFI SETUP");
  display.setCursor(2, 28);
  display.print("JOIN:");
  display.setCursor(2, 38);
  printUpper(apName);
  display.setCursor(2, 50);
  display.print("THEN 192.168.4.1");
  display.display();
}

void drawWifiReset() {
  uiState = UI_SETUP;
  displayBlank = false;
  if (!displayOk) return;
  display.clearDisplay();
  drawHeader();
  display.setCursor(2, 25);
  display.print("WIFI RESET");
  display.setCursor(2, 40);
  display.print("REBOOTING...");
  display.display();
}

void drawDisconnected() {
  uiState = UI_DISCONNECTED;
  returnToHomeAt = 0;
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

// 24-step sine table x64: sin(i*15deg)*64. Drives the idle disc screensaver.
const int8_t SIN24[24] = {
  0, 17, 32, 45, 55, 62, 64, 62, 55, 45, 32, 17,
  0, -17, -32, -45, -55, -62, -64, -62, -55, -45, -32, -17
};

// Spinning-disc screensaver frame - shown after IDLE_BLANK_MS on STANDBY/PLAY
// instead of powering the panel off. Keeps the OLED lit and the I2C bus busy so
// a USB power-bank feeding the remote doesn't hit its no-load auto-shutoff.
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

// Returns true if this call actually woke a blanked screen - callers use that
// to swallow the wake press so it doesn't also trigger a menu action.
bool wakeDisplay() {
  if (!displayBlank || !displayOk) return false;
  display.ssd1306_command(0xAF);
  displayBlank = false;
  switch (uiState) {
    case UI_STATUS: drawStatus(); break;
    case UI_PLAY:
      if (vuSeenReal && visualizerActive && (long)(millis() - lastVuAt) < VU_TIMEOUT_MS &&
          (long)(millis() - vuSuppressUntil) >= 0) drawPlayVisualizer();
      else drawPlay();
      break;
    case UI_STANDBY: drawStandby(); break;
    case UI_DISCONNECTED: drawDisconnected(); break;
    case UI_SETUP: drawSetup(); break;
  }
  return true;
}

#define VU_TOP_MARGIN 16   // px of empty headroom above the tallest possible bar

// Full-screen-width spectrum bars, no header/chrome, capped short of the top
// edge (VU_TOP_MARGIN) so a loud peak doesn't run the panel edge-to-edge.
// Falls back to the normal drawPlay() text screen once VU: updates stop
// arriving (see VU_TIMEOUT_MS in loop()).
void drawPlayVisualizer() {
  uiState = UI_PLAY;
  returnToHomeAt = 0;
  if (!displayOk) return;

  display.clearDisplay();
  const int gap = 2;
  const int maxH = SCREEN_HEIGHT - VU_TOP_MARGIN;
  const int barW = (SCREEN_WIDTH - gap * (VU_BARS - 1)) / VU_BARS;
  int x = (SCREEN_WIDTH - (barW * VU_BARS + gap * (VU_BARS - 1))) / 2;
  for (int i = 0; i < VU_BARS; i++) {
    int h = map(vuLevel[i], 0, 63, 0, maxH);
    if (h > 0) display.fillRect(x, SCREEN_HEIGHT - h, barW, h, SSD1306_WHITE);
    x += barW + gap;
  }
  display.display();
}

void drawPlay() {
  uiState = UI_PLAY;
  returnToHomeAt = 0;
  lastInputTime = millis();   // real status content (track change, pause/resume) gets a full
                               // IDLE_BLANK_MS on screen before the screensaver reclaims it
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

void parseMessage(String msg) {
  msg.trim();

  if (msg == "PING") {
    lastMsgTime = millis();
    Serial.println("PONG");
    if (uiState == UI_DISCONNECTED) drawStandby();
    return;
  }

  if (msg.startsWith("VU:")) {
    // No wakeDisplay()/RCV echo here - this arrives ~15x/sec while playing,
    // wakeDisplay() would draw stale bars a frame early, and echoing it
    // would spam the serial log for no reason.
    lastMsgTime = millis();
    String rest = msg.substring(3);
    for (int i = 0; i < VU_BARS; i++) vuLevel[i] = 0;
    bool anyNonzero = false;
    for (int i = 0; i < VU_BARS && rest.length() > 0; i++) {
      int comma = rest.indexOf(',');
      String tok = (comma < 0) ? rest : rest.substring(0, comma);
      vuLevel[i] = (uint8_t)constrain(tok.toInt(), 0, 63);
      if (vuLevel[i] > 0) anyNonzero = true;
      if (comma < 0) break;
      rest = rest.substring(comma + 1);
    }
    if (anyNonzero) vuSeenReal = true;
    // lastVuAt/visualizerActive/lastInputTime update on EVERY frame, zero or
    // not - they track "is the stream still alive", and real music routinely
    // produces a frame where every bar reads 0. Only gating those three on
    // anyNonzero would make ordinary quiet frames look like "host stopped
    // sending" to the unrelated VU_TIMEOUT_MS check below, flickering
    // bars/screensaver during normal playback.
    visualizerActive = true;
    lastVuAt = millis();
    lastInputTime = millis();   // the visualizer's own continuous redraw already beats the
                                 // power-bank shutoff - no need for the screensaver too
    // Drawing bars (and reclaiming the screen from the disc-spinner screensaver,
    // if it's up) is the one thing that DOES stay gated on vuSeenReal - a host
    // that's technically capturing but getting only silence sends real VU:
    // frames that are all zero forever. Without this gate, every one of those
    // frames would re-draw a blank bars screen and set displayBlank = false,
    // fighting the disc-spinner screensaver for the display instead of
    // leaving it alone once chosen.
    if (vuSeenReal && uiState == UI_PLAY && (long)(millis() - vuSuppressUntil) >= 0) {
      displayBlank = false;
      drawPlayVisualizer();
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
    isPaused = false;               // assume playing at track start
    vuSuppressUntil = millis() + VU_ENTRY_DELAY_MS;   // text screen first, bars after a beat
    lastVuAt = 0;                  // fresh session - unknown yet whether the host even sends VU: at all
    visualizerActive = false;
    vuSeenReal = false;            // unknown yet whether this session ever gets a real (nonzero) frame
    Serial.print("POT:");          // push the last-used volume so playback starts at it
    Serial.println(playVolume);
    drawPlay();

  } else if (msg.startsWith("PLAY_STATUS:")) {
    line1 = msg.substring(12);
    if (line1.length() > 20) line1 = line1.substring(0, 20);
    if (line1 == "PLAYING" || line1 == "PAUSED" || line1.startsWith("TRACK ")) {
      playStatusTemp = false;
      if (line1 == "PLAYING") isPaused = false;
      else if (line1 == "PAUSED") isPaused = true;
    } else {
      playStatusAt = millis();
      playStatusTemp = true;
    }
    drawPlay();

  } else if (msg.startsWith("STATUS:")) {
    returnToHomeAt = 0;
    line1 = msg.substring(7);
    line2 = "";
    line3 = "";
    drawStatus();

  } else if (msg.startsWith("WARNING:")) {
    returnToHomeAt = 0;
    line1 = "!! WARNING !!";
    line2 = msg.substring(8);
    if (line2.length() > 20) line2 = line2.substring(0, 20);
    line3 = "";
    drawStatus();

  } else if (msg.startsWith("ERROR:")) {
    returnToHomeAt = 0;
    line1 = "!! ERROR";
    line2 = msg.substring(6);
    line3 = "";
    drawStatus();
  }
}

void setup() {
  setCpuFrequencyMhz(160);  // 240 -> 160: ~halves CPU power, Wi-Fi/I2C/OTA all fine at 160
  Serial.begin(115200);
  esp_task_wdt_add(NULL);
  Wire.begin(21, 22);
  Wire.setClock(400000);   // SSD1306 supports I2C fast-mode; the default 100kHz was too slow to
                           // push a full 128x64 frame at the visualizer's ~15fps without stutter
  pinMode(BTN_STOP_PIN, INPUT_PULLUP);
  pinMode(BTN_HOME_PIN, INPUT_PULLUP);
  pinMode(BTN_PLAYPAUSE_PIN, INPUT_PULLUP);
  pinMode(ENC_SW_PIN, INPUT_PULLUP);
  pinMode(ENC_CLK_PIN, INPUT_PULLUP);
  pinMode(ENC_DT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT_PIN), encoderISR, CHANGE);

  displayOk = display.begin(SSD1306_SWITCHCAPVCC, I2C_ADDRESS);

  if (displayOk) {
    Serial.println("Display OK");
  } else {
    Serial.println("Display FAILED");
    delay(3000);
  }

  drawStandby();
  lastMsgTime = millis();
  lastInputTime = millis();
  Serial.println("CDPLAYER_READY");
}

void handleSelectPress(bool longPress) {
  if (wakeDisplay()) { lastInputTime = millis(); return; }  // wake-only press, swallow the action
  lastInputTime = millis();
  if (uiState == UI_STATUS && line1 == "Stopping...") {
    // Manual escape if stuck on the stopping screen with no host response
    drawStandby();

  } else if (uiState == UI_DISCONNECTED) {
    drawStandby();

  } else if (uiState == UI_PLAY) {
    vuSuppressUntil = millis() + VU_RESUME_DELAY_MS;   // any PLAY input -> back to text for a beat
    playSeekMode = !playSeekMode;   // encoder click swaps what rotating does
    drawPlay();
  }
}

// Encoder rotation. STANDBY just flips displayRotation (upside-down mounting).
// PLAY defaults to adjusting volume; a short encoder click (see
// handleSelectPress) swaps it to track-skip instead.
void handleEncoderCW() {
  if (wakeDisplay()) { lastInputTime = millis(); return; }  // wake-only turn, swallow the action
  lastInputTime = millis();
  if (uiState == UI_STANDBY) {
    displayRotation = 2;
    drawStandby();
  } else if (uiState == UI_PLAY) {
    vuSuppressUntil = millis() + VU_RESUME_DELAY_MS;   // any PLAY input -> back to text for a beat
    if (!playSeekMode) {
      playVolume = min(100, playVolume + 5);
      Serial.print("POT:");
      Serial.println(playVolume);
      drawPlay();
    } else {
      Serial.println("NEXT");   // ponytail: track-skip not wired up host-side yet; host ignores
                                 // unrecognized lines gracefully, add mpv chapter-seek when needed
    }
  }
}

void handleEncoderCCW() {
  if (wakeDisplay()) { lastInputTime = millis(); return; }  // wake-only turn, swallow the action
  lastInputTime = millis();
  if (uiState == UI_STANDBY) {
    displayRotation = 0;
    drawStandby();
  } else if (uiState == UI_PLAY) {
    vuSuppressUntil = millis() + VU_RESUME_DELAY_MS;   // any PLAY input -> back to text for a beat
    if (!playSeekMode) {
      playVolume = max(0, playVolume - 5);
      Serial.print("POT:");
      Serial.println(playVolume);
      drawPlay();
    } else {
      Serial.println("PREV");   // ponytail: see NEXT above
    }
  }
}

// Shared by every STOP trigger (dedicated button, HOME from PLAY, PLAY/PAUSE
// long-press). Rate-limited by STOP_COOLDOWN_MS - see its comment above -
// found necessary after a faulty button/wire flooded this line tens of
// thousands of times/sec during bring-up; a real press can never re-trigger
// this fast, so silently dropping repeats inside the cooldown is safe.
void sendStop() {
  if ((long)(millis() - lastStopSentAt) < STOP_COOLDOWN_MS) return;
  lastStopSentAt = millis();
  Serial.println("STOP");
  line1 = "Stopping...";
  line2 = "";
  line3 = "";
  returnToHomeAt = millis() + STOP_MSG_MS;
  drawStatus();
}

// STOP: physically the old EJECT button. Never actually ejects - the drive is
// locked against ejection by the host, and the disc comes out by hand.
void handleStopButton() {
  if (wakeDisplay()) { lastInputTime = millis(); return; }
  lastInputTime = millis();
  if (uiState == UI_PLAY || uiState == UI_STANDBY || uiState == UI_DISCONNECTED) {
    sendStop();
  }
}

// HOME/BACK: from PLAY, also STOPs (there's no menu to back out of otherwise).
void handleHomeButton() {
  if (wakeDisplay()) { lastInputTime = millis(); return; }
  lastInputTime = millis();
  if (uiState == UI_PLAY) {
    sendStop();
  } else if (uiState != UI_STANDBY) {
    drawStandby();
  }
}

// PLAY/PAUSE: during PLAY, short press toggles pause and long press STOPs
// (same as the dedicated STOP button). From STANDBY, a short press also
// sends PLAY_BUTTON - the host takes that as "probe the drive and play"
// when nothing's currently playing, so this is how you resume after STOP
// without having to physically remove and reinsert the disc.
void handlePlayPauseButton(bool longPress) {
  if (wakeDisplay()) { lastInputTime = millis(); return; }
  lastInputTime = millis();
  if (uiState == UI_PLAY) {
    vuSuppressUntil = millis() + VU_RESUME_DELAY_MS;   // any PLAY input -> back to text for a beat
    if (longPress) {
      sendStop();
    } else {
      Serial.println("PLAY_BUTTON");
    }
  } else if (uiState == UI_STANDBY && !longPress) {
    Serial.println("PLAY_BUTTON");
  }
}

// Drains every line currently buffered on the serial link in one go instead
// of one per loop() call. VU: frames arrive fast enough (~15/sec) that
// drawing each one in turn made the display fall further and further behind
// real-time once a single draw took longer than the send interval - only the
// last VU: seen in a batch is kept/rendered, so the visualizer always shows
// "now" instead of working through a backlog. Every other message type still
// gets parsed in order; only VU: is collapsible like this.
void drainAndDispatch() {
  String pendingVu = "";
  while (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    if (msg.length() == 0) continue;
    if (msg.startsWith("VU:")) pendingVu = msg;
    else parseMessage(msg);
  }
  if (pendingVu.length() > 0) parseMessage(pendingVu);
}

void loop() {
  if (!wifiInitDone && millis() > 3000) {
    wifiInitDone = true;
    initWiFi();
  }

  if (credsJustSaved) {          // portal saved new creds -> reboot into clean STA
    drawWifiReset();
    delay(600);
    ESP.restart();
  }

  if (portalActive) {
    wm.process();
    if (WiFi.status() == WL_CONNECTED) onWifiUp();
  }

  // Live link dropped: WiFi.setAutoReconnect handles most; if still down after
  // WIFI_RETRY_MS, force a fresh join.
  if (wifiConnected && WiFi.status() != WL_CONNECTED) {
    if (wifiDropAt == 0) wifiDropAt = millis();
    else if (millis() - wifiDropAt > WIFI_RETRY_MS) {
      wifiDropAt = millis();
      WiFi.disconnect();
      WiFi.begin();
    }
  } else if (wifiConnected) {
    wifiDropAt = 0;
  }

  if (wifiConnected && WiFi.status() == WL_CONNECTED && otaEnabled) {
    ArduinoOTA.handle();
  }

  esp_task_wdt_reset();
  if (Serial.available()) {
    drainAndDispatch();
  }

  if (returnToHomeAt != 0 && (long)(millis() - returnToHomeAt) >= 0) {
    drawStandby();
  }

  if (playStatusTemp && (long)(millis() - playStatusAt) >= 2500) {
    playStatusTemp = false;
    line1 = "PLAYING";
    drawPlay();
  }

  // --- Encoder rotation: drain ticks accumulated by encoderISR ---
  {
    noInterrupts();
    int16_t ticks = encTicks;
    encTicks = 0;
    interrupts();
    while (ticks > 0) { handleEncoderCW(); ticks--; }
    while (ticks < 0) { handleEncoderCCW(); ticks++; }
  }

  // --- Encoder click (SW) - same debounce/long-press shape as a button.
  // Also guarded against rotation: spinning the knob can momentarily bounce
  // SW low too, which was being misread as a real click. ---
  {
    bool sw = digitalRead(ENC_SW_PIN) == LOW;
    if (sw && !encClickDown && millis() - encClickLastDebounce > DEBOUNCE_MS &&
        millis() - encLastActivityMs > ENC_CLICK_GUARD_MS) {
      encClickDown = true;
      encClickDownAt = millis();
    }
    if (!sw && encClickDown) {
      encClickDown = false;
      encClickLastDebounce = millis();
      handleSelectPress(millis() - encClickDownAt >= LONG_PRESS_MS);
    }
  }

  // --- STOP button (single press, no long-press timer) ---
  {
    bool st = digitalRead(BTN_STOP_PIN) == LOW;
    if (st && !stopBtnDown && millis() - stopBtnLastDebounce > DEBOUNCE_MS) {
      stopBtnDown = true;
    }
    if (!st && stopBtnDown) {
      stopBtnDown = false;
      stopBtnLastDebounce = millis();
      handleStopButton();
    }
  }

  // --- HOME/BACK button (single press; 10s hold = Wi-Fi reset) ---
  {
    bool hm = digitalRead(BTN_HOME_PIN) == LOW;
    if (hm && !homeDown && millis() - homeLastDebounce > DEBOUNCE_MS) {
      homeDown = true;
      homeDownAt = millis();
      wifiResetArmed = false;
    }
    // 10s hold on HOME/SETUP/STANDBY = wipe Wi-Fi creds + reboot to portal. Fires
    // while still held so the eventual release doesn't also trigger the normal action.
    if (hm && homeDown && !wifiResetArmed &&
        (uiState == UI_SETUP || uiState == UI_STANDBY) &&
        millis() - homeDownAt >= WIFI_RESET_HOLD_MS) {
      wifiResetArmed = true;
      Serial.println("WiFi: creds wiped, rebooting");
      clearCreds();
      drawWifiReset();
      delay(800);
      ESP.restart();
    }
    if (!hm && homeDown) {
      homeDown = false;
      homeLastDebounce = millis();
      if (!wifiResetArmed) handleHomeButton();
      wifiResetArmed = false;
    }
  }

  // --- PLAY/PAUSE button (short = pause/resume, long = stop) ---
  {
    bool pp = digitalRead(BTN_PLAYPAUSE_PIN) == LOW;
    if (pp && !playpauseDown && millis() - playpauseLastDebounce > DEBOUNCE_MS) {
      playpauseDown = true;
      playpauseDownAt = millis();
    }
    if (!pp && playpauseDown) {
      playpauseDown = false;
      playpauseLastDebounce = millis();
      handlePlayPauseButton(millis() - playpauseDownAt >= LONG_PRESS_MS);
    }
  }

  // --- Visualizer timeout: host stopped sending VU: (paused/stopped) ---
  if (visualizerActive && (long)(millis() - lastVuAt) >= VU_TIMEOUT_MS) {
    visualizerActive = false;
    if (uiState == UI_PLAY && !displayBlank) drawPlay();
  }

  // --- Idle disc screensaver (STANDBY, or PLAY while paused) ---
  // PLAY is excluded while actively playing - blanking away from the one
  // screen with actually-useful live content (track/volume text or the VU
  // bars) while a disc is playing was an active annoyance with no upside
  // (this ESP32 is USB-powered from the PC it talks to over serial, not the
  // battery-pack mod the screensaver's power-saving purpose was originally
  // for - see the removal note this replaced). But "nothing is playing" in
  // PLAY only ever means paused (a real stop exits to UI_STANDBY already),
  // so it's fine - wanted, even - for the screensaver to take over after the
  // same idle timeout once paused.
  if (displayOk && !displayBlank &&
      (uiState == UI_STANDBY || (uiState == UI_PLAY && isPaused)) &&
      (long)(millis() - lastInputTime) >= IDLE_BLANK_MS) {
    displayBlank = true;
    saverStep = 0;
    lastSaverFrame = 0;
  }
  if (displayBlank && displayOk &&
      (long)(millis() - lastSaverFrame) >= SAVER_FRAME_MS) {
    lastSaverFrame = millis();
    drawDiscSaver(saverStep);
    saverStep = (saverStep + 1) % 24;
  }

  // --- PING timeout (disconnected) ---
  if (uiState != UI_DISCONNECTED &&
      (long)(millis() - lastMsgTime) >= PING_TIMEOUT_MS) {
    drawDisconnected();
  }

  delay(20);
}
