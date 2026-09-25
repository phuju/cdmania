# cdmania

Companion for a slot-load CD drive (Panasonic UJ8C1 via JMS567 USB-SATA) with an ESP32 front panel.

- Insert a disc: it auto-plays (mpv). STOP is SCSI stop-only; the drive stays locked, lift the disc out by hand.
- PLAY/PAUSE, encoder volume and track skip, track number and CD-Text title on the OLED.
- 64-bar OLED spectrum visualizer with falling peak caps; stereo WS2812B LED level meters.

## Install
```
npm install -g cdmania
sudo apt install mpv sg3-utils pulseaudio-utils python3-pip
cdmania deps
cdmania install-service
```
`cdmania path` shows where the code and `firmware/` live.

## ESP32 firmware
Board: ESP32 WROOM-32 DevKit. Libraries: Adafruit SSD1306 + GFX, FastLED.
```
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/CDPlayer
arduino-cli upload  -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 firmware/CDPlayer
```
Pins: OLED SDA21/SCL22; STOP 14, HOME 13, PLAY/PAUSE 15; encoder CLK32/DT33/SW4; LED data GPIO18.

## Serial protocol (115200)
Firmware to host: `STOP`, `PLAY_BUTTON`, `NEXT`, `PREV`, `POT:<0-100>`.
Host to firmware: `STANDBY:`, `PLAY:`, `PLAY_STATUS:`, `VU:<64 chars, level = char-'0'>`, `LR:<l>,<r>`, `PING`.
