# cdmania

Companion for a slot-load CD drive (Panasonic UJ8C1 via JMS567 USB-SATA) with an ESP32 front panel.

- Insert a disc: it auto-plays (mpv). STOP is SCSI stop-only; the drive stays locked, lift the disc out by hand.
- PLAY/PAUSE, encoder volume and track skip, track number and CD-Text title on the TFT.
- 64-bar neon spectrum visualizer with falling peak caps, plus two segmented stereo level meters, all on-screen.

## Install (Debian / Raspberry Pi OS)
```
npm install -g cdmania
cdmania setup      # apt packages (mpv, sg3-utils, pipewire-pulse, pyserial, numpy), groups, systemd service
cdmania flash      # arduino-cli + ESP32 core + libraries, compiles and uploads the front-panel firmware
cdmania pi-dac     # Raspberry Pi DAC+ hat only: enables the audio overlay (then reboot)
```
Log out and in once after `setup` so the new groups apply. `cdmania path` shows where the code and `firmware/` live.

## ESP32 firmware
Board: ESP32 WROOM-32 DevKit, 1.8" ST7735R TFT (160x128, SPI). `cdmania flash` does all of this; manual equivalent (libraries: Adafruit ST7735 and ST7789 Library + GFX):
```
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/CDPlayer
arduino-cli upload  -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 firmware/CDPlayer
```
Pins: TFT CS5, DC16 (RX2), RST17 (TX2), SPI MOSI23/SCK18; STOP 14, HOME 13, PLAY/PAUSE 15; encoder CLK32/DT33/SW4.

## Serial protocol (115200)
Firmware to host: `STOP`, `PLAY_BUTTON`, `NEXT`, `PREV`, `POT:<0-100>`.
Host to firmware: `STANDBY:`, `PLAY:`, `PLAY_STATUS:`, `VU:<64 chars, level = char-'0'>`, `LR:<l>,<r>`, `PING`.
