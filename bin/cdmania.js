#!/usr/bin/env node
// Installer wrapper: the player itself is Python (cd_player.py); this only places it and manages the systemd user service.
const fs = require("fs");
const os = require("os");
const path = require("path");
const { spawnSync } = require("child_process");

const root = path.resolve(__dirname, "..");
const unitDir = path.join(os.homedir(), ".config", "systemd", "user");
const unitPath = path.join(unitDir, "cd-player.service");

const unit = `[Unit]
Description=CD Player companion (drive lock, auto-play, ESP32 front panel)
After=network.target sound.target

[Service]
WorkingDirectory=${root}
ExecStart=/usr/bin/python3 -u ${path.join(root, "cd_player.py")}
Restart=on-failure
RestartSec=3

[Install]
WantedBy=default.target
`;

const sh = (cmd, args, env) => spawnSync(cmd, args, { stdio: "inherit", env: env || process.env }).status === 0;
const have = (cmd) => spawnSync("which", [cmd]).status === 0;
const user = os.userInfo().username;
const localBin = path.join(os.homedir(), ".local", "bin");
const withLocalBin = { ...process.env, PATH: `${localBin}:${process.env.PATH}` };
const APT = ["mpv", "sg3-utils", "pulseaudio-utils", "pipewire", "pipewire-pulse", "wireplumber",
  "python3", "python3-serial", "python3-numpy", "curl"];
const DAC_LINE = "dtoverlay=iqaudio-dacplus";
const FQBN = "esp32:esp32:esp32";

// First cable flash: write the prebuilt merged image with esptool (no compiler needed). Later updates go over OTA.
function flashRelease(port, image) {
  if (!have("esptool") && !have("esptool.py")) sh("sudo", ["apt-get", "install", "-y", "esptool"]);
  const tool = have("esptool") ? "esptool" : "esptool.py";
  const svc = spawnSync("systemctl", ["--user", "is-active", "--quiet", "cd-player.service"]).status === 0;
  if (svc) sh("systemctl", ["--user", "stop", "cd-player.service"]);  // it holds the serial port
  const ok = sh(tool, ["--chip", "esp32", "--port", port, "--baud", "460800", "write_flash", "0x0", image]);
  if (svc) sh("systemctl", ["--user", "start", "cd-player.service"]);
  if (!ok) console.error("flash failed; check the port (cdmania flash /dev/ttyUSB1) and the USB cable");
}

// Dev checkout with no release image: compile the sketch with arduino-cli.
function flashFromSource(port) {
  if (!have("arduino-cli") && !fs.existsSync(path.join(localBin, "arduino-cli"))) {
    console.log("installing arduino-cli to ~/.local/bin (official installer)");
    if (!sh("sh", ["-c", `curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR="${localBin}" sh`])) return;
  }
  const cli = (...a) => sh("arduino-cli", a, withLocalBin);
  const sketch = path.join(root, "firmware", "CDPlayer");
  cli("core", "update-index");
  cli("core", "install", "esp32:esp32");
  cli("lib", "install", "Adafruit ST7735 and ST7789 Library", "Adafruit GFX Library");
  if (cli("compile", "--fqbn", FQBN, sketch)) cli("upload", "-p", port, "--fqbn", FQBN, sketch);
}

const commands = {
  path: () => console.log(root),
  "install-service": () => {
    fs.mkdirSync(unitDir, { recursive: true });
    fs.writeFileSync(unitPath, unit);
    console.log(`wrote ${unitPath}`);
    sh("systemctl", ["--user", "daemon-reload"]);
    sh("systemctl", ["--user", "enable", "--now", "cd-player.service"]);
  },
  "uninstall-service": () => {
    sh("systemctl", ["--user", "disable", "--now", "cd-player.service"]);
    fs.rmSync(unitPath, { force: true });
    sh("systemctl", ["--user", "daemon-reload"]);
  },
  setup: () => {
    if (!have("apt-get")) return console.error("setup needs a Debian-family OS (apt). Install per README instead.");
    if (!sh("sudo", ["apt-get", "install", "-y", ...APT])) return console.error("apt install failed");
    sh("sudo", ["usermod", "-aG", "dialout,cdrom,audio", user]);
    sh("sudo", ["loginctl", "enable-linger", user]);
    commands["install-service"]();
    console.log("\nsetup done. Log out and back in once (new groups), plug in the ESP32 + drive.");
    console.log("Next: `cdmania flash` for the front panel; on a Pi with the DAC+ hat: `cdmania pi-dac`.");
  },
  "pi-dac": () => {
    const cfg = ["/boot/firmware/config.txt", "/boot/config.txt"].find((p) => fs.existsSync(p));
    if (!cfg) return console.error("no Raspberry Pi config.txt found; is this a Pi?");
    if (fs.readFileSync(cfg, "utf8").includes(DAC_LINE)) return console.log("already enabled");
    sh("sudo", ["sh", "-c", `printf '\n${DAC_LINE}\n' >> ${cfg}`]);
    console.log(`added ${DAC_LINE} to ${cfg}; reboot to activate the Raspberry Pi DAC+`);
  },
  flash: () => {
    const port = process.argv[3] || "/dev/ttyUSB0";
    const full = path.join(root, "firmware", "release", "cdplayer-full.bin");
    if (fs.existsSync(full)) return flashRelease(port, full);
    flashFromSource(port);
  },
  deps: () => sh("python3", ["-m", "pip", "install", "--user", "-r", path.join(root, "requirements.txt")]),
  version: () => console.log(require("../package.json").version),
};

const cmd = process.argv[2];
if (commands[cmd]) commands[cmd]();
else {
  console.log(`cdmania ${require("../package.json").version}
usage: cdmania <command>
  setup              install everything (apt packages, groups, systemd service); run once
  flash [port]       first cable flash of the front panel (esptool + prebuilt image); updates later go over serial automatically
  pi-dac             enable the Raspberry Pi DAC+ overlay in config.txt (Pi only)
  install-service    write + enable the systemd user service (cd_player.py from this package)
  uninstall-service  stop, disable and remove it
  deps               pip fallback for pyserial/numpy (setup already installs them via apt)
  path               print where the player and firmware/ live
  version
quick start: cdmania setup && cdmania flash
update: npm i -g cdmania@latest (restarts the service; a newer firmware is pushed to the panel by itself)`);
  process.exit(cmd ? 1 : 0);
}
