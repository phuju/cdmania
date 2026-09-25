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

const sh = (cmd, args) => spawnSync(cmd, args, { stdio: "inherit" }).status === 0;

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
  deps: () => sh("python3", ["-m", "pip", "install", "--user", "-r", path.join(root, "requirements.txt")]),
  version: () => console.log(require("../package.json").version),
};

const cmd = process.argv[2];
if (commands[cmd]) commands[cmd]();
else {
  console.log(`cdmania ${require("../package.json").version}
usage: cdmania <command>
  install-service    write + enable the systemd user service (cd_player.py from this package)
  uninstall-service  stop, disable and remove it
  deps               pip install pyserial numpy (needs pip)
  path               print where the player and firmware/ live
  version
system packages needed: mpv sg3-utils pulseaudio-utils (parec/pactl) python3-pip
firmware: open firmware/CDPlayer/CDPlayer.ino in Arduino IDE / arduino-cli (esp32:esp32:esp32)`);
  process.exit(cmd ? 1 : 0);
}
