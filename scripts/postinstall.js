// After `npm i -g cdmania@latest`: restart the service so the new host runs and (if the shipped
// firmware is newer than the front panel's) pushes it over the serial cable. Never fails the install.
const fs = require("fs");
const os = require("os");
const path = require("path");
const { spawnSync } = require("child_process");

const unit = path.join(os.homedir(), ".config", "systemd", "user", "cd-player.service");
if (process.platform === "linux" && fs.existsSync(unit)) {
  spawnSync("systemctl", ["--user", "restart", "cd-player.service"], { stdio: "ignore" });
  console.log("cdmania: restarted cd-player.service");
}
