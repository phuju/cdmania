// After `npm i -g cdmania@latest`: restart the service so the new host runs and (if the shipped
// firmware is newer than the front panel's) pushes it over the serial cable. Never fails the install.
// Under `sudo npm i -g` this runs as root, so act on the invoking user's systemd instance instead.
const fs = require("fs");
const os = require("os");
const path = require("path");
const { spawnSync } = require("child_process");

const out = (cmd, args) => (spawnSync(cmd, args, { encoding: "utf8" }).stdout || "").trim();
const sudoUser = process.getuid && process.getuid() === 0 ? process.env.SUDO_USER : null;
const home = sudoUser ? out("getent", ["passwd", sudoUser]).split(":")[5] : os.homedir();
const unit = home && path.join(home, ".config", "systemd", "user", "cd-player.service");

if (process.platform === "linux" && unit && fs.existsSync(unit)) {
  const restart = ["systemctl", "--user", "restart", "cd-player.service"];
  const uid = sudoUser && out("id", ["-u", sudoUser]);
  const [cmd, args] = sudoUser
    ? ["sudo", ["-u", sudoUser, `XDG_RUNTIME_DIR=/run/user/${uid}`, ...restart]]
    : [restart[0], restart.slice(1)];
  if (spawnSync(cmd, args, { stdio: "ignore" }).status === 0) console.log("cdmania: restarted cd-player.service");
}
