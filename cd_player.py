#!/usr/bin/env python3
"""Standalone CD player companion.

Locks the optical drive against ejection at startup, auto-plays whenever a
disc is inserted, and maps the ESP32 front panel's STOP button to a plain
SCSI stop (never a real eject) - the disc comes out by hand, never through
the slot's own eject mechanism.

Talks to firmware/CDPlayer/CDPlayer.ino over USB serial. Needs `mpv` and
sg3-utils' `sg_raw`; numpy + PulseAudio's `parec`/`pactl` drive the OLED
spectrum visualizer if present (skipped silently otherwise).
"""

import base64
import fcntl
import hashlib
import json
import os
import shutil
import socket
import subprocess
import threading
import time
from pathlib import Path

import serial
from serial.tools import list_ports

try:
    import numpy as np
except ImportError:
    np = None

DISC_POLL_SECONDS = 1.5
PING_SECONDS = 10
TRACK_CHECK_SECONDS = 2
FIRMWARE_DIR = Path(__file__).resolve().parent / "firmware" / "release"  # built by scripts/build-firmware.sh
OTA_CHUNK = 512  # raw bytes per OTA_DATA line; must match the firmware's decode buffer
VERSION_ASKS = 5  # opening the port resets the ESP32, so keep asking VERSION until it answers
MPV_SOCKET = "/tmp/cd_player_mpv.sock"

_CDROM_DRIVE_STATUS = 0x5326  # CDS_NO_DISC=1 TRAY_OPEN=2 DRIVE_NOT_READY=3 DISC_OK=4

VU_BARS = 64  # must match the firmware's VU_BARS
VU_WINDOW = 2048  # FFT window (10.8Hz bins); hop stays VU_SAMPLE_RATE // VU_RATE_HZ
VU_RATE_HZ = 30
VU_SAMPLE_RATE = 22050
VU_DB_FLOOR = -40      # dB below the adaptive reference that maps to a flat bar
VU_REF_DECAY = 0.9975  # per-frame relaxation of the reference level
VU_PEAK_DECAY = 2      # bar units/frame a peak falls when nothing louder follows
LR_DB_FLOOR = -50      # stereo LED meters use an absolute dBFS scale (they should track loudness):
LR_DB_CEIL = -6        # peaks at/below the floor light nothing, at/above the ceiling light all 8

_ser_lock = threading.Lock()


def find_drive_device():
    override = os.environ.get("DISC_DEVICE")
    if override:
        return override
    for name in ("/dev/dvd", "/dev/cdrom"):
        if Path(name).exists():
            return str(Path(name).resolve())
    drives = sorted(Path("/dev").glob("sr*"))
    if drives:
        return str(drives[0])
    raise FileNotFoundError("No optical drive found; set DISC_DEVICE explicitly")


def find_serial_port():
    override = os.environ.get("DISC_PORT")
    if override:
        return override
    for port in list_ports.comports():
        desc = (port.description or "").lower()
        if "cp210" in desc or "esp32" in desc:
            return port.device
    raise FileNotFoundError("No ESP32 front panel found; set DISC_PORT explicitly")


def sg_raw(device, *cdb_bytes, timeout=5):
    """A hung SCSI command (e.g. sent while mpv is mid-read) is logged, never fatal."""
    try:
        subprocess.run(["sg_raw", device, *cdb_bytes], timeout=timeout, capture_output=True)
    except (subprocess.TimeoutExpired, OSError) as e:
        print(f"sg_raw {' '.join(cdb_bytes)} failed: {e}")


def lock_drive(device):
    """PREVENT/ALLOW MEDIUM REMOVAL, Prevent=1: blocks software eject and the drive's own button."""
    sg_raw(device, "1e", "00", "00", "00", "01", "00")


def stop_drive(device):
    """START STOP UNIT, LoEj=0/Start=0: spins down only, never ejects."""
    sg_raw(device, "1b", "00", "00", "00", "00", "00")


def drive_status(device):
    """'disc' | 'no_disc' | 'open' | 'loading' | 'unknown'. Never raises."""
    try:
        fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK)
    except OSError:
        return "unknown"
    try:
        st = fcntl.ioctl(fd, _CDROM_DRIVE_STATUS, 0)
    except OSError:
        return "unknown"
    finally:
        os.close(fd)
    return {4: "disc", 1: "no_disc", 2: "open", 3: "loading"}.get(st, "unknown")


def send_line(ser, line):
    """Thread-safe: the main loop and the VU thread share one port."""
    with _ser_lock:
        try:
            ser.write((line + "\n").encode())
        except OSError:
            pass


def standby(ser):
    send_line(ser, "STANDBY:Insert disc")


ota_active = threading.Event()  # set while a firmware image is streaming: VU/LR frames must stay quiet


def shipped_firmware():
    """(image path, {version,size,md5}) of the firmware this package ships, or (None, None)."""
    try:
        meta = json.loads((FIRMWARE_DIR / "firmware.json").read_text())
        image = FIRMWARE_DIR / "cdplayer.bin"
        return (image, meta) if image.is_file() else (None, None)
    except (OSError, ValueError):
        return None, None


def fw_newer(shipped, running):
    try:
        return tuple(map(int, shipped.split("."))) > tuple(map(int, running.split(".")))
    except ValueError:  # "dev" builds never get overwritten
        return False


def _ota_reply(ser, timeout=5):
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = ser.readline().decode(errors="ignore").strip()
        if line.startswith("OTA:"):
            return line[4:]
    return None


def ota_update(ser, image, meta):
    """Stream the image to the ESP32 (stop-and-wait, one ACK per chunk). The firmware
    verifies the MD5 before switching slots, so a failed transfer leaves the old firmware."""
    data = image.read_bytes()
    if len(data) != meta["size"] or hashlib.md5(data).hexdigest() != meta["md5"]:
        print("OTA: shipped image does not match firmware.json, not sending")
        return False
    ota_active.set()
    try:
        time.sleep(0.3)  # let an in-flight VU/LR write finish
        ser.reset_input_buffer()
        send_line(ser, f"OTA_BEGIN:{len(data)}|{meta['md5']}")
        if _ota_reply(ser) != "READY":
            print("OTA: remote refused the update")
            return False
        for i in range(0, len(data), OTA_CHUNK):
            send_line(ser, "OTA_DATA:" + base64.b64encode(data[i:i + OTA_CHUNK]).decode())
            if _ota_reply(ser) != "ACK":
                print(f"OTA: transfer failed at byte {i}")
                return False
            if (i // OTA_CHUNK) % 100 == 0:
                print(f"OTA: {i * 100 // len(data)}%")
        send_line(ser, "OTA_END")
        ok = _ota_reply(ser, timeout=15) == "DONE"
        print("OTA: done, remote restarting" if ok else "OTA: verify failed on the remote")
        return ok
    finally:
        ota_active.clear()


def _mpv_ipc(payload, timeout=0.5):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout)
        sock.connect(MPV_SOCKET)
        sock.sendall(payload)
        return sock.recv(4096)


def _mpv_command(command):
    """True if mpv accepted it (False while its IPC socket isn't up yet)."""
    try:
        _mpv_ipc(json.dumps({"command": command}).encode() + b"\n")
        return True
    except OSError:
        return False


def _mpv_query(command):
    try:
        payload = json.dumps({"command": command, "request_id": 1}).encode() + b"\n"
        return json.loads(_mpv_ipc(payload).decode(errors="ignore")).get("data")
    except (OSError, ValueError):
        return None


def _pulse_default_monitor():
    try:
        sink = subprocess.run(["pactl", "get-default-sink"], capture_output=True,
                              text=True, timeout=2).stdout.strip()
    except Exception:
        return None
    return f"{sink}.monitor" if sink else None


def vu_loop(ser, stop_event, pause_event):
    """Streams `VU:<64 levels>` (FFT of the mono mix, log-spaced bands) and
    `LR:<left>,<right>` (per-channel peak) to the ESP32, both 0-63. The bars use
    adaptive-dB scaling (a fixed gain can't show quiet motion without clipping);
    the L/R meters use fixed dBFS so they follow the actual volume.
    While paused the capture pipe is still drained but nothing is sent: parec keeps tapping the silent monitor, and an all-zero
    stream would keep forcing the bars screen over the screensaver."""
    monitor = _pulse_default_monitor() if np is not None else None
    if not monitor:
        return
    chunk_samples = VU_SAMPLE_RATE // VU_RATE_HZ   # hop between frames
    chunk_bytes = chunk_samples * 4  # s16le stereo
    # --latency-msec=50: PulseAudio's default capture buffer delivers in ~2s
    # bursts, which starves the firmware's VU_TIMEOUT_MS and flickers to text.
    cmd = ["parec", "--format=s16le", f"--rate={VU_SAMPLE_RATE}", "--channels=2",
           "--latency-msec=50", "-d", monitor]
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    except OSError:
        return
    try:
        # 64 log-spaced bands need finer bins than one 67ms hop gives, so the FFT runs
        # over a sliding window of the last VU_WINDOW mono samples.
        window = np.hanning(VU_WINDOW)
        freqs = np.fft.rfftfreq(VU_WINDOW, d=1.0 / VU_SAMPLE_RATE)
        edges = np.searchsorted(freqs, np.geomspace(50, VU_SAMPLE_RATE / 2, VU_BARS + 1))
        mono_buf = np.zeros(VU_WINDOW, dtype=np.float32)
        ref_level = 1e-6
        lr_shown = [0.0, 0.0]
        while not stop_event.is_set():
            raw = proc.stdout.read(chunk_bytes)
            if len(raw) < chunk_bytes:
                if proc.poll() is not None:
                    break
                continue
            if pause_event.is_set() or ota_active.is_set():
                continue
            stereo = np.frombuffer(raw, dtype=np.int16).reshape(-1, 2).astype(np.float32) / 32768.0
            mono_buf = np.concatenate((mono_buf, stereo.mean(axis=1)))[-VU_WINDOW:]
            spectrum = np.abs(np.fft.rfft(mono_buf * window))
            mags = [float(spectrum[lo:max(hi, lo + 1)].max())
                    for lo, hi in zip(edges[:-1], edges[1:])]
            ref_level = max(max(mags), ref_level * VU_REF_DECAY, 1e-6)
            # Raw targets, one char per band (chr(48+level)); the firmware does the smoothing.
            db = 20 * np.log10(np.array(mags) / ref_level + 1e-9)
            levels = np.clip((db - VU_DB_FLOOR) * 63.0 / -VU_DB_FLOOR, 0, 63).astype(int)
            send_line(ser, "VU:" + "".join(chr(48 + v) for v in levels))
            # Stereo level meters (LED sticks): per-channel peak on a fixed dBFS scale.
            peaks = np.abs(stereo).max(axis=0)
            for i, peak in enumerate(peaks):
                db = 20 * float(np.log10(peak + 1e-9))
                target = max(0.0, min(63.0, (db - LR_DB_FLOOR) * 63.0 / (LR_DB_CEIL - LR_DB_FLOOR)))
                lr_shown[i] = target if target > lr_shown[i] else max(0.0, lr_shown[i] - VU_PEAK_DECAY)
            send_line(ser, f"LR:{int(lr_shown[0])},{int(lr_shown[1])}")
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()


class Player:
    def __init__(self, ser, device):
        self.ser = ser
        self.device = device
        self.proc = None
        self.vu_thread = None
        self.vu_stop = self.vu_pause = None
        self.last_track = None
        self.volume = 50            # matches the firmware's default; updated by POT: lines
        self.volume_dirty = False   # a POT: arrived before mpv's IPC was up

    def playing(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self):
        if self.playing():
            return
        mpv = shutil.which("mpv")
        if not mpv:
            print("mpv not found on PATH")
            return
        try:
            os.unlink(MPV_SOCKET)
        except OSError:
            pass
        print(f"Playing {self.device}")
        self.proc = subprocess.Popen(
            [mpv, "--input-ipc-server=" + MPV_SOCKET, "--force-window=no", "--idle=no",
             f"--volume={self.volume}", "--cdrom-device=" + self.device, "--cdda-cdtext=yes", "cdda://"])
        self.last_track = None
        send_line(self.ser, "PLAY:Audio CD")
        if np is not None:
            self.vu_stop, self.vu_pause = threading.Event(), threading.Event()
            self.vu_thread = threading.Thread(
                target=vu_loop, args=(self.ser, self.vu_stop, self.vu_pause), daemon=True)
            self.vu_thread.start()

    def stop(self):
        if self.vu_stop:
            self.vu_stop.set()
            self.vu_thread.join(timeout=2)
        self.vu_thread = self.vu_stop = self.vu_pause = None
        if self.playing():
            self.proc.terminate()
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc = None

    def check_ended(self):
        """True once, when mpv exits on its own (end of disc)."""
        if self.proc is not None and not self.playing():
            self.stop()
            return True
        return False

    def toggle_pause(self):
        if not self.playing():
            return
        _mpv_command(["cycle", "pause"])
        paused = _mpv_query(["get_property", "pause"])
        if isinstance(paused, bool):
            send_line(self.ser, "PLAY_STATUS:PAUSED" if paused else "PLAY_STATUS:PLAYING")
            if self.vu_pause:
                self.vu_pause.set() if paused else self.vu_pause.clear()

    def set_volume(self, vol):
        """The panel answers PLAY: with POT: right after mpv is launched, before its
        IPC socket exists - so remember the value and retry (apply_volume) instead
        of losing it, which left mpv at its default 100 while the OLED said 50."""
        self.volume = vol
        self.volume_dirty = True
        self.apply_volume()

    def apply_volume(self):
        if self.volume_dirty and self.playing() and _mpv_command(["set_property", "volume", self.volume]):
            self.volume_dirty = False

    def skip_track(self, delta):
        if self.playing():
            _mpv_command(["add", "chapter", delta])  # libcdio exposes each CD track as a chapter
            self.check_track()

    def check_track(self):
        """Send PLAY_STATUS:TRACK nn [// CD-Text title] when the chapter changed."""
        if not self.playing():
            return
        chapter = _mpv_query(["get_property", "chapter"])
        if not isinstance(chapter, int) or chapter == self.last_track:
            return
        self.last_track = chapter
        status = f"TRACK {chapter + 1:02d}"
        chapters = _mpv_query(["get_property", "chapter-list"])
        if isinstance(chapters, list) and 0 <= chapter < len(chapters):
            title = (chapters[chapter].get("title") or "").strip()
            if title:
                status += f" // {title}"
        send_line(self.ser, f"PLAY_STATUS:{status}")


def main():
    device = find_drive_device()
    port = find_serial_port()
    print(f"Drive: {device}\nFront panel: {port}")

    lock_drive(device)
    print("Drive locked - physical eject button is now a no-op")

    # exclusive=True: fail loudly if another process holds the port instead of
    # silently racing it for lines (the original DiscStation-service bug).
    ser = serial.Serial(port, 115200, timeout=0.2, exclusive=True)
    player = Player(ser, device)
    standby(ser)

    last_status = None
    last_poll = last_ping = last_track_check = last_ver_ask = 0.0
    ver_asks = 0
    fw_version = None
    fw_pending = False

    try:
        while True:
            now = time.time()
            if now - last_poll >= DISC_POLL_SECONDS:
                last_poll = now
                status = drive_status(device)
                if status != last_status:
                    print(f"Drive status: {status}")
                    if status == "disc":
                        player.start()
                    elif status in ("no_disc", "open") and player.playing():
                        player.stop()
                        standby(ser)
                    last_status = status

            if player.check_ended():
                print("Playback finished")
                standby(ser)

            if now - last_ping >= PING_SECONDS:
                last_ping = now
                send_line(ser, "PING")

            if now - last_track_check >= TRACK_CHECK_SECONDS:
                last_track_check = now
                player.apply_volume()
                player.check_track()

            if fw_version is None and ver_asks < VERSION_ASKS and now - last_ver_ask >= 3:
                last_ver_ask = now
                ver_asks += 1
                send_line(ser, "VERSION")
                if ver_asks == VERSION_ASKS and shipped_firmware()[0]:
                    print("Front panel never answered VERSION: firmware predates OTA, flash it once by cable (cdmania flash)")

            if fw_pending and not player.playing():
                fw_pending = False
                image, meta = shipped_firmware()
                if image and ota_update(ser, image, meta):
                    time.sleep(4)  # the ESP32 reboots; USB serial stays up
                    ser.reset_input_buffer()
                    fw_version = None
                    ver_asks = 0
                    standby(ser)

            line = ser.readline().decode(errors="ignore").strip()
            if not line or line == "PONG" or line.startswith("RCV:"):
                continue
            print(f"< {line}")
            if line.startswith("VERSION:"):
                fw_version = line[8:]
                image, meta = shipped_firmware()
                fw_pending = bool(image and fw_newer(meta["version"], fw_version))
                continue
            if line == "STOP":
                # Release the device before the SCSI stop: sent while mpv is
                # mid-read it hung sg_raw for 5s on this USB-SATA bridge.
                # (The firmware shows its own Stopping... -> STANDBY.)
                player.stop()
                stop_drive(device)
            elif line == "PLAY_BUTTON":
                if player.playing():
                    player.toggle_pause()
                elif drive_status(device) == "disc":
                    player.start()  # resume after STOP: the disc never left the drive
            elif line.startswith("POT:"):
                try:
                    player.set_volume(int(line[4:]))
                except ValueError:
                    pass
            elif line == "NEXT":
                player.skip_track(1)
            elif line == "PREV":
                player.skip_track(-1)
    except KeyboardInterrupt:
        pass
    finally:
        player.stop()
        ser.close()


if __name__ == "__main__":
    main()
