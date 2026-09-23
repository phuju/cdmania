#!/usr/bin/env python3
"""Standalone CD player companion.

Locks the optical drive against ejection at startup, auto-plays whenever a
disc is inserted, and maps the CDPlayer ESP32 front panel's STOP button to a
plain SCSI stop (never a real eject) - the disc always comes out by hand,
never through the slot's own eject mechanism. See
~/Desktop/cd-player-project-findings.md for why.

Talks to firmware/CDPlayer/CDPlayer.ino (this project's own fork of
DiscStation's front-panel firmware) over USB serial. Requires `mpv` and
sg3-utils' `sg_raw` on PATH; numpy + PulseAudio's `parec`/`pactl` are used for
the OLED spectrum visualizer if available (silently skipped otherwise).
"""

import fcntl
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
MPV_SOCKET = "/tmp/cd_player_mpv.sock"
STOP_LINES = {"STOP"}

_CDROM_DRIVE_STATUS = 0x5326  # CDS_NO_DISC=1 TRAY_OPEN=2 DRIVE_NOT_READY=3 DISC_OK=4

# VU spectrum visualizer - adapted from DiscStation's src/discstation.py _vu_loop.
VU_BARS = 16          # must match the firmware's VU_BARS
VU_RATE_HZ = 15
VU_SAMPLE_RATE = 22050
VU_DB_FLOOR = -40      # dB below the adaptive reference that maps to a flat bar
VU_REF_DECAY = 0.995   # per-frame relaxation of the reference level (~a few sec to settle down)
VU_PEAK_DECAY = 4      # bar units/frame a peak falls by when nothing louder follows

_ser_lock = threading.Lock()


def find_drive_device():
    override = os.environ.get("DISC_DEVICE")
    if override:
        return override
    for name in ("/dev/dvd", "/dev/cdrom"):
        path = Path(name)
        if path.exists():
            return str(path.resolve())
    drives = sorted(Path("/dev").glob("sr*"))
    if drives:
        return str(drives[0])
    raise FileNotFoundError("No optical drive found; set DISC_DEVICE explicitly")


def find_serial_port():
    override = os.environ.get("DISC_PORT")
    if override:
        return override
    for port in list_ports.comports():
        description = (port.description or "").lower()
        if (port.vid, port.pid) == (0x303A, 0x1001) or "esp32" in description or "cp210" in description:
            return port.device
    raise FileNotFoundError("No ESP32 front panel found; set DISC_PORT explicitly")


def sg_raw(device, *cdb_bytes, timeout=5):
    """Never lets a stuck/contended SCSI command take the whole player down -
    a command sent while mpv still has the device open mid-read can block for
    a while on some USB-SATA bridges; a hung drive is a reason to log and
    keep running, not to crash."""
    try:
        subprocess.run(["sg_raw", device, *cdb_bytes], timeout=timeout, capture_output=True)
    except (subprocess.TimeoutExpired, OSError) as e:
        print(f"sg_raw {' '.join(cdb_bytes)} failed: {e}")


def lock_drive(device):
    """SCSI PREVENT/ALLOW MEDIUM REMOVAL, Prevent=1 - blocks both a software
    eject and the drive's own physical eject button."""
    sg_raw(device, "1e", "00", "00", "00", "01", "00")


def stop_drive(device):
    """SCSI START STOP UNIT, LoEj=0/Start=0 - spins down only, never ejects."""
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
    """Thread-safe write - both the main loop and the VU thread write to the
    same serial port, and this keeps their lines from interleaving mid-write."""
    with _ser_lock:
        try:
            ser.write((line + "\n").encode())
        except OSError:
            pass


def _mpv_ipc(payload, timeout=0.5):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout)
        sock.connect(MPV_SOCKET)
        sock.sendall(payload)
        return sock.recv(4096)


def _mpv_command(command):
    try:
        _mpv_ipc(json.dumps({"command": command}).encode() + b"\n")
    except OSError:
        pass


def _mpv_query(command):
    try:
        payload = json.dumps({"command": command, "request_id": 1}).encode() + b"\n"
        response = json.loads(_mpv_ipc(payload).decode(errors="ignore"))
        return response.get("data")
    except (OSError, ValueError, json.JSONDecodeError):
        return None


def _pulse_default_monitor():
    try:
        sink = subprocess.run(["pactl", "get-default-sink"], capture_output=True,
                               text=True, timeout=2).stdout.strip()
    except Exception:
        return None
    return f"{sink}.monitor" if sink else None


def vu_loop(ser, stop_event, pause_event):
    """Streams `VU:<16 levels>` lines to the ESP32 while it runs, adapted from
    DiscStation's src/discstation.py _vu_loop - same FFT/log-band/adaptive-dB
    scaling, since a fixed linear gain can't show quiet-volume motion without
    clipping loud sections. pause_event: skip sending (but keep draining the
    capture pipe) while mpv is paused - parec keeps tapping the now-silent
    monitor source regardless, and an unconditional all-zero VU: stream would
    otherwise keep forcing the visualizer bars back up over the screensaver."""
    if np is None:
        return
    monitor = _pulse_default_monitor()
    if not monitor:
        return
    chunk_samples = max(256, VU_SAMPLE_RATE // VU_RATE_HZ)
    chunk_bytes = chunk_samples * 2  # s16le, mono
    cmd = ["parec", "--format=s16le", f"--rate={VU_SAMPLE_RATE}", "--channels=1",
           # --latency-msec=50: PulseAudio's default capture buffer is several
           # hundred ms to seconds - without this, parec hands us data in
           # ~1.5-2s bursts instead of a steady trickle, which starves the
           # firmware's VU_TIMEOUT_MS fallback and flickers back to text.
           "--latency-msec=50", "-d", monitor]
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    except OSError:
        return
    try:
        window = np.hanning(chunk_samples)
        freqs = np.fft.rfftfreq(chunk_samples, d=1.0 / VU_SAMPLE_RATE)
        edges = np.searchsorted(freqs, np.geomspace(40, VU_SAMPLE_RATE / 2, VU_BARS + 1))
        ref_level = 1e-6
        shown = [0.0] * VU_BARS
        while not stop_event.is_set():
            raw = proc.stdout.read(chunk_bytes)
            if len(raw) < chunk_bytes:
                if proc.poll() is not None:
                    break
                continue
            if pause_event.is_set():
                continue
            samples = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
            spectrum = np.abs(np.fft.rfft(samples * window))
            mags = []
            for i in range(VU_BARS):
                lo, hi = edges[i], max(edges[i + 1], edges[i] + 1)
                band = spectrum[lo:hi]
                mags.append(float(band.max()) if band.size else 0.0)
            ref_level = max(max(mags, default=0.0), ref_level * VU_REF_DECAY, 1e-6)
            levels = []
            for mag in mags:
                db = 20 * float(np.log10(mag / ref_level + 1e-9))
                target = max(0.0, min(63.0, (db - VU_DB_FLOOR) * 63.0 / -VU_DB_FLOOR))
                shown_i = target if target > shown[len(levels)] else max(0.0, shown[len(levels)] - VU_PEAK_DECAY)
                shown[len(levels)] = shown_i
                levels.append(int(shown_i))
            send_line(ser, "VU:" + ",".join(str(v) for v in levels))
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
        self.vu_stop = None
        self.vu_pause = None
        self.last_track = None

    def start(self):
        if self.playing():
            return
        try:
            os.unlink(MPV_SOCKET)
        except OSError:
            pass
        mpv = shutil.which("mpv")
        if not mpv:
            print("mpv not found on PATH")
            return
        cmd = [mpv, "--input-ipc-server=" + MPV_SOCKET, "--force-window=no",
               "--idle=no", "--cdrom-device=" + self.device, "--cdda-cdtext=yes", "cdda://"]
        print(f"Playing {self.device}")
        self.proc = subprocess.Popen(cmd)
        self.last_track = None
        send_line(self.ser, "PLAY:Audio CD")
        if np is not None:
            self.vu_stop = threading.Event()
            self.vu_pause = threading.Event()
            self.vu_thread = threading.Thread(
                target=vu_loop, args=(self.ser, self.vu_stop, self.vu_pause), daemon=True)
            self.vu_thread.start()

    def stop(self):
        self._stop_vu()
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc = None

    def _stop_vu(self):
        if self.vu_stop:
            self.vu_stop.set()
        if self.vu_thread:
            self.vu_thread.join(timeout=2)
        self.vu_thread = None
        self.vu_stop = None
        self.vu_pause = None

    def check_ended(self):
        """True the moment mpv has exited on its own (end of disc) - callers
        use this to notice playback finished without a STOP button press."""
        if self.proc is not None and self.proc.poll() is not None:
            self._stop_vu()
            self.proc = None
            return True
        return False

    def toggle_pause(self):
        if not self.playing():
            return
        _mpv_command(["cycle", "pause"])
        paused = _mpv_query(["get_property", "pause"])
        if paused is True:
            send_line(self.ser, "PLAY_STATUS:PAUSED")
            if self.vu_pause:
                self.vu_pause.set()
        elif paused is False:
            send_line(self.ser, "PLAY_STATUS:PLAYING")
            if self.vu_pause:
                self.vu_pause.clear()

    def set_volume(self, vol):
        if self.playing():
            _mpv_command(["set_property", "volume", vol])

    def skip_track(self, delta):
        if not self.playing():
            return
        # cdda:// exposes each CD track as an mpv chapter via libcdio - no
        # separate TOC/track-start lookup needed, same fallback discstation.py
        # uses when it has no track_starts data.
        _mpv_command(["add", "chapter", delta])
        self.check_track()

    def check_track(self):
        """Send an updated PLAY_STATUS:TRACK line when the current chapter
        (1:1 with CD track) has changed - called after a manual skip and
        periodically so a natural track change also updates the display."""
        if not self.playing():
            return
        chapter = _mpv_query(["get_property", "chapter"])
        if not isinstance(chapter, int) or chapter == self.last_track:
            return
        self.last_track = chapter
        status = f"TRACK {chapter + 1:02d}"
        # CD-Text (if this disc actually has any - most don't) shows up as a
        # per-chapter title here, since --cdda-cdtext=yes is already passed.
        chapters = _mpv_query(["get_property", "chapter-list"])
        if isinstance(chapters, list) and 0 <= chapter < len(chapters):
            title = (chapters[chapter].get("title") or "").strip()
            if title:
                status += f" // {title}"
        send_line(self.ser, f"PLAY_STATUS:{status}")

    def playing(self):
        return self.proc is not None and self.proc.poll() is None


def main():
    device = find_drive_device()
    port = find_serial_port()
    print(f"Drive: {device}")
    print(f"Front panel: {port}")

    lock_drive(device)
    print("Drive locked (PREVENT/ALLOW MEDIUM REMOVAL) - physical eject button is now a no-op")

    # exclusive=True: fail loudly if some other process already holds this
    # port, instead of silently racing it for lines (the original bug).
    ser = serial.Serial(port, 115200, timeout=0.2, exclusive=True)
    player = Player(ser, device)
    send_line(ser, "STANDBY:Insert disc")

    last_status = None
    last_poll = 0.0
    last_ping = 0.0
    last_track_check = 0.0

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
                        send_line(ser, "STANDBY:Insert disc")
                    last_status = status

            if player.check_ended():
                print("Playback finished")
                send_line(ser, "STANDBY:Insert disc")

            if now - last_ping >= PING_SECONDS:
                last_ping = now
                send_line(ser, "PING")

            if now - last_track_check >= TRACK_CHECK_SECONDS:
                last_track_check = now
                player.check_track()

            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            print(f"< {line}")
            if line in STOP_LINES:
                # The firmware already shows its own "Stopping..." -> STANDBY
                # transition locally on a timer - nothing to send back here.
                # Order matters: mpv still has the device open and is mid-read
                # while playing, and sending the SCSI stop before releasing
                # that hung sg_raw for 5s on this USB-SATA bridge and crashed
                # the whole script - kill the player first, then stop the drive.
                player.stop()
                stop_drive(device)
            elif line == "PLAY_BUTTON":
                if player.playing():
                    player.toggle_pause()
                elif drive_status(device) == "disc":
                    # Pressed from STANDBY after a STOP - the disc never left
                    # the drive, so re-probe and start it back up without
                    # needing a physical remove/reinsert.
                    player.start()
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
