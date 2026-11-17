#!/usr/bin/env python3
"""Halt the Orange Pi when the rover's battery guard reports CRITICAL.

Reads the rover board's serial log on /dev/ttyUSB0 (standard library only, no pyserial):
  - the status line every 10 s:  "11.87V -0.42A | ROS OK 5.0W | wifi ..."  (or "| BATT LOW |" ...)
  - state changes:               "battery LOW -> CRITICAL (10.05 V at 0.30 A, 10.15 V resting)"
At CRITICAL the firmware has already stopped the wheels and lidar. If it stays critical for
HALT_AFTER_S (a charger plugged in clears it), this halts the Pi so its SD card isn't cut mid-write
when the pack or the backup battery runs out.

Raw serial lines are never logged or stored: the boot log can hold network settings.
Once micro-ROS moves to this USB link (firmware phase 3), read /power instead.
Run as root (systemd unit: battery-watch.service). --dry-run logs instead of halting.
"""
import argparse
import fcntl
import os
import re
import subprocess
import sys
import termios
import time
from dataclasses import dataclass

PORT = "/dev/ttyUSB0"
BAUD = termios.B115200
HALT_AFTER_S = 60.0     # critical this long (6 status lines) before halting
STALE_S = 30.0          # a reading older than this doesn't count towards the halt
SUMMARY_EVERY_S = 300.0
NO_PACK_V = 6.0         # matches BATT_NO_PACK_V in the firmware

STATUS_RE = re.compile(r"(\d+\.\d+)V(?: (-?\d+\.\d+)A)? \| (BATT CRITICAL|BATT LOW|ROS OK|AGENT|WIFI)")
CHANGE_RE = re.compile(r"battery (?:NO PACK|OK|LOW|CRITICAL) -> (NO PACK|OK|LOW|CRITICAL) \((\d+\.\d+) V at (-?\d+\.\d+) A")


@dataclass(frozen=True)
class Reading:
    state: str          # "NO PACK", "OK", "LOW" or "CRITICAL"
    volts: float
    amps: float | None


def parse(line: str) -> Reading | None:
    m = CHANGE_RE.search(line)
    if m:
        return Reading(m.group(1), float(m.group(2)), float(m.group(3)))
    m = STATUS_RE.search(line)
    if not m:
        return None
    volts = float(m.group(1))
    amps = float(m.group(2)) if m.group(2) else None
    tag = m.group(3)
    if tag == "BATT CRITICAL":
        state = "CRITICAL"
    elif tag == "BATT LOW":
        state = "LOW"
    else:
        state = "NO PACK" if volts < NO_PACK_V else "OK"
    return Reading(state, volts, amps)


@dataclass(frozen=True)
class Watch:
    state: str = "UNKNOWN"
    critical_since: float | None = None
    last_seen: float | None = None


def step(w: Watch, r: Reading, now: float) -> Watch:
    """New watch state after a reading. Critical time only counts while readings keep coming."""
    if r.state != "CRITICAL":
        return Watch(r.state, None, now)
    fresh = w.last_seen is not None and now - w.last_seen <= STALE_S
    since = w.critical_since if (w.critical_since is not None and fresh) else now
    return Watch(r.state, since, now)


def should_halt(w: Watch, now: float) -> bool:
    return (w.critical_since is not None and w.last_seen is not None
            and now - w.last_seen <= STALE_S and now - w.critical_since >= HALT_AFTER_S)


def open_port(path: str) -> int:
    fd = os.open(path, os.O_RDONLY | os.O_NOCTTY)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0                                   # iflag: raw
    attrs[1] = 0                                   # oflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # no HUPCL: closing doesn't toggle DTR
    attrs[3] = 0                                   # lflag: no echo, no canonical mode
    attrs[4] = attrs[5] = BAUD
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 10                   # read returns after 1 s of silence
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    fcntl.fcntl(fd, fcntl.F_SETFL, 0)
    return fd


def log(msg: str) -> None:
    print(msg, flush=True)                         # journald adds the timestamp


def describe(r: Reading) -> str:
    amps = f" {r.amps:.2f} A" if r.amps is not None else ""
    return f"{r.state} {r.volts:.2f} V{amps}"


def run(port: str, dry_run: bool) -> None:
    w = Watch()
    last_summary = 0.0
    while True:
        try:
            fd = open_port(port)
        except OSError as e:
            log(f"cannot open {port}: {e.strerror}; retrying in 10 s")
            time.sleep(10)
            continue
        log(f"watching {port}")
        buf = b""
        try:
            while True:
                chunk = os.read(fd, 512)
                now = time.monotonic()
                buf += chunk
                *lines, buf = buf.split(b"\n")
                buf = buf[-1024:]
                for raw in lines:
                    r = parse(raw.decode("ascii", "replace"))
                    if r is None:
                        continue
                    before = w.state
                    w = step(w, r, now)
                    if w.state != before or now - last_summary >= SUMMARY_EVERY_S:
                        log(f"battery {describe(r)}" + (f" (was {before})" if w.state != before else ""))
                        last_summary = now
                if should_halt(w, now):
                    log(f"battery critical for {HALT_AFTER_S:.0f} s: halting")
                    if dry_run:
                        w = Watch()
                        continue
                    subprocess.run(["systemctl", "halt"], check=False)
                    return
        except OSError as e:
            log(f"{port} lost: {e.strerror}; reopening")
            time.sleep(2)
        finally:
            try:
                os.close(fd)
            except OSError:
                pass


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--dry-run", action="store_true", help="log the halt instead of doing it")
    args = ap.parse_args()
    run(args.port, args.dry_run)
    return 0


if __name__ == "__main__":
    sys.exit(main())
