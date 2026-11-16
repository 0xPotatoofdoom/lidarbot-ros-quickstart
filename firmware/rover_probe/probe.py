#!/usr/bin/env python3
"""Run rover_probe tests over the board's USB serial port and print the results.

    python probe.py /dev/cu.usbserial-0001 ipb          # IMU, power, buzzer/LEDs
    python probe.py /dev/cu.usbserial-0001 L 5          # left motor, after a 5 s countdown

Opening the port resets the board; the script waits for "ready", optionally waits the
countdown (so you can get in position to watch), then sends each test letter in turn.
Needs pyserial (installed with esptool).
"""
import re
import sys
import time

import serial

BOOT_NOISE = re.compile(r"^([IWE] \(|ESP-ROM|Build:|rst:|SPIWP|mode:|load:|entry)")


def until(port: serial.Serial, pattern: str, limit: float) -> None:
    buf, end = "", time.time() + limit
    while time.time() < end:
        buf += port.read(4096).decode(errors="replace")
        if re.search(pattern, buf):
            break
    text = re.sub(r"\x1b\[[0-9;]*m", "", buf)
    print("\n".join(line for line in text.splitlines() if line.strip() and not BOOT_NOISE.match(line)))


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    port = serial.Serial()
    port.port, port.baudrate, port.timeout = sys.argv[1], 115200, 0.1
    port.dtr = port.rts = False
    port.open()
    port.rts = True                      # pulse EN: reset into the app
    time.sleep(0.1)
    port.rts = False
    until(port, r"ready:", 10)
    time.sleep(float(sys.argv[3]) if len(sys.argv) > 3 else 0)
    for cmd in sys.argv[2]:
        port.write(cmd.encode())
        until(port, rf"done {re.escape(cmd)}", 40)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
