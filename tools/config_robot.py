#!/usr/bin/env python3
"""Configure a Yahboom MicroROS V2 robot over USB serial from macOS.

Adapted from Yahboom's microros_v2_bringup/cli/config_robot. Values come from
command-line arguments instead of being hardcoded, and the Wi-Fi password is
prompted for (or read from LIDARBOT_WIFI_PASSWD) so it never lands in a file.

Examples:
    python tools/config_robot.py --show
    python tools/config_robot.py --ssid MyWifi --agent-ip 192.168.1.50
"""
from __future__ import annotations

import argparse
import getpass
import glob
import ipaddress
import os
import struct
import sys
import time
from dataclasses import dataclass

import serial

BAUD = 115200
HEAD = 0xFF
DEVICE_ID = 0xF8
RETURN_ID = 0xF7
READ_DATA = 0x50
RX_BUF_MAX = 256

CMD = {
    "WIFI_SSID": 0x01,
    "WIFI_PASSWD": 0x02,
    "AGENT_IP": 0x03,
    "AGENT_PORT": 0x04,
    "DOMAIN_ID": 0x06,
    "SERVO_OFFSET": 0x08,
    "MOTOR_PID": 0x09,
    "ROS_NAMESPACE": 0x0B,
    "IMU_YAW_PID": 0x0C,
    "CAMERA_WIFI_SSID": 0x30,
    "CAMERA_WIFI_PASSWD": 0x31,
    "CAMERA_CONFIG_NOW": 0x33,
    "CAMERA_IP": 0x34,
    "CAMERA_QUERY_IP_NOW": 0x35,
    "CONFIG_DONE": 0x22,
    "ENTER_CONFIG": 0x23,
    "FIRMWARE_VERSION": 0x51,
}

CAMERA_STATUS = {
    1: "SSID command failed",
    2: "password command failed",
    3: "invalid argument",
    4: "UART2 busy or internal error",
}


@dataclass(frozen=True)
class Frame:
    addr: int
    payload: bytes


class Robot:
    """Serial protocol client for the MicroROS V2 control board."""

    def __init__(self, port: str) -> None:
        self.ser = serial.Serial(port, BAUD, timeout=0.05)
        for setter in (self.ser.setDTR, self.ser.setRTS):
            try:
                setter(False)
            except (OSError, serial.SerialException):
                pass

    def close(self) -> None:
        self.ser.close()

    # -- framing ---------------------------------------------------------

    def send(self, order: int, payload: bytes | list[int] = b"") -> None:
        data = bytes(int(b) & 0xFF for b in payload)
        length = len(data) + 5
        checksum = (HEAD + DEVICE_ID + length + order + sum(data)) % 256
        self.ser.write(bytes([HEAD, DEVICE_ID, length, order]) + data + bytes([checksum]))
        self.ser.flush()
        time.sleep(0.03)

    def _parse(self, buf: bytes, want: int) -> Frame | None:
        """Scan buf for a valid response frame with address == want."""
        i = 0
        while i + 4 < len(buf):
            if buf[i] != HEAD or buf[i + 1] != RETURN_ID:
                i += 1
                continue
            length = buf[i + 2]
            if length < 5 or length - 5 > RX_BUF_MAX or i + length > len(buf):
                i += 1
                continue
            addr = buf[i + 3]
            payload = buf[i + 4 : i + length - 1]
            checksum = buf[i + length - 1]
            if (HEAD + RETURN_ID + length + addr + sum(payload)) % 256 == checksum and addr == want:
                return Frame(addr, bytes(payload))
            i += 1
        return None

    def wait_for(self, addr: int, timeout: float) -> Frame | None:
        buf = b""
        deadline = time.time() + timeout
        while time.time() < deadline:
            time.sleep(0.05)
            if self.ser.in_waiting:
                buf += self.ser.read(self.ser.in_waiting)
                frame = self._parse(buf, addr)
                if frame:
                    return frame
        return None

    def request(self, addr: int, timeout: float = 0.75) -> Frame | None:
        self.ser.reset_input_buffer()
        self.send(READ_DATA, [addr, 0])
        return self.wait_for(addr, timeout)

    # -- session ---------------------------------------------------------

    def reboot(self) -> None:
        """Pulse DTR/RTS to reset the ESP32-S3 (without entering download mode)."""
        self.ser.setDTR(False)
        self.ser.setRTS(True)
        time.sleep(0.10)
        self.ser.setDTR(True)
        self.ser.setRTS(True)
        time.sleep(1.80)
        self.ser.reset_input_buffer()

    def version(self) -> str | None:
        frame = self.request(CMD["FIRMWARE_VERSION"], timeout=0.3)
        if not frame or len(frame.payload) < 3:
            return None
        v = "%d.%d.%d" % tuple(frame.payload[:3])
        return None if v == "0.0.0" else v

    def enter_config(self, timeout: float = 12.0) -> str:
        """Reset the board and catch its ~2 s post-boot config window."""
        self.reboot()
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.send(CMD["ENTER_CONFIG"])
            v = self.version()
            if v:
                return v
            time.sleep(0.1)
        raise RuntimeError(
            "Board did not open its config window. Check the USB-C data cable "
            "and that the robot's power switch is on."
        )

    def finish(self) -> None:
        self.send(CMD["CONFIG_DONE"])

    # -- reads -----------------------------------------------------------

    def read_string(self, key: str) -> str:
        frame = self.request(CMD[key])
        if not frame or not frame.payload or frame.payload[0] == 0:
            return ""
        return frame.payload.decode("utf-8", errors="ignore").rstrip("\x00")

    def read_settings(self) -> dict[str, str]:
        out: dict[str, str] = {"wifi_ssid": self.read_string("WIFI_SSID")}
        f = self.request(CMD["AGENT_IP"])
        out["agent_ip"] = ".".join(str(b) for b in f.payload[:4]) if f else "?"
        f = self.request(CMD["AGENT_PORT"])
        out["agent_port"] = str(struct.unpack("<H", f.payload[:2])[0]) if f else "?"
        f = self.request(CMD["DOMAIN_ID"])
        out["domain_id"] = str(struct.unpack("<h", f.payload[:2])[0]) if f else "?"
        out["ros_namespace"] = self.read_string("ROS_NAMESPACE") or "(none)"
        out["camera_ip"] = self.read_string("CAMERA_IP") or "(none)"
        for key, name in (("MOTOR_PID", "motor_pid"), ("IMU_YAW_PID", "imu_yaw_pid")):
            f = self.request(CMD[key])
            if f and len(f.payload) >= 6:
                vals = struct.unpack("<hhh", f.payload[:6])
                out[name] = ", ".join("%.2f" % (v / 100.0) for v in vals)
            else:
                out[name] = "?"
        return out

    # -- writes ----------------------------------------------------------

    def write_string(self, key: str, text: str, allow_empty: bool = False) -> None:
        data = text.encode("utf-8")
        if not data:
            if not allow_empty:
                raise ValueError(f"{key} cannot be empty")
            data = b"\x00"
        for _ in range(3):
            self.send(CMD[key], data)
            time.sleep(0.3)
            if self.read_string(key) == text:
                return
        raise RuntimeError(f"{key} write could not be verified")

    def set_agent(self, ip: ipaddress.IPv4Address, port: int) -> None:
        self.send(CMD["AGENT_IP"], ip.packed)
        self.send(CMD["AGENT_PORT"], struct.pack("<H", port))

    def set_domain_id(self, domain_id: int) -> None:
        self.send(CMD["DOMAIN_ID"], struct.pack("<h", domain_id))

    def apply_camera_wifi(self, timeout: float = 22.0) -> None:
        self.ser.reset_input_buffer()
        self.send(CMD["CAMERA_CONFIG_NOW"], [0])
        frame = self.wait_for(CMD["CAMERA_CONFIG_NOW"], timeout)
        if frame is None:
            raise RuntimeError("camera did not answer (timed out)")
        status = frame.payload[0] if frame.payload else 0
        if status:
            raise RuntimeError(CAMERA_STATUS.get(status, f"status {status}"))

    def query_camera_ip(self, timeout: float = 4.0) -> str | None:
        self.ser.reset_input_buffer()
        self.send(CMD["CAMERA_QUERY_IP_NOW"], [0])
        frame = self.wait_for(CMD["CAMERA_IP"], timeout)
        if not frame or not frame.payload or frame.payload[0] == 0:
            return None
        return frame.payload.decode("utf-8", errors="ignore").rstrip("\x00")


def find_port() -> str:
    candidates = [
        p
        for pattern in ("/dev/cu.usbserial*", "/dev/cu.wchusbserial*", "/dev/cu.usbmodem*", "/dev/cu.SLAB*")
        for p in sorted(glob.glob(pattern))
    ]
    if not candidates:
        raise SystemExit("No USB serial device found. Is the robot on and plugged in with a data cable?")
    if len(candidates) > 1:
        print(f"Several serial ports found, using {candidates[0]}: {candidates}", file=sys.stderr)
    return candidates[0]


def print_settings(settings: dict[str, str]) -> None:
    width = max(len(k) for k in settings)
    for key, value in settings.items():
        print(f"  {key:<{width}}  {value}")


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial device (auto-detected if omitted)")
    ap.add_argument("--show", action="store_true", help="only print current settings")
    ap.add_argument("--ssid", help="2.4 GHz Wi-Fi network name")
    ap.add_argument("--agent-ip", type=ipaddress.IPv4Address, help="IP of the machine running the micro-ROS agent")
    ap.add_argument("--agent-port", type=int, default=8090)
    ap.add_argument("--domain-id", type=int, default=77, choices=range(0, 101), metavar="0-100")
    ap.add_argument("--namespace", default="")
    ap.add_argument("--no-camera", action="store_true", help="skip camera Wi-Fi setup")
    args = ap.parse_args()
    if not args.show and (not args.ssid or not args.agent_ip):
        ap.error("--ssid and --agent-ip are required unless --show is given")
    return args


def main() -> int:
    args = parse_args()
    passwd = ""
    if not args.show:
        passwd = os.environ.get("LIDARBOT_WIFI_PASSWD") or getpass.getpass(f"Wi-Fi password for {args.ssid}: ")

    robot = Robot(args.port or find_port())
    try:
        version = robot.enter_config()
        print(f"Connected. Firmware {version}")

        if not args.show:
            print("Writing settings...")
            robot.write_string("WIFI_SSID", args.ssid)
            robot.write_string("WIFI_PASSWD", passwd, allow_empty=True)
            robot.set_agent(args.agent_ip, args.agent_port)
            robot.set_domain_id(args.domain_id)
            robot.write_string("ROS_NAMESPACE", args.namespace, allow_empty=True)
            if not args.no_camera:
                robot.write_string("CAMERA_WIFI_SSID", args.ssid)
                robot.write_string("CAMERA_WIFI_PASSWD", passwd, allow_empty=True)
                try:
                    robot.apply_camera_wifi()
                    time.sleep(2)
                    ip = next((ip for _ in range(5) if (ip := robot.query_camera_ip())), None)
                    print(f"Camera IP: {ip}" if ip else "Warning: camera did not report an IP yet")
                except RuntimeError as exc:
                    print(f"Warning: camera Wi-Fi setup failed: {exc}", file=sys.stderr)

        print("Current settings:")
        print_settings(robot.read_settings())
        robot.finish()
    finally:
        robot.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
