# lidarbot-cam: open firmware for Yahboom's ESP32-S3 camera module

Replacement firmware for the Wi-Fi camera on the Yahboom MicroROS V2 (ESP32-S3 with 2 MB
PSRAM and 4 MB flash, GC2145 2MP sensor). It serves the same endpoints as Yahboom's
firmware, so Yahboom's ROS `camera_driver` and `./up` work without changes.

| | Yahboom AI_V1.5.2 | lidarbot-cam 0.1.0 |
|---|---|---|
| 320x240 | about 6 fps | 18 fps |
| 640x480 | not available (stream fixed at 320x240) | 5 fps |
| Settings that work | none (resolution ignored, others rejected) | resolution, JPEG quality, mirror/flip |
| Wi-Fi hotspot | open hotspot on by default | station only |
| Wi-Fi power save | on (pings 0.3-1.3 s with hotspot on) | off |

Measured 2026-11-13 on our robot, -27 to -34 dBm. Exposure, gain, white balance and
brightness aren't implemented by the esp32-camera GC2145 driver, so they return HTTP 500.

## Endpoints

- `http://<ip>:81/stream`: MJPEG
- `http://<ip>/capture`: one JPEG
- `http://<ip>/status`: JSON with settings, measured `fps`, `encode_ms`, `grab_ms`, `rssi`,
  free memory, uptime
- `http://<ip>/control?var=<name>&val=<n>`: `framesize` (6 = 320x240, 10 = 640x480, the
  largest that fits), `quality` (0-63, lower is better), `hmirror`, `vflip`, `pixformat`
  (0 RGB565, 1 YUV422), `xclk` (MHz)
- `http://<ip>/log`: the last 4 KB of log output (opening the serial port resets the module,
  so this is the easier way to read it)

## How it works

- Camera pins were read from Yahboom's firmware (the `esp_camera_init` call): PWDN 48,
  RESET 47, XCLK 15, SDA 4, SCL 5, D7-D0 16 17 18 9 11 8 12 10, VSYNC 6, HREF 7, PCLK 13.
  Horizontal mirror on and vertical flip off gives an upright, unmirrored image on our
  module (checked against a known scene); both can be changed through `/control`.
- The GC2145 can't compress, so frames arrive as YUV422 and are compressed with Espressif's
  SIMD encoder (`esp_new_jpeg`), about 45 ms at 320x240 and 180 ms at 640x480. The generic
  `frame2jpg` took twice as long.
- The stream socket has Nagle's algorithm off and lwIP gets 64 KB TCP windows. With the
  defaults (5.7 KB) the network, not the camera, limited 320x240 to under 1 fps.
- Wi-Fi uses the network already saved in the module's nvs partition (by Yahboom's firmware or
  by `sta_ssid:`/`sta_pd:` on the serial port). Nothing about the network is in the source,
  and the Wi-Fi driver's own log, which prints the network name, is set to warnings only.
- Only the app partition (0x10000) is flashed. Yahboom's bootloader, partition table and
  nvs stay as they are.

## Build and flash

Needs Docker (Espressif's `espressif/idf:v5.1.2` image) and a Linux machine with the module
plugged into USB: its USB chip is a CH340 variant (1a86:7522) that macOS has no driver for.
We use the Pi Zero on the robot, with esptool installed in a venv.

```bash
./build_and_flash.sh build                    # build only
./build_and_flash.sh                          # build, copy to the Pi, flash 0x10000 at 115200
```

Faster baud rates gave SLIP errors with this CH340. Flashing takes about 100 s.

## Back up the original first

```bash
python -m esptool --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 --baud 115200 \
  read-flash 0 0x400000 yahboom_cam_full.bin          # about 9 minutes
```

The backup contains your Wi-Fi password (nvs partition): keep it private. To restore
Yahboom's firmware, write it back with `write-flash 0 yahboom_cam_full.bin`.
