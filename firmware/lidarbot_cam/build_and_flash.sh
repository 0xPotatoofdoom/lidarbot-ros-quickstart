#!/bin/bash
# Build lidarbot-cam in Espressif's Docker image and flash only the app partition (0x10000)
# through a Linux machine the camera module is plugged into (macOS has no driver for its
# CH340 variant). The bootloader, partition table and nvs (Wi-Fi settings) are not touched.
#
#   ./build_and_flash.sh                 build + flash via PI_HOST
#   ./build_and_flash.sh build           build only
#   PI_HOST=matt@192.168.1.181 PI_KEY=~/.ssh/2026 ./build_and_flash.sh
#
# Back up the original firmware first (see docs/06-camera.md) so it can be restored.
set -euo pipefail
cd "$(dirname "$0")"
export PATH="/Applications/Docker.app/Contents/Resources/bin:$PATH"
PI_HOST="${PI_HOST:-matt@192.168.1.181}"
PI_KEY="${PI_KEY:-$HOME/.ssh/2026}"
PI_PY="${PI_PY:-~/spiderbot/.venv/bin/python}"          # a venv with esptool installed
PORT="${PORT:-/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0}"
SSH=(ssh -i "$PI_KEY" -o IdentitiesOnly=yes -o ServerAliveInterval=30)

docker run --rm -v "$PWD":/project -w /project espressif/idf:v5.1.2 \
  bash -c 'idf.py build 2>&1 | grep -E "warning:|error:|binary size|FAILED" || true; test -f build/lidarbot_cam.bin'
shasum -a 256 build/lidarbot_cam.bin
[ "${1:-}" = "build" ] && exit 0

scp -q -i "$PI_KEY" -o IdentitiesOnly=yes build/lidarbot_cam.bin "$PI_HOST:/tmp/lidarbot_cam.bin"
# 115200 baud: faster rates gave SLIP errors with this CH340.
"${SSH[@]}" "$PI_HOST" "$PI_PY -m esptool --chip esp32s3 --port $PORT --baud 115200 \
  write-flash 0x10000 /tmp/lidarbot_cam.bin 2>&1 | tr '\r' '\n' | grep -E 'Wrote|verified|rror'; rm -f /tmp/lidarbot_cam.bin"
