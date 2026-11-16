#!/bin/bash
# Build lidarbot_base in Docker and flash only the app partition (0x10000) over the board's
# USB serial port. The bootloader, partition table and nvs (Wi-Fi, agent IP, domain) are kept.
#
#   ./build_and_flash.sh              build + flash
#   ./build_and_flash.sh build        build only
#   PORT=/dev/cu.usbserial-0001 ./build_and_flash.sh
#
# Back up the original firmware first (firmware/rover_probe/README.md).
set -euo pipefail
cd "$(dirname "$0")"
export PATH="/Applications/Docker.app/Contents/Resources/bin:$PATH"
IMAGE=lidarbot-fw:5.5.1
PORT="${PORT:-/dev/cu.usbserial-0001}"
ESPTOOL="${ESPTOOL:-arch -arm64 ../../.venv-fw/bin/esptool}"

docker image inspect "$IMAGE" >/dev/null 2>&1 || docker build -q -t "$IMAGE" .
docker run --rm -v "$PWD":/project -w /project "$IMAGE" \
  bash -c 'idf.py build 2>&1 | grep -E "warning:|error:|binary size|FAILED" || true; test -f build/lidarbot_base.bin'
shasum -a 256 build/lidarbot_base.bin
[ "${1:-}" = "build" ] && exit 0
# 115200: faster rates gave "Invalid head of packet" errors on this board's CP2102.
$ESPTOOL --chip esp32s3 --port "$PORT" --baud 115200 write-flash 0x10000 build/lidarbot_base.bin \
  2>&1 | tr '\r' '\n' | grep -E 'Wrote|verified|rror'
