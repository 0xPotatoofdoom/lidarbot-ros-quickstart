# Yahboom MicroROS V2: getting started on a Mac

A working ROS 2 Humble setup for the **Yahboom MicroROS V2** robot car
(ESP32-S3, differential drive, T-mini Plus lidar, Wi-Fi pan/tilt camera), running
in Docker on an Apple Silicon Mac. Yahboom only supports an x86 VMware image, which
doesn't run on M-series Macs.

With it you can:

- connect the robot over Wi-Fi and see its lidar, odometry, IMU and camera in Foxglove
- drive it with closed-loop moves that stop for obstacles
- build a 2D map of your home with slam_toolbox, by hand or with a simple explorer
- save and reload maps

It also documents the problems we hit along the way (Docker dropping UDP, the
gamepad receiver silently blocking commands, stale camera frames, and more) so you
don't have to rediscover them.

```text
 robot (ESP32-S3, micro-ROS client)
   |  Wi-Fi, UDP 8090
   v
 tools/udp_tcp_bridge.py          native on the Mac
   |  TCP 8888
   v
 Docker container (ROS 2 Humble, arm64)
   micro-ROS agent -> EKF (/odom) -> slam_toolbox (/map) -> foxglove_bridge :8765
                                                              |
                                              Foxglove app on the Mac
```

## Quick start

Assumes the robot is built, charged, and already drives with the gamepad.

```bash
./fetch-yahboom                                   # download + patch Yahboom's ROS packages
cp config/robot_info.example.json config/robot_info.json   # then set camera_ip
uv venv .venv && uv pip install --python .venv pyserial
docker compose build                              # several minutes the first time
.venv/bin/python tools/config_robot.py --ssid "<2.4 GHz Wi-Fi>" --agent-ip "$(ipconfig getifaddr en0)"
# unplug the gamepad's USB receiver from the robot, then:
./up slam
```

Open Foxglove, connect to `ws://localhost:8765`, and add a 3D panel for `/scan` and `/map`.
`./down` stops everything.

## Guide

1. [Hardware and how it fits together](docs/01-hardware.md)
2. [Mac setup: Docker, Python, Yahboom packages](docs/02-mac-setup.md)
3. [Robot setup: Wi-Fi and agent address](docs/03-robot-setup.md)
4. [First drive: Foxglove, teleop, scripted moves](docs/04-first-drive.md)
5. [Mapping with slam_toolbox](docs/05-mapping.md)
6. [Camera and pan/tilt](docs/06-camera.md)
7. [Calibrating odometry](docs/07-calibration.md)
8. [Recording datasets](docs/08-recording.md)
9. [Navigating with Nav2](docs/09-navigation.md)
9. [Troubleshooting and known issues](docs/troubleshooting.md)

## Status

Tested on an M3 Pro Mac (macOS 26, Docker Desktop) with the Superior kit
(`CAR_TYPE=ptz`) and firmware 1.1.0. Other kits (`no_ptz`, `basic_car`) and other
hosts (Intel Macs, Linux, Windows) should work with small changes but haven't been tried.
Odometry is calibrated against the lidar to within about 0.5% for distance and 0.05%
for heading (see [calibration](docs/07-calibration.md)).

## License

This repository is MIT licensed (see [LICENSE](LICENSE)). Yahboom's packages are
**not** included: `./fetch-yahboom` downloads them from Yahboom's own release and
applies the small patches in [patches/](patches/). They remain under Yahboom's
Business Source License 1.1.

This project is not affiliated with or endorsed by Yahboom.
