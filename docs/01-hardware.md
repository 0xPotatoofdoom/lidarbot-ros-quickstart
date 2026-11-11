# 1. Hardware and how it fits together

## The robot

| Part | Details |
|---|---|
| Controller | ESP32-S3 (ESP32-S3-WROOM-1U-N4R2), runs Yahboom's micro-ROS firmware |
| Drive | 2 DC gear motors with Hall encoders: 1320 counts/rev, 67.5 mm wheels, 171.4 mm track |
| IMU | QMI8658B, published as `/imu` |
| Lidar | Yahboom T-mini Plus, wired to the ESP32 (UART1), published as `/scan`, about 19.5 Hz, 667 points, 360 degrees |
| Camera | Separate ESP32 Wi-Fi camera with its own IP, MJPEG at `http://<camera_ip>:81/stream`, 640x480 |
| Pan/tilt | Two servos under the camera (Superior kit only) |
| Power | 12 V pack, charged through the board |

## Where the software runs

The ESP32 is a **micro-ROS client**. It can't run ROS 2 itself; it publishes and
subscribes through a **micro-ROS agent** on another computer, over Wi-Fi UDP (port 8090).
Everything else (odometry fusion, SLAM, visualization) runs on that computer too.

In this project that computer is your Mac, running a Docker container with ROS 2 Humble:

| Where | What |
|---|---|
| Robot | micro-ROS firmware: `/scan`, `/imu`, `/odom_raw`, `/cmd_vel`, `/cmd_ptz_angle`, `/beep` |
| Mac, native | `tools/udp_tcp_bridge.py`, a UDP-to-TCP relay (see below) |
| Container | micro-ROS agent, robot model (URDF + TF), robot_localization EKF (`/odom`), slam_toolbox (`/map`), camera driver, foxglove_bridge |
| Mac, native | Foxglove, to view everything |

### Why the relay?

Docker Desktop on macOS delivers incoming UDP packets to a container but drops the
replies, so the robot never hears back from the agent. The agent therefore listens on
TCP 8888 inside the container, and `udp_tcp_bridge.py` runs on the Mac, relaying the
robot's UDP datagrams to it and the replies back. `./up` starts it for you. On a
Linux host you wouldn't need it.

## Network

The robot and its camera join your **2.4 GHz** Wi-Fi network (the ESP32 has no 5 GHz
radio). The robot needs to know your Mac's IP address, since that's where it sends
micro-ROS traffic. Give the Mac a DHCP reservation in your router so it doesn't change.

## What about a Raspberry Pi?

A 64-bit Pi (Pi 3, 4 or 5) running Ubuntu 22.04 could run the whole container stack
natively, with no relay, and your Mac would only need Foxglove. That setup isn't
covered here yet. Older 32-bit boards (Pi 1, Pi 2 v1.1, Pi Zero) can't run ROS 2 Humble.
