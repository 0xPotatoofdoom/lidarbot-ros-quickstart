# lidarbot_base: open firmware for the Yahboom MicroROS V2 control board

Replacement firmware for the robot's ESP32-S3 control board, built on
[micro-ROS](https://github.com/micro-ROS/micro_ros_espidf_component) (Jazzy) and ESP-IDF 5.5.
It reuses the settings Yahboom's firmware saved on the board (Wi-Fi network, agent IP and port,
ROS domain), so nothing about your network is in the source, and `tools/config_robot.py` still
works for changing them.

Built in phases. It replaces Yahboom's firmware for driving only once odometry and Nav2 match
the calibration we measured with Yahboom's firmware (docs/07).

| Phase | What | Status |
|---|---|---|
| 1 | Sensors: IMU, battery current (INA228), OLED status | **done** (2026-11-16) |
| 2 | Driving: motors with PID speed control from the encoders, `/cmd_vel`, `/odom_raw`, lidar `/scan`, pan/tilt, buzzer | **running on the stand** (2026-11-16); floor calibration and Nav2 next |
| 3 | micro-ROS over USB serial to the Orange Pi on the robot, instead of Wi-Fi | **running** (2026-11-17): full topic rates over the cable; floor test next |

## Phase 1

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/imu` | `sensor_msgs/Imu` | 50 Hz | Robot frame (x forward, y left, z up), frame `imu_frame`. Orientation from a complementary filter: roll and pitch from gravity, yaw from the gyro. |
| `/battery` | `std_msgs/Float32` | 5 Hz | Pack volts, same topic as Yahboom's firmware, so `tools/battery_monitor.py` keeps working |
| `/power` | `sensor_msgs/BatteryState` | 5 Hz | INA228: voltage, current (negative while discharging, the ROS convention), the INA228's temperature |
| `/power_used_wh` | `std_msgs/Float32` | 5 Hz | Energy drawn from the pack since boot, from the INA228's own accumulator |

**Keep the robot still for the first 2 seconds after power-on**: the firmware measures the gyro's
bias then. The OLED shows volts and amps on top and the connection state below (`WIFI...`,
`AGENT...`, `ROS OK` with the power draw). Blue LED blinks while it looks for the agent, green is
on while connected, and it beeps once when it connects.

Measured on our robot (2026-11-16, robot still, Humble agent on the Mac):

| | |
|---|---|
| `/imu` rate | 47.7 Hz over 60 s (best effort over Wi-Fi drops a few) |
| Timestamps | 26 ms behind ROS time (median; 24-31 ms): the Wi-Fi and relay delay |
| Yaw drift while still | -0.02 deg in 60 s |
| Idle draw | 12.05 V, 0.18 A, 2.2 W with Wi-Fi on |

The firmware is built with micro-ROS Jazzy and works with our Humble agent: the agent only
relays messages, and these message types are the same in both.

## Phase 2

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/odom_raw` | `nav_msgs/Odometry` | 25 Hz | `odom` -> `base_footprint`, from the encoders; Yahboom's covariances, so `odom_calibrator` and the EKF take it unchanged |
| `/scan` | `sensor_msgs/LaserScan` | about 6 Hz | `laser_frame`, 667 rays from -pi to pi, 0.12-8 m, 0 = no echo (Yahboom's layout). Reliable QoS; decoded on the board from the T-mini Plus |
| `/ptz_angle` | `geometry_msgs/Vector3` | 10 Hz | pan (x), tilt (y) in degrees, as commanded |
| `/cmd_vel` (in) | `geometry_msgs/Twist` | | linear.x, angular.z. The wheels stop 0.5 s after the last command. Wheel speeds are capped at 0.6 m/s, scaling both together so the turn radius is kept |
| `/cmd_ptz_angle` (in) | `geometry_msgs/Vector3` | | pan -90..90, tilt -90..20. Power-on position pan 0, tilt -40 (camera level) |
| `/beep` (in) | `std_msgs/UInt16` | | milliseconds |

Each wheel has a 100 Hz speed loop: feedforward (the motors need about 50% duty to start) plus
PI on the encoder speed. A target of 0 brakes the wheel (both motor inputs high) until it has
stopped, then lets go; also after the 0.5 s command timeout. Stopping distance after a stop
command, on the floor: 8 cm from 0.6 m/s (28 cm when coasting, as Yahboom's firmware does), 4 cm
from 0.3 m/s.

Checked on the stand (2026-11-16, wheels in the air):

| | |
|---|---|
| Lidar angles | box straight ahead at 0 deg (-4 deg, box slightly off square), 0.405 m measured vs 16 in (0.406 m); a person behind-left at +122..+158 deg, so not mirrored |
| Lidar packets | 0 bad out of 2215 |
| Forward / reverse 0.15 m/s | 0.148-0.152 m/s measured, sideways drift about 1 cm over 0.6 m |
| Turn on the spot 1.0 rad/s | 0.97-1.08 rad/s |
| Command timeout | stopped about 0.6 s after the last `/cmd_vel` |
| Draw | 0.30 A idle (lidar spinning), about 0.40 A with both wheels at 0.15 m/s |
| Full load (floor) | 0.6 m/s held at 0.601-0.604, 0.66-0.75 A mean, 1.14 A peak, pack 11.5 -> 11.28 V |
| Pan/tilt | left, right, up, down as commanded; a tilt request of +60 is capped at +20 |

Odometry on the floor and Nav2 are not checked yet: Yahboom's firmware stays the reference for
driving until they match docs/07.

## micro-ROS link: UDP or the USB cable

`main/transport.c` is a micro-ROS custom transport with two backends, chosen at boot from nvs
(`LIDARBOT` / `transport`):

- **udp** (default): Wi-Fi to the agent address in Yahboom's settings, the same packets as
  micro-ROS's own UDP transport.
- **serial**: UART0, the board's CP2102 on the USB-C port, at 921600 baud, to the agent on the
  Orange Pi (`micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600`). Logs are off from the
  moment it starts (they would land in micro-ROS's byte stream); the OLED still shows the state.

Console commands (115200 baud, UDP mode only, since serial mode owns the console):

| Command | Effect |
|---|---|
| `transport serial` / `transport udp` | saved, then the board restarts |
| `agent 192.168.1.140 8090` | UDP agent address, saved for the next restart |
| `batt sim 10.4` / `batt sim off` | battery guard test mode (also the `/battery_sim` topic) |

**Back to UDP without reflashing:** hold KEY while switching on. That boot uses UDP with logs and
the console, and `transport udp` makes it stick. Flashing works in either mode (the ROM bootloader
takes the port).

Measured over the cable (2026-11-17, Pi agent): `/imu` 49.6 Hz, `/odom_raw` 25.0 Hz, `/scan`
6.1 Hz, `/power` and `/battery_guard` about 4.6 Hz. At first `/imu` ran at 42 Hz: each reliable
`/scan` publish waited up to 200 ms for the agent's acknowledgement, which takes tens of
milliseconds over serial (a scan alone is 30 ms on the wire). In serial mode it no longer waits;
the scan stays queued and is resent until acknowledged.

New topics for serial mode: `/battery_guard` (std_msgs/UInt8, the guard's state: 0 no pack,
1 OK, 2 LOW, 3 CRITICAL) and `/battery_sim` (std_msgs/Float32: test volts, 0 ends it).

## Build and flash

```bash
./build_and_flash.sh build     # build only (first build compiles micro-ROS: a few minutes)
./build_and_flash.sh           # build and flash the app partition over USB (115200, about 50 s)
```

Needs Docker; the script builds a `lidarbot-fw:5.5.1` image (`espressif/idf:v5.5.1` plus the
Python tools micro-ROS needs) the first time. Flashing needs esptool (`.venv-fw` at the repo root)
and the board's USB-C port on a USB-A cable or adapter: with USB-C to USB-C the Mac doesn't see it.

Only the app partition (0x10000) is written. Yahboom's bootloader (ESP-IDF 5.1.2) starts this
5.5 app fine, and their nvs partition keeps the settings. Back up the original firmware first and
restore it with `write-flash 0 <backup>`: see `firmware/rover_probe/README.md`.

## Settings it reads

From the `YAHBOOM` namespace in nvs (seen on our board; `tools/config_robot.py` sets them):
`CFG_AGENT_IP` (4 bytes), `CFG_AGENT_PORT`, `CFG_DOMAIN_ID`, and `CFG_SSID` / `CFG_PASSWD` if the
Wi-Fi driver has no saved network of its own. Pins and directions are in `main/board.h`, checked
with `firmware/rover_probe`.
