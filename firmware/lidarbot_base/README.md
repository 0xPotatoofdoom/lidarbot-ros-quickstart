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
| 1 | Sensors: IMU, battery current (INA228), OLED status | **running** (2026-11-16) |
| 2 | Driving: motors with PID speed control from the encoders, `/cmd_vel`, `/odom_raw`, lidar `/scan`, pan/tilt, buzzer | next |
| 3 | micro-ROS over USB serial to the Orange Pi on the robot, instead of Wi-Fi | later |

## Phase 1

Motors are held off and the pan/tilt servos hold the power-on position (pan 0, tilt -45).

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
