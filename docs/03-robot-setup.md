# 3. Robot setup

The robot stores its Wi-Fi network, the agent's address and its ROS domain ID. You set
them once over the USB-C cable with `tools/config_robot.py`, a Mac-friendly version of
Yahboom's `config_robot` script.

## Before you start

- Charge the robot and turn it on.
- Plug it into the Mac with a USB-C data cable. It shows up as
  `/dev/cu.usbserial-*` (a CP2102 chip; macOS has the driver built in).
- Find your Mac's IP: `ipconfig getifaddr en0`.

**Opening the serial port reboots the robot.** That's normal, and it's how the tool
enters the configuration window.

## Read the current settings

```bash
.venv/bin/python tools/config_robot.py --show
```

## Set Wi-Fi and agent address

```bash
.venv/bin/python tools/config_robot.py --ssid "<2.4 GHz network>" --agent-ip <Mac IP>
```

The tool prompts for the Wi-Fi password (or reads `LIDARBOT_WIFI_PASSWD`), so it never
ends up in a file or your shell history. Other options:

| Option | Default | |
|---|---|---|
| `--agent-port` | 8090 | UDP port the robot sends to; the relay listens here |
| `--domain-id` | 77 | Must match `ROS_DOMAIN_ID` in docker-compose.yml |
| `--namespace` | none | Topic prefix, for running several robots |
| `--no-camera` | | Skip sending Wi-Fi settings to the camera |
| `--port` | auto | Serial device, if more than one is plugged in |

> The firmware prints the Wi-Fi password in plain text in its serial boot log. Don't
> paste serial logs into issues or chats without removing it.

The camera gets the same Wi-Fi settings and picks up its own IP address. Find it in
your router's client list (it's a separate device from the robot) and put it in
`config/robot_info.json`.

## Unplug the gamepad receiver

**While the gamepad's USB receiver is plugged into the robot, the firmware ignores
`/cmd_vel`, `/beep` and the pan/tilt servos.** Everything else (lidar, IMU, odometry)
still publishes, so this one is easy to miss. Unplug it whenever you want ROS to drive.

## Check it's talking

```bash
./up
./ros ros2 topic list         # expect /scan, /imu, /odom_raw, /odom, /cmd_vel, ...
./ros ros2 topic hz /scan     # about 19.5 Hz
tail -f logs/bridge.log       # "session opened for <robot IP>" when the robot connects
```

If nothing shows up, see [troubleshooting](troubleshooting.md#robot-doesnt-connect).
