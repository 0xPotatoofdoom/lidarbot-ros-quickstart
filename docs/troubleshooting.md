# Troubleshooting and known issues

## Robot doesn't connect

Work through these in order:

1. **Is the robot on Wi-Fi?** Ping its IP (from your router's client list).
2. **Does it have the right agent address?** Run `.venv/bin/python tools/config_robot.py --show`.
   The agent IP must be the Mac's current IP, and the port must be 8090.
3. **Is the relay receiving?** `tail -f logs/bridge.log` should show
   `session opened for <robot IP>`.
   - `cannot reach agent at 127.0.0.1:8888` means the container's agent isn't up yet
     (or crashed). Check `./ros cat /tmp/base.log`.
   - Nothing at all means the robot's packets aren't reaching the Mac. Check the
     Mac's firewall (allow incoming connections for Python), and that both are on the
     same network. Guest networks sometimes isolate clients from each other.
4. **Do the domain IDs match?** The robot's domain ID (`--show`) must equal
   `ROS_DOMAIN_ID` in docker-compose.yml.
5. **Restart the robot** after changing any of the above.

Docker Desktop's UDP port publishing never delivered the robot's packets, and even with
host networking the replies were dropped. That's why the relay exists. Don't remove
`network_mode: host`.

## Robot ignores drive or servo commands

Unplug the gamepad's USB receiver from the robot. While it's plugged in, the firmware
ignores `/cmd_vel`, `/beep` and `/cmd_ptz_angle`, even though sensor topics keep working.

## slam_toolbox: "Message Filter dropping message"

It can't find the transform for a scan's timestamp, usually because the EKF isn't
running or clocks are off. Check `./ros python3 /work/tools/stamp_check.py`: lags
should be tens of milliseconds, not seconds or negative.

## The map smears or doubles

- Drive and turn more slowly.
- Odometry under-reports rotation by about 6-12% (see below), which the scan matcher has
  to correct. Fast turns make it worse.
- Before suspecting a mirrored or rotated lidar, measure it:
  `./ros python3 /work/tools/scan_rotation_check.py 45` turns 45 degrees and compares the
  rotation found by aligning the scans with what odometry says. The signs should
  match. On this robot they do: `/scan` is correct (REP-103), so don't flip it.

## Known issues

- **Angular odometry:** the wheels and IMU report roughly 6-12% less rotation than the
  lidar measures. Calibration hasn't been done yet.
- **Mirrors and glossy surfaces** return no lidar echo and look like open space. Mirrored
  closet doors are the usual culprit. `move.py` treats a direction as blocked at 0.6 m
  when more than 30% of the beams there are empty, but the map will still show gaps.
- **Opening the robot's USB serial port reboots it**, even with DTR/RTS held low.
- **The firmware's serial boot log includes your Wi-Fi password** in plain text.
- **The ROS 2 CLI daemon** (`ros2 topic list` and so on) occasionally hangs in the
  container. `./ros ros2 daemon stop` fixes it, or use `--no-daemon`.
