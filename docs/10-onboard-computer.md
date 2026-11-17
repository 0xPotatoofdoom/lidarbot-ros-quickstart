# 10. Onboard computer: Orange Pi Zero 2W

An Orange Pi Zero 2W rides on the rover as its ROS computer. It will run the micro-ROS agent
over USB serial to the control board (instead of Wi-Fi to the Mac), and it carries the sensors
the control board has no room for: air temperature, humidity and pressure (BME280), battery
and motor temperatures (DS18B20 probes) and an IR receiver for the charging dock's beacons.

Our board: 2 GB RAM, 4 cores (H618), the vendor image "Orange Pi 1.0.4 Noble" (Ubuntu 24.04,
kernel 6.1.31-sun50iw9), hostname `roverbot`. Set up on the desk 2026-11-16 before moving it
onto the rover.

## Access

SSH with a key; password logins are off:

```bash
ssh-copy-id -i ~/.ssh/<key>.pub orangepi@roverbot.local      # once, while passwords still work
echo 'PasswordAuthentication no' | sudo tee /etc/ssh/sshd_config.d/10-keys-only.conf
sudo systemctl restart ssh                                     # test a new login before closing this one
```

During setup we gave the `orangepi` user passwordless sudo (`/etc/sudoers.d/90-orangepi-nopasswd`,
written with `visudo -c` checking it). Remove that file once the board is set up.

## Serial console off the header (unclean reboots)

After soldering on the 40-pin header the board sometimes didn't come back from a reboot, and
the kernel logged "sysrq: HELP" messages. The image runs a serial console on header pins 8
and 10 (UART0), and the kernel's magic SysRq keys were on (`kernel.sysrq = 176`, which includes
immediate reboot). With nothing connected, pin 10 (RX) picks up noise: in Linux that can
trigger SysRq, and during boot U-Boot reads it as a key press and stops autoboot at its prompt,
which looks like a hang. Fix:

```bash
echo 'kernel.sysrq = 0' | sudo tee /etc/sysctl.d/99-no-sysrq.conf && sudo sysctl --system
sudo systemctl disable --now serial-getty@ttyS0
sudo cp /boot/orangepiEnv.txt /boot/orangepiEnv.txt.bak
sudo sed -i 's/^console=both/console=none/' /boot/orangepiEnv.txt
echo 'extraargs=console=tty1' | sudo tee -a /boot/orangepiEnv.txt    # kernel console on HDMI only
```

`console=display` is not enough: the boot script (`/boot/boot.cmd`) adds `ttyS0` for both
`display` and `both`, and with no `console=` at all the kernel falls back to the serial port from
the device tree. Cleaning the flux off the header helps too. If it ever hangs at boot again, a
monitor on the mini-HDMI port shows whether it stopped at U-Boot's `=>` prompt; a 10 kOhm
resistor from pin 10 to pin 1 (3.3 V) would hold RX quiet.

## Power off restarts it; use halt

`sudo poweroff` brings the board straight back up. Neither telling the power chip it controls
power-off (device tree) nor writing the AXP313A's shutdown register at the end of shutdown kept
it off; most likely the power chip switches on whenever input power is present. Before cutting
power with the rover's switch:

```bash
sudo systemctl halt      # clean shutdown, then the CPU stops and stays stopped
```

The kernel also prints "irq 70: nobody cared ... Disabling IRQ #70" about a minute after every
boot: the AXP313A's interrupt line fires continuously and the kernel switches it off. Harmless
(only power-button events are lost).

## ROS 2 Jazzy and the micro-ROS agent

```bash
sudo apt-get update && sudo apt-get install -y software-properties-common curl
sudo add-apt-repository -y universe
V=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F tag_name | cut -d'"' -f4)
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${V}/ros2-apt-source_${V}.noble_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb && sudo apt-get update
sudo apt-get install -y --no-install-recommends ros-jazzy-ros-base ros-dev-tools \
  ros-jazzy-robot-localization ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

1,257 packages, 3 GB on the SD card. Install only; don't `apt upgrade` the whole system, so the
vendor kernel stays as it is. The micro-ROS agent isn't packaged for Jazzy; build it like the
Docker image does, two jobs at a time for 2 GB of RAM (13 minutes):

```bash
source /opt/ros/jazzy/setup.bash
sudo rosdep init; rosdep update
mkdir -p ~/uros_ws/src && cd ~/uros_ws
git clone --depth 1 -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git src/micro-ROS-Agent
git clone --depth 1 -b jazzy https://github.com/micro-ROS/micro_ros_msgs.git src/micro_ros_msgs
rosdep install --from-paths src --ignore-src -y
MAKEFLAGS=-j2 colcon build --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The kernel has the CP210x driver, so the control board's USB port shows up as `/dev/ttyUSB0`.

## Sensors and header pins

Everything on 3.3 V (pins 1 and 17): the Orange Pi's pins are not 5 V tolerant.

| Sensor | Wires | Linux |
|---|---|---|
| BME280 | VCC pin 1, SDA pin 3, SCL pin 5, GND pin 9 | `/dev/i2c-2`, address 0x76 (`overlays=pi-i2c1`) |
| DS18B20 adapter + probes | VCC pin 17, DATA **pin 29**, GND pin 14 or 30 | `/sys/bus/w1/devices/28-*/temperature` (`onboard/overlays/rover-w1-pin29.dts`) |
| 38 kHz IR receiver | VCC pin 1 or 17, OUT pin 12, GND pin 20 | `/dev/lirc0` (`onboard/overlays/rover-ir-pin12.dts`) |

Pins 3 and 5 are I2C controller 1 in the overlay's name but Linux bus 2 (`/dev/i2c-2`, the
controller at 5002400); bus 3 is an on-board device. Install the overlays from this repo:

```bash
sudo orangepi-add-overlay onboard/overlays/rover-w1-pin29.dts
sudo orangepi-add-overlay onboard/overlays/rover-ir-pin12.dts
grep -q '^overlays=' /boot/orangepiEnv.txt || echo 'overlays=pi-i2c1' | sudo tee -a /boot/orangepiEnv.txt
echo w1-therm | sudo tee /etc/modules-load.d/w1-therm.conf
sudo apt-get install -y i2c-tools gpiod ir-keytable
sudo systemctl reboot
```

**Pins to avoid for inputs:** on our board, pins 7, 16, 18, 32, 35, 36, 37, 38 and 40 read low
even with the internal pull-up on, and pin 15 reads high with the pull-down on (libgpiod,
2026-11-16). Pin 7 (the one-wire pin first) also stayed low with nothing connected, so the
DS18B20s moved to pin 29. Pins that respond normally: 11, 13, 22, 29, 31, 33. Test a pin before
using it (`gpioget --bias=pull-up gpiochip0 <line>`, then `--bias=pull-down`; the line numbers
are in `gpio readall`). The wiringOP `gpio read` command returns 0 for every pin on this kernel,
so don't trust it.

### Checks (2026-11-16, on the desk)

- **BME280:** chip ID register 0xD0 reads 0x60 (a BMP280 would read 0x58). Read it through the
  kernel driver: `sudo modprobe bmp280-i2c; echo bme280 0x76 | sudo tee /sys/bus/i2c/devices/i2c-2/new_device`,
  then `/sys/bus/iio/devices/iio:device0/in_{temp,humidityrelative,pressure}_input`. 22-26 C,
  36% RH and a sensible pressure for our altitude. Mounted on the board it reads 3-4 C warm (the
  CPU runs at about 55 C): put it away from the electronics.
- **DS18B20:** one probe found, 22.5 C at 12-bit resolution, ROM ID in the genuine pattern
  (`28-0000...`; clones sometimes copy it, so it's a hint, not proof). With the data line stuck
  low the bus instead reports a phantom `00-...` device and "max_slave_count 64 reached".
- **IR receiver:** a Sharp TV remote was received cleanly (194 frames in 30 s, address 1,
  commands 0x01 and 0x02). The kernel's decoders found nothing: timing edges by GPIO interrupt
  shortens pulses by about 30 us and stretches gaps by 50-140 us, outside the Sharp decoder's
  tolerance. Capture raw and decode in userspace:

```bash
sudo timeout 30 ir-ctl -d /dev/lirc0 -r --mode2 > ir_raw.txt
python3 onboard/ir_decode.py ir_raw.txt
```

The Roomba Home Base's beacons use their own codes, so they'll get their own decoder there.
The receiver is directional: aim at its dome from under a metre while testing.

## Battery watch: halt at critical

The Pi connects to the rover board over USB (the board's CP2102 appears as `/dev/ttyUSB0`).
The firmware's battery guard stops the wheels and lidar at CRITICAL (under 10.2 V resting, see
`firmware/lidarbot_base/main/battery_guard.h`). `onboard/battery_watch.py` reads the board's status
line (every 10 s) from the serial log and halts the Pi if CRITICAL lasts 60 s, so the SD card
isn't cut mid-write when the pack or the Pi's backup battery runs out. A charger that brings the
pack back above the threshold cancels it. Silence on the port never triggers a halt.

```bash
scp onboard/battery_watch.py onboard/battery-watch.service orangepi@roverbot.local:~/
ssh orangepi@roverbot.local
sudo install -m 755 battery_watch.py /usr/local/bin/
sudo install -m 644 battery-watch.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now battery-watch
journalctl -u battery-watch -f          # state changes, plus a summary every 5 minutes
```

- Standard library only (termios), so nothing to install. Tests: `python -m pytest onboard/test_battery_watch.py`.
- Opening the port doesn't reset the ESP32: the board's scan counter kept counting across opens.
- Raw serial lines are never logged: the boot log can include network settings.
- `--dry-run` logs the halt instead of doing it.

### Testing it with the firmware's battery test mode

The firmware takes `batt sim <volts>` on its serial console: the battery guard uses that resting
voltage instead of the INA228 (the OLED shows `SIM 10.40V`; `/power` keeps the real readings).
`batt sim off` ends it, and it ends by itself after 10 minutes. Keep the port open while sending
(the watcher does), and clear `hupcl` once so closing it can't reset the board:

```bash
sudo systemctl stop battery-watch
sudo stty -F /dev/ttyUSB0 -hupcl
sudo python3 -u /usr/local/bin/battery_watch.py --dry-run &
printf 'batt sim 10.4\n' | sudo tee /dev/ttyUSB0    # LOW after ~10 s: double beeps
printf 'batt sim 10.0\n' | sudo tee /dev/ttyUSB0    # CRITICAL: long beeps, wheels and lidar stop
# ~60 s later the watcher logs "halting" (dry run: it doesn't)
printf 'batt sim off\n' | sudo tee /dev/ttyUSB0
kill %1; sudo systemctl start battery-watch
```

Checked 2026-11-17: OK, LOW, CRITICAL, "halting" after 60 s, then back to OK after `batt sim off`.

### Flashing the rover board from the Pi

With the rover's USB on the Pi, flash from it instead of the Mac. Build on the Mac
(`firmware/lidarbot_base/build_and_flash.sh build`), copy `build/lidarbot_base.bin` over, then:

```bash
sudo apt install esptool     # 4.7; Ubuntu's package has no flasher stubs, hence --no-stub
sudo systemctl stop battery-watch
esptool --chip esp32s3 --port /dev/ttyUSB0 --baud 115200 --no-stub write_flash 0x10000 lidarbot_base.bin
sudo systemctl start battery-watch
```

It takes about 95 s (no stub) and keeps the bootloader, partition table and saved settings.
- **After a halt, the Pi stays off until its power is removed.** The backup battery keeps it
  powered, so recharging the rover doesn't restart it: switch off the backup board or unplug the Pi.
- With micro-ROS on this USB link (firmware `transport serial`) the serial log isn't free to read:
  `BATTERY_WATCH_SOURCE=ros` in `/etc/default/battery-watch` makes the watch read `/battery_guard`
  and `/power` instead. It runs as `orangepi`, not root: as root it never received the rover's
  topics (DDS between root and the user's processes didn't connect, with or without Fast DDS
  shared memory). Halting goes through one sudoers rule, `onboard/battery-watch.sudoers`
  (`systemctl halt` only), installed with `visudo -c`.
- Checked over ROS 2026-11-17 (dry run): `/battery_sim` 10.4 -> LOW, 10.0 -> CRITICAL, "halting"
  after 60 s, `/battery_sim` 0 -> OK.

## The rover's ROS stack on the Pi

The Pi runs the stack natively (ROS 2 Jazzy, no Docker): micro-ROS agent, robot model,
odometry calibrator, EKF, battery monitor, Foxglove bridge, and a lean Nav2. The Mac only needs
the Foxglove app, connected to `ws://roverbot.local:8765`.

| File | What it does |
|---|---|
| `onboard/rover-up.sh` | sources ROS and the agent workspace, domain 77, DDS kept on the Pi, then launches |
| `onboard/launch/rover_pi.launch.py` | agent, `robot_state_publisher`, `ptz_joint_states.py`, calibrator, EKF, battery monitor, Foxglove; `nav:=true` adds Nav2 |
| `onboard/launch/nav_pi.launch.py` | Nav2 in one process: map server, AMCL, planner, controller, behaviours, BT navigator, velocity smoother, collision monitor |
| `onboard/ptz_joint_states.py` | `/ptz_angle` to `/joint_states` (pan/tilt), replacing Yahboom's `base_driver` and `joint_state_publisher` |

The robot model comes from Yahboom's `robot_description` (Business Source License, so it isn't
in this repo). Generate it in the Mac container and copy it to the Pi:

```bash
docker exec lidarbot bash -lc 'source /root/microros_ws/install/setup.bash; xacro $(ros2 pkg prefix robot_description)/share/robot_description/urdf/microros_v2.urdf.xacro camera_type:=$CAR_TYPE' > robot.urdf
scp robot.urdf orangepi@roverbot.local:lidarbot/private/robot.urdf
```

Copy `onboard/`, `config/`, `maps/` and the tools the launch file uses (`odom_calibrator.py`,
`battery_monitor.py`, `power_meter.py`) to `~/lidarbot` on the Pi, install
`ros-jazzy-foxglove-bridge`, then:

```bash
onboard/rover-up.sh agent_transport:=tcp4 agent_port:=8888 nav:=true   # behind the Mac's relay
onboard/rover-up.sh nav:=true                                          # rover sends UDP 8090 straight to the Pi
```

At boot, `onboard/rover-stack.service` starts it (as `orangepi`, arguments from
`/etc/default/rover-stack`, here `ROVER_ARGS="agent_transport:=serial nav:=true"`). Stop it
before flashing the board: `sudo systemctl stop rover-stack`.

The rover talks to the Pi over the USB cable (firmware `transport serial`, see
`firmware/lidarbot_base/README.md`): `onboard/rover-up.sh agent_transport:=serial nav:=true`.
Over Wi-Fi instead, either `agent <pi-ip> 8090` on the firmware console with the default
`agent_transport:=udp4`, or the Mac's relay pointed at the Pi:
`.venv/bin/python tools/udp_tcp_bridge.py --agent-host 192.168.1.140` with `agent_transport:=tcp4
agent_port:=8888` (plain `./up` points the relay back at the container).

### Does it fit? (2026-11-17)

Replaying the 7-minute Nav2 loop of 2026-11-12 at real time (`tools/replay_amcl.py`, run natively
with `LIDARBOT_MAP` / `LIDARBOT_CONFIG`), compared with the pose AMCL produced live on the robot:

| Run on the Pi | Position diff, median / 95% / max | Nav2 CPU (of 400%) |
|---|---|---|
| Localization only | 1.1 / 2.7 / 3.7 cm | AMCL 12% |
| Jazzy `bringup_launch.py` | 1.2 / 2.9 / 4.0 cm | 191% |
| `nav_pi.launch.py` | 1.1 / 2.6 / 3.3 cm | 99% |

Jazzy's bringup also starts the smoother, route, waypoint and docking servers, which our
behaviour trees don't use; one process per node (`use_composition:=False`) saturated the CPU.

Live, with the rover's data arriving and the robot standing still: Nav2 76%, calibrator 28%
(Python, 75 messages a second), EKF 10-12%, agent 5%, battery monitor 4-5%; about a third of the
CPU in total, 54 C. Memory is the tighter limit on this 1 GB board: about 340 MB available and
60-100 MB in swap; each Python node costs about 70 MB. Not running yet: the camera driver and the
floor obstacle detector.

- Start the agent binary as a launch `Node`, not `ros2 run`: `ros2 run` didn't pass SIGINT on, the
  orphaned agent kept port 8888, and the next start failed with "bind error".
- The EKF misses its 20 Hz rate for a few cycles while everything starts; it settles at about 19-20 Hz.

## Power: the Pi runs from the pack

The Pi takes 5 V on header pins 4 (5 V) and 6 (GND) from a 3 A buck converter, fed from the rover
board's charge jack. That jack is live with the main switch off and sits after INA228 #1, so the
Pi's power is in every `/power` reading: loading all four Pi cores raised the pack current from
0.49 to 0.56 A (about 0.8 W) and it fell back after. Nothing may be plugged into that jack as a
charger now; charge through the pack's own pigtail. (A power-bank board between the rover's USB-A
and the Pi cut out under load: the Pi crashed twice and two flashes failed. Don't use one.)

With the switch off, the Pi's USB port powers the rover board back through the cable: the ESP32,
lidar and guard kept running and the wheels and servos still moved (at about half speed). So the
firmware reads the switched rail (GPIO3, `/rail_voltage`: 11.3 V on, 4.9 V off) and below 6 V for
1 s stops the motors, lidar and servos (OLED: SWITCH OFF). Checked on the stand: 0.15 m/s commanded
gave 0.000 at the encoders with the switch off, 0.164 with it on.

The battery watch halts the Pi when:

| Condition | After |
|---|---|
| `/battery_guard` CRITICAL | 60 s |
| `/rail_voltage` under 6 V (main switch off) | 30 s |
| no `/battery_guard` at all while `rover-stack` runs (a cable with its 5 V blocked: the board really powers down) | 30 s |

A halted Pi stays on the converter until its power is cut: unplug the pigtail or add an inline
switch, or the halted Pi and the converter keep drawing from the pack with nothing watching.
