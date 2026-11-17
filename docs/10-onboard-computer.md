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
- **After a halt, the Pi stays off until its power is removed.** The backup battery keeps it
  powered, so recharging the rover doesn't restart it: switch off the backup board or unplug the Pi.
- When micro-ROS moves to this USB link (firmware phase 3), the serial log is no longer free to
  read; the watch should switch to the `/power` topic then.
