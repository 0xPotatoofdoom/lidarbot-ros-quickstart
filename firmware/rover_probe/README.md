# rover_probe: hardware check for the Yahboom MicroROS V2 control board

A small ESP-IDF app that checks every part of the robot's ESP32-S3 control board before we
replace Yahboom's firmware with our own. It boots with the motors off and no servo pulses, scans
both I2C buses, and then runs one test per letter sent over the USB serial port, so a reset never
moves anything.

| Letter | Test |
|---|---|
| `i` | IMU: 1 s of accelerometer and gyro, gravity magnitude, gyro bias |
| `a` | IMU axes: 12 s between beeps while you tip the robot |
| `p` | Power: 3 s of INA228 voltage, current and power, next to the board's own battery reading |
| `b` | Buzzer (two beeps), blue LED, green LED |
| `e` | Encoders: 10 s between beeps while you turn the wheels by hand |
| `m` | Motors: each at 50% and 70%, both directions, 1.5 s each, with encoder counts and current |
| `L` / `R` | Left or right motor only, +70% for 2 s |
| `s` | Servos: pan 0, -30, +30, 0; tilt -45, -30, -55, -45 |
| `l` | Lidar: start scan, count bytes and packets for 3 s, stop |
| `k` | KEY1: 8 s between beeps, counts presses |
| `o` | OLED: all pixels on 2 s, blank 1 s, then a test pattern that stays up |

**Put the robot on a stand before `m`, `L` or `R`.**

## Pin map (V2 board)

From Yahboom's V2 board page ([study/MicroROS-V2/20031](https://www.yahboom.net/study/MicroROS-V2/20031)) and
confirmed with this probe. Yahboom's ESP-IDF samples are for an older 4-motor board and use
different pins: don't take pins from them.

| Function | GPIO | Checked |
|---|---|---|
| Left motor (M1) A / B | 13 / 14 | yes: a positive command on A turns the wheel **backward** |
| Right motor (M2) A / B | 11 / 12 | yes: a positive command on A turns the wheel forward |
| Left encoder A / B | 5 / 4 | yes: counts **down** going forward |
| Right encoder A / B | 7 / 6 | yes: counts up going forward, about 1320 counts per wheel turn |
| Servo S1 (pan) / S2 (tilt) | 8 / 16 | yes: 500-2500 us for -90..+90; pan -30 turns right, tilt larger looks up |
| IMU QMI8658 (0x6B), I2C | SCL 39, SDA 40, INT 41 | yes: mounted turned 180 deg (x points back, y right, z up) |
| OLED header, separate I2C bus | SCL 2, SDA 1 | yes: SSD1306 128x32 OLED at 0x3C (segment remap 0xA1 + COM scan 0xC8 shows it the right way up), our INA228 at 0x40 |
| Buzzer (passive) | 47 | yes |
| LEDs blue / green | 48 / 45 | yes |
| KEY1 (active low) | 42 | yes |
| Battery voltage (ADC1 ch 2, divider x4.03) | 3 | yes: within 20 mV of the INA228 |
| Lidar UART1 TX / RX (230400) | 17 / 18 | yes: about 112 packets/s after `A5 60` |
| Camera module UART2 TX / RX | 36 / 35 | not tested |
| USB host D- / D+ | 19 / 20 | not tested |
| Console UART0 TX / RX (CP2102, USB-C) | 43 / 44 | yes |

## Results on our robot (2026-11-15)

- Motors need more than 50% PWM at 25 kHz to start (Yahboom's own code starts at 50%). At 70%
  with the wheels in the air: about 2700 counts/s (2 wheel turns per second), about 0.1 A per
  motor, 0.6 A peak when starting.
- Idle with this probe (no Wi-Fi, motors off, camera powered): 12.1 V, 0.15 A, 1.8 W.
- IMU at rest: 0.98 g, gyro bias about 1 deg/s per axis before calibration.

## INA228 wiring (battery output)

High side, on the OLED header's I2C bus: VIN+ to the battery plug, VIN- to the board's power
input, **VBUS jumpered to VIN-** (VBUS is the voltage sense input, not ground; on ground it reads
0 V), VS to 3.3 V and GND to GND on the header. 2 mOhm shunt; zero offset under 1 mA.

## Build, flash, restore

The board's CP2102 shows up on a Mac (`/dev/cu.usbserial-*`) only through a USB-A cable or a USB-A
adapter; with a USB-C to USB-C cable the Mac sees nothing. Use 115200 baud: faster rates gave
"Invalid head of packet" errors.

```bash
docker run --rm -v "$PWD":/project -w /project espressif/idf:v5.1.2 idf.py build
esptool --chip esp32s3 --port /dev/cu.usbserial-0001 --baud 115200 write-flash 0x10000 build/rover_probe.bin
python probe.py /dev/cu.usbserial-0001 ipb
```

Only the app partition (0x10000) is written: the partition table matches Yahboom's, and their
bootloader (ESP-IDF 5.1.2) and settings partition (Wi-Fi, agent IP) stay as they are.

**Back up first** (about 6 minutes; the copy contains your Wi-Fi password, keep it private):

```bash
esptool --port /dev/cu.usbserial-0001 --baud 115200 read-flash 0 0x400000 rover_full_4MB.bin
```

To go back to Yahboom's firmware, write the backup (or Yahboom's `MicroROS-V2-v1.1.0-complete.bin`
from their Google Drive) at 0: `write-flash 0 rover_full_4MB.bin`.
