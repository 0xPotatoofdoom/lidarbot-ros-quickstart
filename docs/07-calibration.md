# 7. Calibrating odometry

The firmware has no wheel-size or track-width settings, so corrections are applied on
the host. `tools/odom_calibrator.py` (started by `./up`) rescales the robot's output
before the EKF:

```text
/odom_raw -> odom_calibrator -> /odom_cal --+
/imu      -> odom_calibrator -> /imu_cal  --+--> EKF -> /odom
```

The factors live in [config/calibration.yaml](../config/calibration.yaml). Edit them and
run `./up` again to apply.

## Measuring

`tools/calibrate.py` measures each motion source against the lidar, which is accurate
to a fraction of a degree and a few millimeters here.

**Rotation.** It turns in place (two full turns left, then two right), then compares the
lidar scans taken before and after. Scan alignment gives the true rotation, modulo 360
degrees, and the EKF supplies the whole turns. It needs about 0.5 m of space around the
robot, and somewhere with walls and furniture in view (not a featureless circle).

```bash
./record run calib-angular ./ros python3 /work/tools/calibrate.py angular 2
```

**Distance.** It drives 1 m toward a wall and back. The true distance is the change in
the lidar range straight ahead, which doesn't depend on the wall's angle. It needs a flat,
non-reflective wall 1.5-5 m ahead.

```bash
./record run calib-linear ./ros python3 /work/tools/calibrate.py linear 1.0
```

For a manual check, lay a tape measure along the path, starting under the front edge of
the chassis. The robot pauses for 15 seconds at the far point so you can read it.

Each run prints what every source measured and the suggested factors:

| Source | What it is |
|---|---|
| `/odom_raw pose` | The firmware's own integrated position. **Suggested factors use this.** |
| `/odom_raw speed x time` | Its wheel speeds integrated over message arrival time. For comparison only: Wi-Fi jitter makes this under-count by about 2%. |
| `/imu yaw` | The firmware's IMU heading. The EKF fuses this as its heading. |
| `/imu gyro z` | Raw gyro rate, integrated. Not fused. |
| `/odom EKF` | The final estimate. After calibration this should be within about 0.5% of the lidar. |

`./record run` saves each run as a dataset in `datasets/raw/`, with the printout in
`output.txt`.

## Check distance with a tape measure

The lidar is not a perfect ruler. Ours over-reads range differences by about 3%: it is
nearly exact at 0.66 m and 2.6-2.8 cm long at 1.65 m. A distance factor calibrated against
it inherits that error, so measure distance against a tape:

1. Lay a tape measure along the path with its zero at the wall, and put the front edge of
   the chassis on a mark (we used 60 in).
2. Drive out and hold there: `./record run tape-out ./ros python3 /work/tools/calibrate.py out 1.0`
3. Read the tape at the front edge. The true distance is the difference between the two
   readings. The factor is that distance divided by the `/odom_raw pose` value the tool printed.
4. Return: `./ros python3 /work/tools/calibrate.py back 1.0`

Rotation doesn't have this problem: aligning scans measures angles, which don't depend on
range scale.

## Results on our robot

On a hardwood floor (2026-11-11):

| | Error before | After calibration |
|---|---|---|
| Wheels, distance | 1.9% over (tape) | |
| Wheels, turning | 0.3-0.4% over | |
| IMU yaw | 0.1% under | |
| EKF distance, 1 m | | 0.0% against the tape (1.019 m driven, 1.019 m measured) |
| EKF heading, 720 degrees | | 0.00% / +0.03% against the lidar |

Yahboom's own calibration file suggests a linear factor of 1.26, which would make distances
about 28% too long here. Measure your own robot rather than using those values.

An earlier quick check (45-degree turns, a single scan pair, no sub-beam refinement,
robot body not filtered out) suggested odometry under-reported rotation by 6-12%. The
method above doesn't reproduce that, and the cause wasn't pinned down, so treat short-turn
checks with caution. `calibrate.py` ignores returns within 0.2 m, because returns from the
robot's own body stay at the same beam angles however it turns.

## Wall target: sideways drift and heading

The tape tracker only sees how far along the tape the robot is. To see sideways drift and
heading, tape a printed target to the wall at the end of the tape:

1. Print `print/wall_target.svg` at 100% and check the 100 mm bar with a ruler. Stick it to
   stiff card so it stays flat.
2. Tape it to the wall with the red cross on the tape line, at the height of the camera lens,
   square to the wall.
3. Aim the camera level at it (`./ros python3 /work/tools/ptz.py <pan> <tilt>`) and check:
   `./ros python3 /work/tools/wall_target.py snap`. It prints where the board center sits in
   the image; adjust the tilt until it is near 320,240 and the board is fully in view at both
   ends of the tape. The frame is saved to `datasets/wall/`.
4. Run the repeatability test with `--wall`:
   `./record run linear-repeat-lf ./ros python3 /work/tools/repeat_linear.py 1.0 --wall`

At each stop it combines two measurements:

- **Camera:** distance to the board and the board's bearing in the image (OpenCV solvePnP,
  Yahboom's factory intrinsics from `ost.yaml`, or `config/camera_intrinsics.yaml` if you
  add your own).
- **Lidar:** a straight-line fit to the wall ahead, which gives the robot's heading relative
  to the wall.

Heading comes from the lidar because the camera can't measure it on its own. The camera's
field of view is narrow (about 35 degrees), and with a flat target a 5 mm sideways shift
looks almost exactly like a 0.5 degree turn; `tests/test_wall_target.py` shows this. The
sideways offset is the camera distance times the tangent of (lidar heading + camera
bearing). That includes the pan servo's fixed offset from the robot axis, so compare
offsets between stops at the same end of the tape. In simulation the lidar heading is good
to about 0.2-0.4 degrees per reading at 0.6 m from the wall and under 0.15 degrees at 1.6 m;
the real figure depends on the lidar's noise.

## Stopping accuracy

After a stop command the robot keeps moving for a moment (Wi-Fi latency and motor
run-down): about 18 mm after a 1 m drive and 3-4.6 degrees after a turn. `move.py` stops
early by `LINEAR_COAST` and `ANGULAR_COAST` to compensate. With those, a 1 m drive lands
within about 5 mm and a full turn within about 1.5 degrees.

## Restart safety

Right after the micro-ROS agent restarts, the robot can send a message stamped with its
unsynced clock (decades off). The EKF integrated one of those into a jump of hundreds of
meters. `odom_calibrator.py` drops any message stamped more than 0.5 s from the host clock.

## Battery

`./up` starts `tools/battery_monitor.py`. It reads the pack voltage the firmware sends on
`/battery`, publishes `/battery_state` (sensor_msgs/BatteryState, with percent and amp-hours
left) and logs to `datasets/battery/battery_YYYYMMDD.csv` (files are named by UTC date). The
voltage drops while the motors pull current, so the percent only updates once the robot has
been still for 20 s.

The voltage-to-percent curve in `config/battery.yaml` comes from one full idle discharge:

1. Charge the pack until the charger light turns green, then switch the robot on and run `./up`.
2. Leave the robot still (no gamepad, no drive commands) until it beeps at 10.2 V, then
   switch it off.
3. Fit the curve: `.venv/bin/python tools/battery_report.py datasets/battery/battery_*.csv`
   and paste the `ocv_curve` block it prints into `config/battery.yaml`.

Our 3S 2200 mAh pack (2026-11-11):

| Remaining | Resting V | V/cell |
|---|---|---|
| 100% | 12.47 | 4.16 |
| 80% | 12.03 | 4.01 |
| 60% | 11.90 | 3.97 |
| 50% | 11.63 | 3.88 |
| 40% | 11.50 | 3.83 |
| 20% | 11.25 | 3.75 |
| 10% | 10.83 | 3.61 |
| 0% (alarm) | 10.19 | 3.40 |

- Idle runtime from full to the alarm was 7.1 hours, an average draw of about 0.29 A
  (3.2 W) with the lidar and camera running.
- The last 10% goes in about 40 minutes. Below 11.0 V, end the session.
- Straight off the charger, the voltage drops about 0.3 V in the first 20 minutes as the
  charger's surface charge drains. That doesn't mean the pack is losing capacity.
- Driving draws much more than idle; see the driving test below.

### Driving test

`tools/battery_drive_test.py` measures runtime while driving. Put the robot on a clear
straight run (the 1 m tape is fine), charge the pack fully, unplug the gamepad receiver, then:

```bash
./ros python3 /work/tools/battery_drive_test.py run            # --dist 1.0 --rest-every 600
```

It drives 1 m out and back along the same line, over and over. Every 10 minutes it rests for
60 s, reads the resting voltage against the fitted curve, and compares it with the voltage
while driving just before (the sag under load). It stops by itself when the resting voltage
reaches 10.6 V or after 5 hours. If the lidar safety check stops a leg (a person or a pet in
the way), it waits up to 2 minutes for 0.5 m of clearance and carries on.

Each time it gets back to the start it matches a lidar scan against one taken there at the
beginning (`tools/scan_anchor.py`) and resets its line, so odometry drift doesn't build up
over hundreds of cycles. It stops after 3 failed matches in a row. Use `--no-scan-match` to
steer by odometry alone. The log line for each cycle shows where the match put the robot.

Every voltage sample goes to `datasets/battery/drive_<time>.csv`. At the end, or for a run
that was stopped early, it prints the driving runtime, average current, distance per charge
and voltage sag:

```bash
./ros python3 /work/tools/battery_drive_test.py report /root/datasets/battery/drive_<time>.csv
```

Current comes from charge used (the idle curve) over time, since the robot has no current
sensor. The pack resistance estimate assumes the 0.29 A idle draw measured above, so treat it
as rough.

Results on our robot (2026-11-11, two legs, with a 3 minute pause to oil the caster):

| Leg | Charge | Time (driving) | Distance | Average draw | Projected from full |
|-----|--------|----------------|----------|--------------|---------------------|
| 1 | 80% -> 49% | 1.49 h (1.36 h) | 606 m | 0.46 A | 4.8 h, 1.96 km |
| 2, caster oiled | 45% -> 11% | 1.86 h (1.70 h) | 765 m | 0.40 A | 5.5 h, 2.28 km |

So on flat ground, expect roughly 2 km and 5 hours per charge with this stop-start driving,
against 7 hours sitting still. Voltage sag under load was only 0.03 to 0.06 V (pack
resistance about 0.35 ohm). A dry caster cost about 15% of the current. Leg 2 was stopped at
11% by hand; the last load reading was 10.60 V.

Return to the start over 430 scan-matched cycles: -6.1 mm on average, spread 2.9 mm, range
-17.7 to +5.4 mm, heading within 1.3 deg, with no growth over the run. Odometry alone drifts
about 4 mm and 0.07 deg per cycle. One match (cycle 102) jumped 84 mm and 20 deg and the
next cycle corrected it. `relocate()` now rejects a match more than 5 cm or 3 deg from
odometry's guess and keeps the previous line instead.
