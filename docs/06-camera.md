# 6. Camera and pan/tilt

The camera is a separate ESP32-S3 module (board name `ESP-S3-YAHBOOM`, 2MP GC2145 sensor)
with its own IP address. It serves MJPEG at `http://<camera_ip>:81/stream`: 320x240 at
about 6 fps on our Wi-Fi (measured 2026-11-13). You can open that URL in a browser to check
it; `http://<camera_ip>/status` shows its settings. Yahboom's firmware accepts resolution
and clock changes through `/control` but keeps sending 320x240, and rejects quality, white
balance, exposure, gain and brightness (HTTP 500): the sensor runs on its own automatic
settings.

`./up` starts Yahboom's `camera_driver`, which keeps the stream open and publishes
`/camera/image_raw`, then points the camera at the floor just ahead of the robot. The driver
upscales every frame to 640x480, so with Yahboom's firmware the ROS images have no more detail
than 320x240.

## Our own camera firmware

`firmware/lidarbot_cam/` is an open replacement for Yahboom's camera firmware with the same
endpoints, so `./up` and `camera_driver` work unchanged. It streams real 640x480 at about
5 fps (18 fps at 320x240), runs no hotspot, and turns Wi-Fi power save off. Our robot runs
it since 2026-11-13. See its README for build, flashing (from a Linux machine), the backup
and how to restore Yahboom's firmware.

By default the module also runs its own open Wi-Fi hotspot (`Yahboom_ESP32_WIFI`, no
password, 192.168.4.1) alongside your network, and anyone in range can view the camera
through it. The hotspot also shares the radio: with it on, pings to the camera took
0.3-1.3 s; with it off, about 18 ms on average.

To turn it off, connect the module's USB-C port to a Linux computer (its USB chip is a CH340
variant, 1a86:7522, that macOS has no built-in driver for; we used the Pi Zero) and send
`wifi_mode:1` at 115200 baud with no line ending. Ours restarted instead of replying `OK`;
afterwards `ap_ip` answers `ap_ip:null`. `wifi_ver` shows the firmware version (ours:
`AI_V1.5.2`). Opening the serial port restarts the module.

## Stale frames

The camera's `/capture` endpoint, and short reads of the stream, often return an old
frame from its buffer. When you're using the camera to decide where to drive, that's
misleading. Keep the stream open (as `camera_driver` does) and use frames that arrive
after you ask:

```bash
./ros python3 /work/tools/look.py        # saves maps/camera.jpg, prints pan/tilt
```

## Pan/tilt (Superior kit)

```bash
./ros python3 /work/tools/ptz.py 0 -53    # pan, tilt in degrees
```

| | Command range | |
|---|---|---|
| Pan | -90 to 90 | 0 = straight ahead |
| Tilt | -90 to 15 | positive = up; on our robot -40 is level (0.87 real deg per commanded deg), the power-on -45 is 4 deg down, and `./up` aims at -53, about 11 deg down |

The command maps 1:1 onto real degrees, but where 0 points depends on how the servo horn
sits on its spline. On our robot it had slipped so far that level needed -61 and the camera
could no longer look down at the floor; re-seating the horn fixed it. Measure yours:

```bash
./ros python3 /work/tools/tilt_check.py    # face a wall 1-2 m away with bare floor in front
```

It sweeps the tilt, finds where the floor meets the wall in each frame and, with the lidar's
distance to the wall, works out how far below level the camera points. It prints the
command that is level. It only measures angles where the floor line is in view (about 15 deg
either side of level); further down, ours turned out steeper than 1:1 (-15 pointed about
35 deg down, not 21). Before the horn was re-seated the tilt stalled on a stop near -80 and
the stalled servo dropped the camera off Wi-Fi.

### Finding the real range without limit switches

The servos have no position feedback: `/ptz_angle` only echoes the command. So the camera is
the sensor:

```bash
./ros python3 /work/tools/ptz_range.py --axis both    # needs a textured view, not a blank wall
```

It steps a servo outward and measures how far the image moves per step (8-9 px per degree at
320x240 on ours). A step that moves it much less means the servo stopped; it checks against the
frame two steps back, because repeating texture (tiles, cabinet doors) can fake a stop, and
backs off immediately. Blank walls and ceilings still fool it, so treat its ends as "at least
here" unless the match quality is high.

Results on ours (2026-11-13, horn re-seated): tilt moves freely from -90 to at least +15, pan
-90 to +90 (the right end is the servo's own end). 1 deg steps found a 1-2 deg deadband and a
sticky zone on the pan servo between about -34 and -41: in both directions it lags about 5 deg
and then jumps through. It turns freely by hand with the power off, so it's most likely a
worn spot on the servo's position sensor; aims in that zone can be about 5 deg off until the
servo is replaced.

It publishes `/cmd_ptz_angle` (a `geometry_msgs/Vector3`, with x = pan and y = tilt) until
`/ptz_angle` confirms the new position. Like `/cmd_vel`, this is ignored while the gamepad
receiver is plugged in.
