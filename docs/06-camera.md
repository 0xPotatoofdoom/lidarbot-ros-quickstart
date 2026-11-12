# 6. Camera and pan/tilt

The camera is a separate ESP32 with its own IP address. It serves MJPEG at
`http://<camera_ip>:81/stream` (640x480, about 2-3 fps over Wi-Fi). You can open that
URL in a browser to check it.

`./up` starts Yahboom's `camera_driver`, which keeps the stream open and publishes
`/camera/image_raw`, then points the camera at the floor just ahead of the robot.

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
./ros python3 /work/tools/ptz.py 0 -5     # pan, tilt in degrees
```

| | Command range | |
|---|---|---|
| Pan | -90 to 90 | 0 = straight ahead |
| Tilt | -80 to 20 | positive = up; on our robot +6 is level and -5 shows the floor from about 0.3 m ahead to the far wall |

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
35 deg down, not 21). Don't command the tilt past its mechanical stop (about -80 on ours):
the stalled servo can drop the camera off Wi-Fi.

It publishes `/cmd_ptz_angle` (a `geometry_msgs/Vector3`, with x = pan and y = tilt) until
`/ptz_angle` confirms the new position. Like `/cmd_vel`, this is ignored while the gamepad
receiver is plugged in.
