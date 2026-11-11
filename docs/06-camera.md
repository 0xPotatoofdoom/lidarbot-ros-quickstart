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
./ros python3 /work/tools/ptz.py 0 -62    # pan, tilt in degrees
```

| | Range | |
|---|---|---|
| Pan | -90 to 90 | 0 = straight ahead |
| Tilt | -90 to 20 | positive = up; -62 shows the floor just ahead |

It publishes `/cmd_ptz_angle` (a `geometry_msgs/Vector3`, with x = pan and y = tilt) until
`/ptz_angle` confirms the new position. Like `/cmd_vel`, this is ignored while the gamepad
receiver is plugged in.
