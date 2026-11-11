# 4. First drive

For your first runs, put the robot on a stand or a box so the wheels can spin freely.

## Start the stack

```bash
./up          # agent, robot model, EKF, camera, Foxglove bridge
./up slam     # the same plus slam_toolbox mapping
./down        # stop everything, including the relay
```

`./up` also restarts the ROS processes if they're already running, so you can run it
again to reset. Logs:

- `logs/bridge.log` on the Mac, for the relay
- `/tmp/base.log`, `/tmp/slam.log`, `/tmp/camera.log` and `/tmp/foxglove.log` in the container
  (`./ros cat /tmp/base.log`)

`./ros <command>` runs any command inside the container with ROS sourced.

## Foxglove

Install [Foxglove](https://foxglove.dev/download), choose **Open connection >
Foxglove WebSocket**, and enter `ws://localhost:8765`. Useful panels:

- **3D:** set the fixed frame to `odom` (or `map` when SLAM is running), and enable
  `/scan`, `/map` and the robot model.
- **Image:** `/camera/image_raw`
- **Plot:** `/odom.twist.twist.linear.x`, `/imu.angular_velocity.z`

## Teleop from the keyboard

```bash
docker exec -it lidarbot /entrypoint.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

(`./ros` doesn't allocate a terminal, so use `docker exec -it` for anything
interactive.) Start slowly: press `z` a few times to lower the speed.

## Scripted moves

`tools/move.py` makes closed-loop moves using the fused odometry (`/odom`), and stops if
the lidar sees something in the robot's path:

```bash
./ros python3 /work/tools/move.py forward 0.5    # meters; negative = backward
./ros python3 /work/tools/move.py rotate 90      # degrees; positive = left (counter-clockwise)
```

Before driving, it checks a robot-width corridor ahead and refuses the move if there's
less than the distance plus 0.35 m of clearance. A heading where many lidar beams return
nothing (mirrors, glossy doors) counts as blocked at 0.6 m.

`tools/drive_test.py` runs a short sequence of speeds and prints the commanded and
measured values. It's a good check that the motors and encoders are working.
