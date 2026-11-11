# 8. Recording datasets

`./record` saves what the robot publishes as an MCAP rosbag, along with metadata:

```bash
./record start living-room-drive "teleop around the living room"
./record stop
./record run calib-linear ./ros python3 /work/tools/calibrate.py linear 1.0   # record while a command runs
./record list
```

Each recording is a folder in `datasets/raw/<UTC time>_<label>/`:

| File | Contents |
|---|---|
| `bag/` | MCAP rosbag. Open it in Foxglove, or with `ros2 bag info`. |
| `metadata.json` | Label, task description, start time, kit type, git commit, the calibration in use |
| `output.txt` | Output of the command, for `./record run` |
| `record.log` | rosbag2's log |

Topics recorded: `/scan`, `/odom`, `/odom_raw`, `/odom_cal`, `/imu`, `/imu_cal`, `/cmd_vel`,
`/tf`, `/tf_static`, `/joint_states`, `/robot_description`, `/ptz_angle`,
`/camera/camera_info`, `/camera/image_raw/compressed` (JPEG), `/map` and `/map_metadata`.
A minute of driving with the camera is roughly 10 MB.

## Privacy

Recordings made at home contain camera images of your rooms (and anyone in them), and
maps that are floor plans. `datasets/` is git-ignored. Review anything before you share it.

## Export to Hugging Face (LeRobot format)

`tools/export_lerobot.py` turns recordings into a [LeRobot](https://github.com/huggingface/lerobot)
dataset, Hugging Face's standard format for robot data. It runs on the Mac in its own environment:

```bash
uv venv --python 3.12 .venv-hf
uv pip install --python .venv-hf 'lerobot[dataset]==0.7.0' mcap-ros2-support==0.6.0

.venv-hf/bin/python tools/export_lerobot.py --repo-id <user>/<name> \
    --out datasets/lerobot/<name> datasets/raw/<run> [datasets/raw/<run> ...]
```

Each recording becomes one episode at 10 Hz: EKF state, raw wheel odometry, IMU, lidar
ranges and the commanded velocity. Camera frames are left out unless you pass `--camera`.
A `ground_truth.json` next to a recording (for example tape readings) is carried into
`meta/lidarbot_episodes.json`.

Nothing is uploaded until you run, after reviewing the folder and writing its `README.md`:

```bash
huggingface-cli login     # once, in your own terminal
.venv-hf/bin/python tools/export_lerobot.py --push datasets/lerobot/<name> --repo-id <user>/<name>
```
