#!/usr/bin/env python3
"""Convert ./record recordings into a LeRobot dataset (Hugging Face format).

Runs on the Mac in the .venv-hf environment (no ROS needed):
    .venv-hf/bin/python tools/export_lerobot.py --repo-id potatoofdoom/NAME \
        --out datasets/lerobot/NAME  datasets/raw/<run> [datasets/raw/<run> ...]

Each recording becomes one episode, resampled to --fps on the recorder's clock
(message receive time) by holding the latest value of each topic. Camera frames
are left out unless --camera is given: they show the inside of your home, so
review them before including them.

Nothing is uploaded. Review the output, then push with:
    .venv-hf/bin/python tools/export_lerobot.py --push datasets/lerobot/NAME --repo-id ...
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

ROBOT_TYPE = "yahboom_microros_v2"
POSE_NAMES = ["x", "y", "yaw", "linear_x", "angular_z"]
IMU_NAMES = ["yaw", "gyro_z", "accel_x", "accel_y", "accel_z"]
ACTION_NAMES = ["linear_x", "angular_z"]
TASKS = {  # label prefix -> task description
    "calib-angular": "Rotate in place two full turns counter-clockwise, then two clockwise (odometry calibration).",
    "calib-verify-angular": "Rotate in place two full turns counter-clockwise, then two clockwise (calibration check).",
    "stop-accuracy-angular": "Rotate in place one full turn counter-clockwise, then one clockwise (stopping accuracy).",
    "calib-linear": "Drive 1 m straight toward a wall, then 1 m back (odometry calibration).",
    "calib-verify-linear": "Drive 1 m straight toward a wall, then 1 m back (calibration check).",
    "stop-accuracy-linear": "Drive 1 m straight toward a wall, then 1 m back (stopping accuracy).",
    "tape-out": "Drive 1 m straight toward a wall along a tape measure (tape ground truth).",
    "tape-verify-out": "Drive 1 m straight toward a wall along a tape measure (tape ground truth).",
    "tape-back": "Reverse 1 m straight away from a wall along a tape measure (tape ground truth).",
    "tape-verify-back": "Reverse 1 m straight away from a wall along a tape measure.",
}


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


@dataclass
class Series:
    """Timestamped samples of one topic (receive time, seconds)."""
    t: list[float] = field(default_factory=list)
    v: list[np.ndarray] = field(default_factory=list)

    def at(self, times: np.ndarray) -> np.ndarray:
        """Latest sample at or before each time (zero-order hold)."""
        t = np.asarray(self.t)
        idx = np.searchsorted(t, times, side="right") - 1
        if (idx < 0).any():
            raise ValueError("requested time before first sample")
        return np.stack(self.v)[idx]


def task_for(label: str) -> str:
    for prefix in sorted(TASKS, key=len, reverse=True):
        if label.startswith(prefix):
            return TASKS[prefix]
    raise SystemExit(f"no task description for label {label!r}; add it to TASKS")


def read_recording(bag_dir: Path, camera: bool) -> tuple[dict[str, Series], dict]:
    mcaps = sorted(bag_dir.glob("*.mcap"))
    if not mcaps:
        raise SystemExit(f"no .mcap in {bag_dir}")
    s: dict[str, Series] = {k: Series() for k in ("odom", "odom_raw", "imu", "scan", "cmd_vel", "image")}
    scan_info: dict = {}
    topics = ["/odom", "/odom_raw", "/imu", "/scan", "/cmd_vel"] + (["/camera/image_raw/compressed"] if camera else [])
    for path in mcaps:
        with path.open("rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            for _, channel, message, m in reader.iter_decoded_messages(topics=topics):
                t = message.log_time * 1e-9
                topic = channel.topic
                if topic in ("/odom", "/odom_raw"):
                    p, tw = m.pose.pose.position, m.twist.twist
                    vec = np.array([p.x, p.y, yaw_of(m.pose.pose.orientation), tw.linear.x, tw.angular.z])
                    key = "odom" if topic == "/odom" else "odom_raw"
                elif topic == "/imu":
                    a = m.linear_acceleration
                    vec = np.array([yaw_of(m.orientation), m.angular_velocity.z, a.x, a.y, a.z])
                    key = "imu"
                elif topic == "/scan":
                    r = np.asarray(m.ranges, dtype=np.float64)
                    r[~np.isfinite(r)] = 0.0  # 0 = no return, as the firmware reports it
                    vec = r
                    key = "scan"
                    if not scan_info:
                        scan_info = {"angle_min": m.angle_min, "angle_increment": m.angle_increment,
                                     "range_min": m.range_min, "range_max": m.range_max, "beams": len(r),
                                     "frame_id": m.header.frame_id}
                elif topic == "/cmd_vel":
                    vec = np.array([m.linear.x, m.angular.z])
                    key = "cmd_vel"
                else:
                    import cv2  # only needed with --camera
                    img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
                    vec = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    key = "image"
                s[key].t.append(t)
                s[key].v.append(vec)
    return s, scan_info


def frame_times(s: dict[str, Series], fps: int, camera: bool) -> np.ndarray:
    needed = ["odom", "odom_raw", "imu", "scan"] + (["image"] if camera else [])
    for k in needed:
        if not s[k].t:
            raise SystemExit(f"recording has no {k} messages")
    start = max(s[k].t[0] for k in needed)
    end = min(s[k].t[-1] for k in needed)
    return np.arange(start, end, 1.0 / fps)


def features(beams: int, camera: bool) -> dict:
    f = {
        "observation.state": {"dtype": "float32", "shape": (5,), "names": POSE_NAMES},
        "observation.odom_raw": {"dtype": "float32", "shape": (5,), "names": POSE_NAMES},
        "observation.imu": {"dtype": "float32", "shape": (5,), "names": IMU_NAMES},
        "observation.scan": {"dtype": "float32", "shape": (beams,), "names": None},
        "action": {"dtype": "float32", "shape": (2,), "names": ACTION_NAMES},
    }
    if camera:
        f["observation.images.front"] = {"dtype": "video", "shape": (480, 640, 3),
                                         "names": ["height", "width", "channels"]}
    return f


def export(args: argparse.Namespace) -> None:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    out = Path(args.out)
    if out.exists():
        if not args.overwrite:
            raise SystemExit(f"{out} exists; pass --overwrite to replace it")
        shutil.rmtree(out)
    runs = [Path(r) for r in args.recordings]
    ds = None
    episodes = []
    for i, run in enumerate(runs):
        meta = json.loads((run / "metadata.json").read_text())
        series, scan_info = read_recording(run / "bag", args.camera)
        times = frame_times(series, args.fps, args.camera)
        if ds is None:
            ds = LeRobotDataset.create(repo_id=args.repo_id, fps=args.fps, root=out, robot_type=ROBOT_TYPE,
                                       features=features(scan_info["beams"], args.camera),
                                       use_videos=args.camera)
        cols = {k: series[k].at(times).astype(np.float32) for k in ("odom", "odom_raw", "imu", "scan")}
        # cmd_vel is only published while a tool drives; before the first command it is zero.
        if series["cmd_vel"].t:
            first = series["cmd_vel"].t[0]
            act = np.zeros((len(times), 2), np.float32)
            mask = times >= first
            act[mask] = series["cmd_vel"].at(times[mask])
        else:
            act = np.zeros((len(times), 2), np.float32)
        images = series["image"].at(times) if args.camera else None
        task = meta.get("task") or ""
        task = task if task and not task.startswith(("./ros", "sleep")) else task_for(meta["label"])
        for j in range(len(times)):
            frame = {"observation.state": cols["odom"][j], "observation.odom_raw": cols["odom_raw"][j],
                     "observation.imu": cols["imu"][j], "observation.scan": cols["scan"][j],
                     "action": act[j], "task": task}
            if images is not None:
                frame["observation.images.front"] = images[j]
            ds.add_frame(frame)
        ds.save_episode()
        gt_path = run / "ground_truth.json"
        episodes.append({"episode_index": i, "recording": run.name, "label": meta["label"], "task": task,
                         "frames": len(times), "duration_s": round(float(times[-1] - times[0]), 2),
                         "calibration_in_use": meta.get("calibration"),
                         "ground_truth": json.loads(gt_path.read_text()) if gt_path.exists() else None})
        print(f"episode {i}: {run.name}  {len(times)} frames  {task}")
    assert ds is not None
    ds.finalize()
    (out / "meta" / "lidarbot_episodes.json").write_text(json.dumps(episodes, indent=2) + "\n")
    (out / "meta" / "lidarbot_scan.json").write_text(json.dumps(scan_info, indent=2) + "\n")
    print(f"\nwrote {out}  ({sum(e['frames'] for e in episodes)} frames, {len(episodes)} episodes)")
    print("review it, then push with --push")


def push(args: argparse.Namespace) -> None:
    from huggingface_hub import HfApi

    root = Path(args.push)
    if not (root / "meta" / "info.json").exists():
        raise SystemExit(f"{root} is not a LeRobot dataset")
    api = HfApi()
    api.create_repo(args.repo_id, repo_type="dataset", exist_ok=True, private=args.private)
    api.upload_folder(repo_id=args.repo_id, repo_type="dataset", folder_path=str(root),
                      commit_message="Upload lidarbot dataset")
    print(f"pushed https://huggingface.co/datasets/{args.repo_id}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recordings", nargs="*", help="datasets/raw/<run> folders")
    ap.add_argument("--repo-id", required=True, help="e.g. potatoofdoom/lidarbot-odometry-calibration")
    ap.add_argument("--out", help="output folder, e.g. datasets/lerobot/<name>")
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--camera", action="store_true", help="include robot camera frames (review them first)")
    ap.add_argument("--overwrite", action="store_true")
    ap.add_argument("--push", metavar="DATASET_DIR", help="upload a reviewed dataset folder instead of exporting")
    ap.add_argument("--private", action="store_true", help="create the Hub repo as private")
    args = ap.parse_args()
    if args.push:
        push(args)
    elif args.recordings and args.out:
        export(args)
    else:
        ap.error("give recordings and --out, or --push DATASET_DIR")


if __name__ == "__main__":
    sys.exit(main())
