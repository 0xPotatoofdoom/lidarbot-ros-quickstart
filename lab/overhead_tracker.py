#!/usr/bin/env python3
"""Overhead ground-truth tracker: robot pose on the floor from a phone camera.

Runs on the lab Pi (plain Python + OpenCV, no ROS). A phone running an IP camera
app (e.g. "IP Webcam") looks straight down at the floor. Four ArUco markers lie
flat on the floor at measured positions; one more is taped on top of the robot.

    python3 overhead_tracker.py calibrate lab.json     # floor markers -> lab.json "homography"
    python3 overhead_tracker.py track lab.json         # log poses, serve them over HTTP

lab.json (see lab.example.json):
    camera_url        e.g. http://192.168.1.50:8080   (IP Webcam: /shot.jpg is fetched)
    camera_height_m   lens height above the floor, measured with a tape
    robot_marker      {"id": 0, "size_m": 0.060, "height_m": 0.17, "yaw_offset_deg": 0}
    floor_markers     {"1": [x_m, y_m], ...}   marker centers in the floor frame; lay them
                      square to the x/y axes (any of the four ways round)
    floor_marker_size_m
    log_dir, http_port

Floor frame: x along the test path (e.g. along the tape measure), y to its left,
in meters. Poses are logged as CSV: unix_time,x,y,yaw,quality.
HTTP: GET /pose -> latest pose as JSON; GET /log?since=<unix_time> -> CSV rows.
"""
from __future__ import annotations

import json
import math
import sys
import threading
import time
import urllib.request
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

DICT = cv2.aruco.DICT_4X4_50
ROI_HALF = 160          # px; search window around the last detection
CALIB_FRAMES = 10
FETCH_TIMEOUT = 5.0


@dataclass(frozen=True)
class Pose:
    t: float
    x: float
    y: float
    yaw: float
    quality: float  # marker side length in pixels (bigger = more reliable)

    def csv(self) -> str:
        return f"{self.t:.3f},{self.x:.4f},{self.y:.4f},{self.yaw:.4f},{self.quality:.1f}"


def load_config(path: str) -> dict:
    cfg = json.loads(Path(path).read_text())
    for key in ("camera_url", "camera_height_m", "robot_marker", "floor_markers", "floor_marker_size_m"):
        if key not in cfg:
            raise SystemExit(f"{path}: missing {key!r}")
    if cfg["robot_marker"]["height_m"] >= cfg["camera_height_m"]:
        raise SystemExit("robot marker height must be below the camera height")
    return cfg


def fetch(url: str) -> tuple[float, np.ndarray]:
    """One fresh frame, timestamped at the middle of the request."""
    t0 = time.time()
    with urllib.request.urlopen(url.rstrip("/") + "/shot.jpg", timeout=FETCH_TIMEOUT) as r:
        data = r.read()
    t1 = time.time()
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise RuntimeError("camera returned something that isn't a JPEG")
    return (t0 + t1) / 2, img


def detector() -> cv2.aruco.ArucoDetector:
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(DICT), params)


def floor_corners(center: list[float], size: float) -> np.ndarray:
    """Corners of an axis-aligned square marker in floor coordinates (one ring order)."""
    cx, cy, h = center[0], center[1], size / 2
    return np.array([[cx + h, cy + h], [cx + h, cy - h], [cx - h, cy - h], [cx - h, cy + h]], np.float64)


def match_rotation(img_c: np.ndarray, floor_c: np.ndarray, h0: np.ndarray) -> np.ndarray:
    """Reorder floor corners to match the detected corners, whatever way the marker was laid.

    Markers only need to be square to the x/y axes, not facing a particular way.
    h0 is a rough image->floor homography from the marker centers.
    """
    projected = cv2.perspectiveTransform(img_c[None].astype(np.float64), h0)[0]
    best = min(range(4), key=lambda k: float(np.linalg.norm(np.roll(floor_c, k, axis=0) - projected)))
    flipped = floor_c[::-1]
    best_f = min(range(4), key=lambda k: float(np.linalg.norm(np.roll(flipped, k, axis=0) - projected)))
    a, b = np.roll(floor_c, best, axis=0), np.roll(flipped, best_f, axis=0)
    return a if np.linalg.norm(a - projected) <= np.linalg.norm(b - projected) else b


def solve_floor(cfg: dict, images: list[np.ndarray]) -> dict:
    """Image->floor homography from the floor markers seen in a few frames."""
    det = detector()
    img_pts: dict[int, list[np.ndarray]] = {}
    for img in images:
        corners, ids, _ = det.detectMarkers(img)
        for c, i in zip(corners, [] if ids is None else ids.ravel(), strict=True):
            img_pts.setdefault(int(i), []).append(c.reshape(4, 2))
    want = {int(k): v for k, v in cfg["floor_markers"].items()}
    missing = sorted(set(want) - set(img_pts))
    if missing:
        raise SystemExit(f"floor markers not visible: {missing} (seen: {sorted(img_pts)})")
    img_c = {mid: np.mean(img_pts[mid], axis=0) for mid in want}
    centers_img = np.array([img_c[m].mean(axis=0) for m in want])
    centers_floor = np.array([want[m] for m in want], np.float64)
    h0, _ = cv2.findHomography(centers_img, centers_floor)
    if h0 is None:
        raise SystemExit("floor markers are collinear or too close together; spread them out")
    src = [img_c[m] for m in want]
    dst = [match_rotation(img_c[m], floor_corners(want[m], cfg["floor_marker_size_m"]), h0) for m in want]
    src_a, dst_a = np.concatenate(src), np.concatenate(dst)
    h, _ = cv2.findHomography(src_a, dst_a)
    err = np.linalg.norm(cv2.perspectiveTransform(src_a[None], h)[0] - dst_a, axis=1)
    # Nadir: the floor point straight below the lens. For a camera looking
    # (nearly) straight down this is where the image center lands.
    hh, ww = images[0].shape[:2]
    nadir = cv2.perspectiveTransform(np.array([[[ww / 2, hh / 2]]], np.float64), h)[0, 0]
    return {"homography": h.tolist(), "nadir_m": nadir.tolist(),
            "calibration_rms_mm": round(float(np.sqrt(np.mean(err ** 2))) * 1000, 1),
            "calibration_max_mm": round(float(err.max()) * 1000, 1)}


def calibrate(cfg_path: str) -> None:
    cfg = load_config(cfg_path)
    frames = [fetch(cfg["camera_url"])[1] for _ in range(CALIB_FRAMES)]
    result = solve_floor(cfg, frames)
    cfg.update(result, calibrated_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()))
    Path(cfg_path).write_text(json.dumps(cfg, indent=2) + "\n")
    print(f"calibrated from {len(cfg['floor_markers'])} markers: corner rms {result['calibration_rms_mm']} mm, "
          f"max {result['calibration_max_mm']} mm, nadir {np.round(result['nadir_m'], 3).tolist()} m")


class Tracker:
    def __init__(self, cfg: dict) -> None:
        if "homography" not in cfg:
            raise SystemExit("not calibrated: run `calibrate` first")
        self.cfg = cfg
        self.h = np.array(cfg["homography"])
        self.nadir = np.array(cfg["nadir_m"])
        rm = cfg["robot_marker"]
        self.marker_id = rm["id"]
        # A point at height z projects onto the floor further from the nadir than
        # it really is, by camera_height / (camera_height - z). Undo that.
        self.parallax = (cfg["camera_height_m"] - rm["height_m"]) / cfg["camera_height_m"]
        self.yaw_offset = math.radians(rm.get("yaw_offset_deg", 0.0))
        self.det = detector()
        self.last_px: np.ndarray | None = None
        self.latest: Pose | None = None
        self.lock = threading.Lock()
        log_dir = Path(cfg.get("log_dir", "~/lab/logs")).expanduser()
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = log_dir / f"poses_{time.strftime('%Y%m%d', time.gmtime())}.csv"
        if not self.log_path.exists():
            self.log_path.write_text("unix_time,x,y,yaw,quality\n")

    def _to_floor(self, px: np.ndarray) -> np.ndarray:
        pts = cv2.perspectiveTransform(px.reshape(-1, 1, 2).astype(np.float64), self.h).reshape(-1, 2)
        return self.nadir + (pts - self.nadir) * self.parallax

    def _find(self, img: np.ndarray) -> np.ndarray | None:
        """Robot marker corners in image pixels, searching near the last fix first."""
        if self.last_px is not None:
            cx, cy = self.last_px.mean(axis=0).astype(int)
            x0, y0 = max(cx - ROI_HALF, 0), max(cy - ROI_HALF, 0)
            roi = img[y0:cy + ROI_HALF, x0:cx + ROI_HALF]
            found = self._detect(roi)
            if found is not None:
                return found + np.array([x0, y0])
        return self._detect(img)

    def _detect(self, img: np.ndarray) -> np.ndarray | None:
        corners, ids, _ = self.det.detectMarkers(img)
        if ids is None:
            return None
        for c, i in zip(corners, ids.ravel(), strict=True):
            if int(i) == self.marker_id:
                return c.reshape(4, 2)
        return None

    def step(self) -> Pose | None:
        t, img = fetch(self.cfg["camera_url"])
        pose = self.locate(t, img)
        if pose is not None:
            with self.log_path.open("a") as f:
                f.write(pose.csv() + "\n")
        return pose

    def locate(self, t: float, img: np.ndarray) -> Pose | None:
        """Robot pose on the floor from one frame (None if the marker isn't seen)."""
        px = self._find(img)
        self.last_px = px
        if px is None:
            return None
        floor = self._to_floor(px)
        center = floor.mean(axis=0)
        front = (floor[0] + floor[1]) / 2  # midpoint of the marker's top edge
        yaw = math.atan2(front[1] - center[1], front[0] - center[0]) + self.yaw_offset
        side = float(np.mean(np.linalg.norm(px - np.roll(px, 1, axis=0), axis=1)))
        pose = Pose(t, float(center[0]), float(center[1]), math.atan2(math.sin(yaw), math.cos(yaw)), side)
        with self.lock:
            self.latest = pose
        return pose

    def since(self, t: float) -> str:
        rows = [ln for ln in self.log_path.read_text().splitlines()[1:] if float(ln.split(",", 1)[0]) >= t]
        return "unix_time,x,y,yaw,quality\n" + "".join(r + "\n" for r in rows)


def serve(tracker: Tracker, port: int) -> None:
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802 (http.server API)
            url = urlparse(self.path)
            if url.path == "/pose":
                with tracker.lock:
                    p = tracker.latest
                body = json.dumps(None if p is None else p.__dict__ | {"age_s": round(time.time() - p.t, 3)})
                ctype = "application/json"
            elif url.path == "/log":
                try:
                    since = float(parse_qs(url.query).get("since", ["0"])[0])
                except ValueError:
                    self.send_error(400, "since must be a unix time")
                    return
                body, ctype = tracker.since(since), "text/csv"
            else:
                self.send_error(404)
                return
            data = body.encode()
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, *_args) -> None:
            pass

    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()


def track(cfg_path: str) -> None:
    cfg = load_config(cfg_path)
    tracker = Tracker(cfg)
    port = int(cfg.get("http_port", 8091))
    serve(tracker, port)
    print(f"tracking marker {tracker.marker_id}; http://<pi>:{port}/pose ; log {tracker.log_path}", flush=True)
    n, lost, t_report = 0, 0, time.time()
    while True:
        try:
            pose = tracker.step()
        except (OSError, RuntimeError) as e:
            print(f"camera error: {e}", flush=True)
            time.sleep(1.0)
            continue
        n += 1
        lost += pose is None
        if time.time() - t_report > 10:
            rate = n / (time.time() - t_report)
            p = tracker.latest
            where = f"x={p.x:.3f} y={p.y:.3f} yaw={math.degrees(p.yaw):.1f}" if p else "no fix yet"
            print(f"{rate:.1f} frames/s, {lost} without marker; {where}", flush=True)
            n, lost, t_report = 0, 0, time.time()


def main() -> None:
    if len(sys.argv) != 3 or sys.argv[1] not in ("calibrate", "track"):
        raise SystemExit(__doc__)
    {"calibrate": calibrate, "track": track}[sys.argv[1]](sys.argv[2])


if __name__ == "__main__":
    main()
