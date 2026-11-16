#!/usr/bin/env python3
"""Low obstacles from the camera: things on the floor the lidar's beam passes over.

    ./ros -d python3 /work/tools/floor_obstacles.py --ros-args      (./up nav starts it)
    ./ros python3 /work/tools/floor_obstacles.py --test IMAGE.jpg   (offline: writes IMAGE_floor.jpg)

The lidar scans a plane about 15 cm up, so a dog toy or a sock is invisible to Nav2. The camera
looks down at the floor ahead while driving (tilt -53, about 8 degrees below level). Each frame:

  1. learn the floor's colours from the whole visible floor (hue/saturation histogram, blended
     over frames): obstacles are small, so their colours are rare there; mark pixels with rare
     colours as not floor, keep only blobs the size of a real object; bright, colourless glare
     on a shiny floor counts as floor
  2. in each image column, the lowest run of non-floor pixels belongs to the nearest object;
     the object meets the floor at its blob's lowest row (a rounded object's sides would
     otherwise project further away and stretch it along the view)
  3. that row and column go through the camera model onto the floor plane (camera 0.12 m up,
     pitched down by the measured angle), giving a point in base_footprint

Published, for a separate Nav2 obstacle layer (the lidar layer would clear these cells, its
beam passes over them):
  /camera_obstacles   PointCloud2, base_footprint: where objects meet the floor (marking)
  /camera_floor_free  PointCloud2, base_footprint: how far each column saw clear floor (clearing)
Only while the camera points straight ahead at the driving tilt (pan 0, tilt -53).

Camera: GC2145, 640x480, fx about 1000 (checked 2026-11-16 against the 80 mm wall markers),
stream about 0.4 s behind at 5 fps. At tilt -53 the floor is visible from about 0.30 m ahead;
the field of view is narrow (about 35 degrees across), so this only covers the path ahead.
"""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

CAM_HEIGHT_M = 0.12        # optical centre above the floor (tools/tilt_check.py)
CAM_X_M = 0.05             # optical centre ahead of base_footprint (lidar is at -0.028)
DRIVE_TILT = -53.0         # ./up aims the camera here for driving
PITCH_AT_DRIVE_DEG = 8.4   # measured 2026-11-16: wall/floor line row vs lidar wall distance
TILT_SCALE = 0.872         # real degrees per commanded degree (tools/tilt_check.py)
MAX_RANGE_M = 1.5          # ignore anything further: the floor is too foreshortened
REF_ROWS = (420, 480)      # strip just ahead (about 0.32-0.36 m), drawn for reference only
REF_COLS = (200, 440)
MIN_BLOB_PX = 250          # smaller non-floor blobs are noise (the toy at 1 m is about 2500)
MIN_HEIGHT_M = 0.03        # blobs that would stand less tall than this are flat: floor stains, grain
COL_STEP = 8               # columns sampled
MIN_RUN = 6                # rows of non-floor in a column before it counts as an object
FLOOR_PROB = 0.03          # colour bins rarer than this (fraction of the commonest) = not floor
GLARE_V, GLARE_S = 200, 60 # brighter and greyer than this = reflection on the floor
MODEL_BLEND = 0.2          # weight of each new frame in the floor colour model
H_BINS, S_BINS = 30, 32


@dataclass(frozen=True)
class Camera:
    k: np.ndarray
    dist: np.ndarray
    pitch_deg: float

    def valid(self, shape: tuple[int, int]) -> np.ndarray:
        """Pixels that hold image after undistortion (the corners don't), shrunk a little."""
        import cv2
        white = np.full(shape, 255, np.uint8)
        ok = cv2.undistort(white, self.k, self.dist) > 250
        return cv2.erode(ok.astype(np.uint8), np.ones((7, 7), np.uint8)).astype(bool)

    def floor_point(self, u: float, v: float) -> tuple[float, float] | None:
        """Image pixel (undistorted) -> (x forward, y left) on the floor in base_footprint."""
        fx, fy, cx, cy = self.k[0, 0], self.k[1, 1], self.k[0, 2], self.k[1, 2]
        # ray in camera axes: forward, left, up (before pitch)
        f, l, up = 1.0, -(u - cx) / fx, -(v - cy) / fy
        p = math.radians(self.pitch_deg)
        fw = f * math.cos(p) + up * math.sin(p)        # rotate down by the pitch
        zz = -f * math.sin(p) + up * math.cos(p)
        if zz >= -1e-6:
            return None                                # at or above the horizon
        t = CAM_HEIGHT_M / -zz
        return CAM_X_M + fw * t, l * t

    def upright_height(self, bottom_row: float, top_row: float) -> float:
        """Height of an upright object standing where bottom_row meets the floor and reaching
        up to top_row (image centre column). A flat patch on the floor comes out low: for one
        from d to d2 ahead, CAM_HEIGHT_M * (1 - d / d2)."""
        cy, fy = self.k[1, 2], self.k[1, 1]
        down = lambda v: math.radians(self.pitch_deg) + math.atan((v - cy) / fy)   # noqa: E731
        if down(bottom_row) <= 0:
            return math.inf
        d = CAM_HEIGHT_M / math.tan(down(bottom_row))
        return CAM_HEIGHT_M - d * math.tan(down(top_row))


def pitch_for_tilt(tilt_cmd: float) -> float:
    return PITCH_AT_DRIVE_DEG + TILT_SCALE * (DRIVE_TILT - tilt_cmd)


class FloorModel:
    """Hue/saturation histogram of the floor, blended across frames."""

    def __init__(self) -> None:
        self.hist: np.ndarray | None = None

    def update(self, hsv: np.ndarray, top: int) -> None:
        import cv2
        ref = hsv[top:]
        mask = ~self._glare(ref)
        h = cv2.calcHist([ref], [0, 1], mask.astype(np.uint8), [H_BINS, S_BINS], [0, 180, 0, 256])
        h = cv2.GaussianBlur(h, (3, 3), 0)
        h /= max(float(h.max()), 1e-6)
        self.hist = h if self.hist is None else (1 - MODEL_BLEND) * self.hist + MODEL_BLEND * h

    @staticmethod
    def _glare(hsv: np.ndarray) -> np.ndarray:
        return (hsv[..., 2] > GLARE_V) & (hsv[..., 1] < GLARE_S)

    def not_floor(self, hsv: np.ndarray) -> np.ndarray:
        import cv2
        prob = cv2.calcBackProject([hsv], [0, 1], self.hist.astype(np.float32), [0, 180, 0, 256], 255.0)
        prob = cv2.GaussianBlur(prob, (5, 5), 0)
        mask = (prob < FLOOR_PROB * 255) & ~self._glare(hsv)
        mask = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        keep = np.zeros(n, bool)
        keep[1:] = stats[1:, cv2.CC_STAT_AREA] >= MIN_BLOB_PX
        return keep[labels]


@dataclass(frozen=True)
class Detection:
    obstacles: list[tuple[float, float]]       # floor contact points (x, y)
    free: list[tuple[float, float]]            # end of clear floor per column (x, y)
    rows: list[tuple[int, int | None]]         # (column, contact row) for drawing


def detect(bgr: np.ndarray, cam: Camera, model: FloorModel) -> tuple[Detection, np.ndarray]:
    """Undistort, update the floor model, find each column's lowest object run."""
    import cv2
    img = cv2.undistort(bgr, cam.k, cam.dist)
    hsv = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2HSV)
    # the farthest useful row: where the floor is MAX_RANGE_M away, never above the horizon
    top = 0
    for v in range(img.shape[0] - 1, -1, -1):
        pt = cam.floor_point(cam.k[0, 2], v)
        if pt is None or pt[0] > MAX_RANGE_M:
            top = v + 1
            break
    model.update(hsv, top)
    mask = model.not_floor(hsv) & cam.valid(img.shape[:2])   # the black undistortion corners
    mask[:top] = False
    # (2026-11-16: without this, the bottom-right corner was an obstacle 0.35 m ahead in every frame)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    blob_top = stats[:, cv2.CC_STAT_TOP]
    blob_bottom = blob_top + stats[:, cv2.CC_STAT_HEIGHT] - 1
    # keep blobs that stand up; one cut off at the top row goes on up, so it counts as tall
    upright = np.array([i > 0 and (blob_top[i] <= top or
                        cam.upright_height(blob_bottom[i], blob_top[i]) >= MIN_HEIGHT_M) for i in range(n)])
    mask = upright[labels]
    obstacles, free, rows = [], [], []
    for u in range(COL_STEP // 2, img.shape[1], COL_STEP):
        col = mask[top:, u]
        hit = None
        run = 0
        for i in range(len(col) - 1, -1, -1):          # bottom up
            run = run + 1 if col[i] else 0
            if run >= MIN_RUN:
                hit = int(blob_bottom[labels[top + i, u]])   # where this object meets the floor
                break
        rows.append((u, hit))
        end = cam.floor_point(u, hit if hit is not None else top)
        if end is None:
            continue
        if hit is not None:
            obstacles.append(end)
        free.append(end)
    return Detection(obstacles, free, rows), mask


def draw(bgr: np.ndarray, cam: Camera, det: Detection, mask: np.ndarray) -> np.ndarray:
    import cv2
    out = cv2.undistort(bgr, cam.k, cam.dist)
    out[mask] = (0.5 * out[mask] + [0, 0, 127]).astype(np.uint8)
    for (u, v), pt in zip([(u, v) for u, v in det.rows if v is not None], det.obstacles):
        cv2.circle(out, (u, v), 3, (0, 255, 255), -1)
        cv2.putText(out, f"{pt[0]:.2f}", (u - 12, v + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
    cv2.rectangle(out, (REF_COLS[0], REF_ROWS[0]), (REF_COLS[1], REF_ROWS[1] - 1), (0, 255, 0), 1)
    return out


def load_camera(pitch_deg: float) -> Camera:
    from wall_target import load_intrinsics
    k, d, _ = load_intrinsics()
    return Camera(k, d, pitch_deg)


def test(path: str) -> None:
    import cv2
    cam = load_camera(PITCH_AT_DRIVE_DEG)
    img = cv2.imread(path)
    model = FloorModel()
    det, mask = detect(img, cam, model)
    out = Path(path).with_name(Path(path).stem + "_floor.jpg")
    cv2.imwrite(str(out), draw(img, cam, det, mask))
    print(f"{len(det.obstacles)} obstacle columns -> {out}")
    for (u, v), pt in zip([(u, v) for u, v in det.rows if v is not None], det.obstacles):
        print(f"  column {u:3d} row {v:3d}: {pt[0]:.2f} m ahead, {pt[1]:+.2f} m left")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--test", metavar="IMAGE", help="run on one saved frame and exit")
    args, _ = ap.parse_known_args()
    if args.test:
        test(args.test)
        return
    from floor_obstacles_node import run
    run()


if __name__ == "__main__":
    main()
