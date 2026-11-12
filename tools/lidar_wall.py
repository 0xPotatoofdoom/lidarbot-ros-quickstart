"""Straight-wall fits from a lidar scan (numpy only; used by move.py and wall_target.py)."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

SELF_RANGE = 0.2   # m; closer returns are the robot's own body


@dataclass(frozen=True)
class LineFit:
    normal_deg: float   # direction of the perpendicular from the laser to the line, CCW + (0 = ahead)
    distance_m: float   # perpendicular distance from the laser
    rms_mm: float
    points: int


def fit_line(ranges: np.ndarray, angle_min: float, inc: float, center: float = 0.0,
             arc: float = math.radians(25), max_range: float = math.inf, min_points: int = 20) -> LineFit | None:
    """Total-least-squares line through returns within `arc` of `center` (rad), one outlier pass."""
    a = angle_min + inc * np.arange(len(ranges))
    rel = np.arctan2(np.sin(a - center), np.cos(a - center))
    r = np.asarray(ranges, np.float64)
    with np.errstate(invalid="ignore"):
        keep = np.isfinite(r) & (r > SELF_RANGE) & (r < max_range) & (np.abs(rel) <= arc)
    pts = np.column_stack([r[keep] * np.cos(a[keep]), r[keep] * np.sin(a[keep])])
    for _ in range(2):
        if len(pts) < min_points:
            return None
        c = pts.mean(axis=0)
        normal = np.linalg.svd(pts - c)[2][1]          # smallest-variance direction
        resid = (pts - c) @ normal
        rms = float(np.sqrt(np.mean(resid ** 2)))
        inliers = np.abs(resid) <= max(3 * rms, 0.005)
        if inliers.all():
            break
        pts = pts[inliers]
    dist = float(c @ normal)
    if dist < 0:
        normal, dist = -normal, -dist
    return LineFit(math.degrees(math.atan2(normal[1], normal[0])), dist, rms * 1000, len(pts))
