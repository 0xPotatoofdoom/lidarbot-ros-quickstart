"""Locate the robot against a reference lidar scan taken at a fixed spot (scan matching).

Odometry can't see its own sideways drift. A scan taken at the start of a run captures the
walls and furniture around the robot; matching later scans against it (2D ICP) gives the
robot's true offset and heading relative to that spot, whatever shape the walls are.

Only trust it near the reference spot. Measured 2026-11-11: standing at the spot it repeats
to 0.9 mm and 0.09 deg, but 1 m away it was off by about 2 cm (point-to-point matching on a
sparse scan, plus the lidar's ~3% range error at distance). So move.py steers by odometry
within a drive and uses the anchor to correct its line each time it returns (relocate).
numpy only (the container has no scipy), so point sets are subsampled to keep each match
to a few milliseconds.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

SELF_RANGE = 0.2      # m; closer returns are the robot's own body
MAX_RANGE = 3.0       # m; farther returns are sparse and noisy
REF_POINTS = 400
SCAN_POINTS = 200
ITERS = 20
PAIR_DIST = (0.20, 0.10, 0.05)   # m; inlier distance, tightened as the match converges
MIN_INLIERS = 0.5     # share of scan points that must match the reference
MAX_RMS = 0.03        # m


@dataclass(frozen=True)
class Located:
    x: float      # m, along the reference heading
    y: float      # m, left positive
    yaw: float    # rad, CCW positive
    rms: float    # m
    inliers: float


def scan_points(ranges, angle_min: float, inc: float, limit: int) -> np.ndarray:
    """Valid returns as (N, 2) points in the laser frame, evenly subsampled to <= limit."""
    r = np.asarray(ranges, np.float64)
    a = angle_min + inc * np.arange(len(r))
    with np.errstate(invalid="ignore"):
        keep = np.isfinite(r) & (r > SELF_RANGE) & (r < MAX_RANGE)
    pts = np.column_stack([r[keep] * np.cos(a[keep]), r[keep] * np.sin(a[keep])])
    if len(pts) > limit:
        pts = pts[np.linspace(0, len(pts) - 1, limit).astype(int)]
    return pts


def transform(pts: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return pts @ np.array([[c, s], [-s, c]]) + np.array([x, y])


def icp(src: np.ndarray, ref: np.ndarray, guess: tuple[float, float, float]) -> Located | None:
    """Pose (x, y, yaw) that maps src (current laser frame) onto ref (reference laser frame)."""
    x, y, yaw = guess
    if len(src) < 30 or len(ref) < 30:
        return None
    stage_len = ITERS // len(PAIR_DIST)
    rms, frac = math.inf, 0.0
    for it in range(ITERS):
        limit = PAIR_DIST[min(it // stage_len, len(PAIR_DIST) - 1)]
        moved = transform(src, x, y, yaw)
        d2 = ((moved[:, None, :] - ref[None, :, :]) ** 2).sum(axis=2)
        nn = d2.argmin(axis=1)
        dist = np.sqrt(d2[np.arange(len(src)), nn])
        ok = dist < limit
        frac = float(ok.mean())
        if ok.sum() < 20:
            return None
        rms = float(np.sqrt(np.mean(dist[ok] ** 2)))
        # Kabsch on the inlier pairs: best rigid motion from moved -> matched reference
        p, q = moved[ok], ref[nn[ok]]
        pc, qc = p.mean(axis=0), q.mean(axis=0)
        h = (p - pc).T @ (q - qc)
        dyaw = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
        c, s = math.cos(dyaw), math.sin(dyaw)
        rot = np.array([[c, -s], [s, c]])
        t = qc - rot @ pc
        # compose: new = rot * (R(yaw) p + [x, y]) + t
        x, y = (rot @ np.array([x, y]) + t).tolist()
        yaw = math.atan2(math.sin(yaw + dyaw), math.cos(yaw + dyaw))
        if abs(dyaw) < 1e-5 and np.hypot(*t) < 1e-5:
            break
    if frac < MIN_INLIERS or rms > MAX_RMS:
        return None
    return Located(x, y, yaw, rms, frac)


class ScanAnchor:
    """Reference scan plus the odometry pose it was taken at."""

    def __init__(self, scan, odom_pose: tuple[float, float, float]) -> None:
        self.ref = scan_points(scan.ranges, scan.angle_min, scan.angle_increment, REF_POINTS)
        self.odom0 = odom_pose
        if len(self.ref) < 100:
            raise ValueError(f"only {len(self.ref)} lidar returns within {MAX_RANGE} m; too open to anchor")

    def odom_guess(self, odom_pose: tuple[float, float, float],
                   anchor_odom: tuple[float, float, float] | None = None) -> tuple[float, float, float]:
        """Odometry's idea of the current pose, in the anchor's frame.

        anchor_odom: where the anchor spot is in odometry coordinates now (the last
        anchor_in_odom() result). Odometry drifts by centimetres over many cycles, so the
        guess must start from the latest correction, not from where the anchor was taken:
        once the guess is ~20 cm off, matching can't find the way back (2026-11-11 run).
        """
        x0, y0, yaw0 = anchor_odom or self.odom0
        x, y, yaw = odom_pose
        c, s = math.cos(yaw0), math.sin(yaw0)
        dx, dy = x - x0, y - y0
        return c * dx + s * dy, -s * dx + c * dy, math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0))

    def locate(self, scan, odom_pose: tuple[float, float, float],
               anchor_odom: tuple[float, float, float] | None = None) -> Located | None:
        src = scan_points(scan.ranges, scan.angle_min, scan.angle_increment, SCAN_POINTS)
        return icp(src, self.ref, self.odom_guess(odom_pose, anchor_odom))


def compose(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    """Pose b expressed in a's frame, mapped to a's parent frame (a then b)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    yaw = a[2] + b[2]
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], math.atan2(math.sin(yaw), math.cos(yaw))


def invert(p: tuple[float, float, float]) -> tuple[float, float, float]:
    c, s = math.cos(p[2]), math.sin(p[2])
    return -(c * p[0] + s * p[1]), -(-s * p[0] + c * p[1]), -p[2]


def anchor_in_odom(odom_pose: tuple[float, float, float], located: Located) -> tuple[float, float, float]:
    """Where the anchor spot is now, in odometry coordinates, given where the scan match says
    the robot is relative to it. Use it as the line for move.Mover.forward()."""
    return compose(odom_pose, invert((located.x, located.y, located.yaw)))
