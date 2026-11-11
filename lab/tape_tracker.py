#!/usr/bin/env python3
"""Robot position along a tape measure, from an overhead phone camera.

A strip of blue painter's tape runs along the robot's path, with black marks
drawn across it at known tape-measure readings (e.g. every foot). The robot
drives along the strip and hides it; the place where the strip disappears is
the robot's front edge. This gives the same number a person reading the tape
measure at the bumper would, automatically, in every frame.

    python3 tape_tracker.py check tape.json            # one frame: marks found, robot position
    python3 tape_tracker.py track tape.json            # log readings, serve over HTTP

tape.json (see tape.example.json):
    camera_url        IP Webcam base URL (/shot.jpg is fetched)
    marks_in          readings of the black marks, in order from the door end, e.g. [12, 24, 36, 48, 60]
    camera_height_m   phone lens height above the floor
    edge_bottom_m     height of the bottom of the robot's front bumper above the floor
    edge_top_m        height of the top of the front bumper
    log_dir, http_port

Readings are in inches along the tape (the robot's front edge, projected straight
down). The log (CSV: unix_time,front_in,raw_in,s_px,gap_px,edge_x,edge_y) keeps the raw
image position of the edge, so readings can be recomputed if heights are re-measured.
HTTP: GET /reading -> latest reading as JSON; GET /log?since=<unix_time> -> CSV rows.
"""
from __future__ import annotations

import json
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
from numpy.lib.stride_tricks import sliding_window_view

BLUE_LO = np.array([95, 80, 100])    # HSV; painter's tape blue (V >= 100 excludes the navy robot)
BLUE_HI = np.array([130, 255, 255])
BAND_HALF = 6          # px either side of the center line that are sampled
MIN_GAP = 60           # px of non-blue along the line that counts as the robot (marks are ~5 px)
BLUE_FILL = 50         # px; shorter blue runs between dark ones are reflections on the robot's top plate
MARK_BAND = 14         # px either side of the center line searched for marks
MARK_WINDOW = 41       # px; a mark is darker than the median of this window around it
MARK_MIN_DIP = 10      # brightness units a mark must be darker than its surroundings
MARK_SEPARATION = 60   # px; minimum distance between two marks
MAX_CAL_RMS_IN = 0.25  # a good fit is ~0.06 in; much worse means a wrong mark
FETCH_TIMEOUT = 5.0
LOG_HEADER = "unix_time,front_in,raw_in,s_px,gap_px,edge_x,edge_y\n"


@dataclass(frozen=True)
class Calibration:
    origin: tuple[float, float]       # a point on the tape center line (px)
    direction: tuple[float, float]    # unit vector along the line, toward higher readings
    proj: tuple[float, float, float]  # 1D homography: inches = (a*s + b) / (c*s + 1)
    marks_px: tuple[float, ...]       # mark positions along the line (px from origin)
    rms_in: float                     # fit residual of the marks
    nadir_s: float                    # image center projected onto the line (px)


def fetch(url: str) -> tuple[float, np.ndarray]:
    t0 = time.time()
    with urllib.request.urlopen(url.rstrip("/") + "/shot.jpg", timeout=FETCH_TIMEOUT) as r:
        data = r.read()
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError("camera returned something that isn't a JPEG")
    return (t0 + time.time()) / 2, img


def blue_mask(img: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, BLUE_LO, BLUE_HI)


def center_line(mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Least-squares line through the blue strip: (point, unit direction pointing image-right)."""
    ys, xs = np.nonzero(mask)
    if len(xs) < 500:
        raise ValueError("blue tape not found")
    pts = np.column_stack([xs, ys]).astype(np.float32)
    vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
    d = np.array([vx, vy])
    if d[0] < 0:
        d = -d
    return np.array([x0, y0]), d


def profile(mask: np.ndarray, origin: np.ndarray, d: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Fraction of blue across the strip's center band at each position along the line."""
    h, w = mask.shape
    n = np.array([-d[1], d[0]])
    # range of s that stays inside the image
    s_vals = np.arange(-4000, 4000, 1.0)
    pts = origin + np.outer(s_vals, d)
    inside = (pts[:, 0] >= 0) & (pts[:, 0] < w - 1) & (pts[:, 1] >= 0) & (pts[:, 1] < h - 1)
    s_vals = s_vals[inside]
    offsets = np.arange(-BAND_HALF, BAND_HALF + 1)
    frac = np.zeros(len(s_vals))
    for o in offsets:
        p = origin + np.outer(s_vals, d) + o * n
        frac += mask[p[:, 1].round().astype(int), p[:, 0].round().astype(int)] > 0
    return s_vals, frac / len(offsets)


def runs(flags: np.ndarray) -> list[tuple[int, int]]:
    """(start, end) index pairs of consecutive True values."""
    edges = np.diff(np.concatenate([[0], flags.astype(int), [0]]))
    return list(zip(np.nonzero(edges == 1)[0], np.nonzero(edges == -1)[0], strict=True))


def fill_reflections(blue: np.ndarray) -> np.ndarray:
    """Treat short blue runs with dark on both sides as part of the robot.

    The robot's glossy top plate reflects bluish light that can pass the tape color
    test, splitting the robot into several dark pieces along the line.
    """
    out = blue.copy()
    for a, b in runs(blue):
        if a > 0 and b < len(blue) and b - a < BLUE_FILL:
            out[a:b] = False
    return out


def fit_projective(s: np.ndarray, inches: np.ndarray) -> tuple[float, float, float]:
    """inches = (a*s + b) / (c*s + 1), linear least squares in a, b, c."""
    a_mat = np.column_stack([s, np.ones_like(s), -s * inches])
    a, b, c = np.linalg.lstsq(a_mat, inches, rcond=None)[0]
    return float(a), float(b), float(c)


def to_inches(proj: tuple[float, float, float], s: float) -> float:
    a, b, c = proj
    return (a * s + b) / (c * s + 1)


def find_marks(img: np.ndarray, origin: np.ndarray, d: np.ndarray, s_vals: np.ndarray, count: int) -> list[float]:
    """Positions (px along the line) of the `count` strongest dark marks on the tape.

    Hand-drawn marks are thin and may cover only part of the tape's width, so each
    position is scored by how much darker than its neighborhood it is, using the
    strongest few of several lines across the tape.
    """
    v = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)[:, :, 2].astype(float)
    n = np.array([-d[1], d[0]])
    h, w = v.shape
    rows = []
    for off in range(-MARK_BAND, MARK_BAND + 1, 2):
        p = origin + np.outer(s_vals, d) + off * n
        x = np.clip(p[:, 0].round().astype(int), 0, w - 1)
        y = np.clip(p[:, 1].round().astype(int), 0, h - 1)
        rows.append(v[y, x])
    lines = np.array(rows)
    half = MARK_WINDOW // 2
    med = np.median(sliding_window_view(np.pad(lines, ((0, 0), (half, half)), mode="edge"), MARK_WINDOW, axis=1), axis=2)
    score = np.sort(med - lines, axis=0)[-5:].mean(axis=0)
    picked: list[int] = []
    for i in np.argsort(-score):
        if score[i] < MARK_MIN_DIP or len(picked) == count:
            break
        if all(abs(int(i) - j) >= MARK_SEPARATION for j in picked):
            picked.append(int(i))
    return sorted(float(s_vals[i]) for i in picked)


def calibrate(img: np.ndarray, marks_in: list[float]) -> Calibration:
    """Find the tape and its marks in a frame with the robot clear of the marks."""
    mask = blue_mask(img)
    origin, d = center_line(mask)
    s_vals, frac = profile(mask, origin, d)
    # only search the stretch of tape before the robot (if it's in view)
    blue = fill_reflections(frac > 0.5)
    gaps = [(a, b) for a, b in runs(~blue) if b - a >= MIN_GAP and a > 0]
    end = gaps[0][0] if gaps else len(s_vals)
    marks = find_marks(img, origin, d, s_vals[:end], len(marks_in))
    if len(marks) != len(marks_in):
        raise ValueError(f"found {len(marks)} marks on the tape, expected {len(marks_in)}: "
                         f"{[round(m) for m in marks]}")
    s = np.array(marks)
    inches = np.array(marks_in, dtype=float)
    proj = fit_projective(s, inches)
    resid = np.array([to_inches(proj, x) for x in s]) - inches
    rms = float(np.sqrt(np.mean(resid ** 2)))
    if rms > MAX_CAL_RMS_IN:
        raise ValueError(f"marks don't fit their readings (rms {rms:.2f} in): a mark was missed or "
                         f"something else was taken for one; found at px {[round(m) for m in marks]}")
    h, w = mask.shape
    nadir_s = float(np.dot(np.array([w / 2, h / 2]) - origin, d))
    return Calibration((float(origin[0]), float(origin[1])), (float(d[0]), float(d[1])), proj,
                       tuple(marks), rms, nadir_s)


@dataclass(frozen=True)
class Reading:
    front_in: float   # front edge projected to the floor (height-corrected), inches
    raw_in: float     # where the edge appears on the floor plane, no height correction
    s_px: float       # edge position along the line in pixels, for recomputing later
    gap_px: float     # length of the hidden stretch of tape (robot size check)


def strip_mask(img: np.ndarray, cal: Calibration) -> np.ndarray:
    """Blue mask computed only in the band of rows the tape crosses (much faster)."""
    h, w = img.shape[:2]
    ox, oy = cal.origin
    dx, dy = cal.direction
    ys = [oy + (x - ox) * dy / dx for x in (0, w - 1)]
    y0, y1 = max(int(min(ys)) - 60, 0), min(int(max(ys)) + 60, h)
    mask = np.zeros((h, w), np.uint8)
    mask[y0:y1] = blue_mask(img[y0:y1])
    return mask


def front_edge(img: np.ndarray, cal: Calibration, cfg: dict) -> Reading | None:
    """Robot front edge, or None if no robot is on the tape."""
    origin, d = np.array(cal.origin), np.array(cal.direction)
    s_vals, frac = profile(strip_mask(img, cal), origin, d)
    blue = fill_reflections(frac > 0.5)
    # readings increase with s or decrease; the robot sits at the high-reading end of the visible strip
    increasing = to_inches(cal.proj, cal.marks_px[-1]) > to_inches(cal.proj, cal.marks_px[0])
    gaps = [(a, b) for a, b in runs(~blue) if b - a >= MIN_GAP]
    if not gaps:
        return None
    # The front edge is where the strip, coming from the door (low readings), first disappears.
    if increasing:
        a, b = min(gaps, key=lambda g: s_vals[g[0]])
        s_edge = float(s_vals[a])
    else:
        a, b = max(gaps, key=lambda g: s_vals[g[1] - 1])
        s_edge = float(s_vals[b - 1])
    # The edge is above the floor: in the image it sits further from the nadir (the
    # point under the lens) than its footprint. Pull it back by (H - h) / H. Which
    # edge hides the tape depends on the side: with the lens in front of the robot
    # it sees the bumper's face, so the bottom edge; past it, the top edge.
    toward_readings = 1.0 if increasing else -1.0
    lens_in_front = (s_edge - cal.nadir_s) * toward_readings > 0
    edge_h = cfg["edge_bottom_m"] if lens_in_front else cfg["edge_top_m"]
    scale = (cfg["camera_height_m"] - edge_h) / cfg["camera_height_m"]
    s_true = cal.nadir_s + (s_edge - cal.nadir_s) * scale
    return Reading(to_inches(cal.proj, s_true), to_inches(cal.proj, s_edge), s_edge, float(b - a))


def check(cfg_path: str) -> None:
    cfg = json.loads(Path(cfg_path).read_text())
    _, img = fetch(cfg["camera_url"])
    cal = calibrate(img, cfg["marks_in"])
    print(f"marks at px {[round(m) for m in cal.marks_px]}, fit rms {cal.rms_in:.3f} in")
    edge = front_edge(img, cal, cfg)
    print("no robot on the tape" if edge is None else
          f"robot front edge at {edge.front_in:.2f} in (uncorrected {edge.raw_in:.2f} in)")


def track(cfg_path: str) -> None:
    cfg = json.loads(Path(cfg_path).read_text())
    _, img = fetch(cfg["camera_url"])
    cal = calibrate(img, cfg["marks_in"])
    log_dir = Path(cfg.get("log_dir", "~/lab/logs")).expanduser()
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"tape_{time.strftime('%Y%m%d', time.gmtime())}.csv"
    if not log_path.exists():
        log_path.write_text(LOG_HEADER)
    latest: dict = {}
    lock = threading.Lock()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802 (http.server API)
            url = urlparse(self.path)
            if url.path == "/reading":
                with lock:
                    body, ctype = json.dumps(latest or None), "application/json"
            elif url.path == "/log":
                try:
                    since = float(parse_qs(url.query).get("since", ["0"])[0])
                except ValueError:
                    self.send_error(400, "since must be a unix time")
                    return
                rows = [ln for ln in log_path.read_text().splitlines()[1:] if float(ln.split(",", 1)[0]) >= since]
                body, ctype = LOG_HEADER + "".join(r + "\n" for r in rows), "text/csv"
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

    (log_dir / "tape_calibration.json").write_text(json.dumps(cal.__dict__ | {"utc": time.strftime(
        "%Y-%m-%dT%H:%M:%SZ", time.gmtime())}, indent=2) + "\n")
    port = int(cfg.get("http_port", 8092))
    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"calibrated: {len(cal.marks_px)} marks, rms {cal.rms_in:.3f} in; http://<pi>:{port}/reading", flush=True)
    while True:
        try:
            t, img = fetch(cfg["camera_url"])
        except (OSError, RuntimeError) as e:
            print(f"camera error: {e}", flush=True)
            time.sleep(1.0)
            continue
        edge = front_edge(img, cal, cfg)
        if edge is None:
            continue
        with lock:
            latest.update(unix_time=round(t, 3), front_in=round(edge.front_in, 3), raw_in=round(edge.raw_in, 3),
                          s_px=round(edge.s_px, 1), gap_px=edge.gap_px)
        with log_path.open("a") as f:
            ex = cal.origin[0] + edge.s_px * cal.direction[0]
            ey = cal.origin[1] + edge.s_px * cal.direction[1]
            f.write(f"{t:.3f},{edge.front_in:.3f},{edge.raw_in:.3f},{edge.s_px:.1f},{edge.gap_px:.0f},"
                    f"{ex:.1f},{ey:.1f}\n")


def main() -> None:
    if len(sys.argv) != 3 or sys.argv[1] not in ("check", "track"):
        raise SystemExit(__doc__)
    {"check": check, "track": track}[sys.argv[1]](sys.argv[2])


if __name__ == "__main__":
    main()
