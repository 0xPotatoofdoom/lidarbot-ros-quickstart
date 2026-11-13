#!/usr/bin/env python3
"""Figures for posts and write-ups, made from the logs in datasets/ (runs in .venv-hf).

    .venv-hf/bin/python tools/figures.py battery     # idle discharge curve
    .venv-hf/bin/python tools/figures.py drift       # return-to-start offset, 772 cycles
    .venv-hf/bin/python tools/figures.py nav         # Nav2 arrival error by tuning step
    .venv-hf/bin/python tools/figures.py gif PATH.npz  # replay from tools/extract_path.py

PNG files go to datasets/figures/ (git-ignored; copy what you publish). 1600 x 900, light
background, sized for X. Colors: the validated two-slot categorical palette (blue, orange).
"""
from __future__ import annotations

import csv
import re
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "datasets"
OUT = DATA / "figures"
SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
BLUE, ORANGE = "#2a78d6", "#eb6834"
SIZE, DPI = (16, 9), 100

IDLE_START, IDLE_END = 1794416469.0, 1794416469.0 + 7.2 * 3600   # 2026-11-11 09:01 local
DRIVE_LOGS = ["battery/drive_run_20261112T034817Z.log", "battery/drive_run.log"]
NAV_RUNS = [("Nav2 defaults\n10 cm tolerance", "nav/loop_20261112T231442Z.csv"),
            ("Tuned AMCL\n5 cm tolerance", "nav/loop_20261112T235420Z.csv"),
            ("+ slower final turn", "nav/loop_20261113T000159Z.csv")]


def style(ax, title: str | None = None, subtitle: str = "") -> None:
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(INK2)
    ax.tick_params(colors=INK2, labelsize=15)
    ax.grid(color=GRID, linewidth=1)
    ax.set_axisbelow(True)
    if title:
        fig = ax.figure
        fig.text(0.06, 0.94, title, fontsize=28, weight="bold", color=INK)
        fig.text(0.06, 0.895, subtitle, fontsize=17, color=INK2)


def new_fig(ncols: int = 1):
    fig, axes = plt.subplots(1, ncols, figsize=SIZE, dpi=DPI, facecolor=SURFACE)
    fig.subplots_adjust(left=0.08, right=0.97, top=0.82, bottom=0.12, wspace=0.25)
    return fig, axes


def save(fig, name: str) -> Path:
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / name
    fig.text(0.97, 0.02, "lidarbot / github.com/0xPotatoofdoom/lidarbot-ros-quickstart",
             fontsize=12, color=INK2, ha="right")
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)
    return path


def battery() -> Path:
    rows = []
    for f in sorted((DATA / "battery").glob("battery_2026111[12].csv")):
        with f.open() as fh:
            for r in csv.DictReader(fh):
                t = float(r["unix_time"])
                if IDLE_START <= t <= IDLE_END and r["rest_voltage"]:
                    rows.append((t, float(r["rest_voltage"])))
    rows.sort()
    t = (np.array([r[0] for r in rows]) - IDLE_START) / 3600
    v = np.array([r[1] for r in rows])
    fig, ax = new_fig()
    style(ax, "7.1 hours on one charge, sitting still",
          "3S 2200 mAh pack, lidar spinning and camera streaming, logged every 5 s until the low-voltage beep")
    ax.plot(t, v, color=BLUE, linewidth=2.5)
    ax.set_xlabel("hours", fontsize=16, color=INK2)
    ax.set_ylabel("pack voltage (V)", fontsize=16, color=INK2)
    ax.set_xlim(0, 7.4)
    for x, y, label, dy in [(0.0, v[0], f"{v[0]:.2f} V full", 0.08), (t[-1] - 1.0, v[-1], f"{v[-1]:.2f} V beep", -0.05)]:
        ax.annotate(label, (x, y), (x + 0.1, y + dy), fontsize=15, color=INK)
    ax.annotate("step near 11.9 V: the cells' chemistry,\nwhich a generic Li-ion curve misses",
                (3.9, 11.75), (1.2, 11.0), fontsize=15, color=INK,
                arrowprops={"arrowstyle": "-", "color": INK2})
    ax.annotate("last 10% lasts about 40 min", (6.6, 10.8), (4.6, 10.45), fontsize=15, color=INK,
                arrowprops={"arrowstyle": "-", "color": INK2})
    return save(fig, "battery_idle.png")


def drift() -> Path:
    offsets = []
    for log in DRIVE_LOGS:
        for line in (DATA / log).read_text().splitlines():
            if m := re.search(r"at start ([-+0-9.]+) mm", line):
                offsets.append(float(m.group(1)))
    y = np.array(offsets)
    x = np.arange(1, len(y) + 1)
    fig, ax = new_fig()
    style(ax, f"Back to the same spot {len(y)} times, within millimeters",
          "1 m out and back, two battery-test legs. Each return: lidar scan matched against the first scan, line reset")
    ax.axhline(0, color=INK2, linewidth=1)
    ax.scatter(x, y, s=22, color=BLUE, label="scan-matched return to start", zorder=3)
    odo = -3.8 * x
    keep = odo > -205
    ax.plot(x[keep], odo[keep], color=ORANGE, linewidth=2.5, linestyle="--",
            label="odometry alone (measured: 19 cm off after 50 cycles)")
    ax.set_ylim(-205, 95)
    ax.set_xlim(0, len(y) + 10)
    ax.set_xlabel("cycle", fontsize=16, color=INK2)
    ax.set_ylabel("offset from start (mm)", fontsize=16, color=INK2)
    worst = int(np.argmax(y))
    ax.annotate("one false match (84 mm), now rejected by a sanity check",
                (x[worst], y[worst]), (x[worst] + 25, 80), fontsize=14, color=INK,
                arrowprops={"arrowstyle": "-", "color": INK2})
    core = np.delete(y, worst)
    ax.text(len(y) * 0.55, -150, f"mean {core.mean():+.1f} mm, spread {core.std():.1f} mm, no growth",
            fontsize=17, color=INK)
    ax.legend(loc="lower left", fontsize=15, frameon=False, labelcolor=INK)
    return save(fig, "drift_return_to_start.png")


def nav() -> Path:
    fig, (ax1, ax2) = new_fig(2)
    fig.subplots_adjust(left=0.17)
    style(ax2)
    style(ax1, "Nav2 on a $200 rover: 45 of 45 goals, now 1-4 cm",
          "Kitchen -> den doorway -> living room -> back, about 15 m per lap. Each dot is one arrival (AMCL's pose)")
    for i, (label, path) in enumerate(NAV_RUNS):
        with (DATA / path).open() as fh:
            rows = list(csv.DictReader(fh))
        d = [float(r["amcl_err_cm"]) for r in rows]
        h = []
        for r in rows:
            yaw, goal = float(r["amcl_yaw_deg"]), float(r["goal_yaw_deg"])
            h.append(abs((yaw - goal + 180) % 360 - 180))
        jitter = (np.random.default_rng(i).random(len(d)) - 0.5) * 0.25
        color = BLUE if i == len(NAV_RUNS) - 1 else INK2
        ax1.scatter(d, np.full(len(d), i) + jitter, s=90, color=color, zorder=3)
        ax2.scatter(h, np.full(len(h), i) + jitter, s=90, color=color, zorder=3)
    for ax, xlabel in ((ax1, "distance from goal (cm)"), (ax2, "heading error (deg)")):
        ax.set_yticks(range(len(NAV_RUNS)))
        ax.set_yticklabels([r[0] for r in NAV_RUNS], fontsize=15, color=INK)
        ax.invert_yaxis()
        ax.set_xlim(left=0)
        ax.set_xlabel(xlabel, fontsize=16, color=INK2)
    ax2.set_yticklabels([])
    ax2.annotate("robot coasted past\nthe goal heading", (7.9, 1), (5.2, 1.65), fontsize=14, color=INK,
                 arrowprops={"arrowstyle": "-", "color": INK2})
    return save(fig, "nav2_tuning.png")


def gif(npz_path: str) -> Path:
    """Robot-centered replay: lidar points and the path so far, robot fixed at the center."""
    from PIL import Image
    d = np.load(npz_path)
    poses, pts, counts = d["poses"], d["points"], d["counts"]
    frames, start, half = [], 0, 2.0
    for k in range(len(poses)):
        x, y, yaw = poses[k]
        scan = pts[start:start + counts[k]]
        start += counts[k]
        fig = plt.figure(figsize=(6, 6), dpi=90, facecolor=SURFACE)
        ax = fig.add_axes([0, 0, 1, 1])
        ax.set_facecolor(SURFACE)
        ax.set_xlim(x - half, x + half)
        ax.set_ylim(y - half, y + half)
        ax.set_aspect("equal")
        ax.axis("off")
        ax.plot(poses[:k + 1, 0], poses[:k + 1, 1], color=ORANGE, linewidth=2)
        ax.scatter(scan[:, 0], scan[:, 1], s=6, color=BLUE)
        c, s = np.cos(yaw), np.sin(yaw)
        body = np.array([[0.11, 0.105], [0.11, -0.105], [-0.12, -0.105], [-0.12, 0.105], [0.11, 0.105]])
        world = body @ np.array([[c, s], [-s, c]]) + [x, y]
        ax.fill(world[:, 0], world[:, 1], color=INK, alpha=0.85)
        ax.text(x - half + 0.08, y - half + 0.1, f"Nav2 lap, {k * d['step'][0]:.0f} s",
                fontsize=14, color=INK2)
        fig.canvas.draw()
        frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3]))
        plt.close(fig)
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / (Path(npz_path).stem + ".gif")
    frames[0].save(path, save_all=True, append_images=frames[1:], duration=60, loop=0, optimize=True)
    return path


def main() -> None:
    cmds = {"battery": battery, "drift": drift, "nav": nav}
    if len(sys.argv) >= 2 and sys.argv[1] == "gif" and len(sys.argv) == 3:
        print(gif(sys.argv[2]))
    elif len(sys.argv) == 2 and sys.argv[1] in cmds:
        print(cmds[sys.argv[1]]())
    else:
        raise SystemExit(__doc__)


if __name__ == "__main__":
    main()
