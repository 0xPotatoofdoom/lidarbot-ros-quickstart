#!/usr/bin/env python3
"""Print-ready PDF of every marker and pattern the lab uses (runs in .venv-hf).

    .venv-hf/bin/python tools/print_sheets.py [out.pdf]     # default print/lidarbot_print_sheets.pdf

Pages fit both US Letter and A4. Print at 100% ("actual size", never "fit to page") on matte
paper, then check the 100 mm scale bar on each page with a ruler before using any marker:
the trackers convert pixels to meters using these exact sizes.

All markers are ArUco DICT_4X4_50. IDs are reserved by use so nothing collides:
    0        robot top plate (overhead tracker)           60 mm   lab/overhead_tracker.py
    1-4      floor calibration (overhead tracker)         80 mm   lab/lab.json floor_markers
    10-13    wall target board (camera drift check)       80 mm   tools/wall_target.py
    20-23    servo test bench (horn discs)                35 mm   TO-DO builds: servo bench
plus a 10 x 7 checkerboard (25 mm squares, 9 x 6 inner corners) for camera intrinsics.
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from wall_target import GAP_MM, MARKER_MM, marker_bitmap, marker_centers  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
PAGE_W, PAGE_H = 210.0, 279.0          # mm: the overlap of A4 and US Letter
INK, INK2, CUT = "#0b0b0b", "#52514e", "#9a9893"
ROBOT = (0, 60.0)
FLOOR = ([1, 2, 3, 4], 80.0)
SERVO = ([20, 21, 22, 23], 35.0)
SQUARE_MM, SQUARES = 25.0, (10, 7)


def new_page():
    fig = plt.figure(figsize=(PAGE_W / 25.4, PAGE_H / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, PAGE_W)
    ax.set_ylim(PAGE_H, 0)              # mm, origin top-left, y down
    ax.set_aspect("equal")
    ax.axis("off")
    return fig, ax


def header(ax, title: str, lines: list[str], page: int) -> None:
    ax.text(15, 16, title, fontsize=15, weight="bold", color=INK)
    for i, line in enumerate(lines):
        ax.text(15, 24 + i * 5.2, line, fontsize=8.5, color=INK2)
    # scale check bar: 100 mm with 10 mm ticks
    x0, y = 15, PAGE_H - 18
    ax.plot([x0, x0 + 100], [y, y], color=INK, linewidth=1)
    for k in range(11):
        h = 3 if k % 5 == 0 else 1.6
        ax.plot([x0 + 10 * k, x0 + 10 * k], [y, y - h], color=INK, linewidth=0.8)
    ax.text(x0 + 104, y, "must measure 100 mm. If not, reprint at 100% / actual size",
            fontsize=7.5, color=INK2, va="center")
    ax.text(PAGE_W - 15, PAGE_H - 8, f"lidarbot print sheets, page {page}  |  ArUco DICT_4X4_50",
            fontsize=7, color=INK2, ha="right")


def marker(ax, marker_id: int, size: float, x: float, y: float, label: str = "", cut: float = 8.0) -> None:
    """Marker with its top-left corner at (x, y) mm, a dashed cut line `cut` mm outside."""
    img = marker_bitmap(marker_id, 600)
    ax.imshow(img, cmap="gray", vmin=0, vmax=255, extent=(x, x + size, y + size, y),
              interpolation="nearest")
    c = cut
    ax.plot([x - c, x + size + c, x + size + c, x - c, x - c], [y - c, y - c, y + size + c, y + size + c, y - c],
            color=CUT, linewidth=0.5, linestyle=(0, (3, 3)))
    ax.text(x + size / 2, y + size + c + 4, label or f"ID {marker_id}  -  {size:.0f} mm",
            fontsize=8, color=INK, ha="center")
    # orientation tick: this edge faces the robot's front / the board's top
    ax.annotate("", (x + size / 2, y - c + 0.5), (x + size / 2, y - c - 4),
                arrowprops={"arrowstyle": "-|>", "color": INK2, "lw": 0.6})


def cover(pdf) -> None:
    fig, ax = new_page()
    header(ax, "Lidarbot print sheets", [
        "Print every page at 100% (\"actual size\"; never \"fit to page\"), on matte paper or matte card.",
        "Glossy paper reflects the overhead lights and the camera loses the marker.",
        "Check the 100 mm bar at the bottom of each page with a ruler before using anything.",
        "Keep the white border around each marker (cut on the dashed line, not at the black edge).",
        "Mount markers flat: on card or foam board, no bubbles or curl.",
    ], 1)
    rows = [("Page", "What", "Size", "Use"),
            ("2", "Robot marker ID 0 (+ spare)", "60 mm", "robot top plate, arrow to the front"),
            ("3-4", "Floor markers IDs 1-4", "80 mm", "tape to the floor at measured spots"),
            ("5", "Wall target, IDs 10-13", "180 mm board", "wall ahead of the tape start"),
            ("6", "Servo bench markers IDs 20-23", "35 mm", "on a disc or arm on each servo horn"),
            ("7", "Checkerboard 10 x 7", "25 mm squares", "camera intrinsics")]
    for i, row in enumerate(rows):
        y = 62 + i * 9
        weight = "bold" if i == 0 else "normal"
        for x, text in zip((15, 30, 98, 125), row, strict=True):
            ax.text(x, y, text, fontsize=8.5, color=INK, weight=weight)
    ax.text(15, 125, "After printing, measure and write down the real black-square size of each marker type.\n"
                     "If it differs from the nominal size by more than 0.5 mm, put the measured value in the config:\n"
                     "lab/lab.json (robot_marker.size_m, floor_marker_size_m) or tools/wall_target.py (MARKER_MM).",
            fontsize=8.5, color=INK, linespacing=1.6)
    pdf.savefig(fig)
    plt.close(fig)


def robot_page(pdf) -> None:
    fig, ax = new_page()
    mid, size = ROBOT
    header(ax, "Robot marker (overhead tracker)", [
        "Tape one to the robot's top plate, flat, with the arrow pointing toward the robot's front.",
        "Its height above the floor goes in lab.json (robot_marker.height_m). The second copy is a spare.",
    ], 2)
    marker(ax, mid, size, 30, 55, f"ID {mid}  -  {size:.0f} mm  (robot)")
    marker(ax, mid, size, 120, 55, f"ID {mid}  -  {size:.0f} mm  (spare)")
    pdf.savefig(fig)
    plt.close(fig)


def floor_pages(pdf) -> None:
    ids, size = FLOOR
    for n, pair in enumerate((ids[:2], ids[2:])):
        fig, ax = new_page()
        header(ax, f"Floor markers {pair[0]} and {pair[1]} (overhead tracker)", [
            "Tape flat to the floor at tape-measured positions around the test area (lab.json floor_markers).",
            "Measure each marker's center from the tape start. Arrow points along the tape (+x).",
        ], 3 + n)
        marker(ax, pair[0], size, (PAGE_W - size) / 2, 45)
        marker(ax, pair[1], size, (PAGE_W - size) / 2, 150)
        pdf.savefig(fig)
        plt.close(fig)


def wall_page(pdf) -> None:
    fig, ax = new_page()
    header(ax, "Wall target (camera drift check)", [
        "Mount flat on the wall ahead of the tape start, centered at the robot camera's height, arrow up.",
        f"Four {MARKER_MM:.0f} mm markers with {GAP_MM:.0f} mm gaps; tools/wall_target.py expects exactly this layout.",
    ], 5)
    board = 2 * MARKER_MM + GAP_MM
    cx, cy = PAGE_W / 2, 60 + board / 2
    for mid, (mx, my) in marker_centers().items():
        img = marker_bitmap(mid, 600)
        x, y = cx + mx - MARKER_MM / 2, cy + my - MARKER_MM / 2
        ax.imshow(img, cmap="gray", vmin=0, vmax=255, extent=(x, x + MARKER_MM, y + MARKER_MM, y),
                  interpolation="nearest")
        ax.text(x + MARKER_MM / 2, y + MARKER_MM / 2, "", fontsize=6)
    c = 4      # the 180 mm board leaves only 15 mm of page on each side
    x0, y0 = cx - board / 2, cy - board / 2
    ax.plot([x0 - c, x0 + board + c, x0 + board + c, x0 - c, x0 - c],
            [y0 - c, y0 - c, y0 + board + c, y0 + board + c, y0 - c], color=CUT, linewidth=0.5,
            linestyle=(0, (3, 3)))
    ax.annotate("", (cx, y0 - c + 0.5), (cx, y0 - c - 5), arrowprops={"arrowstyle": "-|>", "color": INK2, "lw": 0.6})
    ax.text(cx, y0 + board + c + 5, "IDs 10 (top left), 11 (top right), 12 (bottom left), 13 (bottom right)",
            fontsize=8, color=INK, ha="center")
    pdf.savefig(fig)
    plt.close(fig)


def servo_page(pdf) -> None:
    fig, ax = new_page()
    ids, size = SERVO
    header(ax, "Servo test bench markers", [
        "Glue one to a light disc or arm on each servo horn, centered on the output shaft, flat.",
        "One marker per servo under test; the overhead camera reads its angle.",
    ], 6)
    for k, mid in enumerate(ids):
        marker(ax, mid, size, 35 + (k % 2) * 85, 50 + (k // 2) * 80, cut=6)
    pdf.savefig(fig)
    plt.close(fig)


def checkerboard_page(pdf) -> None:
    fig, ax = new_page()
    cols, rows = SQUARES
    header(ax, "Camera calibration checkerboard", [
        f"{cols} x {rows} squares of {SQUARE_MM:.0f} mm ({cols - 1} x {rows - 1} inner corners). Mount on stiff board,",
        "perfectly flat. Show it to the camera at many angles and distances (OpenCV calibrateCamera).",
    ], 7)
    board = np.indices((rows, cols)).sum(axis=0) % 2 == 0
    x0 = (PAGE_W - cols * SQUARE_MM) / 2
    y0 = 55
    ax.imshow(np.where(board, 0, 255), cmap="gray", vmin=0, vmax=255,
              extent=(x0, x0 + cols * SQUARE_MM, y0 + rows * SQUARE_MM, y0), interpolation="nearest")
    ax.plot([x0, x0 + cols * SQUARE_MM, x0 + cols * SQUARE_MM, x0, x0],
            [y0, y0, y0 + rows * SQUARE_MM, y0 + rows * SQUARE_MM, y0], color=INK, linewidth=0.4)
    pdf.savefig(fig)
    plt.close(fig)


def main() -> None:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "print" / "lidarbot_print_sheets.pdf"
    out.parent.mkdir(parents=True, exist_ok=True)
    with PdfPages(out) as pdf:
        cover(pdf)
        robot_page(pdf)
        floor_pages(pdf)
        wall_page(pdf)
        servo_page(pdf)
        checkerboard_page(pdf)
    print(out)


if __name__ == "__main__":
    main()
