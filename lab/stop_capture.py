#!/usr/bin/env python3
"""Save an overhead phone frame (and the tape tracker reading) at every stop of a run.

Runs on the Mac next to a test that prints one line per stop, for example
tools/battery_drive_test.py ("  cycle N: ..." after each return to the start, and
"... rest ..." at each rest). Standard library only:
    .venv/bin/python lab/stop_capture.py datasets/battery/drive_run.log datasets/battery/frames/<run>

Writes <out>/stops.csv (unix_time,event,cycle,frame,front_in,tracker_time) and one JPEG per
stop. The frames show the room, so they stay local (datasets/ is git-ignored).
Camera and tracker addresses come from config/lab.json ("phone_url", "tape_url").
"""
from __future__ import annotations

import json
import re
import sys
import time
import urllib.request
from pathlib import Path

POLL_S = 0.2
CYCLE = re.compile(r"^\s+cycle (\d+):")
REST = re.compile(r"\brest \d")
STOPPED = re.compile(r"^stopped:")


def fetch(url: str, timeout: float = 3.0) -> bytes | None:
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.read()
    except OSError:
        return None


def capture(cfg: dict, out: Path, event: str, cycle: int, index: int) -> str:
    t = time.time()
    frame = fetch(cfg["phone_url"].rstrip("/") + "/shot.jpg")
    name = ""
    if frame:
        name = f"{index:05d}_{event}_{cycle:04d}.jpg"
        (out / name).write_bytes(frame)
    reading = fetch(cfg["tape_url"].rstrip("/") + "/reading")
    rd = json.loads(reading) if reading else {}
    front = rd.get("front_in", "") if rd else ""
    return f"{t:.2f},{event},{cycle},{name},{front},{rd.get('unix_time', '') if rd else ''}\n"


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    log, out = Path(sys.argv[1]), Path(sys.argv[2])
    cfg = json.loads(Path("config/lab.json").read_text())
    if "phone_url" not in cfg or "tape_url" not in cfg:
        raise SystemExit("config/lab.json needs phone_url and tape_url")
    out.mkdir(parents=True, exist_ok=True)
    stops = out / "stops.csv"
    if not stops.exists():
        stops.write_text("unix_time,event,cycle,frame,front_in,tracker_time\n")
    seen = len(log.read_text().splitlines()) if log.exists() else 0
    index = sum(1 for _ in out.glob("*.jpg"))
    print(f"watching {log} from line {seen}, saving to {out}", flush=True)
    while True:
        lines = log.read_text().splitlines() if log.exists() else []
        if len(lines) < seen:          # log was restarted
            seen = 0
        for line in lines[seen:]:
            if m := CYCLE.match(line):
                event, cycle = "start", int(m.group(1))
            elif REST.search(line):
                event, cycle = "rest", -1
            elif STOPPED.match(line):
                print("test stopped; done", flush=True)
                return
            else:
                continue
            with stops.open("a") as f:
                f.write(capture(cfg, out, event, cycle, index))
            index += 1
        seen = len(lines)
        time.sleep(POLL_S)


if __name__ == "__main__":
    main()
