#!/usr/bin/env python3
"""Decode raw IR captures from the onboard receiver (Sharp protocol for now).

    sudo timeout 30 ir-ctl -d /dev/lirc0 -r --mode2 > ir_raw.txt
    python3 ir_decode.py ir_raw.txt          (or pipe the capture in on stdin)

The receiver on a GPIO pin times edges by interrupt, which shortens pulses by about 30 us
and stretches gaps by 50-140 us (measured 2026-11-16). That puts Sharp's "1" gaps outside the
kernel decoder's tolerance, so the kernel decodes nothing; these wide thresholds don't care.
The Roomba Home Base's beacons use their own codes and will get their own decoder here.
"""
from __future__ import annotations

import re
import sys
from collections import Counter

FRAME_GAP_US = 10_000      # longer spaces end a frame
ONE_SPACE_US = 1_200       # Sharp: 680 us = 0, 1680 us = 1
MIN_SPACE_US = 400
SHARP_BITS = 15            # 5 address, 8 command, expansion, check (LSB first)


def frames(text: str) -> list[list[int]]:
    """15-bit frames from ir-ctl mode2 text, judged by the space after each pulse."""
    out: list[list[int]] = []
    bits: list[int] = []
    for kind, value in re.findall(r"(pulse|space|timeout) (\d+)", text):
        us = int(value)
        if kind == "pulse":
            continue
        if kind == "timeout" or us > FRAME_GAP_US:
            if len(bits) == SHARP_BITS:
                out.append(bits)
            bits = []
        elif us > ONE_SPACE_US:
            bits.append(1)
        elif us > MIN_SPACE_US:
            bits.append(0)
    if len(bits) == SHARP_BITS:
        out.append(bits)
    return out


def sharp(bits: list[int]) -> tuple[int, int, int, int]:
    """(address, command, expansion, check). Each key is sent twice, the second copy with the
    command, expansion and check bits inverted."""
    def value(chunk: list[int]) -> int:
        return sum(bit << i for i, bit in enumerate(chunk))
    return value(bits[0:5]), value(bits[5:13]), bits[13], bits[14]


def main() -> int:
    text = open(sys.argv[1]).read() if len(sys.argv) > 1 else sys.stdin.read()
    found = frames(text)
    print(f"{len(found)} complete {SHARP_BITS}-bit frames")
    for (addr, cmd, exp, chk), n in Counter(sharp(f) for f in found).most_common():
        kind = "inverted copy" if exp == 0 else "key"
        print(f"  address {addr:2d}  command 0x{cmd:02x}  ({kind})  x{n}")
    return 0 if found else 1


if __name__ == "__main__":
    raise SystemExit(main())
