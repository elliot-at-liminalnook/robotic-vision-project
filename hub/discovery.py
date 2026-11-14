"""Finding out what is plugged in.

The hub is not told which devices exist. It asks each serial port which servo ids answer, and
matches what it finds to the kinds of device it has drivers for. Cameras say only what they are.
"""
from __future__ import annotations

from .bus import BusError, FeetechBus

SCAN_IDS = range(1, 31)
SCAN_TIMEOUT_S = 0.004

# The kinds of device this hub has a driver for, by the servo ids one answers on.
KINDS = {
    "arm": {"ids": (1, 2, 3, 4, 5, 6), "what": "SO-101 arm: five joints and a two-jaw gripper"},
    "rover": {"ids": (11, 12, 21, 22, 23, 24), "what": "rover: two driven wheels and a four-axis camera head"},
}


def scan(port) -> list[int]:
    """The servo ids that answer on a port."""
    bus = FeetechBus(port, reply_timeout_s=SCAN_TIMEOUT_S + float(getattr(port, "reply_time_s", 0.0)), retries=0)
    found = []
    for i in SCAN_IDS:
        try:
            bus.ping(i)
            found.append(i)
        except BusError:
            pass
    return found


def match(ids: list[int]) -> tuple[list[str], list[int]]:
    """(kinds of device whose servos are all there, ids that belong to none of them)."""
    kinds = [k for k, d in KINDS.items() if set(d["ids"]) <= set(ids)]
    used = {i for k in kinds for i in KINDS[k]["ids"]}
    return kinds, [i for i in ids if i not in used]
