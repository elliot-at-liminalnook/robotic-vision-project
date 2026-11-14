"""What each kind of device IS: the knowledge that ships with its driver.

Built-in knowledge is about a device, never about a place: the same wherever the device is put
down (docs/MULTI_DEVICE_PLAN.md, 5.3). It is a starting value, to be checked against what is seen.

  tags      where a device's printed tags are on its own body          hardware/tags/layout.toml
  lenses    a camera kind's nominal lens values, from its datasheet    LENSES below
"""
from __future__ import annotations

import tomllib
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np

from . import geometry as G

HARDWARE = Path(__file__).resolve().parents[1] / "hardware"


@dataclass(frozen=True)
class Tag:
    id: int
    size_m: float               # side of the black square
    on: str                     # the part of the device it is fixed to
    T_part_tag: np.ndarray      # the tag's frame (x right, y up, z out of its face) in that part's frame

    def corners(self) -> np.ndarray:
        """Its four corners in the part's frame, in the order a detector reports them:
        top-left, top-right, bottom-right, bottom-left of the artwork."""
        s = self.size_m / 2
        local = np.array([[-s, s, 0, 1], [s, s, 0, 1], [s, -s, 0, 1], [-s, -s, 0, 1]], float)
        return (self.T_part_tag @ local.T).T[:, :3]


@lru_cache(maxsize=None)
def _layout() -> dict:
    with (HARDWARE / "tags" / "layout.toml").open("rb") as f:
        return tomllib.load(f)


def tag_family() -> str:
    return _layout()["family"]


def tags(kind: str) -> dict[int, Tag]:
    """The tags on a kind of device, by id."""
    out = {}
    for t in _layout()["devices"][kind]["tags"]:
        T = G.pose(G.from_axes(t["normal"], t["up"]), t["at_m"])
        out[int(t["id"])] = Tag(int(t["id"]), float(t["size_m"]), t["on"], T)
    return out


def kind_of_tag(tag_id: int) -> str | None:
    """Which kind of device carries this tag id, if any."""
    for kind, d in _layout()["devices"].items():
        if any(int(t["id"]) == tag_id for t in d["tags"]):
            return kind
    return None


@dataclass(frozen=True)
class Lens:
    """A camera kind's nominal pinhole values at one picture size, with OpenCV's distortion terms."""
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    dist: tuple = (0.0, 0.0, 0.0, 0.0, 0.0)             # k1, k2, p1, p2, k3

    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])

    def at(self, width: int, height: int) -> "Lens":
        """The same lens for a picture scaled to another size (same field of view)."""
        sx, sy = width / self.width, height / self.height           # (cx, cy are pixel-index coordinates)
        return Lens(width, height, self.fx * sx, self.fy * sy, (self.cx + 0.5) * sx - 0.5, (self.cy + 0.5) * sy - 0.5, self.dist)


# Raspberry Pi Camera Module 3 Wide, 1920x1080 mode, from its datasheet alone:
#   f = 2.75 mm over 2.8 um binned pixels -> 982.1 px; centre = the middle of the picture;
#   barrel distortion from the stated field of view: 102 deg across the full sensor where a pinhole of
#   this focal length gives 99 deg -> k1 = -0.033 (one term).
# One unit differs from the next, so these are a starting point the hub corrects from what it sees.
LENSES = {"cm3_wide": Lens(1920, 1080, 982.1, 982.1, 959.5, 539.5, (-0.033, 0.0, 0.0, 0.0, 0.0))}
LENS_FOCAL_TOLERANCE = 0.015         # how far one unit's focal length may be from the datasheet's, as a fraction (one sigma).
#                                      A distance measured through an uncalibrated lens is off by this much of itself.
LENSES["pinhole"] = Lens(1920, 1080, 982.1, 982.1, 959.5, 539.5)      # no distortion at all: for comparison only
