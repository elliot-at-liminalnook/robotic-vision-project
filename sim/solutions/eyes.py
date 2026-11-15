"""A solution's eyes: where the things of one colour are in a picture. A model does this by looking."""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

PALE = ("white", "cream")
HUES = {"red": [(0, 9), (171, 180)], "green": [(40, 88)], "blue": [(98, 132)], "yellow": [(20, 36)], "purple": [(133, 160)]}      # OpenCV hue, 0 to 180


@dataclass
class Blob:
    u: float                   # its middle, in the picture [px]
    v: float
    area: float                # [px^2]
    corners: np.ndarray        # the four corners of the tightest rectangle round it (4 x 2 px)
    ring: bool                 # it is a frame with the surface showing through it (a bin seen from above), not a block or a printed tag
    cut_off: bool = False      # it runs off the picture's edge: its middle here is not the thing's middle
    tag: bool = False          # it is pale with something black inside it: a printed tag, not a thing of that colour
    outline: np.ndarray | None = None      # its outline in the picture (N x 2 px)


def find(image: np.ndarray, colour: str, min_area: float = 120.0) -> list[Blob]:
    """Every patch of that colour in an RGB picture, largest first."""
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    if colour == "white":                                           # lit white, and nothing else: bare steel and pale wood are not
        mask = cv2.inRange(hsv, (0, 0, 215), (180, 18, 255))
    elif colour == "black":                                         # dark, whatever its tint
        mask = cv2.inRange(hsv, (0, 0, 0), (180, 255, 70))
    elif colour == "grey":                                          # colourless and neither white nor black: bare steel, grey paint
        mask = cv2.inRange(hsv, (0, 0, 90), (180, 45, 255)) & ~cv2.inRange(hsv, (0, 0, 215), (180, 18, 255))
    elif colour in PALE:                                            # cream, and whatever else is bright and nearly colourless
        mask = cv2.inRange(hsv, (0, 0, 185), (180, 60, 255))
    for lo, hi in HUES.get(colour, []):
        mask |= cv2.inRange(hsv, (lo, 110, 50), (hi, 255, 255))
    # (specks are cleaned off strong colours; a pale frame may be a few pixels wide, and that would clean it off too)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE if colour == "cream" else cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, tree = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for k, c in enumerate(contours):
        if tree[0][k][3] != -1 or cv2.contourArea(c) < min_area:      # (a hole's own outline is not a thing)
            continue
        m = cv2.moments(c)
        inner = tree[0][k][2]
        holes = []
        while inner != -1:                                          # its largest hole
            holes.append(contours[inner])
            inner = tree[0][inner][0]
        hole = max(holes, key=cv2.contourArea) if holes else None
        ring = hole is not None and cv2.contourArea(hole) > 0.15 * cv2.contourArea(c)
        dark = False
        if ring:                                                    # what shows through it: the surface, or something black (a printed tag)
            inside = np.zeros(mask.shape, np.uint8)
            cv2.drawContours(inside, [hole], -1, 255, -1)
            dark = float(hsv[..., 2][inside > 0].mean()) < 90
        x, y, w, h = cv2.boundingRect(c)
        cut = x <= 1 or y <= 1 or x + w >= mask.shape[1] - 1 or y + h >= mask.shape[0] - 1
        out.append(Blob(m["m10"] / m["m00"], m["m01"] / m["m00"], float(cv2.contourArea(c)), cv2.boxPoints(cv2.minAreaRect(c)),
                        bool(ring and not dark), bool(cut), bool(ring and dark), c.reshape(-1, 2)))
    return sorted(out, key=lambda b: -b.area)
