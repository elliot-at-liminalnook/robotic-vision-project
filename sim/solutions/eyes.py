"""A solution's eyes: where the things of one colour are in a picture. A model does this by looking."""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

HUES = {"red": [(0, 9), (171, 180)], "green": [(40, 88)], "blue": [(98, 132)], "yellow": [(20, 36)]}      # OpenCV hue, 0 to 180


@dataclass
class Blob:
    u: float                   # its middle, in the picture [px]
    v: float
    area: float                # [px^2]
    corners: np.ndarray        # the four corners of the tightest rectangle round it (4 x 2 px)
    ring: bool                 # it has a hole in it (a bin seen from above, not a block)


def find(image: np.ndarray, colour: str, min_area: float = 120.0) -> list[Blob]:
    """Every patch of that colour in an RGB picture, largest first."""
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in HUES[colour]:
        mask |= cv2.inRange(hsv, (lo, 110, 50), (hi, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, tree = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for k, c in enumerate(contours):
        if tree[0][k][3] != -1 or cv2.contourArea(c) < min_area:      # (a hole's own outline is not a thing)
            continue
        m = cv2.moments(c)
        hole = tree[0][k][2] != -1 and cv2.contourArea(contours[tree[0][k][2]]) > 0.15 * cv2.contourArea(c)
        out.append(Blob(m["m10"] / m["m00"], m["m01"] / m["m00"], float(cv2.contourArea(c)), cv2.boxPoints(cv2.minAreaRect(c)), bool(hole)))
    return sorted(out, key=lambda b: -b.area)
