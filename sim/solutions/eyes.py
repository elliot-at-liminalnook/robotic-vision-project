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


def lit_surface(image: np.ndarray, samples, hue: int = 6, sat: int = 40, value: float = 0.7) -> np.ndarray | None:
    """The patch of an RGB picture that is the surface where the light falls on it: coloured like the picture is at
    `samples` (pixels (u, v) taken to be on the surface), and all in one piece. A mask, or None if the samples do not
    agree on a colour. Shadowed surface is left out, and so is a raised edge of the same stuff: both are darker."""
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV).astype(np.int32)
    h, w = hsv.shape[:2]
    seen = [np.median(hsv[max(v - 15, 0):v + 16, max(u - 15, 0):u + 16].reshape(-1, 3), axis=0)
            for u, v in ((int(round(u)), int(round(v))) for u, v in samples) if 0 <= u < w and 0 <= v < h]
    # the samples that agree with each other (one may have landed on a thing, or off the surface)
    like = lambda a, b: abs((a[0] - b[0] + 90) % 180 - 90) <= hue and abs(a[1] - b[1]) <= sat           # noqa: E731
    groups = [[b for b in seen if like(a, b)] for a in seen]
    if not groups or len(max(groups, key=len)) < 2:
        return None
    agreed = np.array(max(groups, key=len))
    h0, s0 = np.median(agreed[:, :2], axis=0)
    v0 = agreed[:, 2].max()                                         # (the brightest of them: a sample may lie in shadow)
    mask = ((np.abs((hsv[..., 0] - h0 + 90) % 180 - 90) <= hue) & (np.abs(hsv[..., 1] - s0) <= sat) & (hsv[..., 2] >= value * v0)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))          # (its grain)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    if n < 2:
        return None
    return (labels == 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))).astype(np.uint8) * 255


def straight_sides(mask: np.ndarray, min_px: float = 150.0) -> list:
    """The straight stretches of a patch's outline, as pairs of pixels (2 x 2 each), leaving out those that run along the
    picture's own edge (where the patch was cut off, not where it ends)."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return []
    poly = cv2.approxPolyDP(max(contours, key=cv2.contourArea), 8.0, True).reshape(-1, 2).astype(float)
    h, w = mask.shape
    out = []
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        along_edge = ((a[0] < 4 and b[0] < 4) or (a[1] < 4 and b[1] < 4) or (a[0] > w - 5 and b[0] > w - 5) or (a[1] > h - 5 and b[1] > h - 5))
        if np.linalg.norm(b - a) >= min_px and not along_edge:
            out.append(np.array([a, b]))
    return out
