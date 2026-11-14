"""The place: what the hub has found of its surroundings. It starts empty and is told nothing.

A grid on the surface the rover stands on, in the rover's frame as it stood when the map was begun
("the map's frame": origin under the middle of its axle, x the way it faced, y to its left). Every
cell is one of three things, and the third matters most:

    seen free       the surface was seen there
    seen occupied   something stands there, or the surface's raised edge (with how high)
    never seen      nothing has looked there: where to look next, and where to be careful

Depth comes from the rover's two cameras (hub/vision/stereo.py). Each cloud is first levelled by the
ground it contains: the rover knows it stands on a surface, and what it sees of that surface near
itself is where zero height is, not what its head's joints say (they sag a few millimetres).
"""
from __future__ import annotations

import base64
import math

import cv2
import numpy as np
from scipy import ndimage

CELL_M = 0.01
HALF_CELLS = 150                       # the grid reaches this many cells each way from the origin (1.5 m)
MIN_POINTS = 12                        # points in a cell before one view says anything about it
GROUND_BAND_M = 0.006                  # the surface, give or take
HIGH_M = 0.010                         # this far above the surface is something standing on it
MAX_RANGE_M = 1.2                      # points further than this from the camera are not used
UNSEEN, FREE, OCCUPIED = 0, 1, 2


class Place:
    def __init__(self):
        n = 2 * HALF_CELLS
        self.ground = np.zeros((n, n), np.int32)        # views that saw the surface in each cell
        self.high = np.zeros((n, n), np.int32)          # views that saw something standing in it
        self.height = np.zeros((n, n), np.float32)      # how high the highest thing seen there is
        self.views, self.used, self.levels = 0, 0, []
        self.rulers: dict = {}

    @property
    def empty(self) -> bool:
        return self.used == 0

    # ---- taking in what was seen -------------------------------------------------
    def add_cloud(self, points: np.ndarray, camera_at: np.ndarray) -> dict:
        """Points (N x 3) in the map's frame, seen from camera_at. Returns what this view added."""
        self.views += 1
        p = points[np.linalg.norm(points - camera_at, axis=1) < MAX_RANGE_M]
        near = p[(np.abs(p[:, 2]) < 0.02) & (np.linalg.norm(p[:, :2] - camera_at[:2], axis=1) < 0.6)]
        if len(near) < 500:
            return {"used": False, "why": "too little of the surface in view to level the cloud by"}
        level = float(np.median(near[:, 2]))                 # where this cloud puts the surface: that is height zero
        p = p - [0.0, 0.0, level]
        ij = np.floor(p[:, :2] / CELL_M).astype(int) + HALF_CELLS
        ok = (ij >= 0).all(axis=1) & (ij < 2 * HALF_CELLS).all(axis=1)
        ij, z = ij[ok], p[ok, 2]
        flat = ij[:, 0] * (2 * HALF_CELLS) + ij[:, 1]
        order = np.argsort(flat)
        flat, z = flat[order], z[order]
        cells, starts, counts = np.unique(flat, return_index=True, return_counts=True)
        added = {"free": 0, "occupied": 0}
        for c, s, n in zip(cells, starts, counts):
            if n < MIN_POINTS:
                continue
            zs = z[s:s + n]
            i, j = divmod(int(c), 2 * HALF_CELLS)
            if np.percentile(zs, 70) > HIGH_M:
                self.high[i, j] += 1
                self.height[i, j] = max(self.height[i, j], float(np.percentile(zs, 90)))
                added["occupied"] += 1
            elif abs(float(np.median(zs))) < GROUND_BAND_M:
                self.ground[i, j] += 1
                added["free"] += 1
        self.used += 1
        self.levels.append(level)
        return {"used": True, "surface_read_at_mm": round(level * 1000, 1), **added}

    # ---- what it adds up to --------------------------------------------------------
    def cells(self) -> np.ndarray:
        out = np.full(self.ground.shape, UNSEEN, np.uint8)
        out[self.ground > 0] = FREE
        out[(self.high > 0) & (self.high >= self.ground)] = OCCUPIED
        return out

    def _xy(self, i, j):
        return ((np.asarray(i) - HALF_CELLS + 0.5) * CELL_M, (np.asarray(j) - HALF_CELLS + 0.5) * CELL_M)

    def things(self) -> list[dict]:
        """Each connected patch of occupied cells: where, how big, how high, and whether it looks like an edge of the surface."""
        occ = ndimage.binary_closing(self.cells() == OCCUPIED, structure=np.ones((3, 3)))
        labels, n = ndimage.label(occ, structure=np.ones((3, 3)))
        out = []
        for k in range(1, n + 1):
            i, j = np.nonzero(labels == k)
            if len(i) < 4:
                continue                                         # a speck: one view's noise
            x, y = self._xy(i, j)
            pts = np.c_[x, y]
            centre = pts.mean(axis=0)
            _, _, axes = np.linalg.svd(pts - centre, full_matrices=False)
            along, across = (pts - centre) @ axes[0], (pts - centre) @ axes[1]
            length, width = float(np.ptp(along) + CELL_M), float(np.ptp(across) + CELL_M)
            h = self.height[i, j]
            height = float(np.percentile(h[h > 0], 80)) if (h > 0).any() else 0.0
            edge = length > 0.20 and width < 0.06 and height < 0.045
            out.append({"at_m": [round(float(centre[0]), 3), round(float(centre[1]), 3)], "length_m": round(length, 3),
                        "width_m": round(width, 3), "height_m": round(height, 3),
                        "long_axis_deg": round(math.degrees(math.atan2(axes[0][1], axes[0][0])) % 180.0, 1),
                        "kind": "edge of the surface" if edge else "thing", "cells": int(len(i))})
        return sorted(out, key=lambda t: -t["cells"])

    def surface(self) -> dict:
        """The surface as far as it has been seen: its area, how far it reaches, and an outline."""
        c = self.cells()
        things = self.things()
        inside = c == FREE
        for t in things:                                         # a thing stands on the surface; an edge is its boundary
            if t["kind"] == "thing":
                pass
        seen = ndimage.binary_closing(inside | (c == OCCUPIED), structure=np.ones((5, 5)))
        seen = ndimage.binary_fill_holes(seen)
        edges = np.zeros_like(seen)
        labels, n = ndimage.label(ndimage.binary_closing(c == OCCUPIED, structure=np.ones((3, 3))), structure=np.ones((3, 3)))
        for k, t in zip(range(1, n + 1), [None] * n):
            pass
        i, j = np.nonzero(seen)
        if len(i) == 0:
            return {"seen": False}
        x, y = self._xy(i, j)
        contours, _ = cv2.findContours(seen.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big = max(contours, key=cv2.contourArea)
        poly = cv2.approxPolyDP(big, 3.0, True).reshape(-1, 2)   # (points are (column j, row i))
        outline = [[round(float(v), 3) for v in self._xy(pi, pj)] for pj, pi in poly]
        return {"seen": True, "area_seen_m2": round(float(seen.sum()) * CELL_M ** 2, 4),
                "reaches_m": {"x": [round(float(x.min()), 3), round(float(x.max()), 3)], "y": [round(float(y.min()), 3), round(float(y.max()), 3)]},
                "outline_m": outline,
                "edges_found": [t for t in things if t["kind"] == "edge of the surface"],
                "note": "area_seen is what has been seen so far, edges included; where no edge was found the surface may go on"}

    def unseen(self) -> dict:
        """Where nothing has looked: inside the box the seen cells fill, and which ways from the map's origin."""
        c = self.cells()
        i, j = np.nonzero(c != UNSEEN)
        if len(i) == 0:
            return {"share_of_the_box_around_what_was_seen": 1.0, "directions_deg": [[-180, 180]]}
        box = c[i.min():i.max() + 1, j.min():j.max() + 1]
        x, y = self._xy(i, j)
        bearing = np.degrees(np.arctan2(y, x))
        seen_dirs = np.zeros(36, bool)
        seen_dirs[((bearing + 180) // 10).astype(int) % 36] = True
        gaps, k = [], 0
        while k < 36:
            if not seen_dirs[k]:
                start = k
                while k < 36 and not seen_dirs[k]:
                    k += 1
                gaps.append([start * 10 - 180, k * 10 - 180])
            else:
                k += 1
        return {"share_of_the_box_around_what_was_seen": round(float((box == UNSEEN).mean()), 3),
                "directions_never_looked_deg": gaps,
                "note": "directions are from the map's origin, 0 = the way the rover faced when the map was begun, left positive"}

    def summary(self) -> dict:
        if self.empty:
            return {"found": "nothing yet", "how": "POST /survey: the rover looks all round from where it stands"}
        things = self.things()
        return {"frame": "the map's frame: the rover's own frame as it stood when the map was begun (x the way it faced, y to its "
                         "left, z up from the surface, metres). GET /where?of=rover&relative_to=... relates it to other devices",
                "cell_m": CELL_M, "views": {"taken": self.views, "used": self.used},
                "surface": self.surface(),
                "things": [t for t in things if t["kind"] == "thing"],
                "unseen": self.unseen(),
                "rulers": self.rulers,
                "note": "from the rover's two cameras: distances are good to about 5 %, heights to about 5 mm; a thing's far side "
                        "and what stands behind it are not seen"}

    def picture(self, scale: int = 3) -> bytes:
        """The grid as a PNG: grey = never seen, light = free, dark = occupied (brighter red = higher). The rover began at the centre, facing right."""
        c = self.cells()
        img = np.full((*c.shape, 3), 110, np.uint8)
        img[c == FREE] = (225, 215, 190)
        occ = c == OCCUPIED
        img[occ] = np.stack([np.clip(80 + self.height[occ] * 1800, 80, 255), np.full(occ.sum(), 40), np.full(occ.sum(), 40)], axis=1).astype(np.uint8)
        img = np.rot90(img)                                       # x to the right, y up
        img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        ok, buf = cv2.imencode(".png", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        return buf.tobytes()

    def picture_b64(self) -> str:
        return base64.b64encode(self.picture()).decode()
