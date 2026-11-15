"""The place: what the hub has found of its surroundings. It starts empty and is told nothing.

A grid on the surface the rover stands on, in the rover's frame as it stood when the map was begun
("the map's frame": origin under the middle of its axle, x the way it faced, y to its left). Every
cell is one of three things, and the third matters most:

    seen free       the surface was seen there
    seen occupied   something stands there, or the surface's raised edge (with how high)
    never seen      nothing has looked there: where to look next, and where to be careful

A cell is left as never seen when a view's points in it disagree with there being a surface at all
(a fifth of them well below it): that is what mismatched points look like, and stereo mismatches
whatever is very near the lenses.

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
BELOW_M = 0.012                        # a cell with a fifth of its points this far under the surface is not believed
HIGH_M = 0.010                         # this far above the surface is something standing on it
MET_M = 0.012                          # what stopped the rover's wheels is taken to be at least this high
STEP_M = 0.015                         # neighbouring cells whose heights differ by more than this are different things
EDGE_MIN_M = 0.18                      # a straight thin run of occupied cells at least this long is reported as an edge
MAX_RANGE_M = 1.2                      # points further than this from the camera are not used
UNSEEN, FREE, OCCUPIED = 0, 1, 2


def colour_of(rgb) -> dict:
    """A mean colour as numbers and as a word: what a person would call it, roughly."""
    r, g, b = (float(v) / 255.0 for v in rgb)
    hi, lo = max(r, g, b), min(r, g, b)
    if hi - lo < 0.18 * max(hi, 1e-6) or hi < 0.12:
        word = "black" if hi < 0.2 else "white" if hi > 0.8 else "grey"
    else:
        hue = (60.0 * ((g - b) / (hi - lo)) % 360.0 if hi == r else 60.0 * ((b - r) / (hi - lo)) + 120.0 if hi == g
               else 60.0 * ((r - g) / (hi - lo)) + 240.0)
        word = next(name for top, name in ((18, "red"), (45, "orange"), (70, "yellow"), (165, "green"), (200, "cyan"), (262, "blue"),
                                           (300, "purple"), (340, "pink"), (361, "red")) if hue < top)
        if word in ("orange", "yellow") and hi < 0.45:
            word = "brown"
    return {"colour": word, "rgb": [int(round(v * 255)) for v in (r, g, b)]}


def re_frame(summary: dict, T: np.ndarray, frame: str) -> dict:
    """A place's summary with every position and direction given in another frame: T is the map's frame in that frame
    (4 x 4). Both frames stand on the same surface, so heights do not change."""
    if "surface" not in summary:
        return summary
    yaw = math.degrees(math.atan2(T[1, 0], T[0, 0]))
    pt = lambda p: [round(float(v), 3) for v in (T @ [p[0], p[1], 0.0, 1.0])[:2]]            # noqa: E731
    way = lambda d: [round(float(v), 2) for v in (T[:2, :2] @ np.asarray(d, float))]         # noqa: E731
    out = dict(summary)
    out["frame"] = frame
    out["things"] = [{**t, "at_m": pt(t["at_m"]), "long_axis_deg": round((t["long_axis_deg"] + yaw) % 180.0, 1)} for t in summary["things"]]
    surface = dict(summary["surface"])
    if surface.get("seen"):
        surface["outline_m"] = [pt(p) for p in surface["outline_m"]]
        xs, ys = zip(*surface["outline_m"])
        surface["reaches_m"] = {"x": [min(xs), max(xs)], "y": [min(ys), max(ys)]}
        surface["edges_found"] = [{**e, "from_m": pt(e["from_m"]), "to_m": pt(e["to_m"]),
                                   "direction_deg": round((e["direction_deg"] + yaw) % 180.0, 1),
                                   **({"surface_is_towards": way(e["surface_is_towards"])} if "surface_is_towards" in e else {})}
                                  for e in surface["edges_found"]]
    out["surface"] = surface
    unseen = dict(summary["unseen"])
    if "directions_never_looked_deg" in unseen:
        unseen["note"] = ("directions are from where the rover stood when the map was begun, which is at "
                          f"{pt([0.0, 0.0])} in this frame; 0 = the way it faced then, which is {round(yaw, 1)} deg in this frame; left positive")
    out["unseen"] = unseen
    return out


class Place:
    def __init__(self):
        n = 2 * HALF_CELLS
        self.ground = np.zeros((n, n), np.int32)        # views that saw the surface in each cell
        self.high = np.zeros((n, n), np.int32)          # views that saw something standing in it
        self.height = np.zeros((n, n), np.float32)      # how high the highest thing seen there is
        self.low = np.zeros((n, n), np.float32)         # how high the lowest part seen of what stands there is (0: not known)
        self.rgb = np.zeros((n, n, 3), np.float64)      # the colours of what stands in each cell, summed ...
        self.rgb_n = np.zeros((n, n), np.int64)         # ... over this many points
        self.views, self.used, self.levels = 0, 0, []
        self.rulers: dict = {}

    @property
    def empty(self) -> bool:
        return self.used == 0

    # ---- taking in what was seen -------------------------------------------------
    def add_cloud(self, points: np.ndarray, camera_at: np.ndarray, colours: np.ndarray | None = None) -> dict:
        """Points (N x 3) in the map's frame, seen from camera_at, and (N x 3) what colour each was. Returns what this view added."""
        self.views += 1
        within = np.linalg.norm(points - camera_at, axis=1) < MAX_RANGE_M
        p = points[within]
        colours = None if colours is None else np.asarray(colours)[within]
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
        rgb = None if colours is None else colours[ok][order].astype(np.float64)
        cells, starts, counts = np.unique(flat, return_index=True, return_counts=True)
        added = {"free": 0, "occupied": 0, "doubtful": 0}
        for c, s, n in zip(cells, starts, counts):
            if n < MIN_POINTS:
                continue
            zs = z[s:s + n]
            i, j = divmod(int(c), 2 * HALF_CELLS)
            if np.mean(zs < -BELOW_M) > 0.2:
                added["doubtful"] += 1                       # nothing is seen under the surface: these points are mismatched
                continue                                     # (or it is past the surface's edge: either way, not known)
            if np.percentile(zs, 70) > HIGH_M:
                self.high[i, j] += 1
                self.height[i, j] = max(self.height[i, j], float(np.percentile(zs, 90)))
                foot = float(np.percentile(zs[zs > HIGH_M], 10))
                self.low[i, j] = foot if self.low[i, j] == 0 else min(self.low[i, j], foot)
                added["occupied"] += 1
                if rgb is not None:
                    up = zs > HIGH_M                         # the thing's own surface, not the ground beside it
                    self.rgb[i, j] += rgb[s:s + n][up].sum(axis=0)
                    self.rgb_n[i, j] += int(up.sum())
            elif abs(float(np.median(zs))) < GROUND_BAND_M:
                self.ground[i, j] += 1
                added["free"] += 1
        self.used += 1
        self.levels.append(level)
        return {"used": True, "surface_read_at_mm": round(level * 1000, 1), **added}

    def mark_free(self, xy: np.ndarray):
        """These points (N x 2, map frame) are on bare surface for certain: the rover is standing on them. One count each,
        and nothing there is occupied any more (a mismatch under the rover's own nose, or a thing it has since been told of)."""
        ij = np.floor(np.asarray(xy, float) / CELL_M).astype(int) + HALF_CELLS
        ij = ij[(ij >= 0).all(axis=1) & (ij < 2 * HALF_CELLS).all(axis=1)]
        self.ground[ij[:, 0], ij[:, 1]] = np.maximum(self.ground[ij[:, 0], ij[:, 1]], 1)
        self.high[ij[:, 0], ij[:, 1]] = 0
        self.height[ij[:, 0], ij[:, 1]] = 0.0
        self.low[ij[:, 0], ij[:, 1]] = 0.0

    def mark_met(self, xy: np.ndarray, at_least_m: float = MET_M):
        """Something stopped the rover at these points (N x 2, map frame): occupied, and at least high enough to stop it."""
        ij = np.floor(np.asarray(xy, float) / CELL_M).astype(int) + HALF_CELLS
        ij = ij[(ij >= 0).all(axis=1) & (ij < 2 * HALF_CELLS).all(axis=1)]
        self.high[ij[:, 0], ij[:, 1]] = np.maximum(self.high[ij[:, 0], ij[:, 1]], self.ground[ij[:, 0], ij[:, 1]] + 1)
        self.height[ij[:, 0], ij[:, 1]] = np.maximum(self.height[ij[:, 0], ij[:, 1]], at_least_m)

    # ---- what it adds up to --------------------------------------------------------
    def cells(self) -> np.ndarray:
        out = np.full(self.ground.shape, UNSEEN, np.uint8)
        out[self.ground > 0] = FREE
        out[(self.high > 0) & (self.high >= self.ground)] = OCCUPIED
        return out

    def _xy(self, i, j):
        return ((np.asarray(i) - HALF_CELLS + 0.5) * CELL_M, (np.asarray(j) - HALF_CELLS + 0.5) * CELL_M)

    def _split(self) -> tuple[list[dict], np.ndarray]:
        """The occupied cells as (long straight thin runs, the mask of what is a thing). A raised edge round the surface
        shows as a long straight thin run; so does the face of something long. Which it is shows in what is beside it: an
        edge of the surface has the surface on one side only, and is taken out of the things; a thing's side stays in it."""
        c = self.cells()
        left = ndimage.binary_closing(c == OCCUPIED, structure=np.ones((3, 3)))
        occupied, things, runs = left.copy(), left.copy(), []
        for _ in range(16):
            lines = cv2.HoughLinesP(left.astype(np.uint8) * 255, 1, np.pi / 180, threshold=14,
                                    minLineLength=int(EDGE_MIN_M / CELL_M), maxLineGap=5)
            if lines is None:
                break
            j1, i1, j2, i2 = max(lines.reshape(-1, 4), key=lambda l: (l[2] - l[0]) ** 2 + (l[3] - l[1]) ** 2)   # (x = column j, y = row i)
            # The segment found is often only part of the run (a line a few degrees off the grid is a staircase): follow
            # the run along its own line, both ways, as far as occupied cells go on without a gap.
            ii, jj = np.nonzero(left)
            cells_ji = np.c_[jj, ii].astype(float)
            a0, b0 = np.array([j1, i1], float), np.array([j2, i2], float)
            for width in (3.0, 2.5):
                u = (b0 - a0) / max(float(np.linalg.norm(b0 - a0)), 1e-9)
                rel = cells_ji - a0
                t, off = rel @ u, rel @ np.array([-u[1], u[0]])
                on = np.abs(off) < width
                ts = np.sort(t[on])
                mid = float(np.linalg.norm(b0 - a0)) / 2
                k = int(np.argmin(np.abs(ts - mid)))
                lo = hi = k
                while lo > 0 and ts[lo] - ts[lo - 1] <= 6.0:
                    lo -= 1
                while hi < len(ts) - 1 and ts[hi + 1] - ts[hi] <= 6.0:
                    hi += 1
                pick = cells_ji[on & (t >= ts[lo]) & (t <= ts[hi])]
                if len(pick) < 3:
                    break
                centre = pick.mean(axis=0)
                u = np.linalg.svd(pick - centre, full_matrices=False)[2][0]
                span = (pick - centre) @ u
                a0, b0 = centre + u * span.min(), centre + u * span.max()
            j1, i1, j2, i2 = (int(round(v)) for v in (a0[0], a0[1], b0[0], b0[1]))
            near, wide = (np.zeros(left.shape, np.uint8) for _ in range(2))
            cv2.line(near, (int(j1), int(i1)), (int(j2), int(i2)), 1, 5)
            cv2.line(wide, (int(j1), int(i1)), (int(j2), int(i2)), 1, 15)
            on = occupied & (near > 0)
            if (occupied & (wide > 0)).sum() > 1.7 * on.sum():       # not thin: the side of something broad
                left &= near == 0
                continue
            left &= near == 0
            a, b = np.array(self._xy(i1, j1), float), np.array(self._xy(i2, j2), float)
            along = (b - a) / max(float(np.linalg.norm(b - a)), 1e-9)
            normal = np.array([-along[1], along[0]])
            sides = []
            for sign in (1, -1):                                  # is the surface seen on this side of it?
                pts = [a + along * t + normal * sign * 0.04 for t in np.arange(0.0, float(np.linalg.norm(b - a)), CELL_M)]
                ij = np.floor(np.array(pts) / CELL_M).astype(int) + HALF_CELLS
                ij = ij[(ij >= 0).all(axis=1) & (ij < 2 * HALF_CELLS).all(axis=1)]
                sides.append(float((c[ij[:, 0], ij[:, 1]] == FREE).mean()) if len(ij) else 0.0)
            h = self.height[on]
            one_sided = max(sides) > 0.3 and min(sides) < 0.15
            if one_sided:
                things &= near == 0                              # the surface's edge is not a thing on it; a thing's side is part of it
            inward = normal * (1 if sides[0] >= sides[1] else -1)
            runs.append({"from_m": [round(float(v), 3) for v in a], "to_m": [round(float(v), 3) for v in b],
                         "length_m": round(float(np.linalg.norm(b - a)), 3),
                         "height_m": round(float(np.percentile(h[h > 0], 80)) if (h > 0).any() else 0.0, 3),
                         "direction_deg": round(math.degrees(math.atan2(along[1], along[0])) % 180.0, 1),
                         "kind": "edge of the surface" if one_sided else "straight side of something",
                         **({"surface_is_towards": [round(float(v), 2) for v in inward]} if one_sided else {})})
        return runs, things

    def _by_height(self, occ: np.ndarray) -> tuple[np.ndarray, int]:
        """Occupied cells gathered into things: neighbours belong together if their heights are within STEP_M. Things that
        touch in the map but stand at different heights (a hand hovering over a block, a tall thing beside a low one) are
        told apart; a thing whose top slopes is not."""
        filled = np.where(self.height > 0, self.height, ndimage.maximum_filter(self.height, size=3))   # (cells the closing added
        #                                                                                                take a neighbour's height)
        labels, n = np.zeros(occ.shape, np.int32), 0
        for i0, j0 in zip(*np.nonzero(occ)):
            if labels[i0, j0]:
                continue
            n += 1
            labels[i0, j0] = n
            stack = [(i0, j0)]
            while stack:
                i, j = stack.pop()
                for di in (-1, 0, 1):
                    for dj in (-1, 0, 1):
                        a, b = i + di, j + dj
                        if (di or dj) and 0 <= a < occ.shape[0] and 0 <= b < occ.shape[1] and occ[a, b] and not labels[a, b] \
                                and not abs(filled[a, b] - filled[i, j]) > STEP_M:
                            labels[a, b] = n
                            stack.append((a, b))
        return labels, n

    def things(self) -> list[dict]:
        """Each connected patch of occupied cells that is not a long straight thin run: where, how big, how high."""
        _, occ = self._split()
        labels, n = self._by_height(occ)
        out = []
        for k in range(1, n + 1):
            i, j = np.nonzero(labels == k)
            if len(i) < 4 or (len(i) < 8 and self.high[i, j].max() < 2):
                continue                                         # a speck: a few cells that only one view saw
            x, y = self._xy(i, j)
            pts = np.c_[x, y]
            centre = pts.mean(axis=0)
            _, _, axes = np.linalg.svd(pts - centre, full_matrices=False)
            along, across = (pts - centre) @ axes[0], (pts - centre) @ axes[1]
            h = self.height[i, j]
            seen = int(self.rgb_n[i, j].sum())
            out.append({"at_m": [round(float(centre[0]), 3), round(float(centre[1]), 3)],
                        "length_m": round(float(np.ptp(along) + CELL_M), 3), "width_m": round(float(np.ptp(across) + CELL_M), 3),
                        "height_m": round(float(np.percentile(h[h > 0], 80)) if (h > 0).any() else 0.0, 3),
                        "long_axis_deg": round(math.degrees(math.atan2(axes[0][1], axes[0][0])) % 180.0, 1), "cells": int(len(i)),
                        **({"lowest_seen_m": round(float(np.percentile(self.low[i, j][self.low[i, j] > 0], 25)), 3)}
                           if (self.low[i, j] > 0).any() else {}),
                        **(colour_of(self.rgb[i, j].sum(axis=0) / seen) if seen >= 30 else {})})
        return sorted(out, key=lambda t: -t["cells"])

    def surface(self) -> dict:
        """The surface as far as it has been seen: its area, how far it reaches, an outline, and the straight edges found."""
        c = self.cells()
        seen = ndimage.binary_fill_holes(ndimage.binary_closing(c != UNSEEN, structure=np.ones((5, 5))))
        i, j = np.nonzero(seen)
        if len(i) == 0:
            return {"seen": False}
        x, y = self._xy(i, j)
        contours, _ = cv2.findContours(seen.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big = max(contours, key=cv2.contourArea)
        poly = cv2.approxPolyDP(big, 3.0, True).reshape(-1, 2)   # (points are (column j, row i))
        outline = [[round(float(v), 3) for v in self._xy(pi, pj)] for pj, pi in poly]
        runs, _ = self._split()
        return {"seen": True, "area_seen_m2": round(float(seen.sum()) * CELL_M ** 2, 4),
                "reaches_m": {"x": [round(float(x.min()), 3), round(float(x.max()), 3)], "y": [round(float(y.min()), 3), round(float(y.max()), 3)]},
                "outline_m": outline,
                "edges_found": runs,
                "note": "area_seen is what has been seen so far, edges included; where no edge was found the surface may go on. "
                        "An 'edge of the surface' has the surface seen on one side of it only: a long one is the surface's "
                        "raised edge; a short one may be the hidden side of something standing on it"}

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
                "things": things,
                "unseen": self.unseen(),
                "rulers": self.rulers,
                "about_things": "height_m is a thing's top; lowest_seen_m is the lowest part of it the cameras saw: near the surface for a "
                                "thing standing on it, and the height of whatever it stands on for a thing that stands on something raised",
                "note": "from the rover's two cameras, out to about 0.7 m from where it stood: distances from there are good to "
                        "about 5 % (the pair is not yet calibrated), heights to about 5 mm; a thing's far side and what stands "
                        "behind it are not seen"}

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
