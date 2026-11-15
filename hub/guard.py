"""What the map says about a motion before it is made (plan 5.9): for the rover's wheels and the arm's hand.

The hub is told nothing about the place, so nothing here is a list of keep-outs. A motion is checked
against what has been found:

    seen occupied   not driven into; not passed through by the arm's hand (it may end at a thing: that is a touch or a grasp)
    never seen      gone into slowly, so that what is there is met gently and noticed
    seen free       gone through as usual

and against the other device: the rover's own shape where the locator puts it, grown by how unsure
that is, is occupied space to the arm.

With no map, everything is never seen: slow.
"""
from __future__ import annotations

import math

import cv2
import numpy as np

from . import geometry as G
from .errors import Refused
from .place import CELL_M, FREE, HALF_CELLS, OCCUPIED, UNSEEN

ROVER_MARGIN_M = 0.015            # the rover's outline is grown by this before it is laid on the map
STOP_SHORT_M = 0.02               # a drive ends this far before the first occupied cell it would reach
SLOW_CELLS = 12                   # a drive that takes the rover over this many never-seen cells (12 cm2) is slow, all of it
HAND_RADIUS_M = 0.03              # the hand is taken to be this wide around its tip
OVER_M = 0.015                    # the hand passes over a thing if it is this far above the thing's top
END_RADIUS_M = 0.05               # within this of where a move ends, a thing is being approached, not passed through
ROVER_GROW_M = 0.03               # the rover's outline is grown by this, and by twice how unsure its place is, for the arm


class Guard:
    def __init__(self, hub):
        self.hub = hub
        self.arm_was: list[np.ndarray] = []          # the arm's poses when the rover's cameras looked: what the map has of it

    # ---- the map, cell by cell -----------------------------------------------------------
    def _cells(self, xy_map: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """The state and height of the cell under each point (N x 2, map frame). Points off the grid are never seen."""
        place = self.hub.place
        ij = np.floor(np.asarray(xy_map, float) / CELL_M).astype(int) + HALF_CELLS
        ok = (ij >= 0).all(axis=1) & (ij < 2 * HALF_CELLS).all(axis=1)
        state, height = np.full(len(ij), UNSEEN, np.uint8), np.zeros(len(ij))
        cells = place.cells()
        state[ok] = cells[ij[ok, 0], ij[ok, 1]]
        height[ok] = place.height[ij[ok, 0], ij[ok, 1]]
        return state, height

    # ---- the rover -----------------------------------------------------------------------------
    def _rover_outline(self) -> np.ndarray:
        """Points 10 mm apart filling the rover's outline (stowed, as it drives), grown by the margin: N x 2 in its frame."""
        rover = self.hub.rover
        shape = rover.rover.shape(rover.head_now())[:, :2]
        hull = cv2.convexHull(shape.astype(np.float32)).reshape(-1, 2)
        centre = hull.mean(axis=0)
        grown = (centre + (hull - centre) * (1.0 + ROVER_MARGIN_M / np.maximum(np.linalg.norm(hull - centre, axis=1, keepdims=True), 1e-6))).astype(np.float32)
        lo, hi = grown.min(axis=0), grown.max(axis=0)
        gx, gy = np.meshgrid(np.arange(lo[0], hi[0] + 1e-9, CELL_M), np.arange(lo[1], hi[1] + 1e-9, CELL_M))
        pts = np.c_[gx.ravel(), gy.ravel()]
        inside = np.array([cv2.pointPolygonTest(grown.reshape(-1, 1, 2), (float(x), float(y)), False) >= 0 for x, y in pts])
        return pts[inside]

    def stood_here(self, way: int = 0, met: bool = False):
        """Where the rover stands is free: it is standing there. way, met: it has just driven forward (1) or back (-1) and
        was stopped by something: that is in the map now too, just past the end of it that led."""
        hub = self.hub
        if hub.rover is None or hub.place.empty:
            return
        T = hub.map_from_rover()
        pts = self._rover_outline()
        hub.place.mark_free((np.c_[pts, np.zeros(len(pts)), np.ones(len(pts))] @ T.T)[:, :2])
        if met and way:
            lead = pts[:, 0].max() if way > 0 else pts[:, 0].min()
            edge = pts[np.abs(pts[:, 0] - lead) < CELL_M] + [way * 1.5 * CELL_M, 0.0]
            hub.place.mark_met((np.c_[edge, np.zeros(len(edge)), np.ones(len(edge))] @ T.T)[:, :2])

    def drive(self, distance: float, angle: float) -> tuple[float, float, bool, str | None]:
        """A drive as it may be made: (distance, angle, slow, what was changed and why). Raises Refused if none of it may."""
        hub = self.hub
        if hub.place.empty:
            return distance, angle, True, ("slow: nothing has been looked at yet (POST /survey), so whatever is in the way is met "
                                           "at a crawl")
        outline = self._rover_outline()
        here = hub.map_from_rover()
        turning = distance == 0.0
        goal = abs(angle) if turning else abs(distance)
        step = math.radians(3.0) if turning else CELL_M
        sign = math.copysign(1.0, angle if turning else distance)
        at_start = self._cells((np.c_[outline, np.zeros(len(outline)), np.ones(len(outline))] @ here.T)[:, :2])[0]
        unseen: set[tuple[int, int]] = set()
        done, hit = goal, None
        for k in range(1, int(math.ceil(goal / step)) + 1):
            s = min(k * step, goal)
            move = G.pose(G.rot_z(sign * s), [0.0, 0.0, 0.0]) if turning else G.pose(np.eye(3), [sign * s, 0.0, 0.0])
            xy = (np.c_[outline, np.zeros(len(outline)), np.ones(len(outline))] @ (here @ move).T)[:, :2]
            state, height = self._cells(xy)
            new = state == OCCUPIED
            if k == 1:
                new &= at_start != OCCUPIED                          # (what it already stands against does not stop it leaving)
            if new.any():
                n = int(np.argmax(new))
                hit = (xy[n], float(height[n]))
                done = max(0.0, (k - 1) * step - (STOP_SHORT_M if not turning else math.radians(3.0)))
                break
            unseen.update(map(tuple, np.floor(xy[state == UNSEEN] / CELL_M).astype(int)))
        if hit is not None:
            where = (G.inv(here) @ [hit[0][0], hit[0][1], 0.0, 1.0])[:2]
            what = (f"the map has something {hit[1]:.3f} m high in the way, at ({where[0]:.2f}, {where[1]:.2f}) in the rover's own frame "
                    "(x ahead, y to its left)")
            if done < (math.radians(2.0) if turning else 0.01):
                raise Refused(f"not driven: {what}. GET /place shows what the map holds; turn away from it or back off first.", 409)
            note = (f"stopped short: {what}; it {'turns' if turning else 'goes'} "
                    f"{math.degrees(done):.0f} deg" if turning else f"stopped short: {what}; it goes {done:.2f} m") + " and no further"
            slow = len(unseen) >= SLOW_CELLS
            return (0.0, sign * done, slow, note) if turning else (sign * done, 0.0, slow, note)
        slow = len(unseen) >= SLOW_CELLS
        return distance, angle, slow, ("slow: part of where this goes has never been seen (GET /place: unseen), so whatever is "
                                       "there is met at a crawl" if slow else None)

    # ---- the arm -------------------------------------------------------------------------------
    def saw_arm(self, q):
        """The rover's cameras are looking with the arm in this pose: what they map of it is the arm, not a thing."""
        if q is not None and not any(np.abs(np.asarray(q) - p).max() < 0.02 for p in self.arm_was):
            self.arm_was.append(np.asarray(q, float).copy())
            del self.arm_was[:-12]

    def arm_shadow(self) -> np.ndarray:
        """Where the arm was, seen from above, at each pose the rover's cameras saw it in: N x 2 in the arm's frame, points
        20 mm apart filling each part's outline. (Not where it is now: what is in front of the hand now is not the hand.)"""
        safety = getattr(self.hub.arm.link, "safety", None) if self.hub.arm is not None else None
        if safety is None or not self.arm_was:
            return np.zeros((0, 2))
        out = []
        for q in self.arm_was:
            for part in safety.outlines(q):
                hull = cv2.convexHull(part.astype(np.float32)).reshape(-1, 1, 2)
                lo, hi = part.min(axis=0), part.max(axis=0)
                gx, gy = np.meshgrid(np.arange(lo[0], hi[0] + 0.02, 0.02), np.arange(lo[1], hi[1] + 0.02, 0.02))
                grid = np.c_[gx.ravel(), gy.ravel()]
                out.append(np.vstack([part, grid[[cv2.pointPolygonTest(hull, (float(x), float(y)), True) > -0.01 for x, y in grid]]]))
        return np.vstack(out)

    def _rover_for_arm(self):
        """The rover's shape in the arm's frame, how far to stay from it, and its top: (N x 3, margin) or (None, top) if not placed."""
        hub = self.hub
        shape = hub.rover.rover.shape(hub.rover.head_now())
        link, _ = hub.locator.chain("rover", "arm", hub.arm.link.q())
        if link is None:
            return None, float(shape[:, 2].max())
        pts = (np.c_[shape, np.ones(len(shape))] @ link.T.T)[:, :3]
        return pts, ROVER_GROW_M + 2.0 * link.sigma_m

    def hand(self, points: np.ndarray, touching: bool = False) -> tuple[bool, str | None]:
        """A path of the arm's hand (N x 3 in the arm's frame, its lowest point): (cautious, why). Raises Refused where it
        would pass through something the map has, or into the rover. touching: the path ends on a thing on purpose."""
        hub = self.hub
        points = np.asarray(points, float).reshape(-1, 3)
        end = points[-1]
        why = None
        if hub.rover is not None:
            shape, margin = self._rover_for_arm()
            if shape is None:
                if points[:, 2].min() < margin + OVER_M:
                    why = ("nothing relates the rover to the arm yet (POST /locate, POST /survey), and this goes lower than the rover "
                           f"is tall ({margin:.2f} m)")
            else:
                top = float(shape[:, 2].max()) + OVER_M
                hull = cv2.convexHull(shape[:, :2].astype(np.float32)).reshape(-1, 1, 2)
                away = lambda p: -cv2.pointPolygonTest(hull, (float(p[0]), float(p[1])), True)       # noqa: E731
                at_start = away(points[0]) if points[0][2] < top else np.inf
                for p in points:
                    # (a hand that is already nearer than it should be may always move away, or it could never leave)
                    if p[2] < top and away(p) < margin + HAND_RADIUS_M and away(p) < at_start - 0.005:
                        raise Refused(f"not moved: that takes the hand into where the rover is (its place in the arm's frame is known "
                                      f"to about {(margin - ROVER_GROW_M) / 2 * 1000:.0f} mm, and the hand keeps {margin * 1000:.0f} mm from "
                                      f"it), at ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}). Stay above {top:.2f} m there, or go round.", 409)
        T, _ = hub._map_in("arm") if hub.rover is not None else (None, "")
        if T is None:
            return True, why or "the arm's surroundings have not been mapped, or the map is not related to the arm (GET /place?relative_to=arm)"
        back = G.inv(T)
        shadow = self.arm_shadow()
        ring = np.array([[0.0, 0.0]] + [[HAND_RADIUS_M * math.cos(a), HAND_RADIUS_M * math.sin(a)] for a in np.arange(0, 2 * math.pi, math.pi / 4)]
                        + [[0.5 * HAND_RADIUS_M * math.cos(a), 0.5 * HAND_RADIUS_M * math.sin(a)] for a in np.arange(0, 2 * math.pi, math.pi / 2)])
        for p in points:
            around = p[:2] + ring
            mine = np.array([float(np.linalg.norm(shadow - a, axis=1).min()) < 0.04 for a in around]) if len(shadow) else np.zeros(len(around), bool)
            state, height = self._cells((np.c_[around, np.zeros(len(around)), np.ones(len(around))] @ back.T)[:, :2])
            solid = (state == OCCUPIED) & ~mine & (height + OVER_M > p[2])
            if solid.any():
                if np.linalg.norm(p[:2] - end[:2]) > END_RADIUS_M and np.linalg.norm(p[:2] - points[0][:2]) > END_RADIUS_M:
                    # (within END_RADIUS_M of where the move starts it is leaving a thing, and of where it ends, approaching one)
                    h = float(height[solid].max())
                    raise Refused(f"not moved: the line passes through something the map has, {h:.3f} m high, at ({p[0]:.2f}, {p[1]:.2f}) "
                                  f"in the arm's frame, with the hand at {p[2]:.3f} m there. Go over it (above {h + OVER_M + 0.005:.3f} m) "
                                  "or round it; GET /place?relative_to=arm shows what the map holds.", 409)
                why = why or "this ends at something the map has: it is approached slowly and stopped at the first touch"
            elif ((state == UNSEEN) | mine).any():
                why = why or "part of where the hand goes has never been seen: slowly, and stopped at the first touch"
        return why is not None, why
