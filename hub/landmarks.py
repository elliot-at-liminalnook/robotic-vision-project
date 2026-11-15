"""Landmarks: two devices that cannot see each other, related by things both have found.

    T, sigma_m, sigma_rad, used = match(touches, things)       # the things' frame in the touches' frame

The arm finds a thing by touching it: where its tip stopped, and how high (hub/arm/actions.py, probe).
The rover's map has things with a place and a height (hub/place.py). If two or more of the arm's touches
are of things in the map, the one turn and shift that lays the map's things on the touches relates the
two frames. Both stand on the same surface, so it is a turn about the vertical and a shift along the
surface: two things settle it, a third confirms it.

It is refused rather than guessed when the touches fit the map in more than one way about equally
well: two things of the same height fit both ways round. (The map holds the arm itself too, in pieces of
several heights; a fit through one of those is nearly always a worse fit, and is set aside for that.)
"""
from __future__ import annotations

import math
from itertools import permutations

import numpy as np

HEIGHT_TOL_M = 0.012           # a touch and a thing in the map are the same thing only if their heights agree this well
PLACE_TOL_M = 0.035            # ... and, once laid over each other, their places (a thing in the map is the part of it that was seen)
CLEARLY_WORSE_M = 0.003        # another way of laying the map on the touches is set aside if it misses by more than this, and
#                                by more than two and a half times what the best way misses by
MIN_APART_M = 0.06             # two landmarks nearer each other than this say nothing about direction


def _fit(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """The 2-D turn and shift T (3 x 3) with a = T b, least squares, for matched points (N x 2 each)."""
    ca, cb = a.mean(axis=0), b.mean(axis=0)
    H = (b - cb).T @ (a - ca)
    yaw = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    T = np.eye(3)
    T[:2, :2], T[:2, 2] = R, ca - R @ cb
    return T


def match(touches: list[dict], things: list[dict]) -> tuple[np.ndarray, float, float, list[tuple[int, int]]] | tuple[None, str]:
    """touches: [{"at_m": [x, y], "height_m": h}] in one frame; things: the same in another. Returns the things' frame in
    the touches' frame as a 4 x 4, how sure [m, rad], and which touch was matched to which thing; or (None, why)."""
    A = [(np.array(t["at_m"], float), float(t["height_m"])) for t in touches if t["height_m"] > HEIGHT_TOL_M]
    B = [(np.array(t["at_m"], float), float(t["height_m"])) for t in things]
    if len(A) < 2:
        return None, "fewer than two things have been touched (a touch of the bare surface says nothing about where)"
    fits = []
    for ia, ka in permutations(range(len(A)), 2):
        if ia > ka or np.linalg.norm(A[ia][0] - A[ka][0]) < MIN_APART_M:
            continue
        for ib, kb in permutations(range(len(B)), 2):
            if abs(A[ia][1] - B[ib][1]) > HEIGHT_TOL_M or abs(A[ka][1] - B[kb][1]) > HEIGHT_TOL_M:
                continue
            if abs(np.linalg.norm(A[ia][0] - A[ka][0]) - np.linalg.norm(B[ib][0] - B[kb][0])) > 1.5 * PLACE_TOL_M:
                continue
            T = _fit(np.array([A[ia][0], A[ka][0]]), np.array([B[ib][0], B[kb][0]]))
            pairs = []                                          # every touch this lays on a thing of its height
            for n, (pa, ha) in enumerate(A):
                d = [(np.linalg.norm(pa - (T @ [pb[0], pb[1], 1.0])[:2]), m) for m, (pb, hb) in enumerate(B) if abs(ha - hb) <= HEIGHT_TOL_M]
                if d and min(d)[0] <= PLACE_TOL_M:
                    pairs.append((n, min(d)[1]))
            if len(pairs) >= 2 and len({m for _, m in pairs}) == len(pairs):
                T = _fit(np.array([A[n][0] for n, _ in pairs]), np.array([B[m][0] for _, m in pairs]))
                miss = float(np.sqrt(np.mean([np.sum((A[n][0] - (T @ [B[m][0][0], B[m][0][1], 1.0])[:2]) ** 2) for n, m in pairs])))
                fits.append((len(pairs), T, pairs, miss))
    if not fits:
        return None, "the things touched do not fit the things in the map, by their heights and how far apart they are"
    most = max(f[0] for f in fits)
    best = sorted((f for f in fits if f[0] == most), key=lambda f: f[3])     # the most things laid on touches, then the closest fit
    yaw = lambda T: math.atan2(T[1, 0], T[0, 0])                                             # noqa: E731
    apart = lambda T, U: max(abs((yaw(T) - yaw(U) + math.pi) % (2 * math.pi) - math.pi), np.linalg.norm(T[:2, 2] - U[:2, 2]) / 0.1)   # noqa: E731
    # Another way of laying them that fits nearly as well makes it a guess: it must be clearly worse to be set aside.
    if any(apart(best[0][1], f[1]) > 0.2 and f[3] < max(2.5 * best[0][3], CLEARLY_WORSE_M) for f in best[1:]):
        return None, ("the things touched fit the map in more than one way (things of the same height, the same distance apart): "
                      "touch one more thing, of another height or off their line")
    _, T2, pairs, _ = best[0]
    spread = max(np.linalg.norm(A[n][0] - A[k][0]) for n, _ in pairs for k, _ in pairs)
    sigma_m = PLACE_TOL_M / 2 / math.sqrt(len(pairs))
    sigma_rad = math.sqrt(2) * (PLACE_TOL_M / 2) / max(spread, MIN_APART_M)
    T = np.eye(4)
    T[:2, :2], T[:2, 3] = T2[:2, :2], T2[:2, 2]
    return T, sigma_m, sigma_rad, pairs
