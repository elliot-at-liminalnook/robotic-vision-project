"""Outlines of surfaces: right-angled polygons, cut into rectangles."""
from __future__ import annotations

import numpy as np


def is_right_angled(outline) -> bool:
    p = np.asarray(outline, float)
    if p.ndim != 2 or p.shape[1] != 2 or len(p) < 4:
        return False
    d = np.roll(p, -1, axis=0) - p
    return bool(np.all((np.abs(d[:, 0]) < 1e-9) ^ (np.abs(d[:, 1]) < 1e-9)))


def inside(outline, x: float, y: float) -> bool:
    """Is the point inside the polygon? (even-odd rule)"""
    p = np.asarray(outline, float)
    n, hit = len(p), False
    for i in range(n):
        (x0, y0), (x1, y1) = p[i], p[(i + 1) % n]
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            hit = not hit
    return hit


def rectangles(outline) -> list[tuple[float, float, float, float]]:
    """A right-angled outline as rectangles (x0, y0, x1, y1) that tile it without overlap."""
    p = np.asarray(outline, float)
    xs, ys = sorted(set(np.round(p[:, 0], 9))), sorted(set(np.round(p[:, 1], 9)))
    rects = []
    for i in range(len(xs) - 1):
        run = None
        for j in range(len(ys) - 1):
            cx, cy = 0.5 * (xs[i] + xs[i + 1]), 0.5 * (ys[j] + ys[j + 1])
            if inside(p, cx, cy):
                run = [ys[j], ys[j + 1]] if run is None else [run[0], ys[j + 1]]
            elif run is not None:
                rects.append((xs[i], run[0], xs[i + 1], run[1]))
                run = None
        if run is not None:
            rects.append((xs[i], run[0], xs[i + 1], run[1]))
    merged: list[list[float]] = []                 # join slabs that share their y interval
    for r in rects:
        if merged and merged[-1][1] == r[1] and merged[-1][3] == r[3] and abs(merged[-1][2] - r[0]) < 1e-9:
            merged[-1][2] = r[2]
        else:
            merged.append(list(r))
    return [tuple(float(v) for v in r) for r in merged]


def edges(outline) -> list[tuple[np.ndarray, np.ndarray]]:
    p = np.asarray(outline, float)
    return [(p[i], p[(i + 1) % len(p)]) for i in range(len(p))]


def area(outline) -> float:
    p = np.asarray(outline, float)
    x, y = p[:, 0], p[:, 1]
    return float(abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))) / 2)


def signed_area(outline) -> float:
    p = np.asarray(outline, float)
    x, y = p[:, 0], p[:, 1]
    return float((np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))) / 2)


def distance_to_edge(outline, x: float, y: float) -> float:
    """Distance from a point to the nearest edge of the outline."""
    q, best = np.array([x, y], float), np.inf
    for a, b in edges(outline):
        t = np.clip(np.dot(q - a, b - a) / max(np.dot(b - a, b - a), 1e-18), 0, 1)
        best = min(best, float(np.linalg.norm(q - (a + t * (b - a)))))
    return best
