"""Depth from two pictures taken side by side: the rover's two cameras at a known spacing.

    cloud = depth(left, right, lens_left, lens_right, baseline_m)       # points in the left camera's frame

The two cameras are nominally parallel and level with each other, but each has its own lens and sits
a little differently, so a point is not on the same row in both pictures. Before matching, the rows
are lined up from the pictures themselves (`align_rows`): patches with detail are found again in the
other picture, and a shift and stretch of the rows is fitted to how far up or down each had moved.
Only the spacing is taken from the rover's own knowledge (the spacing drive's travel): it is what
gives the cloud its size.

Sideways sizes come out right whatever the lenses' true focal lengths are (they cancel); distances
along the view are off by the focal length's error, a per cent or two until the cameras are calibrated.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from ..builtin import Lens


@dataclass
class Cloud:
    points: np.ndarray             # N x 3 in the left camera's frame (x right, y down, z forward) [m]
    pixels: np.ndarray             # N x 2: where each was in the left picture (at the size used for matching)
    row_shift_px: float            # how far the right picture's rows had to move, at the image's middle
    row_stretch: float             # ... and how much they had to stretch
    matched: float                 # share of the picture that found a match
    scale: float                   # the size the pictures were matched at, relative to full size


def _level(image: np.ndarray, lens: Lens, scale: float) -> np.ndarray:
    """A picture with its lens's (nominal) distortion taken out, in grey, at the size used for matching."""
    und = cv2.undistort(image, lens.K(), np.asarray(lens.dist, float))
    grey = cv2.cvtColor(und, cv2.COLOR_RGB2GRAY) if und.ndim == 3 else und
    return cv2.resize(grey, (int(round(grey.shape[1] * scale)), int(round(grey.shape[0] * scale))), interpolation=cv2.INTER_AREA)


def align_rows(left: np.ndarray, right: np.ndarray, max_disp: int, patches: int = 200, half_h: int = 3, half_w: int = 28, reach: int = 16):
    """How the right picture's rows must move to line up with the left's: (shift at the middle row, stretch),
    so that a point on row y of the left picture is on row  mid + (y - mid) * stretch + shift  of the right.
    None if too few patches could be found again to tell.

    The patches are wide and only a few rows high: seen from low down, the ground's nearer rows have moved
    much further sideways between the two pictures than its farther rows, which shears a square patch
    beyond recognition (none of 150 were found again) but hardly changes a thin strip."""
    h, w = left.shape
    corners = cv2.goodFeaturesToTrack(left, patches, 0.01, 20, blockSize=7)
    if corners is None:
        return None
    votes = []
    for x, y in corners.reshape(-1, 2).astype(int):
        if not (half_h <= y < h - half_h and half_w <= x < w - half_w):
            continue
        tpl = left[y - half_h:y + half_h + 1, x - half_w:x + half_w + 1]
        x0, x1 = max(0, x - max_disp - half_w), min(w, x + half_w + 4)
        y0, y1 = max(0, y - half_h - reach), min(h, y + half_h + reach + 1)
        if x1 - x0 <= 2 * half_w + 1 or y1 - y0 <= 2 * half_h + 1:
            continue
        res = cv2.matchTemplate(right[y0:y1, x0:x1], tpl, cv2.TM_CCOEFF_NORMED)
        _, best, _, loc = cv2.minMaxLoc(res)
        if best < 0.8:
            continue
        res[max(0, loc[1] - 2):loc[1] + 3, max(0, loc[0] - 4):loc[0] + 5] = -1          # is the match the only good one?
        if cv2.minMaxLoc(res)[1] > best - 0.08:
            continue
        votes.append((y, y0 + loc[1] + half_h - y))
    if len(votes) < 12:
        return None
    y, dy = np.array(votes, float).T
    mid = (h - 1) / 2
    keep = np.abs(dy - np.median(dy)) < 6
    for _ in range(3):                                         # a line through the votes, dropping the stragglers
        if keep.sum() < 10:
            return None
        A = np.c_[np.ones(keep.sum()), y[keep] - mid]
        shift, slope = np.linalg.lstsq(A, dy[keep], rcond=None)[0]
        resid = dy - (shift + slope * (y - mid))
        keep = np.abs(resid) < max(1.0, 2.5 * np.std(resid[keep]))
    return float(shift), float(1.0 + slope)


def depth(left: np.ndarray, right: np.ndarray, lens_left: Lens, lens_right: Lens, baseline_m: float,
          scale: float = 0.5, max_disp: int = 256, min_texture: float = 4.0) -> Cloud | None:
    """Points seen by both cameras, in the left camera's frame. None if the rows could not be lined up
    (too little detail in view)."""
    gl, gr = (cv2.GaussianBlur(_level(img, lens, scale), (0, 0), 0.8) for img, lens in ((left, lens_left), (right, lens_right)))
    fit = align_rows(gl, gr, max_disp)
    if fit is None:
        return None
    shift, stretch = fit
    h, w = gl.shape
    mid = (h - 1) / 2
    # warpAffine with WARP_INVERSE_MAP: output row y takes the right picture's row  mid + (y - mid) * stretch + shift
    M = np.float32([[1, 0, 0], [0, stretch, mid * (1 - stretch) + shift]])
    gr = cv2.warpAffine(gr, M, (w, h), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
    sgbm = cv2.StereoSGBM_create(minDisparity=0, numDisparities=max_disp, blockSize=7, P1=8 * 49, P2=32 * 49, uniquenessRatio=15,
                                 speckleWindowSize=150, speckleRange=2, disp12MaxDiff=1, preFilterCap=31,
                                 mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    disp = sgbm.compute(gl, gr).astype(np.float32) / 16.0
    blur = cv2.GaussianBlur(gl.astype(np.float32), (0, 0), 3)
    texture = np.sqrt(cv2.GaussianBlur((gl.astype(np.float32) - blur) ** 2, (0, 0), 3))       # is there anything here to match on?
    ok = (disp > 1.0) & (texture > min_texture)
    v, u = np.nonzero(ok)
    d = disp[v, u]
    lens = lens_left.at(w, h)
    Z = lens.fx * baseline_m / d
    pts = np.c_[(u - lens.cx) * Z / lens.fx, (v - lens.cy) * Z / lens.fy, Z]
    return Cloud(pts, np.c_[u, v], shift, stretch, float(ok.mean()), scale)


def triangulate(px_left, px_right, lens_left: Lens, lens_right: Lens, baseline_m: float) -> np.ndarray:
    """Points seen at given pixels in both full-size pictures (N x 2 each), in the left camera's frame. The rows are
    not assumed to line up: each point is put where its two rays, a baseline apart, pass closest."""
    def rays(px, lens):
        n = cv2.undistortPoints(np.asarray(px, np.float64).reshape(-1, 1, 2), lens.K(), np.asarray(lens.dist, float)).reshape(-1, 2)
        return np.c_[n, np.ones(len(n))]
    a, b = rays(px_left, lens_left), rays(px_right, lens_right)
    o = np.array([baseline_m, 0.0, 0.0])                       # the right camera is a baseline to the left one's right
    out = []
    for ra, rb in zip(a, b):
        A = np.c_[ra, -rb]
        s, t = np.linalg.lstsq(A, o, rcond=None)[0]
        out.append((s * ra + o + t * rb) / 2)
    return np.array(out)
