"""Depth from two pictures taken side by side: the rover's two cameras at a known spacing.

    cloud = depth(left, right, lens_left, lens_right, baseline_m)       # points in the left camera's frame

The two cameras are nominally parallel and level with each other, but each has its own lens and sits
a little differently, so a point is not on the same row in both pictures. Before matching, the rows
are lined up from the pictures themselves (`align_rows`): patches with detail are found again in the
other picture, and a shift and stretch of the rows is fitted to how far up or down each had moved.
Only the spacing is taken from the rover's own knowledge (the spacing drive's travel): it is what
gives the cloud its size.

What the rows cannot show is how far the two pictures are shifted sideways against each other (each
lens's centre is a few pixels from where it should be, and the cameras toe in or out a little). That
shift adds the same amount to every disparity, which puts things a few per cent too far or too near
(about 3 % at 0.3 m with the cameras 60 mm apart, for lens centres 5 pixels apart). `depth` takes it
as `offset_px`; until something has measured it, it is zero and the map says how good its distances are.
(Measuring it from the spacing drive alone was tried: the same view at two spacings gives it in
principle, but only if the spacings are known to 0.05 %, which a printed rack and pinion is not.)

Sideways sizes come out right whatever the lenses' true focal lengths are (they cancel); distances
along the view are off by the focal length's error, a per cent or two until the cameras are calibrated.

What it cannot do:
  * a thing nearer than the search reaches (about 60 mm from lenses 60 mm apart, 100 mm at 100 mm) is
    not left out: it is matched to something else and lands in the wrong place;
  * nearer than about 2.7 times the spacing (0.16 m at 60 mm apart, 0.27 m at 100 mm) the two
    cameras see a thing from directions more than 20 deg apart: a tenth to a third of what is
    matched there is matched wrongly, and a plain thing is not matched at all. (The map leaves out
    cells whose points are scattered below the surface, which is what such mismatches look like.);
  * a plain surface gives nothing, and a plain straight edge that runs along the picture's rows
    gives nothing either: only where it crosses them at an angle;
  * the two pictures must be of the same instant, or of a scene and a head that are both still.
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
    disparity: np.ndarray | None = None      # as matched, before any offset: how far left each point is in the right picture [px at that size]; < 0 = none
    colours: np.ndarray | None = None        # N x 3 (RGB, 0 to 255): what the left picture shows at each point


def _level(image: np.ndarray, lens: Lens, scale: float) -> np.ndarray:
    """A picture with its lens's (nominal) distortion taken out, in grey, at the size used for matching."""
    und = cv2.undistort(image, lens.K(), np.asarray(lens.dist, float))
    grey = cv2.cvtColor(und, cv2.COLOR_RGB2GRAY) if und.ndim == 3 else und
    return cv2.resize(grey, (int(round(grey.shape[1] * scale)), int(round(grey.shape[0] * scale))), interpolation=cv2.INTER_AREA)


def align_rows(left: np.ndarray, right: np.ndarray, max_disp: int, patches: int = 200, half_h: int = 3, half_w: int = 28, reach: int = 16,
               stretch: float | None = None):
    """How the right picture's rows must move to line up with the left's: (shift at the middle row, stretch),
    so that a point on row y of the left picture is on row  mid + (y - mid) * stretch + shift  of the right.
    None if too few patches could be found again to tell. stretch: if the pair's stretch is already known (it is one
    lens's focal length over the other's, and does not change), only the shift is fitted.

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
    if stretch is not None:
        off = dy - (stretch - 1.0) * (y - mid)
        keep = np.abs(off - np.median(off)) < 1.5
        return (float(np.mean(off[keep])), float(stretch)) if keep.sum() >= 10 else None
    keep = np.abs(dy - np.median(dy)) < 6
    for _ in range(3):                                         # a line through the votes, dropping the stragglers
        if keep.sum() < 10:
            return None
        A = np.c_[np.ones(keep.sum()), y[keep] - mid]
        shift, slope = np.linalg.lstsq(A, dy[keep], rcond=None)[0]
        resid = dy - (shift + slope * (y - mid))
        keep = np.abs(resid) < max(1.0, 2.5 * np.std(resid[keep]))
    return float(shift), float(1.0 + slope)


def _match(left: np.ndarray, right: np.ndarray, max_disp: int, speckle: int) -> np.ndarray:
    """How far each point of the left picture is to the left in the right one [px]; below zero where no match was found."""
    sgbm = cv2.StereoSGBM_create(minDisparity=0, numDisparities=max_disp, blockSize=7, P1=8 * 49, P2=32 * 49, uniquenessRatio=15,
                                 speckleWindowSize=speckle, speckleRange=2, disp12MaxDiff=1, preFilterCap=31,
                                 mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    return sgbm.compute(left, right).astype(np.float32) / 16.0


def depth(left: np.ndarray, right: np.ndarray, lens_left: Lens, lens_right: Lens, baseline_m: float, offset_px: float = 0.0,
          stretch: float | None = None, scale: float = 0.5, max_disp: int = 256, min_texture: float = 4.0) -> Cloud | None:
    """Points seen by both cameras, in the left camera's frame. None if the rows could not be lined up
    (too little detail in view). offset_px: the pair's sideways shift (see pair_offset), in full-size pixels."""
    gl, gr = (cv2.GaussianBlur(_level(img, lens, scale), (0, 0), 0.8) for img, lens in ((left, lens_left), (right, lens_right)))
    fit = align_rows(gl, gr, max_disp, stretch=stretch)
    if fit is None:
        return None
    shift, stretch = fit
    h, w = gl.shape
    mid = (h - 1) / 2
    # warpAffine with WARP_INVERSE_MAP: output row y takes the right picture's row  mid + (y - mid) * stretch + shift
    # (the stretch is one lens's focal length over the other's, so it applies across the picture as well as down it)
    M = np.float32([[stretch, 0, (w - 1) / 2 * (1 - stretch)], [0, stretch, mid * (1 - stretch) + shift]])
    gr = cv2.warpAffine(gr, M, (w, h), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
    # Matched twice: at this size, and at half of it, where the same search reaches things twice as near. A thing nearer
    # than the search reaches is not left unmatched by the finer pass: it is matched, confidently, to something else, and
    # lands far away. So where the two passes disagree the coarser one is believed.
    fine = _match(gl, gr, max_disp, speckle=150)
    half = lambda g: cv2.resize(g, (w // 2, h // 2), interpolation=cv2.INTER_AREA)      # noqa: E731
    coarse = cv2.resize(_match(half(gl), half(gr), max_disp, speckle=40), (w, h), interpolation=cv2.INTER_NEAREST)
    coarse = np.where(coarse > 0, coarse * 2.0, -1.0)
    agree = (fine > 0) & ((coarse <= 0) | (np.abs(fine - coarse) < 6.0))
    disp = np.where(agree, fine, coarse).astype(np.float32)
    blur = cv2.GaussianBlur(gl.astype(np.float32), (0, 0), 3)
    texture = np.sqrt(cv2.GaussianBlur((gl.astype(np.float32) - blur) ** 2, (0, 0), 3))       # is there anything here to match on?
    raw = disp
    disp = np.where(disp > 0, disp - offset_px * scale, -1.0)
    ok = (disp > 1.0) & (texture > min_texture)
    v, u = np.nonzero(ok)
    d = disp[v, u]
    lens = lens_left.at(w, h)
    Z = lens.fx * baseline_m / d
    pts = np.c_[(u - lens.cx) * Z / lens.fx, (v - lens.cy) * Z / lens.fy, Z]
    colours = None
    if left.ndim == 3:
        small = cv2.resize(cv2.undistort(left, lens_left.K(), np.asarray(lens_left.dist, float)), (w, h), interpolation=cv2.INTER_AREA)
        colours = small[v, u]
    return Cloud(pts, np.c_[u, v], shift, stretch, float(ok.mean()), scale, raw, colours)


def ground_plane(points: np.ndarray, down: np.ndarray, height_m: float, band_m: float = 0.03, reach_m: float = 0.5):
    """The flat ground in a cloud, as the vector m with  m . p = 1  for every point p on it (so 1 / |m| is how far the
    camera is above it, and m / |m| is which way is down). It is looked for near where the rover's own shape says the
    surface it stands on must be: `down` (unit, in the camera's frame) and `height_m`. None if too little of it is in view."""
    p = points[::7]
    p = p[(np.linalg.norm(p, axis=1) < reach_m) & (np.abs(p @ down - height_m) < band_m)]
    if len(p) < 800:
        return None
    for _ in range(4):
        m = np.linalg.lstsq(p, np.ones(len(p)), rcond=None)[0]
        off = (p @ m - 1.0) / np.linalg.norm(m)
        keep = np.abs(off) < max(0.0015, 2.0 * float(np.std(off)))
        if keep.sum() < 800:
            return None
        p = p[keep]
    return m


def triangulate(px_left, px_right, lens_left: Lens, lens_right: Lens, baseline_m: float, offset_px: float = 0.0) -> np.ndarray:
    """Points seen at given pixels in both full-size pictures (N x 2 each), in the left camera's frame. The rows are
    not assumed to line up: each point is put where its two rays, a baseline apart, pass closest."""
    def rays(px, lens):
        n = cv2.undistortPoints(np.asarray(px, np.float64).reshape(-1, 1, 2), lens.K(), np.asarray(lens.dist, float)).reshape(-1, 2)
        return np.c_[n, np.ones(len(n))]
    px_right = np.asarray(px_right, np.float64) + [offset_px, 0.0]
    a, b = rays(px_left, lens_left), rays(px_right, lens_right)
    o = np.array([baseline_m, 0.0, 0.0])                       # the right camera is a baseline to the left one's right
    out = []
    for ra, rb in zip(a, b):
        A = np.c_[ra, -rb]
        s, t = np.linalg.lstsq(A, o, rcond=None)[0]
        out.append((s * ra + o + t * rb) / 2)
    return np.array(out)
