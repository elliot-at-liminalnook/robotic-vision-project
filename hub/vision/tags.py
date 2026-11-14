"""Printed tags in a picture, and the pose of the rigid body that carries them.

    found = detect(image)                               # [Detection(id, corners)]
    fix = body_pose(found, corners_by_id, lens)         # where that body is, as the camera sees it

One small flat tag alone can be read tilted the wrong way (two poses explain its four corners
nearly equally well). `body_pose` therefore uses every tag of one rigid body together, and says so
when what it had cannot tell the two readings apart (`ambiguous`).
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .. import geometry as G
from ..builtin import Lens

PIXEL_NOISE = 0.5              # how well a corner is found, at best [px]; the floor for stated uncertainty
MIN_SIDE_PX = 12.0             # a tag smaller than this in the picture is not used


@dataclass(frozen=True)
class Detection:
    id: int
    corners: np.ndarray        # 4 x 2 [px]: top-left, top-right, bottom-right, bottom-left of the artwork

    @property
    def side_px(self) -> float:
        c = self.corners
        return float(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)]))


@dataclass
class BodyFix:
    """Where a body is in the camera's frame (OpenCV's: x right, y down, z forward)."""
    T_cam_body: np.ndarray
    cov: np.ndarray            # 6 x 6 for (rotation vector, translation) of T_cam_body: its uncertainty
    rms_px: float              # how well the pose explains the corners
    tags: list[int]
    ambiguous: bool            # the corners cannot tell this pose from a tilted one

    @property
    def sigma_m(self) -> float:
        """One number for the position's uncertainty [m]."""
        return float(np.sqrt(np.trace(self.cov[3:, 3:])))

    @property
    def sigma_rad(self) -> float:
        return float(np.sqrt(np.trace(self.cov[:3, :3])))


_DETECTOR = None


def _detector():
    global _DETECTOR
    if _DETECTOR is None:
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        _DETECTOR = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), params)
    return _DETECTOR


def detect(image: np.ndarray) -> list[Detection]:
    """Every tag found in an RGB or grey picture."""
    grey = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    corners, ids, _ = _detector().detectMarkers(grey)
    if ids is None:
        return []
    out = [Detection(int(i), c.reshape(4, 2).astype(float)) for i, c in zip(ids.ravel(), corners)]
    return sorted((d for d in out if d.side_px >= MIN_SIDE_PX), key=lambda d: d.id)


def _residuals(obj, img, rvec, tvec, K, dist):
    proj, jac = cv2.projectPoints(obj, rvec, tvec, K, dist)
    return (proj.reshape(-1, 2) - img).ravel(), jac[:, :6]


def body_pose(found: list[Detection], corners_by_id: dict[int, np.ndarray], lens: Lens) -> BodyFix | None:
    """The pose of one rigid body from every one of its tags in the picture.

    corners_by_id: tag id -> its four corners in the body's frame (builtin.Tag.corners()).
    Returns None if none of the body's tags was found.
    """
    use = [d for d in found if d.id in corners_by_id]
    if not use:
        return None
    obj = np.vstack([corners_by_id[d.id] for d in use]).astype(np.float64)
    img = np.vstack([d.corners for d in use]).astype(np.float64)
    K, dist = lens.K(), np.asarray(lens.dist, float)
    centred = obj - obj.mean(axis=0)
    flat = np.linalg.svd(centred, compute_uv=False)[-1] < 1e-4          # all corners in one plane?
    candidates = []
    if flat:
        # Two poses fit a flat pattern; get both and keep the better, remembering how close the other came.
        Rp = np.linalg.svd(centred)[2].T                                 # the plane's own axes
        if np.linalg.det(Rp) < 0:
            Rp[:, 2] *= -1
        local = centred @ Rp
        local[:, 2] = 0.0
        ok, rvecs, tvecs, errs = cv2.solvePnPGeneric(local, img, K, dist, flags=cv2.SOLVEPNP_IPPE)
        for rv, tv in zip(rvecs, tvecs):
            R = G.rodrigues(rv) @ Rp.T                                   # back to the body's frame
            t = tv.ravel() - R @ obj.mean(axis=0)
            candidates.append((cv2.Rodrigues(R)[0], t.reshape(3, 1)))
    else:
        ok, rv, tv = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_SQPNP)
        if ok:
            candidates.append((rv, tv))
    scored = []
    for rv, tv in candidates:
        rv, tv = cv2.solvePnPRefineLM(obj, img, K, dist, rv.astype(np.float64), tv.astype(np.float64))
        res, J = _residuals(obj, img, rv, tv, K, dist)
        if float(tv.ravel()[2]) > 0:                                     # in front of the camera
            scored.append((float(np.sqrt(np.mean(res ** 2))), rv, tv, J))
    if not scored:
        return None
    scored.sort(key=lambda s: s[0])
    rms, rv, tv, J = scored[0]
    ambiguous = bool(flat and len(scored) > 1 and scored[1][0] < max(3.0 * rms, 2.0 * PIXEL_NOISE))
    sigma = max(rms, PIXEL_NOISE)
    try:
        cov = sigma ** 2 * np.linalg.inv(J.T @ J)
    except np.linalg.LinAlgError:
        cov = np.eye(6) * 1e6
    if ambiguous:                                                        # the other reading is a tilt of tens of degrees
        cov = cov + np.diag([0.5, 0.5, 0.5, 0.0, 0.0, 0.0])
    return BodyFix(G.pose(G.rodrigues(rv), tv), cov, rms, [d.id for d in use], ambiguous)
