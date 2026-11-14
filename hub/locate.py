"""Where things are relative to each other, from what cameras see of the devices' own tags.

Step 3 of the plan: a fixed camera locates itself relative to the arm from one picture of the tag
blocks beside the arm's base. No number about any place is used: only the picture, the camera
kind's nominal lens and where the arm's tags are on the arm.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import builtin
from . import geometry as G
from .vision import tags as V


@dataclass
class CameraFix:
    """A camera's pose in the arm's base frame (camera frame: x right, y down, z forward)."""
    T_base_cam: np.ndarray
    sigma_m: float                 # uncertainty of its position [m]
    sigma_rad: float               # ... and of its orientation
    tags: list[int]                # the arm tags that were used
    rms_px: float
    ambiguous: bool

    @property
    def trusted(self) -> bool:
        """Good enough to act on: at least two tags of the base, and a pose that explains them."""
        return len(self.tags) >= 2 and not self.ambiguous and self.rms_px < 2.0


def arm_base_corners() -> dict[int, np.ndarray]:
    """Corners of the tags fixed to the arm's base, in the base's frame."""
    return {i: t.corners() for i, t in builtin.tags("arm").items() if t.on == "base"}


def camera_from_arm(image: np.ndarray, lens: builtin.Lens | None = None) -> CameraFix | None:
    """Locate the camera that took this picture relative to the arm, from the arm's base tags.

    lens: the camera's lens at the picture's size (default: the Camera Module 3 Wide's nominal values).
    None if no base tag is in the picture.
    """
    if lens is None:
        lens = builtin.LENSES["cm3_wide"].at(image.shape[1], image.shape[0])
    fix = V.body_pose(V.detect(image), arm_base_corners(), lens)
    if fix is None:
        return None
    T = G.inv(fix.T_cam_body)
    # Uncertainty of the camera's place in the base's frame: the body's in the camera's, carried over
    # (position error also grows with distance times the angle error).
    dist = float(np.linalg.norm(fix.T_cam_body[:3, 3]))
    sigma_m = float(np.hypot(fix.sigma_m, dist * fix.sigma_rad))
    return CameraFix(T, sigma_m, fix.sigma_rad, fix.tags, fix.rms_px, fix.ambiguous)
