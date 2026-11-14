"""A camera's lens and sensor: what turns MuJoCo's perfect pinhole picture into a picture a camera gives.

MuJoCo draws an ideal pinhole image. A real Camera Module 3 Wide does not give one. Each simulated
camera therefore has a `LensTruth` (its own, slightly different from every other camera's), and every
picture passes through it before anything else sees it:

  1. distortion   the scene is rendered wider and larger than needed, then warped by OpenCV's
                  lens model (k1, k2, p1, p2, k3 about the true focal lengths and centre)
  2. darkening    toward the corners
  3. blur         a little everywhere, more off-centre
  4. noise        read noise and shot noise, different in every frame

JPEG is not applied here: the camera port does that, as the firmware does.

The numbers are PLACEHOLDERS matched to the datasheet until one of the real cameras is calibrated
(docs/MULTI_DEVICE_PLAN.md, O7). Distortion: the datasheet gives a 102 deg horizontal field on the full
sensor where a pinhole of the same focal length would give 99 deg, so a ray 51 deg off-axis lands where
a pinhole would put one 49.5 deg off: tan(49.5)/tan(51) = 0.95 = 1 + k1 * tan(51)^2, k1 = -0.033.

These are ground truth: the hub is never given them. It starts from the camera kind's nominal values
(hub/builtin.py) and has to correct them from what it sees.
"""
from __future__ import annotations

import math
import zlib
from dataclasses import dataclass, replace

import cv2
import numpy as np

FULL_W, FULL_H = 1920, 1080            # the sensor mode the values below are for
MAX_RENDER = (2400, 1350)              # the off-screen buffer scenes are built with (scenes/build.py)


@dataclass(frozen=True)
class LensTruth:
    fx: float = 982.1
    fy: float = 982.1
    cx: float = 959.5                  # pixel-index coordinates: the middle of a 1920-wide picture is 959.5
    cy: float = 539.5
    k1: float = -0.033
    k2: float = 0.0015
    p1: float = 0.0
    p2: float = 0.0
    k3: float = 0.0
    vignette: float = 0.35             # brightness lost at the corner, as a fraction
    blur_centre_px: float = 0.6        # Gaussian sigma at 1920 x 1080
    blur_corner_px: float = 1.4
    read_noise: float = 1.5            # grey levels (of 255)
    shot_noise: float = 0.25           # times sqrt(grey level)

    @property
    def dist(self) -> np.ndarray:
        return np.array([self.k1, self.k2, self.p1, self.p2, self.k3])

    def K(self, width: int = FULL_W, height: int = FULL_H) -> np.ndarray:
        """The true camera matrix for a picture of this size (the same field of view, scaled)."""
        sx, sy = width / FULL_W, height / FULL_H
        return np.array([[self.fx * sx, 0.0, (self.cx + 0.5) * sx - 0.5],
                         [0.0, self.fy * sy, (self.cy + 0.5) * sy - 0.5], [0.0, 0.0, 1.0]])


IDEAL = LensTruth(k1=0.0, k2=0.0, vignette=0.0, blur_centre_px=0.0, blur_corner_px=0.0, read_noise=0.0, shot_noise=0.0)


def lens_for(camera: str, seed: int = 0) -> LensTruth:
    """This camera's own lens: the placeholder values, off by what one unit differs from the next."""
    rng = np.random.default_rng([int(seed), zlib.crc32(camera.encode())])
    f = 982.1 * (1 + rng.normal(0, 0.008))
    return LensTruth(fx=f, fy=f * (1 + rng.normal(0, 0.001)),
                     cx=959.5 + rng.normal(0, 6.0), cy=539.5 + rng.normal(0, 6.0),
                     k1=-0.033 + rng.normal(0, 0.004), k2=0.0015 + rng.normal(0, 0.0008),
                     p1=rng.normal(0, 0.0004), p2=rng.normal(0, 0.0004))


class Lens:
    """One camera's lens at one picture size. `shoot(render)` gives the picture the camera would."""

    def __init__(self, truth: LensTruth, width: int, height: int):
        self.truth, self.width, self.height = truth, int(width), int(height)
        K = truth.K(width, height)
        u, v = np.meshgrid(np.arange(width, dtype=np.float64), np.arange(height, dtype=np.float64))
        pix = np.stack([u.ravel(), v.ravel()], axis=1).reshape(-1, 1, 2)
        # Where the ray through each pixel of the real picture would have landed in a pinhole picture.
        ideal = cv2.undistortPoints(pix, K, truth.dist).reshape(height, width, 2)
        x_max, y_max = float(np.abs(ideal[..., 0]).max()) * 1.01, float(np.abs(ideal[..., 1]).max()) * 1.01
        f = max(K[0, 0], K[1, 1])                                   # render at least as finely as the picture
        f = min(f, 0.5 * MAX_RENDER[0] / x_max, 0.5 * MAX_RENDER[1] / y_max)
        self.render_width, self.render_height = 2 * math.ceil(x_max * f), 2 * math.ceil(y_max * f)
        self.render_fovy_deg = math.degrees(2 * math.atan(0.5 * self.render_height / f))
        self._map_x = (ideal[..., 0] * f + self.render_width / 2 - 0.5).astype(np.float32)
        self._map_y = (ideal[..., 1] * f + self.render_height / 2 - 0.5).astype(np.float32)
        cx, cy = K[0, 2], K[1, 2]
        r2 = ((u - cx) ** 2 + (v - cy) ** 2) / ((width / 2) ** 2 + (height / 2) ** 2)
        self._gain = (1.0 - truth.vignette * r2).astype(np.float32)[..., None]
        self._edge = np.clip(r2, 0.0, 1.0).astype(np.float32)[..., None]
        self._scale = width / FULL_W
        self.plain = truth == IDEAL

    def shoot(self, render, rng: np.random.Generator | None = None) -> np.ndarray:
        """render(width, height, fovy_deg) -> an ideal pinhole RGB picture. Returns what this camera gives (uint8 RGB)."""
        ideal = render(self.render_width, self.render_height, self.render_fovy_deg)
        t = self.truth
        img = cv2.remap(ideal, self._map_x, self._map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT).astype(np.float32)
        if self.plain:
            return np.clip(img + 0.5, 0, 255).astype(np.uint8)
        img *= self._gain
        s0, s1 = t.blur_centre_px * self._scale, t.blur_corner_px * self._scale
        if s1 > 0.05:
            sharp = cv2.GaussianBlur(img, (0, 0), s0) if s0 > 0.05 else img
            img = sharp * (1 - self._edge) + cv2.GaussianBlur(img, (0, 0), s1) * self._edge
        if rng is not None and (t.read_noise > 0 or t.shot_noise > 0):
            img += rng.normal(0.0, 1.0, img.shape).astype(np.float32) * (t.read_noise + t.shot_noise * np.sqrt(np.maximum(img, 0.0)))
        return np.clip(img + 0.5, 0, 255).astype(np.uint8)


def with_values(truth: LensTruth, **values) -> LensTruth:
    return replace(truth, **values)
