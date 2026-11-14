"""The lens and sensor model: pictures are not perfect, in the ways a real camera's are not.

Aimed at classes of bugs:
  * a warp that is not the lens model it claims to be (checked against OpenCV's own projection);
  * a lens that leaves straight lines straight, or bends them the wrong way;
  * corners that are not darker, noise that is the same in every frame or changes the picture's level;
  * every camera having the same lens, or the hub's nominal lens being some camera's true one;
  * locating through the lens getting worse than stated, or claiming more certainty than it has.
"""
import math

import cv2
import numpy as np
import pytest

from hub import builtin, locate
from so101sim import lens as L
from so101sim.world import World
from test_markers import errors, scene_with_camera

QUIET = dict(vignette=0.0, blur_centre_px=0.0, blur_corner_px=0.0, read_noise=0.0, shot_noise=0.0)


def dots(points):
    """A render function that draws a soft dot at each ideal (pinhole) image point (x, y) = (X/Z, Y/Z)."""
    def render(width, height, fovy_deg):
        f = 0.5 * height / math.tan(math.radians(fovy_deg) / 2)
        img = np.zeros((height, width), np.float32)
        yy, xx = np.mgrid[0:height, 0:width].astype(np.float32)
        for x, y in points:
            u, v = x * f + width / 2 - 0.5, y * f + height / 2 - 0.5
            img += np.exp(-((xx - u) ** 2 + (yy - v) ** 2) / (2 * 2.5 ** 2))
        return np.repeat((np.clip(img, 0, 1) * 255).astype(np.uint8)[..., None], 3, axis=2)
    return render


def centroid(img, near, r=12):
    u0, v0 = int(round(near[0])), int(round(near[1]))
    patch = img[max(v0 - r, 0):v0 + r + 1, max(u0 - r, 0):u0 + r + 1, 0].astype(float)
    ys, xs = np.mgrid[max(v0 - r, 0):max(v0 - r, 0) + patch.shape[0], max(u0 - r, 0):max(u0 - r, 0) + patch.shape[1]]
    return np.array([(xs * patch).sum(), (ys * patch).sum()]) / patch.sum()


@pytest.mark.parametrize("camera,size", [("camera_1", (1920, 1080)), ("wrist", (1920, 1080)), ("scene_side", (1280, 720))])
def test_the_warp_is_opencvs_lens_model(camera, size):
    """Dots at known ideal points come out where cv2.projectPoints puts them with the camera's true values."""
    truth = L.with_values(L.lens_for(camera, seed=3), **QUIET)
    lens = L.Lens(truth, *size)
    pts = [(x, y) for x in np.linspace(-0.85, 0.85, 7) for y in np.linspace(-0.45, 0.45, 5)]
    out = lens.shoot(dots(pts))
    want, _ = cv2.projectPoints(np.array([[x, y, 1.0] for x, y in pts]), np.zeros(3), np.zeros(3), truth.K(*size), truth.dist)
    for (x, y), w in zip(pts, want.reshape(-1, 2)):
        if 14 < w[0] < size[0] - 14 and 14 < w[1] < size[1] - 14:
            assert np.linalg.norm(centroid(out, w) - w) < 0.35, (x, y)


def test_straight_lines_come_out_curved_and_bowed_outward():
    """A horizontal line above the middle: through a barrel lens its ends are nearer the middle row than its centre."""
    lens = L.Lens(L.with_values(L.LensTruth(), **QUIET), 1920, 1080)

    def render(width, height, fovy_deg):
        f = 0.5 * height / math.tan(math.radians(fovy_deg) / 2)
        img = np.full((height, width, 3), 255, np.uint8)
        row = int(round(-0.45 * f + height / 2 - 0.5))
        img[row - 1:row + 2] = 0
        return img
    out = lens.shoot(render)[..., 0].astype(float)
    rows = np.array([np.average(np.arange(1080), weights=255 - out[:, c]) for c in (40, 960, 1880)])
    assert rows[1] < rows[0] - 4 and rows[1] < rows[2] - 4, f"the line is straight or bends inward: rows {rows}"
    assert abs(rows[0] - rows[2]) < 1.0, "a centred lens bends a line evenly"
    ideal = L.Lens(L.IDEAL, 1920, 1080).shoot(render)[..., 0].astype(float)
    flat = np.array([np.average(np.arange(1080), weights=255 - ideal[:, c]) for c in (40, 960, 1880)])
    assert np.ptp(flat) < 0.2, "the ideal lens must leave the line straight"


def test_corners_are_darker_and_the_middle_is_not():
    truth = L.with_values(L.LensTruth(), blur_centre_px=0.0, blur_corner_px=0.0, read_noise=0.0, shot_noise=0.0)
    out = L.Lens(truth, 1920, 1080).shoot(lambda w, h, fovy: np.full((h, w, 3), 200, np.uint8)).astype(float)
    assert abs(out[540, 960].mean() - 200) < 1.5
    assert abs(out[5, 5].mean() - 200 * (1 - truth.vignette)) < 4
    assert out[540, 100].mean() < out[540, 500].mean() < out[540, 960].mean() + 0.5


def test_noise_differs_frame_to_frame_and_leaves_the_level_alone():
    lens = L.Lens(L.LensTruth(vignette=0.0), 640, 360)
    flat = lambda w, h, fovy: np.full((h, w, 3), 120, np.uint8)        # noqa: E731
    a = lens.shoot(flat, np.random.default_rng(1)).astype(float)
    b = lens.shoot(flat, np.random.default_rng(2)).astype(float)
    again = lens.shoot(flat, np.random.default_rng(1)).astype(float)
    assert np.array_equal(a, again) and not np.array_equal(a, b)
    assert abs(a.mean() - 120) < 0.3 and 1.0 < a.std() < 8.0


def test_every_camera_has_its_own_lens_and_none_is_the_nominal_one():
    names = ["wrist", "scene_front", "scene_side", "camera_1", "camera_2", "rover_left", "rover_right"]
    lenses = [L.lens_for(n, seed=0) for n in names]
    assert [L.lens_for(n, seed=0) for n in names] == lenses, "a camera's lens must be the same every time"
    assert len({(l.fx, l.cx, l.k1) for l in lenses}) == len(names)
    assert L.lens_for("wrist", seed=1) != L.lens_for("wrist", seed=0)
    nominal = builtin.LENSES["cm3_wide"]
    for l in lenses:
        assert abs(l.fx / nominal.fx - 1) < 0.04 and abs(l.cx - nominal.cx) < 25 and abs(l.k1 - nominal.dist[0]) < 0.02
        assert (l.fx, l.cx, l.cy, l.k1) != (nominal.fx, nominal.cx, nominal.cy, nominal.dist[0])


def test_the_world_gives_pictures_through_the_lens_unless_asked_for_the_ideal_one():
    w = World("open_small")
    a, b = w.render("camera_1", 640, 360), w.render("camera_1", 640, 360)
    ideal = w.render("camera_1", 640, 360, ideal=True)
    assert not np.array_equal(a, b), "two frames of a still scene should differ by their noise"
    assert np.array_equal(ideal, w.render("camera_1", 640, 360, ideal=True))
    assert float(np.abs(a.astype(float) - ideal).mean()) > 2.0
    assert a[4:20, 4:20].mean() < ideal[4:20, 4:20].mean() + 1.0
    with pytest.raises(KeyError):
        w.render("no_such_camera")


@pytest.mark.parametrize("seed", range(12))
def test_cameras_are_still_located_through_the_lens_within_what_is_stated(seed):
    """Through its own lens, corrected with the camera kind's nominal values only: within 6 % of the distance
    and 2.5 deg at full size, and the stated uncertainty still covers the error."""
    rng = np.random.default_rng(seed)
    arm = np.array([0.14, 0.27])
    bearing, r, h = rng.uniform(-math.pi, math.pi), rng.uniform(0.35, 0.8), rng.uniform(0.18, 0.5)
    at = [*(arm + r * np.array([math.cos(bearing), math.sin(bearing)])), h]
    look = [*(arm + rng.uniform(-0.04, 0.04, 2)), rng.uniform(0.0, 0.06)]
    s, cam = scene_with_camera(at, look)
    s["surface"]["outline_m"] = [[-0.9, -0.8], [1.2, -0.8], [1.2, 1.3], [-0.9, 1.3]]
    s["imperfections"] = {"seed": seed}
    w = World(s)
    fix = locate.camera_from_arm(w.render(cam))
    if fix is None or not fix.trusted:
        pytest.skip("too little of the arm's tags in view to trust a fix (said so, not guessed)")
    truth = w.camera_pose(cam, "base")
    e, a = errors(fix, truth)
    assert e < 0.06 * np.linalg.norm(truth[:3, 3]) and a < 2.5, (e, a, fix.tags)
    assert e < 3.5 * fix.sigma_m
