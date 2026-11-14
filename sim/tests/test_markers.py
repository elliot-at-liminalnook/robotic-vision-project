"""Printed tags: the artwork, where it is on the arm, and locating a camera from it.

Aimed at classes of bugs:
  * the simulated tag is not the printed tag, or not where the hub believes it is (artwork and
    layout are each in one place and everything else is checked against it);
  * a tag that is mirrored or turned on its plate: each tag alone must give the right pose,
    orientation included, and a tampered one must not yield a trusted wrong answer;
  * a flat tag read tilted the wrong way and believed;
  * an uncertainty that is not honest: the true error must lie inside what is stated.
"""
import copy
import math

import cv2
import mujoco
import numpy as np
import pytest

from hub import builtin, locate
from hub import geometry as G
from hub.vision import tags as V
from so101sim import scenes
from so101sim.config import MODEL_DIR, ROOT
from so101sim.world import World

ARM_TAGS = builtin.tags("arm")
BASE_TAGS = sorted(i for i, t in ARM_TAGS.items() if t.on == "base")
PINHOLE = builtin.LENSES["pinhole"]
LIMIT = {1080: (0.035, 2.0), 720: (0.08, 4.0)}           # stated error of a trusted fix: fraction of the camera's distance, degrees


def scene_with_camera(at, look, name="open_small"):
    s = scenes.read(name)
    cam = next(d for d in s["devices"] if d["kind"] == "camera")
    cam["at_m"], cam["looking_at_m"] = [float(v) for v in at], [float(v) for v in look]
    s["devices"] = [d for d in s["devices"] if d["kind"] != "rover"]
    s.pop("tasks", None)                                      # (they are about the rover)
    return s, cam["name"]


def locate_ideal(w, cam, height=1080):
    """Locate from the ideal pinhole picture: these tests are about tags and geometry, not the lens (test_lens.py)."""
    img = w.render(cam, height * 16 // 9, height, ideal=True)
    return locate.camera_from_arm(img, PINHOLE.at(img.shape[1], img.shape[0]))


def errors(fix, truth):
    return (float(np.linalg.norm(fix.T_base_cam[:3, 3] - truth[:3, 3])),
            math.degrees(G.angle_between(fix.T_base_cam[:3, :3], truth[:3, :3])))


# ---------------------------------------------------------------- artwork and layout: one source each
def test_the_simulated_tag_is_the_printed_tag():
    """The PNG the model shows is the file in hardware/tags, and both are the family's bits for that id."""
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    for i in ARM_TAGS:
        printed = (ROOT / "hardware/tags/png" / f"tag36h11_{i}.png").read_bytes()
        assert (MODEL_DIR / "assets/tags" / f"tag36h11_{i}.png").read_bytes() == printed, f"tag {i}: the model shows another file"
        img = cv2.imdecode(np.frombuffer(printed, np.uint8), cv2.IMREAD_GRAYSCALE)
        cells = img.reshape(10, img.shape[0] // 10, 10, img.shape[1] // 10).mean(axis=(1, 3)) > 127
        assert cells[0].all() and cells[-1].all() and cells[:, 0].all() and cells[:, -1].all(), "quiet zone"
        assert np.array_equal(cells[1:-1, 1:-1], cv2.aruco.generateImageMarker(d, i, 8) > 127)
        assert f"id {i}:" in (ROOT / "hardware/tags/print/arm.svg").read_text()


def test_the_hub_believes_the_tags_are_where_the_model_has_them():
    m = mujoco.MjModel.from_xml_path(str(MODEL_DIR / "so101_follower.xml"))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    assert len(ARM_TAGS) >= 9
    for i, tag in ARM_TAGS.items():
        g = m.geom(f"tag_{i}")
        assert m.body(m.geom_bodyid[g.id]).name == tag.on
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, g.quat)
        R = R.reshape(3, 3)
        assert np.allclose(R, tag.T_part_tag[:3, :3], atol=1e-4), f"tag {i} faces or is turned another way in the model"
        assert np.allclose(g.pos - R[:, 2] * 0.0004, tag.T_part_tag[:3, 3], atol=1e-5), f"tag {i} is somewhere else in the model"
        assert abs(g.size[0] * 2 * 8 / 10 - tag.size_m) < 1e-6, f"tag {i} has another size in the model"


def test_every_tag_lies_on_the_face_of_the_part_that_carries_it():
    m = mujoco.MjModel.from_xml_path(str(MODEL_DIR / "so101_follower.xml"))
    parts = [m.geom(g) for g in range(m.ngeom) if m.geom(g).name.endswith("_collision") and "tag" in m.geom(g).name]
    assert len(parts) == 3
    for i, tag in ARM_TAGS.items():
        p, n = tag.T_part_tag[:3, 3], tag.T_part_tag[:3, 2]
        on_a_face = False
        for part in parts:
            if m.body(m.geom_bodyid[part.id]).name != tag.on:
                continue
            R = np.zeros(9)
            mujoco.mju_quat2Mat(R, part.quat)
            local, ln = R.reshape(3, 3).T @ (p - part.pos), R.reshape(3, 3).T @ n
            k = int(np.argmax(np.abs(ln)))
            flush = abs(abs(local[k]) - part.size[k]) < 1e-3 and local[k] * ln[k] > 0
            within = all(abs(local[j]) + tag.size_m / 2 <= part.size[j] + 1e-6 for j in range(3) if j != k)
            on_a_face |= bool(abs(ln[k]) > 0.999 and flush and within)
        assert on_a_face, f"tag {i} is not flat on an outer face of a tag block or plate"


# ---------------------------------------------------------------- each tag alone, seen from in front
@pytest.fixture(scope="module")
def tag_views():
    """For each base tag: a picture from 0.25 m in front of it, a little off its axis, and the camera's true pose."""
    out = {}
    arm = next(d for d in scenes.read("open_small")["devices"] if d["kind"] == "arm")
    T_scene_base = G.pose(G.rot_z(math.radians(arm["facing_deg"])), [*arm["at_m"], 0.0024])
    for i in BASE_TAGS:
        tag = ARM_TAGS[i]
        centre = (T_scene_base @ tag.T_part_tag)[:3, 3]
        normal, up = T_scene_base[:3, :3] @ tag.T_part_tag[:3, 2], T_scene_base[:3, :3] @ tag.T_part_tag[:3, 1]
        eye = centre + 0.25 * normal + 0.05 * up + 0.04 * np.cross(up, normal)
        if eye[2] < 0.03:
            eye[2] = 0.03
        s, cam = scene_with_camera(eye, centre)
        next(d for d in s["devices"] if d["kind"] == "camera")["post"] = False
        s["objects"] = []
        w = World(s)
        out[i] = (w.render(cam, 1280, 720, ideal=True), w.camera_pose(cam, "base"))
    return out


@pytest.mark.parametrize("tag_id", BASE_TAGS)
def test_each_tag_alone_gives_the_right_pose_orientation_included(tag_views, tag_id):
    """A mirrored texture, a plate turned 90 degrees or a wrong `up` in the layout each fail this for their tag."""
    image, truth = tag_views[tag_id]
    found = [d for d in V.detect(image) if d.id == tag_id]
    assert found and found[0].side_px > 60, f"tag {tag_id} is not seen from in front of it"
    fix = V.body_pose(found, {tag_id: ARM_TAGS[tag_id].corners()}, PINHOLE.at(1280, 720))
    T = G.inv(fix.T_cam_body)
    assert np.linalg.norm(T[:3, 3] - truth[:3, 3]) < 0.006 and math.degrees(G.angle_between(T[:3, :3], truth[:3, :3])) < 2.5


@pytest.mark.parametrize("how", ["mirrored", "turned"])
def test_a_tampered_tag_never_gives_a_trusted_wrong_answer(how):
    """Mirror or turn one tag's artwork in the model. The fix must either still be right (that tag ignored)
    or not be trusted: the tags no longer agree with each other."""
    s, cam = scene_with_camera([0.55, 0.50, 0.30], [0.14, 0.27, 0.02])
    w = World(s)
    clean = locate_ideal(w, cam)
    victim = clean.tags[0]
    tex = w.model.texture(f"tag_{victim}")
    n = int(tex.height[0]) * int(tex.width[0]) * int(tex.nchannel[0])
    data = w.model.tex_data[int(tex.adr[0]):int(tex.adr[0]) + n].reshape(int(tex.height[0]), int(tex.width[0]), int(tex.nchannel[0]))
    data[:] = data[:, ::-1] if how == "mirrored" else np.rot90(data)
    for r in w._renderers.values():                          # (new renderers: textures are loaded when one is made)
        r.close()
    w._renderers.clear()
    fix = locate_ideal(w, cam)
    truth = w.camera_pose(cam, "base")
    if fix is not None and fix.trusted:
        e, a = errors(fix, truth)
        assert e < LIMIT[1080][0] * np.linalg.norm(truth[:3, 3]) and a < LIMIT[1080][1], \
            f"a {how} tag {victim} gave a trusted fix {e * 1000:.0f} mm off"
        assert how == "mirrored" and victim not in fix.tags, "a turned tag must not be part of a trusted fix"


# ---------------------------------------------------------------- locating cameras
SCENES_WITH_BOTH = [n for n in scenes.names() if {"arm", "camera"} <= {d["kind"] for d in scenes.read(n)["devices"]}]


@pytest.mark.parametrize("name", SCENES_WITH_BOTH)
def test_cameras_in_every_scene_are_located_from_the_picture_alone(name):
    w = World(name)
    for cam in [d["name"] for d in w.scene["devices"] if d["kind"] == "camera"]:
        truth = w.camera_pose(cam, "base")
        for height in (1080, 720):
            fix = locate_ideal(w, cam, height)
            if fix is None or not fix.trusted:
                continue                                     # not in view, or too little to go on: said so, not guessed
            e, a = errors(fix, truth)
            assert e < LIMIT[height][0] * np.linalg.norm(truth[:3, 3]) and a < LIMIT[height][1], (cam, height, e, a)
            assert e < 3.5 * fix.sigma_m, f"{cam}: the stated uncertainty ({fix.sigma_m * 1000:.0f} mm) does not cover the error ({e * 1000:.0f} mm)"


@pytest.mark.parametrize("seed", range(12))
def test_a_camera_put_anywhere_is_followed(seed):
    """Cameras at places drawn from a seed, which no file or constant knows: the estimate follows the truth."""
    rng = np.random.default_rng(seed)
    arm = np.array([0.14, 0.27])
    bearing, r, h = rng.uniform(-math.pi, math.pi), rng.uniform(0.35, 0.8), rng.uniform(0.18, 0.5)
    at = [*(arm + r * np.array([math.cos(bearing), math.sin(bearing)])), h]
    look = [*(arm + rng.uniform(-0.04, 0.04, 2)), rng.uniform(0.0, 0.06)]
    s, cam = scene_with_camera(at, look)
    s["surface"]["outline_m"] = [[-0.9, -0.8], [1.2, -0.8], [1.2, 1.3], [-0.9, 1.3]]      # room for the camera's post anywhere
    w = World(s)
    truth = w.camera_pose(cam, "base")
    trusted = 0
    for height in (1080, 720):
        fix = locate_ideal(w, cam, height)
        if fix is None or not fix.trusted:
            continue
        trusted += 1
        e, a = errors(fix, truth)
        assert e < LIMIT[height][0] * np.linalg.norm(truth[:3, 3]) and a < LIMIT[height][1], (height, e, a, fix.tags)
        assert e < 3.5 * fix.sigma_m
    assert trusted >= 1 or h < 0.25, "a camera with a clear view of the arm's base could not be located at either size"


def test_moving_the_camera_three_centimetres_moves_the_estimate_three_centimetres():
    a, cam = scene_with_camera([0.55, 0.50, 0.30], [0.14, 0.27, 0.02])
    b = copy.deepcopy(a)
    next(d for d in b["devices"] if d["kind"] == "camera")["at_m"] = [0.58, 0.50, 0.30]
    fa, fb = (locate_ideal(World(s), cam) for s in (a, b))
    moved = fb.T_base_cam[:3, 3] - fa.T_base_cam[:3, 3]
    assert abs(moved[0] - 0.03) < 0.008 and abs(moved[1]) < 0.008 and abs(moved[2]) < 0.008, moved


def test_a_flat_reading_is_flagged_not_believed():
    """Only the two front faces in view (one plane): the fix says it cannot tell the tilt, and is not trusted."""
    s, cam = scene_with_camera([0.60, 0.27, 0.035], [0.14, 0.27, 0.02])
    next(d for d in s["devices"] if d["kind"] == "camera")["post"] = False
    s["objects"] = []
    w = World(s)
    fix = locate_ideal(w, cam, 720)
    assert fix is not None and set(fix.tags) <= {1, 5}, fix
    if len(fix.tags) == 2 and not fix.ambiguous:
        e, a = errors(fix, w.camera_pose(cam, "base"))
        assert e < 0.03 and a < 3.0, "a flat pattern read without doubt must be read right"
    else:
        assert not fix.trusted


def test_no_tag_in_view_is_no_answer():
    s, cam = scene_with_camera([0.55, 0.50, 0.30], [0.65, 0.10, 0.0])       # looking away from the arm
    assert locate_ideal(World(s), cam) is None
