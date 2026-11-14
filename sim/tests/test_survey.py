"""The start-up survey: depth from the rover's two cameras, the map of the place, the rover in the locator.

Aimed at classes of bugs:
  * a map in the wrong frame: mirrored, turned, measured from the wrong origin (seen-free cells outside the
    true surface, nothing found where a thing stands), checked against the referee in whole scenes and
    with made-up clouds turned every which way;
  * depth that is confidently wrong: things nearer than the matcher's search reaches, rows that do not
    line up between two different lenses;
  * the hub knowing what it was not told: a map before anything has looked;
  * links through the rover that are surer than they are right, or that forget it has driven since;
  * a survey that moves what it promised not to (the wheels), leaves the head up, or trusts a map made
    while the rover was being pushed round by its own head.
"""
import json
import math
from types import SimpleNamespace

import cv2
import mujoco
import numpy as np
import pytest

from hub import api as A
from hub import geometry as G
from hub import survey as S
from hub.builtin import Lens
from hub.bus import hx_registers as R
from hub.hub import Hub
from hub.locator import Locator
from hub.place import CELL_M, FREE, HALF_CELLS, OCCUPIED, UNSEEN, Place
from hub.rover.device import HEAD_TORQUE_LIMIT, PAN_TORQUE_LIMIT
from hub.vision import stereo
from so101sim import scenes
from so101sim.world import World

NOMINAL = Lens(1920, 1080, 982.1, 982.1, 959.5, 539.5, (-0.033, 0.0, 0.0, 0.0, 0.0))


def call(hub, method, path, body=None):
    status, ctype, out = A.dispatch(hub, method, path, b"" if body is None else json.dumps(body).encode())
    return status, (json.loads(out) if ctype == "application/json" else out)


def rover_frame(world) -> np.ndarray:
    """The referee's: the rover's frame in the scene's."""
    m, d = world.model, world.data
    i = m.body("rover").id
    return G.pose(d.xmat[i].reshape(3, 3), d.xpos[i])


def loose_things(world, T_rover) -> dict[str, np.ndarray]:
    """The referee's: every loose thing's centre, in the rover's frame."""
    m, d = world.model, world.data
    return {m.body(i).name: (G.inv(T_rover) @ np.append(d.xpos[i], 1.0))[:3] for i in range(m.nbody)
            if m.body(i).parentid[0] == 0 and m.body_dofnum[i] == 6 and m.body(i).name != "rover"}


def outline(world, T_rover) -> np.ndarray:
    """The referee's: the surface's outline in the rover's frame (N x 2)."""
    pts = np.array(world.scene["surface"]["outline_m"], float)
    return np.array([(G.inv(T_rover) @ [x, y, 0.0, 1.0])[:2] for x, y in pts], np.float32)


def cell_xy(i, j):
    return (np.asarray(i) - HALF_CELLS + 0.5) * CELL_M, (np.asarray(j) - HALF_CELLS + 0.5) * CELL_M


@pytest.fixture(scope="module")
def no_arm():
    """A surface with a rover, two cameras and a few things; surveyed once, on request."""
    world = World("no_arm")
    hub = Hub(world.ports)
    before = call(hub, "GET", "/place/state")[1]
    T0 = rover_frame(world)
    status, result = call(hub, "POST", "/survey")
    assert status == 200, result
    return SimpleNamespace(world=world, hub=hub, before=before, T0=T0, result=result)


@pytest.fixture(scope="module")
def bench():
    """The bench, with an arm: surveyed at start-up, as run_system.py --survey does."""
    world = World("bench")
    T0 = rover_frame(world)
    hub = Hub(world.ports, survey=True)
    return SimpleNamespace(world=world, hub=hub, T0=T0)


# ---------------------------------------------------------------- depth from two pictures
def true_depth(world, camera: str, lens: Lens, width: int, height: int) -> np.ndarray:
    m, d = world.model, world.data
    cam = m.camera(camera).id
    keep = float(m.cam_fovy[cam])
    m.cam_fovy[cam] = math.degrees(2 * math.atan(lens.height / 2 / lens.fy))
    ren = mujoco.Renderer(m, height, width)
    ren.enable_depth_rendering()
    ren.update_scene(d, camera=cam)
    out = ren.render().copy()
    ren.close()
    m.cam_fovy[cam] = keep
    return out


@pytest.mark.parametrize("pan", [-40.0, 80.0])
@pytest.mark.parametrize("height,tilt,spacing", S.PASSES)
def test_depth_from_perfect_pictures_is_the_true_depth_near_and_far(height, tilt, spacing, pan):
    """Each of the survey's passes, over the part of the view it uses. Nearer than the fine pass's search reaches included:
    a thing there must not come back matched to something else, far away."""
    world = World("no_arm")
    r = Hub(world.ports).rover
    for step in (lambda: r.raise_head(height), lambda: r.spread(spacing), lambda: r.point(tilt_deg=tilt), lambda: r.point(pan_deg=pan)):
        assert step()["reached"]
    assert r.settle()
    pinhole = Lens(1920, 1080, NOMINAL.fx, NOMINAL.fy, NOMINAL.cx, NOMINAL.cy, (0.0,) * 5)
    left, right = (world.render(c, 1920, 1080, ideal=True) for c in ("rover_left", "rover_right"))
    cloud = stereo.depth(left, right, pinhole, pinhole, r.rover.lens_spacing(r.head_now()))
    assert cloud is not None and abs(cloud.row_shift_px) < 0.5 and abs(cloud.row_stretch - 1) < 0.002
    truth = true_depth(world, "rover_left", pinhole, 960, 540)[cloud.pixels[:, 1], cloud.pixels[:, 0]]
    used = (np.linalg.norm(cloud.points, axis=1) < S.REACH_PER_SPACING * spacing) & (np.abs(cloud.points[:, 0]) < S.USE_FIELD * cloud.points[:, 2])
    # Not judged: picture points beside an edge with only empty background behind them (the matcher gives them the edge's
    # depth, which puts them on the edge), and what is so near that the cameras see it from over 20 deg apart (stereo.py
    # says what happens there; the map's answer to it is tested below).
    nearest = 2.7 * spacing
    used &= (truth < 1.5) & (truth > nearest)
    ratio, truth = (cloud.points[:, 2] / truth)[used], truth[used]
    assert len(ratio) > 8000
    assert float(np.mean(np.abs(ratio - 1) > 0.10)) < 0.05, "too many points far from where they are"
    for lo, hi in ((0.08, 0.2), (0.2, 0.3), (0.3, 0.45), (0.45, 0.7)):
        band = (truth > lo) & (truth < hi)
        if band.sum() > 2000:
            assert abs(float(np.median(ratio[band])) - 1) < 0.02, (lo, hi, float(np.median(ratio[band])))


def test_two_different_lenses_have_their_rows_lined_up_from_the_pictures():
    world = World("no_arm")
    r = Hub(world.ports).rover
    for step in (lambda: r.raise_head(0.16), lambda: r.point(tilt_deg=-38.0)):
        assert step()["reached"]
    assert r.settle()
    l, rt = world.lens("rover_left").truth, world.lens("rover_right").truth
    assert abs(l.cy - rt.cy) > 2.0, "this scene's two lenses are too alike to test with"
    left, right = (c.grab().image for c in (r.cameras["left"], r.cameras["right"]))
    cloud = stereo.depth(left, right, NOMINAL, NOMINAL, r.rover.lens_spacing(r.head_now()))
    assert cloud is not None
    assert cloud.row_shift_px == pytest.approx((rt.cy - l.cy) / 2, abs=1.0)        # (matched at half size)
    assert cloud.row_stretch < 1.0 if rt.fx < l.fx else cloud.row_stretch > 1.0


def project(points, lens: Lens, baseline=0.0):
    px, _ = cv2.projectPoints(np.asarray(points, float) - [baseline, 0.0, 0.0], np.zeros(3), np.zeros(3), lens.K(), np.asarray(lens.dist, float))
    return px.reshape(-1, 2)


@pytest.mark.parametrize("seed", range(4))
def test_triangulation_puts_points_back_where_they_were(seed):
    rng = np.random.default_rng(seed)
    pts = np.c_[rng.uniform(-0.2, 0.2, 30), rng.uniform(-0.1, 0.1, 30), rng.uniform(0.15, 0.8, 30)]
    b = float(rng.uniform(0.06, 0.2))
    got = stereo.triangulate(project(pts, NOMINAL), project(pts, NOMINAL, b), NOMINAL, NOMINAL, b)
    assert np.abs(got - pts).max() < 1e-4
    off = float(rng.uniform(-8, 8))                               # a pair whose right picture is shifted, and says so
    got = stereo.triangulate(project(pts, NOMINAL), project(pts, NOMINAL, b) - [off, 0.0], NOMINAL, NOMINAL, b, offset_px=off)
    assert np.abs(got - pts).max() < 1e-4
    wrong = stereo.triangulate(project(pts, NOMINAL), project(pts, NOMINAL, b) - [off, 0.0], NOMINAL, NOMINAL, b)
    assert abs(off) < 1 or np.abs(wrong - pts).max() > 1e-3


def test_a_tag_measures_its_printed_size_across_the_pair_and_a_wrong_spacing_shows():
    side, b = 0.038, 0.08
    corners = np.array([[-side / 2, -side / 2, 0.3], [side / 2, -side / 2, 0.3], [side / 2, side / 2, 0.3], [-side / 2, side / 2, 0.3]])
    tag = lambda px: SimpleNamespace(id=4, corners=px, side_px=float(np.linalg.norm(px[0] - px[1])))      # noqa: E731
    left, right = [tag(project(corners, NOMINAL))], [tag(project(corners, NOMINAL, b))]
    assert S.measure_tags(left, right, NOMINAL, NOMINAL, b)[4] == pytest.approx(0.038 / S.builtin.tags("arm")[4].size_m, rel=2e-3)
    assert S.measure_tags(left, right, NOMINAL, NOMINAL, b * 1.05)[4] == pytest.approx(1.05 * 0.038 / S.builtin.tags("arm")[4].size_m, rel=2e-3)
    assert S.measure_tags(left, [], NOMINAL, NOMINAL, b) == {}


@pytest.mark.parametrize("seed", range(4))
def test_the_ground_is_found_in_a_cloud_whatever_else_is_in_it(seed):
    rng = np.random.default_rng(seed)
    tilt, h = math.radians(rng.uniform(10, 45)), float(rng.uniform(0.08, 0.17))
    down = np.array([0.0, math.cos(tilt), math.sin(tilt)])              # the camera looks tilt below level
    e1, e2 = np.array([1.0, 0.0, 0.0]), np.cross(down, [1.0, 0.0, 0.0])
    a, b = rng.uniform(-0.3, 0.3, 40_000), rng.uniform(-0.4, 0.4, 40_000)
    ground = h * down + a[:, None] * e1 + b[:, None] * e2 + rng.normal(0, 0.0007, (40_000, 3))
    box = ground[:8000] - down * 0.04                                   # something standing 40 mm above it
    m = stereo.ground_plane(np.r_[ground, box], down, h)
    assert 1 / np.linalg.norm(m) == pytest.approx(h, abs=0.001)
    assert math.degrees(math.acos(float(m @ down) / np.linalg.norm(m))) < 0.3
    c, s_ = math.cos(math.radians(1.0)), math.sin(math.radians(1.0))
    lean = np.array([[1.0, 0.0, 0.0], [0.0, c, -s_], [0.0, s_, c]])                         # the rover's idea of level is a degree off
    m = stereo.ground_plane(np.r_[ground, box], lean @ down, h + 0.002)
    assert 1 / np.linalg.norm(m) == pytest.approx(h, abs=0.001), "the plane is the one in the cloud, not the one expected"
    assert stereo.ground_plane(box[:3000], down, h) is None


# ---------------------------------------------------------------- the place, from made-up clouds
def made_up(yaw=0.0, shift=(0.0, 0.0), level=0.0):
    """Flat ground 0.5 x 0.6 m ahead of the camera, a box 60 x 40 x 50 mm on it, and a raised edge 20 mm high along its far side."""
    g = np.mgrid[0.10:0.60:0.002, -0.30:0.30:0.002].reshape(2, -1).T
    box = np.mgrid[0.32:0.38:0.002, 0.08:0.12:0.002].reshape(2, -1).T
    lip = np.mgrid[0.60:0.608:0.002, -0.30:0.30:0.002].reshape(2, -1).T
    g = g[~((g[:, 0] > 0.32) & (g[:, 0] < 0.38) & (g[:, 1] > 0.08) & (g[:, 1] < 0.12))]
    pts = np.r_[np.c_[g, np.zeros(len(g))], np.c_[box, np.full(len(box), 0.05)], np.c_[lip, np.full(len(lip), 0.02)]]
    R = G.rot_z(yaw)[:3, :3]
    pts = pts @ R.T + [shift[0], shift[1], level]
    return pts, R @ np.array([0.0, 0.0, 0.15]) + [shift[0], shift[1], 0.0], R


def test_a_place_is_empty_until_something_is_seen():
    place = Place()
    assert place.empty and (place.cells() == UNSEEN).all()
    assert place.summary()["found"] == "nothing yet"
    assert place.things() == [] and place.surface() == {"seen": False}
    assert place.unseen()["share_of_the_box_around_what_was_seen"] == 1.0
    assert place.add_cloud(np.c_[np.random.default_rng(0).uniform(0, 1, (200, 2)), np.zeros(200)], np.zeros(3))["used"] is False
    assert place.empty, "a cloud with too little ground in it to level by must add nothing"


@pytest.mark.parametrize("seed", range(6))
def test_what_is_found_is_where_it_was_put_however_the_map_is_turned(seed):
    """The class: x and y swapped, a mirrored axis, rows for columns. Nothing in a square-on layout shows those."""
    rng = np.random.default_rng(seed)
    yaw, shift, level = float(rng.uniform(-math.pi, math.pi)), rng.uniform(-0.2, 0.2, 2), float(rng.uniform(-0.006, 0.006))
    pts, cam, R = made_up(yaw, shift, level)
    place = Place()
    got = place.add_cloud(pts, cam)
    assert got["used"] and got["surface_read_at_mm"] == pytest.approx(level * 1000, abs=0.5)
    to_map = lambda p: R[:2, :2] @ np.asarray(p) + shift                                    # noqa: E731
    things = place.things()
    assert len(things) == 1, things
    t = things[0]
    assert np.linalg.norm(np.array(t["at_m"]) - to_map([0.35, 0.10])) < 0.012
    assert t["height_m"] == pytest.approx(0.05, abs=0.003)
    assert t["length_m"] == pytest.approx(0.06, abs=0.025) and t["width_m"] == pytest.approx(0.04, abs=0.025)
    edges = place.surface()["edges_found"]
    assert len(edges) == 1 and edges[0]["kind"] == "edge of the surface", edges
    e = edges[0]
    assert e["length_m"] == pytest.approx(0.6, abs=0.05) and e["height_m"] == pytest.approx(0.02, abs=0.003)
    for end in (e["from_m"], e["to_m"]):
        back = R[:2, :2].T @ (np.array(end) - shift)
        assert abs(back[0] - 0.604) < 0.015 and abs(abs(back[1]) - 0.3) < 0.04
    assert np.array(e["surface_is_towards"]) @ (R[:2, :2] @ [-1.0, 0.0]) > 0.97, "the surface is on the camera's side of its edge"
    c = place.cells()
    for p, what in (([0.2, 0.0], FREE), ([0.5, -0.2], FREE), ([0.35, 0.10], OCCUPIED), ([-0.2, 0.0], UNSEEN), ([0.8, 0.0], UNSEEN)):
        i, j = (np.floor(to_map(p) / CELL_M).astype(int) + HALF_CELLS)
        assert c[i, j] == what, (p, what)
    assert place.surface()["area_seen_m2"] == pytest.approx(0.5 * 0.6, rel=0.08)
    assert place.picture()[:8] == b"\x89PNG\r\n\x1a\n"


def test_points_scattered_below_the_surface_are_not_believed():
    """What stereo makes of something too near the lenses: points all over the place, many under the ground."""
    rng = np.random.default_rng(1)
    pts, cam, _ = made_up()
    patch = (pts[:, 0] > 0.20) & (pts[:, 0] < 0.26) & (np.abs(pts[:, 1] + 0.1) < 0.03)
    pts[patch, 2] = rng.uniform(-0.06, 0.06, int(patch.sum()))
    place = Place()
    got = place.add_cloud(pts, cam)
    assert got["doubtful"] >= 20
    c = place.cells()
    i, j = (np.floor(np.array([0.23, -0.10]) / CELL_M).astype(int) + HALF_CELLS)
    assert (c[i - 1:i + 2, j - 1:j + 2] == UNSEEN).all(), "neither free nor occupied: not known"
    assert len(place.things()) == 1, "and no thing is made of it"


def test_unseen_says_which_ways_nothing_has_looked():
    place = Place()
    pts, cam, _ = made_up()
    place.add_cloud(pts, cam)
    gaps = place.unseen()["directions_never_looked_deg"]
    covered = lambda deg: any(a <= deg < b for a, b in gaps)                               # noqa: E731
    assert covered(180 - 1) and covered(-179) and covered(90) and covered(-90), gaps      # behind, and to both sides
    assert not covered(0) and not covered(20) and not covered(-20), gaps                  # ahead was seen


# ---------------------------------------------------------------- the rover in the locator
def test_nothing_links_a_rover_nobody_has_seen():
    clock = SimpleNamespace(now=lambda: 0.0)
    loc = Locator(clock)
    loc.reset(False, {"camera_1": NOMINAL}, rover_now=lambda: ({"pan": 0.0, "lift": 0.0, "tilt": 0.0, "spacing": 0.0}, np.zeros(3)))
    assert set(loc.names()) == {"camera_1", "rover", "rover/head/left", "rover/head/right"}
    out = loc.where("rover", "camera_1")
    assert out["known"] is False and "rover" in out["why"]
    with pytest.raises(KeyError):
        loc.where("arm", "rover")                                  # there is no arm here to ask about


@pytest.mark.parametrize("name", ["no_arm", "bench", "shelf", "cluttered"])
def test_cameras_that_see_the_rovers_tags_place_it_within_what_they_state(name):
    world = World(name)
    hub = Hub(world.ports)
    assert call(hub, "POST", "/locate")[0] == 200
    T_rover = rover_frame(world)
    by_serial = {c.identity["serial"]: c.camera for c in world.ports().cameras}
    known = 0
    for cam_name, cam in hub.cameras.items():
        out = call(hub, "GET", f"/where?of={cam_name}&relative_to=rover")[1]
        if not out["known"]:
            assert out["why"]
            continue
        known += 1
        truth = G.inv(T_rover) @ world.camera_pose(by_serial[cam.serial])
        err = float(np.linalg.norm(np.array(out["position_m"]) - truth[:3, 3]))
        assert err < 3.5 * out["uncertainty_m"] + 0.003, (name, cam_name, err, out["uncertainty_m"])
        assert any("rover's tags" in s for s in out["support"])
        back = call(hub, "GET", f"/where?of=rover&relative_to={cam_name}")[1]              # the same link read from the other end
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = np.array([out["axes"][k] for k in "xyz"]).T, out["position_m"]
        assert np.abs(np.array(back["position_m"]) - G.inv(T)[:3, 3]).max() < 2e-3
        assert back["uncertainty_m"] == pytest.approx(out["uncertainty_m"], rel=0.5)
    assert known >= 1 or name in ("shelf", "cluttered"), f"no camera in '{name}' placed the rover"


def test_two_cameras_that_both_see_the_rover_are_related_through_it():
    """No arm: before the rover, nothing linked these two. The chain's uncertainty must cover its error."""
    world = World("no_arm")
    hub = Hub(world.ports)
    result = call(hub, "POST", "/locate")[1]
    assert all(c["located"] and c["relative_to"] == "rover" for c in result["cameras"].values()), result
    out = call(hub, "GET", "/where?of=camera_2&relative_to=camera_1")[1]
    assert out["known"] and len(out["support"]) == 2 and all("rover's tags" in s for s in out["support"])
    pose = {n: world.camera_pose(next(c.camera for c in world.ports().cameras if c.identity["serial"] == hub.cameras[n].serial))
            for n in ("camera_1", "camera_2")}
    truth = G.inv(pose["camera_1"]) @ pose["camera_2"]
    one = call(hub, "GET", "/where?of=rover&relative_to=camera_1")[1]
    assert np.linalg.norm(np.array(out["position_m"]) - truth[:3, 3]) < 3.5 * out["uncertainty_m"] + 0.003
    assert out["uncertainty_m"] > one["uncertainty_m"], "a chain of two links is less sure than either"


def test_the_rovers_own_cameras_are_where_its_head_puts_them():
    world = World("no_arm")
    hub = Hub(world.ports)
    assert hub.rover.raise_head(0.14)["reached"] and hub.rover.point(pan_deg=50.0, tilt_deg=-20.0)["reached"]
    for side in ("left", "right"):
        out = call(hub, "GET", f"/where?of=rover/head/{side}&relative_to=rover")[1]
        truth = world.camera_pose(f"rover_{side}", relative_to_body="rover")
        assert out["known"] and np.abs(np.array(out["position_m"]) - truth[:3, 3]).max() < 0.003
        assert np.array(out["axes"]["z"]) @ truth[:3, 2] > math.cos(math.radians(1.5))


def test_a_sighting_is_carried_forward_by_the_wheels_and_grows_less_sure():
    s = scenes.read("no_arm")                                       # with nothing for the rover to drive into
    s["objects"], s["fixtures"] = [], []
    s.pop("tasks")
    world = World(s)
    hub = Hub(world.ports)
    call(hub, "POST", "/locate")
    cam = next(n for n in hub.cameras if call(hub, "GET", f"/where?of={n}&relative_to=rover")[1]["known"])
    before = call(hub, "GET", f"/where?of={cam}&relative_to=rover")[1]
    assert hub.rover.forward(0.08)["reached"] and hub.rover.turn(25.0)["reached"]
    after = hub.locator.where(cam, "rover")                         # (asked of the locator: the hub would look again)
    assert after["known"] and after["uncertainty_m"] > before["uncertainty_m"]
    assert any("wheels" in s for s in after["support"])
    serial = hub.cameras[cam].serial
    truth = G.inv(rover_frame(world)) @ world.camera_pose(next(c.camera for c in world.ports().cameras if c.identity["serial"] == serial))
    assert np.linalg.norm(np.array(after["position_m"]) - truth[:3, 3]) < 3.5 * after["uncertainty_m"] + 0.005
    assert np.linalg.norm(np.array(after["position_m"]) - np.array(before["position_m"])) > 0.05, "the rover moved: so did the answer"
    hub.locator.forget_rover()
    assert hub.locator.where(cam, "rover")["known"] is False


# ---------------------------------------------------------------- the survey, whole
def test_nothing_is_known_of_the_place_before_anything_has_looked(no_arm):
    assert no_arm.before["found"] == "nothing yet" and "POST /survey" in no_arm.before["how"]
    assert not {"surface", "things", "unseen", "rulers"} & set(no_arm.before)


def test_the_survey_moves_only_the_head_and_puts_it_back(no_arm, bench):
    for s in (no_arm, bench):
        T = G.inv(s.T0) @ rover_frame(s.world)
        assert np.linalg.norm(T[:2, 3]) < 0.004 and abs(math.degrees(G.yaw_of(T))) < 1.0
        assert s.hub.rover.stowed() and np.abs(s.hub.rover.pose).max() < 1e-3
    r = no_arm.result
    assert r["wheels_moved"] is False and r["map_kept"] is True and r["head_met_something_at"] == []
    assert r["stood_still"]["checked"] and r["stood_still"]["matched"] and abs(r["stood_still"]["turned_deg"]) < 1.0
    assert r["views"]["used_for_the_map"] >= 12


def test_what_was_seen_free_is_on_the_true_surface(no_arm, bench):
    for s in (no_arm, bench):
        poly = outline(s.world, s.T0).reshape(-1, 1, 2)
        i, j = np.nonzero(s.hub.place.cells() == FREE)
        x, y = cell_xy(i, j)
        assert len(i) > 1500
        outside = np.array([cv2.pointPolygonTest(poly, (float(a), float(b)), True) for a, b in zip(x, y)])
        assert float(np.mean(outside < -0.03)) < 0.05 and float(np.mean(outside < -0.06)) < 0.01, \
            "seen-free cells lie off the true surface: the map is in the wrong frame"


def test_every_loose_thing_in_range_and_in_view_is_in_the_map(no_arm):
    place = no_arm.hub.place
    things = place.things()
    for name, at in loose_things(no_arm.world, no_arm.T0).items():
        if not 0.15 < np.linalg.norm(at[:2]) < 0.40:
            continue
        near = [t for t in things if np.linalg.norm(np.array(t["at_m"]) - at[:2]) < 0.035]
        assert near, (name, at, things)
        assert min(abs(t["height_m"] - 2 * at[2]) for t in near) < 0.010, (name, near)         # (a thing resting on the surface is twice its centre high)


def test_edges_of_the_surface_lie_on_its_true_outline_with_the_surface_inside(no_arm):
    poly = outline(no_arm.world, no_arm.T0).reshape(-1, 1, 2)
    edges = [e for e in no_arm.hub.place.surface()["edges_found"] if e["kind"] == "edge of the surface"]
    on_it = lambda e: all(abs(cv2.pointPolygonTest(poly, (float(p[0]), float(p[1])), True)) < 0.07 for p in (e["from_m"], e["to_m"]))   # noqa: E731
    long = [e for e in edges if e["length_m"] >= 0.30]
    assert len(long) >= 2 and all(on_it(e) for e in long), long     # (a plain edge is the hardest thing to match: 7 cm)
    assert sum(not on_it(e) for e in edges) <= 1, edges             # a short one may be the shadowed side of something else
    for e in long:
        mid = (np.array(e["from_m"]) + e["to_m"]) / 2 + 0.08 * np.array(e["surface_is_towards"])
        assert cv2.pointPolygonTest(poly, (float(mid[0]), float(mid[1])), False) > 0, e
        assert e["height_m"] == pytest.approx(no_arm.world.scene["surface"]["lip_height_m"], abs=0.012)


def test_the_rulers_are_checked_against_each_other(no_arm, bench):
    for s in (no_arm, bench):
        lift = s.hub.place.rulers["lens_height_by_the_camera_spacing"]
        assert lift["views"] >= 8 and lift["overall"] == pytest.approx(1.0, abs=0.06)
        assert lift["agree"] == (abs(lift["overall"] - 1) <= S.RULER_TOLERANCE) and ("warning" in lift) == (not lift["agree"])
    tags = bench.hub.place.rulers.get("tags_measured_by_the_camera_spacing")
    if tags is not None:                                           # only where both cameras saw a tag large enough
        assert tags["overall"] == pytest.approx(1.0, abs=0.12) and ("warning" in tags) == (not tags["agree"])


def test_a_survey_at_start_up_relates_the_arm_to_the_rover(bench):
    hub = bench.hub
    assert not hub.place.empty and hub.last_survey
    out = hub.locator.where("arm", "rover", hub.arm.link.q())
    assert out["known"], out
    m, d = bench.world.model, bench.world.data
    truth = (G.inv(bench.T0) @ np.append(d.xpos[m.body("base").id], 1.0))[:3]
    assert np.linalg.norm(np.array(out["position_m"])[:2] - truth[:2]) < 3.5 * out["uncertainty_m"] + 0.005


def test_the_place_is_served_and_names_nothing_from_the_scene(no_arm):
    hub = no_arm.hub
    status, state = call(hub, "GET", "/place/state")
    assert status == 200 and state["surface"]["seen"] and state["views"]["used"] >= 12 and "frame" in state
    status, png = call(hub, "GET", "/place/picture")
    assert status == 200 and bytes(png)[:8] == b"\x89PNG\r\n\x1a\n"
    text = json.dumps(state) + json.dumps(no_arm.result)
    s = no_arm.world.scene
    for word in [o["name"] for o in s.get("objects", [])] + [f["name"] for f in s.get("fixtures", [])] + [s["name"]]:
        assert word not in text, word
    guide = bytes(call(hub, "GET", "/")[1]).decode()
    assert "survey" in guide and "/place/picture" in guide


def test_no_rover_no_survey():
    hub = Hub(World("workcell").ports, survey=True)                  # asked for at start-up, with nothing to do it
    assert hub.rover is None and hub.place.empty
    status, out = call(hub, "POST", "/survey")
    assert status in (400, 404), out
    with pytest.raises(S.Refused):
        S.survey(hub)


def test_a_low_battery_surveys_nothing():
    s = scenes.read("no_arm")
    next(d for d in s["devices"] if d["kind"] == "rover")["battery_charge"] = 0.02
    world = World(s)
    hub = Hub(world.ports)
    T0 = rover_frame(world)
    status, out = call(hub, "POST", "/survey")
    assert status == 409 and "battery" in json.dumps(out), out
    assert hub.place.empty and np.allclose(rover_frame(world), T0, atol=1e-3)


# ---------------------------------------------------------------- a head that meets something
def test_the_pan_pushes_with_far_less_than_the_rest_of_the_head():
    hub = Hub(World("no_arm").ports)
    r = hub.rover
    assert PAN_TORQUE_LIMIT <= HEAD_TORQUE_LIMIT / 5
    for joint, servo in r.head_servo.items():
        reg = R.BY_NAME["max_torque_limit"]
        assert reg.decode(r.group.bus.read(servo["id"], reg.addr, reg.size)) == (PAN_TORQUE_LIMIT if joint == "pan" else HEAD_TORQUE_LIMIT), joint
    assert r.raise_head(0.10)["reached"]
    for pan in (-160.0, 0.0, 160.0):                                # and still turns the head, all the way, in the open
        assert r.point(pan_deg=pan, timeout_s=12.0)["reached"], pan


def post_beside_the_rover():
    """A bare surface with a tall post where the head's end passes when it turns 75 deg to the left: behind the cameras
    as they start, so in no picture before the head gets there."""
    s = scenes.read("no_arm")
    s["objects"] = []
    s.pop("tasks")
    rover = next(d for d in s["devices"] if d["kind"] == "rover")
    yaw = math.radians(rover.get("facing_deg", 0.0))
    at = np.array(rover["at_m"][:2]) + G.rot_z(yaw)[:2, :2] @ [0.081, 0.227]
    s["fixtures"] = [{"name": "post", "kind": "box", "at_m": [float(at[0]), float(at[1])], "size_m": [0.03, 0.03, 0.30]}]
    return s


def test_a_head_that_meets_something_stops_and_the_rover_is_not_spun_round():
    """The class: a move that reports 'reached' because the body turned instead of the head."""
    world = World(post_beside_the_rover())
    hub = Hub(world.ports)
    T0 = rover_frame(world)
    assert hub.rover.raise_head(0.10)["reached"]
    out = hub.rover.point(pan_deg=120.0, timeout_s=12.0)
    assert out["reached"] is False and hub.rover.last_block == "pan" and "in the head's way" in out["why_not_reached"]
    assert abs(math.degrees(G.yaw_of(G.inv(T0) @ rover_frame(world)))) < 10.0
    assert abs(math.degrees(hub.rover.head_now()["pan"])) < 100.0


def test_a_survey_beside_a_post_says_so_and_does_not_trust_a_map_made_while_turning():
    world = World(post_beside_the_rover())
    hub = Hub(world.ports)
    T0 = rover_frame(world)
    status, r = call(hub, "POST", "/survey")
    assert status == 200, r
    turned = abs(math.degrees(G.yaw_of(G.inv(T0) @ rover_frame(world))))
    assert r["head_met_something_at"] and "note" in r and r["views"]["skipped"]
    assert turned < 12.0, "the head's push turned the rover a long way"
    still = r["stood_still"]
    assert still["checked"]
    if turned > 2.0:                                                # a turn that matters is noticed, one way or the other
        assert "warning" in still, (turned, still)
    if r["map_kept"]:
        assert not hub.place.empty and still["matched"] and abs(still["turned_deg"]) <= S.TURNED_DROP_DEG
    else:
        assert hub.place.empty and not hub.locator.rover_seen
    assert r["wheels_moved"] is False and hub.rover.stowed()


@pytest.mark.parametrize("deg", [0.0, 2.0, -5.0])
def test_a_turned_view_is_measured_and_a_different_view_is_not_matched(deg):
    world = World("no_arm")
    hub = Hub(world.ports)
    r = hub.rover
    assert r.raise_head(0.16)["reached"] and r.point(tilt_deg=-38.0)["reached"]
    before = r.cameras["left"].grab().image
    assert r.point(pan_deg=deg)["reached"]                          # the head turned stands in for the rover turned under it
    got = S.turned(before, r.cameras["left"].grab().image, NOMINAL)
    assert got is not None and got == pytest.approx(deg, abs=1.5 + 0.5 * abs(deg))
    assert r.point(pan_deg=120.0)["reached"]
    far = S.turned(before, r.cameras["left"].grab().image, NOMINAL)
    assert far is None or abs(far) > S.TURNED_DROP_DEG
