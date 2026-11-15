"""Self-calibration: devices that differ from what their drivers believe, and the hub finding by how much.

Aimed at classes of bugs:
  * a correction applied the wrong way round (a lens found long and its distances made longer still; a wheel found large
    and the count made smaller), checked against the referee's true values, in both directions;
  * a correction that is noise: every routine must say how sure it is, and what it leaves must be within that;
  * a calibration made from what cannot support it (tags all at one distance, a tag misread, a drive that ended against
    something), which must be refused or left out, not believed;
  * the simulated imperfection not being there at all, which would make every test here pass for nothing.
"""
import json
import math

import numpy as np
import pytest

from hub import api as A
from hub import calibrate as C
from hub import geometry as G
from hub.builtin import LENS_FOCAL_TOLERANCE
from hub.hub import Hub
from so101sim import scenes
from so101sim.world import World


def call(hub, method, path, body=None):
    status, ctype, out = A.dispatch(hub, method, path, b"" if body is None else json.dumps(body).encode())
    return status, (json.loads(out) if ctype == "application/json" else out)


def frame_of(world, body: str) -> np.ndarray:
    m, d = world.model, world.data
    i = m.body(body).id
    return G.pose(d.xmat[i].reshape(3, 3), d.xpos[i])


def imperfect(name: str, **how) -> dict:
    s = scenes.read(name)
    s["imperfections"] = how
    return s


def true_lens(world, hub, camera: str):
    serial = hub.cameras[camera].serial
    return world.lens(next(c.camera for c in world.ports().cameras if c.identity["serial"] == serial)).truth


# ---------------------------------------------------------------- the imperfections are there
@pytest.mark.parametrize("scale", [0.98, 1.05])
def test_a_rover_on_wheels_of_another_size_goes_that_much_further_than_it_counts(scale):
    s = imperfect("no_arm", wheel_scale=scale)
    s["objects"], s["fixtures"] = [], []
    s.pop("tasks")
    world = World(s)
    hub = Hub(world.ports)
    start = frame_of(world, "rover")
    r = hub.rover._drive(0.15, 0.0, 20.0)
    went = float((G.inv(start) @ frame_of(world, "rover"))[0, 3])
    assert r["reached"] and went / r["moved"]["forward_m"] == pytest.approx(scale, abs=0.012)
    assert hub.rover.wheel_scale == 1.0, "the hub is not told"


def test_tags_printed_at_another_scale_are_that_size_in_the_world():
    plain, large = World(imperfect("bench")), World(imperfect("bench", tag_scale=1.04))
    sizes = lambda w: {w.model.geom(g).name: w.model.geom_size[g][:2].copy() for g in range(w.model.ngeom)          # noqa: E731
                       if w.model.geom(g).name.startswith("tag_") and w.model.geom(g).name[4:].isdigit()}
    a, b = sizes(plain), sizes(large)
    assert len(a) >= 12 and all(np.allclose(b[k], 1.04 * a[k]) for k in a), "every tag, the arm's and the rover's"


@pytest.mark.parametrize("how", [{"wheel_scale": 1.3}, {"wheel_scale": 0.9}, {"tag_scale": 0.5}, {"wheel_scale": "big"}, {"wheels": 1.0}])
def test_an_imperfection_out_of_range_is_a_scene_problem(how):
    assert scenes.validate(imperfect("no_arm", **how)), how
    assert scenes.validate(imperfect("no_arm", wheel_scale=1.03, tag_scale=0.98, seed=4)) == []


# ---------------------------------------------------------------- the pieces
@pytest.mark.parametrize("a,sa,b,sb", [(1.0, 0.015, 1.03, 0.004), (1.0, 0.03, 0.96, 0.03), (1.0, 0.001, 1.2, 0.5)])
def test_two_values_are_weighed_by_how_sure_each_is(a, sa, b, sb):
    best, sure = C._weigh(a, sa, b, sb)
    assert min(a, b) <= best <= max(a, b) and sure < min(sa, sb)
    assert abs(best - (a if sa < sb else b)) <= abs(best - (b if sa < sb else a)) + 1e-12, "nearer the surer one"
    assert C._weigh(b, sb, a, sa) == pytest.approx((best, sure))


@pytest.mark.parametrize("seed", range(6))
def test_what_is_left_at_no_spacing_is_the_shift_and_one_misread_spacing_does_not_spoil_it(seed):
    rng = np.random.default_rng(seed)
    B = np.array(C.SPACINGS)
    growth, shift = float(rng.uniform(1500, 4000)), float(rng.uniform(-12, 12))        # a tag 0.25 to 0.65 m away
    D = growth * B + shift + rng.normal(0, 0.1, len(B))
    k, s, miss = C.shift_line(B, D)
    assert s == pytest.approx(shift, abs=0.5) and k == pytest.approx(growth, rel=0.005) and miss < 0.3
    D[int(rng.integers(len(B)))] += 9.0                             # one spacing with the tag half out of a picture
    k, s, miss = C.shift_line(B, D)
    assert s == pytest.approx(shift, abs=0.8) and miss < 0.4, "the odd one was left out"
    D2 = growth * B + shift + 6.0 * np.sin(40 * B)                  # no straight line at all
    assert C.shift_line(B, D2)[2] > 1.5, "and what is not a line is not made into one"


def test_a_camera_whose_focal_length_was_found_is_used_with_it_and_stated_more_surely():
    world = World("open_small")
    hub = Hub(world.ports)
    cam = hub.cameras["camera_1"]
    assert cam.lens(1920, 1080) == cam.nominal(1920, 1080) and cam.focal_sigma == LENS_FOCAL_TOLERANCE
    call(hub, "POST", "/locate")
    before = call(hub, "GET", "/where?of=camera_1&relative_to=arm")[1]
    cam.focal_scale, cam.focal_sigma = 1.02, 0.004
    hub.locator.new_lens("camera_1", cam.lens(1920, 1080), cam.focal_sigma)
    assert cam.lens(1920, 1080).fx == pytest.approx(1.02 * cam.nominal(1920, 1080).fx) and cam.lens(640, 360).fx == pytest.approx(1.02 * cam.nominal(640, 360).fx)
    after = call(hub, "GET", "/where?of=camera_1&relative_to=arm")[1]
    far = lambda r: float(np.linalg.norm(r["position_m"]))                                  # noqa: E731
    assert 1.005 < far(after) / far(before) < 1.022, "a longer lens puts what it sees further away, by up to as much as it is longer"
    assert after["uncertainty_m"] < before["uncertainty_m"]


@pytest.mark.parametrize("path,status,words", [
    ("/devices/camera_1/calibrate", 409, "too little"), ("/devices/rover/head/calibrate", 409, "stowed"),
    ("/devices/rover/drive/calibrate", 409, "POST /survey first"), ("/devices/camera_7/calibrate", 404, "camera_7")])
def test_a_calibration_that_has_nothing_to_go_on_is_refused_and_changes_nothing(path, status, words):
    hub = Hub(World("open_small").ports)
    got, r = call(hub, "POST", path)
    assert got == status and words in r["error"], r
    assert hub.cameras["camera_1"].focal_scale == 1.0 and hub.rover.wheel_scale == 1.0 and hub.rover.pair == {"offset_px": 0.0, "stretch": None}


def test_the_pair_is_not_calibrated_on_no_tag():
    hub = Hub(World("no_arm").ports)                                # nothing here carries a tag the rover can see
    assert hub.rover.raise_head(0.13)["reached"]
    status, r = call(hub, "POST", "/devices/rover/head/calibrate")
    assert status == 409 and "no printed tag" in r["error"]
    assert abs(hub.rover.rover.lens_spacing(hub.rover.head_now()) - 0.06) < 0.004, "the cameras are put back where they were"


# ---------------------------------------------------------------- everything, against the referee
@pytest.fixture(scope="module", params=[(3, 1.05, 1.0), (4, 1.0, 1.04)], ids=["wheels 5 % large", "tags 4 % large"])
def calibrated(request):
    """The bench with each camera's own lens, and the wheels or the tags not the size their files say. POST /calibrate."""
    seed, wheels, tags = request.param
    world = World(imperfect("bench", seed=seed, wheel_scale=wheels, tag_scale=tags))
    hub = Hub(world.ports)
    status, result = call(hub, "POST", "/calibrate")
    assert status == 200, result
    return world, hub, result, wheels, tags


def test_the_wheels_size_is_found_and_distances_are_counted_right_afterwards(calibrated):
    world, hub, result, wheels, _ = calibrated
    w = result["wheels"]
    assert w["applied"] and len([leg for leg in w["legs"] if "went_m" in leg]) >= 2, w
    assert hub.rover.wheel_scale == pytest.approx(wheels, abs=0.02), "nearer the truth than the description was, or as near"
    assert abs(hub.rover.wheel_scale - wheels) <= 3 * w["good_to"]
    start, odo = frame_of(world, "rover"), hub.rover.pose.copy()
    d, slow = 0.0, True
    for way in (0.15, -0.15):
        try:
            d, _, slow, _ = hub.guard.drive(way, 0.0)
        except C.Refused:
            continue
        if not slow and abs(d) >= 0.06:
            break
    if slow or abs(d) < 0.06:
        return                                                      # no free ground left here to check the count over
    assert hub.rover.forward(d)["reached"]
    went = float(np.linalg.norm((G.inv(start) @ frame_of(world, "rover"))[:2, 3]))
    counted = float(np.linalg.norm(hub.rover.pose[:2] - odo[:2]))
    assert counted == pytest.approx(went, rel=0.025), "what it counts is what it went"


def test_the_pairs_shift_and_focal_ratio_are_found(calibrated):
    world, hub, result, _, tags = calibrated
    p = result["pair"]
    assert p["applied"], p
    left, right = world.lens("rover_left").truth, world.lens("rover_right").truth
    assert hub.rover.pair["offset_px"] == pytest.approx(left.cx - right.cx, abs=2.5), "the two lenses' centres, one against the other"
    assert hub.rover.pair["stretch"] == pytest.approx(right.fx / left.fx, abs=0.004)
    assert p["line_misses_by_px"] < 1.0 and len(p["spacings_used"]) >= 4
    ruler = hub.place.rulers["tags_by_the_camera_spacing"]
    assert ruler["overall"] == pytest.approx(tags, abs=0.02), "the tags' true size over their printed size"
    assert ruler["agree"] == (abs(tags - 1) < 0.02) and ("warning" in ruler) == (not ruler["agree"])
    assert hub.rover.stowed(), "the head is put away afterwards"


def test_each_cameras_focal_length_is_nearer_the_truth_or_left_with_a_reason(calibrated):
    world, hub, result, _, _ = calibrated
    done = 0
    for name, r in result["lenses"].items():
        cam, truth = hub.cameras[name], true_lens(world, hub, name).fx / 982.1
        if not r["applied"]:
            assert r["why"] and cam.focal_scale == 1.0 and cam.focal_sigma == LENS_FOCAL_TOLERANCE, r
            continue
        done += 1
        assert cam.focal_sigma < LENS_FOCAL_TOLERANCE and abs(cam.focal_scale - truth) <= 3 * cam.focal_sigma, (name, cam.focal_scale, truth)
        assert abs(cam.focal_scale - truth) <= abs(1.0 - truth) + 0.008, "not much further from the truth than the datasheet was"
        where = call(hub, "GET", f"/where?of={name}&relative_to=arm")[1]
        serial = cam.serial
        pose = world.camera_pose(next(c.camera for c in world.ports().cameras if c.identity["serial"] == serial), "base")
        err = float(np.linalg.norm(np.array(where["position_m"]) - pose[:3, 3]))
        assert where["known"] and err < 3.5 * where["uncertainty_m"], (name, err, where["uncertainty_m"])
    assert done >= 1, result["lenses"]


def test_the_calibration_says_what_it_did_in_the_guide_and_the_tree(calibrated):
    _, hub, result, _, _ = calibrated
    guide = bytes(call(hub, "GET", "/")[1]).decode()
    for path in ("POST /calibrate", "/devices/camera_1/calibrate", "/devices/rover/head/calibrate", "/devices/rover/drive/calibrate"):
        assert path in guide, path
    assert result["survey"]["map_kept"] and set(result) >= {"lenses", "wheels", "pair"}
    assert json.dumps(result).count("applied") >= 3
