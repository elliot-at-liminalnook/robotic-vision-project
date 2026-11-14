"""The map from every device: the rover's views from more than one place, the arm's touch, landmarks, GET /place.

Aimed at classes of bugs:
  * an answer given in a frame it is not in: the map re-expressed for another device must be what a map made in that
    device's frame would have been, at any turn and shift;
  * a touch that reports a height the tip was not at, presses on, or moves what it touched;
  * two frames related by things that are not the same things: landmarks that fit more than one way must be refused;
  * a second view laid into the map somewhere else than the first (the wheels' count used the wrong way round);
  * the hub knowing where a thing is in a camera's picture when nothing relates that camera to the map.
"""
import json
import math
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from hub import api as A
from hub import geometry as G
from hub import landmarks
from hub.builtin import Lens
from hub.hub import Hub
from hub.locator import Locator
from hub.place import Place, colour_of, re_frame
from so101sim import scenes
from so101sim.world import World

NOMINAL = Lens(1920, 1080, 982.1, 982.1, 959.5, 539.5, (-0.033, 0.0, 0.0, 0.0, 0.0))


def call(hub, method, path, body=None):
    status, ctype, out = A.dispatch(hub, method, path, b"" if body is None else json.dumps(body).encode())
    return status, (json.loads(out) if ctype == "application/json" else out)


def frame_of(world, body: str) -> np.ndarray:
    m, d = world.model, world.data
    i = m.body(body).id
    return G.pose(d.xmat[i].reshape(3, 3), d.xpos[i])


def loose_things(world, T) -> dict[str, np.ndarray]:
    """The referee's: every loose thing's centre, in the frame T (4 x 4, in the scene's)."""
    m, d = world.model, world.data
    return {m.body(i).name: (G.inv(T) @ np.append(d.xpos[i], 1.0))[:3] for i in range(m.nbody)
            if m.body(i).parentid[0] == 0 and m.body_dofnum[i] == 6 and m.body(i).name != "rover"}


def top_of(world, name: str) -> float:
    """The referee's: how high a thing's top is above the surface."""
    o = next(o for o in world.scene["objects"] if o["name"] == name)
    return float(world.data.xpos[world.model.body(name).id][2]) + (o["size_m"][2] if o["shape"] == "box" else o["size_m"][1]) / 2


# ---------------------------------------------------------------- colour
@pytest.mark.parametrize("rgb,word", [((200, 30, 30), "red"), ((40, 60, 200), "blue"), ((30, 160, 50), "green"), ((230, 220, 40), "yellow"),
                                      ((130, 50, 180), "purple"), ((120, 120, 125), "grey"), ((20, 20, 22), "black"), ((240, 240, 235), "white"),
                                      ((110, 20, 20), "red"), ((20, 30, 110), "blue")])
def test_a_colour_gets_the_word_a_person_would_use_light_or_dark(rgb, word):
    assert colour_of(rgb) == {"colour": word, "rgb": list(rgb)}


def made_up(yaw=0.0, shift=(0.0, 0.0), box_rgb=(200, 30, 30)):
    """Flat ground ahead of the camera, a coloured box on it, and a raised edge along its far side; with each point's colour."""
    g = np.mgrid[0.10:0.60:0.002, -0.30:0.30:0.002].reshape(2, -1).T
    box = np.mgrid[0.32:0.38:0.002, 0.08:0.12:0.002].reshape(2, -1).T
    lip = np.mgrid[0.60:0.608:0.002, -0.30:0.30:0.002].reshape(2, -1).T
    g = g[~((g[:, 0] > 0.32) & (g[:, 0] < 0.38) & (g[:, 1] > 0.08) & (g[:, 1] < 0.12))]
    pts = np.r_[np.c_[g, np.zeros(len(g))], np.c_[box, np.full(len(box), 0.05)], np.c_[lip, np.full(len(lip), 0.02)]]
    rgb = np.r_[np.tile([150, 120, 80], (len(g), 1)), np.tile(box_rgb, (len(box), 1)), np.tile([90, 80, 70], (len(lip), 1))]
    R = G.rot_z(yaw)[:3, :3]
    return pts @ R.T + [shift[0], shift[1], 0.0], R @ np.array([0.0, 0.0, 0.15]) + [shift[0], shift[1], 0.0], rgb


@pytest.mark.parametrize("rgb,word", [((200, 30, 30), "red"), ((40, 60, 200), "blue"), ((230, 220, 40), "yellow")])
def test_a_thing_has_the_colour_of_its_own_points_not_the_grounds(rgb, word):
    place = Place()
    pts, cam, colours = made_up(box_rgb=rgb)
    place.add_cloud(pts, cam, colours)
    (thing,) = place.things()
    assert thing["colour"] == word and np.abs(np.array(thing["rgb"]) - rgb).max() <= 2
    bare = Place()
    bare.add_cloud(pts, cam)
    assert "colour" not in bare.things()[0], "no colours given, none claimed"


# ---------------------------------------------------------------- the map in another frame
@pytest.mark.parametrize("seed", range(6))
def test_the_map_in_another_frame_is_the_map_that_would_have_been_made_there(seed):
    """The class: a turn applied the wrong way round, a shift before the turn, a direction not turned with its points."""
    rng = np.random.default_rng(seed)
    yaw, shift = float(rng.uniform(-math.pi, math.pi)), rng.uniform(-0.3, 0.3, 2)
    T = G.pose(G.rot_z(yaw), [shift[0], shift[1], 0.0])                 # the first map's frame, in the other frame
    here, there = Place(), Place()
    here.add_cloud(*made_up())
    there.add_cloud(*made_up(yaw, shift))
    got, want = re_frame(here.summary(), T, "the other frame"), there.summary()
    assert got["frame"] == "the other frame" and len(got["things"]) == len(want["things"]) == 1
    a, b = got["things"][0], want["things"][0]
    assert np.linalg.norm(np.array(a["at_m"]) - b["at_m"]) < 0.012 and a["height_m"] == pytest.approx(b["height_m"], abs=0.002)
    assert a["colour"] == b["colour"]
    (ea,), (eb,) = got["surface"]["edges_found"], want["surface"]["edges_found"]
    ends = lambda e: sorted([tuple(e["from_m"]), tuple(e["to_m"])])                          # noqa: E731
    assert np.abs(np.array(ends(ea)) - np.array(ends(eb))).max() < 0.03
    assert np.array(ea["surface_is_towards"]) @ np.array(eb["surface_is_towards"]) > 0.97
    turn = abs((ea["direction_deg"] - eb["direction_deg"] + 90) % 180 - 90)
    assert turn < 4.0
    for k in "xy":
        assert np.abs(np.array(got["surface"]["reaches_m"][k]) - want["surface"]["reaches_m"][k]).max() < 0.03
    assert got["surface"]["area_seen_m2"] == pytest.approx(want["surface"]["area_seen_m2"], rel=0.05)
    assert re_frame(Place().summary(), T, "x") == Place().summary(), "an empty map is empty in every frame"


# ---------------------------------------------------------------- landmarks
def scattered(rng, n=3):
    """Things with different heights, well apart, and the same things as touched in a frame turned and shifted from theirs."""
    while True:
        at = rng.uniform(-0.25, 0.25, (n, 2))
        if min(np.linalg.norm(at[i] - at[k]) for i in range(n) for k in range(i)) > 0.09:
            break
    heights = 0.02 + 0.025 * np.arange(n)
    yaw, shift = float(rng.uniform(-math.pi, math.pi)), rng.uniform(-0.5, 0.5, 2)
    T = G.pose(G.rot_z(yaw), [shift[0], shift[1], 0.0])
    things = [{"at_m": list(p + rng.normal(0, 0.006, 2)), "height_m": float(h + rng.normal(0, 0.003))} for p, h in zip(at, heights)]
    touches = [{"at_m": list((T @ [p[0], p[1], 0.0, 1.0])[:2] + rng.normal(0, 0.003, 2)), "height_m": float(h + 0.004)} for p, h in zip(at, heights)]
    return things, touches, T


@pytest.mark.parametrize("seed", range(8))
def test_things_found_twice_relate_the_two_frames(seed):
    rng = np.random.default_rng(seed)
    things, touches, T = scattered(rng)
    extra = [{"at_m": [0.6, 0.6], "height_m": 0.2}, {"at_m": [-0.6, 0.1], "height_m": 0.021}]         # in the map, never touched
    bare = [{"at_m": [0.0, 0.0], "height_m": 0.0}]                                                   # a touch of the bare surface
    got = landmarks.match(touches + bare, things + extra)
    assert got[0] is not None, got
    found, sigma_m, sigma_rad, pairs = got
    assert sorted(pairs) == [(0, 0), (1, 1), (2, 2)]
    err = np.linalg.norm(found[:2, 3] - T[:2, 3])
    turn = abs((math.atan2(found[1, 0], found[0, 0]) - math.atan2(T[1, 0], T[0, 0]) + math.pi) % (2 * math.pi) - math.pi)
    lever = max(np.linalg.norm(np.array(t["at_m"])) for t in things)       # (a turn's error moves the origin by this much)
    assert turn < 3.5 * sigma_rad and err < 3.5 * (sigma_m + sigma_rad * (lever + np.linalg.norm(T[:2, 3]))), (err, turn, sigma_m, sigma_rad)
    assert turn < math.radians(6) and found[2, 2] == 1.0 and found[2, 3] == 0.0, "a turn about the vertical and a shift along the surface"


def test_landmarks_that_fit_more_than_one_way_are_refused():
    things = [{"at_m": [0.0, 0.0], "height_m": 0.03}, {"at_m": [0.2, 0.0], "height_m": 0.03}]
    touches = [{"at_m": [0.5, 0.5], "height_m": 0.033}, {"at_m": [0.5, 0.7], "height_m": 0.033}]
    got = landmarks.match(touches, things)                          # two alike: they fit either way round
    assert got[0] is None and "more than one way" in got[1]
    third = landmarks.match(touches + [{"at_m": [0.4, 0.5], "height_m": 0.064}], things + [{"at_m": [0.0, 0.1], "height_m": 0.06}])
    assert third[0] is not None and len(third[3]) == 3, "a third thing, of another height and off their line, settles it"


@pytest.mark.parametrize("touches,why", [
    ([{"at_m": [0.1, 0.1], "height_m": 0.03}], "fewer than two"),
    ([{"at_m": [0.1, 0.1], "height_m": 0.0}, {"at_m": [0.3, 0.1], "height_m": 0.0}], "fewer than two"),             # the bare surface, twice
    ([{"at_m": [0.1, 0.1], "height_m": 0.09}, {"at_m": [0.3, 0.1], "height_m": 0.12}], "do not fit"),               # heights the map has not got
    ([{"at_m": [0.1, 0.1], "height_m": 0.03}, {"at_m": [0.6, 0.1], "height_m": 0.06}], "do not fit"),               # the wrong distance apart
])
def test_touches_that_do_not_match_the_map_relate_nothing(touches, why):
    things = [{"at_m": [0.0, 0.0], "height_m": 0.03}, {"at_m": [0.2, 0.0], "height_m": 0.06}]
    got = landmarks.match(touches, things)
    assert got[0] is None and why in got[1]


def test_a_rover_placed_by_landmarks_is_carried_by_its_wheels_like_a_sighting():
    clock, odo = SimpleNamespace(now=lambda: 5.0), [np.zeros(3)]
    loc = Locator(clock)
    loc.reset(True, {}, rover_now=lambda: ({"pan": 0.0, "lift": 0.0, "tilt": 0.0, "spacing": 0.0}, odo[0]))
    assert loc.where("rover", "arm", np.zeros(6))["known"] is False
    loc.rover_placed("arm", G.pose(G.rot_z(0.5), [0.4, 0.1, 0.0]), 0.012, 0.05, "two things both have found (landmarks)")
    first = loc.where("rover", "arm", np.zeros(6))
    assert first["known"] and first["position_m"][:2] == [0.4, 0.1] and "landmarks" in first["support"][0]
    odo[0] = np.array([0.2, 0.0, 0.0])                              # it drives 0.2 m along its own x, which is turned 0.5 rad in the arm's
    later = loc.where("rover", "arm", np.zeros(6))
    assert np.allclose(later["position_m"][:2], [0.4 + 0.2 * math.cos(0.5), 0.1 + 0.2 * math.sin(0.5)], atol=1e-3)
    assert later["uncertainty_m"] > first["uncertainty_m"] and any("wheels" in s for s in later["support"])


# ---------------------------------------------------------------- the arm's touch
@pytest.fixture(scope="module")
def workcell():
    world = World("workcell")
    hub = Hub(world.ports)
    assert call(hub, "POST", "/devices/arm/gripper/set", {"opening": 0.0})[0] == 200
    return world, hub


def test_a_touch_reads_the_height_of_what_it_rests_on_and_leaves_it_there(workcell):
    world, hub = workcell
    base = frame_of(world, "base")
    n = 0
    for name, at in loose_things(world, base).items():
        if not 0.12 < np.linalg.norm(at[:2]) < 0.33:
            continue
        status, r = call(hub, "POST", "/devices/arm/probe", {"x": float(at[0]), "y": float(at[1])})
        assert status == 200 and r["touched"] is True and r["lifted_back"], (name, r)
        assert r["height_m"] == pytest.approx(top_of(world, name), abs=0.008), name
        assert np.linalg.norm(np.array(r["at_m"]) - at[:2]) < 0.012
        assert np.linalg.norm(loose_things(world, base)[name] - at) < 0.006, f"the touch moved the {name}"
        assert hub.arm.actions.tip()[2] > 0.10, "the tip is back up"
        n += 1
    assert n >= 2


def test_a_touch_of_the_bare_surface_says_nothing_stands_there(workcell):
    _, hub = workcell
    status, r = call(hub, "POST", "/devices/arm/probe", {"x": 0.2, "y": -0.12})
    assert status == 200 and r["touched"] is False and r["height_m"] == 0.0 and "nothing stands" in r["meaning"]


def test_touches_are_kept_and_given_in_the_arms_frame_until_there_is_a_map(workcell):
    world, hub = workcell
    at = next(iter(loose_things(world, frame_of(world, "base")).values()))
    assert call(hub, "POST", "/devices/arm/probe", {"x": float(at[0]), "y": float(at[1])})[1]["touched"]
    assert call(hub, "POST", "/devices/arm/probe", {"x": 0.2, "y": -0.12})[1]["touched"] is False
    place = call(hub, "GET", "/place")[1]
    assert place["found"] == "nothing yet" and "arm's own frame" in place["touched"]["frame"]
    pts = place["touched"]["points"]
    assert len(pts) == len(hub.arm.touches) >= 2 and any(not p["touched_something"] for p in pts) and any(p["touched_something"] for p in pts)
    assert all(p["height_m"] == 0.0 for p in pts if not p["touched_something"])
    assert "landmarks" not in place and call(hub, "GET", "/place?relative_to=arm")[1]["found"] == "nothing yet"


@pytest.mark.parametrize("body,status,words", [
    ({"x": 0.9, "y": 0.0}, 400, "outside"), ({"x": 0.45, "y": 0.45}, 400, "cannot be reached"), ({"y": 0.1}, 400, "required")])
def test_a_touch_that_cannot_be_made_is_refused_and_nothing_moves(workcell, body, status, words):
    _, hub = workcell
    before, kept = hub.arm.link.q().copy(), len(hub.arm.touches)
    got, r = call(hub, "POST", "/devices/arm/probe", body)
    assert got == status and words in r["error"], r
    assert np.abs(hub.arm.link.q() - before).max() < 0.01 and len(hub.arm.touches) == kept


# ---------------------------------------------------------------- one more view, from somewhere else
def test_a_look_is_refused_with_the_head_stowed_or_no_rover():
    hub = Hub(World("no_arm").ports)
    status, r = call(hub, "POST", "/devices/rover/head/look")
    assert status == 400 and "stowed" in r["error"] and hub.place.empty
    assert call(Hub(World("workcell").ports), "POST", "/devices/rover/head/look")[0] == 404


def test_a_view_from_where_the_rover_has_driven_to_lands_in_the_same_map():
    """The class: the second view placed by the wheels' count the wrong way round, or in the rover's frame as it now is."""
    world = World("no_arm")
    hub = Hub(world.ports)
    T0 = frame_of(world, "rover")
    cube = loose_things(world, T0)["red_cube"]
    r = hub.rover

    def view(pan):
        assert r.raise_head(0.16)["reached"] and r.point(tilt_deg=-38.0, pan_deg=pan)["reached"]
        status, out = call(hub, "POST", "/devices/rover/head/look")
        assert status == 200 and out["view"]["used"], out
        return out

    def the_cube(out):
        near = [t for t in out["place"]["things"] if np.linalg.norm(np.array(t["at_m"]) - cube[:2]) < 0.04]
        assert near, out["place"]["things"]
        return near[0]

    first = view(-25.0)
    assert the_cube(first)["colour"] == "red" and first["driven_since_the_map_was_begun_m"] == 0.0 and "note" not in first
    assert r.stow()["reached"] and r.turn(-20.0)["reached"] and r.forward(-0.06)["reached"]
    moved = G.inv(T0) @ frame_of(world, "rover")
    assert np.linalg.norm(moved[:2, 3]) > 0.04 and abs(math.degrees(G.yaw_of(moved))) > 12, "the rover did not move as asked"
    second = view(0.0)
    assert second["driven_since_the_map_was_begun_m"] > 0.04 and "wheels" in second["note"]
    assert second["place"]["views"]["used"] == 2
    again = the_cube(second)
    assert np.linalg.norm(np.array(again["at_m"]) - cube[:2]) < 0.035, "the second view put the cube somewhere else"
    assert len([t for t in second["place"]["things"] if t.get("colour") == "red"]) == 1, "one cube, not one per view"
    assert again["height_m"] == pytest.approx(2 * cube[2], abs=0.008)
    both = call(hub, "GET", "/place?relative_to=rover")[1]          # and in the rover's frame as it stands now
    now = (G.inv(moved) @ np.append(cube, 1.0))[:2]
    assert min(np.linalg.norm(np.array(t["at_m"]) - now) for t in both["things"]) < 0.035


# ---------------------------------------------------------------- the map for the arm, and for the cameras
@pytest.fixture(scope="module")
def bench():
    world = World("bench")
    hub = Hub(world.ports, survey=True)
    return world, hub


def test_the_map_is_given_in_the_arms_frame_with_what_stands_where_it_does(bench):
    world, hub = bench
    status, place = call(hub, "GET", "/place?relative_to=arm")
    assert status == 200 and "arm's own frame" in place["frame"] and 0 < place["placed_to_about_m"] < 0.04
    base = frame_of(world, "base")
    things = loose_things(world, base)
    for name in ("blue_cylinder", "yellow_peg"):                    # standing clear of each other, each 55 mm high
        near = [t for t in place["things"] if np.linalg.norm(np.array(t["at_m"]) - things[name][:2]) < 0.03]
        assert near and min(abs(t["height_m"] - top_of(world, name)) for t in near) < 0.012, (name, place["things"])
    stack = things["purple_cube"]                                   # on the red cube, with other things close by: found, less exactly
    assert any(np.linalg.norm(np.array(t["at_m"]) - stack[:2]) < 0.05 and t["height_m"] > 0.045 for t in place["things"]), place["things"]
    poly = np.array([(G.inv(base) @ [x, y, 0.0, 1.0])[:2] for x, y in world.scene["surface"]["outline_m"]], np.float32).reshape(-1, 1, 2)
    long = [e for e in place["surface"]["edges_found"] if e["kind"] == "edge of the surface" and e["length_m"] >= 0.3]
    assert long
    for e in long:
        for end in (e["from_m"], e["to_m"]):
            assert abs(cv2.pointPolygonTest(poly, (float(end[0]), float(end[1])), True)) < 0.07, e
    own = [t for t in place["things"] if "this_is" in t]
    assert own and all(np.linalg.norm(t["at_m"]) < 0.45 for t in own), "what the rover saw of the arm is said to be the arm"
    assert not any("this_is" in t for t in place["things"] if np.linalg.norm(t["at_m"]) > 0.5)


def test_each_thing_is_where_the_map_says_in_each_cameras_picture(bench):
    world, hub = bench
    place = call(hub, "GET", "/place?relative_to=arm")[1]
    base = frame_of(world, "base")
    serial = {c.identity["serial"]: c.camera for c in world.ports().cameras}
    checked = 0
    for t in place["things"]:
        if "this_is" in t:
            continue
        for cam, px in t.get("in_picture", {}).items():
            name = serial[hub.cameras[cam].serial]
            p = G.inv(world.camera_pose(name)) @ base @ [t["at_m"][0], t["at_m"][1], t["height_m"], 1.0]      # the same point, by the referee
            lens = world.lens(name).truth
            true_px = cv2.projectPoints(p[:3].reshape(1, 3), np.zeros(3), np.zeros(3), lens.K(), lens.dist)[0].reshape(2)
            assert np.linalg.norm(true_px - px) < 45, (t["at_m"], cam, px, true_px)             # (45 px: 2 to 3 cm at these distances)
            checked += 1
    assert checked >= 4


def test_the_map_is_not_given_in_a_frame_nothing_relates_it_to(bench):
    _, hub = bench
    assert call(hub, "GET", "/place?relative_to=camera_9")[0] == 404
    seen, look = hub.locator.rover_seen, hub._look
    hub.locator.rover_seen, hub._look = [], lambda: {}             # as if nothing had seen the rover, and could not now
    try:
        status, r = call(hub, "GET", "/place?relative_to=arm")
        assert status == 409 and "cannot be given in arm's frame" in r["error"]
        own = call(hub, "GET", "/place")[1]
        assert own["things"] and not any("in_picture" in t or "this_is" in t for t in own["things"]), "nothing claimed that needs the link"
    finally:
        hub.locator.rover_seen, hub._look = seen, look


# ---------------------------------------------------------------- landmarks, whole
def test_the_arm_and_the_rover_are_related_by_things_one_touched_and_the_other_saw():
    """No fixed camera, and neither has the other's tags: only the two things on the surface between them."""
    s = scenes.read("open_small")
    s.pop("tasks")
    s["surface"]["outline_m"] = [[0.0, 0.0], [0.9, 0.0], [0.9, 0.55], [0.0, 0.55]]
    s["devices"] = [d for d in s["devices"] if d["kind"] != "camera"]
    next(d for d in s["devices"] if d["kind"] == "rover").update(at_m=[0.74, 0.27], facing_deg=180)
    for o, at in zip(s["objects"], ([0.40, 0.39], [0.36, 0.13])):
        o["at_m"] = at
    world = World(s)
    hub = Hub(world.ports)
    assert call(hub, "POST", "/survey")[1]["map_kept"]
    hub.locator.forget_rover()                                      # as if the arm's tags had been out of the rover's sight
    hub._look = lambda: {}
    assert call(hub, "GET", "/where?of=arm&relative_to=rover")[1]["known"] is False
    assert call(hub, "GET", "/place?relative_to=arm")[0] == 409
    base, rover = frame_of(world, "base"), frame_of(world, "rover")
    call(hub, "POST", "/devices/arm/gripper/set", {"opening": 0.0})
    things = loose_things(world, base)
    for n, (name, at) in enumerate(things.items()):
        assert call(hub, "POST", "/devices/arm/probe", {"x": float(at[0]), "y": float(at[1])})[1]["touched"], name
        if n == 0:                                                  # one thing is not enough
            assert call(hub, "GET", "/place?relative_to=arm")[0] == 409
            assert "fewer than two" in call(hub, "GET", "/place")[1]["landmarks"]
    status, place = call(hub, "GET", "/place?relative_to=arm")
    assert status == 200 and "2 things" in place["landmarks"], place.get("error")
    for name, at in things.items():
        near = [t for t in place["things"] if np.linalg.norm(np.array(t["at_m"]) - at[:2]) < 0.03]
        assert near and near[0]["touched_height_m"] == pytest.approx(top_of(world, name), abs=0.008), (name, place["things"])
    where = call(hub, "GET", "/where?of=arm&relative_to=rover")[1]
    truth = G.inv(rover) @ base
    assert where["known"] and "landmarks" in where["support"][0]
    err = float(np.linalg.norm(np.array(where["position_m"])[:2] - truth[:2, 3]))
    assert err < 0.03 and err < 3.5 * where["uncertainty_m"], (err, where)
