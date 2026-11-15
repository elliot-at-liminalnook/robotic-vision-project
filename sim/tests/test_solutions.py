"""Tasks done through the hub's API alone: the point-in-a-picture endpoint, the solutions' eyes, and the solutions themselves.

Aimed at classes of bugs:
  * a place read off a picture in the wrong frame, or along the wrong line of sight (checked against the referee's own
    projection, for fixed cameras and for the camera on the wrist);
  * a solution that reaches past the hub's API for what it should have had to find (the import rule below);
  * a solution that answers when it should have said it cannot tell;
  * a part of a thing taken for the whole of it (a bin's frame that runs off the picture), and one patch of colour that is
    not a thing ending a task that had other patches to try;
  * the head or the hand going where the hub already knows something stands.
"""
import ast
import json
import math
from pathlib import Path

import cv2
import numpy as np
import pytest

from hub import api as A
from hub import geometry as G
from hub.hub import Hub
from so101sim import scenes
from so101sim.world import World

ROOT = Path(__file__).resolve().parents[1]


def call(hub, method, path, body=None):
    status, ctype, out = A.dispatch(hub, method, path, b"" if body is None else json.dumps(body).encode())
    return status, (json.loads(out) if ctype == "application/json" else out)


def frame_of(world, body: str) -> np.ndarray:
    m, d = world.model, world.data
    i = m.body(body).id
    return G.pose(d.xmat[i].reshape(3, 3), d.xpos[i])


def true_pixel(world, camera: str, point_world) -> np.ndarray:
    """The referee's: where a point of the scene is in a camera's full-size picture, through that camera's own lens."""
    lens = world.lens(camera).truth
    p = G.inv(world.camera_pose(camera)) @ np.append(point_world, 1.0)
    return cv2.projectPoints(p[:3].reshape(1, 3), np.zeros(3), np.zeros(3), lens.K(), lens.dist)[0].reshape(2)


# ---------------------------------------------------------------- a point in a picture, as a place
@pytest.fixture(scope="module")
def workcell():
    world = World("workcell")
    hub = Hub(world.ports)
    assert call(hub, "POST", "/locate", {"show_wrist_tag": True})[0] == 200
    return world, hub


def test_a_point_in_a_fixed_cameras_picture_is_placed_where_the_thing_is(workcell):
    world, hub = workcell
    base = frame_of(world, "base")
    serial = {c.identity["serial"]: c.camera for c in world.ports().cameras}
    checked = 0
    for o in world.scene["objects"]:
        centre = world.data.xpos[world.model.body(o["name"]).id]
        top = float(centre[2]) + (o["size_m"][2] if o["shape"] == "box" else o["size_m"][1]) / 2
        at = (G.inv(base) @ [centre[0], centre[1], top, 1.0])[:2]
        for name, cam in hub.cameras.items():
            u, v = true_pixel(world, serial[cam.serial], [centre[0], centre[1], top])
            status, r = call(hub, "GET", f"/where/pixel?camera={name}&u={u:.1f}&v={v:.1f}&height_m={top:.4f}&relative_to=arm")
            assert status == 200 and r["known"], r
            err = float(np.linalg.norm(np.array(r["at_m"]) - at))
            assert err < 0.015 and err < 2.0 * r["uncertainty_m"], (o["name"], name, err, r["uncertainty_m"])
            # a wrong height moves the place along the line of sight, by what the answer says it does
            low = call(hub, "GET", f"/where/pixel?camera={name}&u={u:.1f}&v={v:.1f}&height_m={top - 0.02:.4f}&relative_to=arm")[1]
            moved = float(np.linalg.norm(np.array(low["at_m"]) - r["at_m"]))
            assert moved == pytest.approx(0.02 / math.tan(math.radians(r["slope_deg"])), rel=0.1)
            checked += 1
    assert checked >= 4


def test_the_wrist_camera_places_what_is_under_it_to_a_few_millimetres(workcell):
    world, hub = workcell
    base = frame_of(world, "base")
    o = world.scene["objects"][0]
    centre = world.data.xpos[world.model.body(o["name"]).id].copy()
    top = float(centre[2]) + o["size_m"][2] / 2
    at = (G.inv(base) @ np.append(centre, 1.0))[:2]
    assert call(hub, "POST", "/devices/arm/move_tip", {"x": float(at[0]), "y": float(at[1]), "z": 0.14, "tolerance_m": 0.01})[0] == 200
    u, v = true_pixel(world, "wrist", [centre[0], centre[1], top])
    assert 0 <= u < 1920 and 0 <= v < 1080, "hovering over a thing, the wrist camera has it in view"
    r = call(hub, "GET", f"/where/pixel?camera=arm/wrist_camera&u={u:.1f}&v={v:.1f}&height_m={top:.4f}")[1]
    assert r["known"] and np.linalg.norm(np.array(r["at_m"]) - at) < 0.006, r
    assert r["uncertainty_m"] < 0.02
    call(hub, "POST", "/devices/arm/home")


@pytest.mark.parametrize("query,status,words", [
    ("camera=camera_1&u=5000&v=10", 400, "outside the picture"), ("camera=camera_1", 400, "which point"),
    ("camera=camera_9&u=10&v=10", 404, "camera_9"), ("camera=camera_1&u=10&v=10&relative_to=camera_2", 404, "stands on the surface"),
    ("camera=camera_1&u=10&v=10&height_m=7", 400, "height_m")])
def test_a_point_that_cannot_be_placed_is_refused_with_the_reason(workcell, query, status, words):
    _, hub = workcell
    got, r = call(hub, "GET", f"/where/pixel?{query}")
    assert got == status and words in r["error"], r


def test_a_line_of_sight_that_does_not_come_down_is_said_not_to(workcell):
    _, hub = workcell
    r = call(hub, "GET", "/where/pixel?camera=camera_1&u=960&v=2&height_m=0.9")[1]           # above the camera: no such place
    assert r["ok"] and r["known"] is False and "does not come down" in r["why"]
    assert "/where/pixel" in bytes(call(hub, "GET", "/")[1]).decode()


def test_a_camera_nothing_relates_to_the_arm_places_nothing():
    world = World("workcell")
    hub = Hub(world.ports)
    hub._look = lambda: {}
    r = call(hub, "GET", "/where/pixel?camera=camera_1&u=960&v=540")[1]
    assert r["known"] is False and "nothing relates camera_1 to arm" in r["why"]


# ---------------------------------------------------------------- a place, as a point in a picture
def test_a_place_is_put_in_the_picture_where_the_referee_sees_it(workcell):
    """The class: the reverse answered in the wrong frame, or through the wrong lens. Every loose thing, in every fixed
    camera, against the referee's own projection."""
    world, hub = workcell
    base = frame_of(world, "base")
    serial = {c.identity["serial"]: c.camera for c in world.ports().cameras}
    checked = 0
    for o in world.scene["objects"]:
        centre = world.data.xpos[world.model.body(o["name"]).id]
        at = (G.inv(base) @ np.append(centre, 1.0))
        for name, cam in hub.cameras.items():
            u, v = true_pixel(world, serial[cam.serial], centre)
            r = call(hub, "GET", f"/where/point?camera={name}&x={at[0]:.4f}&y={at[1]:.4f}&height_m={at[2]:.4f}&relative_to=arm")[1]
            assert r["known"] and r["in_picture"] == (0 <= u < 1920 and 0 <= v < 1080), (o["name"], name, r, (u, v))
            if r["in_picture"]:
                err = float(np.linalg.norm(np.array(r["pixel"]) - [u, v]))
                assert err < 40 and err < 2.5 * r["uncertainty_px"], (o["name"], name, err, r)
                checked += 1
    assert checked >= 4


def test_a_point_taken_to_a_place_and_back_is_the_same_point(workcell):
    """The class: /where/pixel and /where/point not each other's inverse (a frame the wrong way round in one of them).
    Points all over the picture, at two heights, for a fixed camera and for the camera on the wrist."""
    _, hub = workcell
    tried = 0
    for camera in ("camera_1", "arm/wrist_camera"):
        for u in (100, 500, 960, 1400, 1820):
            for v in (300, 540, 800, 1040):
                for h in (0.0, 0.05):
                    there = call(hub, "GET", f"/where/pixel?camera={camera}&u={u}&v={v}&height_m={h}")[1]
                    if not there["known"]:
                        continue
                    x, y = there["at_m"]
                    back = call(hub, "GET", f"/where/point?camera={camera}&x={x}&y={y}&height_m={h}")[1]
                    # (places are answered to a tenth of a millimetre: far along a shallow line that is a few pixels)
                    slack = 1.0 + 0.0002 / max(there["line_of_sight_m"], 0.05) * 1500 / max(math.sin(math.radians(there["slope_deg"])), 0.05)
                    assert back["in_picture"] and np.linalg.norm(np.array(back["pixel"]) - [u, v]) < slack, (camera, u, v, h, there, back)
                    tried += 1
    assert tried >= 40


def test_what_is_outside_a_cameras_picture_is_said_to_be_wherever_it_is(workcell):
    """The class: a lens model carried past the edge of its picture folds a far-off place back into it; a place behind
    the camera projects into the picture as if it were in front. Places all round the arm, near and far: in the picture
    exactly when the referee's projection (with what is behind the camera left out) says so, but for a band at the edge."""
    world, hub = workcell
    base = frame_of(world, "base")
    serial = {c.identity["serial"]: c.camera for c in world.ports().cameras}
    inside = outside = 0
    for name, cam in hub.cameras.items():
        pose = world.camera_pose(serial[cam.serial])
        # (above the surface too: behind a camera that looks down, a place higher than it mirrors into the middle of its picture)
        for bearing, reach, h in ((b, r, h) for b in range(0, 360, 15) for r in (0.15, 0.4, 1.0, 3.0) for h in (0.0, 0.4, 1.0)):
            x, y = reach * math.cos(math.radians(bearing)), reach * math.sin(math.radians(bearing))
            r = call(hub, "GET", f"/where/point?camera={name}&x={x:.4f}&y={y:.4f}&height_m={h}&relative_to=arm")[1]
            p_world = (base @ [x, y, h, 1.0])[:3]
            in_front = (G.inv(pose) @ np.append(p_world, 1.0))[2] > 0.05
            lens = world.lens(serial[cam.serial]).truth
            n = (G.inv(pose) @ np.append(p_world, 1.0))[:2] / max((G.inv(pose) @ np.append(p_world, 1.0))[2], 1e-9)
            wide = in_front and abs(n[0]) < 0.9 * lens.cx / lens.fx and abs(n[1]) < 0.9 * lens.cy / lens.fy       # well inside, by angle
            far_out = (not in_front) or abs(n[0]) > 1.3 * lens.cx / lens.fx or abs(n[1]) > 1.3 * lens.cy / lens.fy  # well outside
            assert r["known"]
            if wide:
                assert r["in_picture"], (name, bearing, reach, r)
                inside += 1
            if far_out:
                assert not r["in_picture"] and r["why"], (name, bearing, reach, r)
                outside += 1
    assert inside >= 20 and outside >= 100


@pytest.mark.parametrize("query,status,words", [
    ("camera=camera_1&x=0.2", 400, "which place"), ("camera=camera_9&x=0.2&y=0", 404, "camera_9"), ("x=0.2&y=0", 400, "required"),
    ("camera=camera_1&x=0.2&y=0&relative_to=camera_2", 404, "stands on the surface"), ("camera=camera_1&x=0.2&y=0&height_m=7", 400, "height_m"),
    ("camera=camera_1&x=nan&y=0", 400, "required"), ("camera=camera_1&x=99&y=0", 400, "within 5 m")])
def test_a_place_that_cannot_be_looked_up_is_refused_with_the_reason(workcell, query, status, words):
    _, hub = workcell
    got, r = call(hub, "GET", f"/where/point?{query}")
    assert got == status and words in r["error"], r
    assert "/where/point" in bytes(call(hub, "GET", "/")[1]).decode()


# ---------------------------------------------------------------- the solutions' eyes
def test_the_eyes_find_each_colour_where_it_is_and_tell_a_frame_from_a_block():
    from solutions import eyes
    img = np.full((400, 600, 3), (150, 120, 80), np.uint8)
    cv2.rectangle(img, (100, 100), (140, 140), (200, 30, 30), -1)                    # a red block
    cv2.rectangle(img, (300, 200), (420, 300), (40, 60, 200), 8)                     # a blue frame
    cv2.rectangle(img, (500, 50), (530, 80), (30, 160, 50), -1)                      # a green block
    red, blue, green = (eyes.find(img, c) for c in ("red", "blue", "green"))
    assert len(red) == 1 and abs(red[0].u - 120) < 2 and abs(red[0].v - 120) < 2 and not red[0].ring
    assert len(blue) == 1 and blue[0].ring and abs(blue[0].u - 360) < 4 and abs(blue[0].v - 250) < 4
    assert len(green) == 1 and not green[0].ring and eyes.find(img, "yellow") == []
    assert red[0].corners.shape == (4, 2)
    cv2.rectangle(img, (40, 250), (160, 350), (235, 232, 225), 3)                    # a thin cream frame, the surface showing through
    cv2.rectangle(img, (220, 40), (290, 110), (245, 245, 245), -1)                   # a printed tag: white, with black inside
    cv2.rectangle(img, (235, 55), (275, 95), (15, 15, 15), -1)
    pale = eyes.find(img, "cream")
    frames = [b for b in pale if b.ring]
    assert len(frames) == 1 and abs(frames[0].u - 100) < 4 and abs(frames[0].v - 300) < 4, "the frame is a bin; the tag is not"
    assert any(abs(b.u - 255) < 4 and abs(b.v - 75) < 4 and not b.ring for b in pale), "the tag is seen, and is not taken for a frame"


@pytest.mark.parametrize("corner,cut", [((300, 200), False), ((-40, 200), True), ((300, -30), True), ((520, 200), True), ((300, 330), True),
                                        ((8, 200), False), ((300, 8), False)])      # (the line is 8 px wide: 4 px clear of the edge)
def test_a_frame_that_runs_off_the_picture_is_said_to(corner, cut):
    """The class: the middle of the part of a thing that is in the picture, taken for the thing's middle."""
    from solutions import eyes
    img = np.full((400, 600, 3), (150, 120, 80), np.uint8)
    x, y = corner
    cv2.rectangle(img, (x, y), (x + 120, y + 100), (40, 60, 200), 8)
    blue = eyes.find(img, "blue")
    assert len(blue) == 1 and blue[0].cut_off == cut
    if not cut:
        assert blue[0].ring and abs(blue[0].u - (x + 60)) < 4 and abs(blue[0].v - (y + 50)) < 4


def test_white_is_not_whatever_is_pale_and_grey_is_neither_white_nor_wood():
    """The class: bare steel and bright wood taken for white (the cabinet's roof was the largest 'pale' patch in view)."""
    from solutions import eyes
    rgb = lambda h, s_, v: tuple(int(c) for c in cv2.cvtColor(np.uint8([[[h, s_, v]]]), cv2.COLOR_HSV2RGB)[0, 0])       # noqa: E731
    img = np.full((400, 700, 3), rgb(20, 90, 235), np.uint8)                         # bright wood
    cv2.rectangle(img, (50, 50), (150, 150), rgb(111, 25, 202), -1)                  # bare steel
    cv2.rectangle(img, (250, 50), (350, 150), (252, 252, 252), -1)                   # white
    cv2.rectangle(img, (450, 50), (550, 150), rgb(26, 31, 165), -1)                  # white in shadow
    cv2.rectangle(img, (250, 250), (350, 350), (250, 250, 250), -1)                  # a printed tag: white round black
    cv2.rectangle(img, (265, 265), (335, 335), (10, 10, 10), -1)
    white, grey = eyes.find(img, "white"), eyes.find(img, "grey")
    assert sorted((round(b.u, -1), round(b.v, -1), b.tag) for b in white) == [(300, 100, False), (300, 300, True)]
    assert sorted((round(b.u, -1), round(b.v, -1)) for b in grey) == [(100, 100), (500, 100)]
    assert all(b.outline is not None and len(b.outline) >= 4 for b in white + grey)


class _Cabinet:
    """An Api whose wrist camera, wherever the hand hovers, shows a grey thing with white patches as given (each a rectangle
    in px), 0.2 mm to the pixel; `tags` are the printed tags the hub finds in that picture (4 x 2 px each)."""
    def __init__(self, white, tags=(), grey=((700, 300), (1100, 700)), reachable=True):
        self.white, self.tags, self.grey, self.reachable, self.calls, self.rest = white, tags, grey, reachable, 0, None

    def devices(self):
        return {"arm": {}}

    def post(self, path, body=None):
        return (200, {"reached": True}) if self.reachable else (409, {"error": "that point cannot be reached"})

    def get(self, path):
        return 200, {"tags": [{"corners_px": [list(map(float, c)) for c in t]} for t in self.tags]}

    def picture(self, node):
        img = np.full((1080, 1920, 3), (236, 190, 150), np.uint8)
        if self.grey:
            cv2.rectangle(img, *self.grey, (176, 180, 200), -1)
        for a, b in self.white:
            cv2.rectangle(img, a, b, (252, 252, 252), -1)
        for t in self.tags:
            cv2.fillPoly(img, [np.array(t, np.int32)], (10, 10, 10))
        return img

    def ok(self, method, path, body=None):
        if "/where/pixel" in path:
            q = dict(kv.split("=") for kv in path.split("?")[1].split("&"))
            return {"known": True, "at_m": [float(q["u"]) * 0.0002, float(q["v"]) * 0.0002], "uncertainty_m": 0.002}
        return {"joints": {"joints_rad": {"shoulder_pan": 0.0, "gripper": 0.1}}}


@pytest.mark.parametrize("white,tags,answer", [
    ([((640, 320), (699, 680))], (), "no"),                                          # 12 mm of white beside the grey: the drawer's front
    ([((400, 320), (699, 680))], (), "yes"),                                         # 60 mm: its tray
    ([((640, 320), (699, 680)), ((200, 800), (500, 1000))], (), "no"),               # ... and 60 mm of white that is beside nothing grey
    ([((640, 320), (699, 680)), ((1101, 300), (1400, 600))], [[(1150, 350), (1350, 350), (1350, 550), (1150, 550)]], "no"),   # ... on a printed tag
    ([((640, 320), (699, 680)), ((0, 300), (699, 310))], (), "no"),                  # ... or running off the picture
    ([((640, 320), (699, 680)), ((1101, 300), (1400, 600))], (), "yes")])            # (the same patch with no tag on it is a tray)
def test_a_drawer_is_out_when_more_than_its_front_shows_beside_the_cabinet(white, tags, answer):
    """The classes: the arm's own tag block taken for a drawer (white round a printed tag, with a shaded grey side: it
    answered 'yes' to a shut drawer); white somewhere else in view; part of a patch measured as the whole of it."""
    from solutions import tasks
    assert tasks.drawer_open(_Cabinet(white, tags)) == {"open": answer}


def test_with_no_drawer_in_sight_the_question_is_not_answered():
    from solutions import tasks
    from solutions.api import GaveUp
    with pytest.raises(GaveUp, match="nothing white beside anything grey"):          # a cabinet, and no white at it
        tasks.drawer_open(_Cabinet([]))
    with pytest.raises(GaveUp, match="nothing white beside anything grey"):          # white, and nothing grey
        tasks.drawer_open(_Cabinet([((400, 320), (699, 680))], grey=None))
    with pytest.raises(GaveUp, match="could not be held anywhere"):
        tasks.drawer_open(_Cabinet([((400, 320), (699, 680))], reachable=False))


class _Looking:
    """An Api whose wrist camera shows a given picture and whose pictures are 1 mm to the pixel, the arm's origin at (960, 540)."""
    def __init__(self, image):
        self.image, self.calls = image, 0

    def post(self, path, body=None):
        return 200, {"reached": True}

    def picture(self, node):
        return self.image

    def ok(self, method, path, body=None):
        q = dict(kv.split("=") for kv in path.split("?")[1].split("&"))
        return {"known": True, "at_m": [(float(q["u"]) - 960) / 1000, (float(q["v"]) - 540) / 1000], "uncertainty_m": 0.002}


@pytest.mark.parametrize("left,whole", [(1000, True), (1790, True), (1830, False), (1900, False)])
def test_a_bin_is_placed_from_close_up_only_when_its_whole_frame_is_in_the_picture(left, whole):
    """A bin seen whole is placed where its frame's middle is. One that runs off the picture is left where the first look
    put it: its visible part's middle is not its middle (found as a cylinder set down on a bin's wall, 27 mm off)."""
    from solutions import moves
    img = np.full((1080, 1920, 3), (150, 120, 80), np.uint8)
    cv2.rectangle(img, (left, 500), (left + 110, 610), (40, 60, 200), 9)
    true = np.array([(left + 55 - 960) / 1000, (555 - 540) / 1000])
    near = true + [-0.02, 0.015]                                    # where the fixed camera put it
    at, side = moves.closely(_Looking(img), "blue", near, 0.012, ring=True)
    assert side is None
    assert np.linalg.norm(at - (true if whole else near)) < 0.004, (at, true, near)


def _carrying(monkeypatch, patches, real):
    """into_bins with the looking and the carrying stood in for: `patches` are where the fixed camera sees the colour (in
    order), `real` says which of them is a thing (the others raise). Returns (the places carried from, the solver)."""
    from solutions import moves, tasks
    from solutions.api import GaveUp, Missed, NotThere, OutOfReach
    carried, there = [], [np.array(p, float) for p in patches]

    def roughly(api, colour, ring=False, height=0.015, pick=0):
        if ring:
            return np.array([0.0, 0.3])
        if pick >= len(there):
            raise GaveUp("no fixed camera sees one")
        return there[pick]

    def find_thing(api, colour, pick=0, height_guess=0.03, near=None):
        k = next(i for i, p in enumerate(there) if np.allclose(p, near))
        if real[k] in (NotThere, OutOfReach):
            raise real[k]("not a thing" if real[k] is NotThere else "cannot hover")
        if real[k] == "in the bin":                                 # (seen somewhere else by the fixed camera; from close above, in the bin)
            return {"at": np.array([0.01, 0.31]), "top": 0.03, "side_deg": 0.0}
        return {"at": near, "top": 0.03, "side_deg": 0.0}

    def pick(api, thing):
        k = next(i for i, p in enumerate(there) if np.allclose(p, thing["at"]))
        if real[k] is Missed:
            raise Missed("the jaws closed on nothing")
        carried.append(tuple(thing["at"]))
        there.pop(k), real.pop(k)                                   # (it is in the bin now, and is no longer seen out there)
        return 0.011

    for name, f in (("roughly", roughly), ("find_thing", find_thing), ("pick", pick), ("closely", lambda api, c, near, h, ring=False: (near, None)),
                    ("put", lambda *a, **k: None), ("locate_cameras", lambda api: None), ("park", lambda api: None)):
        monkeypatch.setattr(moves, name, f)
    return carried, tasks.into_bins((("yellow", "blue"),))


def test_a_patch_that_is_not_a_thing_is_passed_over_and_the_next_one_tried(monkeypatch):
    """The class: the first patch of the colour is the arm itself (it is printed yellow), and the task ends there; or it is
    a thing already carried to the bin, which the task then goes back for. Every order of 'not a thing', 'out of reach',
    'nothing between the jaws', 'in the bin already' and 'a thing' must end with exactly the things carried, each once."""
    import itertools
    from solutions.api import Missed, NotThere, OutOfReach
    places = [(0.2, 0.0), (0.25, -0.1), (0.3, 0.1), (0.15, -0.15)]
    for kinds in itertools.product((True, NotThere, OutOfReach, Missed, "in the bin"), repeat=3):
        carried, solve = _carrying(monkeypatch, places[:3], list(kinds))
        if True not in kinds:
            continue
        solve(None)
        assert sorted(carried) == sorted(p for p, k in zip(places, kinds) if k is True), kinds


def test_patches_that_are_none_of_them_things_end_in_giving_up_with_what_was_looked_at(monkeypatch):
    from solutions.api import GaveUp, NotThere, OutOfReach
    carried, solve = _carrying(monkeypatch, [(0.2, 0.0), (0.25, -0.1)], [NotThere, OutOfReach])
    with pytest.raises(GaveUp, match="2 patches of that colour were looked at: not a thing"):
        solve(None)
    assert carried == []


# ---------------------------------------------------------------- the solutions
def test_a_solution_touches_nothing_but_the_hubs_api():
    """The class: a solution that 'finds' a thing by asking the simulation. Solutions may import only themselves, numpy,
    OpenCV and the standard library; they are handed an Api and nothing else."""
    allowed = {"numpy", "cv2", "math", "json", "dataclasses", "__future__", "typing"}
    for path in sorted((ROOT / "solutions").glob("*.py")):
        for node in ast.walk(ast.parse(path.read_text())):
            names = ([a.name for a in node.names] if isinstance(node, ast.Import)
                     else [node.module or ""] if isinstance(node, ast.ImportFrom) and node.level == 0 else [])
            for name in names:
                assert name.split(".")[0] in allowed, f"{path.name} imports {name}"
        text = path.read_text()
        for word in ("so101sim", "mujoco", "World(", "operator_api", "truth"):
            assert word not in text, f"{path.name} mentions {word}"


def test_every_task_has_a_solution_or_a_reason_why_not():
    from solutions import NO_SOLUTION, SOLVERS
    every = {(s, t["name"]) for s in scenes.names() for t in scenes.read(s).get("tasks", [])}
    assert set(SOLVERS) | set(NO_SOLUTION) == every and not set(SOLVERS) & set(NO_SOLUTION)
    assert all(len(why) > 30 for why in NO_SOLUTION.values())


@pytest.mark.parametrize("scene,task", [("open_small", "where_is_everyone"), ("no_arm", "where_is_everyone"), ("moved_arm", "where_is_everyone"),
                                        ("l_shaped", "where_is_everyone"), ("no_arm", "how_big"), ("shelf", "on_the_shelf"),
                                        ("shelf", "under_the_ledge"), ("moved_arm", "carried"), ("workcell", "stack"),
                                        ("moved_arm", "fetch"), ("shelf", "fetch_from_shelf"), ("cluttered", "tidy_by_colour"),
                                        ("cluttered", "fetch_hidden"), ("l_shaped", "far_leg"), ("bench", "drawer_open")])
def test_a_solution_does_its_task_as_the_referee_sees_it(scene, task):
    import solve
    r = solve.run(scene, task)
    assert r["done"], r
    assert r["api_calls"] >= 2 and "gave_up" not in r


@pytest.mark.parametrize("out_m", [0.01, 0.04, 0.085])
def test_the_drawer_is_judged_as_it_is_and_not_as_it_usually_is(out_m):
    """The class: an answer that is right because the scene always begins the same way. The same task with the drawer
    begun ajar, half out and fully out (the cube in it moved with it)."""
    import solve

    def change(data):
        next(f for f in data["fixtures"] if f["name"] == "cabinet")["drawer_open_m"] = out_m
        cube = next(o for o in data["objects"] if o["name"] == "red_cube")
        cube["at_m"] = [cube["at_m"][0] - out_m, cube["at_m"][1], cube["at_m"][2]]

    r = solve.run("bench", "drawer_open", change=change)
    assert r["done"] and r["answers"] == {"open": "yes" if out_m > 0.02 else "no"}, r


@pytest.mark.parametrize("scene,task,words", [("open_small", "how_big", "far edge"), ("l_shaped", "how_big", "not a plain rectangle")])
def test_a_solution_that_cannot_finish_says_so_and_hands_in_no_guess(scene, task, words):
    """This one does not pass yet. What must hold meanwhile: no wrong answer handed in, and the reason given."""
    import solve
    r = solve.run(scene, task)
    if r["done"]:
        return                                                      # (it has been made to work: this test has had its day)
    assert "gave_up" in r and words in r["gave_up"] and not r.get("answers"), r


class _Rover:
    """An Api with a rover whose head turns and whose picture shows `things` (colour, x, y in the rover's frame) as patches:
    a degree is 12 px across the picture, and a thing r metres off has its foot 500 r px above row 1000. The fixed camera
    covers the places `covered` says it does."""
    PAINT = {"red": (200, 30, 30), "blue": (40, 60, 200), "green": (30, 160, 50), "yellow": (220, 190, 30), "purple": (150, 75, 200)}

    def __init__(self, things, covered, arm=(0.5, 0.0), blocked=(), related=True):
        self.things, self.covered, self.arm, self.blocked, self.related = things, covered, arm, blocked, related
        self.pan, self.calls, self.stowed = 0.0, 0, False

    def devices(self):
        return {"rover": {}, "camera_1": {}, **({"arm": {}} if self.arm else {})}

    def post(self, path, body=None):
        if path.endswith("/head/point"):
            if body["pan_deg"] in self.blocked:
                return 409, {"error": "the arm is in the way"}
            self.pan = body["pan_deg"]
        self.stowed = path.endswith("/head/stow")
        return 200, {"reached": True}

    def picture(self, node):
        img = np.full((1080, 1920, 3), (150, 120, 80), np.uint8)
        for colour, x, y in self.things:
            u = 960 - ((math.degrees(math.atan2(y, x)) - self.pan + 180) % 360 - 180) * 12
            foot = 1000 - 500 * math.hypot(x, y)
            cv2.rectangle(img, (int(u) - 15, int(foot) - 24), (int(u) + 15, int(foot)), self.PAINT[colour], -1)
        return img

    def ok(self, method, path, body=None):
        if path.startswith("/where?of=arm"):
            return {"known": True, "position_m": [*self.arm, 0.0]}
        if path.startswith("/where/pixel"):
            q = dict(kv.split("=") for kv in path.split("?")[1].split("&"))
            bearing, reach = math.radians(self.pan - (float(q["u"]) - 960) / 12), (1000 - float(q["v"])) / 500
            return {"known": True, "at_m": [reach * math.cos(bearing), reach * math.sin(bearing)]}
        if path.startswith("/where/point"):
            q = dict(kv.split("=") for kv in path.split("?")[1].split("&"))
            if not self.related:
                return {"known": False, "why": "nothing relates camera_1 to rover"}
            return {"known": True, "in_picture": bool(self.covered(float(q["x"]), float(q["y"])))}
        return {}


FAR_LEG = [("red", 0.3, -0.05), ("yellow", 0.6, -0.5), ("green", 0.45, -0.6), ("purple", 0.55, -0.4), ("blue", -0.4, 0.3),
           ("yellow", 0.5, 0.0), ("yellow", 1.4, -0.04)]            # (the last two: the arm, and the arm's top, which lands beyond it)


@pytest.mark.parametrize("blocked", [(), (40.0,), (-40.0, 160.0), (0.0, 80.0, -160.0)])
def test_what_no_fixed_camera_covers_is_counted_once_and_the_arm_is_not_a_thing(monkeypatch, blocked):
    """The classes: a thing seen in two overlapping views counted twice; the arm's own colour counted as a loose thing;
    a thing the camera covers counted as out of its sight; a way the head cannot turn ending the task."""
    from solutions import moves, tasks
    monkeypatch.setattr(moves, "locate_cameras", lambda api: None)
    api = _Rover(FAR_LEG, covered=lambda x, y: 0 < x < 1.0 and y > -0.2, blocked=blocked)        # (not what lies beyond the arm)
    assert tasks.out_of_sight(api) == {"count": 4, "colours": ["blue", "green", "purple", "yellow"]}
    assert api.stowed, "the head is left as it must be to drive"


def test_out_of_sight_gives_up_when_it_cannot_tell(monkeypatch):
    from solutions import moves, tasks
    from solutions.api import GaveUp
    monkeypatch.setattr(moves, "locate_cameras", lambda api: None)
    with pytest.raises(GaveUp, match="cannot be told"):             # nothing relates the camera to the rover: no guess
        tasks.out_of_sight(_Rover(FAR_LEG, covered=lambda x, y: True, related=False))
    every = tuple(tasks.LOOK_PANS)
    stuck = _Rover(FAR_LEG, covered=lambda x, y: True, blocked=every)
    with pytest.raises(GaveUp, match="could not be turned"):
        tasks.out_of_sight(stuck)
    assert stuck.stowed
    assert tasks.out_of_sight(_Rover(FAR_LEG, covered=lambda x, y: True)) == {"count": 0, "colours": []}


def test_how_big_does_not_take_an_l_for_a_rectangle():
    from solutions import tasks
    from solutions.api import Api, GaveUp

    class Fake(Api):
        def __init__(self):
            super().__init__(None)

        def ok(self, method, path, body=None):
            if path == "/survey":
                return {"map_kept": True}
            if path == "/devices/rover/drive/state":
                return {"since_taken_over": {"heading_deg": 0.0}}
            edge = lambda a, b: {"kind": "edge of the surface", "from_m": a, "to_m": b, "length_m": float(np.linalg.norm(np.subtract(b, a))),       # noqa: E731
                                 "direction_deg": math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 180}
            return {"surface": {"area_seen_m2": 0.5, "edges_found": [edge([0, 0], [0.5, 0]), edge([0, 0.4], [0.5, 0.4]),
                                                                    edge([0, 0], [0, 0.4]), edge([0.5, 0], [0.5, 0.4])]}}

    with pytest.raises(GaveUp, match="not a plain rectangle"):     # 0.5 m2 seen, and the edges found enclose 0.2
        tasks.how_big(Fake())


# ---------------------------------------------------------------- the head keeps out of what is known to be there
def test_the_rovers_head_is_not_turned_into_the_arm_and_may_go_back_the_way_it_came():
    world = World("open_small")
    hub = Hub(world.ports)
    hub._look()
    arm_in_rover, _ = hub.locator.chain("arm", "rover", hub.arm.link.q())
    assert arm_in_rover is not None, "the camera did not relate the arm to the rover"
    assert call(hub, "POST", "/devices/rover/head/raise", {"lens_height_m": 0.16})[1]["reached"]
    start = frame_of(world, "rover")
    bearing = math.degrees(math.atan2(arm_in_rover.T[1, 3], arm_in_rover.T[0, 3] - 0.02))
    refused = None
    for step in range(1, 9):                                        # turn towards the arm, a little at a time
        pan = float(np.clip(bearing * step / 8, -159, 159))
        status, r = call(hub, "POST", "/devices/rover/head/point", {"pan_deg": pan})
        if status == 409:
            refused = r["error"]
            break
        assert status == 200 and r["reached"], r
    reach = float(np.hypot(arm_in_rover.T[0, 3], arm_in_rover.T[1, 3]))
    if reach < 0.45:
        assert refused and "where the arm is" in refused, "the head was turned all the way to an arm within its reach"
    assert np.allclose(frame_of(world, "rover"), start, atol=2e-3), "the rover was not pushed"
    assert call(hub, "POST", "/devices/rover/head/stow")[1]["reached"], "back through where it has been, it may always go"
