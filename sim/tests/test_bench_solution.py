"""The bench tidied through the hub's API (sim/solutions/bench.py): its parts against made-up pictures, and the whole of it.

Aimed at classes of bugs:
  * a direction with the wrong sign or along the wrong axis (a drawer pulled into its cabinet, jaws across the pull
    instead of across the handle): every way round a cabinet can stand;
  * one thing counted as two because it was seen from two places, or two as one;
  * the arm's own tag blocks, or white that is beside nothing, taken for a drawer's front;
  * jaws that have closed on nothing reported as holding; jaws opened so wide the hand cannot get down over a long thing;
  * a thing set down as if it had been picked up from the surface when it was picked up from something raised.
"""
import math

import cv2
import numpy as np
import pytest

PX_PER_M = 4000.0                 # the made-up camera: straight down, 0.25 mm to the pixel, the same wherever the hand is
ORIGIN = np.array([0.05, -0.05])  # the place at the picture's corner (0, 0)
WOOD, GREY, WHITE, BLACK = (236, 190, 150), (176, 180, 200), (252, 252, 252), (10, 10, 10)


def px(p):
    return (np.asarray(p, float) - ORIGIN) * PX_PER_M


def rot(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def rect(img, centre, size, yaw, colour):
    half = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * np.asarray(size) / 2
    cv2.fillPoly(img, [np.round(px(np.asarray(centre) + half @ rot(yaw).T)).astype(np.int32)], colour)


class Above:
    """An Api whose wrist camera shows a picture drawn by `draw(img)`, and whose touches find `heights(x, y)`."""
    def __init__(self, draw, heights=lambda x, y: 0.0, tags=(), gap_after_closing=0.03, max_gap=0.0809):
        self.draw, self.heights, self.tags, self.calls, self.rest = draw, heights, tags, 0, None
        self.gap, self.gap_after_closing, self.max_gap, self.tip, self.log = max_gap, gap_after_closing, max_gap, np.zeros(3), []

    def devices(self):
        return {"arm": {}}

    def picture(self, node):
        img = np.full((1080, 1920, 3), WOOD, np.uint8)
        self.draw(img)
        return img

    def get(self, path):
        return 200, {"tags": [{"corners_px": [list(map(float, c)) for c in px(t)]} for t in self.tags]}

    def post(self, path, body=None):
        self.log.append((path.rsplit("/devices/arm/", 1)[-1], dict(body or {})))
        if path.endswith("/probe"):
            h = self.heights(body["x"], body["y"])
            return 200, {"touched": h > 0.002, "height_m": h}
        if path.endswith("/gripper/set"):
            self.gap = body["gap_m"] if "gap_m" in body else (self.max_gap if body["opening"] > 0.5 else self.gap_after_closing)
            return 200, {"holding": body.get("opening") == 0.0}
        if path.endswith("/move_tip"):
            self.tip = np.array([body["x"], body["y"], body["z"]])
        return 200, {"reached": True}

    def ok(self, method, path, body=None):
        if method == "POST":
            status, r = self.post(path, body)
            return r
        if "/where/pixel" in path:
            q = dict(kv.split("=") for kv in path.split("?")[1].split("&"))
            at = np.array([float(q["u"]), float(q["v"])]) / PX_PER_M + ORIGIN
            return {"known": True, "at_m": [float(at[0]), float(at[1])], "uncertainty_m": 0.002}
        return {"tip_m": dict(zip("xyz", map(float, self.tip))), "gripper": {"gap_m": self.gap, "max_gap_m": self.max_gap},
                "joints": {"joints_rad": {"shoulder_pan": 0.0, "gripper": 0.1}}}


def cabinet(img, centre, yaw, out_m=0.0):
    """A cabinet seen from above: a grey roof, and at its front the white top of its drawer's front, the black handle in
    the middle of it. The drawer slides out along the cabinet's own -x."""
    rect(img, centre, (0.09, 0.09), yaw, GREY)
    front = np.asarray(centre) + rot(yaw) @ [-0.049 - out_m, 0.0]
    if out_m:
        rect(img, np.asarray(centre) + rot(yaw) @ [-0.045 - out_m / 2, 0.0], (out_m, 0.082), yaw, WHITE)
    rect(img, front, (0.008, 0.082), yaw, WHITE)
    rect(img, front, (0.012, 0.012), yaw, BLACK)
    return front


# ---------------------------------------------------------------- the drawer, every way round
@pytest.mark.parametrize("yaw", [0, 30, 90, 135, 180, 250, 315])
def test_a_drawer_is_pulled_away_from_its_cabinet_with_the_jaws_across_its_handle(yaw):
    from solutions import bench
    centre = np.array([0.28, 0.08])
    front = [None]
    api = Above(lambda img: front.__setitem__(0, cabinet(img, centre, yaw)),
                heights=lambda x, y: 0.071 if np.linalg.norm([x, y] - front[0]) < 0.008 else 0.0)
    api.picture(None)
    drawer = bench.Drawer(api, centre + [0.004, -0.006], front[0] + rot(yaw) @ [0.0, 0.012])        # (both known only roughly)
    out = rot(yaw) @ [-1.0, 0.0]
    assert np.linalg.norm(drawer.handle - front[0]) < 0.003, "the handle is the middle of the front, end to end"
    assert drawer.out @ out > 0.995, "it slides square to its front, away from the cabinet"
    assert abs((drawer.jaws - (yaw + 90.0) + 90.0) % 180.0 - 90.0) < 3.0, "the jaws close along the front: across the handle"
    assert drawer.top == pytest.approx(0.071)
    api.log.clear()
    drawer.open()
    moves = [b for name, b in api.log if name == "move_tip"]
    grip_z = 0.071 - bench.HANDLE_GRIP_BELOW_M
    assert any(abs(m["z"] - grip_z) < 1e-6 and np.linalg.norm([m["x"], m["y"]] - front[0]) < 0.003 for m in moves), "it takes the handle"
    pulled = next(m for m in moves if "timeout_s" in m)
    assert np.linalg.norm([pulled["x"], pulled["y"]] - (front[0] + bench.PULL_M * out)) < 0.004 and abs(pulled["z"] - grip_z) < 1e-6
    assert np.linalg.norm(drawer.handle - (front[0] + bench.PULL_M * out)) < 0.004, "the handle is where the hand left it"
    assert np.linalg.norm(drawer.inside() - (front[0] + (bench.PULL_M - 0.045) * out)) < 0.006, "the tray is behind the handle"
    api.log.clear()
    drawer.shut()
    assert np.linalg.norm(drawer.handle - front[0]) < 0.003, "shut is where it was found"
    moves = [b for name, b in api.log if name == "move_tip"]
    held = next(m for m in moves if "timeout_s" in m)
    assert np.linalg.norm([held["x"], held["y"]] - front[0]) < 0.004, "by its handle, back to where the handle was"
    push = [m for m in moves if "timeout_s" in m][-1]
    low = max(0.071 - bench.PUSH_BELOW_TOP_M, 0.012)
    assert abs(push["z"] - low) < 1e-6 and 0.0 < (np.array([push["x"], push["y"]]) - front[0]) @ out < 0.01, "then pushed home by its front"
    start = moves[moves.index(push) - 1]
    assert (np.array([start["x"], start["y"]]) - front[0]) @ out > 0.025 and abs(start["z"] - low) < 1e-6, "from in front of it, below the handle"
    along = rot(yaw) @ [0.0, 1.0]
    for m in (start, push):                                         # beside the handle (12 mm wide), and still on the front (82 mm wide)
        assert 0.016 < abs((np.array([m["x"], m["y"]]) - front[0]) @ along) < 0.034, "beside the handle, not under it"
    closed = [b for name, b in api.log if name == "gripper/set"]
    assert {"opening": 0.0} in closed, "with the jaws closed"
    names = [name for name, _ in api.log]
    assert names.count("gripper/set") >= 4 and all(b.get("gap_m", 0.03) <= 0.03 for name, b in api.log if name == "gripper/set" and "gap_m" in b)


def test_a_handle_the_jaws_close_on_nothing_at_is_said_so():
    from solutions import bench
    from solutions.api import GaveUp
    centre = np.array([0.28, 0.08])
    api = Above(lambda img: cabinet(img, centre, 0), heights=lambda x, y: 0.07)
    api.post = lambda path, body=None, post=api.post: (200, {"holding": False}) if path.endswith("/gripper/set") else post(path, body)
    drawer = bench.Drawer(api, centre, centre + [-0.049, 0.0])
    with pytest.raises(GaveUp, match="closed on nothing where the drawer's handle"):
        drawer.open()


def test_a_drawer_front_that_is_not_in_view_is_not_guessed_at():
    from solutions import bench
    from solutions.api import GaveUp
    with pytest.raises(GaveUp, match="does not see it"):
        bench.Drawer(Above(lambda img: rect(img, (0.28, 0.08), (0.09, 0.09), 0, GREY)), np.array([0.28, 0.08]), np.array([0.23, 0.08]))


# ---------------------------------------------------------------- the scan, and the furniture in it
def _scan(monkeypatch, draw, tags=()):
    from solutions import bench, moves
    monkeypatch.setattr(moves, "hover_over", lambda api, near, hover=0.14: {"reached": True})
    return bench.Scan(Above(draw, tags=tags), {"grey": 0.05, "white": 0.04, "red": 0.012})


def test_a_thing_seen_from_every_place_is_one_thing_and_two_things_are_two(monkeypatch):
    def draw(img):
        rect(img, (0.15, 0.05), (0.03, 0.03), 0, (200, 30, 30))
        rect(img, (0.30, 0.15), (0.03, 0.03), 0, (200, 30, 30))
    scan = _scan(monkeypatch, draw)
    assert len([s for s in scan.seen if s[0] == "red"]) >= 8, "each is in every picture"
    red = scan.things("red")
    assert len(red) == 2 and sorted(round(float(t[0][0]), 2) for t in red) == [0.15, 0.30]


@pytest.mark.parametrize("yaw", [0, 90, 200])
def test_the_cabinet_is_the_grey_thing_with_a_drawer_and_the_platform_the_other(monkeypatch, yaw):
    from solutions import bench
    front = [None]

    def draw(img):
        front[0] = cabinet(img, (0.30, 0.13), yaw)
        rect(img, (0.13, 0.03), (0.08, 0.08), 0, GREY)                                # the platform
        rect(img, (0.42, 0.02), (0.04, 0.04), 0, WHITE)                               # the arm's tag block: white round a printed tag,
        rect(img, (0.42, 0.045), (0.04, 0.01), 0, GREY)                               # ... with a shaded side

    tag = np.array([[0.405, 0.005], [0.435, 0.005], [0.435, 0.035], [0.405, 0.035]])
    scan = _scan(monkeypatch, draw, tags=[tag])
    where_cabinet, where_front, where_platform = bench.furniture(scan)
    assert np.linalg.norm(where_cabinet - [0.30, 0.13]) < 0.01 and np.linalg.norm(where_platform - [0.13, 0.03]) < 0.01
    assert np.linalg.norm(where_front - front[0]) < 0.02


def test_without_a_drawer_or_without_a_second_grey_thing_the_furniture_is_not_made_up(monkeypatch):
    from solutions import bench
    from solutions.api import GaveUp
    with pytest.raises(GaveUp, match="no cabinet"):                                   # two grey things, no white at either
        bench.furniture(_scan(monkeypatch, lambda img: (rect(img, (0.30, 0.13), (0.09, 0.09), 0, GREY), rect(img, (0.13, 0.03), (0.08, 0.08), 0, GREY))))
    with pytest.raises(GaveUp, match="no platform"):
        bench.furniture(_scan(monkeypatch, lambda img: cabinet(img, (0.30, 0.13), 0)))
    with pytest.raises(GaveUp, match="no cabinet"):                                   # white, but beside nothing grey
        bench.furniture(_scan(monkeypatch, lambda img: (rect(img, (0.30, 0.13), (0.09, 0.09), 0, GREY), rect(img, (0.13, 0.03), (0.01, 0.08), 0, WHITE))))


# ---------------------------------------------------------------- taking hold, and what a thing stood on
@pytest.mark.parametrize("long_m", [None, 0.018, 0.03, 0.04, 0.055, 0.07])
def test_the_jaws_open_wide_for_a_short_thing_and_to_its_length_for_a_long_one(long_m):
    """Opened as wide as they go, the hand does not get down over a thing 40 mm long: part of it rests on the thing's top
    (found between 64 and 70 mm of gap). A short thing gets the wide opening: it forgives a place a few millimetres out
    (narrowed for every thing, the jaws came down on a standing cylinder and knocked it over)."""
    from solutions import moves
    api = Above(lambda img: None)
    moves.pick(api, {"at": np.array([0.2, 0.1]), "top": 0.03, "side_deg": 20.0, "along_m": long_m})
    first = next(b for name, b in api.log if name == "gripper/set")
    if long_m is None or long_m <= 0.036:
        assert first == {"opening": 1.0}
    else:
        assert min(long_m + 0.012, api.max_gap - 0.002) <= first["gap_m"] <= min(long_m + 0.024, api.max_gap) and first["gap_m"] < api.max_gap
        assert long_m > 0.045 or first["gap_m"] <= 0.064
    assert any(b.get("jaws_along_deg") == 20.0 for name, b in api.log if name == "move_tip")


def test_jaws_that_met_each_other_hold_nothing_whatever_the_gripper_says():
    """A jaw pressed down on a thing's top is loaded, and the gripper reports holding. The jaws have then closed to nothing."""
    from solutions import moves
    from solutions.api import GaveUp
    with pytest.raises(GaveUp, match="closed on nothing"):
        moves.pick(Above(lambda img: None, gap_after_closing=0.003), {"at": np.array([0.2, 0.1]), "top": 0.03, "side_deg": 0.0, "along_m": 0.04})


@pytest.mark.parametrize("blocked_ways,carried", [((), True), ((20.0,), True), ((110.0,), True), ((20.0, 110.0), False)])
def test_a_hand_stopped_on_its_way_down_turns_its_jaws_the_other_way_and_tries_once_more(blocked_ways, carried):
    """The class: a thing that can be taken one way round and not the other (something tall stands beside it), tried the
    one way only. The second way opens to the thing's other size. A hand that got down and found nothing does not turn."""
    from solutions import moves
    from solutions.api import Missed
    api = Above(lambda img: None)
    post, state = api.post, {"jaws": None, "down": True}

    def blocked(path, body=None):
        status, r = post(path, body)
        if path.endswith("/move_tip"):
            state["jaws"] = body.get("jaws_along_deg", state["jaws"])
            if body["z"] < 0.02:                                    # the descent
                state["down"] = state["jaws"] not in blocked_ways
                return 200, {"reached": state["down"], "why_not_reached": "The arm stopped itself"}
        if path.endswith("/gripper/set") and body.get("opening") == 0.0:
            api.gap = 0.03 if state["down"] else 0.003
            return 200, {"holding": True}
        return status, r

    api.post = blocked
    thing = {"at": np.array([0.2, 0.1]), "top": 0.03, "side_deg": 20.0, "along_m": 0.025, "across_m": 0.04}
    if not carried:
        with pytest.raises(Missed, match="did not get down"):
            moves.pick(api, thing)
        return
    assert moves.pick(api, thing) == pytest.approx(0.011)
    if 20.0 in blocked_ways:
        gaps = [b for name, b in api.log if name == "gripper/set" and b != {"opening": 0.0}]
        assert gaps[0] == {"opening": 1.0} and 0.05 < gaps[-1]["gap_m"] <= 0.064, "the other way round it is 40 mm long: the jaws open to that"
    # a hand that got down and closed on nothing: the place is wrong, and turning the jaws would only knock what is there
    empty = Above(lambda img: None, gap_after_closing=0.003)
    with pytest.raises(Missed):
        moves.pick(empty, thing)
    assert len({b["jaws_along_deg"] for name, b in empty.log if name == "move_tip" and "jaws_along_deg" in b}) == 1


@pytest.mark.parametrize("roof", [0.0, 0.04, 0.06])
def test_what_a_thing_stands_on_is_touched_beside_it_not_on_it(roof):
    """The class: the height of what a thing stands on read off the thing itself (the touch came down on the cube)."""
    from solutions import bench
    at = np.array([0.25, 0.10])
    cube_top = roof + 0.025
    heights = lambda x, y: cube_top if np.linalg.norm([x, y] - at) < 0.02 else (roof if np.linalg.norm([x, y] - at) < 0.06 else 0.0)        # noqa: E731
    assert bench.stands_on(Above(lambda img: None, heights=heights), {"at": at, "top": cube_top}) == pytest.approx(roof)
    # ... and where the side towards the arm has something as high as the thing, the other side is tried
    wall = lambda x, y: cube_top if (np.linalg.norm([x, y] - at) < 0.02 or np.linalg.norm([x, y]) < np.linalg.norm(at) - 0.02) else roof    # noqa: E731
    assert bench.stands_on(Above(lambda img: None, heights=wall), {"at": at, "top": cube_top}) == pytest.approx(roof)


@pytest.mark.parametrize("yaw", [0, 25, 70, 90, 140])
def test_a_low_long_block_is_taken_by_its_ends_whichever_way_it_lies(yaw):
    """The class: the jaws turned along whichever side of the patch happens to come first. Asked for lengthwise, the side
    given is the 40 mm one, and its length; not asked, it is one of the two."""
    from solutions import moves
    at = np.array([0.25, 0.08])
    api = Above(lambda img: rect(img, at, (0.04, 0.025), yaw, (30, 160, 50)))
    place, side, long, across = moves.sized(api, "green", at + [0.004, -0.003], 0.02, lengthwise=True)
    assert np.linalg.norm(place - at) < 0.002 and abs((side - yaw + 90.0) % 180.0 - 90.0) < 3.0 and long == pytest.approx(0.04, abs=0.003)
    assert across == pytest.approx(0.025, abs=0.003)
    place, side, long, across = moves.sized(api, "green", at, 0.02)
    assert sorted([round(long, 3), round(across, 3)]) == pytest.approx([0.025, 0.04], abs=0.003)
    assert abs((side - yaw + 90.0) % 180.0 - 90.0) < 3.0 if long > 0.033 else abs((side - yaw) % 180.0 - 90.0) < 3.0


# ---------------------------------------------------------------- the whole of it
def test_the_bench_is_tidied_through_the_hubs_api_alone():
    import solve
    r = solve.run("bench", "tidy")
    assert r["done"] and "gave_up" not in r, r
