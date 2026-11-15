"""Safety from what has been found: the stop path for every device, the rover's wheels and the arm's hand against the map.

Aimed at classes of bugs:
  * a motion made into something the map already has (the guard reading the map in the wrong frame, or not at all);
  * a motion into never-seen space made at full speed, so that what is there is shoved, not met;
  * wheels that go on counting while the rover pushes against something (the map and every link then drift);
  * the arm's hand taken into the rover, or kept out of where the rover is not (its place read the wrong way round);
  * a device left powered or moving on the way out.

The fuzzed runs send many random commands and check only what must always hold, against the referee.
"""
import json
import math

import cv2
import mujoco
import numpy as np
import pytest

from hub import api as A
from hub import geometry as G
from hub.guard import Guard
from hub.hub import Hub
from hub.place import FREE, OCCUPIED, UNSEEN
from hub.rover.device import V_MAX, V_SLOW
from so101sim import scenes
from so101sim.world import World


def call(hub, method, path, body=None):
    status, ctype, out = A.dispatch(hub, method, path, b"" if body is None else json.dumps(body).encode())
    return status, (json.loads(out) if ctype == "application/json" else out)


def frame_of(world, body: str) -> np.ndarray:
    m, d = world.model, world.data
    i = m.body(body).id
    return G.pose(d.xmat[i].reshape(3, 3), d.xpos[i])


def loose(world) -> dict[str, np.ndarray]:
    m, d = world.model, world.data
    return {m.body(i).name: d.xpos[i].copy() for i in range(m.nbody)
            if m.body(i).parentid[0] == 0 and m.body_dofnum[i] == 6 and m.body(i).name != "rover"}


def most_moved(world, before: dict) -> float:
    return max((float(np.linalg.norm(p - before[k])) for k, p in loose(world).items()), default=0.0)


def rover_outline(world) -> np.ndarray:
    """The referee's: every corner of the rover's collision shapes, seen from above, in the scene's frame."""
    m, d = world.model, world.data
    root = m.body("rover").id
    pts = []
    for g in range(m.ngeom):
        b = m.geom_bodyid[g]
        while b and b != root and m.body_parentid[b]:
            b = m.body_parentid[b]
        if b == root and m.geom_contype[g]:
            half = np.maximum(m.geom_size[g], 1e-4) if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH else np.full(3, 0.01)
            for sx in (-1, 1):
                for sy in (-1, 1):
                    pts.append(d.geom_xpos[g] + d.geom_xmat[g].reshape(3, 3) @ (half * [sx, sy, 0.0]))
    return np.array(pts)[:, :2]


def touching(world, a: str, b: str) -> bool:
    """The referee's: is any shape of the body tree a in contact with any of b's?"""
    m, d = world.model, world.data

    def root(body):
        while m.body_parentid[body]:
            body = m.body_parentid[body]
        return m.body(body).name

    for c in d.contact[:d.ncon]:
        r = {root(m.geom_bodyid[c.geom1]), root(m.geom_bodyid[c.geom2])}
        if r == {a, b} and c.dist < 0.0:
            return True
    return False


# ---------------------------------------------------------------- the stop path, for every device
def test_stop_at_the_root_freezes_every_device_and_the_way_out_leaves_no_servo_powered():
    world = World("bench")
    hub = Hub(world.ports)
    assert call(hub, "POST", "/devices/rover/head/raise", {"lens_height_m": 0.12})[1]["reached"]
    assert call(hub, "POST", "/devices/arm/move_joints", {"shoulder_pan": 0.3})[0] == 200
    status, r = call(hub, "POST", "/stop")
    assert status == 200 and {"arm", "rover"} <= {s.split("/")[-1] for s in r["stopped"]}, r
    q, pose, head = hub.arm.link.q().copy(), frame_of(world, "rover"), dict(hub.rover.head_now())
    call(hub, "POST", "/wait", {"seconds": 1.0})
    assert np.abs(hub.arm.link.q() - q).max() < 0.01 and np.allclose(frame_of(world, "rover"), pose, atol=1e-3)
    assert max(abs(hub.rover.head_now()[j] - head[j]) for j in head) < 0.02, "the head is held where it was, not dropped"
    assert "torque off" in hub.shutdown()
    servos = [s for bus in world.buses for s in bus.servos]
    assert len(servos) == 12 and all(s.get("torque_switch") == 0 for s in servos), "every servo of every device, off"


def test_stop_under_one_device_leaves_the_other_alone():
    hub = Hub(World("bench").ports)
    status, r = call(hub, "POST", "/devices/rover/stop")
    assert status == 200 and all("arm" not in s for s in r["stopped"])
    assert call(hub, "POST", "/devices/arm/move_joints", {"shoulder_pan": 0.2})[1]["reached"]


# ---------------------------------------------------------------- the rover's wheels and the map
@pytest.fixture(scope="module")
def surveyed():
    """A surface with a rover and a few things, surveyed. A cube stands where the rover's head would sweep."""
    world = World("no_arm")
    hub = Hub(world.ports)
    assert call(hub, "POST", "/survey")[1]["map_kept"]
    return world, hub


def test_with_nothing_looked_at_the_rover_only_crawls():
    world = World("no_arm")
    hub = Hub(world.ports)
    before, start = loose(world), frame_of(world, "rover")
    status, r = call(hub, "POST", "/devices/rover/drive/forward", {"distance_m": -0.04})
    assert status == 200 and r["reached"] and "nothing has been looked at" in r["limited"]
    assert r["elapsed_s"] > 0.04 / V_MAX * 2, "at a crawl, not at its usual speed"
    assert abs((G.inv(start) @ frame_of(world, "rover"))[0, 3] + 0.04) < 0.006 and most_moved(world, before) < 0.002


def test_the_rover_does_not_drive_or_turn_into_what_the_map_has(surveyed):
    world, hub = surveyed
    before, start = loose(world), frame_of(world, "rover")
    cube = (G.inv(start) @ np.append(before["red_cube"], 1.0))[:2]
    for path, body in (("forward", {"distance_m": 0.10}), ("forward", {"distance_m": 0.5}), ("turn", {"angle_deg": 90})):
        status, r = call(hub, "POST", f"/devices/rover/drive/{path}", body)
        assert status == 409 and "the map has something" in r["error"], (path, r)
        at = [float(v) for v in r["error"].split("at (")[1].split(")")[0].split(",")]
        assert np.linalg.norm(np.array(at) - cube) < 0.05, "what it names is the cube, where the cube is"
    assert np.allclose(frame_of(world, "rover"), start, atol=2e-3) and most_moved(world, before) < 0.001
    assert np.abs(hub.rover.pose).max() < 2e-3, "a refused drive counts nothing"


def test_backing_into_what_was_never_seen_is_slow_and_what_stops_it_goes_into_the_map(surveyed):
    """Behind the rover nothing has looked. 0.10 m back is the surface's raised edge; the tyres slip against it."""
    world, hub = surveyed
    start = frame_of(world, "rover")
    status, r = call(hub, "POST", "/devices/rover/drive/forward", {"distance_m": -0.30})
    assert status == 200 and not r["reached"] and "never been seen" in r["limited"] and "harder than rolling" in r["why_not_reached"]
    went = (G.inv(start) @ frame_of(world, "rover"))[0, 3]
    assert -0.13 < went < -0.07, "it was stopped by the edge"
    assert abs(r["moved"]["forward_m"] - went) < 0.015, "what the wheels counted while slipping was taken back"
    assert abs(hub.rover.pose[0] - went) < 0.015
    status, again = call(hub, "POST", "/devices/rover/drive/forward", {"distance_m": -0.10})
    assert status == 409 and "the map has something" in again["error"], "what stopped it is in the map now"
    assert call(hub, "POST", "/devices/rover/drive/forward", {"distance_m": 0.05})[1]["reached"], "and it can leave"


def test_where_the_rover_stands_is_free_in_the_map(surveyed):
    _, hub = surveyed
    T = hub.map_from_rover()
    for p in ([0.0, 0.0], [0.08, 0.03], [-0.03, -0.04]):
        i, j = np.floor((T @ [p[0], p[1], 0.0, 1.0])[:2] / 0.01).astype(int) + 150
        assert hub.place.cells()[i, j] == FREE, p


@pytest.mark.parametrize("seed", range(2))
def test_fuzzed_drives_move_nothing_leave_the_surface_never_and_are_counted(seed):
    """Thirty random drives and turns after a survey. Whatever the guard allows, shortens, slows or refuses: no loose
    thing is moved, the rover stays on the surface, and where its wheels say it is stays near where it is."""
    rng = np.random.default_rng(seed)
    s = scenes.read("no_arm")
    s.pop("tasks")
    world = World(s)
    hub = Hub(world.ports)
    assert call(hub, "POST", "/survey")[1]["map_kept"]
    before, start = loose(world), frame_of(world, "rover")
    poly = np.array(world.scene["surface"]["outline_m"], np.float32).reshape(-1, 1, 2)
    made = refused = 0
    for _ in range(30):
        if rng.random() < 0.5:
            status, r = call(hub, "POST", "/devices/rover/drive/forward", {"distance_m": float(rng.choice([-1, 1]) * rng.uniform(0.02, 0.3))})
        else:
            status, r = call(hub, "POST", "/devices/rover/drive/turn", {"angle_deg": float(rng.choice([-1, 1]) * rng.uniform(10, 120))})
        assert status in (200, 409), r
        made, refused = made + (status == 200), refused + (status == 409)
        if status == 409:
            assert "the map has something" in r["error"]
        elif not r["reached"]:
            assert r["why_not_reached"]
        assert most_moved(world, before) < 0.006, "a loose thing was pushed"
        assert min(cv2.pointPolygonTest(poly, (float(x), float(y)), True) for x, y in rover_outline(world)) > -0.012, "off the surface"
        true = G.inv(start) @ frame_of(world, "rover")
        said = hub.rover.pose
        assert np.linalg.norm(true[:2, 3] - said[:2]) < 0.05 and abs(math.degrees((G.yaw_of(true) - said[2] + math.pi) % (2 * math.pi) - math.pi)) < 12, \
            (true[:2, 3], G.yaw_of(true), said)
    assert made >= 8 and refused >= 1, (made, refused)


# ---------------------------------------------------------------- the arm's hand and the map
@pytest.fixture(scope="module")
def bench():
    world = World("bench")
    hub = Hub(world.ports, survey=True)
    assert not hub.place.empty and call(hub, "GET", "/where?of=rover&relative_to=arm")[1]["known"]
    return world, hub


@pytest.fixture(scope="module")
def small():
    """The small open surface: room in the arm's reach with nothing standing in it."""
    world = World("open_small")
    hub = Hub(world.ports)
    assert call(hub, "POST", "/devices/rover/head/raise", {"lens_height_m": 0.16})[1]["reached"]
    assert call(hub, "POST", "/devices/rover/head/point", {"tilt_deg": -38.0})[1]["reached"]
    assert call(hub, "POST", "/devices/rover/head/look")[1]["view"]["used"]          # one view is a map; the camera relates the rest
    assert call(hub, "POST", "/devices/rover/head/stow")[1]["reached"]
    assert hub._map_in("arm")[0] is not None, "nothing relates the arm to the map"
    return world, hub


def test_the_hand_does_not_pass_through_what_the_map_has_and_may_go_over_it(small):
    """The class: the map read in the wrong frame for the arm. A block is put into the map (not the world) at a clear spot
    near the arm, and paths of the hand are put to the same guard its moves are (the arm's reach has no say here)."""
    from hub.errors import Refused
    world, hub = small
    T, _ = hub._map_in("arm")
    guard = hub.arm.actions.guard
    assert guard is not None and guard.__self__ is hub.guard
    block, h = np.array([0.225, 0.19]), 0.06                        # in the arm's frame: 60 mm high, 30 mm square
    keep = (hub.place.high.copy(), hub.place.height.copy(), hub.place.ground.copy())
    line = lambda a, b: np.linspace(a, b, 30)                                               # noqa: E731
    try:
        for dx in np.arange(-0.03, 0.031, 0.005):                   # seen free all round it, so that only the block matters
            for dy in np.arange(-0.2, 0.201, 0.005):
                a, b = np.floor((G.inv(T) @ [block[0] + dy, block[1] + dx, 0.0, 1.0])[:2] / 0.01).astype(int) + 150
                hub.place.ground[a, b], hub.place.high[a, b], hub.place.height[a, b] = 1, 0, 0.0
        for dx in np.arange(-0.015, 0.016, 0.005):
            for dy in np.arange(-0.015, 0.016, 0.005):
                a, b = np.floor((G.inv(T) @ [block[0] + dx, block[1] + dy, 0.0, 1.0])[:2] / 0.01).astype(int) + 150
                hub.place.high[a, b], hub.place.height[a, b] = hub.place.ground[a, b] + 2, h
        with pytest.raises(Refused) as e:                           # along x, low, through it
            guard(line([0.10, 0.19, 0.04], [0.35, 0.19, 0.04]))
        assert e.value.status == 409 and "passes through something the map has" in str(e.value) and "0.060 m high" in str(e.value)
        at = [float(v) for v in str(e.value).split("at (")[1].split(")")[0].split(",")]
        assert np.linalg.norm(np.array(at) - block) < 0.06, "what it names is the block, where the block is in the arm's frame"
        assert guard(line([0.10, 0.19, 0.09], [0.35, 0.19, 0.09])) == (False, None), "over it, through what was seen free: as usual"
        cautious, why = guard(line([0.10, 0.19, 0.04], block.tolist() + [0.04]))
        assert cautious and "ends at something the map has" in why, "ending at it is an approach, made carefully"
        assert guard(line(block.tolist() + [0.04], [0.10, 0.19, 0.04]))[0], "and it may leave again"
        with pytest.raises(Refused):                                # the same line turned a right angle misses the free strip: unseen
            guard(line([0.225, 0.02, 0.04], [0.225, 0.36, 0.04]))
        try:                                                        # nothing the map has is at the block's mirror image
            guard(line([0.10, -0.19, 0.04], [0.35, -0.19, 0.04]))
        except Refused as other:
            assert "0.060 m high" not in str(other), other          # (the rover stands that side: it may be refused for that)
    finally:
        hub.place.high, hub.place.height, hub.place.ground = keep


def test_the_hand_may_end_at_a_thing_and_goes_there_carefully(bench):
    world, hub = bench
    stack = (G.inv(frame_of(world, "base")) @ np.append(loose(world)["purple_cube"], 1.0))[:3]
    before = loose(world)
    assert call(hub, "POST", "/devices/arm/move_tip", {"x": float(stack[0]), "y": float(stack[1]), "z": 0.14})[0] == 200
    status, r = call(hub, "POST", "/devices/arm/move_tip", {"x": float(stack[0]), "y": float(stack[1]), "z": 0.10})
    assert status == 200 and "careful" in r and most_moved(world, before) < 0.004
    call(hub, "POST", "/devices/arm/home")


def test_the_hand_is_kept_out_of_where_the_rover_is(bench):
    world, hub = bench
    base, rover = frame_of(world, "base"), frame_of(world, "rover")
    shape = hub.rover.rover.shape(hub.rover.head_now())
    pts = (np.c_[shape, np.ones(len(shape))] @ (G.inv(base) @ rover).T)[:, :2]
    hull = cv2.convexHull(pts.astype(np.float32)).reshape(-1, 2)
    near = min(hull, key=lambda p: np.linalg.norm(p))               # the rover's corner nearest the arm
    aim = near * (1.0 - 0.07 / np.linalg.norm(near))                # 70 mm short of it: inside what the hand keeps clear
    call(hub, "POST", "/devices/arm/home")
    q, pose = hub.arm.link.q().copy(), rover.copy()
    said = []
    for z in (0.03, 0.06, 0.09):
        status, r = call(hub, "POST", "/devices/arm/move_tip", {"x": float(aim[0]), "y": float(aim[1]), "z": z})
        assert status in (400, 409), r                              # (400: beyond the arm's reach at that height)
        said.append(r["error"])
    assert any("where the rover is" in e for e in said), said
    status, r = call(hub, "POST", "/devices/arm/probe", {"x": float(aim[0]), "y": float(aim[1])})
    assert status == 409 and "where the rover is" in r["error"], "a thing may be touched; the rover may not"
    assert np.abs(hub.arm.link.q() - q).max() < 0.01 and np.allclose(frame_of(world, "rover"), pose, atol=1e-3)
    assert not touching(world, "base", "rover")


@pytest.mark.parametrize("seed", range(2))
def test_fuzzed_moves_of_the_hand_never_touch_the_rover_and_shove_nothing(bench, seed):
    """Forty random points for the tip. Refused, careful or free: the rover is never touched and no loose thing is pushed
    more than a first touch pushes it."""
    world, hub = bench
    rng = np.random.default_rng(seed)
    call(hub, "POST", "/devices/arm/home")
    before, pose = loose(world), frame_of(world, "rover").copy()
    counts = {200: 0, 400: 0, 409: 0}
    for _ in range(40):
        r_, a = rng.uniform(0.12, 0.38), rng.uniform(-1.4, 1.4)
        status, r = call(hub, "POST", "/devices/arm/move_tip", {"x": float(r_ * math.cos(a)), "y": float(r_ * math.sin(a)), "z": float(rng.uniform(0.01, 0.2)),
                                                                "timeout_s": 5.0})
        assert status in counts, r
        counts[status] += 1
        if status != 200:
            assert r["error"]
        elif not r["reached"]:
            assert r["why_not_reached"]
        assert not touching(world, "base", "rover"), "the arm is touching the rover"
        assert np.allclose(frame_of(world, "rover"), pose, atol=2e-3), "the rover was pushed"
        assert most_moved(world, before) < 0.02, "a loose thing was shoved"
    assert counts[200] >= 5 and counts[409] >= 1, counts


def test_with_no_map_the_hand_goes_slowly_and_stops_at_the_first_touch_of_what_does_not_give():
    """The class: full speed into what nothing has looked at. A block fixed to the surface stands in the hand's way. With
    the guard the move is slow and ends at the first touch; without it the arm pushes until its stall guard gives up."""
    def run(guarded: bool):
        s = scenes.read("workcell")
        s["objects"] = []
        s.pop("tasks", None)
        s["fixtures"] = [{"name": "post", "kind": "box", "at_m": [0.25, 0.0], "size_m": [0.04, 0.04, 0.06]}]
        arm = next(d for d in s["devices"] if d["kind"] == "arm")
        s["fixtures"][0]["at_m"] = [arm["at_m"][0] + 0.25, arm["at_m"][1]]
        world = World(s)
        hub = Hub(world.ports)
        if not guarded:
            hub.arm.actions.guard = None
        call(hub, "POST", "/devices/arm/gripper/set", {"opening": 0.0})
        assert call(hub, "POST", "/devices/arm/move_tip", {"x": 0.25, "y": -0.11, "z": 0.10})[0] == 200
        assert call(hub, "POST", "/devices/arm/move_tip", {"x": 0.25, "y": -0.11, "z": 0.03})[0] == 200
        loads = []
        hub.arm.actions.on_frame = lambda: loads.append(float(np.max(hub.arm.ctrl.load_frac())))
        status, r = call(hub, "POST", "/devices/arm/move_tip", {"x": 0.25, "y": 0.11, "z": 0.03})
        hub.arm.actions.on_frame = None
        return status, r, max(loads)

    status, r, load = run(guarded=True)
    assert status == 200 and not r["reached"] and "first touch" in r["why_not_reached"], r
    assert "not been mapped" in r["careful"] or "never been seen" in r["careful"]
    status, free, shove = run(guarded=False)
    assert status == 200 and not free["reached"] and "first touch" not in free["why_not_reached"] and "careful" not in free
    assert r["elapsed_s"] < free["elapsed_s"] or load < shove, "the guarded move gave up sooner, or pushed less hard"
    assert load < 0.8 * shove or r["state"]["tip_m"]["y"] <= free["state"]["tip_m"]["y"] + 0.002, (load, shove)


def test_an_unplaced_rover_makes_low_moves_careful_not_impossible():
    world = World("bench")
    hub = Hub(world.ports)
    hub._look = lambda: {}                                         # nothing relates the two, and nothing can
    assert call(hub, "GET", "/where?of=rover&relative_to=arm")[1]["known"] is False
    status, low = call(hub, "POST", "/devices/arm/move_tip", {"x": 0.2, "y": -0.1, "z": 0.05})
    assert status == 200 and "nothing relates the rover to the arm" in low["careful"]
    assert call(hub, "POST", "/devices/arm/move_tip", {"x": 0.2, "y": -0.1, "z": 0.2})[0] == 200
    status, high = call(hub, "POST", "/devices/arm/move_tip", {"x": 0.2, "y": 0.1, "z": 0.2})          # from above it, to above it
    assert status == 200 and "rover" not in high.get("careful", ""), "above the rover's height its place does not matter"


# ---------------------------------------------------------------- the guard's reading of the map, on made-up maps
class Stub:
    """Just enough of a hub for the guard: a place, and a rover that stands where the map began."""
    def __init__(self, place, rover):
        self.place, self.rover, self.arm = place, rover, None

    def map_from_rover(self):
        return np.eye(4)


@pytest.mark.parametrize("seed", range(6))
def test_a_drive_is_cut_short_of_what_the_map_has_whichever_way_it_goes(seed):
    """The class: the outline laid on the map mirrored or turned the wrong way. Made-up maps, random things."""
    from hub.place import Place
    from hub.rover.description import STOWED, Rover
    rng = np.random.default_rng(seed)
    place = Place()
    place.ground[:] = 1                                             # everything seen free ...
    place.used = 1
    way = float(rng.choice([-1, 1]))
    ahead, side = float(rng.uniform(0.15, 0.4)), float(rng.uniform(-0.05, 0.05))
    shape = Rover().shape(dict(STOWED))
    lead = shape[:, 0].max() if way > 0 else -shape[:, 0].min()
    i, j = int(round((way * (lead + ahead)) / 0.01)) + 150, int(round(side / 0.01)) + 150
    place.high[i, j], place.height[i, j] = 2, 0.03                  # ... but one cell, this far beyond the rover's leading end
    rover = type("R", (), {"rover": Rover(), "head_now": lambda self: dict(STOWED)})()
    guard = Guard(Stub(place, rover))
    d, a, slow, note = guard.drive(way * (ahead + 0.2), 0.0)
    assert not slow and "stopped short" in note and abs(abs(d) - (ahead - 0.02 - 0.015)) < 0.025, (d, ahead)
    assert guard.drive(way * (ahead - 0.08), 0.0)[:3] == (way * (ahead - 0.08), 0.0, False), "short of it, nothing is changed"
    assert guard.drive(-way * 0.3, 0.0)[0] == -way * 0.3, "and the other way is clear"
    place.ground[:] = 0                                             # the same with nothing seen: slow
    assert guard.drive(-way * 0.1, 0.0)[2] is True
