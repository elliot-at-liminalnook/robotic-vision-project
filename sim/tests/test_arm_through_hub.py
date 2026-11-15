"""The arm's own actions, through the hub's API: what the old single-arm API's tests (test_api.py) held the arm to,
kept when that API was retired. The actions are the same code (hub/arm/actions.py); only the way in has changed.

Aimed at classes of bugs:
  * looking costing time, or the same requests giving a different run (an action that reads a wall clock);
  * a refused request that moved something, or that does not say why in numbers that are true;
  * an accepted move that stops part-way for want of a pose it should have been refused for;
  * the gripper's command disturbing the arm's;
  * an action held back by a protection that does not say which, where and by how much.
"""
import json
import random
import re

import numpy as np
import pytest

from hub import api as A
from hub.hub import Hub
from so101sim.world import World

ARM = "/devices/arm"


def call(hub, method, path, body=None):
    status, ctype, out = A.dispatch(hub, method, path, b"" if body is None else json.dumps(body).encode())
    return status, (json.loads(out) if ctype == "application/json" else out)


def fresh():
    world = World("workcell")
    return world, Hub(world.ports)


@pytest.fixture(scope="module")
def rig():
    return fresh()


def joints(hub) -> dict:
    return call(hub, "GET", f"{ARM}/state")[1]["joints"]["joints_rad"]


def test_reading_costs_no_time_and_actions_do(rig):
    world, hub = rig
    t0 = world.time
    for url in ("/", "/tree", ARM, f"{ARM}/state", "/devices/camera_1", "/place"):
        assert call(hub, "GET", url)[0] == 200, url
    assert world.time == t0, "a look at what is known moved the clock"
    # (GET /where may take a picture to find out, and a picture takes time: it is not among them)
    status, r = call(hub, "POST", "/wait", {"seconds": 0.5})
    assert status == 200 and world.time == pytest.approx(t0 + 0.5, abs=0.04)


def test_the_same_requests_give_the_same_run():
    def run():
        world, hub = fresh()
        out = []
        for path, body in ((f"{ARM}/move_tip", {"x": 0.25, "y": -0.08, "z": 0.10}), (f"{ARM}/gripper/set", {"opening": 0.3}),
                           (f"{ARM}/move_joints", {"shoulder_pan": 0.4, "wrist_roll": -1.0}), ("/wait", {"seconds": 0.3})):
            out.append(call(hub, "POST", path, body)[1])
        out.append(call(hub, "GET", f"{ARM}/state")[1])
        hub.shutdown()
        return json.dumps(out, sort_keys=True), world.time
    assert run() == run()


@pytest.mark.parametrize("path,body,words", [
    ("/move_tip", {"x": 0.9, "y": 0, "z": 0.1}, "x"),                         # beyond the stated range
    ("/move_tip", {"x": 0.45, "y": 0.3, "z": 0.3}, "cannot be reached"),      # inside the range, outside the arm's reach
    ("/move_tip", {"x": 0.2, "y": 0}, "'z' is required"),
    ("/move_tip", {"x": "far", "y": 0, "z": 0.1}, "number"),
    ("/move_tip", {"x": 0.2, "y": 0, "z": 0.1, "speed": 3}, "speed"),
    ("/move_joints", {}, "joint"),
    ("/move_joints", {"elbow_flex": 9.0}, "elbow_flex"),
    ("/gripper/set", {"opening": 1.5}, "opening"),
    ("/gripper/set", {}, "exactly one"),
    ("/gripper/set", {"opening": 0.5, "gap_m": 0.03}, "exactly one"),
])
def test_an_arm_request_that_cannot_be_done_says_why_and_changes_nothing(rig, path, body, words):
    world, hub = rig
    before, t0 = joints(hub), world.time
    status, r = call(hub, "POST", ARM + path, body)
    assert status in (400, 409) and r["ok"] is False and words in r["error"], r
    assert joints(hub) == before and world.time == t0, "a refused request changed something"


@pytest.mark.parametrize("seed", range(4))
def test_the_gripper_moves_only_the_jaws(rig, seed):
    """Whatever pose the arm was put in (move_tip or move_joints, fingers down or not), opening or closing the jaws leaves
    the five arm joints where they were commanded."""
    world, hub = rig
    rng = random.Random(seed)
    call(hub, "POST", f"{ARM}/home")
    if seed % 2:
        pose = {"shoulder_pan": rng.uniform(-0.5, 0.5), "wrist_flex": rng.uniform(-0.3, 0.6), "wrist_roll": rng.uniform(-1.5, 0.3)}
        assert call(hub, "POST", f"{ARM}/move_joints", pose)[1]["reached"]
    else:
        call(hub, "POST", f"{ARM}/move_tip", {"x": rng.uniform(0.2, 0.28), "y": rng.uniform(-0.1, 0.1), "z": rng.uniform(0.03, 0.16)})
    before = np.array(list(joints(hub).values())[:5])
    goal = np.array(hub.arm.ctrl.goal[:5], float)
    for body in ({"opening": 0.2}, {"gap_m": 0.05}, {"opening": 1.0}):
        status, r = call(hub, "POST", f"{ARM}/gripper/set", body)
        assert r["ok"] and np.allclose(hub.arm.ctrl.goal[:5], goal), "the gripper changed the arm's target"
        after = np.array(list(joints(hub).values())[:5])
        assert np.abs(after - before).max() < 0.03, f"the arm moved {np.abs(after - before).max():.3f} rad when the jaws did"


@pytest.mark.parametrize("seed", range(3))
def test_a_move_tip_is_refused_whole_or_never_stops_for_want_of_a_pose(seed):
    """Random targets, near and far, reachable and not. Either the request is refused before anything moves, with the
    reason in numbers that are true, or it is accepted and then never stops part-way because the controller could not
    solve a point on its line."""
    rng = random.Random(seed)
    world, hub = fresh()
    refused = accepted = 0
    for k in range(16):
        body = ({"x": rng.uniform(0.02, 0.45), "y": rng.uniform(-0.3, 0.3), "z": rng.uniform(0.004, 0.30)} if k % 2 else
                {"x": rng.uniform(0.15, 0.32), "y": rng.uniform(-0.15, 0.15), "z": rng.uniform(0.02, 0.16)})        # (every other one, nearer the middle of its reach)
        before, t0 = joints(hub), world.time
        status, r = call(hub, "POST", f"{ARM}/move_tip", body)
        if not r["ok"]:
            refused += 1
            assert world.time == t0 and joints(hub) == before, "a refused move moved"
            assert "nothing was moved" in r["error"]
            m = re.search(r"The nearest the tip gets to \(([-\d.]+), ([-\d.]+), ([-\d.]+)\) is \(([-\d.]+), ([-\d.]+), ([-\d.]+)\): (\d+) mm short", r["error"])
            assert m, r["error"]
            v = [float(g) for g in m.groups()]
            assert abs(np.linalg.norm(np.array(v[:3]) - np.array(v[3:6])) * 1000 - v[6]) <= 1.5, "the distance it gives is not the distance between the points it gives"
            assert v[6] >= 4, "refused a point it says it can nearly reach"
            assert "can be about" in r["error"] or "nothing at that height" in r["error"] or "no height" in r["error"]
            if "cannot be reached" in r["error"]:
                # ... and it does not contradict itself: a point said to be out of reach is not inside both the range of
                # distances and the range of heights it then says the arm can reach there.
                reach, height = float(np.hypot(body["x"], body["y"])), max(body["z"], 0.004)
                rs = re.search(r"the tip can be about ([\d.]+) to ([\d.]+) m from the base", r["error"])
                zs = re.search(r"it can be at heights of about ([\d.]+) to ([\d.]+) m", r["error"])
                inside_r = rs is not None and float(rs.group(1)) + 0.011 < reach < float(rs.group(2)) - 0.011
                inside_z = zs is not None and float(zs.group(1)) + 0.021 < height < float(zs.group(2)) - 0.021
                assert not (inside_r and inside_z), f"refused as out of reach a point its own answer puts within reach: {body} {r['error']}"
        else:
            accepted += 1
            assert hub.arm.ctrl.failed_point is None, f"an accepted move stopped at a point it could not solve: {body} {r.get('why_not_reached')}"
            assert isinstance(r["limited"], list)
            if not r["reached"]:
                assert len(r["why_not_reached"]) > 40 and "mm" in r["why_not_reached"]
            else:
                assert "why_not_reached" not in r
    hub.shutdown()
    assert refused >= 2 and accepted >= 3, (refused, accepted)


def test_an_action_held_back_by_the_surface_says_which_part_and_how_far():
    """Joint angles that would put the hand through the surface: the answer names the filter, the part and the depth, and
    the depth is the one the filter's own model gives for the deepest pose the controller meant on the way (which is
    past the pose asked for: its path to a joint angle overshoots, here by about 0.1 rad, before it settles)."""
    import mujoco
    world, hub = fresh()
    call(hub, "POST", f"{ARM}/move_tip", {"x": 0.22, "y": 0.0, "z": 0.06})
    f = hub.arm.link.safety
    asked = np.array(hub.arm.ctrl.goal, float)
    asked[2] += 0.6                                                 # the tip stays above the surface; the wrist camera would not
    meant = []

    def watch():                                                    # (the filter's model is put back as it was after each look)
        keep = f.d.qpos.copy()
        meant.append(float(f.depths(np.array(hub.arm.link._intended, float))[0]))
        f.d.qpos[:] = keep
        mujoco.mj_kinematics(f.m, f.d)
    world.on_frame.append(watch)
    status, r = call(hub, "POST", f"{ARM}/move_joints", {"elbow_flex": float(asked[2]), "timeout_s": 3})
    world.on_frame.remove(watch)
    assert r["ok"] and not r["reached"], r
    assert "command filter held the arm back" in r["why_not_reached"]
    line = next(t for t in r["limited"] if "below the lowest height" in t)
    depth = float(re.search(r"([\d.]+) mm below the lowest height", line).group(1)) / 1000
    at_rest = float(f.depths(asked)[0])
    assert at_rest > 0.005, "the pose asked for is itself below the surface"
    assert at_rest - 0.0015 <= depth <= max(meant) + 0.003, (line, at_rest, max(meant))
    assert abs(depth - max(meant)) < 0.003, "the depth said is the deepest the controller meant to go"
    part = f.m.body(f.low_body).name.replace("_so101_v1", "").replace("_", " ")
    assert part in line
    checks = call(hub, "GET", f"{ARM}/state")[1]["checks"]
    assert checks["command_filter"]["table"]["times"] >= 1 and checks["command_filter"]["table"]["seconds"] > 0.1
    assert any("below the lowest height" in t for t in checks["command_filter_recent"])
    hub.shutdown()
