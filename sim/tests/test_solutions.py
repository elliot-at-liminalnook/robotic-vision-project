"""Tasks done through the hub's API alone: the point-in-a-picture endpoint, the solutions' eyes, and the solutions themselves.

Aimed at classes of bugs:
  * a place read off a picture in the wrong frame, or along the wrong line of sight (checked against the referee's own
    projection, for fixed cameras and for the camera on the wrist);
  * a solution that reaches past the hub's API for what it should have had to find (the import rule below);
  * a solution that answers when it should have said it cannot tell;
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
                                        ("moved_arm", "carried"), ("workcell", "stack")])
def test_a_solution_does_its_task_as_the_referee_sees_it(scene, task):
    import solve
    r = solve.run(scene, task)
    assert r["done"], r
    assert r["api_calls"] >= 2 and "gave_up" not in r


@pytest.mark.parametrize("scene,task,words", [("open_small", "how_big", "far edge"), ("cluttered", "tidy_by_colour", "")])
def test_a_solution_that_cannot_finish_says_so_and_hands_in_no_guess(scene, task, words):
    """These two do not pass yet. What must hold meanwhile: no wrong answer handed in, and the reason given."""
    import solve
    r = solve.run(scene, task)
    if r["done"]:
        return                                                      # (it has been made to work: this test has had its day)
    assert "gave_up" in r and words in r["gave_up"] and not r.get("answers"), r


def test_how_big_does_not_take_an_l_for_a_rectangle():
    from solutions import tasks
    from solutions.api import Api, GaveUp

    class Fake(Api):
        def __init__(self):
            super().__init__(None)

        def ok(self, method, path, body=None):
            if path == "/survey":
                return {"map_kept": True}
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
