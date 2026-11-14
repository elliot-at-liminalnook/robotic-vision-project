"""The hub: started from ports alone, in places it was told nothing about.

Aimed at classes of bugs:
  * the hub knowing something it was not told through a port: it imports nothing from the
    simulation, holds no reference to the world, and its answers do not change when the truth
    changes behind its back (the blindfold test) or contain any name from the scene file;
  * a hub that only works in the place it was written in: the same hub discovers the right
    devices and picks an object in several scenes, one with the arm turned 225 degrees;
  * a wrong guess presented as knowledge: /where says "not known" when it cannot tell;
  * a request, however malformed, that is answered with a traceback, or a refusal that changed something;
  * devices that vanish (the scene is swapped): answered as "did not answer", and found again on discover.
"""
import ast
import json
import random
import re
import subprocess
import sys
import threading
import urllib.request
from http.server import HTTPServer

import numpy as np
import pytest

from hub import api as A
from hub.hub import Hub
from so101sim import scenes
from so101sim.config import ROOT, SIM
from so101sim.world import World, WirePort, WorldCamera, WorldClock

ARM_SCENES = [n for n in scenes.names() if any(d["kind"] == "arm" for d in scenes.read(n)["devices"])]


def call(hub, method, path, body=None):
    status, ctype, out = A.dispatch(hub, method, path, b"" if body is None else json.dumps(body).encode())
    return status, (json.loads(out) if ctype == "application/json" else out)


def in_arm_frame(world, body):
    """The referee's: where a body is in the arm's own frame (origin on the surface under its base)."""
    m, d = world.model, world.data
    b = m.body("base").id
    p = d.xmat[b].reshape(3, 3).T @ (d.xpos[m.body(body).id] - d.xpos[b])
    p[2] += d.xpos[b][2]
    return p


def heading_in_arm_frame(world, body) -> float:
    """The referee's: which way a body's own x axis points in the arm's frame [deg, 0 to 180]."""
    m, d = world.model, world.data
    R = d.xmat[m.body("base").id].reshape(3, 3).T @ d.xmat[m.body(body).id].reshape(3, 3)
    return float(np.degrees(np.arctan2(R[1, 0], R[0, 0])) % 180.0)


@pytest.fixture(scope="module")
def started():
    """One world and hub for the tests that only read."""
    w = World("open_small")
    return w, Hub(w.ports)


# ---------------------------------------------------------------- the hub is told nothing
def test_the_hub_imports_nothing_from_the_simulation():
    allowed = {"hub", "numpy", "cv2", "scipy", "mujoco", "tomllib", "math", "dataclasses", "functools", "pathlib", "typing",
               "__future__", "collections", "json", "time", "itertools", "http", "urllib", "argparse", "sys", "threading",
               "io", "base64", "copy", "enum", "abc", "socket", "struct", "queue", "logging", "os", "signal", "re", "zlib"}
    for path in (ROOT / "hub").rglob("*.py"):
        for node in ast.walk(ast.parse(path.read_text())):
            names = []
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom) and node.level == 0:
                names = [node.module or ""]
            for n in names:
                assert n.split(".")[0] in allowed, f"{path.relative_to(ROOT)} imports {n}"


def test_the_hub_holds_nothing_of_the_world_but_its_ports(started):
    """Walk everything reachable from the hub. The world may only be reached through a port object."""
    world, hub = started
    forbidden = {id(world): "the world", id(world.data): "the world's state", id(world.model): "the world's model",
                 id(world.built): "the built scene", id(world.scene): "the scene file"}
    seen, todo = set(), [hub]
    while todo:
        o = todo.pop()
        if id(o) in seen or isinstance(o, (str, bytes, int, float, bool, type(None), np.ndarray, type)):
            continue
        seen.add(id(o))
        assert id(o) not in forbidden, f"the hub can reach {forbidden[id(o)]} without going through a port"
        if isinstance(o, (WirePort, WorldCamera, WorldClock)) or getattr(o, "__self__", None) is world:
            continue                                        # a port (or the world's own ports()): what is behind it is the world's
        if isinstance(o, dict):
            todo += list(o.values())
        elif isinstance(o, (list, tuple, set, frozenset)):
            todo += list(o)
        elif type(o).__module__.split(".")[0] == "hub" or callable(o):
            todo += list(getattr(o, "__dict__", {}).values())
            todo += [c.cell_contents for c in (getattr(o, "__closure__", None) or ()) if _has(c)]
            todo += [getattr(o, "__self__", None), getattr(o, "__func__", None)]


def _has(cell) -> bool:
    try:
        cell.cell_contents
        return True
    except ValueError:
        return False


def every_get(hub) -> str:
    """Everything the API says without being asked to act: the guide, the tree, every node and its state."""
    text = [call(hub, "GET", "/")[1].decode(), json.dumps(call(hub, "GET", "/tree")[1])]
    for n in hub.root.under():
        text.append(json.dumps(call(hub, "GET", "/" + n.path)[1]))
        text.append(json.dumps(call(hub, "GET", f"/{n.path}/state")[1]))
    return "\n".join(text)


@pytest.mark.parametrize("name", scenes.names())
def test_the_api_names_nothing_from_the_scene_file(name):
    """No object, fixture, region, task, camera or scene name, and no description, reaches the model."""
    world = World(name)
    text = every_get(Hub(world.ports)).lower()
    s = world.scene
    words = {s["name"], *[o["name"] for o in s.get("objects", [])], *[f["name"] for f in s.get("fixtures", [])],
             *[r["name"] for r in s.get("regions", [])], *[t["name"] for t in s.get("tasks", [])],
             *[d["name"] for d in s["devices"] if d["kind"] == "camera"], *world.built.regions}
    words -= {"arm", "rover", "camera_1", "camera_2"}       # (the hub's own names for what it finds may coincide)
    for w in sorted(words):
        assert not re.search(rf"\b{re.escape(w.lower())}\b", text), f"'{w}' from the scene file is in the API's answers"
    assert s.get("description", "\0").lower() not in text


def test_blindfold_changing_what_nobody_senses_changes_no_answer():
    """Two worlds that differ only in what no sensor can tell (how heavy and how slippery the untouched
    things are, the scene's name): the hub's answers are identical, byte for byte."""
    a = scenes.read("open_small")
    b = scenes.read("open_small")
    for o in b["objects"]:
        o["mass_kg"], o["friction"] = o["mass_kg"] * 3, 0.9
    b["surface"]["friction"] = 0.8
    b["description"] = "another place altogether"
    answers = []
    for scene in (a, b):
        hub = Hub(World(scene).ports)
        got = [every_get(hub)]
        for path, body in (("/devices/arm/move_joints", {"shoulder_pan": 0.4}), ("/devices/arm/gripper/set", {"opening": 0.3}),
                           ("/devices/arm/move_tip", {"x": 0.2, "y": 0.1, "z": 0.12}), ("/wait", {"seconds": 0.3})):
            got.append(json.dumps(call(hub, "POST", path, body)[1]))
        got.append(json.dumps(call(hub, "GET", "/devices/arm/state")[1]))
        answers.append(got)
    assert answers[0] == answers[1]


# ---------------------------------------------------------------- new places
@pytest.mark.parametrize("name", scenes.names())
def test_the_hub_finds_what_is_there_in_every_scene(name):
    world = World(name)
    hub = Hub(world.ports)
    kinds = [d["kind"] for d in world.scene["devices"]]
    devices = call(hub, "GET", "/tree")[1]["tree"]["under_it"]["devices"].get("under_it", {})
    assert ("arm" in devices) == ("arm" in kinds)
    assert sum(1 for n in devices if n.startswith("camera_")) == kinds.count("camera")
    if "arm" in kinds:
        true_wrist = next(c.identity["serial"] for c in world.ports().cameras if c.camera == "wrist")
        assert hub.wrist is not None and hub.wrist.serial == true_wrist, "the wrong camera was taken to be the wrist's"
        assert "wrist_camera" in devices["arm"]["under_it"]
        assert np.abs(world.data.qpos[world.buses[0].qadr] - world.buses[0].hold)[:4].max() < 0.1, "finding the wrist camera moved the arm away"
    else:
        assert hub.wrist is None and hub.arm is None
    members = call(hub, "GET", "/groups/cameras")[1]["under_it"]
    assert len(members) == kinds.count("camera") + ("arm" in kinds)


PICKS = [("workcell", "cube", 0.03), ("open_small", "red_cube", 0.03), ("moved_arm", "red_cube", 0.03),
         ("bench", "blue_cylinder", 0.03), ("cluttered", "red_cube_1", 0.03)]


@pytest.mark.parametrize("name,thing,width", PICKS, ids=[p[0] for p in PICKS])
def test_a_pick_through_the_hub_in_a_place_it_was_told_nothing_about(name, thing, width):
    """The test plays a model that has already found the object (it asks the referee where it is in the
    arm's frame and which way it is turned). Everything else goes through the hub's API, and the referee
    says whether it was lifted. In moved_arm the cube sits 45 degrees to the arm's axes: jaws set square
    to the arm would land on its corner."""
    world = World(name)
    hub = Hub(world.ports)
    p = in_arm_frame(world, thing)
    grasp_z = max(0.011, 2 * p[2] - 0.025)                 # jaws 25 mm down from its top: a tall thing is held near its top
    assert call(hub, "POST", "/devices/arm/gripper/set", {"gap_m": width + 0.014})[1]["ok"]
    r = call(hub, "POST", "/devices/arm/move_tip", {"x": p[0], "y": p[1], "z": grasp_z + 0.08,
                                                    "jaws_along_deg": heading_in_arm_frame(world, thing)})[1]
    assert r["ok"] and r["reached"], r
    r = call(hub, "POST", "/devices/arm/move_tip", {"x": p[0], "y": p[1], "z": grasp_z})[1]
    assert r["reached"], r.get("why_not_reached")
    r = call(hub, "POST", "/devices/arm/gripper/set", {"opening": 0})[1]
    assert r["holding"] and abs(r["state"]["gripper"]["gap_m"] - width) < 0.006, r["state"]["gripper"]
    r = call(hub, "POST", "/devices/arm/move_tip", {"x": p[0], "y": p[1], "z": grasp_z + 0.10})[1]
    assert r["reached"], r.get("why_not_reached")
    assert in_arm_frame(world, thing)[2] > p[2] + 0.07, "the object did not come up with the gripper"
    assert "torque off" in hub.shutdown()
    assert all(s.get("torque_switch") == 0 for s in world.buses[0].servos)


# ---------------------------------------------------------------- where
def test_where_gives_an_estimate_with_honest_uncertainty_or_says_it_cannot_tell(started):
    world, hub = started
    status, r = call(hub, "GET", "/where?of=camera_1&relative_to=arm")
    assert status == 200 and r["known"], r
    truth = world.camera_pose(world.scene["devices"][1]["name"], "base")
    err = np.linalg.norm(np.array(r["position_m"]) - truth[:3, 3])
    assert err < 0.06 * np.linalg.norm(truth[:3, 3]) and err < 3.5 * r["uncertainty_m"], (err, r)
    assert np.dot(r["looking_along"], truth[:3, 2]) > 0.995
    back = call(hub, "GET", "/where?of=arm&relative_to=camera_1")[1]
    assert back["known"] and abs(np.linalg.norm(back["position_m"]) - np.linalg.norm(r["position_m"])) < 0.02
    for q, code in (("of=camera_9", 404), ("of=arm&relative_to=arm", 400), ("relative_to=arm", 400)):
        assert call(hub, "GET", f"/where?{q}")[0] == code


def test_where_is_not_known_when_the_camera_cannot_see_the_arm():
    world = World("l_shaped")                                # its camera looks past the arm's base
    r = call(Hub(world.ports), "GET", "/where?of=camera_1&relative_to=arm")[1]
    assert r["ok"] and r["known"] is False and "tags" in r["why"] and "position_m" not in r


def test_a_camera_moved_three_centimetres_is_followed():
    found = []
    for x in (0.66, 0.69):
        s = scenes.read("open_small")
        next(d for d in s["devices"] if d["kind"] == "camera")["at_m"] = [x, 0.50, 0.36]
        r = call(Hub(World(s).ports), "GET", "/where?of=camera_1&relative_to=arm")[1]
        assert r["known"], r
        found.append(np.array(r["position_m"]))
    moved = found[1] - found[0]
    assert abs(moved[0] - 0.03) < 0.012 and np.abs(moved[1:]).max() < 0.012, moved


# ---------------------------------------------------------------- stopping, refusing, vanishing
def test_stop_and_torque_through_the_tree():
    world = World("open_small")
    hub = Hub(world.ports)
    call(hub, "POST", "/devices/arm/move_joints", {"shoulder_pan": 0.5, "timeout_s": 0.15})        # still on its way when this returns
    r = call(hub, "POST", "/stop")[1]
    assert r["stopped"] == ["devices/arm"]
    call(hub, "POST", "/wait", {"seconds": 0.6})
    s = call(hub, "GET", "/devices/arm/state")[1]
    assert not s["moving"] and abs(s["joints"]["joints_rad"]["shoulder_pan"]) < 0.45, "the stop did not stop the move"
    assert call(hub, "POST", "/devices/arm/torque_off")[1]["ok"]
    for path, body in (("/devices/arm/move_tip", {"x": 0.2, "y": 0.0, "z": 0.1}), ("/devices/arm/gripper/set", {"opening": 1})):
        status, r = call(hub, "POST", path, body)
        assert status == 409 and "torque" in r["error"]
    call(hub, "POST", "/wait", {"seconds": 0.5})
    assert max(abs(float(s.torque)) for s in world.buses[0].servos) == 0.0
    assert call(hub, "POST", "/devices/arm/torque_on")[1]["ok"]
    assert call(hub, "POST", "/devices/arm/move_joints", {"shoulder_pan": 0.2})[1]["ok"]


BAD = [
    ("GET", "/devices/nobody", None, 404, "nothing at"), ("GET", "/devices/arm/joints/elbow/state", None, 404, "nothing at"),
    ("POST", "/devices/arm/juggle", {}, 404, "no command"), ("POST", "/devices/camera_1/move_tip", {"x": 0.2, "y": 0, "z": 0.1}, 404, "no command"),
    ("POST", "/devices/arm/move_tip", {"x": 0.2, "y": 0.0}, 400, "'z' is required"),
    ("POST", "/devices/arm/move_tip", {"x": 9, "y": 0, "z": 0.1}, 400, "outside"),
    ("POST", "/devices/arm/move_tip", {"x": "far", "y": 0, "z": 0.1}, 400, "finite number"),
    ("POST", "/devices/arm/move_tip", {"x": 0.2, "y": 0, "z": 0.1, "speed": 3}, 400, "unknown field"),
    ("POST", "/devices/arm/move_tip", {"x": 0.45, "y": 0.3, "z": 0.3}, 400, "cannot be reached"),
    ("POST", "/devices/arm/move_joints", {}, 400, "at least one joint"), ("POST", "/devices/arm/gripper/set", {}, 400, "exactly one"),
    ("POST", "/devices/arm/gripper/set", [1], 400, "JSON object"), ("POST", "/wait", {"seconds": 99}, 400, "outside"),
    ("GET", "/devices/arm/picture", None, 404, "not a camera"), ("GET", "/devices/camera_1/picture?width=3", None, 400, "width"),
    ("PUT", "/devices/arm", {}, 405, "GET and POST"), ("DELETE", "/", None, 405, "GET and POST"),
]


@pytest.mark.parametrize("method,path,body,status,words", BAD, ids=[f"{m} {p} {i}" for i, (m, p, *_) in enumerate(BAD)])
def test_a_refused_request_says_why_and_moves_nothing(started, method, path, body, status, words):
    world, hub = started
    before = world.data.qpos.copy(), world.time
    got, r = call(hub, method, path, body)
    assert got == status and r["ok"] is False and words in r["error"], (got, r)
    # (working out why a point cannot be reached reads the encoders, which takes a few milliseconds on the bus)
    assert np.abs(world.data.qpos - before[0]).max() < 1e-3 and world.time - before[1] < 0.1, "a refused request moved something"


@pytest.mark.parametrize("seed", range(3))
def test_no_request_gets_a_traceback(seed):
    rng = random.Random(seed)
    hub = Hub(World("open_small").ports)
    paths = ["/", "/tree", "/where", "/where?of=camera_1", "/devices", "/devices/arm", "/devices/arm/state", "/devices/arm/move_tip",
             "/devices/arm/gripper/set", "/devices/arm/wrist_camera/picture", "/devices/camera_1/picture?tags=true&width=200", "/stop",
             "/wait", "/groups/cameras/state", "/place/state", "/%00", "//", "/devices/../..", "/" + "a/" * 200]
    bodies = [None, {}, [], 7, "x", {"x": 0.2, "y": 0.0, "z": 0.1}, {"x": 1e308, "y": -1e308, "z": 0}, {"seconds": 0.05}, {"opening": 0.5},
              {"x": None}, {"timeout_s": "soon"}, {"a": {"b": {"c": 1}}}]
    for _ in range(60):
        method, path, body = rng.choice(["GET", "POST", "PUT", "BREW"]), rng.choice(paths), rng.choice(bodies)
        raw = rng.choice([b"{", b"\xff\xfe"]) if rng.random() < 0.1 else (b"" if body is None else json.dumps(body).encode())
        status, ctype, out = A.dispatch(hub, method, path, raw)
        assert status != 500, (method, path, raw, out)
        if ctype == "application/json":
            r = json.loads(out)
            assert isinstance(r["ok"], bool) and (r["ok"] or r["error"])


def test_devices_that_vanish_are_said_to_have_vanished_and_are_found_again():
    """The scene is swapped under the hub: to it, everything goes quiet. No traceback, no stale answer;
    after discover it has the new place's devices and none of the old."""
    world = World("open_small")
    hub = Hub(world.ports)
    assert call(hub, "GET", "/devices/arm/state")[0] == 200
    world.load("no_arm")
    for method, path, body in (("POST", "/devices/arm/move_joints", {"shoulder_pan": 0.2}), ("GET", "/devices/camera_1/picture", None),
                               ("GET", "/where?of=camera_1&relative_to=arm", None)):
        status, r = call(hub, method, path, body)
        assert status == 503 and "did not answer" in r["error"] and "discover" in r["error"], (path, status, r)
    r = call(hub, "POST", "/discover")[1]
    assert sorted(r["devices"]) == ["camera_1", "camera_2"] and hub.arm is None
    assert call(hub, "GET", "/devices/arm")[0] == 404
    assert call(hub, "GET", "/devices/camera_2/picture?width=320")[0] == 200
    world.load("moved_arm")
    assert sorted(call(hub, "POST", "/discover")[1]["devices"]) == ["arm", "camera_1"]
    assert call(hub, "POST", "/devices/arm/move_joints", {"shoulder_pan": 0.2})[1]["reached"]


# ---------------------------------------------------------------- the API itself
def test_the_guide_covers_every_command_and_parameter(started):
    _, hub = started
    text = call(hub, "GET", "/")[1].decode()
    for n in [hub.root] + hub.root.under():
        for name, c in n.commands.items():
            assert f"POST /{(n.path + '/') if n.path else ''}{name} " in text and c.description in text
            for p, spec in c.params.items():
                assert spec.description in text, (name, p)
        if n.path:                                          # (GET / is the guide itself)
            assert set(call(hub, "GET", "/" + n.path)[1]["commands"]) == set(n.commands)


def test_pictures_come_as_jpeg_or_json_with_tags(started):
    _, hub = started
    status, ctype, out = A.dispatch(hub, "GET", "/devices/camera_1/picture?width=640")
    assert status == 200 and ctype == "image/jpeg" and out[:2] == b"\xff\xd8"
    r = call(hub, "GET", "/devices/camera_1/picture?tags=true&width=320&base64=true")[1]
    assert r["width"] == 320 and r["media_type"] == "image/jpeg" and r["data"]
    assert r["tags"] and all(t["on_a"] == "arm" and len(t["corners_px"]) == 4 for t in r["tags"])


def test_it_works_over_a_real_socket_and_the_launcher_answers_help(started):
    _, hub = started
    server = HTTPServer(("127.0.0.1", 0), A.make_handler(hub, verbose=False))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        base = f"http://127.0.0.1:{server.server_address[1]}"
        assert b"What was found" in urllib.request.urlopen(base + "/").read()
        req = urllib.request.Request(base + "/wait", data=b'{"seconds": 0.05}', method="POST")
        assert json.loads(urllib.request.urlopen(req).read())["ok"]
    finally:
        server.shutdown()
        server.server_close()
    r = subprocess.run([sys.executable, str(SIM / "scripts" / "run_system.py"), "--help"], capture_output=True, text=True, timeout=60)
    assert r.returncode == 0 and "operator" in r.stdout
