"""The operator's REST API: scenes listed, edited, validated, loaded and swapped; the referee.

Aimed at classes of bugs:
  * an edit that leaves a broken scene on disk or in the running world (every refusal changes nothing);
  * an edit to the loaded scene that the world does not pick up, or a load that leaves old state behind;
  * the referee scoring the wrong thing: each kind of goal is made true and made false through the API;
  * a request, however malformed, that is answered with anything but JSON and a sentence;
  * the guide and the table of endpoints drifting apart.
"""
import json
import random
import shutil
import threading
import urllib.request
from http.server import HTTPServer

import numpy as np
import pytest

import operator_api as A
from so101sim import scenes
from so101sim.world import World


@pytest.fixture()
def op(tmp_path):
    """An operator over a private copy of the scene files, with open_small loaded."""
    for p in scenes.SCENE_DIR.glob("*.toml"):
        shutil.copy(p, tmp_path / p.name)
    return A.Operator(World("open_small", directory=tmp_path))


def call(op, method, path, body=None):
    status, ctype, out = A.dispatch(op, method, path, b"" if body is None else json.dumps(body).encode())
    return status, (json.loads(out) if ctype == "application/json" else out.decode())


def files(op):
    return {p.name: p.read_text() for p in op.directory.glob("*.toml")}


# ---------------------------------------------------------------- scenes
def test_every_scene_file_is_listed_and_read_back_as_it_is(op):
    status, r = call(op, "GET", "/scenes")
    assert status == 200 and [s["name"] for s in r["scenes"]] == scenes.names()
    assert [s["name"] for s in r["scenes"] if s["loaded"]] == ["open_small"]
    for name in scenes.names():
        assert call(op, "GET", f"/scenes/{name}")[1]["scene"] == scenes.read(name)


def test_a_new_scene_can_be_made_checked_loaded_and_deleted(op):
    scene = scenes.read("open_small")
    scene["name"] = "mine"
    scene["objects"].append({"name": "extra", "shape": "box", "size_m": [0.02, 0.02, 0.02], "mass_kg": 0.01,
                             "colour": "green", "at_m": [0.3, 0.45]})
    assert call(op, "POST", "/scenes/mine/validate", scene)[1]["valid"]
    assert not (op.directory / "mine.toml").exists(), "validating saved the scene"
    status, r = call(op, "PUT", "/scenes/mine", scene)
    assert status == 200 and not r["reloaded"]
    assert call(op, "GET", "/scenes/mine")[1]["scene"] == scene
    status, r = call(op, "POST", "/world/load", {"scene": "mine"})
    assert status == 200 and r["scene"] == "mine"
    assert "extra" in call(op, "GET", "/world/truth")[1]["objects"]
    assert call(op, "DELETE", "/scenes/mine")[0] == 409, "the loaded scene was deleted"
    call(op, "POST", "/world/load", {"scene": "open_small"})
    assert call(op, "DELETE", "/scenes/mine")[0] == 200 and "mine" not in scenes.names(op.directory)


def test_a_patch_changes_only_what_it_names(op):
    before = scenes.read("shelf", op.directory)
    status, r = call(op, "PATCH", "/scenes/shelf", {
        "surface": {"lip_height_m": 0.03},
        "objects": {"yellow_cylinder": {"at_m": [0.75, 0.2]}, "green_block": None,
                    "new_cube": {"shape": "box", "size_m": [0.02, 0.02, 0.02], "mass_kg": 0.01, "colour": "blue", "at_m": [0.2, 0.6]}},
        "tasks": {"under_the_ledge": None},
        "description": None})
    assert status == 200, r
    after = scenes.read("shelf", op.directory)
    assert after["surface"] == {**before["surface"], "lip_height_m": 0.03}
    assert [o["name"] for o in after["objects"]] == ["red_cube", "yellow_cylinder", "new_cube"]
    assert next(o for o in after["objects"] if o["name"] == "yellow_cylinder")["at_m"] == [0.75, 0.2]
    assert next(o for o in after["objects"] if o["name"] == "red_cube") == next(o for o in before["objects"] if o["name"] == "red_cube")
    assert [t["name"] for t in after["tasks"]] == ["fetch_from_shelf", "on_the_shelf"] and "description" not in after
    assert after["fixtures"] == before["fixtures"] and after["devices"] == before["devices"]


def test_changing_the_loaded_scene_restarts_the_world_from_the_new_file(op):
    call(op, "POST", "/world/step", {"seconds": 0.3})
    gen = call(op, "GET", "/world")[1]["generation"]
    status, r = call(op, "PATCH", "/scenes/open_small", {"objects": {"red_cube": {"at_m": [0.5, 0.4]}}})
    assert status == 200 and r["reloaded"] and r["generation"] == gen + 1
    w = call(op, "GET", "/world")[1]
    assert w["time_s"] == 0.0
    assert np.allclose(call(op, "GET", "/world/truth")[1]["objects"]["red_cube"]["position_m"][:2], [0.5, 0.4], atol=1e-3)
    assert call(op, "PATCH", "/scenes/shelf", {"surface": {"lip_height_m": 0.025}})[1]["reloaded"] is False


BAD_EDITS = [
    ("PUT", "/scenes/open_small", {"name": "open_small"}, 422, "is required"),
    ("PUT", "/scenes/open_small", [1, 2, 3], 400, "JSON object"),
    ("PUT", "/scenes/other_name", {"name": "open_small"}, 400, "must match"),
    ("PATCH", "/scenes/open_small", {"objects": {"red_cube": {"at_m": [3.0, 3.0]}}}, 422, "past the edge"),
    ("PATCH", "/scenes/open_small", {"objects": {"red_cube": {"at_m": [0.33, 0.185]}}}, 422, "overlap"),
    ("PATCH", "/scenes/open_small", {"surface": {"lip_height_m": 5}}, 422, "outside"),
    ("PATCH", "/scenes/open_small", {"surface": {"outline_m": [[0, 0], [1, 1], [0, 1]]}}, 422, "four"),
    ("PATCH", "/scenes/open_small", {"devices": {"arm": None, "rover": None, "camera_1": None}}, 422, "required"),
    ("PATCH", "/scenes/open_small", {"objects": {"ghost": None}}, 404, "no 'ghost'"),
    ("PATCH", "/scenes/open_small", {"objects": {"red_cube": 7}}, 400, "table of changes"),
    ("PATCH", "/scenes/open_small", {"wibble": 1}, 422, "not a field"),
    ("PATCH", "/scenes/nowhere", {"description": "x"}, 404, "no scene"),
    ("PUT", "/scenes/bad name!", {"name": "bad name!"}, 422, ""),
    ("DELETE", "/scenes/nowhere", None, 404, "no scene"),
    ("POST", "/world/load", {"scene": "nowhere"}, 404, "no scene"),
    ("POST", "/world/load", {}, 400, "scene"),
    ("POST", "/world/step", {"seconds": -1}, 400, "0 to 60"),
    ("POST", "/world/step", {"seconds": "long"}, 400, "0 to 60"),
    ("GET", "/world/tasks/juggle", None, 404, "no task"),
    ("POST", "/world/tasks/how_big/answer", {}, 400, "hand in"),
    ("POST", "/world/tasks/juggle/answer", {"x": 1}, 404, "no task"),
    ("GET", "/nothing/here", None, 404, "no such endpoint"),
    ("DELETE", "/world", None, 405, "takes"),
    ("POST", "/scenes", None, 405, "takes"),
]


@pytest.mark.parametrize("method,path,body,status,words", BAD_EDITS, ids=[f"{m} {p} {i}" for i, (m, p, *_) in enumerate(BAD_EDITS)])
def test_a_refused_request_says_why_and_changes_nothing(op, method, path, body, status, words):
    before, truth, info = files(op), call(op, "GET", "/world/truth")[1], call(op, "GET", "/world")[1]
    got, r = call(op, method, path, body)
    assert got in (status, 400, 422) if status == 422 else got == status, (got, r)
    assert r["ok"] is False and words in (r["error"] + " ".join(r.get("problems", []))), r
    assert files(op) == before, "a refused request changed a scene file"
    assert call(op, "GET", "/world/truth")[1] == truth and call(op, "GET", "/world")[1] == info, "a refused request changed the world"


@pytest.mark.parametrize("seed", range(4))
def test_no_request_gets_anything_but_json_and_a_sentence(op, seed):
    rng = random.Random(seed)
    before = files(op)
    paths = ["/", "/scenes", "/scenes/open_small", "/scenes/x", "/scenes/open_small/validate", "/world", "/world/load",
             "/world/reset", "/world/step", "/world/truth", "/world/tasks", "/world/tasks/how_big",
             "/world/tasks/how_big/answer", "/world/tasks/x/answer", "/%00", "//", "/scenes/../../etc", "/scenes/" + "a" * 300]
    bodies = [b"", b"{", b"null", b"[]", b"7", b'{"scene": 3}', b'{"seconds": 1e999}', b'{"objects": {"red_cube": {"at_m": "here"}}}',
              b'{"surface": null}', b"\xff\xfe", b'{"a": ' * 200]
    for _ in range(120):
        method = rng.choice(["GET", "POST", "PUT", "PATCH", "DELETE", "BREW"])
        path, body = rng.choice(paths), rng.choice(bodies)
        if method == "DELETE" or (method in ("PUT", "PATCH") and body in (b"null", b"[]", b"7", b"")):
            path = rng.choice(["/scenes/x", "/world", "/nope"])              # (do not really delete or blank the fixtures' files)
        status, ctype, out = A.dispatch(op, method, path, body)
        assert status != 500, (method, path, body, out)
        if ctype == "application/json":
            r = json.loads(out)
            assert isinstance(r["ok"], bool) and (r["ok"] or (isinstance(r["error"], str) and r["error"]))
    for name, text in before.items():
        assert scenes.validate(scenes.toml_io.loads((op.directory / name).read_text())) == [], f"{name} was left invalid"


# ---------------------------------------------------------------- the world and the referee
def test_loading_another_scene_leaves_nothing_of_the_old_one(op):
    call(op, "POST", "/world/tasks/how_big/answer", {"area_m2": 0.385})
    a = call(op, "GET", "/world/truth")[1]
    status, w = call(op, "POST", "/world/load", {"scene": "shelf"})
    assert status == 200 and w["scene"] == "shelf" and w["time_s"] == 0.0 and w["generation"] == a["generation"] + 1
    b = call(op, "GET", "/world/truth")[1]
    assert set(b["objects"]) == {o["name"] for o in scenes.read("shelf")["objects"]} and "shelf" in b["fixtures"]
    assert call(op, "GET", "/world/tasks/how_big")[0] == 404
    call(op, "POST", "/world/load", {"scene": "open_small"})
    assert call(op, "GET", "/world/tasks/how_big")[1]["done"] is False, "an answer survived the scene being swapped"


def test_truth_follows_the_physics_and_reset_puts_it_back(op):
    w = op.world
    start = call(op, "GET", "/world/truth")[1]
    cube = w.model.body("red_cube")
    w.data.qpos[w.model.jnt_qposadr[cube.jntadr[0]]] += 0.05
    call(op, "POST", "/world/step", {"seconds": 0.4})
    moved = call(op, "GET", "/world/truth")[1]
    assert abs(moved["objects"]["red_cube"]["position_m"][0] - start["objects"]["red_cube"]["position_m"][0] - 0.05) < 2e-3
    assert moved["time_s"] == pytest.approx(0.4, abs=1e-6)
    call(op, "POST", "/world/reset")
    assert call(op, "GET", "/world/truth")[1]["objects"] == start["objects"]


def _place(world, body, xyz, quat=(1, 0, 0, 0)):
    m = world.model
    adr = m.jnt_qposadr[m.body(body).jntadr[0]]
    world.data.qpos[adr:adr + 3] = xyz
    world.data.qpos[adr + 3:adr + 7] = quat
    world.data.qvel[m.jnt_dofadr[m.body(body).jntadr[0]]:][:6] = 0


def goals(op, task):
    return {i: g["done"] for i, g in enumerate(call(op, "GET", f"/world/tasks/{task}")[1]["goals"])}


def test_arrangement_goals_follow_the_things_themselves(op):
    """Each physical goal of the bench task becomes true when its thing is put there, and no sooner."""
    call(op, "POST", "/world/load", {"scene": "bench"})
    w = op.world
    assert not any(goals(op, "tidy").values()) or goals(op, "tidy")[8], "only 'drawer closed' may hold at the start"
    first = goals(op, "tidy")
    assert first == {0: False, 1: False, 2: True, 3: False, 4: True, 5: False, 6: True, 7: False, 8: True}
    _place(w, "red_cube", [0.10, 0.24, 0.0152])
    _place(w, "blue_cylinder", [0.13, -0.23, 0.0302])
    _place(w, "yellow_peg", [0.30, -0.02, 0.0302])
    _place(w, "green_block", [0.27, -0.14, 0.0502])
    _place(w, "purple_cube", [0.27, -0.14, 0.0728])
    call(op, "POST", "/world/step", {"seconds": 0.5})
    r = call(op, "GET", "/world/tasks/tidy")[1]
    assert r["done"], r["goals"]
    _place(w, "blue_cylinder", [0.13, -0.23, 0.0152], quat=(0.7071, 0, 0.7071, 0))        # lying down in its bin
    call(op, "POST", "/world/step", {"seconds": 0.5})
    after = goals(op, "tidy")
    assert after[2] is False and not call(op, "GET", "/world/tasks/tidy")[1]["done"]
    w.data.qpos[w.model.joint("cabinet_drawer").qposadr[0]] = 0.05
    call(op, "POST", "/world/step", {"seconds": 0.05})
    assert goals(op, "tidy")[8] is False


ANSWERS = [
    ("open_small", "how_big", {"area_m2": 0.385}, True), ("open_small", "how_big", {"area_m2": 0.36}, True),
    ("open_small", "how_big", {"area_m2": 0.30}, False), ("open_small", "how_big", {"area_m2": "big"}, False),
    ("open_small", "how_big", {"wrong_key": 0.385}, False),
    ("l_shaped", "far_leg", {"count": 3, "colours": ["purple", "Yellow", "green"]}, True),
    ("l_shaped", "far_leg", {"count": 3, "colours": ["purple", "yellow"]}, False),
    ("l_shaped", "far_leg", {"count": 4, "colours": ["purple", "yellow", "green"]}, False),
    ("shelf", "on_the_shelf", {"colours": ["red"], "height_m": 0.085}, True),
    ("shelf", "on_the_shelf", {"colours": ["red"], "height_m": 0.12}, False),
    ("shelf", "under_the_ledge", {"colours": ["green"]}, True), ("shelf", "under_the_ledge", {"colours": []}, False),
    ("bench", "drawer_open", {"open": "No"}, True), ("bench", "drawer_open", {"open": "yes"}, False),
]


@pytest.mark.parametrize("scene,task,answers,right", ANSWERS, ids=[f"{s}:{t}:{i}" for i, (s, t, *_) in enumerate(ANSWERS)])
def test_answers_are_scored_against_the_truth_and_the_caller_is_not_told(op, scene, task, answers, right):
    call(op, "POST", "/world/load", {"scene": scene})
    status, r = call(op, "POST", f"/world/tasks/{task}/answer", answers)
    assert status == 200 and set(r) == {"ok", "task", "received"}, "handing in an answer must not say whether it is right"
    assert call(op, "GET", f"/world/tasks/{task}")[1]["done"] is right


def test_the_drawer_answer_changes_when_the_operator_opens_the_drawer(op):
    call(op, "POST", "/world/load", {"scene": "bench"})
    call(op, "POST", "/world/tasks/drawer_open/answer", {"open": "no"})
    assert call(op, "GET", "/world/tasks/drawer_open")[1]["done"]
    status, r = call(op, "PATCH", "/scenes/bench", {"fixtures": {"cabinet": {"drawer_open_m": 0.06}}})
    assert status == 422 and any("red_cube" in p for p in r["problems"]), "the cube in the drawer was left behind, inside its back wall"
    r = call(op, "PATCH", "/scenes/bench", {"fixtures": {"cabinet": {"drawer_open_m": 0.06}},
                                           "objects": {"red_cube": {"at_m": [0.217, 0.13, 0.004]}}})[1]      # the cube rides out with it
    assert r["reloaded"], r
    call(op, "POST", "/world/tasks/drawer_open/answer", {"open": "no"})
    assert not call(op, "GET", "/world/tasks/drawer_open")[1]["done"]
    call(op, "POST", "/world/tasks/drawer_open/answer", {"open": "yes"})
    assert call(op, "GET", "/world/tasks/drawer_open")[1]["done"]


def test_where_is_everyone_is_scored_in_the_named_devices_own_frame(op):
    """moved_arm: the arm faces 225 deg, so the rover's place in the arm's frame is not its place in the scene's."""
    call(op, "POST", "/world/load", {"scene": "moved_arm"})
    t = call(op, "GET", "/world/truth")[1]["devices"]
    arm, rover = np.array(t["arm"]["position_m"][:2]), np.array(t["rover"]["position_m"][:2])
    yaw = np.radians(t["arm"]["yaw_deg"])
    d = rover - arm
    rel = [float(np.cos(yaw) * d[0] + np.sin(yaw) * d[1]), float(-np.sin(yaw) * d[0] + np.cos(yaw) * d[1])]
    assert rel[0] > 0.3, "the rover is ahead of the arm in this scene"
    for answer, right in (({"x": rel[0], "y": rel[1]}, True), ({"x": rel[0] + 0.02, "y": rel[1]}, True),
                          ({"x": float(d[0]), "y": float(d[1])}, False), ({"x": rel[0] + 0.05, "y": rel[1]}, False), ({"x": "far"}, False)):
        call(op, "POST", "/world/tasks/where_is_everyone/answer", {"pose": answer})
        assert call(op, "GET", "/world/tasks/where_is_everyone")[1]["done"] is right, answer


def test_a_task_that_swaps_scenes_is_scored_in_the_scene_swapped_to(op):
    call(op, "POST", "/world/load", {"scene": "moved_arm"})
    r = call(op, "GET", "/world/tasks/carried")[1]
    assert r["done"] is False and r["waiting_for_swap_to"] == "open_small"
    call(op, "POST", "/world/load", {"scene": "open_small"})
    t = call(op, "GET", "/world/truth")[1]["devices"]
    d = np.array(t["rover"]["position_m"][:2]) - np.array(t["arm"]["position_m"][:2])
    call(op, "POST", "/world/tasks/carried/answer", {"moved": "yes", "pose": {"x": float(d[0]), "y": float(d[1])}})
    assert call(op, "GET", "/world/tasks/carried")[1]["done"]
    call(op, "POST", "/world/tasks/carried/answer", {"moved": "no"})
    assert not call(op, "GET", "/world/tasks/carried")[1]["done"]


# ---------------------------------------------------------------- the API itself
def test_the_guide_lists_every_endpoint_and_every_endpoint_answers(op):
    text = call(op, "GET", "/")[1]
    for method, path, what in A.ROUTES:
        assert f"{method:6s} {path}" in text and what in text
        concrete = path.replace("{name}", "how_big" if "tasks" in path else "open_small")
        status, _, _ = A.dispatch(op, method, concrete, b'{"scene": "open_small", "seconds": 0, "area_m2": 1}' if method == "POST" else
                                  json.dumps(scenes.read("open_small", op.directory)).encode() if method == "PUT" else b"{}")
        assert status in (200, 409), (method, path, status)        # 409: deleting the loaded scene


def test_it_works_over_a_real_socket(op):
    server = HTTPServer(("127.0.0.1", 0), A.make_handler(op, verbose=False))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        base = f"http://127.0.0.1:{server.server_address[1]}"
        assert json.loads(urllib.request.urlopen(base + "/world").read())["scene"] == "open_small"
        req = urllib.request.Request(base + "/scenes/open_small", data=json.dumps({"surface": {"lip_height_m": 0.03}}).encode(), method="PATCH")
        assert json.loads(urllib.request.urlopen(req).read())["reloaded"] is True
        assert json.loads(urllib.request.urlopen(base + "/world/truth").read())["surface"]["lip_height_m"] == 0.03
    finally:
        server.shutdown()
        server.server_close()
