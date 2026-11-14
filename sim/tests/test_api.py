"""The REST API for a language model (sim/scripts/sim_api.py).

Aimed at classes of bugs:
  * a caller that knows only the API cannot do the job (a whole pick is done through requests alone);
  * a bad request breaks something: any malformed or out-of-range request (fuzzed) gets a JSON answer with
    ok = false and a reason, changes nothing, and leaves the server and the simulation sound;
  * time leaks: reading never advances the simulation, actions always do, and a run is repeatable;
  * the description drifts from the behaviour: the guide and the OpenAPI document come from the table
    that validates requests, and every endpoint in it answers.
"""
import io
import json
import random
import threading
import urllib.error
import urllib.request
from http.server import HTTPServer

import numpy as np
import pytest

import sim_api as A


@pytest.fixture(scope="module")
def api():
    return A.SimApi()


def call(api, method, url, body=None):
    status, ctype, data = A.dispatch(api, method, url, b"" if body is None else json.dumps(body).encode())
    return status, (json.loads(data) if ctype == "application/json" else data)


def test_a_pick_using_only_the_api(api):
    assert call(api, "POST", "/reset")[0] == 200
    _, st = call(api, "GET", "/state")
    cube = st["objects"]["cube"]
    steps = [("/gripper", {"gap_m": 0.044}), ("/move_tip", {"x": cube["x"], "y": cube["y"], "z": 0.09, "square_jaws": True}),
             ("/move_tip", {"x": cube["x"], "y": cube["y"], "z": 0.011}), ("/gripper", {"opening": 0}),
             ("/move_tip", {"x": cube["x"], "y": cube["y"], "z": 0.12})]
    for path, body in steps:
        status, r = call(api, "POST", path, body)
        assert status == 200 and r["ok"] and r["reached"], (path, body, r.get("message"), r.get("tip_error_m"))
        if path == "/gripper" and body.get("opening") == 0:
            assert r["holding"] and r["state"]["gripper"]["holding"]
    after = r["state"]
    assert after["objects"]["cube"]["z"] > cube["z"] + 0.08, "the cube was not lifted"
    assert not after["checks"]["physics_violations"]
    assert after["time_s"] > st["time_s"] + 1.0


def test_reading_costs_no_time_and_actions_do(api):
    call(api, "POST", "/reset")
    t0 = call(api, "GET", "/state")[1]["time_s"]
    for url in ("/state", "/cameras", "/camera/wrist", "/camera/scene_front?base64=true", "/report", "/", "/openapi.json"):
        assert call(api, "GET", url)[0] == 200, url
    assert call(api, "GET", "/state")[1]["time_s"] == t0
    _, r = call(api, "POST", "/wait", {"seconds": 0.5})
    assert r["state"]["time_s"] == pytest.approx(t0 + 0.5, abs=0.04) and r["elapsed_s"] == pytest.approx(0.5, abs=0.04)


def test_the_same_requests_give_the_same_run(api):
    def run():
        call(api, "POST", "/reset")
        out = []
        for path, body in (("/move_tip", {"x": 0.25, "y": -0.08, "z": 0.10}), ("/gripper", {"opening": 0.3}),
                           ("/move_joints", {"shoulder_pan": 0.4, "wrist_roll": -1.0}), ("/wait", {"seconds": 0.3})):
            out.append(call(api, "POST", path, body)[1])
        return json.dumps(out, sort_keys=True)
    assert run() == run()


def test_camera_pictures_are_jpegs_of_the_right_size(api):
    from PIL import Image
    import base64
    cams = call(api, "GET", "/cameras")[1]["cameras"]
    assert set(cams) == {"wrist", "scene_front", "scene_side"}
    for name, c in cams.items():
        status, data = call(api, "GET", f"/camera/{name}")
        img = Image.open(io.BytesIO(data))
        assert status == 200 and img.format == "JPEG" and img.size == (c["width"], c["height"])
        assert np.asarray(img).std() > 5, f"{name} is blank"
    r = call(api, "GET", "/camera/wrist?base64=true")[1]
    assert r["media_type"] == "image/jpeg" and Image.open(io.BytesIO(base64.b64decode(r["data"]))).size == (640, 360)
    status, r = call(api, "GET", "/camera/nope")
    assert status == 404 and "scene_front" in r["error"]


@pytest.mark.parametrize("path,body,words", [
    ("/move_tip", {"x": 0.9, "y": 0, "z": 0.1}, "outside"),                 # beyond the stated range
    ("/move_tip", {"x": 0.45, "y": 0.3, "z": 0.3}, "cannot be reached"),    # inside the range, outside the arm's reach
    ("/move_tip", {"x": 0.2, "y": 0}, "'z' is required"),
    ("/move_tip", {"x": "far", "y": 0, "z": 0.1}, "finite number"),
    ("/move_tip", {"x": 0.2, "y": 0, "z": 0.1, "speed": 3}, "unknown field"),
    ("/move_joints", {}, "at least one joint"),
    ("/move_joints", {"elbow_flex": 9.0}, "outside its range"),
    ("/move_joints", {"shoulder_lift": 1.2, "elbow_flex": -1.0, "wrist_flex": 1.6}, "below"),
    ("/gripper", {"opening": 1.5}, "outside"),
    ("/gripper", {}, "exactly one"),
    ("/gripper", {"opening": 0.5, "gap_m": 0.03}, "exactly one"),
    ("/wait", {"seconds": -1}, "outside"),
])
def test_a_request_that_cannot_be_done_says_why_and_changes_nothing(api, path, body, words):
    call(api, "POST", "/reset")
    before = call(api, "GET", "/state")[1]
    status, r = call(api, "POST", path, body)
    assert status == 400 and r["ok"] is False and words in r["error"], r
    assert call(api, "GET", "/state")[1] == before, "a refused request changed the simulation"


@pytest.mark.parametrize("seed", range(3))
def test_no_request_breaks_the_server_or_the_simulation(api, seed):
    """Fuzzed methods, paths and bodies. Every answer is JSON with ok, or a picture; never a crash or a 500."""
    rng = random.Random(seed)
    call(api, "POST", "/reset")
    paths = [r[1].replace("{name}", rng.choice(["wrist", "x", "../etc"])) for r in A.ROUTES] + ["/nope", "/move_tip/", "//"]
    junk = [None, [], "x", 3, {"x": None}, {"x": float("nan")}, {"x": 1e308, "y": -1e308, "z": 0}, {"opening": True},
            {"seconds": "5"}, {"x": 0.2, "y": 0.0, "z": 0.05}, {"opening": 0.5}, {"seconds": 0.1},
            {"shoulder_pan": rng.uniform(-3, 3)}, {"x": rng.uniform(-1, 1), "y": rng.uniform(-1, 1), "z": rng.uniform(-1, 1)}]
    for _ in range(120):
        method, path, body = rng.choice(["GET", "POST", "PUT"]), rng.choice(paths), rng.choice(junk)
        raw = rng.choice([None, b"{not json", b"\xff\xfe", b""])
        status, ctype, data = A.dispatch(api, method, path, raw if raw is not None else json.dumps(body).encode())
        assert status in (200, 400, 404, 405), (method, path, body, raw, data[:200])
        if ctype == "application/json":
            assert "ok" in json.loads(data) or path == "/openapi.json"
    st = call(api, "GET", "/state")[1]
    assert st["ok"] and not st["checks"]["physics_violations"] and all(np.isfinite(list(st["joints_rad"].values())))
    assert call(api, "POST", "/move_tip", {"x": 0.22, "y": 0.0, "z": 0.12})[1]["reached"], "the arm no longer obeys"


def test_the_guide_and_openapi_cover_every_endpoint_and_parameter():
    text, spec = A.guide(), A.openapi()
    for method, path, _, params in A.ROUTES:
        assert f"{method:4s} {path}" in text
        op = spec["paths"][path][method.lower()]
        described = (op.get("requestBody", {}).get("content", {}).get("application/json", {}).get("schema", {}).get("properties", {})
                     if method == "POST" else {p["name"]: p for p in op.get("parameters", [])})
        for name in params:
            assert name in text and name in described, (path, name)
    json.dumps(spec)


def test_object_positions_can_be_withheld():
    api = A.SimApi(privileged=False)
    assert "objects" not in call(api, "GET", "/state")[1]


def test_over_http():
    """The same thing through a real socket: the server thread owns the simulation and its renderer."""
    ready, box = threading.Event(), {}

    def serve():
        box["server"] = HTTPServer(("127.0.0.1", 0), A.make_handler(A.SimApi()))
        ready.set()
        box["server"].serve_forever()
    threading.Thread(target=serve, daemon=True).start()
    assert ready.wait(60)
    base = f"http://127.0.0.1:{box['server'].server_address[1]}"

    def http(method, path, body=None):
        req = urllib.request.Request(base + path, method=method, data=None if body is None else json.dumps(body).encode())
        try:
            with urllib.request.urlopen(req, timeout=120) as r:
                return r.status, r.headers["Content-Type"], r.read()
        except urllib.error.HTTPError as e:
            return e.code, e.headers["Content-Type"], e.read()
    try:
        assert b"Endpoints" in http("GET", "/")[2]
        status, ctype, data = http("POST", "/move_tip", {"x": 0.22, "y": 0.05, "z": 0.10})
        assert status == 200 and json.loads(data)["reached"]
        status, ctype, data = http("GET", "/camera/scene_side")
        assert status == 200 and ctype == "image/jpeg" and data[:2] == b"\xff\xd8"
        status, ctype, data = http("POST", "/gripper", {"opening": 7})
        assert status == 400 and json.loads(data)["ok"] is False
    finally:
        box["server"].shutdown()


def test_the_window_draws_while_actions_run():
    """--window: the watcher's window is drawn after every simulated frame (here on SDL's dummy display)."""
    api = A.SimApi()
    window = A.Window(api)
    frames = []
    hook = api.on_frame
    api.on_frame = lambda: (hook(), frames.append(api.app.sim.time))
    status, r = call(api, "POST", "/move_tip", {"x": 0.22, "y": 0.05, "z": 0.10})
    assert status == 200 and r["reached"] and len(frames) >= 10 and window.open
    assert window.screen.get_size() == (2 * A.T.VIEW_W, 2 * (A.T.VIEW_H + A.T.PANEL))
    assert np.asarray(window.pg.surfarray.array3d(window.screen)).std() > 5, "nothing was drawn"



def test_hands_on_window_shares_the_arm_with_the_api():
    """--hands-on: between requests the window's keys drive the arm and time passes; during a request's action
    they are left alone; and the API then acts from where the person left the arm."""
    api = A.SimApi()
    window = A.Window(api, hands_on=True)
    pg, app = window.pg, api.app
    t0, grip0 = app.sim.time, app.grip
    pg.event.post(pg.event.Event(pg.KEYDOWN, key=pg.K_SPACE))
    import time
    w0 = time.perf_counter()
    for _ in range(30):
        window.frame(idle=True)
    wall = time.perf_counter() - w0
    assert app.sim.time - t0 == pytest.approx(wall, rel=0.15, abs=0.05), "between requests time must run at real speed"
    assert app.grip != grip0, "the person's key did nothing"
    # A key that arrives while an action runs is not acted on (the API has the arm), and the action still works.
    grip1 = app.grip
    pg.event.post(pg.event.Event(pg.KEYDOWN, key=pg.K_SPACE))
    pg.event.post(pg.event.Event(pg.KEYDOWN, key=pg.K_p))
    status, r = call(api, "POST", "/move_tip", {"x": 0.22, "y": 0.05, "z": 0.10})
    assert status == 200 and r["reached"] and app.grip == grip1 and not app.paused and window.open
    # Paused by the person: an action unpauses it rather than timing out with nothing done.
    app.paused = True
    status, r = call(api, "POST", "/move_tip", {"x": 0.24, "y": -0.04, "z": 0.10})
    assert r["reached"] and r["tip_error_m"] < 0.006
    # Without hands_on the same key between requests does nothing and no time passes.
    api2 = A.SimApi()
    w2 = A.Window(api2)
    t, g = api2.app.sim.time, api2.app.grip
    w2.pg.event.post(w2.pg.event.Event(w2.pg.KEYDOWN, key=w2.pg.K_SPACE))
    w2.frame(idle=True)
    assert api2.app.sim.time == t and api2.app.grip == g


@pytest.mark.parametrize("scene", ["workcell", "bench"])
def test_randomize_leaves_every_object_standing_free_within_reach(scene):
    """/randomize, over many seeds: each object upright on the table, inside the reach band, clear of the others
    and of the fixtures, and at rest; the same seed gives the same layout and another seed a different one."""
    api = A.SimApi(scene=scene)
    layouts = {}
    for seed in range(12):
        status, r = call(api, "POST", "/randomize", {"seed": seed})
        assert status == 200 and r["seed"] == seed, r
        objs = r["objects"]
        layouts[seed] = objs
        m, d = api.inner.m, api.inner.d
        pts = []
        for name, o in objs.items():
            rad = float(np.hypot(o["x"], o["y"]))
            assert A.RANDOM_R[0] - 0.003 <= rad <= A.RANDOM_R[1] + 0.003, (seed, name, o)
            R = d.xmat[m.body(name).id].reshape(3, 3)
            assert R[2, 2] > 0.999, f"seed {seed}: {name} is not upright"
            assert np.abs(d.cvel[m.body(name).id]).max() < 0.02, f"seed {seed}: {name} is still moving"
            pts.append((o["x"], o["y"]))
        assert all(np.hypot(a[0] - b[0], a[1] - b[1]) > A.RANDOM_APART - 0.004 for i, a in enumerate(pts) for b in pts[:i])
        _, later = call(api, "POST", "/wait", {"seconds": 0.5})
        for name, o in objs.items():
            n = later["state"]["objects"][name]
            assert max(abs(n[k] - o[k]) for k in "xyz") < 0.001, f"seed {seed}: {name} moved after being placed (it was not free)"
        assert not later["state"]["checks"]["physics_violations"]
    _, again = call(api, "POST", "/randomize", {"seed": 3})
    assert again["objects"] == layouts[3] and layouts[3] != layouts[4]
    assert len({json.dumps(v) for v in layouts.values()}) == len(layouts)


def test_randomize_tells_a_caller_without_object_positions_nothing_about_them():
    api = A.SimApi(privileged=False)
    _, r = call(api, "POST", "/randomize", {})
    assert "objects" not in r and "fixtures" not in r and isinstance(r["seed"], int)
    _, r2 = call(api, "POST", "/randomize", {})
    assert r2["seed"] != r["seed"]


def test_the_guide_describes_the_server_it_is_served_by():
    """The guide must not send a caller to something this server does not give, and must explain what it can say.
    (A newcomer given only the guide found each of these wrong or missing on 2026-11-12.)"""
    import re
    import mujoco
    from pathlib import Path
    for scene in ("workcell", "bench"):
        for privileged in (True, False):
            api = A.SimApi(scene=scene, privileged=privileged)
            status, text = call(api, "GET", "/")
            text = text.decode()
            state = api.state()
            # object positions: pointed to only if /state has them
            assert ("objects.<name>" in text) == ("objects" in state)
            assert ('no "objects"' in text) == ("objects" not in state)
            # every loose object, with its true size
            m = api.inner.m
            for name in api.objects:
                line = next(ln for ln in text.splitlines() if ln.strip().startswith(name + " "))
                g = int(m.body_geomadr[m.body(name).id])
                for v in m.geom_size[g][:2 if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER else 3]:
                    assert f"{v * 2000:g}" in line, (name, line)
            # time: says so when a person shares the arm
            assert "only inside an action" in text and "window" not in text
            api.hands_on = True
            shared = call(api, "GET", "/")[1].decode()
            assert "between your requests" in shared and "only inside an action" not in shared
    text = A.guide()
    # every reason the command filter can give is explained, in the words the message uses
    src = (Path(A.__file__).resolve().parents[1] / "so101sim" / "safety.py").read_text()
    reasons = set(re.findall(r'_note\("(\w+)"', src))
    assert {"jump", "table", "joint_limit"} <= reasons
    explained = text[text.index("limited lists what the command filter did"):text.index("message is the last short thing")]
    for r in reasons:
        assert re.search(rf"^\s+{r}\b", explained, re.M), f"the guide does not explain the filter's '{r}'"
    assert "/pixel" not in text


def test_gripper_and_wait_answer_with_their_own_message(api):
    call(api, "POST", "/reset")
    assert call(api, "POST", "/move_tip", {"x": 0.22, "y": 0.0, "z": 0.12})[1]["message"] == "api: move_tip"
    assert call(api, "POST", "/wait", {"seconds": 0.2})[1]["message"] == "api: wait"
    assert "move_tip" not in call(api, "POST", "/gripper", {"opening": 0.5})[1]["message"]


def test_the_guides_account_of_the_wrist_camera_is_true():
    """The guide says: about 9 cm above the jaw tips, about 5 cm to one side, tilted about 25 degrees from the fingers."""
    api = A.SimApi()
    call(api, "POST", "/move_tip", {"x": 0.22, "y": 0.0, "z": 0.14})
    m, d = api.inner.m, api.inner.d
    c = m.camera("wrist").id
    rel = d.cam_xpos[c] - api.tip()
    assert 0.075 < rel[2] < 0.105 and 0.035 < np.hypot(rel[0], rel[1]) < 0.065
    look = -d.cam_xmat[c].reshape(3, 3)[:, 2]
    assert 20 < np.degrees(np.arccos(-look[2])) < 30
    text = A.guide()
    assert "about 9 cm above" in text and "about 5 cm" in text and "about 25 degrees" in text


@pytest.mark.parametrize("seed", range(4))
def test_the_gripper_moves_only_the_jaws(api, seed):
    """Whatever pose the arm was put in (move_tip or move_joints, fingers down or not), opening or closing the
    jaws leaves the five arm joints where they were commanded."""
    rng = random.Random(seed)
    call(api, "POST", "/reset")
    call(api, "POST", "/move_tip", {"x": 0.22, "y": 0.0, "z": 0.14})
    if seed % 2:
        pose = {"shoulder_pan": rng.uniform(-0.5, 0.5), "wrist_flex": rng.uniform(-0.3, 0.6), "wrist_roll": rng.uniform(-1.5, 0.3)}
        assert call(api, "POST", "/move_joints", pose)[1]["reached"]
    else:
        call(api, "POST", "/move_tip", {"x": rng.uniform(0.2, 0.28), "y": rng.uniform(-0.1, 0.1), "z": rng.uniform(0.03, 0.16)})
    before = np.array(list(call(api, "GET", "/state")[1]["joints_rad"].values())[:5])
    goal = api.app.goal[:5].copy()
    for body in ({"opening": 0.2}, {"gap_m": 0.05}, {"opening": 1.0}):
        _, r = call(api, "POST", "/gripper", body)
        assert r["ok"] and np.allclose(api.app.goal[:5], goal), "the gripper changed the arm's target"
        after = np.array(list(r["state"]["joints_rad"].values())[:5])
        assert np.abs(after - before).max() < 0.03, f"the arm moved {np.abs(after - before).max():.3f} rad when the jaws did"


@pytest.mark.parametrize("seed", range(3))
def test_a_move_tip_is_refused_whole_or_never_stops_for_want_of_a_pose(seed):
    """Random targets, near and far, reachable and not. Either the request is refused before anything moves, with
    the reason in numbers that are true, or it is accepted and then never stops part-way because the controller
    could not solve a point on its line (a newcomer got ok = true, reached = false, "unreachable" for that on
    2026-11-12, where other unreachable targets were refused)."""
    import re
    rng = random.Random(seed)
    api = A.SimApi()
    refused = accepted = 0
    for _ in range(14):
        body = {"x": rng.uniform(0.02, 0.45), "y": rng.uniform(-0.3, 0.3), "z": rng.uniform(0.004, 0.30)}
        before = call(api, "GET", "/state")[1]
        status, r = call(api, "POST", "/move_tip", body)
        if not r["ok"]:
            refused += 1
            after = call(api, "GET", "/state")[1]
            assert after["time_s"] == before["time_s"] and after["joints_rad"] == before["joints_rad"], "a refused move moved"
            assert "nothing was moved" in r["error"]
            m = re.search(r"The nearest the tip gets to \(([-\d.]+), ([-\d.]+), ([-\d.]+)\) is \(([-\d.]+), ([-\d.]+), ([-\d.]+)\): (\d+) mm short", r["error"])
            assert m, r["error"]
            v = [float(g) for g in m.groups()]
            assert abs(np.linalg.norm(np.array(v[:3]) - np.array(v[3:6])) * 1000 - v[6]) <= 1.5, "the distance it gives is not the distance between the points it gives"
            assert v[6] >= 4, "refused a point it says it can nearly reach"
            assert "can be about" in r["error"] or "nothing at that height" in r["error"] or "no height" in r["error"]
        else:
            accepted += 1
            assert api.app.failed_point is None, f"an accepted move stopped at a point it could not solve: {body} {r.get('why_not_reached')}"
            assert isinstance(r["limited"], list)
            if not r["reached"]:
                assert len(r["why_not_reached"]) > 40 and "mm" in r["why_not_reached"]
            else:
                assert "why_not_reached" not in r
    assert refused >= 2 and accepted >= 3, (refused, accepted)


def test_an_action_held_back_by_the_table_says_which_part_and_how_far():
    """Joint angles that would put the jaws through the table: the answer names the filter, the part and the depth,
    and the depth is the one the filter's own model gives for the pose that was asked for."""
    import re
    api = A.SimApi()
    call(api, "POST", "/move_tip", {"x": 0.22, "y": 0.0, "z": 0.06})
    asked = {"elbow_flex": float(api.app.goal[2]) + 0.6}     # the tip stays above the table; the wrist camera would not
    status, r = call(api, "POST", "/move_joints", {**asked, "timeout_s": 3})
    assert r["ok"] and not r["reached"], r
    assert "command filter held the arm back at the table" in r["why_not_reached"]
    line = next(t for t in r["limited"] if "below the lowest height" in t)
    depth = float(re.search(r"([\d.]+) mm below the lowest height", line).group(1))
    f = api.app.sim.safety
    pose = np.array(api.app.goal, float)
    assert abs(depth - f.depths(pose)[0] * 1000) < 1.5, (line, f.depths(pose))
    part = f.m.body(f.low_body).name.replace("_so101_v1", "").replace("_", " ")
    assert part in line
    st = r["state"]["checks"]
    assert st["command_filter"]["table"]["times"] >= 1 and st["command_filter"]["table"]["seconds"] > 0.1
    assert any("below the lowest height" in t for t in st["command_filter_recent"])
    assert not st["physics_violations"]
