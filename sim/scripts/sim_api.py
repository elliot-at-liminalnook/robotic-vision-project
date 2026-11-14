#!/usr/bin/env python3
"""REST API for driving the simulated arm, made for a language model (or any program) to use.

    python sim/scripts/sim_api.py                      # http://127.0.0.1:8765
    python sim/scripts/sim_api.py --bus --camera-fw    # same layers as teleop.py: --bus, --camera-fw, --cables
    python sim/scripts/sim_api.py --window             # also show it: the three camera views and the status panel,
                                                       # with actions played at real speed so they can be watched
    python sim/scripts/sim_api.py --hands-on           # the window also takes teleop's keys and mouse, so a person
                                                       # and the API's caller share the arm (time then runs between requests)
    curl http://127.0.0.1:8765/                        # a guide written for the model: read this first
    curl http://127.0.0.1:8765/state
    curl -X POST http://127.0.0.1:8765/move_tip -d '{"x": 0.24, "y": 0.06, "z": 0.09}'
    curl http://127.0.0.1:8765/camera/wrist -o wrist.jpg

How it behaves:
  * Turn-based. Simulated time only moves inside an action (move_tip, move_joints, gripper, wait). An
    action runs the simulation until it is finished or its timeout passes, then answers. Reading
    (state, camera) costs no time, so a slow caller changes nothing and a run is repeatable.
  * It drives the same controller as teleop.py (straight-line moves, torque governor, feed-forward,
    grip-by-offset, stall guard) through the same command filter and physics monitor, so what the
    caller can and cannot do to the arm is what a person at the keyboard can.
  * Every answer is JSON. A request that cannot be carried out is answered with ok = false and a
    sentence saying why (and what the limits are), not with a traceback.
  * GET /openapi.json describes every endpoint (OpenAPI 3), from the same table that validates requests.

It listens on 127.0.0.1 only unless --host says otherwise, has no authentication, and serves one
request at a time.
"""
from __future__ import annotations

import argparse
import time
import base64
import io
import json
import math
import sys
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import teleop as T  # noqa: E402
from hub.arm.actions import ArmActions  # noqa: E402
from hub.errors import Refused  # noqa: E402
from so101sim import JOINTS  # noqa: E402

FRAME = 1.0 / T.FRAME_HZ
RANDOM_R = (0.20, 0.31)          # /randomize: distance from the base [m] (where the arm reaches with fingers down, DECISIONS T7)
RANDOM_BEARING = math.radians(65)
RANDOM_APART = 0.075             # centres at least this far apart [m] (room for the jaws between them)
NUM = "number"


def _p(kind, description, required=False, **limits):
    return {"type": kind, "description": description, "required": required, **limits}


# One table: validates requests, and is the source of the guide and of /openapi.json.
ROUTES = [
    ("GET", "/", "This guide, as plain text.", {}),
    ("GET", "/openapi.json", "OpenAPI 3 description of every endpoint.", {}),
    ("GET", "/state", "Where everything is now. Costs no simulated time.", {}),
    ("GET", "/cameras", "The cameras and their image sizes.", {}),
    ("GET", "/camera/{name}", "The camera's current picture as a JPEG. Costs no simulated time.", {
        "base64": _p("boolean", "true: answer with JSON {media_type, data} (base64) instead of the image bytes"),
    }),
    ("GET", "/task", "The task set in this scene, and which of its goals hold right now. Costs no simulated time.", {}),
    ("GET", "/report", "Text reports of the physics monitor, the command filter and any optional layers.", {}),
    ("POST", "/reset", "Put the arm and the objects back to the start. Simulated time restarts at 0.", {}),
    ("POST", "/randomize", "Reset, then put every loose object somewhere else on the table: standing upright, turned by a "
                           "random angle, within the arm's reach and clear of everything else.", {
        "seed": _p(NUM, "a whole number: the same seed gives the same layout; omit for one nobody chose",
                   minimum=0, maximum=2 ** 31),
    }),
    ("POST", "/move_tip", "Move the gripper tip in a straight line to a point, fingers pointing down.", {
        "x": _p(NUM, "metres forward of the robot's base", True, minimum=-0.5, maximum=0.7),
        "y": _p(NUM, "metres to the robot's left", True, minimum=-0.6, maximum=0.6),
        "z": _p(NUM, "metres above the table top (the tip is never taken below 0.004)", True, minimum=-0.1, maximum=0.6),
        "roll": _p(NUM, "wrist roll in radians, which turns the jaws about the vertical; omit to keep it",
                   minimum=-2.3416, maximum=0.8),
        "square_jaws": _p("boolean", "true: also turn the wrist so the jaws close along the table's x or y axis "
                                     "(whichever is nearer), which is how to take hold of a box that sits square on the table"),
        "jaws_along_deg": _p(NUM, "instead of square_jaws: turn the wrist so the jaws close along this direction in the "
                                  "table's plane (0 = along x, 90 = along y), as near as the wrist's range allows",
                             minimum=0.0, maximum=180.0),
        "timeout_s": _p(NUM, "give up after this much simulated time", minimum=0.1, maximum=30, default=6.0),
        "tolerance_m": _p(NUM, "how close to the point counts as there", minimum=0.001, maximum=0.05, default=0.004),
    }),
    ("POST", "/move_joints", "Move joints to angles (radians). Joints not named keep their target.", {
        **{j: _p(NUM, f"{j} angle in radians") for j in JOINTS[:5]},
        "timeout_s": _p(NUM, "give up after this much simulated time", minimum=0.1, maximum=30, default=6.0),
    }),
    ("POST", "/gripper", "Open or close the gripper (give opening or gap_m). Closing on an object holds it gently.", {
        "opening": _p(NUM, "0 = closed, 1 = fully open", minimum=0.0, maximum=1.0),
        "gap_m": _p(NUM, "instead of opening: the distance between the jaw pads in metres", minimum=0.0, maximum=0.11),
        "timeout_s": _p(NUM, "give up after this much simulated time", minimum=0.1, maximum=10, default=2.5),
    }),
    ("POST", "/wait", "Let simulated time pass with the arm holding its target.", {
        "seconds": _p(NUM, "how long", True, minimum=0.0, maximum=30),
    }),
]

GUIDE = """SO-101 robot arm simulation: REST API
======================================
A 5-joint arm with a two-jaw gripper on a table, some loose objects and three cameras (wrist, scene_front,
scene_side). You control it with the endpoints below. All bodies and answers are JSON.
{scene}
Conventions
  Units: metres, radians, seconds, newton-metres.
  Frame: origin at the robot's base on the table top; x forward, y to the robot's left, z up. The table top is z = 0.
  The "tip" is the point midway between the two jaw pads, at the height of the jaw tips: put the tip
  on an object's centre and the object is between the jaws. gripper.gap_m is the distance between the pads.
  One jaw is fixed and one moves, so the tip shifts sideways (by half the change in gap) as the gripper opens
  and closes. move_tip puts the tip, as the jaws are at that moment, on the point. When the jaws close on an
  object they end up with it centred between them: after a grasp, the tip is on the object, and where you
  send the tip is where the object goes. gripper.holding = the jaws stopped on something before they shut;
  gap_m is then that thing's width.
  jaws_close_along_deg in /state is the direction the jaws close along (0 = along x, 90 = along y).
  The arm reaches roughly 0.10 to 0.35 m from its base. Fingers point straight down below 5 cm.
  To take hold of something: open the jaws 1 to 2 cm wider than it (not fully: the moving jaw then pushes
  it a long way before it is held), and go down until the tip is a few mm below its middle.
  The servos are soft: expect the arm a few millimetres from where you sent it, and check /state.

Time
{time}

Cameras (GET /cameras for their picture sizes and fields of view)
  scene_front stands beyond the far end of the table and looks back down at the robot: the robot's left (+y)
    is on the picture's right, and things nearer the robot are higher in the picture.
  scene_side stands off the table at the robot's right and looks across it: forward (+x) is to the picture's
    right, and the robot's left (+y) is away from the camera.
  wrist is on the gripper and moves and turns with it. It sits about 9 cm above the jaw tips and about 5 cm
    to one side of them, and looks along the fingers but tilted about 25 degrees away from them. With the
    fingers pointing down it sees the jaws at the top of its picture and the table under and beyond them; the
    spot under the tip is not at the picture's centre, and moves in the picture as the tip's height changes.
  The pictures show the arm too: it is yellow, with a camera holder at the wrist that is blue with a green
  circuit board and an orange cable, and it throws shadows on the table. Do not mistake those for the objects.
  The API gives no camera calibration beyond this: where a pixel is on the table is for you to work out.

POST /randomize puts the objects somewhere new.

GET /task says what is to be done in this scene and which of its goals hold now (some scenes have no task).

Answers
  Every answer has "ok". When ok is false, "error" says why in a sentence; nothing was changed unless it says so.
  An action's answer also has: reached (did it get there), elapsed_s, tip_error_m, message, and the state afterwards.
  When reached is false, why_not_reached says why in full: what stopped the arm, how far short it is in x, y and z,
  which joint is furthest from its target and how hard its servo is working.
  limited lists what the command filter did during the action. The filter sits between the controller and the
  servos and changes any command the arm must not or cannot follow. Each entry is a sentence with the numbers
  (which joint, how many degrees past which limit, which part of the arm how far below the allowed height). Kinds:
    jump              the command changed faster than a servo can turn, so it was spread over time. Normal
                      when a move starts or the jaws close on something. Harmless.
    joint_limit       a joint was asked past its range and was held at the limit. The sentence says whether the
                      pose itself was past the limit, or only the controller's correction for load (harmless,
                      but the joint may sag a little there).
    lead              the controller's correction for load and lag was capped. Harmless.
    table, base       the pose asked for would put a part of the arm into the table or into its own base, so the
                      command stopped short of it. Expect reached to be false.
    restart_from_arm  the arm had been held away from its command, so the command was restarted from where the arm is
    nonfinite         a command that was not a number was dropped
  message is the last short thing the controller said ("api: move_tip", "closing gripper", "blocked at <joint>:
  stopped pushing", "command filter: ...", "physics check: ..."); it may be from an earlier action. Prefer
  why_not_reached and limited.
  checks.command_filter in /state says how many times and for how long each kind has happened since the start,
  and checks.command_filter_recent has the last few as sentences (GET /report has more).
  checks.physics_violations should be empty. An entry means something happened that could not happen to a real
  arm (parts pushed into each other, a joint faster than its servo): what follows is not to be trusted.
  A move_tip that cannot be done is refused before anything moves (ok is false), whether it is the point itself
  or a point on the straight line to it that cannot be reached; the error says how near the tip can get, which
  joints are at their limits there, and what distances and heights can be reached instead.

A pick, step by step
{pick}

Endpoints
"""

PICK_KNOWN = """  1. GET /state                                              find the object (objects.<name>) and the tip
  2. POST /gripper  {"gap_m": 0.044}                         open 14 mm wider than a 30 mm cube
  3. POST /move_tip {x, y, "z": 0.09, "square_jaws": true}   above it, jaws square to it
  4. POST /move_tip {x, y, "z": 0.011}                       down around it (check tip_error_m)
  5. POST /gripper  {"opening": 0}                           close; gripper.holding should be true
  6. POST /move_tip {x, y, "z": 0.12}                        lift, then look at objects.<name>.z or a camera picture"""
PICK_UNKNOWN = """  This server does not say where the objects are (/state has no "objects"): find them in the camera pictures.
  1. GET /camera/scene_front, /camera/scene_side             work out where the object is on the table (x, y) and how it is turned
  2. POST /gripper  {"gap_m": 0.044}                         open 14 mm wider than a 30 mm cube
  3. POST /move_tip {x, y, "z": 0.09, "jaws_along_deg": a}   above it, jaws closing across two of its faces
  4. GET /camera/wrist                                       look again from close by, and correct x, y
  5. POST /move_tip {x, y, "z": 0.011}                       down around it (check tip_error_m)
  6. POST /gripper  {"opening": 0}                           close; gripper.holding should be true and gap_m its width
  7. POST /move_tip {x, y, "z": 0.12}                        lift, then check in a camera picture that it came along"""
TIME_TURNS = """  Simulated time moves only inside an action (move_tip, move_joints, gripper, wait). Each action runs
  until it is done or its timeout passes, then answers with what happened. Reading /state or a camera is free."""
TIME_SHARED = """  Each action (move_tip, move_joints, gripper, wait) runs until it is done or its timeout passes, then
  answers with what happened. This server also has a window in which a person can drive the arm, so the
  simulation runs at real speed between your requests as well: time_s goes up while you think, and the arm
  or the objects may have been moved by the person since your last request. Read /state before you act.
  The person can also switch the servos' torque off. While it is off the arm is limp and move_tip, move_joints
  and gripper are refused (ok is false, the error says so) until the person switches it back on or you
  POST /reset."""


def _colour_name(rgba) -> str:
    import colorsys
    h, sat, v = colorsys.rgb_to_hsv(*[float(c) for c in rgba[:3]])
    if sat < 0.2:
        return "white" if v > 0.8 else "grey" if v > 0.3 else "black"
    for limit, name in ((20, "red"), (45, "orange"), (70, "yellow"), (170, "green"), (260, "blue"), (320, "purple"), (361, "red")):
        if h * 360 < limit:
            return name
    return "red"


def describe_objects(api) -> list[str]:
    """One line per loose object: what it is, its colour and its size (what a person setting the task would know)."""
    import mujoco
    m, lines = api.inner.m, []
    for name in api.objects:
        g = int(m.body_geomadr[m.body(name).id])
        sz = [round(float(v) * 2000, 1) for v in m.geom_size[g]]
        colour = _colour_name(m.geom_rgba[g])
        if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER:
            what = f"{colour} cylinder, {sz[0]:g} mm across and {sz[1]:g} mm long"
        elif m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
            what = f"{colour} box, {sz[0]:g} x {sz[1]:g} mm and {sz[2]:g} mm high"
        else:
            what = f"{colour} object"
        lines.append(f"  {name:14s} {what}, {float(m.body_mass[m.body(name).id]) * 1000:.0f} g")
    return lines


ApiError = Refused        # a request that cannot be carried out (hub/errors.py); the old name is kept


def guide(api=None) -> str:
    """The guide for a caller. With the running simulation (api) it also lists this scene's objects and is
    worded for how this server was started (object positions given or not; a person at the window or not)."""
    scene = ""
    if api is not None:
        scene = "\nLoose objects in this scene (sizes as they stand at the start; /randomize stands them upright):\n" + "\n".join(describe_objects(api)) + "\n"
        if api.fixtures:
            scene += "Things that move on a slide or hinge (their positions are in /state under fixtures, when positions are given): " \
                     + ", ".join(api.fixtures) + "\n"
    privileged = api is None or api.privileged
    shared = api is not None and api.hands_on
    lines = [GUIDE.replace("{scene}", scene).replace("{time}", TIME_SHARED if shared else TIME_TURNS)
             .replace("{pick}", PICK_KNOWN if privileged else PICK_UNKNOWN).rstrip()]
    for method, path, summary, params in ROUTES:
        lines.append(f"  {method:4s} {path:16s} {summary}")
        for name, p in params.items():
            rng = (f", {p['minimum']} to {p['maximum']}" if "minimum" in p else "")
            dflt = f", default {p['default']}" if "default" in p else ""
            lines.append(f"         {name:12s} {p['type']}{', required' if p['required'] else ''}{rng}{dflt}: {p['description']}")
    return "\n".join(lines) + "\n"


def openapi() -> dict:
    paths: dict = {}
    for method, path, summary, params in ROUTES:
        op: dict = {"summary": summary, "responses": {"200": {"description": "JSON (or a JPEG for a camera)"},
                                                      "400": {"description": "ok = false, with the reason in error"}}}
        props = {n: {k: v for k, v in p.items() if k not in ("required",)} for n, p in params.items()}
        if method == "POST":
            op["requestBody"] = {"required": bool(params), "content": {"application/json": {"schema": {
                "type": "object", "additionalProperties": False, "properties": props,
                "required": [n for n, p in params.items() if p["required"]]}}}}
        else:
            op["parameters"] = [{"name": n, "in": "query", "schema": {k: v for k, v in s.items() if k != "description"},
                                 "description": s["description"]} for n, s in props.items()]
            if "{name}" in path:
                op["parameters"].append({"name": "name", "in": "path", "required": True, "schema": {"type": "string"},
                                         "description": "a camera name from /cameras"})
        paths.setdefault(path, {})[method.lower()] = op
    return {"openapi": "3.0.3", "info": {"title": "SO-101 simulation", "version": "1",
                                         "description": "Turn-based control of the simulated arm. GET / is a guide."},
            "paths": paths}


def validate(params: dict, body) -> dict:
    """The request's values with defaults filled in, or an ApiError that says what is wrong and what is allowed."""
    if not isinstance(body, dict):
        raise ApiError("the request body must be a JSON object, e.g. {\"x\": 0.2}")
    unknown = sorted(set(body) - set(params))
    if unknown:
        raise ApiError(f"unknown field(s) {unknown}; allowed here: {sorted(params) or 'none'}")
    out = {}
    for name, p in params.items():
        if name not in body or body[name] is None:
            if p["required"]:
                raise ApiError(f"'{name}' is required: {p['description']}")
            if "default" in p:
                out[name] = p["default"]
            continue
        v = body[name]
        if p["type"] == NUM:
            if isinstance(v, str):                       # (a GET's values arrive as text)
                try:
                    v = float(v)
                except ValueError:
                    pass
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise ApiError(f"'{name}' must be a finite number ({p['description']}), got {v!r}")
            if "minimum" in p and not p["minimum"] <= v <= p["maximum"]:
                raise ApiError(f"'{name}' = {v} is outside {p['minimum']} to {p['maximum']} ({p['description']})")
            out[name] = float(v)
        elif p["type"] == "string":
            out[name] = str(v)
        elif p["type"] == "boolean":
            if isinstance(v, str):
                v = v.lower() in ("1", "true", "yes")
            out[name] = bool(v)
    return out


class SimApi(ArmActions):
    """The simulation behind the API. Each method is one endpoint and returns a JSON-able dict.
    The arm's own actions (move_tip, move_joints, gripper, wait) are the hub's: hub/arm/actions.py."""

    def __init__(self, bus: dict | None = None, camera_fw: tuple | None = None, cables: bool = False,
                 privileged: bool = True, scene: str = "workcell"):
        """privileged: include the objects' true positions in /state (a real robot would not know them).
        scene: "workcell" or "bench" (sim/scene/); a scene may come with a task (so101sim/tasks.py)."""
        self.app = T.Teleop("mujoco", None, render_every=10 ** 9, bus=bus, camera_fw=camera_fw, cables=cables, scene=scene)
        self.privileged = privileged
        self.verbose = False
        self.hands_on = False            # a person can drive the arm from the window between requests (Window sets it)
        self.last_request = ""
        inner = getattr(self.app.sim, "inner", self.app.sim)
        m = inner.m
        import mujoco
        self.objects = [m.body(m.jnt_bodyid[j]).name for j in range(m.njnt) if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE]
        # Things in the scene that move on a joint of their own (a drawer): name -> where its position is stored.
        self.fixtures = {m.joint(j).name: int(m.jnt_qposadr[j]) for j in range(m.njnt)
                         if m.jnt_type[j] in (mujoco.mjtJoint.mjJNT_SLIDE, mujoco.mjtJoint.mjJNT_HINGE)
                         and m.joint(j).name not in JOINTS and not m.joint(j).name.startswith("cable_")}
        from so101sim.tasks import TASKS
        self.inner = inner
        self.task = TASKS[scene](m) if scene in TASKS else None
        t = m.geom("table").id
        ArmActions.__init__(self, self.app)
        self.table = {"x": [float(m.geom_pos[t][0] - m.geom_size[t][0]), float(m.geom_pos[t][0] + m.geom_size[t][0])],
                      "y": [float(m.geom_pos[t][1] - m.geom_size[t][1]), float(m.geom_pos[t][1] + m.geom_size[t][1])], "z": 0.0}

    # ---- helpers ---------------------------------------------------------------
    # ---- endpoints ---------------------------------------------------------------
    def state(self) -> dict:
        out = super().state()
        app, sim = self.app, self.app.sim
        out["table_m"] = self.table
        out["checks"]["physics_violations"] = list(app.monitor.violations[-5:]) if app.monitor else []
        if self.privileged:
            out["objects"] = {n: {k: round(float(v), 4) for k, v in zip("xyz", sim.body_pos(n))} for n in self.objects}
            if self.fixtures:
                out["fixtures"] = {n: round(float(self.inner.d.qpos[a]), 4) for n, a in self.fixtures.items()}
        return out

    def task_status(self) -> dict:
        if self.task is None:
            return {"ok": True, "task": None, "description": "No task is set in this scene. Do what you are asked.", "goals": [], "done": False}
        goals = self.task.check(self.inner.d)
        return {"ok": True, "task": self.task.name, "description": self.task.description, "goals": goals,
                "done": all(g["done"] for g in goals)}

    def reset(self) -> dict:
        self.app.reset_scene()
        self._goal_point = None
        if self.app.monitor is not None:
            self.app.monitor.violations.clear()
            self.app.monitor.kinds.clear()
            self.app.monitor.reset()
        return self.state()

    def randomize(self, seed=None) -> dict:
        """Reset, then scatter the loose objects (see ROUTES). The answer names the seed, not where anything went."""
        import mujoco
        import os
        seed = int.from_bytes(os.urandom(4), "little") >> 1 if seed is None else int(seed)
        rng = np.random.default_rng(seed)
        self.reset()
        m, d = self.inner.m, self.inner.d
        table = m.geom("table").id
        parts = []
        for name in self.objects:
            body = m.body(name).id
            g = int(m.body_geomadr[body])
            half = float(m.geom_size[g][1] if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER else m.geom_size[g][2])
            parts.append((body, int(m.jnt_qposadr[m.body_jntadr[body]]), half))
        bodies = {b for b, _, _ in parts}
        for _ in range(2000):
            spots = []
            for body, adr, half in parts:
                r, a, yaw = rng.uniform(RANDOM_R[0], RANDOM_R[1]), rng.uniform(-RANDOM_BEARING, RANDOM_BEARING), rng.uniform(0, 2 * math.pi)
                spots.append((r * math.cos(a), r * math.sin(a)))
                d.qpos[adr:adr + 7] = [spots[-1][0], spots[-1][1], half + 0.0005, math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
            if any(math.dist(a, b) < RANDOM_APART for i, a in enumerate(spots) for b in spots[:i]):
                continue
            mujoco.mj_forward(m, d)
            touching = [(int(c.geom1), int(c.geom2)) for c in d.contact[:d.ncon] if c.dist < 0.002
                        and (int(m.geom_bodyid[c.geom1]) in bodies or int(m.geom_bodyid[c.geom2]) in bodies)]
            if all(table in pair for pair in touching):          # each object touches the table and nothing else
                break
        else:
            raise ApiError("could not find a free place for every object", 500)
        d.qvel[:] = 0
        for _ in range(10):                                      # let them settle (the arm holds still)
            self._frame()
        return {**self.state(), "seed": seed}

    def cameras(self) -> dict:
        cams = self.app.sim.cameras
        return {"ok": True, "cameras": {n: {"width": c.render_width, "height": c.render_height,
                                            "mounted_on": c.parent_body, "vertical_fov_deg": round(c.fovy_deg, 1)}
                                        for n, c in cams.items()}}

    def camera_jpeg(self, name: str) -> bytes:
        from PIL import Image
        if name not in self.app.sim.cameras:
            raise ApiError(f"no camera '{name}'; there are {sorted(self.app.sim.cameras)}", 404)
        self.app.refresh_images()
        out = io.BytesIO()
        Image.fromarray(self.app.images[name]).save(out, "JPEG", quality=85)
        return out.getvalue()

    def report(self) -> dict:
        app, sim = self.app, self.app.sim
        parts = []
        if app.monitor is not None:
            parts.append(app.monitor.report())
        if getattr(sim, "safety", None) is not None:
            parts.append(sim.safety.report())
        if hasattr(sim, "report"):
            parts.append(sim.report())
        inner = getattr(sim, "inner", sim)
        if getattr(inner, "cables", None) is not None:
            parts.append(inner.cables.report(inner.d))
        if app.fwcam is not None:
            parts.append(app.fwcam.report(log_lines=0))
        return {"ok": True, "report": "\n".join(parts)}


def dispatch(api: SimApi, method: str, url: str, body_bytes: bytes = b"") -> tuple[int, str, bytes]:
    """One request -> (status, content type, body). Never raises."""
    try:
        parsed = urlparse(url)
        path = parsed.path.rstrip("/") or "/"
        query = {k: v[-1] for k, v in parse_qs(parsed.query).items()}
        cam = path[len("/camera/"):] if path.startswith("/camera/") else None
        key = "/camera/{name}" if cam is not None else path
        route = next((r for r in ROUTES if r[1] == key), None)
        if route is None:
            raise ApiError(f"no such endpoint {path}; GET / lists them", 404)
        if route[0] != method:
            raise ApiError(f"{path} takes {route[0]}, not {method}", 405)
        if method == "POST":
            api.last_request = f"POST {path} {body_bytes.decode(errors='replace')[:90]}"
            try:
                body = json.loads(body_bytes.decode() or "{}")
            except (ValueError, UnicodeDecodeError):
                raise ApiError("the request body is not valid JSON") from None
            args = validate(route[3], body)
        else:
            args = validate(route[3], query)
        if key == "/":
            return 200, "text/plain; charset=utf-8", guide(api).encode()
        if key == "/openapi.json":
            result = openapi()
        elif cam is not None:
            jpeg = api.camera_jpeg(cam)
            if not args.get("base64"):
                return 200, "image/jpeg", jpeg
            result = {"ok": True, "media_type": "image/jpeg", "data": base64.b64encode(jpeg).decode()}
        elif key == "/task":
            result = api.task_status()
        else:
            result = getattr(api, key[1:])(**args)
        return 200, "application/json", json.dumps(result).encode()
    except ApiError as e:
        return e.status, "application/json", json.dumps({"ok": False, "error": str(e)}).encode()
    except Exception as e:  # noqa: BLE001  (a bug here must reach the caller as JSON, not kill the server)
        return 500, "application/json", json.dumps({"ok": False, "error": f"internal error: {type(e).__name__}: {e}"}).encode()


def make_handler(api: SimApi):
    class Handler(BaseHTTPRequestHandler):
        def _answer(self, method: str):
            n = int(self.headers.get("Content-Length") or 0)
            status, ctype, body = dispatch(api, method, self.path, self.rfile.read(n) if 0 < n <= 1_000_000 else b"")
            self.send_response(status)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):       # noqa: N802
            self._answer("GET")

        def do_POST(self):      # noqa: N802
            self._answer("POST")

        def log_message(self, fmt, *args):
            if api.verbose:
                sys.stderr.write("%s  %s\n" % (self.log_date_time_string(), fmt % args))
    return Handler


class Window:
    """Teleop's window (wrist, scene_front, scene_side, status panel), watching a simulation driven over HTTP.

    Drawn after every simulated frame and paced at real speed, so an action can be watched as it happens.
    Closing it stops the server. Keys do nothing: the arm is the API's.

    hands_on: between requests the window is teleop (same keys and mouse, same controller) and the
    simulation runs at real speed, so a person and the API's caller take turns at the one arm. While a
    request's action runs, the keys are ignored (Esc and closing the window still work); the action
    starts from wherever the person left the arm. Simulated time then also passes between requests.
    """

    def __init__(self, api: SimApi, hands_on: bool = False):
        self.hands_on = api.hands_on = hands_on
        import pygame as pg
        self.pg, self.api = pg, api
        pg.init()
        self.screen = pg.display.set_mode((2 * T.VIEW_W, 2 * (T.VIEW_H + T.PANEL)))
        pg.display.set_caption("SO-101 simulation, driven over the REST API" + (" and from this window" if hands_on else ""))
        self.font, self.small = pg.font.SysFont(None, 26), pg.font.SysFont(None, 20)
        self.clock = pg.time.Clock()
        self.open = True
        self._last = time.perf_counter()
        api.on_frame = self.frame

    def frame(self, idle: bool = False):
        """idle: no request is being served (the main loop's call), so in hands-on mode the person has the arm."""
        pg, app = self.pg, self.api.app
        mine = self.hands_on and idle
        for e in pg.event.get():
            if e.type == pg.QUIT or (e.type == pg.KEYDOWN and e.key == pg.K_ESCAPE):
                self.open = False
            elif mine:
                app.handle_event(e, pg)
        now = time.perf_counter()
        if mine:                         # by the wall clock, so the arm moves at real speed however long a frame takes
            dt = min(now - self._last, 0.1)
            app.handle_held_keys(pg.key.get_pressed(), dt, pg)
            app.physics(dt)
        self._last = now
        app.refresh_images()
        app.draw(self.screen, self.font, self.small, pg)
        if self.api.last_request:
            text = self.small.render("last request: " + self.api.last_request, True, (140, 220, 255))
            self.screen.blit(text, (T.VIEW_W + 12, 2 * (T.VIEW_H + T.PANEL) - 58))
        pg.display.flip()
        self.clock.tick(T.FRAME_HZ)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1", help="address to listen on (default: this machine only)")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--bus", action="store_true", help="servo bus in the loop, as in teleop.py")
    ap.add_argument("--camera-fw", nargs="?", const="640x480", default=None, metavar="WxH",
                    help="wrist camera through the camera firmware, as in teleop.py")
    ap.add_argument("--cables", action="store_true", help="the arm's wires in the model, as in teleop.py")
    ap.add_argument("--scene", default="workcell",
                    help="workcell (default: table and three objects) or bench (a drawer, bins, a platform, a peg and "
                         "socket, and a task: GET /task)")
    ap.add_argument("--no-object-positions", action="store_true",
                    help="leave the objects' true positions out of /state (the caller must use the cameras)")
    ap.add_argument("--window", action="store_true",
                    help="show the camera views and the status panel, and play actions at real speed")
    ap.add_argument("--hands-on", action="store_true",
                    help="--window, and the window takes teleop's keys and mouse between requests: a person and the "
                         "API's caller share the arm, and simulated time also runs between requests")
    ap.add_argument("--quiet", action="store_true", help="don't log each request")
    a = ap.parse_args()
    a.window = a.window or a.hands_on
    mode = None
    if a.camera_fw:
        try:
            mode = tuple(int(v) for v in a.camera_fw.lower().split("x"))
            assert len(mode) == 2
        except (ValueError, AssertionError):
            ap.error("--camera-fw takes a size like 640x480")
    try:
        api = SimApi(bus={} if a.bus else None, camera_fw=mode, cables=a.cables, privileged=not a.no_object_positions,
                     scene=a.scene)
    except ValueError as e:
        ap.error(str(e))
    api.verbose = not a.quiet
    T.exit_on_sigterm()
    server = HTTPServer((a.host, a.port), make_handler(api))     # one request at a time, all on this thread
    print(f"simulation API on http://{a.host}:{server.server_address[1]}/   (GET / for the guide; Ctrl-C to stop)", flush=True)
    try:
        if a.window:
            window = Window(api, hands_on=a.hands_on)
            server.timeout = 0.002 if a.hands_on else 0.03
            while window.open:               # between requests the window keeps drawing
                server.handle_request()
                window.frame(idle=True)
        else:
            server.serve_forever()
    except KeyboardInterrupt:
        print()
    finally:
        server.server_close()
        print("stop path:", api.app.shutdown())
        print(api.report()["report"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
