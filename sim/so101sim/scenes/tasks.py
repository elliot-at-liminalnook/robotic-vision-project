"""Tasks as data: a request in words and goals the referee can check.

A task lives in its scene's file:

    [[tasks]]
    name = "fetch"
    request = "Put the red cube in the tray."      # all the model is ever given
    needs = ["arm", "rover"]                        # kinds of device the task cannot be done without
    [[tasks.goals]]
    kind = "inside"
    object = "red_cube"
    region = "tray"

Goals are about the state at one moment, so they can be checked at any time and in any order;
a task is done when all hold at once. Two families:

  arrangement   inside, on, upright, fixture_at          checked on the simulation's true state
  answer        answer_number, answer_text, answer_list, answer_pose
                                                          checked on what was handed in to the
                                                          operator API under the goal's `key`,
                                                          against a truth worked out from the scene

Truth expressions for answers (the `truth` field):
  surface_area_m2 | surface_length_x_m | surface_length_y_m
  fixture_top_m:<fixture>        height of a fixture's top face above the surface
  drawer_open_m:<fixture>        how far a cabinet's drawer is out
  open:<fixture>                 "yes" if its drawer is more than 20 mm out, else "no"
  count_objects | count_in:<region>
  colours_in:<region> | colours_not_seen_by:<camera>      (lists of colour names)
  is:<text>                      a fixed answer
answer_pose compares where device `of` is, in the frame of device `relative_to`.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

from . import geometry as G
from .build import Built

SLOW_M_S = 0.01
OPEN_M = 0.02


def _body_speed(built: Built, d, body: str) -> float:
    m = built.model
    b = m.body(body)
    if b.jntnum[0] == 0:
        return 0.0
    adr = m.jnt_dofadr[b.jntadr[0]]
    return float(np.linalg.norm(d.qvel[adr:adr + 3]))


def _colour_name(built: Built, obj: str) -> str:
    o = next(o for o in built.scene.get("objects", []) if o["name"] == obj)
    return o["colour"] if isinstance(o["colour"], str) else "other"


def device_frame(built: Built, d, name: str):
    """(position, yaw or None) of a device in the scene frame, from the true state."""
    dev = next((x for x in built.devices if x["name"] == name), None)
    if dev is None:
        raise KeyError(name)
    m = built.model
    if dev["kind"] == "arm":
        b = m.body("base").id
        R = d.xmat[b].reshape(3, 3)
        return d.xpos[b] - [0, 0, d.xpos[b][2]], math.atan2(R[1, 0], R[0, 0])
    if dev["kind"] == "rover":
        b = m.body(dev["body"]).id
        R = d.xmat[b].reshape(3, 3)
        p = d.xpos[b].copy()
        p[2] = 0.0
        return p, math.atan2(R[1, 0], R[0, 0])
    c = m.camera(name).id
    return d.cam_xpos[c].copy(), None


def truth(built: Built, d, expr: str):
    """The true value of a truth expression (see the module docstring)."""
    m = built.model
    head, _, arg = expr.partition(":")
    out = np.asarray(built.outline)
    if head == "surface_area_m2":
        return G.area(built.outline)
    if head == "surface_length_x_m":
        return float(np.ptp(out[:, 0]))
    if head == "surface_length_y_m":
        return float(np.ptp(out[:, 1]))
    if head == "fixture_top_m":
        return float(built.fixtures[arg]["top_z"])
    if head == "drawer_open_m":
        return float(d.qpos[m.joint(built.fixtures[arg]["joint"]).qposadr[0]])
    if head == "open":
        return "yes" if truth(built, d, f"drawer_open_m:{arg}") > OPEN_M else "no"
    if head == "count_objects":
        return len(built.objects)
    if head == "count_in":
        return sum(built.regions[arg].holds(m, d, d.xpos[m.body(o).id]) for o in built.objects)
    if head == "colours_in":
        return sorted(_colour_name(built, o) for o in built.objects if built.regions[arg].holds(m, d, d.xpos[m.body(o).id]))
    if head == "is":
        return arg
    raise KeyError(f"unknown truth expression '{expr}'")


def check_goal(built: Built, d, goal: dict, answers: dict | None = None) -> tuple[bool, str]:
    """(does the goal hold now, a few words on what is the case)."""
    m, kind = built.model, goal["kind"]
    answers = answers or {}
    if kind == "inside":
        p = d.xpos[m.body(goal["object"]).id]
        ok = built.regions[goal["region"]].holds(m, d, p) and _body_speed(built, d, goal["object"]) < SLOW_M_S
        return ok, f"{goal['object']} at ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})"
    if kind == "on":
        p = d.xpos[m.body(goal["object"]).id]
        target = goal["on"]
        if target in built.fixtures:
            region = built.regions.get(f"{target}_top") or built.regions.get(target)
            ok = region is not None and region.holds(m, d, p)
        else:                                               # on another loose thing: above it and touching it
            q = d.xpos[m.body(target).id]
            touching = any({m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]} == {m.body(goal["object"]).id, m.body(target).id}
                           for c in d.contact[:d.ncon])
            ok = touching and p[2] > q[2] + 0.003 and np.linalg.norm(p[:2] - q[:2]) < 0.03
        return bool(ok and _body_speed(built, d, goal["object"]) < SLOW_M_S), f"{goal['object']} at z = {p[2]:.3f}"
    if kind == "upright":
        up = abs(d.xmat[m.body(goal["object"]).id].reshape(3, 3)[2, 2])
        return bool(up > 0.95), f"{goal['object']} tilted {math.degrees(math.acos(min(1.0, up))):.0f} deg"
    if kind == "fixture_at":
        v = truth(built, d, f"drawer_open_m:{goal['fixture']}")
        return bool(goal.get("min_m", -1e9) <= v <= goal.get("max_m", 1e9)), f"{goal['fixture']} is {v * 1000:.0f} mm out"
    key = goal["key"]
    if key not in answers:
        return False, f"no answer handed in for '{key}'"
    got = answers[key]
    if kind == "answer_number":
        want = float(truth(built, d, goal["truth"]))
        try:
            got = float(got)
        except (TypeError, ValueError):
            return False, f"'{key}' is not a number"
        tol = max(goal.get("tolerance", 0.0), goal.get("relative_tolerance", 0.0) * abs(want))
        return bool(abs(got - want) <= tol), f"answer {got:g}, off by {abs(got - want):g}, allowed {tol:g}"
    if kind == "answer_text":
        want = str(truth(built, d, goal["truth"]))
        return str(got).strip().lower() == want.lower(), "right" if str(got).strip().lower() == want.lower() else "wrong"
    if kind == "answer_list":
        want = truth(built, d, goal["truth"])
        ok = isinstance(got, list) and sorted(str(g).strip().lower() for g in got) == sorted(want)
        return ok, "right" if ok else f"wrong ({len(want)} expected)"
    if kind == "answer_pose":
        try:
            p_of, yaw_of = device_frame(built, d, goal["of"])
            p_ref, yaw_ref = device_frame(built, d, goal["relative_to"])
            gx, gy = float(got["x"]), float(got["y"])
        except (KeyError, TypeError, ValueError):
            return False, f"'{key}' needs x and y in metres (and yaw_deg)"
        c, s = math.cos(-(yaw_ref or 0.0)), math.sin(-(yaw_ref or 0.0))
        rel = np.array([c * (p_of[0] - p_ref[0]) - s * (p_of[1] - p_ref[1]), s * (p_of[0] - p_ref[0]) + c * (p_of[1] - p_ref[1])])
        err = float(np.linalg.norm(rel - [gx, gy]))
        ok = err <= goal.get("tolerance", 0.02)
        detail = f"position off by {err * 1000:.0f} mm"
        if ok and yaw_of is not None and yaw_ref is not None and "tolerance_deg" in goal and "yaw_deg" in got:
            want = math.degrees(yaw_of - yaw_ref)
            dyaw = abs((float(got["yaw_deg"]) - want + 180) % 360 - 180)
            ok = dyaw <= goal["tolerance_deg"]
            detail += f", heading off by {dyaw:.0f} deg"
        return bool(ok), detail
    raise KeyError(kind)


def check_task(built: Built, d, task: dict, answers: dict | None = None) -> list[dict]:
    out = []
    for g in task["goals"]:
        ok, detail = check_goal(built, d, g, answers)
        out.append({"goal": g, "done": bool(ok), "detail": detail})
    return out


def static_problems(built: Built, task: dict) -> list[str]:
    """What is wrong with a task before anything moves: names that do not exist, goals that could never hold."""
    out, t = [], f"tasks '{task['name']}'"
    kinds = {d["kind"] for d in built.devices}
    for k in task.get("needs", []):
        if k not in kinds:
            out.append(f"{t}: needs a {k}, and this scene has none")
    need = {"inside": ("object", "region"), "on": ("object", "on"), "upright": ("object",), "fixture_at": ("fixture",),
            "answer_number": ("key", "truth"), "answer_text": ("key", "truth"), "answer_list": ("key", "truth"),
            "answer_pose": ("key", "of", "relative_to")}
    d = built.new_data()
    for i, g in enumerate(task["goals"]):
        where = f"{t} goal {i + 1} ({g['kind']})"
        missing = [k for k in need[g["kind"]] if k not in g]
        if missing:
            out.append(f"{where}: needs {', '.join(missing)}")
            continue
        if "object" in g and g["object"] not in built.objects:
            out.append(f"{where}: there is no loose object '{g['object']}'")
            continue
        if g["kind"] == "inside":
            if g["region"] not in built.regions:
                out.append(f"{where}: there is no region '{g['region']}'; there are {sorted(built.regions)}")
                continue
            o = next(o for o in built.scene["objects"] if o["name"] == g["object"])
            foot = min(o["size_m"][:2]) if o["shape"] == "box" else o["size_m"][0]
            room = built.regions[g["region"]].inner_size
            if foot > min(room[0], room[1]) + 1e-9:
                out.append(f"{where}: {g['object']} ({foot * 1000:.0f} mm across) does not fit in {g['region']} "
                           f"({min(room[0], room[1]) * 1000:.0f} mm)")
        elif g["kind"] == "on" and g["on"] not in built.fixtures and g["on"] not in built.objects:
            out.append(f"{where}: there is nothing called '{g['on']}' to put it on")
        elif g["kind"] == "fixture_at" and "joint" not in built.fixtures.get(g["fixture"], {}):
            out.append(f"{where}: '{g['fixture']}' is not a fixture with a moving part")
        elif g["kind"].startswith("answer") and g["kind"] != "answer_pose":
            try:
                truth(built, d, g["truth"])
            except KeyError as e:
                out.append(f"{where}: truth '{g['truth']}' cannot be worked out ({e})")
        elif g["kind"] == "answer_pose":
            names = {x["name"] for x in built.devices}
            for k in ("of", "relative_to"):
                if g[k] not in names:
                    out.append(f"{where}: there is no device '{g[k]}'")
    return out
