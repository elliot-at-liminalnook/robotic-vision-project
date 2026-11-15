"""Is a scene file a place that can exist? Checked without running any device.

    problems = validate(scene)        # [] means valid

In order, stopping at the first stage that fails (later stages need the earlier ones):
  1. fields    the file has the required fields, no unknown ones, numbers in range   (schema.check)
  2. build     the model compiles                                                     (build.build)
  3. placing   everything that stands on the surface is inside the lip, and nothing overlaps
  4. rest      left alone for `settle_s` of physics nothing moves (the arm is held by its own
               servos at its starting pose; no controller runs)
  5. tasks     every task names things that exist, starts not done, and its goals could hold

That a task can actually be done is a stronger claim; it waits for a reference solution.
"""
from __future__ import annotations

import cv2
import mujoco
import numpy as np

from . import geometry as G
from . import schema, tasks
from .build import Built, SceneError, build

UNDER_HEAD_M = 0.015         # a loose thing's centre this near the parked rover's outline, seen from above, is under its head
OVERLAP_M = 0.0005          # deeper than this at the start is an overlap, not a resting contact
MOVED_M = 0.002             # a thing that ends this far from where it started was not at rest
INSIDE_MARGIN_M = 0.0       # a footprint may touch the lip's inner face


def _root(m, body: int) -> int:
    while m.body_parentid[body] != 0:
        body = m.body_parentid[body]
    return body


def _corners(m, d, g: int) -> np.ndarray:
    c, h = m.geom_aabb[g, :3], m.geom_aabb[g, 3:]
    local = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h + c
    return local @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]


def placing_problems(built: Built, d) -> list[str]:
    m, out = built.model, []
    standing = {o: "objects" for o in built.objects}
    standing.update({f: "fixtures" for f in built.fixtures})
    for dev in built.devices:
        if dev["kind"] == "arm":
            standing["base"] = "devices"
        elif dev["kind"] == "rover":
            standing[dev["body"]] = "devices"
    for name, group in standing.items():
        root = m.body(name).id
        worst = 0.0
        for g in range(m.ngeom):
            if _root(m, m.geom_bodyid[g]) != root or (m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0):
                continue
            for p in _corners(m, d, g):
                if not G.inside(built.outline, p[0], p[1]):
                    worst = max(worst, G.distance_to_edge(built.outline, p[0], p[1]))
        if worst > INSIDE_MARGIN_M + 1e-6:
            label = "the arm's base" if name == "base" else f"{group} '{name}'"
            out.append(f"{label}: reaches {worst * 1000:.0f} mm past the edge of the surface")
    arm_root = m.body("base").id if any(dv["kind"] == "arm" for dv in built.devices) else -1
    seen = set()
    for c in d.contact[:d.ncon]:
        if c.dist >= -OVERLAP_M:
            continue
        b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
        r1, r2 = (_root(m, b) if b else 0 for b in (b1, b2))
        if r1 == r2 == arm_root and arm_root > 0:
            continue                                    # the arm against itself is the arm model's business
        pair = tuple(sorted((m.geom(c.geom1).name or m.body(b1).name, m.geom(c.geom2).name or m.body(b2).name)))
        if pair not in seen:
            seen.add(pair)
            out.append(f"{pair[0]} and {pair[1]} overlap by {-c.dist * 1000:.1f} mm at the start")
    # Nothing loose stands under the rover's parked head. The head reaches 0.25 m ahead of the axle and 0.12 m to each
    # side, a few centimetres above the surface: a thing under it touches nothing, so the overlap check passes, but the
    # rover cannot drive off without pushing it and the arm cannot pick it without meeting the head.
    for dev in built.devices:
        if dev["kind"] != "rover":
            continue
        rb = m.body(dev["body"]).id
        pts = np.vstack([_corners(m, d, g) for g in range(m.ngeom) if m.geom_contype[g] and _root(m, m.geom_bodyid[g]) == rb])
        hull = cv2.convexHull(pts[:, :2].astype(np.float32)).reshape(-1, 1, 2)
        for o in built.objects:
            x, y = (float(v) for v in d.xpos[m.body(o).id][:2])
            if cv2.pointPolygonTest(hull, (x, y), True) > -UNDER_HEAD_M:
                out.append(f"objects '{o}': under the parked rover's head, or within {UNDER_HEAD_M * 1000:.0f} mm of its outline "
                           "(the head reaches 0.25 m ahead of the rover and 0.12 m to each side)")
    return out


def rest_problems(built: Built, d, settle_s: float) -> list[str]:
    m, out = built.model, []
    watch = list(built.objects) + [dv["body"] for dv in built.devices if dv["kind"] == "rover"]
    before = {b: d.xpos[m.body(b).id].copy() for b in watch}
    joints = {f["joint"]: d.qpos[m.joint(f["joint"]).qposadr[0]] for f in built.fixtures.values() if "joint" in f}
    for _ in range(int(round(settle_s / m.opt.timestep))):
        mujoco.mj_step(m, d)
        if not np.isfinite(d.qpos).all():
            return ["the simulation blew up while the scene was left alone"]
    for b, p0 in before.items():
        moved = float(np.linalg.norm(d.xpos[m.body(b).id] - p0))
        if moved > MOVED_M:
            out.append(f"'{b}' is not at rest: it moved {moved * 1000:.0f} mm in {settle_s:g} s left alone")
    for j, q0 in joints.items():
        moved = abs(float(d.qpos[m.joint(j).qposadr[0]]) - q0)
        if moved > MOVED_M:
            out.append(f"'{j}' is not at rest: it moved {moved * 1000:.0f} mm in {settle_s:g} s left alone")
    return out


def task_problems(built: Built, d, others=None) -> list[str]:
    """others: name -> scene, for tasks that swap to another scene (default: the files in sim/scenes/)."""
    out = []
    for t in built.scene.get("tasks", []):
        where = built
        if "swap_to" in t:                              # the goals are about the scene swapped to
            try:
                if others is not None:
                    other = others[t["swap_to"]]
                else:
                    from . import read
                    other = read(t["swap_to"])
                if schema.check(other):
                    raise SceneError("it is not a valid scene")
                where = build(other)
            except (KeyError, FileNotFoundError, SceneError) as e:
                out.append(f"tasks '{t['name']}': cannot swap to scene '{t['swap_to']}' ({e})")
                continue
        static = tasks.static_problems(where, t)
        out += static
        if static:
            continue
        if where is not built:
            continue
        results = tasks.check_task(built, d, t, answers={})
        if results and all(r["done"] for r in results):
            out.append(f"tasks '{t['name']}': every goal already holds at the start")
    return out


def validate(scene, settle_s: float = 1.0) -> list[str]:
    """Every problem found, as sentences. [] means the scene is valid."""
    problems = schema.check(scene)
    if problems:
        return problems
    try:
        built = build(scene)
    except SceneError as e:
        return [str(e)]
    d = built.new_data()
    problems = placing_problems(built, d)
    if problems:
        return problems
    problems = rest_problems(built, d, settle_s)
    if problems:
        return problems
    return task_problems(built, d)
