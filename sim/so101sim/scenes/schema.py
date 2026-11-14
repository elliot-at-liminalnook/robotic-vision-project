"""What a scene file may hold, and the check that a file holds only that.

A scene is everything about a place and nothing about how to work in it
(docs/MULTI_DEVICE_PLAN.md, 5.7). It is ground truth: the hub never reads one.

The schema is data: field -> (type, required, limits). `check(scene)` returns a list of
sentences, one per problem, each starting with the path of the field it is about.
"""
from __future__ import annotations

import math

NUM = (int, float)
COLOURS = {
    "red": (0.85, 0.2, 0.2), "green": (0.2, 0.7, 0.3), "blue": (0.2, 0.4, 0.85), "yellow": (0.95, 0.85, 0.15),
    "purple": (0.6, 0.3, 0.8), "orange": (0.95, 0.55, 0.15), "white": (0.92, 0.92, 0.9), "grey": (0.5, 0.5, 0.55),
    "black": (0.12, 0.12, 0.14), "wood": (0.72, 0.6, 0.45), "cream": (0.8, 0.78, 0.7), "steel": (0.45, 0.47, 0.52),
}
FIXTURE_KINDS = ("box", "bin", "socket", "cabinet", "shelf", "parts")
OBJECT_SHAPES = ("box", "cylinder")
DEVICE_KINDS = ("arm", "camera", "rover")
GOAL_KINDS = ("inside", "on", "upright", "fixture_at", "answer_number", "answer_text", "answer_list", "answer_pose")


def F(kind, required=False, lo=None, hi=None, n=None, choices=None, of=None):
    """A field: kind is a type, 'vec' (list of numbers, length n), 'colour', 'list' (of tables `of`) or a dict (a table)."""
    return {"kind": kind, "required": required, "lo": lo, "hi": hi, "n": n, "choices": choices, "of": of}


PART = {"name": F(str), "box_m": F("vec", True, 0.0005, 2.0, n=3), "at_m": F("vec", True, -2, 2, n=3), "colour": F("colour"),
        "mass_kg": F(NUM, lo=0.0005, hi=5.0), "friction": F(NUM, lo=0.0, hi=3.0)}
FIXTURE = {
    "kind": F(str, True, choices=FIXTURE_KINDS), "name": F(str, True), "at_m": F("vec", True, -3, 3, n=(2, 3)),
    "yaw_deg": F(NUM, lo=-360, hi=360), "colour": F("colour"),
    "size_m": F("vec", lo=0.001, hi=2.0, n=3),                      # box, socket (outer), shelf (top: x, y, thickness)
    "inner_m": F("vec", lo=0.005, hi=1.0, n=(2, 3)),                # bin (x, y), cabinet (depth, width, height)
    "wall_m": F(NUM, lo=0.001, hi=0.05), "wall_height_m": F(NUM, lo=0.002, hi=0.3),
    "hole_m": F(NUM, lo=0.002, hi=0.5),                             # socket: side of its square hole
    "height_m": F(NUM, lo=0.01, hi=0.6),                            # shelf: height of its top face
    "support": F(str, choices=("legs", "back")),                    # shelf
    "drawer_travel_m": F(NUM, lo=0.0, hi=0.3), "drawer_open_m": F(NUM, lo=0.0, hi=0.3),   # cabinet
    "parts": F("list", of=PART),
}
OBJECT = {
    "name": F(str, True), "shape": F(str, True, choices=OBJECT_SHAPES), "size_m": F("vec", True, 0.002, 0.5, n=(2, 3)),
    "mass_kg": F(NUM, True, 0.0005, 2.0), "colour": F("colour", True), "at_m": F("vec", lo=-3, hi=3, n=(2, 3)),
    "on": F(str), "yaw_deg": F(NUM, lo=-360, hi=360), "lying": F(bool), "friction": F(NUM, lo=0.0, hi=3.0),
}
DEVICE = {
    "kind": F(str, True, choices=DEVICE_KINDS), "name": F(str, True), "at_m": F("vec", True, -3, 3, n=(2, 3)),
    "facing_deg": F(NUM, lo=-360, hi=360), "looking_at_m": F("vec", lo=-3, hi=3, n=3),
    "joints_rad": F("vec", lo=-3.2, hi=3.2, n=6), "post": F(bool),
}
REGION = {"name": F(str, True), "rect_m": F("vec", True, -3, 3, n=4), "z_m": F("vec", lo=-1, hi=2, n=2)}
GOAL = {
    "kind": F(str, True, choices=GOAL_KINDS), "object": F(str), "region": F(str), "on": F(str), "fixture": F(str),
    "min_m": F(NUM, lo=-1, hi=1), "max_m": F(NUM, lo=-1, hi=1),
    "key": F(str), "truth": F(str), "tolerance": F(NUM, lo=0.0, hi=10.0), "relative_tolerance": F(NUM, lo=0.0, hi=1.0),
    "tolerance_deg": F(NUM, lo=0.0, hi=180.0), "of": F(str), "relative_to": F(str),
}
TASK = {"name": F(str, True), "request": F(str, True), "goals": F("list", True, of=GOAL), "needs": F("vec_str")}
SCENE = {
    "name": F(str, True), "description": F(str),
    "surface": F({
        "outline_m": F("outline", True), "lip_height_m": F(NUM, True, 0.005, 0.1), "lip_thickness_m": F(NUM, lo=0.002, hi=0.05),
        "friction": F(NUM, lo=0.05, hi=2.0), "colour": F("colour"), "texture": F(str, choices=("plain", "grain")),
        "depth_m": F(NUM, lo=0.02, hi=2.0),
    }, True),
    "light": F({"key": F({"from_m": F("vec", True, -5, 5, n=3), "strength": F(NUM, lo=0, hi=1)}),
                "fill": F({"from_m": F("vec", True, -5, 5, n=3), "strength": F(NUM, lo=0, hi=1)}),
                "ambient": F(NUM, lo=0, hi=1)}),
    "fixtures": F("list", of=FIXTURE), "objects": F("list", of=OBJECT), "devices": F("list", True, of=DEVICE),
    "regions": F("list", of=REGION), "tasks": F("list", of=TASK),
    "imperfections": F({"seed": F(int, lo=0, hi=2 ** 31)}),
}


def _num(v) -> bool:
    return isinstance(v, NUM) and not isinstance(v, bool) and math.isfinite(v)


def _check(value, field, path: str, out: list[str]):
    kind = field["kind"]
    if isinstance(kind, dict):
        if not isinstance(value, dict):
            out.append(f"{path}: must be a table")
            return
        _table(value, kind, path, out)
    elif kind == "list":
        if not isinstance(value, list) or not all(isinstance(v, dict) for v in value):
            out.append(f"{path}: must be a list of tables")
            return
        for i, v in enumerate(value):
            _table(v, field["of"], f"{path}[{i}]" + (f" ({v.get('name')})" if isinstance(v.get("name"), str) else ""), out)
    elif kind == "vec":
        n = field["n"]
        ok_len = n is None or (len(value) in n if isinstance(n, tuple) else len(value) == n) if isinstance(value, list) else False
        if not isinstance(value, list) or not ok_len or not all(_num(v) for v in value):
            out.append(f"{path}: must be a list of {n if not isinstance(n, tuple) else ' or '.join(map(str, n))} numbers")
        elif any((field["lo"] is not None and v < field["lo"]) or (field["hi"] is not None and v > field["hi"]) for v in value):
            out.append(f"{path}: {value} is outside {field['lo']} to {field['hi']}")
    elif kind == "vec_str":
        if not isinstance(value, list) or not all(isinstance(v, str) for v in value):
            out.append(f"{path}: must be a list of words")
    elif kind == "outline":
        if not isinstance(value, list) or len(value) < 4 or not all(
                isinstance(p, list) and len(p) == 2 and all(_num(v) for v in p) for p in value):
            out.append(f"{path}: must be a list of at least four [x, y] corners")
    elif kind == "colour":
        if isinstance(value, str):
            if value not in COLOURS:
                out.append(f"{path}: no colour called '{value}'; there are {sorted(COLOURS)}")
        elif not (isinstance(value, list) and len(value) == 3 and all(_num(v) and 0 <= v <= 1 for v in value)):
            out.append(f"{path}: must be a colour name or [r, g, b] between 0 and 1")
    else:
        if isinstance(value, bool) and kind is not bool or not isinstance(value, kind):
            out.append(f"{path}: must be {'a number' if kind == NUM else kind.__name__}")
            return
        if field["choices"] is not None and value not in field["choices"]:
            out.append(f"{path}: '{value}' is not one of {list(field['choices'])}")
        if kind in (NUM, int) and ((field["lo"] is not None and value < field["lo"]) or (field["hi"] is not None and value > field["hi"])):
            out.append(f"{path}: {value} is outside {field['lo']} to {field['hi']}")
        if kind in (NUM, int) and not _num(value):
            out.append(f"{path}: must be a finite number")


def _table(table: dict, spec: dict, path: str, out: list[str]):
    for k in table:
        if k not in spec:
            out.append(f"{path + '.' if path else ''}{k}: not a field of a scene; allowed here: {sorted(spec)}")
    for k, field in spec.items():
        if k not in table:
            if field["required"]:
                out.append(f"{path + '.' if path else ''}{k}: is required")
            continue
        _check(table[k], field, f"{path + '.' if path else ''}{k}", out)


def check(scene) -> list[str]:
    """Problems with the file's fields alone (no model is built). Empty means well-formed."""
    if not isinstance(scene, dict):
        return ["a scene must be a table"]
    out: list[str] = []
    _table(scene, SCENE, "", out)
    if out:
        return out
    names: dict[str, str] = {}
    for group in ("fixtures", "objects", "devices", "regions"):
        for item in scene.get(group, []):
            n = item["name"]
            if not n.replace("_", "").isalnum():
                out.append(f"{group} '{n}': names use letters, digits and _ only")
            if n in names:
                out.append(f"{group} '{n}': the name is already used by one of the {names[n]}")
            names[n] = group
    kinds = [d["kind"] for d in scene["devices"]]
    if kinds.count("arm") > 1:
        out.append("devices: more than one arm is not supported yet")
    if kinds.count("rover") > 1:
        out.append("devices: more than one rover is not supported yet")
    for d in scene["devices"]:
        if d["kind"] == "camera" and ("looking_at_m" not in d or len(d["at_m"]) != 3):
            out.append(f"devices '{d['name']}': a camera needs at_m = [x, y, z] and looking_at_m")
    for o in scene.get("objects", []):
        if "at_m" not in o and "on" not in o:
            out.append(f"objects '{o['name']}': needs at_m or on")
        if len(o["size_m"]) != (3 if o["shape"] == "box" else 2):
            out.append(f"objects '{o['name']}': a {o['shape']} has size_m = " +
                       ("[x, y, z]" if o["shape"] == "box" else "[diameter, length]"))
    need = {"box": ("size_m",), "bin": ("inner_m", "wall_height_m"), "socket": ("size_m", "hole_m"),
            "cabinet": ("inner_m",), "shelf": ("size_m", "height_m"), "parts": ("parts",)}
    for f in scene.get("fixtures", []):
        for k in need[f["kind"]]:
            if k not in f:
                out.append(f"fixtures '{f['name']}': a {f['kind']} needs {k}")
    tnames = set()
    for t in scene.get("tasks", []):
        if t["name"] in tnames:
            out.append(f"tasks '{t['name']}': the name is used twice")
        tnames.add(t["name"])
    return out
