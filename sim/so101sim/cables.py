"""Wires as part of the model: the servo bus leads and the wrist camera's USB cable.

Each wire is a chain of short capsules joined by ball joints with a spring
(bending stiffness) and damping. Its first segment is fixed to the part it
plugs into, leaving in the connector's direction; its far end is held at the
other connector (or clip) by a ball constraint. So a wire has its real length:
it hangs slack, is dragged along by the arm, pulls on the joints, lies on the
table, and goes taut if it is too short.

Two halves, used at different times:

  add_cables(root)     generator (tools/build_model.py): adds the chains to the robot's MJCF tree.
  CableSet(model)      runtime (MujocoBackend): lays every wire out for the arm's pose at a reset
                       (a chain cannot start from the model's reference pose: its rest shape is straight).

What is real and what is a guess is listed in config/cables.toml.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .config import _load

# The two 3-pin sockets of an STS3215-shaped servo, in the servo mesh's own frame [m]: on the face
# opposite the output horn, 14 mm from the shaft toward the middle of the body (measured from the mesh).
SERVO_PORT = np.array([-0.0015, 0.0, -0.0194])
SERVO_PORT_DIR = np.array([0.0, 0.0, -1.0])
SERVO_MESHES = ("sts3215_03a_v1", "sts3215_03a_no_horn_v1")


def load_cables() -> dict:
    return _load("cables.toml")


def _quat_to_mat(q) -> np.ndarray:
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(q, float))
    return R.reshape(3, 3)


def _mat_to_quat(R) -> np.ndarray:
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(R, float).reshape(9))
    return q


def _frame_along(direction, y_axis=None) -> np.ndarray:
    """A rotation whose x axis is `direction` (and whose y axis is `y_axis`, if given: a ribbon's fold axis)."""
    x = np.asarray(direction, float)
    x = x / np.linalg.norm(x)
    if y_axis is None:
        helper = np.array([0.0, 0.0, 1.0]) if abs(x[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        y = np.cross(helper, x)
    else:
        y = np.asarray(y_axis, float) - x * float(np.dot(y_axis, x))
    y /= np.linalg.norm(y)
    return np.column_stack([x, y, np.cross(x, y)])


def _vec(el, key, default):
    return np.array([float(v) for v in el.get(key, default).split()])


# ---------------------------------------------------------------- generator side
def _parent_map(root):
    return {child: parent for parent in root.iter() for child in parent}


def resolve_anchor(root, spec: dict) -> tuple[str, np.ndarray, np.ndarray]:
    """(body name, position, exit direction) of one end of a wire, in that body's frame.

    spec is {servo = "<joint it drives>"}, {body = "...", pos = [...], dir = [...]}, or
    {site = "<a site the generator put in the model>", dir = [...]}.
    """
    if "site" in spec:
        parents = _parent_map(root)
        site = next(el for el in root.iter("site") if el.get("name") == spec["site"])
        d = np.asarray(spec.get("dir", [0.0, 0.0, -1.0]), float)
        return parents[site].get("name") or "world", _vec(site, "pos", "0 0 0"), d / np.linalg.norm(d)
    if "servo" in spec:
        parents = _parent_map(root)
        joint = next(j for j in root.iter("joint") if j.get("name") == spec["servo"])
        body = parents[parents[joint]]              # the servo sits in the link before the joint it drives
        geom = next((g for g in body.findall("geom") if g.get("mesh") in SERVO_MESHES and g.get("class") == "visual"), None)
        if geom is None:
            raise ValueError(f"no servo mesh found in body {body.get('name')} for joint {spec['servo']}")
        R = _quat_to_mat(_vec(geom, "quat", "1 0 0 0"))
        return body.get("name"), R @ SERVO_PORT + _vec(geom, "pos", "0 0 0"), R @ SERVO_PORT_DIR
    d = np.asarray(spec["dir"], float)
    return spec["body"], np.asarray(spec["pos"], float), d / np.linalg.norm(d)


def add_cables(root, cfg: dict | None = None) -> list[dict]:
    """Add every wire of config/cables.toml to the robot's MJCF tree. Returns one record per span."""
    cfg = cfg or load_cables()
    worldbody = root.find("worldbody")
    bodies = {b.get("name"): b for b in root.iter("body")}
    bodies["world"] = worldbody
    equality = root.find("equality")
    if equality is None:
        equality = ET.SubElement(root, "equality")
    contact = root.find("contact")
    if contact is None:
        contact = ET.Element("contact")
        root.insert(list(root).index(root.find("actuator")), contact)
    spans = []
    for cable in cfg["cables"]:
        kind = cfg["types"][cable["type"]]
        points = [cable["from"], *cable.get("via", []), cable["to"]]
        lengths = cable["lengths_m"] if "lengths_m" in cable else [cable["length_m"]]
        if len(lengths) != len(points) - 1:
            raise ValueError(f"{cable['name']}: {len(points) - 1} spans but {len(lengths)} lengths")
        for k, length in enumerate(lengths):
            name = cable["name"] if len(lengths) == 1 else f"{cable['name']}_{k}"
            a_body, a_pos, a_dir = resolve_anchor(root, points[k])
            b_body, b_pos, _ = resolve_anchor(root, points[k + 1])
            n = max(4, int(round(length / float(kind["segment_m"]))))
            seg = length / n
            r, mass = float(kind["radius_m"]), float(kind["mass_per_m"]) * seg
            stiff = float(kind["bend_stiffness_nm2"]) / seg
            inertia = mass * seg ** 2 / 3 + float(kind["joint_armature"])
            damp = 2 * float(kind["damping_ratio"]) * math.sqrt(stiff * inertia)
            # What a wire touches. Wires never collide with each other. Every wire collides with the
            # table and the objects (collision bit 2, which the scene gives them). A wire collides with
            # the arm's own parts (bit 1) only if collide_arm is set: the leads that run inside or
            # against the brackets would otherwise be laid out through solid plastic and jam the joint.
            ribbon = kind.get("shape") == "ribbon"      # a flat cable: a strip that folds about one axis only
            shape = ({"type": "box", "pos": f"{seg / 2:.6g} 0 0",
                      "size": f"{seg / 2:.6g} {float(kind['width_m']) / 2:.6g} {r:.6g}"} if ribbon
                     else {"type": "capsule", "fromto": f"0 0 0 {seg:.6g} 0 0", "size": f"{r:.6g}"})
            geom = {**shape, "mass": f"{mass:.6g}",
                    "rgba": kind["rgba"], "group": "2", "contype": "0",
                    "conaffinity": "3" if cable.get("collide_arm", False) else "2", "friction": "0.5 0.005 0.0001",
                    # A wire is soft: its contacts give, whatever it touches (solmix makes the wire's
                    # values win over the other shape's stiff ones). This also lets a wire that was laid
                    # out through a part work its way out instead of being shot out.
                    "solref": str(kind["contact_solref"]), "solimp": str(kind["contact_solimp"]), "solmix": "1000"}
            parent = ET.SubElement(bodies[a_body], "body", name=f"cable_{name}_0",
                                   pos=" ".join(f"{v:.6g}" for v in a_pos),
                                   quat=" ".join(f"{v:.6g}" for v in _mat_to_quat(_frame_along(a_dir, points[k].get("axis") if ribbon else None))))
            ET.SubElement(parent, "geom", geom)
            for i in range(1, n):
                parent = ET.SubElement(parent, "body", name=f"cable_{name}_{i}", pos=f"{seg:.6g} 0 0")
                ET.SubElement(parent, "joint", {"name": f"cable_{name}_j{i}", "stiffness": f"{stiff:.6g}",
                                                "damping": f"{damp:.6g}", "armature": str(kind["joint_armature"]),
                                                **({"type": "hinge", "axis": "0 1 0"} if ribbon else {"type": "ball"})})
                ET.SubElement(parent, "geom", geom)
            ET.SubElement(parent, "site", name=f"cable_{name}_tip", pos=f"{seg:.6g} 0 0", size="0.001", group="4")
            ET.SubElement(bodies[b_body], "site", name=f"cable_{name}_to", pos=" ".join(f"{v:.6g}" for v in b_pos),
                          size="0.001", group="4")
            ET.SubElement(equality, "connect", name=f"cable_{name}", site1=f"cable_{name}_tip", site2=f"cable_{name}_to",
                          solref="0.004 1", solimp="0.95 0.99 0.001")      # a plug in its socket: as stiff as the rigid parts
            # A wire starts and ends inside or against the part it plugs into: its end segments ignore that part.
            free = max(1, int(round(float(kind["free_end_m"]) / seg)))
            for i in list(range(min(free, n))):
                if a_body != "world":
                    ET.SubElement(contact, "exclude", body1=f"cable_{name}_{i}", body2=a_body)
            for i in range(max(0, n - free), n):
                if b_body != "world" and not (b_body == a_body and i < free):
                    ET.SubElement(contact, "exclude", body1=f"cable_{name}_{i}", body2=b_body)
            spans.append({"name": name, "cable": cable["name"], "type": cable["type"], "ribbon": ribbon, "segments": n, "segment_m": seg,
                          "length_m": length, "from_body": a_body, "to_body": b_body,
                          # which way the slack bows when the wire is laid out, in the frame of the part it
                          # leaves; by default the way it leaves the connector (it loops outward)
                          "bulge": (cable["bulges"][k] if "bulges" in cable else cable.get("bulge", a_dir.tolist())),
                          "collide_arm": bool(cable.get("collide_arm", False))})
    return spans


# ---------------------------------------------------------------- runtime side
class CableSet:
    """The wires of a compiled model: lays them out for the current arm pose, and reports on them."""

    def __init__(self, m: mujoco.MjModel, spans: list[dict]):
        self.m = m
        self.spans = []
        for s in spans:
            n = s["segments"]
            self.spans.append({
                **s,
                "body0": m.body(f"cable_{s['name']}_0").id,
                "from_id": m.body(s["from_body"]).id,
                "qadr": [m.joint(f"cable_{s['name']}_j{i}").qposadr[0] for i in range(1, n)],
                "dadr": [m.joint(f"cable_{s['name']}_j{i}").dofadr[0] for i in range(1, n)],
                "tip": m.site(f"cable_{s['name']}_tip").id, "to": m.site(f"cable_{s['name']}_to").id,
                "eq": m.equality(f"cable_{s['name']}").id,
            })
        self.settle_steps = int(round(float(load_cables()["settle_s"]) / m.opt.timestep))

    def place(self, d: mujoco.MjData):
        """Lay every wire along a smooth slack curve between its ends for the arm pose now in d."""
        mujoco.mj_kinematics(self.m, d)
        for s in self.spans:
            n, seg = s["segments"], s["segment_m"]
            R0 = d.xmat[s["body0"]].reshape(3, 3).copy()
            a = d.xpos[s["body0"]].copy()
            b = d.site_xpos[s["to"]].copy()
            bulge = d.xmat[s["from_id"]].reshape(3, 3) @ np.asarray(s["bulge"], float)
            if s.get("ribbon"):                    # a ribbon stays in the plane it folds in
                y = R0[:, 1]
                b = b - y * float((b - a) @ y)
                bulge = bulge - y * float(bulge @ y)
            pts = slack_path(a, R0[:, 0], b, n, seg, bulge)
            R_prev = R0
            for i in range(1, n):
                x_new = (pts[i + 1] - pts[i]) / seg
                if s.get("ribbon"):               # folds about its own y axis only: one angle per joint
                    x_old, y = R_prev[:, 0], R0[:, 1]
                    angle = math.atan2(float(np.cross(x_old, x_new) @ y), float(x_old @ x_new))
                    d.qpos[s["qadr"][i - 1]] = angle
                    c, sn = math.cos(angle), math.sin(angle)
                    K = np.array([[0, -y[2], y[1]], [y[2], 0, -y[0]], [-y[1], y[0], 0]])
                    R_prev = (np.eye(3) + sn * K + (1 - c) * K @ K) @ R_prev
                else:
                    R_i = _rotation_between(R_prev[:, 0], x_new) @ R_prev
                    d.qpos[s["qadr"][i - 1]:s["qadr"][i - 1] + 4] = _mat_to_quat(R_prev.T @ R_i)
                    R_prev = R_i
        for s in self.spans:
            for da in s["dadr"]:
                d.qvel[da:da + (1 if s.get("ribbon") else 3)] = 0

    def state(self, d: mujoco.MjData) -> list[dict]:
        """Per span: how far its ends are apart compared with its length, and how hard its end is being pulled."""
        out = []
        for s in self.spans:
            a, b = d.xpos[s["body0"]], d.site_xpos[s["to"]]
            gap = float(np.linalg.norm(d.site_xpos[s["tip"]] - b))
            out.append({"name": s["name"], "length_m": s["length_m"], "span_m": float(np.linalg.norm(b - a)),
                        "end_gap_m": gap, "tension_n": self.tension(d, s)})
        return out

    def tension(self, d: mujoco.MjData, s: dict) -> float:
        """Force the ball constraint at the wire's far end is applying [N]."""
        for i in range(d.nefc):
            if d.efc_id[i] == s["eq"] and d.efc_type[i] == mujoco.mjtConstraint.mjCNSTR_EQUALITY:
                return float(np.linalg.norm(d.efc_force[i:i + 3]))
        return 0.0

    def report(self, d: mujoco.MjData) -> str:
        lines = ["wires:"]
        for st in self.state(d):
            slack = st["length_m"] - st["span_m"]
            lines.append(f"  {st['name']:14s} length {st['length_m'] * 1000:4.0f} mm, ends {st['span_m'] * 1000:4.0f} mm apart "
                         f"({'slack ' + format(slack * 1000, '.0f') + ' mm' if slack >= 0 else 'TOO SHORT by ' + format(-slack * 1000, '.0f') + ' mm'}), "
                         f"pull at the end {st['tension_n']:.2f} N")
        return "\n".join(lines)


def _rotation_between(u, v) -> np.ndarray:
    """The smallest rotation taking unit vector u to unit vector v."""
    axis = np.cross(u, v)
    s, c = np.linalg.norm(axis), float(np.dot(u, v))
    if s < 1e-12:
        if c > 0:
            return np.eye(3)
        perp = np.cross(u, [1.0, 0.0, 0.0] if abs(u[0]) < 0.9 else [0.0, 1.0, 0.0])
        axis, s, c = perp / np.linalg.norm(perp), 0.0, -1.0
        K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        return np.eye(3) + 2 * K @ K
    axis = axis / s
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + s * K + (1 - c) * K @ K


def slack_path(a, direction, b, n: int, seg: float, bulge) -> np.ndarray:
    """n + 1 points from a to b, each `seg` apart, leaving a along `direction`, the slack bowing toward `bulge`.

    If the wire is too short to reach, the points run straight toward b and stop short of it.
    """
    direction = direction / np.linalg.norm(direction)
    p1 = a + direction * seg
    m = n - 1                                    # free segments from p1 to b
    chord = b - p1
    dist = float(np.linalg.norm(chord))
    if dist >= m * seg - 1e-9:                   # taut or too short
        u = chord / max(dist, 1e-12)
        return np.vstack([a] + [p1 + u * seg * i for i in range(m + 1)])
    bulge = np.asarray(bulge, float)
    if dist > 1e-9:                              # bow sideways to the chord
        bulge = bulge - chord * (bulge @ chord) / dist ** 2
    if np.linalg.norm(bulge) < 1e-9:
        bulge = np.cross(chord, [1.0, 0.0, 0.0]) if dist > 1e-9 else np.array([0.0, 0.0, -1.0])
    bulge = bulge / np.linalg.norm(bulge)
    t = np.linspace(0.0, 1.0, m + 1)

    def curve(sag):
        return p1 + np.outer(t, chord) + np.outer(4 * t * (1 - t), bulge) * sag

    lo, hi = 0.0, m * seg
    for _ in range(40):                          # sag that gives the right total length
        mid = 0.5 * (lo + hi)
        if np.linalg.norm(np.diff(curve(mid), axis=0), axis=1).sum() < m * seg:
            lo = mid
        else:
            hi = mid
    pts = curve(0.5 * (lo + hi))
    for _ in range(60):                          # equal segment lengths, both ends fixed (FABRIK)
        pts[-1] = b
        for i in range(m - 1, 0, -1):
            v = pts[i] - pts[i + 1]
            pts[i] = pts[i + 1] + v / max(np.linalg.norm(v), 1e-12) * seg
        pts[0] = p1
        for i in range(1, m + 1):
            v = pts[i] - pts[i - 1]
            pts[i] = pts[i - 1] + v / max(np.linalg.norm(v), 1e-12) * seg
    return np.vstack([a, pts])
