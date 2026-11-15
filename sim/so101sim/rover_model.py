"""The rover in the simulation, generated from its CAD (hardware/rover/rover.json).

    info = add_rover(spec, "rover", x, y, yaw_rad, contact)

adds the rover standing on the surface: a free chassis, two wheels, and the head's chain (pan, lift
with the coupled joint that keeps the head level, tilt, and two camera carriages that move apart
together), with each body's mass and inertia from the CAD, the printed parts as they are for the
cameras to see, simple shapes for contact, two cameras, and its printed tags.

Every servo joint gets a position actuator like the arm's (kp from the same servo), so a rover left
alone holds its pose; the world replaces them with emulated servos on a bus. The camera-spacing
drive is a slide joint: one radian of its servo is `pitch_radius_m` of travel.
"""
from __future__ import annotations

import json
import math
import tomllib

import mujoco
import numpy as np

from hub.arm.config import position_loop, servo_limits

from .config import ROOT

ROVER_DIR = ROOT / "hardware" / "rover"
TAGS = ROOT / "hardware" / "tags"
STOW = {"lift": math.radians(-15.0)}                 # how it stands when put down: head forward and low, cameras together
PRINT_RGBA = [0.86, 0.5, 0.16, 1.0]
SERVO_FRICTIONLOSS, SERVO_ARMATURE = 0.052, 0.028    # as the arm's HX-30HM joints (sim/tools/build_model.py)


def assembly() -> dict:
    return json.loads((ROVER_DIR / "rover.json").read_text())


def _quat(R) -> list[float]:
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).flatten())
    return [float(v) for v in q]


def add_rover(spec: mujoco.MjSpec, name: str, x: float, y: float, yaw_rad: float, contact: dict, wheel_scale: float = 1.0) -> dict:
    """Add the rover standing on the surface at (x, y), facing yaw. Returns what was added (ground truth).
    wheel_scale: its wheels are this many times the size its description gives (an imperfection: the hub is not told)."""
    asm = assembly()
    loop, lim = position_loop(), servo_limits()
    kp, kv_damping, stall = float(loop["stiffness_nm_per_rad"]), float(loop["back_emf_nm_s_per_rad"]), float(lim["stall_torque_nm"])
    with (TAGS / "layout.toml").open("rb") as f:
        layout = tomllib.load(f)
    bodies, origin = {}, {}
    joints, start, cameras = [], {}, []
    for b in asm["bodies"]:
        o = np.array(b["origin_m"])
        origin[b["name"]] = o
        bname = name if b["parent"] is None else f"{name}_{b['name']}"
        if b["parent"] is None:
            body = spec.worldbody.add_body(name=bname, pos=[x, y, max(0.0, asm["wheel_radius_m"] * (wheel_scale - 1.0))],
                                           quat=[math.cos(yaw_rad / 2), 0, 0, math.sin(yaw_rad / 2)])
            body.add_freejoint(name=f"{name}_free")
        else:
            body = bodies[b["parent"]].add_body(name=bname, pos=(o - origin[b["parent"]]).tolist())
        bodies[b["name"]] = body
        body.mass, body.ipos = b["mass_kg"], b["com_m"]
        body.fullinertia = b["inertia_kg_m2"]
        body.explicitinertial = True
        j = b["joint"]
        if j is not None:
            jname = f"{name}_{j['name']}"
            slide = j["type"] == "slide"
            ratio = abs(float(j["ratio"])) if slide else 1.0            # joint units per radian of its servo
            joint = body.add_joint(name=jname, type=mujoco.mjtJoint.mjJNT_SLIDE if slide else mujoco.mjtJoint.mjJNT_HINGE,
                                   axis=j["axis"], damping=kv_damping / ratio ** 2, frictionloss=SERVO_FRICTIONLOSS / ratio,
                                   armature=SERVO_ARMATURE / ratio ** 2)
            rng = j.get("range_m") or j.get("range_rad")
            if rng is not None:
                joint.range = [float(rng[0]), float(rng[1])]
                joint.limited = mujoco.mjtLimited.mjLIMITED_TRUE
            if "coupled_to" in j:                                       # no servo of its own: it follows another joint
                eq = spec.add_equality(name=f"{jname}_follows", type=mujoco.mjtEq.mjEQ_JOINT, objtype=mujoco.mjtObj.mjOBJ_JOINT,
                                       name1=jname, name2=f"{name}_{j['coupled_to']}")
                eq.data = [0.0, float(j["ratio"]), 0.0, 0.0, 0.0] + [0.0] * 6
                eq.solref, eq.solimp = [0.004, 1.0], [0.98, 0.99, 0.0005, 0.5, 2.0]
                joint.damping = [0.0, 0.0, 0.0]                         # (MuJoCo 3.10 keeps damping as three coefficients)
                joint.frictionloss, joint.armature = 0.0, 1e-4 / ratio ** 2
            else:
                act = spec.add_actuator(name=jname, target=jname, trntype=mujoco.mjtTrn.mjTRN_JOINT)
                act.gaintype, act.biastype = mujoco.mjtGain.mjGAIN_FIXED, mujoco.mjtBias.mjBIAS_AFFINE
                act.gainprm = [kp / ratio ** 2] + [0.0] * 9
                act.biasprm = [0.0, -kp / ratio ** 2, 0.0] + [0.0] * 7
                act.forcelimited, act.forcerange = True, [-stall / ratio, stall / ratio]
                joints.append({"name": jname, "joint": j["name"], "servo": int(j["servo"]), "direction": int(j.get("direction", 1)),
                               "ratio": ratio, "mode": j.get("mode", "position"), "range": rng,
                               "zero_ticks": int(j.get("zero_ticks", 2048))})
            start[jname] = STOW.get(j["name"], 0.0) if "coupled_to" not in j else STOW.get(j["coupled_to"], 0.0) * float(j["ratio"])
        for part in b["prints"]:
            mesh = f"{name}_{part}"
            spec.add_mesh(name=mesh, file=str(ROVER_DIR / "parts" / f"{part}.stl"), scale=[0.001, 0.001, 0.001])
            body.add_geom(name=mesh, type=mujoco.mjtGeom.mjGEOM_MESH, meshname=mesh, rgba=PRINT_RGBA, contype=0, conaffinity=0,
                          group=2, mass=0)
        for k, box in enumerate(b["bought"]):
            lo, hi = np.array(box["lo_m"]), np.array(box["hi_m"])
            body.add_geom(name=f"{name}_{box['name']}", type=mujoco.mjtGeom.mjGEOM_BOX, pos=((lo + hi) / 2).tolist(),
                          size=((hi - lo) / 2).tolist(), rgba=box["rgba"], contype=0, conaffinity=0, group=2, mass=0)
        for k, c in enumerate(b["collision"]):
            fr = [float(c.get("friction", 0.6)), 0.005, 0.0001]
            common = dict(name=f"{name}_{b['name']}_{c.get('name', f'collision{k}')}", rgba=[0.3, 0.6, 0.3, 0.3], group=3, mass=0,
                          friction=fr, **contact)
            if "friction" in c and c["friction"] < 0.2:
                # A ball caster and a skid slide: their own low friction must win. Without this MuJoCo takes the
                # larger of the two surfaces' frictions (the table's 0.5), and the caster drags the rover sideways.
                common["priority"] = 1
            if c["type"] == "box":
                lo, hi = np.array(c["lo"]), np.array(c["hi"])
                body.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, pos=((lo + hi) / 2).tolist(), size=((hi - lo) / 2).tolist(), **common)
            elif c["type"] == "sphere":
                body.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, pos=c["centre"], size=[c["radius"], 0, 0], **common)
            else:
                r = c["radii"]                                           # (a wheel: round about its axle, which is its y)
                body.add_geom(type=mujoco.mjtGeom.mjGEOM_ELLIPSOID, pos=c["centre"], size=[r[0] * wheel_scale, r[1], r[2] * wheel_scale],
                              condim=4, **common)
        for cam in b["cameras"]:
            z, up = -np.array(cam["looks"], float), np.array(cam["up"], float)      # a MuJoCo camera looks along its -z
            body.add_camera(name=f"{name}_{cam['name'].split('_', 1)[1]}", pos=cam["lens_m"],
                            quat=_quat(np.column_stack([np.cross(up, z), up, z])), fovy=57.608)
            cameras.append({"name": f"{name}_{cam['name'].split('_', 1)[1]}", "body": bname})
    for tag in layout["devices"]["rover"]["tags"]:
        png = TAGS / "png" / f"{layout['family']}_{tag['id']}.png"
        tex = f"tag_{tag['id']}"
        spec.add_texture(name=tex, type=mujoco.mjtTexture.mjTEXTURE_2D, file=str(png))
        mat = spec.add_material(name=tex)
        mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = tex
        mat.specular, mat.shininess = 0.0, 0.0
        z, up = np.array(tag["normal"], float), np.array(tag["up"], float)
        half = tag["size_m"] * (8 + 2 * int(layout["quiet_zone_cells"])) / 8 / 2
        bodies[tag["on"]].add_geom(name=tex, type=mujoco.mjtGeom.mjGEOM_BOX, size=[half, half, 0.0003],
                                   pos=(np.array(tag["at_m"]) + z * 0.0004).tolist(), quat=_quat(np.column_stack([np.cross(up, z), up, z])),
                                   material=tex, contype=0, conaffinity=0, group=2, mass=0)
    return {"body": name, "free_joint": f"{name}_free", "joints": joints, "start": start, "cameras": cameras,
            "wheel_radius_m": asm["wheel_radius_m"], "track_m": asm["track_m"], "footprint_m": [0.184, 0.142], "height_m": 0.117,
            "mass_kg": round(sum(b["mass_kg"] for b in asm["bodies"]), 4)}
