#!/usr/bin/env python3
"""Build our SO-101 follower MJCF from the pristine upstream model.

Reads third_party/SO-ARM100/Simulation/SO101/so101_new_calib.xml and writes
sim/model/ (so101_follower.xml, cameras_world.xml, assets/). Changes, all
reproducible from this script:

1. Jaw collision. MuJoCo and Newton collide meshes as their convex hull. The
   fixed jaw is one mesh with the wrist housing, so its hull fills the gap
   between the jaws (an object there collides with empty air). Both jaw meshes
   are replaced by a CoACD convex decomposition.
2. Finger pads. A thin box on each jaw's inner face, fitted to the measured
   jaw surface, with higher friction and torsional friction (condim 4), so
   grasp contact is stable and its friction is tunable in one place.
3. Actuators. Torque limit = HX-30HM stall torque (config/servos.toml). Gains
   and joint damping/friction/armature stay upstream's STS3215 values (same
   1:345 gear ratio, near-identical speed).
4. Wrist camera. A <camera> on the gripper body (pose from
   hardware/wrist_mount/wrist_mount_pose.json via config/cameras.toml) and the
   wrist camera assembly (bracket, holder, Camera Module 3, ESP32-P4 board) as
   a welded child body with its mass and collision shapes.
5. Fixed cameras. cameras_world.xml, included by workcell.xml.

Run (needs mujoco, trimesh, coacd, numpy):  python sim/tools/build_model.py
"""
from __future__ import annotations

import math
import shutil
import sys
import tomllib
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim"))
from so101sim.config import load_wrist_mount, look_at_axes, resolve_camera_entry  # noqa: E402  (shared with the runtime)
UPSTREAM = ROOT / "third_party/SO-ARM100/Simulation/SO101"
SIM = ROOT / "sim"
OUT = SIM / "model"
ASSETS = OUT / "assets"
COLL = ASSETS / "collision"

JAW_MESHES = {
    # mesh name -> (body that carries it, max convex hulls)
    # The fixed jaw is one part with the wrist housing; the housing needs few hulls.
    "wrist_roll_follower_so101_v1": ("gripper", 16),
    "moving_jaw_so101_v1": ("moving_jaw_so101_v1", 10),
}
COACD = dict(threshold=0.05, decimate=True, max_ch_vertex=64, seed=0)
# Gap check: points between the jaws (gripper frame, jaws at q = 0, 15.8 mm apart
# at the tip) that no jaw collision hull may contain.
GAP_PROBE = dict(x=(-0.005, 0.005), z=(-0.100, -0.066), y=0.0)
# The jaws' inner faces are stepped (flat lands at x = -7.9 / -9.9 / -11.6 mm
# from the tip back, measured), so each flat land gets its own pad.
PAD = dict(
    scan_z=(-0.104, -0.062),   # window along the jaw, gripper frame [m] (tip .. base)
    min_land=0.003,            # ignore flat lands shorter than this [m]
    inset=0.0005,              # keep pad ends this far inside a land [m]
    half_width=0.0055,         # across the jaw (y) [m]; the jaw is ~13 mm wide at the tip
    half_thickness=0.001,      # [m]
    protrude=0.0005,           # how far the pad face stands proud of the jaw surface [m]
    # Friction assumes rubber/TPU grip pads on the real jaws too (recommended:
    # bare PLA jaws are ~0.3-0.4 and slip). Sliding, torsional, rolling.
    friction="1.0 0.02 0.0005",
    condim="4",
)


def load_toml(p: Path) -> dict:
    with p.open("rb") as f:
        return tomllib.load(f)


def fmt(v, n=6):
    return " ".join(f"{float(a):.{n}g}" for a in v)


def mat_to_quat(R):
    import mujoco
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(R, float).reshape(9))
    return q


def decompose(mesh_file: Path, name: str, max_hulls: int) -> list[str]:
    """CoACD decomposition, cached in sim/.cache/ by mesh content + settings (it takes minutes)."""
    import hashlib
    import coacd
    import trimesh
    key = hashlib.sha256(mesh_file.read_bytes() + repr((sorted(COACD.items()), max_hulls)).encode()).hexdigest()[:16]
    cache = SIM / ".cache" / f"coacd_{name}_{key}"
    if not cache.exists():
        m = trimesh.load(mesh_file, force="mesh")
        parts = coacd.run_coacd(coacd.Mesh(m.vertices, m.faces), max_convex_hull=max_hulls, **COACD)
        tmp = cache.with_suffix(".tmp")
        shutil.rmtree(tmp, ignore_errors=True)
        tmp.mkdir(parents=True)
        for i, (v, f) in enumerate(parts):
            trimesh.Trimesh(v, f).export(tmp / f"{name}_hull{i:02d}.stl")
        tmp.rename(cache)
    names = []
    for hull in sorted(cache.glob("*.stl")):
        shutil.copy2(hull, COLL / hull.name)
        names.append(hull.stem)
    return names


def geom_triangles(model, data, g, frame_body):
    """Triangles (n, 3, 3) of mesh geom g, expressed in frame_body's frame."""
    mid = model.geom_dataid[g]
    v = model.mesh_vert[model.mesh_vertadr[mid]:model.mesh_vertadr[mid] + model.mesh_vertnum[mid]]
    f = model.mesh_face[model.mesh_faceadr[mid]:model.mesh_faceadr[mid] + model.mesh_facenum[mid]]
    w = (data.geom_xmat[g].reshape(3, 3) @ v.T).T + data.geom_xpos[g]
    fb = model.body(frame_body).id
    R, p = data.xmat[fb].reshape(3, 3), data.xpos[fb]
    return ((R.T @ (w - p).T).T)[f]


def ray_hits(tris, origin, direction):
    """Nearest hit distance of one ray against triangles (Moller-Trumbore), or inf."""
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    e1, e2 = v1 - v0, v2 - v0
    h = np.cross(direction, e2)
    a = np.einsum("ij,ij->i", e1, h)
    ok = np.abs(a) > 1e-12
    f = np.where(ok, 1.0 / np.where(ok, a, 1.0), 0.0)
    s = origin - v0
    u = f * np.einsum("ij,ij->i", s, h)
    q = np.cross(s, e1)
    v = f * (q @ direction)
    t = f * np.einsum("ij,ij->i", e2, q)
    hit = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-9)
    return t[hit].min() if hit.any() else np.inf


def jaw_lands(model, data, mesh_name, body_name, side):
    """Flat lands of a jaw's inner face, in the gripper frame (jaws at q = 0).

    Casts rays from the gap centre (x = 0) toward the jaw (side=+1: fixed jaw,
    toward -x; side=-1: moving jaw, toward +x) every 0.5 mm along z, at three
    y offsets, and groups runs of constant x. Returns [(z_lo, z_hi, x_face)].
    """
    import mujoco
    data.qpos[:] = 0
    mujoco.mj_forward(model, data)
    bid = model.body(body_name).id
    g = next(g for g in range(model.ngeom)
             if model.geom_bodyid[g] == bid and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH
             and mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, model.geom_dataid[g]) == mesh_name)
    tris = geom_triangles(model, data, g, "gripper")
    direction = np.array([-1.0, 0.0, 0.0]) * side
    samples = []
    for z in np.arange(*PAD["scan_z"], 0.0005):
        hits = [ray_hits(tris, np.array([0.0, y, z]), direction)
                for y in (-0.6 * PAD["half_width"], 0.0, 0.6 * PAD["half_width"])]
        hits = sorted(h for h in hits if np.isfinite(h))
        if len(hits) >= 2:
            samples.append((z, -side * hits[len(hits) // 2]))   # median: robust to a hole at one y
    lands, run = [], []
    for z, x in samples + [(None, None)]:
        if run and (z is None or abs(x - run[-1][1]) > 0.0002 or z - run[-1][0] > 0.0006):
            if run[-1][0] - run[0][0] >= PAD["min_land"]:
                lands.append((run[0][0], run[-1][0], float(np.median([r[1] for r in run]))))
            run = []
        if z is not None:
            run.append((z, x))
    if not lands:
        raise RuntimeError(f"{mesh_name}: no flat lands found on the inner face")
    return lands


def pad_pose_gripper_frame(z_lo, z_hi, x_face, side):
    """Axis-aligned pad on one land; its face stands PAD['protrude'] proud."""
    n = np.array([side, 0.0, 0.0])                      # into the gap
    center = np.array([x_face, 0.0, 0.5 * (z_lo + z_hi)]) + n * (PAD["protrude"] - PAD["half_thickness"])
    R = np.column_stack([n, np.cross([0.0, 0.0, 1.0], n), [0.0, 0.0, 1.0]])
    half = [PAD["half_thickness"], PAD["half_width"], 0.5 * (z_hi - z_lo) - PAD["inset"]]
    return center, R, half


def gap_intrusions(model, data):
    """Count gap probe points inside any jaw collision hull (jaws at q = 0)."""
    import mujoco
    data.qpos[:] = 0
    mujoco.mj_forward(model, data)
    jaw_bodies = {model.body(b).id for b, _ in JAW_MESHES.values()}
    xs = np.linspace(*GAP_PROBE["x"], 5)
    zs = np.linspace(*GAP_PROBE["z"], 12)
    probes = np.array([[x, GAP_PROBE["y"], z] for x in xs for z in zs])
    inside = np.zeros(len(probes), bool)
    for g in range(model.ngeom):
        if (model.geom_bodyid[g] not in jaw_bodies or model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH
                or model.geom_contype[g] == 0):
            continue
        tris = geom_triangles(model, data, g, "gripper")
        c = tris.reshape(-1, 3).mean(0)
        n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
        n *= np.sign(np.einsum("ij,ij->i", n, tris[:, 0] - c))[:, None]   # outward
        d = np.einsum("pj,fj->pf", probes, n) - np.einsum("fj,fj->f", tris[:, 0], n)[None]
        inside |= (d <= 0).all(axis=1)
    return int(inside.sum()), len(probes)


WRIST_MOUNT = ROOT / "hardware/wrist_mount"
# Printed-part masses at typical settings (3 walls, ~20% infill): about 40-45 %
# of the solid-PLA mass from the CAD reports. Camera and board are datasheet-ish.
BRACKET_MASS = 0.014


def add_wrist_assembly(root, asset) -> str:
    """Add the wrist camera assembly as a welded child body of the gripper."""
    import trimesh
    pose = load_wrist_mount()
    R = np.array(pose["holder_rotation"])
    o = np.array(pose["holder_origin"])
    out = ASSETS / "wrist_mount"
    out.mkdir(exist_ok=True)
    asm = WRIST_MOUNT / "assembly"
    shutil.copy2(asm / "bracket_gripper_frame.stl", out / "bracket.stl")
    shutil.copy2(asm / "holder_gripper_frame.stl", out / "holder.stl")
    shutil.copy2(asm / "board_envelope_gripper_frame.stl", out / "board.stl")
    cam = trimesh.load(ROOT / "hardware/camera_mount/preview_camera_module3.stl")   # holder frame, mm
    cam.apply_transform(np.block([[R, (o * 1000)[:, None]], [np.zeros((1, 3)), np.ones((1, 1))]]))
    cam.export(out / "camera_module3.stl")
    for n in ("bracket", "holder", "board", "camera_module3"):
        ET.SubElement(asset, "mesh", name=f"wm_{n}", file=f"wrist_mount/{n}.stl", scale="0.001 0.001 0.001")

    masses = dict(pose["masses_kg"], bracket=BRACKET_MASS)
    parts = {k: trimesh.load(out / f"{k}.stl") for k in ("bracket", "holder", "board")}
    parts["camera"] = cam
    total = sum(masses.values())
    com = sum(masses[k] * parts[k].bounding_box.centroid / 1000 for k in masses) / total
    allv = np.vstack([p.vertices for p in parts.values()]) / 1000
    ext = allv.max(0) - allv.min(0)
    inertia = total / 12 * np.array([ext[1] ** 2 + ext[2] ** 2, ext[0] ** 2 + ext[2] ** 2, ext[0] ** 2 + ext[1] ** 2])

    gripper = next(b for b in root.iter("body") if b.get("name") == "gripper")
    body = ET.SubElement(gripper, "body", name="wrist_camera_assembly", pos="0 0 0")
    ET.SubElement(body, "inertial", pos=fmt(com), mass=f"{total:.4f}", diaginertia=fmt(inertia))
    colours = {"bracket": "0.25 0.5 0.9 1", "holder": "0.95 0.56 0.2 1", "board": "0.1 0.45 0.2 1",
               "camera_module3": "0.2 0.2 0.25 1"}
    for n, rgba in colours.items():
        ET.SubElement(body, "geom", {"name": f"wm_{n}_visual", "type": "mesh", "class": "visual",
                                     "mesh": f"wm_{n}", "rgba": rgba})
    # Collision: bracket hull (all outside the gripper's +y face) + holder and board boxes.
    ET.SubElement(body, "geom", {"name": "wm_bracket_collision", "type": "mesh", "class": "collision",
                                 "mesh": "wm_bracket"})
    q = mat_to_quat(R)
    for n, box in (("holder", pose["holder_box_holder_frame"]), ("board", pose["board_box_holder_frame"])):
        c_local = np.array([(b[0] + b[1]) / 2 for b in box])
        half = [(b[1] - b[0]) / 2 for b in box]
        ET.SubElement(body, "geom", {"name": f"wm_{n}_collision", "type": "box", "class": "collision",
                                     "pos": fmt(R @ c_local + o), "quat": fmt(q), "size": fmt(half)})
    # The moving jaw is a sibling body (not filtered as parent/child); the mount
    # sits outside its swing anyway (y > 24 mm vs <= 23.9 mm), but be explicit.
    contact = root.find("contact")
    if contact is None:
        contact = ET.Element("contact")
        root.insert(list(root).index(root.find("actuator")), contact)
    ET.SubElement(contact, "exclude", body1="wrist_camera_assembly", body2="moving_jaw_so101_v1")
    return (f"wrist camera assembly: {total * 1000:.0f} g (bracket {BRACKET_MASS * 1000:.0f}, "
            f"holder {masses['holder'] * 1000:.0f}, camera {masses['camera'] * 1000:.0f}, "
            f"board {masses['board'] * 1000:.0f}), COM {np.round(com * 1000, 1).tolist()} mm")


def main() -> int:
    import mujoco

    cams = load_toml(SIM / "config/cameras.toml")
    servos = load_toml(SIM / "config/servos.toml")

    if OUT.exists():
        shutil.rmtree(OUT)
    COLL.mkdir(parents=True)

    src_xml = UPSTREAM / "so101_new_calib.xml"
    tree = ET.parse(src_xml)
    root = tree.getroot()
    root.set("model", "so101_follower")

    # Meshes: copy only what the model references.
    asset = root.find("asset")
    for mesh in asset.findall("mesh"):
        shutil.copy2(UPSTREAM / "assets" / mesh.get("file"), ASSETS / mesh.get("file"))

    # Upstream kinematics for measuring the jaws.
    up_model = mujoco.MjModel.from_xml_path(str(src_xml))
    up_data = mujoco.MjData(up_model)

    # 1. Jaw collision -> convex decomposition.
    report = []
    for mesh_name, (body_name, max_hulls) in JAW_MESHES.items():
        hulls = decompose(UPSTREAM / "assets" / f"{mesh_name}.stl", mesh_name, max_hulls)
        for hn in hulls:
            ET.SubElement(asset, "mesh", name=hn, file=f"collision/{hn}.stl")
        body = next(b for b in root.iter("body") if b.get("name") == body_name)
        coll = [g for g in body.findall("geom") if g.get("class") == "collision" and g.get("mesh") == mesh_name]
        assert len(coll) == 1, (mesh_name, len(coll))
        g = coll[0]
        idx = list(body).index(g)
        body.remove(g)
        for k, hn in enumerate(hulls):
            body.insert(idx + k, ET.Element("geom", {
                "type": "mesh", "class": "collision", "mesh": hn,
                "pos": g.get("pos"), "quat": g.get("quat")}))
        report.append(f"{mesh_name}: {len(hulls)} convex hulls (CoACD {COACD}, max {max_hulls})")

    # 2. Finger pads, fitted to the jaw faces (moving jaw measured at q = 0).
    up_data.qpos[:] = 0
    mujoco.mj_forward(up_model, up_data)
    gb, mb = up_model.body("gripper").id, up_model.body("moving_jaw_so101_v1").id
    Rg, pg = up_data.xmat[gb].reshape(3, 3), up_data.xpos[gb]
    Rm, pm = up_data.xmat[mb].reshape(3, 3), up_data.xpos[mb]
    R_gm = Rg.T @ Rm                  # moving-jaw body in gripper frame (q = 0)
    p_gm = Rg.T @ (pm - pg)
    for side, mesh_name, body_name, prefix in (
            (+1, "wrist_roll_follower_so101_v1", "gripper", "pad_fixed"),
            (-1, "moving_jaw_so101_v1", "moving_jaw_so101_v1", "pad_moving")):
        lands = jaw_lands(up_model, up_data, mesh_name, body_name, side)
        body = next(bd for bd in root.iter("body") if bd.get("name") == body_name)
        for k, (z_lo, z_hi, x_face) in enumerate(lands):
            c, R, half = pad_pose_gripper_frame(z_lo, z_hi, x_face, side)
            if body_name != "gripper":   # express in the moving jaw's own frame
                c = R_gm.T @ (c - p_gm)
                R = R_gm.T @ R
            body.append(ET.Element("geom", {
                "name": f"{prefix}_{k}", "type": "box", "class": "collision",
                "pos": fmt(c), "quat": fmt(mat_to_quat(R)), "size": fmt(half),
                "friction": PAD["friction"], "condim": PAD["condim"], "rgba": "0.9 0.3 0.2 1"}))
            report.append(f"{prefix}_{k}: land x = {x_face * 1000:+.2f} mm, z {z_lo * 1000:.1f}..{z_hi * 1000:.1f} mm")

    # 3. Actuator torque limit = HX-30HM stall torque.
    tl = servos["follower"]["torque_limit_nm"]
    for d in root.iter("default"):
        if d.get("class") == "sts3215":
            d.set("class", "hx30hm")
            pos_el = d.find("position")
            pos_el.set("forcerange", f"{-tl} {tl}")
    for el in root.iter():
        if el.get("class") == "sts3215":
            el.set("class", "hx30hm")
    for act in root.find("actuator"):
        act.set("forcerange", f"{-tl} {tl}")
    report.append(f"actuators: forcerange +/-{tl} N.m (HX-30HM stall)")

    # 4. Wrist camera on the gripper body.
    def cam_element(name, spec):
        intr = cams["intrinsics"][spec["intrinsics"]]
        x, y, _ = look_at_axes(spec["pos"], spec["look_at"], spec["up"])
        fovy = math.degrees(2 * math.atan(0.5 * intr["calib_height"] / intr["fy"]))
        return ET.Element("camera", {"name": name, "pos": fmt(spec["pos"]),
                                     "xyaxes": fmt(np.concatenate([x, y])), "fovy": f"{fovy:.3f}"})

    for name, spec in cams["cameras"].items():
        spec = resolve_camera_entry(spec)
        if spec.get("enabled", True) is False or spec["parent_body"] == "world":
            continue
        body = next(bd for bd in root.iter("body") if bd.get("name") == spec["parent_body"])
        cam = cam_element(name, spec)
        body.append(cam)
        report.append(f"camera {name}: on {spec['parent_body']}, fovy {cam.get('fovy')} deg")

    # 4b. Wrist camera assembly (bracket + holder + Camera Module 3 + ESP32-P4)
    # from hardware/wrist_mount; a welded child body of the gripper.
    report.append(add_wrist_assembly(root, asset))

    ET.indent(tree, space="  ")
    header = ("<!-- GENERATED by sim/tools/build_model.py from "
              "third_party/SO-ARM100/Simulation/SO101/so101_new_calib.xml. Do not edit; "
              "change the generator or sim/config/*.toml and re-run. -->\n")
    (OUT / "so101_follower.xml").write_text(header + ET.tostring(root, encoding="unicode") + "\n")

    # 5. Fixed cameras for the workcell.
    world = ET.Element("mujoco")
    for name, spec in cams["cameras"].items():
        spec = resolve_camera_entry(spec)
        if spec.get("enabled", True) is False or spec["parent_body"] != "world":
            continue
        world.append(cam_element(name, spec))
        report.append(f"camera {name}: fixed at {spec['pos']} looking at {spec['look_at']}")
    ET.indent(world, space="  ")
    (OUT / "cameras_world.xml").write_text(
        "<!-- GENERATED by sim/tools/build_model.py from sim/config/cameras.toml -->\n"
        + ET.tostring(world, encoding="unicode") + "\n")

    shutil.copy2(SIM / "scene/workcell.xml", OUT / "workcell.xml")

    # Validate: compiles, and no jaw hull intrudes into the gap between the jaws.
    model = mujoco.MjModel.from_xml_path(str(OUT / "so101_follower.xml"))
    n_in, n_probe = gap_intrusions(model, mujoco.MjData(model))
    report.append(f"gap check: {n_in}/{n_probe} probe points between the jaws inside a jaw hull")
    print("\n".join(report))
    if n_in:
        print("FAIL: jaw collision hulls fill the gripping gap; lower COACD threshold or raise max hulls")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
