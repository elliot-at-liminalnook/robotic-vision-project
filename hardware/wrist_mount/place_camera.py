#!/usr/bin/env python3
"""Choose where the wrist camera sits on the follower's gripper.

The camera holder (hardware/camera_mount) goes on the gripper's +y side, the
side perpendicular to the jaws' swing plane, so the camera sees both jaws
side-on with the object between them, and the moving jaw never sweeps into
it. The ESP32-P4 board rides on the holder's back, so the camera ribbon
crosses no joints. A bracket (build_wrist_mount.py) bolts to the gripper's
two M3 hex-nut recesses (upstream's wrist-camera attachment) and to the
holder's two M3 holes.

Candidates vary the lens position (distance out from the gripper face, height
along the gripper). Each is scored in MuJoCo on the generated follower model:
  - visibility: both fingertip pads and the grasp centre project inside the
    wrist image (640x360, datasheet intrinsics) with a margin, jaws open and closed;
  - no overlap with the gripper or the moving jaw over the whole jaw range;
  - clearance above the table at a straight-down grasp with the tips 11 mm up;
  - self-collision against the rest of the arm (parent filtering disabled, so
    the wrist link is checked too), on TASK poses: random reachable tip targets
    over the table, fingers as close to straight down as reachable, wrist roll
    within the camera-safe range ROLL_RANGE (spanning pi, so every top-down
    jaw orientation is still available). Must be zero. Uniform samples over the full joint ranges
    are reported for information: they include folded poses (gripper curled
    back toward the base) that no grasping task uses.
Writes wrist_mount_pose.json (holder pose + camera pose in the gripper frame).

Run: python hardware/wrist_mount/place_camera.py   (needs mujoco, numpy)
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "sim"))
from so101sim.config import MODEL_DIR, load_cameras  # noqa: E402

# Gripper frame, metres. The gripper's +y outer face is y = 0.024; upstream's
# attachment screws run along y at x = -0.0050 / +0.0031, z = -0.02335.
GRIPPER_FACE_Y = 0.024
SCREWS_XZ = ((-0.0050, -0.02335), (0.0031, -0.02335))
TARGET = np.array([0.002, 0.0, -0.092])   # between the jaw tips (grasp region)

# Holder (camera_mount) in its own frame, metres: lens axis +z at x = y = 0,
# frame x +/-0.034, y -0.0525..0.033, z -0.006..0.00995 (lens front 0.0158).
HOLDER_BOX = ((-0.034, 0.034), (-0.0525, 0.033), (-0.006, 0.0158))
LENS_OPTICAL_Z = 0.011           # optical centre (lens ~2.75 mm above the sensor)
# ESP32-P4-WIFI6-M (71 x 21 mm, ~14 mm with headers) on standoffs behind the
# holder, long side along the holder's y.
BOARD_BOX = ((-0.0105, 0.0105), (-0.054, 0.017), (-0.024, -0.009))   # below the bracket pad (holder y >= 17.5 mm)
BOARD_MASS = 0.012
HOLDER_MASS = 0.016              # ~33 g solid -> ~15 g at typical infill + walls
CAMERA_MASS = 0.004

# The side-mounted holder swings toward the shoulder at large positive wrist
# roll near the base. Top-down jaw orientation repeats every pi (roll r and
# r - pi square the jaws the same way), so capping roll here loses no grasp
# orientation. The range is exactly pi wide, [hi - pi, hi]: that is the band
# the grasp-pose collision study below samples, so nothing outside what was
# tested is allowed. sim/tools/build_model.py limits the wrist_roll joint and
# actuator to it; set the real servo's angle limits to match.
ROLL_HI = 0.8
ROLL_RANGE = (round(ROLL_HI - math.pi, 5), ROLL_HI)

CANDIDATES = [(y, z) for y in (0.050, 0.058, 0.066) for z in (-0.005, -0.015, -0.025)] + [(0.066, -0.035), (0.074, -0.025)]


def holder_frame(lens_pos: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Holder rotation (columns = holder x, y, z in the gripper frame) and origin."""
    z = TARGET - lens_pos
    z /= np.linalg.norm(z)
    up = np.array([0.0, 0.0, 1.0])            # holder top band toward the wrist
    y = up - z * (up @ z)
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    R = np.column_stack([x, y, z])
    return R, lens_pos - R @ np.array([0.0, 0.0, LENS_OPTICAL_Z])


def box_corners(box, R, o):
    pts = np.array([[x, y, z] for x in box[0] for y in box[1] for z in box[2]])
    return (R @ pts.T).T + o


def inside_box(points, box, R, o, margin=0.0):
    local = (R.T @ (points - o).T).T
    lo = np.array([b[0] for b in box]) - margin
    hi = np.array([b[1] for b in box]) + margin
    return np.all((local >= lo) & (local <= hi), axis=1)


def mesh_points(m, d, body, frame_body="gripper"):
    """Vertices of a body's collision meshes, in frame_body's frame."""
    fb = m.body(frame_body).id
    R, p = d.xmat[fb].reshape(3, 3), d.xpos[fb]
    out = []
    for g in range(m.ngeom):
        if m.geom_bodyid[g] == m.body(body).id and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mid = m.geom_dataid[g]
            v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
            w = (d.geom_xmat[g].reshape(3, 3) @ v.T).T + d.geom_xpos[g]
            out.append((R.T @ (w - p).T).T)
    return np.vstack(out)


def project(points_g, cam_R, cam_p, spec):
    """Pinhole projection (cameras look along -z, +y up) -> pixel coords at render size."""
    i = spec.intrinsics
    s = spec.render_width / i["calib_width"]
    pc = (cam_R.T @ (points_g - cam_p).T).T
    u = i["cx"] * s + i["fx"] * s * (pc[:, 0] / -pc[:, 2])
    v = i["cy"] * s - i["fy"] * s * (pc[:, 1] / -pc[:, 2])
    return np.column_stack([u, v]), -pc[:, 2]


def evaluate(lens_pos, m, d, spec):
    R, o = holder_frame(lens_pos)
    res = {"lens_pos_mm": np.round(lens_pos * 1000, 1).tolist()}
    # Camera frame for projection: looks along -z_cam = holder +z, up = holder +y.
    cam_R = np.column_stack([R[:, 0], R[:, 1], -R[:, 2]]) @ np.diag([-1, 1, 1])
    # (x flipped so the frame stays right-handed: x_cam = -x_holder)
    res["camera_R"] = cam_R
    jq = m.joint("gripper").qposadr[0]
    margin_px, worst_margin = 15, np.inf
    overlap = 0
    for qg in np.linspace(-0.17, 1.74, 12):          # whole jaw range
        d.qpos[:] = 0
        d.qpos[jq] = qg
        mujoco.mj_forward(m, d)
        for body in ("gripper", "moving_jaw_so101_v1"):
            pts = mesh_points(m, d, body)
            overlap += int(inside_box(pts, HOLDER_BOX, R, o, 0.002).sum() + inside_box(pts, BOARD_BOX, R, o, 0.002).sum())
        if qg in (np.linspace(-0.17, 1.74, 12)[1], np.linspace(-0.17, 1.74, 12)[3]):
            pass
    # Visibility: tip pads (jaws open 0.39 rad and closed on a 30 mm cube ~0.10) + grasp centre.
    pts = []
    for qg in (0.10, 0.39):
        d.qpos[:] = 0
        d.qpos[jq] = qg
        mujoco.mj_forward(m, d)
        gb = m.body("gripper").id
        Rg, pg = d.xmat[gb].reshape(3, 3), d.xpos[gb]
        for name in ("pad_fixed_0", "pad_moving_0"):
            pts.append(Rg.T @ (d.geom_xpos[m.geom(name).id] - pg))
    pts.append(TARGET)
    uv, depth = project(np.array(pts), cam_R, lens_pos, spec)
    w, h = spec.render_width, spec.render_height
    margins = np.minimum.reduce([uv[:, 0], w - uv[:, 0], uv[:, 1], h - uv[:, 1]])
    worst_margin = float(margins.min()) if np.all(depth > 0) else -1.0
    res["tips_uv_px"] = np.round(uv, 1).tolist()
    res["visibility_margin_px"] = round(worst_margin, 1)
    res["gripper_overlap_points"] = overlap
    return res, R, o


def table_clearance(R, o, m, d):
    """Lowest point of holder/board above the table, straight-down grasp, tips 11 mm up."""
    from so101sim.ik import ArmIK
    ik = ArmIK()
    q, _ = ik.solve([0.24, 0.06, 0.011], [0, 0, 0, 0, 0, 0.39])
    d.qpos[:] = 0
    d.qpos[:6] = q
    mujoco.mj_forward(m, d)
    gb = m.body("gripper").id
    Rg, pg = d.xmat[gb].reshape(3, 3), d.xpos[gb]
    z = min(((Rg @ box_corners(b, R, o).T).T + pg)[:, 2].min() for b in (HOLDER_BOX, BOARD_BOX))
    return float(z)


def _task_poses(n=200, seed=1):
    """Joint vectors for random reachable grasp-like poses over the table."""
    from so101sim.ik import ArmIK
    ik = ArmIK()
    rng = np.random.default_rng(seed)
    poses = []
    while len(poses) < n:
        r, a = rng.uniform(0.14, 0.32), rng.uniform(-1.2, 1.2)
        target = [r * math.cos(a), r * math.sin(a), rng.uniform(0.01, 0.15)]
        q, err = ik.solve(target, [0, 0, 0, 0, rng.uniform(ROLL_RANGE[1] - math.pi, ROLL_RANGE[1]), rng.uniform(-0.1, 1.2)])
        if err < 0.002:
            poses.append(q)
    return np.array(poses)


_TASK_POSES = None


def self_collisions(R, o, n=400, seed=0, task=False):
    """Fraction of joint samples where holder/board touch another arm link.

    task=False: uniform over the full joint ranges. task=True: grasp-like poses.
    """
    global _TASK_POSES
    xml = (MODEL_DIR / "so101_follower.xml").read_text()
    geoms = "".join(
        f'<geom name="probe_{k}" type="box" pos="{c[0]} {c[1]} {c[2]}" size="{s[0]} {s[1]} {s[2]}" '
        f'quat="{q[0]} {q[1]} {q[2]} {q[3]}" contype="2" conaffinity="1" group="4"/>'
        for k, (c, s, q) in enumerate(_probe_boxes(R, o)))
    marker = '<site group="3" name="gripperframe"'
    xml = xml.replace(marker, geoms + marker, 1)
    m = mujoco.MjModel.from_xml_string(xml.replace('meshdir="assets"', f'meshdir="{MODEL_DIR / "assets"}"'))
    m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_FILTERPARENT
    d = mujoco.MjData(m)
    probes = {m.geom(f"probe_{k}").id for k in range(2)}
    own = {m.body("gripper").id, m.body("moving_jaw_so101_v1").id}
    rng = np.random.default_rng(seed)
    lo, hi = m.jnt_range[:6, 0], m.jnt_range[:6, 1]
    if task:
        if _TASK_POSES is None:
            _TASK_POSES = _task_poses()
        samples = _TASK_POSES
    else:
        samples = rng.uniform(lo, hi, size=(n, 6))
    hits = 0
    for qs in samples:
        d.qpos[:6] = qs
        mujoco.mj_forward(m, d)
        for c in d.contact[:d.ncon]:
            a, b = c.geom1, c.geom2
            if (a in probes) != (b in probes):
                other = b if a in probes else a
                if m.geom_bodyid[other] not in own and c.dist < 0:
                    hits += 1
                    break
    return hits / len(samples)


def _probe_boxes(R, o):
    out = []
    for box in (HOLDER_BOX, BOARD_BOX):
        c_local = np.array([(b[0] + b[1]) / 2 for b in box])
        size = [(b[1] - b[0]) / 2 for b in box]
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, np.ascontiguousarray(R).reshape(9))
        out.append((R @ c_local + o, size, q))
    return out


def main() -> int:
    m = mujoco.MjModel.from_xml_path(str(MODEL_DIR / "so101_follower.xml"))
    d = mujoco.MjData(m)
    spec = load_cameras()["wrist"]
    rows = []
    for y, z in CANDIDATES:
        lens = np.array([TARGET[0], y, z])
        res, R, o = evaluate(lens, m, d, spec)
        res["table_clearance_mm"] = round(table_clearance(R, o, m, d) * 1000, 1)
        res["self_collision_task_poses"] = round(self_collisions(R, o, task=True), 4)
        res["self_collision_full_range_info"] = round(self_collisions(R, o), 4)
        res["ok"] = (res["visibility_margin_px"] >= 15 and res["gripper_overlap_points"] == 0
                     and res["table_clearance_mm"] >= 15 and res["self_collision_task_poses"] == 0)
        res["_R"], res["_o"] = R, o
        rows.append(res)
        print({k: v for k, v in res.items() if not k.startswith("_") and k != "camera_R"})
    good = [r for r in rows if r["ok"]]
    if not good:
        print("no candidate passed all checks")
        return 1
    # Prefer the lens at the gripper screws' height (the bracket goes straight out:
    # shortest, stiffest), then the closest to the grasp (bigger view of it).
    screw_z_mm = SCREWS_XZ[0][1] * 1000
    best = min(good, key=lambda r: (round(abs(r["lens_pos_mm"][2] - screw_z_mm) / 5),
                                    np.linalg.norm(np.array(r["lens_pos_mm"]) - TARGET * 1000)))
    R, o = best["_R"], best["_o"]
    lens = R @ np.array([0, 0, LENS_OPTICAL_Z]) + o
    pose = {
        "frame": "gripper body (sim/model/so101_follower.xml), metres",
        "holder_rotation": R.tolist(), "holder_origin": o.tolist(),
        "camera": {"parent_body": "gripper", "pos": lens.tolist(), "look_at": TARGET.tolist(),
                   "up": R[:, 1].tolist()},
        "screws_xz": SCREWS_XZ, "gripper_face_y": GRIPPER_FACE_Y,
        "wrist_roll_range_rad": ROLL_RANGE,
        "board_box_holder_frame": BOARD_BOX, "holder_box_holder_frame": HOLDER_BOX,
        "masses_kg": {"holder": HOLDER_MASS, "camera": CAMERA_MASS, "board": BOARD_MASS},
        "metrics": {k: v for k, v in best.items() if not k.startswith("_") and k != "camera_R"},
        "candidates": [{k: v for k, v in r.items() if not k.startswith("_") and k != "camera_R"} for r in rows],
    }
    (HERE / "wrist_mount_pose.json").write_text(json.dumps(pose, indent=2) + "\n")
    print("chosen:", best["lens_pos_mm"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
