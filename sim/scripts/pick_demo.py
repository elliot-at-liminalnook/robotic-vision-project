#!/usr/bin/env python3
"""Scripted top-down pick of the cube: an end-to-end check of the sim.

Exercises the robot model (IK, servo speed limit, torque-limited gripper, the
jaw collision hulls and finger pads), the workcell objects, and all cameras.
Saves each camera's view at every phase plus a contact sheet and result.json.

    python sim/scripts/pick_demo.py --backend newton            # Linux (GPU: MuJoCo Warp)
    python sim/scripts/pick_demo.py --backend newton --solver cpu
    python sim/scripts/pick_demo.py --backend mujoco            # anywhere MuJoCo runs

Exit status 0 = cube lifted and held.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from so101sim import JOINTS, SlewLimiter, load_backend  # noqa: E402
from so101sim.ik import ArmIK  # noqa: E402

CUBE_SIZE = 0.030
GRIPPER_CLOSED_CMD = -0.17   # command past contact; the torque limit sets grip force


def gripper_open_angle(ik: ArmIK, width: float) -> float:
    for q in np.linspace(0.0, 1.7, 171):
        if ik.jaw_gap(q) >= width:
            return float(q)
    raise RuntimeError(f"gripper can't open to {width * 1000:.0f} mm")


def fixed_pad_face(ik: ArmIK) -> np.ndarray:
    """Centre of the fixed jaw's tip pad face, in the gripper body frame."""
    import mujoco
    ik.d.qpos[:] = 0
    mujoco.mj_forward(ik.m, ik.d)
    g, pad = ik.gripper, ik.m.geom("pad_fixed_0").id
    R, p = ik.d.xmat[g].reshape(3, 3), ik.d.xpos[g]
    c = R.T @ (ik.d.geom_xpos[pad] - p)
    n = R.T @ ik.d.geom_xmat[pad].reshape(3, 3)[:, 0]   # pad local x = into the gap
    return c + n * ik.m.geom_size[pad][0]


GRASP_CLEARANCE = 0.004   # descend with the fixed jaw this far off the object's face


def solve_grasp(ik: ArmIK, object_center, width: float, q_seed, dz: float = 0.0,
                clearance: float = GRASP_CLEARANCE):
    """IK so the object's centre lands width/2 + clearance in from the fixed jaw's pad.

    The fixed jaw doesn't move; the moving jaw closes the object onto it,
    sliding it the last few mm. The clearance keeps the fixed jaw from landing
    on the object's edge on the way down (with zero clearance, any small error
    pushes the object before the grasp). The IK
    target is the 'gripperframe' site, so convert: site = point - R @ offset,
    where offset is (pad face + width/2 along the closing axis) - site, in the
    gripper frame. Iterated because R depends on the solution.
    """
    import mujoco
    face = fixed_pad_face(ik)
    ik.d.qpos[:] = 0
    mujoco.mj_forward(ik.m, ik.d)
    g = ik.gripper
    Rg0, pg0 = ik.d.xmat[g].reshape(3, 3), ik.d.xpos[g]
    site_local = Rg0.T @ (ik.d.site_xpos[ik.site] - pg0)
    offset = face + np.array([0.5 * width + clearance, 0.0, 0.0]) - site_local
    offset[2] = 0.0          # keep the tip height as given by dz
    target = np.asarray(object_center, float) + [0.0, 0.0, dz]
    q, err = np.array(q_seed, float), np.inf
    for _ in range(4):
        ik.d.qpos[:6] = q
        mujoco.mj_forward(ik.m, ik.d)
        R = ik.d.xmat[g].reshape(3, 3)
        q, err = ik.solve(target - R @ offset, q)
    return q, err


def square_roll(ik: ArmIK, q: np.ndarray) -> np.ndarray:
    """Pick wrist_roll so the jaws close along a world axis (cube faces).

    Searches only the camera-safe roll range (it spans pi, so every jaw
    orientation is available; see hardware/wrist_mount/place_camera.py).
    """
    import mujoco
    from so101sim.config import wrist_roll_range
    lo, hi = wrist_roll_range()
    best, best_cost = q[4], np.inf
    for roll in np.linspace(max(lo, hi - np.pi), hi, 181):
        ik.d.qpos[:6] = q
        ik.d.qpos[4] = roll
        mujoco.mj_forward(ik.m, ik.d)
        xg = ik.d.xmat[ik.gripper].reshape(3, 3)[:, 0]   # jaw closing axis
        cost = abs(np.sin(2 * np.arctan2(xg[1], xg[0])))
        if cost < best_cost - 1e-9:
            best, best_cost = roll, cost
    out = q.copy()
    out[4] = best
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", choices=("newton", "mujoco"), default="newton")
    ap.add_argument("--solver", choices=("warp", "cpu"), default=None, help="Newton only")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--no-render", action="store_true")
    args = ap.parse_args()

    kw = {"solver": args.solver} if args.backend == "newton" else {}
    t0 = time.perf_counter()
    sim = load_backend(args.backend, **kw)
    t_build = time.perf_counter() - t0
    label = sim.name + (f"-{sim.solver_kind}" if args.backend == "newton" else "")
    out = args.out or Path(__file__).resolve().parents[1] / "out" / label
    out.mkdir(parents=True, exist_ok=True)

    ik = ArmIK()
    q_open = gripper_open_angle(ik, CUBE_SIZE + 0.014)
    home, _ = ik.solve([0.20, 0.0, 0.16], [0, 0, 0, 0, 0, q_open])
    sim.reset(home)
    sim.step(int(0.3 / sim.dt))   # settle objects

    cube0 = sim.body_pos("cube")
    above, e1 = ik.solve(cube0 + [0, 0, 0.08], home)
    above = square_roll(ik, above)
    above, e1 = solve_grasp(ik, cube0, CUBE_SIZE, above, dz=0.08)
    grasp, e2 = solve_grasp(ik, cube0, CUBE_SIZE, above, dz=-0.004)
    lift, e3 = solve_grasp(ik, cube0, CUBE_SIZE, above, dz=0.10)
    closed = lambda q: np.r_[q[:5], GRIPPER_CLOSED_CMD]  # noqa: E731
    phases = [
        ("home", home, 0.3),
        ("above", above, 0.4),
        ("descend", grasp, 0.4),
        ("close", closed(grasp), 0.8),
        ("lift", closed(lift), 0.4),
        ("hold", closed(lift), 1.0),
    ]
    ik_err = max(e1, e2, e3)

    slew = SlewLimiter(sim.q())
    frames = {}
    t_sim0 = time.perf_counter()
    sim_steps = 0
    for phase, goal, settle in phases:
        # Move at servo speed, then let it settle.
        while np.abs(slew.current - goal).max() > 1e-4:
            sim.set_targets(slew.update(goal, sim.dt))
            sim.step(1)
            sim_steps += 1
        n = int(settle / sim.dt)
        sim.set_targets(goal)
        sim.step(n)
        sim_steps += n
        if not args.no_render:
            frames[phase] = sim.render()
    t_sim = time.perf_counter() - t_sim0

    cube1 = sim.body_pos("cube")
    tip = None
    try:
        import mujoco
        ik.d.qpos[:6] = sim.q()
        mujoco.mj_forward(ik.m, ik.d)
        tip = ik.d.site_xpos[ik.site].copy()
    except Exception:  # noqa: BLE001
        pass
    lifted = float(cube1[2] - cube0[2])
    held = tip is not None and float(np.linalg.norm(cube1 - tip)) < 0.03
    success = lifted > 0.07 and held

    result = {
        "backend": label, "success": bool(success),
        "cube_lift_m": round(lifted, 4), "cube_to_tip_m": None if tip is None else round(float(np.linalg.norm(cube1 - tip)), 4),
        "ik_max_error_m": round(ik_err, 5), "gripper_open_rad": round(q_open, 3),
        "tilt_deg": {"above": round(ik.tilt_deg(above), 1), "grasp": round(ik.tilt_deg(grasp), 1),
                     "lift": round(ik.tilt_deg(lift), 1)},
        "final_q": dict(zip(JOINTS, [round(float(v), 4) for v in sim.q()])),
        "sim_time_s": round(sim_steps * sim.dt, 3), "wall_time_s": round(t_sim, 2),
        "real_time_factor": round(sim_steps * sim.dt / max(t_sim, 1e-9), 3), "build_time_s": round(t_build, 2),
    }

    if frames:
        from PIL import Image
        for phase, imgs in frames.items():
            for cam, img in imgs.items():
                Image.fromarray(img).save(out / f"{phase}_{cam}.png")
        cams = list(next(iter(frames.values())).keys())
        h, w = next(iter(frames.values()))[cams[0]].shape[:2]
        sheet = Image.new("RGB", (w * len(cams), h * len(frames)))
        for r, imgs in enumerate(frames.values()):
            for c, cam in enumerate(cams):
                sheet.paste(Image.fromarray(imgs[cam]).resize((w, h)), (c * w, r * h))
        sheet.save(out / "contact_sheet.png")
        result["images"] = str(out)

    (out / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
