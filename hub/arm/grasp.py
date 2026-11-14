"""The gripper's own geometry: how wide it opens, where its fixed jaw's pad is, how to turn it square.

Built-in knowledge of the arm (its jaws), used by the arm's actions and by the scripted pick.
"""
from __future__ import annotations

import numpy as np

from .config import wrist_roll_range
from .ik import ArmIK


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


def square_roll(ik: ArmIK, q: np.ndarray) -> np.ndarray:
    """Pick wrist_roll so the jaws close along a world axis (cube faces).

    Searches only the camera-safe roll range (it spans pi, so every jaw
    orientation is available; see hardware/wrist_mount/place_camera.py).
    """
    import mujoco
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
