"""Small damped-least-squares IK for scripted motions (top-down grasps).

Uses plain MuJoCo on the generated robot model (MuJoCo is a dependency of
Newton's MuJoCo solver too), so both backends share the same kinematics.
The arm has 5 DOF: shoulder_pan/lift, elbow_flex and wrist_flex place the
gripper tip (site 'gripperframe') and point the fingers straight down;
wrist_roll then sets the jaw direction independently.
"""
from __future__ import annotations

import mujoco
import numpy as np

from .config import MODEL_DIR

_POSITION_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")


class ArmIK:
    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path(str(MODEL_DIR / "so101_follower.xml"))
        self.d = mujoco.MjData(self.m)
        self.site = self.m.site("gripperframe").id
        self.gripper = self.m.body("gripper").id
        self.dofs = [self.m.joint(n).dofadr[0] for n in _POSITION_JOINTS]
        self.qadr = [self.m.joint(n).qposadr[0] for n in _POSITION_JOINTS]
        self.lo = self.m.jnt_range[[self.m.joint(n).id for n in _POSITION_JOINTS], 0]
        self.hi = self.m.jnt_range[[self.m.joint(n).id for n in _POSITION_JOINTS], 1]

    def solve(self, target_pos, q_init, *, iters: int = 400, tol: float = 5e-4) -> tuple[np.ndarray, float]:
        """Joint angles placing the tip at target_pos, fingers as close to straight down as reachable.

        Task-priority IK: the tip position is the primary task; pointing the
        fingers down only uses the remaining (null-space) freedom. On the SO-101,
        straight-down is only reachable fairly low to the table: higher up it
        needs wrist_flex past its limit, so the gripper tilts (see tilt_deg()).

        q_init: full 6-vector (arm + gripper); wrist_roll and gripper are kept.
        Returns (q, position error [m]).
        """
        q = np.array(q_init, float)
        d, m = self.d, self.m
        d.qpos[:] = 0
        d.qpos[:6] = q
        jacp = np.zeros((3, m.nv))
        jacr = np.zeros((3, m.nv))
        z_des = np.array([0.0, 0.0, 1.0])   # gripper body +z up = fingers (-z) down
        lam = 1e-4
        for _ in range(iters):
            mujoco.mj_forward(m, d)
            e_pos = np.asarray(target_pos) - d.site_xpos[self.site]
            z_cur = d.xmat[self.gripper].reshape(3, 3)[:, 2]
            e_rot = np.cross(z_cur, z_des)
            mujoco.mj_jacSite(m, d, jacp, jacr, self.site)
            qa = d.qpos[self.qadr]
            active = np.ones(len(self.dofs), bool)
            for _ in range(len(self.dofs)):
                # Joints at a limit that the step would push further are frozen,
                # so the position task is solved with the joints that can move.
                Jp = jacp[:, self.dofs] * active
                Jo = jacr[:, self.dofs] * active
                Jp_pinv = Jp.T @ np.linalg.inv(Jp @ Jp.T + lam * np.eye(3))
                dq1 = Jp_pinv @ e_pos
                N = np.eye(len(self.dofs)) - Jp_pinv @ Jp
                JoN = Jo @ N
                dq2 = JoN.T @ np.linalg.solve(JoN @ JoN.T + 1e-3 * np.eye(3), e_rot - Jo @ dq1)
                dq = (dq1 + N @ dq2) * active
                blocked = active & (((qa <= self.lo + 1e-6) & (dq < 0)) | ((qa >= self.hi - 1e-6) & (dq > 0)))
                if not blocked.any():
                    break
                active &= ~blocked
            dq = np.clip(dq, -0.1, 0.1)
            if np.linalg.norm(e_pos) < tol and np.linalg.norm(dq) < 1e-6:
                break
            d.qpos[self.qadr] = np.clip(d.qpos[self.qadr] + dq, self.lo, self.hi)
        mujoco.mj_forward(m, d)
        q[:4] = d.qpos[self.qadr]
        return q, float(np.linalg.norm(np.asarray(target_pos) - d.site_xpos[self.site]))

    def tilt_deg(self, q) -> float:
        """Angle between the fingers and straight down at joint angles q [deg]."""
        self.d.qpos[:6] = q
        mujoco.mj_forward(self.m, self.d)
        z = self.d.xmat[self.gripper].reshape(3, 3)[:, 2]
        return float(np.degrees(np.arccos(np.clip(z[2], -1, 1))))

    def jaw_gap(self, q_gripper: float) -> float:
        """Distance between the tip pads' faces [m] at a gripper angle."""
        d, m = self.d, self.m
        d.qpos[:] = 0
        d.qpos[m.joint("gripper").qposadr[0]] = q_gripper
        mujoco.mj_forward(m, d)
        g = m.body("gripper").id
        R, p = d.xmat[g].reshape(3, 3), d.xpos[g]

        def face_x(name):
            gid = m.geom(name).id
            c = R.T @ (d.geom_xpos[gid] - p)
            n = R.T @ d.geom_xmat[gid].reshape(3, 3)[:, 0]    # pad local x = into the gap
            return c + n * m.geom_size[gid][0]

        return float(np.linalg.norm(face_x("pad_moving_0") - face_x("pad_fixed_0")))
