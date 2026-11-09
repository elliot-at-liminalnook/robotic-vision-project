"""Small damped-least-squares IK for scripted motions (top-down grasps).

Uses plain MuJoCo on the generated robot model (MuJoCo is a dependency of
Newton's MuJoCo solver too), so both backends share the same kinematics.
The arm has 5 DOF: shoulder_pan/lift, elbow_flex and wrist_flex place the
gripper tip (site 'gripperframe') and point the fingers straight down;
wrist_roll then sets the jaw direction independently.

Load-aware mode (tilt_tol_deg > 0): the fingers may tilt up to that many
degrees from straight down, and the arm's one spare degree of freedom is used
to minimise the servos' holding torque instead (sum of squared gravity +
payload torques, so the most loaded joints, shoulder_lift and elbow_flex,
dominate). Default is 0 deg: exactly the strict fingers-down solver.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

from .config import MODEL_DIR
from .dynamics import ArmDynamics

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
        self._dyn = None

    @property
    def dyn(self) -> ArmDynamics:
        if self._dyn is None:
            self._dyn = ArmDynamics()
        return self._dyn

    def load_cost(self, q, payload_kg: float = 0.0) -> float:
        """Sum of squared holding torques [N.m^2] of the arm joints at q (6-vector)."""
        t = self.dyn.gravity(q, payload_kg)[:5]
        return float(t @ t)

    def solve(self, target_pos, q_init, *, iters: int = 400, tol: float = 5e-4, tilt_tol_deg: float = 0.0,
              payload_kg: float = 0.0, smooth: float = 0.2) -> tuple[np.ndarray, float]:
        """Joint angles placing the tip at target_pos, fingers as close to straight down as reachable.

        tilt_tol_deg > 0: load-aware (see module doc). Within the tilt cone the
        null-space motion descends the holding-torque cost, plus `smooth` *
        |q - q_init|^2 so successive solves (teleop) don't drift or jump;
        outside it, the fingers are rotated back toward the cone.

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
        if tilt_tol_deg > 0:
            return self._solve_load_aware(np.asarray(target_pos, float), q, iters, tol,
                                          math.radians(tilt_tol_deg), payload_kg, smooth)
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

    def _solve_load_aware(self, target, q, iters, tol, tilt_tol, payload_kg, smooth):
        """Position task first; the null space (one direction on this arm) does a
        1-D Newton step on cost = holding torque^2 + smooth * |q - q_init|^2,
        never tilting past the cone; outside the cone it rotates back toward it."""
        d, m = self.d, self.m
        d.qpos[:] = 0
        d.qpos[:6] = q
        q0 = q[:4].copy()
        jacp = np.zeros((3, m.nv))
        jacr = np.zeros((3, m.nv))
        z_des = np.array([0.0, 0.0, 1.0])
        lam, nd, h = 1e-4, len(self.dofs), 0.01
        full = np.array(q, float)

        def cost(qa):
            full[:4] = np.clip(qa, self.lo, self.hi)
            dq = qa - q0
            return self.load_cost(full, payload_kg) + smooth * float(dq @ dq)

        for _ in range(iters):
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)
            e_pos = target - d.site_xpos[self.site]
            z_cur = d.xmat[self.gripper].reshape(3, 3)[:, 2]
            cr = np.cross(z_cur, z_des)
            tilt = math.atan2(np.linalg.norm(cr), z_cur @ z_des)
            axis = cr / max(np.linalg.norm(cr), 1e-9)
            mujoco.mj_jacSite(m, d, jacp, jacr, self.site)
            qa = d.qpos[self.qadr].copy()
            active = np.ones(nd, bool)
            for _ in range(nd):
                Jp = jacp[:, self.dofs] * active
                Jo = jacr[:, self.dofs] * active
                Jp_pinv = Jp.T @ np.linalg.inv(Jp @ Jp.T + lam * np.eye(3))
                dq1 = Jp_pinv @ e_pos
                N = (np.eye(nd) - Jp_pinv @ Jp) * active[:, None] * active[None, :]
                dq2 = np.zeros(nd)
                if tilt > tilt_tol:
                    e_rot = axis * (tilt - tilt_tol)       # rotate back by the excess only
                    JoN = Jo @ N
                    dq2 = N @ (JoN.T @ np.linalg.solve(JoN @ JoN.T + 1e-3 * np.eye(3), e_rot - Jo @ dq1))
                else:
                    w, V = np.linalg.eigh(N)
                    if w[-1] > 0.5:
                        n = V[:, -1]
                        c0, cp, cm = cost(qa), cost(qa + h * n), cost(qa - h * n)
                        d1, d2 = (cp - cm) / (2 * h), (cp - 2 * c0 + cm) / h ** 2
                        # Newton step if convex along n, else a fixed step downhill
                        # (the cost is often concave here: the optimum is on the cone).
                        alpha = -d1 / d2 if d2 > 1e-6 else -np.sign(d1) * 0.1
                        alpha = float(np.clip(alpha, -0.1, 0.1))
                        # stay inside the cone: d(tilt)/dq = -axis . Jo
                        ct = -(axis @ Jo)
                        room = tilt_tol - tilt - ct @ dq1
                        rate = ct @ n
                        if alpha * rate > max(room, 0.0):
                            alpha = max(room, 0.0) / rate
                        dq2 = alpha * n
                dq = (dq1 + dq2) * active
                blocked = active & (((qa <= self.lo + 1e-6) & (dq < 0)) | ((qa >= self.hi - 1e-6) & (dq > 0)))
                if not blocked.any():
                    break
                active &= ~blocked
            dq = np.clip(dq, -0.1, 0.1)
            if np.linalg.norm(e_pos) < tol and np.linalg.norm(dq) < 1e-4:
                break
            d.qpos[self.qadr] = np.clip(d.qpos[self.qadr] + dq, self.lo, self.hi)
        mujoco.mj_forward(m, d)
        q = q.copy()
        q[:4] = d.qpos[self.qadr]
        return q, float(np.linalg.norm(target - d.site_xpos[self.site]))

    def lighter_roll(self, target_pos, q, roll_lo: float, roll_hi: float, payload_kg: float = 0.0, **kw):
        """Of wrist roll r and r +/- pi (same jaw line, camera on the other side), the one with less holding torque.

        Returns (q, err, cost) for the best candidate inside [roll_lo, roll_hi].
        """
        best = None
        for r in (q[4], q[4] + math.pi, q[4] - math.pi):
            if not (roll_lo - 1e-9 <= r <= roll_hi + 1e-9):
                continue
            seed = np.array(q, float)
            seed[4] = r
            qq, err = self.solve(target_pos, seed, **kw)
            cost = self.load_cost(qq, payload_kg)
            ok, best_ok = err < 0.003, best is not None and best[1] < 0.003
            # A candidate that reaches the target beats one that doesn't; among equals, less torque (or less error).
            if best is None or (ok and not best_ok) or (ok and cost < best[2]) or (not ok and not best_ok and err < best[1]):
                best = (qq, err, cost)
        return best

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
