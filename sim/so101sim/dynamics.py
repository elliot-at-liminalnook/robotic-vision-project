"""Arm inverse dynamics: the joint torques a motion needs, from the robot model.

Shared by the torque governor (so101sim/governor.py), load-aware IK
(so101sim/ik.py) and the teleop load display. Uses the generated follower
model (sim/model/so101_follower.xml: link masses + the wrist camera assembly),
so it works the same for the MuJoCo and Newton backends and for the real arm.

    tau = M(q) a + c(q, v) + g(q) + damping * v + friction * sign(v) - J(q)^T F_payload

Everything the joint's own servo must supply. Only kinematics/inertia
routines are run (no collision detection), so one call costs ~0.1 ms.
"""
from __future__ import annotations

import mujoco
import numpy as np

from . import JOINTS
from .config import MODEL_DIR

G = 9.81


class ArmDynamics:
    def __init__(self, model_path=MODEL_DIR / "so101_follower.xml", joint_damping: float | None = 0.0):
        """joint_damping: override the model's joint damping. Default 0: the real
        servo's losses are in its torque-speed line (servo_model module doc), not
        in the joint; None keeps the model's (upstream 0.6, for ideal actuators)."""
        self.m = mujoco.MjModel.from_xml_path(str(model_path))
        self.d = mujoco.MjData(self.m)
        self.qadr = np.array([self.m.joint(n).qposadr[0] for n in JOINTS])
        self.dof = np.array([self.m.joint(n).dofadr[0] for n in JOINTS])
        self.damping = self.m.dof_damping[self.dof].copy()
        if joint_damping is not None:
            self.damping[:] = joint_damping
        self.friction = self.m.dof_frictionloss[self.dof].copy()
        self.site = self.m.site("gripperframe").id
        self._jacp = np.zeros((3, self.m.nv))
        self._out = np.zeros(self.m.nv)
        self._M = np.zeros((self.m.nv, self.m.nv))

    def _set(self, q, v):
        d = self.d
        d.qpos[self.qadr] = q
        d.qvel[:] = 0
        d.qvel[self.dof] = v
        mujoco.mj_kinematics(self.m, d)
        mujoco.mj_comPos(self.m, d)
        mujoco.mj_comVel(self.m, d)

    def _payload(self, payload_kg: float) -> np.ndarray:
        if payload_kg <= 0:
            return 0.0
        mujoco.mj_jacSite(self.m, self.d, self._jacp, None, self.site)
        return self._jacp[:, self.dof].T @ np.array([0.0, 0.0, payload_kg * G])

    def split(self, q, v, payload_kg: float = 0.0):
        """(h, M, g): velocity+gravity+passive torque h (zero accel), mass matrix M, gravity-only g."""
        self._set(q, np.zeros(len(JOINTS)))
        mujoco.mj_rne(self.m, self.d, 0, self._out)
        pay = self._payload(payload_kg)
        g = self._out[self.dof] + pay
        self._set(q, v)
        mujoco.mj_rne(self.m, self.d, 0, self._out)
        v = np.asarray(v, float)
        h = self._out[self.dof] + pay + self.damping * v + self.friction * np.tanh(v / 0.02)
        mujoco.mj_crb(self.m, self.d)
        try:                                   # MuJoCo >= 3.10: (m, d, dst)
            mujoco.mj_fullM(self.m, self.d, self._M)
        except TypeError:                      # older: (m, dst, qM)
            mujoco.mj_fullM(self.m, self._M, self.d.qM)
        M = self._M[np.ix_(self.dof, self.dof)].copy()
        return h, M, g

    def gravity(self, q, payload_kg: float = 0.0) -> np.ndarray:
        """Holding torque at rest (gravity + payload) per joint [N.m]."""
        self._set(q, np.zeros(len(JOINTS)))
        mujoco.mj_rne(self.m, self.d, 0, self._out)
        return self._out[self.dof] + self._payload(payload_kg)

    def torque(self, q, v, a, payload_kg: float = 0.0) -> np.ndarray:
        h, M, _ = self.split(q, v, payload_kg)
        return h + M @ np.asarray(a, float)
