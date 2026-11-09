"""Command-side compensation for soft position servos: feed-forward and grip-by-offset.

A hobby bus servo is a saturating P(D) position loop: it only produces torque
in proportion to position error, reaching stall torque at a few degrees of
error (config/servos.toml [follower.position_loop] sat_error_deg). So under
gravity it droops, and at speed it lags, because it needs error just to supply
the voltage that overcomes its own back-EMF. Two fixes, applied to the command
stream before it reaches the servos (the same code drives the sim and, later,
the real arm):

CommandFeedforward
    q_target = q_cmd + (gravity_torque(q_cmd) + k_emf * qdot_cmd) / stiffness
    with stiffness = stall torque / sat_error and k_emf = stall torque /
    no-load speed. The gravity term (from so101sim.dynamics, including the
    wrist camera) removes droop; the back-EMF term removes most of the
    speed-dependent lag. In the study (docs/POLICY_TRAINING_IDEAS.md B7) this
    made the scripted pick succeed at realistic stiffness (it fails without),
    robust to +/-20 % mass error, and cut fast-teleop tip lag from 30 to 10 mm.
    It needs the stiffness to within roughly +/-20 %: measure it on the bench.

GripOffset
    Close until the gripper stalls on the object, then hold a target
    grip_offset_deg past the contact angle instead of driving "fully closed".
    Squeeze = stiffness x offset, so it stays well below stall and the servo's
    overload protection (80 % for 2 s) never trips. Opening releases the latch.
"""
from __future__ import annotations

import math

import numpy as np

from .config import position_loop
from .dynamics import ArmDynamics

N_ARM = 5


class CommandFeedforward:
    def __init__(self, dyn: ArmDynamics | None = None, cfg: dict | None = None):
        self.cfg = cfg or position_loop()
        self.dyn = dyn or ArmDynamics()
        self.stiffness = float(self.cfg["stiffness_nm_per_rad"])
        self.k_emf = float(self.cfg["back_emf_nm_s_per_rad"])
        self.gravity = bool(self.cfg.get("feedforward_gravity", True))
        self.back_emf = bool(self.cfg.get("feedforward_back_emf", True))
        self.payload_kg = float(self.cfg.get("feedforward_payload_kg", 0.0))
        self.enabled = self.gravity or self.back_emf
        self._prev = None
        self.last_offset = np.zeros(N_ARM)

    def reset(self):
        self._prev = None
        self.last_offset[:] = 0

    def apply(self, q_cmd, dt: float) -> np.ndarray:
        """Command with feed-forward offsets on the arm joints (gripper untouched)."""
        q = np.array(q_cmd, float)
        if not self.enabled:
            return q
        v = np.zeros_like(q) if self._prev is None else (q - self._prev) / dt
        self._prev = q.copy()
        tau = np.zeros(N_ARM)
        if self.gravity:
            tau += self.dyn.gravity(q, self.payload_kg)[:N_ARM]
        if self.back_emf:
            tau += self.k_emf * v[:N_ARM]
        self.last_offset = tau / self.stiffness
        out = q.copy()
        out[:N_ARM] += self.last_offset
        return out


class GripOffset:
    """Latches the gripper target a few degrees past contact once the jaws stall on an object."""

    def __init__(self, cfg: dict | None = None):
        cfg = cfg or position_loop()
        self.offset = math.radians(float(cfg["grip_offset_deg"]))
        self.stall_speed = float(cfg["grip_stall_speed_rad_s"])
        self.stall_time = float(cfg["grip_stall_time_s"])
        self.reset()

    def reset(self):
        self.latched = None          # target held after contact, or None
        self._prev_q = None
        self._stall_t = 0.0
        self._moved = False          # the jaws started moving since this close command
        self._last_cmd = None

    @property
    def holding(self) -> bool:
        return self.latched is not None

    def update(self, cmd: float, q: float, dt: float) -> float:
        """Gripper target to send, given the commanded target and the measured jaw angle."""
        v = 0.0 if self._prev_q is None or dt <= 0 else (q - self._prev_q) / dt
        self._prev_q = q
        if self._last_cmd is None or abs(cmd - self._last_cmd) > 1e-6:
            self._moved = False      # a new command: start watching again
            self._stall_t = 0.0
        self._last_cmd = cmd
        if self.latched is not None:
            if cmd >= q:             # opening (or a target above the jaws): release
                self.latched = None
            else:
                return self.latched
        closing_far = cmd < q - self.offset
        if not closing_far:
            self._stall_t = 0.0
            return cmd
        if abs(v) > self.stall_speed:
            self._moved = True
            self._stall_t = 0.0
            return cmd
        if self._moved:
            self._stall_t += dt
            if self._stall_t >= self.stall_time:
                self.latched = max(q - self.offset, cmd)
                return self.latched
        return cmd
