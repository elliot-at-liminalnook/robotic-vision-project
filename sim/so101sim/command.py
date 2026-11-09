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
    Contact is detected from the measured jaw alone (commanded well past the
    jaw, yet the jaw isn't moving), so it works the same for a stepped, ramped
    or streamed command.
"""
from __future__ import annotations

import math

import numpy as np

from .config import position_loop, servo_limits
from .dynamics import ArmDynamics

N_ARM = 5


class CommandFeedforward:
    def __init__(self, dyn: ArmDynamics | None = None, cfg: dict | None = None):
        self.cfg = cfg or position_loop()
        self.dyn = dyn or ArmDynamics()
        self.stiffness = float(self.cfg["stiffness_nm_per_rad"])
        self.k_emf = float(self.cfg["back_emf_nm_s_per_rad"])
        self.v_max = servo_limits()["no_load_speed_rad_s"]
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
        # A servo can't turn faster than its no-load speed, so a faster command
        # (a step) gets no extra back-EMF offset: unclamped, one 0.5 rad step in
        # a 2 ms tick would add a 380 deg offset for that tick.
        v = np.clip(v, -self.v_max, self.v_max)
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
    """Latches the gripper target a few degrees past contact once the jaws stall on an object.

    State machine, driven only by the commanded target and the measured jaw:
      free     send the command. If it is more than `offset` past the jaw
               (closing hard) while the jaw is slower than stall_speed for
               stall_time, the jaws are on something: latch.
      latched  send contact - offset. For settle_time after latching, the
               contact angle is re-measured as the squeeze relaxes from the
               closing torque to the holding torque (the object and the pads
               spring back a little; without this the hold is ~20 % harder
               than intended). Released when the command opens past the
               latched target, or when the jaw reaches the latched target
               (the object is gone: nothing is holding the jaw back).
    """

    LOST_FRAC = 0.3      # jaw within this fraction of `offset` of the target = nothing in the jaws

    def __init__(self, cfg: dict | None = None):
        cfg = cfg or position_loop()
        self.offset = math.radians(float(cfg["grip_offset_deg"]))
        self.stall_speed = float(cfg["grip_stall_speed_rad_s"])
        self.stall_time = float(cfg["grip_stall_time_s"])
        self.settle_time = float(cfg.get("grip_settle_time_s", 0.3))
        self.reset()

    def reset(self):
        self.latched = None          # target held after contact, or None
        self._prev_q = None
        self._stall_t = 0.0
        self._settle_t = 0.0

    @property
    def holding(self) -> bool:
        return self.latched is not None

    def update(self, cmd: float, q: float, dt: float) -> float:
        """Gripper target to send, given the commanded target and the measured jaw angle."""
        v = 0.0 if self._prev_q is None or dt <= 0 else (q - self._prev_q) / dt
        self._prev_q = q
        if self.latched is not None:
            if cmd > self.latched:                               # opening: follow the command
                self.latched = None
            elif q < self.latched + self.LOST_FRAC * self.offset:  # object gone
                self.latched = None
            else:
                if self._settle_t < self.settle_time:            # contact relaxing: re-measure it
                    self._settle_t += dt
                    self.latched = max(self.latched, q - self.offset)
                return self.latched
            self._stall_t = 0.0
        if cmd >= q - self.offset or abs(v) > self.stall_speed:  # not closing hard, or still moving
            self._stall_t = 0.0
            return cmd
        self._stall_t += dt
        if self._stall_t >= self.stall_time:
            self.latched = max(q - self.offset, cmd)
            self._settle_t = 0.0
            return self.latched
        return cmd
