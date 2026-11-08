"""Torque governor: command the fastest motion the servos can actually follow.

A fixed speed/acceleration profile (servo.ProfileLimiter) has to be set for the
worst pose and the worst load, so it's slow everywhere else, and still
saturates the shoulder at far reach. The governor replaces it for the arm
joints. Every control tick it:

  1. predicts the torque each joint needs for the next step of motion with the
     arm's dynamics model (so101sim.dynamics: inertia, gravity, Coriolis,
     damping/friction, an assumed payload);
  2. checks it against what each HX-30HM can deliver right now:
       voltage   |tau + k v| <= voltage_margin * stall   (back-EMF: less torque at speed)
       overload  |tau| above 80 % of stall only for overload_budget_s at a time
                 (the servo itself trips at 2.0 s and caps output at 20 %)
       heat      if a joint's RMS torque over heat_window_s nears the continuous
                 rating (1/3 stall), motion is slowed down
  3. caps each joint's speed so that, at that speed, it still holds gravity with
     reserve_frac of stall torque to spare (to brake and correct);
  4. scales the whole arm's acceleration by one factor so every joint stays
     inside its limits (braking is never scaled down first, so it doesn't
     overshoot).

Feed it a smooth goal: interpolate IK goals between control ticks
(servo.GoalInterpolator). A goal that jumps every tick makes any generator
that's fast enough to catch it stop and restart each tick, which rings the
servo loop.

With a measured joint position (the real servo's position register, or the
sim), it also slows down when the arm lags the command by more than
max_lag_deg: a guard against model error.

The gripper is not governed (its squeeze is a contact force, not motion): it
uses the fixed profile. Limits come from config/servos.toml [follower.governor].
"""
from __future__ import annotations

import math

import numpy as np

from .config import load_servos
from .dynamics import ArmDynamics
from .servo_model import ServoSpec

N_ARM = 5          # shoulder_pan .. wrist_roll; index 5 = gripper


class TorqueGovernor:
    def __init__(self, initial, spec: ServoSpec | None = None, cfg: dict | None = None,
                 dyn: ArmDynamics | None = None, payload_kg: float | None = None):
        f = load_servos()["follower"]
        self.cfg = dict(f["governor"], **(cfg or {}))
        self.spec = spec or ServoSpec.from_config()
        self.dyn = dyn or ArmDynamics()
        self.payload_kg = float(self.cfg["assumed_payload_kg"] if payload_kg is None else payload_kg)
        self.grip_speed = float(f["profile_speed_rad_s"])
        self.grip_accel = float(f["profile_accel_rad_s2"])
        self.current = np.array(initial, float)
        self.vel = np.zeros_like(self.current)
        self.acc = np.zeros_like(self.current)
        n = len(self.current)
        self.over_timer = np.zeros(n)
        self.heat_ms = np.zeros(n)          # running mean of tau^2 over heat_window_s
        self.tau_pred = np.zeros(n)
        self.tau_hi = np.full(n, self.spec.tau_s)   # torque available in the + / - direction now
        self.tau_lo = np.full(n, -self.spec.tau_s)
        self.scale = 1.0                    # acceleration scale applied this tick (1 = unconstrained)
        self.limiting = None                # joint index that set the scale, if any
        self.heat_scale = 1.0
        self.lag_scale = 1.0

    # ------------------------------------------------------------------ helpers
    def heat_frac(self) -> np.ndarray:
        """RMS torque / continuous rating (1/3 stall), per joint."""
        return np.sqrt(self.heat_ms) / self.spec.continuous_hi

    def load_frac(self) -> np.ndarray:
        """Predicted |torque| / stall torque at the supply voltage, per joint."""
        return np.abs(self.tau_pred) / self.spec.tau_s

    # ------------------------------------------------------------------ main
    def update(self, goal, dt: float, q_meas=None) -> np.ndarray:
        """Next joint command. q_meas: measured joint positions (enables the lag guard)."""
        c, s = self.cfg, self.spec
        n = N_ARM
        q, v = self.current, self.vel
        goal = np.asarray(goal, float)
        err = goal - q
        tau_s, k = s.tau_s, s.tau_s / s.w0
        u_max = float(c["voltage_margin"])
        h, M, g = self.dyn.split(q, v, self.payload_kg)

        # Slow-down factors: heat (any joint near its continuous rating) and lag.
        heat = float(self.heat_frac()[:n].max())
        self.heat_scale = float(np.clip((1.25 - heat) / 0.35, 0.2, 1.0))   # full below 0.9, 0.2 at 1.18+
        self.lag_scale = 1.0
        if q_meas is not None:
            lag = float(np.abs(self.current[:n] - np.asarray(q_meas, float)[:n]).max())
            lim = math.radians(float(c["max_lag_deg"]))
            if lag > lim:
                self.lag_scale = max(0.1, lim / lag)
        slow = self.heat_scale * self.lag_scale

        # Speed cap: at that speed the joint can still hold gravity with reserve.
        sigma = np.sign(err[:n])
        sigma[sigma == 0] = 1
        u_cruise = (u_max - float(c["reserve_frac"])) * tau_s
        v_cap = np.clip((u_cruise - sigma * g[:n]) / k, 0.3, float(c["speed_frac"]) * s.w0) * slow
        # Per-joint acceleration bound and the deceleration we plan braking with.
        Mjj = np.diag(M)[:n]
        a_avail = np.maximum(u_max * tau_s - np.abs(g[:n]), 0.1 * tau_s) / Mjj
        a_lim = np.clip(a_avail, 2.0, float(c["accel_max_rad_s2"])) * slow
        a_brake = 0.5 * a_lim
        # Reference: cascaded position -> velocity loop, critically damped at
        # `bandwidth_rad_s` (well below the servo's own ~45 rad/s loop, so the
        # command is smooth and doesn't ring it), the velocity demand capped by
        # v_cap and by what can still brake in time. (Feeding the streamed goal's
        # velocity forward was tried: it steps every control tick and made the
        # command jerky enough for the arm to fall off the path.)
        lam = 0.5 * float(c["bandwidth_rad_s"])
        e = np.abs(err[:n])
        v_des = sigma * np.minimum(np.minimum(lam * e, np.sqrt(2 * a_brake * e)), v_cap)
        a_des = np.zeros(len(q))
        jerk = float(c["jerk_max_rad_s3"])
        a_ref = np.clip(4 * lam * (v_des - v[:n]), -a_lim, a_lim)
        a_des[:n] = np.clip(a_ref, self.acc[:n] - jerk * dt, self.acc[:n] + jerk * dt)
        # gripper: fixed trapezoidal profile
        eg = err[n:]
        vg = np.sign(eg) * np.minimum(self.grip_speed, np.sqrt(2 * self.grip_accel * np.abs(eg)))
        a_des[n:] = np.clip((vg - v[n:]) / dt, -self.grip_accel, self.grip_accel)

        # Torque window per joint right now: voltage (back-EMF) and overload budget.
        hi = u_max * tau_s - k * v
        lo = -u_max * tau_s - k * v
        ovl = s.overload_torque
        spent = self.over_timer >= float(c["overload_budget_s"])
        hi = np.where(spent, np.minimum(hi, 0.95 * ovl), hi)
        lo = np.where(spent, np.maximum(lo, -0.95 * ovl), lo)
        self.tau_hi, self.tau_lo = hi, lo

        # One scale for the arm's acceleration; braking components first.
        braking = np.zeros(len(q), bool)
        braking[:n] = a_des[:n] * v[:n] < 0
        a_brk = np.where(braking, a_des, 0.0)
        a_acc = np.where(braking, 0.0, a_des)
        a_acc[n:] = 0
        a_brk[n:] = a_des[n:]
        base = h + M @ a_brk
        s_brk, j_brk = self._max_scale(h, M @ a_brk, lo, hi, n)
        if s_brk < 1.0:                       # even braking is too much: scale it, no acceleration
            a = a_brk * s_brk
            self.scale, self.limiting = s_brk, j_brk
            a[n:] = a_des[n:]
        else:
            s_acc, j_acc = self._max_scale(base, M @ a_acc, lo, hi, n)
            a = a_brk + s_acc * a_acc
            self.scale, self.limiting = s_acc, (j_acc if s_acc < 1.0 else None)
        self.tau_pred = h + M @ a
        self.acc = a

        # Integrate. No hard "stop at the goal" clamp (zeroing the velocity in one
        # tick is a torque spike); arrive with the jerk-limited profile and snap
        # only once both the error and the speed are tiny.
        v_new = v + a * dt
        new = q + v_new * dt
        snap = (np.abs(goal - new) < 2e-4) & (np.abs(v_new) < 0.05)
        self.current = np.where(snap, goal, new)
        self.vel = np.where(snap, 0.0, v_new)
        self.acc = np.where(snap, 0.0, self.acc)

        # Overload timer (mirrors the servo's) and heat budget.
        above = np.abs(self.tau_pred) > ovl
        self.over_timer = np.where(above, self.over_timer + dt, 0.0)
        w = min(1.0, dt / float(c["heat_window_s"]))
        self.heat_ms += (self.tau_pred ** 2 - self.heat_ms) * w
        return self.current.copy()

    @staticmethod
    def _max_scale(base, dtau, lo, hi, n):
        """Largest s in [0, 1] with lo <= base + s * dtau <= hi on the first n joints."""
        s, arg = 1.0, None
        for j in range(n):
            if dtau[j] > 1e-12:
                sj = (hi[j] - base[j]) / dtau[j]
            elif dtau[j] < -1e-12:
                sj = (lo[j] - base[j]) / dtau[j]
            else:
                continue
            if sj < s:
                s, arg = sj, j
        return float(np.clip(s, 0.0, 1.0)), arg
