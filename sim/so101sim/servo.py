"""Servo-side behaviour the physics engine doesn't model on its own.

ServoMap   : real servo encoder ticks <-> sim joint radians (per-joint zero/direction).
SlewLimiter: caps how fast the commanded position moves, like a real bus servo's
             top speed. MuJoCo/Newton position actuators have no speed limit, so
             without this the sim arm can move far faster than the HX-30HM
             (~5.5 rad/s no-load) and demonstrations would not transfer.
"""
from __future__ import annotations

import math

import numpy as np

from .config import load_servos


class ServoMap:
    def __init__(self, joints, arm: str = "follower"):
        cfg = load_servos()[arm]
        self.joints = tuple(joints)
        self.ticks_per_rev = int(cfg["ticks_per_rev"])
        j = cfg["joints"]
        self.zero = np.array([j[n]["zero_ticks"] for n in self.joints], float)
        self.direction = np.array([j[n]["direction"] for n in self.joints], float)
        self.ids = tuple(int(j[n]["id"]) for n in self.joints)

    def ticks_to_rad(self, ticks) -> np.ndarray:
        return self.direction * (np.asarray(ticks, float) - self.zero) * (2 * math.pi / self.ticks_per_rev)

    def rad_to_ticks(self, rad) -> np.ndarray:
        t = self.zero + self.direction * np.asarray(rad, float) * (self.ticks_per_rev / (2 * math.pi))
        return np.clip(np.rint(t), 0, self.ticks_per_rev - 1).astype(int)


class ProfileLimiter:
    """Trapezoidal motion profile: speed AND acceleration limited, per joint.

    Like a bus servo's goal-speed + acceleration registers. Decelerates in time
    to stop at the goal (v <= sqrt(2 a |remaining|)).
    """

    def __init__(self, initial, max_speed_rad_s: float, max_accel_rad_s2: float):
        self.max_speed = float(max_speed_rad_s)
        self.max_accel = float(max_accel_rad_s2)
        self.current = np.array(initial, float)
        self.vel = np.zeros_like(self.current)

    def update(self, goal, dt: float) -> np.ndarray:
        err = np.asarray(goal, float) - self.current
        v_des = np.sign(err) * np.minimum(self.max_speed, np.sqrt(2 * self.max_accel * np.abs(err)))
        dv = np.clip(v_des - self.vel, -self.max_accel * dt, self.max_accel * dt)
        self.vel = self.vel + dv
        step = self.vel * dt
        # don't overshoot
        step = np.where(np.abs(step) > np.abs(err), err, step)
        self.current = self.current + step
        self.vel = np.where(np.abs(err) < 1e-6, 0.0, self.vel)
        return self.current.copy()


class SlewLimiter:
    """Moves a commanded position toward the goal at no more than max_speed."""

    def __init__(self, initial, max_speed_rad_s: float | None = None, arm: str = "follower"):
        if max_speed_rad_s is None:
            max_speed_rad_s = float(load_servos()[arm]["max_speed_rad_s"])
        self.max_speed = float(max_speed_rad_s)
        self.current = np.array(initial, float)

    def update(self, goal, dt: float) -> np.ndarray:
        step = self.max_speed * dt
        self.current = self.current + np.clip(np.asarray(goal, float) - self.current, -step, step)
        return self.current.copy()
