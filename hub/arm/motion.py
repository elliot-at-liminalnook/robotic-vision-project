"""Straight-line Cartesian moves for click-to-move.

Jumping the joint goal straight to a far target lets every joint interpolate
on its own, so the tip doesn't travel in a line: from a low pose it can dive
into the table on the way (found in sim/scripts/fast_teleop_study.py: the
gripper stalled against the table at 100 % torque until overload tripped).

CartesianMove moves the tip *target* along the straight line with a
trapezoidal speed profile; the caller solves IK each control tick and feeds
the motion generator (governor or fixed profile). `pace` (0..1) lets the
caller slow the path down when the arm falls behind it, so the joint goal
never runs far ahead of the command and the path is followed, not cut.
"""
from __future__ import annotations

import numpy as np


class CartesianMove:
    def __init__(self, start, end, v_max: float, a_max: float):
        self.start = np.asarray(start, float)
        self.end = np.asarray(end, float)
        self.length = float(np.linalg.norm(self.end - self.start))
        self.v_max, self.a_max = float(v_max), float(a_max)
        self.s = 0.0
        self.v = 0.0

    @property
    def done(self) -> bool:
        return self.s >= self.length - 1e-9

    def step(self, dt: float, pace: float = 1.0) -> np.ndarray:
        """Advance by dt and return the next tip target."""
        if self.length < 1e-9:
            self.s = self.length
            return self.end.copy()
        remaining = self.length - self.s
        v_des = min(self.v_max * float(np.clip(pace, 0.0, 1.0)), (2 * self.a_max * remaining) ** 0.5)
        self.v = float(np.clip(v_des, self.v - self.a_max * dt, self.v + self.a_max * dt))
        self.v = max(self.v, 0.0)
        self.s = min(self.length, self.s + self.v * dt)
        if remaining < 1e-4:
            self.s = self.length
        return self.start + (self.end - self.start) * (self.s / self.length)


def pace_from_lag(goal, measured, lag_ref: float = 0.4) -> float:
    """1 while the arm is within lag_ref/2 rad of the goal, falling to 0 at 1.5 * lag_ref.

    Pass the MEASURED joint positions (sim state, or the real servos' position
    registers), not the motion generator's command: if different joints lag by
    different amounts the real tip leaves the straight line (in the study it
    dipped into the table), so the path waits for the real arm.
    """
    lag = float(np.abs(np.asarray(goal)[:5] - np.asarray(measured)[:5]).max())
    return float(np.clip(1.5 - lag / lag_ref, 0.0, 1.0))
