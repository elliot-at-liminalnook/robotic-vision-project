"""Realistic bus-servo model (Hiwonder HX-30HM) for torque-controlled simulation.

Replaces the engine's ideal position actuator with what a hobby bus servo does:

  * Position loop, P only (HX default I = 0): the commanded motor voltage is
    proportional to position error and saturates at the supply. No integral
    term, so the joint must droop to hold a load.
  * DC motor: torque = stall_torque * (voltage fraction) - (stall_torque /
    no_load_speed) * velocity. Available torque falls to zero at the no-load
    speed (back-EMF), and the joint can't exceed it.
  * Torque limit register (100 % by default).
  * Overload protection (HX defaults, register 34/35/36): if |torque| stays
    above 80 % of max for 2.0 s, output is capped at 20 % until the load drops.
  * Current = no-load + (stall - no-load) * |torque| / stall_torque (ideal DC
    motor: current proportional to torque).

Datasheet values (docs/datasheets/hiwonder-hx-servos): stall 30 kg.cm (2.94 N.m)
@ 11.1 V, no-load 0.19 s/60 deg (5.51 rad/s) @ 11.1 V, stall current 3 A,
no-load current 0.1 A. Continuous load recommendation: 1/5 to 1/3 of stall.

UNKNOWN: the position-loop stiffness (how much error gives full voltage). HX
P gain = 32 in undocumented units. It's a parameter here ("full voltage at
`sat_error_deg` of error"); measure it on the real servo (hang a known weight,
read position error) and set it.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class ServoSpec:
    stall_torque: float = 2.94          # N.m @ 11.1 V
    no_load_speed: float = 5.51         # rad/s @ 11.1 V
    stall_current: float = 3.0          # A
    no_load_current: float = 0.10       # A
    torque_limit: float = 1.0           # register 16 (fraction of stall)
    overload_threshold: float = 0.80    # register 36 (fraction of max torque)
    overload_time: float = 2.0          # register 35 (s)
    overload_cap: float = 0.20          # register 34 (fraction of max torque once tripped)
    sat_error_deg: float = 2.0          # position error that gives full voltage (UNKNOWN; see module doc)
    deadband_deg: float = 0.088         # 1 encoder step (registers 26/27)

    @property
    def continuous_hi(self) -> float:   # manual: load 1/5 .. 1/3 of stall
        return self.stall_torque / 3

    @property
    def continuous_lo(self) -> float:
        return self.stall_torque / 5


@dataclass
class ServoBank:
    """N independent servos. Call torque() every physics step."""
    n: int
    spec: ServoSpec = field(default_factory=ServoSpec)

    def __post_init__(self):
        self.over_timer = np.zeros(self.n)
        self.tripped = np.zeros(self.n, bool)
        self.trip_count = np.zeros(self.n, int)
        self.last_current = np.zeros(self.n)

    def torque(self, q_target, q, qd, dt: float) -> np.ndarray:
        s = self.spec
        err = np.asarray(q_target) - np.asarray(q)
        db = math.radians(s.deadband_deg)
        err = np.where(np.abs(err) <= db, 0.0, err - np.sign(err) * db)
        volt = np.clip(err / math.radians(s.sat_error_deg), -1.0, 1.0)     # fraction of supply
        demand = s.stall_torque * volt - (s.stall_torque / s.no_load_speed) * np.asarray(qd)
        demand = np.clip(demand, -s.torque_limit * s.stall_torque, s.torque_limit * s.stall_torque)
        # Overload protection: the timer runs while the (uncapped) output is above
        # the threshold; at overload_time the output is capped. The cap is released
        # once the demand falls back to what the capped servo can deliver.
        above = np.abs(demand) > s.overload_threshold * s.torque_limit * s.stall_torque
        self.over_timer = np.where(above & ~self.tripped, self.over_timer + dt, 0.0)
        newly = (self.over_timer >= s.overload_time) & ~self.tripped
        self.trip_count += newly
        self.tripped |= newly
        self.tripped &= np.abs(demand) >= 0.95 * s.overload_cap * s.stall_torque
        cap = np.where(self.tripped, s.overload_cap, s.torque_limit) * s.stall_torque
        tau = np.clip(demand, -cap, cap)
        self.last_current = s.no_load_current + (s.stall_current - s.no_load_current) * np.abs(tau) / s.stall_torque
        return tau
