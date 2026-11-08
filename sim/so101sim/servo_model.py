"""Realistic bus-servo model (Hiwonder HX-30HM) for torque-controlled simulation.

Replaces the engine's ideal position actuator with what a hobby bus servo does:

  * Position loop, PD (HX defaults P = 32, D = 32, I = 0): the commanded motor
    voltage is proportional to position error plus d_time_s * its rate, and
    saturates at the supply. No integral term, so the joint must droop to hold
    a load.
  * DC motor: torque = stall_torque * (voltage fraction) - (stall_torque /
    no_load_speed) * velocity. Available torque falls to zero at the no-load
    speed (back-EMF), and the joint can't exceed it.
  * Torque limit register (100 % by default).
  * Overload protection (HX defaults, register 34/35/36): if |torque| stays
    above 80 % of max for 2.0 s, output is capped at 20 % until the load drops.
  * Current = no-load + (stall - no-load) * |torque| / stall_torque (ideal DC
    motor: current proportional to torque).
  * Supply voltage: stall torque and no-load speed scale with supply / rated
    voltage (the datasheet figures are at 11.1 V). Current per N.m (the torque
    constant) doesn't change, so the current formula uses the rated values.
    ASSUMED: the overload threshold is relative to the RATED stall torque
    (the conservative reading: at 12 V it trips at a lower fraction of what
    the motor can deliver).

Datasheet values (docs/datasheets/hiwonder-hx-servos): stall 30 kg.cm (2.94 N.m)
@ 11.1 V, no-load 0.19 s/60 deg (5.51 rad/s) @ 11.1 V, stall current 3 A,
no-load current 0.1 A. Continuous load recommendation: 1/5 to 1/3 of stall.

Joint damping: the datasheet no-load speed already includes the motor's and
gearbox's losses, and the back-EMF term reproduces it. So the engine's joint
damping must be zero when this model drives the joints (zero_joint_damping());
upstream's 0.6 N.m.s/rad was fitted for an ideal position actuator, where it
stands in for those same losses. Counting both halves the top speed.

UNKNOWN: the position-loop stiffness (how much error gives full voltage). HX
P gain = 32 in undocumented units. It's a parameter here ("full voltage at
`sat_error_deg` of error"); measure it on the real servo (hang a known weight,
read position error) and set it. Also UNKNOWN: the derivative gain
(d_time_s). The default gives a damping ratio of ~0.5 at the shoulder;
measure a step response and set it.
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
    d_time_s: float = 0.017             # derivative time on the error rate (UNKNOWN; see module doc)
    supply_v: float = 11.1              # supply voltage; datasheet values are at rated_v
    rated_v: float = 11.1

    @classmethod
    def from_config(cls, arm: str = "follower", **kw) -> "ServoSpec":
        """HX-30HM at the supply voltage in config/servos.toml."""
        from .config import load_servos
        f = load_servos()[arm]
        return cls(stall_torque=float(f["torque_limit_nm"]), no_load_speed=float(f["max_speed_rad_s"]),
                   supply_v=float(f["supply_voltage_v"]), rated_v=float(f["rated_voltage_v"]), **kw)

    @property
    def tau_s(self) -> float:           # stall torque at the supply voltage
        return self.stall_torque * self.supply_v / self.rated_v

    @property
    def w0(self) -> float:              # no-load speed at the supply voltage
        return self.no_load_speed * self.supply_v / self.rated_v

    @property
    def overload_torque(self) -> float:  # |torque| above which the overload timer runs
        return self.overload_threshold * self.torque_limit * self.stall_torque

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
        self.prev_target = None

    def torque(self, q_target, q, qd, dt: float) -> np.ndarray:
        s = self.spec
        q_target = np.asarray(q_target, float)
        err = q_target - np.asarray(q)
        if self.prev_target is None:
            self.prev_target = q_target.copy()
        target_rate = (q_target - self.prev_target) / dt
        self.prev_target = q_target.copy()
        err_rate = target_rate - np.asarray(qd)
        db = math.radians(s.deadband_deg)
        err = np.where(np.abs(err) <= db, 0.0, err - np.sign(err) * db)
        volt = np.clip((err + s.d_time_s * err_rate) / math.radians(s.sat_error_deg), -1.0, 1.0)  # fraction of supply
        demand = s.tau_s * volt - (s.tau_s / s.w0) * np.asarray(qd)
        tmax = s.torque_limit * s.tau_s
        demand = np.clip(demand, -tmax, tmax)
        # Overload protection: the timer runs while the (uncapped) output is above
        # the threshold; at overload_time the output is capped. The cap is released
        # once the demand falls back to what the capped servo can deliver.
        above = np.abs(demand) > s.overload_torque
        self.over_timer = np.where(above & ~self.tripped, self.over_timer + dt, 0.0)
        newly = (self.over_timer >= s.overload_time) & ~self.tripped
        self.trip_count += newly
        self.tripped |= newly
        self.tripped &= np.abs(demand) >= 0.95 * s.overload_cap * s.stall_torque
        cap = np.where(self.tripped, s.overload_cap * s.stall_torque, tmax)
        tau = np.clip(demand, -cap, cap)
        self.last_current = s.no_load_current + (s.stall_current - s.no_load_current) * np.abs(tau) / s.stall_torque
        return tau


def zero_joint_damping(model, joint_names) -> None:
    """Remove engine joint damping where ServoBank drives the joints (see module doc)."""
    for name in joint_names:
        model.dof_damping[model.joint(name).dofadr[0]] = 0.0

