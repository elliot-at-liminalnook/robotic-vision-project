"""The arm's controller: goals in, servo commands out, with every protection the arm has.

This is the part of teleop that is not a window: Cartesian and joint goals, straight-line moves,
the torque governor (FAST) or a fixed profile (GENTLE), feed-forward, grip-by-offset, the stall
guard and the stop path. It commands the arm through `sim`, which is anything with

    q() -> joint angles [rad]      set_targets(q, intended=...)      step(n)      dt      time
    torque_off()   torque_on() -> pose now held        (optional: safety, q_fresh)

That is a simulation backend (sim/so101sim) for teleop.py, or the arm on its servo bus
(hub/arm/link.py) for the hub. The controller knows the arm, not the place it is in. One assumption
about the place remains and is a cautious starting point only: the arm stands on a surface at the
height of its own base, so the tip is never sent below MIN_TIP_Z above it.
"""
from __future__ import annotations

import math
import time

import numpy as np

from . import JOINTS
from .command import CommandFeedforward, GripOffset
from .config import load_servos, wrist_roll_range
from .dynamics import ArmDynamics
from .governor import TorqueGovernor
from .ik import ArmIK
from .motion import CartesianMove, pace_from_lag
from .servo import GoalInterpolator, ProfileLimiter
from .servo_model import ServoSpec

LIN_SPEED = {True: 0.25, False: 0.10}     # m/s tip speed when jogged, FAST / GENTLE
JOINT_SPEED = {True: 2.5, False: 1.0}     # rad/s in joint mode
ROLL_SPEED = 1.5                # rad/s
FRAME_HZ = 30                   # control/IK rate (window frame rate)
TILT_STEP, TILT_MAX = 5.0, 25.0  # deg, load-aware IK cone ([ / ])
TILT_RAMP = (0.05, 0.10)        # m: cone is 0 below 5 cm tip height, full above 10 cm
WHEEL_STEP = 0.01               # m per wheel notch (joint mode: rad)
MIN_TIP_Z = 0.004               # m above the table
IK_TOL = 0.005                  # m: a target the IK misses by more is refused
BLOCK_ERR = math.radians(12.0)  # an arm joint this far behind its command (the servo is at full torque) ...
BLOCK_TIME = 0.4                # ... for this long is blocked: by the table, an object or the arm itself
GRIP_OPEN, GRIP_CLOSED = 0.9, -0.17
HOME_TIP = (0.20, 0.0, 0.16)


class ArmController:
    def __init__(self, sim):
        self.sim = sim
        self.monitor = getattr(self, "monitor", None)   # a physics monitor, where there is one (simulation only)
        self._alerts = (0, 0)            # (monitor violation kinds, command-filter interventions) already announced
        self.ik = ArmIK()
        self.roll_lo, self.roll_hi = wrist_roll_range()
        jr = self.ik.m.jnt_range
        self.lo = np.array([jr[self.ik.m.joint(n).id][0] for n in JOINTS])
        self.hi = np.array([jr[self.ik.m.joint(n).id][1] for n in JOINTS])
        self.hi[4] = min(self.hi[4], self.roll_hi)
        self.lo[4] = max(self.lo[4], self.roll_lo)
        self.mode = "cartesian"
        self.sel_joint = 0
        self.paused = False
        self.message = "ready"
        self.message_t = 0.0
        self.fast = True                # torque governor
        self.tilt_tol = 0.0             # load-aware IK cone [deg]; 0 = strict fingers-down
        self.spec = ServoSpec.from_config()
        self.dyn = ArmDynamics()
        self.gov_cfg = load_servos()["follower"]["governor"]
        self.ff = CommandFeedforward(dyn=self.dyn)
        self.ff_on = True
        self.grip_latch = GripOffset()

    def start(self, q):
        """Begin commanding from joint angles q: the arm is there, at rest, and that is its goal."""
        q = np.clip(np.asarray(q, float), self.lo, self.hi)
        self.goal = q.copy()
        self.interp = GoalInterpolator(q, 1.0 / FRAME_HZ)
        self._make_generator(q)
        self.ff.reset()
        self.grip_latch.reset()
        self.move = None                # active straight-line CartesianMove
        self.failed_point = None
        self.last_block = None
        self._blocked_t = 0.0
        self.limp = False               # torque is off: nothing is commanded until it is back on
        self.stop_latch = False         # stopped with movement keys still down: they wait for a release
        self.target = self.tip_pos(q)
        self.roll = float(q[4])
        self.grip = float(q[5])

    def _make_generator(self, q):
        """FAST: torque governor; GENTLE: fixed trapezoidal profile (SERVO_LOAD_FINDINGS.md)."""
        q = np.asarray(q, float)
        if self.fast:
            self.gen = TorqueGovernor(q, self.spec, dyn=self.dyn)
        else:
            f = load_servos()["follower"]
            self.gen = ProfileLimiter(q, f["profile_speed_rad_s"], f["profile_accel_rad_s2"])
        self.tau_pred = np.zeros(len(JOINTS))

    def toggle_speed(self):
        cmd, vel = self.gen.current.copy(), self.gen.vel.copy()
        self.fast = not self.fast
        self._make_generator(cmd)
        self.gen.vel = vel            # continue smoothly from the current command
        self.say("FAST: torque governor" if self.fast else "GENTLE: fixed profile")

    def effective_tilt(self, z: float) -> float:
        """Tilt cone at tip height z: 0 near the table (grasps straight down), full above."""
        f = np.clip((z - TILT_RAMP[0]) / (TILT_RAMP[1] - TILT_RAMP[0]), 0.0, 1.0)
        return float(self.tilt_tol * f)

    def load_frac(self) -> np.ndarray:
        """Predicted |torque| / stall per joint (governor's, or the dynamics model's for GENTLE)."""
        if self.limp:
            return np.zeros(len(JOINTS))
        if isinstance(self.gen, TorqueGovernor):
            return self.gen.load_frac()
        return np.abs(self.tau_pred) / self.spec.tau_s

    def _roll_default(self) -> float:
        return float(np.clip(-math.pi / 2, self.roll_lo, self.roll_hi))

    def tip_pos(self, q=None) -> np.ndarray:
        import mujoco
        self.ik.d.qpos[:6] = self.sim.q() if q is None else q
        mujoco.mj_forward(self.ik.m, self.ik.d)
        return self.ik.d.site_xpos[self.ik.site].copy()

    def say(self, msg: str):
        self.message, self.message_t = msg, time.time()

    def set_cartesian(self, target, roll=None, grip=None) -> bool:
        target = np.array(target, float)
        target[2] = max(target[2], MIN_TIP_Z)
        roll = self.roll if roll is None else float(np.clip(roll, self.roll_lo, self.roll_hi))
        grip = self.grip if grip is None else grip
        seed = self.goal.copy()
        seed[4], seed[5] = roll, grip
        tilt = self.effective_tilt(target[2])
        payload = float(self.gov_cfg["assumed_payload_kg"]) if grip < 0.3 else 0.0
        if tilt > 0:
            q, err = self.ik.solve(target, seed, iters=30, tilt_tol_deg=tilt, payload_kg=payload)
        else:
            q, err = self.ik.solve(target, seed, iters=120)
        if not (err <= IK_TOL and np.isfinite(q).all()):     # also catches NaN
            self.say(f"unreachable ({err * 1000:.0f} mm off)")
            return False
        self.goal = np.clip(q, self.lo, self.hi)
        self.target, self.roll, self.grip = target, roll, grip
        return True

    def jog_joint(self, j: int, delta: float):
        self.move = None
        g = self.goal.copy()
        g[j] = np.clip(g[j] + delta, self.lo[j], self.hi[j])
        z = self.tip_pos(g)[2]
        if z < MIN_TIP_Z and z < self.tip_pos(self.goal)[2]:    # may still back out from below
            self.say("refused: the tip would go into the table")
            return
        self.goal = g
        # Keep the Cartesian target in sync, so switching modes doesn't jump.
        self.target, self.roll, self.grip = self.tip_pos(g), g[4], g[5]

    def toggle_grip(self):
        new = GRIP_CLOSED if self.grip > 0.3 else GRIP_OPEN
        if self.mode == "joint":
            self.goal[5] = new
            self.grip = new
        else:
            self.set_cartesian(self.target, grip=new)
        self.say("closing gripper" if new < 0.3 else "opening gripper")

    def set_mode(self, mode: str):
        if mode != self.mode:
            self.move = None            # a straight-line move never outlives a mode switch
            self.mode = mode
            self.say(f"{mode} mode")

    def move_to(self, end, msg: str, roll=None, speed=None):
        """Start a straight-line tip move (click-to-move, home). Always runs in Cartesian mode. speed: no faster than this."""
        self.set_mode("cartesian")
        end = np.array(end, float)
        end[2] = max(end[2], MIN_TIP_Z)
        q, err = self.ik.solve(end, self.goal, iters=200)      # check the end is reachable first
        if not (err <= IK_TOL and np.isfinite(q).all()):
            self.say(f"unreachable ({err * 1000:.0f} mm off)")
            return False
        g = self.gov_cfg
        v = float(g["click_speed_m_s"]) if self.fast else LIN_SPEED[False] * 2
        if speed is not None:
            v = min(v, float(speed))
        self.move = CartesianMove(self.target, end, v, float(g["click_accel_m_s2"]))
        self.move_roll = roll
        self.say(msg)
        return True

    def advance_move(self, dt: float):
        if self.move is None:
            return
        pace = pace_from_lag(self.goal, self.sim.q(), float(self.gov_cfg["pace_lag_rad"]))
        p = self.move.step(dt, pace)
        roll = None
        if self.move_roll is not None:       # roll eases toward its target along the move
            a = self.move.s / max(self.move.length, 1e-9)
            roll = self.roll + (self.move_roll - self.roll) * min(1.0, a + 0.05)
        if not self.set_cartesian(p, roll=roll):
            self.failed_point = p            # the point on the line it could not find joint angles for
            self.move = None
            return
        if self.move.done:
            if self.move_roll is not None:
                self.set_cartesian(p, roll=self.move_roll)
            self.move = None

    def hold_here(self, q_meas, grip=None):
        """Make the pose the arm is in the goal, and start the command again from it, at rest.
        The gripper keeps its command (a held object stays held) unless grip is given."""
        self.move = None
        self._blocked_t = 0.0
        hold = self.goal.copy()
        hold[:5] = q_meas[:5]
        if grip is not None:
            hold[5] = grip
        self.goal = np.clip(hold, self.lo, self.hi)
        self.interp = GoalInterpolator(self.goal, 1.0 / FRAME_HZ)
        self._make_generator(np.r_[self.goal[:5], self.gen.current[5] if grip is None else self.goal[5]])
        self.ff.reset()
        self.target, self.roll = self.tip_pos(self.goal), self.goal[4]

    def freeze(self):
        """STOP (X): drop the move under way and hold where the arm is. Torque stays on, so nothing drops."""
        if self.limp:
            self.say("torque is off (T switches it on)")
            return
        self.hold_here(self.sim.q())
        self.stop_latch = True
        self.say("STOPPED: holding here")

    def set_torque(self, on: bool) -> bool:
        """Torque off: the arm goes limp. On: it holds the pose it is in by then. True if it was done."""
        if not hasattr(self.sim, "torque_off"):
            self.say(f"the {self.sim.name} backend has no torque switch yet")
            return False
        if on:
            try:
                q = self.sim.torque_on()
            except Exception as e:                   # no fresh encoder reading, or a servo did not confirm
                self.say(f"torque stays OFF: {e}")
                return False
            self.limp, self.stop_latch = False, True
            self.grip_latch.reset()
            self.hold_here(q, grip=q[5])
            self.grip = GRIP_OPEN if q[5] > 0.3 else GRIP_CLOSED
            self.say("torque on: holding here")
            return True
        self.limp, self.move = True, None            # first: from here on this controller commands nothing
        self.grip_latch.reset()
        try:
            self.sim.torque_off()
        except Exception as e:
            self.say(f"TORQUE OFF NOT CONFIRMED, cut the power: {e}")
            return False
        self.say("torque OFF: the arm is limp (T switches it on)")
        return True

    def shutdown(self) -> str:
        """Leave the servos with torque off. For every way out of the program; never raises."""
        if not hasattr(self.sim, "torque_off"):
            return f"the {self.sim.name} backend has no torque switch yet"
        try:
            self.limp, self.move = True, None
            self.sim.torque_off()
            return "torque off on every servo"
        except Exception as e:
            return f"TORQUE OFF NOT CONFIRMED, cut the power: {e}"

    def physics(self, dt_wall: float):
        if self.paused:
            return
        if self.limp:                    # torque off: time passes, the arm sags, nothing is commanded
            self.sim.step(max(1, int(round(min(dt_wall, 0.1) / self.sim.dt))))
            return
        dt = self.sim.dt
        n = max(1, int(round(min(dt_wall, 0.1) / dt)))
        frame_dt = n * dt
        self.interp.set(self.goal)
        q_meas = self.sim.q()              # once per frame (the real arm: position registers)
        # Over the servo bus a read can fail; q() then repeats the last good reading. An old reading says
        # nothing about whether the arm is following, so the stall guard and the lag guard wait for a new one.
        fresh = getattr(self.sim, "q_fresh", True)
        if fresh and self._blocked(q_meas, frame_dt):
            return
        governed = isinstance(self.gen, TorqueGovernor)
        if governed and fresh:
            self.gen.observe(q_meas)       # lag vs the command as it is now, not as it advances below
        # Grip-by-offset: once the jaws stall on an object, hold a few degrees past contact.
        latched = self.grip_latch.update(float(self.gen.current[5]), float(q_meas[5]), frame_dt)
        if self.grip_latch.holding and self.gen.current[5] < latched:
            # Park the gripper's command generator at the held target, so opening starts from there.
            self.gen.current[5], self.gen.vel[5] = latched, 0.0
        vel0 = self.gen.vel.copy()
        for _ in range(n):
            g = self.interp.advance(dt)
            cmd = np.array(self.gen.update(g, dt), float)
            if self.grip_latch.holding:
                cmd[5] = max(cmd[5], latched)
            ref = cmd                      # where the arm is meant to be; the command below is shifted to get it there
            if self.ff_on:
                cmd = self.ff.apply(cmd, dt)
            self.sim.set_targets(cmd, intended=ref)
            self.sim.step(1)
        safety = getattr(self.sim, "safety", None)
        seen = (len(self.monitor.kinds) if self.monitor else 0, sum(safety.started.values()) if safety else 0)
        if seen[0] > self._alerts[0]:
            self.say("physics check: " + self.monitor.violations[-1].split(": ", 1)[1])
        elif seen[1] > self._alerts[1]:
            self.say("command filter: " + safety.describe(safety.events[-1]))
        self._alerts = seen
        if not governed:                   # GENTLE: predicted torque for the load panel
            self.tau_pred = self.dyn.torque(self.gen.current, self.gen.vel, (self.gen.vel - vel0) / frame_dt,
                                            float(self.gov_cfg["assumed_payload_kg"]))

    def _blocked(self, q_meas, frame_dt: float) -> bool:
        """Stall guard: if the arm can't follow its command, stop pushing.

        A servo more than its saturation error (9 deg) behind is already at
        full torque; if it stays there, something is in the way (the tip-height
        limit only covers the tip: the jaws can meet the shoulder, the camera
        the table). Holding that would trip the servo's overload protection, so
        the goal is pulled back to where the arm is.
        """
        behind = np.abs(self.gen.current[:5] - q_meas[:5])
        self._blocked_t = self._blocked_t + frame_dt if behind.max() > BLOCK_ERR else 0.0
        if self._blocked_t < BLOCK_TIME:
            return False
        j = int(behind.argmax())
        self.last_block = {"joint": JOINTS[j], "behind_rad": float(behind[j]), "seconds": float(self._blocked_t),
                           "load": float(self.load_frac()[j])}
        self.hold_here(q_meas)
        self.say(f"blocked at {JOINTS[j]}: stopped pushing")
        return True
