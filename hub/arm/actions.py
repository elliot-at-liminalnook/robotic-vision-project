"""The arm's actions, at the level a person thinks: move the tip, move joints, set the gripper, wait.

Each action runs the arm's controller (controller.py) until it is done or its timeout passes, then
answers with what happened: whether it got there, how far off, what the command filter did, and why
not if not. Time passes only inside an action.

"Tip" here is the point midway between the two jaw pads at the height of the jaw tips; the
controller's own tip is on the fixed jaw's pad. Coordinates are in the arm's own frame: origin at
its base on the surface it stands on, x forward, y to its left, z up.
"""
from __future__ import annotations

import math

import numpy as np

from ..errors import Refused
from . import JOINTS
from . import controller as C
from . import grasp as G

FRAME = 1.0 / C.FRAME_HZ
CAUTIOUS_SPEED = 0.08           # m/s: the tip's speed where nothing has looked, or where it ends at a thing
CAUTIOUS_LAG_M = 0.012          # ... and it has met something if its command went this far in PROBE_WINDOW_S and it did not follow
PROBE_SPEED = 0.03              # m/s: the tip's speed going down in a probe
PROBE_WINDOW_S = 0.3            # the tip has stopped if over this long ...
PROBE_LAG_M = 0.005             # ... its command went down this far and it did not follow
PROBE_SURFACE_M = 0.006         # a hand whose lowest point stopped lower than this is on the surface, not on a thing


class ArmActions:
    def __init__(self, app: C.ArmController):
        self.app = app
        self.on_frame = None             # called after every control frame (a window, if there is one)
        ik = app.ik
        face = G.fixed_pad_face(ik)                      # (leaves the IK model at its zero pose)
        g = ik.gripper
        site_local = ik.d.xmat[g].reshape(3, 3).T @ (ik.d.site_xpos[ik.site] - ik.d.xpos[g])
        self._pad_offset = face - site_local             # fixed jaw's pad face relative to the IK's tip site, gripper frame
        self.max_gap = float(ik.jaw_gap(C.GRIP_OPEN))
        self._goal_point = None                          # where the last move_tip was asked to put the tip
        self._tick0, self._timeout = 0, 0.0
        self.guard = None                                # set by the hub: what the map says of a path of the hand (hub/guard.py)
        self._cautious = False                           # this move is slow and stops at the first touch

    def state(self) -> dict:
        """What the arm knows about itself now, from its encoders and its own controller."""
        app, sim = self.app, self.app.sim
        q = sim.q()
        tip = self.tip(q)
        safety = getattr(sim, "safety", None)
        return {
            "ok": True,
            "time_s": round(float(sim.time), 3),
            "joints_rad": {j: round(float(v), 4) for j, v in zip(JOINTS, q)},
            "joint_limits_rad": {j: [round(float(a), 4), round(float(b), 4)] for j, a, b in zip(JOINTS, app.lo, app.hi)},
            "tip_m": {k: round(float(v), 4) for k, v in zip("xyz", tip)},
            "wrist_roll_rad": round(float(q[4]), 4),
            "jaws_close_along_deg": self._jaw_axis_deg(q),
            "moving": app.move is not None,
            "torque": not app.limp,
            "gripper": {"opening": round(self._opening(float(q[5])), 3), "gap_m": round(float(app.ik.jaw_gap(float(q[5]))), 4),
                        "max_gap_m": round(self.max_gap, 4), "angle_rad": round(float(q[5]), 4),
                        "target_opening": round(self._opening(float(app.grip)), 3), "holding": bool(app.grip_latch.holding)},
            "servo_load_fraction_of_stall": {j: round(float(v), 3) for j, v in zip(JOINTS, app.load_frac())},
            "message": app.message,
            "checks": {"command_filter": {k: {"times": int(safety.started[k]), "seconds": round(v * safety._dt, 3)}
                                          for k, v in safety.interventions.items()} if safety is not None else {},
                       "command_filter_recent": [safety.describe(e) for e in list(safety.events)[-3:]] if safety is not None else []},
        }

    def _frame(self):
        self.app.advance_move(FRAME)
        self.app.physics(FRAME)
        if self.on_frame is not None:
            self.on_frame()

    def _run(self, timeout_s: float, done) -> tuple[bool, float]:
        """Simulate until done() has held for a few frames (the arm at rest where it should be) or the timeout."""
        app, t0 = self.app, self.app.sim.time
        self._tick0 = self._filter_tick()
        self._timeout = timeout_s
        app.failed_point = app.last_block = None
        app.paused = False               # (hands-on: a person may have paused it; an action needs time to pass)
        prev, still = app.sim.q(), 0
        window = max(3, int(round(PROBE_WINDOW_S / FRAME)))
        told, went = [np.array(app.target, float)], [app.tip_pos(prev)]
        for _ in range(max(1, int(round(timeout_s / FRAME)))):
            self._frame()
            q = app.sim.q()
            if self._cautious and app.move is not None:
                # Going where nothing has looked: stop at the first touch. The tip has met something if its command has
                # gone on over the last PROBE_WINDOW_S and it has not followed (long before the stall guard would say so).
                told.append(np.array(app.target, float))
                went.append(app.tip_pos(q))
                if (len(told) > window and np.linalg.norm(went[-1] - went[0]) > 0.004
                        and np.linalg.norm(told[-1] - told[-window]) > CAUTIOUS_LAG_M and np.linalg.norm(went[-1] - went[-window]) < 0.002):
                    app.hold_here(q)
                    app.last_block = {"joint": "the tip", "behind_rad": 0.0, "seconds": PROBE_WINDOW_S,
                                      "load": float(np.max(app.load_frac())), "gentle": True}
                    return False, app.sim.time - t0
            still = still + 1 if app.move is None and np.abs(q - prev).max() < 0.002 and done() else 0
            prev = q
            if still >= 4:
                return True, app.sim.time - t0
        return False, app.sim.time - t0

    def _mid(self, q, gap: float) -> np.ndarray:
        """From the controller's tip (on the fixed jaw's pad) to the point midway between the pads, world frame."""
        import mujoco
        ik = self.app.ik
        ik.d.qpos[:6] = q
        mujoco.mj_forward(ik.m, ik.d)
        off = self._pad_offset + np.array([0.5 * gap, 0.0, 0.0])
        off[2] = 0.0                                     # keep the jaw tips' height
        return ik.d.xmat[ik.gripper].reshape(3, 3) @ off

    def tip(self, q=None) -> np.ndarray:
        """The point midway between the jaw pads, at the jaw tips' height."""
        q = self.app.sim.q() if q is None else np.asarray(q, float)
        return self.app.tip_pos(q) + self._mid(q, float(self.app.ik.jaw_gap(float(q[5]))))

    def _jaw_axis_deg(self, q) -> float:
        """Direction the jaws close along, in the table's plane: 0 = along x, 90 = along y (0 to 180)."""
        import mujoco
        ik = self.app.ik
        ik.d.qpos[:6] = q
        mujoco.mj_forward(ik.m, ik.d)
        x = ik.d.xmat[ik.gripper].reshape(3, 3)[:, 0]
        return round(float(np.degrees(np.arctan2(x[1], x[0])) % 180.0), 1)

    def _opening(self, angle: float) -> float:
        return float(np.clip((angle - C.GRIP_CLOSED) / (C.GRIP_OPEN - C.GRIP_CLOSED), 0.0, 1.0))

    def _filter_tick(self) -> int:
        safety = getattr(self.app.sim, "safety", None)
        return safety.tick if safety is not None else 0

    def _limited(self) -> list[str]:
        """What the command filter did during this action, each as a sentence with its numbers."""
        safety = getattr(self.app.sim, "safety", None)
        return [] if safety is None else [safety.describe(e) for e in safety.events_since(self._tick0)]

    def _action(self, reached: bool, elapsed: float, extra: dict | None = None) -> dict:
        app = self.app
        q = app.sim.q()
        want = self._goal_point if self._goal_point is not None else app.target + self._mid(app.goal, float(app.ik.jaw_gap(float(q[5]))))
        err = self.tip(q) - want
        out = {"ok": True, "reached": bool(reached), "elapsed_s": round(float(elapsed), 3),
               "tip_error_m": round(float(np.linalg.norm(err)), 4), "message": app.message}
        if not reached:
            out["why_not_reached"] = self._why_not_reached(err, q)
        return {**out, "limited": self._limited(), **(extra or {}), "state": self.state()}

    def _why_not_reached(self, err, q) -> str:
        """Why an action ended without getting there, from what the controller knows: its own stall guard, the
        point it could not solve, the command filter's events, the joint angles and the servo loads."""
        app = self.app
        mm = lambda v: [round(float(x) * 1000, 1) for x in v]       # noqa: E731
        if app.last_block is not None and app.last_block.get("gentle"):
            return (f"The tip met something and the arm stopped at the first touch: its command went on for {PROBE_WINDOW_S} s and "
                    f"the tip did not follow. It holds where it is, {np.linalg.norm(err) * 1000:.0f} mm short (x, y, z: {mm(-err)} mm "
                    "still to go). What it met is at the tip, or against the hand or the arm behind it.")
        if app.last_block is not None:
            b = app.last_block
            return (f"The arm stopped itself: {b['joint']} stayed {math.degrees(b['behind_rad']):.1f} deg behind its command for "
                    f"{b['seconds']:.2f} s (its servo at {b['load'] * 100:.0f}% of stall torque), which means something is in the "
                    f"arm's way or the load is too much. It gave up the move and holds where it is, {np.linalg.norm(err) * 1000:.0f} mm "
                    f"short (x, y, z: {mm(-err)} mm still to go).")
        if app.failed_point is not None:
            return ("Part-way along the line the controller could not find joint angles for the next point, and stopped there. "
                    + self._why_unreachable(app.failed_point + self._mid(app.goal, float(app.ik.jaw_gap(float(q[5])))), app.goal.copy()))
        safety = getattr(app.sim, "safety", None)
        held = [e["reason"] for e in (safety.events_since(self._tick0) if safety else []) if e["reason"] in ("table", "base")]
        lag = np.abs(app.goal[:5] - q[:5])
        j = int(lag.argmax())
        where = (f"The tip is {np.linalg.norm(err) * 1000:.1f} mm from the point (x, y, z: {mm(-err)} mm still to go); "
                 f"{JOINTS[j]} is {math.degrees(lag[j]):.1f} deg from its target, its servo at {app.load_frac()[j] * 100:.0f}% of stall torque.")
        if held:
            return (f"The command filter held the arm back at the {' and the '.join(sorted(set(held)))} (see limited). " + where)
        if app.move is not None:
            return f"The {self._timeout:g} s allowed (timeout_s) ran out while the arm was still on its way. " + where
        return ("The arm came to rest outside the tolerance (soft servos sag under load, and a part resting against "
                "something is not pushed further). " + where)

    def _solve_tip(self, point, q0, rounds: int = 4, roll_fn=None, iters: int = 150):
        """Joint angles that put the tip (the middle between the pads) on a point, and how far off they are."""
        ik = self.app.ik
        gap = float(ik.jaw_gap(float(self.app.sim.q()[5])))
        q, err = np.array(q0, float), np.inf
        for k in range(rounds):
            q, err = ik.solve(point - self._mid(q, gap), q, iters=iters)
            if k == 1 and roll_fn is not None:
                q = roll_fn(q)
        return q, err

    def _why_unreachable(self, point, q=None) -> str:
        """Why the tip cannot be put on a point (fingers down), worked out from the arm's own geometry: the
        nearest it gets, which joints are at their limits there, and what can be reached instead.
        q: the joint angles the controller ended with when it tried (default: try now, from the arm's target)."""
        app = self.app
        point = np.asarray(point, float)
        if q is None:
            q, _ = self._solve_tip(point, app.goal)
        best = self.tip(np.r_[q[:5], app.sim.q()[5]])
        short = point - best
        at = []
        for j in range(5):
            for lim, word in ((app.lo[j], "lower"), (app.hi[j], "upper")):
                if abs(q[j] - lim) < math.radians(0.5):
                    at.append(f"{JOINTS[j]} at its {word} limit ({math.degrees(lim):.0f} deg)")
        r, bearing = float(np.hypot(point[0], point[1])), math.degrees(math.atan2(point[1], point[0]))
        text = (f"The nearest the tip gets to ({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f}) is ({best[0]:.3f}, {best[1]:.3f}, "
                f"{best[2]:.3f}): {np.linalg.norm(short) * 1000:.0f} mm short (x, y, z: "
                f"{[round(float(v) * 1000) for v in short]} mm). ")
        text += ("There the arm has " + ", ".join(at) + ". ") if at else "No joint is at a limit there: the arm is simply at full stretch or folded as far as it goes. "
        pan = math.degrees(max(abs(app.lo[0]), abs(app.hi[0])))
        if abs(bearing) > pan:
            text += f"The point is {abs(bearing):.0f} deg round from straight ahead and the base only turns {pan:.0f} deg either way. "
        u = point[:2] / max(r, 1e-9)

        def ok(rr, zz):
            return self._solve_tip(np.array([u[0] * rr, u[1] * rr, zz]), q, rounds=2, iters=100)[1] <= C.IK_TOL

        rs = [rr for rr in np.arange(0.06, 0.43, 0.01) if ok(rr, point[2])]
        zs = [zz for zz in np.arange(0.01, 0.41, 0.02) if ok(r, zz)]
        text += f"The point is {r:.3f} m from the base at height {point[2]:.3f} m. "
        text += (f"In that direction at that height the tip can be about {min(rs):.2f} to {max(rs):.2f} m from the base. " if rs
                 else "In that direction nothing at that height can be reached. ")
        text += (f"At that distance it can be at heights of about {min(zs):.2f} to {max(zs):.2f} m." if zs
                 else "At that distance from the base no height can be reached.")
        return text

    def _line_blocked(self, start, end, q0):
        """Walk the straight line the controller would follow (1 cm steps, as it solves them: each from the last)
        and return the first point it cannot reach, or None."""
        ik = self.app.ik
        n = max(1, int(np.ceil(np.linalg.norm(end - start) / 0.01)))
        q = np.array(q0, float)
        for k in range(1, n + 1):
            p = start + (end - start) * k / n
            q, err = ik.solve(p, q, iters=120)
            if not (err <= C.IK_TOL and np.isfinite(q).all()):
                return p, k / n, q
        return None

    def _roll_for_axis(self, q, axis_deg: float) -> np.ndarray:
        """q with the wrist roll (inside its range) that brings the jaws' closing direction nearest to axis_deg."""
        best, cost = q[4], np.inf
        for roll in np.linspace(self.app.roll_lo, self.app.roll_hi, 181):
            t = q.copy()
            t[4] = roll
            c = abs(math.sin(math.radians(self._jaw_axis_deg(t) - axis_deg)))
            if c < cost - 1e-9:
                best, cost = roll, c
        out = q.copy()
        out[4] = best
        return out

    def _need_torque(self):
        if self.app.limp:
            raise Refused("the servos' torque is switched off (the arm is limp), so nothing was moved. "
                          "Switch it back on first (the arm then holds the pose it has sagged to).", 409)

    def _hand_path(self, start, end) -> np.ndarray:
        """The hand's lowest point along a straight move of the tip, every 10 mm: N x 3."""
        safety = getattr(self.app.sim, "safety", None)
        q = self.app.sim.q()
        drop = float(safety.lowest_of_hand(q)[2] - self.tip(q)[2]) if safety is not None else 0.0
        start, end = np.asarray(start, float), np.asarray(end, float)
        n = max(2, int(np.linalg.norm(end - start) / 0.01) + 1)
        return start + (end - start) * np.linspace(0.0, 1.0, n)[:, None] + [0.0, 0.0, min(drop, 0.0)]

    def move_tip(self, x, y, z, roll=None, square_jaws=False, jaws_along_deg=None, timeout_s=6.0, tolerance_m=0.004) -> dict:
        self._need_torque()
        app = self.app
        point = np.array([x, y, max(z, C.MIN_TIP_Z)])
        said = app.message
        # The controller steers the fixed jaw's pad; the caller means the middle between the pads. The
        # difference depends on which way the gripper ends up facing, so solve for it (a few rounds).
        gap = float(app.ik.jaw_gap(float(app.sim.q()[5])))
        q0 = app.goal.copy()
        if roll is not None:
            q0[4] = roll
        turn = ((lambda q: self._roll_for_axis(q, jaws_along_deg)) if jaws_along_deg is not None
                else (lambda q: G.square_roll(app.ik, q)) if square_jaws else None)
        q, err = self._solve_tip(point, q0, roll_fn=turn)
        if not (err <= C.IK_TOL and np.isfinite(q).all()) and np.isfinite(q).all():
            # From far off the solver can stop short of a point that is within reach: it goes downhill from where the arm
            # is. Once more, from where it got to. (Without this a point was refused as out of reach by an answer that
            # went on to say the arm reaches further than that in the same direction at the same height.)
            again, err_again = self._solve_tip(point, q, roll_fn=turn)
            if err_again <= C.IK_TOL and np.isfinite(again).all():
                q, err = again, err_again
        # Refuse before anything moves, with the reason: the end point, or a point on the straight line to it.
        if not (err <= C.IK_TOL and np.isfinite(q).all()):
            raise Refused("that point cannot be reached with the fingers pointing down, so nothing was moved. "
                           + self._why_unreachable(point, q if np.isfinite(q).all() else None)
                           + f" The tip is now at {np.round(self.tip(), 3).tolist()}.")
        end = point - self._mid(q, gap)
        bad = self._line_blocked(np.array(app.target, float), end, app.goal)
        if bad is not None:
            p, frac, q_bad = bad
            raise Refused(f"the point itself can be reached, but not in a straight line from where the tip is "
                           f"({np.round(self.tip(), 3).tolist()}), so nothing was moved: {frac * 100:.0f}% of the way along, the line "
                           f"passes through a place the arm cannot put the tip while following the line (each point is solved from the "
                           f"joint angles of the one before, as the controller does when it moves). "
                           + self._why_unreachable(p + self._mid(q_bad, gap), q_bad)
                           + " Go by way of another point (for example lift or lower the tip first, or move nearer the "
                             "middle of the arm's reach), or use move_joints.")
        cautious, careful = (False, None) if self.guard is None else self.guard(self._hand_path(self.tip(), point))
        if not app.move_to(end, "api: move_tip", roll=float(q[4]) if (turn is not None or roll is not None) else None,
                           speed=CAUTIOUS_SPEED if cautious else None):
            why = app.message
            app.message = said
            raise Refused(f"the controller refused the move ({why}), so nothing was moved. " + self._why_unreachable(point))
        self._goal_point = point
        self._cautious = cautious
        try:
            reached, elapsed = self._run(timeout_s, lambda: np.linalg.norm(self.tip() - point) <= tolerance_m)
        finally:
            self._cautious = False
        return self._action(reached, elapsed, {"careful": careful} if cautious else None)

    def probe(self, x, y, from_z=0.12, timeout_s=12.0) -> dict:
        """Touch: lower the tip slowly, straight down at (x, y), until it rests on something or reaches the surface. Says
        how high it stopped. It stops as soon as the tip has stopped following its command down, long before the stall
        guard would, so what it rests on is pressed only lightly."""
        self._need_torque()
        app = self.app
        if self.guard is not None:                                  # a thing may be touched; the rover may not
            self.guard(self._hand_path([x, y, from_z], [x, y, 0.0]), touching=True)
        above = self.move_tip(x, y, from_z, timeout_s=min(6.0, timeout_s), tolerance_m=0.006)
        if not above["reached"]:
            return {**above, "touched": None, "note": "the tip could not be brought above that point, so nothing was touched there"}
        t0, top = app.sim.time, self.tip()
        end = np.array(app.target, float)
        end[2] = C.MIN_TIP_Z - float(self._mid(app.goal, float(app.ik.jaw_gap(float(app.sim.q()[5]))))[2])
        app.set_mode("cartesian")
        app.failed_point = app.last_block = None
        app.move, app.move_roll = C.CartesianMove(app.target, end, PROBE_SPEED, float(app.gov_cfg["click_accel_m_s2"])), None
        # Resting on something: the command has gone on down over the last PROBE_WINDOW_S while the tip itself has not.
        # (The gap between the two is no measure: the tip trails its command by several millimetres when nothing is there.)
        window = max(3, int(round(PROBE_WINDOW_S / FRAME)))
        told, went, touched = [float(app.target[2])], [float(app.tip_pos(app.sim.q())[2])], False
        for _ in range(int(round(timeout_s / FRAME))):
            self._frame()
            told.append(float(app.target[2]))
            went.append(float(app.tip_pos(app.sim.q())[2]))
            moving = went[0] - went[-1] > 0.004                     # (it has got going: not the moment it sets off)
            if app.last_block is not None or (moving and len(told) > window and told[-window] - told[-1] > PROBE_LAG_M
                                              and went[-window] - went[-1] < 0.0012):
                touched = True
                break
            if app.move is None:
                for _ in range(window):                             # the command is at the bottom: let the tip arrive
                    self._frame()
                break                                               # all the way down: nothing but the surface
        app.hold_here(app.sim.q())                                  # stop pressing
        for _ in range(6):
            self._frame()
        at = self.tip()
        safety = getattr(app.sim, "safety", None)
        if safety is not None:                                      # what touched is the hand's lowest point, not the pads
            low = safety.lowest_of_hand(app.sim.q())
            at = np.array([low[0], low[1], low[2]])
        on_surface = at[2] < PROBE_SURFACE_M
        back = self.move_tip(float(top[0]), float(top[1]), float(top[2]), timeout_s=6.0, tolerance_m=0.008)
        return {"ok": True, "touched": bool(touched and not on_surface),
                "at_m": [round(float(v), 4) for v in at[:2]], "height_m": round(float(max(at[2], 0.0)), 4),
                "meaning": ("the tip came to rest on something this high above the surface the arm stands on" if touched and not on_surface
                            else "the tip went down to the surface: nothing stands at this point"),
                "elapsed_s": round(app.sim.time - t0, 3), "lifted_back": bool(back["reached"]),
                "accuracy": "at_m and height_m are the lowest point of the gripper where it stopped, from the arm's joint angles: "
                            "good to about 5 mm, and it reads a few millimetres high",
                "state": self.state()}

    def move_joints(self, timeout_s=6.0, **angles) -> dict:
        self._need_torque()
        app = self.app
        if not angles:
            raise Refused(f"name at least one joint: {list(JOINTS[:5])}")
        g = app.goal.copy()
        for name, v in angles.items():
            j = JOINTS.index(name)
            if not app.lo[j] <= v <= app.hi[j]:
                raise Refused(f"{name} = {v} is outside its range {round(float(app.lo[j]), 3)} to {round(float(app.hi[j]), 3)} rad")
            g[j] = v
        if app.tip_pos(g)[2] < C.MIN_TIP_Z:
            raise Refused(f"those angles would put the tip {app.tip_pos(g)[2] * 1000:.0f} mm relative to the table top (below it)")
        app.move = None
        app.goal = g
        app.target, app.roll = app.tip_pos(g), g[4]
        self._goal_point = None
        reached, elapsed = self._run(timeout_s, lambda: np.abs(app.sim.q()[:5] - g[:5]).max() < 0.05)
        return self._action(reached, elapsed)

    def gripper(self, opening=None, gap_m=None, timeout_s=2.5) -> dict:
        self._need_torque()
        app = self.app
        if (opening is None) == (gap_m is None):
            raise Refused("give exactly one of 'opening' (0 = closed, 1 = fully open) or 'gap_m' (metres between the jaw pads)")
        if gap_m is not None:
            if gap_m > self.max_gap:
                raise Refused(f"gap_m = {gap_m} is more than the jaws open ({self.max_gap:.3f} m)")
            angle = min(G.gripper_open_angle(app.ik, gap_m), C.GRIP_OPEN) if gap_m > 0 else C.GRIP_CLOSED
        else:
            angle = C.GRIP_CLOSED + opening * (C.GRIP_OPEN - C.GRIP_CLOSED)
        app.move = None
        self._goal_point = None                          # (the tip shifts sideways as the jaws move)
        # Only the jaws' target changes. (Until 2026-11-12 this re-solved the arm's pose for the tip with the
        # fingers down, so after a move_joints the arm swung back to that pose as the jaws moved.)
        app.goal[5] = app.grip = angle
        app.say("api: gripper")
        reached, elapsed = self._run(timeout_s, lambda: app.grip_latch.holding or abs(app.sim.q()[5] - angle) < 0.06)
        return self._action(reached, elapsed, {"holding": bool(app.grip_latch.holding)})

    def wait(self, seconds) -> dict:
        t0 = self.app.sim.time
        self._tick0, self._timeout = self._filter_tick(), seconds
        self.app.failed_point = self.app.last_block = None
        self.app.say("api: wait")
        for _ in range(int(round(seconds / FRAME))):
            self._frame()
        return self._action(True, self.app.sim.time - t0)
