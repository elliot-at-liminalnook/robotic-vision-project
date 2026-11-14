"""The command filter: the last stage before the servos.

Whatever produced a joint command (teleop, a script, a policy, a bug), the
servos only receive one that
  * is finite and inside the joint limits,
  * is not a jump: one tick's change no servo could follow is replaced by the
    fastest motion the controller itself plans (config/safety.toml),
  * does not ask for a pose with any part below the table top or inside the
    arm's own base.

It is deliberately a backstop, not a motion planner: a well-formed command
stream passes through unchanged (`interventions` stays empty) and well-paced
motion still comes from the torque governor or a motion profile. The speed and
acceleration of a command are not limited as such: with soft servos and
feed-forward a command leads the arm and jitters tick to tick, and bounding
that would mangle good commands without making the arm any gentler. A command that would enter a keep-out is cut
back along the straight line from the last accepted command to the largest
step that stays out; a pose already inside (after a reset, say) may always
move out.

The keep-out is checked on the pose the controller INTENDS, which is not always
the command: feed-forward (so101sim/command.py) shifts each command ahead of
the arm to cancel droop and lag, by up to several degrees at speed, so a
command can point below the table while the arm is nowhere near it. A caller
that compensates passes the uncompensated pose as `intended`; without it the
command itself is taken as the intended pose.

It works with the model's collision shapes:
the lowest point of every moving shape for the table, and MuJoCo's own
collision check against the base's shapes. It runs on a private copy of the
robot model, so it is the same code for the simulation and the real arm.

What it did is kept as EVENTS: one per stretch of ticks in which it stepped in for the same reason, with
the numbers it had at the time (which joint, how far past what limit, which part of the arm how deep
in which keep-out). `describe(event)` puts one into a sentence. Everything in an event comes from the
commands, the joint limits, the filter's own model of the arm and, for a restart, the measured joint
angles: nothing a controller on the real arm would not have.
"""
from __future__ import annotations

import math
from collections import Counter, deque

import mujoco
import numpy as np

from . import JOINTS
from .config import MODEL_DIR, load_safety, load_servos, servo_limits

HAND_BODIES = ("lower_arm", "wrist", "gripper", "moving_jaw_so101_v1", "wrist_camera_assembly")


class CommandFilter:
    def __init__(self, q0=None, cfg: dict | None = None, model_path=MODEL_DIR / "so101_follower.xml"):
        self.cfg = {**load_safety()["command_filter"], **(cfg or {})}
        self.m = mujoco.MjModel.from_xml_path(str(model_path))
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.qadr = np.array([m.joint(n).qposadr[0] for n in JOINTS])
        self.lo = np.array([m.joint(n).range[0] for n in JOINTS])
        self.hi = np.array([m.joint(n).range[1] for n in JOINTS])
        w0 = servo_limits()["no_load_speed_rad_s"]
        gov = load_servos()["follower"]["governor"]
        self.v_jump = float(self.cfg["jump_speed_frac"]) * w0      # faster than this in one tick: a jump
        self.v_max = float(gov["speed_frac"]) * w0                 # catching up with a jump: the governor's caps
        self.a_max = float(gov["accel_max_rad_s2"])
        self.table_z = float(self.cfg["table_z_m"]) + float(self.cfg["table_margin_m"])
        self.table_tol = float(self.cfg["table_tolerance_m"])
        self.base_margin = float(self.cfg["base_margin_m"])
        self.max_lead = math.radians(float(self.cfg["max_lead_deg"]))
        self.reanchor = math.radians(float(self.cfg["restart_from_arm_deg"]))
        self.restart_after = float(self.cfg["restart_after_s"])
        self._collect_shapes()
        self.interventions: Counter = Counter()
        self.limited_joints: Counter = Counter()     # which joints a jump was caught on
        self.last_reason = ""
        self.tick = 0                                # update() calls so far
        self.events: deque = deque(maxlen=200)       # what it did, oldest first (see the module docstring)
        self.started: Counter = Counter()            # events per reason (interventions counts ticks)
        self._open: dict[str, dict] = {}
        self._dt = 0.002
        self.reset(np.zeros(len(JOINTS)) if q0 is None else q0)

    # ---- geometry ------------------------------------------------------------
    def _collect_shapes(self):
        """Corner points of every moving collision shape, per body, in the body's frame."""
        m, d = self.m, self.d
        mujoco.mj_forward(m, d)
        base = m.body("base").id
        moving = [b for b in range(1, m.nbody) if b != base]
        self.points: dict[int, np.ndarray] = {}
        for b in moving:
            pts = []
            R_b, p_b = d.xmat[b].reshape(3, 3), d.xpos[b]
            for g in range(m.ngeom):
                if m.geom_bodyid[g] != b or not m.geom_contype[g]:
                    continue
                R_g, p_g = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
                if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                    v = _hull_vertices(m, m.geom_dataid[g])
                elif m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
                    v = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * m.geom_size[g]
                else:
                    raise ValueError(f"collision shape type {m.geom_type[g]} on {m.body(b).name} is not handled")
                pts.append((R_b.T @ ((R_g @ v.T).T + p_g - p_b).T).T)
            if pts:
                self.points[b] = np.vstack(pts)
        self.base_geoms = {g for g in range(m.ngeom) if m.geom_bodyid[g] == base and m.geom_contype[g]}
        self.hand_bodies = {m.body(n).id for n in HAND_BODIES}
        if not self.base_geoms:
            raise ValueError("the robot model's base has no collision shapes (rebuild sim/model)")
        # A box around the base's shapes: only poses that reach into it need the exact check.
        corners = np.vstack([(d.geom_xmat[g].reshape(3, 3) @ _hull_vertices(m, m.geom_dataid[g]).T).T + d.geom_xpos[g]
                             for g in self.base_geoms])
        self.base_lo, self.base_hi = corners.min(0) - 0.01, corners.max(0) + 0.01
        for g in self.base_geoms:                    # so the check reports shapes closer than the margin, not only overlaps
            m.geom_margin[g] = self.base_margin

    def depths(self, q) -> np.ndarray:
        """How far the pose q reaches into each keep-out [m]: (table, base). <= 0 means clear."""
        m, d = self.m, self.d
        d.qpos[self.qadr] = q
        mujoco.mj_kinematics(m, d)
        low, near_base = np.inf, False
        self.low_body = self.base_body = None        # which part is lowest / nearest the base in this pose
        for b, pts in self.points.items():
            w = pts @ d.xmat[b].reshape(3, 3).T + d.xpos[b]
            if float(w[:, 2].min()) < low:
                low, self.low_body = float(w[:, 2].min()), b
            if b in self.hand_bodies and not near_base:
                near_base = bool(((w > self.base_lo) & (w < self.base_hi)).all(axis=1).any())
        base = -np.inf
        if near_base:
            mujoco.mj_collision(m, d)
            for c in d.contact[:d.ncon]:
                if (c.geom1 in self.base_geoms) != (c.geom2 in self.base_geoms):
                    other = c.geom2 if c.geom1 in self.base_geoms else c.geom1
                    if m.geom_bodyid[other] in self.hand_bodies and self.base_margin - c.dist > base:
                        base, self.base_body = self.base_margin - c.dist, int(m.geom_bodyid[other])
        return np.array([self.table_z - low, base])

    def depth(self, q) -> tuple[float, str]:
        """The deeper of the two, and which one it is."""
        dd = self.depths(q)
        return (float(dd[0]), "table") if dd[0] >= dd[1] else (float(dd[1]), "base")

    def _allowed_from(self, here) -> bool:
        """Is the arm's own pose clear of every keep-out?"""
        return bool((self.depths(here) <= self.table_tol).all())

    def _allowed(self, dd) -> bool:
        """A pose may be entered if it is clear of each keep-out, or no deeper in it than the pose it comes from."""
        return bool(((dd <= self.table_tol) | (dd <= self._depths + 1e-9)).all())

    # ---- the filter ------------------------------------------------------------
    def reset(self, q):
        self.q = np.clip(np.asarray(q, float), self.lo, self.hi)     # the command last sent
        self.intended = self.q.copy()                # the pose it stands for (the command minus its feed-forward shift)
        self.v = np.zeros_like(self.q)
        self.catching, self.speed = False, 0.0       # catching up with a jump, and how fast (furthest joint, rad/s)
        self._last_want = self.q.copy()
        self._stuck_t = 0.0
        self._depths = self.depths(self.intended)

    def update(self, request, dt: float, intended=None, measured=None) -> np.ndarray:
        """The command to send this tick, given what was asked for.

        intended: the pose the request stands for before feed-forward (default: the request itself).
        The filter works on that pose (jumps, keep-outs) and then puts the feed-forward shift back on.
        measured: where the arm is now, if known. Used only to get unstuck: see below.
        """
        req = np.asarray(request, float)
        self.tick += 1
        self._dt = dt
        if not np.isfinite(req).all() or (intended is not None and not np.isfinite(intended).all()):
            bad = ~np.isfinite(req) if intended is None else ~np.isfinite(req) | ~np.isfinite(intended)
            self._note("nonfinite", joints=[JOINTS[j] for j in np.nonzero(bad)[0]])
            req = np.where(np.isfinite(req), req, self.q)
            intended = None if intended is None else np.where(np.isfinite(intended), intended, self.intended)
        clipped = np.clip(req, self.lo, self.hi)
        if np.abs(clipped - req).max() > 1e-9:
            j = int(np.abs(clipped - req).argmax())
            # Was the pose itself past the limit, or only the feed-forward correction added to it?
            pose_over = 0.0 if intended is None else float(abs(np.clip(intended[j], self.lo[j], self.hi[j]) - intended[j]))
            self._note("joint_limit", size=float(abs(clipped[j] - req[j])), joint=JOINTS[j], asked=float(req[j]),
                       limit=float(clipped[j]), pose_past_limit=bool(intended is None or pose_over > 1e-9))
        want = clipped if intended is None else np.clip(np.asarray(intended, float), self.lo, self.hi)
        shift = clipped - want                       # feed-forward: how far the command leads the pose it is for
        if np.abs(shift).max() > self.max_lead:
            j = int(np.abs(shift).argmax())
            self._note("lead", size=float(abs(shift[j])), joint=JOINTS[j], cap=self.max_lead)
            shift = np.clip(shift, -self.max_lead, self.max_lead)

        err = want - self.intended
        far = float(np.abs(err).max())
        moved = want - self._last_want               # how far the request itself went since the last tick
        self._last_want = want.copy()
        passed = not self.catching and far <= self.v_jump * dt + 1e-12
        if passed:
            step = err                               # within one tick's reach: pass it on exactly
        else:
            # A jump. Close the gap in a straight line in joint space (all joints arrive together), at
            # the governor's speed and acceleration caps for the joint with furthest to go, slowing in
            # time to stop on it; meanwhile keep following whatever smooth motion the request still makes.
            follow = np.where(np.abs(moved) <= self.v_jump * dt, moved, 0.0)
            err = err - follow
            far = float(np.abs(err).max())
            if not self.catching:
                self.catching = True
                norm = max(float(np.linalg.norm(err)), 1e-12)
                self.speed = float(np.clip(float(self.v @ err) / norm * far / norm, 0.0, self.v_max))   # keep the speed it had that way
            big = np.nonzero(np.abs(err) > self.v_jump * dt)[0]
            self._note("jump", size=far, joints={JOINTS[j]: float(err[j]) for j in big},
                       reach=self.v_jump * dt, tick_s=dt, speed_cap=self.v_max)
            for j in big:
                self.limited_joints[JOINTS[j]] += 1
            brake = -self.a_max * dt + math.sqrt((self.a_max * dt) ** 2 + 2 * self.a_max * far)
            self.speed += min(max(min(self.v_max, brake) - self.speed, -self.a_max * dt), self.a_max * dt)
            if self.speed * dt >= far or far < 1e-9:
                step, self.catching, self.speed = err + follow, False, 0.0
            else:
                step = err * (self.speed * dt / far) + follow
            step = np.clip(self.intended + step, self.lo, self.hi) - self.intended

        # Keep-outs. If the new pose is not allowed, go only as far along the straight line to it as is;
        # zero means staying exactly where it was.
        pose = self.intended + step
        dd = self.depths(pose)
        if not self._allowed(dd):
            over = dd - np.maximum(self._depths, self.table_tol)
            which = 0 if over[0] >= over[1] else 1
            part = self.low_body if which == 0 else self.base_body
            asked = self.depths(want)                # the pose asked for, not just this tick's step toward it
            if asked[which] > dd[which]:
                part = self.low_body if which == 0 else self.base_body
            self._note("table" if which == 0 else "base", size=float(max(asked[which], dd[which])),
                       part=None if part is None else self.m.body(part).name,
                       keep=self.table_z if which == 0 else self.base_margin)
            lo_f, hi_f = 0.0, 1.0
            for _ in range(6):
                mid = 0.5 * (lo_f + hi_f)
                if self._allowed(self.depths(self.intended + mid * step)):
                    lo_f = mid
                else:
                    hi_f = mid
            pose = self.intended + lo_f * step
            dd = self.depths(pose) if lo_f > 0 else self._depths
            self.speed, passed = 0.0, False          # stopped at the keep-out
            # Stuck: no part of the step is allowed. If the arm itself is somewhere else (something
            # held it back while the accepted pose went on to the keep-out's edge), the accepted pose
            # is a place the arm never was, and the way from THERE to the request may be barred when
            # the way from the arm is not. Start again from where the arm is.
            self._stuck_t = self._stuck_t + dt if lo_f == 0.0 else 0.0
            if self._stuck_t >= self.restart_after and measured is not None:
                here = np.clip(np.asarray(measured, float), self.lo, self.hi)
                if np.abs(here[:5] - self.intended[:5]).max() > self.reanchor and self._allowed_from(here):
                    j = int(np.abs(here[:5] - self.intended[:5]).argmax())
                    self._note("restart_from_arm", size=float(abs(here[j] - self.intended[j])), joint=JOINTS[j],
                               stuck_s=self._stuck_t)
                    pose, dd, self.catching, self._stuck_t = here, self.depths(here), True, 0.0
        else:
            self._stuck_t = 0.0
        self.v = (pose - self.intended) / dt
        self.intended = want.copy() if passed else pose
        self._depths = dd
        self.q = clipped.copy() if passed and shift is not None and np.array_equal(shift, clipped - want) \
            else np.clip(self.intended + shift, self.lo, self.hi)
        return self.q.copy()

    def _note(self, reason: str, size: float = 0.0, **detail):
        """Count one tick of stepping in, and add it to that reason's event (a new event if the last one for
        this reason ended more than a quarter of a second ago). An event keeps the details of its worst tick."""
        self.interventions[reason] += 1
        self.last_reason = reason
        ev = self._open.get(reason)
        if ev is None or self.tick - ev["last_tick"] > max(1, int(0.25 / self._dt)):
            ev = {"reason": reason, "first_tick": self.tick, "last_tick": self.tick, "ticks": 0, "seconds": 0.0, "size": -1.0}
            self._open[reason] = ev
            self.events.append(ev)
            self.started[reason] += 1
        ev["last_tick"], ev["ticks"], ev["seconds"] = self.tick, ev["ticks"] + 1, ev["seconds"] + self._dt
        if size >= ev["size"]:
            ev.update(detail, size=size)

    def events_since(self, tick: int) -> list[dict]:
        """Events that were still going on after the given tick (self.tick at some earlier moment)."""
        return [e for e in self.events if e["last_tick"] > tick]

    def describe(self, ev: dict) -> str:
        """One event as a sentence, with the numbers the filter had."""
        deg = lambda v: f"{math.degrees(v):.1f}"          # noqa: E731
        mm = lambda v: f"{v * 1000:.1f}"                   # noqa: E731
        name = lambda b: str(b).replace("_so101_v1", "").replace("_", " ")   # noqa: E731
        r, t = ev["reason"], f"{ev['seconds']:.2f} s"
        going = "; still going on" if ev["last_tick"] >= self.tick - 1 else ""
        if r == "nonfinite":
            return f"The command for {', '.join(ev['joints'])} was not a number, so the last valid command was kept (for {t}{going})."
        if r == "joint_limit":
            if ev["pose_past_limit"]:
                return (f"{ev['joint']} was asked to go to {deg(ev['asked'])} deg, which is {deg(ev['size'])} deg past its limit of "
                        f"{deg(ev['limit'])} deg; it was held at the limit (for {t}{going}).")
            return (f"{ev['joint']} is at or near its limit of {deg(ev['limit'])} deg. The pose asked for is inside the limit, but the "
                    f"controller's own correction for load and lag asked for up to {deg(ev['size'])} deg beyond it and was cut off "
                    f"there, so this joint may sag or lag slightly here (for {t}{going}).")
        if r == "lead":
            return (f"The controller's correction for load and lag on {ev['joint']} reached {deg(ev['size'])} deg; no more than "
                    f"{deg(ev['cap'])} deg is allowed, so it was capped (for {t}{going}).")
        if r == "jump":
            js = sorted(ev["joints"].items(), key=lambda kv: -abs(kv[1]))
            parts = ", ".join(f"{j} by {deg(abs(v))} deg" for j, v in js[:3]) + (f" and {len(js) - 3} more" if len(js) > 3 else "")
            return (f"The command changed in one {ev['tick_s'] * 1000:.0f} ms step by more than a servo can turn in that time "
                    f"({deg(ev['reach'])} deg): {parts}. The filter moved the command there gradually instead, all joints "
                    f"together, at up to {deg(ev['speed_cap'])} deg/s (took {t}{going}).")
        if r == "table":
            return (f"The pose asked for would put the {name(ev['part'])} {mm(ev['size'])} mm below the lowest height any part of "
                    f"the arm may go ({mm(ev['keep'])} mm above the table top). The command was stopped at that height, short of "
                    f"the pose asked for (held back for {t}{going}).")
        if r == "base":
            gap = ev["keep"] - ev["size"]
            where = f"{mm(-gap)} mm into" if gap < 0 else f"within {mm(gap)} mm of"
            return (f"The pose asked for would bring the {name(ev['part'])} {where} the robot's own base; at least "
                    f"{mm(ev['keep'])} mm is kept. The command was stopped short of the pose asked for (held back for {t}{going}).")
        if r == "restart_from_arm":
            return (f"The arm had been held {deg(ev['size'])} deg ({ev['joint']}) away from its command at a keep-out for "
                    f"{ev['stuck_s']:.1f} s, so the command was started again from where the arm actually is.")
        return f"{r} (for {t})"

    def report(self) -> str:
        if not self.interventions:
            return "command filter: never had to intervene"
        lines = ["command filter stepped in: " + ", ".join(
            f"{k} {self.started[k]} time(s), {v * self._dt:.2f} s in all" for k, v in self.interventions.most_common())]
        lines += ["  " + self.describe(e) for e in list(self.events)[-8:]]
        return "\n".join(lines)


def _hull_vertices(m, mesh_id: int) -> np.ndarray:
    """Vertices of a mesh's convex hull (MuJoCo collides meshes as their hull), in the geom frame."""
    v = m.mesh_vert[m.mesh_vertadr[mesh_id]:m.mesh_vertadr[mesh_id] + m.mesh_vertnum[mesh_id]]
    if len(v) <= 80:
        return v.copy()
    from scipy.spatial import ConvexHull
    return v[ConvexHull(v).vertices].copy()
