"""The physics monitor: watches the simulation every step for things that should not happen.

    mon = PhysicsMonitor(sim)        # MuJoCo backend, with or without the servo bus
    ... run ...
    mon.assert_ok()                  # or mon.report(), mon.violations

Each check is an invariant of a correct simulation (limits in config/safety.toml):
  penetration   no two shapes overlap by more than 5 mm even for a step, or 1.5 mm for longer than 50 ms
  joint speed   no joint turns faster than its servo can
  joint accel   no joint accelerates harder than its motor could drive it
  object speed  no free object moves faster than the arm could throw it
  workcell      no free object is below the table top while over the table (it passed through)
  finite        the state is finite

With strict=True the first violation raises PhysicsViolation at the step it
happens, with the simulation state still there to look at.
"""
from __future__ import annotations

from collections import deque

import mujoco
import numpy as np

from . import JOINTS
from .config import load_safety, servo_limits


class PhysicsViolation(AssertionError):
    pass


class PhysicsMonitor:
    def __init__(self, sim, strict: bool = False, cfg: dict | None = None, attach: bool = True):
        self.cfg = {**load_safety()["monitor"], **(cfg or {})}
        inner = getattr(sim, "inner", sim)                   # BusBackend wraps the MuJoCo backend
        self.m, self.d = inner.m, inner.d
        self.strict = strict
        m = self.m
        self.dof = np.array([m.joint(n).dofadr[0] for n in JOINTS])
        self.v_max = float(self.cfg["max_joint_speed_frac"]) * servo_limits()["no_load_speed_rad_s"]
        self.a_max = float(self.cfg["max_joint_accel_rad_s2"])
        self.pen_max = float(self.cfg["max_penetration_m"])
        self.pen_sustained = float(self.cfg["max_sustained_penetration_m"])
        self.sustained_s = float(self.cfg["sustained_s"])
        self.obj_v_max = float(self.cfg["max_object_speed_m_s"])
        self.floor = float(self.cfg["workcell_floor_m"])
        self.window = max(1, int(round(float(self.cfg["accel_window_s"]) / m.opt.timestep)))
        self.free = [(m.body(m.jnt_bodyid[j]).name, m.jnt_qposadr[j], m.jnt_dofadr[j])
                     for j in range(m.njnt) if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE]
        t = m.geom("table").id if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "table") >= 0 else None
        self.table_xy = None if t is None else (m.geom_pos[t][:2] - m.geom_size[t][:2], m.geom_pos[t][:2] + m.geom_size[t][:2])
        # Wires are soft and thin: a wire pressed against something sinks in by part of its own
        # thickness. For a contact with a wire the limits are its radius (sustained) and its diameter.
        # (A ribbon is a thin box: at least 2 mm, or it could never touch anything without "passing through".)
        self.wire_radius = np.array([
            (m.geom_size[g][0] if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CAPSULE else max(float(m.geom_size[g].min()), 0.002))
            if m.body(m.geom_bodyid[g]).name.startswith("cable_") else 0.0 for g in range(m.ngeom)])
        self.violations: list[str] = []
        self.kinds: set[str] = set()
        self.reset()
        if attach:
            inner.step_hooks.append(self.check)

    def reset(self):
        self.worst = {"penetration_m": 0.0, "joint_speed_rad_s": 0.0, "joint_accel_rad_s2": 0.0, "object_speed_m_s": 0.0}
        self._vel: deque = deque(maxlen=self.window + 1)
        self._deep_t = 0.0
        self._bad = 0
        self._last_time = None

    def check(self, t: float | None = None):
        m, d = self.m, self.d
        if self._last_time is not None and d.time < self._last_time:      # the simulation was reset
            self._vel.clear()
        self._last_time = d.time
        bad = sum(int(d.warning[w].number) for w in (mujoco.mjtWarning.mjWARN_BADQPOS, mujoco.mjtWarning.mjWARN_BADQVEL,
                                                     mujoco.mjtWarning.mjWARN_BADQACC))
        if bad > self._bad or not (np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()):
            self._bad = bad                          # MuJoCo resets the state itself when it finds NaN or a blow-up
            return self._violation("finite", "the simulation state was not finite (MuJoCo reset it)")
        if d.ncon:
            depths = -d.contact.dist[:d.ncon]
            wire = np.maximum(self.wire_radius[d.contact.geom1[:d.ncon]], self.wire_radius[d.contact.geom2[:d.ncon]])
            sustained = np.where(wire > 0, wire, self.pen_sustained)
            instant = np.where(wire > 0, 2 * wire, self.pen_max)
            k = int((depths - sustained).argmax())              # the contact furthest past its limit
            depth = float(depths[k])
            rigid = depths[wire == 0]
            if len(rigid):
                self.worst["penetration_m"] = max(self.worst["penetration_m"], float(rigid.max()))
            self._deep_t = self._deep_t + m.opt.timestep if depth > sustained[k] else 0.0
            if depth > instant[k] or self._deep_t > self.sustained_s:
                self._violation("penetration", f"{self._geom(d.contact.geom1[k])} and {self._geom(d.contact.geom2[k])} "
                                               f"overlap by {depth * 1000:.1f} mm"
                                               + ("" if depth > instant[k] else f" for more than {self.sustained_s * 1000:.0f} ms"))
        else:
            self._deep_t = 0.0
        v = d.qvel[self.dof]
        speed = float(np.abs(v).max())
        self.worst["joint_speed_rad_s"] = max(self.worst["joint_speed_rad_s"], speed)
        if speed > self.v_max:
            self._violation("joint speed", f"{JOINTS[int(np.abs(v).argmax())]} turns at {speed:.1f} rad/s "
                                           f"(limit {self.v_max:.1f})")
        self._vel.append(v.copy())
        if len(self._vel) > self.window:
            a = (self._vel[-1] - self._vel[0]) / (self.window * m.opt.timestep)
            acc = float(np.abs(a).max())
            self.worst["joint_accel_rad_s2"] = max(self.worst["joint_accel_rad_s2"], acc)
            if acc > self.a_max:
                self._violation("joint acceleration", f"{JOINTS[int(np.abs(a).argmax())]} accelerates at {acc:.0f} rad/s2 "
                                                      f"(limit {self.a_max:.0f})")
        for name, qa, da in self.free:
            pos, sp = d.qpos[qa:qa + 3], float(np.linalg.norm(d.qvel[da:da + 3]))
            self.worst["object_speed_m_s"] = max(self.worst["object_speed_m_s"], sp)
            if sp > self.obj_v_max:
                self._violation("object speed", f"{name} moves at {sp:.1f} m/s (limit {self.obj_v_max:.1f})")
            if (self.table_xy is not None and pos[2] < self.floor
                    and (pos[:2] > self.table_xy[0]).all() and (pos[:2] < self.table_xy[1]).all()):
                self._violation("workcell", f"{name} is {-pos[2] * 1000:.0f} mm below the table top: it passed through the table")

    def _geom(self, g: int) -> str:
        m = self.m
        return f"{m.geom(g).name or 'a shape'} ({m.body(m.geom_bodyid[g]).name})"

    def _violation(self, kind: str, msg: str):
        text = f"t = {self.d.time:.3f} s: {kind}: {msg}"
        if kind not in self.kinds or len(self.violations) < 20:
            self.violations.append(text)
        self.kinds.add(kind)
        if self.strict:
            raise PhysicsViolation(text)

    def assert_ok(self):
        if self.violations:
            raise PhysicsViolation("; ".join(self.violations[:5]))

    def report(self) -> str:
        w = self.worst
        head = (f"physics monitor: deepest overlap {w['penetration_m'] * 1000:.2f} mm, fastest joint {w['joint_speed_rad_s']:.2f} rad/s, "
                f"hardest joint acceleration {w['joint_accel_rad_s2']:.0f} rad/s2, fastest object {w['object_speed_m_s']:.2f} m/s")
        if not self.violations:
            return head + "; no violations"
        return "\n".join([head, f"  {len(self.kinds)} kind(s) of violation:"] + [f"  {v}" for v in self.violations[:12]])
