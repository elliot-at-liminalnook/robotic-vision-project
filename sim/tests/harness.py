"""Test rigs: a headless Teleop driven frame by frame, and the invariant monitor."""
from __future__ import annotations

import numpy as np

from so101sim import JOINTS
from so101sim.config import servo_limits
from so101sim.servo_model import ServoSpec

FRAME = 1 / 30


class Monitor:
    """Invariants that must hold in every scenario, checked after every frame.

    Reads the MuJoCo backend's actuator forces: the torque each servo actually
    delivers, which is what overload protection and heat depend on.
    """

    SATURATED = 0.98        # fraction of an actuator's force limit that counts as "at the limit"
    MAX_SATURATED_S = 1.0   # no servo sits at its limit longer than this (stalled against something)
    MAX_OVERLOAD_S = 1.0    # ... or above its overload threshold (the servo trips at 2 s)
    MIN_TIP_Z = -0.002      # tip below the table top [m]

    def __init__(self, app):
        self.app, sim = app, app.sim
        self.m, self.d = sim.m, sim.d
        self.lo, self.hi = self.m.actuator_ctrlrange[sim.act].T
        self.fmax = np.abs(self.m.actuator_forcerange[sim.act]).max(axis=1)
        spec = ServoSpec.from_config()
        # Overload threshold per joint: 80 % of the torque limit (the gripper's limit is 50 %).
        frac = self.fmax / servo_limits()["stall_torque_nm"]
        self.overload = spec.overload_threshold * frac * spec.stall_torque
        self.sat_t = np.zeros(len(JOINTS))
        self.over_t = np.zeros(len(JOINTS))
        self.worst_sat = np.zeros(len(JOINTS))
        self.worst_over = np.zeros(len(JOINTS))
        self.min_tip_z = np.inf
        self.problems: list[str] = []

    def check(self, dt: float = FRAME):
        sim, app = self.app.sim, self.app
        force = np.abs(self.d.actuator_force[sim.act])        # ~ motor voltage: saturates at the limit
        net = np.abs(self.net_torque())                       # ~ motor current: what overload protection sees
        ctrl = self.d.ctrl[sim.act]
        if not np.isfinite(ctrl).all() or not np.isfinite(app.goal).all():
            self.problems.append(f"non-finite command at t={sim.time:.2f}")
        if (app.goal < app.lo - 1e-9).any() or (app.goal > app.hi + 1e-9).any():
            self.problems.append(f"goal outside the joint limits at t={sim.time:.2f}: {app.goal}")
        gen = app.gen.current
        if (gen < self.lo - 1e-4).any() or (gen > self.hi + 1e-4).any():
            self.problems.append(f"generator command outside the joint limits at t={sim.time:.2f}: {gen}")
        self.sat_t = np.where(force >= self.SATURATED * self.fmax, self.sat_t + dt, 0.0)
        self.over_t = np.where(net > self.overload, self.over_t + dt, 0.0)
        self.worst_sat = np.maximum(self.worst_sat, self.sat_t)
        self.worst_over = np.maximum(self.worst_over, self.over_t)
        self.min_tip_z = min(self.min_tip_z, float(app.tip_pos()[2]))

    def net_torque(self) -> np.ndarray:
        """Torque each servo delivers to its joint: actuator force minus the back-EMF loss.

        The model carries the motor's back-EMF as joint damping, so the
        position actuator's force includes the part that only overcomes it.
        """
        sim = self.app.sim
        dof = np.array([self.m.joint(n).dofadr[0] for n in JOINTS])
        return self.d.actuator_force[sim.act] - self.m.dof_damping[dof] * self.d.qvel[dof]

    def assert_ok(self):
        assert not self.problems, self.problems[:5]
        for j, name in enumerate(JOINTS):
            assert self.worst_sat[j] <= self.MAX_SATURATED_S, \
                f"{name} sat at its torque limit for {self.worst_sat[j]:.2f} s (stalled against something)"
            assert self.worst_over[j] <= self.MAX_OVERLOAD_S, \
                f"{name} above its overload threshold for {self.worst_over[j]:.2f} s (the servo trips at 2 s)"
        assert self.min_tip_z >= self.MIN_TIP_Z, f"tip went {-self.min_tip_z * 1000:.1f} mm below the table"


class Rig:
    """A Teleop app without a window: call its methods, then run() frames."""

    def __init__(self):
        import pygame as pg
        import teleop
        pg.init()
        self.pg, self.teleop = pg, teleop
        self.app = teleop.Teleop("mujoco", None, render_every=10 ** 9)
        self.keys = teleop._FakeKeys(pg)
        self.mon = Monitor(self.app)

    def run(self, seconds: float, each=None):
        for _ in range(int(round(seconds / FRAME))):
            self.app.handle_held_keys(self.keys, FRAME, self.pg)
            self.app.physics(FRAME)
            self.mon.check()
            if each is not None:
                each()

    def key(self, k):
        self.app.handle_event(self.pg.event.Event(self.pg.KEYDOWN, key=k, mod=0, unicode="", scancode=0), self.pg)

    def event(self, kind, **kw):
        self.app.handle_event(self.pg.event.Event(kind, **kw), self.pg)

    def click_table(self, xy, cam: str = "scene_front"):
        """Left-click the pixel that shows table point xy in a fixed camera view."""
        from so101sim.config import look_at_axes
        T, spec = self.teleop, self.app.sim.cameras[cam]
        x, y, z = look_at_axes(spec.pos, spec.look_at, spec.up)
        i = spec.intrinsics
        s = spec.render_width / i["calib_width"]
        d = np.array([xy[0], xy[1], 0.0]) - np.asarray(spec.pos)
        u = i["cx"] * s + i["fx"] * s * (d @ x) / (-(d @ z))
        v = i["cy"] * s - i["fy"] * s * (d @ y) / (-(d @ z))
        c, r = T.LAYOUT[cam]
        pos = (int(c * T.VIEW_W + u * T.VIEW_W / spec.render_width),
               int(r * (T.VIEW_H + T.PANEL) + T.PANEL + v * T.VIEW_H / spec.render_height))
        self.event(self.pg.MOUSEBUTTONDOWN, button=1, pos=pos)

    def grasp_pose(self, body: str = "cube", width: float = 0.030):
        """(tip position, roll, open angle) that puts `body` between the jaws, as pick_demo does."""
        import pick_demo as pd
        app = self.app
        obj = app.sim.body_pos(body)
        q_open = pd.gripper_open_angle(app.ik, width + 0.014)
        above = pd.square_roll(app.ik, app.ik.solve(obj + [0, 0, 0.08], app.goal)[0])
        grasp, _ = pd.solve_grasp(app.ik, obj, width, above, dz=-0.004)
        return app.tip_pos(grasp), float(grasp[4]), q_open

    def gripper_torque(self) -> float:
        sim = self.app.sim
        return float(sim.d.actuator_force[sim.act][5])
