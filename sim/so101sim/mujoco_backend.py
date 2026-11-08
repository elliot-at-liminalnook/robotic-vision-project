"""Plain MuJoCo backend: same model and API as NewtonBackend.

For machines where Newton can't run (e.g. Intel Macs: Warp ships no x86_64
macOS build) and as a reference to compare Newton against. Renders with
MuJoCo's OpenGL pinhole cameras, so lens distortion from cameras.toml is NOT
applied here (only fovy); Newton's ray-traced cameras apply it.
"""
from __future__ import annotations

import mujoco
import numpy as np

from . import JOINTS
from .config import WORKCELL, load_cameras


class MujocoBackend:
    name = "mujoco"

    def __init__(self, model_path=WORKCELL):
        self.m = mujoco.MjModel.from_xml_path(str(model_path))
        self.d = mujoco.MjData(self.m)
        self.dt = float(self.m.opt.timestep)
        self.qadr = np.array([self.m.joint(n).qposadr[0] for n in JOINTS])
        self.act = np.array([self.m.actuator(n).id for n in JOINTS])
        self.cameras = load_cameras()
        self._renderers: dict = {}

    def reset(self, q=None):
        mujoco.mj_resetData(self.m, self.d)
        if q is not None:
            self.d.qpos[self.qadr] = q
            self.d.ctrl[self.act] = q
        mujoco.mj_forward(self.m, self.d)

    def q(self) -> np.ndarray:
        return self.d.qpos[self.qadr].copy()

    def set_targets(self, q):
        self.d.ctrl[self.act] = q

    def step(self, n: int = 1):
        mujoco.mj_step(self.m, self.d, nstep=n)

    @property
    def time(self) -> float:
        return float(self.d.time)

    def body_pos(self, name: str) -> np.ndarray:
        return self.d.body(name).xpos.copy()

    def render(self, names=None) -> dict[str, np.ndarray]:
        out = {}
        for name, spec in self.cameras.items():
            if names is not None and name not in names:
                continue
            key = (spec.render_height, spec.render_width)
            if key not in self._renderers:
                self._renderers[key] = mujoco.Renderer(self.m, *key)
            r = self._renderers[key]
            r.update_scene(self.d, camera=name)
            out[name] = r.render().copy()
        return out
