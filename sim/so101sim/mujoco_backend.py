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
from .config import MODEL_DIR, WORKCELL, WORKCELL_CABLES, load_cameras
from .safety import CommandFilter


class MujocoBackend:
    name = "mujoco"

    def __init__(self, model_path=None, safety: bool = True, cables: bool = False, scene: str = "workcell"):
        """safety: pass every joint command through the command filter (so101sim/safety.py) on its way to
        the actuators. Off only for experiments that need the raw actuators.
        cables: use the model with the arm's wires (so101sim/cables.py). Several times slower.
        scene: which scene the arm is in: "workcell" (table and three objects) or "bench" (sim/scene/)."""
        if model_path is None:
            model_path = MODEL_DIR / f"{scene}{'_cables' if cables else ''}.xml"
            if not model_path.exists():
                raise ValueError(f"no scene '{scene}'; there are {sorted(p.stem for p in MODEL_DIR.glob('*.xml') if (MODEL_DIR.parent / 'scene' / p.name).exists())}")
        self.scene = scene
        self.m = mujoco.MjModel.from_xml_path(str(model_path))
        self.d = mujoco.MjData(self.m)
        self.dt = float(self.m.opt.timestep)
        self.qadr = np.array([self.m.joint(n).qposadr[0] for n in JOINTS])
        self.act = np.array([self.m.actuator(n).id for n in JOINTS])
        self.cameras = load_cameras()
        self._renderers: dict = {}
        self.step_hooks: list = []     # called with the time after every physics step (devices on the same clock)
        self.safety = CommandFilter() if safety else None
        self.cables = None
        if cables:
            import json
            from .cables import CableSet
            self.cables = CableSet(self.m, json.loads((MODEL_DIR / "cables.json").read_text()))
        self._request = self.d.ctrl[self.act].copy()   # what set_targets() last asked for
        self._intended = None
        self.torque_enabled = True
        dof = np.array([self.m.joint(n).dofadr[0] for n in JOINTS])
        self._powered = (dof, self.m.actuator_gainprm[self.act].copy(), self.m.actuator_biasprm[self.act].copy(),
                         self.m.dof_damping[dof].copy())

    # ---- torque switch -------------------------------------------------------
    def torque_off(self):
        """Switch the servos' output off: the arm goes limp and no command reaches it until torque_on().

        ASSUMED (as in bus/servo_emu.py): a servo with torque off coasts, so the back-EMF damping goes
        too and only the gear train's friction is left. If the real servo brakes instead, it sags slower."""
        dof = self._powered[0]
        self.m.actuator_gainprm[self.act] = 0
        self.m.actuator_biasprm[self.act] = 0
        self.m.dof_damping[dof] = 0
        self.torque_enabled = False

    def torque_on(self) -> np.ndarray:
        """Torque back on, holding the pose the arm is in now. Returns that pose: the caller's command
        must start again from it."""
        dof, gain, bias, damping = self._powered
        q = self.q()
        if self.safety is not None:
            self.safety.reset(q)
            q = self.safety.q.copy()
        self.d.ctrl[self.act] = q
        self._request, self._intended = q.copy(), None
        self.m.actuator_gainprm[self.act] = gain
        self.m.actuator_biasprm[self.act] = bias
        self.m.dof_damping[dof] = damping
        self.torque_enabled = True
        return q

    def reset(self, q=None):
        if not self.torque_enabled:
            self.torque_on()
        mujoco.mj_resetData(self.m, self.d)
        if q is not None:
            self.d.qpos[self.qadr] = q
            self.d.ctrl[self.act] = q
        mujoco.mj_forward(self.m, self.d)
        if self.cables is not None:
            self._settle_cables()
        self._request = self.d.ctrl[self.act].copy()
        self._intended = None
        if self.safety is not None:
            self.safety.reset(self._request)

    def _settle_cables(self):
        """Lay the wires out for this arm pose and let them settle, with the arm (and time) held still."""
        m, d = self.m, self.d
        arm_q = d.qpos[self.qadr].copy()
        dof = np.array([m.joint(n).dofadr[0] for n in JOINTS])
        self.cables.place(d)
        mujoco.mj_forward(m, d)
        for _ in range(self.cables.settle_steps):
            mujoco.mj_step(m, d)
            d.qpos[self.qadr] = arm_q
            d.qvel[dof] = 0
        d.qvel[:] = 0
        d.time = 0.0
        mujoco.mj_forward(m, d)

    def q(self) -> np.ndarray:
        return self.d.qpos[self.qadr].copy()

    def set_targets(self, q, intended=None):
        """Ask for joint positions. They reach the actuators through the command filter, one step at a time.

        intended: the pose q stands for before feed-forward, if q was compensated (the filter's keep-out
        is about where the arm is meant to be, not where the compensated command points).
        """
        self._request = np.array(q, float)
        self._intended = None if intended is None else np.array(intended, float)
        if self.safety is None:
            self.d.ctrl[self.act] = q

    def step(self, n: int = 1):
        if self.safety is None and not self.step_hooks:      # (torque off: the actuators' gains are zero)
            mujoco.mj_step(self.m, self.d, nstep=n)
            return
        for _ in range(n):
            if self.safety is not None and self.torque_enabled:
                self.d.ctrl[self.act] = self.safety.update(self._request, self.dt, self._intended,
                                                           measured=self.d.qpos[self.qadr])
            mujoco.mj_step(self.m, self.d)
            for hook in self.step_hooks:
                hook(float(self.d.time))

    @property
    def time(self) -> float:
        return float(self.d.time)

    def body_pos(self, name: str) -> np.ndarray:
        return self.d.body(name).xpos.copy()

    def render_camera(self, name: str, width: int, height: int, fovy_deg: float | None = None) -> np.ndarray:
        """One camera at any size and vertical field of view (a sensor mode that crops sees less)."""
        key = (height, width)
        if key not in self._renderers:
            self._renderers[key] = mujoco.Renderer(self.m, *key)
        cam = self.m.camera(name).id
        keep = float(self.m.cam_fovy[cam])
        if fovy_deg is not None:
            self.m.cam_fovy[cam] = fovy_deg
        try:
            r = self._renderers[key]
            r.update_scene(self.d, camera=name)
            return r.render().copy()
        finally:
            self.m.cam_fovy[cam] = keep

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
