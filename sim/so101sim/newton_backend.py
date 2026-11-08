"""Newton backend: SolverMuJoCo on the generated workcell MJCF.

solver="warp" (default on CUDA): MuJoCo Warp, the GPU backend.
solver="cpu"  (default without CUDA): MuJoCo's C backend through the same
              Newton model, for development on CPU-only Linux machines.
Cameras are Newton's ray-traced SensorCamera, with rays generated from the
OpenCV intrinsics/distortion in config/cameras.toml (so calibrated lens
distortion is reproduced in the rendered images).
"""
from __future__ import annotations

import numpy as np
import warp as wp

import newton
from newton.sensors import SensorCamera

from . import JOINTS
from .config import WORKCELL, load_cameras


def _mat_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    t = np.trace(R)
    if t > 0:
        s = 2.0 * np.sqrt(t + 1.0)
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
        q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
        q[i] = 0.25 * s
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
    q = np.asarray(q)
    return q / np.linalg.norm(q)


def _quat_xyzw_to_mat(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _index(labels, name: str) -> int:
    """Index of the label equal to name or ending in '/name'."""
    hits = [i for i, lab in enumerate(labels) if lab == name or lab.rsplit("/", 1)[-1] == name]
    if len(hits) != 1:
        raise KeyError(f"{name!r}: {len(hits)} matches in {list(labels)[:12]}...")
    return hits[0]


class NewtonBackend:
    name = "newton"

    # MuJoCo Warp preallocates contact/constraint buffers per world; its default
    # (256 contacts) overflows during a grasp (26 jaw hulls + 6 pads + object +
    # table) and silently drops contacts, so the jaws pass through the object.
    WARP_NCONMAX = 2048
    WARP_NJMAX = 8192

    def __init__(self, model_path=WORKCELL, device=None, solver: str | None = None,
                 nconmax: int | None = None, njmax: int | None = None):
        self.device = wp.get_device(device) if device else wp.get_preferred_device()
        if solver is None:
            solver = "warp" if self.device.is_cuda else "cpu"
        self.solver_kind = solver

        builder = newton.ModelBuilder()
        newton.solvers.SolverMuJoCo.register_custom_attributes(builder)
        builder.add_mjcf(str(model_path))
        self._show_colliders_without_visuals(builder)
        self.model = builder.finalize(device=self.device)
        if solver == "cpu":
            self.solver = newton.solvers.SolverMuJoCo(self.model, use_mujoco_cpu=True)
        else:
            self.solver = newton.solvers.SolverMuJoCo(
                self.model, use_mujoco_cpu=False,
                nconmax=nconmax or self.WARP_NCONMAX, njmax=njmax or self.WARP_NJMAX)
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()
        self.dt = 0.002   # matches <option timestep> in workcell.xml
        self._time = 0.0

        labels = list(self.model.joint_label)
        jidx = [_index(labels, n) for n in JOINTS]
        q_start = self.model.joint_q_start.numpy()
        qd_start = self.model.joint_qd_start.numpy()
        self.qadr = np.array([q_start[j] for j in jidx])
        self.dofadr = np.array([qd_start[j] for j in jidx])
        self.body_labels = list(self.model.body_label)
        self._q0 = self.model.joint_q.numpy().copy()

        self.cameras = load_cameras()
        self._init_cameras()

    @staticmethod
    def _show_colliders_without_visuals(builder):
        """Make collision shapes visible on bodies that have no visual shapes.

        Newton's MJCF importer hides all colliders when the model has any visual
        geoms (the robot does), which would leave the table and the objects
        (collision-only geoms) invisible to the cameras.
        """
        visible = int(newton.ShapeFlags.VISIBLE)
        bodies_with_visuals = {b for b, f in zip(builder.shape_body, builder.shape_flags) if f & visible}
        for i, (b, f) in enumerate(zip(builder.shape_body, builder.shape_flags)):
            if b not in bodies_with_visuals and not f & int(newton.ShapeFlags.SITE):
                builder.shape_flags[i] = f | visible

    # ---- simulation -------------------------------------------------------
    def reset(self, q=None):
        joint_q = self._q0.copy()
        if q is not None:
            joint_q[self.qadr] = q
        for st in (self.state_0, self.state_1):
            st.joint_q.assign(joint_q)
            st.joint_qd.zero_()
            newton.eval_fk(self.model, st.joint_q, st.joint_qd, st)
        targets = self.control.joint_target_q.numpy()
        if q is not None:
            targets[self.dofadr] = q
        self.control.joint_target_q.assign(targets)
        self._time = 0.0

    def q(self) -> np.ndarray:
        return self.state_0.joint_q.numpy()[self.qadr].copy()

    def set_targets(self, q):
        targets = self.control.joint_target_q.numpy()
        targets[self.dofadr] = q
        self.control.joint_target_q.assign(targets)

    def step(self, n: int = 1):
        for _ in range(n):
            self.solver.step(self.state_0, self.state_1, self.control, None, self.dt)
            self.state_0, self.state_1 = self.state_1, self.state_0
        self._time += n * self.dt

    @property
    def time(self) -> float:
        return self._time

    def body_pos(self, name: str) -> np.ndarray:
        return self.state_0.body_q.numpy()[_index(self.body_labels, name), :3].copy()

    # ---- cameras ----------------------------------------------------------
    def _init_cameras(self):
        self.sensor = SensorCamera(
            self.model, default_clear_data=SensorCamera.ClearData(clear_color=0xFF3A3A3A))
        self.sensor.create_default_light(enable_shadows=True)
        self._cam = {}
        for name, spec in self.cameras.items():
            i = spec.intrinsics
            kw = dict(image_width=i["calib_width"], image_height=i["calib_height"], device=self.device)
            if i["model"] == "pinhole_opencv":
                rays = SensorCamera.compute_camera_rays_pinhole_opencv(
                    spec.render_width, spec.render_height, i["fx"], i["fy"], i["cx"], i["cy"],
                    k1=i.get("k1", 0.0), k2=i.get("k2", 0.0), k3=i.get("k3", 0.0),
                    p1=i.get("p1", 0.0), p2=i.get("p2", 0.0), **kw)
            elif i["model"] == "fisheye_opencv":
                rays = SensorCamera.compute_camera_rays_fisheye_opencv(
                    spec.render_width, spec.render_height, i["fx"], i["fy"], i["cx"], i["cy"],
                    k1=i.get("k1", 0.0), k2=i.get("k2", 0.0), k3=i.get("k3", 0.0), k4=i.get("k4", 0.0), **kw)
            else:
                raise ValueError(f"{name}: unknown intrinsics model {i['model']!r}")
            body = -1 if spec.parent_body == "world" else _index(self.body_labels, spec.parent_body)
            self._cam[name] = dict(
                rays=rays, body=body, R=spec.rotation, p=np.asarray(spec.pos, float),
                color=self.sensor.allocate_image_color(1, spec.render_width, spec.render_height))

    def _camera_world_pose(self, cam) -> tuple[np.ndarray, np.ndarray]:
        if cam["body"] < 0:
            return cam["R"], cam["p"]
        bq = self.state_0.body_q.numpy()[cam["body"]]
        Rb = _quat_xyzw_to_mat(bq[3:])
        return Rb @ cam["R"], bq[:3] + Rb @ cam["p"]

    def render(self, names=None) -> dict[str, np.ndarray]:
        # The renderer ray-traces a shape BVH built at finalize(); move it to
        # the current body poses first.
        self.model.bvh_refit_shapes(self.state_0)
        out = {}
        for name, cam in self._cam.items():
            if names is not None and name not in names:
                continue
            R, p = self._camera_world_pose(cam)
            q = _mat_to_quat_xyzw(R)
            tf = wp.array([wp.transformf(wp.vec3f(*p), wp.quatf(*q))], dtype=wp.transformf, device=self.device)
            self.sensor.update(self.state_0, tf, cam["rays"], color_image=cam["color"])
            rgba = cam["color"].numpy()[0].view(np.uint8).reshape(*cam["color"].shape[1:], 4)
            out[name] = rgba[..., :3].copy()
        return out
