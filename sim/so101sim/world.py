"""The world: the scene now loaded, its physics, and the referee.

    world = World("open_small")
    world.step(0.5)                      # let time pass
    world.load("shelf")                  # another place, while the program runs
    world.truth()                        # the referee: where everything really is
    world.task_status("fetch_from_shelf")

Everything here is the simulation's side. The hub is given none of it (docs/MULTI_DEVICE_PLAN.md, 5.1):
what a hub may use will be the world's ports. Until the hub exists (checklist step 5) no device is
driven: an arm holds its starting pose on its own servos and the rover stands where it was put.
"""
from __future__ import annotations

import copy
import math

import mujoco
import numpy as np

from . import scenes
from .lens import FULL_H, FULL_W, Lens, lens_for
from .scenes import geometry as G
from .scenes import tasks as T
from .scenes.build import JOINTS


class World:
    def __init__(self, scene: str | dict = "workcell", directory=None):
        self.directory = directory                # where scene files are (default: sim/scenes/)
        self.generation = 0                       # goes up with every load: "this is a different place now"
        self.on_load: list = []                   # called with the world after each load (ports re-attach here)
        self.load(scene)

    # ---- scenes ------------------------------------------------------------
    def load(self, scene: str | dict):
        """Make a scene the running one. A scene that is not valid is refused (ValueError) and nothing changes."""
        if isinstance(scene, str):
            scene = scenes.read(scene, self.directory)
        problems = scenes.validate(scene)
        if problems:
            raise ValueError("; ".join(problems))
        self.scene = copy.deepcopy(scene)
        self.built = scenes.build(self.scene)
        self.model = self.built.model
        self.generation += 1
        self.answers: dict[str, dict] = {}        # task name -> what was handed in
        for r in getattr(self, "_renderers", {}).values():
            r.close()
        self._renderers, self._lenses, self._frames = {}, {}, 0
        self.reset()
        for hook in self.on_load:
            hook(self)

    def reset(self):
        """The running scene back to its start. Answers handed in are kept."""
        self.data = self.built.new_data()

    @property
    def name(self) -> str:
        return self.scene["name"]

    @property
    def time(self) -> float:
        return float(self.data.time)

    def step(self, seconds: float):
        for _ in range(max(0, int(round(seconds / self.model.opt.timestep)))):
            mujoco.mj_step(self.model, self.data)

    # ---- pictures ------------------------------------------------------------
    def _pinhole(self, camera: str, width: int, height: int, fovy_deg: float | None = None) -> np.ndarray:
        key = (self.generation, width, height)
        if key not in self._renderers:
            if len(self._renderers) >= 4:                       # (each holds a frame buffer)
                self._renderers.pop(next(iter(self._renderers))).close()
            self._renderers[key] = mujoco.Renderer(self.model, height, width)
        c = self.model.camera(camera).id
        keep = float(self.model.cam_fovy[c])
        if fovy_deg is not None:
            self.model.cam_fovy[c] = fovy_deg
        try:
            self._renderers[key].update_scene(self.data, camera=camera)
            return self._renderers[key].render().copy()
        finally:
            self.model.cam_fovy[c] = keep

    def lens(self, camera: str, width: int = FULL_W, height: int = FULL_H) -> Lens:
        """This camera's lens at this picture size. Ground truth: the hub is never given it."""
        key = (self.generation, camera, width, height)
        if key not in self._lenses:
            seed = int(self.scene.get("imperfections", {}).get("seed", 0))
            self._lenses[key] = Lens(lens_for(camera, seed), width, height)
        return self._lenses[key]

    def render(self, camera: str, width: int = FULL_W, height: int = FULL_H, ideal: bool = False) -> np.ndarray:
        """What a camera gives now (RGB): the scene through that camera's own lens and sensor (so101sim/lens.py).
        ideal: the perfect pinhole picture instead, for checks that are not about the lens."""
        if camera not in self.built.cameras:
            raise KeyError(f"no camera '{camera}'; there are {sorted(self.built.cameras)}")
        if ideal:
            return self._pinhole(camera, width, height)
        self._frames += 1
        rng = np.random.default_rng([int(self.scene.get("imperfections", {}).get("seed", 0)), self.generation, self._frames])
        return self.lens(camera, width, height).shoot(lambda w, h, fovy: self._pinhole(camera, w, h, fovy), rng)

    def camera_pose(self, camera: str, relative_to_body: str | None = None) -> np.ndarray:
        """The referee's: a camera's true pose (4x4; camera frame x right, y down, z forward) in the scene's
        frame, or in a body's."""
        m, d = self.model, self.data
        c = m.camera(camera).id
        T = np.eye(4)
        T[:3, :3] = d.cam_xmat[c].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0])
        T[:3, 3] = d.cam_xpos[c]
        if relative_to_body is not None:
            b = m.body(relative_to_body).id
            Tb = np.eye(4)
            Tb[:3, :3], Tb[:3, 3] = d.xmat[b].reshape(3, 3), d.xpos[b]
            T = np.linalg.inv(Tb) @ T
        return T

    # ---- the referee -------------------------------------------------------
    def truth(self) -> dict:
        """Where everything really is, in the scene's frame. For tests and scoring: never for the model."""
        m, d, b = self.model, self.data, self.built
        r3 = lambda v: [round(float(x), 4) for x in v]                # noqa: E731

        def pose(body: str) -> dict:
            i = m.body(body).id
            R = d.xmat[i].reshape(3, 3)
            return {"position_m": r3(d.xpos[i]), "yaw_deg": round(math.degrees(math.atan2(R[1, 0], R[0, 0])), 2),
                    "upright": bool(abs(R[2, 2]) > 0.95)}
        devices = {}
        for dev in b.devices:
            if dev["kind"] == "arm":
                q = {j: round(float(d.qpos[m.joint(j).qposadr[0]]), 4) for j in b.start if j in JOINTS}
                devices[dev["name"]] = {"kind": "arm", **pose("base"), "joints_rad": q}
            elif dev["kind"] == "rover":
                devices[dev["name"]] = {"kind": "rover", **pose(dev["body"])}
            else:
                c = m.camera(dev["name"]).id
                devices[dev["name"]] = {"kind": "camera", "position_m": r3(d.cam_xpos[c]),
                                        "looking_along": r3(-d.cam_xmat[c].reshape(3, 3)[:, 2])}
        return {
            "scene": self.name, "generation": self.generation, "time_s": round(self.time, 3),
            "surface": {"outline_m": b.outline, "area_m2": round(G.area(b.outline), 5),
                        "lip_height_m": self.scene["surface"]["lip_height_m"]},
            "devices": devices,
            "objects": {o: pose(o) for o in b.objects},
            "fixtures": {n: {"kind": f["kind"], "top_m": round(f["top_z"], 4),
                             **({"out_m": round(float(d.qpos[m.joint(f["joint"]).qposadr[0]]), 4)} if "joint" in f else {})}
                         for n, f in b.fixtures.items()},
            "regions": {n: {"holds": [o for o in b.objects if r.holds(m, d, d.xpos[m.body(o).id])]} for n, r in b.regions.items()},
        }

    def tasks(self) -> list[str]:
        return [t["name"] for t in self.scene.get("tasks", [])]

    def _task(self, name: str) -> dict:
        t = next((t for t in self.scene.get("tasks", []) if t["name"] == name), None)
        if t is None:
            raise KeyError(f"no task '{name}' in scene '{self.name}'; there are {self.tasks()}")
        return t

    def hand_in(self, task: str, answers: dict):
        """Record answers for a task (key -> value). The caller is told nothing about whether they are right."""
        self.answers.setdefault(task, {}).update(answers)

    def task_status(self, name: str, task: dict | None = None) -> dict:
        """A task's goals and whether each holds now. `task`: check this task's goals here instead
        (a task that swaps scenes is checked in the scene it swapped to)."""
        t = task or self._task(name)
        goals = T.check_task(self.built, self.data, t, self.answers.get(name, {}))
        return {"task": name, "scene": self.name, "request": t["request"],
                "goals": [{"kind": g["goal"]["kind"], "done": g["done"], "detail": g["detail"],
                           **{k: v for k, v in g["goal"].items() if k in ("object", "region", "on", "fixture", "key")}} for g in goals],
                "done": bool(goals) and all(g["done"] for g in goals)}

