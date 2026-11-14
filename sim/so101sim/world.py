"""The world: the scene now loaded, its physics, and the referee.

    world = World("open_small")
    world.step(0.5)                      # let time pass
    world.load("shelf")                  # another place, while the program runs
    world.truth()                        # the referee: where everything really is
    world.task_status("fetch_from_shelf")

Everything here is the simulation's side. The hub is given none of it (docs/MULTI_DEVICE_PLAN.md, 5.1),
only the world's ports:

    ports = world.ports()                # a serial port per servo bus, a camera port per camera, the clock
    hub = Hub(world.ports)               # the hub asks again whenever it looks for what is plugged in

There is one clock. Bus traffic and taking a picture cost time on it, and the physics runs whenever
the clock passes a step boundary (as so101sim/bus_backend.py does for the single arm). An arm's
servos are emulated HX-30HMs on a simulated wire; they were left powered, holding the arm's
starting pose, so an arm no hub has connected to stays where it is.
"""
from __future__ import annotations

import copy
import math
import zlib
from dataclasses import dataclass

import cv2
import mujoco
import numpy as np

from hub.arm import JOINTS as ARM_JOINTS
from hub.arm.config import load_servos, position_loop
from hub.arm.servo import ServoMap
from hub.arm.servo_model import ServoSpec, zero_joint_damping
from hub.bus import hx_registers as R
from hub.ports import Frame, PortError, Ports

from . import scenes
from .bus.servo_emu import ServoEmulator
from .bus.wire import VirtualBus
from .lens import FULL_H, FULL_W, Lens, lens_for
from .scenes import geometry as G
from .scenes import tasks as T
from .scenes.build import JOINTS


_WIRE_KEYS = ("baud", "response_delay_s", "usb_tx_latency_s", "usb_rx_latency_s", "echo",
              "drop_command_prob", "drop_reply_prob", "corrupt_prob", "seed")
FRAME_PERIOD_S = 1 / 28.0               # the sensor's frame rate (firmware/p4-uvc-camera/README.md)
USB_BYTES_PER_S = 8.0e6                 # what the camera's USB link carries
JPEG_QUALITY = 85


@dataclass
class Bus:
    """One servo bus: emulated servos on a simulated wire, each driving one joint of the model."""
    wire: VirtualBus
    servos: list
    joints: tuple                       # joint names, in the servos' order
    act: np.ndarray                     # actuator ids
    qadr: np.ndarray
    dof: np.ndarray
    hold: np.ndarray                    # the pose the servos were left holding [rad]
    smap: ServoMap


class WirePort:
    """A serial port on one of the world's buses. A blocking read advances the clock."""

    def __init__(self, world: "World", index: int):
        self.world, self.index, self.generation = world, index, world.generation

    def _wire(self):
        w = self.world
        if self.generation != w.generation or self.index >= len(w.buses):
            return None                               # that adapter is not plugged in any more: silence
        return w.buses[self.index].wire

    def write(self, data: bytes):
        wire = self._wire()
        if wire is not None:
            wire.host_write(data, self.world.t)

    def read(self, n: int, timeout_s: float) -> bytes:
        w, wire = self.world, self._wire()
        deadline = w.t + timeout_s
        if wire is None:
            w.advance_to(deadline)
            return b""
        while wire.available(w.t) < n:
            nxt = wire.next_event_time(w.t)
            if nxt is None or nxt > deadline:
                w.advance_to(deadline)
                break
            w.advance_to(nxt)
        return wire.host_read(n, w.t)

    def reset_input(self):
        wire = self._wire()
        if wire is not None:
            wire.host_discard(self.world.t)


class WorldClock:
    def __init__(self, world: "World"):
        self.world = world

    def now(self) -> float:
        return self.world.t

    def sleep_until(self, t: float):
        self.world.advance_to(t)


class WorldCamera:
    """A camera port: one fresh full-size picture per grab, through the camera's lens and JPEG, costing clock time."""

    def __init__(self, world: "World", camera: str):
        self.world, self.camera, self.generation = world, camera, world.generation
        seed = int(world.scene.get("imperfections", {}).get("seed", 0))
        self.identity = {"serial": f"P4CAM-{zlib.crc32(f'{camera}:{seed}'.encode()) & 0xFFFFFF:06X}",
                         "product": "ESP32-P4 UVC camera (Camera Module 3 Wide)", "kind": "cm3_wide"}

    def grab(self) -> Frame:
        w = self.world
        if self.generation != w.generation:
            raise PortError(f"camera {self.identity['serial']} is not there any more")
        w.advance_to(w.t + FRAME_PERIOD_S)                      # the next frame the sensor starts
        t = w.t
        image = w.render(self.camera)
        ok, buf = cv2.imencode(".jpg", cv2.cvtColor(image, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        w.advance_to(w.t + len(buf) / USB_BYTES_PER_S)          # the frame crossing the USB link
        return Frame(cv2.cvtColor(cv2.imdecode(buf, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB), t)


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
        self._attach_devices()
        self.reset()
        for hook in self.on_load:
            hook(self)

    def _attach_devices(self):
        """Put each device's servos on a bus. The model's position actuators become torque motors
        driven by emulated servos (as so101sim/bus_backend.py does for the single arm)."""
        m = self.model
        self.buses: list[Bus] = []
        cfg = load_servos().get("bus", {})
        if self.built.device("arm") is not None:
            act = np.array([m.actuator(n).id for n in ARM_JOINTS])
            for a in act:
                m.actuator_gaintype[a] = mujoco.mjtGain.mjGAIN_FIXED
                m.actuator_gainprm[a, :] = 0
                m.actuator_gainprm[a, 0] = 1.0
                m.actuator_biastype[a] = mujoco.mjtBias.mjBIAS_NONE
                m.actuator_biasprm[a, :] = 0
                m.actuator_ctrllimited[a] = 0
            zero_joint_damping(m, ARM_JOINTS)                    # back-EMF is in the servo's torque-speed line
            loop = position_loop()
            spec = ServoSpec.from_config(d_time_s=0.0 if float(loop["actuator_kv"]) == 0.0 else ServoSpec.d_time_s)
            smap = ServoMap(ARM_JOINTS)
            servos = [ServoEmulator(i, spec, mount_zero_ticks=z, mount_direction=int(d))
                      for i, z, d in zip(smap.ids, smap.zero, smap.direction)]
            wire = VirtualBus(servos, **{k: cfg[k] for k in _WIRE_KEYS if k in cfg})
            self.buses.append(Bus(wire, servos, ARM_JOINTS, act,
                                  np.array([m.joint(n).qposadr[0] for n in ARM_JOINTS]),
                                  np.array([m.joint(n).dofadr[0] for n in ARM_JOINTS]),
                                  np.array([self.built.start[n] for n in ARM_JOINTS]), smap))

    def reset(self):
        """The running scene back to its start. Answers handed in are kept."""
        self.data = self.built.new_data()
        self.t = 0.0
        goal, torque = R.BY_NAME["target_position"], R.BY_NAME["torque_switch"]
        for bus in self.buses:
            self.data.ctrl[bus.act] = 0.0
            bus.wire.reset()
            for s, ticks in zip(bus.servos, bus.smap.rad_to_ticks(bus.hold)):
                s.power_cycle()
                s._write(goal.addr, goal.encode(int(ticks)))    # left powered, holding the starting pose
                s._write(torque.addr, b"\x01")
        mujoco.mj_forward(self.model, self.data)

    @property
    def name(self) -> str:
        return self.scene["name"]

    @property
    def time(self) -> float:
        return float(self.t)

    def advance_to(self, t: float):
        """Run the wires, the servos and the physics up to time t on the one clock."""
        m, d = self.model, self.data
        dt = m.opt.timestep
        while d.time + dt <= t + 1e-9:
            for bus in self.buses:
                bus.wire.process(d.time)
                q, qd = d.qpos[bus.qadr], d.qvel[bus.dof]
                for k, s in enumerate(bus.servos):
                    d.ctrl[bus.act[k]] = s.control(float(q[k]), float(qd[k]), dt)
                    m.actuator_forcerange[bus.act[k]] = (-s.torque_limit_nm, s.torque_limit_nm)
            mujoco.mj_step(m, d)
        for bus in self.buses:
            bus.wire.process(t)
        self.t = max(self.t, t)

    def step(self, seconds: float):
        self.advance_to(self.t + max(0.0, float(seconds)))

    # ---- ports: all the hub ever gets --------------------------------------
    def ports(self) -> Ports:
        """What is plugged in right now: a serial port per servo bus, a camera port per camera, the clock."""
        cams = sorted((WorldCamera(self, name) for name in self.built.cameras), key=lambda c: c.identity["serial"])
        return Ports(clock=WorldClock(self), serial=[WirePort(self, i) for i in range(len(self.buses))], cameras=cams)

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

