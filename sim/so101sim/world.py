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
from dataclasses import dataclass, field, replace

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


RADIO = dict(delay_s=0.004, jitter_s=0.003, spike_prob=0.01, spike_s=0.08)      # each way, for a rover's Wi-Fi link (placeholders)
WIFI_BYTES_PER_S = 2.5e6                # what a rover's camera gets through over Wi-Fi (placeholder)
BOARD_WATCHDOG_S = 0.4                  # a rover's own board stops its wheels after this long without a word from the hub
BATTERY = dict(capacity_ah=5.0, cells=3, resistance_ohm=0.03, idle_a=0.35)        # the Gens ace 5000 mAh 3S pack; boards draw idle_a
TICKS = 4096


def cell_voltage(charge: float) -> float:
    """A LiPo cell's resting voltage against how full it is (0..1): a rough curve, good enough to run low on."""
    pts = [(0.0, 3.0), (0.05, 3.3), (0.15, 3.6), (0.5, 3.8), (0.9, 4.1), (1.0, 4.2)]
    return float(np.interp(charge, [p[0] for p in pts], [p[1] for p in pts]))


@dataclass
class Bus:
    """One servo bus: emulated servos on a simulated wire, each driving one joint of the model."""
    wire: VirtualBus
    servos: list
    joints: tuple                       # joint names, in the servos' order
    act: np.ndarray                     # actuator ids
    qadr: np.ndarray
    dof: np.ndarray
    hold: np.ndarray                    # the pose the servos were left holding (joint units)
    zero: np.ndarray                    # servo ticks at joint value 0
    ratio: np.ndarray                   # joint units per radian of the servo's shaft (1 for a hinge)
    device: str = "arm"
    radio: bool = False                 # reached over a radio link, through a board on the device
    link_up: bool = True                # (tests take a radio link down)
    last_word: float = 0.0              # when the hub's last bytes reached the device's board
    wheels: tuple = ()                  # indices of servos the board's watchdog stops
    board_stops: int = 0
    fw: object = None                   # the board's firmware, when it is run in the loop (rover_fw.RoverFirmware)
    fw_inbox: list = field(default_factory=list)       # (arrival time, bytes) on their way from the hub to the board


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


class RadioPort(WirePort):
    """A serial port reached over a rover's radio link: every byte is late by a few milliseconds each way, now
    and then by much more, and nothing passes while the link is down. `reply_time_s` tells a driver how much
    longer than on a wire a reply may take."""

    reply_time_s = 0.05

    def __init__(self, world: "World", index: int):
        super().__init__(world, index)
        self.rng = np.random.default_rng([world.generation, index, 77])
        self._last_arrival = -1.0

    def _delay(self) -> float:
        d = RADIO["delay_s"] + self.rng.exponential(RADIO["jitter_s"])
        return d + (RADIO["spike_s"] * self.rng.random() if self.rng.random() < RADIO["spike_prob"] else 0.0)

    def write(self, data: bytes):
        wire, w = self._wire(), self.world
        if wire is None or not w.buses[self.index].link_up:
            return
        arrival = max(self._last_arrival, w.t + self._delay())       # bytes arrive in the order they were sent
        self._last_arrival = arrival
        bus = w.buses[self.index]
        if bus.fw is not None:                                       # the board's firmware sees them when they arrive
            bus.fw_inbox.append((arrival, bytes(data)))
        wire.host_write(data, arrival)
        bus.last_word = arrival

    def read(self, n: int, timeout_s: float) -> bytes:
        w, wire = self.world, self._wire()
        deadline = w.t + timeout_s
        if wire is None or not w.buses[self.index].link_up:
            w.advance_to(deadline)
            return b""
        back = self._delay()
        while wire.available(w.t - back) < n:
            times = [a + back for a, _, _ in wire._rx if a > w.t - back + 1e-12]
            if wire._tx:
                times.append(max(wire._tx[0][0], w.t))
            nxt = min(times) if times else None
            if nxt is None or nxt > deadline:
                w.advance_to(deadline)
                break
            w.advance_to(max(nxt, w.t + 1e-6))
        if not w.buses[self.index].link_up:
            return b""
        got = wire.host_read(n, w.t - back)
        fw = w.buses[self.index].fw
        return fw.from_bus(got, int((w.t - back) * 1000)) if fw is not None and got else got

    def reset_input(self):
        wire = self._wire()
        if wire is not None:
            wire.host_discard(self.world.t - RADIO["delay_s"])


class WorldClock:
    def __init__(self, world: "World"):
        self.world = world

    def now(self) -> float:
        return self.world.t

    def sleep_until(self, t: float):
        self.world.advance_to(t)


class WorldCamera:
    """A camera port: one fresh full-size picture per grab, through the camera's lens and JPEG, costing clock time."""

    def __init__(self, world: "World", camera: str, part_of: str | None = None, role: str | None = None, bus: int | None = None):
        """part_of, role: for a camera that is one of a device's own parts and says so ("rover", "left").
        bus: the index of the radio bus it shares a link with, if it is reached by radio."""
        self.world, self.camera, self.generation, self.bus = world, camera, world.generation, bus
        seed = int(world.scene.get("imperfections", {}).get("seed", 0))
        self.identity = {"serial": f"P4CAM-{zlib.crc32(f'{camera}:{seed}'.encode()) & 0xFFFFFF:06X}",
                         "product": "ESP32-P4 camera (Camera Module 3 Wide)" + (f", {part_of} {role}" if part_of else ", UVC"),
                         "kind": "cm3_wide", **({"part_of": part_of, "role": role} if part_of else {})}

    def grab(self) -> Frame:
        w = self.world
        if self.generation != w.generation:
            raise PortError(f"camera {self.identity['serial']} is not there any more")
        w.advance_to(w.t + FRAME_PERIOD_S)                      # the next frame the sensor starts
        t = w.t
        image = w.render(self.camera)
        ok, buf = cv2.imencode(".jpg", cv2.cvtColor(image, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if self.bus is not None:                                 # over the rover's Wi-Fi: slower, and not at all if the link is down
            if not w.buses[self.bus].link_up:
                raise PortError(f"camera {self.identity['serial']} does not answer (its radio link is down)")
            w.advance_to(w.t + len(buf) / WIFI_BYTES_PER_S + 2 * RADIO["delay_s"])
        else:
            w.advance_to(w.t + len(buf) / USB_BYTES_PER_S)      # the frame crossing the USB link
        return Frame(cv2.cvtColor(cv2.imdecode(buf, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB), t)


class World:
    def __init__(self, scene: str | dict = "workcell", directory=None, rover_firmware: bool = False):
        """rover_firmware: run the rover board's own firmware (firmware/rover-bridge, compiled for this PC) between the
        radio link and the rover's servos, in place of this file's model of what that board does."""
        self.rover_firmware = bool(rover_firmware)
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
                                  np.array([self.built.start[n] for n in ARM_JOINTS]), smap.zero.copy(), np.ones(len(ARM_JOINTS))))
        rover = self.built.device("rover")
        self.battery = None
        if rover is not None:
            joints = rover["joints"]
            names = tuple(j["name"] for j in joints)
            act = np.array([m.actuator(n).id for n in names])
            ratio = np.array([j["ratio"] for j in joints])
            for a, r in zip(act, ratio):                         # ctrl = the servo's torque at its shaft
                m.actuator_gaintype[a] = mujoco.mjtGain.mjGAIN_FIXED
                m.actuator_gainprm[a, :] = 0
                m.actuator_gainprm[a, 0] = 1.0 / r
                m.actuator_biastype[a] = mujoco.mjtBias.mjBIAS_NONE
                m.actuator_biasprm[a, :] = 0
                m.actuator_ctrllimited[a] = 0
            zero_joint_damping(m, names)
            spec = ServoSpec.from_config(d_time_s=0.0)
            zero = np.array([j["zero_ticks"] for j in joints], float)
            servos = [ServoEmulator(j["servo"], spec, mount_zero_ticks=z, mount_direction=j["direction"]) for j, z in zip(joints, zero)]
            wire = VirtualBus(servos, **{k: cfg[k] for k in _WIRE_KEYS if k in cfg})
            self.buses.append(Bus(wire, servos, names, act, np.array([m.joint(n).qposadr[0] for n in names]),
                                  np.array([m.joint(n).dofadr[0] for n in names]), np.array([self.built.start[n] for n in names]),
                                  zero, ratio, device="rover", radio=True,
                                  wheels=tuple(k for k, j in enumerate(joints) if j["mode"] == "wheel")))
            self.battery = {"charge": float(rover["battery_charge"]), "bus": len(self.buses) - 1, "volts": 0.0, "next": 0.0}

    def reset(self):
        """The running scene back to its start. Answers handed in are kept."""
        self.data = self.built.new_data()
        self.t = 0.0
        goal, torque = R.BY_NAME["target_position"], R.BY_NAME["torque_switch"]
        for bus in self.buses:
            self.data.ctrl[bus.act] = 0.0
            bus.wire.reset()
            bus.last_word, bus.board_stops = 0.0, 0
            bus.fw, bus.fw_inbox = None, []
            if self.rover_firmware and bus.radio and bus.wheels:     # the board starts up with the scene
                from .rover_fw import RoverFirmware
                speed = R.BY_NAME["motion_speed"]
                bus.fw = RoverFirmware(wheel_ids=tuple(bus.servos[k].id for k in bus.wheels), speed_addr=speed.addr, speed_size=speed.size,
                                       watchdog_ms=int(round(BOARD_WATCHDOG_S * 1000)), now_ms=0)
            for s, hold, zero, ratio in zip(bus.servos, bus.hold, bus.zero, bus.ratio):
                s.power_cycle()
                ticks = int(round(zero + s.mount_dir * (hold / ratio) * TICKS / (2 * math.pi)))
                s._write(goal.addr, goal.encode(ticks))         # left powered, holding the starting pose
                s._write(torque.addr, b"\x01")
        if self.battery is not None:
            self.battery["charge"] = float(self.built.device("rover")["battery_charge"])
            self.battery["next"] = 0.0
            self._battery_step(0.0)
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
                if bus.fw is not None:                           # the board's own firmware decides
                    while bus.fw_inbox and bus.fw_inbox[0][0] <= d.time:
                        _, data = bus.fw_inbox.pop(0)
                        if bus.fw.from_hub(data, int(d.time * 1000)) != data:
                            raise AssertionError("the rover's firmware changed bytes on their way from the hub to the bus")
                    stop = bus.fw.tick(int(d.time * 1000))
                    if stop:
                        bus.wire.host_write(stop, d.time)
                        bus.board_stops = bus.fw.count("trips")
                elif bus.radio and bus.wheels and d.time - bus.last_word > BOARD_WATCHDOG_S:
                    self._board_stops_wheels(bus)
                q, qd = d.qpos[bus.qadr] / bus.ratio, d.qvel[bus.dof] / bus.ratio       # at each servo's shaft
                for k, s in enumerate(bus.servos):
                    d.ctrl[bus.act[k]] = s.control(float(q[k]), float(qd[k]), dt)
                    lim = s.torque_limit_nm / bus.ratio[k]
                    m.actuator_forcerange[bus.act[k]] = (-lim, lim)
            if self.battery is not None:
                self._battery_step(dt)
            mujoco.mj_step(m, d)
        for bus in self.buses:
            bus.wire.process(t)
        self.t = max(self.t, t)

    def step(self, seconds: float):
        self.advance_to(self.t + max(0.0, float(seconds)))

    def _board_stops_wheels(self, bus: Bus):
        """The rover's own board has heard nothing from the hub for a while: it stops the wheels itself."""
        speed = R.BY_NAME["motion_speed"]
        moving = [k for k in bus.wheels if bus.servos[k].get("motion_speed") & 0x7FFF]
        for k in moving:
            bus.servos[k]._write(speed.addr, speed.encode(0))
        if moving:
            bus.board_stops += 1

    def _battery_step(self, dt: float):
        """The battery drains by what the rover draws, and its servos see the voltage that leaves."""
        b = self.battery
        bus = self.buses[b["bus"]]
        amps = BATTERY["idle_a"] + sum(float(s.bank.last_current[0]) for s in bus.servos)
        b["charge"] = max(0.0, b["charge"] - amps * dt / 3600.0 / BATTERY["capacity_ah"])
        b["volts"] = BATTERY["cells"] * cell_voltage(b["charge"]) - BATTERY["resistance_ohm"] * amps
        if self.data.time >= b["next"]:                          # the servos' supply follows, ten times a second
            b["next"] = self.data.time + 0.1
            for s in bus.servos:
                s.base = replace(s.base, supply_v=b["volts"])
                s._apply_registers()

    # ---- ports: all the hub ever gets --------------------------------------
    def ports(self) -> Ports:
        """What is plugged in right now: a serial port per servo bus, a camera port per camera, the clock."""
        radio = next((i for i, b in enumerate(self.buses) if b.radio), None)
        cams = []
        for name, info in self.built.cameras.items():
            dev = next(d for d in self.built.devices if d["name"] == info["device"])
            if dev["kind"] == "rover":                           # a rover's cameras are its own parts, reached over its radio link
                cams.append(WorldCamera(self, name, part_of="rover", role=name.rsplit("_", 1)[1], bus=radio))
            else:
                cams.append(WorldCamera(self, name))
        cams.sort(key=lambda c: c.identity["serial"])
        serial = [RadioPort(self, i) if b.radio else WirePort(self, i) for i, b in enumerate(self.buses)]
        return Ports(clock=WorldClock(self), serial=serial, cameras=cams)

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

