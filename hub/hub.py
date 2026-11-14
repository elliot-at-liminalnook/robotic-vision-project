"""The hub: what is plugged in, as a tree of interests, and the commands that reach it.

    hub = Hub(provider)            # provider() -> hub.ports.Ports: what is plugged in right now
    hub.root                       # the tree (hub/tree.py)
    hub.act("devices/arm", "move_tip", {"x": 0.2, "y": 0.0, "z": 0.1})
    hub.where("camera_1", "arm")   # an estimate with its uncertainty, or "not known"

The hub is told nothing about the place it is in, or how many devices there are: it finds what
answers on its ports (discovery.py). Time is the ports' clock; it moves only inside an action.
"""
from __future__ import annotations

import math

import cv2
import numpy as np

from . import discovery, locate
from . import geometry as G
from .arm.device import ArmDevice
from .bus import BusError
from .camera import Camera
from .errors import Refused
from .ports import PortError, PortProvider
from .tree import Command, Node, Param

WIGGLE_MIN = 4.0               # grey levels: the wrist camera's picture changes by at least this when the wrist turns


class Hub:
    def __init__(self, provider: PortProvider):
        self.provider = provider
        self.arm: ArmDevice | None = None
        self.cameras: dict[str, Camera] = {}            # by the name the hub gave each
        self.notes: list[str] = []                      # what discovery found worth saying
        self.discover()

    # ---- discovery -----------------------------------------------------------
    def discover(self) -> dict:
        """Find out what is plugged in and take it over. Called at start, and again whenever asked."""
        ports = self.provider()
        self.clock = ports.clock
        self.notes = []
        self.arm, self.cameras, self.wrist = None, {}, None
        for k, port in enumerate(ports.serial):
            ids = discovery.scan(port)
            kinds, strangers = discovery.match(ids)
            if strangers:
                self.notes.append(f"serial port {k}: servo ids {strangers} answer but belong to no device this hub has a driver for")
            if "arm" in kinds and self.arm is None:
                arm = ArmDevice(port, self.clock)
                try:
                    arm.start()
                    self.arm = arm
                except BusError as e:
                    self.notes.append(f"serial port {k}: an arm answers but could not be taken over: {e}")
            if "rover" in kinds:
                self.notes.append(f"serial port {k}: a rover answers; this hub has no rover driver yet")
            if not ids:
                self.notes.append(f"serial port {k}: nothing answers")
        cams = sorted((Camera(p) for p in ports.cameras), key=lambda c: c.serial)
        wrist = self._find_wrist_camera(cams) if self.arm is not None and cams else None
        self.wrist = wrist
        for c in cams:
            if c is not wrist:
                self.cameras[f"camera_{len(self.cameras) + 1}"] = c
        self._build_tree()
        return self.found()

    def _find_wrist_camera(self, cams: list[Camera]) -> Camera | None:
        """Which camera rides on the arm's wrist? Turn the wrist a little: its picture changes, the others' barely do."""
        def small(c):
            return cv2.cvtColor(cv2.resize(c.grab().image, (320, 180), interpolation=cv2.INTER_AREA), cv2.COLOR_RGB2GRAY).astype(np.float32)
        try:
            before = [small(c) for c in cams]
            again = [small(c) for c in cams]                                # how much a still picture changes by itself
            self.arm.wiggle()
            after = [small(c) for c in cams]
            self.arm.wiggle(back=True)
        except (PortError, BusError, Refused) as e:
            self.notes.append(f"could not tell which camera is on the wrist: {e}")
            return None
        change = [float(np.abs(a - b).mean() - np.abs(b - b2).mean()) for a, b, b2 in zip(after, before, again)]
        order = np.argsort(change)[::-1]
        best = int(order[0])
        runner_up = change[int(order[1])] if len(cams) > 1 else 0.0
        if change[best] >= WIGGLE_MIN and change[best] >= 3.0 * max(runner_up, 0.3):
            return cams[best]
        self.notes.append("no camera's picture changed enough when the wrist turned: none was taken to be the wrist camera "
                          f"(changes: {[round(c, 1) for c in change]})")
        return None

    def _camera_node(self, name: str, cam: Camera, summary: str) -> Node:
        n = Node(name, "camera", summary, state=cam.state,
                 about={"lens": f"{cam.kind}: nominal values only; this unit's own are not known",
                        "pictures": "GET <this path>/picture gives a JPEG; ?width=640 scales it down (same field of view); "
                                    "?tags=true gives JSON with the printed tags found in the full-size picture; "
                                    "?base64=true gives the JPEG inside JSON"})
        n.camera = cam
        return n

    def _build_tree(self):
        self.root = Node("", "root", "everything this hub has found", state=lambda: {"time_s": round(float(self.clock.now()), 3)})
        devices = self.root.add(Node("devices", "branch", "each device that answered, by the name the hub gave it"))
        groups = self.root.add(Node("groups", "branch", "the same parts, gathered by kind of interest across devices"))
        self.root.add(Node("place", "branch", "what has been found of the surroundings",
                           state=lambda: {"found": "nothing yet: the hub does not map its surroundings yet"}))
        cams = groups.add(Node("cameras", "group", "every camera, wherever it is mounted"))
        movers = groups.add(Node("movers", "group", "everything that can move itself"))
        if self.arm is not None:
            arm = devices.add(self.arm.node("arm"))
            movers.links["arm"] = arm
            if self.wrist is not None:
                w = arm.add(self._camera_node("wrist_camera", self.wrist, "the camera on the arm's wrist: it moves and turns with the gripper"))
                cams.links["arm_wrist_camera"] = w
        for name, cam in self.cameras.items():
            node = devices.add(self._camera_node(name, cam, "a camera that does not move. Where it stands is not known until it is worked out"))
            node.kind = "device"
            node.about["device"] = "camera"
            cams.links[name] = node
        self.root.commands["wait"] = Command("Let time pass with every device holding still.", self.wait,
                                             {"seconds": Param("number", "how long", True, 0.0, 30.0)})
        self.root.commands["discover"] = Command(
            "Look again for what is plugged in, and take it over afresh. Use it when devices stop answering, or after being moved.",
            self.discover)

    def found(self) -> dict:
        return {"ok": True, "devices": {n.name: {"kind": n.about.get("device", n.kind), "what": n.summary}
                                        for n in self.root.children["devices"].children.values()},
                "notes": list(self.notes), "time_s": round(float(self.clock.now()), 3)}

    # ---- commands ------------------------------------------------------------
    def node(self, path: str) -> Node:
        n = self.root.find(path)
        if n is None:
            raise Refused(f"there is nothing at /{path.strip('/')}; GET /tree shows what there is", 404)
        return n

    def wait(self, seconds: float) -> dict:
        if self.arm is not None:
            self.arm.actions.wait(seconds)                  # the arm's controller keeps running while it holds
        else:
            self.clock.sleep_until(self.clock.now() + float(seconds))
        return {"ok": True, "time_s": round(float(self.clock.now()), 3)}

    def stop(self, path: str = "") -> dict:
        return {"ok": True, "stopped": self.node(path).stop(), "time_s": round(float(self.clock.now()), 3)}

    def shutdown(self) -> str:
        """Leave every servo with torque off. For every way out of the program; never raises."""
        return self.arm.shutdown() if self.arm is not None else "no servos to switch off"

    # ---- where things are ------------------------------------------------------
    def where(self, of: str, relative_to: str = "arm") -> dict:
        """Where one device is in another's frame, from what cameras see of the devices' tags right now.
        So far: a fixed camera and the arm, either way round (the locator comes with step 6 of the plan)."""
        names = set(self.cameras) | ({"arm"} if self.arm is not None else set())
        for n in (of, relative_to):
            if n not in names:
                raise Refused(f"there is no device '{n}'; there are {sorted(names)}", 404)
        if of == relative_to:
            raise Refused("of and relative_to are the same device")
        cam_name = of if of in self.cameras else relative_to if relative_to in self.cameras else None
        other = relative_to if cam_name == of else of
        if cam_name is None or other != "arm":
            return {"ok": True, "known": False, "of": of, "relative_to": relative_to,
                    "why": "the hub can so far only relate a fixed camera and the arm"}
        cam = self.cameras[cam_name]
        frame = cam.grab()
        fix = locate.camera_from_arm(frame.image, cam.lens(frame.image.shape[1], frame.image.shape[0]))
        if fix is None:
            return {"ok": True, "known": False, "of": of, "relative_to": relative_to,
                    "why": f"{cam_name} sees none of the tags on the arm's base"}
        if not fix.trusted:
            return {"ok": True, "known": False, "of": of, "relative_to": relative_to, "tags_seen": fix.tags,
                    "why": f"{cam_name} sees too little of the arm's tags to tell (tags {fix.tags}"
                           + (", all in one plane: the reading could be tilted either way)" if fix.ambiguous else ")")}
        T = fix.T_base_cam if of == cam_name else G.inv(fix.T_base_cam)
        out = {"ok": True, "known": True, "of": of, "relative_to": relative_to,
               "position_m": [round(float(v), 4) for v in T[:3, 3]],
               "uncertainty_m": round(fix.sigma_m, 4), "uncertainty_deg": round(math.degrees(fix.sigma_rad), 2),
               "from": {"camera": cam_name, "tags": fix.tags, "picture_s": round(frame.t, 3)},
               "frame": "the arm's own frame" if relative_to == "arm" else
                        f"{relative_to}'s own frame: x to the picture's right, y down the picture, z the way it looks"}
        if of == cam_name:
            out["looking_along"] = [round(float(v), 4) for v in T[:3, 2]]
        return out
