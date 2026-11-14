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

from . import discovery
from .arm.device import ArmDevice
from .bus import BusError
from .camera import Camera
from .errors import Refused
from .locator import Locator
from .place import Place, re_frame
from . import landmarks
from . import geometry as G
from . import survey as S
from .vision import tags as V
from .ports import PortError, PortProvider
from .rover.device import RoverDevice
from .tree import Command, Node, Param

WIGGLE_MIN = 4.0               # grey levels: the wrist camera's picture changes by at least this when the wrist turns


class Hub:
    def __init__(self, provider: PortProvider, survey: bool = False):
        """survey: when a rover is found, have it look all round from where it stands before anything else."""
        self.provider = provider
        self.survey_at_start = survey
        self.arm: ArmDevice | None = None
        self.rover: RoverDevice | None = None
        self.cameras: dict[str, Camera] = {}            # fixed cameras, by the name the hub gave each
        self.notes: list[str] = []                      # what discovery found worth saying
        self.locator: Locator | None = None
        self.discover()

    # ---- discovery -----------------------------------------------------------
    def discover(self) -> dict:
        """Find out what is plugged in and take it over. Called at start, and again whenever asked."""
        ports = self.provider()
        self.clock = ports.clock
        self.notes = []
        self.arm, self.rover, self.cameras, self.wrist = None, None, {}, None
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
            if "rover" in kinds and self.rover is None:
                rover = RoverDevice(port, self.clock)
                try:
                    rover.start()
                    self.rover = rover
                except BusError as e:
                    self.notes.append(f"serial port {k}: a rover answers but could not be taken over: {e}")
            if not ids:
                self.notes.append(f"serial port {k}: nothing answers")
        cams = sorted((Camera(p) for p in ports.cameras), key=lambda c: c.serial)
        mine = [c for c in cams if c.identity.get("part_of") == "rover"]          # a rover's cameras say they are its own, and which
        cams = [c for c in cams if c not in mine]
        if self.rover is not None:
            self.rover.cameras = {c.identity.get("role"): c for c in mine if c.identity.get("role") in ("left", "right")}
        elif mine:
            self.notes.append(f"{len(mine)} camera(s) say they belong to a rover, but no rover answered")
        wrist = self._find_wrist_camera(cams) if self.arm is not None and cams else None
        self.wrist = wrist
        for c in cams:
            if c is not wrist:
                self.cameras[f"camera_{len(self.cameras) + 1}"] = c
        if self.locator is None:
            self.locator = Locator(self.clock)
        self.locator.clock = self.clock
        self.locator.reset(self.arm is not None, {name: c.lens(1920, 1080) for name, c in self.cameras.items()},     # nothing seen yet
                           rover_now=(lambda: (self.rover.head_now(), self.rover.pose.copy())) if self.rover is not None else None)
        self.place = Place()                                # and nothing known of the place: it may be another one
        self.map_origin = np.zeros(3)
        self.last_survey: list[dict] = []                   # every view of the last survey, in detail
        self._touches_tried, self.landmark_note = 0, None
        self._build_tree()
        out = self.found()
        if self.survey_at_start and self.rover is not None and len(self.rover.cameras) == 2:
            try:
                out["survey"] = S.survey(self)
            except Refused as e:
                self.notes.append(f"the start-up survey was not made: {e}")
        return out

    # ---- the place, from every device ---------------------------------------------------------
    def _map_in(self, name: str, flat: bool = True):
        """The map's frame in a device's frame, and how well that is known: (4 x 4, sigma_m) or (None, why).
        flat: as a turn about the vertical and a shift along the surface, for a device that stands on the same surface."""
        if self.rover is None or self.place.empty:
            return None, "there is no map yet"
        back = G.inv(self.map_from_rover())                    # the map's frame in the rover's, now
        if name == "rover":
            T, sigma = back, 0.0
        else:
            q = self.arm.link.q() if self.arm is not None else None
            link, why = self.locator.chain("rover", name, q)
            if link is None:
                return None, why
            T, sigma = link.T @ back, link.sigma_m
        if flat:
            yaw = math.atan2(T[1, 0], T[0, 0])
            T = G.pose(G.rot_z(yaw), [float(T[0, 3]), float(T[1, 3]), 0.0])
        return T, sigma

    def _landmarks(self):
        """If nothing relates the arm to the rover, try the things both have found: the arm by touch, the rover in its map."""
        if self.arm is None or self.rover is None or self.place.empty or len(self.arm.touches) == self._touches_tried:
            return
        self._touches_tried = len(self.arm.touches)
        if self.locator.chain("rover", "arm", self.arm.link.q())[0] is not None:
            return
        got = landmarks.match(self.arm.touches, self.place.things())
        if got[0] is None:
            self.landmark_note = "the arm and the map are not related by what the arm has touched: " + got[1]
            return
        T_arm_map, sigma_m, sigma_rad, pairs = got
        self.locator.rover_placed("arm", T_arm_map @ self.map_from_rover(), sigma_m, sigma_rad,
                                  f"{len(pairs)} things the arm has touched are things in the rover's map (landmarks)")
        self.landmark_note = f"the arm is related to the map by {len(pairs)} things it has touched that the rover's map also has"

    def place_state(self, relative_to: str | None = None) -> dict:
        """What has been found of the surroundings, from every device: the rover's map, what the arm has touched, and where
        each thing is in each fixed camera's picture. In the map's own frame, or in a device's if the two are related."""
        self._landmarks()
        out = self.place.summary()
        touches = list(self.arm.touches) if self.arm is not None else []
        if relative_to is not None and relative_to not in self.locator.names():
            raise Refused(f"there is no device '{relative_to}'; there are {self.locator.names()}", 404)
        if "things" in out:
            for name, cam in self.cameras.items():               # where each thing is in each fixed camera's picture
                T, _ = self._map_in(name, flat=False)
                if T is None:
                    continue
                lens = cam.lens(1920, 1080)
                for t in out["things"]:
                    p = T @ [t["at_m"][0], t["at_m"][1], t["height_m"], 1.0]
                    if p[2] < 0.05:
                        continue
                    px = cv2.projectPoints(p[:3].reshape(1, 3), np.zeros(3), np.zeros(3), lens.K(), np.asarray(lens.dist, float))[0].reshape(2)
                    if 0 <= px[0] < lens.width and 0 <= px[1] < lens.height:
                        t.setdefault("in_picture", {})[name] = [int(round(float(px[0]))), int(round(float(px[1])))]
            T_arm, _ = self._map_in("arm") if self.arm is not None else (None, "")
            safety = getattr(self.arm.link, "safety", None) if self.arm is not None else None
            if T_arm is not None and safety is not None:         # the rover's cameras see the arm too: say which things are it
                own = safety.footprint(self.arm.link.q())
                for t in out["things"]:
                    p = (T_arm @ [t["at_m"][0], t["at_m"][1], 0.0, 1.0])[:2]
                    if float(np.linalg.norm(own - p, axis=1).min()) < 0.04:
                        t["this_is"] = "the arm itself, or something under it"
            if T_arm is not None:                                # a touch is a better height than two cameras give
                for touch in touches:
                    if not touch["touched_something"]:
                        continue
                    near = [t for t in out["things"] if np.linalg.norm((T_arm @ [t["at_m"][0], t["at_m"][1], 0.0, 1.0])[:2] - touch["at_m"])
                            < landmarks.PLACE_TOL_M]
                    for t in near[:1]:
                        t["touched_height_m"] = touch["height_m"]
        frame = None
        if relative_to is not None and "things" in out:
            T, sigma = self._map_in(relative_to)
            if T is None:
                raise Refused(f"the map cannot be given in {relative_to}'s frame: {sigma} The map is in the rover's frame as it stood "
                              "when the map was begun; GET /place gives it so."
                              + (f" Also: {self.landmark_note}." if getattr(self, "landmark_note", None) else ""), 409)
            out = re_frame(out, T, f"{relative_to}'s own frame (x, y along the surface it stands on, metres); the map was laid into it "
                                   f"through what relates {relative_to} to the rover, which is known to about {sigma * 1000:.0f} mm, on top "
                                   "of the map's own error")
            out["placed_to_about_m"] = round(float(sigma), 4)
            frame = relative_to
        if touches:
            T_arm, _ = self._map_in("arm") if "things" in out else (None, "")
            if frame == "arm" or T_arm is None:                  # as the arm found them
                shown, where = touches, "the arm's own frame"
            else:                                                # into the frame the rest of the answer is in
                back = G.inv(T_arm) if frame is None else self._map_in(frame)[0] @ G.inv(T_arm)
                shown = [{**t, "at_m": [round(float(v), 3) for v in (back @ [t["at_m"][0], t["at_m"][1], 0.0, 1.0])[:2]]} for t in touches]
                where = "the same frame as the rest of this answer"
            out["touched"] = {"points": shown, "frame": where,
                              "note": "where the arm's tip was lowered until it stopped, and how high above the surface it stopped"}
        if getattr(self, "landmark_note", None):
            out["landmarks"] = self.landmark_note
        return out

    def map_from_rover(self) -> np.ndarray:
        """The rover's frame now, in the map's frame (the rover's frame as it stood when the map was begun)."""
        se2 = lambda p: G.pose(G.rot_z(float(p[2])), [float(p[0]), float(p[1]), 0.0])      # noqa: E731
        return G.inv(se2(self.map_origin)) @ se2(self.rover.pose)

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
        self.root.add(Node("place", "branch", "what has been found of the surroundings: the surface, things on it, and where "
                                              "nothing has looked yet", state=lambda: self.place_state(),
                           about={"picture": "GET /place/picture gives the map as a PNG: grey = never seen, light = seen free, "
                                             "red = something stands there (brighter = higher). The rover began at the centre, facing right."}))
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
        if self.rover is not None:
            rover = devices.add(self.rover.node("rover"))
            movers.links["rover"] = rover
            groups.add(Node("power", "group", "everything with a supply to watch")).links["rover_power"] = rover.children["power"]
            for side, cam in self.rover.cameras.items():
                node = rover.children["head"].add(self._camera_node(side, cam, f"the head's {side} camera: it moves with the head"))
                cams.links[f"rover_{side}"] = node
            rover.children["head"].commands["look"] = Command(
                "Take one pair of pictures with the head where it is and add what they show to the map (GET /place). For looking "
                "again at something, or from somewhere the rover has driven to. Raise and point the head first.", lambda: S.look(self))
        self.root.commands["wait"] = Command("Let time pass with every device holding still.", self.wait,
                                             {"seconds": Param("number", "how long", True, 0.0, 30.0)})
        if self.rover is not None:
            self.root.commands["survey"] = Command(
                "The rover looks all round from where it stands, without driving: its head turns through its range at a few "
                "heights, tilts and camera spacings, taking a pair of pictures at each. From them come the map (GET /place), "
                "the devices whose tags it saw (GET /where), and a check of its rulers. Takes a few minutes.",
                lambda: S.survey(self))
        self.root.commands["locate"] = Command(
            "Take a fresh full-size picture on every fixed camera and work out again where the devices are relative to each "
            "other, from the printed tags in view. With show_wrist_tag the arm also turns the tag behind its wrist camera "
            "toward several directions and a picture is taken at each: a camera that cannot see the arm's base can be "
            "located that way, and every camera is located better.", self.locate, {
                "show_wrist_tag": Param("boolean", "true: the arm moves through a few poses near its home to show its wrist tag"),
            })
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
        if self.rover is not None and self.rover.low_battery():
            self.rover.stop()
        return {"ok": True, "time_s": round(float(self.clock.now()), 3)}

    def stop(self, path: str = "") -> dict:
        return {"ok": True, "stopped": self.node(path).stop(), "time_s": round(float(self.clock.now()), 3)}

    def shutdown(self) -> str:
        """Leave every servo with torque off. For every way out of the program; never raises."""
        said = [d.shutdown() for d in (self.arm, self.rover) if d is not None]
        return "; ".join(said) if said else "no servos to switch off"

    # ---- where things are ------------------------------------------------------
    SHOW_POSES = ((-0.7, -1.5708), (0.0, -1.5708), (0.7, -1.5708), (-0.35, -0.6), (0.35, -2.2), (0.0, 0.3))   # (pan, wrist roll) [rad]

    def _look(self) -> dict:
        """One fresh full-size picture from every fixed camera, taken in by the locator."""
        q = self.arm.link.q() if self.arm is not None else None
        out = {}
        for name, cam in self.cameras.items():
            frame = cam.grab()
            out[name] = self.locator.see(name, frame.image, frame.t, q)
        if self.rover is not None and self.wrist is not None:      # the arm's own camera may have the rover in view
            frame = self.wrist.grab()
            self.locator.see_from_wrist(V.detect(frame.image), frame.t, q, self.wrist.lens(frame.image.shape[1], frame.image.shape[0]))
        if self.rover is not None and self.arm is not None:        # and the rover's cameras may have the arm in view
            head = self.rover.head_now()
            for side, cam in self.rover.cameras.items():
                frame = cam.grab()
                self.locator.see_from_rover(side, V.detect(frame.image), frame.t, head, q, self.rover.pose.copy(),
                                            cam.lens(frame.image.shape[1], frame.image.shape[0]))
        return out

    def locate(self, show_wrist_tag: bool = False) -> dict:
        """Look, and work out again where the devices are (hub/locator.py)."""
        if not self.cameras and self.rover is None:
            return {"ok": True, "cameras": {}, "note": "there is no fixed camera to look with", "time_s": round(float(self.clock.now()), 3)}
        seen = [self._look()]
        shown = 0
        if show_wrist_tag:
            if self.arm is None:
                raise Refused("there is no arm to show a tag with")
            self.arm.actions._need_torque()
            x, y, z = (0.20, 0.0, 0.16)                             # the arm's home, where it is known to have room
            self.arm.actions.move_tip(x, y, z, roll=self.arm.ctrl._roll_default(), timeout_s=6.0, tolerance_m=0.02)
            home = self.arm.ctrl.goal.copy()
            for pan, roll in self.SHOW_POSES:
                r = self.arm.actions.move_joints(shoulder_pan=float(np.clip(home[0] + pan, self.arm.ctrl.lo[0], self.arm.ctrl.hi[0])),
                                                 wrist_roll=float(np.clip(roll, self.arm.ctrl.lo[4], self.arm.ctrl.hi[4])), timeout_s=4.0)
                if self.arm.ctrl.last_block is not None:           # it met something: go no further that way
                    break
                self.arm.actions.wait(0.4)                         # at rest when the picture is taken
                seen.append(self._look())
                shown += 1 if r.get("ok") else 0
            self.arm.actions.move_joints(shoulder_pan=float(home[0]), wrist_roll=float(home[4]), timeout_s=4.0)
        return {"ok": True, **self.locator.summary(), "pictures_per_camera": len(seen), "wrist_tag_poses": shown,
                "tags_seen": {name: sorted({t for look in seen for t in look[name]["tags_seen"]}) for name in self.cameras},
                "time_s": round(float(self.clock.now()), 3)}

    def where(self, of: str, relative_to: str = "arm") -> dict:
        """Where one device is in another's frame, from what has been seen of the devices' tags.
        If nothing links them yet, one fresh look is taken first."""
        alias = {"wrist_camera": "arm/wrist_camera", "gripper": "arm/gripper", "rover/left": "rover/head/left", "rover/right": "rover/head/right"}
        of, relative_to = alias.get(of, of), alias.get(relative_to, relative_to)
        names = self.locator.names()
        for n in (of, relative_to):
            if n not in names:
                raise Refused(f"there is no device '{n}'; there are {names}", 404)
        if of == relative_to:
            raise Refused("of and relative_to are the same device")
        q = self.arm.link.q() if self.arm is not None else None
        out = self.locator.where(of, relative_to, q)
        if not out["known"] and (self.cameras or (self.rover is not None and self.arm is not None)):
            self._look()
            out = self.locator.where(of, relative_to, q)
        if not out["known"] and self.arm is not None:
            out["try"] = "POST /locate {\"show_wrist_tag\": true}: the arm shows the tag behind its wrist camera at several poses"
        return out
