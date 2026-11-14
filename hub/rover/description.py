"""What the rover is: its shape, its joints and servos. Built-in knowledge (hardware/rover/rover.json),
the same wherever the rover is put down. The hub checks it against what it sees."""
from __future__ import annotations

import json
import math
from functools import lru_cache
from pathlib import Path

import numpy as np

from .. import builtin
from .. import geometry as G

ROVER_JSON = Path(__file__).resolve().parents[2] / "hardware" / "rover" / "rover.json"
HEAD_JOINTS = ("pan", "lift", "tilt", "spacing")
STOWED = {"pan": 0.0, "lift": math.radians(-15.0), "tilt": 0.0, "spacing": 0.0}      # how it drives: head forward and low, cameras together


@lru_cache(maxsize=None)
def assembly() -> dict:
    return json.loads(ROVER_JSON.read_text())


class Rover:
    """The rover's own geometry. Frame "rover": origin on the surface under the middle of its wheel axle,
    x forward, y left, z up."""

    def __init__(self):
        a = assembly()
        self.wheel_radius, self.track, self.pitch_radius = a["wheel_radius_m"], a["track_m"], a["pitch_radius_m"]
        self.bodies = {b["name"]: b for b in a["bodies"]}
        self.joints = {b["joint"]["name"]: {**b["joint"], "body": b["name"]} for b in a["bodies"] if b["joint"]}
        self.limits = {j: (self.joints[j].get("range_rad") or self.joints[j].get("range_m")) for j in HEAD_JOINTS}
        self.tags = builtin.tags("rover")

    def servo(self, joint: str) -> dict:
        j = self.joints[joint]
        return {"id": int(j["servo"]), "direction": int(j.get("direction", 1)), "ratio": abs(float(j["ratio"])),
                "zero_ticks": int(j.get("zero_ticks", 2048))}

    # ---- where its parts are -------------------------------------------------
    def _value(self, joint: dict, q: dict) -> float:
        if "coupled_to" in joint:
            return float(q.get(joint["coupled_to"], 0.0)) * float(joint["ratio"])
        return float(q.get(joint["name"], 0.0))

    def body(self, name: str, q: dict) -> np.ndarray:
        """T_rover_body at head joint values q (pan, lift, tilt [rad]; spacing [m, each camera from its closest])."""
        chain = []
        while name is not None:
            chain.append(self.bodies[name])
            name = self.bodies[name]["parent"]
        T, prev = np.eye(4), np.zeros(3)
        for b in reversed(chain):
            o = np.array(b["origin_m"])
            T = T @ G.pose(None, o - prev)
            prev = o
            j = b["joint"]
            if j is not None:
                v, axis = self._value(j, q), np.array(j["axis"], float)
                T = T @ (G.pose(G.rodrigues(axis * v)) if j["type"] == "hinge" else G.pose(None, axis * v))
        return T

    def camera(self, side: str, q: dict) -> np.ndarray:
        """T_rover_camera for "left" or "right" (camera frame: x right, y down, z forward)."""
        b = self.bodies[f"carriage_{side}"]
        cam = b["cameras"][0]
        z, up = np.array(cam["looks"], float), np.array(cam["up"], float)
        R = np.column_stack([np.cross(z, up), -up, z])
        return self.body(b["name"], q) @ G.pose(R, cam["lens_m"])

    def tag_corners(self, q: dict) -> dict[int, np.ndarray]:
        """Each of the rover's tags' four corners in the rover's frame, by id."""
        out = {}
        for i, tag in self.tags.items():
            T = self.body(tag.on, q)
            out[i] = tag.corners() @ T[:3, :3].T + T[:3, 3]
        return out

    def fixed_tags(self) -> set[int]:
        return {i for i, t in self.tags.items() if t.on == "chassis"}

    def lowest(self, q: dict) -> float:
        """Height of the lowest point of the head's parts above the surface the rover stands on [m]."""
        low = np.inf
        for name in ("arm", "head_block", "head", "carriage_left", "carriage_right"):
            T = self.body(name, q)
            for c in self.bodies[name]["collision"]:
                if c["type"] != "box":
                    continue
                lo, hi = np.array(c["lo"]), np.array(c["hi"])
                for corner in ((x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])):
                    low = min(low, float((T @ np.array([*corner, 1.0]))[2]))
        return low

    def lens_height(self, q: dict) -> float:
        return float(self.camera("left", q)[2, 3])

    def lens_spacing(self, q: dict) -> float:
        return float(np.linalg.norm(self.camera("left", q)[:3, 3] - self.camera("right", q)[:3, 3]))

    def lift_for_height(self, height_m: float) -> float:
        """The lift angle that puts the lenses at a height with the head level (clipped to the lift's range)."""
        lo, hi = self.limits["lift"]
        grid = np.linspace(lo, hi, 211)
        heights = np.array([self.lens_height({"lift": a}) for a in grid])
        return float(grid[int(np.abs(heights - height_m).argmin())])
