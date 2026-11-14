"""Where the arm's parts are, from its joint angles and its own shape.

The arm's frame ("arm") is the one its commands use: origin at its base on the surface it stands
on, x forward, y to its left, z up. Its base *part* sits 2.4 mm above that origin.

    k = ArmKinematics()
    k.part(q, "base")                     # T_arm_part: a part's frame in the arm's frame
    k.wrist_camera(q)                     # T_arm_camera (camera frame: x right, y down, z forward)
    k.tag_corners(q)                      # every tag's four corners in the arm's frame, by id

Built-in knowledge: the arm's shape is its own (sim/model/so101_follower.xml), the same wherever it stands.
What comes out is only as good as the encoders, the servos' zeros and the links' stiffness: FK_SIGMA_M
and FK_SIGMA_RAD are what the hub allows for that at the wrist.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

from .. import builtin
from .. import geometry as G
from . import JOINTS
from .config import MODEL_DIR

FK_SIGMA_M = 0.004             # how well the wrist's place is known from the joint angles [m]
FK_SIGMA_RAD = math.radians(1.5)


class ArmKinematics:
    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path(str(MODEL_DIR / "so101_follower.xml"))
        self.d = mujoco.MjData(self.m)
        self.qadr = np.array([self.m.joint(n).qposadr[0] for n in JOINTS])
        self.tags = builtin.tags("arm")

    def _set(self, q):
        self.d.qpos[self.qadr] = np.asarray(q, float)
        mujoco.mj_forward(self.m, self.d)

    def part(self, q, name: str) -> np.ndarray:
        self._set(q)
        b = self.m.body(name).id
        return G.pose(self.d.xmat[b].reshape(3, 3), self.d.xpos[b])

    def wrist_camera(self, q) -> np.ndarray:
        self._set(q)
        c = self.m.camera("wrist").id
        return G.pose(self.d.cam_xmat[c].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0]), self.d.cam_xpos[c])

    def tag_corners(self, q) -> dict[int, np.ndarray]:
        """Each tag's four corners in the arm's frame at joint angles q, by tag id."""
        self._set(q)
        out = {}
        for i, tag in self.tags.items():
            b = self.m.body(tag.on).id
            out[i] = tag.corners() @ self.d.xmat[b].reshape(3, 3).T + self.d.xpos[b]
        return out

    def fixed_tags(self) -> set[int]:
        """The tags that do not move with the joints (those on the base)."""
        return {i for i, t in self.tags.items() if t.on == "base"}
