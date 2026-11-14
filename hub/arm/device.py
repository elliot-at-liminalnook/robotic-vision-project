"""The arm as a device in the hub: its place in the tree of interests, and its commands."""
from __future__ import annotations

import numpy as np

from ..errors import Refused
from ..tree import Command, Node, Param
from . import JOINTS
from . import controller as C
from .actions import ArmActions
from .link import ArmLink

NUM, BOOL = "number", "boolean"
ABOUT = {
    "frame": "The arm's own frame: origin at its base on the surface it stands on, x forward, y to its left, z up. "
             "Metres and radians.",
    "tip": "The 'tip' is the point midway between the two jaw pads at the height of the jaw tips. One jaw is fixed and "
           "one moves, so the tip shifts sideways by half the change in gap as the gripper opens and closes.",
    "reach": "Roughly 0.10 to 0.35 m from its base. The fingers point straight down below about 5 cm; straight-down "
             "reach above 12 cm exists only 0.20 to 0.32 m out.",
    "servos": "Soft: expect the arm a few millimetres from where it was sent, and check its state.",
    "tags": "Carries printed tags 0 to 7 on two blocks beside its base and tag 8 behind its wrist camera.",
}


class ArmDevice:
    kind = "arm"

    def __init__(self, port, clock):
        self.link = ArmLink(port, clock)
        self.ctrl: C.ArmController | None = None
        self.actions: ArmActions | None = None

    def start(self):
        """Identify the servos and take over, holding the pose the arm is in."""
        q = self.link.connect()
        self.ctrl = C.ArmController(self.link)
        self.ctrl.start(q)
        self.actions = ArmActions(self.ctrl)

    # ---- commands -------------------------------------------------------------
    def _home(self, timeout_s: float = 6.0) -> dict:
        x, y, z = C.HOME_TIP
        return self.actions.move_tip(x, y, z, roll=self.ctrl._roll_default(), timeout_s=timeout_s, tolerance_m=0.01)

    def _torque_off(self) -> dict:
        done = self.ctrl.set_torque(False)
        return {"ok": bool(done), "message": self.ctrl.message, "state": self.actions.state()} if done else \
            {"ok": False, "error": self.ctrl.message}

    def _torque_on(self) -> dict:
        if not self.ctrl.limp:
            raise Refused("the arm's torque is already on")
        done = self.ctrl.set_torque(True)
        return {"ok": bool(done), "message": self.ctrl.message, "state": self.actions.state()} if done else \
            {"ok": False, "error": self.ctrl.message}

    def stop(self):
        self.ctrl.freeze()

    def shutdown(self) -> str:
        return self.ctrl.shutdown() if self.ctrl is not None else "the arm was never started"

    def joints_state(self) -> dict:
        s = self.actions.state()
        return {k: s[k] for k in ("joints_rad", "joint_limits_rad", "servo_load_fraction_of_stall")}

    def gripper_state(self) -> dict:
        return self.actions.state()["gripper"]

    def own_state(self) -> dict:
        s = self.actions.state()
        return {k: s[k] for k in ("tip_m", "wrist_roll_rad", "jaws_close_along_deg", "moving", "torque", "message", "checks")}

    def node(self, name: str = "arm") -> Node:
        n = Node(name, "device", "SO-101 arm: five joints and a two-jaw gripper, with a camera at its wrist",
                 state=self.own_state, stop=self.stop, about={"device": "arm", **ABOUT})
        a = self.actions
        lo, hi = self.ctrl.lo, self.ctrl.hi
        n.commands["move_tip"] = Command(
            "Move the gripper tip in a straight line to a point in the arm's own frame, fingers pointing down.", a.move_tip, {
                "x": Param(NUM, "metres forward of the arm's base", True, -0.5, 0.7),
                "y": Param(NUM, "metres to the arm's left", True, -0.6, 0.6),
                "z": Param(NUM, f"metres above the surface the arm stands on (the tip is never taken below {C.MIN_TIP_Z})", True, -0.1, 0.6),
                "roll": Param(NUM, "wrist roll in radians, which turns the jaws about the vertical; omit to keep it",
                              minimum=round(float(lo[4]), 4), maximum=round(float(hi[4]), 4)),
                "square_jaws": Param(BOOL, "true: also turn the wrist so the jaws close along the arm's x or y axis, whichever is nearer"),
                "jaws_along_deg": Param(NUM, "instead of square_jaws: turn the wrist so the jaws close along this direction in the "
                                             "arm's x-y plane (0 = along x, 90 = along y), as near as the wrist's range allows", minimum=0.0, maximum=180.0),
                "timeout_s": Param(NUM, "give up after this much time", minimum=0.1, maximum=30, default=6.0),
                "tolerance_m": Param(NUM, "how close to the point counts as there", minimum=0.001, maximum=0.05, default=0.004),
            })
        n.commands["move_joints"] = Command(
            "Move joints to angles (radians). Joints not named keep their target.", a.move_joints, {
                **{j: Param(NUM, f"{j} angle in radians", minimum=round(float(lo[k]), 4), maximum=round(float(hi[k]), 4))
                   for k, j in enumerate(JOINTS[:5])},
                "timeout_s": Param(NUM, "give up after this much time", minimum=0.1, maximum=30, default=6.0),
            })
        n.commands["home"] = Command("Move the tip to the arm's home: 16 cm up, 20 cm ahead of its base, fingers down.", self._home,
                                     {"timeout_s": Param(NUM, "give up after this much time", minimum=0.1, maximum=30, default=6.0)})
        n.commands["torque_off"] = Command("Switch the servos' torque off. The arm goes limp and sags; nothing can be commanded "
                                           "until torque_on.", self._torque_off, takes_time=False)
        n.commands["torque_on"] = Command("Switch torque back on. The arm holds the pose it is in by then.", self._torque_on, takes_time=False)
        n.add(Node("joints", "part", "the arm's five joints and the gripper's: angles, limits, how hard each servo is working",
                   state=self.joints_state))
        g = n.add(Node("gripper", "part", "the two-jaw gripper", state=self.gripper_state))
        g.commands["set"] = Command(
            "Open or close the gripper (give opening or gap_m). Closing on something holds it gently.", a.gripper, {
                "opening": Param(NUM, "0 = closed, 1 = fully open", minimum=0.0, maximum=1.0),
                "gap_m": Param(NUM, "instead of opening: the distance between the jaw pads in metres", minimum=0.0,
                               maximum=round(float(a.max_gap), 3)),
                "timeout_s": Param(NUM, "give up after this much time", minimum=0.1, maximum=10, default=2.5),
            })
        return n

    def wiggle(self, back: bool = False) -> None:
        """A small turn of the wrist (and back), for finding which camera rides on it."""
        q = self.link.q()
        step = 0.3 if q[4] + 0.3 <= self.ctrl.hi[4] else -0.3
        if back:
            step = -step
        self.actions.move_joints(wrist_roll=float(np.clip(q[4] + step, self.ctrl.lo[4], self.ctrl.hi[4])), timeout_s=2.0)
