"""Tasks: what is to be done in a scene, and whether it has been.

A task is a description for whoever is driving (a person, a script, a language model) and a list of
goals, each a check of the simulation's state. Goals are about the end state only, so they can be
evaluated at any moment and in any order; a task is done when all of them hold at once.

    task = TASKS["bench"](model)
    task.description
    task.check(data) -> [{"goal": ..., "done": bool, "detail": ...}, ...]

"bench" (sim/scene/bench.xml) is feasible: sim/tests/test_tasks.py solves it through the REST API.
"""
from __future__ import annotations

import mujoco
import numpy as np


class Task:
    name = ""
    description = ""

    def __init__(self, m: mujoco.MjModel):
        self.m = m

    def pos(self, d, body: str) -> np.ndarray:
        return d.xpos[self.m.body(body).id]

    def upright(self, d, body: str) -> bool:
        return bool(d.xmat[self.m.body(body).id].reshape(3, 3)[2, 2] > 0.95)

    def flat(self, d, body: str) -> bool:
        """Is the body's own z axis vertical (either way up)?"""
        return bool(abs(d.xmat[self.m.body(body).id].reshape(3, 3)[2, 2]) > 0.95)

    def speed(self, d, body: str) -> float:
        j = self.m.body(body).jntadr[0]
        return float(np.linalg.norm(d.qvel[self.m.jnt_dofadr[j]:self.m.jnt_dofadr[j] + 3]))

    def goals(self, d) -> list[tuple[str, bool, str]]:
        raise NotImplementedError

    def check(self, d) -> list[dict]:
        return [{"goal": g, "done": bool(ok), "detail": detail} for g, ok, detail in self.goals(d)]

    def done(self, d) -> bool:
        return all(g["done"] for g in self.check(d))


def _mm(v) -> str:
    return "(" + ", ".join(f"{x * 1000:.0f}" for x in v) + ") mm"


class BenchTask(Task):
    name = "bench"
    description = """Tidy the bench. All of these must be true at the same time:
  1. The red cube is in the red bin. (It starts inside the closed drawer of the grey cabinet. The drawer
     slides out toward the robot, up to 85 mm; its handle is the dark post on top of its front panel.)
  2. The blue cylinder stands upright in the blue bin.
  3. The yellow peg stands in the square hole of the yellow socket.
  4. The green block lies flat (on its largest face) on the grey platform.
  5. The purple cube sits on top of the green block. (It starts on the cabinet's roof.)
  6. The drawer is closed again.
  7. Nothing is held and everything is at rest.
Where things are (metres; x forward of the robot, y to its left, z above the table):
  cabinet: 0.235 to 0.319 in x, 0.085 to 0.175 in y, roof top at z 0.052. Drawer handle (closed) at (0.224, 0.13),
    12 mm square, from z 0.037 to 0.067.
  red bin centre (0.10, 0.24); blue bin centre (0.13, -0.23); each 56 mm square inside, walls 14 mm high.
  platform centre (0.27, -0.14), 80 mm square, top at z 0.040.
  socket centre (0.30, -0.02), hole 26 mm square, 25 mm deep; the peg is 18 mm across and 60 mm long.
The arm cannot hold its fingers straight down higher than about 10 cm when the tip is closer than 0.20 m to its base.
Sizes: red cube 30 mm; purple cube 25 mm; blue cylinder 30 mm across, 60 mm tall; green block 40 x 25 x 20 mm."""

    RED_BIN, BLUE_BIN = np.array([0.10, 0.24]), np.array([0.13, -0.23])
    PLATFORM, SOCKET = np.array([0.27, -0.14]), np.array([0.30, -0.02])
    HANDLE_CLOSED = np.array([0.224, 0.13])
    PARTS = ("red_cube", "purple_cube", "blue_cylinder", "yellow_peg", "green_block")

    def goals(self, d):
        cube, purple = self.pos(d, "red_cube"), self.pos(d, "purple_cube")
        cyl, peg, block = self.pos(d, "blue_cylinder"), self.pos(d, "yellow_peg"), self.pos(d, "green_block")
        drawer = float(d.qpos[self.m.joint("drawer").qposadr[0]])
        moving = [p for p in self.PARTS if self.speed(d, p) > 0.01]
        return [
            ("red cube in the red bin",
             np.abs(cube[:2] - self.RED_BIN).max() < 0.013 and cube[2] < 0.02, f"red cube at {_mm(cube)}"),
            ("blue cylinder upright in the blue bin",
             np.abs(cyl[:2] - self.BLUE_BIN).max() < 0.013 and abs(cyl[2] - 0.03) < 0.004 and self.upright(d, "blue_cylinder"),
             f"blue cylinder at {_mm(cyl)}, {'upright' if self.upright(d, 'blue_cylinder') else 'not upright'}"),
            ("yellow peg in the socket",
             np.abs(peg[:2] - self.SOCKET).max() < 0.005 and abs(peg[2] - 0.03) < 0.004 and self.upright(d, "yellow_peg"),
             f"yellow peg at {_mm(peg)}, {'upright' if self.upright(d, 'yellow_peg') else 'not upright'}"),
            ("green block on the platform",
             np.abs(block[:2] - self.PLATFORM).max() < 0.028 and abs(block[2] - 0.05) < 0.002 and self.flat(d, "green_block"),
             f"green block at {_mm(block)}, {'lying flat' if self.flat(d, 'green_block') else 'not lying flat'}"),
            ("purple cube on the green block",
             np.abs(purple[:2] - block[:2]).max() < 0.012 and abs(purple[2] - block[2] - 0.0225) < 0.002,
             f"purple cube at {_mm(purple)}, green block at {_mm(block)}"),
            ("drawer closed", drawer < 0.005, f"drawer open by {drawer * 1000:.0f} mm"),
            ("everything at rest", not moving, "still moving: " + ", ".join(moving) if moving else "nothing is moving"),
        ]


TASKS = {"bench": BenchTask}
