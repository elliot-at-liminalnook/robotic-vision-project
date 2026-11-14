"""The rover in the simulation.

PLACEHOLDER until the rover is designed (docs/MULTI_DEVICE_CHECKLIST.md, step 7): one free box of
about the estimated size and weight, so that scenes with a rover can be built and validated.
"""
from __future__ import annotations

import math

import mujoco

SIZE_M = (0.18, 0.14, 0.08)        # length, width, height: the plan's estimate
MASS_KG = 0.9
CLEARANCE_M = 0.0005


def add_rover(spec: mujoco.MjSpec, name: str, x: float, y: float, yaw_rad: float, contact: dict) -> dict:
    """Add the rover standing on the surface at (x, y), facing yaw. Returns what was added.

    It stands on four small round feet, not on its flat underside: a large flat box lying on the
    deep table box now and then got a false contact 0.3 m deep from MuJoCo's box-box test and was
    thrown off the table (seen at one position and heading in the `cluttered` scene)."""
    half = [0.5 * s for s in SIZE_M]
    foot = 0.006
    half[2] -= foot / 2
    body = spec.worldbody.add_body(name=name, pos=[x, y, half[2] + foot + CLEARANCE_M],
                                   quat=[math.cos(yaw_rad / 2), 0, 0, math.sin(yaw_rad / 2)])
    body.add_freejoint(name=f"{name}_free")
    body.add_geom(name=f"{name}_chassis", type=mujoco.mjtGeom.mjGEOM_BOX, size=half, mass=MASS_KG,
                  rgba=[0.25, 0.3, 0.38, 1], friction=[0.8, 0.005, 0.0001], **contact)
    for k, (ex, ey) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        body.add_geom(name=f"{name}_foot{k}", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[foot, 0, 0], mass=0.001,
                      pos=[ex * (half[0] - foot), ey * (half[1] - foot), -half[2]], rgba=[0.1, 0.1, 0.1, 1],
                      friction=[0.8, 0.005, 0.0001], **contact)
    return {"body": name, "free_joint": f"{name}_free", "footprint_m": [SIZE_M[0], SIZE_M[1]], "height_m": SIZE_M[2],
            "placeholder": True}
