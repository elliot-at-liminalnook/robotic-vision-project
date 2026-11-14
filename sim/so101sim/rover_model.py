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
    """Add the rover standing on the surface at (x, y), facing yaw. Returns what was added."""
    half = [0.5 * s for s in SIZE_M]
    body = spec.worldbody.add_body(name=name, pos=[x, y, half[2] + CLEARANCE_M],
                                   quat=[math.cos(yaw_rad / 2), 0, 0, math.sin(yaw_rad / 2)])
    body.add_freejoint(name=f"{name}_free")
    body.add_geom(name=f"{name}_chassis", type=mujoco.mjtGeom.mjGEOM_BOX, size=half, mass=MASS_KG,
                  rgba=[0.25, 0.3, 0.38, 1], friction=[0.8, 0.005, 0.0001], **contact)
    return {"body": name, "free_joint": f"{name}_free", "footprint_m": [SIZE_M[0], SIZE_M[1]], "height_m": SIZE_M[2],
            "placeholder": True}
