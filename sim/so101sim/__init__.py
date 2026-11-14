"""SO-101 follower simulation: Newton (MuJoCo Warp) runtime plus a MuJoCo backend
for checks on machines without Newton.

    from so101sim import load_backend
    sim = load_backend("newton")          # or "mujoco"
    sim.reset(); sim.set_targets(q); sim.step(n); images = sim.render()
"""
import sys as _sys
from pathlib import Path as _Path

if str(_Path(__file__).resolve().parents[2]) not in _sys.path:      # the hub package is at the repo's top level
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))

from hub.arm import ARM_JOINTS, JOINTS  # noqa: E402  (the arm's controller-side code lives in hub/arm)
from hub.arm.servo import ServoMap, SlewLimiter  # noqa: E402

from .config import ROOT, SIM, MODEL_DIR, WORKCELL, CameraSpec, load_cameras, load_servos  # noqa: E402


def load_backend(name: str, **kwargs):
    """'newton' (Linux; MuJoCo Warp on CUDA, MuJoCo C on CPU) or 'mujoco'."""
    if name == "newton":
        from .newton_backend import NewtonBackend
        return NewtonBackend(**kwargs)
    if name == "mujoco":
        from .mujoco_backend import MujocoBackend
        return MujocoBackend(**kwargs)
    raise ValueError(f"unknown backend {name!r}")


__all__ = ["ROOT", "SIM", "MODEL_DIR", "WORKCELL", "CameraSpec", "load_cameras", "load_servos",
           "ServoMap", "SlewLimiter", "ARM_JOINTS", "JOINTS", "load_backend"]
