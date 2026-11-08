"""SO-101 follower simulation: Newton (MuJoCo Warp) runtime plus a MuJoCo backend
for checks on machines without Newton.

    from so101sim import load_backend
    sim = load_backend("newton")          # or "mujoco"
    sim.reset(); sim.set_targets(q); sim.step(n); images = sim.render()
"""
from .config import ROOT, SIM, MODEL_DIR, WORKCELL, CameraSpec, load_cameras, load_servos
from .servo import ServoMap, SlewLimiter

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
JOINTS = ARM_JOINTS + ("gripper",)


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
