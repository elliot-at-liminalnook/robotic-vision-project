"""Configuration: paths, camera specs (config/cameras.toml), servos (config/servos.toml)."""
from __future__ import annotations

import tomllib
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

SIM = Path(__file__).resolve().parents[1]
ROOT = SIM.parent
MODEL_DIR = SIM / "model"
WORKCELL = MODEL_DIR / "workcell.xml"


def _load(name: str) -> dict:
    with (SIM / "config" / name).open("rb") as f:
        return tomllib.load(f)


def look_at_axes(pos, look_at, up):
    """Camera axes (x right, y up, z back) for 'looks along -Z, +Y up'.

    MuJoCo and Newton's SensorCamera share this convention.
    """
    pos, look_at, up = (np.asarray(v, float) for v in (pos, look_at, up))
    f = look_at - pos
    f /= np.linalg.norm(f)
    z = -f
    x = np.cross(up, z)
    if np.linalg.norm(x) < 1e-6:
        raise ValueError("camera 'up' is parallel to the viewing direction")
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return x, y, z


@dataclass(frozen=True)
class CameraSpec:
    name: str
    parent_body: str            # "world" for fixed cameras
    pos: tuple                  # in the parent body frame [m]
    look_at: tuple
    up: tuple
    render_width: int
    render_height: int
    intrinsics: dict = field(hash=False)

    @property
    def rotation(self) -> np.ndarray:
        """3x3 camera-to-parent rotation, columns = camera x, y, z axes."""
        return np.column_stack(look_at_axes(self.pos, self.look_at, self.up))

    @property
    def fovy_deg(self) -> float:
        i = self.intrinsics
        return float(np.degrees(2 * np.arctan(0.5 * i["calib_height"] / i["fy"])))


def load_cameras(include_disabled: bool = False) -> dict[str, CameraSpec]:
    cfg = _load("cameras.toml")
    out = {}
    for name, c in cfg["cameras"].items():
        if not include_disabled and c.get("enabled", True) is False:
            continue
        out[name] = CameraSpec(
            name=name, parent_body=c["parent_body"], pos=tuple(c["pos"]),
            look_at=tuple(c["look_at"]), up=tuple(c["up"]),
            render_width=int(c["render_width"]), render_height=int(c["render_height"]),
            intrinsics=dict(cfg["intrinsics"][c["intrinsics"]]))
    return out


def load_servos() -> dict:
    return _load("servos.toml")
