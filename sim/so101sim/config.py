"""Configuration: paths, camera specs (config/cameras.toml), servos (config/servos.toml)."""
from __future__ import annotations

import json
import tomllib
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

SIM = Path(__file__).resolve().parents[1]
ROOT = SIM.parent
MODEL_DIR = SIM / "model"
WORKCELL = MODEL_DIR / "workcell.xml"
WORKCELL_CABLES = MODEL_DIR / "workcell_cables.xml"     # the same, with the arm's wires (so101sim/cables.py)


from hub.arm.config import (load_safety, load_servos, load_wrist_mount, position_loop, servo_limits,  # noqa: E402,F401
                            wrist_roll_range)             # the arm's own configuration lives with its driver


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


def resolve_camera_entry(c: dict) -> dict:
    """cameras.toml entry with 'pose_from' replaced by the pose it points to."""
    if "pose_from" not in c:
        return c
    pose = json.loads((ROOT / c["pose_from"]).read_text())["camera"]
    return {**c, "parent_body": pose["parent_body"], "pos": pose["pos"],
            "look_at": pose["look_at"], "up": pose["up"]}


def model_timestep(model_path=WORKCELL) -> float:
    """<option timestep> of an MJCF file (MuJoCo's default 2 ms if unset)."""
    import xml.etree.ElementTree as ET
    opt = ET.parse(model_path).getroot().find("option")
    return float(opt.get("timestep", 0.002)) if opt is not None else 0.002


def load_cameras(include_disabled: bool = False) -> dict[str, CameraSpec]:
    cfg = _load("cameras.toml")
    out = {}
    for name, c in cfg["cameras"].items():
        if not include_disabled and c.get("enabled", True) is False:
            continue
        c = resolve_camera_entry(c)
        out[name] = CameraSpec(
            name=name, parent_body=c["parent_body"], pos=tuple(c["pos"]),
            look_at=tuple(c["look_at"]), up=tuple(c["up"]),
            render_width=int(c["render_width"]), render_height=int(c["render_height"]),
            intrinsics=dict(cfg["intrinsics"][c["intrinsics"]]))
    return out


