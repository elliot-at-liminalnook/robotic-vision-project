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


def load_wrist_mount() -> dict:
    """The wrist camera mount placement (hardware/wrist_mount/wrist_mount_pose.json)."""
    return json.loads((ROOT / "hardware/wrist_mount/wrist_mount_pose.json").read_text())


def wrist_roll_range() -> tuple[float, float]:
    """Wrist roll range that keeps the side-mounted camera clear of the arm.

    Spans pi, so every top-down jaw orientation is still reachable.
    """
    lo, hi = load_wrist_mount()["wrist_roll_range_rad"]
    return float(lo), float(hi)


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


def load_servos() -> dict:
    return _load("servos.toml")


def servo_limits(arm: str = "follower") -> dict:
    """Servo torque/speed at the configured supply voltage (DC motor: both scale with voltage).

    Returns stall_torque_nm, no_load_speed_rad_s, gripper_torque_nm (torque-limit
    register applied), rated values and the voltages.
    """
    f = load_servos()[arm]
    k = float(f["supply_voltage_v"]) / float(f["rated_voltage_v"])
    stall = float(f["torque_limit_nm"]) * k
    return dict(stall_torque_nm=stall, no_load_speed_rad_s=float(f["max_speed_rad_s"]) * k,
                gripper_torque_nm=stall * float(f["gripper_torque_limit_frac"]),
                rated_stall_torque_nm=float(f["torque_limit_nm"]), rated_speed_rad_s=float(f["max_speed_rad_s"]),
                supply_voltage_v=float(f["supply_voltage_v"]), rated_voltage_v=float(f["rated_voltage_v"]),
                voltage_scale=k)
