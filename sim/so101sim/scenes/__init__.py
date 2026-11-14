"""Scenes: places as config files (docs/MULTI_DEVICE_PLAN.md, 5.7).

    from so101sim import scenes
    scenes.names()                       # the files in sim/scenes/
    scene = scenes.read("workcell")      # a dict, as the file has it
    scenes.validate(scene)               # [] if it is a place that can exist
    built = scenes.build(scene)          # the MuJoCo model, in memory, plus what the referee needs
    scenes.write("mine", scene)          # save (checked first)

A scene file is ground truth. It belongs to the world and the referee; the hub never reads one.
"""
from __future__ import annotations

from pathlib import Path

from ..config import SIM
from . import schema, toml_io
from .build import Built, Region, SceneError, build
from .validate import validate

SCENE_DIR = SIM / "scenes"


def _path(name: str, directory=None) -> Path:
    if not name or not name.replace("_", "").isalnum():
        raise ValueError(f"'{name}' is not a scene name (letters, digits and _ only)")
    return Path(directory or SCENE_DIR) / f"{name}.toml"


def names(directory=None) -> list[str]:
    return sorted(p.stem for p in Path(directory or SCENE_DIR).glob("*.toml"))


def read(name: str, directory=None) -> dict:
    p = _path(name, directory)
    if not p.exists():
        raise FileNotFoundError(f"no scene '{name}'; there are {names(directory)}")
    return toml_io.load(p)


def write(name: str, scene: dict, directory=None, check: bool = True) -> Path:
    """Save a scene. With check, a scene that is not valid is refused (ValueError lists why)."""
    if scene.get("name") != name:
        raise ValueError(f"the scene's name field ('{scene.get('name')}') must match its file name ('{name}')")
    if check:
        problems = validate(scene)
        if problems:
            raise ValueError("; ".join(problems))
    p = _path(name, directory)
    toml_io.dump(scene, p)
    return p


def delete(name: str, directory=None):
    p = _path(name, directory)
    if not p.exists():
        raise FileNotFoundError(f"no scene '{name}'")
    p.unlink()


__all__ = ["SCENE_DIR", "Built", "Region", "SceneError", "build", "validate", "schema", "toml_io", "names", "read",
           "write", "delete"]
