"""Scene files on disk: TOML in, TOML out.

Reading is the standard library's. Writing is a small writer for the shapes a scene uses
(tables, arrays of tables, strings, numbers, booleans, arrays of those). A file rewritten here
loses its comments: an edit over the operator API costs that (docs/MULTI_DEVICE_PLAN.md, 5.7).
"""
from __future__ import annotations

import tomllib
from pathlib import Path


def loads(text: str) -> dict:
    return tomllib.loads(text)


def load(path) -> dict:
    with Path(path).open("rb") as f:
        return tomllib.load(f)


def _scalar(v) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        if v != v or v in (float("inf"), float("-inf")):
            raise ValueError("a scene file cannot hold a number that is not finite")
        return repr(round(v, 9))
    if isinstance(v, str):
        return '"' + v.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n") + '"'
    if isinstance(v, (list, tuple)):
        return "[" + ", ".join(_scalar(x) for x in v) + "]"
    if isinstance(v, dict):                       # inline table (only used inside arrays of scalars)
        return "{ " + ", ".join(f"{k} = {_scalar(x)}" for k, x in v.items()) + " }"
    raise TypeError(f"cannot write {type(v).__name__} to a scene file")


def _is_table_array(v) -> bool:
    return isinstance(v, list) and len(v) > 0 and all(isinstance(x, dict) for x in v)


def _emit(d: dict, path: list[str], out: list[str]):
    plain = {k: v for k, v in d.items() if not isinstance(v, dict) and not _is_table_array(v)}
    for k, v in plain.items():
        out.append(f"{k} = {_scalar(v)}")
    for k, v in d.items():
        if isinstance(v, dict):
            out.append("")
            out.append("[" + ".".join(path + [k]) + "]")
            _emit(v, path + [k], out)
        elif _is_table_array(v):
            for item in v:
                out.append("")
                out.append("[[" + ".".join(path + [k]) + "]]")
                _emit(item, path + [k], out)


def dumps(scene: dict) -> str:
    out: list[str] = []
    _emit(scene, [], out)
    return "\n".join(out).lstrip("\n") + "\n"


def dump(scene: dict, path):
    Path(path).write_text(dumps(scene))
