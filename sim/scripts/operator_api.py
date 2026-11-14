#!/usr/bin/env python3
"""The operator's REST API: scenes as files, the running world, and the referee.

    python sim/scripts/operator_api.py                     # http://127.0.0.1:8766, scene open_small loaded
    python sim/scripts/operator_api.py --scene shelf
    curl http://127.0.0.1:8766/                            # this guide
    curl http://127.0.0.1:8766/scenes
    curl -X PATCH http://127.0.0.1:8766/scenes/shelf -d '{"objects": {"red_cube": {"at_m": [0.5, 0.2]}}}'
    curl -X POST http://127.0.0.1:8766/world/load -d '{"scene": "shelf"}'
    curl http://127.0.0.1:8766/world/truth

This is for people, tests and scripts. It holds ground truth (a scene file says where everything is),
so it is on its own address and the model driving the hub is never given it
(docs/MULTI_DEVICE_PLAN.md, 5.1 and 5.8).

  * Every answer is JSON with "ok". When ok is false, "error" says why and nothing was changed.
  * A scene is checked before it is saved (so101sim.scenes.validate). One that is not a place
    that can exist is refused, with every problem listed in "problems".
  * Changing the scene that is loaded reloads it: the world starts again from the new file.
  * It listens on 127.0.0.1 only unless --host says otherwise, has no authentication, and serves
    one request at a time.
"""
from __future__ import annotations

import argparse
import copy
import json
import sys
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from urllib.parse import urlparse

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from so101sim import scenes  # noqa: E402
from so101sim.world import World  # noqa: E402

BY_NAME = ("fixtures", "objects", "devices", "regions", "tasks")      # lists whose items a PATCH addresses by name

# (method, path pattern, what it does). The guide and the dispatcher are both made from this table.
ROUTES = [
    ("GET", "/", "This guide."),
    ("GET", "/scenes", "The scene files: name, description, devices, tasks, and which one is loaded."),
    ("GET", "/scenes/{name}", "One scene, as data (the file's contents as JSON)."),
    ("PUT", "/scenes/{name}", "Create or replace a scene. Body: the whole scene. Checked first; refused with the problems if invalid."),
    ("PATCH", "/scenes/{name}", "Change part of a scene. Body: only what changes. Tables merge; in fixtures, objects, devices, "
                                "regions and tasks, address an item by its name: {\"objects\": {\"red_cube\": {\"at_m\": [0.5, 0.2]}}}. "
                                "null removes a field or an item; a new name adds an item."),
    ("DELETE", "/scenes/{name}", "Delete a scene file. Refused for the scene that is loaded."),
    ("POST", "/scenes/{name}/validate", "Check a scene without saving or loading it. Body: a scene to check instead of the file (optional)."),
    ("GET", "/world", "What is running: scene, generation (goes up with every load), simulated time, tasks."),
    ("POST", "/world/load", "Make a scene the running one. Body: {\"scene\": name}."),
    ("POST", "/world/reset", "The running scene back to its start."),
    ("POST", "/world/step", "Let simulated time pass. Body: {\"seconds\": s}, up to 60."),
    ("GET", "/world/truth", "The referee: where everything really is."),
    ("GET", "/world/tasks", "The running scene's tasks: name and request."),
    ("GET", "/world/tasks/{name}", "A task's goals and whether each holds now."),
    ("POST", "/world/tasks/{name}/answer", "Hand in answers for a task. Body: {key: value, ...}. The reply does not say whether they are right."),
]


class ApiError(Exception):
    def __init__(self, message: str, status: int = 400, problems: list[str] | None = None):
        super().__init__(message)
        self.status, self.problems = status, problems


def guide() -> str:
    lines = [__doc__.split("\n\n", 2)[0].strip(), "", "Endpoints"]
    lines += [f"  {m:6s} {p:28s} {what}" for m, p, what in ROUTES]
    lines += ["", "A scene file, field by field: sim/README.md, \"Scenes as files\".",
              "Task goals and the truth expressions answers are scored against: sim/so101sim/scenes/tasks.py."]
    return "\n".join(lines) + "\n"


def merge(scene: dict, patch: dict) -> dict:
    """The scene with a patch applied (see PATCH in ROUTES). Neither argument is changed."""
    out = copy.deepcopy(scene)
    for k, v in patch.items():
        if v is None:
            out.pop(k, None)
        elif k in BY_NAME and isinstance(v, dict):
            items = out.setdefault(k, [])
            for name, change in v.items():
                idx = next((i for i, it in enumerate(items) if it.get("name") == name), None)
                if change is None:
                    if idx is None:
                        raise ApiError(f"{k}: there is no '{name}' to remove", 404)
                    items.pop(idx)
                elif not isinstance(change, dict):
                    raise ApiError(f"{k}.{name}: must be a table of changes, or null to remove it")
                elif idx is None:
                    items.append({"name": name, **change})
                else:
                    items[idx] = merge(items[idx], change)
            if not items:
                out.pop(k)
        elif isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = merge(out[k], v)
        else:
            out[k] = v
    return out


class Operator:
    """The API's methods, one per endpoint. `world` is the running world."""

    def __init__(self, world: World):
        self.world = world

    @property
    def directory(self):
        return self.world.directory

    def _read(self, name: str) -> dict:
        try:
            return scenes.read(name, self.directory)
        except FileNotFoundError as e:
            raise ApiError(str(e), 404) from None
        except ValueError as e:
            raise ApiError(str(e)) from None

    def _save(self, name: str, scene: dict) -> dict:
        if not isinstance(scene, dict):
            raise ApiError("a scene must be a JSON object")
        scene = {**scene, "name": scene.get("name", name)}
        if scene["name"] != name:
            raise ApiError(f"the scene's name field ('{scene['name']}') must match its address ('{name}')")
        problems = scenes.validate(scene)
        if problems:
            raise ApiError(f"scene '{name}' is not valid, so nothing was saved", 422, problems)
        try:
            scenes.write(name, scene, self.directory, check=False)
        except ValueError as e:
            raise ApiError(str(e)) from None
        reloaded = self.world.name == name
        if reloaded:
            self.world.load(scene)
        return {"ok": True, "scene": name, "reloaded": reloaded, "generation": self.world.generation}

    # ---- scenes ------------------------------------------------------------
    def list_scenes(self) -> dict:
        out = []
        for n in scenes.names(self.directory):
            s = scenes.read(n, self.directory)
            out.append({"name": n, "description": s.get("description", ""), "loaded": n == self.world.name,
                        "devices": [d["kind"] for d in s.get("devices", [])], "tasks": [t["name"] for t in s.get("tasks", [])]})
        return {"ok": True, "scenes": out}

    def get_scene(self, name: str) -> dict:
        return {"ok": True, "scene": self._read(name)}

    def put_scene(self, name: str, body) -> dict:
        return self._save(name, body)

    def patch_scene(self, name: str, body) -> dict:
        if not isinstance(body, dict):
            raise ApiError("a patch must be a JSON object")
        return self._save(name, merge(self._read(name), body))

    def delete_scene(self, name: str) -> dict:
        if name == self.world.name:
            raise ApiError(f"scene '{name}' is the one loaded; load another before deleting it", 409)
        self._read(name)
        scenes.delete(name, self.directory)
        return {"ok": True, "deleted": name}

    def validate_scene(self, name: str, body) -> dict:
        scene = body if isinstance(body, dict) and body else self._read(name)
        problems = scenes.validate(scene)
        return {"ok": True, "valid": not problems, "problems": problems}

    # ---- the world ---------------------------------------------------------
    def world_info(self) -> dict:
        w = self.world
        return {"ok": True, "scene": w.name, "generation": w.generation, "time_s": round(w.time, 3), "tasks": w.tasks()}

    def load(self, body) -> dict:
        name = body.get("scene") if isinstance(body, dict) else None
        if not isinstance(name, str):
            raise ApiError("give the scene to load: {\"scene\": name}")
        scene = self._read(name)
        try:
            self.world.load(scene)
        except ValueError as e:
            raise ApiError(f"scene '{name}' is not valid, so it was not loaded", 422, str(e).split("; ")) from None
        return self.world_info()

    def reset(self) -> dict:
        self.world.reset()
        return self.world_info()

    def step(self, body) -> dict:
        s = body.get("seconds") if isinstance(body, dict) else None
        if isinstance(s, bool) or not isinstance(s, (int, float)) or not 0 <= s <= 60:
            raise ApiError("give how long: {\"seconds\": s}, from 0 to 60")
        self.world.step(float(s))
        return self.world_info()

    def truth(self) -> dict:
        return {"ok": True, **self.world.truth()}

    def tasks(self) -> dict:
        return {"ok": True, "scene": self.world.name,
                "tasks": [{"name": t["name"], "request": t["request"], "needs": t.get("needs", []),
                           **({"swap_to": t["swap_to"]} if "swap_to" in t else {})} for t in self.world.scene.get("tasks", [])]}

    def _find_task(self, name: str) -> dict | None:
        """The task of this name in the running scene, or one that swaps to the running scene (checked here)."""
        own = next((t for t in self.world.scene.get("tasks", []) if t["name"] == name), None)
        if own is not None and "swap_to" not in own:
            return own
        for n in scenes.names(self.directory):
            for t in scenes.read(n, self.directory).get("tasks", []):
                if t["name"] == name and t.get("swap_to") == self.world.name:
                    return t
        return own

    def task(self, name: str) -> dict:
        t = self._find_task(name)
        if t is None:
            raise ApiError(f"no task '{name}' in scene '{self.world.name}'; there are {self.world.tasks()}", 404)
        if t.get("swap_to") not in (None, self.world.name):
            return {"ok": True, "task": name, "scene": self.world.name, "request": t["request"], "done": False,
                    "waiting_for_swap_to": t["swap_to"], "goals": []}
        return {"ok": True, **self.world.task_status(name, t)}

    def answer(self, name: str, body) -> dict:
        if self._find_task(name) is None:
            raise ApiError(f"no task '{name}' in scene '{self.world.name}'; there are {self.world.tasks()}", 404)
        if not isinstance(body, dict) or not body:
            raise ApiError("hand in answers as {key: value, ...}")
        self.world.hand_in(name, body)
        return {"ok": True, "task": name, "received": sorted(body)}


def dispatch(op: Operator, method: str, url: str, body_bytes: bytes = b"") -> tuple[int, str, bytes]:
    """One request -> (status, content type, body). Never raises."""
    try:
        path = urlparse(url).path.rstrip("/") or "/"
        parts = [p for p in path.split("/") if p]
        try:
            body = json.loads(body_bytes.decode() or "null")
        except (ValueError, UnicodeDecodeError):
            raise ApiError("the request body is not valid JSON") from None
        table = {
            ("GET", ()): lambda: None,
            ("GET", ("scenes",)): op.list_scenes,
            ("GET", ("scenes", "*")): lambda: op.get_scene(parts[1]),
            ("PUT", ("scenes", "*")): lambda: op.put_scene(parts[1], body),
            ("PATCH", ("scenes", "*")): lambda: op.patch_scene(parts[1], body),
            ("DELETE", ("scenes", "*")): lambda: op.delete_scene(parts[1]),
            ("POST", ("scenes", "*", "validate")): lambda: op.validate_scene(parts[1], body),
            ("GET", ("world",)): op.world_info,
            ("POST", ("world", "load")): lambda: op.load(body),
            ("POST", ("world", "reset")): op.reset,
            ("POST", ("world", "step")): lambda: op.step(body),
            ("GET", ("world", "truth")): op.truth,
            ("GET", ("world", "tasks")): op.tasks,
            ("GET", ("world", "tasks", "*")): lambda: op.task(parts[2]),
            ("POST", ("world", "tasks", "*", "answer")): lambda: op.answer(parts[2], body),
        }
        fits = [sh for sh in {k[1] for k in table} if len(sh) == len(parts) and all(a in ("*", b) for a, b in zip(sh, parts))]
        shape = max(fits, key=lambda sh: sum(a != "*" for a in sh), default=None)     # a literal segment wins over a name
        if shape is None:
            raise ApiError(f"no such endpoint {path}; GET / lists them", 404)
        if (method, shape) not in table:
            allowed = sorted(m for m, s in table if s == shape)
            raise ApiError(f"{path} takes {' or '.join(allowed)}, not {method}", 405)
        if (method, shape) == ("GET", ()):
            return 200, "text/plain; charset=utf-8", guide().encode()
        return 200, "application/json", json.dumps(table[(method, shape)]()).encode()
    except ApiError as e:
        out = {"ok": False, "error": str(e), **({"problems": e.problems} if e.problems else {})}
        return e.status, "application/json", json.dumps(out).encode()
    except Exception as e:  # noqa: BLE001  (a bug here must reach the caller as JSON, not kill the server)
        return 500, "application/json", json.dumps({"ok": False, "error": f"internal error: {type(e).__name__}: {e}"}).encode()


def make_handler(op: Operator, verbose: bool = True):
    class Handler(BaseHTTPRequestHandler):
        def _answer(self, method: str):
            n = int(self.headers.get("Content-Length") or 0)
            status, ctype, body = dispatch(op, method, self.path, self.rfile.read(n) if 0 < n <= 4_000_000 else b"")
            self.send_response(status)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):       # noqa: N802
            self._answer("GET")

        def do_POST(self):      # noqa: N802
            self._answer("POST")

        def do_PUT(self):       # noqa: N802
            self._answer("PUT")

        def do_PATCH(self):     # noqa: N802
            self._answer("PATCH")

        def do_DELETE(self):    # noqa: N802
            self._answer("DELETE")

        def log_message(self, fmt, *args):
            if verbose:
                sys.stderr.write("%s  %s\n" % (self.log_date_time_string(), fmt % args))
    return Handler


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1", help="address to listen on (default: this machine only)")
    ap.add_argument("--port", type=int, default=8766)
    ap.add_argument("--scene", default="open_small", help="the scene loaded at the start (a file in sim/scenes/)")
    ap.add_argument("--scenes-dir", default=None, help="where the scene files are (default: sim/scenes/)")
    ap.add_argument("--quiet", action="store_true", help="don't log each request")
    a = ap.parse_args()
    try:
        world = World(a.scene, directory=a.scenes_dir)
    except (FileNotFoundError, ValueError) as e:
        ap.error(str(e))
    server = HTTPServer((a.host, a.port), make_handler(Operator(world), verbose=not a.quiet))
    print(f"operator API on http://{a.host}:{server.server_address[1]}/   scene '{world.name}'   (Ctrl-C to stop)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print()
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
