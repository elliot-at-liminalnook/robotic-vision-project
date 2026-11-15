"""All a solution may touch: the hub's REST API."""
from __future__ import annotations

import json

import cv2
import numpy as np


class GaveUp(Exception):
    """The solution could not do the task, and says why."""


class OutOfReach(GaveUp):
    """The hand cannot be held over that place to look: whatever is there cannot be picked up either."""


class Missed(GaveUp):
    """The jaws closed where a thing was taken to be and there was nothing between them."""


class NotThere(GaveUp):
    """A closer look did not find what a first look took for a thing: not a failure of the arm, but news about that patch."""


class Api:
    def __init__(self, request):
        """request(method, url, body_bytes) -> (status, content type, body bytes): the hub's API, over a socket or in-process."""
        self._request = request
        self.calls = 0

    def _do(self, method: str, path: str, body=None):
        self.calls += 1
        status, ctype, out = self._request(method, path, b"" if body is None else json.dumps(body).encode())
        return status, (json.loads(out) if ctype.startswith("application/json") else out)

    def get(self, path: str):
        return self._do("GET", path)

    def post(self, path: str, body=None):
        return self._do("POST", path, body)

    def ok(self, method: str, path: str, body=None) -> dict:
        """The answer, or GaveUp with the hub's own words if it was refused."""
        status, r = self._do(method, path, body)
        if status != 200:
            raise GaveUp(f"{method} {path} -> {status}: {r.get('error', r) if isinstance(r, dict) else r}")
        return r

    def picture(self, node: str) -> np.ndarray:
        """A camera's full-size picture as RGB. node: e.g. /devices/camera_1 or /devices/arm/wrist_camera."""
        status, data = self.get(f"{node}/picture")
        if status != 200:
            raise GaveUp(f"no picture from {node}: {data}")
        return cv2.cvtColor(cv2.imdecode(np.frombuffer(bytes(data), np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)

    def devices(self) -> dict:
        return self.ok("GET", "/tree")["tree"]["under_it"]["devices"].get("under_it", {})
