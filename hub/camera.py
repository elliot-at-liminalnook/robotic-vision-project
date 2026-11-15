"""A camera as the hub has it: pictures through a port, and the nominal lens of its kind."""
from __future__ import annotations

import cv2
import numpy as np

from . import builtin
from .ports import Frame, PortError
from .vision import tags as V


class Camera:
    def __init__(self, port):
        self.port = port
        self.identity = dict(port.identity)
        self.kind = self.identity.get("kind", "cm3_wide")
        self.last: Frame | None = None
        self.pictures = 0
        self.focal_scale = 1.0           # this unit's focal length over its kind's nominal one, once found (hub/calibrate.py)
        self.focal_sigma = builtin.LENS_FOCAL_TOLERANCE      # ... and how well that is known

    @property
    def serial(self) -> str:
        return str(self.identity.get("serial", "?"))

    def nominal(self, width: int, height: int) -> builtin.Lens:
        """The nominal lens of this kind of camera at a picture size. Not this unit's true one."""
        return builtin.LENSES[self.kind if self.kind in builtin.LENSES else "cm3_wide"].at(width, height)

    def lens(self, width: int, height: int) -> builtin.Lens:
        """This camera's lens at a picture size, as well as the hub knows it: its kind's nominal values, with this unit's
        focal length where that has been found."""
        n = self.nominal(width, height)
        return n if self.focal_scale == 1.0 else builtin.Lens(n.width, n.height, n.fx * self.focal_scale, n.fy * self.focal_scale, n.cx, n.cy, n.dist)

    def grab(self) -> Frame:
        """A fresh full-size picture. Raises PortError if the camera is gone."""
        self.last = self.port.grab()
        self.pictures += 1
        return self.last

    def picture(self, width: int | None = None) -> Frame:
        """A fresh picture, scaled down to `width` if given (the whole field of view either way)."""
        f = self.grab()
        if width is None or width >= f.image.shape[1]:
            return f
        height = int(round(f.image.shape[0] * width / f.image.shape[1]))
        return Frame(cv2.resize(f.image, (int(width), height), interpolation=cv2.INTER_AREA), f.t)

    def tags(self, frame: Frame | None = None) -> list[dict]:
        """The printed tags in a full-size picture: id, corners [px], which kind of device carries that id."""
        frame = frame or self.grab()
        return [{"id": d.id, "corners_px": np.round(d.corners, 1).tolist(), "side_px": round(d.side_px, 1),
                 "on_a": builtin.kind_of_tag(d.id)} for d in V.detect(frame.image)]

    def state(self) -> dict:
        return {"serial": self.serial, "product": self.identity.get("product", ""), "pictures_taken": self.pictures,
                "last_picture_s": None if self.last is None else round(self.last.t, 3)}


def jpeg(image: np.ndarray, quality: int = 85) -> bytes:
    ok, buf = cv2.imencode(".jpg", cv2.cvtColor(image, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        raise PortError("the picture could not be encoded")
    return buf.tobytes()
