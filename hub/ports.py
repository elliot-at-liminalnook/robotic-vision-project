"""Ports: the only way anything gets into or out of the hub.

On the real machine a serial port is a USB adapter (or a network link to a rover), a camera port is
a camera stream, and the clock is the wall clock. In the simulation the world provides all three
(sim/so101sim/world.py). The hub cannot tell which it has, and is given nothing else.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Protocol

import numpy as np


class PortError(Exception):
    """A port that is not there any more, or does not answer."""


class SerialPort(Protocol):
    def write(self, data: bytes) -> None: ...
    def read(self, n: int, timeout_s: float) -> bytes: ...          # up to n bytes; fewer means the timeout passed
    def reset_input(self) -> None: ...


class Clock(Protocol):
    def now(self) -> float: ...                                     # seconds
    def sleep_until(self, t: float) -> None: ...


@dataclass
class Frame:
    image: np.ndarray           # RGB, as the camera delivered it (through its lens, JPEG and all)
    t: float                    # when it was taken, on the clock


class CameraPort(Protocol):
    identity: dict              # what the camera says it is: {"serial": ..., "product": ..., "kind": ...}. Nothing about where it is

    def grab(self) -> Frame: ...                                    # one fresh full-size picture; takes time on the clock


@dataclass
class Ports:
    """What is plugged in right now."""
    clock: Clock
    serial: list = field(default_factory=list)
    cameras: list = field(default_factory=list)


PortProvider = Callable[[], Ports]      # asked again at every discovery: things get plugged in and unplugged
