"""Ports on the real machine: a USB serial adapter for an arm, a network link to a rover's board, cameras, the wall clock.

    hub = Hub(provider("machine.toml"))

The hub is given these and nothing else, exactly as in the simulation (sim/so101sim/world.py gives it the same
three kinds of port). What is plugged in is found, not listed: with no file every USB serial adapter and every
camera the system shows is offered, and the hub finds out which servos answer on which. A file is needed only
for what cannot be found by looking: a rover's board is somewhere on the network, and a rover's own cameras
have to say they are the rover's.

    [[serial]]
    tcp = "192.168.4.20:3333"            # a rover's board (firmware/rover-bridge)

    [[camera]]
    url = "http://192.168.4.21/stream"   # one of the rover's own cameras
    part_of = "rover"
    role = "left"

NOT YET TRIED ON HARDWARE. The network port is tested against a local server (sim/tests/test_real_ports.py);
the USB serial and camera ports are written from their libraries' documentation.
"""
from __future__ import annotations

import select
import socket
import time
import tomllib
from pathlib import Path

import cv2

from .ports import Frame, PortError, Ports

BAUD = 1_000_000


class WallClock:
    """Seconds since the hub started, from a clock that never goes backwards."""

    def __init__(self):
        self.t0 = time.monotonic()

    def now(self) -> float:
        return time.monotonic() - self.t0

    def sleep_until(self, t: float):
        wait = t - self.now()
        if wait > 0:
            time.sleep(wait)


class UsbSerial:
    """A servo bus on a USB adapter."""

    def __init__(self, device: str, baud: int = BAUD):
        try:
            import serial
        except ImportError:
            raise PortError("a USB serial port needs the pyserial package (pip install pyserial)") from None
        self.device = device
        try:
            self.s = serial.Serial(device, baud, timeout=0, write_timeout=0.2)
        except (serial.SerialException, OSError) as e:
            raise PortError(f"{device} could not be opened: {e}") from None

    def write(self, data: bytes):
        try:
            self.s.write(data)
            self.s.flush()
        except Exception as e:  # noqa: BLE001  (pyserial raises several kinds; all mean the adapter is gone)
            raise PortError(f"{self.device} is not there any more: {e}") from None

    def read(self, n: int, timeout_s: float) -> bytes:
        try:
            self.s.timeout = max(0.0, timeout_s)
            return self.s.read(n)
        except Exception as e:  # noqa: BLE001
            raise PortError(f"{self.device} is not there any more: {e}") from None

    def reset_input(self):
        try:
            self.s.reset_input_buffer()
        except Exception as e:  # noqa: BLE001
            raise PortError(f"{self.device} is not there any more: {e}") from None

    def close(self):
        self.s.close()


class TcpSerial:
    """A servo bus on the far side of a network link: a rover's board passes the bytes through (firmware/rover-bridge).
    `reply_time_s` tells a driver how much longer than on a wire a reply may take."""

    reply_time_s = 0.05

    def __init__(self, host: str, port: int, connect_timeout_s: float = 1.0):
        self.host, self.port = host, int(port)
        try:
            self.s = socket.create_connection((host, self.port), timeout=connect_timeout_s)
        except OSError as e:
            raise PortError(f"nothing answers at {host}:{port}: {e}") from None
        self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)      # servo packets are a few bytes: send each at once
        self.s.setblocking(False)
        self.gone = None

    def _lost(self, e):
        self.gone = f"the link to {self.host}:{self.port} is gone: {e}"
        raise PortError(self.gone) from None

    def write(self, data: bytes):
        if self.gone:
            raise PortError(self.gone)
        view, deadline = memoryview(data), time.monotonic() + 0.5
        while view:
            try:
                view = view[self.s.send(view):]
            except BlockingIOError:
                if time.monotonic() > deadline:
                    self._lost("it has stopped taking bytes")
                select.select([], [self.s], [], 0.05)
            except OSError as e:
                self._lost(e)

    def read(self, n: int, timeout_s: float) -> bytes:
        if self.gone:
            raise PortError(self.gone)
        out, deadline = bytearray(), time.monotonic() + max(0.0, timeout_s)
        while len(out) < n:
            wait = deadline - time.monotonic()
            ready, _, _ = select.select([self.s], [], [], max(0.0, wait))
            if not ready:
                break                                               # fewer than n: the time allowed has passed
            try:
                got = self.s.recv(n - len(out))
            except BlockingIOError:
                continue
            except OSError as e:
                self._lost(e)
            if not got:
                self._lost("the other end closed it")
            out += got
        return bytes(out)

    def reset_input(self):
        if self.gone:
            raise PortError(self.gone)
        try:
            while select.select([self.s], [], [], 0.0)[0]:
                if not self.s.recv(4096):
                    self._lost("the other end closed it")
        except BlockingIOError:
            pass
        except OSError as e:
            self._lost(e)

    def close(self):
        self.gone = self.gone or f"the link to {self.host}:{self.port} was closed (ports are opened afresh at each discovery)"
        try:
            self.s.close()
        except OSError:
            pass


class UvcCamera:
    """A camera the system shows as a video device (an ESP32-P4 camera board over USB), or a stream at a URL."""

    def __init__(self, source, clock: WallClock, kind: str = "cm3_wide", part_of: str | None = None, role: str | None = None,
                 width: int = 1920, height: int = 1080):
        self.source, self.clock = source, clock
        self.cap = cv2.VideoCapture(source)
        if not self.cap.isOpened():
            raise PortError(f"camera {source} could not be opened")
        if isinstance(source, int):
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # What the camera says it is. A video device does not give its serial number through this library, so the system's
        # name for it stands in: it tells one camera from another, and says nothing about where either is.
        self.identity = {"serial": f"video-{source}", "product": "UVC camera", "kind": kind,
                         **({"part_of": part_of, "role": role} if part_of else {})}

    def grab(self) -> Frame:
        for _ in range(3):                                          # (frames taken before now may be waiting: pass them by)
            self.cap.grab()
        ok, bgr = self.cap.read()
        if not ok or bgr is None:
            raise PortError(f"camera {self.source} is not there any more")
        return Frame(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), self.clock.now())

    def close(self):
        self.cap.release()


def usb_serial_devices() -> list[str]:
    """Every USB serial adapter the system shows."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return sorted(p.device for p in list_ports.comports() if p.vid is not None)


def provider(config: str | Path | None = None, find_usb: bool = True, find_cameras: int = 4):
    """A function that says what is plugged in now (hub.ports.PortProvider). Ports opened at one asking are closed at the next.

    config: a file naming what cannot be found by looking (see the top of this file), or None.
    find_usb: offer every USB serial adapter. find_cameras: try this many of the system's video devices (0: none).
    """
    cfg = tomllib.loads(Path(config).read_text()) if config else {}
    clock = WallClock()
    opened: list = []
    notes: list[str] = []

    def ports() -> Ports:
        for p in opened:
            p.close()
        opened.clear()
        notes.clear()
        serial, cameras = [], []

        def add(make, to: list, what: str):
            try:
                port = make()
            except PortError as e:
                notes.append(f"{what}: {e}")                        # not there today: said, not fatal
                return
            opened.append(port)
            to.append(port)

        named = {s["device"] for s in cfg.get("serial", []) if "device" in s}
        for entry in cfg.get("serial", []):
            if "tcp" in entry:
                host, _, port = entry["tcp"].rpartition(":")
                add(lambda h=host, p=port: TcpSerial(h, int(p)), serial, f"the link to {entry['tcp']}")
            elif "device" in entry:
                add(lambda d=entry["device"]: UsbSerial(d, int(entry.get("baud", BAUD))), serial, entry["device"])
        if find_usb:
            for dev in usb_serial_devices():
                if dev not in named:
                    add(lambda d=dev: UsbSerial(d), serial, dev)
        for entry in cfg.get("camera", []):
            src = entry.get("url", entry.get("device"))
            add(lambda s=src, e=entry: UvcCamera(s, clock, e.get("kind", "cm3_wide"), e.get("part_of"), e.get("role")), cameras, f"camera {src}")
        listed = {c.get("device") for c in cfg.get("camera", [])}
        for index in range(find_cameras):
            if index not in listed:
                add(lambda i=index: UvcCamera(i, clock), cameras, f"camera {index}")
        return Ports(clock=clock, serial=serial, cameras=cameras)

    ports.notes = notes                                             # what was looked for and not found, at the last asking
    return ports
