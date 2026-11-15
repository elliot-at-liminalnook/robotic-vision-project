"""The rover board's firmware in the loop: firmware/rover-bridge/main/bridge.c, compiled for this PC and run
between the hub's radio link and the rover's emulated servo bus.

    fw = RoverFirmware(wheel_ids=(11, 12), speed_addr=46, speed_size=2, watchdog_ms=400)
    to_bus = fw.from_hub(data, now_ms)        # what the board writes to the bus (the same bytes)
    to_hub = fw.from_bus(data, now_ms)
    stop = fw.tick(now_ms)                    # b"" most of the time; the stop packet when the hub has gone quiet

The same C runs on the board (rover_bridge_main.c is the part that is not built here: Wi-Fi, the socket,
the UART). What is tested here is everything the board decides by itself.
"""
from __future__ import annotations

import ctypes
import hashlib
import os
import shutil
import subprocess
import sys
from pathlib import Path

from .config import ROOT, SIM

FW_DIR = ROOT / "firmware" / "rover-bridge"
CACHE = SIM / ".cache" / "rover_bridge"
SOURCES = (FW_DIR / "host" / "host_api.c", FW_DIR / "main" / "bridge.c", FW_DIR / "main" / "bridge.h")


class FirmwareBuildError(RuntimeError):
    pass


def compiler() -> str | None:
    return os.environ.get("CC") or shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")


def build() -> Path:
    """Compile the firmware's core for this PC (cached by its source). Warnings are errors: it is small enough to be clean."""
    cc = compiler()
    if cc is None:
        raise FirmwareBuildError("no C compiler found (set CC, or install cc/gcc/clang)")
    h = hashlib.sha256()
    for f in SOURCES:
        h.update(f.read_bytes())
    out = CACHE / f"rover_bridge_{h.hexdigest()[:16]}{'.dylib' if sys.platform == 'darwin' else '.so'}"
    if not out.exists():
        CACHE.mkdir(parents=True, exist_ok=True)
        tmp = out.with_suffix(f".{os.getpid()}.tmp")
        r = subprocess.run([cc, "-std=c99", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC", "-O1", "-g", str(SOURCES[0]), "-o", str(tmp)],
                           capture_output=True, text=True)
        if r.returncode != 0:
            raise FirmwareBuildError(f"the rover firmware did not compile for the PC:\n{r.stderr[-3000:]}")
        tmp.replace(out)
    return out


_lib = None


def _load():
    global _lib
    if _lib is None:
        lib = ctypes.CDLL(str(build()))
        u8p, sz, u32 = ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t, ctypes.c_uint32
        lib.fw_new.restype = ctypes.c_void_p
        lib.fw_new.argtypes = [u8p, ctypes.c_int, ctypes.c_int, ctypes.c_int, u32, u32, ctypes.c_int, u32]
        lib.fw_free.argtypes = [ctypes.c_void_p]
        for name in ("fw_from_hub", "fw_from_bus"):
            getattr(lib, name).restype = sz
            getattr(lib, name).argtypes = [ctypes.c_void_p, u8p, sz, u32, u8p, sz]
        lib.fw_tick.restype = sz
        lib.fw_tick.argtypes = [ctypes.c_void_p, u32, u8p, sz]
        lib.fw_count.restype = u32
        lib.fw_count.argtypes = [ctypes.c_void_p, ctypes.c_int]
        _lib = lib
    return _lib


class RoverFirmware:
    """One board. Times are the board's millisecond counter, which wraps at 2**32 like the real one."""

    def __init__(self, wheel_ids=(11, 12), speed_addr: int = 46, speed_size: int = 2, watchdog_ms: int = 400, repeat_ms: int = 1000,
                 strip_echo: bool = False, now_ms: int = 0):
        self.lib = _load()
        ids = (ctypes.c_uint8 * len(wheel_ids))(*wheel_ids)
        self.h = self.lib.fw_new(ids, len(wheel_ids), speed_addr, speed_size, watchdog_ms, repeat_ms, int(strip_echo), now_ms & 0xFFFFFFFF)
        if not self.h:
            raise ValueError("the firmware refused that configuration (1 to 4 wheels, a speed register of 1 to 4 bytes, a watchdog over 0 ms)")
        self._out = (ctypes.c_uint8 * 4096)()

    def __del__(self):
        if getattr(self, "h", None):
            self.lib.fw_free(self.h)
            self.h = None

    def _pass(self, fn, data: bytes, now_ms: int) -> bytes:
        out = bytearray()
        for k in range(0, len(data), 4096):                       # (as the board reads its link: a buffer at a time)
            chunk = data[k:k + 4096]
            buf = (ctypes.c_uint8 * len(chunk)).from_buffer_copy(chunk)
            n = fn(self.h, buf, len(chunk), now_ms & 0xFFFFFFFF, self._out, 4096)
            out += bytes(self._out[:n])
        return bytes(out)

    def from_hub(self, data: bytes, now_ms: int) -> bytes:
        return self._pass(self.lib.fw_from_hub, data, now_ms)

    def from_bus(self, data: bytes, now_ms: int) -> bytes:
        return self._pass(self.lib.fw_from_bus, data, now_ms)

    def tick(self, now_ms: int) -> bytes:
        n = self.lib.fw_tick(self.h, now_ms & 0xFFFFFFFF, self._out, 4096)
        return bytes(self._out[:n])

    def count(self, what: str) -> int:
        return int(self.lib.fw_count(self.h, ("trips", "stops_sent", "hub_bytes", "bus_bytes", "echo_dropped", "stopped").index(what)))
