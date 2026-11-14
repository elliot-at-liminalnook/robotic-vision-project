"""The camera firmware in the loop: firmware/p4-uvc-camera built for the PC, fed by the simulation.

    scene --render--> sensor stand-in --> THE FIRMWARE (uvc_camera_main.c, unchanged) --> USB stand-in --> PC side
                      (mode crop, 28 fps,   camera task, UVC task, locking,              (frame time on    (V4L2 queue,
                       flips, exposure)      idle stop, bad-frame recovery                the 8 MB/s link)   JPEG decode)

The C side (firmware/p4-uvc-camera/host/) is the firmware plus stand-ins for
FreeRTOS, esp_video's capture and JPEG devices and usb_device_uvc, all on
virtual time. This module compiles it, loads one copy per camera, and answers
its requests: a picture for each sensor frame, a JPEG for each encode. Frames
that come out go into a model of the PC's V4L2 buffer queue.

What it is for: the firmware's logic (bring-up and teardown, locking, timing,
recovery) and the delay and frame rate the controller will see. What it is
not: the image quality of the real sensor, ISP and JPEG hardware. Timing
numbers of the stand-ins are placeholders (firmware/p4-uvc-camera/host/README.md).
"""
from __future__ import annotations

import ctypes
import hashlib
import io
import math
import os
import shutil
import subprocess
import sys
import tempfile
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image

from .config import ROOT, SIM

FW_DIR = ROOT / "firmware" / "p4-uvc-camera"
HOST_DIR = FW_DIR / "host"
SOURCES = ("host_api.c", "rtos.c", "video_fake.c", "uvc_fake.c")
CACHE = SIM / ".cache" / "p4cam"
EV_NONE, EV_NEED_FRAME, EV_NEED_JPEG, EV_FRAME_OUT = 0, 1, 2, 3


class FirmwareBuildError(RuntimeError):
    pass


def compiler() -> str | None:
    return os.environ.get("CC") or shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")


def build(options: dict | None = None) -> Path:
    """Compile the firmware for this PC and return the shared library (cached by source and options).

    options: the firmware's image options without the CONFIG_P4CAM_ prefix, as in sdkconfig:
    HFLIP, VFLIP, FIXED_EXPOSURE, FIXED_FOCUS (True/False), EXPOSURE_US, GAIN_INDEX,
    FOCUS_POSITION, JPEG_QUALITY (numbers).
    """
    cc = compiler()
    if cc is None:
        raise FirmwareBuildError("no C compiler found (set CC, or install cc/gcc/clang)")
    defines = []
    for key, value in sorted((options or {}).items()):
        if value is False or value is None:
            continue
        defines.append(f"-DCONFIG_P4CAM_{key}={1 if value is True else int(value)}")
    files = [HOST_DIR / s for s in SOURCES]
    deps = files + sorted((HOST_DIR / "include").rglob("*.h")) + [FW_DIR / "main" / "uvc_camera_main.c"]
    h = hashlib.sha256(" ".join(defines).encode())
    for f in deps:
        h.update(f.read_bytes())
    out = CACHE / f"p4cam_{h.hexdigest()[:16]}{'.dylib' if sys.platform == 'darwin' else '.so'}"
    if not out.exists():
        CACHE.mkdir(parents=True, exist_ok=True)
        tmp = out.with_suffix(f".{os.getpid()}.tmp")
        cmd = [cc, "-shared", "-fPIC", "-O1", "-g", "-DNDEBUG", "-Wno-unused-variable", f"-I{HOST_DIR / 'include'}",
               "-include", "host_shim.h", *defines, *map(str, files), "-o", str(tmp), "-lpthread"]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            raise FirmwareBuildError(f"the firmware did not compile for the PC:\n{r.stderr[-3000:]}")
        tmp.replace(out)
    return out


class _Event(ctypes.Structure):
    _fields_ = [("kind", ctypes.c_int32), ("width", ctypes.c_int32), ("height", ctypes.c_int32),
                ("hflip", ctypes.c_int32), ("vflip", ctypes.c_int32), ("wrong_colours", ctypes.c_int32),
                ("exposure_us", ctypes.c_int32), ("gain_index", ctypes.c_int32), ("quality", ctypes.c_int32),
                ("corrupt", ctypes.c_int32), ("len", ctypes.c_uint32), ("ptr", ctypes.c_void_p),
                ("src", ctypes.c_void_p), ("seq", ctypes.c_uint64), ("t_exposure_us", ctypes.c_int64),
                ("t_us", ctypes.c_int64)]


class Firmware:
    """One running copy of the firmware (the library keeps its state in globals, so each copy is loaded separately)."""

    def __init__(self, options: dict | None = None, mac: bytes = bytes.fromhex("30eda0000001")):
        lib = build(options)
        with tempfile.TemporaryDirectory(dir=CACHE) as d:      # a private copy = private globals
            copy = Path(d) / lib.name
            shutil.copy(lib, copy)
            self.lib = ctypes.CDLL(str(copy))
        L = self.lib
        L.fw_run.argtypes = [ctypes.c_int64, ctypes.POINTER(_Event)]
        L.fw_jpeg_done.argtypes = [ctypes.c_uint32]
        L.fw_set.argtypes = [ctypes.c_char_p, ctypes.c_int64]
        L.fw_get.argtypes = [ctypes.c_char_p]
        L.fw_get.restype = ctypes.c_int64
        L.fw_frame_size.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
        for name in ("fw_log_line", "fw_violation_line", "fw_serial"):
            getattr(L, name).restype = ctypes.c_char_p
        L.fw_tasks.argtypes = [ctypes.c_char_p, ctypes.c_int]
        L.fw_boot.argtypes = [ctypes.c_char_p]
        L.fw_boot(mac)

    def get(self, name: str) -> int:
        v = self.lib.fw_get(name.encode())
        if v == -2 ** 63:
            raise KeyError(name)
        return v

    def set(self, name: str, value: int):
        if self.lib.fw_set(name.encode(), int(value)) != 0:
            raise KeyError(name)

    def frame_sizes(self) -> list[tuple[int, int]]:
        w, h, out = ctypes.c_int(), ctypes.c_int(), []
        for i in range(1, self.get("frame_sizes") + 1):
            self.lib.fw_frame_size(i, ctypes.byref(w), ctypes.byref(h))
            out.append((w.value, h.value))
        return out

    def log(self) -> list[str]:
        return [self.lib.fw_log_line(i).decode(errors="replace") for i in range(self.lib.fw_log_count())]

    def violations(self) -> list[str]:
        n = self.lib.fw_violation_count()
        return [self.lib.fw_violation_line(i).decode(errors="replace") for i in range(n)]

    def tasks(self) -> str:
        buf = ctypes.create_string_buffer(400)
        self.lib.fw_tasks(buf, 400)
        return buf.value.decode()

    def serial(self) -> str:
        return self.lib.fw_serial().decode()


# ---- pixels ------------------------------------------------------------------
def rgb_to_rgb565(img: np.ndarray) -> bytes:
    r, g, b = (img[..., i].astype(np.uint16) for i in range(3))
    return (((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)).astype("<u2").tobytes()


def rgb565_to_rgb(data: bytes, width: int, height: int) -> np.ndarray:
    v = np.frombuffer(data, dtype="<u2").reshape(height, width)
    r, g, b = (v >> 11) & 0x1F, (v >> 5) & 0x3F, v & 0x1F
    return np.stack([(r << 3) | (r >> 2), (g << 2) | (g >> 4), (b << 3) | (b >> 2)], axis=-1).astype(np.uint8)


def gain_factor(index: int) -> float:
    """The IMX708 driver's analog gain table: index 0 = 1.12x, 10 = 2x, 22 = 4x, 34 = 8x, 46 = 16x."""
    return 1.12 * 2 ** (index / 12)


def mode_fovy_deg(height: int, intrinsics: dict) -> float:
    """Vertical field of view of a sensor mode `height` pixels tall.

    Every mode is a centre crop at the same pixel pitch as the calibrated mode,
    so the focal length in pixels is the same and a smaller mode sees less.
    """
    return math.degrees(2 * math.atan(0.5 * height / float(intrinsics["fy"])))


def backend_source(backend, camera: str, max_width: int = 640):
    """What sensor mode (width, height) of `camera` sees in the simulation right now.

    Rendered at up to max_width pixels wide and scaled up to the mode's size
    (speed; the firmware and the JPEG still handle full-size frames).
    """
    intr = backend.cameras[camera].intrinsics
    off_h = int(backend.m.vis.global_.offheight)

    def source(width: int, height: int) -> np.ndarray:
        scale = min(1.0, max_width / width, off_h / height)
        rw, rh = max(2, int(width * scale)), max(2, int(height * scale))
        img = backend.render_camera(camera, rw, rh, mode_fovy_deg(height, intr))
        if (rw, rh) != (width, height):
            img = np.asarray(Image.fromarray(img).resize((width, height), Image.BILINEAR))
        return img
    return source


@dataclass
class Frame:
    image: np.ndarray          # decoded RGB
    seq: int                   # sensor frame number
    t_exposure: float          # when the sensor took it [s]
    t_arrival: float           # when the last byte reached the PC [s]
    jpeg_bytes: int


class UvcCamera:
    """One camera: sensor, firmware, USB link and the PC's end of it.

        cam = UvcCamera(source)          # source(width, height) -> RGB uint8 picture the sensor sees now
        cam.open(640, 480)               # the PC selects a size and starts the stream
        cam.advance_to(t)                # run everything up to time t [s]
        frame = cam.read_latest()        # or cam.read(): the oldest queued frame, like cv2.VideoCapture.read

    The PC keeps `host_buffers` frames (V4L2's queue). When they are full, new
    frames are dropped and the queued ones get older: a reader slower than
    the camera sees frames several periods late unless it drains the queue.
    """

    def __init__(self, source, *, options: dict | None = None, host_buffers: int = 4, **fw_settings):
        self.source = source
        self.fw = Firmware(options)
        for k, v in fw_settings.items():
            self.fw.set(k, v)
        self.host_buffers = int(host_buffers)
        self.queue: deque = deque()
        self.host_dropped = 0
        self.frames_arrived = 0
        self.arrivals: deque = deque(maxlen=400)      # (t_arrival, t_exposure, bytes)
        self.sensor_images: dict[int, np.ndarray] = {}    # the last few pictures the sensor took, by frame number
        self.mode: tuple[int, int] | None = None
        self._ev = _Event()
        self._t0 = None           # world time at firmware time 0
        self.t = 0.0

    # ---- the PC's actions --------------------------------------------------
    def open(self, width: int, height: int, fps: int = 30):
        """Select a frame size and start streaming (a UVC commit). The firmware takes about half a second."""
        sizes = self.fw.frame_sizes()
        if (width, height) not in sizes:
            raise ValueError(f"{width}x{height} is not advertised; the firmware offers {sizes}")
        if self.fw.lib.fw_host_commit(sizes.index((width, height)) + 1, fps) != 0:
            raise RuntimeError("the camera's USB task is still busy with the previous request")
        self.mode = (width, height)
        self.queue.clear()

    def close(self):
        """Stop reading (the stream is closed). The firmware gets no callback; it notices by itself."""
        self.fw.lib.fw_host_stream(0)
        self.queue.clear()

    def resume(self):
        """Start reading again without selecting a size (not what Linux does, but the firmware allows it)."""
        self.fw.lib.fw_host_stream(1)

    def suspend(self):
        if self.fw.lib.fw_host_suspend() != 0:
            raise RuntimeError("the camera's USB task is still busy with the previous request")
        self.queue.clear()

    # ---- time ----------------------------------------------------------------
    def advance_to(self, t: float):
        """Run sensor, firmware and USB up to world time t [s]."""
        fw_now = self.fw.get("now_us") / 1e6
        if self._t0 is None or t < self._t0 + fw_now - 1e-9:      # first call, or the world's clock was reset
            self._t0 = t - fw_now
        until = int(round((t - self._t0) * 1e6))
        ev, lib = self._ev, self.fw.lib
        while lib.fw_run(until, ctypes.byref(ev)) != EV_NONE:
            if ev.kind == EV_NEED_FRAME:
                self._sensor_frame(ev)
                lib.fw_frame_filled()
            elif ev.kind == EV_NEED_JPEG:
                lib.fw_jpeg_done(self._encode(ev))
            elif ev.kind == EV_FRAME_OUT:
                self._arrive(ev)
        self.t = t

    def _sensor_frame(self, ev):
        img = np.asarray(self.source(ev.width, ev.height))
        if img.shape != (ev.height, ev.width, 3):
            raise ValueError(f"the picture source returned {img.shape}, the sensor mode is {ev.width}x{ev.height}")
        if ev.hflip:
            img = img[:, ::-1]
        if ev.vflip:
            img = img[::-1]
        if ev.wrong_colours:                     # ISP demosaics with the wrong Bayer phase: red and blue swap
            img = img[..., ::-1]
        if ev.exposure_us:                       # crude: brightness in proportion to exposure x gain
            k = ev.exposure_us / 10000 * gain_factor(ev.gain_index) / gain_factor(0)
            img = np.clip(img.astype(np.float32) * k, 0, 255).astype(np.uint8)
        img = np.ascontiguousarray(img, dtype=np.uint8)
        self.sensor_images[ev.seq] = img
        for old in [s for s in self.sensor_images if s < ev.seq - 8]:
            del self.sensor_images[old]
        data = rgb_to_rgb565(img)
        ctypes.memmove(ev.ptr, data, min(len(data), ev.len))

    def _encode(self, ev) -> int:
        img = rgb565_to_rgb(ctypes.string_at(ev.src, ev.width * ev.height * 2), ev.width, ev.height)
        if ev.corrupt:                           # stand-in for what the hardware does with a misaligned buffer
            img = np.roll(img, ev.width // 3, axis=1)[::-1]
        out = io.BytesIO()
        Image.fromarray(img).save(out, "JPEG", quality=int(ev.quality))
        data = out.getvalue()
        if len(data) > ev.len:
            return 0                             # does not fit the encoder's buffer: the encode fails
        ctypes.memmove(ev.ptr, data, len(data))
        return len(data)

    def _arrive(self, ev):
        t_arr, t_exp = self._t0 + ev.t_us / 1e6, self._t0 + ev.t_exposure_us / 1e6
        self.frames_arrived += 1
        self.arrivals.append((t_arr, t_exp, int(ev.len)))
        if len(self.queue) >= self.host_buffers:
            self.host_dropped += 1               # no free buffer on the PC: this frame is lost
            return
        self.queue.append((ctypes.string_at(ev.ptr, ev.len), int(ev.seq), t_exp, t_arr))

    # ---- reading -------------------------------------------------------------
    def read(self) -> Frame | None:
        """The oldest queued frame, or None if none has arrived."""
        if not self.queue:
            return None
        jpeg, seq, t_exp, t_arr = self.queue.popleft()
        return Frame(np.asarray(Image.open(io.BytesIO(jpeg)).convert("RGB")), seq, t_exp, t_arr, len(jpeg))

    def read_latest(self) -> Frame | None:
        """The newest queued frame, discarding older ones."""
        while len(self.queue) > 1:
            self.queue.popleft()
        return self.read()

    # ---- what happened ---------------------------------------------------------
    def stats(self, window_s: float = 2.0) -> dict:
        recent = [a for a in self.arrivals if a[0] > self.t - window_s]
        fps = (len(recent) - 1) / (recent[-1][0] - recent[0][0]) if len(recent) > 1 else 0.0
        return {
            "fps": fps,
            "delay_ms": 1000 * float(np.mean([a - e for a, e, _ in recent])) if recent else float("nan"),
            "jpeg_kb": float(np.mean([b for _, _, b in recent])) / 1000 if recent else float("nan"),
            "frames_arrived": self.frames_arrived,
            "pc_dropped": self.host_dropped,
            "sensor_frames": self.fw.get("sensor_frames"),
            "sensor_dropped": self.fw.get("sensor_dropped"),
            "running": bool(self.fw.get("cam_running")),
        }

    def report(self, log_lines: int = 12) -> str:
        s = self.stats()
        mode = f"{self.mode[0]}x{self.mode[1]}" if self.mode else "not opened"
        lines = [
            f"camera firmware ({mode}, USB serial {self.fw.serial()}): {s['frames_arrived']} frames reached the PC, "
            f"last 2 s: {s['fps']:.1f} fps, {s['delay_ms']:.0f} ms from exposure to arrival, {s['jpeg_kb']:.0f} kB each",
            f"  sensor frames {s['sensor_frames']} (lost in the firmware {s['sensor_dropped']}, on the PC {s['pc_dropped']}); "
            f"pipeline {'running' if s['running'] else 'stopped'}",
            f"  tasks: {self.fw.tasks()}",
        ]
        lines += [f"  [misuse] {v}" for v in self.fw.violations()]
        if log_lines:
            lines += ["  firmware log (last lines):"] + [f"    {line}" for line in self.fw.log()[-log_lines:]]
        return "\n".join(lines)
