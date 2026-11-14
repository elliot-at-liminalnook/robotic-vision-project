"""The camera firmware in the loop: firmware/p4-uvc-camera built for this PC, with stand-in hardware.

Aimed at classes of bugs:
  * what reaches the PC is what the sensor took, for every frame and every size (not just "a frame arrives");
  * the stand-in hardware reports any misuse by the firmware, and no run in this file may produce one;
  * no order of PC actions and hardware faults (seeded fuzzing) leaves the firmware stuck, leaking or misusing a device;
  * frame rate and delay are what the sensor, the encoder and the USB link allow;
  * every build option changes the picture the way its description says;
  * the same run gives the same result every time.
Skipped where there is no C compiler.
"""
import ctypes
import random

import numpy as np
import pytest

from so101sim import camera_fw as cf

pytestmark = pytest.mark.skipif(cf.compiler() is None, reason="no C compiler to build the firmware for this PC")

PERIOD = 1 / 28            # the IMX708 modes run at 28 fps
STEP = 0.002


class Scene:
    """A picture source that changes every frame: a gradient with a square that moves with the frame count."""

    def __init__(self, noise: bool = False):
        self.n = 0
        self.noise = noise
        self.rng = np.random.default_rng(0)

    def __call__(self, w, h):
        self.n += 1
        if self.noise:
            return self.rng.integers(0, 256, (h, w, 3), dtype=np.uint8)
        y, x = np.mgrid[0:h, 0:w]
        img = np.stack([x * 200 // w + 20, y * 200 // h + 20, np.full((h, w), 60)], -1).astype(np.uint8)
        x0 = (self.n * 37) % (w - h // 4)
        img[h // 3:h // 3 + h // 4, x0:x0 + h // 4] = (250, 250, 250)
        return img


class Rig:
    def __init__(self, source=None, **kw):
        self.cam = cf.UvcCamera(source or Scene(), **kw)
        self.t = 0.0

    def run(self, seconds, read="latest"):
        """Advance time; returns the frames the PC read (every 2 ms, newest only unless read='all')."""
        frames = []
        for _ in range(int(round(seconds / STEP))):
            self.t += STEP
            self.cam.advance_to(self.t)
            if read == "all":
                while (f := self.cam.read()) is not None:
                    frames.append(f)
            elif read == "latest" and (f := self.cam.read_latest()) is not None:
                frames.append(f)
        return frames

    def get(self, name):
        return self.cam.fw.get(name)

    def assert_clean(self):
        assert not self.cam.fw.violations(), self.cam.fw.violations()
        assert "ended" not in self.cam.fw.tasks(), self.cam.fw.tasks()

    def assert_idle(self):
        """After the stream has been closed for a while: everything is shut down and given back."""
        assert not self.get("cam_running") and not self.get("cap_streaming") and not self.get("video_inited")
        assert self.get("live_maps") == 0, "a video buffer is still mapped"
        assert self.get("live_allocs") == 1, "memory leaked (only the USB transfer buffer should remain)"
        assert not self.get("lock_held"), "the camera lock is still held"


def diff(a, b):
    return float(np.abs(a.astype(int) - b.astype(int)).mean())


# ---------------------------------------------------------------- frames
@pytest.mark.parametrize("size", [(640, 480), (800, 600), (1280, 720), (1920, 1080)], ids=lambda s: f"{s[0]}x{s[1]}")
def test_every_frame_that_arrives_is_the_frame_the_sensor_took(size):
    rig = Rig()
    rig.cam.open(*size)
    frames = rig.run(1.2 if size[0] > 800 else 2.0, read="all")
    assert len(frames) >= 15, rig.cam.report()
    seqs = [f.seq for f in frames]
    assert seqs == sorted(set(seqs)), "frames arrived out of order or twice"
    for f in frames:
        assert f.image.shape == (size[1], size[0], 3)
        assert f.t_exposure < f.t_arrival <= rig.t
        if f.seq in rig.cam.sensor_images:
            assert diff(f.image, rig.cam.sensor_images[f.seq]) < 4, f"frame {f.seq} is not the picture the sensor took"
    assert sum(f.seq in rig.cam.sensor_images for f in frames) >= 5
    rig.assert_clean()


def test_the_firmware_advertises_exactly_the_sensors_modes():
    rig = Rig()
    assert rig.cam.fw.frame_sizes() == [(1920, 1080), (1280, 720), (800, 600), (640, 480)]
    with pytest.raises(ValueError, match="not advertised"):
        rig.cam.open(1024, 768)
    assert rig.cam.fw.serial() == "30EDA0000001", "the USB serial number is the board's MAC"


# ---------------------------------------------------------------- timing
def test_first_frame_rate_and_delay():
    rig = Rig()
    rig.cam.open(640, 480)
    frames = rig.run(3.0, read="all")
    p = {k: rig.get(k) / 1e6 for k in ("init_us", "sensor_mode_us", "readout_us", "jpeg_us_per_mp")}
    first = frames[0].t_arrival
    assert p["init_us"] + p["sensor_mode_us"] < first < p["init_us"] + p["sensor_mode_us"] + 0.15, first
    st = rig.cam.stats()
    assert 27.0 <= st["fps"] <= 28.5, st            # the sensor's 28 fps, not the 30 the USB descriptor says
    lo = p["readout_us"] + p["jpeg_us_per_mp"] * 0.3072
    assert lo * 1000 < st["delay_ms"] < lo * 1000 + 20, st
    assert st["sensor_dropped"] == 0 and st["pc_dropped"] == 0
    rig.assert_clean()


@pytest.mark.parametrize("bytes_per_s", [1_000_000, 3_000_000])
def test_a_slow_link_lowers_the_frame_rate_and_corrupts_nothing(bytes_per_s):
    rig = Rig(Scene(noise=True), usb_bytes_per_s=bytes_per_s)      # noise: large JPEGs
    rig.cam.open(640, 480)
    frames = rig.run(4.0, read="all")
    st = rig.cam.stats()
    assert st["jpeg_kb"] > 100
    assert st["fps"] == pytest.approx(bytes_per_s / (st["jpeg_kb"] * 1000), rel=0.25), st
    assert st["sensor_dropped"] > 0, "the sensor is faster than the link: frames must be dropped in the firmware"
    assert all(f.image.shape == (480, 640, 3) for f in frames) and len(frames) >= 2
    rig.assert_clean()


def test_a_slow_reader_sees_old_frames_unless_it_drains_the_queue():
    ages = {}
    for how in ("read", "read_latest"):
        rig = Rig()
        rig.cam.open(640, 480)
        rig.run(1.0, read=None)
        got = []
        for _ in range(20):                          # a 10 Hz loop reading a 28 fps camera
            rig.run(0.1, read=None)
            f = getattr(rig.cam, how)()
            got.append(rig.t - f.t_exposure)
        ages[how] = float(np.mean(got[5:]))
        assert rig.cam.host_dropped > 0 or how == "read_latest"
    assert ages["read"] > 3 * PERIOD, "the PC's 4-frame queue should make read() several frames late"
    assert ages["read_latest"] < 2.5 * PERIOD
    assert ages["read"] > 2 * ages["read_latest"]


# ---------------------------------------------------------------- lifecycle
def test_closing_the_stream_shuts_the_camera_down_and_opening_brings_it_back():
    rig = Rig()
    for size in [(640, 480), (800, 600), (640, 480)]:
        rig.cam.open(*size)
        assert len(rig.run(1.5)) > 10
        assert rig.get("cam_running") and rig.get("cap_streaming")
        rig.cam.close()
        rig.run(1.3)
        assert rig.get("cam_running"), "the camera should keep running for 1.5 s after the last frame request"
        assert not rig.run(0.6), "frames arrived after the stream was closed"
        rig.assert_idle()
    assert any("host stopped reading frames" in line for line in rig.cam.fw.log())
    rig.assert_clean()


def test_reading_again_without_a_new_size_restarts_the_last_mode():
    rig = Rig()
    rig.cam.open(800, 600)
    rig.run(1.0)
    rig.cam.close()
    rig.run(2.5)
    rig.assert_idle()
    rig.cam.resume()
    frames = rig.run(2.5)
    assert len(frames) > 10 and frames[-1].image.shape == (600, 800, 3)
    rig.assert_clean()


def test_usb_suspend_stops_the_camera_at_once():
    rig = Rig()
    rig.cam.open(640, 480)
    rig.run(1.0)
    rig.cam.suspend()
    rig.run(0.2)
    rig.assert_idle()
    rig.cam.open(640, 480)
    assert len(rig.run(1.5)) > 10
    rig.assert_clean()


def test_changing_size_while_streaming():
    rig = Rig()
    rig.cam.open(640, 480)
    rig.run(1.0)
    rig.cam.open(800, 600)                           # no close in between
    frames = rig.run(1.5)
    assert frames and frames[-1].image.shape == (600, 800, 3)
    rig.assert_clean()


# ---------------------------------------------------------------- faults
def test_no_camera_on_the_ribbon_cable_is_reported_and_leaks_nothing():
    rig = Rig(sensor_present=0)
    rig.cam.open(640, 480)
    assert not rig.run(1.5)
    assert rig.get("commit_result") != 0, "the PC was told the camera started"
    log = rig.cam.fw.log()
    assert any("ribbon cable" in line for line in log)
    assert not any("restarting camera" in line for line in log), "a failed start was retried though nobody reads frames"
    rig.assert_idle()
    rig.cam.fw.set("sensor_present", 1)
    rig.cam.open(640, 480)
    assert len(rig.run(1.5)) > 10
    rig.assert_clean()


@pytest.mark.parametrize("stall_s", [1.0, 8.0], ids=["short stall", "longer than 30 bad frames"])
def test_a_stalled_sensor_drops_frames_and_recovers(stall_s):
    rig = Rig()
    rig.cam.open(640, 480)
    rig.run(1.0)
    rig.cam.fw.set("sensor_stalled", 1)
    rig.run(0.3)
    assert not rig.run(stall_s - 0.3), "frames arrived from a stalled sensor"
    assert not rig.get("lock_held") or rig.get("cam_running"), "stuck holding the lock"
    rig.cam.fw.set("sensor_stalled", 0)
    assert len(rig.run(3.0)) > 20, rig.cam.report()
    log = "\n".join(rig.cam.fw.log())
    assert "no frame from the sensor" in log
    assert ("too many bad frames" in log) == (stall_s > 7)
    rig.assert_clean()


def test_failed_encodes_drop_those_frames_and_never_reach_usb():
    rig = Rig()
    rig.cam.open(640, 480)
    rig.run(1.0)
    before = rig.cam.frames_arrived
    rig.cam.fw.set("jpeg_fail", 6)
    frames = rig.run(2.0, read="all")
    assert rig.get("jpeg_fail") == 0, "the faults were not consumed"
    assert rig.get("zero_length_frames") == 0
    assert 20 < rig.cam.frames_arrived - before < 2.0 * 28 - 4, "the failed frames should be missing, the rest not"
    assert all(f.jpeg_bytes > 1000 for f in frames)
    rig.assert_clean()


def test_an_encoder_queue_error_costs_a_pipeline_restart():
    """The firmware cannot tell what state the encoder's buffers are in, so it fails every frame until its
    30-bad-frame limit restarts the pipeline. One such error is an outage of about 1.5 to 2.5 s."""
    rig = Rig()
    rig.cam.open(640, 480)
    rig.run(1.5)
    rig.cam.fw.set("jpeg_queue_error", 1)
    t0, frames = rig.t, rig.run(4.0, read="all")
    assert frames, f"the stream did not come back:\n{rig.cam.report()}"
    outage = frames[0].t_arrival - t0
    assert 1.0 < outage < 2.6, f"outage {outage:.2f} s"
    assert frames[-1].t_arrival > t0 + 3.5 and len(frames) > 30, "the stream came back but did not keep going"
    assert any("too many bad frames" in line for line in rig.cam.fw.log())
    assert rig.get("zero_length_frames") == 0
    rig.assert_clean()


def test_misaligned_capture_buffers_go_through_the_aligned_copy():
    rig = Rig(misalign=1)
    rig.cam.open(640, 480)
    frames = rig.run(1.5, read="all")
    assert rig.get("staging"), "the firmware did not notice the misaligned buffers"
    assert len(frames) > 10 and all(diff(f.image, rig.cam.sensor_images[f.seq]) < 4
                                    for f in frames if f.seq in rig.cam.sensor_images)
    rig.assert_clean()                               # the encoder reports a misaligned input as misuse
    rig.cam.close()
    rig.run(2.0)
    rig.assert_idle()


@pytest.mark.parametrize("seed", range(6))
def test_no_order_of_pc_actions_and_faults_breaks_the_firmware(seed):
    rng = random.Random(seed)
    rig = Rig()
    sizes = [(640, 480), (640, 480), (800, 600), (1280, 720)]
    for _ in range(25):
        op = rng.choice(["open", "open", "close", "resume", "suspend", "fault", "fault", "wait"])
        try:
            if op == "open":
                rig.cam.open(*rng.choice(sizes))
            elif op == "close":
                rig.cam.close()
            elif op == "resume":
                rig.cam.resume()
            elif op == "suspend":
                rig.cam.suspend()
            elif op == "fault":
                name = rng.choice(["sensor_stalled", "sensor_present", "misalign", "jpeg_fail", "jpeg_queue_error", "init_fail"])
                rig.cam.fw.set(name, rng.choice([0, 1]) if name in ("sensor_stalled", "sensor_present", "misalign") else rng.randrange(4))
        except RuntimeError:
            pass                                     # the USB task was still busy with the previous request
        rig.run(rng.choice([0.02, 0.1, 0.3, 0.7, 1.6]), read="latest")
        assert not rig.cam.fw.violations(), (op, rig.cam.fw.violations(), rig.cam.fw.log()[-8:])
    for name in ("sensor_stalled", "misalign", "jpeg_fail", "jpeg_queue_error", "init_fail"):
        rig.cam.fw.set(name, 0)
    rig.cam.fw.set("sensor_present", 1)
    rig.run(1.0)                                     # let a pending request finish
    rig.cam.open(640, 480)
    assert len(rig.run(4.0)) > 20, f"the firmware did not recover:\n{rig.cam.report()}"
    rig.cam.close()
    rig.run(2.5)
    rig.assert_idle()
    rig.assert_clean()


def test_the_same_run_gives_the_same_result():
    def run():
        rig = Rig()
        rig.cam.open(640, 480)
        rig.run(1.0)
        rig.cam.fw.set("jpeg_fail", 3)
        rig.run(0.5)
        rig.cam.close()
        rig.run(2.0)
        rig.cam.open(800, 600)
        rig.run(1.0)
        return list(rig.cam.arrivals), rig.cam.fw.log()
    assert run() == run()


# ---------------------------------------------------------------- build options
def one_frame(options, scene=None, **kw):
    rig = Rig(scene or Scene(), options=options, **kw)
    rig.cam.open(640, 480)
    frames = rig.run(1.0, read="all")
    rig.assert_clean()
    return rig, frames[-1]


@pytest.mark.parametrize("options,flip", [({"HFLIP": True}, (slice(None), slice(None, None, -1))),
                                          ({"VFLIP": True}, (slice(None, None, -1), slice(None))),
                                          ({"HFLIP": True, "VFLIP": True}, (slice(None, None, -1), slice(None, None, -1)))],
                         ids=["hflip", "vflip", "both"])
def test_flips_mirror_the_picture_and_keep_its_colours(options, flip):
    class Still(Scene):
        def __call__(self, w, h):
            self.n = 0
            return super().__call__(w, h)
    rig, f = one_frame(options, Still())
    plain = Still()(640, 480)
    assert diff(f.image, plain[flip]) < 4, "not mirrored as configured (or red and blue are swapped)"
    assert diff(f.image, plain) > 20


def test_fixed_exposure_reaches_the_sensor_and_out_of_range_is_clamped():
    _, auto = one_frame({})
    rig, half = one_frame({"FIXED_EXPOSURE": True, "EXPOSURE_US": 5000, "GAIN_INDEX": 0})
    assert half.image.mean() == pytest.approx(0.5 * auto.image.mean(), rel=0.1), "exposure not applied (or auto exposure still on)"
    rig, _ = one_frame({"FIXED_EXPOSURE": True, "EXPOSURE_US": 200000})
    log = "\n".join(rig.cam.fw.log())
    assert "outside this mode's range" in log and f"exposure_us = {1000000 // 28}" in log


def test_jpeg_quality_changes_the_frame_size():
    _, low = one_frame({"JPEG_QUALITY": 30}, Scene(noise=True))
    _, high = one_frame({"JPEG_QUALITY": 90}, Scene(noise=True))
    assert low.jpeg_bytes < 0.6 * high.jpeg_bytes


# ---------------------------------------------------------------- the stand-ins catch misuse
class _Fmt(ctypes.Structure):
    _fields_ = [("type", ctypes.c_uint32), ("width", ctypes.c_uint32), ("height", ctypes.c_uint32),
                ("pixelformat", ctypes.c_uint32), ("sizeimage", ctypes.c_uint32)]


class _Req(ctypes.Structure):
    _fields_ = [("count", ctypes.c_uint32), ("type", ctypes.c_uint32), ("memory", ctypes.c_uint32)]


class _Buf(ctypes.Structure):
    _fields_ = [("index", ctypes.c_uint32), ("type", ctypes.c_uint32), ("bytesused", ctypes.c_uint32),
                ("flags", ctypes.c_uint32), ("memory", ctypes.c_uint32), ("m", ctypes.c_ulong), ("length", ctypes.c_uint32)]


S_FMT, REQBUFS, QBUF, S_SENSOR_FMT = 0x5601, 0x5603, 0x5605, 0x560B


@pytest.mark.parametrize("misuse,expect", [("queue twice", "queued twice"),
                                           ("mode under buffers", "sensor mode changed while capture buffers are allocated"),
                                           ("close while mapped", "still mapped"),
                                           ("close unknown", "not an open video device")])
def test_the_stand_in_devices_report_misuse(misuse, expect):
    """Drive the stand-in capture device directly, the wrong way: each mistake must be reported."""
    lib = cf.Firmware().lib
    lib.imx708_format_by_size.restype = ctypes.c_void_p
    lib.host_mmap.restype = ctypes.c_void_p
    lib.host_mmap.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_long]
    io = lambda req, arg: lib.host_ioctl(fd, ctypes.c_ulong(req), arg)   # noqa: E731
    assert lib.esp_video_init(None) == 0
    fd = lib.host_open(b"/dev/video0", 2)
    mode = ctypes.c_void_p(lib.imx708_format_by_size(640, 480))
    assert fd > 0 and io(S_SENSOR_FMT, mode) == 0
    assert io(S_FMT, ctypes.byref(_Fmt(1, 640, 480, 0x50424752, 0))) == 0
    assert io(REQBUFS, ctypes.byref(_Req(2, 1, 1))) == 0
    assert lib.fw_violation_count() == 0
    if misuse == "queue twice":
        assert io(QBUF, ctypes.byref(_Buf(index=0, type=1, memory=1))) == 0
        assert io(QBUF, ctypes.byref(_Buf(index=0, type=1, memory=1))) != 0
    elif misuse == "mode under buffers":
        assert io(S_SENSOR_FMT, mode) != 0
    elif misuse == "close while mapped":
        assert lib.host_mmap(None, 640 * 480 * 2, 3, 1, fd, 0)
        lib.host_close(fd)
    else:
        lib.host_close(55)
    assert lib.fw_violation_count() == 1 and expect in lib.fw_violation_line(0).decode()


# ---------------------------------------------------------------- with the simulated arm
@pytest.fixture(scope="module")
def sim():
    from so101sim import load_backend
    s = load_backend("mujoco")
    s.reset(np.array([0.0, -0.6, 0.9, 0.9, -1.57, 0.5]))
    return s


def test_a_smaller_size_is_the_centre_of_the_larger_picture(sim):
    """640x480 over the firmware equals the middle 640x480 pixels of what the 1920x1080 mode sees."""
    intr = sim.cameras["wrist"].intrinsics
    assert (intr["calib_width"], intr["calib_height"]) == (1920, 1080)
    cam = cf.UvcCamera(cf.backend_source(sim, "wrist"))
    cam.open(640, 480)
    frame, t = None, 0.0
    while frame is None and t < 2.0:
        t += STEP
        cam.advance_to(t)
        frame = cam.read_latest()
    full = sim.render_camera("wrist", 1920, 1080, cf.mode_fovy_deg(1080, intr))
    centre = full[300:780, 640:1280]
    assert centre.std() > 10, "the wrist view is blank: nothing to compare"
    assert diff(frame.image, centre) < 6
    assert diff(frame.image, np.asarray(full[::2, ::3][:480, :640])) > 12, "the test cannot tell a crop from a downscale"
    assert cf.mode_fovy_deg(1080, intr) == pytest.approx(sim.cameras["wrist"].fovy_deg)
    assert not cam.fw.violations()


def test_teleop_shows_the_wrist_view_that_came_through_the_firmware():
    from harness import Rig as TeleopRig
    rig = TeleopRig(camera_fw=(640, 480))
    app = rig.app
    rig.run(0.3)
    app.refresh_images()
    assert not app.images["wrist"].any(), "a picture before the camera has started"
    rig.run(1.5)
    app.refresh_images()
    view = app.images["wrist"]
    assert view.shape == (360, 640, 3) and view[:, 80:560].std() > 10
    assert not view[:, :78].any() and not view[:, 562:].any(), "a 4:3 frame should be letterboxed in the 16:9 view"
    st = app.fwcam.stats()
    assert 26 < st["fps"] < 29 and st["delay_ms"] < 60
    app.reset_scene()                                # the simulation clock goes back to 0: the camera keeps running
    rig.run(1.0)
    assert app.fwcam.stats()["fps"] > 26 and not app.fwcam.fw.violations()
    rig.mon.assert_ok()
