#!/usr/bin/env python3
"""Drive the simulated SO-101 follower with keyboard + mouse, watching all three cameras.

    python sim/scripts/teleop.py                    # MuJoCo backend (any OS, incl. Intel Mac)
    python sim/scripts/teleop.py --backend newton   # Linux; MuJoCo Warp on a CUDA GPU
    python sim/scripts/teleop.py --selftest         # headless scripted check (no window)

Window: wrist | scene_front on top, scene_side | status panel below.

Cartesian mode (default): move the gripper tip; IK keeps the fingers as close
to straight down as the arm allows.
  W / S  or  Up / Down      tip forward / back   (+x / -x, away from / toward the base)
  A / D  or  Left / Right   tip left / right     (+y / -y)
  R / F  or  mouse wheel    tip up / down
  Q / E                     wrist roll
  Space  or  right-click    open / close gripper
  left-click on the table   (in scene_front or scene_side) move the tip above that point
  hold Shift                fine (slow) motion
Joint mode (toggle J): 1-6 select a joint; Left/Right or the wheel move it.
  G          FAST (torque governor, default) / GENTLE (fixed profile)
  [ / ]      tilt cone for load-aware IK, 0..25 deg (default 0 = fingers straight down)
  V          servo feed-forward on/off (default on; off shows the droop it removes)
  H home   Backspace reset scene   P pause physics   F12 screenshot   Esc quit
  X          STOP: drop whatever move is under way and hold the arm where it is
             (movement keys still held down do nothing until they are let go)
  T          torque off (the arm goes limp and sags) / back on, holding where it then is

Motion (sim/docs/FAST_TELEOP_FINDINGS.md):
  * FAST: so101sim.governor.TorqueGovernor predicts every joint's torque each
    servo tick and commands the fastest smooth motion the HX-30HMs (12 V) can
    follow, staying clear of their overload protection and heat rating.
    Keyboard tip speed 0.25 m/s. GENTLE: the fixed 2.75 rad/s, 8 rad/s^2
    profile and 0.10 m/s.
  * Click-to-move and H travel along a straight line (so101sim.motion), paced
    by how far the measured arm is behind; jumping straight to the far joint
    angles let the tip dive into the table on the way.
  * Load-aware IK: with a tilt cone > 0, the fingers may tilt up to that much
    to lower the servos' holding torque. It ramps to 0 below 10 cm above the
    table, so grasps are always straight down.
  * The panel shows each joint's predicted torque (% of stall, tick = the
    overload-protection threshold) and heat (RMS torque vs continuous rating).
  * Servo compensation (so101sim/command.py): every arm command gets gravity +
    back-EMF feed-forward, because a real servo position loop is soft (stall
    torque only at ~9 deg of error) and would otherwise droop and lag. Closing
    the gripper stops a few degrees past contact (grip-by-offset) instead of
    stalling the servo into its overload protection.

Always enforced: joint limits, the camera-safe wrist roll range (see
hardware/wrist_mount/place_camera.py) and a minimum tip height above the
table (in joint mode: a jog that would take the tip below it is refused).
Unreachable targets are refused. Stall guard: if an arm joint stays more than
12 deg behind its command for 0.4 s (blocked by the table, an object or the
arm itself), the goal is pulled back to where the arm is.

Stop path: X freezes (the servos keep holding, nothing drops); T switches the
servos' torque off, and it is switched off whenever the program ends, however
it ends (Esc, an error, Ctrl-C, a kill). Neither helps if this computer or the
bus is dead: that takes a switch on the 12 V lead.
"""
from __future__ import annotations

import argparse
import math
import os
import signal
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from so101sim import JOINTS, load_backend  # noqa: E402
from so101sim.config import load_servos, look_at_axes  # noqa: E402,F401
from hub.arm.controller import (BLOCK_ERR, BLOCK_TIME, FRAME_HZ, GRIP_CLOSED, GRIP_OPEN, HOME_TIP, IK_TOL,  # noqa: E402,F401
                                JOINT_SPEED, LIN_SPEED, MIN_TIP_Z, ROLL_SPEED, TILT_MAX, TILT_RAMP, TILT_STEP, WHEEL_STEP,
                                ArmController)
from hub.arm.governor import TorqueGovernor  # noqa: E402
from hub.arm.servo import GoalInterpolator, ProfileLimiter  # noqa: E402,F401

VIEW_W, VIEW_H = 640, 360
PANEL = 30                      # title strip height above each view
LAYOUT = {"wrist": (0, 0), "scene_front": (1, 0), "scene_side": (0, 1)}   # (col, row)


class Teleop(ArmController):
    """The arm's controller (hub/arm/controller.py) with a window, driving a simulation backend."""

    def __init__(self, backend: str, solver: str | None, render_every: int, bus: dict | None = None,
                 camera_fw: tuple | None = None, cables: bool = False, scene: str = "workcell"):
        kw = {"solver": solver} if backend == "newton" else {}
        if scene != "workcell":          # another scene from sim/scene/ (MuJoCo backend)
            kw["scene"] = scene
        if cables:                       # the arm's wires as part of the model (so101sim/cables.py)
            kw["cables"] = True
        sim = load_backend(backend, **kw)
        if bus is not None:              # servo bus in the loop: bytes to emulated servos instead of joint targets
            from so101sim.bus_backend import BusBackend
            sim = BusBackend(sim, **bus)
        self.monitor = None              # physics monitor (MuJoCo backends): overlaps, speeds, objects through the table
        if hasattr(sim, "step_hooks"):
            from so101sim.monitor import PhysicsMonitor
            self.monitor = PhysicsMonitor(sim)
        self.fwcam = None                # wrist camera through the real firmware (camera_fw = (width, height))
        if camera_fw is not None:
            from so101sim.camera_fw import UvcCamera, backend_source
            self.fwcam = UvcCamera(backend_source(sim, "wrist"))
            sim.step_hooks.append(self.fwcam.advance_to)     # same clock as the physics
            self.fwcam.open(*camera_fw)
            self._fw_view = None
        super().__init__(sim)
        self.render_every = max(1, render_every)
        self.reset_scene()

    # ---- state -------------------------------------------------------------
    def reset_scene(self):
        q, _ = self.ik.solve(HOME_TIP, [0, 0, 0, 0, self._roll_default(), GRIP_OPEN])
        self.sim.reset(q)
        self.start(q)
        self.target = np.array(HOME_TIP, float)
        self.grip = GRIP_OPEN
        self.refresh_images()
        self.frame = 0

    # ---- commands -----------------------------------------------------------
    def click_table(self, cam: str, px: float, py: float):
        """Move the tip above the table point under a click in a fixed camera."""
        spec = self.sim.cameras[cam]
        if spec.parent_body != "world":
            self.say("click the table in a fixed camera view")
            return
        i = spec.intrinsics
        s = spec.render_width / i["calib_width"]
        x, y, z = look_at_axes(spec.pos, spec.look_at, spec.up)
        ray = (x * (px - i["cx"] * s) / (i["fx"] * s) - y * (py - i["cy"] * s) / (i["fy"] * s) - z)
        if ray[2] >= -1e-6:
            self.say("that's not the table")
            return
        t = -spec.pos[2] / ray[2]
        hit = np.asarray(spec.pos) + t * ray
        self.move_to([hit[0], hit[1], self.target[2]], f"moving above ({hit[0]:.3f}, {hit[1]:.3f})")

    # ---- loop ---------------------------------------------------------------
    def handle_held_keys(self, keys, dt: float, pg):
        if self.paused or self.limp:     # the target must not run ahead of a frozen or limp arm
            return
        if self.stop_latch:              # after a stop, movement keys still held down do nothing until let go
            if any(keys[k] for k in (pg.K_w, pg.K_s, pg.K_a, pg.K_d, pg.K_r, pg.K_f, pg.K_q, pg.K_e,
                                     pg.K_UP, pg.K_DOWN, pg.K_LEFT, pg.K_RIGHT)):
                return
            self.stop_latch = False
        fine = 0.2 if (keys[pg.K_LSHIFT] or keys[pg.K_RSHIFT]) else 1.0
        if self.mode == "cartesian":
            v = np.zeros(3)
            v[0] += (keys[pg.K_w] or keys[pg.K_UP]) - (keys[pg.K_s] or keys[pg.K_DOWN])
            v[1] += (keys[pg.K_a] or keys[pg.K_LEFT]) - (keys[pg.K_d] or keys[pg.K_RIGHT])
            v[2] += keys[pg.K_r] - keys[pg.K_f]
            droll = (keys[pg.K_e] - keys[pg.K_q]) * ROLL_SPEED * fine * dt
            if v.any() or droll:
                self.move = None               # the keyboard takes over from a click move
                self.set_cartesian(self.target + v * LIN_SPEED[self.fast] * fine * dt, roll=self.roll + droll)
            else:
                self.advance_move(dt)
        else:
            dj = (keys[pg.K_RIGHT] or keys[pg.K_d]) - (keys[pg.K_LEFT] or keys[pg.K_a])
            if dj:
                self.jog_joint(self.sel_joint, dj * JOINT_SPEED[self.fast] * fine * dt)

    def handle_event(self, e, pg) -> bool:
        """Returns False to quit."""
        if e.type == pg.QUIT:
            return False
        if e.type == pg.KEYDOWN:
            k = e.key
            if k == pg.K_ESCAPE:
                return False
            if k == pg.K_x:
                self.freeze()
            elif k == pg.K_t:
                self.set_torque(self.limp)
            elif self.limp and k not in (pg.K_BACKSPACE, pg.K_p, pg.K_F12):
                self.say("torque is off (T switches it on)")
            elif k == pg.K_SPACE:
                self.toggle_grip()
            elif k == pg.K_j:
                self.set_mode("joint" if self.mode == "cartesian" else "cartesian")
            elif k == pg.K_h:
                self.move_to(HOME_TIP, "home", roll=self._roll_default())
            elif k == pg.K_g:
                self.toggle_speed()
            elif k == pg.K_v:
                self.ff_on = not self.ff_on
                self.ff.reset()
                self.say("servo feed-forward " + ("ON" if self.ff_on else "OFF (watch the droop)"))
            elif k in (pg.K_LEFTBRACKET, pg.K_RIGHTBRACKET):
                step = TILT_STEP if k == pg.K_RIGHTBRACKET else -TILT_STEP
                self.tilt_tol = float(np.clip(self.tilt_tol + step, 0.0, TILT_MAX))
                self.say(f"tilt cone {self.tilt_tol:.0f} deg" + (" (strict: fingers straight down)" if self.tilt_tol == 0 else
                                                               " (load-aware above 10 cm)"))
            elif k == pg.K_BACKSPACE:
                self.reset_scene()
                self.say("scene reset")
            elif k == pg.K_p:
                self.paused = not self.paused
                self.say("paused" if self.paused else "running")
            elif k == pg.K_F12:
                self.save_screenshot(pg)
            elif pg.K_1 <= k <= pg.K_6:
                self.sel_joint = k - pg.K_1
                self.say(f"selected {JOINTS[self.sel_joint]}")
        if (self.paused or self.limp) and e.type in (pg.MOUSEWHEEL, pg.MOUSEBUTTONDOWN):
            self.say("torque is off (T switches it on)" if self.limp else "paused (P to resume)")
            return True
        if e.type == pg.MOUSEWHEEL:
            if self.mode == "cartesian":
                self.move = None
                self.set_cartesian(self.target + [0, 0, e.y * WHEEL_STEP])
            else:
                self.jog_joint(self.sel_joint, e.y * WHEEL_STEP * 3)
        if e.type == pg.MOUSEBUTTONDOWN:
            if e.button == 3:
                self.toggle_grip()
            elif e.button == 1:
                cam, px, py = self.view_at(*e.pos)
                if cam:
                    self.click_table(cam, px, py)
        return True

    def view_at(self, x, y):
        for cam, (c, r) in LAYOUT.items():
            x0, y0 = c * VIEW_W, r * (VIEW_H + PANEL) + PANEL
            if x0 <= x < x0 + VIEW_W and y0 <= y < y0 + VIEW_H:
                spec = self.sim.cameras[cam]
                return cam, (x - x0) * spec.render_width / VIEW_W, (y - y0) * spec.render_height / VIEW_H
        return None, 0, 0

    def refresh_images(self):
        """Camera views. With the firmware in the loop, the wrist view is what reached the PC over USB."""
        self.images = self.sim.render()
        if self.fwcam is None:
            return
        from PIL import Image
        frame = self.fwcam.read_latest()
        if frame is not None:                # letterboxed into the view: a cropping sensor mode shows as a narrower picture
            h, w = frame.image.shape[:2]
            k = min(VIEW_W / w, VIEW_H / h)
            small_img = Image.fromarray(frame.image).resize((max(1, int(w * k)), max(1, int(h * k))), Image.BILINEAR)
            view = Image.new("RGB", (VIEW_W, VIEW_H))
            view.paste(small_img, ((VIEW_W - small_img.width) // 2, (VIEW_H - small_img.height) // 2))
            self._fw_view = np.asarray(view)
        if not self.fwcam.stats()["running"] or self._fw_view is None:
            self._fw_view = np.zeros((VIEW_H, VIEW_W, 3), np.uint8)       # nothing is streaming
        self.images["wrist"] = self._fw_view

    # ---- drawing ------------------------------------------------------------
    def draw(self, screen, font, small, pg):
        screen.fill((24, 24, 28))
        for cam, (c, r) in LAYOUT.items():
            x0, y0 = c * VIEW_W, r * (VIEW_H + PANEL)
            label = cam
            if cam == "wrist" and self.fwcam is not None:
                st = self.fwcam.stats()
                label = (f"wrist via firmware {self.fwcam.mode[0]}x{self.fwcam.mode[1]}: {st['fps']:.0f} fps, "
                         f"{st['delay_ms']:.0f} ms old on arrival" if st["fps"] else "wrist via firmware: no frames")
            screen.blit(font.render(label, True, (220, 220, 220)), (x0 + 8, y0 + 6))
            img = self.images.get(cam)
            if img is not None:
                surf = pg.surfarray.make_surface(np.ascontiguousarray(img.swapaxes(0, 1)))
                if surf.get_size() != (VIEW_W, VIEW_H):
                    surf = pg.transform.smoothscale(surf, (VIEW_W, VIEW_H))
                screen.blit(surf, (x0, y0 + PANEL))
        # status panel
        x0, y0 = VIEW_W, VIEW_H + PANEL
        q = self.sim.q()
        tip = self.tip_pos(q)
        lines = [
            f"backend {self.sim.name}{'-' + self.sim.solver_kind if hasattr(self.sim, 'solver_kind') else ''}"
            f"   mode {self.mode.upper()}   {'TORQUE OFF' if self.limp else 'PAUSED' if self.paused else 'running'}   sim t {self.sim.time:6.1f}s",
            f"target  x {self.target[0]:+.3f}  y {self.target[1]:+.3f}  z {self.target[2]:+.3f} m"
            f"   roll {self.roll:+.2f}   grip {'open' if self.grip > 0.3 else 'closed'}",
            f"tip     x {tip[0]:+.3f}  y {tip[1]:+.3f}  z {tip[2]:+.3f} m"
            f"   fingers tilt {self.ik.tilt_deg(q):4.1f} deg",
            f"motion {'FAST (torque governor)' if self.fast else 'GENTLE (fixed profile)'}   "
            f"tilt cone {self.tilt_tol:.0f} deg (now {self.effective_tilt(self.target[2]):.0f})"
            f"   ff {'on' if self.ff_on else 'OFF'}" + ("   grip HOLD" if self.grip_latch.holding else "")
            + (f"   limited by {JOINTS[self.gen.limiting]}" if isinstance(self.gen, TorqueGovernor)
               and self.gen.limiting is not None and self.gen.scale < 0.99 else ""),
        ]
        for k, ln in enumerate(lines):
            screen.blit(small.render(ln, True, (230, 230, 230)), (x0 + 12, y0 + 8 + 19 * k))
        load = self.load_frac()
        heat = self.gen.heat_frac() if isinstance(self.gen, TorqueGovernor) else None
        thr = self.spec.overload_torque / self.spec.tau_s
        hy = y0 + 92
        screen.blit(small.render("joint                 position", True, (150, 150, 160)), (x0 + 12, hy - 4))
        screen.blit(small.render("torque (tick = overload threshold)", True, (150, 150, 160)), (x0 + 350, hy - 4))
        for j, name in enumerate(JOINTS):          # joint bars
            yy = hy + 14 + j * 22
            lo, hi = self.lo[j], self.hi[j]
            col = (255, 210, 90) if (self.mode == "joint" and j == self.sel_joint) else (200, 200, 200)
            screen.blit(small.render(f"{j + 1} {name}", True, col), (x0 + 12, yy))
            bx, bw = x0 + 140, 150
            pg.draw.rect(screen, (70, 70, 80), (bx, yy + 4, bw, 10))
            f = (q[j] - lo) / (hi - lo) if hi > lo else 0.5
            pg.draw.rect(screen, (90, 170, 255), (bx, yy + 4, int(bw * np.clip(f, 0, 1)), 10))
            screen.blit(small.render(f"{q[j]:+.2f}", True, col), (bx + bw + 6, yy))
            lx, lw = x0 + 350, 160
            lf = float(np.clip(load[j], 0, 1))
            lc = (90, 200, 120) if lf < 0.5 else (240, 200, 80) if lf < thr else (240, 90, 80)
            pg.draw.rect(screen, (70, 70, 80), (lx, yy + 4, lw, 10))
            pg.draw.rect(screen, lc, (lx, yy + 4, int(lw * lf), 10))
            pg.draw.line(screen, (240, 90, 80), (lx + int(lw * thr), yy + 1), (lx + int(lw * thr), yy + 16), 2)
            txt = f"{load[j] * 100:3.0f}%" + (f" heat {heat[j] * 100:3.0f}%" if heat is not None else "")
            screen.blit(small.render(txt, True, col), (lx + lw + 6, yy))
        help_ = ["WASD/arrows move  R/F/wheel up-down  Q/E roll  Space/right-click grip  Shift fine",
                 "click table in a scene view: go there   J joint mode (1-6, L/R)   G fast/gentle",
                 "X STOP  T torque off/on  [ ] tilt  V feed-fwd  H home  Backspace reset  P pause  Esc quit"]
        for k, ln in enumerate(help_):
            screen.blit(small.render(ln, True, (150, 150, 160)), (x0 + 12, y0 + 246 + 18 * k))
        if time.time() - self.message_t < 3.0:
            screen.blit(font.render(self.message, True, (255, 220, 120)), (x0 + 12, y0 + VIEW_H - 32))

    def save_screenshot(self, pg, path=None):
        path = Path(path or Path(__file__).resolve().parents[1] / "out" / f"teleop_{time.strftime('%Y%m%d_%H%M%S')}.png")
        path.parent.mkdir(parents=True, exist_ok=True)
        pg.image.save(pg.display.get_surface(), str(path))
        self.say(f"saved {path.name}")
        return path

    def run(self, selftest: bool = False):
        if selftest:
            os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        import pygame as pg
        pg.init()
        screen = pg.display.set_mode((2 * VIEW_W, 2 * (VIEW_H + PANEL)))
        pg.display.set_caption("SO-101 follower - teleop (sim)")
        font = pg.font.SysFont(None, 26)
        small = pg.font.SysFont(None, 20)
        clock = pg.time.Clock()
        script = selftest_script(self, pg) if selftest else None
        running, last = True, time.perf_counter()
        while running:
            now = time.perf_counter()
            dt = now - last
            last = now
            if script is not None:
                try:
                    next(script)
                except StopIteration:
                    running = False
            for e in pg.event.get():
                running &= self.handle_event(e, pg)
            self.handle_held_keys(pg.key.get_pressed() if script is None else _FakeKeys.current, dt if script is None else 1 / 30, pg)
            self.physics(dt if script is None else 1 / 30)
            if self.frame % self.render_every == 0:
                self.refresh_images()
            self.frame += 1
            self.draw(screen, font, small, pg)
            pg.display.flip()
            if script is None:
                clock.tick(30)
        pg.quit()


class _FakeKeys:
    """Held-key state for the self-test (pygame's key state needs a real keyboard)."""
    current = None

    def __init__(self, pg):
        self._down = set()
        _FakeKeys.current = self

    def hold(self, *keys):
        self._down = set(keys)

    def __getitem__(self, k):
        return int(k in self._down)


def selftest_script(app: Teleop, pg):
    """Scripted session: move with keys, click the table, close the gripper, check results."""
    keys = _FakeKeys(pg)
    results = {}
    tip0 = app.tip_pos().copy()
    keys.hold(pg.K_w)                                  # forward for 1 s
    for _ in range(30):
        yield
    keys.hold()
    for _ in range(30):                                # settle
        yield
    results["forward_dx_m"] = app.tip_pos()[0] - tip0[0]
    # Click the table right under the red cube's centre in scene_front (the
    # click is cast onto the table plane, so aim at the cube's footprint).
    cube = app.sim.body_pos("cube")
    cube = np.array([cube[0], cube[1], 0.0])
    spec = app.sim.cameras["scene_front"]
    x, y, z = look_at_axes(spec.pos, spec.look_at, spec.up)
    i = spec.intrinsics
    s = spec.render_width / i["calib_width"]
    d = cube - np.asarray(spec.pos)
    u = i["cx"] * s + i["fx"] * s * (d @ x) / (-(d @ z))
    v = i["cy"] * s - i["fy"] * s * (d @ y) / (-(d @ z))
    c, r = LAYOUT["scene_front"]
    pos = (int(c * VIEW_W + u * VIEW_W / spec.render_width), int(r * (VIEW_H + PANEL) + PANEL + v * VIEW_H / spec.render_height))
    pg.event.post(pg.event.Event(pg.MOUSEBUTTONDOWN, button=1, pos=pos))
    for _ in range(90):
        yield
    tip = app.tip_pos()
    results["click_xy_error_m"] = float(np.linalg.norm(tip[:2] - cube[:2]))
    for _ in range(10):                                 # wheel down 10 notches = -0.10 m
        pg.event.post(pg.event.Event(pg.MOUSEWHEEL, x=0, y=-1))
        yield
    for _ in range(60):
        yield
    results["wheel_z_m"] = float(app.tip_pos()[2])
    pg.event.post(pg.event.Event(pg.KEYDOWN, key=pg.K_SPACE, mod=0, unicode=" ", scancode=0))
    for _ in range(45):
        yield
    results["grip_after_space"] = float(app.sim.q()[5])
    pg.event.post(pg.event.Event(pg.KEYDOWN, key=pg.K_j, mod=0, unicode="j", scancode=0))
    pg.event.post(pg.event.Event(pg.KEYDOWN, key=pg.K_1, mod=0, unicode="1", scancode=0))
    yield
    pan0 = app.sim.q()[0]
    keys.hold(pg.K_RIGHT)
    for _ in range(15):
        yield
    keys.hold()
    for _ in range(30):
        yield
    results["joint_mode_pan_delta"] = float(app.sim.q()[0] - pan0)
    results["shoulder_load_frac"] = float(app.load_frac()[1])
    # G toggles FAST/GENTLE; ] widens the tilt cone; back in Cartesian mode, H
    # travels home along a straight line, then R raises the tip above 10 cm
    # where the cone applies (load-aware IK).
    key = lambda k, u="": pg.event.post(pg.event.Event(pg.KEYDOWN, key=k, mod=0, unicode=u, scancode=0))  # noqa: E731
    key(pg.K_g, "g")
    yield
    results["g_gentle"] = float(not app.fast)
    key(pg.K_g, "g")
    for _ in range(3):
        key(pg.K_RIGHTBRACKET, "]")
    key(pg.K_j, "j")
    yield
    results["tilt_cone_deg"] = app.tilt_tol
    key(pg.K_h, "h")
    for _ in range(90):
        yield
    results["home_err_m"] = float(np.linalg.norm(app.tip_pos() - np.array(HOME_TIP)))
    tilt_strict = app.ik.tilt_deg(app.sim.q())
    keys.hold(pg.K_s)                                  # back toward the base: fingers-down gets harder
    for _ in range(15):
        yield
    keys.hold()
    for _ in range(45):
        yield
    results["fast_after_toggles"] = float(app.fast)
    # V toggles servo feed-forward: without it the soft servo loop droops under gravity.
    key(pg.K_v, "v")
    for _ in range(45):
        yield
    results["droop_ff_off_mm"] = float(np.linalg.norm(app.tip_pos() - app.tip_pos(app.goal))) * 1000
    key(pg.K_v, "v")
    for _ in range(45):
        yield
    results["droop_ff_on_mm"] = float(np.linalg.norm(app.tip_pos() - app.tip_pos(app.goal))) * 1000
    results["tilt_strict_at_home_deg"] = tilt_strict
    results["effective_cone_deg"] = app.effective_tilt(app.target[2])
    shot = app.save_screenshot(pg, Path(__file__).resolve().parents[1] / "out" / "teleop_selftest.png")
    checks = {
        "W moves the tip forward": results["forward_dx_m"] > 0.05,
        "clicking the table moves the tip above it (<15 mm)": results["click_xy_error_m"] < 0.015,
        "wheel lowers the tip": results["wheel_z_m"] < 0.08,
        "Space closes the gripper": results["grip_after_space"] < 0.2,
        "joint mode moves shoulder_pan": results["joint_mode_pan_delta"] > 0.1,
        "load panel reports shoulder torque": results["shoulder_load_frac"] > 0.03,
        "G toggles to GENTLE and back": results["g_gentle"] == 1.0 and results["fast_after_toggles"] == 1.0,
        "] widens the tilt cone to 15 deg": results["tilt_cone_deg"] == 15.0,
        "H travels home (<10 mm)": results["home_err_m"] < 0.010,
        "cone applies above 10 cm": results["effective_cone_deg"] > 14.9,
        "feed-forward removes droop (V)": results["droop_ff_on_mm"] < 3.0 and results["droop_ff_off_mm"] > 2 * results["droop_ff_on_mm"],
    }
    for k, v in results.items():
        print(f"  {k}: {v:+.4f}")
    for k, ok in checks.items():
        print(("PASS " if ok else "FAIL ") + k)
    print("screenshot:", shot)
    app.selftest_ok = all(checks.values())


def exit_on_sigterm():
    """A kill (SIGTERM) leaves through the same `finally` as everything else, so torque still goes off."""
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", choices=("mujoco", "newton"), default="mujoco")
    ap.add_argument("--solver", choices=("warp", "cpu"), default=None, help="Newton only")
    ap.add_argument("--render-every", type=int, default=None,
                    help="render cameras every N frames (default 1; 10 for Newton on CPU)")
    ap.add_argument("--selftest", action="store_true", help="headless scripted check")
    ap.add_argument("--bus", action="store_true",
                    help="drive emulated HX-30HM servos over a simulated servo bus (MuJoCo backend); "
                         "prints a bus report on exit")
    ap.add_argument("--bus-drop", type=float, default=0.0, metavar="P",
                    help="with --bus: probability that a servo's reply is lost")
    ap.add_argument("--bus-trace", type=int, default=0, metavar="N",
                    help="with --bus: also print the last N frames on the wire on exit")
    ap.add_argument("--camera-fw", nargs="?", const="640x480", default=None, metavar="WxH",
                    help="run the wrist camera through the camera firmware built for this PC "
                         "(MuJoCo backend; needs a C compiler). Sizes: 1920x1080, 1280x720, 800x600, 640x480 "
                         "(default; each is a crop, so smaller sees less). Prints a camera report on exit")
    ap.add_argument("--scene", default="workcell", help="which scene: workcell (default) or bench (sim/scene/*.xml)")
    ap.add_argument("--cables", action="store_true",
                    help="include the arm's wires (servo bus leads, wrist camera USB cable) in the model "
                         "(MuJoCo backend; about ten times slower to simulate). Prints a wire report on exit")
    a = ap.parse_args()
    if (a.bus or a.camera_fw or a.cables or a.scene != "workcell") and a.backend != "mujoco":
        ap.error("--bus, --camera-fw, --cables and --scene need --backend mujoco")
    mode = None
    if a.camera_fw:
        try:
            mode = tuple(int(v) for v in a.camera_fw.lower().split("x"))
            assert len(mode) == 2
        except (ValueError, AssertionError):
            ap.error("--camera-fw takes a size like 640x480")
    try:
        app = Teleop(a.backend, a.solver, a.render_every or 1,
                     bus={"drop_reply_prob": a.bus_drop} if a.bus else None, camera_fw=mode, cables=a.cables,
                     scene=a.scene)
    except ValueError as e:              # a size the firmware does not advertise
        ap.error(str(e))
    if a.render_every is None and getattr(app.sim, "solver_kind", None) == "cpu":
        app.render_every = 10            # ray-traced cameras are slow without a GPU (also when cpu was auto-selected)
    app.selftest_ok = True
    exit_on_sigterm()
    try:
        app.run(selftest=a.selftest)
    finally:
        print("stop path:", app.shutdown())
        if a.bus:
            print(app.sim.report(trace=a.bus_trace))
        if app.fwcam is not None:
            print(app.fwcam.report())
        inner = getattr(app.sim, "inner", app.sim)
        if getattr(inner, "cables", None) is not None:
            print(inner.cables.report(inner.d))
        if app.monitor is not None:
            print(app.monitor.report())
        if getattr(app.sim, "safety", None) is not None:
            print(app.sim.safety.report())
    return 0 if app.selftest_ok else 1


if __name__ == "__main__":
    sys.exit(main())
