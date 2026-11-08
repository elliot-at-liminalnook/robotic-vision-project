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
  H home   Backspace reset scene   P pause physics   F12 screenshot   Esc quit

Always enforced: joint limits, the camera-safe wrist roll range (see
hardware/wrist_mount/place_camera.py), a gentle speed/acceleration profile
(config/servos.toml; see sim/docs/SERVO_LOAD_FINDINGS.md) and
a minimum tip height above the table. Unreachable targets are refused.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from so101sim import JOINTS, load_backend  # noqa: E402
from so101sim.config import load_servos  # noqa: E402
from so101sim.servo import ProfileLimiter  # noqa: E402
from so101sim.config import look_at_axes, wrist_roll_range  # noqa: E402
from so101sim.ik import ArmIK  # noqa: E402

VIEW_W, VIEW_H = 640, 360
PANEL = 30                      # title strip height above each view
LAYOUT = {"wrist": (0, 0), "scene_front": (1, 0), "scene_side": (0, 1)}   # (col, row)
LIN_SPEED = 0.10                # m/s tip speed (Shift: x0.2)
ROLL_SPEED = 1.5                # rad/s
JOINT_SPEED = 1.0               # rad/s in joint mode
WHEEL_STEP = 0.01               # m per wheel notch (joint mode: rad)
MIN_TIP_Z = 0.004               # m above the table
GRIP_OPEN, GRIP_CLOSED = 0.9, -0.17
HOME_TIP = (0.20, 0.0, 0.16)


class Teleop:
    def __init__(self, backend: str, solver: str | None, render_every: int):
        kw = {"solver": solver} if backend == "newton" else {}
        self.sim = load_backend(backend, **kw)
        self.ik = ArmIK()
        self.roll_lo, self.roll_hi = wrist_roll_range()
        jr = self.ik.m.jnt_range
        self.lo = np.array([jr[self.ik.m.joint(n).id][0] for n in JOINTS])
        self.hi = np.array([jr[self.ik.m.joint(n).id][1] for n in JOINTS])
        self.hi[4] = min(self.hi[4], self.roll_hi)
        self.lo[4] = max(self.lo[4], self.roll_lo)
        self.render_every = max(1, render_every)
        self.mode = "cartesian"
        self.sel_joint = 0
        self.paused = False
        self.message = "ready"
        self.message_t = 0.0
        self.reset_scene()

    # ---- state -------------------------------------------------------------
    def reset_scene(self):
        q, _ = self.ik.solve(HOME_TIP, [0, 0, 0, 0, self._roll_default(), GRIP_OPEN])
        self.sim.reset(q)
        self.goal = q.copy()
        f = load_servos()["follower"]   # gentle trapezoidal profile (see SERVO_LOAD_FINDINGS.md)
        self.slew = ProfileLimiter(q, f["profile_speed_rad_s"], f["profile_accel_rad_s2"])
        self.target = np.array(HOME_TIP, float)
        self.roll = q[4]
        self.grip = GRIP_OPEN
        self.images = self.sim.render()
        self.frame = 0

    def _roll_default(self) -> float:
        return float(np.clip(-math.pi / 2, self.roll_lo, self.roll_hi))

    def tip_pos(self, q=None) -> np.ndarray:
        import mujoco
        self.ik.d.qpos[:6] = self.sim.q() if q is None else q
        mujoco.mj_forward(self.ik.m, self.ik.d)
        return self.ik.d.site_xpos[self.ik.site].copy()

    def say(self, msg: str):
        self.message, self.message_t = msg, time.time()

    # ---- commands -----------------------------------------------------------
    def set_cartesian(self, target, roll=None, grip=None) -> bool:
        target = np.array(target, float)
        target[2] = max(target[2], MIN_TIP_Z)
        roll = self.roll if roll is None else float(np.clip(roll, self.roll_lo, self.roll_hi))
        grip = self.grip if grip is None else grip
        seed = self.goal.copy()
        seed[4], seed[5] = roll, grip
        q, err = self.ik.solve(target, seed, iters=120)
        if err > 0.005:
            self.say(f"unreachable ({err * 1000:.0f} mm off)")
            return False
        self.goal = np.clip(q, self.lo, self.hi)
        self.target, self.roll, self.grip = target, roll, grip
        return True

    def jog_joint(self, j: int, delta: float):
        g = self.goal.copy()
        g[j] = np.clip(g[j] + delta, self.lo[j], self.hi[j])
        self.goal = g
        # Keep the Cartesian target in sync, so switching modes doesn't jump.
        self.target, self.roll, self.grip = self.tip_pos(g), g[4], g[5]

    def toggle_grip(self):
        new = GRIP_CLOSED if self.grip > 0.3 else GRIP_OPEN
        if self.mode == "joint":
            self.goal[5] = new
            self.grip = new
        else:
            self.set_cartesian(self.target, grip=new)
        self.say("closing gripper" if new < 0.3 else "opening gripper")

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
        if self.set_cartesian([hit[0], hit[1], self.target[2]]):
            self.say(f"moving above ({hit[0]:.3f}, {hit[1]:.3f})")

    # ---- loop ---------------------------------------------------------------
    def physics(self, dt_wall: float):
        if self.paused:
            return
        n = max(1, int(round(min(dt_wall, 0.1) / self.sim.dt)))
        for _ in range(n):
            self.sim.set_targets(self.slew.update(self.goal, self.sim.dt))
            self.sim.step(1)

    def handle_held_keys(self, keys, dt: float, pg):
        fine = 0.2 if (keys[pg.K_LSHIFT] or keys[pg.K_RSHIFT]) else 1.0
        if self.mode == "cartesian":
            v = np.zeros(3)
            v[0] += (keys[pg.K_w] or keys[pg.K_UP]) - (keys[pg.K_s] or keys[pg.K_DOWN])
            v[1] += (keys[pg.K_a] or keys[pg.K_LEFT]) - (keys[pg.K_d] or keys[pg.K_RIGHT])
            v[2] += keys[pg.K_r] - keys[pg.K_f]
            droll = (keys[pg.K_e] - keys[pg.K_q]) * ROLL_SPEED * fine * dt
            if v.any() or droll:
                self.set_cartesian(self.target + v * LIN_SPEED * fine * dt, roll=self.roll + droll)
        else:
            dj = (keys[pg.K_RIGHT] or keys[pg.K_d]) - (keys[pg.K_LEFT] or keys[pg.K_a])
            if dj:
                self.jog_joint(self.sel_joint, dj * JOINT_SPEED * fine * dt)

    def handle_event(self, e, pg) -> bool:
        """Returns False to quit."""
        if e.type == pg.QUIT:
            return False
        if e.type == pg.KEYDOWN:
            k = e.key
            if k == pg.K_ESCAPE:
                return False
            if k == pg.K_SPACE:
                self.toggle_grip()
            elif k == pg.K_j:
                self.mode = "joint" if self.mode == "cartesian" else "cartesian"
                self.say(f"{self.mode} mode")
            elif k == pg.K_h:
                self.set_cartesian(HOME_TIP, roll=self._roll_default())
                self.say("home")
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
        if e.type == pg.MOUSEWHEEL:
            if self.mode == "cartesian":
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

    # ---- drawing ------------------------------------------------------------
    def draw(self, screen, font, small, pg):
        screen.fill((24, 24, 28))
        for cam, (c, r) in LAYOUT.items():
            x0, y0 = c * VIEW_W, r * (VIEW_H + PANEL)
            screen.blit(font.render(cam, True, (220, 220, 220)), (x0 + 8, y0 + 6))
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
            f"   mode {self.mode.upper()}   {'PAUSED' if self.paused else 'running'}   sim t {self.sim.time:6.1f}s",
            f"target  x {self.target[0]:+.3f}  y {self.target[1]:+.3f}  z {self.target[2]:+.3f} m"
            f"   roll {self.roll:+.2f}   grip {'open' if self.grip > 0.3 else 'closed'}",
            f"tip     x {tip[0]:+.3f}  y {tip[1]:+.3f}  z {tip[2]:+.3f} m",
        ]
        for k, ln in enumerate(lines):
            screen.blit(small.render(ln, True, (230, 230, 230)), (x0 + 12, y0 + 10 + 20 * k))
        for j, name in enumerate(JOINTS):          # joint bars
            yy = y0 + 80 + j * 24
            lo, hi = self.lo[j], self.hi[j]
            col = (255, 210, 90) if (self.mode == "joint" and j == self.sel_joint) else (200, 200, 200)
            screen.blit(small.render(f"{j + 1} {name}", True, col), (x0 + 12, yy))
            bx, bw = x0 + 160, 300
            pg.draw.rect(screen, (70, 70, 80), (bx, yy + 4, bw, 10))
            f = (q[j] - lo) / (hi - lo) if hi > lo else 0.5
            pg.draw.rect(screen, (90, 170, 255), (bx, yy + 4, int(bw * np.clip(f, 0, 1)), 10))
            screen.blit(small.render(f"{q[j]:+.2f}", True, col), (bx + bw + 10, yy))
        help_ = ["WASD/arrows move  R/F/wheel up-down  Q/E roll  Space/right-click grip",
                 "click table in a scene view: go there   Shift fine   J joint mode (1-6, L/R)",
                 "H home   Backspace reset   P pause   F12 screenshot   Esc quit"]
        for k, ln in enumerate(help_):
            screen.blit(small.render(ln, True, (150, 150, 160)), (x0 + 12, y0 + 232 + 18 * k))
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
                self.images = self.sim.render()
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
    shot = app.save_screenshot(pg, Path(__file__).resolve().parents[1] / "out" / "teleop_selftest.png")
    checks = {
        "W moves the tip forward": results["forward_dx_m"] > 0.05,
        "clicking the table moves the tip above it (<15 mm)": results["click_xy_error_m"] < 0.015,
        "wheel lowers the tip": results["wheel_z_m"] < 0.08,
        "Space closes the gripper": results["grip_after_space"] < 0.2,
        "joint mode moves shoulder_pan": results["joint_mode_pan_delta"] > 0.1,
    }
    for k, v in results.items():
        print(f"  {k}: {v:+.4f}")
    for k, ok in checks.items():
        print(("PASS " if ok else "FAIL ") + k)
    print("screenshot:", shot)
    app.selftest_ok = all(checks.values())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", choices=("mujoco", "newton"), default="mujoco")
    ap.add_argument("--solver", choices=("warp", "cpu"), default=None, help="Newton only")
    ap.add_argument("--render-every", type=int, default=None,
                    help="render cameras every N frames (default 1; 10 for Newton on CPU)")
    ap.add_argument("--selftest", action="store_true", help="headless scripted check")
    a = ap.parse_args()
    render_every = a.render_every or (10 if (a.backend == "newton" and a.solver == "cpu") else 1)
    app = Teleop(a.backend, a.solver, render_every)
    app.selftest_ok = True
    app.run(selftest=a.selftest)
    return 0 if app.selftest_ok else 1


if __name__ == "__main__":
    sys.exit(main())
