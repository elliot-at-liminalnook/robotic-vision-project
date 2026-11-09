#!/usr/bin/env python3
"""Render the README pictures and GIFs of the simulated arm into docs/media/.

    python sim/tools/render_media.py            # all (~1-2 min on a laptop CPU)
    python sim/tools/render_media.py --only orbit pick

Uses the plain MuJoCo backend (OpenGL), so it runs anywhere MuJoCo does. The
pick GIF replays the scripted pick from scripts/pick_demo.py (same IK, servo
slew limit, feed-forward and grip-by-offset) and shows the wrist camera
beside a free camera.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim"))
sys.path.insert(0, str(ROOT / "sim" / "scripts"))
from so101sim import SlewLimiter  # noqa: E402
from so101sim.command import CommandFeedforward, GripOffset  # noqa: E402
from so101sim.ik import ArmIK  # noqa: E402
from so101sim.mujoco_backend import MujocoBackend  # noqa: E402

OUT = ROOT / "docs" / "media"


def free_cam(lookat, distance, azimuth, elevation) -> mujoco.MjvCamera:
    c = mujoco.MjvCamera()
    c.type = mujoco.mjtCamera.mjCAMERA_FREE
    c.lookat[:] = lookat
    c.distance, c.azimuth, c.elevation = distance, azimuth, elevation
    return c


def load_sim() -> MujocoBackend:
    """The workcell with a gradient sky added (render-only; physics unchanged)."""
    from so101sim.config import WORKCELL
    sim = MujocoBackend()
    spec = mujoco.MjSpec.from_file(str(WORKCELL))
    spec.add_texture(name="render_sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                     rgb1=[0.82, 0.86, 0.92], rgb2=[0.24, 0.27, 0.32], width=512, height=3072)
    sim.m = spec.compile()
    sim.d = mujoco.MjData(sim.m)
    return sim


class Shooter:
    def __init__(self, sim: MujocoBackend):
        self.sim = sim
        self._r: dict = {}
        self.opt = mujoco.MjvOption()

    def shot(self, cam, w: int, h: int) -> np.ndarray:
        if (h, w) not in self._r:
            self._r[(h, w)] = mujoco.Renderer(self.sim.m, h, w)
        r = self._r[(h, w)]
        r.update_scene(self.sim.d, camera=cam, scene_option=self.opt)
        return r.render().copy()


def save_gif(frames, path: Path, fps: float, colors: int = 255):
    """Animated GIF with one shared palette (smaller and flicker-free)."""
    imgs = [Image.fromarray(f) for f in frames]
    # Palette from a strip of sampled frames so it covers the whole clip.
    sample = imgs[:: max(1, len(imgs) // 8)]
    strip = Image.new("RGB", (sample[0].width, sample[0].height * len(sample)))
    for i, im in enumerate(sample):
        strip.paste(im, (0, i * im.height))
    pal = strip.quantize(colors=colors, method=Image.Quantize.FASTOCTREE)
    q = [im.quantize(palette=pal, dither=Image.Dither.NONE) for im in imgs]
    q[0].save(path, save_all=True, append_images=q[1:], duration=int(round(1000 / fps)), loop=0,
              optimize=True, disposal=1)
    print(f"  {path.relative_to(ROOT)}  {len(frames)} frames, {path.stat().st_size / 1e6:.1f} MB")


def label(img: np.ndarray, text: str) -> np.ndarray:
    im = Image.fromarray(img)
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, 8 + 7 * len(text), 20], fill=(20, 20, 24))
    d.text((5, 4), text, fill=(235, 235, 235))
    return np.asarray(im)


def pose(ik: ArmIK, tip, q_seed=None, grip=0.6):
    q, _ = ik.solve(tip, np.r_[q_seed[:5], grip] if q_seed is not None else [0, 0, 0, 0, 0, grip])
    q[5] = grip
    return q


# --------------------------------------------------------------------------- stills

def stills(sim: MujocoBackend, sh: Shooter, ik: ArmIK):
    q = pose(ik, [0.22, 0.05, 0.10])
    sim.reset(q)
    hero = sh.shot(free_cam([0.14, 0.02, 0.11], 0.52, 305, -20), 1280, 800)
    Image.fromarray(hero).save(OUT / "arm_hero.png")
    print(f"  docs/media/arm_hero.png")

    # Gripper + wrist camera close-up: look at the gripper body.
    gp = sim.d.body("gripper").xpos.copy()
    close = sh.shot(free_cam(gp + [0.0, 0.02, -0.01], 0.24, 200, -12), 1280, 800)
    Image.fromarray(close).save(OUT / "wrist_closeup.png")
    print(f"  docs/media/wrist_closeup.png")

    # What the three cameras see at that pose.
    views = sim.render()
    names = [n for n in ("scene_front", "scene_side", "wrist") if n in views]
    tiles = [label(np.asarray(Image.fromarray(views[n]).resize((480, 360))), n) for n in names]
    Image.fromarray(np.concatenate(tiles, axis=1)).save(OUT / "camera_views.png")
    print(f"  docs/media/camera_views.png")


# --------------------------------------------------------------------------- orbit

def orbit(sim: MujocoBackend, sh: Shooter, ik: ArmIK, n: int = 90):
    """Arm reaching back and forth while the camera circles it."""
    a = pose(ik, [0.20, -0.08, 0.14])
    b = pose(ik, [0.26, 0.10, 0.06], a)
    frames = []
    for i in range(n):
        s = 0.5 - 0.5 * np.cos(2 * np.pi * i / n)
        q = (1 - s) * a + s * b
        sim.reset(q)
        frames.append(sh.shot(free_cam([0.13, 0.0, 0.11], 0.58, 90 + 360 * i / n, -20), 640, 400))
    save_gif(frames, OUT / "arm_orbit.gif", fps=20)


# --------------------------------------------------------------------------- joints

def joints(sim: MujocoBackend, sh: Shooter, ik: ArmIK, per: int = 24):
    """Each joint swept in turn from a neutral pose: what the six servos do."""
    from so101sim import JOINTS
    base = pose(ik, [0.20, 0.0, 0.14], grip=0.3)
    amp = {"shoulder_pan": 0.7, "shoulder_lift": 0.45, "elbow_flex": 0.45, "wrist_flex": 0.7,
           "wrist_roll": 1.2, "gripper": 0.6}
    lo, hi = sim.m.jnt_range[[sim.m.joint(j).id for j in JOINTS]].T
    cam = free_cam([0.12, 0.0, 0.12], 0.60, 135, -20)
    frames = []
    for j, name in enumerate(JOINTS):
        for i in range(per):
            q = base.copy()
            q[j] = np.clip(base[j] + amp[name] * np.sin(2 * np.pi * i / per), lo[j], hi[j])
            sim.reset(q)
            frames.append(label(sh.shot(cam, 640, 400), name))
    save_gif(frames, OUT / "arm_joints.gif", fps=16)


# --------------------------------------------------------------------------- pick

def pick(sim: MujocoBackend, sh: Shooter, ik: ArmIK, fps: float = 25):
    import pick_demo as pd
    q_open = pd.gripper_open_angle(ik, pd.CUBE_SIZE + 0.014)
    home, _ = ik.solve([0.20, 0.0, 0.16], [0, 0, 0, 0, 0, q_open])
    sim.reset(home)
    sim.step(int(0.3 / sim.dt))
    cube0 = sim.body_pos("cube")
    above, _ = ik.solve(cube0 + [0, 0, 0.08], home)
    above = pd.square_roll(ik, above)
    above, _ = pd.solve_grasp(ik, cube0, pd.CUBE_SIZE, above, dz=0.08)
    grasp, _ = pd.solve_grasp(ik, cube0, pd.CUBE_SIZE, above, dz=-0.004)
    lift, _ = pd.solve_grasp(ik, cube0, pd.CUBE_SIZE, above, dz=0.10)
    closed = lambda q: np.r_[q[:5], pd.GRIPPER_CLOSED_CMD]  # noqa: E731
    phases = [("home", home, 0.3), ("above", above, 0.2), ("descend", grasp, 0.2),
              ("close", closed(grasp), 0.6), ("lift", closed(lift), 0.2), ("hold", closed(lift), 0.6),
              ("place", closed(grasp), 0.2), ("release", grasp, 0.4), ("home", home, 0.4)]

    slew, ff, grip = SlewLimiter(sim.q()), CommandFeedforward(), GripOffset()
    chunk = 5
    every = max(1, int(round(1.0 / fps / (chunk * sim.dt))))
    cam = free_cam([0.15, 0.02, 0.10], 0.64, 150, -22)
    frames, k = [], 0

    def tick(goal, move: bool, ph: str):
        nonlocal k
        cmd = None
        if move:
            for _ in range(chunk):
                cmd = slew.update(goal, sim.dt)
        else:
            cmd = np.array(goal, float)
        cmd = np.array(cmd, float)
        cmd[5] = grip.update(cmd[5], float(sim.q()[5]), chunk * sim.dt)
        sim.set_targets(ff.apply(cmd, chunk * sim.dt))
        sim.step(chunk)
        k += 1
        if k % every == 0:
            free = label(sh.shot(cam, 560, 420), f"sim  t={sim.time:4.1f}s  {ph}")
            wrist = sim.render(["wrist"])["wrist"]
            wrist = label(np.asarray(Image.fromarray(wrist).resize((560, 420))), "wrist camera")
            frames.append(np.concatenate([free, wrist], axis=1))

    for ph, goal, settle in phases:
        while np.abs(slew.current - goal).max() > 1e-4:
            tick(goal, True, ph)
        for _ in range(int(settle / (chunk * sim.dt))):
            tick(goal, False, ph)
    # Every 2nd frame at half the rate keeps the file small but plays in real time.
    save_gif(frames[::2], OUT / "pick_demo.gif", fps=fps / 2)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", nargs="*", choices=("stills", "orbit", "joints", "pick"))
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    sim = load_sim()
    sh, ik = Shooter(sim), ArmIK()
    for name, fn in (("stills", stills), ("orbit", orbit), ("joints", joints), ("pick", pick)):
        if not args.only or name in args.only:
            print(name)
            fn(sim, sh, ik)


if __name__ == "__main__":
    main()
