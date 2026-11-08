#!/usr/bin/env python3
"""Fast teleop study: torque governor + load-aware IK vs the fixed gentle profile.

Realistic servo model (so101sim.servo_model, HX-30HM at the 12 V bench supply
from config/servos.toml): voltage saturation, back-EMF, overload protection,
current. Same physics model as the main sim (MuJoCo C), wrist camera mounted.

Motion generators (what turns the IK goal into per-step servo targets):
  gentle    fixed trapezoidal profile, 2.75 rad/s, 8 rad/s^2 (the current default)
  max       servo no-load speed, no acceleration limit ("as fast as possible")
  governor  so101sim.governor.TorqueGovernor (fed the measured joint positions)
IK: strict (fingers straight down, 0 deg) or load-aware with a 15 deg tilt
cone (ramped to 0 deg below 10 cm, like teleop).

Tasks, scripted like an operator:
  clicks   click-to-move through 8 targets across the workspace (incl. a wide pan) and home
  stream   keyboard-style: the target slides along a path at a fixed tip speed
           (0.10 m/s = the old teleop speed, 0.25 m/s = the new fast speed)
  robust   governor with a WRONG model: real link masses +20 % and a 100 g
           payload it doesn't know about (it assumes 50 g)
Plus a static map: holding torque, strict vs load-aware, over the workspace.

    python sim/scripts/fast_teleop_study.py   # -> sim/out/fast_teleop/, sim/docs/FAST_TELEOP_STUDY.md
"""
from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))
from so101sim import JOINTS  # noqa: E402
from so101sim.config import MODEL_DIR, load_servos, wrist_roll_range  # noqa: E402
from so101sim.governor import TorqueGovernor  # noqa: E402
from so101sim.ik import ArmIK  # noqa: E402
from so101sim.motion import CartesianMove, pace_from_lag  # noqa: E402
from so101sim.servo import GoalInterpolator, ProfileLimiter, SlewLimiter  # noqa: E402
from so101sim.servo_model import ServoBank, ServoSpec, zero_joint_damping  # noqa: E402

OUT = SIM / "out" / "fast_teleop"
DOCS = SIM / "docs"
G = 9.81
SPEC = ServoSpec.from_config(sat_error_deg=2.0)
CTRL_HZ = 30                 # IK / operator input rate (teleop frame rate)
HOME = (0.20, 0.0, 0.16)
CLICKS = [(0.30, 0.15, 0.06), (0.30, -0.12, 0.05), (0.15, -0.20, 0.12), (0.30, 0.0, 0.08),
          (0.22, 0.12, 0.03), (0.16, 0.05, 0.10), (0.25, 0.22, 0.06), (0.25, -0.22, 0.06), HOME]
STREAM_PATH = [(0.20, 0.0, 0.08), (0.30, 0.0, 0.08), (0.30, 0.12, 0.08), (0.20, 0.12, 0.04),
               (0.20, -0.12, 0.04), (0.30, -0.12, 0.10), (0.30, 0.0, 0.03), (0.20, 0.0, 0.08)]
_G = load_servos()["follower"]["governor"]
CLICK_V, CLICK_A = float(_G["click_speed_m_s"]), float(_G["click_accel_m_s2"])   # straight-line click moves
PACE = float(_G["pace_lag_rad"])
TILT = 15.0
RAMP = (0.05, 0.10)          # tilt cone ramps from 0 at z <= 5 cm to full at z >= 10 cm


def tilt_for(z, tol):
    return tol * float(np.clip((z - RAMP[0]) / (RAMP[1] - RAMP[0]), 0.0, 1.0))


def make_model(mass_scale=1.0):
    m = mujoco.MjModel.from_xml_path(str(MODEL_DIR / "workcell.xml"))
    for a in range(m.nu):                      # position actuators -> torque motors (ctrl = N.m)
        m.actuator_gaintype[a] = mujoco.mjtGain.mjGAIN_FIXED
        m.actuator_gainprm[a, :] = 0
        m.actuator_gainprm[a, 0] = 1.0
        m.actuator_biastype[a] = mujoco.mjtBias.mjBIAS_NONE
        m.actuator_biasprm[a, :] = 0
        m.actuator_ctrlrange[a] = [-SPEC.tau_s, SPEC.tau_s]
        m.actuator_forcerange[a] = [-SPEC.tau_s, SPEC.tau_s]
    zero_joint_damping(m, JOINTS)
    if mass_scale != 1.0:
        for name in ("shoulder", "upper_arm", "lower_arm", "wrist", "gripper", "moving_jaw_so101_v1",
                     "wrist_camera_assembly"):
            b = m.body(name).id
            m.body_mass[b] *= mass_scale
            m.body_inertia[b] *= mass_scale
    # Motion study, not a grasp study: take the loose objects off the table so a
    # path crossing them doesn't turn into a collision (arm-table contact stays).
    for name in ("cube", "cylinder", "block"):
        b = m.body(name).id
        for gi in range(m.ngeom):
            if m.geom_bodyid[gi] == b:
                m.geom_contype[gi] = 0
                m.geom_conaffinity[gi] = 0
        m.qpos0[m.jnt_qposadr[m.body_jntadr[b]]:][:3] = [1.5, 1.5, 0.05]   # and park them off the table
    d = mujoco.MjData(m)
    mujoco.mj_setConst(m, d)
    return m, d


class Run:
    """One simulated session: servo model + motion generator + metrics."""

    def __init__(self, gen: str, tilt: float, *, mass_scale=1.0, payload=0.0):
        self.m, self.d = make_model(mass_scale)
        m = self.m
        self.qa = np.array([m.joint(n).qposadr[0] for n in JOINTS])
        self.dof = np.array([m.joint(n).dofadr[0] for n in JOINTS])
        self.act = np.array([m.actuator(n).id for n in JOINTS])
        self.tip = m.site("gripperframe").id
        self.gb = m.body("gripper").id
        self.td = mujoco.MjData(m)
        self.ik = ArmIK()
        self.tilt = tilt
        self.payload = payload
        self.dt = m.opt.timestep
        self.servo = ServoBank(len(JOINTS), SPEC)
        q0, _ = self.ik.solve(HOME, [0, 0, 0, 0, -math.pi / 2, 0.3])
        self.d.qpos[self.qa] = q0
        mujoco.mj_forward(m, self.d)
        f = load_servos()["follower"]
        self.gen_name = gen
        if gen == "gentle":
            self.gen = ProfileLimiter(q0, f["profile_speed_rad_s"], f["profile_accel_rad_s2"])
        elif gen == "max":
            self.gen = SlewLimiter(q0, SPEC.w0)
        else:
            self.gen = TorqueGovernor(q0, SPEC)
        self.goal = q0.copy()
        self.interp = GoalInterpolator(q0, 1.0 / CTRL_HZ)
        self.t = 0.0
        self.peak = np.zeros(len(JOINTS))
        self.sq = np.zeros(len(JOINTS))
        self.longest_over = np.zeros(len(JOINTS))
        self.cur_sum = 0.0
        self.cur_peak = 0.0
        self.max_lag = 0.0
        self.n = 0
        self.tilts = []
        self.droops = []
        self.table_contact_s = 0.0
        self.table = m.geom("table").id
        self.log = {"t": [], "load": [], "cur": [], "lag": [], "pred": []}

    def tip_of(self, q):
        self.td.qpos[:] = self.d.qpos
        self.td.qpos[self.qa] = q
        mujoco.mj_kinematics(self.m, self.td)
        return self.td.site_xpos[self.tip].copy()

    def set_target(self, target):
        tol = tilt_for(target[2], self.tilt)
        seed = self.goal.copy()
        q, err = self.ik.solve(target, seed, iters=30 if tol > 0 else 120, tilt_tol_deg=tol)
        if err < 0.006:
            self.goal = q
            self.interp.set(q)
        return err

    def step(self):
        m, d = self.m, self.d
        g = self.interp.advance(self.dt)
        if self.gen_name == "governor":
            cmd = self.gen.update(g, self.dt, q_meas=d.qpos[self.qa])
        else:
            cmd = self.gen.update(g, self.dt)
        d.xfrc_applied[:] = 0
        if self.payload > 0:
            p = d.site_xpos[self.tip]
            F = np.array([0, 0, -self.payload * G])
            d.xfrc_applied[self.gb, :3] = F
            d.xfrc_applied[self.gb, 3:] = np.cross(p - d.xipos[self.gb], F)
        tau = self.servo.torque(cmd, d.qpos[self.qa], d.qvel[self.dof], self.dt)
        d.ctrl[self.act] = tau
        mujoco.mj_step(m, d)
        if any(self.table in (c.geom1, c.geom2) for c in d.contact[:d.ncon]):
            self.table_contact_s += self.dt
        self.t += self.dt
        self.n += 1
        at = np.abs(tau)
        self.peak = np.maximum(self.peak, at)
        self.sq += at ** 2
        self.longest_over = np.maximum(self.longest_over, self.servo.over_timer)
        c = float(self.servo.last_current.sum())
        self.cur_sum += c
        self.cur_peak = max(self.cur_peak, c)
        lag = float(np.linalg.norm(self.tip_of(cmd) - d.site_xpos[self.tip]))
        self.max_lag = max(self.max_lag, lag)
        if self.n % 10 == 0:
            self.log["t"].append(self.t)
            self.log["load"].append(at / SPEC.tau_s)
            self.log["cur"].append(c)
            self.log["lag"].append(lag)
            self.log["pred"].append(self.gen.load_frac().copy() if self.gen_name == "governor" else np.zeros(6))
        return cmd

    def settled(self, cmd):
        """Command at the goal and the arm at rest (within 2 mm of where it will end up).

        The remaining offset from the goal is static droop (P-only servo under
        gravity), the same for every motion generator; it's reported separately.
        """
        d = self.d
        J = np.zeros((3, self.m.nv))
        mujoco.mj_jacSite(self.m, d, J, None, self.tip)
        tip_speed = float(np.linalg.norm(J @ d.qvel))
        return np.abs(cmd - self.goal).max() < 1e-4 and tip_speed < 0.005

    def metrics(self):
        trips = {j: int(c) for j, c in zip(JOINTS, self.servo.trip_count) if c}
        return dict(
            peak_frac_stall=dict(zip(JOINTS, np.round(self.peak / SPEC.tau_s, 3).tolist())),
            rms_frac_continuous=dict(zip(JOINTS, np.round(np.sqrt(self.sq / self.n) / SPEC.continuous_hi, 3).tolist())),
            longest_above_overload_s=dict(zip(JOINTS, np.round(self.longest_over, 2).tolist())),
            overload_trips=trips, peak_total_current_a=round(self.cur_peak, 2),
            mean_total_current_a=round(self.cur_sum / self.n, 2), max_tip_lag_mm=round(self.max_lag * 1000, 1),
            sim_time_s=round(self.t, 2))


def task_clicks(gen, tilt, *, path="line", **kw):
    """path='line': the tip target moves along the straight line (teleop now);
    'jump': the joint goal jumps straight to the target (teleop before)."""
    r = Run(gen, tilt, **kw)
    times, refused = [], 0
    target = np.array(HOME, float)
    steps_per_ctrl = int(round(1 / (CTRL_HZ * r.dt)))
    for click in CLICKS:
        click = np.array(click, float)
        t0 = r.t
        mv = CartesianMove(target, click, CLICK_V, CLICK_A) if path == "line" else None
        if mv is None and r.set_target(click) >= 0.006:
            refused += 1
        while True:
            if mv is not None and not mv.done:
                p = mv.step(1 / CTRL_HZ, pace_from_lag(r.goal, r.d.qpos[r.qa], PACE))
                if r.set_target(p) >= 0.006:
                    refused += 1
                    mv = None
            cmd = None
            for _ in range(steps_per_ctrl):
                cmd = r.step()
            if (mv is None or mv.done) and r.settled(cmd):
                break
            if r.t - t0 > 10.0:
                break
        target = click
        times.append(round(r.t - t0, 3))
        r.droops.append(float(np.linalg.norm(r.tip_of(r.goal) - r.d.site_xpos[r.tip])))
        r.tilts.append(r.ik.tilt_deg(r.d.qpos[r.qa]))
        for _ in range(int(0.3 / r.dt)):    # operator pause
            r.step()
    res = r.metrics()
    res.update(task="clicks", gen=gen, tilt_deg=tilt, path=path, move_times_s=times, total_move_time_s=round(sum(times), 2),
               refused_targets=refused, table_contact_s=round(r.table_contact_s, 2),
               mean_tilt_at_targets_deg=round(float(np.mean(r.tilts)), 1),
               max_droop_at_targets_mm=round(float(np.max(r.droops)) * 1000, 1),
               _log={k: np.array(v) for k, v in r.log.items()})
    return res


def task_stream(gen, tilt, speed, **kw):
    r = Run(gen, tilt, **kw)
    pts = [np.array(p, float) for p in STREAM_PATH]
    r.set_target(pts[0])
    for _ in range(int(2.0 / r.dt)):
        r.step()
    steps_per_ctrl = int(round(1 / (CTRL_HZ * r.dt)))
    errs, refused = [], 0
    target = pts[0].copy()
    for a, b in zip(pts[:-1], pts[1:]):
        L = np.linalg.norm(b - a)
        n = max(1, int(math.ceil(L / speed * CTRL_HZ)))
        for i in range(1, n + 1):
            target = a + (b - a) * i / n
            if r.set_target(target) >= 0.006:
                refused += 1
            for _ in range(steps_per_ctrl):
                r.step()
            errs.append(np.linalg.norm(r.d.site_xpos[r.tip] - target))
    t_end = r.t
    while np.linalg.norm(r.d.site_xpos[r.tip] - target) > 0.004 and r.t - t_end < 5:
        r.step()
    res = r.metrics()
    res.update(task="stream", gen=gen, tilt_deg=tilt, key_speed_m_s=speed,
               mean_track_err_mm=round(float(np.mean(errs)) * 1000, 1), max_track_err_mm=round(float(np.max(errs)) * 1000, 1),
               catch_up_s=round(r.t - t_end, 2), refused_targets=refused)
    return res


def task_jump_hazard(gen="gentle"):
    """The old click-to-move: from a low pose, jump the joint goal to a target 10 cm away."""
    out = {}
    for path in ("jump", "line"):
        r = Run(gen, 0.0)
        for start in ((0.22, 0.12, 0.03),):
            r.set_target(np.array(start))
            for _ in range(int(1.5 / r.dt)):
                r.step()
        t0 = r.t
        r.table_contact_s = 0.0
        end = np.array((0.25, 0.22, 0.06))
        if path == "jump":
            r.set_target(end)
            for _ in range(int(2.0 / r.dt)):
                r.step()
        else:
            mv = CartesianMove(np.array(start), end, CLICK_V, CLICK_A)
            spc = int(round(1 / (CTRL_HZ * r.dt)))
            while r.t - t0 < 2.0:
                if not mv.done:
                    r.set_target(mv.step(1 / CTRL_HZ, pace_from_lag(r.goal, r.d.qpos[r.qa], PACE)))
                for _ in range(spc):
                    r.step()
        res = r.metrics()
        out[path] = dict(table_contact_s=round(r.table_contact_s, 2), overload_trips=res["overload_trips"],
                         peak_frac_stall=res["peak_frac_stall"],
                         final_tip_err_mm=round(float(np.linalg.norm(r.d.site_xpos[r.tip] - end)) * 1000, 1))
    return out


def static_map():
    ik = ArmIK()
    home, _ = ik.solve(HOME, [0, 0, 0, 0, -math.pi / 2, 0.3])
    rs = np.arange(0.12, 0.341, 0.02)
    zs = np.arange(0.02, 0.201, 0.02)
    out = {}
    for tol in (0.0, 10.0, 15.0, 25.0):
        grid = np.full((len(zs), len(rs), 3), np.nan)    # shoulder, elbow, tilt
        for iz, z in enumerate(zs):
            seed = home.copy()
            for ir, r in enumerate(rs):
                q, err = ik.solve([r, 0.0, z], seed, iters=150)
                if err > 0.003:
                    continue
                seed = q.copy()
                if tol > 0:
                    q, err = ik.solve([r, 0.0, z], q, iters=150, tilt_tol_deg=tol, smooth=0.0)
                    if err > 0.003:
                        continue
                g = ik.dyn.gravity(q, 0.03)
                grid[iz, ir] = [abs(g[1]), abs(g[2]), ik.tilt_deg(q)]
        out[tol] = grid
    return rs, zs, out


def roll_alternatives():
    """How often is the 180-deg-equivalent wrist roll inside the camera-safe range, and what does it save?"""
    lo, hi = wrist_roll_range()
    rolls = np.linspace(lo, hi, 200)
    has_alt = [(lo <= r + math.pi <= hi) or (lo <= r - math.pi <= hi) for r in rolls]
    ik = ArmIK()
    home, _ = ik.solve(HOME, [0, 0, 0, 0, -math.pi / 2, 0.3])
    saves = []
    for tgt in ([0.30, 0.0, 0.06], [0.25, 0.15, 0.03], [0.22, -0.1, 0.08]):
        for r0 in (lo + 0.05, lo + 0.3):
            seed = home.copy()
            seed[4] = r0
            q, err = ik.solve(tgt, seed)
            best = ik.lighter_roll(tgt, q, lo, hi, payload_kg=0.03)
            c0 = ik.load_cost(q, 0.03)
            saves.append(dict(target=tgt, roll=round(r0, 2), alt_roll=round(float(best[0][4]), 2),
                              shoulder_nm=round(abs(ik.dyn.gravity(q, 0.03)[1]), 3),
                              shoulder_alt_nm=round(abs(ik.dyn.gravity(best[0], 0.03)[1]), 3),
                              cost_saving=round(1 - best[2] / c0, 3)))
    return dict(fraction_of_safe_range_with_alternative=round(float(np.mean(has_alt)), 3),
                safe_range_rad=[round(lo, 3), round(hi, 3)], examples=saves)


# ---------------------------------------------------------------- report
def charts(rows, smap):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    OUT.mkdir(parents=True, exist_ok=True)
    files = {}
    rs, zs, grids = smap
    fig, axs = plt.subplots(1, 3, figsize=(15, 4.2), constrained_layout=True)
    base = grids[0.0][..., 0]
    for ax, tol in zip(axs, (10.0, 15.0, 25.0)):
        red = 1 - grids[tol][..., 0] / base
        im = ax.imshow(red * 100, origin="lower", aspect="auto", vmin=-10, vmax=40, cmap="viridis",
                       extent=[rs[0] - 0.01, rs[-1] + 0.01, zs[0] - 0.01, zs[-1] + 0.01])
        ax.set_title(f"shoulder holding torque saved, {tol:.0f}° cone")
        ax.set_xlabel("tip distance from base [m]")
        ax.set_ylabel("tip height [m]")
    fig.colorbar(im, ax=axs, label="% less shoulder_lift torque than strict fingers-down (30 g payload)")
    p = OUT / "load_aware_static.png"
    fig.savefig(p, dpi=110)
    plt.close(fig)
    files["static"] = p

    clicks = {(r["gen"], r["tilt_deg"]): r for r in rows if r["task"] == "clicks" and "_log" in r and "label" not in r}
    fig, axs = plt.subplots(3, 1, figsize=(11, 8.5), sharex=False, constrained_layout=True)
    for (gen, tilt), ls in ((("gentle", 0.0), ":"), (("max", 0.0), "--"), (("governor", 0.0), "-")):
        r = clicks.get((gen, tilt))
        if r is None:
            continue
        lg = r["_log"]
        axs[0].plot(lg["t"], lg["load"][:, 1], ls, label=f"shoulder_lift ({gen})")
        axs[1].plot(lg["t"], lg["cur"], ls, label=f"total current ({gen})")
        axs[2].plot(lg["t"], np.array(lg["lag"]) * 1000, ls, label=f"tip lag behind command ({gen})")
    axs[0].axhline(SPEC.overload_torque / SPEC.tau_s, color="#d33", lw=1, ls=":")
    axs[0].text(0.2, SPEC.overload_torque / SPEC.tau_s + 0.02, "overload timer threshold", color="#d33", fontsize=8)
    axs[0].set_ylabel("|torque| / stall @12 V")
    axs[1].set_ylabel("A (all 6 servos)")
    axs[2].set_ylabel("mm")
    axs[2].set_xlabel("time [s]  (click-to-move task: 9 moves across the workspace)")
    for ax in axs:
        ax.legend(fontsize=8, ncol=3)
    p = OUT / "clicks_traces.png"
    fig.savefig(p, dpi=110)
    plt.close(fig)
    files["clicks"] = p
    return files


def write_report(rows, smap, roll, files, hazard):
    import shutil
    DOCS.mkdir(exist_ok=True)
    for k, p in files.items():
        shutil.copy2(p, DOCS / f"fast_{p.name}")
    rs, zs, grids = smap
    js = dict(rows=[{k: v for k, v in r.items() if k != "_log"} for r in rows], roll=roll, hazard=hazard,
              static={str(t): np.where(np.isnan(g), None, np.round(g, 4)).tolist() for t, g in grids.items()},
              rs=rs.tolist(), zs=zs.tolist(), spec=dict(tau_s=SPEC.tau_s, w0=SPEC.w0, supply_v=SPEC.supply_v))
    (OUT / "report.json").write_text(json.dumps(js, indent=2))

    def pk(r, j):
        return f"{r['peak_frac_stall'][j] * 100:.0f}%"

    L = ["# Fast teleop study: torque governor + load-aware IK\n",
         "Generated by `sim/scripts/fast_teleop_study.py`. Raw numbers: `sim/out/fast_teleop/report.json`. "
         "Interpretation: [FAST_TELEOP_FINDINGS.md](FAST_TELEOP_FINDINGS.md).\n",
         f"Servo model: HX-30HM at {SPEC.supply_v:.1f} V: stall {SPEC.tau_s:.2f} N·m, no-load {SPEC.w0:.2f} rad/s; "
         f"overload timer above {SPEC.overload_torque:.2f} N·m (80 % of rated stall), trips after 2 s. "
         "Position-loop stiffness: full voltage at 2° error. Wrist camera mounted (46 g).\n",
         "## 1. Click-to-move (9 moves across the workspace, incl. a wide pan sweep)\n",
         "| motion | IK | total move time | per move [s] | peak shoulder / elbow (of stall) | longest above overload threshold | trips | peak / mean current | max tip lag | tilt at targets |",
         "|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        if r["task"] != "clicks":
            continue
        tag = r.get("label", "")
        ik_label = "strict 0°" if r["tilt_deg"] == 0 else f"load-aware {r['tilt_deg']:.0f}°"
        L.append(f"| {r['gen']}{tag} | {ik_label} | "
                 f"**{r['total_move_time_s']} s** | {', '.join(f'{t:.2f}' for t in r['move_times_s'])} | "
                 f"{pk(r, 'shoulder_lift')} / {pk(r, 'elbow_flex')} | {max(r['longest_above_overload_s'].values()):.2f} s | "
                 f"{', '.join(f'{k} ×{v}' for k, v in r['overload_trips'].items()) or 0} | "
                 f"{r['peak_total_current_a']} / {r['mean_total_current_a']} A | {r['max_tip_lag_mm']} mm | {r['mean_tilt_at_targets_deg']}° |")
    L.append("\nClick-to-move from a low pose (tip 3 cm up) to a target 10 cm away, gentle profile:\n")
    L.append("| click move | time on the table | overload trips | final tip error |")
    L.append("|---|---|---|---|")
    for k, lab in (("jump", "old: joint goal jumps to the target"), ("line", "new: straight line, paced by the arm")):
        h = hazard[k]
        L.append(f"| {lab} | {h['table_contact_s']} s | {', '.join(f'{a} ×{b}' for a, b in h['overload_trips'].items()) or 0} | {h['final_tip_err_mm']} mm |")
    L += ["\n## 2. Keyboard streaming (target slides along a path)\n",
          "| motion | IK | key speed | mean / max tracking error | catch-up after release | peak shoulder / elbow | longest above threshold | trips | peak / mean current |",
          "|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        if r["task"] != "stream":
            continue
        L.append(f"| {r['gen']} | {'strict' if r['tilt_deg'] == 0 else 'load-aware'} | {r['key_speed_m_s']} m/s | "
                 f"{r['mean_track_err_mm']} / {r['max_track_err_mm']} mm | {r['catch_up_s']} s | "
                 f"{pk(r, 'shoulder_lift')} / {pk(r, 'elbow_flex')} | {max(r['longest_above_overload_s'].values()):.2f} s | "
                 f"{', '.join(f'{k} ×{v}' for k, v in r['overload_trips'].items()) or 0} | "
                 f"{r['peak_total_current_a']} / {r['mean_total_current_a']} A |")
    L += ["\n## 3. Load-aware IK, static holding torque (30 g payload, tip in the x-z plane)\n",
          "| tilt cone | mean / max shoulder_lift saving | mean / max elbow_flex change | worst shoulder torque | mean tilt used |",
          "|---|---|---|---|---|"]
    base = grids[0.0]
    for tol, g in grids.items():
        ok = ~np.isnan(g[..., 0]) & ~np.isnan(base[..., 0])
        sh = 1 - g[..., 0][ok] / base[..., 0][ok]
        el = g[..., 1][ok] / base[..., 1][ok] - 1
        L.append(f"| {tol:.0f}° | {np.mean(sh) * 100:.0f}% / {np.max(sh) * 100:.0f}% | {np.mean(el) * 100:+.0f}% / {np.max(el) * 100:+.0f}% | "
                 f"{np.nanmax(g[..., 0]):.2f} N·m ({np.nanmax(g[..., 0]) / SPEC.tau_s * 100:.0f}% of stall) | {np.nanmean(g[..., 2]):.1f}° |")
    L.append(f"\n![load-aware static](fast_{files['static'].name})\n")
    L.append("## 4. Choosing the 180°-equivalent wrist roll\n")
    L.append(f"Only {roll['fraction_of_safe_range_with_alternative'] * 100:.0f}% of the camera-safe roll range "
             f"({roll['safe_range_rad'][0]} … {roll['safe_range_rad'][1]} rad) has its 180° twin also inside the range. "
             "Where it does:\n")
    L.append("| target | roll → twin | shoulder torque | holding-cost saving |")
    L.append("|---|---|---|---|")
    for e in roll["examples"]:
        L.append(f"| {e['target']} | {e['roll']} → {e['alt_roll']} | {e['shoulder_nm']} → {e['shoulder_alt_nm']} N·m | {e['cost_saving'] * 100:.1f}% |")
    L.append(f"\n![click traces](fast_{files['clicks'].name})\n")
    (DOCS / "FAST_TELEOP_STUDY.md").write_text("\n".join(L) + "\n")
    return DOCS / "FAST_TELEOP_STUDY.md"


def main() -> int:
    t0 = time.perf_counter()
    rows = []
    for gen in ("gentle", "max", "governor"):
        for tilt in ((0.0, TILT) if gen != "max" else (0.0,)):
            r = task_clicks(gen, tilt)
            rows.append(r)
            print(f"clicks {gen:8s} tilt {tilt:4.0f}: total {r['total_move_time_s']:5.2f} s  moves {r['move_times_s']}  "
                  f"peak sh {r['peak_frac_stall']['shoulder_lift']:.2f} el {r['peak_frac_stall']['elbow_flex']:.2f}  "
                  f"over {max(r['longest_above_overload_s'].values()):.2f}s trips {r['overload_trips']}  "
                  f"I {r['peak_total_current_a']}/{r['mean_total_current_a']} A lag {r['max_tip_lag_mm']} mm", flush=True)
    for ms, pl in ((1.2, 0.10), (1.4, 0.15)):
        r = task_clicks("governor", 0.0, mass_scale=ms, payload=pl)
        r["label"] = f" (wrong model: +{(ms - 1) * 100:.0f} % mass, {pl * 1000:.0f} g unknown payload)"
        rows.append(r)
        print("robust", ms, pl, r["total_move_time_s"], r["peak_frac_stall"], r["overload_trips"], r["max_tip_lag_mm"], flush=True)
    global SPEC
    base_spec = SPEC
    for sat, td in ((0.5, 0.017), (5.0, 0.017), (2.0, 0.0)):
        SPEC = ServoSpec.from_config(sat_error_deg=sat, d_time_s=td)
        for gen in ("gentle", "governor"):
            r = task_clicks(gen, 0.0)
            r["label"] = f" (servo: full voltage at {sat}°, D {td * 1000:.0f} ms)"
            rows.append(r)
            print("servo", sat, td, gen, r["total_move_time_s"], max(r["peak_frac_stall"].values()), r["overload_trips"], flush=True)
    SPEC = base_spec
    hazard = task_jump_hazard()
    print("hazard", hazard, flush=True)
    for speed in (0.10, 0.25):
        for gen in ("gentle", "governor"):
            for tilt in (0.0, TILT):
                r = task_stream(gen, tilt, speed)
                rows.append(r)
                print(f"stream {gen:8s} tilt {tilt:4.0f} v {speed}: err {r['mean_track_err_mm']}/{r['max_track_err_mm']} mm "
                      f"catch-up {r['catch_up_s']} s peak sh {r['peak_frac_stall']['shoulder_lift']:.2f} "
                      f"trips {r['overload_trips']} refused {r['refused_targets']}", flush=True)
    smap = static_map()
    roll = roll_alternatives()
    print("roll", roll, flush=True)
    files = charts(rows, smap)
    print(write_report(rows, smap, roll, files, hazard), f"{time.perf_counter() - t0:.0f} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
