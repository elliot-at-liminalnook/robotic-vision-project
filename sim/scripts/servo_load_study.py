#!/usr/bin/env python3
"""Can the HX-30HM servos carry the wrist camera? Static + dynamic load study.

Variants of the wrist camera assembly (sim/model, hardware/wrist_mount):
  none       no camera (reference)
  designed   bracket + holder + Camera Module 3 + ESP32-P4 board (46 g)
  no_board   board moved off the wrist (34 g)
Payloads (at the gripper tip): 0, 30 g (the test cube) and 100 g.

1. STATIC (robust: depends only on masses and geometry). Gravity + payload
   holding torque per joint over the reachable workspace (fingers as close to
   down as reachable, several wrist rolls), and the worst case anywhere in the
   joint ranges. Compared with stall (2.94 N.m), the overload-protection
   trigger (80 %), and the manual's continuous ratings (1/3 and 1/5 of stall).

2. DYNAMIC (realistic servo model, so101sim.servo_model): torque-controlled
   joints with voltage saturation, back-EMF speed limit, torque limit, overload
   protection and current. Tasks:
     pick   scripted top-down pick of the 30 g cube (pick_demo planner)
     sweep  home -> far reach -> +/-0.9 rad pan sweep -> home, at max speed
     hold   arm stretched out (tip 0.36 m away, 0.10 m up) for 30 s
   Run at three position-loop stiffnesses (full voltage at 0.5 / 2 / 5 deg of
   error), because the real HX stiffness is unmeasured; droop depends on it,
   torque mostly doesn't.

Supply: the datasheet's 11.1 V (ServoSpec default; fast_teleop_study.py uses
the 12 V bench supply). Joint damping is zeroed when the servo model drives the
joints (the motor's losses are in its torque-speed line; upstream's damping
stood in for them in the ideal-actuator model; counting both halved the top
speed in the first version of this study).

Uses MuJoCo C (the same physics model Newton's SolverMuJoCo runs), so the
whole study takes minutes on a laptop.

    python sim/scripts/servo_load_study.py          # writes sim/out/servo_study/ + sim/docs/SERVO_LOAD_STUDY.md
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
sys.path.insert(0, str(SIM / "scripts"))
from so101sim import JOINTS, SlewLimiter  # noqa: E402
from so101sim.servo import ProfileLimiter  # noqa: E402
from so101sim.config import MODEL_DIR, load_wrist_mount, wrist_roll_range  # noqa: E402
from so101sim.ik import ArmIK  # noqa: E402
from so101sim.servo_model import ServoBank, ServoSpec, zero_joint_damping  # noqa: E402

OUT = SIM / "out" / "servo_study"
DOCS = SIM / "docs"
G = 9.81
SPEC = ServoSpec()
STALL = SPEC.stall_torque
VARIANTS = ("none", "designed", "no_board")
PAYLOADS = (0.0, 0.030, 0.100)
STIFFNESS_DEG = (0.5, 2.0, 5.0)
# Motion profiles for the commanded trajectory:
#   max    joints commanded at the servo's no-load speed (5.51 rad/s), no accel limit:
#          what "move as fast as possible" does (HX goal speed 0 = max).
#   gentle half the no-load speed, 8 rad/s^2 acceleration (servo accel register).
PROFILES = {"max": (5.51, None), "gentle": (2.75, 8.0)}


# ---------------------------------------------------------------- model setup
def assembly_parts():
    """Mass and centroid (gripper frame, m) of each wrist-assembly part."""
    import trimesh
    pose = load_wrist_mount()
    m = dict(pose["masses_kg"], bracket=0.014)     # bracket: sim/tools/build_model.py BRACKET_MASS
    d = MODEL_DIR / "assets" / "wrist_mount"
    files = {"bracket": "bracket.stl", "holder": "holder.stl", "board": "board.stl", "camera": "camera_module3.stl"}
    parts = {}
    for k, f in files.items():
        mesh = trimesh.load(d / f)
        c = (mesh.center_mass if mesh.is_watertight else mesh.bounding_box.centroid) / 1000.0
        parts[k] = (m[k], c, mesh.extents / 1000.0)
    return parts


def set_variant(model, variant: str, parts):
    b = model.body("wrist_camera_assembly").id
    use = {"none": [], "designed": ["bracket", "holder", "board", "camera"],
           "no_board": ["bracket", "holder", "camera"]}[variant]
    if not use:
        model.body_mass[b] = 1e-6
        model.body_inertia[b] = 1e-9
        return 0.0
    mass = sum(parts[k][0] for k in use)
    com = sum(parts[k][0] * parts[k][1] for k in use) / mass
    I = np.zeros(3)
    for k in use:                                  # boxes about the common COM (parallel axis)
        mk, ck, ek = parts[k]
        Ik = mk / 12 * np.array([ek[1] ** 2 + ek[2] ** 2, ek[0] ** 2 + ek[2] ** 2, ek[0] ** 2 + ek[1] ** 2])
        r = ck - com
        I += Ik + mk * (r @ r - r ** 2)
    model.body_mass[b] = mass
    model.body_ipos[b] = com
    model.body_iquat[b] = [1, 0, 0, 0]
    model.body_inertia[b] = I
    return mass


def load_model():
    m = mujoco.MjModel.from_xml_path(str(MODEL_DIR / "workcell.xml"))
    d = mujoco.MjData(m)
    return m, d


def qadr(m):
    return np.array([m.joint(n).qposadr[0] for n in JOINTS]), np.array([m.joint(n).dofadr[0] for n in JOINTS])


def payload_torque(m, d, mass):
    """Joint torques needed to hold `mass` kg at the gripper tip (J^T F)."""
    if mass <= 0:
        return np.zeros(len(JOINTS))
    jacp = np.zeros((3, m.nv))
    mujoco.mj_jacSite(m, d, jacp, None, m.site("gripperframe").id)
    _, dof = qadr(m)
    return -(jacp[:, dof].T @ np.array([0, 0, -mass * G]))


# ---------------------------------------------------------------- static
def static_study(parts):
    m, d = load_model()
    qa, dof = qadr(m)
    ik = ArmIK()
    lo, hi = wrist_roll_range()
    rolls = np.linspace(lo, hi, 5)
    rs = np.arange(0.10, 0.381, 0.02)
    zs = np.arange(0.01, 0.261, 0.025)
    out = {}
    # IK doesn't depend on masses: solve the grid once, reuse for every variant.
    ik_cache = {}
    seed = np.array([0, 0, 0, 0, rolls[0], 0.3])
    for iz, z in enumerate(zs):
        for ir, r in enumerate(rs):
            for roll in rolls:
                seed[4] = roll
                q, err = ik.solve([r, 0.0, z], seed, iters=150)
                if err <= 0.003:
                    ik_cache[(iz, ir, roll)] = q.copy()
                    seed = q.copy()
    for variant in VARIANTS:
        mass = set_variant(m, variant, parts)
        mujoco.mj_setConst(m, d)
        for payload in PAYLOADS:
            grid = np.full((len(zs), len(rs), len(JOINTS)), np.nan)
            for iz, z in enumerate(zs):
                for ir, r in enumerate(rs):
                    worst = np.zeros(len(JOINTS))
                    ok = False
                    for roll in rolls:
                        q = ik_cache.get((iz, ir, roll))
                        if q is None:
                            continue
                        ok = True
                        d.qpos[:] = 0
                        d.qpos[qa] = q
                        d.qvel[:] = 0
                        mujoco.mj_forward(m, d)
                        tau = np.abs(d.qfrc_bias[dof] + payload_torque(m, d, payload))
                        worst = np.maximum(worst, tau)
                    if ok:
                        grid[iz, ir] = worst
            # Worst case anywhere in the joint ranges (roll within the safe range).
            rng = np.random.default_rng(0)
            jlo = m.jnt_range[[m.joint(n).id for n in JOINTS], 0].copy()
            jhi = m.jnt_range[[m.joint(n).id for n in JOINTS], 1].copy()
            jlo[4], jhi[4] = lo, hi
            anywhere = np.zeros(len(JOINTS))
            worst_q = None
            for _ in range(20000):
                q = rng.uniform(jlo, jhi)
                d.qpos[:] = 0
                d.qpos[qa] = q
                mujoco.mj_forward(m, d)
                tau = np.abs(d.qfrc_bias[dof] + payload_torque(m, d, payload))
                if tau[1] > anywhere[1]:
                    worst_q = q.copy()
                anywhere = np.maximum(anywhere, tau)
            reach = ~np.isnan(grid[..., 0])
            mx = np.nanmax(grid, axis=(0, 1))
            frac_cont = float(np.mean(np.nanmax(grid, axis=2)[reach] > SPEC.continuous_hi))
            frac_cont_lo = float(np.mean(np.nanmax(grid, axis=2)[reach] > SPEC.continuous_lo))
            frac_trip = float(np.mean(np.nanmax(grid, axis=2)[reach] > SPEC.overload_threshold * STALL))
            out[(variant, payload)] = dict(
                assembly_mass_g=round(mass * 1000, 1), payload_g=round(payload * 1000),
                workspace_max_torque_nm=dict(zip(JOINTS, np.round(mx, 3).tolist())),
                workspace_max_frac_stall=dict(zip(JOINTS, np.round(mx / STALL, 3).tolist())),
                anywhere_max_torque_nm=dict(zip(JOINTS, np.round(anywhere, 3).tolist())),
                anywhere_worst_shoulder_pose=np.round(worst_q, 3).tolist(),
                frac_workspace_above_third_stall=round(frac_cont, 3),
                frac_workspace_above_fifth_stall=round(frac_cont_lo, 3),
                frac_workspace_above_overload_trigger=round(frac_trip, 3),
                grid=grid, rs=rs, zs=zs)
            print(f"static {variant:9s} payload {payload * 1000:3.0f} g: workspace max "
                  + " ".join(f"{j[:5]} {v / STALL * 100:4.0f}%" for j, v in zip(JOINTS, mx))
                  + f" | >1/3 stall {frac_cont * 100:4.0f}% of workspace | anywhere shoulder {anywhere[1] / STALL * 100:.0f}%")
    return out


# ---------------------------------------------------------------- dynamic
def to_torque_actuators(m):
    """Turn the MJCF position actuators into direct torque motors (ctrl = N.m)."""
    for a in range(m.nu):
        m.actuator_gaintype[a] = mujoco.mjtGain.mjGAIN_FIXED
        m.actuator_gainprm[a, :] = 0
        m.actuator_gainprm[a, 0] = 1.0
        m.actuator_biastype[a] = mujoco.mjtBias.mjBIAS_NONE
        m.actuator_biasprm[a, :] = 0
        m.actuator_ctrlrange[a] = [-STALL, STALL]
        m.actuator_forcerange[a] = [-STALL, STALL]


def run_task(task, variant, sat_deg, payload, parts, record=False, profile="max"):
    import pick_demo as pd
    m, d = load_model()
    set_variant(m, variant, parts)
    to_torque_actuators(m)
    zero_joint_damping(m, JOINTS)      # motor losses are in the servo's torque-speed line (servo_model doc)
    mujoco.mj_setConst(m, d)
    qa, dof = qadr(m)
    act = np.array([m.actuator(n).id for n in JOINTS])
    tgt_d = mujoco.MjData(m)
    ik = ArmIK()
    servo = ServoBank(len(JOINTS), ServoSpec(sat_error_deg=sat_deg))
    dt = m.opt.timestep
    tip = m.site("gripperframe").id
    gb = m.body("gripper").id

    q_open = pd.gripper_open_angle(ik, pd.CUBE_SIZE + 0.014)
    home, _ = ik.solve([0.20, 0.0, 0.16], [0, 0, 0, 0, -math.pi / 2, q_open])
    d.qpos[qa] = home
    mujoco.mj_forward(m, d)

    # Waypoints: (goal joint vector, settle seconds)
    if task == "pick":
        cube0 = d.body("cube").xpos.copy()
        above, _ = ik.solve(cube0 + [0, 0, 0.08], home)
        above = pd.square_roll(ik, above)
        above, _ = pd.solve_grasp(ik, cube0, pd.CUBE_SIZE, above, dz=0.08)
        grasp, _ = pd.solve_grasp(ik, cube0, pd.CUBE_SIZE, above, dz=-0.004)
        lift, _ = pd.solve_grasp(ik, cube0, pd.CUBE_SIZE, above, dz=0.10)
        cl = lambda q: np.r_[q[:5], pd.GRIPPER_CLOSED_CMD]  # noqa: E731
        plan = [(home, 0.5), (above, 0.5), (grasp, 0.5), (cl(grasp), 0.8), (cl(lift), 0.5), (cl(lift), 2.0)]
    else:
        far, _ = ik.solve([0.36, 0.0, 0.10 if task == "hold" else 0.06], home)
        if task == "sweep":
            left, right = far.copy(), far.copy()
            left[0], right[0] = far[0] + 0.9, far[0] - 0.9
            plan = [(home, 0.5), (far, 0.5), (left, 0.3), (right, 0.3), (far, 0.3), (home, 1.0)]
        else:   # hold
            plan = [(home, 0.5), (far, 30.0)]
    vmax, amax = PROFILES[profile]
    slew = SlewLimiter(home, vmax) if amax is None else ProfileLimiter(home, vmax, amax)
    servo_q = home.copy()
    n_steps = 0
    log = {"t": [], "tau": [], "cur": [], "err": [], "tip_err": []}
    peak = np.zeros(len(JOINTS))
    sq_sum = np.zeros(len(JOINTS))
    above_time = np.zeros(len(JOINTS))
    max_lag = np.zeros(len(JOINTS))
    max_tip_err = 0.0
    peak_total_current = 0.0
    cur_sum = 0.0
    b_tip = m.body("gripper").id

    def step(goal):
        nonlocal n_steps, max_tip_err, peak_total_current, cur_sum
        target = slew.update(goal, dt)
        # payload: a weight at the tip (only when a payload is simulated and no cube task)
        d.xfrc_applied[:] = 0
        if payload > 0 and task != "pick":
            p = d.site_xpos[tip]
            F = np.array([0, 0, -payload * G])
            d.xfrc_applied[b_tip, :3] = F
            d.xfrc_applied[b_tip, 3:] = np.cross(p - d.xipos[b_tip], F)
        tau = servo.torque(target, d.qpos[qa], d.qvel[dof], dt)
        d.ctrl[act] = tau
        mujoco.mj_step(m, d)
        n_steps += 1
        at = np.abs(tau)
        peak[:] = np.maximum(peak, at)
        sq_sum[:] += at ** 2
        above_time[:] = np.maximum(above_time, servo.over_timer)
        max_lag[:] = np.maximum(max_lag, np.abs(target - d.qpos[qa]))
        tgt_d.qpos[:] = d.qpos
        tgt_d.qpos[qa] = target
        mujoco.mj_kinematics(m, tgt_d)
        e = float(np.linalg.norm(tgt_d.site_xpos[tip] - d.site_xpos[tip]))
        max_tip_err = max(max_tip_err, e)
        tc = float(servo.last_current.sum())
        peak_total_current = max(peak_total_current, tc)
        cur_sum += tc
        if record and n_steps % 10 == 0:
            log["t"].append(n_steps * dt)
            log["tau"].append(tau.copy())
            log["cur"].append(servo.last_current.copy())
            log["err"].append(np.degrees(target - d.qpos[qa]))
            log["tip_err"].append(e)
        return e

    for goal, settle in plan:
        while np.abs(slew.current - goal).max() > 1e-4:
            step(goal)
        for _ in range(int(settle / dt)):
            last_e = step(goal)
    res = dict(
        task=task, variant=variant, sat_error_deg=sat_deg, payload_g=round(payload * 1000),
        peak_frac_stall=dict(zip(JOINTS, np.round(peak / STALL, 3).tolist())),
        rms_torque_nm=dict(zip(JOINTS, np.round(np.sqrt(sq_sum / n_steps), 3).tolist())),
        longest_above_80pct_s=dict(zip(JOINTS, np.round(above_time, 2).tolist())),
        overload_trips=int(servo.trip_count.sum()),
        trips_by_joint={j: int(c) for j, c in zip(JOINTS, servo.trip_count) if c},
        profile=profile,
        max_joint_lag_deg=dict(zip(JOINTS, np.round(np.degrees(max_lag), 2).tolist())),
        max_tip_error_mm=round(max_tip_err * 1000, 1),
        final_tip_droop_mm=round(last_e * 1000, 1),
        peak_total_current_a=round(peak_total_current, 2),
        mean_total_current_a=round(cur_sum / n_steps, 2),
        sim_time_s=round(n_steps * dt, 2),
    )
    if task == "pick":
        res["cube_lifted_m"] = round(float(d.body("cube").xpos[2] - 0.015), 4)
        res["pick_success"] = bool(res["cube_lifted_m"] > 0.07)
    if record:
        res["_log"] = {k: np.array(v) for k, v in log.items()}
    return res


def dynamic_study(parts):
    rows = []
    combos = []
    for profile in PROFILES:
        for task in ("pick", "sweep", "hold"):
            for variant in VARIANTS:
                for sat in (STIFFNESS_DEG if profile == "max" else (2.0,)):
                    for payload in ((0.0,) if task == "pick" else (0.0, 0.100)):
                        combos.append((profile, task, variant, sat, payload))
    for profile, task, variant, sat, payload in combos:
                    t0 = time.perf_counter()
                    r = run_task(task, variant, sat, payload, parts, profile=profile,
                                 record=(sat == 2.0 and variant in ("none", "designed") and payload == 0.0))
                    r["wall_s"] = round(time.perf_counter() - t0, 1)
                    rows.append(r)
                    print(f"{profile:6s} {task:5s} {variant:9s} sat {sat:3.1f} deg payload {r['payload_g']:3d} g: "
                          f"peak {max(r['peak_frac_stall'].values()) * 100:4.0f}% "
                          f"(lift {r['peak_frac_stall']['shoulder_lift'] * 100:3.0f}%, elbow {r['peak_frac_stall']['elbow_flex'] * 100:3.0f}%) "
                          f"trips {r['trips_by_joint'] or 0} tip err max {r['max_tip_error_mm']:5.1f} mm droop {r['final_tip_droop_mm']:5.1f} mm "
                          f"I peak {r['peak_total_current_a']:.1f} A mean {r['mean_total_current_a']:.2f} A" + (f" pick {'OK' if r.get('pick_success') else 'FAIL'}" if task == "pick" else ""))
    return rows


# ---------------------------------------------------------------- report
def charts(static, rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    OUT.mkdir(parents=True, exist_ok=True)
    files = {}
    # 1. Static: worst-joint torque fraction over the workspace, none vs designed (30 g payload).
    fig, axs = plt.subplots(1, 3, figsize=(15, 4.4), constrained_layout=True)
    for ax, variant in zip(axs, VARIANTS):
        s = static[(variant, 0.030)]
        frac = np.nanmax(s["grid"], axis=2) / STALL
        im = ax.imshow(frac, origin="lower", aspect="auto", vmin=0, vmax=1.0, cmap="magma",
                       extent=[s["rs"][0] - 0.01, s["rs"][-1] + 0.01, s["zs"][0] - 0.0125, s["zs"][-1] + 0.0125])
        cs = ax.contour(s["rs"], s["zs"], frac, levels=[1 / 5, 1 / 3, 0.8], colors=["#7fd", "#fd7", "#f55"], linewidths=1.5)
        ax.clabel(cs, fmt={1 / 5: "1/5", 1 / 3: "1/3", 0.8: "80% trip"}, fontsize=8)
        ax.set_title(f"{variant} ({s['assembly_mass_g']:.0f} g mount), 30 g payload")
        ax.set_xlabel("tip distance from base [m]")
        ax.set_ylabel("tip height [m]")
    fig.colorbar(im, ax=axs, label="highest joint holding torque / stall torque")
    p = OUT / "static_workspace.png"
    fig.savefig(p, dpi=110)
    plt.close(fig)
    files["static_workspace"] = p
    # 2. Static per-joint maxima.
    fig, ax = plt.subplots(figsize=(10, 4.2), constrained_layout=True)
    w = 0.13
    k = 0
    for variant in VARIANTS:
        for payload in (0.0, 0.100):
            s = static[(variant, payload)]
            vals = [s["workspace_max_frac_stall"][j] for j in JOINTS]
            ax.bar(np.arange(len(JOINTS)) + (k - 2.5) * w, vals, w, label=f"{variant}, +{payload * 1000:.0f} g")
            k += 1
    for y, lab, c in ((1 / 5, "1/5 stall (manual, conservative)", "#3a3"), (1 / 3, "1/3 stall (manual)", "#ca0"),
                      (0.8, "overload trigger 80%", "#d33")):
        ax.axhline(y, color=c, ls="--", lw=1)
        ax.text(5.45, y + 0.01, lab, color=c, fontsize=8, ha="right")
    ax.set_xticks(range(len(JOINTS)), JOINTS)
    ax.set_ylabel("max holding torque / stall")
    ax.set_title("Static holding torque, worst pose in the reachable workspace")
    ax.legend(fontsize=8, ncol=3)
    p = OUT / "static_per_joint.png"
    fig.savefig(p, dpi=110)
    plt.close(fig)
    files["static_per_joint"] = p
    # 3. Dynamic traces (sweep task, 2 deg stiffness): none vs designed.
    rec = {(r["task"], r["variant"]): r["_log"] for r in rows if "_log" in r and r["profile"] == "max"}
    rec_g = {(r["task"], r["variant"]): r["_log"] for r in rows if "_log" in r and r["profile"] == "gentle"}
    fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True, constrained_layout=True)
    for variant, ls, src, tag in (("none", "--", "max", "max"), ("designed", "-", "max", "max"),
                                  ("designed", ":", "gentle", "gentle")):
        lg = (rec if src == "max" else rec_g).get(("sweep", variant))
        variant = f"{variant}, {tag}"
        if lg is None:
            continue
        for j, name in ((1, "shoulder_lift"), (2, "elbow_flex"), (3, "wrist_flex")):
            axs[0].plot(lg["t"], np.abs(lg["tau"][:, j]) / STALL, ls, label=f"{name} ({variant})")
        axs[1].plot(lg["t"], lg["cur"].sum(axis=1), ls, label=f"total current ({variant})")
        axs[2].plot(lg["t"], np.array(lg["tip_err"]) * 1000, ls, label=f"tip error ({variant})")
    axs[0].axhline(0.8, color="#d33", lw=1, ls=":")
    axs[0].set_ylabel("|torque| / stall")
    axs[1].set_ylabel("A (all 6 servos)")
    axs[2].set_ylabel("mm")
    axs[2].set_xlabel("time [s]  (sweep task: home -> far reach -> +/-0.9 rad pan -> home)")
    for ax in axs:
        ax.legend(fontsize=8, ncol=3)
    p = OUT / "dynamic_sweep.png"
    fig.savefig(p, dpi=110)
    plt.close(fig)
    files["dynamic_sweep"] = p
    return files


def write_report(static, rows, files, parts):
    import shutil
    DOCS.mkdir(exist_ok=True)
    for k, p in files.items():
        shutil.copy2(p, DOCS / f"servo_{p.name}")
    js = {f"{v}|{int(pl * 1000)}g": {k: val for k, val in s.items() if k not in ("grid", "rs", "zs")}
          for (v, pl), s in static.items()}
    (OUT / "report.json").write_text(json.dumps({"static": js, "dynamic": [{k: v for k, v in r.items() if k != "_log"} for r in rows]}, indent=2))

    def pct(x):
        return f"{x * 100:.0f}%"

    L = []
    L.append("# Servo load study: can the HX-30HM carry the wrist camera?\n")
    L.append("Generated by `sim/scripts/servo_load_study.py`. Raw numbers: `sim/out/servo_study/report.json`.\n")
    m = {v: static[(v, 0.0)]["assembly_mass_g"] for v in VARIANTS}
    L.append(f"Mount variants: none ({m['none']:.0f} g), as designed ({m['designed']:.0f} g), "
             f"board moved off the wrist ({m['no_board']:.0f} g).\n")
    L.append("Servo limits (HX-30HM): stall 2.94 N·m; overload protection trips when torque stays above 80 % (2.35 N·m) "
             "for 2 s and then caps output at 20 % (0.59 N·m); the manual recommends continuous load of 1/5–1/3 of stall "
             "(0.59–0.98 N·m).\n")
    L.append("## 1. Static holding torque (geometry + mass only; robust)\n")
    L.append("Highest holding torque per joint anywhere in the reachable workspace (fingers as close to down as "
             "reachable, wrist roll over its camera-safe range), as a fraction of stall:\n")
    L.append("| mount | payload | " + " | ".join(JOINTS) + " | workspace > 1/3 stall | > 80 % (trip) |")
    L.append("|---|---|" + "---|" * len(JOINTS) + "---|---|")
    for v in VARIANTS:
        for pl in PAYLOADS:
            s = static[(v, pl)]
            L.append(f"| {v} | {pl * 1000:.0f} g | " + " | ".join(pct(s["workspace_max_frac_stall"][j]) for j in JOINTS)
                     + f" | {pct(s['frac_workspace_above_third_stall'])} | {pct(s['frac_workspace_above_overload_trigger'])} |")
    L.append("\nWorst case anywhere in the joint ranges (arm stretched out horizontally), shoulder_lift: "
             + ", ".join(f"{v} +{int(pl * 1000)} g: {static[(v, pl)]['anywhere_max_torque_nm']['shoulder_lift']:.2f} N·m "
                         f"({pct(static[(v, pl)]['anywhere_max_torque_nm']['shoulder_lift'] / STALL)})"
                         for v in VARIANTS for pl in (0.0, 0.100)) + ".\n")
    L.append(f"![static workspace](servo_{files['static_workspace'].name})\n")
    L.append(f"![static per joint](servo_{files['static_per_joint'].name})\n")
    L.append("## 2. Dynamic, realistic servo model\n")
    L.append("Torque-controlled joints (`so101sim/servo_model.py`): voltage saturation, back-EMF speed limit, overload "
             "protection, current. Peak torque (worst joint), overload trips, tracking and current. `sat` = position "
             "error giving full voltage (the real servo's stiffness is unmeasured).\n")
    L.append("| profile | task | mount | sat | payload | peak torque (joint) | overload trips | max tip error | final droop | peak / mean current | pick |")
    L.append("|---|---|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        j = max(r["peak_frac_stall"], key=r["peak_frac_stall"].get)
        trips = ", ".join(f"{k} ×{v}" for k, v in r["trips_by_joint"].items()) or "0"
        L.append(f"| {r['profile']} | {r['task']} | {r['variant']} | {r['sat_error_deg']}° | {r['payload_g']} g | "
                 f"{pct(r['peak_frac_stall'][j])} ({j}) | {trips} | {r['max_tip_error_mm']} mm | "
                 f"{r['final_tip_droop_mm']} mm | {r['peak_total_current_a']} / {r['mean_total_current_a']} A | "
                 f"{'' if r['task'] != 'pick' else ('✅' if r['pick_success'] else '❌')} |")
    L.append(f"\n![dynamic sweep](servo_{files['dynamic_sweep'].name})\n")
    (DOCS / "SERVO_LOAD_STUDY.md").write_text("\n".join(L) + "\n")
    return DOCS / "SERVO_LOAD_STUDY.md"


def main() -> int:
    parts = assembly_parts()
    print({k: (round(v[0] * 1000), np.round(v[1] * 1000, 1).tolist()) for k, v in parts.items()})
    static = static_study(parts)
    rows = dynamic_study(parts)
    files = charts(static, rows)
    print(write_report(static, rows, files, parts))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
