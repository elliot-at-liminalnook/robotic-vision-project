"""The arm's wires as part of the model (so101sim/cables.py, config/cables.toml).

Aimed at classes of bugs:
  * a wire too short somewhere in the joint range (checked for every wire over the whole range);
  * a wire laid out wrongly at a reset: not reaching its socket, or thrown about (any arm pose);
  * wires that disturb the simulation: jamming a joint, exploding, sinking through the table,
    being stretched (watched while the arm moves through random poses);
  * the cabled model drifting from the plain one (it must be the same robot plus wires).
"""
import json
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from so101sim import JOINTS, load_backend
from so101sim.cables import load_cables, resolve_anchor, slack_path
from so101sim.config import MODEL_DIR, servo_limits
from so101sim.monitor import PhysicsMonitor

HOME = np.array([0.0, -0.6, 0.9, 0.9, -1.57, 0.5])
SPANS = json.loads((MODEL_DIR / "cables.json").read_text())


@pytest.fixture(scope="module")
def sims():
    return load_backend("mujoco"), load_backend("mujoco", cables=True)


@pytest.mark.parametrize("seed", range(5))
def test_slack_path_meets_both_ends_with_equal_segments(seed):
    rng = np.random.default_rng(seed)
    for _ in range(200):
        n, seg = int(rng.integers(4, 20)), float(rng.uniform(0.01, 0.03))
        a, d0 = rng.normal(0, 0.1, 3), rng.normal(0, 1, 3)
        b = a + rng.normal(0, 1, 3) * rng.uniform(0.0, 1.0) * n * seg * 0.55
        pts = slack_path(a, d0, b, n, seg, rng.normal(0, 1, 3))
        lengths = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        assert pts.shape == (n + 1, 3) and np.allclose(pts[0], a)
        assert np.allclose(lengths, seg, atol=1e-6), "segments of unequal length"
        assert np.allclose(pts[1] - pts[0], d0 / np.linalg.norm(d0) * seg, atol=1e-9), "does not leave along the connector"
        if np.linalg.norm(b - pts[1]) < (n - 1) * seg * 0.98:              # reachable: the far end is met
            assert np.linalg.norm(pts[-1] - b) < 1e-4


def test_every_wire_is_long_enough_over_the_whole_joint_range():
    """The furthest apart a wire's two ends ever get must be less than its length, with 5 % to spare."""
    root = ET.parse(MODEL_DIR / "so101_follower_cables.xml").getroot()       # (it has the fixed cameras' placeholder bodies too)
    m = mujoco.MjModel.from_xml_path(str(MODEL_DIR / "so101_follower_cables.xml"))
    d = mujoco.MjData(m)
    ends = []
    for cable in load_cables()["cables"]:
        points = [cable["from"], *cable.get("via", []), cable["to"]]
        lengths = cable["lengths_m"] if "lengths_m" in cable else [cable["length_m"]]
        for k, length in enumerate(lengths):
            ends.append((cable["name"], k, length, resolve_anchor(root, points[k]), resolve_anchor(root, points[k + 1])))
    assert len(ends) == len(SPANS)
    rng = np.random.default_rng(0)
    lo = np.array([m.joint(j).range[0] for j in JOINTS])
    hi = np.array([m.joint(j).range[1] for j in JOINTS])
    qadr = [m.joint(j).qposadr[0] for j in JOINTS]
    corners = [np.where(rng.random(6) < 0.5, lo, hi) for _ in range(400)]       # the range's corners stretch wires most
    worst = np.zeros(len(ends))
    for q in corners + [rng.uniform(lo, hi) for _ in range(3000)]:
        d.qpos[qadr] = q
        mujoco.mj_kinematics(m, d)

        def world(anchor):
            b = 0 if anchor[0] == "world" else m.body(anchor[0]).id
            return d.xmat[b].reshape(3, 3) @ anchor[1] + d.xpos[b]
        worst = np.maximum(worst, [np.linalg.norm(world(a) - world(b)) for *_, a, b in ends])
    for (name, k, length, *_), far in zip(ends, worst):
        assert far < length / 1.05, f"{name} stretch {k}: {length * 1000:.0f} mm long, its ends get {far * 1000:.0f} mm apart"


def test_the_cabled_model_is_the_same_robot_plus_wires(sims):
    plain, cabled = sims
    for j in JOINTS:
        a, b = plain.m.joint(j), cabled.m.joint(j)
        assert np.allclose(a.range, b.range) and np.allclose(a.axis, b.axis)
        assert np.allclose(plain.m.actuator(j).gainprm, cabled.m.actuator(j).gainprm)
        assert np.allclose(plain.m.actuator(j).forcerange, cabled.m.actuator(j).forcerange)
    for b in ("base", "shoulder", "upper_arm", "lower_arm", "wrist", "gripper", "moving_jaw_so101_v1", "wrist_camera_assembly"):
        assert plain.m.body(b).mass[0] == cabled.m.body(b).mass[0]
    wire_bodies = [b for b in range(cabled.m.nbody) if cabled.m.body(b).name.startswith("cable_")]
    assert len(wire_bodies) == sum(s["segments"] for s in SPANS) and cabled.m.neq == len(SPANS)
    wire_mass = float(cabled.m.body_mass[wire_bodies].sum())
    assert 0.03 < wire_mass < 0.12, f"{wire_mass * 1000:.0f} g of wire"
    extra = {cabled.m.body(b).name for b in range(cabled.m.nbody)} - {plain.m.body(b).name for b in range(plain.m.nbody)}
    assert {n for n in extra if not n.startswith("cable_")} == {"scene_front_camera", "scene_side_camera"}, \
        "besides wires, only the fixed cameras' placeholder hardware may be added"
    assert any(s["ribbon"] for s in SPANS) and any(not s["ribbon"] for s in SPANS)


@pytest.mark.parametrize("seed", range(4))
def test_wires_are_laid_out_reaching_their_sockets_and_at_rest_for_any_pose(sims, seed):
    plain, cabled = sims
    rng = np.random.default_rng(seed)
    lo, hi = plain.m.actuator_ctrlrange[plain.act].T
    q = HOME if seed == 0 else np.clip(HOME + rng.uniform(-0.6, 0.6, 6) * [1, 0.5, 0.5, 0.5, 1, 0.5], lo + 0.05, hi - 0.05)
    cabled.reset(q)
    assert cabled.time == 0.0 and np.allclose(cabled.q(), q, atol=1e-9), "laying out the wires moved the arm or the clock"
    mon = PhysicsMonitor(cabled, attach=False)
    for st in cabled.cables.state(cabled.d):
        assert st["end_gap_m"] < 0.002, f"{st['name']} ends {st['end_gap_m'] * 1000:.1f} mm from its socket"
    plain.reset(q)
    for sim in (plain, cabled):
        sim.set_targets(q)
    for _ in range(500):
        cabled.step(1)
        mon.check()
    plain.step(500)
    mon.assert_ok()
    m, d = cabled.m, cabled.d
    speed = max(float(np.abs(d.qvel[da:da + (1 if s["ribbon"] else 3)]).max()) for s in cabled.cables.spans for da in s["dadr"])
    assert speed < 1.0, f"a wire is still whipping about at {speed:.1f} rad/s a second after the reset"
    low = min(float(d.xpos[b][2]) for b in range(m.nbody) if m.body(b).name.startswith("cable_"))
    assert low > -0.003, f"a wire is {-low * 1000:.0f} mm under the table top"
    assert np.abs(cabled.q() - plain.q()).max() < 0.02, "the wires hold the arm away from where it would rest without them"
    assert max(st["tension_n"] for st in cabled.cables.state(d)) < 3.0, cabled.cables.report(d)


def test_moving_through_random_poses_with_wires_breaks_nothing(sims):
    plain, cabled = sims
    rng = np.random.default_rng(7)
    lo, hi = plain.m.actuator_ctrlrange[plain.act].T
    for sim in (plain, cabled):
        sim.reset(HOME)
    mon = PhysicsMonitor(cabled, attach=False)
    worst_pull, worst_gap = 0.0, 0.0
    for _ in range(6):
        goal = np.clip(HOME + rng.uniform(-1.0, 1.0, 6) * [1.5, 0.5, 0.6, 0.6, 0.7, 0.5], lo + 0.05, hi - 0.05)
        for sim in (plain, cabled):
            sim.set_targets(goal)
        for k in range(600):
            cabled.step(1)
            mon.check()
            if k % 20 == 0:
                st = cabled.cables.state(cabled.d)
                worst_pull = max(worst_pull, max(s["tension_n"] for s in st))
                worst_gap = max(worst_gap, max(s["end_gap_m"] for s in st))
        plain.step(600)
        assert np.abs(cabled.q() - plain.q()).max() < 0.03, f"with wires the arm ends {np.abs(cabled.q() - plain.q()).max():.3f} rad off"
    mon.assert_ok()
    assert worst_gap < 0.003, f"a wire came {worst_gap * 1000:.1f} mm away from its socket"
    assert worst_pull < 5.0, f"a wire was pulled with {worst_pull:.1f} N"
    stall = servo_limits()["stall_torque_nm"]
    extra = np.abs(cabled.d.actuator_force[cabled.act] - plain.d.actuator_force[plain.act])[:5].max()
    assert extra < 0.05 * stall, f"holding against the wires costs {extra / stall:.0%} of stall torque"


def test_teleop_runs_with_wires():
    from harness import Rig
    rig = Rig(cables=True)
    app = rig.app
    assert app.sim.cables is not None
    rig.click_table((0.28, 0.10))
    rig.run(2.5)
    assert np.linalg.norm(app.tip_pos()[:2] - (0.28, 0.10)) < 0.015
    rig.mon.assert_ok()
