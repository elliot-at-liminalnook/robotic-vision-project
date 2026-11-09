"""Numbers that live in more than one place must agree.

Config -> generated model -> runtime -> CAD study -> docs. The bugs this
guards against: a wrist-roll range wider than what the collision study tested,
a limit enforced by two scripts but not by the model, a servo spec that
ignored the configured stiffness, README figures left over from an older run.
"""
import ast
import json
import math
import re
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

from so101sim import JOINTS
from so101sim.command import CommandFeedforward
from so101sim.config import (MODEL_DIR, ROOT, SIM, WORKCELL, load_cameras, load_servos, load_wrist_mount,
                             model_timestep, position_loop, servo_limits, wrist_roll_range)
from so101sim.servo import SlewLimiter
from so101sim.servo_model import ServoSpec

ARM = JOINTS[:5]


@pytest.fixture(scope="module")
def m():
    return mujoco.MjModel.from_xml_path(str(WORKCELL))


# ---------------------------------------------------------------- config -> generated model

def test_generated_actuators_match_the_servo_config(m):
    lim, loop = servo_limits(), position_loop()
    for name in JOINTS:
        a = m.actuator(name).id
        dof = m.joint(name).dofadr[0]
        assert m.actuator_gainprm[a][0] == pytest.approx(loop["stiffness_nm_per_rad"], rel=1e-3), name
        assert m.dof_damping[dof] == pytest.approx(loop["back_emf_nm_s_per_rad"], rel=1e-3), name
        want = lim["gripper_torque_nm"] if name == "gripper" else lim["stall_torque_nm"]
        assert np.abs(m.actuator_forcerange[a]).max() == pytest.approx(want, rel=1e-3), name
        # the actuator can't be commanded outside its joint
        assert np.allclose(m.actuator_ctrlrange[a], m.jnt_range[m.joint(name).id], atol=1e-4), name


def test_generated_cameras_match_the_camera_config(m):
    for name, spec in load_cameras().items():
        cam = m.camera(name)
        assert m.cam_fovy[cam.id] == pytest.approx(spec.fovy_deg, abs=1e-2), name
        assert np.allclose(m.cam_pos[cam.id], spec.pos, atol=1e-6), name


def test_workcell_copy_and_timestep(m):
    assert (MODEL_DIR / "workcell.xml").read_text() == (SIM / "scene" / "workcell.xml").read_text(), \
        "sim/model/workcell.xml is stale: re-run tools/build_model.py"
    assert model_timestep() == pytest.approx(m.opt.timestep)


def test_model_is_what_the_generator_would_write_now(tmp_path, monkeypatch):
    """Re-run the generator into a temp dir: the committed model must be identical (nobody forgot to rebuild)."""
    import build_model
    out = tmp_path / "model"
    monkeypatch.setattr(build_model, "OUT", out)
    monkeypatch.setattr(build_model, "ASSETS", out / "assets")
    monkeypatch.setattr(build_model, "COLL", out / "assets" / "collision")
    monkeypatch.setattr(sys, "argv", ["build_model.py"])
    assert build_model.main() == 0
    for name in ("so101_follower.xml", "cameras_world.xml", "workcell.xml"):
        assert (out / name).read_text() == (MODEL_DIR / name).read_text(), f"{name} is stale: re-run tools/build_model.py"


# ---------------------------------------------------------------- runtime reads the same numbers

def test_runtime_servo_numbers_come_from_the_config():
    lim, loop = servo_limits(), position_loop()
    spec = ServoSpec.from_config()
    assert spec.sat_error_deg == loop["sat_error_deg"], "the servo model ignores the configured stiffness"
    assert spec.tau_s == pytest.approx(lim["stall_torque_nm"])
    assert spec.w0 == pytest.approx(lim["no_load_speed_rad_s"])
    assert SlewLimiter(np.zeros(6)).max_speed == pytest.approx(lim["no_load_speed_rad_s"]), \
        "the default speed cap must be the servo's speed at the supply voltage"
    ff = CommandFeedforward()
    assert ff.stiffness == pytest.approx(loop["stiffness_nm_per_rad"])
    assert ff.k_emf == pytest.approx(loop["back_emf_nm_s_per_rad"])


def test_grip_offset_stays_under_the_grippers_overload_threshold():
    """Design check: the held squeeze must sit below the gripper servo's overload threshold with margin."""
    f, loop, spec = load_servos()["follower"], position_loop(), ServoSpec.from_config()
    hold = loop["stiffness_nm_per_rad"] * math.radians(loop["grip_offset_deg"])
    overload = spec.overload_threshold * f["gripper_torque_limit_frac"] * spec.stall_torque
    assert hold < 0.95 * overload, f"hold {hold:.2f} N.m vs overload threshold {overload:.2f} N.m"


# ---------------------------------------------------------------- wrist camera: CAD study -> model -> runtime

def test_wrist_roll_range_is_what_the_collision_study_tested(m):
    import place_camera
    lo, hi = wrist_roll_range()
    assert (lo, hi) == tuple(place_camera.ROLL_RANGE), "pose JSON and place_camera.py disagree: re-run place_camera.py"
    # The study samples grasp poses with roll in [hi - pi, hi]; nothing outside that band may be allowed.
    assert lo >= hi - math.pi - 1e-4, "allowed roll range is wider than the band the collision study sampled"
    assert hi - lo >= math.pi - 1e-3, "every top-down jaw orientation must stay reachable (range must span pi)"
    j = m.joint("wrist_roll").id
    assert np.allclose(m.jnt_range[j], (lo, hi), atol=1e-4), "the model must enforce the camera-safe roll range"
    assert np.allclose(m.actuator_ctrlrange[m.actuator("wrist_roll").id], (lo, hi), atol=1e-4)


def test_wrist_camera_can_collide_with_the_wrist_link(m):
    """The mount is welded to the gripper; without explicit pairs the engine skips its contacts with the wrist link."""
    wrist = m.joint("wrist_flex").bodyid[0]
    link_geoms = {g for g in range(m.ngeom) if m.geom_bodyid[g] == wrist and m.geom_contype[g]}
    mount_geoms = {m.geom(f"wm_{n}_collision").id for n in ("bracket", "holder", "board")}
    assert link_geoms
    pairs = {(int(a), int(b)) for a, b in zip(m.pair_geom1, m.pair_geom2)}
    pairs |= {(b, a) for a, b in pairs}
    missing = [(a, b) for a in mount_geoms for b in link_geoms if (a, b) not in pairs]
    assert not missing, f"{len(missing)} mount/wrist-link geom pairs can't collide"
    # ... and the pairs really produce contacts: fold the wrist until the mount meets the link.
    d = mujoco.MjData(m)
    hit = False
    for roll in np.linspace(*wrist_roll_range(), 25):
        for flex in np.linspace(*m.jnt_range[m.joint("wrist_flex").id], 25):
            d.qpos[:] = 0
            d.qpos[m.joint("wrist_roll").qposadr[0]] = roll
            d.qpos[m.joint("wrist_flex").qposadr[0]] = flex
            mujoco.mj_forward(m, d)
            hit |= any({int(c.geom1), int(c.geom2)} & mount_geoms and {int(c.geom1), int(c.geom2)} & link_geoms
                       for c in d.contact[:d.ncon])
    if not hit:
        pytest.skip("no wrist pose inside the safe ranges brings the mount onto the wrist link (nothing to detect)")


def test_wrist_assembly_mass_matches_the_placement_study(m):
    masses = load_wrist_mount()["masses_kg"]
    body = m.body("wrist_camera_assembly").id
    import build_model
    assert m.body_mass[body] == pytest.approx(sum(masses.values()) + build_model.BRACKET_MASS
                                              - masses.get("bracket", 0.0), abs=1e-4)


# ---------------------------------------------------------------- docs quote the data

def _text(rel):
    return (ROOT / rel).read_text()


def test_docs_quote_the_current_wrist_roll_range():
    lo, hi = wrist_roll_range()
    want = f"{lo:.2f}".replace("-", "−")
    for rel in ("hardware/wrist_mount/README.md", "sim/README.md", "docs/DECISIONS.md"):
        text = _text(rel)
        assert want in text, f"{rel} doesn't mention the roll limit {want}"
        for stale in re.findall(r"[−-]2\.74\b", text):
            # the upstream joint's own lower limit may be mentioned, but only as superseded
            assert "was" in text[max(0, text.index(stale) - 200): text.index(stale) + 200].lower(), \
                f"{rel} still quotes the old roll limit −2.74"


def test_wrist_readme_quotes_the_studys_collision_rates():
    pose = load_wrist_mount()
    bad = [c["self_collision_task_poses"] for c in pose["candidates"] if c["self_collision_task_poses"] > 0]
    want = f"{min(bad) * 100:.0f}–{math.ceil(max(bad) * 100):.0f} %"
    assert want in _text("hardware/wrist_mount/README.md"), f"README should say {want} (from wrist_mount_pose.json)"


def test_docs_quote_the_configured_servo_numbers():
    lim, loop = servo_limits(), position_loop()
    readme = _text("sim/README.md")
    for number, what in ((f"{lim['stall_torque_nm']:.2f}", "stall torque at the supply voltage"),
                         (f"{loop['stiffness_nm_per_rad']:.1f}", "position-loop stiffness"),
                         (f"{loop['sat_error_deg']:.0f}°", "saturation error")):
        assert number in readme, f"sim/README.md doesn't quote the {what} ({number})"


# ---------------------------------------------------------------- dependencies

PROVIDED = {            # import name -> requirement that provides it
    "mujoco": "newton", "warp": "newton", "mujoco_warp": "newton", "newton": "newton",
    "numpy": "numpy", "PIL": "pillow", "scipy": "scipy", "pygame": "pygame", "pytest": "pytest",
    "trimesh": "trimesh", "coacd": "coacd", "matplotlib": "matplotlib",
}


def test_every_import_is_in_requirements():
    reqs = {re.split(r"[\[=<>~; ]", ln.strip())[0].lower() for ln in (SIM / "requirements.txt").read_text().splitlines()
            if ln.strip() and not ln.lstrip().startswith("#")}
    local = {p.stem for p in SIM.rglob("*.py")} | {"so101sim", "place_camera"}
    missing = {}
    for path in SIM.rglob("*.py"):
        if ".cache" in path.parts or "out" in path.parts:
            continue
        for node in ast.walk(ast.parse(path.read_text())):
            names = [a.name for a in node.names] if isinstance(node, ast.Import) else \
                [node.module] if isinstance(node, ast.ImportFrom) and node.module and node.level == 0 else []
            for name in names:
                top = name.split(".")[0]
                if top in sys.stdlib_module_names or top in local:
                    continue
                if PROVIDED.get(top, top).lower() not in reqs:
                    missing.setdefault(top, path.relative_to(SIM))
    assert not missing, f"imported but not in sim/requirements.txt: {missing}"
