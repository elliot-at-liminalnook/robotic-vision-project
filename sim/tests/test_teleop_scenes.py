"""The scenes teleop.py can be started in (sim/scene/*.xml): each has its models and starts at rest.

(Kept from test_tasks.py when the old single-arm API and its bench task were retired: the bench's task is now a
task of the scene file sim/scenes/bench.toml, done through the hub's API in test_bench_solution.py.)
"""
import mujoco
import numpy as np
import pytest

import teleop as T
from so101sim import load_backend
from so101sim.config import MODEL_DIR, SIM
from so101sim.monitor import PhysicsMonitor

SCENES = sorted(p.stem for p in (SIM / "scene").glob("*.xml"))


def test_every_scene_has_its_models_and_sits_still():
    assert {"workcell", "bench"} <= set(SCENES)
    for scene in SCENES:
        assert (MODEL_DIR / f"{scene}.xml").read_text() == (SIM / "scene" / f"{scene}.xml").read_text()
        assert (MODEL_DIR / f"{scene}_cables.xml").exists()
        sim = T.Teleop("mujoco", None, render_every=10 ** 9, scene=scene).sim      # the arm at teleop's home pose
        mon = PhysicsMonitor(sim)
        m, d = sim.m, sim.d
        free = [m.jnt_bodyid[j] for j in range(m.njnt) if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE]
        start = {b: d.xpos[b].copy() for b in free}
        sim.step(500)
        mon.assert_ok()
        for b in free:
            assert np.linalg.norm(d.xpos[b] - start[b]) < 0.0005, f"{scene}: {m.body(b).name} does not start at rest"
    with pytest.raises(ValueError, match="no scene"):
        load_backend("mujoco", scene="nowhere")
