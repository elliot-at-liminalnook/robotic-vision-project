"""Both backends must behave the same through the shared API (so101sim.load_backend).

Runs on MuJoCo everywhere and on Newton where it is installed (Linux).
"""
import importlib.util

import numpy as np
import pytest

from so101sim import JOINTS, load_backend
from so101sim.config import model_timestep

BACKENDS = ["mujoco"] + (["newton"] if importlib.util.find_spec("newton") else [])
HOME = np.array([0.0, -0.6, 0.9, 0.9, -1.57, 0.5])


@pytest.fixture(scope="module", params=BACKENDS)
def sim(request):
    kw = {"solver": "cpu"} if request.param == "newton" else {}
    return load_backend(request.param, **kw)


def test_api_surface(sim):
    for name in ("reset", "q", "set_targets", "step", "time", "body_pos", "render", "cameras", "dt", "name"):
        assert hasattr(sim, name), name
    assert sim.dt == pytest.approx(model_timestep()), "the backend's step must be the model's timestep"


def test_reset_to_a_pose_holds_it(sim):
    sim.reset(HOME)
    assert np.allclose(sim.q(), HOME, atol=1e-6) and sim.time == 0.0
    sim.step(int(0.5 / sim.dt))
    assert sim.time == pytest.approx(0.5, abs=sim.dt)
    assert np.abs(sim.q() - HOME).max() < 0.08, "targets must start at the reset pose (only gravity droop allowed)"


def test_reset_clears_old_targets(sim):
    sim.reset(HOME)
    sim.set_targets(HOME + [1.0, 0.3, -0.3, 0.0, 0.5, 0.0])
    sim.reset()                                   # no pose: back to the model's default, targets included
    q0 = sim.q()
    sim.step(int(0.3 / sim.dt))
    assert np.abs(sim.q()[0] - q0[0]) < 0.02, "a target from before the reset is still driving the arm"


def test_targets_reach_the_joints(sim):
    sim.reset(HOME)
    goal = HOME + [0.4, 0.0, 0.0, 0.0, 0.3, 0.2]
    sim.set_targets(goal)
    sim.step(int(1.5 / sim.dt))
    err = np.abs(sim.q() - goal)
    assert err[[0, 4, 5]].max() < 0.03, dict(zip(JOINTS, err.round(3)))


def test_objects_rest_on_the_table(sim):
    sim.reset(HOME)
    sim.step(int(0.3 / sim.dt))
    for body, half_height in (("cube", 0.015), ("cylinder", 0.03), ("block", 0.01)):
        assert sim.body_pos(body)[2] == pytest.approx(half_height, abs=0.003), body
