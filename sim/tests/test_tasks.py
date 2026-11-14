"""Scenes and tasks (sim/scene/, so101sim/tasks.py).

Aimed at classes of bugs:
  * a task that cannot be done: the reference solution (sim/scripts/bench_solution.py) does the bench task
    through the REST API alone, in a closed loop, with the physics monitor clean;
  * a scene that is not at rest or is built overlapping: every scene sits still from the start;
  * goals that are true when they should not be: at the start only the trivially true ones hold, and
    undoing any one of them afterwards makes that goal (and the task) false again;
  * a task whose description does not match its scene: every place named in the description is where
    the model has it.
"""
import json
import re

import mujoco
import numpy as np
import pytest

import bench_solution as B
import sim_api as A
from so101sim import load_backend
from so101sim.config import MODEL_DIR, SIM
from so101sim.monitor import PhysicsMonitor
from so101sim.tasks import TASKS

SCENES = sorted(p.stem for p in (SIM / "scene").glob("*.xml"))


def caller(api):
    def call(method, path, body=None):
        return json.loads(A.dispatch(api, method, path, b"" if body is None else json.dumps(body).encode())[2])
    return call


@pytest.fixture(scope="module")
def solved():
    api = A.SimApi(scene="bench")
    log = []
    task = B.Solver(caller(api), log=log.append).solve()
    return api, task, log


def test_every_scene_has_its_models_and_sits_still():
    assert {"workcell", "bench"} <= set(SCENES)
    for scene in SCENES:
        assert (MODEL_DIR / f"{scene}.xml").read_text() == (SIM / "scene" / f"{scene}.xml").read_text()
        assert (MODEL_DIR / f"{scene}_cables.xml").exists()
        sim = A.T.Teleop("mujoco", None, render_every=10 ** 9, scene=scene).sim      # the arm at teleop's home pose
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


def test_the_bench_task_starts_undone():
    api = A.SimApi(scene="bench")
    r = caller(api)("GET", "/task")
    assert r["task"] == "bench" and not r["done"]
    assert {g["goal"] for g in r["goals"] if g["done"]} == {"drawer closed", "everything at rest"}
    assert caller(A.SimApi())("GET", "/task")["task"] is None, "the plain workcell has no task"


def test_the_bench_task_can_be_done_through_the_api(solved):
    api, task, log = solved
    assert task["done"], [g for g in task["goals"] if not g["done"]]
    st = caller(api)("GET", "/state")
    assert not st["checks"]["physics_violations"] and not st["gripper"]["holding"]
    assert st["time_s"] < 120, "the reference solution should take about a minute of simulated time"
    assert set(st["checks"]["command_filter"]) <= {"jump", "joint_limit", "table"}


def test_each_goal_is_false_again_when_undone(solved):
    api, task, _ = solved
    m, d = api.inner.m, api.inner.d
    keep = d.qpos.copy()

    def moved(body, dx=0.0, dz=0.0, tip=False):
        d.qpos[:] = keep
        a = m.jnt_qposadr[m.body(body).jntadr[0]]
        d.qpos[a] += dx
        d.qpos[a + 2] += dz
        if tip:
            d.qpos[a + 3:a + 7] = [0.7071, 0.7071, 0, 0]            # lying on its side
        mujoco.mj_forward(m, d)
        return {g["goal"]: g["done"] for g in api.task.check(d)}

    assert not moved("red_cube", dx=0.03)["red cube in the red bin"]
    assert not moved("blue_cylinder", tip=True)["blue cylinder upright in the blue bin"]
    assert not moved("yellow_peg", dz=0.02)["yellow peg in the socket"]
    assert not moved("green_block", tip=True)["green block on the platform"]
    assert not moved("purple_cube", dx=0.02)["purple cube on the green block"]
    d.qpos[:] = keep
    d.qpos[m.joint("drawer").qposadr[0]] = 0.03
    mujoco.mj_forward(m, d)
    assert not api.task.done(d)
    d.qpos[:] = keep
    mujoco.mj_forward(m, d)
    assert api.task.done(d)


def test_the_description_names_places_where_the_model_has_them():
    m = mujoco.MjModel.from_xml_path(str(MODEL_DIR / "bench.xml"))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    text = TASKS["bench"].description
    for label, body in (("red bin centre", "red_bin"), ("blue bin centre", "blue_bin"), ("platform centre", "platform"),
                        ("socket centre", "socket")):
        x, y = map(float, re.search(re.escape(label) + r" \(([-0-9.]+), ([-0-9.]+)\)", text).groups())
        assert np.allclose(d.body(body).xpos[:2], [x, y], atol=1e-6), label
    hx, hy = map(float, re.search(r"handle \(closed\) at \(([-0-9.]+), ([-0-9.]+)\)", text).groups())
    assert np.allclose(d.geom("drawer_handle").xpos[:2], [hx, hy], atol=1e-6) and hx == B.HANDLE_X
    assert f"up to {m.joint('drawer').range[1] * 1000:.0f} mm" in text
    task = TASKS["bench"](m)
    assert np.allclose(task.RED_BIN, d.body("red_bin").xpos[:2]) and np.allclose(task.BLUE_BIN, d.body("blue_bin").xpos[:2])
    assert np.allclose(task.PLATFORM, d.body("platform").xpos[:2]) and np.allclose(task.SOCKET, d.body("socket").xpos[:2])
