"""The stop path: freeze (hold where the arm is) and torque off, by key and on every way out.

Aimed at classes of bugs:
  * a stop that does not stop: from any motion, the arm is at rest near where it was, and stays;
  * a stop that drops things: a freeze keeps a held object held;
  * torque off that is not off: no servo delivers torque, on either backend and over a faulty bus,
    and nothing a controller does while it is off (keys, clicks, goals) reaches the servos;
  * torque on that jumps: the arm holds where it lies, whatever was asked for before;
  * an unconfirmed switch: a broadcast that a servo missed is noticed and repaired, and a servo
    that cannot be reached is named;
  * a way out of the program that leaves torque on.
"""
import os
import signal
import subprocess
import sys

import numpy as np
import pytest

from harness import Rig
from so101sim import JOINTS, load_backend
from so101sim.bus import BusError
from so101sim.bus_backend import BusBackend
from so101sim.config import SIM

HOME = np.array([0.0, -0.6, 0.9, 0.9, -1.57, 0.5])
FAULTS = {"drop_reply_prob": 0.05, "drop_command_prob": 0.05, "corrupt_prob": 0.05}
RIGS = {"direct": None, "bus": {}, "faulty bus": FAULTS}


def true_q(sim):
    return sim.true_q() if hasattr(sim, "true_q") else sim.q()


def servo_torque(sim) -> np.ndarray:
    """What the servos put on the joints right now [N.m]."""
    inner = getattr(sim, "inner", sim)
    return np.abs(inner.d.actuator_force[inner.act])


def joint_speed(sim) -> float:
    inner = getattr(sim, "inner", sim)
    dof = [inner.m.joint(n).dofadr[0] for n in JOINTS]
    return float(np.abs(inner.d.qvel[dof][:5]).max())


@pytest.fixture(params=list(RIGS))
def rig(request):
    faults = RIGS[request.param]
    r = Rig(bus=None if faults is None else {"seed": 1})
    for name, p in (faults or {}).items():       # faults start once the arm is connected and holding
        setattr(r.app.sim.wire, name, p)
    return r


# ---------------------------------------------------------------- freeze
@pytest.mark.parametrize("motion", ["click", "keys", "joint jog", "home"])
@pytest.mark.parametrize("after_s", [0.15, 0.4])
def test_stop_from_any_motion_holds_the_arm_where_it_is(rig, motion, after_s):
    app, pg = rig.app, rig.pg
    if motion == "click":
        rig.click_table((0.28, 0.12))
    elif motion == "keys":
        rig.keys.hold(pg.K_w, pg.K_a)
    elif motion == "joint jog":
        rig.key(pg.K_j)
        rig.keys.hold(pg.K_RIGHT)
    else:
        rig.click_table((0.26, -0.12))
        rig.run(1.5)
        rig.key(pg.K_h)
    rig.run(after_s)
    assert joint_speed(app.sim) > 0.2, "the scenario is not moving when the stop comes"
    at_stop = true_q(app.sim)
    rig.key(pg.K_x)
    worst = 0.0

    def watch():
        nonlocal worst
        worst = max(worst, float(np.abs(true_q(app.sim)[:5] - at_stop[:5]).max()))
    rig.run(0.7, each=watch)                 # any movement key is still held down: it must not undo the stop
    assert app.move is None
    rig.keys.hold()
    rig.run(0.8, each=watch)
    assert joint_speed(app.sim) < 0.02, "still moving a second after the stop"
    assert worst < 0.12, f"the arm ran on {np.degrees(worst):.1f} deg after the stop"
    assert np.abs(app.goal[:5] - true_q(app.sim)[:5]).max() < 0.06, "the goal is not where the arm is"
    rig.mon.assert_ok()


def test_stop_keeps_a_held_object_held():
    rig = Rig()
    app, pg = rig.app, rig.pg
    tip, roll, q_open = rig.grasp_pose()
    app.move_to(tip + [0, 0, 0.08], "above", roll=roll)
    rig.run(1.5)
    app.move_to(tip, "down")
    rig.run(1.5)
    rig.key(pg.K_SPACE)
    rig.run(1.0)
    app.move_to(tip + [0, 0, 0.10], "lift")
    rig.run(0.25)
    rig.key(pg.K_x)
    rig.run(1.5)
    assert app.grip_latch.holding and app.sim.body_pos("cube")[2] > 0.02, "the cube was dropped by the stop"
    rig.mon.assert_ok()


# ---------------------------------------------------------------- torque off / on
def test_torque_off_means_no_servo_pushes_and_nothing_gets_through(rig):
    app, pg = rig.app, rig.pg
    rig.click_table((0.28, 0.10))
    rig.run(0.2)
    rig.key(pg.K_t)
    assert app.limp and not app.sim.torque_enabled
    rig.run(0.1)
    goal, peak = app.goal.copy(), 0.0

    def watch():
        nonlocal peak
        peak = max(peak, float(servo_torque(app.sim).max()))
    rig.keys.hold(pg.K_w, pg.K_r)                           # everything a person might still press
    for k in (pg.K_SPACE, pg.K_h, pg.K_j, pg.K_x, pg.K_g):
        rig.key(k)
    rig.click_table((0.22, -0.10))
    rig.event(pg.MOUSEWHEEL, y=3)
    rig.run(1.5, each=watch)
    assert peak == 0.0, f"a servo delivered {peak:.2f} N.m with torque off"
    assert np.array_equal(app.goal, goal) and app.move is None, "a command was accepted while limp"
    assert true_q(app.sim)[1] > HOME[1] + 0.3 or app.tip_pos(true_q(app.sim))[2] < 0.05, "a limp arm should have sagged"
    if hasattr(app.sim, "servos"):
        assert [s.get("torque_switch") for s in app.sim.servos] == [0] * 6


@pytest.mark.parametrize("seed", range(3))
def test_torque_on_holds_the_arm_where_it_lies(rig, seed):
    """Wherever the arm was heading and wherever it has fallen to, switching torque on moves nothing."""
    app, pg = rig.app, rig.pg
    rng = np.random.default_rng(seed)
    rig.click_table((float(rng.uniform(0.2, 0.3)), float(rng.uniform(-0.12, 0.12))))
    rig.run(float(rng.uniform(0.1, 0.6)))
    rig.key(pg.K_t)
    rig.run(float(rng.uniform(0.3, 2.0)))                   # falling, or already lying on the table
    for _ in range(4):                                      # a faulty bus may refuse; the answer must then be "still off"
        rig.key(pg.K_t)
        if not app.limp:
            break
        assert not app.sim.torque_enabled and "stays OFF" in app.message
    assert not app.limp and app.sim.torque_enabled, app.message
    lying = true_q(app.sim)
    worst = 0.0

    def watch():
        nonlocal worst
        worst = max(worst, float(np.abs(true_q(app.sim)[:5] - lying[:5]).max()))
    rig.run(1.5, each=watch)
    # A jump back to the old command would be about 1 rad. An arm caught in mid-fall runs on a little: the
    # servo needs degrees of error to brake it, and over a faulty bus the switch takes tens of ms to arrive.
    assert worst < 0.25, f"the arm moved {np.degrees(worst):.1f} deg when torque came back"
    assert joint_speed(app.sim) < 0.02
    rig.keys.hold(pg.K_r)                                   # and it is a working arm again
    rig.run(1.0)
    rig.keys.hold()
    assert app.tip_pos(true_q(app.sim))[2] > app.tip_pos(lying)[2] + 0.03, "the arm does not move after torque on"


def test_a_reset_switches_torque_back_on(rig):
    app, pg = rig.app, rig.pg
    rig.key(pg.K_t)
    rig.run(0.5)
    if hasattr(app.sim, "wire"):                 # (start-up refuses to go on over a faulty bus: test_bus.py)
        app.sim.wire.drop_reply_prob = app.sim.wire.drop_command_prob = app.sim.wire.corrupt_prob = 0.0
    rig.key(pg.K_BACKSPACE)
    rig.run(1.0)
    assert not app.limp and app.sim.torque_enabled
    assert np.abs(true_q(app.sim)[:5] - app.goal[:5]).max() < 0.08, "the arm is not held after the reset"


# ---------------------------------------------------------------- the switch itself, over the bus
@pytest.mark.parametrize("seed", range(8))
def test_a_missed_broadcast_is_noticed_and_repaired(seed):
    """A broadcast write has no reply. With a third of all packets and replies lost, torque off still ends
    with every servo off or with an error that names the ones that are not."""
    bus = BusBackend(load_backend("mujoco"), drop_command_prob=0.0, drop_reply_prob=0.0, seed=seed)
    bus.reset(HOME)
    bus.wire.drop_command_prob = bus.wire.drop_reply_prob = 0.33
    try:
        bus.torque_off()
        named = ""
    except BusError as e:
        named = str(e)
    assert not bus.torque_enabled
    for s, joint in zip(bus.servos, JOINTS):
        assert s.get("torque_switch") == 0 or joint in named, f"{joint} is still on and nobody was told"
    sent = bus.driver.stats["tx_packets"]
    bus.set_targets(HOME + 0.2)
    bus.step(50)
    assert bus.driver.stats["tx_packets"] == sent, "goals were written with torque off"


def test_a_servo_that_missed_its_goal_is_not_switched_on_to_an_old_one():
    """Power-on goal is 0 ticks. If enable()'s goal broadcast is lost, torque on would fling the arm there."""
    bus = BusBackend(load_backend("mujoco"))
    for seed in range(6):
        bus.wire.rng.seed(seed)
        bus.wire.drop_command_prob = 0.3
        try:
            bus.reset(HOME)
        except BusError:
            continue                                        # refused to start: fine
        finally:
            bus.wire.drop_command_prob = 0.0
        bus.step(int(0.3 / bus.dt))
        assert np.abs(bus.true_q() - HOME).max() < 0.08, f"seed {seed}: the arm jumped at start-up"


def test_a_dead_bus_is_reported_and_no_goal_is_sent_afterwards():
    rig = Rig(bus={})
    app, pg = rig.app, rig.pg
    rig.run(0.2)
    app.sim.wire.drop_reply_prob = 1.0
    rig.key(pg.K_t)
    assert app.limp and "NOT CONFIRMED" in app.message and "shoulder_pan" in app.message
    assert "NOT CONFIRMED" in app.shutdown()
    sent = app.sim.driver.stats["tx_packets"]
    rig.keys.hold(pg.K_w)
    rig.run(0.3)
    assert app.sim.driver.stats["tx_packets"] == sent


# ---------------------------------------------------------------- every way out
def test_shutdown_leaves_torque_off(rig):
    app = rig.app
    rig.click_table((0.28, 0.10))
    rig.run(0.2)
    for _ in range(3):
        if "NOT CONFIRMED" not in app.shutdown():
            break
    rig.run(0.3)
    assert servo_torque(app.sim).max() == 0.0


@pytest.mark.parametrize("how", ["esc", "error", "SIGINT", "SIGTERM"])
def test_every_way_out_of_teleop_switches_torque_off(how, tmp_path):
    """The real program, with the bus: its exit report must show no servo left with torque on."""
    script = tmp_path / "run.py"
    script.write_text(f"""
import os, signal, sys
os.environ["SDL_VIDEODRIVER"] = "dummy"
sys.path.insert(0, {str(SIM / "scripts")!r})
import teleop
frames = 0
physics = teleop.Teleop.physics
def patched(self, dt):
    global frames
    frames += 1
    if frames == 20:
        if {how!r} == "esc":
            import pygame as pg
            pg.event.post(pg.event.Event(pg.KEYDOWN, key=pg.K_ESCAPE, mod=0, unicode="", scancode=0))
        elif {how!r} == "error":
            raise RuntimeError("a bug in the control loop")
        else:
            os.kill(os.getpid(), getattr(signal, {how!r}))
    physics(self, dt)
teleop.Teleop.physics = patched
sys.argv = ["teleop.py", "--bus"]
sys.exit(teleop.main())
""")
    r = subprocess.run([sys.executable, str(script)], capture_output=True, text=True, timeout=120)
    assert "stop path: torque off on every servo" in r.stdout, r.stdout[-800:] + r.stderr[-800:]
    assert "with torque on: none" in r.stdout, r.stdout[-800:]
    assert (r.returncode == 0) == (how == "esc"), (r.returncode, r.stderr[-400:])


def test_the_api_refuses_to_move_a_limp_arm_and_says_why():
    """A remote caller must not switch on what the person at the arm switched off."""
    import json
    import sim_api as A
    api = A.SimApi()
    api.app.set_torque(False)
    before = api.app.goal.copy()
    for path, body in (("/move_tip", {"x": 0.22, "y": 0.0, "z": 0.12}), ("/move_joints", {"shoulder_pan": 0.3}),
                       ("/gripper", {"opening": 0.5})):
        status, _, out = A.dispatch(api, "POST", path, json.dumps(body).encode())
        r = json.loads(out)
        assert status == 409 and not r["ok"] and "torque" in r["error"], (path, r)
    assert api.app.limp and not api.app.sim.torque_enabled and np.array_equal(api.app.goal, before)
    assert A.dispatch(api, "POST", "/wait", json.dumps({"seconds": 0.2}).encode())[0] == 200
    assert A.dispatch(api, "POST", "/reset", b"")[0] == 200 and not api.app.limp
    assert json.loads(A.dispatch(api, "POST", "/move_joints", json.dumps({"shoulder_pan": 0.3}).encode())[2])["reached"]
