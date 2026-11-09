"""Teleop driven headless, with the invariant monitor watching every frame.

Each scenario asserts its own result AND that no invariant broke on the way
(harness.Monitor): no servo stalled or in overload, nothing commanded outside
its limits, the tip never under the table. The old self-test passed while the
gripper sat at its torque limit, because it only looked at end positions.
"""
import math

import numpy as np
import pytest

from harness import FRAME, Rig
from so101sim import JOINTS
from so101sim.config import position_loop, servo_limits, wrist_roll_range

STALL = servo_limits()["stall_torque_nm"]


@pytest.fixture()
def rig():
    r = Rig()
    yield r
    r.mon.assert_ok()            # every scenario must also keep the invariants


@pytest.mark.parametrize("fast", [True, False])
def test_grasp_holds_the_cube_gently_lifts_it_and_lets_go(rig, fast):
    app, pg = rig.app, rig.pg
    if not fast:
        rig.key(pg.K_g)
    cube0 = app.sim.body_pos("cube")
    tip, roll, q_open = rig.grasp_pose()
    assert app.set_cartesian([tip[0], tip[1], 0.09], roll=roll, grip=q_open)
    rig.run(2.5)
    assert app.set_cartesian(tip, roll=roll)
    rig.run(2.0)
    app.toggle_grip()
    rig.run(3.0)                                         # longer than the servo's 2 s overload time
    assert app.grip_latch.holding, "grip-by-offset did not engage: the gripper is stalled at its torque limit"
    design = math.radians(position_loop()["grip_offset_deg"]) / position_loop()["sat_error_rad"]
    hold = abs(rig.gripper_torque()) / STALL
    assert design - 0.08 <= hold <= design + 0.05, f"squeezing at {hold:.0%} of stall, designed {design:.0%}"
    assert app.set_cartesian(tip + [0, 0, 0.10])
    rig.run(2.5)
    assert app.sim.body_pos("cube")[2] - cube0[2] > 0.08, "cube not lifted"
    assert app.grip_latch.holding
    squeeze = []
    app.toggle_grip()
    rig.run(1.5, each=lambda: squeeze.append(-rig.gripper_torque()))
    assert not app.grip_latch.holding and app.sim.q()[5] > 0.5
    assert max(squeeze) <= hold * STALL * 1.1, "opening squeezed harder first"


@pytest.mark.parametrize("fast", [True, False])
@pytest.mark.parametrize("joint", ["shoulder_pan", "shoulder_lift", "elbow_flex"])
def test_load_panel_agrees_with_the_torque_the_servo_delivers(rig, fast, joint):
    """What the panel shows must track the simulated servo's torque, in both motion modes."""
    app, pg = rig.app, rig.pg
    if not fast:
        rig.key(pg.K_g)
    j = JOINTS.index(joint)
    rig.key(pg.K_j)
    rig.key(pg.K_1 + j)
    shown, real = [], []

    def sample():
        shown.append(app.load_frac()[j])
        real.append(abs(rig.mon.net_torque()[j]) / STALL)

    for key in (pg.K_RIGHT, pg.K_LEFT, None):               # short jogs: stay clear of the arm's own links
        rig.keys.hold(*([key] if key else []))
        rig.run(0.25 if key else 0.5, each=sample)
    shown, real = np.array(shown), np.array(real)
    # The panel predicts the torque for the commanded motion; the servo's follows it loosely (it lags).
    assert np.abs(shown - real).mean() < 0.15, f"panel off by {np.abs(shown - real).mean():.0%} of stall on average"
    assert shown.max() < real.max() + 0.25, f"panel peaked at {shown.max():.0%}, the servo at {real.max():.0%}"


def test_click_in_joint_mode_acts_now_not_later(rig):
    app, pg = rig.app, rig.pg
    rig.key(pg.K_j)
    assert app.mode == "joint"
    rig.click_table((0.24, -0.08))
    assert app.mode == "cartesian", "a click that starts a move must leave joint mode"
    rig.run(3.0)
    assert np.linalg.norm(app.tip_pos()[:2] - [0.24, -0.08]) < 0.015
    assert app.move is None


def test_no_move_survives_a_mode_switch(rig):
    app, pg = rig.app, rig.pg
    rig.click_table((0.24, -0.08))
    rig.run(0.1)
    rig.key(pg.K_j)                                       # into joint mode mid-move
    assert app.move is None
    rig.run(1.0)
    goal = app.goal.copy()
    rig.key(pg.K_j)                                       # back: nothing stale may start
    rig.run(1.0)
    assert np.allclose(app.goal, goal)


@pytest.mark.parametrize("joint", ["shoulder_lift", "elbow_flex", "wrist_flex"])
@pytest.mark.parametrize("direction", ["K_LEFT", "K_RIGHT"])
def test_joint_jog_cannot_drive_the_tip_into_the_table(rig, joint, direction):
    app, pg = rig.app, rig.pg
    rig.key(pg.K_j)
    rig.key(pg.K_1 + JOINTS.index(joint))
    rig.keys.hold(getattr(pg, direction))
    rig.run(4.0)                                          # the monitor checks tip height and stalls


def test_arm_blocked_by_itself_stops_pushing(rig):
    """Folding the elbow drives the jaws into the shoulder: the stall guard must give up, not hold 100 % torque."""
    app, pg = rig.app, rig.pg
    rig.key(pg.K_j)
    rig.key(pg.K_3)
    said = set()
    rig.keys.hold(pg.K_RIGHT)
    rig.run(4.0, each=lambda: said.add(app.message))
    rig.keys.hold()
    rig.run(1.0)
    assert any("blocked" in m for m in said), said
    # (the arm springs back a little once the pushing stops)
    assert np.abs(app.goal[:5] - app.sim.q()[:5]).max() < 0.2, "the goal is still far past where the arm can go"


def test_paused_means_nothing_moves_including_the_target(rig):
    app, pg = rig.app, rig.pg
    rig.key(pg.K_p)
    target, goal, q = app.target.copy(), app.goal.copy(), app.sim.q()
    rig.keys.hold(pg.K_w)
    rig.run(1.0)
    rig.keys.hold()
    rig.event(pg.MOUSEWHEEL, x=0, y=-3)
    rig.click_table((0.24, -0.08))
    rig.run(0.5)
    assert np.allclose(app.target, target) and np.allclose(app.goal, goal) and np.allclose(app.sim.q(), q)
    rig.key(pg.K_p)
    rig.run(0.5)
    assert np.abs(app.sim.q() - q).max() < 0.02, "the arm jumped on unpause"


@pytest.mark.parametrize("bad", [[float("nan"), 0.0, 0.1], [0.2, float("inf"), 0.1], [5.0, 0.0, 0.1], [0.2, 0.0, 3.0]])
def test_bad_targets_are_refused_and_leave_the_goal_alone(rig, bad):
    app = rig.app
    goal = app.goal.copy()
    assert not app.set_cartesian(bad)
    assert not app.move_to(bad, "x")
    assert np.array_equal(app.goal, goal)
    rig.run(0.2)


def test_wrist_roll_stays_inside_the_camera_safe_range(rig):
    app, pg = rig.app, rig.pg
    lo, hi = wrist_roll_range()
    seen = []
    for key in (pg.K_e, pg.K_q):
        rig.keys.hold(key)
        rig.run(4.0, each=lambda: seen.append(app.sim.q()[4]))
    rig.keys.hold()
    assert min(seen) >= lo - 0.05 and max(seen) <= hi + 0.05
    assert max(seen) > hi - 0.1 and min(seen) < lo + 0.1, "the whole safe range should be usable"


def test_clicks_across_the_workspace_keep_every_invariant(rig):
    app = rig.app
    for xy in [(0.28, 0.12), (0.16, -0.14), (0.30, -0.05), (0.20, 0.12), (0.22, 0.0)]:
        rig.click_table(xy)
        rig.run(2.5)
        assert np.linalg.norm(app.tip_pos()[:2] - xy) < 0.015
