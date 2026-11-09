"""Torque governor: properties that must hold for any move, checked by seeded fuzzing.

The bugs this guards against were invisible on the one click sequence the
study and the self-test use: braking scaled to zero on full-range joint jumps,
and the lag guard throttling moves only when measurements arrive once per
frame (teleop) rather than every step (the study).
"""
import numpy as np
import pytest

from so101sim.dynamics import ArmDynamics
from so101sim.governor import N_ARM, TorqueGovernor

DT = 0.002
DYN = ArmDynamics()
LO, HI = TorqueGovernor(np.zeros(6), dyn=DYN).lo, TorqueGovernor(np.zeros(6), dyn=DYN).hi


def jump(seed):
    rng = np.random.default_rng(seed)
    return rng.uniform(LO, HI), rng.uniform(LO, HI)


def drive(gov, goal, seconds, observe_every=None, plant_lag=0.0):
    """Run the governor alone. Returns (settle time or None, worst overshoot, min lag_scale, commands)."""
    sign = np.sign(goal - gov.current)
    over, min_lag, settle, cmds = 0.0, 1.0, None, []
    for i in range(int(seconds / DT)):
        if observe_every and i % observe_every == 0:
            gov.observe(gov.current - plant_lag * np.sign(gov.vel))     # an arm that follows, plant_lag behind
        c = gov.update(goal, DT)
        cmds.append(c)
        over = max(over, float(((c - goal) * sign)[:N_ARM].max()))
        min_lag = min(min_lag, gov.lag_scale)
        if np.abs(c - goal)[:N_ARM].max() < 1e-3 and np.abs(gov.vel[:N_ARM]).max() < 0.05:
            settle = i * DT
            break
    return settle, over, min_lag, np.array(cmds)


# 200 seeds: the old braking bug needed a fast, heavily loaded joint and showed on about 1 jump in 20.
@pytest.mark.parametrize("payload", [0.05, 0.3])
@pytest.mark.parametrize("seed", range(200))
def test_any_joint_space_jump_arrives_without_running_past_the_goal(seed, payload):
    q0, goal = jump(seed)
    gov = TorqueGovernor(q0, dyn=DYN, payload_kg=payload)
    settle, over, _, cmds = drive(gov, goal, 3.5)
    assert (cmds >= LO - 1e-9).all() and (cmds <= HI + 1e-9).all(), "commanded outside the joint limits"
    assert settle is not None, "never arrived (coasting or stuck)"
    assert settle < 3.0
    assert over < 0.12, f"ran {over:.2f} rad past the goal"


@pytest.mark.parametrize("seed", range(10))
def test_command_stays_in_limits_even_with_a_payload_the_arm_cannot_carry(seed):
    """1 kg is beyond the servos; the governor may refuse to move, but it must not run away."""
    q0, goal = jump(seed)
    gov = TorqueGovernor(q0, dyn=DYN, payload_kg=1.0)
    _, _, _, cmds = drive(gov, goal, 4.0)
    assert np.isfinite(cmds).all()
    assert (cmds >= LO - 1e-9).all() and (cmds <= HI + 1e-9).all()


def test_goal_outside_the_limits_is_clamped():
    gov = TorqueGovernor(np.zeros(6), dyn=DYN)
    _, _, _, cmds = drive(gov, HI + 1.0, 4.0)
    assert (cmds <= HI + 1e-9).all()
    assert np.allclose(cmds[-1][:N_ARM], HI[:N_ARM], atol=2e-3)


@pytest.mark.parametrize("observe_every", [1, 5, 17, 50])      # every step ... every 100 ms
def test_lag_guard_ignores_how_often_the_arm_is_measured(observe_every):
    """An arm that tracks perfectly must never be throttled, whatever the measurement rate."""
    q0 = np.array([-1.2, -0.3, 0.4, 0.3, -1.5, 0.5])
    goal = np.array([1.2, 0.2, -0.3, -0.4, 0.5, 0.5])
    ref, _, _, _ = drive(TorqueGovernor(q0, dyn=DYN), goal, 6.0)
    settle, _, min_lag, _ = drive(TorqueGovernor(q0, dyn=DYN), goal, 6.0, observe_every=observe_every)
    assert min_lag == 1.0, f"throttled to {min_lag:.2f} with perfect tracking"
    assert settle == pytest.approx(ref, abs=0.02)


@pytest.mark.parametrize("observe_every", [1, 17])
def test_lag_guard_slows_down_for_an_arm_that_really_lags(observe_every):
    q0 = np.array([-1.2, -0.3, 0.4, 0.3, -1.5, 0.5])
    goal = np.array([1.2, 0.2, -0.3, -0.4, 0.5, 0.5])
    ref, _, _, _ = drive(TorqueGovernor(q0, dyn=DYN), goal, 6.0)
    settle, _, min_lag, _ = drive(TorqueGovernor(q0, dyn=DYN), goal, 8.0, observe_every=observe_every, plant_lag=0.2)
    assert min_lag < 0.5
    assert settle is None or settle > ref * 1.3
