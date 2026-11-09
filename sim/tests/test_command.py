"""Grip-by-offset and feed-forward, against a simple jaw model, for every way of commanding them.

The teleop bug this guards against: the latch only worked for a command that
jumped to "closed" (pick_demo) and never for one that ramped (teleop). So
every behaviour here is checked for all command shapes and update rates.
"""
import math

import numpy as np
import pytest

from so101sim.command import CommandFeedforward, GripOffset
from so101sim.config import position_loop, servo_limits

LOOP, LIM = position_loop(), servo_limits()
STALL = LIM["stall_torque_nm"]
T_LIMIT = LIM["gripper_torque_nm"]
OPEN, CLOSED = 0.9, -0.17
DT = 0.002


class Jaw:
    """One gripper servo (the sim's actuator model) closing on a springy object at angle `contact`."""

    def __init__(self, q=OPEN, contact=0.10, object_stiffness=60.0):
        self.q, self.v, self.contact, self.kc = q, 0.0, contact, object_stiffness
        self.torque = 0.0

    def step(self, target):
        k, c, inertia = LOOP["stiffness_nm_per_rad"], LOOP["back_emf_nm_s_per_rad"], 0.028
        self.torque = float(np.clip(k * (target - self.q), -T_LIMIT, T_LIMIT))
        push = self.kc * (self.contact - self.q) if (self.contact is not None and self.q < self.contact) else 0.0
        # semi-implicit: the back-EMF damping is stiff next to the 2 ms step
        self.v = (self.v + DT * (self.torque + push) / inertia) / (1 + DT * (c + (2.0 if push else 0.0)) / inertia)
        self.q += self.v * DT


def command(shape: str, t: float, start=OPEN) -> float:
    if shape == "step":
        return CLOSED
    rate = {"ramp_fast": 2.75, "ramp_slow": 0.10, "stream_30hz": 2.75}[shape]
    if shape == "stream_30hz":                 # a ramp that only updates once per frame
        t = math.floor(t * 30) / 30
    return max(CLOSED, start - rate * t)


SHAPES = ["step", "ramp_fast", "ramp_slow", "stream_30hz"]
EVERY = [1, 5, 17]                              # physics steps per latch update (500 Hz, 100 Hz, 30 Hz)


def run(jaw, latch, shape, seconds, every, t0=0.0, start=OPEN, cmd_fn=None, log=None):
    target = jaw.q
    for i in range(int(seconds / DT)):
        t = t0 + i * DT
        if i % every == 0:
            cmd = cmd_fn(t) if cmd_fn else command(shape, t, start)
            target = latch.update(cmd, jaw.q, every * DT)
        jaw.step(target)
        if log is not None:
            log.append((t, jaw.q, jaw.torque, latch.holding))


@pytest.mark.parametrize("every", EVERY)
@pytest.mark.parametrize("shape", SHAPES)
def test_latches_on_contact_and_holds_at_the_designed_squeeze(shape, every):
    jaw, latch, log = Jaw(), GripOffset(), []
    run(jaw, latch, shape, 12.0, every, log=log)
    assert latch.holding, "never latched: the gripper would sit at its torque limit"
    hold = abs(jaw.torque) / STALL
    design = math.radians(LOOP["grip_offset_deg"]) / LOOP["sat_error_rad"]      # 3 deg / 9 deg = 33 %
    assert design - 0.06 <= hold <= design + 0.04, f"holding at {hold:.0%} of stall, designed {design:.0%}"
    # ... and it never sat near the torque limit for long on the way there.
    at_limit = sum(DT for _, _, tq, _ in log if abs(tq) > 0.8 * T_LIMIT)
    assert at_limit < 1.0, f"{at_limit:.2f} s above 80 % of the gripper's torque limit"


@pytest.mark.parametrize("every", EVERY)
@pytest.mark.parametrize("shape", SHAPES)
def test_never_latches_on_empty_air(shape, every):
    jaw, latch = Jaw(contact=None), GripOffset()
    run(jaw, latch, shape, 12.0, every)
    assert not latch.holding
    assert abs(jaw.q - CLOSED) < 0.01, "the jaw must reach the commanded angle when nothing is in the way"


@pytest.mark.parametrize("every", EVERY)
def test_latches_when_the_jaws_already_touch_the_object(every):
    jaw, latch = Jaw(q=0.10, contact=0.10), GripOffset()
    run(jaw, latch, "step", 2.0, every)
    assert latch.holding


@pytest.mark.parametrize("every", EVERY)
@pytest.mark.parametrize("shape", SHAPES)
def test_releases_when_the_object_is_lost(shape, every):
    jaw, latch = Jaw(), GripOffset()
    run(jaw, latch, shape, 12.0, every)
    assert latch.holding
    jaw.contact = None                                    # the object slips out
    run(jaw, latch, "step", 2.0, every)
    assert not latch.holding, "still 'holding' with nothing in the jaws"
    assert abs(jaw.q - CLOSED) < 0.01


@pytest.mark.parametrize("every", EVERY)
@pytest.mark.parametrize("opening", ["step", "ramp"])
def test_opening_releases_without_squeezing_harder(opening, every):
    jaw, latch, log = Jaw(), GripOffset(), []
    run(jaw, latch, "step", 3.0, every)
    hold = abs(jaw.torque)
    # A ramped opening starts from "closed" (where the command had got to), as a profile generator would.
    fn = (lambda t: OPEN) if opening == "step" else (lambda t: min(OPEN, CLOSED + 2.75 * t))
    run(jaw, latch, None, 2.0, every, cmd_fn=fn, log=log)
    assert not latch.holding and jaw.q > 0.8
    worst = max(-tq for _, _, tq, _ in log)               # closing torque is negative
    assert worst <= hold * 1.05, f"opening squeezed to {worst:.2f} N.m (was holding {hold:.2f})"


# ---------------------------------------------------------------- feed-forward

def test_feedforward_offset_is_bounded_for_any_command():
    """No command (steps, noise, NaN-free garbage) may produce an offset beyond what the servo could use."""
    ff = CommandFeedforward()
    rng = np.random.default_rng(0)
    lo, hi = ff.dyn.m.jnt_range[[ff.dyn.m.joint(j).id for j in
                                 ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")]].T
    g_max = 0.0
    worst = 0.0
    for _ in range(400):
        q = np.r_[rng.uniform(lo, hi), rng.uniform(-0.17, 1.7)]          # jumps anywhere each tick
        out = ff.apply(q, DT)
        g_max = max(g_max, float(np.abs(ff.dyn.gravity(q)[:5]).max()))
        worst = max(worst, float(np.abs(out[:5] - q[:5]).max()))
        assert out[5] == q[5], "the gripper command must pass through untouched"
    bound = (g_max + ff.k_emf * LIM["no_load_speed_rad_s"]) / ff.stiffness
    assert worst <= bound + 1e-9, f"offset {math.degrees(worst):.0f} deg, bound {math.degrees(bound):.0f} deg"


def test_feedforward_has_no_velocity_term_right_after_reset():
    ff = CommandFeedforward()
    q = np.array([0.3, -0.5, 0.6, 0.4, -1.0, 0.5])
    ff.apply(np.zeros(6), DT)
    ff.reset()
    first = ff.apply(q, DT)
    again = ff.apply(q, DT)                               # same command, zero velocity
    assert np.allclose(first, again)


def test_feedforward_cancels_gravity_at_the_model_stiffness():
    """The offset times the actuator's kp must equal the gravity torque (model and compensation agree)."""
    import mujoco
    from so101sim.config import MODEL_DIR
    m = mujoco.MjModel.from_xml_path(str(MODEL_DIR / "so101_follower.xml"))
    ff = CommandFeedforward()
    q = np.array([0.2, -0.4, 0.7, 0.5, -1.2, 0.4])
    off = ff.apply(q, DT)[:5] - q[:5]
    kp = np.array([m.actuator_gainprm[m.actuator(j).id][0] for j in
                   ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")])
    assert np.allclose(kp * off, ff.dyn.gravity(q)[:5], atol=2e-3)
