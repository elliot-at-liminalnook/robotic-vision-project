"""Every script starts, explains itself, and fails with a message (not a traceback) on bad arguments.

Plus the scripted end-to-end checks themselves, so `pytest sim/tests` is the one command to run.
"""
import subprocess
import sys

import numpy as np
import pytest

from so101sim.config import SIM, servo_limits

SCRIPTS = sorted(p for d in ("scripts", "tools") for p in (SIM / d).glob("*.py"))


def run(*args, timeout=600):
    return subprocess.run([sys.executable, *map(str, args)], capture_output=True, text=True, timeout=timeout,
                          env={**__import__("os").environ, "SDL_VIDEODRIVER": "dummy"})


@pytest.mark.parametrize("script", SCRIPTS, ids=lambda p: p.name)
def test_help_works(script):
    r = run(script, "--help", timeout=60)
    assert r.returncode == 0, r.stderr[-500:]
    assert "usage" in r.stdout.lower()


@pytest.mark.parametrize("script", SCRIPTS, ids=lambda p: p.name)
@pytest.mark.parametrize("args", [["--no-such-flag"], ["--backend", "nope"]])
def test_bad_arguments_give_a_usage_error_not_a_traceback(script, args):
    r = run(script, *args, timeout=60)        # a script that ignores its arguments would start working: time out
    assert r.returncode != 0
    assert "Traceback" not in r.stderr, r.stderr[-800:]


def test_vision_check_without_arguments_explains_what_it_needs():
    r = run(SIM / "scripts" / "vision_check.py")
    assert r.returncode != 0 and "Traceback" not in r.stderr and "--backend" in r.stderr


def test_vision_check_mujoco(tmp_path):
    r = run(SIM / "scripts" / "vision_check.py", "--backend", "mujoco")
    assert r.returncode == 0, r.stdout[-800:] + r.stderr[-800:]


def test_teleop_selftest():
    r = run(SIM / "scripts" / "teleop.py", "--selftest")
    assert r.returncode == 0 and "FAIL" not in r.stdout, r.stdout[-1500:] + r.stderr[-800:]


@pytest.mark.parametrize("flags, lifted", [((), True), (("--no-grip-offset",), True), (("--no-feedforward",), False)])
def test_pick_demo(tmp_path, monkeypatch, flags, lifted):
    """The scripted pick, watched: with compensation it lifts the cube and the gripper never sits in overload."""
    import pick_demo
    from so101sim import load_backend as real_load
    from so101sim.servo_model import ServoSpec
    log = []

    def load(name, **kw):
        sim = real_load(name, **kw)
        step = sim.step

        def watched(n=1):
            step(n)
            log.append((sim.time, abs(float(sim.d.actuator_force[sim.act][5]))))
        sim.step = watched
        return sim

    monkeypatch.setattr(pick_demo, "load_backend", load)
    monkeypatch.setattr(sys, "argv", ["pick_demo.py", "--backend", "mujoco", "--no-render", "--out", str(tmp_path), *flags])
    assert (pick_demo.main() == 0) == lifted
    if "--no-grip-offset" in flags or not lifted:
        return
    spec, lim = ServoSpec.from_config(), servo_limits()
    overload = spec.overload_threshold * (lim["gripper_torque_nm"] / lim["stall_torque_nm"]) * spec.stall_torque
    t, f = np.array(log).T
    longest = run_len = 0.0
    for dt, above in zip(np.diff(t, prepend=t[0]), f > overload):
        run_len = run_len + dt if above else 0.0
        longest = max(longest, run_len)
    assert longest < 1.0, f"gripper above its overload threshold for {longest:.2f} s (the servo trips at 2 s)"
