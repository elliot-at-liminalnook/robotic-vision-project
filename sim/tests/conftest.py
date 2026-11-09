"""Shared setup for the sim tests.

    pytest sim/tests            # MuJoCo backend; Newton tests run too where Newton is installed

These tests aim at classes of bugs, not single instances. The patterns:
  * invariants watched during every scenario (harness.Monitor), not just one
    end result: nothing commanded outside limits, no servo sitting in
    overload, the tip never under the table;
  * the same behaviour checked across every way of driving it (stepped,
    ramped, streamed commands; per-step and per-frame measurements);
  * seeded fuzzing over the whole joint range instead of one hand-picked move;
  * what the UI reports compared with what the physics does;
  * numbers that live in two places (config, generated model, CAD study,
    docs) compared with each other.
"""
import os
import sys
from pathlib import Path

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")     # pygame without a window

SIM = Path(__file__).resolve().parents[1]
ROOT = SIM.parent
for p in (SIM, SIM / "scripts", SIM / "tools", SIM / "tests", ROOT / "hardware" / "wrist_mount"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
