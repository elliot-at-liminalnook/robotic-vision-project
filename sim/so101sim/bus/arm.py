"""Moved to hub/bus/arm.py (the hub runs on the real machine and may not import the simulation, so what the
controller needs lives there). This name still works: it is the same module."""
import sys

from hub.bus import arm as _moved

sys.modules[__name__] = _moved
