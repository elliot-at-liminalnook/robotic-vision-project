"""Moved to hub/arm/motion.py (the hub runs on the real machine and may not import the simulation, so what the
controller needs lives there). This name still works: it is the same module."""
import sys

from hub.arm import motion as _moved

sys.modules[__name__] = _moved
