"""Moved to hub/bus/protocol.py (the hub runs on the real machine and may not import the simulation, so what the
controller needs lives there). This name still works: it is the same module."""
import sys

from hub.bus import protocol as _moved

sys.modules[__name__] = _moved
