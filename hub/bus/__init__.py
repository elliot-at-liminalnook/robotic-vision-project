"""The servo bus as the controller sees it: packets, the register table, the driver, the arm on the bus.

    port (bytes)  ->  FeetechBus (checked transactions)  ->  BusArm (joints in radians)

A port is anything with write(data), read(n, timeout_s) and reset_input(): a serial port on the
real machine, the simulated wire in the simulation (hub/ports.py).
"""
from .arm import BusArm
from .driver import BusError, FeetechBus

__all__ = ["BusArm", "BusError", "FeetechBus"]
