"""The servo bus at byte level: the controller's driver and an emulated set of servos.

Controller side (also for the real arm):  protocol, driver.FeetechBus, arm.BusArm
Simulated devices:                        servo_emu.ServoEmulator, wire.VirtualBus
Joined to the physics in so101sim.bus_backend.BusBackend.
"""
from .arm import BusArm
from .driver import BusError, FeetechBus
from .servo_emu import ServoEmulator
from .wire import VirtualBus

__all__ = ["BusArm", "BusError", "FeetechBus", "ServoEmulator", "VirtualBus"]
