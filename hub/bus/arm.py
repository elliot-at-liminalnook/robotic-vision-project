"""The arm as the controller sees it over the bus: joints in radians, servos in ticks.

Controller-side code (it knows nothing about the simulation). Start-up is in
three steps so that nothing is written before the servos are identified:

    arm.connect()        read only: every servo answers, and its registers look like the HX table
    arm.configure()      the few settings we rely on (written only where they differ)
    arm.enable(q)        goal := q, then torque on (both read back)

    arm.disable()        torque off (read back): the arm goes limp
"""
from __future__ import annotations

import numpy as np

from ..arm import JOINTS
from ..arm.config import load_servos
from ..arm.servo import ServoMap
from . import hx_registers as R
from .driver import BusError, FeetechBus
from .protocol import from_sign_mag, from_u16, sign_mag, u16

_POS, _GOAL, _TORQUE = R.BY_NAME["current_position"], R.BY_NAME["target_position"], R.BY_NAME["torque_switch"]
_FEEDBACK_LEN = R.BY_NAME["present_current"].addr + 2 - _POS.addr       # registers 56..70 in one read


class BusArm:
    def __init__(self, bus: FeetechBus, joints=JOINTS, arm: str = "follower"):
        self.bus = bus
        self.joints = tuple(joints)
        self.map = ServoMap(self.joints, arm)
        self.ids = self.map.ids
        self.cfg = load_servos()[arm]
        self.firmware: dict[str, tuple] = {}

    def _name(self, servo_id: int) -> str:
        return f"{self.joints[self.ids.index(servo_id)]} (id {servo_id})"

    # ---- start-up ----------------------------------------------------------
    def connect(self):
        """Find every servo and check its register layout. Reads only."""
        if len(set(self.ids)) != len(self.ids):
            raise BusError(f"servos.toml gives two joints the same servo id: {self.ids}")
        for i in self.ids:
            try:
                self.bus.ping(i)
            except BusError as e:
                raise BusError(f"no valid answer from {self._name(i)}. Check the cable, the supply, the baud rate, "
                               f"and that every servo has its own id (new servos all ship as id 1). [{e}]") from e
        for i in self.ids:
            fw, model = self.bus.read(i, 0, 2), self.bus.read(i, 3, 4)      # registers 0-1 and 3-6
            level = self.bus.read(i, 8, 1)[0]
            self.firmware[self._name(i)] = (fw[0], fw[1], model[0], model[1])
            checks = {
                "register 5 (id) equals the id that answered": model[2] == i,
                "register 6 (baud index) is 0..7": model[3] <= 7,
                "register 8 (response level) is 0 or 1": level <= 1,
                "register 16 (max torque limit) is 0..1000": from_u16(self.bus.read(i, 16, 2)) <= 1000,
                "register 33 (operating mode) is 0..2": self.bus.read(i, 33, 1)[0] <= 2,
            }
            failed = [what for what, ok in checks.items() if not ok]
            if failed:
                raise BusError(f"{self._name(i)} does not look like the HX-30HM register table, so nothing was "
                               f"written. Failed: {'; '.join(failed)}")
            if level != 1:
                raise BusError(f"{self._name(i)} has response level {level}: writes would not be acknowledged")

    def configure(self):
        """Position mode at full speed (the controller shapes the motion), and the gripper's torque limit."""
        for i in self.ids:
            self._ensure(i, "operating_mode", 0)
        grip = self.ids[self.joints.index("gripper")] if "gripper" in self.joints else None
        if grip is not None:
            self._ensure(grip, "max_torque_limit", int(round(float(self.cfg["gripper_torque_limit_frac"]) * 1000)))
        self.bus.sync_write(R.BY_NAME["acceleration"].addr, {i: b"\x00" for i in self.ids})
        self.bus.sync_write(R.BY_NAME["motion_speed"].addr, {i: u16(0) for i in self.ids})

    def _ensure(self, servo_id: int, name: str, value: int):
        """Write a register only if it differs (NVS registers wear), then read it back."""
        r = R.BY_NAME[name]
        if r.decode(self.bus.read(servo_id, r.addr, r.size)) == value:
            return
        self.bus.write(servo_id, r.addr, r.encode(value))
        back = r.decode(self.bus.read(servo_id, r.addr, r.size))
        if back != value:
            raise BusError(f"{self._name(servo_id)}: wrote {name} = {value} but it reads back {back}")

    def enable(self, q):
        """Hold pose q: goals first, then torque on, so the arm does not jump.

        Both are confirmed by reading them back: a servo that missed the goal but got the torque switch
        would drive to whatever goal it had before (0 after power-on)."""
        ticks = self.map.rad_to_ticks(q)
        self._write_all_checked("target_position", {i: int(t) for i, t in zip(self.ids, ticks)})
        self._write_all_checked("torque_switch", {i: 1 for i in self.ids})

    def disable(self):
        """Torque off on every servo: the arm goes limp. Raises BusError naming every servo that could not
        be confirmed off (the others are off)."""
        self._write_all_checked("torque_switch", {i: 0 for i in self.ids})

    def _write_all_checked(self, name: str, values: dict[int, int], tries: int = 3):
        """One broadcast write, then each servo's register is read back; a servo that does not hold the value
        is written singly (acknowledged) and read again. A broadcast has no reply, so on its own it
        confirms nothing. Every servo is attempted even when another fails."""
        r = R.BY_NAME[name]
        try:
            self.bus.sync_write(r.addr, {i: r.encode(v) for i, v in values.items()})
        except BusError:
            pass                                   # the single writes below still get their turn
        failed = []
        for i, v in values.items():
            for _ in range(tries):
                try:
                    if r.decode(self.bus.read(i, r.addr, r.size)) == v:
                        break
                    self.bus.write(i, r.addr, r.encode(v))
                except BusError:
                    continue
            else:
                failed.append(self._name(i))
        if failed:
            raise BusError(f"{name} = {sorted(set(values.values()))} could not be confirmed on {', '.join(failed)}")

    # ---- every control tick ------------------------------------------------
    def write_goals(self, q):
        ticks = self.map.rad_to_ticks(q)
        self.bus.sync_write(_GOAL.addr, {i: u16(sign_mag(int(t), 15)) for i, t in zip(self.ids, ticks)})

    def read_positions(self) -> np.ndarray:
        """Joint angles [rad] from the encoders (one tick = 0.088 deg)."""
        data = self.bus.sync_read(_POS.addr, 2, self.ids)
        return self.map.ticks_to_rad([from_sign_mag(from_u16(data[i]), 15) for i in self.ids])

    def read_feedback(self) -> dict[str, np.ndarray]:
        """Position [rad], speed [rad/s], load (motor duty, -1..1), voltage [V], temperature [C],
        status bits, moving flag and current [A] of every servo, in one sync read."""
        data = self.bus.sync_read(_POS.addr, _FEEDBACK_LEN, self.ids)
        rows = [data[i] for i in self.ids]
        k = 2 * np.pi / self.map.ticks_per_rev
        return {
            "position": self.map.ticks_to_rad([from_sign_mag(from_u16(d[0:2]), 15) for d in rows]),
            "speed": self.map.direction * k * np.array([from_sign_mag(from_u16(d[2:4]), 15) for d in rows], float),
            "load": self.map.direction * np.array([from_sign_mag(from_u16(d[4:6]), 10) for d in rows], float) / 1000,
            "voltage": np.array([d[6] for d in rows], float) / 10,
            "temperature": np.array([d[7] for d in rows], float),
            "status": np.array([d[9] for d in rows], int),
            "moving": np.array([d[10] for d in rows], bool),
            "current": np.array([from_u16(d[13:15]) for d in rows], float) / 1000,
        }
