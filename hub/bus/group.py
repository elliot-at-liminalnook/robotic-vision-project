"""Any set of servos on one bus, by id: identify them, set their modes, read them, command them.

BusArm (arm.py) is the arm's six in radians; this is the general case the rover uses: servos in
position mode and in motor mode side by side, each with its own zero and direction.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from . import hx_registers as R
from .driver import BusError, FeetechBus
from .protocol import from_sign_mag, from_u16, u16

TICKS = 4096
RAD_PER_TICK = 2 * math.pi / TICKS
_POS = R.BY_NAME["current_position"]
_FEEDBACK_LEN = R.BY_NAME["present_current"].addr + 2 - _POS.addr


@dataclass(frozen=True)
class Servo:
    id: int
    name: str
    motor: bool = False            # operating mode 1 (a wheel) rather than 0 (a position)
    zero_ticks: int = 2048
    direction: int = 1


class ServoGroup:
    def __init__(self, bus: FeetechBus, servos: list[Servo]):
        self.bus, self.servos = bus, list(servos)
        self.ids = [s.id for s in servos]
        self.by_id = {s.id: s for s in servos}

    def _name(self, i: int) -> str:
        return f"{self.by_id[i].name} (id {i})"

    # ---- start-up --------------------------------------------------------------
    def connect(self):
        """Every servo answers and its registers look like the HX table. Reads only."""
        for i in self.ids:
            try:
                self.bus.ping(i)
            except BusError as e:
                raise BusError(f"no valid answer from {self._name(i)} [{e}]") from e
        for i in self.ids:
            model = self.bus.read(i, 3, 4)
            level = self.bus.read(i, 8, 1)[0]
            ok = (model[2] == i and model[3] <= 7 and level == 1 and from_u16(self.bus.read(i, 16, 2)) <= 1000
                  and self.bus.read(i, 33, 1)[0] <= 2)
            if not ok:
                raise BusError(f"{self._name(i)} does not look like the HX-30HM register table, so nothing was written")

    def configure(self):
        """Each servo in its mode, stopped, with no speed or acceleration limit of its own."""
        for s in self.servos:
            self._ensure(s.id, "operating_mode", 1 if s.motor else 0)
        self.bus.sync_write(R.BY_NAME["acceleration"].addr, {i: b"\x00" for i in self.ids})
        self.bus.sync_write(R.BY_NAME["motion_speed"].addr, {i: u16(0) for i in self.ids})

    def _ensure(self, i: int, name: str, value: int):
        r = R.BY_NAME[name]
        if r.decode(self.bus.read(i, r.addr, r.size)) == value:
            return
        self.bus.write(i, r.addr, r.encode(value))
        if r.decode(self.bus.read(i, r.addr, r.size)) != value:
            raise BusError(f"{self._name(i)}: wrote {name} = {value} but it does not read back")

    def write_checked(self, name: str, values: dict[int, int], tries: int = 3):
        """One broadcast write, then each servo's register read back and, where it differs, written singly."""
        r = R.BY_NAME[name]
        try:
            self.bus.sync_write(r.addr, {i: r.encode(v) for i, v in values.items()})
        except BusError:
            pass
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
            raise BusError(f"{name} could not be confirmed on {', '.join(failed)}")

    def torque(self, on: bool, ids=None):
        self.write_checked("torque_switch", {i: int(on) for i in (ids or self.ids)})

    # ---- every tick --------------------------------------------------------------
    def read(self, ids=None) -> dict[int, dict]:
        """Each servo's feedback: ticks (as read), angle [rad at the shaft, from its zero], speed [rad/s], load (-1..1),
        volts, temperature [C], status bits, amps."""
        ids = list(ids or self.ids)
        data = self.bus.sync_read(_POS.addr, _FEEDBACK_LEN, ids)
        out = {}
        for i in ids:
            d, s = data[i], self.by_id[i]
            ticks = from_sign_mag(from_u16(d[0:2]), 15)
            out[i] = {"ticks": ticks, "angle": s.direction * (ticks - s.zero_ticks) * RAD_PER_TICK,
                      "speed": s.direction * from_sign_mag(from_u16(d[2:4]), 15) * RAD_PER_TICK,
                      "load": s.direction * from_sign_mag(from_u16(d[4:6]), 10) / 1000.0,
                      "volts": d[6] / 10.0, "temperature": float(d[7]), "status": int(d[9]), "amps": from_u16(d[13:15]) / 1000.0}
        return out

    def write_angles(self, angles: dict[int, float]):
        """Position goals [rad at the shaft, from each servo's zero]. A broadcast: not acknowledged."""
        goal = R.BY_NAME["target_position"]
        self.bus.sync_write(goal.addr, {i: goal.encode(int(round(self.by_id[i].zero_ticks + self.by_id[i].direction * a / RAD_PER_TICK)))
                                        for i, a in angles.items()})

    def write_speeds(self, speeds: dict[int, float]):
        """Motor-mode speeds [rad/s at the shaft]. Bit 15 is the direction. A broadcast: not acknowledged."""
        out = {}
        for i, w in speeds.items():
            steps = int(round(self.by_id[i].direction * w / RAD_PER_TICK))
            out[i] = u16((0x8000 if steps < 0 else 0) | min(abs(steps), 3400))
        self.bus.sync_write(R.BY_NAME["motion_speed"].addr, out)
