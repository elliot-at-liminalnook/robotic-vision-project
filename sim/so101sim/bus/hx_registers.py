"""HX-30HM register table V3.7, transcribed from
docs/datasheets/hiwonder-hx-servos/Hiwonder_HX-30HM_Register_List_and_Parameters.xlsx
(decimal address column). sim/tests/test_bus.py compares this file with the spreadsheet.

The layout matches the Feetech STS3215 table for every register both have.
What the HX table does NOT list (and STS has): angle limits (9-12), the
run-time torque limit (48), the goal-position copy (67). Units also differ:
current is in 1 mA (STS: 6.5 mA), speed in steps/s.

UNVERIFIED on a real servo. Two-byte values are low byte first. Signed values
are assumed to be sign-magnitude like Feetech's (the table names the sign bit
for registers 31, 44 and 60; bit 15 for 42, 56, 58 is the STS convention).
"""
from __future__ import annotations

from dataclasses import dataclass

from .protocol import from_sign_mag, sign_mag

SIZE = 128                      # bytes of register memory the emulator keeps


@dataclass(frozen=True)
class Register:
    addr: int
    name: str
    size: int = 1
    default: int = 0
    area: str = "NVS"           # ROM (fixed), NVS (kept over power-off unless locked), SRAM
    writable: bool = True
    lo: int = 0
    hi: int = 254
    sign_bit: int | None = None  # sign-magnitude: this bit is the sign

    def encode(self, value: int) -> bytes:
        raw = sign_mag(value, self.sign_bit) if self.sign_bit is not None else int(value)
        return bytes((raw >> (8 * i)) & 0xFF for i in range(self.size))

    def decode(self, data: bytes) -> int:
        raw = sum(b << (8 * i) for i, b in enumerate(data[:self.size]))
        return from_sign_mag(raw, self.sign_bit) if self.sign_bit is not None else raw


def _ro(addr, name, size=1, default=0, area="SRAM", sign_bit=None):
    return Register(addr, name, size, default, area, False, 0, 0, sign_bit)


REGISTERS = (
    _ro(0, "firmware_major", default=3, area="ROM"),
    _ro(1, "firmware_minor", default=7, area="ROM"),
    _ro(3, "servo_major", default=9, area="ROM"),
    _ro(4, "servo_minor", default=3, area="ROM"),
    Register(5, "id", default=1, hi=253),
    Register(6, "baud_rate", default=0, hi=7),
    Register(8, "response_level", default=1, hi=1),
    Register(13, "overtemperature_threshold", default=70, hi=120),
    Register(14, "overvoltage_threshold", default=80),
    Register(15, "undervoltage_threshold", default=40),
    Register(16, "max_torque_limit", 2, default=1000, hi=1000),
    Register(19, "protection_control", default=44),
    Register(20, "led_alarm_condition", default=47),
    Register(21, "position_p_gain", default=32),
    Register(22, "position_d_gain", default=32),
    Register(23, "position_i_gain", default=0),
    Register(24, "min_startup_torque", 2, default=16, hi=1000),
    Register(26, "cw_deadband", default=1, hi=32),
    Register(27, "ccw_deadband", default=1, hi=32),
    Register(28, "overcurrent_threshold", 2, default=6000, hi=6000),   # the sheet's "max 511" contradicts its default
    Register(31, "position_calibration", 2, default=0, lo=-2047, hi=2047, sign_bit=11),
    Register(33, "operating_mode", default=0, hi=2),
    Register(34, "overload_torque_limit", default=20, hi=100),
    Register(35, "overload_trigger_time", default=200),
    Register(36, "overload_torque_threshold", default=80, hi=100),
    Register(37, "speed_p_gain", default=10, hi=100),
    Register(38, "overcurrent_protection_time", default=200),
    Register(39, "speed_i_gain", default=10),
    Register(40, "torque_switch", area="SRAM", hi=128),
    Register(41, "acceleration", area="SRAM"),
    Register(42, "target_position", 2, area="SRAM", lo=-30719, hi=30719, sign_bit=15),
    Register(44, "pwm_open_loop_speed", 2, area="SRAM", lo=-1000, hi=1000, sign_bit=10),
    Register(46, "motion_speed", 2, area="SRAM", hi=3400),
    Register(55, "lock_flag", area="SRAM", hi=1),
    _ro(56, "current_position", 2, sign_bit=15),
    _ro(58, "current_speed", 2, sign_bit=15),
    _ro(60, "current_load", 2, sign_bit=10),
    _ro(62, "current_voltage"),
    _ro(63, "current_temperature"),
    _ro(64, "async_write_flag"),
    _ro(65, "servo_status"),
    _ro(66, "moving_flag"),
    _ro(69, "present_current", 2),
)

BY_NAME = {r.name: r for r in REGISTERS}
BY_ADDR = {r.addr: r for r in REGISTERS}
BYTE_OWNER = {r.addr + i: r for r in REGISTERS for i in range(r.size)}   # every documented byte -> its register

# Status / protection bits (registers 19, 20, 65).
BIT_VOLTAGE, BIT_SENSOR, BIT_TEMPERATURE, BIT_CURRENT, BIT_ANGLE, BIT_OVERLOAD = (1 << i for i in range(6))

NO_LOAD_SPEED_STEPS_S = 3400    # "Basic Parameters" sheet, at 11.1 V
