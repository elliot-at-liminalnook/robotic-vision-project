"""One emulated HX-30HM: register memory, packet handling and the control loop.

This is OUR stand-in for the servo's closed firmware, built from the register
table (hx_registers.py), the protocol manual and the motor model in
so101sim.servo_model. It is not Hiwonder's code. Where the documents don't say
what the firmware does, the choice made here is listed below, and the emulator
logs an event whenever the controller relies on one of them, so the bus trace
shows it.

ASSUMPTIONS (check each on a real servo):
  * A write outside a register's range is clamped. A write to a read-only or
    undocumented address is ignored. Both are logged.
  * Writing a goal position does NOT switch torque on (Feetech STS firmware
    does). Strict on purpose: a controller that forgets to enable torque fails
    here rather than on the bench.
  * Motion speed 0 and acceleration 0 mean "no limit".
  * P gain scales the loop stiffness linearly from its default (32 = the
    configured sat_error_deg); D likewise scales d_time_s. I is not modelled.
  * Overload protection: so101sim.servo_model.ServoBank (80 % of the torque
    limit for 2 s caps the output at 20 %; released when the demand drops).
  * Voltage, temperature, current and angle protections are not modelled;
    temperature reads a constant.
  * Torque off: the motor coasts (no output and no back-EMF braking); only the
    joint's friction is left. If the real servo shorts the motor instead, a
    limp arm sags more slowly than here.
  * Operating mode 1 ("constant-speed motor mode"): the shaft is driven at the speed in register 46
    (steps/s, bit 15 = direction) by a speed loop with proportional and integral terms scaled by
    registers 37 and 39 (their units are not documented: at the defaults the loop settles in about
    50 ms with no steady error). The present position then reads the angle within one turn
    (0..4095): the turn count is not kept. Mode 2 (open-loop PWM) is not modelled.
  * Position calibration (register 31) is added to the encoder reading;
    torque switch = 128 sets it so the present position reads 2048.
"""
from __future__ import annotations

import math

import numpy as np

from ..servo_model import ServoBank, ServoSpec
from . import hx_registers as R
from . import protocol as P

TICKS_PER_REV = 4096
RAD_PER_TICK = 2 * math.pi / TICKS_PER_REV
DEFAULT_P = R.BY_NAME["position_p_gain"].default
DEFAULT_D = R.BY_NAME["position_d_gain"].default
MOVING_STEPS_S = 20             # |speed| above this sets the moving flag
TEMPERATURE_C = 30
GOAL_NOTE_TICKS = 20            # a goal this far from the shaft, written with torque off, is worth a note
UNMODELLED = {"overtemperature_threshold", "overvoltage_threshold", "undervoltage_threshold", "led_alarm_condition",
              "position_i_gain", "min_startup_torque", "overcurrent_threshold", "overcurrent_protection_time",
              "pwm_open_loop_speed"}


class ServoEmulator:
    def __init__(self, servo_id: int, spec: ServoSpec | None = None, *, mount_zero_ticks: float = 2048,
                 mount_direction: int = 1, log=None):
        """mount_*: how the shaft sits on the joint (ticks at joint angle 0, and the sense of rotation)."""
        self.base = spec or ServoSpec.from_config()
        self.mount_zero = float(mount_zero_ticks)
        self.mount_dir = 1 if mount_direction >= 0 else -1
        self.log = log or (lambda kind, msg: None)
        self.nvs = {r.addr: r.default for r in R.REGISTERS if r.area == "NVS"}
        self.nvs[R.BY_NAME["id"].addr] = int(servo_id)
        self._noted: set[int] = set()
        self.power_cycle()

    # ---- register access ---------------------------------------------------
    def get(self, name: str) -> int:
        r = R.BY_NAME[name]
        return r.decode(self.mem[r.addr:r.addr + r.size])

    def _set(self, name: str, value: int):
        r = R.BY_NAME[name]
        self.mem[r.addr:r.addr + r.size] = r.encode(value)

    @property
    def id(self) -> int:
        return self.mem[R.BY_NAME["id"].addr]

    @property
    def baud(self) -> int:
        return P.BAUD_RATES[self.mem[R.BY_NAME["baud_rate"].addr]]

    @property
    def tripped(self) -> bool:
        return bool(self.bank.tripped[0])

    @property
    def torque_limit_nm(self) -> float:
        return self.spec.torque_limit * self.spec.tau_s

    def power_cycle(self):
        """Power off and on: SRAM back to defaults, NVS as last saved."""
        self.mem = bytearray(R.SIZE)
        for r in R.REGISTERS:
            self.mem[r.addr:r.addr + r.size] = r.encode(self.nvs.get(r.addr, r.default))
        self.pending: tuple[int, bytes] | None = None       # REG_WRITE waiting for ACTION
        self.setpoint = None                                # profile position [ticks]; None until torque is on
        self.setpoint_vel = 0.0
        self.speed_integral = 0.0
        self.torque = 0.0
        self._apply_registers()
        self.bank = ServoBank(1, self.spec)

    def _apply_registers(self):
        """Control-loop parameters from the register values."""
        b = self.base
        p, d = self.get("position_p_gain"), self.get("position_d_gain")
        overload_on = bool(self.get("protection_control") & R.BIT_OVERLOAD)
        self.spec = ServoSpec(
            stall_torque=b.stall_torque, no_load_speed=b.no_load_speed, stall_current=b.stall_current,
            no_load_current=b.no_load_current, supply_v=b.supply_v, rated_v=b.rated_v,
            torque_limit=self.get("max_torque_limit") / 1000,
            overload_threshold=self.get("overload_torque_threshold") / 100,
            overload_time=self.get("overload_trigger_time") * 0.01 if overload_on else math.inf,
            overload_cap=self.get("overload_torque_limit") / 100,
            sat_error_deg=b.sat_error_deg * DEFAULT_P / p if p else math.inf,
            deadband_deg=self.get("cw_deadband") * 360 / TICKS_PER_REV,
            d_time_s=b.d_time_s * d / DEFAULT_D)
        if hasattr(self, "bank"):
            self.bank.spec = self.spec

    def _write(self, addr: int, data: bytes):
        touched, before = [], bytes(self.mem)
        for i, byte in enumerate(data):
            r = R.BYTE_OWNER.get(addr + i)
            if r is None:
                self.log("warn", f"servo {self.id}: write to undocumented register {addr + i} ignored")
            elif not r.writable:
                self.log("warn", f"servo {self.id}: write to read-only register {r.addr} ({r.name}) ignored")
            else:
                self.mem[addr + i] = byte
                if r not in touched:
                    touched.append(r)
        for r in touched:
            value = r.decode(self.mem[r.addr:r.addr + r.size])
            clamped = min(max(value, r.lo), r.hi)
            if r.name == "motion_speed":            # bit 15 is the direction in motor mode: only the speed itself is limited
                clamped = (value & 0x8000) | min(value & 0x7FFF, r.hi)
            if clamped != value:
                self.log("warn", f"servo {self.id}: {r.name} = {value} is outside {r.lo}..{r.hi}, clamped")
                self.mem[r.addr:r.addr + r.size] = r.encode(clamped)
            if r.name in UNMODELLED and self.mem[r.addr:r.addr + r.size] != before[r.addr:r.addr + r.size]:
                self.log("info", f"servo {self.id}: {r.name} changed, but the emulator does not model it")
            self._side_effects(r, before)
        for r in touched:                       # after side effects (the lock flag may be in the same write)
            if r.area == "NVS" and not self.get("lock_flag"):
                self.nvs[r.addr] = r.decode(self.mem[r.addr:r.addr + r.size])
        self._apply_registers()

    def _side_effects(self, r: R.Register, before: bytes):
        if r.name == "torque_switch":
            was, now = before[r.addr], self.mem[r.addr]
            if now == 128:                      # calibrate: the present position becomes 2048
                raw = self.get("current_position") - self.get("position_calibration")
                self._set("position_calibration", 2048 - raw)
                if not self.get("lock_flag"):
                    self.nvs[R.BY_NAME["position_calibration"].addr] = self.get("position_calibration")
                self.mem[r.addr] = was
            elif now not in (0, 1):
                self.log("warn", f"servo {self.id}: torque switch = {now} is not a documented value; treated as off")
            elif now == 1 and was != 1:
                self.setpoint = None            # start the profile from where the shaft is
        elif (r.name == "target_position" and self.mem[R.BY_NAME["torque_switch"].addr] != 1
              and abs(self.get("target_position") - self.get("current_position")) > GOAL_NOTE_TICKS):
            self.log("info", f"servo {self.id}: goal far from the present position written while torque is off "
                             "(no motion here; Feetech STS firmware would switch torque on and move, HX unverified)")
        elif r.name == "operating_mode":
            self.speed_integral = 0.0
            if self.mem[r.addr] not in (0, 1):
                self.log("warn", f"servo {self.id}: operating mode {self.mem[r.addr]} is not modelled; output is off")

    # ---- packets -----------------------------------------------------------
    def receive(self, pkt: P.Packet) -> tuple[bytes, int] | None:
        """Handle an instruction packet. Returns (status packet, reply slot) or None for no reply.

        The slot is this servo's position in a SYNC_READ id list (0 otherwise):
        the bus uses it to order the replies in time.
        """
        me, broadcast = self.id, pkt.id == P.BROADCAST
        if pkt.id != me and not broadcast:
            return None
        p = pkt.params
        ack = None if broadcast or not self.get("response_level") else (self._status(), 0)
        if pkt.code == P.PING:
            return self._status(), 0
        if pkt.code == P.READ and len(p) == 2:
            if broadcast:
                return None
            if p[0] + p[1] > R.SIZE:
                self.log("warn", f"servo {me}: read of {p[1]} bytes at {p[0]} runs past the register memory")
                return None
            return self._status(self._read(p[0], p[1])), 0
        if pkt.code == P.WRITE and len(p) >= 2:
            self._write(p[0], p[1:])
            return ack
        if pkt.code == P.REG_WRITE and len(p) >= 2:
            self.pending = (p[0], bytes(p[1:]))
            self.mem[R.BY_NAME["async_write_flag"].addr] = 1
            return ack
        if pkt.code == P.ACTION:
            if self.pending:
                self._write(*self.pending)
            self.pending = None
            self.mem[R.BY_NAME["async_write_flag"].addr] = 0
            return ack
        if pkt.code == P.RESET:
            return ack
        if pkt.code == P.SYNC_WRITE and broadcast and len(p) >= 2:
            addr, n = p[0], p[1]
            if n == 0 or (len(p) - 2) % (n + 1):
                self.log("warn", f"servo {me}: malformed SYNC_WRITE ({len(p) - 2} data bytes for length {n})")
                return None
            for k in range(2, len(p), n + 1):
                if p[k] == me:
                    self._write(addr, p[k + 1:k + 1 + n])
            return None
        if pkt.code == P.SYNC_READ and broadcast and len(p) >= 2:
            addr, n, ids = p[0], p[1], list(p[2:])
            if me not in ids or addr + n > R.SIZE:
                return None
            return self._status(self._read(addr, n)), ids.index(me)
        self.log("warn", f"servo {me}: instruction 0x{pkt.code:02X} with {len(p)} parameter bytes not understood")
        return None

    def _read(self, addr: int, n: int) -> bytes:
        for a in range(addr, addr + n):
            if a not in R.BYTE_OWNER and a not in self._noted:
                self._noted.add(a)
                self.log("info", f"servo {self.id}: read of undocumented register {a} (returns 0 here)")
                break
        return bytes(self.mem[addr:addr + n])

    def _status(self, params: bytes = b"") -> bytes:
        return P.encode(self.id, self.mem[R.BY_NAME["servo_status"].addr], params)

    # ---- control loop (once per physics step) ------------------------------
    def control(self, q_joint: float, qd_joint: float, dt: float) -> float:
        """Joint position/velocity in -> joint torque out; also refreshes the feedback registers."""
        calib = self.get("position_calibration")
        ticks = int(round(self.mount_zero + self.mount_dir * q_joint / RAD_PER_TICK)) + calib   # the encoder reading
        speed = self.mount_dir * qd_joint                                                       # shaft [rad/s]
        mode = self.get("operating_mode")
        on = self.mem[R.BY_NAME["torque_switch"].addr] == 1 and mode in (0, 1)
        tau = 0.0
        if on and mode == 1:                                    # motor mode: a speed loop
            raw = self.get("motion_speed")
            want = (-1 if raw & 0x8000 else 1) * (raw & 0x7FFF) * RAD_PER_TICK          # rad/s at the shaft
            err = (want - speed) / self.spec.w0
            self.speed_integral = float(np.clip(self.speed_integral + err * dt, -0.05, 0.05))
            kp, ki = 2.0 * self.get("speed_p_gain") / 10.0, 40.0 * self.get("speed_i_gain") / 10.0
            volt = float(np.clip(kp * err + ki * self.speed_integral, -1.0, 1.0))
            tau = float(self.bank.torque_from_voltage([volt], [speed], dt)[0])
            self.setpoint = None
            ticks = ticks % TICKS_PER_REV                       # within one turn: the count of turns is not kept
        elif on:
            sp = self._profile(ticks, dt)
            if math.isinf(self.spec.sat_error_deg):             # P gain 0: no output
                tau = 0.0
            else:
                tau = float(self.bank.torque([sp * RAD_PER_TICK], [ticks * RAD_PER_TICK], [speed], dt)[0])
        else:
            self.setpoint = None
            self.bank.over_timer[:] = 0
            self.bank.tripped[:] = False
            self.bank.prev_target = None
            self.bank.last_current[:] = 0
        s = self.spec
        duty = np.clip((tau + s.tau_s / s.w0 * speed) / s.tau_s, -1, 1) if on else 0.0
        self._set("current_position", ticks)
        self._set("current_speed", int(round(speed / RAD_PER_TICK)))
        self._set("current_load", int(round(duty * 1000)))
        self._set("current_voltage", int(round(s.supply_v * 10)))
        self._set("current_temperature", TEMPERATURE_C)
        self._set("moving_flag", int(on and abs(speed / RAD_PER_TICK) > MOVING_STEPS_S))
        self._set("present_current", int(round(float(self.bank.last_current[0]) * 1000)))
        status = self.mem[R.BY_NAME["servo_status"].addr] & ~R.BIT_OVERLOAD
        self.mem[R.BY_NAME["servo_status"].addr] = status | (R.BIT_OVERLOAD if self.tripped else 0)
        self.torque = self.mount_dir * tau
        return self.torque

    def _profile(self, ticks: int, dt: float) -> float:
        """The firmware's own motion profile toward the goal (speed and acceleration registers)."""
        goal = float(self.get("target_position"))
        v_max = float(self.get("motion_speed")) or math.inf
        a_max = self.get("acceleration") * 100.0 or math.inf
        if self.setpoint is None:
            self.setpoint, self.setpoint_vel = float(ticks), 0.0
        if math.isinf(v_max) and math.isinf(a_max):
            self.setpoint, self.setpoint_vel = goal, 0.0
            return goal
        err = goal - self.setpoint
        v_des = math.copysign(min(v_max, math.sqrt(2 * a_max * abs(err)) if math.isfinite(a_max) else v_max), err)
        dv = v_des - self.setpoint_vel
        if math.isfinite(a_max):
            dv = min(max(dv, -a_max * dt), a_max * dt)
        self.setpoint_vel += dv
        step = self.setpoint_vel * dt
        if abs(step) >= abs(err):
            step, self.setpoint_vel = err, 0.0
        self.setpoint += step
        return self.setpoint
