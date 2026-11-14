"""The MuJoCo backend with the servo bus in the loop.

    controller  --bytes-->  simulated wire  -->  6 emulated HX-30HM  --torque-->  MuJoCo
    (teleop, pick_demo)  <--bytes--                (registers + control loop)  <--angle, speed--

Same API as the other backends, so the controller code doesn't change:
  q()            a SYNC_READ of the encoders (quantised to ticks; costs bus time)
  set_targets()  a SYNC_WRITE of goal positions (arrives a little later)
  step(n)        wait for the next n control ticks

Time: there is one clock. Bus traffic takes time on it (bytes at the baud rate,
servo response delay, USB latency), and the physics steps whenever the clock
passes a step boundary. step() keeps an absolute schedule like a fixed-rate
loop that sleeps until its next tick: time spent on the bus comes out of the
sleep, and `overruns` counts the ticks where the bus took longer than that.
Everything runs in one thread and is deterministic.

The model's position actuators become torque motors; each emulated servo
computes its torque from its registers (stiffness from the same
servos.toml value the model generator uses, torque limit from register 16).
"""
from __future__ import annotations

import mujoco
import numpy as np

from . import JOINTS
from .bus import BusArm, BusError, FeetechBus, ServoEmulator, VirtualBus
from .config import load_servos, position_loop
from .safety import CommandFilter
from .servo import ServoMap
from .servo_model import ServoSpec, zero_joint_damping

_WIRE_KEYS = ("baud", "response_delay_s", "usb_tx_latency_s", "usb_rx_latency_s", "echo",
              "drop_command_prob", "drop_reply_prob", "corrupt_prob", "seed")


class SimPort:
    """The controller's serial port, backed by the simulated wire. A blocking read advances the clock."""

    def __init__(self, world: "BusBackend"):
        self.world = world

    def write(self, data: bytes):
        self.world.wire.host_write(data, self.world.t)

    def read(self, n: int, timeout_s: float) -> bytes:
        w = self.world
        deadline = w.t + timeout_s
        while w.wire.available(w.t) < n:
            nxt = w.wire.next_event_time(w.t)
            if nxt is None or nxt > deadline:
                w.advance_to(deadline)
                break
            w.advance_to(nxt)
        return w.wire.host_read(n, w.t)

    def reset_input(self):
        self.world.wire.host_discard(self.world.t)


class BusBackend:
    name = "mujoco+bus"

    def __init__(self, inner, servos: list[ServoEmulator] | None = None, **overrides):
        """inner: a MujocoBackend. overrides: [bus] settings from servos.toml (e.g. drop_reply_prob=0.05).

        servos: emulators to put on the bus instead of the six configured ones
        (to simulate a wiring or set-up mistake).
        """
        if getattr(inner, "name", None) != "mujoco":
            raise ValueError("the bus-in-the-loop backend needs the MuJoCo backend")
        self.inner = inner
        inner.safety = None                              # the filter sits on the controller's side of the bus here
        self.safety = CommandFilter()
        self.cfg = {**load_servos().get("bus", {}), **overrides}
        m = inner.m
        loop = position_loop()
        # The emulated servo's loop matches the generated model's actuator: stiffness from sat_error_deg, no D term.
        self.spec = ServoSpec.from_config(d_time_s=0.0 if float(loop["actuator_kv"]) == 0.0 else ServoSpec.d_time_s)
        for a in inner.act:                              # position actuators -> torque motors (ctrl = N.m)
            m.actuator_gaintype[a] = mujoco.mjtGain.mjGAIN_FIXED
            m.actuator_gainprm[a, :] = 0
            m.actuator_gainprm[a, 0] = 1.0
            m.actuator_biastype[a] = mujoco.mjtBias.mjBIAS_NONE
            m.actuator_biasprm[a, :] = 0
            m.actuator_ctrllimited[a] = 0                # ctrlrange keeps the joint range (tests read it)
        zero_joint_damping(m, JOINTS)                    # back-EMF is in the servo's torque-speed line now
        self.dof = np.array([m.joint(n).dofadr[0] for n in JOINTS])
        smap = ServoMap(JOINTS)
        self.servos = servos if servos is not None else [
            ServoEmulator(i, self.spec, mount_zero_ticks=z, mount_direction=int(s))
            for i, z, s in zip(smap.ids, smap.zero, smap.direction)]
        self.wire = VirtualBus(self.servos, **{k: self.cfg[k] for k in _WIRE_KEYS if k in self.cfg})
        self.port = SimPort(self)
        self.driver = FeetechBus(self.port, reply_timeout_s=float(self.cfg.get("reply_timeout_s", 0.02)),
                                 retries=int(self.cfg.get("retries", 1)), echo=bool(self.cfg.get("echo", False)))
        self.arm = BusArm(self.driver)
        self.overruns = 0
        self.max_late_s = 0.0
        self.read_failures = 0
        self.reset()

    def __getattr__(self, name):                         # m, d, act, qadr, cameras, render, body_pos ...
        return getattr(self.inner, name)

    # ---- the clock ---------------------------------------------------------
    def advance_to(self, t: float):
        """Run the wire, the servos and the physics up to time t."""
        dt = self.inner.dt
        while not self._bench and self.inner.time + dt <= t + 1e-9:
            self.wire.process(self.inner.time)
            self._servo_step(dt)
            self.inner.step(1)
        self.wire.process(t)
        self.t = max(self.t, t)

    def _servo_step(self, dt: float):
        d, m = self.inner.d, self.inner.m
        q, qd = d.qpos[self.inner.qadr], d.qvel[self.dof]
        for k, s in enumerate(self.servos[:len(JOINTS)]):
            a = self.inner.act[k]
            d.ctrl[a] = s.control(float(q[k]), float(qd[k]), dt)
            m.actuator_forcerange[a] = (-s.torque_limit_nm, s.torque_limit_nm)

    # ---- backend API -------------------------------------------------------
    @property
    def dt(self) -> float:
        return self.inner.dt

    @property
    def time(self) -> float:
        return self.t

    def reset(self, q=None):
        """Power-cycle the servos, connect, configure, and hold pose q. Physics time starts at 0 after that."""
        self.inner.reset(q)
        self.inner.d.ctrl[self.inner.act] = 0.0
        for s in self.servos:
            s.power_cycle()
        self.wire.reset()
        self.wire.now = -1.0
        self._bench, self.t = True, -1.0                 # start-up traffic happens before the physics runs
        self._servo_step(self.inner.dt)                  # the servos read their encoders
        try:
            self.arm.connect()
            self.arm.configure()
            self.arm.enable(self.inner.q())
            first = self.arm.read_positions()            # so a failed read always has an earlier one to repeat
            while not self.wire.idle():                  # let the last writes reach the servos
                self.advance_to(self.wire.next_event_time(self.t))
        finally:
            self._bench, self.t = False, 0.0
        self.wire.now = 0.0
        self._tick = 0.0
        self._q, self._q_time = first, None
        self._request = self.inner.q()
        self._intended = None
        self._sent = self._request.copy()                # enable() wrote this pose as the goal
        self.safety.reset(self._request)
        self.q_fresh = True                              # False while q() is repeating an old reading
        self.torque_enabled = True
        mujoco.mj_forward(self.inner.m, self.inner.d)

    # ---- torque switch -------------------------------------------------------
    def torque_off(self):
        """Torque off on every servo: the arm goes limp. From this call on no goal is written, whether or
        not the servos could be reached (a goal write may switch a real servo's torque back on).
        Raises BusError if a servo could not be confirmed off."""
        self.torque_enabled = False
        self.arm.disable()

    def torque_on(self) -> np.ndarray:
        """Torque back on, holding the pose the encoders report now. Returns that pose: the caller's
        command must start again from it. Raises BusError (and stays off) without a fresh reading."""
        measured = self.arm.read_positions()
        self.safety.reset(measured)
        q = self.safety.q.copy()
        self.arm.enable(q)
        self._request, self._intended, self._sent = q.copy(), None, q.copy()
        self._q, self._q_time, self.q_fresh = measured, None, True
        self.torque_enabled = True
        return q

    def q(self) -> np.ndarray:
        """Encoder positions over the bus. Read at most once per instant; a failed read returns the last good one."""
        if self._q is None or self._q_time != self.t:
            try:
                self._q = self.arm.read_positions()
                self.q_fresh = True
            except BusError:
                self.read_failures += 1
                self.q_fresh = False
                if self._q is None:
                    raise
            self._q_time = self.t
        return self._q.copy()

    def true_q(self) -> np.ndarray:
        """The joint angles in the physics (not what the encoders report)."""
        return self.inner.q()

    def set_targets(self, q, intended=None):
        """Ask for joint positions. Each control tick, the command filter's output goes out as a SYNC_WRITE.
        intended: as in MujocoBackend.set_targets."""
        self._request = np.array(q, float)
        self._intended = None if intended is None else np.array(intended, float)

    def step(self, n: int = 1):
        for _ in range(n):
            if self.torque_enabled:
                cmd = self.safety.update(self._request, self.inner.dt, self._intended, measured=self._q)   # the last encoder reading
                if not np.array_equal(cmd, self._sent):      # nothing new to say: stay off the bus
                    self.arm.write_goals(cmd)
                    self._sent = cmd
            self._tick += self.inner.dt
            late = self.t - self._tick
            if late > 1e-9:
                self.overruns += 1
                self.max_late_s = max(self.max_late_s, late)
            else:
                self.advance_to(self._tick)

    # ---- debugging ---------------------------------------------------------
    def report(self, trace: int = 0) -> str:
        w, st = self.wire, self.driver.stats
        used = (w.bytes_tx + w.bytes_rx) * w.byte_time / self.t if self.t > 0 else 0.0
        lines = [
            f"servo bus after {self.t:.2f} s: {w.bytes_tx} bytes sent, {w.bytes_rx} received "
            f"({used * 100:.0f} % of the wire's capacity at {w.baud} baud)",
            f"  packets sent {st['tx_packets']}, replies {st['rx_packets']}, retries {st['retries']}, "
            f"timeouts {st['timeouts']}, checksum errors {st['checksum_errors']}, failed {st['failures']}",
            f"  control ticks that overran: {self.overruns} (worst {self.max_late_s * 1000:.2f} ms late); "
            f"position reads that failed: {self.read_failures}",
            f"  servos in overload protection: {[s.id for s in self.servos if s.tripped] or 'none'}; "
            f"with torque on: {[s.id for s in self.servos if s.get("torque_switch") == 1] or 'none'}",
        ]
        notes = {}
        for _, kind, msg in w.events:
            notes[(kind, msg)] = notes.get((kind, msg), 0) + 1
        for (kind, msg), count in notes.items():
            lines.append(f"  [{kind}] {msg}" + (f" (x{count})" if count > 1 else ""))
        if trace:
            lines += ["  last frames on the wire:", w.format_trace(trace)]
        return "\n".join(lines)
