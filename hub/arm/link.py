"""The arm on its servo bus, as the arm's controller wants it: joint angles in, joint angles out.

    link = ArmLink(port, clock)
    q = link.connect()             # identify the servos, then hold the pose the arm is in
    link.set_targets(q_cmd, intended=q_pose); link.step(1)      # every control tick
    link.q()                       # encoders

Every command goes through the command filter (safety.py) on its way to the bus. One control tick
is `dt` on the clock: step() sleeps until the next tick is due, and counts the ticks where the bus
took longer than that. While torque is off no goal is written (a goal write may switch a real
servo's torque back on).
"""
from __future__ import annotations

import numpy as np

from ..bus import BusArm, BusError, FeetechBus
from .config import load_servos
from .safety import CommandFilter

RESYNC_S = 0.02                # this far behind schedule, the schedule starts again from now
GONE_AFTER = 8                 # this many position reads failing in a row: the arm is not answering any more


class ArmLink:
    name = "servo bus"

    def __init__(self, port, clock, tick_s: float = 0.002):
        cfg = load_servos().get("bus", {})
        self.driver = FeetechBus(port, reply_timeout_s=float(cfg.get("reply_timeout_s", 0.02)),
                                 retries=int(cfg.get("retries", 1)), echo=bool(cfg.get("echo", False)))
        self.arm = BusArm(self.driver)
        self.clock, self.dt = clock, float(tick_s)
        self.safety = CommandFilter()
        self.overruns, self.max_late_s, self.read_failures, self.resyncs = 0, 0.0, 0, 0
        self.torque_enabled = False
        self.q_fresh = False
        self._q = None
        self._failed_in_a_row = 0

    @property
    def time(self) -> float:
        return float(self.clock.now())

    def connect(self) -> np.ndarray:
        """Identify the servos (reads only), set the few registers we rely on, and hold the pose the arm is in.
        Returns that pose. Raises BusError if a servo is missing or does not look like the register table."""
        self.arm.connect()
        self.arm.configure()
        measured = self.arm.read_positions()
        self.safety.reset(measured)
        hold = self.safety.q.copy()
        self.arm.enable(hold)
        self._request, self._intended, self._sent = hold.copy(), None, hold.copy()
        self._q, self._q_time, self.q_fresh = measured, None, True
        self._tick = self.clock.now()
        self.torque_enabled = True
        return hold

    def q(self) -> np.ndarray:
        """Encoder positions. Read at most once per instant; a failed read returns the last good one."""
        now = self.clock.now()
        if self._q is None or self._q_time != now:
            try:
                self._q = self.arm.read_positions()
                self.q_fresh = True
                self._failed_in_a_row = 0
            except BusError as e:
                self.read_failures += 1
                self._failed_in_a_row += 1
                self.q_fresh = False
                if self._q is None:
                    raise
                if self._failed_in_a_row >= GONE_AFTER:           # one lost reply is noise; this many is a dead bus
                    raise BusError(f"the arm has not answered {self._failed_in_a_row} position reads in a row [{e}]") from e
            self._q_time = self.clock.now()
        return self._q.copy()

    def set_targets(self, q, intended=None):
        self._request = np.array(q, float)
        self._intended = None if intended is None else np.array(intended, float)

    def step(self, n: int = 1):
        for _ in range(n):
            if self.torque_enabled:
                cmd = self.safety.update(self._request, self.dt, self._intended, measured=self._q)
                if not np.array_equal(cmd, self._sent):          # nothing new to say: stay off the bus
                    self.arm.write_goals(cmd)
                    self._sent = cmd
            self._tick += self.dt
            late = self.clock.now() - self._tick
            if late > RESYNC_S:                                  # the loop was not running (between actions): start again
                self._tick = self.clock.now()
                self.resyncs += 1
            elif late > 1e-9:
                self.overruns += 1
                self.max_late_s = max(self.max_late_s, late)
            else:
                self.clock.sleep_until(self._tick)

    def torque_off(self):
        """Torque off on every servo: the arm goes limp. From this call on no goal is written.
        Raises BusError if a servo could not be confirmed off."""
        self.torque_enabled = False
        self.arm.disable()

    def torque_on(self) -> np.ndarray:
        """Torque back on, holding the pose the encoders report now. Returns that pose."""
        measured = self.arm.read_positions()
        self.safety.reset(measured)
        q = self.safety.q.copy()
        self.arm.enable(q)
        self._request, self._intended, self._sent = q.copy(), None, q.copy()
        self._q, self._q_time, self.q_fresh = measured, None, True
        self._tick = self.clock.now()
        self.torque_enabled = True
        return q

    def report(self) -> str:
        st = self.driver.stats
        return (f"servo bus: packets sent {st['tx_packets']}, replies {st['rx_packets']}, retries {st['retries']}, timeouts "
                f"{st['timeouts']}, failed {st['failures']}; control ticks that overran {self.overruns} "
                f"(worst {self.max_late_s * 1000:.2f} ms late); position reads that failed {self.read_failures}")
