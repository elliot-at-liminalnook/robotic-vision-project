"""Controller-side driver for the servo bus. The same code is meant to talk to
the simulated bus (bus_backend.SimPort) and to the real one (a serial port).

A port is anything with:
    write(data: bytes) -> None
    read(n: int, timeout_s: float) -> bytes      # up to n bytes; fewer means the timeout passed
    reset_input() -> None                        # drop bytes received so far

Every reply is checked (checksum, id, length). A transaction that fails is
retried `retries` times and then raises; nothing is ever returned unchecked.
`stats` counts what happened.
"""
from __future__ import annotations

from collections import Counter

from . import protocol as P


class BusError(Exception):
    """A transaction failed after all retries."""


class FeetechBus:
    def __init__(self, port, *, reply_timeout_s: float = 0.02, retries: int = 1, echo: bool = False):
        self.port = port
        self.timeout = float(reply_timeout_s)
        self.retries = int(retries)
        self.echo = echo                 # the adapter repeats what we send: read it back and drop it
        self.stats: Counter = Counter()
        self.status: dict[int, int] = {}  # last status byte per servo (register 65: error bits)

    # ---- transactions ------------------------------------------------------
    def ping(self, servo_id: int) -> int:
        """Returns the servo's status byte. Raises BusError if nothing valid answers."""
        self._transact(servo_id, P.PING, b"", {servo_id: 0})
        return self.status[servo_id]

    def read(self, servo_id: int, addr: int, n: int) -> bytes:
        return self._transact(servo_id, P.READ, bytes([addr, n]), {servo_id: n})[servo_id]

    def write(self, servo_id: int, addr: int, data: bytes, ack: bool = True):
        """Write registers. With ack (the servo's default response level) the status reply is awaited."""
        self._transact(servo_id, P.WRITE, bytes([addr]) + bytes(data), {servo_id: 0} if ack else {})

    def sync_write(self, addr: int, data_by_id: dict[int, bytes]):
        """One broadcast packet writing the same registers of several servos. No reply, so no confirmation."""
        sizes = {len(d) for d in data_by_id.values()}
        if len(sizes) != 1:
            raise ValueError("sync_write needs the same data length for every servo")
        params = bytes([addr, sizes.pop()]) + b"".join(bytes([i]) + bytes(d) for i, d in data_by_id.items())
        self._transact(P.BROADCAST, P.SYNC_WRITE, params, {})

    def sync_read(self, addr: int, n: int, ids) -> dict[int, bytes]:
        ids = list(ids)
        return self._transact(P.BROADCAST, P.SYNC_READ, bytes([addr, n]) + bytes(ids), {i: n for i in ids})

    # ---- internals ---------------------------------------------------------
    def _transact(self, target: int, instr: int, params: bytes, expect: dict[int, int]) -> dict[int, bytes]:
        """Send one packet and collect a reply of expect[id] parameter bytes from each expected servo."""
        packet = P.encode(target, instr, params)
        problem = ""
        for attempt in range(self.retries + 1):
            if attempt:
                self.stats["retries"] += 1
            self.port.reset_input()
            self.port.write(packet)
            self.stats["tx_packets"] += 1
            if self.echo:
                self.port.read(len(packet), self.timeout)
            if not expect:
                return {}
            got, problem = self._collect(expect)
            if not problem:
                return got
        self.stats["failures"] += 1
        raise BusError(f"{P.INSTRUCTION_NAMES.get(instr, hex(instr))} failed after {self.retries + 1} tries: {problem}")

    def _collect(self, expect: dict[int, int]) -> tuple[dict[int, bytes], str]:
        want = sum(6 + n for n in expect.values())
        parser, got, received = P.Parser(), {}, 0
        while len(got) < len(expect):
            asked = max(1, want - received)
            chunk = self.port.read(asked, self.timeout)
            received += len(chunk)
            for pkt in parser.feed(chunk):
                if pkt.id not in expect or pkt.id in got:
                    self.stats["unexpected_replies"] += 1
                elif len(pkt.params) != expect[pkt.id]:
                    self.stats["wrong_length_replies"] += 1
                else:
                    got[pkt.id] = pkt.params
                    self.status[pkt.id] = pkt.code
                    self.stats["rx_packets"] += 1
            if len(chunk) < asked:               # the read timed out: don't wait a second time
                break
        self.stats["checksum_errors"] += parser.bad
        if len(got) == len(expect):
            return got, ""
        self.stats["timeouts"] += 1
        missing = sorted(set(expect) - set(got))
        detail = f"{parser.bad} corrupt" if parser.bad else "nothing valid"
        return got, f"no reply from servo id {', '.join(map(str, missing))} ({received} bytes received, {detail})"
