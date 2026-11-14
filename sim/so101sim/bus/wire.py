"""The simulated half-duplex servo bus: USB adapter, one wire, the servos on it.

Models what the controller can observe from its end:
  * time: every byte takes 10 bit times (8N1) at the baud rate; a servo answers
    `response_delay_s` after the request ends; SYNC_READ replies come one after
    another in the order of the id list; the USB adapter adds a latency in each
    direction;
  * faults (seeded): a servo misses a packet, a reply is lost, a reply byte is
    corrupted;
  * wiring mistakes: two servos with the same id answer at once and their bytes
    collide; a servo set to another baud rate hears nothing; an adapter that
    echoes what the host sends.

Not modelled: collisions between the host and a servo talking at the same time
(a late reply simply arrives late, and can be mistaken for the next answer,
as on a real bus).

`trace` is a logic-analyser view of every frame; `events` are things worth a
look (ignored writes, collisions, injected faults).
"""
from __future__ import annotations

import heapq
import random
from collections import deque

from . import protocol as P


class VirtualBus:
    def __init__(self, servos, *, baud: int = 1_000_000, response_delay_s: float = 250e-6,
                 usb_tx_latency_s: float = 1e-3, usb_rx_latency_s: float = 1e-3, echo: bool = False,
                 drop_command_prob: float = 0.0, drop_reply_prob: float = 0.0, corrupt_prob: float = 0.0,
                 seed: int = 0, trace_len: int = 4000):
        self.servos = list(servos)
        self.baud = int(baud)
        self.byte_time = 10.0 / self.baud
        self.response_delay = float(response_delay_s)
        self.tx_latency, self.rx_latency = float(usb_tx_latency_s), float(usb_rx_latency_s)
        self.echo = echo
        self.drop_command_prob, self.drop_reply_prob, self.corrupt_prob = drop_command_prob, drop_reply_prob, corrupt_prob
        self.rng = random.Random(seed)
        self.trace: deque = deque(maxlen=trace_len)      # (time, source, bytes)
        self.events: deque = deque(maxlen=trace_len)     # (time, kind, message)
        self.now = 0.0
        self.reset()
        for s in self.servos:
            s.log = self.log

    def reset(self):
        self.parser = P.Parser()
        self.wire_free = -float("inf")
        self._tx: list = []          # heap of (end time, seq, bytes): host frames on their way to the servos
        self._rx: list = []          # heap of (arrival time, seq, byte): bytes on their way to the host
        self._seq = 0
        self.bytes_tx = self.bytes_rx = 0

    def log(self, kind: str, msg: str):
        self.events.append((self.now, kind, msg))

    def warnings(self) -> list[str]:
        return [m for _, k, m in self.events if k == "warn"]

    # ---- host side ---------------------------------------------------------
    def host_write(self, data: bytes, t: float):
        start = max(t + self.tx_latency, self.wire_free)
        end = start + len(data) * self.byte_time
        self.wire_free = end
        self._seq += 1
        heapq.heappush(self._tx, (end, self._seq, bytes(data)))
        self.bytes_tx += len(data)
        self.trace.append((start, "host", bytes(data)))
        if self.echo:
            self._to_host(data, start)

    def host_read(self, n: int, t: float) -> bytes:
        """Up to n bytes that have reached the host by time t."""
        out = bytearray()
        while self._rx and len(out) < n and self._rx[0][0] <= t + 1e-12:
            out.append(heapq.heappop(self._rx)[2])
        return bytes(out)

    def host_discard(self, t: float):
        while self._rx and self._rx[0][0] <= t + 1e-12:
            heapq.heappop(self._rx)

    def available(self, t: float) -> int:
        return sum(1 for a, _, _ in self._rx if a <= t + 1e-12)

    def next_event_time(self, after: float) -> float | None:
        """When something next happens after time `after`: a host frame reaches the servos
        (those are processed in order, so the earliest pending one counts even if it is due),
        or another byte reaches the host."""
        times = [a for a, _, _ in self._rx if a > after + 1e-12]
        if self._tx:
            times.append(max(self._tx[0][0], after))
        return min(times) if times else None

    def idle(self) -> bool:
        """No host frame is still on its way to the servos."""
        return not self._tx

    # ---- the wire ----------------------------------------------------------
    def process(self, t: float):
        """Deliver every host frame that has finished transmitting by time t."""
        while self._tx and self._tx[0][0] <= t + 1e-12:
            end, _, data = heapq.heappop(self._tx)
            self.now = end
            for pkt in self.parser.feed(data):
                self._dispatch(pkt, end)
        self.now = max(self.now, t)

    def _dispatch(self, pkt: P.Packet, t: float):
        replies: dict[int, list] = {}
        for s in self.servos:
            if s.baud != self.baud:
                continue                                  # wrong baud rate: it hears only noise
            if (pkt.id == s.id or pkt.id == P.BROADCAST) and self.rng.random() < self.drop_command_prob:
                self.log("fault", f"servo {s.id} missed a packet (injected)")
                continue
            r = s.receive(pkt)
            if r is not None:
                replies.setdefault(r[1], []).append((s, r[0]))
        for slot, group in sorted(replies.items()):
            data = group[0][1]
            if len(group) > 1:                            # same id twice: both drive the wire, low bits win
                n = min(len(d) for _, d in group)
                data = bytes(_and(*(d[i] for _, d in group)) for i in range(n))
                self.log("warn", f"{len(group)} servos answered as id {group[0][0].id} at once: reply corrupted")
            start = t + self.response_delay + slot * (len(data) * self.byte_time + self.response_delay)
            if self.rng.random() < self.drop_reply_prob:
                self.log("fault", f"reply from servo {group[0][0].id} lost (injected)")
                continue
            if self.rng.random() < self.corrupt_prob:
                i = self.rng.randrange(len(data))
                data = data[:i] + bytes([data[i] ^ (1 << self.rng.randrange(8))]) + data[i + 1:]
                self.log("fault", f"reply from servo {group[0][0].id} corrupted (injected)")
            self.trace.append((start, f"servo {group[0][0].id}", data))
            self._to_host(data, start)

    def _to_host(self, data: bytes, start: float):
        for i, b in enumerate(data):
            self._seq += 1
            heapq.heappush(self._rx, (start + (i + 1) * self.byte_time + self.rx_latency, self._seq, b))
        self.bytes_rx += len(data)

    # ---- debugging ---------------------------------------------------------
    def format_trace(self, last: int = 40) -> str:
        lines = []
        for t, src, data in list(self.trace)[-last:]:
            pkts = P.Parser().feed(data)
            what = "; ".join(_describe(p, src == "host") for p in pkts) or "(not a valid packet)"
            lines.append(f"{t * 1000:10.3f} ms  {src:8s} {data.hex(' ')}   {what}")
        return "\n".join(lines)


def _and(*values: int) -> int:
    out = 0xFF
    for v in values:
        out &= v
    return out


def _describe(p: P.Packet, from_host: bool) -> str:
    if not from_host:
        return f"status 0x{p.code:02X}" + (f" data [{p.params.hex(' ')}]" if p.params else "")
    name = P.INSTRUCTION_NAMES.get(p.code, f"0x{p.code:02X}")
    target = "all" if p.id == P.BROADCAST else f"id {p.id}"
    if p.code in (P.READ, P.SYNC_READ) and len(p.params) >= 2:
        extra = f" ids {list(p.params[2:])}" if p.code == P.SYNC_READ else ""
        return f"{name} {target}: {p.params[1]} bytes at {p.params[0]}{extra}"
    if p.code in (P.WRITE, P.REG_WRITE, P.SYNC_WRITE) and p.params:
        return f"{name} {target}: at {p.params[0]}"
    return f"{name} {target}"
