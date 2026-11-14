"""Feetech-style servo bus packets (the framing the HX-30HM protocol manual describes).

    instruction: FF FF | ID | LEN | INSTR  | params... | CHK
    status:      FF FF | ID | LEN | STATUS | params... | CHK
    LEN = number of params + 2,  CHK = ~(ID + LEN + INSTR/STATUS + params) & 0xFF

Used by both ends: the controller's driver (driver.py) and the emulated servos
(servo_emu.py). Sources: docs/datasheets/feetech-protocol/ and
third_party/hiwonder-so-arm101/manuals/HX-30HM_Communication_Protocol_v1.0.pdf.
"""
from __future__ import annotations

from dataclasses import dataclass

HEADER = b"\xff\xff"
BROADCAST = 0xFE

PING = 0x01
READ = 0x02
WRITE = 0x03
REG_WRITE = 0x04
ACTION = 0x05
RESET = 0x0A
SYNC_READ = 0x82
SYNC_WRITE = 0x83

INSTRUCTION_NAMES = {PING: "PING", READ: "READ", WRITE: "WRITE", REG_WRITE: "REG_WRITE", ACTION: "ACTION",
                     RESET: "RESET", SYNC_READ: "SYNC_READ", SYNC_WRITE: "SYNC_WRITE"}

BAUD_RATES = (1_000_000, 500_000, 250_000, 128_000, 115_200, 76_800, 57_600, 38_400)   # register 6 index


def checksum(body: bytes) -> int:
    """Checksum over ID, LEN, INSTR/STATUS and the params."""
    return (~sum(body)) & 0xFF


def encode(servo_id: int, code: int, params: bytes = b"") -> bytes:
    """One packet. `code` is the instruction (to a servo) or the status byte (from one)."""
    if not 0 <= servo_id <= 0xFE:
        raise ValueError(f"servo id {servo_id} out of range")
    if len(params) > 253:
        raise ValueError("too many parameters for one packet")
    body = bytes([servo_id, len(params) + 2, code]) + bytes(params)
    return HEADER + body + bytes([checksum(body)])


@dataclass(frozen=True)
class Packet:
    id: int
    code: int               # instruction or status
    params: bytes

    def __str__(self) -> str:
        return f"id={self.id} code=0x{self.code:02X} params=[{self.params.hex(' ')}]"


class Parser:
    """Byte-stream parser. feed() returns the complete, valid packets found so far.

    Resynchronises on anything that isn't a packet: bytes before a header are
    skipped, and a packet with a bad checksum or length costs only its first
    byte, so a real packet that starts inside the garbage is still found.
    `bad` counts discarded candidates (checksum or length errors), `skipped`
    the stray bytes.
    """

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0
        self.skipped = 0

    def reset(self):
        self.buf.clear()

    def feed(self, data: bytes) -> list[Packet]:
        self.buf += data
        out = []
        while True:
            start = self.buf.find(HEADER)
            if start < 0:
                keep = 1 if self.buf.endswith(b"\xff") else 0     # a header may be half-received
                self.skipped += len(self.buf) - keep
                del self.buf[:len(self.buf) - keep]
                return out
            if start:
                self.skipped += start
                del self.buf[:start]
            if len(self.buf) < 4:
                return out
            if self.buf[2] == 0xFF:              # FF FF FF ...: the header starts one byte later
                self.skipped += 1
                del self.buf[:1]
                continue
            length = self.buf[3]
            if length < 2 or length > 255 - 2:
                self.bad += 1
                del self.buf[:1]
                continue
            total = 4 + length
            if len(self.buf) < total:
                return out
            body = bytes(self.buf[2:total - 1])
            if checksum(body) != self.buf[total - 1]:
                self.bad += 1
                del self.buf[:1]
                continue
            out.append(Packet(body[0], body[2], body[3:]))
            del self.buf[:total]


def u16(value: int) -> bytes:
    """Unsigned 16-bit, low byte first."""
    return bytes([value & 0xFF, (value >> 8) & 0xFF])


def from_u16(data: bytes) -> int:
    return data[0] | (data[1] << 8)


def sign_mag(value: int, sign_bit: int) -> int:
    """Signed value -> sign-magnitude register value (the sign is one bit, the rest the magnitude)."""
    mag = min(abs(int(value)), (1 << sign_bit) - 1)
    return mag | (1 << sign_bit) if value < 0 else mag


def from_sign_mag(raw: int, sign_bit: int) -> int:
    mag = raw & ((1 << sign_bit) - 1)
    return -mag if raw & (1 << sign_bit) else mag
