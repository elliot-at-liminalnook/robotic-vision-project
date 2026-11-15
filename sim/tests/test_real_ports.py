"""Ports on the real machine (hub/real.py), as far as they can be tried without the machine.

Aimed at classes of bugs:
  * a read that returns early, late, or more than was asked for; bytes lost or reordered across reads;
  * a link that has gone being treated as a link that is slow (an empty read, for ever), instead of said so;
  * a device that is not there today stopping the hub from starting with the ones that are.

The network port is tried against a real socket on this machine, and through it the hub's own bus driver talks to
emulated servos behind the rover board's firmware: the whole path a rover's servos are reached by, in real time.
"""
import socket
import threading
import time

import pytest

from hub import real
from hub.bus import BusError, FeetechBus
from hub.bus import hx_registers as R
from hub.discovery import scan
from hub.ports import PortError
from hub.bus import protocol as P
from so101sim.bus import ServoEmulator
from so101sim.rover_fw import RoverFirmware


class Board(threading.Thread):
    """A rover's board on this machine: a TCP server, the firmware's own logic, and servos that answer at once."""

    def __init__(self, ids=(11, 12, 21, 22, 23, 24), delay_s: float = 0.0):
        super().__init__(daemon=True)
        self.server = socket.socket()
        self.server.bind(("127.0.0.1", 0))
        self.server.listen(1)
        self.port = self.server.getsockname()[1]
        self.servos = {i: ServoEmulator(i) for i in ids}
        self.fw = RoverFirmware(wheel_ids=(11, 12), watchdog_ms=400, repeat_ms=0)
        self.delay_s, self.running, self.conn = delay_s, True, None
        self.stop_packets = 0
        self.start()

    def ms(self) -> int:
        return int(time.monotonic() * 1000)

    def to_servos(self, data: bytes) -> bytes:
        out = b""
        for pkt in self.parser.feed(data):
            for s in self.servos.values():
                reply = s.receive(pkt)
                if reply:
                    out += reply[0]
        return out

    def run(self):
        self.parser = P.Parser()
        self.to_servos(self.fw.tick(self.ms()))
        self.server.settimeout(0.05)
        while self.running:
            if self.conn is None:
                try:
                    self.conn, _ = self.server.accept()
                    self.conn.settimeout(0.005)
                except OSError:
                    pass
            else:
                try:
                    data = self.conn.recv(4096)
                    if not data:
                        self.conn.close()
                        self.conn = None
                        continue
                    reply = self.to_servos(self.fw.from_hub(data, self.ms()))
                    if reply:
                        time.sleep(self.delay_s)
                        self.conn.sendall(self.fw.from_bus(reply, self.ms()))
                except socket.timeout:
                    pass
                except OSError:
                    self.conn = None
            stop = self.fw.tick(self.ms())
            if stop:
                self.stop_packets += 1
                self.to_servos(stop)

    def close(self):
        self.running = False
        self.join(1.0)
        for s in (self.conn, self.server):
            try:
                s and s.close()
            except OSError:
                pass


@pytest.fixture()
def board():
    b = Board()
    yield b
    b.close()


def test_the_clock_only_goes_forward_and_waits_until_asked():
    clock = real.WallClock()
    a = clock.now()
    clock.sleep_until(a + 0.05)
    b = clock.now()
    assert 0.045 <= b - a < 0.3
    clock.sleep_until(b - 1.0)                                      # a time already past: no wait
    assert clock.now() - b < 0.02


def test_a_servo_bus_is_reached_through_the_network_and_the_boards_firmware(board):
    port = real.TcpSerial("127.0.0.1", board.port)
    assert port.reply_time_s > 0
    assert scan(port) == [11, 12, 21, 22, 23, 24], "the hub's own discovery, over a real socket"
    bus = FeetechBus(port, reply_timeout_s=0.02 + port.reply_time_s, retries=1)
    reg = R.BY_NAME["max_torque_limit"]
    bus.write(21, reg.addr, reg.encode(50))
    assert reg.decode(bus.read(21, reg.addr, reg.size)) == 50 and board.servos[21].get("max_torque_limit") == 50
    with pytest.raises(BusError):
        bus.ping(5)                                                 # nobody of that id: a timeout, not a hang
    port.close()


def test_a_read_gives_what_has_come_within_the_time_and_no_more(board):
    port = real.TcpSerial("127.0.0.1", board.port)
    t = time.monotonic()
    assert port.read(10, 0.05) == b"" and 0.04 <= time.monotonic() - t < 0.2, "nothing sent, nothing read, after the time allowed"
    port.write(P.encode(11, P.PING))
    time.sleep(0.05)
    port.write(P.encode(12, P.PING))
    time.sleep(0.05)
    first = port.read(3, 0.2)
    rest = port.read(64, 0.05)
    assert len(first) == 3 and len(first + rest) == 12, "two replies of six bytes, split where the reads fell"
    assert [p.id for p in P.Parser().feed(first + rest)] == [11, 12], "in order, nothing lost"
    port.write(P.encode(21, P.PING))
    time.sleep(0.05)
    port.reset_input()
    assert port.read(6, 0.03) == b"", "what had arrived was thrown away"
    port.close()


def test_the_wheels_are_stopped_by_the_board_when_the_hub_stops_talking(board):
    port = real.TcpSerial("127.0.0.1", board.port)
    bus = FeetechBus(port, reply_timeout_s=0.07, retries=1)
    mode, torque, speed = (R.BY_NAME[n] for n in ("operating_mode", "torque_switch", "motion_speed"))
    for i in (11, 12):
        bus.write(i, mode.addr, mode.encode(1))
        bus.write(i, torque.addr, torque.encode(1))
        bus.write(i, speed.addr, speed.encode(900))
    assert all(board.servos[i].get("motion_speed") == 900 for i in (11, 12))
    for _ in range(4):                                              # talking: the wheels keep their speed
        time.sleep(0.15)
        bus.ping(21)
    assert all(board.servos[i].get("motion_speed") == 900 for i in (11, 12))
    time.sleep(0.7)                                                 # silence
    assert all(board.servos[i].get("motion_speed") == 0 for i in (11, 12)) and board.stop_packets >= 1
    port.close()


def test_a_link_that_has_gone_is_said_to_have_gone(board):
    port = real.TcpSerial("127.0.0.1", board.port)
    assert scan(port)[:1] == [11]
    board.close()
    with pytest.raises(PortError, match="gone|closed"):
        for _ in range(50):
            port.write(P.encode(11, P.PING))
            port.read(6, 0.02)
    with pytest.raises(PortError):                                  # and stays gone: no silent empty reads
        port.read(1, 0.01)
    with pytest.raises(PortError, match="nothing answers"):
        real.TcpSerial("127.0.0.1", board.port, connect_timeout_s=0.3)


def test_what_is_not_there_today_is_noted_and_the_rest_is_offered(board, tmp_path):
    config = tmp_path / "machine.toml"
    config.write_text(f'[[serial]]\ntcp = "127.0.0.1:{board.port}"\n\n[[serial]]\ntcp = "127.0.0.1:1"\n\n'
                      '[[camera]]\nurl = "http://127.0.0.1:1/nothing"\npart_of = "rover"\nrole = "left"\n')
    ports = real.provider(config, find_usb=False, find_cameras=0)
    found = ports()
    assert len(found.serial) == 1 and scan(found.serial[0]) == [11, 12, 21, 22, 23, 24]
    assert found.cameras == [] and len(ports.notes) == 2 and any("127.0.0.1:1" in n for n in ports.notes)
    assert found.clock.now() >= 0
    again = ports()                                                 # asked again: the old link is closed, a new one opened
    with pytest.raises(PortError):
        found.serial[0].read(1, 0.01)
    assert scan(again.serial[0])[:2] == [11, 12]


def test_a_usb_adapter_or_camera_that_is_not_there_says_so():
    with pytest.raises(PortError):
        real.UsbSerial("/dev/no-such-adapter")
    with pytest.raises(PortError, match="could not be opened"):
        real.UvcCamera("/no/such/video", real.WallClock())
