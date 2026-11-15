"""The rover board's firmware (firmware/rover-bridge/main/bridge.c), compiled for this PC: by itself, and between
the hub and the rover's emulated servos.

Aimed at classes of bugs:
  * a bridge that changes, drops, repeats or reorders a byte (any chunking, either way);
  * a watchdog that does not trip, trips while the hub is talking, trips on every tick, or is kept alive by the
    servos' own chatter;
  * a stop packet the servos would not accept (wrong framing, length or checksum), or one that draws a reply;
  * time: the board's millisecond counter wraps after 49 days;
  * the firmware in the loop behaving differently from the simulation's model of it.
"""
import math

import numpy as np
import pytest

from hub.bus import hx_registers as R
from hub.bus import protocol as P
from hub.hub import Hub
from so101sim import scenes
from so101sim.rover_fw import RoverFirmware
from so101sim.world import BOARD_WATCHDOG_S, World


def bare() -> dict:
    s = scenes.read("no_arm")
    s["objects"], s["fixtures"] = [], []
    s.pop("tasks")
    return s


# ---------------------------------------------------------------- the core, by itself
@pytest.mark.parametrize("seed", range(6))
def test_bytes_pass_unchanged_in_any_pieces_both_ways(seed):
    rng = np.random.default_rng(seed)
    fw = RoverFirmware()
    fw.tick(0)
    for way in (fw.from_hub, fw.from_bus):
        data = rng.integers(0, 256, int(rng.integers(1, 9000)), dtype=np.uint8).tobytes()
        out, k, t = bytearray(), 0, 1
        while k < len(data):
            n = int(rng.integers(1, 700))
            out += way(data[k:k + n], t)
            k += n
            t += int(rng.integers(0, 3))
        assert bytes(out) == data
    assert fw.count("hub_bytes") > 0 and fw.count("bus_bytes") > 0 and fw.count("echo_dropped") == 0


def test_the_stop_is_one_packet_the_servos_accept_and_none_answers():
    fw = RoverFirmware(wheel_ids=(11, 12), speed_addr=R.BY_NAME["motion_speed"].addr, speed_size=R.BY_NAME["motion_speed"].size)
    raw = fw.tick(0)                                                # at start-up: it cannot know what the wheels were told
    (pkt,) = P.Parser().feed(raw)
    assert pkt.id == P.BROADCAST and pkt.code == P.SYNC_WRITE, "a broadcast draws no reply"
    assert pkt.params == bytes([46, 2, 11, 0, 0, 12, 0, 0]) and raw == P.encode(P.BROADCAST, P.SYNC_WRITE, pkt.params)
    four = RoverFirmware(wheel_ids=(3, 4, 5, 6), speed_addr=50, speed_size=4).tick(0)
    (pkt,) = P.Parser().feed(four)
    assert pkt.params == bytes([50, 4]) + b"".join(bytes([i, 0, 0, 0, 0]) for i in (3, 4, 5, 6))


@pytest.mark.parametrize("bad", [dict(wheel_ids=()), dict(wheel_ids=(1, 2, 3, 4, 5)), dict(speed_size=0), dict(speed_size=5), dict(watchdog_ms=0)])
def test_a_configuration_it_cannot_use_is_refused(bad):
    with pytest.raises(ValueError):
        RoverFirmware(**bad)


@pytest.mark.parametrize("start", [0, 5_000, 2 ** 32 - 150, 2 ** 32 - 1])
def test_it_trips_once_per_silence_and_never_while_the_hub_talks(start):
    """`start` puts the millisecond counter's wrap-around before, inside and after the silence."""
    fw = RoverFirmware(watchdog_ms=400, repeat_ms=0, now_ms=start)
    assert fw.tick(start) and fw.count("trips") == 1                # the start-up stop
    assert fw.tick(start + 1) == b"" and fw.tick(start + 5000) == b"", "once: not on every tick"
    t = start + 6000
    for _ in range(40):                                             # the hub talks every 100 ms for 4 s
        fw.from_hub(b"\xff\xff\x0b\x02\x01\xf1", t)
        for dt in range(0, 100, 7):
            assert fw.tick(t + dt) == b"", "tripped while the hub was talking"
        t += 100
    assert fw.count("trips") == 1 and fw.count("stopped") == 0
    assert fw.tick(t - 100 + 399) == b"" and fw.tick(t - 100 + 400) != b"", "400 ms after its last word, not before"
    assert fw.count("trips") == 2 and fw.count("stopped") == 1
    assert fw.tick(t + 10_000) == b"" and fw.count("trips") == 2
    fw.from_hub(b"\x00", t + 20_000)                                # it speaks again: armed again
    assert fw.count("stopped") == 0 and fw.tick(t + 20_399) == b"" and fw.tick(t + 20_400) != b"" and fw.count("trips") == 3


def test_what_the_servos_say_does_not_keep_the_wheels_going():
    fw = RoverFirmware(watchdog_ms=400, repeat_ms=0)
    fw.tick(0)
    fw.from_hub(b"\xff\xff\xfe\x04\x82\x38\x02", 1000)
    for t in range(1001, 1400, 10):                                 # a servo chatters all the while
        assert fw.from_bus(b"\xff\xff\x0b\x04\x00\x00\x08\xe8", t)
    assert fw.tick(1399) == b"" and fw.tick(1400) != b""


def test_the_stop_is_repeated_while_the_silence_lasts():
    fw = RoverFirmware(watchdog_ms=400, repeat_ms=1000)
    first = fw.tick(0)
    assert fw.tick(999) == b"" and fw.tick(1000) == first and fw.tick(1500) == b"" and fw.tick(2000) == first
    assert fw.count("trips") == 1 and fw.count("stops_sent") == 3, "one silence, three packets"


def test_an_interface_that_echoes_is_not_heard_twice():
    fw = RoverFirmware(strip_echo=True, repeat_ms=0)
    stop = fw.tick(0)
    assert fw.from_bus(stop, 1) == b"", "the stop's own echo"
    sent = b"\xff\xff\x0b\x04\x02\x38\x02\xb4"
    assert fw.from_hub(sent, 100) == sent
    reply = b"\xff\xff\x0b\x04\x00\x00\x08\xe8"
    assert fw.from_bus(sent[:5], 101) == b"" and fw.from_bus(sent[5:] + reply, 102) == reply, "what was sent comes back first, then the answer"
    assert fw.count("echo_dropped") == len(stop) + len(sent)
    assert fw.from_hub(sent, 200) == sent                           # an echo that never comes is forgotten, and the next reply kept
    assert fw.from_bus(reply, 300) == reply


# ---------------------------------------------------------------- between the hub and the servos
def wheel_speeds(world):
    bus = next(b for b in world.buses if b.radio)
    return [s.get("motion_speed") & 0x7FFF for s in bus.servos if s.id in (11, 12)]


@pytest.mark.parametrize("firmware", [False, True], ids=["the simulation's model", "the firmware"])
def test_wheels_left_turning_are_stopped_when_the_hub_goes_quiet(firmware):
    world = World(bare(), rover_firmware=firmware)
    hub = Hub(world.ports)
    bus = next(b for b in world.buses if b.radio)
    assert (bus.fw is not None) == firmware
    hub.rover._wheels(0.06, 0.0)                                    # set off, and say no more
    world.step(BOARD_WATCHDOG_S - 0.1)
    assert all(v > 0 for v in wheel_speeds(world)), "stopped before the watchdog time"
    world.step(0.25)
    assert wheel_speeds(world) == [0, 0], "still told to turn after the watchdog time"
    world.step(0.5)
    m, d = world.model, world.data
    a = d.xpos[m.body("rover").id].copy()
    world.step(0.5)
    assert np.linalg.norm(d.xpos[m.body("rover").id] - a) < 1e-3, "the rover is still moving"
    assert bus.board_stops >= 1 and not bus.wire.warnings(), "the servos took the stop without complaint"


def test_the_hub_drives_and_looks_through_the_firmware_as_through_the_model():
    """The same commands with the firmware in the loop and with the simulation's model of it: the same motion."""
    ends = []
    for firmware in (False, True):
        world = World(bare(), rover_firmware=firmware)
        hub = Hub(world.ports)
        r = hub.rover
        assert r.forward(0.12)["reached"] and r.turn(40.0)["reached"] and r.raise_head(0.12)["reached"] and r.stow()["reached"]
        m, d = world.model, world.data
        i = m.body("rover").id
        ends.append((d.xpos[i][:2].copy(), math.atan2(d.xmat[i].reshape(3, 3)[1, 0], d.xmat[i].reshape(3, 3)[0, 0]), r.pose.copy()))
        bus = next(b for b in world.buses if b.radio)
        assert not bus.wire.warnings()
        if firmware:
            assert bus.fw.count("hub_bytes") > 1000 and bus.fw.count("bus_bytes") > 1000
            assert bus.fw.count("trips") == 1, "only the start-up stop: it never cut in on the hub"
    assert np.linalg.norm(ends[0][0] - ends[1][0]) < 0.004 and abs(ends[0][1] - ends[1][1]) < math.radians(1.5)
    assert np.abs(ends[0][2] - ends[1][2]).max() < 0.01


def test_a_link_cut_mid_drive_stops_the_rover_with_the_firmware_in_the_loop():
    world = World(bare(), rover_firmware=True)
    hub = Hub(world.ports)
    bus = next(b for b in world.buses if b.radio)
    hub.rover._wheels(0.07, 0.0)
    world.step(0.3)
    bus.link_up = False                                             # nothing passes
    world.step(1.0)
    assert wheel_speeds(world) == [0, 0] and bus.fw.count("stopped") == 1
    m, d = world.model, world.data
    a = d.xpos[m.body("rover").id].copy()
    world.step(0.5)
    assert np.linalg.norm(d.xpos[m.body("rover").id] - a) < 1e-3
    bus.link_up = True
    assert hub.rover.forward(0.05)["reached"] and bus.fw.count("stopped") == 0, "and it drives again when the link is back"
