"""The servo bus in the loop: packets, emulated servos, the wire, the driver, and teleop through all of it.

Aimed at classes of bugs:
  * the packet layer round-trips any packet and finds packets in any garbage (fuzzed);
  * the register table is the vendor spreadsheet, and every register behaves by its own row;
  * no instruction stream, however malformed, breaks a servo or changes what it must not;
  * under injected faults the driver returns checked data or raises, never something wrong;
  * bus time is what the byte counts say it is;
  * the classic set-up mistakes are caught at connect, before any write;
  * the arm behaves the same with the bus in the loop as with direct joint targets;
  * the controller never writes a register the servo would ignore.
"""
import math
import random

import numpy as np
import pytest

from so101sim import JOINTS, load_backend
from so101sim.bus import BusError, FeetechBus, ServoEmulator, VirtualBus
from so101sim.bus import hx_registers as R
from so101sim.bus import protocol as P
from so101sim.bus_backend import BusBackend
from so101sim.config import ROOT, load_servos, position_loop, servo_limits
from so101sim.servo import ServoMap
from so101sim.servo_model import ServoSpec

HOME = np.array([0.0, -0.6, 0.9, 0.9, -1.57, 0.5])
TICK = 2 * math.pi / 4096


# ---------------------------------------------------------------- packets
def test_the_manuals_example_frames():
    assert P.encode(1, P.PING) == bytes.fromhex("ffff010201fb")
    assert P.encode(1, P.READ, bytes([0x38, 2])) == bytes.fromhex("ffff01040238 02be".replace(" ", ""))
    assert P.encode(1, P.WRITE, bytes.fromhex("2a00080000e803")) == bytes.fromhex("ffff0109032a00080000e803d5")
    assert P.encode(P.BROADCAST, P.ACTION) == bytes.fromhex("fffffe0205fa")
    (reply,) = P.Parser().feed(bytes.fromhex("ffff0104001805dd"))
    assert (reply.id, reply.code, P.from_u16(reply.params)) == (1, 0, 1304)


@pytest.mark.parametrize("seed", range(5))
def test_parser_finds_every_packet_in_any_stream(seed):
    """Packets separated by garbage, delivered in arbitrary chunks, all come out, in order, and nothing else."""
    rng = random.Random(seed)
    sent, stream = [], bytearray()
    for _ in range(200):
        if rng.random() < 0.4:                                     # noise that cannot hold a whole valid packet
            stream += bytes(rng.choice([0x00, 0x55, 0xFF, 0xFE, 0x02]) for _ in range(rng.randrange(1, 4)))
            stream += b"\x00"                                      # ... and does not end in a half header
        pkt = P.Packet(rng.randrange(0, 255), rng.randrange(0, 256), bytes(rng.randrange(256) for _ in range(rng.randrange(0, 40))))
        sent.append(pkt)
        stream += P.encode(pkt.id, pkt.code, pkt.params)
    parser, got, i = P.Parser(), [], 0
    while i < len(stream):
        n = rng.randrange(1, 30)
        got += parser.feed(bytes(stream[i:i + n]))
        i += n
    assert got == sent


@pytest.mark.parametrize("seed", range(3))
def test_a_flipped_bit_is_never_accepted(seed):
    rng = random.Random(seed)
    for _ in range(500):
        frame = bytearray(P.encode(rng.randrange(0, 254), rng.randrange(256), bytes(rng.randrange(256) for _ in range(rng.randrange(0, 12)))))
        good = P.Parser().feed(bytes(frame))[0]
        frame[rng.randrange(len(frame))] ^= 1 << rng.randrange(8)
        assert good not in P.Parser().feed(bytes(frame))


@pytest.mark.parametrize("bit", [10, 11, 15])
def test_sign_magnitude_round_trips(bit):
    for v in (0, 1, -1, 37, -37, (1 << bit) - 1, -((1 << bit) - 1)):
        assert P.from_sign_mag(P.sign_mag(v, bit), bit) == v


# ---------------------------------------------------------------- register table
def test_register_table_is_the_vendor_spreadsheet():
    openpyxl = pytest.importorskip("openpyxl")
    sheet = openpyxl.load_workbook(ROOT / "docs/datasheets/hiwonder-hx-servos/Hiwonder_HX-30HM_Register_List_and_Parameters.xlsx")["Register List"]
    rows = [r for r in sheet.iter_rows(min_row=5, values_only=True) if isinstance(r[0], int)]
    assert [r[0] for r in rows] == [r.addr for r in R.REGISTERS], "addresses differ from the spreadsheet"
    known_sheet_errors = {("overcurrent_threshold", "hi")}        # the sheet says max 511 with default 6000
    for row, reg in zip(rows, R.REGISTERS):
        _, _, _, size, default, area, access, lo, hi = row[:9]
        assert reg.size == size and reg.default == default and reg.area == area, reg.name
        assert reg.writable == (access == "Read/write"), reg.name
        if reg.writable:
            for key, want, have in (("lo", lo, reg.lo), ("hi", hi, reg.hi)):
                if isinstance(want, int) and (reg.name, key) not in known_sheet_errors:
                    assert have == want, f"{reg.name}.{key}"
    assert not any(a.addr < b.addr + b.size and b.addr < a.addr + a.size
                   for i, a in enumerate(R.REGISTERS) for b in R.REGISTERS[i + 1:]), "registers overlap"


def fresh_servo(servo_id=1, **kw):
    notes = []
    s = ServoEmulator(servo_id, ServoSpec.from_config(), log=lambda kind, msg: notes.append((kind, msg)), **kw)
    return s, notes


def ask(servo, instr, params=b"", servo_id=None):
    reply = servo.receive(P.Packet(servo.id if servo_id is None else servo_id, instr, bytes(params)))
    return None if reply is None else P.Parser().feed(reply[0])[0]


@pytest.mark.parametrize("reg", R.REGISTERS, ids=lambda r: r.name)
def test_every_register_behaves_by_its_row(reg):
    s, notes = fresh_servo(7)
    read = lambda: reg.decode(ask(s, P.READ, [reg.addr, reg.size]).params)   # noqa: E731
    assert read() == (7 if reg.name == "id" else reg.default)
    if not reg.writable:
        ask(s, P.WRITE, bytes([reg.addr]) + b"\x05" * reg.size)
        assert read() == reg.default and any(k == "warn" for k, _ in notes), "a read-only register took a write"
        return
    if reg.name in ("id", "baud_rate", "response_level", "torque_switch"):
        return                                   # changing these changes how the servo answers; tested below
    for value in (reg.lo, reg.hi, (reg.lo + reg.hi) // 2):
        assert ask(s, P.WRITE, bytes([reg.addr]) + reg.encode(value)) is not None, "no acknowledgement"
        assert read() == value
    if reg.name != "operating_mode":             # modes 1 and 2 are accepted but reported as not modelled
        assert not [m for k, m in notes if k == "warn"], notes
    top = (1 << (reg.sign_bit if reg.sign_bit is not None else 8 * reg.size)) - 1
    if reg.hi < top:                             # one past the documented maximum is clamped and reported
        ask(s, P.WRITE, bytes([reg.addr]) + reg.encode(reg.hi + 1))
        assert read() == reg.hi and any(k == "warn" for k, _ in notes)


def test_saved_registers_survive_power_off_only_when_unlocked():
    s, _ = fresh_servo()
    p = R.BY_NAME["position_p_gain"]
    ask(s, P.WRITE, [p.addr, 20])
    s.power_cycle()
    assert s.get("position_p_gain") == 20, "lock flag 0: the value is saved"
    ask(s, P.WRITE, [R.BY_NAME["lock_flag"].addr, 1])
    ask(s, P.WRITE, [p.addr, 50])
    assert s.get("position_p_gain") == 50
    s.power_cycle()
    assert s.get("position_p_gain") == 20, "lock flag 1: the value is lost at power-off"
    ask(s, P.WRITE, [R.BY_NAME["torque_switch"].addr, 1])
    s.power_cycle()
    assert s.get("torque_switch") == 0, "SRAM registers return to their defaults"


def test_reg_write_waits_for_action_and_broadcasts_get_no_reply():
    s, _ = fresh_servo(3)
    goal = R.BY_NAME["target_position"]
    assert ask(s, P.REG_WRITE, bytes([goal.addr]) + goal.encode(1234)) is not None
    assert s.get("target_position") == 0 and s.get("async_write_flag") == 1
    assert ask(s, P.ACTION, servo_id=P.BROADCAST) is None
    assert s.get("target_position") == 1234 and s.get("async_write_flag") == 0
    assert ask(s, P.WRITE, bytes([goal.addr]) + goal.encode(99), servo_id=P.BROADCAST) is None
    assert s.get("target_position") == 99
    assert ask(s, P.PING, servo_id=4) is None, "answered a packet for another servo"
    ask(s, P.WRITE, [R.BY_NAME["response_level"].addr, 0])
    assert ask(s, P.WRITE, bytes([goal.addr]) + goal.encode(5)) is None, "response level 0: writes are not acknowledged"
    assert ask(s, P.READ, [goal.addr, 2]) is not None and ask(s, P.PING) is not None


@pytest.mark.parametrize("seed", range(4))
def test_no_instruction_stream_breaks_a_servo(seed):
    """Random and malformed instructions: no exception, fixed registers stay fixed, every value stays in range."""
    rng = random.Random(seed)
    s, _ = fresh_servo(1)
    codes = [P.PING, P.READ, P.WRITE, P.REG_WRITE, P.ACTION, P.RESET, P.SYNC_READ, P.SYNC_WRITE, 0x06, 0x99]
    for _ in range(3000):
        pkt = P.Packet(rng.choice([1, 1, P.BROADCAST, 2]), rng.choice(codes),
                       bytes(rng.randrange(256) if rng.random() < 0.5 else rng.randrange(72) for _ in range(rng.randrange(0, 9))))
        reply = s.receive(pkt)
        assert reply is None or len(P.Parser().feed(reply[0])) == 1, "the reply is not a valid packet"
        s.control(rng.uniform(-2, 2), rng.uniform(-6, 6), 0.002)
        if s.id != 1 or s.get("baud_rate") != 0:                 # the stream renamed or re-bauded the servo: put it back
            s._set("id", 1)
            s._set("baud_rate", 0)
        for reg in R.REGISTERS:
            v = s.get(reg.name)
            if reg.area == "ROM":
                assert v == reg.default, f"{reg.name} changed"
            elif reg.writable:
                assert reg.lo <= v <= reg.hi, f"{reg.name} = {v}"
        assert math.isfinite(s.torque) and abs(s.torque) <= s.base.tau_s + 1e-9


# ---------------------------------------------------------------- the servo's control loop
def test_a_blocked_servo_trips_overload_protection_and_reports_it():
    s, _ = fresh_servo()
    goal = R.BY_NAME["target_position"]
    ask(s, P.WRITE, bytes([goal.addr]) + goal.encode(2048 + 400))
    assert s.control(0.0, 0.0, 0.002) == 0.0, "torque is off after power-on"
    ask(s, P.WRITE, [R.BY_NAME["torque_switch"].addr, 1])
    spec = s.spec
    for _ in range(int(1.9 / 0.002)):
        tau = s.control(0.0, 0.0, 0.002)
    assert tau == pytest.approx(spec.tau_s) and not s.tripped
    for _ in range(int(0.2 / 0.002)):
        tau = s.control(0.0, 0.0, 0.002)
    assert s.tripped and tau == pytest.approx(spec.overload_cap * spec.stall_torque)
    assert ask(s, P.PING).code & R.BIT_OVERLOAD, "the status byte of every reply carries the overload bit"
    assert s.get("present_current") == pytest.approx(1000 * (0.1 + 2.9 * spec.overload_cap), abs=2)


@pytest.mark.parametrize("name,value,expect", [
    ("max_torque_limit", 500, 0.5),            # half the torque limit -> half the stall torque
    ("position_p_gain", 16, 0.5),              # half the P gain -> half the torque for the same small error
])
def test_registers_change_the_control_loop(name, value, expect):
    def torque(error_ticks, write=None):
        s, _ = fresh_servo()
        if write:
            r = R.BY_NAME[write[0]]
            ask(s, P.WRITE, bytes([r.addr]) + r.encode(write[1]))
        goal = R.BY_NAME["target_position"]
        ask(s, P.WRITE, bytes([goal.addr]) + goal.encode(2048 + error_ticks))
        ask(s, P.WRITE, [R.BY_NAME["torque_switch"].addr, 1])
        return s.control(0.0, 0.0, 0.002)
    error = 400 if name == "max_torque_limit" else 40
    assert torque(error, (name, value)) == pytest.approx(expect * torque(error), rel=0.03)


def test_the_speed_register_limits_how_fast_the_goal_moves():
    s, _ = fresh_servo()
    speed, goal = R.BY_NAME["motion_speed"], R.BY_NAME["target_position"]
    s.control(0.0, 0.0, 0.002)
    ask(s, P.WRITE, [R.BY_NAME["torque_switch"].addr, 1])
    ask(s, P.WRITE, bytes([speed.addr]) + speed.encode(500))
    ask(s, P.WRITE, bytes([goal.addr]) + goal.encode(3048))
    for _ in range(500):
        s.control(0.0, 0.0, 0.002)
    assert s.setpoint == pytest.approx(2048 + 500, abs=2), "after 1 s at 500 steps/s"


def test_calibration_makes_the_present_position_read_2048():
    s, _ = fresh_servo(mount_zero_ticks=1900)
    s.control(0.1, 0.0, 0.002)
    ask(s, P.WRITE, [R.BY_NAME["torque_switch"].addr, 128])
    s.control(0.1, 0.0, 0.002)
    assert s.get("current_position") == 2048 and s.get("torque_switch") == 0
    s.control(0.1 + 100 * TICK, 0.0, 0.002)
    assert s.get("current_position") == 2148


# ---------------------------------------------------------------- wire + driver
class Bench:
    """Servos on a wire with a driver, and a clock that only the bus advances (no physics)."""

    def __init__(self, ids=(1, 2, 3, 4, 5, 6), **wire):
        self.servos = [ServoEmulator(i, ServoSpec.from_config()) for i in ids]
        self.wire = VirtualBus(self.servos, **wire)
        self.t = 0.0
        self.driver = FeetechBus(self, retries=wire.pop("retries", 1) if "retries" in wire else 1)
        for s in self.servos:
            s.control(0.0, 0.0, 0.002)

    def write(self, data):
        self.wire.host_write(data, self.t)

    def read(self, n, timeout_s):
        deadline = self.t + timeout_s
        while self.wire.available(self.t) < n:
            nxt = self.wire.next_event_time(self.t)
            if nxt is None or nxt > deadline:
                self.t = deadline
                self.wire.process(self.t)
                break
            self.t = nxt
            self.wire.process(self.t)
        return self.wire.host_read(n, self.t)

    def reset_input(self):
        self.wire.host_discard(self.t)


@pytest.mark.parametrize("baud", [1_000_000, 500_000, 115_200])
def test_a_transaction_takes_the_time_its_bytes_take(baud):
    delay, tx, rx = 300e-6, 0.7e-3, 1.3e-3
    b = Bench(baud=baud, response_delay_s=delay, usb_tx_latency_s=tx, usb_rx_latency_s=rx)
    for s in b.servos:
        s._set("baud_rate", P.BAUD_RATES.index(baud))
    byte = 10 / baud
    b.driver.read(3, 56, 2)
    assert b.t == pytest.approx(tx + 8 * byte + delay + 8 * byte + rx, rel=1e-6), "READ: 8 bytes out, 8 back"
    t0 = b.t
    b.driver.sync_read(56, 2, [1, 2, 3, 4, 5, 6])
    assert b.t - t0 == pytest.approx(tx + 14 * byte + 6 * (delay + 8 * byte) + rx, rel=1e-6), "SYNC_READ of six"
    t0 = b.t
    b.driver.sync_write(42, {i: b"\x00\x08" for i in range(1, 7)})
    assert b.t == t0, "a sync write does not wait for anything"


@pytest.mark.parametrize("fault", ["drop_reply_prob", "drop_command_prob", "corrupt_prob"])
@pytest.mark.parametrize("seed", range(3))
def test_under_faults_the_driver_returns_checked_data_or_raises(fault, seed):
    b = Bench(seed=seed, **{fault: 0.2})
    for k, s in enumerate(b.servos):
        s.control((k - 3) * 0.3, 0.0, 0.002)                     # six different encoder readings
    truth = {s.id: bytes(s.mem[56:58]) for s in b.servos}
    ok = failed = 0
    for _ in range(300):
        try:
            got = b.driver.sync_read(56, 2, list(truth))
            assert got == truth, "a wrong value came back as if it were good"
            ok += 1
        except BusError:
            failed += 1
    st = b.driver.stats
    assert ok > 30 and st["retries"] > 0, "the fault was not injected, or retries never helped"
    assert st["failures"] == failed and st["timeouts"] >= failed
    if fault == "corrupt_prob":
        assert st["checksum_errors"] > 0


def test_a_late_reply_is_not_taken_for_the_next_answer():
    """Timeout shorter than the round trip: the reply to request 1 arrives during request 2."""
    b = Bench(usb_rx_latency_s=5e-3)
    b.driver.timeout, b.driver.retries = 3e-3, 0
    b.servos[0].control(0.5, 0.0, 0.002)
    with pytest.raises(BusError):
        b.driver.read(1, 56, 2)
    with pytest.raises(BusError):                                # the stale reply has servo 1's id: must not be accepted
        b.driver.read(2, 56, 2)


def test_an_echoing_adapter_needs_the_echo_option():
    b = Bench(echo=True)
    b.driver.echo = True
    assert b.driver.ping(4) == 0
    assert b.driver.sync_read(56, 2, [1, 2]).keys() == {1, 2}


# ---------------------------------------------------------------- set-up mistakes
def six_servos(ids=(1, 2, 3, 4, 5, 6)):
    m = ServoMap(JOINTS)
    return [ServoEmulator(i, ServoSpec.from_config(), mount_zero_ticks=z, mount_direction=int(d))
            for i, z, d in zip(ids, m.zero, m.direction)]


def test_new_servos_that_all_have_id_1_are_reported_before_any_write():
    with pytest.raises(BusError, match="id 1"):
        BusBackend(load_backend("mujoco"), servos=six_servos((1, 1, 1, 1, 1, 1)))


def test_a_missing_servo_is_named():
    servos = six_servos()
    del servos[3]
    with pytest.raises(BusError, match="wrist_flex"):
        BusBackend(load_backend("mujoco"), servos=servos + six_servos((9,)))


def test_a_servo_at_another_baud_rate_is_named():
    servos = six_servos()
    servos[4].nvs[R.BY_NAME["baud_rate"].addr] = 4               # 115200
    servos[4].power_cycle()
    with pytest.raises(BusError, match="wrist_roll"):
        BusBackend(load_backend("mujoco"), servos=servos)


def test_a_different_register_layout_stops_connect_before_any_write():
    """A servo whose register 5 is not its id (another control table): refuse, and write nothing."""
    servos = six_servos()
    odd = servos[2]
    real_read = odd._read
    odd._read = lambda addr, n: bytes([77] * n) if addr <= 5 < addr + n else real_read(addr, n)
    settings = lambda: [[s.get(r.name) for r in R.REGISTERS if r.writable] for s in servos]   # noqa: E731
    before = settings()
    with pytest.raises(BusError, match="does not look like"):
        BusBackend(load_backend("mujoco"), servos=servos)
    assert settings() == before, "something was written"


# ---------------------------------------------------------------- bus in the loop
@pytest.fixture(scope="module")
def pair():
    return load_backend("mujoco"), BusBackend(load_backend("mujoco"))


def test_reset_holds_the_pose_and_time_starts_at_zero(pair):
    _, bus = pair
    bus.reset(HOME)
    assert bus.time == 0.0 and np.allclose(bus.true_q(), HOME, atol=1e-9), "start-up traffic moved the arm"
    assert np.abs(bus.q() - HOME).max() <= TICK / 2 + 1e-9, "encoders read to within half a tick"
    assert 0 < bus.time < 0.02, "reading the encoders takes bus time"
    bus.step(int(0.5 / bus.dt))
    assert np.abs(bus.true_q() - HOME).max() < 0.08, "only gravity droop allowed"
    assert not bus.wire.warnings(), bus.wire.warnings()


@pytest.mark.parametrize("seed", range(3))
def test_the_arm_moves_the_same_with_the_bus_in_the_loop(pair, seed):
    """The same command stream through direct joint targets and through the bus: the same motion.

    Differences allowed: one tick of quantisation and deadband, and the few ms the commands spend on the bus.
    """
    direct, bus = pair
    rng = np.random.default_rng(seed)
    lo, hi = direct.m.actuator_ctrlrange[direct.act].T
    goal = np.clip(HOME + rng.uniform(-0.5, 0.5, 6) * [1, 0.5, 0.5, 0.5, 1, 0.5], lo + 0.1, hi - 0.1)
    worst = 0.0
    for sim in (direct, bus):
        sim.reset(HOME)
    for k in range(int(1.5 / direct.dt)):
        cmd = HOME + (goal - HOME) * min(1.0, k * direct.dt / 0.6)        # a 0.6 s ramp, then hold
        for sim in (direct, bus):
            sim.set_targets(cmd)
            sim.step(1)
        worst = max(worst, np.abs(direct.q() - bus.true_q()).max())
    assert worst < 0.03, f"paths differ by {worst:.3f} rad"
    assert np.abs(direct.q() - bus.true_q()).max() < 0.006, "resting poses differ by more than a few ticks"


def test_the_gripper_torque_limit_is_what_the_controller_wrote(pair):
    _, bus = pair
    bus.reset(HOME)
    limit = servo_limits()["gripper_torque_nm"]
    assert bus.servos[5].get("max_torque_limit") == round(1000 * load_servos()["follower"]["gripper_torque_limit_frac"])
    # Drive the jaw into its closed stop. Written to the servo directly: through set_targets the command
    # filter would (rightly) stop at the joint limit and the servo would never push.
    bus.arm.write_goals(HOME + [0, 0, 0, 0, 0, -1.0])
    bus.step(int(0.6 / bus.dt))
    assert abs(bus.d.actuator_force[bus.act][5]) == pytest.approx(limit, rel=0.02)
    assert bus.servos[0].get("max_torque_limit") == 1000


def test_feedback_registers_match_the_physics(pair):
    _, bus = pair
    bus.reset(HOME)
    bus.set_targets(HOME + [0.8, 0, 0, 0, 0, 0])
    bus.step(int(0.15 / bus.dt))
    t0 = bus.time
    fb = bus.arm.read_feedback()
    dof = bus.dof
    # The reading is as old as the transaction is long: the arm has moved on by speed x that time.
    stale = servo_limits()["no_load_speed_rad_s"] * (bus.time - t0) + 2 * TICK
    assert (np.abs(fb["position"] - bus.true_q()) <= stale).all()
    assert 0.003 < bus.time - t0 < 0.008, "six 15-byte replies at 1 Mbaud plus the USB latencies"
    # (the reading is a few ms old and the arm is still accelerating: up to 40 rad/s2 x 5 ms = 0.2 rad/s)
    assert fb["speed"][0] == pytest.approx(bus.d.qvel[dof][0], abs=0.35) and fb["speed"][0] > 0.5
    assert fb["voltage"] == pytest.approx(load_servos()["follower"]["supply_voltage_v"], abs=0.05)
    spec = ServoSpec.from_config()
    tau = np.abs(bus.d.actuator_force[bus.act])
    assert fb["current"] == pytest.approx(0.1 + 2.9 * tau / spec.stall_torque, abs=0.15)
    assert not fb["status"].any() and fb["moving"][0]


def test_stiffness_comes_from_the_one_config_value(pair):
    """Hold a pose: droop through the bus equals droop with the generated model's actuators (same sat_error_deg)."""
    direct, bus = pair
    for sim in (direct, bus):
        sim.reset(HOME)
        sim.set_targets(HOME)
        sim.step(int(1.0 / sim.dt))
    droop_direct, droop_bus = direct.q() - HOME, bus.true_q() - HOME
    assert np.abs(droop_direct).max() > 0.01, "expected visible droop at the configured stiffness"
    assert droop_bus == pytest.approx(droop_direct, abs=0.004)
    assert position_loop()["sat_error_deg"] == bus.servos[0].spec.sat_error_deg


# ---------------------------------------------------------------- teleop through the bus
def grasp_and_lift(rig):
    app = rig.app
    cube0 = app.sim.body_pos("cube")
    tip, roll, q_open = rig.grasp_pose()
    assert app.set_cartesian([tip[0], tip[1], 0.09], roll=roll, grip=q_open)
    rig.run(2.5)
    # Down in a straight line, as a click or the wheel does. (A single 8 cm jump of the target makes the
    # joints race each other and the jaw tip dip under the table top on the way; the command filter
    # cuts that short, and the detour it leaves can clip the cube.)
    assert app.move_to(tip, "descend", roll=roll)
    rig.run(2.0)
    app.toggle_grip()
    rig.run(3.0)
    assert app.grip_latch.holding, "grip-by-offset did not engage"
    assert app.set_cartesian(tip + [0, 0, 0.10])
    rig.run(2.5)
    return app.sim.body_pos("cube")[2] - cube0[2]


@pytest.mark.parametrize("faults", [{}, {"drop_reply_prob": 0.05, "drop_command_prob": 0.05, "corrupt_prob": 0.05}],
                         ids=["clean bus", "5 % faults"])
def test_teleop_grasps_and_lifts_through_the_bus(faults):
    from harness import Rig
    rig = Rig(bus={"seed": 1})
    sim = rig.app.sim
    for name, p in faults.items():               # faults start once the arm is connected and holding
        setattr(sim.wire, name, p)
    assert grasp_and_lift(rig) > 0.08, "cube not lifted"
    rig.mon.assert_ok()
    assert not any(s.tripped for s in sim.servos), "a servo went into overload protection"
    assert not sim.wire.warnings(), f"the controller wrote something a servo ignores: {sim.wire.warnings()[:3]}"
    assert sim.driver.stats["failures"] == sim.read_failures
    if faults:
        assert sim.driver.stats["retries"] > 0, "no fault was injected"
    else:
        assert sim.driver.stats["retries"] == 0 and sim.read_failures == 0
    used = (sim.wire.bytes_tx + sim.wire.bytes_rx) * sim.wire.byte_time / sim.time
    assert used < 0.5, f"the controller uses {used:.0%} of the wire's capacity"


def test_the_trace_shows_what_was_on_the_wire(pair):
    _, bus = pair
    bus.reset(HOME)
    bus.q()
    text = bus.report(trace=8)
    assert "SYNC_READ all: 2 bytes at 56 ids [1, 2, 3, 4, 5, 6]" in text and "servo 6" in text
