"""The rover under control: motor mode, the radio link, the battery, driving by odometry, the head, stopping.

Aimed at classes of bugs:
  * the hub's idea of the rover differing from the rover: its own geometry (cameras, tags, lowest point)
    is compared with the referee at random head poses, and what it reports with what happened;
  * odometry that does not count what the wheels did (turns lost when the position wraps, a wheel
    counted backwards, a heading that grows the wrong way);
  * motion that should have been refused: driving with the head up, a head sent into the ground,
    anything at all on a low battery;
  * a rover that keeps going when nobody is talking to it: the link down, the hub stopped, torque off;
  * a link that is slow being treated as dead, or dead being treated as slow.
"""
import json
import math

import mujoco
import numpy as np
import pytest

from hub import api as A
from hub import geometry as G
from hub.bus import BusError, FeetechBus
from hub.bus import hx_registers as R
from hub.hub import Hub
from hub.rover.description import STOWED, Rover
from so101sim import scenes
from so101sim.bus import ServoEmulator
from so101sim.world import BOARD_WATCHDOG_S, RADIO, World, cell_voltage

RAD_PER_TICK = 2 * math.pi / 4096


def call(hub, method, path, body=None):
    status, ctype, out = A.dispatch(hub, method, path, b"" if body is None else json.dumps(body).encode())
    return status, (json.loads(out) if ctype == "application/json" else out)


def bare(at=(0.4, 0.3), facing=0.0, fixtures=None, size=None, **rover):
    """A bare surface (0.8 x 0.6 m unless size is given) with the rover and two cameras."""
    s = scenes.read("no_arm")
    if size:
        s["surface"]["outline_m"] = [[0.0, 0.0], [size[0], 0.0], [size[0], size[1]], [0.0, size[1]]]
    s["objects"] = []
    s.pop("tasks")
    s.pop("fixtures")
    if fixtures:
        s["fixtures"] = fixtures
    next(d for d in s["devices"] if d["kind"] == "rover").update(at_m=list(at), facing_deg=facing, **rover)
    return s


def truth(world):
    m, d = world.model, world.data
    i = m.body("rover").id
    R_ = d.xmat[i].reshape(3, 3)
    return d.xpos[i][:2].copy(), math.atan2(R_[1, 0], R_[0, 0])


def turned(a, b):
    return math.degrees((b - a + math.pi) % (2 * math.pi) - math.pi)


@pytest.fixture()
def system():
    world = World(bare())
    return world, Hub(world.ports)


# ---------------------------------------------------------------- the servo in motor mode
def motor(servo_id=11):
    s = ServoEmulator(servo_id)
    for name, value in (("operating_mode", 1), ("torque_switch", 1)):
        r = R.BY_NAME[name]
        s._write(r.addr, r.encode(value))
    return s


def run(s, q, qd, seconds, load=0.0, inertia=0.03):
    """A shaft with some inertia and a constant load torque, driven by the servo."""
    for _ in range(int(seconds / 0.002)):
        qd += (s.control(q, qd, 0.002) - load) / inertia * 0.002
        q += qd * 0.002
    return q, qd


@pytest.mark.parametrize("steps", [200, 1000, 2500])
@pytest.mark.parametrize("direction", [1, -1])
def test_motor_mode_holds_the_speed_it_is_given_under_load(steps, direction):
    s = motor()
    speed = R.BY_NAME["motion_speed"]
    s._write(speed.addr, bytes([steps & 0xFF, (steps >> 8) | (0x80 if direction < 0 else 0)]))
    want = direction * steps * RAD_PER_TICK
    for load in (0.0, 0.3 * direction):
        _, qd = run(s, 0.0, 0.0, 0.6, load=load)
        assert abs(qd - want) < 0.03 * abs(want) + 0.01, f"asked {want:.2f} rad/s, got {qd:.2f} against a load of {load} N.m"
    assert abs(s.get("current_speed") * RAD_PER_TICK - qd) < 0.02


def test_motor_mode_reads_its_position_within_one_turn_and_stops_at_zero():
    s = motor()
    speed = R.BY_NAME["motion_speed"]
    s._write(speed.addr, bytes([0xD0, 0x07]))                # 2000 steps/s
    q, qd = run(s, 0.0, 0.0, 3.0)
    assert q > 2 * math.pi, "it should have gone round more than once"
    assert 0 <= s.get("current_position") <= 4095
    s._write(speed.addr, b"\x00\x00")
    q, qd = run(s, q, qd, 0.5)
    assert abs(qd) < 0.01
    over = bytes([0xFF, 0x7F])                               # 32767 steps/s forward: more than it has
    s._write(speed.addr, over)
    assert s.get("motion_speed") == 3400


# ---------------------------------------------------------------- the radio link and the rover's own board
def test_a_slow_link_is_not_a_dead_one_and_a_dead_one_is_not_slow():
    world = World(bare())
    port = world.ports().serial[0]
    bus = FeetechBus(port, reply_timeout_s=0.02 + port.reply_time_s, retries=1)
    t0 = world.time
    for _ in range(60):
        assert bus.read(11, 5, 1) == b"\x0b"
    per = (world.time - t0) / 60
    assert 2 * RADIO["delay_s"] < per < 0.05, f"a transaction took {per * 1000:.1f} ms"
    assert bus.stats["failures"] == 0
    world.buses[0].link_up = False
    with pytest.raises(BusError):
        bus.read(11, 5, 1)
    world.buses[0].link_up = True
    assert bus.read(11, 5, 1) == b"\x0b"


def test_the_rovers_board_stops_the_wheels_when_the_hub_goes_quiet(system):
    world, hub = system
    rover, bus = hub.rover, world.buses[0]
    rover._wheels(0.06, 0.0)
    world.step(0.2)
    assert all(abs(v) > 1.0 for v in world.data.qvel[bus.dof[list(bus.wheels)]]), "the wheels should be turning"
    world.step(BOARD_WATCHDOG_S - 0.25)
    assert bus.board_stops == 0, "the board stopped the wheels before its time was up"
    world.step(0.5)
    assert bus.board_stops == 1 and np.abs(world.data.qvel[bus.dof[list(bus.wheels)]]).max() < 0.05


def test_the_link_going_down_mid_drive_stops_the_rover_and_is_said_so(system):
    world, hub = system
    bus = world.buses[0]
    cut_at, advance = world.t + 0.8, world.advance_to

    def cutting(t):
        advance(t)
        if world.t > cut_at:
            bus.link_up = False
    world.advance_to = cutting
    status, r = call(hub, "POST", "/devices/rover/drive/forward", {"distance_m": 0.3})
    world.advance_to = advance
    assert status == 503 and "not answered" in r["error"] and "discover" in r["error"]
    world.step(1.0)
    assert np.abs(world.data.qvel[bus.dof[list(bus.wheels)]]).max() < 0.05 and bus.board_stops >= 1
    bus.link_up = True
    assert "rover" in call(hub, "POST", "/discover")[1]["devices"]
    assert call(hub, "POST", "/devices/rover/drive/forward", {"distance_m": 0.03})[1]["reached"]


# ---------------------------------------------------------------- the battery
def test_a_cell_runs_from_full_to_empty_and_the_servos_see_the_pack():
    assert cell_voltage(1.0) == pytest.approx(4.2) and cell_voltage(0.0) == pytest.approx(3.0)
    assert all(cell_voltage(a) > cell_voltage(b) for a, b in zip((1.0, 0.6, 0.2, 0.06), (0.6, 0.2, 0.06, 0.0)))
    for charge in (0.9, 0.3, 0.03):
        world = World(bare(battery_charge=charge))
        hub = Hub(world.ports)
        v = call(hub, "GET", "/devices/rover/power/state")[1]
        assert abs(v["volts"] - 3 * cell_voltage(charge)) < 0.25 and v["low"] == (v["volts"] < 9.9)


def test_nothing_moves_on_a_low_battery():
    world = World(bare(battery_charge=0.03))
    hub = Hub(world.ports)
    before = truth(world)
    for path, body in (("/devices/rover/drive/forward", {"distance_m": 0.05}), ("/devices/rover/drive/turn", {"angle_deg": 30}),
                       ("/devices/rover/head/raise", {"lens_height_m": 0.1}), ("/devices/rover/head/point", {"pan_deg": 20})):
        status, r = call(hub, "POST", path, body)
        assert status == 409 and "battery" in r["error"] and "V" in r["error"]
    world.step(0.5)
    assert np.allclose(truth(world)[0], before[0], atol=1e-4)


# ---------------------------------------------------------------- the hub's idea of the rover is the rover
@pytest.mark.parametrize("seed", range(5))
def test_the_hub_knows_where_the_rovers_cameras_and_tags_are(seed):
    world = World(bare(facing=40.0))
    m, d = world.model, world.data
    rng = np.random.default_rng(seed)
    rover = Rover()
    q = {"pan": rng.uniform(-2.9, 2.9), "lift": rng.uniform(-0.2, 1.5), "tilt": rng.uniform(-1.0, 0.5), "spacing": rng.uniform(0, 0.07)}
    for j, v in q.items():
        d.qpos[m.joint(f"rover_{j}").qposadr[0]] = v
    d.qpos[m.joint("rover_level").qposadr[0]], d.qpos[m.joint("rover_spacing_right").qposadr[0]] = -q["lift"], q["spacing"]
    mujoco.mj_forward(m, d)
    b = m.body("rover").id
    T = G.pose(d.xmat[b].reshape(3, 3), d.xpos[b])
    for side in ("left", "right"):
        assert np.allclose(T @ rover.camera(side, q), world.camera_pose(f"rover_{side}"), atol=1e-9)
    for i, corners in rover.tag_corners(q).items():
        g = m.geom(f"tag_{i}").id
        centre = d.geom_xpos[g] - d.geom_xmat[g].reshape(3, 3)[:, 2] * 0.0004
        assert np.linalg.norm((T @ np.append(corners.mean(axis=0), 1.0))[:3] - centre) < 1e-9
    lowest = min(float((d.geom_xpos[g] + d.geom_xmat[g].reshape(3, 3) @ (m.geom_size[g] * s))[2])
                 for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX and m.geom_contype[g]
                 and m.body(m.geom_bodyid[g]).name in ("rover_arm", "rover_head_block", "rover_head", "rover_carriage_left", "rover_carriage_right")
                 for s in np.array([[a, b_, c] for a in (-1, 1) for b_ in (-1, 1) for c in (-1, 1)]))
    assert abs(rover.lowest(q) - lowest) < 1e-9
    assert abs(rover.lens_spacing(q) - np.linalg.norm(world.camera_pose("rover_left")[:3, 3] - world.camera_pose("rover_right")[:3, 3])) < 1e-9


# ---------------------------------------------------------------- driving
@pytest.mark.parametrize("distance", [0.05, 0.15, -0.10, 0.40])
def test_it_drives_the_distance_asked_and_its_wheels_count_it(distance):
    """0.40 m is more than two turns of a wheel: the count must survive the position wrapping round."""
    world = World(bare(at=(0.12, 0.3) if distance > 0 else (0.45, 0.3)))      # (room ahead for the stowed head, 245 mm past the axle)
    hub = Hub(world.ports)
    p0, h0 = truth(world)
    r = call(hub, "POST", "/devices/rover/drive/forward", {"distance_m": distance})[1]
    p1, h1 = truth(world)
    went = float((p1 - p0) @ [math.cos(h0), math.sin(h0)])
    assert r["reached"] and abs(went - distance) < 0.004 + 0.02 * abs(distance)
    assert abs(r["moved"]["forward_m"] - went) < 0.002 + 0.01 * abs(distance), "the wheels' count is not what it did"
    assert abs(turned(h0, h1)) < 1.0 and abs(float((p1 - p0) @ [-math.sin(h0), math.cos(h0)])) < 0.004, "not straight"
    s = call(hub, "GET", "/devices/rover/drive/state")[1]["since_taken_over"]
    assert abs(s["x_m"] - went) < 0.004 and abs(s["y_m"]) < 0.004


@pytest.mark.parametrize("angle", [90, -45, 180, -200])
def test_it_turns_the_angle_asked_on_the_spot(angle, system):
    world, hub = system
    p0, h0 = truth(world)
    r = call(hub, "POST", "/devices/rover/drive/turn", {"angle_deg": angle})[1]
    p1, h1 = truth(world)
    actual = min((turned(h0, h1) + k for k in (-360, 0, 360)), key=lambda a: abs(a - angle))
    assert r["reached"] and abs(actual - angle) < 4.0, (actual, r["moved"])
    assert abs(r["moved"]["turned_deg"] - actual) < 1.0 + 0.025 * abs(angle) and np.linalg.norm(p1 - p0) < 0.008


def test_a_square_comes_back_to_where_it_began_by_its_own_count_and_nearly_in_truth():
    world = World(bare(at=(0.6, 1.0), size=(1.4, 1.6)))      # (room for the head to swing clear of the lip and the cameras' posts)
    hub = Hub(world.ports)
    p0, h0 = truth(world)
    for _ in range(4):
        assert call(hub, "POST", "/devices/rover/drive/forward", {"distance_m": 0.12})[1]["reached"]
        assert call(hub, "POST", "/devices/rover/drive/turn", {"angle_deg": 90})[1]["reached"]
    p1, h1 = truth(world)
    s = call(hub, "GET", "/devices/rover/drive/state")[1]["since_taken_over"]
    drift = float(np.linalg.norm(p1 - p0))
    believed = math.hypot(s["x_m"], s["y_m"])
    assert drift < 0.03 and abs(turned(h0, h1)) < 8.0, f"after a square it is {drift * 1000:.0f} mm and {turned(h0, h1):.1f} deg off"
    assert abs(believed - drift) < 0.02, "what it believes and what happened have come apart"


def test_it_will_not_drive_with_its_head_up_and_stow_puts_that_right(system):
    world, hub = system
    assert call(hub, "POST", "/devices/rover/head/raise", {"lens_height_m": 0.12})[1]["reached"]
    before = truth(world)
    status, r = call(hub, "POST", "/devices/rover/drive/forward", {"distance_m": 0.05})
    assert status == 409 and "stow" in r["error"] and np.allclose(truth(world)[0], before[0], atol=1e-4)
    assert call(hub, "POST", "/devices/rover/head/stow")[1]["reached"]
    assert call(hub, "POST", "/devices/rover/drive/forward", {"distance_m": 0.05})[1]["reached"]


# ---------------------------------------------------------------- the head
def test_the_head_goes_where_it_is_sent_and_reports_what_happened(system):
    world, hub = system
    m, d = world.model, world.data
    r = call(hub, "POST", "/devices/rover/head/raise", {"lens_height_m": 0.12})[1]
    assert r["reached"] and abs(d.cam_xpos[m.camera("rover_left").id][2] - 0.12) < 0.006
    assert abs(r["state"]["lens_height_m"] - d.cam_xpos[m.camera("rover_left").id][2]) < 0.003
    r = call(hub, "POST", "/devices/rover/head/point", {"pan_deg": -70, "tilt_deg": -35})[1]
    assert r["reached"] and abs(math.degrees(d.qpos[m.joint("rover_pan").qposadr[0]]) + 70) < 2 and abs(math.degrees(d.qpos[m.joint("rover_tilt").qposadr[0]]) + 35) < 2
    r = call(hub, "POST", "/devices/rover/head/spread", {"lens_spacing_m": 0.18})[1]
    gap = np.linalg.norm(d.cam_xpos[m.camera("rover_left").id] - d.cam_xpos[m.camera("rover_right").id])
    assert r["reached"] and abs(gap - 0.18) < 0.003 and abs(r["state"]["lens_spacing_m"] - gap) < 0.002
    assert abs(d.qpos[m.joint("rover_level").qposadr[0]] + d.qpos[m.joint("rover_lift").qposadr[0]]) < math.radians(0.5), "the head is not level"


def test_the_head_is_never_sent_into_the_surface_it_stands_on(system):
    world, hub = system
    m, d = world.model, world.data
    r = call(hub, "POST", "/devices/rover/head/point", {"tilt_deg": -55})[1]          # stowed low: tilting down would hit the ground
    assert not r["reached"] and "surface" in r["why_not_reached"]
    world.step(0.5)
    head = {i for i in range(m.nbody) if m.body(i).name in ("rover_head", "rover_carriage_left", "rover_carriage_right", "rover_head_block")}
    floor = {g for g in range(m.ngeom) if m.geom(g).name.startswith(("table", "lip_"))}
    assert not any((c.geom1 in floor and m.geom_bodyid[c.geom2] in head) or (c.geom2 in floor and m.geom_bodyid[c.geom1] in head)
                   for c in d.contact[:d.ncon]), "the head is on the ground"
    assert call(hub, "POST", "/devices/rover/head/raise", {"lens_height_m": 0.13})[1]["reached"]
    assert call(hub, "POST", "/devices/rover/head/point", {"tilt_deg": -55})[1]["reached"], "raised, it may look down"


@pytest.mark.parametrize("seed", range(4))
def test_stow_from_anywhere_ends_stowed_without_touching_the_ground(seed, system):
    world, hub = system
    m, d = world.model, world.data
    rng = np.random.default_rng(seed)
    call(hub, "POST", "/devices/rover/head/raise", {"lens_height_m": float(rng.uniform(0.08, 0.16))})
    call(hub, "POST", "/devices/rover/head/point", {"pan_deg": float(rng.uniform(-160, 160)), "tilt_deg": float(rng.uniform(-50, 25))})
    call(hub, "POST", "/devices/rover/head/spread", {"lens_spacing_m": float(rng.uniform(0.07, 0.2))})
    low = [1.0]
    advance = world.advance_to

    def watching(t):
        advance(t)
        low[0] = min(low[0], min(float(d.geom_xpos[g][2] - np.abs(d.geom_xmat[g].reshape(3, 3) @ np.diag(m.geom_size[g])).sum(axis=1)[2])
                                 for g in range(m.ngeom) if "_collision" in m.geom(g).name
                                 and m.geom(g).name.startswith(("rover_carriage", "rover_head_c"))))
    world.advance_to = watching
    r = call(hub, "POST", "/devices/rover/head/stow")[1]
    world.advance_to = advance
    assert r["reached"] and r["state"]["stowed"] and low[0] > 0.003, f"lowest point on the way: {low[0] * 1000:.1f} mm"
    for j, want in STOWED.items():
        assert abs(d.qpos[m.joint(f"rover_{j}").qposadr[0]] - want) < (0.004 if j == "spacing" else math.radians(4))


def test_a_head_joint_that_cannot_follow_stops_pushing(system):
    """Jam the pan at 25 degrees (the world narrows the joint's range behind the hub's back). The head gives up,
    says which joint, and is not left straining against it."""
    world, hub = system
    call(hub, "POST", "/devices/rover/head/raise", {"lens_height_m": 0.1})
    world.model.jnt_range[world.model.joint("rover_pan").id] = [math.radians(-170), math.radians(25)]
    r = call(hub, "POST", "/devices/rover/head/point", {"pan_deg": 110})[1]
    assert not r["reached"] and "pan" in r["why_not_reached"] and abs(r["state"]["pan_deg"] - 25) < 4
    world.step(0.5)
    pan = next(s for s in world.buses[0].servos if s.id == 21)
    assert abs(pan.torque) < 0.3 * pan.torque_limit_nm, "it is still pushing against the stop"


def test_the_heads_servos_are_held_to_a_third_of_their_torque(system):
    world, _ = system
    for s in world.buses[0].servos:
        assert s.get("max_torque_limit") == (350 if s.id >= 21 else 1000), s.id
        assert s.get("operating_mode") == (1 if s.id in (11, 12) else 0)


# ---------------------------------------------------------------- stopping
def test_stop_at_the_rover_stops_wheels_and_holds_the_head(system):
    world, hub = system
    bus = world.buses[0]
    hub.rover._wheels(0.06, 0.2)
    world.step(0.3)
    assert call(hub, "POST", "/devices/rover/stop")[1]["stopped"] == ["devices/rover"]
    world.step(0.4)
    assert np.abs(world.data.qvel[bus.dof]).max() < 0.05
    hub.rover._wheels(0.06, 0.0)
    world.step(0.2)
    assert "devices/rover" in call(hub, "POST", "/stop")[1]["stopped"]
    world.step(0.4)
    assert np.abs(world.data.qvel[bus.dof[list(bus.wheels)]]).max() < 0.05


def test_shutdown_leaves_every_rover_servo_with_torque_off(system):
    world, hub = system
    assert "torque off on every rover servo" in hub.shutdown()
    assert all(s.get("torque_switch") == 0 for s in world.buses[0].servos)
    status, r = call(hub, "POST", "/devices/rover/drive/forward", {"distance_m": 0.05})
    assert status == 409 and "torque" in r["error"]


# ---------------------------------------------------------------- its place in the tree
def test_the_rover_is_in_the_tree_with_its_parts_and_groups(system):
    _, hub = system
    tree = call(hub, "GET", "/tree")[1]["tree"]["under_it"]
    rover = tree["devices"]["under_it"]["rover"]
    assert set(rover["under_it"]) == {"drive", "head", "power"} and set(rover["under_it"]["head"]["under_it"]) == {"left", "right"}
    assert set(rover["under_it"]["drive"]["commands"]) == {"forward", "turn"}
    assert set(rover["under_it"]["head"]["commands"]) == {"point", "raise", "spread", "stow"}
    groups = tree["groups"]["under_it"]
    assert {"/devices/rover/head/left", "/devices/rover/head/right"} <= set(groups["cameras"]["members"])
    assert groups["power"]["members"] == ["/devices/rover/power"] and "/devices/rover" in groups["movers"]["members"]
    status, ctype, out = A.dispatch(hub, "GET", "/devices/rover/head/left/picture?width=320")
    assert status == 200 and ctype == "image/jpeg"
    state = call(hub, "GET", "/devices/rover/state")[1]
    assert {"drive", "head", "power"} <= set(state) and state["head"]["stowed"] is True
    guide = call(hub, "GET", "/")[1].decode()
    for words in ("/devices/rover/drive/forward", "/devices/rover/head/spread", "drift", "stowed"):
        assert words in guide
