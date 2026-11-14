"""Physical invariants: what the model, the command filter and the physics monitor each guarantee.

Aimed at classes of bugs:
  * a collision shape that is not where the part is: contacts where the real parts are clear (phantom
    friction), or none where they are solid (the base);
  * soft or thin contacts: the arm sinking into the table, objects passing through it;
  * a command that should never reach the servos: not finite, outside the limits, faster than a servo
    can move, or asking for a pose under the table or inside the base, from any source and backend;
  * the monitor itself: every kind of violation is caught when provoked, and ordinary runs are clean.
"""
import math

import mujoco
import numpy as np
import pytest

from so101sim import JOINTS, load_backend
from so101sim.bus_backend import BusBackend
from so101sim.config import WORKCELL, load_safety, servo_limits
from so101sim.ik import ArmIK
from so101sim.monitor import PhysicsMonitor, PhysicsViolation
from so101sim.safety import CommandFilter

HOME = np.array([0.0, -0.6, 0.9, 0.9, -1.57, 0.5])
ROBOT = {"base", "shoulder", "upper_arm", "lower_arm", "wrist", "gripper", "moving_jaw_so101_v1", "wrist_camera_assembly"}
STALL = servo_limits()["stall_torque_nm"]


class Model:
    """The workcell model with raw access (no backend, no command filter)."""

    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path(str(WORKCELL))
        self.d = mujoco.MjData(self.m)
        self.qadr = np.array([self.m.joint(j).qposadr[0] for j in JOINTS])
        self.dof = np.array([self.m.joint(j).dofadr[0] for j in JOINTS])
        self.lo = np.array([self.m.joint(j).range[0] for j in JOINTS])
        self.hi = np.array([self.m.joint(j).range[1] for j in JOINTS])

    def set(self, q):
        mujoco.mj_resetData(self.m, self.d)
        self.d.qpos[self.qadr] = q
        self.d.ctrl[:6] = q
        mujoco.mj_forward(self.m, self.d)

    def body_of(self, g):
        return self.m.body(self.m.geom_bodyid[g]).name

    def contacts(self):
        d = self.d
        return [(self.body_of(d.contact.geom1[k]), self.body_of(d.contact.geom2[k]), float(d.contact.dist[k]))
                for k in range(d.ncon)]

    def touches_table(self):
        t = self.m.geom("table").id
        return any(t in (self.d.contact.geom1[k], self.d.contact.geom2[k]) and self.d.contact.dist[k] < 0
                   for k in range(self.d.ncon))

    def true_mesh(self, g):
        """World-frame mesh of a visual geom, from the compiled model (MuJoCo re-centres meshes on load)."""
        import trimesh
        m, d = self.m, self.d
        mid = m.geom_dataid[g]
        v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        f = m.mesh_face[m.mesh_faceadr[mid]:m.mesh_faceadr[mid] + m.mesh_facenum[mid]]
        return trimesh.Trimesh((d.geom_xmat[g].reshape(3, 3) @ v.T).T + d.geom_xpos[g], f, process=False)

    def visual_geoms(self, body):
        m = self.m
        return [g for g in range(m.ngeom) if self.body_of(g) == body and m.geom_contype[g] == 0
                and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH]

    def true_clearance(self, body_a, body_b, n=200):
        """Smallest distance between the real (visual) meshes of two bodies [m]; negative if they intersect."""
        import trimesh
        pts = []
        for g in self.visual_geoms(body_a):
            w = self.true_mesh(g)
            pts += [w.vertices[::max(1, len(w.vertices) // n)], trimesh.sample.sample_surface(w, n, seed=0)[0]]
        pts = np.vstack(pts)
        return min(-float(trimesh.proximity.signed_distance(self.true_mesh(g), pts).max()) for g in self.visual_geoms(body_b))


@pytest.fixture(scope="module")
def model():
    return Model()


# ---------------------------------------------------------------- collision shapes are where the parts are
def test_wrist_roll_is_free_over_its_whole_range(model):
    """The wrist camera bracket passes close to the wrist link. Their collision shapes must not touch
    anywhere in the roll range (the real parts clear by 0.7 mm or more), at any pose of the other joints."""
    rng = np.random.default_rng(0)
    for other in [HOME, np.zeros(6)] + [rng.uniform(model.lo, model.hi) for _ in range(4)]:
        for roll in np.linspace(model.lo[4], model.hi[4], 80):
            q = np.array(other, float)
            q[4] = roll
            model.set(q)
            hits = [c for c in model.contacts() if {c[0], c[1]} == {"wrist", "wrist_camera_assembly"}]
            assert not hits, f"camera mount touches the wrist link at roll {roll:+.2f} (depth {min(h[2] for h in hits) * 1000:.2f} mm)"


def test_wrist_roll_needs_the_same_torque_everywhere(model):
    """A slow roll sweep costs only the motor's own losses: no stretch of the range has extra friction."""
    m, d = model.m, model.d
    q = HOME.copy()
    q[4] = model.lo[4]
    model.set(q)
    n = int((model.hi[4] - model.lo[4]) / 0.5 / m.opt.timestep)          # 0.5 rad/s
    torque = []
    for k in range(n):
        d.ctrl[4] = model.lo[4] + (model.hi[4] - model.lo[4]) * k / n
        mujoco.mj_step(m, d)
        if k > n // 10:
            torque.append(abs(d.actuator_force[4]))
    expected = 0.5 * m.dof_damping[model.dof[4]] + m.dof_frictionloss[model.dof[4]]      # back-EMF at 0.5 rad/s + friction
    assert max(torque) < expected + 0.15, f"peak {max(torque):.2f} N.m in the sweep, expected about {expected:.2f}"


def test_contacts_are_reported_only_where_the_real_parts_are_close(model):
    """Random poses: wherever the simulation reports the arm touching itself, the real meshes are within 3 mm."""
    pytest.importorskip("rtree")
    rng = np.random.default_rng(2)
    checked, phantom = 0, []
    while checked < 10:
        q = rng.uniform(model.lo, model.hi)
        model.set(q)
        if model.touches_table():
            continue
        pairs = {tuple(sorted((a, b))) for a, b, dist in model.contacts() if a in ROBOT and b in ROBOT and dist < 0}
        for a, b in sorted(pairs)[:2]:
            checked += 1
            gap = model.true_clearance(a, b)
            if gap > 0.003:
                phantom.append((a, b, round(gap * 1000, 1), np.round(q, 2).tolist()))
    assert not phantom, f"contacts where the real parts are more than 3 mm apart: {phantom}"


def test_the_base_stands_on_the_table_and_is_solid(model):
    m, d = model.m, model.d
    model.set(np.array([1.9, -1.7, 1.6, 1.0, -1.57, 0.5]))              # arm swung out of the way
    lowest = min(float(model.true_mesh(g).vertices[:, 2].min()) for g in model.visual_geoms("base"))
    assert lowest == pytest.approx(0.0, abs=1e-4), f"the base's lowest point is at z = {lowest * 1000:.2f} mm"
    assert sum(1 for g in range(m.ngeom) if model.body_of(g) == "base" and m.geom_contype[g]) >= 4
    # Drop a cube on the base in several places: it must land on it (touch it) and never sink into it.
    base_geoms = {g for g in range(m.ngeom) if model.body_of(g) == "base" and m.geom_contype[g]}
    cube = m.geom("cube").id
    a = m.joint("cube_free").qposadr[0]
    for x, y in ((-0.02, 0.0), (-0.015, 0.035), (0.03, 0.045)):
        model.set(np.array([1.9, -1.7, 1.6, 1.0, -1.57, 0.5]))
        d.qpos[a:a + 3] = [x, y, 0.13]
        mujoco.mj_forward(m, d)
        touched, deepest = False, 0.0
        for _ in range(500):
            mujoco.mj_step(m, d)
            for k in range(d.ncon):
                pair = {int(d.contact.geom1[k]), int(d.contact.geom2[k])}
                if cube in pair and pair & base_geoms:
                    touched, deepest = True, min(deepest, float(d.contact.dist[k]))
        assert touched, f"a cube dropped at ({x}, {y}) fell past the base without touching it"
        assert deepest > -0.004, f"a cube dropped at ({x}, {y}) sank {-deepest * 1000:.1f} mm into the base"


# ---------------------------------------------------------------- nothing sinks into or passes through the table
def test_the_arm_at_full_torque_does_not_sink_into_the_table(model):
    m, d = model.m, model.d
    ik = ArmIK()
    high, _ = ik.solve([0.25, 0.0, 0.15], HOME)
    low, _ = ik.solve([0.25, 0.0, -0.10], high)                          # 10 cm under the table top
    model.set(high)
    d.ctrl[:6] = low
    table, deepest = m.geom("table").id, 0.0
    for _ in range(1500):
        mujoco.mj_step(m, d)
        for k in range(d.ncon):
            if table in (d.contact.geom1[k], d.contact.geom2[k]):
                deepest = min(deepest, float(d.contact.dist[k]))
    assert np.abs(d.actuator_force[:5]).max() > 0.9 * STALL, "the arm was not pushing at full torque"
    assert deepest > -load_safety()["monitor"]["max_penetration_m"], f"sank {-deepest * 1000:.1f} mm into the table"


@pytest.mark.parametrize("speed", [2, 5, 10, 20, 40])
def test_no_object_passes_through_the_table(model, speed):
    m, d = model.m, model.d
    model.set(HOME)
    a, v = m.joint("cube_free").qposadr[0], m.joint("cube_free").dofadr[0]
    d.qpos[a:a + 3] = [0.3, 0.2, 0.3]
    d.qvel[v + 2] = -speed
    mujoco.mj_forward(m, d)
    for _ in range(600):
        mujoco.mj_step(m, d)
    assert d.qpos[a + 2] > 0.01, f"a cube thrown down at {speed} m/s ended at z = {d.qpos[a + 2] * 1000:.0f} mm"


def test_a_cube_pressed_by_the_gripper_is_not_pushed_into_the_table(model):
    m, d = model.m, model.d
    ik = ArmIK()
    model.set(HOME)
    for _ in range(200):
        mujoco.mj_step(m, d)
    cube = d.body("cube").xpos.copy()
    above, _ = ik.solve([cube[0], cube[1], 0.06], HOME)
    below, _ = ik.solve([cube[0], cube[1], -0.02], above)
    above[5] = below[5] = -0.17
    model.set(above)
    d.ctrl[:6] = below
    a = m.joint("cube_free").qposadr[0]
    lowest = 1.0
    for _ in range(1500):
        mujoco.mj_step(m, d)
        if abs(d.qpos[a] - cube[0]) < 0.03 and abs(d.qpos[a + 1] - cube[1]) < 0.03:      # while it is still under the gripper
            lowest = min(lowest, float(d.qpos[a + 2]))
    assert lowest > 0.015 - 0.002, f"the cube's centre went down to {lowest * 1000:.1f} mm (rests at 15.0)"


# ---------------------------------------------------------------- the command filter
def lowest_point(f: CommandFilter, q) -> float:
    return f.table_z - f.depth(q)[0] if f.depth(q)[1] == "table" else f.table_z


@pytest.mark.parametrize("seed", range(6))
def test_no_request_stream_gets_an_impossible_command_through(seed):
    """Steps, ramps, noise, out-of-range values, NaN and inf: what comes out is always sendable."""
    rng = np.random.default_rng(seed)
    f = CommandFilter(HOME)
    dt = 0.002
    prev_q, prev_v = f.q.copy(), np.zeros(6)
    request = HOME.copy()
    for k in range(3000):
        r = rng.random()
        if r < 0.02:
            request = rng.uniform(f.lo - 1.0, f.hi + 1.0)                 # a jump, possibly outside the limits
        elif r < 0.03:
            request = np.where(rng.random(6) < 0.3, rng.choice([np.nan, np.inf, -np.inf]), request)
        elif r < 0.05:
            request = HOME + rng.normal(0, 0.5, 6)
        elif r < 0.3:
            request = np.nan_to_num(request, nan=0.0, posinf=1.0, neginf=-1.0) + rng.normal(0, 0.002, 6)
        q = f.update(request, dt)
        v = (q - prev_q) / dt
        assert np.isfinite(q).all()
        assert (q >= f.lo - 1e-9).all() and (q <= f.hi + 1e-9).all(), "outside the joint limits"
        # One tick never moves a joint further than the catch-up speed plus what the request itself moved smoothly.
        assert np.abs(v).max() <= f.v_max + f.v_jump + 1e-6
        depth, what = f.depth(f.intended)
        assert depth <= f.table_tol + 1e-6, f"commanded {depth * 1000:.2f} mm into the {what} keep-out"
        assert np.allclose(f.intended, q, atol=1e-9), "with no feed-forward the intended pose is the command"
        prev_q, prev_v = q, v
    assert {"jump", "joint_limit", "nonfinite"} <= set(f.interventions), f.interventions


def test_a_smooth_stream_passes_through_unchanged_and_a_step_becomes_a_planned_move():
    up = np.array([0.0, -1.0, 0.9, 0.9, -1.57, 0.5])                      # well above the table
    f = CommandFilter(up)
    dt = 0.002
    for k in range(1500):                                                 # a fast, jittery but continuous stream
        req = up + [0.8 * math.sin(6 * k * dt), 0, 0.3 * math.sin(9 * k * dt), 0, 0, 0] + 0.002 * np.sin(1000 * k * dt)
        assert np.array_equal(f.update(req, dt), np.clip(req, f.lo, f.hi))        # bit for bit
    assert not f.interventions
    f = CommandFilter(HOME)
    goal = HOME + [1.2, 0, 0, 0, 0, 0]
    qs = np.array([f.update(goal, dt) for _ in range(1000)])
    v = np.diff(qs[:, 0]) / dt
    assert np.allclose(qs[-1], goal, atol=1e-9), "the step is not reached in the end"
    assert v.max() <= f.v_max + 1e-9 and np.abs(np.diff(v)).max() <= f.a_max * dt + 1e-9 and (qs[:, 0] <= goal[0] + 1e-9).all()
    assert 0.3 < np.argmax(np.abs(qs[:, 0] - goal[0]) < 1e-9) * dt < 0.6, "about 0.4 s for 1.2 rad at 5.4 rad/s and 40 rad/s2"
    assert set(f.limited_joints) == {"shoulder_pan"}


def test_the_filter_stops_at_the_table_and_at_the_base_and_lets_the_arm_back_out():
    ik = ArmIK()
    f = CommandFilter(HOME)
    under, _ = ik.solve([0.25, 0.0, -0.08], HOME)
    for _ in range(1500):
        q = f.update(under, 0.002)
    assert f.interventions["table"] > 0
    assert f.depth(q)[0] <= f.table_tol + 1e-9 and f.depth(q)[0] > -0.004, "should stop just above the table"
    into_base = np.array([0.0, -1.2, 1.69, 1.65, -1.57, 0.3])            # hand folded down onto the base
    f = CommandFilter(HOME)
    assert f.depth(into_base)[0] > 0.003, "the test pose is not inside a keep-out"
    for _ in range(1500):
        q = f.update(into_base, 0.002)
    assert f.depth(q)[0] <= f.table_tol + 1e-9
    f.reset(under)                                                        # starts inside (as after a bad reset)
    assert f.depth(f.q)[0] > 0.01
    for _ in range(1500):
        q = f.update(HOME, 0.002)
    assert np.allclose(q, HOME, atol=1e-6), "a pose inside a keep-out must be allowed to move out"


@pytest.mark.parametrize("kind", ["mujoco", "bus"])
def test_every_backend_sends_its_commands_through_the_filter(kind):
    """A raw step far outside what is possible, straight into set_targets: the servos get a limited ramp,
    and the arm ends above the table."""
    sim = load_backend("mujoco")
    if kind == "bus":
        sim = BusBackend(sim)
    mon = PhysicsMonitor(sim)
    inner = getattr(sim, "inner", sim)
    ik = ArmIK()
    under, _ = ik.solve([0.25, 0.0, -0.08], HOME)
    sim.reset(HOME)
    sim.set_targets(under + [3.0, 0, 0, 0, 0, 0])                         # also far past the pan limit
    f = sim.safety
    goals, dt = [], sim.dt
    for _ in range(800):
        sim.step(1)
        goals.append(f.q.copy())
    goals = np.array(goals)
    v = np.diff(goals, axis=0) / dt
    assert np.abs(v).max() <= f.v_max + 1e-6, "the step reached the servos faster than the governor's speed cap"
    assert (goals <= f.hi + 1e-9).all() and f.interventions["joint_limit"] and f.interventions["table"] and f.interventions["jump"]
    if kind == "bus":                                                     # what the servos were told is the filter's output
        told = sim.arm.map.ticks_to_rad([s.get("target_position") for s in sim.servos])
        assert np.abs(told - goals[-1]).max() < 2 * math.pi / 4096
    table = inner.m.geom("table").id
    deepest = min([float(inner.d.contact.dist[k]) for k in range(inner.d.ncon)
                   if table in (inner.d.contact.geom1[k], inner.d.contact.geom2[k])], default=0.0)
    assert deepest > -0.001, f"the arm is {-deepest * 1000:.1f} mm into the table"
    mon.assert_ok()


def test_ordinary_teleop_never_touches_the_filter_or_trips_the_monitor():
    from harness import Rig
    rig = Rig()
    app = rig.app
    tip, roll, q_open = rig.grasp_pose()
    assert app.set_cartesian([tip[0], tip[1], 0.09], roll=roll, grip=q_open)
    rig.run(2.5)
    assert app.set_cartesian(tip, roll=roll)
    rig.run(2.0)
    app.toggle_grip()
    rig.run(2.0)
    assert app.set_cartesian(tip + [0, 0, 0.10])
    rig.run(2.0)
    rig.click_table((0.30, -0.10))
    rig.run(2.5)
    f = app.sim.safety
    # Allowed: the two deliberate steps in teleop's commands (feed-forward switching on at the first tick,
    # the grip latch), each smoothed over a few ticks; and the table keep-out during the fast descent,
    # whose planned path asks the fixed jaw's tip to dip about 3 mm under the table top on the way down.
    # (joint_limit: closing the gripper asks for a little past its closed stop.)
    assert set(f.interventions) <= {"jump", "table", "joint_limit"}, f"the filter changed a well-formed command: {f.interventions}"
    assert f.interventions["jump"] < 400 and f.interventions["table"] < 150, f.interventions
    rig.mon.assert_ok()
    w = app.monitor.worst
    assert w["penetration_m"] < 0.005 and w["joint_speed_rad_s"] < servo_limits()["no_load_speed_rad_s"]


# ---------------------------------------------------------------- the monitor catches what it should
@pytest.mark.parametrize("kind", ["penetration", "joint speed", "joint acceleration", "object speed", "workcell", "finite"])
def test_the_monitor_catches_each_kind_of_violation(kind):
    sim = load_backend("mujoco", safety=False)
    sim.reset(HOME)
    mon = PhysicsMonitor(sim)
    m, d = sim.m, sim.d
    a, v = m.joint("cube_free").qposadr[0], m.joint("cube_free").dofadr[0]
    sim.step(50)
    assert not mon.violations, mon.violations
    if kind == "penetration":
        d.qpos[a:a + 3] = [0.3, 0.2, 0.005]                               # a cube 10 mm into the table
        mujoco.mj_forward(m, d)
        mon.check()
    elif kind == "joint speed":
        d.qvel[sim.m.joint("shoulder_pan").dofadr[0]] = 12.0
    elif kind == "joint acceleration":
        m.actuator_forcerange[:, :] = [-60, 60]                           # a motor twenty times too strong
        m.actuator_gainprm[:, 0] *= 60
        m.actuator_biasprm[:, 1] *= 60
        sim.set_targets(HOME + [1.5, 0, 0, 0, 0, 0])
    elif kind == "object speed":
        d.qpos[a + 2] = 0.3
        d.qvel[v] = 8.0
    elif kind == "workcell":
        d.qpos[a:a + 3] = [0.3, 0.2, -0.2]
    else:
        d.qvel[0] = np.nan
    try:
        sim.step(40)
    except Exception:                                                     # noqa: BLE001  (MuJoCo may warn on NaN)
        pass
    assert kind in mon.kinds, f"not caught; saw {mon.violations}"
    with pytest.raises(PhysicsViolation):
        mon.assert_ok()


def test_strict_mode_stops_at_the_first_violation():
    sim = load_backend("mujoco", safety=False)
    sim.reset(HOME)
    PhysicsMonitor(sim, strict=True)
    a = sim.m.joint("cube_free").qposadr[0]
    sim.d.qpos[a:a + 3] = [0.3, 0.2, -0.2]
    with pytest.raises(PhysicsViolation, match="overlap|passed through the table"):
        sim.step(5)


@pytest.mark.parametrize("seed", range(4))
def test_every_thing_the_filter_does_is_recorded_as_an_event_with_true_numbers(seed):
    """Random command streams, well-formed and not. Whatever the filter did: the events account for every tick
    it counted, every event can be put into a sentence, and the numbers in a joint-limit sentence are the
    command's own."""
    import re
    from so101sim.safety import CommandFilter
    rng = np.random.default_rng(seed)
    f = CommandFilter(np.zeros(6))
    q, dt = np.zeros(6), 0.002
    for k in range(1500):
        q = q + rng.normal(0, 0.004, 6)
        if rng.random() < 0.01:
            q = q + rng.normal(0, 0.8, 6)                    # a jump
        if rng.random() < 0.01:
            j = int(rng.integers(0, 5))
            q[j] = f.hi[j] + rng.uniform(0.02, 0.5)          # past a limit
        req = q.copy()
        if rng.random() < 0.003:
            req[int(rng.integers(0, 6))] = np.nan
        out = f.update(req, dt)
        assert np.isfinite(out).all()
        q = np.clip(q, f.lo - 1.0, f.hi + 1.0)
    assert f.interventions, "the stream never needed the filter: the test tests nothing"
    for reason, ticks in f.interventions.items():
        mine = [e for e in f.events if e["reason"] == reason]
        assert sum(e["ticks"] for e in mine) == ticks and len(mine) == f.started[reason]
        assert all(abs(e["seconds"] - e["ticks"] * dt) < 1e-9 and e["last_tick"] - e["first_tick"] + 1 >= e["ticks"] for e in mine)
    seen_limit = 0
    for e in f.events:
        text = f.describe(e)
        assert len(text) > 30 and "nan" not in text.lower() and "None" not in text, text
        m = re.match(r"(\w+) was asked to go to ([-\d.]+) deg, which is ([-\d.]+) deg past its limit of ([-\d.]+) deg", text)
        if m:
            asked, over, limit = float(m.group(2)), float(m.group(3)), float(m.group(4))
            assert abs(abs(asked - limit) - over) <= 0.11, text
            j = JOINTS.index(m.group(1))
            assert min(abs(limit - np.degrees(f.lo[j])), abs(limit - np.degrees(f.hi[j]))) <= 0.06, text
            seen_limit += 1
    assert seen_limit and {"jump", "joint_limit", "nonfinite"} <= set(f.started)
    assert all(k in f.report() for k in f.started)
