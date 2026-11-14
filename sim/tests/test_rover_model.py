"""The rover: its CAD, and the simulated rover generated from it.

Aimed at classes of bugs:
  * a part that is not one printable solid, or two parts that run into each other somewhere in the
    axes' travel (the CAD script's own report, and again in the physics by sweeping every axis);
  * the simulated rover not being the designed one: masses, the reach of each axis, where the lenses
    and the tags are, all come from one place and are compared;
  * a rover that does not stand, roll or steer as its wheel angles say (odometry depends on it):
    rolling without slip, turning by the wheels' difference over the track, straight when asked;
  * a head whose coupled joints drift apart (the head must stay level; the cameras must move evenly);
  * a lip that does not hold the rover.
"""
import json
import math

import mujoco
import numpy as np
import pytest

from hub import builtin
from so101sim import scenes
from so101sim.config import ROOT
from so101sim.rover_model import STOW, assembly

ROVER = ROOT / "hardware" / "rover"
REPORT = json.loads((ROVER / "build_report.json").read_text())
ASM = assembly()
R_WHEEL, TRACK = ASM["wheel_radius_m"], ASM["track_m"]


def bare(at=(0.4, 0.3), facing=0.0, outline=None):
    """A bare surface with the rover on it and nothing else."""
    s = scenes.read("no_arm")
    s["objects"], s["devices"] = [], [d for d in s["devices"] if d["kind"] == "rover"] + [d for d in s["devices"] if d["kind"] == "camera"][:1]
    s.pop("fixtures", None)
    s.pop("tasks", None)
    rv = next(d for d in s["devices"] if d["kind"] == "rover")
    rv["at_m"], rv["facing_deg"] = list(at), facing
    if outline:
        s["surface"]["outline_m"] = outline
    b = scenes.build(s)
    return b, b.model, b.new_data()


def q(m, d, joint):
    return float(d.qpos[m.joint(joint).qposadr[0]])


def pose(m, d):
    i = m.body("rover").id
    R = d.xmat[i].reshape(3, 3)
    return d.xpos[i].copy(), math.atan2(R[1, 0], R[0, 0]), R


def drive(m, d, wl, wr, seconds):
    """Turn the wheels at wl, wr [rad/s] by moving their servos' targets, then let it settle."""
    al, ar = m.actuator("rover_wheel_left").id, m.actuator("rover_wheel_right").id
    for _ in range(int(seconds / m.opt.timestep)):
        d.ctrl[al] += wl * m.opt.timestep
        d.ctrl[ar] += wr * m.opt.timestep
        mujoco.mj_step(m, d)
    for _ in range(250):
        mujoco.mj_step(m, d)


def go(m, d, joint, target, seconds=1.2, each=None):
    a = m.actuator(joint).id
    start = d.ctrl[a]
    n = int(seconds / m.opt.timestep)
    for k in range(n):
        d.ctrl[a] = start + (target - start) * min(1.0, (k + 1) / (0.7 * n))
        mujoco.mj_step(m, d)
        if each is not None and k % 10 == 0:
            each()


# ---------------------------------------------------------------- the CAD
def test_every_printed_part_is_one_solid_and_has_its_files():
    assert REPORT["all_parts_one_solid"] and len(REPORT["parts"]) >= 12
    for name, part in REPORT["parts"].items():
        assert part["solids"] == 1 and part["volume_mm3"] > 100, name
        for ext in ("step", "stl"):
            assert (ROVER / "parts" / f"{name}.{ext}").stat().st_size > 200, f"{name}.{ext}"


def test_no_two_parts_meet_anywhere_in_the_axes_travel():
    check = REPORT["interference_mm3"]
    assert check["poses_checked"] >= 90 and check["pairs_that_overlap"] == {}


def test_the_leads_between_the_axes_are_long_enough():
    for name, lead in REPORT["wires"].items():
        if isinstance(lead, dict):
            assert lead["ok"] and lead["needs_mm"] <= lead["stock_lead_mm"], name


def test_its_weight_sits_between_the_wheels_and_the_caster():
    """Wheels at x = 0, y = +/-65; caster at x = 128. Stowed, raised, or turned to the side, the centre of mass
    is inside that triangle; turned right round it rests on the rear skid (x = -42), which is what the skid is for."""
    com = REPORT["centre_of_mass_mm"]
    for key in ("head stowed", "head raised", "head stowed, turned 90 deg, cameras apart"):
        x, y, _ = com[key]
        assert 5 < x < 123 and abs(y) < 65 * (1 - x / 128) - 3, (key, com[key])
    assert -42 < com["head stowed, turned 170 deg"][0] < 10


# ---------------------------------------------------------------- the simulated rover is the designed one
def test_the_simulated_rover_weighs_what_the_cad_says():
    _, m, _ = bare()
    sim = sum(m.body_mass[i] for i in range(m.nbody) if m.body(i).name.startswith("rover"))
    assert abs(sim * 1000 - REPORT["total_mass_g"]) < 0.5
    for b in ASM["bodies"]:
        name = "rover" if b["parent"] is None else f"rover_{b['name']}"
        assert abs(m.body(name).mass[0] - b["mass_kg"]) < 1e-6 and np.allclose(m.body(name).ipos, b["com_m"], atol=1e-6), name
        assert (m.body(name).inertia > 0).all(), name


def test_the_hub_believes_the_rovers_tags_are_where_the_model_has_them():
    _, m, d = bare()
    tags = builtin.tags("rover")
    assert sorted(tags) == [10, 11, 12, 13, 14]
    for i, tag in tags.items():
        g = m.geom(f"tag_{i}")
        assert m.body(m.geom_bodyid[g.id]).name == ("rover" if tag.on == "chassis" else f"rover_{tag.on}")
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, g.quat)
        R = R.reshape(3, 3)
        assert np.allclose(R, tag.T_part_tag[:3, :3], atol=1e-6) and np.allclose(g.pos - R[:, 2] * 0.0004, tag.T_part_tag[:3, 3], atol=1e-6), i
        # ... and each lies on the outside of something solid: 1 mm behind its face is inside a collision shape
        body = m.geom_bodyid[g.id]
        p = d.xpos[body] + d.xmat[body].reshape(3, 3) @ (tag.T_part_tag[:3, 3] - tag.T_part_tag[:3, 2] * 0.001)
        inside = any(np.all(np.abs(d.geom_xmat[k].reshape(3, 3).T @ (p - d.geom_xpos[k])) <= m.geom_size[k] + 5e-4)
                     for k in range(m.ngeom) if m.geom_bodyid[k] == body and m.geom_contype[k] and m.geom_type[k] == mujoco.mjtGeom.mjGEOM_BOX)
        assert inside, f"tag {i} hangs in the air"


def test_the_lenses_are_where_the_cad_puts_them_and_look_ahead():
    _, m, d = bare(facing=0.0)
    left, right = (m.camera(f"rover_{s}").id for s in ("left", "right"))
    lo_h, hi_h = REPORT["ranges"]["lens_height_mm"]
    assert abs(d.cam_xpos[left][2] * 1000 - lo_h) < 0.5, "stowed, the lens is at the CAD's lowest height"
    assert abs(np.linalg.norm(d.cam_xpos[left] - d.cam_xpos[right]) * 1000 - REPORT["ranges"]["lens_spacing_mm"][0]) < 0.1
    for c in (left, right):
        look = -d.cam_xmat[c].reshape(3, 3)[:, 2]
        assert look[0] > 0.999 and abs(d.cam_xmat[c].reshape(3, 3)[2, 1] - 1) < 1e-6, "cameras look along the rover's x, the right way up"
    assert d.cam_xpos[left][1] > d.cam_xpos[right][1], "the left camera is on the left"


# ---------------------------------------------------------------- standing, rolling, steering
def test_it_stands_level_on_its_wheels_and_caster():
    _, m, d = bare()
    for _ in range(1500):
        mujoco.mj_step(m, d)
    p, _, R = pose(m, d)
    assert abs(p[2]) < 5e-4 and abs(math.degrees(math.asin(R[2, 0]))) < 0.3 and abs(math.degrees(math.asin(R[2, 1]))) < 0.3
    touching = {m.geom(c.geom1).name for c in d.contact[:d.ncon]} | {m.geom(c.geom2).name for c in d.contact[:d.ncon]}
    assert {"rover_chassis_caster"} <= touching and any("wheel_left" in n for n in touching) and any("wheel_right" in n for n in touching)
    assert "rover_chassis_skid" not in touching, "the rear skid is clear of the surface when the rover stands level"
    assert np.abs(d.qvel).max() < 1e-3


@pytest.mark.parametrize("speed", [1.5, 3.0, 5.0])
@pytest.mark.parametrize("facing", [0.0, 135.0])
def test_it_rolls_as_far_as_its_wheels_turn_and_straight(speed, facing):
    _, m, d = bare(at=(0.35, 0.3) if facing == 0 else (0.6, 0.2), facing=facing)
    p0, y0, _ = pose(m, d)
    drive(m, d, speed, speed, 1.2)
    p1, y1, _ = pose(m, d)
    ahead = np.array([math.cos(y0), math.sin(y0)])
    moved = p1[:2] - p0[:2]
    assert abs(moved @ ahead - R_WHEEL * q(m, d, "rover_wheel_left")) < 0.002, "rolling without slip"
    assert abs(moved @ [-ahead[1], ahead[0]]) < 0.002 and abs(math.degrees(y1 - y0)) < 0.5, "straight"


@pytest.mark.parametrize("speed", [1.0, 2.0, -2.0])
def test_it_turns_on_the_spot_by_what_its_wheels_say(speed):
    _, m, d = bare()
    p0, y0, _ = pose(m, d)
    drive(m, d, -speed, speed, 1.5)
    p1, y1, _ = pose(m, d)
    expected = R_WHEEL * (q(m, d, "rover_wheel_right") - q(m, d, "rover_wheel_left")) / TRACK
    assert abs((y1 - y0) / expected - 1) < 0.04, f"turned {math.degrees(y1 - y0):.1f} deg, the wheels say {math.degrees(expected):.1f}"
    assert np.linalg.norm(p1[:2] - p0[:2]) < 0.006, "the middle of the axle stays put"


def test_the_lip_holds_it():
    """Head raised out of the way, driven at the lip for four seconds: it stops against it and stays on the surface."""
    b, m, d = bare(at=(0.55, 0.3), facing=0.0)
    go(m, d, "rover_lift", math.radians(80))
    drive(m, d, 5.0, 5.0, 4.0)
    p, _, R = pose(m, d)
    edge = max(x for x, _ in b.outline)
    assert p[0] + 0.136 < edge + 0.004, "the chassis went through or over the lip"
    assert abs(p[2]) < 0.004 and abs(math.degrees(math.asin(R[2, 0]))) < 3.0, "it climbed the lip"


# ---------------------------------------------------------------- the head
AXES = [("rover_pan", "pan_deg", math.radians), ("rover_lift", "lift_deg", math.radians), ("rover_tilt", "tilt_deg", math.radians)]


def test_every_axis_reaches_its_range_and_nothing_touches_on_the_way():
    """Sweep pan, tilt, spacing and lift through their whole ranges (head up, so the surface is not in the way).
    No part of the rover touches another, the head stays level, and the cameras move apart evenly."""
    _, m, d = bare()
    rover = {i for i in range(m.nbody) if m.body(i).name.startswith("rover")}
    worst = {"depth": 0.0, "level": 0.0, "even": 0.0}

    def watch():
        for c in d.contact[:d.ncon]:
            if m.geom_bodyid[c.geom1] in rover and m.geom_bodyid[c.geom2] in rover:
                worst["depth"] = min(worst["depth"], c.dist)
        worst["level"] = max(worst["level"], abs(q(m, d, "rover_level") + q(m, d, "rover_lift")))
        worst["even"] = max(worst["even"], abs(q(m, d, "rover_spacing_right") - q(m, d, "rover_spacing")))
    go(m, d, "rover_lift", math.radians(45), each=watch)
    for joint, key, conv in AXES[:1] + AXES[2:]:
        lo, hi = (conv(v) for v in REPORT["ranges"][key])
        for target in (hi, lo, 0.0):
            go(m, d, joint, target, seconds=2.0, each=watch)
            if target != 0.0:
                assert abs(q(m, d, joint) - target) < math.radians(2.0), f"{joint} did not reach {math.degrees(target):.0f} deg"
    travel = (REPORT["ranges"]["lens_spacing_mm"][1] - REPORT["ranges"]["lens_spacing_mm"][0]) / 2000
    go(m, d, "rover_spacing", travel, seconds=2.5, each=watch)
    left, right = (m.camera(f"rover_{s}").id for s in ("left", "right"))
    assert abs(np.linalg.norm(d.cam_xpos[left] - d.cam_xpos[right]) * 1000 - REPORT["ranges"]["lens_spacing_mm"][1]) < 1.0
    go(m, d, "rover_spacing", 0.0, seconds=2.5, each=watch)
    lo, hi = (math.radians(v) for v in REPORT["ranges"]["lift_deg"])
    go(m, d, "rover_lift", hi, seconds=2.0, each=watch)
    assert abs(d.cam_xpos[left][2] * 1000 - REPORT["ranges"]["lens_height_mm"][1]) < 3.0
    go(m, d, "rover_lift", lo, seconds=2.0, each=watch)
    assert worst["depth"] > -5e-4, f"parts of the rover ran {-worst['depth'] * 1000:.1f} mm into each other"
    assert math.degrees(worst["level"]) < 0.5 and worst["even"] < 3e-4, worst


def test_it_starts_stowed_and_holds_that_when_left_alone():
    _, m, d = bare()
    assert abs(q(m, d, "rover_lift") - STOW["lift"]) < 1e-9 and abs(q(m, d, "rover_spacing")) < 1e-9
    for _ in range(1000):
        mujoco.mj_step(m, d)
    assert abs(q(m, d, "rover_lift") - STOW["lift"]) < math.radians(1.5), "the head sagged"
    assert abs(q(m, d, "rover_pan")) < math.radians(0.5) and abs(q(m, d, "rover_tilt")) < math.radians(1.5)
