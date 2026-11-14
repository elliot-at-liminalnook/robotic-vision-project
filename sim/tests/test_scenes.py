"""Scenes as files: every scene and task in sim/scenes/ is valid, and the validator catches what it should.

This is the small suite that stands for "created and validated": no hub, no rover controller, nothing
driven. It builds each scene in memory, checks placing, lets it sit for a second, and checks its tasks.

Aimed at classes of bugs:
  * a scene file that is not a place that can exist (bad fields, things past the lip, overlaps,
    things not at rest, tasks about things that are not there or that are already done);
  * a validator that passes such a file: each kind of fault is provoked and must be named;
  * a surface whose lip has a gap, for any right-angled outline;
  * a file that changes when it is read and written back;
  * a build that depends on what was built before it.
"""
import copy

import mujoco
import numpy as np
import pytest

from so101sim import scenes
from so101sim.config import MODEL_DIR
from so101sim.scenes import geometry as G
from so101sim.scenes import tasks as T
from so101sim.scenes import toml_io

NAMES = scenes.names()


def test_there_are_scenes():
    assert {"workcell", "open_small"} <= set(NAMES)


# ---------------------------------------------------------------- every file
@pytest.mark.parametrize("name", NAMES)
def test_scene_is_valid(name):
    scene = scenes.read(name)
    assert scene["name"] == name, "a scene's name is its file's name"
    assert scenes.validate(scene) == []


@pytest.mark.parametrize("name", NAMES)
def test_scene_survives_being_written_and_read_back(name):
    scene = scenes.read(name)
    assert toml_io.loads(toml_io.dumps(scene)) == scene


@pytest.mark.parametrize("name", NAMES)
def test_the_lip_runs_all_the_way_round(name):
    """Walk the outline 3 mm outside it: every point is inside a lip box, as high as the file says."""
    built = scenes.build(scenes.read(name))
    m, d = built.model, built.new_data()
    lips = [g for g in range(m.ngeom) if m.geom(g).name.startswith("lip_")]
    h = built.scene["surface"]["lip_height_m"]
    ccw = G.signed_area(built.outline) > 0
    for a, b in G.edges(built.outline):
        e = (b - a) / np.linalg.norm(b - a)
        out = np.array([e[1], -e[0]]) if ccw else np.array([-e[1], e[0]])
        for s in np.linspace(0, 1, 60):
            p = np.array([*(a + s * (b - a) + out * 0.003), h - 0.001])
            covered = any(np.all(np.abs(d.geom_xmat[g].reshape(3, 3).T @ (p - d.geom_xpos[g])) <= m.geom_size[g] + 1e-9) for g in lips)
            assert covered, f"gap in the lip at {p[:2]}"


@pytest.mark.parametrize("name", NAMES)
def test_every_task_starts_undone_and_reads_as_a_request(name):
    built = scenes.build(scenes.read(name))
    d = built.new_data()
    for task in built.scene.get("tasks", []):
        assert len(task["request"].split()) >= 4, "a request is a sentence for the model"
        results = T.check_task(built, d, task, answers={})
        assert not all(r["done"] for r in results), task["name"]
        for r in results:
            assert isinstance(r["detail"], str) and r["detail"]


@pytest.mark.parametrize("name", NAMES)
def test_building_is_repeatable_and_leaves_no_trace(name, tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    scene = scenes.read(name)
    before = copy.deepcopy(scene)
    a, b = scenes.build(scene), scenes.build(scene)
    assert scene == before, "building changed the scene it was given"
    assert a.model.nq == b.model.nq and a.model.ngeom == b.model.ngeom
    assert np.array_equal(a.model.geom_size, b.model.geom_size) and np.array_equal(a.model.body_pos, b.model.body_pos)
    assert not list(tmp_path.iterdir()), "building wrote a file"


# ---------------------------------------------------------------- the old workcell, in the new form
def test_workcell_is_todays_model_plus_a_lip():
    old = mujoco.MjModel.from_xml_path(str(MODEL_DIR / "workcell.xml"))
    new = scenes.build(scenes.read("workcell")).model
    assert (old.nq, old.nu, old.ncam) == (new.nq, new.nu, new.ncam)
    for i in range(old.nbody):
        o, n = old.body(i), new.body(old.body(i).name)
        for attr in ("pos", "quat", "mass", "inertia"):
            assert np.allclose(getattr(o, attr), getattr(n, attr), atol=1e-7), (o.name, attr)
    for i in range(old.ngeom):
        o = old.geom(i)
        if not o.name or o.name == "floor":
            continue
        n = new.geom(o.name)
        for attr in ("size", "pos", "friction", "contype", "conaffinity", "condim", "solref", "solimp"):
            assert np.allclose(getattr(o, attr), getattr(n, attr), atol=1e-9), (o.name, attr)
    for i in range(old.njnt):
        o, n = old.joint(i), new.joint(old.joint(i).name)
        assert np.allclose(o.range, n.range) and np.allclose(o.damping, n.damping), o.name
    for i in range(old.nu):
        o, n = old.actuator(i), new.actuator(old.actuator(i).name)
        assert np.allclose(o.gainprm, n.gainprm) and np.allclose(o.biasprm, n.biasprm) and np.allclose(o.forcerange, n.forcerange)
    for i in range(old.ncam):
        o, n = old.camera(i), new.camera(old.camera(i).name)
        assert np.allclose(o.pos, n.pos, atol=1e-9) and np.allclose(o.fovy, n.fovy)
        assert abs(abs(float(np.dot(o.quat, n.quat))) - 1) < 1e-6, o.name
    extra = {new.geom(g).name for g in range(new.ngeom)} - {old.geom(g).name for g in range(old.ngeom)}
    assert {e for e in extra if not e.startswith(("lip_", "scene_"))} == set(), extra


# ---------------------------------------------------------------- outlines
@pytest.mark.parametrize("seed", range(20))
def test_any_right_angled_outline_is_tiled_exactly(seed):
    """Random staircase outlines: the rectangles cover the outline's area, do not overlap, and stay inside."""
    rng = np.random.default_rng(seed)
    n = int(rng.integers(1, 5))
    xs = np.cumsum(rng.uniform(0.1, 0.4, n + 1))
    hs = rng.uniform(0.2, 0.8, n)
    pts = [[xs[0], 0.0]]
    for k in range(n):
        pts += [[xs[k], hs[k]], [xs[k + 1], hs[k]]]
    pts += [[xs[-1], 0.0]]
    outline = [p for i, p in enumerate(pts) if i == 0 or p != pts[i - 1]]
    assert G.is_right_angled(outline)
    rects = G.rectangles(outline)
    assert abs(sum((r[2] - r[0]) * (r[3] - r[1]) for r in rects) - G.area(outline)) < 1e-9
    for i, a in enumerate(rects):
        assert G.inside(outline, (a[0] + a[2]) / 2, (a[1] + a[3]) / 2)
        for b in rects[i + 1:]:
            assert min(a[2], b[2]) - max(a[0], b[0]) <= 1e-9 or min(a[3], b[3]) - max(a[1], b[1]) <= 1e-9, "overlap"


# ---------------------------------------------------------------- the validator catches each kind of fault
def _base():
    return scenes.read("open_small")


def _with_task(scene, goals, **more):
    scene["tasks"] = [{"name": "t", "request": "Do the thing that is asked.", "goals": goals, **more}]
    return scene


def _mutations():
    def unknown_field(s):
        s["surface"]["colour_of_money"] = "green"
    def missing_field(s):
        del s["surface"]["lip_height_m"]
    def out_of_range(s):
        s["objects"][0]["mass_kg"] = 50.0
    def bad_colour(s):
        s["objects"][0]["colour"] = "ultraviolet"
    def twice_named(s):
        s["objects"][1]["name"] = s["objects"][0]["name"]
    def slanted_outline(s):
        s["surface"]["outline_m"] = [[0, 0], [0.7, 0.1], [0.7, 0.55], [0, 0.55]]
    def object_past_the_lip(s):
        s["objects"][0]["at_m"] = [0.695, 0.3]
    def rover_past_the_lip(s):
        next(d for d in s["devices"] if d["kind"] == "rover")["at_m"] = [0.66, 0.12]
    def arm_past_the_lip(s):
        next(d for d in s["devices"] if d["kind"] == "arm")["at_m"] = [0.01, 0.27]
    def two_things_in_one_place(s):
        s["objects"][1]["at_m"] = [s["objects"][0]["at_m"][0] + 0.01, s["objects"][0]["at_m"][1]]
    def thing_in_mid_air(s):
        s["objects"][0]["at_m"] = [0.38, 0.33, 0.15]
    def stands_on_nothing(s):
        s["objects"][0]["on"] = "the_moon"
    def task_about_nothing(s):
        _with_task(s, [{"kind": "upright", "object": "unicorn"}])
    def task_needs_a_missing_device(s):
        s["devices"] = [d for d in s["devices"] if d["kind"] != "rover"]
        _with_task(s, [{"kind": "upright", "object": "red_cube"}, {"kind": "answer_number", "key": "n", "truth": "count_objects"}],
                   needs=["rover"])
    def task_already_done(s):
        _with_task(s, [{"kind": "upright", "object": "red_cube"}])
    def thing_too_big_for_its_place(s):
        s["fixtures"] = [{"kind": "bin", "name": "tray", "at_m": [0.5, 0.42], "inner_m": [0.02, 0.02], "wall_height_m": 0.01}]
        _with_task(s, [{"kind": "inside", "object": "red_cube", "region": "tray"}])
    def answer_nobody_can_check(s):
        _with_task(s, [{"kind": "answer_number", "key": "n", "truth": "weight_of_the_soul"}])
    return [(f.__name__, f, words) for f, words in (
        (unknown_field, "not a field"), (missing_field, "is required"), (out_of_range, "outside"), (bad_colour, "no colour"),
        (twice_named, "already used"), (slanted_outline, "right-angled"), (object_past_the_lip, "past the edge"),
        (rover_past_the_lip, "past the edge"), (arm_past_the_lip, "past the edge"), (two_things_in_one_place, "overlap"),
        (thing_in_mid_air, "not at rest"), (stands_on_nothing, "no fixture"), (task_about_nothing, "no loose object"),
        (task_needs_a_missing_device, "needs a rover"), (task_already_done, "already holds"),
        (thing_too_big_for_its_place, "does not fit"), (answer_nobody_can_check, "cannot be worked out"))]


@pytest.mark.parametrize("name,mutate,words", _mutations(), ids=[m[0] for m in _mutations()])
def test_validator_names_the_fault(name, mutate, words):
    scene = _base()
    assert scenes.validate(scene) == [], "the scene the faults are made from must itself be valid"
    mutate(scene)
    problems = scenes.validate(scene)
    assert problems and any(words in p for p in problems), problems


def test_an_invalid_scene_is_not_saved(tmp_path):
    scene = _base()
    scene["objects"][0]["at_m"] = [5.0, 5.0]
    with pytest.raises(ValueError):
        scenes.write("open_small", scene, directory=tmp_path)
    assert not list(tmp_path.iterdir())
    scenes.write("open_small", _base(), directory=tmp_path)
    assert scenes.read("open_small", directory=tmp_path) == _base()
