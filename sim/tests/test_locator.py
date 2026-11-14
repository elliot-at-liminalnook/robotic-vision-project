"""The locator: where devices are relative to each other, from what was seen, with honest uncertainty.

Aimed at classes of bugs:
  * a pose stated more surely than it is known (the true error must lie inside what is stated,
    over many places, lenses and arm poses);
  * a guess where there is no chain of sightings (no arm, a camera that has seen nothing, one flat tag);
  * the arm's contribution being wrong: its own idea of where its tags and wrist camera are must
    match the referee's at any joint angles;
  * chained links composed or inverted wrongly (checked as algebra, and against the referee
    for camera-to-camera and camera-to-wrist-camera);
  * a camera that was moved being believed to be where it was.
"""
import json
import math

import numpy as np
import pytest

from hub import api as A
from hub import geometry as G
from hub.arm.kinematics import ArmKinematics
from hub.hub import Hub
from hub.locator import Link
from so101sim import scenes
from so101sim.world import World


def call(hub, method, path, body=None):
    status, ctype, out = A.dispatch(hub, method, path, b"" if body is None else json.dumps(body).encode())
    return status, (json.loads(out) if ctype == "application/json" else out)


def arm_frame(world) -> np.ndarray:
    """The referee's: the arm's own frame (origin on the surface under its base) in the scene's frame."""
    m, d = world.model, world.data
    b = m.body("base").id
    T = G.pose(d.xmat[b].reshape(3, 3), d.xpos[b])
    T[2, 3] = 0.0
    return T


def true_camera(world, hub, hub_name: str) -> np.ndarray:
    """The referee's: a hub camera's pose in the arm's frame (matched to the world's camera by serial number)."""
    by_serial = {c.identity["serial"]: c.camera for c in world.ports().cameras}
    cam = hub.wrist if hub_name == "arm/wrist_camera" else hub.cameras[hub_name]
    return G.inv(arm_frame(world)) @ world.camera_pose(by_serial[cam.serial])


def pose_of(answer) -> np.ndarray:
    return G.pose(np.array([answer["axes"][k] for k in "xyz"]).T, answer["position_m"])


def check(answer, truth, share=0.06):
    assert answer["known"], answer
    T = pose_of(answer)
    err, ang = np.linalg.norm(T[:3, 3] - truth[:3, 3]), math.degrees(G.angle_between(T[:3, :3], truth[:3, :3]))
    assert err < 3.5 * answer["uncertainty_m"], f"off by {err * 1000:.0f} mm, stated {answer['uncertainty_m'] * 1000:.0f} mm"
    assert err < share * max(np.linalg.norm(truth[:3, 3]), 0.3) and ang < 3.0, (err, ang)
    return err


# ---------------------------------------------------------------- the algebra of links
def random_link(rng, scale=1.0) -> Link:
    T = G.pose(G.rodrigues(rng.normal(0, 1.0, 3)), rng.normal(0, 0.5, 3))
    a = rng.normal(0, 1, (6, 6)) * np.array([0.01] * 3 + [0.004] * 3) * scale
    return Link(T, a @ a.T, float(rng.uniform(0, 10)), [f"s{rng.integers(100)}"])


@pytest.mark.parametrize("seed", range(10))
def test_links_chain_and_invert_like_poses(seed):
    rng = np.random.default_rng(seed)
    ab, bc = random_link(rng), random_link(rng)
    ac = ab.then(bc)
    assert np.allclose(ac.T, ab.T @ bc.T) and ac.t == min(ab.t, bc.t) and ac.support == ab.support + bc.support
    assert np.allclose(ab.inverse().T, G.inv(ab.T)) and np.allclose(ab.inverse().inverse().T, ab.T)
    assert np.allclose(ab.then(ab.inverse()).T, np.eye(4), atol=1e-9)
    for link in (ac, ab.inverse()):
        assert np.allclose(link.cov, link.cov.T) and np.linalg.eigvalsh(link.cov).min() > -1e-12
    assert ac.sigma_m >= max(ab.sigma_m, 0) - 1e-12, "a chain is never surer than its first link"
    still = ab.then(Link(np.eye(4), np.zeros((6, 6)), 99.0, []))
    assert np.allclose(still.T, ab.T) and np.allclose(still.cov, ab.cov)


@pytest.mark.parametrize("seed", range(4))
def test_a_chains_stated_uncertainty_is_what_sampling_gives(seed):
    """Perturb both links many times by their covariances: the spread of the chained position is the chained covariance."""
    rng = np.random.default_rng(seed)
    ab, bc = random_link(rng), random_link(rng)
    pts = []
    for _ in range(4000):
        da, db = rng.multivariate_normal(np.zeros(6), ab.cov), rng.multivariate_normal(np.zeros(6), bc.cov)
        Ta = G.pose(G.rodrigues(da[:3]) @ ab.T[:3, :3], ab.T[:3, 3] + da[3:])
        Tb = G.pose(G.rodrigues(db[:3]) @ bc.T[:3, :3], bc.T[:3, 3] + db[3:])
        pts.append((Ta @ Tb)[:3, 3])
    spread = np.sqrt(np.trace(np.cov(np.array(pts).T)))
    assert abs(spread / ab.then(bc).sigma_m - 1) < 0.15


# ---------------------------------------------------------------- the arm's own contribution
@pytest.mark.parametrize("seed", range(6))
def test_the_arm_knows_where_its_tags_and_wrist_camera_are(seed):
    """At any joint angles, the hub's built-in idea of the arm puts its tags and wrist camera where the referee has them."""
    world = World("moved_arm")                               # (an arm that is neither at the origin nor facing +x)
    rng = np.random.default_rng(seed)
    kin = ArmKinematics()
    lo, hi = world.model.jnt_range[[world.model.joint(j).id for j in kin_joints()]].T
    q = rng.uniform(lo * 0.8, hi * 0.8)
    world.data.qpos[world.buses[0].qadr] = q
    import mujoco
    mujoco.mj_forward(world.model, world.data)
    T_scene_arm = arm_frame(world)
    assert np.allclose(T_scene_arm @ kin.wrist_camera(q), world.camera_pose("wrist"), atol=1e-6)
    corners = kin.tag_corners(q)
    for i, tag in kin.tags.items():
        g = world.model.geom(f"tag_{i}").id
        centre = world.data.geom_xpos[g] - world.data.geom_xmat[g].reshape(3, 3)[:, 2] * 0.0004
        mine = (T_scene_arm @ np.append(corners[i].mean(axis=0), 1.0))[:3]
        assert np.linalg.norm(mine - centre) < 1e-5, f"tag {i}"


def kin_joints():
    from hub.arm import JOINTS
    return JOINTS


# ---------------------------------------------------------------- locating, in every place
ARM_SCENES = [n for n in scenes.names() if {"arm", "camera"} <= {d["kind"] for d in scenes.read(n)["devices"]}]


@pytest.fixture(scope="module", params=ARM_SCENES)
def located(request):
    """Each scene with an arm and a camera, after the hub has looked with the arm showing its wrist tag."""
    world = World(request.param)
    hub = Hub(world.ports)
    result = call(hub, "POST", "/locate", {"show_wrist_tag": True})[1]
    return world, hub, result


def test_every_camera_is_located_or_said_not_to_be(located):
    world, hub, result = located
    assert result["ok"] and set(result["cameras"]) == set(hub.cameras)
    for name in hub.cameras:
        r = call(hub, "GET", f"/where?of={name}&relative_to=arm")[1]
        if r["known"]:
            check(r, true_camera(world, hub, name))
            assert r["support"] and r["age_s"] >= 0 and result["cameras"][name]["located"]
        else:
            assert r["why"] and "position_m" not in r and not result["cameras"][name]["located"]
    assert any(c["located"] for c in result["cameras"].values()), "with the wrist tag shown, some camera should see the arm"


def test_the_answer_reads_the_same_from_the_other_end(located):
    world, hub, _ = located
    for name in hub.cameras:
        fwd, back = (call(hub, "GET", f"/where?of={a}&relative_to={b}")[1] for a, b in ((name, "arm"), ("arm", name)))
        if fwd["known"]:
            assert back["known"] and np.allclose(pose_of(back), G.inv(pose_of(fwd)), atol=2e-3)


def test_the_wrist_camera_is_placed_in_a_fixed_cameras_frame_through_the_arm(located):
    world, hub, _ = located
    for name in hub.cameras:
        r = call(hub, "GET", f"/where?of=arm/wrist_camera&relative_to={name}")[1]
        if not r["known"]:
            continue
        truth = G.inv(true_camera(world, hub, name)) @ true_camera(world, hub, "arm/wrist_camera")
        check(r, truth, share=0.08)
        assert any("joint angles" in s for s in r["support"]) and any(name in s for s in r["support"])


def test_two_cameras_are_related_through_the_arm_they_both_saw():
    world = World("workcell")
    hub = Hub(world.ports)
    call(hub, "POST", "/locate", {"show_wrist_tag": True})
    r = call(hub, "GET", "/where?of=camera_2&relative_to=camera_1")[1]
    truth = G.inv(true_camera(world, hub, "camera_1")) @ true_camera(world, hub, "camera_2")
    check(r, truth, share=0.08)
    alone = max(call(hub, "GET", f"/where?of={c}&relative_to=arm")[1]["uncertainty_m"] for c in ("camera_1", "camera_2"))
    assert r["uncertainty_m"] >= alone, "a chain of two links is not surer than either"


def test_a_camera_that_cannot_see_the_base_is_found_by_the_wrist_tag():
    world = World("l_shaped")
    hub = Hub(world.ports)
    before = call(hub, "GET", "/where?of=camera_1&relative_to=arm")[1]
    assert before["known"] is False and "show_wrist_tag" in before["try"]
    call(hub, "POST", "/locate", {"show_wrist_tag": True})
    after = call(hub, "GET", "/where?of=camera_1&relative_to=arm")[1]
    check(after, true_camera(world, hub, "camera_1"))
    assert "wrist tag" in " ".join(after["support"]) and "base" not in " ".join(after["support"])


@pytest.mark.parametrize("seed", range(8))
def test_stated_uncertainty_covers_the_error_wherever_the_camera_is(seed):
    """Cameras at seeded places, each with its own lens: located, and never more surely than truly."""
    rng = np.random.default_rng(seed)
    s = scenes.read("open_small")
    s["devices"] = [d for d in s["devices"] if d["kind"] != "rover"]
    s.pop("tasks", None)
    arm = np.array([0.14, 0.27])
    bearing, r, h = rng.uniform(-math.pi, math.pi), rng.uniform(0.4, 0.8), rng.uniform(0.22, 0.5)
    cam = next(d for d in s["devices"] if d["kind"] == "camera")
    cam["at_m"] = [float(v) for v in (*(arm + r * np.array([math.cos(bearing), math.sin(bearing)])), h)]
    cam["looking_at_m"] = [float(arm[0] + 0.1 * math.cos(bearing + 2.5)), float(arm[1] + 0.1 * math.sin(bearing + 2.5)), 0.06]
    s["surface"]["outline_m"] = [[-0.9, -0.8], [1.2, -0.8], [1.2, 1.3], [-0.9, 1.3]]
    s["imperfections"] = {"seed": seed}
    world = World(s)
    hub = Hub(world.ports)
    call(hub, "POST", "/locate", {"show_wrist_tag": True})
    answer = call(hub, "GET", "/where?of=camera_1&relative_to=arm")[1]
    if not answer["known"]:
        pytest.skip("the arm's tags were not in this camera's view: " + answer["why"])
    check(answer, true_camera(world, hub, "camera_1"))


# ---------------------------------------------------------------- not known, and no longer true
def test_nothing_links_two_cameras_without_an_arm():
    hub = Hub(World("no_arm").ports)
    r = call(hub, "GET", "/where?of=camera_2&relative_to=camera_1")[1]
    assert r["ok"] and r["known"] is False and "no arm" in r["why"]
    assert call(hub, "POST", "/locate", {"show_wrist_tag": True})[0] == 400
    assert call(hub, "POST", "/locate")[1]["ok"]


def test_one_flat_tag_seen_once_is_not_an_answer():
    """A camera that sees only the wrist tag, once: the reading could be tilted either way, and the locator says so."""
    world = World("l_shaped")
    hub = Hub(world.ports)
    call(hub, "POST", "/devices/arm/move_joints", {"shoulder_pan": 0.0})
    r = call(hub, "GET", "/where?of=camera_1&relative_to=arm")[1]
    seen = call(hub, "GET", "/devices/camera_1/picture?tags=true")[1]["tags"]
    assert r["known"] is False
    if seen:
        assert [t["id"] for t in seen] == [8] and ("one plane" in r["why"] or "too little" in r["why"])


def test_age_counts_from_the_newest_sighting():
    hub = Hub(World("open_small").ports)
    first = call(hub, "GET", "/where?of=camera_1&relative_to=arm")[1]
    call(hub, "POST", "/wait", {"seconds": 2.0})
    later = call(hub, "GET", "/where?of=camera_1&relative_to=arm")[1]
    assert later["age_s"] >= first["age_s"] + 1.9 and later["position_m"] == first["position_m"]
    call(hub, "POST", "/locate")
    assert call(hub, "GET", "/where?of=camera_1&relative_to=arm")[1]["age_s"] < 0.5


def test_a_camera_that_was_moved_is_not_believed_to_be_where_it_was():
    """Shift the camera 12 cm behind the hub's back (no reload, so its port stays). The next look notices,
    drops what it saw before, and the estimate follows."""
    world = World("open_small")
    hub = Hub(world.ports)
    call(hub, "POST", "/locate")
    before = call(hub, "GET", "/where?of=camera_1&relative_to=arm")[1]
    cam = world.model.camera("camera_1").id
    world.model.cam_pos[cam] += [0.0, -0.12, 0.03]
    import mujoco
    mujoco.mj_forward(world.model, world.data)
    result = call(hub, "POST", "/locate")[1]
    after = call(hub, "GET", "/where?of=camera_1&relative_to=arm")[1]
    assert "moved" in " ".join(result["cameras"]["camera_1"].get("notes", []))
    check(after, true_camera(world, hub, "camera_1"))
    assert np.linalg.norm(np.array(after["position_m"]) - np.array(before["position_m"])) > 0.08
