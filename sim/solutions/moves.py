"""Finding a thing by its colour and picking it up, through the hub's API.

Coarse, then fine. A fixed camera says roughly where the thing is (its place relative to the arm is good to
a centimetre or two, and the thing's height is a guess). The hand then hovers over that place and the wrist
camera, whose place the arm knows from its own joints, looks down at it; a touch gives its height; a second
look from above, with the height known, gives its place to a few millimetres.
"""
from __future__ import annotations

import math

import numpy as np

from . import eyes
from .api import Api, GaveUp

HOVER_M = 0.14                 # how high the tip hovers to look down with the wrist camera
WRIST = "/devices/arm/wrist_camera"
NEAR_M = 0.12                  # what the wrist camera takes for the thing must be this near where it was expected
REACH_M = (0.11, 0.40)         # how near and how far from its base the arm can pick something up (nearer are its own tag blocks)


def fixed_cameras(api: Api) -> list[str]:
    return sorted(n for n, d in api.devices().items() if n.startswith("camera_"))


def park(api: Api):
    """Put the arm back as it stood when the solution began: out of the way of the fixed cameras' view of the surface."""
    if getattr(api, "rest", None):
        api.post("/devices/arm/move_joints", api.rest)


def locate_cameras(api: Api):
    """Have the arm show its tags, so that each fixed camera's place relative to the arm is known as well as it can be."""
    joints = api.ok("GET", "/devices/arm/state")["joints"]["joints_rad"]
    api.rest = {k: v for k, v in joints.items() if k != "gripper"}       # how it stood: clear of what the cameras look at
    api.ok("POST", "/locate", {"show_wrist_tag": True})
    for cam in fixed_cameras(api):
        api.post(f"/devices/{cam}/calibrate")                  # (refused if it saw too little: then it stays as it is)
    park(api)


def goto(api: Api, x: float, y: float, z: float, **more):
    """Move the tip to a point: straight there if the arm can follow the line, else by swinging the shoulder round to face the
    point first and going out or in from there. Returns (status, answer) as the hub gave them for the last move tried."""
    status, r = api.post("/devices/arm/move_tip", {"x": x, "y": y, "z": z, **more})
    if status == 200 or "straight line" not in str(r.get("error", "")):
        return status, r
    def facing():
        st = api.ok("GET", "/devices/arm/state")
        return st["joints"]["joints_rad"]["shoulder_pan"], math.atan2(st["tip_m"]["y"], st["tip_m"]["x"]), st["joints"]["joint_limits_rad"]["shoulder_pan"]

    pan, bearing, limits = facing()
    turn = (math.atan2(y, x) - bearing + math.pi) % (2 * math.pi) - math.pi
    for sign in (1.0, -1.0):                                        # (which way the shoulder's angle counts is found by trying)
        api.post("/devices/arm/move_joints", {"shoulder_pan": float(np.clip(pan + sign * turn, limits[0] + 0.02, limits[1] - 0.02))})
        left = (math.atan2(y, x) - facing()[1] + math.pi) % (2 * math.pi) - math.pi
        if abs(left) < max(0.15, 0.5 * abs(turn)):
            break
    return api.post("/devices/arm/move_tip", {"x": x, "y": y, "z": z, **more})


def _go(api: Api, x: float, y: float, z, **more) -> dict:
    """Send the hand to (x, y) at height z, or at the first of several heights it can reach (highest first: near its base the
    arm cannot hold its hand as high)."""
    r = {}
    for height in (z if isinstance(z, (list, tuple)) else [z]):
        status, r = goto(api, x, y, float(height), **more)
        if status == 200:
            return r
    raise GaveUp(f"the hand could not be sent to ({x:.3f}, {y:.3f}, {z}): {r.get('error', r)}")


def _place(api: Api, camera: str, u: float, v: float, height: float):
    if not (0 <= u < 1920 and 0 <= v < 1080):                       # (the corner of a patch cut off by the picture's edge)
        return None, None
    r = api.ok("GET", f"/where/pixel?camera={camera}&u={u:.1f}&v={v:.1f}&height_m={height:.4f}&relative_to=arm")
    return (np.array(r["at_m"]), r["uncertainty_m"]) if r["known"] else (None, None)


def _kind(blobs: list, ring: bool) -> list:
    """The patches that may be what is looked for. A bin is an open frame: seen whole it has a hole in it, and it is the largest
    patch of its colour when something hides part of it. A loose thing may be any patch (the caller leaves out the bin by where
    it is)."""
    if not ring:
        return blobs
    return [b for b in blobs if b.ring] or blobs[:1]


def roughly(api: Api, colour: str, ring: bool = False, height: float = 0.015, pick=0) -> np.ndarray:
    """Where the pick-th largest thing of that colour within the arm's reach is, in the arm's frame, as the fixed cameras see
    it. ring: a bin, not a block."""
    park(api)                                                       # (the hand over the surface hides what is under it)
    best = None
    for cam in fixed_cameras(api):
        seen = []
        for b in _kind(eyes.find(api.picture(f"/devices/{cam}"), colour), ring)[:8]:
            at, sure = _place(api, cam, b.u, b.v, height)
            if at is not None and REACH_M[0] < np.linalg.norm(at) < REACH_M[1]:      # (a patch of that colour far off is something else)
                seen.append((at, sure))
        if len(seen) > pick and (best is None or seen[pick][1] < best[1]):
            best = seen[pick]
    if best is None:
        raise GaveUp(f"no fixed camera sees a {colour} {'bin' if ring else 'thing'} within the arm's reach")
    return best[0]


def closely(api: Api, colour: str, near: np.ndarray, height: float, ring: bool = False, hover: float = HOVER_M) -> tuple[np.ndarray, float | None]:
    """Hover over `near` and look down with the wrist camera: the thing's place in the arm's frame, and which way one of its
    sides runs [deg in the arm's x-y plane] (None for a bin). hover: how high the tip is held while looking."""
    # Straight above a thing near the arm's base is out of its reach at hovering height: hover a little further out, or lower.
    reach, way = float(np.linalg.norm(near)), np.asarray(near, float) / max(float(np.linalg.norm(near)), 1e-6)
    r = None
    for out, z in ((max(reach, 0.215), hover), (max(reach, 0.20), hover - 0.03), (reach, hover - 0.05)):
        status, r = goto(api, float(way[0] * out), float(way[1] * out), z, tolerance_m=0.01)
        if status == 200:
            break
    else:
        raise GaveUp(f"the hand cannot hover over ({near[0]:.3f}, {near[1]:.3f}): {r.get('error', r)}")
    seen = eyes.find(api.picture(WRIST), colour, min_area=400)
    # the one nearest where it was expected, among those of a fair size
    placed = [(b, _place(api, "arm/wrist_camera", b.u, b.v, height)[0]) for b in _kind(seen, ring)[:4]]
    placed = [(b, p) for b, p in placed if p is not None and np.linalg.norm(p - near) < NEAR_M]
    if not placed and ring:
        # From this close a bin's frame may not fit in the picture as a whole frame, and what does look like a frame is
        # something else (the arm's own tag block). The fixed camera saw the whole of it: keep its place.
        return np.asarray(near, float), None
    if not placed:
        raise GaveUp(f"hovering over ({near[0]:.3f}, {near[1]:.3f}) the wrist camera sees no {colour} thing within {NEAR_M * 1000:.0f} mm of there"
                     + ("" if r["reached"] else f" (the hand did not get there: {r.get('why_not_reached')})"))
    b, at = min(placed, key=lambda bp: np.linalg.norm(bp[1] - near))
    side = None
    if not ring:
        a, c = (_place(api, "arm/wrist_camera", float(p[0]), float(p[1]), height)[0] for p in b.corners[:2])
        if a is not None and c is not None and np.linalg.norm(c - a) > 0.005:
            side = math.degrees(math.atan2(c[1] - a[1], c[0] - a[0])) % 180.0
    return at, side


def touch(api: Api, at: np.ndarray, guess: float, start: float = 0.10) -> float:
    """How high the thing at `at` is, by lowering the closed gripper onto it; the guess if it cannot be touched."""
    api.ok("POST", "/devices/arm/gripper/set", {"opening": 0.0})
    for from_z in (max(start, 0.10), max(start, 0.10) - 0.02, max(start, 0.10) - 0.03):    # (near the arm's base it cannot start from as high)
        status, r = api.post("/devices/arm/probe", {"x": float(at[0]), "y": float(at[1]), "from_z": from_z})
        if status == 200:
            return float(r["height_m"]) if r.get("touched") else guess
    return guess


def find_thing(api: Api, colour: str, pick=0, height_guess: float = 0.03) -> dict:
    """A loose thing of that colour: where it is in the arm's frame, how high, and which way its sides run.
    height_guess: about how high its top is; it need not be right, a touch finds out."""
    hover = max(HOVER_M, height_guess + 0.11)
    near = roughly(api, colour, height=max(height_guess - 0.015, 0.0), pick=pick)
    at, _ = closely(api, colour, near, height_guess, hover=hover)
    at, _ = closely(api, colour, at, height_guess, hover=hover)     # from more nearly above it, the guess matters less
    top = touch(api, at, height_guess, start=height_guess + 0.07)
    at, side = closely(api, colour, at, top, hover=max(HOVER_M, top + 0.11))      # again, now that its height is known
    return {"at": at, "top": float(top), "side_deg": side}


def pick(api: Api, thing: dict) -> float:
    """Take hold of a thing found by find_thing and lift it. Returns how high above the surface the jaws hold it."""
    grasp = max(0.011, thing["top"] - 0.022)
    x, y = float(thing["at"][0]), float(thing["at"][1])
    api.ok("POST", "/devices/arm/gripper/set", {"opening": 1.0})
    turn = {"jaws_along_deg": thing["side_deg"]} if thing["side_deg"] is not None else {"square_jaws": True}
    _go(api, x, y, [grasp + 0.07, grasp + 0.05, grasp + 0.035], **turn)
    down = _go(api, x, y, grasp)
    held = api.ok("POST", "/devices/arm/gripper/set", {"opening": 0.0})
    if not held.get("holding"):
        raise GaveUp(f"the jaws closed on nothing at ({x:.3f}, {y:.3f}, {grasp:.3f})"
                     + ("" if down["reached"] else f"; the hand did not get down: {down.get('why_not_reached')}"))
    _go(api, x, y, [grasp + 0.10, grasp + 0.07, grasp + 0.05])
    return grasp


def put(api: Api, at: np.ndarray, on_top_of: float, grasp: float, let_go_above: float = 0.004):
    """Carry what is held to `at` and set it down on something `on_top_of` metres high (0: the surface)."""
    x, y = float(at[0]), float(at[1])
    z = on_top_of + grasp + let_go_above
    _go(api, x, y, [z + 0.08, z + 0.05, z + 0.03])
    _go(api, x, y, z)
    api.ok("POST", "/devices/arm/gripper/set", {"opening": 1.0})
    _go(api, x, y, [z + 0.08, z + 0.05, z + 0.03])
