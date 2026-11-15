"""One solution per task. Each takes the Api (and, for a task where the place may change under it, a function that says
'this is the moment') and returns the answers to hand in, or nothing for a task that is judged by how things were left."""
from __future__ import annotations

import math

import cv2
import numpy as np

from . import bench, eyes, moves
from .api import Api, GaveUp, Missed, NotThere, OutOfReach


# ---------------------------------------------------------------- where is everyone
def where_is(of: str, relative_to: str):
    def solve(api: Api, **_) -> dict:
        devices = api.devices()
        if "arm" in devices and any(n.startswith("camera_") for n in devices):
            moves.locate_cameras(api)
        r = api.ok("GET", f"/where?of={of}&relative_to={relative_to}")
        if not r["known"] and "rover" in devices:               # the rover looks round: it may see the other's tags itself
            api.ok("POST", "/survey")
            r = api.ok("GET", f"/where?of={of}&relative_to={relative_to}")
        if not r["known"]:
            raise GaveUp(f"nothing relates {of} to {relative_to}: {r['why']}")
        return {"pose": {"x": r["position_m"][0], "y": r["position_m"][1]}}
    return solve


EDGE_WIDTH_M = 0.01            # a raised edge's width, as it shows in the map: one cell


# ---------------------------------------------------------------- how big is this place
SEEN_SHARE = 0.65              # a rectangle is answered only if this much of it has been seen as surface


def _spans(edges: list):
    """The surface's two spans from its edges, or which way to go to see the edge that is missing.
    Returns (spans, None) or (None, (bearing in the edges' frame [deg], why))."""
    edges = [e for e in edges if e["kind"] == "edge of the surface" and e["length_m"] >= 0.15]
    if not edges:
        return None, (0.0, "the survey found no edge of the surface")
    way = max(edges, key=lambda e: e["length_m"])["direction_deg"]
    spans = []
    for turn in (0.0, 90.0):                                    # edges along the longest one, and edges across it
        a = math.radians(way + turn)
        normal = np.array([-math.sin(a), math.cos(a)])
        # (a raised edge in the map stands outside the surface by half its width; an edge read off a picture is where the
        # surface itself ends)
        off = [(float(normal @ (np.array(e["from_m"]) + e["to_m"]) / 2), 0.0 if e.get("inner") else EDGE_WIDTH_M / 2) for e in edges
               if abs((e["direction_deg"] - way - turn + 90) % 180 - 90) < 20]
        if len(off) < 2 or max(off)[0] - min(off)[0] < 0.15:
            away = -normal if (off and np.mean([o for o, _ in off]) > 0) else normal        # the side no edge was found on
            return None, (math.degrees(math.atan2(away[1], away[0])),
                          f"only one side of the surface was seen along {((way + turn) % 180):.0f} deg: its far edge is beyond what the "
                          "rover could see from where it stood")
        spans.append((max(off)[0] - max(off)[1]) - (min(off)[0] + min(off)[1]))
    return spans, None


def surface_places(place: dict, step: float = 0.08, inside: float = 0.04, clear: float = 0.06) -> list:
    """Places the map says are bare surface: a grid of points well inside what it has seen, clear of the things in it.
    (Not places round the rover: on a narrow surface most of those are off it, and the 'surface' found in a picture was
    the floor beyond.)"""
    surface = place.get("surface") or {}
    outline = np.array(surface.get("outline_m", []), np.float32)
    if len(outline) < 3:
        return []
    things = [np.array(t["at_m"]) for t in place.get("things", [])]
    lo, hi = outline.min(axis=0), outline.max(axis=0)
    out = []
    for x in np.arange(lo[0], hi[0], step):
        for y in np.arange(lo[1], hi[1], step):
            if (cv2.pointPolygonTest(outline.reshape(-1, 1, 2), (float(x), float(y)), True) >= inside
                    and all(np.linalg.norm(t - [x, y]) > clear + 0.0 for t in things)):
                out.append((float(x), float(y)))
    return out


def in_picture(api: Api, cam: str, places: list) -> list:
    """Where each of some places on the surface (rover's frame) is in a camera's picture: the pixels of those it covers."""
    out = []
    for x, y in places:
        r = api.ok("GET", f"/where/point?camera={cam}&x={x:.3f}&y={y:.3f}&height_m=0&relative_to=rover")
        if not r["known"]:
            return []                                            # (nothing relates this camera to the rover)
        if r["in_picture"] and r["inside_the_edge_px"] > 30:
            out.append(r["pixel"])
    return out


def edges_in_pictures(api: Api, place: dict) -> tuple[list, float]:
    """The surface's edges as the fixed cameras see them, in the rover's frame, and the largest area of lit surface any one
    of them sees [m2]. A fixed camera looks down on the surface from above and sees edges the rover, low down, cannot
    (behind the arm, or further off than its two cameras range). What the surface looks like is taken from the
    picture itself, at places the map says are bare surface."""
    edges, lit = [], 0.0
    places = surface_places(place)
    for cam in moves.fixed_cameras(api):
        samples = in_picture(api, cam, places)
        mask = eyes.lit_surface(api.picture(f"/devices/{cam}"), samples) if len(samples) >= 2 else None
        if mask is None:
            continue
        place = lambda p: moves_place(api, cam, p)               # noqa: E731
        for a, b in eyes.straight_sides(mask):
            pa, pb = place(a), place(b)
            if pa is None or pb is None or np.linalg.norm(pb - pa) < 0.05:
                continue
            edges.append({"kind": "edge of the surface", "from_m": pa.tolist(), "to_m": pb.tolist(), "length_m": float(np.linalg.norm(pb - pa)),
                          "direction_deg": math.degrees(math.atan2(pb[1] - pa[1], pb[0] - pa[0])) % 180.0, "inner": True, "seen_by": cam})
        outline = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        ring = [place(q) for q in cv2.approxPolyDP(max(outline, key=cv2.contourArea), 8.0, True).reshape(-1, 2).astype(float)]
        if ring and all(q is not None for q in ring):
            lit = max(lit, 0.5 * abs(sum(p[0] * q[1] - q[0] * p[1] for p, q in zip(ring, ring[1:] + ring[:1]))))
    return edges, lit


def moves_place(api: Api, cam: str, p):
    """A pixel of a fixed camera's picture as a place on the surface, in the rover's frame (None if it cannot be told)."""
    r = api.ok("GET", f"/where/pixel?camera={cam}&u={float(p[0]):.1f}&v={float(p[1]):.1f}&height_m=0&relative_to=rover")
    return np.array(r["at_m"]) if r["known"] else None


def how_big(api: Api, **_) -> dict:
    """The surface's area from its edges: two pairs of parallel edges are a rectangle. The rover's map has the raised edges
    near it. If one is out of its sight, the fixed cameras' pictures are asked for the surface's edges as they see them;
    failing that the rover turns to face that way, drives as far as the map lets it, and looks again."""
    why = "the survey kept no map"
    for standpoint in range(3):
        survey = api.ok("POST", "/survey")
        place = api.ok("GET", "/place?relative_to=rover")       # (in the rover's own frame, as it stands now)
        surface = place.get("surface")
        if surface is None:
            raise GaveUp("the survey kept no map: " + str(survey.get("stood_still", {}).get("warning", "it saw too little")))
        edges, seen = list(surface.get("edges_found", [])), surface["area_seen_m2"]
        spans, missing = _spans(edges)
        if spans is None and moves.fixed_cameras(api):
            if "arm" in api.devices() and not getattr(api, "rest", None):
                moves.locate_cameras(api)                        # (the cameras are related to the rover through the arm's tags)
            more, lit = edges_in_pictures(api, place)
            spans, still = _spans(edges + more)
            seen = max(seen, lit)
            missing = missing if spans is None and not more else still
        if spans is not None:
            area = spans[0] * spans[1]
            if seen > 1.2 * area:
                raise GaveUp(f"the surface is not a plain rectangle: {seen:.2f} m2 of it has been seen, and the edges "
                             f"found enclose {area:.2f} m2. Its other edges are beyond what the rover could see")
            if seen < SEEN_SHARE * area:
                raise GaveUp(f"the edges found enclose {area:.2f} m2, and only {seen:.2f} m2 of that has been seen to be surface: "
                             "it may not be a plain rectangle (a corner cut out of it would not have been seen)")
            return {"area_m2": round(area, 4)}
        bearing, why = missing                                   # (in the rover's frame: 0 is straight ahead)
        turn = (bearing + 180.0) % 360.0 - 180.0
        if abs(turn) > 2.0 and api.post("/devices/rover/drive/turn", {"angle_deg": turn})[0] != 200:
            break                                               # something the map has is in the way of turning
        if api.post("/devices/rover/drive/forward", {"distance_m": 0.25})[0] != 200 and abs(turn) <= 2.0:
            break                                               # it can neither turn nor go: nothing new to see from here
    raise GaveUp(why + " (and the rover could not get anywhere it could be seen from)")


# ---------------------------------------------------------------- what is on the shelf
def on_the_shelf(api: Api, **_) -> dict:
    """The thing that stands on something raised: its colour, and how high its lowest part is."""
    if not api.ok("POST", "/survey")["map_kept"]:
        api.ok("POST", "/survey")
    things = api.ok("GET", "/place").get("things", [])
    # A shelf's bare top gives two cameras nothing to match, so the shelf itself may not be in the map at all. What stands
    # on it is: a small coloured thing whose lowest seen part is well above the surface. That lowest part is the shelf's top.
    up = [t for t in things if t.get("colour") in ("red", "green", "blue", "yellow", "purple", "orange") and t["length_m"] < 0.12
          and t.get("lowest_seen_m", 0.0) >= 0.04 and t["height_m"] - t.get("lowest_seen_m", 0.0) < 0.06]
    if not up:
        raise GaveUp(f"nothing coloured in the map stands on something raised ({len(things)} things, none with its lowest part above 40 mm)")
    thing = max(up, key=lambda t: t["cells"])
    return {"colours": [thing["colour"]], "height_m": thing["lowest_seen_m"]}


# ---------------------------------------------------------------- carried somewhere else
def carried(of: str, relative_to: str):
    def solve(api: Api, intermission=None, **_) -> dict:
        where_is(of, relative_to)(api)                           # (what it was, before)
        if intermission is not None:
            intermission()
        # Has everything been picked up? A device that was put down somewhere else is, to the hub, a device that has stopped
        # answering: ask each for something fresh.
        asked = [api.get(f"/devices/{n}/state")[0] for n in api.devices()] + [api.get(f"/devices/{c}/picture?width=160")[0]
                                                                              for c in moves.fixed_cameras(api)]
        gone = 503 in asked
        if gone:
            api.ok("POST", "/discover")
        after = where_is(of, relative_to)(api)
        return {"moved": "yes" if gone else "no", **after}
    return solve


# ---------------------------------------------------------------- pick and place
def stack(thing: str, on: str):
    def solve(api: Api, **_) -> None:
        moves.locate_cameras(api)
        under = moves.find_thing(api, on, height_guess=0.02)
        top = moves.find_thing(api, thing)
        grasp = moves.pick(api, top)
        moves.put(api, under["at"], under["top"], grasp)
        moves.park(api)
    return solve


IN_THE_BIN_M = 0.05            # a thing found this near a bin's middle is in it
BIN_SPOTS = np.array([[0.0, 0.0], [0.018, 0.018], [-0.018, -0.018], [0.018, -0.018]])      # where in a bin the first, second, ... thing goes


def into_bins(pairs=(("red", "red"), ("blue", "blue")), high: float = 0.03):
    """Carry every loose thing of a colour into the bin of a colour: pairs of (thing's colour, bin's colour).
    high: about how high the things' tops are (a thing on a shelf: its top above the surface)."""
    def solve(api: Api, **_) -> None:
        moves.locate_cameras(api)
        for colour, bin_colour in pairs:
            bin_near = moves.roughly(api, bin_colour, ring=True, height=0.012)
            bin_at, _ = moves.closely(api, bin_colour, bin_near, 0.012, ring=True)
            carried, passed, why_not = 0, [], []                # passed: patches of that colour that were not a thing
            for _ in range(8):                                  # until no loose thing of that colour is left outside the bin
                near = None
                for k in range(4):
                    try:
                        at = moves.roughly(api, colour, pick=k, height=max(high - 0.015, 0.0))
                    except GaveUp:
                        break
                    if np.linalg.norm(at - bin_at) > 0.06 and all(np.linalg.norm(at - p) > 0.04 for p in passed):
                        near = at
                        break
                if near is None:
                    break
                try:
                    thing = moves.find_thing(api, colour, height_guess=high, near=near)
                    if np.linalg.norm(thing["at"] - bin_at) < IN_THE_BIN_M:
                        # Found from close above, it is in the bin: one carried there already, which the fixed camera put
                        # somewhere else because it guessed its height as it was before (up on a shelf).
                        raise NotThere(f"what the fixed camera saw at ({near[0]:.3f}, {near[1]:.3f}) is in the bin already")
                    grasp = moves.pick(api, thing)
                except (NotThere, OutOfReach, Missed) as e:
                    # What a fixed camera took for such a thing is not one from close above, or is where the hand cannot be
                    # held to look, or the jaws found nothing there: part of the arm itself (it is printed in one of these
                    # colours), or a thing too far off its guessed height. Pass over it.
                    passed.append(near)
                    why_not.append(str(e))
                    continue
                moves.put(api, bin_at + BIN_SPOTS[carried % len(BIN_SPOTS)], 0.0, grasp, let_go_above=0.02)     # (not onto the last one)
                carried += 1
            if not carried:
                raise GaveUp(f"no {colour} thing was found within the arm's reach to carry to the {bin_colour} bin"
                             + (f" ({len(why_not)} patches of that colour were looked at: {why_not[0]})" if why_not else ""))
        moves.park(api)
    return solve


def under_the_ledge(api: Api, **_) -> dict:
    """The colours of what lies under something raised: things in the map that are lower than a raised flat thing's
    underside and within its outline. (The rover's low pass looks under; the map keeps the top over each cell, so
    what is seen under an overhang shows only where the overhang itself was not seen from above.)"""
    if not api.ok("POST", "/survey")["map_kept"]:
        api.ok("POST", "/survey")
    things = api.ok("GET", "/place").get("things", [])
    raised = [t for t in things if t["length_m"] >= 0.10 and t["width_m"] >= 0.06 and t.get("lowest_seen_m", 0.0) >= 0.03]
    if not raised:
        raise GaveUp(f"nothing raised and flat is in the map ({len(things)} things)")
    colours = []
    for t in things:
        for r in raised:
            if (t is not r and "colour" in t and t["colour"] not in ("grey", "black", "white", "brown")
                    and t["height_m"] < r["lowest_seen_m"] - 0.01
                    and np.linalg.norm(np.array(t["at_m"]) - r["at_m"]) < max(r["length_m"], r["width_m"]) / 2 + 0.02):
                colours.append(t["colour"])
                break
    return {"colours": sorted(set(colours))}


# ---------------------------------------------------------------- what the fixed cameras cannot see
LOOSE = ("red", "green", "blue", "yellow", "purple")           # the colours a loose thing may be
LOOK_PANS = (0.0, -40.0, 40.0, -80.0, 80.0, -120.0, 120.0, -160.0, 160.0)
SAME_THING_M = 0.12            # two patches of one colour placed this near each other are one thing seen twice
ARM_HALF_WIDTH_M = 0.15        # what is in line with the arm, this far either side of its base, may be the arm itself


def out_of_sight(api: Api, **_) -> dict:
    """The loose things no fixed camera can see: how many, and their colours. The rover need not drive: it turns its head
    each way, and each patch of colour in its picture is put on the surface where its foot is (GET /where/pixel). A place
    that is outside every fixed camera's picture (GET /where/point) is out of their sight."""
    devices = api.devices()
    cameras = moves.fixed_cameras(api)
    if "rover" not in devices or not cameras:
        raise GaveUp("this takes a rover to look and a fixed camera whose sight is in question")
    if "arm" in devices:
        moves.locate_cameras(api)                                # (the cameras are related to the rover through the arm's tags)
    arm = api.ok("GET", "/where?of=arm&relative_to=rover") if "arm" in devices else {"known": False}
    api.ok("POST", "/devices/rover/head/raise", {"lens_height_m": 0.16})
    seen, looked = [], 0                                         # (colour, place in the rover's frame, the patch's area)
    try:
        for pan in LOOK_PANS:
            status, r = api.post("/devices/rover/head/point", {"pan_deg": pan, "tilt_deg": -12.0})
            if status != 200 or not r.get("reached"):
                continue                                         # the arm or something else is in the head's way: not that way
            looked += 1
            picture = api.picture("/devices/rover/head/left")
            for colour in LOOSE:
                for b in eyes.find(picture, colour, min_area=60.0)[:6]:
                    if b.cut_off:
                        continue                                 # (its foot is not in the picture; the next view has it whole)
                    foot = min(float(b.corners[:, 1].max()), picture.shape[0] - 1.0)
                    w = api.ok("GET", f"/where/pixel?camera=rover/head/left&u={b.u:.1f}&v={foot:.1f}&height_m=0&relative_to=rover")
                    if not w["known"]:
                        continue                                 # (its line of sight does not come down: something high, not on the surface)
                    at = np.array(w["at_m"])
                    if arm["known"]:
                        # The arm is printed in one of these colours. Its parts stand above the surface, so their lines of
                        # sight meet the surface beyond it: whatever is in line with the arm, at it or past it, is left out.
                        base = np.array(arm["position_m"][:2])
                        along = float(at @ base) / max(float(np.linalg.norm(base)), 1e-6)
                        across = float(np.linalg.norm(at - along * base / max(float(np.linalg.norm(base)), 1e-6)))
                        if along > np.linalg.norm(base) - ARM_HALF_WIDTH_M and across < ARM_HALF_WIDTH_M * max(1.0, along / np.linalg.norm(base)):
                            continue
                    seen.append((colour, at, b.area))
    finally:
        api.post("/devices/rover/head/stow")
    if not looked:
        raise GaveUp("the rover's head could not be turned any way to look")
    things = []                                                  # one entry per thing: the largest patch of it
    for colour, at, area in sorted(seen, key=lambda s: -s[2]):
        if not any(c == colour and np.linalg.norm(at - p) < SAME_THING_M for c, p in things):
            things.append((colour, at))
    hidden = []
    for colour, at in things:
        covered = False
        for cam in cameras:
            r = api.ok("GET", f"/where/point?camera={cam}&x={at[0]:.4f}&y={at[1]:.4f}&height_m=0.015&relative_to=rover")
            if not r["known"]:
                raise GaveUp(f"whether {cam} can see a place cannot be told: {r['why']}")
            covered |= bool(r["in_picture"])
        if not covered:
            hidden.append(colour)
    return {"count": len(hidden), "colours": sorted(set(hidden))}


# ---------------------------------------------------------------- is the drawer out
DRAWER_SHOWS_M = 0.030         # a white patch at the cabinet wider than this is the inside of a drawer, not its front edge (which
#                                measures 10 to 12 mm from above; 12 mm ajar measures 22 to 24 and is not called open)
LOOK_RING_M = 0.24             # the hand hovers this far out to look down, at each of LOOK_BEARINGS: the wrist camera sees 0.15 m each way
LOOK_BEARINGS = (0.0, 35.0, -35.0, 70.0, -70.0, 105.0, -105.0)


def drawer_open(api: Api, **_) -> dict:
    """Whether the grey cabinet's drawer is out, by looking down with the wrist camera from places all round the arm.
    The cabinet is something grey with something white at its side: its drawer. Shut, what shows of the drawer is the top
    of its front, a strip some 10 mm wide. Out, its tray shows. The answer is how wide the widest such patch is.
    (The rover's map is not used: the cabinet may stand at the edge of what the rover's two cameras can range.)"""
    if "arm" not in api.devices():
        raise GaveUp("this takes an arm, to look down with the camera on its wrist")
    joints = api.ok("GET", "/devices/arm/state")["joints"]["joints_rad"]
    api.rest = {k: v for k, v in joints.items() if k != "gripper"}
    widths, looked = [], 0
    for bearing in LOOK_BEARINGS:
        way = np.array([math.cos(math.radians(bearing)), math.sin(math.radians(bearing))])
        try:
            moves.hover_over(api, LOOK_RING_M * way, hover=0.16)
        except OutOfReach:
            continue
        looked += 1
        # (What is on a printed tag is left out: the arm's own tag blocks are white round their tags and grey on their
        # shaded sides. bench.Look asks the hub where the tags are in the picture.)
        look = bench.Look(api)
        grey = look.find("grey", 4000.0)
        for b in look.find("white", 300.0)[:6]:
            if b.cut_off or not any(bench.beside(b, g) for g in grey):
                continue                                         # part of a patch, or white that is not at anything grey
            corners = [moves._place(api, "arm/wrist_camera", float(c[0]), float(c[1]), 0.01)[0] for c in b.corners]
            if any(c is None for c in corners):
                continue
            widths.append(min(float(np.linalg.norm(corners[1] - corners[0])), float(np.linalg.norm(corners[2] - corners[1]))))
    moves.park(api)
    if not looked:
        raise GaveUp("the hand could not be held anywhere to look down")
    if not widths:
        raise GaveUp(f"looking down from {looked} places round the arm, the wrist camera saw nothing white beside anything grey: "
                     "no drawer to judge")
    return {"open": "yes" if max(widths) > DRAWER_SHOWS_M else "no"}


SOLVERS = {
    ("open_small", "where_is_everyone"): where_is("rover", "arm"),
    ("moved_arm", "where_is_everyone"): where_is("rover", "arm"),
    ("l_shaped", "where_is_everyone"): where_is("rover", "arm"),
    ("no_arm", "where_is_everyone"): where_is("camera_1", "rover"),
    ("open_small", "how_big"): how_big,
    ("l_shaped", "far_leg"): out_of_sight,
    ("bench", "drawer_open"): drawer_open,
    ("bench", "tidy"): bench.tidy,
    ("l_shaped", "how_big"): how_big,
    ("no_arm", "how_big"): how_big,
    ("shelf", "on_the_shelf"): on_the_shelf,
    ("moved_arm", "carried"): carried("rover", "arm"),
    ("workcell", "stack"): stack("red", "green"),
    ("cluttered", "tidy_by_colour"): into_bins(),
    ("cluttered", "fetch_hidden"): into_bins((("yellow", "blue"),), high=0.06),
    ("moved_arm", "fetch"): into_bins((("red", "cream"),)),
    ("shelf", "fetch_from_shelf"): into_bins((("red", "cream"),), high=0.11),
    ("shelf", "under_the_ledge"): under_the_ledge,
}

NO_SOLUTION = {
}
