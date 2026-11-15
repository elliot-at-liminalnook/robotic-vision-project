"""One solution per task. Each takes the Api (and, for a task where the place may change under it, a function that says
'this is the moment') and returns the answers to hand in, or nothing for a task that is judged by how things were left."""
from __future__ import annotations

import math

import numpy as np

from . import moves
from .api import Api, GaveUp


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
def _spans(surface: dict):
    """The surface's two spans from its raised edges, or which way to go to see the edge that is missing.
    Returns (spans, None) or (None, (bearing in the map's frame [deg], why))."""
    edges = [e for e in surface.get("edges_found", []) if e["kind"] == "edge of the surface" and e["length_m"] >= 0.15]
    if not edges:
        return None, (0.0, "the survey found no edge of the surface")
    way = max(edges, key=lambda e: e["length_m"])["direction_deg"]
    spans = []
    for turn in (0.0, 90.0):                                    # edges along the longest one, and edges across it
        a = math.radians(way + turn)
        normal = np.array([-math.sin(a), math.cos(a)])
        off = [float(normal @ (np.array(e["from_m"]) + e["to_m"]) / 2) for e in edges
               if abs((e["direction_deg"] - way - turn + 90) % 180 - 90) < 20]
        if len(off) < 2 or max(off) - min(off) < 0.15:
            away = -normal if (off and np.mean(off) > 0) else normal        # the side no edge was found on
            return None, (math.degrees(math.atan2(away[1], away[0])),
                          f"only one side of the surface was seen along {((way + turn) % 180):.0f} deg: its far edge is beyond what the "
                          "rover could see from where it stood")
        spans.append(max(off) - min(off) - EDGE_WIDTH_M)        # (each raised edge stands outside the surface: half its width a side)
    return spans, None


def how_big(api: Api, **_) -> dict:
    """The surface's area from its raised edges in the rover's map: two pairs of parallel edges are a rectangle. If an edge
    is out of sight from where the rover stands, it turns to face that way, drives as far as the map lets it, and looks again."""
    began = api.ok("GET", "/devices/rover/drive/state")["since_taken_over"]["heading_deg"]
    why = "the survey kept no map"
    for standpoint in range(3):
        survey = api.ok("POST", "/survey")
        surface = api.ok("GET", "/place").get("surface")
        if surface is None:
            raise GaveUp("the survey kept no map: " + str(survey.get("stood_still", {}).get("warning", "it saw too little")))
        spans, missing = _spans(surface)
        if spans is not None:
            area = spans[0] * spans[1]
            if surface["area_seen_m2"] > 1.2 * area:
                raise GaveUp(f"the surface is not a plain rectangle: {surface['area_seen_m2']:.2f} m2 of it has been seen, and the edges "
                             f"found enclose {area:.2f} m2. Its other edges are beyond what the rover could see")
            return {"area_m2": round(area, 4)}
        bearing, why = missing
        heading = api.ok("GET", "/devices/rover/drive/state")["since_taken_over"]["heading_deg"] - began      # in the map's frame
        turn = (bearing - heading + 180.0) % 360.0 - 180.0
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


BIN_SPOTS = np.array([[0.0, 0.0], [0.018, 0.018], [-0.018, -0.018], [0.018, -0.018]])      # where in a bin the first, second, ... thing goes


def into_bins(pairs=(("red", "red"), ("blue", "blue")), high: float = 0.03):
    """Carry every loose thing of a colour into the bin of a colour: pairs of (thing's colour, bin's colour).
    high: about how high the things' tops are (a thing on a shelf: its top above the surface)."""
    def solve(api: Api, **_) -> None:
        moves.locate_cameras(api)
        for colour, bin_colour in pairs:
            bin_near = moves.roughly(api, bin_colour, ring=True, height=0.012)
            bin_at, _ = moves.closely(api, bin_colour, bin_near, 0.012, ring=True)
            carried = 0
            for _ in range(4):                                  # until no loose thing of that colour is left outside the bin
                try:
                    k = next(k for k in range(3) if np.linalg.norm(moves.roughly(api, colour, pick=k, height=max(high - 0.015, 0.0)) - bin_at) > 0.06)
                except (StopIteration, GaveUp):
                    break
                thing = moves.find_thing(api, colour, pick=k, height_guess=high)
                grasp = moves.pick(api, thing)
                moves.put(api, bin_at + BIN_SPOTS[carried % len(BIN_SPOTS)], 0.0, grasp, let_go_above=0.02)     # (not onto the last one)
                carried += 1
            if not carried:
                raise GaveUp(f"no {colour} thing was found within the arm's reach to carry to the {bin_colour} bin")
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


SOLVERS = {
    ("open_small", "where_is_everyone"): where_is("rover", "arm"),
    ("moved_arm", "where_is_everyone"): where_is("rover", "arm"),
    ("l_shaped", "where_is_everyone"): where_is("rover", "arm"),
    ("no_arm", "where_is_everyone"): where_is("camera_1", "rover"),
    ("open_small", "how_big"): how_big,
    ("no_arm", "how_big"): how_big,
    ("shelf", "on_the_shelf"): on_the_shelf,
    ("moved_arm", "carried"): carried("rover", "arm"),
    ("workcell", "stack"): stack("red", "green"),
    ("cluttered", "tidy_by_colour"): into_bins(),
    ("moved_arm", "fetch"): into_bins((("red", "cream"),)),
    ("shelf", "fetch_from_shelf"): into_bins((("red", "cream"),), high=0.11),
    ("shelf", "under_the_ledge"): under_the_ledge,
}

NO_SOLUTION = {
    ("bench", "tidy"): "eight steps with a drawer, a socket and two bins; its solution still runs on the old API (sim/scripts/sim_api.py), "
                       "which is given where things are. Through the hub it needs each thing found by sight first",
    ("bench", "drawer_open"): "nothing in the hub says whether a drawer is out: it takes looking at a picture",
    ("cluttered", "fetch_hidden"): "the peg stands behind a wall, out of every camera's sight from where they are; the rover has to be "
                                   "driven round to look, and nothing here plans a path",
    ("l_shaped", "far_leg"): "the rover has to be driven round the corner to look, and nothing here plans a path",
    ("l_shaped", "how_big"): "the same: from where the rover stands it sees one leg of the L",
}
