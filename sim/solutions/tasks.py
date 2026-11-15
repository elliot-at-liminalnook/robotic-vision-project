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
def how_big(api: Api, **_) -> dict:
    """The surface's area from its raised edges in the rover's map: two pairs of parallel edges are a rectangle."""
    survey = api.ok("POST", "/survey")
    if not survey["map_kept"]:
        survey = api.ok("POST", "/survey")                      # (the head met something and turned the rover: once more)
    surface = api.ok("GET", "/place").get("surface")
    if surface is None:
        raise GaveUp("the survey kept no map: " + str(survey.get("stood_still", {}).get("warning", "it saw too little")))
    edges = [e for e in surface.get("edges_found", []) if e["kind"] == "edge of the surface" and e["length_m"] >= 0.15]
    if not edges:
        raise GaveUp("the survey found no edge of the surface")
    way = max(edges, key=lambda e: e["length_m"])["direction_deg"]
    spans = []
    for turn in (0.0, 90.0):                                    # edges along the longest one, and edges across it
        a = math.radians(way + turn)
        normal = np.array([-math.sin(a), math.cos(a)])
        off = [float(normal @ (np.array(e["from_m"]) + e["to_m"]) / 2) for e in edges
               if abs((e["direction_deg"] - way - turn + 90) % 180 - 90) < 20]
        if len(off) < 2 or max(off) - min(off) < 0.15:
            raise GaveUp(f"only one side of the surface was seen along {((way + turn) % 180):.0f} deg: its far edge is beyond what the "
                         "rover could see from where it stands")
        spans.append(max(off) - min(off) - EDGE_WIDTH_M)        # (each raised edge stands outside the surface: half its width a side)
    area = spans[0] * spans[1]
    if surface["area_seen_m2"] > 1.2 * area:
        raise GaveUp(f"the surface is not a plain rectangle: {surface['area_seen_m2']:.2f} m2 of it has been seen, and the edges found "
                     f"enclose {area:.2f} m2. Its other edges are beyond what the rover could see from where it stands")
    return {"area_m2": round(area, 4)}


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
        api.ok("POST", "/devices/arm/home")
    return solve


def sort_by_colour(colours=("red", "blue")):
    def solve(api: Api, **_) -> None:
        moves.locate_cameras(api)
        for colour in colours:
            bin_near = moves.roughly(api, colour, ring=True, height=0.012)
            bin_at, _ = moves.closely(api, colour, bin_near, 0.012, ring=True)
            for _ in range(4):                                  # until no loose thing of that colour is left outside the bin
                try:
                    near = next(p for p in (moves.roughly(api, colour, pick=k) for k in range(3))
                                if np.linalg.norm(p - bin_at) > 0.06)
                except (StopIteration, GaveUp):
                    break
                at, _ = moves.closely(api, colour, near, 0.03)
                top = moves.touch(api, at, 0.03)
                at, side = moves.closely(api, colour, at, top)
                grasp = moves.pick(api, {"at": at, "top": top, "side_deg": side})
                moves.put(api, bin_at, 0.0, grasp, let_go_above=0.02)
        api.ok("POST", "/devices/arm/home")
    return solve


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
    ("cluttered", "tidy_by_colour"): sort_by_colour(),
}

NO_SOLUTION = {
    ("bench", "tidy"): "eight steps with a drawer, a socket and two bins; its solution still runs on the old API (sim/scripts/sim_api.py), "
                       "which is given where things are. Through the hub it needs each thing found by sight first",
    ("bench", "drawer_open"): "nothing in the hub says whether a drawer is out: it takes looking at a picture",
    ("cluttered", "fetch_hidden"): "the peg stands behind a wall, out of every camera's sight from where they are; the rover has to be "
                                   "driven round to look, and nothing here plans a path",
    ("moved_arm", "fetch"): "the tray is cream on wood: these solutions' eyes go by strong colour only",
    ("shelf", "fetch_from_shelf"): "the cube is on a shelf 80 mm up: picking from a raised top is not written",
    ("shelf", "under_the_ledge"): "what lies under an overhang is not in a map of heights; it takes looking at the rover's low pictures",
    ("l_shaped", "far_leg"): "the rover has to be driven round the corner to look, and nothing here plans a path",
    ("l_shaped", "how_big"): "the same: from where the rover stands it sees one leg of the L",
}
