"""Tidying the bench through the hub's API: eight steps with a drawer, a platform, a socket and two bins.

Nothing here is told where anything is. The arm looks down with the camera on its wrist from places all round it
(`Scan`), and each thing is then found the way moves.py finds things: hover over it, look, touch it for its height,
look again. What is particular to this task:

  * the cabinet is the grey thing with white at its side (its drawer's front); the platform is the other grey thing;
  * the drawer's handle stands in the middle of that white strip, and the drawer slides square to the strip, away from
    the cabinet. A touch gives the handle's height; the jaws take it across the strip and pull;
  * a thing picked up from something raised is held a known height above its own underside, which is what setting it
    down on something else needs.
"""
from __future__ import annotations

import math

import cv2
import numpy as np

from . import eyes, moves
from .api import Api, GaveUp, NotThere, OutOfReach

RING_M = 0.24                  # the hand hovers this far out to look down, at each of BEARINGS
BEARINGS = (0.0, 35.0, -35.0, 70.0, -70.0, 105.0, -105.0)
LOOK_Z = 0.16                  # ... and this high
TAG_GROWN = 1.8                # a printed tag's outline, grown by this, covers the white border it is printed in
BESIDE_PX = 60.0               # a white patch this near a grey one in the picture is part of the same thing
BIG_GREY_PX = 30000.0          # a grey patch at least this large is a piece of furniture, not a part of the arm
SAME_M = 0.07                  # two sightings this near each other are of one thing
PULL_M = 0.083                 # how far a drawer is pulled; one that stops sooner stops the hand
PUSH_FROM_M = 0.035            # the closed jaws come down this far in front of a drawer's handle to push it home,
PUSH_BELOW_TOP_M = 0.045       # ... this far below the handle's top: against the drawer's front,
PUSH_ASIDE_M = 0.026           # ... this far to one side of the handle, along the front,
PUSH_TO_M = 0.004              # ... and are sent to this far in front of where the handle was when shut: further than they can go
HANDLE_GRIP_BELOW_M = 0.019    # the jaws take a handle this far below its top
MIN_AREA = {"grey": 4000.0}    # the smallest patch of a colour worth placing [px]; 300 for the others


class Look:
    """One picture from the wrist camera, with what the hub found of printed tags in it."""
    def __init__(self, api: Api):
        self.picture = api.picture(moves.WRIST)
        status, found = api.get(f"{moves.WRIST}/picture?tags=true")
        tags = [np.array(t["corners_px"], np.float32) for t in found.get("tags", [])] if status == 200 else []
        self.tags = [(c.mean(axis=0) + TAG_GROWN * (c - c.mean(axis=0))).reshape(-1, 1, 2) for c in tags]

    def find(self, colour: str, min_area: float = 300.0) -> list:
        """Patches of that colour that are things: not on a printed tag."""
        return [b for b in eyes.find(self.picture, colour, min_area) if not b.tag
                and not any(cv2.pointPolygonTest(t, (float(b.u), float(b.v)), False) >= 0 for t in self.tags)]

    def off_centre(self, b) -> float:
        return math.hypot(b.u - self.picture.shape[1] / 2, b.v - self.picture.shape[0] / 2)


def beside(a, b, within_px: float = BESIDE_PX) -> bool:
    """Whether patch a comes within some pixels of patch b in the picture."""
    outline = b.outline.reshape(-1, 1, 2).astype(np.float32)
    return any(cv2.pointPolygonTest(outline, (float(q[0]), float(q[1])), True) > -within_px for q in a.outline[::max(1, len(a.outline) // 16)])


def middle(api: Api, b, height: float):
    """The middle of a patch's outline (not of its area: a frame with a thing in it is lopsided), on the plane at a height.
    Asked while the hand is still where the picture was taken: the hub places a point from where the wrist camera is now."""
    c = b.corners.mean(axis=0)
    return moves._place(api, "arm/wrist_camera", float(c[0]), float(c[1]), height)[0]


class Scan:
    """What the wrist camera sees from all round the arm: every patch of the colours asked for, placed on the plane at the
    height given for its colour."""
    def __init__(self, api: Api, heights: dict):
        self.seen = []                                           # (colour, place, patch, look)
        for bearing in BEARINGS:
            way = np.array([math.cos(math.radians(bearing)), math.sin(math.radians(bearing))])
            try:
                moves.hover_over(api, RING_M * way, hover=LOOK_Z)
            except OutOfReach:
                continue
            look = Look(api)
            for colour, height in heights.items():
                for b in look.find(colour, MIN_AREA.get(colour, 300.0))[:8]:
                    at = middle(api, b, height)
                    if at is not None:
                        self.seen.append((colour, at, b, look))
        if not self.seen:
            raise GaveUp("looking down from all round the arm, the wrist camera saw nothing to tidy")

    def things(self, colour: str, ring: bool = False, whole: bool = True, min_area: float = 0.0) -> list:
        """One (place, patch, look) per thing of a colour, each from the view that had it nearest the picture's middle."""
        out = []
        for c, at, b, look in sorted(self.seen, key=lambda s: s[3].off_centre(s[2])):
            if c != colour or (whole and b.cut_off) or (ring and not b.ring) or b.area < min_area:
                continue
            if not any(np.linalg.norm(at - o[0]) < SAME_M for o in out):
                out.append((at, b, look))
        return out


def furniture(scan: Scan):
    """The cabinet (grey, with the white of a drawer's front at its side) and the platform (the other large grey thing):
    (where the cabinet is, where its drawer's front is, where the platform is), roughly."""
    grey = [(at, b, look) for c, at, b, look in scan.seen if c == "grey" and b.area >= BIG_GREY_PX]
    fronts = []
    for at, b, look in grey:
        white = [(w_at, w) for c, w_at, w, l in scan.seen if c == "white" and l is look and not w.cut_off and beside(w, b)]
        if white:
            fronts.append((look.off_centre(white[0][1]), at, np.mean([w_at for w_at, _ in white], axis=0)))
    if not fronts:
        raise GaveUp("no grey thing with the white of a drawer at its side was seen: no cabinet")
    _, cabinet, front = min(fronts, key=lambda f: f[0])
    others = [(look.off_centre(b) + (5000.0 if b.cut_off else 0.0), at) for at, b, look in grey if np.linalg.norm(at - cabinet) > 0.15]
    if not others:
        raise GaveUp("no grey thing other than the cabinet was seen: no platform")
    return cabinet, front, min(others, key=lambda o: o[0])[1]


# ---------------------------------------------------------------- looking again from above
def above(api: Api, at, z: float = LOOK_Z) -> Look:
    moves.hover_over(api, at, hover=z)
    return Look(api)


def centre_of(api: Api, colour: str, near, height: float, min_area: float = 300.0, within: float = 0.08):
    """Hover over `near` and give the middle of the patch of that colour nearest it (by its outline), on the plane at height."""
    look = above(api, near)
    got = [(b, middle(api, b, height)) for b in look.find(colour, min_area)[:6] if not b.cut_off]
    got = [(b, at) for b, at in got if at is not None and np.linalg.norm(at - near) < within]
    if not got:
        raise NotThere(f"hovering over ({near[0]:.3f}, {near[1]:.3f}) the wrist camera sees no whole {colour} thing within {within * 1000:.0f} mm")
    return min(got, key=lambda g: np.linalg.norm(g[1] - near))[1]


# ---------------------------------------------------------------- the drawer
class Drawer:
    """A drawer found by its front: where its handle is, which way it slides out, how high the handle stands."""
    def __init__(self, api: Api, cabinet, front):
        self.api = api
        look = above(api, front)
        pieces = [b for b in look.find("white") if not b.cut_off]
        corners = [(b, [moves._place(api, "arm/wrist_camera", float(c[0]), float(c[1]), 0.04)[0] for c in b.corners]) for b in pieces]
        pts = np.array([p for b, cs in corners if all(c is not None for c in cs) and np.linalg.norm(np.mean(cs, axis=0) - front) < 0.07 for p in cs])
        if len(pts) < 4:
            raise GaveUp(f"hovering over the drawer's front at ({front[0]:.3f}, {front[1]:.3f}), the wrist camera does not see it")
        mid = pts.mean(axis=0)
        _, _, axes = np.linalg.svd(pts - mid)
        self.along = axes[0] / np.linalg.norm(axes[0])           # the way the front runs
        reach = (pts - mid) @ self.along
        self.handle = mid + self.along * (reach.max() + reach.min()) / 2      # the middle of the front, end to end
        out = np.array([-self.along[1], self.along[0]])
        self.out = out if (self.handle - cabinet) @ out > 0 else -out         # square to the front, away from the cabinet
        self.jaws = math.degrees(math.atan2(self.along[1], self.along[0])) % 180.0
        self.top = moves.touch(api, self.handle, 0.06, start=0.12)
        self.shut_at = self.handle.copy()

    def slide(self, to):
        """Take the handle where it is and bring it to `to`. The handle is where the hand leaves it."""
        api, z = self.api, self.top - HANDLE_GRIP_BELOW_M
        x, y = float(self.handle[0]), float(self.handle[1])
        api.ok("POST", "/devices/arm/gripper/set", {"gap_m": 0.028})
        moves._go(api, x, y, [self.top + 0.03, self.top + 0.02], jaws_along_deg=self.jaws)        # (jaws across the handle, along the front)
        moves._go(api, x, y, z)
        if not api.ok("POST", "/devices/arm/gripper/set", {"opening": 0.0}).get("holding"):
            raise GaveUp(f"the jaws closed on nothing where the drawer's handle should be, at ({x:.3f}, {y:.3f}, {z:.3f})")
        api.post("/devices/arm/move_tip", {"x": float(to[0]), "y": float(to[1]), "z": z, "timeout_s": 8.0})
        # Where the handle is now: between the jaws, while they still hold it. (The tip is the middle between the jaws:
        # once they open it is somewhere else, 8 mm to one side, and the next grasp taken from there missed the handle.)
        tip = api.ok("GET", "/devices/arm/state")["tip_m"]
        went = np.array([tip["x"], tip["y"]]) - self.handle
        self.handle = np.array([tip["x"], tip["y"]])
        api.ok("POST", "/devices/arm/gripper/set", {"gap_m": 0.030})
        tip = api.ok("GET", "/devices/arm/state")["tip_m"]
        api.post("/devices/arm/move_tip", {"x": tip["x"], "y": tip["y"], "z": self.top + 0.03})
        return float(np.linalg.norm(went))

    def open(self) -> float:
        return self.slide(self.shut_at + PULL_M * self.out)

    def shut(self) -> float:
        """Bring the handle back to where it was found, then push the drawer home by its front. Held by its handle the
        drawer stops 8 or 9 mm short: the handle sits a few millimetres off the jaws' middle on the way back, and the hand
        meets the cabinet before the drawer is in. The closed jaws against the front have nothing else in their way; a
        drawer that is shut stops them. They push beside the handle, not under it: under it the fingers catch the
        handle's underside, and the drawer came back out a centimetre with the hand."""
        api = self.api
        went = self.slide(self.shut_at)
        api.ok("POST", "/devices/arm/gripper/set", {"opening": 0.0})
        aside = PUSH_ASIDE_M * self.along
        before = self.handle + PUSH_FROM_M * self.out + aside
        low = max(self.top - PUSH_BELOW_TOP_M, 0.012)
        moves._go(api, float(before[0]), float(before[1]), [self.top + 0.03, self.top + 0.02], jaws_along_deg=self.jaws)
        moves._go(api, float(before[0]), float(before[1]), low)
        to = self.shut_at + PUSH_TO_M * self.out + aside
        api.post("/devices/arm/move_tip", {"x": float(to[0]), "y": float(to[1]), "z": low, "timeout_s": 6.0})
        tip = api.ok("GET", "/devices/arm/state")["tip_m"]
        back = np.array([tip["x"], tip["y"]]) + 0.02 * self.out                 # (off the front before lifting: the handle is above)
        api.post("/devices/arm/move_tip", {"x": float(back[0]), "y": float(back[1]), "z": low})
        api.post("/devices/arm/move_tip", {"x": float(back[0]), "y": float(back[1]), "z": self.top + 0.03})
        self.handle = self.shut_at.copy()
        return went

    def inside(self) -> np.ndarray:
        """About the middle of the tray, when the drawer is out."""
        return self.handle - 0.045 * self.out


# ---------------------------------------------------------------- carrying
def take(api: Api, colour: str, near, height_guess: float, stands_on: float = 0.0, snug: bool = False, lengthwise: bool = False):
    """Find the thing of a colour near a place, pick it up, and say how high the jaws hold it above its own underside.
    snug: it stands where jaws opened wide would not fit (in a drawer). lengthwise: it is low and long, and is taken by its ends."""
    thing = moves.find_thing(api, colour, height_guess=height_guess, near=np.asarray(near, float), lengthwise=lengthwise)
    grasp = moves.pick(api, thing, snug=snug)
    return thing, max(grasp - stands_on, 0.0)


def stands_on(api: Api, thing: dict, beside_m: float = 0.032) -> float:
    """How high what a thing stands on is: a touch just beside the thing, on the side towards the arm, or failing that
    (the touch finds something as high as the thing, or the hand cannot get there) on the side away from it."""
    way = thing["at"] / max(float(np.linalg.norm(thing["at"])), 1e-6)
    for sign in (-1.0, 1.0):
        under = moves.touch(api, thing["at"] + sign * beside_m * way, -1.0, start=thing["top"] + 0.05)
        if 0.0 <= under < thing["top"] - 0.008:
            return under
    return 0.0


def tidy(api: Api, **_) -> None:
    if "arm" not in api.devices():
        raise GaveUp("this takes an arm")
    joints = api.ok("GET", "/devices/arm/state")["joints"]["joints_rad"]
    api.rest = {k: v for k, v in joints.items() if k != "gripper"}
    # where each kind of thing is taken to be, in height, for a first placing: the scan only says roughly where
    scan = Scan(api, {"grey": 0.05, "white": 0.04, "green": 0.02, "purple": 0.085, "red": 0.012, "blue": 0.03, "yellow": 0.03})
    cabinet, front, platform = furniture(scan)

    def one(colour, what, ring=False, not_near=()):
        """Roughly where the largest thing of a colour is. A thing further out than the hand hovered is at the edge of
        every picture, part of it cut off: then where that part is will do, for a place to go and look from above."""
        for whole in (True, False):
            got = [t for t in scan.things(colour, ring=ring and whole, whole=whole) if all(np.linalg.norm(t[0] - p) > 0.06 for p in not_near)]
            if got:
                return max(got, key=lambda t: t[1].area)[0]
        raise GaveUp(f"the wrist camera saw no {what} from anywhere round the arm")

    # 1. the green block onto the platform
    platform = centre_of(api, "grey", platform, 0.04, min_area=BIG_GREY_PX)
    platform_top = moves.touch(api, platform, 0.04, start=0.11)
    platform = centre_of(api, "grey", platform, platform_top, min_area=BIG_GREY_PX)
    block, held = take(api, "green", one("green", "green block"), 0.02, lengthwise=True)
    moves.put(api, platform, platform_top, held)
    # 2. the purple cube, from the cabinet's roof, onto the block
    block_at = centre_of(api, "green", platform, platform_top + block["top"])
    block_top = moves.touch(api, block_at, platform_top + block["top"], start=platform_top + block["top"] + 0.07)
    cube = moves.find_thing(api, "purple", height_guess=0.085, near=one("purple", "purple cube"))
    roof = stands_on(api, cube)                                  # (what the cube stands on: the jaws' height above its underside)
    held = max(moves.pick(api, cube) - roof, 0.0)
    moves.put(api, block_at, block_top, held)
    # 3. the drawer out, the red cube from it to the red bin, the drawer shut
    drawer = Drawer(api, cabinet, front)
    drawer.open()
    red_bin = one("red", "red bin", ring=True)
    red_bin, _ = moves.closely(api, "red", red_bin, 0.012, ring=True)
    cube, held = take(api, "red", drawer.inside(), 0.034, snug=True)        # (between the drawer's front and the cabinet)
    moves.put(api, red_bin, 0.0, held, let_go_above=0.01)
    drawer.shut()
    # 4. the blue cylinder into the blue bin
    blue_bin = one("blue", "blue bin", ring=True)
    blue_bin, _ = moves.closely(api, "blue", blue_bin, 0.012, ring=True)
    cylinder, held = take(api, "blue", one("blue", "blue cylinder", not_near=[blue_bin]), 0.06)
    moves.put(api, blue_bin, 0.0, held, let_go_above=0.006)
    # 5. the yellow peg into the socket: the socket is the large yellow thing, the peg the small one
    yellow = sorted(scan.things("yellow"), key=lambda t: -t[1].area)
    if len(yellow) < 2:
        raise GaveUp("the wrist camera saw fewer than two whole yellow things: a socket and a peg were looked for")
    socket = centre_of(api, "yellow", yellow[0][0], 0.025, min_area=3000.0)
    socket = centre_of(api, "yellow", socket, 0.025, min_area=3000.0)          # (from straight above, its height matters less)
    for at, b, _ in yellow[1:]:
        try:
            peg, held = take(api, "yellow", at, 0.06)
            break
        except (NotThere, OutOfReach):
            continue                                             # (a part of the arm: it is printed yellow)
    else:
        raise GaveUp("none of the small yellow patches seen was a peg")
    moves.put(api, socket, 0.0, held, let_go_above=0.004)
    moves.park(api)
