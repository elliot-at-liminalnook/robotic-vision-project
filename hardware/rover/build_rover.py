#!/usr/bin/env python3
"""The rover: a two-wheeled base with a motorised stereo head (pan, lift, tilt, camera spacing).

    <CadQuery env>/python hardware/rover/build_rover.py        # a few minutes: it checks every part against every other

Writes, all in this directory:
  parts/<name>.step, parts/<name>.stl   the printed parts, each one solid, in its body's frame (mm)
  rover.json                            the assembly: bodies, joints, masses, collision shapes, cameras.
                                        The simulation's rover is generated from this file (sim/so101sim/rover_model.py)
  build_report.json                     the checks: solids, interference over the axes' travel, wire lengths, ranges

Frames (mm): the rover's frame has its origin on the surface under the middle of the wheel axle,
x forward, y left, z up. Each body's frame is the rover's frame at every joint = 0, moved to that
body's joint. Joints, all on HX-30HM servos:

  wheel_left, wheel_right   ids 11, 12   rear-wheel drive; a ball caster at the front, a skid at the back
  pan                       id 21        the head's mast turns about the vertical
  lift                      id 22        an arm raises the head; a second, coupled joint keeps the head level
  tilt                      id 23        the head looks up and down
  spacing                   id 24        a pinion between two racks moves the cameras apart, evenly

What this is and is not: geometry that fits together and clears itself over its whole travel, with
masses and collision shapes for the simulation. Not yet: screw holes and bosses for every joint,
print tolerances, a tested print. See README.md.
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
from pathlib import Path

import cadquery as cq
import numpy as np

HERE = Path(__file__).resolve().parent
PLA_G_MM3, FILL = 1.24e-3, 0.4                # density, and the share of solid PLA a typical print has (DECISIONS W8)

# ---- bought parts (mm, g) ------------------------------------------------------------------------
SERVO = dict(depth=29.0, width=25.0, height=45.0, shaft_from_end=10.25, horn_d=20.0, horn_t=5.5, idler_t=3.0, mass=52.0)
BATTERY = dict(size=(154.0, 42.5, 24.7), mass=355.0)
BOARD = dict(size=(71.0, 21.0, 15.0), mass=12.0)           # ESP32-P4 board with its connectors (hardware/wrist_mount's envelope)
CAMERA = dict(size=(11.5, 25.0, 24.0), mass=4.0)           # Camera Module 3 Wide: depth, width, height
WHEEL_D, WHEEL_W, AXLE_Z, TRACK = 60.0, 12.0, 30.0, 128.0
ARM_L = 110.0
PITCH_R = 12.0                                             # the spacing pinion's pitch radius: 12 mm of camera travel per radian
PAN_X = 20.0
LIMITS = {"pan": (-170.0, 170.0), "lift": (-15.0, 90.0), "tilt": (-60.0, 30.0), "spacing": (30.0, 100.0)}   # deg, deg, deg, mm (each camera from the centre)


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl(axis, centre, d, length):
    """A cylinder of diameter d, centred on `centre`, along 'x', 'y' or 'z'."""
    w = {"x": "YZ", "y": "XZ", "z": "XY"}[axis]
    c = cq.Workplane(w).circle(d / 2).extrude(length / 2, both=True)
    return c.translate(tuple(centre))


def servo_solid(shaft_axis, shaft_point, long_axis, long_sign=+1, out_sign=+1):
    """A servo as its body and two horns. Its output shaft lies along shaft_axis through shaft_point; its long
    (45 mm) side lies along long_axis with the shaft `shaft_from_end` from the end on the long_sign side;
    the output horn is on the out_sign side. Returns (body, horns, body bounds)."""
    ax = "xyz".index(shaft_axis)
    lx = "xyz".index(long_axis)
    wx = 3 - ax - lx
    lo, hi = [0.0] * 3, [0.0] * 3
    lo[ax], hi[ax] = shaft_point[ax] - SERVO["depth"] / 2, shaft_point[ax] + SERVO["depth"] / 2
    end = shaft_point[lx] + long_sign * SERVO["shaft_from_end"]
    lo[lx], hi[lx] = sorted((end, end - long_sign * SERVO["height"]))
    lo[wx], hi[wx] = shaft_point[wx] - SERVO["width"] / 2, shaft_point[wx] + SERVO["width"] / 2
    body = box(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])
    c_out, c_idl = list(shaft_point), list(shaft_point)
    c_out[ax] += out_sign * (SERVO["depth"] / 2 + SERVO["horn_t"] / 2)
    c_idl[ax] -= out_sign * (SERVO["depth"] / 2 + SERVO["idler_t"] / 2)
    horns = cyl(shaft_axis, c_out, SERVO["horn_d"], SERVO["horn_t"]).union(cyl(shaft_axis, c_idl, SERVO["horn_d"], SERVO["idler_t"]))
    return body, horns, (lo, hi)


class Body:
    def __init__(self, name, parent=None, origin=(0, 0, 0), joint=None):
        self.name, self.parent, self.origin, self.joint = name, parent, np.array(origin, float), joint
        self.prints: dict[str, cq.Workplane] = {}       # printed parts (rover frame at zero pose)
        self.bought: list[dict] = []                    # {"name", "lo", "hi", "mass", "rgba"} boxes, rover frame
        self.extra: list[cq.Workplane] = []             # bought shapes that are not boxes (horns), for the interference check
        self.collision: list[dict] = []
        self.cameras: list[dict] = []

    def buy(self, name, lo, hi, mass, rgba):
        self.bought.append({"name": name, "lo": [float(v) for v in lo], "hi": [float(v) for v in hi], "mass": mass, "rgba": rgba})

    def servo(self, name, *a, **k):
        body, horns, (lo, hi) = servo_solid(*a, **k)
        self.buy(name, lo, hi, SERVO["mass"], [0.1, 0.1, 0.12, 1])
        self.extra.append(horns)
        return lo, hi

    def solid(self) -> cq.Workplane:
        """Everything this body is made of, as one shape in the rover frame at zero pose."""
        shapes = list(self.prints.values()) + self.extra + [box(b["lo"][0], b["hi"][0], b["lo"][1], b["hi"][1], b["lo"][2], b["hi"][2]) for b in self.bought]
        out = shapes[0]
        for s in shapes[1:]:
            out = out.union(s)
        return out


def build() -> dict[str, Body]:
    B: dict[str, Body] = {}
    half = TRACK / 2

    # ---- chassis ------------------------------------------------------------------------------------
    c = B["chassis"] = Body("chassis")
    base = box(-48, 136, -54, 54, 9, 12)
    for sy in (1, -1):                                    # cradles under the wheel servos, rails beside the battery
        base = base.union(box(-10.25, 34.75, sy * 23 if sy > 0 else -52, 52 if sy > 0 else -23, 12, 17.5))
        base = base.union(box(40, 132, sy * 22.2 if sy > 0 else -25.2, 25.2 if sy > 0 else -22.2, 12, 20))
        for px in (-44, 58):                              # posts up to the deck
            base = base.union(box(px - 4, px + 4, sy * 50 - 4, sy * 50 + 4, 12, 44))
        base = base.union(box(70, 108, sy * 51 if sy > 0 else -54, 54 if sy > 0 else -51, 9, 48))       # side tag plates
    base = base.union(box(-48, -45, -19, 19, 9, 48))      # rear tag plate
    base = base.union(cyl("z", (128, 0, 8.5), 14, 5).union(cyl("z", (-42, 0, 8.25), 10, 5.5)))        # caster and skid bosses
    c.prints["chassis_base"] = base
    deck = box(-48, 62, -54, 54, 44, 47)
    deck = deck.union(box(-48, -18, -37, 37, 62, 65)).union(box(-48, -45, -37, 37, 47, 62)).union(box(-21, -18, -37, 37, 47, 62))   # cover over the controller board, with the top tag
    c.prints["deck"] = deck
    c.buy("battery", (-22, -21.25, 12), (132, 21.25, 36.7), BATTERY["mass"], [0.15, 0.15, 0.6, 1])
    c.servo("wheel_servo_left", "y", (0, half - 26.5, AXLE_Z), "x", long_sign=-1, out_sign=+1)
    c.servo("wheel_servo_right", "y", (0, -half + 26.5, AXLE_Z), "x", long_sign=-1, out_sign=-1)
    c.servo("pan_servo", "z", (PAN_X, 0, 61.5), "x", long_sign=+1, out_sign=+1)
    c.buy("controller_board", (-44.5, -35.5, 47), (-26.5, 35.5, 62), BOARD["mass"], [0.1, 0.45, 0.2, 1])
    c.collision += [dict(type="box", lo=[-48, -54, 9], hi=[136, 54, 47]),
                    dict(type="box", lo=[-48, -37, 47], hi=[-18, 37, 65]),
                    dict(type="box", lo=[PAN_X - 34.75, -12.5, 47], hi=[PAN_X + 10.25, 12.5, 81.5]),
                    dict(type="sphere", centre=[128, 0, 6], radius=6, friction=0.02, name="caster"),
                    dict(type="sphere", centre=[-42, 0, 5.5], radius=4, friction=0.05, name="skid")]

    # ---- wheels -------------------------------------------------------------------------------------
    for side, sy, sid in (("left", 1, 11), ("right", -1, 12)):
        y = sy * (half + 1.0)                             # the wheel sits on the output horn
        w = B[f"wheel_{side}"] = Body(f"wheel_{side}", "chassis", (0, y, AXLE_Z),
                                      dict(name=f"wheel_{side}", type="hinge", axis=[0, 1, 0], servo=sid, direction=1,
                                           mode="wheel", ratio=1.0))
        wheel = cyl("y", (0, y, AXLE_Z), WHEEL_D - 6, WHEEL_W)
        wheel = wheel.cut(cyl("y", (0, y - sy * (WHEEL_W / 2 - SERVO["horn_t"] / 2 + 0.01), AXLE_Z), SERVO["horn_d"] + 0.4, SERVO["horn_t"]))
        w.prints[f"wheel_{side}"] = wheel
        w.collision.append(dict(type="ellipsoid", centre=[0, y, AXLE_Z], radii=[WHEEL_D / 2, WHEEL_W / 2, WHEEL_D / 2], friction=0.9))
        w.tyre_mass = 8.0                                 # an O-ring tyre in the rim's groove (not drawn)

    # ---- turret: turns on the pan servo's horn, carries the lift servo -----------------------------
    pan_z = 81.5
    t = B["turret"] = Body("turret", "chassis", (PAN_X, 0, pan_z),
                           dict(name="pan", type="hinge", axis=[0, 0, 1], servo=21, direction=1, ratio=1.0, range_deg=LIMITS["pan"]))
    lift_z = pan_z + 19.5
    t.prints["turret"] = cyl("z", (PAN_X, 0, pan_z + 2), 44, 4).union(box(PAN_X - 34.75, PAN_X + 10.25, -14.5, 14.5, pan_z + 4, lift_z - 12.5))
    t.servo("lift_servo", "y", (PAN_X, 0, lift_z), "x", long_sign=+1, out_sign=+1)
    t.collision += [dict(type="box", lo=[PAN_X - 34.75, -17.5, pan_z], hi=[PAN_X + 10.25, 20, lift_z + 12.5])]

    # ---- lift arm: two bars, one on the lift servo's horn, one on its idler --------------------------
    a = B["arm"] = Body("arm", "turret", (PAN_X, 0, lift_z),
                        dict(name="lift", type="hinge", axis=[0, -1, 0], servo=22, direction=1, ratio=1.0, range_deg=LIMITS["lift"]))
    # The bars run outside the planes the head's yoke plates move in (y 20..23 and -20.5..-17.5), on spacers from the horns.
    arm = box(PAN_X - 8, PAN_X + ARM_L + 6, 24, 27, lift_z - 7, lift_z + 7).union(box(PAN_X - 8, PAN_X + ARM_L + 6, -24.5, -21.5, lift_z - 7, lift_z + 7))
    arm = arm.union(cyl("y", (PAN_X, 22, lift_z), 14, 4)).union(cyl("y", (PAN_X, -19.5, lift_z), 14, 4))
    arm = arm.union(box(PAN_X + 45, PAN_X + 55, -24.5, 27, lift_z - 7, lift_z - 3))
    arm = arm.cut(cyl("y", (PAN_X + ARM_L, 0, lift_z), 6.4, 60))                                # holes for the head block's pins
    a.prints["lift_arm"] = arm
    a.collision += [dict(type="box", lo=[PAN_X - 8, 24, lift_z - 7], hi=[PAN_X + ARM_L + 6, 27, lift_z + 7]),
                    dict(type="box", lo=[PAN_X - 8, -24.5, lift_z - 7], hi=[PAN_X + ARM_L + 6, -21.5, lift_z + 7])]

    # ---- head block: hangs level at the arm's end, carries the tilt servo ---------------------------
    hx = PAN_X + ARM_L
    h = B["head_block"] = Body("head_block", "arm", (hx, 0, lift_z),
                               dict(name="level", type="hinge", axis=[0, -1, 0], coupled_to="lift", ratio=-1.0))
    tilt_x = hx + 20.0
    h.servo("tilt_servo", "y", (tilt_x, 0, lift_z), "x", long_sign=+1, out_sign=+1)
    block = box(hx - 14.75, hx + 24, -14.5, 14.5, lift_z - 15.5, lift_z - 12.5)        # (short of the yoke's cross piece when the head looks down)
    block = block.union(cyl("y", (hx, 20.75, lift_z), 6, 12.5)).union(cyl("y", (hx, -19.5, lift_z), 6, 10.0))    # the pins the arm's bars turn on
    block = block.union(box(hx - 3, hx + 3, 14.5, 17.0, lift_z - 15.5, lift_z)).union(box(hx - 3, hx + 3, -17.0, -14.5, lift_z - 15.5, lift_z))
    h.prints["head_block"] = block
    h.collision += [dict(type="box", lo=[hx - 14.75, -17.5, lift_z - 15.5], hi=[hx + 30.25, 20, lift_z + 12.5])]

    # ---- head: tilts on the tilt servo; a beam, the spacing servo above it, the pinion under it ------
    T = np.array([tilt_x, 0.0, lift_z])                   # the tilt axis; head parts are given relative to it
    hd = B["head"] = Body("head", "head_block", T, dict(name="tilt", type="hinge", axis=[0, -1, 0], servo=23, direction=1,
                                                         ratio=1.0, range_deg=LIMITS["tilt"]))

    def tb(x0, x1, y0, y1, z0, z1):
        return box(T[0] + x0, T[0] + x1, y0, y1, T[2] + z0, T[2] + z1)
    # The yoke: a plate on each of the tilt servo's horns, joined in front of the servo, then a spine to the beam's
    # centre pad. Everything of the head but the plates is further from the tilt axis than the servo's corners
    # (16.2 mm), so it clears the servo at any tilt; and the spine is narrower than the carriages' closest approach.
    head = tb(-10, 21.5, 20, 23, -8, 8).union(tb(-10, 21.5, -20.5, -17.5, -8, 8)).union(tb(17.5, 21.5, -20.5, 23, -8, 8))
    head = head.union(tb(21.5, 50, -6, 6, -6, 6))
    head = head.union(tb(58, 70, -118, 118, -6, 6)).union(tb(50, 78, -13, 13, -6, 6))         # the beam and its centre pad
    head = head.cut(cyl("z", (T[0] + 64, 0, T[2]), 21, 14))                                   # bore for the spacing servo's horn
    hd.prints["head_beam"] = head
    lo, hi = hd.servo("spacing_servo", "z", (T[0] + 64, 0, T[2] + 23.5), "y", long_sign=+1, out_sign=-1)
    # (drawn at its root diameter: the teeth, not drawn, reach the racks' faces)
    hd.prints["pinion"] = cyl("z", (T[0] + 64, 0, T[2] - 15), 2 * PITCH_R - 3.5, 11).union(cyl("z", (T[0] + 64, 0, T[2] - 3), 8, 13))
    hd.prints["head_tag_plate"] = box(lo[0] - 2.5, hi[0] + 2.5, -15, 15, hi[2], hi[2] + 2)    # on top of the spacing servo
    hd.collision += [dict(type="box", lo=[T[0] + 58, -118, T[2] - 6], hi=[T[0] + 70, 118, T[2] + 6]),
                     dict(type="box", lo=[lo[0], lo[1], T[2] + 6], hi=[hi[0], hi[1], hi[2] + 2]),
                     dict(type="box", lo=[T[0] - 10, 20, T[2] - 8], hi=[T[0] + 21.5, 23, T[2] + 8]),
                     dict(type="box", lo=[T[0] - 10, -20.5, T[2] - 8], hi=[T[0] + 21.5, -17.5, T[2] + 8]),
                     dict(type="box", lo=[T[0] + 17.5, -20.5, T[2] - 8], hi=[T[0] + 58, 23, T[2] + 8])]

    # ---- carriages: slide along the beam; each carries a rack, a camera and its board --------------
    c0 = LIMITS["spacing"][0]                             # drawn at their closest
    for side, sy in (("left", 1), ("right", -1)):
        yc = sy * c0
        cr = B[f"carriage_{side}"] = Body(f"carriage_{side}", "head", T + [0, yc, 0],
                                          dict(name="spacing" if side == "left" else "spacing_right", type="slide", axis=[0, sy, 0],
                                               **(dict(servo=24, direction=1, ratio=PITCH_R / 1000.0, zero_ticks=148,   # its 334 deg of travel inside one turn
                                                       range_m=[0.0, (LIMITS["spacing"][1] - c0) / 1000.0]) if side == "left"
                                                  else dict(coupled_to="spacing", ratio=1.0))))
        y0, y1 = yc - 15, yc + 15
        car = tb(54, 74, y0, y1, -9, 9).cut(tb(57.7, 70.3, y0 - 1, y1 + 1, -6.3, 6.3))        # a sleeve round the beam
        car = car.union(tb(74, 87, y0, y1, -9, 9)).union(tb(83, 87, y0, y1, -62, 9))          # a bridge over the front rack's path, and the camera plate
        if side == "left":                                # front rack, teeth toward the pinion, reaching across the centre
            ra, rb = sorted((yc - sy * 2, yc - sy * 108))
            car = car.union(tb(74.5, 82, ra, rb, -21, -9))
        else:                                             # rear rack, on a ledge behind the beam
            ra, rb = sorted((yc - sy * 2, yc - sy * 108))
            car = car.union(tb(46, 54, y0, y1, -9, 0)).union(tb(46, 53.5, ra, rb, -21, -9))
        cr.prints[f"carriage_{side}"] = car
        cr.buy(f"camera_{side}", (T[0] + 87, yc - 12.5, T[2] - 58), (T[0] + 98.5, yc + 12.5, T[2] - 34), CAMERA["mass"], [0.2, 0.2, 0.25, 1])
        cr.buy(f"camera_board_{side}", (T[0] + 87, yc - 10.5, T[2] - 30), (T[0] + 102, yc + 10.5, T[2] + 41), BOARD["mass"], [0.1, 0.45, 0.2, 1])
        cr.cameras.append(dict(name=f"rover_{side}", lens=[T[0] + 98.5, yc, T[2] - 46], looks=[1, 0, 0], up=[0, 0, 1]))
        cr.collision += [dict(type="box", lo=[T[0] + 54, y0, T[2] - 9], hi=[T[0] + 87, y1, T[2] + 9]),
                         dict(type="box", lo=[T[0] + 83, y0, T[2] - 62], hi=[T[0] + 102, y1, T[2] + 41])]
    return B


# ---- placing bodies at joint values ---------------------------------------------------------------
def chain(B, name):
    out = []
    while name is not None:
        out.append(B[name])
        name = B[name].parent
    return out[::-1]


def joint_value(body, q):
    j = body.joint
    if j is None:
        return 0.0
    if "coupled_to" in j:
        return q.get(j["coupled_to"], 0.0) * j["ratio"]
    return q.get(j["name"], 0.0)


def placed(B, name, q, shape=None):
    """A body's shape moved to where joint values q put it. q: pan, lift, tilt [deg]; spacing [mm from closest]."""
    shape = shape if shape is not None else B[name].solid()
    for body in chain(B, name)[::-1]:                     # innermost joint first
        j = body.joint
        if j is None:
            continue
        v = joint_value(body, q)
        o = tuple(body.origin)
        if j["type"] == "hinge":
            shape = shape.rotate(o, tuple(body.origin + np.array(j["axis"], float)), v)
        else:
            shape = shape.translate(tuple(np.array(j["axis"], float) * v))
    return shape


def lowest_and_lens(B, q):
    lens = B["carriage_left"].cameras[0]["lens"]
    p = placed(B, "carriage_left", q, cq.Workplane("XY").sphere(0.5).translate(tuple(lens)))
    low = min(placed(B, n, q).val().BoundingBox().zmin for n in ("head", "carriage_left", "carriage_right", "head_block", "arm"))
    return low, p.val().Center().z, p.val().Center().x


def interference(B):
    """Volume shared by any two bodies over the axes' travel [mm3], for the pairs that could meet."""
    upper = ["turret", "arm", "head_block", "head", "carriage_left", "carriage_right"]
    lower = ["chassis", "wheel_left", "wheel_right"]
    solids = {n: B[n].solid() for n in B}
    grid_head = [dict(lift=l, tilt=t, spacing=s) for l in (LIMITS["lift"][0], 20, 55, LIMITS["lift"][1]) for t in (LIMITS["tilt"][0], -15, LIMITS["tilt"][1])
                 for s in (0.0, 35.0, LIMITS["spacing"][1] - LIMITS["spacing"][0])]
    grid_pan = [dict(pan=p, lift=l, tilt=t, spacing=s) for p in (-170, -120, -60, 0, 60, 120, 170) for l in (LIMITS["lift"][0], 30)
                for t in (LIMITS["tilt"][0], 0) for s in (0.0, LIMITS["spacing"][1] - LIMITS["spacing"][0])]
    worst: dict[str, float] = {}

    def check(q, pairs):
        at = {n: placed(B, n, q, solids[n]) for n in {m for p in pairs for m in p}}
        bb = {n: s.val().BoundingBox() for n, s in at.items()}
        for a, b in pairs:
            A, Bb = bb[a], bb[b]
            if A.xmax < Bb.xmin or Bb.xmax < A.xmin or A.ymax < Bb.ymin or Bb.ymax < A.ymin or A.zmax < Bb.zmin or Bb.zmax < A.zmin:
                continue
            v = at[a].intersect(at[b]).val().Volume()
            if v > 1.0:
                key = f"{a} / {b}"
                if v > worst.get(key, 0.0):
                    worst[key] = round(v, 1)
                    worst[key + " at"] = q
    for q in grid_head:
        check(q, list(itertools.combinations(upper, 2)))
    for q in grid_pan:
        check(q, [(u, l) for u in upper for l in lower])
    check({}, list(itertools.combinations(lower, 2)))
    return worst, len(grid_head) + len(grid_pan) + 1


def mass_props(body: Body):
    """(mass kg, centre of mass m, inertia about it kg.m2 as xx yy zz xy xz yz) of everything in the body."""
    comps = []
    for shape in body.prints.values():
        s = shape.val()
        m = s.Volume() * PLA_G_MM3 * FILL
        I = np.array([[cq.Shape.matrixOfInertia(s)[i][j] for j in range(3)] for i in range(3)]) * PLA_G_MM3 * FILL
        comps.append((m, np.array(cq.Shape.centerOfMass(s).toTuple()), I))
    for b in body.bought:
        lo, hi = np.array(b["lo"]), np.array(b["hi"])
        d = hi - lo
        I = np.diag([d[1] ** 2 + d[2] ** 2, d[0] ** 2 + d[2] ** 2, d[0] ** 2 + d[1] ** 2]) * b["mass"] / 12.0
        comps.append((b["mass"], (lo + hi) / 2, I))
    if hasattr(body, "tyre_mass"):                        # a ring at the rim
        r = WHEEL_D / 2
        comps.append((body.tyre_mass, body.origin.copy(), np.diag([0.5, 1.0, 0.5]) * body.tyre_mass * r * r))
    M = sum(c[0] for c in comps)
    com = sum(c[0] * c[1] for c in comps) / M
    I = np.zeros((3, 3))
    for m, p, Ic in comps:
        d = p - com
        I += Ic + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    I *= 1e-9                                             # g.mm2 -> kg.m2
    return M / 1000.0, com / 1000.0, [I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quick", action="store_true", help="skip the interference check (the slow part)")
    a = ap.parse_args()
    B = build()
    report = {"parts": {}, "bodies": {}}
    (HERE / "parts").mkdir(exist_ok=True)
    for body in B.values():
        for name, shape in body.prints.items():
            solids = shape.val().Solids() if hasattr(shape.val(), "Solids") else [shape.val()]
            local = shape.translate(tuple(-body.origin))            # each part in its own body's frame
            cq.exporters.export(local, str(HERE / "parts" / f"{name}.step"))
            cq.exporters.export(local, str(HERE / "parts" / f"{name}.stl"), tolerance=0.05, angularTolerance=0.2)
            report["parts"][name] = {"solids": len(solids), "volume_mm3": round(shape.val().Volume(), 1),
                                     "mass_g_printed": round(shape.val().Volume() * PLA_G_MM3 * FILL, 1), "body": body.name}
    assembly = {"units": "metres, kilograms, radians", "wheel_radius_m": WHEEL_D / 2000, "track_m": (TRACK + 2.0) / 1000,
                "pitch_radius_m": PITCH_R / 1000, "bodies": []}
    total = 0.0
    for body in B.values():
        m, com, I = mass_props(body)
        total += m
        o = body.origin
        j = dict(body.joint) if body.joint else None
        if j and "range_deg" in j:
            j["range_rad"] = [math.radians(v) for v in j.pop("range_deg")]
        assembly["bodies"].append({
            "name": body.name, "parent": body.parent, "origin_m": (o / 1000).tolist(), "joint": j,
            "mass_kg": round(m, 5), "com_m": [round(float(v), 6) for v in com - o / 1000], "inertia_kg_m2": [float(f"{v:.4g}") for v in I],
            "prints": sorted(body.prints),
            "bought": [{"name": b["name"], "lo_m": [(v - o[i]) / 1000 for i, v in enumerate(b["lo"])],
                        "hi_m": [(v - o[i]) / 1000 for i, v in enumerate(b["hi"])], "rgba": b["rgba"]} for b in body.bought],
            "collision": [{k: ([(x - o[i]) / 1000 for i, x in enumerate(v)] if k in ("lo", "hi", "centre") else
                               [x / 1000 for x in v] if k == "radii" else v / 1000 if k == "radius" else v) for k, v in c.items()}
                          for c in body.collision],
            "cameras": [{"name": cam["name"], "lens_m": [(v - o[i]) / 1000 for i, v in enumerate(cam["lens"])], "looks": cam["looks"],
                         "up": cam["up"]} for cam in body.cameras]})
        report["bodies"][body.name] = {"mass_g": round(m * 1000, 1)}
    report["total_mass_g"] = round(total * 1000, 1)
    # where the weight sits, head stowed forward and low: it must be between the wheels (x = 0) and the caster (x = 128)
    def cog(q):
        num = np.zeros(3)
        for body in B.values():
            m, com, _ = mass_props(body)
            p = placed(B, body.name, q, cq.Workplane("XY").sphere(0.5).translate(tuple(com * 1000))).val().Center()
            num += m * np.array([p.x, p.y, p.z])
        return (num / total).round(1).tolist()
    stow = dict(lift=LIMITS["lift"][0], tilt=0, spacing=0)
    report["centre_of_mass_mm"] = {"head stowed": cog(stow), "head raised": cog(dict(lift=LIMITS["lift"][1], spacing=0)),
                                   "head stowed, turned 90 deg, cameras apart": cog(dict(pan=90, lift=LIMITS["lift"][0], spacing=70)),
                                   "head stowed, turned 170 deg": cog(dict(pan=170, lift=LIMITS["lift"][0])),
                                   "support": "wheels touch at x = 0, y = +/-65; the caster at x = 128; the rear skid at x = -42 is 1.5 mm clear"}
    low_stow, lens_low, reach = lowest_and_lens(B, stow)
    low_down, _, _ = lowest_and_lens(B, dict(lift=LIMITS["lift"][0], tilt=LIMITS["tilt"][0]))
    _, lens_high, _ = lowest_and_lens(B, dict(lift=LIMITS["lift"][1]))
    report["ranges"] = {"lens_height_mm": [round(lens_low, 1), round(lens_high, 1)], "lens_spacing_mm": [2 * LIMITS["spacing"][0], 2 * LIMITS["spacing"][1]],
                        "pan_deg": LIMITS["pan"], "lift_deg": LIMITS["lift"], "tilt_deg": LIMITS["tilt"],
                        "lowest_point_of_head_stowed_mm": round(low_stow, 1), "lowest_point_stowed_and_tilted_down_mm": round(low_down, 1),
                        "lens_ahead_of_axle_stowed_mm": round(reach, 1)}
    # wires: how far apart the two ends of each lead get over the travel, against the 200 mm lead a servo ships with
    def span(body_a, pa, body_b, pb, grid):
        worst = 0.0
        for q in grid:
            A = placed(B, body_a, q, cq.Workplane("XY").sphere(0.5).translate(tuple(pa))).val().Center()
            Bp = placed(B, body_b, q, cq.Workplane("XY").sphere(0.5).translate(tuple(pb))).val().Center()
            worst = max(worst, math.dist((A.x, A.y, A.z), (Bp.x, Bp.y, Bp.z)))
        return worst
    T = B["head"].origin
    g = [dict(pan=p, lift=l, tilt=t, spacing=s) for p in (-170, 0, 170) for l in LIMITS["lift"] for t in LIMITS["tilt"] for s in (0, 70)]
    leads = {"pan servo to lift servo (across pan)": span("chassis", (PAN_X - 20, 0, 60), "turret", (PAN_X - 20, 0, 101), g),
             "lift servo to tilt servo (along the arm)": span("turret", (PAN_X - 20, 0, 101), "head_block", (T[0] - 20, 0, T[2]), g),
             "tilt servo to spacing servo (across tilt)": span("head_block", (T[0] - 20, 0, T[2]), "head", (T[0] + 64, -20, T[2] + 23), g),
             "head to a camera board (across the slide)": span("head", (T[0] + 64, 0, T[2] + 10), "carriage_left", (T[0] + 95, 30, T[2] + 5), g)}
    report["wires"] = {k: {"ends_apart_at_most_mm": round(v, 1), "needs_mm": round(v * 1.25 + 30, 0), "stock_lead_mm": 200,
                           "ok": bool(v * 1.25 + 30 <= 200)} for k, v in leads.items()}
    report["wires"]["note"] = ("needs = 25 % slack plus 30 mm for the plugs. The lead across pan must also take a turn of +/-170 deg: "
                               "leave a loop. Routing is not designed.")
    if not a.quick:
        worst, poses = interference(B)
        report["interference_mm3"] = {"poses_checked": poses, "pairs_that_overlap": worst}
    report["all_parts_one_solid"] = all(p["solids"] == 1 for p in report["parts"].values())
    (HERE / "rover.json").write_text(json.dumps(assembly, indent=1) + "\n")
    (HERE / "build_report.json").write_text(json.dumps(report, indent=1) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k != "parts"}, indent=1))
    print({n: (p["solids"], p["mass_g_printed"]) for n, p in report["parts"].items()})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
