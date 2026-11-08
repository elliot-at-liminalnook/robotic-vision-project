#!/usr/bin/env python3
"""Single Camera Module 3 holder, cut from the V9 stereo camera beam and reinforced.

Source: source/stereo_v9_camera_beam.step (copied unmodified from
~/Projects/esp32-stereo-camera/outputs/stereo-prototype-v9/camera_beam.step).
That beam holds two cameras 120 mm apart; this keeps ONE cradle (the left,
lens centre at x = -60 in beam coordinates) and moves it so the lens centre
line is x = 0.

Kept unchanged: the snap mechanism. That's the two spring arms with hook caps
over the camera's mounting holes, the side release paddles, two diagonal
locating pins, four support pads and the two lower-corner shelves.

Added support (all outside the parts that move):
  1. Outer side rails +2 mm and bottom rail +2 mm, added on the OUTSIDE only,
     so the camera opening and the spring channels keep their size.
  2. Solid top band from the upper rail to the old crossbar. It replaces the
     stereo carrier's snap latch, which a single holder doesn't need. Two
     symmetric M3-clearance mounting holes go through it.
  3. 2 mm back flange around the whole perimeter for twist stiffness, leaving
     the inner opening (camera back and ribbon path) clear.
  4. Gussets in the two bottom inner corners, beside the spring arms' fixed
     roots (not the flexing length).
  5. Board-edge collar around the camera PCB itself:
     - side and top guide walls 0.3 mm outside the PCB outline, up to z = 7.5
       (0.45 mm below the keeper hooks), with a lead-in chamfer so the board
       still drops straight in from the front;
     - thin back ledges under the left, right and top PCB edges, flush with the
       four pads (z = 7), kept clear of the connector and back-side parts;
     - the walls run from the bottom corner shelves up into the top band, so
       the collar is tied into the frame. The bottom edge stays open for the
       FFC connector and ribbon (the existing corner shelves carry it).
Checks: each spring's moving pieces, swept over the full 4 mm release, touch no
added material; nothing touches the Camera Module 3 reference model.

Frame (mm): x across, y up (ribbon connector at the bottom, landscape camera),
z out of the lens side. The camera board sits on pads at z = 7, with the lens
pointing +z. The holder's back face is z = -6; print on it.

Needs CadQuery 2.x. Run:  python hardware/camera_mount/build_single_holder.py
Writes camera_holder_single.step / .stl and added_material.stl next to this file.
"""
from __future__ import annotations

import json
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent
SRC = HERE / "source/stereo_v9_camera_beam.step"
# Camera Module 3 placed in the V9 beam (from the stereo project's assembly):
# PCB x +/-12.5, y -9.46..14.4, back face z = 7.0; lens axis at x = y = 0 (local).
CAMERA_REF = HERE / "source/stereo_v9_camera_left_display.brep"
CX = -60.0                       # lens centre of the kept cradle, beam coords

# V9 cradle geometry (from stereo-prototype-v9/build_prototype.py), local coords.
RAIL_OUTER_X = 32.0              # side rails span |x| 27..32
RAIL_INNER_X = 27.0
BOTTOM_Y = -50.5                 # bottom rail spans y -50.5..-45.5
BOTTOM_RAIL_TOP_Y = -45.5
UPPER_RAIL_Y = (17.5, 22.5)
TOP_Y = 33.0                     # old crossbar spans y 25..33, z -4..3
FRAME_Z = (-4.0, 2.0)            # rails are 6 mm deep
SPRING_X = 18.75                 # spring arm centre |x|; arm 3 mm wide
SPRING_RELEASE = 4.0             # outward release travel at the paddle (design value)

# Reinforcement sizes.
RIM = 2.0                        # extra width on the outer side and bottom rails
FLANGE_DEPTH = 2.0               # back flange, z -6..-4
GUSSET = 4.5                     # bottom corner gusset leg length
MOUNT_HOLE_R = 1.7               # 3.4 mm: M3 clearance
MOUNT_HOLES_X = 28.0             # at the old beam's mounting boss x
MOUNT_HOLE_Y = 29.0

# Board-edge collar (camera PCB, local coords).
PCB_X = 12.5                     # PCB half width
PCB_Y = (-9.46, 14.4)            # PCB bottom / top edge
PCB_BACK_Z = 7.0                 # PCB back face = pad tops
EDGE_GAP = 0.3                   # clearance PCB edge -> wall
WALL_T = 1.6                     # wall thickness
WALL_TOP_Z = 7.5                 # keeper hook underside is 7.95
LEAD_IN = 0.4                    # chamfer on the wall's inner top edge
LEDGE_SIDE = 0.7                 # back ledge width under the left/right edges (connector ends at |x| 11.46)
LEDGE_TOP = 0.8                  # back ledge width under the top edge (nearest part at y 13.1)
LEDGE_Y0 = -2.2                  # side ledges start above the connector (it ends at y -2.55)
LEDGE_DEPTH = 1.5                # ledge thickness below z = 7
SHELF_Y = (-12.15, -9.65)        # V9 bottom corner shelves, |x| 8.5..12.5, z -4..8


def box(x0, x1, y0, y1, z0, z1) -> cq.Solid:
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))


def gusset(sx: float) -> cq.Solid:
    """Triangular gusset in a bottom inner corner (sx = -1 left, +1 right)."""
    x, y = sx * RAIL_INNER_X, BOTTOM_RAIL_TOP_Y
    pts = [(x, y), (x - sx * GUSSET, y), (x, y + GUSSET)]
    return (cq.Workplane("XY").workplane(offset=FRAME_Z[0]).polyline(pts).close()
            .extrude(FRAME_Z[1] - FRAME_Z[0]).val())


def added_material() -> cq.Shape:
    xo = RAIL_OUTER_X + RIM
    yb = BOTTOM_Y - RIM
    parts = [
        box(-xo, -RAIL_OUTER_X, yb, TOP_Y, *FRAME_Z),             # 1. side rails, outside
        box(RAIL_OUTER_X, xo, yb, TOP_Y, *FRAME_Z),
        box(-xo, xo, yb, BOTTOM_Y, *FRAME_Z),                     #    bottom rail, below
        box(-xo, xo, UPPER_RAIL_Y[0], TOP_Y, FRAME_Z[0], 3.0),    # 2. solid top band
    ]
    # 3. back flange ring: outer outline minus the frame's inner opening
    ring = box(-xo, xo, yb, TOP_Y, FRAME_Z[0] - FLANGE_DEPTH, FRAME_Z[0]).cut(
        box(-RAIL_INNER_X, RAIL_INNER_X, BOTTOM_RAIL_TOP_Y, UPPER_RAIL_Y[0],
            FRAME_Z[0] - FLANGE_DEPTH - 1, FRAME_Z[0] + 1))
    parts.append(ring)
    parts += [gusset(-1), gusset(+1)]                             # 4. corner gussets
    out = parts[0].fuse(*parts[1:]).clean()
    return out


def board_edge_collar() -> cq.Shape:
    """Walls + back ledges around the camera PCB edges (see module docstring)."""
    xi, xo = PCB_X + EDGE_GAP, PCB_X + EDGE_GAP + WALL_T          # wall inner/outer |x|
    yt_i, yt_o = PCB_Y[1] + EDGE_GAP, UPPER_RAIL_Y[0]              # top wall inner y .. upper rail
    z0 = FRAME_Z[0]
    parts = []
    for sx in (-1, 1):
        # Side wall from the bottom shelf to the upper rail, full depth, with a
        # lead-in chamfer on the inner top edge.
        lo, hi = sorted((sx * xi, sx * xo))
        wall = cq.Workplane("XY").box(hi - lo, yt_o - SHELF_Y[0], WALL_TOP_Z - z0, centered=False) \
            .translate((lo, SHELF_Y[0], z0))
        wall = wall.edges("|Y").edges(">Z").edges("<X" if sx > 0 else ">X").chamfer(LEAD_IN)
        parts.append(wall.val())
        # Bridge the 0.3 mm gap to the bottom corner shelf (shelf outer face |x| = 12.5).
        lo, hi = sorted((sx * PCB_X, sx * xi))
        parts.append(box(lo, hi, SHELF_Y[0], SHELF_Y[1], z0, 8.0))
        # Back ledge under the side edge, above the connector, up to the top edge.
        lo, hi = sorted((sx * (PCB_X - LEDGE_SIDE), sx * xi))
        parts.append(box(lo, hi, LEDGE_Y0, PCB_Y[1], PCB_BACK_Z - LEDGE_DEPTH, PCB_BACK_Z))
    # Top wall between the side walls, up to the upper rail, chamfered on its lower inner edge.
    top = cq.Workplane("XY").box(2 * xo, yt_o - yt_i, WALL_TOP_Z - z0, centered=False) \
        .translate((-xo, yt_i, z0))
    top = top.edges("|X").edges(">Z").edges("<Y").chamfer(LEAD_IN)
    parts.append(top.val())
    # Back ledge under the top edge.
    parts.append(box(-xi, xi, PCB_Y[1] - LEDGE_TOP, yt_i, PCB_BACK_Z - LEDGE_DEPTH, PCB_BACK_Z))
    return parts[0].fuse(*parts[1:]).clean()


def spring_sweep(sx: float) -> cq.Shape:
    """Envelope of one spring's moving pieces, swept over its 4 mm outward release.

    From the V9 generator: arm (|x| 17.25..20.25, y -47..12, frame depth),
    upright (|x| 17.25..20.25, y -1.8..14.1, z 2..9.95), two keeper bars
    (|x| 10.3..19.1, y +/-1.5 around -0.1 and 12.4, z 7.95..9.95), two hook caps
    (r 1.7 at |x| 10.5, same y, z 7.95..9.95) and the release paddle
    (|x| 17.5..24.5, y -3.35..15.65, z 5..7). The arm's root (y < -40) barely
    moves and is excluded. Each piece is a box extended outward by the travel.
    """
    t = SPRING_RELEASE
    pieces = [
        (17.25, 20.25 + t, -40.0, 12.0, FRAME_Z[0], FRAME_Z[1]),   # arm
        (17.25, 20.25 + t, -1.8, 14.1, FRAME_Z[1], 9.95),          # upright
        (17.5, 24.5 + t, -3.35, 15.65, 5.0, 7.0),                  # release paddle
    ]
    for y in (-0.1, 12.4):
        pieces.append((10.3, 19.1 + t, y - 1.5, y + 1.5, 7.95, 9.95))   # keeper bar
        pieces.append((8.8, 12.2 + t, y - 1.7, y + 1.7, 7.95, 9.95))    # hook cap
    solids = []
    for x0, x1, y0, y1, z0, z1 in pieces:
        lo, hi = sorted((sx * x0, sx * x1))
        solids.append(box(lo, hi, y0, y1, z0, z1))
    return solids[0].fuse(*solids[1:])


def main() -> int:
    beam = cq.importers.importStep(str(SRC)).val()
    cradle = beam.intersect(box(-92.5, -28.0, -60, 40, -10, 20)).translate(cq.Vector(-CX, 0, 0))
    outer = added_material()
    collar = board_edge_collar()
    add = outer.fuse(collar).clean()
    holder = cradle.fuse(add).clean()
    camera = cq.importers.importBrep(str(CAMERA_REF)).val().translate(cq.Vector(-CX, 0, 0))
    for sx in (-1, 1):   # mounting holes through the top band and flange
        holder = holder.cut(cq.Solid.makeCylinder(MOUNT_HOLE_R, 20, cq.Vector(sx * MOUNT_HOLES_X, MOUNT_HOLE_Y, -10)))

    # Clearance: added material vs. the springs' swept release envelope.
    clash = {f"{'left' if sx < 0 else 'right'}_spring": round(add.intersect(spring_sweep(sx)).Volume(), 4)
             for sx in (-1, 1)}
    cam_clash = round(holder.intersect(camera).Volume(), 4)
    n_solids = len(holder.Solids())
    bb = holder.BoundingBox()
    report = {
        "source": str(SRC.relative_to(HERE)),
        "solids": n_solids,
        "bbox_mm": [round(bb.xlen, 2), round(bb.ylen, 2), round(bb.zlen, 2)],
        "volume_mm3": {"cradle_as_cut": round(cradle.Volume(), 1),
                       "added_outer": round(outer.cut(cradle).Volume(), 1),
                       "added_board_edge_collar": round(collar.cut(cradle).cut(outer).Volume(), 1),
                       "holder": round(holder.Volume(), 1)},
        "mass_g_pla_100pct": round(holder.Volume() * 1.24e-3, 1),
        "spring_clearance_overlap_mm3": clash,
        "camera_model_overlap_mm3": cam_clash,
    }
    assert n_solids == 1, "holder must be one solid"
    assert all(v == 0 for v in clash.values()), f"added material intrudes on a spring's travel: {clash}"
    assert cam_clash == 0, f"holder intersects the camera model: {cam_clash} mm3"

    cq.exporters.export(holder, str(HERE / "camera_holder_single.step"))
    cq.exporters.export(holder, str(HERE / "camera_holder_single.stl"), tolerance=0.02, angularTolerance=0.1)
    # Two-tone preview pieces taken from the FINAL part (so holes etc. are shown).
    stl = dict(tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(holder.intersect(cradle), str(HERE / "preview_original.stl"), **stl)
    cq.exporters.export(holder.cut(cradle).cut(collar), str(HERE / "preview_added_outer.stl"), **stl)
    cq.exporters.export(holder.intersect(collar).cut(cradle), str(HERE / "preview_added_collar.stl"), **stl)
    # Display only; coarse so it stays under MuJoCo's 200k-triangle STL limit.
    cq.exporters.export(camera, str(HERE / "preview_camera_module3.stl"), tolerance=0.1, angularTolerance=0.5)
    (HERE / "build_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
