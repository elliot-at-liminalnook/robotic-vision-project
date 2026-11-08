#!/usr/bin/env python3
"""Wrist bracket: holds the single camera holder on the follower's gripper.

Inputs:
  wrist_mount_pose.json            holder pose on the gripper (place_camera.py)
  upstream_mount_registration.json upstream wrist mount STEP -> gripper frame
  ../camera_mount/camera_holder_single.step, camera preview (for the assembly)

The bracket, in the gripper body frame (mm):
  1. Gripper-side flange: upstream's own SO-101 wrist-camera flange, cut from
     its STEP and moved into place. That's a 2 mm plate on the gripper's +y face
     (y = 24) with the two M3 screw holes into the gripper's hex-nut recesses,
     plus its locating key. Reusing it keeps the proven fit.
  2. Thickened to 6 mm (y 24..30) over the same footprint.
  3. Pad on the FRONT of the holder's top band (holder frame x +/-34,
     y 17.5..33, z 3..8). Seen from the holder, the gripper flange is in
     front of it (lens side) and past the top band, so a pad on the back would
     force the arm through the holder. Two M3 heat-set inserts at the holder's
     M3 holes (+/-28, 29); the holder bolts on from its open back with M3 x 12.
     The pad is behind the lens plane, outside the camera's view.
  4. Arm: a ruled loft from the flange to the pad's front face.
  5. Screwdriver/head channels (dia 6.4) along the two gripper screws, so the
     bracket can be screwed on after the arm is in place.
Checks: one solid; no overlap with the holder, the ESP32 board envelope or
the gripper's side of y = 24 (except the upstream locating key).
Writes wrist_bracket.step/.stl (gripper frame) and assembly STLs for sim/renders.

Run (CadQuery env):  python hardware/wrist_mount/build_wrist_mount.py
"""
from __future__ import annotations

import json
from pathlib import Path

import cadquery as cq
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
UP_STEP = ROOT / "third_party/SO-ARM100/Optional/SO101_Wrist_Cam_Hex-Nut_Mount_32x32_UVC_Module/stl/SO-ARM101_camera_wrist_mount.step"
HOLDER_STEP = ROOT / "hardware/camera_mount/camera_holder_single.step"
CAMERA_STL = ROOT / "hardware/camera_mount/preview_camera_module3.stl"

FLANGE_EXTRA = 4.0            # thicken upstream's 2 mm flange to 6 mm
PAD = dict(x=(-34.0, 34.0), y=(17.5, 33.0), z=(3.0, 8.0))     # holder frame, on the top band's front
INSERT_HOLES = [(-28.0, 29.0), (28.0, 29.0)]                      # holder frame
INSERT_D = 4.0                # M3 heat-set insert pilot hole
SCREW_ACCESS_D = 6.4          # M3 head + screwdriver


def transform(shape: cq.Shape, R: np.ndarray, t: np.ndarray) -> cq.Shape:
    """Rigid transform p -> R p + t (R re-orthonormalised against rounding)."""
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.gp import gp_Trsf
    U, _, Vt = np.linalg.svd(np.asarray(R, float))
    R = U @ Vt
    tr = gp_Trsf()
    tr.SetValues(*[float(v) for row in np.column_stack([R, t]) for v in row])
    return cq.Shape.cast(BRepBuilderAPI_Transform(shape.wrapped, tr, True).Shape())


def holder_to_gripper(pose) -> tuple[np.ndarray, np.ndarray]:
    R = np.array(pose["holder_rotation"])
    o = np.array(pose["holder_origin"]) * 1000.0
    return R, o


def hpt(R, o, p):
    return R @ np.asarray(p, float) + o


def in_view_count(shape: cq.Shape, pose) -> int:
    """Bracket vertices that project inside the wrist camera image (1080p pinhole)."""
    import tempfile
    cam = pose["camera"]
    p = np.array(cam["pos"]) * 1000
    f = np.array(cam["look_at"]) * 1000 - p
    f /= np.linalg.norm(f)
    up = np.array(cam["up"])
    x = np.cross(f, up); x /= np.linalg.norm(x)
    y = np.cross(x, f)
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / "b.stl"
        cq.exporters.export(shape, str(path), tolerance=0.05, angularTolerance=0.2)
        import struct
        data = path.read_bytes()
        if data[:5] == b"solid":
            verts = np.array([[float(v) for v in ln.split()[1:4]] for ln in data.decode().splitlines()
                              if ln.strip().startswith("vertex")])
        else:
            n = struct.unpack("<I", data[80:84])[0]
            verts = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))["v"].reshape(-1, 3)
    d = verts - p
    zc = d @ f
    fx = fy = 982.1
    u = 960 + fx * (d @ x) / np.maximum(zc, 1e-9)
    v = 540 - fy * (d @ y) / np.maximum(zc, 1e-9)
    inside = (zc > 1.0) & (u >= 0) & (u < 1920) & (v >= 0) & (v < 1080)
    return int(inside.sum())


def main() -> int:
    pose = json.loads((HERE / "wrist_mount_pose.json").read_text())
    reg = json.loads((HERE / "upstream_mount_registration.json").read_text())
    Ru = np.round(np.array(reg["R"]))              # snap: axis-aligned to within 0.04 deg
    tu = np.array(reg["t_mm"])
    Rh, oh = holder_to_gripper(pose)

    # 1. upstream flange (STEP x in [-2, 2]: the 2 mm plate + locating key)
    up = cq.importers.importStep(str(UP_STEP)).val()
    flange = up.intersect(cq.Solid.makeBox(4.05, 200, 200, cq.Vector(-2.0, -100, -50)))
    flange = transform(flange, Ru, tu)
    # 2. thicken over the same footprint: extrude its outer (+y) face
    outer = max((f for f in flange.Faces() if f.geomType() == "PLANE" and f.normalAt().y > 0.99),
                key=lambda f: f.Center().y)
    thick = cq.Solid.extrudeLinear(outer, cq.Vector(0, FLANGE_EXTRA, 0))
    flange_y_out = outer.Center().y + FLANGE_EXTRA
    ob = outer.BoundingBox()

    # 3. pad behind the holder's top band (holder frame -> gripper frame)
    pad = cq.Solid.makeBox(PAD["x"][1] - PAD["x"][0], PAD["y"][1] - PAD["y"][0], PAD["z"][1] - PAD["z"][0],
                           cq.Vector(PAD["x"][0], PAD["y"][0], PAD["z"][0]))
    pad = transform(pad, Rh, oh)

    # 4. arm: ruled loft from the flange's outer rectangle to the pad's back rectangle
    fl = [cq.Vector(x, flange_y_out, z) for x, z in
          ((ob.xmin, ob.zmin), (ob.xmax, ob.zmin), (ob.xmax, ob.zmax), (ob.xmin, ob.zmax))]
    px, py, pz = PAD["x"], PAD["y"], PAD["z"][1]
    pd = [cq.Vector(*hpt(Rh, oh, p)) for p in ((px[0], py[0], pz), (px[1], py[0], pz), (px[1], py[1], pz), (px[0], py[1], pz))]
    # Match corner order by nearest x so the loft doesn't twist.
    pd = sorted(pd, key=lambda v: (round(v.x), v.z))
    pd = [pd[0], pd[2], pd[3], pd[1]] if pd[0].z < pd[1].z else [pd[1], pd[3], pd[2], pd[0]]
    arm = cq.Solid.makeLoft([cq.Wire.makePolygon(fl, close=True), cq.Wire.makePolygon(pd, close=True)], True)

    bracket = flange.fuse(thick, pad, arm).clean()

    # Heat-set insert holes (through the pad, along the holder z axis).
    for x, y in INSERT_HOLES:
        a = hpt(Rh, oh, (x, y, PAD["z"][0] - 1.0))
        bracket = bracket.cut(cq.Solid.makeCylinder(INSERT_D / 2, PAD["z"][1] - PAD["z"][0] + 2.0,
                                                    cq.Vector(*a), cq.Vector(*Rh[:, 2])))
    # Screw holes through the thickened flange + head/driver channels outward.
    for x, z in pose["screws_xz"]:
        x, z = x * 1000, z * 1000
        bracket = bracket.cut(cq.Solid.makeCylinder(1.7, 20, cq.Vector(x, 20, z), cq.Vector(0, 1, 0)))
        bracket = bracket.cut(cq.Solid.makeCylinder(SCREW_ACCESS_D / 2, 120, cq.Vector(x, flange_y_out - FLANGE_EXTRA + 2.0, z),
                                                    cq.Vector(0, 1, 0)))

    # Assembly pieces in the gripper frame.
    holder = transform(cq.importers.importStep(str(HOLDER_STEP)).val(), Rh, oh)
    bb_board = pose["board_box_holder_frame"]
    board = transform(cq.Solid.makeBox(*(1000 * (b[1] - b[0]) for b in bb_board),
                                       cq.Vector(*(1000 * b[0] for b in bb_board))), Rh, oh)

    report = {
        "solids": len(bracket.Solids()),
        "volume_mm3": round(bracket.Volume(), 1),
        "mass_g_pla_100pct": round(bracket.Volume() * 1.24e-3, 1),
        "overlap_mm3": {
            "holder": round(bracket.intersect(holder).Volume(), 3),
            "board_envelope": round(bracket.intersect(board).Volume(), 3),
            "inside_gripper_face_y_lt_24_excluding_key": round(bracket.cut(flange).intersect(
                cq.Solid.makeBox(300, 100, 300, cq.Vector(-150, -76, -150))).Volume(), 3),
        },
        "bbox_mm": [round(v, 1) for v in (bracket.BoundingBox().xmin, bracket.BoundingBox().xmax,
                                          bracket.BoundingBox().ymin, bracket.BoundingBox().ymax,
                                          bracket.BoundingBox().zmin, bracket.BoundingBox().zmax)],
    }
    report["in_wrist_camera_view_vertices"] = in_view_count(bracket, pose)
    print(json.dumps(report, indent=2))
    assert report["solids"] == 1, "bracket must be one solid"
    assert report["in_wrist_camera_view_vertices"] == 0, "bracket is visible in the wrist camera"
    assert all(v < 0.01 for v in report["overlap_mm3"].values()), report["overlap_mm3"]

    stl = dict(tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(bracket, str(HERE / "wrist_bracket.step"))
    cq.exporters.export(bracket, str(HERE / "wrist_bracket.stl"), **stl)
    asm = HERE / "assembly"
    asm.mkdir(exist_ok=True)
    cq.exporters.export(bracket, str(asm / "bracket_gripper_frame.stl"), **stl)
    cq.exporters.export(holder, str(asm / "holder_gripper_frame.stl"), **stl)
    cq.exporters.export(board, str(asm / "board_envelope_gripper_frame.stl"), **stl)
    (HERE / "build_report.json").write_text(json.dumps(report, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
