#!/usr/bin/env python3
"""Register upstream's SO-101 wrist-camera mount STEP into the gripper body frame.

Upstream ships the mount as a STEP (its own frame) and, separately, places the
same part on the gripper in its sim model (so101_new_calib_camera.xml). Fitting
one to the other gives the STEP -> gripper transform, so the bracket can reuse
upstream's exact gripper-side flange (screw holes + locating key).

Two steps, because the CAD and MuJoCo environments differ:
  1. (CadQuery env)  python register_upstream_mount.py export
        writes upstream_mount_stepframe.stl next to this file
  2. (MuJoCo env)    python register_upstream_mount.py fit
        writes upstream_mount_registration.json (R, t in mm; gripper = R @ step + t)

Result on 2026-11-08: RMS fit error 0.061 mm.
"""
from __future__ import annotations

import itertools
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
UP = ROOT / "third_party/SO-ARM100"
STEP = UP / "Optional/SO101_Wrist_Cam_Hex-Nut_Mount_32x32_UVC_Module/stl/SO-ARM101_camera_wrist_mount.step"
STL = HERE / "upstream_mount_stepframe.stl"
OUT = HERE / "upstream_mount_registration.json"


def export():
    import cadquery as cq
    cq.exporters.export(cq.importers.importStep(str(STEP)).val(), str(STL), tolerance=0.05, angularTolerance=0.2)


def fit():
    import mujoco
    import numpy as np
    import trimesh
    from scipy.spatial import cKDTree

    m = mujoco.MjModel.from_xml_path(str(UP / "Simulation/SO101/so101_new_calib_camera.xml"))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    gb = m.body("gripper").id
    Rg, pg = d.xmat[gb].reshape(3, 3), d.xpos[gb]
    g = next(g for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and
             mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, m.geom_dataid[g]) == "wrist_camera_mount_so101_v1")
    mid = m.geom_dataid[g]
    v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
    G = (Rg.T @ ((d.geom_xmat[g].reshape(3, 3) @ v.T).T + d.geom_xpos[g] - pg).T).T * 1000
    P = trimesh.load(STL).vertices
    Ps = P[np.random.default_rng(0).choice(len(P), min(4000, len(P)), replace=False)]
    tree = cKDTree(G)
    best = None
    for perm in itertools.permutations(range(3)):           # 24 axis-aligned starts + ICP
        for sg in itertools.product([1, -1], repeat=3):
            R = np.zeros((3, 3))
            for i, (j, s) in enumerate(zip(perm, sg)):
                R[i, j] = s
            if np.linalg.det(R) < 0:
                continue
            t = G.mean(0) - R @ P.mean(0)
            for _ in range(30):
                B = G[tree.query((R @ Ps.T).T + t)[1]]
                ca, cb = Ps.mean(0), B.mean(0)
                U, _, Vt = np.linalg.svd((Ps - ca).T @ (B - cb))
                if np.linalg.det(Vt.T @ U.T) < 0:
                    Vt[-1] *= -1
                R = Vt.T @ U.T
                t = cb - R @ ca
            err = float(np.sqrt(np.mean(tree.query((R @ Ps.T).T + t)[0] ** 2)))
            if best is None or err < best[0]:
                best = (err, R, t)
    err, R, t = best
    OUT.write_text(json.dumps({"R": np.round(R, 6).tolist(), "t_mm": np.round(t, 4).tolist(),
                               "rms_mm": round(err, 4), "convention": "gripper = R @ step + t (mm)"}, indent=2) + "\n")
    print(f"RMS {err:.3f} mm -> {OUT.name}")


if __name__ == "__main__":
    {"export": export, "fit": fit}[sys.argv[1]]()
