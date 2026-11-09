#!/usr/bin/env python3
"""Check that every camera sees what it should, in each backend.

For three arm poses (home, above the cube, at the grasp) this renders all
cameras and finds the red cube in each image (colour mask). It compares the
mask's centre with where the cube's centre SHOULD project, computed
independently from the camera pose (forward kinematics of the same joint
angles) and the intrinsics in config/cameras.toml. A camera that is mis-posed,
mirrored, flipped, or has the wrong focal length shows up as pixel error.

    python sim/scripts/vision_check.py --backend mujoco
    python sim/scripts/vision_check.py --backend newton [--solver cpu|warp]
    python sim/scripts/vision_check.py --compare      # side-by-side of saved runs

Writes sim/out/vision/<backend>/ (images + result.json).
Pass: every visible cube within MAX_ERR_PX of its predicted position.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from so101sim import JOINTS, load_backend  # noqa: E402
from so101sim.config import MODEL_DIR, look_at_axes  # noqa: E402
from so101sim.ik import ArmIK  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "out" / "vision"
MAX_ERR_PX = 12.0      # at 640x360; the cube is ~25-60 px across in these views
CUBE_HALF = 0.015


def red_mask(img: np.ndarray) -> np.ndarray:
    r, g, b = (img[..., i].astype(int) for i in range(3))
    return (r > 90) & (r > 2 * g) & (r > 2 * b)


def predicted_pixels(spec, q, cube_pos):
    """Project the cube's corners + centre with an independent pinhole model."""
    m = mujoco.MjModel.from_xml_path(str(MODEL_DIR / "workcell.xml"))
    d = mujoco.MjData(m)
    for name, v in zip(JOINTS, q):
        d.qpos[m.joint(name).qposadr[0]] = v
    mujoco.mj_kinematics(m, d)
    Rc = np.column_stack(look_at_axes(spec.pos, spec.look_at, spec.up))   # camera axes in parent frame
    p = np.asarray(spec.pos, float)
    if spec.parent_body != "world":
        b = m.body(spec.parent_body).id
        Rb = d.xmat[b].reshape(3, 3)
        Rc, p = Rb @ Rc, d.xpos[b] + Rb @ p
    i = spec.intrinsics
    s = spec.render_width / i["calib_width"]
    corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * CUBE_HALF + cube_pos
    pts = np.vstack([cube_pos, corners])
    pc = (Rc.T @ (pts - p).T).T                       # camera frame: looks along -z
    depth = -pc[:, 2]
    u = i["cx"] * s + i["fx"] * s * pc[:, 0] / depth
    v = i["cy"] * s - i["fy"] * s * pc[:, 1] / depth
    return np.column_stack([u, v]), depth


def run(backend: str, solver: str | None):
    kw = {"solver": solver} if backend == "newton" else {}
    sim = load_backend(backend, **kw)
    label = sim.name + (f"-{sim.solver_kind}" if backend == "newton" else "")
    out = OUT / label
    out.mkdir(parents=True, exist_ok=True)
    import pick_demo as pd
    ik = ArmIK()
    q_open = pd.gripper_open_angle(ik, pd.CUBE_SIZE + 0.014)
    home, _ = ik.solve([0.20, 0.0, 0.16], [0, 0, 0, 0, 0, q_open])
    sim.reset(home)
    sim.step(int(0.3 / sim.dt))
    cube = sim.body_pos("cube")
    above, _ = ik.solve(cube + [0, 0, 0.08], home)
    above = pd.square_roll(ik, above)
    above, _ = pd.solve_grasp(ik, cube, pd.CUBE_SIZE, above, dz=0.08)
    grasp, _ = pd.solve_grasp(ik, cube, pd.CUBE_SIZE, above, dz=-0.004)
    poses = {"home": home, "above": above, "grasp": grasp}

    from PIL import Image
    rows, ok = [], True
    for pname, q in poses.items():
        sim.reset(q)              # place the arm exactly (kinematic check, no motion)
        imgs = sim.render()
        q_now = sim.q()
        cube = sim.body_pos("cube")
        for cname, img in imgs.items():
            Image.fromarray(img).save(out / f"{pname}_{cname}.png")
            spec = sim.cameras[cname]
            uv, depth = predicted_pixels(spec, q_now, cube)
            h, w = img.shape[:2]
            pred_visible = depth[0] > 0 and 0 <= uv[0, 0] < w and 0 <= uv[0, 1] < h
            mask = red_mask(img)
            row = {"pose": pname, "camera": cname, "predicted_px": np.round(uv[0], 1).tolist(),
                   "predicted_visible": bool(pred_visible), "mask_pixels": int(mask.sum())}
            if mask.sum() > 20:
                ys, xs = np.nonzero(mask)
                # Compare against the projected corners' box centre (the visible
                # silhouette's centre), which a perspective view shifts slightly
                # from the projected body centre.
                box_c = (uv[1:].min(0) + uv[1:].max(0)) / 2
                seen = np.array([(xs.min() + xs.max()) / 2, (ys.min() + ys.max()) / 2])
                row["seen_px"] = np.round(seen, 1).tolist()
                row["error_px"] = round(float(np.linalg.norm(seen - box_c)), 1)
                row["pass"] = row["error_px"] <= MAX_ERR_PX
            else:
                # Not seen: fine only if the prediction says it's out of view (or occluded by the jaws at the grasp).
                row["pass"] = (not pred_visible) or (cname == "wrist" and pname == "grasp")
                row["note"] = "cube not visible in image"
            ok &= row["pass"]
            rows.append(row)
    result = {"backend": label, "pass": bool(ok), "max_err_px": MAX_ERR_PX, "checks": rows}
    (out / "result.json").write_text(json.dumps(result, indent=2))
    for r in rows:
        print(f"{r['pose']:6s} {r['camera']:12s} pred {r['predicted_px']}  seen {r.get('seen_px', '-')}  "
              f"err {r.get('error_px', '-')}  {'PASS' if r['pass'] else 'FAIL'}")
    print("PASS" if ok else "FAIL", label)
    return 0 if ok else 1


def compare():
    """Side-by-side sheet of all saved backend runs: rows = pose x camera."""
    from PIL import Image, ImageDraw
    runs = sorted(p for p in OUT.iterdir() if (p / "result.json").exists()) if OUT.exists() else []
    if not runs:
        print(f"no saved runs in {OUT}; run vision_check.py --backend ... first")
        return 1
    names = [(r["pose"], r["camera"]) for r in json.loads((runs[0] / "result.json").read_text())["checks"]]
    w, h = 640, 360
    sheet = Image.new("RGB", (w * len(runs), (h + 24) * len(names) + 30), (255, 255, 255))
    d = ImageDraw.Draw(sheet)
    for c, run_dir in enumerate(runs):
        d.text((c * w + 8, 8), run_dir.name, fill=(0, 0, 0))
        res = {(r["pose"], r["camera"]): r for r in json.loads((run_dir / "result.json").read_text())["checks"]}
        for r, key in enumerate(names):
            img = Image.open(run_dir / f"{key[0]}_{key[1]}.png").resize((w, h))
            sheet.paste(img, (c * w, 30 + r * (h + 24)))
            rr = res.get(key, {})
            d.text((c * w + 8, 30 + r * (h + 24) + h + 4),
                   f"{key[0]} / {key[1]}: err {rr.get('error_px', '-')} px  {'PASS' if rr.get('pass') else 'FAIL'}",
                   fill=(0, 100, 0) if rr.get("pass") else (180, 0, 0))
    sheet.save(OUT / "compare.png")
    print(OUT / "compare.png")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", choices=("newton", "mujoco"))
    ap.add_argument("--solver", choices=("warp", "cpu"), default=None)
    ap.add_argument("--compare", action="store_true")
    a = ap.parse_args()
    if a.compare:
        return compare()
    if a.backend is None:
        ap.error("--backend is required (or use --compare)")
    return run(a.backend, a.solver)


if __name__ == "__main__":
    sys.exit(main())
