# Vendored: TheRobotStudio/SO-ARM100 (SO-101 subset)

| | |
|---|---|
| Upstream | https://github.com/TheRobotStudio/SO-ARM100 |
| Commit | see [`UPSTREAM_COMMIT`](UPSTREAM_COMMIT) (initially `a758567c3978dfeefe282ede0500085a48fe8f78`) |
| License | Apache-2.0, see [`LICENSE`](LICENSE) |
| Retrieved | 2026-11-08 |

**Do not edit files in this directory.** It is an unmodified mirror so we can
always diff our changes against the original design. To modify a part, copy it
into `hardware/` (CAD/print files) or `sim/` (URDF/MJCF) and edit the copy.

Re-sync with `scripts/sync_so_arm100.sh [ref]`.

## What's included

| Path | Contents |
|---|---|
| `STEP/SO101/` | Editable CAD (STEP) for every SO-101 part, plus the full assembly |
| `STL/SO101/Individual/` | Print-ready STL per part |
| `STL/SO101/Leader/`, `STL/SO101/Follower/` | Whole-arm print plates for Prusa / Bambu A1 mini / Ender bed sizes |
| `STL/Gauges/` | Printer accuracy test pieces (servo-fit and Lego-fit) |
| `Simulation/SO101/` | URDF + MuJoCo MJCF models (`so101_new_calib.*`, with/without wrist camera, leader arm URDF), `scene.xml`, meshes |
| `Optional/` | Add-ons: wrist/overhead camera mounts (UVC, RealSense D405/D435, webcam), compliant gripper, raised base, 4040 mount, etc. |
| `README.md` | Upstream BOM, print settings, kit vendors |
| `3DPRINT.md` | Print services |
| `Software/` | Web UI calibration notes |

## What's excluded

SO-100 (the deprecated predecessor) STEP/STL/simulation files and `SO100.md`.
`Mini/` (a separate smaller arm). Fetch them from upstream if ever needed.
