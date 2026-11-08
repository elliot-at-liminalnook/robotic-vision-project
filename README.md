# Robotic Vision Project

Modifying the open-source **SO-101** leader/follower robot arm (LeRobot ecosystem)
to use components we already have, and running the arms in simulation.

## Layout

| Path | What | Edit? |
|---|---|---|
| `third_party/SO-ARM100/` | Upstream SO-101 design from [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) (Apache-2.0): STEP CAD, STL print files, URDF/MuJoCo models, optional mounts, BOM. Pinned commit in `UPSTREAM_COMMIT`. | No, mirror only |
| `third_party/hiwonder-so-arm101/` | Hiwonder's variant (HX-10HM/HX-30HM servos): manuals, servo protocol, print files, Rhino CAD, LeRobot patch | No, reference only |
| `docs/PLAN.md` | Build decisions, camera pipeline, LeRobot integration options, open items | Yes |
| `docs/datasheets/` | Datasheets + official CAD: servos, servo driver board, Camera Module 3, ESP32-P4 board/chip | No, reference only |
| `firmware/p4-uvc-camera/` | ESP32-P4 + Camera Module 3 → USB UVC webcam firmware (ESP-IDF) | Yes |
| `firmware/components/` | Vendored IMX708 camera driver (Apache-2.0) used by the firmware | Carefully |
| `hardware/` | *(to come)* Our modified CAD / print files | Yes |
| `sim/` | *(to come)* Our modified URDF / MJCF models and sim scripts | Yes |
| `scripts/` | `sync_so_arm100.sh`: re-sync the upstream mirror | |

Keep `third_party/` and `docs/datasheets/` pristine so we can always diff our
changes against the originals. Copy a file into `hardware/` or `sim/` before changing it.

## Large files

STL / STEP / 3MF / 3DM / PDF files are stored with **Git LFS**. Install it before cloning:

```bash
brew install git-lfs   # or your package manager
git lfs install
git clone https://github.com/elliot-at-liminalnook/robotic-vision-project.git
```

Without LFS you get small pointer files instead of meshes, and simulation won't load.

## Quick start: view the arm in simulation

The upstream MuJoCo scene works as-is (verified with MuJoCo 3.10.0, Python 3.13):

```bash
pip install --only-binary :all: mujoco   # Intel Macs: MuJoCo 4.x has no x86_64 wheel; this resolves to 3.10.0
python -m mujoco.viewer --mjcf=third_party/SO-ARM100/Simulation/SO101/scene.xml
```

The model has 6 actuated joints: `shoulder_pan`, `shoulder_lift`, `elbow_flex`,
`wrist_flex`, `wrist_roll`, `gripper`.

Known gaps for this project:
- `so101_new_calib_camera.xml` adds the wrist-camera *mesh* but defines **no MuJoCo camera**,
  so there's no simulated camera image yet.
- The sim uses STS3215 geometry and motor parameters. Hiwonder's HX-servo variant has no sim model.
- LeRobot's gripper is 0–100 (closed→open); the URDF/MJCF use radians (noted upstream).

## Licensing

- `third_party/SO-ARM100/`: Apache-2.0 (see its `LICENSE`).
- `third_party/hiwonder-so-arm101/` and `docs/datasheets/`: manufacturer documents with no
  stated license, included for reference; this repo is public, so remove them if a vendor objects.
