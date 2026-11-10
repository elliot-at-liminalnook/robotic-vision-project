# Robotic Vision Project

Modifying the open-source **SO-101** robot arm to use components we already have, and running it in
simulation (Newton / MuJoCo, not LeRobot).

![The simulated SO-101 follower with the wrist camera mounted](docs/media/arm_hero.png)

## Pictures

All from the simulation (MuJoCo render of `sim/model/workcell.xml`). Regenerate them with
`python sim/tools/render_media.py`.

| | |
|---|---|
| ![Arm orbit](docs/media/arm_orbit.gif) | ![Each joint in turn](docs/media/arm_joints.gif) |
| The follower, reaching while the camera circles it. | Each of the six servos in turn: pan, shoulder, elbow, wrist flex, wrist roll, gripper. |

**Scripted pick**, with the free camera on the left and the simulated wrist camera on the right. It uses the realistic
(soft) servo model with gravity/back-EMF feed-forward and grip-by-offset (`sim/scripts/pick_demo.py`):

![Pick demo: free view and wrist camera](docs/media/pick_demo.gif)

| | |
|---|---|
| ![Wrist close-up](docs/media/wrist_closeup.png) | ![Camera holder](hardware/camera_mount/render_single_holder.png) |
| Gripper and wrist camera. The blue part is the wrist bracket, the orange part the camera holder, and the green part the ESP32-P4 board. | The single Camera Module 3 holder (`hardware/camera_mount/`). |

What the three simulated cameras see (two fixed scene cameras and the wrist camera):

![Scene front, scene side and wrist camera views](docs/media/camera_views.png)

The keyboard/mouse teleop window (`sim/scripts/teleop.py`):

![Teleop](sim/docs/teleop.png)

## Layout

| Path | What | Edit? |
|---|---|---|
| `third_party/SO-ARM100/` | Upstream SO-101 design from [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) (Apache-2.0): STEP CAD, STL print files, URDF/MuJoCo models, optional mounts, BOM. Pinned commit in `UPSTREAM_COMMIT`. | No, mirror only |
| `third_party/hiwonder-so-arm101/` | Hiwonder's variant (HX-10HM/HX-30HM servos): manuals, servo protocol, print files, Rhino CAD, LeRobot patch | No, reference only |
| `docs/PLAN.md` | Build decisions, camera pipeline, open items and open decisions | Yes |
| `docs/BOM.md` | Parts ordered and still to buy, with links, prices and the power wiring | Yes |
| `docs/CONCEPTS.md` | Self-contained lessons on the ideas behind the arm (motors, control, IK, contact, camera) | Yes |
| `docs/datasheets/` | Datasheets + official CAD: servos, servo driver board, Camera Module 3, ESP32-P4 board/chip | No, reference only |
| `firmware/p4-uvc-camera/` | ESP32-P4 + Camera Module 3 → USB UVC webcam firmware (ESP-IDF) | Yes |
| `firmware/components/` | Vendored IMX708 camera driver (Apache-2.0) used by the firmware | Carefully |
| `hardware/camera_mount/` | Single Camera Module 3 holder (from the stereo V9 beam, reinforced): STEP/STL + CadQuery build | Yes |
| `hardware/wrist_mount/` | Wrist bracket + camera placement study (gripper +y side, lens between the jaw tips) | Yes |
| `sim/` | Simulation (Newton/MuJoCo Warp + MuJoCo): follower with the mounted wrist camera, workcell, 3 cameras, keyboard/mouse **teleop** (fast: torque governor; optional load-aware IK), pick demo, vision check, servo load + fast-teleop studies. See `sim/README.md` | Yes (not `sim/model/`, which is generated) |
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

## Quick start: drive the simulated arm

```bash
python sim/scripts/teleop.py                    # keyboard + mouse, all three cameras (MuJoCo backend)
python sim/scripts/teleop.py --backend newton   # Linux: Newton / MuJoCo Warp
```

Details, verified results and backend notes: `sim/README.md`. Decisions so far: `docs/DECISIONS.md`.

Run the tests with `pytest sim/tests` (about 3 minutes).

## View the upstream arm

Our simulation lives in `sim/` (see `sim/README.md`). The upstream model can still be viewed directly:

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

## Licensing

- `third_party/SO-ARM100/`: Apache-2.0 (see its `LICENSE`).
- `third_party/hiwonder-so-arm101/` and `docs/datasheets/`: manufacturer documents with no
  stated license, included for reference; this repo is public, so remove them if a vendor objects.
