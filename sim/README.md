# sim/: SO-101 follower simulation (Newton + MuJoCo Warp)

A simulation of our follower arm, workcell and cameras, built to run on
**[Newton](https://github.com/newton-physics/newton) with `SolverMuJoCo` on the
MuJoCo Warp backend** (GPU). The same model also runs in plain MuJoCo for quick
checks on machines Newton doesn't support.

```
sim/
├── config/cameras.toml     # camera poses + intrinsics (single source of truth)
├── config/servos.toml      # HX-30HM limits, encoder ticks <-> radians mapping
├── scene/workcell.xml      # hand-written: table, objects, lights, physics options
├── tools/build_model.py    # generates model/ from the pristine upstream MJCF
├── model/                  # GENERATED (committed): robot + workcell MJCF and meshes
├── so101sim/               # runtime: Newton and MuJoCo backends, IK, servo model
└── scripts/pick_demo.py    # scripted pick: end-to-end check incl. camera renders
```

## Where it runs

| Machine | Backend | Notes |
|---|---|---|
| Linux + NVIDIA GPU | `newton`, solver `warp` (default with CUDA) | The target: MuJoCo Warp |
| Linux, CPU only | `newton`, solver `cpu` | MuJoCo C backend through Newton; same model and cameras |
| Intel Mac | `mujoco` only | Warp ships no x86_64 macOS build, so Newton can't install |

On the Intel Mac, Newton is exercised in a Lima VM (`limactl shell newton-sim`,
Ubuntu 24.04 x86_64, CPU only), which can't run the GPU path.

```bash
# Linux
python3 -m venv ~/newton-venv && ~/newton-venv/bin/pip install -r sim/requirements.txt
~/newton-venv/bin/python sim/scripts/pick_demo.py --backend newton    # --solver warp|cpu

# Anywhere MuJoCo runs (Intel Mac: pip install --only-binary :all: mujoco -> 3.10.0)
python sim/scripts/pick_demo.py --backend mujoco
python -m mujoco.viewer --mjcf=sim/model/workcell.xml                 # interactive
```

Images and `result.json` land in `sim/out/<backend>/` (gitignored).

## What changed vs upstream (tools/build_model.py)

Upstream: `third_party/SO-ARM100/Simulation/SO101/so101_new_calib.xml`.

1. **Jaw collision.** Engines collide a mesh as its convex hull. Upstream's fixed
   jaw is one mesh with the wrist housing, so its hull fills the space between the
   jaws: **30 of 60 probe points in the gripping gap were inside it**, so objects
   collided with empty air. Both jaw meshes are replaced by CoACD convex
   decompositions (16 + 10 hulls); the generator fails the build if any hull
   intrudes into the gap (now 0/60).
2. **Finger pads.** The jaws' inner faces are stepped, not sloped: flat lands at
   x = ∓7.9 / 9.9 / 11.6–11.9 mm, measured by ray casting the meshes. Each land
   gets a thin box pad (0.5 mm proud, friction 1.0, torsional friction). The
   friction value assumes rubber/TPU grip pads on the real jaws too; bare PLA is
   ~0.3–0.4.
3. **Actuators.** Torque limit ±2.94 N·m = HX-30HM stall torque. Gains and joint
   damping/friction/armature remain upstream's STS3215 values (same 1:345 ratio,
   near-identical speed) until we fit them to real step responses.
4. **Cameras.** `wrist` on the gripper body (with a visual-only stand-in for the
   Camera Module 3 board), `scene_front` and `scene_side` fixed in the world.
5. **Workcell.** Table (top at z = 0), three PLA-sized objects, elliptic friction
   cones with `impratio = 10` (MuJoCo's recommended grasping settings),
   `implicitfast` integrator, 2 ms step.

## Things the engine doesn't model, done in `so101sim`

- **Servo top speed.** Position actuators have no velocity limit; `SlewLimiter`
  caps commanded motion at the HX-30HM's 5.51 rad/s.
- **Encoder mapping.** `ServoMap` converts real servo ticks (0–4095) to sim
  radians. Per-joint `zero_ticks`/`direction` in `servos.toml` are placeholders
  until measured.
- **Lens model (Newton only).** Cameras render from rays generated from OpenCV
  intrinsics + distortion, so calibrated lens distortion is reproduced. The
  MuJoCo backend renders an undistorted pinhole with the same vertical FOV.

## Camera placeholders, to replace with measurements

`config/cameras.toml` intrinsics are datasheet-derived (f = 2.75 mm, 2.8 µm
binned pixels at 1920×1080 → fx = fy ≈ 982 px, no distortion). The wrist pose is
upstream's 32×32 UVC mount, about 73 mm to the side of the jaws; replace it with
the lens pose from our Camera Module 3 mount CAD. Fixed poses: replace with
marker-board measurements of the real cameras. After editing, re-run
`python sim/tools/build_model.py`. CoACD results are cached in `sim/.cache/`, so
re-runs take seconds.

## Known limits

- **Reach.** Straight-down grasps are only reachable fairly low over the table.
  Higher up, the fingers tilt (up to ~10–30°) because `wrist_flex` hits its limit.
  `ArmIK` keeps the tip position exact and gets as close to vertical as the joints
  allow.
- **TCP.** Upstream's `gripperframe` site is on the fixed jaw's face, not between
  the jaws. `pick_demo.solve_grasp` targets the point half an object width in from
  the fixed pad.
- **Unmeasured.** Actuator dynamics, joint friction and backlash aren't fitted to
  HX servos yet. Object masses and frictions are estimates.
