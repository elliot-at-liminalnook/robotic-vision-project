# Decisions log

Choices made along the way, with the reason and what would change them.
Newest at the bottom. Dates are 2026.

## Hardware direction (2026-11-08, with the user)

| # | Decision | Why | Revisit if |
|---|---|---|---|
| H1 | Fully custom arm, self-printed; upstream SO-101 + Hiwonder files are starting points | User's goal | — |
| H2 | Hiwonder HX-10HM (leader) / HX-30HM (follower) servos, 12 V | User's choice; Feetech-protocol compatible | — |
| H3 | Cameras: 1 wrist + 1 fixed scene camera (+ optional 2nd fixed) | Wrist view gives grasp precision, scene view gives context | — |
| H4 | Camera transport: ESP32-P4 as USB UVC webcam | Standard webcam on Linux, no Wi-Fi, no custom drivers | UVC firmware fails on hardware |
| H5 | No LeRobot; simulate with Newton + SolverMuJoCo (MuJoCo Warp) | User's choice | — |

## Simulation (2026-11-08)

| # | Decision | Why |
|---|---|---|
| S1 | Generate `sim/model/` from the pristine upstream MJCF with a script | Every change reproducible and reviewable; upstream stays untouched |
| S2 | Replace jaw collision meshes with CoACD decompositions (16 + 10 hulls) | Upstream's convex hull filled the gap between the jaws (30/60 probe points) |
| S3 | One flat finger pad per jaw "land" (3 per jaw), friction 1.0 | Jaw faces are stepped; friction assumes grip pads on the real jaws too |
| S4 | Torque limit ±2.94 N·m (HX-30HM stall); keep upstream STS3215 gains | Same gear ratio and speed; refit after system ID on real servos |
| S5 | Servo speed limit in software (`SlewLimiter`, 5.51 rad/s) | Engines' position actuators have no velocity limit |
| S6 | Task-priority IK (position exact, "fingers down" best-effort) | Straight-down is unreachable high above the table (wrist_flex limit) |
| S7 | Grasp target = half the object's width in from the fixed jaw pad | Upstream TCP site is on the fixed jaw face, not between the jaws |
| S8 | Two backends behind one API: Newton (Linux) and MuJoCo (anywhere) | Newton can't install on the Intel Mac (no x86_64 macOS Warp build) |
| S9 | Make collision-only shapes visible in Newton's renderer; refit BVH per frame | Newton hides colliders when visuals exist; renderer BVH is built once |
| S10 | MuJoCo Warp contact buffers: nconmax 2048, njmax 8192 | Default 256 overflowed and dropped grasp contacts |
| S11 | Lima VM `newton-sim` (Ubuntu x86_64, CPU) for testing Newton on the Mac | GPU path can only be verified on the Linux GPU PC |

## Camera holder (2026-11-08)

| # | Decision | Why |
|---|---|---|
| C1 | Start from the stereo project's V9 `camera_beam` (left cradle) | User likes that design; its snap mechanism needs no tools |
| C2 | Edit the final V9 STEP in CadQuery, not the V9 generator | The generator chain is dense and post-processed; the STEP is canonical |
| C3 | Outer reinforcement only on the outside of rails; solid top band with 2× M3 holes (56 mm apart) replaces the stereo carrier latch | Keep the camera opening and spring channels unchanged |
| C4 | Board-edge collar: guide walls 0.3 mm outside the PCB (to z 7.5), back ledges flush with the pads, bottom edge left open | Supports the board's edges; the FFC connector/ribbon is at the bottom edge |
| C5 | Build checks: springs swept over 4 mm release vs added material; holder vs camera model | Catch any interference with the snap mechanism or the camera |

## Wrist camera mount (2026-11-08)

| # | Decision | Why |
|---|---|---|
| W1 | Camera on the gripper's **+y side** (perpendicular to the jaws' swing plane), lens aimed between the jaw tips | Sees both jaws side-on with the object between them; the moving jaw never sweeps into it (as upstream's wrist cam) |
| W2 | **ESP32-P4 board on the holder's back** (on the gripper) | The flat camera ribbon then crosses no joints (FFC through a ±150° roll would fail); only the round USB cable runs down the arm. Costs ~12 g at the wrist |
| W3 | Placement chosen by a scored study in MuJoCo (`place_camera.py`): visibility, gripper overlap, table clearance, self-collision on **grasp-like poses** | Uniform joint samples include folded poses no task uses; grasp poses are what matter |
| W4 | Lens at (2, 66, −25) mm in the gripper frame | Same height as the gripper screws, so the bracket goes straight out (shortest, stiffest); 44 mm table clearance. Closer-in options hit the wrist link in 47–97 % of grasp poses |
| W5 | **Wrist roll capped at +0.80 rad** (range −2.74…0.80) | Positive roll near the base swings the holder into the shoulder; top-down jaw orientation repeats every π, so no grasp orientation is lost |
| W6 | Bracket reuses **upstream's wrist-camera flange** (2× M3 into the gripper's hex-nut recesses + locating key), registered from its STEP to the gripper frame (0.06 mm RMS) | Proven fit to the real gripper; no new holes in the arm |
| W7 | Holder bolts to a pad on the **front** of its top band via **M3 heat-set inserts** | The flange is in front of the holder; a back pad forced the arm through the holder, and nut traps under the arm would be unreachable |
| W8 | Assembly masses at typical print settings (~40 % of solid PLA): bracket 14 g, holder 16 g; camera 4 g, board 12 g (estimates) | Realistic wrist inertia in the sim; refine by weighing printed parts |
| W9 | Image "up" = holder top band (toward the wrist) | Sim convention; match the real sensor with the firmware's hflip/vflip after mounting |

## Simulation follow-ups (2026-11-08)

| # | Decision | Why |
|---|---|---|
| S12 | Wrist camera pose read from `wrist_mount_pose.json` (`pose_from` in cameras.toml) | CAD placement and sim camera can't drift apart |
| S13 | Grasp with **4 mm clearance** from the fixed jaw on descent | 0 mm made the fixed jaw land on the cube edge (held 0.6 s); 2–6 mm held 3 s at both rolls |
| S14 | Vision check = colour-mask centroid vs independent pinhole projection, per camera, per backend | Catches mis-posed, mirrored, flipped or wrongly-scaled cameras in either engine |

## Control software (2026-11-08)

| # | Decision | Why |
|---|---|---|
| T1 | pygame app (`sim/scripts/teleop.py`), same code for both backends | Runs on the Intel Mac (MuJoCo) and Linux (Newton); simple keyboard + mouse handling |
| T2 | Cartesian control by default (IK, fingers-down best effort) + joint mode | Cartesian is what you want for picking; joint mode for recovery and reaching odd poses |
| T3 | Click-to-move: ray from a fixed camera pixel onto the table plane | Uses the calibrated camera model directly; works in both fixed views |
| T4 | `--selftest` drives the window headless with synthetic events | Verifiable without a screen (CI, the VM) |

## Servo load / weight (2026-11-08)

| # | Decision | Why |
|---|---|---|
| L1 | Keep the wrist camera assembly as designed (46 g, board on the wrist) | Worst-case static holding torque +4–5 % of stall (shoulder 28 → 32 % with 30 g payload); far from the 80 % overload trigger; droop +0.3–2 mm |
| L2 | Realistic servo model (`servo_model.py`) with unknown stiffness swept (0.5 / 2 / 5 deg) | HX P-gain units are undocumented; conclusions must not hinge on a guess |
| L3 | Control software commands a gentle profile: 2.75 rad/s, 8 rad/s² | Commanding the no-load speed saturates every joint even without the camera (100 % peaks, 85–340 mm lag, 10–12 A); gentle: ≤ 75 % peaks, ≤ 50 mm lag, ~4 A |
| L4 | Gripper torque limit 50 % (register 16 = 500) | The only overload trips were the gripper squeezing at stall for > 2 s; a lower limit means less heat and squeeze, and the grasp works even after a trip |
| L5 | Power supply ≥ 10 A at 12 V if full-speed motion is ever used; 5 A OK with the gentle profile | Simulated current peaks |
| L6 | Measure real servo stiffness, overload semantics, temperature before trusting droop numbers | Model assumptions (see SERVO_LOAD_FINDINGS.md) |
