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
