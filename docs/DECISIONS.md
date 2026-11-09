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
| S4 | ~~Torque limit ±2.94 N·m (HX-30HM stall); keep upstream STS3215 gains~~ → superseded: ±3.18 N·m at 12 V (F1) and a realistic position loop (P1) | Same gear ratio and speed; refit after system ID on real servos |
| S5 | Servo speed limit in software (`SlewLimiter`; the no-load speed at the supply voltage, 5.96 rad/s at 12 V) | Engines' position actuators have no velocity limit |
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
| W5 | **Wrist roll capped at +0.80 rad**; range −2.34…+0.80 (exactly π; was −2.74…+0.80 until A5) | Positive roll near the base swings the holder into the shoulder; top-down jaw orientation repeats every π, so no grasp orientation is lost |
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
| L3 | Control software commands a gentle profile: 2.75 rad/s, 8 rad/s² (now GENTLE mode; FAST is the default, F3) | Commanding the no-load speed with unlimited acceleration saturates every joint even without the camera (100 % peaks, 160–180 mm lag, 12 A); gentle: ≤ 55 % peaks, ≤ 10 mm lag, ~3 A (corrected run, L7) |
| L4 | Gripper torque limit 50 % (register 16 = 500) | The only overload trips were the gripper squeezing at stall for > 2 s; a lower limit means less heat and squeeze, and the grasp works even after a trip |
| L5 | ~~Power supply ≥ 10 A at 12 V if full-speed motion is ever used~~ → superseded by F1 | Simulated current peaks |
| L6 | Measure real servo stiffness, overload semantics, temperature before trusting droop numbers | Model assumptions (see SERVO_LOAD_FINDINGS.md) |
| L7 | Correction: no joint damping when the servo model drives the joints; add a derivative term (`d_time_s`) | Upstream's 0.6 N·m·s/rad damping stands in for motor losses in the ideal-actuator model; adding back-EMF on top halved the top speed. Static results unchanged; dynamic headroom much larger (gentle peak 72 → 45 %) |

## Fast teleop (2026-11-08, with the user)

See `sim/docs/FAST_TELEOP_FINDINGS.md`.

| # | Decision | Why |
|---|---|---|
| F1 | 12.0 V bench supply (`supply_voltage_v`), sized for ~20 A | User's choice: plugged-in supply, current not limited. 12 V gives +8 % torque and speed over the 11.1 V datasheet point while staying below the 12.6 V maximum; ~20 A = all six servos stalled |
| F2 | Torque governor (`so101sim/governor.py`) instead of a fixed profile | A fixed profile has to be set for the worst pose and servo; the governor predicts torque with the dynamics model every tick. 18 % faster clicks, lower peaks, no trips even with +40 % mass / 150 g unknown payload or the servo loop 4× stiffer or softer |
| F3 | Teleop defaults to FAST (governor, 0.25 m/s keys, 0.8 m/s clicks); `G` toggles GENTLE | User wants fast teleop; at 0.25 m/s the governor tracks to 16 mm mean vs 37 mm (112 mm worst) with the gentle profile |
| F4 | Click-to-move and Home travel a straight line, paced by the measured arm | The old joint-space jump drove the gripper into the table from a low pose (1.7 s at 100 % torque, stuck 111 mm short) |
| F5 | Smooth reference (critically damped, 35 rad/s), goals interpolated between frames, no velocity feed-forward | Bang-bang commands, frame-rate stop-and-go and stepped feed-forward each rang the lightly damped servo loop (100 % spikes); 50 rad/s rings the wrist |
| F6 | Load-aware IK available, tilt cone **default 0°** (`[` / `]`), ramped to 0° below 10 cm | User's choice, for dependability. Data agrees: 15° saves 8 % shoulder holding torque on average (45 % max) but almost only below 10 cm, where grasps need straight-down fingers; it doesn't lower the worst case |
| F7 | Don't flip wrist roll by 180° for load | Twin roll is inside the camera-safe range for only 23 % of rolls and saves ≤ 4 %; it would flip the wrist view |
| F8 | Teleop panel shows each joint's predicted torque (tick = overload threshold) and heat | Same numbers the governor uses; on the real arm compare them with present current (register 69) |


## Servo position loop and command compensation (2026-11-08)

See `docs/POLICY_TRAINING_IDEAS.md` (B0, B4, B7, B9).

| # | Decision | Why |
|---|---|---|
| P1 | Sim actuators use a realistic position loop: kp = stall / 9° (20.2 N·m/rad), kv 0, motor back-EMF as joint damping (0.534 N·m·s/rad) | Upstream / Menagerie kp 998 & kv 2.73 come from treating the P register as volts per degree with an assumed R = 1 Ω (~20× too stiff). Identified servos of this family reach stall only at ≈9–14° (BAM's STS3215 fit; Open Duck Mini). With the stiff model the scripted pick "worked" only because droop was hidden |
| P2 | 9° is a prior, set in one place (`servos.toml [follower.position_loop] sat_error_deg`) | Unmeasured until the HX bench test; the model generator and the feed-forward both read it |
| P3 | Every arm command gets gravity + back-EMF feed-forward (`so101sim/command.py`), in teleop (`V` toggles) and `pick_demo` | At realistic stiffness the pick fails without it (cube not lifted, MuJoCo and Newton) and succeeds with it; at home, droop drops from 4.9 to 0.2 mm. Study: robust to ±20 % mass error; fast-teleop lag 30 → 10 mm. Needs stiffness within ~±20 % |
| P4 | Grip by offset: after the jaws stall on an object, hold 3° past contact (≈33 % of stall) | "Fully closed" stalls the servo into overload protection (80 % for 2 s); in the study the offset grip removed every trip and still lifted. 3° keeps the squeeze under the gripper's 50 % limit |
| P5 | Verify the HX register map before any register write (incl. stock LeRobot `connect()`) | The vendor's "STS-compatible" claim and the HX register table v3.7 disagree on P/D/I, torque limit and torque-enable addresses; a wrong map could set the under-voltage threshold or a goal position |

## Audit fixes (2026-11-09)

An audit of the sim, the wrist mount and the UVC firmware found bugs the scripted checks missed. Fixes, and the
tests that now guard each class of bug (`sim/tests/`, run with `pytest sim/tests`).

| # | Decision | Why |
|---|---|---|
| A1 | Grip-by-offset detects contact from the measured jaw only (commanded more than the offset past the jaw, jaw not moving), not from the command having stopped changing | Teleop ramps the gripper command, and every change reset the old latch: it never engaged in teleop, so a grasp sat at the torque limit. Now the same for stepped, ramped and streamed commands |
| A2 | After latching, the contact angle is re-measured for 0.3 s while the squeeze relaxes; the latch releases when the object is lost or the command opens past it | The hold was 40 % of stall, not the designed 33 % (latched while still compressed at full torque), at the gripper's overload threshold. A lost object left the gripper "holding" forever |
| A3 | The governor never scales braking, and never commands outside the joint limits | With one joint already outside its torque window, braking was scaled to zero for every joint and the arm coasted through its goals (36 of 300 random joint-space jumps; up to 0.74 rad overshoot). Overshoot is now ≤ 0.09 rad on fuzzed full-range jumps |
| A4 | The lag guard uses the lag at the moment of measurement (`governor.observe`) | Teleop measures once per frame but the command advances ~17 steps; that progress was read as lag and fast moves were throttled (a 1.5 rad pan took 0.60 s instead of 0.43 s) |
| A5 | Camera-safe wrist roll is exactly π wide, [0.80 − π, 0.80] = −2.34…+0.80 rad, and the generated model enforces it (joint range and actuator ctrlrange) | The published range was 23° wider than the band the collision study sampled, and only teleop and `pick_demo` enforced the cap: a policy or a new script could swing the camera into the shoulder |
| A6 | Explicit contact pairs between the wrist-camera assembly and the wrist link | The assembly is welded to the gripper, so the engine skipped its contacts with the gripper's parent link: the one collision the placement study ruled out by sampling was invisible in the sim. Verified in MuJoCo; whether Newton's importer honours `<pair>` is unchecked |
| A7 | Teleop stall guard: an arm joint more than 12° behind its command for 0.4 s pulls the goal back to the arm | The tip-height limit only covers the tip. A joint jog could fold the jaws into the shoulder and hold 100 % torque; the load panel (a prediction) showed 4 %. Also: joint jogs that take the tip below the table are refused |
| A8 | A click or `H` in joint mode switches to Cartesian mode; a mode switch cancels any move; pause freezes the target too | A click in joint mode was stored and fired later on `J`; a paused arm jumped on resume |
| A9 | `ServoSpec.from_config()` reads the configured stiffness; `SlewLimiter` defaults to the speed at the supply voltage | Two more places where a number was read from the wrong source (2° and 11.1 V) |
| A10 | Every script parses its arguments (`--help`, unknown flags) | The two studies and the model generator ignored them, so `--help` started a 15-minute run that rewrote `sim/docs/` |
| A11 | Tests target classes of bugs: an invariant monitor on every scenario (no servo stalled or in overload, nothing outside limits, tip above the table), the same behaviour across all command shapes and measurement rates, seeded fuzzing, panel vs physics, config vs model vs CAD study vs docs, CLI smoke tests, one backend contract for MuJoCo and Newton | The self-test only looked at end positions on one path, so all of the above passed it |
| A12 | The fast-teleop and servo-load study numbers are kept but flagged as predating the 9° servo loop; rerun is an open item | They were measured at 2° stiffness without feed-forward (and the load study at 11.1 V with the gripper at 100 %) |

## UVC firmware (2026-11-09)

Not yet flashed; these remove the first-flash risks the audit found (`firmware/p4-uvc-camera/README.md`).

| # | Decision | Why |
|---|---|---|
| U1 | A dedicated camera task (8 KB stack) owns pipeline start and stop; the USB callbacks only post requests | Bring-up ran on TinyUSB's 4 KB stack and needs ~3 KB before any logging |
| U2 | One mutex around start, teardown and frame capture; 200 ms dequeue timeout; a failed JPEG encode returns no frame (30 bad frames in a row restart the pipeline) | Teardown could free buffers under the frame task, and a zero-length frame froze the stream until the host reopened it |
| U3 | The pipeline stops after 1.5 s without a frame request and restarts on the next one | The stop callback only fires on USB suspend, so the sensor, ISP and lens kept running after the host closed the stream |
| U4 | The build fails if the three patched `esp_video` files don't match their checksums | A fresh clone re-downloads `managed_components/` unpatched |
| U5 | Build-time options for h/v flip, fixed exposure + gain, fixed focus, JPEG quality; `usb_device_uvc` pinned to the exact version | Dataset recording needs fixed exposure and focus (white balance can't be fixed yet); the MAC serial number relies on a non-public symbol of that component |
