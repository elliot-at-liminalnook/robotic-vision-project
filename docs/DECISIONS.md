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
| P5 | Verify the HX register map before any register write (incl. stock LeRobot `connect()`) | ~~The vendor's "STS-compatible" claim and the HX register table v3.7 disagree on P/D/I, torque limit and torque-enable addresses~~ → corrected in X10: the table agrees with the STS layout; the disagreement was a misreading. The check stays, because the table has still not been compared with a real servo |

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

## Buying (2026-11-10)

| # | Decision | Why |
|---|---|---|
| B1 | No leader arm; no HX-10HM servos | User's choice. Policies come from sim and teleop is keyboard/mouse |
| B2 | Servo driver: Hiwonder BusLinker V3.0 | Sold by the same store as the servos; has a screw terminal for the supply |
| B3 | Two stores, Hiwonder and DigiKey, not Amazon | User's choice: avoid Amazon, use as few sources as possible |
| B4 | Supply: Mean Well GST280A12-C6P (12 V, 21 A desktop brick) | Meets F1 (~20 A) with no exposed mains wiring. Its 6-pin plug is cut off and wired to the BusLinker's screw terminal |
| B5 | Camera USB cable = Molex PicoBlade 4-circuit lead spliced to a USB-A lead, plus a USB 2.0 extension | No ready-made MX1.25-to-USB-A cable was found at these stores |
| B6 | 10 A inline blade fuse on the supply's + lead | The 21 A supply would overheat the thin servo leads in a short. Limits usable current to 10 A |

Parts, links and prices: `BOM.md`.

## Low-level simulation: servo bus in the loop (2026-11-11, with the user)

See `sim/README.md`, "Servo bus in the loop".

| # | Decision | Why |
|---|---|---|
| X1 | Simulate the hardware's interfaces at byte level, and have the controller talk only to those interfaces | User's goal: debug the controller and firmware before (and alongside) the hardware. Not for policy training: it is serial and slow |
| X2 | Order: servo bus, then a camera model (latency, dropped frames, compression), then a PC build of the camera firmware | The bus driver is also the real-robot backend's lower half, so it pays twice. A PC build of the camera firmware tests its logic but not the image (the sensor, ISP and JPEG blocks would be faked) |
| X3 | The HX-30HM firmware is emulated from documents (register table V3.7, protocol manual, our motor model), not run | It ships as closed `.sfw` binaries for a microcontroller we haven't identified. The emulator's guesses are listed in `servo_emu.py` and logged whenever the controller relies on one |
| X4 | The servo emulator is Python | It is our model, not real firmware, so there is no C to keep faithful to. C matters for the camera firmware, which is real code |
| X5 | One clock for everything, one thread. Bus traffic takes time on it; a blocking read advances the simulation; the control loop keeps an absolute schedule and counts overruns | Deterministic and repeatable, and timing problems (a read that takes longer than a tick, a timeout longer than a frame) show up as numbers |
| X6 | `BusBackend` has the same API as the other backends (`q`, `set_targets`, `step`) | Teleop, `pick_demo` and the test rig run through it unchanged. MuJoCo only for now |
| X7 | The emulated servo's loop reads the same `sat_error_deg` as the model generator, and a test requires the bus path and the direct path to move the arm the same | One source for the stiffness (P2); the bus must add only what a bus adds (ticks, delay) |
| X8 | Where the firmware's behaviour is unknown, the emulator takes the strict reading: a goal write does not switch torque on; writes to read-only or undocumented registers are ignored and reported; out-of-range values are clamped and reported | A controller that leans on a guess fails in the simulation instead of on the bench |
| X9 | `BusArm.connect()` only reads, and refuses to continue unless each servo's registers look like the HX table (register 5 = its id, plausible baud index, response level, torque limit, mode) | P5 as code: nothing is written to a servo that hasn't been identified |
| X10 | Correction to P5 and `POLICY_TRAINING_IDEAS.md` B9: the HX register table V3.7 **agrees** with the Feetech STS layout (16 torque limit, 21/22/23 P/D/I, 40 torque switch, 42 goal, 56 position). The reported conflict came from the spreadsheet's hex column, which holds formulas and was read one row off; the decimal column is the address. Real differences: the HX table lists **no angle-limit registers (9–12)** and no run-time torque limit (48); current is in 1 mA and speed in steps/s | Found while transcribing the table for the emulator; a test now compares our table with the spreadsheet. Still unverified against a real servo |
| X11 | Teleop's stall guard and lag guard skip a repeated encoder reading (a failed bus read), and the driver waits one timeout per failed read, not two | Found with the bus simulation: with 5 % of replies lost, corrupted and commands missed, teleop saw old readings as an arm that wasn't following, reported "blocked" and stopped 20 mm short of the cube. A lost reply also stalled the loop for 40 ms, longer than a 33 ms frame |
| X12 | The bus timing numbers (`servos.toml [bus]`: servo response delay, USB latency each way, echo, reply timeout) are placeholders | Unknown until measured on the BusLinker; a 20 ms timeout is already most of a control frame |

## Low-level simulation: camera firmware in the loop (2026-11-11, with the user)

See `firmware/p4-uvc-camera/host/README.md` and `sim/README.md`, "Camera firmware in the loop".

| # | Decision | Why |
|---|---|---|
| X13 | The camera model and the PC build of the firmware are one thing: the firmware source is compiled unchanged, and the "camera model" is the stand-in hardware under it (sensor timing, crop, encoder, USB link) plus the PC's buffer queue above it | X2 planned them as two steps. Built separately, the model would have needed its own copy of the firmware's behaviour (start-up time, pacing, idle stop), which is the part most worth testing |
| X14 | The firmware file is `#include`d into `host/host_api.c`; its `open`/`ioctl`/`mmap` calls are redirected by a force-included header | No edits to the firmware for the sake of the simulation, and tests can read its private state (`s_cam.running`, the lock) |
| X15 | FreeRTOS stand-in: one thread per task, one running at a time, virtual time, priorities, hand-over when a higher-priority task is readied; no preemption inside plain code | Exactly repeatable runs, and no dependence on the PC's speed. The cost: races that need an interrupt between two statements are not explored, and stack sizes are not modelled |
| X16 | The C side never calls Python. It returns events (`need a picture`, `need a JPEG`, `frame arrived`) and Python answers | MuJoCo's renderer must be used from the thread that created it; firmware tasks run on other threads |
| X17 | The UVC task in the stand-in follows `usb_device_uvc`'s `video_task` line for line | That loop sets the frame rate and decides what a failed frame costs; an approximation would have hidden both |
| X18 | The stand-in devices validate how the firmware uses them and list every misuse; tests require an empty list, and one test misuses a device on purpose to prove the checks fire | The firmware's riskiest code is buffer and lifecycle handling (audit items U1–U3); checks in the hardware model catch whole classes of those mistakes |
| X19 | A sensor mode is a pinhole camera with the calibrated focal length in pixels and the mode's size, so a smaller size sees less. In teleop only the wrist camera goes through the firmware | Matches the driver (every mode is a centre crop at one pixel pitch). Click-to-move needs the fixed views' calibrated full picture |
| X20 | Firmware fix: `s_last_fb_get` starts as "long ago", not 0 | Found by the PC build: for the first 1.5 s after boot the camera task treated frames as wanted, so a start that failed in that window (no camera on the cable) was retried with nobody reading. Not rebuilt with ESP-IDF since the change |
| X21 | Left as designed, now measured: one encoder queue error stops the stream for about 1.7 s (30 bad frames, then a restart) | The firmware can't know the encoder's buffer state after such an error. Worth revisiting only if these errors turn out to happen on the board |
| X22 | Stand-in timings (bring-up 0.5 s, readout 20 ms, encode 8 ms per megapixel, link 8.18 MB/s) and the PC's 4-frame queue are placeholders | None has been measured. Simulated pictures also compress much better than real ones, so USB frame rates at large sizes are optimistic |

## Model audit and fixes (2026-11-11, with the user)

An audit of the generated model against upstream, the CAD and the true meshes. Fixed in this order.

| # | Decision | Why |
|---|---|---|
| M1 | The camera bracket's collision shape is 8 convex pieces, and the wrist link's meshes are decomposed too | As single hulls they overlapped by up to 1 mm for wrist roll between −0.7 and +0.8 rad, whatever the other joints did: phantom friction costing wrist roll 1.4 N·m on average and 2.1 N·m at peak (67 % of stall) in a slow sweep, against 0.3 N·m outside that zone. The real parts clear by 0.7 to 2 mm. Cause: the explicit contact pairs of A6 on top of hulls that fill the parts' hollows. Home has roll −1.57, outside the zone, so no test saw it |
| M2 | Every arm part whose hull was more than about 1.5 times its volume is decomposed (ten meshes, 8 to 16 hulls) | Same class as M1 and S2. Step time is unchanged (0.10 ms) |
| M3 | The base gets collision shapes, with an explicit exclusion against the shoulder | Upstream gives it none: a cube dropped on it fell through to the table and the arm could enter it. The exclusion is needed because MuJoCo does not skip parent/child contacts when the parent is fixed to the world |
| M4 | The base is raised 2.4 mm so that its lowest face is on the table | Upstream's origin is above the base's lowest flat (20 cm² at −2.4 mm, 36 cm² at 0), so the base was sunk into the table top. Assumes the base rests on its lowest faces; check on the printed part |
| M5 | The table is solid down to the floor | A box pushes an overlapping shape out through its nearest face: the 40 mm table top let a cube at 5 m/s through. Now nothing passes at 40 m/s |
| M6 | Contact `solref 0.004`, `solimp 0.95 0.99` on every shape | With MuJoCo's defaults the arm at full torque sank 10 mm into the table and pressed a cube 7 mm into it. 0.004 s is the stiffest a 2 ms step allows; now 1.3 mm on impact, 0.04 mm at rest, and a pressed cube 0.6 mm |
| M7 | The wrist camera assembly's centre of mass and inertia come from the part meshes (mass spread evenly through each) | They came from bounding boxes: centre of mass 2.6 mm off, inertia 1.3 to 2 times too high |
| M8 | `vision_check.py` prints the number it compares, allows 3 px (was 12), and checks each edge of the cube's outline where nothing hides it | The centre of a near-central object barely moves with a wrong focal length; the edges do. Measured: under 1 px |
| M9 | `pick_demo.py` moves on the speed-and-acceleration profile of `servos.toml` | It limited speed only, so every move started and stopped with a step in speed |
| M10 | A correction to my own audit: clearances between parts must be taken from the compiled model's mesh, not the STL file | MuJoCo re-centres meshes on load. Two numbers in the first audit report were wrong for this reason (the base "41 mm into the table": really 2.4; the bracket "5 to 9 mm clear": really 0.7 to 2) |

## Physical invariants (2026-11-11, with the user)

See `sim/README.md`, "Physical invariants". Limits in `sim/config/safety.toml`.

| # | Decision | Why |
|---|---|---|
| V1 | Enforce each invariant in several layers: model, command filter, physics monitor, tests | A model fix prevents; the filter stops bad commands on the real arm too; the monitor catches what neither foresaw; tests keep all three honest |
| V2 | The command filter is part of every backend (`set_targets` → filter → actuators or bus), not of the scripts | Nothing can bypass it: a new script or a policy gets it without knowing |
| V3 | The filter limits **jumps**, not a command's speed or acceleration | Tried first, and wrong: with soft servos and feed-forward the command leads the arm and jitters, so ordinary teleop commands show 1400 rad/s² tick to tick. A limit there mangles good commands and does not make the arm gentler |
| V4 | A jump is closed along a straight line in joint space at the governor's speed and acceleration caps, while still following the request | The fastest motion the controller itself ever plans, so it is not a new behaviour. Per-joint catching-up bent the path and could walk the arm into a keep-out |
| V5 | Keep-outs (table, base) are checked on the **intended** pose, which callers that add feed-forward pass along | The compensated command can point 11° past the arm. Checked on the command, the keep-out fired in every fast descent though the arm was clear |
| V6 | A blocked step means the command stays exactly where it was. If the filter has been stuck for 0.2 s while the arm is more than 6° from the accepted pose, it restarts from where the arm is | Otherwise the accepted pose can sit at a keep-out's edge, a place the arm never was (something else held the arm back), and bar the way back |
| V7 | The monitor allows 5 mm of overlap for a single step and 1.5 mm sustained; for wires, their radius | An impact at full closing speed overlaps 2 to 3 mm for one or two steps even with the stiffest contacts; a sustained overlap is something being pushed into something |
| V8 | A `pick_demo` run with a physics violation counts as failed | A pick that needed impossible physics is not a pick |
| V9 | **Not done: servo-side speed and acceleration caps.** | Speed is already bounded by the motor (it cannot exceed its no-load speed). The acceleration register would make the servo plan to stop at every goal it is sent; on a stream of goals that means lagging by v²/2a (about 20° at full speed with the largest setting), by the emulator's reading of the register. Worth revisiting only for point-to-point moves, and only after the register's behaviour is checked on a servo |
| V10 | Known and left alone: one large jump of the Cartesian target makes the joints race, and the planned path dips the fixed jaw's tip about 3 mm under the table top | Found by the filter, which now cuts it short. Clicks, Home and the wheel are straight-line or small moves; only a programmatic 8 cm jump does it. Tests that descend use the straight-line move |

## Wires (2026-11-11, with the user)

See `sim/README.md`, "Wires", and `sim/config/cables.toml`.

| # | Decision | Why |
|---|---|---|
| K1 | Wires are chains of capsules with ball joints (spring = bending stiffness), not MuJoCo's cable plugin and not tendons | Plain MJCF: no plugin to load, and it should carry over to Newton / MuJoCo Warp. A tendon has no slack shape and touches nothing |
| K2 | A separate model file, off by default (`load_backend("mujoco", cables=True)`, `teleop.py --cables`) | 90 segments make a step about ten times as expensive (1.1 ms). Like the bus and camera layers: detail for checking a design, the fast model for everything else |
| K3 | One end fixed in its connector, the other held by a ball constraint; laid out along a slack curve at every reset, then settled for 0.4 s with the arm and clock held | A chain's rest shape is straight, so it cannot start from the model's reference pose; and the layout must work for any arm pose |
| K4 | Wires never collide with each other; every wire collides with the table and objects; only the exposed ones (two bus leads, the USB cable) collide with the arm | The other four leads run inside or against the brackets. Laid out through solid plastic, one jammed the pan joint with millions of newtons. Their real path through the hollow parts is not known well enough to model |
| K5 | Wire contacts are softer than the rigid parts' (`solref 0.01`), and a wire's values win in any contact | A wire gives; and one laid out through a part then works its way out instead of being shot out |
| K6 | The USB cable is clipped half-way along the forearm and the upper arm, on the side the servo sockets are on, 8 mm proud, and taped to the table behind the base | A choice, since the routing was open (PLAN). It keeps the cable off the gripper and gives each joint its own slack |
| K7 | The USB cable's first stretch (board to forearm clip) is 260 mm | At 200 mm, 20 % more than its ends are ever apart, the simulation showed it wrapped around the wrist link and pulled taut at one pose (3.7 N). The straight-line check is necessary, not sufficient |
| K8 | Weight per metre and bending stiffness of each wire are guesses, marked in the config with how to measure them | Nothing here depends on them yet: slack wires cost under 5 % of stall torque either way. They matter once a wire is nearly taut |
| K9 | Camera ribbons are chains of flat strips joined by hinges (they fold about one axis only), 1 mm thick | A ribbon cannot twist or bend sideways; a chain of ball joints would let it. 0.1 mm is too thin for contact |
| K10 | The fixed cameras get placeholder hardware in the cabled model only: the wrist's holder, camera and board, placed so the lens is exactly at the configured camera position, on a plain post | A cable needs something to hang from, and their stand is not designed (PLAN). Kept out of the plain model so that nothing else changes until the stand is real |
| K11 | A fixed camera's USB cable is tied to the top of its post, right behind the holder | It leaves the board upward. Free, it flopped forward within a few seconds of simulated time and hung straight down across the lens: a dark bar down the middle of that camera's picture |
| K12 | The supply lead and the driver board's USB lead are modelled from the board to the table's back edge only | Beyond that they are off the workcell and touch nothing |

## REST API for a language model (2026-11-11, with the user)

See `sim/README.md`, "REST API for a language model".

| # | Decision | Why |
|---|---|---|
| R1 | Turn-based: simulated time moves only inside an action, which blocks until it is done or times out | A model takes seconds to answer. With the simulation running meanwhile, the result would depend on how fast the model is; this way a run is repeatable and a slow model is not penalised |
| R2 | The API drives the Teleop controller (straight-line moves, governor, feed-forward, grip-by-offset, stall guard), behind the command filter and the monitor | One controller to trust and test, and the model gets the same protections as a person. It also means the API will carry over to the real arm when teleop does |
| R3 | Actions are few and at the level a person thinks: move the tip, move joints, set the gripper, wait | A model is good at sequencing and bad at servo loops. No velocity or torque commands |
| R4 | "Tip" in the API is the midpoint between the jaw pads; the controller's own tip is on the fixed jaw's pad | With the controller's tip, sending the tip to an object's centre drove the fixed jaw onto the object (the first API pick failed that way). `gap_m` and `square_jaws` exist for the same reason: they are what a clean grasp needs |
| R5 | Every refusal is a sentence with the allowed range, unknown fields are refused by name, and a refused request changes nothing | The model's only way to recover is to read the answer |
| R6 | The guide (`GET /`), the OpenAPI document and request validation come from one table | They cannot drift apart; a test checks every endpoint and parameter is in all three |
| R7 | Standard library HTTP server, one request at a time, bound to 127.0.0.1, no authentication | No new dependency. One thread also keeps MuJoCo's renderer on the thread that made it. Not for exposing to a network as it is |
| R8 | `/state` gives the objects' true positions by default; `--no-object-positions` withholds them | Useful for getting a model going; not available on a real robot, so it can be switched off to make the model use the cameras |
| R9 | `--hands-on` (2026-11-12): the API's window also takes teleop's keys and mouse between requests, and the simulation then runs at real speed; keys are ignored while a request's action runs, and an action unpauses a paused simulation | The user asked to control the arm together with the model. Taking turns needs no arbitration between two sources of commands: both go through the one teleop controller, one at a time. The cost is that time passes between requests, so such a run is not repeatable; without the flag nothing changes |
| R10 | `POST /randomize` (2026-11-12): objects are placed upright with a random turn, 0.20 to 0.31 m from the base and within 65 deg of straight ahead, centres at least 75 mm apart, and a layout is redrawn until each object touches only the table. The answer gives the seed, and the positions only if `/state` would | The user asked for a random layout to be worked from pictures. The band is where the arm reaches with fingers down (T7), so every layout can be worked; the contact check keeps objects out of the fixtures in any scene without a list of them; the seed lets a layout be replayed afterwards to score a picture-only run |
| R11 | No camera calibration in the API. `GET /pixel` (the ray through a pixel and where it meets the table, from the camera's pose and field of view) was added on 2026-11-12 and removed the same day at the user's request | With it, both the author and a fresh agent given only the guide built a tower from pictures on the first try, locating objects within 1.5 mm: the endpoint did the hard part. The user wants to see how a newcomer manages with the pictures alone |
| R12 | The guide (`GET /`) is written for the server it is served by (2026-11-12): it lists the scene's objects with their sizes and colours, sends the caller to `objects` in `/state` only if positions are given, says that time runs between requests under `--hands-on`, says where each camera stands and what the arm looks like in the pictures, and explains every message the controller and the filter can give. `/gripper` and `/wait` now set their own message | A fresh agent given only the guide reported each of these as wrong or missing: the walkthrough pointed at a field that was withheld, sizes had to be guessed from pictures, the clock moved when the guide said it could not, messages were stale or unexplained, and the arm's own coloured parts were taken for objects. Tests tie the guide to the model and to the filter's source so it cannot drift again |
| R13 | `/gripper` changes only the jaws' target (2026-11-12). Before, it re-solved the arm's pose for the tip with the fingers down | Found by the second newcomer run: it had turned the wrist with `/move_joints` to hold a cylinder upright over a tower, and opening the jaws swung the arm back to fingers-down through the tower. A test now checks, from poses reached either way, that the five arm joints stay put when the jaws move |

## Scenes and tasks: the bench (2026-11-11, with the user)

See `sim/README.md`, "The bench scene and task".

| # | Decision | Why |
|---|---|---|
| T5 | A second scene, `bench`: a cabinet with a sliding drawer, two bins, a platform, a socket for a peg, five loose parts | The user asked for a really complex environment and task. It adds an articulated fixture, containers with walls, an insertion with 4 mm of clearance, stacking at height, and steps that depend on each other |
| T6 | A task is a description plus goals about the end state only, all of which must hold at once | Goals can then be checked at any moment and in any order, by `GET /task` or a test, without watching the run. Order still matters where the scene forces it: the red cube is in the closed drawer |
| T7 | "Physically feasible" is proved, not argued: a reference solution made only of REST calls does the task, and a test runs it | The first layout looked fine and was not: three places were out of the arm's fingers-down reach. The solution found each one |
| T8 | Everything that must be approached from above sits between 0.20 and 0.32 m from the base | Measured: with the fingers straight down the arm reaches 12 cm or higher only in that band; closer in the ceiling drops (10 cm at 0.18 m, 5 cm at 0.12 m), too low to carry anything over the fixtures |
| T9 | `/move_tip` takes `jaws_along_deg`, and `/state` reports which way the jaws close | The task needed it twice: the low block rolls over if pinched across its short side, and at the drawer handle one jaw otherwise lands on the cabinet's roof |
| T10 | The reference solution holds low parts with the tip at least 10 mm above the table | Lower, the gripper could not close: the moving jaw swings down a little as it closes and the command filter refuses a pose with any part below the table top (V5). The filter is right; the grasp height was wrong |
| T11 | The drawer does not rest on the table (1 mm clear) and stays put by friction in its slide (0.5 N) | Contact between a moving tray and the table adds nothing but solver work. Not measured against any real drawer: it is a made-up fixture |

## Stop path (2026-11-14, with the user)

See `sim/README.md`, "teleop.py" (Stop path). Tests: `sim/tests/test_stop.py`.

| # | Decision | Why |
|---|---|---|
| E1 | Two software levels: **freeze** (`X`: the goal becomes the measured pose, torque stays on) and **torque off** (`T`, and on every way out of the program) | Every other protection assumes the controller and its model are right. Freeze is for "stop that": nothing drops. Torque off is for when holding is itself the danger, and for leaving; the arm sags |
| E2 | Not built: a power cut. A switch on the 12 V lead is still to buy | The only level that works with the computer or the bus dead. Software cannot stand in for it |
| E3 | Freeze uses the stall guard's own code (`Teleop.hold_here`), and keeps the gripper's command | One way of "start again from where the arm is" to trust. Opening the jaws on a stop would drop what is held |
| E4 | After a stop, movement keys that are still down do nothing until they are let go | Found by the first test: a key held through the stop moved the arm again on the next frame |
| E5 | Torque is a backend operation (`torque_off()`, `torque_on()` returning the pose now held), and while it is off the backend sends nothing | Same reason as V2: no script can forget it. Feetech firmware switches torque on when a goal is written (HX unverified), so "off" has to mean "no goal writes" on the controller's side |
| E6 | Torque on holds the pose the encoders report at that moment, and is refused without a fresh reading | The arm has fallen since the last command. Any older pose would make it jump |
| E7 | Torque switches and `enable()`'s goals are read back from every servo; a servo that differs is written singly with an acknowledgement; one that cannot be confirmed is named | They went out as broadcasts, which have no reply. A servo that missed the goal but got the switch would drive to its old goal (0 ticks after power-on) |
| E8 | The program switches torque off in one `finally`; SIGTERM is turned into a normal exit so it gets there too | Before, every exit left the servos holding their last goal at up to full torque |
| E9 | Assumed: a servo with torque off coasts (no back-EMF braking), in both MuJoCo backends | Unknown for the HX. Coasting is the worse case: the arm falls faster. Check on a servo |
| E10 | The API refuses actions while torque is off (hands-on mode); it has no stop endpoints | A remote caller must not switch on what the person at the arm switched off. A stop request has no meaning in a turn-based API where time only runs inside an action |
| E11 | Not done: Newton backend (no `torque_off`; teleop reports it), `pick_demo.py`, and stops triggered by servo feedback (temperature, status bits, lost replies) | Newton cannot be run on this machine; the feedback watchdog is the next piece |

## Rover (2026-11-14, with the user)

See `MULTI_DEVICE_PLAN.md`, section 5.6. Nothing is built or bought.

| # | Decision | Why |
|---|---|---|
| RV1 | The rover runs from a battery: Gens ace 5000 mAh 3S 60C LiPo with XT60 (11.1 V, 154 × 42.5 × 24.7 mm, 355 g, $70.09) | User's choice. A 3S pack's 9.9 to 12.6 V is the HX-30HM's own range, so the wheel servos take it directly. Larger than needed by my estimate (55 Wh against roughly 5 to 15 W), and it makes the rover about 170 mm long and 570 g |
| RV2 | Follows from RV1: the rover's data goes by Wi-Fi, through a board on the rover that also drives the wheel servos | A battery with a data cable is still a tether. Needs new firmware, a 5 V regulator for the boards, a fuse at the pack and a balance charger; none chosen yet |
| RV3 | The hub reads the battery's voltage from the wheel servos (register 62) and stops the rover before about 3.3 V per cell | No extra part, and the servo's own under-voltage threshold (4.0 V by default) is far too low to protect a LiPo |

## Multi-device system (2026-11-14, with the user)

The design, the steps and the open questions are in `MULTI_DEVICE_PLAN.md`; progress in `MULTI_DEVICE_CHECKLIST.md`.

| # | Decision | Why |
|---|---|---|
| MD1 | One language model commands several devices (the arm, fixed cameras, a rover); devices form a logical network, grouped by interest in a tree in the hub | User's goal and choice |
| MD2 | The system is given no prior information about its surroundings: no table size, object list, camera or device positions, or count of devices | User's choice: it should work out a place it has never seen |
| MD3 | Ground truth is removed from the model's API. It lives behind a separate operator API and a referee | User's choice: the system has to orient |
| MD4 | Scenes are TOML config files, loadable while the program runs and editable over REST | User's choice. TOML matches the repo's other configs; the cost is that an edit over the API drops a file's comments |
| MD5 | MuJoCo only for this work, not Newton | User's choice. Newton's renderer shows no textures, so it could not show markers |
| MD6 | The hub is a top-level package beside `sim/` | User's choice |
| MD7 | OpenCV for reading tags (AprilTag 36h11); tags on devices only | User accepted the recommendation |
| MD8 | The rover's two cameras sit on a motorised head (spacing, tilt, pan, lift), and the rover surveys from where it stands at start-up | User's choice |
| MD9 | A lip round the surface keeps the rover on the table | User's choice |
| MD10 | Camera pictures pass through a realistic lens model | User's choice |
| MD11 | A scene is built in memory from the arm's own model plus MuJoCo's `MjSpec`; nothing is written to disk | That is what lets a scene be swapped at run time. `sim/scene/*.xml` and `tools/build_model.py` stay for the old path |
| MD12 | Validation runs no device: fields, compile, placing, overlaps, one second left alone, tasks | User's choice: create and validate before anything runs. It still caught real faults in the first drafts (the arm parked through a shelf; the rover placeholder thrown off the table) |
| MD13 | The rover placeholder stands on four round feet, not on its flat underside | A large flat box on the deep table box got a false contact 0.33 m deep from MuJoCo's box-box test at one position and heading, and was thrown 19 m. Small boxes (the loose parts) have not shown it. Worth remembering for any large flat-bottomed body |
| MD14 | An arm in a scene starts at teleop's home pose (tip 16 cm up, 20 cm ahead) | The old test pose had the gripper low over the table, inside things placed in front of the arm |
| MD15 | Tasks are data in the scene file; a task that swaps scenes (`swap_to`) is scored in the scene swapped to | User's choice that tasks be created as files; "carried to another table" is a task about a swap |
| MD16 | The arm's tags are on two tag blocks beside its base (four faces each) and one plate behind the wrist camera, not stuck on the arm's own faces | A camera anywhere round the arm then sees at least two tags of one rigid body on different faces, which removes the flipped reading of a single flat tag. The arm's own faces are small and curved |
| MD17 | Tag places are written once, in `hardware/tags/layout.toml`; the arm's model is generated from it and the hub's driver reads it | A device's tags are part of what it is (built in), and one source means the simulated arm and the hub's belief cannot drift apart |
| MD18 | The tag blocks and plate are in the arm's own model, with collision shapes | The command filter works on that model, so the arm keeps clear of them without being told. `safety.py` now takes boxes as well as meshes on the base |
| MD19 | A camera fix is trusted only with two or more base tags not all in one plane and a pose that explains them; otherwise it is reported as ambiguous with a very large uncertainty | Seen in the first checks: two front faces alone (one plane) gave 38 mm and 2.7 deg of error at 720p. Said, not guessed |
| MD20 | Stated accuracy of locating a camera from the arm's tags, ideal pictures: within 3.5 % of the camera's distance and 2 deg at 1080p; 8 % and 4 deg at 720p | Measured over the scenes and twelve seeded placements at 0.35 to 0.85 m. Not enough for grasping by itself, as the plan expected; the wrist camera and touch are the close-range checks |
| MD21 | The lens model is applied after rendering: render wider and larger, then warp with OpenCV's lens model; then darkening, blur and noise. Values are placeholders from the datasheet (k1 = −0.033 from its 102° against a pinhole's 99°) | MuJoCo only draws pinholes. The warp was checked against OpenCV's projection to 0.03 px. Real values need one of the cameras on hand calibrated (plan O7) |
| MD22 | Through the lens, locating a camera is about twice as bad (median 8 → 18 mm, worst 21 → 37 mm at 1080p), and the nominal distortion correction does not help | A unit's focal length differing from the datasheet dominates: 2.2 % long is 2.2 % of the distance. So self-calibration (plan step 12) is what buys accuracy back, not a better nominal model. Stated until then: 6 % of distance, 2.5° |
| MD23 | The hub's nominal lens for the Camera Module 3 Wide includes the datasheet-derived distortion, and its centre is in pixel-index coordinates (959.5, 539.5) | Built-in knowledge is what a maker's datasheet gives. Half a pixel is small, but one convention everywhere avoids a bias |
| MD24 | The arm's controller-side code moved from `sim/so101sim` into `hub/` (controller, actions, IK, dynamics, governor, feed-forward, filter, bus driver); the old module names are aliases of the same modules | The hub may not import the simulation, and this code is what runs on the real machine. Aliases keep every old import and test working. The arm's description files are still read from under `sim/` (generated model, `servos.toml`, `safety.toml`): moving them is housekeeping |
| MD25 | The world gives the hub ports and nothing else: a serial port per servo bus, a camera port per camera, the clock. The hub asks for them again at every discovery | The seam between simulation and real machine. A port that outlives a scene swap goes silent, as an unplugged adapter would |
| MD26 | An arm's servos start powered and holding its starting pose | Otherwise an arm no hub has connected to collapses onto the scene before anything starts. On the real arm the hub's take-over is the same either way: read where it is, hold there |
| MD27 | The hub finds the wrist camera by turning the wrist 0.3 rad and seeing whose picture changes | A camera says what it is, not where it is mounted, and the hub is told nothing. Right in all seven scenes with an arm |
| MD28 | Eight position reads failing in a row is an error ("the arm has not answered"), not a repeated old reading | Found when a scene was swapped under the hub: an action ran to its timeout on stale readings and reported "not reached". One lost reply is still noise (X11) |
| MD29 | `square_jaws` squares the jaws to the arm's own axes, and a caller that knows a thing's heading uses `jaws_along_deg` | Found with the arm turned 225 deg: square to the arm was 45 deg to the cube, and a jaw landed on its corner. There is no shared frame for "square" to mean anything else |
| MD30 | Retiring `sim_api.py` waits for the last step | It still carries the bench task's reference solution, its tests and the hands-on window. Its arm actions are already the hub's |
| MD31 | All of one camera's sightings of the arm's tags are solved together, as points known in the arm's frame against pixels | One flat tag alone can be read tilted either way; the wrist tag at several arm poses, or two faces of a tag block, cannot. It also fuses base tags and wrist-tag poses without choosing between them |
| MD32 | The arm contributes by showing the tag behind its wrist camera at six poses near its home (`POST /locate {"show_wrist_tag": true}`) | A camera that cannot see the arm's base (the `l_shaped` scene's) was located this way to 12 mm. The wrist tag's place comes from the joint angles and the arm's own shape |
| MD33 | The frame called "arm" is the one its commands use (origin on the surface under its base), not its base part's (2.4 mm higher) | One frame for where to send the tip and where things are |
| MD34 | Stated uncertainty has a floor of 1.5 % of the distance (and 0.4 deg) for an uncalibrated lens | With many sightings the fit's own covariance shrank to 1 to 3 mm while the true error was 6 to 17 mm: the error is systematic (this lens is not the datasheet's) and does not average away. With the floor, every case tried was inside 3.5 times what was stated. It can come down once the hub calibrates each camera (plan step 12) |
| MD35 | Chained links carry the full 6 x 6 covariance, cross terms included | The first version dropped them; the test that compares a chain with sampling caught it |
| MD36 | New sightings more than 12 px from where the old pose puts them mean the camera (or the arm) was moved: earlier sightings are dropped | Otherwise a moved camera is averaged with where it used to be |
| MD37 | The rover is designed by script and the simulated rover is generated from the script's output (`hardware/rover/rover.json`), as the wrist mount was | One source for dimensions, masses, joints and collision shapes. The script checks every pair of bodies at 93 poses of the head and found four overlaps in the first draft |
| MD38 | Rear-wheel drive with the ball caster at the front; a skid at the back | The stowed head puts the centre of mass 68 mm ahead of the axle. With the caster behind, the rover would nose-dive |
| MD39 | All six rover servos are HX-30HM | One kind of servo, one register table, one voltage; only the tilt and spacing servos ride on the head. Smaller head servos are still worth pricing (plan O6) |
| MD40 | The camera-spacing drive's zero is at tick 148, so its 334° of travel lies inside one turn | The HX loses its turn count at power-off. A joint whose travel crosses a turn boundary would come back in the wrong place |
| MD41 | Caster and skid contacts have priority, so their own low friction is used | MuJoCo otherwise takes the larger of two surfaces' frictions (the table's 0.5): the caster dragged, the rover turned 22° where its wheels said 79°, and rolled 10 % short |
| MD42 | The emulator's motor mode (operating mode 1) is a speed loop with P and I terms on register 46, bit 15 the direction; the position reads within one turn | From the register table's one-line description; the gains' units are undocumented, so the loop's behaviour (settles in about 50 ms, no steady error) is a guess. To check on a servo, with the rest of the table |
| MD43 | A rover's servos are reached through a radio link with delay, jitter and outages, and the rover's own board stops the wheels after 0.4 s of silence | The battery decision makes the link wireless, and a late or lost packet must not be a rover that keeps driving. All timing numbers are placeholders |
| MD44 | A rover's cameras announce that they are the rover's and which side | Its boards are provisioned with it: a device knows its own parts. (The arm's wrist camera is found by turning the wrist, because that camera is a plain USB device) |
| MD45 | Head servos are limited to 35 % torque | Found in the first check: a head pushed against a post did not stop; it turned the whole 0.9 kg rover, and the wheels could not count it. Lower torque makes such a push gentle; noticing it needs cameras (later steps) |
| MD46 | One lost reply from the rover is retried; six in a row is an error | The radio link's occasional long delays exceed the reply timeout. The first version let one lost read escape as a server error |
| MD47 | The rover's parts list is in `BOM.md`, chosen from store pages read on 2026-11-14: $416.27 confirmed for one rover; nothing ordered | The plan asked for a real parts list. Three things it turned up: the ESP32-P4 board is out of stock everywhere checked; no lighter servo fits the head (so HX-30HM throughout, settling plan O6); the caster sold is taller than the one drawn |
| MD48 | The rover's bus interface is a BusLinker V3.0 in UART mode, driven by a third ESP32-P4 board fixed to the chassis | Documented by Hiwonder with this servo at 1 Mbps; the camera boards ride on the moving carriages. A bare buffer IC would be smaller but the bus's logic level is not documented |
| MD49 | The survey is three passes: low (0.10 m, 25° down, 60 mm apart), high and near (0.16 m, 38°, 60 mm), high and far (0.16 m, 12°, 100 mm); the head turns outward from straight ahead in 40° steps | The spacing stays under 0.6 of the height: from low down with the cameras far apart the ground's near rows shift so much more than its far rows that the pictures cannot be matched. A thing 0.17 m from lenses 100 mm apart is seen from 34° apart and was not matched at all; at 60 mm it is |
| MD50 | Matching runs at half size and again at quarter size; where they disagree the coarser is believed | A thing nearer than the search reaches is not left unmatched: it is matched, confidently, to something else, and lands far away. Found as a cube 36 mm from where it was |
| MD51 | Each cloud is laid on the ground found in it; heights are not taken from the head's joints | A lens centre 11 px off tilts everything that camera sees by 0.7° |
| MD52 | The pair's sideways shift (the lens centres' difference) is not measured yet; distances are stated as good to about 5 % | Tried from the spacing drive: the same view at two spacings gives the shift in principle, but an error of 0.3 % in a spacing is 4 px of shift, and the estimates disagreed by more than what they were measuring. Step 12 does it from tags, whose size is known |
| MD53 | Only the middle of each picture (31° each side) and out to 7 times the spacing go into the map | Two lenses differ most at the sides; and a pair's error in distance grows as distance squared over spacing |
| MD54 | Both pictures of a pair are taken only once the head has been still for 0.3 s | The two cameras are separate boards. A move counts as reached within a few degrees and the head goes on swinging: one picture was several pixels beside the other, which put far edges 10 % out and made a tag measure 0.87 of its size. With the wait: 1.016 |
| MD55 | The pan servo is held to 5 % of its torque (the rest of the head stays at 35 %) | The head's end leads its own cameras by 0.22 m into places nothing has seen. At 35 % a head that met a post turned the rover 175° and every move reported "reached". At 5 % the pan stalls, the lag is noticed and the rover is turned 3 to 5°. At 2.5 % it cannot reliably move the head |
| MD56 | The survey does not turn again towards where the head met something, at any height; at the end it takes its first view again and measures how far the picture has slid. Over 8°, or no match: the map and what the rover saw are dropped | The wheels cannot count a rover pushed round on its tyres. A gyro on the rover would say it directly: worth adding to the parts list (not decided) |
| MD57 | A map cell whose points scatter below the surface is left as never seen | That is what stereo makes of something too near the lenses; nothing real is seen under the surface the rover stands on |
| MD58 | The locator answers through the surest chain of links, arm or rover, searched from the same end whichever way round it is asked | A chain and its reverse are not equally sure in position; asked both ways the two answers went through different chains and were not each other's inverse |
| MD59 | A sighting of the rover is carried forward by its wheels, loosening by 3 % of the distance and angle gone; a newer sighting is used unless an older one is at least a tenth surer | The rover moves; the cameras that saw it may not see it now |
| MD60 | The rover's lip in the scenes stays plain | A plain straight edge is the hard case for two cameras side by side (it gives depth only where it crosses the rows at an angle). Texturing it would flatter the simulation |
| MD61 | The map stays in the rover's frame as it stood at its first view; a later view is laid into it by the wheels' count | One frame for everything found. What the wheels add to the error is said with the view |
| MD62 | `GET /place?relative_to=X` gives every position in X's frame through the locator's chain, as a turn about the vertical and a shift along the surface; 409 when nothing relates them | The arm needs things in its own frame to pick them. Both stand on one surface, so heights are left as they are |
| MD63 | A touch stops when the tip has stopped following its command down (5 mm of command in 0.3 s with under 1.2 mm of motion), not on the gap between them and not on the stall guard | The gap is several millimetres with nothing there; the stall guard needs a joint 12° behind for 0.4 s, which is the arm's full push on the thing |
| MD64 | A touch reports the gripper's lowest point, from the arm's own shapes | The pads sit above the jaw's end. It reads about 4 mm high: stated as good to 5 mm |
| MD65 | Things are gathered by height as well as by touching (15 mm) | In the map the arm's hand hovering over a block, or a stack beside a cylinder, ran together into one thing |
| MD66 | An edge of the surface is taken out of the things; a straight side with the surface on both sides or neither stays in its thing | Taking every straight run out removed the faces of real things and split what was left |
| MD67 | A straight run is followed along its own line past the segment first found | A raised edge a few degrees off the grid is a staircase: a third of it was left over and reported as a thing |
| MD68 | Landmarks: two or more touches matched to map things by height (12 mm) and spacing, the closest fit taken only if no other way fits nearly as well | The map holds the arm itself in pieces of several heights; a fit through one of those was as acceptable as the right one until fits were ranked |
| MD69 | Fixed cameras add where each thing is in their pictures, not occupancy | Two fixed cameras could find raised things by where their ground-plane views disagree, but with lenses 2 % off the datasheet the views do not line up well enough to trust. The wrist camera at two arm poses would give real depth near the arm: not done |
| MD70 | One guard (`hub/guard.py`) reads the map for both devices: occupied is not entered, never seen is entered slowly, free is as usual | The plan's rule (5.9), in one place |
| MD71 | A drive that would reach something in the map goes as far as 20 mm short of it and reports that it stopped short; it is refused only if it cannot move at all | A shortened drive is still progress, and the answer says where it ended and why |
| MD72 | A drive is slow, all of it, if it crosses more than 12 cm2 of never-seen cells | A share of the cells crossed was tried first: 3 cm of never-seen strip at the end of a 13 cm drive was 1 % of them and went unnoticed, at full speed, into the edge |
| MD73 | The rover stops when its wheels work 1 % of full harder than rolling takes for 0.5 s, and takes back what they counted since | Against the edge with tyres slipping the servos read 0.19 instead of 0.15, far under the stall guard's 0.9; 0.20 m was counted that was not driven. Thresholds are the simulation's |
| MD74 | What stops the rover is put into the map, 12 mm high, just past its leading end | The edge behind it cannot be seen from where it stands. Having met it is finding it |
| MD75 | Where nothing has looked, the arm goes at 80 mm/s and stops when its tip stops following (12 mm of command in 0.3 s) | The stall guard meets things with the arm's full push. This does not feel a light loose thing, which slides along |
| MD76 | With the rover not placed relative to the arm, the hand goes carefully below the rover's height, rather than staying above it as the plan said | Staying above it, the arm could never touch a landmark, which is one of the ways the two get related |
| MD77 | A hand already inside the rover's margin, or at a thing, may always move away | Otherwise a move that ends near the rover, or a touch, could not be followed by any other |
| MD78 | The cells the rover's cameras mapped of the arm count as never seen for the hand, at the poses the arm was seen in, not the pose it is in now | The arm's own body in the map refused every move; masking round the present pose hid whatever stood just ahead of the hand |
| MD79 | The scene validator refuses a loose thing under the parked rover's head | The head reaches 0.25 m ahead and touches nothing below it, so the overlap check passed three scenes where the arm could not pick the thing or the rover could not leave without pushing it. The rover was moved in `open_small` and `moved_arm`, the cylinder in `shelf` |
| MD80 | Imperfections in a scene file: each camera's own lens (`seed`), `wheel_scale`, `tag_scale` | The devices the hub drives are not their descriptions. The wheels are built that size in the model; under 0.97 the simulated rover rests on its skid, so the range stops there |
| MD81 | The pair's sideways shift is what is left of a tag's disparity when the line through five camera spacings is carried back to no spacing | It needs no known size and no known distance, so a mis-scaled print cannot spoil it; a tag is small enough to be matched at any spacing, which the ground was not (MD52). Found to 0.5 px |
| MD82 | The ratio of the pair's two focal lengths comes from how the rows of one picture run against the other's, not from the tag's size in each | Seen from two places a tag is foreshortened differently: its size gave 0.90 and 1.07 where the truth was 1.00 |
| MD83 | A fixed camera's focal length alone is fitted (centre and distortion stay the datasheet's), and the result is weighed against the datasheet by how sure each is | Seven or eight sightings do not carry more. Applying only what differed significantly from the datasheet left a 0.9 % error in place; weighing uses the measurement for what it is worth |
| MD84 | The wheels are calibrated only over ground the map shows free, by a camera that watches, and weighed against the description (3 %). A camera places the rover to 4 mm at each end, and driving there and back again does not average that away | A leg into unseen ground ended against the edge with the tyres slipping, and made wheels that were 5 % large look right. Four 0.12 m legs between the same two places agreed to 0.1 % and were all 4 % short: the uncertainty first stated (1.1 %) counted them as four measurements. A 0.25 m leg tells the wheels' size to about 2 %; a 0.12 m one to 5 %, which changes little |
| MD85 | Not calibrated: lens centres and distortion, servo zeros, the arm's link lengths | Each needs more than a handful of tag sightings; the stated uncertainties cover them |
| MD86 | `GET /where/pixel` turns a point in a camera's picture and a height into a place in the arm's or the rover's frame | The hub knows where the cameras are; whoever looks at the picture knows what is in it. This is where the two meet. It keeps recognition out of the hub |
| MD87 | Reference solutions get the hub's API and nothing else, find things by colour in pictures, and are scored by the referee; a solution that cannot finish gives up with its reason and hands in no guess | A solution that asked the simulation where things are would prove nothing about the hub. A test checks what they import |
| MD88 | Picking by sight goes coarse then fine: a fixed camera for roughly where, the wrist camera hovering above for exactly where, a touch for how high | A fixed camera places a thing to a centimetre or two and needs its height; the wrist camera's own place comes from the arm's joints, good to millimetres |
| MD89 | The rover's head is checked against the arm's place and the map before it moves, in three dimensions; what it has swept since the last drive it may always cross again | The survey turned the head into the arm's raised hand and into a shelf it had already mapped. Checking in plan only refused turns that passed under the hand; without the way back, a head that had turned past the arm could not be stowed |
| MD90 | A map thing carries the height of its lowest seen part | A shelf's plain top gives two cameras nothing to match, so the shelf may not be in the map at all; what stands on it is, and its lowest part is the shelf's top. That answered the shelf task to the millimetre |
| MD91 | `open_small` is 0.8 by 0.6 m, with the rover parked in a corner facing along the surface | At 0.7 by 0.55 the rover's head overhung the edge wherever it stood clear of the arm and the two things, and its survey saw almost nothing |
| MD92 | Thirteen of eighteen tasks have a reference solution; twelve pass. `model_run.py` is written and has not been run | The other five need the rover driven round something to look, or a judgement from a picture, or (the bench's tidy) every thing found by sight first. Each has its reason in `solve.py --list`. A model run is billed to the user's account, so none was made unasked |
| MD93 | `sim_api.py` is kept | The bench's eight-step tidy still runs on it, and porting it needs every thing found by sight first |
| MD94 | The rover's board passes the servo bus's bytes through unread; the hub speaks the servos' own protocol to a port that happens to be far away | The hub's rover driver already speaks it, to a wire or a radio alike. A protocol of the board's own would be one more thing to get wrong and to keep in step |
| MD95 | Everything the board decides is in one plain C file with no operating system under it (`bridge.c`), compiled for the PC and run in the simulation in place of the simulation's model of the board | As with the camera firmware (X13): the code that will run is the code that is tested. The part that needs the board is thin and is not built yet |
| MD96 | The board stops the wheels with one broadcast SYNC_WRITE of zero speed: at start-up, when the hub has been silent 0.4 s, and again every second while the silence lasts | A broadcast draws no reply, so nothing unexpected reaches the hub when it returns. At start-up because the board cannot know what the wheels were last told; repeated because one packet can be lost. The servos' own replies do not count as word from the hub |
| MD97 | The firmware's time is an unsigned millisecond counter compared by subtraction | It wraps after 49 days; tested with the wrap before, inside and after a silence. (The first wiring into the simulation stamped a word with its future arrival time, which the same arithmetic read as a very long silence: the watchdog tripped ninety times in two seconds. Words are now given to the firmware when they arrive.) |
| MD98 | Real ports are found, not listed: every USB serial adapter and video device the system shows is offered to the hub, which finds out what answers. A file names only what cannot be found by looking (a rover's board on the network; which cameras are a rover's own) | The same rule as in the simulation: the hub is given ports and nothing else. A list of devices would be one more thing to keep true |
| MD99 | A port that is not there today is a note, and the hub starts with the rest; a link that has gone is an error at once and stays one until the next discovery | A missing camera should not stop an arm from being used; a dead link read as "slow" would have the hub wait on it for ever |
| MD100 | A bin is placed from close up only when its whole frame is in the wrist camera's picture; otherwise the fixed camera's place for it is kept | The middle of the part of a frame that is in the picture is not the frame's middle: one bin was put 27 mm off, and a cylinder was set down on its wall. (The run that had passed before did so by carrying again what had rolled out.) |
| MD101 | A patch of colour that the wrist camera does not find from above, or that the hand cannot hover over, is passed over and the next patch tried; giving up comes only when none was a thing, and says how many were looked at | The arm is printed yellow: asked for the yellow peg, the fixed camera's largest yellow patches are the arm. `fetch_hidden` had been written off as needing the rover driven round a wall; looking showed the fixed camera sees the peg, and this was all that stood in the way |
| MD102 | `GET /where/point` is `/where/pixel` the other way round: where a place is in a camera's picture, or that it is outside it. A place behind the camera, or one whose pixel does not come back to the same direction, is outside | "What can that camera not see?" is a question about the camera's picture, and only the hub knows the camera's lens and where it stands. Without the two checks a place behind a camera mirrors into the middle of its picture, and a lens model carried past its picture's edge can fold a far-off place back in |
| MD103 | What is out of the fixed camera's sight is found by the rover turning its head, not by driving: patches of colour in its picture, each put on the surface where its foot is | In `l_shaped` the rover cannot go anywhere: its outline reaches 0.26 m from its axle and the leg is 0.4 m wide, so it cannot turn, and a cube stands 10 mm ahead of it. The task had been written off as needing a path planned round the corner. Its cameras see the far leg from where it stands; the map does not reach the furthest thing (0.84 m; the map stops at 0.7), but a picture does |
| MD104 | The area of the L is not answered: the solution says the surface is not a plain rectangle and hands in nothing | From the one place the rover can stand, 0.55 of the surface's 0.625 m2 is seen, and neither leg's far end. The seen area is 12 % under, outside the task's 10 %; handing it in would be a guess that fails |
| MD105 | Whether the drawer is out is judged from the wrist camera alone, looking down from seven places round the arm: white beside grey, wider than 30 mm | The rover's map was tried first, to find the cabinet: it stands 0.45 m from the rover, past what the 60 mm spacing ranges, and was in the map in one run and not in the next. The wrist camera needs nothing but the arm |
| MD106 | What is on a printed tag is not a white thing: the solution asks the hub where the tags are in the same picture | The arm's tag blocks are white round their tags and grey on their shaded sides, which is exactly "white beside grey": a shut drawer was answered "yes". Telling a tag by the black inside a white frame fails when part of the frame is in shadow |
