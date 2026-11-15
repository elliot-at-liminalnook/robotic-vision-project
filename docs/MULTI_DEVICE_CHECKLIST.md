# Multi-device checklist

Every tangible deliverable of [the multi-device plan](MULTI_DEVICE_PLAN.md), one box each. The plan has
the design and the reasons; this file only tracks what exists. Tick a box when the thing is in the repo.

Started 2026-11-14.

Tests are written with each step and run only when the user asks. Step 2's small suite is already asked for.

## Step 1. Scenes as files

Plan: [steps](MULTI_DEVICE_PLAN.md#6-steps-in-order), [5.7 Scenes and tasks are config files](MULTI_DEVICE_PLAN.md#57-scenes-and-tasks-are-config-files)

- [x] Scene file format (TOML), written down
- [x] Loader that builds the simulation model from a scene file at run time
- [x] A lip on every scene
- [x] Validator: fields and ranges; the model compiles; nothing overlaps; everything inside the lip; nothing moves when left alone
- [x] `workcell` converted to a scene file
- [x] `open_small` scene
- [x] Tests written (`sim/tests/test_scenes.py`)

## Step 2. Environments, tasks, operator API

Plan: [5.7](MULTI_DEVICE_PLAN.md#57-scenes-and-tasks-are-config-files), [5.8 Two APIs](MULTI_DEVICE_PLAN.md#58-two-apis)

Scenes:

- [x] `bench` converted to a scene file
- [x] `l_shaped`
- [x] `shelf`
- [x] `cluttered`
- [x] `no_arm`
- [x] `moved_arm`

Tasks, as data:

- [x] Task format, with the referee's checks (inside a region, on top of, upright, fixture open or closed, reported number within tolerance)
- [x] Where is everyone
- [x] How big is this place
- [x] What is on the far leg
- [x] What is on the shelf, and how high is it
- [x] Fetch the hidden part
- [x] Is the drawer open
- [x] Tidy by colour
- [x] Carried to another table (`moved_arm.toml`, task `carried`, swaps to `open_small`)
- [x] The existing bench task moved from `tasks.py` to a task file (`bench.toml`, task `tidy`; `tasks.py` stays until the old API is retired)

Operator API:

- [x] List, read, create, change and delete scene files over REST
- [x] Validate a scene over REST; an invalid edit is refused with its reason
- [x] Load a scene, swap scenes and reset while the world runs
- [x] Referee endpoints: true state, task goals, handing in an answer

Checks:

- [x] Scene validation suite written (`sim/tests/test_scenes.py`, `sim/tests/test_operator_api.py`)
- [x] Scene validation suite run, every scene and task passing (134 passed, 2026-11-14)

## Step 3. Markers

Plan: [5.4 Finding where the devices are](MULTI_DEVICE_PLAN.md#54-finding-where-the-devices-are), [4. Probe](MULTI_DEVICE_PLAN.md#4-probe-do-printed-markers-work-in-these-pictures)

- [x] OpenCV added to the requirements
- [x] Tag files that can be printed, the same files the simulation shows
- [x] Tags on the arm's base (a pair) and on the wrist camera holder, in the model
- [x] Tag detection in full-size stills
- [x] Each fixed camera located relative to the arm from pictures alone
- [x] Tests written (mirrored or rotated tag, flipped reading, no scene numbers in the code): `sim/tests/test_markers.py`, not run

## Step 4. Lens

Plan: [5.10 Keeping the gap small](MULTI_DEVICE_PLAN.md#510-keeping-the-gap-small)

- [x] Lens distortion on every camera's picture
- [x] Corner darkening, off-centre blur, sensor noise
- [x] A slightly different lens per simulated camera
- [x] Recorded: how much worse step 3's accuracy is through the lens (median 8 → 18 mm, worst 21 → 37 mm at 1080p; `hub/README.md`, DECISIONS MD22)
- [x] Tests written (`sim/tests/test_lens.py`, not run)

## Step 5. Hub

Plan: [5.1](MULTI_DEVICE_PLAN.md#51-four-parts-two-apis-and-one-rule-about-what-may-cross), [5.2 Devices as a logical network of interests](MULTI_DEVICE_PLAN.md#52-devices-as-a-logical-network-of-interests), [5.3 The knowledge rule](MULTI_DEVICE_PLAN.md#53-the-knowledge-rule-nothing-about-the-surroundings), [5.8](MULTI_DEVICE_PLAN.md#58-two-apis)

- [x] Top-level `hub/` package, started from ports alone, importing nothing from `sim/`
- [x] Discovery of devices from the ports
- [x] The tree of interests: devices and their parts, the place, groups
- [x] Arm and fixed cameras as devices
- [x] The model's REST API, with no ground truth and no description of the place
- [x] A pick through the hub's API in several scenes (workcell, open_small, moved_arm, bench, cluttered: smoke-checked)
- [x] Tests written (import rule, new places, blindfold, a moved camera is followed): `sim/tests/test_hub.py`, not run as a suite
- [ ] Old `sim_api.py` retired. **Kept, for one reason now:** its window, in which a person can watch and drive the arm by hand between a model's requests (`--window`, `--hands-on`); the hub's system has no window. The bench's eight-step tidy, the other reason it was kept, now runs through the hub's API (`sim/solutions/bench.py`). Retiring it means giving that window up or building one on the hub: a choice, not a chore (DECISIONS MD93, MD113)

## Step 6. Locator

Plan: [5.4](MULTI_DEVICE_PLAN.md#54-finding-where-the-devices-are)

- [x] Pose estimate between devices from all links, with uncertainty, age and support
- [x] "Not known" when no chain of links connects two devices
- [x] The arm contributing: wrist camera, wrist tag shown to cameras
- [x] `GET /where`, and `POST /locate` (fresh pictures; `show_wrist_tag` has the arm show its wrist tag at six poses)
- [x] Tests written (error within stated bounds, honest uncertainty): `sim/tests/test_locator.py`, not run as a suite

## Step 7. Rover designed

Plan: [5.6 The rover](MULTI_DEVICE_PLAN.md#56-the-rover)

- [x] CAD scripts under `hardware/rover/`: chassis, mast and lift, head with slide and racks, wheels
- [x] Printable parts as STEP and STL
- [x] CAD checks: one solid each, nothing overlapping through full travel, wires long enough
- [x] Simulation model generated from the CAD: masses and inertia from the meshes, a joint per axis
- [x] Contacts: tyres, skid, lip, head against chassis, camera carriages and end stops
- [x] Rover section in `docs/BOM.md`: real parts with links and prices, including head servos, controller board, 5 V regulator, fuse, charger (prices read 2026-11-14; DigiKey rows and the fuse unconfirmed; nothing ordered)
- [x] Tests written (stands, rolls, steers; head axes reach their ranges): `sim/tests/test_rover_model.py`, not run as a suite

## Step 8. Rover controlled

Plan: [5.6](MULTI_DEVICE_PLAN.md#56-the-rover), [5.9 Safety when nothing is known](MULTI_DEVICE_PLAN.md#59-safety-when-nothing-is-known)

- [x] Wheel mode (operating mode 1) in the servo emulator
- [x] Radio-link port: delay, jitter, losses
- [x] Battery model: voltage falling with charge, read from the servos
- [x] Wheel driver: drive, turn, odometry
- [x] Head driver: pan, tilt, lift, spacing
- [x] Command filters for wheels and head
- [x] Stop path for the rover: on `stop`, on a lost link, on low battery
- [x] Tests written (`sim/tests/test_rover.py`, not run as a suite)

## Step 9. Start-up survey

Plan: [5.5 Finding out what the place is](MULTI_DEVICE_PLAN.md#55-finding-out-what-the-place-is), [5.3](MULTI_DEVICE_PLAN.md#53-the-knowledge-rule-nothing-about-the-surroundings)

- [x] Survey routine: pan sweeps at several tilts, spacings and heights, wheels still (`hub/survey.py`; `POST /survey`, or at start with `--survey`)
- [x] Size from the head's own motion, with the other rulers checked against it (tags' printed size, the lift's height; the wheels are not checked: they do not move)
- [x] The rover in the locator: its cameras, its tags, its odometry
- [x] A first map with sizes and the devices in view, from a standing start (`GET /place/state`, `/place/picture`; distances good to about 5 % until the pair is calibrated in step 12)
- [x] Tests written (`sim/tests/test_survey.py`, not run as a suite; each test called directly once, passing)

Found on the way, and dealt with: the head's blind swing spinning the rover (pan torque cut to 5 %, the survey checks at the end that it stood still); pictures taken while the head was still swinging.
Left open: a gyro for the rover (DECISIONS MD56); the survey does not see behind the rover.

## Step 10. Map

Plan: [5.5](MULTI_DEVICE_PLAN.md#55-finding-out-what-the-place-is)

- [x] Map of the place from every device: surface and lip, things, never-seen space (rover: depth from any standpoint, `head/look`; arm: touch; fixed cameras: where each thing is in their pictures. The wrist camera adds no depth yet, and fixed cameras say nothing about cells the rover has not seen: DECISIONS MD69)
- [x] Touch: the arm as a probe (`POST /devices/arm/probe`; it stops on the tip no longer following, well before the stall guard would)
- [x] Landmarks as links between devices that cannot see each other (`hub/landmarks.py`)
- [x] `GET /place`, and `GET /place?relative_to=<device>`
- [x] Tests written (`sim/tests/test_map.py`, not run as a suite; each test called directly once, passing)

## Step 11. Safety

Plan: [5.9](MULTI_DEVICE_PLAN.md#59-safety-when-nothing-is-known)

- [x] Stop at the root freezes every device; every exit switches every servo's torque off
- [x] Rover filter working from the map: stays inside the lip, slow into unseen space, no driving with the head up (and stops when it is pushing, not rolling)
- [x] Arm filter working from the map: out of occupied space, gentle into unseen space (gentle: 80 mm/s and stopped at the first touch; a light loose thing is not felt)
- [x] Arm kept out of the rover's estimated space (unplaced rover: careful below its height, not kept above it: DECISIONS MD76)
- [x] Tests written (fuzzed runs): `sim/tests/test_safety.py`, not run as a suite; each test called directly

Found on the way: three scenes had a loose thing under the parked rover's head; two pick tests had been failing since the rover got its shape (never run). Fixed, and the validator now refuses it.

## Step 12. Self-calibration

Plan: [5.10](MULTI_DEVICE_PLAN.md#510-keeping-the-gap-small)

- [x] Simulated devices that differ from their descriptions, set in the scene file (`[imperfections]`: each camera's lens, `wheel_scale`, `tag_scale`. Not simulated: servo zeros, play in the head's slide)
- [x] Routines that find and correct the differences using the devices' own tags (`POST /calibrate`: wheel size, the rover's camera pair; the tags' print scale is measured and reported, not corrected)
- [x] Lens values corrected from what is seen (each fixed camera's focal length; the pair's shift and focal ratio. Centres and distortion stay the datasheet's)
- [x] Tests written (`sim/tests/test_calibrate.py`, not run as a suite; each test called directly)

## Step 13. Tasks run

Plan: [5.7](MULTI_DEVICE_PLAN.md#57-scenes-and-tasks-are-config-files)

- [ ] A reference solution for each task, through the hub's API. **All 18 have one** (`sim/solutions/`, `python sim/scripts/solve.py`); 17 pass the referee. The other is the area of the L-shaped surface (`l_shaped` / `how_big`): neither leg's far end can be seen from the one place the rover can stand, nor by the fixed camera, and the solution says so and hands in nothing rather than a guess. Left unticked for that one
- [ ] Model runs, scored by the referee. **Not run:** the harness is written (`sim/scripts/model_run.py`: the task, the hub's guide and three tools to a model; the referee scores). It needs `pip install anthropic`, credentials, and is billed per run
- [x] A point in a picture as a place (`GET /where/pixel`), which the solutions and a model both need, and a place as a point in a picture (`GET /where/point`: can that camera see there?)
- [x] Tests written (`sim/tests/test_solutions.py`, not run as a suite; each test called directly)

Found on the way: the survey swung the rover's head into the arm and into a shelf it had mapped (the head is now checked against both); `open_small` was too small for the rover to survey from anywhere.

## Step 14. Hardware

Plan: [5.6](MULTI_DEVICE_PLAN.md#56-the-rover), [8. Risks and unknowns](MULTI_DEVICE_PLAN.md#8-risks-and-unknowns)

None of these can be ticked without the hardware. What they need from the software is in the repo, untried on it:
real ports and a launcher (`hub/real.py`, `python scripts/run_hub.py`), the rover board's firmware
(`firmware/rover-bridge`), printable tags (`hardware/tags/print/`), the rover's parts (`hardware/rover/parts/`, `docs/BOM.md`).

- [ ] Arm on its real port
- [ ] Fixed cameras on their real ports
- [ ] One real camera calibrated, and its numbers in the lens model
- [ ] Rover printed and assembled
- [ ] Lip on the real bench
- [ ] Rover on its real ports

## Has no step yet

- [x] Rover firmware: drives the servos from a board and carries the bus over Wi-Fi; stops the wheels by itself when the link goes quiet. Needed by step 14 ([5.6](MULTI_DEVICE_PLAN.md#56-the-rover)). In `firmware/rover-bridge/`. What the board decides (`bridge.c`) is compiled for the PC, tested (`sim/tests/test_rover_fw.py`, 23 cases called directly) and runs in the simulation (`run_system.py --rover-fw`). **The part that needs the board (Wi-Fi, socket, UART) is written and has not been built or flashed:** there is no ESP-IDF on this machine; its README lists what to check first

## Open questions

Tracked in the plan: [7. Decisions](MULTI_DEVICE_PLAN.md#7-decisions).
