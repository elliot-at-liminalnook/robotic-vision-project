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

- [ ] `bench` converted to a scene file
- [ ] `l_shaped`
- [ ] `shelf`
- [ ] `cluttered`
- [ ] `no_arm`
- [ ] `moved_arm`

Tasks, as data:

- [ ] Task format, with the referee's checks (inside a region, on top of, upright, fixture open or closed, reported number within tolerance)
- [ ] Where is everyone
- [ ] How big is this place
- [ ] What is on the far leg
- [ ] What is on the shelf, and how high is it
- [ ] Fetch the hidden part
- [ ] Is the drawer open
- [ ] Tidy by colour
- [ ] Carried to another table
- [ ] The existing bench task moved from `tasks.py` to a task file

Operator API:

- [ ] List, read, create, change and delete scene files over REST
- [ ] Validate a scene over REST; an invalid edit is refused with its reason
- [ ] Load a scene, swap scenes and reset while the world runs
- [ ] Referee endpoints: true state, task goals, handing in an answer

Checks:

- [ ] Scene validation suite written
- [ ] Scene validation suite run, every scene and task passing

## Step 3. Markers

Plan: [5.4 Finding where the devices are](MULTI_DEVICE_PLAN.md#54-finding-where-the-devices-are), [4. Probe](MULTI_DEVICE_PLAN.md#4-probe-do-printed-markers-work-in-these-pictures)

- [ ] OpenCV added to the requirements
- [ ] Tag files that can be printed, the same files the simulation shows
- [ ] Tags on the arm's base (a pair) and on the wrist camera holder, in the model
- [ ] Tag detection in full-size stills
- [ ] Each fixed camera located relative to the arm from pictures alone
- [ ] Tests written (mirrored or rotated tag, flipped reading, no scene numbers in the code)

## Step 4. Lens

Plan: [5.10 Keeping the gap small](MULTI_DEVICE_PLAN.md#510-keeping-the-gap-small)

- [ ] Lens distortion on every camera's picture
- [ ] Corner darkening, off-centre blur, sensor noise
- [ ] A slightly different lens per simulated camera
- [ ] Recorded: how much worse step 3's accuracy is through the lens
- [ ] Tests written

## Step 5. Hub

Plan: [5.1](MULTI_DEVICE_PLAN.md#51-four-parts-two-apis-and-one-rule-about-what-may-cross), [5.2 Devices as a logical network of interests](MULTI_DEVICE_PLAN.md#52-devices-as-a-logical-network-of-interests), [5.3 The knowledge rule](MULTI_DEVICE_PLAN.md#53-the-knowledge-rule-nothing-about-the-surroundings), [5.8](MULTI_DEVICE_PLAN.md#58-two-apis)

- [ ] Top-level `hub/` package, started from ports alone, importing nothing from `sim/`
- [ ] Discovery of devices from the ports
- [ ] The tree of interests: devices and their parts, the place, groups
- [ ] Arm and fixed cameras as devices
- [ ] The model's REST API, with no ground truth and no description of the place
- [ ] A pick through the hub's API in several scenes
- [ ] Tests written (import rule, new places, blindfold, a moved camera is followed)
- [ ] Old `sim_api.py` retired

## Step 6. Locator

Plan: [5.4](MULTI_DEVICE_PLAN.md#54-finding-where-the-devices-are)

- [ ] Pose estimate between devices from all links, with uncertainty, age and support
- [ ] "Not known" when no chain of links connects two devices
- [ ] The arm contributing: wrist camera, wrist tag shown to cameras
- [ ] `GET /where`
- [ ] Tests written (error within stated bounds, honest uncertainty)

## Step 7. Rover designed

Plan: [5.6 The rover](MULTI_DEVICE_PLAN.md#56-the-rover)

- [ ] CAD scripts under `hardware/rover/`: chassis, mast and lift, head with slide and racks, wheels
- [ ] Printable parts as STEP and STL
- [ ] CAD checks: one solid each, nothing overlapping through full travel, wires long enough
- [ ] Simulation model generated from the CAD: masses and inertia from the meshes, a joint per axis
- [ ] Contacts: tyres, skid, lip, head against chassis, camera carriages and end stops
- [ ] Rover section in `docs/BOM.md`: real parts with links and prices, including head servos, controller board, 5 V regulator, fuse, charger
- [ ] Tests written (stands, rolls, steers; head axes reach their ranges)

## Step 8. Rover controlled

Plan: [5.6](MULTI_DEVICE_PLAN.md#56-the-rover), [5.9 Safety when nothing is known](MULTI_DEVICE_PLAN.md#59-safety-when-nothing-is-known)

- [ ] Wheel mode (operating mode 1) in the servo emulator
- [ ] Radio-link port: delay, jitter, losses
- [ ] Battery model: voltage falling with charge, read from the servos
- [ ] Wheel driver: drive, turn, odometry
- [ ] Head driver: pan, tilt, lift, spacing
- [ ] Command filters for wheels and head
- [ ] Stop path for the rover: on `stop`, on a lost link, on low battery
- [ ] Tests written

## Step 9. Start-up survey

Plan: [5.5 Finding out what the place is](MULTI_DEVICE_PLAN.md#55-finding-out-what-the-place-is), [5.3](MULTI_DEVICE_PLAN.md#53-the-knowledge-rule-nothing-about-the-surroundings)

- [ ] Survey routine: pan sweeps at several tilts, spacings and heights, wheels still
- [ ] Size from the head's own motion, with the other rulers checked against it
- [ ] The rover in the locator: its cameras, its tags, its odometry
- [ ] A first map with sizes and the devices in view, from a standing start
- [ ] Tests written

## Step 10. Map

Plan: [5.5](MULTI_DEVICE_PLAN.md#55-finding-out-what-the-place-is)

- [ ] Map of the place from every device: surface and lip, things, never-seen space
- [ ] Touch: the arm's stall guard as a probe
- [ ] Landmarks as links between devices that cannot see each other
- [ ] `GET /place`
- [ ] Tests written

## Step 11. Safety

Plan: [5.9](MULTI_DEVICE_PLAN.md#59-safety-when-nothing-is-known)

- [ ] Stop at the root freezes every device; every exit switches every servo's torque off
- [ ] Rover filter working from the map: stays inside the lip, slow into unseen space, no driving with the head up
- [ ] Arm filter working from the map: out of occupied space, gentle into unseen space
- [ ] Arm kept out of the rover's estimated space
- [ ] Tests written (fuzzed runs)

## Step 12. Self-calibration

Plan: [5.10](MULTI_DEVICE_PLAN.md#510-keeping-the-gap-small)

- [ ] Simulated devices that differ from their descriptions, set in the scene file
- [ ] Routines that find and correct the differences using the devices' own tags
- [ ] Lens values corrected from what is seen
- [ ] Tests written

## Step 13. Tasks run

Plan: [5.7](MULTI_DEVICE_PLAN.md#57-scenes-and-tasks-are-config-files)

- [ ] A reference solution for each task, through the hub's API
- [ ] Model runs, scored by the referee

## Step 14. Hardware

Plan: [5.6](MULTI_DEVICE_PLAN.md#56-the-rover), [8. Risks and unknowns](MULTI_DEVICE_PLAN.md#8-risks-and-unknowns)

- [ ] Arm on its real port
- [ ] Fixed cameras on their real ports
- [ ] One real camera calibrated, and its numbers in the lens model
- [ ] Rover printed and assembled
- [ ] Lip on the real bench
- [ ] Rover on its real ports

## Has no step yet

- [ ] Rover firmware: drives the servos from a board and carries the bus over Wi-Fi; stops the wheels by itself when the link goes quiet. Needed by step 14 ([5.6](MULTI_DEVICE_PLAN.md#56-the-rover))

## Open questions

Tracked in the plan: [7. Decisions](MULTI_DEVICE_PLAN.md#7-decisions).
