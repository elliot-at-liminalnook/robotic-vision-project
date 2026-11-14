# Multi-device plan: several robots, one language model

Written 2026-11-14. **Planning only: nothing here is built.** Decisions are marked D1 to D13 and listed in
section 7; once made, they belong in `DECISIONS.md` and the steps in `PLAN.md`.

Revised the same day: **the system is given no prior information about its surroundings** (sections 1,
5.3 to 5.5, 5.8, 5.9 and the steps). The first version had a set-up file filled in by a person; that is gone.
Then: **size comes from the rover driving a known distance between pictures** (5.3, 5.6, D13).

## 1. The goal

Turn the single-arm simulation into an environment with several devices commanded by one language model.

- The model is the only decision-maker. Devices are peripherals it commands and queries, like nodes on a bus.
- A "device" is any cluster of sensors and actuators with positions: the arm, a fixed camera, and first of
  the new ones, a very small wheeled rover with a stereo camera pair that drives on the table and only looks.
- **No prior information.** The system is told nothing about where it is: not the table's size or height,
  not what is on it, not where any device stands, not how many devices there are. It finds out. The aim
  is a system that can be put down somewhere new and work the place out.
- **Devices do not know where they are relative to each other.** Those positions are estimated from what
  the sensors see, with every device contributing. Printed markers on the devices to start with.
- **Smallest possible gap to the real thing.** The model's API gives only what the real system has worked
  out from its own sensors. Never the simulator's true state, and no description of the scene written by
  a person.
- Simulation only for now, but the rover will be built, from parts the project already uses.

## 2. The repo as it stands

### What this can be built on

| Piece | Where | Why it matters here |
|---|---|---|
| Controller-side bus code that knows nothing about the simulation | `so101sim/bus/driver.py`, `arm.py` | Already the shape we want: it talks to a port (bytes), and the port is either the simulated wire or a serial port |
| Byte-level servo bus with emulated HX-30HM servos | `so101sim/bus_backend.py`, `bus/servo_emu.py`, `bus/wire.py` | More servos on the bus (the rover's wheels) is mostly more entries in a list |
| Camera firmware compiled for the PC, fed by the simulation | `so101sim/camera_fw.py`, `firmware/p4-uvc-camera/host/` | A camera as the PC really sees it: frames with delay, drops and start-up time. Today for the wrist camera only |
| Devices on the physics clock | `MujocoBackend.step_hooks` | The hook the camera firmware uses; other devices can tick on it |
| One table that produces the guide, the OpenAPI document and request validation | `sim_api.py` `ROUTES` | The same trick works per device: the guide is generated from device descriptions |
| Command filter, physics monitor, stop path | `safety.py`, `monitor.py`, `Teleop.freeze` / `set_torque` / `shutdown` | The arm's protections stay as they are. The stop path needs widening to every device (section 5.8) |
| The stall guard | `Teleop._blocked` | It already notices when the arm meets something. That is a touch sensor (section 5.5) |
| Wires as part of the model | `so101sim/cables.py` | A tethered rover's cable can be simulated the same way |
| Scenes and tasks with a reference solution | `scene/`, `so101sim/tasks.py`, `tests/test_tasks.py` | The pattern for "feasible is proved, not argued" carries over to multi-device tasks |

### What is in the way

| Assumption | Where | What it costs |
|---|---|---|
| One arm, six joints, everywhere | `JOINTS` is used in 15 files; `MujocoBackend.act` / `qadr`; `safety.py`; `servos.toml` has one `[follower]` | A second kind of device needs its own joint set, limits and filter. The arm's code can stay as it is if the rover gets its own, parallel path |
| `Teleop` is the arm's controller, the window and the thing that steps the world | `scripts/teleop.py` | `SimApi` drives it headless already. Other devices need to tick in the same loop |
| The API is the arm's API | `sim_api.py` (`/move_tip`, `/state`, ...) | Endpoints need a device in their address |
| **The scene is written into the code** | The guide's text (three named cameras and where they stand, the objects and their sizes); `safety.toml` (`table_z_m`); `/state` `table_m`; `MIN_TIP_Z`; `HOME_TIP` | Each is a fact about one particular table. All of it has to be found by sensing or dropped |
| The servo emulator has position mode only | `servo_emu.py` logs "operating mode N is not modelled; output is off" | Wheels need mode 1, constant speed |
| Fixed cameras are placed by the config, and the same config is what the controller reads | `config/cameras.toml`, `Teleop.click_table` | The controller's "calibration" is the simulator's truth, exactly |
| Newton's renderer ignores textures | comment in `scene/workcell.xml` | Markers as textures are MuJoCo only (D9) |
| No vision library | `sim/requirements.txt` | Marker detection needs one new dependency (D5) |

Two housekeeping facts: the working tree has 131 uncommitted entries, and the full test suite has not
been run since the stop path was added. Both should be settled before step 1.

## 3. Where prior information reaches the model today

An audit of `sim_api.py` against the rule in section 1. Two kinds of leak: the simulator's truth, and
facts about this scene that a person wrote down.

| What the model gets | Source | Plan |
|---|---|---|
| `objects`, `fixtures` in `/state` (on by default) | True state | Gone. Truth goes to the referee (5.1) |
| `GET /task` "which goals hold now" | True state | The model gets the request in words only. Goal checking is the referee's |
| `POST /randomize` answer with positions | True state | Becomes "a new place from a seed", an operator's action that answers with nothing about the place |
| `joints_rad`, `tip_m` | True joint angles unless `--bus` | The hub always reads through the bus (ticks, delay, lost replies) |
| Camera pictures | MuJoCo render: perfect pinhole, no delay, no noise | Every camera goes through a camera port with delay and drops; lens error added later (5.9) |
| Camera lens values used by the controller | `cameras.toml`, which also builds the sim | Each kind of camera starts from its lens's nominal values and corrects them from what it sees (5.9) |
| Fixed cameras' poses (`click_table`) and the guide's sentences on where each camera stands | `cameras.toml`; written by hand | Estimated from sensing. The sentences go |
| Which devices and cameras exist | Written into the guide | Discovered at start-up (5.2) |
| The list of objects with sizes, colours and weights | Model geoms, via the guide | Gone. What is on the table is for the system to find |
| `table_m`: the table's extent | Model geom | Gone. The surface and its edges are found by looking (5.5) |
| The table's height in the arm's frame | `safety.toml`, `MIN_TIP_Z` | A cautious starting assumption only, replaced by what is sensed (5.8) |
| `checks.physics_violations` | Physics monitor | Referee only. A run with a violation is still a failed run |
| `checks.command_filter`, `limited`, `why_not_reached` | The controller's own filter and encoders | Keep |
| Simulated time, turn-based | The sim clock | Keep (D7) |

## 4. Probe: do printed markers work in these pictures?

Before planning around markers I checked them in a throwaway script outside the repo: the existing
`sim/model/workcell.xml` loaded read-only, AprilTag 36h11 tags added in memory as textured thin boxes,
rendered by MuJoCo 3.10, detected with OpenCV 5.0 (`cv2.aruco`), pose from the four corners. Truth was
used only to score.

| Camera and size | Result |
|---|---|
| `scene_front`, `scene_side` at 640×360 (today's size) | Mostly fails: a 40 mm tag at 0.5 m is 24 to 27 px wide and is found; smaller or farther tags are not |
| Same at 1280×720 | 20 to 40 mm tags found at 0.46 to 0.71 m; position error 1.4 to 9 mm where the pose is right |
| Same at 1920×1080 | Position error 1 to 10 mm at 0.46 to 0.83 m; tilt error 0.4 to 2.2° |
| A camera 55 mm above the table (rover height), looking level to 30° down | 30 mm tags flat on the table: 0.6 to 3 mm error out to 0.21 m; at 0.30 m only when tilted 30° down or at 1080p. An upright tag at 0.21 m: about 1 mm |

What it settles and what it does not:

- **Markers work in this simulation**, with one new dependency that installs on the Intel Mac (Python 3.13).
- **640×360 is too small for localisation.** Locating should use full-size stills; previews can stay small.
- **A single small tag can be read flipped.** At 720p two tags (19 and 26 px wide) came out tilted about
  90° wrong, with the right answer as the solver's second candidate. Known behaviour of flat markers. The
  fix is to never trust one small tag alone: several tags on one body, several views, or both.
- **Lighting matters.** Upright tags in the scene's shadow side were not found beyond 0.2 m.
- Not settled: these pictures have no blur, noise, lens distortion or JPEG artefacts, so real numbers
  will be worse. And the probe did not pin down which way round a texture sits on a box face; step 1's
  test must check for a mirrored or rotated tag.
- The probe put tags on the table. The plan no longer does (5.4): tags sit on devices only.

## 5. Recommended design

### 5.1 Four parts, and one rule about what may cross between them

```
                 the model
                     │  HTTP, JSON (section 5.7)
      ┌──────────────┴───────────────┐
      │            HUB               │   runs unchanged on the real machine
      │  devices it has discovered   │   starts knowing only what each kind of
      │  locator, map of the place   │   device is; everything else is sensed
      │  safety, stop path           │
      └───┬──────────┬───────────┬───┘
   serial ports   camera ports  clock            ← PORTS: the only way in or out
      ┌───┴──────────┴───────────┴───┐
      │           WORLD              │   simulation only (MuJoCo)
      │  physics, emulated servos,   │   a different place every run;
      │  camera firmware, wires      │   tells the hub nothing
      └──────────────┬───────────────┘
                     │ true state
                 REFEREE                         tests and scoring only; no route to the model
```

- **World.** One MuJoCo model with every device in it, one clock. It offers ports and nothing else.
- **Ports.** A serial port per bus adapter (bytes), a camera port per camera board (frames with a time
  stamp), a clock. On the real machine these are pyserial, V4L2 and the wall clock. Both kinds of port
  exist in the repo today.
- **Hub.** Everything that would run on the real PC: device drivers, the locator, the map, safety, the
  API. It is started with ports and nothing else. There is no set-up file.
- **Referee.** Reads the world's true state for tests, task goals and scoring. It is not reachable from
  the API.

Two speeds of port, same hub: byte-level (bus emulation, camera firmware) for checking the real code
paths, and a quick kind (encoder ticks and delayed pictures without the byte traffic) for long runs.

### 5.2 A device, and how the hub comes to know of it

**Discovery.** At start-up the hub scans its ports: which servo ids answer on each bus, which cameras
are plugged in. It matches what it finds to the kinds of device it has drivers for (six servos that
look like an arm; two wheel servos and two cameras that look like a rover; a camera on its own). The
number of devices is not given; a device that is unplugged is simply not there.

**A device** is then a node with an id, a kind, and its driver's description of that kind: its frames
(named points on it: each camera, each tag, the gripper tip, the wheels' axle), its sensors, and its
commands with their parameters and ranges. The API's guide, validation and OpenAPI document are
generated from the devices found, as `ROUTES` does today.

| Device | Sensors | Actuators | Knows about itself | Does not know |
|---|---|---|---|---|
| `arm` | 6 encoders, servo feedback, wrist camera | 6 servos | Its own shape, so where the wrist camera and gripper are relative to its base | What it stands on, what is around it |
| a fixed camera | one camera | none | What kind of lens it has | Where it stands or what it looks at |
| `rover` | 2 wheel encoders, 2 cameras | 2 wheel servos | Its own shape, how far its wheels have turned | Where it is, how big its surface is |

### 5.3 The knowledge rule: nothing about the surroundings

Every value the hub uses is one of two kinds:

- **Sensed**: came through a port.
- **Built in**: what a device *is*. Its maker knows it, it ships with the device's driver, and it is
  the same wherever the device is put down: the arm's link lengths and joint ranges, the rover's wheel
  size and where its cameras sit on it, a lens's nominal values, the tags printed on a device's own body.

Two kinds it never has:

- **True**: the simulator's state.
- **Told**: anything a person could write down about the place. No table size or height, no object
  list, no camera positions, no device positions, no count of devices, no positions or sizes of anything
  in the surroundings.

Built-in values are treated as starting values, not as facts: they are checked against what is seen
and corrected (5.9).

**Where size comes from.** Pictures alone give shapes, not sizes: a scene twice as large and twice as
far looks the same. A length has to come from somewhere, and the choice is which one (D13):

| Ruler | The one built-in number it rests on | Strength | Weakness |
|---|---|---|---|
| **The rover moves a known distance between two pictures** | The wheel's radius: the encoders count turns, and turns become distance only through the wheel's size | Needs one camera, not two. The distance moved can be as long as wanted, which measures far things better than a short fixed baseline. Nothing in the surroundings is involved | Wheels slip and tyres squash, so it is the least exact; turning adds error, so measure on straight moves |
| The rover's two cameras | The distance between them | Rigid, exact, works standing still | Short, so poor at distance; a second camera and board |
| A tag on a device | Its printed size | Works from any camera that sees it | Only where a tag is in view |
| The arm moves its wrist camera | Its link lengths | Exact near the arm | Only within the arm's reach |

The plan takes rover motion as the primary ruler, since it asks least: one number about one device.
The others are not trusted as given. They are measured against it, and a disagreement is reported
(a rover whose two rulers differ by 3 % has a wrong wheel size or a slipping wheel). No ruler gets
below one built-in number; the only ones that would are new parts (an accelerometer measures against
gravity, a time-of-flight sensor against the speed of light).

Enforced by tests, not by care:

1. The hub's package never imports the world's modules (checked by reading its imports), and is started
   from ports alone.
2. **New places.** The suite runs the same hub, unchanged, in worlds generated from seeds: different
   surface sizes and shapes, different clutter, cameras and devices put down anywhere, devices missing.
   Nothing may depend on a number that was true of the old workcell.
3. A blindfold test: change the truth without changing anything sensed (shift a camera that no one is
   looking through, move an object out of every view) and every API answer stays the same.
4. The opposite test: move a fixed camera 3 cm in the world and the hub's estimate of it follows, from
   pictures alone.

### 5.4 Finding where the devices are

**Markers are on devices only.** A device's tags are part of its body, so their size and position on it
are built in. Nothing is stuck on the table or anywhere else in the surroundings: a place the system has
to work in cannot be assumed to have been prepared.

| Where | Seen by |
|---|---|
| On the arm's base (a pair, to defeat the flipped reading), and one on the wrist camera holder | Fixed cameras, rover |
| On the rover: top, and upright on its sides | Fixed cameras, wrist camera |
| In the surroundings | None placed. If one happens to be there it is a landmark of unknown size and position, like any other distinctive thing |

**Every device contributes a kind of link:**

| Link | From |
|---|---|
| camera → a device's tag | Any camera that sees it (pose from its corners, with an uncertainty that grows with distance and shrinks with pixel size) |
| arm base → wrist camera, arm base → wrist tag | The arm's encoders and its own shape. The arm can also move its wrist tag to where a camera can see it |
| rover then → rover now | Wheel encoders (drifts; good over short, straight moves). With a picture at each end, this link is what gives everything its size (5.3) |
| left camera → right camera | The rover's own shape |
| camera → landmark | The same distinctive point in the surroundings seen from two places. It ties together devices that cannot see each other |
| "on one surface" | The rover rolls on a surface it has itself found to be flat there (5.5), not one it was told about |

**The estimate.** All links go into one least-squares problem over the unknown poses (scipy is already a
dependency). There is no world frame: answers are given relative to a frame the caller names, by default
the arm's base, which is where `move_tip` coordinates already live (D11). Each answer carries an
uncertainty, its age, and which links support it. Two devices with no chain of links between them are
reported as "not known", never guessed.

The same problem also corrects built-in values that are slightly off: each servo's zero, a lens's
values. Two of these are placeholders `PLAN.md` still lists as "measure on the real arm".

### 5.5 Finding out what the place is

With nothing told, the hub builds its own picture of the surroundings, and it starts empty.

| What is found | From |
|---|---|
| The surface the devices are on: where it is flat, how far it extends, where it ends | The rover's stereo pair (the ground ahead); fixed cameras once located; the wrist camera moved by the arm (a known motion between two pictures gives depth) |
| Things on it: where something stands and roughly how big | The same, plus any two located cameras seeing the same thing |
| That something is solid, and exactly how high | **Touch**: the arm lowered slowly until the stall guard says it has met something |
| Where nothing has been looked at yet | Everything not covered above |

Every part of the map is one of three states: **seen free**, **seen occupied**, **never seen**. The
third is the important one. It is what tells the model where to send the rover or point the wrist
camera next, and it is what safety is cautious about (5.8).

The split of work: the hub does geometry (surfaces, extents, occupied space), because safety needs it
whoever is deciding. What a thing *is* and what to do with it is the model's job, from pictures (D1).

### 5.6 The rover

Built from what the project uses, plus more of the same parts:

| Part | Qty | Note |
|---|---:|---|
| HX-30HM servo | 2 | As wheel motors: operating mode 1, "constant-speed motor mode" in the register table. Unverified on a real servo |
| Camera Module 3 Wide | 2 | Fixed focus (a firmware build option today) |
| ESP32-P4 board | 2 | One camera port per board, so stereo needs two boards |
| Printed chassis, two wheels, a skid or ball at the back | | Tags on top and sides are part of the print |

- **Size.** The servo body is 45.2 × 24.7 × 35 mm and 52 g. Two back to back, two cameras and two boards
  come to roughly 100 × 100 × 70 mm and 200 g. An estimate, not a design.
- **Speed.** No-load 5.96 rad/s at 12 V; with 50 mm wheels that is about 0.15 m/s at most.
- **Odometry.** The wheel servos' position registers, read often enough to count turns.
- **Stereo.** The two boards run unsynchronised (no shared trigger; noted in `PLAN.md`), up to a frame
  apart. So the rover looks while standing still. The stereo project's beam has a 120 mm baseline; at
  table distances 60 mm is plenty and makes a smaller rover (D4).
- **Size from motion.** Drive straight a known distance, take a picture at each end: the two pictures
  are a stereo pair whose baseline is the distance driven (5.3). This works with one camera, so a
  one-camera rover (half the camera parts, no synchronisation question) is a real option; with two, the
  fixed pair and the driven pair check each other (D4).
- **Camera tilt.** About 20° down: it has to see the ground just ahead, to know there is ground.
- **On hand:** two cameras and two boards, both already assigned (wrist, scene). The rover needs two more
  of each and two more servos (D4).
- **Power and data** is the largest open choice (D3): a tether (the 3-pin servo cable carries 12 V and
  data, plus one USB cable per camera) or a battery with a board on the rover running the wheels.

In the simulation: a chassis with two wheel joints and a rear skid, in the same model as the arm; the
emulator gains mode 1; the wheel servos sit on the bus as ids 7 and 8 (or their own adapter); each
camera gets its own camera port.

Commands at the level a person thinks: drive a distance, turn an angle, stop, take a stereo pair.
Later, "go to this point relative to that device", which needs the locator.

### 5.7 The model's API, sketched

```
GET  /                               guide: how the API works and what devices were found. Nothing about the place
GET  /devices                        every device discovered: id, kind, health
GET  /devices/{id}                   its description: frames, sensors, commands and ranges
GET  /devices/{id}/state             what its own sensors say now
GET  /devices/{id}/camera/{name}     a picture, with its time stamp; ?tags=true adds the tags found in it
POST /devices/{id}/{command}         an action (move_tip, gripper, drive, turn, ...)
POST /devices/{id}/stop              freeze that device
GET  /where?of=rover&relative_to=arm the estimated pose, its uncertainty, its age, what supports it
GET  /map?relative_to=arm            what has been found of the place: surface, things, and what is still unseen
POST /locate                         take fresh full-size stills on every camera and re-estimate
POST /wait, POST /stop_all
```

The arm's present endpoints keep their meaning under `/devices/arm/`. What a person asks for arrives as
words, in the conversation, not through the API. The simulation's operator actions (start again, a new
place from a seed) stay outside the model's API.

### 5.8 Safety when nothing is known

- **Stop path for all.** `stop_all` freezes every device; every exit switches every servo's torque off.
  The rover's "freeze" is wheels stopped.
- **The rover drives only onto surface it has seen.** It has no keep-in area to be given. Ground that
  no camera has seen as ground is treated as an edge. A lip around the bench is a backup while this is
  being developed, not something the system relies on (D8).
- **The arm in unseen space.** Its keep-outs today are the table and its own base. Its base stays (that
  is itself). The table's height becomes a cautious starting assumption: nothing below the plane it
  stands on. Everything else comes from the map: seen-occupied space is kept out of, and into never-seen
  space the arm moves slowly, so that the stall guard meets anything there gently.
- **Arm and rover.** The rover's estimated position, grown by its uncertainty, is occupied space to the
  arm. With no estimate, the arm stays above the rover's height wherever the rover could be.

### 5.9 Keeping the gap small

- **Every run is a new place.** Worlds are generated from a seed: the surface's size and shape, clutter,
  lighting, where each device is put down and which are present. The hub never sees the seed or anything
  made from it. The fixed `workcell` and `bench` scenes become two places among many.
- **Built in is not exact.** The simulated devices differ from their drivers' descriptions by realistic
  errors, also from the seed: a servo zero a few ticks out, a wheel 1 % larger, a lens with distortion,
  a tag printed 1 to 2 mm off. The hub has to notice and correct.
- **Calibration without a prepared target.** The devices' own tags are the targets: the arm shows its
  wrist tag to a camera at several poses; the rover turns in front of one. No board placed by a person.
- **Every camera has delay and drops**, through the camera firmware build or the quick port.
- **The same artwork**: the tag images the simulation shows are the files that get printed.

## 6. Steps, in order

Each step ends with tests in the repo's style: properties, checked against the referee.

| # | Step | Size | Done when |
|---:|---|---|---|
| 0 | Commit the working tree; run the full suite once | | Clean tree, suite green |
| **1** | **Locate cameras from the arm's own tags.** A marker module (generate the tag files; put a pair on the arm's base and one on the wrist holder through `build_model.py`); detect tags in full-size stills; estimate each fixed camera relative to the arm's base from pictures alone. Nothing on the table, no API change, no rover, no new architecture | Small | A test puts the cameras at poses drawn from a seed and the estimate follows, to within a stated error at 720p and 1080p; a mirrored or rotated tag fails the test; the flipped reading is caught; the code holds no number about the scene |
| 2 | **Hub and device API, nothing told.** Split out the hub, started from ports alone; discovery; arm and cameras as devices; referee; a first generator of places (surface size, camera placement, objects); the tests of 5.3. The old `sim_api.py` stays until the new one can do the same pick | Medium | A pick through the new API in several generated places; the blindfold and new-places tests pass |
| 3 | **Locator.** The pose problem of 5.4 with the arm contributing (wrist camera, wrist tag shown to cameras); `GET /where`, `POST /locate` | Medium | Poses within stated error for generated placements; "not known" when the chain is broken; the uncertainty is honest (the true error is inside it at the stated rate) |
| 4 | **Rover, driving blind.** Model, wheel mode in the emulator, drive and turn, odometry, its filter and stop | Medium | Drives a square and reports where it thinks it is; the referee shows the drift; it stops on `stop` |
| 5 | **Rover in the locator, and size from its motion.** Its cameras through camera ports, tags on it, odometry links; drive-and-look as the ruler, with the fixed pair and the tags measured against it | Medium | The rover's pose is known while any device sees it or it sees a device, and degrades smoothly when neither. Sizes measured by driving match the referee's within a stated error, with slip in the simulation; a wrong wheel size is reported, not absorbed |
| 6 | **The map** (5.5): the surface and its edges, things on it, never-seen space, touch; landmarks as links; `GET /map` | Large | On generated surfaces the found extent matches the referee's within a stated error; nothing is reported as seen that was not |
| 7 | **Safety when nothing is known** (5.8) | Medium | In fuzzed runs on generated surfaces the rover never leaves the surface and the arm never enters the rover's estimated space or hits an unseen thing hard |
| 8 | **Built in is not exact** (5.9), self-calibration | Medium | The whole suite passes with seeded device errors; calibration recovers them |
| 9 | **A task for two devices in a place nobody described, with a model driving.** For example: the rover finds a part no fixed camera can see, the arm fetches it | Medium | A model run in generated places, scored by the referee |
| 10 | Real hardware, device by device | | Each device's port swapped for the real one |

Step 1 is small on purpose: it produces the number everything else depends on (how well a camera can be
located from a device's markers here) and commits to nothing in the architecture.

## 7. Decisions needed

| # | Decision | Options | My recommendation | Needed by |
|---|---|---|---|---|
| D1 | Who works out positions and the shape of the place | (a) the hub: sensing in; poses, surfaces and occupied space out, with uncertainty; (b) the model, from pictures or raw tag sightings | (a) for geometry, and also give the raw sightings. What things *are* stays the model's job, in line with removing `/pixel` (DECISIONS R11). This is the decision that most shapes the API | Step 2 |
| D2 | "Like a CAN bus": a way of organising devices, or a real bus to build | (a) logical only: devices are entries in the hub; the wires stay the servo bus and USB; (b) a real shared bus between boards | (a). Descriptions and addressing are shaped so a real bus could carry them later | Step 2 |
| D3 | Rover power and data | (a) tether: servo cable (12 V + data) and two USB cables; (b) battery, with a board on the rover running the wheels and cameras over Wi-Fi | (a) to start: every interface exists in the repo. Its risk is real: three cables may drag a 200 g rover; the wire simulation can measure that before anything is built. (b) needs new firmware | Step 4 |
| D4 | One camera or two on the rover, the baseline, and buying parts | Two cameras 120 mm apart (the stereo project's beam as it is) or about 60 mm (new, smaller); or one camera, with depth from driving (5.6). Two more servos either way; one or two more cameras and boards | Two at 60 mm, as first asked: the fixed pair checks the driven one and sees depth without moving. Simulate both in step 5 before buying | Step 4 |
| D5 | Marker family and the new dependency | AprilTag 36h11 via `opencv-python-headless`; or a dedicated AprilTag library | OpenCV: one package, also gives calibration and stereo. Tags 30 to 40 mm, on devices only; locate at full size, not 640×360 | Step 1 |
| D6 | Truth and scene description in the API | Today object positions are on by default, and the guide describes the scene | Remove both entirely; the referee has the truth | Step 2 |
| D7 | Time | (a) turn-based, one action at a time, as now; (b) devices act at once | (a) first. Add actions that return at once when a task needs two devices moving together | Step 2 |
| D8 | Keeping the real rover on its surface | (a) it drives only onto surface it has seen; (b) a lip around the bench; (c) a downward distance sensor (a new part) | (a), with (b) as a backup during development | Step 7 |
| D9 | Backends | MuJoCo only, or Newton too | MuJoCo only: Newton's renderer shows no textures, and the bus, cameras and API are MuJoCo only already | Step 1 |
| D10 | Where the hub lives | A new top-level package beside `sim/`, or inside `sim/` | Top level: it is the real machine's code, and the import test of 5.3 is then simple | Step 2 |
| D11 | Frame for coordinates given to the model | The arm's base, or whichever device the caller names | The caller names it; the arm's base by default when there is an arm | Step 3 |
| D12 | The first two-device task | | A generated place with a part hidden from the fixed cameras | Step 9 |
| **D13** | **How much may a device know about itself, and what is the ruler?** | (a) each device's own body as its driver describes it, all trusted; (b) one trusted length, the rover's wheel radius, with size taken from driving and every other built-in length (tag sizes, camera spacing, the arm's links) measured against it; (c) as (b), and the arm also learns its own shape by watching itself move; (d) nothing at all | (b). The arm still uses its driver's description to move, but the locator does not take its lengths on trust. (c) is a research project in its own right. (d) is not possible without a new kind of sensor (5.3). Consequence of (b): until the rover has driven, sizes are provisional, so step 1 reports camera poses up to the tag's nominal size | Step 1 |

D5, D9 and D13 are needed to start step 1.

## 8. Risks and unknowns

- **This is a harder problem than the first version of the plan.** With nothing on the table, two
  devices that cannot see each other's tags share no reference until both have seen the same landmark.
  That work (step 6) was "later" before and is now on the main path.
- **Size rests on the rover's wheels.** A wheel 1 % larger than its driver says makes every distance 1 %
  short, and slip adds to it. How large this is on a real table surface is unknown; step 5 measures it in
  simulation with slip, and the cross-check against the other rulers is what catches it. A place with no
  rover has only the weaker rulers.
- **Real pictures are worse than these.** Blur, noise, distortion, JPEG and uneven light will cut the
  range and accuracy of section 4. Only hardware settles it.
- **Accuracy against what grasping needs.** One tag gives a few millimetres at half a metre in ideal
  pictures; a grasp wants a few millimetres in total. The locator has to earn its accuracy from many
  links, and the wrist camera and touch remain the close-range checks.
- **Exploring takes time.** A system that starts knowing nothing spends its first minutes looking. At
  several seconds per model decision, how much of that the hub does unprompted is worth deciding early (D1).
- **Servo wheel mode is unverified**, like the rest of the register table: speed control quality at low
  speed, and whether the position register counts turns in mode 1.
- **Unsynchronised stereo** limits the rover to looking while still, unless the boards can be triggered together.
- **Simulation speed.** Byte-level bus and camera firmware for four cameras is slow; hence the two kinds of port.
- **The arm's code is single-arm throughout.** The plan avoids rewriting it (the rover gets a parallel
  path), which leaves two styles side by side until someone unifies them.
- **A second arm** is not covered. The device model allows it; `JOINTS` does not.

## 9. Not in this plan

Policy training, the Newton backend, the real rover's CAD, and devices recognising each other without
tags (the step after markers: the same links, from the devices' shapes).
