# Multi-device plan: several robots, one language model

Written 2026-11-14. **Planning only: nothing here is built.** Section 7 lists what has been decided and
what is still open. `DECISIONS.md` has not yet been brought up to date with the answers of the last revision.
Progress is tracked in [MULTI_DEVICE_CHECKLIST.md](MULTI_DEVICE_CHECKLIST.md).

Revisions, all 2026-11-14:

1. The system is given no prior information about its surroundings. The set-up file filled in by a
   person is gone.
2. Size comes from a device moving its camera a known distance between pictures.
3. The rover runs from a battery.
4. Answers from the user: scenes are config files, loadable at run time and editable over REST; the
   rover's stereo head is motorised (camera spacing, tilt, pan, lift) and surveys the place at start-up
   without driving; devices form a logical network of interest groups; ground truth leaves the model's
   API; a lip keeps the rover on the table; MuJoCo only; the hub is a top-level package; OpenCV for tags;
   realistic lens distortion; several environments and tasks, created and validated before anything runs;
   then the rover's CAD, contacts and a real parts list.

## 1. The goal

Turn the single-arm simulation into an environment with several devices commanded by one language model.

- The model is the only decision-maker. Devices are peripherals it commands and queries, like nodes on a bus.
- A "device" is any cluster of sensors and actuators with positions: the arm, a fixed camera, and first of
  the new ones, a small wheeled rover that drives on the table and only looks. Its two cameras sit on a
  motorised head, so it can look around, up and down, from higher or lower, and with the cameras closer
  together or further apart.
- **No prior information.** The system is told nothing about where it is: not the table's size or height,
  not what is on it, not where any device stands, not how many devices there are. It finds out. The aim
  is a system that can be put down somewhere new and work the place out.
- **Devices do not know where they are relative to each other.** Those positions are estimated from what
  the sensors see, with every device contributing. Printed markers on the devices to start with.
- **Smallest possible gap to the real thing.** The model's API gives only what the real system has worked
  out from its own sensors. Never the simulator's true state, and no description of the scene written by
  a person. Camera pictures are not perfect: they pass through a realistic lens.
- Simulation only for now (MuJoCo), but the rover will be built.

## 2. The repo as it stands

### What this can be built on

| Piece | Where | Why it matters here |
|---|---|---|
| Controller-side bus code that knows nothing about the simulation | `so101sim/bus/driver.py`, `arm.py` | Already the shape we want: it talks to a port (bytes), and the port is either the simulated wire or a serial port |
| Byte-level servo bus with emulated HX-30HM servos | `so101sim/bus_backend.py`, `bus/servo_emu.py`, `bus/wire.py` | More servos on a bus (the rover's wheels and head) is mostly more entries in a list |
| Camera firmware compiled for the PC, fed by the simulation | `so101sim/camera_fw.py`, `firmware/p4-uvc-camera/host/` | A camera as the PC really sees it: frames with delay, drops and start-up time. Today for the wrist camera only |
| Devices on the physics clock | `MujocoBackend.step_hooks` | The hook the camera firmware uses; other devices can tick on it |
| One table that produces the guide, the OpenAPI document and request validation | `sim_api.py` `ROUTES` | The same trick works per device: the guide is generated from device descriptions |
| Command filter, physics monitor, stop path | `safety.py`, `monitor.py`, `Teleop.freeze` / `set_torque` / `shutdown` | The arm's protections stay as they are. The stop path needs widening to every device (5.9) |
| The stall guard | `Teleop._blocked` | It already notices when the arm meets something. That is a touch sensor (5.5) |
| CAD by script, and a sim model made from the CAD | `hardware/wrist_mount/`, `hardware/camera_mount/` (CadQuery), `tools/build_model.py` | The route the rover takes too: parts from a script, masses and collision shapes from the meshes |
| Scenes and tasks with a reference solution | `scene/`, `so101sim/tasks.py`, `tests/test_tasks.py` | The pattern for "feasible is proved, not argued" carries over |

### What is in the way

| Assumption | Where | What it costs |
|---|---|---|
| One arm, six joints, everywhere | `JOINTS` is used in 15 files; `MujocoBackend.act` / `qadr`; `safety.py`; `servos.toml` has one `[follower]` | A second kind of device needs its own joint set, limits and filter. The arm's code can stay as it is if the rover gets its own, parallel path |
| `Teleop` is the arm's controller, the window and the thing that steps the world | `scripts/teleop.py` | `SimApi` drives it headless already. Other devices need to tick in the same loop |
| The API is the arm's API | `sim_api.py` (`/move_tip`, `/state`, ...) | Endpoints need a device in their address |
| **Scenes are fixed files, built ahead of time** | `scene/*.xml` hand-written; `tools/build_model.py` writes `sim/model/*.xml`; chosen by `--scene` at start | A scene cannot be changed or swapped while the program runs |
| **Tasks are code** | `so101sim/tasks.py`, one class per task | A new task needs a programmer. Tasks should be data, like scenes |
| **The scene is written into the code** | The guide's text (three named cameras and where they stand, the objects and their sizes); `safety.toml` (`table_z_m`); `/state` `table_m`; `MIN_TIP_Z`; `HOME_TIP` | Each is a fact about one particular table. All of it has to be found by sensing or dropped |
| The servo emulator has position mode only | `servo_emu.py` logs "operating mode N is not modelled; output is off" | Wheels need mode 1, constant speed |
| Fixed cameras are placed by the config, and the same config is what the controller reads | `config/cameras.toml`, `Teleop.click_table` | The controller's "calibration" is the simulator's truth, exactly |
| MuJoCo pictures are a perfect pinhole | `MujocoBackend.render`; `sim/README.md` says lens distortion is applied on Newton only | The lens has to be added after rendering (5.10) |
| No vision library | `sim/requirements.txt` | OpenCV is the one new dependency (decided) |

## 3. Where prior information reaches the model today

An audit of `sim_api.py` against the rule in section 1. Two kinds of leak: the simulator's truth, and
facts about this scene that a person wrote down.

| What the model gets | Source | Plan |
|---|---|---|
| `objects`, `fixtures` in `/state` (on by default) | True state | Gone. Truth goes to the referee (5.1) |
| `GET /task` "which goals hold now" | True state | The model gets the request in words only. Goal checking is the referee's |
| `POST /randomize` answer with positions | True state | Moves to the operator's API (5.8), which the model cannot reach |
| `joints_rad`, `tip_m` | True joint angles unless `--bus` | The hub always reads through the bus (ticks, delay, lost replies) |
| Camera pictures | MuJoCo render: perfect pinhole, no delay, no noise | Every camera goes through a lens model (5.10) and a camera port with delay and drops |
| Camera lens values used by the controller | `cameras.toml`, which also builds the sim | Each kind of camera starts from its lens's nominal values and corrects them from what it sees (5.10) |
| Fixed cameras' poses (`click_table`) and the guide's sentences on where each camera stands | `cameras.toml`; written by hand | Estimated from sensing. The sentences go |
| Which devices and cameras exist | Written into the guide | Discovered at start-up (5.2) |
| The list of objects with sizes, colours and weights | Model geoms, via the guide | Gone. What is on the table is for the system to find |
| `table_m`: the table's extent | Model geom | Gone. The surface and its edges are found by looking (5.5) |
| The table's height in the arm's frame | `safety.toml`, `MIN_TIP_Z` | A cautious starting assumption only, replaced by what is sensed (5.9) |
| `checks.physics_violations` | Physics monitor | Referee only. A run with a violation is still a failed run |
| `checks.command_filter`, `limited`, `why_not_reached` | The controller's own filter and encoders | Keep |
| Simulated time, turn-based | The sim clock | Keep (open, O4) |

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
- **The scene can be put together in memory at run time.** The probe added textures and shapes to a
  loaded model through MuJoCo's `MjSpec` and compiled it. That is the mechanism run-time scenes need (5.7).
- **640×360 is too small for localisation.** Locating should use full-size stills; previews can stay small.
- **A single small tag can be read flipped.** At 720p two tags (19 and 26 px wide) came out tilted about
  90° wrong, with the right answer as the solver's second candidate. Known behaviour of flat markers. The
  fix is to never trust one small tag alone: several tags on one body, several views, or both.
- **Lighting matters.** Upright tags in the scene's shadow side were not found beyond 0.2 m.
- Not settled: these pictures had no blur, noise, lens distortion or JPEG artefacts, so numbers through
  the lens model of 5.10 will be worse. And the probe did not pin down which way round a texture sits on
  a box face; the marker step's test must check for a mirrored or rotated tag.
- The probe put tags on the table. The plan does not (5.4): tags sit on devices only.

## 5. Recommended design

### 5.1 Four parts, two APIs, and one rule about what may cross

```
                 the model                              a person, a test, a script
                     │  hub API (5.8)                              │  operator API (5.8)
      ┌──────────────┴───────────────┐                             │
      │            HUB               │   top-level package `hub/`  │
      │  devices it has discovered,  │   runs unchanged on the     │
      │  as a tree of interests      │   real machine              │
      │  locator, map of the place   │                             │
      │  safety, stop path           │                             │
      └───┬──────────┬───────────┬───┘                             │
   serial ports   camera ports  clock       ← PORTS: the only way in or out
      ┌───┴──────────┴───────────┴───┐                             │
      │           WORLD              │   package `sim/`, MuJoCo    │
      │  the scene now loaded,       │◄────────────────────────────┘
      │  physics, emulated servos,   │   load / edit / swap scenes,
      │  lenses, camera firmware     │   referee: truth and scores
      └──────────────────────────────┘
```

- **World** (`sim/`). One MuJoCo model built from the scene file now loaded, one clock. To the hub it
  offers ports and nothing else.
- **Ports.** A serial port per servo bus (bytes), a camera port per camera board (frames with a time
  stamp), a clock. On the real machine these are a serial or network connection, a camera stream and the
  wall clock.
- **Hub** (`hub/`, beside `sim/`). Everything that would run on the real PC: device drivers, the
  locator, the map, safety, the model's API. It is started with ports and nothing else, and never
  imports from `sim/`.
- **Operator API**, on the world's side and on a separate address. Scenes are listed, loaded, edited
  and swapped here, and the referee answers here (true poses, task goals, scores). The model is never
  given this address.

Two speeds of port, same hub: byte-level (bus emulation, camera firmware) for checking the real code
paths, and a quick kind (encoder ticks and delayed pictures without the byte traffic) for long runs.

### 5.2 Devices as a logical network of interests

Decided: the network is logical. The wires stay what they are (servo buses, camera links); the hub
presents what it finds as one tree.

**Discovery.** At start-up the hub scans its ports: which servo ids answer on each bus, which cameras
are there. It matches what it finds to the kinds of device it has drivers for. The number of devices is
not given; a device that is unplugged is simply not there.

**The tree.** My reading of "grouped by interest, hierarchical interest entities": every node is
something a caller can be interested in, each with an address, and what is asked of a node applies to
everything under it.

```
devices/
  rover/
    drive/            wheels, odometry
    head/             pan, tilt, lift, spacing
      left/  right/   the two cameras
    power/            battery voltage
  arm/
    joints/  gripper/  wrist_camera/
  camera_1/
place/                what has been found of the surroundings (5.5)
  surface/  things/  unseen/
groups/               the same leaves, gathered by kind of interest across devices
  cameras/            every camera, wherever it is mounted
  movers/             everything that can move itself
  power/              everything with a supply to watch
```

- Reading a node gives the state of everything below it: `devices/rover` is the whole rover,
  `groups/cameras` is every camera.
- Commanding a node reaches everything below it: stop at `devices/rover` stops wheels and head; stop at
  the root stops everything.
- A leaf belongs to one device and to any number of groups.

This reading needs confirming (O1).

| Device | Sensors | Actuators | Knows about itself | Does not know |
|---|---|---|---|---|
| `arm` | 6 encoders, servo feedback, wrist camera | 6 servos | Its own shape, so where the wrist camera and gripper are relative to its base | What it stands on, what is around it |
| a fixed camera | one camera | none | What kind of lens it has | Where it stands or what it looks at |
| `rover` | 2 wheel encoders, 4 head encoders, 2 cameras, battery voltage | 2 wheel servos, 4 head servos | Its own shape; how far its wheels have turned; where each camera is relative to its chassis | Where it is, how big its surface is |

### 5.3 The knowledge rule: nothing about the surroundings

Every value the hub uses is one of two kinds:

- **Sensed**: came through a port.
- **Built in**: what a device *is*. Its maker knows it, it ships with the device's driver, and it is
  the same wherever the device is put down: the arm's link lengths and joint ranges, the rover's wheel
  size and the geometry of its head, a lens's nominal values, the tags printed on a device's own body.

Two kinds it never has:

- **True**: the simulator's state.
- **Told**: anything a person could write down about the place. No table size or height, no object
  list, no camera positions, no device positions, no count of devices, no positions or sizes of anything
  in the surroundings. The scene file (5.7) is exactly this, and it stays on the world's side.

Built-in values are treated as starting values, not as facts: they are checked against what is seen
and corrected (5.10).

**Where size comes from.** Pictures alone give shapes, not sizes: a scene twice as large and twice as
far looks the same. A length has to come from somewhere: a camera moved a known distance between two
pictures. The choice is which motion to trust (O2):

| Ruler | The one built-in number it rests on | Strength | Weakness |
|---|---|---|---|
| **The head moves the cameras apart** | How far the spacing drive travels per motor turn | Rigid, no slip, works standing still, so it works during the start-up survey. Wide for far things, narrow for near ones | Only as straight as the slide; play in it tilts the cameras |
| **The head lifts** | How far the lift travels per motor turn | Same, in the vertical direction; an independent check | Same |
| The rover drives a known distance | The wheel's radius | The distance can be long, which measures far things best | Wheels slip and tyres squash, so it is the least exact; and it needs somewhere safe to drive |
| A tag on a device | Its printed size | Works from any camera that sees it | Only where a tag is in view |
| The arm moves its wrist camera | Its link lengths | Exact near the arm | Only within the arm's reach |

With a motorised head, the rover has a better ruler than its wheels. I recommend the head's spacing
drive as the primary ruler, with the lift, the wheels and the tags measured against it and any
disagreement reported. No ruler gets below one built-in number; the only ones that would are new kinds
of sensor (an accelerometer measures against gravity, a time-of-flight sensor against the speed of light).

Enforced by tests, not by care:

1. `hub/` never imports from `sim/` (checked by reading its imports), and is started from ports alone.
2. **New places.** The same hub, unchanged, runs in every scene of 5.7. Nothing may depend on a number
   that was true of the old workcell.
3. A blindfold test: change the truth without changing anything sensed (shift a camera that no one is
   looking through, move an object out of every view) and every hub answer stays the same.
4. The opposite test: move a fixed camera 3 cm in the scene and the hub's estimate of it follows, from
   pictures alone.

### 5.4 Finding where the devices are

**Markers are on devices only.** A device's tags are part of its body, so their size and position on it
are built in. Nothing is stuck on the table or anywhere else in the surroundings: a place the system has
to work in cannot be assumed to have been prepared. AprilTag 36h11, read with OpenCV (decided).

| Where | Seen by |
|---|---|
| On the arm's base (a pair, to defeat the flipped reading), and one on the wrist camera holder | Fixed cameras, rover |
| On the rover: on its chassis top and sides, and on its head | Fixed cameras, wrist camera |
| In the surroundings | None placed. If one happens to be there it is a landmark of unknown size and position, like any other distinctive thing |

**Every device contributes a kind of link:**

| Link | From |
|---|---|
| camera → a device's tag | Any camera that sees it (pose from its corners, with an uncertainty that grows with distance and shrinks with pixel size) |
| arm base → wrist camera, arm base → wrist tag | The arm's encoders and its own shape. The arm can also move its wrist tag to where a camera can see it |
| rover chassis → each of its cameras | The head's four encoders and its own shape |
| rover then → rover now | Wheel encoders (drifts; good over short, straight moves) |
| camera → landmark | The same distinctive point in the surroundings seen from two places. It ties together devices that cannot see each other |
| "on one surface" | The rover rolls on a surface it has itself found to be flat there (5.5), not one it was told about |

**The estimate.** All links go into one least-squares problem over the unknown poses (scipy is already a
dependency). There is no world frame: answers are given relative to a node the caller names, by default
the arm's base when there is an arm. Each answer carries an uncertainty, its age, and which links
support it. Two devices with no chain of links between them are reported as "not known", never guessed.

The same problem also corrects built-in values that are slightly off: each servo's zero, a lens's
values. Two of these are placeholders `PLAN.md` still lists as "measure on the real arm".

### 5.5 Finding out what the place is

With nothing told, the hub builds its own picture of the surroundings, and it starts empty.

| What is found | From |
|---|---|
| The surface the devices are on: where it is flat, how far it extends, where the lip is | The rover's head; fixed cameras once located; the wrist camera moved by the arm |
| Things on it: where something stands and roughly how big | The same, plus any two located cameras seeing the same thing |
| That something is solid, and exactly how high | **Touch**: the arm lowered slowly until the stall guard says it has met something |
| Where nothing has been looked at yet | Everything not covered above |

Every part of the map is one of three states: **seen free**, **seen occupied**, **never seen**. The
third is the important one. It tells the model where to look next, and it is what safety is cautious
about (5.9).

**The start-up survey.** Before anything drives, the rover looks around from where it was put down. The
hub runs this by itself when it finds a rover; the model can ask for it again later.

1. Head low, cameras close together, level: turn through the pan range. Near things, the nearest tags.
2. Repeat tilted down (the ground close by, the lip if it is near) and tilted up.
3. Move the cameras apart and repeat: the same directions with a wide spacing reach further.
4. Lift and repeat: over low things, onto raised surfaces, and the height change is a second ruler.
5. Join it all: a first map with sizes, the devices whose tags were seen, and what is still unseen
   (behind things, and behind the rover if the pan range is less than a full turn).

The wheels do not move, so the survey needs no knowledge of the surface. The head moves slowly and
stops on contact, as the arm does.

The split of work: the hub does geometry (surfaces, extents, occupied space), because safety needs it
whoever is deciding. What a thing *is* and what to do with it is the model's job, from pictures.

### 5.6 The rover

**The head.** Four motorised axes carry the two cameras, each camera with its own board riding with it.

| Axis | What it does | A way to build it (to be settled in CAD) | Range to aim for |
|---|---|---|---|
| Pan | Turns the head about the vertical | A servo under the mast | About ±170°; a full turn would wind the wires up |
| Lift | Raises and lowers the head | A servo driving a parallel-bar arm, which keeps the head level as it rises | Lens height about 60 to 220 mm |
| Tilt | Looks up and down | A servo at the head | About 60° down to 30° up |
| Spacing | Moves the cameras apart and together, evenly about the centre | One servo, a pinion between two opposed racks | About 40 to 200 mm between lenses |

- **Control.** The head's servos are position servos like the arm's and get the same treatment: a driver
  with its own shape model, a command filter (limits, speed, the head against its own chassis, the
  cameras against each other), the stall guard, the stop path. Wheels and head are separate interests
  under one device.
- **One rule joins them:** the rover drives only with the head lowered and the cameras in. Raised and
  wide, it is top-heavy.
- **The cameras stand still to look.** The two boards are unsynchronised (`PLAN.md`), up to a frame
  apart. Every picture used for measuring is taken at rest.

**Parts.** Counts are firm; which parts is the job of the parts-list step (6, step 7).

| Part | Qty | Note |
|---|---:|---|
| Wheel servos | 2 | HX-30HM in operating mode 1, "constant-speed motor mode". Unverified on a real servo |
| Head servos | 4 | HX-30HM would work and is already understood, but at 52 g each it is heavy for a head; smaller bus servos are to be looked at |
| Camera Module 3 Wide | 2 | Fixed focus (a firmware build option today) |
| ESP32-P4 board | 2 | One camera port per board |
| A controller board for wheels, head and radio link | 1 | Either one of the camera boards or a third, fixed to the chassis. The camera boards move with the cameras, so a third is likelier |
| **Battery: Gens ace 5000 mAh 3S 60C LiPo, XT60** (decided) | 1 | 11.1 V nominal, 154 × 42.5 × 24.7 mm, 355 g, $70.09 |
| 5 V regulator, fuse at the battery, XT60 lead, 3S balance charger | | Not chosen |
| Printed chassis, mast, head, racks; wheels with tyres; a rear skid or ball | | Tags are part of the print |

On hand: two cameras and two boards, both already assigned (wrist, scene).

- **Size and weight.** Six servos, the battery, two cameras, three boards and the print come to very
  roughly 180 × 140 mm on the table and 0.9 kg. An estimate, not a design. With six servos to feed, the
  large battery is in proportion.
- **Speed.** No-load 5.96 rad/s at 12 V; with 50 mm wheels that is about 0.15 m/s at most.
- **Odometry.** The wheel servos' position registers, read often enough to count turns.

**Power: the battery.**

- *Voltage fits.* A 3S pack runs from 12.6 V full to about 9.9 V at a safe empty (3.3 V per cell). The
  HX-30HM is rated 11.1 V with a 9 to 12.6 V range, so those servos take the pack directly. Smaller head
  servos may need their own regulator.
- *The servos are the fuel gauge.* Each reports its supply voltage (register 62, 0.1 V steps). The hub
  reads it and stops the rover before a cell goes under about 3.3 V; a LiPo taken lower is damaged.
  The servo's own under-voltage protection is no help: its default threshold is 4.0 V.
- *Speed and torque change with charge* (both scale with voltage): about 20 % lower at empty than full.
  Distance driven does not, since it is counted from the encoders.
- *A fuse at the pack.* A 60C 5 Ah pack can deliver hundreds of amps into a short. Same reasoning as
  the 10 A fuse on the bench supply (DECISIONS B6), with more at stake.

**Data: by radio.** A battery only removes the tether if the data goes by radio. The ESP32-P4 boards
have Wi-Fi, and the existing Camera Lab firmware already streams pictures over it (640×480 at about
27 fps, measured). What does not exist:

- firmware that drives the servos from a board's serial pin, and carries the bus over Wi-Fi;
- the wiring between a 3.3 V board pin and the one-wire servo bus (the bus's signal level is unchecked);
- on the PC, a serial port that is a network connection. This fits the port idea of 5.1 unchanged.

**From CAD to simulation.** As the wrist mount was done:

- **CAD** by script (CadQuery), under `hardware/rover/`: chassis, mast and lift arm, head with its
  slide and racks, wheels. Printable parts as STEP and STL, with the checks the wrist mount has (one
  solid each, nothing overlapping through the axes' full travel, wires long enough at every pose).
- **Simulation model** generated from that CAD: each part's mass and inertia from its mesh; one joint
  per axis with the servo model; collision shapes by convex decomposition, not one hull per part
  (DECISIONS M1, M2).
- **Contacts**, which is where a wheeled model goes wrong: tyre on surface (friction, and the slip
  that odometry will see), the rear skid, the chassis and wheels against the lip, the head against the
  chassis, and the two camera carriages against each other and their end stops.
- **Parts list** chosen from what can actually be bought, with links and prices, in `BOM.md`.

In the simulation the rover's servos sit on a bus of their own, reached through a port with a radio
link's delay, jitter and losses; their supply voltage falls as the battery drains; each camera has its
own lens and camera port.

Commands at the level a person thinks: drive a distance, turn an angle, point the head, set height and
spacing, take a stereo pair, survey, stop.

### 5.7 Scenes and tasks are config files

Decided: a scene is a config file, loadable while the program runs, and editable over REST.

**What a scene file holds.** Everything about a place, and nothing about how to work in it:

```toml
name = "shelf"

[surface]                       # what the devices stand on
outline_m = [[0, 0], [0.9, 0], [0.9, 0.7], [0, 0.7]]     # any polygon, so L-shapes are just more points
lip_height_m = 0.02             # every scene has a lip (decided)
friction = 0.5

[light]
key = { from = [0.3, -0.4, 1.2], strength = 0.7 }

[[fixtures]]                    # things that do not move, or move on a slide or hinge
kind = "box"
name = "shelf"
size_m = [0.25, 0.12, 0.10]
at_m = [0.6, 0.5]

[[objects]]                     # loose things
name = "red_cube"
shape = "box"
size_m = [0.03, 0.03, 0.03]
mass_kg = 0.03
colour = "red"
on = "shelf"                    # or at_m = [x, y]

[[devices]]                     # where each device was put down. The hub is never told
kind = "arm"
at_m = [0.1, 0.35]
facing_deg = 0

[[devices]]
kind = "camera"
at_m = [0.95, 0.35, 0.40]
looking_at_m = [0.4, 0.35, 0.0]

[[devices]]
kind = "rover"
at_m = [0.5, 0.15]
facing_deg = 120

[imperfections]                 # how each device differs from its driver's description (5.10)
seed = 7

[[tasks]]
name = "fetch_from_shelf"
request = "There is a red cube somewhere up high. Put it in the tray."

[[tasks.goals]]
kind = "inside"
object = "red_cube"
region = "tray"
```

- **Loading at run time.** The world builds its MuJoCo model from the file in memory (the mechanism the
  probe used) and replaces the running one. The hub is not restarted and is not told: from its side the
  ports go quiet and come back, as if everything had been carried to another table. It has to discover
  and orient again.
- **Editing over REST** is the operator API of 5.8. An edit is checked before it is accepted; an edit
  to the loaded scene takes effect by reloading it.
- **Tasks are data too**: a request in words, and goals built from a small set of checks the referee
  knows (inside a region, on top of, upright, a fixture open or closed, a reported number within a
  tolerance). Tasks whose result is an answer, not an arrangement, are scored from what the model
  reports to the operator API.
- The existing `workcell` and `bench` scenes are converted to this form; `tasks.py`'s bench task becomes
  the first task file.
- Files are TOML (decided).

**Validation, without running any device.** A scene is valid when:

- the file has the required fields and no unknown ones, with every number in range;
- the model compiles; nothing overlaps at the start; every object and device is on the surface and
  inside the lip;
- left alone for a second of physics, with devices unpowered at their start poses, nothing moves;
- every task names things that exist, starts with its goals not met, and each goal could hold (the
  region exists and the object fits in it).

That a task can actually be done is a stronger claim and waits for a reference solution, once the
system runs (DECISIONS T7).

**Environments and tasks I propose.** To be created and validated (step 2), not run.

| Environment | What it is | What it is for |
|---|---|---|
| `workcell` | Today's table and three objects, with a lip | Continuity: the arm's existing behaviour in the new form |
| `bench` | Today's cabinet with drawer, bins, platform, socket, with a lip | Fixtures that move; things inside other things |
| `open_small` | A small bare surface, one arm, one camera, the rover, two objects | The easiest place to orient in |
| `l_shaped` | An L-shaped surface; the camera sees one leg only; the arm reaches neither end | Places only the rover can see; driving round a corner |
| `shelf` | A raised platform and an overhang; things on top and underneath | The head's lift and tilt |
| `cluttered` | Many things of different heights that hide each other; uneven light | Unseen space; choosing where to look next |
| `no_arm` | Rover and two fixed cameras only | Discovery with a device missing; there is no arm frame to lean on |
| `moved_arm` | The arm at a corner, turned 90° | Nothing may assume the arm is at the origin facing +x |

| Task | Environment | Asks for | Scored on |
|---|---|---|---|
| Where is everyone | any | The position of each device relative to another | Estimated poses against truth, and whether the stated uncertainty was honest |
| How big is this place | `open_small`, `l_shaped`, `no_arm` | The surface's outline and size | Reported outline against truth: this is the test of size |
| What is on the far leg | `l_shaped` | Count and colours of the things the fixed camera cannot see | The reported list |
| What is on the shelf, and how high is it | `shelf` | A description and a height | Reported height within tolerance |
| Fetch the hidden part | `shelf`, `cluttered`, `bench` | The rover finds it, the arm brings it to a tray | Final arrangement |
| Is the drawer open | `bench` | A yes or no that no fixed camera can give | The answer; then after an operator edit to the scene, the answer again |
| Tidy by colour | `cluttered` | Sort loose parts into bins | Final arrangement |
| Carried to another table | any two | The scene is swapped mid-run | That the hub notices, drops its old map, and orients again |

### 5.8 Two APIs

**The hub's, for the model.** Addresses follow the tree of 5.2. No ground truth anywhere (decided).

```
GET  /                                  guide: how the API works and what was found. Nothing about the place
GET  /{node}                            description of a node and what is under it (devices, devices/rover/head, groups/cameras, ...)
GET  /{node}/state                      what the sensors under it say now
GET  /devices/{id}/.../{camera}/picture a picture, with its time stamp; ?tags=true adds the tags found in it
POST /{node}/{command}                  an action (move_tip, gripper, drive, turn, point, lift, spacing, survey, ...)
POST /{node}/stop                       freeze everything under the node; at the root, everything
GET  /where?of=...&relative_to=...      an estimated pose, its uncertainty, its age, what supports it
GET  /place                             what has been found: surface, things, and what is still unseen
POST /wait
```

**The operator's, for people and tests.** On the world's side, on a separate address the model is never given.

```
GET    /scenes                          the scene files
GET    /scenes/{name}                   one scene, as data
PUT    /scenes/{name}                   create or replace (checked first; refused with reasons if invalid)
PATCH  /scenes/{name}                   change part of one (move an object, add a fixture, change the lip)
DELETE /scenes/{name}
POST   /scenes/{name}/validate          the checks of 5.7, without loading it
POST   /world/load                      make a scene the running one
POST   /world/reset                     the running scene back to its start
GET    /world/truth                     the referee: true poses and states
GET    /world/tasks/{name}              a task's goals and whether each holds now
POST   /world/tasks/{name}/answer       hand in a reported answer for scoring
```

### 5.9 Safety when nothing is known

- **Stop path for all.** Stop at the root freezes every device; every exit switches every servo's torque off.
- **The lip keeps the rover on the table** (decided). Every scene has one, and the real bench gets one.
  It must be higher than the rover can climb and low enough to see over; about 20 mm, to be checked
  against the wheel size in CAD. The system is not told about the lip. It sees it and maps it as the
  surface's edge, and the rover's filter keeps away from what the map says is occupied.
- **The rover's command filter.** Speed and acceleration caps; no driving into seen-occupied space;
  slow into never-seen space; no driving with the head raised or wide; stop when the radio link goes
  quiet or the battery is low.
- **The arm in unseen space.** Its keep-outs today are the table and its own base. Its base stays (that
  is itself). The table's height becomes a cautious starting assumption: nothing below the plane it
  stands on. Everything else comes from the map: seen-occupied space is kept out of, and into never-seen
  space the arm moves slowly, so that the stall guard meets anything there gently.
- **Arm and rover.** The rover's estimated position and head, grown by their uncertainty, are occupied
  space to the arm. With no estimate, the arm stays above the rover's greatest height wherever the rover
  could be.

### 5.10 Keeping the gap small

- **Many places.** The hub is developed against all the scenes of 5.7 and any new one made over the
  operator API. A generator that writes scene files from a seed comes later and costs little once
  scenes are data.
- **A realistic lens** (decided). MuJoCo draws a perfect pinhole picture. Each camera's picture is then
  passed through a model of its lens and sensor before anything else sees it:
  - *distortion*: the scene is rendered wider and larger than needed, then warped by the lens's
    distortion (OpenCV's lens models). The Camera Module 3 Wide's is strong: the datasheet's 102° across
    against 99° for a pinhole of the same focal length is what `cameras.toml` already notes;
  - *darkening toward the corners, blur that grows off-centre, sensor noise*;
  - *JPEG*, which the firmware path already applies.

  The numbers should come from calibrating one of the real cameras on hand; until then, values chosen
  to match the datasheet, marked as placeholders. Each simulated camera gets its own slightly different
  lens. The hub is given none of these numbers.
- **Built in is not exact.** The simulated devices differ from their drivers' descriptions by realistic
  errors, set in the scene file: a servo zero a few ticks out, a wheel 1 % larger, play in the head's
  slide, a tag printed 1 to 2 mm off. The hub has to notice and correct.
- **Calibration without a prepared target.** The devices' own tags are the targets: the arm shows its
  wrist tag to a camera at several poses; the rover's head sweeps past a tag. No board placed by a person.
- **Every camera has delay and drops**, through the camera firmware build or the quick port.
- **The same artwork**: the tag images the simulation shows are the files that get printed.

## 6. Steps, in order

Tests are written with each step, in the repo's style (properties, checked against the referee). They
are run when the user asks; step 2 is asked for already, and only its own small suite.

| # | Step | Size | Done when |
|---:|---|---|---|
| **1** | **Scenes as files.** The scene format, the loader that builds the model in memory, the lip, validation. `workcell` converted, and `open_small` as the first new place. No API, no hub, no rover | Small | Both scenes load at run time and validate; the converted `workcell` is the same model as today's apart from the lip |
| 2 | **Environments, tasks, and the operator API.** The rest of 5.7's environments and its tasks as files; scene listing, editing, loading and validation over REST; tasks as data with the referee's checks. **Created and validated only: no rover, no hub, nothing run but the scene suite** | Medium | Every scene and task passes validation; an invalid edit is refused with its reason; a scene can be swapped while the world runs |
| 3 | **Markers, and locating cameras from the arm's own tags.** Tag files; a pair on the arm's base and one on the wrist holder; detection in full-size stills; each fixed camera's pose relative to the arm from pictures alone | Small | In every scene with an arm and a camera the estimate is within a stated error; a mirrored or rotated tag fails; the flipped reading is caught; the code holds no number about any scene |
| 4 | **The lens.** The lens and sensor model of 5.10 on every camera | Small | Straight edges are visibly curved; step 3 still passes once the hub corrects for the lens from nominal values, and by how much it got worse is recorded |
| 5 | **The hub.** Top-level `hub/`, started from ports alone; discovery; the tree of interests; arm and cameras as devices; the model's API with no truth; the tests of 5.3. The old `sim_api.py` stays until the new one can do the same pick | Medium | A pick through the hub's API in several scenes; the blindfold and new-places tests pass |
| 6 | **Locator.** The pose problem of 5.4 with the arm contributing; `GET /where` | Medium | Poses within stated error across scenes; "not known" when the chain is broken; the uncertainty is honest |
| 7 | **The rover designed.** CAD by script; the simulation model from it with its contacts; the parts list chosen from real parts with links and prices | Large | The CAD checks pass; the model stands, rolls and steers in a bare scene without a controller's help; the head's axes reach their ranges without touching anything; `BOM.md` has a rover section |
| 8 | **The rover controlled.** Wheel mode in the emulator; the radio-link port; battery; drivers, filters and stop for wheels and head | Medium | Drives a square and reports where it thinks it is; the head goes where sent; it stops on `stop`, on a lost link, and before the battery is too low |
| 9 | **The start-up survey, the rover in the locator, and size** | Large | From where it is put down, without driving, the rover produces a map with sizes that match the referee's within a stated error, and finds the devices in view |
| 10 | **The map** (5.5) from every device; never-seen space; touch; landmarks as links; `GET /place` | Large | The found outline matches the referee's across scenes; nothing is reported as seen that was not |
| 11 | **Safety when nothing is known** (5.9) | Medium | In fuzzed runs the rover never climbs the lip and the arm never enters the rover's estimated space or hits an unseen thing hard |
| 12 | **Built in is not exact** (5.10), self-calibration | Medium | Everything passes with seeded device errors; calibration recovers them |
| 13 | **The tasks run, with a model driving** | Medium | Reference solutions through the hub's API for each task; model runs scored by the referee |
| 14 | Real hardware, device by device | | Each device's port swapped for the real one |

Step 1 is small on purpose and everything loads through it.

## 7. Decisions

### Decided

| What | Decision |
|---|---|
| Scenes | Config files, loadable at run time, editable over REST. TOML on disk, like the repo's other configs; the API speaks the same structure as JSON. An edit over the API rewrites the file and drops its comments |
| Vision library | OpenCV, AprilTag 36h11; tags 30 to 40 mm, on devices only; locate at full size |
| The network | Logical. Devices are grouped by interest in a hierarchy in the hub (my reading of this is O1) |
| Ground truth | Removed from the model's API, so the system has to orient |
| Keeping the rover on the table | A lip around the edge |
| Simulator | MuJoCo only, not Newton |
| Where the hub lives | A top-level package beside the simulation package |
| Rover power | Battery: the Gens ace 5000 mAh 3S pack. Data therefore by radio |
| Rover's cameras | Two, on a motorised head: spacing, tilt, pan, lift |
| Start-up | The rover surveys from where it stands before anything drives |
| Lens | Realistic distortion; pictures are not perfect |
| Environments and tasks | Several of each, created and validated before anything runs; only their own small suite is run |
| Rover build | CAD model, contacts and a real parts list, after the plan is agreed |

### Open

| # | Question | Options | My recommendation | Needed by |
|---|---|---|---|---|
| O1 | Is the tree of 5.2 what "hierarchical interest entities" means? | As drawn: devices and their parts, the place, and groups across devices, where asking a node reaches everything under it; or something else | As drawn | Step 5 |
| O2 | Which motion is the trusted ruler? | (a) the head's spacing drive; (b) the wheels, as agreed before the head was motorised; (c) the arm | (a): rigid, no slip, and it works during the stationary survey. The wheels become a check | Step 9 |
| O4 | Time | (a) turn-based, one action at a time, as now; (b) devices act at once | (a) first. Add actions that return at once when a task needs two devices moving together | Step 5 |
| O5 | How much may a device know about itself? | (a) its own body as its driver describes it, all trusted; (b) one trusted ruler (O2), the rest checked against it; (c) as (b), and the arm learns its own shape by watching itself; (d) nothing | (b). (c) is a research project. (d) is not possible without a new kind of sensor (5.3) | Step 3 |
| O6 | Head servos | HX-30HM throughout (known, heavy), or smaller bus servos for the head (lighter, a second kind to learn and maybe a second voltage) | Settle in step 7 with real parts in front of us | Step 7 |
| O7 | Where the lens numbers come from | Calibrate one of the real cameras on hand, or placeholders matched to the datasheet | Placeholders for step 4; replace as soon as a real camera has been calibrated | Step 4 |
| O8 | Does the survey start by itself? | The hub runs it when it finds a rover; or only when the model asks | By itself, and again on request | Step 9 |

Nothing open blocks step 1. O5 is the first needed, at step 3.

## 8. Risks and unknowns

- **This is a much larger project than the first version of this plan.** A motorised head triples the
  rover's servos, and mapping an unknown place is on the main path. The steps are ordered so that each
  is useful alone.
- **A sliding camera mount is hard to keep straight.** Play in the spacing slide turns the cameras a
  little relative to each other, and stereo is very sensitive to that. The hub will have to work out the
  cameras' relative angle from each pair of pictures and take only the distance from the encoder.
- **Top-heavy.** Head up and wide on a small base tips easily, on a bump or against the lip. The
  drive-only-when-lowered rule and the CAD's weight distribution have to hold.
- **Wires to parts that move four ways.** Power to two moving boards and the bus to the head's servos,
  through pan, lift and slide. The arm's wire simulation exists for exactly this.
- **Swapping a scene under a running hub** has to be clean: no reading from the old place may survive
  into the new map.
- **The rover's firmware does not exist.** Servos and bus over Wi-Fi is new code on a board, and unlike
  the camera firmware it sits in the control path: a late or lost packet is a rover that keeps driving.
  The board must stop the wheels by itself when it has heard nothing for a moment.
- **Servo replies over a radio link.** The bus driver waits 20 ms for a reply today. Wi-Fi adds delay
  that varies; the timing placeholders in `servos.toml [bus]` matter more for the rover than the arm.
- **A LiPo is the most hazardous part in the project.** It needs a balance charger, charging attended
  and off the rover, a fuse, and a lip that really holds: a punctured or crushed pack can burn. The
  listing does not say whether this pack has a hard case.
- **Real pictures are worse than simulated ones**, even through a lens model. Only hardware settles it.
- **Accuracy against what grasping needs.** One tag gives a few millimetres at half a metre in ideal
  pictures; a grasp wants a few millimetres in total. The wrist camera and touch remain the close-range checks.
- **Servo wheel mode is unverified**, like the rest of the register table.
- **Simulation speed.** Lens warping, byte-level buses and camera firmware for four or five cameras is
  slow; hence the two kinds of port.
- **The arm's code is single-arm throughout.** The plan avoids rewriting it, which leaves two styles
  side by side until someone unifies them.

## 9. Not in this plan

Policy training, the Newton backend, a second arm, and devices recognising each other without tags
(the step after markers: the same links, from the devices' shapes).
