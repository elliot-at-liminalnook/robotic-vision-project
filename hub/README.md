# hub: what runs on the real machine

The hub commands the devices and works out where they are. It is started from ports (serial buses,
cameras, a clock) and nothing else, and it never imports the simulation. What it knows is what came
through a port and what each kind of device is; it is told nothing about the place it is in.
Design: [docs/MULTI_DEVICE_PLAN.md](../docs/MULTI_DEVICE_PLAN.md). Progress: [docs/MULTI_DEVICE_CHECKLIST.md](../docs/MULTI_DEVICE_CHECKLIST.md).

```bash
python sim/scripts/run_system.py --scene open_small     # a simulated world, the hub on its ports, both APIs
curl http://127.0.0.1:8765/                             # the hub's API: the guide to give the model
curl http://127.0.0.1:8765/tree
curl -X POST http://127.0.0.1:8765/devices/arm/move_tip -d '{"x": 0.2, "y": 0.0, "z": 0.1}'
curl "http://127.0.0.1:8765/where?of=camera_1&relative_to=arm"
```

## How it is put together

```
hub/
  ports.py        what a port is: a serial port, a camera port, a clock. The only way in or out
  discovery.py    which servo ids answer on each serial port, and which kinds of device they make
  tree.py         the tree of interests: nodes, commands, state, stop
  hub.py          Hub(provider): discover, build the tree, wait, stop, where, shutdown
  api.py          the REST API over the tree, and the guide written for the model
  camera.py       a camera: pictures through its port, the nominal lens of its kind
  builtin.py      what each kind of device is: where its tags are, a camera kind's nominal lens
  geometry.py     poses as 4x4 matrices
  locate.py       a camera's pose relative to the arm from one picture (the first, single-picture version)
  locator.py      where devices are relative to each other: links, chains, uncertainty, age, support
  vision/tags.py  tags in a picture; the pose of the rigid body that carries them
  errors.py       Refused: a request that will not be carried out, with the reason
  arm/            the arm's driver: its controller and everything the controller needs
    controller.py   goals, straight-line moves, governor, feed-forward, grip-by-offset, stall guard, stop path
    actions.py      move_tip, move_joints, gripper, wait: run until done, then say what happened and why
    device.py       the arm's nodes and commands in the tree
    link.py         the arm on its servo bus, through the command filter, one control tick at a time
    kinematics.py   where the arm's tags, wrist camera and gripper are, from its joint angles
    ik.py dynamics.py governor.py command.py motion.py servo.py servo_model.py safety.py grasp.py config.py
  bus/            the servo bus: packets, the register table, the checked driver, BusArm
```

The arm's code moved here from `sim/so101sim` (the old names still work: they are the same modules).
The arm's description files are still read from where they are generated: `sim/model/so101_follower.xml`,
`sim/config/servos.toml`, `sim/config/safety.toml`. Reading the arm's own description is allowed; nothing
about a scene is read.

## What it does at start-up

1. **Asks what is plugged in** (`provider()`), and scans each serial port for servo ids 1 to 30.
2. **Matches ids to kinds of device.** Ids 1 to 6 all answering is an arm. The number of devices is not given.
3. **Takes the arm over** holding the pose it is in: identify (reads only), set the few registers relied on,
   goal = where it is, torque on (both read back).
4. **Finds which camera is on the wrist** by turning the wrist 0.3 rad and back: that camera's picture changes,
   the others' barely do. Cameras say only what they are, never where they are mounted.
5. **Builds the tree.** Devices by the names the hub gives them (`arm`, `camera_1`, ...), their parts, and groups.

```
/devices/arm                 move_tip, move_joints, home, torque_off, torque_on
/devices/arm/joints          (state)
/devices/arm/gripper         set
/devices/arm/wrist_camera    picture
/devices/camera_1            picture
/groups/cameras              every camera, wherever it is mounted
/groups/movers               everything that can move itself
/place                       what has been found of the surroundings: nothing until something has looked
```

`/devices/arm` also has `probe` (touch).

With a rover there is also `/devices/rover/...` (below) and a root command `survey`.

Reading a node gives the state of everything under it; `stop` at a node freezes everything under it.

## The API (`api.py`)

| Request | What |
|---|---|
| `GET /` | The guide for the model, written from what was found. It says nothing about the place |
| `GET /tree`, `GET /{node}`, `GET /{node}/state` | The tree; a node with its commands and every parameter's range; what the sensors under it say |
| `GET /{camera}/picture` | JPEG. `?width=640` smaller; `?tags=true` JSON with the tags in view; `?base64=true` |
| `POST /{node}/{command}` | An action. Checked against the command's parameters first |
| `POST /{node}/stop`, `POST /stop` | Freeze everything under the node; everything |
| `GET /where?of=A&relative_to=B` | Where A is in B's frame, with uncertainty, age and support, or `known: false` and why |
| `POST /locate` | Look afresh; `show_wrist_tag` has the arm show its wrist tag at several poses |
| `POST /wait`, `POST /discover` | Let time pass; look again for what is plugged in |

- No ground truth and no scene description: tests check that no name from the scene file appears in any answer,
  that the hub holds no reference to the world except through ports, and that changing what no sensor can
  tell changes no answer.
- A device that stops answering (eight position reads in a row fail, or a camera is gone) gives a 503 that
  says so and points at `/discover`. Never a stale answer.
- Time moves only inside an action, and a little for each picture (a frame period plus the transfer).

## The rover (`rover/`)

| Module | What |
|---|---|
| `rover/description.py` | The rover's own shape from `hardware/rover/rover.json`: where its cameras, tags and lowest point are at any head pose; wheel radius and track |
| `rover/device.py` | Wheels with odometry, the head, power; their commands, protections and place in the tree |
| `bus/group.py` | Any set of servos on a bus by id: position servos and motor-mode servos side by side |

```
/devices/rover/drive     forward {distance_m}, turn {angle_deg}        state: where its wheels say it has gone
/devices/rover/head      point {pan_deg, tilt_deg}, raise {lens_height_m}, spread {lens_spacing_m}, stow
/devices/rover/head/left, /right                                       picture
/devices/rover/power                                                   state: volts, volts per cell, low
```

- **Found** by its six servo ids (11, 12, 21 to 24) on a port; that port is a radio link, so replies are
  allowed 50 ms longer. Its two cameras say they are the rover's and which side (its own boards carry its
  name: built in); they are not put through the wrist-camera test.
- **Wheels** run in the servo's motor mode. Distance and heading are counted from the wheel positions,
  which read within one turn, so the hub counts the turns. Speed 0.08 m/s, acceleration 0.25 m/s².
- **Protections:** it drives only with the head stowed; the head is never sent to where its own shape puts
  it within 6 mm of the surface; a head joint 12° behind for 0.5 s is held where it is; a wheel servo at
  its limit for 0.4 s stops the wheels; nothing moves below 3.3 V per cell; six reads lost in a row is
  "the rover has not answered". The rover's own board stops the wheels after 0.4 s without a word.
- **The rover is light.** A head that meets something can turn the whole rover instead of stopping, and
  the wheels do not count it; wheels that spin against something are counted as distance. Only looking
  tells. The lift, tilt and spacing servos are held to 35 % of their torque. The pan, which swings the
  head's whole length into places no camera has seen, is held to 5 %: against a post it stalls and is
  noticed, and the rover is turned 3 to 5° where at 35 % it was turned right round.
- `settle()` waits until the head has been still for 0.3 s. The two cameras are separate boards: anything
  measured across both pictures needs a head that is not still swinging.

Smoke-checked: 0.05 to 0.40 m drives within 2 % and counted within 1 %; turns within 4° and counted within
2.5 %; a 0.12 m square ends within 30 mm and 8° of where it began; every head command against the true
joint; the link cut mid-drive.

## The locator (`locator.py`)

Where devices are relative to each other, from what has been seen, each answer with an uncertainty,
an age and what supports it; or `known: false` and why.

| Link | From |
|---|---|
| fixed camera → arm | Every sighting by that camera of the arm's tags: the blocks beside its base, and the tag behind its wrist camera at each pose the arm showed it in. All of one camera's sightings are solved together, as points known in the arm's frame against where they fell in the pictures |
| arm → its wrist camera, its gripper | The joint angles and the arm's own shape (`arm/kinematics.py`), now |
| fixed camera → rover, arm's wrist camera → rover | The rover's five tags in that camera's picture: where the rover was then |
| rover → arm | The arm's tags in one of the rover's own pictures, through where the head's joints put that camera |
| rover → its two cameras | The head's joint angles and the rover's own shape (`rover/description.py`), now |
| anything → anything | Chained through the arm or the rover, by whichever chain is surest: two cameras with no arm between them are related through the rover both can see |

A sighting of the rover is of where it was. Its wheels carry the link forward to now, and the link
loosens by 3 % of the distance and of the angle they say it has gone since. A newer sighting is
preferred unless an older one is clearly surer. If the rover turns out to have been moved without its
wheels counting it (below), what it saw and what saw it during that time is dropped.

- `POST /locate` takes a fresh full-size picture on every fixed camera. With `{"show_wrist_tag": true}` the
  arm goes to its home and turns its wrist tag through six poses, with a picture at each: a camera that
  cannot see the arm's base is located that way.
- `GET /where?of=A&relative_to=B`: names are a device's name, `arm/wrist_camera`, `arm/gripper`,
  `rover/head/left`, `rover/head/right`. It answers from what has been seen so far; if nothing links the two
  it takes one fresh look first (every camera, the rover's included). Asked the other way round it gives
  the inverse through the same chain.
- `POST /locate` says for each fixed camera whether it is placed against the arm (or, with no arm, the
  rover) by any chain, and for the rover whether it is placed against the arm.
- **Not known** when: there is no arm; a camera has seen none of the arm's tags; all it saw lies in one
  plane (the reading could be tilted either way); or the sightings do not agree.
- **Moved:** if new sightings fall more than 12 px from where the old pose puts them, what was seen before
  is dropped and a note says so.
- **Uncertainty** is the fit's own, plus a floor many sightings cannot average away: each lens differs from
  its datasheet, so every distance through an uncalibrated camera is off by about 1.5 % of itself
  (`builtin.LENS_FOCAL_TOLERANCE`). Without that floor it stated 1 to 3 mm where the error was 6 to 17 mm.

Smoke-checked in the seven scenes with an arm and a camera and at eight seeded camera places with their
own lenses: errors 6 to 17 mm at 0.5 to 0.85 m, every one inside 3.5 times what was stated.

## The start-up survey, and the place (`survey.py`, `place.py`, `vision/stereo.py`)

`POST /survey` (or `Hub(provider, survey=True)`, `run_system.py --survey`): the rover looks all round
from where it stands. Its wheels do not move. About 75 s.

| Pass | Lens height | Tilt | Lens spacing | For |
|---|---|---|---|---|
| 1 | 0.10 m | 25° down | 60 mm | what stands under and between things |
| 2 | 0.16 m | 38° down | 60 mm | what is near, from above |
| 3 | 0.16 m | 12° down | 100 mm | further out |

In each pass the head turns outward from straight ahead in 40° steps, one way and then the other, and
at each stop, once it is still, both cameras take a full-size picture. From each pair:

- **Depth** (`vision/stereo.py`). The two lenses differ, so the rows of the two pictures are first lined
  up from the pictures themselves. Matching runs at half size and again at quarter size, where the
  same search reaches twice as near; where they disagree the coarser is believed. Only the middle of
  the picture (31° each side) and the nearer part (7 times the spacing: 0.42 m, 0.7 m) go into the map.
- **Level.** The flat ground in each cloud is found and heights are measured from it, not from what the
  head's joints say is level.
- **The map** (`place.py`): a 10 mm grid in the rover's frame as it stood. Each cell is seen free, seen
  occupied (with how high), or never seen. From it: the surface's seen area and outline; long straight
  thin runs (a raised edge has the surface on one side only); things, each with where, how big, how
  high; and which ways nothing has looked. `GET /place/state`, `GET /place/picture` (PNG).
- **The locator**: any arm tag in view relates the arm to the rover.
- **The rulers.** Everything is sized by the spacing drive. Checked against it: a tag's side measured
  across the pair over its printed side, and the lenses' height above the ground as the pair measures
  it over what the lift says. A ruler more than 4 % out is reported as disagreeing.

**The head swings into places no camera has looked at.** Its end is 0.22 m ahead of the pan axis and
leads the view. So the pan pushes with 5 % of its torque: against a post it stalls, the lag is
noticed, the survey does not turn that way again (at any height), and the rover has been turned
3 to 5°. At the end the first view is taken again and set against what it was: a picture that has slid
says how far the rover was turned (`stood_still`). Up to 8° the map is kept with a warning; beyond
that, or if the two pictures cannot be matched, the map and what the rover saw of other devices are
dropped, and the answer says so.

What it gives, smoke-checked against the referee (not known to the hub):

| Scene | Views used | Found |
|---|---|---|
| `no_arm` | 23 of 27 | All four raised edges, straight, 5 to 30 mm out; the cube and cylinder within 10 mm, heights within 2 mm; both cameras related through the rover |
| `bench` | 17 of 27 | The arm 5 mm from where it is (stated 16 mm); the stack of things at the right place; tags measure 1.016 of their printed size |
| `shelf`, `cluttered` | 9 to 14 | The rover stands beside tall things; the head meets one, the rover is turned 1.4° and 8°: a warning in one, the map dropped in the other |

What it does not do yet:

- **The pair is not calibrated.** The two lenses' centres are a few pixels apart, which shifts every
  disparity by the same amount: distances from the rover are 2 to 5 % long or short. Finding the
  shift from the spacing drive alone was tried and does not work with a printed rack (DECISIONS MD52).
  Step 12 does it from tags.
- **Very near things.** Nearer than about 2.7 times the spacing a tenth to a third of what is matched
  is matched wrongly. Cells whose points scatter below the surface are left as never seen.
- **Plain things.** A plain surface gives no depth. A plain straight edge gives depth only where it
  crosses the picture's rows at an angle; such an edge is placed within about 70 mm.
- **Behind the rover** (beyond 160° each way) and within about 0.15 m of it are never seen.
- A short 'edge of the surface' may be the hidden side of something standing on the surface.

## The map from every device (`place.py`, `landmarks.py`, `arm/actions.py`)

`GET /place` is what has been found, from everything that can find it. `GET /place?relative_to=arm` gives
every position in that device's own frame, if something relates the two (409 and why, if not).

| Device | What it adds |
|---|---|
| Rover | Depth. The start-up survey, and `POST /devices/rover/head/look`: one more pair of pictures with the head where it is, from wherever the rover has driven to. The map stays in the frame it was begun in; a later view is laid into it by what the wheels say the rover has done since (3 % of that is added to what is said of it) |
| Arm | Touch. `POST /devices/arm/probe {x, y}` lowers the tip at 30 mm/s until it stops following its command down, then lifts it back. It answers how high the gripper's lowest point stopped: within 5 mm of the thing's top (it reads a few millimetres high), or that the tip reached the surface. The thing is pressed lightly and not moved. A thing in the map that has been touched carries `touched_height_m` |
| Fixed cameras | No depth of their own. Once related to the rover, each thing carries where it is in each camera's picture (`in_picture`, full-size pixels), so the picture can be looked at there. They look again after every survey and every `look` |

- **Things** are occupied cells that touch and stand within 15 mm of each other's height: a hand hovering
  over a block is not the block. Each has where, how long and wide, how high, a colour (the mean of its
  own points, and the word for it), and `this_is: the arm itself, or something under it` when it lies
  where the arm's own shape is.
- **Edges of the surface** are long straight thin runs with the surface seen on one side only; they are
  not things. A straight run with the surface on both sides or neither is reported as the side of
  something and stays part of that thing.
- **Landmarks.** If no chain of tag sightings relates the arm to the rover, things the arm has touched
  are laid over the things in the map: a turn about the vertical and a shift along the surface, from two
  or more things matched by height and by how far apart they are. It then answers `/where` like a
  sighting does, and is carried forward by the wheels. It is refused, with the reason, when the touches
  fit the map about equally well in more than one way (two things of one height fit both ways round).

Smoke-checked: a cube seen again after the rover turned 20° and backed 60 mm stays within 35 mm and stays
one cube; on the bench the cylinder and peg come out within 30 mm in the arm's frame, and things are
within 45 px of where the map says in both cameras' pictures; with every tag link cut, two touches place
the arm 7 mm from where it is in the rover's frame (stated 83 mm).

Not done: the wrist camera adds no depth yet (two pictures from two arm poses would); the fixed cameras
say nothing about cells the rover has not seen.

## Safety from what has been found (`guard.py`)

Nothing here is a list of keep-outs: the hub is told nothing of the place. A motion is put to the map
before it is made.

| What the map says there | The rover's wheels | The arm's hand |
|---|---|---|
| Seen occupied | Not driven into. A drive stops 20 mm short and says so; if it cannot move at all it is refused (409) with what is in the way and where | Not passed through (409, with how high it is and how to go over). A move may *end* at a thing, or start at one: that is a touch or a grasp, made carefully |
| Never seen | At a crawl (25 mm/s, 0.3 rad/s) for the whole drive, if it crosses more than 12 cm² of it | Carefully |
| Seen free | As usual | As usual |
| No map at all | At a crawl | Carefully |

- **Carefully** means 80 mm/s and stopping at the first touch: when the tip's command has gone on 12 mm
  in 0.3 s and the tip has not followed, the arm holds where it is and says so, long before its stall
  guard would (which needs a joint 12° behind for 0.4 s, the arm's full push). The answer carries
  `careful` and why. *It does not feel a light loose thing*: a 30 g cube slides along with the hand.
- **The arm and the rover.** The rover's own shape, where the locator puts it in the arm's frame, grown
  by 30 mm and twice how unsure that place is, is closed to the hand up to the rover's height; so is a
  touch (`probe`) there. A hand already inside it may always move away. With nothing relating the two,
  the hand goes carefully wherever it is lower than the rover is tall (the plan had it stay above that
  height; then it could never touch the landmark that would relate them).
- **What the rover's cameras map of the arm is the arm**, at each pose it was seen in: those cells count
  as never seen for the hand, not as things.
- **Pushing.** A rover against something low with its tyres slipping is nowhere near its servos'
  limit, and its wheels go on counting. Once at a steady speed the hub learns what rolling takes; a
  load that stays 1 % of full above it for 0.5 s is pushing. The drive stops, what the wheels counted
  since the load rose is taken back, and what stopped it goes into the map just past the end of the
  rover that led. (Found when backing 0.30 m into the surface's edge counted 0.30 m and moved 0.10.)
  The numbers are from the simulation and must be set again on the real rover.
- **Where the rover stands is free**: its outline is marked so at every view and after every drive.
- **Stop** at the root freezes every device; under a device, only that one. Every way out of the
  program switches every servo's torque off.

Fuzzed, against the referee: thirty random drives and turns after a survey move no loose thing, never
leave the surface, and leave the wheels' count within 50 mm and 12° of where the rover is; forty random
tip targets never touch the rover and shove nothing.

## Self-calibration (`calibrate.py`)

The devices are not quite what their drivers believe: each lens differs from its datasheet, a printed
wheel is not its drawing's size, a printer may have scaled the tags. `POST /calibrate` finds what it can
from what the devices see of each other's tags, with no target put down by a person. Each part says
what it found, how sure it is, and what it changed; each can also be asked for alone.

| What | How | Command | In the simulation |
|---|---|---|---|
| A fixed camera's focal length | The arm shows the tag behind its wrist at several poses. The tags' places are known from the arm's joints; the focal length is the one that makes them all land where they were seen. Weighed against the datasheet by how sure each is | `POST /devices/camera_N/calibrate` (after `/locate` with `show_wrist_tag`) | Found to 0.1 to 0.5 % of itself from 7 or 8 sightings; stated as good to 0.5 to 0.9 % where the datasheet is good to 1.5 % |
| The rover's pair of cameras | The head looks at a tag while the spacing drive moves the cameras apart in five steps. The tag's disparity is a straight line in the spacing; what is left at no spacing is the pair's sideways shift. The ratio of the two focal lengths comes from how the rows of one picture run against the other's | `POST /devices/rover/head/calibrate` (head raised, pointed at a tag) | Shift found to 0.5 px (it was 3 to 10 px: 2 to 7 % of a distance at 0.4 m); focal ratio to 0.2 % |
| The rover's wheels | A fixed camera watches the rover drive up to 0.25 m and back, twice, over ground the map shows free. Weighed against the description (good to 3 %) | `POST /devices/rover/drive/calibrate` (after a survey) | Wheels 5 % large: taken as 4.6 to 4.7 % large afterwards. Wheels right: taken as 0.5 % large |

- After the pair is calibrated the survey and `look` use its shift and focal ratio, and the tags are
  measured again by the spacing drive: tags printed 4 % large measure 1.035 to 1.04 of their printed
  size and are reported as disagreeing.
- **Refused, or left out, rather than believed:** a camera that has seen the wrist tag at fewer than
  three poses; tags all at one distance; a disparity that is not a straight line in the spacing (one
  misread spacing is left out); a leg of the wheel drive that goes where nothing has looked, because
  what stops the rover there spoils the count (found: backing into the edge made the wheels look 5 %
  small).
- **Not found:** the lenses' centres and distortion (still the datasheet's); the zero of each servo; the
  arm's own link lengths.

## How well locating does

`locate.camera_from_arm(image, lens)`: a camera's pose in the arm's base frame from one picture of the arm's
tag blocks. In the simulation's ideal pictures, camera 0.35 to 0.85 m from the arm:

| Picture | Position error | Orientation error |
|---|---|---|
| 1920x1080 | 4 to 21 mm: within 3.5 % of the camera's distance | under 2 deg |
| 1280x720 | 7 to 52 mm: within 8 % of the distance | under 4 deg |

Through the lens model (`sim/so101sim/lens.py`: each camera's own distortion, darkening, blur and noise),
corrected with the camera kind's nominal values only, 1920x1080, the same twelve placements:

| | Median error | Worst error | Worst, as a share of distance | Error over stated uncertainty, worst |
|---|---|---|---|---|
| Ideal picture | 8 mm | 21 mm | 2.6 % | 1.2 |
| Through the lens, nominal correction | 18 mm | 37 mm | 4.2 % | 1.7 |
| Through the lens, no correction at all | 17 mm | 34 mm | 4.3 % | 2.0 |

- About twice as bad through the lens. Correcting with the nominal distortion makes no difference to
  position here: what dominates is that a unit's focal length differs from the datasheet's. The hub has to
  find each camera's own values from what it sees (step 12 of the plan); until then, 6 % of the distance and
  2.5 deg is what is stated.
- A fix is **trusted** only with two or more base tags that are not all in one plane, and a pose that
  explains them. Otherwise it is reported as not known, with the reason.
- These numbers come from smoke checks while writing it; `sim/tests/test_markers.py`, `test_lens.py` and
  `test_hub.py` hold them as tests. Those have not been run as suites.
