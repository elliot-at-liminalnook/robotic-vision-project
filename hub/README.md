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
  locate.py       a camera's pose relative to the arm, from the arm's tags
  vision/tags.py  tags in a picture; the pose of the rigid body that carries them
  errors.py       Refused: a request that will not be carried out, with the reason
  arm/            the arm's driver: its controller and everything the controller needs
    controller.py   goals, straight-line moves, governor, feed-forward, grip-by-offset, stall guard, stop path
    actions.py      move_tip, move_joints, gripper, wait: run until done, then say what happened and why
    device.py       the arm's nodes and commands in the tree
    link.py         the arm on its servo bus, through the command filter, one control tick at a time
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
/place                       what has been found of the surroundings (nothing yet)
```

Reading a node gives the state of everything under it; `stop` at a node freezes everything under it.

## The API (`api.py`)

| Request | What |
|---|---|
| `GET /` | The guide for the model, written from what was found. It says nothing about the place |
| `GET /tree`, `GET /{node}`, `GET /{node}/state` | The tree; a node with its commands and every parameter's range; what the sensors under it say |
| `GET /{camera}/picture` | JPEG. `?width=640` smaller; `?tags=true` JSON with the tags in view; `?base64=true` |
| `POST /{node}/{command}` | An action. Checked against the command's parameters first |
| `POST /{node}/stop`, `POST /stop` | Freeze everything under the node; everything |
| `GET /where?of=A&relative_to=B` | Where A is in B's frame, with uncertainty, or `known: false` and why. So far a fixed camera and the arm |
| `POST /wait`, `POST /discover` | Let time pass; look again for what is plugged in |

- No ground truth and no scene description: tests check that no name from the scene file appears in any answer,
  that the hub holds no reference to the world except through ports, and that changing what no sensor can
  tell changes no answer.
- A device that stops answering (eight position reads in a row fail, or a camera is gone) gives a 503 that
  says so and points at `/discover`. Never a stale answer.
- Time moves only inside an action, and a little for each picture (a frame period plus the transfer).

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
