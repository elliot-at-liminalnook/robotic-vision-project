# hub: what runs on the real machine

The hub commands the devices and works out where they are. It is started from ports (serial buses,
cameras, a clock) and nothing else, and it never imports the simulation. What it knows is what came
through a port and what each kind of device is; it is told nothing about the place it is in.
Design: [docs/MULTI_DEVICE_PLAN.md](../docs/MULTI_DEVICE_PLAN.md). Progress: [docs/MULTI_DEVICE_CHECKLIST.md](../docs/MULTI_DEVICE_CHECKLIST.md).

So far (step 3 of the plan):

| Module | What |
|---|---|
| `geometry.py` | Poses as 4x4 matrices. `T_a_b` takes a point in frame b to frame a |
| `builtin.py` | What each kind of device is: where its tags are on its own body (`hardware/tags/layout.toml`), a camera kind's nominal lens |
| `vision/tags.py` | `detect(image)`: the tags in a picture. `body_pose(...)`: the pose of the rigid body that carries them, from all of its tags together, with its uncertainty and whether the reading is ambiguous |
| `locate.py` | `camera_from_arm(image)`: a camera's pose in the arm's base frame from one picture of the arm's tag blocks |

```python
from hub import locate
fix = locate.camera_from_arm(picture)          # RGB array, any size; None if no base tag is in it
fix.T_base_cam, fix.sigma_m, fix.tags, fix.trusted
```

How well it does, in the simulation's ideal pictures (no lens model yet), camera 0.35 to 0.85 m from the arm:

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
  position here: what dominates is that a unit's focal length differs from the datasheet's (the camera
  in this check was 2.2 % long, which is 2.2 % of the distance). The hub has to find each camera's own
  values from what it sees (step 12 of the plan); until then, 6 % of the distance and 2.5 deg is what is stated.

- A fix is **trusted** only with two or more base tags that are not all in one plane, and a pose that
  explains them. Two front faces alone, or one tag, are reported as ambiguous with a very large
  uncertainty, not guessed.
- The stated uncertainty covered the true error in every case tried (the error was at most 1.2 times it).
- These numbers come from smoke checks while writing it; `sim/tests/test_markers.py` holds them as tests.
