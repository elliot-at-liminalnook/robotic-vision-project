# rover: a two-wheeled base with a motorised stereo head

Designed by script (`build_rover.py`, CadQuery), as the wrist mount was. The simulated rover is generated
from what the script writes, so the two cannot drift apart. Plan: [docs/MULTI_DEVICE_PLAN.md](../../docs/MULTI_DEVICE_PLAN.md), 5.6.

```bash
~/Projects/esp32-stereo-camera/work/cad-setup/venv/bin/python3 hardware/rover/build_rover.py     # about 20 s
```

| File | What |
|---|---|
| `build_rover.py` | The design: every dimension, the bought parts as boxes, the joints, the checks |
| `parts/<name>.step`, `.stl` | The twelve printed parts, each one solid, each in its own body's frame (mm) |
| `rover.json` | The assembly: bodies, joints, servo ids, masses and inertias, collision shapes, cameras. `sim/so101sim/rover_model.py` builds the simulated rover from it |
| `build_report.json` | The checks' results |

## What it is

| | |
|---|---|
| Drive | Two wheels (60 mm) at the back on HX-30HM servos in wheel mode (ids 11, 12); a ball caster at the front; a skid at the back, 1.5 mm clear, for when the head is turned right round |
| Head | Pan (id 21, ±170°), lift (id 22, −15° to 90°: an arm with a second, coupled joint that keeps the head level), tilt (id 23, −60° to +30°), camera spacing (id 24: one pinion between two racks, 12 mm of travel per radian) |
| Cameras | Two Camera Module 3 Wide, each with its ESP32-P4 board riding on its carriage |
| Lens height | 26 to 165 mm |
| Lens spacing | 60 to 200 mm |
| Size, stowed | 184 mm long in the chassis, 142 mm wide, 117 mm high; the stowed head reaches 245 mm ahead of the axle |
| Mass | 0.89 kg: battery 355 g, six servos 312 g, print about 165 g (at 40 % of solid PLA), boards and cameras 44 g |
| Battery | Gens ace 5000 mAh 3S, lying along the middle under the deck |
| Tags | Five (`../tags/layout.toml`): top, rear and both sides of the chassis, and one on the head |

Frame: origin on the surface under the middle of the wheel axle, x forward, y left, z up.

## Checks the script makes

- **One solid each**: all twelve parts.
- **Nothing meets anything** over the axes' travel: every pair of bodies at 93 poses (lift × tilt × spacing,
  and pan × lift × tilt × spacing against the chassis and wheels). It found four overlaps in the first
  draft (the arm's bars in the plane of the head's yoke, the yoke's plates through the carriages, the
  pinion into a rack, the head block under the yoke at full down-tilt); the design changed until none were left.
- **Weight between the wheels and the caster**: centre of mass 68 mm ahead of the axle stowed, 45 mm raised,
  32 mm ahead and 37 mm to the side with the head turned 90° and the cameras apart (inside the support
  triangle by 9 mm). Turned 170° it is 5 mm behind the axle and rests on the skid.
- **Leads**: the farthest apart the two ends of each servo lead get, with 25 % slack and 30 mm for plugs,
  is within the 200 mm lead a servo ships with. The lift-to-tilt lead is the tight one (192 mm).

## What is not done

- **It is geometry, not a finished mechanical design.** No screw holes or bosses at the joints, no bearing
  for the arm's pins, no print tolerances, no tooth form on the pinion and racks (drawn at the root
  diameter and the rack faces), no cable routing. Nothing has been printed.
- **Collision shapes are simple boxes** written next to the geometry (and an ellipsoid per wheel, a sphere
  for the caster and the skid), not derived from the meshes. The chassis's is its whole envelope.
- **Tilted down while stowed, the head goes 42 mm below the surface**: lift and tilt limit each other. The
  rover's command filter has to keep the head off the ground (plan step 8).
- **The stowed head is the rover's bumper**: it reaches 110 mm past the caster. Driven at the lip with the
  head down, the camera carriages meet it first.
- **Lens height stops at 165 mm**, short of the plan's 220 mm. A longer arm is the change; it moves the
  weight forward and needs the checks run again.
- **Top-heavy to the side**: 9 mm of margin with the head turned 90° and the cameras apart. Hence the plan's
  rule that it drives only with the head stowed.
- **Wheel servos share the servo's 345:1 gear**: top speed about 0.18 m/s at 12 V.
