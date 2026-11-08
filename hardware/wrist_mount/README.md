# wrist_mount: wrist camera on the follower's gripper

The camera holder (`../camera_mount`) on a bracket on the gripper's +y side. The
camera looks at the point between the jaw tips, seeing both jaws side-on with the
object between them.

## Placement (place_camera.py → wrist_mount_pose.json)

| | |
|---|---|
| Lens (gripper body frame) | x 2, y 66, z −25 mm; looking at (2, 0, −92) mm, between the jaw tips |
| Holder orientation | Top band toward the wrist; lens axis tilted ~45° toward the jaws |
| ESP32-P4 board | On the holder's back, so the camera ribbon crosses **no** joints; only the round USB cable runs down the arm |
| Added mass | ~46 g (bracket 14, holder 16, camera 4, board 12) |
| Camera-safe wrist roll | −2.74 … +0.80 rad (spans π, so every top-down jaw orientation stays available) |

Scored in MuJoCo on the generated follower model. The chosen pose:
- has the tips and grasp centre ≥ 162 px inside the 640×360 wrist image (jaws open and closed);
- doesn't overlap the gripper or the moving jaw anywhere in the jaw's range;
- clears the table by 44 mm at a straight-down grasp;
- has zero self-collisions over 200 random reachable grasp poses within the roll range.

Closer-in placements collided with the wrist link in 47–97 % of grasp poses. Large positive wrist roll near
the base swings the holder into the shoulder, which is why the roll range is capped.

## Bracket (build_wrist_mount.py → wrist_bracket.step/.stl)

- **Gripper side:** upstream's SO-101 wrist-camera flange, cut from its STEP and placed by
  `register_upstream_mount.py` (0.06 mm RMS fit). Two M3 × 8 screws go into the hex nuts in the gripper's
  wrist-roll part, exactly as upstream's mount, with its locating key. Thickened to 6 mm.
- **Holder side:** a pad on the **front** of the holder's top band with two **M3 heat-set inserts**. The holder
  bolts on from its open back with M3 × 12. (The flange sits in front of the holder, so a back pad would have
  forced the arm through the holder.)
- **Arm:** ruled loft between the two; 6.4 mm channels give screwdriver access to the gripper screws.
- **Size:** 28 cm³ (~35 g solid, ~14 g at typical infill).
- **Checks:**
  - one solid;
  - no overlap with the holder, the board envelope, or the gripper (y < 24 mm, apart from upstream's key);
  - no bracket vertex visible in the wrist camera image.

```bash
# placement (MuJoCo env), then CAD (CadQuery env)
python hardware/wrist_mount/place_camera.py
~/Projects/esp32-stereo-camera/work/cad-setup/venv/bin/python3 hardware/wrist_mount/build_wrist_mount.py
python sim/tools/build_model.py      # puts the assembly + camera into the sim
```

`assembly/` holds the bracket, holder and board envelope in the gripper frame (mm), which the sim generator uses.

## Hardware

| Part | Qty |
|---|---|
| M3 × 8 screw + M3 hex nut (bracket → gripper, nuts in the gripper's recesses) | 2 |
| M3 heat-set insert (bracket pad) | 2 |
| M3 × 12 screw (holder → bracket) | 2 |

## Not yet designed / verified

- A printed cradle for the ESP32-P4 on the holder's back. The sim models it as a 71 × 21 × 15 mm envelope with its
  mass.
- USB cable routing down the arm; check it with the arm moving.
- Printed fit, stiffness (wobble at the camera), and the real sensor's image orientation. Set the firmware's
  hflip/vflip so that image-up = the holder's top band, as in the sim.
