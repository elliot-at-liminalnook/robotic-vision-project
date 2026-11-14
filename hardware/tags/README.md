# tags: the printed markers on each device

Each device carries AprilTag 36h11 tags so that other devices' cameras can find it. Where they are is part
of what the device is: `layout.toml` is the one place it is written down. The hub's driver reads it
(`hub/builtin.py`), and the simulated arm is generated from it (`sim/tools/build_model.py`), so the
simulated tag is the printed tag in the place the hub believes it to be.

| File | What |
|---|---|
| `layout.toml` | Per device: each tag's id, size, the part it is on, its centre, the way it faces and which edge is up; and the parts that carry tags |
| `make_tags.py` | Makes the artwork from the family's bits: `png/` (what the simulation shows) and `print/<device>.svg` (A4, true size) |
| `png/tag36h11_<id>.png` | One per tag: 10 x 10 cells, the 8-cell tag with a 1-cell white border |
| `print/arm.svg` | The arm's nine tags. Print at 100 %; a 50 mm line on the sheet is there to measure |

## The arm (ids 0 to 8)

- **Two tag blocks**, 38 mm cubes, one each side of the base on the same surface, 83 mm from its centre
  line. Each has a 30 mm tag on its top, front, back and outer face. From any direction a camera sees at
  least two tags of the one rigid body, on different faces. That matters: a single flat tag can be read
  tilted the wrong way.
- **One 24 mm tag on a cover plate** over the back of the wrist camera's board. It faces away from the
  lens, so with the fingers down it looks up and outward.

The blocks and the plate are parts of the arm's model, so the arm's command filter keeps clear of them.

Not yet made: the printable blocks and plate themselves (plain boxes; a 38 mm cube and a 30 x 30 x 1 mm
plate), and a check on a printed arm that nothing fouls them. The plate adds 1.5 mm to the outside of the
wrist camera assembly, which the wrist-mount placement study has not been re-run with.
