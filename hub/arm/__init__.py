"""The arm's driver: what the hub knows about the SO-101 follower and how it moves it.

Everything here is about the arm itself (its joints, its shape, its servos), never about a place.
The arm's description files are read from where they are generated: `sim/model/so101_follower.xml`
(its shape, by sim/tools/build_model.py), `sim/config/servos.toml` and `sim/config/safety.toml`.
"""
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
JOINTS = ARM_JOINTS + ("gripper",)
