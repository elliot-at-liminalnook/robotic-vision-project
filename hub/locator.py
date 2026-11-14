"""Where the devices are relative to each other, worked out from what their sensors have seen.

There is no world frame and nothing is given: every answer is one device's frame in another's,
with an uncertainty, an age and what supports it; or "not known" when no chain of links joins them.

Links so far (docs/MULTI_DEVICE_PLAN.md, 5.4):

  camera -> arm          every sighting of the arm's tags by a fixed camera: the tag blocks beside
                         its base, and the tag behind its wrist camera at each pose the arm showed
                         it in. The wrist tag's place comes from the arm's own joint angles and
                         shape, so the arm contributes even when its base is out of the camera's view.
                         All sightings by one camera are solved together, as points known in the
                         arm's frame against where they fell in the pictures: one small flat tag
                         alone can be read tilted the wrong way, several poses cannot.
  arm -> its own parts   the wrist camera and the gripper, from the joint angles, now.

Links chain: a camera is related to another camera through the arm, if both have seen it.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from . import geometry as G
from .arm.kinematics import FK_SIGMA_M, FK_SIGMA_RAD, ArmKinematics
from .builtin import LENS_FOCAL_TOLERANCE, Lens
from .vision import tags as V

MAX_RMS_PX = 4.0               # a pose that explains the corners worse than this (at 1920 wide) is not trusted
MOVED_PX = 12.0                # new sightings this far from where the old pose puts them: the camera has moved


@dataclass
class Link:
    """Frame b in frame a (T_a_b), with how well it is known."""
    T: np.ndarray
    cov: np.ndarray            # 6 x 6: rotation (rad) then position (m), in frame a
    t: float                   # when the newest sighting behind it was made
    support: list[str]

    # The uncertainty is of a small turn (about frame a's axes) and a small shift (along them) applied to T.
    def inverse(self) -> "Link":
        R, t = self.T[:3, :3], self.T[:3, 3]
        J = np.zeros((6, 6))
        J[:3, :3] = -R.T
        J[3:, :3] = -R.T @ _skew(t)
        J[3:, 3:] = -R.T
        return Link(G.inv(self.T), J @ self.cov @ J.T, self.t, self.support)

    def then(self, other: "Link") -> "Link":
        """self = a -> b, other = b -> c: the chain a -> c."""
        R = self.T[:3, :3]
        Ja = np.eye(6)
        Ja[3:, :3] = -_skew(R @ other.T[:3, 3])              # a turn of b swings c about it
        Jb = np.zeros((6, 6))
        Jb[:3, :3], Jb[3:, 3:] = R, R
        cov = Ja @ self.cov @ Ja.T + Jb @ other.cov @ Jb.T
        return Link(self.T @ other.T, cov, min(self.t, other.t), self.support + other.support)

    @property
    def sigma_m(self) -> float:
        return float(np.sqrt(np.trace(self.cov[3:, 3:])))

    @property
    def sigma_rad(self) -> float:
        return float(np.sqrt(np.trace(self.cov[:3, :3])))


def _skew(v) -> np.ndarray:
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]], float)


@dataclass
class Sighting:
    """One tag in one picture: its corners in the arm's frame (as the arm knew them then) and in the picture."""
    tag: int
    corners_arm: np.ndarray    # 4 x 3
    corners_px: np.ndarray     # 4 x 2, in full-size pixels
    t: float
    moving_part: bool          # on a part that moves with the joints (its place came from the joint angles)
    view: int                  # which picture


@dataclass
class CameraTrack:
    """Everything one fixed camera has seen of the arm, and the pose that explains it."""
    lens: Lens
    sightings: list[Sighting] = field(default_factory=list)
    views: int = 0
    link: Link | None = None           # arm in the camera's frame... kept as camera -> arm
    why_not: str = "it has not seen any of the arm's tags"
    notes: list[str] = field(default_factory=list)


class Locator:
    def __init__(self, clock):
        self.clock = clock
        self.kin = ArmKinematics()
        self.cameras: dict[str, CameraTrack] = {}
        self.has_arm = False

    def reset(self, has_arm: bool, cameras: dict[str, Lens]):
        """Start again: nothing seen yet. cameras: name -> the nominal lens of each fixed camera at full size."""
        self.has_arm = has_arm
        self.cameras = {name: CameraTrack(lens) for name, lens in cameras.items()}

    # ---- taking in what was seen ---------------------------------------------
    def see(self, camera: str, image: np.ndarray, t: float, arm_q=None) -> dict:
        """A fixed camera's full-size picture. arm_q: the arm's joint angles when it was taken (the arm at rest).
        Returns what was made of it."""
        track = self.cameras[camera]
        found = V.detect(image)
        if not self.has_arm or arm_q is None:
            return {"tags_seen": [d.id for d in found], "used": []}
        corners = self.kin.tag_corners(arm_q)
        fixed = self.kin.fixed_tags()
        new = [Sighting(d.id, corners[d.id], d.corners, t, d.id not in fixed, track.views) for d in found if d.id in corners]
        track.views += 1
        if new and track.link is not None and self._moved(track, new):
            track.notes.append(f"at {t:.1f} s the arm's tags were not where this camera's pose put them: it (or the arm) has been "
                               "moved, and what was seen before was dropped")
            track.sightings = []
        # A tag that does not move, seen again, adds nothing new: keep its newest sighting only.
        seen_again = {s.tag for s in new if not s.moving_part}
        track.sightings = [s for s in track.sightings if s.moving_part or s.tag not in seen_again] + new
        self._solve(track)
        return {"tags_seen": [d.id for d in found], "used": [s.tag for s in new]}

    def _moved(self, track: CameraTrack, new: list[Sighting]) -> bool:
        import cv2
        T = track.link.T                                    # camera -> arm
        obj = np.vstack([s.corners_arm for s in new if not s.moving_part] or [np.zeros((0, 3))])
        if len(obj) == 0:
            return False
        img = np.vstack([s.corners_px for s in new if not s.moving_part])
        proj, _ = cv2.projectPoints(obj, cv2.Rodrigues(T[:3, :3])[0], T[:3, 3], track.lens.K(), np.asarray(track.lens.dist, float))
        return float(np.sqrt(np.mean((proj.reshape(-1, 2) - img) ** 2))) > MOVED_PX

    def _solve(self, track: CameraTrack):
        track.link = None
        if not track.sightings:
            track.why_not = "it has not seen any of the arm's tags"
            return
        found = [V.Detection(k, s.corners_px) for k, s in enumerate(track.sightings)]
        fix = V.body_pose(found, {k: s.corners_arm for k, s in enumerate(track.sightings)}, track.lens)
        tags = sorted({s.tag for s in track.sightings})
        poses = len({s.view for s in track.sightings if s.moving_part})
        if fix is None:
            track.why_not = "no pose explains what it saw"
        elif fix.ambiguous:
            track.why_not = (f"what it saw (tag{'s' if len(tags) > 1 else ''} {tags}) lies in one plane, so the reading could be tilted "
                             "either way; another tag, or the wrist tag at another pose, would settle it")
        elif len(track.sightings) < 2:
            track.why_not = f"it has seen one tag ({tags[0]}) once: too little to trust"
        elif fix.rms_px > MAX_RMS_PX * track.lens.width / 1920:
            track.why_not = f"the best pose misses what it saw by {fix.rms_px:.1f} px: the sightings do not agree"
        else:
            cov = fix.cov.copy()
            # What many sightings cannot average away: this lens is not the datasheet's. Until the camera's own
            # values are found, every distance through it is off by about LENS_FOCAL_TOLERANCE of itself.
            reach = float(np.linalg.norm(fix.T_cam_body[:3, 3]))
            cov[3:, 3:] += np.eye(3) * (LENS_FOCAL_TOLERANCE * reach) ** 2
            cov[:3, :3] += np.eye(3) * math.radians(0.4) ** 2
            if any(s.moving_part for s in track.sightings) and not any(not s.moving_part for s in track.sightings):
                # Resting on the wrist tag alone: the arm's own knowledge of where its wrist is limits it.
                cov[:3, :3] += np.eye(3) * FK_SIGMA_RAD ** 2 / max(poses, 1)
                cov[3:, 3:] += np.eye(3) * FK_SIGMA_M ** 2 / max(poses, 1)
            base = sorted({s.tag for s in track.sightings if not s.moving_part})
            support = ([f"tags {base} on the arm's base"] if base else []) + \
                      ([f"the wrist tag at {poses} pose{'s' if poses != 1 else ''} of the arm"] if poses else [])
            track.link = Link(fix.T_cam_body, cov, max(s.t for s in track.sightings), support)
            track.why_not = ""

    # ---- answering -------------------------------------------------------------
    def names(self) -> list[str]:
        return (["arm", "arm/wrist_camera", "arm/gripper"] if self.has_arm else []) + sorted(self.cameras)

    def _to_arm(self, name: str, arm_q) -> tuple[Link | None, str]:
        """(arm -> name, why not if not known)."""
        now = float(self.clock.now())
        if name == "arm":
            return Link(np.eye(4), np.zeros((6, 6)), now, []), ""
        if name in ("arm/wrist_camera", "arm/gripper"):
            T = self.kin.wrist_camera(arm_q) if name.endswith("camera") else self.kin.part(arm_q, "gripper")
            cov = np.diag([FK_SIGMA_RAD ** 2] * 3 + [FK_SIGMA_M ** 2] * 3)
            return Link(T, cov, now, ["the arm's joint angles and its own shape"]), ""
        track = self.cameras[name]
        if track.link is None:
            return None, f"{name}: {track.why_not}"
        link = track.link.inverse()
        return Link(link.T, link.cov, link.t, [f"{name} saw " + " and ".join(track.link.support)]), ""

    def where(self, of: str, relative_to: str, arm_q=None) -> dict:
        """`of`'s frame in `relative_to`'s, or known = False and why."""
        known = self.names()
        for n in (of, relative_to):
            if n not in known:
                raise KeyError(n)
        out = {"ok": True, "of": of, "relative_to": relative_to}
        if not self.has_arm:
            return {**out, "known": False, "why": "nothing links these two: the only links so far go through an arm's tags, "
                                                  "and there is no arm"}
        a, why_a = self._to_arm(relative_to, arm_q)
        b, why_b = self._to_arm(of, arm_q)
        if a is None or b is None:
            return {**out, "known": False, "why": "no chain of sightings links them. " + " ".join(w for w in (why_a, why_b) if w)}
        link = a.inverse().then(b)
        T = link.T
        support = list(dict.fromkeys(link.support))
        return {**out, "known": True,
                "position_m": [round(float(v), 4) for v in T[:3, 3]],
                "axes": {k: [round(float(v), 4) for v in T[:3, i]] for i, k in enumerate("xyz")},
                "uncertainty_m": round(link.sigma_m, 4), "uncertainty_deg": round(math.degrees(link.sigma_rad), 2),
                "age_s": round(float(self.clock.now()) - link.t, 3),
                "support": support,
                "frames": {"position_m": f"the origin of {of}'s frame, in {relative_to}'s frame",
                           "axes": f"the directions of {of}'s x, y and z axes, in {relative_to}'s frame",
                           "arm": "origin at its base on the surface it stands on, x forward, y left, z up",
                           "a camera": "x to the picture's right, y down the picture, z the way it looks"}}

    def summary(self) -> dict:
        return {name: ({"located": True, "uncertainty_m": round(t.link.sigma_m, 4), "from": t.link.support}
                       if t.link is not None else {"located": False, "why": t.why_not}) | ({"notes": t.notes} if t.notes else {})
                for name, t in self.cameras.items()}
