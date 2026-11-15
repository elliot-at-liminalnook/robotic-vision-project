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

  camera -> rover        a fixed camera, or the arm's wrist camera, seeing the rover's tags; and the
  rover -> arm           rover's own cameras seeing the arm's tags. The rover moves, so each such
                         sighting is of where it was then: its wheels carry the link forward to now,
                         with an uncertainty that grows with how far they say it has gone since.
  rover -> its cameras   the head's joint angles and the rover's own shape, now.

Links chain: a camera is related to another camera through the arm, or through the rover, if both
have seen it. The best chain (the surest) is the one answered with.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from . import geometry as G
from .arm.kinematics import FK_SIGMA_M, FK_SIGMA_RAD, ArmKinematics
from .builtin import LENS_FOCAL_TOLERANCE, Lens
from .rover.description import Rover
from .vision import tags as V

ODOMETRY_DRIFT = 0.03          # of the distance the wheels say, and of the angle: how much a link carried forward by them loosens
HEAD_SIGMA_M, HEAD_SIGMA_RAD = 0.003, math.radians(1.0)        # how well the rover knows where its cameras are from its head's joints

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
    focal_tolerance: float = LENS_FOCAL_TOLERANCE      # how far its focal length may be from what `lens` says, as a fraction


@dataclass
class RoverSighting:
    """The rover in some other device's frame, as it was at one moment."""
    other: str                 # "arm" or a fixed camera's name
    link: Link                 # other -> rover, then
    odometry: np.ndarray       # what the rover's wheels said its pose was then (x, y, heading)


def _se2(pose) -> np.ndarray:
    return G.pose(G.rot_z(float(pose[2])), [float(pose[0]), float(pose[1]), 0.0])


def _body_link(fix: V.BodyFix, t: float, support: str, tolerance: float = LENS_FOCAL_TOLERANCE) -> Link:
    """A tag sighting as a link camera -> body, with the floor an uncalibrated lens puts under its uncertainty."""
    cov = fix.cov.copy()
    reach = float(np.linalg.norm(fix.T_cam_body[:3, 3]))
    cov[3:, 3:] += np.eye(3) * (tolerance * reach) ** 2
    cov[:3, :3] += np.eye(3) * math.radians(0.4) ** 2
    return Link(fix.T_cam_body, cov, t, [support])


class Locator:
    def __init__(self, clock):
        self.clock = clock
        self.kin = ArmKinematics()
        self.rover = Rover()
        self.cameras: dict[str, CameraTrack] = {}
        self.has_arm = self.has_rover = False
        self.rover_seen: list[RoverSighting] = []
        self.rover_now = None                               # () -> (head joint values, odometry pose), when there is a rover

    def reset(self, has_arm: bool, cameras: dict[str, Lens], rover_now=None):
        """Start again: nothing seen yet. cameras: name -> the nominal lens of each fixed camera at full size.
        rover_now: a function giving the rover's head joint values and its wheels' pose, if there is a rover."""
        self.has_arm, self.has_rover, self.rover_now = has_arm, rover_now is not None, rover_now
        self.cameras = {name: CameraTrack(lens) for name, lens in cameras.items()}
        self.rover_seen = []

    # ---- taking in what was seen ---------------------------------------------
    def see(self, camera: str, image: np.ndarray, t: float, arm_q=None) -> dict:
        """A fixed camera's full-size picture. arm_q: the arm's joint angles when it was taken (the arm at rest).
        Returns what was made of it."""
        track = self.cameras[camera]
        found = V.detect(image)
        rover_used = self._rover_in_view(camera, found, track.lens, t)
        if not self.has_arm or arm_q is None:
            return {"tags_seen": [d.id for d in found], "used": rover_used}
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
        return {"tags_seen": [d.id for d in found], "used": [s.tag for s in new] + rover_used}

    # ---- the rover: seen, and seeing ------------------------------------------------
    def _rover_in_view(self, other: str, found, lens: Lens, t: float, T_other_cam: np.ndarray | None = None,
                       cam_cov: np.ndarray | None = None) -> list[int]:
        """The rover's tags in a picture taken by `other`'s camera (a fixed camera, or the arm's wrist camera at
        T_other_cam in the arm's frame). Records where the rover was then, if the tags settle it."""
        if not self.has_rover:
            return []
        head, odometry = self.rover_now()
        corners = self.rover.tag_corners(head)
        use = [d for d in found if d.id in corners]
        if not use:
            return []
        fix = V.body_pose(use, corners, lens)
        if fix is None or fix.ambiguous or fix.rms_px > MAX_RMS_PX * lens.width / 1920:
            return []
        tolerance = self.cameras[other].focal_tolerance if other in self.cameras else LENS_FOCAL_TOLERANCE
        link = _body_link(fix, t, f"{other} saw the rover's tags {sorted(d.id for d in use)}", tolerance)
        if T_other_cam is not None:                          # seen from a camera that itself rides on a device
            link = Link(T_other_cam, cam_cov, t, []).then(link)
        self.rover_seen.append(RoverSighting(other, link, np.array(odometry, float)))
        del self.rover_seen[:-200]
        return [d.id for d in use]

    def new_lens(self, camera: str, lens: Lens, tolerance: float):
        """This camera's own lens has been found (hub/calibrate.py): everything it saw is solved again with it."""
        track = self.cameras[camera]
        track.lens, track.focal_tolerance = lens, tolerance
        self._solve(track)
        self.rover_seen = [s for s in self.rover_seen if s.other != camera]      # (they were placed with the old lens)

    def forget_rover(self, since: float = -math.inf):
        """Drop what was seen of the rover, or by it, from `since` on: it has been moved without its wheels counting it."""
        self.rover_seen = [s for s in self.rover_seen if s.link.t < since]

    def rover_placed(self, other: str, T_other_rover: np.ndarray, sigma_m: float, sigma_rad: float, support: str):
        """The rover placed in another device's frame by something other than a tag in a picture (things both have found:
        hub/landmarks.py). Kept like a sighting: of where the rover is now, carried forward by its wheels afterwards."""
        if not self.has_rover:
            return
        _, odometry = self.rover_now()
        cov = np.diag([sigma_rad ** 2] * 3 + [sigma_m ** 2] * 3)
        self.rover_seen.append(RoverSighting(other, Link(np.asarray(T_other_rover, float), cov, float(self.clock.now()), [support]),
                                             np.array(odometry, float)))

    def see_from_wrist(self, found, t: float, arm_q, lens: Lens) -> list[int]:
        """The arm's wrist camera's picture: is the rover in it?"""
        if not self.has_arm or arm_q is None:
            return []
        cov = np.diag([FK_SIGMA_RAD ** 2] * 3 + [FK_SIGMA_M ** 2] * 3)
        return self._rover_in_view("arm", found, lens, t, self.kin.wrist_camera(arm_q), cov)

    def see_from_rover(self, side: str, found, t: float, head: dict, arm_q, odometry, lens: Lens) -> list[int]:
        """One of the rover's own cameras' pictures: are the arm's tags in it?"""
        if not (self.has_arm and self.has_rover) or arm_q is None:
            return []
        corners = self.kin.tag_corners(arm_q)
        use = [d for d in found if d.id in corners]
        if not use:
            return []
        fix = V.body_pose(use, corners, lens)
        if fix is None or fix.ambiguous or fix.rms_px > MAX_RMS_PX * lens.width / 1920:
            return []
        cam = Link(self.rover.camera(side, head), np.diag([HEAD_SIGMA_RAD ** 2] * 3 + [HEAD_SIGMA_M ** 2] * 3), t, [])
        rover_to_arm = cam.then(_body_link(fix, t, f"the rover's {side} camera saw the arm's tags {sorted(d.id for d in use)}"))
        self.rover_seen.append(RoverSighting("arm", rover_to_arm.inverse(), np.array(odometry, float)))
        del self.rover_seen[:-200]
        return [d.id for d in use]

    def _rover_from(self, other: str) -> Link | None:
        """other -> rover, now: the surest sighting of the rover by (or of) `other`, carried forward by its wheels."""
        if not self.has_rover:
            return None
        _, odometry = self.rover_now()
        best = None
        for s in reversed(self.rover_seen):                   # newest first: an older sighting must be clearly surer to be used
            if s.other != other:
                continue
            step = G.inv(_se2(s.odometry)) @ _se2(odometry)
            went, turned = float(np.linalg.norm(step[:2, 3])), abs(G.yaw_of(step))
            cov = np.zeros((6, 6))
            if went > 1e-4 or turned > 1e-4:
                cov[:3, :3] = np.eye(3) * (ODOMETRY_DRIFT * turned + math.radians(0.5)) ** 2
                cov[3:, 3:] = np.eye(3) * (ODOMETRY_DRIFT * went + 0.002) ** 2
            moved = Link(step, cov, s.link.t, ["its wheels since then"] if went > 1e-4 or turned > 1e-4 else [])
            link = s.link.then(moved)
            if best is None or link.sigma_m < 0.9 * best.sigma_m:
                best = link
        return best

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
            cov[3:, 3:] += np.eye(3) * (track.focal_tolerance * reach) ** 2
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
        return ((["arm", "arm/wrist_camera", "arm/gripper"] if self.has_arm else []) + sorted(self.cameras)
                + (["rover", "rover/head/left", "rover/head/right"] if self.has_rover else []))

    def _part(self, name: str, arm_q) -> tuple[str, Link | None]:
        """A name as (the device it belongs to, the link device -> it, or None if it is the device itself)."""
        now = float(self.clock.now())
        if name in ("arm/wrist_camera", "arm/gripper"):
            T = self.kin.wrist_camera(arm_q) if name.endswith("camera") else self.kin.part(arm_q, "gripper")
            return "arm", Link(T, np.diag([FK_SIGMA_RAD ** 2] * 3 + [FK_SIGMA_M ** 2] * 3), now, ["the arm's joint angles and its own shape"])
        if name.startswith("rover/head/"):
            head, _ = self.rover_now()
            return "rover", Link(self.rover.camera(name.rsplit("/", 1)[1], head), np.diag([HEAD_SIGMA_RAD ** 2] * 3 + [HEAD_SIGMA_M ** 2] * 3),
                                 now, ["the rover's head joints and its own shape"])
        return name, None

    def _edges(self) -> tuple[dict, list[str]]:
        """Every direct link there is now, both ways round, and why the missing ones are missing."""
        edges, why = {}, []
        for name, track in self.cameras.items():
            if track.link is not None and self.has_arm:
                link = Link(track.link.T, track.link.cov, track.link.t, [f"{name} saw " + " and ".join(track.link.support)])
                edges[(name, "arm")], edges[("arm", name)] = link, link.inverse()
            elif self.has_arm:
                why.append(f"{name} and the arm: {track.why_not}.")
        if self.has_rover:
            for other in (["arm"] if self.has_arm else []) + list(self.cameras):
                link = self._rover_from(other)
                if link is not None:
                    edges[(other, "rover")], edges[("rover", other)] = link, link.inverse()
                else:
                    why.append(f"{other} and the rover: neither has seen the other's tags.")
        return edges, why

    def chain(self, of: str, relative_to: str, arm_q=None) -> tuple[Link | None, str]:
        """`of`'s frame in `relative_to`'s as a link (or None), and if none, why."""
        known = self.names()
        for n in (of, relative_to):
            if n not in known:
                raise KeyError(n)
        (root_a, part_a), (root_b, part_b) = self._part(relative_to, arm_q), self._part(of, arm_q)
        edges, why = self._edges()
        # The surest chain of links from one device to the other (few devices: try every simple path). Always searched
        # from the same end, whichever way round it was asked: a chain and its reverse are not equally sure in position,
        # and the two questions must not be answered through different chains.
        start, end = sorted((root_a, root_b))
        best, stack = None, [(start, Link(np.eye(4), np.zeros((6, 6)), float(self.clock.now()), []), {start})]
        while stack:
            node, link, seen = stack.pop()
            if node == end:
                if best is None or link.sigma_m < best.sigma_m:
                    best = link
                continue
            for (a, b), step in edges.items():
                if a == node and b not in seen:
                    stack.append((b, link.then(step), seen | {b}))
        if best is None:
            return None, ("no chain of sightings links them. " + " ".join(why) if why
                          else "no chain of sightings links them: nothing here carries tags that the other could see.")
        if start != root_a:
            best = best.inverse()
        if part_a is not None:
            best = part_a.inverse().then(best)
        if part_b is not None:
            best = best.then(part_b)
        return best, ""

    def where(self, of: str, relative_to: str, arm_q=None) -> dict:
        """`of`'s frame in `relative_to`'s, or known = False and why."""
        out = {"ok": True, "of": of, "relative_to": relative_to}
        link, why = self.chain(of, relative_to, arm_q)
        if link is None:
            return {**out, "known": False, "why": why}
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
                           "rover": "origin on the surface under the middle of its wheel axle, x forward, y left, z up",
                           "a camera": "x to the picture's right, y down the picture, z the way it looks"}}

    def summary(self) -> dict:
        """Each fixed camera: whether it is placed against the arm (or, where there is no arm, the rover) by any chain of
        sightings, how surely, and from what."""
        anchor = "arm" if self.has_arm else "rover" if self.has_rover else None
        out = {}
        for name, t in self.cameras.items():
            w = self.where(name, anchor) if anchor else None
            if w is not None and w["known"]:
                row = {"located": True, "relative_to": anchor, "uncertainty_m": w["uncertainty_m"], "from": w["support"]}
            else:
                row = {"located": False, "why": (t.why_not if self.has_arm and t.why_not else w["why"] if w is not None
                                                 else "there is neither an arm nor a rover to place it against")}
            out[name] = row | ({"notes": t.notes} if t.notes else {})
        if self.has_rover and self.has_arm:
            w = self.where("rover", "arm")
            out_rover = ({"located": True, "relative_to": "arm", "uncertainty_m": w["uncertainty_m"], "from": w["support"]} if w["known"]
                         else {"located": False, "why": w["why"]})
            return {"cameras": out, "rover": out_rover}
        return {"cameras": out}
