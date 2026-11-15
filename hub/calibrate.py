"""Finding how the devices differ from what their drivers believe, from what they see of each other's printed tags.

No target is put down by a person. Three things are found, each from a motion the devices can make themselves:

  a fixed camera's focal length   the arm shows the tag behind its wrist at several poses (POST /locate with
                                  show_wrist_tag); the tags' places are known from the arm's joints, and the focal
                                  length is the one that makes all of them land where they were seen
  the rover's pair of cameras     the head looks at a tag while the spacing drive moves the cameras apart. A tag's
                                  disparity grows in proportion to the spacing; what is left over at no spacing at
                                  all is the pair's sideways shift. The ratio of the tag's size in the two pictures
                                  is the ratio of the two lenses' focal lengths
  the rover's wheels              a fixed camera watches the rover drive: how far it went by the camera, over how
                                  far its wheels counted

Each says what it found, how sure it is, and what it changed. A correction smaller than its own uncertainty is
not applied: the description is left as it is.
"""
from __future__ import annotations

import math

import cv2
import numpy as np

from . import geometry as G
from .builtin import LENS_FOCAL_TOLERANCE, Lens
from .errors import Refused
from .vision import tags as V

SPACINGS = (0.06, 0.09, 0.12, 0.15, 0.18)        # camera spacings the pair is measured at [m]
END_M = 0.004                                    # how well a camera places the rover at one end of a drive
WHEEL_TOLERANCE = 0.03                           # how far a wheel's true size may be from its description (printed tyre, one sigma)
MIN_TAG_PX = 45                                  # a tag smaller than this in either picture is not used


# ---------------------------------------------------------------- a fixed camera's lens
def lens(hub, name: str) -> dict:
    """The focal length of one fixed camera, from everything it has seen of the arm's tags."""
    if name not in hub.cameras:
        raise Refused(f"there is no fixed camera '{name}'; there are {sorted(hub.cameras)}", 404)
    cam, track = hub.cameras[name], hub.locator.cameras[name]
    seen = track.sightings
    poses = len({s.view for s in seen if s.moving_part})
    if len(seen) < 6 or poses < 3:
        raise Refused(f"{name} has seen {len(seen)} tag(s) of the arm, with the wrist tag at {poses} pose(s): too little to find its "
                      "focal length from. POST /locate {\"show_wrist_tag\": true} has the arm show its wrist tag at six poses.", 409)
    nominal = cam.nominal(1920, 1080)
    obj = np.vstack([s.corners_arm for s in seen]).astype(np.float32)
    img = np.vstack([s.corners_px for s in seen]).astype(np.float32)
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_ZERO_TANGENT_DIST
             | cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3)
    rms, K, _, rvecs, tvecs, sd_in, _, _ = cv2.calibrateCameraExtended([obj], [img], (1920, 1080), nominal.K(), np.asarray(nominal.dist, float),
                                                                        flags=flags)
    scale, sigma = float(K[0, 0] / nominal.fx), float(max(np.ravel(sd_in)[:2]) / nominal.fx)
    # How much the tags are spread along the camera's view: with none, a longer lens and a nearer camera look the same.
    depth = (cv2.Rodrigues(rvecs[0])[0] @ obj.T).T[:, 2] + float(np.ravel(tvecs[0])[2])
    relief = float(np.ptp(depth) / np.mean(depth))
    out = {"ok": True, "camera": name, "focal_length_over_nominal": round(scale, 4), "uncertainty": round(sigma, 4),
           "from": f"{len(seen)} sightings of the arm's tags, the wrist tag at {poses} poses", "misses_by_px": round(float(rms), 2),
           "depth_spread": round(relief, 3)}
    if rms > 2.0 or sigma > 0.012 or relief < 0.08 or abs(scale - 1) > 0.08:
        return {**out, "applied": False,
                "why": ("the tags it saw are nearly all at one distance, which leaves the focal length undetermined" if relief < 0.08
                        else "the fit is too loose to trust" if rms > 2.0 or sigma > 0.012
                        else "the answer is further from the datasheet than a lens of this kind can be")}
    # The datasheet (good to LENS_FOCAL_TOLERANCE) and what was just measured (good to twice its own figure, which
    # leaves out what the fit cannot see: the lens's centre and bulge are still the datasheet's), weighed by how sure each is.
    best, sure = _weigh(1.0, LENS_FOCAL_TOLERANCE, scale, max(2 * sigma, 0.003))
    cam.focal_scale, cam.focal_sigma = best, max(sure, 0.004)
    hub.locator.new_lens(name, cam.lens(1920, 1080), cam.focal_sigma)
    return {**out, "applied": True, "focal_length_now_taken_as": round(best, 4),
            "note": f"distances through {name} are now taken to be good to {cam.focal_sigma * 100:.1f} % of themselves, where the datasheet "
                    f"alone was good to {LENS_FOCAL_TOLERANCE * 100:.1f} %"}


def _weigh(a: float, sigma_a: float, b: float, sigma_b: float) -> tuple[float, float]:
    """Two values for one thing, each with how sure it is: the value both support best, and how sure that is."""
    wa, wb = 1.0 / sigma_a ** 2, 1.0 / sigma_b ** 2
    return (a * wa + b * wb) / (wa + wb), 1.0 / math.sqrt(wa + wb)


# ---------------------------------------------------------------- the rover's two cameras
def _flat(corners: np.ndarray, lens_: Lens) -> np.ndarray:
    """Picture points with the nominal distortion taken out, still in pixels."""
    return cv2.undistortPoints(np.asarray(corners, np.float64).reshape(-1, 1, 2), lens_.K(), np.asarray(lens_.dist, float), P=lens_.K()).reshape(-1, 2)


def shift_line(B: np.ndarray, D: np.ndarray) -> tuple[float, float, float]:
    """A tag's disparity D [px] at camera spacings B [m] as a straight line: (growth per metre, what is left at no spacing,
    how far the line misses [px]). One spacing misread (the tag at a picture's edge) is left out if the rest then agree."""
    B, D = np.asarray(B, float), np.asarray(D, float)

    def line(keep):
        A = np.c_[B[keep], np.ones(int(keep.sum()))]
        (k, shift), *_ = np.linalg.lstsq(A, D[keep], rcond=None)
        return float(k), float(shift), float(np.sqrt(np.mean((A @ [k, shift] - D[keep]) ** 2)))

    best = line(np.ones(len(B), bool))
    if best[2] > 1.0 and len(B) >= 4:
        for drop in range(len(B)):
            keep = np.arange(len(B)) != drop
            if np.ptp(B[keep]) >= 0.05 and line(keep)[2] < min(best[2], 1.0):
                best = line(keep)
    return best


def pair(hub, spacings=SPACINGS) -> dict:
    """The rover's pair of cameras, from a tag both can see while the spacing drive moves them apart."""
    rover = hub.rover
    if rover is None or len(rover.cameras) != 2:
        raise Refused("there is no rover with two cameras", 404)
    if rover.stowed():
        raise Refused("the head is stowed. Raise it and point it at a printed tag (the arm's are beside its base) first.", 409)
    left, right = rover.cameras["left"], rover.cameras["right"]
    lens_ = left.nominal(1920, 1080)
    was = rover.rover.lens_spacing(rover.head_now())
    seen: dict[int, list[tuple[float, np.ndarray, np.ndarray]]] = {}
    try:
        for b in spacings:
            if not rover.spread(b, timeout_s=10.0)["reached"]:
                continue
            rover.settle()
            B = rover.rover.lens_spacing(rover.head_now())
            fl, fr = left.grab(), right.grab()
            in_right = {d.id: d for d in V.detect(fr.image)}
            for d in V.detect(fl.image):
                if d.id in in_right and min(d.side_px, in_right[d.id].side_px) >= MIN_TAG_PX:
                    seen.setdefault(d.id, []).append((B, _flat(d.corners, lens_), _flat(in_right[d.id].corners, lens_)))
    finally:
        rover.spread(was, timeout_s=10.0)
    good = {t: v for t, v in seen.items() if len(v) >= 3 and v[-1][0] - v[0][0] >= 0.05}
    if not good:
        raise Refused("no printed tag was in both cameras' view at three or more spacings (tags seen: "
                      f"{sorted(seen) or 'none'}). Point the head at a tag about 0.25 to 0.5 m away and try again.", 409)
    # The ratio of the two focal lengths, from the rows: two cameras side by side see a point on the same row whatever its
    # distance, so how the right picture's rows run against the left's is the lenses' doing (and the cameras' small tilt and
    # roll against each other, which the other two terms take up). A tag's size in the two pictures would not do: seen from
    # two places it is foreshortened differently.
    L = np.vstack([l_ for v in good.values() for _, l_, _ in v])
    R = np.vstack([r_ for v in good.values() for _, _, r_ in v])
    cx, cy = lens_.cx, lens_.cy
    rows = np.c_[L[:, 1] - cy, L[:, 0] - cx, np.ones(len(L))]
    (stretch, roll, _), *_ = np.linalg.lstsq(rows, R[:, 1] - cy, rcond=None)
    stretch = float(stretch) if np.ptp(L[:, 1]) >= 50 else 1.0       # (rows too alike to tell: leave the two lenses equal)
    shifts, growth, misses = [], {}, []
    for t, v in good.items():
        B = np.array([b for b, _, _ in v])
        D = np.array([float(np.mean(l_[:, 0] - (cx + (r_[:, 0] - cx) / stretch))) for _, l_, r_ in v])
        k, shift, miss = shift_line(B, D)
        shifts.append(shift)
        growth[t] = k
        misses.append(miss)
    shift = float(np.median(shifts))
    spread = float(np.ptp(shifts)) if len(shifts) > 1 else float("nan")
    out = {"ok": True, "tags_used": sorted(good), "spacings_used": sorted({round(b, 3) for v in good.values() for b, _, _ in v}),
           "sideways_shift_px": round(shift, 2), "right_over_left_focal_length": round(stretch, 4),
           "line_misses_by_px": round(max(misses), 2), **({"tags_disagree_by_px": round(spread, 2)} if len(shifts) > 1 else {})}
    if max(misses) > 1.5 or (len(shifts) > 1 and spread > 4.0) or abs(shift) > 60 or abs(stretch - 1) > 0.05:
        return {**out, "applied": False, "why": "the disparity did not grow in a straight line with the spacing, or two tags gave different "
                                                "answers: the head moved, or a tag was misread"}
    rover.pair = {"offset_px": shift, "stretch": stretch}
    # The tags' printed size against the spacing drive, now that the shift is out of it: a tag's disparity grows by
    # (focal length / its distance) per metre of spacing, and its distance as read from its printed size is off by however
    # much the print is.
    ratios = {}
    for t, v in good.items():
        tag = _tag(t)
        if tag is not None:
            ok, _, tvec = cv2.solvePnP(_square(tag.size_m), v[0][1].astype(np.float64), lens_.K(), None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                # (its true distance is focal length / growth; a tag larger than printed reads nearer than it is by its size)
                ratios[str(t)] = round(float(lens_.fx / (growth[t] * float(np.ravel(tvec)[2]))), 4)
    if ratios:
        overall = float(np.median(list(ratios.values())))
        hub.place.rulers["tags_by_the_camera_spacing"] = {
            "printed_size_over": "each tag's side as the spacing drive measures it, over its printed side, with the pair's shift taken out",
            "per_tag": ratios, "overall": round(overall, 4), "agree": bool(abs(overall - 1) <= 0.03),
            **({} if abs(overall - 1) <= 0.03 else
               {"warning": f"the tags measure {overall:.3f} of their printed size by the spacing drive: the tags were printed at another "
                           "scale, or the spacing drive's travel is not what the rover believes"})}
        out["tags_measure_of_their_printed_size"] = round(overall, 4)
    return {**out, "applied": True, "note": f"everything the pair measured was {abs(shift):.1f} px of disparity out: at 0.4 m with the cameras "
                                            f"60 mm apart that is {abs(shift) / (lens_.fx * 0.06 / 0.4) * 100:.1f} % of the distance"}


def _tag(tag_id: int):
    from . import builtin
    kind = builtin.kind_of_tag(tag_id)
    return builtin.tags(kind)[tag_id] if kind is not None else None


def _square(side: float) -> np.ndarray:
    h = side / 2
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])


# ---------------------------------------------------------------- the rover's wheels
def wheels(hub, distance_m: float = 0.25) -> dict:
    """How far the rover goes for what its wheels count, by a fixed camera that watches it drive there and back.
    The longer the drive the better this is: a camera places the rover to a few millimetres at each end."""
    rover = hub.rover
    if rover is None:
        raise Refused("there is no rover", 404)
    if not hub.cameras:
        raise Refused("no fixed camera is there to watch the rover drive", 409)

    def sight():
        t = float(hub.clock.now())
        hub._look()
        return {s.other: s for s in hub.locator.rover_seen if s.other in hub.cameras and s.link.t >= t}

    if hub.place.empty:
        raise Refused("the wheels are calibrated only over ground the map shows free (POST /survey first): a drive into what nothing "
                      "has looked at may end against something, and wheels that slip there spoil the count", 409)
    first = sight()
    if not first:
        raise Refused("no fixed camera can see the rover's tags where it stands", 409)
    ratios, legs, lengths = [], [], []
    go = None
    for sign in (1.0, -1.0):                                         # which way is there more room that has been seen?
        try:
            d, _, slow, _ = hub.guard.drive(sign * distance_m, 0.0)
        except Refused:
            continue
        if not slow and abs(d) >= 0.10 and (go is None or abs(d) > abs(go) + 0.03):     # (forward, unless back is clearly longer)
            go = d
    if go is None:
        return {"ok": True, "applied": False,
                "why": "there is not 0.10 m of ground the map shows free ahead of the rover or behind it to drive over and back"}
    for d in (go, -go, go, -go):
        try:
            rover.forward(d)
        except Refused as e:
            legs.append({"asked_m": round(d, 3), "not_driven": str(e)})
            continue
        sign = 1.0
        then = sight()
        for name in set(first) & set(then):
            counted = float(np.linalg.norm(then[name].odometry[:2] - first[name].odometry[:2]))
            went = float(np.linalg.norm(then[name].link.T[:3, 3] - first[name].link.T[:3, 3]))
            if counted >= 0.06 and rover.last_block is None:
                ratios.append(went / counted)
                lengths.append(counted)
                legs.append({"asked_m": round(d, 3), "counted_m": round(counted, 4), "by": name, "went_m": round(went, 4)})
        first = then or first
    if not ratios:
        return {"ok": True, "applied": False, "legs": legs,
                "why": "the rover could not be driven far enough in the cameras' sight to compare (the map may have something in the way)"}
    ratio = float(np.average(ratios, weights=lengths))
    # How well a camera can tell: each end of a leg to about 4 mm, and every distance through it to its focal length's
    # uncertainty. Driving there and back again visits the same two places, so repeating it does not average either away:
    # a camera that puts the rover 4 mm out at one end does so every time. (Found: four 0.12 m legs agreed with each other
    # to 0.1 % and were all 4 % short.)
    seeing = END_M * math.sqrt(2) / float(np.mean(lengths))
    through = max(hub.cameras[leg_["by"]].focal_sigma for leg_ in legs if "by" in leg_)
    sure = math.hypot(max(seeing, float(np.std(ratios)) if len(ratios) > 1 else 0.0), through)
    best, now = _weigh(1.0, WHEEL_TOLERANCE, ratio, sure)                # the description, and what was seen, weighed together
    rover.wheel_scale *= best
    return {"ok": True, "went_over_counted": round(ratio, 4), "uncertainty": round(sure, 4), "legs": legs, "applied": True,
            "wheels_are_now_taken_to_be": round(rover.wheel_scale, 4), "good_to": round(now, 4),
            "note": f"the rover went {ratio:.3f} m for every metre its wheels counted, as far as the camera can tell (to {sure * 100:.1f} %). "
                    f"Weighed against its description (good to {WHEEL_TOLERANCE * 100:.0f} %), distances are now counted {best:.3f} times as long"}


# ---------------------------------------------------------------- everything that can be
def everything(hub) -> dict:
    """Each calibration that the devices here can make, in the order that lets each use the one before."""
    out: dict = {"ok": True}
    if hub.rover is not None and hub.place.empty and len(hub.rover.cameras) == 2:
        from . import survey as S
        survey = S.survey(hub)                                       # the wheels are calibrated over ground seen free
        out["survey"] = {k: survey[k] for k in ("views", "stood_still", "map_kept")}
    if hub.arm is not None and hub.cameras:
        hub.locate(show_wrist_tag=True)
        out["lenses"] = {}
        for name in hub.cameras:
            try:
                out["lenses"][name] = lens(hub, name)
            except Refused as e:
                out["lenses"][name] = {"applied": False, "why": str(e)}
    else:
        out["lenses"] = "not done: it takes an arm (to show its tags) and a fixed camera"
    if hub.rover is not None:
        try:
            out["wheels"] = wheels(hub)
        except Refused as e:
            out["wheels"] = {"applied": False, "why": str(e)}
        try:
            out["pair"] = _aimed_pair(hub)
        except Refused as e:
            out["pair"] = {"applied": False, "why": str(e)}
    return out


def _aimed_pair(hub) -> dict:
    """Point the rover's head at the arm's tags, if the arm's place is known, and calibrate the pair there."""
    rover = hub.rover
    link, why = (hub.locator.chain("arm", "rover", hub.arm.link.q()) if hub.arm is not None else (None, "there is no arm, so no tag to look at"))
    if link is None:
        raise Refused("the rover does not know where a tag is to look at: " + why, 409)
    at = link.T[:3, 3]
    pan = math.degrees(math.atan2(at[1], at[0] - 0.02))
    lim = rover.rover.limits["pan"]
    if not math.degrees(lim[0]) <= pan <= math.degrees(lim[1]):
        raise Refused("the arm's tags are behind the rover: turn it to face the arm first", 409)
    reach = float(np.hypot(at[0], at[1]))
    try:
        for step in (lambda: rover.raise_head(0.13), lambda: rover.point(pan_deg=pan), lambda: rover.point(tilt_deg=-math.degrees(math.atan2(0.10, max(reach - 0.2, 0.05))))):
            if not step()["reached"]:
                raise Refused("the head could not be pointed at the arm: something is in its way", 409)
        return pair(hub)
    finally:
        try:
            rover.stow()
        except Refused:
            pass
