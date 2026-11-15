"""The start-up survey: the rover looks all round from where it stands, before anything drives.

    result = survey(hub)

Its wheels do not move. The head goes through a few passes (a height, a tilt, a camera spacing) and
at each turns outward from straight ahead, one way and then the other; at every stop both cameras
take a full-size picture. The head's ends swing into places no camera has looked at yet, so the pan
pushes with very little (rover/device.py), stops where it meets something, and does not try that way
again. At the end the first view is taken again: if the picture has slid, the rover was turned where
it stood, and the map is marked or dropped. From the pictures:

  * the map (hub/place.py): depth from the pair, sized by the spacing drive's travel;
  * the locator (hub/locator.py): any other device's tags in view;
  * the rulers. The spacing drive is the ruler everything is sized by. Two others are checked against
    it: where both cameras see the same tag, its side as measured across the pair against its printed
    side; and the height of the lenses above the ground as the pair measures it against what the
    lift's angle says it is. One that disagrees says which of them is not what the rover believes.

Each cloud is laid on the ground it contains: the flat ground near the rover is found in it, and
heights are measured from that, not from what the head's joints say is level (a lens's centre a few
pixels off tilts everything it sees by half a degree; the rover may not sit quite level either).
"""
from __future__ import annotations

import math

import cv2
import numpy as np

from . import builtin
from .bus import BusError
from .errors import Refused
from .ports import PortError
from .vision import stereo
from .vision import tags as V

# lens height [m], tilt [deg], lens spacing [m]. The spacing stays under 0.6 of the height: seen from low down with the
# cameras far apart, the ground's near rows shift so much more than its far rows that the two pictures cannot be matched.
# Three passes: from low down (what stands under and between things); from high up with the cameras close together, for
# what is near (a thing 0.17 m from lenses 100 mm apart is seen from directions 34 deg apart and cannot be matched; at
# 60 mm it can); and from high up, looking further out, with them wider apart.
PASSES = ((0.10, -25.0, 0.06), (0.16, -38.0, 0.06), (0.16, -12.0, 0.10))
PANS = (0.0, 40.0, 80.0, 120.0, 160.0)                # each way from straight ahead: outward one way, back, outward the other
TURNED_DROP_DEG = 8.0               # a rover found turned by more than this at the end has smeared its map: the map is dropped
TURNED_NOTE_DEG = 0.7
USE_FIELD = 0.6                     # only this far off the middle of the picture goes into the map (tan of the angle: 31 deg each
                                    # side of the lens's 44). Two lenses differ most at the sides, where one's bulge against the
                                    # other's bends far things by several per cent; neighbouring views overlap enough to do without.
REACH_PER_SPACING = 7.0             # a pair is used out to this many times its spacing (0.42 m at 60 mm, 0.7 m at 100 mm): its error
                                    # in distance grows as distance squared over spacing, and an uncalibrated pair at 0.5 m with the
                                    # cameras 60 mm apart is 5 to 8 % out, which smears a far edge across several cells
HEAD_MOVE_S = 15.0                  # time allowed for each move of the head
MIN_TAG_PX = 60                     # a tag smaller than this in either picture is not measured: its corners are a pixel or so
                                    # uncertain, and across two cameras that is several per cent of its side
RULER_TOLERANCE = 0.04              # rulers that disagree by more than this are reported as disagreeing


def measure_tags(found_left, found_right, lens_left, lens_right, baseline_m: float) -> dict[int, float]:
    """Tags seen by both cameras: tag id -> its side as measured across the pair, over its printed side."""
    right = {d.id: d for d in found_right}
    out = {}
    for d in found_left:
        kind = builtin.kind_of_tag(d.id)
        if d.id not in right or kind is None or min(d.side_px, right[d.id].side_px) < MIN_TAG_PX:
            continue
        if max(abs(float(np.mean(d.corners[:, 0])) - lens_left.cx), abs(float(np.mean(right[d.id].corners[:, 0])) - lens_right.cx)) > USE_FIELD * lens_left.fx:
            continue                                         # at the side of a picture, where the lenses are least alike
        p = stereo.triangulate(d.corners, right[d.id].corners, lens_left, lens_right, baseline_m)
        sides = [float(np.linalg.norm(p[k] - p[(k + 1) % 4])) for k in range(4)]
        out[d.id] = float(np.mean(sides)) / builtin.tags(kind)[d.id].size_m
    return out


def turned(before: np.ndarray, after: np.ndarray, lens) -> float | None:
    """How far the view has turned between two pictures taken with the head in the same pose [deg, left positive], from
    how far the picture has slid sideways. None if the two pictures cannot be matched at all."""
    small = [cv2.resize(cv2.cvtColor(x, cv2.COLOR_RGB2GRAY), None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA).astype(np.float32)
             for x in (before, after)]
    (dx, _), response = cv2.phaseCorrelate(small[0], small[1], cv2.createHanningWindow(small[0].shape[::-1], cv2.CV_32F))
    if response < 0.08:
        return None
    return math.degrees(math.atan(4.0 * dx / lens.fx))       # the picture slides right when the camera turns left


def take_view(hub, ratios: dict, heights: list, arm_q=None) -> tuple[dict, np.ndarray, object]:
    """One pair of pictures with the head where it is: into the map, into the locator, into the rulers.
    Returns what the view gave, the left picture and its lens."""
    rover, place, locator = hub.rover, hub.place, hub.locator
    left, right = rover.cameras["left"], rover.cameras["right"]
    rover.settle()                                           # the two pictures are not taken at the same instant
    head = rover.head_now()
    fl, fr = left.grab(), right.grab()
    lens_l, lens_r = left.lens(fl.image.shape[1], fl.image.shape[0]), right.lens(fr.image.shape[1], fr.image.shape[0])
    baseline = rover.rover.lens_spacing(head)
    found_l, found_r = V.detect(fl.image), V.detect(fr.image)
    for side, found, frame in (("left", found_l, fl), ("right", found_r, fr)):
        locator.see_from_rover(side, found, frame.t, head, arm_q, rover.pose.copy(), (left if side == "left" else right).lens(1920, 1080))
    for tag, ratio in measure_tags(found_l, found_r, lens_l, lens_r, baseline).items():
        ratios.setdefault(tag, []).append(ratio)
    cloud = stereo.depth(fl.image, fr.image, lens_l, lens_r, baseline)
    T_cam = rover.rover.camera("left", head)
    view = {"lens_height_m": round(float(T_cam[2, 3]), 3), "tilt_deg": round(math.degrees(head["tilt"]), 1),
            "lens_spacing_m": round(baseline, 3), "pan_deg": round(math.degrees(head["pan"]), 1),
            "tags": sorted({d.id for d in found_l} | {d.id for d in found_r})}
    if cloud is None:
        view.update(used=False, why="too little detail in view to line the two pictures up")
        return view, fl.image, lens_l
    if place.empty:
        hub.map_origin = rover.pose.copy()                   # the map's frame is the rover's as it stands at its first view
    hub.guard.saw_arm(arm_q)                                 # what this view maps of the arm is the arm
    T = hub.map_from_rover() @ T_cam
    keep = (np.abs(cloud.points[:, 0]) < USE_FIELD * cloud.points[:, 2]) & (np.linalg.norm(cloud.points, axis=1) < REACH_PER_SPACING * baseline)
    seen = cloud.points[keep]
    pts = seen @ T[:3, :3].T + T[:3, 3]
    ground = stereo.ground_plane(seen, T_cam[:3, :3].T @ np.array([0.0, 0.0, -1.0]), float(T_cam[2, 3]))
    if ground is not None:                                   # heights from the ground that is in the picture
        pts[:, 2] = (1.0 - seen @ ground) / np.linalg.norm(ground)
        heights.append(1.0 / float(np.linalg.norm(ground)) / float(T_cam[2, 3]))
    view.update(place.add_cloud(pts, T[:3, 3], None if cloud.colours is None else cloud.colours[keep]), matched=True)
    hub.guard.stood_here()                                   # and where the rover stands is bare surface
    return view, fl.image, lens_l


def look(hub) -> dict:
    """One pair of pictures with the rover's head where it is now, added to the map. For looking again at something, or
    from somewhere the rover has driven to: the map stays in the frame it was begun in, and the new view is placed in it
    by what the wheels say the rover has done since."""
    rover = hub.rover
    if rover is None:
        raise Refused("there is no rover to look with")
    if "left" not in rover.cameras or "right" not in rover.cameras:
        raise Refused("the rover's two cameras did not both answer, so it cannot measure depth")
    if rover.stowed():
        raise Refused("the head is stowed: its cameras are 26 mm above the surface and see almost nothing of it. Raise it first "
                      "(POST /devices/rover/head/raise), then point it")
    heights: list[float] = []
    view, _, _ = take_view(hub, {}, heights, hub.arm.link.q() if hub.arm is not None else None)
    hub.last_survey.append(view)
    if hub.cameras:
        hub._look()                                          # the fixed cameras look too: where the rover is, for them, now
    went = float(np.linalg.norm(hub.map_from_rover()[:2, 3]))
    return {"ok": True, "view": view,
            "driven_since_the_map_was_begun_m": round(went, 3),
            **({"note": "this view is placed in the map by the rover's wheels: about 3 % of what they say it has driven and turned "
                        "since the map was begun, on top of the map's own error"} if went > 0.01 else {}),
            "place": hub.place_state()}


def _sweep(side_first: int, pans, limit: dict[int, float]):
    """The pan angles of one pass, in order: outward on one side, then outward on the other, no further on either side
    than where the head met something earlier in this survey."""
    for side in (side_first, -side_first):
        for pan in pans:
            if pan == 0.0 and side != side_first:
                continue
            if pan <= limit[side]:
                yield side, side * pan


def survey(hub, passes=PASSES, pans=PANS) -> dict:
    rover = hub.rover
    if rover is None:
        raise Refused("there is no rover to survey with")
    if "left" not in rover.cameras or "right" not in rover.cameras:
        raise Refused("the rover's two cameras did not both answer, so it cannot measure depth")
    rover._may_move()
    t0 = hub.clock.now()
    place, locator = hub.place, hub.locator
    start = rover.pose.copy()                                # the wheels stay still
    left = rover.cameras["left"]
    ratios: dict[int, list[float]] = {}
    heights: list[float] = []
    views, skipped, met, stopped = [], [], [], None
    arm_q = hub.arm.link.q() if hub.arm is not None else None
    limit = {1: 360.0, -1: 360.0}                            # how far each way the head has been able to turn
    reference = None                                         # the first view that had enough in it: looked at again at the end
    try:
        for n, (height, tilt, spacing) in enumerate(passes):
            for step in (lambda: rover.point(pan_deg=0.0, timeout_s=HEAD_MOVE_S), lambda: rover.raise_head(height, timeout_s=HEAD_MOVE_S),
                         lambda: rover.spread(spacing, timeout_s=HEAD_MOVE_S), lambda: rover.point(tilt_deg=tilt, timeout_s=HEAD_MOVE_S)):
                r = step()                                   # (raised and spread while facing forward, where it began)
                if not r["reached"]:
                    stopped = r.get("why_not_reached", "the head did not get there")
                    break
            if stopped:
                break
            blocked_side = None
            for side, pan in _sweep(1 if n % 2 == 0 else -1, pans, limit):
                if side == blocked_side:
                    continue
                r = rover.point(pan_deg=pan, timeout_s=HEAD_MOVE_S)
                if not r["reached"]:
                    skipped.append({"lens_height_m": height, "pan_deg": pan, "why": r.get("why_not_reached", "the head did not get there")})
                    if rover.last_block:                     # it met something: no further that way, in this pass or a later one
                        blocked_side, limit[side] = side, abs(pan) - (pans[1] - pans[0])
                        met.append({"lens_height_m": height, "pan_deg": round(math.degrees(rover.head_now()["pan"]), 1)})
                    continue
                view, picture, lens_l = take_view(hub, ratios, heights, arm_q)
                view.update(lens_height_m=height, tilt_deg=tilt, pan_deg=pan)
                if view["used"] is not False and reference is None and view.get("matched"):
                    reference = (height, tilt, spacing, pan, picture, lens_l)
                views.append(view)
        # Has the rover been turned where it stands? Its wheels cannot say: a head that meets something pushes the whole
        # rover round on its tyres. The first view is taken again and set against what it was.
        if reference is not None and not stopped:
            height, tilt, spacing, pan, before, lens = reference
            if all(step()["reached"] for step in (
                    lambda: rover.point(pan_deg=0.0, timeout_s=HEAD_MOVE_S), lambda: rover.raise_head(height, timeout_s=HEAD_MOVE_S),
                    lambda: rover.spread(spacing, timeout_s=HEAD_MOVE_S), lambda: rover.point(tilt_deg=tilt, timeout_s=HEAD_MOVE_S),
                    lambda: rover.point(pan_deg=pan, timeout_s=HEAD_MOVE_S))):
                rover.settle()
                moved = turned(before, left.grab().image, lens)
                check = {"checked": True, "matched": moved is not None, **({"turned_deg": round(moved, 1)} if moved is not None else {})}
            else:
                check = {"checked": False, "why": "the head could not be brought back to where the first view was taken from"}
        else:
            check = {"checked": False, "why": "no view had enough in it to compare" if reference is None else "the survey stopped early"}
    finally:
        try:
            rover.stow()
        except (BusError, Refused):
            pass
    if hub.cameras:
        hub._look()                                          # the fixed cameras look too: where the rover is, for them, now
    kept = True
    if check["checked"] and (not check["matched"] or abs(check["turned_deg"]) > TURNED_DROP_DEG):
        kept = False                                         # what was mapped before and after it moved does not fit together
        hub.place = place = type(place)()
        locator.forget_rover(since=t0)
        check["warning"] = ("the rover was turned or moved where it stood during the survey (most likely its head met something): "
                            "the map made was dropped, and so was what the rover saw of other devices. Look at where it stands "
                            "(its cameras' pictures) before surveying again")
    elif check["checked"] and abs(check["turned_deg"]) > TURNED_NOTE_DEG:
        check["warning"] = (f"the rover was turned about {abs(check['turned_deg']):.0f} deg where it stood during the survey (its head met "
                            "something): the map is that much less sure in direction")
    if ratios:
        per_tag = {str(t): round(float(np.median(v)), 4) for t, v in sorted(ratios.items())}
        overall = float(np.median([np.median(v) for v in ratios.values()]))
        place.rulers["tags_measured_by_the_camera_spacing"] = {
            "printed_size_over": "each tag's side measured across the two cameras, divided by its printed side",
            "per_tag": per_tag, "overall": round(overall, 4), "agree": bool(abs(overall - 1) <= RULER_TOLERANCE),
            **({} if abs(overall - 1) <= RULER_TOLERANCE else
               {"warning": f"the camera spacing and the tags' printed sizes disagree by {abs(overall - 1) * 100:.1f} %. Most likely the two "
                           "cameras do not look quite parallel (the pair is not yet calibrated), which puts everything that much too far or "
                           "too near; otherwise the spacing drive's travel or the tags' print scale is not what the rover believes"})}
    if len(heights) >= 3:
        ratio = float(np.median(heights))
        place.rulers["lens_height_by_the_camera_spacing"] = {
            "measured_over_expected": "how high the lenses are above the ground as the two cameras measure it, divided by what the "
                                      "lift's angle says", "overall": round(ratio, 4), "views": len(heights),
            "agree": bool(abs(ratio - 1) <= RULER_TOLERANCE),
            **({} if abs(ratio - 1) <= RULER_TOLERANCE else
               {"warning": f"the camera spacing and the lift disagree by {abs(ratio - 1) * 100:.1f} %: the spacing drive's travel, the "
                           "lift's angle, or what the rover stands on is not what it believes"})}
    used = sum(1 for v in views if v.get("used"))
    hub.last_survey = views                                  # every view, for whoever wants the detail
    return {"ok": True, "views": {"taken": len(views), "used_for_the_map": used, "skipped": skipped,
                                  "not_used": [{k: v[k] for k in ("lens_height_m", "tilt_deg", "pan_deg", "why")} for v in views if not v.get("used")]},
            **({"stopped_early": stopped} if stopped else {}),
            "tags_seen": sorted({t for v in views for t in v["tags"]}),
            "head_met_something_at": met, "stood_still": check, "map_kept": kept,
            "place": hub.place_state(), "elapsed_s": round(hub.clock.now() - t0, 2),
            "wheels_moved": bool(np.abs(rover.pose - start).max() > 1e-3),
            **({"note": "the head met something and did not look further that way. What is there is not in the map: it is nearer "
                        "than the cameras can see, or in a direction the head could not turn to"} if met else {})}
