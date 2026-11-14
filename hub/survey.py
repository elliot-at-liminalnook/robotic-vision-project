"""The start-up survey: the rover looks all round from where it stands, before anything drives.

    result = survey(hub)

Its wheels do not move. The head goes through a few passes (a height, a tilt, a camera spacing) and
at each turns through its pan range; at every stop both cameras take a full-size picture. From them:

  * the map (hub/place.py): depth from the pair, sized by the spacing drive's travel;
  * the locator (hub/locator.py): any other device's tags in view;
  * the rulers: where both cameras see the same tag, its size as measured across the pair is
    compared with its printed size. The spacing drive is the ruler everything is sized by; a tag
    that does not measure what it is printed as says one of the two is wrong.
"""
from __future__ import annotations

import numpy as np

from . import builtin
from .bus import BusError
from .errors import Refused
from .ports import PortError
from .vision import stereo
from .vision import tags as V

PASSES = ((0.10, -25.0, 0.06), (0.16, -35.0, 0.10), (0.16, -15.0, 0.16))      # lens height [m], tilt [deg], lens spacing [m]
PANS = (-160.0, -120.0, -80.0, -40.0, 0.0, 40.0, 80.0, 120.0, 160.0)
RULER_TOLERANCE = 0.04              # rulers that disagree by more than this are reported as disagreeing


def measure_tags(found_left, found_right, lens_left, lens_right, baseline_m: float) -> dict[int, float]:
    """Tags seen by both cameras: tag id -> its side as measured across the pair, over its printed side."""
    right = {d.id: d for d in found_right}
    out = {}
    for d in found_left:
        kind = builtin.kind_of_tag(d.id)
        if d.id not in right or kind is None or min(d.side_px, right[d.id].side_px) < 40:
            continue
        p = stereo.triangulate(d.corners, right[d.id].corners, lens_left, lens_right, baseline_m)
        sides = [float(np.linalg.norm(p[k] - p[(k + 1) % 4])) for k in range(4)]
        out[d.id] = float(np.mean(sides)) / builtin.tags(kind)[d.id].size_m
    return out


def survey(hub, passes=PASSES, pans=PANS) -> dict:
    rover = hub.rover
    if rover is None:
        raise Refused("there is no rover to survey with")
    if "left" not in rover.cameras or "right" not in rover.cameras:
        raise Refused("the rover's two cameras did not both answer, so it cannot measure depth")
    rover._may_move()
    t0 = hub.clock.now()
    place, locator = hub.place, hub.locator
    start = rover.pose.copy()                                # the wheels stay still: the map's frame is where it stands now
    if place.empty:
        hub.map_origin = start.copy()
    left, right = rover.cameras["left"], rover.cameras["right"]
    ratios: dict[int, list[float]] = {}
    views, skipped, stopped = [], [], None
    arm_q = hub.arm.link.q() if hub.arm is not None else None
    try:
        for height, tilt, spacing in passes:
            for step in (lambda: rover.raise_head(height), lambda: rover.spread(spacing), lambda: rover.point(tilt_deg=tilt)):
                r = step()
                if not r["reached"]:
                    stopped = r.get("why_not_reached", "the head did not get there")
                    break
            if stopped:
                break
            for pan in pans:
                r = rover.point(pan_deg=pan)
                if not r["reached"]:
                    skipped.append({"pan_deg": pan, "why": r.get("why_not_reached", "the head did not get there")})
                    if rover.last_block:
                        break                                # it met something: no further that way on this pass
                    continue
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
                view = {"lens_height_m": height, "tilt_deg": tilt, "lens_spacing_m": round(baseline, 3), "pan_deg": pan,
                        "tags": sorted({d.id for d in found_l} | {d.id for d in found_r})}
                if cloud is None:
                    view.update(used=False, why="too little detail in view to line the two pictures up")
                else:
                    T = hub.map_from_rover() @ rover.rover.camera("left", head)
                    pts = cloud.points @ T[:3, :3].T + T[:3, 3]
                    view.update(place.add_cloud(pts, T[:3, 3]))
                views.append(view)
    except (BusError, PortError):
        raise
    finally:
        try:
            rover.stow()
        except (BusError, Refused):
            pass
    if ratios:
        per_tag = {str(t): round(float(np.median(v)), 4) for t, v in sorted(ratios.items())}
        overall = float(np.median([np.median(v) for v in ratios.values()]))
        place.rulers["tags_measured_by_the_camera_spacing"] = {
            "printed_size_over": "each tag's side measured across the two cameras, divided by its printed side",
            "per_tag": per_tag, "overall": round(overall, 4), "agree": bool(abs(overall - 1) <= RULER_TOLERANCE),
            **({} if abs(overall - 1) <= RULER_TOLERANCE else
               {"warning": f"the camera spacing and the tags' printed sizes disagree by {abs(overall - 1) * 100:.1f} %: one of them is not what "
                           "the rover believes (the spacing drive's travel, or the tags' print scale)"})}
    used = sum(1 for v in views if v.get("used"))
    return {"ok": True, "views": {"taken": len(views), "used_for_the_map": used, "skipped": skipped},
            **({"stopped_early": stopped} if stopped else {}),
            "tags_seen": sorted({t for v in views for t in v["tags"]}),
            "place": place.summary(), "elapsed_s": round(hub.clock.now() - t0, 2),
            "wheels_moved": bool(np.abs(rover.pose - start).max() > 1e-3)}
