"""The hub's REST API: what a language model uses to command the devices and ask what they sense.

Addresses follow the tree of interests (hub/tree.py):

    GET  /                          a guide, written for the model: read this first
    GET  /tree                      everything the hub has found, as a tree
    GET  /{node}                    what a node is, its commands and their parameters
    GET  /{node}/state              what the sensors under it say now
    GET  /{camera node}/picture     a picture (JPEG); ?width= ?tags=true ?base64=true
    POST /{node}/{command}          an action; JSON body with its parameters
    POST /{node}/stop               freeze everything under the node (POST /stop: everything)
    GET  /where?of=..&relative_to=..   where one device is in another's frame, if the hub can tell
    POST /wait                      let time pass

It gives no ground truth and no description of the place: only what the devices sense and what the
hub has worked out from that. Every answer is JSON with "ok" (pictures apart); when ok is false,
"error" says why in a sentence and nothing was changed.
"""
from __future__ import annotations

import base64
import json
import math
import sys
from http.server import BaseHTTPRequestHandler
from urllib.parse import parse_qs, urlparse

from .bus import BusError
from .camera import jpeg
from .errors import Refused
from .hub import Hub
from .ports import PortError
from .tree import Command, Node

RESERVED = ("state", "picture", "stop")


def validate(cmd: Command, body) -> dict:
    """The command's arguments from a request body: checked, with defaults filled in. Raises Refused."""
    if body is None:
        body = {}
    if not isinstance(body, dict):
        raise Refused("the request body must be a JSON object, e.g. {\"x\": 0.2}")
    unknown = sorted(set(body) - set(cmd.params))
    if unknown:
        raise Refused(f"unknown field(s) {unknown}; allowed here: {sorted(cmd.params) or 'none'}")
    args = {}
    for name, p in cmd.params.items():
        if name not in body or body[name] is None:
            if p.required:
                raise Refused(f"'{name}' is required: {p.description}")
            if p.default is not None:
                args[name] = p.default
            continue
        v = body[name]
        if p.type == "boolean":
            if not isinstance(v, bool):
                raise Refused(f"'{name}' must be true or false ({p.description})")
        elif p.type == "number":
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise Refused(f"'{name}' must be a finite number ({p.description}), got {v!r}")
            if (p.minimum is not None and v < p.minimum) or (p.maximum is not None and v > p.maximum):
                raise Refused(f"'{name}' = {v} is outside {p.minimum} to {p.maximum} ({p.description})")
        elif not isinstance(v, str):
            raise Refused(f"'{name}' must be text ({p.description})")
        args[name] = v
    return args


def guide(hub: Hub) -> str:
    """The guide for a caller, written for what this hub has found. It says nothing about the place."""
    lines = ["Hub API: several devices, commanded by you", "===========================================",
             "You command the devices listed below and ask them what they sense. You are told nothing about the",
             "place they are in: not its size, not what is in it, not where the devices stand relative to each",
             "other. Find out, from pictures and from the devices' own sensors.", "",
             "How it works",
             "  Everything is a node in a tree, addressed by its path. GET a node to see what it is and what it can do.",
             "    GET  /tree                      the whole tree",
             "    GET  /{node}                    what the node is; its commands with every parameter and range",
             "    GET  /{node}/state              what the sensors under it say now (a branch gives all of its parts)",
             "    GET  /{camera}/picture          a JPEG; ?width=640 smaller, ?tags=true the printed tags in it (JSON), ?base64=true",
             "    POST /{node}/{command}          an action; the body is JSON with its parameters",
             "    POST /{node}/stop               freeze everything under the node; POST /stop freezes everything",
             "    GET  /where?of=A&relative_to=B  where device A is in B's own frame: an estimate with its uncertainty, or known = false",
             "    GET  /where/pixel?camera=C&u=U&v=V&height_m=H&relative_to=arm",
             "                                    where the line of sight through point (U, V) of camera C's full-size picture meets",
             "                                    the level plane H metres above the surface, in the arm's (or the rover's) own frame:",
             "                                    for turning something seen in a picture into a place. C is a fixed camera's name,",
             "                                    arm/wrist_camera, rover/head/left or rover/head/right",
             "    GET  /where/point?camera=C&x=X&y=Y&height_m=H&relative_to=arm",
             "                                    the other way round: where the place (X, Y) at height H, in the arm's (or the rover's)",
             "                                    own frame, is in camera C's full-size picture, or that it is outside it: for 'can",
             "                                    that camera see there?' and for finding in one camera what another has found",
             "    GET  /place                     what has been found of the surroundings, from every device: the surface and its edges,",
             "                                    things (where, how big, how high, what colour, where in each fixed camera's picture),",
             "                                    what the arm has touched, and what is still unseen",
             "    GET  /place?relative_to=arm     the same with every position in that device's own frame, if the two are related",
             "    GET  /place/picture             the same as a picture (PNG)",
             "    POST /wait                      {\"seconds\": s}: let time pass",
             "    POST /discover                  look again for what is plugged in",
             "  Units: metres, radians, seconds. Every answer is JSON with \"ok\"; when ok is false, \"error\" says why and nothing was changed.",
             "  Time moves only inside an action (and a little for each picture). Reading state is free.",
             "  Each device has its own frame; there is no shared one. A position you get or give is always in some device's frame.",
             "  Devices carry printed tags so that cameras can find them: a picture's ?tags=true lists the ones in view and what kind of",
             "  device carries each. /where uses them: it answers from what the cameras have seen so far, with an uncertainty, how",
             "  old the newest sighting is and what supports it; or known = false and why. POST /locate looks afresh; with",
             "  show_wrist_tag the arm shows a tag at several poses, which locates cameras that cannot see its base.",
             "  Names for /where: a device's name, and arm/wrist_camera, arm/gripper, rover/head/left and rover/head/right for those parts.",
             "  The place starts empty. POST /survey has the rover look all round from where it stands and build the map; the map is in",
             "  the rover's frame as it stood then. POST /devices/rover/head/look adds one more view from wherever the rover and its",
             "  head are now. POST /devices/arm/probe lowers the arm's tip until it rests on something: a height good to 3 mm, and a",
             "  thing both the arm and the map have found relates the two even if neither can see the other.", "",
             "What was found"]
    devices = hub.root.children["devices"].children
    if not devices:
        lines.append("  nothing answered on any port")
    for name, n in devices.items():
        lines.append(f"  /devices/{name}: {n.summary}")
        for k, text in n.about.items():
            if k not in ("device", "lens", "pictures"):
                lines.append(f"      {k}: {text}")
    for note in hub.notes:
        lines.append(f"  note: {note}")
    lines += ["", "Every command"]
    for n in [hub.root] + hub.root.under():
        for cname, c in n.commands.items():
            lines.append(f"  POST /{(n.path + '/') if n.path else ''}{cname}    {c.description}")
            for pname, p in c.params.items():
                rng = f", {p.minimum} to {p.maximum}" if p.minimum is not None else ""
                dflt = f", default {p.default}" if p.default is not None else ""
                lines.append(f"         {pname:14s} {p.type}{', required' if p.required else ''}{rng}{dflt}: {p.description}")
    lines += ["", "What an arm's action answers",
              "  reached (did it get there), elapsed_s, tip_error_m, message, and its state afterwards. When reached is false,",
              "  why_not_reached says what stopped it. limited lists what the arm's command filter did: it sits between the",
              "  controller and the servos and changes any command the arm must not or cannot follow (a jump spread over time,",
              "  a joint held at its limit, a pose that would put part of the arm into the surface it stands on or into its own base)."]
    return "\n".join(lines) + "\n"


def _picture(node: Node, query: dict, hub: Hub | None = None):
    cam = node.camera
    if cam is None and hub is not None and node is hub.root.children.get("place"):
        return 200, "image/png", hub.place.picture()
    if cam is None:
        raise Refused(f"/{node.path} is not a camera", 404)
    width = query.get("width")
    if width is not None:
        try:
            width = int(width)
            assert 64 <= width <= 4096
        except (ValueError, AssertionError):
            raise Refused("'width' must be a whole number of pixels from 64 to 4096") from None
    want_tags = query.get("tags", "").lower() == "true"
    full = cam.grab()
    image = full.image
    if width is not None and width < image.shape[1]:
        import cv2
        image = cv2.resize(image, (width, int(round(image.shape[0] * width / image.shape[1]))), interpolation=cv2.INTER_AREA)
    data = jpeg(image)
    if not want_tags and query.get("base64", "").lower() != "true":
        return 200, "image/jpeg", data
    out = {"ok": True, "taken_at_s": round(full.t, 3), "width": int(image.shape[1]), "height": int(image.shape[0])}
    if want_tags:
        out["tags"] = cam.tags(full)
        out["tags_are_in"] = f"pixels of the full-size picture ({full.image.shape[1]} x {full.image.shape[0]})"
    if query.get("base64", "").lower() == "true":
        out.update(media_type="image/jpeg", data=base64.b64encode(data).decode())
    return 200, "application/json", json.dumps(out).encode()


def dispatch(hub: Hub, method: str, url: str, body_bytes: bytes = b"") -> tuple[int, str, bytes]:
    """One request -> (status, content type, body). Never raises."""
    try:
        parsed = urlparse(url)
        path = parsed.path.strip("/")
        query = {k: v[-1] for k, v in parse_qs(parsed.query).items()}
        if method not in ("GET", "POST"):
            raise Refused(f"this API takes GET and POST, not {method}", 405)
        if method == "GET":
            if path == "":
                return 200, "text/plain; charset=utf-8", guide(hub).encode()
            if path == "tree":
                result = {"ok": True, "tree": hub.root.outline()}
            elif path == "place":                             # the node, like any other, and what has been found with it
                result = {"ok": True, **hub.node(path).describe(), **hub.place_state(query.get("relative_to"))}
            elif path == "place/state":
                result = {"ok": True, "path": "/place", **hub.place_state(query.get("relative_to"))}
            elif path == "where/pixel":
                try:
                    u, v, h = float(query["u"]), float(query["v"]), float(query.get("height_m", 0.0))
                except (KeyError, ValueError):
                    raise Refused("say which camera and which point of its full-size picture: "
                                  "/where/pixel?camera=camera_1&u=960&v=540&height_m=0.03&relative_to=arm") from None
                if "camera" not in query or not (math.isfinite(u) and math.isfinite(v) and -0.05 <= h <= 1.0):
                    raise Refused("camera, u and v are required; height_m is from -0.05 to 1")
                result = hub.where_pixel(query["camera"], u, v, h, query.get("relative_to", "arm"))
            elif path == "where/point":
                try:
                    x, y, h = float(query["x"]), float(query["y"]), float(query.get("height_m", 0.0))
                except (KeyError, ValueError):
                    raise Refused("say which camera and which place: "
                                  "/where/point?camera=camera_1&x=0.25&y=0.05&height_m=0.03&relative_to=arm") from None
                if "camera" not in query or not (math.isfinite(x) and math.isfinite(y) and abs(x) <= 5.0 and abs(y) <= 5.0 and -0.05 <= h <= 1.0):
                    raise Refused("camera, x and y are required (within 5 m); height_m is from -0.05 to 1")
                result = hub.where_point(query["camera"], x, y, h, query.get("relative_to", "arm"))
            elif path == "where":
                if "of" not in query:
                    raise Refused("say which device: /where?of=camera_1&relative_to=arm")
                result = hub.where(query["of"], query.get("relative_to", "arm"))
            else:
                head, _, last = path.rpartition("/")
                if last == "state":
                    result = {"ok": True, "path": "/" + head, **hub.node(head).state()}
                elif last == "picture":
                    return _picture(hub.node(head), query, hub)
                else:
                    result = {"ok": True, **hub.node(path).describe()}
        else:
            try:
                body = json.loads(body_bytes.decode() or "null")
            except (ValueError, UnicodeDecodeError):
                raise Refused("the request body is not valid JSON") from None
            head, _, last = path.rpartition("/")
            node = hub.node(head)
            if last == "stop":
                result = hub.stop(head)
            elif last in node.commands:
                result = node.commands[last].run(**validate(node.commands[last], body))
            else:
                raise Refused(f"/{head} has no command '{last}'; it has {sorted(node.commands) + ['stop']}", 404)
        return 200, "application/json", json.dumps(result).encode()
    except Refused as e:
        return e.status, "application/json", json.dumps({"ok": False, "error": str(e)}).encode()
    except (BusError, PortError) as e:
        return 503, "application/json", json.dumps({"ok": False, "error": f"a device did not answer: {e}. POST /discover looks "
                                                                          "again for what is plugged in."}).encode()
    except Exception as e:  # noqa: BLE001  (a bug here must reach the caller as JSON, not kill the server)
        return 500, "application/json", json.dumps({"ok": False, "error": f"internal error: {type(e).__name__}: {e}"}).encode()


def make_handler(hub: Hub, verbose: bool = True):
    class Handler(BaseHTTPRequestHandler):
        def _answer(self, method: str):
            n = int(self.headers.get("Content-Length") or 0)
            status, ctype, body = dispatch(hub, method, self.path, self.rfile.read(n) if 0 < n <= 1_000_000 else b"")
            self.send_response(status)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):       # noqa: N802
            self._answer("GET")

        def do_POST(self):      # noqa: N802
            self._answer("POST")

        def do_PUT(self):       # noqa: N802
            self._answer("PUT")

        def do_DELETE(self):    # noqa: N802
            self._answer("DELETE")

        def log_message(self, fmt, *args):
            if verbose:
                sys.stderr.write("%s  %s\n" % (self.log_date_time_string(), fmt % args))
    return Handler
