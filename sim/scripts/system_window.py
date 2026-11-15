"""A window on the whole system (run_system.py --window): every camera's view and a status line, drawn while the
simulated world runs, at real speed. With --hands-on a person at the keys takes turns with whoever is calling the
hub's API: each key is one command to the hub, the same command a caller would send, with the same protections.

    arm     W S  tip away / back        A D  tip left / right        R F  tip up / down
            Space  open or close the jaws        H  home
    rover   arrows  forward, back, turn left, turn right        U  raise the head      J  stow it
            , .  turn the head left / right
    Esc     close the window (and stop the system)

The views are drawn straight from the simulation (plain pictures, not through each camera's lens), for the person
watching: nothing here is given to the hub. A key pressed while a command is still running is left alone.
"""
from __future__ import annotations

import json
import math
import time

from hub import api as hub_api

VIEW_W, VIEW_H = 480, 270
TITLE, STATUS = 24, 74            # the strip over each view, and the status lines under them all [px]
FRAME_HZ = 30
TIP_STEP_M, DRIVE_STEP_M, TURN_STEP_DEG, PAN_STEP_DEG, HEAD_UP_M = 0.01, 0.05, 15.0, 20.0, 0.12


class Window:
    def __init__(self, world, hub, hands_on: bool = False):
        import pygame as pg
        self.pg, self.world, self.hub, self.hands_on = pg, world, hub, bool(hands_on)
        pg.init()
        self.font, self.small = pg.font.SysFont(None, 22), pg.font.SysFont(None, 19)
        self.clock = pg.time.Clock()
        self.open, self.busy = True, False
        self.last_request, self.last_key = "", ""
        self._shape = None
        self._lay_out()
        pg.display.set_caption("The hub's system: " + ("watching, and driving from this window" if hands_on else "watching"))
        world.on_frame.append(self.frame)

    def close(self):
        if self.frame in self.world.on_frame:
            self.world.on_frame.remove(self.frame)

    # ---- drawing ------------------------------------------------------------------------
    def _lay_out(self):
        """A grid for the cameras there are now (another scene may have more or fewer)."""
        self.cameras = sorted(self.world.built.cameras)
        cols = max(1, min(len(self.cameras), 2 if len(self.cameras) <= 4 else 3))
        rows = max(1, math.ceil(len(self.cameras) / cols))
        if (cols, rows) != self._shape:
            self._shape = (cols, rows)
            self.screen = self.pg.display.set_mode((cols * VIEW_W, rows * (VIEW_H + TITLE) + STATUS))

    def said(self, text: str):
        """What was last asked of the hub's API, for the status line."""
        self.last_request = text[:110]

    def frame(self, idle: bool = False):
        """Draw once. idle: nothing is being done (the main loop's call), so in hands-on mode a key is acted on."""
        pg = self.pg
        for e in pg.event.get():
            if e.type == pg.QUIT or (e.type == pg.KEYDOWN and e.key == pg.K_ESCAPE):
                self.open = False
            elif e.type == pg.KEYDOWN and self.hands_on and idle and not self.busy:
                self.key(e.key)
        if sorted(self.world.built.cameras) != self.cameras:
            self._lay_out()
        cols, rows = self._shape
        self.screen.fill((24, 24, 28))
        for k, name in enumerate(self.cameras):
            x, y = (k % cols) * VIEW_W, (k // cols) * (VIEW_H + TITLE)
            picture = self.world.render(name, VIEW_W, VIEW_H, ideal=True)          # (plain: it does not touch what the hub's cameras give)
            self.screen.blit(self.small.render(name, True, (210, 210, 210)), (x + 8, y + 5))
            self.screen.blit(pg.surfarray.make_surface(picture.swapaxes(0, 1)), (x, y + TITLE))
        y = rows * (VIEW_H + TITLE) + 6
        found = ", ".join(self.hub.found()["devices"]) or "nothing"
        lines = [(f"scene '{self.world.name}'    t = {self.world.time:7.2f} s    the hub has found: {found}", (230, 230, 230)),
                 ("last request: " + self.last_request if self.last_request else "no request yet", (140, 220, 255)),
                 ((self.last_key or "keys: W S A D R F  Space  H  |  arrows  U J  , .  |  Esc") if self.hands_on
                  else "watching only (run with --hands-on to drive from here)", (255, 215, 140))]
        for n, (text, colour) in enumerate(lines):
            self.screen.blit(self.small.render(text[:160], True, colour), (8, y + n * 21))
        pg.display.flip()
        self.clock.tick(FRAME_HZ)

    # ---- a person at the keys -----------------------------------------------------------
    def _ask(self, method: str, path: str, body=None):
        status, ctype, out = hub_api.dispatch(self.hub, method, path, b"" if body is None else json.dumps(body).encode())
        return status, (json.loads(out) if ctype.startswith("application/json") else {})

    def command(self, key: int):
        """The command a key stands for, as (path, body), worked out from what the hub says of its devices now; or None."""
        pg, have = self.pg, self.hub.found()["devices"]
        if "arm" in have:
            step = {pg.K_w: (TIP_STEP_M, 0, 0), pg.K_s: (-TIP_STEP_M, 0, 0), pg.K_a: (0, TIP_STEP_M, 0), pg.K_d: (0, -TIP_STEP_M, 0),
                    pg.K_r: (0, 0, TIP_STEP_M), pg.K_f: (0, 0, -TIP_STEP_M)}.get(key)
            if step is not None or key == pg.K_SPACE:
                state = self._ask("GET", "/devices/arm/state")[1]
                if step is not None:
                    tip = state["tip_m"]
                    return "/devices/arm/move_tip", {"x": round(tip["x"] + step[0], 4), "y": round(tip["y"] + step[1], 4),
                                                     "z": round(tip["z"] + step[2], 4), "tolerance_m": 0.006, "timeout_s": 3.0}
                return "/devices/arm/gripper/set", {"opening": 0.0 if state["gripper"]["target_opening"] > 0.5 else 1.0}
            if key == pg.K_h:
                return "/devices/arm/home", {}
        if "rover" in have:
            drive = {pg.K_UP: ("forward", {"distance_m": DRIVE_STEP_M}), pg.K_DOWN: ("forward", {"distance_m": -DRIVE_STEP_M}),
                     pg.K_LEFT: ("turn", {"angle_deg": TURN_STEP_DEG}), pg.K_RIGHT: ("turn", {"angle_deg": -TURN_STEP_DEG})}.get(key)
            if drive is not None:
                return f"/devices/rover/drive/{drive[0]}", drive[1]
            if key == pg.K_u:
                return "/devices/rover/head/raise", {"lens_height_m": HEAD_UP_M}
            if key == pg.K_j:
                return "/devices/rover/head/stow", {}
            if key in (pg.K_COMMA, pg.K_PERIOD):
                pan = self._ask("GET", "/devices/rover/head/state")[1].get("pan_deg", 0.0)
                return "/devices/rover/head/point", {"pan_deg": round(pan + (PAN_STEP_DEG if key == pg.K_COMMA else -PAN_STEP_DEG), 1)}
        return None

    def key(self, key: int):
        """Do what a key stands for, through the hub's API, and say on the status line how it went."""
        what = self.command(key)
        if what is None:
            return
        path, body = what
        self.busy = True                                             # (frames are drawn while it runs; keys wait)
        try:
            status, r = self._ask("POST", path, body)
        finally:
            self.busy = False
        said = r.get("error") or r.get("why_not_reached") or r.get("note") or ("done" if r.get("reached", True) else "not reached")
        self.last_key = f"key: POST {path} {json.dumps(body)} -> {status}: {said}"


def serve(window: Window, servers, period_s: float = 0.004):
    """Answer requests and keep the window drawn until it is closed."""
    for s in servers:
        s.timeout = period_s
    while window.open:
        for s in servers:
            s.handle_request()
        window.frame(idle=True)
        time.sleep(0)
