"""The rover as a device in the hub: wheels with odometry, the head, power; their commands and protections.

    rover = RoverDevice(port, clock)
    rover.start()                         # identify its six servos, set their modes, hold where it is
    rover.forward(0.10)                   # metres, counted from its wheels
    rover.turn(90)                        # degrees, on the spot
    rover.point(pan_deg=30, tilt_deg=-20) ; rover.raise_head(0.12) ; rover.spread(0.15) ; rover.stow()

Protections, all from what the rover itself knows:
  * it drives only with the head stowed (raised and wide it is top-heavy), at capped speed and acceleration;
  * the head is never sent to where its own shape says it would touch the surface it stands on;
  * a head joint that stays behind its goal has met something: the goal is pulled back to where it is;
  * a wheel servo at its torque limit has met something: the wheels are stopped;
  * below 3.3 V per cell nothing moves; a link that stops answering is an error, and the rover's own
    board stops the wheels by itself.
The place it is in plays no part yet: keeping away from what the map shows comes with the map.

Known and not solved here: the rover is light. A head that meets something solid turns the whole
rover instead of stopping (its wheels skid sideways, which they cannot count), and wheels that spin
against something are counted as distance. Only looking tells; the head's servos are held to a
third of their torque so that such a push is at least gentle.
"""
from __future__ import annotations

import math

import numpy as np

from ..bus import BusError, FeetechBus
from ..bus.group import Servo, ServoGroup
from ..arm.config import load_servos
from ..errors import Refused
from ..tree import Command, Node, Param
from .description import HEAD_JOINTS, STOWED, Rover

TICK_S = 0.05                              # control tick over the radio link
V_MAX, A_MAX = 0.08, 0.25                  # m/s, m/s2 along the ground
PUSH_LEARN, PUSH_LOAD, PUSH_S = 8, 0.010, 0.5   # ticks at a steady speed to learn what rolling takes; this much more load (of 1),
#                                            for this long, is pushing. Set from the simulation: to be set again on the real rover
V_SLOW, W_SLOW = 0.025, 0.3                # ... and into what has never been seen: what is there is met at a crawl
W_MAX = 1.0                                # rad/s turning on the spot (its heading)
HEAD_RATE = {"pan": 1.2, "lift": 0.8, "tilt": 1.2, "spacing": 0.03}        # rad/s, rad/s, rad/s, m/s
GROUND_CLEAR_M = 0.006                     # the head's lowest point is kept this far above the surface
STOW_TOL = {"pan": math.radians(4), "lift": math.radians(4), "tilt": math.radians(6), "spacing": 0.004}
LAG = {"pan": math.radians(12), "lift": math.radians(12), "tilt": math.radians(12), "spacing": 0.008}
LAG_S = 0.5
LOW_CELL_V, CELLS = 3.3, 3
GONE_AFTER = 6                             # reads failing in a row: the rover is not answering
HEAD_TORQUE_LIMIT = 350                    # of 1000: the head's servos push with about a third of what they could.
PAN_TORQUE_LIMIT = 50                      # of 1000. The pan carries no weight, and it swings the head's whole width into places no
                                           # camera has looked at yet. At 35 % a head that met a post turned the rover right round
                                           # (175 deg) and every move reported "reached"; at 5 % the pan stalls against less than the
                                           # tyres' grip, the lag is noticed, and the rover is turned 3 to 5 deg. At 2.5 % it cannot
                                           # reliably move the head at all.
#                                            The head weighs 0.2 kg; at full torque a servo shoves the whole rover aside
NUM, BOOL = "number", "boolean"
ABOUT = {
    "frame": "The rover's own frame: origin on the surface under the middle of its wheel axle, x forward, y to its left, z up. "
             "Metres; angles in degrees in commands.",
    "drive": "Two driven wheels at the back, a ball caster at the front. It moves forward or back and turns on the spot. Distances "
             "and headings come from its wheels, so they drift: check them against what cameras see.",
    "head": "Two cameras on a head that pans (left positive), tilts (up positive), rises (lens height 0.03 to 0.16 m) and "
            "spreads its cameras (0.06 to 0.20 m between lenses). It drives only with the head stowed: forward, low, cameras together.",
    "tags": "Carries printed tags 10 to 13 on its chassis (top, rear, left, right) and tag 14 on its head.",
}


class RoverDevice:
    kind = "rover"

    def __init__(self, port, clock):
        cfg = load_servos().get("bus", {})
        slow = float(getattr(port, "reply_time_s", 0.0))
        self.bus = FeetechBus(port, reply_timeout_s=float(cfg.get("reply_timeout_s", 0.02)) + slow, retries=int(cfg.get("retries", 1)))
        self.clock, self.rover = clock, Rover()
        self.wheel_ids = {side: self.rover.servo(f"wheel_{side}")["id"] for side in ("left", "right")}
        self.head_servo = {j: self.rover.servo(j) for j in HEAD_JOINTS}
        servos = [Servo(self.wheel_ids["left"], "left wheel", motor=True, direction=self.rover.servo("wheel_left")["direction"]),
                  Servo(self.wheel_ids["right"], "right wheel", motor=True, direction=self.rover.servo("wheel_right")["direction"])]
        servos += [Servo(s["id"], j, zero_ticks=s["zero_ticks"], direction=s["direction"]) for j, s in self.head_servo.items()]
        self.group = ServoGroup(self.bus, servos)
        self.cameras: dict[str, object] = {}                 # "left", "right" -> hub.camera.Camera
        self.message = "ready"
        self.torque_on = False
        self._failed = 0

    # ---- start-up ------------------------------------------------------------
    def start(self):
        """Identify the servos (reads only), set the wheels to motor mode, and hold the head where it is."""
        self.group.connect()
        self.group.configure()
        for s in self.head_servo.values():
            self.group._ensure(s["id"], "max_torque_limit", PAN_TORQUE_LIMIT if s is self.head_servo.get("pan") else HEAD_TORQUE_LIMIT)
        fb = self._read()
        self.head_goal = self._head_from(fb)
        self.group.write_angles({self.head_servo[j]["id"]: self.head_goal[j] / self.head_servo[j]["ratio"] for j in HEAD_JOINTS})
        self.group.torque(True)
        self.torque_on = True
        self.pose = np.zeros(3)                               # x, y, heading since it was taken over, from its wheels
        self._wheel_ticks = {s: fb[i]["ticks"] for s, i in self.wheel_ids.items()}
        self.travel = {"left": 0.0, "right": 0.0}             # metres each wheel has rolled
        self.volts = float(np.mean([f["volts"] for f in fb.values()]))
        self._lag_t = 0.0
        self.guard = self.after_drive = None                 # set by the hub: what the map says of a drive (hub/guard.py)
        self.last_block = None

    # ---- sensing ---------------------------------------------------------------
    def _read(self) -> dict:
        try:
            fb = self.group.read()
            self._failed = 0
        except BusError as e:
            self._failed += 1
            if self._failed >= GONE_AFTER:
                raise BusError(f"the rover has not answered {self._failed} reads in a row [{e}]") from e
            raise _Missed() from e
        self.volts = float(np.mean([f["volts"] for f in fb.values()]))
        return fb

    def _read_sure(self) -> dict:
        """A reading, trying again over a lost reply. Raises BusError once the rover has stopped answering."""
        while True:
            try:
                return self._read()
            except _Missed:
                continue

    def _head_from(self, fb) -> dict:
        return {j: fb[s["id"]]["angle"] * s["ratio"] for j, s in self.head_servo.items()}

    def _odometry(self, fb):
        """Count each wheel's turning (its position reads within one turn) and move the pose by it."""
        step = {}
        for side, i in self.wheel_ids.items():
            d = (fb[i]["ticks"] - self._wheel_ticks[side] + 2048) % 4096 - 2048
            self._wheel_ticks[side] = fb[i]["ticks"]
            step[side] = self.group.by_id[i].direction * d * 2 * math.pi / 4096 * self.rover.wheel_radius
            self.travel[side] += step[side]
        ds, dth = (step["left"] + step["right"]) / 2, (step["right"] - step["left"]) / self.rover.track
        th = self.pose[2] + dth / 2
        self.pose += [ds * math.cos(th), ds * math.sin(th), dth]

    def _tick(self) -> dict | None:
        """One control tick: wait, then read everything. None if the read was lost (the next tick tries again)."""
        self.clock.sleep_until(self.clock.now() + TICK_S)
        try:
            fb = self._read()
        except _Missed:
            return None
        self._odometry(fb)
        return fb

    # ---- state -----------------------------------------------------------------
    def head_now(self) -> dict:
        return self._head_from(self._read_sure())

    def stowed(self, head: dict | None = None) -> bool:
        head = head or self.head_now()
        return all(abs(head[j] - STOWED[j]) <= STOW_TOL[j] for j in HEAD_JOINTS)

    def drive_state(self) -> dict:
        self._odometry(self._read_sure())
        return {"since_taken_over": {"x_m": round(float(self.pose[0]), 4), "y_m": round(float(self.pose[1]), 4),
                                     "heading_deg": round(math.degrees(float(self.pose[2])), 2)},
                "wheels_rolled_m": {k: round(v, 4) for k, v in self.travel.items()},
                "note": "counted from the wheels: it drifts, and slipping wheels are counted as moving"}

    def head_state(self) -> dict:
        h = self.head_now()
        q = {j: h[j] for j in HEAD_JOINTS}
        return {"pan_deg": round(math.degrees(h["pan"]), 2), "tilt_deg": round(math.degrees(h["tilt"]), 2),
                "lift_deg": round(math.degrees(h["lift"]), 2), "lens_height_m": round(self.rover.lens_height(q), 4),
                "lens_spacing_m": round(self.rover.lens_spacing(q), 4), "stowed": self.stowed(h)}

    def power_state(self) -> dict:
        self._read_sure()
        return {"volts": round(self.volts, 2), "volts_per_cell": round(self.volts / CELLS, 2), "low": self.low_battery(),
                "note": f"a {CELLS}-cell pack; nothing moves below {LOW_CELL_V} V per cell"}

    def own_state(self) -> dict:
        return {"message": self.message, "torque": self.torque_on}

    def low_battery(self) -> bool:
        return self.volts < LOW_CELL_V * CELLS

    # ---- protections -------------------------------------------------------------
    def _may_move(self):
        if not self.torque_on:
            raise Refused("the rover's torque is switched off, so nothing was moved", 409)
        self._read_sure()
        if self.low_battery():
            raise Refused(f"the rover's battery is at {self.volts:.1f} V ({self.volts / CELLS:.2f} V per cell): too low to move. "
                          "It needs charging.", 409)

    # ---- the head ----------------------------------------------------------------
    def _move_head(self, target: dict, timeout_s: float) -> dict:
        self._may_move()
        lim = self.rover.limits
        target = {j: float(np.clip(target.get(j, self.head_goal[j]), lim[j][0], lim[j][1])) for j in HEAD_JOINTS}
        t0, self.last_block, held_back = self.clock.now(), None, None
        self._lag_t = 0.0
        reached = False
        while self.clock.now() - t0 < timeout_s:
            step = {j: self.head_goal[j] + float(np.clip(target[j] - self.head_goal[j], -HEAD_RATE[j] * TICK_S, HEAD_RATE[j] * TICK_S))
                    for j in HEAD_JOINTS}
            if self.rover.lowest(step) < GROUND_CLEAR_M and self.rover.lowest(step) < self.rover.lowest(self.head_goal) - 1e-6:
                held_back = (f"the head would come within {GROUND_CLEAR_M * 1000:.0f} mm of the surface the rover stands on "
                             "(its own shape says so): raise it before tilting it down further")
                break
            self.head_goal = step
            self.group.write_angles({self.head_servo[j]["id"]: step[j] / self.head_servo[j]["ratio"] for j in HEAD_JOINTS})
            fb = self._tick()
            if fb is None:
                continue
            now = self._head_from(fb)
            behind = {j: abs(self.head_goal[j] - now[j]) for j in HEAD_JOINTS}
            worst = max(HEAD_JOINTS, key=lambda j: behind[j] / LAG[j])
            self._lag_t = self._lag_t + TICK_S if behind[worst] > LAG[worst] else 0.0
            if self._lag_t >= LAG_S:                           # it has met something: stop pushing
                self.last_block = worst
                self.head_goal = dict(now)
                self.group.write_angles({self.head_servo[j]["id"]: now[j] / self.head_servo[j]["ratio"] for j in HEAD_JOINTS})
                break
            if all(abs(target[j] - now[j]) <= STOW_TOL[j] / 2 for j in HEAD_JOINTS) and self.head_goal == target:
                reached = True
                break
        self.message = ("head: there" if reached else f"head: {self.last_block} met something and stopped" if self.last_block
                        else "head: held back" if held_back else "head: ran out of time")
        out = {"ok": True, "reached": reached, "elapsed_s": round(self.clock.now() - t0, 3), "message": self.message,
               "state": self.head_state()}
        if not reached:
            out["why_not_reached"] = (held_back or (f"{self.last_block} stayed behind its goal: something is in the head's way. It holds "
                                                    "where it is." if self.last_block else f"the {timeout_s:g} s allowed ran out"))
        return out

    def settle(self, timeout_s: float = 2.5) -> bool:
        """Wait until the head is still: every joint within a tick and a half of where it was over the last 0.3 s.
        A move is 'reached' once it is within a few degrees, and the head goes on swinging for a moment after. The two
        cameras are separate boards and do not take their pictures at the same instant: from a head still moving, one
        picture is several pixels beside the other, and everything measured across the pair is that much out."""
        tol = {j: (0.0003 if j == "spacing" else 1.5 * 2 * math.pi / 4096 * abs(self.head_servo[j]["ratio"])) for j in HEAD_JOINTS}
        seen, t0 = [], self.clock.now()
        while self.clock.now() - t0 < timeout_s:
            fb = self._tick()
            if fb is None:
                continue
            seen.append(self._head_from(fb))
            last = seen[-max(2, int(round(0.3 / TICK_S))):]
            if len(last) * TICK_S >= 0.3 and all(max(h[j] for h in last) - min(h[j] for h in last) <= tol[j] for j in HEAD_JOINTS):
                return True
        return False

    def point(self, pan_deg=None, tilt_deg=None, timeout_s=6.0) -> dict:
        t = {}
        if pan_deg is not None:
            t["pan"] = math.radians(pan_deg)
        if tilt_deg is not None:
            t["tilt"] = math.radians(tilt_deg)
        if not t:
            raise Refused("give pan_deg, tilt_deg or both")
        return self._move_head(t, timeout_s)

    def raise_head(self, lens_height_m, timeout_s=6.0) -> dict:
        return self._move_head({"lift": self.rover.lift_for_height(float(lens_height_m))}, timeout_s)

    def spread(self, lens_spacing_m, timeout_s=6.0) -> dict:
        closest = self.rover.lens_spacing({})
        return self._move_head({"spacing": (float(lens_spacing_m) - closest) / 2}, timeout_s)

    def stow(self, timeout_s=10.0) -> dict:
        """Forward, level, cameras together, then down: the order that cannot put the head on the ground."""
        first = self._move_head({"tilt": 0.0, "spacing": 0.0}, timeout_s)
        if not first["reached"]:
            return first
        second = self._move_head({"pan": 0.0}, timeout_s)
        if not second["reached"]:
            return second
        return self._move_head(dict(STOWED), timeout_s)

    # ---- the wheels ------------------------------------------------------------------
    def _wheels(self, v: float, w: float):
        """Ground speed v [m/s] and turning rate w [rad/s], as wheel speeds."""
        r, half = self.rover.wheel_radius, self.rover.track / 2
        self.group.write_speeds({self.wheel_ids["left"]: (v - w * half) / r, self.wheel_ids["right"]: (v + w * half) / r})

    def _drive(self, distance: float, angle: float, timeout_s: float, slow: bool = False) -> dict:
        self._may_move()
        if not self.stowed():
            raise Refused("the rover drives only with its head stowed (forward, low, cameras together): it is top-heavy otherwise. "
                          "POST .../head/stow first.", 409)
        t0, start, s0 = self.clock.now(), self.pose.copy(), dict(self.travel)
        turning = distance == 0.0
        goal = abs(angle) if turning else abs(distance)
        sign = math.copysign(1.0, angle if turning else distance)
        v_cap, a_cap = ((W_SLOW if slow else W_MAX), 3.0) if turning else ((V_SLOW if slow else V_MAX), A_MAX)
        speed, stalled_t, self.last_block, reached = 0.0, 0.0, None, False
        cruise, rolling, pushing_t, onset = [], None, 0.0, None
        try:
            while self.clock.now() - t0 < timeout_s:
                done = abs(self.pose[2] - start[2]) if turning else abs(((self.travel["left"] - s0["left"]) + (self.travel["right"] - s0["right"])) / 2)
                left = goal - done
                if left <= (0.004 if turning else 0.001):
                    reached = True
                    break
                want = min(v_cap, math.sqrt(2 * a_cap * max(left, 0.0)) * 0.9)
                speed = min(want, speed + a_cap * TICK_S)
                if turning:
                    self._wheels(0.0, sign * speed)
                else:                                          # hold the heading: steer back the difference between the wheels
                    drift = (self.travel["right"] - s0["right"]) - (self.travel["left"] - s0["left"])
                    self._wheels(sign * speed, float(np.clip(-4.0 * drift / self.rover.track, -0.3, 0.3)))
                fb = self._tick()
                if fb is None:
                    continue
                if self.low_battery():
                    self.last_block = "battery"
                    break
                load = max(abs(fb[i]["load"]) for i in self.wheel_ids.values())
                stalled_t = stalled_t + TICK_S if load > 0.9 else 0.0
                if stalled_t >= 0.4:                           # a wheel servo at its limit: something is in the way
                    self.last_block = "wheels"
                    break
                # Against something low with the tyres slipping, the servos are nowhere near their limit: they only work a
                # little harder than rolling takes. Once at a steady speed, what rolling takes is measured, and a load that
                # stays above it means the rover is pushing, not going. What the wheels counted since then is taken back.
                if speed >= v_cap - 1e-9:
                    cruise.append(float(np.mean([abs(fb[i]["load"]) for i in self.wheel_ids.values()])))
                    if len(cruise) == PUSH_LEARN:
                        rolling = float(np.median(cruise[2:]))
                    elif rolling is not None:
                        if float(np.mean(cruise[-5:])) > rolling + PUSH_LOAD:
                            if pushing_t == 0.0:
                                onset = (self.pose.copy(), dict(self.travel), dict(self._last_ticks) if hasattr(self, "_last_ticks") else None)
                            pushing_t += TICK_S
                        else:
                            pushing_t = 0.0
                        if pushing_t >= PUSH_S:
                            self.pose, self.travel = onset[0], onset[1]
                            self.last_block = "pushing"
                            break
                else:
                    cruise, pushing_t = [], 0.0
        finally:
            try:
                self._wheels(0.0, 0.0)
            except BusError:
                pass                                           # the link is gone; the rover's own board stops the wheels
        for _ in range(4):                                     # let it come to rest, still counting
            self._tick()
        moved = self.pose - start
        if self.last_block == "pushing":                       # (the ticks while it came to rest were not travel either)
            self.pose, self.travel = onset[0], onset[1]
        moved = self.pose - start
        self.message = ("drive: there" if reached else "drive: the battery ran low" if self.last_block == "battery"
                        else "drive: the wheels met something and stopped" if self.last_block else "drive: ran out of time")
        out = {"ok": True, "reached": reached, "elapsed_s": round(self.clock.now() - t0, 3), "message": self.message,
               "moved": {"forward_m": round(float(moved[0] * math.cos(start[2]) + moved[1] * math.sin(start[2])), 4),
                         "turned_deg": round(math.degrees(float(moved[2])), 2)},
               "state": self.drive_state()}
        if not reached:
            out["why_not_reached"] = ("a wheel servo was at its torque limit: the rover is against something" if self.last_block == "wheels"
                                      else "the wheels were working harder than rolling takes: the rover is against something and its "
                                           "tyres were slipping. What they counted while slipping has been taken back" if self.last_block == "pushing"
                                      else "the battery fell below its limit" if self.last_block == "battery" else f"the {timeout_s:g} s allowed ran out")
        return out

    def _guarded(self, distance: float, angle: float, timeout_s) -> dict:
        """A drive as the map allows it (hub/guard.py): not into what has been seen there, slowly into what has not."""
        self._may_move()
        asked, slow, note = (distance, angle), False, None
        if self.guard is not None and self.stowed():
            distance, angle, slow, note = self.guard(distance, angle)
        turning = distance == 0.0
        speed = (W_SLOW if slow else W_MAX) if turning else (V_SLOW if slow else V_MAX)
        out = self._drive(distance, angle, timeout_s or abs(angle if turning else distance) / speed * 1.5 + 3.0, slow=slow)
        if note is not None:
            out["limited"] = note
            if (distance, angle) != asked:                         # it went as far as the map allows, which is not where it was sent
                out["reached"] = False
                out["why_not_reached"] = note
        if self.after_drive is not None:
            self.after_drive(0 if turning else int(math.copysign(1, distance)), self.last_block in ("wheels", "pushing"))
        return out

    def forward(self, distance_m, timeout_s=None) -> dict:
        return self._guarded(float(distance_m), 0.0, timeout_s)

    def turn(self, angle_deg, timeout_s=None) -> dict:
        a = math.radians(float(angle_deg))
        if a == 0.0:
            raise Refused("an angle of 0 is no turn")
        return self._guarded(0.0, a, timeout_s)

    # ---- stopping ------------------------------------------------------------------------
    def stop(self):
        """Wheels stopped, head held where it is."""
        self._wheels(0.0, 0.0)
        now = self.head_now()
        self.head_goal = dict(now)
        self.group.write_angles({self.head_servo[j]["id"]: now[j] / self.head_servo[j]["ratio"] for j in HEAD_JOINTS})
        self.message = "STOPPED: wheels stopped, head held"

    def shutdown(self) -> str:
        try:
            self.torque_on = False
            self.group.write_speeds({i: 0.0 for i in self.wheel_ids.values()})
            self.group.torque(False)
            return "torque off on every rover servo"
        except Exception as e:  # noqa: BLE001
            return f"ROVER TORQUE OFF NOT CONFIRMED: {e}"

    # ---- its place in the tree ---------------------------------------------------------------
    def node(self, name: str = "rover") -> Node:
        n = Node(name, "device", "rover: two driven wheels and a camera head that pans, tilts, rises and spreads its two cameras",
                 state=self.own_state, stop=self.stop, about={"device": "rover", **ABOUT})
        d = n.add(Node("drive", "part", "the wheels: move and turn, and how far the wheels say it has gone", state=self.drive_state))
        d.commands["forward"] = Command("Drive straight ahead (negative: back) by a distance counted from the wheels.", self.forward, {
            "distance_m": Param(NUM, "metres; negative drives backward", True, -1.0, 1.0),
            "timeout_s": Param(NUM, "give up after this much time", minimum=0.5, maximum=60)})
        d.commands["turn"] = Command("Turn on the spot (positive: to its left) by an angle counted from the wheels.", self.turn, {
            "angle_deg": Param(NUM, "degrees; positive turns left", True, -360.0, 360.0),
            "timeout_s": Param(NUM, "give up after this much time", minimum=0.5, maximum=60)})
        lim = self.rover.limits
        h = n.add(Node("head", "part", "the camera head: pan, tilt, height and the spacing of its two cameras", state=self.head_state))
        h.commands["point"] = Command("Turn the head to a pan and tilt (either may be left out to keep it).", self.point, {
            "pan_deg": Param(NUM, "left positive", minimum=round(math.degrees(lim["pan"][0]), 1), maximum=round(math.degrees(lim["pan"][1]), 1)),
            "tilt_deg": Param(NUM, "up positive", minimum=round(math.degrees(lim["tilt"][0]), 1), maximum=round(math.degrees(lim["tilt"][1]), 1)),
            "timeout_s": Param(NUM, "give up after this much time", minimum=0.5, maximum=30, default=6.0)})
        h.commands["raise"] = Command("Raise or lower the head so that the lenses are at a height above the surface the rover stands on.",
                                      self.raise_head, {"lens_height_m": Param(NUM, "metres", True, 0.02, 0.17),
                                                        "timeout_s": Param(NUM, "give up after this much time", minimum=0.5, maximum=30, default=6.0)})
        h.commands["spread"] = Command("Set the distance between the two lenses.", self.spread, {
            "lens_spacing_m": Param(NUM, "metres", True, 0.06, 0.20),
            "timeout_s": Param(NUM, "give up after this much time", minimum=0.5, maximum=30, default=6.0)})
        h.commands["stow"] = Command("Put the head forward, level, low, cameras together: how it must be to drive.", self.stow,
                                     {"timeout_s": Param(NUM, "give up after this much time", minimum=0.5, maximum=30, default=10.0)})
        n.add(Node("power", "part", "the battery, as its servos see it", state=self.power_state))
        return n


class _Missed(Exception):
    """One read was lost: not yet a reason to stop."""
