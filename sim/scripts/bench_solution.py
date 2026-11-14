#!/usr/bin/env python3
"""A solution to the "bench" task, made only of REST API calls: the proof that the task can be done.

    python sim/scripts/sim_api.py --scene bench --window      # in one terminal
    python sim/scripts/bench_solution.py                      # in another: watch it tidy the bench

It reads /state before each step and decides from what it finds (where the part is now, which way the
jaws ended up, how high the arm can reach there), so it is a closed loop, not a recording.
sim/tests/test_tasks.py runs the same steps without a network.
"""
from __future__ import annotations

import argparse
import json
import sys
import urllib.error
import urllib.request

HANDLE_X = 0.224      # the drawer's handle when the drawer is closed [m]
CARRY = 0.12          # preferred height to carry things at [m]; lowered where the arm cannot reach it


class Failed(RuntimeError):
    pass


def http_caller(base: str):
    def call(method: str, path: str, body: dict | None = None) -> dict:
        req = urllib.request.Request(base + path, method=method, data=None if body is None else json.dumps(body).encode())
        try:
            with urllib.request.urlopen(req, timeout=300) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as e:
            return json.loads(e.read())
    return call


class Solver:
    def __init__(self, call, log=print):
        self.call, self.log = call, log

    # ---- small steps, each checked ---------------------------------------------
    def state(self) -> dict:
        return self.call("GET", "/state")

    def move(self, x, y, z, need=True, **kw) -> dict:
        r = self.call("POST", "/move_tip", {"x": round(float(x), 4), "y": round(float(y), 4), "z": round(float(z), 4), **kw})
        if need and not (r["ok"] and (r["reached"] or r["tip_error_m"] < 0.006)):
            raise Failed(f"move_tip to ({x:.3f}, {y:.3f}, {z:.3f}) failed: {r.get('error') or r.get('message')} "
                         f"(tip error {r.get('tip_error_m')})")
        return r

    def above(self, x, y, z=CARRY, floor=0.06, **kw) -> float:
        """Go above (x, y) as high as the arm reaches there, from z down to floor. Returns the height used."""
        while z >= floor - 1e-9:
            r = self.move(x, y, z, need=False, **kw)
            if r["ok"] and (r["reached"] or r["tip_error_m"] < 0.006):
                return z
            z -= 0.01
        raise Failed(f"cannot get above ({x:.3f}, {y:.3f}) at any height down to {floor}")

    def grip(self, **kw) -> dict:
        r = self.call("POST", "/gripper", kw)
        if not r["ok"]:
            raise Failed(f"gripper {kw}: {r['error']}")
        return r

    def take(self, name: str, size_x: float, size_y: float, tip_below_centre: float = 0.004, square: bool = True,
             across: float | None = None):
        """Pick a part up by its sides. size_x / size_y: its width along x and y (the jaws go across one of them).
        across: close the jaws along this direction (degrees; 0 = x, 90 = y) instead of whichever is nearer."""
        p = self.state()["objects"][name]
        self.grip(opening=1.0 if max(size_x, size_y) > 0.05 else 0.75)
        how = {"jaws_along_deg": across} if across is not None else ({"square_jaws": True} if square else {})
        h = self.above(p["x"], p["y"], **how)
        along = self.state()["jaws_close_along_deg"]
        width = size_x if (along < 45 or along > 135) else size_y
        self.grip(gap_m=width + 0.014)
        p = self.state()["objects"][name]
        # Not lower than 10 mm: the moving jaw swings a little lower as it closes, and the command filter
        # will not let any part of the arm be sent below the table top, so a tip at the limit cannot close.
        self.move(p["x"], p["y"], max(p["z"] - tip_below_centre, 0.010))
        r = self.grip(opening=0.0)
        gap = r["state"]["gripper"]["gap_m"]
        if not r["holding"] or abs(gap - width) > 0.006:
            raise Failed(f"did not get hold of {name}: holding={r['holding']}, jaws {gap * 1000:.0f} mm apart, part {width * 1000:.0f} mm")
        self.move(p["x"], p["y"], h)
        self.log(f"  holding {name} (jaws {gap * 1000:.0f} mm apart)")
        return name, width

    def put(self, held, x, y, centre_z: float, tip_below_centre: float = 0.004, drop: float = 0.002):
        """Set the held part down with its centre at (x, y, centre_z): go above, lower, let go, back up."""
        name, width = held
        s = self.state()
        off = (s["objects"][name]["x"] - s["tip_m"]["x"], s["objects"][name]["y"] - s["tip_m"]["y"])   # part vs tip, as held
        h = self.above(x - off[0], y - off[1])
        self.move(x - off[0], y - off[1], centre_z - tip_below_centre + drop)
        self.grip(gap_m=min(width + 0.016, 0.08))
        self.call("POST", "/wait", {"seconds": 0.4})
        t = self.state()["tip_m"]
        self.move(t["x"], t["y"], h, need=False)
        p = self.state()["objects"][name]
        self.log(f"  put {name} at ({p['x']:.3f}, {p['y']:.3f}, {p['z']:.3f})")

    def drawer(self, to_x: float):
        """Take the drawer's handle and slide it until the handle is at x = to_x."""
        opening = self.state()["fixtures"]["drawer"]
        hx, hy = HANDLE_X - opening, 0.13
        self.grip(gap_m=0.028)
        # Jaws across the handle in y: closing along x, one jaw would come down on the cabinet's roof.
        self.above(hx, hy, z=0.10, jaws_along_deg=90.0)
        self.move(hx, hy, 0.052)
        r = self.grip(opening=0.0)
        if not r["holding"]:
            raise Failed("did not get hold of the drawer's handle")
        self.move(to_x, hy, 0.052, need=False, timeout_s=8)
        self.grip(gap_m=0.030)
        t = self.state()["tip_m"]
        self.move(t["x"], t["y"], 0.10, need=False)
        self.log(f"  drawer now open by {self.state()['fixtures']['drawer'] * 1000:.0f} mm")

    # ---- the task ----------------------------------------------------------------
    def solve(self) -> dict:
        self.log("green block -> platform")
        # (across its 40 mm length: pinched across its 25 mm width, the low block rolls over onto its side)
        self.put(self.take("green_block", 0.040, 0.025, across=0.0), 0.27, -0.14, 0.050)
        self.log("purple cube (from the cabinet's roof) -> on the green block")
        block = self.state()["objects"]["green_block"]
        self.put(self.take("purple_cube", 0.025, 0.025), block["x"], block["y"], block["z"] + 0.0225)
        self.log("open the drawer")
        self.drawer(HANDLE_X - 0.083)
        self.log("red cube (from the drawer) -> red bin")
        self.put(self.take("red_cube", 0.030, 0.030), 0.10, 0.24, 0.0151, drop=0.004)
        self.log("close the drawer")
        self.drawer(HANDLE_X)
        self.log("blue cylinder -> blue bin")
        self.put(self.take("blue_cylinder", 0.030, 0.030, square=False), 0.13, -0.23, 0.0301, drop=0.003)
        self.log("yellow peg -> socket")
        self.put(self.take("yellow_peg", 0.018, 0.018, tip_below_centre=-0.005, square=False), 0.30, -0.02, 0.0301,
                 tip_below_centre=-0.005, drop=0.003)
        self.log("park")
        self.grip(opening=0.5)
        self.above(0.20, 0.0, z=0.13)
        self.call("POST", "/wait", {"seconds": 1.0})
        task = self.call("GET", "/task")
        for g in task["goals"]:
            self.log(f"  {'done    ' if g['done'] else 'NOT DONE'} {g['goal']}: {g['detail']}")
        return task


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default="http://127.0.0.1:8765", help="where sim_api.py --scene bench is listening")
    a = ap.parse_args()
    call = http_caller(a.url)
    try:
        if call("GET", "/task").get("task") != "bench":
            print("the server is not running the bench scene: start it with  sim_api.py --scene bench", file=sys.stderr)
            return 2
        call("POST", "/reset")
        task = Solver(call).solve()
    except (urllib.error.URLError, ConnectionError) as e:
        print(f"no simulation API at {a.url} ({e}); start it with  sim_api.py --scene bench", file=sys.stderr)
        return 2
    except Failed as e:
        print(f"FAILED: {e}", file=sys.stderr)
        return 1
    print("task done" if task["done"] else "task NOT done")
    return 0 if task["done"] else 1


if __name__ == "__main__":
    sys.exit(main())
