#!/usr/bin/env python3
"""Run the reference solutions and have the referee score them.

    python sim/scripts/solve.py                       # every task that has a solution
    python sim/scripts/solve.py shelf on_the_shelf    # one
    python sim/scripts/solve.py --list                # which tasks have one, and why the others do not

Each task gets a fresh world. The solution (sim/solutions/) is given the hub's API and nothing else; this
script plays the operator: it hands in what the solution answers and asks the referee whether the task's
goals hold. For a task whose scene is swapped part-way, it does the swap when the solution says it is ready.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import operator_api  # noqa: E402
from so101sim import scenes  # noqa: E402
from so101sim.world import World  # noqa: E402
from hub import api as hub_api  # noqa: E402
from hub.hub import Hub  # noqa: E402
from solutions import NO_SOLUTION, SOLVERS  # noqa: E402
from solutions.api import Api, GaveUp  # noqa: E402


def run(scene: str, task: str, seed: int | None = None) -> dict:
    """One task, start to finish. Returns what the referee says, and how the solution got on."""
    data = scenes.read(scene)
    if seed is not None:
        data.setdefault("imperfections", {})["seed"] = seed
    world = World(data)
    hub = Hub(world.ports)
    op = operator_api.Operator(world)
    spec = next(t for t in data["tasks"] if t["name"] == task)
    api = Api(lambda method, url, body: hub_api.dispatch(hub, method, url, body))

    def intermission():
        if "swap_to" in spec:
            op.load({"scene": spec["swap_to"]})

    t0, out = time.time(), {"scene": scene, "task": task, "request": spec["request"]}
    try:
        answers = SOLVERS[(scene, task)](api, intermission=intermission)
        out["answers"] = answers
        if answers:
            op.answer(task, answers)
    except GaveUp as e:
        out["gave_up"] = str(e)
    world.step(1.0)                                              # let whatever was put down come to rest
    status = op.task(task)
    out.update(done=bool(status["done"]), goals=[{"holds": g["done"], "detail": g["detail"]} for g in status["goals"]],
               api_calls=api.calls, simulated_s=round(float(hub.clock.now()), 1), real_s=round(time.time() - t0, 1))
    hub.shutdown()
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scene", nargs="?")
    ap.add_argument("task", nargs="?")
    ap.add_argument("--list", action="store_true", help="say which tasks have a solution and why the others do not")
    ap.add_argument("--seed", type=int, help="give every camera another lens (the scene's imperfections seed)")
    ap.add_argument("--json", action="store_true", help="print each result as a line of JSON")
    a = ap.parse_args()
    every = [(s, t["name"]) for s in scenes.names() for t in scenes.read(s).get("tasks", [])]
    if a.list:
        for key in every:
            print(f"{key[0]:11s} {key[1]:18s} " + ("has a solution" if key in SOLVERS else "none: " + NO_SOLUTION.get(key, "not written")))
        return 0
    todo = [k for k in every if k in SOLVERS and (a.scene is None or k[0] == a.scene) and (a.task is None or k[1] == a.task)]
    if not todo:
        ap.error("no task with a solution matches (see --list)")
    done = 0
    for scene, task in todo:
        r = run(scene, task, a.seed)
        done += r["done"]
        if a.json:
            print(json.dumps(r), flush=True)
            continue
        print(f"{'DONE  ' if r['done'] else 'NOT   '}{scene} / {task}   ({r['api_calls']} calls, {r['simulated_s']} s simulated, {r['real_s']} s)")
        if "gave_up" in r:
            print(f"      gave up: {r['gave_up']}")
        elif r.get("answers"):
            print(f"      answered: {json.dumps(r['answers'])}")
        for g in r["goals"]:
            print(f"      {'holds' if g['holds'] else 'FAILS'}: {g['detail']}", flush=True)
    print(f"{done} of {len(todo)} done")
    return 0 if done == len(todo) else 1


if __name__ == "__main__":
    sys.exit(main())
