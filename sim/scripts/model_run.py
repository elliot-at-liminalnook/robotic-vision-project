#!/usr/bin/env python3
"""Give a task to a language model and let it work through the hub's API; the referee scores what it did.

    python sim/scripts/model_run.py shelf on_the_shelf
    python sim/scripts/model_run.py workcell stack --model claude-opus-5-5 --max-calls 120 --log runs/stack.jsonl

The model is given the task's request in words, the hub's own guide (GET /) and three tools: read from the
hub's API, send it a command, and say it has finished (with its answers, if the task asks a question). It
is given nothing else: not the scene file, not the simulation, not the referee. Pictures come back to it as
pictures. This script plays the operator: it hands in the model's answers and asks the referee whether
the task's goals hold.

Needs the `anthropic` package and credentials (ANTHROPIC_API_KEY, or a profile from `ant auth login`).
Every run is billed to that account: a task takes tens to a few hundred requests, many with pictures.
"""
from __future__ import annotations

import argparse
import base64
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

MODEL = "claude-opus-5-5"
SYSTEM = """You command a small set of devices on a table through a REST API: perhaps a robot arm, fixed cameras, and a small rover \
that only looks. You have been told nothing about the place they are in. Everything you learn about it comes from the API: \
pictures, what the devices report, and what the hub works out from them.

Use the tools to read from the API and to send it commands, one request at a time. Positions are in metres in the frame the \
API says they are in; check which frame before you send a hand somewhere. Uncertainties it states are real. A request that \
is refused says why: read the reason. When the task is done, or you can do no more, call `finished`. If the task asks a \
question, give your answers in `finished` under the keys the task names.

The API's own guide follows.

"""
TOOLS = [
    {"name": "hub_get",
     "description": "GET a path of the hub's API. Returns the JSON it answers with. A path ending in /picture returns the camera's "
                    "picture as an image (add ?width=960 or smaller unless you need full detail: the full picture is 1920 x 1080). "
                    "Examples: /tree, /devices/arm/state, /devices/camera_1/picture?width=960, /place, "
                    "/where?of=rover&relative_to=arm, /where/pixel?camera=camera_1&u=960&v=540&height_m=0.03&relative_to=arm.",
     "input_schema": {"type": "object", "properties": {"path": {"type": "string", "description": "the path and query, starting with /"}},
                      "required": ["path"], "additionalProperties": False}},
    {"name": "hub_post",
     "description": "POST a command to the hub's API. Returns the JSON it answers with: for a motion, whether it reached, and if not, "
                    "why not. Examples: /devices/arm/move_tip with {\"x\": 0.2, \"y\": 0.0, \"z\": 0.1}; /survey with no body; "
                    "/stop.",
     "input_schema": {"type": "object", "properties": {"path": {"type": "string", "description": "the path, starting with /"},
                                                        "body": {"type": "object", "description": "the command's arguments, if it takes any"}},
                      "required": ["path"], "additionalProperties": False}},
    {"name": "finished",
     "description": "Say the task is done, or that you can do no more. Ends the run.",
     "input_schema": {"type": "object", "properties": {
         "answers": {"type": "object", "description": "for a task that asks a question: the answers, under the keys the task names. "
                                                      "A place is {\"x\": metres, \"y\": metres}; a list of colours is a list of words"},
         "note": {"type": "string", "description": "in a sentence or two: what you did, and anything you could not do"}},
         "required": ["note"], "additionalProperties": False}},
]


def answer_keys(task: dict) -> str:
    keys = [(g["key"], g["kind"]) for g in task.get("goals", []) if g["kind"].startswith("answer")]
    if not keys:
        return "This task is judged by how things are left, not by an answer."
    how = {"answer_number": "a number", "answer_text": "a word or two", "answer_list": "a list of words", "answer_pose": "{\"x\": metres, \"y\": metres}"}
    return "Hand in your answers under these keys: " + ", ".join(f"`{k}` ({how[kind]})" for k, kind in keys) + "."


def run(scene: str, task: str, model: str, max_calls: int, effort: str, log_path: Path | None, swap_after: int) -> dict:
    try:
        import anthropic
    except ImportError:
        raise SystemExit("this needs the anthropic package: pip install anthropic") from None

    data = scenes.read(scene)
    spec = next(t for t in data["tasks"] if t["name"] == task)
    world = World(data)
    hub = Hub(world.ports)
    op = operator_api.Operator(world)
    client = anthropic.Anthropic()
    log = log_path.open("w") if log_path else None

    def note(kind: str, **what):
        if log:
            log.write(json.dumps({"t": round(time.time(), 2), "kind": kind, **what}) + "\n")
            log.flush()

    def hub_call(method: str, path: str, body=None) -> list[dict]:
        """One request to the hub, as the content of a tool result."""
        if not isinstance(path, str) or not path.startswith("/"):
            return [{"type": "text", "text": "the path must start with /"}]
        status, ctype, out = hub_api.dispatch(hub, method, path, b"" if body is None else json.dumps(body).encode())
        if ctype.startswith("image/"):
            return [{"type": "image", "source": {"type": "base64", "media_type": ctype, "data": base64.standard_b64encode(bytes(out)).decode()}}]
        text = bytes(out).decode(errors="replace")
        return [{"type": "text", "text": f"{status}\n{text[:60000]}"}]

    guide = hub_api.dispatch(hub, "GET", "/")[2].decode()
    messages = [{"role": "user", "content": f"The task: {spec['request']}\n\n{answer_keys(spec)}"}]
    note("start", scene=scene, task=task, model=model, request=spec["request"])
    calls, answers, said, usage, swapped = 0, None, None, {"input": 0, "output": 0, "cache_read": 0}, False
    t0 = time.time()
    while said is None:
        try:
            response = client.beta.messages.create(
                model=model, max_tokens=16000, system=SYSTEM + guide, tools=TOOLS, messages=messages,
                cache_control={"type": "ephemeral"},                    # the guide and the growing history are a stable prefix
                output_config={"effort": effort},
                betas=["server-side-fallback-2026-07-01"], fallbacks="default",     # a refusal is retried on another model
            )
        except anthropic.RateLimitError as e:
            wait = int(e.response.headers.get("retry-after", "30"))
            note("rate_limited", wait_s=wait)
            time.sleep(wait)
            continue
        except anthropic.APIStatusError as e:
            if e.status_code >= 500:
                note("server_error", status=e.status_code)
                time.sleep(10)
                continue
            raise
        except anthropic.APIConnectionError:
            note("connection_error")
            time.sleep(10)
            continue
        usage["input"] += response.usage.input_tokens
        usage["output"] += response.usage.output_tokens
        usage["cache_read"] += response.usage.cache_read_input_tokens or 0
        messages.append({"role": "assistant", "content": response.content})
        for block in response.content:
            if block.type == "text" and block.text.strip():
                note("says", text=block.text)
        if response.stop_reason == "refusal":
            said = "the model declined: " + str(getattr(response.stop_details, "category", None))
            break
        if response.stop_reason == "max_tokens":
            said = "the model's reply was cut off at the token limit"
            break
        if response.stop_reason != "tool_use":
            said = "the model stopped without calling `finished`"
            break
        results = []
        for block in response.content:
            if block.type != "tool_use":
                continue
            calls += 1
            args = block.input if isinstance(block.input, dict) else {}
            note("call", tool=block.name, input=args)
            if block.name == "finished":
                answers, said = args.get("answers"), str(args.get("note", ""))
                content = [{"type": "text", "text": "noted"}]
            elif block.name == "hub_get":
                content = hub_call("GET", args.get("path"))
            elif block.name == "hub_post":
                content = hub_call("POST", args.get("path"), args.get("body"))
            else:
                content = [{"type": "text", "text": f"there is no tool '{block.name}'"}]
            note("result", tool=block.name, result=[c if c["type"] == "text" else {"type": "image"} for c in content])
            results.append({"type": "tool_result", "tool_use_id": block.id, "content": content})
            if "swap_to" in spec and not swapped and calls >= swap_after:      # everything is picked up and put down elsewhere
                op.load({"scene": spec["swap_to"]})
                swapped = True
                note("swapped", to=spec["swap_to"])
        if calls >= max_calls and said is None:
            said = f"stopped by this script after {calls} calls"
        messages.append({"role": "user", "content": results})                 # every result of this turn, in one message
    if isinstance(answers, dict) and answers:
        op.answer(task, answers)
    world.step(1.0)
    status = op.task(task)
    out = {"scene": scene, "task": task, "model": model, "done": bool(status["done"]), "answers": answers, "note": said,
           "goals": [{"holds": g["done"], "detail": g["detail"]} for g in status["goals"]], "calls": calls, "tokens": usage,
           "simulated_s": round(float(hub.clock.now()), 1), "real_s": round(time.time() - t0, 1)}
    note("end", **out)
    hub.shutdown()
    if log:
        log.close()
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scene")
    ap.add_argument("task")
    ap.add_argument("--model", default=MODEL, help=f"the model's id (default {MODEL})")
    ap.add_argument("--effort", default="high", choices=["low", "medium", "high", "xhigh", "max"], help="how hard it thinks (default high)")
    ap.add_argument("--max-calls", type=int, default=150, help="stop it after this many tool calls (default 150)")
    ap.add_argument("--swap-after", type=int, default=12, help="for a task whose scene is swapped part-way: after this many calls")
    ap.add_argument("--log", type=Path, help="write every step to this file, one JSON object a line")
    a = ap.parse_args()
    if a.scene not in scenes.names() or not any(t["name"] == a.task for t in scenes.read(a.scene).get("tasks", [])):
        ap.error(f"no task '{a.task}' in scene '{a.scene}' (python sim/scripts/solve.py --list shows them)")
    r = run(a.scene, a.task, a.model, a.max_calls, a.effort, a.log, a.swap_after)
    print(f"{'DONE  ' if r['done'] else 'NOT   '}{r['scene']} / {r['task']}   by {r['model']}: {r['calls']} calls, {r['tokens']['input']} tokens in "
          f"({r['tokens']['cache_read']} read from cache), {r['tokens']['output']} out, {r['simulated_s']} s simulated, {r['real_s']} s")
    print(f"      it said: {r['note']}")
    if r["answers"]:
        print(f"      answered: {json.dumps(r['answers'])}")
    for g in r["goals"]:
        print(f"      {'holds' if g['holds'] else 'FAILS'}: {g['detail']}")
    return 0 if r["done"] else 1


if __name__ == "__main__":
    sys.exit(main())
