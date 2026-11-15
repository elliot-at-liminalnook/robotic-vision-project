"""The harness that gives a task to a language model (sim/scripts/model_run.py), run against stand-in models.

Nothing here is sent anywhere and nothing is billed: the stand-in has the Anthropic client's `beta.messages.create`
and answers from this file. What a run with a real model would exercise that this cannot: the model's own judgement,
and the service's errors.

Aimed at classes of bugs:
  * the loop itself: a tool call that is not answered, an answer that does not reach the model, a picture that reaches it
    as text, a run that does not end when the model says it has finished (or never, when it does not);
  * the operator's part done wrongly: answers not handed in, the swap of scenes not made, the referee not asked;
  * the model given something it should not have (the scene, the truth).
"""
import base64
import json
import queue
import threading
from types import SimpleNamespace

import pytest

import model_run as M
from solutions import SOLVERS
from solutions.api import Api, GaveUp


def reply(*blocks, stop="tool_use"):
    return SimpleNamespace(content=list(blocks), stop_reason=stop, stop_details=None,
                           usage=SimpleNamespace(input_tokens=100, output_tokens=10, cache_read_input_tokens=0))


def tool(name, n, **args):
    return SimpleNamespace(type="tool_use", name=name, input=args, id=f"call_{n}")


class Scripted:
    """A model that says what it was told to say, one reply per request, and keeps what it was sent."""
    def __init__(self, replies):
        self.replies, self.sent = list(replies), []
        self.beta = SimpleNamespace(messages=SimpleNamespace(create=self.create))

    def create(self, **request):
        self.sent.append({**request, "messages": list(request["messages"])})        # (as it was when sent: the harness goes on adding to it)
        return self.replies.pop(0) if self.replies else reply(SimpleNamespace(type="text", text="..."), stop="end_turn")


class PlaysASolution:
    """A model that does what the reference solution does: each request the solution makes of the hub's API becomes one
    tool call, and what the harness sends back as the tool's result is what the solution is given."""
    def __init__(self, scene, task):
        self.asks, self.answers, self.n, self.sent = queue.Queue(), queue.Queue(), 0, []
        self.beta = SimpleNamespace(messages=SimpleNamespace(create=self.create))
        api = Api(self._request)

        def work():
            try:
                done = SOLVERS[(scene, task)](api, intermission=lambda: None)
                self.asks.put(("finished", {"answers": done or {}, "note": "done as the reference solution does it"}))
            except GaveUp as e:
                self.asks.put(("finished", {"note": f"gave up: {e}"}))
        self.thread = threading.Thread(target=work, daemon=True)

    def _request(self, method, path, body):
        self.asks.put(("hub_get", {"path": path}) if method == "GET" else ("hub_post", {"path": path, **({"body": json.loads(body)} if body else {})}))
        return self.answers.get(timeout=600)

    def create(self, **request):
        self.sent.append({k: v for k, v in request.items() if k != "messages"})
        if not self.thread.is_alive() and self.n == 0:
            self.thread.start()
        else:                                                       # the last message holds the result of the call just made
            content = request["messages"][-1]["content"][0]["content"][0]
            if content["type"] == "image":
                self.answers.put((200, content["source"]["media_type"], base64.standard_b64decode(content["source"]["data"])))
            else:
                status, _, text = content["text"].partition("\n")
                self.answers.put((int(status), "application/json", text.encode()))
        name, args = self.asks.get(timeout=600)
        self.n += 1
        return reply(tool(name, self.n, **args))


def run(scene, task, client, **more):
    return M.run(scene, task, "stand-in", more.pop("max_calls", 400), "high", more.pop("log", None), more.pop("swap_after", 12), client=client)


@pytest.mark.parametrize("scene,task", [("no_arm", "where_is_everyone"), ("workcell", "stack"), ("moved_arm", "carried")])
def test_a_model_that_does_what_the_solution_does_gets_the_task_done(scene, task, tmp_path):
    """The whole loop: requests go to the hub, results come back (pictures as pictures), the answers are handed in, the
    scene is swapped when the task says so, and the referee is asked. A question, a task judged by how things are left
    (with many pictures on the way), and one where everything is carried elsewhere part-way."""
    client = PlaysASolution(scene, task)
    # (the solution takes 7 requests to say where the rover is; the harness swaps the scene after that many, which is
    # when the solution is ready for it. A real model is not ready: it meets devices that have stopped answering.)
    swap = {"swap_after": 7} if scene == "moved_arm" else {}
    r = run(scene, task, client, log=tmp_path / "run.jsonl", **swap)
    assert r["done"] and r["calls"] == client.n and all(g["holds"] for g in r["goals"]), r
    assert r["tokens"]["input"] == 100 * len(client.sent)
    lines = [json.loads(line) for line in (tmp_path / "run.jsonl").read_text().splitlines()]
    assert lines[0]["kind"] == "start" and lines[-1]["kind"] == "end" and lines[-1]["done"]
    assert sum(line["kind"] == "call" for line in lines) == r["calls"] == sum(line["kind"] == "result" for line in lines)
    if scene == "workcell":
        assert any(line["kind"] == "result" and line["result"][0]["type"] == "image" for line in lines), "no picture reached the model as a picture"
    if scene == "moved_arm":
        assert any(line["kind"] == "swapped" for line in lines) and r["answers"]["moved"] == "yes"
    # what the model was given: the task's words, the hub's guide and the three tools, and nothing of the scene
    first = client.sent[0]
    assert [t["name"] for t in first["tools"]] == ["hub_get", "hub_post", "finished"] and "GET  /tree" in first["system"]
    for word in ("at_m", "truth", "sim/scenes", "looking_at_m"):
        assert word not in first["system"]


def test_what_the_model_is_first_told_is_the_task_and_how_to_answer_it():
    client = Scripted([reply(tool("finished", 1, note="nothing done"))])
    r = run("no_arm", "how_big", client)
    said = client.sent[0]["messages"][0]["content"]
    assert "How large is the surface" in said and "`area_m2` (a number)" in said
    assert not r["done"] and r["note"] == "nothing done" and r["answers"] is None and r["calls"] == 1
    assert any("no answer handed in" in g["detail"] for g in r["goals"])


@pytest.mark.parametrize("replies,words", [
    ([reply(SimpleNamespace(type="text", text="I would rather not"), stop="refusal")], "declined"),
    ([reply(SimpleNamespace(type="text", text="and so"), stop="max_tokens")], "cut off"),
    ([reply(SimpleNamespace(type="text", text="all done, I think"), stop="end_turn")], "without calling"),
    ([reply(tool("hub_get", n, path="/tree")) for n in range(1, 30)], "stopped by this script after 5 calls"),
])
def test_a_run_ends_whatever_the_model_does_and_says_how(replies, words):
    """The class: a loop that waits for `finished` for ever. A refusal, a reply cut off, a model that just stops, and one
    that would go on calling without end."""
    r = run("no_arm", "how_big", Scripted(replies), max_calls=5)
    assert words in r["note"] and not r["done"] and r["calls"] <= 5


def test_a_wrong_call_is_answered_in_words_and_the_run_goes_on():
    """A tool that does not exist, a path that is not one, a request the hub refuses: each gets an answer the model can
    read, in the turn it asked, and the next call still works."""
    client = Scripted([reply(tool("hub_delete", 1, path="/place"), tool("hub_get", 2, path="tree"), tool("hub_post", 3, path="/devices/rover/drive/forward", body={})),
                       reply(tool("hub_get", 4, path="/tree")),
                       reply(tool("finished", 5, note="looked", answers={"area_m2": 0.5}))])
    r = run("no_arm", "how_big", client)
    back = client.sent[1]["messages"][-1]["content"]
    assert [b["tool_use_id"] for b in back] == ["call_1", "call_2", "call_3"], "every call of a turn is answered, in one message"
    texts = [b["content"][0]["text"] for b in back]
    assert "no tool 'hub_delete'" in texts[0] and "must start with /" in texts[1] and texts[2].startswith("400") and "distance_m" in texts[2]
    assert client.sent[2]["messages"][-1]["content"][0]["content"][0]["text"].startswith("200")
    assert r["calls"] == 5 and r["answers"] == {"area_m2": 0.5} and r["done"], r
