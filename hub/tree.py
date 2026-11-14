"""The tree of interests: every node is something a caller can be interested in.

    devices/arm            the whole arm              groups/cameras     every camera, wherever it is mounted
    devices/arm/gripper    one part of it             place              what has been found of the surroundings

Reading a node gives the state of everything under it; a command given to a node reaches everything
under it (stop at devices/arm stops the arm; stop at the root stops everything). A leaf belongs to
one device and may also appear in any number of groups.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable


@dataclass
class Param:
    type: str                              # "number", "boolean" or "string"
    description: str
    required: bool = False
    minimum: float | None = None
    maximum: float | None = None
    default: object = None


@dataclass
class Command:
    description: str
    run: Callable[..., dict]
    params: dict[str, Param] = field(default_factory=dict)
    takes_time: bool = True                # does the clock move while it runs?


class Node:
    def __init__(self, name: str, kind: str, summary: str, state: Callable[[], dict] | None = None,
                 stop: Callable[[], None] | None = None, about: dict | None = None):
        self.name, self.kind, self.summary = name, kind, summary
        self.parent: Node | None = None
        self.children: dict[str, Node] = {}
        self.commands: dict[str, Command] = {}
        self.links: dict[str, Node] = {}                # members of a group: nodes that live elsewhere in the tree
        self.camera = None                              # a hub.camera.Camera, on camera nodes
        self._state, self._stop, self.about = state, stop, dict(about or {})

    def add(self, child: "Node") -> "Node":
        child.parent = self
        self.children[child.name] = child
        return child

    @property
    def path(self) -> str:
        parts, n = [], self
        while n is not None and n.parent is not None:
            parts.append(n.name)
            n = n.parent
        return "/".join(reversed(parts))

    def find(self, path: str) -> "Node | None":
        node = self
        for part in [p for p in path.split("/") if p]:
            node = node.children.get(part) or node.links.get(part)
            if node is None:
                return None
        return node

    def under(self) -> list["Node"]:
        """Every node below this one, members of groups included, each once."""
        seen, out, todo = set(), [], list(self.children.values()) + list(self.links.values())
        while todo:
            n = todo.pop(0)
            if id(n) in seen:
                continue
            seen.add(id(n))
            out.append(n)
            todo += list(n.children.values()) + list(n.links.values())
        return out

    def state(self) -> dict:
        """What the sensors under this node say now."""
        out = dict(self._state()) if self._state is not None else {}
        for name, child in {**self.children, **self.links}.items():
            out[name] = child.state()
        return out

    def stop(self) -> list[str]:
        """Freeze everything under this node. Returns the paths of what was stopped."""
        done = []
        for n in [self] + self.under():
            if n._stop is not None:
                n._stop()
                done.append(n.path or "/")
        return done

    def describe(self) -> dict:
        return {
            "path": "/" + self.path, "kind": self.kind, "what": self.summary, **self.about,
            "commands": {name: {"what": c.description, "takes_time": c.takes_time,
                                "parameters": {p: {k: v for k, v in vars(spec).items() if v is not None and v is not False}
                                               for p, spec in c.params.items()}}
                         for name, c in self.commands.items()},
            "under_it": {name: {"kind": c.kind, "what": c.summary} for name, c in {**self.children, **self.links}.items()},
            "is_a_camera": self.camera is not None,
        }

    def outline(self) -> dict:
        return {"kind": self.kind, "what": self.summary, "commands": sorted(self.commands),
                **({"picture": True} if self.camera is not None else {}),
                **({"members": sorted("/" + n.path for n in self.links.values())} if self.links else {}),
                **({"under_it": {n: c.outline() for n, c in self.children.items()}} if self.children else {})}
