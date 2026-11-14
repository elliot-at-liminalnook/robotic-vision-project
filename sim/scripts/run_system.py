#!/usr/bin/env python3
"""The whole thing in one process: a simulated world, the hub on its ports, and both REST APIs.

    python sim/scripts/run_system.py                       # scene open_small
    python sim/scripts/run_system.py --scene shelf
    curl http://127.0.0.1:8765/                            # the hub's API: give this guide to the model
    curl http://127.0.0.1:8766/                            # the operator's API: scenes, truth, scoring (never for the model)

The hub (hub/) is what would run on the real machine. Here it is started on the world's ports
(servo buses, cameras, a clock) and given nothing else: not the scene, not where anything is.
Loading another scene over the operator API is, to the hub, everything going quiet and coming
back somewhere else; POST /discover on the hub's API looks again for what is plugged in.

Both servers answer on one thread, one request at a time, on 127.0.0.1 only unless --host says
otherwise, with no authentication.
"""
from __future__ import annotations

import argparse
import signal
import sys
from http.server import HTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import operator_api  # noqa: E402
from so101sim.world import World  # noqa: E402
from hub import api as hub_api  # noqa: E402
from hub.hub import Hub  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", default="open_small", help="the scene loaded at the start (a file in sim/scenes/)")
    ap.add_argument("--host", default="127.0.0.1", help="address to listen on (default: this machine only)")
    ap.add_argument("--hub-port", type=int, default=8765, help="the hub's API, for the model")
    ap.add_argument("--operator-port", type=int, default=8766, help="the operator's API: scenes and ground truth")
    ap.add_argument("--quiet", action="store_true", help="don't log each request")
    a = ap.parse_args()
    try:
        world = World(a.scene)
    except (FileNotFoundError, ValueError) as e:
        ap.error(str(e))
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))          # a kill leaves through the same finally
    hub = Hub(world.ports)
    hub_server = HTTPServer((a.host, a.hub_port), hub_api.make_handler(hub, verbose=not a.quiet))
    op_server = HTTPServer((a.host, a.operator_port), operator_api.make_handler(operator_api.Operator(world), verbose=not a.quiet))
    hub_server.timeout = op_server.timeout = 0.02
    print(f"hub API       http://{a.host}:{hub_server.server_address[1]}/   found: {', '.join(hub.found()['devices']) or 'nothing'}")
    print(f"operator API  http://{a.host}:{op_server.server_address[1]}/   scene '{world.name}'   (Ctrl-C to stop)", flush=True)
    for note in hub.notes:
        print("  note:", note)
    try:
        while True:
            hub_server.handle_request()
            op_server.handle_request()
    except KeyboardInterrupt:
        print()
    finally:
        hub_server.server_close()
        op_server.server_close()
        print("stop path:", hub.shutdown())
    return 0


if __name__ == "__main__":
    sys.exit(main())
