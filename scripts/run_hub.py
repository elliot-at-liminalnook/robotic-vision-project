#!/usr/bin/env python3
"""The hub on the real machine: whatever is plugged in, and its REST API.

    python scripts/run_hub.py                          # every USB serial adapter and camera the system shows
    python scripts/run_hub.py --config machine.toml    # and what cannot be found by looking: a rover's board on the network
    curl http://127.0.0.1:8765/                        # the API's guide: give this to the model

Every way out of this program switches every servo's torque off (the arm goes limp: support it).

NOT YET TRIED ON HARDWARE (hub/real.py). In the simulation the same hub runs on the world's ports:
sim/scripts/run_system.py.
"""
from __future__ import annotations

import argparse
import signal
import sys
from http.server import HTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hub import api as hub_api  # noqa: E402
from hub.hub import Hub  # noqa: E402
from hub.real import provider  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", help="a file naming what cannot be found by looking (hub/real.py says what goes in it)")
    ap.add_argument("--no-usb", action="store_true", help="do not offer every USB serial adapter; only those in the file")
    ap.add_argument("--cameras", type=int, default=4, help="how many of the system's video devices to try (default 4; 0: only those in the file)")
    ap.add_argument("--host", default="127.0.0.1", help="address to listen on (default: this machine only; there is no authentication)")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--quiet", action="store_true", help="don't log each request")
    a = ap.parse_args()
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))          # a kill leaves through the same finally
    ports = provider(a.config, find_usb=not a.no_usb, find_cameras=a.cameras)
    hub = Hub(ports)
    try:
        server = HTTPServer((a.host, a.port), hub_api.make_handler(hub, verbose=not a.quiet))
        print(f"hub API  http://{a.host}:{server.server_address[1]}/   found: {', '.join(hub.found()['devices']) or 'nothing'}   (Ctrl-C to stop)")
        for note in list(ports.notes) + hub.notes:
            print("  note:", note)
        sys.stdout.flush()
        server.serve_forever()
    except KeyboardInterrupt:
        print()
    finally:
        print("stop path:", hub.shutdown())
    return 0


if __name__ == "__main__":
    sys.exit(main())
