#!/usr/bin/env python3
"""Apply the verified esp_video 2.5.0 repairs to managed_components/.

Run after `idf.py reconfigure` (which downloads managed components) and before
`idf.py build`. Files are replaced only when their current SHA-256 matches the
known 2.5.0 original; anything else is refused. Re-running is harmless.

Origin: ~/Projects/esp32-stereo-camera/work/esp32-p4-imx-camera/tools/
apply_video_fixes.py and video-fixes/ (RAW10 receive fix, Bayer-phase refresh
after sensor format/flip changes, AWB threshold validation).
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parent
FIXES = PROJECT / "patches" / "esp_video-2.5.0"


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--project", type=Path, default=PROJECT)
    args = ap.parse_args()

    base = args.project / "managed_components" / "espressif__esp_video"
    plans = []
    for entry in json.loads((FIXES / "manifest.json").read_text()):
        target = base / entry["file"]
        if not target.exists():
            sys.exit(f"{target} missing; run `idf.py reconfigure` first")
        replacement = (FIXES / entry["replacement"]).read_bytes()
        if sha256(replacement) != entry["after_sha256"]:
            sys.exit(f"corrupt repair source: {entry['replacement']}")
        current = sha256(target.read_bytes())
        if current == entry["after_sha256"]:
            print(f"already repaired: {entry['file']}")
        elif current == entry["before_sha256"]:
            plans.append((target, replacement))
        else:
            sys.exit(f"{target}: unexpected content (not esp_video 2.5.0?); refusing")

    for target, data in plans:
        target.write_bytes(data)
        print(f"repaired: {target.relative_to(args.project)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
