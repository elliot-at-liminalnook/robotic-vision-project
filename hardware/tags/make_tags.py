#!/usr/bin/env python3
"""Make the tag artwork: one PNG per tag (what the simulation shows) and one SVG sheet per device
at true size (what gets printed). Both come from the same bits, so the simulated tag is the printed tag.

    python hardware/tags/make_tags.py          # needs opencv (cv2.aruco) and numpy

Writes hardware/tags/png/tag36h11_<id>.png (10 x 10 cells: the 8-cell tag and a 1-cell white border,
40 px per cell) for every id any device uses, and hardware/tags/print/<device>.svg.
Print the SVG at 100 % ("actual size"); each tag has a 50 mm check line beside it to measure.
"""
from __future__ import annotations

import argparse
import tomllib
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
CELL_PX = 40


def layout() -> dict:
    with (HERE / "layout.toml").open("rb") as f:
        return tomllib.load(f)


def bits(tag_id: int, quiet: int) -> np.ndarray:
    """The tag as a grid of cells, True = white, with its quiet zone."""
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    img = cv2.aruco.generateImageMarker(d, tag_id, 8)            # one pixel per cell
    return np.pad(img > 127, quiet, constant_values=True)


def write_png(tag_id: int, quiet: int, path: Path):
    cells = bits(tag_id, quiet)
    img = np.kron(cells.astype(np.uint8) * 255, np.ones((CELL_PX, CELL_PX), np.uint8))
    cv2.imwrite(str(path), img)


def svg_sheet(device: str, tags: list[dict], quiet: int) -> str:
    """Tags laid out in rows on an A4 page, each at its true size, labelled."""
    page_w, page_h, margin, gap = 210.0, 297.0, 15.0, 12.0
    out, x, y, row_h = [], margin, margin, 0.0
    for t in tags:
        side = t["size_m"] * 1000.0
        cell = side / 8.0
        full = cell * (8 + 2 * quiet)
        if x + full > page_w - margin:
            x, y, row_h = margin, y + row_h + gap + 6, 0.0
        cells = bits(t["id"], quiet)
        out.append(f'<rect x="{x:.3f}" y="{y:.3f}" width="{full:.3f}" height="{full:.3f}" fill="white" stroke="#999" stroke-width="0.1"/>')
        for r in range(cells.shape[0]):
            for c in range(cells.shape[1]):
                if not cells[r, c]:
                    out.append(f'<rect x="{x + c * cell:.3f}" y="{y + r * cell:.3f}" width="{cell:.3f}" height="{cell:.3f}" fill="black"/>')
        out.append(f'<text x="{x:.2f}" y="{y + full + 4:.2f}" font-size="3" font-family="sans-serif">'
                   f'{device} id {t["id"]}: {side:.0f} mm black square, on {t["on"]}; this edge up ^</text>')
        x, row_h = x + full + gap, max(row_h, full)
    yb = page_h - margin
    out.append(f'<line x1="{margin}" y1="{yb}" x2="{margin + 50}" y2="{yb}" stroke="black" stroke-width="0.3"/>')
    out.append(f'<text x="{margin + 53}" y="{yb + 1}" font-size="3" font-family="sans-serif">this line is 50 mm: measure it after printing</text>')
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{page_w}mm" height="{page_h}mm" viewBox="0 0 {page_w} {page_h}">\n'
            + "\n".join(out) + "\n</svg>\n")


def main() -> int:
    argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter).parse_args()
    cfg = layout()
    quiet = int(cfg["quiet_zone_cells"])
    (HERE / "png").mkdir(exist_ok=True)
    (HERE / "print").mkdir(exist_ok=True)
    for device, d in cfg["devices"].items():
        for t in d["tags"]:
            write_png(t["id"], quiet, HERE / "png" / f"{cfg['family']}_{t['id']}.png")
        (HERE / "print" / f"{device}.svg").write_text(svg_sheet(device, d["tags"], quiet))
        print(f"{device}: {len(d['tags'])} tags (ids {min(t['id'] for t in d['tags'])} to {max(t['id'] for t in d['tags'])})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
