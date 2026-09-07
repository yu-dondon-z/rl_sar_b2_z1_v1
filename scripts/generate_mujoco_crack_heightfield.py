#!/usr/bin/env python3
"""Generate one continuous MuJoCo road with a single curved crack."""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

from PIL import Image, ImageDraw


FROMTO = re.compile(r'fromto="([^"]+)"')


def load_centerline(path: Path) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for match in FROMTO.finditer(path.read_text(encoding="utf-8")):
        values = [float(value) for value in match.group(1).split()]
        start = (values[0], values[1])
        end = (values[3], values[4])
        if not points:
            points.append(start)
        if math.dist(points[-1], end) > 1e-9:
            points.append(end)
    if len(points) < 2:
        raise RuntimeError(f"No crack centreline found in {path}")
    return points


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--x-min", type=float, default=-5.0)
    parser.add_argument("--x-max", type=float, default=40.0)
    parser.add_argument("--y-min", type=float, default=-2.0)
    parser.add_argument("--y-max", type=float, default=2.0)
    parser.add_argument("--x-resolution", type=float, default=0.10)
    parser.add_argument("--y-resolution", type=float, default=0.02)
    parser.add_argument("--crack-width", type=float, default=0.02)
    args = parser.parse_args()

    centerline = load_centerline(args.source)
    # Continue straight beyond the authored curved section so the height field
    # has no artificial platform edge in the front camera's working range.
    points = [
        (args.x_min, centerline[0][1]),
        *centerline,
        (args.x_max, centerline[-1][1]),
    ]
    width = round((args.x_max - args.x_min) / args.x_resolution) + 1
    height = round((args.y_max - args.y_min) / args.y_resolution) + 1

    def pixel(point: tuple[float, float]) -> tuple[int, int]:
        x, y = point
        column = round((x - args.x_min) / (args.x_max - args.x_min) * (width - 1))
        # Image rows run top-to-bottom; positive world y is the upper side.
        row = round((args.y_max - y) / (args.y_max - args.y_min) * (height - 1))
        return column, row

    # MuJoCo normalizes hfield image values: 180 becomes the common road top
    # and 0 the groove bottom. The same image is also used as the material, so
    # the physical groove remains visible to the RGB segmentation network.
    image = Image.new("L", (width, height), color=180)
    draw = ImageDraw.Draw(image)
    line_pixels = max(1, round(args.crack_width / args.y_resolution))
    draw.line([pixel(point) for point in points], fill=0, width=line_pixels)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    image.save(args.output)
    print(
        f"generated continuous {width}x{height} heightfield with one "
        f"{args.crack_width:.3f} m crack -> {args.output}"
    )


if __name__ == "__main__":
    main()
