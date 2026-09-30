#!/usr/bin/env python3
"""Generate a collision road with a 2 cm wide, 5 cm deep curved crack."""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path


FROMTO = re.compile(r'fromto="([^"]+)"')


def load_centerline(path: Path) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for match in FROMTO.finditer(path.read_text(encoding="utf-8")):
        values = [float(value) for value in match.group(1).split()]
        start = (values[0], values[1])
        end = (values[3], values[4])
        if not points:
            points.append(start)
        if math.hypot(points[-1][0] - end[0], points[-1][1] - end[1]) > 1e-8:
            points.append(end)
    if len(points) < 2:
        raise RuntimeError(f"No centerline segments found in {path}")
    return points


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--crack-width", type=float, default=0.02)
    parser.add_argument("--crack-depth", type=float, default=0.05)
    parser.add_argument("--road-half-width", type=float, default=10.0)
    args = parser.parse_args()

    points = load_centerline(args.source)
    gap_half = 0.5 * args.crack_width
    slab_half_width = 0.5 * (args.road_half_width - gap_half)
    slab_offset = gap_half + slab_half_width
    slab_half_depth = 0.5 * args.crack_depth
    lines = [
        "<mujocoinclude>",
        "  <!-- Generated physical road: 2 cm opening, 5 cm bottom depth. -->",
    ]
    for index, (start, end) in enumerate(zip(points, points[1:])):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx)
        nx, ny = -dy / length, dx / length
        mx, my = 0.5 * (start[0] + end[0]), 0.5 * (start[1] + end[1])
        qw, qz = math.cos(0.5 * yaw), math.sin(0.5 * yaw)
        for side, sign in (("left", 1.0), ("right", -1.0)):
            px, py = mx + sign * nx * slab_offset, my + sign * ny * slab_offset
            lines.append(
                f'  <geom name="road_{side}_{index:03d}" type="box" '
                f'pos="{px:.6f} {py:.6f} {-slab_half_depth:.6f}" '
                f'size="{0.5 * length:.6f} {slab_half_width:.6f} {slab_half_depth:.6f}" '
                f'quat="{qw:.8f} 0 0 {qz:.8f}" material="groundplane" '
                'friction="1.0 0.005 0.0001"/>'
            )
        # A visual-only dark ribbon at the bottom makes the true trench clear
        # to both the camera and the user without filling its collision gap.
        lines.append(
            f'  <geom name="crack_bottom_ribbon_{index:03d}" type="capsule" '
            f'size="{gap_half:.6f}" '
            f'fromto="{start[0]:.6f} {start[1]:.6f} {-args.crack_depth + 0.001:.6f} '
            f'{end[0]:.6f} {end[1]:.6f} {-args.crack_depth + 0.001:.6f}" '
            'material="crack_bottom" contype="0" conaffinity="0" group="2"/>'
        )
    lines.append("</mujocoinclude>")
    args.output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(
        f"generated {len(points) - 1} segments: width={args.crack_width:.3f} m, "
        f"depth={args.crack_depth:.3f} m -> {args.output}"
    )


if __name__ == "__main__":
    main()
