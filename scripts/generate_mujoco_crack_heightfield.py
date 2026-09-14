#!/usr/bin/env python3
"""Generate one continuous MuJoCo road with a single crack.

Besides loading an authored centreline, this utility can generate the
repeatable left-turn test route used by ``scene_vision.xml``: a straight
approach, a 90-degree circular arc, and a long straight exit.
"""

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
    parser.add_argument("--source", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--x-min", type=float, default=-5.0)
    parser.add_argument("--x-max", type=float, default=40.0)
    parser.add_argument("--y-min", type=float, default=-2.0)
    parser.add_argument("--y-max", type=float, default=2.0)
    parser.add_argument("--x-resolution", type=float, default=0.10)
    parser.add_argument("--y-resolution", type=float, default=0.02)
    parser.add_argument("--crack-width", type=float, default=0.02)
    parser.add_argument("--surface-gray", type=int, default=180)
    parser.add_argument(
        "--straight-y",
        type=float,
        default=None,
        help="Generate a straight world-X crack at this Y coordinate instead of using --source geometry.",
    )
    parser.add_argument(
        "--left-turn",
        action="store_true",
        help="Generate a straight, 90-degree left arc, and straight-exit test route.",
    )
    parser.add_argument(
        "--cubic-sine-amplitude",
        type=float,
        default=None,
        help="generate y=A*sin(2*pi*x/period)^3 instead of using --source",
    )
    parser.add_argument("--cubic-sine-period", type=float, default=20.0)
    parser.add_argument("--turn-start-x", type=float, default=6.0)
    parser.add_argument("--turn-radius", type=float, default=2.0)
    parser.add_argument("--turn-exit-y", type=float, default=24.0)
    parser.add_argument("--route-spacing", type=float, default=0.05)
    args = parser.parse_args()

    generated_modes = sum((
        bool(args.left_turn),
        args.straight_y is not None,
        args.cubic_sine_amplitude is not None,
    ))
    if generated_modes > 1:
        raise ValueError(
            "--left-turn, --straight-y and --cubic-sine-amplitude are mutually exclusive"
        )
    if not 0 <= args.surface_gray <= 255:
        raise ValueError("--surface-gray must be in [0, 255]")

    if args.left_turn:
        if args.turn_radius <= 0.0 or args.route_spacing <= 0.0:
            raise ValueError("--turn-radius and --route-spacing must be positive")
        turn_end_x = args.turn_start_x + args.turn_radius
        if not args.x_min <= args.turn_start_x < turn_end_x <= args.x_max:
            raise ValueError("The left-turn arc must lie inside [--x-min, --x-max]")
        if not args.y_min <= 0.0 < args.turn_radius < args.turn_exit_y <= args.y_max:
            raise ValueError("The left-turn route must lie inside [--y-min, --y-max]")

        # Positive world Y is left.  A circle centred at (start_x, radius)
        # goes from angle -pi/2 to 0: its incoming tangent is +X and its
        # outgoing tangent is +Y.
        arc_length = 0.5 * math.pi * args.turn_radius
        arc_segments = max(2, math.ceil(arc_length / args.route_spacing))
        points = [(args.x_min, 0.0), (args.turn_start_x, 0.0)]
        points.extend(
            (
                args.turn_start_x + args.turn_radius * math.cos(theta),
                args.turn_radius + args.turn_radius * math.sin(theta),
            )
            for theta in (
                -0.5 * math.pi + 0.5 * math.pi * index / arc_segments
                for index in range(1, arc_segments + 1)
            )
        )
        points.append((turn_end_x, args.turn_exit_y))
        route_description = (
            f"left90: straight to x={args.turn_start_x:.2f} m, "
            f"radius={args.turn_radius:.2f} m, exit to y={args.turn_exit_y:.2f} m"
        )
    elif args.cubic_sine_amplitude is not None:
        if args.cubic_sine_period <= 0.0 or args.route_spacing <= 0.0:
            raise ValueError("cubic-sine period and route spacing must be positive")
        segments = max(2, math.ceil((args.x_max-args.x_min)/args.route_spacing))
        points = [
            (
                args.x_min+(args.x_max-args.x_min)*index/segments,
                args.cubic_sine_amplitude*math.sin(
                    2.0*math.pi*(args.x_min+(args.x_max-args.x_min)*index/segments)
                    / args.cubic_sine_period
                )**3,
            )
            for index in range(segments+1)
        ]
        if any(not args.y_min <= y <= args.y_max for _, y in points):
            raise ValueError("cubic-sine route lies outside [--y-min, --y-max]")
        route_description = (
            f"cubic-sine A={args.cubic_sine_amplitude:.3f} m, "
            f"period={args.cubic_sine_period:.3f} m"
        )
    elif args.straight_y is None:
        if args.source is None:
            raise ValueError("--source is required unless --straight-y or --left-turn is used")
        centerline = load_centerline(args.source)
        # Continue straight beyond the authored curved section so the height
        # field has no artificial platform edge in the camera's working range.
        points = [
            (args.x_min, centerline[0][1]),
            *centerline,
            (args.x_max, centerline[-1][1]),
        ]
        route_description = f"source={args.source}"
    else:
        if not args.y_min <= args.straight_y <= args.y_max:
            raise ValueError("--straight-y must lie inside [--y-min, --y-max]")
        points = [(args.x_min, args.straight_y), (args.x_max, args.straight_y)]
        route_description = f"straight y={args.straight_y:.3f} m"
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
    image = Image.new("L", (width, height), color=args.surface_gray)
    draw = ImageDraw.Draw(image)
    line_pixels = max(1, round(args.crack_width / args.y_resolution))
    draw.line([pixel(point) for point in points], fill=0, width=line_pixels)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    image.save(args.output)
    print(
        f"generated continuous {width}x{height} heightfield with one "
        f"{args.crack_width:.3f} m crack ({route_description}) -> {args.output}"
    )


if __name__ == "__main__":
    main()
