#!/usr/bin/env python3
"""Scale the lateral curvature of an authored MuJoCo crack centerline."""

from __future__ import annotations

import argparse
from pathlib import Path
import re


FROMTO = re.compile(r'fromto="([^"]+)"')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--lateral-scale",
        type=float,
        default=0.40,
        help="multiply every centerline y coordinate by this value",
    )
    parser.add_argument(
        "--lateral-offset",
        type=float,
        default=0.0,
        help="world-frame y offset added after scaling",
    )
    args = parser.parse_args()
    if not 0.0 <= args.lateral_scale <= 1.0:
        parser.error("--lateral-scale must be between 0 and 1")

    source_text = args.source.read_text(encoding="utf-8")
    segment_count = 0
    y_values: list[float] = []

    def replace(match: re.Match[str]) -> str:
        nonlocal segment_count
        values = [float(value) for value in match.group(1).split()]
        if len(values) != 6:
            raise ValueError("each MuJoCo fromto attribute must contain six values")
        values[1] = values[1] * args.lateral_scale + args.lateral_offset
        values[4] = values[4] * args.lateral_scale + args.lateral_offset
        y_values.extend((values[1], values[4]))
        segment_count += 1
        formatted = " ".join(f"{value:.4f}" for value in values)
        return f'fromto="{formatted}"'

    output_text = FROMTO.sub(replace, source_text)
    if segment_count == 0:
        raise RuntimeError(f"no fromto segments found in {args.source}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(output_text, encoding="utf-8")
    temporary.replace(args.output)
    print(
        f"scaled {segment_count} crack segments by {args.lateral_scale:.3f}: "
        f"y=[{min(y_values):.4f}, {max(y_values):.4f}] -> {args.output}"
    )


if __name__ == "__main__":
    main()
