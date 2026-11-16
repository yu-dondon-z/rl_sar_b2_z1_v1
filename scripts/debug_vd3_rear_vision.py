#!/usr/bin/env python3
"""Run the current VD3 visual MuJoCo deployment and summarize rear vision.

The run keeps the normal paired front-RGB/rear-RGBD bridge, guarded controller,
and no-privileged-information policy path.  After the run this wrapper reads
the retained packet history and writes a compact rear_debug_summary.json next
to the session logs.  It never connects to hardware.
"""
from __future__ import annotations

import argparse
import collections
import gzip
import json
from pathlib import Path
import sys

from play_vd1_mujoco import main as launch


ROOT = Path(__file__).resolve().parents[1]
PYTHON = "/home/dong/anaconda3/envs/crack_vision/bin/python"


def steady_evaluation(session: Path) -> dict:
    """Read output-only truth after shutdown; never supply controller inputs."""
    import numpy as np

    with gzip.open(session / "evaluation_truth.jsonl.gz", "rt", encoding="utf-8") as source:
        rows = [json.loads(line) for line in source]
    result = dict(evaluation_only=True, excluded_startup_s=5.0,
                  height_width_definition="P95 minus P5; world vertical position")
    if len(rows) < 2:
        return dict(result, available=False, reason="not enough physics samples")
    times = np.asarray([row["sim_time_s"] for row in rows])
    keep = times >= 5.0
    if np.count_nonzero(keep) < 2 or np.any(np.diff(times) <= 0):
        return dict(result, available=False, reason="need motion samples after 5 simulation seconds")
    base = np.asarray([row["base_position_w"] for row in rows])
    tcp = np.asarray([row["tcp_position_w"] for row in rows])[keep]
    yaw = np.asarray([row["base_yaw_rad"] for row in rows])
    velocity = np.gradient(base, times, axis=0)
    forward = (np.cos(yaw) * velocity[:, 0] + np.sin(yaw) * velocity[:, 1])[keep]
    error = np.asarray([row["tcp_projection_error_m"] for row in rows])[keep]
    feet = np.asarray([row["foot_position_w"] for row in rows])[keep]
    force = np.asarray([row["foot_net_force_w"] for row in rows])[keep]

    def width_mm(values):
        return float(1000 * (np.percentile(values, 95) - np.percentile(values, 5)))

    result.update(
        available=True, sample_count=int(np.count_nonzero(keep)),
        time_range_s=[float(times[keep][0]), float(times[keep][-1])],
        actual_forward_horizontal_mean_mps=float(forward.mean()),
        base_height_width_mm=width_mm(base[keep, 2]), tcp_height_width_mm=width_mm(tcp[:, 2]),
        tcp_ground_projection_error_p95_mm=float(1000 * np.percentile(error, 95)),
        tcp_ground_projection_error_max_mm=float(1000 * error.max()),
        tcp_ground_projection_over_50mm_fraction=float(np.mean(error > .05)),
        foot_height_width_mm={leg: width_mm(feet[:, i, 2])
                              for i, leg in enumerate(("FL", "FR", "RL", "RR"))},
        foot_vertical_force_mean_n={leg: float(force[:, i, 2].mean())
                                    for i, leg in enumerate(("FL", "FR", "RL", "RR"))},
        note="Foot height widths describe this gait; they are not a symmetry acceptance test.")
    return result


def summarize(session: Path) -> dict:
    summary = json.loads((session / "summary.json").read_text(encoding="utf-8"))
    metadata = json.loads((session / "session.json").read_text(encoding="utf-8"))
    reasons = collections.Counter()
    packet_count = valid_count = rear_valid_count = 0
    gaps = []
    ages = []
    covered = 0
    history = session / "dual_vision_packets.jsonl.gz"
    with gzip.open(history, "rt", encoding="utf-8") as source:
        for line in source:
            packet = json.loads(line)
            packet_count += 1
            ages.append(float(packet.get("capture_age_s", float("nan"))))
            if packet.get("valid"):
                valid_count += 1
            else:
                reasons[packet.get("reason", "unknown")] += 1
            rear = packet.get("rear", {})
            if not rear.get("valid"):
                continue
            rear_valid_count += 1
            points = rear.get("points_b", [])
            if len(points) > 1:
                import numpy as np
                values = np.asarray(points, dtype=float)
                gaps.append(float(np.linalg.norm(np.diff(values, axis=0), axis=1).max()))
                covered += int(values[:, 0].min() <= -1.02 <= values[:, 0].max())
    finite_ages = [value for value in ages if value == value]
    result = {
        "session": str(session),
        "policy": metadata["policy"],
        "policy_sha256": metadata["policy_sha256"],
        "runtime_summary": summary,
        "steady_evaluation": steady_evaluation(session),
        "packet_count": packet_count,
        "paired_valid_packets": valid_count,
        "rear_valid_packets": rear_valid_count,
        "invalid_reason_counts": dict(reasons),
        "rear_max_local_gap_m": max(gaps) if gaps else None,
        "rear_target_x_covered_packets": covered,
        "capture_age_s": {
            "mean": sum(finite_ages) / len(finite_ages) if finite_ages else None,
            "max": max(finite_ages) if finite_ages else None,
        },
        "retained_artifacts": [
            "vision.log", "play.log", "dual_vision_packets.jsonl.gz",
            "latest_dual_overlay.png", "front/", "rear/",
            "actor_trace.jsonl.gz", "evaluation_truth.jsonl.gz", "session.json",
        ],
    }
    (session / "rear_debug_summary.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-steps", type=int, default=3000,
                        help="actor steps; 3000 is 60 s; 0 runs until viewer closes/Ctrl-C")
    parser.add_argument("--speed", type=float, default=0.20)
    parser.add_argument("--curvature-stage", type=int, default=4,
                        help="scene curve stage; 4 selects maximum curvature 0.40/m")
    parser.add_argument("--output-root", type=Path,
                        default=ROOT / "outputs/VD3_REAR_VISION_DEBUG")
    parser.add_argument("--show", action="store_true",
                        help="show the MuJoCo and perception windows")
    parser.add_argument("--no-display", action="store_true",
                        help="hide only the perception window; --show still shows MuJoCo")
    parser.add_argument("--dry-run", action="store_true",
                        help="record policy and commands without starting any subprocess")
    args = parser.parse_args(argv)
    if args.max_steps < 0:
        parser.error("--max-steps must be nonnegative")
    args.output_root.mkdir(parents=True, exist_ok=True)
    before = {path.resolve() for path in args.output_root.glob("session_*")}
    launch_args = [
        "--robot-version", "vd3", "--visual-controller", "guarded",
        "--speed", str(args.speed), "--curvature-stage", str(args.curvature_stage),
        "--road-surface", "tape", "--physics-dt", "0.005",
        "--velocity-constraint", "impulse", "--physx-implicit-position",
        "--max-steps", str(args.max_steps),
        "--mujoco-python", PYTHON, "--vision-python", PYTHON,
        "--output-root", str(args.output_root),
        "--cache-dir", "/dev/shm/vd3_rear_vision_debug_cache",
    ]
    if not args.show:
        launch_args.extend(["--headless", "--no-display"])
    elif args.no_display:
        launch_args.append("--no-display")
    if args.dry_run:
        launch_args.append("--dry-run")
    returncode = launch(launch_args)
    if args.dry_run:
        return returncode
    candidates = sorted(
        (path for path in args.output_root.glob("session_*")
         if path.resolve() not in before and (path / "summary.json").is_file()),
        key=lambda path: path.stat().st_mtime,
    )
    if not candidates:
        if returncode:
            return returncode
        raise RuntimeError("No completed MuJoCo session found under " + str(args.output_root))
    result = summarize(candidates[-1])
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return returncode


if __name__ == "__main__":
    raise SystemExit(main())
