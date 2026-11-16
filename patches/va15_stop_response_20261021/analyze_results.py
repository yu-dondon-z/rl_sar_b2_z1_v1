#!/usr/bin/env python3
"""Audit paired offline B-stop results; cannot authorize hardware control."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def identical_shared(a, b):
    return len(a) == len(b) and all(
        all(left[key] == right[key] for key in left.keys() & right.keys())
        for left, right in zip(a, b)
    )


def aggregate(reports):
    return {
        "cases": len(reports),
        "passed": sum(report["case_pass"] for report in reports),
        **{f"max_{key}": max((report[key] for report in reports if report[key] is not None), default=None) for key in (
            "capture_after_B", "lower_after_B", "prone_after_B", "max_travel_after_B",
            "max_tilt_deg", "max_tilt_after_B_deg", "max_xy_after_B", "max_xy_before_lower",
            "truth_quiet_half_second_after_B", "xy_at_capture",
        )},
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sweep", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--baseline", type=Path, default=ROOT.parent / "va15_independent_stop_20261021/outputs/suite_v2")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise RuntimeError("Refusing to overwrite analysis")
    sweep = json.loads((args.sweep / "summary.json").read_text())
    candidate = json.loads((args.candidate / "summary.json").read_text())
    baseline = json.loads((args.baseline / "summary.json").read_text())
    checks = {
        "sweep_48_passed": sweep["total"] == sweep["passed"] == 48,
        "full_30_passed": candidate["total"] == candidate["passed"] == 30,
        "no_hardware_authority": all(not report["hardware_qualified"] and not report["control_authorized"]
                                     for report in (sweep, candidate)),
        "truth_not_control_input": all(not report["truth_drives_stop_decisions"] for report in (sweep, candidate)),
    }
    # Original policy, support/settling thresholds, catch, observer and original
    # simulation controller must still be byte-identical to the earlier suite.
    unchanged = {}
    for path, digest in baseline["hashes"].items():
        if "va15_independent_stop_20261021/" not in path:
            unchanged[path] = hashlib.sha256(Path(path).read_bytes()).hexdigest() == digest
    unchanged["feedback_adapter_copy"] = (ROOT / "simulation_feedback.hpp").read_bytes() == (
        ROOT.parent / "va15_independent_stop_20261021/simulation_feedback.hpp").read_bytes()
    checks["shared_protected_sources_unchanged"] = all(unchanged.values())
    baseline_replays = {}
    for report in sweep["results"]:
        if report["B_decel_limit"] != .12:
            continue
        old_name = report["name"].split("brake_0.12_")[1]
        baseline_replays[old_name] = identical_shared(
            rows(args.baseline / (old_name + ".csv")), rows(args.sweep / (report["name"] + ".csv")))
    checks["eight_baseline_replays_exact"] = len(baseline_replays) == 8 and all(baseline_replays.values())
    pre_b = {}
    for report in candidate["results"]:
        boundary = 3. + report["walk_seconds"]
        name = report["name"] + ".csv"
        old = [row for row in rows(args.baseline / name) if float(row["elapsed"]) < boundary - 1.e-6]
        new = [row for row in rows(args.candidate / name) if float(row["elapsed"]) < boundary - 1.e-6]
        pre_b[report["name"]] = dict(rows=len(new), identical=identical_shared(old, new))
    checks["all_pre_B_shared_cells_exact"] = all(item["identical"] for item in pre_b.values())
    sweep_groups = {}
    for decel in sorted({report["B_decel_limit"] for report in sweep["results"]}):
        sweep_groups[str(decel)] = aggregate([r for r in sweep["results"] if r["B_decel_limit"] == decel])
    speed_pairs = {}
    for speed in (.2, .3):
        speed_pairs[str(speed)] = {
            "baseline": aggregate([r for r in sweep["results"] if r["B_decel_limit"] == .12 and r["speed"] == speed]),
            "candidate": aggregate([r for r in candidate["results"]
                                    if r["name"].startswith("walk_") and r["speed"] == speed]),
        }
    normal = [report for report in candidate["results"] if report["fault"] == "none"]
    faults = [report for report in candidate["results"] if report["fault"] != "none"]
    output = dict(
        audit_pass=all(checks.values()), hardware_qualified=False, control_authorized=False,
        checks=checks, protected_source_checks=unchanged, exact_baseline_replays=baseline_replays,
        pre_B_trajectory_checks=pre_b, sweep=sweep_groups, speed_pairs=speed_pairs,
        candidate_normal_aggregate=aggregate(normal),
        fault_cases=len(faults), fault_rejected_rows=sum(report["fault_rejected_rows"] for report in faults),
        fault_checked_rows=sum(report["fault_checked_rows"] for report in faults),
        false_supported_catches=sum(report["false_supported_catches"] for report in candidate["results"]),
        false_lowering_permissions=sum(report["false_lowering_permissions"] for report in candidate["results"]),
        sources=dict(sweep=str(args.sweep.resolve()), candidate=str(args.candidate.resolve()), baseline=str(args.baseline.resolve())),
    )
    with args.output.open("x") as stream:
        json.dump(output, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(json.dumps(dict(audit_pass=output["audit_pass"], checks=checks, speed_pairs=speed_pairs,
                         normal=output["candidate_normal_aggregate"]), indent=2))
    return 0 if output["audit_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
