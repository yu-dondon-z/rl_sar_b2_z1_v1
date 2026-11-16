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


def unchanged_gates():
    """Whitelist every edit to the archived stop controller, including comments."""
    expected = (ROOT.parent / "va15_controlled_getdown_20261020/va15_controlled_stop.hpp").read_text()
    substitutions = [
        ("#pragma once\n", '#pragma once\n#ifndef VA15_ESTIMATED_STOP_SIMULATION_ONLY\n#error "Local stop-transition shape experiment is simulation-only"\n#endif\n'),
        ("    double brake_time = 0.;\n", "    double brake_time = 0.;\n    double pose_blend_duration = 2.; // trajectory duration, NOT a support/quiet threshold\n    bool hold_supported_pose = false;\n"),
        ("        if (!std::isfinite(dt) || dt <= 0. || dt > .02 || !ValidPose(state) ||\n", "        if (!std::isfinite(dt) || dt <= 0. || dt > .02 || !ValidPose(state) ||\n            !std::isfinite(pose_blend_duration) || pose_blend_duration<.5 || pose_blend_duration>2. ||\n"),
        ("            if(four)progress=std::min(1.,progress+dt/2.);", "            if(four)progress=std::min(1.,progress+dt/pose_blend_duration);"),
        ("                held.motor_command.q[i]=(1.f-blend)*equivalent+blend*stand_q[i];", """                // Optional local experiment: do not drag planted feet toward
                // an unloaded nominal joint pose. Hold the supported measured
                // geometry with the PRE-walk verified standing effort. Initial
                // total torque is still exactly equivalent; all final support,
                // height, uncertainty, speed and quiet-time gates are unchanged.
                const float settled_target=hold_supported_pose
                    ? capture_q[i]+stand_tau[i]/stand_kp[i] : stand_q[i];
                held.motor_command.q[i]=(1.f-blend)*equivalent+blend*settled_target;"""),
    ]
    for before, after in substitutions:
        if expected.count(before) != 1:
            return False
        expected = expected.replace(before, after)
    return expected == (ROOT / "stop_response_controller.hpp").read_text()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sweep", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--transition-sweep", type=Path)
    parser.add_argument("--capture-sweep", type=Path)
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
        "controller_only_whitelisted_trajectory_edits": unchanged_gates(),
        "candidate_sources_match_recorded": all(
            hashlib.sha256(Path(path).read_bytes()).hexdigest() == digest
            for path, digest in candidate["hashes"].items()),
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
    native_compatibility = {}
    for label, speed in (("020", ".20"), ("030", ".30")):
        old = rows(args.baseline / ("walk_0" + speed + "_phase_0.00.csv"))
        new = rows(ROOT / ("outputs/compatibility_" + label + ".csv"))
        native_compatibility[label] = dict(rows=len(new), exact=identical_shared(old, new))
    checks["new_binary_ordinary_mode_exact"] = all(item["exact"] for item in native_compatibility.values())
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
        simulator_sha256=hashlib.sha256((ROOT / "build/simulate_independent_stop").read_bytes()).hexdigest(),
        ordinary_mode_compatibility=native_compatibility,
        checks=checks, protected_source_checks=unchanged, exact_baseline_replays=baseline_replays,
        pre_B_trajectory_checks=pre_b, sweep=sweep_groups, speed_pairs=speed_pairs,
        candidate_normal_aggregate=aggregate(normal),
        fault_cases=len(faults), fault_rejected_rows=sum(report["fault_rejected_rows"] for report in faults),
        fault_checked_rows=sum(report["fault_checked_rows"] for report in faults),
        false_supported_catches=sum(report["false_supported_catches"] for report in candidate["results"]),
        false_lowering_permissions=sum(report["false_lowering_permissions"] for report in candidate["results"]),
        sources=dict(sweep=str(args.sweep.resolve()), candidate=str(args.candidate.resolve()), baseline=str(args.baseline.resolve())),
    )
    for key, folder in (("transition_sweep", args.transition_sweep), ("capture_sweep", args.capture_sweep)):
        if folder:
            report = json.loads((folder / "summary.json").read_text())
            output[key] = dict(total=report["total"], passed=report["passed"], source=str(folder.resolve()))
            output[key]["durations"] = {
                str(duration): aggregate([item for item in report["results"] if item["pose_blend_duration"] == duration])
                for duration in sorted({item["pose_blend_duration"] for item in report["results"]})
            }
    with args.output.open("x") as stream:
        json.dump(output, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(json.dumps(dict(audit_pass=output["audit_pass"], checks=checks, speed_pairs=speed_pairs,
                         normal=output["candidate_normal_aggregate"]), indent=2))
    return 0 if output["audit_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
