#!/usr/bin/env python3
"""Offline B-specific braking comparison. Never invokes a real robot launcher."""
import argparse
import concurrent.futures
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--jobs", type=int, choices=(1, 2), default=2)
    parser.add_argument("--suite", choices=("full", "sweep"), default="full")
    parser.add_argument("--b-decel", type=float, default=.3)
    args = parser.parse_args()
    if not .12 <= args.b_decel <= .6:
        parser.error("B-only simulation deceleration must be 0.12..0.6")
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    binary = ROOT / "build/simulate_independent_stop"
    linkage = subprocess.check_output(["ldd", str(binary)], text=True)
    (out / "runtime_libraries.log").write_text(linkage)
    if any(name in linkage for name in ("libddsc", "libZ1_SDK", "libunitree_sdk")):
        raise RuntimeError("Hardware transport library present in simulation binary")
    cases = [("standby_B", dict(speed=0, walk_seconds=0))]
    for speed in (.1, .2, .3):
        for phase in (0., .15, .3, .45):
            cases.append((f"walk_{speed:.2f}_phase_{phase:.2f}",
                          dict(speed=speed, walk_seconds=8 + phase)))
    for delay in (.05, .25, .75):
        cases.append((f"handoff_B_{delay}", dict(speed=.2, walk_seconds=delay)))
    for friction in (.5, 1.):
        cases.append((f"friction_{friction}", dict(speed=.2, walk_seconds=8.1, friction=friction)))
    for speed in (.2, .3):
        cases.append((f"long_{speed}", dict(speed=speed, walk_seconds=30)))
    cases.append(("observer_200hz", dict(speed=.2, walk_seconds=8, observer_period=.005)))
    for scale in (.75, 1.25):
        for speed in (.2, .3):
            cases.append((f"torque_scale_{scale}_speed_{speed}",
                          dict(speed=speed, walk_seconds=8.15, tau_scale=scale)))
    for fault in ("stale", "tau_nan", "imu_sign", "partial_tau_zero", "stale_after_catch"):
        cases.append((f"fault_{fault}", dict(speed=.2, walk_seconds=8.15, fault=fault)))
    if args.suite == "sweep":
        cases = []
        for decel in (.12, .2, .3, .4, .5, .6):
            for speed in (.2, .3):
                for phase in (0., .15, .3, .45):
                    cases.append((f"brake_{decel:.2f}_walk_{speed:.2f}_phase_{phase:.2f}",
                                  dict(speed=speed, walk_seconds=8+phase, b_decel=decel)))
    else:
        cases = [(name, dict(options, b_decel=args.b_decel)) for name, options in cases]
    env = dict(os.environ, OMP_NUM_THREADS="1", MKL_NUM_THREADS="1")
    env["LD_LIBRARY_PATH"] = "/home/dong/rl_sar_b2_z1_old/library/inference_runtime/libtorch/lib"

    def run(case):
        name, options = case
        prefix = out / (name + ".csv")
        command = [str(binary), "--model", str(ROOT.parent / "va15_independent_state_20261020/outputs/b2_independent_model.json"),
                   "--output", str(prefix)]
        for key, value in options.items():
            command.extend(["--" + key.replace("_", "-"), str(value)])
        started = time.monotonic()
        try:
            with (out / (name + ".log")).open("x") as log:
                result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
                                        env=env, timeout=180, check=False)
            report = json.loads(Path(str(prefix) + ".json").read_text())
            rows = list(csv.DictReader(prefix.open()))
            no_sport = all(row["source_sport_fresh"] == "0" for row in rows)
            report.update(name=name, returncode=result.returncode,
                          wall_seconds=time.monotonic() - started, csv_sport_always_false=no_sport)
            report["case_pass"] = bool(report["offline_closed_loop_pass"] and result.returncode == 0
                and no_sport and not report["control_authorized"] and not report["hardware_qualified"]
                and not report["truth_drives_stop_decisions"])
        except (subprocess.TimeoutExpired, FileNotFoundError, json.JSONDecodeError) as error:
            report = dict(name=name, case_pass=False, reason=str(error))
        print(f"{name}: pass={report['case_pass']}, lower={report.get('lower_after_B')}, "
              f"reason={report.get('reason')}", flush=True)
        return report

    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        reports = list(pool.map(run, cases))
    files = [ROOT / name for name in ("simulation_feedback.hpp", "simulate_independent_stop.cpp", "run_suite.py", "CMakeLists.txt", "b_stop_brake.hpp", "test_b_stop_brake.cpp")]
    files += [ROOT.parent / "va15_independent_state_20261020/va15_independent_state.hpp",
              ROOT.parent / "va15_independent_state_20261020/outputs/b2_independent_model.json",
              ROOT.parent / "va15_controlled_getdown_20261020/simulate_controlled_getdown.cpp",
              ROOT.parent / "va15_controlled_getdown_20261020/va15_controlled_stop.hpp",
              ROOT.parent / "va15_controlled_getdown_20261020/fsm_b2_z1_no_gun.hpp",
              ROOT.parent / "va15_stop_feedback_20261020/work/b2_z1_no_gun/VA15/policy.pt",
              ROOT.parent / "va15_stop_feedback_20261020/work/b2_z1_no_gun/VA15/config.yaml"]
    summary = dict(hardware_qualified=False, control_authorized=False,
                   observer_in_stop_control_loop=True, truth_drives_stop_decisions=False,
                   suite=args.suite, b_decel=args.b_decel if args.suite=="full" else None,
                   total=len(reports), passed=sum(report["case_pass"] for report in reports),
                   hashes={str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in files},
                   results=reports)
    (out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    print(f"SUMMARY: {summary['passed']}/{summary['total']} OFFLINE estimated-stop cases; no hardware authority", flush=True)
    return 0 if summary["passed"] == summary["total"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
