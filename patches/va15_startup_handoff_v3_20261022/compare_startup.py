#!/usr/bin/env python3
"""Offline paired MuJoCo comparison; no network/robot/camera clients."""
import argparse
import concurrent.futures
import csv as csv_module
import hashlib
import json
import os
import subprocess
from pathlib import Path


def main():
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-name", default="paired_startup")
    parser.add_argument("--offline-release-seconds", type=float)
    args = parser.parse_args()
    if Path(args.output_name).name != args.output_name:
        raise SystemExit("One new output directory name required")
    out = root / "outputs" / args.output_name
    out.mkdir(parents=True, exist_ok=False)
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    paths = list((root / "production").glob("*")) + [root / "simulate_trial.cpp", root / "build/simulate_trial"]
    fingerprints = {str(p.relative_to(root)): digest(p) for p in paths if p.is_file()}
    policy = root.parent / "va15_stop_feedback_20261020/work/b2_z1_no_gun/VA15/policy.pt"
    if digest(policy) != "8dfe4bcad4f0f2edd5d51858ef6e7f5ef9403c2db967cad8f45759e4bdb71098":
        raise RuntimeError("Simulator is not using the audited board VA15 weight")
    cases = [(mu, tau, legacy) for mu, tau in ((.8, 1.), (.5, 1.), (1., 1.), (.8, .9), (.8, 1.1))
             for legacy in (True, False)]

    def run(case):
        mu, tau, legacy = case
        name = f'{"legacy" if legacy else "candidate"}_mu{mu:.1f}_tau{tau:.1f}'
        csv = out / (name + ".csv")
        cmd = [str(root / "build/simulate_trial"), str(root / "production/b2_independent_model.json"),
               str(csv), "deadline", str(mu), str(tau)]
        if legacy:
            cmd.append("--legacy-startup")
        with (out / (name + ".log")).open("w") as log:
            env=os.environ.copy()
            if args.offline_release_seconds is not None:
                env["VA15_OFFLINE_RELEASE_SECONDS"]=str(args.offline_release_seconds)
            result = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, timeout=120,env=env)
        report = json.loads(Path(str(csv) + ".json").read_text())
        with csv.open() as data:
            rows = [r for r in csv_module.DictReader(data) if 6. <= float(r["time"]) < 7.5]
        heights = [float(r["truth_h"]) for r in rows]
        report["startup_peak_h_from_csv_m"] = max(heights)
        report["startup_vertical_range_from_csv_m"] = max(heights) - min(heights)
        return {"name": name, "friction": mu, "observed_tau_scale": tau, "legacy": legacy,
                "exit": result.returncode, "report": report}

    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(run, cases))
    if any(digest(root / p) != sha for p, sha in fingerprints.items()):
        raise RuntimeError("Sources/executable changed during paired run")
    pairs = []
    for old, new in zip(results[::2], results[1::2]):
        a, b = old["report"], new["report"]
        pairs.append({
            "friction": old["friction"], "observed_tau_scale": old["observed_tau_scale"],
            "legacy_pass": a["pass"], "candidate_pass": b["pass"],
            "requested_pd_jump_Nm_legacy_candidate": [a["max_same_state_requested_pd_jump_until_first_actor_Nm"],
                                                     b["max_same_state_requested_pd_jump_until_first_actor_Nm"]],
            "startup_min_height_m_legacy_candidate": [a["startup_min_h_m"], b["startup_min_h_m"]],
            "startup_peak_height_m_legacy_candidate": [a["startup_peak_h_from_csv_m"], b["startup_peak_h_from_csv_m"]],
            "startup_vertical_range_m_legacy_candidate": [a["startup_vertical_range_from_csv_m"], b["startup_vertical_range_from_csv_m"]],
            "startup_max_tilt_rad_legacy_candidate": [a["startup_max_tilt_rad"], b["startup_max_tilt_rad"]],
            "forward_at_request_end_m_legacy_candidate": [a["forward_at_request_end_m"], b["forward_at_request_end_m"]],
            "lateral_at_request_end_m_legacy_candidate": [a["lateral_at_request_end_m"], b["lateral_at_request_end_m"]],
        })
    summary = {"hardware_qualified": False, "production_replaced": False, "policy_sha256": digest(policy),
               "offline_release_seconds": args.offline_release_seconds,
               "note": "tau_scale perturbs estimator tau input, not motor strength; MuJoCo Z1 servo is a proxy, not SDK validation",
               "all_candidate_contracts_pass": all(r["exit"] == 0 and r["report"]["pass"] for r in results if not r["legacy"]),
               "pairs": pairs, "results": results, "sources_sha256": fingerprints}
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({k: v for k, v in summary.items() if k not in ("results", "sources_sha256")}, indent=2))
    raise SystemExit(0 if summary["all_candidate_contracts_pass"] else 2)


if __name__ == "__main__":
    main()
