"""Offline rear-TCP acceptance, distinct from legacy front-camera stop tests.

Control uses synthetic calibrated image packets and IMU/joint local odometry.
Physical truth is used ONLY to generate images and independently score arrival.
No SDK/DDS/UDP is initialized. A failure is retained, never relabelled success.
"""
import concurrent.futures
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def main():
    root = Path(__file__).resolve().parent
    name = sys.argv[1] if len(sys.argv) == 2 else "rear_endpoint_final"
    if Path(name).name != name:
        raise SystemExit("One new output directory name required")
    out = root / "outputs" / name
    out.mkdir(parents=True, exist_ok=False)
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    tracked = [*(root / "production").glob("*"), root / "simulate_rear_completion.cpp", Path(__file__).resolve()]
    before = {str(p.relative_to(root)): sha(p) for p in tracked if p.is_file()}
    executable = root / "build/simulate_rear_completion"
    executable_sha = sha(executable)
    cases = [(s,mu,5.,0.) for s in ("straight","curve") for mu in (.5,.8,1.)]
    cases += [("straight",.7,5.,0.)]
    cases += [("straight",.8,length,0.) for length in (2.,3.,8.)]
    cases += [(s,.8,5.,3.) for s in ("straight","curve")]
    cases += [(s,.8,5.,0.) for s in ["no_end","unknown_loss","camera_failure","one_false_cap","operator_b","terminal_b","terminal_pose_loss"]]

    def run(case):
        scenario, mu, length, noise = case
        stem = f"{scenario}_mu{mu}_end{length}_noise{noise}"
        csv = out / (stem + ".csv")
        with (out / (stem + ".log")).open("w") as log:
            try:
                result = subprocess.run([str(executable), scenario, str(csv), str(mu), str(length), str(noise)],
                                        stdout=log, stderr=subprocess.STDOUT, timeout=240)
                code = result.returncode
            except subprocess.TimeoutExpired:
                code = 124
        file = Path(str(csv) + ".json")
        report = json.loads(file.read_text()) if file.exists() else {"pass": False, "reason": "missing report or timeout"}
        row = dict(name=stem, exit=code, report=report)
        print(json.dumps(row), flush=True)
        return row

    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(run, cases))
    if any(sha(root / p) != h for p, h in before.items()) or sha(executable) != executable_sha:
        raise RuntimeError("Source/executable changed during validation; results not accepted")
    report = dict(passed=sum(r["exit"] == 0 and r["report"]["pass"] for r in results),
                  total=len(results), results=results, production_sha256=before,
                  executable_sha256=executable_sha, synthetic_rgb=True,
                  rear_tcp_arrival_required=True, independent_estimated_pose=True,
                  truth_used_for_control=False, hardware_qualified=False,
                  physical_endpoint_xy_tolerance_at_brake_m=.10,
                  physical_endpoint_xy_tolerance_after_stop_m=.12)
    (out / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"{report['passed']}/{report['total']} offline checks; not a hardware release", flush=True)
    return 0 if report["passed"] == report["total"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
