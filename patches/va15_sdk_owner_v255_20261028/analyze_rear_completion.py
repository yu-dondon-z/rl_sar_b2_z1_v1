"""Read saved acceptance CSVs; no runtime, camera, SDK or control imports."""
import argparse
import csv
import json
import math
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    summary = json.loads((args.directory / "summary.json").read_text())
    results = []
    for case in summary["results"]:
        if case["report"].get("scenario") not in ("straight", "curve"):
            continue
        with (args.directory / (case["name"] + ".csv")).open() as source:
            rows = list(csv.DictReader(source))
        approach = [r for r in rows if int(r["end_stage"]) == 3]
        active = [r for r in rows if int(r["tcp_active"])]
        result = dict(name=case["name"], acceptance_pass=case["report"]["pass"],
                      maximum_confirmed_cap_frames=max(int(r["end_frames"]) for r in rows),
                      tcp_tracking_samples=len(active),
                      first_active_s=float(active[0]["time"]) if active else None,
                      max_abs_target_body_y_m=max(abs(float(r["target_y"])) for r in rows),
                      final_reason=rows[-1]["reason"],
                      final_truth_xy_error_m=case["report"]["final_truth_tcp_xy_error_m"],
                      controller_reported_complete=case["report"]["normal_visual_end_stood"])
        if approach:
            best = min(approach, key=lambda r: float(r["xy_error"]))
            end_x=case["report"]["physical_endpoint_x_m"]
            end_y=.035*math.sin(.8*end_x) if case["report"]["scenario"] == "curve" else 0.
            result.update(minimum_estimated_xy_error_m=float(best["xy_error"]),
                          at_minimum_estimated_error_s=float(best["time"]),
                          truth_xy_at_same_time_m=math.hypot(float(best["tcp_truth_x"])-end_x,
                                                           float(best["tcp_truth_y"])-end_y),
                          body_yaw_at_same_time_rad=float(best["base_yaw"]),
                          target_body_at_same_time=[float(best[k]) for k in ("target_x","target_y","target_z")],
                          estimated_base_at_same_time=[float(best[k]) for k in ("estimate_x","estimate_y")])
        results.append(result)
    report=dict(schema="va15_rear_endpoint_diagnosis_v1",results=results,
                cap_frames_are_observation_proof_not_arrival=True,
                note="Legacy cap_frames in harness summary counts frames at brake request; 0 does NOT mean no cap confirmation. CSV end_frames is the observation count.",
                source="Immutable saved synthetic-vision/IMU-joint physics logs; not hardware evidence",
                hardware_qualified=False,release_allowed=False)
    output=args.directory / "diagnosis.json"
    if output.exists():
        raise RuntimeError("Refusing to overwrite prior diagnosis")
    output.write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(report,indent=2))


if __name__ == "__main__":
    main()
