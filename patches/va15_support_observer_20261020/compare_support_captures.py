#!/usr/bin/env python3
"""Compare static probe captures. NEVER generate control calibration/permission."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from va15_support_candidate import ORDER, load_candidate
from probe_b2_support import assess


def compare(prone_files, standing_file, candidate):
    def read(path):
        with Path(path).open(newline="") as handle: rows = list(csv.DictReader(handle))
        for r in rows:
            r["sequence_invalid"] = str(r.get("sequence_invalid", "False")).lower() in ("true", "1")
        return rows
    prone = [row for path in prone_files for row in read(path)]
    standing = read(standing_file)
    results = {"prone":assess(prone,candidate), "standing":assess(standing,candidate)}
    separation = {}
    for name in ORDER:
        p = results["prone"]["force_equivalent_z_dds"][name]
        s = results["standing"]["force_equivalent_z_dds"][name]
        raw_p = results["prone"]["raw_bridge_dds"][name]
        raw_s = results["standing"]["raw_bridge_dds"][name]
        separation[name] = dict(
            equivalent_force_p05_stand_minus_p95_prone=(s["p05"]-p["p95"]) if p and s else None,
            raw_central_ranges_overlap=bool(raw_p and raw_s and
                max(raw_p["p05"],raw_s["p05"]) <= min(raw_p["p95"],raw_s["p95"])))
    # Separate a useful diagnostic result from qualification. A central-range
    # gap is not a false-positive/negative rate or a held-out contact test.
    diagnostics_separable = all(v["equivalent_force_p05_stand_minus_p95_prone"] is not None and
        v["equivalent_force_p05_stand_minus_p95_prone"] > 30. for v in separation.values())
    fz = [candidate.evaluate(row) for row in standing]
    totals = [sum(v[2] for v in r["force_equivalent_world"]) for r in fz if r["model_valid"]]
    reference_weight = candidate.model.get("urdf_total_mass", 0.) * 9.81
    return dict(read_only=True, control_authorized=False, motion_qualified=False,
        calibration_generated=False, static_diagnostic_separation=diagnostics_separable,
        separation_dds=separation, captures=results,
        standing_total_force_equivalent_median=float(np.median(totals)) if totals else None,
        urdf_reference_weight=reference_weight,
        total_equivalent_to_model_weight_ratio=float(np.median(totals)/reference_weight)
            if totals and reference_weight > 0. else None,
        source_files={str(path):hashlib.sha256(Path(path).read_bytes()).hexdigest()
                      for path in list(prone_files)+[standing_file]},
        required_before_authority=["Independent contact/unloading ground truth (not pose labels alone)",
            "Torque scale/bias/friction and external leg contact checks", "Dynamic inertial compensation or qualified stop-only envelope",
            "Foot slip/rolling and height/velocity accuracy validation", "Bounded stop/FSM hardware safety qualification"],
        note="A separated static proxy is promising, but cannot certify foot contacts or authorize walking.")


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prone",type=Path,nargs="+",required=True)
    parser.add_argument("--standing",type=Path,required=True)
    parser.add_argument("--model",type=Path,default=Path(__file__).with_name("b2_support_model.json"))
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists(): parser.error("Output already exists")
    report=compare(args.prone,args.standing,load_candidate(args.model))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open("x") as handle: json.dump(report,handle,indent=2,allow_nan=False)
    print(json.dumps(report,indent=2,allow_nan=False))
