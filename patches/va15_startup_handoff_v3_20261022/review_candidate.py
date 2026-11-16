#!/usr/bin/env python3
"""Separate a passing software regression from a dynamic rollout decision."""
import json
from pathlib import Path

root = Path(__file__).resolve().parent
paired = json.loads((root / "outputs/paired_final/summary.json").read_text())
results = []
for p in paired["pairs"]:
    before, after = p["lateral_at_request_end_m_legacy_candidate"]
    results.append({"friction": p["friction"], "tau_input_scale": p["observed_tau_scale"],
                    "legacy_lateral_m": before, "candidate_lateral_m": after,
                    "direction_nonregression": abs(after) <= abs(before) + .015})
review = {"hardware_qualified": False, "motion_approved": False,
          "startup_pd_continuity_improved": True,
          "software_scenario_contracts_pass": paired["all_candidate_contracts_pass"],
          "direction_nonregression_all_pairs": all(r["direction_nonregression"] for r in results),
          "decision": "KEEP V2; V3 candidate offline only. Do not replace walking entrypoint.",
          "note": "1.5 cm added lateral drift is an offline screening margin, not a relaxed contact/stop gate or hardware safety threshold",
          "pairs": results}
target = root / "outputs/review_final.json"
if target.exists():
    raise SystemExit("Review already exists; refusing overwrite")
target.write_text(json.dumps(review, indent=2) + "\n")
print(json.dumps(review, indent=2))
