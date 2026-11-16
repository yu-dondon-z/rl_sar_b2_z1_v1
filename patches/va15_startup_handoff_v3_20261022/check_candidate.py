#!/usr/bin/env python3
"""Hash audit + actor-only native check; NEVER initializes robot transports."""
import hashlib
import json
from pathlib import Path
import subprocess


def main():
    root = Path(__file__).resolve().parent
    repo = root.parent.parent
    build = root / "native_build_v3_final"
    bundle = json.loads((build / "bundle.json").read_text())
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    for path, expected in bundle["files"].items():
        if digest(Path(path)) != expected:
            raise RuntimeError(f"Candidate build/input changed: {path}")
    unchanged = {
        "policy/b2_z1_no_gun/base.yaml": "909d04f2ce4820effd7a284ccff5955dc6a9c8fba921f0fe42a439627ced7164",
        "policy/b2_z1_no_gun/VA15/config.yaml": "8faa51628b3704ce3e7e3a384f1c0eb8bb59b7dcf5a0069992069efed8aab8d1",
        "policy/b2_z1_no_gun/VA15/policy.pt": "8dfe4bcad4f0f2edd5d51858ef6e7f5ef9403c2db967cad8f45759e4bdb71098",
        "diagnostics/va15_motor_contract_v2_20261021/run_trial.sh": "4a49fa3fb68759e46d9ea1483093273de3944376259dc7b9797f319262bef9ce",
        "diagnostics/va15_motor_contract_v2_20261021/native_build_v2/va15_forward_trial": "4058a6e11f3deecd4ad705a64842428dd42b4daa120f88fd331301b47ef019b6",
    }
    for name, expected in unchanged.items():
        if digest(repo / name) != expected:
            raise RuntimeError(f"Audited policy/config/V2 baseline changed: {name}")
    subprocess.run([str(build / "va15_forward_trial"), "--offline-check"], cwd=repo, check=True)
    print("PASS: isolated V3 source/executable bundle and unchanged V2 + VA15 weight/config")
    print("No DDS/UDP/camera/robot initialized. NOT permission or acceptance for real walking.")


if __name__ == "__main__":
    main()
