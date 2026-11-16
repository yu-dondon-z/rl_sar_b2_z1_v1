#!/usr/bin/env python3
"""Export the reviewed VD5 Startup checkpoint to a small standalone TorchScript actor."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path

import torch

from vd5_mujoco_policy import (POLICY_DIR, VD5_CHECKPOINT_SHA256, VD5_MANIFEST,
                               VD5_POLICY, VD5_POLICY_SHA256)


CHECKPOINT = Path(
    "/home/dong/b2_z1_VD2/logs/rsl_rl/b2_z1_vd5_startup_amp/"
    "2026-11-10_11-47-57_vd5_startup_80ref_20stand_continue1000/model_4498.pt"
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    actual_checkpoint_sha = sha256(CHECKPOINT)
    if actual_checkpoint_sha != VD5_CHECKPOINT_SHA256:
        raise ValueError("VD5 checkpoint SHA256 mismatch: " + actual_checkpoint_sha)
    checkpoint = torch.load(CHECKPOINT, map_location="cpu", weights_only=False)
    if "obs_norm_state_dict" in checkpoint:
        raise ValueError("VD5 checkpoint unexpectedly requires an observation normalizer")
    state = {key.removeprefix("actor."): value
             for key, value in checkpoint["model_state_dict"].items()
             if key.startswith("actor.")}
    actor = torch.nn.Sequential(
        torch.nn.Linear(70, 512), torch.nn.ELU(),
        torch.nn.Linear(512, 256), torch.nn.ELU(),
        torch.nn.Linear(256, 128), torch.nn.ELU(),
        torch.nn.Linear(128, 18),
    ).eval()
    actor.load_state_dict(state, strict=True)
    example = torch.zeros(1, 70)
    with torch.inference_mode():
        traced = torch.jit.trace(actor, example)
        probes = torch.linspace(-1.0, 1.0, 7 * 70).reshape(7, 70)
        expected = actor(probes)
        actual = traced(probes)
    max_error = float((expected - actual).abs().max())
    if not torch.isfinite(actual).all() or max_error > 1e-6:
        raise AssertionError("TorchScript export does not match the source actor")

    POLICY_DIR.mkdir(parents=True, exist_ok=True)
    temporary = VD5_POLICY.with_suffix(".pt.tmp")
    traced.save(str(temporary))
    os.replace(temporary, VD5_POLICY)
    policy_sha = sha256(VD5_POLICY)
    if VD5_POLICY_SHA256 != "TO_BE_EXPORTED" and policy_sha != VD5_POLICY_SHA256:
        raise ValueError("Exported VD5 actor differs from pinned MuJoCo actor")
    manifest = {
        "strategy": "VD5 Startup",
        "checkpoint": str(CHECKPOINT),
        "checkpoint_sha256": actual_checkpoint_sha,
        "checkpoint_iteration": int(checkpoint.get("iter", -1)),
        "training_experiment": checkpoint.get("training_experiment_name"),
        "policy_file": VD5_POLICY.name,
        "policy_sha256": policy_sha,
        "actor_architecture": [70, 512, 256, 128, 18],
        "activation": "ELU",
        "observation_normalizer": False,
        "observation_terms": [
            "base_velocity[3]", "tcp_target_in_base_wxyz[7]", "base_angular_velocity*0.2[3]",
            "projected_gravity[3]", "joint_position_relative[18]",
            "joint_velocity*0.05[18]", "previous_raw_action[18]",
        ],
        "action_scale": 0.25,
        "source_torch": torch.__version__,
        "trace_probe_max_abs_error": max_error,
    }
    VD5_MANIFEST.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
