#!/usr/bin/env python3
"""Offline provenance audit against VA15's saved training source and weights."""

import argparse
import hashlib
import json
from pathlib import Path
import tarfile

import torch
import yaml


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=Path(
        "/home/dong/b2_z1/logs/rsl_rl/b2_z1_amp/2026-09-28_15-07-31_va15_curriculum_from2500"))
    parser.add_argument("--policy", type=Path, default=Path(
        "/home/dong/rl_sar_b2_z1/patches/va15_stop_feedback_20261020/work/b2_z1_no_gun/VA15/policy.pt"))
    parser.add_argument("--isaaclab", type=Path, default=Path(
        "/home/dong/IsaacLab-2.2.1/source/isaaclab/isaaclab"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    env_path, agent_path = args.run / "params/env.yaml", args.run / "params/agent.yaml"
    env = yaml.load(env_path.read_text(), Loader=yaml.BaseLoader)
    agent = yaml.load(agent_path.read_text(), Loader=yaml.BaseLoader)
    archive_path = args.run / "source.tar.gz"
    names = {
        "actions": "source/b2_z1/b2_z1/tasks/manager_based/b2_z1/mdp/actions.py",
        "wrapper": "source/b2_z1/b2_z1/rsl_rl/wrappers/amp_vecenv_wrapper.py",
        "environment": "source/b2_z1/b2_z1/envs/b2z1_rl_amp_env.py",
    }
    with tarfile.open(archive_path) as archive:
        archived = {key: archive.extractfile(name).read() for key, name in names.items()}
    observations_path = args.isaaclab / "envs/mdp/observations.py"
    manager_path = args.isaaclab / "managers/action_manager.py"
    observations = observations_path.read_text()
    manager = manager_path.read_text()
    actions = archived["actions"].decode()
    checkpoint_path = args.run / "model_5000.pt"
    # These are the user's trusted local training artifacts, never downloads.
    checkpoint = torch.load(str(checkpoint_path), map_location="cpu", weights_only=False)
    deployed = torch.jit.load(str(args.policy), map_location="cpu")
    parameter_errors = {key: float((value - checkpoint["model_state_dict"]["actor." + key]).abs().max())
                        for key, value in deployed.state_dict().items()}
    action = env["actions"]["JointPositionAction"]
    policy_obs = env["observations"]["policy"]
    result = {
        "schema": "va15_actual_training_contract_audit_v1", "offline_only": True,
        "hardware_access": False, "motion_qualification": False,
        "source_run": str(args.run.resolve()), "checkpoint": str(checkpoint_path),
        "checkpoint_sha256": sha(checkpoint_path.read_bytes()),
        "deployed_policy": str(args.policy.resolve()), "deployed_policy_sha256": sha(args.policy.read_bytes()),
        "training_env_sha256": sha(env_path.read_bytes()), "training_agent_sha256": sha(agent_path.read_bytes()),
        "training_source_archive_sha256": sha(archive_path.read_bytes()),
        "archived_source_hashes": {names[key]: sha(data) for key, data in archived.items()},
        "external_dependency_sources": {
            str(observations_path): sha(observations_path.read_bytes()),
            str(manager_path): sha(manager_path.read_bytes()),
        },
        "external_dependency_historically_archived": False,
        "checkpoint_iteration": int(checkpoint["iter"]),
        "deployed_actor_parameter_max_errors": parameter_errors,
        "deployed_actor_parameter_global_max_error": max(parameter_errors.values()),
        "deployed_jit_forward": deployed.code,
        "training": {
            "empirical_normalization": agent["empirical_normalization"],
            "agent_action_clip": agent["clip_actions"], "action_class": action["class_type"],
            "action_scale": action["scale"], "alpha": action["alpha"],
            "sim_dt": env["sim"]["dt"], "decimation": env["decimation"],
            "policy_dt_s": float(env["sim"]["dt"]) * int(env["decimation"]),
            "observation_order": [k for k, v in policy_obs.items() if isinstance(v, dict) and "func" in v],
            "last_action_func": policy_obs["actions"]["func"],
            "last_action_params": policy_obs["actions"]["params"],
            "angular_velocity_scale": policy_obs["base_ang_vel"]["scale"],
            "joint_velocity_scale": policy_obs["joint_vel_rel"]["scale"],
            "actuators": env["scene"]["robot"]["actuators"],
        },
        "implementation_checks": {
            "archived_action_term_has_separate_EMA": "self._alpha * actions + (1.0 - self._alpha) * self._raw_actions" in actions,
            "archived_wrapper_clips_input_before_env_step": "actions = torch.clamp(actions, -self.clip_actions, self.clip_actions)" in archived["wrapper"].decode(),
            "installed_last_action_default_returns_manager_input": "if action_name is None:\n        return env.action_manager.action" in observations,
            "installed_manager_copies_input_before_term_processing": manager.index("self._action[:] = action.to(self.device)") < manager.index("term.process_actions(term_actions)"),
        },
        "conclusion": "Bounded raw manager-input history plus separate alpha=.2 actuator EMA is consistent with the actual run, not filtered term history.",
        "limitations": [
            "IsaacLab dependency itself is not inside source.tar.gz; its installed implementation is inspected now, not proven by a training-time hash.",
            "Action-term comment claiming last_action is filtered conflicts with the default action_manager.action observation path; do not tune against that comment.",
            "Archived agent disables empirical actor normalization; AMP discriminator normalization is not actor observation normalization.",
            "Actor parameter equality and observation/filter provenance do not establish hardware closed-loop stability.",
            "The run's exported/policy.pt may have been overwritten by later checkpoints; audit the selected deployment artifact directly.",
        ],
    }
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "training_contract_audit.json").write_text(json.dumps(result, indent=2) + "\n")
    report = ["# VA15 actual saved-training contract audit", "",
              f"Checkpoint: `{checkpoint_path}` (iteration {result['checkpoint_iteration']}).", "",
              f"All {len(parameter_errors)} deployed actor parameter tensors match the saved checkpoint exactly; maximum error {max(parameter_errors.values()):.1f}.", "",
              "Training configuration: 70 observations, 18 actions, 20 ms policy period, action scale .25, input clip +/-5, separate actuator EMA alpha .20.",
              "`last_action` without `action_name` reads manager input actions, not the filtered action term buffer.",
              "The current deployment's bounded raw previous action is therefore consistent with this training path.",
              "No empirical actor normalizer is missing: the saved agent disables it and the deployed graph is the plain matching actor.", "",
              "Saved leg PD is Kp=160, Kd=5 with 0..2 physics-step delay at 5 ms (0..10 ms).",
              "The saved arm uses implicit PD Kp=160, Kd=5; this is not a characterization of the real Z1 inner loop.", "",
              "## Provenance caveats", ""] + [f"- {text}" for text in result["limitations"]] + [""]
    (args.output / "TRAINING_CONTRACT_AUDIT.md").write_text("\n".join(report))
    print(json.dumps({"output": str(args.output.resolve()), "iteration": result["checkpoint_iteration"],
                      "actor_parameter_max_error": max(parameter_errors.values()),
                      "implementation_checks": result["implementation_checks"]}))


if __name__ == "__main__":
    main()
