#!/usr/bin/env python3
"""Read saved V256 telemetry. No robot, SDK, camera or network imports.

Decompose virtual TCP error in the SAME base frame:
  FK(actual) - target = [FK(FSM target) - target] + [FK(actual) - FK(FSM target)].
These vectors add; their norms do not. This is model geometry, not tool calibration.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


ARM = (8, 13, 14, 15, 16, 17)  # policy 12 is RR calf, NOT Z1 joint1


def read_csv(path):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"empty telemetry: {path}")
    return rows


def matrix(rows, prefix, count):
    return np.array([[float(row[f"{prefix}{j}"]) for j in range(count)] for row in rows])


def fk(policy_q):
    """Same link offsets and rotations as production/tcp_fk.hpp; batched."""
    count = len(policy_q)
    position = np.tile([0., 0., .09], (count, 1))
    rotation = np.tile(np.diag([-1., -1., 1.]), (count, 1, 1))
    offsets = ([0, 0, .0585], [0, 0, .045], [-.35, 0, 0],
               [.218, 0, .057], [.07, 0, 0], [.0492, 0, 0])
    for index, axis_id, offset in zip(ARM, (2, 1, 1, 1, 2, 0), offsets):
        position += np.einsum("nij,j->ni", rotation, offset)
        angle = policy_q[:, index]
        axis = np.eye(3)[axis_id]
        cross = np.array([[0., -axis[2], axis[1]], [axis[2], 0., -axis[0]],
                          [-axis[1], axis[0], 0.]])
        c, s = np.cos(angle)[:, None, None], np.sin(angle)[:, None, None]
        joint_rotation = c*np.eye(3) + (1.-c)*np.outer(axis, axis) + s*cross
        rotation = rotation @ joint_rotation
    return position + np.einsum("nij,j->ni", rotation, [.105, 0., -.820])


def stats(vector):
    norm = np.linalg.norm(vector, axis=1)
    return dict(mean_vector_m=vector.mean(axis=0).tolist(),
                mean_norm_m=float(norm.mean()), p95_norm_m=float(np.quantile(norm, .95)),
                max_norm_m=float(norm.max()))


def analyze(directory, policy=None):
    rows = read_csv(directory / "control.csv")
    actors = read_csv(directory / "actor.csv")
    ns = np.array([int(r["control_start_ns"]) for r in rows], dtype=np.int64)
    if np.any(np.diff(ns) <= 0):
        raise ValueError("control timestamps are not strictly increasing")
    times = (ns-ns[0])*1.e-9
    widths = np.diff(ns, append=ns[-1])*1.e-9
    actual = matrix(rows, "actual_q_policy_", 18)
    commanded = matrix(rows, "fsm_target_q_policy_", 18)
    target = matrix(rows, "tcp_target_b_", 3)
    measured = matrix(rows, "tcp_measured_b_", 3)
    desired = matrix(rows, "tcp_desired_b_", 3)
    measured_fk, commanded_fk = fk(actual), fk(commanded)
    valid = np.array([int(r["tcp_measured_valid"]) == 1 for r in rows])
    geometry_error = float(np.max(np.linalg.norm((measured_fk-measured)[valid], axis=1)))
    if geometry_error > 1.e-6:
        raise ValueError("FK disagrees with recorded TCP; do not interpret error decomposition")
    forward = np.array([int(r["trial_stage"]) == 3 for r in rows])
    tracking = forward & valid & np.array([int(r["tcp_active"]) == 1 for r in rows])
    stopped_vision = np.array([int(r["visual_stage"]) == 2 for r in rows])
    actor_ns = np.array([int(r["actor_end_ns"]) for r in actors], dtype=np.int64)
    if np.any(np.diff(actor_ns) <= 0):
        raise ValueError("actor timestamps are not strictly increasing")
    # Diagnostic time alignment only: most recent completed actor at control
    # START. An inference concurrent with that cycle may be consumed later.
    actor_index = np.searchsorted(actor_ns, ns, side="right")-1
    actor_q = matrix(actors, "target_q_policy_", 18)
    observations = matrix(actors, "observation_", 70)
    raw_actions = matrix(actors, "raw_action_", 18)
    report = dict(schema="va15_recorded_tcp_chain_v1", source=str(directory.resolve()),
                  rows=len(rows), actor_rows=len(actors), arm_policy_indices=list(ARM),
                  fk_vs_recorded_max_error_m=geometry_error,
                  visual_stopped_but_forward_s=float(widths[forward & stopped_vision].sum()),
                  terminal_ever_confirmed=any(int(r["terminal_confirmed"]) for r in rows),
                  policy_forward_speed_max_m_s=max(float(r["policy_command_0"]) for r in actors),
                  requested_forward_speed_max_m_s=max(float(r["requested_0"]) for r in rows),
                  target_observation_range_m=np.ptp(observations[:, 3:6], axis=0).tolist(),
                  source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in (directory/"control.csv", directory/"actor.csv")},
                  hardware_qualified=False, precision_qualified=False,
                  interpretation="Virtual TCP model only; FSM target is not a transmitted Z1 packet or motor acknowledgement",
                  phases={})
    masks = dict(before_rear=forward & valid & ~tracking, rear_tracking=tracking,
                 late_tracking=tracking & (times >= 60.))
    for name, mask in masks.items():
        if not mask.any():
            continue
        error = actual[mask][:, ARM]-commanded[mask][:, ARM]
        phase = dict(rows=int(mask.sum()), first_s=float(times[mask][0]), last_s=float(times[mask][-1]),
                     measured_minus_goal=stats(measured[mask]-target[mask]),
                     fsm_fk_minus_goal=stats(commanded_fk[mask]-target[mask]),
                     measured_minus_fsm_fk=stats(measured[mask]-commanded_fk[mask]),
                     target_range_m=np.ptp(target[mask], axis=0).tolist(),
                     target_filter_error=stats(target[mask]-desired[mask]),
                     joint_actual_minus_fsm_mean_rad=error.mean(axis=0).tolist(),
                     joint_actual_minus_fsm_mae_rad=np.abs(error).mean(axis=0).tolist())
        aligned = mask & (actor_index >= 0)
        if aligned.any():
            delta = commanded[aligned][:, ARM]-actor_q[actor_index[aligned]][:, ARM]
            phase["fsm_minus_latest_completed_actor_mean_rad"] = delta.mean(axis=0).tolist()
            phase["actor_fk_minus_goal"] = stats(fk(actor_q[actor_index[aligned]])-target[aligned])
        report["phases"][name] = phase
    if policy:
        import torch
        torch.set_num_threads(1)
        model = torch.jit.load(str(policy), map_location="cpu").eval()
        with torch.no_grad():
            replay = np.concatenate([model(torch.tensor(batch, dtype=torch.float32)).numpy()
                                     for batch in np.array_split(observations, max(1, len(actors)//256))])
        report["actor_replay"] = dict(policy_sha256=hashlib.sha256(policy.read_bytes()).hexdigest(),
                                      max_abs_error=float(np.max(np.abs(replay-raw_actions))),
                                      rmse=float(np.sqrt(np.mean((replay-raw_actions)**2))))
    trace = dict(time=times, tracking=tracking, goal=target, measured=measured,
                 fsm_fk=commanded_fk, error=measured-target)
    return report, trace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("output", type=Path, help="new directory; previous evidence is never overwritten")
    parser.add_argument("--policy", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite: {args.output}")
    report, trace = analyze(args.directory, args.policy)
    args.output.mkdir(parents=True)
    (args.output/"diagnosis.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    with (args.output/"tcp_chain.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time_s", "rear_tracking"] + [f"{p}_{a}" for p in ("goal", "measured", "fsm_fk", "error") for a in "xyz"])
        for i in range(len(trace["time"])):
            writer.writerow([trace["time"][i], int(trace["tracking"][i])] +
                            [x for p in ("goal", "measured", "fsm_fk", "error") for x in trace[p][i]])
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
