#!/usr/bin/env python3
"""Read-only one-step VA15 actor sensitivity audit for a recorded CSV."""

import argparse
import csv
from pathlib import Path

import numpy as np
import torch


OBSERVATION_DIM = 70
ACTION_DIM = 18
INPUT_GROUPS = {
    "commands": range(0, 3),
    "target_commands": range(3, 10),
    "base_ang_vel": range(10, 13),
    "projected_gravity": range(13, 16),
    "joint_pos_rel": range(16, 34),
    "joint_vel_rel": range(34, 52),
    "previous_raw_action": range(52, 70),
}
# Policy ordering from policy/b2_z1_no_gun/base.yaml: hips 0:4, thighs 4:8,
# joint1 at 8, calves 9:13, then the remaining Z1 joints.
ACTION_GROUPS = {
    "left_thigh": (4, 6),
    "right_thigh": (5, 7),
    "left_calf": (9, 11),
    "right_calf": (10, 12),
}
ACTION_CONTRASTS = {
    "thigh_left_minus_right": ((4, 6), (5, 7)),
    "calf_left_minus_right": ((9, 11), (10, 12)),
}
ANGULAR_VELOCITY_AXES = {10: "gyro_x", 11: "gyro_y", 12: "gyro_z"}


def fit_basis(times, frequency):
    centered = times - times.mean()
    phase = 2.0 * np.pi * frequency * times
    return np.column_stack((np.ones(len(times)), centered, np.sin(phase), np.cos(phase)))


def fit_amplitudes(values, basis):
    coefficients = np.linalg.lstsq(basis, values, rcond=None)[0]
    return np.hypot(coefficients[2], coefficients[3])


def contrast_amplitude(values, left_indices, right_indices, selected, basis):
    left = values[:, left_indices].mean(axis=1)
    right = values[:, right_indices].mean(axis=1)
    return float(fit_amplitudes((left - right)[selected], basis))


def load_rows(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--actor-csv", required=True)
    parser.add_argument("--policy", required=True)
    parser.add_argument("--zero-ns", required=True, type=int, help="Recorded motion-start timestamp in ns")
    parser.add_argument("--start", type=float, default=2.0)
    parser.add_argument("--end", type=float, default=10.0)
    parser.add_argument("--frequency", type=float, default=5.875)
    args = parser.parse_args()

    rows = load_rows(args.actor_csv)
    observations = np.asarray(
        [[float(row[f"observation_{i}"]) for i in range(OBSERVATION_DIM)] for row in rows],
        dtype=np.float32,
    )
    recorded_actions = np.asarray(
        [[float(row[f"raw_action_{i}"]) for i in range(ACTION_DIM)] for row in rows],
        dtype=np.float64,
    )
    times = np.asarray([int(row["actor_begin_ns"]) for row in rows], dtype=np.float64)
    times = (times - args.zero_ns) * 1e-9
    selected = (times >= args.start) & (times <= args.end)
    if int(selected.sum()) < 8:
        raise SystemExit("not enough actor samples in requested time window")

    model = torch.jit.load(args.policy, map_location="cpu").eval()
    with torch.no_grad():
        baseline = model(torch.from_numpy(observations)).numpy().astype(np.float64)
    if baseline.shape != recorded_actions.shape:
        raise SystemExit(f"unexpected actor shape: model={baseline.shape}, log={recorded_actions.shape}")
    replay_error = baseline - recorded_actions

    window_times = times[selected]
    basis = fit_basis(window_times, args.frequency)
    group_amplitudes = {}
    for name, indices in ACTION_GROUPS.items():
        group_signal = baseline[:, indices].mean(axis=1)
        group_amplitudes[name] = float(fit_amplitudes(group_signal[selected], basis))

    print(
        f"rows={len(rows)} window_rows={int(selected.sum())} "
        f"replay_max_error={np.max(np.abs(replay_error)):.6g} "
        f"replay_rmse={np.sqrt(np.mean(replay_error ** 2)):.6g}"
    )
    print("baseline_action_amplitude=" + " ".join(f"{k}:{v:.6f}" for k, v in group_amplitudes.items()))
    baseline_contrasts = {
        name: contrast_amplitude(baseline, left, right, selected, basis)
        for name, (left, right) in ACTION_CONTRASTS.items()
    }
    print("baseline_left_minus_right=" + " ".join(f"{k}:{v:.6f}" for k, v in baseline_contrasts.items()))

    phase = np.column_stack(
        (np.sin(2.0 * np.pi * args.frequency * window_times),
         np.cos(2.0 * np.pi * args.frequency * window_times))
    )
    for name, indices in INPUT_GROUPS.items():
        indices = list(indices)
        counterfactual = observations.astype(np.float64).copy()
        coefficients = np.linalg.lstsq(
            basis, counterfactual[selected][:, indices], rcond=None
        )[0]
        component = phase @ coefficients[2:4]
        counterfactual[np.ix_(selected, indices)] -= component
        with torch.no_grad():
            prediction = model(torch.from_numpy(counterfactual.astype(np.float32))).numpy().astype(np.float64)

        ratios = {}
        for action_name, action_indices in ACTION_GROUPS.items():
            signal = prediction[:, action_indices].mean(axis=1)
            amplitude = float(fit_amplitudes(signal[selected], basis))
            ratios[action_name] = amplitude / group_amplitudes[action_name]
        input_rms = float(np.sqrt(np.mean(component ** 2)))
        output_delta = float(np.sqrt(np.mean((prediction[selected] - baseline[selected]) ** 2)))
        ratio_text = " ".join(f"{key}:{value:.3f}" for key, value in ratios.items())
        print(f"{name:20s} input_sine_rms={input_rms:.6g} action_delta_rms={output_delta:.6g} ratios={ratio_text}")

    for index, name in ANGULAR_VELOCITY_AXES.items():
        counterfactual = observations.astype(np.float64).copy()
        coefficients = np.linalg.lstsq(basis, counterfactual[selected, index], rcond=None)[0]
        component = phase @ coefficients[2:4]
        counterfactual[selected, index] -= component
        with torch.no_grad():
            prediction = model(torch.from_numpy(counterfactual.astype(np.float32))).numpy().astype(np.float64)
        contrasts = {
            key: contrast_amplitude(prediction, left, right, selected, basis)
            for key, (left, right) in ACTION_CONTRASTS.items()
        }
        ratios = " ".join(f"{key}:{contrasts[key] / baseline_contrasts[key]:.3f}" for key in contrasts)
        input_amplitude = float(np.hypot(coefficients[2], coefficients[3]))
        output_delta = float(np.sqrt(np.mean((prediction[selected] - baseline[selected]) ** 2)))
        contrast_values = " ".join(f"{key}:{value:.6f}" for key, value in contrasts.items())
        print(
            f"{name + '_only':20s} input_sine_amplitude={input_amplitude:.6g} "
            f"action_delta_rms={output_delta:.6g} contrasts={contrast_values} ratios={ratios}"
        )


if __name__ == "__main__":
    main()
