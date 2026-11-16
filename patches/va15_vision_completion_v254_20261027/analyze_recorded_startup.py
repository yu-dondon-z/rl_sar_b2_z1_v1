#!/usr/bin/env python3
"""Read-only analysis of recorded VA15 command/state CSVs, never robot I/O.

The fitted response is an empirical CLOSED-LOOP proxy, not an identified motor
plant. Commands are policy outputs correlated with state, feet carry loads,
and CSV records are decimated snapshots. Delay/bandwidth are descriptive only.
"""

import argparse
import csv
import hashlib
import io
import json
from pathlib import Path

import numpy as np
from scipy import signal


JOINTS = [f"{leg}_{joint}" for leg in ("FR", "FL", "RR", "RL")
          for joint in ("hip", "thigh", "calf")]
POLICY_LEGS = [0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12]


def load_csv(path):
    raw = path.read_bytes()
    reader = csv.DictReader(io.StringIO(raw.replace(b"\0", b"").decode()))
    good, rejected = [], 0
    for row in reader:
        if None in row or None in row.values():
            rejected += 1
        else:
            good.append(row)
    return good, {"sha256": hashlib.sha256(raw).hexdigest(), "complete_rows": len(good),
                  "rejected_incomplete_rows": rejected, "nul_bytes": raw.count(b"\0")}


def values(rows, key):
    return np.asarray([float(row[key]) for row in rows], dtype=float)


def hold_sample(time, value, desired):
    """Causal command zero-order hold. Do not interpolate future commands."""
    return value[np.clip(np.searchsorted(time, desired, side="right") - 1, 0, len(time) - 1)]


def quantiles(x):
    return {"p50": float(np.percentile(x, 50)), "p95": float(np.percentile(x, 95)),
            "max": float(np.max(x))}


def response_proxy(t, y, command_t, command, dt):
    """Fit y[k]=a*y[k-1]+b*u(t[k-1]-d)+c; held-out replay is open-loop.

    Selection uses the first 60% only. A stable first-order candidate is chosen
    by training one-step RMSE. Validation and persistence metrics disclose weak
    fits. It is NOT an intervention or proof of actuator delay/causality.
    """
    split = max(12, int(len(t) * .60))
    best = None
    for delay in np.arange(0., .15001, .002):
        delayed = hold_sample(command_t, command, t[:-1] - delay)
        matrix = np.column_stack((y[:-1], delayed, np.ones(len(y) - 1)))
        coefficient, *_ = np.linalg.lstsq(matrix[:split - 1], y[1:split], rcond=None)
        a, b, offset = coefficient
        if not (0. < a < 1. and b > 0.):
            continue
        train_error = y[1:split] - matrix[:split - 1] @ coefficient
        loss = float(np.mean(train_error ** 2))
        if best is None or loss < best[0]:
            best = (loss, float(delay), coefficient, matrix)
    if best is None:
        return {"valid_stable_fit": False, "reason": "no positive stable first-order fit"}
    train_loss, delay, coefficient, matrix = best
    a, b, offset = coefficient
    prediction = matrix[split - 1:] @ coefficient
    validation = y[split:]
    error = validation - prediction
    persistence_error = validation - y[split - 1:-1]
    replay = [float(y[split - 1])]
    for u in matrix[split - 1:, 1]:
        replay.append(float(a * replay[-1] + b * u + offset))
    replay_error = validation - np.asarray(replay[1:])
    tau = -dt / np.log(a)
    improvement = float(1. - np.mean(error ** 2) / max(np.mean(persistence_error ** 2), 1.e-20))
    replay_rmse = float(np.sqrt(np.mean(replay_error ** 2)))
    replay_screen_pass = improvement > .05 and replay_rmse < .5 * np.std(validation)
    return {
        "valid_stable_fit": True, "delay_proxy_ms": delay * 1000.,
        "a": float(a), "b": float(b), "intercept": float(offset),
        "dc_gain_proxy": float(b / (1. - a)), "time_constant_proxy_ms": float(tau * 1000.),
        "bandwidth_proxy_hz": float(1. / (2. * np.pi * tau)),
        "training_one_step_rmse_rad": float(np.sqrt(train_loss)),
        "validation_one_step_rmse_rad": float(np.sqrt(np.mean(error ** 2))),
        "validation_persistence_rmse_rad": float(np.sqrt(np.mean(persistence_error ** 2))),
        "validation_open_loop_replay_rmse_rad": replay_rmse,
        "validation_signal_std_rad": float(np.std(validation)),
        "validation_improvement_over_persistence": improvement,
        "descriptive_replay_screen_pass": bool(replay_screen_pass),
        "replay_screen_is_hardware_qualification": False,
        "delay_grid_upper_boundary": bool(delay >= .148),
    }


def spectrum(time, command, actual):
    dt = float(np.median(np.diff(time)))
    fs = 1. / dt
    nperseg = min(128, len(time))
    f, suu = signal.welch(command, fs, nperseg=nperseg, detrend="linear")
    _, syy = signal.welch(actual, fs, nperseg=nperseg, detrend="linear")
    _, suy = signal.csd(command, actual, fs, nperseg=nperseg, detrend="linear")
    coherence = np.abs(suy) ** 2 / np.maximum(suu * syy, 1.e-30)
    allowed = (f >= 1.) & (f <= min(12., .9 * fs / 2.))
    index = int(np.argmax(np.where(allowed, suu, -1.)))
    transfer = suy[index] / max(suu[index], 1.e-30)
    phase = float(np.angle(transfer))
    return {
        "dominant_command_frequency_hz": float(f[index]), "frequency_bin_hz": fs / nperseg,
        "command_peak_to_peak_rad": float(np.ptp(command)),
        "actual_peak_to_peak_rad": float(np.ptp(actual)),
        "actual_over_command_rms": float(np.std(actual) / max(np.std(command), 1.e-20)),
        "response_magnitude_at_command_peak": float(np.abs(transfer)),
        "coherence_at_command_peak": float(coherence[index]),
        "phase_at_command_peak_rad": phase,
        "phase_delay_proxy_ms": float(-phase / (2. * np.pi * f[index]) * 1000.),
        "phase_delay_is_aliased_modulo_period": True,
    }


def finite_json(value):
    if isinstance(value, dict):
        return {k: finite_json(v) for k, v in value.items()}
    if isinstance(value, list):
        return [finite_json(v) for v in value]
    if isinstance(value, (float, np.floating)) and not np.isfinite(value):
        return None
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    control, control_source = load_csv(args.recording / "control.csv")
    actor, actor_source = load_csv(args.recording / "actor.csv")
    commands, command_source = load_csv(args.recording / "commands.csv")
    started = [r for r in control if r["visual_stage"] == "1"]
    if not started:
        raise SystemExit("No running visual interval in recording")
    t0 = float(started[0]["control_start_ns"])
    first_stop = next(r for r in control
                      if float(r["control_start_ns"]) > t0 and r["visual_stage"] == "2")
    duration = (float(first_stop["control_start_ns"]) - t0) * 1.e-9
    selected = [r for r in commands
                if t0 - 3.e9 < float(r["publish_begin_ns"]) < t0 + duration * 1.e9]
    command_t = (values(selected, "publish_begin_ns") - t0) * 1.e-9
    # Actual motor states precede command publication. Preserve that direction
    # instead of assuming the current command already caused current state.
    source_t = (values(selected, "low_callback_received_ns") - t0) * 1.e-9
    unique = np.r_[True, np.diff(source_t) > 0.]
    source_t = source_t[unique]
    dt = float(np.median(np.diff(command_t)))
    start, end = 2., min(10., duration - .05)
    time = np.arange(start, end, dt)
    result = {
        "schema": "va15_recorded_startup_analysis_v1", "offline_only": True,
        "hardware_access": False, "motion_qualification": False,
        "recording": str(args.recording.resolve()), "sources": {
            "control.csv": control_source, "commands.csv": command_source, "actor.csv": actor_source},
        "running_start_ns": int(t0), "running_until_first_stop_s": duration,
        "identification_window_s": [start, end], "uniform_sample_period_s": dt,
        "limitations": [
            "Closed-loop observational fit; actuator, policy, contacts and body dynamics are confounded.",
            "Decimated CSV snapshots do not identify the 200 Hz motor command stream.",
            "Transport callback receive time is not a hardware acquisition timestamp.",
            "First-order delay/gain/bandwidth are unqualified descriptive proxies, not motor specifications.",
            "Frequency phase delay is ambiguous modulo a gait period and may be non-causal in feedback.",
            "No production actor, observations, raw history, gains, thresholds or weights are modified.",
        ],
    }
    joints = []
    for j, name in enumerate(JOINTS):
        commanded = values(selected, f"sent_q_dds_{j}")
        actual = values(selected, f"actual_q_dds_{j}")[unique]
        u = hold_sample(command_t, commanded, time)
        y = np.interp(time, source_t, actual)
        joints.append({"joint": name, **spectrum(time, u, y),
                       "first_order_proxy": response_proxy(time, y, command_t, commanded, dt)})
        # Requested PD uses the state available to each outgoing command. The
        # estimate is delayed telemetry, not a calibrated torque transducer.
        measured_q = values(selected, f"actual_q_dds_{j}")
        measured_v = values(selected, f"actual_dq_dds_{j}")
        pd = (values(selected, f"sent_kp_dds_{j}") * (commanded - measured_q) +
              values(selected, f"sent_kd_dds_{j}") *
              (values(selected, f"sent_dq_dds_{j}") - measured_v) +
              values(selected, f"sent_ff_tau_dds_{j}"))
        requested_torque = hold_sample(command_t, pd, time)
        measured_torque = np.interp(time, source_t, values(selected, f"actual_tau_est_dds_{j}")[unique])
        torque_spectrum = spectrum(time, requested_torque, measured_torque)
        joints[-1]["torque_estimate_spectral_response"] = {
            "requested_PD_dominant_frequency_hz": torque_spectrum["dominant_command_frequency_hz"],
            "requested_PD_peak_to_peak_nm": torque_spectrum["command_peak_to_peak_rad"],
            "measured_tau_est_peak_to_peak_nm": torque_spectrum["actual_peak_to_peak_rad"],
            "response_magnitude_at_PD_peak": torque_spectrum["response_magnitude_at_command_peak"],
            "coherence_at_PD_peak": torque_spectrum["coherence_at_command_peak"],
            "phase_delay_proxy_ms": torque_spectrum["phase_delay_proxy_ms"],
            "phase_delay_is_aliased_modulo_period": True,
        }
    result["joint_response"] = joints
    ac = [r for r in actor if t0 <= float(r["actor_begin_ns"]) < t0 + duration * 1.e9]
    action_errors = [max(abs(float(r[f"observation_{52+j}"]) - np.clip(
        float(previous[f"raw_action_{j}"]), -5., 5.)) for j in range(18))
                     for previous, r in zip(ac, ac[1:])]
    result["actor_contract"] = {
        "executed_frames": sum(r["model_executed"] == "1" for r in ac), "recorded_frames": len(ac),
        "begin_interval_ms": quantiles(np.diff(values(ac, "actor_begin_ns")) * 1.e-6),
        "bounded_previous_raw_observation_max_error": max(action_errors, default=0.),
        "tcp_active_rows": sum(r["tcp_active"] == "1" for r in started),
        "tcp_target_peak_to_peak_m": [float(np.ptp(values(started, f"tcp_target_b_{j}"))) for j in range(3)],
        "policy_velocity_peak": [float(np.max(np.abs(values(started, f"policy_command_{j}")))) for j in range(3)],
    }
    startup = [r for r in selected if -.15 <= (float(r["publish_begin_ns"]) - t0) * 1.e-9 <= .30]
    torque = []
    for r in startup:
        item = {"time_s": (float(r["publish_begin_ns"]) - t0) * 1.e-9,
                "torque_handoff_active": int(r["torque_handoff_active"])}
        for j, name in enumerate(JOINTS):
            q, v = float(r[f"actual_q_dds_{j}"]), float(r[f"actual_dq_dds_{j}"])
            kp, kd = float(r[f"sent_kp_dds_{j}"]), float(r[f"sent_kd_dds_{j}"])
            target, desired_v = float(r[f"sent_q_dds_{j}"]), float(r[f"sent_dq_dds_{j}"])
            effort = kp * (target - q) + kd * (desired_v - v) + float(r[f"sent_ff_tau_dds_{j}"])
            item[f"{name}_requested_pd_nm"] = effort
            item[f"{name}_measured_tau_est_nm"] = float(r[f"actual_tau_est_dds_{j}"])
            item[f"{name}_kp"] = kp
        torque.append(item)
    changes = []
    for previous, current in zip(torque, torque[1:]):
        if current["FR_calf_kp"] != previous["FR_calf_kp"]:
            changes.append({"time_s": current["time_s"],
                            "FR_calf_kp_before": previous["FR_calf_kp"],
                            "FR_calf_kp_after": current["FR_calf_kp"],
                            "requested_pd_before_nm": previous["FR_calf_requested_pd_nm"],
                            "requested_pd_after_nm": current["FR_calf_requested_pd_nm"]})
    result["startup_gain_change_snapshots"] = changes
    result["FR_calf_requested_pd_max_first_300ms_nm"] = max(
        row["FR_calf_requested_pd_nm"] for row in torque if row["time_s"] >= 0.)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "analysis.json").write_text(json.dumps(finite_json(result), indent=2) + "\n")
    with (args.output / "joint_response.csv").open("w", newline="") as stream:
        flattened = [{**{k: v for k, v in j.items()
                         if k not in ("first_order_proxy", "torque_estimate_spectral_response")},
                      **{f"fit_{k}": v for k, v in j["first_order_proxy"].items()},
                      **{f"torque_{k}": v for k, v in j["torque_estimate_spectral_response"].items()}}
                     for j in joints]
        keys = list(dict.fromkeys(key for row in flattened for key in row))
        writer = csv.DictWriter(stream, keys); writer.writeheader(); writer.writerows(flattened)
    with (args.output / "startup_torque.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, list(torque[0])); writer.writeheader(); writer.writerows(torque)
    table = ["# Recorded V251 startup response — offline descriptive analysis", "",
             "This is closed-loop recorded-data analysis, not a hardware plant identification or motion acceptance.",
             "No robot connections, motor commands, production changes or parameter tuning are performed.", "",
             f"Running interval: {duration:.3f} s. Dynamic fit window: {start:.2f}–{end:.2f} s.",
             f"CSV snapshots: approximately {1./dt:.1f} Hz; do not confuse this with motor writer rate.", "",
             "| DDS joint | Dominant command Hz | Position response magnitude at peak | Coherence | Validation improvement vs persistence | First-order replay screen |",
             "|---|---:|---:|---:|---:|---|"]
    for j in joints:
        fit = j["first_order_proxy"]
        if fit.get("valid_stable_fit"):
            fit_text = f"{fit['validation_improvement_over_persistence']:.2f} | {'PASS (descriptive only)' if fit['descriptive_replay_screen_pass'] else 'FAIL'}"
        else:
            fit_text = "unidentified | FAIL"
        table.append(f"| {j['joint']} | {j['dominant_command_frequency_hz']:.2f} | "
                     f"{j['response_magnitude_at_command_peak']:.3f} | {j['coherence_at_command_peak']:.2f} | {fit_text} |")
    usable = sum(j["first_order_proxy"].get("descriptive_replay_screen_pass", False) for j in joints)
    result["first_order_replay_screen_pass_count"] = usable
    # Re-write after adding aggregate status; do not silently promote stable
    # one-step coefficients to measured hardware delay or bandwidth.
    (args.output / "analysis.json").write_text(json.dumps(finite_json(result), indent=2) + "\n")
    table += ["", f"First-order open-loop replay screen: {usable}/12. "
              "Delay and bandwidth coefficients are retained in JSON for reproducibility; failed fits must not be used for tuning.",
              "", "| DDS joint | PD request dominant Hz | Estimated torque response magnitude | Torque coherence |",
              "|---|---:|---:|---:|"]
    for j in joints:
        estimate = j["torque_estimate_spectral_response"]
        table.append(f"| {j['joint']} | {estimate['requested_PD_dominant_frequency_hz']:.2f} | "
                     f"{estimate['response_magnitude_at_PD_peak']:.2f} | {estimate['coherence_at_PD_peak']:.2f} |")
    table += ["", "## Interpretation boundaries", ""] + [f"- {s}" for s in result["limitations"]]
    table += ["", "The handoff can preserve its first torque sample while later learned excursions still raise requested torque.",
              "Stable raw-action history and a fixed TCP target rule out those particular discontinuities in this recording;",
              "they do not prove the policy/real actuator closed loop is stable. Negative validation improvement means",
              "the fitted proxy is worse than simply retaining the previous measured joint position.", ""]
    (args.output / "REPORT.md").write_text("\n".join(table))
    print(json.dumps({"output": str(args.output.resolve()), "running_s": duration,
                      "tcp_active_rows": result["actor_contract"]["tcp_active_rows"],
                      "history_max_error": result["actor_contract"]["bounded_previous_raw_observation_max_error"],
                      "first_300ms_FR_calf_requested_PD_max_nm": result["FR_calf_requested_pd_max_first_300ms_nm"]}))


if __name__ == "__main__":
    main()
