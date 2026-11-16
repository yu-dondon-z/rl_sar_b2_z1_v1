#!/usr/bin/env python3
"""Read-only raw motor-field audit. No estimator, publisher or control client."""
import argparse
import csv
import json
import math
from pathlib import Path
import sys
import threading
import time

JOINTS = [f"{leg}_{joint}" for leg in ("FR", "FL", "RR", "RL")
          for joint in ("hip", "thigh", "calf")]


def motor_reasons(row):
    """Same BASIC motor rejection checks as the unchanged observer, not its full validator."""
    reasons = []
    for i in range(12):
        for field in ("q", "dq", "tau"):
            if not math.isfinite(row[f"{field}_{i}"]):
                reasons.append(f"motor[{i}].{field}=nonfinite")
        if abs(row[f"dq_{i}"]) > 30:
            reasons.append(f"motor[{i}].dq_out_of_range")
        if abs(row[f"tau_{i}"]) > 360:
            reasons.append(f"motor[{i}].tau_out_of_range")
        if row[f"mode_{i}"] != 10:
            reasons.append(f"motor[{i}].mode={row[f'mode_{i}']}")
        if row[f"lost_{i}"] != 0:
            reasons.append(f"motor[{i}].lost={row[f'lost_{i}']}")
    return reasons


def summarize(rows, posture):
    if not rows:
        raise ValueError("No raw data; no conclusion")
    motors = []
    for i, name in enumerate(JOINTS):
        values = [row[f"lost_{i}"] for row in rows]
        changes = [(row["elapsed"], a, b) for row, a, b in zip(rows[1:], values, values[1:]) if a != b]
        motors.append(dict(dds_index=i, joint=name, first=values[0], last=values[-1],
                           minimum=min(values), maximum=max(values), nonzero_rows=sum(v != 0 for v in values),
                           changed=len(changes), rises=sum(b > a for _, a, b in changes),
                           decreases_or_wraps=sum(b < a for _, a, b in changes),
                           events=[dict(elapsed=t, previous=a, current=b, raw_difference=b-a) for t, a, b in changes]))
        channels = {}
        for name in ("q", "dq", "tau", "q_raw", "dq_raw", "ddq_raw", "temperature"):
            series = [row.get(f"{name}_{i}", math.nan) for row in rows]
            finite = [value for value in series if math.isfinite(value)]
            channels[name] = dict(finite_rows=len(finite), distinct=len(set(finite)),
                                  minimum=min(finite) if finite else None, maximum=max(finite) if finite else None,
                                  adjacent_value_changes=sum(a!=b for a,b in zip(finite,finite[1:])))
        motors[-1]["observed_channel_variation_not_freshness_proof"] = channels
    gaps = reversals = duplicates = 0
    for old, new in zip(rows, rows[1:]):
        delta = (new["low_tick"] - old["low_tick"]) & 0xffffffff
        if delta == 0:
            duplicates += 1
        elif delta > 0x80000000:
            reversals += 1
        elif delta > 50:
            gaps += 1
    return dict(
        read_only=True, control_authorized=False, hardware_qualified=False,
        operator_posture_label=posture, sampled_rows=len(rows),
        sampled_duration=rows[-1]["elapsed"]-rows[0]["elapsed"],
        last_source_age=rows[-1]["low_age"], source_age_over_100ms_rows=sum(r["low_age"] > .1 for r in rows),
        tick_duplicate_sample_pairs=duplicates, tick_backwards_pairs=reversals, tick_gap_over_50ms_pairs=gaps,
        basic_motor_checks_rejected_rows=sum(bool(motor_reasons(row)) for row in rows),
        raw_lost_semantics_verified=False, source_tick_alone_proves_each_motor_fresh=False,
        motors=motors,
        note="A constant nonzero raw lost field is NOT a health acceptance or proven historical packet counter. "
             "Growth, decreases and raw differences are observations only. No flags or raw fields are overridden.")


def snapshot_message(msg, received):
    row = dict(low_tick=int(msg.tick), received=received)
    for i, motor in enumerate(msg.motor_state[:12]):
        for key, value in (("q", motor.q), ("dq", motor.dq), ("tau", motor.tau_est),
                           ("ddq_reported", motor.ddq), ("temperature", motor.temperature),
                           ("q_raw", motor.q_raw), ("dq_raw", motor.dq_raw), ("ddq_raw", motor.ddq_raw)):
            row[f"{key}_{i}"] = float(value)
        row[f"mode_{i}"] = int(motor.mode)
        row[f"lost_{i}"] = int(motor.lost)
        for j, value in enumerate(motor.reserve):
            row[f"reserve_{i}_{j}"] = int(value)
    for prefix, values in (("quat", msg.imu_state.quaternion), ("gyro", msg.imu_state.gyroscope),
                           ("accel", msg.imu_state.accelerometer)):
        for i, value in enumerate(values):
            row[f"{prefix}_{i}"] = float(value)
    for i,value in enumerate(msg.version):
        row[f"firmware_version_raw_{i}"] = int(value)
    row["bit_flag_raw"] = int(msg.bit_flag)
    row["crc_raw"] = int(msg.crc) # preserved only, NOT a CRC validation claim
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface")
    parser.add_argument("--seconds", type=float, default=15.)
    parser.add_argument("--posture", choices=("prone", "standing", "unknown"), default="unknown")
    parser.add_argument("--output", type=Path, required=True, help="New CSV/JSON prefix")
    parser.add_argument("--sdk-root", type=Path, default=Path.home()/"unitree_sdk2_python")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not .5 <= args.seconds <= 60:
        parser.error("Only finite 0.5..60 second read-only capture is allowed")
    if not (Path("/sys/class/net")/args.interface).is_dir():
        parser.error("No such network interface")
    csv_path, json_path = Path(str(args.output)+".csv"), Path(str(args.output)+".json")
    if csv_path.exists() or json_path.exists():
        parser.error("Refusing to overwrite capture")
    sys.path.insert(0, str(args.sdk_root))
    # Imports remain below argument checks; importing/testing this file is offline.
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

    lock = threading.Lock()
    latest = {}

    def callback(msg):
        row = snapshot_message(msg, time.monotonic())
        with lock:
            row["source_received"] = row["received"] if not latest or row["low_tick"] != latest["low_tick"] else latest["source_received"]
            latest.clear()
            latest.update(row)

    print("READ ONLY: rt/lowstate subscriber; no B2/Z1 commands or controller release.", flush=True)
    ChannelFactoryInitialize(0, args.interface)
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(callback, 10)
    rows, previous = [], None
    start = time.monotonic()
    interrupted = False
    try:
        while time.monotonic()-start < args.seconds:
            with lock:
                row = dict(latest)
            if row:
                now = time.monotonic()
                row["elapsed"] = now-start
                row["arrival_age"] = now-row.pop("received")
                row["low_age"] = now-row.pop("source_received")
                reasons = motor_reasons(row)
                row["basic_motor_reasons"] = "; ".join(reasons)
                rows.append(row)
                signature = tuple(row[f"lost_{i}"] for i in range(12))
                if signature != previous:
                    print(json.dumps(dict(elapsed=row["elapsed"], tick=row["low_tick"],
                                          lost=signature, reasons=reasons)), flush=True)
                    previous = signature
            time.sleep(.01)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        sub.Close()
    report = summarize(rows, args.posture)
    report["interrupted"] = interrupted
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    with json_path.open("x") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps(report, indent=2, allow_nan=False), flush=True)
    return 130 if interrupted else 0


if __name__ == "__main__":
    raise SystemExit(main())
