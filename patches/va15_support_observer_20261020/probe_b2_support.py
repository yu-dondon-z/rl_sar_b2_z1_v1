#!/usr/bin/env python3
"""Read-only B2 support-source probe/replay. No publishers or motion clients."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import threading
import time

import numpy as np
from va15_support_candidate import ORDER, export_model, load_candidate


def statistics(values):
    values = np.array([v for v in values if math.isfinite(float(v))], dtype=float)
    return {} if not len(values) else dict(count=len(values), p05=float(np.percentile(values, 5)),
        median=float(np.median(values)), p95=float(np.percentile(values, 95)))


def assess(rows, candidate):
    results = [candidate.evaluate(row) for row in rows]
    numeric = [r for r in results if r["model_valid"]]
    return dict(read_only=True, samples=len(rows), control_authorized=False,
        motion_qualified=False, contact_calibration_generated=False,
        model_source_sha256=candidate.model["source_urdf_sha256"],
        lowstate_fresh_rows=sum(0 <= float(r["low_age"]) <= .1 for r in rows),
        numerical_model_rows=len(numeric), quasi_static_rows=sum(r["quasi_static"] for r in results),
        force_equivalent_z_dds={name:statistics(r["force_equivalent_world"][i][2] for r in numeric)
                                for i, name in enumerate(ORDER)},
        raw_bridge_dds={name:statistics(float(r["raw_"+name]) for r in rows) for name in ORDER},
        height_hypothesis=statistics(r["height_hypothesis"] for r in numeric),
        no_slip_speed_hypothesis=statistics(np.linalg.norm(r["no_slip_velocity_hypothesis"][:2]) for r in numeric),
        invalid_reasons={reason:sum(r["reason"] == reason for r in results)
                         for reason in sorted(set(r["reason"] for r in results))},
        note="N-equivalent under static-model assumptions, NOT measured Newtons or certified contacts. "
             "Prone/standing labels are operator reports. No walking or stopping qualification.")


def capture(args):
    if not (Path("/sys/class/net") / args.interface).is_dir():
        raise ValueError("Missing interface: " + args.interface)
    sys.path.insert(0, str(args.sdk_root))
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_
    lock, latest, sport = threading.Lock(), {}, {}
    def low_callback(msg):
        value = dict(low_tick=int(msg.tick), received=time.monotonic())
        for key, values in (("raw", msg.foot_force), ("raw_est", msg.foot_force_est)):
            for name, v in zip(ORDER, values): value[key+"_"+name] = float(v)
        for key in ("q", "dq", "ddq", "tau_est", "mode", "lost"):
            for i, motor in enumerate(msg.motor_state[:12]):
                value[f"{'tau' if key == 'tau_est' else key}_{i}"] = (
                    int(getattr(motor, key)) if key in ("mode", "lost") else float(getattr(motor, key)))
        for key, values in (("quat", msg.imu_state.quaternion), ("gyro", msg.imu_state.gyroscope),
                            ("accel", msg.imu_state.accelerometer)):
            for i, v in enumerate(values): value[f"{key}_{i}"] = float(v)
        with lock:
            if not latest or value["low_tick"] != latest["low_tick"]:
                latest.clear(); latest.update(value)
    def sport_callback(name):
        def callback(msg):
            with lock:
                previous = sport.get(name, {})
                stamp = int(msg.stamp.sec)*1000000000+int(msg.stamp.nanosec)
                now = time.monotonic()
                advanced = stamp > 0 and stamp > previous.get("stamp", 0)
                sport[name] = dict(stamp=stamp, received=now,
                    advanced_at=now if advanced else previous.get("advanced_at", -math.inf),
                    height=float(msg.body_height), speed=math.hypot(*msg.velocity[:2]), error=int(msg.error_code))
        return callback
    print("READ ONLY: no lowcmd, motion service, Z1 SDK or control configuration writes.", flush=True)
    ChannelFactoryInitialize(0, args.interface)
    subscribers, rows = [], []
    try:
        for topic, datatype, callback in [("rt/lowstate", LowState_, low_callback),
                ("rt/sportmodestate", SportModeState_, sport_callback("sport")),
                ("rt/lf/sportmodestate", SportModeState_, sport_callback("lf_sport"))]:
            subscriber = ChannelSubscriber(topic, datatype)
            subscribers.append(subscriber)
            subscriber.Init(callback, 10)
        start, last_tick, last_receive = time.monotonic(), None, None
        while time.monotonic()-start < args.seconds:
            with lock:
                row = latest.copy()
                snapshot = {name:v.copy() for name,v in sport.items()}
            now = time.monotonic()
            if row and row["low_tick"] != last_tick:
                received = row.pop("received")
                delta = (row["low_tick"]-last_tick) % (2**32) if last_tick is not None else 0
                row.update(elapsed=now-start, low_age=now-received,
                    sequence_invalid=last_tick is None or not 0 < delta <= 50 or received-last_receive > .1)
                last_tick, last_receive = row["low_tick"], received
                for name in ("sport", "lf_sport"):
                    value = snapshot.get(name, {})
                    row[name+"_stamp"] = value.get("stamp", 0)
                    row[name+"_receive_age"] = now-value.get("received", -math.inf)
                    row[name+"_source_age"] = now-value.get("advanced_at", -math.inf)
                    row[name+"_height"] = value.get("height", math.nan)
                    row[name+"_speed"] = value.get("speed", math.nan)
                    row[name+"_error"] = value.get("error", -1)
                rows.append(row)
            time.sleep(.01)
    except KeyboardInterrupt:
        print("Read-only probe interrupted; saving completed samples.")
    finally:
        for subscriber in reversed(subscribers): subscriber.Close()
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface", nargs="?")
    parser.add_argument("--seconds", type=float, default=10.)
    parser.add_argument("--posture", choices=("prone", "official_stand", "custom_stand"))
    parser.add_argument("--sdk-root", type=Path, default=Path.home()/"unitree_sdk2_python")
    parser.add_argument("--model", type=Path, default=Path(__file__).with_name("b2_support_model.json"))
    parser.add_argument("--export-urdf", type=Path)
    parser.add_argument("--replay", type=Path)
    parser.add_argument("--output", type=Path, required=True, help="New directory; existing paths are refused")
    args = parser.parse_args()
    if args.output.exists(): parser.error("Output already exists")
    if args.export_urdf:
        args.output.mkdir(parents=True)
        (args.output/"b2_support_model.json").write_text(json.dumps(export_model(args.export_urdf), indent=2))
        return
    if not args.replay and (not args.interface or not args.posture or not math.isfinite(args.seconds) or not .5 <= args.seconds <= 60):
        parser.error("Live probe needs interface, posture and 0.5 <= seconds <= 60")
    candidate = load_candidate(args.model)
    if args.replay:
        with args.replay.open(newline="") as handle: rows = list(csv.DictReader(handle))
        for row in rows:
            row["sequence_invalid"] = str(row.get("sequence_invalid", "False")).lower() in ("true", "1")
    else:
        rows = capture(args)
    if not rows: raise SystemExit("No samples. No changes to calibration or robot control.")
    args.output.mkdir(parents=True)
    with (args.output/"samples.csv").open("x", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    report = assess(rows, candidate)
    report.update(operator_posture_label=args.posture, replay_source=str(args.replay) if args.replay else None,
                  samples_sha256=hashlib.sha256((args.output/"samples.csv").read_bytes()).hexdigest())
    for topic in ("sport", "lf_sport"):
        report[topic+"_proven_fresh_rows"] = sum(float(r.get(topic+"_source_age", math.inf)) <= .15 for r in rows)
    (args.output/"report.json").write_text(json.dumps(report, indent=2, allow_nan=False))
    print(json.dumps(report, indent=2, allow_nan=False))
    print("Saved", args.output, "-- no calibration or motion permission created.")


if __name__ == "__main__": main()
