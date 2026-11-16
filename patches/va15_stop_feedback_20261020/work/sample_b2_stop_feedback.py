#!/usr/bin/env python3
"""Read-only lowstate/sport-state capture. No publishers or motion clients."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import sys
import threading
import time


def describe(values):
    finite = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not finite:
        return {"count": 0}
    return {"count": len(finite), "min": finite[0], "median": statistics.median(finite),
            "p05": finite[int((len(finite) - 1) * .05)],
            "p95": finite[int((len(finite) - 1) * .95)], "max": finite[-1]}


def summarize(rows, posture):
    names = ("FR", "FL", "RR", "RL")
    return {"operator_posture_label": posture, "read_only": True, "samples": len(rows),
            "calibration_generated": False, "motion_qualified": False,
            "raw_bridge_dds_order": list(names),
            "raw_bridge": {name: describe(row["raw_" + name] for row in rows) for name in names},
            "sport_height_raw": describe(row["sport_height"] for row in rows),
            "sport_speed_raw": describe(row["sport_speed"] for row in rows),
            "fresh_sport_rows": sum(row["sport_age"] <= .15 and row["sport_error"] == 0 for row in rows),
            "fresh_sport_receive_rows": sum(row.get("sport_receive_age", row["sport_age"]) <= .15 for row in rows),
            "note": "Raw bridge counts are not Newtons. Posture labels do not independently prove foot loading. "
                    "Sport fields must also be validated after low-level takeover before motion is enabled."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface")
    parser.add_argument("--seconds", type=float, default=10.)
    parser.add_argument("--posture", choices=("prone", "official_stand", "custom_stand"), required=True)
    parser.add_argument("--output", type=Path, required=True, help="New CSV/JSON filename prefix")
    parser.add_argument("--sdk-root", type=Path, default=Path.home() / "unitree_sdk2_python")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not .5 <= args.seconds <= 60:
        parser.error("--seconds must be 0.5 to 60")
    if not (Path("/sys/class/net") / args.interface).is_dir():
        parser.error("Network interface does not exist: " + args.interface)
    csv_path, json_path = Path(str(args.output) + ".csv"), Path(str(args.output) + ".json")
    if csv_path.exists() or json_path.exists():
        parser.error("Output already exists; choose a new --output prefix")
    sys.path.insert(0, str(args.sdk_root))
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_

    lock = threading.Lock()
    latest = {"low": None, "sport": None}

    def low_callback(msg):
        values = {"tick": int(msg.tick), "received": time.monotonic(),
                  "raw": list(msg.foot_force), "q": [float(m.q) for m in msg.motor_state[:12]],
                  "dq": [float(m.dq) for m in msg.motor_state[:12]],
                  "tau": [float(m.tau_est) for m in msg.motor_state[:12]],
                  "quat": list(msg.imu_state.quaternion), "gyro": list(msg.imu_state.gyroscope)}
        with lock:
            if latest["low"] is None or values["tick"] != latest["low"]["tick"]:
                latest["low"] = values

    def sport_callback(msg):
        now = time.monotonic()
        values = {"stamp": int(msg.stamp.sec) * 1000000000 + int(msg.stamp.nanosec),
                  "received": now, "height": float(msg.body_height),
                  "speed": math.hypot(float(msg.velocity[0]), float(msg.velocity[1])),
                  "error": int(msg.error_code)}
        with lock:
            previous = latest["sport"]
            advanced = values["stamp"] > 0 and (previous is None or values["stamp"] > previous["stamp"])
            values["stamp_advanced_at"] = now if advanced else (
                previous["stamp_advanced_at"] if previous is not None and values["stamp"] == previous["stamp"] else None)
            # Record every arrival, even when the source uses a zero stamp.
            # Arrival freshness and proven source-time advancement are distinct.
            latest["sport"] = values

    print("Read-only capture: no B2/Z1 commands, no factory-controller release. Keep the robot still.", flush=True)
    ChannelFactoryInitialize(0, args.interface)
    low_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    sport_subscriber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    low_subscriber.Init(low_callback, 10)
    sport_subscriber.Init(sport_callback, 10)
    rows, last_tick = [], None
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.seconds:
            with lock:
                low, sport = latest["low"], latest["sport"]
            now = time.monotonic()
            if low is not None and low["tick"] != last_tick:
                last_tick = low["tick"]
                row = {"elapsed": now - start, "low_tick": last_tick, "low_age": now - low["received"],
                       "sport_stamp": sport["stamp"] if sport else 0,
                       "sport_age": now - sport["stamp_advanced_at"] if sport and sport["stamp_advanced_at"] is not None else math.inf,
                       "sport_receive_age": now - sport["received"] if sport else math.inf,
                       "sport_error": sport["error"] if sport else -1,
                       "sport_height": sport["height"] if sport else math.nan,
                       "sport_speed": sport["speed"] if sport else math.nan}
                for i, name in enumerate(("FR", "FL", "RR", "RL")): row["raw_" + name] = low["raw"][i]
                for key in ("q", "dq", "tau", "quat", "gyro"):
                    for i, value in enumerate(low[key]): row[f"{key}_{i}"] = value
                rows.append(row)
            time.sleep(.01)
    except KeyboardInterrupt:
        print("Capture interrupted; saving received samples.")
    finally:
        low_subscriber.Close()
        sport_subscriber.Close()
    if not rows:
        raise SystemExit("No lowstate samples received; no calibration or motion permission created.")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("x", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    summary = summarize(rows, args.posture)
    with json_path.open("x") as handle:
        json.dump(summary, handle, indent=2, allow_nan=False)
    print(json.dumps(summary, indent=2, allow_nan=False))
    print("Saved:", csv_path, json_path)


if __name__ == "__main__":
    main()
