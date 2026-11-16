#!/usr/bin/env python3
"""Process paired front RGB and rear RGB-D frames without robot middleware."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np


SCHEMA = "b2_z1_dual_vision_v1"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exchange-dir", type=Path, required=True)
    parser.add_argument("--rhacracknet-root", type=Path, default=Path("/home/dong/RHACrackNet/code"))
    parser.add_argument("--checkpoint", type=Path, default=Path(
        "/home/dong/RHACrackNet/code/exps/rhanet_dep_CamCrack789_b2_stage1/ckpt/best_ckpt.pth"))
    parser.add_argument("--model", choices=("RHANet", "RHANet_dep"), default="RHANet_dep")
    parser.add_argument("--mode", choices=("rhanet", "classical"), default="rhanet",
                        help="classical is an explicit scene diagnostic, not a learned detector")
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--threshold", type=float, default=0.90)
    parser.add_argument("--input-height", type=int, default=360)
    parser.add_argument("--input-width", type=int, default=640)
    parser.add_argument("--max-frames", type=int, default=0, help="Number of paired captures; 0 runs until Ctrl-C")
    parser.add_argument("--max-frame-age-s", type=float, default=1.0)
    parser.add_argument("--poll-period", type=float, default=0.01)
    parser.add_argument("--display", action="store_true")
    parser.add_argument("--save-overlay", action="store_true")
    return parser.parse_args()


def write_json_atomic(path: Path, payload: dict) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(payload, allow_nan=False, indent=2), encoding="utf-8")
    temporary.replace(path)


def capture_key(metadata: dict) -> tuple[int, int]:
    return int(metadata["capture_generation"]), int(metadata["sequence"])


def read_frame(exchange: Path, stream: str, require_depth: bool):
    """Read a complete ring-buffer slot and detect replacement during the read."""
    directory = exchange / stream
    metadata_path = directory / "latest_frame.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    key = capture_key(metadata)
    capture_time = float(metadata["capture_timestamp"])
    if not math.isfinite(capture_time):
        raise ValueError("capture timestamp must be finite")

    def image_path(field):
        path = (directory / metadata[field]).resolve()
        if path.parent != directory.resolve():
            raise ValueError(f"{stream} image must be inside its stream directory")
        return path

    rgb = cv2.imread(str(image_path("rgb_file")), cv2.IMREAD_COLOR)
    depth = cv2.imread(str(image_path("depth_file")), cv2.IMREAD_UNCHANGED) if require_depth else None
    if rgb is None or (require_depth and (depth is None or depth.ndim != 2 or depth.shape != rgb.shape[:2])):
        raise ValueError(f"{stream} image slot is incomplete")
    current = json.loads(metadata_path.read_text(encoding="utf-8"))
    if capture_key(current) != key or current.get("capture_timestamp") != metadata["capture_timestamp"]:
        raise ValueError(f"{stream} capture changed during image read")
    # Preserve the publisher's monotonic capture time. This is never refreshed
    # from file modification time, processing time, or the other camera.
    metadata["capture_timestamp_ns"] = int(capture_time * 1_000_000_000)
    mounting = metadata.get("camera_to_base")
    if mounting:
        metadata["camera_position_b"] = mounting["position"]
        metadata["camera_quaternion_b_ros_wxyz"] = mounting["quaternion_wxyz"]
    return metadata, rgb, depth


def result_with_capture(result: dict, metadata: dict) -> dict:
    result = dict(result)
    result.update(sequence=int(metadata["sequence"]),
                  capture_generation=int(metadata["capture_generation"]),
                  capture_timestamp=float(metadata["capture_timestamp"]),
                  frame_id="b2_base_at_capture")
    for field in ("base_position_w", "base_quaternion_w", "capture_sim_time_s"):
        if field in metadata:
            result[field] = metadata[field]
    result.setdefault("valid", False)
    result.setdefault("reason", "")
    result.setdefault("points_b", [])
    return result


def overlay_frame(image: np.ndarray, result: dict, label: str) -> np.ndarray:
    overlay = image.copy()
    uv = np.asarray(result.get("waypoints_uv", []), dtype=np.float64)
    if uv.size and uv.ndim == 2 and uv.shape[1] == 2 and np.isfinite(uv).all():
        cv2.polylines(overlay, [np.rint(uv).astype(np.int32)], False, (30, 240, 40), 2, cv2.LINE_AA)
    for name, marker in result.get("markers", {}).items():
        if not isinstance(marker, dict):
            continue
        point = marker.get("uv", marker.get("center_uv"))
        if point is not None and len(point) == 2 and np.isfinite(point).all():
            u, v = np.rint(point).astype(int)
            cv2.circle(overlay, (u, v), 7, (0, 255, 255), 2)
            cv2.putText(overlay, str(name), (u + 8, v), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 255), 1)
    tcp_uv = result.get("tcp_pixel_uv")
    if tcp_uv is not None and len(tcp_uv) == 2 and np.isfinite(tcp_uv).all():
        cv2.drawMarker(overlay, tuple(np.rint(tcp_uv).astype(int)), (255, 255, 0), cv2.MARKER_CROSS, 18, 2)
    valid = bool(result.get("valid"))
    color = (30, 230, 40) if valid else (30, 70, 255)
    status = f"{label} {'VALID' if valid else 'WAIT'}  seq={result.get('sequence', '?')}"
    cv2.rectangle(overlay, (0, 0), (overlay.shape[1], 64), (25, 25, 25), -1)
    cv2.putText(overlay, status, (10, 23), cv2.FONT_HERSHEY_SIMPLEX, .55, color, 1, cv2.LINE_AA)
    text = result.get("reason", "")
    if valid and "tcp_error_m" in result:
        text = f"projected TCP-to-seam error: {100 * result['tcp_error_m']:.1f} cm"
    cv2.putText(overlay, str(text)[:100], (10, 48), cv2.FONT_HERSHEY_SIMPLEX, .43, color, 1, cv2.LINE_AA)
    return overlay


def depth_panel(depth: np.ndarray, scale: float) -> np.ndarray:
    metres = depth.astype(np.float32) * scale
    valid = np.isfinite(metres) & (metres > 0)
    normal = np.zeros(depth.shape, dtype=np.uint8)
    normal[valid] = np.clip(255 * (1 - metres[valid] / 3.0), 0, 255).astype(np.uint8)
    panel = cv2.applyColorMap(normal, cv2.COLORMAP_TURBO)
    panel[~valid] = 0
    cv2.putText(panel, "Rear aligned depth (0-3 m)", (10, 23), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1)
    return panel


def main() -> None:
    args = parse_args()
    if args.max_frames < 0 or args.max_frame_age_s <= 0 or args.poll_period <= 0:
        raise ValueError("frame count must be nonnegative; timing limits must be positive")
    if args.input_height <= 0 or args.input_width <= 0:
        raise ValueError("inference image dimensions must be positive")
    root = args.rhacracknet_root.expanduser().resolve()
    sys.path.insert(0, str(root))
    from dual_camera_perception import DualCameraPerception, PerceptionConfig

    segmenter = None
    if args.mode == "rhanet":
        from crack_inference import CrackSegmenter
        segmenter = CrackSegmenter(args.checkpoint, args.model, args.device, args.threshold,
                                   input_size=(args.input_height, args.input_width))
        segmenter.warmup(args.input_height, args.input_width)
    perception = DualCameraPerception(config=PerceptionConfig(mode=args.mode, rhacracknet_root=str(root)),
                                      segmenter=segmenter)
    exchange = args.exchange_dir.expanduser().resolve()
    exchange.mkdir(parents=True, exist_ok=True)
    last_key = None
    processed = 0
    status_key = None
    print(f"[Dual vision] mode={args.mode}; watching {exchange}/{{front,rear}}/latest_frame.json", flush=True)
    print("[Dual vision] rear depth estimates ground; front uses RGB and that measured plane.", flush=True)
    write_json_atomic(exchange / "vision_ready.json", {"ready": True, "mode": args.mode,
                                                      "model": args.model if segmenter else None,
                                                      "timestamp": time.time()})
    with (exchange / "dual_vision_packets.jsonl").open("a", encoding="utf-8") as history:
        try:
            while args.max_frames == 0 or processed < args.max_frames:
                try:
                    front_metadata, front_rgb, _ = read_frame(exchange, "front", False)
                    rear_metadata, rear_rgb, rear_depth = read_frame(exchange, "rear", True)
                    key = capture_key(front_metadata)
                    if key != capture_key(rear_metadata) or key == last_key:
                        time.sleep(args.poll_period)
                        continue
                    if abs(front_metadata["capture_timestamp"] - rear_metadata["capture_timestamp"]) > .04:
                        raise ValueError("paired camera captures differ by more than 40 ms")
                except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
                    time.sleep(args.poll_period)
                    continue
                last_key = key
                processed += 1
                start = time.monotonic()
                oldest_capture = min(front_metadata["capture_timestamp"], rear_metadata["capture_timestamp"])
                if start - oldest_capture > args.max_frame_age_s or oldest_capture > start + .01:
                    reason = "capture timestamp is stale or in the future"
                    rear = {"valid": False, "reason": reason}
                    front = {"valid": False, "reason": reason}
                else:
                    try:
                        rear = perception.process_rear(rear_metadata, rear_rgb, rear_depth)
                    except (KeyError, RuntimeError, TypeError, ValueError) as error:
                        rear = {"valid": False, "reason": f"rear perception: {error}"}
                    ground = rear.get("ground_plane_b")
                    if ground is not None:
                        front_metadata["ground_plane_b"] = ground
                        front_metadata["ground_plane_source"] = "paired_rear_depth"
                        try:
                            front = perception.process_front(front_metadata, front_rgb)
                        except (KeyError, RuntimeError, TypeError, ValueError) as error:
                            front = {"valid": False, "reason": f"front perception: {error}"}
                    else:
                        front = {"valid": False, "reason": "rear depth ground plane unavailable"}
                rear = result_with_capture(rear, rear_metadata)
                front = result_with_capture(front, front_metadata)
                now = time.monotonic()
                age = now - oldest_capture
                # Expired measurements stay expired even after successful inference.
                if age > args.max_frame_age_s:
                    for result in (front, rear):
                        result.update(valid=False, reason="capture expired during processing")
                valid = bool(front["valid"] and rear["valid"])
                reason = "paired front steering and rear TCP measurements valid" if valid else (
                    f"front: {front['reason']}; rear: {rear['reason']}")
                packet = {"schema": SCHEMA, "timestamp": time.time(), "capture_generation": key[0],
                          "mode": args.mode, "model": args.model if segmenter else None,
                          "front": front, "rear": rear, "valid": valid, "reason": reason,
                          "processing_ms": 1000 * (now - start), "capture_age_s": age}
                write_json_atomic(exchange / "latest_dual_trajectory.json", packet)
                write_json_atomic(exchange / "latest_vision_status.json", {
                    "schema": SCHEMA, "valid": valid, "reason": reason,
                    "front_sequence": front["sequence"], "rear_sequence": rear["sequence"],
                    "mode": args.mode, "processing_ms": packet["processing_ms"], "capture_age_s": age})
                history.write(json.dumps(packet, allow_nan=False) + "\n")
                history.flush()
                current_status = (valid, reason)
                if current_status != status_key or processed % 25 == 0:
                    print(f"[Dual vision] pair={key} {'VALID' if valid else 'WAIT'} "
                          f"age={age:.3f}s processing={packet['processing_ms']:.1f}ms {reason}", flush=True)
                    status_key = current_status
                if args.save_overlay or args.display:
                    front_overlay = overlay_frame(front_rgb, front, "Front RGB / steering")
                    rear_overlay = overlay_frame(rear_rgb, rear, "Rear D435i / TCP")
                    depth_overlay = depth_panel(rear_depth, float(rear_metadata.get("depth_unit_m", .001)))
                    height = min(360, front_overlay.shape[0], rear_overlay.shape[0])
                    panels = [cv2.resize(panel, (640, height)) for panel in (front_overlay, rear_overlay, depth_overlay)]
                    panel = np.hstack(panels)
                    if args.save_overlay:
                        temporary = exchange / f".latest_dual_overlay.{os.getpid()}.tmp.png"
                        if cv2.imwrite(str(temporary), panel):
                            temporary.replace(exchange / "latest_dual_overlay.png")
                    if args.display:
                        cv2.imshow("VA16: front RGB | rear RGB + measured TCP | rear depth", panel)
                        if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                            break
        except KeyboardInterrupt:
            pass
        finally:
            if args.display:
                cv2.destroyAllWindows()
            write_json_atomic(exchange / "vision_finished.json", {"processed_pairs": processed,
                "timestamp": datetime.now(timezone.utc).isoformat(), "mode": args.mode})


if __name__ == "__main__":
    main()
