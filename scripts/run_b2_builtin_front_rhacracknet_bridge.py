#!/usr/bin/env python3
"""Extract a metric crack path from B2's built-in FrontVideoClient stream."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import yaml


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("network_interface")
    parser.add_argument("--camera-calibration", required=True)
    parser.add_argument("--exchange-dir", default="/tmp/b2_z1_real_vision")
    parser.add_argument("--unitree-sdk2-python", default="/home/dong/unitree_sdk2_python")
    parser.add_argument("--rhacracknet-root", default="/home/dong/RHACrackNet/code")
    parser.add_argument(
        "--checkpoint",
        default=("/home/dong/RHACrackNet/code/exps/"
                 "rhanet_dep_CamCrack789_b2_stage1/ckpt/best_ckpt.pth"),
    )
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--threshold", type=float, default=0.90)
    parser.add_argument("--spacing-px", type=float, default=8.0)
    parser.add_argument("--spacing-m", type=float, default=0.03)
    parser.add_argument("--min-confidence", type=float, default=0.90)
    parser.add_argument("--min-path-length", type=float, default=0.30)
    parser.add_argument("--min-forward-span", type=float, default=0.20)
    parser.add_argument("--minimum-valid-streak", type=int, default=3)
    parser.add_argument("--display", action="store_true")
    parser.add_argument("--save-overlay", action="store_true")
    return parser.parse_args()


def write_json_atomic(path: Path, payload: dict):
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    temporary.replace(path)


def load_calibration(path: Path):
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(config, dict) or config.get("calibrated") is not True:
        raise RuntimeError(
            "built-in front-camera calibration is not approved; keep motor control disabled"
        )
    intrinsics = config["intrinsics"]
    values = [float(intrinsics[name]) for name in ("fx", "fy", "cx", "cy")]
    if values[0] <= 0.0 or values[1] <= 0.0:
        raise ValueError("camera fx/fy must be measured positive values")
    transform = np.asarray(config["T_base_camera"], dtype=np.float64)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError("T_base_camera must be a finite 4x4 matrix")
    plane_config = config["road_plane_base"]
    normal = tuple(float(value) for value in plane_config["normal"])
    offset = float(plane_config["offset"])
    return config, values, transform, normal, offset


def main():
    args = parse_args()
    calibration, intrinsics, transform, plane_normal, plane_offset = load_calibration(
        Path(args.camera_calibration).expanduser().resolve())
    for source_root in (args.unitree_sdk2_python, args.rhacracknet_root):
        resolved = str(Path(source_root).expanduser().resolve())
        if resolved not in sys.path:
            sys.path.insert(0, resolved)

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.b2.front_video.front_video_client import FrontVideoClient
    from crack_inference import CrackSegmenter
    from crack_trajectory import (
        CenterlineConfig, PinholeCamera, Plane, build_metric_trajectory,
        draw_trajectory_overlay, extract_centerline, project_pixels_to_plane,
    )

    exchange = Path(args.exchange_dir).expanduser().resolve()
    exchange.mkdir(parents=True, exist_ok=True)
    trajectory_path = exchange / "latest_trajectory.json"
    status_path = exchange / "latest_vision_status.json"
    trajectory_path.unlink(missing_ok=True)

    ChannelFactoryInitialize(0, args.network_interface)
    camera_client = FrontVideoClient()
    camera_client.SetTimeout(1.0)
    camera_client.Init()
    segmenter = CrackSegmenter(args.checkpoint, "RHANet_dep", args.device,
                               args.threshold)
    camera = PinholeCamera(*intrinsics,
        distortion=tuple(float(v) for v in calibration["intrinsics"].get("distortion", [])))
    road_plane = Plane(plane_normal, plane_offset)
    centerline_config = CenterlineConfig(waypoint_spacing_px=args.spacing_px)
    warmed_up = False
    sequence = 0
    valid_streak = 0
    print("[RealVision] B2 FrontVideoClient RGB stream started")
    try:
        while True:
            code, encoded = camera_client.GetImageSample()
            capture_monotonic_ns = time.monotonic_ns()
            if code != 0 or not encoded:
                trajectory_path.unlink(missing_ok=True)
                valid_streak = 0
                time.sleep(0.02)
                continue
            image = cv2.imdecode(np.frombuffer(bytes(encoded), dtype=np.uint8),
                                 cv2.IMREAD_COLOR)
            if image is None:
                continue
            sequence += 1
            if not warmed_up:
                segmenter.warmup(image.shape[0], image.shape[1])
                warmed_up = True
            overlay = image.copy()
            try:
                result = segmenter.predict_bgr(image)
                anchor = ((image.shape[1] - 1) * 0.5,
                          (image.shape[0] - 1) * 0.62)
                centerline = extract_centerline(result.mask, centerline_config, anchor)
                pixels = centerline.waypoints_uv
                uv = np.rint(pixels).astype(int)
                uv[:, 0] = np.clip(uv[:, 0], 0, image.shape[1] - 1)
                uv[:, 1] = np.clip(uv[:, 1], 0, image.shape[0] - 1)
                confidence = float(np.mean(result.probability[uv[:, 1], uv[:, 0]]))
                if confidence < args.min_confidence:
                    raise RuntimeError(f"confidence too low: {confidence:.3f}")
                points, valid = project_pixels_to_plane(
                    pixels, camera, transform, road_plane)
                control = build_metric_trajectory(points, valid, args.spacing_m, 0.30)
                if len(control) < 3:
                    raise RuntimeError("fewer than three road-plane waypoints")
                if control[0, 0] > control[-1, 0]:
                    control = control[::-1].copy()
                length = float(np.linalg.norm(np.diff(control, axis=0), axis=1).sum())
                span = float(control[-1, 0] - control[0, 0])
                if length < args.min_path_length or span < args.min_forward_span:
                    raise RuntimeError(f"path too short: length={length:.2f}, span={span:.2f}")
                valid_streak += 1
                if valid_streak < args.minimum_valid_streak:
                    trajectory_path.unlink(missing_ok=True)
                    raise RuntimeError(
                        f"waiting for consistent frames {valid_streak}/{args.minimum_valid_streak}")
                payload = {
                    "source_sequence": sequence,
                    "capture_monotonic_ns": capture_monotonic_ns,
                    "frame_id": "b2_base",
                    "camera_name": calibration.get("camera_name", "b2_builtin_front"),
                    "projection_method": "calibrated_local_road_plane",
                    "inference_ms": float(result.inference_ms),
                    "mean_waypoint_confidence": confidence,
                    "path_length_m": length,
                    "forward_span_m": span,
                    "control_waypoints_robot_xyz": control.tolist(),
                }
                write_json_atomic(trajectory_path, payload)
                write_json_atomic(status_path, {"valid": True, **payload})
                overlay = draw_trajectory_overlay(image, centerline)
                label, color = f"VALID RGB path: {length:.2f} m", (30, 220, 30)
            except (KeyError, RuntimeError, TypeError, ValueError) as error:
                if not str(error).startswith("waiting for consistent"):
                    valid_streak = 0
                trajectory_path.unlink(missing_ok=True)
                write_json_atomic(status_path, {
                    "valid": False, "source_sequence": sequence,
                    "capture_monotonic_ns": capture_monotonic_ns,
                    "reason": str(error),
                })
                label, color = f"INVALID: {error}", (20, 20, 240)
            cv2.putText(overlay, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, color, 2, cv2.LINE_AA)
            if args.save_overlay:
                temporary = exchange / f".latest_overlay.{os.getpid()}.png"
                if cv2.imwrite(str(temporary), overlay):
                    temporary.replace(exchange / "latest_overlay.png")
            if args.display:
                cv2.imshow("B2 built-in front camera crack trajectory", overlay)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        trajectory_path.unlink(missing_ok=True)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
