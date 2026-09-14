#!/usr/bin/env python3
"""Publish timestamped B2-base crack paths from a calibrated D405 RGB-D camera."""

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
    parser.add_argument("--extrinsics", required=True,
                        help="calibrated camera-to-B2 YAML file")
    parser.add_argument("--exchange-dir", default="/tmp/b2_z1_real_vision")
    parser.add_argument("--rhacracknet-root", default="/home/dong/RHACrackNet/code")
    parser.add_argument(
        "--checkpoint",
        default=("/home/dong/RHACrackNet/code/exps/"
                 "rhanet_dep_CamCrack789_b2_stage1/ckpt/best_ckpt.pth"),
    )
    parser.add_argument("--model", choices=("RHANet", "RHANet_dep"),
                        default="RHANet_dep")
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--serial", default="",
                        help="RealSense serial; empty selects the first camera")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--threshold", type=float, default=0.90)
    parser.add_argument("--spacing-px", type=float, default=8.0)
    parser.add_argument("--spacing-m", type=float, default=0.03)
    parser.add_argument("--min-confidence", type=float, default=0.90)
    parser.add_argument("--min-depth-ratio", type=float, default=0.70)
    parser.add_argument("--min-path-length", type=float, default=0.30)
    parser.add_argument("--min-forward-span", type=float, default=0.20)
    parser.add_argument("--depth-probe-offset-px", type=float, default=6.0)
    parser.add_argument("--min-depth-contrast", type=float, default=0.010)
    parser.add_argument("--min-depth-support-ratio", type=float, default=0.45)
    parser.add_argument("--display", action="store_true")
    parser.add_argument("--save-overlay", action="store_true")
    return parser.parse_args()


def quaternion_wxyz_to_matrix(quaternion):
    quaternion = np.asarray(quaternion, dtype=np.float64)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise ValueError("camera quaternion must contain four finite wxyz values")
    norm = float(np.linalg.norm(quaternion))
    if norm < 1.0e-9:
        raise ValueError("camera quaternion has zero norm")
    w, x, y, z = quaternion / norm
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ], dtype=np.float64)


def write_json_atomic(path: Path, payload: dict):
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    temporary.replace(path)


def median_depth(depth, u, v, scale, radius=1):
    u, v = int(round(u)), int(round(v))
    if (u - radius < 0 or v - radius < 0 or
            u + radius >= depth.shape[1] or v + radius >= depth.shape[0]):
        return float("nan")
    values = depth[v-radius:v+radius+1, u-radius:u+radius+1]
    values = values.astype(np.float64).reshape(-1) * scale
    values = values[np.isfinite(values) & (values > 0.0)]
    return float(np.median(values)) if len(values) >= 3 else float("nan")


def depth_support(waypoints, depth, scale, offset, minimum_contrast):
    contrasts = []
    usable = 0
    for index, waypoint in enumerate(waypoints):
        before = waypoints[max(index - 1, 0)]
        after = waypoints[min(index + 1, len(waypoints) - 1)]
        tangent = after - before
        norm = float(np.linalg.norm(tangent))
        if norm < 1.0e-6:
            continue
        normal = np.array([-tangent[1], tangent[0]]) / norm
        center = median_depth(depth, *waypoint, scale)
        sides = [median_depth(depth, *(waypoint + sign * offset * normal), scale)
                 for sign in (-1.0, 1.0)]
        if np.isfinite(center) and np.all(np.isfinite(sides)):
            usable += 1
            contrasts.append(center - 0.5 * (sides[0] + sides[1]))
    probe_ratio = usable / max(len(waypoints), 1)
    support_ratio = (float(np.mean(np.asarray(contrasts) >= minimum_contrast))
                     if contrasts else 0.0)
    return support_ratio, probe_ratio, float(np.mean(contrasts)) if contrasts else 0.0


def side_probe_pixels(waypoints, offset):
    """Return symmetric road probes beside each centerline waypoint."""
    negative = np.full_like(waypoints, np.nan, dtype=np.float64)
    positive = np.full_like(waypoints, np.nan, dtype=np.float64)
    for index, waypoint in enumerate(waypoints):
        before = waypoints[max(index - 1, 0)]
        after = waypoints[min(index + 1, len(waypoints) - 1)]
        tangent = after - before
        norm = float(np.linalg.norm(tangent))
        if norm < 1.0e-6:
            continue
        normal = np.array([-tangent[1], tangent[0]]) / norm
        negative[index] = waypoint - offset * normal
        positive[index] = waypoint + offset * normal
    return negative, positive


def load_extrinsics(path: Path):
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(config, dict) or config.get("calibrated") is not True:
        raise RuntimeError(
            "camera extrinsics are not approved: set calibrated: true only after calibration"
        )
    position = np.asarray(config["camera_position_b_m"], dtype=np.float64)
    quaternion = np.asarray(config["camera_quaternion_b_ros_wxyz"], dtype=np.float64)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError("camera_position_b_m must contain three finite values")
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = quaternion_wxyz_to_matrix(quaternion)
    transform[:3, 3] = position
    return transform, position.tolist(), quaternion.tolist()


def main():
    args = parse_args()
    try:
        import pyrealsense2 as rs
    except ImportError as error:
        raise RuntimeError(
            "pyrealsense2 is required; install it in the crack_vision environment"
        ) from error

    code_root = Path(args.rhacracknet_root).expanduser().resolve()
    sys.path.insert(0, str(code_root))
    from crack_inference import CrackSegmenter
    from crack_trajectory import (
        CenterlineConfig, DepthProjectionConfig, PinholeCamera,
        build_metric_trajectory, draw_trajectory_overlay,
        extract_centerline, project_pixels_with_depth,
    )

    transform, camera_position, camera_quaternion = load_extrinsics(
        Path(args.extrinsics).expanduser().resolve())
    exchange = Path(args.exchange_dir).expanduser().resolve()
    exchange.mkdir(parents=True, exist_ok=True)
    trajectory_path = exchange / "latest_trajectory.json"
    status_path = exchange / "latest_vision_status.json"
    trajectory_path.unlink(missing_ok=True)

    pipeline = rs.pipeline()
    rs_config = rs.config()
    if args.serial:
        rs_config.enable_device(args.serial)
    rs_config.enable_stream(rs.stream.depth, args.width, args.height,
                            rs.format.z16, args.fps)
    rs_config.enable_stream(rs.stream.color, args.width, args.height,
                            rs.format.bgr8, args.fps)
    profile = pipeline.start(rs_config)
    align = rs.align(rs.stream.color)
    depth_scale = float(profile.get_device().first_depth_sensor().get_depth_scale())
    segmenter = CrackSegmenter(args.checkpoint, args.model, args.device,
                               args.threshold)
    centerline_config = CenterlineConfig(waypoint_spacing_px=args.spacing_px)
    segmenter.warmup(args.height, args.width)
    sequence = 0
    print(f"[RealVision] D405 running; output={trajectory_path}")
    try:
        while True:
            before_wait_ns = time.monotonic_ns()
            frames = align.process(pipeline.wait_for_frames(1000))
            after_wait_ns = time.monotonic_ns()
            capture_monotonic_ns = (before_wait_ns + after_wait_ns) // 2
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue
            image = np.asanyarray(color_frame.get_data()).copy()
            depth = np.asanyarray(depth_frame.get_data()).copy()
            sequence += 1
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

                support, probe_ratio, contrast = depth_support(
                    pixels, depth, depth_scale, args.depth_probe_offset_px,
                    args.min_depth_contrast)
                if probe_ratio < 0.60 or support < args.min_depth_support_ratio:
                    raise RuntimeError(
                        f"RGB line lacks crack depth support: support={support:.2f}, "
                        f"probes={probe_ratio:.2f}"
                    )
                intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
                camera = PinholeCamera(intrinsics.fx, intrinsics.fy,
                                       intrinsics.ppx, intrinsics.ppy)
                depth_config = DepthProjectionConfig(
                    scale_m_per_unit=depth_scale, min_depth_m=0.08,
                    max_depth_m=6.0, sample_radius_px=2, min_valid_samples=3)
                points, valid, depths = project_pixels_with_depth(
                    pixels, depth, camera, transform, depth_config)
                # The center pixel sees the bottom of a real groove. The TCP,
                # however, is specified above the road surface. Recover that
                # surface from symmetric probes beside the crack and retain
                # the centerline x/y. This avoids lowering the tool by the
                # crack depth a second time.
                side_negative_uv, side_positive_uv = side_probe_pixels(
                    pixels, args.depth_probe_offset_px)
                side_negative, side_negative_valid, _ = project_pixels_with_depth(
                    side_negative_uv, depth, camera, transform, depth_config)
                side_positive, side_positive_valid, _ = project_pixels_with_depth(
                    side_positive_uv, depth, camera, transform, depth_config)
                surface_valid = side_negative_valid & side_positive_valid
                points[surface_valid, 2] = 0.5 * (
                    side_negative[surface_valid, 2] +
                    side_positive[surface_valid, 2])
                valid = valid & surface_valid
                valid_ratio = float(np.mean(valid))
                if valid_ratio < args.min_depth_ratio:
                    raise RuntimeError(f"valid depth ratio too low: {valid_ratio:.2f}")
                control = build_metric_trajectory(points, valid, args.spacing_m, 0.25)
                if len(control) < 3:
                    raise RuntimeError("fewer than three metric points")
                if control[0, 0] > control[-1, 0]:
                    control = control[::-1].copy()
                length = float(np.linalg.norm(np.diff(control, axis=0), axis=1).sum())
                span = float(control[-1, 0] - control[0, 0])
                if length < args.min_path_length or span < args.min_forward_span:
                    raise RuntimeError(f"path too short: length={length:.2f}, span={span:.2f}")
                payload = {
                    "source_sequence": sequence,
                    "capture_monotonic_ns": capture_monotonic_ns,
                    "frame_id": "b2_base",
                    "camera_name": "b2_front_d405",
                    "camera_position_b_m": camera_position,
                    "camera_quaternion_b_ros_wxyz": camera_quaternion,
                    "inference_ms": float(result.inference_ms),
                    "mean_waypoint_confidence": confidence,
                    "depth_support_ratio": support,
                    "depth_probe_ratio": probe_ratio,
                    "mean_depth_contrast_m": contrast,
                    "z_reference": "adjacent_road_surface",
                    "valid_3d_ratio": valid_ratio,
                    "path_length_m": length,
                    "forward_span_m": span,
                    "control_waypoints_robot_xyz": control.tolist(),
                }
                write_json_atomic(trajectory_path, payload)
                write_json_atomic(status_path, {"valid": True, **payload})
                overlay = draw_trajectory_overlay(image, centerline)
                label = f"VALID 3-D crack: {length:.2f} m"
                color = (30, 220, 30)
            except (KeyError, RuntimeError, TypeError, ValueError) as error:
                trajectory_path.unlink(missing_ok=True)
                write_json_atomic(status_path, {
                    "valid": False, "source_sequence": sequence,
                    "capture_monotonic_ns": capture_monotonic_ns,
                    "reason": str(error),
                })
                label = f"INVALID: {error}"
                color = (20, 20, 240)

            cv2.putText(overlay, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, color, 2, cv2.LINE_AA)
            if args.save_overlay:
                temporary = exchange / f".latest_overlay.{os.getpid()}.png"
                if cv2.imwrite(str(temporary), overlay):
                    temporary.replace(exchange / "latest_overlay.png")
            if args.display:
                cv2.imshow("B2 D405 crack trajectory", overlay)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        trajectory_path.unlink(missing_ok=True)
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
