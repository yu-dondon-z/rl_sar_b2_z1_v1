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
    parser.add_argument("--input-width", type=int, default=640)
    parser.add_argument("--input-height", type=int, default=360)
    parser.add_argument("--spacing-px", type=float, default=8.0)
    parser.add_argument("--min-confidence", type=float, default=0.90)
    parser.add_argument("--min-image-path-length-px", type=float, default=160.0)
    parser.add_argument("--min-image-forward-span-px", type=float, default=120.0)
    parser.add_argument("--floor-roi-top", type=float, default=0.50)
    parser.add_argument("--bottom-entry-fraction", type=float, default=0.96)
    parser.add_argument(
        "--bottom-ignore-px",
        type=int,
        default=8,
        help=("ignore the lowest image rows, which the segmentation network "
              "can mistake for a crack at the camera-frame boundary"),
    )
    parser.add_argument("--max-lateral-to-forward-ratio", type=float, default=1.25)
    parser.add_argument(
        "--near-tail-vertical-tolerance-px",
        type=float,
        default=5.0,
        help=("trim a short horizontal branch at the image-bottom entry, such "
              "as a connected tile grout line"),
    )
    parser.add_argument(
        "--min-near-path-width-px", type=float, default=5.0,
        help=("minimum median mask width along the near half of the selected "
              "path; rejects thin tile-grout responses while retaining the "
              "wider black tape used for the real test"),
    )
    parser.add_argument(
        "--temporal-lock-max-error-px", type=float, default=35.0,
        help=("maximum median near-field pixel distance from the currently "
              "locked path; larger jumps are rejected instead of switching "
              "to another crack-like line"),
    )
    parser.add_argument(
        "--temporal-lock-reset-frames", type=int, default=30,
        help=("invalid frames required before releasing a lost path lock; a "
              "new path must still pass minimum-valid-streak before publish"),
    )
    parser.add_argument("--minimum-valid-streak", type=int, default=6)
    parser.add_argument(
        "--preview-only",
        action="store_true",
        help=("show RGB segmentation and the pixel centreline without loading "
              "camera calibration or publishing a metric trajectory"),
    )
    parser.add_argument("--display", action="store_true")
    parser.add_argument("--save-overlay", action="store_true")
    return parser.parse_args()


def write_json_atomic(path: Path, payload: dict):
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    temporary.replace(path)


def safe_metric_prefix(points):
    """Conservative near-field prefix, matching real_motion_transition.hpp.

    Truncate connected grout/wall branches rather than jumping over them.
    This is a geometry guard, not proof that the retained line is black tape.
    """
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("metric path must be Nx3")
    anchor, previous_heading = 0, None
    for i, point in enumerate(points):
        x, y, _ = point
        if (not np.all(np.isfinite(point)) or x < -0.20 or
                x > 2.0 or abs(y) > 0.70):
            return points[:i].copy(), "near-field envelope"
        if i and x < points[i - 1, 0] - 0.02:
            return points[:i].copy(), "backward branch"
        delta = point[:2] - points[anchor, :2]
        if np.linalg.norm(delta) < 0.20:
            continue
        heading = float(np.arctan2(delta[1], delta[0]))
        turn = (0.0 if previous_heading is None else
                float(np.arctan2(np.sin(heading - previous_heading),
                                 np.cos(heading - previous_heading))))
        if abs(heading) > np.pi / 4 or abs(turn) > np.pi / 6:
            return points[:anchor + 1].copy(), "sharp connected branch"
        anchor, previous_heading = i, heading
    return points.copy(), "none"


def draw_accepted_metric_path(image, points, camera, transform):
    """Only the accepted control prefix is yellow; no misleading far branch."""
    camera_from_base = np.linalg.inv(transform)
    rvec, _ = cv2.Rodrigues(camera_from_base[:3, :3])
    uv, _ = cv2.projectPoints(
        np.asarray(points, dtype=np.float64), rvec, camera_from_base[:3, 3],
        camera.matrix, np.asarray(camera.distortion, dtype=np.float64)
        if camera.distortion else None)
    overlay = image.copy()
    cv2.polylines(overlay, [np.rint(uv).astype(np.int32)], False,
                  (0, 230, 255), 3, cv2.LINE_AA)
    return overlay


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


def calibrated_ray_payload(pixels, camera, transform):
    """No ground intersection here: the controller owns timestamped pose."""
    transform = np.asarray(transform, dtype=np.float64)
    if (transform.shape != (4, 4) or not np.isfinite(transform).all() or
            not np.allclose(transform[3], [0, 0, 0, 1]) or
            not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-5) or
            not np.isclose(np.linalg.det(transform[:3, :3]), 1., atol=1e-5)):
        raise ValueError('invalid camera rigid transform')
    uv = np.asarray(pixels, dtype=np.float64)
    if uv.ndim != 2 or uv.shape[1] != 2 or not 3 <= len(uv) <= 4096 or not np.isfinite(uv).all():
        raise ValueError('invalid ordered pixel rays')
    directions = camera.pixels_to_rays(uv) @ transform[:3, :3].T
    norms = np.linalg.norm(directions, axis=1)
    if not np.isfinite(directions).all() or np.any(norms < 1e-9):
        raise ValueError('invalid calibrated rays')
    directions /= norms[:, None]
    return {'schema_version': 2, 'projection_method': 'calibrated_rays_pose_ground_v2',
            'timestamp_basis': 'host_receive_monotonic',
            'camera_origin_base_xyz': transform[:3, 3].tolist(),
            'ray_directions_base_xyz': directions.tolist()}


def trim_near_horizontal_tail(centerline, vertical_tolerance_px):
    """Drop the bottom-most horizontal prefix while keeping the forward path."""
    if vertical_tolerance_px <= 0.0:
        return centerline

    ordered = centerline.ordered_path_uv
    waypoints = centerline.waypoints_uv
    if len(ordered) < 3 or len(waypoints) < 3:
        return centerline

    # The anchor is selected from the bottom band, so the ordered path starts
    # close to the robot and travels upward in the image.  A tile grout line
    # connected to the cable can add a horizontal prefix at nearly constant v.
    bottom_v = float(np.max(ordered[:, 1]))
    ordered_bottom = np.flatnonzero(
        ordered[:, 1] >= bottom_v - vertical_tolerance_px)
    waypoint_bottom = np.flatnonzero(
        waypoints[:, 1] >= bottom_v - vertical_tolerance_px)
    if len(ordered_bottom) == 0 or len(waypoint_bottom) == 0:
        return centerline

    ordered_start = int(ordered_bottom[-1])
    waypoint_start = int(waypoint_bottom[-1])
    if ordered_start == 0 or waypoint_start == 0:
        return centerline
    if len(ordered) - ordered_start < 2 or len(waypoints) - waypoint_start < 2:
        return centerline

    centerline.ordered_path_uv = ordered[ordered_start:].copy()
    centerline.waypoints_uv = waypoints[waypoint_start:].copy()
    centerline.path_length_px = float(np.linalg.norm(
        np.diff(centerline.ordered_path_uv, axis=0), axis=1).sum())
    return centerline


def near_path_temporal_error(reference_uv, candidate_uv):
    """Return a robust near-field pixel distance between two path curves."""
    reference = np.asarray(reference_uv, dtype=np.float32)
    candidate = np.asarray(candidate_uv, dtype=np.float32)
    if len(reference) < 2 or len(candidate) < 2:
        return float("inf")

    # Compare the lower 80 percent of both paths.  The top 20 percent is
    # ignored because normal forward motion continually reveals new pixels,
    # but keeping the middle field in the comparison also catches a branch
    # switch at a tape/tile-grout intersection before it changes steering.
    reference = reference[
        reference[:, 1] >= np.percentile(reference[:, 1], 20.0)]
    candidate = candidate[
        candidate[:, 1] >= np.percentile(candidate[:, 1], 20.0)]
    if len(reference) < 2 or len(candidate) < 2:
        return float("inf")
    distances = np.linalg.norm(
        reference[:, None, :] - candidate[None, :, :], axis=2)
    forward = float(np.median(np.min(distances, axis=1)))
    reverse = float(np.median(np.min(distances, axis=0)))
    return max(forward, reverse)


def near_path_mask_width(component, pixels):
    """Estimate full mask width along the near half of a centerline."""
    points = np.asarray(pixels, dtype=np.float32)
    if len(points) < 2:
        return 0.0
    near = points[points[:, 1] >= np.percentile(points[:, 1], 50.0)]
    if len(near) == 0:
        return 0.0
    uv = np.rint(near).astype(np.int32)
    uv[:, 0] = np.clip(uv[:, 0], 0, component.shape[1] - 1)
    uv[:, 1] = np.clip(uv[:, 1], 0, component.shape[0] - 1)
    radius = cv2.distanceTransform(
        (component > 0).astype(np.uint8), cv2.DIST_L2, 5)
    # The distance transform is a radius measured from the path centre to the
    # mask edge; report a full width. Percentile 40 is robust to occasional
    # skeleton samples close to a branch boundary.
    return float(2.0 * np.percentile(radius[uv[:, 1], uv[:, 0]], 40.0))


def extract_floor_path(mask, config, extractor, min_path_length_px,
                       min_forward_span_px, bottom_entry_fraction,
                       max_lateral_to_forward_ratio,
                       near_tail_vertical_tolerance_px=5.0,
                       min_near_path_width_px=0.0,
                       reference_path_uv=None,
                       temporal_lock_max_error_px=float("inf")):
    """Select a near-field floor path without assuming it is image-centred."""
    binary = (mask > 0).astype(np.uint8) * 255
    kernel_size = config.morphology_kernel
    if kernel_size > 1:
        if kernel_size % 2 == 0:
            kernel_size += 1
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        binary, connectivity=8)
    height, width = binary.shape
    bottom_minimum = float(
        np.clip(bottom_entry_fraction, 0.85, 0.995) * height)
    candidates = []
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        component_height = int(stats[label, cv2.CC_STAT_HEIGHT])
        component_bottom = (
            int(stats[label, cv2.CC_STAT_TOP]) + component_height - 1)
        if area < config.min_component_area or component_bottom < bottom_minimum:
            continue

        component = (labels == label).astype(np.uint8) * 255
        v_coords, u_coords = np.where(component > 0)
        near_band = v_coords >= component_bottom - max(8, int(0.02 * height))
        if not np.any(near_band):
            continue
        near_u = u_coords[near_band]
        near_v = v_coords[near_band]
        nearest = int(np.argmin(np.abs(near_u - 0.5 * width)))
        anchor = (float(near_u[nearest]), float(near_v[nearest]))
        try:
            centerline = extractor(component, config, anchor)
        except RuntimeError:
            continue
        centerline = trim_near_horizontal_tail(
            centerline, near_tail_vertical_tolerance_px)

        pixels = centerline.waypoints_uv
        forward_span = float(np.max(pixels[:, 1]) - np.min(pixels[:, 1]))
        lateral_span = float(np.max(pixels[:, 0]) - np.min(pixels[:, 0]))
        if centerline.path_length_px < min_path_length_px:
            continue
        if forward_span < min_forward_span_px:
            continue
        if lateral_span > max_lateral_to_forward_ratio * forward_span:
            continue
        if float(np.max(pixels[:, 1])) < bottom_minimum:
            continue
        path_width = near_path_mask_width(component, pixels)
        if path_width < min_near_path_width_px:
            continue
        temporal_error = (0.0 if reference_path_uv is None else
                          near_path_temporal_error(reference_path_uv, pixels))
        if temporal_error > temporal_lock_max_error_px:
            continue
        # Width is deliberately first: the 25 mm black tape is substantially
        # wider than real tile grout. Forward span and length still reject
        # short dark blobs before this ranking is applied.
        score = (-temporal_error, path_width, forward_span,
                 centerline.path_length_px,
                 -abs(float(np.mean(pixels[:, 0])) - 0.5 * width))
        candidates.append(
            (score, centerline, forward_span, path_width, temporal_error))

    if not candidates:
        if reference_path_uv is not None:
            raise RuntimeError(
                "locked path continuity lost; refusing automatic line switch")
        raise RuntimeError(
            "no sufficiently wide valid path enters the near-field floor ROI")
    _, centerline, forward_span, path_width, temporal_error = max(
        candidates, key=lambda item: item[0])
    return centerline, forward_span, path_width, temporal_error


def main():
    args = parse_args()
    if args.minimum_valid_streak < 1:
        raise ValueError("minimum-valid-streak must be at least 1")
    if args.temporal_lock_reset_frames < 1:
        raise ValueError("temporal-lock-reset-frames must be at least 1")
    if args.temporal_lock_max_error_px <= 0.0:
        raise ValueError("temporal-lock-max-error-px must be positive")
    if args.min_near_path_width_px < 0.0:
        raise ValueError("min-near-path-width-px cannot be negative")
    if args.preview_only:
        calibration = {}
        intrinsics = transform = plane_normal = plane_offset = None
    else:
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
        CenterlineConfig, PinholeCamera,
        draw_trajectory_overlay, extract_centerline,
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
    input_size = None
    if args.input_width > 0 and args.input_height > 0:
        input_size = (args.input_height, args.input_width)
    segmenter = CrackSegmenter(args.checkpoint, "RHANet_dep", args.device,
                               args.threshold, input_size=input_size)
    camera = None
    if not args.preview_only:
        camera = PinholeCamera(*intrinsics,
            distortion=tuple(float(v) for v in calibration["intrinsics"].get("distortion", [])))
    centerline_config = CenterlineConfig(waypoint_spacing_px=args.spacing_px)
    warmed_up = False
    sequence = 0
    valid_streak = 0
    pending_path_pixels = None
    locked_path_pixels = None
    lock_lost_frames = 0
    print("[RealVision] B2 FrontVideoClient RGB stream started")
    try:
        while True:
            code, encoded = camera_client.GetImageSample()
            capture_monotonic_ns = time.monotonic_ns()
            if code != 0 or not encoded:
                trajectory_path.unlink(missing_ok=True)
                valid_streak = 0
                pending_path_pixels = None
                if locked_path_pixels is not None:
                    lock_lost_frames += 1
                    if lock_lost_frames >= args.temporal_lock_reset_frames:
                        locked_path_pixels = None
                        lock_lost_frames = 0
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
                height, width = image.shape[:2]
                roi = np.zeros((height, width), dtype=np.uint8)
                roi_top = int(np.clip(args.floor_roi_top, 0.35, 0.80) * height)
                floor_polygon = np.asarray([
                    [int(0.20 * width), height - 1],
                    [int(0.80 * width), height - 1],
                    [int(0.60 * width), roi_top],
                    [int(0.40 * width), roi_top],
                ], dtype=np.int32)
                cv2.fillPoly(roi, [floor_polygon], 255)
                floor_mask = cv2.bitwise_and(result.mask, roi)
                # RHACrackNet can produce a thin, full-width response exactly
                # on the lower image boundary.  When it touches the real path,
                # skeletonization follows that horizontal branch and rejects
                # an otherwise valid near-field trajectory.  Remove only this
                # non-physical boundary strip; the path must still enter the
                # configured bottom band below.
                bottom_ignore_px = int(np.clip(args.bottom_ignore_px, 0, height))
                if bottom_ignore_px > 0:
                    floor_mask[height - bottom_ignore_px:, :] = 0
                centerline, image_forward_span, path_width, temporal_error = extract_floor_path(
                    floor_mask,
                    centerline_config,
                    extract_centerline,
                    args.min_image_path_length_px,
                    args.min_image_forward_span_px,
                    args.bottom_entry_fraction,
                    args.max_lateral_to_forward_ratio,
                    args.near_tail_vertical_tolerance_px,
                    args.min_near_path_width_px,
                    locked_path_pixels,
                    args.temporal_lock_max_error_px,
                )
                pixels = centerline.waypoints_uv
                uv = np.rint(pixels).astype(int)
                uv[:, 0] = np.clip(uv[:, 0], 0, image.shape[1] - 1)
                uv[:, 1] = np.clip(uv[:, 1], 0, image.shape[0] - 1)
                confidence = float(np.mean(result.probability[uv[:, 1], uv[:, 0]]))
                if confidence < args.min_confidence:
                    raise RuntimeError(f"confidence too low: {confidence:.3f}")
                overlay = draw_trajectory_overlay(image, centerline)
                ray_payload = None
                guard_reason = "preview_only"
                guard_removed = 0
                if not args.preview_only:
                    ray_payload = calibrated_ray_payload(pixels, camera, transform)
                    guard_reason = 'awaiting_controller_pose_ground_validation'

                # Acquisition must contain the same physical line for several
                # frames. The old code incremented this counter even when it
                # alternated between black tape and tile grout.
                if locked_path_pixels is None:
                    pending_error = (float("inf") if pending_path_pixels is None
                                     else near_path_temporal_error(
                                         pending_path_pixels, pixels))
                    if (pending_path_pixels is None or
                            pending_error > args.temporal_lock_max_error_px):
                        pending_path_pixels = pixels.copy()
                        valid_streak = 1
                    else:
                        pending_path_pixels = pixels.copy()
                        valid_streak += 1
                    if valid_streak < args.minimum_valid_streak:
                        trajectory_path.unlink(missing_ok=True)
                        raise RuntimeError(
                            "waiting for same-path lock "
                            f"{valid_streak}/{args.minimum_valid_streak}")
                    locked_path_pixels = pixels.copy()
                    pending_path_pixels = None
                else:
                    locked_path_pixels = pixels.copy()
                    valid_streak = max(valid_streak, args.minimum_valid_streak)
                lock_lost_frames = 0

                lock_payload = {
                    "path_lock": "locked",
                    "path_width_px": path_width,
                    "temporal_error_px": temporal_error,
                    "path_lock_streak": valid_streak,
                    "metric_guard_reason": guard_reason,
                    "metric_guard_removed_waypoints": guard_removed,
                }
                if args.preview_only:
                    trajectory_path.unlink(missing_ok=True)
                    payload = {
                        "valid": True,
                        "preview_only": True,
                        "source_sequence": sequence,
                        "capture_monotonic_ns": capture_monotonic_ns,
                        "inference_ms": float(result.inference_ms),
                        "mean_waypoint_confidence": confidence,
                        "path_length_px": float(centerline.path_length_px),
                        "image_forward_span_px": image_forward_span,
                        "waypoint_count": int(len(centerline.waypoints_uv)),
                        **lock_payload,
                    }
                    write_json_atomic(status_path, payload)
                    label = (f"LOCKED PATH: {centerline.path_length_px:.0f}px, "
                             f"width={path_width:.1f}px")
                    color = (30, 220, 30)
                else:
                    payload = {
                        "source_sequence": sequence,
                        "capture_monotonic_ns": capture_monotonic_ns,
                        "frame_id": "b2_base",
                        "camera_name": calibration.get("camera_name", "b2_builtin_front"),
                        "inference_ms": float(result.inference_ms),
                        "mean_waypoint_confidence": confidence,
                        **ray_payload,
                        **lock_payload,
                    }
                    write_json_atomic(trajectory_path, payload)
                    write_json_atomic(status_path, {"valid": False, "rays_valid": True, **payload})
                    label = (f"RAYS LOCKED (2D): width={path_width:.1f}px; "
                             "controller 3D validation required")
                    color = (0, 210, 255)
            except (KeyError, RuntimeError, TypeError, ValueError) as error:
                waiting_for_lock = str(error).startswith(
                    "waiting for same-path lock")
                if locked_path_pixels is not None:
                    lock_lost_frames += 1
                    if lock_lost_frames >= args.temporal_lock_reset_frames:
                        locked_path_pixels = None
                        pending_path_pixels = None
                        valid_streak = 0
                        lock_lost_frames = 0
                elif not waiting_for_lock:
                    valid_streak = 0
                    pending_path_pixels = None
                trajectory_path.unlink(missing_ok=True)
                write_json_atomic(status_path, {
                    "valid": False, "source_sequence": sequence,
                    "capture_monotonic_ns": capture_monotonic_ns,
                    "reason": str(error),
                    "path_lock": ("locked" if locked_path_pixels is not None
                                  else "acquiring"),
                    "path_lock_streak": valid_streak,
                    "lock_lost_frames": lock_lost_frames,
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
