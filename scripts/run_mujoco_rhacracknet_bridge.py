#!/usr/bin/env python3
"""Render the B2 built-in MuJoCo RGB-D camera and publish live crack paths.

The MuJoCo controller exports qpos snapshots. This process mirrors those
snapshots into a second renderer, runs RHACrackNet, projects the ordered
centerline with aligned depth, and atomically publishes world-frame XYZ points.
It intentionally remains outside the 50 Hz policy process.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
import time

import cv2
import mujoco
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exchange-dir", default="/tmp/b2_z1_mujoco_vision")
    parser.add_argument("--scene", required=True)
    parser.add_argument("--camera", default="front_depth_camera")
    parser.add_argument("--rhacracknet-root", default="/home/dong/RHACrackNet/code")
    parser.add_argument(
        "--checkpoint",
        default=(
            "/home/dong/RHACrackNet/code/exps/"
            "rhanet_dep_CamCrack789_b2_stage1/ckpt/best_ckpt.pth"
        ),
    )
    parser.add_argument("--model", choices=("RHANet", "RHANet_dep"), default="RHANet_dep")
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--threshold", type=float, default=0.55)
    parser.add_argument(
        "--segmentation-mode",
        choices=("network", "hybrid", "dark"),
        default="hybrid",
        help="hybrid uses RHACrackNet first and a dark-line fallback for synthetic-domain gaps",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    # Keep the detected crack centreline on the road.  This is deliberately
    # independent of the TCP working height: the controller derives the TCP
    # target in the base frame, while these points describe road geometry.
    # Leave this unset for the physical height-field scene so RGB-D preserves
    # the real groove depth.  The override remains useful for flat-road scenes.
    parser.add_argument(
        "--path-ground-z",
        type=float,
        default=None,
        help="optionally override RGB-D z with a known flat road height",
    )
    # A few millimetres of display lift prevents z-fighting with the road.
    parser.add_argument("--path-height-offset", type=float, default=0.005)
    parser.add_argument("--min-forward", type=float, default=0.25)
    parser.add_argument("--max-forward", type=float, default=5.0)
    parser.add_argument("--spacing-m", type=float, default=0.04)
    parser.add_argument("--display", action="store_true")
    parser.add_argument("--save-overlay", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--poll-period", type=float, default=0.01)
    return parser.parse_args()


def read_state(path: Path, expected_nq: int):
    text = path.read_text(encoding="utf-8").split()
    if len(text) < 3:
        raise ValueError("incomplete state header")
    sequence = int(text[0])
    sim_time = float(text[1])
    nq = int(text[2])
    if nq != expected_nq or len(text) < 3 + nq:
        raise ValueError(f"qpos size mismatch: file={nq}, model={expected_nq}")
    qpos = np.asarray(text[3 : 3 + nq], dtype=np.float64)
    if not np.all(np.isfinite(qpos)):
        raise ValueError("state contains non-finite qpos")
    return sequence, sim_time, qpos


def write_text_atomic(path: Path, text: str) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def dark_line_mask(image_bgr: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    mask = np.where(gray < 55, 255, 0).astype(np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def network_guided_dark_mask(
    network_mask: np.ndarray, dark_mask: np.ndarray
) -> np.ndarray:
    """Keep dark crack cores that are spatially supported by RHACrackNet.

    RHACrackNet was trained on real road imagery and can mark high-contrast
    MuJoCo tile seams.  Pure intensity segmentation is clean in the synthetic
    scene but is less reliable on real concrete.  Their intersection gives
    the simulator a clean centreline while retaining semantic network support.
    Dilation tolerates a few pixels of boundary disagreement.
    """
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    network_support = cv2.dilate(network_mask, kernel, iterations=1)
    fused = cv2.bitwise_and(dark_mask, network_support)
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    return cv2.morphologyEx(fused, cv2.MORPH_CLOSE, close_kernel)


def main() -> None:
    args = parse_args()
    code_root = Path(args.rhacracknet_root).expanduser().resolve()
    if str(code_root) not in sys.path:
        sys.path.insert(0, str(code_root))

    from crack_inference import CrackSegmenter
    from crack_trajectory import (
        CenterlineConfig,
        DepthProjectionConfig,
        PinholeCamera,
        build_metric_trajectory,
        draw_trajectory_overlay,
        extract_centerline,
        project_pixels_with_depth,
    )

    exchange = Path(args.exchange_dir).expanduser().resolve()
    exchange.mkdir(parents=True, exist_ok=True)
    state_path = exchange / "latest_state.txt"
    trajectory_path = exchange / "latest_trajectory.csv"
    status_path = exchange / "latest_vision_status.txt"
    trajectory_path.unlink(missing_ok=True)

    model = mujoco.MjModel.from_xml_path(str(Path(args.scene).expanduser().resolve()))
    data = mujoco.MjData(model)
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, args.camera)
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    if camera_id < 0:
        raise RuntimeError(f"MuJoCo camera not found: {args.camera}")
    if base_id < 0:
        raise RuntimeError("MuJoCo body not found: base_link")

    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    segmenter = None
    if args.segmentation_mode != "dark":
        segmenter = CrackSegmenter(
            args.checkpoint, args.model, args.device, args.threshold
        )
        segmenter.warmup(args.height, args.width)

    fovy = np.deg2rad(float(model.cam_fovy[camera_id]))
    fy = 0.5 * args.height / np.tan(0.5 * fovy)
    camera = PinholeCamera(
        fx=float(fy),
        fy=float(fy),
        cx=(args.width - 1.0) / 2.0,
        cy=(args.height - 1.0) / 2.0,
    )
    depth_config = DepthProjectionConfig(
        scale_m_per_unit=1.0,
        min_depth_m=0.10,
        max_depth_m=8.0,
        sample_radius_px=2,
        min_valid_samples=3,
    )
    centerline_config = CenterlineConfig(
        min_component_area=45,
        morphology_kernel=3,
        smooth_window=9,
        waypoint_spacing_px=7.0,
    )

    last_sequence = -1
    processed = 0
    cached_path_world = np.empty((0, 3), dtype=np.float64)
    print(
        f"[Vision] scene={args.scene} camera={args.camera} "
        f"mode={args.segmentation_mode} exchange={exchange}"
    )
    try:
        while args.max_frames <= 0 or processed < args.max_frames:
            try:
                sequence, sim_time, qpos = read_state(state_path, model.nq)
            except (FileNotFoundError, OSError, TypeError, ValueError):
                time.sleep(args.poll_period)
                continue
            if sequence == last_sequence:
                time.sleep(args.poll_period)
                continue
            if sequence < last_sequence:
                cached_path_world = np.empty((0, 3), dtype=np.float64)
            last_sequence = sequence
            processed += 1

            data.qpos[:] = qpos
            data.qvel[:] = 0.0
            mujoco.mj_forward(model, data)
            renderer.disable_depth_rendering()
            renderer.update_scene(data, camera=args.camera)
            rgb = renderer.render().copy()
            image_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            renderer.enable_depth_rendering()
            renderer.update_scene(data, camera=args.camera)
            depth = renderer.render().copy()
            renderer.disable_depth_rendering()

            network_result = (
                segmenter.predict_bgr(image_bgr) if segmenter is not None else None
            )
            masks = []
            dark_mask = None
            if args.segmentation_mode in ("hybrid", "dark"):
                dark_mask = dark_line_mask(image_bgr)
            if args.segmentation_mode == "hybrid" and network_result is not None:
                # Prefer the semantic/geometric agreement.  Fall back in both
                # directions so a domain-gap network miss or poor illumination
                # does not silently remove an otherwise valid path.
                masks.append((
                    "RHACrackNet+dark",
                    network_guided_dark_mask(network_result.mask, dark_mask),
                ))
                masks.append(("dark fallback", dark_mask))
                masks.append(("RHACrackNet fallback", network_result.mask))
            elif args.segmentation_mode == "network" and network_result is not None:
                masks.append(("RHACrackNet", network_result.mask))
            elif dark_mask is not None:
                masks.append(("dark fallback", dark_mask))

            accepted = None
            last_error = "no segmentation candidate"
            for source, mask in masks:
                try:
                    area_ratio = float(np.mean(mask > 0))
                    if not 0.00015 <= area_ratio <= 0.12:
                        raise RuntimeError(
                            f"mask area {area_ratio:.4f} outside 0.00015..0.12"
                        )
                    centerline = extract_centerline(
                        mask,
                        centerline_config,
                        ((args.width - 1.0) / 2.0, float(args.height - 1)),
                    )

                    camera_position = np.asarray(
                        data.cam_xpos[camera_id], dtype=np.float64
                    )
                    rotation_world_mujoco = np.asarray(
                        data.cam_xmat[camera_id], dtype=np.float64
                    ).reshape(3, 3)
                    # MuJoCo camera coordinates are x-right, y-up, -z-forward.
                    # RHACrackNet projection utilities use ROS optical:
                    # x-right, y-down, z-forward.
                    rotation_world_optical = rotation_world_mujoco @ np.diag(
                        [1.0, -1.0, -1.0]
                    )
                    transform = np.eye(4, dtype=np.float64)
                    transform[:3, :3] = rotation_world_optical
                    transform[:3, 3] = camera_position
                    points_world, valid, _ = project_pixels_with_depth(
                        centerline.waypoints_uv,
                        depth,
                        camera,
                        transform,
                        depth_config,
                    )
                    control = build_metric_trajectory(
                        points_world, valid, args.spacing_m, 0.35
                    )
                    if len(control) < 4:
                        raise RuntimeError("fewer than four valid 3-D points")

                    base_position = np.asarray(data.xpos[base_id], dtype=np.float64)
                    base_rotation = np.asarray(
                        data.xmat[base_id], dtype=np.float64
                    ).reshape(3, 3)
                    forward = base_rotation[:, 0].copy()
                    forward[2] = 0.0
                    forward /= max(float(np.linalg.norm(forward)), 1.0e-9)
                    left = np.array([-forward[1], forward[0], 0.0])
                    relative = control - base_position
                    longitudinal = relative @ forward
                    lateral = relative @ left
                    usable = (
                        np.isfinite(control).all(axis=1)
                        & (longitudinal >= args.min_forward)
                        & (longitudinal <= args.max_forward)
                        & (np.abs(lateral) <= 1.25)
                    )
                    control = control[usable]
                    longitudinal = longitudinal[usable]
                    if len(control) < 4:
                        raise RuntimeError("projected path is outside the forward workspace")
                    order = np.argsort(longitudinal)
                    control = control[order]
                    if args.path_ground_z is None:
                        # A physical groove must retain the depth measured by
                        # RGB-D; otherwise a 3-D crack is silently flattened.
                        control[:, 2] += args.path_height_offset
                    else:
                        control[:, 2] = (
                            args.path_ground_z + args.path_height_offset
                        )
                    length = float(
                        np.linalg.norm(np.diff(control[:, :2], axis=0), axis=1).sum()
                    )
                    span = float(
                        (control[-1, :2] - control[0, :2]) @ forward[:2]
                    )
                    if length < 0.45 or span < 0.35:
                        raise RuntimeError(
                            f"path too short: length={length:.2f}, span={span:.2f}"
                        )
                    accepted = (
                        source, mask, centerline, control, length, area_ratio,
                        base_position, forward,
                    )
                    break
                except (RuntimeError, TypeError, ValueError) as error:
                    last_error = f"{source}: {error}"

            if accepted is None:
                trajectory_path.unlink(missing_ok=True)
                write_text_atomic(
                    status_path,
                    f"INVALID sequence={sequence} sim_time={sim_time:.3f} {last_error}\n",
                )
                overlay = image_bgr
                status = f"INVALID: {last_error}"
            else:
                (
                    source, mask, centerline, control, length, area_ratio,
                    base_position, forward,
                ) = accepted
                if len(cached_path_world) == 0:
                    cached_path_world = control.copy()
                else:
                    accepted_points = []
                    for point in control:
                        distance = np.linalg.norm(
                            cached_path_world[:, :2] - point[None, :2], axis=1
                        )
                        if float(np.min(distance)) >= 0.03:
                            accepted_points.append(point)
                    if accepted_points:
                        cached_path_world = np.vstack(
                            (cached_path_world, np.asarray(accepted_points))
                        )

                cache_longitudinal = (
                    cached_path_world - base_position[None, :]
                ) @ forward
                keep = (
                    (cache_longitudinal >= -2.0)
                    & (cache_longitudinal <= args.max_forward + 0.5)
                )
                cached_path_world = cached_path_world[keep]
                cache_longitudinal = cache_longitudinal[keep]
                order = np.argsort(cache_longitudinal)
                cached_path_world = cached_path_world[order]
                sorted_longitudinal = cache_longitudinal[order]
                rear_points = int(np.count_nonzero(
                    (sorted_longitudinal >= -1.0)
                    & (sorted_longitudinal <= -0.55)
                ))
                lines = [
                    f"# sequence {sequence}",
                    f"# sim_time {sim_time:.9f}",
                    f"# source {source}",
                ]
                lines.extend(
                    f"{point[0]:.7f},{point[1]:.7f},{point[2]:.7f}"
                    for point in cached_path_world
                )
                write_text_atomic(trajectory_path, "\n".join(lines) + "\n")
                write_text_atomic(
                    status_path,
                    (
                        f"VALID sequence={sequence} sim_time={sim_time:.3f} "
                        f"source={source} current_points={len(control)} "
                        f"cached_points={len(cached_path_world)} "
                        f"rear_points={rear_points} "
                        f"length={length:.3f} area={area_ratio:.5f}\n"
                    ),
                )
                overlay = draw_trajectory_overlay(image_bgr, centerline)
                status = (
                    f"VALID {source}: {len(control)}/{len(cached_path_world)} points, "
                    f"{length:.2f} m, {network_result.inference_ms:.1f} ms"
                    if network_result is not None
                    else f"VALID {source}: {len(control)} points, {length:.2f} m"
                )

            color = (40, 220, 40) if accepted is not None else (20, 20, 240)
            cv2.putText(
                overlay, status, (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, color, 2, cv2.LINE_AA,
            )
            if args.save_overlay:
                temporary = exchange / f".latest_overlay.{os.getpid()}.tmp.png"
                if cv2.imwrite(str(temporary), overlay):
                    temporary.replace(exchange / "latest_overlay.png")
            if args.display:
                mask_panel = (
                    accepted[1] if accepted is not None
                    else (network_result.mask if network_result is not None
                          else dark_line_mask(image_bgr))
                )
                mask_bgr = cv2.cvtColor(mask_panel, cv2.COLOR_GRAY2BGR)
                panel = np.hstack((image_bgr, mask_bgr, overlay))
                panel = cv2.resize(panel, (1440, 360), interpolation=cv2.INTER_AREA)
                cv2.imshow("B2 MuJoCo vision: RGB | mask | centerline", panel)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        trajectory_path.unlink(missing_ok=True)
        renderer.close()
        if args.display:
            cv2.destroyAllWindows()
        print("[Vision] bridge stopped")


if __name__ == "__main__":
    main()
