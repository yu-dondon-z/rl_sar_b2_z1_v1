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
        "--dark-threshold",
        type=int,
        default=130,
        help="grayscale threshold for the synthetic physical-crack fallback",
    )
    parser.add_argument(
        "--ignore-bottom-rows",
        type=int,
        default=50,
        help="mask the bottom camera strip where the B2 body occludes the road",
    )
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
        help=(
            "intersect segmented camera rays with this known flat road "
            "height instead of sampling RGB-D"
        ),
    )
    # A few millimetres of display lift prevents z-fighting with the road.
    parser.add_argument("--path-height-offset", type=float, default=0.005)
    parser.add_argument(
        "--physical-crack-surface-z",
        type=float,
        default=None,
        help=(
            "known road-surface world z; when set, reject RGB-D points that "
            "are not physically below the surface"
        ),
    )
    parser.add_argument(
        "--physical-crack-min-depth",
        type=float,
        default=0.02,
        help="minimum accepted depth below --physical-crack-surface-z",
    )
    parser.add_argument(
        "--physical-crack-max-depth",
        type=float,
        default=0.08,
        help="maximum accepted depth below --physical-crack-surface-z",
    )
    parser.add_argument("--min-forward", type=float, default=0.25)
    parser.add_argument("--max-forward", type=float, default=5.0)
    parser.add_argument(
        "--max-lateral",
        type=float,
        default=3.0,
        help="maximum absolute lateral extent retained from the camera path",
    )
    parser.add_argument("--spacing-m", type=float, default=0.04)
    parser.add_argument("--minimum-path-length", type=float, default=0.45)
    parser.add_argument("--minimum-path-span", type=float, default=0.35)
    parser.add_argument(
        "--cache-hold-seconds",
        type=float,
        default=4.0,
        help="keep publishing a validated world path across short vision dropouts",
    )
    parser.add_argument(
        "--cache-behind",
        type=float,
        default=1.2,
        help="ordered path distance retained behind the base for a rear TCP",
    )
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
    trailer = text[3 + nq :]
    following = bool(int(trailer[0])) if len(trailer) >= 1 else False
    command = (
        np.asarray(trailer[1:4], dtype=np.float64)
        if len(trailer) >= 4
        else np.zeros(3, dtype=np.float64)
    )
    return sequence, sim_time, qpos, following, command


def write_text_atomic(path: Path, text: str) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def write_trajectory_atomic(
    path: Path,
    sequence: int,
    sim_time: float,
    source: str,
    points_world: np.ndarray,
) -> None:
    lines = [
        f"# sequence {sequence}",
        f"# sim_time {sim_time:.9f}",
        f"# source {source}",
    ]
    lines.extend(
        f"{point[0]:.7f},{point[1]:.7f},{point[2]:.7f}"
        for point in points_world
    )
    write_text_atomic(path, "\n".join(lines) + "\n")


def dark_line_mask(
    image_bgr: np.ndarray,
    threshold: int = 130,
    ignore_bottom_rows: int = 50,
) -> np.ndarray:
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    mask = np.where(gray < threshold, 255, 0).astype(np.uint8)
    # The built-in camera sees a horizontal strip of the B2 body at the
    # bottom of the image. On a bend that strip touches the physical crack,
    # turning both into one connected component and sending the skeletonizer
    # sideways. Remove only this self-occluded region; the crack immediately
    # above it remains the closest valid component to the adjusted anchor.
    ignored = min(max(int(ignore_bottom_rows), 0), mask.shape[0] - 1)
    if ignored > 0:
        mask[-ignored:, :] = 0
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


def project_pixels_to_horizontal_plane(
    pixels_uv: np.ndarray,
    camera,
    transform_world_optical: np.ndarray,
    plane_z: float,
    minimum_range: float,
    maximum_range: float,
):
    """Intersect optical rays with a known horizontal world plane.

    A 2 cm synthetic groove becomes sub-pixel in far-field depth. Sampling
    the MuJoCo depth buffer there therefore returns the adjacent road surface
    or no usable point at all. For a known flat road, ray-plane intersection
    is both less noisy and geometrically correct for the crack centreline.
    Real RGB-D operation keeps using ``project_pixels_with_depth`` when the
    plane override is not supplied.
    """
    pixels = np.asarray(pixels_uv, dtype=np.float64)
    points = np.full((len(pixels), 3), np.nan, dtype=np.float64)
    if len(pixels) == 0:
        return points, np.zeros(0, dtype=bool)

    rays_optical = np.column_stack((
        (pixels[:, 0] - camera.cx) / camera.fx,
        (pixels[:, 1] - camera.cy) / camera.fy,
        np.ones(len(pixels), dtype=np.float64),
    ))
    rotation = transform_world_optical[:3, :3]
    origin = transform_world_optical[:3, 3]
    rays_world = (rotation @ rays_optical.T).T
    denominator = rays_world[:, 2]
    valid = np.isfinite(rays_world).all(axis=1) & (
        np.abs(denominator) > 1.0e-8
    )
    scale = np.full(len(pixels), np.nan, dtype=np.float64)
    scale[valid] = (float(plane_z) - origin[2]) / denominator[valid]
    points[valid] = origin[None, :] + scale[valid, None] * rays_world[valid]
    ranges = np.linalg.norm(points - origin[None, :], axis=1)
    valid &= (
        (scale > 0.0)
        & np.isfinite(points).all(axis=1)
        & np.isfinite(ranges)
        & (ranges >= minimum_range)
        & (ranges <= maximum_range)
    )
    points[~valid] = np.nan
    return points, valid


def main() -> None:
    args = parse_args()
    if args.cache_behind < 1.0:
        raise ValueError(
            "--cache-behind must be at least 1.0 m for the rear-mounted TCP"
        )
    if args.physical_crack_min_depth < 0.0:
        raise ValueError("--physical-crack-min-depth must be non-negative")
    if not 0 <= args.dark_threshold <= 255:
        raise ValueError("--dark-threshold must be in [0, 255]")
    if not 0 <= args.ignore_bottom_rows < args.height:
        raise ValueError("--ignore-bottom-rows must be in [0, image height)")
    if args.physical_crack_max_depth < args.physical_crack_min_depth:
        raise ValueError(
            "--physical-crack-max-depth must be >= --physical-crack-min-depth"
        )
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
    last_valid_wall_time = -np.inf
    print(
        f"[Vision] scene={args.scene} camera={args.camera} "
        f"mode={args.segmentation_mode} exchange={exchange}"
    )
    try:
        while args.max_frames <= 0 or processed < args.max_frames:
            try:
                sequence, sim_time, qpos, following, command = read_state(
                    state_path, model.nq
                )
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
            base_position = np.asarray(data.xpos[base_id], dtype=np.float64)
            base_rotation = np.asarray(
                data.xmat[base_id], dtype=np.float64
            ).reshape(3, 3)
            forward = base_rotation[:, 0].copy()
            forward[2] = 0.0
            forward /= max(float(np.linalg.norm(forward)), 1.0e-9)
            renderer.disable_depth_rendering()
            renderer.update_scene(data, camera=args.camera)
            rgb = renderer.render().copy()
            image_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            if args.path_ground_z is None:
                # Real RGB-D deployment needs aligned depth. In the known-flat
                # MuJoCo test scene, camera rays are intersected directly with
                # the road plane, so a second full render is pure overhead.
                renderer.enable_depth_rendering()
                renderer.update_scene(data, camera=args.camera)
                depth = renderer.render().copy()
                renderer.disable_depth_rendering()
            else:
                depth = None

            network_result = (
                segmenter.predict_bgr(image_bgr) if segmenter is not None else None
            )
            masks = []
            dark_mask = None
            if args.segmentation_mode in ("hybrid", "dark"):
                dark_mask = dark_line_mask(
                    image_bgr,
                    args.dark_threshold,
                    args.ignore_bottom_rows,
                )
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
            candidate_errors = []
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
                        (
                            (args.width - 1.0) / 2.0,
                            float(args.height - args.ignore_bottom_rows - 1),
                        ),
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
                    if args.path_ground_z is None:
                        points_world, valid, _ = project_pixels_with_depth(
                            centerline.waypoints_uv,
                            depth,
                            camera,
                            transform,
                            depth_config,
                        )
                    else:
                        points_world, valid = project_pixels_to_horizontal_plane(
                            centerline.waypoints_uv,
                            camera,
                            transform,
                            args.path_ground_z,
                            depth_config.min_depth_m,
                            depth_config.max_depth_m,
                        )
                    if args.physical_crack_surface_z is not None:
                        # A dark appearance alone is not enough: shadows,
                        # chassis pixels and the road texture can all satisfy
                        # the hybrid mask.  In the geometric-trench scene the
                        # crack is the only feature whose aligned RGB-D point
                        # lies measurably below the road surface.  Rejecting
                        # surface-height points here prevents alternating
                        # ground/crack samples from becoming a false bend.
                        depth_below_surface = (
                            args.physical_crack_surface_z - points_world[:, 2]
                        )
                        valid = (
                            valid
                            & np.isfinite(depth_below_surface)
                            & (depth_below_surface >= args.physical_crack_min_depth)
                            & (depth_below_surface <= args.physical_crack_max_depth)
                        )
                    control = build_metric_trajectory(
                        points_world, valid, args.spacing_m, 0.35
                    )
                    if len(control) < 4:
                        raise RuntimeError("fewer than four valid 3-D points")

                    left = np.array([-forward[1], forward[0], 0.0])
                    relative = control - base_position
                    longitudinal = relative @ forward
                    lateral = relative @ left
                    usable = (
                        np.isfinite(control).all(axis=1)
                        & (longitudinal >= args.min_forward)
                        & (longitudinal <= args.max_forward)
                        & (np.abs(lateral) <= args.max_lateral)
                    )
                    control = control[usable]
                    longitudinal = longitudinal[usable]
                    if len(control) < 4:
                        raise RuntimeError("projected path is outside the forward workspace")
                    # ``extract_centerline`` already returns a connected path.
                    # Sorting every point by body-forward distance destroys
                    # that topology at a 90-degree bend, where many points can
                    # have the same longitudinal coordinate. Only correct a
                    # possible global reversal and preserve curve order.
                    if longitudinal[0] > longitudinal[-1]:
                        control = control[::-1].copy()
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
                    if (
                        length < args.minimum_path_length
                        or span < args.minimum_path_span
                    ):
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
                    candidate_errors.append(last_error)

            if accepted is None:
                if candidate_errors:
                    last_error = "; ".join(candidate_errors)
                cache_age = time.monotonic() - last_valid_wall_time
                cached_remaining = 0.0
                if len(cached_path_world) >= 4:
                    nearest_cache = int(np.argmin(np.linalg.norm(
                        cached_path_world[:, :2] - base_position[None, :2],
                        axis=1,
                    )))
                    if nearest_cache + 1 < len(cached_path_world):
                        cached_remaining = float(np.linalg.norm(
                            np.diff(cached_path_world[nearest_cache:, :2], axis=0),
                            axis=1,
                        ).sum())
                using_cache = (
                    len(cached_path_world) >= 4
                    and cache_age <= args.cache_hold_seconds
                    and cached_remaining >= max(0.80, args.minimum_path_length)
                )
                if using_cache:
                    # Refresh the file timestamp as well as its sequence. The
                    # controller can safely follow this already validated
                    # world-frame polyline while the next camera frame
                    # reacquires the crack.
                    write_trajectory_atomic(
                        trajectory_path,
                        sequence,
                        sim_time,
                        "validated cache",
                        cached_path_world,
                    )
                    write_text_atomic(
                        status_path,
                        f"CACHED sequence={sequence} sim_time={sim_time:.3f} "
                        f"following={int(following)} age={cache_age:.2f} "
                        f"remaining={cached_remaining:.2f} {last_error}\n",
                    )
                    overlay = image_bgr
                    status = (
                        f"CACHED PATH: age {cache_age:.1f} s, "
                        f"{cached_remaining:.1f} m remaining"
                    )
                else:
                    trajectory_path.unlink(missing_ok=True)
                    write_text_atomic(
                        status_path,
                        f"INVALID sequence={sequence} sim_time={sim_time:.3f} "
                        f"following={int(following)} cache_age={cache_age:.2f} "
                        f"cache_remaining={cached_remaining:.2f} {last_error}\n",
                    )
                    overlay = image_bgr
                    status = f"INVALID: {last_error}"
            else:
                (
                    source, mask, centerline, control, length, area_ratio,
                    base_position, forward,
                ) = accepted
                last_valid_wall_time = time.monotonic()
                if len(cached_path_world) == 0:
                    cached_path_world = control.copy()
                else:
                    # Extend the ordered history only beyond its existing
                    # tail. Re-sorting the accumulated cloud by the current
                    # body-forward axis works for a straight line, but folds a
                    # 90-degree route back on itself and corrupts arc length.
                    tail_distance = np.linalg.norm(
                        control[:, :2] - cached_path_world[-1, :2], axis=1
                    )
                    tail_match = int(np.argmin(tail_distance))
                    if float(tail_distance[tail_match]) <= 0.20:
                        extension = control[tail_match + 1 :]
                        for point in extension:
                            if np.linalg.norm(
                                point[:2] - cached_path_world[-1, :2]
                            ) >= 0.025:
                                cached_path_world = np.vstack(
                                    (cached_path_world, point[None, :])
                                )
                    else:
                        # If the camera skipped enough frames that the two
                        # polylines do not overlap, append only across a small
                        # plausible gap. Larger jumps indicate a false mask;
                        # retaining the last coherent cache is safer.
                        gap = float(np.linalg.norm(
                            control[0, :2] - cached_path_world[-1, :2]
                        ))
                        if gap <= 0.35:
                            cached_path_world = np.vstack(
                                (cached_path_world, control)
                            )

                # Discard old history by path order, not by longitudinal
                # projection. The latter removes the incoming straight while
                # the base rotates through the bend. Keep about 2 m behind the
                # point nearest the base so the rear-mounted TCP still has a
                # continuous target.  The rear TCP samples around -0.78 m, so
                # 1.2 m is sufficient without retaining a long history.
                base_distance = np.linalg.norm(
                    cached_path_world[:, :2] - base_position[None, :2], axis=1
                )
                nearest_cache = int(np.argmin(base_distance))
                cache_start = nearest_cache
                retained_behind = 0.0
                while cache_start > 0 and retained_behind < args.cache_behind:
                    retained_behind += float(np.linalg.norm(
                        cached_path_world[cache_start, :2]
                        - cached_path_world[cache_start - 1, :2]
                    ))
                    cache_start -= 1
                cached_path_world = cached_path_world[cache_start:]
                cache_longitudinal = (
                    cached_path_world - base_position[None, :]
                ) @ forward
                sorted_longitudinal = cache_longitudinal
                rear_points = int(np.count_nonzero(
                    (sorted_longitudinal >= -1.0)
                    & (sorted_longitudinal <= -0.55)
                ))
                write_trajectory_atomic(
                    trajectory_path,
                    sequence,
                    sim_time,
                    source,
                    cached_path_world,
                )
                write_text_atomic(
                    status_path,
                    (
                        f"VALID sequence={sequence} sim_time={sim_time:.3f} "
                        f"following={int(following)} "
                        f"command={command[0]:.3f},{command[1]:.3f},{command[2]:.3f} "
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

            mode_text = "FOLLOWING" if following else "IDLE - click MuJoCo and press T"
            status = f"{mode_text} | {status}"
            color = (
                (40, 220, 40)
                if accepted is not None
                else ((0, 180, 255) if using_cache else (20, 20, 240))
            )
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
                          else dark_line_mask(
                              image_bgr,
                              args.dark_threshold,
                              args.ignore_bottom_rows,
                          ))
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
