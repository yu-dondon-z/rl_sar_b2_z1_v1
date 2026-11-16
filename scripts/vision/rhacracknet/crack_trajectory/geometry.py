"""Project ordered image pixels into the robot frame.

The preferred path uses an RGB-aligned depth image from a RealSense D405.
Road-plane intersection is retained as a fallback for missing depth.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import cv2
import numpy as np
import yaml


@dataclass(frozen=True)
class PinholeCamera:
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: tuple[float, ...] = ()

    def __post_init__(self) -> None:
        if self.fx <= 0 or self.fy <= 0:
            raise ValueError("fx and fy must be positive")

    @property
    def matrix(self) -> np.ndarray:
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])

    def pixels_to_rays(self, pixels_uv: np.ndarray) -> np.ndarray:
        pixels = np.asarray(pixels_uv, dtype=np.float64).reshape(-1, 2)
        if self.distortion:
            normalized = cv2.undistortPoints(
                pixels.reshape(-1, 1, 2), self.matrix, np.asarray(self.distortion, dtype=np.float64)
            ).reshape(-1, 2)
        else:
            normalized = np.column_stack(((pixels[:, 0] - self.cx) / self.fx, (pixels[:, 1] - self.cy) / self.fy))
        return np.column_stack((normalized, np.ones(len(normalized), dtype=np.float64)))


@dataclass(frozen=True)
class Plane:
    normal: tuple[float, float, float]
    offset: float = 0.0

    def normalized(self) -> tuple[np.ndarray, float]:
        normal = np.asarray(self.normal, dtype=np.float64)
        norm = float(np.linalg.norm(normal))
        if norm <= 1.0e-12:
            raise ValueError("plane normal cannot be zero")
        return normal / norm, float(self.offset) / norm


@dataclass(frozen=True)
class DepthProjectionConfig:
    """Rules for robustly sampling an RGB-aligned depth image."""

    scale_m_per_unit: float = 0.001
    min_depth_m: float = 0.07
    max_depth_m: float = 3.0
    sample_radius_px: int = 2
    min_valid_samples: int = 3

    def __post_init__(self) -> None:
        if self.scale_m_per_unit <= 0:
            raise ValueError("depth scale must be positive")
        if self.min_depth_m < 0 or self.max_depth_m <= self.min_depth_m:
            raise ValueError("depth range must satisfy 0 <= min_depth_m < max_depth_m")
        if self.sample_radius_px < 0:
            raise ValueError("sample_radius_px cannot be negative")
        if self.min_valid_samples < 1:
            raise ValueError("min_valid_samples must be positive")


def _as_transform(value: Any) -> np.ndarray:
    transform = np.asarray(value, dtype=np.float64)
    if transform.shape != (4, 4):
        raise ValueError("T_robot_camera must be a 4x4 matrix")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1.0e-6):
        raise ValueError("T_robot_camera last row must be [0, 0, 0, 1]")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-4):
        raise ValueError("T_robot_camera rotation must be orthonormal")
    return transform


def load_projection_config(path: str | Path) -> tuple[PinholeCamera, np.ndarray, Plane]:
    data: Mapping[str, Any] = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    camera_data = data["camera"]
    camera = PinholeCamera(
        fx=float(camera_data["fx"]), fy=float(camera_data["fy"]),
        cx=float(camera_data["cx"]), cy=float(camera_data["cy"]),
        distortion=tuple(float(value) for value in camera_data.get("distortion", ())),
    )
    plane_data = data.get("plane_robot", {})
    plane = Plane(tuple(float(value) for value in plane_data.get("normal", (0, 0, 1))),
                  float(plane_data.get("offset", 0.0)))
    return camera, _as_transform(data["T_robot_camera"]), plane


def load_d405_config(
    path: str | Path,
    camera_name: str | None = None,
) -> tuple[str, PinholeCamera, np.ndarray, DepthProjectionConfig]:
    """Load one D405 entry from a multi-camera YAML configuration.

    The selected camera must contain intrinsics for the RGB stream to which
    the depth image was aligned. ``T_robot_camera`` maps that optical frame
    into the B2 base frame.
    """
    data: Mapping[str, Any] = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    cameras = data.get("cameras")
    if not isinstance(cameras, Mapping) or not cameras:
        raise ValueError("D405 configuration must contain a non-empty 'cameras' mapping")
    selected = camera_name or data.get("primary_camera")
    if not selected:
        raise ValueError("camera name is required when 'primary_camera' is not configured")
    if selected not in cameras:
        raise ValueError(f"camera '{selected}' not found; available: {', '.join(cameras)}")
    camera_data = cameras[selected]
    if not bool(camera_data.get("enabled", True)):
        raise ValueError(f"camera '{selected}' is disabled")
    intrinsics = camera_data.get("intrinsics", {})
    required = ("fx", "fy", "cx", "cy")
    missing = [key for key in required if intrinsics.get(key) is None]
    if missing:
        raise ValueError(
            f"camera '{selected}' is missing measured RGB intrinsics: {', '.join(missing)}"
        )
    camera = PinholeCamera(
        fx=float(intrinsics["fx"]),
        fy=float(intrinsics["fy"]),
        cx=float(intrinsics["cx"]),
        cy=float(intrinsics["cy"]),
        distortion=tuple(float(value) for value in intrinsics.get("distortion", ())),
    )
    depth_data = camera_data.get("depth", {})
    depth = DepthProjectionConfig(
        scale_m_per_unit=float(depth_data.get("scale_m_per_unit", 0.001)),
        min_depth_m=float(depth_data.get("min_m", 0.07)),
        max_depth_m=float(depth_data.get("max_m", 3.0)),
        sample_radius_px=int(depth_data.get("sample_radius_px", 2)),
        min_valid_samples=int(depth_data.get("min_valid_samples", 3)),
    )
    return str(selected), camera, _as_transform(camera_data["T_robot_camera"]), depth


def sample_aligned_depth(
    pixels_uv: np.ndarray,
    depth_image: np.ndarray,
    config: DepthProjectionConfig,
) -> tuple[np.ndarray, np.ndarray]:
    """Sample median metric depth around each pixel.

    ``depth_image`` must already be aligned to the RGB image used for crack
    segmentation. Integer images are interpreted using ``scale_m_per_unit``.
    Floating-point images are also scaled, allowing either millimetres
    (scale=0.001) or metres (scale=1.0) to be supplied explicitly.
    """
    depth = np.asarray(depth_image)
    if depth.ndim != 2:
        raise ValueError("aligned depth image must be a single-channel HxW array")
    pixels = np.asarray(pixels_uv, dtype=np.float64).reshape(-1, 2)
    sampled = np.full(len(pixels), np.nan, dtype=np.float64)
    valid = np.zeros(len(pixels), dtype=bool)
    height, width = depth.shape
    radius = config.sample_radius_px
    for index, (u, v) in enumerate(pixels):
        if not np.isfinite(u) or not np.isfinite(v):
            continue
        center_u, center_v = int(round(u)), int(round(v))
        if center_u < 0 or center_u >= width or center_v < 0 or center_v >= height:
            continue
        u0, u1 = max(0, center_u - radius), min(width, center_u + radius + 1)
        v0, v1 = max(0, center_v - radius), min(height, center_v + radius + 1)
        values_m = depth[v0:v1, u0:u1].astype(np.float64, copy=False).reshape(-1)
        values_m = values_m * config.scale_m_per_unit
        usable = values_m[
            np.isfinite(values_m)
            & (values_m >= config.min_depth_m)
            & (values_m <= config.max_depth_m)
        ]
        if len(usable) < config.min_valid_samples:
            continue
        sampled[index] = float(np.median(usable))
        valid[index] = True
    return sampled, valid


def project_pixels_with_depth(
    pixels_uv: np.ndarray,
    depth_image: np.ndarray,
    camera: PinholeCamera,
    T_robot_camera: np.ndarray,
    depth_config: DepthProjectionConfig | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Back-project RGB pixels with aligned depth into the B2 robot frame."""
    config = depth_config or DepthProjectionConfig()
    transform = _as_transform(T_robot_camera)
    pixels = np.asarray(pixels_uv, dtype=np.float64).reshape(-1, 2)
    depths_m, valid = sample_aligned_depth(pixels, depth_image, config)
    rays_camera = camera.pixels_to_rays(pixels)
    points_camera = rays_camera * depths_m[:, None]
    points_robot = (transform[:3, :3] @ points_camera.T).T + transform[:3, 3]
    valid &= np.all(np.isfinite(points_robot), axis=1)
    points_robot[~valid] = np.nan
    return points_robot, valid, depths_m


def build_metric_trajectory(
    points_robot: np.ndarray,
    valid: np.ndarray,
    spacing_m: float = 0.03,
    max_step_m: float = 0.25,
) -> np.ndarray:
    """Return the longest continuous valid 3-D path resampled in metres.

    Invalid depth and implausibly large adjacent jumps split the path. This
    avoids commanding a robot across an unobserved gap in the crack.
    """
    points = np.asarray(points_robot, dtype=np.float64).reshape(-1, 3)
    mask = np.asarray(valid, dtype=bool).reshape(-1)
    if len(points) != len(mask):
        raise ValueError("points_robot and valid must have the same length")
    if spacing_m <= 0 or max_step_m <= 0:
        raise ValueError("spacing_m and max_step_m must be positive")

    segments: list[np.ndarray] = []
    start = None
    for index in range(len(points)):
        usable = mask[index] and np.all(np.isfinite(points[index]))
        if usable and start is not None:
            usable = float(np.linalg.norm(points[index] - points[index - 1])) <= max_step_m
        if usable:
            if start is None:
                start = index
        elif start is not None:
            if index - start >= 2:
                segments.append(points[start:index])
            start = index if mask[index] and np.all(np.isfinite(points[index])) else None
    if start is not None and len(points) - start >= 2:
        segments.append(points[start:])
    if not segments:
        return np.empty((0, 3), dtype=np.float64)

    def arc_length(segment: np.ndarray) -> float:
        return float(np.linalg.norm(np.diff(segment, axis=0), axis=1).sum())

    path = max(segments, key=arc_length).copy()
    if len(path) >= 3:
        smoothed = path.copy()
        smoothed[1:-1] = (path[:-2] + 2.0 * path[1:-1] + path[2:]) / 4.0
        path = smoothed
    distances = np.linalg.norm(np.diff(path, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(distances)))
    keep = np.concatenate(([True], np.diff(cumulative) > 1.0e-8))
    path, cumulative = path[keep], cumulative[keep]
    if len(path) < 2 or cumulative[-1] <= 1.0e-8:
        return np.empty((0, 3), dtype=np.float64)
    samples = np.arange(0.0, cumulative[-1], spacing_m)
    if len(samples) == 0 or cumulative[-1] - samples[-1] > 1.0e-6:
        samples = np.append(samples, cumulative[-1])
    return np.column_stack([
        np.interp(samples, cumulative, path[:, axis]) for axis in range(3)
    ])


def project_pixels_to_plane(
    pixels_uv: np.ndarray,
    camera: PinholeCamera,
    T_robot_camera: np.ndarray,
    plane_robot: Plane,
) -> tuple[np.ndarray, np.ndarray]:
    """Return Nx3 robot-frame points and a valid-ray mask.

    Camera convention is the ROS optical frame: x right, y down, z forward.
    ``T_robot_camera`` maps camera-frame points into the robot frame.
    """
    transform = _as_transform(T_robot_camera)
    rays_camera = camera.pixels_to_rays(pixels_uv)
    directions_robot = (transform[:3, :3] @ rays_camera.T).T
    origin_robot = transform[:3, 3]
    normal, offset = plane_robot.normalized()
    denominator = directions_robot @ normal
    numerator = -(float(origin_robot @ normal) + offset)
    scale = np.full(len(directions_robot), np.nan, dtype=np.float64)
    non_parallel = np.abs(denominator) > 1.0e-9
    scale[non_parallel] = numerator / denominator[non_parallel]
    valid = non_parallel & np.isfinite(scale) & (scale > 0.0)
    points = origin_robot[None, :] + scale[:, None] * directions_robot
    points[~valid] = np.nan
    return points, valid
