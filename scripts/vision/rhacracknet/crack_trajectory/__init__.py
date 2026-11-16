"""Crack-mask centerline extraction and camera projection utilities."""

from .centerline import CenterlineConfig, CenterlineResult, extract_centerline, draw_trajectory_overlay
from .geometry import (
    DepthProjectionConfig,
    PinholeCamera,
    Plane,
    build_metric_trajectory,
    load_d405_config,
    load_projection_config,
    project_pixels_to_plane,
    project_pixels_with_depth,
    sample_aligned_depth,
)

__all__ = [
    "CenterlineConfig",
    "CenterlineResult",
    "DepthProjectionConfig",
    "PinholeCamera",
    "Plane",
    "build_metric_trajectory",
    "draw_trajectory_overlay",
    "extract_centerline",
    "load_d405_config",
    "load_projection_config",
    "project_pixels_to_plane",
    "project_pixels_with_depth",
    "sample_aligned_depth",
]
