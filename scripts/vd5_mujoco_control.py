"""VD5 pose utilities plus the retired ideal-geometry command reference.

The current MuJoCo runner uses vd5_visual_control and rendered RGB/RGB-D;
front_camera_command and seam_target_pose remain only for comparison tests.
"""

from __future__ import annotations

import math

import numpy as np


def startup_fraction(time_s: float, duration_s: float = 2.0) -> float:
    if not math.isfinite(time_s) or not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("startup time and duration must be finite; duration must be positive")
    t = float(np.clip(time_s / duration_s, 0.0, 1.0))
    return t**3 * (10.0 + t * (-15.0 + 6.0 * t))


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    norm = np.linalg.norm(q)
    if q.shape != (4,) or not np.isfinite(q).all() or norm < 1e-12:
        raise ValueError("quaternion must be finite, nonzero wxyz")
    return q / norm


def quaternion_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = normalize_quaternion(a)
    bw, bx, by, bz = normalize_quaternion(b)
    return normalize_quaternion(np.array((
        aw*bw-ax*bx-ay*by-az*bz,
        aw*bx+ax*bw+ay*bz-az*by,
        aw*by-ax*bz+ay*bw+az*bx,
        aw*bz+ax*by-ay*bx+az*bw,
    )))


def quaternion_conjugate(q: np.ndarray) -> np.ndarray:
    q = normalize_quaternion(q)
    return q * np.array((1.0, -1.0, -1.0, -1.0))


def quaternion_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = normalize_quaternion(q)
    return np.array((
        (1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
        (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
        (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)),
    ))


def quaternion_from_euler_xyz(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return normalize_quaternion(np.array((
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    )))


def normalized_lerp_quaternion(first: np.ndarray, second: np.ndarray, fraction: float) -> np.ndarray:
    first, second = normalize_quaternion(first), normalize_quaternion(second)
    if float(np.dot(first, second)) < 0.0:
        second = -second
    return normalize_quaternion(first + float(fraction) * (second - first))


def world_position_to_base(position_w: np.ndarray, base_pos_w: np.ndarray,
                           base_quat_w: np.ndarray) -> np.ndarray:
    return quaternion_matrix(base_quat_w).T @ (np.asarray(position_w) - np.asarray(base_pos_w))


def sample_polyline_behind_position(points_w: np.ndarray, position_w: np.ndarray,
                                    trailing_distance: float) -> np.ndarray:
    """Select an arc-length target behind the closest point, extending at the start."""
    points = np.asarray(points_w, dtype=np.float64)
    position = np.asarray(position_w, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or position.shape != (3,):
        raise ValueError("expected seam (N,3), position (3,)")
    if not np.isfinite(points).all() or not np.isfinite(position).all() or trailing_distance < 0:
        raise ValueError("seam inputs must be finite and trailing distance nonnegative")
    segments = points[1:] - points[:-1]
    lengths = np.linalg.norm(segments, axis=1)
    if np.any(lengths < 1e-9):
        raise ValueError("seam polyline contains a degenerate segment")
    fractions = np.clip(np.sum((position - points[:-1]) * segments, axis=1) / lengths**2, 0.0, 1.0)
    projections = points[:-1] + fractions[:, None] * segments
    nearest = int(np.argmin(np.linalg.norm(projections - position, axis=1)))
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    target_s = cumulative[nearest] + fractions[nearest] * lengths[nearest] - trailing_distance
    if target_s < 0.0:
        return points[0] + target_s * segments[0] / lengths[0]
    if target_s >= cumulative[-1]:
        return points[-1].copy()
    index = int(np.searchsorted(cumulative[1:], target_s, side="right"))
    fraction = (target_s - cumulative[index]) / lengths[index]
    return points[index] + fraction * segments[index]


def _camera_mount_matrix() -> np.ndarray:
    pitch = math.radians(45.0)
    c, s = math.cos(pitch), math.sin(pitch)
    return np.array(((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c)))


def _tracking_errors(points_h: np.ndarray, visible: np.ndarray, lookahead: float):
    starts = points_h[:-1, :2]
    segments = points_h[1:, :2] - starts
    length2 = np.sum(segments * segments, axis=1)
    segment_valid = visible[:-1] & visible[1:] & (length2 > 1e-8)
    if not segment_valid.any():
        return 0.0, 0.0, 0.0, False
    projection = -np.sum(starts * segments, axis=1) / np.maximum(length2, 1e-8)
    closest = starts + np.clip(projection, 0.0, 1.0)[:, None] * segments
    distance2 = np.sum(closest * closest, axis=1)
    nearest = int(np.argmin(np.where(segment_valid, distance2, np.inf)))
    tangent = segments[nearest] / max(math.sqrt(length2[nearest]), 1e-4)
    point = starts[nearest]
    lateral = float(point[0]*tangent[1] - point[1]*tangent[0])
    heading = math.atan2(float(tangent[1]), float(tangent[0])) - math.atan2(lateral, lookahead)
    heading = math.atan2(math.sin(heading), math.cos(heading))

    previous, following = segments[:-1], segments[1:]
    cross = previous[:, 0]*following[:, 1] - previous[:, 1]*following[:, 0]
    denominator = (np.linalg.norm(previous, axis=1) * np.linalg.norm(following, axis=1)
                   * np.linalg.norm(previous + following, axis=1))
    vertex_valid = segment_valid[:-1] & segment_valid[1:]
    vertex_curvature = np.where(vertex_valid, 2.0*cross/np.maximum(denominator, 1e-8), 0.0)
    if len(vertex_curvature):
        padded_curvature = np.concatenate((vertex_curvature[:1], vertex_curvature, vertex_curvature[-1:]))
        padded_valid = np.concatenate((vertex_valid[:1], vertex_valid, vertex_valid[-1:]))
        weights = int(padded_valid[nearest]) + int(padded_valid[nearest+1])
        curvature = ((padded_curvature[nearest] + padded_curvature[nearest+1]) / max(weights, 1))
    else:
        curvature = 0.0
    return lateral, heading, float(curvature), True


def front_camera_command(base_pos_w: np.ndarray, base_quat_w: np.ndarray, seam_points_w: np.ndarray,
                         *, speed: float = 0.2, camera_height_offset: float = 0.04765,
                         lookahead: float = 0.6, heading_gain: float = 1.5):
    """Reference-only ideal pinhole steering helper; not called by the live runner."""
    base_pos = np.asarray(base_pos_w, dtype=np.float64)
    base_rotation = quaternion_matrix(base_quat_w)
    seam = np.asarray(seam_points_w, dtype=np.float64)
    if base_pos.shape != (3,) or seam.ndim != 2 or seam.shape[1] != 3:
        raise ValueError("invalid base pose or seam shape")
    camera_offset_b = np.array((0.41251, 0.024997, camera_height_offset))
    camera_pos = base_pos + base_rotation @ camera_offset_b
    camera_rotation = base_rotation @ _camera_mount_matrix()
    points_c = (seam - camera_pos) @ camera_rotation
    depth = points_c[:, 0]
    fx = fy = 320.0
    cx, cy = 320.0, 240.0
    safe_depth = np.maximum(depth, 0.05)
    pixels = np.column_stack((cx - fx*points_c[:, 1]/safe_depth,
                              cy - fy*points_c[:, 2]/safe_depth))
    rays = np.column_stack((np.ones(len(depth)), -(pixels[:, 0]-cx)/fx,
                            -(pixels[:, 1]-cy)/fy))
    ground_normal_c = camera_rotation.T @ np.array((0.0, 0.0, 1.0))
    denominator = rays @ ground_normal_c
    valid_ray = denominator < -1e-6
    safe_denominator = np.where(valid_ray, denominator, -1.0)
    distance = -camera_pos[2] / safe_denominator
    visible = ((depth >= 0.05) & (depth <= 5.0)
               & (pixels[:, 0] >= 0.0) & (pixels[:, 0] < 640.0)
               & (pixels[:, 1] >= 0.0) & (pixels[:, 1] < 480.0)
               & valid_ray & (distance > 0.0) & np.isfinite(points_c).all(axis=1))
    points_c_recovered = np.where(visible[:, None], rays * distance[:, None], 0.0)
    perceived_w = camera_pos + points_c_recovered @ camera_rotation.T

    yaw = math.atan2(base_rotation[1, 0], base_rotation[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    rotation_world_to_horizontal = np.array(((c, -s), (s, c)))
    points_h = (perceived_w[:, :2] - base_pos[:2]) @ rotation_world_to_horizontal
    lateral, heading, curvature, path_visible = _tracking_errors(points_h, visible, lookahead)
    yaw_rate = float(np.clip(heading_gain*heading + speed*curvature, -0.8, 0.8)) if path_visible else 0.0
    return np.array((speed, 0.0, yaw_rate), dtype=np.float64), {
        "camera_visible_points": int(visible.sum()),
        "path_visible": bool(path_visible),
        "lateral_error_m": lateral,
        "heading_error_rad": heading,
        "curvature_m_inv": curvature,
        "yaw_rate_rps": yaw_rate,
    }


def seam_target_pose(base_pos_w: np.ndarray, base_quat_w: np.ndarray, tcp_start_pos_w: np.ndarray,
                     tcp_start_quat_w: np.ndarray, seam_points_w: np.ndarray, time_s: float,
                     *, trailing_distance: float = 0.70, startup_duration_s: float = 2.0):
    """Reference-only world-seam target helper; not called by the live runner."""
    base_pos = np.asarray(base_pos_w, dtype=np.float64)
    base_quat = normalize_quaternion(base_quat_w)
    fraction = startup_fraction(time_s, startup_duration_s)
    seam_w = sample_polyline_behind_position(seam_points_w, base_pos, trailing_distance)
    target_pos_w = np.asarray(tcp_start_pos_w) + fraction*(seam_w - np.asarray(tcp_start_pos_w))
    target_pos_b = world_position_to_base(target_pos_w, base_pos, base_quat)

    final_quat_b = quaternion_from_euler_xyz(0.0, math.pi/2.0-0.45, math.pi)
    start_quat_b = quaternion_multiply(quaternion_conjugate(base_quat), tcp_start_quat_w)
    target_quat_b = normalized_lerp_quaternion(start_quat_b, final_quat_b, fraction)
    return np.concatenate((target_pos_b, target_quat_b)), target_pos_w, fraction
