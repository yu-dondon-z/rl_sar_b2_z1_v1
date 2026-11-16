"""Convert a binary crack mask into one ordered, resampled centerline path."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class CenterlineConfig:
    min_component_area: int = 80
    morphology_kernel: int = 3
    smooth_window: int = 7
    waypoint_spacing_px: float = 8.0
    max_thinning_iterations: int = 256

    def __post_init__(self) -> None:
        if self.min_component_area < 1:
            raise ValueError("min_component_area must be positive")
        if self.morphology_kernel < 0:
            raise ValueError("morphology_kernel cannot be negative")
        if self.smooth_window < 1:
            raise ValueError("smooth_window must be positive")
        if self.waypoint_spacing_px <= 0:
            raise ValueError("waypoint_spacing_px must be positive")


@dataclass
class CenterlineResult:
    cleaned_mask: np.ndarray
    skeleton: np.ndarray
    ordered_path_uv: np.ndarray
    waypoints_uv: np.ndarray
    path_length_px: float


def clean_binary_mask(mask: np.ndarray, config: CenterlineConfig) -> np.ndarray:
    """Binarize, close small gaps, and remove tiny connected components."""
    if mask is None or mask.ndim != 2:
        raise ValueError("mask must be a non-empty HxW array")
    binary = (mask > 0).astype(np.uint8) * 255
    kernel_size = config.morphology_kernel
    if kernel_size > 1:
        if kernel_size % 2 == 0:
            kernel_size += 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    cleaned = np.zeros_like(binary)
    for label in range(1, count):
        if int(stats[label, cv2.CC_STAT_AREA]) >= config.min_component_area:
            cleaned[labels == label] = 255
    return cleaned


def thin_zhang_suen(mask: np.ndarray, max_iterations: int = 256) -> np.ndarray:
    """Vectorized Zhang-Suen thinning, returned as a uint8 0/255 image."""
    image = (mask > 0).astype(np.uint8)
    if not np.any(image):
        return image

    for _ in range(max_iterations):
        changed = False
        for sub_iteration in (0, 1):
            padded = np.pad(image, 1, mode="constant")
            p2 = padded[:-2, 1:-1]
            p3 = padded[:-2, 2:]
            p4 = padded[1:-1, 2:]
            p5 = padded[2:, 2:]
            p6 = padded[2:, 1:-1]
            p7 = padded[2:, :-2]
            p8 = padded[1:-1, :-2]
            p9 = padded[:-2, :-2]
            neighbors = (p2, p3, p4, p5, p6, p7, p8, p9)
            neighbor_count = sum(neighbors)
            transitions = sum(
                ((neighbors[index] == 0) & (neighbors[(index + 1) % 8] == 1)).astype(np.uint8)
                for index in range(8)
            )
            if sub_iteration == 0:
                topology = (p2 * p4 * p6 == 0) & (p4 * p6 * p8 == 0)
            else:
                topology = (p2 * p4 * p8 == 0) & (p2 * p6 * p8 == 0)
            remove = (
                (image == 1)
                & (neighbor_count >= 2)
                & (neighbor_count <= 6)
                & (transitions == 1)
                & topology
            )
            if np.any(remove):
                image[remove] = 0
                changed = True
        if not changed:
            break
    return image.astype(np.uint8) * 255


_NEIGHBORS = (
    (-1, -1, math.sqrt(2.0)), (-1, 0, 1.0), (-1, 1, math.sqrt(2.0)),
    (0, -1, 1.0),                              (0, 1, 1.0),
    (1, -1, math.sqrt(2.0)),  (1, 0, 1.0),  (1, 1, math.sqrt(2.0)),
)


def _select_component(skeleton: np.ndarray, anchor_uv: tuple[float, float] | None) -> np.ndarray:
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        (skeleton > 0).astype(np.uint8), connectivity=8
    )
    if count <= 1:
        return np.zeros_like(skeleton)
    candidates = list(range(1, count))
    if anchor_uv is None:
        selected = max(candidates, key=lambda label: int(stats[label, cv2.CC_STAT_AREA]))
    else:
        anchor_u, anchor_v = anchor_uv
        def score(label: int) -> tuple[float, int]:
            coords = np.argwhere(labels == label)
            distance_sq = np.min((coords[:, 1] - anchor_u) ** 2 + (coords[:, 0] - anchor_v) ** 2)
            return float(distance_sq), -int(coords.shape[0])
        selected = min(candidates, key=score)
    return (labels == selected).astype(np.uint8) * 255


def _build_graph(skeleton: np.ndarray):
    coords = np.argwhere(skeleton > 0)
    lookup = {tuple(coord): index for index, coord in enumerate(coords)}
    adjacency: list[list[tuple[int, float]]] = [[] for _ in range(len(coords))]
    for index, (v, u) in enumerate(coords):
        for dv, du, weight in _NEIGHBORS:
            neighbor = lookup.get((int(v + dv), int(u + du)))
            if neighbor is not None:
                adjacency[index].append((neighbor, weight))
    return coords, adjacency


def _shortest_paths(adjacency, start: int):
    distances = np.full(len(adjacency), np.inf, dtype=np.float64)
    previous = np.full(len(adjacency), -1, dtype=np.int64)
    distances[start] = 0.0
    queue = [(0.0, start)]
    while queue:
        distance, node = heapq.heappop(queue)
        if distance != distances[node]:
            continue
        for neighbor, weight in adjacency[node]:
            candidate = distance + weight
            if candidate < distances[neighbor]:
                distances[neighbor] = candidate
                previous[neighbor] = node
                heapq.heappush(queue, (candidate, neighbor))
    return distances, previous


def _ordered_primary_path(skeleton: np.ndarray, anchor_uv: tuple[float, float] | None) -> np.ndarray:
    component = _select_component(skeleton, anchor_uv)
    coords, adjacency = _build_graph(component)
    if len(coords) < 2:
        return np.empty((0, 2), dtype=np.float32)
    endpoints = [index for index, neighbors in enumerate(adjacency) if len(neighbors) <= 1]
    candidates = endpoints or list(range(len(coords)))
    if anchor_uv is None:
        start = max(candidates, key=lambda index: (int(coords[index, 0]), -int(coords[index, 1])))
    else:
        anchor_u, anchor_v = anchor_uv
        start = min(
            candidates,
            key=lambda index: (float(coords[index, 1]) - anchor_u) ** 2 + (float(coords[index, 0]) - anchor_v) ** 2,
        )
    distances, previous = _shortest_paths(adjacency, start)
    reachable = [index for index in candidates if np.isfinite(distances[index])]
    end = max(reachable, key=lambda index: distances[index])
    path_indices = []
    node = end
    while node >= 0:
        path_indices.append(node)
        if node == start:
            break
        node = int(previous[node])
    path_indices.reverse()
    path_vu = coords[path_indices].astype(np.float32)
    return path_vu[:, [1, 0]]


def _smooth_path(path_uv: np.ndarray, window: int) -> np.ndarray:
    if len(path_uv) < 3 or window <= 1:
        return path_uv.copy()
    window = min(window if window % 2 == 1 else window + 1, len(path_uv) | 1)
    if window <= 1:
        return path_uv.copy()
    pad = window // 2
    kernel = np.full(window, 1.0 / window, dtype=np.float32)
    padded = np.pad(path_uv, ((pad, pad), (0, 0)), mode="edge")
    smoothed = np.column_stack([
        np.convolve(padded[:, axis], kernel, mode="valid") for axis in range(2)
    ]).astype(np.float32)
    smoothed[0] = path_uv[0]
    smoothed[-1] = path_uv[-1]
    return smoothed


def _resample_path(path_uv: np.ndarray, spacing: float) -> tuple[np.ndarray, float]:
    if len(path_uv) < 2:
        return path_uv.copy(), 0.0
    segment_lengths = np.linalg.norm(np.diff(path_uv, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    keep = np.concatenate(([True], np.diff(cumulative) > 1.0e-6))
    path_uv = path_uv[keep]
    cumulative = cumulative[keep]
    total = float(cumulative[-1])
    if total <= 1.0e-6:
        return path_uv[:1].copy(), total
    samples = np.arange(0.0, total, spacing, dtype=np.float32)
    if len(samples) == 0 or total - float(samples[-1]) > 1.0e-3:
        samples = np.append(samples, total)
    points = np.column_stack([
        np.interp(samples, cumulative, path_uv[:, axis]) for axis in range(2)
    ]).astype(np.float32)
    return points, total


def extract_centerline(
    mask: np.ndarray,
    config: CenterlineConfig | None = None,
    anchor_uv: tuple[float, float] | None = None,
) -> CenterlineResult:
    config = config or CenterlineConfig()
    cleaned = clean_binary_mask(mask, config)
    skeleton = thin_zhang_suen(cleaned, config.max_thinning_iterations)
    ordered = _ordered_primary_path(skeleton, anchor_uv)
    if len(ordered) < 2:
        raise RuntimeError("No usable crack centerline was found after mask cleanup")
    smoothed = _smooth_path(ordered, config.smooth_window)
    waypoints, path_length = _resample_path(smoothed, config.waypoint_spacing_px)
    return CenterlineResult(cleaned, skeleton, ordered, waypoints, path_length)


def draw_trajectory_overlay(image_bgr: np.ndarray, result: CenterlineResult) -> np.ndarray:
    overlay = image_bgr.copy()
    skeleton_pixels = result.skeleton > 0
    overlay[skeleton_pixels] = (255, 255, 0)
    if len(result.ordered_path_uv) >= 2:
        points = np.round(result.ordered_path_uv).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(overlay, [points], False, (255, 0, 0), 2, cv2.LINE_AA)
    for u, v in result.waypoints_uv:
        cv2.circle(overlay, (int(round(u)), int(round(v))), 3, (0, 255, 255), -1, cv2.LINE_AA)
    start = tuple(np.round(result.waypoints_uv[0]).astype(int))
    end = tuple(np.round(result.waypoints_uv[-1]).astype(int))
    cv2.circle(overlay, start, 7, (0, 255, 0), -1, cv2.LINE_AA)
    cv2.circle(overlay, end, 7, (0, 0, 255), -1, cv2.LINE_AA)
    return cv2.addWeighted(image_bgr, 0.55, overlay, 0.75, 0.0)
