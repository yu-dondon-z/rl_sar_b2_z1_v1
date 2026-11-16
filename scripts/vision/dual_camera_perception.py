"""Image-only seam/TCP measurements for the experimental dual-camera task.

All returned geometry is in the B2 base frame *at image capture*.  Camera
optical axes are ROS/OpenCV (right, down, forward); base axes are forward,
left, up.  No robot joint state, USD object pose, or simulated TCP/ground
position is read here.  RGB and depth must already share the RGB intrinsics.

The production mode consumes the existing RHANet ``CrackSegmenter``.  The
explicit ``classical`` mode is only a dark-tape scene diagnostic: its score
is image contrast, never a neural probability or evidence of shadow rejection.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys

import cv2
import numpy as np


@dataclass(frozen=True)
class PerceptionConfig:
    mode: str = "rhanet"
    rhacracknet_root: str = "/home/dong/RHACrackNet/code"
    min_seam_confidence: float = 0.90
    min_path_points: int = 6
    min_path_length_m: float = 0.12
    front_max_forward_m: float = 3.0
    image_border_margin_px: int = 8
    rear_max_foreground_height_m: float = 0.015
    rear_max_seam_depression_m: float = 0.10
    rear_foreground_margin_px: int = 4
    # The rear controller samples the seam at x=-1.02 m. Do not validate
    # arbitrarily distant seam pixels: perspective makes a small image
    # waypoint spacing look like a large metric jump there, although that
    # section is never used for the tool target.
    rear_validation_x_min_m: float = -2.0
    rear_validation_x_max_m: float = -0.4
    min_valid_depth_ratio: float = 0.65
    min_depth_m: float = 0.08
    max_depth_m: float = 4.5
    ground_inlier_distance_m: float = 0.012
    ground_min_inliers: int = 100
    ground_min_support_ratio: float = 0.20
    ground_max_tilt_deg: float = 40.0
    marker_radius_m: float = 0.035
    marker_axial_spacing_m: float = 0.10
    marker_tip_extension_m: float = 0.10
    marker_spacing_tolerance_m: float = 0.025
    marker_max_fit_rms_m: float = 0.008


def quaternion_wxyz_to_matrix(quaternion) -> np.ndarray:
    q = np.asarray(quaternion, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(q))
    if not np.all(np.isfinite(q)) or norm < 1.0e-10:
        raise ValueError("invalid camera quaternion")
    w, x, y, z = q / norm
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def camera_geometry(metadata: dict) -> tuple[np.ndarray, np.ndarray]:
    intrinsic = np.asarray(metadata["intrinsic_matrix"], dtype=np.float64).reshape(3, 3)
    if not np.all(np.isfinite(intrinsic)) or min(intrinsic[0, 0], intrinsic[1, 1]) <= 0:
        raise ValueError("invalid RGB intrinsic matrix")
    if "T_base_camera" in metadata:
        transform = np.asarray(metadata["T_base_camera"], dtype=np.float64).reshape(4, 4)
    else:
        camera = metadata.get("camera_to_base", {})
        position = camera.get("position", metadata.get("camera_position_b"))
        quaternion = camera.get("quaternion_wxyz", metadata.get("camera_quaternion_b_ros_wxyz"))
        if position is None or quaternion is None:
            raise ValueError("camera-to-base calibration is missing")
        transform = np.eye(4)
        transform[:3, :3] = quaternion_wxyz_to_matrix(quaternion)
        transform[:3, 3] = np.asarray(position, dtype=np.float64).reshape(3)
    rotation = transform[:3, :3]
    if (not np.all(np.isfinite(transform))
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-4)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1.0e-4)
            or not np.allclose(transform[3], [0, 0, 0, 1])):
        raise ValueError("camera-to-base transform is not rigid")
    # Distorted real images must be rectified/aligned before using this pinhole
    # geometry; silently applying pinhole projection would introduce TCP bias.
    if np.any(np.abs(np.asarray(metadata.get("distortion_coefficients", []))) > 1.0e-10):
        raise ValueError("rectify RGB and aligned depth before pinhole projection")
    return intrinsic, transform


def unproject(uv: np.ndarray, depth_m: np.ndarray, intrinsic: np.ndarray,
              transform: np.ndarray) -> np.ndarray:
    """Depth is distance to image plane (optical Z), not Euclidean range."""
    pixels = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
    rays = np.column_stack((pixels, np.ones(len(pixels)))) @ np.linalg.inv(intrinsic).T
    points = rays * np.asarray(depth_m, dtype=np.float64).reshape(-1, 1)
    return points @ transform[:3, :3].T + transform[:3, 3]


def normalize_plane(plane) -> np.ndarray:
    result = np.asarray(plane, dtype=np.float64).reshape(4)
    length = float(np.linalg.norm(result[:3]))
    if not np.all(np.isfinite(result)) or length < 1.0e-8:
        raise ValueError("ground plane is missing or invalid")
    result = result / length
    return -result if result[2] < 0 else result


def intersect_pixels_with_plane(uv: np.ndarray, intrinsic: np.ndarray,
                                 transform: np.ndarray, plane) -> tuple[np.ndarray, np.ndarray]:
    plane = normalize_plane(plane)
    pixels = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
    rays = np.column_stack((pixels, np.ones(len(pixels)))) @ np.linalg.inv(intrinsic).T
    directions = rays @ transform[:3, :3].T
    denominator = directions @ plane[:3]
    distance = np.full(len(pixels), np.nan)
    usable = np.abs(denominator) > 1.0e-8
    distance[usable] = -(transform[:3, 3] @ plane[:3] + plane[3]) / denominator[usable]
    valid = usable & np.isfinite(distance) & (distance > 0.08) & (distance < 8.0)
    return transform[:3, 3] + directions * distance[:, None], valid


def front_ground_roi(image_shape, intrinsic: np.ndarray, transform: np.ndarray,
                     plane, max_forward_m: float) -> np.ndarray:
    """Pixels whose calibrated rays hit the measured local steering region."""
    plane = normalize_plane(plane)
    vv, uu = np.indices(image_shape[:2], dtype=np.float64)
    inverse = np.linalg.inv(intrinsic)
    normal_ray = plane[:3] @ transform[:3, :3] @ inverse
    x_ray = transform[0, :3] @ inverse
    denominator = normal_ray[0]*uu + normal_ray[1]*vv + normal_ray[2]
    depth = np.full(uu.shape, np.nan)
    usable = np.abs(denominator) > 1.0e-8
    depth[usable] = -(plane[:3] @ transform[:3, 3] + plane[3]) / denominator[usable]
    forward = transform[0, 3] + depth*(x_ray[0]*uu + x_ray[1]*vv + x_ray[2])
    return usable & (depth > .08) & (depth < 8.) & (forward > .10) & (forward <= max_forward_m)


def rear_ground_roi(depth_m: np.ndarray, intrinsic: np.ndarray, transform: np.ndarray,
                    plane, config: PerceptionConfig) -> tuple[np.ndarray, dict]:
    """Keep measured road/trench pixels and exclude elevated tool/leg edges.

    Depth/plane classification happens *before* the neural-mask centerline.
    Projecting a gun silhouette onto the plane afterwards would manufacture a
    seam at the TCP, which can make a large real tracking error look perfect.
    """
    plane = normalize_plane(plane)
    vv, uu = np.indices(depth_m.shape, dtype=np.float64)
    normal_ray = plane[:3] @ transform[:3, :3] @ np.linalg.inv(intrinsic)
    height = depth_m*(normal_ray[0]*uu + normal_ray[1]*vv + normal_ray[2])
    height += plane[:3] @ transform[:3, 3] + plane[3]
    measured = np.isfinite(depth_m) & np.isfinite(height)
    elevated = measured & (height > config.rear_max_foreground_height_m)
    margin = max(0, config.rear_foreground_margin_px)
    if margin:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*margin+1, 2*margin+1))
        elevated = cv2.dilate(elevated.astype(np.uint8), kernel) > 0
    allowed = measured & ~elevated & (height >= -config.rear_max_seam_depression_m)
    return allowed, {"rear_ground_roi_ratio": float(np.mean(allowed)),
                     "rear_elevated_foreground_ratio": float(np.mean(elevated)),
                     "rear_max_foreground_height_m": config.rear_max_foreground_height_m,
                     "rear_max_seam_depression_m": config.rear_max_seam_depression_m,
                     "rear_foreground_margin_px": margin}


def project_to_plane(points: np.ndarray, plane) -> np.ndarray:
    plane = normalize_plane(plane)
    points = np.asarray(points, dtype=np.float64)
    return points - (points @ plane[:3] + plane[3])[..., None] * plane[:3]


def nearest_point_on_polyline(point: np.ndarray, path: np.ndarray) -> np.ndarray:
    path = np.asarray(path, dtype=np.float64).reshape(-1, 3)
    if len(path) < 2:
        raise ValueError("seam has fewer than two geometric points")
    starts, vectors = path[:-1], np.diff(path, axis=0)
    denominator = np.einsum("ij,ij->i", vectors, vectors)
    fractions = np.clip(np.einsum("ij,ij->i", point - starts, vectors)
                        / np.maximum(denominator, 1.0e-12), 0, 1)
    candidates = starts + fractions[:, None] * vectors
    return candidates[np.argmin(np.linalg.norm(candidates - point, axis=1))]


def fit_ground_plane(points_b: np.ndarray, config: PerceptionConfig) -> tuple[np.ndarray, dict]:
    """Fit the largest near-horizontal depth surface using deterministic RANSAC."""
    points = np.asarray(points_b, dtype=np.float64).reshape(-1, 3)
    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) < config.ground_min_inliers:
        raise ValueError("not enough valid rear depth points for ground plane")
    generator = np.random.default_rng(20261102)
    if len(points) > 4000:
        points = points[generator.choice(len(points), 4000, replace=False)]
    best = None
    cosine = np.cos(np.deg2rad(config.ground_max_tilt_deg))
    for _ in range(96):
        a, b, c = points[generator.choice(len(points), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        length = np.linalg.norm(normal)
        if length < 1.0e-7:
            continue
        normal /= length
        if abs(normal[2]) < cosine:
            continue
        offset = -a @ normal
        inliers = np.abs(points @ normal + offset) < config.ground_inlier_distance_m
        if best is None or np.count_nonzero(inliers) > np.count_nonzero(best):
            best = inliers
    if best is None or np.count_nonzero(best) < config.ground_min_inliers:
        raise ValueError("rear depth has no supported near-horizontal ground plane")
    support = float(np.mean(best))
    if support < config.ground_min_support_ratio:
        raise ValueError(f"ground depth support is too small ({support:.3f})")
    # Refit with measured inliers only; this never uses nominal ground height.
    for _ in range(2):
        center = points[best].mean(axis=0)
        _, _, vh = np.linalg.svd(points[best] - center, full_matrices=False)
        normal = vh[-1]
        plane = normalize_plane(np.r_[normal, -normal @ center])
        best = np.abs(points @ plane[:3] + plane[3]) < config.ground_inlier_distance_m
    if plane[2] < cosine:
        raise ValueError("refitted depth plane is too steep")
    spread = np.linalg.svd(points[best] - points[best].mean(axis=0), compute_uv=False)
    if len(spread) < 2 or spread[1] / np.sqrt(np.count_nonzero(best)) < 0.06:
        raise ValueError("ground points lack two-dimensional spatial support")
    residual = points[best] @ plane[:3] + plane[3]
    return plane, {"ground_inlier_ratio": float(np.mean(best)),
                   "ground_rms_m": float(np.sqrt(np.mean(residual**2))),
                   "ground_depth_points": len(points), "ground_source": "rear_aligned_depth_ransac"}


def fit_known_radius_sphere(points_camera: np.ndarray, radius_m: float) -> tuple[np.ndarray, float]:
    """Recover a sphere center from its visible depth surface, not its front face."""
    points = np.asarray(points_camera, dtype=np.float64).reshape(-1, 3)
    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) < 12 or radius_m <= 0:
        raise ValueError("insufficient marker surface depth")
    if len(points) > 1000:
        points = points[np.linspace(0, len(points) - 1, 1000).astype(int)]
    center = np.median(points, axis=0)
    center += radius_m * center / max(np.linalg.norm(center), 1.0e-9)
    for _ in range(25):
        delta = center - points
        distance = np.linalg.norm(delta, axis=1)
        residual = distance - radius_m
        jacobian = delta / np.maximum(distance[:, None], 1.0e-9)
        # Huber weights tolerate mixed foreground/background depth on the rim.
        weights = np.minimum(1.0, 0.004 / np.maximum(np.abs(residual), 1.0e-9))
        system = jacobian * np.sqrt(weights[:, None])
        step, _, _, _ = np.linalg.lstsq(system, -residual * np.sqrt(weights), rcond=None)
        if np.linalg.norm(step) > radius_m:
            step *= radius_m / np.linalg.norm(step)
        center += step
        if np.linalg.norm(step) < 1.0e-7:
            break
    residual = np.linalg.norm(points - center, axis=1) - radius_m
    good = np.abs(residual) <= 0.01
    if np.mean(good) < 0.75 or np.linalg.norm(center) <= np.median(np.linalg.norm(points, axis=1)):
        raise ValueError("marker depth is inconsistent with a visible sphere surface")
    return center, float(np.sqrt(np.mean(residual[good]**2)))


def marker_color_masks(image_bgr: np.ndarray) -> dict[str, np.ndarray]:
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    return {"cyan": cv2.inRange(hsv, (78, 110, 65), (103, 255, 255)),
            "magenta": cv2.inRange(hsv, (138, 110, 65), (172, 255, 255))}


class DualCameraPerception:
    def __init__(self, config: PerceptionConfig | None = None, segmenter=None):
        self.config = config or PerceptionConfig()
        if self.config.mode not in ("rhanet", "classical"):
            raise ValueError("perception mode must be rhanet or classical")
        if self.config.mode == "rhanet" and segmenter is None:
            raise ValueError("RHANet mode requires a loaded CrackSegmenter")
        self.segmenter = segmenter
        root = str(Path(self.config.rhacracknet_root).expanduser().resolve())
        if root not in sys.path:
            sys.path.insert(0, root)
        from crack_trajectory import CenterlineConfig, extract_centerline
        self._extract_centerline = extract_centerline
        self._centerline_config = CenterlineConfig(waypoint_spacing_px=6.0)

    def _base_result(self, metadata: dict) -> dict:
        result = {"valid": False, "reason": "not processed", "frame_id": "b2_base_at_capture",
                  "sequence": int(metadata.get("sequence", -1)),
                  "source_sequence": int(metadata.get("sequence", -1)),
                  "camera_name": metadata.get("camera_name", "unknown"),
                  "perception_mode": self.config.mode,
                  "confidence_kind": "neural_probability" if self.config.mode == "rhanet" else "dark_pixel_contrast",
                  "points_b": [], "control_waypoints_robot_xyz": [], "waypoints_uv": []}
        for key in ("capture_timestamp", "capture_timestamp_ns", "timestamp_ns", "capture_generation",
                    "base_position_w", "base_quaternion_w", "capture_base_position_w",
                    "capture_base_quaternion_wxyz"):
            if key in metadata:
                result[key] = metadata[key]
        return result

    def _centerline(self, image: np.ndarray, anchor_v_ratio: float,
                    allowed_pixels: np.ndarray | None = None) -> tuple[np.ndarray, float, dict]:
        if image is None or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("RGB input must be an HxWx3 BGR image")
        if self.config.mode == "rhanet":
            segmentation = self.segmenter.predict_bgr(image)
            mask = np.asarray(segmentation.mask, dtype=np.uint8).copy()
            probability = np.asarray(segmentation.probability, dtype=np.float64)
            diagnostics = {"inference_ms": float(segmentation.inference_ms)}
        else:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            mask = ((gray < 70) & (hsv[..., 1] < 100)).astype(np.uint8) * 255
            probability = 1.0 - gray.astype(np.float64) / 255.0
            diagnostics = {"diagnostic_only": True}
        if probability.shape != image.shape[:2] or mask.shape != image.shape[:2]:
            raise ValueError("segmentation output dimensions differ from RGB")
        diagnostics["raw_mask_area_ratio"] = float(np.mean(mask > 0))
        if allowed_pixels is not None:
            if allowed_pixels.shape != mask.shape:
                raise ValueError("seam geometric ROI dimensions differ from RGB")
            mask[~allowed_pixels] = 0
            diagnostics["geometric_roi_ratio"] = float(np.mean(allowed_pixels))
        # Encoder/decoder padding can create a highly confident thin stripe
        # exactly on the image boundary. Exclude this unmeasurable rim before
        # thinning so it cannot attach to the real seam as a long false branch.
        margin = self.config.image_border_margin_px
        if margin > 0:
            mask[:margin] = mask[-margin:] = 0
            mask[:, :margin] = mask[:, -margin:] = 0
        # The colored TCP markers are measurements, not seam candidates.
        colors = marker_color_masks(image)
        marker_pixels = cv2.dilate(colors["cyan"] | colors["magenta"], np.ones((5, 5), np.uint8))
        mask[marker_pixels > 0] = 0
        area = float(np.mean(mask > 0))
        if not 0.0003 <= area <= 0.20:
            raise ValueError(f"seam mask area is implausible ({area:.4f})")
        height, width = image.shape[:2]
        trajectory = self._extract_centerline(mask, self._centerline_config,
                                             ((width - 1) / 2, anchor_v_ratio * (height - 1)))
        uv = np.asarray(trajectory.waypoints_uv, dtype=np.float64)
        if len(uv) < self.config.min_path_points or trajectory.path_length_px < 30:
            raise ValueError("seam centerline is too short")
        pixels = np.rint(uv).astype(int)
        pixels[:, 0] = np.clip(pixels[:, 0], 0, width - 1)
        pixels[:, 1] = np.clip(pixels[:, 1], 0, height - 1)
        confidence = float(np.mean(probability[pixels[:, 1], pixels[:, 0]]))
        if confidence < self.config.min_seam_confidence:
            raise ValueError(f"seam confidence too low ({confidence:.3f})")
        return uv, confidence, {**diagnostics, "mask_area_ratio": area}

    def _path_result(self, result: dict, points: np.ndarray, uv: np.ndarray, confidence: float) -> None:
        if len(points) < self.config.min_path_points:
            raise ValueError("too few valid geometric seam points")
        if points[-1, 0] < points[0, 0]:
            points, uv = points[::-1].copy(), uv[::-1].copy()
        steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
        if np.max(steps) > 0.25:
            raise ValueError("seam geometry contains a gap larger than 0.25 m")
        length = float(np.sum(steps))
        if length < self.config.min_path_length_m:
            raise ValueError("geometric seam path is too short")
        result.update(points_b=points.tolist(), control_waypoints_robot_xyz=points.tolist(),
                      waypoints_uv=uv.tolist(), confidence=confidence,
                      mean_waypoint_confidence=confidence, path_length_m=length)

    def process_front(self, metadata: dict, rgb_bgr: np.ndarray) -> dict:
        result = self._base_result(metadata)
        try:
            intrinsic, transform = camera_geometry(metadata)
            if "ground_plane_b" not in metadata:
                raise ValueError("front RGB needs a synchronized rear-measured ground plane")
            plane = normalize_plane(metadata["ground_plane_b"])
            allowed = front_ground_roi(rgb_bgr.shape, intrinsic, transform, plane,
                                       self.config.front_max_forward_m)
            uv, confidence, diagnostics = self._centerline(rgb_bgr, 0.65, allowed)
            points, valid = intersect_pixels_with_plane(uv, intrinsic, transform, plane)
            if np.mean(valid) < 0.65:
                raise ValueError("too few front seam rays intersect measured ground in front of camera")
            # A horizontal camera sees the seam all the way to the horizon.
            # Near-horizon pixel noise corresponds to arbitrarily large metric
            # gaps; the steering loop needs a bounded visible local section,
            # not that ill-conditioned distant extension. This is measured
            # geometry clipping, not a depth or confidence replacement.
            valid &= (points[:, 0] > 0.10) & (points[:, 0] <= self.config.front_max_forward_m)
            points, uv = points[valid], uv[valid]
            self._path_result(result, points, uv, confidence)
            result.update(diagnostics, ground_plane_b=plane.tolist(),
                          front_max_forward_m=self.config.front_max_forward_m,
                          geometry_source="front_rgb_rays_intersect_rear_depth_plane", valid=True, reason="ok")
        except (KeyError, ValueError, RuntimeError, cv2.error) as error:
            result["reason"] = str(error)
        return result

    def _depth_m(self, metadata: dict, rgb: np.ndarray, depth: np.ndarray) -> np.ndarray:
        if depth is None or depth.ndim != 2 or depth.shape != rgb.shape[:2]:
            raise ValueError("rear RGB and aligned depth dimensions must match")
        if metadata.get("depth_aligned_to_rgb", True) is not True:
            raise ValueError("rear depth is not aligned to RGB")
        unit = float(metadata.get("depth_unit_m", 0.001))
        if not np.isfinite(unit) or unit <= 0:
            raise ValueError("invalid rear depth unit")
        measured = np.asarray(depth, dtype=np.float64) * unit
        valid = np.isfinite(measured) & (measured >= self.config.min_depth_m) & (measured <= self.config.max_depth_m)
        return np.where(valid, measured, np.nan)

    def _markers(self, image: np.ndarray, depth_m: np.ndarray, intrinsic: np.ndarray,
                 transform: np.ndarray, metadata: dict) -> tuple[np.ndarray, dict]:
        markers = {}
        radius = float(metadata.get("marker_radius_m", self.config.marker_radius_m))
        for name, mask in marker_color_masks(image).items():
            count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
            candidates = []
            for label in range(1, count):
                if stats[label, cv2.CC_STAT_AREA] < 16:
                    continue
                # Erode the color rim where aligned depth mixes background.
                selected = cv2.erode((labels == label).astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
                vv, uu = np.nonzero(selected & np.isfinite(depth_m))
                if len(uu) < 12:
                    continue
                points = unproject(np.column_stack((uu, vv)), depth_m[vv, uu], intrinsic, np.eye(4))
                try:
                    center_camera, rms = fit_known_radius_sphere(points, radius)
                    if rms > self.config.marker_max_fit_rms_m:
                        continue
                    center_b = transform[:3, :3] @ center_camera + transform[:3, 3]
                    projected = intrinsic @ center_camera
                    candidates.append((len(uu), {"uv": (projected[:2] / projected[2]).tolist(),
                        "center_b": center_b.tolist(), "fit_rms_m": rms, "surface_depth_samples": len(uu)}))
                except ValueError:
                    continue
            if not candidates:
                raise ValueError(f"{name} TCP marker missing, occluded, or has invalid sphere depth")
            markers[name] = max(candidates, key=lambda item: item[0])[1]
        cyan = np.asarray(markers["cyan"]["center_b"])
        magenta = np.asarray(markers["magenta"]["center_b"])
        axis = cyan - magenta
        spacing = float(np.linalg.norm(axis))
        expected = float(metadata.get("marker_axial_spacing_m", self.config.marker_axial_spacing_m))
        if abs(spacing - expected) > self.config.marker_spacing_tolerance_m:
            raise ValueError(f"TCP marker spacing inconsistent with calibration ({spacing:.3f} m)")
        extension = float(metadata.get("marker_tip_extension_m", self.config.marker_tip_extension_m))
        tcp = cyan + axis / spacing * extension
        markers["measured_spacing_m"] = spacing
        return tcp, markers

    def process_rear(self, metadata: dict, rgb_bgr: np.ndarray, depth: np.ndarray) -> dict:
        result = self._base_result(metadata)
        try:
            intrinsic, transform = camera_geometry(metadata)
            depth_m = self._depth_m(metadata, rgb_bgr, depth)
            yy, xx = np.mgrid[0:depth_m.shape[0]:6, 0:depth_m.shape[1]:6]
            sampled = depth_m[yy, xx]
            valid = np.isfinite(sampled)
            points_b = unproject(np.column_stack((xx[valid], yy[valid])), sampled[valid], intrinsic, transform)
            plane, diagnostics = fit_ground_plane(points_b, self.config)
            result.update(diagnostics, ground_plane_b=plane.tolist())
            tcp, markers = self._markers(rgb_bgr, depth_m, intrinsic, transform, metadata)
            camera_tcp = transform[:3, :3].T @ (tcp - transform[:3, 3])
            projected_tcp = intrinsic @ camera_tcp
            result.update(tcp_measured_b=tcp.tolist(), markers=markers,
                          tcp_pixel_uv=(projected_tcp[:2] / projected_tcp[2]).tolist(),
                          tcp_source="two_colored_spheres_fitted_from_aligned_depth")
            ground_pixels, ground_roi_diagnostics = rear_ground_roi(
                depth_m, intrinsic, transform, plane, self.config)
            result.update(ground_roi_diagnostics)
            uv, confidence, seam_diagnostics = self._centerline(rgb_bgr, 0.65, ground_pixels)
            seam_depth_m = np.where(ground_pixels, depth_m, np.nan)
            depths = np.full(len(uv), np.nan)
            for i, (u, v) in enumerate(np.rint(uv).astype(int)):
                if 1 <= u < depth_m.shape[1] - 1 and 1 <= v < depth_m.shape[0] - 1:
                    patch = seam_depth_m[v-1:v+2, u-1:u+2]
                    good = patch[np.isfinite(patch)]
                    if len(good) >= 3:
                        depths[i] = np.median(good)
            valid = np.isfinite(depths)
            ratio = float(np.mean(valid))
            if ratio < self.config.min_valid_depth_ratio:
                raise ValueError(f"rear seam depth support too low ({ratio:.3f})")
            points = unproject(uv[valid], depths[valid], intrinsic, transform)
            # This measures the seam's ground projection, including a real
            # recessed crack; no nominal TCP standoff is substituted here.
            points = project_to_plane(points, plane)
            self._path_result(result, points, uv[valid], confidence)
            tcp_ground = project_to_plane(tcp, plane)
            closest = nearest_point_on_polyline(tcp_ground, points)
            error = closest - tcp_ground
            result.update(seam_diagnostics, valid=True, reason="ok", valid_3d_ratio=ratio,
                          tcp_projection_b=tcp_ground.tolist(), seam_projection_b=closest.tolist(),
                          tcp_error_b=error.tolist(), tcp_error_m=float(np.linalg.norm(error)),
                          tcp_height_above_plane_m=float(tcp @ plane[:3] + plane[3]),
                          geometry_source="rear_rgb_seam_and_aligned_depth")
        except (KeyError, ValueError, RuntimeError, cv2.error) as error:
            result["reason"] = str(error)
        return result
