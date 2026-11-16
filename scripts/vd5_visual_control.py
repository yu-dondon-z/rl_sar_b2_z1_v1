"""VD5 commands from rendered/real dual-camera measurements only."""

from __future__ import annotations

import math
import importlib.util
from pathlib import Path
import sys

import numpy as np

from vd5_mujoco_control import (normalize_quaternion, quaternion_from_euler_xyz,
                               startup_fraction)


SCHEMA = "b2_z1_dual_vision_v1"
_VISION_ROOT = Path(__file__).resolve().parent / "vision"
if _VISION_ROOT.is_dir() and str(_VISION_ROOT) not in sys.path:
    sys.path.insert(0, str(_VISION_ROOT))
_RHANET_ROOT = _VISION_ROOT / "rhacracknet"
if _RHANET_ROOT.is_dir() and str(_RHANET_ROOT) not in sys.path:
    sys.path.insert(0, str(_RHANET_ROOT))
from vd1_control import _front_geometry


class RearTargetSupportShortfall(ValueError):
    """Valid visible path with insufficient work-point coverage.

    Legacy callers retain ValueError/fail-closed behavior. VD14 may explicitly
    pause its commanded motion instead of ending the simulation.
    """
    def __init__(self, message, required_m, allowed_m):
        super().__init__(message)
        self.diagnostics = dict(rear_support_required_extension_m=float(required_m),
                                rear_support_allowed_extension_m=float(allowed_m))


def _finite_points(value, minimum=6):
    points = np.asarray(value, dtype=np.float64)
    if (points.ndim != 2 or points.shape[1] != 3 or len(points) < minimum
            or not np.isfinite(points).all() or np.max(np.abs(points)) > 20.0):
        raise ValueError("visual path needs finite xyz points")
    return points


def _plane(value):
    plane = np.asarray(value, dtype=np.float64)
    if plane.shape != (4,) or not np.isfinite(plane).all():
        raise ValueError("rear camera did not provide a measured ground plane")
    norm = float(np.linalg.norm(plane[:3]))
    if not 0.5 <= norm <= 2.0:
        raise ValueError("invalid rear depth ground plane")
    plane = plane / norm
    if plane[2] < 0.0:
        plane = -plane
    if plane[2] < 0.4:
        raise ValueError("measured ground plane is too tilted")
    return plane


def _ground_basis(plane):
    normal = plane[:3]
    forward = np.array((1.0, 0.0, 0.0)) - normal[0] * normal
    norm = float(np.linalg.norm(forward))
    if norm < 0.25:
        raise ValueError("base-forward axis is inconsistent with measured ground")
    forward /= norm
    left = np.cross(normal, forward)
    return np.column_stack((forward, left, normal))


def front_visual_command(points_b, ground_plane_b, speed, *, heading_gain=1.5,
                         lookahead=0.6, max_yaw_rate=0.8, geometry_fn=None):
    """Convert front RGB centerline points projected by measured rear depth to [vx, vy, wz]."""
    points = _finite_points(points_b)
    plane = _plane(ground_plane_b)
    basis = _ground_basis(plane)
    points = points[points[:, 0] > 0.10]
    if len(points) < 6 or np.ptp(points[:, 0]) < 0.30:
        raise ValueError("front path needs at least 0.30 m forward visual support")
    level = points @ basis
    projected_base = -plane[3] * plane[:3]
    base_level = projected_base @ basis
    centered = level - base_level
    centered = centered[np.argsort(centered[:, 0], kind="stable")]
    xy = centered[:, :2]
    if np.ptp(xy[:, 0]) < 0.30:
        raise ValueError("front ground-projected path support is too short")
    geometry = _front_geometry if geometry_fn is None else geometry_fn
    heading, lateral, path_offset, curvature, lookahead_point = geometry(centered)
    yaw_rate = float(np.clip(heading_gain * heading + speed * curvature,
                             -max_yaw_rate, max_yaw_rate))
    return np.array((float(speed), 0.0, yaw_rate)), {
        "visible_point_count": int(len(xy)),
        "visible_forward_support_m": float(np.ptp(xy[:, 0])),
        "lateral_error_m": lateral,
        "path_left_offset_m": path_offset,
        "heading_error_rad": heading,
        "curvature_m_inv": curvature,
        "lookahead_b": lookahead_point.tolist(),
        "yaw_rate_rps": yaw_rate,
        "ground_plane_b": plane.tolist(),
        "command_source": "front_rgb_centerline_plus_rear_depth_ground_plane",
    }


def rear_visual_target(points_b, ground_plane_b, trailing_distance=0.70,
                       max_occlusion_extension=0.0):
    """Sample measured rear seam support at a fixed arc distance behind base origin.

    The default only bridges the camera/body-occluded near field (0.25 m).
    An explicit diagnostic option estimates at most 5 cm past the far endpoint
    from a measured, locally straight, well-supported 12 cm or longer segment.
    Such a target is labelled as an estimate, not a directly observed point.
    """
    points = _finite_points(points_b)
    plane = _plane(ground_plane_b)
    if not math.isfinite(trailing_distance) or trailing_distance <= 0.0:
        raise ValueError("trailing distance must be positive")
    if not math.isfinite(max_occlusion_extension) or not 0. <= max_occlusion_extension <= .05:
        raise ValueError('gun-occlusion extension must be between zero and 0.05 m')
    if points[-1, 0] < points[0, 0]:
        points = points[::-1].copy()
    delta = np.diff(points, axis=0)
    lengths = np.linalg.norm(delta, axis=1)
    keep = lengths > 1.0e-5
    if not np.all(keep):
        points = np.vstack((points[0], points[1:][keep]))
        delta = np.diff(points, axis=0)
        lengths = np.linalg.norm(delta, axis=1)
    if len(lengths) < 2:
        raise ValueError("rear seam has too few nondegenerate segments")
    projected_base = -plane[3] * plane[:3]
    relative_xy = projected_base[:2] - points[:-1, :2]
    segment_xy = delta[:, :2]
    length2 = np.sum(segment_xy * segment_xy, axis=1)
    fraction = np.clip(np.sum(relative_xy * segment_xy, axis=1)
                       / np.maximum(length2, 1.0e-10), 0.0, 1.0)
    nearest = points[:-1] + fraction[:, None] * delta
    distance2 = np.sum((nearest[:, :2] - projected_base[:2]) ** 2, axis=1)
    index = int(np.argmin(distance2))
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    nearest_station = cumulative[index] + fraction[index] * lengths[index]
    # A one-pixel hook at the image/foreground boundary must not set the
    # endpoint heading. Fit a 15 cm neighbourhood in measured arc length.
    neighbourhood = np.abs(cumulative-nearest_station) <= .15
    if np.count_nonzero(neighbourhood) < 3:
        distances = np.argsort(np.abs(cumulative-nearest_station))[:3]
        neighbourhood[distances] = True
    station = cumulative[neighbourhood]
    centered_station = station-station.mean()
    tangent = centered_station @ points[neighbourhood]
    tangent_norm = float(np.linalg.norm(tangent))
    if tangent_norm < 1.e-8:
        raise ValueError('rear measured centreline has no consistent local tangent')
    path_tangent = tangent / tangent_norm
    gap_vector = projected_base - nearest[index]
    gap_along = float(gap_vector @ path_tangent)
    gap_cross = float(np.linalg.norm(gap_vector - gap_along * path_tangent))
    if not 0.0 <= gap_along <= 1.20:
        raise ValueError("rear seam endpoint is not behind the base along its visible tangent")
    base_s = nearest_station + gap_along
    target_s = base_s - trailing_distance
    occlusion_extension = max(0., -target_s)
    fit_diagnostics = {}
    if target_s < 0.0:
        if occlusion_extension > max_occlusion_extension:
            raise RearTargetSupportShortfall(
                "rear RGB-D seam support is too short behind the TCP target",
                occlusion_extension, max_occlusion_extension)
        support = cumulative <= .18
        if np.count_nonzero(support) < 8 or np.ptp(cumulative[support]) < .12:
            raise ValueError('gun-occlusion estimate needs eight measured points spanning at least 0.12 m')
        station = cumulative[support]
        fit_points = points[support]
        design = np.column_stack((np.ones(len(station)), station))
        coefficients = np.linalg.lstsq(design, fit_points, rcond=None)[0]
        residuals = np.linalg.norm(fit_points-design @ coefficients, axis=1)
        residual_p90 = float(np.percentile(residuals, 90))
        direction = coefficients[1]
        scale = float(np.linalg.norm(direction))
        # Curled edges and short noisy fragments are not support for a hidden point.
        if residual_p90 > .008 or float(np.max(residuals)) > .015 or not .8 <= scale <= 1.05:
            raise ValueError('gun-occlusion visible-line fit is not reliable enough for extrapolation')
        tangent = direction / scale
        if tangent[0] < .7:
            raise ValueError('gun-occlusion visible line is not directed towards the base')
        fitted_endpoint = coefficients[0]
        target = fitted_endpoint-occlusion_extension*tangent
        actual_extension = float(np.linalg.norm(target-points[0]))
        if actual_extension > max_occlusion_extension:
            raise RearTargetSupportShortfall(
                'fitted gun-occlusion target exceeds the configured extension distance',
                actual_extension, max_occlusion_extension)
        occlusion_extension = actual_extension
        fit_diagnostics = dict(rear_occlusion_fit_points=int(len(station)),
                               rear_occlusion_fit_span_m=float(np.ptp(station)),
                               rear_occlusion_fit_residual_p90_m=residual_p90)
    near_field_extension = max(0.0, target_s - cumulative[-1])
    if near_field_extension > 0.25:
        raise ValueError("rear TCP target lies more than 0.25 m inside the body-occluded near field")
    if near_field_extension > 0.0:
        target = points[-1] + near_field_extension * path_tangent
    elif target_s >= 0.:
        index = min(int(np.searchsorted(cumulative[1:], target_s, side="right")), len(lengths) - 1)
        portion = (target_s - cumulative[index]) / lengths[index]
        target = points[index] + portion * delta[index]
    if not -trailing_distance - 0.35 <= target[0] <= -trailing_distance + 0.35 or abs(target[1]) > 0.35:
        raise ValueError("rear visual target lies outside the %.2f m capture-base working envelope" % trailing_distance)
    return target, {
        "rear_visible_path_length_m": float(cumulative[-1]),
        "rear_base_nearest_distance_m": float(math.sqrt(float(distance2.min()))),
        "rear_unobserved_near_field_gap_m": gap_along,
        "rear_near_field_lateral_residual_m": gap_cross,
        "rear_target_near_field_extension_m": near_field_extension,
        "rear_target_distance_behind_base_m": float(trailing_distance),
        "rear_target_b": target.tolist(),
        "rear_target_source": ("rear_rgbd_local_occlusion_estimate" if occlusion_extension > 0.
                               else "rear_rgbd_near_field_tangent_estimate" if near_field_extension > 0.
                               else "rear_rgbd_centerline_and_aligned_depth"),
        "rear_target_directly_observed": occlusion_extension == 0. and near_field_extension == 0.,
        "rear_target_occlusion_extension_m": occlusion_extension,
        "rear_target_max_occlusion_extension_m": max_occlusion_extension,
        "rear_tangent_fit_points": int(np.count_nonzero(neighbourhood)),
        "rear_tangent_b": path_tangent.tolist(),
        **fit_diagnostics,
    }


def quaternion_from_matrix(matrix):
    matrix = np.asarray(matrix, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("rotation matrix must be finite 3x3")
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array((0.25 * scale,
                               (matrix[2, 1] - matrix[1, 2]) / scale,
                               (matrix[0, 2] - matrix[2, 0]) / scale,
                               (matrix[1, 0] - matrix[0, 1]) / scale))
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quaternion = np.array(((matrix[2, 1] - matrix[1, 2]) / scale, 0.25 * scale,
                                   (matrix[0, 1] + matrix[1, 0]) / scale,
                                   (matrix[0, 2] + matrix[2, 0]) / scale))
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quaternion = np.array(((matrix[0, 2] - matrix[2, 0]) / scale,
                                   (matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale,
                                   (matrix[1, 2] + matrix[2, 1]) / scale))
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quaternion = np.array(((matrix[1, 0] - matrix[0, 1]) / scale,
                                   (matrix[0, 2] + matrix[2, 0]) / scale,
                                   (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale))
    return normalize_quaternion(quaternion)


def encoder_tcp_pose(q_arm, urdf_path, tcp_position_link, tcp_rotation_link):
    """Forward-kinematic TCP pose from arm encoders and fixed VD3 tool transform."""
    import torch

    kinematics_path = Path("/home/dong/b2_z1/source/b2_z1/b2_z1/tasks/manager_based/"
                           "b2_z1/mdp/commands/dual_vision_kinematics.py")
    spec = importlib.util.spec_from_file_location("vd5_encoder_kinematics", str(kinematics_path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    fk = module.URDFToolKinematics(urdf_path, device="cpu", tcp_link="link06")
    q = torch.as_tensor(np.asarray(q_arm, dtype=np.float32).reshape(1, 6))
    with torch.inference_mode():
        link_position = fk.position(q)[0].numpy().astype(np.float64)
        rotation = torch.eye(3, dtype=q.dtype)
        for number, _name, index in fk.chain:
            rotation = rotation @ getattr(fk, "origin_R_%d" % number)
            if index is not None:
                axis = getattr(fk, "axis_%d" % number)
                rotation = rotation @ module._axis_rotation(axis, q[:, index])[0]
        link_rotation = rotation.numpy().astype(np.float64)
    tcp_position = np.asarray(tcp_position_link, dtype=np.float64)
    tcp_rotation = np.asarray(tcp_rotation_link, dtype=np.float64)
    return (link_position + link_rotation @ tcp_position,
            quaternion_from_matrix(link_rotation @ tcp_rotation))


class VD5VisualController:
    """Validate paired camera provenance/freshness and create actor commands."""

    def __init__(self, init_tcp_b, init_quat_b, speed=0.20, trailing_distance=0.70,
                 stale_timeout_s=1.0, ramp_s=2.0, max_occlusion_extension=0.0,
                 rear_hold_s=0.0, rear_path_memory_s=0.0):
        self._position = np.asarray(init_tcp_b, dtype=np.float64).copy()
        self._quaternion = normalize_quaternion(init_quat_b)
        self._start_position = self._position.copy()
        self._start_quaternion = self._quaternion.copy()
        self.speed = float(speed)
        self.trailing_distance = float(trailing_distance)
        self.max_occlusion_extension = float(max_occlusion_extension)
        self.stale_timeout_s = float(stale_timeout_s)
        self.ramp_s = float(ramp_s)
        self.rear_hold_s = float(rear_hold_s)
        if not math.isfinite(self.rear_hold_s) or not 0. <= self.rear_hold_s <= .30:
            raise ValueError('rear target hold must be between zero and 0.30 seconds')
        self.rear_path_memory_s = float(rear_path_memory_s)
        if not math.isfinite(self.rear_path_memory_s) or not 0. <= self.rear_path_memory_s <= .8:
            raise ValueError('rear path memory must be between zero and 0.8 seconds')
        if self.rear_path_memory_s and self.rear_hold_s:
            raise ValueError('choose rear path memory or body-relative target hold separately')
        self._path_memory = None
        if self.rear_path_memory_s:
            from vd5_rear_path_memory import RearPathMemory
            self._path_memory = RearPathMemory(self.rear_path_memory_s)
        self._accepted_at = None
        self._accepted_status = None
        self._accepted_pose = None
        if (self._position.shape != (3,) or not np.isfinite(self._position).all()
                or not 0.0 < self.speed <= 0.4 or self.stale_timeout_s <= 0.0 or self.ramp_s <= 0.0):
            raise ValueError("invalid visual controller initialization")
        self._elapsed = 0.0
        self._velocity = np.zeros(3, dtype=np.float64)
        self._generation = None
        self._sequence = -1
        self._capture = None
        self._final_quaternion = quaternion_from_euler_xyz(0.0, math.pi / 2.0 - 0.45, math.pi)
        if float(np.dot(self._start_quaternion, self._final_quaternion)) < 0.0:
            self._final_quaternion = -self._final_quaternion

    def _front_visual_command(self, points_b, plane):
        """Version hook; VD5 keeps the original robust-fit selection unchanged."""
        return front_visual_command(points_b, plane, self.speed)

    def _memory_visual_target(self, points_b, plane):
        """VD5 accepts only directly covered work points from historical paths."""
        target, diagnostics = rear_visual_target(points_b, plane, self.trailing_distance, 0.)
        return (target, diagnostics) if diagnostics['rear_target_directly_observed'] else None

    def _validate(self, packet, now):
        short_rear = (isinstance(packet, dict) and isinstance(packet.get('rear'), dict)
                      and packet['rear'].get('reason') in (
                          'seam centerline is too short', 'geometric seam path is too short',
                          'rear seam target section is too short'))
        allow_memory_loss = self._path_memory is not None and short_rear
        if (not isinstance(packet, dict) or packet.get("schema") != SCHEMA
                or (packet.get("valid") is not True and not allow_memory_loss)):
            reason = str(packet.get('reason', ''))[:300] if isinstance(packet, dict) else ''
            raise ValueError("waiting for a valid paired RGB/RGB-D measurement" + (': ' + reason if reason else ''))
        generation = packet.get("capture_generation")
        streams = {}
        timestamps = []
        sequences = []
        for name in ("front", "rear"):
            stream = packet.get(name)
            if (not isinstance(stream, dict) or (stream.get("valid") is not True
                    and not (name == 'rear' and allow_memory_loss))):
                raise ValueError("%s visual measurement is invalid" % name)
            if stream.get("capture_generation") != generation:
                raise ValueError("front/rear captures have different generations")
            sequence = stream.get("sequence")
            capture = stream.get("capture_timestamp")
            if type(sequence) is not int or sequence < 0 or not isinstance(capture, (int, float)):
                raise ValueError("camera capture identity is incomplete")
            capture = float(capture)
            if not math.isfinite(capture) or not -0.002 <= now - capture <= self.stale_timeout_s:
                raise ValueError("camera capture is stale or from the future")
            streams[name] = stream
            timestamps.append(capture)
            sequences.append(sequence)
        if sequences[0] != sequences[1] or abs(timestamps[0] - timestamps[1]) > 0.04:
            raise ValueError("front RGB and rear RGB-D captures are not paired")
        if type(generation) is not int or generation < 0:
            raise ValueError("invalid capture generation")
        if self._generation is not None:
            if generation < self._generation or (generation == self._generation and sequences[0] < self._sequence):
                raise ValueError("camera capture identity moved backwards")
            if (generation == self._generation and sequences[0] == self._sequence
                    and tuple(timestamps) != self._capture):
                raise ValueError("repeated frame sequence changed its timestamp")
        # The front path uses the rear depth plane only for metric projection.
        front = streams["front"]
        rear = streams["rear"]
        plane = _plane(rear.get("ground_plane_b"))
        if front.get("ground_plane_b") is not None:
            front_plane = _plane(front["ground_plane_b"])
            if not np.allclose(front_plane, plane, atol=1.0e-4):
                raise ValueError("front projection plane differs from paired rear depth plane")
        front_command, front_diag = self._front_visual_command(front.get("points_b"), plane)
        if self._path_memory is not None:
            memory = self._path_memory
            if rear.get('valid') is True:
                memory.ingest(_finite_points(rear.get('points_b')), plane, _ground_basis(plane),
                              generation, sequences[1], timestamps[1], now)
                # Prefer a currently covered work point. Endpoint extrapolation
                # is a fallback after checking previously observed support.
                try:
                    target, rear_diag = rear_visual_target(rear['points_b'], plane,
                                                          self.trailing_distance, 0.)
                    if rear_diag['rear_target_directly_observed']:
                        rear_diag['rear_memory_used'] = False
                        return (generation, sequences[0], tuple(timestamps), front_command,
                                front_diag, target, rear_diag)
                except ValueError:
                    pass
            # A new capture generation cannot consume points from its predecessor.
            if memory.identity is not None and memory.identity[0] == generation:
                for points, diagnostics in memory.paths(plane, _ground_basis(plane), now):
                    try:
                        selected = self._memory_visual_target(points, plane)
                    except ValueError:
                        continue
                    if selected is not None:
                        target, rear_diag = selected
                        rear_diag.update(diagnostics, rear_memory_used=True,
                                         rear_target_source='rear_rgbd_observed_path_command_propagation',
                                         rear_target_directly_observed=False)
                        return (generation, sequences[0], tuple(timestamps), front_command,
                                front_diag, target, rear_diag)
            if rear.get('valid') is not True:
                raise ValueError('rear path memory has no observed support for the work point')
        target, rear_diag = rear_visual_target(rear.get("points_b"), plane, self.trailing_distance,
                                              self.max_occlusion_extension)
        rear_diag['rear_memory_used'] = False
        return generation, sequences[0], tuple(timestamps), front_command, front_diag, target, rear_diag

    def _rear_hold(self, packet, now):
        """Bridge short rear-geometry losses; preserve original measurement identity.

        The held target is a body-relative setpoint, not a re-observed ground point.
        No odometry, timestamps, or image geometry are synthesized.
        """
        if (self.rear_hold_s == 0. or self._accepted_at is None
                or self._elapsed < self.ramp_s
                or now-self._accepted_at > self.rear_hold_s
                or not isinstance(packet, dict) or packet.get('schema') != SCHEMA
                or packet.get('capture_generation') != self._generation):
            return None
        front, rear = packet.get('front', {}), packet.get('rear', {})
        if (not isinstance(front, dict) or front.get('valid') is not True
                or not isinstance(rear, dict)):
            return None
        captures = []
        for stream, previous_stamp in zip((front, rear), self._capture):
            stamp, sequence = stream.get('capture_timestamp'), stream.get('sequence')
            if (not isinstance(stamp, (int, float)) or not math.isfinite(stamp)
                    or not -.002 <= now-stamp <= self.stale_timeout_s
                    or stream.get('capture_generation') != self._generation
                    or type(sequence) is not int or sequence < self._sequence
                    or (sequence == self._sequence and stamp != previous_stamp)):
                return None
            captures.append(stamp)
        if (front['sequence'] != rear['sequence'] or abs(captures[0]-captures[1]) > .04
                or self._capture is None or now-min(self._capture) > self.stale_timeout_s):
            return None
        # Never override depth, encoder, time, or calibration failures.
        if rear.get('valid') is not True and rear.get('reason') not in (
                'seam centerline is too short', 'geometric seam path is too short',
                'rear seam target section is too short'):
            return None
        plane = _plane(rear.get('ground_plane_b'))
        if front.get('ground_plane_b') is not None and not np.allclose(
                _plane(front['ground_plane_b']), plane, atol=1.e-4):
            return None
        command, diagnostics = front_visual_command(front.get('points_b'), plane, self.speed)
        # Stable ground/heading are required for a brief body-relative hold.
        old = self._accepted_status
        if (np.linalg.norm(plane-old['ground_plane_b']) > .03
                or abs(diagnostics['heading_error_rad']-old['heading_error_rad']) > .10
                or abs(diagnostics['curvature_m_inv']) > .20
                or np.linalg.norm(self._position-self._accepted_pose) > .01):
            return None
        diagnostics.update(control_state='rear_hold', measurement_valid=False,
                           rear_target_held=True, rear_target_source='held_body_relative_setpoint',
                           rear_target_directly_observed=False,
                           rear_hold_elapsed_s=now-self._accepted_at,
                           rear_held_capture_age_s=now-min(self._capture),
                           rear_held_source_sequence=self._sequence,
                           rear_target_b=self._position.tolist(),
                           front_source_sequence=front['sequence'])
        return command, diagnostics

    def update(self, packet, now, dt):
        now, dt = float(now), float(dt)
        if not math.isfinite(now) or not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("control time must be finite and dt positive")
        if self._path_memory is not None:
            self._path_memory.advance(now, dt, self._velocity)
        desired = np.zeros(3, dtype=np.float64)
        status = {"valid": False, "reason": "waiting for paired camera frames",
                  "command_source": "front_rgb_and_rear_rgbd_only",
                  "uses_world_pose": False, "uses_simulator_seam": False}
        # Diagnostic only: a new rejected image is different from a dead camera.
        # capture_age_s is the last accepted packet's age. Historical target
        # source ages are recorded separately and never renewed by this packet.
        if isinstance(packet, dict):
            latest = [stream.get('capture_timestamp') if isinstance(stream, dict) else None
                      for stream in (packet.get('front'), packet.get('rear'))]
            if all(isinstance(value, (int, float)) and math.isfinite(value) for value in latest):
                status['latest_packet_capture_age_s'] = now-min(latest)
        try:
            generation, sequence, captures, command, front_diag, target, rear_diag = self._validate(packet, now)
            if generation != self._generation:
                self._elapsed = 0.0
                self._position = self._start_position.copy()
                self._quaternion = self._start_quaternion.copy()
            self._generation, self._sequence, self._capture = generation, sequence, captures
            self._elapsed += dt
            blend = startup_fraction(self._elapsed, self.ramp_s)
            desired[:] = command * blend
            alpha = 1.0 - math.exp(-dt / 0.30)
            goal = (1.0 - blend) * self._start_position + blend * target
            delta = alpha * (goal - self._position)
            limit = 0.15 * dt
            norm = float(np.linalg.norm(delta))
            if norm > limit:
                delta *= limit / norm
            self._position += delta
            self._quaternion = self._start_quaternion + blend * (self._final_quaternion - self._start_quaternion)
            self._quaternion = normalize_quaternion(self._quaternion)
            status.update(front_diag)
            status.update(rear_diag)
            from_memory = rear_diag.get('rear_memory_used', False)
            status.update(valid=True, measurement_valid=not from_memory,
                          control_state='rear_memory' if from_memory else 'tracking',
                          rear_target_held=False,
                          reason=('previously observed rear path; sent-command propagation' if from_memory
                                  else 'paired RGB/RGB-D centerlines valid'), startup_blend=blend)
            if not from_memory:
                self._accepted_at = now
                self._accepted_status = dict(status)
                self._accepted_pose = self._position.copy()
        except (KeyError, TypeError, ValueError, FloatingPointError) as error:
            status["reason"] = str(error)
            status.update(measurement_valid=False, control_state='braking')
            try:
                held = self._rear_hold(packet, now)
            except (KeyError, TypeError, ValueError, FloatingPointError):
                held = None
            if held is not None:
                command, diagnostics = held
                desired[:] = command
                status.update(diagnostics, valid=True,
                              reason='brief rear target hold: '+str(error))
        self._velocity += (1.0 - math.exp(-dt / 0.20)) * (desired - self._velocity)
        if not status["valid"] and np.linalg.norm(self._velocity) < 1.0e-5:
            self._velocity[:] = 0.0
        self._velocity[1] = 0.0
        status.update(capture_generation=self._generation, source_sequence=self._sequence,
                      capture_age_s=None if self._capture is None else now - min(self._capture))
        pose = np.concatenate((self._position, self._quaternion))
        return {"base_command_b": self._velocity.tolist(), "tool_pose_b": pose.tolist(),
                "status": status}
