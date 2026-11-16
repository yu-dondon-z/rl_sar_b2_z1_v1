"""VD1 image-derived commands and the upstream 70-value actor interface.

This module deliberately has no simulator dependency.  Coordinates remain in
the calibrated camera-capture base frame until a newer image is received;
sample holding is not a world-pose or visual-odometry estimate.  The initial
TCP must come from encoder FK, never a simulator link pose.
"""

from __future__ import annotations

import math
from itertools import combinations
from numbers import Real

import numpy as np


SCHEMA = "b2_z1_dual_vision_v1"
WORKSPACE_LOW = np.array([-1.08, -.15, -.34])
WORKSPACE_HIGH = np.array([-.96, .15, -.26])


def _array(value, shape=None):
    result = np.asarray(value, dtype=np.float64)
    if (shape is not None and result.shape != shape) or not np.isfinite(result).all():
        raise ValueError("nonfinite or incorrectly shaped input")
    return result


def _check_payload(value):
    """Fail closed on truth fields, including fields hidden inside metadata."""
    if isinstance(value, dict):
        for key, item in value.items():
            name = str(key).lower()
            forbidden = (
                name.endswith("_w") or "world" in name or "privileged" in name
                or "ground_truth" in name or "true_tcp" in name or "tcp_true" in name
                or name.startswith("evaluation_only") or name.startswith("body_state")
                or name.startswith("root_pos") or name.startswith("root_quat")
                or name.startswith(("base_position", "base_quaternion", "capture_base_",
                                    "current_base_", "actual_tcp", "tcp_actual", "true_path"))
            )
            if forbidden:
                raise ValueError(f"privileged field is forbidden: {key}")
            _check_payload(item)
    elif isinstance(value, (list, tuple)):
        for item in value:
            _check_payload(item)
    elif isinstance(value, np.ndarray):
        if not np.isfinite(value).all():
            raise ValueError("nonfinite packet array")
    elif isinstance(value, Real) and not isinstance(value, bool) and not math.isfinite(value):
        raise ValueError("nonfinite packet number")


def _points(value):
    points = _array(value)
    if points.ndim != 2 or points.shape[1] != 3 or not 3 <= len(points) <= 4096:
        raise ValueError("visual path requires 3..4096 xyz points")
    if np.max(np.abs(points)) > 20 or np.linalg.norm(np.ptp(points[:, :2], axis=0)) < .02:
        raise ValueError("degenerate or unbounded visual path")
    return points


def _tool_quaternion():
    # Fixed-axis RPY (0, pi/2-.45, pi), quaternion in Isaac/actor wxyz order.
    half_pitch = .5 * (math.pi / 2 - .45)
    half_yaw = math.pi / 2
    return np.array([math.cos(half_pitch) * math.cos(half_yaw),
                     -math.sin(half_pitch) * math.sin(half_yaw),
                     math.sin(half_pitch) * math.cos(half_yaw),
                     math.cos(half_pitch) * math.sin(half_yaw)])


def _front_geometry(points):
    points = points[points[:, 0] > .10]
    points = points[np.argsort(points[:, 0], kind="stable")]
    if len(points) < 3 or np.ptp(points[:, 0]) < .30:
        raise ValueError("front seam needs at least 0.30 m of visible support")
    # A centreline may end in a tiny pixel hook.  Its first millimetre-long
    # segment must not determine a metre-scale steering error.  Fit a local
    # visible section, without any world pose, nominal seam, or future path.
    local = points[points[:, 0] <= points[0, 0] + 1.0]
    if len(local) < 3 or np.ptp(local[:, 0]) < .30:
        raise ValueError("front seam has insufficient local fitting support")
    x0, scale = float(local[0, 0]), float(np.ptp(local[:, 0]))
    t = (local[:, 0] - x0) / scale
    y = local[:, 1]
    design = np.column_stack((t*t, t, np.ones_like(t)))
    # Deterministic least-median fitting tolerates minority endpoint hooks.
    # A fixed, spread subset bounds the work even for dense image polylines.
    samples = np.unique(np.linspace(0, len(local)-1, min(10, len(local))).astype(int))
    best, best_score = None, float("inf")
    for indices in combinations(samples, 3):
        if min(np.diff(t[list(indices)])) < .08:
            continue
        candidate = np.linalg.solve(design[list(indices)], y[list(indices)])
        score = float(np.median(np.abs(design @ candidate - y)))
        if score < best_score:
            best, best_score = candidate, score
    if best is None:
        raise ValueError("front seam lacks separated fitting samples")
    tolerance = max(.0025, 3.5 * 1.4826 * best_score)
    inliers = np.abs(design @ best - y) <= tolerance
    for _ in range(3):
        if np.count_nonzero(inliers) < max(3, (len(local)+1)//2) or np.ptp(local[inliers, 0]) < .30:
            raise ValueError("front seam has insufficient robust inlier support")
        best = np.linalg.lstsq(design[inliers], y[inliers], rcond=None)[0]
        updated = np.abs(design @ best - y) <= tolerance
        if np.array_equal(updated, inliers):
            break
        inliers = updated
    a, b, c = best
    reference_x = float(local[inliers, 0].min())
    reference_t = (reference_x - x0) / scale
    reference_y = a*reference_t**2 + b*reference_t + c
    slope = float((2*a*reference_t + b) / scale)
    tangent_norm = math.sqrt(1 + slope*slope)
    if 1 / tangent_norm < .35:
        raise ValueError("front seam has no usable forward tangent")
    heading = float(math.atan(slope))
    # Existing path_guidance reports path-left offset.  The minus sign in
    # the VD1 steering equation requires the opposite, robot-left error.
    path_offset = float((reference_y - slope*reference_x) / tangent_norm)
    robot_lateral = -path_offset
    curvature = float(np.clip((2*a/scale**2) / tangent_norm**3, -2.0, 2.0))
    reference_z = float(local[inliers, 2][0])
    lookahead = np.array([reference_x, reference_y, reference_z], dtype=np.float64)
    return heading, robot_lateral, path_offset, curvature, lookahead


def _rear_geometry(points, tcp, plane):
    normal_length = float(np.linalg.norm(plane[:3]))
    if not .5 <= normal_length <= 2:
        raise ValueError("invalid observed ground plane")
    plane = plane / normal_length
    if abs(plane[2]) < .4:
        raise ValueError("ground is not observable along base vertical")
    if plane[2] < 0:
        plane = -plane
    ordered = points[np.argsort(points[:, 0], kind="stable")]
    x = -1.02
    if x < ordered[0, 0] or x > ordered[-1, 0]:
        raise ValueError("visible rear seam does not cover x=-1.02 m")
    y = float(np.interp(x, ordered[:, 0], ordered[:, 1]))
    nominal = np.array([x, y, -.30])
    feedback = .35 * (nominal - tcp)
    feedback[0] = 0.0  # Preserve the upstream policy's fixed rear working x.
    norm = float(np.linalg.norm(feedback))
    if norm > .03:
        feedback *= .03 / norm
    raw = nominal + feedback
    target = np.clip(raw, WORKSPACE_LOW, WORKSPACE_HIGH)
    # Only the measured RGB-D plane is used.  This is a normal projection;
    # no known world ground elevation or base height is substituted.
    projection = tcp - plane[:3] * (plane[:3] @ tcp + plane[3])
    seam = points - (points @ plane[:3] + plane[3])[:, None] * plane[:3]
    starts, delta = seam[:-1], np.diff(seam, axis=0)
    fraction = np.sum((projection - starts) * delta, axis=1) / np.maximum(np.sum(delta*delta, axis=1), 1.e-12)
    nearest = starts + np.clip(fraction, 0, 1)[:, None] * delta
    projection_error = float(np.linalg.norm(nearest - projection, axis=1).min())
    return target, {
        "tcp_projection_error_m": projection_error,
        "tcp_height_above_measured_plane_m": float(plane[:3] @ tcp + plane[3]),
        "tcp_target_error_m": float(np.linalg.norm(nominal - tcp)),
        "rear_nominal_target_b": nominal.tolist(),
        "rear_feedback_b": feedback.tolist(),
        "target_clipped": bool(np.any(np.abs(raw - target) > 1.e-12)),
    }


class VD1VisualController:
    """Pure visual outer loop, with smooth startup and genuine stale stopping."""

    def __init__(self, init_tcp_b, forward_speed=.2, stale_timeout_s=1.0, ramp_s=2):
        self._tool_position = _array(init_tcp_b, (3,)).copy()
        self._tool_quat = _tool_quaternion()
        self.forward_speed = float(forward_speed)
        self.stale_timeout_s = float(stale_timeout_s)
        self.ramp_s = float(ramp_s)
        if (not all(math.isfinite(v) for v in (self.forward_speed, self.stale_timeout_s, self.ramp_s))
                or not 0 < self.forward_speed <= 1 or self.stale_timeout_s <= 0 or self.ramp_s < 0):
            raise ValueError("invalid speed, frame age, or startup duration")
        self._velocity = np.zeros(3)
        self._tracking_elapsed = 0.0
        self._generation = None
        self._sequence = -1
        self._capture_times = None

    def _validate(self, packet, now):
        if not isinstance(packet, dict):
            raise ValueError("waiting for paired visual measurements")
        _check_payload(packet)
        if packet.get("schema") != SCHEMA or packet.get("valid") is not True:
            raise ValueError(str(packet.get("reason", "invalid dual-camera packet")))
        generation = packet.get("capture_generation")
        if type(generation) is not int or generation < 0:
            raise ValueError("invalid capture generation")
        streams, capture_times, sequences = {}, [], []
        for name in ("front", "rear"):
            stream = packet.get(name)
            if not isinstance(stream, dict) or stream.get("valid") is not True:
                raise ValueError(f"{name} visual measurement is invalid")
            if stream.get("capture_generation") != generation:
                raise ValueError("paired streams have different reset generations")
            sequence = stream.get("sequence")
            if type(sequence) is not int or sequence < 0:
                raise ValueError("invalid visual sequence")
            capture = stream.get("capture_timestamp")
            if not isinstance(capture, Real) or isinstance(capture, bool) or not math.isfinite(capture):
                raise ValueError("missing finite monotonic capture timestamp")
            if not -.002 <= now - capture <= self.stale_timeout_s:
                raise ValueError(f"{name} capture is stale or in the future")
            streams[name] = (stream, _points(stream.get("points_b")))
            capture_times.append(float(capture))
            sequences.append(sequence)
        if sequences[0] != sequences[1] or abs(capture_times[0] - capture_times[1]) > .04:
            raise ValueError("camera captures are not synchronized")
        if self._generation is not None:
            if generation < self._generation:
                raise ValueError("capture generation moved backwards")
            if generation == self._generation:
                if sequences[0] < self._sequence:
                    raise ValueError("visual sequence moved backwards")
                if sequences[0] == self._sequence and tuple(capture_times) != self._capture_times:
                    raise ValueError("repeated sequence cannot refresh capture timestamp")
                if sequences[0] > self._sequence and min(capture_times) < min(self._capture_times):
                    raise ValueError("capture time moved backwards")
        rear, rear_points = streams["rear"]
        target, diagnostics = _rear_geometry(rear_points, _array(rear.get("tcp_measured_b"), (3,)),
                                             _array(rear.get("ground_plane_b"), (4,)))
        geometry = _front_geometry(streams["front"][1])
        return generation, sequences[0], tuple(capture_times), target, diagnostics, geometry

    def update(self, packet, now, dt):
        """Return Python lists and JSON-safe status; rejected frames never move the target."""
        now, dt = float(now), float(dt)
        if not math.isfinite(now) or not math.isfinite(dt) or dt <= 0:
            raise ValueError("now must be finite and dt must be finite and positive")
        desired_velocity = np.zeros(3)
        status = {"valid": False, "reason": "waiting for images", "target_clipped": False,
                  "geometry_mode": "capture_base_frame_sample_hold", "uses_world_pose": False,
                  "tcp_projection_error_m": None}
        try:
            generation, sequence, captures, target, diagnostics, geometry = self._validate(packet, now)
            if generation != self._generation:
                self._tracking_elapsed = 0.0
            self._generation, self._sequence, self._capture_times = generation, sequence, captures
            heading, lateral, path_offset, curvature, lookahead = geometry
            self._tracking_elapsed += dt
            ratio = 1.0 if self.ramp_s == 0 else min(self._tracking_elapsed / self.ramp_s, 1.0)
            blend = ratio * ratio * (3 - 2 * ratio)
            # Pure pursuit aims at a fitted point inside the observed section.
            # A distant visible tangent's intercept is not a measured error at
            # the base; using it as cross-track feedback reverses some curves.
            # 2*v*y/(x*x+y*y) already includes arc curvature; add no second
            # curvature feedforward term.
            planar_distance_squared = float(lookahead[0]**2 + lookahead[1]**2)
            yaw = float(np.clip(2 * self.forward_speed * lookahead[1]
                                / max(planar_distance_squared, 1.e-6), -.8, .8))
            desired_velocity[:] = [self.forward_speed * blend, 0.0, yaw * blend]
            # No world-position stabilization: interpolate only local targets.
            delta = (1 - math.exp(-dt / .30)) * (target - self._tool_position)
            max_step = .15 * dt
            norm = float(np.linalg.norm(delta))
            if norm > max_step:
                delta *= max_step / norm
            self._tool_position += delta
            status.update(diagnostics, valid=True, reason="paired visual measurements valid",
                          front_heading_error_rad=heading, robot_lateral_error_m=lateral,
                          path_left_offset_m=path_offset, front_curvature_m_inv=curvature,
                          front_lookahead_b=lookahead.tolist(), front_steering_method="visible_point_pure_pursuit",
                          front_diagnostics_reference="visible_tangent_not_base_pose",
                          requested_yaw_rate_rps=yaw, startup_blend=blend)
        except (KeyError, TypeError, ValueError, FloatingPointError) as error:
            self._tracking_elapsed = 0.0
            status["reason"] = str(error)
        self._velocity += (1 - math.exp(-dt / .20)) * (desired_velocity - self._velocity)
        if not status["valid"] and np.linalg.norm(self._velocity) < 1.e-5:
            self._velocity[:] = 0
        self._velocity[1] = 0.0
        status.update(capture_generation=self._generation, source_sequence=self._sequence,
                      capture_age_s=None if self._capture_times is None else now - min(self._capture_times),
                      target_transition_outside_workspace=bool(np.any(self._tool_position < WORKSPACE_LOW)
                                                              or np.any(self._tool_position > WORKSPACE_HIGH)))
        return {"base_command_b": self._velocity.tolist(),
                "tool_pose_b": np.concatenate((self._tool_position, self._tool_quat)).tolist(),
                "status": status}


def build_actor_observation(base_command_b, tool_pose_b, omega_b, gravity_b,
                            q, qd, default_q, last_action):
    """Match upstream term order and Isaac's clip-then-scale convention.

    The 18 encoder/action values must already be in the upstream articulation
    order.  qd uses the upstream zero default velocity.  No critic observation,
    base linear velocity, global pose, or actual TCP is read here.
    """
    base = _array(base_command_b, (3,))
    tool = _array(tool_pose_b, (7,))
    if abs(float(np.linalg.norm(tool[3:])) - 1) > 1.e-3:
        raise ValueError("tool quaternion must be unit wxyz")
    omega, gravity = _array(omega_b, (3,)), _array(gravity_b, (3,))
    positions, velocities = _array(q, (18,)), _array(qd, (18,))
    defaults, actions = _array(default_q, (18,)), _array(last_action, (18,))
    terms = [np.clip(base, -100, 100), np.clip(tool, -100, 100),
             np.clip(omega, -100, 100) * .2, np.clip(gravity, -100, 100),
             np.clip(positions - defaults, -100, 100),
             np.clip(velocities, -100, 100) * .05, np.clip(actions, -100, 100)]
    observation = np.concatenate(terms).astype(np.float32)
    if observation.shape != (70,):
        raise AssertionError("upstream VD1 actor observation must have 70 values")
    return observation
