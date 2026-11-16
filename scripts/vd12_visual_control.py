"""VD12 commands from paired image-derived paths and a measured depth plane.

The actor interface stays [base velocity (3), TCP pose in base (7)]. No world
pose, simulator seam, contact state, or privileged geometry is an input here.
The caller obtains the initial TCP from encoder FK and supplies measurements
from the existing front-RGB/rear-RGB-D image pipeline, not ideal seam points.

The two-second quintic startup, normalized quaternion interpolation, working
pitch, and finite-cylinder rim correction follow VD10SafeSeamCommand at full
ground curriculum. Unlike that training teacher, the initial TCP is in the
base frame: this controller has no world odometry with which to anchor it in
world. Plane-normal lifting generalizes the teacher's horizontal-world-Z
correction to the measured surface. There is no extra target low-pass,
Cartesian speed limiter, velocity low-pass, or setpoint hold. The original
VD5 bounded near-field tangent estimate may bridge up to 0.25 m of camera/body
occlusion, explicitly labelled as an estimate; configure
max_near_field_extension_m=0 for directly observed work points only. Existing
VD5 command-propagated observed path memory and bounded far-end fit are
optional and default off. Neither uses world pose or measured odometry; the
memory's source capture ages never become the age of a newer camera packet.
"""

from __future__ import annotations

import hashlib
import math
from itertools import combinations

import numpy as np

from vd5_mujoco_control import (normalized_lerp_quaternion, quaternion_matrix,
                               startup_fraction)
from vd5_rear_path_memory import RearPathMemory
from vd5_visual_control import (VD5VisualController, _front_geometry, _plane,
                                front_visual_command, rear_visual_target,
                                RearTargetSupportShortfall)


def supported_front_geometry(points, diagnostics=None):
    """Try ranked fits if the least-median winner lacks the existing support.

    Keep successful legacy results exactly unchanged. A failed candidate must
    not veto other fits with enough measured inliers and 0.30 m of support.
    Every alternative uses the ORIGINAL winner's residual tolerance, never a
    larger tolerance derived from an increasingly poor candidate. No pixels,
    path points, or support thresholds are invented or extended here.
    """
    try:
        result = _front_geometry(points)
        if diagnostics is not None:
            diagnostics.update(front_fit_candidate_fallback=False, front_fit_candidate_rank=1)
        return result
    except ValueError as error:
        if str(error) != 'front seam has insufficient robust inlier support':
            raise
    points = np.asarray(points, dtype=np.float64)
    points = points[points[:, 0] > .10]
    points = points[np.argsort(points[:, 0], kind='stable')]
    local = points[points[:, 0] <= points[0, 0] + 1.0]
    x0, scale = float(local[0, 0]), float(np.ptp(local[:, 0]))
    t, y = (local[:, 0] - x0) / scale, local[:, 1]
    design = np.column_stack((t*t, t, np.ones_like(t)))
    samples = np.unique(np.linspace(0, len(local)-1, min(10, len(local))).astype(int))
    candidates = []
    for indices in combinations(samples, 3):
        if min(np.diff(t[list(indices)])) < .08:
            continue
        fit = np.linalg.solve(design[list(indices)], y[list(indices)])
        score = float(np.median(np.abs(design @ fit - y)))
        candidates.append((score, fit))
    candidates.sort(key=lambda item: item[0])
    if not candidates:
        raise ValueError('front seam lacks separated fitting samples')
    tolerance = max(.0025, 3.5 * 1.4826 * candidates[0][0])
    required_count = max(3, (len(local)+1)//2)

    def supported(mask):
        return (np.count_nonzero(mask) >= required_count
                and np.ptp(local[mask, 0]) >= .30)

    for rank, (_, fit) in enumerate(candidates, 1):
        inliers = np.abs(design @ fit - y) <= tolerance
        for _ in range(3):
            if not supported(inliers):
                break
            fit = np.linalg.lstsq(design[inliers], y[inliers], rcond=None)[0]
            updated = np.abs(design @ fit - y) <= tolerance
            if np.array_equal(updated, inliers):
                break
            inliers = updated
        if not supported(inliers):
            continue
        a, b, c = fit
        reference_x = float(local[inliers, 0].min())
        reference_t = (reference_x - x0) / scale
        reference_y = a*reference_t**2 + b*reference_t + c
        slope = float((2*a*reference_t + b) / scale)
        tangent_norm = math.sqrt(1 + slope*slope)
        if 1 / tangent_norm < .35:
            continue
        heading = float(math.atan(slope))
        path_offset = float((reference_y - slope*reference_x) / tangent_norm)
        curvature = float(np.clip((2*a/scale**2) / tangent_norm**3, -2., 2.))
        reference = np.array([reference_x, reference_y, float(local[inliers, 2][0])])
        if diagnostics is not None:
            diagnostics.update(front_fit_candidate_fallback=True, front_fit_candidate_rank=rank,
                               front_fit_residual_tolerance_m=tolerance,
                               front_fit_inlier_count=int(np.count_nonzero(inliers)),
                               front_fit_inlier_span_m=float(np.ptp(local[inliers, 0])))
        return heading, -path_offset, path_offset, curvature, reference
    raise ValueError('front seam has insufficient robust inlier support')


class _ObservedEnvelopeRearPathMemory(RearPathMemory):
    """Optional cache accepts only measured points inside its existing envelope.

    A distant irrelevant tail must not veto a valid current-image work point.
    No point is moved or invented, and the current target still uses the full
    original image path. The inherited paths() keeps its 25 mm gap splitting;
    discarded out-of-envelope samples cannot become a connecting segment.
    """

    def ingest(self, points, plane, basis, generation, sequence, stamp, now):
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError('rear memory requires measured xyz points')
        origin = -plane[3] * plane[:3]
        xy = (points - origin) @ basis[:, :2]
        keep = np.abs(xy[:, 1]) <= .35
        if np.count_nonzero(keep) < 6 or np.ptp(xy[keep, 0]) < .10:
            # Not a cached observation: do not assign an identity or a motion
            # pose to unsupported data. Current-image validation is separate.
            return
        super().ingest(points[keep], plane, basis, generation, sequence, stamp, now)


def cylinder_endface_height(axis_b, ground_plane_b, *, length_m=1.1,
                           radius_m=.028, clearance_m=.005):
    """Required front end-face centre height along the measured plane normal.

    The axis points rear-to-TCP. An upward axis lowers the rear by L*axis.n;
    the end-face rim extends by r*sqrt(1-(axis.n)^2). This is geometric
    clearance only, not a contact-force or penetration measurement.
    """
    axis = np.asarray(axis_b, dtype=np.float64)
    if axis.shape != (3,) or not np.isfinite(axis).all():
        raise ValueError('gun axis must be finite xyz')
    norm = float(np.linalg.norm(axis))
    if norm < 1.e-12:
        raise ValueError('gun axis must be nonzero')
    dimensions = (float(length_m), float(radius_m), float(clearance_m))
    if (not all(math.isfinite(value) for value in dimensions)
            or dimensions[0] <= 0. or dimensions[1] <= 0. or dimensions[2] < 0.):
        raise ValueError('tube dimensions must be positive and clearance nonnegative')
    normal = _plane(ground_plane_b)[:3]
    dot = float(np.clip(axis @ normal / norm, -1., 1.))
    return (max(dimensions[0] * dot, 0.)
            + dimensions[1] * math.sqrt(max(1. - dot * dot, 0.)) + dimensions[2])


class VD12VisualController(VD5VisualController):
    """Fail-closed image-only VD12 command adapter, never a truth fallback.

    ``update(packet, now, dt)`` returns ``base_command_b``, ``tool_pose_b``
    and ``status``, matching VD5's shape. ``now`` and capture timestamps must
    share the capture clock. The runtime must end the episode on the first
    ``status.valid is False``. A failure is latched; a new controller with new
    encoder FK is required for a deliberate reset, never an automatic restart.
    """

    def __init__(self, init_tcp_b, init_quat_b, speed=.20, trailing_distance=.80,
                 stale_timeout_s=.15, ramp_s=2., tube_length_m=1.1,
                 tube_outer_radius_m=.028, working_clearance_m=.005,
                 max_near_field_extension_m=.25, rear_path_memory_s=0.,
                 max_occlusion_extension=0., historical_tangent_estimate=False):
        values = (float(speed), float(trailing_distance), float(stale_timeout_s),
                  float(ramp_s), float(tube_length_m), float(tube_outer_radius_m),
                  float(working_clearance_m), float(max_near_field_extension_m),
                  float(rear_path_memory_s), float(max_occlusion_extension))
        if (not all(math.isfinite(value) for value in values)
                or values[1] <= 0. or values[2] <= 0. or values[3] <= 0.
                or values[4] <= 0. or values[5] <= 0. or values[6] < 0.
                or not 0. <= values[7] <= .25 or not 0. <= values[8] <= .8
                or not 0. <= values[9] <= .05):
            raise ValueError('invalid VD12 visual controller configuration')
        if not math.isclose(values[3], 2., rel_tol=0., abs_tol=1.e-12):
            raise ValueError('VD12 checked startup must match the trained two-second ramp')
        super().__init__(init_tcp_b, init_quat_b, speed=values[0],
                         trailing_distance=values[1], stale_timeout_s=values[2],
                         ramp_s=values[3], max_occlusion_extension=values[9],
                         rear_hold_s=0., rear_path_memory_s=values[8])
        self.tube_length_m = values[4]
        self.tube_outer_radius_m = values[5]
        self.working_clearance_m = values[6]
        self.max_near_field_extension_m = values[7]
        if type(historical_tangent_estimate) is not bool:
            raise ValueError('historical tangent estimate must be an explicit boolean')
        self.historical_tangent_estimate = historical_tangent_estimate
        if self._path_memory is not None:
            self._path_memory = _ObservedEnvelopeRearPathMemory(self.rear_path_memory_s)
        self._failed_reason = None
        self._last_now = None
        self._payload_digest = None

    def _front_visual_command(self, points_b, plane):
        fit_diagnostics = {}
        command, diagnostics = front_visual_command(points_b, plane, self.speed,
            geometry_fn=lambda points: supported_front_geometry(points, fit_diagnostics))
        diagnostics.update(fit_diagnostics)
        diagnostics['front_fit_policy'] = 'ranked_candidates_original_residual_and_support_bounds'
        return command, diagnostics

    def _memory_visual_target(self, points_b, plane):
        """Optional bounded tangent of actual historical support, never a hold.

        In addition to all memory age/motion/gap rules, require the same 8
        points / 12cm / 8mm P90 / 15mm maximum residual as the current-image
        far-end fit. The original 25cm near-field limit is not increased.
        """
        target, diagnostics = rear_visual_target(points_b, plane, self.trailing_distance, 0.)
        if diagnostics['rear_target_directly_observed']:
            return target, diagnostics
        extension = float(diagnostics['rear_target_near_field_extension_m'])
        if (not self.historical_tangent_estimate or extension <= 0.
                or extension > self.max_near_field_extension_m):
            return None
        points = np.asarray(points_b, dtype=float)
        if points[-1, 0] < points[0, 0]:
            points = points[::-1]
        station = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
        selected = station >= station[-1]-.15
        if np.count_nonzero(selected) < 8 or np.ptp(station[selected]) < .12:
            return None
        design = np.c_[np.ones(np.count_nonzero(selected)), station[selected]]
        fit = np.linalg.lstsq(design, points[selected], rcond=None)[0]
        residual = np.linalg.norm(points[selected]-design @ fit, axis=1)
        if np.percentile(residual, 90) > .008 or residual.max() > .015:
            return None
        # Generate the estimate from the very fit that passed validation.
        # Anchoring the old tangent at the raw final pixel preserves an
        # endpoint hook even when the least-squares fit correctly rejects it.
        direction = fit[1]
        scale = float(np.linalg.norm(direction))
        if not .8 <= scale <= 1.05:
            return None
        direction = direction / scale
        if direction[0] < .7:
            return None
        endpoint = fit[0] + station[-1]*fit[1]
        normalized_plane = _plane(plane)
        projected_base = -normalized_plane[3]*normalized_plane[:3]
        extension = float((projected_base-endpoint) @ direction - self.trailing_distance)
        target = endpoint + extension*direction
        actual_extension = float(np.linalg.norm(target-points[-1]))
        if (extension <= 0. or actual_extension > self.max_near_field_extension_m
                or not -self.trailing_distance-.35 <= target[0] <= -self.trailing_distance+.35
                or abs(target[1]) > .35):
            return None
        diagnostics.update(rear_memory_tangent_estimated=True,
            rear_target_b=target.tolist(), rear_target_near_field_extension_m=actual_extension,
            rear_tangent_b=direction.tolist(), rear_memory_tangent_fitted_endpoint_b=endpoint.tolist(),
            rear_memory_tangent_fit_scale=scale,
            rear_memory_tangent_fit_extension_m=extension,
            rear_memory_tangent_fit_points=int(np.count_nonzero(selected)),
            rear_memory_tangent_fit_span_m=float(np.ptp(station[selected])),
            rear_memory_tangent_fit_residual_p90_m=float(np.percentile(residual, 90)),
            rear_memory_tangent_fit_residual_max_m=float(residual.max()))
        return target, diagnostics

    @staticmethod
    def _digest(packet):
        """A repeated identity cannot silently acquire different geometry."""
        digest = hashlib.sha256()
        for name in ('front', 'rear'):
            stream = packet[name]
            for field in ('points_b', 'ground_plane_b'):
                value = stream.get(field)
                if value is None:
                    digest.update(b'none')
                else:
                    array = np.asarray(value, dtype='<f8')
                    digest.update(repr(array.shape).encode('ascii'))
                    digest.update(array.tobytes())
        return digest.hexdigest()

    @staticmethod
    def _latest_identity(packet, now):
        """Report rejected packet identities without renewing accepted freshness."""
        if not isinstance(packet, dict):
            return {}
        result = {'latest_packet_capture_generation': packet.get('capture_generation')}
        ages = []
        for name in ('front', 'rear'):
            stream = packet.get(name)
            if not isinstance(stream, dict):
                continue
            sequence = stream.get('sequence')
            stamp = stream.get('capture_timestamp')
            result['latest_packet_' + name + '_sequence'] = sequence
            if isinstance(stamp, (int, float)) and math.isfinite(stamp):
                result['latest_packet_' + name + '_capture_age_s'] = now - stamp
                ages.append(now - stamp)
        if len(ages) == 2:
            result['latest_packet_capture_age_s'] = max(ages)
        return result

    def update(self, packet, now, dt):
        now, dt = float(now), float(dt)
        if not math.isfinite(now) or not math.isfinite(dt) or dt <= 0.:
            raise ValueError('control time must be finite and dt positive')
        status = dict(valid=False, measurement_valid=False, measurement_is_new=False,
                      control_state='stopped', reason='waiting for paired camera frames',
                      command_source='front_rgb_and_rear_rgbd_only',
                      uses_world_pose=False, uses_simulator_seam=False,
                      rear_target_directly_observed=False, rear_target_held=False,
                      target_estimated=False, rear_target_estimated=False,
                      rear_memory_used=False, target_filter='none',
                      rear_memory_tangent_estimate_configured=self.historical_tangent_estimate,
                      rear_target_max_near_field_extension_m=self.max_near_field_extension_m,
                      rear_path_memory_configured_s=self.rear_path_memory_s,
                      rear_target_max_occlusion_extension_m=self.max_occlusion_extension,
                      startup_initial_anchor='encoder_fk_base_frame_not_world',
                      runtime_must_terminate_on_invalid=True)
        status.update(self._latest_identity(packet, now))
        desired = np.zeros(3, dtype=np.float64)
        try:
            if self._failed_reason is not None:
                raise ValueError('visual failure is latched: ' + self._failed_reason)
            if self._last_now is not None and now < self._last_now - 1.e-10:
                raise ValueError('control clock moved backwards')
            if self._path_memory is not None:
                # The existing buffer integrates exactly the preceding sent
                # command over dt. Clock timestamps associate real captures
                # with that ledger; they do not imply measured robot motion.
                self._path_memory.advance(now, dt, self._velocity)
            generation, sequence, captures, command, front_diag, target, rear_diag = self._validate(packet, now)
            if self._generation is not None and generation != self._generation:
                raise ValueError('capture generation changed; explicit reset with new encoder FK is required')
            # The inherited fit provides tangent heading and robot-left lateral
            # error. VD12's training teacher additionally corrects that heading
            # by atan2(e_y, 0.6); omitting it lets a parallel offset persist.
            # Restore the saved command equation, without changing its gains.
            tangent_heading = float(front_diag['heading_error_rad'])
            lateral = float(front_diag['lateral_error_m'])
            heading = tangent_heading - math.atan2(lateral, .6)
            heading = math.atan2(math.sin(heading), math.cos(heading))
            yaw_rate = float(np.clip(1.5 * heading + self.speed * front_diag['curvature_m_inv'], -.8, .8))
            command = np.asarray(command, dtype=np.float64).copy()
            command[2] = yaw_rate
            front_diag.update(tangent_heading_rad=tangent_heading, heading_error_rad=heading,
                              yaw_rate_rps=yaw_rate, heading_lookahead_m=.6, heading_gain=1.5,
                              front_command_formula='eh=wrap(tangent_heading-atan2(robot_left_error,0.6)); '
                                  'w=clip(1.5*eh+speed*curvature,-0.8,0.8)',
                              front_command_formula_source='VD12_saved_training_ideal_front_command_equation_on_measured_geometry')
            # Preserve the original geometry helper and optional observed-path
            # buffer, including their already-defined support/fit bounds.
            # NumPy bool_ diagnostics must become actual JSON-safe booleans.
            rear_diag = {key: value.item() if isinstance(value, np.generic) else value
                         for key, value in rear_diag.items()}
            directly_observed = bool(rear_diag.get('rear_target_directly_observed', False))
            from_memory = bool(rear_diag.get('rear_memory_used', False))
            near_extension = float(rear_diag.get('rear_target_near_field_extension_m', 0.))
            far_extension = float(rear_diag.get('rear_target_occlusion_extension_m', 0.))
            if from_memory:
                if self._path_memory is None:
                    raise ValueError('observed-path memory was not configured')
                target_stamp = float(rear_diag['rear_memory_source_capture_timestamp_min'])
                target_age = float(rear_diag['rear_memory_source_age_max_s'])
                target_sequences = list(rear_diag['rear_memory_source_sequences'])
                target_sequence = target_sequences[0] if len(target_sequences) == 1 else None
                assumption = 'previously_observed_path_propagated_by_sent_base_commands_not_measured_odometry'
                if rear_diag.get('rear_memory_tangent_estimated', False):
                    assumption += '_with_bounded_near_field_tangent_estimate'
                    rear_diag['rear_target_source'] = 'rear_rgbd_historical_near_field_tangent_estimate'
            elif far_extension > 0.:
                if (rear_diag.get('rear_target_source') != 'rear_rgbd_local_occlusion_estimate'
                        or far_extension > self.max_occlusion_extension + 1.e-12):
                    raise ValueError('far-end gun-occlusion estimate exceeds the configured bound')
                target_stamp, target_age = captures[1], now - captures[1]
                target_sequence, target_sequences = sequence, [sequence]
                assumption = 'bounded_local_line_fit_from_current_measured_rear_path'
            elif not directly_observed:
                if (rear_diag.get('rear_target_source') != 'rear_rgbd_near_field_tangent_estimate'
                        or near_extension <= 0.
                        or near_extension > self.max_near_field_extension_m + 1.e-12):
                    raise ValueError('rear near-field tangent estimate exceeds the configured bound; '
                                     'set zero to require a directly observed TCP work point')
                target_stamp, target_age = captures[1], now - captures[1]
                target_sequence, target_sequences = sequence, [sequence]
                assumption = 'local_measured_tangent_continues_across_camera_body_near_field_occlusion'
            else:
                target_stamp, target_age = captures[1], now - captures[1]
                target_sequence, target_sequences = sequence, [sequence]
                assumption = None
            payload = self._digest(packet)
            new = self._generation is None or sequence != self._sequence
            if self._capture is not None:
                if not new and payload != self._payload_digest:
                    raise ValueError('repeated frame sequence changed its image-derived geometry')
                if new and any(stamp <= old for stamp, old in zip(captures, self._capture)):
                    raise ValueError('new camera sequence did not advance both capture timestamps')
            # First accepted actor command uses t=0, as in the training teacher.
            blend = startup_fraction(self._elapsed, self.ramp_s)
            quaternion = normalized_lerp_quaternion(
                self._start_quaternion, self._final_quaternion, blend)
            plane = _plane(packet['rear']['ground_plane_b'])
            normal = plane[:3]
            axis = quaternion_matrix(quaternion)[:, 0]
            height = cylinder_endface_height(axis, plane, length_m=self.tube_length_m,
                                            radius_m=self.tube_outer_radius_m,
                                            clearance_m=self.working_clearance_m)
            # Remove small depth residual perpendicular to the measured plane;
            # every term here is measured geometry, fixed dimensions, or FK init.
            target_on_plane = target - (target @ normal + plane[3]) * normal
            goal = target_on_plane + height * normal
            position = self._start_position + blend * (goal - self._start_position)
            desired = np.asarray(command, dtype=np.float64) * blend
            if not np.isfinite(position).all() or not np.isfinite(desired).all():
                raise ValueError('nonfinite visual command')
            self._position, self._quaternion = position, quaternion
            self._generation, self._sequence, self._capture = generation, sequence, captures
            self._payload_digest = payload
            self._elapsed += dt
            self._last_now = now
            self._accepted_at = now
            status.update(front_diag)
            status.update(rear_diag)
            status.update(valid=True, measurement_valid=not from_memory, measurement_is_new=new,
                          control_state='rear_memory' if from_memory else 'tracking',
                          reason=('previously observed path; sent-command propagation, not measured odometry'
                                  if from_memory else 'paired image-derived paths and depth plane valid'),
                          measurement_valid_scope='paired_image_paths_and_depth_plane',
                          target_estimated=not directly_observed,
                          rear_target_estimated=not directly_observed,
                          rear_target_directly_observed=directly_observed,
                          rear_target_source_capture_generation=generation,
                          rear_target_source_sequence=target_sequence,
                          rear_target_source_sequences=target_sequences,
                          rear_target_source_capture_timestamp=target_stamp,
                          rear_target_source_age_s=target_age,
                          rear_target_estimate_assumption=assumption,
                          startup_blend=blend, startup_elapsed_s=self._elapsed - dt,
                          target_on_measured_plane_b=target_on_plane.tolist(),
                          tool_goal_b=goal.tolist(), tool_rim_centre_height_m=height,
                          gun_axis_dot_measured_normal=float(axis @ normal),
                          plane_projection_residual_m=float(target @ normal + plane[3]))
        except (KeyError, TypeError, ValueError, FloatingPointError) as error:
            if self._failed_reason is None:
                self._failed_reason = str(error)
            status['reason'] = str(error)
            if isinstance(error, RearTargetSupportShortfall):
                status.update(failure_code='rear_target_support_shortfall', **error.diagnostics)
        # No velocity carry-over on image loss. The last tool setpoint is NOT a
        # new observation and is returned only for an orderly episode stop.
        self._velocity = desired if status['valid'] else np.zeros(3, dtype=np.float64)
        self._velocity[1] = 0.
        status.update(capture_generation=self._generation, source_sequence=self._sequence,
                      capture_timestamps=None if self._capture is None else list(self._capture),
                      capture_age_s=None if self._capture is None else now - min(self._capture),
                      failure_latched=self._failed_reason is not None,
                      tool_pose_source='current_visual_command' if status['valid'] else 'last_setpoint_not_reobserved')
        return dict(base_command_b=self._velocity.tolist(),
                    tool_pose_b=np.r_[self._position, self._quaternion].tolist(), status=status)
