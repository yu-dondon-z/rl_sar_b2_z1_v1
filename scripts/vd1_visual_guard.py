"""Image-only temporal guard and bounded rear-dropout state machine.

Reuse the original geometric fits and actor command convention. No simulator
state, nominal world curve, or generated camera points enter this controller.
"""
from __future__ import annotations

import math
from numbers import Real

import numpy as np

from vd1_control import (SCHEMA, VD1VisualController, WORKSPACE_LOW, WORKSPACE_HIGH,
                         _array, _check_payload, _front_geometry, _points)
from vd1_visual_path import paired_ground_guidance


class FrontStartupGuidance:
    """Count distinct captures only; no commanded-distance or ego-motion estimate."""

    def __init__(self):
        self.key = self.previous_yaw = self.first_capture = None
        self.count = 0
        self.elapsed = self.weight = 0.
        self.stable = False

    def update(self, diagnostics, identity, yaw, speed, dt):
        self.elapsed += dt
        key, capture = identity[:2], identity[2][0]
        if key != self.key:
            self.stable = (diagnostics['front_estimator_reliable']
                and not diagnostics['front_estimator_reset']
                and diagnostics['guidance_fit_p95_m'] <= .02
                and diagnostics['front_estimated_target_std_m'] <= .04
                and self.previous_yaw is not None and abs(yaw-self.previous_yaw) <= .08)
            if not self.stable:
                self.count, self.first_capture = 1, capture
            else:
                self.count += 1
            self.key, self.previous_yaw = key, yaw
        ready = (self.count >= 6 and capture-self.first_capture >= 1.5
                 and self.elapsed >= 2.)
        # Continue checking after handover. The output yaw filter smooths the
        # fallback; recovery must qualify again using distinct camera captures.
        if not self.stable:
            self.weight = 0.
        elif ready:
            self.weight = min(1., self.weight + dt/1.5)
        x, y = diagnostics['front_visible_line_target_xy_m']
        line_yaw = float(np.clip(2*speed*y/(x*x+y*y), -.12, .12))
        if self.weight == 0. and (diagnostics['front_visible_line_fit_p95_m'] > .025
                or abs(diagnostics['front_visible_line_heading_rad']) > .35):
            raise ValueError('front startup: visible segment is not sufficiently straight/aligned')
        blend = self.weight*self.weight*(3-2*self.weight)
        diagnostics.update(front_startup_phase=('estimator' if self.weight == 1. else
            'blending' if self.weight > 0. else 'visible_line'),
            front_estimator_blend=blend, front_startup_distinct_stable_frames=self.count,
            front_estimator_fallback=not self.stable,
            front_startup_elapsed_s=self.elapsed, front_visible_line_yaw_rps=line_yaw,
            front_quadratic_yaw_rps=yaw)
        return (1-blend)*line_yaw + blend*yaw


def rear_seam_target(points, plane):
    """Give the actor a desired position without adding measured TCP error.

    Preserve the trained rear working x and target height. Measured depth
    locates the seam; this does not introduce a new welding standoff mode.
    """
    normal_length = float(np.linalg.norm(plane[:3]))
    if not .5 <= normal_length <= 2:
        raise ValueError('invalid observed ground plane')
    if abs(plane[2]/normal_length) < .4:
        raise ValueError('ground is not observable along base vertical')
    ordered = points[np.argsort(points[:, 0], kind='stable')]
    x = -1.02
    if not ordered[0, 0] <= x <= ordered[-1, 0]:
        raise ValueError('visible rear seam does not cover x=-1.02 m')
    nominal = np.array([x, float(np.interp(x, ordered[:, 0], ordered[:, 1])), -.30])
    target = np.clip(nominal, WORKSPACE_LOW, WORKSPACE_HIGH)
    return target, dict(rear_nominal_target_b=nominal.tolist(),
                        tool_target_mode='rear_seam_direct', visual_tcp_feedback=False,
                        requires_tcp_markers=False,
                        target_clipped=bool(np.any(np.abs(nominal-target) > 1e-12)))


class GuardedVD1VisualController(VD1VisualController):
    """Hold the tool briefly on rear loss while using fresh front steering.

    Persistent rear loss or any front/provenance/clock failure still brakes.
    After a stop, three distinct good paired captures are needed to restart.
    """

    def __init__(self, init_tcp_b, forward_speed=.4, stale_timeout_s=1., ramp_s=2.,
                 rear_hold_s=.6, rear_jump_m=.04, rear_rate_mps=.45,
                 recovery_frames=3, paired_guidance=True, front_path_estimation=False,
                 front_path_filter=None):
        super().__init__(init_tcp_b, forward_speed, stale_timeout_s, ramp_s)
        if (not np.isfinite([rear_hold_s, rear_jump_m, rear_rate_mps]).all()
                or not 0 < rear_hold_s <= stale_timeout_s or rear_jump_m <= 0 or rear_rate_mps <= 0
                or type(recovery_frames) is not int or recovery_frames < 2):
            raise ValueError('Invalid visual guard configuration')
        self.guard_config = dict(rear_hold_s=rear_hold_s, rear_jump_m=rear_jump_m,
                                 rear_rate_mps=rear_rate_mps, recovery_frames=recovery_frames,
                                 continuity_max_interval_s=.25, paired_guidance=bool(paired_guidance),
                                 guidance_lookahead_m=.6, guidance_adaptive_lookahead=True,
                                 yaw_filter_tau_s=.12, speed_filter_tau_s=.20,
                                 tool_target_mode='rear_seam_direct', visual_tcp_feedback=False,
                                 requires_tcp_markers=False)
        self._seen = None
        self._stream_seen = {}
        self._rear_anchor = None
        self._rear_pending = None
        self._rear_decision = None
        self._rear_missing_since = None
        self._mode = 'waiting'
        self._ever_tracking = False
        self._recovery_count = 0
        self._recovery_sequence = None
        self._front_estimator = None
        self._front_startup = FrontStartupGuidance()
        if front_path_estimation:
            from vd1_front_path_estimator import FrontPathEstimator
            if front_path_filter is not None and not isinstance(front_path_filter,FrontPathEstimator):
                raise ValueError('front_path_filter must be a FrontPathEstimator')
            self._front_estimator = front_path_filter if front_path_filter is not None else FrontPathEstimator()
            self.guard_config.update(paired_guidance=False, front_path_estimation=True,
                                     body_motion_estimation=False)

    def _envelope(self, packet, now):
        if not isinstance(packet, dict):
            raise ValueError('waiting for paired visual measurements')
        _check_payload(packet)
        if packet.get('schema') != SCHEMA:
            raise ValueError('invalid dual-camera schema')
        generation = packet.get('capture_generation')
        if type(generation) is not int or generation < 0:
            raise ValueError('invalid capture generation')
        independent = packet.get('stream_mode') == 'independent_latest'
        if independent and self._front_estimator is None:
            raise ValueError('independent streams require front-only estimation')
        streams, captures, sequences = {}, [], []
        for name in ('front', 'rear'):
            stream = packet.get(name)
            if not isinstance(stream, dict) or stream.get('capture_generation') != generation:
                raise ValueError('paired streams have different reset generations')
            sequence, capture = stream.get('sequence'), stream.get('capture_timestamp')
            if type(sequence) is not int or sequence < 0:
                raise ValueError('invalid visual sequence')
            if not isinstance(capture, Real) or isinstance(capture, bool) or not math.isfinite(capture):
                raise ValueError('missing finite monotonic capture timestamp')
            if not -.002 <= now-capture <= self.stale_timeout_s:
                raise ValueError(name+' capture is stale or in the future')
            streams[name] = stream
            sequences.append(sequence)
            captures.append(float(capture))
            previous = self._stream_seen.get(name)
            if independent and previous is not None and previous[0] == generation:
                if sequence < previous[1] or capture < previous[2]:
                    raise ValueError(name+' capture moved backwards')
                if (sequence == previous[1]) != (capture == previous[2]):
                    raise ValueError(name+' repeated capture identity or timestamp changed')
        if not independent and (sequences[0] != sequences[1] or abs(captures[0]-captures[1]) > .04):
            raise ValueError('camera captures are not synchronized')
        publication = packet.get('publication_sequence') if independent else sequences[0]
        if type(publication) is not int or publication < 0:
            raise ValueError('invalid publication sequence')
        identity = (generation, publication, tuple(captures))
        if self._seen is not None:
            old_generation, old_sequence, old_captures = self._seen
            if generation < old_generation:
                raise ValueError('capture generation moved backwards')
            if generation == old_generation:
                if publication < old_sequence:
                    raise ValueError('visual sequence moved backwards')
                if publication == old_sequence and tuple(captures) != old_captures:
                    raise ValueError('repeated sequence cannot refresh capture timestamp')
                if publication > old_sequence and min(captures) < min(old_captures):
                    raise ValueError('capture time moved backwards')
            else:
                self._rear_anchor = self._rear_pending = self._rear_decision = None
                self._front_startup = FrontStartupGuidance()
                self._tracking_elapsed = 0.
                self._mode = 'stopped' if self._ever_tracking else 'waiting'
                self._recovery_count = 0
                self._recovery_sequence = None
        self._seen = identity
        self._stream_seen = {name: (generation, streams[name]['sequence'], captures[i])
                             for i, name in enumerate(('front', 'rear'))}
        self._generation, self._sequence, self._capture_times = identity
        return streams, identity

    def _rear_candidate(self, stream, identity, now=None):
        if stream.get('valid') is not True:
            self._rear_pending = None
            raise ValueError(str(stream.get('reason', 'rear visual measurement is invalid')))
        try:
            target, diagnostics = rear_seam_target(_points(stream.get('points_b')),
                _array(stream.get('ground_plane_b'), (4,)))
        except (KeyError, TypeError, ValueError, FloatingPointError):
            self._rear_pending = None
            raise
        key = (stream['capture_generation'], stream['sequence'])
        if self._rear_decision is not None and self._rear_decision[0] == key:
            if self._rear_decision[1] is not None:
                raise ValueError(self._rear_decision[1])
            return target, diagnostics
        capture = identity[2][1]
        y = diagnostics['rear_nominal_target_b'][1]  # Before workspace clipping.
        candidate = dict(key=key, capture=capture, y=y)
        error = None
        if self._rear_anchor is not None:
            elapsed = capture-self._rear_anchor['capture']
            bound = self.guard_config['rear_jump_m'] + self.guard_config['rear_rate_mps'] * min(
                max(elapsed, 0.), self.guard_config['continuity_max_interval_s'])
            if abs(y-self._rear_anchor['y']) > bound:
                error = 'rear path discontinuity: %.3fm exceeds %.3fm' % (
                    abs(y-self._rear_anchor['y']), bound)
            # A long unobserved interval needs independent fresh captures,
            # even when its first candidate happens to match the old anchor.
            short_observed_dropout = (now is not None and self._rear_missing_since is not None
                and 0 <= now-self._rear_missing_since <= self.guard_config['rear_hold_s'])
            if elapsed > self.guard_config['rear_hold_s'] and not short_observed_dropout:
                pending = self._rear_pending
                if pending is not None and pending['key'] != key:
                    gap = capture-pending['capture']
                    consistent = (0 < gap <= self.guard_config['rear_hold_s']
                        and abs(y-pending['y']) <= self.guard_config['rear_jump_m']
                            + self.guard_config['rear_rate_mps']*min(gap, .25))
                    count = pending['count']+1 if consistent else 1
                else:
                    count = 1
                self._rear_pending = dict(candidate, count=count)
                error = 'rear reacquisition requires distinct consistent captures'
                if count >= self.guard_config['recovery_frames']:
                    error = None
        self._rear_decision = (key, error)
        if error is not None:
            raise ValueError(error)
        self._rear_anchor = candidate
        self._rear_pending = None
        return target, diagnostics

    def update(self, packet, now, dt):
        now, dt = float(now), float(dt)
        if not np.isfinite([now, dt]).all() or dt <= 0:
            raise ValueError('now must be finite and dt must be positive')
        desired = np.zeros(3)
        status = dict(valid=False, reason='waiting for images', front_valid=False, rear_valid=False,
                      target_clipped=False, target_updated=False, geometry_mode='capture_base_frame_sample_hold',
                      uses_world_pose=False, tcp_projection_error_m=None,
                      tool_target_mode='rear_seam_direct', visual_tcp_feedback=False,
                      requires_tcp_markers=False)
        geometry = target = None
        guidance_status = dict(front_steering_method='visible_point_pure_pursuit',
                               front_diagnostics_reference='visible_tangent_not_base_pose')
        identity = None
        try:
            streams, identity = self._envelope(packet, now)
            if streams['front'].get('valid') is not True:
                raise ValueError('front: '+str(streams['front'].get('reason', 'invalid measurement')))
            if self._front_estimator is None:
                geometry = _front_geometry(_points(streams['front'].get('points_b')))
            else:
                geometry, guidance_status = self._front_estimator.update(streams['front'])
            status['front_valid'] = True
            try:
                target, diagnostics = self._rear_candidate(streams['rear'], identity, now)
                status.update(diagnostics, rear_valid=True)
                if self.guard_config['paired_guidance']:
                    try:
                        geometry, guidance_status = paired_ground_guidance(
                            _points(streams['front']['points_b']), _points(streams['rear']['points_b']),
                            _array(streams['rear']['ground_plane_b'], (4,)),
                            self.guard_config['guidance_lookahead_m'],
                            adaptive_lookahead=self.guard_config['guidance_adaptive_lookahead'])
                    except ValueError as error:
                        status['paired_guidance_rejection'] = str(error)
            except (KeyError, TypeError, ValueError, FloatingPointError) as error:
                status['rear_reason'] = str(error)
        except (KeyError, TypeError, ValueError, FloatingPointError) as error:
            status['reason'] = str(error)
        rear_age = now-self._rear_anchor['capture'] if self._rear_anchor is not None else None
        if status['rear_valid']:
            self._rear_missing_since = None
        elif self._rear_missing_since is None:
            self._rear_missing_since = now
        outage = 0. if self._rear_missing_since is None else now-self._rear_missing_since
        # This holds a previously submitted position reference, not a new
        # measurement. The envelope still rejects any incoming capture older
        # than stale_timeout_s. Do not rewrite the trusted capture age: it may
        # exceed that limit while the explicitly labelled rear_hold is active.
        can_hold = (self._ever_tracking and status['front_valid'] and rear_age is not None
                    and -.002 <= rear_age
                    and outage <= self.guard_config['rear_hold_s'])
        usable = status['front_valid'] and (status['rear_valid'] or can_hold)
        outside_workspace = bool(status['rear_valid'] and status.get('target_clipped'))
        if outside_workspace:
            usable = False
            status.update(workspace_reachable=False,
                reason='observed rear seam outside actor workspace: y=%.3f m; allowed [-0.150, 0.150] m' %
                       status['rear_nominal_target_b'][1])
        if self._mode == 'stopped':
            if status['front_valid'] and status['rear_valid'] and not outside_workspace:
                recovery_key = tuple(streams[name]['sequence'] for name in ('front', 'rear'))
                if (self._recovery_sequence is None or all(
                        a > b for a, b in zip(recovery_key, self._recovery_sequence))):
                    self._recovery_count += 1
                    self._recovery_sequence = recovery_key
                usable = self._recovery_count >= self.guard_config['recovery_frames']
                if not usable:
                    status['reason'] = 'waiting for consecutive paired recovery captures'
            else:
                self._recovery_count = 0
                self._recovery_sequence = None
                usable = False
        if usable:
            heading, lateral, offset, curvature, lookahead = geometry
            yaw = float(np.clip(2*self.forward_speed*lookahead[1] / max(
                float(lookahead[0]**2+lookahead[1]**2), 1e-6), -.8, .8))
            if self._front_estimator is not None:
                try:
                    front_identity = (identity[0], streams['front']['sequence'], identity[2])
                    yaw = self._front_startup.update(guidance_status, front_identity, yaw,
                                                    self.forward_speed, dt)
                except ValueError as error:
                    usable = False
                    status['reason'] = str(error)
        if usable:
            self._ever_tracking = True
            self._mode = 'tracking' if status['rear_valid'] else 'rear_hold'
            self._tracking_elapsed += dt
            blend = 1. if self.ramp_s == 0 else min(self._tracking_elapsed/self.ramp_s, 1.)
            blend = blend*blend*(3-2*blend)
            desired[:] = [self.forward_speed*blend, 0., yaw*blend]
            if status['rear_valid']:
                delta = (1-math.exp(-dt/.30))*(target-self._tool_position)
                norm = float(np.linalg.norm(delta))
                if norm > .15*dt:
                    delta *= .15*dt/norm
                self._tool_position += delta
                status['target_updated'] = True
            status.update(valid=True, reason='paired visual measurements valid' if status['rear_valid'] else
                'rear briefly unavailable: tool held; fresh front steering active',
                startup_blend=blend, requested_yaw_rate_rps=yaw, front_heading_error_rad=heading,
                robot_lateral_error_m=lateral, path_left_offset_m=offset, front_curvature_m_inv=curvature,
                front_lookahead_b=lookahead.tolist())
            status.update(guidance_status)
        else:
            self._front_startup = FrontStartupGuidance()
            if self._ever_tracking:
                if self._mode != 'stopped':
                    self._tracking_elapsed = 0.
                    self._recovery_count = 0
                    self._recovery_sequence = None
                self._mode = 'stopped'
            if status['front_valid'] and not status['rear_valid']:
                status['reason'] = 'rear unavailable beyond hold interval: '+status.get('rear_reason', '')
        # Keep the existing speed/braking transition. Only valid paired path
        # steering gets a shorter time constant; no curvature is added twice.
        tau = np.full(3, self.guard_config['speed_filter_tau_s'])
        if usable and guidance_status['front_steering_method'] in (
                'paired_ground_cubic_interpolation', 'front_only_cubic_kalman',
                'front_only_local_quadratic_kalman'):
            tau[2] = self.guard_config['yaw_filter_tau_s']
        self._velocity += (1-np.exp(-dt/tau))*(desired-self._velocity)
        if not status['valid'] and np.linalg.norm(self._velocity) < 1e-5:
            self._velocity[:] = 0.
        self._velocity[1] = 0.
        status.update(state=self._mode, recovery_frames=self._recovery_count,
            stream_mode=packet.get('stream_mode', 'paired') if isinstance(packet, dict) else 'paired',
            capture_generation=self._generation, source_sequence=self._sequence,
            capture_age_s=None if self._capture_times is None else now-min(self._capture_times),
            rear_trusted_capture_age_s=rear_age,
            rear_outage_s=outage,
            rear_source_sequence=None if self._rear_anchor is None else self._rear_anchor['key'][1],
            target_transition_outside_workspace=bool(np.any(self._tool_position < [-1.08, -.15, -.34])
                                                    or np.any(self._tool_position > [-.96, .15, -.26])))
        return dict(base_command_b=self._velocity.tolist(),
                    tool_pose_b=np.concatenate((self._tool_position, self._tool_quat)).tolist(), status=status)
