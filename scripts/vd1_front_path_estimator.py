"""Front-only local curve estimation, without odometry or world coordinates.

The three states are coefficients of a local quadratic in the gravity-levelled capture
frame, not a robot pose. A random-walk Kalman model smooths successive curve
observations; it does not register old points or compensate robot motion.
Steering targets stay within visible support; the blind intercept is diagnostic only.
"""
from __future__ import annotations

import numpy as np


def nominal_front_plane(gravity_b, height_m):
    """Flat-floor assumption from IMU gravity and a configured constant height.

    Height is a user calibration, never a live simulator position or rear
    measurement. This is not a base-height or base-motion estimator.
    """
    gravity = np.asarray(gravity_b, dtype=float)
    if (gravity.shape != (3,) or not np.isfinite(gravity).all()
            or not .9 <= np.linalg.norm(gravity) <= 1.1
            or not np.isfinite(height_m) or not .3 <= height_m <= 1.):
        raise ValueError('front projection needs IMU gravity and a calibrated nominal height')
    normal = -gravity / np.linalg.norm(gravity)
    if normal[2] < .75:
        raise ValueError('front flat-floor projection is too tilted')
    return np.r_[normal, height_m]


def ground_basis(plane):
    plane = np.asarray(plane, dtype=float)
    if plane.shape != (4,) or not np.isfinite(plane).all():
        raise ValueError('front path requires its own finite projection plane')
    length = np.linalg.norm(plane[:3])
    if not .9 <= length <= 1.1 or plane[2] / length < .75:
        raise ValueError('front projection plane is invalid')
    normal = plane[:3] / length
    forward = np.array([1., 0., 0.]) - normal[0] * normal
    forward /= np.linalg.norm(forward)
    return np.column_stack((forward, np.cross(normal, forward), normal))


class FrontPathEstimator:
    """Fit the nearest half-metre; do not extend a distant S bend to the base."""

    method = 'front_only_local_quadratic_kalman'

    def __init__(self, lookahead_m=.6, reset_gap_s=.75):
        if not .3 <= lookahead_m <= 1.2 or not .1 <= reset_gap_s <= 2.:
            raise ValueError('invalid front path estimator configuration')
        self.lookahead_m = float(lookahead_m)
        self.reset_gap_s = float(reset_gap_s)
        self._key = self._capture = self._coefficients = self._covariance = None
        self._cached = None
        self._origin_x = None

    def update(self, stream):
        generation, sequence = stream.get('capture_generation'), stream.get('sequence')
        capture = stream.get('capture_timestamp')
        if (type(generation) is not int or type(sequence) is not int or min(generation, sequence) < 0
                or not isinstance(capture, (float, int)) or isinstance(capture, bool)
                or not np.isfinite(capture)):
            raise ValueError('front estimator requires original capture identity and timestamp')
        key = (generation, sequence)
        if key == self._key:
            if capture != self._capture:
                raise ValueError('a repeated front capture cannot refresh its timestamp')
            return self._cached
        if self._key is not None and (key < self._key or (key[0] == self._key[0] and capture <= self._capture)):
            raise ValueError('front path capture moved backwards')
        points = np.asarray(stream.get('points_b'), dtype=float)
        if (points.ndim != 2 or points.shape[1] != 3 or not 6 <= len(points) <= 4096
                or not np.isfinite(points).all() or np.max(np.abs(points)) > 20):
            raise ValueError('front estimator needs finite observed xyz samples')
        plane = np.asarray(stream.get('ground_plane_b'), dtype=float)
        basis = ground_basis(plane)
        level = points @ basis
        level = level[(level[:, 0] > .1) & (level[:, 0] <= 3.5)]
        level = level[np.argsort(level[:, 0], kind='stable')]
        if len(level) < 6 or np.ptp(level[:, 0]) < .35:
            raise ValueError('front path has insufficient visible fitting support')
        # Equal spatial support prevents near-camera pixel density dominating.
        # The local tangent/curvature are observable here. Fitting a complete
        # distant S bend gives a low in-view residual but an arbitrary blind
        # intercept. Express the nearest section at its own measured origin.
        x = np.linspace(level[0, 0], min(level[-1, 0], level[0, 0] + .55), 32)
        y = np.interp(x, level[:, 0], level[:, 1])
        # Startup uses a target INSIDE the current visible segment, not the
        # blind intercept of either the linear or quadratic model.
        straight = np.polynomial.polynomial.polyfit(x, y, 1)
        straight_x = float((x[0] + x[-1]) / 2)
        straight_y = float(np.polynomial.polynomial.polyval(straight_x, straight))
        straight_residual = float(np.percentile(np.abs(
            np.polynomial.polynomial.polyval(x, straight) - y), 95))
        origin_x=float(x[0]);local_x=x-origin_x
        design = np.vander(local_x, 3, increasing=True)
        reset = (self._key is None or key[0] != self._key[0]
                 or capture - self._capture > self.reset_gap_s)
        if reset:
            prior = np.zeros(3)
            covariance = np.diag(np.square([.5, .6, .35]))
        else:
            delta=origin_x-self._origin_x
            change=np.array([[1.,delta,delta*delta],[0.,1.,2*delta],[0.,0.,1.]])
            # Polynomial origin change only; no robot pose registration.
            prior = change @ self._coefficients
            # Curve evolution in capture coordinates, with no velocity,
            # gyro integration, command propagation, or pose state.
            covariance = change @ self._covariance @ change.T + np.diag(
                np.square([.008, .04, .035])) * (capture - self._capture)
        precision = np.linalg.inv(covariance)
        variance = np.square(.003 + .002 * x*x)
        weights = np.ones(len(x))
        for _ in range(4):
            information = design.T @ ((weights / variance)[:, None] * design)
            posterior = np.linalg.inv(precision + information)
            coefficients = posterior @ (precision @ prior + design.T @ (weights * y / variance))
            residual = np.abs(design @ coefficients - y)
            tolerance = max(.008, 2.5 * float(np.median(residual)))
            weights = np.minimum(1., tolerance / np.maximum(residual, 1e-12))
        fit_p95 = float(np.percentile(residual, 95))
        if fit_p95 > .045:
            raise ValueError('front curve fit residual exceeds 4.5 cm')
        a,b,c=coefficients
        c0,c1,c2=a-b*origin_x+c*origin_x**2,b-2*c*origin_x,c
        # Do not let inferred curvature pull the target further into the blind
        # region. Use at least the centre of the nearest visible fitting span.
        lookahead = float(np.clip(self.lookahead_m, straight_x, x[-1]))
        tangent = c1 + 2*c2*lookahead
        curvature = float(2*c2 / (1+tangent*tangent)**1.5)
        target_y = float(np.polynomial.polynomial.polyval(lookahead-origin_x, coefficients))
        if abs(target_y) > 1.:
            raise ValueError('front inferred curve exceeds local tracking range '
                             '(curvature=%.3f/m, intercept=%.3fm, target_y=%.3fm)' %
                             (curvature, c0, target_y))
        target = np.array([lookahead, target_y, 0.])
        offset = float(c0 / np.sqrt(1+c1*c1))
        geometry = (float(np.arctan(c1)), -offset, offset, curvature, target)
        row = np.array([1., lookahead-origin_x, (lookahead-origin_x)**2])
        offset_row=np.array([1.,-origin_x,origin_x**2])
        target_std = float(np.sqrt(max(0., row @ posterior @ row)))
        observed_target_y = float(np.interp(lookahead, level[:, 0], level[:, 1]))
        target_innovation = abs(target_y-observed_target_y)
        reliable = (fit_p95 <= .02 and target_std <= .04
                    and target_innovation <= .025 and abs(curvature) <= .8)
        diagnostics = dict(front_steering_method=self.method,
            front_diagnostics_reference='front_curve_model_extrapolation',
            guidance_lookahead_m=lookahead,
            guidance_lookahead_xy_ground_m=target[:2].tolist(),
            front_lookahead_b=(basis @ [lookahead, target_y, -plane[3]]).tolist(),
            guidance_fit_p95_m=fit_p95, guidance_local_curvature_m_inv=curvature,
            front_visible_x_range_m=[float(level[0,0]), float(level[-1,0])],
            front_fitting_x_range_m=[float(x[0]),float(x[-1])],
            front_estimated_coefficients=[float(c0),float(c1),float(c2)],
            front_estimated_offset_std_m=float(np.sqrt(max(0., offset_row @ posterior @ offset_row))),
            front_estimated_target_std_m=target_std,
            front_estimator_reliable=bool(reliable),
            front_target_observation_error_m=float(target_innovation),
            front_blind_intercept_used_for_steering=False,
            front_uncertainty_excludes_projection_model_bias=True,
            front_extrapolation_distance_m=max(0., float(x[0]-lookahead)),
            front_near_field_is_extrapolated=bool(lookahead < x[0]),
            front_estimator_reset=bool(reset), front_estimator_capture_timestamp=float(capture),
            front_estimator_sequence=sequence, rear_used_for_steering=False,
            body_motion_estimation=False, historical_points_registered=False,
            front_projection_source=stream.get('geometry_source'))
        diagnostics.update(front_visible_line_target_xy_m=[straight_x, straight_y],
            front_visible_line_heading_rad=float(np.arctan(straight[1])),
            front_visible_line_fit_p95_m=straight_residual)
        self._key, self._capture = key, float(capture)
        self._coefficients, self._covariance = coefficients, posterior
        self._origin_x=origin_x
        self._cached = (geometry, diagnostics)
        return self._cached
