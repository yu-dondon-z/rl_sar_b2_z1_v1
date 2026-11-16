"""Local path interpolation from synchronized, accepted camera measurements.

The unobserved space between cameras is a fitted estimate, never a known road
profile. Ground leveling uses the measured RGBD plane; no simulator state or
world trajectory is accepted here.
"""
import numpy as np


def paired_ground_guidance(front, rear, plane, lookahead=.6, adaptive_lookahead=False):
    """Fit both visible sections and interpolate a nearby steering reference.

    Returns the same geometry tuple as vd1_control._front_geometry plus fit
    diagnostics. Call only after the rear temporal/working-point guard passed.
    A rejected fit must leave the independently valid front guidance usable.
    """
    front, rear, plane = (np.asarray(value, dtype=float) for value in (front, rear, plane))
    if (front.ndim != 2 or rear.ndim != 2 or front.shape[1] != 3 or rear.shape[1] != 3
            or plane.shape != (4,) or not all(np.isfinite(v).all() for v in (front, rear, plane))
            or not .3 <= lookahead <= 1.2):
        raise ValueError('invalid paired path geometry')
    normal = plane[:3].copy()
    length = np.linalg.norm(normal)
    if not .5 <= length <= 2.:
        raise ValueError('invalid observed ground normal')
    normal /= length
    if normal[2] < 0:
        normal *= -1
    if normal[2] < .85:
        raise ValueError('paired ground frame is too tilted')
    forward = np.array([1., 0., 0.])-normal[0]*normal
    forward /= np.linalg.norm(forward)
    left = np.cross(normal, forward)
    basis = np.column_stack((forward, left, normal))
    sections = []
    for index, points in enumerate((rear, front)):
        level = points @ basis
        level = level[level[:, 0] < -.4] if index == 0 else level[level[:, 0] > .1]
        if len(level) < 3:
            raise ValueError('paired path lacks visible samples')
        level = level[np.argsort(level[:, 0], kind='stable')]
        # Fit only the local metre of front support and the rear working area.
        if index == 0:
            level = level[(level[:, 0] >= -1.5) & (level[:, 0] <= -.4)]
        else:
            level = level[level[:, 0] <= level[0, 0]+1.]
        if len(level) < 3 or np.ptp(level[:, 0]) < (.15 if index == 0 else .30):
            raise ValueError('paired path has insufficient section support')
        # Equal spatial support per camera avoids pixel-density weighting.
        x = np.linspace(level[0, 0], level[-1, 0], 12)
        sections.append(np.column_stack((x, np.interp(x, level[:, 0], level[:, 1]))))
    back, ahead = sections
    if not back[-1, 0] < 0 < lookahead < ahead[0, 0] or ahead[0, 0]-back[-1, 0] > 3.:
        raise ValueError('near reference is not bracketed by paired sections')
    samples = np.vstack(sections)
    x, y = samples.T
    scale = max(abs(x).max(), 1.)
    design = np.vander(x/scale, 4, increasing=True)
    weights = np.ones(len(x))
    for _ in range(5):
        root = np.sqrt(weights)
        coefficients, _, rank, _ = np.linalg.lstsq(design*root[:, None], y*root, rcond=None)
        if rank != 4:
            raise ValueError('paired path fit is singular')
        residual = np.abs(design @ coefficients-y)
        tolerance = max(.004, 2.5*float(np.median(residual)))
        weights = np.minimum(1., tolerance/np.maximum(residual, 1e-12))
    if any(np.median(r) > .015 or np.percentile(r, 90) > .035 for r in np.split(residual, 2)):
        raise ValueError('paired path sections do not agree with a smooth local fit')
    coefficients /= scale**np.arange(4)
    value = lambda at: float(np.polynomial.polynomial.polyval(at, coefficients))
    derivative = np.polynomial.polynomial.polyder(coefficients)
    second = np.polynomial.polynomial.polyder(derivative)
    probe = np.linspace(back[-1, 0], ahead[0, 0], 25)
    slope = np.polynomial.polynomial.polyval(probe, derivative)
    curvature = np.polynomial.polynomial.polyval(probe, second)/(1+slope*slope)**1.5
    if np.max(np.abs(curvature)) > .6 or np.max(np.abs(slope)) > 1.:
        raise ValueError('paired path has excessive inferred bend in the camera gap')
    offset, heading = value(0.), float(np.arctan(derivative[0]))
    local_curvature = float(second[0]/(1+derivative[0]**2)**1.5)
    if adaptive_lookahead:
        # Retain the original horizon for gentle bends. A nearer target gives
        # more cross-track feedback where the measured bend exceeds .15/m.
        lookahead = max(min(.4, lookahead), lookahead/(1+3*max(abs(local_curvature)-.15, 0.)))
    target = np.array([lookahead, value(lookahead), 0.])
    # Steering is in the ground-leveled forward/left frame. Actor tool targets
    # remain in the original base frame and are not transformed by this fit.
    geometry = (heading, -offset, offset, local_curvature, target)
    return geometry, dict(front_steering_method='paired_ground_cubic_interpolation',
        front_diagnostics_reference='observed_plane_local_fit',
        guidance_lookahead_xy_ground_m=target[:2].tolist(),
        front_lookahead_b=(basis @ [target[0], target[1], -plane[3]/length]).tolist(),
        guidance_lookahead_m=lookahead, guidance_fit_p95_m=float(np.percentile(residual, 95)),
        guidance_local_curvature_m_inv=local_curvature,
        guidance_adaptive_lookahead=bool(adaptive_lookahead),
        guidance_section_x_ranges_m=[[float(s[0, 0]), float(s[-1, 0])] for s in sections],
        guidance_gap_is_interpolated=True)
