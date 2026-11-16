"""Conservative visible tape-cap evidence, not an endpoint/motion authority.

The only reported point is the existing track's last observed supported point.
An estimated tangent is used ONLY to look for contradictory RGB continuation;
it never creates a path point. Image-space evidence cannot distinguish a cap
from an exactly floor-coloured occluder hiding all continuation. Same-track
multi-frame confirmation and the original healthy supported stop are required
downstream. This does not certify arrival of the rear TCP at the endpoint.
"""
import numpy as np


ROI_LEFT, ROI_RIGHT, ROI_TOP = 128, 512, int(.57 * 360)
SCALE = 3.


def visible_endpoint(frame_fields, observed_points, path_valid):
    """Return finite JSON fields; keep every pre-existing tracker gate intact.

    ``frame_fields`` are NearTracker's 640x360 raw gray, neural probability,
    and RGB contrast mask. Points and outputs use calibrated native pixels.
    ``endpoint_clear_pixels`` is observed clear distance beyond the cap, not
    predicted remaining metric distance. False candidates carry zero geometry.
    """
    result = dict(endpoint_candidate=False, endpoint_u=0., endpoint_v=0.,
                  endpoint_confidence=0., endpoint_clear_pixels=0.,
                  endpoint_reason='No qualified supported long path')

    def reject(reason):
        result['endpoint_reason'] = reason
        return result

    gray, prob, dark = frame_fields
    points = np.asarray(observed_points, dtype=float)
    if (not path_valid or points.ndim != 2 or points.shape[1:] != (2,) or
            len(points) < 8 or not np.isfinite(points).all()):
        return result
    if gray.shape != (360, 640) or prob.shape != gray.shape or dark.shape != gray.shape:
        return reject('Uncalibrated endpoint evidence fields')
    tail = points[-8:] / SCALE
    tip_u, tip_v = tail[-1]
    row = int(round(tip_v))
    col = int(round(tip_u))
    # A track ending at the search boundary is an ROI cutoff, NOT a cap.
    # Require 72 native pixels of inspectable floor inside the tracking ROI.
    if (row - ROI_TOP < 24 or not ROI_LEFT + 12 <= col < ROI_RIGHT - 12 or
            not 1 <= row < 359):
        return reject('Tracked far tip touches endpoint inspection ROI boundary')
    if not dark[row, col]:
        return reject('Tracked far point has no observed RGB tape')
    left, right = col, col + 1
    while left > 0 and dark[row, left - 1]:
        left -= 1
    while right < 640 and dark[row, right]:
        right += 1
    width = right - left
    if (width < 2 or width > 60 or left < ROI_LEFT + 6 or
            right > ROI_RIGHT - 6):
        return reject('Far-tip tape width or side boundary is ambiguous')
    if (float(np.median(prob[row, left:right])) < .95 or
            float(np.mean(prob[row, left:right] >= .90)) < .60):
        return reject('Far cap lacks broad high-confidence neural support')
    if np.ptp(tail[:, 1]) < 3.:
        return reject('Insufficient observed far-tip tangent')
    slope, _ = np.polyfit(tail[:, 1], tail[:, 0], 1)
    if not np.isfinite(slope) or abs(slope) > 1.5:
        return reject('Far-tip tangent too oblique for conservative cap evidence')
    # The actual observed cap must lie within one tracker row step (6 px).
    # If a dark line still exists ahead, a network miss/width gate stopped
    # tracking: it is NOT evidence that the physical tape ended.
    transition = None
    core_half = max(1, min(4, int(width * .25)))
    for distance in range(1, 5):
        v = row - distance
        u = int(round(tip_u - slope * distance))
        if u - core_half < 0 or u + core_half >= 640:
            return reject('Far-tip corridor leaves image')
        if not np.any(dark[v, u - core_half:u + core_half + 1]):
            transition = distance
            break
    if transition is None or transition > 3:
        return reject('RGB tape continues beyond supported neural path')

    # Inspect a widening corridor through visible FLOOR for 144 native px.
    # This deliberately ignores neural confidence when vetoing dark RGB
    # continuation, so a glare gap followed by missed segmentation cannot
    # masquerade as a terminal cap. Wider nearby branches/occluders veto too.
    max_distance = 48
    if row - max_distance < 8:
        return reject('Insufficient image floor beyond far tip')
    clear_rows = 0
    min_floor_quality = 1.
    for distance in range(transition + 2, max_distance + 1):
        v = row - distance
        u = int(round(tip_u - slope * distance))
        half = max(4, int(np.ceil(width * .60))) + int(distance * .08)
        margin = max(4, width // 2)
        if u - half - margin < ROI_LEFT or u + half + margin >= ROI_RIGHT:
            return reject('Endpoint clearance corridor reaches side ROI boundary')
        corridor = slice(u - half, u + half + 1)
        # Any appreciable dark patch here is an unseen continuation or an
        # obstacle, including continuations whose network support is absent.
        if np.count_nonzero(dark[v, corridor]) >= 2:
            return reject('RGB continuation, gap or occluding object beyond cap')
        center = gray[v, corridor]
        flanks = np.r_[gray[v, u - half - margin:u - half],
                       gray[v, u + half + 1:u + half + margin + 1]]
        background = float(np.median(flanks))
        brightness = float(np.median(center))
        # Reject dark regions, glare/bright coverings, and strong patch edges
        # instead of calling an unknown appearance "clear observed floor".
        if (background < 40. or abs(brightness - background) > .12 * background or
                float(np.ptp(center)) > .20 * background):
            return reject('Floor beyond cap is photometrically ambiguous or occluded')
        if np.any(prob[v, corridor] >= .50):
            return reject('Neural continuation or uncertain floor beyond cap')
        quality = 1. - abs(brightness - background) / max(1., background)
        min_floor_quality = min(min_floor_quality, quality)
        clear_rows += 1
    if clear_rows < 24:
        return reject('Too little independently observed clear floor beyond cap')
    # A long interior gap can look exactly like the local clear-floor cap.
    # Search substantially farther into the visible image for contradictory
    # tape BEFORE accepting. This region is veto-only: it is neither added to
    # the path nor required to have neural support. Do not follow a distant
    # candidate. Repeated RGB evidence is needed to avoid single-pixel noise.
    continuation_hits = 0
    for distance in range(max_distance + 2, min(144, row - 12) + 1, 2):
        v = row - distance
        u = int(round(tip_u - slope * distance))
        half = max(6, int(np.ceil(width * .70))) + int(distance * .12)
        if u - half < 0 or u + half >= 640:
            return reject('Distant endpoint contradiction corridor leaves image')
        hit = np.count_nonzero(dark[v, u - half:u + half + 1]) >= 2
        continuation_hits = continuation_hits + 1 if hit else 0
        if continuation_hits >= 2:
            return reject('Distant RGB continuation after a long interior gap')
    result.update(endpoint_candidate=True,
                  # This is an already-observed supported point, never the
                  # extrapolated tangent used for contradiction checks.
                  endpoint_u=float(points[-1, 0]), endpoint_v=float(points[-1, 1]),
                  endpoint_confidence=float(min(np.median(prob[row, left:right]), min_floor_quality)),
                  endpoint_clear_pixels=float(clear_rows * SCALE),
                  endpoint_reason='Observed interior cap with RGB/neural clear floor beyond same track')
    return result
