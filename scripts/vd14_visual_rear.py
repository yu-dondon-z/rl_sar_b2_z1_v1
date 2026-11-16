"""VD14 rear RGB evidence gate, not a replacement for the learned detector.

The deployed scene assumes a dark seam on a lighter surface. Require darker
pixels with brighter support on BOTH transverse sides: a uniform hallucinated
stripe or a single gun/lighting edge is not a measured dark ridge. Intersect
this evidence with RHANet and the original depth/self-exclusion ROI, never
create seam pixels or change neural probabilities. Not validated for bright
seams, extreme low contrast or arbitrary real-world textures.
"""
import cv2
import numpy as np

from vd12_visual_rear import VD12RearSupportPerception


CONTRAST_LEVELS = 12
RADII_PX = (3, 6, 12, 20)


def dark_ridge_support(image, allowed):
    """Bilateral contrast across four directions/scales, no image wrapping."""
    image = np.asarray(image)
    allowed = np.asarray(allowed, dtype=bool)
    if (image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3
            or image.shape[:2] != allowed.shape):
        raise ValueError('Rear RGB evidence needs uint8 BGR and a matching surface ROI')
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(np.int16)
    height, width = gray.shape
    supported = np.zeros(allowed.shape, dtype=bool)
    for radius in RADII_PX:
        for dx, dy in ((radius, 0), (0, radius), (radius, radius), (radius, -radius)):
            ax, ay = abs(dx), abs(dy)
            if height <= 2*ay or width <= 2*ax:
                continue
            ys, xs = slice(ay, height-ay), slice(ax, width-ax)
            plus = (slice(ay+dy, height-ay+dy), slice(ax+dx, width-ax+dx))
            minus = (slice(ay-dy, height-ay-dy), slice(ax-dx, width-ax-dx))
            brighter = np.minimum(gray[plus], gray[minus])-gray[ys, xs] >= CONTRAST_LEVELS
            supported[ys, xs] |= brighter & allowed[plus] & allowed[minus]
    return supported & allowed


class VD14RearEvidencePerception(VD12RearSupportPerception):
    """The original RGB-D/component/working-envelope checks plus RGB evidence."""

    def _candidate_geometry(self, trajectory, context, diagnostics):
        try:
            return super()._candidate_geometry(trajectory, context, diagnostics)
        except ValueError as error:
            if not diagnostics.get('observed_rgbd_path_validated'):
                raise
            # Observations are still in CAPTURE coordinates. A section which
            # does not cover the capture-time work point can cover it after
            # sensor motion transport, or contribute to the existing bounded
            # observed-path cache. Publish a measured section, not an invented
            # target. ALL original target bounds remain in the controller.
            diagnostics.update(target_supported=False,
                target_validation_deferred_to_control=True,
                capture_time_target_rejection=str(error))
            return (2, float('inf'), -diagnostics['path_length_m'])

    def _centerline(self, image, anchor_v_ratio, allowed_pixels=None):
        if self._rear_geometry_context is None:
            # Front steering retains the shared implementation.
            return super()._centerline(image, anchor_v_ratio, allowed_pixels)
        original = self._rear_allowed_pixels
        context_allowed = self._rear_geometry_context['allowed_pixels']
        supported = dark_ridge_support(image, original)
        self._rear_centerline_diagnostics.update(
            rear_rgb_evidence='bilateral_dark_ridge_intersected_with_detector_and_surface_ROI',
            rear_rgb_evidence_contrast_levels=CONTRAST_LEVELS,
            rear_rgb_evidence_radii_px=list(RADII_PX),
            rear_rgb_evidence_area_ratio=float(np.mean(supported)),
            rear_rgb_evidence_assumption='dark_seam_brighter_surface_on_both_sides',
            rear_detector_probability_modified=False)
        # The parent repeats this ROI AFTER morphological closing. Thus closing
        # cannot regenerate pixels rejected for lack of measured RGB support.
        self._rear_allowed_pixels = supported
        self._rear_geometry_context['allowed_pixels'] = supported
        try:
            return super()._centerline(image, anchor_v_ratio, supported)
        finally:
            self._rear_allowed_pixels = original
            self._rear_geometry_context['allowed_pixels'] = context_allowed
