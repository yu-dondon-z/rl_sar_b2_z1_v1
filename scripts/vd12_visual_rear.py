"""VD12-only selection among independently observed rear seam components.

Each candidate keeps the original segmentation, depth, path and work-point
bounds. No components, image pixels, depths or historical observations are
joined. Front extraction and the existing VD5 entry points are unchanged.
"""
from dataclasses import replace
import math

import cv2
import numpy as np

from dual_camera_perception import project_to_plane, unproject
from vd1_visual_rear import SeamOnlyDualPerception
from vd5_visual_control import rear_visual_target


class VD12RearSupportPerception(SeamOnlyDualPerception):
    def __init__(self, *args, trailing_distance=.8, max_occlusion_extension=.05, **kwargs):
        super().__init__(*args, **kwargs)
        self.trailing_distance = float(trailing_distance)
        self.max_occlusion_extension = float(max_occlusion_extension)
        if (not math.isfinite(self.trailing_distance) or self.trailing_distance <= 0.
                or not math.isfinite(self.max_occlusion_extension)
                or not 0. <= self.max_occlusion_extension <= .05):
            raise ValueError('rear component work-point bounds are invalid')

    def _candidate_geometry(self, trajectory, context, diagnostics):
        """Replay the parent's original RGB-D/path checks before target checks."""
        uv = np.asarray(trajectory.waypoints_uv, dtype=float)
        if len(uv) < self.config.min_path_points or trajectory.path_length_px < 30:
            raise ValueError('seam centerline is too short')
        depth = context['depth_m']
        allowed = context['allowed_pixels']
        seam_depth = np.where(allowed, depth, np.nan)
        depths = np.full(len(uv), np.nan)
        for i, (u, v) in enumerate(np.rint(uv).astype(int)):
            if 1 <= u < depth.shape[1]-1 and 1 <= v < depth.shape[0]-1 and allowed[v, u]:
                good = seam_depth[v-1:v+2, u-1:u+2]
                good = good[np.isfinite(good)]
                if len(good) >= 3:
                    depths[i] = np.median(good)
        valid = np.isfinite(depths)
        ratio = float(np.mean(valid))
        diagnostics['valid_3d_ratio'] = ratio
        if ratio < self.config.min_valid_depth_ratio:
            raise ValueError('rear seam depth support too low (%.3f)' % ratio)
        uv = uv[valid]
        points = project_to_plane(unproject(uv, depths[valid], context['intrinsic'],
                                           context['transform']), context['plane'])
        local = ((points[:, 0] >= self.config.rear_validation_x_min_m)
                 & (points[:, 0] <= self.config.rear_validation_x_max_m))
        if np.count_nonzero(local) < self.config.min_path_points:
            raise ValueError('rear seam target section is too short')
        points, uv = points[local], uv[local]
        diagnostics.update(point_count=int(len(points)),
                           path_length_m=float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
                           x_range_m=[float(points[:, 0].min()), float(points[:, 0].max())])
        # Confidence is intentionally NOT invented here. The parent's _centerline
        # computes the unchanged probability threshold on the selected real pixels.
        checked = {}
        self._path_result(checked, points, uv, 0.)
        diagnostics['observed_rgbd_path_validated'] = True
        _, target = rear_visual_target(points, context['plane'], self.trailing_distance,
                                       self.max_occlusion_extension)
        direct = bool(target['rear_target_directly_observed'])
        extension = float(target.get('rear_target_near_field_extension_m', 0.))
        extension += float(target.get('rear_target_occlusion_extension_m', 0.))
        diagnostics.update(target_supported=True, directly_observed=direct,
                           target_extension_m=extension, target_source=target['rear_target_source'])
        return (0 if direct else 1, extension, -diagnostics['path_length_m'])

    def _extract_with_roi(self, mask, config, anchor):
        context = getattr(self, '_rear_geometry_context', None)
        if context is None or self._rear_allowed_pixels is None:
            return super()._extract_with_roi(mask, config, anchor)
        from crack_trajectory.centerline import clean_binary_mask
        from vd5_rear_centerline import recenter_visible_sections

        cleaned = clean_binary_mask(mask, config)
        cleaned[~self._rear_allowed_pixels] = 0
        count, labels, stats, _ = cv2.connectedComponentsWithStats(cleaned, connectivity=8)
        extraction_config = replace(config, morphology_kernel=0)
        candidates = []

        def extract(component, label, original=False, trajectory=None):
            info = dict(component_id=int(label), original_anchor=bool(original),
                        target_supported=False, accepted=False)
            candidates.append(info)
            try:
                if trajectory is None:
                    trajectory = self._original_extract(component, extraction_config, anchor)
                center_diagnostics = dict(rear_centerline_method='skeleton')
                if self.rear_centerline_mode == 'cross_sections':
                    trajectory, center_diagnostics = recenter_visible_sections(
                        trajectory, config.waypoint_spacing_px)
                score = self._candidate_geometry(trajectory, context, info)
                info.update(accepted=True, reason=(
                    'observed RGB-D path valid; target deferred to motion-compensated control'
                    if info.get('target_validation_deferred_to_control') else
                    'original RGB-D, path and target bounds passed'))
                return trajectory, center_diagnostics, score, info
            except (KeyError, ValueError, RuntimeError, cv2.error) as error:
                info['reason'] = str(error)
                return None

        # Preserve a usable original result (including its existing estimate
        # semantics); try another component only when the original cannot work.
        original_result, original_error = None, None
        original_label = 0
        try:
            original_result = self._original_extract(cleaned, extraction_config, anchor)
            middle = np.rint(original_result.ordered_path_uv[len(original_result.ordered_path_uv)//2]).astype(int)
            if 0 <= middle[1] < labels.shape[0] and 0 <= middle[0] < labels.shape[1]:
                original_label = int(labels[middle[1], middle[0]])
        except (ValueError, RuntimeError, cv2.error) as error:
            original_error = error
        chosen = (extract(cleaned, original_label, True, original_result)
                  if original_result is not None else None)
        if chosen is None:
            # Preserve a valid observed original path even when VD14 defers
            # work-point validation: capture-time target support must not
            # force a component switch before capture-to-control transport.
            supported = []
            for label in range(1, count):
                if label == original_label:
                    continue
                selected = extract((labels == label).astype(np.uint8)*255, label)
                if selected is not None:
                    supported.append(selected)
            if supported:
                chosen = min(supported, key=lambda item: (*item[2], item[3]['component_id']))
        if chosen is not None:
            trajectory, center_diagnostics, _, selected = chosen
            self._rear_centerline_diagnostics.update(center_diagnostics)
            self._rear_centerline_diagnostics.update(
                rear_component_policy='independent_components_original_rgbd_path_target_bounds',
                rear_component_selected_id=selected['component_id'],
                rear_component_original_id=original_label,
                rear_component_fallback=not selected['original_anchor'],
                rear_component_target_supported=bool(selected['target_supported']),
                rear_component_target_validation_deferred=bool(
                    selected.get('target_validation_deferred_to_control', False)),
                rear_component_candidates=candidates)
            return trajectory
        # Do not change legacy short-path reasons or discard a path which the
        # existing observed-memory controller might still use. Outer validation
        # remains authoritative; no failed candidate is relabelled as accepted.
        self._rear_centerline_diagnostics.update(
            rear_component_policy='independent_components_original_rgbd_path_target_bounds',
            rear_component_selected_id=original_label,
            rear_component_original_id=original_label,
            rear_component_fallback=False, rear_component_target_supported=False,
            rear_component_candidates=candidates)
        if original_result is None:
            raise original_error or ValueError('seam centerline is too short')
        if self.rear_centerline_mode == 'cross_sections':
            original_result, diagnostics = recenter_visible_sections(original_result, config.waypoint_spacing_px)
            self._rear_centerline_diagnostics.update(diagnostics)
        return original_result
