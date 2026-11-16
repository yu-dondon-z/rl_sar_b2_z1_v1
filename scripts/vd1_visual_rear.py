"""Measure the rear seam and ground without observing the welding gun.

The actor receives a desired tool position, not a visually measured TCP.
Keep the same RGB-D validity and foreground rejection as the original rear
pipeline, but do not require markers or produce TCP feedback measurements.
"""
from __future__ import annotations

from dataclasses import replace

import cv2
import numpy as np

from dual_camera_perception import (camera_geometry, fit_ground_plane,
                                    project_to_plane, rear_ground_roi, unproject)
from vd1_visual_front import SupportedFrontPerception


class SeamOnlyDualPerception(SupportedFrontPerception):
    """Front steering plus marker-independent rear RGB-D seam measurements."""

    def __init__(self, *args, gun_mask=None, rear_centerline_mode='skeleton', **kwargs):
        super().__init__(*args, **kwargs)
        if rear_centerline_mode not in ('skeleton', 'cross_sections'):
            raise ValueError('unknown rear centerline mode')
        self.rear_centerline_mode = rear_centerline_mode
        self._rear_centerline_diagnostics = {}
        self.gun_mask = gun_mask
        self._rear_allowed_pixels = None
        self._rear_geometry_context = None
        self._original_extract = self._extract_centerline
        self._extract_centerline = self._extract_with_roi

    def _extract_with_roi(self, mask, config, anchor):
        if self._rear_allowed_pixels is None:
            return self._original_extract(mask, config, anchor)
        from crack_trajectory.centerline import clean_binary_mask
        # Closing can reconnect gun-edge pixels across the exclusion boundary.
        # Apply that boundary again after closing and before skeletonization.
        mask = clean_binary_mask(mask, config)
        mask[~self._rear_allowed_pixels] = 0
        result = self._original_extract(mask, replace(config, morphology_kernel=0), anchor)
        if self.rear_centerline_mode == 'cross_sections':
            from vd5_rear_centerline import recenter_visible_sections
            result, diagnostics = recenter_visible_sections(result, config.waypoint_spacing_px)
            self._rear_centerline_diagnostics.update(diagnostics)
        return result

    def process_rear(self, metadata, rgb_bgr, depth):
        result = self._base_result(metadata)
        result.update(tool_target_mode='rear_seam_direct', visual_tcp_feedback=False,
                      requires_tcp_markers=False)
        self._rear_centerline_diagnostics = dict(rear_centerline_method='skeleton')
        try:
            intrinsic, transform = camera_geometry(metadata)
            depth_m = self._depth_m(metadata, rgb_bgr, depth)
            yy, xx = np.mgrid[0:depth_m.shape[0]:6, 0:depth_m.shape[1]:6]
            sampled = depth_m[yy, xx]
            valid = np.isfinite(sampled)
            points_b = unproject(np.column_stack((xx[valid], yy[valid])),
                                 sampled[valid], intrinsic, transform)
            plane, diagnostics = fit_ground_plane(points_b, self.config)
            result.update(diagnostics, ground_plane_b=plane.tolist())
            ground_pixels, diagnostics = rear_ground_roi(
                depth_m, intrinsic, transform, plane, self.config)
            result.update(diagnostics)
            if self.gun_mask is not None:
                gun_pixels, diagnostics = self.gun_mask.project(
                    metadata, intrinsic, transform, depth_m.shape)
                result.update(diagnostics)
                ground_pixels &= ~gun_pixels
            self._rear_allowed_pixels = ground_pixels
            # Read-only capture geometry for an explicitly selected versioned
            # component selector. The legacy anchor selector ignores it.
            self._rear_geometry_context = dict(intrinsic=intrinsic, transform=transform,
                depth_m=depth_m, plane=plane, allowed_pixels=ground_pixels)
            try:
                uv, confidence, diagnostics = self._centerline(rgb_bgr, .65, ground_pixels)
            finally:
                self._rear_allowed_pixels = None
                self._rear_geometry_context = None
            seam_depth_m = np.where(ground_pixels, depth_m, np.nan)
            depths = np.full(len(uv), np.nan)
            for i, (u, v) in enumerate(np.rint(uv).astype(int)):
                if 1 <= u < depth_m.shape[1]-1 and 1 <= v < depth_m.shape[0]-1:
                    if not ground_pixels[v, u]:
                        continue
                    patch = seam_depth_m[v-1:v+2, u-1:u+2]
                    good = patch[np.isfinite(patch)]
                    if len(good) >= 3:
                        depths[i] = np.median(good)
            valid = np.isfinite(depths)
            ratio = float(np.mean(valid))
            if ratio < self.config.min_valid_depth_ratio:
                raise ValueError('rear seam depth support too low (%.3f)' % ratio)
            uv = uv[valid]
            points = unproject(uv, depths[valid], intrinsic, transform)
            points = project_to_plane(points, plane)
            # Validate only the local section used by rear_seam_target().
            # At 3--4 m, a few pixels can exceed 0.25 m after perspective
            # projection even though the target region remains continuous.
            local = ((points[:, 0] >= self.config.rear_validation_x_min_m)
                     & (points[:, 0] <= self.config.rear_validation_x_max_m))
            if np.count_nonzero(local) < self.config.min_path_points:
                raise ValueError('rear seam target section is too short')
            local_points = points[local]
            local_uv = uv[local]
            result.update(rear_candidate_waypoints_uv=local_uv.tolist(),
                          rear_candidate_x_range_m=[float(local_points[:, 0].min()),
                                                    float(local_points[:, 0].max())],
                          rear_candidate_path_length_m=float(np.linalg.norm(
                              np.diff(local_points, axis=0), axis=1).sum()))
            self._path_result(result, local_points, local_uv, confidence)
            result.update(diagnostics, valid=True, reason='ok', valid_3d_ratio=ratio,
                          geometry_source='rear_rgb_seam_and_aligned_depth',
                          rear_validation_x_range_m=[self.config.rear_validation_x_min_m,
                                                     self.config.rear_validation_x_max_m],
                          rear_validation_point_count=int(len(local_points)))
        except (KeyError, ValueError, RuntimeError, cv2.error) as error:
            result['reason'] = str(error)
        result.update(self._rear_centerline_diagnostics)
        return result
