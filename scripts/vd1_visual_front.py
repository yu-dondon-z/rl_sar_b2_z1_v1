"""Choose front crack components by measured path support, not image centre.

An acute bend can move the crack away from the fixed central image anchor.
Keep the original RHANet mask, confidence, camera projection and rear pipeline.
Only the selection of the front mask's connected component changes here.
"""
from __future__ import annotations

import cv2
import numpy as np

from dual_camera_perception import (DualCameraPerception, camera_geometry,
                                    intersect_pixels_with_plane, normalize_plane)


def component_distance_lower_bound(stat, shape, intrinsic, transform, plane):
    """Conservative XY distance bound; no thinning or new seam points.

    Projection of a positive-depth pixel rectangle onto a plane stays inside
    the XY bounds of its projected corners. Smoothed/resampled centerlines
    stay inside the original rectangle. A one-pixel halo also covers rounding.
    If any corner crosses the horizon, no finite bound is claimed.
    """
    x,y,w,h=map(int,stat[:4])
    x0=max(0,x-1);x1=min(shape[1]-1,x+w)
    y0=max(0,y-1);y1=min(shape[0]-1,y+h)
    points,valid=intersect_pixels_with_plane(
        np.array([[x0,y0],[x1,y0],[x1,y1],[x0,y1]],dtype=float),
        intrinsic,transform,plane)
    if not np.all(valid) or not np.isfinite(points).all():return 0.
    low=points[:,:2].min(axis=0);high=points[:,:2].max(axis=0)
    distance=np.maximum(np.maximum(low,-high),0.)
    return max(0.,float(distance@distance)-1e-8)


def supported_front_centerline(mask, config, anchor_uv, extract, intrinsic, transform, plane,
                               nearest_ground=False,prune=True):
    """Prefer a long visible metric section over a nearby false image stripe."""
    from crack_trajectory.centerline import clean_binary_mask

    cleaned = clean_binary_mask(mask, config)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(cleaned, connectivity=8)
    candidates = []
    labels_to_try=[label for label in range(1,count)
                   if stats[label,cv2.CC_STAT_HEIGHT]>=12]
    bounds={label:component_distance_lower_bound(stats[label],mask.shape,intrinsic,transform,plane)
            for label in labels_to_try} if nearest_ground and prune else {}
    if bounds:labels_to_try.sort(key=lambda label:(bounds[label],label))
    best_distance=float('inf')
    for label in labels_to_try:
        # The original cleaner already enforces minimum area. Exclude flat
        # transverse fragments before fitting metric forward support.
        if bounds.get(label,0.)>best_distance+1e-8:continue
        component = (labels == label).astype(np.uint8)*255
        try:
            chosen_anchor=anchor_uv
            if nearest_ground:
                v,u=np.nonzero(component)
                uv_pixels=np.column_stack((u,v))
                projected,valid=intersect_pixels_with_plane(uv_pixels,intrinsic,transform,plane)
                valid&=(projected[:,0]>.1)&(projected[:,0]<=3.)
                ids=np.flatnonzero(valid)
                if not len(ids):continue
                near=ids[np.argmin(np.sum(projected[ids,:2]**2,axis=1))]
                chosen_anchor=tuple(uv_pixels[near])
            trajectory = extract(component, config, chosen_anchor)
            uv = np.asarray(trajectory.waypoints_uv, dtype=float)
            points, valid = intersect_pixels_with_plane(uv, intrinsic, transform, plane)
            points = points[valid]
            if nearest_ground:
                points=points[(points[:,0]>.1)&(points[:,0]<=3.)]
            if len(points) < 6 or np.ptp(points[:, 0]) < .30:
                continue
            x = np.sort(points[:, 0])
            # Favor coherent forward coverage, not a long transverse edge.
            span = float(x[-1]-x[0])
            distance = float(np.min(np.sum((uv-np.asarray(anchor_uv))**2, axis=1)))
            if nearest_ground:
                near=points[np.argmin(np.sum(points[:,:2]**2,axis=1))]
                # A distant side edge must not become a near path merely
                # because its extrapolated cubic intercept happens to pass.
                if abs(near[1])>.8:continue
                score=(-float(np.dot(near[:2],near[:2])),span)
            else:score=(span,-distance)
            candidates.append((score, label, trajectory))
            if nearest_ground:best_distance=min(best_distance,-score[0])
        except (ValueError, RuntimeError, cv2.error):
            continue
    if not candidates:
        raise ValueError('front seam has no reachable near-ground component' if nearest_ground else
                         'front seam has no component with 0.30 m forward support')
    # Preserve the original label-order tie break when the bound changes the
    # evaluation order. Every potentially better component is still evaluated.
    return max(candidates, key=lambda item: (item[0],-item[1]))[2]


class SupportedFrontPerception(DualCameraPerception):
    """Keep upstream rear processing and all image validity thresholds."""

    def process_front(self, metadata, rgb_bgr):
        original = self._extract_centerline
        try:
            intrinsic, transform = camera_geometry(metadata)
            plane = normalize_plane(metadata['ground_plane_b'])
        except (KeyError, ValueError):
            return super().process_front(metadata, rgb_bgr)

        def extract(mask, config, anchor_uv):
            return supported_front_centerline(mask, config, anchor_uv, original,
                intrinsic, transform, plane, nearest_ground=independent)

        independent=metadata.get('ground_plane_source')=='imu_gravity_and_configured_nominal_height'
        self._extract_centerline = extract
        try:
            result = super().process_front(metadata, rgb_bgr)
            result['front_component_selection'] = ('nearest_reachable_ground_support' if independent else
                                                   'longest_measured_forward_support')
            return result
        finally:
            self._extract_centerline = original
