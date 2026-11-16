"""Recenter visible rear seam cross sections without extending the image mask."""
from dataclasses import replace

import cv2
import numpy as np


def recenter_visible_sections(result, spacing_px):
    """Replace skeleton corner hooks by centres of its selected mask component.

    Skeleton graph endpoints can follow the corners of a thick, clipped tape.
    Sample transverse pixel runs instead, always inside the same visible mask.
    Ambiguous folds are not bridged. Use only one continuous visible strip.
    """
    ordered = np.asarray(result.ordered_path_uv, dtype=float)
    diagnostics = dict(rear_centerline_method='skeleton', rear_cross_section_fallback=None)
    spans = np.ptp(ordered, axis=0)
    axis = int(np.argmax(spans))
    transverse = 1-axis
    if spans[axis] < 30 or spans[axis] < 2*spans[transverse]:
        diagnostics['rear_cross_section_fallback'] = 'visible component is not a single elongated strip'
        return result, diagnostics
    binary = (result.cleaned_mask > 0).astype(np.uint8)
    _, labels = cv2.connectedComponents(binary, connectivity=8)
    middle = np.rint(ordered[len(ordered)//2]).astype(int)
    label = labels[middle[1], middle[0]]
    if label == 0:
        diagnostics['rear_cross_section_fallback'] = 'skeleton midpoint is outside the visible mask'
        return result, diagnostics
    yy, xx = np.nonzero(labels == label)
    coordinates = (xx, yy)
    sections = []
    for station in range(int(np.min(ordered[:, axis])), int(np.max(ordered[:, axis]))+1):
        across = coordinates[transverse][coordinates[axis] == station]
        if len(across) < 3:
            continue
        across = np.sort(across)
        runs = sorted(np.split(across, np.flatnonzero(np.diff(across)>1)+1), key=len, reverse=True)
        if len(runs)>1 and len(runs[0]) < 2*len(runs[1]):
            continue
        across = runs[0]
        point = np.zeros(2)
        point[axis], point[transverse] = station, .5*(across[0]+across[-1])
        sections.append(point)
    if len(sections) < 30:
        diagnostics['rear_cross_section_fallback'] = 'too few observed cross sections'
        return result, diagnostics
    sections = np.asarray(sections)
    breaks = ((np.diff(sections[:, axis])>1)
              | (np.abs(np.diff(sections[:, transverse]))>2.))
    strips = np.split(sections, np.flatnonzero(breaks)+1)
    centers = max(strips, key=len)
    if len(centers) < 30:
        diagnostics['rear_cross_section_fallback'] = 'no continuous unambiguous visible strip'
        return result, diagnostics
    if ordered[-1, axis] < ordered[0, axis]:
        centers = centers[::-1].copy()
    cumulative = np.r_[0., np.cumsum(np.linalg.norm(np.diff(centers, axis=0), axis=1))]
    stations = np.r_[np.arange(0., cumulative[-1], spacing_px), cumulative[-1]]
    waypoints = np.column_stack([np.interp(stations, cumulative, centers[:, i]) for i in range(2)])
    # Rounding centres must still land on observed pixels, including the ends.
    pixels = np.rint(waypoints).astype(int)
    if not np.all(binary[pixels[:, 1], pixels[:, 0]]):
        diagnostics['rear_cross_section_fallback'] = 'resampled centre leaves the visible mask'
        return result, diagnostics
    diagnostics.update(rear_centerline_method='visible_mask_cross_sections',
                       rear_cross_section_count=len(centers),
                       rear_cross_section_discarded_count=len(sections)-len(centers),
                       rear_skeleton_path_length_px=float(result.path_length_px))
    return replace(result, ordered_path_uv=centers, waypoints_uv=waypoints,
                   path_length_px=float(cumulative[-1])), diagnostics
