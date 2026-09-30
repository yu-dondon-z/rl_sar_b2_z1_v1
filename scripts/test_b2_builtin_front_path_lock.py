#!/usr/bin/env python3
"""Offline regression checks for the real B2 vision path selector."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
RHACRACKNET_ROOT = Path(
    __import__("os").environ.get(
        "RL_SAR_RHACRACKNET_ROOT", "/home/dong/RHACrackNet/code"))
sys.path.insert(0, str(RHACRACKNET_ROOT))

from crack_trajectory import CenterlineConfig, extract_centerline  # noqa: E402


def load_bridge_module():
    path = REPO_ROOT / "scripts/run_b2_builtin_front_rhacracknet_bridge.py"
    spec = importlib.util.spec_from_file_location("real_vision_bridge", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


BRIDGE = load_bridge_module()


def make_mask(include_tape=True, include_grout=True, tape_shift=0):
    mask = np.zeros((360, 640), dtype=np.uint8)
    if include_tape:
        cv2.line(
            mask, (250 + tape_shift, 352), (270 + tape_shift, 170), 255, 11)
    if include_grout:
        cv2.line(mask, (390, 352), (380, 160), 255, 2)
    return mask


def select(mask, reference=None, width=5.0, lock_error=35.0):
    return BRIDGE.extract_floor_path(
        mask,
        CenterlineConfig(waypoint_spacing_px=8.0),
        extract_centerline,
        min_path_length_px=100.0,
        min_forward_span_px=100.0,
        bottom_entry_fraction=0.95,
        max_lateral_to_forward_ratio=1.25,
        near_tail_vertical_tolerance_px=5.0,
        min_near_path_width_px=width,
        reference_path_uv=reference,
        temporal_lock_max_error_px=lock_error,
    )


class PathLockTest(unittest.TestCase):
    def test_unlocked_selector_prefers_wide_tape_over_thin_grout(self):
        centerline, _, path_width, _ = select(make_mask())
        self.assertLess(float(np.mean(centerline.waypoints_uv[:, 0])), 300.0)
        self.assertGreater(path_width, 5.0)

    def test_locked_selector_tracks_small_frame_to_frame_motion(self):
        initial, _, _, _ = select(make_mask())
        updated, _, _, error = select(
            make_mask(tape_shift=8), reference=initial.waypoints_uv)
        self.assertLess(float(np.mean(updated.waypoints_uv[:, 0])), 310.0)
        self.assertLess(error, 35.0)

    def test_locked_selector_refuses_jump_to_other_line(self):
        initial, _, _, _ = select(make_mask())
        with self.assertRaisesRegex(RuntimeError, "refusing automatic line switch"):
            select(
                make_mask(include_tape=False, include_grout=True),
                reference=initial.waypoints_uv,
                width=0.0,
            )

    def test_thin_grout_alone_is_not_acquired(self):
        with self.assertRaisesRegex(RuntimeError, "no sufficiently wide"):
            select(make_mask(include_tape=False, include_grout=True))


if __name__ == "__main__":
    unittest.main()
