"""Synthetic association tests only, not real segmentation accuracy."""
import unittest
import cv2
import numpy as np
from near_tracker import NearTracker


def scene(center=935, width=24, far_branch=False):
    image = np.full((1080, 1920, 3), 220, np.uint8)
    mask = np.zeros((1080, 1920), np.uint8)
    cv2.line(image, (center, 1080), (center, 0), (15, 15, 15), width)
    cv2.line(mask, (center, 1080), (center, 0), 1, width)
    if far_branch:
        cv2.line(image, (center, 880), (500, 300), (0, 0, 0), 30)
        cv2.line(mask, (center, 880), (500, 300), 1, 30)
    probability = np.where(mask, .995, .001).astype(np.float32)
    return image, mask, probability


class TrackerTests(unittest.TestCase):
    def test_near_line(self):
        tracker = NearTracker()
        for n in range(6):
            result = tracker.detect(*scene())
            self.assertEqual(result['stable_frames'], n + 1)
            self.assertLess(abs(result['near_u'] - 935), 3)
        self.assertGreater(result['native_width_px'], 20)

    def test_far_branch_cannot_steer(self):
        a = NearTracker().detect(*scene())
        b = NearTracker().detect(*scene(far_branch=True))
        self.assertLess(abs(a['near_u'] - b['near_u']), .01)

    def test_thin_tile_and_no_dark_line_rejected(self):
        for width in (1,):
            with self.assertRaises(ValueError):
                NearTracker().detect(*scene(width=width))
        image, mask, probability = scene()
        image[:] = 220
        with self.assertRaises(ValueError):
            NearTracker().detect(image, mask, probability)

    def test_line_switch_rejected(self):
        tracker = NearTracker()
        tracker.detect(*scene())
        for _ in range(3):
            with self.assertRaises(ValueError):
                tracker.detect(*scene(center=1020))
        self.assertEqual(tracker.streak, 0)

    def test_low_confidence_rejected(self):
        image, mask, probability = scene()
        probability[:] = .7
        with self.assertRaises(ValueError):
            NearTracker().detect(image, mask, probability)


if __name__ == '__main__':
    unittest.main()
