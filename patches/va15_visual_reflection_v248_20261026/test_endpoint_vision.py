"""Synthetic cap evidence tests; not an accuracy or hardware qualification."""
import unittest

import cv2
import numpy as np

from endpoint_vision import visible_endpoint
from long_tracker import LongTracker
from test_tracker import scene


def capped_scene(center=935, tip=780, width=24):
    image, mask, probability = scene(center, width)
    image[:tip] = 220
    mask[:tip] = 0
    probability[:tip] = .001
    return image, mask, probability


class EndpointVisionTests(unittest.TestCase):
    def test_genuine_cap_is_existing_same_track_point(self):
        result = LongTracker().detect(*capped_scene())
        self.assertTrue(result['long_valid'])
        self.assertTrue(result['endpoint_candidate'], result['endpoint_reason'])
        self.assertGreaterEqual(result['endpoint_confidence'], .95)
        self.assertGreaterEqual(result['endpoint_clear_pixels'], 72.)
        self.assertLess(abs(result['endpoint_u'] - 935.), 3.)
        self.assertLess(abs(result['endpoint_v'] - 780.), 7.)
        np.testing.assert_allclose([result['endpoint_u'], result['endpoint_v']], result['points'][-1])

    def test_continuous_rgb_tape_never_becomes_cap_at_search_cutoff(self):
        result = LongTracker().detect(*scene())
        self.assertTrue(result['long_valid'])
        self.assertFalse(result['endpoint_candidate'])

    def test_rgb_continuation_without_neural_support_is_not_cap(self):
        image, mask, probability = scene()
        mask[:780] = 0
        probability[:780] = .001
        result = LongTracker().detect(image, mask, probability)
        self.assertTrue(result['long_valid'])
        self.assertFalse(result['endpoint_candidate'])
        self.assertIn('RGB tape continues', result['endpoint_reason'])

    def test_glare_gap_with_continuation_is_not_terminal_cap(self):
        image, mask, probability = scene()
        image[735:780] = 220
        mask[735:780] = 0
        probability[735:780] = .001
        result = LongTracker().detect(image, mask, probability)
        self.assertTrue(result['long_valid'])
        self.assertFalse(result['endpoint_candidate'])
        self.assertIn('continuation', result['endpoint_reason'])

    def test_long_interior_gap_with_visible_continuation_is_rejected(self):
        image, mask, probability = scene()
        image[510:780] = 220
        mask[510:780] = 0
        probability[510:780] = .001
        result = LongTracker().detect(image, mask, probability)
        self.assertTrue(result['long_valid'])
        self.assertFalse(result['endpoint_candidate'])
        self.assertIn('Distant RGB continuation', result['endpoint_reason'])

    def test_distant_continuation_veto_does_not_require_network_support(self):
        image, mask, probability = scene()
        image[510:780] = 220
        mask[:780] = 0
        probability[:780] = .001
        result = LongTracker().detect(image, mask, probability)
        self.assertFalse(result['endpoint_candidate'])
        self.assertIn('Distant RGB continuation', result['endpoint_reason'])

    def test_bright_occluding_blob_is_not_observed_floor(self):
        image, mask, probability = scene()
        # A bright object covers the tape for longer than the inspection
        # corridor, but visibly differs from adjacent floor.
        image[400:780, 910:960] = 255
        mask[400:780, 910:960] = 0
        probability[400:780, 910:960] = .001
        result = LongTracker().detect(image, mask, probability)
        self.assertTrue(result['long_valid'])
        self.assertFalse(result['endpoint_candidate'])
        self.assertIn('ambiguous or occluded', result['endpoint_reason'])

    def test_dark_occluding_blob_is_not_cap(self):
        image, mask, probability = capped_scene()
        image[700:778, 890:975] = 30
        result = LongTracker().detect(image, mask, probability)
        self.assertFalse(result['endpoint_candidate'])

    def test_roi_cutoff_cannot_be_a_cap(self):
        image = np.full((1080,1920,3),220,np.uint8)
        mask = np.zeros((1080,1920),np.uint8)
        polygon = np.array([[923,1080],[947,1080],[939,630],[931,630]],np.int32)
        cv2.fillPoly(image,[polygon],(15,15,15))
        cv2.fillPoly(mask,[polygon],1)
        probability = np.where(mask,.995,.001).astype(np.float32)
        result = LongTracker().detect(image, mask, probability)
        self.assertFalse(result['endpoint_candidate'])
        self.assertIn('boundary', result['endpoint_reason'])

    def test_low_neural_confidence_at_far_cap_is_not_endpoint(self):
        image, mask, probability = capped_scene()
        probability[780:795] *= .93
        result = LongTracker().detect(image, mask, probability)
        self.assertFalse(result['endpoint_candidate'])

    def test_side_and_distant_cap_cannot_replace_the_active_track(self):
        image, mask, probability = scene(center=860)
        cv2.line(image, (1320, 700), (1320, 400), (15, 15, 15), 24)
        cv2.line(mask, (1320, 700), (1320, 400), 1, 24)
        cv2.line(probability, (1320, 700), (1320, 400), .995, 24)
        result = LongTracker().detect(image, mask, probability)
        self.assertTrue(result['long_valid'])
        self.assertFalse(result['endpoint_candidate'])
        self.assertLess(abs(result['look_u'] - 860), 3.)

    def test_short_cap_does_not_relax_existing_path_requirement(self):
        result = LongTracker().detect(*capped_scene(tip=900))
        self.assertFalse(result['long_valid'])
        self.assertFalse(result['endpoint_candidate'])

    def test_endpoint_near_side_boundary_is_rejected(self):
        result = LongTracker().detect(*capped_scene(center=420))
        self.assertFalse(result['endpoint_candidate'])

    def test_neural_floor_ambiguity_is_not_a_confirmed_cap(self):
        image, mask, probability = capped_scene()
        probability[690:720, 935:940] = .8
        result = LongTracker().detect(image, mask, probability)
        self.assertFalse(result['endpoint_candidate'])

    def test_fields_are_finite_and_no_candidate_for_invalid_path(self):
        tracker = LongTracker()
        features = tracker.detect(*capped_scene())
        result = visible_endpoint(tracker.frame_fields, features['points'], False)
        self.assertFalse(result['endpoint_candidate'])
        self.assertTrue(all(np.isfinite(result[key]) for key in
                            ('endpoint_u', 'endpoint_v', 'endpoint_confidence', 'endpoint_clear_pixels')))


if __name__ == '__main__':
    unittest.main()
