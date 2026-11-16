"""RGB + neural evidence on the SAME observed tape, no stale extrapolation."""
import unittest
import cv2
import numpy as np
from long_tracker import LongTracker
from test_tracker import scene


def reflected(value=180, probability=.995):
    image,mask,prob=scene()
    # Brightened tape (still measured darker than the floor), far from the
    # unchanged strong near acquisition strip. The old .25 mask cuts here.
    region=(mask>0)&(np.indices(mask.shape)[0]<870)
    image[region]=value
    prob[region]=probability
    return image,mask,prob


class ReflectionTests(unittest.TestCase):
    def test_broad_high_confidence_reflected_tape_continues(self):
        f=LongTracker().detect(*reflected())
        self.assertTrue(f['long_valid'])
        self.assertGreater(f['reflection_recovered_rows'],0)
        self.assertLess(abs(f['look_u']-935.),3.)
        self.assertGreater(f['path_vertical_span_px'],300.)
        self.assertFalse(f['endpoint_candidate'])

    def test_perfectly_blank_rgb_with_false_neural_tape_is_not_filled(self):
        f=LongTracker().detect(*reflected(value=220))
        self.assertFalse(f['long_valid'])
        self.assertEqual(f['reflection_recovered_rows'],0)

    def test_weak_neural_support_does_not_lower_rgb_gate(self):
        f=LongTracker().detect(*reflected(probability=.8))
        self.assertFalse(f['long_valid'])
        self.assertEqual(f['reflection_recovered_rows'],0)

    def test_strong_rgb_original_path_is_unchanged(self):
        f=LongTracker().detect(*scene())
        self.assertTrue(f['long_valid'])
        self.assertEqual(f['reflection_recovered_rows'],0)

    def test_near_acquisition_is_not_relaxed(self):
        image,mask,prob=scene()
        image[mask>0]=180
        with self.assertRaises(ValueError):LongTracker().detect(image,mask,prob)

    def test_reflected_side_path_cannot_replace_a_gap(self):
        image,mask,prob=scene()
        image[:870]=220;mask[:870]=0;prob[:870]=.001
        cv2.line(image,(1040,870),(1040,610),(180,180,180),24)
        cv2.line(prob,(1040,870),(1040,610),.995,24)
        f=LongTracker().detect(image,mask,prob)
        self.assertFalse(f['long_valid'])
        self.assertLess(max(p[0] for p in f['points']),940.)

    def test_real_cap_after_reflection_remains_a_visible_cap(self):
        image,mask,prob=reflected()
        # Return to strong RGB before the true cap; faint support is only
        # used in the middle, not invented past the physical tape end.
        image[(mask>0)&(np.indices(mask.shape)[0]<800)]=15
        image[:750]=220;mask[:750]=0;prob[:750]=.001
        f=LongTracker().detect(image,mask,prob)
        self.assertTrue(f['long_valid'])
        self.assertGreater(f['reflection_recovered_rows'],0)
        self.assertTrue(f['endpoint_candidate'],f['endpoint_reason'])
        self.assertGreaterEqual(f['points'][-1][1],750.)

    def test_faint_continuation_is_not_false_cap(self):
        f=LongTracker().detect(*reflected())
        self.assertFalse(f['endpoint_candidate'])


if __name__=='__main__':unittest.main()
