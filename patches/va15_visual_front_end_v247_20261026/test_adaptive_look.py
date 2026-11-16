import unittest
from pathlib import Path

import numpy as np
import cv2

from long_tracker import LongTracker
from test_tracker import scene


class AdaptiveLookTests(unittest.TestCase):
    def test_native_uses_the_same_bounded_observed_rows(self):
        header=(Path(__file__).parent/'production/near_visual_guide.hpp').read_text()
        self.assertIn('NominalLookV=.76*1080.,MaximumLookV=.82*1080.',header)
        self.assertEqual(LongTracker.LOOK_V,.76*1080)
        self.assertEqual(LongTracker.MAX_LOOK_V,.82*1080)

    def test_supported_curve_missing_only_fixed_far_row_is_usable(self):
        rgb,mask,prob=scene()
        rgb[:833]=220;mask[:833]=0;prob[:833]=.001
        f=LongTracker().detect(rgb,mask,prob)
        self.assertTrue(f['long_valid'])
        self.assertGreater(f['look_v'],820.8)
        self.assertLessEqual(f['look_v'],885.6)
        points=np.asarray(f['path_points_uv'])
        self.assertLessEqual(points[-1,1],f['look_v'])
        self.assertGreaterEqual(f['path_vertical_span_px'],200.)
        self.assertGreaterEqual(f['look_confidence'],.9)
        self.assertLess(abs(f['look_u']-935.),3.)

    def test_unseen_or_short_curve_does_not_become_a_target(self):
        for end in [873,927]:
            rgb,mask,prob=scene();rgb[:end]=220;mask[:end]=0;prob[:end]=.001
            f=LongTracker().detect(rgb,mask,prob)
            self.assertFalse(f['long_valid'])

    def test_full_observation_keeps_nominal_row(self):
        f=LongTracker().detect(*scene())
        self.assertTrue(f['long_valid'])
        self.assertEqual(f['look_v'],820.8)

    def test_native_look_anchor_is_preserved_through_arc_resampling(self):
        image=np.full((1080,1920,3),220,np.uint8);mask=np.zeros((1080,1920),np.uint8)
        rows=np.arange(610,1080)
        cols=1035.+38.*np.sin((1080-rows)*np.pi/290.)
        points=np.c_[cols,rows].astype(np.int32)
        cv2.polylines(image,[points],False,(15,15,15),24)
        cv2.polylines(mask,[points],False,1,24)
        f=LongTracker().detect(image,mask,np.where(mask,.995,.001).astype(np.float32))
        self.assertTrue(f['long_valid'])
        curve=np.asarray(f['path_points_uv'])
        self.assertLess(np.min(np.abs(curve[:,1]-f['look_v'])),.001)
        u=np.interp(f['look_v'],curve[::-1,1],curve[::-1,0])
        self.assertLess(abs(u-f['look_u']),.001)


if __name__=='__main__':unittest.main()
