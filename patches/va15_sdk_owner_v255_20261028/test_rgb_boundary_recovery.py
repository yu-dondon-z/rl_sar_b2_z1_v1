"""Keep full RGB boundaries during reflected-tape recovery, not overlap veto."""
import unittest
import cv2
import numpy as np
from long_tracker import LongTracker,clips_strong_rgb_region


class RGBBoundaryRecovery(unittest.TestCase):
    def test_identical_overlap_is_not_clipping(self):
        row=np.zeros(40,dtype=bool);row[10:18]=True
        self.assertFalse(clips_strong_rgb_region(row,10,18))
        self.assertFalse(clips_strong_rgb_region(row,8,20))

    def test_fragment_of_broad_dark_seam_still_rejected(self):
        row=np.zeros(40,dtype=bool);row[4:30]=True
        for a,b in [(10,18),(4,18),(10,30)]:
            self.assertTrue(clips_strong_rgb_region(row,a,b))

    def test_separate_unrelated_region_is_not_clipped(self):
        row=np.zeros(40,dtype=bool);row[4:7]=True;row[12:18]=True
        self.assertFalse(clips_strong_rgb_region(row,10,20))

    def test_border_test_matches_full_component_containment(self):
        rng=np.random.RandomState(253)
        for _ in range(200):
            row=rng.uniform(size=40)>.4
            a,b=sorted(rng.choice(41,2,replace=False))
            changes=np.diff(np.r_[False,row,False].astype(np.int8))
            expected=any(start<b and end>a and (start<a or end>b)
                for start,end in zip(np.flatnonzero(changes==1),np.flatnonzero(changes==-1)))
            self.assertEqual(clips_strong_rgb_region(row,a,b),expected)

    def test_neural_good_full_rgb_boundary_recovers_local_glare(self):
        # Wide closing background is 250; nearby floor is only 200. Tape160
        # is "strong" relative to closing background but only .20 darker
        # than the local flanks. Same full RGB component, NOT neural trimming.
        # Pixel-aligned boundaries avoid neural-resize edge fragments: this
        # test specifically concerns an IDENTICAL weak/strong component.
        image=np.full((360,640,3),220,dtype=np.uint8)
        image[:290]=250;image[:290,295:329]=200
        image[:,308:316]=15;image[:290,308:316]=160
        mask=np.zeros((360,640),dtype=np.uint8);mask[:,308:316]=1
        image=cv2.resize(image,(1920,1080),interpolation=cv2.INTER_NEAREST)
        mask=cv2.resize(mask,(1920,1080),interpolation=cv2.INTER_NEAREST)
        prob=np.where(mask,.995,.001).astype(np.float32)
        features=LongTracker().detect(image,mask,prob)
        self.assertTrue(features['long_valid'])
        self.assertGreater(features['reflection_recovered_rows'],0)
        self.assertLess(abs(features['look_u']-935.),3.)


if __name__=='__main__':unittest.main()
