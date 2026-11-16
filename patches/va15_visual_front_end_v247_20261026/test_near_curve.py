import unittest

import cv2
import numpy as np

from long_tracker import LongTracker
from near_tracker import fit_near_curve


class NearCurveTests(unittest.TestCase):
    def test_smooth_bend_is_not_an_irregular_straight_line(self):
        rows=np.arange(1047.,926.,-6.)
        cols=980.+.006*(rows-987.)**2
        coeff,rms,degree=fit_near_curve(np.c_[cols,rows])
        self.assertEqual(degree,2)
        self.assertLess(rms,1.e-6)
        self.assertLess(abs(float(np.polyval(coeff,972.))-(980.+.006*15.**2)),1.e-6)

    def test_jagged_or_excessive_bend_is_still_rejected(self):
        rows=np.arange(1047.,926.,-6.)
        for cols in [980.+np.where(np.arange(len(rows))%2,12.,-12.),
                     980.+.012*(rows-987.)**2]:
            with self.assertRaisesRegex(ValueError,'too irregular'):
                fit_near_curve(np.c_[cols,rows])

    def test_pixel_path_keeps_observed_bend_and_rejects_a_new_tape(self):
        def scene(offset=0.):
            rgb=np.full((1080,1920,3),220,np.uint8)
            mask=np.zeros((1080,1920),np.uint8)
            v=np.arange(610,1080)
            u=935.+.006*(np.maximum(v,927.)-987.)**2+offset
            curve=np.c_[u,v].astype(np.int32)
            cv2.polylines(rgb,[curve],False,(15,15,15),24)
            cv2.polylines(mask,[curve],False,1,24)
            return rgb,mask,np.where(mask,.995,.001).astype(np.float32)
        tracker=LongTracker()
        f=tracker.detect(*scene(),timestamp_ns=1_000_000_000)
        self.assertEqual(tracker.debug['near_fit_degree'],2)
        self.assertTrue(f['long_valid'])
        self.assertLess(f['native_fit_rms_px'],3.)
        with self.assertRaisesRegex(ValueError,'Near tape changed'):
            tracker.detect(*scene(offset=100.),timestamp_ns=1_200_000_000)


if __name__=='__main__':unittest.main()
