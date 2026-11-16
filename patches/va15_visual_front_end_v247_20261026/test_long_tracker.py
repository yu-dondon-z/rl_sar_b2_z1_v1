"""Pixel association regressions; synthetic masks are not network accuracy."""
import unittest
import cv2
import numpy as np
from long_tracker import LongTracker
from test_tracker import scene


class LongTrackerTests(unittest.TestCase):
    def test_long_supported_path_and_equal_spacing(self):
        result=LongTracker().detect(*scene(center=860))
        self.assertTrue(result['long_valid'])
        # Constant image width is intentionally stopped once it disagrees
        # with the perspective-width check; it still covers lookahead.
        self.assertGreater(result['path_vertical_span_px'],300)
        self.assertGreater(result['path_point_count'],40)
        points=np.asarray(result['path_points_uv'])
        self.assertTrue(np.all(np.diff(points[:,1])<=0))
        self.assertLess(np.max(np.linalg.norm(np.diff(points,axis=0),axis=1)),8.01)
        self.assertLess(abs(result['near_u']-860),3)
        self.assertLess(abs(result['look_u']-860),3)

    def test_short_path_not_extrapolated(self):
        image,mask,prob=scene()
        image[:870]=220;mask[:870]=0;prob[:870]=.001
        result=LongTracker().detect(image,mask,prob)
        self.assertFalse(result['long_valid'])
        self.assertGreater(result['points'][-1][1],860)

    def test_far_low_confidence_not_filled(self):
        image,mask,prob=scene();prob[:870]=.4
        result=LongTracker().detect(image,mask,prob)
        self.assertFalse(result['long_valid'])

    def test_gap_does_not_jump_to_a_seam(self):
        image,mask,prob=scene()
        image[840:875]=220;mask[840:875]=0;prob[840:875]=.001
        cv2.line(image,(1000,880),(1050,610),(0,0,0),3)
        cv2.line(prob,(1000,880),(1050,610),.999,3)
        result=LongTracker().detect(image,mask,prob)
        self.assertFalse(result['long_valid'])
        self.assertLess(max(p[0] for p in result['points']),940)

    def test_curved_far_path_not_replaced_by_a_line(self):
        image=np.full((1080,1920,3),220,np.uint8)
        mask=np.zeros((1080,1920),np.uint8)
        v=np.arange(610,1080)
        u=935+30*np.sin((1080-v)*np.pi/360)
        path=np.c_[u,v].astype(np.int32)
        cv2.polylines(image,[path],False,(15,15,15),24)
        cv2.polylines(mask,[path],False,1,24)
        prob=np.where(mask,.995,.001).astype(np.float32)
        result=LongTracker().detect(image,mask,prob)
        self.assertTrue(result['long_valid'])
        points=np.asarray(result['points'])
        self.assertGreater(np.ptp(points[:,0]),30)
        self.assertGreater(np.max(np.abs(points[:,0]-np.polyval(np.polyfit(points[:,1],points[:,0],1),points[:,1]))),10)
        self.assertLess(np.median(np.abs(points[:,0]-(935+30*np.sin((1080-points[:,1])*np.pi/360)))),3)


if __name__=='__main__':unittest.main()
