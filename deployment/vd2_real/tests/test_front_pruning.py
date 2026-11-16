"""Conservative component pruning preserves the exhaustive full-pixel result."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'python'),str(ROOT/'python/vision'),str(ROOT/'thirdparty/rhacracknet')]
from crack_trajectory import CenterlineConfig,extract_centerline
from vd1_visual_front import supported_front_centerline,component_distance_lower_bound

class FrontPruningTests(unittest.TestCase):
    def setUp(self):
        self.k=np.array([[460.,0,320],[0,460.,240],[0,0,1.]])
        self.tf=np.array([[0.,0.,1.,.39],[-1.,0.,0.,0.],[0.,-1.,0.,0.],[0.,0.,0.,1.]])
        self.cfg=CenterlineConfig(waypoint_spacing_px=6.)

    def compare(self,mask):
        reference=supported_front_centerline(mask,self.cfg,(320,311),extract_centerline,
            self.k,self.tf,[0,0,1,.55],nearest_ground=True,prune=False)
        extract=Mock(wraps=extract_centerline)
        result=supported_front_centerline(mask,self.cfg,(320,311),extract,
            self.k,self.tf,[0,0,1,.55],nearest_ground=True)
        for key in ('waypoints_uv','ordered_path_uv','skeleton','cleaned_mask'):
            np.testing.assert_array_equal(getattr(result,key),getattr(reference,key))
        return extract.call_count

    def test_far_clutter_is_skipped_without_changing_selected_pixels(self):
        mask=np.zeros((480,640),np.uint8)
        cv2.line(mask,(330,340),(330,465),255,6)
        for x in (40,120,200,460,540):cv2.line(mask,(x,338),(x,365),255,6)
        before=mask.copy();self.assertLess(self.compare(mask),6)
        np.testing.assert_array_equal(mask,before)

    def test_equal_distance_tie_and_horizon_crossing_keep_original_label_order(self):
        mask=np.zeros((480,640),np.uint8)
        for x in (270,370):cv2.line(mask,(x,300),(x,460),255,6)
        cv2.line(mask,(30,230),(30,470),255,6)
        self.compare(mask)
        self.assertEqual(component_distance_lower_bound([25,220,12,100,1200],mask.shape,
            self.k,self.tf,[0,0,1,.55]),0.)

    def test_curved_and_branched_near_path_stays_identical(self):
        for shift in (-80,0,80):
            mask=np.zeros((480,640),np.uint8)
            cv2.polylines(mask,[np.array([[330+shift,466],[300+shift,410],[350+shift,345]])],False,255,9)
            cv2.line(mask,(310+shift,430),(260+shift,405),255,6)
            for x in (40,560):cv2.line(mask,(x,340),(x,465),255,6)
            self.compare(mask)

if __name__=='__main__':unittest.main()
