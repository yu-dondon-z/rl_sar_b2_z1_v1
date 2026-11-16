"""Front steering isolation, IMU provenance and expiry without robot IO."""
import multiprocessing as mp
from pathlib import Path
import sys
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'python'),str(ROOT/'python/vision')]
from front_projection import HISTORY_SIZE,publish_gravity,front_metadata,process_front
from vd1_front_path_estimator import FrontPathEstimator
from vd1_visual_guard import GuardedVD1VisualController
from vd1_control import SCHEMA


class FrontEstimationTests(unittest.TestCase):
    def setUp(self):
        self.history=mp.get_context('spawn').Array('d',4*HISTORY_SIZE,lock=True)
        self.meta=dict(capture_timestamp=10.,sequence=0,capture_generation=1)

    def test_uses_preceding_real_imu_receipt_and_preserves_capture(self):
        publish_gravity(self.history,9.96,[0,0,-1])
        publish_gravity(self.history,10.01,[.1,0,-np.sqrt(.99)])
        result=front_metadata(self.meta,self.history,.55)
        np.testing.assert_allclose(result['ground_plane_b'],[0,0,1,.55])
        self.assertEqual(result['capture_timestamp'],10.)
        self.assertEqual(result['imu_host_receive_timestamp'],9.96)
        self.assertFalse(result['rear_depth_used'])

    def test_missing_future_or_stale_imu_is_rejected(self):
        for timestamp in (None,9.89,10.01):
            history=mp.get_context('spawn').Array('d',4*HISTORY_SIZE,lock=True)
            if timestamp:publish_gravity(history,timestamp,[0,0,-1])
            with self.assertRaises(ValueError):front_metadata(self.meta,history,.55)

    def test_repeated_feedback_does_not_refresh_imu_timestamp(self):
        publish_gravity(self.history,9.9,[0,0,-1])
        before=list(self.history[:])
        publish_gravity(self.history,9.9,[0,0,-1])
        self.assertEqual(list(self.history[:]),before)
        with self.assertRaises(ValueError):
            front_metadata(dict(self.meta,capture_timestamp=10.1),self.history,.55)

    def test_front_projection_does_not_require_rear_plane(self):
        publish_gravity(self.history,9.99,[0,0,-1])
        class Perception:
            def process_front(self,meta,image):
                return dict(valid=True,ground_plane_b=meta['ground_plane_b'])
        result,_=process_front(Perception(),self.meta,None,dict(valid=False),self.history,.55)
        self.assertTrue(result['valid']);self.assertFalse(result['rear_depth_used'])

    def stream(self):
        x=np.linspace(1.5,3.,80)
        return dict(self.meta,valid=True,ground_plane_b=[0,0,1,.55],
                    points_b=np.c_[x,.02+.04*x+.03*x*x,x*0-.55].tolist())

    def test_rear_geometry_does_not_change_base_steering(self):
        results=[];x=np.linspace(-1.3,-.8,30)
        for y in (-.08,.08):
            front=self.stream()
            rear=dict(front,points_b=np.c_[x,x*0+y,x*0-.55].tolist())
            packet=dict(schema=SCHEMA,capture_generation=1,front=front,rear=rear,valid=True)
            controller=GuardedVD1VisualController([-1.02,0,-.3],forward_speed=.2,
                                                  ramp_s=0,front_path_estimation=True)
            result=controller.update(packet,10.1,.02)
            self.assertTrue(result['status']['valid'])
            self.assertFalse(result['status']['rear_used_for_steering'])
            results.append(result)
            stale=controller.update(packet,11.01,.02)
            self.assertFalse(stale['status']['valid'])
        np.testing.assert_array_equal(results[0]['base_command_b'],results[1]['base_command_b'])
        self.assertNotEqual(results[0]['tool_pose_b'],results[1]['tool_pose_b'])

    def test_repeated_capture_cannot_refresh_filter_or_time(self):
        estimator=FrontPathEstimator();stream=self.stream()
        result=estimator.update(stream)
        self.assertIs(estimator.update(stream),result)
        with self.assertRaisesRegex(ValueError,'refresh'):
            estimator.update(dict(stream,capture_timestamp=10.02))

    def test_far_s_bend_does_not_override_observed_near_tangent(self):
        x=np.linspace(1.2,3.,150)
        y=-.05-.14*(x-1.2)-.4*np.maximum(x-1.7,0)**2+.4*np.maximum(x-2.3,0)**2
        stream=dict(self.stream(),points_b=np.c_[x,y,0*x-.55].tolist())
        geometry,status=FrontPathEstimator().update(stream)
        expected=-.05-.14*(geometry[-1][0]-1.2)
        self.assertLess(abs(geometry[-1][1]-expected),.02)
        self.assertLessEqual(np.ptp(status['front_fitting_x_range_m']),.55000001)

    def test_ready_filter_carries_into_tracking_without_warming_speed_or_tool(self):
        front=self.stream();x=np.linspace(-1.3,-.8,30)
        rear=dict(front,points_b=np.c_[x,x*0+.05,x*0-.55].tolist())
        packet=dict(schema=SCHEMA,capture_generation=1,front=front,rear=rear,valid=True)
        estimator=FrontPathEstimator()
        candidate=GuardedVD1VisualController([-1.02,0,-.3],forward_speed=.2,
            front_path_estimation=True,front_path_filter=estimator)
        for _ in range(100):candidate.update(packet,10.1,.02)
        controller=GuardedVD1VisualController([-1.02,0,-.3],forward_speed=.2,
            front_path_estimation=True,front_path_filter=estimator)
        first=controller.update(packet,10.1,.02)
        self.assertTrue(first['status']['valid'])
        self.assertLess(first['base_command_b'][0],.001)
        self.assertLess(abs(first['tool_pose_b'][1]),.004)


if __name__=='__main__':unittest.main()
