import importlib.util
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import cv2
import numpy as np
import pyrealsense2 as rs

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
from cameras import Capture,rear_loop
from realsense_geometry import rectification_maps,intrinsics_from_record
spec=importlib.util.spec_from_file_location('calibrate_rear_geometry',ROOT/'tools/calibrate_rear.py')
calibrate=importlib.util.module_from_spec(spec);spec.loader.exec_module(calibrate)


class RealSenseGeometryTests(unittest.TestCase):
    def intrinsics(self,model=None,zero=False):
        sensor=rs.intrinsics()
        sensor.width=64;sensor.height=48;sensor.fx=55;sensor.fy=56
        sensor.ppx=31.4;sensor.ppy=23.2
        sensor.model=rs.distortion.inverse_brown_conrady if model is None else model
        sensor.coeffs=[0.]*5 if zero else [.12,-.03,.009,-.007,.002]
        return sensor

    def record(self,sensor):
        return dict(image_size=[sensor.width,sensor.height],
            intrinsic_matrix=[[sensor.fx,0,sensor.ppx],[0,sensor.fy,sensor.ppy],[0,0,1]],
            distortion_coefficients=list(sensor.coeffs),distortion_model=str(sensor.model))

    def test_zero_inverse_model_is_an_identity_map(self):
        sensor=self.intrinsics(zero=True)
        x,y=rectification_maps(sensor)
        v,u=np.indices((48,64),dtype=np.float32)
        np.testing.assert_array_equal(x,u);np.testing.assert_array_equal(y,v)

    def test_rectified_rays_round_trip_through_sdk_deprojection(self):
        # This checks map direction as well as model semantics: deprojecting
        # each sampled *source* pixel must recover the destination pinhole ray.
        for model in (rs.distortion.inverse_brown_conrady,rs.distortion.brown_conrady):
            with self.subTest(model=str(model)):
                sensor=self.intrinsics(model);x,y=rectification_maps(sensor)
                for v,u in ((0,0),(47,63),(24,32),(12,48)):
                    point=rs.rs2_deproject_pixel_to_point(sensor,[float(x[v,u]),float(y[v,u])],1.)
                    np.testing.assert_allclose(point,[(u-sensor.ppx)/sensor.fx,
                        (v-sensor.ppy)/sensor.fy,1.],atol=2e-6)

    def test_inverse_and_modified_maps_are_not_ordinary_opencv_brown(self):
        for model in (rs.distortion.inverse_brown_conrady,rs.distortion.modified_brown_conrady):
            sensor=self.intrinsics(model);x,y=rectification_maps(sensor)
            k=np.asarray(self.record(sensor)['intrinsic_matrix'],dtype=float)
            ordinary=cv2.initUndistortRectifyMap(k,np.asarray(sensor.coeffs),None,k,(64,48),cv2.CV_32FC1)
            self.assertGreater(float(np.max(np.abs(x-ordinary[0]))),.01)
            self.assertTrue(np.isfinite(y).all())

    def test_raw_calibration_uses_the_recorded_model_and_zero_distortion_after_rectification(self):
        sensor=self.intrinsics();record=self.record(sensor)
        image=np.arange(48*64*3,dtype=np.uint8).reshape(48,64,3)
        reconstructed=intrinsics_from_record(record)
        self.assertEqual(reconstructed.model,rs.distortion.inverse_brown_conrady)
        rectified,k,distortion=calibrate.prepare_image(image,record)
        np.testing.assert_array_equal(rectified,cv2.remap(image,*rectification_maps(sensor),cv2.INTER_LINEAR))
        np.testing.assert_array_equal(k,record['intrinsic_matrix'])
        np.testing.assert_array_equal(distortion,np.zeros(5))
        with self.assertRaises(ValueError):calibrate.prepare_image(image[:-1],record)

    def test_unknown_and_invalid_intrinsics_are_not_silently_accepted(self):
        sensor=self.intrinsics(rs.distortion.ftheta)
        with self.assertRaises(ValueError):rectification_maps(sensor)
        record=self.record(sensor)
        with self.assertRaises(ValueError):intrinsics_from_record(record)
        sensor=self.intrinsics();sensor.fx=0
        with self.assertRaises(ValueError):rectification_maps(sensor)

    def test_rear_capture_accepts_inverse_and_keeps_rgb_depth_and_source_timestamps(self):
        sensor=self.intrinsics(zero=True);sensor.width=640;sensor.height=480
        image=np.zeros((480,640,3),dtype=np.uint8);image[200,300]=[10,20,30]
        depth=np.full((480,640),800,dtype=np.uint16);depth[100,100]=0
        color=SimpleNamespace(get_data=lambda:image,get_frame_number=lambda:37,
            get_timestamp=lambda:1234.,get_frame_timestamp_domain=lambda:'hardware_clock')
        depth_frame=SimpleNamespace(get_data=lambda:depth,get_timestamp=lambda:1235.,
            get_frame_timestamp_domain=lambda:'hardware_clock')
        frames=SimpleNamespace(get_color_frame=lambda:color,get_depth_frame=lambda:depth_frame)
        profile=SimpleNamespace(get_device=lambda:SimpleNamespace(first_depth_sensor=lambda:
            SimpleNamespace(get_depth_scale=lambda:.001)),get_stream=lambda stream:
            SimpleNamespace(as_video_stream_profile=lambda:SimpleNamespace(get_intrinsics=lambda:sensor)))
        stop=threading.Event();stopped=[]
        def next_frame(timeout):
            stop.set();return frames
        pipeline=SimpleNamespace(start=lambda settings:profile,wait_for_frames=next_frame,
                                 stop=lambda:stopped.append(True))
        settings=SimpleNamespace(enable_device=lambda serial:None,enable_stream=lambda *args:None)
        device=SimpleNamespace(get_info=lambda field:'Intel RealSense D435I' if field==rs.camera_info.name else 'measured-serial')
        sdk=SimpleNamespace(pipeline=lambda:pipeline,config=lambda:settings,
            context=lambda:SimpleNamespace(query_devices=lambda:[device]),
            align=lambda stream:SimpleNamespace(process=lambda value:value),
            stream=rs.stream,format=rs.format,camera_info=rs.camera_info,distortion=rs.distortion,
            rs2_project_point_to_pixel=rs.rs2_project_point_to_pixel)
        capture=Capture()
        with patch.dict(sys.modules,{'pyrealsense2':sdk}):rear_loop(capture,stop,dict(serial=''))
        sample,error=capture.get()
        self.assertEqual(error,'');self.assertIsNotNone(sample);self.assertEqual(stopped,[True])
        np.testing.assert_array_equal(sample['raw_image'],image)
        np.testing.assert_array_equal(sample['image'][200,300],[10,20,30])
        self.assertAlmostEqual(float(sample['depth'][200,300]),.8,places=6)
        self.assertTrue(np.isnan(sample['depth'][100,100]))
        self.assertEqual(sample['number'],37)
        self.assertEqual(sample['device_time_ms'],1234.)
        self.assertEqual(sample['depth_device_time_ms'],1235.)
        self.assertEqual(sample['distortion_model'],'distortion.inverse_brown_conrady')


if __name__=='__main__':unittest.main()
