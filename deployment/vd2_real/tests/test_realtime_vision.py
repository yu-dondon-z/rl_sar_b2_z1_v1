"""Evidence limits for the rear geometry and non-blocking diagnostic writer."""
import threading
import json
import queue
import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path
import sys
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'python'),str(ROOT/'python/vision'),str(ROOT/'thirdparty/rhacracknet')]
from cameras import ImageWriter,FailureFrames,FrontDecoder,Capture,front_loop,rear_covers_work_point,save_images
from dual_camera_perception import PerceptionConfig,rear_ground_roi
from realtime_perception import supported_depth_holes,rear_component,ObservableRearPerception,rear_work_section
from crack_trajectory import CenterlineConfig,extract_centerline


class RearEvidenceTests(unittest.TestCase):
    def test_discontinuous_cable_cannot_outrank_tracked_tape(self):
        mask=np.zeros((180,400),np.uint8)
        cv2.line(mask,(100,20),(100,140),255,5)
        cv2.line(mask,(350,20),(350,140),255,5)
        k=np.diag([1000.,1000.,1.]);tf=np.eye(4)
        tf[:3,:3]=[[0,1,0],[1,0,0],[0,0,-1]];tf[:3,3]=[-1.1,-.1,0]
        chosen=rear_component(mask,CenterlineConfig(waypoint_spacing_px=6.),(350,80),
                              extract_centerline,k,tf,np.array([0,0,1,1]),previous_y=0.)
        self.assertLess(np.max(np.abs(chosen.waypoints_uv[:,0]-100)),3.)
        mask[:,95:106]=0
        with self.assertRaisesRegex(ValueError,'no usable'):
            rear_component(mask,CenterlineConfig(waypoint_spacing_px=6.),(350,80),
                           extract_centerline,k,tf,np.array([0,0,1,1]),previous_y=0.)

    def test_continuous_real_seam_outside_workspace_is_preserved(self):
        mask=np.zeros((180,400),np.uint8)
        cv2.line(mask,(100,20),(100,140),255,5)
        k=np.diag([1000.,1000.,1.]);tf=np.eye(4)
        tf[:3,:3]=[[0,1,0],[1,0,0],[0,0,-1]];tf[:3,3]=[-1.1,-.274,0]
        chosen=rear_component(mask,CenterlineConfig(waypoint_spacing_px=6.),(100,80),
                              extract_centerline,k,tf,np.array([0,0,1,1]),previous_y=-.122)
        self.assertGreater(len(chosen.waypoints_uv),6)

    def setUp(self):
        self.k=np.array([[600.,0,80],[0,600.,60],[0,0,1]])
        # Camera looks vertically down; a real floor measurement has optical Z=1 m.
        self.tf=np.diag([1.,-1.,-1.,1.]);self.tf[0,3]=-1.02
        self.plane=np.array([0.,0.,1.,1.]);self.config=PerceptionConfig()

    def complete(self,depth):
        original=depth.copy();allowed,_=rear_ground_roi(depth,self.k,self.tf,self.plane,self.config)
        result,holes=supported_depth_holes(depth,allowed,self.k,self.tf,self.plane,self.config)
        np.testing.assert_array_equal(depth,original)
        return result,holes

    def test_small_depth_hole_has_explicit_plane_support_without_changing_depth(self):
        depth=np.ones((120,160));depth[56:65,76:85]=np.nan
        allowed,holes=self.complete(depth)
        self.assertTrue(holes[60,80]);self.assertTrue(allowed[60,80])
        self.assertTrue(np.isnan(depth[60,80]))

    def test_large_unknown_regions_borders_and_foreground_are_not_filled(self):
        depth=np.ones((120,160));depth[30:90,50:110]=np.nan
        allowed,holes=self.complete(depth);self.assertFalse(holes[60,80])
        depth[:,:40]=np.nan;_,holes=self.complete(depth);self.assertFalse(holes[60,1])
        depth=np.ones((120,160));depth[:,:4]=np.nan
        _,holes=self.complete(depth);self.assertFalse(holes[60,1])
        depth=np.ones((120,160));depth[50:70,70:90]=.8;depth[57:64,77:84]=np.nan
        allowed,holes=self.complete(depth)
        self.assertFalse(allowed[55,75]);self.assertFalse(holes[60,80])

    def test_same_pixel_hole_is_rejected_when_its_metric_radius_is_too_large(self):
        depth=np.full((120,160),2.);depth[56:65,76:85]=np.nan
        plane=np.array([0.,0.,1.,2.])
        allowed,_=rear_ground_roi(depth,self.k,self.tf,plane,self.config)
        _,holes=supported_depth_holes(depth,allowed,self.k,self.tf,plane,self.config)
        self.assertFalse(holes[60,80])

    def test_work_point_selection_uses_an_observed_component_not_image_anchor(self):
        mask=np.zeros((120,160),np.uint8)
        cv2.line(mask,(80,10),(80,110),255,5)
        cv2.line(mask,(135,10),(135,110),255,5)
        cfg=CenterlineConfig(waypoint_spacing_px=6.)
        selected=rear_component(mask,cfg,(135.,60.),extract_centerline,self.k,self.tf,self.plane)
        self.assertLess(np.max(np.abs(selected.waypoints_uv[:,0]-80)),2.)
        # No line is manufactured when the target is outside all components.
        mask[:,70:90]=0
        selected=rear_component(mask,cfg,(135.,60.),extract_centerline,self.k,self.tf,self.plane)
        self.assertGreater(np.min(selected.waypoints_uv[:,0]),130.)

    def test_low_direct_depth_support_cannot_be_relabelled_valid(self):
        cfg=PerceptionConfig(mode='classical')
        perception=ObservableRearPerception(cfg,None)
        uv=np.c_[np.full(12,80.),np.linspace(10,110,12)]
        depth=np.ones((240,320));depth[:,70:91]=np.nan
        meta=dict(sequence=1,capture_generation=1,capture_timestamp=10.,
            intrinsic_matrix=self.k.tolist(),T_base_camera=self.tf.tolist(),
            depth_aligned_to_rgb=True,depth_unit_m=1.,distortion_coefficients=[0.]*5)
        with patch.object(perception,'_centerline',return_value=(uv,.99,{})):
            result=perception.process_rear(meta,np.zeros((240,320,3),np.uint8),depth)
        self.assertFalse(result['valid']);self.assertIn('direct depth support too low',result['reason'])
        self.assertEqual(result['capture_timestamp'],10.)

    def test_target_coverage_is_not_assumed_from_detector_validity(self):
        self.assertFalse(rear_covers_work_point(dict(valid=True,points_b=[[-.9,0,0],[-.6,0,0]])))
        self.assertTrue(rear_covers_work_point(dict(valid=True,points_b=[[-1.2,0,0],[-.6,0,0]])))
        self.assertFalse(rear_covers_work_point(dict(valid=False,points_b=[[-1.2,0,0],[-.6,0,0]])))

    def test_measured_eight_cm_section_locates_work_point_without_a_twelve_cm_path(self):
        perception=ObservableRearPerception(PerceptionConfig(mode='classical'),None)
        uv=np.c_[np.linspace(56,104,12),np.full(12,60.)]
        meta=dict(sequence=1,capture_generation=1,capture_timestamp=10.,
            intrinsic_matrix=self.k.tolist(),T_base_camera=self.tf.tolist(),
            depth_aligned_to_rgb=True,depth_unit_m=1.,distortion_coefficients=[0.]*5)
        with patch.object(perception,'_centerline',return_value=(uv,.99,{})):
            result=perception.process_rear(meta,np.zeros((240,320,3),np.uint8),np.ones((240,320)))
        self.assertTrue(result['valid'],result)
        self.assertAlmostEqual(result['path_length_m'],.08)
        self.assertEqual(result['direct_depth_point_count'],12)
        self.assertLess(result['rear_observed_work_bracket_m'],.025)
        self.assertEqual(result['capture_timestamp'],10.)
        # The former common front/rear length gate rejects these same points.
        with self.assertRaisesRegex(ValueError,'geometric seam path is too short'):
            perception._path_result({},np.array(result['points_b']),uv,.99)

    def test_rgb_coverage_without_direct_depth_on_both_sides_is_rejected(self):
        points=np.c_[np.linspace(-1.06,-.98,12),np.zeros(12),np.zeros(12)]
        direct=points[:,0]<-1.02
        with self.assertRaisesRegex(ValueError,'direct depth support on both sides'):
            rear_work_section({},points,np.zeros((12,2)),direct,.99,self.config)

    def test_unobserved_work_point_or_wide_depth_bracket_cannot_be_extrapolated(self):
        for xs,reason in ((np.linspace(-.96,-.88,12),'both sides'),
                          (np.r_[np.linspace(-1.10,-1.05,6),np.linspace(-.99,-.94,6)],'2.5 cm')):
            points=np.c_[xs,np.zeros(12),np.zeros(12)]
            with self.assertRaisesRegex(ValueError,reason):
                rear_work_section({},points,np.zeros((12,2)),np.ones(12,dtype=bool),.99,self.config)

    def test_curve_bracket_is_checked_in_three_dimensions(self):
        points=np.c_[np.linspace(-1.06,-.98,12),np.zeros(12),np.zeros(12)]
        points[6:,1]=.04
        with self.assertRaisesRegex(ValueError,'2.5 cm'):
            rear_work_section({},points,np.zeros((12,2)),np.ones(12,dtype=bool),.99,self.config)


class ImageWriterTests(unittest.TestCase):
    def test_failed_input_retains_depth_below_png_resolution_and_its_nan_mask(self):
        depth=np.full((12,16),.654321,np.float32);depth[5,7]=np.nan
        rear=dict(image=np.zeros((12,16,3),np.uint8),raw_image=np.zeros((12,16,3),np.uint8),
                  depth=depth,k=np.eye(3),distortion=[0.]*5,distortion_model='test',serial='test')
        front=dict(image=rear['image'],jpeg=b'original camera bytes')
        rear['raw_mask']=np.full(depth.shape,255,np.uint8)
        rear['filtered_mask']=np.zeros(depth.shape,np.uint8)
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            save_images(root,front,rear,prefix='visual_stop_000_',pair=dict(failure_diagnostic={}))
            np.testing.assert_array_equal(np.load(root/'visual_stop_000_rear_depth_m.npy'),depth)
            self.assertEqual((root/'visual_stop_000_front_source.jpg').read_bytes(),front['jpeg'])
            for key in ('raw_mask','filtered_mask'):
                np.testing.assert_array_equal(cv2.imread(str(root/('visual_stop_000_rear_'+key+'.png')),0),rear[key])

    def test_ready_failure_preserves_its_own_frame_under_distinct_prefix(self):
        from unittest.mock import Mock
        frames=FailureFrames();requests=queue.Queue();writer=Mock();writer.submit.return_value=True
        rear=dict(raw_mask=np.ones((2,2),np.uint8))
        frames.remember({},rear,{},dict(capture_generation=1,front=dict(sequence=10)))
        requests.put(dict(state='ready',capture_generation=1,sequence=10,reason='missing depth'))
        with tempfile.TemporaryDirectory() as directory:
            self.assertTrue(frames.poll(requests,writer,Path(directory)))
        self.assertEqual(writer.submit.call_args.kwargs['prefix'],'ready_failure_000_')
        self.assertIs(writer.submit.call_args.args[2],rear)

    def test_slow_disk_does_not_block_or_expand_the_perception_queue(self):
        entered=threading.Event();release=threading.Event();calls=[]
        def save(*args,**kwargs):
            calls.append(args[0]);entered.set();release.wait(2.)
        with patch('cameras.save_images',save):
            writer=ImageWriter()
            try:
                self.assertTrue(writer.submit('first'));self.assertTrue(entered.wait(1.))
                self.assertTrue(writer.submit('second'));self.assertFalse(writer.submit('third'))
                self.assertEqual(writer.pending.qsize(),1)
            finally:release.set();writer.close()
        self.assertEqual(calls,['first','second'])

    def test_rejection_keeps_exact_pair_and_packet_even_when_writer_is_busy(self):
        from unittest.mock import Mock
        frames=FailureFrames();requests=queue.Queue();writer=Mock()
        writer.submit.side_effect=[False,True]
        images=[]
        for seq in (55,56,57,58):
            front,rear=object(),object();images.append((front,rear))
            frames.remember(front,rear,dict(front=dict(sequence=seq)),
                dict(capture_generation=11,front=dict(sequence=seq),valid=seq!=57))
        requests.put(dict(capture_generation=11,sequence=57,reason='short seam'))
        with tempfile.TemporaryDirectory() as directory:
            self.assertTrue(frames.poll(requests,writer,Path(directory)))
            frames.remember(object(),object(),{},dict(capture_generation=11,front=dict(sequence=59)))
            self.assertTrue(frames.poll(requests,writer,Path(directory)))
            self.assertIsNone(frames.pending)
        for call in writer.submit.call_args_list:
            self.assertEqual(call.args[1:3],images[2])
            self.assertEqual(call.kwargs['prefix'],'visual_stop_000_')
            evidence=call.kwargs['pair']['failure_diagnostic']
            self.assertEqual(evidence['packet']['front']['sequence'],57)
            self.assertFalse(evidence['packet']['valid'])

    def test_diagnostic_cache_never_substitutes_a_different_frame(self):
        from unittest.mock import Mock
        frames=FailureFrames();requests=queue.Queue();writer=Mock()
        for seq in range(10):
            frames.remember(None,None,{},dict(capture_generation=11,front=dict(sequence=seq)))
        self.assertEqual(len(frames.recent),4)
        requests.put(dict(capture_generation=11,sequence=2))
        with tempfile.TemporaryDirectory() as directory:
            self.assertFalse(frames.poll(requests,writer,Path(directory)))
            evidence=json.loads((Path(directory)/'visual_stop_000_missing.json').read_text())
            self.assertEqual(evidence['request']['sequence'],2)
        writer.submit.assert_not_called()


class FrontDecodeTests(unittest.TestCase):
    def config(self):
        return dict(intrinsics=dict(fx=900.,fy=900.,cx=960.,cy=540.,distortion=[.01,-.01,0.,0.,0.]))

    def sample(self,image):
        config=self.config()['intrinsics']
        return dict(jpeg=cv2.imencode('.jpg',image)[1].tobytes(),number=17,received=120.,requested=119.9,
            k=np.array([[config['fx'],0,config['cx']],[0,config['fy'],config['cy']],[0,0,1.]]))

    def test_selected_front_decode_preserves_pixels_capture_time_and_source_identity(self):
        image=np.full((1080,1920,3),180,np.uint8)
        cv2.line(image,(150,50),(1700,1000),(20,30,40),9)
        sample=self.sample(image);decoder=FrontDecoder(self.config());result=decoder.decode(sample)
        original=cv2.imdecode(np.frombuffer(sample['jpeg'],np.uint8),cv2.IMREAD_COLOR)
        maps=cv2.initUndistortRectifyMap(sample['k'],np.asarray(self.config()['intrinsics']['distortion']),
            None,sample['k'],(1920,1080),cv2.CV_32FC1)
        np.testing.assert_array_equal(result['image'],cv2.remap(original,*maps,interpolation=cv2.INTER_LINEAR))
        self.assertEqual(result['received'],120.);self.assertEqual(result['number'],17)
        self.assertNotIn('image',sample)
        with patch('cameras.cv2.imdecode',side_effect=AssertionError('same capture decoded twice')):
            self.assertIs(decoder.decode(sample),result)

    def test_corrupt_or_changed_resolution_is_not_accepted_as_a_calibrated_capture(self):
        decoder=FrontDecoder(self.config());sample=self.sample(np.zeros((480,640,3),np.uint8))
        with self.assertRaisesRegex(ValueError,'recalibrate'):decoder.decode(sample)
        sample['jpeg']=b'corrupt'
        with self.assertRaisesRegex(ValueError,'decode failed'):decoder.decode(sample)

    def test_front_capture_copies_sdk_buffer_and_does_not_decode_discarded_frames(self):
        stop=threading.Event();capture=Capture()
        class Library:
            def vd2_front_open(self,interface):return 0
            def vd2_front_image(self,raw,size,requested,received):
                raw[:4]=b'abcd';requested._obj.value=19.;received._obj.value=20.
                stop.set();self.raw=raw;return 4
        lib=Library()
        with patch('cameras.library',return_value=lib),patch('cameras.cv2.imdecode',side_effect=AssertionError('capture decoded')):
            front_loop(capture,stop,'eth0',self.config())
        sample,error=capture.get();self.assertEqual(error,'');self.assertEqual(sample['jpeg'],b'abcd')
        lib.raw[:4]=b'zzzz';self.assertEqual(sample['jpeg'],b'abcd')
        self.assertEqual(sample['received'],20.)


class CpuLayoutTests(unittest.TestCase):
    def test_real_weights_preserve_probability_and_mask_for_fixed_input(self):
        import torch
        from crack_inference import CrackSegmenter
        from realtime_segmenter import RealtimeSegmenter
        torch.set_num_threads(2)
        image=np.full((480,640,3),210,np.uint8)
        cv2.line(image,(400,460),(360,20),(25,25,25),16)
        arguments=dict(device='cpu',threshold=.90,input_size=(360,640))
        reference=CrackSegmenter(ROOT/'assets/rhanet.pth',**arguments).predict_bgr(image)
        model=RealtimeSegmenter(ROOT/'assets/rhanet.pth',**arguments)
        result=model.predict_bgr(image)
        self.assertTrue(model.channels_last)
        np.testing.assert_allclose(reference.probability,result.probability,atol=2e-5,rtol=0.)
        np.testing.assert_array_equal(reference.mask,result.mask)
        np.testing.assert_array_equal(reference.overlay_bgr,result.overlay_bgr)


if __name__=='__main__':unittest.main()
