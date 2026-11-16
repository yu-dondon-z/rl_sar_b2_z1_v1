"""Pixel evidence and sensor-local target validity; no robot I/O."""
from copy import deepcopy
import ast
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'scripts'), '/home/dong/b2_z1/scripts/vision',
               '/home/dong/RHACrackNet/code']
from vd14_visual_rear import dark_ridge_support, VD14RearEvidencePerception
from dual_camera_perception import PerceptionConfig
from crack_trajectory import CenterlineConfig
from vd14_motion_compensation import MotionCompensatedVisualController
from vd5_mujoco_control import quaternion_from_euler_xyz
import vd5_mujoco_control as conversion
from vd12_visual_control import cylinder_endface_height
import test_vd14_motion_compensation as sensor_tests
from test_vd12_vision_entry import packet


class PixelEvidence(unittest.TestCase):
    def test_uniform_and_single_bright_edge_are_not_dark_ridges(self):
        image=np.full((120,160,3),107,np.uint8)
        allowed=np.ones(image.shape[:2],bool)
        self.assertFalse(dark_ridge_support(image,allowed).any())
        image[:,80:]=210
        self.assertFalse(dark_ridge_support(image,allowed).any())

    def test_dark_strips_at_multiple_orientations_survive(self):
        for end in ((80,110),(145,110),(145,60)):
            image=np.full((120,160,3),107,np.uint8)
            start=(80,10) if end==(80,110) else (15,10) if end==(145,110) else (15,60)
            cv2.line(image,start,end,(2,2,2),7)
            supported=dark_ridge_support(image,np.ones(image.shape[:2],bool))
            middle=tuple(np.rint((np.array(start)+end)/2).astype(int))
            self.assertTrue(supported[middle[1],middle[0]])
            self.assertFalse(np.any(supported & (image[:,:,0]==107)))

    def test_exclusion_and_gaps_cannot_be_recreated_as_measurements(self):
        image=np.full((120,160,3),107,np.uint8)
        image[:,75:85]=2
        allowed=np.ones(image.shape[:2],bool);allowed[50:70]=False
        supported=dark_ridge_support(image,allowed)
        self.assertFalse(supported[50:70].any())
        self.assertTrue(supported[30,80])
        # Missing both flanks is not evidence for a ridge behind the gun.
        allowed[:]=False;allowed[:,75:85]=True
        self.assertFalse(dark_ridge_support(image,allowed).any())

    def test_neural_gray_hallucination_is_removed_without_creating_dark_pixels(self):
        image=np.full((320,320,3),107,np.uint8);image[:,155:165]=2
        mask=np.zeros((320,320),np.uint8)
        cv2.rectangle(mask,(150,30),(170,290),255,-1)
        cv2.line(mask,(230,30),(175,260),255,9)
        class Mask:
            def predict_bgr(self,_):
                return SimpleNamespace(mask=mask.copy(),probability=(mask>0)*.99,inference_ms=0.)
        perception=VD14RearEvidencePerception(PerceptionConfig(mode='rhanet'),segmenter=Mask(),
                                             rear_centerline_mode='cross_sections')
        allowed=np.ones((320,320),bool)
        perception._rear_allowed_pixels=allowed
        perception._rear_geometry_context=dict(allowed_pixels=allowed,
            intrinsic=np.array([[200.,0.,160.],[0.,200.,160.],[0.,0.,1.]]),
            transform=np.array([[0.,-1.,0.,-.8],[-1.,0.,0.,0.],[0.,0.,-1.,.5],[0.,0.,0.,1.]]),
            plane=np.array([0.,0.,1.,.5]),depth_m=np.ones((320,320)))
        uv,confidence,_=perception._centerline(image,.65,allowed)
        self.assertTrue(np.all((uv[:,0]>=155)&(uv[:,0]<=164)))
        self.assertAlmostEqual(confidence,.99)
        pixels=np.rint(uv).astype(int)
        self.assertTrue(np.all(mask[pixels[:,1],pixels[:,0]]>0))
        self.assertTrue(np.all(image[pixels[:,1],pixels[:,0],0]==2))
        self.assertIs(perception._rear_allowed_pixels,allowed)

    def test_retained_real_false_gun_path_lacks_rgb_evidence(self):
        directory=ROOT/'outputs/vd14_vision/vision_20261116_135848_wp39tnac'
        metadata_path=directory/'rear/frame_slot_6.json'
        if not metadata_path.is_file():self.skipTest('Retained simulation camera frame unavailable')
        metadata=json.loads(metadata_path.read_text())
        self.assertEqual(metadata['sequence'],334)
        raw=cv2.imread(str(directory/'rear'/metadata['rgb_file']))
        packets=[json.loads(x) for x in (directory/'dual_vision_packets.jsonl').read_text().splitlines()]
        rear=next(p['rear'] for p in packets if p['rear']['sequence']==334)
        uv=np.rint(rear['waypoints_uv']).astype(int)
        support=dark_ridge_support(raw,np.ones(raw.shape[:2],bool))
        self.assertLess(float(np.mean(support[uv[:,1],uv[:,0]])),.2)
        self.assertTrue(support[350,348:352].any())


class TargetContinuity(unittest.TestCase):
    def test_deferred_candidate_is_not_reported_as_a_supported_work_point(self):
        perception=VD14RearEvidencePerception(PerceptionConfig(mode='classical'),
            trailing_distance=.95,rear_centerline_mode='cross_sections')
        allowed=np.ones((320,320),bool)
        perception._rear_allowed_pixels=allowed
        perception._rear_geometry_context=dict(allowed_pixels=allowed,depth_m=np.ones((320,320)),
            intrinsic=np.array([[200.,0.,160.],[0.,200.,160.],[0.,0.,1.]]),
            transform=np.array([[0.,-1.,0.,-.8],[-1.,0.,0.,0.],[0.,0.,-1.,.5],[0.,0.,0.,1.]]),
            plane=np.array([0.,0.,1.,.5]))
        mask=np.zeros((320,320),np.uint8)
        cv2.rectangle(mask,(155,100),(165,160),255,-1)
        perception._extract_with_roi(mask,CenterlineConfig(waypoint_spacing_px=6.),(160.,130.))
        diagnostic=perception._rear_centerline_diagnostics
        self.assertFalse(diagnostic['rear_component_target_supported'])
        self.assertTrue(diagnostic['rear_component_target_validation_deferred'])
        cv2.rectangle(mask,(175,180),(185,290),255,-1)
        perception._extract_with_roi(mask,CenterlineConfig(waypoint_spacing_px=6.),(160.,130.))
        diagnostic=perception._rear_centerline_diagnostics
        # The alternate is target-supported at CAPTURE time only. Do not
        # switch away from a valid original path before sensor transport.
        self.assertFalse(diagnostic['rear_component_target_supported'])
        self.assertTrue(diagnostic['rear_component_target_validation_deferred'])
        self.assertFalse(diagnostic['rear_component_fallback'])

    def test_deferred_capture_target_is_still_rejected_if_current_support_is_missing(self):
        perception=VD14RearEvidencePerception(PerceptionConfig(mode='classical'),trailing_distance=.95)
        trajectory=SimpleNamespace(waypoints_uv=np.c_[np.full(30,160.),np.linspace(100.,160.,30)],
                                   path_length_px=60.)
        context=dict(allowed_pixels=np.ones((320,320),bool),depth_m=np.ones((320,320)),
            intrinsic=np.array([[200.,0.,160.],[0.,200.,160.],[0.,0.,1.]]),
            transform=np.array([[0.,-1.,0.,-.8],[-1.,0.,0.,0.],[0.,0.,-1.,.5],[0.,0.,0.,1.]]),
            plane=np.array([0.,0.,1.,.5]))
        diagnostic={}
        perception._candidate_geometry(trajectory,context,diagnostic)
        self.assertTrue(diagnostic['observed_rgbd_path_validated'])
        self.assertTrue(diagnostic['target_validation_deferred_to_control'])
        self.assertFalse(diagnostic['target_supported'])
        h=sensor_tests.SensorTransport().history(steps=0)
        c=MotionCompensatedVisualController([-.95,0.,.35],quaternion_from_euler_xyz(0.,.3,np.pi),
                                           motion_history=h,trailing_distance=.95)
        image=packet(stamp=10.)
        for name in ('front','rear'):image[name]['capture_sim_time_s']=0.
        image['rear']['points_b']=np.c_[np.linspace(-.8,-.5,30),np.zeros(30),np.full(30,-.5)].tolist()
        result=c.update(image,10.,.02,control_time_s=0.)
        self.assertFalse(result['status']['valid'])
        self.assertIn('support is too short',result['status']['reason'])

    def test_pi_yaw_initial_sign_survives_sensor_motion_and_never_flips_actor_quaternion(self):
        h=sensor_tests.SensorTransport().history(steps=0)
        # Actual FK reset quaternion has tiny NEGATIVE w and positive z. q.w
        # canonicalization would invert it and then flip twice per yaw cycle.
        initial=np.array([-4.32205559e-8,-.149438161,-6.53215020e-9,.988771074])
        initial/=np.linalg.norm(initial)
        c=MotionCompensatedVisualController([-.8067,0.,.3943],initial,
                                           motion_history=h,trailing_distance=.95)
        poses=[]
        for i in range(151):
            clock=i*.02
            if i:
                rotation=sensor_tests.exp_rotation(np.array([0.,0.,.02*np.sin(clock*6.)]))
                h.samples.append(dict(time_s=clock,rotation=rotation,position=np.zeros(3)))
            image=packet(sequence=i,stamp=10.+clock)
            for name in ('front','rear'):image[name]['capture_sim_time_s']=clock
            result=c.update(image,10.+clock,.02,control_time_s=clock)
            self.assertTrue(result['status']['valid'],result['status']['reason'])
            poses.append(result['tool_pose_b'][3:])
        np.testing.assert_allclose(poses[0],initial,atol=1.e-12)
        poses=np.array(poses)
        self.assertGreater(np.min(np.sum(poses[:-1]*poses[1:],axis=1)),.99)
        self.assertTrue(np.all(poses[100:,3]>.8))

    def test_new_high_confidence_frame_cannot_jump_20cm_or_reset_failure(self):
        h=sensor_tests.SensorTransport().history(steps=0)
        c=MotionCompensatedVisualController([-.95,0.,.35],quaternion_from_euler_xyz(0.,.3,np.pi),
                                           motion_history=h,trailing_distance=.95)
        first=packet(stamp=10.)
        for name in ('front','rear'):first[name]['capture_sim_time_s']=0.
        self.assertTrue(c.update(first,10.,.02,control_time_s=0.)['status']['valid'])
        for i in range(1,5):h.sample(i*.005,sensor_tests.proprio(),[100.]*4)
        bad=deepcopy(first)
        for name in ('front','rear'):
            bad[name].update(sequence=1,capture_timestamp=10.02,capture_sim_time_s=.02)
        for p in bad['rear']['points_b']:p[1]=.2
        rejected=c.update(bad,10.02,.02,control_time_s=.02)
        self.assertFalse(rejected['status']['valid'])
        self.assertIn('work point jumped',rejected['status']['reason'])
        self.assertTrue(rejected['status']['failure_latched'])
        np.testing.assert_array_equal(rejected['base_command_b'],[0.,0.,0.])
        self.assertFalse(c.update(first,10.03,.02,control_time_s=.02)['status']['valid'])


class TrainingConversionParity(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import torch
        cls.torch=torch
        training=Path('/home/dong/b2_z1_VD2/source/b2_z1/b2_z1/tasks/manager_based/b2_z1/mdp')
        def load(name,path):
            spec=importlib.util.spec_from_file_location(name,str(path))
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            return module
        cls.geometry=load('vd14_reference_geometry',training/'commands/vd5_geometry.py')
        cls.cylinder=load('vd14_reference_cylinder',training/'vd10_geometry.py')
        # Execute the exact two pure training functions without importing an
        # Isaac runtime into the visualization interpreter.
        source=ast.parse((training/'commands/vd5_startup_command.py').read_text())
        selected=[n for n in source.body if isinstance(n,ast.FunctionDef)
                  and n.name in ('startup_fraction','interpolate_quaternion')]
        cls.reference={'torch':torch,'math':math}
        exec(compile(ast.Module(body=selected,type_ignores=[]),'training_startup_pure','exec'),cls.reference)

    def test_training_and_deployment_rigid_conversion_and_arc_sampling_agree(self):
        t=self.torch
        points=np.c_[np.linspace(-2.,2.,150),.03*np.sin(np.linspace(-2.,2.,150)),np.zeros(150)]
        position=np.array([.15,.02,.5]);q=quaternion_from_euler_xyz(.05,-.04,.3)
        expected=self.geometry.sample_polyline_behind_position(t.tensor(points[None]),t.tensor(position[None]),.95)[0].numpy()
        np.testing.assert_allclose(conversion.sample_polyline_behind_position(points,position,.95),expected,atol=1.e-12)
        body=self.geometry.world_position_to_base(t.tensor(expected[None]),t.tensor(position[None]),t.tensor(q[None]))[0].numpy()
        np.testing.assert_allclose(conversion.world_position_to_base(expected,position,q),body,atol=1.e-12)

    def test_two_second_ramp_and_shortest_wxyz_interpolation_match_training(self):
        t=self.torch;steps=t.arange(0,121,dtype=t.float64)
        reference=self.reference['startup_fraction'](steps,.02,2.).numpy()
        np.testing.assert_allclose([conversion.startup_fraction(i*.02) for i in range(121)],reference,atol=1.e-12)
        first=quaternion_from_euler_xyz(.1,.2,.3);second=-quaternion_from_euler_xyz(0.,1.12,np.pi)
        for fraction in (0.,.2,.5,1.):
            ref=self.reference['interpolate_quaternion'](t.tensor(first[None]),t.tensor(second[None]),t.tensor([fraction],dtype=t.float64))[0].numpy()
            np.testing.assert_allclose(conversion.normalized_lerp_quaternion(first,second,fraction),ref,atol=1.e-12)

    def test_finite_tube_rim_clearance_matches_training(self):
        axes=np.array([[0.,0.,1.],[0.,0.,-1.],[.8,0.,-.6],[.6,0.,.8],[1.,0.,0.]])
        expected=self.cylinder.cylinder_tip_height_above_plane(self.torch.tensor(axes),.005).numpy()
        np.testing.assert_allclose([cylinder_endface_height(a,[0.,0.,1.,0.]) for a in axes],expected,atol=1.e-12)


if __name__=='__main__':unittest.main()
