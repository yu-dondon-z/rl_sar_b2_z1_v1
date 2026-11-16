"""CPU regression of the existing encoder-only mask with the actual VD12 gun.

No renderer, physics stepping, world pose input, or hardware IO is used.
"""
from pathlib import Path
import copy
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from vd5_gun_mask import EncoderGunMask
from vd12_gun_mask import NearClippedEncoderGunMask, clip_convex_vertices
from vd5_visual_control import encoder_tcp_pose, quaternion_from_matrix
from vd1_mujoco_scene import VD10_URDF_PATH


class NewGunMask(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gun = EncoderGunMask(VD10_URDF_PATH)
        cls.intrinsic = np.array([[300.,0.,320.],[0.,300.,240.],[0.,0.,1.]])
        cls.camera_to_base = np.eye(4)
        cls.camera_to_base[2,3] = -3.

    def packet(self, q=None):
        return dict(arm_joint_names=['joint%d' % i for i in range(1,7)],
            arm_joint_positions_rad=[0.]*6 if q is None else list(q),
            arm_encoder_timestamp=10., capture_timestamp=10.)

    def project(self, packet):
        return self.gun.project(packet, self.intrinsic, self.camera_to_base, (480,640))

    def test_actual_new_hollow_gun_dimensions_and_mount(self):
        vertices = self.gun.vertices
        np.testing.assert_allclose(vertices.min(0),[-.125,-.028,.072],atol=2e-8)
        np.testing.assert_allclose(vertices.max(0),[.975,.028,.128],atol=2e-8)
        radial = np.linalg.norm(vertices[:,1:]-[0.,.1],axis=1)
        self.assertAlmostEqual(float(radial.min()),.026,places=7)
        self.assertAlmostEqual(float(radial.max()),.028,places=7)
        self.assertGreaterEqual(len(vertices),96)

    def test_encoder_fk_matches_independent_urdf_kinematics(self):
        for q in ([0.]*6,[0.,.66647356,-1.25178768,.88531411,0.,0.],
                  [.2,.7,-1.2,.5,-.1,.3]):
            with self.subTest(q=q):
                position,rotation = self.gun.link_pose(q)
                expected_position,expected_quaternion = encoder_tcp_pose(q,VD10_URDF_PATH,np.zeros(3),np.eye(3))
                np.testing.assert_allclose(position,expected_position,atol=2e-7)
                quaternion = quaternion_from_matrix(rotation)
                self.assertGreater(abs(float(quaternion @ expected_quaternion)),1.-2e-7)

    def test_project_new_mesh_with_capture_encoders(self):
        mask,diagnostics = self.project(self.packet())
        self.assertEqual(mask.shape,(480,640))
        self.assertEqual(mask.dtype,np.bool_)
        self.assertGreater(int(mask.sum()),0)
        self.assertEqual(diagnostics['gun_self_mask_source'],'capture_arm_encoders_and_fixed_urdf_gun_mesh')
        self.assertGreaterEqual(len(diagnostics['gun_exclusion_contour_uv']),3)

    def test_end_caps_and_outer_rim_are_excluded(self):
        metadata=self.packet()
        mask,_=self.project(metadata)
        position,rotation=self.gun.link_pose(metadata['arm_joint_positions_rad'])
        theta=np.linspace(0,2*np.pi,361)
        for x in (-.125,.425,.975):
            tube=np.column_stack((np.full(len(theta),x),.028*np.cos(theta),.1+.028*np.sin(theta)))
            base=tube@rotation.T+position
            camera=(base-self.camera_to_base[:3,3])@self.camera_to_base[:3,:3]
            projected=camera@self.intrinsic.T
            uv=np.rint(projected[:,:2]/projected[:,2,None]).astype(int)
            self.assertTrue(np.all(mask[uv[:,1],uv[:,0]]))

    def test_missing_or_mismatched_capture_encoder_rejected(self):
        for key,value in [('arm_joint_names',None),('arm_joint_positions_rad',None),
                          ('arm_encoder_timestamp',None),('capture_timestamp',None),
                          ('arm_encoder_timestamp',9.95),('arm_joint_positions_rad',[0.]*5),
                          ('arm_joint_positions_rad',[float('nan')]*6)]:
            packet=self.packet()
            if value is None: packet.pop(key)
            else: packet[key]=value
            with self.subTest(key=key,value=value),self.assertRaises((ValueError,TypeError)):
                self.project(packet)

    def test_world_fields_never_influence_projection(self):
        packet=self.packet()
        baseline,_=self.project(packet)
        changed=copy.deepcopy(packet)
        changed.update(base_position_world=[100.,-100.,20.],tcp_position_world=[-1000.]*3,
                       simulator_seam_points_world=[[200.,300.,0.]])
        actual,_=self.project(changed)
        np.testing.assert_array_equal(actual,baseline)

    def test_near_plane_intersections_preserve_crossing_tube_sides(self):
        # A square tube with one end behind the camera's near plane. The
        # legacy hull contains only the far cap, missing its visible sides.
        ring = np.array([[-.02, -.02], [-.02, .02], [.02, -.02], [.02, .02]])
        vertices = np.vstack((np.c_[ring, np.full(4, -.02)],
                              np.c_[ring, np.full(4, .5)]))
        clipped = clip_convex_vertices(vertices)
        self.assertTrue(np.all(clipped[:, 2] >= .08))
        intersections = clipped[np.isclose(clipped[:, 2], .08)]
        self.assertGreaterEqual(len(intersections), 4)
        np.testing.assert_allclose(intersections[:, :2].min(0), [-.02, -.02])
        np.testing.assert_allclose(intersections[:, :2].max(0), [.02, .02])
        self.assertGreater(float(np.max(np.abs(clipped[:, :2] / clipped[:, 2, None]))),
                           float(np.max(np.abs(vertices[4:, :2] / vertices[4:, 2, None]))))
        np.testing.assert_array_equal(clip_convex_vertices(vertices[4:]), vertices[4:])
        self.assertEqual(clip_convex_vertices(vertices[:4]).shape, (0, 3))

    def test_clipped_projection_contains_sampled_tube_surface_at_real_mount(self):
        from vd5_camera_geometry import measured_camera_calibration
        from dual_camera_perception import camera_geometry
        camera = measured_camera_calibration()['rear_d435i']
        calibration = dict(intrinsic_matrix=camera['intrinsic_matrix'],
            camera_to_base=dict(position=camera['position_b'],
                                quaternion_wxyz=camera['quaternion_ros_wxyz']))
        intrinsic, transform = camera_geometry(calibration)
        gun = NearClippedEncoderGunMask(VD10_URDF_PATH)
        # Actual capture-110 arm encoders: the 1.1m tube crosses the camera
        # near plane here, unlike the nominal startup pose outside the image.
        metadata = self.packet([-.16066510, 1.73661162, -1.07638274,
                                .53942789, .28169197, -.02700695])
        mask, diagnostics = gun.project(metadata, intrinsic, transform, (480, 640))
        self.assertTrue(diagnostics['gun_self_mask_near_plane_clipped'])
        theta = np.linspace(0., 2*np.pi, 97)
        x, angle = np.meshgrid(np.linspace(-.125, .975, 301), theta)
        surface = np.c_[x.ravel(), .028*np.cos(angle.ravel()), .1+.028*np.sin(angle.ravel())]
        position, rotation = gun.link_pose(metadata['arm_joint_positions_rad'])
        camera_points = (surface @ rotation.T+position-transform[:3, 3]) @ transform[:3, :3]
        camera_points = camera_points[camera_points[:, 2] >= .08]
        rays = camera_points @ intrinsic.T
        uv = np.rint(rays[:, :2]/rays[:, 2, None]).astype(int)
        inside = (uv[:, 0] >= 0) & (uv[:, 0] < 640) & (uv[:, 1] >= 0) & (uv[:, 1] < 480)
        self.assertGreater(np.count_nonzero(inside), 100)
        self.assertTrue(np.all(mask[uv[inside, 1], uv[inside, 0]]))


if __name__=='__main__':
    unittest.main()
