"""Regression coverage for measured camera projection and false gun seams."""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')

from pathlib import Path
import tempfile
import unittest

import cv2
import mujoco
import numpy as np

from dual_camera_perception import PerceptionConfig
from vd1_mujoco_scene import (VD3_URDF_PATH, build_scene, quaternion_matrix)
from vd1_visual_rear import SeamOnlyDualPerception
from vd5_camera_geometry import measured_camera_calibration
from vd5_gun_mask import EncoderGunMask


class MeasuredVisionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cameras = measured_camera_calibration()
        cls.gun = EncoderGunMask(VD3_URDF_PATH)

    def fixture(self):
        camera = self.cameras['rear_d435i']
        intrinsic = np.array(camera['intrinsic_matrix'])
        transform = np.eye(4)
        transform[:3, :3] = quaternion_matrix(camera['quaternion_ros_wxyz'])
        transform[:3, 3] = camera['position_b']
        metadata = dict(intrinsic_matrix=intrinsic.tolist(), T_base_camera=transform.tolist(),
                        depth_unit_m=.001, depth_aligned_to_rgb=True,
                        arm_joint_positions_rad=[0., 2.55, -1.6, -1.4, 0., 0.],
                        arm_joint_names=self.gun.joint_names, capture_timestamp=10.,
                        arm_encoder_timestamp=10.)
        yy, xx = np.mgrid[:480, :640]
        rays = np.stack((xx, yy, np.ones_like(xx)), axis=-1) @ np.linalg.inv(intrinsic).T
        directions = rays @ transform[:3, :3].T
        depths = (-.5-transform[2, 3])/directions[:, :, 2]
        depth = np.rint(depths*1000).astype(np.uint16)
        mask, diagnostics = self.gun.project(metadata, intrinsic, transform, (480, 640))
        return metadata, depth, mask

    def test_measured_rear_mount_is_not_the_ideal_45_degree_mount(self):
        camera = self.cameras['rear_d435i']
        np.testing.assert_allclose(camera['position_b'], (-.434510, 0., .026751))
        optical_forward = quaternion_matrix(camera['quaternion_ros_wxyz'])[:, 2]
        np.testing.assert_allclose(optical_forward, (-.637519, 0., -.770435), atol=1e-6)
        self.assertAlmostEqual(camera['fx'], 607.6587524414062)

    def test_rendered_off_center_projection_matches_published_intrinsics(self):
        for camera in self.cameras.values():
            fx, fy, cx, cy = (camera[k] for k in ('fx', 'fy', 'cx', 'cy'))
            # Isolate the camera projection from robot geometry/occlusion.
            xml = ('<mujoco><visual><global offwidth="640" offheight="480"/>'
                   '<headlight ambient="1 1 1"/></visual><worldbody>'
                   '<camera name="c" sensorsize="640 480" resolution="640 480" '
                   'focalpixel="%f %f" principalpixel="%f %f"/>'
                   '<geom type="sphere" pos=".1 -.1 -2" size=".012" rgba="1 1 1 1"/>'
                   '</worldbody></mujoco>') % (fx, fy, 320-cx, cy-240)
            model = mujoco.MjModel.from_xml_string(xml)
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            renderer = mujoco.Renderer(model, height=480, width=640)
            try:
                renderer.update_scene(data, camera='c')
                yy, xx = np.where(renderer.render()[:, :, 0] > 150)
                # Image arrays sample integer pixel centres; OpenGL uses half pixels.
                np.testing.assert_allclose((xx.mean()+.5, yy.mean()+.5),
                                           (cx+.05*fx, cy+.05*fy), atol=.7)
            finally:
                renderer.close()

    def test_scene_uses_the_same_measured_camera_as_the_publisher(self):
        with tempfile.TemporaryDirectory(prefix='vd5_measured_', dir='/dev/shm') as cache:
            scene = build_scene(.05, cache, tool_markers=False, road_surface='tape',
                                robot_version='vd3', camera_calibration=self.cameras)
            model = mujoco.MjModel.from_xml_path(str(scene))
        for name, camera in self.cameras.items():
            index = model.camera(name).id
            np.testing.assert_allclose(model.cam_pos[index], camera['position_b'])
            np.testing.assert_allclose(model.cam_intrinsic[index],
                                       (camera['fx'], camera['fy'], 320-camera['cx'], camera['cy']-240), atol=1e-4)
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, model.key('vd1_default').id)
        q = np.array([.1, 2.4, -1.5, -1.3, .1, -.1])
        for name, value in zip(self.gun.joint_names, q):
            data.qpos[model.jnt_qposadr[model.joint(name).id]] = value
        mujoco.mj_forward(model, data)
        base, link = model.body('base_link').id, model.body('link06').id
        base_rotation = data.xmat[base].reshape(3, 3)
        expected_position = base_rotation.T @ (data.xpos[link]-data.xpos[base])
        expected_rotation = base_rotation.T @ data.xmat[link].reshape(3, 3)
        position, matrix = self.gun.link_pose(q)
        np.testing.assert_allclose(position, expected_position, atol=1e-8)
        np.testing.assert_allclose(matrix, expected_rotation, atol=1e-8)

    def test_gun_edge_is_rejected_even_when_depth_bleeds_onto_the_floor(self):
        metadata, depth, gun_pixels = self.fixture()
        rgb = np.full((480, 640, 3), 200, dtype=np.uint8)
        interior = cv2.erode(gun_pixels.astype(np.uint8), np.ones((17, 17), np.uint8))
        edges = cv2.morphologyEx(interior, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8))
        rgb[edges > 0] = 0
        for mode in ('skeleton', 'cross_sections'):
            with self.subTest(mode=mode):
                perception = SeamOnlyDualPerception(PerceptionConfig(mode='classical'),
                    gun_mask=self.gun, rear_centerline_mode=mode)
                result = perception.process_rear(metadata, rgb, depth)
                self.assertFalse(result['valid'], result)
                self.assertGreater(result['gun_self_mask_area_ratio'], 0.)
                self.assertEqual(result['gun_self_mask_source'], 'capture_arm_encoders_and_fixed_urdf_gun_mesh')

    def test_visible_seam_next_to_gun_is_retained_and_stale_encoders_are_rejected(self):
        metadata, depth, gun_pixels = self.fixture()
        rgb = np.full((480, 640, 3), 200, dtype=np.uint8)
        self.assertFalse(gun_pixels[70:410, 80].any())
        cv2.line(rgb, (80, 70), (80, 410), (0, 0, 0), 7)
        for mode in ('skeleton', 'cross_sections'):
            with self.subTest(mode=mode):
                sample = dict(metadata)
                perception = SeamOnlyDualPerception(PerceptionConfig(mode='classical'),
                    gun_mask=self.gun, rear_centerline_mode=mode)
                result = perception.process_rear(sample, rgb, depth)
                self.assertTrue(result['valid'], result['reason'])
                uv = np.rint(result['waypoints_uv']).astype(int)
                self.assertFalse(gun_pixels[uv[:, 1], uv[:, 0]].any())
                sample['arm_encoder_timestamp'] = 9.
                result = perception.process_rear(sample, rgb, depth)
                self.assertFalse(result['valid'])
                self.assertIn('40 ms', result['reason'])


if __name__ == '__main__':
    unittest.main()
