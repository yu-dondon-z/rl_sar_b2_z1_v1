"""Continuous VD14 runtime and spectator-only backdrop; no actor/hardware I/O."""
import contextlib
import io
import itertools
import os
from pathlib import Path
import sys
import tempfile
import unittest

os.environ.setdefault('MUJOCO_GL', 'egl')
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))

import mujoco
import numpy as np

import deploy_vd12_vision as entry
from vd1_mujoco_scene import build_scene
from vd5_camera_geometry import measured_camera_calibration
from vd14_viewer import SpectatorState
import vd14_mujoco_contract as backend


class ContinuousRuntime(unittest.TestCase):
    def test_bare_vd14_command_opens_viewer_and_has_no_time_limit(self):
        args = entry.parse([], version=14)
        self.assertEqual(args.duration, 0.)
        self.assertTrue(args.viewer)
        self.assertTrue(args.vision_window)
        self.assertFalse(entry.parse(['--headless'], version=14).viewer)
        self.assertTrue(entry.parse(['--viewer'], version=14).viewer)
        self.assertFalse(entry.parse(['--no-vision-window'], version=14).vision_window)
        self.assertTrue(entry.parse(['--vision-window'], version=14).vision_window)

    def test_optional_duration_has_no_thirty_second_cap(self):
        for value in ('0', '.02', '30.02', '300', '3600'):
            self.assertEqual(entry.parse(['--duration', value], version=14).duration, float(value))
        for value in ('-.02', '.001', '.021', 'nan', 'inf'):
            with self.subTest(value=value), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    entry.parse(['--duration', value], version=14)

    def test_control_ticks_stream_past_old_limit_without_a_fixed_end(self):
        ticks = entry.control_steps(0.)
        self.assertIsInstance(ticks, itertools.count)
        self.assertEqual(list(itertools.islice(ticks, 1498, 1503)), [1498,1499,1500,1501,1502])
        self.assertEqual(list(entry.control_steps(.06)), [0,1,2])

    def test_other_versions_retain_their_diagnostic_defaults(self):
        for version in (12,13):
            args = entry.parse([], version=version)
            self.assertFalse(args.viewer)
            self.assertFalse(args.vision_window)
            self.assertEqual(args.duration, 30.)


class SpectatorBackground(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cache = tempfile.TemporaryDirectory(prefix='vd14_background_test_', dir='/dev/shm')
        cls.models, cls.states, cls.paths = [], [], []
        for light in (False,True):
            path = build_scene(.05, Path(cls.cache.name)/str(light), road_surface='tape',
                robot_version='vd13', tool_markers=False,
                camera_calibration=measured_camera_calibration(), odometry_sensors=True,
                light_background=light)
            model = mujoco.MjModel.from_xml_path(str(path))
            data = mujoco.MjData(model)
            mujoco.mj_resetDataKeyframe(model,data,model.key('vd1_default').id)
            mujoco.mj_forward(model,data)
            cls.models.append(model); cls.states.append(data)
            cls.paths.append(path)

    @classmethod
    def tearDownClass(cls):
        cls.cache.cleanup()

    def test_backdrop_adds_no_geometry_and_changes_no_dynamics_or_calibration(self):
        original, light = self.models
        self.assertEqual(original.ngeom, light.ngeom)
        self.assertEqual(original.nbody, light.nbody)
        self.assertEqual(original.nu, light.nu)
        for name in ('body_mass','body_inertia','geom_pos','geom_quat','geom_size',
                     'geom_contype','geom_conaffinity','geom_rgba','actuator_gainprm',
                     'actuator_biasprm','cam_pos','cam_quat','cam_intrinsic','sensor_type'):
            np.testing.assert_array_equal(getattr(original,name),getattr(light,name),err_msg=name)

    def test_spectator_range_does_not_change_source_model_or_feed_ui_edits_back(self):
        model, state = self.models[1], self.states[1]
        positions = state.qpos.copy()
        spectator = SpectatorState(self.paths[1], state)
        self.assertAlmostEqual(model.vis.map.zfar*model.stat.extent,6.)
        self.assertAlmostEqual(spectator.model.vis.map.zfar*spectator.model.stat.extent,250.)
        np.testing.assert_array_equal(spectator.data.xpos,state.xpos)
        spectator.data.qpos[0] += 5.
        spectator.update(state)
        np.testing.assert_array_equal(state.qpos,positions)
        np.testing.assert_array_equal(spectator.data.qpos,state.qpos)

    def test_spectator_background_is_light_but_robot_rgb_and_depth_are_unchanged(self):
        rgb_pairs, depth_pairs, spectator = [], [], []
        for model,data in zip(self.models,self.states):
            options = mujoco.MjvOption();options.geomgroup[3]=0
            with mujoco.Renderer(model,height=120,width=160) as renderer:
                camera = mujoco.MjvCamera();mujoco.mjv_defaultCamera(camera)
                backend.shared.configure_camera(camera,data.xpos[model.body('base_link').id])
                renderer.update_scene(data,camera=camera,scene_option=options)
                spectator.append(renderer.render().copy())
                rgb,depth = [],[]
                for name in ('front_rgb','rear_d435i'):
                    entry.update_sensor_scene(renderer,data,name,options)
                    self.assertFalse(renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SKYBOX])
                    rgb.append(renderer.render().copy())
                    renderer.enable_depth_rendering()
                    depth.append(renderer.render().copy())
                    renderer.disable_depth_rendering()
                rgb_pairs.append(rgb);depth_pairs.append(depth)
        self.assertLess(float(spectator[0][0,0].mean()),1.)
        self.assertGreater(float(spectator[1][0,0].mean()),70.)
        self.assertGreater(float(np.ptp(spectator[1][0].astype(float))),5.)
        for index in (0,1):
            np.testing.assert_array_equal(rgb_pairs[0][index],rgb_pairs[1][index])
            np.testing.assert_array_equal(depth_pairs[0][index],depth_pairs[1][index])


if __name__ == '__main__':
    unittest.main()
