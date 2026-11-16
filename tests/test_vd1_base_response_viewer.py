"""Headless presentation/isolation regressions; never create a GUI or robot link."""

from contextlib import nullcontext
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import mujoco
import mujoco.viewer
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import vd1_base_response_viewer as presentation


class FakeHandle:
    """Exercise actual label geometry without launching the passive viewer."""

    def __init__(self, model, data, key_callback, **options):
        self.model, self.data = model, data
        self.key_callback, self.options = key_callback, options
        self.cam, self.opt = mujoco.MjvCamera(), mujoco.MjvOption()
        self.user_scn = mujoco.MjvScene(model, maxgeom=100)
        self.running = True
        self.sync_count = 0
        self.on_sync = None

    def lock(self):
        return nullcontext()

    def is_running(self):
        return self.running

    def sync(self):
        self.sync_count += 1
        if self.on_sync is not None:
            self.on_sync(self)

    def close(self):
        self.running = False


class DiagnosticViewerTests(unittest.TestCase):
    def setUp(self):
        xml = '''<mujoco><worldbody>
          <body name="base_link" pos="0 0 .6"><freejoint/>
            <geom type="sphere" size=".1" mass="1"/>
            <site name="imu"/>
            <body name="test_arm" pos="0 0 .2">
              <joint name="hinge" type="hinge"/>
              <geom type="sphere" size=".05" mass=".1"/>
            </body>
          </body>
          <body name="mocap_marker" mocap="true" pos="1 0 1">
            <geom type="sphere" size=".02" contype="0" conaffinity="0"/>
          </body>
        </worldbody><actuator><motor joint="hinge"/></actuator>
        <sensor><gyro name="imu_gyro" site="imu"/>
          <jointpos name="hinge_position" joint="hinge"/></sensor></mujoco>'''
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.data.time = 1.234
        self.data.qpos[:3] = [.1, .2, .6]
        self.data.qvel[:] = np.arange(self.model.nv) / 100.
        self.data.ctrl[:] = .4
        self.data.mocap_pos[:] = [1.1, .2, .9]
        self.data.qfrc_applied[:] = 1.25
        self.data.xfrc_applied[:] = 2.5
        mujoco.mj_forward(self.model, self.data)
        patcher = patch('mujoco.viewer.launch_passive', side_effect=FakeHandle)
        self.launch = patcher.start()
        self.addCleanup(patcher.stop)

    def viewer(self, **options):
        result = presentation.DiagnosticViewer(self.model, self.data, **options)
        self.addCleanup(result.close)
        return result

    def test_telemetry_distinguishes_command_from_unmodified_evaluation(self):
        command = np.array([.4, 0., -.15])
        actual = np.array([.527, -.031, .008])
        labels = presentation.telemetry_labels(12.345, command, actual, 3.25)
        self.assertIsInstance(labels, tuple)
        self.assertEqual(len(labels), 4)
        self.assertTrue(all(isinstance(label, str) for label in labels))
        self.assertIn('NO VISION', labels[0])
        self.assertIn('RUNNING', labels[0])
        self.assertIn('12.35s', labels[1])
        self.assertIn('command vx=0.400 m/s', labels[1])
        self.assertIn('w=-0.150 rad/s', labels[1])
        self.assertIn('EVAL actual vx=0.527, vy=-0.031 m/s', labels[2])
        self.assertIn('tilt=3.25 deg', labels[2])
        self.assertIn('Fixed BASE-frame TCP', labels[3])
        self.assertIn('no camera or heading feedback', labels[3])
        np.testing.assert_array_equal(command, [.4, 0., -.15])
        np.testing.assert_array_equal(actual, [.527, -.031, .008])

    def test_pause_label_is_explicit(self):
        labels = presentation.telemetry_labels(0., [0., 0., 0.], [0., 0., 0.], 0., paused=True)
        self.assertIn('PAUSED', labels[0])
        self.assertIn('SPACE', labels[0])
        self.assertNotIn('RUNNING', labels[0])

    def test_viewer_has_its_own_model_and_data(self):
        viewer = self.viewer()
        self.assertIsNot(viewer.model, self.model)
        self.assertIsNot(viewer.data, self.data)
        self.assertIs(viewer.handle.model, viewer.model)
        self.assertIs(viewer.handle.data, viewer.data)
        self.assertFalse(np.shares_memory(viewer.model.body_mass, self.model.body_mass))
        self.assertFalse(np.shares_memory(viewer.data.qpos, self.data.qpos))
        self.assertFalse(np.shares_memory(viewer.data.sensordata, self.data.sensordata))
        self.assertFalse(viewer.handle.options['show_left_ui'])
        self.assertFalse(viewer.handle.options['show_right_ui'])
        self.assertEqual(self.launch.call_count, 1)

    def test_update_copies_state_and_renders_real_evaluation_labels(self):
        viewer = self.viewer()
        self.data.time = 4.56
        self.data.qpos[:3] = [.5, -.1, .7]
        self.data.qvel[:] = -.23
        self.data.ctrl[:] = -.1
        self.data.mocap_pos[:] = [1.2, -.3, 1.1]
        evaluation = {'body_velocity_b': [.53, -.02, .001], 'tilt_deg': 4.5}
        viewer.update(self.data, [.4, 0., .15], evaluation)
        self.assertEqual(viewer.data.time, self.data.time)
        for name in ('qpos', 'qvel', 'ctrl', 'mocap_pos', 'mocap_quat'):
            np.testing.assert_array_equal(getattr(viewer.data, name), getattr(self.data, name))
        np.testing.assert_array_equal(viewer.data.qfrc_applied, 0.)
        np.testing.assert_array_equal(viewer.data.xfrc_applied, 0.)
        self.assertEqual(viewer.handle.sync_count, 1)
        self.assertEqual(viewer.handle.user_scn.ngeom, 28)
        self.assertEqual(viewer.model.ngeom, self.model.ngeom)
        labels = [geom.label for geom in viewer.handle.user_scn.geoms[:4]]
        self.assertIn('command vx=0.400 m/s', labels[1])
        self.assertIn('EVAL actual vx=0.530, vy=-0.020 m/s', labels[2])
        self.assertEqual(evaluation, {'body_velocity_b': [.53, -.02, .001], 'tilt_deg': 4.5})
        base = viewer.model.body('base_link').id
        np.testing.assert_allclose(viewer.handle.cam.lookat,
                                   viewer.data.xpos[base] + [-.35, 0., -.08])

    def test_display_sync_cannot_modify_physics_or_actor_inputs(self):
        viewer = self.viewer()
        names = ('qpos', 'qvel', 'ctrl', 'sensordata', 'qfrc_applied', 'xfrc_applied')
        original_data = {name: getattr(self.data, name).copy() for name in names}
        original_mass = self.model.body_mass.copy()
        original_time = self.data.time

        def user_interaction(handle):
            handle.data.qpos[0] = 100.
            handle.data.qvel[:] = -100.
            handle.data.ctrl[:] = 99.
            handle.data.sensordata[:] = 66.
            handle.data.qfrc_applied[:] = 88.
            handle.data.xfrc_applied[:] = 77.
            handle.model.body_mass[:] = 123.

        viewer.handle.on_sync = user_interaction
        viewer.update(self.data, [.4, 0., 0.], {'body_velocity_b': [.5, 0., 0.], 'tilt_deg': 1.})
        self.assertEqual(viewer.data.qpos[0], 100.)
        for name, value in original_data.items():
            np.testing.assert_array_equal(getattr(self.data, name), value, err_msg=name)
        np.testing.assert_array_equal(self.model.body_mass, original_mass)
        self.assertEqual(self.data.time, original_time)
        viewer.handle.on_sync = None
        viewer.update(self.data, [.4, 0., 0.])
        np.testing.assert_array_equal(viewer.data.qpos, self.data.qpos)
        np.testing.assert_array_equal(viewer.data.qfrc_applied, 0.)
        np.testing.assert_array_equal(viewer.data.xfrc_applied, 0.)

    def test_update_also_works_without_a_user_scene(self):
        viewer = self.viewer()
        viewer.handle.user_scn = None
        viewer.update(self.data, [.4, 0., 0.])
        self.assertEqual(viewer.handle.sync_count, 1)

    def test_space_toggles_pause_without_changing_simulation(self):
        viewer = self.viewer(start_paused=True)
        original = self.data.qpos.copy()
        original_time = self.data.time
        self.assertTrue(viewer.paused.is_set())
        viewer.handle.key_callback(ord('Q'))
        self.assertTrue(viewer.paused.is_set())
        viewer.handle.key_callback(32)
        self.assertFalse(viewer.paused.is_set())
        viewer.handle.key_callback(32)
        self.assertTrue(viewer.paused.is_set())
        np.testing.assert_array_equal(self.data.qpos, original)
        self.assertEqual(self.data.time, original_time)

    def test_pause_wait_does_not_step_or_reset_simulation(self):
        viewer = self.viewer(start_paused=True)
        original_time = self.data.time
        with patch.object(presentation.time, 'sleep', side_effect=lambda _: viewer._key(32)) as sleep:
            self.assertTrue(viewer.wait_until_running(self.data, [.4, 0., 0.], None, [False]))
        self.assertEqual(sleep.call_count, 1)
        self.assertEqual(self.data.time, original_time)
        self.assertFalse(viewer.paused.is_set())

    def test_stopped_wait_returns_without_waiting(self):
        viewer = self.viewer(start_paused=True)
        with patch.object(presentation.time, 'sleep') as sleep:
            self.assertFalse(viewer.wait_until_running(self.data, [.4, 0., 0.], None, [True]))
        sleep.assert_not_called()
        self.assertEqual(viewer.handle.sync_count, 0)

    def test_pacing_uses_wall_clock_without_stepping_simulation(self):
        viewer = self.viewer(realtime_factor=2.)
        original_time = self.data.time
        with patch.object(presentation.time, 'monotonic', side_effect=[10.003, 10.03]):
            with patch.object(presentation.time, 'sleep') as sleep:
                viewer.pace(10., .02, [False])
        self.assertEqual(sleep.call_count, 1)
        self.assertAlmostEqual(sleep.call_args.args[0], .007)
        self.assertEqual(self.data.time, original_time)

    def test_close_stops_viewer_updates(self):
        viewer = self.viewer()
        self.assertTrue(viewer.is_running())
        viewer.close()
        self.assertFalse(viewer.is_running())
        viewer.update(self.data, [.4, 0., 0.])
        self.assertEqual(viewer.handle.sync_count, 0)
        self.assertFalse(viewer.wait_until_running(self.data, [.4, 0., 0.], None, [False]))

    def test_invalid_realtime_factors_are_rejected_before_launch(self):
        for value in (0., -1., 4.001, float('nan'), float('inf')):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    presentation.DiagnosticViewer(self.model, self.data, realtime_factor=value)
        self.launch.assert_not_called()


if __name__ == '__main__':
    unittest.main()
