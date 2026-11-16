"""Privileged debug geometry and display contract; no GUI or robot connection."""
import math
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from debug_vd1_truth_curve import (
    PrivilegedCurveCommands, TruthCurveViewer, TruthPolyline, arguments,
    make_curve, rear_curve_target, save_trajectory_svg,
)

TOOL = [-1.02, 0., -.30, 1., 0., 0., 0.]


class TruthCurveTests(unittest.TestCase):
    def test_point_curve_has_requested_profile_and_straight_ends(self):
        path = make_curve(.05)
        self.assertTrue(np.all(np.diff(path.points[:, 0]) > 0))
        np.testing.assert_array_equal(path.points[:, 2], 0.)
        np.testing.assert_array_equal(path.points[path.points[:, 0] <= 2, 1], 0.)
        np.testing.assert_array_equal(path.points[path.points[:, 0] >= 22, 1], 0.)
        self.assertAlmostEqual(float(np.max(path.points[:, 1])), .05/(3*(2*math.pi/20)**2), places=5)

    def test_sharp_curve_evaluation_has_four_times_baseline_curvature(self):
        path = make_curve(.40)
        np.testing.assert_allclose(path.points[:, 1], 4*make_curve(.10).points[:, 1])
        dx = path.points[1, 0]-path.points[0, 0]
        slope = np.gradient(path.points[:, 1], dx)
        curvature = np.gradient(slope, dx)/(1+slope*slope)**1.5
        self.assertAlmostEqual(float(np.max(np.abs(curvature))), .40, delta=.001)

    def test_projection_is_on_segment_not_just_nearest_sample(self):
        path = TruthPolyline([[0., 0., 0.], [1., 0., 0.], [2., 0., 0.]])
        p = path.project([.25, .5])
        np.testing.assert_allclose(p['point'], [.25, 0., 0.])
        self.assertAlmostEqual(p['distance'], .5)
        self.assertAlmostEqual(p['signed_distance'], .5)
        np.testing.assert_allclose(path.at(.75), [.75, 0., 0.])

    def test_invalid_point_geometry_is_rejected(self):
        for points in [[], [[0, 0, 0]]*3, [[0, 0, 0], [1, 0, 0], [math.nan, 0, 0]]]:
            with self.assertRaises(ValueError):
                TruthPolyline(points)

    def test_straight_center_has_no_turn_and_speed_ramp_is_preserved(self):
        commands = PrivilegedCurveCommands(make_curve(0), TOOL)
        v, tool, status = commands.update(0., [0., 0., .55], np.eye(3))
        np.testing.assert_allclose(v, [0., 0., 0.])
        v, tool, status = commands.update(3., [1., 0., .55], np.eye(3))
        np.testing.assert_allclose(v, [.4, 0., 0.])
        np.testing.assert_allclose(tool, TOOL)
        self.assertTrue(status['privileged'])
        self.assertTrue(status['rear_supported'])

    def test_left_offset_requests_right_turn_and_right_offset_left_turn(self):
        for y, sign in [(.10, -1), (-.10, 1)]:
            command, _, _ = PrivilegedCurveCommands(make_curve(0), TOOL).update(3., [0., y, .55], np.eye(3))
            self.assertGreater(sign*command[2], 0.)
            self.assertLessEqual(abs(command[2]), .8)

    def test_guidance_is_equivariant_under_world_yaw_and_translation(self):
        angle = .7
        rotation = np.array([[math.cos(angle), -math.sin(angle), 0.],
                             [math.sin(angle), math.cos(angle), 0.], [0., 0., 1.]])
        translation = np.array([2., -3., 0.])
        original = make_curve(.05)
        transformed = TruthPolyline(original.points @ rotation.T + translation)
        position = np.array([4., .08, .55])
        a = PrivilegedCurveCommands(original, TOOL).update(3., position, np.eye(3))
        b = PrivilegedCurveCommands(transformed, TOOL).update(3., rotation @ position+translation, rotation)
        np.testing.assert_allclose(a[0], b[0], atol=1e-12)
        np.testing.assert_allclose(a[1], b[1], atol=1e-12)

    def test_rear_target_stays_in_workspace_and_reports_clipping(self):
        target, supported, clipped = rear_curve_target(make_curve(0), [2., .5, .55], np.eye(3))
        np.testing.assert_allclose(target, [-1.02, -.15, -.30])
        self.assertTrue(supported)
        self.assertTrue(clipped)
        _, supported, _ = rear_curve_target(make_curve(0), [-20., 0., .55], np.eye(3))
        self.assertFalse(supported)

    def test_fixed_tool_mode_preserves_exact_recorded_pose(self):
        controller = PrivilegedCurveCommands(make_curve(.05), TOOL, tool_mode='fixed')
        _, tool, _ = controller.update(3., [8., 0., .55], np.eye(3))
        np.testing.assert_array_equal(tool, TOOL)

    def test_tool_transition_is_rate_limited_and_returned_arrays_are_copies(self):
        controller = PrivilegedCurveCommands(make_curve(0), TOOL)
        _, tool, _ = controller.update(3., [0., .5, .55], np.eye(3))
        self.assertLessEqual(np.linalg.norm(tool[:3]-TOOL[:3]), .15*.02+1e-12)
        before = controller.tool.copy()
        tool[:] = 100
        np.testing.assert_array_equal(controller.tool, before)

    def test_bad_inputs_rejected_and_default_cli_is_visible(self):
        self.assertFalse(arguments([]).headless)
        self.assertTrue(arguments(['--headless']).headless)
        for kwargs in [dict(speed=0), dict(lookahead=0), dict(tool_mode='fake')]:
            with self.assertRaises(ValueError):
                PrivilegedCurveCommands(make_curve(0), TOOL, **kwargs)
        with self.assertRaises(ValueError):
            PrivilegedCurveCommands(make_curve(0), TOOL).update(0., [math.nan, 0, 0], np.eye(3))

    def test_svg_can_be_previewed_as_xml_with_reference_and_actual_trajectories(self):
        row = dict(base_position_w=[2., .1, .55], tcp_position_w=[1., .05, .2], sim_time_s=3., tcp_curve_error_m=.05)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'trajectory.svg'
            save_trajectory_svg(path, make_curve(.05), [])
            ET.parse(path)
            save_trajectory_svg(path, make_curve(.05), [row])
            tree = ET.parse(path)
            self.assertEqual(len(tree.findall('{http://www.w3.org/2000/svg}polyline')), 3)
            self.assertIn('PRIVILEGED', path.read_text())

    def test_native_draw_geometry_and_display_interaction_are_isolated(self):
        import mujoco
        import mujoco.viewer
        model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
          <body name="base_link" pos="0 0 .55"><freejoint/>
          <geom type="box" size=".2 .1 .1" mass="1"/>
          <site name="tool_tcp" pos="-1.02 0 -.3"/>
          </body></worldbody></mujoco>''')
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        original_qpos, original_mass = data.qpos.copy(), model.body_mass.copy()
        handle = mock.Mock()
        handle.lock.return_value = threading.RLock()
        handle.is_running.return_value = True
        handle.cam = mujoco.MjvCamera()
        handle.opt = mujoco.MjvOption()
        handle.user_scn = mujoco.MjvScene(model, maxgeom=2000)
        with mock.patch('mujoco.viewer.launch_passive', return_value=handle):
            viewer = TruthCurveViewer(model, data, make_curve(.05), start_paused=True)
        _, _, guidance = PrivilegedCurveCommands(make_curve(.05), TOOL).update(0., [0, 0, .55], np.eye(3))
        viewer.update(data, [0, 0, 0], dict(guidance=guidance, body_velocity_b=[0, 0, 0]))
        self.assertGreater(handle.user_scn.ngeom, 50)
        self.assertIn('PRIVILEGED', handle.user_scn.geoms[0].label)
        viewer.data.qpos[:] += 1
        viewer.model.body_mass[:] += 1
        np.testing.assert_array_equal(data.qpos, original_qpos)
        np.testing.assert_array_equal(model.body_mass, original_mass)
        viewer._key(32)
        self.assertFalse(viewer.paused.is_set())
        viewer.close()


if __name__ == '__main__':
    unittest.main()
