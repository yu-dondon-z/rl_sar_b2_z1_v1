"""Pure diagnostic schedule/recording tests; no hardware or full trials."""
import gzip
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from diagnose_vd1_base_response import (arguments, command_at, load_tool_sample, make_flat_ground,
                                       link_frame_velocity, quaternion_from_rotation, tool_command_at,
                                       load_fixed_tool_sample)


class BaseResponseTests(unittest.TestCase):
    def test_default_current_physics_and_speed(self):
        args = arguments([])
        self.assertEqual(args.speed, .4)
        self.assertEqual(args.physics_dt, .005)
        self.assertEqual(args.velocity_constraint, 'impulse')
        self.assertTrue(args.physx_implicit_position)
        self.assertEqual(args.tool_ramp_orientation, 'target')
        self.assertEqual(arguments(['--tool-ramp-orientation', 'slerp']).tool_ramp_orientation, 'slerp')
        self.assertFalse(arguments(['--no-physx-implicit-position']).physx_implicit_position)

    def test_straight_ramp(self):
        np.testing.assert_allclose(command_at('straight', 0), [0, 0, 0])
        np.testing.assert_allclose(command_at('straight', 1), [.2, 0, 0])
        np.testing.assert_allclose(command_at('straight', 2), [.4, 0, 0])
        np.testing.assert_allclose(command_at('straight', 30), [.4, 0, 0])

    def test_viewer_options_do_not_change_physics_defaults(self):
        before = arguments([])
        after = arguments(['--viewer', '--start-paused', '--realtime-factor', '.5'])
        self.assertTrue(after.viewer)
        self.assertTrue(after.start_paused)
        self.assertEqual(after.realtime_factor, .5)
        for field in ('physics_dt', 'speed', 'velocity_constraint', 'physx_implicit_position', 'tool_ramp_s'):
            self.assertEqual(getattr(before, field), getattr(after, field))
        with self.assertRaises(SystemExit):
            arguments(['--start-paused'])
        for value in ['0', '-1', 'nan', 'inf', '5']:
            with self.assertRaises(SystemExit):
                arguments(['--realtime-factor', value])
        with self.assertRaises(SystemExit):
            arguments(['--trace', 'trace.gz', '--tool-sample', 'sample.json'])

    def test_persistent_tool_fixture_has_original_recorded_command(self):
        fixture = Path(__file__).resolve().parents[1] / 'scripts/fixtures/vd1_fixed_camera_tool_command.json'
        sample = load_fixed_tool_sample(fixture)
        np.testing.assert_array_equal(sample['tool_pose_b'], [
            -1.0200000083422316, -0.04950019815216234, -0.3030399561570878,
            5.186645830358377e-17, -0.5315235017799165, 3.254642775631836e-17, 0.847043544958354])
        self.assertEqual(sample['step'], 200)
        self.assertEqual(sample['source_trace_sha256'],
                         'b03f58fcc0418f4b73126081c88edc8c4fdf67b6af2b7ee40efe301ffb13f8d3')

    def test_fixed_tool_fixture_validates_and_excludes_unrelated_truth(self):
        sample = dict(schema='vd1_fixed_camera_tool_command_v1', tool_pose_b=[-1, 0, -.3, 1, 0, 0, 0],
                      actual_world_tcp=[100, 200, 300])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'sample.json'
            path.write_text(json.dumps(sample))
            self.assertNotIn('actual_world_tcp', load_fixed_tool_sample(path))
            sample['tool_pose_b'][3] = 0
            path.write_text(json.dumps(sample))
            with self.assertRaises(ValueError):
                load_fixed_tool_sample(path)
            sample['schema'] = 'not_a_recorded_command'
            path.write_text(json.dumps(sample))
            with self.assertRaises(ValueError):
                load_fixed_tool_sample(path)

    def test_turns_are_symmetric_and_smoothed(self):
        for case, sign in [('left', 1), ('right', -1)]:
            for t, magnitude in [(10, 0), (10.25, .075), (10.5, .15), (15, .15), (19.75, .075), (20, 0)]:
                np.testing.assert_allclose(command_at(case, t), [.4, 0, sign * magnitude], atol=1e-12)

    def test_stop_and_restart_schedule(self):
        for case in ['stop', 'restart']:
            np.testing.assert_allclose(command_at(case, 15), [.4, 0, 0])
            np.testing.assert_allclose(command_at(case, 16), [.2, 0, 0])
            np.testing.assert_allclose(command_at(case, 17), [0, 0, 0])
            np.testing.assert_allclose(command_at(case, 22), [0, 0, 0])
        np.testing.assert_allclose(command_at('stop', 30), [0, 0, 0])
        np.testing.assert_allclose(command_at('restart', 23), [.2, 0, 0])
        np.testing.assert_allclose(command_at('restart', 24), [.4, 0, 0])

    def test_finite_schedule_validation(self):
        with self.assertRaises(ValueError):
            command_at('straight', math.nan)
        with self.assertRaises(ValueError):
            command_at('bogus', 2)
        for option in ['--duration', '--speed', '--yaw-rate', '--sample-time', '--curvature-max', '--tool-ramp-s']:
            with self.assertRaises(SystemExit):
                arguments([option, 'nan'])

    def test_optional_tool_ramp_and_quaternion_slerp(self):
        initial = np.array([-1., .1, -.2, 1., 0., 0., 0.])
        target = np.array([-1.1, -.1, -.3, 0., 0., 0., 1.])
        np.testing.assert_allclose(tool_command_at(0., None, target), target)
        np.testing.assert_allclose(tool_command_at(0., initial, target, 9., 'slerp'), initial)
        middle = tool_command_at(4.5, initial, target, 9., 'slerp')
        np.testing.assert_allclose(middle[:3], (initial[:3] + target[:3]) / 2.)
        np.testing.assert_allclose(middle[3:], [2**-.5, 0, 0, 2**-.5], atol=1e-12)
        np.testing.assert_allclose(tool_command_at(9., initial, target, 9., 'slerp'), target, atol=1e-12)
        np.testing.assert_allclose(tool_command_at(30., initial, target, 9., 'slerp'), target, atol=1e-12)

    def test_xyz_ramp_with_immediate_target_orientation(self):
        initial = np.array([-1., .1, -.2, 1., 0., 0., 0.])
        target = np.array([-1.1, -.1, -.3, 0., 0., 0., 1.])
        for time, blend in [(0., 0.), (4.5, .5), (9., 1.), (30., 1.)]:
            result = tool_command_at(time, initial, target, 9.)
            np.testing.assert_allclose(result[:3], (1.-blend)*initial[:3] + blend*target[:3])
            np.testing.assert_allclose(result[3:], target[3:])
        with self.assertRaises(ValueError):
            tool_command_at(0., initial, target, 9., 'invalid')

    def test_rotation_matrix_conversion(self):
        np.testing.assert_allclose(quaternion_from_rotation(np.eye(3)), [1, 0, 0, 0], atol=1e-12)
        np.testing.assert_allclose(np.abs(quaternion_from_rotation(np.diag([1., -1., -1.]))), [0, 1, 0, 0], atol=1e-12)
        angle = .7
        rotation = np.array([[math.cos(angle), -math.sin(angle), 0],
                             [math.sin(angle), math.cos(angle), 0], [0, 0, 1]])
        np.testing.assert_allclose(quaternion_from_rotation(rotation), [math.cos(angle/2), 0, 0, math.sin(angle/2)], atol=1e-12)

    def test_extracts_only_recorded_command_and_checks_time(self):
        tool = [-1.02, 0, -.3, 1, 0, 0, 0]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'trace.jsonl.gz'
            with gzip.open(path, 'wt') as handle:
                for t in [3.98, 4., 4.02]:
                    handle.write(json.dumps(dict(step=round(t / .02), sim_time_s=t, tool_pose_b=tool,
                                                 base_command_b=[.4, 0, 0], world_truth='must not be returned')) + '\n')
            sample = load_tool_sample(path, 4.)
            self.assertEqual(sample['sim_time_s'], 4.)
            self.assertEqual(sample['tool_pose_b'], tool)
            self.assertNotIn('world_truth', sample)
            with self.assertRaises(ValueError):
                load_tool_sample(path, 100.)

    def test_rejects_nonunit_tool_quaternion(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'trace.jsonl.gz'
            with gzip.open(path, 'wt') as handle:
                handle.write(json.dumps(dict(sim_time_s=4., tool_pose_b=[-1, 0, -.3, 0, 0, 0, 0])) + '\n')
            with self.assertRaises(ValueError):
                load_tool_sample(path, 4.)

    def test_flat_ground_only_changes_generated_terrain(self):
        source = ('<mujoco><asset><mesh name="road_left_0"/><mesh name="robot_mesh_00"/></asset>'
                  '<worldbody><geom name="crack_floor"/><geom name="road_left_0"/>'
                  '<body name="base_link"><inertial mass="60" pos="0 0 0"/>'
                  '<geom name="robot_collision"/><body name="FL_foot"/></body>'
                  '</worldbody><actuator><motor name="FL_motor" gear="1"/></actuator></mujoco>')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'scene.xml'
            path.write_text(source)
            result = make_flat_ground(path)
            self.assertEqual(path.read_text(), source)
            before, after = ET.fromstring(source), ET.parse(result).getroot()
            self.assertEqual(ET.tostring(before.find('worldbody/body')), ET.tostring(after.find('worldbody/body')))
            self.assertEqual(ET.tostring(before.find('actuator')), ET.tostring(after.find('actuator')))
            self.assertEqual(after.find('worldbody/geom').get('name'), 'diagnostic_flat_ground')
            self.assertEqual(after.find('worldbody/geom').get('pos'), '0 0 0')
            self.assertEqual([m.get('name') for m in after.findall('asset/mesh')], ['robot_mesh_00'])
            with self.assertRaises(FileExistsError):
                make_flat_ground(path)

    def test_velocity_uses_link_axes_not_rotated_principal_inertia_axes(self):
        import mujoco
        xml = ('<mujoco><worldbody><body name="base_link"><freejoint/>'
               '<inertial mass="1" pos=".1 .2 .3" diaginertia="1 2 3" quat=".7071067812 0 .7071067812 0"/>'
               '<site name="imu"/></body></worldbody><sensor><gyro name="imu_gyro" site="imu"/></sensor></mujoco>')
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        base = model.body('base_link').id
        for dof in range(6):
            data.qvel[:] = 0
            data.qvel[dof] = 1.
            mujoco.mj_forward(model, data)
            ordinary = link_frame_velocity(model, data, base)
            expected = np.zeros(6)
            expected[dof + 3 if dof < 3 else dof - 3] = 1.
            np.testing.assert_allclose(ordinary, expected, atol=1e-12)
            np.testing.assert_allclose(ordinary[:3], data.sensor('imu_gyro').data, atol=1e-12)
        data.qvel[:] = 0
        data.qvel[0] = 1.
        mujoco.mj_forward(model, data)
        principal = np.empty(6)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, base, principal, 1)
        self.assertGreater(abs(principal[5]), .99)
        self.assertLess(abs(principal[3]), 1e-10)


if __name__ == '__main__':
    unittest.main()
