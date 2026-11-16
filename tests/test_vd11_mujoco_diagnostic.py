"""Pure CPU contract checks; no physics stepping, renderer, export or robot IO."""

import __future__
import copy
import importlib.util
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from diagnose_vd11_gait import (ARM_RESET, COMMAND_CAMERA, EXPERIMENTS, PROFILE, TRAINING, actor_observation,
    default_position, final_body_target, geometry_is_invalid, ideal_front_command,
    load_geometry_audit, parse, quaternion_from_euler_xyz, quaternion_matrix,
    quaternion_multiply, straight_world_seam, termination_causes, tube_tip_height,
    validate_checkpoint, world_seam_command, VD10_TCP_POSITION, VD10_TCP_ROTATION)
from vd1_mujoco_scene import ARM_JOINT_NAMES, DEFAULT_JOINT_POSITION, EXPECTED_JOINT_NAMES


def checkpoint(experiment='b2_z1_vd11_motion_quality_amp'):
    return dict(training_experiment_name=experiment, iter=EXPERIMENTS[experiment],
        vd5_curriculum_state=dict(version=1, startup_profile=copy.deepcopy(PROFILE),
            ground_blend=1., ground_transition_steps=20_000, seam_trailing_distance_m=.8))


class VD11DiagnosticTests(unittest.TestCase):
    def test_nominal_pose_uses_new_arm_without_mutating_vd5(self):
        old = dict(DEFAULT_JOINT_POSITION)
        values = default_position()
        np.testing.assert_array_equal([values[EXPECTED_JOINT_NAMES.index(n)] for n in ARM_JOINT_NAMES], ARM_RESET)
        self.assertEqual(DEFAULT_JOINT_POSITION, old)
        self.assertEqual(DEFAULT_JOINT_POSITION['joint2'], 2.55)

    def test_tcp_axis_and_offset_are_new_tube(self):
        np.testing.assert_array_equal(VD10_TCP_POSITION, (.975, 0., .1))
        np.testing.assert_array_equal(VD10_TCP_ROTATION @ np.array([1., 0., 0.]), [1., 0., 0.])

    def test_explicit_legacy_nominal_target_remains_reproducible(self):
        pose = final_body_target()
        self.assertEqual(pose[0], -.8)
        self.assertAlmostEqual(pose[2], -.4828209650448855, places=7)
        axis = quaternion_matrix(pose[3:])[:, 0]
        self.assertLess(axis[2], 0.)
        self.assertAlmostEqual(pose[2] + .5 - tube_tip_height(axis[2], 0.), .005)

    def test_upward_axis_includes_lower_rear_end(self):
        self.assertAlmostEqual(tube_tip_height(1.), 1.105)
        self.assertAlmostEqual(tube_tip_height(-1.), .005)

    def test_default_checkpoint_is_vd11_only(self):
        self.assertEqual(validate_checkpoint(checkpoint()), ('b2_z1_vd11_motion_quality_amp', 5999))
        with self.assertRaises(ValueError):
            validate_checkpoint(checkpoint('b2_z1_vd10_hollow_gun_amp'))
        self.assertEqual(validate_checkpoint(checkpoint('b2_z1_vd10_hollow_gun_amp'), compare_vd10=True),
                         ('b2_z1_vd10_hollow_gun_amp', 5998))

    def test_world_target_never_sags_with_base(self):
        initial_b = np.array([-.95, 0., .35])
        start_w = initial_b + [0., 0., .5]
        quat = quaternion_from_euler_xyz(0., 0., 0.)
        start_q = quaternion_from_euler_xyz(0., .3, np.pi)
        seam = straight_world_seam([0., 0., .5], quat, .2, 30.)
        for base_z in (.5, .41, .25, .7):
            with self.subTest(base_z=base_z):
                base = np.array([.5, 0., base_z])
                pose, world, fraction = world_seam_command(base, quat, start_w, start_q, seam, 3.)
                np.testing.assert_allclose(base + quaternion_matrix(quat) @ pose[:3], world, atol=1e-12)
                np.testing.assert_allclose(world[:2], [-.3, 0.], atol=1e-12)
                self.assertEqual(fraction, 1.)
                self.assertGreater(world[2], 0.)
                self.assertAlmostEqual(world[2], .0171790349551144)

    def test_world_rim_clearance_tracks_full_base_tilt(self):
        start_w = np.array([-.95, 0., .85])
        start_q = quaternion_from_euler_xyz(0., .3, np.pi)
        seam = straight_world_seam([0., 0., .5], [1., 0., 0., 0.], .2, 30.)
        for rpy in ((.12, .2, -.3), (-.2, -.1, 1.0), (.05, -.4, 0.)):
            quat = quaternion_from_euler_xyz(*rpy)
            base = np.array([.7, .2, .41])
            pose, world, _ = world_seam_command(base, quat, start_w, start_q, seam, 3.)
            axis_world = quaternion_matrix(quat) @ quaternion_matrix(pose[3:])[:, 0]
            self.assertAlmostEqual(world[2] - tube_tip_height(axis_world[2], 0.), .005)
            np.testing.assert_allclose(base + quaternion_matrix(quat) @ pose[:3], world, atol=1e-12)

    def test_start_world_pose_is_anchored_during_ramp_and_base_motion(self):
        start_w = np.array([-.95, 0., .85])
        start_q = quaternion_from_euler_xyz(0., .3, np.pi)
        seam = straight_world_seam([0., 0., .5], [1., 0., 0., 0.], .2, 30.)
        moved_base = np.array([.2, -.04, .41])
        moved_quat = quaternion_from_euler_xyz(.1, -.1, .3)
        pose, world, fraction = world_seam_command(moved_base, moved_quat, start_w, start_q, seam, 0.)
        self.assertEqual(fraction, 0.)
        np.testing.assert_allclose(world, start_w, atol=1e-12)
        actual_start_q = quaternion_multiply(moved_quat, pose[3:])
        np.testing.assert_allclose(quaternion_matrix(actual_start_q), quaternion_matrix(start_q), atol=1e-12)
        pose, world, fraction = world_seam_command(moved_base, moved_quat, start_w, start_q, seam, 1.)
        self.assertAlmostEqual(fraction, .5)
        axis_world = quaternion_matrix(moved_quat) @ quaternion_matrix(pose[3:])[:, 0]
        final_world = np.array([moved_base[0] - .8, 0., tube_tip_height(axis_world[2])])
        np.testing.assert_allclose(world, .5 * (start_w + final_world), atol=1e-12)

    def test_straight_seam_is_world_fixed_and_teacher_steers_toward_it(self):
        quat = np.array([1., 0., 0., 0.])
        seam = straight_world_seam([0., 0., .5], quat, .2, 30.)
        original = seam.copy()
        for y, expected_sign in ((.1, -1.), (-.1, 1.)):
            command, report = ideal_front_command([.5, y, .41], quat, seam, .2)
            self.assertEqual(command.shape, (3,))
            self.assertTrue(report['ideal_path_visible'])
            self.assertTrue(report['truth_generated'])
            self.assertFalse(report['rendered_camera'])
            self.assertGreater(expected_sign * command[2], 0.)
        np.testing.assert_array_equal(seam, original)

    def test_ideal_camera_matches_training_cpu_geometry(self):
        import torch
        source = TRAINING / 'source/b2_z1/b2_z1/tasks/manager_based/b2_z1/mdp/commands/curve_geometry.py'
        spec = importlib.util.spec_from_file_location('training_curve_cpu_reference', source)
        reference = importlib.util.module_from_spec(spec)
        # Training uses Python 3.11, while crack_vision is Python 3.8. Defer
        # annotations only; execute the original CPU geometry unchanged.
        exec(compile(source.read_text(), str(source), 'exec',
                     flags=__future__.annotations.compiler_flag), reference.__dict__)
        seam = straight_world_seam([0., 0., .5], [1., 0., 0., 0.], .2, 30.)
        for base, rpy in (([.5, .1, .41], [.1, -.05, .2]),
                          ([1., -.1, .6], [-.03, .08, -.1]),
                          ([.2, 0., .5], [0., 0., 3.])):
            quat = quaternion_from_euler_xyz(*rpy)
            rotation = quaternion_matrix(quat)
            camera_w = np.asarray(base) + rotation @ np.asarray(COMMAND_CAMERA['position_b'])
            camera_rotation = rotation @ quaternion_matrix(quaternion_from_euler_xyz(*COMMAND_CAMERA['rpy_b']))
            tensor = lambda a: torch.tensor(np.asarray(a), dtype=torch.float64)
            recovered_c, _, visible = reference.perceive_ground_curve(
                tensor((seam - camera_w) @ camera_rotation)[None],
                tensor(camera_rotation.T @ np.array([0., 0., 1.]))[None],
                tensor([camera_w[2]]), tuple(COMMAND_CAMERA['intrinsics']),
                tuple(COMMAND_CAMERA['image_size']), tuple(COMMAND_CAMERA['depth_range']))
            recovered_w = camera_w + recovered_c.numpy()[0] @ camera_rotation.T
            yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
            c, s = np.cos(yaw), np.sin(yaw)
            points_h = (recovered_w[:, :2] - np.asarray(base)[:2]) @ np.array([[c, -s], [s, c]])
            lateral, heading, curvature, valid = reference.curve_tracking_errors(
                tensor(points_h)[None], visible, COMMAND_CAMERA['lookahead_m'])
            actual, report = ideal_front_command(base, quat, seam, .2)
            self.assertEqual(report['ideal_path_visible'], bool(valid.item()))
            np.testing.assert_allclose([report['lateral_error_m'], report['heading_error_rad'],
                                       report['curvature_m_inv']],
                                      [lateral.item(), heading.item(), curvature.item()], atol=1e-12)
            expected_yaw = np.clip(1.5 * heading.item() + .2 * curvature.item(), -.8, .8) if valid else 0.
            self.assertAlmostEqual(actual[2], expected_yaw, places=12)

    def test_sensor_contract_retains_70_values_without_world_observation(self):
        default = default_position()
        sensor = dict(gyro_b=np.array([1., 2., 3.]), gravity_b=np.array([0., 0., -1.]),
                      q=default + .01, qd=np.ones(18) * 2.,
                      base_position_w='MUST NOT BE OBSERVED', tcp_position_w='MUST NOT BE OBSERVED',
                      contact_force='MUST NOT BE OBSERVED')
        pose = np.r_[[-.8, 0., -.4], [1., 0., 0., 0.]]
        obs = actor_observation([.2, 0., 0.], pose, sensor, default, np.zeros(18))
        self.assertEqual(obs.shape, (70,))
        np.testing.assert_allclose(obs[:10], np.r_[[.2, 0., 0.], pose], atol=1e-7)
        np.testing.assert_allclose(obs[10:13], [.2, .4, .6], atol=1e-7)
        np.testing.assert_allclose(obs[16:34], .01, atol=1e-7)
        np.testing.assert_allclose(obs[34:52], .1, atol=1e-7)

    def test_real_geometry_guard_trips_on_logged_pose_not_on_force(self):
        audit = load_geometry_audit()
        q = default_position()
        self.assertFalse(geometry_is_invalid(audit.audit(EXPECTED_JOINT_NAMES, q)))
        # Recorded user replay near t=2 s. No simulation or state editing here:
        # the independent FK checks these actual historical joint angles.
        recorded_arm = (.0011, -.0055, .0081, .7012, -.0256, -.2756)
        for name, angle in zip(ARM_JOINT_NAMES, recorded_arm):
            q[EXPECTED_JOINT_NAMES.index(name)] = angle
        report = audit.audit(EXPECTED_JOINT_NAMES, q)
        self.assertTrue(geometry_is_invalid(report))
        self.assertTrue(any(item['other_link'] == 'base_link' for item in report['confirmed_intersections']))
        self.assertTrue(report['evaluation_only'])
        self.assertFalse(report['actor_input'])

    def test_uncertain_geometry_is_not_a_confirmed_collision(self):
        self.assertFalse(geometry_is_invalid(dict(status='uncertain', confirmed_invalid=False,
                                                 minimum_clearance_lower_bound_m=-.01)))
        with self.assertRaises(ValueError):
            geometry_is_invalid(dict(status='safe', confirmed_invalid=True))
        with self.assertRaises(ValueError):
            geometry_is_invalid(dict(status='invalid'))

    def test_old_actor_other_iteration_and_profile_are_rejected(self):
        for mutate in ('experiment', 'iteration', 'version', 'arm', 'normalizer', 'blend'):
            with self.subTest(mutate=mutate):
                value = checkpoint()
                if mutate == 'experiment':
                    value['training_experiment_name'] = 'b2_z1_vd5_startup_amp'
                elif mutate == 'iteration':
                    value['iter'] = 59
                elif mutate == 'version':
                    value['vd5_curriculum_state']['startup_profile']['vd10']['version'] = 1
                elif mutate == 'arm':
                    value['vd5_curriculum_state']['startup_profile']['vd10']['arm_joint_reset_rad'][1] = 2.55
                elif mutate == 'normalizer':
                    value['obs_norm_state_dict'] = {}
                else:
                    value['vd5_curriculum_state']['ground_blend'] = .9
                with self.assertRaises(ValueError):
                    validate_checkpoint(value)

    def test_limits_are_not_disabled_by_short_startup_history(self):
        force = np.zeros((3, 3))
        self.assertFalse(any(termination_causes(force, [0.] * 19 + [20.], [0], .5).values()))
        self.assertTrue(termination_causes(force, [21.] * 20, [0], 0.)['tool_support'])
        self.assertFalse(termination_causes(force, [21.] * 19, [0], 0.)['tool_support'])
        self.assertTrue(termination_causes(force, [151.], [0], 0.)['tool_support'])
        force[0, 0] = 1.01
        self.assertTrue(termination_causes(force, [0.], [0], .51)['base_contact'])
        self.assertTrue(termination_causes(force, [0.], [0], .51)['bad_orientation'])

    def test_cli_is_headless_and_duration_bounded(self):
        args = parse(['--checkpoint', 'model_5999.pt'])
        self.assertFalse(args.viewer)
        self.assertEqual(args.duration, 30.)
        self.assertEqual(args.command_mode, 'world-seam')
        self.assertFalse(args.compare_vd10)
        self.assertIsNone(args.save_frames)
        args = parse(['--checkpoint', 'model_5998.pt', '--compare-vd10',
                      '--command-mode', 'legacy-fixed-body', '--save-frames', '0', '2', '5'])
        self.assertTrue(args.compare_vd10)
        self.assertEqual(args.command_mode, 'legacy-fixed-body')
        self.assertEqual(args.save_frames, [0., 2., 5.])
        with self.assertRaises(SystemExit):
            parse(['--checkpoint', 'model_5999.pt', '--duration', '61'])
        with self.assertRaises(SystemExit):
            parse(['--checkpoint', 'model_5999.pt', '--save-frames', '-1'])


if __name__ == '__main__':
    unittest.main()
