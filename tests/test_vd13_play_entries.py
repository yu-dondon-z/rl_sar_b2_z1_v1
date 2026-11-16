"""VD13 mount identity and isolated entries; no simulation or robot I/O."""
import copy
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import deploy_vd12_vision as vision
import play_vd12_mujoco as legacy
import vd13_mujoco_contract as backend
from vd12_visual_control import VD12VisualController
from vd5_mujoco_control import quaternion_from_euler_xyz
from vd5_visual_control import SCHEMA

evaluation = backend.evaluation
sys.path.insert(0, str(evaluation.ROOT / 'scripts/diagnostics'))
import play_vd12_checked as isaac


def fixture():
    profile = evaluation.geometry.geometry_profile(evaluation.URDF)
    mount = evaluation.mount.mount_profile()
    state = dict(version=1, startup_profile=copy.deepcopy(evaluation.STARTUP_PROFILE),
                 ground_transition_steps=20000, ground_blend=1., seam_trailing_distance_m=.95)
    checkpoint = dict(training_experiment_name=evaluation.EXPERIMENT, iter=6199,
                      vd13_mount_profile=copy.deepcopy(mount), vd12_geometry_profile=copy.deepcopy(profile),
                      vd5_curriculum_state=state)
    agent = dict(experiment_name=evaluation.EXPERIMENT, empirical_normalization=False,
                 clip_actions=None, policy=dict(activation='elu', actor_hidden_dims=[512, 256, 128]))
    saved = dict(vd13_mount_profile=copy.deepcopy(mount), vd12_geometry_profile=copy.deepcopy(profile),
                 scene=dict(robot=dict(spawn=dict(asset_path=str(evaluation.URDF)))),
                 commands=dict(ee_target=dict(vd5_seam_trailing_distance=.95,
                     class_type='b2_z1.tasks.manager_based.b2_z1.b2_z1_vd13_env_cfg:VD13SafeSeamCommand')))
    return checkpoint, agent, saved, profile


class VD13Entries(unittest.TestCase):
    def check_both(self, values):
        cp, agent, saved, profile = values
        backend.validate_checkpoint(cp, agent, saved, profile)
        isaac.validate_checkpoint(cp, agent, profile, saved, version=13)

    def reject_both(self, values):
        cp, agent, saved, profile = values
        with self.assertRaises(ValueError):
            backend.validate_checkpoint(cp, agent, saved, profile)
        with self.assertRaises(ValueError):
            isaac.validate_checkpoint(cp, agent, profile, saved, version=13)

    def test_valid_mount75_checkpoint_on_both_sides(self):
        self.check_both(fixture())

    def test_defaults_are_explicit_and_do_not_mutate_vd12(self):
        self.assertEqual(vision.parse([], version=13).checkpoint, evaluation.DEFAULT_CHECKPOINT)
        self.assertEqual(legacy.parse([], version=13).checkpoint, evaluation.DEFAULT_CHECKPOINT)
        self.assertEqual(vision.parse([]).checkpoint.name, 'model_5999.pt')
        self.assertEqual(legacy.parse([]).checkpoint.name, 'model_5999.pt')
        self.assertEqual(evaluation.DEFAULT_CHECKPOINT.name, 'model_6199.pt')
        self.assertEqual(backend.ROBOT_VERSION, 'vd13')
        self.assertEqual(legacy.ROBOT_VERSION, 'vd10')
        self.assertNotEqual(backend.URDF, legacy.URDF)
        np.testing.assert_allclose(backend.TCP_POSITION, [.825, 0., .1])
        self.assertEqual(backend.WORK_DISTANCE_M, .95)
        self.assertEqual(vision.parse([], version=13).vision_mode, 'rhanet')

    def test_old_actor_or_agent_rejected_even_with_copied_geometry(self):
        for index in (0, 1):
            for name in ('b2_z1_vd5_world_seam_amp', 'b2_z1_vd12_geometry_guard_amp', None):
                values = fixture()
                values[index]['training_experiment_name' if index == 0 else 'experiment_name'] = name
                with self.subTest(index=index, name=name):
                    self.reject_both(values)

    def test_missing_or_changed_mount_metadata_rejected(self):
        for index in (0, 2):
            for key in evaluation.mount.mount_profile():
                values = fixture()
                del values[index]['vd13_mount_profile'][key]
                with self.subTest(index=index, key=key):
                    self.reject_both(values)

    def test_matching_but_wrong_geometry_is_not_current_mount(self):
        values = fixture()
        for profile in (values[0]['vd12_geometry_profile'], values[2]['vd12_geometry_profile'], values[3]):
            profile['urdf_sha256'] = 'legacy'
        self.reject_both(values)

    def test_wrong_scene_command_or_work_distance_rejected(self):
        for field in ('scene', 'class', 'distance'):
            values = fixture()
            if field == 'scene':
                values[2]['scene']['robot']['spawn']['asset_path'] = str(legacy.URDF)
            elif field == 'class':
                values[2]['commands']['ee_target']['class_type'] = 'legacy:VD10SafeSeamCommand'
            else:
                values[2]['commands']['ee_target']['vd5_seam_trailing_distance'] = .8
            with self.subTest(field=field):
                self.reject_both(values)

    def test_incomplete_ground_or_changed_startup_rejected(self):
        for field, value in (('ground_blend', .9), ('ground_transition_steps', 1),
                             ('seam_trailing_distance_m', .8), ('version', 0)):
            values = fixture()
            values[0]['vd5_curriculum_state'][field] = value
            with self.subTest(field=field):
                self.reject_both(values)
        values = fixture()
        values[0]['vd5_curriculum_state']['startup_profile']['vd10']['tcp_in_link06_m'][0] = .75
        self.reject_both(values)

    def test_iteration_normalization_or_action_clip_rejected(self):
        for index, key, value in ((0, 'iter', True), (0, 'iter', -1),
                                   (0, 'obs_norm_state_dict', {}),
                                   (1, 'empirical_normalization', True), (1, 'clip_actions', 1.)):
            values = fixture()
            values[index][key] = value
            with self.subTest(key=key, value=value):
                self.reject_both(values)

    def test_real_saved_drive_settings_and_observation_order(self):
        saved = legacy.load_yaml(evaluation.DEFAULT_CHECKPOINT.parent / 'params/env.yaml')
        checked = backend.validate_saved_settings(saved)
        self.assertTrue(checked['clock_checked'])
        self.assertEqual(checked['work_distance_m'], .95)
        saved['scene']['robot']['actuators']['z1_motor1']['stiffness'] = 100.
        with self.assertRaises(ValueError):
            backend.validate_saved_settings(saved)

    def test_short_rear_support_is_not_silently_evaluated_at_old_distance(self):
        def stream(xs):
            return dict(valid=True, capture_generation=42, sequence=0, capture_timestamp=10.,
                        ground_plane_b=[0., 0., 1., .5],
                        points_b=np.c_[xs, np.zeros_like(xs), np.full_like(xs, -.5)].tolist())
        packet = dict(schema=SCHEMA, valid=True, capture_generation=42,
                      front=stream(np.linspace(.15, 2., 60)), rear=stream(np.linspace(-.87, -.3, 60)))
        def controller(distance):
            return VD12VisualController([-.95, 0., .35], quaternion_from_euler_xyz(0., .3, np.pi),
                                        trailing_distance=distance, max_occlusion_extension=.05)
        self.assertTrue(controller(.8).update(packet, 10., .02)['status']['valid'])
        result = controller(backend.WORK_DISTANCE_M).update(packet, 10., .02)
        self.assertFalse(result['status']['valid'])
        self.assertTrue(result['status']['failure_latched'])
        np.testing.assert_array_equal(result['base_command_b'], [0., 0., 0.])

    def test_wrappers_select_version_not_old_default(self):
        for path in (ROOT / 'scripts/deploy_vd13_vision.py', ROOT / 'scripts/play_vd13_mujoco_truth.py',
                     evaluation.ROOT / 'scripts/diagnostics/play_vd13_checked.py'):
            self.assertIn('main(version=13)', path.read_text())


if __name__ == '__main__':
    unittest.main()
