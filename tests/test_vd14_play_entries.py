"""VD14 inference identities, unchanged mount, and fail-closed visual defaults."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import deploy_vd12_vision as vision
import play_vd12_mujoco as replay
import vd13_mujoco_contract as old
import vd14_mujoco_contract as backend

evaluation = backend.evaluation
sys.path.insert(0, str(evaluation.ROOT / 'scripts/diagnostics'))
import play_vd12_checked as isaac


def fixture():
    saved = replay.load_yaml(evaluation.DEFAULT_CHECKPOINT.parent / 'params/env.yaml')
    agent = replay.load_yaml(evaluation.DEFAULT_CHECKPOINT.parent / 'params/agent.yaml')
    profile = evaluation.geometry.geometry_profile(evaluation.URDF)
    checkpoint = dict(training_experiment_name=evaluation.EXPERIMENT, iter=5999,
                      vd13_mount_profile=evaluation.mount.mount_profile(),
                      vd12_geometry_profile=deepcopy(profile),
                      vd14_posture_profile=evaluation.posture.posture_profile(),
                      vd5_curriculum_state=dict(version=1, startup_profile=deepcopy(evaluation.STARTUP_PROFILE),
                          ground_transition_steps=20000, ground_blend=1., seam_trailing_distance_m=.95))
    return checkpoint, agent, saved, profile


class VD14Entries(unittest.TestCase):
    def check_both(self, values):
        cp, agent, saved, profile = values
        backend.validate_checkpoint(cp, agent, saved, profile)
        isaac.validate_checkpoint(cp, agent, profile, saved, version=14)

    def reject_both(self, values):
        cp, agent, saved, profile = values
        with self.assertRaises(ValueError):
            backend.validate_checkpoint(cp, agent, saved, profile)
        with self.assertRaises(ValueError):
            isaac.validate_checkpoint(cp, agent, profile, saved, version=14)

    def test_valid_identity_on_both_sides_without_metadata_mutation(self):
        values = fixture()
        before = deepcopy(values)
        self.check_both(values)
        self.assertEqual(values, before)

    def test_final_checkpoint_defaults_and_physical_model_are_explicit(self):
        self.assertEqual(replay.parse([], version=14).checkpoint, evaluation.DEFAULT_CHECKPOINT)
        self.assertEqual(vision.parse([], version=14).checkpoint, evaluation.DEFAULT_CHECKPOINT)
        self.assertEqual(evaluation.DEFAULT_CHECKPOINT.name, 'model_5999.pt')
        self.assertEqual(backend.ROBOT_VERSION, 'vd13')
        self.assertEqual(backend.URDF, old.URDF)
        self.assertEqual(backend.TCP_POSITION, old.TCP_POSITION)
        self.assertEqual(backend.WORK_DISTANCE_M, .95)
        self.assertEqual(replay.parse([], version=13).checkpoint.name, 'model_6199.pt')
        self.assertEqual(vision.parse([], version=13).checkpoint, old.DEFAULT_CHECKPOINT)

    def test_visual_defaults_are_image_driven_and_protection_not_weakened(self):
        args = vision.parse([], version=14)
        self.assertEqual(args.vision_mode, 'rhanet')
        self.assertEqual(args.motion_compensation, 'leg_imu')
        self.assertEqual(vision.parse(['--motion-compensation','none'], version=14).motion_compensation, 'none')
        self.assertEqual(vision.parse([], version=13).motion_compensation, 'none')
        self.assertEqual(vision.parse([], version=12).motion_compensation, 'none')
        self.assertEqual(args.vision_max_age, 1.)
        self.assertEqual(args.rear_occlusion_extension, .05)
        self.assertEqual(args.rear_path_memory_s, .6)
        self.assertFalse(args.historical_tangent_estimate)

    def test_old_actor_agent_and_mount_rejected_even_when_profile_copied(self):
        for index, key in ((0, 'training_experiment_name'), (1, 'experiment_name')):
            for name in (old.evaluation.EXPERIMENT, 'b2_z1_vd5_world_seam_amp', None):
                values = fixture()
                values[index][key] = name
                with self.subTest(index=index, name=name):
                    self.reject_both(values)
        values = fixture()
        values[0]['vd13_mount_profile']['clamp_to_tip_m'] = .9
        self.reject_both(values)

    def test_missing_or_changed_posture_profile_rejected(self):
        for index in (0, 2):
            for key in evaluation.posture.posture_profile():
                values = fixture()
                values[index]['vd14_posture_profile'].pop(key)
                with self.subTest(index=index, key=key):
                    self.reject_both(values)

    def test_actual_saved_reward_weight_scale_function_and_body_order_checked(self):
        changes = [('base_tilt', 'weight', -3.), ('vd14_front_width', 'weight', 0.),
                   ('base_tilt', 'func', 'wrong:reward'), ('vd14_front_width', 'func', 'wrong:reward')]
        for term, key, value in changes:
            values = fixture()
            values[2]['rewards'][term][key] = value
            self.reject_both(values)
        for key, value in (('scale', .2), ('minimum_width', .2)):
            values = fixture()
            values[2]['rewards']['vd14_front_width']['params'][key] = value
            self.reject_both(values)
        for key, value in (('name', 'other_robot'), ('body_names', ['FR_foot', 'FL_foot']), ('preserve_order', False)):
            values = fixture()
            values[2]['rewards']['vd14_front_width']['params']['asset_cfg'][key] = value
            self.reject_both(values)

    def test_work_distance_unfinished_curriculum_or_geometry_rejected(self):
        for field, value in (('ground_blend', .99), ('seam_trailing_distance_m', .8), ('version', 2)):
            values = fixture()
            values[0]['vd5_curriculum_state'][field] = value
            self.reject_both(values)
        values = fixture()
        for p in (values[0]['vd12_geometry_profile'], values[2]['vd12_geometry_profile'], values[3]):
            p['urdf_sha256'] = 'wrong'
        self.reject_both(values)

    def test_drive_settings_and_actor_contract_retained(self):
        values = fixture()
        self.assertTrue(backend.validate_saved_settings(values[2])['clock_checked'])
        values[2]['scene']['robot']['actuators']['z1_motor1']['stiffness'] = 100.
        with self.assertRaises(ValueError):
            backend.validate_saved_settings(values[2])
        for key, value in (('empirical_normalization', True), ('clip_actions', 1.)):
            values = fixture()
            values[1][key] = value
            self.reject_both(values)

    def test_wrappers_select_vd14_not_old_defaults(self):
        for p in (ROOT / 'scripts/play_vd14_mujoco_truth.py', ROOT / 'scripts/deploy_vd14_vision.py',
                  evaluation.ROOT / 'scripts/diagnostics/play_vd14_checked.py'):
            self.assertIn('main(version=14)', p.read_text())


if __name__ == '__main__':
    unittest.main()
