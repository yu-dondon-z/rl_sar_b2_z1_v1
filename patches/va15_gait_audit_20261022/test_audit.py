"""Check conclusions against frozen offline evidence, not hardware qualification."""
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parent


class GaitAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = json.loads((ROOT/'phase_audit.json').read_text())

    def test_analysis_does_not_change_or_qualify_robot(self):
        self.assertFalse(self.report['real_control_modified'])
        self.assertFalse(self.report['hardware_qualified'])

    def test_actual_actor_history_ema_and_scaling_replay(self):
        c = self.report['action_chain']
        self.assertEqual(c['actor_rows'], 244)
        self.assertLess(c['original_checkpoint_vs_native_raw_max_abs'], 1e-5)
        self.assertLess(c['previous_bounded_raw_observation_max_abs'], 1e-6)
        self.assertLess(c['ema_replay_max_abs'], 1e-6)
        self.assertLess(c['scaled_target_formula_max_abs_rad'], 1e-6)
        self.assertEqual(sum(c['raw_clip_hits_per_joint']), 0)
        self.assertEqual(sum(c['additional_filtered_clip_hits_per_joint']), 0)

    def test_transitional_samples_are_not_called_steady_gait(self):
        phases = self.report['phase_summaries']
        names = ['leg_handoff', 'legs_released_arm_handoff',
                 'all_handoffs_released_not_steady_state_qualification']
        self.assertEqual(sum(phases[n]['rows'] for n in names), 189)
        final = phases[names[-1]]
        self.assertGreater(final['start_s'], 2.2)
        self.assertLess(final['end_s']-final['start_s'], 1.8)

    def test_gain_jump_and_counterfactual_are_explicit(self):
        for r in self.report['startup_pd_transition'].values():
            self.assertEqual(r['old_kp'], 675)
            self.assertEqual(r['new_kp'], 160)
            self.assertGreater(r['requested_pd_ratio_at_same_state'], .23)
            self.assertLess(r['requested_pd_ratio_at_same_state'], .25)
            self.assertLess(r['counterfactual_pd_continuity_error_nm'], 1e-10)


if __name__ == '__main__':
    unittest.main()
