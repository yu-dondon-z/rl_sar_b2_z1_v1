"""Output-only diagnostic metrics and launcher failure propagation."""
import gzip
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from debug_vd3_rear_vision import main, steady_evaluation


class VD3DebugSummaryTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='vd3_summary_', dir='/dev/shm')
        self.addCleanup(self.directory.cleanup)
        self.session = Path(self.directory.name)

    def write_truth(self, times):
        with gzip.open(self.session / 'evaluation_truth.jsonl.gz', 'wt') as stream:
            for time in times:
                stream.write(json.dumps(dict(sim_time_s=time, base_position_w=[.2*time, 0., .5],
                    tcp_position_w=[.2*time-1., 0., .2], base_yaw_rad=0.,
                    tcp_projection_error_m=.3 if time < 5. else .01,
                    foot_position_w=[[0., 0., .04]]*4,
                    foot_net_force_w=[[0., 0., f] for f in (180., 190., 200., 210.)])) + '\n')

    def test_startup_errors_are_excluded_and_horizontal_speed_is_measured(self):
        self.write_truth((0., 1., 2., 3., 4., 5., 6., 7.))
        result = steady_evaluation(self.session)
        self.assertTrue(result['evaluation_only'])
        self.assertEqual(result['sample_count'], 3)
        self.assertAlmostEqual(result['actual_forward_horizontal_mean_mps'], .2)
        self.assertAlmostEqual(result['tcp_ground_projection_error_max_mm'], 10.)
        self.assertEqual(result['tcp_ground_projection_over_50mm_fraction'], 0.)
        self.assertEqual(result['base_height_width_mm'], 0.)
        self.assertEqual(result['foot_vertical_force_mean_n']['FR'], 190.)

    def test_short_run_does_not_claim_steady_precision(self):
        self.write_truth((0., 1., 2.))
        result = steady_evaluation(self.session)
        self.assertFalse(result['available'])
        self.assertNotIn('tcp_ground_projection_error_max_mm', result)

    def test_startup_failure_exit_code_is_preserved(self):
        with patch('debug_vd3_rear_vision.launch', return_value=1):
            result = main(['--output-root', str(self.session), '--max-steps', '10'])
        self.assertEqual(result, 1)


if __name__ == '__main__':
    unittest.main()
