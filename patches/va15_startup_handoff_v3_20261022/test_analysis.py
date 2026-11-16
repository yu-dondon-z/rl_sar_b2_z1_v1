#!/usr/bin/env python3
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from analyze_telemetry import analyze

class AnalysisTest(unittest.TestCase):
    def test_generated_records(self):
        root=Path(__file__).resolve().parent
        with tempfile.TemporaryDirectory(prefix='va15_analysis_test_') as tmp:
            subprocess.run([str(root/'build/test_telemetry'),tmp],check=True,stdout=subprocess.PIPE,text=True)
            result=analyze(tmp)
            self.assertEqual(result['command_rows'],200)
            self.assertEqual(result['actor_rows'],200)
            self.assertEqual(result['cached_action_frames'],0)
            self.assertEqual(result['dds_write_failures'],0)
            self.assertFalse(result['hardware_qualified'])
            self.assertFalse(result['dds_write_is_motor_ack'])
            self.assertAlmostEqual(result['joints'][0]['mean_signed_q_error_rad'],.1,places=6)
            self.assertAlmostEqual(result['joints'][0]['requested_pd_tau_nm']['median'],16.,places=4)
            self.assertAlmostEqual(result['source_age_at_dds_return_ms']['median'],.002,places=6)
            self.assertFalse(result['recording_integrity']['io_failed'])
            json.dumps(result,allow_nan=False)

if __name__=='__main__':unittest.main()
