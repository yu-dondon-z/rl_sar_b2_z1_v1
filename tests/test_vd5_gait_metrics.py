import sys
from pathlib import Path
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from vd5_gait_metrics import summarize, high_frequency_rms


class GaitMetricsTest(unittest.TestCase):
    def test_highpass_removes_drift_preserves_oscillation(self):
        t = np.arange(4000)*.005
        rms = high_frequency_rms(.2*t+.01*np.sin(2*np.pi*8*t), .005)
        self.assertAlmostEqual(rms, .01/np.sqrt(2), places=4)

    def test_touchdown_uses_precontact_speed_and_rejects_chatter(self):
        rows = []
        for k in range(100):
            force = np.zeros((4,3)); vel = np.zeros((4,3))
            if 40 <= k < 60 or k == 80:
                force[0,2] = 220.
            vel[0,2] = -.7 if k < 40 else 0.
            rows.append(dict(time_s=k*.005, foot_velocity_w=vel, foot_force_w=force,
                base_velocity_b=[.2,0,0], tilt_deg=0., base_position_w=[0,0,.5],
                tcp_position_w=[0,0,0], tcp_position_b=[0,0,0], tool_target_b=[0,0,0]))
        result = summarize(rows, [], settle=0)
        self.assertEqual(result['touchdowns']['FL']['count'], 1)
        self.assertEqual(result['touchdowns']['FL']['descending_speed_mps']['median'], .7)
        self.assertEqual(result['touchdowns']['FL']['peak_force_next40ms_n']['max'], 220.)
        self.assertEqual(result['touchdowns']['FR']['count'], 0)


if __name__ == '__main__':
    unittest.main()
