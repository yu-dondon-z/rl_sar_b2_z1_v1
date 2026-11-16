import unittest
import json
from pathlib import Path
from camera_near import image_steering_in_range,required_stable_frames

class V24CameraTests(unittest.TestCase):
    def test_recovery_is_not_a_second_acquisition(self):
        for phase in [None,0,1,3]:self.assertEqual(required_stable_frames(phase),6)
        self.assertEqual(required_stable_frames(2),1)

    def test_display_window_accepts_moving_views_but_not_backwards(self):
        self.assertTrue(image_steering_in_range(.5,-.6))
        self.assertFalse(image_steering_in_range(.7,0.))
        self.assertFalse(image_steering_in_range(float('nan'),0.))

    def test_checkpoint_tool_contract_is_explicit(self):
        cfg=json.loads((Path(__file__).parent/'config/va15_training_contract.json').read_text())
        self.assertEqual(cfg['tool_height_m'],.09)
        self.assertEqual(cfg['limit_pos_z_m'],[-.55,-.30])
        self.assertEqual(cfg['tracked_body'],'dummy_target')
        self.assertEqual(cfg['command_frame'],'base_link, full base orientation')

if __name__=='__main__':unittest.main()
