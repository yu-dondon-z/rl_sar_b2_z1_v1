"""Launcher/review regressions, no SDK, camera, hardware or real launcher."""
import csv
import hashlib
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
import review_shadow
from session import camera_live


class SessionTests(unittest.TestCase):
    def test_live_does_not_mean_valid(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)/'near.json'
            now = time.monotonic_ns()
            p.write_text(json.dumps(dict(schema='va15_near_visual_v1', session_id='a',
                                         sequence=1, capture_monotonic_ns=now, valid=False,image_received=True)))
            self.assertTrue(camera_live(p, 'a'))
            self.assertFalse(camera_live(p, 'b'))
            p.write_text('{}')
            self.assertFalse(camera_live(p, 'a'))

    def test_shadow_review(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d);(root/'native_build_visual').mkdir()
            bundle = root/'native_build_visual/bundle.json';bundle.write_text('{}')
            (root/'session.json').write_text(json.dumps(dict(mode='--shadow', created_unix_s=time.time(),
                bundle_sha256=hashlib.sha256(bundle.read_bytes()).hexdigest())))
            (root/'telemetry_summary.json').write_text(json.dumps(dict(io_failed=False,
                commands_dropped=0, control_dropped=0, actor_dropped=0)))
            (root/'actor.csv').write_text('model_executed\n')
            (root/'terminal.log').write_text('[Near visual] four-second monotonic visual request deadline\n')
            rows = []
            for i in range(203):
                stamp = 10_000_000_000+i*20_000_000
                row = dict(fsm=2, visual_stage=1, control_start_ns=stamp,
                    visual_capture_ns=stamp-150_000_000, visual_qualified=6,
                    visual_shadow=1, visual_would_x=.15, visual_would_yaw=.02,
                    estimate_valid=1,h_lower_m=.51,v_upper_m_s=.002,safety_fault=0,
                    limited_0=0, limited_1=0, limited_2=0,max_control_interval_ns=5_000_000)
                rows.append(row)
            rows[0].update(fsm=0, visual_stage=0)
            rows[1].update(fsm=1, visual_stage=0)
            rows[-2].update(fsm=3, visual_stage=2)
            rows[-1].update(fsm=0, visual_stage=2)

            def save():
                with (root/'control.csv').open('w') as f:
                    writer = csv.DictWriter(f, rows[0].keys());writer.writeheader();writer.writerows(rows)
                writes = [dict(visual_stage=r['visual_stage'],dds_write_return=1,contract_ready=1,contract_latched=0,
                    publish_end_ns=r['control_start_ns'],low_callback_received_ns=r['control_start_ns']-1_000_000,
                    max_publish_interval_ns=2_000_000) for r in rows]
                with (root/'commands.csv').open('w') as f:
                    writer = csv.DictWriter(f,writes[0].keys());writer.writeheader();writer.writerows(writes)

            save()
            with patch.object(review_shadow, 'ROOT', root):
                self.assertTrue(review_shadow.review(root)['pass_shadow'])
                rows[100]['limited_0'] = .1;save()
                with self.assertRaises(RuntimeError):review_shadow.review(root)
                rows[100]['limited_0'] = 0;rows[100]['v_upper_m_s'] = .2;save()
                with self.assertRaises(RuntimeError):review_shadow.review(root)
                rows[100]['v_upper_m_s'] = .002;rows[100]['max_control_interval_ns'] = 30_000_000;save()
                with self.assertRaises(RuntimeError):review_shadow.review(root)


if __name__ == '__main__':
    unittest.main()
