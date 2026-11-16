import contextlib
import csv
import io
import json
from pathlib import Path
import tempfile
import unittest
from review_static import main, review_rows

TEST_COUNTS = [0,0,0,6,5,5,5,0,0,0,0,0]


def fixture():
    rows = []
    for state in (0, 1, 2, 3, 0):
        r = dict(low_callback_received_ns=1, contract_ready=1, contract_latched=0, bit_flag=4, fsm=state)
        r.update(contract_baseline_frozen=1,contract_stable_seconds=3.)
        for i in range(3):
            r[f'limited_{i}'] = 0
        for i in range(12):
            for key, value in (('lost', TEST_COUNTS[i]), ('motor_fault', 0), ('motor_frequency', 535), ('mode', 10),
                               ('contract_baseline',TEST_COUNTS[i])):
                r[f'{key}_dds_{i}'] = value
        rows.append(r)
    return rows, 0, 'B2+Z1 real deployment stopped COMMAND_EXIT_CODE="0"', dict(
        io_failed=False, commands_dropped=0, control_dropped=0, actor_dropped=0)


class StaticReviewTests(unittest.TestCase):
    def test_require_pass_exit_code_and_no_motion_authorization(self):
        for expect_pass in (True, False):
            with self.subTest(expect_pass=expect_pass), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                rows, count, terminal, integrity = fixture()
                if not expect_pass:
                    terminal = ''  # An incomplete recorder must NOT open the entry.
                with (root/'commands.csv').open('w', newline='') as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                    writer.writeheader()
                    writer.writerows(rows)
                (root/'actor.csv').write_text('sequence\n')
                (root/'terminal.log').write_text(terminal)
                (root/'telemetry_summary.json').write_text(json.dumps(integrity))
                output = io.StringIO()
                with contextlib.redirect_stdout(output):
                    result = main(['--require-pass', str(root)])
                self.assertEqual(result, 0 if expect_pass else 2)
                self.assertFalse(json.loads(output.getvalue())['motion_authorized'])

    def test_does_not_certify_or_authorize(self):
        result = review_rows(*fixture())
        self.assertTrue(result['software_static_review_pass'])
        self.assertFalse(result['hardware_qualified'])
        self.assertFalse(result['motion_authorized'])
        self.assertFalse(result['baseline_lost_semantics_verified'])

    def test_refuses_incomplete_fault_or_movement(self):
        for field, value in (('bit_flag', 5), ('contract_ready', 0), ('contract_latched', 1),
                             ('motor_fault_dds_5', 256), ('motor_frequency_dds_6', 0),
                             ('lost_dds_6', 0), ('mode_dds_5', 0), ('limited_0', .01),
                             ('contract_baseline_frozen',0),('contract_stable_seconds',2.9)):
            rows, count, terminal, integrity = fixture()
            for r in rows:
                r[field] = value
            self.assertFalse(review_rows(rows, count, terminal, integrity)['software_static_review_pass'])
        rows, count, terminal, integrity = fixture()
        self.assertFalse(review_rows(rows, 1, terminal, integrity)['software_static_review_pass'])
        self.assertFalse(review_rows(rows[:3], count, terminal, integrity)['software_static_review_pass'])
        self.assertFalse(review_rows(rows, count, '', integrity)['software_static_review_pass'])
        rows[1]['contract_baseline_dds_3'] += 1
        rows[1]['lost_dds_3'] += 1
        self.assertFalse(review_rows(rows, count, terminal, integrity)['software_static_review_pass'])
        integrity['commands_dropped'] = 1
        self.assertFalse(review_rows(rows, count, terminal, integrity)['software_static_review_pass'])


if __name__ == '__main__':
    unittest.main()
