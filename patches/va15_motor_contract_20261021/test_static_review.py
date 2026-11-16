import unittest
from review_static import review_rows, EXPECTED_LOST


def fixture():
    rows = []
    for state in (0, 1, 2, 3, 0):
        r = dict(low_callback_received_ns=1, contract_ready=1, contract_latched=0, bit_flag=4, fsm=state)
        for i in range(3):
            r[f'limited_{i}'] = 0
        for i in range(12):
            for key, value in (('lost', EXPECTED_LOST[i]), ('motor_fault', 0), ('motor_frequency', 535), ('mode', 10)):
                r[f'{key}_dds_{i}'] = value
        rows.append(r)
    return rows, 0, 'B2+Z1 real deployment stopped COMMAND_EXIT_CODE="0"', dict(
        io_failed=False, commands_dropped=0, control_dropped=0, actor_dropped=0)


class StaticReviewTests(unittest.TestCase):
    def test_does_not_certify_or_authorize(self):
        result = review_rows(*fixture())
        self.assertTrue(result['software_static_review_pass'])
        self.assertFalse(result['hardware_qualified'])
        self.assertFalse(result['motion_authorized'])
        self.assertFalse(result['baseline_lost_semantics_verified'])

    def test_refuses_incomplete_fault_or_movement(self):
        for field, value in (('bit_flag', 5), ('contract_ready', 0), ('contract_latched', 1),
                             ('motor_fault_dds_5', 256), ('motor_frequency_dds_6', 0),
                             ('lost_dds_6', 0), ('mode_dds_5', 0), ('limited_0', .01)):
            rows, count, terminal, integrity = fixture()
            for r in rows:
                r[field] = value
            self.assertFalse(review_rows(rows, count, terminal, integrity)['software_static_review_pass'])
        rows, count, terminal, integrity = fixture()
        self.assertFalse(review_rows(rows, 1, terminal, integrity)['software_static_review_pass'])
        self.assertFalse(review_rows(rows[:3], count, terminal, integrity)['software_static_review_pass'])
        self.assertFalse(review_rows(rows, count, '', integrity)['software_static_review_pass'])
        integrity['commands_dropped'] = 1
        self.assertFalse(review_rows(rows, count, terminal, integrity)['software_static_review_pass'])


if __name__ == '__main__':
    unittest.main()
