import hashlib
from pathlib import Path
import subprocess
import tempfile
import unittest

from check_static_evidence import verify_hashes, verify_reviewed_run, REVIEWED_RUN


class ForwardEntryTests(unittest.TestCase):
    def test_missing_changed_or_replaced_evidence_is_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            expected = {'one.txt': hashlib.sha256(b'tested').hexdigest()}
            with self.assertRaises(ValueError):
                verify_hashes(folder, expected)
            (folder/'one.txt').write_bytes(b'tested')
            verify_hashes(folder, expected)
            (folder/'one.txt').write_bytes(b'changed')
            with self.assertRaises(ValueError):
                verify_hashes(folder, expected)
            with self.assertRaises(ValueError):
                verify_reviewed_run(folder)

    def test_no_automatic_or_unattended_launch(self):
        script = Path(__file__).with_name('run_forward_once.sh')
        for args in ([], ['eth0'], ['eth0', '--run'], ['eth0', '--forward-test', '--anything'],
                     ['eth0', '--forward-test']):
            result = subprocess.run(['bash', str(script), *args], stdin=subprocess.DEVNULL,
                                    capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 2, result.stdout+result.stderr)
            self.assertNotIn('B2+Z1 real deployment ready', result.stdout)

    def test_real_reviewed_static_fixture_when_present(self):
        root = Path(__file__).resolve().parent
        candidates = (root/'board_results'/REVIEWED_RUN, root.parents[1]/'outputs'/REVIEWED_RUN)
        folder = next((path for path in candidates if path.is_dir()), None)
        if folder is None:
            self.skipTest('Reviewed fixture is not installed on this offline test host')
        result = verify_reviewed_run(folder)
        self.assertTrue(result['software_static_review_pass'])
        self.assertFalse(result['hardware_qualified'])
        self.assertFalse(result['motion_authorized'])


if __name__ == '__main__':
    unittest.main()
