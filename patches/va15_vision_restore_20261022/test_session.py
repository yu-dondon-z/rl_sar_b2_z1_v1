"""Offline orchestration checks; never imports a camera/robot SDK."""
import contextlib
import io
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock

from session import camera_command, camera_ready, control_command, main, stop_camera


class ObserverSessionTests(unittest.TestCase):
    def test_fresh_prone_invalid_is_camera_health_not_motion_permission(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'status.json'
            path.write_text(json.dumps(dict(valid=False, source_sequence=1, capture_monotonic_ns=100)))
            self.assertTrue(camera_ready(path, 200))
            self.assertFalse(camera_ready(path, 3_000_000_100))
            self.assertFalse(camera_ready(path, 99))

    def test_missing_corrupt_and_wrong_type_status_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'status.json'
            self.assertFalse(camera_ready(path, 100))
            for content in ('{', '{}', 'null', '[]', '{"capture_monotonic_ns":true,"source_sequence":1}',
                            '{"capture_monotonic_ns":100,"source_sequence":0}'):
                path.write_text(content)
                self.assertFalse(camera_ready(path, 100))

    def test_camera_always_preview_only_and_separate_exchange(self):
        for headless in (True, False):
            args = camera_command('eth0', '/isolated/session/exchange', '/sdk', '/rha', '/model', headless)
            self.assertIn('--preview-only', args)
            self.assertEqual(args[args.index('--exchange-dir')+1], '/isolated/session/exchange')
            self.assertEqual('--display' in args, not headless)
            self.assertNotIn('--vision', args)

    def test_only_existing_attended_modes_delegated(self):
        for mode in ('--standby', '--forward-test'):
            command = control_command(mode)
            self.assertEqual(command[-1], mode)
            self.assertTrue(command[1].endswith('va15_motor_contract_v2_20261021/run_trial.sh'))
            self.assertNotIn('--vision', command)
        for mode in ('--run', '--vision', '--preview', '--offline-check'):
            with self.assertRaises(ValueError):
                control_command(mode)

    def test_unsupported_follow_argument_cannot_launch(self):
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as result:
                main(['eth0', '--run'])
        self.assertEqual(result.exception.code, 2)

    def test_noninteractive_control_is_refused_before_any_subprocess(self):
        with mock.patch('session.Path.is_dir', return_value=True), \
             mock.patch.dict('session.os.environ', {'DISPLAY': ':test'}), \
             mock.patch('session.sys.stdin.isatty', return_value=False), \
             mock.patch('session.subprocess.run') as run:
            with self.assertRaisesRegex(RuntimeError, 'attended board terminal'):
                main(['eth0', '--standby'])
            run.assert_not_called()

    def test_camera_cleanup_does_not_signal_other_process(self):
        camera = subprocess.Popen(['sleep', '20'], start_new_session=True)
        other = subprocess.Popen(['sleep', '20'], start_new_session=True)
        try:
            stop_camera(camera)
            self.assertIsNotNone(camera.poll())
            self.assertIsNone(other.poll())
        finally:
            stop_camera(camera)
            stop_camera(other)


if __name__ == '__main__':
    unittest.main()
