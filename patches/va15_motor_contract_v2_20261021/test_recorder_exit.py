"""Offline PTY regression; uses only mock sleep/print processes, never SDKs."""
import errno
import os
from pathlib import Path
import pty
import select
import shlex
import signal
import subprocess
import sys
import tempfile
import time
import unittest


@unittest.skipUnless(hasattr(os, 'pidfd_open') and hasattr(signal, 'pidfd_send_signal'),
                     'Mock cleanup requires Linux pidfd support; use /usr/bin/python3 (3.10+)')
class RecorderExitTests(unittest.TestCase):
    def make_directory(self):
        directory = tempfile.TemporaryDirectory()
        # Register before process cleanup so failure paths stop mocks before
        # deleting their PID files (unittest cleanups execute last-in-first-out).
        self.addCleanup(directory.cleanup)
        return directory.name

    def launch_mock(self, directory, command, legacy_exec=False):
        root = Path(directory)
        master, slave = pty.openpty()
        # Same parent/child structure as ensure_local_z1_controller, but the
        # "service" is just sleep. No Unitree files or network are accessed.
        start = ('setsid sleep 30 </dev/null >'+shlex.quote(str(root/'mock_service.log'))+
                 ' 2>&1 &\nprintf "%s\\n" "$!" >'+shlex.quote(str(root/'mock_service.pid'))+'\n')
        recorder = ('script --quiet --flush --return --command '+shlex.quote(command)+' '+
                    shlex.quote(str(root/'terminal.log')))
        if legacy_exec:
            shell = start+'exec '+recorder
        else:
            # Production fix: preserve the original shell as service parent.
            shell = start+'if '+recorder+'; then exit 0; else exit "$?"; fi'
        process = subprocess.Popen(['bash', '-c', shell], stdin=slave, stdout=slave, stderr=slave,
                                   start_new_session=True, close_fds=True)
        os.close(slave)
        self.addCleanup(self.cleanup_mock, process, master, root)
        deadline = time.monotonic()+3
        while not (root/'mock_service.pid').exists() and time.monotonic()<deadline:
            time.sleep(.01)
        self.assertTrue((root/'mock_service.pid').exists())
        return process, master, root

    def cleanup_mock(self, process, master, root):
        # Only the Popen process and the explicitly created MOCK sleep are
        # candidates for signalling, not process groups or real services.
        if process.poll() is None:
            process.kill()
        process.wait(timeout=3)
        path = root/'mock_service.pid'
        if path.exists():
            pid = int(path.read_text())
            try:
                fd = os.pidfd_open(pid)
                try:
                    args = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
                    if args and Path(os.fsdecode(args[0])).name == 'sleep' and b'30' in args:
                        signal.pidfd_send_signal(fd, signal.SIGTERM)
                finally:
                    os.close(fd)
            except (ProcessLookupError, FileNotFoundError):
                pass
        os.close(master)

    def receive_until(self, master, token, seconds=3):
        data = bytearray()
        deadline = time.monotonic()+seconds
        while time.monotonic()<deadline:
            ready, _, _ = select.select([master], [], [], .05)
            if not ready:
                continue
            try:
                chunk = os.read(master, 65536)
            except OSError as exc:
                if exc.errno == errno.EIO:
                    break
                raise
            if not chunk:
                break
            data.extend(chunk)
            if token in data:
                return bytes(data)
        self.fail('Mock PTY did not produce expected marker: '+repr(data[-1000:]))

    def assert_service_alive(self, root):
        pid = int((root/'mock_service.pid').read_text())
        self.assertEqual(Path(f'/proc/{pid}/comm').read_text().strip(), 'sleep')
        os.kill(pid, 0)

    def test_foreground_recording_exits_without_stopping_background_service(self):
        process, master, root = self.launch_mock(self.make_directory(), 'printf "MOCK_NATIVE_DONE\\n"; exit 0')
        self.receive_until(master, b'MOCK_NATIVE_DONE')
        self.assertEqual(process.wait(timeout=3), 0)
        self.assert_service_alive(root)
        self.assertIn('COMMAND_EXIT_CODE="0"', (root/'terminal.log').read_text())

    def test_foreground_recording_preserves_nonzero_exit(self):
        process, master, root = self.launch_mock(self.make_directory(), 'printf "MOCK_NONZERO\\n"; exit 37')
        self.receive_until(master, b'MOCK_NONZERO')
        self.assertEqual(process.wait(timeout=3), 37)
        self.assert_service_alive(root)
        self.assertIn('COMMAND_EXIT_CODE="37"', (root/'terminal.log').read_text())

    def test_ctrl_c_reaches_only_recorded_mock_command(self):
        code = ('import signal,time,sys; signal.signal(signal.SIGINT,lambda s,f:sys.exit(0)); '
                'print("MOCK_READY",flush=True); time.sleep(20)')
        command = 'exec '+shlex.quote(sys.executable)+' -u -c '+shlex.quote(code)
        process, master, root = self.launch_mock(self.make_directory(), command)
        self.receive_until(master, b'MOCK_READY')
        os.write(master, b'\x03')
        self.assertEqual(process.wait(timeout=3), 0)
        self.assert_service_alive(root)

    def test_legacy_exec_hang_reproduced_on_util_linux_2372(self):
        version = subprocess.check_output(['script', '--version'], text=True)
        if '2.37.2' not in version:
            self.skipTest('Legacy reproduction requires the observed util-linux 2.37.2')
        directory = self.make_directory()
        done = Path(directory)/'mock_native.done'
        command = 'printf "MOCK_NATIVE_DONE\\n"; printf done >'+shlex.quote(str(done))+'; exit 0'
        process, master, root = self.launch_mock(directory, command, True)
        # The old waitpid loop can spin before it drains any PTY output.
        # Observe mock completion independently, not via the hung recorder.
        deadline = time.monotonic()+3
        while not done.exists() and time.monotonic()<deadline:
            time.sleep(.01)
        self.assertTrue(done.exists(), 'Mock command did not complete')
        self.assertEqual(done.read_text(), 'done')
        with self.assertRaises(subprocess.TimeoutExpired):
            process.wait(timeout=.5)
        self.assert_service_alive(root)

    def test_current_launcher_has_no_exec_script(self):
        source = Path(__file__).with_name('run_trial.sh').read_text()
        self.assertNotIn('exec script --quiet', source)
        self.assertIn('if script --quiet --flush --return', source)


if __name__ == '__main__':
    unittest.main()
