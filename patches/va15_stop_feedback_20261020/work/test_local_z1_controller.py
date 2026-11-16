"""Launcher tests with fake processes/ports; never imports a robot SDK."""
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import unittest

SOURCE=Path(__file__).resolve().parent

class LauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='va15_z1_launcher_test_')
        self.root=Path(self.temp.name);self.bin=self.root/'bin';self.bin.mkdir()
        self.repo=self.root/'repo';(self.repo/'scripts').mkdir(parents=True)
        self.ctrl=self.root/'controller';(self.ctrl/'build').mkdir(parents=True)
        self.pid=self.root/'test.pid'
        self.env=dict(os.environ,PATH=str(self.bin)+':'+os.environ['PATH'],
                      RL_SAR_Z1_CONTROLLER_ROOT=str(self.ctrl),Z1_TEST_PID=str(self.pid))
        self.executable(self.bin/'pgrep','#!/bin/bash\nif test -s "$Z1_TEST_PID"; then /bin/cat "$Z1_TEST_PID"; else exit 1; fi\n')
        self.executable(self.bin/'ss','#!/bin/bash\nif test -s "$Z1_TEST_PID"; then printf \'UNCONN users:(("z1_ctrl",pid=%s,fd=5))\\n\' "$(/usr/bin/head -n 1 "$Z1_TEST_PID")"; fi\n')
        self.executable(self.ctrl/'build/z1_ctrl','#!/bin/bash\nprintf "%s\\n" "$$" > "$Z1_TEST_PID"\nexec sleep 60\n')
    @staticmethod
    def executable(path,text):
        path.write_text(text);path.chmod(0o755)
    def tearDown(self):
        if self.pid.exists():
            for line in self.pid.read_text().splitlines():
                try:
                    pid=int(line)
                    cmdline=Path(f'/proc/{pid}/cmdline').read_bytes()
                    if pid!=os.getpid() and (b'sleep\x0060' in cmdline or
                            os.fsencode(str(self.ctrl/'build/z1_ctrl')) in cmdline):
                        os.kill(pid,signal.SIGTERM)
                except (ValueError,FileNotFoundError,ProcessLookupError):pass
        self.temp.cleanup()
    def helper(self,*args):
        return subprocess.run(['bash','-c','source "$1"; ensure_local_z1_controller "$2" "${@:3}"',
            'test',str(SOURCE/'local_z1_controller.sh'),str(self.repo),*args],
            env=self.env,capture_output=True,text=True,timeout=8)
    def test_disabled(self):
        self.assertEqual(self.helper('--disable-z1').returncode,0)
        self.assertFalse(self.pid.exists())
    def test_explicit_external_endpoint(self):
        self.assertEqual(self.helper('--z1-ip','10.10.1.5').returncode,0)
        self.assertFalse(self.pid.exists())
    def test_wrong_arm_endpoint(self):
        self.assertEqual(self.helper('--z1-ip','192.168.123.110').returncode,2)
    def test_invalid_port(self):
        for value in ('0','65536','80+71','999999'):
            self.assertEqual(self.helper('--z1-to-port',value).returncode,2)
    def test_start_then_reuse(self):
        first=self.helper();self.assertEqual(first.returncode,0,first.stderr)
        pid=self.pid.read_text();self.assertIn('Started local',first.stdout)
        second=self.helper();self.assertEqual(second.returncode,0,second.stderr)
        self.assertIn('Reusing',second.stdout);self.assertEqual(pid,self.pid.read_text())
    def test_new_controller_has_separate_terminal_group(self):
        first=self.helper();self.assertEqual(first.returncode,0,first.stderr)
        pid=int(self.pid.read_text())
        self.assertNotEqual(os.getpgid(pid),os.getpgrp())
        self.assertEqual(os.getsid(pid),pid)
    def test_terminal_sigint_does_not_kill_controller(self):
        # Like the real z1_ctrl, install a SIGINT handler instead of inheriting
        # the shell's ignored background SIGINT disposition.
        self.executable(self.ctrl/'build/z1_ctrl',
            '#!/bin/bash\ntrap "exit 130" INT\n'
            'printf "%s\\n" "$$" > "$Z1_TEST_PID"\n'
            'while true; do sleep .1; done\n')
        command=['bash','-c',
            'source "$1"; ensure_local_z1_controller "$2" || exit; echo READY; '
            'trap "exit 130" INT; while true; do sleep .1; done',
            'test',str(SOURCE/'local_z1_controller.sh'),str(self.repo)]
        process=subprocess.Popen(command,env=self.env,stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,text=True,start_new_session=True)
        try:
            while True:
                line=process.stdout.readline()
                if line.strip()=='READY':break
                if not line: self.fail('Mock launcher exited before READY')
            pid=int(self.pid.read_text())
            self.assertNotEqual(os.getpgid(pid),process.pid)
            os.killpg(process.pid,signal.SIGINT)
            process.wait(timeout=3)
            self.assertEqual(os.kill(pid,0),None)
            self.assertNotEqual(Path(f'/proc/{pid}/stat').read_text().split(') ',1)[1][0],'Z')
        finally:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=3)
            process.stdout.close();process.stderr.close()
    def test_reuse_existing_process(self):
        self.pid.write_text(str(os.getpid())+'\n')
        out=self.helper();self.assertEqual(out.returncode,0,out.stderr)
        self.assertIn('Reusing',out.stdout)
    def test_multiple_controllers_rejected(self):
        self.pid.write_text(f'{os.getpid()}\n{os.getpid()}\n')
        self.assertEqual(self.helper().returncode,1)
    def test_missing_executable(self):
        (self.ctrl/'build/z1_ctrl').unlink()
        self.assertEqual(self.helper().returncode,1)
    def test_port_owned_by_other_process_rejected(self):
        self.pid.write_text(str(os.getpid())+'\n')
        self.executable(self.bin/'ss','#!/bin/bash\necho \'UNCONN users:(("other",pid=999999,fd=5))\'\n')
        self.assertEqual(self.helper().returncode,1)
    def wrapper(self,*args,input=''):
        shutil.copyfile(SOURCE/'run_b2_z1_real.sh',self.repo/'scripts/run_b2_z1_real.sh')
        shutil.copyfile(SOURCE/'local_z1_controller.sh',self.repo/'scripts/local_z1_controller.sh')
        (self.repo/'scripts/check_va15_legacy.py').write_text('print("mock contract check")\n')
        binary=self.repo/'cmake_build_va15/bin/rl_real_b2_z1_no_gun';binary.parent.mkdir(parents=True,exist_ok=True)
        self.executable(binary,'#!/bin/bash\nprintf "MOCK_BINARY %s\\n" "$*"\n')
        self.executable(self.bin/'ip','#!/bin/bash\necho "1: lo inet 127.0.0.1/8"\n')
        return subprocess.run(['bash',str(self.repo/'scripts/run_b2_z1_real.sh'),*args],
            input=input,env=self.env,capture_output=True,text=True,timeout=8)
    def test_offline_never_starts_controller(self):
        self.assertEqual(self.wrapper('--offline-check').returncode,0)
        self.assertFalse(self.pid.exists())
    def test_cancel_run_never_starts_controller(self):
        self.assertEqual(self.wrapper('lo','--run',input='cancel\n').returncode,1)
        self.assertFalse(self.pid.exists())
    def test_disabled_preflight_never_starts_controller(self):
        out=self.wrapper('lo','--preflight','--disable-z1')
        self.assertEqual(out.returncode,0,out.stderr);self.assertFalse(self.pid.exists())
    def test_confirmed_run_starts_controller_before_binary(self):
        out=self.wrapper('lo','--run',input='RUN\n')
        self.assertEqual(out.returncode,0,out.stderr);self.assertTrue(self.pid.exists())
        self.assertLess(out.stdout.index('Local Z1 controller ready'),out.stdout.index('MOCK_BINARY lo'))

if __name__=='__main__':unittest.main()
