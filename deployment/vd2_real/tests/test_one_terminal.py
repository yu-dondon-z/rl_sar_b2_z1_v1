"""Launcher lifecycle tests use temporary fake processes and loopback UDP only."""
import contextlib
import importlib.util
import io
import json
import os
import pty
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
import yaml

SOURCE=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('launch_real_test',SOURCE/'tools/launch_real.py')
launch=importlib.util.module_from_spec(spec);spec.loader.exec_module(launch)

CONTROLLER='''import json,signal,socket,sys,time
from pathlib import Path
root=Path.cwd().parent
cfg=json.loads((root/'fixture.json').read_text())
if cfg.get('controller_fail'):sys.exit(9)
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock.bind(('127.0.0.1',cfg['controller_port']));sock.settimeout(.1)
def stop(signum,frame):
    with (root/'exit_order.txt').open('a') as handle:handle.write('z1_stop\\n')
    sys.exit(0)
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
print('fake controller ready',flush=True)
while True:
    try:
        payload,peer=sock.recvfrom(128);sock.sendto(payload,peer)
    except socket.timeout:pass
'''

MAIN='''import json,signal,socket,sys,time
from pathlib import Path
root=Path.cwd();cfg=json.loads((root/'fixture.json').read_text())
def stop(signum,frame):
    with (root/'exit_order.txt').open('a') as handle:handle.write('main_stop\\n')
    print('fake main closed',flush=True);sys.exit(0)
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
    sock.bind(('127.0.0.1',cfg['client_port']));sock.settimeout(2)
    sock.sendto(b'loopback-test',('127.0.0.1',cfg['controller_port']))
    assert sock.recvfrom(128)[0]==b'loopback-test'
(root/'main_started.json').write_text(json.dumps(sys.argv))
print('fake main ready',flush=True)
time.sleep(60 if cfg.get('wait_for_signal') else .12)
with (root/'exit_order.txt').open('a') as handle:handle.write('main_done\\n')
sys.exit(cfg.get('main_exit',0))
'''


class OneTerminalTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='vd2_launcher_test_')
        self.root=Path(self.temp.name);self.z1=self.root/'controller';self.z1.mkdir()
        holders=[socket.socket(socket.AF_INET,socket.SOCK_DGRAM) for _ in range(2)]
        for sock in holders:sock.bind(('127.0.0.1',0))
        self.server_port,self.client_port=[sock.getsockname()[1] for sock in holders]
        for sock in holders:sock.close()
        self.fixture=dict(controller_port=self.server_port,client_port=self.client_port)
        self.save_fixture()
        (self.root/'config').mkdir();(self.root/'tools').mkdir()
        (self.root/'config/deployment.yaml').write_text(yaml.safe_dump(dict(
            z1_controller_ip='127.0.0.1',z1_to_port=self.server_port,z1_own_port=self.client_port)))
        (self.z1/'z1_ctrl').write_text('#!'+sys.executable+'\n'+CONTROLLER)
        (self.z1/'z1_ctrl').chmod(0o755)
        (self.root/'run.py').write_text(MAIN)
        shutil.copy2(SOURCE/'tools/launch_real.py',self.root/'tools/launch_real.py')

    def tearDown(self):self.temp.cleanup()

    def save_fixture(self):
        (self.root/'fixture.json').write_text(json.dumps(self.fixture))

    def run_launcher(self,existing=(),capture=False):
        with patch.object(launch,'controller_pids',return_value=list(existing)),contextlib.redirect_stdout(io.StringIO()):
            return launch.supervise(self.root,self.z1,'--preflight',interface='test0',seconds=.1,startup_timeout=2,
                                    capture_z1_network=capture)

    def test_starts_in_controller_directory_and_closes_main_before_owned_controller(self):
        self.assertEqual(self.run_launcher(),0)
        self.assertEqual((self.root/'exit_order.txt').read_text().splitlines(),['main_done','z1_stop'])
        args=json.loads((self.root/'main_started.json').read_text())
        self.assertIn('--preflight',args);self.assertIn('test0',args)
        directory=next((self.root/'runs').iterdir())
        self.assertIn('fake controller ready',(directory/'z1.log').read_text())
        self.assertIn('fake main ready',(directory/'main.log').read_text())
        self.assertTrue(launch.port_available(self.server_port))

    def test_reuses_existing_controller_without_stopping_it(self):
        existing=subprocess.Popen([str(self.z1/'z1_ctrl')],cwd=str(self.z1),stdout=subprocess.DEVNULL)
        try:
            deadline=time.monotonic()+3
            while launch.port_available(self.server_port) and time.monotonic()<deadline:time.sleep(.02)
            self.assertIsNone(existing.poll());self.assertFalse(launch.port_available(self.server_port))
            self.assertEqual(self.run_launcher([existing.pid]),0)
            self.assertIsNone(existing.poll())
            self.assertEqual((self.root/'exit_order.txt').read_text().splitlines(),['main_done'])
        finally:
            existing.send_signal(signal.SIGINT);existing.wait(timeout=3)

    def test_busy_sdk_port_is_reported_before_spawning_children(self):
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as occupied:
            occupied.bind(('127.0.0.1',self.client_port))
            with self.assertRaisesRegex(RuntimeError,'SDK client'):self.run_launcher()
        self.assertFalse((self.root/'main_started.json').exists())

    def test_main_is_not_started_when_controller_exits_during_startup(self):
        self.fixture['controller_fail']=True;self.save_fixture()
        with self.assertRaisesRegex(RuntimeError,'exited before startup'):self.run_launcher()
        self.assertFalse((self.root/'main_started.json').exists())

    def test_main_failure_preserves_exit_code_and_stops_owned_controller(self):
        self.fixture['main_exit']=7;self.save_fixture()
        self.assertEqual(self.run_launcher(),7)
        self.assertTrue(launch.port_available(self.server_port))

    def test_capture_failure_starts_neither_controller_nor_main(self):
        command=[sys.executable,'-c','import sys;print("permission denied",flush=True);sys.exit(1)']
        with patch.object(launch,'capture_command',return_value=command):
            with self.assertRaisesRegex(RuntimeError,'Controllers were not started'):
                self.run_launcher(capture=True)
        directory=next((self.root/'runs').iterdir())
        self.assertFalse((directory/'z1.log').exists())
        self.assertFalse((self.root/'main_started.json').exists())

    def test_capture_is_passive_and_closed_after_main_and_controller(self):
        code='''import signal,sys,time
from pathlib import Path
def stop(signum,frame):
    with Path('exit_order.txt').open('a') as f:f.write('capture_stop\\n')
    sys.exit(0)
signal.signal(signal.SIGINT,stop)
print('listening on fake loopback',flush=True)
time.sleep(30)
'''
        with patch.object(launch,'capture_command',return_value=[sys.executable,'-c',code]):
            self.assertEqual(self.run_launcher(capture=True),0)
        self.assertEqual((self.root/'exit_order.txt').read_text().splitlines(),
                         ['main_done','z1_stop','capture_stop'])
        directory=next((self.root/'runs').iterdir())
        self.assertEqual(json.loads((directory/'launcher.json').read_text())['capture_returncode'],0)

    def test_only_packet_capture_receives_sudo_and_exact_arm_filter(self):
        with patch.object(launch.socket,'if_nametoindex',return_value=2),\
             patch.object(launch.os,'geteuid',return_value=1000),\
             patch.object(launch.shutil,'which',side_effect=lambda name:'/usr/bin/'+name):
            command=launch.capture_command('eth0',self.root)
        self.assertEqual(command[:4],['/usr/bin/sudo','-n','--','/usr/bin/tcpdump'])
        self.assertIn('-Z',command)
        self.assertEqual(command[-1],'udp and host 192.168.123.110 and port 8881')
        self.assertEqual(command[command.index('-w')+1],str(self.root/'z1_arm.pcap'))

    def test_ctrl_c_reaches_main_before_owned_controller_is_stopped(self):
        self.fixture['wait_for_signal']=True;self.save_fixture()
        supervisor=subprocess.Popen([sys.executable,str(self.root/'tools/launch_real.py'),
            '--preflight','--z1-dir',str(self.z1)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        try:
            deadline=time.monotonic()+5
            while not (self.root/'main_started.json').exists() and time.monotonic()<deadline:
                if supervisor.poll() is not None:break
                time.sleep(.02)
            if not (self.root/'main_started.json').exists():
                output=supervisor.communicate(timeout=3)[0]
                self.fail('fake main did not start: '+output)
            supervisor.send_signal(signal.SIGINT)
            output=supervisor.communicate(timeout=8)[0]
            self.assertEqual(supervisor.returncode,130,output)
            self.assertEqual((self.root/'exit_order.txt').read_text().splitlines(),['main_stop','z1_stop'])
            self.assertTrue(launch.port_available(self.server_port))
        finally:
            if supervisor.poll() is None:
                supervisor.send_signal(signal.SIGTERM);supervisor.communicate(timeout=8)


class TimeoutConsoleTests(unittest.TestCase):
    def test_receive_timeouts_are_coalesced_but_disconnect_and_fault_are_immediate(self):
        lines=[];console=launch.TimeoutConsole('main',lines.append)
        warning='[WARNING] UDPPort::recv, unblock version, wait time out\n'
        console.feed(warning,0.)
        for step in range(1,50):console.feed(warning,step*.1)
        self.assertEqual(len(lines),1)
        console.feed('native IO fault 4\n',5.)
        self.assertIn('repeated 49 times',lines[-2]);self.assertIn('native IO fault 4',lines[-1])
        console.feed('[WARNING] Lose connection with z1_sdk\n',5.01)
        self.assertIn('Lose connection',lines[-1])
        console.feed(warning,5.1);console.flush(5.2,force=True)
        self.assertIn('repeated 1 times',lines[-1])

    def test_actual_child_log_keeps_every_suppressed_line(self):
        warning='[WARNING] UDPPort::recv, unblock version, wait time out'
        with tempfile.TemporaryDirectory() as directory,contextlib.redirect_stdout(io.StringIO()) as console:
            root=Path(directory)
            child=launch.LoggedProcess('main',[sys.executable,'-c',
                'print(("'+warning+'\\n")*30,end="");print("fatal example")'],root,root)
            child.process.wait(timeout=3);child.thread.join(timeout=3)
            raw=(root/'main.log').read_text()
            self.assertEqual(raw.count(warning),30);self.assertIn('fatal example',raw)
            events=[json.loads(line) for line in (root/'main_events.jsonl').read_text().splitlines()]
            self.assertEqual(''.join(row['line'] for row in events),raw)
            self.assertEqual([row['monotonic'] for row in events],sorted(row['monotonic'] for row in events))
            self.assertTrue(all(row['timestamp_basis']=='launcher_host_pipe_receipt_not_motor_capture'
                                for row in events))
        self.assertEqual(console.getvalue().count(warning),2)
        self.assertIn('repeated 29 times',console.getvalue());self.assertIn('fatal example',console.getvalue())


class CaptureSessionTests(unittest.TestCase):
    def test_capture_keeps_sudo_terminal_session_and_owns_separate_process_group(self):
        # A real controlling PTY reproduces the identity used by sudo's tty
        # cache. No sudo, packet capture or robot process is executed here.
        fixture='''import fcntl,importlib.util,json,os,sys,termios
from pathlib import Path
fcntl.ioctl(0,termios.TIOCSCTTY,0)
spec=importlib.util.spec_from_file_location('launch',sys.argv[1])
launch=importlib.util.module_from_spec(spec);spec.loader.exec_module(launch)
code="import json,os;fd=os.open('/dev/tty',os.O_RDONLY);print(json.dumps(dict(sid=os.getsid(0),pgid=os.getpgrp(),pid=os.getpid(),tty=True)));os.close(fd)"
root=Path(sys.argv[2])
child=launch.LoggedProcess('capture',[sys.executable,'-c',code],root,root,preserve_terminal_session=True)
child.process.wait(timeout=3);child.thread.join(timeout=3)
assert child.process.returncode==0
row=json.loads((root/'capture.log').read_text())
assert row['sid']==os.getsid(0),row
assert row['pgid']==row['pid'] and row['pgid']!=os.getpgrp(),row
assert row['tty']
'''
        with tempfile.TemporaryDirectory() as directory:
            master,slave=pty.openpty()
            try:
                result=subprocess.run([sys.executable,'-c',fixture,str(SOURCE/'tools/launch_real.py'),directory],
                    stdin=slave,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,
                    start_new_session=True,timeout=8)
                self.assertEqual(result.returncode,0,result.stdout)
            finally:os.close(slave);os.close(master)


if __name__=='__main__':unittest.main()
