#!/usr/bin/env python3
"""Run the existing Z1 controller and VD2 entry in one terminal.

This supervisor never acquires the robot or sends commands. run.py retains the
gamepad-driven takeover. Only a controller started by this invocation is stopped.
"""
import argparse
from datetime import datetime
import fcntl
import json
import os
import pwd
from pathlib import Path
import signal
import shutil
import socket
import subprocess
import sys
import threading
import time
import yaml

ROOT=Path(__file__).resolve().parents[1]


def clock_anchor():
    before=time.monotonic();unix=time.time();after=time.monotonic()
    return dict(unix_s=unix,monotonic_s=(before+after)/2,
                sampling_half_width_s=(after-before)/2)


def port_available(port):
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as probe:
        try:probe.bind(('127.0.0.1',port))
        except OSError:return False
    return True


def udp_port_bound(port):
    # Read socket state during startup instead of briefly binding/unbinding the
    # controller's port in a polling loop, which could race its initial bind.
    for path in (Path('/proc/net/udp'),Path('/proc/net/udp6')):
        if not path.exists():continue
        for line in path.read_text().splitlines()[1:]:
            fields=line.split()
            if len(fields)>1 and int(fields[1].rsplit(':',1)[1],16)==port:return True
    return False


def controller_pids():
    pids=[]
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():continue
        try:
            if (path/'exe').resolve().name=='z1_ctrl':pids.append(int(path.name))
        except (OSError,RuntimeError):continue
    return pids


def pid_alive(pid):
    try:os.kill(pid,0)
    except ProcessLookupError:return False
    except PermissionError:return True
    return True


class TimeoutConsole:
    """Coalesce only the SDK's per-receive notices; raw logs stay complete."""
    NOTICES={
        '[WARNING] UDPPort::recv, unblock version, wait time out',
        '[WARNING] UDPPort::recv, unblock version, connect with z1_sdk wait time out',
    }

    def __init__(self,name,emit,interval=5.):
        self.name=name;self.emit=emit;self.interval=interval
        self.last={};self.suppressed={}

    def flush(self,now,force=False):
        for message,count in list(self.suppressed.items()):
            if count and (force or now-self.last[message]>=self.interval):
                self.emit('[%s] %s (repeated %d times; full output in %s.log)\n' % (
                    self.name,message,count,self.name))
                self.suppressed[message]=0;self.last[message]=now

    def feed(self,line,now):
        self.flush(now)
        message=line.strip()
        if message not in self.NOTICES:
            self.emit('['+self.name+'] '+line);return
        if message not in self.last:
            self.emit('['+self.name+'] '+line);self.last[message]=now
        else:self.suppressed[message]=self.suppressed.get(message,0)+1


class LoggedProcess:
    def __init__(self,name,command,cwd,log_dir,ready_text=None,preserve_terminal_session=False):
        self.name=name;self.thread=None
        self.ready_text=ready_text;self.ready=threading.Event()
        if preserve_terminal_session:
            # sudo's default cache belongs to the controlling terminal/session.
            # setsid loses it, even after sudo -v in the invoking shell. Keep
            # that session but isolate the child's process group for shutdown.
            # An exec wrapper supports Python 3.8/3.10 without unsafe preexec_fn
            # in this threaded parent (Popen process_group needs Python 3.11).
            command=[sys.executable,'-c',
                'import os,sys;os.setpgrp();os.execvp(sys.argv[1],sys.argv[1:])']+list(command)
        self.log=(log_dir/(name+'.log')).open('x',buffering=1)
        try:
            self.events=(log_dir/(name+'_events.jsonl')).open('x',buffering=1)
            self.process=subprocess.Popen(command,cwd=str(cwd),stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1,
                errors='replace',start_new_session=not preserve_terminal_session,
                env=dict(os.environ,PYTHONUNBUFFERED='1'))
        except BaseException:
            self.log.close()
            if hasattr(self,'events'):self.events.close()
            raise
        self.thread=threading.Thread(target=self._copy,daemon=True);self.thread.start()

    def _copy(self):
        def emit(line):
            try:print(line,end='',flush=True)
            except (BrokenPipeError,OSError):pass
        console=TimeoutConsole(self.name,emit)
        try:
            for line in self.process.stdout:
                received=time.monotonic()
                self.log.write(line)
                self.events.write(json.dumps(dict(monotonic=received,source=self.name,line=line,
                    timestamp_basis='launcher_host_pipe_receipt_not_motor_capture'))+'\n')
                if self.ready_text and self.ready_text in line:self.ready.set()
                console.feed(line,received)
        finally:
            console.flush(time.monotonic(),force=True)
            self.log.close();self.events.close();self.process.stdout.close()

    def signal(self,signum):
        if self.process.poll() is None:
            try:os.killpg(self.process.pid,signum)
            except ProcessLookupError:pass

    def finish(self,timeout=15):
        if self.process.poll() is None:self.signal(signal.SIGINT)
        try:self.process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.signal(signal.SIGTERM)
            try:self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.signal(signal.SIGKILL);self.process.wait(timeout=3)
        self.thread.join(timeout=2)


def capture_command(interface,log_dir):
    """Passive UDP capture only; privilege applies to tcpdump, never control."""
    socket.if_nametoindex(interface)
    tcpdump=shutil.which('tcpdump')
    if not tcpdump:raise RuntimeError('tcpdump is unavailable; install it before requesting capture')
    command=[tcpdump,'-i',interface,'-nn','-s','0','-U','-w',str(log_dir/'z1_arm.pcap'),
             'udp and host 192.168.123.110 and port 8881']
    if os.geteuid()!=0:
        sudo=shutil.which('sudo')
        if not sudo:raise RuntimeError('capture requires sudo; run sudo -v in this terminal first')
        command=[sudo,'-n','--',tcpdump,'-Z',pwd.getpwuid(os.getuid()).pw_name]+command[1:]
    return command


def supervise(root,z1_dir,mode,interface='eth0',seconds=None,display=False,device='auto',startup_timeout=8.,
              capture_z1_network=False):
    root=Path(root).resolve();z1_dir=Path(z1_dir).expanduser().resolve()
    cfg=yaml.safe_load((root/'config/deployment.yaml').read_text())
    if cfg['z1_controller_ip'] not in ('127.0.0.1','localhost'):
        raise ValueError('one-terminal launcher requires a local z1_controller_ip')
    controller_port=int(cfg['z1_to_port']);client_port=int(cfg['z1_own_port'])
    if not (0<controller_port<65536 and 0<client_port<65536 and controller_port!=client_port):
        raise ValueError('invalid Z1 UDP port configuration')
    if startup_timeout<=0:raise ValueError('--startup-timeout must be positive')
    if seconds is not None and seconds<0:raise ValueError('--seconds must be nonnegative')
    # Held through both children: a second launcher cannot start another SDK.
    with (root/'.real_launcher.lock').open('a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise RuntimeError('another start_real.sh is already running in this directory')
        if not port_available(client_port):
            raise RuntimeError('UDP %d is occupied by an SDK client; exit that program before starting'%client_port)
        existing=controller_pids()
        if len(existing)>1:raise RuntimeError('multiple z1_ctrl processes: '+str(existing))
        if not existing:
            if not port_available(controller_port):
                raise RuntimeError('UDP %d is occupied, but its owner could not be identified as z1_ctrl'%controller_port)
            executable=z1_dir/'z1_ctrl'
            if not executable.is_file() or not os.access(executable,os.X_OK):
                raise FileNotFoundError('z1_ctrl not executable: '+str(executable)+'; use --z1-dir /actual/build')
        log_dir=root/'runs'/('launcher_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        log_dir.mkdir(parents=True,exist_ok=False)
        print('Launcher logs: '+str(log_dir),flush=True)
        controller=main=capture=None;interrupted=threading.Event();old_handlers={}
        metadata=dict(mode=mode,interface=interface,python=sys.executable,
            z1_directory=str(z1_dir),z1_port=controller_port,sdk_port=client_port,
            reused_controller_pid=existing[0] if existing else None,
            owned_controller_pid=None,main_pid=None,capture_z1_network=capture_z1_network,
            clock_anchor_start=clock_anchor())
        def save_metadata():
            (log_dir/'launcher.json').write_text(json.dumps(metadata,indent=2)+'\n')
        def interrupt(signum,frame):interrupted.set()
        for signum in (signal.SIGINT,signal.SIGTERM):
            old_handlers[signum]=signal.signal(signum,interrupt)
        result=1
        try:
            if capture_z1_network:
                capture=LoggedProcess('capture',capture_command(interface,log_dir),root,log_dir,
                                      ready_text='listening on',preserve_terminal_session=True)
                metadata['capture_pid']=capture.process.pid
                metadata['capture_filter']='udp and host 192.168.123.110 and port 8881'
                metadata['capture_file']='z1_arm.pcap';save_metadata()
                deadline=time.monotonic()+startup_timeout
                while not capture.ready.is_set():
                    if interrupted.is_set():return 130
                    if capture.process.poll() is not None or time.monotonic()>=deadline:
                        raise RuntimeError('Z1 packet capture did not start; see capture.log. '
                                           'Run sudo -v in this terminal, then retry. Controllers were not started.')
                    time.sleep(.02)
                print('Passive Z1 UDP capture ready: '+str(log_dir/'z1_arm.pcap'),flush=True)
            if existing:print('Reusing z1_ctrl PID %d; it will remain running on exit.'%existing[0],flush=True)
            else:
                controller=LoggedProcess('z1',['./z1_ctrl'],z1_dir,log_dir)
                metadata['owned_controller_pid']=controller.process.pid
                print('Started z1_ctrl PID %d.'%controller.process.pid,flush=True)
            save_metadata();deadline=time.monotonic()+startup_timeout
            while True:
                if interrupted.is_set():return 130
                if controller is not None and controller.process.poll() is not None:
                    raise RuntimeError('z1_ctrl exited before startup; see '+str(log_dir/'z1.log'))
                if existing and not pid_alive(existing[0]):raise RuntimeError('existing z1_ctrl exited')
                if udp_port_bound(controller_port):break
                if time.monotonic()>=deadline:
                    raise RuntimeError('z1_ctrl did not bind UDP %d; see controller output'%controller_port)
                time.sleep(.05)
            print('Z1 controller port ready; starting main program. Feedback freshness is checked by run.py.',flush=True)
            command=[sys.executable,'-u',str(root/'run.py'),mode,'--interface',interface,'--vision-device',device]
            if seconds is not None:command+=['--seconds',str(seconds)]
            if display:command.append('--display')
            main=LoggedProcess('main',command,root,log_dir);metadata['main_pid']=main.process.pid;save_metadata()
            stop_deadline=None;controller_lost=False
            capture_lost_reported=False
            while main.process.poll() is None:
                if capture is not None and capture.process.poll() is not None and not capture_lost_reported:
                    print('Z1 packet capture ended early; see capture.log. Control state is unchanged.',flush=True)
                    metadata['capture_ended_early']=True;capture_lost_reported=True
                lost=(controller is not None and controller.process.poll() is not None) or (existing and not pid_alive(existing[0]))
                if lost and not controller_lost:
                    print('z1_ctrl exited; interrupting main so it can close its IO.',flush=True)
                    controller_lost=True
                if (interrupted.is_set() or controller_lost) and stop_deadline is None:
                    main.signal(signal.SIGINT);stop_deadline=time.monotonic()+15
                if stop_deadline is not None and time.monotonic()>stop_deadline:
                    print('Main did not exit after SIGINT; terminating only this launcher\'s child.',flush=True)
                    main.finish(timeout=0);break
                time.sleep(.05)
            result=main.process.returncode
            if controller_lost and result==0:result=1
            if interrupted.is_set():result=130
            return result if result>=0 else 128-result
        finally:
            # The robot entry closes first; only then stop a controller we own.
            try:
                if main is not None:main.finish()
            finally:
                try:
                    if controller is not None:
                        print('Stopping z1_ctrl started by this launcher.',flush=True)
                        controller.finish(timeout=5)
                finally:
                    if capture is not None:capture.finish(timeout=3)
                    metadata['main_returncode']=main.process.returncode if main else None
                    metadata['capture_returncode']=capture.process.returncode if capture else None
                    metadata['clock_anchor_end']=clock_anchor()
                    metadata['interrupted']=interrupted.is_set();save_metadata()
                    for signum,handler in old_handlers.items():signal.signal(signum,handler)
                    print('Launcher logs retained: '+str(log_dir),flush=True)


def parse_args():
    parser=argparse.ArgumentParser(description=__doc__)
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--run',action='store_true',help='default: main waits for gamepad A before takeover')
    modes.add_argument('--preflight',action='store_true',help='state check with Z1 PASSIVE; no B2 takeover')
    parser.add_argument('--interface',default='eth0')
    parser.add_argument('--z1-dir',type=Path,default=Path('/home/unitree/z1_controller/build'))
    parser.add_argument('--seconds',type=float)
    windows=parser.add_mutually_exclusive_group()
    windows.add_argument('--display',dest='display',action='store_true',help='front/rear diagnostic windows (5 Hz)')
    windows.add_argument('--no-display',dest='display',action='store_false',help='disable optional GUI windows')
    parser.set_defaults(display=True)
    parser.add_argument('--vision-device',choices=['auto','cpu','cuda:0'],default='auto')
    parser.add_argument('--startup-timeout',type=float,default=8.)
    parser.add_argument('--capture-z1-network',action='store_true',
                        help='passively record Z1 arm UDP; run sudo -v first; main remains unprivileged')
    return parser.parse_args()


def main():
    args=parse_args()
    try:
        return supervise(ROOT,args.z1_dir,'--preflight' if args.preflight else '--run',
            args.interface,args.seconds,args.display,args.vision_device,args.startup_timeout,
            args.capture_z1_network)
    except (OSError,ValueError,RuntimeError) as error:
        print('Launcher error: '+str(error),file=sys.stderr,flush=True);return 1


if __name__=='__main__':sys.exit(main())
