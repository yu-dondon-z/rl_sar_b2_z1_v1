"""Restored camera observer beside the existing V2 controller, never a follower.

Only the explicitly attended --standby / --forward-test options delegate to the
existing RUN-gated launcher. Camera packets cannot reach that controller.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[1]
TRIAL = ROOT.with_name('va15_motor_contract_v2_20261021')/'run_trial.sh'


def camera_ready(path, now_ns):
    """A live INVALID/prone frame is healthy; never confuse lock with motion permission."""
    try:
        payload = json.loads(Path(path).read_text())
        if not isinstance(payload, dict):
            return False
        stamp = payload.get('capture_monotonic_ns')
        sequence = payload.get('source_sequence')
        return (type(stamp) is int and type(sequence) is int and sequence > 0 and
                0 <= now_ns-stamp <= 2_000_000_000)
    except (OSError, ValueError, TypeError):
        return False


def control_command(mode, interface='eth0'):
    if mode not in ('--standby', '--forward-test'):
        raise ValueError('Not an attended control mode')
    return ['bash', str(TRIAL), interface, mode]


def camera_command(interface, exchange, sdk, rha, checkpoint, headless):
    command = [sys.executable, '-u', str(ROOT/'scripts/start_bridge.py'), interface,
               '--camera-calibration', str(ROOT/'config/b2_builtin_front_camera.yaml'),
               '--unitree-sdk2-python', str(sdk), '--rhacracknet-root', str(rha),
               '--checkpoint', str(checkpoint), '--device', 'cpu',
               '--exchange-dir', str(exchange), '--preview-only', '--save-overlay']
    if not headless:
        command.append('--display')
    return command


def stop_camera(process):
    """Only terminate the child Popen that this session owns, never groups/Z1/control."""
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
    process.wait(timeout=3)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('interface', nargs='?', default='eth0')
    modes = parser.add_mutually_exclusive_group()
    for mode in ('--offline-check', '--preview', '--standby', '--forward-test'):
        modes.add_argument(mode, action='store_const', const=mode, dest='mode')
    parser.set_defaults(mode='--preview')
    parser.add_argument('--headless', action='store_true', help='Save latest_overlay.png instead of opening a window')
    args = parser.parse_args(argv)
    if args.mode == '--offline-check':
        return subprocess.call([sys.executable, str(ROOT/'check_vision.py'), '--model-smoke'])
    if not (Path('/sys/class/net')/args.interface).is_dir():
        raise RuntimeError('Network interface missing: '+args.interface)
    if not args.headless and not os.environ.get('DISPLAY'):
        raise RuntimeError('DISPLAY is empty. Use a NoMachine terminal, or add --headless to save overlays.')
    if args.mode != '--preview' and (not sys.stdin.isatty() or not sys.stdout.isatty()):
        raise RuntimeError('Control requires an attended board terminal. Use --preview for camera-only checks.')
    subprocess.run([sys.executable, str(ROOT/'check_vision.py')], check=True)
    from check_vision import paths
    sdk, rha, checkpoint = paths()
    outputs = REPO/'outputs'
    outputs.mkdir(exist_ok=True)
    session = Path(tempfile.mkdtemp(prefix='vision_observer_20261022_', dir=outputs))
    exchange = session/'exchange'
    exchange.mkdir()
    # No reuse of /tmp/b2_z1_real_vision: stale files and previous controllers
    # cannot cross into this preview, and this preview cannot feed production.
    metadata = dict(schema='va15_vision_observer_v1', mode=args.mode,
                    visual_control_enabled=False, camera_packets_applied_to_actor=False,
                    hardware_qualified=False, exchange_directory=str(exchange),
                    note='Camera display only; existing V2 motor/stop gates are unchanged.')
    with (session/'session.json').open('x') as stream:
        json.dump(metadata, stream, indent=2)
    command = camera_command(args.interface, exchange, sdk, rha, checkpoint, args.headless)
    print('CAMERA OBSERVER ONLY: no visual velocity/TCP commands, no automatic following.', flush=True)
    print('Records and latest image: '+str(exchange), flush=True)
    print('LOCKED PATH is 2D recognition, NOT a validated 3D path or motion permission.', flush=True)
    print('Q/Esc closes the camera; use the existing X/B/prone/Passive/Ctrl+C procedure for control.', flush=True)
    camera = None
    controller = None
    interrupted = False

    def on_interrupt(signum, frame):
        nonlocal interrupted
        interrupted = True
        # Foreground controller receives the terminal signal itself. Do not
        # force-kill it or Z1; physical lowering/stop remains its own workflow.

    previous_handlers = {sig: signal.signal(sig, on_interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        with (session/'vision.log').open('x') as log:
            camera = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                      start_new_session=True)
            deadline = time.monotonic()+45
            while not camera_ready(exchange/'latest_vision_status.json', time.monotonic_ns()):
                if interrupted:
                    return 130
                if camera.poll() is not None:
                    raise RuntimeError('Camera failed before a live frame; inspect '+str(session/'vision.log'))
                if time.monotonic() >= deadline:
                    raise RuntimeError('No recent camera frame after 45 s; no control started. Inspect '+str(session/'vision.log'))
                time.sleep(.1)
            print('Live camera frame received. INVALID while prone does not prevent static startup.', flush=True)
            if args.mode != '--preview':
                call = control_command(args.mode, args.interface)
                print('Starting existing attended V2 launcher. Vision still CANNOT steer or change TCP.', flush=True)
                controller = subprocess.Popen(call)
                camera_warning_printed = False
                while controller.poll() is None:
                    if camera.poll() is not None and not camera_warning_printed:
                        print('Camera closed. Control remains nonvisual: X stop, B lower, Ctrl+C only when prone/Passive.', flush=True)
                        camera_warning_printed = True
                    time.sleep(.1)
                return controller.returncode
            while camera.poll() is None and not interrupted:
                time.sleep(.1)
            return 130 if interrupted else camera.returncode
    finally:
        if camera is not None:
            stop_camera(camera)
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, subprocess.CalledProcessError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(2)
