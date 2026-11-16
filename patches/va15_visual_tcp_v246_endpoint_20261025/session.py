"""Separate attended near-image experiment; never starts movement automatically."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import tempfile
import time
from check_bundle import check

ROOT = Path(__file__).resolve().parent


def camera_live(path, nonce):
    try:
        p = json.loads(path.read_text())
        return (p['session_id'] == nonce and p['schema'] == 'va15_long_visual_v1' and
                p.get('image_received') is True and
                type(p['sequence']) is int and p['sequence'] > 0 and
                0 <= time.monotonic_ns()-p['capture_monotonic_ns'] <= 1_000_000_000)
    except (OSError, ValueError, KeyError, TypeError):
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('interface', nargs='?', default='eth0')
    modes = parser.add_mutually_exclusive_group()
    for name in ('--offline-check', '--preview', '--shadow', '--walk'):
        modes.add_argument(name, action='store_const', const=name, dest='mode')
    parser.set_defaults(mode='--preview')
    parser.add_argument('--headless', action='store_true')
    args = parser.parse_args()
    bundle_sha = check()
    if args.mode == '--offline-check':
        return 0
    if args.mode == '--walk':
        qualification = json.loads((ROOT/'qualification.json').read_text())
        if not qualification.get('walk_release_allowed',False):
            raise RuntimeError(qualification.get('walk_release_reason',
                'Candidate motion not released; offline preparation only.'))
        if not qualification.get('simulation_pass') or qualification.get('hardware_qualified'):
            raise RuntimeError('Simulation acceptance missing; motion remains disabled')
        for name, expected in qualification['production_sha256'].items():
            if hashlib.sha256((ROOT/'production'/name).read_bytes()).hexdigest() != expected:
                raise RuntimeError('Simulation-tested production source changed')
        print('V2.4.6 endpoint candidate: actual rear virtual TCP + finite observed tail + supported standing stop; not hardware-qualified.',flush=True)
    if not (Path('/sys/class/net')/args.interface).is_dir():
        raise RuntimeError('Network interface missing')
    if not args.headless and not os.environ.get('DISPLAY'):
        raise RuntimeError('Use a NoMachine terminal, or --headless for saved overlays')
    if args.mode in ('--shadow', '--walk') and (not sys.stdin.isatty() or not sys.stdout.isatty()):
        raise RuntimeError('Control requires an attended board terminal')
    outputs = ROOT.parents[1]/'outputs';outputs.mkdir(exist_ok=True)
    directory = Path(tempfile.mkdtemp(prefix='va15_near_'+args.mode[2:]+'_', dir=outputs))
    exchange = directory/'exchange';exchange.mkdir()
    nonce = secrets.token_hex(16)
    meta = dict(schema='va15_near_session_v1', mode=args.mode, created_unix_s=time.time(),
                session_id=nonce, bundle_sha256=bundle_sha, hardware_qualified=False,
                tcp_target_changed=True, optical_rgb_no_depth=True, flat_floor_assumption=True,
                local_pose_source='experimental_imu_joint_no_slip',inline_zero_output_review_required=False,
                deployment_version='V2.4.6 endpoint',control_core_version='V2.4.3 actuation/stop + opt-in terminal deceleration',lost_policy='diagnostic_only',lost_semantics_verified=False,
                completion='measured virtual TCP endpoint + original supported stable stand; NOT body crossing or visual loss',
                repeatable=True,continuous_until_stop=True,manual_camera_reset_required=False)
    (directory/'session.json').write_text(json.dumps(meta, indent=2)+'\n')
    command = [sys.executable, '-u', str(ROOT/'camera_near.py'), args.interface,
               '--exchange', str(exchange), '--session-id', nonce]
    if not args.headless:
        command.append('--display')
    if args.mode in ('--walk','--shadow'):
        command.append('--require-context')
    interrupted = False

    def interrupted_handler(signum, frame):
        nonlocal interrupted
        interrupted = True  # foreground controller receives terminal SIGINT itself

    handlers = {sig: signal.signal(sig, interrupted_handler) for sig in (signal.SIGINT, signal.SIGTERM)}
    camera = None
    try:
        print('Session records: '+str(directory), flush=True)
        with (directory/'vision.log').open('x') as log:
            camera = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log,
                                      stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+45
            while not camera_live(exchange/'near_guide.json', nonce):
                if interrupted:
                    return 130
                if camera.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('No live camera; no control started. Inspect '+str(directory/'vision.log'))
                time.sleep(.1)
            print('Camera alive. Prone INVALID is expected; it is not motion permission.', flush=True)
            if args.mode in ('--shadow', '--walk'):
                control = subprocess.Popen(['bash', str(ROOT/'run_control.sh'), args.interface, args.mode,
                                            str(exchange), nonce, str(directory)])
                while control.poll() is None:
                    time.sleep(.1)
                print('Controller exited (code '+str(control.returncode)+'); closing this session camera.',flush=True)
                return control.returncode
            while not interrupted and camera.poll() is None:
                time.sleep(.1)
            return 130 if interrupted else camera.returncode
    finally:
        # Only this owned camera child. Never force-kill a controller, a
        # process group, or the independently running Z1 service.
        if camera is not None and camera.poll() is None:
            camera.terminate()
            try:
                camera.wait(timeout=3)
            except subprocess.TimeoutExpired:
                camera.kill();camera.wait(timeout=3)
        if camera is not None:
            print('Session camera closed. No controller or Z1 service was force-killed.',flush=True)
        for sig, handler in handlers.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, subprocess.CalledProcessError, OSError) as e:
        print(str(e), file=sys.stderr)
        raise SystemExit(2)
