"""Require a completed attended zero-motion shadow session before a walk trial."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import time

ROOT = Path(__file__).resolve().parent


def review(directory):
    directory = Path(directory).resolve()
    meta = json.loads((directory/'session.json').read_text())
    summary = json.loads((directory/'telemetry_summary.json').read_text())
    with (directory/'control.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    with (directory/'actor.csv').open() as stream:
        actors = list(csv.DictReader(stream))
    with (directory/'commands.csv').open() as stream:
        commands = list(csv.DictReader(stream))
    current = hashlib.sha256((ROOT/'native_build_visual/bundle.json').read_bytes()).hexdigest()
    if meta.get('mode') != '--shadow' or meta.get('bundle_sha256') != current:
        raise RuntimeError('Not a shadow from this exact candidate bundle')
    if not 0 <= time.time()-meta['created_unix_s'] <= 3600:
        raise RuntimeError('Shadow evidence must be from the last hour')
    if summary['io_failed'] or any(summary[k] for k in ('commands_dropped', 'control_dropped', 'actor_dropped')):
        raise RuntimeError('Incomplete/dropped diagnostic recording')
    if actors or not rows:
        raise RuntimeError('Shadow must record zero actor execution and complete controls')
    states = {int(r['fsm']) for r in rows}
    if not {0, 1, 2, 3}.issubset(states) or int(rows[-1]['fsm']) != 0:
        raise RuntimeError('Need A -> Y -> B -> Passive and normal exit')
    for r in rows:
        if any(abs(float(r['limited_'+str(i)])) > 1e-8 for i in range(3)) or int(r['safety_fault']):
            raise RuntimeError('Shadow applied motion or latched a fault')
    active = [r for r in rows if int(r['visual_stage']) == 1]
    if len(active) < 100:
        raise RuntimeError('Too few independent shadow control samples')
    span = (int(active[-1]['control_start_ns'])-int(active[0]['control_start_ns']))/1e9
    if span < 3.7 or not any(int(r['visual_stage']) == 2 for r in rows):
        raise RuntimeError('Need the full four-second shadow and latched stop')
    if 'four-second monotonic visual request deadline' not in (directory/'terminal.log').read_text(errors='replace'):
        raise RuntimeError('Need a normal monotonic deadline, not an early/manual/stale stop')
    for r in active:
        age = (int(r['control_start_ns'])-int(r['visual_capture_ns']))/1e9
        if not 0 <= age <= .5 or int(r['visual_qualified']) < 6 or not int(r['visual_shadow']):
            raise RuntimeError('Shadow source freshness/qualification failed')
        if not int(r['estimate_valid']) or float(r['h_lower_m']) < .48 or float(r['v_upper_m_s']) >= .04:
            raise RuntimeError('Shadow quiet experimental support was not maintained')
        if abs(float(r['visual_would_yaw'])) > .060001 or abs(float(r['visual_would_x'])-.15) > 1e-6:
            raise RuntimeError('Shadow command outside declared envelope')
        if float(r['max_control_interval_ns']) >= 20_000_000:
            raise RuntimeError('Control-loop timing not qualified even for the bounded trial')
    writes = [r for r in commands if int(r['visual_stage']) == 1]
    if len(writes) < 100:
        raise RuntimeError('Writer evidence incomplete')
    for r in writes:
        age = int(r['publish_end_ns'])-int(r['low_callback_received_ns'])
        if (not int(r['dds_write_return']) or not int(r['contract_ready']) or int(r['contract_latched']) or
                not 0 <= age < 50_000_000 or float(r['max_publish_interval_ns']) >= 20_000_000):
            raise RuntimeError('Current health / writer timing failed in camera-loaded shadow')
    return dict(pass_shadow=True, path=str(directory), active_samples=len(active), span_s=span,
                max_writer_interval_ms=max(float(r['max_publish_interval_ns']) for r in writes)/1e6,
                hardware_qualified=False, tcp_target_changed=False, bundle_sha256=current)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    print(json.dumps(review(args.directory), indent=2))
