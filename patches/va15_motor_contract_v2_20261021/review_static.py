#!/usr/bin/env python3
"""Read-only CSV review. No robot SDK, commands, automatic permissions or writes."""
import argparse
import csv
import json
import math
from pathlib import Path

def review_rows(rows, actor_count, terminal, integrity):
    rows = [r for r in rows if float(r['low_callback_received_ns']) > 0]
    ready = [r for r in rows if int(r['contract_ready']) == 1]
    flags = sorted({int(r['bit_flag']) for r in rows})
    states = []
    for r in rows:
        state = int(r['fsm'])
        if not states or states[-1] != state:
            states.append(state)
    motors = []
    for i in range(12):
        motors.append(dict(dds_joint=i,
            lost=sorted({int(r[f'lost_dds_{i}']) for r in rows}),
            fault=sorted({int(r[f'motor_fault_dds_{i}']) for r in rows}),
            frequency_min=min((int(r[f'motor_frequency_dds_{i}']) for r in rows), default=None),
            frequency_max=max((int(r[f'motor_frequency_dds_{i}']) for r in rows), default=None)))
    numeric_limits = [float(r[f'limited_{i}']) for r in rows for i in range(3)]
    no_motion = all(math.isfinite(v) and v == 0 for v in numeric_limits) and actor_count == 0
    faults = any(int(r[f'motor_fault_dds_{i}']) != 0 for r in rows for i in range(12))
    latched = sum(int(r['contract_latched']) != 0 for r in rows)
    frozen_baselines = {tuple(int(r[f'contract_baseline_dds_{i}']) for i in range(12)) for r in ready}
    correct_ready = bool(ready) and len(frozen_baselines) == 1 and all(int(r['bit_flag']) == 4 and
        int(r['contract_baseline_frozen']) == 1 and float(r['contract_stable_seconds']) >= 3.-1.e-9 and
        all(0 <= int(r[f'contract_baseline_dds_{i}']) <= 0xffffffff and
            int(r[f'lost_dds_{i}']) == int(r[f'contract_baseline_dds_{i}']) and
            int(r[f'mode_dds_{i}']) == 10 and int(r[f'motor_fault_dds_{i}']) == 0 and
            400 <= int(r[f'motor_frequency_dds_{i}']) <= 800 for i in range(12)) for r in ready)
    # A/Y/B are long states sampled in commands.csv, not inferred from a key print.
    static_sequence = any(states[j:j+5] == [0, 1, 2, 3, 0] for j in range(max(0, len(states)-4)))
    clean_exit = ('COMMAND_EXIT_CODE="0"' in terminal and
                  'B2+Z1 real deployment stopped' in terminal)
    recorder_ok = bool(integrity) and not integrity.get('io_failed', True) and all(
        integrity.get(key, -1) == 0 for key in ('commands_dropped', 'control_dropped', 'actor_dropped'))
    passed = bool(rows) and correct_ready and static_sequence and clean_exit and recorder_ok and no_motion and not latched and not faults
    return dict(software_static_review_pass=passed, hardware_qualified=False,
        motion_authorized=False, physical_prone_confirmation_required=True,
        reason='Software static evidence only; physical state and motion permission are NOT inferred.',
        sourced_rows=len(rows), current_health_ready_rows=len(ready), latched_rows=latched,
        bit_flags_seen=flags, fsm_state_sequence=states, no_motion_or_actor_output=no_motion,
        clean_exit=clean_exit, recorder_complete_without_drops=recorder_ok,
        baseline_lost_semantics_verified=False, motors=motors)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--require-pass', action='store_true',
                        help='Return nonzero for failed static software evidence; never authorizes motion')
    parser.add_argument('directory', type=Path)
    args = parser.parse_args(argv)
    with (args.directory/'commands.csv').open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    with (args.directory/'actor.csv').open(newline='') as stream:
        actors = sum(1 for _ in csv.DictReader(stream))
    integrity = json.loads((args.directory/'telemetry_summary.json').read_text())
    result = review_rows(rows, actors, (args.directory/'terminal.log').read_text(errors='replace'), integrity)
    print(json.dumps(result, indent=2, allow_nan=False))
    return 2 if args.require_pass and not result['software_static_review_pass'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
