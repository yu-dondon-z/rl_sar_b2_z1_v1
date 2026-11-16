#!/usr/bin/env python3
"""Offline expected-behavior checks, NOT permission or real motion acceptance."""
import hashlib
import json
from pathlib import Path

root=Path(__file__).resolve().parent
suite=json.loads((root/'outputs/suite_final/summary.json').read_text())
if suite['hardware_qualified'] or suite['passed']!=suite['total'] or suite['total']<31:
    raise SystemExit('Offline suite incomplete/failed or invalid qualification metadata')
for name,expected in suite['sources_sha256'].items():
    if hashlib.sha256((root/name).read_bytes()).hexdigest()!=expected:
        raise SystemExit('Offline evidence no longer matches source: '+name)
scenarios=set()
for result in suite['results']:
    p=result['report'];scenarios.add(p['scenario'])
    if result['exit']!=0 or not p['pass'] or p['fell'] or p['false_catch'] or p['sport_leak']:
        raise SystemExit('Offline expected-behavior failure: '+result['name'])
    if p['max_command']>.150001 or p['positive_request_s']>4.000001:
        raise SystemExit('Offline motion budget mismatch')
    if p['scenario']=='deadline' and not p['forward_and_each_foot_liftoff']:
        raise SystemExit('Deadline test did not produce forward movement and four-foot liftoff')
    if p['scenario'] in ('lost','stale','motor_fault','frequency','timeout','component','lost_drop','motor_mode') and p['stand_catch_qualified']:
        raise SystemExit('Invalid feedback falsely granted lowering qualification')
if scenarios!={'deadline','release','x','b','lost','stale','preview','held',
    'motor_fault','frequency','timeout','component','lost_drop','motor_mode','timeout_at_start'}:
    raise SystemExit('Missing expected-behavior scenarios')
print('PASS 0.15 m/s / 4 s offline source-bound behavior checks; NOT hardware-qualified')
