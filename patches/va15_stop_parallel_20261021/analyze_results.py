#!/usr/bin/env python3
"""Audit isolated GetDown scope and compare native offline simulation reports."""
import csv
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def class_parts(text):
    prefix, rest = text.split('class RLFSMStateGetDown :', 1)
    body, suffix = rest.split('class RLFSMStateRLLocomotion :', 1)
    return prefix, ('class RLFSMStateGetDown :' + body).strip(), suffix


def main():
    before = (ROOT/'production/fsm_b2_z1_no_gun.before.hpp').read_text()
    after = (ROOT/'production/fsm_b2_z1_no_gun.hpp').read_text()
    a, b = class_parts(before), class_parts(after)
    assert a[0] == b[0] and a[2] == b[2], 'Changes outside GetDown'
    simulated = (ROOT/'parallel_getdown.hpp').read_text()
    simulated = 'class RLFSMStateGetDown :' + simulated.split('class RLFSMStateGetDown :', 1)[1]
    simulated = simulated.rsplit('\n}', 1)[0].strip()
    assert b[1] == simulated, 'Production and simulated GetDown differ'
    old_main = (ROOT/'production/rl_real_b2_z1_no_gun.before.cpp').read_text()
    new_main = (ROOT/'production/rl_real_b2_z1_no_gun.cpp').read_text()
    for head in ('static void CheckStandTransitionsOffline',):
        assert old_main.split(head)[0] == new_main.split(head)[0]
    assert old_main.split('int main(int argc, char **argv)')[1] == new_main.split('int main(int argc, char **argv)')[1]
    old_manifest = json.loads((ROOT/'production/manifest.before.json').read_text())
    new_manifest = json.loads((ROOT/'production/manifest.json').read_text())
    manifest_changes = [k for k in old_manifest.keys() | new_manifest.keys() if old_manifest.get(k) != new_manifest.get(k)]
    assert manifest_changes == ['controlled_stop_fsm_sha256']
    assert new_manifest['controlled_stop_fsm_sha256'] == digest(ROOT/'production/fsm_b2_z1_no_gun.hpp')
    baseline_dir = ROOT.parent/'va15_stop_response_v3_20261021/outputs/capture_030_050_full_v1'
    final_dir = ROOT/'outputs/parallel_full_final'
    baseline = json.loads((baseline_dir/'summary.json').read_text())
    current = json.loads((final_dir/'summary.json').read_text())
    for path, value in current['hashes'].items():
        assert digest(Path(path)) == value, 'Stale simulation result: ' + path
    assert current['total'] == current['passed'] == 30
    old_by_name = {row['name']: row for row in baseline['results']}
    comparisons = []
    for result in current['results']:
        name = result['name']
        with (baseline_dir/(name+'.csv')).open() as stream:
            old_rows = list(csv.DictReader(stream))
        with (final_dir/(name+'.csv')).open() as stream:
            new_rows = list(csv.DictReader(stream))
        identical = 0
        for x, y in zip(old_rows, new_rows):
            if x['state'] == 'RLFSMStateGetDown' or y['state'] == 'RLFSMStateGetDown':
                break
            assert all(x[k] == y[k] for k in x.keys() & y.keys()), 'Pre-GetDown behavior changed: ' + name
            identical += 1
        comparisons.append(dict(name=name, identical_pre_getdown_rows=identical,
            old_prone_after_B=old_by_name[name]['prone_after_B'], new_prone_after_B=result['prone_after_B']))
    normal = [r for r in current['results'] if r['fault'] == 'none']
    old_normal = [old_by_name[r['name']] for r in normal]
    report = dict(hardware_qualified=False, control_authorized=False,
        scope='Only GetDown runtime changed; native main edits are offline regression tests only',
        pre_getdown_trajectories_identical=True, production_matches_simulated_getdown=True,
        production_main_runtime_unchanged=True, manifest_changed_keys=manifest_changes,
        passed=current['passed'], total=current['total'], normal_cases=len(normal), fault_cases=30-len(normal),
        old_max_prone_after_B=max(r['prone_after_B'] for r in old_normal),
        new_max_prone_after_B=max(r['prone_after_B'] for r in normal),
        old_max_tilt_deg=max(r['max_tilt_deg'] for r in old_normal),
        new_max_tilt_deg=max(r['max_tilt_deg'] for r in normal),
        min_arm_floor_proxy_m=min(r['arm_floor_minimum'] for r in normal),
        min_arm_body_proxy_m=min(r['arm_body_minimum'] for r in normal),
        geometry_violations=sum(r['geometry_violations'] for r in normal), comparisons=comparisons)
    (final_dir/'comparison.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k != 'comparisons'}, indent=2))


if __name__ == '__main__':
    main()
