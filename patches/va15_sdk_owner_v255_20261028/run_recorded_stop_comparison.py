"""Bounded, SDK-free baseline/candidate recovery comparison, not robot release."""
import concurrent.futures
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def main():
    root = Path(__file__).resolve().parent
    out = root / 'outputs' / 'recorded_stepwide_final'
    out.mkdir(parents=True, exist_ok=True)
    if any(out.glob('*.csv')) or (out / 'summary.json').exists():
        raise SystemExit('Refusing to overwrite a completed or partial matrix')
    sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    baseline = root.parent / 'va15_curve_entry_v251_20261026'
    dependencies = [root / 'simulate_recorded_stop.cpp', Path(__file__),
                    root / 'production/va15_controlled_stop.hpp',
                    root / 'production/fsm_b2_z1_no_gun.hpp',
                    root / 'production/va15_independent_state.hpp',
                    root / 'production/trial_feedback.hpp',
                    root / 'production/b2_independent_model.json',
                    baseline / 'production/va15_controlled_stop.hpp',
                    baseline / 'production/fsm_b2_z1_no_gun.hpp',
                    baseline / 'production/va15_independent_state.hpp',
                    baseline / 'production/trial_feedback.hpp',
                    root / 'build/simulate_recorded_stop',
                    root / 'build/simulate_recorded_stop_baseline',
                    root.parent / 'va15_controlled_getdown_20261020/simulate_controlled_getdown.cpp']
    hashes = {str(path): sha(path) for path in dependencies}

    def run(case):
        controller, feedback = case
        name = f'{controller}_{feedback}_mu0.8'
        csv = out / (name + '.csv')
        executable = root / 'build' / ('simulate_recorded_stop' + ('_baseline' if controller == 'baseline' else ''))
        with (out / (name + '.log')).open('w') as log:
            process = subprocess.run([str(executable), '--scenario', 'supported_stepwide',
                                      '--feedback', feedback, '--friction', '.8',
                                      '--controller', controller, '--output', str(csv)],
                                     stdout=log, stderr=subprocess.STDOUT, timeout=120)
        report_path = Path(str(csv) + '.json')
        report = json.loads(report_path.read_text()) if report_path.exists() else {'pass': False, 'missing_report': True}
        return dict(name=name, exit=process.returncode, report=report)

    cases = [(controller, feedback) for controller in ['baseline', 'candidate']
             for feedback in ['truth', 'estimated']]
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(run, cases))
    unchanged = all(sha(Path(path)) == digest for path, digest in hashes.items())
    report = dict(passed=sum(result['exit'] == 0 and result['report']['pass'] for result in results),
                  total=len(results), results=results, sources_unchanged=unchanged,
                  source_and_binary_sha256=hashes, hardware_qualified=False,
                  no_deployment_release=True, exact_real_incident_reproduced=False)
    (out / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({key: value for key, value in report.items() if key != 'source_and_binary_sha256'}, indent=2))
    return 0 if unchanged and report['passed'] == report['total'] else 2


if __name__ == '__main__':
    sys.exit(main())
