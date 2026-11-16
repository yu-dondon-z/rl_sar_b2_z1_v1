"""Offline synthetic RGB end-seen stop; never initializes hardware transports."""
import concurrent.futures
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def main():
    root = Path(__file__).resolve().parent
    name = sys.argv[1] if len(sys.argv) == 2 else 'visual_end_acceptance'
    if Path(name).name != name:
        raise SystemExit('One new output directory name required')
    out = root / 'outputs' / name
    out.mkdir(parents=True, exist_ok=False)
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    tracked = [*(root / 'production').glob('*'), root / 'simulate_visual_end.cpp', Path(__file__).resolve()]
    hashes = {str(p.relative_to(root)): sha(p) for p in tracked if p.is_file()}
    cases = [(s, mu) for s in ['straight', 'curve'] for mu in [.5, .8, 1.]]
    cases += [(s, .8) for s in ['no_end', 'unknown_loss', 'camera_failure', 'one_false_cap', 'operator_b']]

    def run(case):
        scenario, mu = case
        stem = f'{scenario}_mu{mu}'
        csv = out / (stem + '.csv')
        with (out / (stem + '.log')).open('w') as log:
            result = subprocess.run([str(root / 'build/simulate_visual_end'), scenario, str(csv), str(mu)],
                                    stdout=log, stderr=subprocess.STDOUT, timeout=180)
        file = Path(str(csv) + '.json')
        report = json.loads(file.read_text()) if file.exists() else {'pass': False}
        return dict(name=stem, exit=result.returncode, report=report)

    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(run, cases))
    if any(sha(root / p) != h for p, h in hashes.items()):
        raise RuntimeError('Sources changed during validation')
    report = dict(passed=sum(r['exit'] == 0 and r['report']['pass'] for r in results),
                  total=len(results), results=results, sources_sha256=hashes,
                  executable_sha256=sha(root / 'build/simulate_visual_end'),
                  synthetic_rgb=True, independent_estimated_pose=True,
                  hardware_qualified=False, rear_tcp_arrival_required=False)
    (out / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ['sources_sha256', 'executable_sha256']}, indent=2))
    return 0 if report['passed'] == report['total'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
