"""Additional SDK-free synthetic pixel-noise stress, NOT sensor qualification."""
import concurrent.futures
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def main():
    root = Path(__file__).resolve().parent
    name = sys.argv[1] if len(sys.argv) == 2 else 'noise_final'
    if Path(name).name != name:
        raise SystemExit('One new output directory name required')
    out = root / 'outputs' / name
    out.mkdir(parents=True, exist_ok=False)
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    tracked = [*(root / 'production').glob('*'), root / 'simulate_preview_noise.cpp', Path(__file__).resolve()]
    hashes = {str(p.relative_to(root)): sha(p) for p in tracked if p.is_file()}
    cases = ['estimated_straight', 'estimated_bend_left', 'estimated_bend_right']

    def run(scenario):
        csv = out / (scenario + '.csv')
        with (out / (scenario + '.log')).open('w') as log:
            result = subprocess.run([str(root / 'build/simulate_preview_noise'), scenario, str(csv), '.8'],
                                    stdout=log, stderr=subprocess.STDOUT, timeout=180)
        file = Path(str(csv) + '.json')
        report = json.loads(file.read_text()) if file.exists() else {'pass': False}
        return dict(name=scenario, exit=result.returncode, report=report)

    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(run, cases))
    if any(sha(root / p) != h for p, h in hashes.items()):
        raise RuntimeError('Sources changed during validation')
    report = dict(passed=sum(r['exit'] == 0 and r['report']['pass'] for r in results),
                  total=len(results), results=results, sources_sha256=hashes,
                  executable_sha256=sha(root / 'build/simulate_preview_noise'),
                  synthetic_pixel_jitter_max_px=4.5, synthetic_rgb=True,
                  independent_estimated_pose=True, hardware_qualified=False)
    (out / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ['sources_sha256', 'executable_sha256']}, indent=2))
    return 0 if report['passed'] == report['total'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
