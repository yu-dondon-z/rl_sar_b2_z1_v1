"""Parallel SDK-free characterisation; no permissions to start real control."""
import concurrent.futures
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent


def main():
    output = ROOT/'outputs'/(sys.argv[1] if len(sys.argv) > 1 else 'functions_verified')
    output.mkdir(parents=True, exist_ok=False)
    sources = [*ROOT.glob('*.cpp'), *(ROOT/'production').glob('*')]
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    fingerprints = {str(p.relative_to(ROOT)): digest(p) for p in sources if p.is_file()}
    names = ['standby', 'forward', 'backward', 'left', 'right', 'yaw_left', 'yaw_right', 'tcp_sweep',
             'visual_straight', 'visual_left', 'visual_right', 'visual_stale', 'visual_invalid', 'visual_x',
             'visual_offset_left', 'visual_offset_right', 'visual_review_writer']
    names += ['visual_straight_mu05','visual_straight_mu10',
              'visual_acquire_left','visual_acquire_right','visual_acquire_left_mu05','visual_acquire_right_mu05',
              'visual_angle_refused','visual_lost_before','visual_lost_mid','visual_lost_churn']

    def run(name):
        csv = output/(name+'.csv')
        scenario = name.rsplit('_mu',1)[0] if '_mu' in name else name
        extra = ['1', '.5' if name.endswith('mu05') else '1.0'] if '_mu' in name else []
        with (output/(name+'.log')).open('w') as log:
            completed = subprocess.run([str(ROOT/'build/simulate_functions'), scenario, str(csv), *extra],
                                       stdout=log, stderr=subprocess.STDOUT, timeout=120)
        report = json.loads(Path(str(csv)+'.json').read_text()) if Path(str(csv)+'.json').exists() else {}
        return dict(name=name, exit=completed.returncode, report=report)

    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(run, names))
    if any(digest(ROOT/p) != sha for p, sha in fingerprints.items()):
        raise RuntimeError('Source changed during function suite')
    summary = dict(results=results, source_sha256=fingerprints,
                   executable_sha256=digest(ROOT/'build/simulate_functions'),
                   software_invariants_pass=all(r['exit'] == 0 for r in results),
                   hardware_qualified=False, actual_camera_used=False,
                   note='Simulation truth + synthetic delayed near bearings; not real visual accuracy or real-robot acceptance')
    (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))
    return 0 if summary['software_invariants_pass'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
