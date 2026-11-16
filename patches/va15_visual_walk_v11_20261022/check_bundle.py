"""Offline only: source/model fingerprints, native golden test, no DDS/UDP."""
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parent


def check(run_native=True):
    bundle_path = ROOT/'native_build_visual/bundle.json'
    bundle = json.loads(bundle_path.read_text())
    if bundle.get('schema') != 'near_visual_trial_bundle_v1' or bundle['hardware_qualified'] or bundle['production_replaced']:
        raise RuntimeError('Unexpected candidate metadata')
    for name, expected in bundle['files'].items():
        p = Path(name)
        if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest() != expected:
            raise RuntimeError('Candidate source/model mismatch: '+str(p))
    if run_native:
        subprocess.run(['python3', str(ROOT.parents[1]/'scripts/check_va15_legacy.py')], check=True)
        subprocess.run([str(ROOT/'native_build_visual/va15_forward_trial'), '--offline-check'], check=True)
    return hashlib.sha256(bundle_path.read_bytes()).hexdigest()


if __name__ == '__main__':
    print('PASS separate near-visual bundle: '+check())
    print('No robot initialized. Optical RGB bearing experiment; TCP unchanged; NOT hardware qualified.')
