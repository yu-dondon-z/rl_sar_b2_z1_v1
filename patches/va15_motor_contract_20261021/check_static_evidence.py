#!/usr/bin/env python3
"""Verify the specific reviewed STATIC run. Never grants current motion permission."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

from review_static import review_rows

REVIEWED_RUN = 'motor_contract_static_20261021_VcRI5C'
EXPECTED_FILES = {
    'terminal.log': '382b6006801ab47300bec1a184a7d46b050d170373c366105a26580c94151e37',
    'commands.csv': 'fe84a1034f1bf4b81a2f7001015a5f09e37145bfecab61d969de42301178c0d9',
    'control.csv': '55432df699c260c389688100d82d96bf078e95e61d2b52ba043d987ea57bf8ba',
    'actor.csv': '4858f0d5c40d509297ac701e33b73a0730688d578e261039e7d6b31d0e5b9678',
    'telemetry_summary.json': 'e2ccafa9eb34425e872d63b96348cc5354f29063f90b05c0891797512af6b5f5',
}
EXPECTED_BUNDLE = 'b884cde9b8edf366ef2b75a3aa2520d7d1b7fee0672358853316929af53b3586'


def verify_hashes(folder, expected):
    for name, digest in expected.items():
        path = folder / name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError('Missing or changed reviewed evidence: ' + str(path))


def verify_reviewed_run(folder):
    verify_hashes(folder, EXPECTED_FILES)
    with (folder / 'commands.csv').open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    with (folder / 'actor.csv').open(newline='') as stream:
        actors = sum(1 for _ in csv.DictReader(stream))
    summary = json.loads((folder / 'telemetry_summary.json').read_text())
    result = review_rows(rows, actors, (folder / 'terminal.log').read_text(errors='replace'), summary)
    if not result['software_static_review_pass']:
        raise ValueError('The pinned static record does not pass software review')
    if result['hardware_qualified'] or result['motion_authorized']:
        raise ValueError('Static review must never imply hardware/motion authorization')
    return result


def main():
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence-dir', type=Path, default=root.parents[1]/'outputs'/REVIEWED_RUN)
    args = parser.parse_args()
    verify_hashes(root/'native_build_v2', {'bundle.json': EXPECTED_BUNDLE})
    result = verify_reviewed_run(args.evidence_dir)
    print('PASS pinned static A/Y/B/Passive + clean shutdown: ' + REVIEWED_RUN)
    print('Recorded evidence only; NOT current support, fall protection, or motion authorization. '
          'Live guards and an attended explicit RUN confirmation remain required.')


if __name__ == '__main__':
    main()
