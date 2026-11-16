"""Counterfactual startup on recorded camera packets; not closed-loop validation."""
import argparse
import gzip
import json
import sys
from pathlib import Path

sys.path.insert(0, '/home/dong/b2_z1/scripts/vision')
from vd1_front_path_estimator import FrontPathEstimator
from vd1_visual_guard import FrontStartupGuidance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    packets = {p['front']['sequence']: p for p in map(json.loads,
        (args.session/'vision.jsonl').read_text().splitlines())}
    with gzip.open(args.session/'control.jsonl.gz', 'rt') as stream:
        rows = [r for r in map(json.loads, stream) if r.get('state') == 'tracking']
    estimator, startup = FrontPathEstimator(), FrontStartupGuidance()
    transitions, previous, first_yaw = [], None, None
    for row in rows:
        sequence = row['visual_status']['front_estimator_sequence']
        packet = packets[sequence]; front = packet['front']
        geometry, diagnostics = estimator.update(front)
        x, y = geometry[-1][:2]
        yaw = max(-.8, min(.8, .4*y/(x*x+y*y)))
        identity = (packet['capture_generation'], sequence,
                    (front['capture_timestamp'], packet['rear']['capture_timestamp']))
        elapsed = row['monotonic']-rows[0]['monotonic']
        try:
            command = startup.update(diagnostics, identity, yaw, .2, .02)
        except ValueError as error:
            transitions.append(dict(t=elapsed, phase='stop', reason=str(error)))
            break
        if first_yaw is None:
            first_yaw = dict(quadratic=yaw, startup=command)
        phase = diagnostics['front_startup_phase']
        if phase != previous:
            transitions.append(dict(t=elapsed, phase=phase))
            previous = phase
    print(json.dumps(dict(closed_loop=False, first_yaw_rps=first_yaw,
                         transitions=transitions), indent=2))


if __name__ == '__main__':
    main()
