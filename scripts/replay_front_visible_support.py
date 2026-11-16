"""Replay recorded visual control inputs, not a closed-loop robot evaluation."""
import argparse
import gzip
import json
from pathlib import Path

from vd1_front_path_estimator import FrontPathEstimator
from vd1_visual_guard import GuardedVD1VisualController


def replay(session):
    packets = {}
    for line in (session/'vision.jsonl').read_text().splitlines():
        packet = json.loads(line)
        if packet.get('kind') == 'packet':
            packets[(packet['capture_generation'], packet['front']['sequence'])] = packet
    with gzip.open(session/'control.jsonl.gz', 'rt') as stream:
        rows = list(map(json.loads, stream))
    start = next(i for i, row in enumerate(rows) if row.get('state') == 'tracking')
    end = next((i for i in range(start, len(rows)) if rows[i].get('state') != 'tracking'), len(rows))
    first = rows[start]
    first_status = first['visual_status']
    generation = first_status['capture_generation']
    estimator = FrontPathEstimator()
    # Warm the filter using original, preceding captures, never robot motion.
    for key, packet in sorted(packets.items()):
        if key[0] == generation and key[1] < first_status['source_sequence']:
            if packet['front'].get('valid') is True:
                try:
                    estimator.update(packet['front'])
                except ValueError:
                    pass
    controller = GuardedVD1VisualController(first['tool_pose_b'][:3], forward_speed=.2,
        front_path_estimation=True, front_path_filter=estimator)
    results, transitions = [], []
    previous_phase = None
    previous_time = first['monotonic']-.02
    for row in rows[start:min(end+1, len(rows))]:
        status = row['visual_status']
        key = (status['capture_generation'], status['source_sequence'])
        now = row['monotonic']
        result = controller.update(packets[key], now, now-previous_time)
        previous_time = now
        updated = result['status']
        phase = updated.get('front_startup_phase', updated['state'])
        if phase != previous_phase:
            transitions.append(dict(t_s=now-first['monotonic'], phase=phase,
                                    reason=updated['reason']))
            previous_phase = phase
        results.append(dict(t_s=now-first['monotonic'], sequence=key[1],
            old_yaw_rps=row['base_command_b'][2], new_yaw_rps=result['base_command_b'][2],
            valid=updated['valid'], reason=updated['reason'],
            target=updated.get('guidance_lookahead_xy_ground_m'),
            visible_line_target=updated.get('front_visible_line_target_xy_m'),
            extrapolation_m=updated.get('front_extrapolation_distance_m')))
    valid = [row for row in results if row['valid']]
    return dict(closed_loop=False, session=str(session),
        replayed_steps=len(results), valid_steps=len(valid), transitions=transitions,
        old_yaw_min_rps=min(r['old_yaw_rps'] for r in results),
        new_yaw_min_rps=min(r['new_yaw_rps'] for r in results),
        new_yaw_max_rps=max(r['new_yaw_rps'] for r in results),
        max_extrapolation_m=max(r['extrapolation_m'] for r in valid),
        example_5_64s=min(results, key=lambda r: abs(r['t_s']-5.64)),
        final=results[-1])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    print(json.dumps(replay(args.session), indent=2))


if __name__ == '__main__':
    main()
