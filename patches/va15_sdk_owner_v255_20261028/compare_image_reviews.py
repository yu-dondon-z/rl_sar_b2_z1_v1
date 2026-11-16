"""Compare identical saved-image geometry, not motion or live performance."""
import argparse
import json
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('baseline',type=Path);p.add_argument('current',type=Path)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    if args.output.exists():raise RuntimeError('New output file required')
    baseline=json.loads((args.baseline/'summary.json').read_text())
    current=json.loads((args.current/'summary.json').read_text())
    assert baseline['checkpoint_sha256']==current['checkpoint_sha256']
    before={(r['sequence'],r['side']):r for r in baseline['reports']}
    after={(r['sequence'],r['side']):r for r in current['reports']}
    assert before.keys()==after.keys()
    comparisons=[]
    for key,a in before.items():
        b=after[key];assert a['image_sha256']==b['image_sha256']
        comparisons.append(dict(sequence=key[0],side=key[1],
            baseline_long=bool(a.get('long_valid')),current_long=bool(b.get('long_valid')),
            baseline_span_px=a.get('span_px'),current_span_px=b.get('span_px'),
            baseline_near=bool(a.get('near_detected')),current_near=bool(b.get('near_detected')),
            false_endpoint=bool(b.get('endpoint_candidate'))))
    regressions=[r for r in comparisons if r['baseline_long'] and not r['current_long']]
    recovered=[r for r in comparisons if not r['baseline_long'] and r['current_long']]
    report=dict(geometry_comparison_pass=not regressions and not any(r['false_endpoint'] for r in comparisons),
        comparisons=comparisons,recovered=recovered,regressions=regressions,
        baseline_tracker_sha256=baseline['tracker_sha256'],current_tracker_sha256=current['tracker_sha256'],
        checkpoint_sha256=current['checkpoint_sha256'],hardware_qualified=False,
        timing_qualified=False,complete_history=False,live_motion_permission=False)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));return 0 if report['geometry_comparison_pass'] else 2


if __name__=='__main__':raise SystemExit(main())
