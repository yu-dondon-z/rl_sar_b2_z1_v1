#!/usr/bin/env python3
"""Build a hashed independent-publication patch against the board manifest."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

SOURCE = Path(__file__).resolve().parents[1]
REPO = SOURCE.parents[1]
PATCH = 'vd2_independent_vision_20261109'
FILES = ('run.py', 'python/cameras.py', 'python/independent_vision.py',
         'python/vision_start.py', 'tests/test_independent_vision.py',
         'README_板载编译与运行.md',
         'tools/benchmark_recorded_vision.py', 'docs/INDEPENDENT_VISION_20261109.md')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    baseline = json.loads(args.manifest.read_text())
    sources = {name: SOURCE/name for name in FILES}
    sources['tests/test_front_startup_guidance.py'] = REPO/'tests/test_front_startup_guidance.py'
    sources.update({'python/vision/'+name: REPO/'scripts'/name for name in
                    ('vd1_front_path_estimator.py', 'vd1_visual_guard.py')})
    records = {name: dict(before_sha256=baseline['files_sha256'].get(name),
        after_sha256=hashlib.sha256(path.read_bytes()).hexdigest()) for name, path in sources.items()}
    update = dict(id=PATCH, package='rl_sar_b2_z1_VD2', requires_cpp_rebuild=False,
        hardware_validation=False, description='independent front/rear publication and visible steering support',
        backup_prefix='independent_vision', files=records)
    with zipfile.ZipFile(args.output, 'w', zipfile.ZIP_DEFLATED) as archive:
        archive.writestr(PATCH+'/PATCH.json', json.dumps(update, indent=2)+'\n')
        archive.write(SOURCE/'tools/install_feedback_fix.py', PATCH+'/install.py')
        for name, path in sources.items():
            archive.write(path, PATCH+'/files/'+name)
    print(json.dumps(dict(archive=str(args.output), files=len(sources),
        sha256=hashlib.sha256(args.output.read_bytes()).hexdigest()), indent=2))


if __name__ == '__main__':
    main()
