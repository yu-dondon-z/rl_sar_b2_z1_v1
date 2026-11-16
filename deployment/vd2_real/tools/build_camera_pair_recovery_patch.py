#!/usr/bin/env python3
"""Package only the camera rejection fix against the installed board hashes."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

SOURCE = Path(__file__).resolve().parents[1]
FILES = ('python/independent_vision.py', 'tests/test_independent_vision.py',
         'docs/CAMERA_PAIR_RECOVERY_20261109.md')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    baseline = json.loads(args.manifest.read_text())
    records = {name: dict(before_sha256=baseline['files_sha256'].get(name),
        after_sha256=hashlib.sha256((SOURCE/name).read_bytes()).hexdigest()) for name in FILES}
    patch = 'vd2_camera_pair_recovery_20261109'
    update = dict(id=patch, package='rl_sar_b2_z1_VD2', requires_cpp_rebuild=False,
        hardware_validation=False, description='recover from rejected D435i RGB/depth pairs',
        backup_prefix='camera_pair_recovery', files=records)
    with zipfile.ZipFile(args.output, 'w', zipfile.ZIP_DEFLATED) as archive:
        archive.writestr(patch+'/PATCH.json', json.dumps(update, indent=2)+'\n')
        archive.write(SOURCE/'tools/install_feedback_fix.py', patch+'/install.py')
        for name in FILES:
            archive.write(SOURCE/name, patch+'/files/'+name)
    print(hashlib.sha256(args.output.read_bytes()).hexdigest(), args.output)


if __name__ == '__main__':
    main()
