#!/usr/bin/env python3
"""Build a hash-checked, reversible Python-only candidate; never starts IO."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

SOURCE=Path(__file__).resolve().parents[1]
PATCH='vd2_arm_handover_20261107'
FILES=('run.py','python/control.py','python/feedback.py','tests/test_arm_handover.py',
       'tests/test_feedback.py','tests/test_stop_arm_reference.py',
       'tools/replay_arm_handover.py','docs/ARM_HANDOVER_20261107.md')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-manifest',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    manifest=json.loads(args.baseline_manifest.read_text())
    if manifest.get('package')!='rl_sar_b2_z1_VD2':raise ValueError('wrong package')
    records={name:dict(before_sha256=manifest['files_sha256'].get(name),
        after_sha256=hashlib.sha256((SOURCE/name).read_bytes()).hexdigest()) for name in FILES}
    update=dict(id=PATCH,package=manifest['package'],requires_cpp_rebuild=False,
        hardware_validation=False,description='arm reference slew limit and frozen moving feedback inhibition',
        backup_prefix='arm_handover',files=records,
        baseline_manifest_sha256=hashlib.sha256(args.baseline_manifest.read_bytes()).hexdigest())
    with zipfile.ZipFile(args.output,'x',zipfile.ZIP_DEFLATED) as archive:
        archive.writestr(PATCH+'/PATCH.json',json.dumps(update,indent=2)+'\n')
        archive.write(SOURCE/'tools/install_feedback_fix.py',PATCH+'/install.py')
        archive.write(SOURCE/'docs/ARM_HANDOVER_20261107.md',PATCH+'/README.md')
        for name in FILES:archive.write(SOURCE/name,PATCH+'/files/'+name)
    print(json.dumps(dict(path=str(args.output),sha256=hashlib.sha256(args.output.read_bytes()).hexdigest())))


if __name__=='__main__':main()
