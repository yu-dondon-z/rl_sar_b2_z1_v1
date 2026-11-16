#!/usr/bin/env python3
"""Package the field-derived stand/B recovery candidate, Python changes only."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

SOURCE=Path(__file__).resolve().parents[1]
PATCH='vd2_stand_recovery_20261106'
FILES=('run.py','python/control.py','tests/test_contract.py','tests/test_feedback.py',
       'tests/test_stand_recovery.py','docs/STAND_RECOVERY_20261106.md')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError('preserve existing update: '+str(args.output))
    manifest=json.loads((args.baseline/'MANIFEST.json').read_text())
    if manifest.get('package')!='rl_sar_b2_z1_VD2':raise ValueError('incorrect baseline package')
    records={}
    for name in FILES:
        old=args.baseline/name
        before=hashlib.sha256(old.read_bytes()).hexdigest() if old.is_file() else None
        if before!=manifest['files_sha256'].get(name):raise ValueError('baseline hash differs: '+name)
        records[name]=dict(before_sha256=before,
            after_sha256=hashlib.sha256((SOURCE/name).read_bytes()).hexdigest())
    update=dict(id=PATCH,package=manifest['package'],requires_cpp_rebuild=False,hardware_validation=False,
        description='stand and B recovery candidate',backup_prefix='stand_recovery',files=records)
    with zipfile.ZipFile(args.output,'w',zipfile.ZIP_DEFLATED) as archive:
        archive.writestr(PATCH+'/PATCH.json',json.dumps(update,ensure_ascii=False,indent=2)+'\n')
        archive.write(SOURCE/'tools/install_feedback_fix.py',PATCH+'/install.py')
        archive.write(SOURCE/'docs/STAND_RECOVERY_20261106.md',PATCH+'/README.md')
        for name in FILES:archive.write(SOURCE/name,PATCH+'/files/'+name)
    print(json.dumps(dict(archive=str(args.output),bytes=args.output.stat().st_size,
        sha256=hashlib.sha256(args.output.read_bytes()).hexdigest()),indent=2))


if __name__=='__main__':main()
