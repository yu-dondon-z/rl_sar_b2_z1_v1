#!/usr/bin/env python3
"""Build a reversible update for the stand-recovery candidate installed on board."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile
SOURCE=Path(__file__).resolve().parents[1]
PATCH='vd2_legacy_getup_20261106'
FILES=('run.py','python/control.py','python/native_io.py','native/io.h','native/io.cpp',
       'tests/test_stand_recovery.py','tests/test_contract.py','tests/test_feedback.py',
       'README_板载编译与运行.md','VALIDATION.md',
       'docs/LEGACY_GETUP_20261106.md','docs/STAND_RECOVERY_20261106.md')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    manifest=json.loads((args.baseline/'MANIFEST.json').read_text());records={}
    if manifest.get('package')!='rl_sar_b2_z1_VD2':raise ValueError('incorrect baseline package')
    for name in FILES:
        old=args.baseline/name;before=hashlib.sha256(old.read_bytes()).hexdigest() if old.is_file() else None
        if before!=manifest['files_sha256'].get(name):raise ValueError('baseline hash differs: '+name)
        records[name]=dict(before_sha256=before,after_sha256=hashlib.sha256((SOURCE/name).read_bytes()).hexdigest())
    update=dict(id=PATCH,package=manifest['package'],requires_cpp_rebuild=True,hardware_validation=False,
                description='legacy real get-up restoration',backup_prefix='legacy_getup',files=records)
    with zipfile.ZipFile(args.output,'w',zipfile.ZIP_DEFLATED) as archive:
        archive.writestr(PATCH+'/PATCH.json',json.dumps(update,ensure_ascii=False,indent=2)+'\n')
        archive.write(SOURCE/'tools/install_feedback_fix.py',PATCH+'/install.py')
        archive.write(SOURCE/'docs/LEGACY_GETUP_20261106.md',PATCH+'/README.md')
        for name in FILES:archive.write(SOURCE/name,PATCH+'/files/'+name)
    print(json.dumps(dict(path=str(args.output),bytes=args.output.stat().st_size,
        sha256=hashlib.sha256(args.output.read_bytes()).hexdigest()),indent=2))

if __name__=='__main__':main()
