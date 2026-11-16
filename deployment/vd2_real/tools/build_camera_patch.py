#!/usr/bin/env python3
"""Build the small camera update against the original immutable VD2 ZIP."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

SOURCE=Path(__file__).resolve().parents[1]
PATCH_ID='vd2_camera_fix_20261105'
FILES=('python/realsense_geometry.py','python/cameras.py','run.py',
       'tools/calibrate_rear.py','tests/test_realsense_geometry.py',
       'docs/CAMERA_FIX_20261105.md')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--original',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError('preserve existing update: '+str(args.output))
    with zipfile.ZipFile(args.original) as original:
        manifest=json.loads(original.read('rl_sar_b2_z1_VD2/MANIFEST.json'))
    records={name:dict(before_sha256=manifest['files_sha256'].get(name),
                      after_sha256=hashlib.sha256((SOURCE/name).read_bytes()).hexdigest())
             for name in FILES}
    update=dict(id=PATCH_ID,package='rl_sar_b2_z1_VD2',files=records,
                hardware_validation=False,requires_cpp_rebuild=False)
    with zipfile.ZipFile(args.output,'w',zipfile.ZIP_DEFLATED) as archive:
        archive.writestr(PATCH_ID+'/PATCH.json',json.dumps(update,ensure_ascii=False,indent=2)+'\n')
        archive.write(SOURCE/'tools/apply_camera_fix.py',PATCH_ID+'/install_patch.py')
        archive.write(SOURCE/'docs/CAMERA_FIX_20261105.md',PATCH_ID+'/README.md')
        for name in FILES:archive.write(SOURCE/name,PATCH_ID+'/files/'+name)
    print(json.dumps(dict(archive=str(args.output),bytes=args.output.stat().st_size,
        sha256=hashlib.sha256(args.output.read_bytes()).hexdigest()),indent=2))


if __name__=='__main__':main()
