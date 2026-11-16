#!/usr/bin/env python3
"""Build the final deployment update on top of the camera-fixed VD2 package."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

SOURCE=Path(__file__).resolve().parents[1]
PATCH='vd2_deploy_ready_20261105'
FILES=('python/control.py','run.py','tests/test_real_start_targets.py','docs/REAR_EXTRINSICS_20261105.md')

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--original',type=Path,required=True);p.add_argument('--camera-fix',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(str(a.output)+' already exists')
    with zipfile.ZipFile(a.original) as z:manifest=json.loads(z.read('rl_sar_b2_z1_VD2/MANIFEST.json'))
    with zipfile.ZipFile(a.camera_fix) as z:camera=json.loads(z.read('vd2_camera_fix_20261105/PATCH.json'))
    expected=manifest['files_sha256'].copy()
    expected.update({name:record['after_sha256'] for name,record in camera['files'].items()})
    config=SOURCE/'config/rear_d435i.yaml'
    records={name:dict(before_sha256=expected.get(name),after_sha256=hashlib.sha256((SOURCE/name).read_bytes()).hexdigest()) for name in FILES}
    update=dict(id=PATCH,package='rl_sar_b2_z1_VD2',files=records,
                requires='vd2_camera_fix_20261105',hardware_validation=False,requires_cpp_rebuild=False)
    readme=SOURCE/'docs/REAR_EXTRINSICS_20261105.md'
    installer=SOURCE/'tools/install_rear_extrinsics.py'
    with zipfile.ZipFile(a.output,'w',zipfile.ZIP_DEFLATED) as z:
        z.writestr(PATCH+'/PATCH.json',json.dumps(update,ensure_ascii=False,indent=2)+'\n')
        z.write(installer,PATCH+'/tools/install_extrinsics.py');z.write(readme,PATCH+'/README.md');z.write(config,PATCH+'/config/rear_d435i.yaml')
        for name in FILES:z.write(SOURCE/name,PATCH+'/files/'+name)
    print(json.dumps(dict(archive=str(a.output),bytes=a.output.stat().st_size,sha256=hashlib.sha256(a.output.read_bytes()).hexdigest()),indent=2))

if __name__=='__main__':main()
