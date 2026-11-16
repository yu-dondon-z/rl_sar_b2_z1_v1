#!/usr/bin/env python3
"""Package front path estimation plus previously verified CPU deadline changes."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile
from build_vision_deadline_patch import FILES as DEADLINE_FILES
SOURCE=Path(__file__).resolve().parents[1]
DEPLOY=SOURCE.parents[1]
PATCH='vd2_front_path_20261106'
EXTRA=('config/deployment.yaml','python/front_projection.py','tests/test_front_estimation.py',
       'tools/replay_front_path.py','docs/FRONT_PATH_BOARD_20261106.md')
SHARED=('vd1_front_path_estimator.py','vd1_visual_guard.py')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    baseline=json.loads(args.manifest.read_text())
    sources={name:SOURCE/name for name in dict.fromkeys(DEADLINE_FILES+EXTRA)}
    sources.update({'python/vision/'+name:DEPLOY/'scripts'/name for name in SHARED})
    records={name:dict(before_sha256=baseline['files_sha256'].get(name),
        after_sha256=hashlib.sha256(path.read_bytes()).hexdigest()) for name,path in sources.items()}
    update=dict(id=PATCH,package='rl_sar_b2_z1_VD2',requires_cpp_rebuild=False,hardware_validation=False,
        description='front-only path estimation and CPU deadline update',backup_prefix='front_path',files=records)
    with zipfile.ZipFile(args.output,'w',zipfile.ZIP_DEFLATED) as archive:
        archive.writestr(PATCH+'/PATCH.json',json.dumps(update,ensure_ascii=False,indent=2)+'\n')
        archive.write(SOURCE/'tools/install_feedback_fix.py',PATCH+'/install.py')
        for name,path in sources.items():archive.write(path,PATCH+'/files/'+name)
    print(json.dumps(dict(archive=str(args.output),sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),
                          files=len(sources),bytes=args.output.stat().st_size),indent=2))


if __name__=='__main__':main()
