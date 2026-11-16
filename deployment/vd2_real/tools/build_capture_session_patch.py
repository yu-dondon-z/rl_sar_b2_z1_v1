#!/usr/bin/env python3
"""Preserve sudo terminal-cache identity for passive capture only."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

SOURCE=Path(__file__).resolve().parents[1]
PATCH='vd2_capture_session_20261106'
FILES=('tools/launch_real.py','tests/test_one_terminal.py',
       'README_板载编译与运行.md','docs/CAPTURE_SESSION_20261106.md')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    baseline=json.loads(args.manifest.read_text())
    sources={name:SOURCE/name for name in FILES}
    records={name:dict(before_sha256=baseline['files_sha256'].get(name),
        after_sha256=hashlib.sha256(path.read_bytes()).hexdigest()) for name,path in sources.items()}
    update=dict(id=PATCH,package='rl_sar_b2_z1_VD2',requires_cpp_rebuild=False,hardware_validation=False,
        description='preserve terminal session for sudo capture',backup_prefix='capture_session',files=records)
    with zipfile.ZipFile(args.output,'w',zipfile.ZIP_DEFLATED) as archive:
        archive.writestr(PATCH+'/PATCH.json',json.dumps(update,ensure_ascii=False,indent=2)+'\n')
        archive.write(SOURCE/'tools/install_feedback_fix.py',PATCH+'/install.py')
        for name,path in sources.items():archive.write(path,PATCH+'/files/'+name)
    print(json.dumps(dict(archive=str(args.output),sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),
        files=len(sources),bytes=args.output.stat().st_size),indent=2))


if __name__=='__main__':main()
