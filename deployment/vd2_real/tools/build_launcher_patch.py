#!/usr/bin/env python3
"""Package only the new one-terminal entry and its offline tests."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

SOURCE=Path(__file__).resolve().parents[1]
PATCH='vd2_one_terminal_20261106'
FILES=('start_real.sh','tools/launch_real.py','tests/test_one_terminal.py','docs/ONE_TERMINAL_20261106.md')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--original',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError('preserve existing update: '+str(args.output))
    with zipfile.ZipFile(args.original) as original:
        manifest=json.loads(original.read('rl_sar_b2_z1_VD2/MANIFEST.json'))
    update=dict(id=PATCH,package='rl_sar_b2_z1_VD2',requires_cpp_rebuild=False,hardware_validation=False,
        files={name:dict(before_sha256=manifest['files_sha256'].get(name),
            after_sha256=hashlib.sha256((SOURCE/name).read_bytes()).hexdigest()) for name in FILES})
    with zipfile.ZipFile(args.output,'w',zipfile.ZIP_DEFLATED) as archive:
        archive.writestr(PATCH+'/PATCH.json',json.dumps(update,ensure_ascii=False,indent=2)+'\n')
        archive.write(SOURCE/'tools/install_launcher.py',PATCH+'/install.py')
        archive.write(SOURCE/'docs/ONE_TERMINAL_20261106.md',PATCH+'/README.md')
        for name in FILES:archive.write(SOURCE/name,PATCH+'/files/'+name)
    print(json.dumps(dict(archive=str(args.output),bytes=args.output.stat().st_size,
        sha256=hashlib.sha256(args.output.read_bytes()).hexdigest()),indent=2))


if __name__=='__main__':main()
