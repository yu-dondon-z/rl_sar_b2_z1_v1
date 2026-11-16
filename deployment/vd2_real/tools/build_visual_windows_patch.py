#!/usr/bin/env python3
"""Bundle pending command fixes plus GUI against the actual board baseline."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile
from build_command_reference_patch import FILES,SOURCE

PATCH='vd2_visual_windows_20261107'
EXTRA=('python/cameras.py','python/visual_preview.py','tools/launch_real.py',
       'tools/benchmark_display.py','tests/test_visual_preview.py','docs/VISUAL_WINDOWS_20261107.md')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    manifest=json.loads(args.baseline_manifest.read_text())
    if manifest.get('package')!='rl_sar_b2_z1_VD2':raise ValueError('wrong baseline package')
    sources={name:SOURCE/name for name in FILES+EXTRA}
    sources['python/vision/vd1_visual_front.py']=SOURCE.parents[1]/'scripts/vd1_visual_front.py'
    records={name:dict(before_sha256=manifest['files_sha256'].get(name),
                      after_sha256=hashlib.sha256(path.read_bytes()).hexdigest()) for name,path in sources.items()}
    update=dict(id=PATCH,package=manifest['package'],requires_cpp_rebuild=True,hardware_validation=False,
        description='front/rear diagnostic windows and pending command-reference fixes',
        backup_prefix='visual_windows',
        baseline_manifest_sha256=hashlib.sha256(args.baseline_manifest.read_bytes()).hexdigest(),files=records)
    with zipfile.ZipFile(args.output,'w',zipfile.ZIP_DEFLATED) as archive:
        archive.writestr(PATCH+'/PATCH.json',json.dumps(update,ensure_ascii=False,indent=2)+'\n')
        archive.write(SOURCE/'tools/install_feedback_fix.py',PATCH+'/install.py')
        archive.write(SOURCE/'docs/VISUAL_WINDOWS_20261107.md',PATCH+'/README.md')
        for name,path in sources.items():archive.write(path,PATCH+'/files/'+name)
    print(json.dumps(dict(path=str(args.output),bytes=args.output.stat().st_size,
        sha256=hashlib.sha256(args.output.read_bytes()).hexdigest()),indent=2))


if __name__=='__main__':main()
