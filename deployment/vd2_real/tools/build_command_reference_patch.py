#!/usr/bin/env python3
"""Package a reversible local candidate against an explicit board manifest."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile
SOURCE=Path(__file__).resolve().parents[1]
PATCH='vd2_command_reference_20261106'
FILES=('CMakeLists.txt','native/io.cpp','native/io.h','native/reply_paced_udp.h',
       'native/z1_controller_echo.cpp','native/z1_latency_check.cpp','native/z1_pacing_check.cpp',
       'python/native_io.py','python/control.py','run.py',
       'tests/test_stop_arm_reference.py','tests/test_front_pruning.py',
       'tools/check_z1_command_latency.py','tools/replay_stop_reference.py','tools/benchmark_front_components.py',
       'README_板载编译与运行.md','docs/COMMAND_REFERENCE_FIX_20261106.md')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    manifest=json.loads(args.baseline_manifest.read_text())
    if manifest.get('package')!='rl_sar_b2_z1_VD2':raise ValueError('wrong baseline package')
    sources={name:SOURCE/name for name in FILES}
    sources['python/vision/vd1_visual_front.py']=SOURCE.parents[1]/'scripts/vd1_visual_front.py'
    records={name:dict(before_sha256=manifest['files_sha256'].get(name),
                      after_sha256=hashlib.sha256(path.read_bytes()).hexdigest()) for name,path in sources.items()}
    update=dict(id=PATCH,package=manifest['package'],requires_cpp_rebuild=True,hardware_validation=False,
        description='reply-paced Z1 commands, latched braking arm reference and exact front component pruning',
        backup_prefix='command_reference',
        baseline_manifest_sha256=hashlib.sha256(args.baseline_manifest.read_bytes()).hexdigest(),files=records)
    with zipfile.ZipFile(args.output,'w',zipfile.ZIP_DEFLATED) as archive:
        archive.writestr(PATCH+'/PATCH.json',json.dumps(update,ensure_ascii=False,indent=2)+'\n')
        archive.write(SOURCE/'tools/install_feedback_fix.py',PATCH+'/install.py')
        archive.write(SOURCE/'docs/COMMAND_REFERENCE_FIX_20261106.md',PATCH+'/README.md')
        for name,path in sources.items():archive.write(path,PATCH+'/files/'+name)
    print(json.dumps(dict(path=str(args.output),bytes=args.output.stat().st_size,
        sha256=hashlib.sha256(args.output.read_bytes()).hexdigest()),indent=2))

if __name__=='__main__':main()
