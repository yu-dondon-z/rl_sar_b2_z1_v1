#!/usr/bin/env python3
"""Build isolated main against existing native core; run offline checks ONLY."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repo',type=Path,required=True)
    p.add_argument('--output',default='native_build_v2')
    args=p.parse_args();repo=args.repo.resolve();root=Path(__file__).resolve().parent
    if Path(args.output).name!=args.output:raise RuntimeError('Output must be one new local directory name')
    out=root/args.output;out.mkdir(exist_ok=False)
    build=repo/'cmake_build_va15'
    native=build/'CMakeFiles/rl_real_b2_z1_no_gun.dir'
    flags={}
    for line in (native/'flags.make').read_text().splitlines():
        if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):
            key,value=line.split('=',1);flags[key.strip()]=shlex.split(value)
    link=shlex.split((native/'link.txt').read_text());compiler=link[0]
    source=root/'production/rl_real_b2_z1_no_gun.cpp'
    tracked=list((root/'production').glob('*'))
    tracked += [repo/'policy/b2_z1_no_gun/base.yaml',repo/'policy/b2_z1_no_gun/VA15/config.yaml',
                repo/'policy/b2_z1_no_gun/VA15/policy.pt',repo/'scripts/check_va15_legacy.py']
    tracked += list((repo/'src/rl_sar/library/core/rl_sdk').glob('va15*.hpp'))
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    before={str(p):digest(p) for p in tracked if p.is_file()}
    obj=out/'candidate_main.o';exe=out/'va15_forward_trial'
    command=[compiler,*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],
             '-I/usr/include/eigen3','-UNDEBUG','-c',str(source),'-o',str(obj)]
    command=[x for x in command if x!='-DNDEBUG']
    objects=[i for i,v in enumerate(link) if v.endswith('/rl_real_b2_z1_no_gun.cpp.o')]
    if len(objects)!=1:raise RuntimeError('Unexpected native object layout')
    link[objects[0]]=str(obj);link[link.index('-o')+1]=str(exe)
    (out/'commands.json').write_text(json.dumps(dict(compile=command,link=link),indent=2)+'\n')
    with (out/'compile.log').open('w') as log:
        subprocess.run(command,cwd=build,stdout=log,stderr=subprocess.STDOUT,check=True)
        subprocess.run(link,cwd=build,stdout=log,stderr=subprocess.STDOUT,check=True)
    with (out/'offline_check.log').open('w') as log:
        subprocess.run([str(exe),'--offline-check'],cwd=repo,stdout=log,stderr=subprocess.STDOUT,check=True)
    if any(digest(Path(p))!=h for p,h in before.items()):raise RuntimeError('Inputs changed during build')
    before[str(exe)]=digest(exe)
    (out/'bundle.json').write_text(json.dumps(dict(schema='isolated_trial_bundle_v1',files=before,
        hardware_qualified=False,production_replaced=False),indent=2)+'\n')
    print((out/'offline_check.log').read_text())
    print('Separate candidate compiled; no DDS/UDP/robot initialized. Production binary unchanged.')

if __name__=='__main__':main()
