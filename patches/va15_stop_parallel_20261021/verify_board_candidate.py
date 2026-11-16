#!/usr/bin/env python3
"""Compile/run memory-only stop tests using the board's native build flags.

No Unitree Python import, control launcher, DDS/UDP or robot process is run.
"""
import argparse
import json
from pathlib import Path
import shlex
import subprocess

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repo',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--test-source',type=Path,required=True)
    args=p.parse_args();repo=args.repo.resolve();out=args.output.resolve()
    out.mkdir(parents=True,exist_ok=False)
    build=repo/'cmake_build_va15'
    native=build/'CMakeFiles/rl_real_b2_z1_no_gun.dir'
    flags={}
    for line in (native/'flags.make').read_text().splitlines():
        if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):
            key,value=line.split('=',1);flags[key.strip()]=shlex.split(value)
    link=shlex.split((native/'link.txt').read_text())
    compiler=link[0]
    source=args.test_source.resolve()
    if not source.is_file() or not source.name.startswith('test_'):
        raise RuntimeError('Expected an explicit offline test source')
    obj=out/'offline_test.o';exe=out/source.stem
    compile_command=[compiler,*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],
        '-UNDEBUG','-c',str(source),'-o',str(obj)]
    compile_command=[x for x in compile_command if x!='-DNDEBUG']
    link_command=list(link)
    objects=[i for i,v in enumerate(link_command) if v.endswith('/rl_real_b2_z1_no_gun.cpp.o')]
    if len(objects)!=1:raise RuntimeError('Unexpected native main object layout')
    link_command[objects[0]]=str(obj)
    link_command[link_command.index('-o')+1]=str(exe)
    transcript=dict(compile=compile_command,link=link_command,no_robot_clients=True)
    (out/'commands.json').write_text(json.dumps(transcript,indent=2)+'\n')
    with (out/'compile.log').open('w') as log:
        subprocess.run(compile_command,cwd=build,stdout=log,stderr=subprocess.STDOUT,check=True)
        subprocess.run(link_command,cwd=build,stdout=log,stderr=subprocess.STDOUT,check=True)
    with (out/'test.log').open('w') as log:
        subprocess.run([str(exe)],cwd=repo,stdout=log,stderr=subprocess.STDOUT,check=True)
    print((out/'test.log').read_text())

if __name__=='__main__':main()
