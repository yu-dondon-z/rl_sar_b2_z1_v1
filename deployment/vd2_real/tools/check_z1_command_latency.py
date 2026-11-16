#!/usr/bin/env python3
"""Exercise actual Z1 SDK/controller libraries on new loopback ports; no hardware IO."""
import argparse
import json
from pathlib import Path
import platform
import socket
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller-root',type=Path,required=True)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args();controller=args.controller_root.resolve();arch=platform.machine()
    if args.output is not None and args.output.exists():raise FileExistsError(args.output)
    with tempfile.TemporaryDirectory(prefix='vd2_z1_loopback_') as directory:
        directory=Path(directory);echo=directory/'echo';client=directory/'client'
        for source,output,include,lib in (
            ('z1_controller_echo.cpp',echo,controller/'include',controller/('lib/libZ1_'+arch+'.so')),
            ('z1_latency_check.cpp',client,ROOT/'thirdparty/z1_sdk/include',ROOT/('thirdparty/z1_sdk/lib/libZ1_SDK_'+arch+'.so'))):
            subprocess.run(['c++','-std=c++17','-O2','-pthread','-I/usr/include/eigen3','-I'+str(include),
                str(ROOT/'native'/source),str(lib),'-Wl,-rpath,'+str(lib.parent),'-o',str(output)],check=True)
        results=[]
        for paced in (False,True):
            # Allocate distinct unused unprivileged ports. Never use the real 8071/8072 pair.
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as a,socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as b:
                a.bind(('127.0.0.1',0));b.bind(('127.0.0.1',0));ap=a.getsockname()[1];bp=b.getsockname()[1]
            process=subprocess.Popen([str(echo),str(ap),str(bp),'.0022'],stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT,text=True)
            try:
                while True:
                    line=process.stdout.readline()
                    if not line:raise RuntimeError('loopback controller did not start')
                    if 'loopback controller SDK ready' in line:break
                output=subprocess.check_output([str(client),str(bp),str(ap),str(int(paced))],text=True,timeout=8)
                result=json.loads(next(line for line in output.splitlines() if line.startswith('{')))
                results.append(result);print(json.dumps(result),flush=True)
            finally:
                process.terminate();process.communicate(timeout=3)
        if results[1]['p95_ms']>=20 or results[1]['max_ms']>=40:
            raise RuntimeError('reply-paced loopback latency exceeded the offline budget')
        if results[0]['p95_ms']<30:raise RuntimeError('baseline FIFO backlog was not reproduced')
        if args.output:args.output.write_text(json.dumps(dict(offline_only=True,hardware_io_opened=False,
            actual_controller_sdk_library=str(controller/('lib/libZ1_'+arch+'.so')),
            controller_period_s=.0022,producer_period_s=.002,results=results),indent=2)+'\n')
        print('PASS: actual controller SDK receiver; loopback only; no robot stability/accuracy claim')

if __name__=='__main__':main()
