"""Actual SDK offline probe + syscall evidence + denial negative controls."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import signal
import subprocess


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--executable',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();exe=args.executable.resolve()
    args.output.mkdir(parents=True,exist_ok=False)
    library=Path('/home/dong/rl_sar_b2_z1/src/rl_sar/library/thirdparty/z1_sdk/lib/libZ1_SDK_x86_64.so')
    sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
    expected='ecfbdd917016de721983ad2cc0770adb110832f506cc9ef67b8c37a9f5f3eafd'
    if sha(library)!=expected:raise RuntimeError('SDK fingerprint changed')
    trace=args.output/'syscalls.log'
    result=subprocess.run(['strace','-f','-o',str(trace),'-e',
        'trace=network,clone,clone3,fork,vfork',str(exe)],capture_output=True,text=True,timeout=5)
    (args.output/'stdout.log').write_text(result.stdout+result.stderr)
    forbidden=re.findall(r'\b(?:socket|socketpair|connect|bind|listen|accept4?|sendto|recvfrom|sendmsg|recvmsg|sendmmsg|recvmmsg|shutdown|setsockopt|getsockopt|clone3?|fork|vfork)\(',trace.read_text())
    negatives={}
    for name in ['network','process']:
        trial=subprocess.run([str(exe),'--negative-'+name],capture_output=True,text=True,timeout=5)
        negatives[name]=dict(returncode=trial.returncode,expected_sigsys=trial.returncode==-signal.SIGSYS)
    passed=result.returncode==0 and not forbidden and all(n['expected_sigsys'] for n in negatives.values())
    report=dict(passed=passed,actual_sdk=True,synthetic_memory_fsm_acks=True,
                hardware_qualified=False,production_integration=False,sdk_sha256=expected,
                executable_sha256=sha(exe),source_sha256=sha(Path(__file__).with_name('actual_z1_no_io.cpp')),
                returncode=result.returncode,forbidden_syscall_attempts=forbidden,negative_controls=negatives)
    (args.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));return 0 if passed else 2


if __name__=='__main__':raise SystemExit(main())
