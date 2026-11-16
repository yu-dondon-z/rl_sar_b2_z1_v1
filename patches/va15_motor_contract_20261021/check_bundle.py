#!/usr/bin/env python3
"""Verify the separate build without any hardware or SDK initialization."""
import hashlib
import json
from pathlib import Path
import subprocess

root=Path(__file__).resolve().parent
bundle=json.loads((root/'native_build_v2/bundle.json').read_text())
if bundle['hardware_qualified'] or bundle['production_replaced']:
    raise SystemExit('Unexpected bundle qualification metadata')
for name,expected in bundle['files'].items():
    path=Path(name)
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
        raise SystemExit('Bundle mismatch: '+name)
repo=root.parents[1]
subprocess.run(['python3',str(repo/'scripts/check_va15_legacy.py')],check=True)
subprocess.run([str(root/'native_build_v2/va15_forward_trial'),'--offline-check'],check=True)
print('PASS separate trial bundle; original validated flags unchanged; NOT a real motion acceptance')
