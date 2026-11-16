#!/usr/bin/env python3
"""Install this camera-only update into an extracted VD2 bundle, with backups."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def install(payload,target):
    payload=Path(payload).resolve();target=Path(target).resolve()
    update=json.loads((payload/'PATCH.json').read_text())
    manifest_path=target/'MANIFEST.json'
    manifest=json.loads(manifest_path.read_text())
    if manifest.get('package')!='rl_sar_b2_z1_VD2':
        raise ValueError('target is not an extracted rl_sar_b2_z1_VD2 bundle')
    changes=[]
    for name,record in update['files'].items():
        relative=Path(name)
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('invalid update path: '+name)
        source=payload/'files'/relative;destination=target/relative
        try:destination.resolve().relative_to(target)
        except ValueError:
            raise ValueError('update destination escapes the target: '+name)
        if digest(source)!=record['after_sha256']:
            raise ValueError('update file hash differs: '+name)
        actual=digest(destination)
        if actual==record['after_sha256']:
            continue
        if actual!=record['before_sha256']:
            raise ValueError('local file differs; preserved without overwrite: '+name)
        changes.append((name,source,destination))
    if not changes:
        print('Camera fix already installed; no files changed.')
        return None
    backup=target/'patch_backups'/('camera_fix_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    backup.mkdir(parents=True,exist_ok=False)
    shutil.copy2(manifest_path,backup/'MANIFEST.json')
    for name,source,destination in changes:
        if destination.exists():
            saved=backup/name;saved.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(destination,saved)
        destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(source,destination)
    for name,record in update['files'].items():
        manifest['files_sha256'][name]=record['after_sha256']
    manifest.setdefault('applied_updates',[]).append(dict(id=update['id'],backup=str(backup)))
    manifest_path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    (backup/'patch_record.json').write_text(json.dumps(update,ensure_ascii=False,indent=2)+'\n')
    print('Installed '+update['id']+'; original files and manifest saved in '+str(backup))
    print('No robot IO initialized. No C++ rebuild is required.')
    return backup


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target',required=True,type=Path)
    args=parser.parse_args()
    install(Path(__file__).resolve().parent,args.target)


if __name__=='__main__':main()
