#!/usr/bin/env python3
"""Install the supplied transform and preparation fix, preserving existing files."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import yaml
import numpy as np

MATRIX=np.array([
    [0.0,0.770435,-0.637519,-0.434510],
    [1.0,0.0,0.0,0.0],
    [0.0,-0.637519,-0.770435,0.026751],
    [0.0,0.0,0.0,1.0],
],dtype=float)

def validate(config):
    if config.get('camera_name')!='rear_d435i' or config.get('calibrated') is not True:
        raise ValueError('rear_d435i.yaml is not the supplied calibrated configuration')
    value=np.asarray(config.get('T_base_camera'),dtype=float)
    if value.shape!=(4,4) or not np.isfinite(value).all():raise ValueError('invalid T_base_camera shape/value')
    if not np.allclose(value,MATRIX,atol=1e-7):raise ValueError('unexpected rear-camera transform')
    if not np.allclose(value[3],[0,0,0,1],atol=1e-9):raise ValueError('invalid homogeneous row')
    if not np.allclose(value[:3,:3].T@value[:3,:3],np.eye(3),atol=2e-5):raise ValueError('rotation is not orthonormal')
    if not np.isclose(np.linalg.det(value[:3,:3]),1.,atol=2e-5):raise ValueError('rotation determinant is not +1')

def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target',type=Path,required=True,help='extracted rl_sar_b2_z1_VD2 directory')
    args=parser.parse_args();target=args.target.resolve()
    config_path=target/'config/rear_d435i.yaml'
    if not (target/'MANIFEST.json').is_file() or not config_path.is_file():
        raise SystemExit('target is not an extracted rl_sar_b2_z1_VD2 package')
    if not (target/'python/realsense_geometry.py').is_file():
        raise SystemExit('install vd2_camera_fix_20261105 before enabling this rear-camera configuration')
    config=yaml.safe_load(config_path.read_text())
    # Never overwrite a user-edited configuration silently.
    if config.get('calibrated') is True:
        validate(config)
    elif config.get('T_base_camera') is not None:
        raise SystemExit('existing rear transform differs; preserved without overwrite')
    payload=Path(__file__).resolve().parents[1]
    new_path=payload/'config/rear_d435i.yaml'
    new_config=yaml.safe_load(new_path.read_text());validate(new_config)
    # Keep an explicitly selected device rather than selecting another camera.
    new_config['serial']=config.get('serial','')
    manifest_path=target/'MANIFEST.json';manifest=json.loads(manifest_path.read_text())
    if manifest.get('package')!='rl_sar_b2_z1_VD2':raise SystemExit('unexpected target package')
    update=json.loads((payload/'PATCH.json').read_text())
    changes=[]
    for name,record in update['files'].items():
        relative=Path(name)
        if relative.is_absolute() or '..' in relative.parts:raise ValueError('invalid update path: '+name)
        source=payload/'files'/relative;destination=target/relative
        if sha256(source)!=record['after_sha256']:raise ValueError('update hash differs: '+name)
        actual=sha256(destination) if destination.is_file() else None
        if actual==record['after_sha256']:continue
        if actual!=record['before_sha256']:raise ValueError('local file differs; preserved without overwrite: '+name)
        changes.append((name,source,destination))
    if not changes and config==new_config:
        print('Deployment update already installed; no files changed.')
        return
    backup=target/'patch_backups'/('rear_extrinsics_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    backup.mkdir(parents=True,exist_ok=False)
    shutil.copy2(config_path,backup/'rear_d435i.yaml')
    shutil.copy2(manifest_path,backup/'MANIFEST.json')
    for name,source,destination in changes:
        if destination.exists():
            saved=backup/name;saved.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(destination,saved)
        destination.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,destination)
        manifest['files_sha256'][name]=sha256(destination)
    config_path.write_text(yaml.safe_dump(new_config,sort_keys=False))
    manifest['files_sha256']['config/rear_d435i.yaml']=sha256(config_path)
    manifest.setdefault('applied_updates',[]).append(dict(id=update['id'],backup=str(backup),
        calibration_quality='provisional_installation_estimate'))
    manifest['rear_extrinsics']='user_supplied_provisional_installation_estimate'
    manifest_path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    print('Installed provisional rear extrinsics; backup: '+str(backup))
    print('No C++ rebuild is required. Verify with: python run.py --offline-check')

if __name__=='__main__':main()
