#!/usr/bin/env python3
"""Create a relocatable source bundle; no robot access and no existing file deletion."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import zipfile

SOURCE=Path(__file__).resolve().parents[1]
DEPLOY=SOURCE.parents[1]
TRAIN=Path('/home/dong/b2_z1')
VENDOR=DEPLOY/'src/rl_sar/library/thirdparty'

def copy(source,target):
    target.parent.mkdir(parents=True,exist_ok=True)
    if source.is_dir():
        shutil.copytree(source,target,dirs_exist_ok=True,
            ignore=shutil.ignore_patterns('__pycache__','*.pyc','.git','build','cmake_build','*.egg-info'))
    else:shutil.copy2(source,target)

def stage(parent):
    root=parent/'rl_sar_b2_z1_VD2';copy(SOURCE,root)
    for name in ('vd1_control.py','dual_camera_perception.py'):
        copy(TRAIN/'scripts/vision'/name,root/'python/vision'/name)
    copy(TRAIN/'source/b2_z1/b2_z1/tasks/manager_based/b2_z1/mdp/commands/dual_vision_kinematics.py',root/'python/dual_vision_kinematics.py')
    for name in ('vd1_visual_front.py','vd1_visual_rear.py','vd1_visual_guard.py','vd1_visual_path.py',
                 'vd1_front_path_estimator.py'):
        copy(DEPLOY/'scripts'/name,root/'python/vision'/name)
    copy(VENDOR/'robot_sdk/unitree/unitree_sdk2',root/'thirdparty/unitree_sdk2')
    copy(VENDOR/'z1_sdk/include',root/'thirdparty/z1_sdk/include')
    for arch in ('x86_64','aarch64'):
        copy(VENDOR/('z1_sdk/lib/libZ1_SDK_'+arch+'.so'),root/('thirdparty/z1_sdk/lib/libZ1_SDK_'+arch+'.so'))
    for name in ('README.md','LICENSE'):
        for vendor,destination in ((VENDOR/'z1_sdk',root/'thirdparty/z1_sdk'),):
            if (vendor/name).is_file():copy(vendor/name,destination/name)
    rha=Path('/home/dong/RHACrackNet/code')
    for name in ('crack_inference','crack_trajectory','models/RHANet_dep','models/RHANet'):
        copy(rha/name,root/'thirdparty/rhacracknet'/name)
    for name in ('LICENSE','README.md'):
        if (rha.parent/name).is_file():copy(rha.parent/name,root/'thirdparty/rhacracknet'/name)
    # Preserve deployment's exact full-resolution thinning optimization when
    # assembling a future full bundle from the original vendor tree.
    copy(SOURCE/'thirdparty/rhacracknet/crack_trajectory/centerline.py',
         root/'thirdparty/rhacracknet/crack_trajectory/centerline.py')
    copy(DEPLOY/'policy/b2_z1_no_gun/mujoco_vd3_mirror_2000/policy.pt',root/'assets/policy.pt')
    copy(DEPLOY/'policy/b2_z1_no_gun/mujoco_vd3_mirror_2000/provenance.json',root/'assets/training_provenance.json')
    copy(rha/'exps/rhanet_dep_CamCrack789_b2_stage1/ckpt/best_ckpt.pth',root/'assets/rhanet.pth')
    copy(Path('/home/dong/b2_z1_VD2/source/b2_z1/b2_z1/assets/b2_z1/b2_z1_vd3_zero_gun.urdf'),root/'assets/robot.urdf')
    copy(TRAIN/'source/b2_z1/b2_z1/assets/b2_z1/b2_z1_vd1.urdf',root/'assets/encoder_fk.urdf')
    # Small actor-only golden fixture from the accepted simulation. No simulator
    # state/ground-truth geometry is delivered to, or consumed by, the controller.
    import gzip
    import numpy as np
    trace=DEPLOY/'outputs/VD3_MIRROR_2000_DEPLOYMENT_CHECK_20261105/session_20261105_160439_unmklgfs/actor_trace.jsonl.gz'
    selected=[]
    with gzip.open(trace,'rt') as handle:
        for i,line in enumerate(handle):
            if i in (0,50,250,500,1000,1500,2000,2500,2999):
                frame=json.loads(line);selected.append((frame['observation'],frame['action']))
    np.savez(root/'assets/actor_golden.npz',observations=np.array([a for a,b in selected],dtype=np.float32),
             actions=np.array([b for a,b in selected],dtype=np.float32))
    # Historical front calibration is specific to the unchanged robot. Do not
    # import its former fixed floor-height fallback into the RGB-D pipeline.
    import yaml
    old=yaml.safe_load((DEPLOY/'patches/va15_tcp_priority_v256_20261028/config/b2_builtin_front_camera.yaml').read_text())
    old.pop('road_plane_base',None)
    old['source']='existing robot calibration 2026-09-15; verify unchanged mounting and resolution'
    (root/'config/front_rgb.yaml').write_text(yaml.safe_dump(old,sort_keys=False))
    files={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
           for p in sorted(root.rglob('*')) if p.is_file() and p.name!='MANIFEST.json'
           and not any(part in ('build','runs','__pycache__') for part in p.relative_to(root).parts)}
    manifest=dict(package='rl_sar_b2_z1_VD2',date='2026-11-05',
        actor='VD3 mirror model_2000',hardware_validation=False,rear_extrinsics='installed_not_calibrated',
        no_logs_or_old_worktrees_included=True,files_sha256=files)
    (root/'MANIFEST.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n')
    return root

def archive(root,destination):
    if destination.exists():raise FileExistsError(str(destination)+' already exists; choose a new output to preserve it')
    with zipfile.ZipFile(destination,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as bundle:
        for path in sorted(root.rglob('*')):
            if path.is_file() and not any(part in ('build','__pycache__','runs') for part in path.relative_to(root).parts):
                bundle.write(path,Path(root.name)/path.relative_to(root))
    return dict(archive=str(destination),bytes=destination.stat().st_size,
                sha256=hashlib.sha256(destination.read_bytes()).hexdigest())

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage-parent',type=Path,help='new persistent staging directory')
    parser.add_argument('--archive',type=Path)
    args=parser.parse_args()
    if args.stage_parent:
        args.stage_parent.mkdir(parents=True,exist_ok=False)
        root=stage(args.stage_parent)
        print(root)
        if args.archive:print(json.dumps(archive(root,args.archive),indent=2))
    else:
        if not args.archive:parser.error('specify --stage-parent or --archive')
        with tempfile.TemporaryDirectory(prefix='vd2_bundle_') as temp:
            print(json.dumps(archive(stage(Path(temp)),args.archive),indent=2))

if __name__=='__main__':main()
