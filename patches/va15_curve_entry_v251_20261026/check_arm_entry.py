"""Local FK/contact sweep only; no camera, DDS, UDP or robot clients."""
import json
from pathlib import Path
import mujoco
import numpy as np
import yaml

root=Path('/home/dong/rl_sar_b2_z1/patches/va15_stop_feedback_20261020/work/b2_z1_no_gun')
cfg={}
for file in [root/'base.yaml',root/'VA15/config.yaml']:
    cfg.update(next(iter(yaml.safe_load(file.read_text()).values())))
m=mujoco.MjModel.from_xml_path('/home/dong/rl_sar_b2_z1_old/policy/b2_z1_no_gun/VA15/scene_flat.xml')
d=mujoco.MjData(m)
joint_ids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n) for n in cfg['joint_names']]
qids=[m.jnt_qposadr[j] for j in joint_ids]
site=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,'ee_site')
gun=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'welding_gun')
if gun>=0:m.geom_contype[gun]=m.geom_conaffinity[gun]=0
indices=[8,13,14,15,16,17]
start=np.array([0.,2.18,-1.6,-.9,0.,0.]);goal=np.array([0.,2.24,-1.27,-1.33,.04,0.])
bad=[];self_pairs=set();minimum_tcp=10.;samples=0
for height in [.50,.52,.56]:
    for pitch in [-.05,0.,.05]:
        for alpha in np.linspace(0,1,101):
            d.qpos[2]=height;d.qpos[3:7]=[np.cos(pitch/2),0,np.sin(pitch/2),0]
            for i,adr in enumerate(qids):d.qpos[adr]=cfg['deployment_stand_dof_pos'][i]
            for i,value in zip(indices,start*(1-alpha)+goal*alpha):
                d.qpos[qids[i]]=value
                if m.jnt_limited[joint_ids[i]] and not m.jnt_range[joint_ids[i],0]<=value<=m.jnt_range[joint_ids[i],1]:
                    bad.append((i,float(value)))
            mujoco.mj_forward(m,d);samples+=1
            minimum_tcp=min(minimum_tcp,float(d.site_xpos[site,2]))
            for c in d.contact:
                b1,b2=m.geom_bodyid[c.geom1],m.geom_bodyid[c.geom2]
                if b1 and b2 and c.dist<0:
                    self_pairs.add((mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,b1),mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,b2)))
passed=not bad and not self_pairs and minimum_tcp>.05
print(json.dumps(dict(pass_check=passed,samples=samples,start_arm=start.tolist(),goal_arm=goal.tolist(),
    minimum_virtual_tcp_height_m=minimum_tcp,joint_limit_violations=bad,self_contacts=sorted(self_pairs),
    gun_collision_disabled=True,existing_model_contacts_only=True,hardware_qualified=False),indent=2))
raise SystemExit(0 if passed else 2)
