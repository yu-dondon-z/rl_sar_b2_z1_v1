#!/usr/bin/env python3
"""Offline VA15 weight and configuration checks; no robot imports."""
import argparse
import hashlib
import json
from pathlib import Path
import yaml

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--repo-root', type=Path, default=Path(__file__).resolve().parents[1])
root=parser.parse_args().repo_root.resolve()
base=yaml.safe_load((root/'policy/b2_z1_no_gun/base.yaml').read_text())['b2_z1_no_gun']
folder=root/'policy/b2_z1_no_gun/VA15'
cfg=yaml.safe_load((folder/'config.yaml').read_text())['b2_z1_no_gun/VA15']
manifest=json.loads((folder/'manifest.json').read_text())
def require(value,label):
    if not value:raise SystemExit('VA15 check failed: '+label)
require(base['deployment_policy_profile']=='VA15','selected profile')
require(manifest['profile']=='VA15' and manifest['iteration']==5000,'checkpoint identity')
require(hashlib.sha256((folder/'policy.pt').read_bytes()).hexdigest()==manifest['model_sha256'],'policy hash')
require(hashlib.sha256((folder/'config.yaml').read_bytes()).hexdigest()==manifest['config_sha256'],'config hash')
require(hashlib.sha256((folder/'golden.json').read_bytes()).hexdigest()==manifest['golden_sha256'],'golden data hash')
require(cfg['num_observations']==70 and cfg['num_of_dofs']==18,'70/18 dimensions')
require(cfg['model_name']=='policy.pt','model filename')
require(base['dt']*base['decimation']==.02,'50 Hz policy')
require(cfg['action_scale']==[.25]*18 and cfg['action_filter_alpha']==.2,'action contract')
require(cfg['policy_input_clip']==5 and cfg['use_raw_policy_action_observation'],'raw last action')
require(not cfg['sync_applied_action_observation'],'last-action feedback')
require(cfg['observations']==['velocity_commands','target_commands','base_ang_vel','projected_gravity','joint_pos_rel','joint_vel_rel','actions'],'observation order')
require(cfg['ang_vel_scale']==.2 and cfg['dof_vel_scale']==.05 and cfg['dof_pos_scale']==1,'observation scale')
require(cfg['commands_scale']==[1]*3 and cfg['target_commands_scale']==[1]*7,'command scales')
require(base['b2_dds_mapping']==[3,0,9,6,4,1,10,7,-1,5,2,11,8,-1,-1,-1,-1,-1],'B2 routing')
require(base['z1_policy_indices']==[8,13,14,15,16,17],'Z1 routing')
require(cfg['default_dof_pos']==[.1,-.1,.1,-.1,.8,.8,1.,1.,0.,-1.5,-1.5,-1.5,-1.5,2.55,-1.6,-1.4,0.,0.],'training offsets')
require(cfg['deployment_stand_kp']==[450.]*8+[60.]+[675.]*4+[60.]*5,'separate stand stiffness')
require(cfg['deployment_stand_kd']==[15.]*8+[1.]+[15.]*4+[1.]*5,'separate stand damping')
require(cfg['deployment_stand_gain_ramp_s']==2. and cfg['deployment_stand_max_joint_error']<=.12,'stand ramp and unchanged error gate')
require(isinstance(cfg.get('real_foot_contact_calibration_validated'), bool) and
        isinstance(cfg.get('real_stop_sport_feedback_validated'), bool),'explicit real stop feedback validation state')
require(cfg['stop_min_base_height']==.48 and cfg['stop_max_body_tilt']==.15,'unchanged stop height/tilt gates')
require(cfg.get('real_kinematics_shadow_only') is True,'kinematics must remain diagnostics-only')
require(cfg.get('real_kinematics_model_file')=='b2_leg_kinematics.json','explicit profile-local model')
require(hashlib.sha256((folder/'b2_leg_kinematics.json').read_bytes()).hexdigest()==manifest['kinematics_model_sha256'],'kinematics geometry hash')
require(hashlib.sha256((folder/'kinematics_golden.json').read_bytes()).hexdigest()==manifest['kinematics_golden_sha256'],'kinematics reference hash')
require(manifest.get('kinematics_control_authorized') is False,'kinematics authorization boundary')
print('PASS: VA15 model_5000 weight/config hashes, 70/18 observations/actions, scale/EMA/raw history and joint routing')
