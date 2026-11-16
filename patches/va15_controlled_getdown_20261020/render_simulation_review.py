#!/usr/bin/env python3
"""Render recorded native-test poses; rendering cannot affect the physics test."""
import argparse
import csv
import json
import os
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
import mujoco
import numpy as np
from PIL import Image,ImageDraw

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--scene',type=Path,default=Path('/home/dong/rl_sar_b2_z1_old/policy/b2_z1_no_gun/VA15/scene_flat.xml'))
    args=parser.parse_args()
    if args.output.exists():raise SystemExit('Refusing overwrite')
    rows=list(csv.DictReader(args.csv.open()))
    report=json.loads(Path(str(args.csv)+'.json').read_text())
    model=mujoco.MjModel.from_xml_path(str(args.scene))
    data=mujoco.MjData(model)
    for name in ('welding_gun','ee_tracked_tcp_visual'):
        g=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,name)
        if g>=0:model.geom_rgba[g,3]=0
    base=model.body('base_link').id
    selected=[('Before B',rows[min(len(rows)-1,int(report['walk_s']/0.02)+140)])]
    for stage,label in (('2','Caught stand'),('3','Settling stand')):
        matches=[r for r in rows if r['stop_stage']==stage]
        if matches:selected.append((label,matches[0]))
    down=[r for r in rows if r['state']=='RLFSMStateGetDown']
    if down:
        selected.extend([('Stow Z1',down[min(50,len(down)-1)]),('Lower B2',down[min(150,len(down)-1)])])
    selected.append(('Final '+('PASS' if report['success'] else 'NOT QUALIFIED'),rows[-1]))
    renderer=mujoco.Renderer(model,height=360,width=480)
    camera=mujoco.MjvCamera();mujoco.mjv_defaultCamera(camera)
    camera.distance=2.8;camera.azimuth=135;camera.elevation=-22
    panels=[]
    for title,row in selected:
        data.qpos[:]=[float(row[f'qpos_{i}']) for i in range(model.nq)]
        data.qvel[:]=0;mujoco.mj_forward(model,data)
        camera.lookat[:]=data.xpos[base]+[0,0,-.04]
        renderer.update_scene(data,camera=camera)
        panel=Image.new('RGB',(480,400),'white')
        panel.paste(Image.fromarray(renderer.render()),(0,40))
        draw=ImageDraw.Draw(panel)
        draw.text((10,5),f"{title} | t={float(row['time']):.2f}s",fill='black')
        draw.text((10,22),f"height={float(row['height']):.3f}m tilt={float(row['tilt'])*57.2958:.1f}deg",fill='black')
        panels.append(panel)
    renderer.close()
    montage=Image.new('RGB',(480*3,400*2),'white')
    for i,panel in enumerate(panels[:6]):montage.paste(panel,((i%3)*480,(i//3)*400))
    args.output.parent.mkdir(parents=True,exist_ok=True);montage.save(args.output)
    print(args.output)

if __name__=='__main__':main()
