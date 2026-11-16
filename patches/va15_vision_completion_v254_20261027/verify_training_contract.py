"""Execute ONLY the allocation prefix from VA15's archived training source.

No Isaac Sim, camera, SDK, model inference, DDS/UDP or robot is initialized.
The AST prefix is checked explicitly before compilation; imports are excluded.
"""
import ast
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import tarfile

import torch
import yaml


def main():
    log=Path('/home/dong/b2_z1/logs/rsl_rl/b2_z1_amp/2026-09-28_15-07-31_va15_curriculum_from2500')
    archive=log/'source.tar.gz'
    entry='source/b2_z1/b2_z1/tasks/manager_based/b2_z1/mdp/commands/vision_trajectory_command.py'
    with tarfile.open(archive,'r:gz') as source:
        stream=source.extractfile(entry)
        if stream is None:
            raise RuntimeError('Training source missing')
        code=stream.read()
    tree=ast.parse(code)
    matches=[node for node in ast.walk(tree) if isinstance(node,ast.FunctionDef) and node.name=='_set_pose_command_b']
    if len(matches)!=1:
        raise RuntimeError('Unexpected source layout')
    body=matches[0].body[:4]
    if len(body)!=4 or not isinstance(body[0],ast.Assign) or any(not isinstance(n,ast.Expr) for n in body[1:]):
        raise RuntimeError('Unexpected training allocation statements')
    for i,name in enumerate(['pos_x','pos_y','pos_z']):
        call=body[i+1].value
        attributes=[n.attr for n in ast.walk(call) if isinstance(n,ast.Attribute)]
        if not isinstance(call,ast.Call) or not isinstance(call.func,ast.Attribute) or call.func.attr!='clamp_' or name not in attributes:
            raise RuntimeError('Training contract changed')
    function=ast.FunctionDef(name='archived_allocate',args=matches[0].args,
        body=body+[ast.Return(value=ast.Name(id='position_b',ctx=ast.Load()))],decorator_list=[],returns=None,type_comment=None)
    module=ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[]))
    namespace={'torch':torch}
    exec(compile(module,'audited_archived_allocation','exec'),namespace)
    params=yaml.load((log/'params/env.yaml').read_text(),Loader=yaml.BaseLoader)
    limits=params['commands']['ee_target']['limit_ranges']
    bounds={k:tuple(map(float,limits[k])) for k in ['pos_x','pos_y','pos_z']}
    assert bounds=={'pos_x':(-1.,-.55),'pos_y':(-.15,.15),'pos_z':(-.55,-.30)}
    config=SimpleNamespace(cfg=SimpleNamespace(limit_ranges=SimpleNamespace(**bounds)))
    original=torch.tensor([[-.775,.24,-.35],[-.8,-.22,-.6],[-.7,.02,-.42]],dtype=torch.float64)
    result=namespace['archived_allocate'](config,original)
    expected=torch.tensor([[-.775,.15,-.35],[-.8,-.15,-.55],[-.7,.02,-.42]],dtype=torch.float64)
    assert torch.equal(result,expected)
    assert torch.equal(original[0],torch.tensor([-.775,.24,-.35],dtype=torch.float64))
    print(json.dumps(dict(pass_training_contract=True,
        archive=str(archive),archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
        entry=entry,source_sha256=hashlib.sha256(code).hexdigest(),bounds=bounds,
        archived_method_executed='allocation prefix only',hardware_initialized=False),indent=2))


if __name__=='__main__':
    main()
