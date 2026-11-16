"""Read-only MuJoCo pose adapter for the shared VD12 geometric keep-out rule.

The caller must update actual body-link poses with MuJoCo before calling
``evaluate(data)``. This module does not call mj_forward/mj_step, infer frame
freshness, compute contact force, change an actor input, or enforce deployment.
Negative capsule-envelope lower bounds mean a conservative exclusion violation,
NOT a proven physical collision or penetration depth.
"""

from __future__ import annotations

import hashlib
import importlib.util
import math
from pathlib import Path

import numpy as np
import torch


TRAINING_ROOT = Path('/home/dong/b2_z1_VD2')
ASSETS = TRAINING_ROOT / 'source/b2_z1/b2_z1/assets'
URDF = ASSETS / 'b2_z1/b2_z1_vd10_hollow_gun.urdf'


def load_pure_module(name, path):
    """Load a standalone source; never import the training/Isaac package."""
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError('Shared VD12 source is unavailable: ' + str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def clearance_decision(lower_bound_m, input_valid, profile):
    """One-tick decision/cost, identical to GeometryWindow.sample's rule.

    No four-substep episode latch is implied. The training window/termination
    clock must be handled by an explicitly integrated caller, not fabricated.
    """
    valid = bool(input_valid) and math.isfinite(float(lower_bound_m))
    violated = (not valid) or float(lower_bound_m) < profile['hard_threshold_m']
    cost = (min(profile['cost_cap'],
                max((profile['soft_margin_m'] - float(lower_bound_m)) / profile['soft_margin_m'], 0.) ** 2)
            if valid else profile['cost_cap'])
    return dict(input_valid=valid, conservative_keepout_violated=violated,
                soft_clearance_cost=float(cost))


class MujocoGunGeometryGuard:
    """Use runtime model body names and actual data.xpos/data.xquat on CPU.

    ``body_names=None`` uses all named actual bodies except MuJoCo world.
    An explicit subset is useful for verifying Isaac's recorded fixed-body
    registry; every selected name must still exist in the actual MuJoCo model.
    Float32 is the training precision; float64 is available for strict static
    contract comparisons. No model, data, collision mask or gain is mutated.
    """

    def __init__(self, model, *, urdf_path=URDF, training_assets=ASSETS,
                 body_names=None, dtype=torch.float32):
        self.model = model
        self.urdf_path = Path(urdf_path).resolve(strict=True)
        assets = Path(training_assets).resolve(strict=True)
        contract_source, geometry_source = assets / 'vd12_contract.py', assets / 'vd12_geometry.py'
        contract = load_pure_module('vd12_shared_contract_cpu', contract_source)
        self.profile = contract.geometry_profile(self.urdf_path)
        geometry = load_pure_module('vd12_shared_geometry_cpu', geometry_source)
        source_hash = hashlib.sha256(geometry_source.read_bytes()).hexdigest()
        if source_hash != self.profile['geometry_source_sha256']:
            raise ValueError('VD12 shared geometry changed while loading its contract')
        runtime = [(index, model.body(index).name) for index in range(1, model.nbody)]
        if any(not name for _, name in runtime):
            raise ValueError('Actual MuJoCo robot bodies must have real, nonempty names')
        available = {name: index for index, name in runtime}
        if len(available) != len(runtime):
            raise ValueError('Actual MuJoCo body names must be unique')
        self.body_names = tuple(name for _, name in runtime) if body_names is None else tuple(body_names)
        if any(name not in available for name in self.body_names):
            raise ValueError('Selected body does not exist in the actual MuJoCo model')
        self.body_indices = np.asarray([available[name] for name in self.body_names], dtype=int)
        self.dtype = dtype
        self.geometry = geometry.GunRobotClearance(
            self.urdf_path, self.body_names, device='cpu', dtype=dtype,
            numerical_guard_m=self.profile['numerical_guard_m'],
            minimum_quaternion_norm=self.profile['minimum_quaternion_norm'])
        if self.geometry.metadata['source_urdf_sha256'] != self.profile['urdf_sha256']:
            raise ValueError('MuJoCo adapter and VD12 profile reference different source URDFs')
        self.metadata = dict(
            adapter_version=1, profile=dict(self.profile), geometry_registry=self.geometry.metadata,
            actual_body_ids=self.body_indices.tolist(), actual_body_names=list(self.body_names),
            pose_source='MuJoCo runtime data.xpos and data.xquat, body-link frames, wxyz',
            dtype=str(dtype), device='cpu', actor_input=False, changes_model=False,
            physics_advanced=False, contact_force_estimated=False,
            temporal_scope='One current pose snapshot; no frame-freshness assertion or four-substep episode latch',
            geometry_source=str(geometry_source), contract_source=str(contract_source))

    def evaluate(self, data):
        """Return JSON-compatible single-tick geometry without changing state."""
        position = np.asarray(data.xpos)[self.body_indices].copy()
        quaternion = np.asarray(data.xquat)[self.body_indices].copy()
        with torch.inference_mode():
            result = self.geometry(torch.tensor(position[None], dtype=self.dtype),
                                   torch.tensor(quaternion[None], dtype=self.dtype))
        minimum = float(result['minimum_clearance_lower_bound_m'][0].item())
        pair = int(result['minimum_pair_index'][0].item())
        decision = clearance_decision(minimum, bool(result['input_valid'][0].item()), self.profile)
        decision.update(
            minimum_clearance_lower_bound_m=minimum, minimum_pair_index=pair,
            minimum_pair_name=self.geometry.pair_names[pair] if pair >= 0 else None,
            pair_names=list(self.geometry.pair_names),
            clearance_lower_bound_m=result['clearance_lower_bound_m'][0].tolist(),
            tube_endpoints_w_m=result['tube_endpoints_w'][0].tolist(),
            semantics='Conservative capsule-envelope lower bound; negative is keep-out, not proven physical collision, penetration or contact force',
            actor_input=False, physics_advanced=False)
        return decision
