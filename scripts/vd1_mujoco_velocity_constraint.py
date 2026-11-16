"""Simulation-only regularized approximation of PhysX joint speed constraints.

MuJoCo has no native maximum hinge velocity constraint. This optional model
adds an implicitly integrated brake when the unconstrained next velocity would
exceed the source joint limit. It never assigns qpos/qvel or alters an encoder,
policy action, target or robot PD gain. It is not an exact PhysX solver port.
"""
from __future__ import annotations

import mujoco
import numpy as np


class ImplicitVelocityBrake:
    def __init__(self, model, joint_ids, velocity_limits, regularization=.1):
        self.model = model
        self.dofs = np.asarray(model.jnt_dofadr[joint_ids], dtype=int)
        self.limits = np.asarray(velocity_limits, dtype=float).copy()
        self.baseline_damping = model.dof_damping.copy()
        self.regularization = float(regularization)
        self.mass = np.empty((model.nv, model.nv))
        self.max_brake_force = 0.
        self.active_substeps = 0
        self._applied = np.zeros(model.nv)

    def step(self, data):
        # These are engine internals for a physical constraint, not actor input.
        self.model.dof_damping[:] = self.baseline_damping
        data.qfrc_applied[:] -= self._applied
        self._applied[:] = 0.
        mujoco.mj_forward(self.model, data)
        dt = self.model.opt.timestep
        predicted = data.qvel[self.dofs]+dt*data.qacc[self.dofs]
        active = np.abs(predicted) > self.limits
        if np.any(active):
            mujoco.mj_fullM(self.model, self.mass, data.qM)
            inverse = np.linalg.inv(self.mass)
            effective_mass = 1./np.diag(inverse)[self.dofs]
            damping = effective_mass/(self.regularization*dt)
            dofs = self.dofs[active]
            self.model.dof_damping[dofs] += damping[active]
            self._applied[dofs] = damping[active]*np.sign(predicted[active])*self.limits[active]
            data.qfrc_applied[:] += self._applied
            self.active_substeps += 1
        mujoco.mj_step(self.model, data)
        if np.any(active):
            force = self._applied[self.dofs]-(self.model.dof_damping[self.dofs]
                    -self.baseline_damping[self.dofs])*data.qvel[self.dofs]
            self.max_brake_force = max(self.max_brake_force, float(np.max(np.abs(force))))
        # Remove approximation forces before readers/PD diagnostics run.
        data.qfrc_applied[:] -= self._applied
        self._applied[:] = 0.
        self.model.dof_damping[:] = self.baseline_damping

    def summary(self):
        return dict(model='implicit deadband brake, approximate velocity constraint',
                    regularization_fraction_of_physics_dt=self.regularization,
                    active_substeps=self.active_substeps,
                    max_brake_generalized_force_nm=self.max_brake_force,
                    qpos_qvel_assigned=False, encoder_clipped=False)
