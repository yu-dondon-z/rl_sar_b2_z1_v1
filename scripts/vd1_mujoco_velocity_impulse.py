"""Simulation-only mass-metric joint velocity constraint impulses.

After MuJoCo's unconstrained step, solve
    min 0.5 * (v_plus-v_minus).T M (v_plus-v_minus)
    subject to -limit <= J v_plus <= limit.
Projected Gauss-Seidel solves the 2*n unilateral dual constraints. The impulse
acts at internal hinge DOFs; inverse mass coupling changes all generalized
velocities, including the floating base. This is NOT per-encoder clipping.

One explicitly labelled physical integration operation writes qvel. No sensor
buffer, qpos, policy target, motor gain, or actor observation is overwritten.
The step integrates position from the original q_n using the constrained
v_plus, not the discarded free velocity. Contacts and impulses are still
operator split; this is not an exact port of the PhysX solver.
"""

from __future__ import annotations

import mujoco
import numpy as np


class VelocityImpulseConstraints:
    def __init__(self, model, joint_ids, limits, max_iterations=256, tolerance=1e-8):
        self.model = model
        self.joints = np.asarray(joint_ids, dtype=int)
        self.limits = np.asarray(limits, dtype=float).copy()
        if (self.joints.ndim != 1 or not len(self.joints)
                or len(set(self.joints.tolist())) != len(self.joints)
                or np.any(self.joints < 0) or np.any(self.joints >= model.njnt)
                or self.limits.shape != self.joints.shape
                or not np.isfinite(self.limits).all() or np.any(self.limits <= 0)
                or np.any(model.jnt_type[self.joints] != mujoco.mjtJoint.mjJNT_HINGE)):
            raise ValueError("Velocity impulses require distinct scalar hinges and positive finite limits")
        self.dofs = np.asarray(model.jnt_dofadr[self.joints], dtype=int)
        self.max_iterations = int(max_iterations)
        self.tolerance = float(tolerance)
        if self.max_iterations <= 0 or not np.isfinite(self.tolerance) or self.tolerance <= 0:
            raise ValueError("Invalid velocity impulse convergence settings")
        self.mass = np.empty((model.nv, model.nv))
        self.selection = np.eye(model.nv)[:, self.dofs]
        self.physics_steps = self.projection_calls = self.active_projections = 0
        self.max_iterations_used = 0
        self.max_residual = self.max_kkt_residual = 0.
        self.max_impulse = self.max_reaction = self.max_positive_energy_change = 0.
        self.last = {}

    def project(self, data, *, mass_override=None, synchronize=True):
        """Apply an internal constraint impulse at the current physical state.

        This method does not advance time or position. It refreshes MuJoCo's
        state-derived mass/sensors, then writes the coupled velocity increment
        once only after convergence and the kinetic energy check succeed.
        """
        if mass_override is None:
            mujoco.mj_forward(self.model, data)
        else:
            source_mass = np.asarray(mass_override, dtype=float)
            if source_mass.shape != self.mass.shape or not np.isfinite(source_mass).all():
                raise ValueError("Invalid physical mass override")
            self.mass[:] = source_mass
        before = data.qvel.copy()
        if not np.isfinite(before).all():
            raise ValueError("Nonfinite physical velocity before constraint solve")
        self.projection_calls += 1
        initial_excess = float(np.maximum(np.abs(before[self.dofs]) - self.limits, 0).max())
        if initial_excess <= self.tolerance:
            self.last = dict(active=False, iterations=0, max_velocity_residual=initial_excess,
                             kinetic_energy_change=0., generalized_impulse=[0.]*self.model.nv)
            self.max_residual = max(self.max_residual, initial_excess)
            if synchronize and mass_override is not None:
                mujoco.mj_forward(self.model, data)
            return
        if mass_override is None:
            mujoco.mj_fullM(self.model, self.mass, data.qM)
        inverse_columns = np.linalg.solve(self.mass, self.selection)
        diagonal = inverse_columns[self.dofs, np.arange(len(self.dofs))]
        if not np.isfinite(inverse_columns).all() or np.any(diagonal <= 0):
            raise ValueError("Constraint solve requires a positive definite physical mass matrix")
        velocity = before.copy()
        multipliers = np.zeros((len(self.dofs), 2))
        signs = (1., -1.)
        kkt = float("inf")
        for iteration in range(1, self.max_iterations + 1):
            for row, dof in enumerate(self.dofs):
                for side, sign in enumerate(signs):
                    violation = sign*velocity[dof] - self.limits[row]
                    previous = multipliers[row, side]
                    updated = max(0., previous + violation/diagonal[row])
                    delta_multiplier = updated - previous
                    if delta_multiplier != 0:
                        velocity -= sign*delta_multiplier*inverse_columns[:, row]
                        multipliers[row, side] = updated
            violations = np.column_stack((velocity[self.dofs] - self.limits,
                                           -velocity[self.dofs] - self.limits))
            residuals = np.where(multipliers > 0, np.abs(violations), np.maximum(violations, 0))
            kkt = float(residuals.max())
            if kkt <= self.tolerance:
                break
        if kkt > self.tolerance:
            raise RuntimeError("Velocity constraint PGS did not converge: residual=%.9g after %d iterations"
                               % (kkt, self.max_iterations))
        impulse = np.zeros(self.model.nv)
        impulse[self.dofs] = multipliers[:, 1] - multipliers[:, 0]
        # Reconstruct from the full mass solve, avoiding accumulated vector
        # update drift. The base entries are generally nonzero in delta_v.
        delta_v = inverse_columns @ impulse[self.dofs]
        after = before + delta_v
        residual = float(np.maximum(np.abs(after[self.dofs]) - self.limits, 0).max())
        energy_before = .5*float(before @ self.mass @ before)
        energy_after = .5*float(after @ self.mass @ after)
        energy_change = energy_after - energy_before
        energy_roundoff = 1e-10*max(1., abs(energy_before))
        if residual > max(2*self.tolerance, 1e-10) or energy_change > energy_roundoff:
            raise RuntimeError("Velocity impulse violates feasibility/passivity: residual=%g, delta_energy=%g"
                               % (residual, energy_change))
        # Physical constraint integration, NOT an observation modification or
        # independent component clip. This is the sole state velocity write.
        data.qvel[:] += delta_v
        if synchronize:
            mujoco.mj_forward(self.model, data)
        self.active_projections += 1
        self.max_iterations_used = max(self.max_iterations_used, iteration)
        self.max_residual = max(self.max_residual, residual)
        self.max_kkt_residual = max(self.max_kkt_residual, kkt)
        self.max_impulse = max(self.max_impulse, float(np.abs(impulse).max()))
        self.max_reaction = max(self.max_reaction, float(np.abs(impulse).max())/self.model.opt.timestep)
        self.max_positive_energy_change = max(self.max_positive_energy_change, max(energy_change, 0.))
        self.last = dict(active=True, iterations=iteration, max_velocity_residual=residual,
                         kkt_velocity_residual=kkt, kinetic_energy_before=energy_before,
                         kinetic_energy_after=energy_after, kinetic_energy_change=energy_change,
                         generalized_impulse=impulse.tolist(), coupled_delta_velocity=delta_v.tolist())

    def step(self, data, *, evaluation_observer=None):
        # Use the beginning-of-step metric, matching the acceleration solve.
        mujoco.mj_forward(self.model, data)
        initial_qpos = data.qpos.copy()
        initial_mass = np.empty_like(self.mass)
        mujoco.mj_fullM(self.model, initial_mass, data.qM)
        # The native free step obtains v_free; its position increment is
        # discarded below, because it used a velocity before our constraints.
        mujoco.mj_step(self.model, data)
        # Optional read-only evaluation tap before mj_forward overwrites the
        # force/constraint buffers used by this native step. No feedback.
        if evaluation_observer is not None:
            evaluation_observer(data)
        self.physics_steps += 1
        self.project(data, mass_override=initial_mass, synchronize=False)
        integrated_qpos = initial_qpos.copy()
        mujoco.mj_integratePos(self.model, integrated_qpos, data.qvel, self.model.opt.timestep)
        # Physical semi-implicit position integration using constrained v_plus.
        # This sole qpos write is explicit, not a sensor/observation correction.
        data.qpos[:] = integrated_qpos
        mujoco.mj_forward(self.model, data)

    def summary(self):
        return dict(model="mass-metric unilateral joint velocity impulses with operator splitting",
                    physics_steps=self.physics_steps, projection_calls=self.projection_calls,
                    active_projections=self.active_projections, max_iterations_used=self.max_iterations_used,
                    tolerance=self.tolerance, max_velocity_residual=self.max_residual,
                    max_kkt_velocity_residual=self.max_kkt_residual,
                    max_generalized_impulse_nms=self.max_impulse,
                    max_equivalent_constraint_reaction_nm=self.max_reaction,
                    max_positive_projection_kinetic_energy_change_j=self.max_positive_energy_change,
                    state_velocity_impulse_applied=True, independent_component_clip=False,
                    sensor_buffer_overwritten=False, actor_targets_changed=False, robot_pd_gains_changed=False,
                    floating_base_inverse_mass_coupling=True, exact_physx_solver=False,
                    position_integrated_from_constrained_velocity=True,
                    beginning_of_step_mass_metric=True, physical_qpos_integration_written=True)
