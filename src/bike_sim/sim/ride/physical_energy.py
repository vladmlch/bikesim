"""Stored energy and momentum without changing the caller's force solution.

Callers refresh position/velocity quantities first. In particular these helpers
never call mj_forward: doing so would overwrite the just-solved brake and native
contact forces before they have been recorded.
"""
from math import expm1, log
import numpy as np
from bike_sim.physics.checks import scalar


def gas_chamber_energy(pressure_pa, initial_volume_m3, volume_m3, gamma):
    """Reversible work relative to the reference volume for P V**gamma = const."""
    p = scalar(pressure_pa, 'absolute pressure', positive=True)
    v0 = scalar(initial_volume_m3, 'reference volume', positive=True)
    v = scalar(volume_m3, 'volume', positive=True)
    g = scalar(gamma, 'polytropic exponent', positive=True)
    ratio_log = log(v/v0)
    if abs(g-1.) < 1e-10:
        return -p*v0*ratio_log
    return p*v0*expm1((1.-g)*ratio_log)/(g-1.)


def fork_air_energy(air_spring, travel_m):
    """Integral of the existing, travel-clamped fork force, not a new force law."""
    x = scalar(travel_m, 'fork travel')
    spec = air_spring.specs
    limit = spec.total_travel_mm/1000.
    inside = min(max(x, 0.), limit)
    vp0 = spec.base_pos_volume_m3-air_spring.num_tokens*spec.token_volume_m3
    vn0 = spec.base_neg_volume_m3
    displacement = spec.piston_area_m2*inside
    p0 = air_spring.abs_pressure_pa
    energy = gas_chamber_energy(p0, vp0, vp0-displacement, spec.gamma)
    energy += gas_chamber_energy(p0, vn0, vn0+displacement, spec.gamma)
    # The original spring clamps its input stroke, hence above nominal travel its
    # nonzero terminal force remains constant. Its integral is linear, not flat.
    if x > limit:
        energy += air_spring.compute_axial_force(spec.total_travel_mm)*(x-limit)
    if energy < -1e-9 or not np.isfinite(energy):
        raise ArithmeticError('invalid fork elastic energy')
    return max(energy, 0.)


def mass_observations(model, data):
    """Compiled CoM, kinetic/potential energy and full spatial momentum."""
    import mujoco
    mass = np.asarray(model.body_mass)
    total = float(mass.sum())
    if total <= 0:
        raise ValueError('model has no physical mass')
    com = (mass[:, None]*data.xipos).sum(axis=0)/total
    mv = np.zeros(model.nv)
    mujoco.mj_mulM(model, data, mv, data.qvel)
    kinetic = .5*float(data.qvel @ mv)
    gravity = -float(np.sum(mass[:, None]*data.xipos*model.opt.gravity))
    # Engine subtree momentum is the same body-COM sum as the independent
    # Jacobian oracle in energy.system_momentum, without Python loops each step.
    # mj_subtreeVel updates observation caches only; it does not solve forces.
    mujoco.mj_subtreeVel(model, data)
    linear = total * np.array(data.subtree_linvel[0], copy=True)
    angular = np.array(data.subtree_angmom[0], copy=True)
    return {
        'mass_kg': total, 'com_m': com, 'com_velocity_mps': linear/total,
        'linear_momentum_kg_mps': linear, 'angular_momentum_kg_m2_s': angular,
        'kinetic_energy_j': kinetic, 'gravitational_energy_j': gravity,
    }


def suspension_energy(applier, data):
    """Elastic terms evaluated at q, independently of the last force call."""
    coil = applier.coil_shock.specs
    x = float(data.qpos[applier.shock_qposadr])
    nominal = coil.stroke_mm/1000.
    bumper_length = coil.bumper_length_mm/1000.
    bumper_depth = min(max(x-(nominal-bumper_length), 0.), bumper_length)
    stop = applier.physics_config.end_stops
    over = max(x-nominal, 0.)
    terms = {
        'fork_air': fork_air_energy(applier.controller.air_spring,
                                   float(data.qpos[applier.fork_qposadr])),
        'shock_coil': .5*coil.rate_n_m*max(x+coil.preload_mm/1000., 0.)**2,
        'shock_bumper': coil.bumper_peak_n*bumper_depth**3/(3.*bumper_length**2),
        'shock_top_out': .5*stop.stiffness_n_m*min(x, 0.)**2,
        # Bumper terminal force continues at the upper stop. Its full energy is
        # already in shock_bumper above, and must not be added here a second time.
        'shock_upper_stop': coil.bumper_peak_n*over+.5*stop.stiffness_n_m*over**2,
    }
    return terms
