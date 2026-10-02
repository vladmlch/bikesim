"""Longitudinal tire contact characterization without motor or rider feedback.

Isolates tire longitudinal contact mechanics (slip ratio, brush compliance,
peak/sliding friction limits, passivity) from motor and rider controllers.
Inputs are pure bench kinematics/materials.
"""
from math import isfinite
from bike_sim.physics.checks import scalar
from bike_sim.physics.tire import brush_step


def steady_brush_force(tire, surface, slip_ratio, load_n, *, speed_mps=2.,
                       dt_s=.000625, duration_s=1.) -> dict:
    """Measure steady longitudinal force and dissipation on a tire-surface bench.

    Slip ratio is dimensionless and traction-positive: kappa = -u / |v_roll|.
    The output is isolated pure material mechanics and does not feed back to
    any motor or rider controller.
    """
    kappa = scalar(slip_ratio, 'slip ratio')
    load = scalar(load_n, 'normal load', positive=True)
    speed = scalar(speed_mps, 'rolling speed', positive=True)
    dt = scalar(dt_s, 'timestep', positive=True)
    duration = scalar(duration_s, 'duration', positive=True)
    n = round(duration / dt)
    if n < 1 or abs(n * dt - duration) > 1e-10:
        raise ValueError('duration must contain integer physical steps')
    u = -kappa * speed
    mu = min(float(tire.mu), float(surface.mu(u)))
    xi, loss, force = 0., 0., 0.
    for _ in range(n):
        xi, force, step_loss = brush_step(xi, u, speed, load,
                                         tire.tangent_k_n_m, mu,
                                         tire.relaxation_length_m, dt)
        loss += step_loss
    if not all(isfinite(x) for x in (xi, force, loss)):
        raise ValueError('nonfinite tire characterization')
    return {'slip_ratio': kappa, 'slip_mps': u, 'force_n': force,
            'mu_used': mu, 'normal_load_n': load, 'loss_j': loss,
            'stored_energy_j': .5 * tire.tangent_k_n_m * xi * xi,
            'configured_ckappa_n': tire.tangent_k_n_m * tire.relaxation_length_m,
            'surface_ckappa_per_load_metadata': surface.slip_stiffness_per_load}
