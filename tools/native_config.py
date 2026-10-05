"""Project a ride environment into the plain-data config the native port reads.

The C++ writers cannot consume live Python objects; ``project(env)`` walks the
public attributes of ``env.sim.applier`` (the ``SuspensionForceApplier``) and
emits only the fields the ported writers actually read, as a versioned dict::

    {'schema': 1,
     'suspension': {'physics_mode': ..., 'joints': ..., 'air_spring': ...,
                    'fork_damper': ..., 'shock_damper': ..., 'coil': ...,
                    'end_stops': ...}}

Every leaf is a builtin int/float/bool/str so the dict crosses nanobind without
pickle or numpy types. ``schema`` versions the layout: the C++ bridge rejects
unknown versions.
"""
import mujoco
import numpy as np

SCHEMA = 1


def _joint_name(model, qposadr):
    """Recover the joint name the applier resolved to this qpos address."""
    jids = np.flatnonzero(model.jnt_qposadr == qposadr)
    if jids.size != 1:
        raise ValueError(f'expected exactly one joint at qposadr {qposadr}')
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(jids[0]))
    if name is None:
        raise ValueError(f'joint at qposadr {qposadr} has no name')
    return name


def _damper_core(d):
    """Fields every BaseDamper subclass shares (post-clamp clicks)."""
    return {
        'max_hsc': int(d.max_hsc), 'max_lsc': int(d.max_lsc),
        'max_reb': int(d.max_reb),
        'hsc_clicks': int(d.hsc_clicks), 'lsc_clicks': int(d.lsc_clicks),
        'rebound_clicks': int(d.rebound_clicks),
        'c_lsc_min': float(d.c_lsc_min), 'c_lsc_max': float(d.c_lsc_max),
        'c_hsc_min': float(d.c_hsc_min), 'c_hsc_max': float(d.c_hsc_max),
        'c_reb_min': float(d.c_reb_min), 'c_reb_max': float(d.c_reb_max),
        'v_knee_comp': float(d.v_knee_comp), 'v_knee_reb': float(d.v_knee_reb),
    }


def project(env) -> dict:
    """Emit the native config dict for env (or a bare RideSimulation)."""
    sim = getattr(env, 'sim', env)
    ap = sim.applier
    air = ap.controller.air_spring
    a = air.specs
    fork = ap.controller.suspension_system.fork_damper
    shock = ap.controller.suspension_system.shock_damper
    coil = ap.coil_shock.specs
    cfg = ap.physics_config
    return {
        'schema': SCHEMA,
        'suspension': {
            'physics_mode': str(cfg.physics_mode),
            'joints': {
                'fork': _joint_name(sim.model, ap.fork_qposadr),
                'shock': _joint_name(sim.model, ap.shock_qposadr),
            },
            'air_spring': {
                'stanchion_inner_diam_mm': float(a.stanchion_inner_diam_mm),
                'total_travel_mm': float(a.total_travel_mm),
                'pos_chamber_length_mm': float(a.pos_chamber_length_mm),
                'neg_chamber_length_mm': float(a.neg_chamber_length_mm),
                'token_volume_cm3': float(a.token_volume_cm3),
                'max_tokens': int(a.max_tokens),
                'gamma': float(a.gamma),
                'atm_pressure_pa': float(a.atm_pressure_pa),
                'num_tokens': int(air.num_tokens),
                'gauge_pressure_psi': float(air.gauge_pressure_psi),
            },
            'fork_damper': {
                **_damper_core(fork),
                'total_travel_mm': float(fork.total_travel_mm),
                'hbo_start_mm': float(fork.hbo_start_mm),
                'c_hbo_base': float(fork.c_hbo_base),
            },
            'shock_damper': {
                **_damper_core(shock),
                'total_stroke_mm': float(shock.total_stroke_mm),
                'max_hbo': int(shock.max_hbo),
                'hbo_clicks': int(shock.hbo_clicks),
                'lockout_firm': bool(shock.lockout_firm),
                'legacy_behavior': bool(shock.legacy_behavior),
                'hbo_start_mm': float(shock.hbo_start_mm),
                'c_hbo_min': float(shock.c_hbo_min),
                'c_hbo_max': float(shock.c_hbo_max),
                'lockout_preload_n': float(shock.lockout_preload_n),
                'lockout_stiffness': float(shock.lockout_stiffness),
            },
            'coil': {
                'rate_n_m': float(coil.rate_n_m),
                'preload_mm': float(coil.preload_mm),
                'stroke_mm': float(coil.stroke_mm),
                'bumper_length_mm': float(coil.bumper_length_mm),
                'bumper_peak_n': float(coil.bumper_peak_n),
                'legacy_behavior': bool(ap.coil_shock.legacy_behavior),
            },
            'end_stops': {
                'stiffness_n_m': float(cfg.end_stops.stiffness_n_m),
                'damping_n_s_m': float(cfg.end_stops.damping_n_s_m),
            },
        },
    }


__all__ = ['project', 'SCHEMA']
