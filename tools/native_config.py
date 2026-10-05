"""Project a ride environment into the plain-data config the native port reads.

The C++ writers cannot consume live Python objects; ``project(env)`` walks the
public attributes of ``env.sim.applier`` (the ``SuspensionForceApplier``),
``env.sim.brakes`` (the ``BrakeController``) and
``physics_config.resistance``, and emits only the fields the ported writers
actually read, as a versioned dict::

    {'schema': 1,
     'suspension': {'physics_mode': ..., 'joints': ..., 'air_spring': ...,
                    'fork_damper': ..., 'shock_damper': ..., 'coil': ...,
                    'end_stops': ...},
     'brake': {'torque_ceiling_nm': ..., 'taper_radps': ...},
     'resistance': {'crr': ..., 'rolling_taper_rad_s': ..., 'rho_kg_m3': ...,
                    'cda_m2': ..., 'wind_world_mps': [...],
                    'point_body_m': [...], 'bodies': {...}},
     'tire': {'backend': ..., 'surface_mode': ...,
              'significant_delta_m': ..., 'significance_fraction': ...,
              'distinct_normal_deg': ...,
              'front': {material, tangent_k_n_m, mu, relaxation_length_m},
              'rear': {...},
              'surface_map': {'surface': {...SurfaceSpec fields...},
                              'sections': [{start_m, end_m, surface}]}},
    }

The 'tire' section is emitted only for ``backend == 'compliant_2d'`` — the
only backend the native port implements (``native_reference`` produces no
tire writer in Python either, so no section; ``distributed_2d_reference``
is a different, unported writer and is rejected). The material must be a
``TireSpec``; a ``TabulatedTireSpec`` follows a different force law the
native writer does not implement and is rejected here. The surface map is
serialized with every named surface resolved to its numeric fields.

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


def _surface_spec(spec) -> dict:
    """SurfaceSpec -> the numeric fields the mu(v) curve and diagnostics read."""
    return {'name': str(spec.name), 'mu_peak': float(spec.mu_peak),
            'mu_slide': float(spec.mu_slide),
            'slip_stiffness_per_load': float(spec.slip_stiffness_per_load),
            'stribeck_speed_mps': float(spec.stribeck_speed_mps)}


def _tire_side(params) -> dict:
    """TireParameters -> the fields compute_qfrc reads (TireSpec only)."""
    from bike_sim.physics.tire import TireSpec
    material = params.material
    if not isinstance(material, TireSpec):
        raise ValueError('native tire port requires TireSpec material, '
                         f'got {type(material).__name__}')
    lo, hi = material.valid_load_range_n
    return {
        'material': {
            'radial_k_n_m': float(material.radial_k_n_m),
            'radial_c_ns_m': float(material.radial_c_ns_m),
            'pressure_pa_gauge': float(material.pressure_pa_gauge),
            'provenance': str(material.provenance),
            'valid_load_range_n': [float(lo), float(hi)],
        },
        'tangent_k_n_m': float(params.tangent_k_n_m),
        'mu': float(params.mu),
        'relaxation_length_m': float(params.relaxation_length_m),
    }


def _tire_section(cfg_tires, surface_map) -> dict:
    """TireBackendConfig + SurfaceMap -> the tire writer's section."""
    out = {
        'backend': str(cfg_tires.backend),
        'surface_mode': str(cfg_tires.surface_mode),
        'significant_delta_m': float(cfg_tires.significant_delta_m),
        'significance_fraction': float(cfg_tires.significance_fraction),
        'distinct_normal_deg': float(cfg_tires.distinct_normal_deg),
        'front': _tire_side(cfg_tires.front),
        'rear': _tire_side(cfg_tires.rear),
    }
    if cfg_tires.surface_mode == 'track':
        if surface_map is None:
            raise ValueError('surface_mode track requires a track surface map')
        from bike_sim.terrain.surface import get_surface
        out['surface_map'] = {
            'surface': _surface_spec(surface_map._surface),
            'sections': [{'start_m': float(s.start_m), 'end_m': float(s.end_m),
                          'surface': _surface_spec(get_surface(s.surface))}
                         for s in surface_map.sections],
        }
    return out


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
    res = cfg.resistance
    # sim.physics_config is the object the runtime's tire writer read.
    tires = sim.physics_config.tires
    if tires.backend not in ('compliant_2d', 'native_reference'):
        raise ValueError(f"native tire backend '{tires.backend}' is not ported")
    out = {
        'schema': SCHEMA,
        'brake': {
            'torque_ceiling_nm': float(sim.brakes.torque_ceiling_nm),
            'taper_radps': float(sim.brakes.taper_radps),
        },
        # The body names are literals in ExternalResistanceApplier.__init__
        # ('frame', '<side>_wheel'); emitted so a renamed model stays
        # describable.
        'resistance': {
            'crr': float(res.crr),
            'rolling_taper_rad_s': float(res.rolling_taper_rad_s),
            'rho_kg_m3': float(res.rho_kg_m3),
            'cda_m2': float(res.cda_m2),
            'wind_world_mps': [float(x) for x in res.wind_world_mps],
            'point_body_m': [float(x) for x in res.point_body_m],
            'bodies': {'frame': 'frame', 'front_wheel': 'front_wheel',
                       'rear_wheel': 'rear_wheel'},
        },
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
    if tires.backend == 'compliant_2d':
        # sim.track.surface_map is a fresh SurfaceMap per access; the tire
        # writer captured an equal-valued one at construction — content is
        # what crosses the boundary.
        out['tire'] = _tire_section(tires, sim.track.surface_map)
    return out


__all__ = ['project', 'SCHEMA']
