"""Project a ride environment into the plain-data config the native port reads.

The C++ writers cannot consume live Python objects; ``project(env)`` walks the
public attributes of ``env.sim.applier`` (the ``SuspensionForceApplier``),
``env.sim.brakes`` (the ``BrakeController``) and
``physics_config.resistance``, and emits only the fields the ported writers
actually read, as a versioned dict::

    {'schema': 2,
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
     'rider_forces': {'paths': [{joint, stiffness_n_m, damping_ns_m,
                                 preload_deflection_m, unilateral,
                                 offset_m}, ...]},
     'rider_contacts': {'arm_reach_m': ..., 'pedal_attachment': ...,
                        'saddle_attachment': ..., 'grip_attachment': ...,
                        <resolved ArticulatedConfig contact fields>},
    }

The 'tire' section is emitted only for ``backend == 'compliant_2d'`` — the
only backend the native port implements (``native_reference`` produces no
tire writer in Python either, so no section; ``distributed_2d_reference``
is a different, unported writer and is rejected). The material must be a
``TireSpec``; a ``TabulatedTireSpec`` follows a different force law the
native writer does not implement and is rejected here. The surface map is
serialized with every named surface resolved to its numeric fields.

The 'rider_contacts' section is emitted only when ``sim.physical`` owns a
``RiderContactApplier`` (an ``articulated_planar`` rider; ``None`` otherwise
or without a physical runtime). ``arm_reach_m`` carries the applier's
pose-resolved reach — ``ArticulatedConfig.arm_reach_fraction`` is a build-time
knob, not a wire field — and ``grip_pair_force_limit_n`` keeps the dataclass's
``float | None`` optionality on the wire.

The 'rider_forces' section is emitted only while
``sim.rider_forces.active`` — on rider variants without a seated pose
(e.g. ``articulated_planar``) the applier is inert and no section appears.
Each entry carries the resolved per-path spring parameters the writer's
``compute`` reads, plus ``offset_m`` — the current pedal offset
``set_pedal_offsets`` left on the path at projection time. These offsets are
mutable: legacy ``_follow_cranks`` updates them during stepping, so P3/P4 must
port those updates and supply current offsets before step-loop equivalence.
The P2 config captures effective values, not the crank derivation.
``preload_deflection_m`` is emitted evaluated — the body's
property computes preload/stiffness, and the resulting double is what the
writer's compute reads.

Every leaf is a builtin int/float/bool/str so the dict crosses nanobind without
pickle or numpy types. ``schema`` versions the layout: the C++ bridge rejects
unknown versions. Schema 2 carries the exact fork ``legacy_behavior`` flag.
Schema 1 infers legacy when resolved ``hbo_start_mm == 160.0`` and keeps
resolved damping parameters, including the ambiguous modern 180 mm fork.
"""
import mujoco
import numpy as np
from tools.native_schema import validate_config, validate_drive_policies, plain, sequence

SCHEMA = 2


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
        'max_hsc': d.max_hsc, 'max_lsc': d.max_lsc,
        'max_reb': d.max_reb,
        'hsc_clicks': d.hsc_clicks, 'lsc_clicks': d.lsc_clicks,
        'rebound_clicks': d.rebound_clicks,
        'c_lsc_min': d.c_lsc_min, 'c_lsc_max': d.c_lsc_max,
        'c_hsc_min': d.c_hsc_min, 'c_hsc_max': d.c_hsc_max,
        'c_reb_min': d.c_reb_min, 'c_reb_max': d.c_reb_max,
        'v_knee_comp': d.v_knee_comp, 'v_knee_reb': d.v_knee_reb,
    }


def _surface_spec(spec) -> dict:
    """SurfaceSpec -> the numeric fields the mu(v) curve and diagnostics read."""
    return {'name': spec.name, 'mu_peak': spec.mu_peak,
            'mu_slide': spec.mu_slide,
            'slip_stiffness_per_load': spec.slip_stiffness_per_load,
            'stribeck_speed_mps': spec.stribeck_speed_mps}


def _tire_side(params) -> dict:
    """TireParameters -> the fields compute_qfrc reads (TireSpec only)."""
    from bike_sim.physics.tire import TireSpec
    material = params.material
    if not isinstance(material, TireSpec):
        raise ValueError('native tire port requires TireSpec material, '
                         f'got {type(material).__name__}')
    return {
        'material': {
            'radial_k_n_m': material.radial_k_n_m,
            'radial_c_ns_m': material.radial_c_ns_m,
            'pressure_pa_gauge': material.pressure_pa_gauge,
            'provenance': material.provenance,
            'valid_load_range_n': material.valid_load_range_n,
        },
        'tangent_k_n_m': params.tangent_k_n_m,
        'mu': params.mu,
        'relaxation_length_m': params.relaxation_length_m,
    }


def _tire_section(cfg_tires, surface_map) -> dict:
    """TireBackendConfig + SurfaceMap -> the tire writer's section."""
    out = {
        'backend': cfg_tires.backend,
        'surface_mode': cfg_tires.surface_mode,
        'significant_delta_m': cfg_tires.significant_delta_m,
        'significance_fraction': cfg_tires.significance_fraction,
        'distinct_normal_deg': cfg_tires.distinct_normal_deg,
        'front': _tire_side(cfg_tires.front),
        'rear': _tire_side(cfg_tires.rear),
    }
    if cfg_tires.surface_mode == 'track':
        if surface_map is None:
            raise ValueError('surface_mode track requires a track surface map')
        from bike_sim.terrain.surface import get_surface
        out['surface_map'] = {
            'surface': _surface_spec(surface_map._surface),
            'sections': [{'start_m': s.start_m, 'end_m': s.end_m,
                          'surface': _surface_spec(get_surface(s.surface))}
                         for s in sequence(surface_map.sections, "config.tire.surface_map.sections")],
        }
    return out


def _project_drive_policies(drive) -> dict:
    """Copy resolved runtime drive policies into owning plain data."""
    a = drive.assist
    profile = None
    if a.profile is not None:
        profile = {
            'mode_gains': {key: [x for x in value] if isinstance(value, tuple)
                           else value for key, value in a.profile.mode_gains.items()},
            'emtb_full_gain_at_nm': a.profile.emtb_full_gain_at_nm,
        }
    return {
        'gearing': {'front_teeth': drive.config.gearing.front_teeth,
                    'rear_teeth': drive.config.gearing.rear_teeth,
                    'chain_pitch_m': drive.config.gearing.chain_pitch_m},
        'pedaling': {'enabled': drive.pedaling.config.enabled,
                     **{key: getattr(drive.pedaling.config, key) for key in
                        ('coast_above_rpm', 'resume_below_rpm', 'stop_time_s',
                         'coast_cadence_tau_s', 'mash_cadence_rpm', 'mash_torque_nm',
                         'effort_slew_nm_s')}},
        'shifting': {'enabled': drive.shifting.config.enabled,
                     'cassette': drive.shifting.config.cassette,
                     'upshift_slip_mode': drive.shifting.config.upshift_slip_mode,
                     **{key: getattr(drive.shifting.config, key) for key in
                        ('target_cadence_min_rpm', 'target_cadence_max_rpm',
                         'shift_cooldown_s', 'shift_cut_duration_s', 'torque_factor',
                         'cadence_smoothing_tau_s', 'upshift_slip_limit_mps')}},
        'assist': {**{key: getattr(a, key) for key in
                     ('gain', 'max_torque', 'max_power', 'tau', 'slew',
                      'engage_torque_nm', 'gate_min_crank_rad_s')},
                   'cutoff_mps': a.cutoff, 'taper_width_mps': a.width,
                   'mode': a.mode, 'profile': profile,
                   'torque_curve': a.torque_curve},
        'battery': {'enabled': drive.config.battery.enabled,
                    **{key: getattr(drive.config.battery, key) for key in
                       ('energy_j', 'copper_w_per_nm2', 'speed_w_per_rad_s2', 'idle_w')}},
        'hub_stiffness_nm_rad': drive.config.freehub_k_nm_rad,
        'hub_damping_nm_s': drive.config.freehub_c_nms_rad,
    }


def _project_drive(drive) -> dict:
    """Own construction-time drivetrain setup; runtime snapshots are separate."""
    cfg = drive.config
    return {**project_drive_policies(drive), 'drive_mode':drive.drive_mode,
            'transmission_model':cfg.transmission_model,
            'motor_clutch':cfg.motor_clutch,
            **{key:getattr(cfg,key) for key in
               ('human_torque_nm','torque_ripple','crank_phase_rad','chain_k_n_m',
                'chain_c_ns_m','bearing_c_nms_rad','rotor_inertia_kgm2')}}


def _project(env) -> dict:
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
            'torque_ceiling_nm': sim.brakes.torque_ceiling_nm,
            'taper_radps': sim.brakes.taper_radps,
        },
        # The body names are literals in ExternalResistanceApplier.__init__
        # ('frame', '<side>_wheel'); emitted so a renamed model stays
        # describable.
        'resistance': {
            'crr': res.crr,
            'rolling_taper_rad_s': res.rolling_taper_rad_s,
            'rho_kg_m3': res.rho_kg_m3,
            'cda_m2': res.cda_m2,
            'wind_world_mps': res.wind_world_mps,
            'point_body_m': res.point_body_m,
            'bodies': {'frame': 'frame', 'front_wheel': 'front_wheel',
                       'rear_wheel': 'rear_wheel'},
        },
        'suspension': {
            'physics_mode': cfg.physics_mode,
            'joints': {
                'fork': _joint_name(sim.model, ap.fork_qposadr),
                'shock': _joint_name(sim.model, ap.shock_qposadr),
            },
            'air_spring': {
                'stanchion_inner_diam_mm': a.stanchion_inner_diam_mm,
                'total_travel_mm': a.total_travel_mm,
                'pos_chamber_length_mm': a.pos_chamber_length_mm,
                'neg_chamber_length_mm': a.neg_chamber_length_mm,
                'token_volume_cm3': a.token_volume_cm3,
                'max_tokens': a.max_tokens,
                'gamma': a.gamma,
                'atm_pressure_pa': a.atm_pressure_pa,
                'num_tokens': air.num_tokens,
                'gauge_pressure_psi': air.gauge_pressure_psi,
            },
            'fork_damper': {
                **_damper_core(fork),
                'total_travel_mm': fork.total_travel_mm,
                'hbo_start_mm': fork.hbo_start_mm,
                'c_hbo_base': fork.c_hbo_base,
                'legacy_behavior': fork.legacy_behavior,
            },
            'shock_damper': {
                **_damper_core(shock),
                'total_stroke_mm': shock.total_stroke_mm,
                'max_hbo': shock.max_hbo,
                'hbo_clicks': shock.hbo_clicks,
                'lockout_firm': shock.lockout_firm,
                'legacy_behavior': shock.legacy_behavior,
                'hbo_start_mm': shock.hbo_start_mm,
                'c_hbo_min': shock.c_hbo_min,
                'c_hbo_max': shock.c_hbo_max,
                'lockout_preload_n': shock.lockout_preload_n,
                'lockout_stiffness': shock.lockout_stiffness,
            },
            'coil': {
                'rate_n_m': coil.rate_n_m,
                'preload_mm': coil.preload_mm,
                'stroke_mm': coil.stroke_mm,
                'bumper_length_mm': coil.bumper_length_mm,
                'bumper_peak_n': coil.bumper_peak_n,
                'legacy_behavior': ap.coil_shock.legacy_behavior,
            },
            'end_stops': {
                'stiffness_n_m': cfg.end_stops.stiffness_n_m,
                'damping_n_s_m': cfg.end_stops.damping_n_s_m,
            },
        },
    }
    if tires.backend == 'compliant_2d':
        # sim.track.surface_map is a fresh SurfaceMap per access; the tire
        # writer captured an equal-valued one at construction — content is
        # what crosses the boundary.
        out['tire'] = _tire_section(tires, sim.track.surface_map)
    if sim.rider_forces.active:
        # _paths order is apply()'s order. Joint names re-resolve through
        # _resolve_slide on the native side, like the suspension joints.
        out['rider_forces'] = {'paths': [
            {'joint': _joint_name(sim.model, p.qposadr),
             'stiffness_n_m': p.body.stiffness_n_m,
             'damping_ns_m': p.body.damping_ns_m,
             'preload_deflection_m': p.body.preload_deflection_m,
             'unilateral': p.body.unilateral,
             'offset_m': p.offset_m}
            for p in sim.rider_forces._paths]}
    cruise = getattr(sim, 'cruise', None)
    if cruise is not None:
        out['cruise'] = {
            'target_speed_kmh': cruise.target_speed_kmh,
            'kp_nm_per_mps': cruise.kp_nm_per_mps,
            'ki_nm_per_mps_s': cruise.ki_nm_per_mps_s,
            'torque_ceiling_nm': cruise.torque_ceiling_nm,
        }
    physical = getattr(sim, 'physical', None)
    drive = getattr(physical, 'drive', None)
    if drive is not None:
        out['drive'] = project_drive(drive)
    rider_contacts = getattr(physical, 'rider_contacts', None)
    if rider_contacts is not None:
        out['rider_contacts'] = project_rider_contacts(rider_contacts)
    return out


def _project_rider_contacts(applier) -> dict:
    """Copy the RiderContactApplier's resolved contact setup.

    The section mirrors the ArticulatedConfig fields compute_qfrc/_pads read
    (rider_contacts.py) plus the reach the applier resolved from the pose.
    Mutable contact material state belongs to snapshots, not this section.
    """
    cfg = applier.config
    return {
        'arm_reach_m': applier.arm_reach,
        **{key: getattr(cfg, key) for key in (
            'saddle_patch_half_length_m', 'pedal_patch_half_length_m',
            'support_pad_radius_m', 'support_k_n_m', 'support_c_ns_m',
            'pedal_c_ns_m', 'support_tangent_k_n_m', 'support_mu',
            'support_length_m', 'grip_k_n_m', 'grip_c_ns_m',
            'grip_release_distance_m', 'grip_pair_force_limit_n',
            'grip_capture_distance_m', 'grip_capture_speed_mps',
            'pedal_attachment', 'saddle_attachment', 'grip_attachment')},
    }


def project_rider_contacts(applier) -> dict:
    out = _project_rider_contacts(applier)
    validate_config({'schema': SCHEMA, 'rider_contacts': out})
    return plain(out)


__all__ = ['project', 'project_drive_policies', 'project_drive',
           'project_rider_contacts', 'SCHEMA']

def project_drive_policies(drive) -> dict:
    out = _project_drive_policies(drive)
    validate_drive_policies(out)
    return plain(out)


def project_drive(drive) -> dict:
    out = _project_drive(drive)
    validate_config({'schema': SCHEMA, 'drive': out})
    return plain(out)


def project(env) -> dict:
    out = _project(env)
    validate_config(out)
    return plain(out)
