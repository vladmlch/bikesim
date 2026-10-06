"""Pure validation of mutable physical inputs; force formulas stay in their owners."""
from bike_sim.physics.checks import boolean, integer, scalar, sequence


def air_specs(specs):
    for key in ('stanchion_inner_diam_mm', 'total_travel_mm', 'pos_chamber_length_mm',
                'neg_chamber_length_mm', 'gamma', 'atm_pressure_pa'):
        scalar(getattr(specs, key), f'AirSpringSpecs.{key}', positive=True)
    scalar(specs.token_volume_cm3, 'AirSpringSpecs.token_volume_cm3', minimum=0.)
    integer(specs.max_tokens, 'AirSpringSpecs.max_tokens', minimum=0)
    integer(specs.default_tokens, 'AirSpringSpecs.default_tokens')


def damper_core(damper):
    for key in ('max_hsc', 'max_lsc', 'max_reb'):
        integer(getattr(damper, key), f'{type(damper).__name__}.{key}', positive=True)
    for key in ('hsc_clicks', 'lsc_clicks', 'rebound_clicks'):
        integer(getattr(damper, key), f'{type(damper).__name__}.{key}')
    for band in ('lsc', 'hsc', 'reb'):
        low, high = f'c_{band}_min', f'c_{band}_max'
        scalar(getattr(damper, low), f'{type(damper).__name__}.{low}', minimum=0.)
        scalar(getattr(damper, high), f'{type(damper).__name__}.{high}', minimum=0.)
        if getattr(damper, low) > getattr(damper, high):
            raise ValueError(f'{type(damper).__name__}.{low}: damping bounds must be ordered')
    for key in ('v_knee_comp', 'v_knee_reb'):
        scalar(getattr(damper, key), f'{type(damper).__name__}.{key}', positive=True)
    scalar(damper.total_travel_mm, f'{type(damper).__name__}.total_travel_mm', positive=True)


def damper(damper):
    damper_core(damper)
    if not hasattr(damper, 'legacy_behavior'):
        return
    name = type(damper).__name__
    boolean(damper.legacy_behavior, f'{name}.legacy_behavior')
    start = scalar(damper.hbo_start_mm, f'{name}.hbo_start_mm', minimum=0.)
    stroke = getattr(damper, 'total_stroke_mm', damper.total_travel_mm)
    scalar(stroke, f'{name}.travel', positive=True)
    if not damper.legacy_behavior and not start < stroke:
        raise ValueError(f'{name}.hbo_start_mm: modern HBO must start within travel')
    if hasattr(damper, 'c_hbo_base'):
        scalar(damper.c_hbo_base, f'{name}.c_hbo_base', minimum=0.)
    else:
        integer(damper.max_hbo, f'{name}.max_hbo', positive=True)
        integer(damper.hbo_clicks, f'{name}.hbo_clicks')
        boolean(damper.lockout_firm, f'{name}.lockout_firm')
        for key in ('c_hbo_min', 'c_hbo_max', 'lockout_preload_n', 'lockout_stiffness'):
            scalar(getattr(damper, key), f'{name}.{key}', minimum=0.)
        if damper.c_hbo_min > damper.c_hbo_max:
            raise ValueError(f'{name}.c_hbo_min: HBO bounds must be ordered')


def coil_specs(specs):
    for key in ('rate_n_m', 'stroke_mm', 'bumper_length_mm', 'bumper_peak_n'):
        scalar(getattr(specs, key), f'CoilShockSpecs.{key}', positive=True)
    scalar(specs.preload_mm, 'CoilShockSpecs.preload_mm', minimum=0.)
    if specs.bumper_length_mm > specs.stroke_mm:
        raise ValueError('CoilShockSpecs.bumper_length_mm: exceeds stroke')


def pedaling_config(config):
    boolean(config.enabled, 'PedalingConfig.enabled')
    for key in ('coast_above_rpm', 'stop_time_s'):
        scalar(getattr(config, key), f'PedalingConfig.{key}', positive=True)
    for key in ('resume_below_rpm', 'coast_cadence_tau_s', 'mash_cadence_rpm', 'mash_torque_nm', 'effort_slew_nm_s'):
        scalar(getattr(config, key), f'PedalingConfig.{key}', minimum=0.)
    if config.resume_below_rpm >= config.coast_above_rpm:
        raise ValueError('PedalingConfig.resume_below_rpm: invalid cadence band')
    if config.mash_torque_nm > 0. and config.mash_cadence_rpm <= 0.:
        raise ValueError('PedalingConfig.mash_cadence_rpm: nonzero mash torque requires positive cadence')


def shifting_config(config, gearing):
    boolean(config.enabled, 'ShiftingConfig.enabled')
    integer(gearing.front_teeth, 'DrivetrainSpecs.front_teeth', minimum=3)
    integer(gearing.rear_teeth, 'DrivetrainSpecs.rear_teeth', minimum=3)
    scalar(gearing.chain_pitch_m, 'DrivetrainSpecs.chain_pitch_m', positive=True)
    teeth = tuple(integer(value, 'ShiftingConfig.cassette', minimum=3) for value in sequence(config.cassette, 'ShiftingConfig.cassette'))
    if not teeth or len(set(teeth)) != len(teeth):
        raise ValueError('ShiftingConfig.cassette: nonempty unique teeth required')
    for key in ('target_cadence_min_rpm', 'target_cadence_max_rpm'):
        scalar(getattr(config, key), f'ShiftingConfig.{key}', positive=True)
    for key in ('shift_cooldown_s', 'shift_cut_duration_s', 'torque_factor', 'cadence_smoothing_tau_s', 'upshift_slip_limit_mps'):
        scalar(getattr(config, key), f'ShiftingConfig.{key}', minimum=0.)
    if config.target_cadence_max_rpm <= config.target_cadence_min_rpm or config.shift_cut_duration_s > config.shift_cooldown_s or config.torque_factor > 1.:
        raise ValueError('ShiftingConfig: invalid timing/cadence/torque band')
    if config.upshift_slip_mode not in ('legacy_signed', 'magnitude'):
        raise ValueError('ShiftingConfig.upshift_slip_mode: unsupported mode')
    if config.enabled and gearing.rear_teeth not in teeth:
        raise ValueError('ShiftingConfig.cassette: enabled rear gear must belong to cassette')
