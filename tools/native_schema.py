"""Explicit schemas for the resolved native wire format, before coercion."""
from collections.abc import Mapping, Sequence
from numbers import Integral
import numpy as np
from bike_sim.physics.checks import scalar

# Required/optional keys are paired with the C++ parser constants.
DAMPER = ('max_hsc max_lsc max_reb hsc_clicks lsc_clicks rebound_clicks '
          'c_lsc_min c_lsc_max c_hsc_min c_hsc_max c_reb_min c_reb_max v_knee_comp v_knee_reb')
SCHEMAS = {
    'config': ('schema', 'drive cruise suspension brake resistance tire rider_forces rider_contacts'),
    'suspension': ('physics_mode joints air_spring fork_damper shock_damper coil end_stops', ''),
    'suspension.joints': ('fork shock', ''),
    'suspension.air_spring': ('stanchion_inner_diam_mm total_travel_mm pos_chamber_length_mm neg_chamber_length_mm token_volume_cm3 max_tokens gamma atm_pressure_pa num_tokens gauge_pressure_psi', ''),
    'suspension.fork_damper': (DAMPER+' total_travel_mm hbo_start_mm c_hbo_base', 'legacy_behavior'),
    'suspension.shock_damper': (DAMPER+' total_stroke_mm max_hbo hbo_clicks lockout_firm legacy_behavior hbo_start_mm c_hbo_min c_hbo_max lockout_preload_n lockout_stiffness', ''),
    'suspension.coil': ('rate_n_m preload_mm stroke_mm bumper_length_mm bumper_peak_n legacy_behavior', ''),
    'suspension.end_stops': ('stiffness_n_m damping_n_s_m', ''),
    'brake': ('torque_ceiling_nm taper_radps', ''),
    'cruise': ('target_speed_kmh kp_nm_per_mps ki_nm_per_mps_s torque_ceiling_nm', ''),
    'resistance': ('crr rolling_taper_rad_s rho_kg_m3 cda_m2 wind_world_mps point_body_m bodies', ''),
    'resistance.bodies': ('frame front_wheel rear_wheel', ''),
    'tire': ('backend surface_mode front rear significant_delta_m significance_fraction distinct_normal_deg', 'surface_map'),
    'tire.side': ('material tangent_k_n_m mu relaxation_length_m', ''),
    'tire.side.material': ('radial_k_n_m radial_c_ns_m pressure_pa_gauge provenance valid_load_range_n', ''),
    'surface': ('name mu_peak mu_slide slip_stiffness_per_load stribeck_speed_mps', ''),
    'tire.surface_map': ('surface sections', ''),
    'tire.surface_map.sections': ('start_m end_m surface', ''),
    'rider_forces': ('paths', ''),
    'rider_forces.paths': ('joint stiffness_n_m damping_ns_m preload_deflection_m offset_m unilateral', ''),
    'drive': ('gearing pedaling shifting assist battery hub_stiffness_nm_rad hub_damping_nm_s drive_mode transmission_model human_torque_nm torque_ripple crank_phase_rad chain_k_n_m chain_c_ns_m bearing_c_nms_rad rotor_inertia_kgm2 motor_clutch', ''),
    'policies': ('gearing pedaling shifting assist battery hub_stiffness_nm_rad hub_damping_nm_s', ''),
    'drive.gearing': ('front_teeth rear_teeth chain_pitch_m', ''),
    'drive.pedaling': ('enabled coast_above_rpm resume_below_rpm stop_time_s coast_cadence_tau_s mash_cadence_rpm mash_torque_nm effort_slew_nm_s', ''),
    'drive.shifting': ('enabled cassette target_cadence_min_rpm target_cadence_max_rpm shift_cooldown_s shift_cut_duration_s torque_factor cadence_smoothing_tau_s upshift_slip_limit_mps upshift_slip_mode', ''),
    'drive.assist': ('gain max_torque max_power tau slew engage_torque_nm gate_min_crank_rad_s cutoff_mps taper_width_mps mode', 'profile torque_curve'),
    'drive.assist.profile': ('mode_gains emtb_full_gain_at_nm', ''),
    'drive.assist.profile.mode_gains': ('eco tour emtb turbo', ''),
    'drive.battery': ('enabled energy_j copper_w_per_nm2 speed_w_per_rad_s2 idle_w', ''),
    'rider_contacts': ('arm_reach_m saddle_patch_half_length_m pedal_patch_half_length_m support_pad_radius_m support_k_n_m support_c_ns_m pedal_c_ns_m support_tangent_k_n_m support_mu support_length_m grip_k_n_m grip_c_ns_m grip_release_distance_m grip_capture_distance_m grip_capture_speed_mps pedal_attachment saddle_attachment grip_attachment', 'grip_pair_force_limit_n'),
}
BOOLEAN = frozenset('enabled lockout_firm legacy_behavior unilateral motor_clutch'.split())
INTEGER = frozenset('schema max_tokens num_tokens max_hsc max_lsc max_reb hsc_clicks lsc_clicks rebound_clicks max_hbo hbo_clicks front_teeth rear_teeth'.split())
STRING = frozenset('physics_mode fork shock frame front_wheel rear_wheel provenance name backend surface_mode joint drive_mode transmission_model upshift_slip_mode mode pedal_attachment saddle_attachment grip_attachment'.split())
WIDTHS = {'wind_world_mps': 3, 'point_body_m': 3, 'valid_load_range_n': 2, 'emtb': 2}


def exact_keys(value, required, optional, path):
    if not isinstance(value, dict):
        raise ValueError(f'{path}: expected dict')
    for key in value:
        if not isinstance(key, str):
            raise ValueError(f'{path}: expected string key')
        if key not in required and key not in optional:
            raise ValueError(f'{path}.{key}: unknown key')
    for key in required:
        if key not in value:
            raise ValueError(f'{path}.{key}: missing required key')


def integer(value, path):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral):
        raise ValueError(f'{path}: expected integer')
    result = int(value)
    if not -(2**31) <= result < 2**31:
        raise ValueError(f'{path}: integer out of range')
    return result


def boolean(value, path):
    if type(value) is not bool:
        raise ValueError(f'{path}: expected bool')
    return value


def sequence(value, path):
    if isinstance(value, np.ndarray) and value.ndim == 0:
        raise ValueError(f'{path}: expected ordered sequence')
    if isinstance(value, (str, bytes, Mapping)) or not isinstance(value, (Sequence, np.ndarray)):
        raise ValueError(f'{path}: expected ordered sequence')
    return value


def _schema(name):
    if name in ('tire.front', 'tire.rear'):
        return 'tire.side'
    if name in ('tire.front.material', 'tire.rear.material'):
        return 'tire.side.material'
    if name.endswith('.surface'):
        return 'surface'
    return name


def validate_mapping(value, name, path):
    required, optional = (part.split() for part in SCHEMAS[_schema(name)])
    exact_keys(value, required, optional, path)
    for key, item in value.items():
        child = f'{path}.{key}'
        nested = f'{name}.{key}' if name not in ('config', 'policies') else key
        if name == 'policies' and key in ('gearing', 'pedaling', 'shifting', 'assist', 'battery'):
            nested = f'drive.{key}'
        if key in ('sections', 'paths'):
            for index, row in enumerate(sequence(item, child)):
                validate_mapping(row, nested, f'{child}.{index}')
        elif _schema(nested) in SCHEMAS:
            if item is None and key == 'profile':
                continue
            validate_mapping(item, nested, child)
        elif key in WIDTHS:
            values = sequence(item, child)
            if len(values) != WIDTHS[key]:
                raise ValueError(f'{child}: incorrect sequence width')
            for index, number in enumerate(values):
                scalar(number, f'{child}.{index}')
        elif key == 'cassette':
            for index, number in enumerate(sequence(item, child)):
                integer(number, f'{child}.{index}')
        elif key == 'grip_pair_force_limit_n':
            if item is not None:
                scalar(item, child)
        elif key == 'torque_curve':
            if item is not None:
                for index, row in enumerate(sequence(item, child)):
                    row_path = f'{child}.{index}'
                    values = sequence(row, row_path)
                    if len(values) != 2:
                        raise ValueError(f'{row_path}: incorrect sequence width')
                    for number in values:
                        scalar(number, row_path)
        elif key in BOOLEAN:
            boolean(item, child)
        elif key in INTEGER:
            integer(item, child)
        elif key in STRING:
            if not isinstance(item, str):
                raise ValueError(f'{child}: expected string')
            try:
                str.encode(item, 'utf-8')
            except UnicodeError as exc:
                raise ValueError(f'{child}: expected UTF-8 string') from exc
        else:
            scalar(item, child)


def validate_policy_modes(drive, path):
    if drive['shifting']['upshift_slip_mode'] not in ('legacy_signed', 'magnitude'):
        raise ValueError(f'{path}.shifting.upshift_slip_mode: unsupported mode')
    assist = drive['assist']
    if assist.get('profile') is not None and assist['mode'] not in ('eco', 'tour', 'emtb', 'turbo'):
        raise ValueError(f'{path}.assist.mode: unsupported profiled mode')


def validate_drive_policies(config):
    validate_mapping(config, 'policies', 'config.drive')
    validate_policy_modes(config, 'config.drive')


def validate_rider_contacts(contacts, path='config.rider_contacts'):
    """Domain checks for the optional rider_contacts section.

    Mirrors ArticulatedConfig.__post_init__ (physical_config.py): every copied
    scalar is finite and nonnegative, the structural lengths and stiffnesses
    its second loop names are positive, and the attachment discriminators are
    closed sets. The resolved arm reach is positive like the segment lengths
    it sums.
    """
    if contacts['pedal_attachment'] not in ('flat', 'weld', 'spindle'):
        raise ValueError(f'{path}.pedal_attachment: unsupported attachment')
    if contacts['saddle_attachment'] not in ('flat', 'weld', 'pin'):
        raise ValueError(f'{path}.saddle_attachment: unsupported attachment')
    if contacts['grip_attachment'] not in ('spring', 'connect'):
        raise ValueError(f'{path}.grip_attachment: unsupported attachment')
    scalar(contacts['arm_reach_m'], f'{path}.arm_reach_m', positive=True)
    for key in ('support_pad_radius_m', 'support_k_n_m', 'support_tangent_k_n_m',
                'support_length_m', 'grip_k_n_m', 'grip_release_distance_m'):
        scalar(contacts[key], f'{path}.{key}', positive=True)
    for key in ('saddle_patch_half_length_m', 'pedal_patch_half_length_m',
                'support_c_ns_m', 'pedal_c_ns_m', 'support_mu', 'grip_c_ns_m',
                'grip_capture_distance_m', 'grip_capture_speed_mps'):
        scalar(contacts[key], f'{path}.{key}', minimum=0.)
    limit = contacts.get('grip_pair_force_limit_n')
    if limit is not None:
        scalar(limit, f'{path}.grip_pair_force_limit_n', positive=True)


def validate_config(config):
    if not isinstance(config, dict):
        raise ValueError('config: expected dict')
    if not config:
        return
    validate_mapping(config, 'config', 'config')
    if config['schema'] not in (1, 2):
        raise ValueError('config.schema: unsupported schema')
    suspension = config.get('suspension')
    if suspension:
        if suspension['physics_mode'] not in ('legacy', 'physical'):
            raise ValueError('config.suspension.physics_mode: unsupported mode')
        fork = suspension['fork_damper']
        if config['schema'] == 2 and 'legacy_behavior' not in fork:
            raise ValueError('config.suspension.fork_damper.legacy_behavior: missing required key')
        if config['schema'] == 1 and 'legacy_behavior' in fork:
            raise ValueError('config.suspension.fork_damper.legacy_behavior: unknown schema-1 key')
        legacy = fork['hbo_start_mm'] == 160. if config['schema'] == 1 else fork['legacy_behavior']
        scalar(fork['total_travel_mm'], 'config.suspension.fork_damper.total_travel_mm', positive=True)
        start = scalar(fork['hbo_start_mm'], 'config.suspension.fork_damper.hbo_start_mm', minimum=0.)
        if not legacy and start >= fork['total_travel_mm']:
            raise ValueError('config.suspension.fork_damper.hbo_start_mm: invalid modern HBO zone')
    tire = config.get('tire')
    if tire:
        if tire['backend'] != 'compliant_2d':
            raise ValueError('config.tire.backend: unsupported backend')
        if tire['surface_mode'] not in ('configured', 'track'):
            raise ValueError('config.tire.surface_mode: unsupported mode')
        if tire['surface_mode'] == 'track' and 'surface_map' not in tire:
            raise ValueError('config.tire.surface_map: missing required key')
    drive = config.get('drive')
    if drive:
        if drive['drive_mode'] not in ('crank_effort', 'articulated_effort', 'coast', 'ideal_speed_control'):
            raise ValueError('config.drive.drive_mode: unsupported mode')
        if drive['transmission_model'] not in ('elastic_chain', 'ideal_mid_drive', 'geometric_ideal_mid_drive'):
            raise ValueError('config.drive.transmission_model: unsupported model')
        validate_policy_modes(drive, 'config.drive')
    rider_contacts = config.get('rider_contacts')
    if rider_contacts:
        validate_rider_contacts(rider_contacts)


def plain(value, key=None):
    # Called only after validation, retaining binary64 conversion and sequence order.
    if isinstance(value, dict):
        return {name: plain(item, name) for name, item in value.items()}
    if isinstance(value, (Sequence, np.ndarray)) and not isinstance(value, (str, bytes)):
        return [plain(item, key) for item in value]
    if isinstance(value, str):
        return str(value)
    if isinstance(value, bool) or value is None:
        return value
    if isinstance(value, Integral) and (key in INTEGER or key == 'cassette'):
        return int(value)
    return float(value)
