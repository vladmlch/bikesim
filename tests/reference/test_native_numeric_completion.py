"""F2 domains at mutable Python boundaries and corresponding native owners."""
import copy
from dataclasses import replace
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from native_loader import load_native
from _bits import assert_bitwise_equal
from test_native_input_contracts import resolved, at
from test_native_numeric_domains import python_suspension


@pytest.mark.parametrize('key', ['piston_area_m2', 'token_volume_m3', 'base_pos_volume_m3', 'base_neg_volume_m3'])
def test_air_properties_revalidate_mutated_specs(key):
    from bike_sim.physics.air_spring import AirSpringSpecs
    specs = AirSpringSpecs()
    specs.neg_chamber_length_mm = -1.
    with pytest.raises(ValueError):
        getattr(specs, key)


@pytest.mark.parametrize('key', ['abs_pressure_pa', 'gauge_pressure_bar'])
def test_air_pressure_properties_revalidate(key):
    from bike_sim.physics.air_spring import ForkAirSpring
    spring = ForkAirSpring()
    spring.gauge_pressure_psi = True
    with pytest.raises(ValueError):
        getattr(spring, key)


def test_air_set_tokens_revalidates_mutated_limit():
    from bike_sim.physics.air_spring import ForkAirSpring
    spring = ForkAirSpring()
    spring.specs.max_tokens = -1
    with pytest.raises(ValueError):
        spring.set_tokens(1)


@pytest.mark.parametrize('key,value', [('pressure_pa_gauge', -1.), ('provenance', ''),
    ('valid_load_range_n', (2., 1.)), ('radial_k_n_m', True)])
def test_mutated_tire_material_revalidated(key, value):
    from bike_sim.physics.tire import TireSpec
    material = TireSpec(1e5, 100., 1e5, 'synthetic', (0., 2000.))
    object.__setattr__(material, key, value)
    for evaluate in (lambda: material.elastic_response(.01), lambda: material.normal_contact(.01, .1),
                     lambda: material.is_load_in_valid_range(100.)):
        with pytest.raises(ValueError):
            evaluate()


@pytest.mark.parametrize('mutation', ['surface', 'section', 'overlap'])
def test_mutated_surface_map_revalidated(mutation):
    from bike_sim.terrain.surface import SurfaceMap, SurfaceSection, SurfaceSpec
    surface = SurfaceSpec('synthetic', .8, .6, 12.)
    mapping = SurfaceMap(surface, [SurfaceSection(0., 1., 'asphalt')])
    if mutation == 'surface':
        object.__setattr__(surface, 'mu_slide', 2.)
    elif mutation == 'section':
        object.__setattr__(mapping.sections[0], 'start_m', -1.)
    else:
        mapping.sections = (SurfaceSection(0., 2., 'asphalt'), SurfaceSection(1., 3., 'wet'))
    with pytest.raises(ValueError):
        mapping.at(.5)


@pytest.mark.parametrize('key,value', [('chain_k_n_m', 0.), ('bearing_c_nms_rad', -1.),
    ('motor_clutch', 'no'), ('torque_ripple', 1.)])
def test_mutated_drive_config_rejected_before_model_writes(tmp_path, key, value):
    from test_native_drivetrain import pair
    from bike_sim.sim.ride.drivetrain_forces import DrivetrainForceApplier
    model, _, drive, _ = pair(tmp_path)
    before = model.wrap_prm.copy()
    object.__setattr__(drive.config, key, value)
    with pytest.raises(ValueError):
        DrivetrainForceApplier(model, drive.config, drive.drive_mode)
    assert_bitwise_equal(model.wrap_prm, before)


@pytest.mark.parametrize('axis,kind', [('0 0 1', 'hinge'), ('0 1 0', 'slide')])
def test_drivetrain_wheel_topology_shared(tmp_path, axis, kind):
    from test_native_drivetrain import model_xml, pair
    from bike_sim.sim.ride.drivetrain_forces import DrivetrainForceApplier
    from tools.native_config import project_drive
    _, _, drive, _ = pair(tmp_path)
    xml = model_xml('elastic_chain').replace('<joint name="front_wheel_spin" axis="0 1 0"/>',
        f'<joint name="front_wheel_spin" type="{kind}" axis="{axis}"/>')
    model = mujoco.MjModel.from_xml_string(xml)
    path = tmp_path/'wheel-domain.mjb'
    mujoco.mj_saveModel(model, str(path))
    cfg = replace(drive.config, transmission_model='elastic_chain')
    wire = project_drive(drive)
    wire['transmission_model'] = 'elastic_chain'
    for make in (lambda: DrivetrainForceApplier(model, cfg, 'crank_effort'),
                 lambda: load_native().Stepper(str(path), {'schema':2, 'drive':wire})):
        with pytest.raises(ValueError, match='front_wheel_spin'):
            make()


@pytest.mark.parametrize('actuator', ['human_crank', 'mid_drive'])
def test_drivetrain_actuator_target_shared(tmp_path, actuator):
    from test_native_drivetrain import model_xml, pair
    from bike_sim.sim.ride.drivetrain_forces import DrivetrainForceApplier
    from tools.native_config import project_drive
    _, _, drive, _ = pair(tmp_path)
    xml = model_xml('ideal_mid_drive').replace(f'<motor name="{actuator}" joint="crank_spin"/>',
                                              f'<motor name="{actuator}" joint="front_wheel_spin"/>')
    model = mujoco.MjModel.from_xml_string(xml)
    path = tmp_path/'actuator-domain.mjb'
    mujoco.mj_saveModel(model, str(path))
    for make in (lambda: DrivetrainForceApplier(model, drive.config, 'crank_effort'),
                 lambda: load_native().Stepper(str(path), {'schema':2, 'drive':project_drive(drive)})):
        with pytest.raises(ValueError, match=actuator):
            make()


@pytest.mark.parametrize('legacy', [False, True])
def test_shock_component_overflow_shared(resolved, legacy):
    from bike_sim.physics.damper import SuperDeluxeDamper
    env, path, original = resolved
    wire = copy.deepcopy(original)
    shock = wire['suspension']['shock_damper']
    shock.update(legacy_behavior=legacy, lockout_firm=True, c_hsc_min=1e308, c_hsc_max=1e308)
    damper = SuperDeluxeDamper(legacy_behavior=legacy)
    for key, value in shock.items():
        setattr(damper, key, value)
    native = load_native().Stepper(path, wire)
    vel = env.sim.data.qvel.copy()
    vel[env.sim.applier.shock_dofadr] = 10.
    native.set_state(env.sim.data.qpos, vel, env.sim.data.act, env.sim.data.qacc_warmstart, 0.)
    for calculate in (lambda: damper.compute_damping_force(10., 30.), native.suspension_components):
        with pytest.raises(OverflowError):
            calculate()


@pytest.mark.parametrize('name', ['chain_tension', 'chain_geometry'])
def test_chain_computation_overflow_shared(name, tmp_path):
    from bike_sim.physics.chain import chain_tension, chain_geometry
    if name == 'chain_tension':
        python = lambda: chain_tension(10., 0., 1e308, 0.)
        from test_native_drivetrain import pair
        model, data, _, native_drive = pair(tmp_path, 'elastic_chain', chain_k_n_m=1e308, chain_c_ns_m=1e308)
        data.qpos[model.joint('crank_spin').qposadr[0]] += 1.
        data.qvel[model.joint('crank_spin').dofadr[0]] = 1000.
        mujoco.mj_forward(model, data)
        native_drive.set_state(data.qpos, data.qvel, data.act, data.qacc_warmstart, 0.)
        native_drive.forward()
        native = lambda: native_drive.drive_components({}, .001, 0.)
    else:
        python = lambda: chain_geometry([0., 0.], [1e308, 0.], .1, .05)
        native = lambda: load_native().chain_geometry([0., 0.], [1e308, 0.], .1, .05, [0., 1.], None)
    for calculate in (python, native):
        with pytest.raises(OverflowError):
            calculate()


@pytest.mark.parametrize('policy', ['assist', 'pedaling', 'shifting'])
def test_policy_intermediate_overflow_shared(policy):
    from test_native_drive_policies import applier, oracle_config
    drive = applier()
    wire = oracle_config(drive)
    if policy == 'assist':
        drive.assist.slew = wire['assist']['slew'] = 1e308
        python = lambda: drive.assist.step(10., 80., 0., False, 10.)
        method, args = 'assist_step', (10., 80., 0., False, 10.)
    elif policy == 'pedaling':
        object.__setattr__(drive.pedaling.config, 'stop_time_s', 1e-308)
        wire['pedaling']['stop_time_s'] = 1e-308
        python = lambda: drive.pedaling.update(0., 10., 80., 0., .001)
        method, args = 'pedaling_update', (0., 10., 80., 0., .001)
    else:
        drive.shifting.cadence_ema = drive.shifting.required_ema = 1e308
        python = lambda: drive.shifting.update(-1e308, -1e308, .001)
        method, args = 'shifting_update', (-1e308, -1e308, .001)
    native = load_native().DrivePolicies(wire)
    if policy == 'shifting':
        state = native.state()
        state['shifting']['cadence_ema'] = state['shifting']['required_ema'] = 1e308
        native.set_state(state)
    for calculate in (python, lambda: getattr(native, method)(*args)):
        with pytest.raises(OverflowError):
            calculate()


def test_cruise_overflow_preserves_finite_telemetry(tmp_path):
    from bike_sim.sim.ride.cruise import CruiseController
    from test_native_cruise import CONFIG
    model = mujoco.MjModel.from_xml_string('<mujoco><option timestep="1e308"/><worldbody><body><joint name="root_x" type="slide" axis="1 0 0"/><geom size=".1" mass="1"/></body></worldbody></mujoco>')
    data = mujoco.MjData(model)
    path = tmp_path/'cruise-overflow.mjb'
    mujoco.mj_saveModel(model, str(path))
    config = {**CONFIG, 'kp_nm_per_mps':1e-308, 'ki_nm_per_mps_s':1e-308}
    controller = CruiseController(model, **config)
    native = load_native().Stepper(str(path), {'schema':2, 'cruise':config})
    state = native.cruise_state()
    for calculate in (lambda: controller.compute(model, data, SimpleNamespace(rear_in_contact=True)),
                      lambda: native.cruise_compute(True)):
        with pytest.raises(OverflowError):
            calculate()
    assert controller.integral_mps_s == 0.
    assert not controller.engaged
    assert native.cruise_state() == state


@pytest.mark.parametrize('section,key', [('fork_damper', 'v_knee_comp'), ('shock_damper', 'max_lsc')])
def test_native_damper_errors_name_full_field_path(resolved, section, key):
    _, path, original = resolved
    wire = copy.deepcopy(original)
    wire['suspension'][section][key] = 0
    with pytest.raises(ValueError, match=f'config.suspension.{section}.{key}'):
        load_native().Stepper(path, wire)


@pytest.mark.parametrize('location,value', [
    (('tire', 'front', 'material', 'radial_c_ns_m'), -1.),
    (('tire', 'rear', 'relaxation_length_m'), 0.),
    (('drive', 'assist', 'profile', 'mode_gains', 'eco'), -1.),
    (('rider_forces', 'paths', 0, 'stiffness_n_m'), -1.),
])
def test_nested_numeric_errors_name_full_field_path(resolved, location, value):
    _, path, original = resolved
    wire = copy.deepcopy(original)
    at(wire, location[:-1])[location[-1]] = value
    public_path = 'config.'+'.'.join(str(part) for part in location)
    with pytest.raises(ValueError, match=public_path):
        load_native().Stepper(path, wire)


def test_end_stop_strict_types_and_computation_overflow():
    from bike_sim.physics.stops import end_stop
    with pytest.raises(ValueError):
        end_stop(0., 0., 0., .1, True, 0.)
    with pytest.raises(OverflowError):
        end_stop(-10., 0., 0., .1, 1e308, 0.)


POSITIVE_FIELDS = [
    ('suspension', 'fork_damper', 'total_travel_mm'),
    ('suspension', 'shock_damper', 'total_stroke_mm'), ('suspension', 'shock_damper', 'max_hbo'),
    ('brake', 'taper_radps'), ('cruise', 'kp_nm_per_mps'), ('cruise', 'ki_nm_per_mps_s'), ('cruise', 'torque_ceiling_nm'),
    ('resistance', 'rolling_taper_rad_s'),
    *(('tire', side, key) for side in ('front', 'rear') for key in ('tangent_k_n_m', 'relaxation_length_m')),
    *(('tire', side, 'material', 'radial_k_n_m') for side in ('front', 'rear')),
    *(('tire', 'surface_map', 'surface', key) for key in ('mu_peak', 'mu_slide', 'slip_stiffness_per_load', 'stribeck_speed_mps')),
    ('drive', 'gearing', 'chain_pitch_m'), ('drive', 'chain_k_n_m'), ('drive', 'hub_stiffness_nm_rad'),
    *(('drive', 'pedaling', key) for key in ('coast_above_rpm', 'stop_time_s')),
    *(('drive', 'shifting', key) for key in ('target_cadence_min_rpm', 'target_cadence_max_rpm')),
    *(('drive', 'assist', key) for key in ('tau', 'slew', 'taper_width_mps')),
]
NONNEGATIVE_FIELDS = [
    ('suspension', 'fork_damper', 'hbo_start_mm'), ('suspension', 'fork_damper', 'c_hbo_base'),
    *(('suspension', 'shock_damper', key) for key in ('hbo_start_mm', 'c_hbo_min', 'c_hbo_max', 'lockout_preload_n', 'lockout_stiffness')),
    ('brake', 'torque_ceiling_nm'), *(('resistance', key) for key in ('crr', 'rho_kg_m3', 'cda_m2')),
    *(('tire', side, 'mu') for side in ('front', 'rear')),
    *(('tire', side, 'material', key) for side in ('front', 'rear') for key in ('radial_c_ns_m', 'pressure_pa_gauge')),
    ('tire', 'significant_delta_m'), ('tire', 'significance_fraction'),
    *(('drive', key) for key in ('human_torque_nm', 'torque_ripple', 'chain_c_ns_m', 'bearing_c_nms_rad', 'rotor_inertia_kgm2', 'hub_damping_nm_s')),
    *(('drive', 'pedaling', key) for key in ('resume_below_rpm', 'coast_cadence_tau_s', 'mash_cadence_rpm', 'mash_torque_nm', 'effort_slew_nm_s')),
    *(('drive', 'shifting', key) for key in ('shift_cooldown_s', 'shift_cut_duration_s', 'torque_factor', 'cadence_smoothing_tau_s', 'upshift_slip_limit_mps')),
    *(('drive', 'assist', key) for key in ('gain', 'max_torque', 'max_power', 'engage_torque_nm', 'gate_min_crank_rad_s', 'cutoff_mps')),
    *(('drive', 'battery', key) for key in ('energy_j', 'copper_w_per_nm2', 'speed_w_per_rad_s2', 'idle_w')),
]


def python_wire_config(env, wire):
    from bike_sim.physics.chain import DrivetrainSpecs
    from bike_sim.physics.motor import AssistController
    from bike_sim.physics.physical_config import (PedalingConfig, ShiftingConfig, BatteryConfig,
        TireParameters, TireBackendConfig, ResistanceConfig, PhysicalDriveConfig)
    from bike_sim.physics.tire import TireSpec
    from bike_sim.terrain.surface import SurfaceSpec
    from bike_sim.sim.ride.braking import BrakeController
    from bike_sim.sim.ride.cruise import CruiseController
    python_suspension(wire['suspension'])
    BrakeController(env.sim.model, **wire['brake'])
    CruiseController(env.sim.model, **wire['cruise'])
    ResistanceConfig(**{k:v for k,v in wire['resistance'].items() if k != 'bodies'})
    tire = wire['tire']
    sides = {side:TireParameters(material=TireSpec(**tire[side]['material']),
        **{k:v for k,v in tire[side].items() if k != 'material'}) for side in ('front', 'rear')}
    TireBackendConfig(**sides, **{k:v for k,v in tire.items() if k not in ('front', 'rear', 'surface_map')})
    SurfaceSpec(**tire['surface_map']['surface'])
    drive = wire['drive']
    DrivetrainSpecs(**drive['gearing'])
    PedalingConfig(**drive['pedaling'])
    ShiftingConfig(**drive['shifting'])
    BatteryConfig(**drive['battery'])
    AssistController(**drive['assist'])
    PhysicalDriveConfig(**{k:v for k,v in drive.items() if k in PhysicalDriveConfig.__dataclass_fields__ and
        k not in ('gearing', 'pedaling', 'shifting', 'assist', 'battery')},
        freehub_k_nm_rad=drive['hub_stiffness_nm_rad'], freehub_c_nms_rad=drive['hub_damping_nm_s'])


@pytest.mark.parametrize('location', POSITIVE_FIELDS + NONNEGATIVE_FIELDS)
@pytest.mark.parametrize('bad', [-1., np.nan, np.inf, -np.inf, True])
def test_remaining_s4_scalar_field_matrix(resolved, location, bad):
    env, path, original = resolved
    wire = copy.deepcopy(original)
    wire['drive']['assist']['profile'] = None
    at(wire, location[:-1])[location[-1]] = bad
    for construct in (lambda: python_wire_config(env, wire), lambda: load_native().Stepper(path, wire)):
        with pytest.raises(ValueError):
            construct()


@pytest.mark.parametrize('location', POSITIVE_FIELDS)
def test_remaining_s4_positive_field_zero(resolved, location):
    env, path, original = resolved
    wire = copy.deepcopy(original)
    wire['drive']['assist']['profile'] = None
    at(wire, location[:-1])[location[-1]] = 0
    for construct in (lambda: python_wire_config(env, wire), lambda: load_native().Stepper(path, wire)):
        with pytest.raises(ValueError):
            construct()


def test_s4_matrix_valid_configuration(resolved):
    env, path, original = resolved
    wire = copy.deepcopy(original)
    wire['drive']['assist']['profile'] = None
    python_wire_config(env, wire)
    load_native().Stepper(path, wire)


@pytest.mark.parametrize('side', ['front', 'rear'])
def test_unicode_blank_tire_provenance_shared(resolved, side):
    from bike_sim.physics.tire import TireSpec
    _, path, original = resolved
    wire = copy.deepcopy(original)
    material = wire['tire'][side]['material']
    material['provenance'] = '\u2003'
    for construct in (lambda: TireSpec(**material), lambda: load_native().Stepper(path, wire)):
        with pytest.raises(ValueError, match='provenance'):
            construct()


@pytest.mark.parametrize('side', ['frame', 'front_wheel', 'rear_wheel'])
def test_resistance_requires_physical_bodies(resolved, side):
    _, path, original = resolved
    wire = copy.deepcopy(original)
    wire['resistance']['bodies'][side] = 'world'
    with pytest.raises(ValueError):
        load_native().Stepper(path, wire)


@pytest.mark.parametrize('field,value', [('cassette', {12, 24}), ('cassette', (24, True)), ('cassette', (24, 24)),
    ('cassette', (np.int64(24), np.int64(32))), ('cassette', [12, 24])])
def test_cassette_ordered_integer_contract(field, value):
    from bike_sim.physics.physical_config import ShiftingConfig
    from test_native_drive_policies import applier, oracle_config
    wire = oracle_config(applier())
    wire['shifting'][field] = value
    valid = isinstance(value, (list, tuple)) and value != (24, True) and value != (24, 24)
    for construct in (lambda: ShiftingConfig(**wire['shifting']), lambda: load_native().DrivePolicies(wire)):
        if valid:
            construct()
        else:
            with pytest.raises(ValueError):
                construct()


@pytest.mark.parametrize('name,key', [('front', 'mu'), ('rear', 'tangent_k_n_m')])
def test_mutated_tire_parameters_revalidated_before_compute(resolved, name, key):
    env, _, _ = resolved
    tire = copy.deepcopy(env.sim.physical.tire)
    parameters = getattr(tire.config, name)
    object.__setattr__(parameters, key, -1.)
    with pytest.raises(ValueError):
        tire.compute_qfrc(env.sim.model, env.sim.data, .001)


def test_damper_mutated_denominator_rejected_by_setter():
    from bike_sim.physics.damper import Charger3Damper
    damper = Charger3Damper()
    damper.max_hsc = 0
    with pytest.raises(ValueError):
        damper.set_clicks(hsc=1)


@pytest.mark.parametrize('location,value', [
    (('drive', 'assist', 'taper_width_mps'), 100.),
    (('drive', 'assist', 'torque_curve'), [[0., 1.], [0., 2.]]),
    (('drive', 'shifting', 'cassette'), [24, 24]),
    (('drive', 'shifting', 'target_cadence_min_rpm'), 200.),
    (('drive', 'shifting', 'shift_cut_duration_s'), 100.),
    (('drive', 'shifting', 'torque_factor'), 1.1),
    (('drive', 'pedaling', 'resume_below_rpm'), 200.),
    (('tire', 'significance_fraction'), 1.1), (('tire', 'distinct_normal_deg'), 180.),
    (('tire', 'front', 'material', 'valid_load_range_n'), [2000., 1000.]),
    (('tire', 'surface_map', 'surface', 'mu_slide'), 20.),
    (('suspension', 'fork_damper', 'c_lsc_min'), 1e6),
    (('suspension', 'shock_damper', 'c_hbo_min'), 1e6),
    (('suspension', 'coil', 'bumper_length_mm'), 100.),
])
def test_s4_relationship_matrix(resolved, location, value):
    env, path, original = resolved
    wire = copy.deepcopy(original)
    wire['drive']['assist']['profile'] = None
    at(wire, location[:-1])[location[-1]] = value
    for construct in (lambda: python_wire_config(env, wire), lambda: load_native().Stepper(path, wire)):
        with pytest.raises(ValueError):
            construct()


@pytest.mark.parametrize('key', ['stiffness_n_m', 'damping_ns_m', 'preload_deflection_m', 'offset_m'])
@pytest.mark.parametrize('bad', [np.nan, np.inf, -np.inf, True])
def test_rider_path_scalar_matrix_shared(resolved, key, bad):
    from bike_sim.sim.ride.rider_forces import _JointPath
    _, path, original = resolved
    wire = copy.deepcopy(original)
    row = wire['rider_forces']['paths'][0]
    row[key] = bad
    body = SimpleNamespace(**{k:v for k,v in row.items() if k not in ('offset_m', 'joint')})
    joint = _JointPath(body, 0, 0)
    joint.offset_m = row['offset_m']
    for calculate in (lambda: joint.compute(0., 0.), lambda: load_native().Stepper(path, wire)):
        with pytest.raises(ValueError):
            calculate()


@pytest.mark.parametrize('attribute,value', [('rated_power_w', 1e6), ('taper_width_mps', 100.)])
def test_mutated_motor_profile_relationships_revalidated(attribute, value):
    from bike_sim.physics.motor_profile import BOSCH_CX_GEN4
    from bike_sim.physics.motor import AssistController
    profile = replace(BOSCH_CX_GEN4)
    controller = AssistController(profile=profile)
    object.__setattr__(profile, attribute, value)
    with pytest.raises(ValueError):
        controller.ceiling(80., 0.)


def test_bearing_force_overflow_before_return_and_telemetry(tmp_path):
    from test_native_drivetrain import pair
    model, data, python, native = pair(tmp_path, bearing_c_nms_rad=1e308)
    data.qvel[model.joint('crank_spin').dofadr[0]] = 100.
    mujoco.mj_forward(model, data)
    native.set_state(data.qpos, data.qvel, data.act, data.qacc_warmstart, 0.)
    native.forward()
    for calculate in (lambda: python.compute_components(model, data, .001, speed_mps=0.),
                      lambda: native.drive_components({}, .001, 0.)):
        with pytest.raises(OverflowError):
            calculate()


def test_chain_dissipation_overflow_before_telemetry(tmp_path):
    from test_native_drivetrain import pair
    from bike_sim.physics.physical_config import BatteryConfig
    model, data, python, native = pair(tmp_path, 'elastic_chain', chain_k_n_m=1e308, chain_c_ns_m=1e108,
                                      battery=BatteryConfig(enabled=False))
    data.qpos[model.joint('crank_spin').qposadr[0]] += 1.
    data.qvel[model.joint('crank_spin').dofadr[0]] = -1e200
    mujoco.mj_forward(model, data)
    native.set_state(data.qpos, data.qvel, data.act, data.qacc_warmstart, 0.)
    native.forward()
    for calculate in (lambda: python.compute_components(model, data, .001, speed_mps=0., active=False),
                      lambda: native.drive_components({}, .001, 0., active=False)):
        with pytest.raises(OverflowError, match='dissipation'):
            calculate()


def test_suspension_total_overflow_before_telemetry(resolved):
    from bike_sim.physics.air_spring import AirSpringSpecs, ForkAirSpring
    env, path, original = resolved
    wire = copy.deepcopy(original)
    wire['suspension']['air_spring'].update(stanchion_inner_diam_mm=1100., num_tokens=0, gauge_pressure_psi=1e304)
    wire['suspension']['fork_damper'].update(c_lsc_min=1e308, c_lsc_max=1e308, v_knee_comp=1.)
    python = copy.deepcopy(env.sim.applier)
    python.controller.air_spring = ForkAirSpring(AirSpringSpecs(stanchion_inner_diam_mm=1100.), 0, 1e304)
    damper = python.controller.suspension_system.fork_damper
    damper.c_lsc_min = damper.c_lsc_max = 1e308
    damper.v_knee_comp = 1.
    data = mujoco.MjData(env.sim.model)
    data.qpos[:] = env.sim.data.qpos
    data.qpos[python.fork_qposadr] = .1
    data.qvel[python.fork_dofadr] = .9
    native = load_native().Stepper(path, wire)
    native.set_state(data.qpos, data.qvel, data.act, data.qacc_warmstart, 0.)
    before = python.fork_total_n
    for calculate in (lambda: python.compute_qfrc_components(env.sim.model, data), native.suspension_components):
        with pytest.raises(OverflowError):
            calculate()
    assert python.fork_total_n == before


def test_end_stop_mutated_property_revalidated():
    from bike_sim.physics.model_config import EndStopConfig
    config = EndStopConfig()
    object.__setattr__(config, 'reference_deflection_m', -1.)
    with pytest.raises(ValueError):
        _ = config.stiffness_n_m


def test_tire_two_wheel_loss_overflow_before_publication(resolved):
    from bike_sim.sim.ride.tire_forces import _BrushState
    from test_native_tire import CANONICAL_NAMES
    env, path, wire = resolved
    python = copy.deepcopy(env.sim.physical.tire)
    python.reset()
    data = mujoco.MjData(env.sim.model)
    data.qpos[:] = env.sim.data.qpos
    data.qpos[env.sim.model.joint('root_z').qposadr[0]] += 10.
    mujoco.mj_forward(env.sim.model, data)
    native = load_native().Stepper(path, wire)
    native.set_state(data.qpos, data.qvel, data.act, data.qacc_warmstart, 0.)
    native.forward()
    row = np.full(22, np.nan)
    row[0] = row[11] = 1.1e152
    native.set_tire_state(CANONICAL_NAMES, row)
    python.states = {side:_BrushState(xi=1.1e152) for side in ('front', 'rear')}
    for calculate in (lambda: python.compute_qfrc(env.sim.model, data, .001), lambda: native.tire_qfrc(.001)):
        with pytest.raises(OverflowError):
            calculate()


@pytest.mark.parametrize('tokens,clicks', [(-100, -100), (100, 100)])
def test_integer_normalization_and_legacy_hbo_bitwise_controls(resolved, tokens, clicks):
    from bike_sim.physics.air_spring import ForkAirSpring
    from bike_sim.physics.damper import Charger3Damper
    env, path, original = resolved
    wire = copy.deepcopy(original)
    wire['suspension']['air_spring']['num_tokens'] = tokens
    fork = wire['suspension']['fork_damper']
    fork.update(total_travel_mm=150., hbo_start_mm=160., legacy_behavior=True,
                hsc_clicks=clicks, lsc_clicks=clicks, rebound_clicks=clicks)
    python = copy.deepcopy(env.sim.applier)
    python.controller.air_spring.set_tokens(tokens)
    python.controller.suspension_system.fork_damper = Charger3Damper(clicks, clicks, clicks, 150., legacy_behavior=True)
    native = load_native().Stepper(path, wire)
    data = mujoco.MjData(env.sim.model)
    data.qpos[:] = env.sim.data.qpos
    data.qpos[python.fork_qposadr] = .17
    data.qvel[python.fork_dofadr] = .7
    native.set_state(data.qpos, data.qvel, data.act, data.qacc_warmstart, 0.)
    # Only the changed fork fields participate; other resolved writers retain
    # their own independently checked golden controls.
    expected = python.compute_qfrc_components(env.sim.model, data)
    actual = native.suspension_components()
    for name in ('fork_spring', 'fork_damper'):
        assert_bitwise_equal(actual[name], expected[name])


def test_air_unreachable_declared_travel_remains_usable():
    from bike_sim.physics.air_spring import AirSpringSpecs, ForkAirSpring
    spring = ForkAirSpring(AirSpringSpecs(total_travel_mm=1000.), 0)
    assert np.isfinite(spring.compute_axial_force(50.))


def test_air_calibration_denominator_overflow_before_pressure_commit():
    from bike_sim.physics.air_spring import AirSpringSpecs, ForkAirSpring
    specs = AirSpringSpecs(stanchion_inner_diam_mm=2e103, total_travel_mm=1e-100,
        pos_chamber_length_mm=2e-100, neg_chamber_length_mm=1e-100,
        token_volume_cm3=0., max_tokens=0, default_tokens=0, gamma=500.)
    spring = ForkAirSpring(specs, 0, 82.)
    with pytest.raises(OverflowError):
        spring.calibrate_psi_for_sag(1e-100, 322.2)
    assert spring.gauge_pressure_psi == 82.


def test_air_force_curve_energy_overflow_before_publication():
    from bike_sim.physics.air_spring import AirSpringSpecs, ForkAirSpring
    spring = ForkAirSpring(AirSpringSpecs(stanchion_inner_diam_mm=2500.), 0, 1e303)
    assert np.isfinite(spring.compute_axial_force(180.))
    with pytest.raises(OverflowError):
        spring.compute_force_curve(n_points=101)


def test_air_calibration_and_curve_valid_controls():
    from bike_sim.physics.air_spring import ForkAirSpring
    spring = ForkAirSpring()
    pressure = spring.calibrate_psi_for_sag(54., 322.2)
    assert np.isfinite(pressure)
    assert spring.gauge_pressure_psi == pressure
    curve = spring.compute_force_curve(n_points=11)
    assert np.isfinite(curve['energy_j']).all()
    assert curve['max_energy_j'] == curve['energy_j'][-1]


@pytest.mark.parametrize('implementation', ['python', 'native'])
def test_battery_cap_numerator_overflow_before_saturation(implementation):
    from bike_sim.physics.battery import limit_torque_by_energy
    from test_native_drive_policies import applier, oracle_config
    wire = oracle_config(applier())
    wire['battery'].update(copper_w_per_nm2=1e-306, speed_w_per_rad_s2=0., idle_w=0.)
    native = load_native().DrivePolicies(wire)
    calculate = (lambda: limit_torque_by_energy(1e308, 0., 1e-306, 0., 0., 1e308)) if implementation == 'python' else (
        lambda: native.energy_limit(1e308, 0., 1e308))
    with pytest.raises(OverflowError):
        calculate()


@pytest.mark.parametrize('implementation', ['python', 'native'])
def test_mutated_zero_dimensional_cassette_is_value_error(implementation):
    from test_native_drive_policies import applier, oracle_config
    drive = applier()
    wire = oracle_config(drive)
    wire['shifting']['cassette'] = np.array(24)
    object.__setattr__(drive.shifting.config, 'cassette', np.array(24))
    calculate = (lambda: drive.shifting.update(80., 80., .001)) if implementation == 'python' else (
        lambda: load_native().DrivePolicies(wire))
    with pytest.raises(ValueError, match='cassette'):
        calculate()


@pytest.mark.parametrize('method', ['brake_torques', 'apply_brake', 'cruise_set_assist_compensation'])
@pytest.mark.parametrize('bad', [True, np.bool_(True), np.array(.5)])
@pytest.mark.parametrize('implementation', ['python', 'native'])
def test_brake_and_cruise_scalars_reject_original_types(resolved, method, bad, implementation):
    env, path, wire = resolved
    native = load_native().Stepper(path, wire)
    if method == 'cruise_set_assist_compensation':
        from bike_sim.sim.ride.cruise import CruiseController
        controller = CruiseController(env.sim.model, **wire['cruise'])
        python = lambda: controller.set_assist_compensation(bad)
        calculate = python if implementation == 'python' else lambda: native.cruise_set_assist_compensation(bad)
    else:
        python = lambda: env.sim.brakes.compute(env.sim.data, bad, .5)
        calculate = python if implementation == 'python' else lambda: getattr(native, method)(bad, .5)
    with pytest.raises(ValueError):
        calculate()


def test_cruise_target_rejects_zero_dimensional_scalar(resolved):
    from bike_sim.sim.ride.cruise import CruiseController
    env, path, wire = resolved
    python = CruiseController(env.sim.model, **wire['cruise'])
    native = load_native().Stepper(path, wire)
    for calculate in (lambda: setattr(python, 'target_speed_kmh', np.array(25.)),
                      lambda: native.cruise_set_target_speed(np.array(25.))):
        with pytest.raises(ValueError):
            calculate()


def test_brake_and_cruise_numpy_numeric_controls(resolved):
    from bike_sim.sim.ride.cruise import CruiseController
    env, path, wire = resolved
    native = load_native().Stepper(path, wire)
    native.set_state(env.sim.data.qpos, env.sim.data.qvel, env.sim.data.act, env.sim.data.qacc_warmstart, 0.)
    demands = (np.float64(-.5), np.float64(1.5))
    assert_bitwise_equal(native.brake_torques(*demands), env.sim.brakes.compute(env.sim.data, *demands))
    native.apply_brake(*demands)
    python = CruiseController(env.sim.model, **wire['cruise'])
    assert_bitwise_equal(native.cruise_set_assist_compensation(np.float64(2.)), python.set_assist_compensation(np.float64(2.)))
    native.cruise_set_target_speed(np.float64(30.))
    python.target_speed_kmh = np.float64(30.)
    assert_bitwise_equal(native.cruise_state()['target_speed_mps'], python.target_speed_mps)


@pytest.mark.parametrize('branch', ['lower', 'upper'])
@pytest.mark.parametrize('implementation', ['python', 'native'])
def test_end_stop_unloading_overflow_before_clamp(resolved, branch, implementation):
    from bike_sim.physics.stops import end_stop
    env, path, original = resolved
    wire = copy.deepcopy(original)
    wire['suspension']['end_stops'].update(stiffness_n_m=0., damping_n_s_m=1e308)
    native = load_native().Stepper(path, wire)
    qpos, qvel = env.sim.data.qpos.copy(), np.zeros(env.sim.model.nv)
    qpos[env.sim.applier.shock_qposadr] = -.01 if branch == 'lower' else wire['suspension']['coil']['stroke_mm']/1000.+.001
    qvel[env.sim.applier.shock_dofadr] = 100. if branch == 'lower' else -100.
    native.set_state(qpos, qvel, env.sim.data.act, env.sim.data.qacc_warmstart, 0.)
    calculate = (lambda: end_stop(-.01 if branch == 'lower' else 1.01,
        100. if branch == 'lower' else -100., 0., 1., 0., 1e308)) if implementation == 'python' else native.suspension_components
    with pytest.raises(OverflowError):
        calculate()


@pytest.mark.parametrize('section,field', [('fork_damper', 'total_travel_mm'), ('shock_damper', 'total_stroke_mm')])
@pytest.mark.parametrize('bad', ['180', pytest.param(10**1000, id='out-of-range-real')])
@pytest.mark.parametrize('implementation', ['python', 'native'])
def test_damper_constructor_original_type_class(resolved, section, field, bad, implementation):
    from bike_sim.physics.damper import Charger3Damper, SuperDeluxeDamper
    _, path, original = resolved
    wire = copy.deepcopy(original)
    wire['suspension'][section][field] = bad
    cls = Charger3Damper if section == 'fork_damper' else SuperDeluxeDamper
    calculate = (lambda: cls(**{field:bad})) if implementation == 'python' else lambda: load_native().Stepper(path, wire)
    with pytest.raises(ValueError):
        calculate()


def test_assist_constructor_revalidates_mutated_profile():
    from bike_sim.physics.motor_profile import BOSCH_CX_GEN4
    from bike_sim.physics.motor import AssistController
    profile = replace(BOSCH_CX_GEN4)
    object.__setattr__(profile, 'mode_gains', None)
    with pytest.raises(ValueError):
        AssistController(profile=profile)


def test_assist_constructor_profile_slew_overflow_before_publication():
    from bike_sim.physics.motor_profile import BOSCH_CX_GEN4
    from bike_sim.physics.motor import AssistController
    profile = replace(BOSCH_CX_GEN4, peak_torque_nm=1e308, torque_tau_s=1e-308)
    with pytest.raises(OverflowError):
        AssistController(profile=profile)


@pytest.mark.parametrize('kind,value,station,expected', [
    ('fork', np.float32(180.2), 170.4, 943.9136962890625),
    ('shock', np.float32(65.2), 58., 2124.9107593837166),
])
def test_numpy_float32_damper_constructor_retains_baseline_bits(kind, value, station, expected):
    from bike_sim.physics.damper import Charger3Damper, SuperDeluxeDamper
    # Captured from an isolated module containing exact 957e4f0 damper.py.
    # Keep this historical scalar path independent of current constructor code.
    damper = Charger3Damper(total_travel_mm=value) if kind == 'fork' else SuperDeluxeDamper(total_stroke_mm=value)
    assert type(damper.hbo_start_mm) is np.float32
    assert_bitwise_equal(damper.compute_damping_force(.7, station), expected)


def test_numpy_float32_air_pressure_override_retains_baseline_bits():
    from bike_sim.physics.air_spring import ForkAirSpring
    force = ForkAirSpring().compute_axial_force(np.float32(54.2), p_gauge_psi=np.float32(83.2))
    assert_bitwise_equal(force, 316.69534273212724)


def test_numpy_float32_assist_ceiling_retains_baseline_bits():
    from bike_sim.physics.motor import AssistController
    ceiling, taper = AssistController().ceiling(np.float32(86.7), np.float32(2.3))
    assert_bitwise_equal(ceiling, np.float32(55.070915))
    assert_bitwise_equal(taper, 1.)


def test_numpy_float32_assist_metadata_retains_baseline_bits():
    from bike_sim.physics.motor import AssistController
    from bike_sim.physics.motor_profile import BOSCH_CX_GEN4
    profile = replace(BOSCH_CX_GEN4, peak_torque_nm=np.float32(85.2), torque_tau_s=np.float32(.041),
        cutoff_mps=np.float32(25/3.6), taper_width_mps=np.float32(2/3.6))
    torque = AssistController(profile=profile).step(20., 80., 6.6, False, .00031)
    assert_bitwise_equal(torque, 0.31756892800331116)


def test_numpy_float32_pedaling_metadata_retains_baseline_bits():
    from bike_sim.physics.pedaling import PedalingPolicy
    from bike_sim.physics.physical_config import PedalingConfig
    config = PedalingConfig(enabled=True, stop_time_s=np.float32(1.01), coast_cadence_tau_s=np.float32(.051), effort_slew_nm_s=np.float32(400.))
    state = PedalingPolicy(config).update(.1, 8.2, 80., 0., .001)
    assert_bitwise_equal(state.target_rate_rad_s, 8.19188117980957)
    assert_bitwise_equal(state.target_phase_rad, 0.1081959405899048)


def test_numpy_float32_shifting_metadata_retains_baseline_bits():
    from bike_sim.physics.shifting import CadenceShifter
    from bike_sim.physics.physical_config import ShiftingConfig
    from bike_sim.physics.chain import DrivetrainSpecs
    config = ShiftingConfig(enabled=True, cadence_smoothing_tau_s=np.float32(.051), shift_cooldown_s=0., shift_cut_duration_s=0.)
    shifter = CadenceShifter(DrivetrainSpecs(), config)
    for cadence in (110.1, 111.2):
        shifter.update(cadence, 120.2, .001)
    assert shifter.rear_teeth == 18
    assert_bitwise_equal(shifter.required_ema, np.float32(90.40253))


@pytest.mark.parametrize('kind,station,expected', [('fork', np.float32(170.4), 969.148193359375),
    ('shock', np.float32(58.2), 2307.181633407154)])
def test_numpy_float32_damper_station_retains_baseline_bits(kind, station, expected):
    from bike_sim.physics.damper import Charger3Damper, SuperDeluxeDamper
    damper = Charger3Damper() if kind == 'fork' else SuperDeluxeDamper()
    assert_bitwise_equal(damper.compute_damping_force(.7, station), expected)


def test_numpy_float32_rider_metadata_retains_baseline_bits():
    from bike_sim.physics.rider import RiderBody
    from bike_sim.sim.ride.rider_forces import _JointPath
    body = RiderBody(name='probe', parent='frame', attach=np.zeros(3), joint='probe', geoms=(),
        stiffness_n_m=np.float32(1000.2), damping_ns_m=np.float32(10.2), supported_mass_kg=np.float32(6.2),
        unilateral=False, interface='saddle')
    assert_bitwise_equal(body.preload_n, np.float32(60.822002))
    assert_bitwise_equal(_JointPath(body, 0, 0).compute(.00127, -.25), np.float32(62.10175))


def test_numpy_float32_end_stop_metadata_retains_baseline_bits():
    from bike_sim.physics.model_config import EndStopConfig
    config = EndStopConfig(reference_force_n=np.float32(7000.2), reference_deflection_m=np.float32(.0102), overtravel_m=.02)
    assert_bitwise_equal(config.stiffness_n_m, np.float32(686294.1))


def test_numpy_float32_end_stop_force_retains_baseline_bits():
    from bike_sim.physics.stops import end_stop
    force, energy = end_stop(*(np.float32(value) for value in (-.012, -.21, 0., .1, 1000.2, 10.2)))
    assert_bitwise_equal(force, np.float32(14.144401))
    assert_bitwise_equal(energy, np.float32(.07201441))


@pytest.mark.parametrize('speed', [np.float32(1.), np.array([1., 2.], dtype=np.float32)])
def test_numpy_float32_surface_law_retains_baseline_bits(speed):
    from bike_sim.terrain.surface import SurfaceSpec
    surface = SurfaceSpec('probe', .8, .6, 12.)
    # The original law evaluates NumPy arithmetic at the input precision.
    expected = .6 + (.8-.6)*np.exp(-np.abs(speed)/4.5)
    if np.ndim(expected) == 0:
        expected = float(expected)
    assert_bitwise_equal(surface.mu(speed), expected)


def test_numpy_float32_surface_map_query_retains_baseline_branch():
    from bike_sim.terrain.surface import SurfaceMap, SurfaceSection, get_surface
    mapping = SurfaceMap(get_surface('asphalt'), [SurfaceSection(0., 1.00000003, 'hardpack')])
    assert mapping.at(np.float32(1.)).name == 'asphalt'
    assert mapping.at(1.).name == 'hardpack'


@pytest.mark.parametrize('kind,axis', [('hinge', '0 0 1'), ('slide', '1 0 0')])
def test_generic_one_way_scalar_preload_is_axis_agnostic(kind, axis):
    from bike_sim.sim.ride.ideal_freehub import IdealFreehubConstraint
    from bike_sim.sim.ride.equilibrium_cache import _state_arrays, restore_state
    model = mujoco.MjModel.from_xml_string(f'''<mujoco><option gravity="0 0 0"/>
      <worldbody><body><joint name="a" type="{kind}" axis="{axis}"/>
      <inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/></body>
      <body><joint name="b" type="{kind}" axis="{axis}"/>
      <inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/></body></worldbody>
      <tendon><fixed name="coupling" limited="true" range="-10 0"><joint joint="a" coef="1"/>
      <joint joint="b" coef="-1"/></fixed></tendon></mujoco>''')
    data = mujoco.MjData(model)
    hub = IdealFreehubConstraint(model, 1., tendon_name='coupling', driver='a', driven='b')
    hub.boundary = -.001
    model.tendon_range[hub.tendon_id, 1] = hub.boundary
    mujoco.mj_forward(model, data)
    before = data.qacc.copy()
    runtime = SimpleNamespace(sim=SimpleNamespace(model=model), rider_contacts=None, tire=None,
        drive=SimpleNamespace(ideal_hub=hub, clutch=None))
    state = _state_arrays(runtime)
    hub.reset(model, data)
    mujoco.mj_forward(model, data)
    assert not np.allclose(data.qacc, before)
    assert restore_state(runtime, state)
    mujoco.mj_forward(model, data)
    assert_bitwise_equal(data.qacc, before)


def test_zero_disable_policy_fields_remain_bitwise_valid(tmp_path):
    from bike_sim.physics.physical_config import PedalingConfig, ShiftingConfig, BatteryConfig, AssistConfig
    from test_native_drivetrain import pair
    from test_native_drive_policies import assert_tree
    config = dict(pedaling=PedalingConfig(coast_cadence_tau_s=0., effort_slew_nm_s=0., mash_torque_nm=0., mash_cadence_rpm=0.),
        shifting=ShiftingConfig(shift_cooldown_s=0., shift_cut_duration_s=0., cadence_smoothing_tau_s=0., upshift_slip_limit_mps=0.),
        battery=BatteryConfig(energy_j=0., copper_w_per_nm2=0., speed_w_per_rad_s2=0., idle_w=0.),
        assist=AssistConfig(gain=0., max_torque=0., max_power=0., engage_torque_nm=0., gate_min_crank_rad_s=0.),
        bearing_c_nms_rad=0., freehub_c_nms_rad=0., chain_c_ns_m=0.)
    model, data, python, native = pair(tmp_path, **config)
    assert_tree(native.drive_components({}, .001, 0.), python.compute_components(model, data, .001, speed_mps=0.))


def test_python_free_typed_numeric_validators(tmp_path):
    import json
    import os
    import subprocess
    from pathlib import Path
    from native_loader import selected_build
    source = tmp_path/'typed.cpp'
    executable = tmp_path/'typed'
    source.write_text('''#include "config_validation.hpp"
#include <limits>
template<class Config> void invalid(const Config& config) {
  try { validate(config); } catch (const std::invalid_argument&) { return; }
  throw std::runtime_error("typed config accepted invalid domain");
}
int main() {
  invalid(nativecfg::AirSpringSpec{}); invalid(nativecfg::AirSpringConfig{});
  invalid(nativecfg::DamperCore{}); invalid(nativecfg::Charger3Config{});
  invalid(nativecfg::SuperDeluxeConfig{}); invalid(nativecfg::CoilConfig{});
  invalid(nativecfg::EndStopConfig{.stiffness_n_m=-1., .damping_n_s_m=0.});
  invalid(nativecfg::BrakeConfig{}); invalid(nativecfg::CruiseConfig{});
  invalid(nativecfg::RiderPathConfig{.stiffness_n_m=-1.});
  invalid(nativecfg::ResistanceConfig{}); invalid(nativecfg::TireMaterial{});
  invalid(nativecfg::TireParams{}); invalid(nativecfg::SurfaceSpec{}); invalid(nativecfg::TireConfig{});
  invalid(drivetrain::GearingConfig{}); invalid(drivetrain::PedalingConfig{});
  invalid(drivetrain::ShiftingConfig{}); invalid(drivetrain::MotorProfile{});
  invalid(drivetrain::AssistConfig{}); invalid(drivetrain::DrivePolicyConfig{}); invalid(drivetrain::DriveConfig{});
  invalid(drivetrain::BatteryConfig{.energy_j=-1.});
  nativecfg::validate(nativecfg::EndStopConfig{});
  nativecfg::validate(nativecfg::BrakeConfig{.torque_ceiling_nm=0., .taper_radps=1.});
  drivetrain::validate(drivetrain::BatteryConfig{});
  return 0;
}
''')
    includes = Path(__file__).resolve().parents[2]/'native/src'
    context = json.loads((selected_build(os.environ)/'native_check_context.json').read_text())
    result = subprocess.run([context['compiler']['path'], '-std=c++23', '-pedantic-errors',
        '-isysroot', context['sdk'], '-I', str(includes), str(source), '-o', str(executable)],
        capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)
