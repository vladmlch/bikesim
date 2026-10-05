"""Scalar policies: unchanged Python oracles, complete snapshots, atomic replay."""
import copy
from dataclasses import asdict, replace
import gc
from pathlib import Path
import sys

import mujoco
import numpy as np
import pytest

from bike_sim.physics.battery import Battery, motor_electrical_power, limit_torque_by_energy
from bike_sim.physics.freehub import Freehub
from bike_sim.physics.motor import AssistController
from bike_sim.physics.pedaling import PedalingPolicy, human_crank_torque
from bike_sim.physics.physical_config import PhysicalDriveConfig, PedalingConfig, ShiftingConfig, AssistConfig
from bike_sim.sim.ride.drivetrain_forces import DrivetrainForceApplier
from _bits import assert_bitwise_equal

BUILD = Path(__file__).resolve().parents[2] / 'native' / 'build'
sys.path.insert(0, str(BUILD))
import bike_native
assert Path(bike_native.__file__).resolve().parent == BUILD.resolve()

# Exact snapshot keys are Python vars(), excluding immutable setup objects.
KEYS = {
    'pedaling': {'coasting', 'target_phase_rad', 'target_rate_rad_s', 'deceleration_rad_s2', '_effort', '_cadence_ema'},
    'shifting': {'rear_teeth', 'cooldown_s', 'cut_remaining_s', 'shift_count', 'direction', 'from_teeth', 'cadence_ema', 'required_ema'},
    'assist': {'torque', 'pedaling', 'last_gain'},
    'battery': {'initial_energy_j', 'energy_j', 'drawn_energy_j'},
    'hub': {'boundary', 'energy_j', 'torque_nm'},
}


def assert_tree(actual, expected):
    if isinstance(expected, dict):
        assert set(actual) == set(expected)
        for k in expected:
            assert_tree(actual[k], expected[k])
    elif isinstance(expected, (list, tuple)):
        assert len(actual) == len(expected)
        for a, b in zip(actual, expected):
            assert_tree(a, b)
    elif isinstance(expected, (str, bool, int)) or expected is None:
        assert type(actual) is type(expected)
        assert actual == expected
    else:
        assert_bitwise_equal(actual, expected)


def applier(**overrides):
    cfg = replace(PhysicalDriveConfig(transmission_model='ideal_mid_drive'), **overrides)
    names = [('frame', 'root_x'), ('crank', 'crank_spin'), ('cassette', 'cassette_spin'),
             ('rear_wheel', 'rear_wheel_spin'), ('front_wheel', 'front_wheel_spin'),
             ('pedal_front', 'pedal_front_spin'), ('pedal_rear', 'pedal_rear_spin')]
    bodies = ''.join(f'<body name="{body}"><joint name="{joint}"/><geom type="sphere" size=".1" mass="1"/></body>' for body, joint in names)
    model = mujoco.MjModel.from_xml_string(f'<mujoco><worldbody>{bodies}</worldbody><tendon><fixed name="ideal_mid_drive_freehub"><joint joint="crank_spin" coef="1.4166666666666667"/><joint joint="rear_wheel_spin" coef="-1"/></fixed></tendon><actuator><motor name="human_crank" joint="crank_spin"/><motor name="mid_drive" joint="crank_spin"/></actuator></mujoco>')
    drive = DrivetrainForceApplier(model, cfg, 'crank_effort')
    drive.hub = Freehub(cfg.freehub_k_nm_rad, cfg.freehub_c_nms_rad)
    return drive


def oracle_config(drive):
    a = drive.assist
    profile = None if a.profile is None else {
        'mode_gains': {k: list(v) if isinstance(v, tuple) else float(v) for k,v in a.profile.mode_gains.items()},
        'emtb_full_gain_at_nm': a.profile.emtb_full_gain_at_nm}
    return {'gearing': asdict(drive.config.gearing),
            'pedaling': {k: getattr(drive.pedaling.config,k) for k in ('enabled','coast_above_rpm','resume_below_rpm','stop_time_s','coast_cadence_tau_s','mash_cadence_rpm','mash_torque_nm','effort_slew_nm_s')},
            'shifting': {**asdict(drive.shifting.config), 'cassette': list(drive.shifting.config.cassette)},
            'assist': {**{k:getattr(a,k) for k in ('gain','max_torque','max_power','tau','slew','engage_torque_nm','gate_min_crank_rad_s','mode')},
                       'cutoff_mps':a.cutoff,'taper_width_mps':a.width,'profile':profile,
                       'torque_curve': None if a.torque_curve is None else a.torque_curve.tolist()},
            'battery':asdict(drive.config.battery),
            'hub_stiffness_nm_rad':drive.config.freehub_k_nm_rad,'hub_damping_nm_s':drive.config.freehub_c_nms_rad}


@pytest.fixture
def drive():
    return applier(pedaling=PedalingConfig(enabled=True, coast_cadence_tau_s=.03),
                   shifting=ShiftingConfig(enabled=True, cadence_smoothing_tau_s=0.))


@pytest.fixture
def config(drive):
    from tools.native_config import project_drive_policies
    config = project_drive_policies(drive)
    assert_tree(config, oracle_config(drive))
    return config


def snapshot(drive):
    return {name: {key: vars(getattr(drive, name))[key] for key in keys}
            for name, keys in KEYS.items()}


def check(native, drive):
    assert_tree(native.state(), snapshot(drive))


def test_native_policy_api_exists():
    # Minimal RED hits the missing API independently of projection.
    assert hasattr(bike_native, 'DrivePolicies')


def test_crank_torque():
    for mean in (-10., -0., 0., 30.):
        for phase in (-23., -np.pi, -0., 0., .71, np.pi/2, 15.):
            for ripple in (0., .35, .999):
                assert_bitwise_equal(bike_native.human_crank_torque(mean, phase, ripple), human_crank_torque(mean, phase, ripple))


def test_battery_draw_matches_python(config, drive):
    native = bike_native.DrivePolicies(config)
    drive.battery = Battery(config['battery']['energy_j'])
    for power, dt in [(5., .002), (1000., .01), (1e9, .1), (0., .01)]:
        assert_bitwise_equal(native.battery_draw(power, dt), drive.battery.draw(power, dt))
        check(native, drive)


def test_pedaling_sequences(config, drive):
    native = bike_native.DrivePolicies(config)
    cases = [(0., 0., 0., 20., .01, True, False), (2., 4., 40., 20., .04, True, False),
             (3., 15., 120., 30., .04, True, False), (4., 11., 80., 30., .01, True, False),
             (4., -8., -30., 30., .4, True, False), (5., -5., -60., 30., .01, True, True),
             (6., -5., 0., 0., .1, True, False), (6., 0., 0., 30., .1, False, False)]
    rng = np.random.default_rng(934)
    cases += [(float(rng.uniform(-40, 40)), float(rng.uniform(-15,15)), float(rng.uniform(-100,130)), float(rng.uniform(0,65)), .007, True, False) for _ in range(200)]
    for phase, rate, required, effort, dt, enabled, braking in cases:
        expected = drive.pedaling.update(phase, rate, required, effort, dt, enabled=enabled, braking=braking)
        assert_tree(native.pedaling_update(phase, rate, required, effort, dt, enabled, braking), asdict(expected))
        check(native, drive)


@pytest.mark.parametrize('mode', ['legacy_signed', 'magnitude'])
@pytest.mark.parametrize('tau', [0., .35])
def test_shifting(mode, tau):
    drive = applier(shifting=ShiftingConfig(enabled=True, upshift_slip_mode=mode, cadence_smoothing_tau_s=tau))
    from tools.native_config import project_drive_policies
    native = bike_native.DrivePolicies(project_drive_policies(drive))
    cases = [(100., 100., .5, True, False, True, 1.), (100., 100., .5, True, False, True, -1.),
             (60., 60., .01, True, False, True, None), (60., 60., .5, True, False, True, None),
             (60., 120., .5, True, False, True, None), (120., 30., .5, True, False, True, None),
             (100., 100., .5, True, False, False, None), (100., 100., .5, False, False, True, None),
             (100., 100., .5, True, True, True, None), (-10., -10., .5, True, False, True, None)]
    rng = np.random.default_rng(11)
    cases += [(float(rng.uniform(30,130)), float(rng.uniform(30,130)), .01, True, False, True, None) for _ in range(250)]
    for rpm, req, dt, pedaling, brake, ground, slip in cases:
        expected = drive.shifting.update(rpm, req, dt, pedaling=pedaling, braking=brake, rear_in_contact=ground, rear_slip_mps=slip)
        assert native.shifting_update(rpm, req, dt, pedaling, brake, ground, slip) == expected
        assert_tree(native.shifting_diagnostics(), {**snapshot(drive)['shifting'], 'gear_ratio': drive.shifting.gear_ratio, 'torque_factor': drive.shifting.torque_factor})
        check(native, drive)


@pytest.mark.parametrize('profile,mode', [(None,'custom'), ('bosch_cx_gen4','eco'), ('bosch_cx_gen4','tour'), ('bosch_cx_gen4','emtb'), ('bosch_cx_gen4','turbo')])
def test_assist(profile, mode):
    drive = applier(assist=AssistConfig(profile=profile, mode=mode, torque_curve=((0., 80.), (43., 71.), (100., 35.), (150., 0.))))
    from tools.native_config import project_drive_policies
    native = bike_native.DrivePolicies(project_drive_policies(drive))
    for shaft in (-10., 0., 10., 43., 77.7, 100., 149., 150., 170.):
        for speed in (0., 6.5, -6.7, 7.):
            assert_bitwise_equal(native.assist_ceiling(shaft, speed), drive.assist.ceiling(shaft, speed))
    rng = np.random.default_rng(75)
    cases = [(human, rpm, speed, brake, .005, cap, shaft) for human, rpm, speed, brake, cap, shaft in
             [(4.,80.,0.,False,None,None), (30.,0.,0.,False,None,None), (30.,80.,0.,True,None,None),
              (10.,80.,0.,False,0.,None), (30.,80.,6.7,False,2.,15.), (40.,80.,0.,False,None,150.),
              (100.,80.,0.,False,None,None), (-1.,80.,0.,False,None,None)]]
    cases += [(float(rng.uniform(0,100)), float(rng.uniform(-5,150)), float(rng.uniform(-7,7)), False, .003, None, float(rng.uniform(-5,150))) for _ in range(250)]
    for human,rpm,speed,brake,dt,cap,shaft in cases:
        assert_bitwise_equal(native.assist_step(human,rpm,speed,brake,dt,cap,shaft), drive.assist.step(human,rpm,speed,brake,dt,torque_request_nm=cap,shaft_rpm=shaft))
        check(native, drive)


@pytest.mark.parametrize('a', [0., .02])
@pytest.mark.parametrize('enabled', [False, True])
def test_energy_helpers(config, a, enabled):
    config['battery'].update(copper_w_per_nm2=a, speed_w_per_rad_s2=.03, idle_w=5., enabled=enabled)
    native = bike_native.DrivePolicies(config)
    for torque in (-0., 0., 5., 37.13, 80.):
        for omega in (-7.3, -0., 0., 5.2):
            assert_bitwise_equal(native.electrical_power(torque,omega,enabled), motor_electrical_power(torque,omega,a,.03,5.,enabled))
            for budget in (0., 4.99, 5., 6., 72., 500.):
                assert_bitwise_equal(native.energy_limit(torque,omega,budget), limit_torque_by_energy(torque,omega,a,.03,5.,budget))


def test_freehub(config, drive):
    native = bike_native.DrivePolicies(config)
    for values in [(0.,0.,0.,0.), (.1,0.,2.,0.), (.3,.5,0.,3.), (.4,.5,4.,0.), (8.,3.,2.,3.), (8.,10.,2.,3.)]:
        assert_bitwise_equal(native.freehub_torque(*values), drive.hub.update(*values))
        check(native, drive)


def test_replay_reset_and_ownership(config, drive):
    native = bike_native.DrivePolicies(config)
    old = native.state()
    native.assist_step(30.,80.,0.,False,.01)
    native.set_state(old)
    check(native, drive)
    config['assist']['gain'] = 999.
    config['shifting']['cassette'].clear()
    del config
    gc.collect()
    assert_bitwise_equal(native.assist_step(30.,80.,0.,False,.01), drive.assist.step(30.,80.,0.,False,.01))
    native.reset()
    drive.assist.reset()
    check(native, drive)
    old['battery']['energy_j'] = 1.
    assert native.state()['battery']['energy_j'] != 1.


@pytest.mark.parametrize('section', KEYS)
def test_invalid_restore_is_atomic(config, section):
    native = bike_native.DrivePolicies(config)
    old = native.state()
    for key in KEYS[section]:
        for value in [np.nan, np.inf, 'invalid', [], None]:
            if old[section][key] is None and value is None:
                continue
            bad = copy.deepcopy(old)
            bad[section][key] = value
            with pytest.raises(ValueError):
                native.set_state(bad)
            assert_tree(native.state(), old)
        bad = copy.deepcopy(old)
        del bad[section][key]
        with pytest.raises(ValueError):
            native.set_state(bad)
        assert_tree(native.state(), old)


@pytest.mark.parametrize('section,key,value', [('battery','energy_j',-1.), ('hub','energy_j',-1.), ('shifting','cooldown_s',-1.), ('shifting','rear_teeth',2), ('shifting','shift_count',-1), ('shifting','rear_teeth',3.5), ('pedaling','coasting',1), ('shifting','cadence_ema',10.)])
def test_invalid_restore_values(config, section, key, value):
    native = bike_native.DrivePolicies(config)
    old = native.state()
    bad = copy.deepcopy(old)
    bad[section][key] = value
    with pytest.raises(ValueError):
        native.set_state(bad)
    assert_tree(native.state(), old)


def test_config_validation(config):
    for section in ('gearing', 'pedaling', 'shifting', 'assist', 'battery'):
        for key, value in config[section].items():
            bad = copy.deepcopy(config)
            del bad[section][key]
            # profile and curve are optional.
            if key in ('profile','torque_curve'):
                continue
            with pytest.raises(ValueError):
                bike_native.DrivePolicies(bad)
            if isinstance(value, float):
                bad[section][key] = np.nan
                with pytest.raises(ValueError):
                    bike_native.DrivePolicies(bad)


def test_projection_resolves_runtime(drive):
    assert hasattr(bike_native, 'DrivePolicies')
    from tools.native_config import project_drive_policies
    drive.assist = AssistController(profile='bosch_cx_gen4', mode='emtb')
    config = project_drive_policies(drive)
    assert config['assist']['max_torque'] == 85.
    assert config['assist']['slew'] == 2125.
    assert config['assist']['profile']['mode_gains'] == {'eco':.6,'tour':1.4,'emtb':[1.4,3.4],'turbo':3.4}
    assert set(config['pedaling']) == {'enabled','coast_above_rpm','resume_below_rpm','stop_time_s','coast_cadence_tau_s','mash_cadence_rpm','mash_torque_nm','effort_slew_nm_s'}
    native = bike_native.DrivePolicies(config)
    del drive
    gc.collect()
    native.assist_step(30.,80.,0.,False,.01)


def test_projection_normalizes_numeric_leaves():
    from bike_sim.physics.physical_config import BatteryConfig
    from tools.native_config import project_drive_policies
    drive = applier(battery=BatteryConfig(energy_j=np.float64(123.), copper_w_per_nm2=np.float64(.02)),
                    shifting=ShiftingConfig(shift_cooldown_s=np.float64(.4)))
    config = project_drive_policies(drive)
    def plain(value):
        if isinstance(value, dict):
            for leaf in value.values():
                plain(leaf)
        elif isinstance(value, list):
            for leaf in value:
                plain(leaf)
        else:
            assert type(value) in (float, int, bool, str, type(None))
    plain(config)
    bike_native.DrivePolicies(config)


def test_complete_reset_and_atomic_multi_policy_restore(config, drive):
    native = bike_native.DrivePolicies(config)
    native.pedaling_update(1., 13., 120., 30., .04)
    native.shifting_update(100.,100.,.01)
    native.assist_step(30.,80.,0.,False,.01)
    native.battery_draw(80.,.01)
    native.freehub_torque(0.,0.,2.,0.)
    native.freehub_torque(.1,0.,2.,0.)
    old = native.state()
    bad = copy.deepcopy(old)
    bad['pedaling']['_effort'] = 17.
    bad['shifting']['rear_teeth'] = 33
    bad['battery']['energy_j'] = 8.
    bad['hub']['torque_nm'] = np.inf
    with pytest.raises(ValueError):
        native.set_state(bad)
    assert_tree(native.state(), old)
    native.reset()
    check(native, drive)


def test_restored_finite_snapshots_preserve_bits(config):
    native = bike_native.DrivePolicies(config)
    state = native.state()
    state['pedaling'].update(target_phase_rad=-1000., target_rate_rad_s=-0., _cadence_ema=-10.)
    state['shifting'].update(rear_teeth=31, from_teeth=37, cadence_ema=-3., required_ema=-5.)
    state['assist'].update(torque=1e8, last_gain=1e9)
    state['battery'].update(initial_energy_j=0., energy_j=1e9, drawn_energy_j=1e8)
    state['hub'].update(boundary=-1e9, torque_nm=1e9)
    native.set_state(state)
    assert_tree(native.state(), state)
    state['pedaling']['target_phase_rad'] = 0.
    assert native.state()['pedaling']['target_phase_rad'] == -1000.


def test_disabled_shifter_initial_teeth_outside_cassette():
    from bike_sim.physics.chain import DrivetrainSpecs
    from tools.native_config import project_drive_policies
    drive = applier(gearing=DrivetrainSpecs(rear_teeth=31))
    native = bike_native.DrivePolicies(project_drive_policies(drive))
    native.set_state(native.state())
    check(native, drive)


@pytest.mark.parametrize('section,key,value', [
    ('gearing','front_teeth',True), ('gearing','rear_teeth',3.1), ('gearing','chain_pitch_m',0.),
    ('pedaling','enabled',1), ('pedaling','stop_time_s',0.), ('pedaling','resume_below_rpm',120.),
    ('shifting','enabled',1), ('shifting','cassette',[10,10]), ('shifting','cassette',[True,24]),
    ('shifting','cassette',[]), ('shifting','torque_factor',1.1), ('shifting','shift_cut_duration_s',1.),
    ('shifting','upshift_slip_mode','other'), ('assist','tau',0.), ('assist','slew',0.),
    ('assist','gain',-1.), ('assist','mode',1), ('assist','torque_curve',[[0.,1.]]),
    ('assist','torque_curve',[[0.,1.],[0.,2.]]), ('assist','torque_curve',[[0.,1.],[2.,-1.]]),
    ('assist','profile',{'mode_gains':{}}), ('battery','enabled',1), ('battery','energy_j',-1.),
])
def test_invalid_config_values(config, section, key, value):
    bad = copy.deepcopy(config)
    bad[section][key] = value
    with pytest.raises(ValueError):
        bike_native.DrivePolicies(bad)


@pytest.mark.parametrize('key', ['gearing', 'pedaling', 'shifting', 'assist', 'battery',
                                'hub_stiffness_nm_rad', 'hub_damping_nm_s'])
def test_missing_config_sections(config, key):
    del config[key]
    with pytest.raises(ValueError):
        bike_native.DrivePolicies(config)


def test_random_curve_interpolation(config):
    # Replacing NumPy's fused interpolation with separate multiply/add fails here.
    config['assist'].update(max_torque=1e6, max_power=1e9,
                            torque_curve=[[0.,80.],[43.,71.],[100.,35.],[150.,0.]])
    native = bike_native.DrivePolicies(config)
    python = AssistController(max_torque=1e6,max_power=1e9,torque_curve=config['assist']['torque_curve'])
    rng = np.random.default_rng(541)
    rpms = [0., 43., 100., 150., -0., -1., 151.]
    rpms += list(rng.uniform(-10.,170.,1000))
    for rpm in rpms:
        assert_bitwise_equal(native.assist_ceiling(float(rpm),0.), python.ceiling(float(rpm),0.))


def test_battery_nonzero_idle_remainder(config):
    native = bike_native.DrivePolicies(config)
    state = native.state()
    state['battery']['energy_j'] = .001
    native.set_state(state)
    assert native.energy_limit(80., 0., .001/.01) == 0.
    assert native.state()['battery']['energy_j'] == .001


def test_config_and_python_object_deletion():
    from tools.native_config import project_drive_policies
    drive = applier(assist=AssistConfig(profile='bosch_cx_gen4',mode='emtb',torque_curve=((0.,85.),(120.,0.))))
    config = project_drive_policies(drive)
    native = bike_native.DrivePolicies(config)
    expected = drive.assist.step(27.,72.,6.7,False,.01)
    expected_state = snapshot(drive)
    del drive, config
    gc.collect()
    assert_bitwise_equal(native.assist_step(27.,72.,6.7,False,.01), expected)
    assert_tree(native.state(), expected_state)


@pytest.mark.parametrize('tau,slew,mash', [(0.,0.,0.), (0.,0.,60.), (.03,300.,0.)])
def test_pedaling_parameter_branches(tau, slew, mash):
    from tools.native_config import project_drive_policies
    drive = applier(pedaling=PedalingConfig(enabled=True,coast_cadence_tau_s=tau,
                                           effort_slew_nm_s=slew,mash_torque_nm=mash))
    native = bike_native.DrivePolicies(project_drive_policies(drive))
    for phase,rate,required,effort,dt in [(0.,0.,0.,20.,.01), (1.,12.,110.,30.,.03),
                                        (2.,10.,100.,30.,.03), (3.,8.,80.,30.,.04),
                                        (4.,-14.,-90.,30.,.04), (5.,-6.,-10.,0.,.5)]:
        assert_tree(native.pedaling_update(phase,rate,required,effort,dt),
                    asdict(drive.pedaling.update(phase,rate,required,effort,dt)))
        check(native, drive)


def test_optional_config_sections(config):
    del config['assist']['profile'], config['assist']['torque_curve']
    bike_native.DrivePolicies(config)


def test_extra_and_missing_state_sections(config):
    native = bike_native.DrivePolicies(config)
    old = native.state()
    candidates = [{**old,'extra':{}}, {k:v for k,v in old.items() if k != 'hub'}]
    bad = copy.deepcopy(old)
    bad['pedaling']['extra'] = 0.
    candidates.append(bad)
    for candidate in candidates:
        with pytest.raises(ValueError):
            native.set_state(candidate)
        assert_tree(native.state(),old)


@pytest.mark.parametrize('method,args', [
    ('pedaling_update',(np.nan,0.,0.,30.,.01)), ('pedaling_update',(0.,0.,0.,30.,0.)),
    ('shifting_update',(80.,80.,0.)), ('shifting_update',(80.,np.inf,.01)),
    ('assist_step',(30.,80.,0.,False,0.)), ('assist_step',(np.inf,80.,0.,False,.01)),
    ('battery_draw',(np.inf,.01)), ('battery_draw',(1.,0.)), ('freehub_torque',(0.,np.inf,0.,0.)),
    ('electrical_power',(np.inf,0.,True)), ('energy_limit',(-1.,0.,100.)),
])
def test_invalid_calls_preserve_state(config, method, args):
    native = bike_native.DrivePolicies(config)
    old = native.state()
    with pytest.raises(ValueError):
        getattr(native,method)(*args)
    assert_tree(native.state(),old)


@pytest.mark.parametrize('mean,phase,ripple', [(np.nan,0.,.35), (1.,np.inf,.35), (1.,0.,-1.), (1.,0.,1.)])
def test_invalid_human_torque(mean,phase,ripple):
    with pytest.raises(ValueError):
        bike_native.human_crank_torque(mean,phase,ripple)


def test_shifter_count_boundary_rejects_successful_increment_atomically(config):
    native = bike_native.DrivePolicies(config)
    state = native.state()
    state['shifting']['shift_count'] = 2**31 - 1
    native.set_state(state)
    with pytest.raises(OverflowError, match='shift_count'):
        native.shifting_update(100.,100.,.01)
    assert_tree(native.state(), state)
    # An interval without a shift remains legal at the representable boundary.
    assert not native.shifting_update(75.,75.,.01)
    assert native.state()['shifting']['shift_count'] == 2**31 - 1


def test_unrepresentable_count_restore_is_atomic(config):
    native = bike_native.DrivePolicies(config)
    old = native.state()
    bad = copy.deepcopy(old)
    bad['shifting']['shift_count'] = 2**31
    with pytest.raises(ValueError):
        native.set_state(bad)
    assert_tree(native.state(),old)


@pytest.mark.parametrize('profile,mode', [(None,'turbo'), ('bosch_cx_gen4','emtb')])
@pytest.mark.parametrize('cadence', [-1e308, 1e308])
@pytest.mark.parametrize('shaft', [None, 80.])
def test_assist_computed_crank_overflow_matches_python_state(profile, mode, cadence, shaft):
    from tools.native_config import project_drive_policies
    drive = applier(assist=AssistConfig(profile=profile,mode=mode))
    native = bike_native.DrivePolicies(project_drive_policies(drive))
    assert_bitwise_equal(native.assist_step(30.,80.,0.,False,.01),
                         drive.assist.step(30.,80.,0.,False,.01))
    drive.assist.last_gain = 123.
    native.set_state(snapshot(drive))
    previous = snapshot(drive)['assist']
    with pytest.raises(ValueError, match='crank rate'):
        drive.assist.step(11.,cadence,0.,False,.01,shaft_rpm=shaft)
    # The late permission validation changes last_gain but retains delivered state.
    assert drive.assist.last_gain != previous['last_gain']
    assert_bitwise_equal(drive.assist.torque,previous['torque'])
    assert drive.assist.pedaling == previous['pedaling']
    with pytest.raises(ValueError, match='crank rate'):
        native.assist_step(11.,cadence,0.,False,.01,shaft_rpm=shaft)
    check(native,drive)
    assert_bitwise_equal(native.assist_step(20.,80.,0.,False,.01),
                         drive.assist.step(20.,80.,0.,False,.01))
    check(native,drive)


def test_assist_braking_resets_before_computed_crank_overflow(config, drive):
    native = bike_native.DrivePolicies(config)
    assert_bitwise_equal(native.assist_step(30.,80.,0.,False,.01),
                         drive.assist.step(30.,80.,0.,False,.01))
    assert_bitwise_equal(native.assist_step(30.,1e308,0.,True,.01),
                         drive.assist.step(30.,1e308,0.,True,.01))
    check(native,drive)
