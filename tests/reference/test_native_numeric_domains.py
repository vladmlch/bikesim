"""Shared physical domains and finite derived results, with untouched goldens."""
import copy
from dataclasses import replace

import numpy as np
import pytest

from native_loader import load_native
from test_native_input_contracts import resolved, at
from bike_sim.physics.air_spring import AirSpringSpecs, ForkAirSpring
from bike_sim.physics.damper import Charger3Damper, SuperDeluxeDamper
from bike_sim.physics.coil_shock import CoilShockSpecs, CoilShock


AIR_FIELDS = ('stanchion_inner_diam_mm', 'total_travel_mm', 'pos_chamber_length_mm',
              'neg_chamber_length_mm', 'gamma', 'atm_pressure_pa')
DAMPER_POSITIVE = ('max_hsc', 'max_lsc', 'max_reb', 'v_knee_comp', 'v_knee_reb')
DAMPER_NONNEGATIVE = ('c_lsc_min', 'c_lsc_max', 'c_hsc_min', 'c_hsc_max', 'c_reb_min', 'c_reb_max')
CASES = [
    *((('suspension', 'air_spring', key), value) for key in AIR_FIELDS for value in (0., -1., float('nan'), float('inf'), True)),
    *((('suspension', 'air_spring', key), value) for key in ('token_volume_cm3', 'gauge_pressure_psi') for value in (-1., float('nan'), float('inf'), True)),
    *((('suspension', section, key), value) for section in ('fork_damper', 'shock_damper')
      for key in DAMPER_POSITIVE for value in (0, -1, float('nan'), float('inf'), True)),
    *((('suspension', section, key), value) for section in ('fork_damper', 'shock_damper')
      for key in DAMPER_NONNEGATIVE for value in (-1., float('nan'), float('inf'), True)),
    *((('suspension', 'coil', key), value) for key in ('rate_n_m', 'stroke_mm', 'bumper_length_mm', 'bumper_peak_n')
      for value in (0., -1., float('nan'), float('inf'), True)),
    *((('suspension', 'coil', 'preload_mm'), value) for value in (-1., float('nan'), float('inf'), True)),
    *((('suspension', 'end_stops', key), value) for key in ('stiffness_n_m', 'damping_n_s_m')
      for value in (-1., float('nan'), float('inf'), True)),
]


def python_suspension(config):
    air = config['air_spring']
    specs = AirSpringSpecs(**{key: value for key, value in air.items()
                             if key not in ('num_tokens', 'gauge_pressure_psi')})
    spring = ForkAirSpring(specs, air['num_tokens'], air['gauge_pressure_psi'])
    spring.compute_axial_force(61.3)
    for name, cls in [('fork_damper', Charger3Damper), ('shock_damper', SuperDeluxeDamper)]:
        damper = cls()
        for key, value in config[name].items():
            setattr(damper, key, value)
        damper.compute_damping_force(.7, 30.)
    coil = config['coil']
    CoilShock(CoilShockSpecs(**{key: value for key, value in coil.items() if key != 'legacy_behavior'}),
              legacy_behavior=coil['legacy_behavior']).compute_axial_force(30.)
    from bike_sim.physics.model_config import EndStopConfig
    stop = config['end_stops']
    EndStopConfig(reference_force_n=stop['stiffness_n_m'] if isinstance(stop['stiffness_n_m'], (bool, np.bool_)) else stop['stiffness_n_m']*.01, damping_n_s_m=stop['damping_n_s_m'])


@pytest.mark.parametrize('location,value', CASES)
def test_suspension_domain_is_shared(resolved, location, value):
    _, path, original = resolved
    candidate = copy.deepcopy(original)
    at(candidate, location[:-1])[location[-1]] = value
    for reader in (lambda: python_suspension(candidate['suspension']),
                   lambda: load_native().Stepper(path, candidate)):
        with pytest.raises(ValueError):
            reader()


@pytest.mark.parametrize('key', ['max_lsc', 'v_knee_comp', 'c_hsc_min'])
def test_damper_mutable_parameters_revalidated(key):
    damper = Charger3Damper()
    setattr(damper, key, 0 if key != 'c_hsc_min' else -1.)
    with pytest.raises(ValueError, match=key):
        damper.compute_damping_force(.7, 61.3)


def test_air_pressure_intermediate_overflow_is_shared(resolved):
    env, path, original = resolved
    candidate = copy.deepcopy(original)
    candidate['suspension']['air_spring']['gauge_pressure_psi'] = 1e308
    spring = ForkAirSpring(gauge_pressure_psi=1e308)
    native = load_native().Stepper(path, candidate)
    native.set_state(env.sim.data.qpos, env.sim.data.qvel, env.sim.data.act,
                     env.sim.data.qacc_warmstart, env.sim.data.time)
    for calculation in (lambda: spring.compute_axial_force(61.3), native.suspension_components):
        with pytest.raises(OverflowError):
            calculation()


@pytest.mark.parametrize('legacy', [False, True])
def test_activated_zero_hbo_denominator_rejected(resolved, legacy):
    env, path, original = resolved
    damper = Charger3Damper(legacy_behavior=legacy)
    damper.hbo_start_mm = damper.total_travel_mm
    with pytest.raises(ValueError):
        damper.compute_damping_force(.7, 181.)
    candidate = copy.deepcopy(original)
    fork = candidate['suspension']['fork_damper']
    fork['legacy_behavior'], fork['hbo_start_mm'] = legacy, fork['total_travel_mm']
    if not legacy:
        with pytest.raises(ValueError):
            load_native().Stepper(path, candidate)
    else:
        native = load_native().Stepper(path, candidate)
        qpos = env.sim.data.qpos.copy()
        qvel = np.zeros(env.sim.model.nv)
        qpos[env.sim.applier.fork_qposadr] = .181
        qvel[env.sim.applier.fork_dofadr] = .7
        native.set_state(qpos, qvel, env.sim.data.act, env.sim.data.qacc_warmstart, 0.)
        with pytest.raises(ValueError):
            native.suspension_components()


@pytest.mark.parametrize('section,key,value', [
    ('pedaling', 'stop_time_s', 0.), ('pedaling', 'coast_cadence_tau_s', -1.),
    ('pedaling', 'effort_slew_nm_s', -1.), ('shifting', 'shift_cooldown_s', -1.),
    ('shifting', 'torque_factor', 1.1), ('shifting', 'cadence_smoothing_tau_s', -1.),
    ('assist', 'tau', 0.), ('assist', 'slew', 0.), ('assist', 'gain', -1.),
    ('assist', 'taper_width_mps', 0.), ('battery', 'energy_j', -1.),
])
def test_mutated_policy_configs_are_revalidated(section, key, value):
    from test_native_drive_policies import applier
    from tools.native_config import project_drive_policies
    drive = applier()
    config = project_drive_policies(drive)
    config[section][key] = value
    target = getattr(drive, section)
    if section == 'battery':
        object.__setattr__(drive.config.battery, key, value)
        from bike_sim.physics.physical_config import BatteryConfig
        oracle = lambda: BatteryConfig(**config['battery'])
    elif section == 'assist':
        setattr(target, {'taper_width_mps':'width'}.get(key, key), value)
        oracle = lambda: target.ceiling(80., 1.)
    else:
        object.__setattr__(target.config, key, value)
        oracle = (lambda: target.update(0., 1., 80., 10., .001)) if section == 'pedaling' else (
            lambda: target.update(80., 80., .001))
    for reader in (oracle, lambda: load_native().DrivePolicies(config)):
        with pytest.raises(ValueError):
            reader()


@pytest.mark.parametrize('method,args', [
    ('assist_step', (1e308, 80., 0., False, .001)),
    ('battery_draw', (1e308, 10.)),
    ('electrical_power', (1e308, 1., True)),
    ('freehub_torque', (1e308, 0., 0., 0.)),
])
def test_policy_derived_overflow_is_shared(method, args):
    from test_native_drive_policies import applier
    from tools.native_config import project_drive_policies
    drive = applier()
    native = load_native().DrivePolicies(project_drive_policies(drive))
    if method == 'assist_step':
        oracle = lambda: drive.assist.step(*args)
    elif method == 'battery_draw':
        oracle = lambda: drive.battery.draw(*args)
    elif method == 'electrical_power':
        from bike_sim.physics.battery import motor_electrical_power
        cfg = drive.config.battery
        oracle = lambda: motor_electrical_power(args[0], args[1], cfg.copper_w_per_nm2,
            cfg.speed_w_per_rad_s2, cfg.idle_w, args[2])
    else:
        drive.hub.boundary = 0.
        state = native.state()
        state['hub']['boundary'] = 0.
        native.set_state(state)
        oracle = lambda: drive.hub.update(*args)
    for calculation in (oracle, lambda: getattr(native, method)(*args)):
        with pytest.raises(OverflowError):
            calculation()


@pytest.mark.parametrize('kind,axis', [('hinge','1 0 0'), ('slide','0 1 0'), ('ball','0 0 1')])
def test_cruise_longitudinal_topology_is_shared(tmp_path, kind, axis):
    import mujoco
    from bike_sim.sim.ride.cruise import CruiseController
    model = mujoco.MjModel.from_xml_string(f'<mujoco><worldbody><body><joint name="root_x" type="{kind}" axis="{axis}"/><geom size=".1" mass="1"/></body></worldbody></mujoco>')
    path = tmp_path/'wrong-root.mjb'
    mujoco.mj_saveModel(model, str(path))
    from test_native_cruise import CONFIG
    for constructor in (lambda: CruiseController(model), lambda: load_native().Stepper(str(path), {'schema':2, 'cruise':CONFIG})):
        with pytest.raises(ValueError):
            constructor()


@pytest.mark.parametrize('kind,axis,target', [('slide','0 1 0','front_wheel_spin'),
    ('ball','0 1 0','front_wheel_spin'), ('hinge','0 0 1','front_wheel_spin'),
    ('hinge','0 1 0','rear_wheel_spin')])
def test_brake_wheel_topology_and_actuator_target_are_shared(tmp_path, kind, axis, target):
    import mujoco
    from bike_sim.sim.ride.braking import BrakeController
    model = mujoco.MjModel.from_xml_string(f'''<mujoco><worldbody>
      <body><joint name="front_wheel_spin" type="{kind}" axis="{axis}"/><geom name="geom_front_contact" type="sphere" size=".3" mass="1"/></body>
      <body><joint name="rear_wheel_spin" type="hinge" axis="0 1 0"/><geom name="geom_rear_contact" type="sphere" size=".3" mass="1"/></body>
      </worldbody><actuator><motor name="front_brake" joint="{target}"/><motor name="rear_brake" joint="rear_wheel_spin"/></actuator></mujoco>''')
    path = tmp_path/'wheel.mjb'
    mujoco.mj_saveModel(model, str(path))
    cfg = dict(torque_ceiling_nm=200., taper_radps=1.)
    for constructor in (lambda: BrakeController(model), lambda: load_native().Stepper(str(path), {'schema':2, 'brake':cfg})):
        with pytest.raises(ValueError):
            constructor()


@pytest.mark.parametrize('rate', [1e308])
def test_coil_force_overflow_before_publication(resolved, rate):
    env, path, original = resolved
    candidate = copy.deepcopy(original)
    candidate['suspension']['coil']['rate_n_m'] = rate
    coil = CoilShock(CoilShockSpecs(rate_n_m=rate))
    native = load_native().Stepper(path, candidate)
    qpos = env.sim.data.qpos.copy()
    qpos[env.sim.applier.shock_qposadr] = .030
    native.set_state(qpos, env.sim.data.qvel, env.sim.data.act, env.sim.data.qacc_warmstart, 0.)
    for calculate in (lambda: coil.compute_spring_force(30.), native.suspension_components):
        with pytest.raises(OverflowError):
            calculate()


def test_rolling_and_drag_computed_overflow():
    from bike_sim.physics.external_resistance import rolling_moment, drag_force
    with pytest.raises(OverflowError):
        rolling_moment(1e308, 10., .3, 1., .2)
    with pytest.raises(OverflowError):
        drag_force([1e308, 0., 0.], 1., 1.)
