from dataclasses import replace
from math import pi

import mujoco
import numpy as np
import pytest

from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.physics.chain import DrivetrainSpecs
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.physical_config import PedalingConfig, PhysicalDriveConfig, ShiftingConfig
from bike_sim.physics.resolution import load_physics_config, resolve_physics_config
from bike_sim.physics.shifting import CadenceShifter
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.drivetrain_forces import DrivetrainForceApplier
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset


def shifter(rear_teeth=51, **options):
    return CadenceShifter(DrivetrainSpecs(rear_teeth=rear_teeth),
                          ShiftingConfig(enabled=True, **options))


def test_fast_profile_starts_stationary_with_pedaling_and_auto_shifting_enabled():
    config = load_physics_config('examples/research/viewer_physics_fast.toml')
    assert config.initial_speed_mps == 0.
    assert config.drive_mode == 'articulated_effort'
    assert config.drive.human_torque_nm > 0.
    assert config.drive.shifting.enabled
    assert config.drive.gearing.rear_teeth == max(config.drive.shifting.cassette)


@pytest.mark.parametrize('cadence, rear_teeth, expected, direction', [
    (95., 51, 45, 'up'), (50., 24, 28, 'down'),
])
def test_auto_shift_selects_one_neighbor(cadence, rear_teeth, expected, direction):
    policy = shifter(rear_teeth)
    assert policy.update(cadence, cadence, .01)
    assert policy.rear_teeth == expected
    assert policy.gear_ratio == pytest.approx(34. / expected)
    assert policy.direction == direction
    assert policy.from_teeth == rear_teeth
    assert policy.shift_count == 1


def test_shift_cooldown_and_torque_cut_have_independent_durations():
    policy = shifter()
    assert policy.update(100., 100., .01)
    assert policy.torque_factor == .3
    assert not policy.update(100., 100., .21)
    assert policy.torque_factor == 1.
    assert not policy.update(100., 100., .18)
    assert policy.update(100., 100., .02)
    assert policy.rear_teeth == 39
    policy.reset()
    assert policy.rear_teeth == 51 and policy.shift_count == 0
    assert policy.torque_factor == 1.


@pytest.mark.parametrize('options', [
    {'pedaling': False}, {'braking': True}, {'rear_in_contact': False},
])
def test_no_shift_when_rider_is_not_driving_or_wheel_is_airborne(options):
    policy = shifter()
    assert not policy.update(120., 120., .01, **options)
    assert policy.rear_teeth == 51


def test_cassette_limits_cadence_band_and_backward_rotation_do_not_shift():
    assert not shifter().update(0., 0., .01)
    assert not shifter(10).update(120., 120., .01)
    assert not shifter(24).update(80., 80., .01)
    assert not shifter(24).update(-50., -50., .01)


def test_overrunning_wheel_can_upshift_with_stationary_pedals():
    policy = shifter()
    assert policy.update(0., 120., .01)
    assert policy.rear_teeth == 45


def test_power_stroke_spike_cannot_upshift_while_the_wheel_grinds():
    # A hard stroke spins the crank fast while the wheel-required cadence stays
    # low; the filtered rate must not trigger a climbing upshift, and a shift
    # that would land required cadence below the band is refused anyway.
    policy = shifter()
    for _ in range(200):
        assert not policy.update(120., 60., .01)
    assert policy.rear_teeth == 51


def test_shift_decision_uses_filtered_cadence_not_one_stroke():
    policy = shifter()
    for _ in range(50):
        policy.update(75., 75., .01)
    assert not policy.update(120., 75., .01)
    shifted = any(policy.update(120., 75., .01) for _ in range(150))
    assert shifted and policy.direction == 'up' and policy.rear_teeth == 45


def test_rollback_hill_hold_latches_and_releases_on_stop():
    from bike_sim.sim.ride.physical_runtime import PhysicalRuntime
    rt = object.__new__(PhysicalRuntime)
    rt.cfg = SimulationPhysicsConfig('physical', drive_mode='crank_effort',
        drive=PhysicalDriveConfig(pedaling=PedalingConfig(
            rollback_engage_mps=.2, rollback_release_mps=.05)))
    rt._rollback_hold = False
    assert rt._rollback_brake_demand(-.1) == 0.
    assert rt._rollback_brake_demand(-.3) == pytest.approx(1.)
    assert rt._rollback_brake_demand(-.1) == pytest.approx(1.)
    assert rt._rollback_brake_demand(-.01) == 0.


def test_shifting_is_opt_in_and_config_is_immutable_and_validated():
    policy = CadenceShifter(DrivetrainSpecs(), ShiftingConfig())
    assert not policy.update(120., 120., .01)
    config = resolve_physics_config({'drive': {'transmission_model': 'ideal_mid_drive',
        'shifting': {'enabled': True, 'cassette': [51, 24, 10]}}})
    assert config.drive.shifting.cassette == (10, 24, 51)
    with pytest.raises(ValueError):
        PhysicalDriveConfig(shifting=ShiftingConfig(enabled=True))
    with pytest.raises(ValueError):
        replace(config.drive, gearing=DrivetrainSpecs(rear_teeth=30))


@pytest.mark.parametrize('options', [
    {'enabled': 1}, {'cassette': []}, {'cassette': [24, 24]}, {'cassette': [10, 20.5]},
    {'target_cadence_min_rpm': 85.}, {'shift_cooldown_s': -.1},
    {'shift_cut_duration_s': .5}, {'torque_factor': 1.1}, {'cadence_smoothing_tau_s': -1.},
])
def test_invalid_shift_config_is_rejected(options):
    with pytest.raises(ValueError):
        ShiftingConfig(**options)


@pytest.fixture
def drive_rig():
    config = PhysicalDriveConfig(transmission_model='ideal_mid_drive', human_torque_nm=20.,
        gearing=DrivetrainSpecs(rear_teeth=51), shifting=ShiftingConfig(enabled=True),
        pedaling=PedalingConfig(enabled=True))
    physics = SimulationPhysicsConfig('physical', drive_mode='crank_effort', drive=config)
    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode='ride', physics_config=physics))
    data = mujoco.MjData(model)
    drive = DrivetrainForceApplier(model, config, 'crank_effort')
    drive.reset(model, data)
    return model, data, drive


def set_cadence(data, drive, rpm):
    rate = rpm * 2. * pi / 60.
    data.qvel[drive.joints['crank_spin'][1]] = rate
    data.qvel[drive.joints['rear_wheel_spin'][1]] = rate * drive.shifting.gear_ratio


def test_first_stationary_command_pedals_without_injected_motion(drive_rig):
    model, data, drive = drive_rig
    positions, velocities = data.qpos.copy(), data.qvel.copy()
    for _ in range(300):
        state = drive.prepare_pedaling(data, .00125, RideControl(), model=model)
    # At zero cadence the rider presses hard on the pedals: effort slews to
    # the configured low-cadence ceiling instead of the cruising torque.
    assert state.mode == 'pedaling' and state.effort_nm == pytest.approx(60.)
    assert drive.shifting.rear_teeth == 51
    np.testing.assert_array_equal(data.qpos, positions)
    np.testing.assert_array_equal(data.qvel, velocities)


@pytest.mark.parametrize('control, options', [
    (RideControl(human_torque_nm=0.), {}),
    (RideControl(rider_enabled=False), {}),
    (RideControl(), {'braking': True}),
    (RideControl(), {'rear_in_contact': False}),
    (RideControl(), {'active': False}),
])
def test_runtime_gear_selection_obeys_rider_brake_and_grounding_gates(drive_rig, control, options):
    model, data, drive = drive_rig
    set_cadence(data, drive, 120.)
    coefficients = model.wrap_prm.copy()
    drive.prepare_pedaling(data, .00125, control, model=model, **options)
    assert drive.shifting.rear_teeth == 51
    assert drive.shifting.shift_count == 0
    np.testing.assert_array_equal(model.wrap_prm, coefficients)


def test_coast_override_does_not_require_an_active_gear_constraint():
    config = load_physics_config('examples/research/viewer_physics_fast.toml')
    physics = replace(config, drive_mode='coast')
    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode='ride', physics_config=physics))
    data = mujoco.MjData(model)
    drive = DrivetrainForceApplier(model, config.drive, 'coast')
    drive.reset(model, data)
    state = drive.prepare_pedaling(data, .00125, RideControl(), model=model)
    assert state.mode == 'disabled'
    assert drive.shifting.shift_count == 0


def test_shift_updates_native_ratio_without_pose_velocity_or_slack_jump(drive_rig):
    model, data, drive = drive_rig
    hub = drive.ideal_hub
    data.qpos[hub.crank_qpos] = 100.
    data.qpos[hub.wheel_qpos] = 70.
    hub.reset(model, data)
    data.qpos[hub.wheel_qpos] += .25
    gap = hub.boundary - hub._relative_angle(data)
    positions, velocities = data.qpos.copy(), data.qvel.copy()
    hub.set_ratio(model, data, 34. / 45.)
    assert hub.boundary - hub._relative_angle(data) == pytest.approx(gap)
    np.testing.assert_array_equal(data.qpos, positions)
    np.testing.assert_array_equal(data.qvel, velocities)
    mujoco.mj_forward(model, data)
    assert data.ten_length[hub.tendon_id] == pytest.approx(hub._relative_angle(data))
    assert model.wrap_prm[hub.crank_coefficient] == pytest.approx(34. / 45.)
    weight = float(model.tendon_invweight0[hub.tendon_id])
    reference = mujoco.MjData(model)
    mujoco.mj_setConst(model, reference)
    assert weight == pytest.approx(model.tendon_invweight0[hub.tendon_id])


def test_shift_cut_reduces_human_and_motor_effort_and_respects_safety_limit(drive_rig):
    model, data, drive = drive_rig
    set_cadence(data, drive, 95.)
    drive.assist.torque = 40.
    drive.compute_components(model, data, .00125, speed_mps=2.,
        control=RideControl(motor_torque_nm=80., motor_limit_nm=5.))
    assert drive.last['gear_rear_teeth'] == 45
    assert drive.last['human_command_nm'] == 6.
    assert drive.last['shift_active']
    assert drive.last['shift_motor_limit_nm'] == 12.
    assert drive.last['motor_torque_nm'] <= 5.


def test_torque_relief_expires_without_retaining_the_motor_shift_cap(drive_rig):
    model, data, drive = drive_rig
    set_cadence(data, drive, 95.)
    drive.assist.torque = 40.
    control = RideControl(motor_torque_nm=80.)
    drive.compute_components(model, data, .00125, speed_mps=2., control=control)
    assert drive.last['motor_torque_nm'] == pytest.approx(12.)
    mujoco.mj_forward(model, data)
    drive.settle_actuation(model, data)
    data.time = .21
    drive.compute_components(model, data, .21, speed_mps=2., control=control)
    assert drive.last['gear_rear_teeth'] == 45
    assert not drive.last['shift_active']
    assert drive.last['shift_motor_limit_nm'] is None
    assert drive.last['human_command_nm'] == 20.
    assert drive.last['motor_torque_nm'] > 12.


def test_nonadvancing_probe_does_not_shift_or_change_native_model(drive_rig):
    model, data, drive = drive_rig
    set_cadence(data, drive, 120.)
    state = vars(drive.shifting).copy()
    coefficients, limits = model.wrap_prm.copy(), model.tendon_range.copy()
    drive.compute_components(model, data, .00125, speed_mps=2., advance=False)
    assert vars(drive.shifting) == state
    assert drive.ideal_hub.ratio == pytest.approx(34. / 51.)
    np.testing.assert_array_equal(model.wrap_prm, coefficients)
    np.testing.assert_array_equal(model.tendon_range, limits)


def test_reset_restores_initial_gear_and_native_transmission(drive_rig):
    model, data, drive = drive_rig
    set_cadence(data, drive, 95.)
    drive.prepare_pedaling(data, .00125, RideControl(), model=model)
    assert drive.shifting.rear_teeth == 45
    drive.reset(model, data)
    assert drive.shifting.rear_teeth == 51
    assert drive.ideal_hub.ratio == pytest.approx(34. / 51.)
    assert drive.shift_motor_limit_nm is None


def test_research_sample_reports_current_gear_ratio_after_shifting():
    config = replace(load_physics_config('examples/research/viewer_physics_fast.toml'),
                     drive_mode='crank_effort')
    sim = RideSimulation(track=get_preset('flat'), rider='none', physics_config=config)
    set_cadence(sim.data, sim.physical.drive, 95.)
    sim.step()
    sample = sim.physical.sample.channels['drive']
    assert sample['gear_front_teeth'] == 34
    assert sample['gear_rear_teeth'] == 45
    assert sample['gear_ratio'] == pytest.approx(34. / 45.)
    assert sample['gear_ratio'] == pytest.approx(sim.physical.drive.ideal_hub.ratio)


def test_cadence_coasting_can_resume_after_upshifting(drive_rig):
    model, data, drive = drive_rig
    wheel = drive.joints['rear_wheel_spin'][1]
    data.qvel[wheel] = 130. * 2. * pi / 60. * drive.shifting.gear_ratio
    state = drive.prepare_pedaling(data, .00125, RideControl(), model=model)
    assert state.mode == 'coasting'
    data.time = .41
    state = drive.prepare_pedaling(data, .41, RideControl(), model=model)
    assert drive.shifting.rear_teeth == 39
    state = drive.prepare_pedaling(data, .41, RideControl(), advance=False)
    assert state.required_cadence_rpm == pytest.approx(130. * 39. / 51.)
    assert state.mode == 'coasting'
    data.time = .82
    state = drive.prepare_pedaling(data, .41, RideControl(), model=model)
    assert drive.shifting.rear_teeth == 33
    assert state.mode == 'pedaling'
