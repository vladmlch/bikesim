"""A4: independent work/contact gates and complete-period strict publication."""
import copy
from dataclasses import asdict
from types import SimpleNamespace
import warnings

import numpy as np
import pytest

from native_loader import load_native
from _native_runtime_support import make_python_ride, assert_tree_close
from bike_sim.native.runtime import create_native_ride
from bike_sim.native.setup import rider_controller_config
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.physical_samples import plain
from bike_sim.sim.ride.reference_monitor import InvalidReferenceRun
from bike_sim.sim.ride.rider_effort import solved_effort

bike_native = load_native()


def test_independent_opposing_work_is_not_cancelled():
    work = bike_native.runtime_step_work([400., -350.], 100., [20., -20.], .1)
    assert work == dict(muscle_signed_j=5., muscle_positive_j=40.,
                        motor_signed_j=10., motor_positive_j=10.,
                        constraint_signed_j=0., constraint_absolute_j=4.)
    assert not bike_native.runtime_constraint_work_ok(4., 50.)
    braking = bike_native.runtime_step_work([-40., -60.], -20., [], .5)
    assert braking['muscle_signed_j'] == -50.
    assert braking['muscle_positive_j'] == 0.
    assert braking['motor_signed_j'] == -10.
    assert braking['motor_positive_j'] == 0.


def test_zero_source_uses_a_fixed_absolute_roundoff_budget():
    assert bike_native.runtime_constraint_work_ok(1e-8, 0.)
    assert not bike_native.runtime_constraint_work_ok(np.nextafter(1e-8, np.inf), 0.)
    assert not bike_native.runtime_constraint_work_ok(800e-8, 0.)
    assert bike_native.runtime_constraint_work_ok(1., 100.)
    assert not bike_native.runtime_constraint_work_ok(1.00001, 100.)


@pytest.mark.parametrize('arguments', [
    ([np.nan], 0., [], .001), ([], np.inf, [], .001),
    ([], 0., [np.inf], .001), ([], 0., [], 0.), ([], 0., [], -.001),
])
def test_work_rejects_nonfinite_or_nonpositive_inputs(arguments):
    with pytest.raises(ValueError):
        bike_native.runtime_step_work(*arguments)


@pytest.mark.parametrize('intervals', [
    [(0, 0., .1), (0, .1, .2)],
    [(0, 0., .1), (2, .1, .2)],
    [(0, 0., .1), (1, .11, .2)],
    [(0, 0., .1), (1, .09, .2)],
    [(0, 0., .1), (1, .1, .1)],
])
@pytest.mark.parametrize('capacity', [1, 4])
def test_interval_markers_survive_period_clear(intervals, capacity):
    with pytest.raises(ValueError):
        bike_native.runtime_validate_intervals(intervals, capacity)


def test_consecutive_intervals_can_cross_multiple_periods():
    bike_native.runtime_validate_intervals(
        [(i, i*.1, (i+1)*.1) for i in range(7)], 2)


def _history_row(interval_id=0, *, front=(), rear=()):
    return dict(interval_id=interval_id, time_s=interval_id*.1,
                end_time_s=(interval_id+1)*.1, powers_w={'muscle': 2.},
                tires={'front': {'patches': list(front)},
                       'rear': {'patches': list(rear)}})


def test_airtime_uses_working_road_contact_not_catch_plane():
    road = {'normal_load_n': 2., 'source_geom': 'terrain'}
    catch = {'normal_load_n': 100., 'source_geom': 'catch_plane'}
    history = bike_native.runtime_work_history([
        _history_row(front=[road], rear=[catch]), _history_row(1)])
    assert history['last_id'] == 1
    assert history['duration_s'] == pytest.approx(.2)
    assert history['work_j'] == {'muscle': pytest.approx(.4)}
    assert history['airtime_s']['front'] == {
        '0.0': pytest.approx(.1), '1.0': pytest.approx(.1), '5.0': pytest.approx(.2)}
    assert history['airtime_s']['rear'] == {
        key: pytest.approx(.2) for key in ('0.0', '1.0', '5.0')}


@pytest.mark.parametrize('problem', ['missing_wheel', 'missing_patches', 'negative_load', 'unknown_source'])
def test_malformed_contact_evidence_is_not_airtime(problem):
    row = _history_row()
    if problem == 'missing_wheel':
        del row['tires']['rear']
    elif problem == 'missing_patches':
        del row['tires']['rear']['patches']
    else:
        row['tires']['rear']['patches'] = [dict(
            normal_load_n=-1. if problem == 'negative_load' else 1.,
            source_geom='unrecognized' if problem == 'unknown_source' else 'terrain')]
    with pytest.raises(ValueError):
        bike_native.runtime_work_history([row])


def test_history_rejects_duplicate_interval_after_valid_evidence():
    with pytest.raises(ValueError, match='repeated'):
        bike_native.runtime_work_history([_history_row(), _history_row()])


@pytest.mark.slow
def test_inter_tick_effort_uses_each_captured_velocity_and_strength(tmp_path):
    sim = make_python_ride()
    driver = create_native_ride(sim, directory=tmp_path / 'capture')
    controller = sim.physical.rider_control
    probe = bike_native.NativeTestAdapter(str(driver._bootstrap.model_path), {
        'spindle_controller': {
            'config': rider_controller_config(controller.config, controller),
            'pose': plain(asdict(controller.pose)),
            'crank_length_m': float(sim.crank_length_m)}})
    probe.spindle_restore(controller.state_dict())
    held = copy.deepcopy(probe.spindle_state())
    name = next(name for name in controller.joints if 'hip' in name)
    _, dof, actuator = controller.joints[name]
    qpos = np.array(sim.data.qpos, copy=True)
    dt = float(sim.model.opt.timestep)
    powers = []
    for torque, speed in [(25., -1.), (26., 10.), (-2., 21.), (1e6, 1.)]:
        velocity = np.zeros(sim.model.nv)
        forces = np.zeros(sim.model.nu)
        passive = np.zeros(sim.model.nv)
        velocity[dof], forces[actuator], passive[dof] = speed, torque, -.5*speed
        controller.effort_diagnostics = {}
        expected = solved_effort(controller, SimpleNamespace(
            actuator_force=forces, qfrc_passive=passive), (qpos, velocity), dt)
        actual = probe.runtime_observe_effort(qpos, velocity, forces, passive, dt)
        assert_tree_close(actual['effort'], expected)
        expected_errors = [f'rider_strength.{joint}' for joint in expected['rider_strength_violations']]
        expected_errors += [f'rider_power.{joint}' for joint in expected['rider_joint_power_violations']]
        expected_errors += [f'rider_speed.{joint}' for joint in expected['rider_joint_speed_violations']]
        if expected['rider_effort_budget_exceeded']:
            expected_errors.append('rider_power.positive')
        assert actual['violations'] == expected_errors
        powers.append(actual['effort']['rider_positive_power_w'])
        if torque == 1e6:
            assert f'rider_strength.{name}' in actual['violations']
            assert 'rider_power.positive' in actual['violations']
        # The engine endpoint and the held command did not change between
        # these captures. In particular, observing is not a control tick.
        assert_tree_close(probe.spindle_state(), held)
    assert powers == [0., 260., 0., 1e6]
    driver.close()


@pytest.mark.slow
def test_strict_rejection_publishes_the_whole_period_before_raising():
    sim = make_python_ride(strict=True, decimation=7)
    native = create_native_ride(sim, strict=True, record_decimation=7)
    expected = []
    python_error = None
    for _ in range(800):
        try:
            sim.step(0., 0., control=RideControl())
        except InvalidReferenceRun as error:
            python_error = error
        for sample in sim.physical.completed_samples:
            if sample.interval_id >= len(expected):
                expected.append(sample.as_dict())
        if python_error is not None:
            break
    assert python_error is not None, 'pinned diagnostic model must exercise a real violation'
    with pytest.raises(InvalidReferenceRun) as native_error:
        native.advance(800)
    assert str(native_error.value) == str(python_error)
    actual = native.drain_samples().as_dict_rows()
    assert len(actual) == len(expected) > 0
    for got, wanted in zip(actual, expected):
        assert_tree_close(got, wanted)
    assert native.snapshot().step == len(expected)
    assert native.accounting_state['history']['last_id'] == len(expected)-1
    assert native.accounting_state['buffered_intervals'] == 0
    assert_tree_close(native.first_failure, sim.physical.reference_monitor.first_failure)
    assert_tree_close(native.model_status, sim.physical.model_status.as_dict())
    earliest = next(row for row in expected if row['attachment_violations'])
    assert native.first_failure[0] == earliest['interval_end_s']
    assert native.first_failure[1] == tuple(earliest['attachment_violations'])
    assert len(native.drain_samples()) == 0
    native.close()


@pytest.mark.slow
def test_diagnostic_warns_once_and_keeps_failure_after_more_periods():
    sim = make_python_ride()
    native = create_native_ride(sim)
    with warnings.catch_warnings(record=True) as observed:
        warnings.simplefilter('always', RuntimeWarning)
        native.advance(41)
        native.flush()
        first = native.first_failure
        native.advance(81)
        native.flush()
    messages = [item for item in observed if 'invalid reference step' in str(item.message)]
    assert len(messages) == 1
    assert first is not None and native.first_failure == first
    assert not native.model_status['model_valid']
    assert native.drain_samples().interval_ids == tuple(range(81))
    native.close()
