"""Run-mode checks using neutral raw intervals, independently of plant validity."""
from types import SimpleNamespace
import inspect

import numpy as np
import pytest

from bike_sim.sim.ride.control_clock import ControlClock
from bike_sim.sim.ride.model_status import ModelStatus
from bike_sim.sim.ride.period_buffer import PeriodBuffer, RawStep
from bike_sim.sim.ride.physical_runtime import PhysicalRuntime
from bike_sim.sim.ride.physical_samples import WorkHistory
from bike_sim.sim.ride.reference_monitor import InvalidReferenceRun


def runtime_for_raws(*, strict=True, decimation=1):
    runtime = PhysicalRuntime.__new__(PhysicalRuntime)
    runtime.sim = SimpleNamespace(steps=0)
    runtime.cfg = SimpleNamespace(drive=SimpleNamespace(battery=SimpleNamespace(enabled=False)))
    runtime.control_clock = ControlClock(.0005, .005)
    runtime._buffer = PeriodBuffer(10)
    runtime.rider_control = runtime.rider_contacts = None
    runtime.model_status = ModelStatus()
    runtime.history = WorkHistory()
    runtime.initial_energy_j = runtime.initial_battery_j = 0.
    runtime.energy_scale_j = 1.
    for name in ('loss_j', 'muscle_signed_j', 'muscle_positive_j', 'motor_signed_j',
                 'motor_positive_j', 'constraint_absolute_j', 'solver_work_j',
                 'active_work_j', 'external_work_j', 'electrical_work_j'):
        setattr(runtime, name, 0.)
    runtime.set_record_decimation(decimation)
    runtime.set_strict(strict)
    return runtime


def neutral_raw(i, *, violation=(), components=None, qvel=None):
    return RawStep(i, i*.0005, (i+1)*.0005, np.zeros(1),
        np.zeros(1) if qvel is None else np.asarray(qvel), components or {}, {},
        np.zeros(0), np.zeros(1), {}, dict(
            loss_step_j=0., electrical_power_w=0., mechanical_energy_j=0.,
            elastic_energy_j={}, battery_energy_j=0., attachment_errors=violation,
            rider_control_terms={}, full=False, constraint_snapshot=None,
            numerical_constraint_power_w={name:float(force@np.asarray(qvel))
                for name,force in (components or {}).items() if name in
                ('joint_limits','shock_solver_limit','closure','ideal_transmission')},
            channels={'tires': {s: {'unloaded_radius_m':.35, 'patches':()}
                                for s in ('front','rear')}},
            diagnostics=dict(rider={}, rider_welds={}, endpoint_mass={}, rider_intent=None,
                rider_allocation={}, rider_ik_saturation={}, rider_support_targets={})))


def test_runtime_defaults_to_strict_and_validates_mode():
    assert inspect.signature(PhysicalRuntime).parameters['strict'].default is True
    for bad in (0, 1, None, 'strict'):
        with pytest.raises(ValueError, match='bool'):
            PhysicalRuntime(None, strict=bad)


@pytest.mark.parametrize('decimation', [1, 400])
@pytest.mark.parametrize('strict', [True, False])
def test_raw_failure_time_and_sticky_status_ignore_decimation(strict, decimation):
    runtime = runtime_for_raws(strict=strict, decimation=decimation)
    for i in range(10):
        runtime._buffer.push(neutral_raw(i, violation=('foot_front.normal',) if i == 3 else ()))
    context = (pytest.raises(InvalidReferenceRun, match='foot_front.normal') if strict else
               pytest.warns(RuntimeWarning, match='foot_front.normal'))
    with context:
        runtime.flush()
    assert runtime.reference_monitor.first_failure == (.002, ('foot_front.normal',))
    assert len(runtime.completed_samples) == 10
    assert runtime.history.duration_s == pytest.approx(.005)
    assert runtime.completed_samples[2].channels['model_status']['model_valid']
    assert not runtime.completed_samples[-1].channels['model_status']['model_valid']
    assert runtime.model_status.first['interval_id'] == 3
    assert not runtime._buffer._raws


def test_monitor_mode_cannot_change_after_a_step():
    runtime = runtime_for_raws()
    for bad in (0, None, 'false'):
        with pytest.raises(ValueError, match='bool'):
            runtime.set_strict(bad)
    runtime.sim.steps = 1
    with pytest.raises(RuntimeError, match='first step'):
        runtime.set_strict(False)


def test_reset_enters_new_episode_before_preparing_an_explicit_initial_state():
    from bike_sim.sim.ride.initial_state import _guard
    runtime=runtime_for_raws()
    class Prepared(Exception):pass
    class Seed:
        def prepare(self,owner):
            _guard(owner)
            raise Prepared
    runtime.sim=SimpleNamespace(steps=3,data=SimpleNamespace(time=.0015),physical_initial_state=Seed())
    runtime.rider_intent=SimpleNamespace(reset=lambda:None)
    with pytest.raises(Prepared):
        runtime.reset()
    assert runtime.sim.steps==0 and runtime.sim.data.time==0.


@pytest.mark.parametrize('tail_error', [ValueError, RuntimeError, ArithmeticError])
def test_headless_tail_failure_preserves_primary_error_and_serializes(tmp_path, monkeypatch, tail_error):
    import json
    from bike_sim.sim.ride import physical_session
    from bike_sim.sim.ride.reference_monitor import ReferenceMonitor
    runtime = SimpleNamespace(set_strict=lambda value: None,
        reference_monitor=ReferenceMonitor(strict=True), vertices=np.zeros((2, 2)),
        model_status=SimpleNamespace(as_dict=lambda: {'model_valid':False}))
    def step():
        raise ValueError('primary integration failure')
    def flush():
        raise tail_error('tail publication failure')
    runtime.flush = flush
    sim = SimpleNamespace(physical=runtime, position_m=0., time_s=.001, steps=2,
        crash=None, model=SimpleNamespace(opt=SimpleNamespace(timestep=.0005)), step=step,
        default_limits=lambda:SimpleNamespace(max_steps=10,max_wall_clock_s=10.))
    monkeypatch.setattr(physical_session, 'build_physical_simulation', lambda *args:sim)
    monkeypatch.setattr(physical_session, 'configuration_metadata', lambda *args:{})
    monkeypatch.setattr(physical_session, 'physical_run_dir_name', lambda *args:'failed')
    monkeypatch.setattr(physical_session, 'physical_summary',
                        lambda sim,meta,reason:dict(meta, reason=reason))
    recorder = SimpleNamespace(record=lambda sim:None, write_csv=lambda path:None,
                               write_jsonl=lambda path:None)
    monkeypatch.setattr(physical_session, 'PhysicalRecorder', lambda *args:recorder)
    args=SimpleNamespace(out=tmp_path,decimate=1,duration=.01,no_plots=True)
    assert physical_session.run_physical_headless(
        SimpleNamespace(name='probe',length_m=100.),args,0,None) == 1
    summary=json.loads((tmp_path/'failed'/'summary.json').read_text())
    assert summary['reason'] == 'simulation_error'
    assert summary['failure'] == 'primary integration failure'
    assert 'tail publication failure' in summary['tail_failure']


@pytest.mark.parametrize('primary_type',[RuntimeError,InvalidReferenceRun])
def test_research_cleanup_keeps_primary_exception_and_terminal_metadata(primary_type):
    from collections import deque
    from bike_sim.sim.research.environment import ResearchEnvironment
    from bike_sim.sim.ride.control import RideControl
    env=ResearchEnvironment.__new__(ResearchEnvironment)
    env.terminated=env.truncated=False;env.reason=env.error=None
    env.demand_nm=None;env.rider_program=env.rider_behavior=None
    env._queue=deque();env.delay_steps=0;env.control_steps=env.max_steps=1
    env._motor_applied=env._applied=RideControl()
    env.commands_requested=[];env.commands_applied=[]
    cfg=SimpleNamespace(physics_mode='physical',drive_mode='articulated_effort')
    def fail(*args,**kwargs):raise primary_type('primary failure')
    env.sim=SimpleNamespace(steps=0,time_s=0.,physics_config=cfg,
        rider=SimpleNamespace(variant='articulated_planar'),step=fail,
        physical=SimpleNamespace(_buffer=SimpleNamespace(_raws=[])))
    env._consume_completed_samples=lambda **kwargs:(_ for _ in ()).throw(ValueError('cleanup failure'))
    with pytest.raises(primary_type,match='primary failure') as captured:
        env.step(RideControl())
    assert env.terminated
    assert env.reason == ('invalid_controller' if primary_type is InvalidReferenceRun else 'simulation_error')
    assert env.error == f'{primary_type.__name__}: primary failure'
    assert any('cleanup failure' in note for note in captured.value.__notes__)


def test_failed_sample_consumption_is_not_retried_or_counted_twice(monkeypatch):
    from bike_sim.sim.research import environment
    env=environment.ResearchEnvironment.__new__(environment.ResearchEnvironment)
    env._sample_cursor=-1;env.dt_s=.0005;env._next_sensor_step=100
    sample=SimpleNamespace(interval_id=0,time_s=0.,dt_s=.0005,channels={
        'suspension':{'shock_stroke_m':0.,'fork_travel_m':0.},
        'drive':{'motor_torque_nm':10.},'control':{'motor_torque_nm':10.}})
    env.sim=SimpleNamespace(physical=SimpleNamespace(completed_samples=(sample,)))
    env.max_shock_stroke_m=env.max_fork_travel_m=0.
    env.torque_requested_nms=env.torque_delivered_nms=0.;env.demand=None
    truth=SimpleNamespace(road_pitch_rad=0.,pitch_rate_up_rad_s=0.,speed_mps=0.)
    monkeypatch.setattr(environment,'truth_from_sample',lambda *args:truth)
    events=[]
    env.tracker=SimpleNamespace(update=lambda *args,**kwargs:events.append('tracker'))
    def record(*args,**kwargs):
        events.append('record')
        raise RuntimeError('recorder failed after starting its write')
    env.recorder=SimpleNamespace(record=record)
    with pytest.raises(RuntimeError,match='recorder failed'):
        env._consume_completed_samples()
    env._consume_completed_samples(stop_at_outcome=False)
    assert events == ['tracker','record']
    assert env.torque_delivered_nms == env.torque_requested_nms == .005


@pytest.mark.slow
def test_research_rejection_keeps_single_owner_reset_mode_and_saved_failure(tmp_path):
    import json
    from dataclasses import replace
    from test_seated_pedaling_cycle import _environment
    from bike_sim.sim.ride.control import RideControl
    env = _environment(tmp_path, duration_s=.02)
    env.config = replace(env.config, stop_on_model_violation=True, record_decimation=1)
    env.reset()
    runtime = env.sim.physical
    assert env.reference_monitor is runtime.reference_monitor
    assert runtime.reference_monitor.strict
    # Isolate the monitor from known, separately tested mechanical failures.
    # Real publication/accounting still processes every physical interval.
    original = runtime._advance_physics
    def inject(*args, **kwargs):
        raw = original(*args, **kwargs)
        metadata = dict(raw.metadata, attachment_errors=(
            ('foot_front.normal',) if raw.interval_id == 2 else ()), invalid_controller=False)
        return replace(raw, attachment_raw={}, metadata=metadata)
    runtime._advance_physics = inject
    with pytest.raises(InvalidReferenceRun, match='foot_front.normal'):
        env.step(RideControl(motor_torque_nm=0., human_torque_nm=0.))
    assert env.reason == 'invalid_controller'
    assert env.reference_monitor.first_failure[0] == pytest.approx(3*env.dt_s)
    env.save(tmp_path / 'saved')
    summary = json.loads((tmp_path / 'saved' / 'summary.json').read_text())
    assert summary['first_failure'][0] == pytest.approx(3*env.dt_s)
    assert not summary['model_status']['model_valid']
    assert env.recorder.rows == env.sim.steps
    env.reset()
    assert env.reference_monitor is runtime.reference_monitor
    assert env.reference_monitor.strict and env.reference_monitor.first_failure is None
