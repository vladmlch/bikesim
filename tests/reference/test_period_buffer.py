from bike_sim.physics.physical_config import ArticulatedConfig
BUDGET = ArticulatedConfig().attachment_budget()
import numpy as np
import pytest

from bike_sim.sim.ride.period_buffer import PeriodBuffer, RawStep, evaluate_attachments
from bike_sim.sim.ride.attachment_wrench import recover_wrench
from bike_sim.physics.attachment_budget import attachment_violations


def _raw(i, jac, qfrc):
    return RawStep(i, i*.0005, (i+1)*.0005, np.zeros(4), np.zeros(4), {},
                   {'foot_front': (jac, qfrc)}, np.zeros(0), np.zeros(4), {})


def _current_budget_contract_on_scalar_sample(sample, *, closes_period):
    """Apply G's new gates to the preserved P scalar force/work oracle.

    Physical integration and work values come from the independent old scalar
    step. The work gate runs only at a closing interval, including a tail.
    """
    from dataclasses import replace
    from bike_sim.physics.energy_ledger import constraint_work_ok
    violations=list(sample.channels['attachment_violations'])
    if sample.channels.get('rider_effort_budget_exceeded'):
        position=(violations.index('rider_controller.infeasible')
                  if 'rider_controller.infeasible' in violations else len(violations))
        violations.insert(position,'rider_power.positive')
    energy=sample.channels['energy']
    if closes_period and not constraint_work_ok(energy['constraint_absolute_j'],
            energy['muscle_positive_j']+energy['motor_positive_j']):
        violations.append('energy.constraint_work')
    return replace(sample,channels=dict(sample.channels,attachment_violations=tuple(violations)))


def test_batched_wrench_matches_scalar_and_first_violation():
    jac = np.array([[1., 0., 0., -1.], [0., 1., 0., 0.], [0., 0., 1., 0.]])
    buffer = PeriodBuffer(10)
    for i in range(10):
        buffer.push(_raw(i, jac, jac.T @ np.array([5., 100.-12.*i, 0.])))
    assert buffer.full
    raws = buffer.drain()
    samples = evaluate_attachments(raws, kinds={'foot_front': 'foot'},
        half_patch_m={'foot_front': .025}, gap_m={'foot_front': 0.})
    first = None
    for i, (raw, sample) in enumerate(zip(raws, samples)):
        wrench = recover_wrench(jac, raw.attachment_raw['foot_front'][1])
        assert sample['foot_front'].normal_n == pytest.approx(wrench[1], abs=1e-9)
        if attachment_violations(sample['foot_front'], BUDGET) and first is None:
            first = i
    assert first == 7
    assert not buffer.full and buffer.drain() == []


def test_missing_wrench_is_not_replaced_by_zero():
    jac = np.zeros((3, 4))
    result = evaluate_attachments([_raw(0, jac, np.ones(4))],
        kinds={'foot_front': 'foot'}, half_patch_m={}, gap_m={'foot_front': 0.})
    assert result == [{}]


def test_buffer_overflow_and_discontinuous_interval_are_rejected():
    buffer = PeriodBuffer(1)
    raw = _raw(0, np.eye(3, 4), np.zeros(4))
    buffer.push(raw)
    with pytest.raises(RuntimeError, match='overflow'):
        buffer.push(raw)
    buffer = PeriodBuffer(3)
    buffer.push(raw)
    with pytest.raises(ValueError, match='consecutive'):
        buffer.push(_raw(2, np.eye(3, 4), np.zeros(4)))


def test_recording_a_published_period_twice_is_idempotent():
    from types import SimpleNamespace
    from bike_sim.sim.ride.physical_recorder import PhysicalRecorder
    recorder = PhysicalRecorder.__new__(PhysicalRecorder)
    recorder._last_id = None
    sim = SimpleNamespace(physical=SimpleNamespace(completed_samples=(
        SimpleNamespace(interval_id=0), SimpleNamespace(interval_id=1))))
    recorded = []
    def record_sample(sim, sample):
        assert recorder._last_id is None or sample.interval_id > recorder._last_id
        recorded.append(sample.interval_id)
        recorder._last_id = sample.interval_id
    recorder._record_sample = record_sample
    recorder.record(sim)
    recorder.record(sim)
    assert recorded == [0, 1]


def test_actual_spatial_body_inputs_match_scalar_attachment_measurement():
    import mujoco
    from dataclasses import replace
    from test_attachment_wrench import _pair_model
    from bike_sim.sim.ride.attachment_wrench import attachment_raw, attachment_sample
    model = _pair_model('<weld name="w" body1="rider" body2="bike"/>')
    data = mujoco.MjData(model)
    rider, bike = model.body('rider').id, model.body('bike').id
    raws, expected = [], []
    for i in range(10):
        mujoco.mj_forward(model, data)
        data.xfrc_applied[rider] = [5.+i, 0., -100.+12.*i, 0., 0., 0.]
        mujoco.mj_forward(model, data)
        args = (model, data, 0, rider, bike, np.array([0., 0., 1.]),
                np.array([0., 0., 1.]), 'foot')
        expected.append(attachment_sample(*args, rotational=True, half_patch_m=.025))
        item = attachment_raw(*args, rotational=True, half_patch_m=.025)
        raws.append(replace(_raw(i, np.eye(3, 4), np.zeros(4)), attachment_raw={'foot_front':item}))
    actual = evaluate_attachments(raws)
    for sample, scalar in zip(actual, expected):
        assert sample['foot_front'] == scalar
    # A spatial minimum-norm answer must still explain both bodies and cancel.
    item = raws[0].attachment_raw['foot_front']
    bad = replace(item, bike_qfrc=item.bike_qfrc+10.)
    assert evaluate_attachments([replace(raws[0], attachment_raw={'foot_front':bad})]) == [{}]
    bad = replace(item, observable=False)
    assert evaluate_attachments([replace(raws[0], attachment_raw={'foot_front':bad})]) == [{}]


def test_held_command_power_and_strength_are_checked_at_each_incoming_state():
    from types import SimpleNamespace
    from bike_sim.sim.ride.period_buffer import evaluate_period
    from bike_sim.sim.ride.rider_effort import solved_effort
    c = SimpleNamespace(joints={'joint':(0, 0, 0)}, last_terms={'joint':{}},
        config=SimpleNamespace(active_positive_power_limit_w=10.), effort_diagnostics={},
        strength_violations=lambda active, q, v: ('joint',) if q[0] >= 2. else ())
    runtime = SimpleNamespace(rider_control=c, control_clock=SimpleNamespace(timestep_s=.0005))
    raws, expected = [], []
    for i, speed in enumerate((-1., 1., 6.)):
        raw = RawStep(i,i*.0005,(i+1)*.0005,np.array([float(i)]),np.array([speed]),
            {'act_rider_joint':np.array([2.])}, {},np.array([2.]),np.array([-.5*speed]),{},
            {'effort_base':{},'numerical_constraint_power_w':{}})
        raws.append(raw)
        c.effort_diagnostics = {}
        expected.append(solved_effort(c, SimpleNamespace(actuator_force=raw.actuator_force,
            qfrc_passive=raw.qfrc_passive),(raw.qpos,raw.qvel),.0005))
    report = evaluate_period(runtime, raws, BUDGET)
    assert report.efforts == tuple(expected)
    assert [e['rider_positive_power_w'] for e in report.efforts] == [0., 2., 12.]
    assert [e['rider_effort_budget_exceeded'] for e in report.efforts] == [False, False, True]
    assert report.violations_by_step == ((), (), ('rider_strength.joint','rider_power.positive'))
    assert report.first_failure == (.0015, ('rider_strength.joint','rider_power.positive'))


@pytest.mark.slow
def test_non_aligned_flush_keeps_each_intervals_actual_held_command_terms(tmp_path):
    from pathlib import Path
    from bike_sim.cli import research as research_cli
    from bike_sim.sim.ride._step_oracle import step_reference
    from bike_sim.sim.ride.control import RideControl
    track = tmp_path / 'flat.toml'
    track.write_text('name="probe"\nlength_m=200.0\nsurface="hardpack"\n'
                     'grade_profile={knots=[[0.0,0.0],[200.0,0.0]]}\n')
    args = research_cli.parser().parse_args([
        '--physics-config', str(Path(__file__).resolve().parents[2] /
                                'examples/research/viewer_physics_welded.toml'),
        '--track-file', str(track), '--dt', '.0005', '--duration', '.03',
        '--control-period', '.0075', '--diagnostic-model-limits',
        '--out', str(tmp_path / 'out')])
    env = research_cli.make_environment(args)
    runtime = env.sim.physical
    # Put both paths through the identical settled-cache reset path; the first
    # cold equilibrium solve can leave a different allocator warm-start iterate.
    env.reset()
    initial_branch = runtime.rider_control._last_branch
    initial_solution = runtime.rider_control._last_solution.copy()
    runtime.set_record_decimation(1)
    commands = [RideControl(motor_torque_nm=0., human_torque_nm=0. if i < 20 else 5.)
                for i in range(60)]
    expected = []
    for command in commands:
        step_reference(runtime, control=command)
        expected.append(runtime.sample)
    env.reset()
    assert runtime.rider_control._last_branch == initial_branch
    np.testing.assert_array_equal(runtime.rider_control._last_solution, initial_solution)
    runtime.set_record_decimation(1)
    actual = []
    for i, command in enumerate(commands):
        runtime.step(control=command)
        actual.extend(runtime.completed_samples)
        if i+1 in (15, 37):
            runtime.flush()
            actual.extend(runtime.completed_samples)
    runtime.flush()
    actual.extend(runtime.completed_samples)
    assert len(actual) == len(expected) == 60
    for scalar, batch in zip(expected, actual):
        np.testing.assert_allclose(batch.qpos,scalar.qpos,rtol=0.,atol=1e-12)
        np.testing.assert_allclose(batch.qvel,scalar.qvel,rtol=0.,atol=1e-12)
        before, after = scalar.channels['rider_control'], batch.channels['rider_control']
        assert before.keys() == after.keys()
        for name in before:
            assert before[name].keys() == after[name].keys()
            for field, value in before[name].items():
                if isinstance(value, bool):
                    assert after[name][field] is value
                else:
                    assert after[name][field] == pytest.approx(value,rel=0.,abs=1e-9), (batch.interval_id,name,field)


@pytest.mark.slow
def test_runtime_batch_matches_preserved_scalar_step_and_flush(tmp_path):
    from pathlib import Path
    from bike_sim.cli import research as research_cli
    from bike_sim.sim.ride._step_oracle import step_reference
    from bike_sim.sim.ride.control import RideControl
    track = tmp_path / 'flat.toml'
    track.write_text('name = "probe"\nlength_m = 200.0\nsurface = "hardpack"\n'
                     'grade_profile = { knots = [[0.0, 0.0], [200.0, 0.0]] }\n')
    args = research_cli.parser().parse_args([
        '--physics-config', str(Path(__file__).resolve().parents[2] /
                                'examples/research/viewer_physics_welded.toml'),
        '--track-file', str(track), '--dt', '.0005', '--duration', '1.',
        '--diagnostic-model-limits', '--out', str(tmp_path / 'out')])
    env = research_cli.make_environment(args)
    sim, runtime = env.sim, env.sim.physical
    command = RideControl(motor_torque_nm=0., human_torque_nm=0.)
    expected = []
    for index in range(2003):
        step_reference(runtime, control=command)
        sample = _current_budget_contract_on_scalar_sample(runtime.sample,
            closes_period=(index+1)%runtime.control_clock.steps_per_period==0 or index==2002)
        expected.append(sample)
        runtime.reference_monitor.accept(sample.end_time_s, sample.channels['attachment_violations'])
    failure = runtime.reference_monitor.first_failure
    env.reset()
    actual = []
    for _ in range(2003):
        sim.step(control=command)
        actual.extend(runtime.completed_samples)
    assert len(actual) == 2000
    runtime.flush()
    actual.extend(runtime.completed_samples)
    assert len(actual) == 2003 and runtime.history.last_id == 2002
    assert runtime.reference_monitor.first_failure == failure
    for scalar, batch in zip(expected, actual):
        assert scalar.interval_id == batch.interval_id
        assert scalar.channels['attachment_violations'] == batch.channels['attachment_violations']
        np.testing.assert_allclose(batch.qpos, scalar.qpos, atol=1e-12, rtol=0.)
        np.testing.assert_allclose(batch.qvel, scalar.qvel, atol=1e-12, rtol=0.)
        for name in ('muscle_positive_j', 'constraint_absolute_j', 'residual_j'):
            assert batch.channels['energy'][name] == pytest.approx(scalar.channels['energy'][name], abs=1e-9)
        assert batch.channels['rider_positive_power_w'] == pytest.approx(
            scalar.channels['rider_positive_power_w'], abs=1e-9)
    env.reset()
    runtime.set_record_decimation(80)
    decimated = []
    for _ in range(2003):
        sim.step(control=command)
        decimated.extend(runtime.completed_samples)
    runtime.flush()
    decimated.extend(runtime.completed_samples)
    assert len(decimated) == len(expected)
    full_ids = {s.interval_id for s in decimated if 'rider_control' in s.channels}
    assert full_ids == {i for i in range(2003) if i % 80 == 0 or (i+1) % 10 == 0 or i == 2002}
    assert set(decimated[-1].channels) == set(expected[-1].channels)
    for scalar, sample in zip(expected, decimated):
        assert scalar.channels['attachment_violations'] == sample.channels['attachment_violations']
        assert sample.channels['energy']['muscle_positive_j'] == pytest.approx(
            scalar.channels['energy']['muscle_positive_j'], abs=1e-9)
