import numpy as np
import pytest

from bike_sim.sim.ride.period_buffer import PeriodBuffer, RawStep, evaluate_attachments
from bike_sim.sim.ride.attachment_wrench import recover_wrench
from bike_sim.physics.attachment_budget import attachment_violations


def _raw(i, jac, qfrc):
    return RawStep(i, i*.0005, (i+1)*.0005, np.zeros(4), np.zeros(4), {},
                   {'foot_front': (jac, qfrc)}, np.zeros(0), np.zeros(4), {})


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
        if attachment_violations(sample['foot_front']) and first is None:
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
            {'effort_base':{}})
        raws.append(raw)
        c.effort_diagnostics = {}
        expected.append(solved_effort(c, SimpleNamespace(actuator_force=raw.actuator_force,
            qfrc_passive=raw.qfrc_passive),(raw.qpos,raw.qvel),.0005))
    report = evaluate_period(runtime, raws)
    assert report.efforts == tuple(expected)
    assert [e['rider_positive_power_w'] for e in report.efforts] == [0., 2., 12.]
    assert [e['rider_effort_budget_exceeded'] for e in report.efforts] == [False, False, True]
    assert report.violations_by_step == ((), (), ('rider_strength.joint',))
    assert report.first_failure == (.0015, ('rider_strength.joint',))


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
    for _ in range(2003):
        step_reference(runtime, control=command)
        sample = runtime.sample
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
