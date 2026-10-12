"""Capture-mode and final-only research lifecycle regressions (source only)."""
from dataclasses import asdict

import numpy as np
import pytest

from bike_sim.recording.json_codec import thaw
from bike_sim.recording.reader import Recording
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.research.recording_lifecycle import ResearchRun
from bike_sim.sim.research.replay import integration_state


def _noise(monkeypatch):
    import bike_sim.cli.research as cli
    from bike_sim.sim.research.sensors import SensorConfig
    monkeypatch.setattr(cli, 'SensorConfig', lambda **kwargs: SensorConfig(
        **kwargs, acceleration_bias_mps2=(.1, -.2, .3), gyro_bias_rad_s=.01,
        dropout_probability=.25))


def test_python_on_off_capture_does_not_change_state_noise_or_results(environment_factory, monkeypatch):
    _noise(monkeypatch)
    off = environment_factory(capture_mode='off')
    full = environment_factory(capture_mode='full')
    for torque in (0., 2., 1.):
        a, b = off.step(RideControl(torque)), full.step(RideControl(torque))
        assert asdict(a) == asdict(b)
        np.testing.assert_array_equal(integration_state(off.sim), integration_state(full.sim))
        assert off.pipeline.samples_attempted == full.pipeline.samples_attempted
        assert off.pipeline.samples_dropped == full.pipeline.samples_dropped
        assert off.tracker.metrics == full.tracker.metrics
    assert not off.commands_requested and not off.commands_applied
    assert not off.observations and not off.trace and off.recorder.rows == 0
    assert not off._capture_samples
    assert len(full.commands_requested) == len(full.trace) == 3
    assert len(full.observations) == 4 and full.recorder.rows == 0
    assert len(full._capture_samples) == full.sim.steps


def test_sealed_python_source_survives_reset_close_and_forbids_array_mutation(environment_factory):
    env = environment_factory(capture_mode='full', decimation=3)
    env.step(RideControl(0.))
    evidence = env.seal_episode(0, reason='reset')
    first = evidence.physical.episode.samples[0]
    held = first.qpos.copy()
    with pytest.raises(ValueError):
        first.qpos.setflags(write=True)
    with pytest.raises(ValueError):
        evidence.research.initial_integration_state.setflags(write=True)
    with pytest.raises(RuntimeError, match='sealed'):
        env.begin_control(RideControl(0.))
    assert env.seal_episode(0, reason='reset') is evidence
    env.reset()
    env.close()
    np.testing.assert_array_equal(first.qpos, held)
    assert sum(batch.row_count for batch in evidence.physical.tables(batch_rows=3)
               if batch.table == 'physics') == evidence.physical.row_count
    assert evidence.summary['outcome']['reason'] == 'not_finished'
    assert evidence.summary['seal_reason'] == 'reset'


def test_python_verify_suffix_is_consumed_only_after_freezing(environment_factory, monkeypatch):
    import bike_sim.recording.contracts as contracts
    env = environment_factory(capture_mode='verify')
    original = contracts.freeze
    def fail(value):
        raise MemoryError('freeze injection')
    monkeypatch.setattr(contracts, 'freeze', fail)
    with pytest.raises(MemoryError):
        env.applied_since(0)
    monkeypatch.setattr(contracts, 'freeze', original)
    initial, cursor = env.applied_since(0)
    assert len(initial) == cursor == 1
    assert env.commands_applied == []
    assert env.applied_since(cursor) == ((), cursor)
    with pytest.raises(ValueError, match='stale'):
        env.applied_since(0)
    with pytest.raises(ValueError):
        env.applied_since(True)
    env.step(RideControl(2.))
    suffix, end = env.applied_since(cursor)
    assert end == cursor + len(suffix)
    assert not env.commands_requested and not env.observations and not env.trace
    assert env.last_requested['control']['motor_torque_nm'] == 2.


def test_two_episode_launch_writes_only_after_finalization(environment_factory, tmp_path):
    env = environment_factory(capture_mode='full')
    output = tmp_path / 'recording'
    launch = ResearchRun(env, output, enabled=True, mode='test')
    env.step(RideControl(0.))
    launch.reset()
    assert not output.exists()
    env.step(RideControl(1.))
    env.stop()
    launch.seal(reason=env.reason)
    assert not output.exists()
    path = launch.finish()
    env.close()
    with Recording(path) as reader:
        assert reader.episode_ids == (0, 1)
        with pytest.raises(ValueError, match='select --episode'):
            reader.episode()
        for identity in (0, 1):
            assert reader.episode(identity)['complete_control_steps'] == 1
        assert reader.rows('SELECT count(*) AS n FROM observations')[0]['n'] == 4
    assert launch.finish() == path


def test_incomplete_window_and_failed_policy_remain_analyzable(environment_factory, tmp_path):
    from bike_sim.recording.replay_input import load_replay_input
    env = environment_factory(capture_mode='full')
    env.begin_control(RideControl(0.))
    env.advance_control(target_step=1)
    launch = ResearchRun(env, tmp_path, enabled=True, mode='test')
    # A failure seal owns the actual prefix, not a fabricated completed step.
    launch.seal(reason='simulation_error')
    path = launch.capture.finalize(tmp_path)
    env.close(discard_pending=True)
    with Recording(path) as reader:
        selected = reader.episode()
        assert selected['complete_control_steps'] == 0
        assert selected['physical_rows'] == 1
        assert not selected['replay_eligible']
        assert reader.rows('SELECT count(*) AS n FROM commands_requested')[0]['n'] == 1
        assert reader.rows('SELECT count(*) AS n FROM observations')[0]['n'] == 1
    with pytest.raises(ValueError, match='incomplete-prefix'):
        load_replay_input(path)


def test_policy_failure_before_first_request_seals_zero_step_evidence(environment_factory, tmp_path):
    env = environment_factory(capture_mode='full')
    launch = ResearchRun(env, tmp_path, enabled=True, mode='test')
    error = RuntimeError('policy setup injection')
    env.fail_policy(error)
    path = launch.finish(primary=error, stop=True)
    with Recording(path) as reader:
        assert reader.episode()['physical_rows'] == 0
        assert not reader.episode()['replay_eligible']
        assert reader.rows('SELECT count(*) AS n FROM observations')[0]['n'] == 1


def test_interruption_drains_only_the_already_accepted_control(environment_factory, tmp_path):
    from bike_sim.sim.research.policy_session import PolicySession
    from test_research_frontend import CountingPolicy
    env = environment_factory(capture_mode='full')
    policy = CountingPolicy()
    launch = ResearchRun(env, tmp_path, enabled=True, mode='test')
    session = PolicySession(env, policy)
    launch.attach(session)
    session.begin_advance()
    session.advance_pending(target_step=1)
    path = launch.finish(primary=KeyboardInterrupt(), stop=True)
    assert policy.calls == 1 and env.sim.steps == env.control_steps
    assert not env.control_pending
    with Recording(path) as reader:
        assert reader.episode()['complete_control_steps'] == 1
        kinds = {row['kind'] for row in reader.rows('SELECT kind FROM events')}
        assert 'launch_interrupted' in kinds
