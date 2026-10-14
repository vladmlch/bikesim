"""Research CLI opt-in and zero-step policy-load failure contracts."""
import signal
import threading
from datetime import datetime, timezone
from uuid import uuid4

import pytest
from bike_sim.cli import research
from bike_sim.recording.capture import RunCapture
from bike_sim.recording.contracts import RunMetadata
from bike_sim.recording.interrupt import InterruptShield
from bike_sim.recording.reader import Recording
from bike_sim.recording.storage import write_recording
from bike_sim.sim.ride.control import RideControl


def _finish_capture(env, publisher=None):
    """An enabled launch capture; ``publisher`` may wrap the real seam."""
    return RunCapture(RunMetadata(uuid4(), datetime.now(timezone.utc),
                                  env.backend, 'test', {}),
                      enabled=True, _publisher=publisher)


def _event_kinds(path):
    with Recording(path) as saved:
        return [row['kind'] for row in saved.rows(
            'SELECT kind FROM events ORDER BY episode_id,event_id')]


def test_research_parser_defaults_off_and_resolves_check_policy():
    args = research.parser().parse_args([])
    assert args.record is False
    assert research.resolve_check_policy(args) == 'strict'
    for policy in ('strict', 'diagnostic'):
        args = research.parser().parse_args(['--record', '--check-policy', policy])
        assert args.record is True and research.resolve_check_policy(args) == policy
    args = research.parser().parse_args(['--check-policy', 'strict', '--diagnostic-model-limits'])
    with pytest.raises(ValueError):
        research.resolve_check_policy(args)


def test_policy_import_failure_publishes_zero_step_evidence(environment_factory, tmp_path, monkeypatch):
    env = environment_factory(capture_mode='full')
    monkeypatch.setattr(research, 'make_environment', lambda args, **kwargs: env)
    def fail(reference):
        raise ValueError('policy factory load injection')
    monkeypatch.setattr(research, 'load_policy', fail)
    output = tmp_path / 'run'
    assert research.main(['--record', '--out', str(output), '--policy', 'missing:factory']) == 3
    manifests = list(output.glob('*/manifest.json'))
    assert len(manifests) == 1
    with Recording(manifests[0].parent) as saved:
        episode = saved.episode()
        assert episode['physical_rows'] == 0 and episode['complete_control_steps'] == 0
        assert not episode['replay_eligible']
        assert saved.rows('SELECT count(*) AS n FROM commands_requested')[0]['n'] == 0
        assert saved.rows('SELECT count(*) AS n FROM observations')[0]['n'] == 1


def test_out_without_record_never_publishes(environment_factory, tmp_path, monkeypatch):
    env = environment_factory(capture_mode='off', duration=.01)
    monkeypatch.setattr(research, 'make_environment', lambda args, **kwargs: env)
    output = tmp_path / 'run'
    research.main(['--out', str(output)])
    assert not output.exists()
    assert not env.commands_requested and not env.commands_applied
    assert not env.observations and not env.trace and env.recorder.rows == 0


def test_finish_second_ctrl_c_during_seal_journals_and_publishes(
        environment_factory, tmp_path, monkeypatch):
    """R04: a second Ctrl-C deferred inside the shielded seal is journaled
    while events are still accepted, the episode publishes, then the
    interrupt is honored — finish exits as an operator stop."""
    from bike_sim.sim.research.recording_lifecycle import ResearchRun
    env = environment_factory(capture_mode='full')
    launch = ResearchRun(env, tmp_path, enabled=True, mode='test')
    env.step(RideControl(0.))
    original = env.seal_episode
    def interrupted(episode_id, *, reason):
        signal.raise_signal(signal.SIGINT)
        return original(episode_id, reason=reason)
    monkeypatch.setattr(env, 'seal_episode', interrupted)
    with pytest.raises(KeyboardInterrupt):
        launch.finish()
    assert launch.path is not None
    assert (launch.path/'manifest.json').is_file()
    assert (launch.path/'run.duckdb').is_file()
    kinds = _event_kinds(launch.path)
    assert kinds[0] == 'episode_started'
    assert 'episode_ended' in kinds
    assert 'operator_stop_deferred' in kinds


def test_finish_second_ctrl_c_during_finalize_is_noted_and_publishes(
        environment_factory, tmp_path):
    """Deferred inside ``finalize`` lands too late to journal; the
    re-raised interrupt carries the note and the manifest exists."""
    from bike_sim.sim.research.recording_lifecycle import ResearchRun
    env = environment_factory(capture_mode='full')
    def publish(capture, out_root):
        signal.raise_signal(signal.SIGINT)
        return write_recording(capture, out_root)
    launch = ResearchRun(env, tmp_path, enabled=True, mode='test',
                         capture=_finish_capture(env, publish))
    env.step(RideControl(0.))
    with pytest.raises(KeyboardInterrupt) as excinfo:
        launch.finish()
    assert any('operator interrupt deferred' in note
               for note in getattr(excinfo.value, '__notes__', ()))
    assert launch.path is not None
    assert (launch.path/'manifest.json').is_file()
    assert 'operator_stop_deferred' not in _event_kinds(launch.path)


def test_finish_off_main_thread_shield_degrades_but_records(
        environment_factory, tmp_path, monkeypatch):
    """Off the main thread no SIGINT handler can be installed, so the
    shield is a flag holder — the injected ``note()`` seam still records
    the deferred interrupt deterministically."""
    import bike_sim.sim.research.recording_lifecycle as lifecycle
    created = []
    class Probe(InterruptShield):
        def __init__(self):
            super().__init__()
            created.append(self)
    monkeypatch.setattr(lifecycle, 'InterruptShield', Probe)
    env = environment_factory(capture_mode='full')
    def publish(capture, out_root):
        created[-1].note()
        return write_recording(capture, out_root)
    launch = lifecycle.ResearchRun(env, tmp_path, enabled=True, mode='test',
                                   capture=_finish_capture(env, publish))
    env.step(RideControl(0.))
    outcome = []
    def run():
        try:
            launch.finish()
        except BaseException as error:
            outcome.append(error)
    worker = threading.Thread(target=run)
    worker.start()
    worker.join(timeout=60)
    assert not worker.is_alive()
    assert len(created) == 1 and created[0]._previous is None
    assert len(outcome) == 1 and isinstance(outcome[0], KeyboardInterrupt)
    assert any('operator interrupt deferred' in note
               for note in getattr(outcome[0], '__notes__', ()))
    assert launch.path is not None
    assert (launch.path/'manifest.json').is_file()


def test_frontend_failure_between_windows_is_not_replay_eligible(environment_factory, tmp_path):
    from bike_sim.recording.json_codec import loads
    from bike_sim.sim.research.recording_lifecycle import ResearchRun
    from bike_sim.sim.ride.control import RideControl
    env = environment_factory(capture_mode='full')
    launch = ResearchRun(env, tmp_path, enabled=True, mode='test')
    env.step(RideControl(0.))
    path = launch.finish(primary=RuntimeError('frontend callback injection'), stop=True)
    with Recording(path) as saved:
        episode = saved.episode()
        assert episode['complete_control_steps'] == 1
        assert not episode['replay_eligible']
        summary = loads(episode['summary_json'])
        assert not summary['research']['valid_for_learning']
        assert summary['research']['run_metadata']['launch_failure']['error_type'] == 'RuntimeError'
