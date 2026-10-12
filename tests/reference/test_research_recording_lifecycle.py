"""Research CLI opt-in and zero-step policy-load failure contracts."""
import pytest
from bike_sim.cli import research
from bike_sim.recording.reader import Recording


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
