from pathlib import Path
from types import SimpleNamespace
import pytest
from bike_sim.cli.ride import parse_args

ROOT = Path(__file__).resolve().parents[2]
PROFILE = str(ROOT/'examples/research/viewer_physics_welded.toml')


def test_default_backend_and_profile_timestep():
    args = parse_args(['--physics-config', PROFILE])
    assert args.backend == 'python'
    assert args.time_scale == 1
    assert args.resolved_physics.timestep_s == .00125


def test_native_visual_scale_keeps_all_physics_settings():
    normal = parse_args(['--physics-config', PROFILE])
    fast = parse_args(['--backend', 'native', '--physics-config', PROFILE, '--time-scale', '4'])
    assert fast.time_scale == 4
    assert fast.resolved_physics == normal.resolved_physics


@pytest.mark.parametrize('scale', ['1', '2', '4', '8'])
def test_explicit_scale_is_invalid_headless_even_one(scale):
    with pytest.raises(SystemExit) as error:
        parse_args(['--physics-config', PROFILE, '--headless', '--time-scale', scale])
    assert error.value.code == 2


def test_legacy_scale_and_native_rejected():
    for values in (['--time-scale', '1'], ['--backend', 'native']):
        with pytest.raises(SystemExit):
            parse_args(['--physics', 'legacy', *values])


def test_headless_native_profile_does_not_replace_timestep():
    args = parse_args(['--backend', 'native', '--physics-config', PROFILE,
                      '--headless', '--duration', '1', '--no-plots'])
    assert args.time_scale == 1
    assert args.resolved_physics.timestep_s == .00125


def test_research_cli_has_backend_but_no_playback_scale():
    from bike_sim.cli.research import parser
    assert parser().parse_args([]).backend == 'python'
    assert parser().parse_args(['--backend', 'native']).backend == 'native'
    with pytest.raises(SystemExit):
        parser().parse_args(['--time-scale', '1'])


def test_replay_requires_viewer_for_explicit_scale():
    from bike_sim.cli.replay import main
    with pytest.raises(SystemExit) as error:
        main(['missing-recording', '--time-scale', '1'])
    assert error.value.code == 2


def test_native_preflight_failure_precedes_ride_setup(monkeypatch):
    import bike_sim.sim.ride.physical_driver as driver
    import bike_sim.sim.ride.physical_session as session
    def refused(*args):
        raise ValueError('selected artifact is missing')
    def forbidden(*args):
        raise AssertionError('setup ran despite rejected native request')
    monkeypatch.setattr(driver, 'require_backend', refused)
    monkeypatch.setattr(session, 'build_physical_simulation', forbidden)
    with pytest.raises(ValueError, match='artifact'):
        driver.build_physical_driver(None, SimpleNamespace(backend='native', resolved_physics=None), None)


def test_missing_artifact_names_directory_commands_and_code(tmp_path, monkeypatch, capsys):
    from bike_sim.cli.ride import main
    missing = tmp_path/'empty-build'
    missing.mkdir()
    monkeypatch.setenv('BIKE_NATIVE_BUILD_PATH', str(missing))
    assert main(['--backend', 'native', '--physics-config', PROFILE,
                 '--headless', '--duration', '1', '--no-plots',
                 '--out', str(tmp_path/'out')]) == 2
    error = capsys.readouterr().err
    assert str(missing.resolve()) in error
    assert 'cmake -S native -B native/build/release -DCMAKE_BUILD_TYPE=Release' in error
    assert 'cmake --build native/build/release' in error


def test_python_command_never_loads_the_extension(tmp_path, monkeypatch):
    """A forced native import failure must not affect a plain Python command."""
    import bike_sim.native.artifact as artifact
    import bike_sim.sim.ride.physical_session as physical_session
    from bike_sim.cli.ride import main
    def forbidden(*args, **kwargs):
        raise AssertionError('python command imported the native extension')
    monkeypatch.setattr(artifact, 'load_native_extension', forbidden)
    monkeypatch.setattr(physical_session, 'run_physical_headless',
                        lambda track, args, seed, rider: 0)
    assert main(['--physics-config', PROFILE, '--headless',
                 '--duration', '1', '--no-plots', '--out', str(tmp_path/'out')]) == 0


def test_cli_dispatches_every_live_and_headless_route(tmp_path, monkeypatch):
    import bike_sim.sim.backend as backend
    import bike_sim.sim.playground as playground
    import bike_sim.sim.ride.physical_driver as physical_driver
    import bike_sim.sim.ride.physical_frontend as physical_frontend
    import bike_sim.sim.ride.physical_session as physical_session
    import bike_sim.sim.research.viewer as research_viewer
    import bike_sim.cli.research as research_cli
    from bike_sim.cli.ride import main
    monkeypatch.setattr(backend, 'require_backend',
                        lambda *args: SimpleNamespace(name='selected-extension'))
    monkeypatch.setattr(playground, 'ensure_macos_mjpython', lambda: None)
    calls = []
    driver = SimpleNamespace(backend='native')
    monkeypatch.setattr(physical_driver, 'build_physical_driver',
                        lambda track, args, rider, **kwargs: driver)
    def viewer(d, *, out_root, time_scale):
        calls.append(('ride-viewer', d is driver, out_root, time_scale))
        return 0
    monkeypatch.setattr(physical_frontend, 'run_viewer', viewer)
    monkeypatch.setattr(physical_session, 'run_physical_headless',
                        lambda track, args, seed, rider:
                        calls.append(('ride-headless', args.backend)) or 7)
    monkeypatch.setattr(research_viewer, 'run_ride_research',
                        lambda track, args, rider:
                        calls.append(('ride-research', args.backend, args.time_scale)) or 0)
    seen = {}
    def environment(args):
        seen['args'] = args
        return SimpleNamespace(done=True, save=lambda *a, **k: None,
            reason='duration_reached', error=None,
            tracker=SimpleNamespace(metrics={}), close=lambda *a, **k: None)
    monkeypatch.setattr(research_cli, 'make_environment', environment)
    monkeypatch.setattr(research_cli, 'episode_exit_code', lambda env: 9)

    out = str(tmp_path/'out')
    assert main(['--backend', 'native', '--physics-config', PROFILE,
                 '--time-scale', '2', '--out', out]) == 0
    assert main(['--backend', 'native', '--physics-config', PROFILE, '--headless',
                 '--duration', '1', '--no-plots', '--out', out]) == 7
    assert main(['--backend', 'native', '--physics-config', PROFILE,
                 '--research', '--out', out]) == 0
    assert research_cli.main(['--backend', 'native', '--duration', '1',
                              '--out', str(tmp_path/'research')]) == 9
    assert calls[0] == ('ride-viewer', True, out, 2)
    assert calls[1] == ('ride-headless', 'native')
    assert calls[2] == ('ride-research', 'native', 1)
    assert seen['args'].backend == 'native'
