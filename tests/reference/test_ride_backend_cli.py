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
