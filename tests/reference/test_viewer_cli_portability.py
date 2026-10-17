"""CLI routing regressions with no windows, physics or process replacement."""
from pathlib import Path
from types import SimpleNamespace

import pytest


PROFILE = str(Path(__file__).resolve().parents[2]
              / 'examples/research/viewer_physics_welded.toml')


def forbidden(*args, **kwargs):
    raise AssertionError('work ran outside the selected CLI route')


@pytest.mark.parametrize('arguments', [
    ['--physics-config', PROFILE],
    ['--physics-config', PROFILE, '--research'],
    ['--physics', 'legacy'],
])
def test_ride_prepares_viewer_before_native_track_or_session_setup(monkeypatch, arguments):
    from bike_sim.cli import ride
    import bike_sim.viewer_runtime as runtime
    seen = []
    def unavailable(**kwargs):
        seen.append(kwargs)
        raise runtime.ViewerUnavailableError('no desktop')
    monkeypatch.setattr(runtime, 'prepare_viewer', unavailable)
    monkeypatch.setattr(ride, 'resolve_rider', forbidden)
    monkeypatch.setattr(ride, 'resolve_track', forbidden)
    assert ride.main(arguments) == 2
    assert seen == [{'argv': arguments, 'module': 'bike_sim.cli.ride'}]


def test_headless_ride_does_not_prepare_a_viewer(monkeypatch):
    from bike_sim.cli import ride
    import bike_sim.sim.backend as backend
    import bike_sim.viewer_runtime as runtime
    monkeypatch.setattr(runtime, 'prepare_viewer', forbidden)
    monkeypatch.setattr(runtime, 'load_viewer', forbidden)
    monkeypatch.setattr(ride, 'resolve_rider',
                        lambda args: SimpleNamespace(variant='lumped', present=True))
    monkeypatch.setattr(ride, 'resolve_track',
                        lambda *args, **kwargs: SimpleNamespace(length_m=10.))
    monkeypatch.setattr(backend, 'require_backend', lambda *args: None)
    monkeypatch.setattr(ride, 'track_seed', lambda *args: 0)
    monkeypatch.setattr(ride, '_tyre_config', lambda args: object())
    monkeypatch.setattr(ride, '_headless', lambda *args: 7)
    assert ride.main(['--physics-config', PROFILE, '--headless',
                      '--duration', '1', '--no-plots']) == 7


@pytest.mark.parametrize('flag', ['--view', '--viewer'])
def test_replay_prepares_before_loading_any_recording_owner(monkeypatch, flag):
    from bike_sim.cli import replay
    from bike_sim.sim.research import playback, replay_session
    import bike_sim.viewer_runtime as runtime
    def unavailable(**kwargs):
        raise runtime.ViewerUnavailableError('no desktop')
    monkeypatch.setattr(runtime, 'prepare_viewer', unavailable)
    monkeypatch.setattr(playback, 'load_playback', forbidden)
    monkeypatch.setattr(replay_session, 'ReplaySession', forbidden)
    assert replay.main(['unused-recording', flag]) == 1


def test_invalid_playback_fps_remains_a_usage_error_before_gui_preflight(monkeypatch):
    from bike_sim.cli import replay
    import bike_sim.viewer_runtime as runtime
    monkeypatch.setattr(runtime, 'prepare_viewer', forbidden)
    with pytest.raises(SystemExit) as failure:
        replay.main(['unused-recording', '--view', '--fps', '0'])
    assert failure.value.code == 2


def test_playback_dry_run_never_prepares_or_loads_a_viewer(monkeypatch):
    from bike_sim.cli import replay
    from bike_sim.sim.research import playback
    from bike_sim.sim.ride import playback_viewer
    import bike_sim.viewer_runtime as runtime
    recording = object()
    monkeypatch.setattr(runtime, 'prepare_viewer', forbidden)
    monkeypatch.setattr(runtime, 'load_viewer', forbidden)
    monkeypatch.setattr(playback, 'load_playback', lambda path: recording)
    seen = []
    def dry_run(loaded, *, fps):
        seen.append((loaded, fps))
        return 0
    monkeypatch.setattr(playback_viewer, 'run_playback_dry_run', dry_run)
    assert replay.main(['unused-recording', '--view', '--dry-run', '--fps', '30']) == 0
    assert seen == [(recording, 30.)]


def test_checked_replay_default_never_prepares_a_viewer(monkeypatch):
    from bike_sim.cli import replay
    import bike_sim.viewer_runtime as runtime
    monkeypatch.setattr(runtime, 'prepare_viewer', forbidden)
    monkeypatch.setattr(runtime, 'load_viewer', forbidden)
    monkeypatch.setattr(replay, 'replay_episode', lambda *args, **kwargs: {'checked': True})
    assert replay.main(['unused-recording']) == 0


def test_playground_cli_refuses_before_simulation_creation(monkeypatch):
    from bike_sim.cli import playground
    import bike_sim.viewer_runtime as runtime
    def unavailable(**kwargs):
        raise runtime.ViewerUnavailableError('no desktop')
    monkeypatch.setattr(runtime, 'prepare_viewer', unavailable)
    monkeypatch.setattr(playground, 'run_interactive_playground', forbidden)
    assert playground.main([]) == 2


def test_main_playground_refuses_before_analysis_or_output_files(monkeypatch, tmp_path):
    from bike_sim.cli import main
    import bike_sim.viewer_runtime as runtime
    def unavailable(**kwargs):
        raise runtime.ViewerUnavailableError('no desktop')
    monkeypatch.setattr(runtime, 'prepare_viewer', unavailable)
    monkeypatch.setattr(main, 'BikeSpecs', forbidden)
    monkeypatch.setattr(main, 'run_interactive_playground', forbidden)
    assert main.main(['--playground', '--all',
                      '--output-plots-dir', str(tmp_path / 'plots'),
                      '--output-models-dir', str(tmp_path / 'models')]) == 2
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize('quit_key', [ord('Q'), 256])
def test_playback_quit_during_launch_closes_window_once(monkeypatch, quit_key):
    from bike_sim.sim.ride import playback_viewer
    import bike_sim.viewer_runtime as runtime
    closed = []
    class Window:
        def __enter__(self):
            return self
        def __exit__(self, *args):
            closed.append('closed')
        def is_running(self):
            return True
    def launch(*args, key_callback, **kwargs):
        key_callback(quit_key)  # No handle has been assigned in the frontend yet.
        return Window()
    monkeypatch.setattr(runtime, 'load_viewer',
                        lambda: SimpleNamespace(launch_passive=launch))
    monkeypatch.setattr(playback_viewer, 'CameraManager',
                        lambda **kwargs: SimpleNamespace(reset_preset=lambda: None))
    recording = SimpleNamespace(
        sim=SimpleNamespace(model=object(), data=object()), duration_s=0., times_s=[])
    assert playback_viewer.run_playback_viewer(recording) == 0
    assert closed == ['closed']
