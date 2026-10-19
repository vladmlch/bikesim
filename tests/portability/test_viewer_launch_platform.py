"""Launcher and ownership regressions with fake windows, never desktop evidence.

These controls require the project Python dependencies, but no native extension,
GLFW context, simulation steps or real re-exec. Real host acceptance is separate.
"""
import builtins
import os
import shutil
import sys
from types import ModuleType, SimpleNamespace

import mujoco
import pytest

from bike_sim.sim import playground
from bike_sim.sim.research import replay_viewer, viewer as research_viewer
from bike_sim.sim.ride import physical_frontend, playback_viewer, viewer as ride_viewer


def forbidden(*args, **kwargs):
    raise AssertionError('unexpected launcher, renderer or physics operation')


@pytest.fixture
def darwin_launcher(monkeypatch):
    monkeypatch.setattr(sys, 'platform', 'darwin')
    monkeypatch.setattr(sys, 'executable', '/selected environment/bin/python')
    monkeypatch.setattr(sys, 'base_prefix', '/base interpreter')
    monkeypatch.setattr(sys, '_mjpython_checked', False, raising=False)
    environment = dict(os.environ)
    environment.pop('MUJOCO_MJPYTHON', None)
    environment.pop('DYLD_FALLBACK_LIBRARY_PATH', None)
    # A plain copy keeps helper mutations out of the runner's real environment.
    monkeypatch.setattr(os, 'environ', environment)
    return environment


@pytest.mark.parametrize('sentinel', [None, False, True])
def test_linux_never_looks_up_or_reexecutes_mjpython(monkeypatch, sentinel):
    monkeypatch.setattr(sys, 'platform', 'linux')
    monkeypatch.setattr(sys, '_mjpython_checked', sentinel, raising=False)
    if sentinel is None:
        monkeypatch.delattr(sys, '_mjpython_checked')
    environment = dict(os.environ, MUJOCO_MJPYTHON='keep-linux-marker',
                       DYLD_FALLBACK_LIBRARY_PATH='/keep/linux/path')
    monkeypatch.setattr(os, 'environ', environment)
    before = dict(environment)
    monkeypatch.setattr(shutil, 'which', forbidden)
    monkeypatch.setattr(os, 'execv', forbidden)
    monkeypatch.setattr(os.path, 'exists', forbidden)
    playground.ensure_macos_mjpython()
    assert getattr(sys, '_mjpython_checked', None) is sentinel
    assert hasattr(sys, '_mjpython_checked') is (sentinel is not None)
    assert environment == before


@pytest.mark.parametrize('already_running', ['sentinel', 'marker', 'executable'])
def test_darwin_existing_launcher_does_not_relaunch(
        monkeypatch, darwin_launcher, already_running):
    if already_running == 'sentinel':
        monkeypatch.setattr(sys, '_mjpython_checked', True)
    elif already_running == 'marker':
        darwin_launcher['MUJOCO_MJPYTHON'] = '1'
    else:
        monkeypatch.setattr(sys, 'executable', '/selected environment/bin/mjpython')
    before = dict(darwin_launcher)
    monkeypatch.setattr(shutil, 'which', forbidden)
    monkeypatch.setattr(os.path, 'exists', forbidden)
    monkeypatch.setattr(os, 'execv', forbidden)
    playground.ensure_macos_mjpython()
    assert sys._mjpython_checked is True
    assert darwin_launcher == before


@pytest.mark.parametrize('fallback', [None, '', '/existing/one:/existing/two'])
@pytest.mark.parametrize('dylib_exists', [False, True])
def test_darwin_reexec_preserves_argv_and_fallback(
        monkeypatch, darwin_launcher, fallback, dylib_exists):
    if fallback is not None:
        darwin_launcher['DYLD_FALLBACK_LIBRARY_PATH'] = fallback
    original_argv = ['/entry point/bike-ride', '--track', 'a b.toml', '--time-scale', '8']
    monkeypatch.setattr(sys, 'argv', original_argv.copy())
    executable = '/selected environment/bin/mjpython'
    lookups, probes, executions = [], [], []

    def which(command):
        lookups.append(command)
        return executable

    def exists(path):
        probes.append(path)
        return dylib_exists

    def reexec(path, argv):
        executions.append((path, argv.copy(), dict(os.environ)))

    monkeypatch.setattr(shutil, 'which', which)
    monkeypatch.setattr(os.path, 'exists', exists)
    monkeypatch.setattr(os, 'execv', reexec)
    playground.ensure_macos_mjpython()
    # A real execv replaces the process; the returning double also exercises
    # the in-process sentinel without launching another interpreter.
    playground.ensure_macos_mjpython()
    # A fresh interpreter retains the environment marker, not the sys sentinel.
    monkeypatch.setattr(sys, '_mjpython_checked', False)
    playground.ensure_macos_mjpython()
    assert lookups == ['mjpython']
    version = f'{sys.version_info.major}.{sys.version_info.minor}'
    assert probes == [f'/base interpreter/lib/libpython{version}.dylib']
    assert len(executions) == 1
    path, argv, environment = executions[0]
    assert path == executable and argv == [executable, *original_argv]
    assert sys.argv == original_argv
    assert environment['MUJOCO_MJPYTHON'] == '1'
    expected = (f'/base interpreter/lib:{fallback}' if fallback else '/base interpreter/lib') \
        if dylib_exists else fallback
    assert environment.get('DYLD_FALLBACK_LIBRARY_PATH') == expected
    assert ('DYLD_FALLBACK_LIBRARY_PATH' in environment) is (expected is not None)


def test_missing_macos_launcher_is_actionable_and_retryable(monkeypatch, darwin_launcher):
    lookups = []

    def missing(command):
        lookups.append(command)
        return None

    monkeypatch.setattr(shutil, 'which', missing)
    monkeypatch.setattr(os.path, 'exists', forbidden)
    monkeypatch.setattr(os, 'execv', forbidden)
    before = dict(darwin_launcher)
    for _ in range(2):
        with pytest.raises(RuntimeError, match='macOS interactive viewer requires mjpython') as caught:
            playground.ensure_macos_mjpython()
        assert 'same Python environment' in str(caught.value)
        assert not sys._mjpython_checked
        assert darwin_launcher == before
    assert lookups == ['mjpython', 'mjpython']


@pytest.mark.parametrize('marker,fallback', [(None, None), ('prior-marker', '/prior/lib'), ('', '')])
def test_failed_macos_exec_restores_environment_and_allows_retry(
        monkeypatch, darwin_launcher, marker, fallback):
    for name, value in [('MUJOCO_MJPYTHON', marker), ('DYLD_FALLBACK_LIBRARY_PATH', fallback)]:
        if value is not None:
            darwin_launcher[name] = value
    before = dict(darwin_launcher)
    calls = []
    primary = OSError('selected mjpython executable could not be started')
    monkeypatch.setattr(shutil, 'which', lambda command: '/selected/bin/mjpython')
    monkeypatch.setattr(os.path, 'exists', lambda path: True)

    def fail(path, argv):
        calls.append(path)
        assert darwin_launcher['MUJOCO_MJPYTHON'] == '1'
        raise primary

    monkeypatch.setattr(os, 'execv', fail)
    for _ in range(2):
        with pytest.raises(OSError) as caught:
            playground.ensure_macos_mjpython()
        assert caught.value is primary
        assert not sys._mjpython_checked
        assert darwin_launcher == before
    assert calls == ['/selected/bin/mjpython', '/selected/bin/mjpython']


@pytest.mark.parametrize('entry', [
    playground.run_interactive_playground,
    ride_viewer.run_interactive_ride,
    lambda: ride_viewer.run_physical_viewer(object()),
    lambda: ride_viewer.run_physical_viewer(
        SimpleNamespace(physical=SimpleNamespace(set_async_close=forbidden)),
        async_telemetry=True),
], ids=['playground', 'legacy_ride', 'resolved_physical', 'resolved_physical_async'])
def test_launcher_runs_before_viewer_import_or_model_setup(monkeypatch, entry):
    primary = RuntimeError('launcher not ready')
    original_import = builtins.__import__

    def guarded_import(name, *args, **kwargs):
        if name == 'mujoco.viewer':
            forbidden()
        return original_import(name, *args, **kwargs)

    def launch():
        raise primary

    monkeypatch.setattr(builtins, '__import__', guarded_import)
    monkeypatch.setattr(playground, 'ensure_macos_mjpython', launch)
    monkeypatch.setattr(playground, 'SuspensionPlayground', forbidden)
    monkeypatch.setattr(ride_viewer, 'RideSimulation', forbidden)
    with pytest.raises(RuntimeError) as caught:
        entry()
    assert caught.value is primary


class Window:
    def __init__(self, failure=None):
        self.failure = failure
        self.entered = 0
        self.exits = []

    def __enter__(self):
        self.entered += 1
        return self

    def __exit__(self, kind, value, traceback):
        self.exits.append(value)

    def is_running(self):
        if self.failure is not None:
            raise self.failure
        return False

    def close(self):
        forbidden()  # Closing belongs to context teardown, never a key callback.


@pytest.fixture
def fake_viewer(monkeypatch):
    viewer = ModuleType('mujoco.viewer')
    monkeypatch.setitem(sys.modules, 'mujoco.viewer', viewer)
    monkeypatch.setattr(mujoco, 'viewer', viewer, raising=False)
    return viewer


@pytest.mark.parametrize('key', [ord('Q'), ord('q'), 256])
def test_playback_quit_during_launch_is_deferred_to_window_owner(monkeypatch, fake_viewer, key):
    window = Window()
    monkeypatch.setattr(window, 'is_running', lambda: True)
    monkeypatch.setattr(playback_viewer.time, 'sleep', forbidden)
    monkeypatch.setattr(playground, 'ensure_macos_mjpython', lambda: None)
    monkeypatch.setattr(playback_viewer, 'CameraManager',
                        lambda **kwargs: SimpleNamespace(reset_preset=lambda: None))
    playback = SimpleNamespace(sim=SimpleNamespace(model=object(), data=object()),
                               duration_s=1., times_s=[0., 1.])

    def launch(model, data, *, key_callback, **kwargs):
        # MuJoCo may dispatch a key while its launch call still owns the handle.
        key_callback(key)
        return window

    fake_viewer.launch_passive = launch
    assert playback_viewer.run_playback_viewer(playback) == 0
    assert window.entered == 1 and window.exits == [None]


class Owner:
    """Minimal unchanged driver/session surface; every physical step is forbidden."""
    backend = 'native'
    dt_s = timestep_s = .00125
    time_s = 0.
    step = 0
    reason = error = None
    done = False

    def __init__(self, close_error=None):
        self.close_error, self.close_calls, self.flush_calls = close_error, 0, 0
        self.metadata = dict(drive_mode='articulated_effort', configuration_sha256='0'*64)
        self.track = SimpleNamespace(name='viewer-fixture')

    def make_render_model(self):
        return object()

    def snapshot(self):
        return SimpleNamespace(step=0, generation=1, time_s=0.)

    def start(self):
        pass

    def advance(self, *args, **kwargs):
        forbidden()

    def flush(self):
        self.flush_calls += 1

    def summary(self, reason=None):
        return dict(outcome=dict(reason=reason), model_status=dict(model_valid=True))

    def close(self):
        self.close_calls += 1
        if self.close_error is not None:
            raise self.close_error


def presenter(model):
    return SimpleNamespace(replica=SimpleNamespace(model=model, data=object(), apply=lambda frame: None))


def invoke(frontend, owner, tmp_path):
    if frontend == 'physical':
        return physical_frontend.run_viewer(owner, out_root=tmp_path)
    return replay_viewer.run_replay_viewer(owner)


@pytest.mark.parametrize('frontend', ['physical', 'replay'])
@pytest.mark.parametrize('stage', ['import', 'launch', 'interrupt'])
@pytest.mark.parametrize('close_fails', [False, True])
def test_viewer_failure_closes_owner_and_preserves_primary(
        monkeypatch, fake_viewer, tmp_path, frontend, stage, close_fails):
    primary = (KeyboardInterrupt() if stage == 'interrupt'
               else ImportError('GLFW library unavailable') if stage == 'import'
               else RuntimeError('GLFW context unavailable'))
    owner = Owner(RuntimeError('owner close failed') if close_fails else None)
    window = Window(primary if stage == 'interrupt' else None)
    monkeypatch.setattr(physical_frontend, 'FramePresenter', presenter)
    monkeypatch.setattr(replay_viewer, 'FramePresenter', presenter)

    def launch(*args, **kwargs):
        if stage == 'launch':
            raise primary
        return window

    fake_viewer.launch_passive = launch
    if stage == 'import':
        original_import = builtins.__import__

        def fail_import(name, *args, **kwargs):
            if name == 'mujoco.viewer':
                raise primary
            return original_import(name, *args, **kwargs)

        monkeypatch.setattr(builtins, '__import__', fail_import)
    with pytest.raises(type(primary)) as caught:
        invoke(frontend, owner, tmp_path)
    assert caught.value is primary
    assert owner.close_calls == 1 and owner.dt_s == .00125
    assert window.exits == ([primary] if stage == 'interrupt' else [])
    if close_fails:
        assert any('close failed' in note and 'owner close failed' in note
                   for note in primary.__notes__)
    if frontend == 'physical':
        assert owner.flush_calls == 1
        assert ('failure' in owner.metadata) is (stage != 'interrupt')
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize('frontend,expected', [('physical', 0), ('replay', 2)])
@pytest.mark.parametrize('close_fails', [False, True])
def test_normal_window_close_keeps_exit_contract_and_reports_cleanup_failure(
        monkeypatch, fake_viewer, tmp_path, frontend, expected, close_fails):
    window = Window()
    fake_viewer.launch_passive = lambda *args, **kwargs: window
    monkeypatch.setattr(physical_frontend, 'FramePresenter', presenter)
    monkeypatch.setattr(replay_viewer, 'FramePresenter', presenter)
    cleanup = RuntimeError('owner close failed') if close_fails else None
    owner = Owner(cleanup)
    if cleanup is None:
        assert invoke(frontend, owner, tmp_path) == expected
    else:
        with pytest.raises(RuntimeError) as caught:
            invoke(frontend, owner, tmp_path)
        assert caught.value is cleanup
    assert owner.close_calls == 1 and window.exits == [None]
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize('stage', ['construction', 'attach'])
@pytest.mark.parametrize('close_fails', [False, True])
def test_research_viewer_setup_failure_closes_environment(monkeypatch, tmp_path, stage, close_fails):
    from bike_sim.sim.research import recording_lifecycle
    primary = RuntimeError(f'research {stage} failed')
    closes, finishes = [], []

    def close(*, discard_pending):
        closes.append(discard_pending)
        if close_fails:
            raise RuntimeError('research close failed')

    env = SimpleNamespace(capture_mode='off', close=close)
    session = SimpleNamespace(env=env)

    class Launch:
        finalization_attempted = False

        def __init__(self, *args, **kwargs):
            if stage == 'construction':
                raise primary

        def attach(self, session):
            raise primary

        def finish(self, *, primary, stop):
            finishes.append((primary, stop))
            self.finalization_attempted = True

    monkeypatch.setattr(recording_lifecycle, 'ResearchRun', Launch)
    with pytest.raises(RuntimeError) as caught:
        research_viewer.run_research_viewer(session, tmp_path, record=False)
    assert caught.value is primary
    assert closes == [True]
    assert finishes == ([(primary, True)] if stage == 'attach' else [])
    if close_fails:
        assert any('research close failed' in note for note in primary.__notes__)
    assert list(tmp_path.iterdir()) == []


def test_research_viewer_normal_close_finishes_once(monkeypatch, fake_viewer, tmp_path):
    from bike_sim.sim.research import recording_lifecycle
    from bike_sim.sim.ride import physical_view, presentation
    window = Window()
    fake_viewer.launch_passive = lambda *args, **kwargs: window
    monkeypatch.setattr(presentation, 'FramePresenter', presenter)
    monkeypatch.setattr(physical_view, 'environment_snapshot', lambda env: object())
    events = []
    env = SimpleNamespace(capture_mode='off', backend='native', dt_s=.00125,
        sim=SimpleNamespace(steps=0, time_s=0.), reason=None, error=None,
        numerically_valid=True, model_valid=True, make_render_model=lambda: object(),
        close=lambda *, discard_pending: events.append(('close', discard_pending)))
    session = SimpleNamespace(env=env)

    class Launch:
        finalization_attempted = False

        def __init__(self, environment, output_dir, *, enabled, mode, capture):
            assert environment is env and output_dir == tmp_path
            assert not enabled and mode == 'research_viewer' and capture is None
            events.append(('construct',))

        def attach(self, attached):
            assert attached is session
            events.append(('attach',))

        def finish(self, *, stop):
            assert not self.finalization_attempted
            self.finalization_attempted = True
            events.append(('finish', stop))
            return None

    monkeypatch.setattr(recording_lifecycle, 'ResearchRun', Launch)
    assert research_viewer.run_research_viewer(session, tmp_path, record=False) == 0
    assert events == [('construct',), ('attach',), ('finish', True), ('close', True)]
    assert window.exits == [None] and env.dt_s == .00125
    assert list(tmp_path.iterdir()) == []
