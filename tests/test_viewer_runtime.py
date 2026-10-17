"""Viewer launch contracts with fake interpreters and graphics modules only.

These tests do not open a window, create a simulation, execute a launcher or
require importing the bike_sim package and its physics dependencies.
"""
import importlib.util
from io import StringIO
import os
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest


SOURCE = Path(__file__).resolve().parents[1] / 'src/bike_sim/viewer_runtime.py'


class Reexecuted(BaseException):
    """A fake successful execve never returns to its caller."""


@pytest.fixture
def runtime(tmp_path):
    spec = importlib.util.spec_from_file_location('_viewer_runtime_contract', SOURCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    executable = str(tmp_path / 'venv/bin/python')
    module.sys = SimpleNamespace(
        platform='linux', modules={}, executable=executable,
        base_prefix=str(tmp_path / 'base'), version_info=sys.version_info,
        orig_argv=[executable, '-m', 'bike_sim.cli.ride', '--track', 'trail with spaces.toml'],
        argv=['ride.py', '--track', 'trail with spaces.toml'], stderr=StringIO())
    def forbidden_exec(*args):
        raise AssertionError('unexpected process replacement')
    module.os = SimpleNamespace(
        environ={}, getpid=lambda: 4321, execve=forbidden_exec,
        access=lambda path, mode: True, R_OK=os.R_OK)
    module.sysconfig = SimpleNamespace(get_path=lambda key: str(tmp_path / 'venv/bin'))
    module.shutil = SimpleNamespace(which=lambda command: None)
    main = object()
    module.threading = SimpleNamespace(current_thread=lambda: main, main_thread=lambda: main)
    module.atexit = SimpleNamespace(register=lambda callback: None)
    return module


def launcher(runtime, tmp_path):
    script = tmp_path / 'venv/bin/mjpython'
    script.parent.mkdir(parents=True, exist_ok=True)
    script.write_text('#!/usr/bin/env python\n', encoding='utf-8')
    runtime.sys.platform = 'darwin'
    captured = {}
    def replace(path, arguments, environment):
        captured.update(path=path, arguments=arguments, environment=environment)
        raise Reexecuted()
    runtime.os.execve = replace
    return script, captured


def fake_graphics(monkeypatch, *, init=True):
    events = []
    viewer = ModuleType('mujoco.viewer')
    viewer.launch_passive = lambda *args, **kwargs: events.append('launch')
    mujoco = ModuleType('mujoco')
    mujoco.__path__ = []
    mujoco.viewer = viewer
    glfw = ModuleType('glfw')
    glfw.init = lambda: events.append('init') or init
    glfw.get_error = lambda: (65544, b'X11: failed to open display')
    glfw.terminate = lambda: events.append('terminate')
    monkeypatch.setitem(sys.modules, 'mujoco', mujoco)
    monkeypatch.setitem(sys.modules, 'mujoco.viewer', viewer)
    monkeypatch.setitem(sys.modules, 'glfw', glfw)
    return viewer, glfw, events


def test_linux_never_looks_up_or_executes_mjpython(runtime):
    runtime.os.environ.update(MUJOCO_MJPYTHON='1', DYLD_FALLBACK_LIBRARY_PATH='/keep')
    before = dict(runtime.os.environ)
    def forbidden():
        raise AssertionError('Linux looked up a macOS launcher')
    runtime._mjpython_script = forbidden
    runtime.ensure_macos_mjpython()
    assert runtime.os.environ == before


def test_linux_no_display_fails_without_graphics_import(runtime, monkeypatch):
    import builtins
    original = builtins.__import__
    def guarded(name, *args, **kwargs):
        if name in ('glfw', 'mujoco.viewer'):
            raise AssertionError('graphics imported before the display check')
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', guarded)
    with pytest.raises(runtime.ViewerUnavailableError, match='DISPLAY and WAYLAND_DISPLAY'):
        runtime.load_viewer()


@pytest.mark.parametrize('display', ['DISPLAY', 'WAYLAND_DISPLAY'])
def test_linux_display_presence_defers_backend_support_to_glfw(runtime, display):
    runtime.os.environ[display] = ':0' if display == 'DISPLAY' else 'wayland-0'
    runtime.prepare_viewer()


def test_linux_worker_thread_is_not_a_viewer_launcher(runtime):
    runtime.os.environ['DISPLAY'] = ':0'
    runtime.threading.current_thread = lambda: object()
    with pytest.raises(runtime.ViewerUnavailableError, match='main Python thread'):
        runtime.prepare_viewer()


@pytest.mark.parametrize('original, trampoline_options', [
    (['-m', 'bike_sim.cli.ride', '--track', 'trail with spaces.toml'], []),
    (['-X', 'dev', '-W', 'error', '/venv/bin/bike-ride', '--record'],
     ['-X', 'dev', '-W', 'error']),
])
def test_macos_reexec_preserves_original_invocation(runtime, tmp_path, original, trampoline_options):
    script, captured = launcher(runtime, tmp_path)
    runtime.sys.orig_argv = [runtime.sys.executable, *original]
    runtime.os.environ.update(
        BIKE_NATIVE_BUILD_PATH='/selected/native-build',
        NATIVE_TEST_BUILD_PATH='/selected/native-build', PYTHONPATH='/selected/source')
    before = dict(runtime.os.environ)
    directory = Path.cwd()
    with pytest.raises(Reexecuted):
        runtime.prepare_viewer()
    assert captured['path'] == runtime.sys.executable
    assert captured['arguments'] == [runtime.sys.executable, *trampoline_options, str(script), *original]
    assert captured['environment']['BIKE_SIM_MJPYTHON_REEXEC_PID'] == '4321'
    assert captured['environment'] == {**before, 'BIKE_SIM_MJPYTHON_REEXEC_PID': '4321'}
    assert runtime.os.environ == before
    assert Path.cwd() == directory


def test_explicit_cli_arguments_keep_interpreter_options(runtime, tmp_path):
    script, captured = launcher(runtime, tmp_path)
    runtime.sys.orig_argv = [runtime.sys.executable, '-I', '-X', 'dev', '-Werror',
                            '--check-hash-based-pycs', 'always', '-m', 'pytest', '-q']
    with pytest.raises(Reexecuted):
        runtime.prepare_viewer(argv=['recording with spaces', '--viewer'],
                               module='bike_sim.cli.replay')
    assert captured['arguments'] == [runtime.sys.executable,
        '-I', '-X', 'dev', '-Werror', '--check-hash-based-pycs', 'always', str(script),
        '-I', '-X', 'dev', '-Werror', '--check-hash-based-pycs', 'always',
        '-m', 'bike_sim.cli.replay', 'recording with spaces', '--viewer']


@pytest.mark.parametrize('prefix, expected', [
    (['-Im', 'pytest'], ['-I']),
    (['-Bic', 'ignored code'], ['-Bi']),
    (['-IWerror', '-X', 'dev', '-m', 'pytest'], ['-IWerror', '-X', 'dev']),
    (['-Ximporttime', '-m', 'pytest'], ['-Ximporttime']),
])
def test_explicit_cli_arguments_handle_combined_python_options(runtime, prefix, expected):
    runtime.sys.orig_argv = [runtime.sys.executable, *prefix]
    assert runtime._python_arguments(['--viewer'], 'bike_sim.cli.replay') == [
        *expected, '-m', 'bike_sim.cli.replay', '--viewer']


def test_same_environment_launcher_wins_over_path(runtime, tmp_path):
    script, captured = launcher(runtime, tmp_path)
    def forbidden(command):
        raise AssertionError('PATH consulted despite a selected-environment launcher')
    runtime.shutil.which = forbidden
    with pytest.raises(Reexecuted):
        runtime.ensure_macos_mjpython()
    assert captured['arguments'][1] == str(script)


def test_path_launcher_uses_selected_interpreter_instead_of_its_shebang(runtime, tmp_path):
    script, captured = launcher(runtime, tmp_path)
    script.unlink()
    path_script = tmp_path / 'foreign/mjpython'
    path_script.parent.mkdir()
    path_script.write_text('#!/foreign/python\n', encoding='utf-8')
    runtime.shutil.which = lambda command: str(path_script)
    with pytest.raises(Reexecuted):
        runtime.ensure_macos_mjpython()
    assert captured['arguments'][:2] == [runtime.sys.executable, str(path_script)]


def test_missing_macos_launcher_fails_without_poisoning_later_attempt(runtime):
    runtime.sys.platform = 'darwin'
    runtime.sys._mjpython_checked = True  # The old mutable marker is not proof.
    runtime.os.environ['MUJOCO_MJPYTHON'] = '1'
    for attempt in range(2):
        with pytest.raises(runtime.ViewerUnavailableError, match='No launcher was found'):
            runtime.ensure_macos_mjpython()
    assert 'BIKE_SIM_MJPYTHON_REEXEC_PID' not in runtime.os.environ


def test_existing_mjpython_bridge_is_accepted_on_its_python_thread(runtime):
    runtime.sys.platform = 'darwin'
    class Bridge:
        pass
    runtime.sys.modules['mujoco.viewer'] = SimpleNamespace(_MjPythonBase=Bridge, _MJPYTHON=Bridge())
    runtime.threading.current_thread = lambda: object()
    runtime.prepare_viewer(allow_reexec=False)


def test_inherited_mjpython_marker_does_not_qualify_an_owned_session(runtime):
    runtime.sys.platform = 'darwin'
    runtime.os.environ['MUJOCO_MJPYTHON'] = '1'
    runtime.sys.executable = '/foreign/mjpython-looking-python'
    with pytest.raises(runtime.ViewerUnavailableError, match='existing owner'):
        runtime.prepare_viewer(allow_reexec=False)


def test_failed_reexec_stops_in_same_pid_but_not_an_inherited_child(runtime, tmp_path):
    script, captured = launcher(runtime, tmp_path)
    runtime.os.environ['BIKE_SIM_MJPYTHON_REEXEC_PID'] = '4321'
    with pytest.raises(runtime.ViewerUnavailableError, match='avoid a loop'):
        runtime.ensure_macos_mjpython()
    runtime.os.getpid = lambda: 5432
    with pytest.raises(Reexecuted):
        runtime.ensure_macos_mjpython()
    assert captured['environment']['BIKE_SIM_MJPYTHON_REEXEC_PID'] == '5432'


def test_exec_error_preserves_environment_and_reports_launcher(runtime, tmp_path):
    script, captured = launcher(runtime, tmp_path)
    runtime.os.environ['KEEP'] = 'value'
    def fail(*args):
        raise OSError('permission denied')
    runtime.os.execve = fail
    with pytest.raises(runtime.ViewerUnavailableError, match='permission denied') as failure:
        runtime.ensure_macos_mjpython()
    assert str(script) in str(failure.value)
    assert isinstance(failure.value.__cause__, OSError)
    assert runtime.os.environ == {'KEEP': 'value'}


@pytest.mark.parametrize('fallback', [None, '/custom:/custom:/another'])
def test_uv_libpython_fallback_preserves_search_locations(runtime, tmp_path, fallback):
    script, captured = launcher(runtime, tmp_path)
    directory = Path(runtime.sys.base_prefix) / 'lib'
    directory.mkdir(parents=True)
    version = f'{sys.version_info.major}.{sys.version_info.minor}'
    (directory / f'libpython{version}.dylib').write_bytes(b'not executed')
    if fallback is not None:
        runtime.os.environ['DYLD_FALLBACK_LIBRARY_PATH'] = fallback
    with pytest.raises(Reexecuted):
        runtime.ensure_macos_mjpython()
    paths = captured['environment']['DYLD_FALLBACK_LIBRARY_PATH'].split(':')
    expected = ([str(Path.home() / 'lib'), '/usr/local/lib', '/usr/lib']
                if fallback is None else ['/custom', '/another'])
    assert paths == [str(directory), *expected]


def test_linux_glfw_init_failure_is_synchronous_and_describes_wayland_limit(runtime, monkeypatch):
    viewer, glfw, events = fake_graphics(monkeypatch, init=False)
    runtime.os.environ['WAYLAND_DISPLAY'] = 'wayland-0'
    with pytest.raises(runtime.ViewerUnavailableError, match='XWayland') as failure:
        runtime.load_viewer()
    assert 'X11: failed to open display' in str(failure.value)
    assert events == ['init']


def test_linux_glfw_exception_remains_cause(runtime, monkeypatch):
    viewer, glfw, events = fake_graphics(monkeypatch)
    runtime.os.environ['DISPLAY'] = ':0'
    def fail():
        raise OSError('libGL unavailable')
    glfw.init = fail
    with pytest.raises(runtime.ViewerUnavailableError, match='libGL unavailable') as failure:
        runtime.load_viewer()
    assert isinstance(failure.value.__cause__, OSError)


def test_linux_glfw_is_initialized_before_launch_without_early_termination(runtime, monkeypatch):
    viewer, glfw, events = fake_graphics(monkeypatch)
    runtime.os.environ['DISPLAY'] = ':0'
    cleanup = []
    runtime.atexit.register = cleanup.append
    selected = runtime.load_viewer()
    assert selected is viewer
    selected.launch_passive(object(), object())
    assert events == ['init', 'launch']
    assert cleanup == [glfw.terminate]


@pytest.mark.parametrize('failure_type', [RuntimeError, KeyboardInterrupt])
def test_window_close_failure_never_masks_render_failure_or_interrupt(runtime, failure_type):
    primary = failure_type('render failed')
    received = []
    class Window:
        def __enter__(self):
            return self
        def __exit__(self, kind, error, traceback):
            received.append(error)
            raise OSError('window close failed')
    viewer = SimpleNamespace(launch_passive=lambda *args, **kwargs: Window())
    with pytest.raises(failure_type) as failure:
        with runtime.passive_viewer(viewer, object(), object()):
            raise primary
    assert failure.value is primary
    assert received == [primary]
    assert any('window close failed' in note for note in primary.__notes__)


def test_ordinary_window_close_error_remains_visible(runtime):
    class Window:
        def __enter__(self):
            return self
        def __exit__(self, *args):
            raise OSError('ordinary close failed')
    viewer = SimpleNamespace(launch_passive=lambda *args, **kwargs: Window())
    with pytest.raises(OSError, match='ordinary close failed'):
        with runtime.passive_viewer(viewer, object(), object()):
            pass


def test_window_construction_error_remains_unchanged(runtime):
    primary = ValueError('invalid render model')
    def fail(*args, **kwargs):
        raise primary
    viewer = SimpleNamespace(launch_passive=fail)
    with pytest.raises(ValueError) as failure:
        with runtime.passive_viewer(viewer, object(), object()):
            raise AssertionError('window body ran after construction failed')
    assert failure.value is primary
