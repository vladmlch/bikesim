"""Platform preparation for interactive MuJoCo viewers.

Importing this module uses only the standard library. CLI entry points may
relaunch before creating simulation owners; viewers receiving an existing
owner must use ``load_viewer``, which never replaces the process.
"""
from __future__ import annotations

import atexit
from contextlib import contextmanager
import os
from pathlib import Path
import shutil
import sys
import sysconfig
import threading


_REEXEC_PID = 'BIKE_SIM_MJPYTHON_REEXEC_PID'


class ViewerUnavailableError(RuntimeError):
    """The requested interactive viewer cannot use this launch environment."""


def _running_mjpython() -> bool:
    # MuJoCo's launcher installs this bridge before running user code. Inspect
    # the loaded module only: importing it here would load GLFW too early.
    viewer = sys.modules.get('mujoco.viewer')
    bridge_type = getattr(viewer, '_MjPythonBase', None)
    return (isinstance(bridge_type, type)
            and isinstance(getattr(viewer, '_MJPYTHON', None), bridge_type))


def _mjpython_script() -> Path:
    candidates = [Path(sysconfig.get_path('scripts')) / 'mjpython',
                  Path(sys.executable).parent / 'mjpython']
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.R_OK):
            return candidate
    found = shutil.which('mjpython')
    if found is not None:
        return Path(found)
    raise ViewerUnavailableError(
        f'macOS interactive viewers require mjpython from the MuJoCo '
        f'installation for {sys.executable}. No launcher was found in the '
        'selected environment or PATH. Install MuJoCo into that environment '
        'and retry the same command; headless commands do not need mjpython.')


def _interpreter_options(original):
    """Keep interpreter options before the original script/module command."""
    arguments = iter(original[1:])
    options = []
    for argument in arguments:
        if argument in ('-', '--') or not argument.startswith('-'):
            break
        needs_value = argument == '--check-hash-based-pycs'
        if not argument.startswith('--'):
            for index, flag in enumerate(argument[1:], start=1):
                if flag in ('m', 'c'):
                    # Python also accepts combined forms, e.g. '-Im module'.
                    if index > 1:
                        options.append(argument[:index])
                    return options
                if flag in ('W', 'X'):
                    # Everything after W/X belongs to that option, even if
                    # the value contains an 'm' or 'c'.
                    needs_value = index == len(argument) - 1
                    break
        options.append(argument)
        if needs_value:
            value = next(arguments, None)
            if value is None:
                raise ViewerUnavailableError(
                    f'Cannot reconstruct the interpreter option {argument}.')
            options.append(value)
    return options


def _python_arguments(argv, module):
    original = getattr(sys, 'orig_argv', None)
    if argv is not None:
        if module is None:
            raise ViewerUnavailableError(
                'An explicit viewer argument list requires its CLI module name.')
        options = _interpreter_options(original) if original else []
        return [*options, '-m', module, *argv]
    # sys.argv has already lost '-m' and interpreter options. mjpython accepts
    # Python's command line, so retain the original arguments without a shell.
    if original and len(original) > 1:
        return list(original[1:])
    if sys.argv and sys.argv[0] not in ('', '-', '-c'):
        return list(sys.argv)
    raise ViewerUnavailableError(
        'Cannot reconstruct this interactive Python invocation for mjpython. '
        'Start the viewer with mjpython explicitly before constructing a session.')


def ensure_macos_mjpython(*, argv=None, module=None, allow_reexec=True) -> None:
    """Ensure the real macOS UI bridge, preserving the selected interpreter.

    An inherited MUJOCO_MJPYTHON variable or an executable's filename is not
    evidence of that bridge. The PID marker detects a failed re-exec without
    making a newly spawned Python child look like a viewer process.
    """
    if sys.platform != 'darwin':
        return
    if _running_mjpython():
        # mjpython owns the OS UI thread; its Python thread need not be the
        # ordinary interpreter main thread.
        return
    if not allow_reexec:
        raise ViewerUnavailableError(
            'macOS interactive viewers require the mjpython UI bridge. '
            'Call prepare_viewer() before constructing the simulation/session, '
            'or launch the command with mjpython. An existing owner cannot '
            'be safely replaced by an automatic re-exec.')
    if threading.current_thread() is not threading.main_thread():
        raise ViewerUnavailableError(
            'Prepare the macOS viewer from the command entry point before '
            'starting worker threads; an existing worker cannot re-exec the process.')
    if os.environ.get(_REEXEC_PID) == str(os.getpid()):
        raise ViewerUnavailableError(
            'mjpython was launched but did not install the MuJoCo UI bridge. '
            f'Check the MuJoCo installation for {sys.executable}; '
            'automatic relaunch has been stopped to avoid a loop.')

    script = _mjpython_script()
    arguments = _python_arguments(argv, module)
    environment = os.environ.copy()
    environment[_REEXEC_PID] = str(os.getpid())
    lib_dir = Path(sys.base_prefix) / 'lib'
    version = f'{sys.version_info.major}.{sys.version_info.minor}'
    if (lib_dir / f'libpython{version}.dylib').is_file():
        # Keep uv/pyenv's libpython reachable and retain traditional dyld
        # search locations when the user supplied no explicit fallback path.
        fallback = environment.get('DYLD_FALLBACK_LIBRARY_PATH')
        previous = ([str(Path.home() / 'lib'), '/usr/local/lib', '/usr/lib']
                    if fallback is None else fallback.split(':'))
        paths = [str(lib_dir), *(path for path in previous if path)]
        environment['DYLD_FALLBACK_LIBRARY_PATH'] = ':'.join(dict.fromkeys(paths))
    # Run the Python trampoline with the selected interpreter and its options,
    # even for a PATH launcher. Its shebang must not select another venv, and
    # e.g. -I/-S must still govern the trampoline's own MuJoCo module lookup.
    # The trampoline then forwards the complete arguments to the application.
    options = _interpreter_options([sys.executable, *arguments])
    command = [sys.executable, *options, str(script), *arguments]
    print(f'[bike-sim viewer] Relaunching with {script}', file=sys.stderr, flush=True)
    try:
        os.execve(sys.executable, command, environment)
    except OSError as error:
        raise ViewerUnavailableError(
            f'Could not launch {script} with {sys.executable}: {error}') from error
    raise ViewerUnavailableError('mjpython re-exec unexpectedly returned without replacing the process')


def prepare_viewer(*, argv=None, module=None, allow_reexec=True) -> None:
    """Check an interactive launch without importing any graphics modules."""
    if sys.platform == 'darwin':
        ensure_macos_mjpython(argv=argv, module=module, allow_reexec=allow_reexec)
    elif sys.platform.startswith('linux'):
        if not (os.environ.get('DISPLAY', '').strip()
                or os.environ.get('WAYLAND_DISPLAY', '').strip()):
            raise ViewerUnavailableError(
                'No Linux desktop display is available: DISPLAY and WAYLAND_DISPLAY '
                'are unset. Start the viewer in a desktop session. For a run without '
                'a window use bike-ride --headless, or bike-replay without --view/--viewer.')
        if threading.current_thread() is not threading.main_thread():
            raise ViewerUnavailableError(
                'Start the Linux interactive viewer on the main Python thread; '
                'GLFW initialization cannot be delegated to a simulation worker.')


def _initialize_linux_glfw() -> None:
    # MuJoCo initializes GLFW in its passive-viewer thread, then waits on a
    # queue for a handle. An initialization failure there may never reach the
    # caller. Initialize synchronously so that an unavailable display returns
    # an actionable error through the owner's normal cleanup path.
    try:
        import glfw
        ready = glfw.init()
        if not ready:
            code, description = glfw.get_error()
            if isinstance(description, bytes):
                description = description.decode('utf-8', errors='replace')
            raise ViewerUnavailableError(
                f'GLFW could not initialize the Linux desktop display '
                f'(error {code}: {description or "no detail"}). '
                'Check display access and the installed OpenGL/GLFW libraries. '
                'A Wayland session may require XWayland and DISPLAY when the '
                'installed GLFW build does not support native Wayland.')
    except ViewerUnavailableError:
        raise
    except Exception as error:
        raise ViewerUnavailableError(
            f'Cannot initialize GLFW for the Linux viewer: '
            f'{type(error).__name__}: {error}. Check desktop access and the '
            'OpenGL/GLFW installation; headless commands do not need a viewer.') from error
    # Do not terminate a display potentially shared by another open viewer.
    # MuJoCo also registers termination once its own launch succeeds.
    atexit.register(glfw.terminate)


def load_viewer():
    """Load a passive viewer for an already owned session, without re-exec."""
    prepare_viewer(allow_reexec=False)
    try:
        import mujoco.viewer
    except (ImportError, OSError) as error:
        raise ViewerUnavailableError(
            f'Cannot load the interactive MuJoCo viewer: '
            f'{type(error).__name__}: {error}. Check MuJoCo, GLFW and the '
            'desktop OpenGL libraries in the selected environment.') from error
    if sys.platform.startswith('linux'):
        _initialize_linux_glfw()
    return mujoco.viewer


@contextmanager
def passive_viewer(viewer_api, *args, **kwargs):
    """Preserve a render/interrupt failure if closing the window also fails."""
    handle = viewer_api.launch_passive(*args, **kwargs)
    viewer = handle.__enter__()
    try:
        yield viewer
    except BaseException as primary:
        try:
            suppressed = handle.__exit__(type(primary), primary, primary.__traceback__)
        except BaseException as cleanup:
            primary.add_note(f'viewer window close failed: {type(cleanup).__name__}: {cleanup}')
            suppressed = False
        if not suppressed:
            raise
    else:
        # On a normal close, a cleanup error is the primary error and must
        # remain visible rather than turning the command into a success.
        handle.__exit__(None, None, None)
