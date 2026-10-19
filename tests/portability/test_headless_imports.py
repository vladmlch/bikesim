"""T18: fresh imports and CLI parsing do not need a display or a renderer.

MuJoCo/GLFW bindings may be imported. Their rendering entry points are
guarded before application imports, including when a CLI catches an error.
These are no-display checks, not desktop/Xvfb viewer qualification.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
DISPLAY_VARIABLES = ('DISPLAY', 'WAYLAND_DISPLAY', 'MUJOCO_GL')
CLI_MODULES = (
    'bike_sim.cli.ride',
    'bike_sim.cli.research',
    'bike_sim.cli.batch',
    'bike_sim.cli.replay',
    'bike_sim.cli.recording',
)
RESULT_PREFIX = 'HEADLESS_IMPORT_RESULT='

# Install guards before importing application code: patching a parent
# process alone cannot catch rendering initialization in a fresh child.
GUARDED_MODULE = r'''
from contextlib import ExitStack
import importlib
import json
import os
import runpy
import sys
from unittest.mock import patch

display_variables = ('DISPLAY', 'WAYLAND_DISPLAY', 'MUJOCO_GL')
assert all(name not in os.environ for name in display_variables)
module, mode, *arguments = sys.argv[1:]
rendering_calls = []

def forbidden(name):
    def call(*args, **kwargs):
        rendering_calls.append(name)
        raise AssertionError('rendering used in headless mode: ' + name)
    return call

with ExitStack() as guards:
    import glfw
    for name in ('init', 'create_window'):
        guards.enter_context(patch.object(glfw, name, forbidden('glfw.' + name)))
    import mujoco
    guards.enter_context(patch.object(mujoco, 'Renderer', forbidden('mujoco.Renderer')))
    # Importing bindings is allowed; the passive-viewer call is forbidden.
    viewer = importlib.import_module('mujoco.viewer')
    guards.enter_context(patch.object(viewer, 'launch_passive',
                                     forbidden('mujoco.viewer.launch_passive')))
    exit_code = 0
    try:
        if mode == 'import':
            importlib.import_module(module)
        else:
            assert mode == 'cli'
            sys.argv = [module, *arguments]
            try:
                runpy.run_module(module, run_name='__main__')
            except SystemExit as error:
                exit_code = 0 if error.code is None else error.code
                assert type(exit_code) is int, repr(error.code)
    finally:
        # Do not let an application's broad exception handler hide a call.
        assert not rendering_calls, rendering_calls
        assert all(name not in os.environ for name in display_variables)
print('HEADLESS_IMPORT_RESULT=' + json.dumps(
    dict(module=module, mode=mode, exit_code=exit_code, rendering_calls=rendering_calls)))
raise SystemExit(exit_code)
'''


def _run_without_display(module, mode, tmp_path, *arguments):
    environment = dict(os.environ)
    for name in DISPLAY_VARIABLES:
        environment.pop(name, None)
    environment['PYTHONDONTWRITEBYTECODE'] = '1'
    environment['PYTHONPATH'] = os.pathsep.join(filter(None, (
        str(ROOT / 'src'), environment.get('PYTHONPATH'),
    )))
    runtime = environment.get('NATIVE_TEST_SANITIZER_RUNTIME')
    if runtime is not None:
        # dyld may remove the parent's preload variable after startup.
        preload = 'DYLD_INSERT_LIBRARIES' if sys.platform == 'darwin' else 'LD_PRELOAD'
        previous = environment.get(preload, '')
        environment[preload] = os.pathsep.join(filter(None, (runtime, previous)))
    result = subprocess.run(
        [sys.executable, '-c', GUARDED_MODULE, module, mode, *arguments],
        cwd=tmp_path, env=environment, capture_output=True, text=True,
        timeout=30, check=False,
    )
    diagnostic = f'exit={result.returncode}\n{result.stdout}\n{result.stderr}'
    records = [line[len(RESULT_PREFIX):] for line in result.stdout.splitlines()
               if line.startswith(RESULT_PREFIX)]
    assert len(records) == 1, diagnostic
    record = json.loads(records[0])
    assert record == dict(module=module, mode=mode,
                          exit_code=result.returncode, rendering_calls=[]), diagnostic
    return result


@pytest.mark.parametrize('module', CLI_MODULES + (
    'bike_sim.sim.ride.physical_driver',
    'bike_sim.sim.research.configuration',
))
def test_import_without_display(module, tmp_path):
    result = _run_without_display(module, 'import', tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize('module', CLI_MODULES)
def test_cli_help_without_display(module, tmp_path):
    result = _run_without_display(module, 'cli', tmp_path, '--help')
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'usage:' in result.stdout.lower()
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize('scale', ['1', '8'])
def test_explicit_time_scale_requires_a_viewer_even_at_one(scale, tmp_path):
    result = _run_without_display(
        'bike_sim.cli.ride', 'cli', tmp_path,
        '--physics-config', str(ROOT / 'examples/research/viewer_physics_welded.toml'),
        '--headless', '--duration', '0.04', '--no-plots', '--time-scale', scale,
    )
    assert result.returncode == 2, result.stdout + result.stderr
    assert '--time-scale requires an interactive physical or research run' in result.stderr
    assert list(tmp_path.iterdir()) == []
