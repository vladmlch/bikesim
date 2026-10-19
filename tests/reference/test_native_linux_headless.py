"""T18: real display-free stepping, CLI outcomes and non-rendering cleanup.

Despite the filename, these controls apply to Linux and macOS. Native
consumers use native_loader's selected-artifact/sanitizer checks; a missing
build fails the gate. No test builds an extension or opens a GUI.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[2]
DISPLAY_VARIABLES = ('DISPLAY', 'WAYLAND_DISPLAY', 'MUJOCO_GL')
RESULT_PREFIX = 'HEADLESS_RIDE_RESULT='


@pytest.fixture
def rendering_forbidden(monkeypatch):
    for name in DISPLAY_VARIABLES:
        monkeypatch.delenv(name, raising=False)
    calls = []

    def forbidden(name):
        def call(*args, **kwargs):
            calls.append(name)
            raise AssertionError(f'rendering used in headless mode: {name}')
        return call

    import glfw
    monkeypatch.setattr(glfw, 'init', forbidden('glfw.init'))
    monkeypatch.setattr(glfw, 'create_window', forbidden('glfw.create_window'))
    import mujoco
    monkeypatch.setattr(mujoco, 'Renderer', forbidden('mujoco.Renderer'))
    import mujoco.viewer
    monkeypatch.setattr(mujoco.viewer, 'launch_passive',
                        forbidden('mujoco.viewer.launch_passive'))
    yield
    assert not calls, calls
    assert all(name not in os.environ for name in DISPLAY_VARIABLES)


def _selected_build():
    from native_loader import selected_build
    build = selected_build(os.environ)
    application = os.environ.get('BIKE_NATIVE_BUILD_PATH')
    if application is not None and Path(application).resolve() != build:
        pytest.fail('test and application native artifact selectors disagree')
    return build


@pytest.mark.parametrize('backend', ['python', 'native'])
def test_four_step_driver_without_rendering(
        rendering_forbidden, environment_factory, monkeypatch, tmp_path, backend):
    from bike_sim.sim.ride.control import RideControl
    from bike_sim.sim.ride.physical_driver import PhysicalRideDriver

    if backend == 'native':
        from native_loader import load_native
        monkeypatch.setenv('BIKE_NATIVE_BUILD_PATH', str(_selected_build()))
        load_native()
    monkeypatch.chdir(tmp_path)
    environment = environment_factory(capture_mode='off')
    driver = None
    try:
        driver = PhysicalRideDriver(
            environment.sim, backend=backend, duration_s=.005,
            decimation=2, record=False,
        )
        result = driver.advance(100, RideControl(0.))
        assert result.step == 4
        assert result.outcome == 'duration_reached'
        snapshot = driver.snapshot()
        assert snapshot.step == 4
        assert snapshot.integration_state.size > 0
        assert driver.recorder.rows == 0
        driver.flush()
        assert driver.recorder.rows == 0
    finally:
        if driver is not None:
            driver.close()
        # Exercise fixture cleanup while the same rendering guards are active.
        environment.close()
    assert list(tmp_path.iterdir()) == []


# This is the real module entry point and real native driver. The summary
# spy only observes the result returned to the CLI; it neither replaces
# stepping nor changes the outcome or enables recording for test evidence.
CLI_PROBE = r'''
from contextlib import ExitStack
import importlib
import json
import os
from pathlib import Path
import runpy
import sys
from unittest.mock import patch

display_variables = ('DISPLAY', 'WAYLAND_DISPLAY', 'MUJOCO_GL')
assert all(name not in os.environ for name in display_variables)
arguments = sys.argv[1:]
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
    viewer = importlib.import_module('mujoco.viewer')
    guards.enter_context(patch.object(viewer, 'launch_passive',
                                     forbidden('mujoco.viewer.launch_passive')))
    from native_loader import load_native, selected_build
    build = selected_build(os.environ)
    assert Path(os.environ['BIKE_NATIVE_BUILD_PATH']).resolve() == build
    extension = load_native()
    from bike_sim.sim.ride.physical_driver import PhysicalRideDriver
    original_summary = PhysicalRideDriver.summary
    summaries = []

    def observe_summary(self, *args, **kwargs):
        summary = original_summary(self, *args, **kwargs)
        summaries.append(summary)
        return summary

    guards.enter_context(patch.object(PhysicalRideDriver, 'summary', observe_summary))
    sys.argv = ['bike-ride', *arguments]
    try:
        runpy.run_module('bike_sim.cli.ride', run_name='__main__')
    except SystemExit as error:
        exit_code = 0 if error.code is None else error.code
        assert type(exit_code) is int, repr(error.code)
    else:
        raise AssertionError('bike-ride module did not exit through its CLI entry point')
    assert not rendering_calls, rendering_calls
    assert all(name not in os.environ for name in display_variables)
    assert len(summaries) == 1, len(summaries)
    summary = summaries[0]
    evidence = dict(exit_code=exit_code, selected_build=str(build),
                    imported_extension=str(Path(extension.__file__).resolve()),
                    outcome=summary['outcome'], first_failure=summary['first_failure'],
                    model_status=summary['model_status'], backend=summary['backend'],
                    rendering_calls=rendering_calls)
print('HEADLESS_RIDE_RESULT=' + json.dumps(evidence, allow_nan=False, sort_keys=True))
raise SystemExit(exit_code)
'''


@pytest.mark.parametrize('policy,duration,expected_reason', [
    (None, '0.04', 'invalid_controller'),
    ('diagnostic', '0.01', 'duration_reached'),
], ids=['ordinary-strict-0.04', 'diagnostic-baseline-0.01'])
def test_native_cli_without_display_retains_real_outcome(
        tmp_path, record_property, policy, duration, expected_reason):
    """The rough-track model violation must not masquerade as a GL failure.

    The .01 diagnostic source baseline is in
    test_native_recording_lifecycle.py::test_headless_native_run_records_one_episode:
    duration_reached, exit 2, sticky energy.constraint_work. The ordinary
    .04 command keeps the strict policy and must stop for that violation.
    These are assertions to qualify on the target host, not claimed runs.
    """
    build = _selected_build()
    environment = dict(os.environ)
    for name in DISPLAY_VARIABLES:
        environment.pop(name, None)
    environment['BIKE_NATIVE_BUILD_PATH'] = str(build)
    environment['NATIVE_TEST_BUILD_PATH'] = str(build)
    environment['PYTHONDONTWRITEBYTECODE'] = '1'
    environment['PYTHONPATH'] = os.pathsep.join(filter(None, (
        str(ROOT / 'src'), str(ROOT / 'tests/reference'),
        environment.get('PYTHONPATH'),
    )))
    # The child verifies its artifact and sanitizer itself. Its probe record
    # must not overwrite the outer pytest collection/provenance record.
    environment.pop('NATIVE_TEST_PROVENANCE_PATH', None)
    runtime = environment.get('NATIVE_TEST_SANITIZER_RUNTIME')
    if runtime is not None:
        # Direct interpreter launch: re-inject a runtime that dyld may have
        # removed from the parent's environment, before native_loader checks it.
        preload = 'DYLD_INSERT_LIBRARIES' if sys.platform == 'darwin' else 'LD_PRELOAD'
        previous = environment.get(preload, '')
        environment[preload] = os.pathsep.join(filter(None, (runtime, previous)))
    output = tmp_path / 'output'
    arguments = [
        '--backend', 'native',
        '--physics-config', str(ROOT / 'examples/research/viewer_physics_welded.toml'),
        '--track', str(ROOT / 'examples/research/rough_uphill_savage.toml'),
        '--headless', '--duration', duration, '--no-plots', '--out', str(output),
    ]
    if policy is not None:
        arguments += ['--check-policy', policy]
    result = subprocess.run(
        [sys.executable, '-c', CLI_PROBE, *arguments],
        cwd=tmp_path, env=environment, capture_output=True, text=True,
        timeout=180, check=False,
    )
    diagnostic = f'exit={result.returncode}\n{result.stdout}\n{result.stderr}'
    # Keep actual evidence even if a subsequent baseline assertion fails.
    record_property('headless_cli_exit_code', result.returncode)
    record_property('headless_cli_stdout', result.stdout)
    record_property('headless_cli_stderr', result.stderr)
    records = [line[len(RESULT_PREFIX):] for line in result.stdout.splitlines()
               if line.startswith(RESULT_PREFIX)]
    assert len(records) == 1, diagnostic
    evidence = json.loads(records[0])
    record_property('headless_cli_evidence', json.dumps(evidence, sort_keys=True))
    assert result.returncode == evidence['exit_code'] == 2, diagnostic
    assert evidence['selected_build'] == str(build), diagnostic
    assert Path(evidence['imported_extension']).parent == build, diagnostic
    assert evidence['backend'] == 'native', diagnostic
    assert evidence['rendering_calls'] == [], diagnostic
    assert evidence['outcome']['reason'] == expected_reason, diagnostic
    assert evidence['outcome']['steps'] > 0, diagnostic
    assert evidence['model_status']['model_valid'] is False, diagnostic
    failure_time, violations = evidence['first_failure']
    assert math.isfinite(failure_time), diagnostic
    assert 0 <= failure_time <= evidence['outcome']['time_s'], diagnostic
    assert 'energy.constraint_work' in violations, diagnostic
    if policy is None:
        assert 0 < evidence['outcome']['time_s'] < float(duration), diagnostic
    else:
        assert evidence['outcome']['time_s'] == pytest.approx(float(duration)), diagnostic
    assert 'recording off (no files)' in result.stdout, diagnostic
    assert not output.exists(), diagnostic
    assert list(tmp_path.iterdir()) == [], diagnostic


class _HeadlessDriver:
    """Only the frontend cleanup seam is doubled; no physics is claimed."""
    strict = False
    max_steps = 4

    def __init__(self, backend, error_type):
        self.backend = backend
        self.error_type = error_type
        self.metadata = {'drive_mode': 'articulated_effort',
                         'configuration_sha256': '0' * 64}
        self.step, self.time_s, self.reason = 0, 0., None
        self.events = []

    def start(self):
        self.events.append('start')

    def advance(self, target, **kwargs):
        self.events.append('advance')
        if self.error_type is not None:
            raise self.error_type('injected runtime stop')
        self.step, self.time_s, self.reason = 4, .005, 'duration_reached'

    def flush(self):
        self.events.append('flush')

    def summary(self, reason=None):
        self.events.append('summary')
        self.final_reason = reason
        return {'outcome': {'reason': reason, 'time_s': self.time_s, 'position_m': 0.},
                'model_status': {'model_valid': True}, 'first_failure': None}

    def close(self):
        self.events.append('close')


@pytest.mark.parametrize('backend', ['python', 'native'])
@pytest.mark.parametrize('error_type,reason,exit_code', [
    (None, 'duration_reached', 0),
    (RuntimeError, 'simulation_error', 1),
    (KeyboardInterrupt, 'operator_stop', 130),
])
def test_headless_frontend_closes_without_initializing_rendering(
        rendering_forbidden, monkeypatch, tmp_path, backend, error_type, reason, exit_code):
    from bike_sim.sim.ride import physical_frontend
    driver = _HeadlessDriver(backend, error_type)
    monkeypatch.setattr(physical_frontend, 'build_physical_driver',
                        lambda *args, **kwargs: driver)
    args = SimpleNamespace(backend=backend, record=False, out=tmp_path,
                           no_plots=True, resolved_check_policy='diagnostic')
    code = physical_frontend.run_headless(SimpleNamespace(name='fixture'), args, 0, None)
    assert code == exit_code
    assert driver.final_reason == reason
    assert driver.events == ['start', 'advance', 'flush', 'summary', 'close']
    if error_type is RuntimeError:
        assert driver.metadata['failure'] == 'RuntimeError: injected runtime stop'
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize('error_type', [ValueError, KeyboardInterrupt])
def test_failed_native_model_construction_cleans_bootstrap_without_rendering(
        rendering_forbidden, monkeypatch, tmp_path, error_type):
    """Inject failure at native model construction and check owned temp files.

    The fake extension here is a cleanup control, not an artifact/load test.
    Real native imports are exercised by the two integration cases above.
    """
    from bike_sim.native import artifact, runtime
    from bike_sim.sim.ride import physical_view
    original_temporary_directory = runtime.tempfile.TemporaryDirectory
    directories = []

    def temporary_directory(*args, **kwargs):
        owner = original_temporary_directory(*args, dir=tmp_path, **kwargs)
        directories.append(Path(owner.name))
        return owner

    def capture_bootstrap(sim, directory):
        model = directory / 'model.xml'
        model.write_text('<mujoco/>', encoding='utf-8')
        return SimpleNamespace(model_path=model, config={}, state=None)

    def construct_model(*args, **kwargs):
        raise error_type('injected model construction failure')

    monkeypatch.setattr(runtime.tempfile, 'TemporaryDirectory', temporary_directory)
    monkeypatch.setattr(physical_view, 'snapshot_python', lambda sim: object())
    monkeypatch.setattr(runtime, 'capture_bootstrap', capture_bootstrap)
    monkeypatch.setattr(artifact, 'load_native_extension',
                        lambda: SimpleNamespace(NativeRideRuntime=construct_model))
    with pytest.raises(error_type, match='injected model construction failure'):
        runtime.create_native_ride(SimpleNamespace(physical=object()))
    assert len(directories) == 1
    assert not directories[0].exists()
    assert list(tmp_path.iterdir()) == []
