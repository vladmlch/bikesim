"""Platform policy controls; the subprocess cases exercise real worker startup."""
import json
import multiprocessing as mp
import os
from pathlib import Path
import re
import subprocess
import sys

import pytest

from bike_sim.cli import batch
from bike_sim.sim.ride import async_close


ROOT = Path(__file__).resolve().parents[2]
PROBE = Path(__file__).with_name('worker_entry_probe.py')


def test_linux_cpu_count_does_not_query_apple(monkeypatch):
    monkeypatch.setattr(batch.sys, 'platform', 'linux')
    monkeypatch.setattr(batch.os, 'process_cpu_count', lambda: 3)
    monkeypatch.setattr(batch.os, 'cpu_count', lambda: 64)

    def forbidden(*args, **kwargs):
        raise AssertionError('Apple sysctl queried on Linux')

    monkeypatch.setattr(batch.subprocess, 'run', forbidden)
    assert batch._performance_cores() == 3


@pytest.mark.parametrize('available,total,expected', [(4, 64, 4), (None, 8, 8),
                                                     (None, None, 1)])
def test_cpu_fallback_respects_process_availability(monkeypatch, available, total, expected):
    monkeypatch.setattr(batch.sys, 'platform', 'linux')
    monkeypatch.setattr(batch.os, 'process_cpu_count', lambda: available)
    monkeypatch.setattr(batch.os, 'cpu_count', lambda: total)
    assert batch._performance_cores() == expected


@pytest.mark.parametrize('stdout,returncode,expected', [('8\n', 0, 8), ('0', 0, 1),
                                                       ('unknown', 0, 3), ('8', 1, 3)])
def test_darwin_performance_core_query(monkeypatch, stdout, returncode, expected):
    monkeypatch.setattr(batch.sys, 'platform', 'darwin')
    monkeypatch.setattr(batch.os, 'process_cpu_count', lambda: 3)

    def query(argv, *, capture_output, text, timeout):
        assert argv == ['sysctl', '-n', 'hw.perflevel0.physicalcpu']
        assert capture_output and text and timeout == 5
        return subprocess.CompletedProcess(argv, returncode, stdout)

    monkeypatch.setattr(batch.subprocess, 'run', query)
    assert batch._performance_cores() == expected


@pytest.mark.parametrize('failure', [FileNotFoundError('sysctl'),
                                    subprocess.TimeoutExpired('sysctl', 5)])
def test_darwin_query_failure_uses_affinity_fallback(monkeypatch, failure):
    monkeypatch.setattr(batch.sys, 'platform', 'darwin')
    monkeypatch.setattr(batch.os, 'process_cpu_count', lambda: 2)

    def query(*args, **kwargs):
        raise failure

    monkeypatch.setattr(batch.subprocess, 'run', query)
    assert batch._performance_cores() == 2


def test_linux_spawn_and_retained_darwin_policy():
    assert async_close._worker_start_method('Linux') == 'spawn'
    assert async_close._worker_start_method('Darwin') == 'fork'


@pytest.mark.parametrize('method', ['not-a-method', '', 7])
def test_invalid_explicit_start_method_creates_no_resources(monkeypatch, method):
    def forbidden(*args, **kwargs):
        raise AssertionError('context allocated before validating start method')

    monkeypatch.setattr(mp, 'get_context', forbidden)
    with pytest.raises(ValueError, match='start_method'):
        async_close.AsyncCloseRunner(start_method=method)


def test_missing_spawn_does_not_fall_back_to_fork(monkeypatch):
    monkeypatch.setattr(async_close.platform, 'system', lambda: 'Linux')
    monkeypatch.setattr(mp, 'get_all_start_methods', lambda: ['fork'])
    with pytest.raises(ValueError, match='spawn'):
        async_close.AsyncCloseRunner()


@pytest.mark.parametrize('interrupt,expected', [(False, 0), (True, 130)])
def test_threaded_parent_worker_probe(interrupt, expected):
    env = dict(os.environ)
    for name in ('DISPLAY', 'WAYLAND_DISPLAY', 'MUJOCO_GL'):
        env.pop(name, None)
    command = [sys.executable, str(PROBE), '--start-method', 'spawn']
    if interrupt:
        command.append('--interrupt')
    result = subprocess.run(command, cwd=ROOT, env=env, capture_output=True,
                            text=True, timeout=60, check=False)
    assert result.returncode == expected, result.stdout + result.stderr
    assert 'resource_tracker:' not in result.stderr
    report = json.loads(result.stdout.strip().splitlines()[-1])
    assert report['start_method'] == 'spawn'
    assert report['live_children'] == []
    assert report['blas_thread_started'] is True
    assert report['interrupted'] is interrupt


@pytest.mark.skipif(sys.platform != 'darwin', reason='real macOS mjpython lane')
def test_mjpython_worker_probe():
    import shutil
    launcher = shutil.which('mjpython')
    assert launcher is not None, 'the macOS native environment must provide mjpython'
    environment = dict(os.environ)
    lib_dir = os.path.join(sys.base_prefix, 'lib')
    version = f'{sys.version_info.major}.{sys.version_info.minor}'
    if os.path.exists(os.path.join(lib_dir, f'libpython{version}.dylib')):
        fallback = environment.get('DYLD_FALLBACK_LIBRARY_PATH')
        environment['DYLD_FALLBACK_LIBRARY_PATH'] = (
            lib_dir if not fallback else f'{lib_dir}:{fallback}')
    result = subprocess.run([launcher, str(PROBE)], cwd=ROOT, env=environment,
                            capture_output=True, text=True, timeout=60, check=False)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'resource_tracker:' not in result.stderr
    report = json.loads(result.stdout.strip().splitlines()[-1])
    assert report['start_method'] == 'fork'
    assert report['live_children'] == []


def test_ride_research_forwards_explicit_async_option(monkeypatch):
    from types import SimpleNamespace
    from bike_sim.cli import ride
    from bike_sim.sim.research import viewer
    args = ride.parse_args([
        '--physics-config', str(ROOT / 'examples/research/viewer_physics_welded.toml'),
        '--research', '--headless', '--async-telemetry', '--duration', '0.01'])
    captured = {}
    environment = object()

    def build(**kwargs):
        captured.update(kwargs)
        return environment

    monkeypatch.setattr(viewer, 'build_environment', build)
    monkeypatch.setattr(viewer, 'load_policy', lambda reference: object())
    monkeypatch.setattr(viewer, 'PolicySession',
                        lambda env, policy, **kwargs: SimpleNamespace(env=env))
    assert viewer.make_ride_session(None, args, None).env is environment
    assert captured['experiment'].async_telemetry is True


@pytest.mark.slow
@pytest.mark.parametrize('application', ['research', 'ride'])
@pytest.mark.parametrize('entry', ['module', 'uv-console'])
def test_real_cli_async_entry_points(tmp_path, application, entry):
    """No wrapper mocks: compare completed sync/async entry-point outcomes.

    Diagnostic model violations remain exit 2; setup/worker failures, short
    execution and silently disabled async workers do not satisfy this gate.
    """
    import shutil
    profile = ROOT / 'examples/research/viewer_physics_welded.toml'
    track = tmp_path / 'flat.toml'
    track.write_text('name = "worker-entry"\nlength_m = 20.0\nsurface = "hardpack"\n'
                     'grade_profile = { knots = [[0.0, 0.0], [20.0, 0.0]] }\n')
    common = ['--physics-config', str(profile), '--duration', '0.01',
              '--check-policy', 'diagnostic']
    if application == 'research':
        common += ['--track-file', str(track), '--motor-torque', '0', '--human-torque', '0']
    else:
        common += ['--track', str(track), '--research', '--headless', '--no-plots', '--demand', '0']
    environment = dict(os.environ)
    for name in ('DISPLAY', 'WAYLAND_DISPLAY', 'MUJOCO_GL'):
        environment.pop(name, None)
    environment['PYTHONPATH'] = os.pathsep.join(filter(None, (
        str(ROOT / 'src'), environment.get('PYTHONPATH'))))
    if entry == 'module':
        prefix = [sys.executable, '-m', f'bike_sim.cli.{application}']
    else:
        uv = shutil.which('uv')
        assert uv is not None, 'entry-point qualification requires uv'
        # Preserve the venv path, not the base interpreter symlink target.
        venv = Path(sys.prefix)
        script = venv / 'bin' / f'bike-{application}'
        assert script.is_file(), f'missing installed console entry point: {script}'
        assert re.search(r'if\s+__name__\s*==\s*[\"\x27]__main__[\"\x27]',
                         script.read_text()), f'unguarded spawn entry point: {script}'
        environment['UV_PROJECT_ENVIRONMENT'] = str(venv)
        prefix = [uv, 'run', '--frozen', '--no-sync', f'bike-{application}']
    results = []
    for async_enabled in (False, True):
        out = tmp_path / f'out-{async_enabled}'
        arguments = [*common, '--out', str(out)]
        if async_enabled:
            arguments.append('--fast-telemetry' if application == 'research' else '--async-telemetry')
        result = subprocess.run([*prefix, *arguments], cwd=ROOT, env=environment,
                                capture_output=True, text=True, timeout=120, check=False)
        diagnostic = result.stdout + result.stderr
        assert result.returncode in (0, 2), diagnostic
        assert 'async close disabled' not in diagnostic
        assert 'async close worker lost' not in diagnostic
        assert 'resource_tracker:' not in result.stderr
        if application == 'research':
            report = batch._tail_json(result.stdout.strip())
            assert report is not None, diagnostic
            assert report['outcome'] == 'duration'
            assert report['recording'] == 'off' and report['output'] is None
        else:
            assert 'duration: 0.010s' in result.stdout, diagnostic
            assert 'recording off' in result.stdout, diagnostic
        assert not out.exists(), diagnostic
        results.append(result.returncode)
    assert results[0] == results[1], results
