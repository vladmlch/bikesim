"""Ordinary ``bike-ride`` recording/check-policy parser contracts (R02, R22).

``--record`` opts ordinary physical native and research Python/native
headless/viewer runs into capture. Legacy and preview modes, and ordinary
physical Python capture, reject the flag before any stepping. ``--check-policy`` resolves the existing
stop-on-model-violation policy independently of recording. ``--out`` alone
never enables recording and ``--time-scale`` stays interactive-only.
"""
from pathlib import Path

import pytest

from bike_sim.cli.ride import parse_args

ROOT = Path(__file__).resolve().parents[2]
PROFILE = str(ROOT/'examples/research/viewer_physics_welded.toml')


def _error_code(argv):
    with pytest.raises(SystemExit) as error:
        parse_args(argv)
    return error.value.code


def test_record_defaults_off_and_check_policy_defaults_unset():
    args = parse_args(['--physics-config', PROFILE])
    assert args.record is False
    assert args.check_policy is None
    # The resolved policy preserves the viewer's historical diagnostic default.
    assert args.resolved_check_policy == 'diagnostic'


def test_out_alone_never_enables_recording(tmp_path):
    args = parse_args(['--physics-config', PROFILE, '--backend', 'native',
                       '--out', str(tmp_path)])
    assert args.record is False


@pytest.mark.parametrize('headless', [False, True])
def test_record_is_accepted_for_physical_native_runs(headless):
    argv = ['--physics-config', PROFILE, '--backend', 'native', '--record']
    if headless:
        argv += ['--headless', '--duration', '1', '--no-plots']
    args = parse_args(argv)
    assert args.record is True
    assert args.backend == 'native'
    assert args.physics == 'physical'


def test_record_rejected_for_python_backend():
    assert _error_code(['--physics-config', PROFILE, '--record']) == 2
    assert _error_code(['--physics-config', PROFILE, '--backend', 'python',
                        '--headless', '--duration', '1', '--record']) == 2


def test_record_rejected_for_legacy_physics():
    assert _error_code(['--physics', 'legacy', '--record']) == 2
    assert _error_code(['--record']) == 2  # legacy is the default physics mode


@pytest.mark.parametrize('backend', ['python', 'native'])
def test_record_accepted_for_research(backend):
    args = parse_args(['--physics-config', PROFILE, '--backend', backend,
                       '--research', '--record'])
    assert args.record is True
    assert args.resolved_check_policy == 'diagnostic'


def test_record_rejected_for_preview():
    assert _error_code(['--physics-config', PROFILE, '--backend', 'native',
                        '--preview', '--record']) == 2


def test_check_policy_resolved_before_runtime_and_independent_of_record():
    args = parse_args(['--physics-config', PROFILE, '--backend', 'native',
                       '--headless', '--duration', '1', '--check-policy', 'diagnostic'])
    assert args.check_policy == 'diagnostic'
    assert args.resolved_check_policy == 'diagnostic'
    assert args.record is False
    args = parse_args(['--physics-config', PROFILE, '--backend', 'native',
                       '--check-policy', 'strict', '--record'])
    assert args.resolved_check_policy == 'strict'
    assert args.record is True
    # Headless keeps its historical strict default when the flag is absent.
    args = parse_args(['--physics-config', PROFILE, '--backend', 'native',
                       '--headless', '--duration', '1'])
    assert args.resolved_check_policy == 'strict'


def test_check_policy_accepted_for_python_backend_physical_runs():
    args = parse_args(['--physics-config', PROFILE, '--check-policy', 'strict'])
    assert args.check_policy == 'strict'
    assert args.resolved_check_policy == 'strict'


@pytest.mark.parametrize('policy', ['strict', 'diagnostic'])
def test_check_policy_rejected_outside_physical_and_research(policy):
    assert _error_code(['--physics', 'legacy', '--check-policy', policy]) == 2
    args = parse_args(['--physics-config', PROFILE, '--research',
                       '--check-policy', policy])
    assert args.resolved_check_policy == policy
    assert _error_code(['--physics-config', PROFILE, '--preview',
                        '--check-policy', policy]) == 2


def test_time_scale_remains_interactive_only():
    assert _error_code(['--physics-config', PROFILE, '--backend', 'native',
                        '--record', '--headless', '--duration', '1',
                        '--time-scale', '2']) == 2
    args = parse_args(['--physics-config', PROFILE, '--backend', 'native',
                       '--record', '--time-scale', '2'])
    assert args.time_scale == 2 and args.record is True


def test_unsupported_combination_names_itself(capsys):
    with pytest.raises(SystemExit):
        parse_args(['--physics-config', PROFILE, '--record'])
    message = capsys.readouterr().err
    assert 'native' in message
    with pytest.raises(SystemExit):
        parse_args(['--physics', 'legacy', '--record'])
    assert 'physical' in capsys.readouterr().err
    with pytest.raises(SystemExit):
        parse_args(['--physics-config', PROFILE, '--backend', 'native',
                    '--preview', '--record'])
    assert 'preview' in capsys.readouterr().err
