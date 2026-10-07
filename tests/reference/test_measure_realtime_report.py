"""Unit coverage for the realtime report's provenance and baseline blocks.

measure_realtime() itself is exercised end-to-end by the slow realtime
gate; these tests pin the bookkeeping contracts — percentile keys,
build/hardening provenance, per-metric deltas and the comparability
flags — without paying for a 20-second simulation.
"""
import json
from pathlib import Path
import sys

import pytest

from tools.measure_realtime import (
    _baseline_comparison, _native_build_provenance)

ROOT = Path(__file__).resolve().parents[2]


def _write_json(path, report):
    path.write_text(json.dumps(report))
    return str(path)


def _report(**overrides):
    report = {'factor': 2., 'step_ms_mean': 1., 'step_ms_p50': .9,
              'step_ms_p95': 1.5, 'step_ms_p99': 2., 'wall_seconds': 10.,
              'steps': 100, 'sim_seconds': .05,
              'git_sha': 'aaa', 'track': 't', 'physics': 'p',
              'machine': 'm', 'effective_timestep_s': .001,
              'requested_duration_s': 20., 'record_decimation': 80}
    report.update(overrides)
    return report


def test_baseline_comparison_emits_per_metric_deltas(tmp_path):
    current = _report()
    baseline = _report(factor=1., step_ms_p95=1., git_sha='bbb')
    result = _baseline_comparison(
        current, _write_json(tmp_path / 'baseline.json', baseline))
    metrics = result['metrics']
    assert metrics['factor']['current'] == 2.
    assert metrics['factor']['baseline'] == 1.
    assert metrics['factor']['delta'] == pytest.approx(1.)
    assert metrics['factor']['delta_pct'] == pytest.approx(100.)
    assert metrics['step_ms_p95']['delta'] == pytest.approx(.5)
    assert metrics['step_ms_p95']['delta_pct'] == pytest.approx(50.)
    # Every measured key reports current/baseline even when identical.
    assert metrics['step_ms_p50']['delta'] == pytest.approx(0.)
    assert metrics['step_ms_p99']['delta'] == pytest.approx(0.)


def test_baseline_comparison_comparability_flags(tmp_path):
    current = _report()
    baseline = _report(git_sha='bbb', machine='other-box',
                       effective_timestep_s=.002)
    result = _baseline_comparison(
        current, _write_json(tmp_path / 'baseline.json', baseline))
    comparability = result['comparability']
    assert comparability['git_sha'] == {
        'current': 'aaa', 'baseline': 'bbb', 'match': False}
    assert comparability['machine']['match'] is False
    assert comparability['effective_timestep_s']['match'] is False
    assert comparability['track']['match'] is True
    assert comparability['physics']['match'] is True
    assert comparability['record_decimation']['match'] is True


def test_baseline_comparison_missing_key_stays_missing(tmp_path):
    baseline = _report()
    del baseline['step_ms_p99']
    result = _baseline_comparison(
        _report(), _write_json(tmp_path / 'baseline.json', baseline))
    entry = result['metrics']['step_ms_p99']
    assert entry == {'current': 2., 'baseline': None}
    assert 'delta' not in entry


def test_baseline_comparison_rejects_unreadable_or_non_object(tmp_path):
    with pytest.raises(ValueError, match='cannot read baseline'):
        _baseline_comparison(_report(), tmp_path / 'missing.json')
    not_object = tmp_path / 'list.json'
    not_object.write_text('[1, 2]')
    with pytest.raises(ValueError, match='not a JSON object'):
        _baseline_comparison(_report(), not_object)


def test_native_build_provenance_reads_configured_context(
        tmp_path, monkeypatch):
    build = tmp_path / 'selected-build'
    build.mkdir()
    (build / 'native_check_context.json').write_text(json.dumps({
        'compiler': {'path': '/usr/bin/clang++', 'id': 'Clang',
                     'version': '19.0'},
        'sdk': {'path': '/sdk'},
        'libcpp_hardening': 'EXTENSIVE',
        'target_contexts': {'bike_native': {
            'configuration': 'Release', 'cxx_standard': 23,
            'cxx_extensions': False, 'compile_definitions': ['NDEBUG'],
            'compile_options': ['-O2']}}}))
    (build / 'CMakeCache.txt').write_text(
        'NATIVE_SANITIZE:BOOL=ON\nNATIVE_RTSAN:BOOL=OFF\n'
        'NATIVE_COVERAGE:BOOL=OFF\nOTHER:STRING=x\n')
    monkeypatch.setenv('NATIVE_TEST_BUILD_PATH', str(build))
    monkeypatch.delenv('NATIVE_TEST_BUILD_DIR', raising=False)
    provenance = _native_build_provenance(ROOT)
    assert provenance['build_dir'] == str(build)
    assert provenance['compiler']['id'] == 'Clang'
    assert provenance['hardening'] == 'EXTENSIVE'
    assert provenance['target_configuration'] == 'Release'
    assert provenance['cxx_standard'] == 23
    assert provenance['cxx_extensions'] is False
    assert provenance['sanitizer_options'] == {
        'address': True, 'realtime': False, 'coverage': False}
    # The imported path reports whatever the interpreter mapped — the key
    # exists whether or not this process loaded bike_native.
    assert 'imported_extension' in provenance


def test_native_build_provenance_degrades_without_context(
        tmp_path, monkeypatch):
    build = tmp_path / 'empty-build'
    build.mkdir()
    monkeypatch.setenv('NATIVE_TEST_BUILD_PATH', str(build))
    monkeypatch.delenv('NATIVE_TEST_BUILD_DIR', raising=False)
    provenance = _native_build_provenance(ROOT)
    assert provenance['build_dir'] == str(build)
    assert 'context_error' in provenance
    # Missing context is recorded, never raised — the measurement must
    # still emit a report for a build tree that predates the context file.
    assert 'compiler' not in provenance
