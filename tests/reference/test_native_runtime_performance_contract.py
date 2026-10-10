"""C4 performance-acceptance contract gates.

The acceptance contract (spec section 1 item 8; plan C4 completion) is:
native unpaced median RTF >= 1.0 headless, and a real window sustaining
mean achieved RTF >= 0.95 at requested 1x. This file pins the contract
boundaries the sibling suites do not already assert:

- the threshold constants as gate behavior (below -> False, at -> True,
  wrong backend/build -> None rather than a False claim),
- failed runs are ineligible by construction: ``measured_ok`` is False,
  ``compare_runs`` refuses them even when every field matches, and a full
  warmup+3x sampling of invalid runs leaves ``headless_rtf_threshold_met``
  undecided (None), not failed (False).

Machinery coverage stays in test_realtime_measurement.py /
test_measure_realtime_report.py: wall-clock composition, chunk
percentiles, comparability settings, short-prefix visibility, sampling
protocol, CLI dt preservation, provenance and baseline blocks.
"""
from copy import deepcopy
from types import SimpleNamespace

import pytest

from native_loader import load_native

from bike_sim.sim.realtime_measurement import (
    build_report, compare_runs, measure_run)


# Named acceptance thresholds (spec: "sustains mean achieved RTF >= 0.95
# at requested 1x"; plan C4: "native unpaced median RTF >= 1.0").
NATIVE_UNPACED_MEDIAN_RTF_MIN = 1.0
VISUAL_1X_MEAN_RTF_MIN = 0.95


class _Clock:
    def __init__(self):
        self.now = 0.

    def __call__(self):
        return self.now


class _FailingDriver:
    """Setup succeeds; the first measured advance raises."""
    backend = 'python'
    dt_s = .00125
    max_steps = 4
    record_decimation = 80
    strict = True

    def __init__(self):
        self.step = 0
        self.time_s = 0.
        self.reason = None
        self.closed = False
        self.metadata = dict(
            configuration_sha256='cfg', terrain_sha256='terrain',
            controller_interval_s=.0025, versions={'python': 'fixture'},
            execution={'python_source_sha256': 'source',
                       'build_context': None})

    def start(self):
        pass

    def advance(self, target_step, *, wall_budget_s):
        raise RuntimeError('engine exploded mid-interval')

    def flush(self):
        pass

    def snapshot(self):
        return SimpleNamespace(step=self.step, time_s=self.time_s)

    def summary(self, reason):
        # Model stays valid: ineligibility must come from the failure
        # alone, not from a model-status flag.
        return dict(outcome={'reason': reason}, model_status=dict(
            model_valid=True, numerically_valid=True))

    def export(self, path, *, reason):
        pass

    def close(self):
        self.closed = True


def _failing_report():
    driver = _FailingDriver()
    clock = _Clock()
    report = measure_run(lambda: driver, clock=clock,
                         identity_reader=lambda backend:
                         driver.metadata['execution'])
    return driver, report


def _native_report(real_time_factor, **overrides):
    """A valid native Release measurement dict for build_report."""
    report = dict(backend='native',
        comparability=dict(timestep_s=.00125, record_decimation=80),
        execution={'build_context': {'build_type': 'Release'}},
        measured_ok=True, valid_prefix=True,
        real_time_factor=real_time_factor,
        prefix=dict(steps=800, requested_steps=800, reason='duration',
                    model_valid=True, numerically_valid=True,
                    simulated_time_s=1.))
    report.update(overrides)
    return report


def _aggregate(report):
    """One warmup plus three identical-classification measurements."""
    return build_report(
        [deepcopy(report)], [deepcopy(report) for _ in range(3)])


def test_selected_native_artifact_loads():
    # These gates bind a native Release measurement — assert the selected
    # artifact itself resolves so this file keeps the test_native_* loader
    # contract instead of only exercising pure-Python aggregation.
    assert load_native() is not None


def test_failed_run_is_measured_not_ok_and_short():
    driver, report = _failing_report()
    assert driver.closed
    assert report['measured_ok'] is False
    assert report['valid_prefix'] is False
    assert report['errors'][0]['stage'] == 'advance'
    assert report['prefix']['reason'] == 'simulation_error'
    assert report['prefix']['steps'] < report['prefix']['requested_steps']
    assert any('failed run is evidence' in line.lower()
               for line in report['limitations'])


def test_failed_runs_are_never_comparable():
    _, report = _failing_report()
    # Identical dicts: refusal comes from measured_ok alone, not from any
    # settings or prefix difference — the audit's not_comparable contract.
    check = compare_runs(report, deepcopy(report))
    assert check['comparable'] is False
    assert not check['differing_settings']
    assert not check['differing_prefix_fields']
    report['backend'] = 'native'
    report['execution'] = {'build_context': {'build_type': 'Release'}}
    aggregate = _aggregate(report)
    assert aggregate['sampling_protocol_satisfied']
    # Full protocol met, every run invalid: gate stays undecided, never
    # silently failed or passed.
    assert aggregate['headless_rtf_threshold_met'] is None


def test_headless_threshold_boundary_is_exactly_the_named_constant():
    below = _aggregate(_native_report(0.5))
    assert below['median_real_time_factor'] == pytest.approx(0.5)
    # Eligible and below the gate: evaluated False, not ineligible None.
    assert below['headless_rtf_threshold_met'] is False
    at = _aggregate(_native_report(NATIVE_UNPACED_MEDIAN_RTF_MIN))
    assert at['median_real_time_factor'] == pytest.approx(1.0)
    assert at['headless_rtf_threshold_met'] is True  # >= is inclusive


def test_threshold_gate_only_applies_to_native_release():
    python_run = _native_report(2., backend='python')
    assert _aggregate(python_run)['headless_rtf_threshold_met'] is None
    debug = _native_report(2., execution={
        'build_context': {'build_type': 'Debug'}})
    assert _aggregate(debug)['headless_rtf_threshold_met'] is None


def test_visual_1x_gate_is_emitted_as_a_pending_gate():
    aggregate = _aggregate(_native_report(2.))
    gates = ' '.join(aggregate['pending_gates'])
    assert (f'1x mean achieved RTF >= {VISUAL_1X_MEAN_RTF_MIN}' in gates)
    # The visual gate is measured on a real window; a headless report
    # must keep it pending rather than claim full acceptance.
    assert aggregate['full_acceptance_verified'] is False
