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
from bike_sim.native.artifact import execution_provenance
from bike_sim.sim.realtime_measurement import (
    build_report, compare_runs, measure_run)


# The selected-artifact import doubles as the sanitizer-runtime check.
bike_native = load_native()


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


class _MeasuredDriver(_FailingDriver):
    """Same fixture shape, but the loop completes and the metadata carries
    the real native execution identity a NativeRideDriver would."""

    backend = 'native'

    def __init__(self, clock, execution):
        super().__init__()
        self.clock = clock
        self.metadata['execution'] = execution

    def start(self):
        self.clock.now += .5  # boundary/loop work, not kernel time

    def advance(self, target_step, *, wall_budget_s):
        assert target_step == self.max_steps
        self.clock.now += 2.
        self.step = self.max_steps
        self.time_s = self.step * self.dt_s
        self.reason = 'duration_reached'

    def flush(self):
        self.clock.now += 1.


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


# ---- Distributed-tyre contract case (Plan B Task 8) ---------------------
# The compliant case's derivation, copied per the plan: the spec gate
# "sustains mean achieved RTF >= 0.95 at requested 1x" is implied in the
# contract's own units by native unpaced median RTF >= 1.0 headless —
# the unpaced run measures raw physics throughput with no pacing, so a
# median of exactly 1.0 leaves .05 of headroom for the paced window's
# scheduling overhead. The distributed backend shares that derivation
# unchanged: same interactive floor, same headless constant, same margin —
# never a backend-specific threshold.
DISTRIBUTED_PHYSICS_CONFIG = \
    'examples/research/viewer_physics_distributed.toml'
DISTRIBUTED_TRACK = 'examples/research/alpine_rocky.toml'


def _distributed_report(real_time_factor, *, valid=True):
    """The distributed-tyre twin of _native_report: the measurement
    identity of an alpine_rocky run under viewer_physics_distributed.toml
    (timestep .00125 s, record decimation 80, 128 stations, 30 s at
    duration) annotated the way tools/measure_realtime.py's
    annotate_report writes it.

    valid=False reproduces the shape this track actually measures: the
    documented baseline energy.constraint_work violation latches
    numerically_valid False, so a --benchmark run still completes the
    full 30 s prefix (steps == requested) while valid_prefix stays False
    — an honest throughput sample that can never meet the gate.
    """
    report = _native_report(real_time_factor)
    report['comparability'] = dict(report['comparability'],
        configuration_sha256='distributed-128station-alpine-rocky',
        terrain_sha256='alpine-rocky-hfield',
        max_steps=24000, strict=valid)
    report['invocation'] = dict(physics_config=DISTRIBUTED_PHYSICS_CONFIG,
        track=DISTRIBUTED_TRACK, duration_s=30., dt_override=None,
        effective_dt_s=.00125, record_decimation=80,
        monitor_strict=valid, benchmark_mode=not valid)
    report['benchmark_mode'] = not valid
    report['prefix'] = dict(report['prefix'],
        steps=24000, requested_steps=24000, simulated_time_s=30.)
    if not valid:
        report['valid_prefix'] = False
        report['limitations'] = [*report.get('limitations', []),
            'Benchmark mode: the reference monitor was non-strict; the '
            'measured prefix is a throughput sample of a ride whose '
            'validity is not established, never a valid-ride claim.']
        report['prefix']['numerically_valid'] = False
        report['prefix']['first_failure'] = dict(
            interval_id=4, time_s=.005, reasons=['energy.constraint_work'])
    return report


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


def test_distributed_case_gates_on_the_same_headless_constant():
    """The distributed-tyre contract case (alpine_rocky under
    viewer_physics_distributed.toml) uses the identical floor — the
    spec's interactive >= .95 at 1x expressed in headless units as
    NATIVE_UNPACED_MEDIAN_RTF_MIN — never a backend-specific number."""
    below = _aggregate(_distributed_report(0.5))
    assert below['headless_rtf_threshold_met'] is False
    at = _aggregate(_distributed_report(NATIVE_UNPACED_MEDIAN_RTF_MIN))
    assert at['median_real_time_factor'] == pytest.approx(
        NATIVE_UNPACED_MEDIAN_RTF_MIN)
    assert at['headless_rtf_threshold_met'] is True  # >= stays inclusive
    gates = ' '.join(at['pending_gates'])
    assert f'1x mean achieved RTF >= {VISUAL_1X_MEAN_RTF_MIN}' in gates


def test_distributed_benchmark_sample_reports_but_never_meets_the_gate():
    """The measured alpine_rocky shape: a --benchmark run completes the
    full 30 s prefix so its median RTF is honest evidence, but the
    latched numerically_valid keeps valid_prefix False — the headless
    gate stays undecided (None), never a silent pass or a fake fail."""
    aggregate = _aggregate(_distributed_report(2., valid=False))
    assert aggregate['median_real_time_factor'] == pytest.approx(2.)
    assert aggregate['sampling_protocol_satisfied']
    assert not any(run['valid_prefix'] for run in aggregate['runs'])
    # Full-duration runs are not a short-prefix case: the honesty check
    # fires only on a genuinely truncated measurement.
    assert aggregate['short_prefix_review_required'] is False
    assert aggregate['headless_rtf_threshold_met'] is None
    assert aggregate['full_acceptance_verified'] is False


def test_measure_run_binds_the_selected_artifact_identity(native_module):
    """The default identity reader re-derives provenance from the loaded
    extension; a driver carrying the real native execution dict stays
    identity_unchanged — the Release gate then reads real build context."""
    execution = execution_provenance('native', artifact=native_module)
    clock = _Clock()
    driver = _MeasuredDriver(clock, execution)
    report = measure_run(lambda: driver, clock=clock)
    assert report['measured_ok'] and report['valid_prefix']
    assert report['identity_unchanged'] is True
    assert report['execution'] == execution
    identity = native_module.runtime_identity()
    assert report['execution']['native_source_sha256'] == \
        identity['native_source_sha256']


def test_threshold_gate_tracks_the_selected_artifact_build_type(native_module):
    """headless_rtf_threshold_met keys off the build_context the selected
    artifact stamps at compile time — not a fabricated 'Release' flag."""
    execution = execution_provenance('native', artifact=native_module)
    report = _native_report(NATIVE_UNPACED_MEDIAN_RTF_MIN,
                            execution=execution)
    aggregate = _aggregate(report)
    is_release = execution['build_context']['build_type'] == 'Release'
    assert aggregate['headless_rtf_threshold_met'] is (
        True if is_release else None)
