"""Whole-boundary throughput measurements; never infer a per-step percentile.

This module does not run anything on import. A caller supplies a fresh physical
owner for every warm-up/measurement. Setup, measured work and export have
separate clocks; the measured interval includes the final accounting flush.
"""
from __future__ import annotations

from copy import deepcopy
import math
from statistics import median
import time

from bike_sim.sim.playback import COMPUTE_SLICE_S


def _elapsed(end, start):
    value = end-start
    if not math.isfinite(value) or value < 0.:
        raise ValueError('measurement clock must be finite and monotonic')
    return value


def chunk_statistics(values):
    """Linear-interpolated quantiles of directly timed boundary calls, seconds."""
    data = sorted(values)
    if any(not math.isfinite(value) or value < 0. for value in data):
        raise ValueError('chunk latencies must be finite and nonnegative')
    def quantile(fraction):
        index = (len(data)-1)*fraction
        lower = math.floor(index)
        return data[lower] + (data[math.ceil(index)]-data[lower])*(index-lower)
    return dict(count=len(data), units='seconds per advance call',
        min=None if not data else data[0], p50=None if not data else quantile(.5),
        p95=None if not data else quantile(.95), max=None if not data else data[-1])


def comparison_key(driver):
    """Backend intentionally excluded; all shared numerical settings retained."""
    metadata = driver.metadata
    return dict(configuration_sha256=metadata['configuration_sha256'],
        terrain_sha256=metadata['terrain_sha256'],
        python_source_sha256=metadata['execution']['python_source_sha256'],
        versions=deepcopy(metadata['versions']),
        timestep_s=driver.dt_s, internal_controller_period_s=metadata['controller_interval_s'],
        external_policy_period_s=None, max_steps=driver.max_steps,
        record_decimation=driver.record_decimation, strict=driver.strict,
        compute_slice_s=COMPUTE_SLICE_S,
        controls='default RideControl; front/rear brake demand 0; profile-owned rider/assist')


def _identity(backend):
    from bike_sim.native.artifact import execution_provenance
    return execution_provenance(backend)


def measure_run(factory, destination=None, *, clock=time.perf_counter, identity_reader=_identity):
    """Measure a fresh owner; return failed prefixes too, with no success claim.

    Setup errors propagate before an output directory is created. Runtime errors
    stop further physics, but a final flush and export of the committed prefix
    are attempted. The owner is always closed. KeyboardInterrupt remains an
    interrupt after cleanup, not a throughput sample.
    """
    setup_start = clock()
    driver = factory()
    try:
        key = comparison_key(driver)
        initial_step, initial_time = driver.step, driver.time_s
        setup_end = clock()
        failures = []
        chunks, step_counts = [], []
        measured_start = clock()
        driver.start()
        try:
            while driver.reason is None:
                before_step = driver.step
                chunk_start = clock()
                try:
                    driver.advance(driver.max_steps, wall_budget_s=COMPUTE_SLICE_S)
                finally:
                    chunks.append(_elapsed(clock(), chunk_start))
                    step_counts.append(driver.step-before_step)
        except Exception as error:
            failures.append(dict(stage='advance', type=type(error).__name__, message=str(error)))
        flush_start = clock()
        try:
            driver.flush()
        except Exception as error:
            failures.append(dict(stage='flush', type=type(error).__name__, message=str(error)))
        measured_end = clock()
        wall = _elapsed(measured_end, measured_start)
        # Snapshot refreshes the authoritative committed boundary after errors.
        frame = driver.snapshot()
        elapsed_sim = frame.time_s-initial_time
        reason = driver.reason
        if failures:
            reason = ('invalid_controller' if failures[0]['type'] == 'InvalidReferenceRun'
                      else 'simulation_error')
        summary = driver.summary(reason)
        try:
            identity_unchanged = identity_reader(driver.backend) == driver.metadata['execution']
        except Exception as error:
            identity_unchanged = False
            failures.append(dict(stage='identity', type=type(error).__name__, message=str(error)))
        if not identity_unchanged and not any(item['stage'] == 'identity' for item in failures):
            failures.append(dict(stage='identity', type='IdentityChanged',
                                 message='source or artifact changed during measurement'))
        export_wall = 0.
        if destination is not None:
            export_start = clock()
            try:
                driver.export(destination, reason=reason)
            except Exception as error:
                failures.append(dict(stage='export', type=type(error).__name__, message=str(error)))
            export_wall = _elapsed(clock(), export_start)
        status = summary['model_status']
        steps = frame.step-initial_step
        requested_steps = driver.max_steps-initial_step
        prefix = dict(steps=steps, simulated_time_s=elapsed_sim,
            requested_steps=requested_steps, fraction_of_requested_steps=(
                steps/requested_steps if requested_steps > 0 else None),
            reason=reason, model_valid=status['model_valid'],
            numerically_valid=status['numerically_valid'],
            first_failure=deepcopy(summary.get('first_failure')),
            first_model_violation=deepcopy(status.get('first_model_violation')))
        limits = []
        if steps < requested_steps:
            limits.append('Natural/failed terminal prefix is shorter than requested; inspect before accepting throughput.')
        if status['model_valid'] is not True or status['numerically_valid'] is not True:
            limits.append('Model/numerical validity is not established for this prefix.')
        if failures:
            limits.append('A failed run is evidence, not a successful performance measurement.')
        return dict(backend=driver.backend, comparability=key,
            execution=deepcopy(driver.metadata['execution']), identity_unchanged=identity_unchanged,
            setup_wall_s=_elapsed(setup_end, setup_start), wall_clock_s=wall,
            final_flush_wall_s=_elapsed(measured_end, flush_start), export_wall_s=export_wall,
            simulated_time_s=elapsed_sim, physics_steps=steps,
            real_time_factor=elapsed_sim/wall if wall > 0. else None,
            measured_scope='Python outer wall clock: all advance calls, boundary/loop overhead, and final flush',
            chunk_latency=chunk_statistics(chunks), chunk_completed_steps=step_counts,
            chunk_wall_s=chunks, per_step_latency=None,
            prefix=prefix, errors=failures, limitations=limits,
            measured_ok=not failures and steps > 0 and wall > 0.,
            valid_prefix=(not failures and steps > 0 and status['model_valid'] is True
                          and status['numerically_valid'] is True and identity_unchanged))
    finally:
        driver.close()


def compare_runs(left, right):
    """Separate matching settings/prefix classes from correctness or calibration."""
    keys = sorted(set(left['comparability']) | set(right['comparability']))
    differences = [name for name in keys if left['comparability'].get(name) != right['comparability'].get(name)]
    a, b = left['prefix'], right['prefix']
    classifications = [name for name in ('reason', 'model_valid', 'numerically_valid', 'steps')
                       if a[name] != b[name]]
    if not math.isclose(a['simulated_time_s'], b['simulated_time_s'], rel_tol=1e-9, abs_tol=1e-9):
        classifications.append('simulated_time_s')
    comparable = not differences and not classifications and left['measured_ok'] and right['measured_ok']
    valid = comparable and left['valid_prefix'] and right['valid_prefix']
    return dict(comparable=comparable, valid_prefix_comparison=valid,
        differing_settings=differences, differing_prefix_fields=classifications,
        classification=('same_valid_prefix' if valid else 'equally_classified_diagnostic_prefix'
                        if comparable else 'not_comparable'),
        # Never conceal a shorter trajectory behind its faster RTF.
        short_prefix=a['steps'] < a['requested_steps'] or b['steps'] < b['requested_steps'],
        scope='Performance comparability only; not a numerical parity or physical calibration check.')


def build_report(warmups, runs):
    if not runs:
        raise ValueError('at least one measured run is required')
    factors = [run['real_time_factor'] for run in runs if run['real_time_factor'] is not None]
    pair_checks = [compare_runs(runs[0], run) for run in runs[1:]]
    same = all(check['comparable'] for check in pair_checks)
    sampling_protocol = len(warmups) >= 1 and len(runs) >= 3
    warmup_checks = [compare_runs(runs[0], run) for run in warmups]
    eligible = (same and all(run['valid_prefix'] for run in runs) and
                len(factors) == len(runs) and sampling_protocol and
                all(check['valid_prefix_comparison'] for check in warmup_checks))
    median_rtf = median(factors) if factors else None
    release = (runs[0]['execution'].get('build_context') or {}).get('build_type') == 'Release'
    return dict(schema_version=1, backend=runs[0]['backend'], warmup_count=len(warmups),
        measurement_count=len(runs), warmups=warmups, runs=runs,
        median_real_time_factor=median_rtf, repeated_runs_comparable=same,
        repeated_run_comparisons=pair_checks,
        sampling_protocol_satisfied=sampling_protocol,
        warmup_comparisons=warmup_checks,
        headless_rtf_threshold_met=(median_rtf >= 1. if eligible and release and
                                    runs[0]['backend'] == 'native' else None),
        short_prefix_review_required=any(run['prefix']['steps'] < run['prefix']['requested_steps'] for run in runs),
        full_acceptance_verified=False,
        pending_gates=['same-prefix Python/native comparison', 'full correctness and sanitizer gates',
                       'actual macOS viewer and 1x mean achieved RTF >= 0.95'],
        scope='Measured headless runtime only; warm-ups excluded from median; no GUI or calibration claim.')
