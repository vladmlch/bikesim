from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import importlib.util
import pytest
from bike_sim.sim.realtime_measurement import measure_run, compare_runs, build_report, chunk_statistics


class Clock:
    def __init__(self):
        self.now = 0.
    def __call__(self):
        return self.now


class Driver:
    backend = 'python'
    dt_s = .00125
    max_steps = 4
    record_decimation = 80
    strict = True
    def __init__(self, clock):
        self.clock = clock
        self.step = 0
        self.time_s = 0.
        self.reason = None
        self.closed = False
        self.exported = False
        self.metadata = dict(configuration_sha256='cfg', terrain_sha256='terrain',
            controller_interval_s=.0025, versions={'python': 'fixture'},
            execution={'python_source_sha256': 'source', 'build_context': None})
    def start(self):
        self.clock.now += .5  # Normal boundary/loop work, not native kernel time.
    def advance(self, target_step, *, wall_budget_s):
        assert target_step == self.max_steps
        assert wall_budget_s == .008
        self.clock.now += 2.
        self.step += 2
        self.time_s = self.step*self.dt_s
        if self.step == self.max_steps:
            self.reason = 'duration_reached'
    def flush(self):
        self.clock.now += 1.
    def snapshot(self):
        return SimpleNamespace(step=self.step, time_s=self.time_s)
    def summary(self, reason):
        return dict(outcome={'reason': reason}, model_status=dict(model_valid=True, numerically_valid=True))
    def export(self, path, *, reason):
        self.clock.now += 3.
        self.exported = True
    def close(self):
        self.closed = True


def sample():
    clock = Clock()
    driver = Driver(clock)
    def factory():
        clock.now += 2.
        return driver
    report = measure_run(factory, 'not-written-by-fake', clock=clock,
                         identity_reader=lambda backend: driver.metadata['execution'])
    return driver, report


def test_whole_loop_includes_boundary_and_final_flush_but_not_setup_export():
    driver, report = sample()
    assert report['setup_wall_s'] == 2.
    assert report['wall_clock_s'] == 5.5
    assert report['final_flush_wall_s'] == 1.
    assert report['export_wall_s'] == 3.
    assert report['chunk_wall_s'] == [2., 2.]
    assert report['chunk_completed_steps'] == [2, 2]
    assert report['real_time_factor'] == pytest.approx(.005/5.5)
    assert report['chunk_latency']['p95'] == 2.
    assert report['per_step_latency'] is None
    assert report['valid_prefix']
    assert driver.closed and driver.exported


@pytest.mark.parametrize('field,value', [('timestep_s', .0005), ('configuration_sha256', 'other'),
    ('record_decimation', 1), ('terrain_sha256', 'other'), ('strict', False)])
def test_different_numerical_or_recording_settings_are_not_comparable(field, value):
    _, report = sample()
    other = deepcopy(report)
    other['comparability'][field] = value
    comparison = compare_runs(report, other)
    assert not comparison['comparable']
    assert field in comparison['differing_settings']


def test_different_outcome_and_short_prefix_remain_visible():
    _, report = sample()
    other = deepcopy(report)
    other['prefix'].update(steps=2, reason='crash', simulated_time_s=.0025)
    check = compare_runs(report, other)
    assert not check['comparable'] and check['short_prefix']
    assert 'reason' in check['differing_prefix_fields']
    aggregate = build_report([other], [report, report, report])
    assert aggregate['warmup_count'] == 1
    assert aggregate['measurement_count'] == 3
    assert aggregate['median_real_time_factor'] == report['real_time_factor']
    assert not aggregate['full_acceptance_verified']


def test_empty_chunk_statistics_are_not_invented():
    assert chunk_statistics([])['p95'] is None


def test_native_threshold_requires_warmup_and_three_valid_repetitions():
    _, report = sample()
    report['backend'] = 'native'
    report['execution']['build_context'] = {'build_type': 'Release'}
    report['real_time_factor'] = 2.
    insufficient = build_report([], [report])
    assert not insufficient['sampling_protocol_satisfied']
    assert insufficient['headless_rtf_threshold_met'] is None
    enough = build_report([report], [report, report, report])
    assert enough['sampling_protocol_satisfied']
    assert enough['headless_rtf_threshold_met'] is True
    assert enough['full_acceptance_verified'] is False


def test_measurement_cli_keeps_profile_dt_until_explicit_override():
    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location('measure_realtime_test_module', root/'tools/measure_realtime.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)  # Imports definitions; the main guard is not entered.
    args = module.parser().parse_args(['--out', 'unused'])
    assert args.dt is None
    assert module.ride_arguments(args).resolved_physics.timestep_s == .00125
    args.dt = .0005
    assert module.ride_arguments(args).resolved_physics.timestep_s == .0005
