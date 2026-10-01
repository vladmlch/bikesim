import pytest

from bike_sim.validation.climbing_benchmark import timing_metrics, run_benchmark


def test_throughput_excludes_setup_and_uses_actual_timed_steps():
    metrics = timing_metrics([1_000_000] * 100, .00125)
    assert metrics['real_time_factor'] == pytest.approx(1.25)
    assert metrics['p99_step_ms'] == pytest.approx(1.)
    assert metrics['step_count'] == 100


@pytest.mark.parametrize('samples', [[], [0], [-1], [float('nan')], [True]])
def test_invalid_timing_cannot_become_a_speed_claim(samples):
    with pytest.raises(ValueError):
        timing_metrics(samples, .00125)


def test_percentiles_do_not_hide_a_slow_step():
    metrics = timing_metrics([1_000_000] * 99 + [100_000_000], .001)
    assert metrics['maximum_step_ms'] == 100.
    assert metrics['real_time_factor'] < 1.


def test_benchmark_never_truncates_an_existing_run(tmp_path):
    output = tmp_path / 'run'
    output.mkdir()
    marker = output / 'keep.txt'
    marker.write_text('original')
    with pytest.raises(FileExistsError):
        run_benchmark('unused', 'unused', mode='preview', duration_s=1., repeats=1, output_dir=output)
    assert marker.read_text() == 'original'
