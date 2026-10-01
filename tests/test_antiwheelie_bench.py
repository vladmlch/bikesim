import pytest

from bike_sim.validation.antiwheelie_bench import evidence_matches


def test_evidence_requires_source_config_and_runtime_match():
    expected = {
        'source_sha256': 'source-a',
        'configuration_sha256': 'config-a',
        'versions': {'python': '3.13.12', 'mujoco': '3.12.0'},
    }
    assert evidence_matches(dict(expected), expected)
    assert not evidence_matches(dict(expected, source_sha256='old'), expected)
    assert not evidence_matches(dict(expected, configuration_sha256='old'), expected)
    assert not evidence_matches({}, expected)


def test_absent_expected_identity_cannot_certify_old_evidence():
    assert not evidence_matches({}, {})


def valid_metrics(**changes):
    return dict(duration_s=15., progress_m=10., outcome='duration',
                numerically_valid=True, model_status={'model_valid': True},
                wheelie={'wheelie_time_s': 0.}) | changes


def test_early_valid_crash_is_not_a_completed_long_run():
    from bike_sim.validation.antiwheelie_bench import assess_run
    result = assess_run(valid_metrics(duration_s=.3, progress_m=.1, outcome='crash:loop_out'),
                        required_duration_s=15., minimum_progress_m=5.)
    assert not result['accepted']
    assert 'incomplete_horizon' in result['reasons']
    assert 'crash' in result['reasons']


@pytest.mark.parametrize('change, reason', [
    ({'numerically_valid': False}, 'numerical_invalid'),
    ({'model_status': {'model_valid': False}}, 'model_invalid'),
    ({'operator_intervention': True}, 'operator_intervention'),
    ({'progress_m': 0.}, 'insufficient_progress'),
])
def test_long_run_gate_requires_valid_progress_without_intervention(change, reason):
    from bike_sim.validation.antiwheelie_bench import assess_run
    result = assess_run(valid_metrics(**change), required_duration_s=15., minimum_progress_m=5.)
    assert not result['accepted']
    assert reason in result['reasons']


def test_missing_or_nonfinite_long_run_evidence_cannot_pass():
    from bike_sim.validation.antiwheelie_bench import assess_run
    assert not assess_run({}, required_duration_s=15.)['accepted']
    for duration in (float('nan'), float('inf'), -1.):
        with pytest.raises(ValueError):
            assess_run(valid_metrics(duration_s=duration), required_duration_s=15.)


def pair_records():
    common = dict(scenario_id='flat', seed=17, plant_and_inputs_sha256='same',
                  operator_intervention=False)
    return [dict(common, policy_id='passthrough', metrics=valid_metrics()),
            dict(common, policy_id='zero', metrics=valid_metrics(progress_m=0.))]


def test_pair_report_keeps_progress_and_does_not_rank_zero_as_success():
    from bike_sim.validation.antiwheelie_bench import paired_summary
    report = paired_summary(pair_records())
    assert report['pairs'][0]['policies']['zero']['progress_m'] == 0.
    assert report['pairs'][0]['policies']['passthrough']['progress_m'] == 10.
    assert 'winner' not in report['pairs'][0]
    assert report['comparable_pair_count'] == 1


@pytest.mark.parametrize('case', ['different_inputs', 'operator', 'numerical', 'model', 'duration', 'missing_baseline'])
def test_incompatible_or_invalid_runs_are_excluded_from_comparable_pairs(case):
    from bike_sim.validation.antiwheelie_bench import paired_summary
    records = pair_records()
    if case == 'different_inputs':
        records[1]['plant_and_inputs_sha256'] = 'other'
    elif case == 'operator':
        records[1]['operator_intervention'] = True
    elif case == 'numerical':
        records[1]['metrics']['numerically_valid'] = False
    elif case == 'model':
        records[1]['metrics']['model_status']['model_valid'] = False
    elif case == 'duration':
        records[1]['metrics']['duration_s'] = 2.
    else:
        records = records[1:]
    report = paired_summary(records)
    assert report['comparable_pair_count'] == 0
    assert report['exclusions']
    assert len(report['records']) == len(records)
