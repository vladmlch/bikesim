import pytest

from bike_sim.validation.climbing_evidence import support_evidence


def interval(start, end, *, load=100., gap=-.001, inside=True, energy=None, valid=True):
    return {
        'time_s': start, 'end_time_s': end,
        'rider': {side + '_pedal': {
            'in_platform': inside, 'normal_load_n': load, 'gap_m': gap,
        } for side in ('front', 'rear')},
        'model_status': {'model_valid': valid, 'numerically_valid': True},
        'energy': {'residual_j': 0., 'energy_scale_j': 10.} if energy is None else energy,
    }


def test_overlap_without_pressure_does_not_count_as_support():
    result = support_evidence([interval(0., .25, load=0., gap=.03)])
    assert result['first_both_unloaded_s'] == 0.
    assert result['max_both_unloaded_s'] == pytest.approx(.25)
    assert result['max_front_gap_m'] == .03
    assert not result['complete']


def test_irregular_intervals_use_duration_and_stop_counting_after_recovery():
    rows = [interval(0., .05), interval(.05, .14, load=0.),
            interval(.14, .27, load=0.), interval(.27, .4),
            interval(.4, .49, load=0.)]
    result = support_evidence(rows, required_duration_s=.49)
    assert result['max_both_unloaded_s'] == pytest.approx(.22)
    assert result['first_both_unloaded_s'] == pytest.approx(.05)
    assert result['complete']
    assert result['numerically_valid']
    assert result['model_valid']


def test_later_bad_energy_is_not_hidden_by_a_stale_numeric_flag():
    rows = [interval(0., .1), interval(.1, .2, energy={
        'residual_j': 1., 'energy_scale_j': 10.,
    })]
    result = support_evidence(rows, required_duration_s=.2)
    assert result['complete']
    assert not result['numerically_valid']
    assert result['first_invalid_s'] == .1
    assert 'energy_quality' in result['invalid_reason']


def test_model_invalidity_latches_even_if_a_later_row_claims_validity():
    rows = [interval(0., .1, valid=False), interval(.1, .2)]
    result = support_evidence(rows, required_duration_s=.2)
    assert not result['model_valid']
    assert result['first_invalid_s'] == 0.


def test_missing_energy_is_unknown_and_cannot_pass_quality():
    row = interval(0., .1)
    del row['energy']
    result = support_evidence([row], required_duration_s=.1)
    assert result['numerically_valid'] is None
    assert result['first_unknown_s'] == 0.
    assert not result['valid_for_learning']


def test_empty_or_short_episode_cannot_count_as_full_completion():
    assert not support_evidence([], required_duration_s=1.)['complete']
    assert not support_evidence([interval(0., .1)], required_duration_s=1.)['complete']


def test_noncontiguous_or_overlapping_history_is_rejected():
    for start in (.05, .15):
        with pytest.raises(ValueError, match='contiguous'):
            support_evidence([interval(0., .1), interval(start, .2)])


def test_pressure_outside_the_platform_is_not_usable_support():
    result = support_evidence([interval(0., .25, inside=False)])
    assert result['max_both_unloaded_s'] == .25


def test_missing_contact_telemetry_is_unknown_rather_than_known_support_loss():
    row = interval(0., .25)
    row['rider'] = {}
    result = support_evidence([row], required_duration_s=.25)
    assert not result['support_observed']
    assert result['first_both_unloaded_s'] is None
    assert result['max_both_unloaded_s'] == 0.


def test_report_preserves_later_invalidity_and_statistics_of_the_valid_prefix():
    rows = [interval(0., .1, gap=.002), interval(.1, .2, valid=False, gap=.1),
            interval(.2, .3, energy={'residual_j': 1., 'energy_scale_j': 10.})]
    result = support_evidence(rows, required_duration_s=.3)
    assert result['valid_prefix_duration_s'] == .1
    assert result['valid_prefix_max_gap_m'] == .002
    assert set(result['all_invalid_reasons']) == {'model_scope', 'energy_quality'}


@pytest.mark.parametrize('duration', [0., -1., float('nan'), True])
def test_horizon_must_be_a_positive_finite_time(duration):
    with pytest.raises(ValueError):
        support_evidence([], required_duration_s=duration)
