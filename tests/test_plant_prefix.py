from types import SimpleNamespace
import pytest
from bike_sim.validation.plant_prefix import ValidPrefix


def sample(t, model=True, numerical=True):
    return SimpleNamespace(time_s=t, end_time_s=t + 0.1, channels={
        'model_status': {'model_valid': model, 'numerically_valid': numerical},
        'tires': {'front': {'normal_load_n': 100.},
                  'rear': {'normal_load_n': 600.}},
    })


def test_invalid_sample_and_all_following_samples_do_not_enter_valid_metrics():
    p = ValidPrefix()
    assert p.observe(sample(0.))
    assert not p.observe(sample(0.1, model=False))
    assert not p.observe(sample(0.2))
    assert (p.seen, p.accepted) == (3, 1)
    assert p.valid_until_s == pytest.approx(0.1)
    assert p.front_normal_impulse_ns == pytest.approx(10.)
    assert p.first_bad['time_s'] == pytest.approx(0.1)


@pytest.mark.parametrize('numerical', [False, 'not_evaluated', None])
def test_preview_and_missing_audit_are_not_numerically_valid(numerical):
    assert not ValidPrefix().observe(sample(0., numerical=numerical))


def test_bad_interval_is_rejected():
    s = sample(0.)
    s.end_time_s = s.time_s
    with pytest.raises(ValueError, match='interval'):
        ValidPrefix().observe(s)


@pytest.mark.parametrize('dt', [-0.01, float('nan'), float('inf')])
def test_nonpositive_or_nonfinite_interval_is_rejected(dt):
    s = sample(0.)
    s.end_time_s = s.time_s + dt
    with pytest.raises(ValueError, match='interval'):
        ValidPrefix().observe(s)


def test_nonfinite_load_is_rejected():
    s = sample(0.)
    s.channels['tires']['rear']['normal_load_n'] = float('nan')
    with pytest.raises(ValueError, match='normal load'):
        ValidPrefix().observe(s)


@pytest.mark.parametrize('bad_load', [-1.0, float('inf'), float('-inf')])
def test_negative_or_infinite_load_is_rejected(bad_load):
    s = sample(0.)
    s.channels['tires']['front']['normal_load_n'] = bad_load
    with pytest.raises(ValueError, match='normal load'):
        ValidPrefix().observe(s)


def test_model_violation_details_preserved_in_first_bad():
    p = ValidPrefix()
    s = sample(0.5, model=False, numerical=False)
    s.channels['model_status']['first_model_violation'] = {'code': 'kinematic_infeasible'}
    assert not p.observe(s)
    assert p.first_bad == {
        'time_s': 0.5,
        'reasons': ['model_violation', 'numerical_quality'],
        'model_event': {'code': 'kinematic_infeasible'},
    }
