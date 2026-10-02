import numpy as np
import pytest

from bike_sim.validation.plant_release import release_decision, trace_integral


def evidence():
    return dict(model_valid=True, numerically_valid=True, contact_passed=True,
                convergence_passed=True, rider_budget_passed=True,
                mass_momentum_passed=True, transmission_parity_passed=True,
                sensor_split_passed=True, finished=True)


def test_finish_with_invalid_physics_fails():
    e = evidence()
    e['model_valid'] = False
    assert not release_decision(e, require_completion=True)['sil_ready']


def test_unfinished_valid_stress_case_is_not_automatically_a_plant_failure():
    e = evidence()
    e['finished'] = False
    assert release_decision(e, require_completion=False)['sil_ready']
    assert not release_decision(e, require_completion=True)['sil_ready']


def test_missing_or_preview_evidence_cannot_self_certify():
    assert not release_decision({}, require_completion=False)['sil_ready']
    e = evidence()
    e['numerically_valid'] = 'not_evaluated'
    assert not release_decision(e, require_completion=True)['sil_ready']


def test_release_decision_full_contract():
    e = evidence()
    res = release_decision(e, require_completion=True)
    assert res['sil_ready'] is True
    assert res['calibration_status'] == 'unvalidated'
    assert res['real_world_safety_claim'] is False
    assert res['checks']['model_valid'] is True
    assert res['checks']['finished'] is True

    # Missing one required key fails closed
    e_missing = evidence()
    del e_missing['rider_budget_passed']
    assert not release_decision(e_missing, require_completion=False)['sil_ready']
    assert release_decision(e_missing, require_completion=False)['checks']['rider_budget_passed'] is False


def test_trace_integral_accurate_on_known_functions():
    # Constant y = 3 over [0, 4] -> integral = 12.0
    t = np.array([0.0, 1.0, 2.5, 4.0])
    y = np.array([3.0, 3.0, 3.0, 3.0])
    assert trace_integral(t, y) == pytest.approx(12.0)

    # Linear y = 2 * t over [0, 3] -> integral = 9.0
    t_lin = np.linspace(0.0, 3.0, 31)
    y_lin = 2.0 * t_lin
    assert trace_integral(t_lin, y_lin) == pytest.approx(9.0)


def test_trace_integral_validation_errors():
    # Fewer than 2 samples
    with pytest.raises(ValueError, match='paired scalar samples'):
        trace_integral([1.0], [2.0])

    # Shape mismatch
    with pytest.raises(ValueError, match='paired scalar samples'):
        trace_integral([1.0, 2.0], [2.0])

    # 2D array
    with pytest.raises(ValueError, match='paired scalar samples'):
        trace_integral([[1.0, 2.0], [3.0, 4.0]], [[1.0, 2.0], [3.0, 4.0]])

    # Non-strictly-increasing times
    with pytest.raises(ValueError, match='strictly increasing'):
        trace_integral([0.0, 0.0], [1.0, 2.0])

    with pytest.raises(ValueError, match='strictly increasing'):
        trace_integral([1.0, 0.5], [1.0, 2.0])

    # Non-finite values
    with pytest.raises(ValueError, match='strictly increasing'):
        trace_integral([0.0, np.nan], [1.0, 2.0])

    with pytest.raises(ValueError, match='strictly increasing'):
        trace_integral([0.0, 1.0], [1.0, np.inf])
