import pytest
from bike_sim.validation.grade_feasibility import necessary_traction_margin


def test_slide_friction_cannot_support_45_percent_even_with_all_load_rear():
    assert necessary_traction_margin(.4, 1., .45) == pytest.approx(-.05)


def test_peak_friction_needs_ninety_percent_rear_load_before_losses():
    assert necessary_traction_margin(.5, .9, .45) == pytest.approx(0.)
    assert necessary_traction_margin(.5, .85, .45) < 0.


@pytest.mark.parametrize('fraction', [-.1, 1.1])
def test_invalid_load_fraction_is_not_accepted(fraction):
    with pytest.raises(ValueError):
        necessary_traction_margin(.5, fraction, .15)
