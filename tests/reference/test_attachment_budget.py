from bike_sim.physics.physical_config import ArticulatedConfig
BUDGET = ArticulatedConfig().attachment_budget()
import pytest
from bike_sim.physics.attachment_budget import (
    AttachmentSample, attachment_violations)


def test_violations_use_the_supplied_budget():
    from bike_sim.physics.attachment_budget import AttachmentBudget
    sample = AttachmentSample('foot', 19., 0., 0., 0.)
    assert 'normal' in attachment_violations(sample, AttachmentBudget(20., .9, .6, 300., .005))
    assert 'normal' not in attachment_violations(sample, AttachmentBudget(10., .9, .6, 300., .005))


def test_platform_cannot_pull_or_scrape_without_normal():
    bad = AttachmentSample('foot', -1.0, 10.0, 0.0, 0.0)
    assert 'normal' in attachment_violations(bad, BUDGET)
    good = AttachmentSample('foot', 20.0, 18.0, 0.0, 0.005)
    assert attachment_violations(good, BUDGET) == ()


def test_a_weld_cannot_supply_a_free_unlimited_couple():
    sample = AttachmentSample('foot', 100.0, 0.0, 10.0, 0.0,
                              half_patch_m=0.025)
    assert 'cop' in attachment_violations(sample, BUDGET)


def test_per_hand_pull_and_saddle_friction():
    assert 'pull' in attachment_violations(
        AttachmentSample('grip', 0, 0, 0, 0, pull_n=300.01), BUDGET)
    assert 'friction' in attachment_violations(
        AttachmentSample('saddle', 100, 60.01, 0, 0), BUDGET)
    assert 'normal' in attachment_violations(
        AttachmentSample('saddle', 0, 0, 0, 0), BUDGET)


@pytest.mark.parametrize('value', (float('nan'), float('inf'), -float('inf')))
def test_nonfinite_measurements_are_rejected(value):
    violations = attachment_violations(
        AttachmentSample('foot', value, 0., 0., 0.), BUDGET)
    assert violations == ('nonfinite',)


@pytest.mark.parametrize('normal,tangent,moment,gap,pull,patch,expected', [
    (19.99, 0., 0., 0., 0., 0., ('normal',)),
    (20.0, 0., 0., 0.005001, 0., 0., ('gap',)),
    (100., 90.01, 0., 0., 0., 0., ('friction',)),
    (100., -90.01, 0., 0., 0., 0., ('friction',)),
    (100., 89.99, 0., 0., 0., 0., ()),
    (100., -89.99, 0., 0., 0., 0., ()),
    (100., 0., 2.501, 0., 0., 0.025, ('cop',)),
    (100., 0., -2.501, 0., 0., 0.025, ('cop',)),
    (100., 0., 2.499, 0., 0., 0.025, ()),
])
def test_foot_boundaries(normal, tangent, moment, gap, pull, patch, expected):
    sample = AttachmentSample('foot', normal, tangent, moment, gap,
                              pull_n=pull, half_patch_m=patch)
    assert attachment_violations(sample, BUDGET) == expected


def test_saddle_pin_reports_no_couple_channel():
    # A pin cannot transmit a free couple: moment must be measured as zero,
    # and zero patch means any recovered moment is a measurement error only
    # at nonzero normal. With normal load a pin moment is a topology defect.
    sample = AttachmentSample('saddle', 100., 0., 5., 0.)
    assert 'cop' in attachment_violations(sample, BUDGET)


def test_unknown_attachment_kind_is_an_error():
    with pytest.raises(ValueError):
        attachment_violations(AttachmentSample('elbow', 0., 0., 0., 0.), BUDGET)


def test_negative_measurement_fields_are_rejected():
    assert attachment_violations(
        AttachmentSample('foot', 100., 0., 0., -0.001), BUDGET) == ('invalid_measurement',)
    assert attachment_violations(
        AttachmentSample('foot', 100., 0., 0., 0., half_patch_m=-0.01), BUDGET) == (
            'invalid_measurement',)
