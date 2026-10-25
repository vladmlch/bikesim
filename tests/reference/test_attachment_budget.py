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


def _weld_budget(**overrides):
    """The welded-plant envelope declared on the pinned reference profile —
    i.e. ArticulatedConfig's weld_* fields resolved by attachment_budget()."""
    from bike_sim.physics.attachment_budget import AttachmentBudget
    return AttachmentBudget(-1050., .9, .6, 300., .005,
                            saddle_min_normal_n=-2250., foot_max_normal_n=3000.,
                            saddle_max_normal_n=1300., foot_shear_n=2250.,
                            saddle_shear_n=1300., **overrides)


def test_weld_envelope_admits_tension_and_shear_within_capacity():
    budget = _weld_budget()
    # A spindle weld pulling 800 N while carrying 1800 N of shear is inside
    # the declared hardware envelope — the unilateral cone would flag both.
    assert attachment_violations(
        AttachmentSample('foot', -800., 1800., 0., 0.), budget) == ()
    # Past the weld's tension retention and shear capacity it is a defect.
    assert 'normal' in attachment_violations(
        AttachmentSample('foot', -1100., 0., 0., 0.), budget)
    assert 'friction' in attachment_violations(
        AttachmentSample('foot', -800., 2300., 0., 0.), budget)
    # The friction cone still stacks on top of the flat shear capacity:
    # 2250 + .9*100 = 2340 admits 2330 N but not 2350 N.
    assert 'friction' not in attachment_violations(
        AttachmentSample('foot', 100., 2330., 0., 0.), budget)
    assert 'friction' in attachment_violations(
        AttachmentSample('foot', 100., 2350., 0., 0.), budget)


def test_weld_envelope_bounds_compression_and_saddle_tension():
    budget = _weld_budget()
    assert 'normal' in attachment_violations(
        AttachmentSample('foot', 3100., 0., 0., 0.), budget)
    assert attachment_violations(
        AttachmentSample('saddle', -1800., 1000., 0., 0.), budget) == ()
    assert 'normal' in attachment_violations(
        AttachmentSample('saddle', -2300., 0., 0., 0.), budget)
    assert 'normal' in attachment_violations(
        AttachmentSample('saddle', 1400., 0., 0., 0.), budget)


def test_weld_fields_apply_only_to_welded_interfaces():
    # Declared weld bounds on a flat-pad plant are inert: the envelope is a
    # weld-hardware contract, not a way to soften unilateral contacts.
    flat = ArticulatedConfig(weld_foot_min_normal_n=-1050.,
                             weld_foot_shear_n=2250.).attachment_budget()
    assert flat.foot_min_normal_n == 20.
    assert flat.foot_shear_n == 0.
    welded = ArticulatedConfig(pedal_attachment='spindle', saddle_attachment='pin',
                             grip_attachment='connect',
                             weld_foot_min_normal_n=-1050.,
                             weld_foot_shear_n=2250.,
                             weld_saddle_min_normal_n=-2250.).attachment_budget()
    assert welded.foot_min_normal_n == -1050.
    assert welded.foot_shear_n == 2250.
    assert welded.saddle_min_normal_n == -2250.
