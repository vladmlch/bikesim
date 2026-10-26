"""Topology normalization and gate budgets belong to their configuration."""
import pytest

from bike_sim.physics.physical_config import ArticulatedConfig


def test_spindle_and_connect_are_valid_attachments():
    cfg = ArticulatedConfig(pedal_attachment='spindle', saddle_attachment='pin', grip_attachment='connect')
    assert cfg.pedal_attachment == 'spindle'
    assert cfg.grip_attachment == 'connect'


def test_grip_weld_alias_normalises_to_connect():
    assert ArticulatedConfig(grip_attachment='weld').grip_attachment == 'connect'


def test_budget_comes_from_config_fields():
    from bike_sim.physics.attachment_budget import AttachmentBudget
    cfg = ArticulatedConfig(pedal_min_normal_n=25., foot_mu=.8, saddle_mu=.5,
                            grip_pull_per_hand_n=350., link_max_gap_m=.004)
    assert cfg.attachment_budget() == AttachmentBudget(25., .8, .5, 350., .004)


def test_delivered_power_budgets_fall_back_to_capability_caps():
    # Unset: the monitor verdict reads the capability bound bitwise.
    cfg = ArticulatedConfig(active_positive_power_limit_w=450.)
    assert cfg.delivered_positive_budget_w == 450.
    assert cfg.delivered_joint_budget_w == cfg.joint_power_limit_w
    assert ArticulatedConfig().delivered_positive_budget_w is None
    # Set: the monitor re-bands the delivered envelope while the command
    # cap — plant physics — is untouched.
    cfg = ArticulatedConfig(active_positive_power_limit_w=450.,
                            delivered_positive_power_budget_w=750.,
                            delivered_joint_power_budget_w=375.)
    assert cfg.delivered_positive_budget_w == 750.
    assert cfg.delivered_joint_budget_w == 375.
    assert cfg.active_positive_power_limit_w == 450.
    assert cfg.joint_power_limit_w == 250.


@pytest.mark.parametrize('field', ('delivered_positive_power_budget_w',
                                   'delivered_joint_power_budget_w'))
@pytest.mark.parametrize('value', (-1., float('nan'), float('inf')))
def test_delivered_power_budgets_validate_like_other_budgets(field, value):
    with pytest.raises(ValueError):
        ArticulatedConfig(**{field: value})


@pytest.mark.parametrize('field', ('weld_foot_min_normal_n',
                                   'weld_foot_max_normal_n',
                                   'weld_saddle_min_normal_n',
                                   'weld_saddle_max_normal_n'))
@pytest.mark.parametrize('value', (float('nan'), float('inf'),
                                   float('-inf'), 'bolts'))
def test_weld_normal_bounds_must_be_finite_when_set(field, value):
    # Tension retention makes negative bounds legal, so the post-init gate
    # is the wire's optional_real one: None or a finite real, nothing else.
    with pytest.raises(ValueError):
        ArticulatedConfig(**{field: value})


@pytest.mark.parametrize('field', ('weld_foot_min_normal_n',
                                   'weld_foot_max_normal_n',
                                   'weld_saddle_min_normal_n',
                                   'weld_saddle_max_normal_n'))
def test_weld_normal_bounds_accept_tension_retention(field):
    cfg = ArticulatedConfig(**{field: -1050.})
    assert getattr(cfg, field) == -1050.


@pytest.mark.parametrize('field', ('weld_foot_shear_n', 'weld_saddle_shear_n'))
@pytest.mark.parametrize('value', (-1., float('nan')))
def test_weld_shear_capacity_is_nonnegative(field, value):
    with pytest.raises(ValueError):
        ArticulatedConfig(**{field: value})


def test_balance_defaults():
    cfg = ArticulatedConfig()
    assert (cfg.balance_floor_kmh, cfg.balance_dwell_s, cfg.balance_grace_s) == (4., .5, 3.)


@pytest.mark.parametrize('field', ['balance_floor_kmh', 'balance_dwell_s'])
@pytest.mark.parametrize('value', [-1., 0., float('nan')])
def test_invalid_balance_thresholds_are_rejected(field, value):
    with pytest.raises(ValueError):
        ArticulatedConfig(**{field: value})


def test_surge_and_trim_are_validated_in_climb_config():
    from bike_sim.physics.seated_climb import SeatedClimbConfig
    cfg = SeatedClimbConfig()
    assert (cfg.surge_power_w, cfg.surge_grade, cfg.surge_budget_s, cfg.surge_recovery_rate) == (400., .2, 15., 1/3)
    for field in ('surge_power_w', 'surge_budget_s'):
        with pytest.raises(ValueError):
            SeatedClimbConfig(**{field: 0.})
    for field in ('surge_grade', 'surge_recovery_rate', 'front_load_share_target', 'lean_trim_gain_rad_s', 'lean_trim_limit_rad'):
        with pytest.raises(ValueError):
            SeatedClimbConfig(**{field: -1.})


@pytest.mark.parametrize('value', [-1., float('nan'), float('inf')])
def test_balance_grace_must_be_finite_and_nonnegative(value):
    with pytest.raises(ValueError):
        ArticulatedConfig(balance_grace_s=value)
