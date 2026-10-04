"""Finite rider surge and delayed posture load feedback."""
import math
import pytest
from bike_sim.physics.seated_climb import SeatedClimbConfig, SeatedClimbPolicy


def test_surge_budget_depletes_and_recovers():
    policy = SeatedClimbPolicy(SeatedClimbConfig(enabled=True, target_crank_power_w=250.))
    for _ in range(150):
        assert policy.power_target_w(.25, .1) == 400.
    assert policy.power_target_w(.25, .1) == 250.
    for _ in range(20):
        assert policy.power_target_w(.25, .1) == 250.
    for _ in range(450):
        assert policy.power_target_w(0., .1) == 250.
    assert policy.power_target_w(.25, .1) == 400.


def test_below_threshold_uses_sustained_power():
    policy = SeatedClimbPolicy(SeatedClimbConfig(enabled=True, target_crank_power_w=250.))
    assert policy.power_target_w(.15, .1) == 250.


def test_front_share_is_posture_only_and_unavailable_when_unloaded():
    from bike_sim.sim.ride.rider_intent import signals_from_channels
    base = {'frame_gyro_body_rad_s': (0.,0.,0.),
            'frame_specific_force_body_mps2': (0.,0.,0.),
            'encoders_rad_s': {'crank': 0.}, 'human_torque_nm': 0.}
    base['tires'] = {'front': {'normal_load_n': 200.}, 'rear': {'normal_load_n': 800.}}
    assert signals_from_channels(base).front_load_share == .2
    base['tires']['front']['normal_load_n'] = 0.
    base['tires']['rear']['normal_load_n'] = 0.
    assert signals_from_channels(base).front_load_share is None


def test_surge_reset_restores_budget_and_rejects_nonfinite_inputs():
    policy = SeatedClimbPolicy(SeatedClimbConfig(enabled=True))
    policy.power_target_w(.25, 15.)
    assert policy.power_target_w(.25, .1) == policy.config.target_crank_power_w
    policy.reset()
    assert policy.power_target_w(.25, .1) == 400.
    for grade, dt in ((math.nan, .1), (.25, 0.), (.25, math.inf)):
        with pytest.raises(ValueError):
            policy.power_target_w(grade, dt)
