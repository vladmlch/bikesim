import pytest
from bike_sim.validation.rider_work import rider_work_ledger


def test_active_and_passive_joint_work_are_not_conflated_with_motor():
    r = rider_work_ledger({'act_rider_knee_front': 100.,
                          'act_rider_knee_rear': 80.,
                          'rider_passive_damping': -150., 'mid_drive': 200.})
    assert r == {'active_joint_work_j': 180., 'passive_joint_work_j': -150.,
                 'net_joint_work_j': 30., 'motor_mechanical_work_j': 200.}


def test_nonfinite_work_is_rejected():
    with pytest.raises(ValueError):
        rider_work_ledger({'act_rider_knee_front': float('nan')})
