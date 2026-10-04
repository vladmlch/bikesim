"""Directional virtual work, preload and positive-work muscle budgets."""
import math

import numpy as np
import pytest

from bike_sim.sim.ride.crank_split import (leg_crank_targets, leg_shares,
    preload_torque_nm, scale_to_power_budget, split_joint_torques)


def test_split_delivers_requested_torque_within_capacity():
    th, tk = split_joint_torques(40., (-.6, .9), lambda j,s: {'hip':180.,'knee':140.}[j])
    assert th*(-.6)+tk*.9 == pytest.approx(40.)


def test_split_prefers_stronger_joint():
    th, tk = split_joint_torques(30., (1.,1.), lambda j,s: 200. if j=='hip' else 50.)
    assert th > tk > 0.
    assert th/tk == pytest.approx(16.)


def test_split_uses_directional_capacity_and_clips():
    th, tk = split_joint_torques(1000., (-1.,1.), lambda j,s: 100. if s>0 else 50.)
    assert th == -50.
    assert tk == 100.


def test_singular_and_zero_capacity_deliver_zero():
    assert split_joint_torques(20., (0.,0.), lambda j,s: 100.) == (0.,0.)
    assert split_joint_torques(20., (1.,1.), lambda j,s: 0.) == (0.,0.)


@pytest.mark.parametrize('phi', [0., .3, 1.2, math.pi/2, 2., math.pi, 4.])
def test_shares_sum_to_one(phi):
    assert sum(leg_shares(phi).values()) == pytest.approx(1.)


def test_vertical_handover_and_opposite_power_strokes():
    assert leg_shares(math.pi/2) == pytest.approx({'front':.5,'rear':.5})
    assert leg_shares(0.) == {'front':1.,'rear':0.}
    assert leg_shares(math.pi) == {'front':0.,'rear':1.}


def test_preload_is_downward_virtual_work_in_native_plus_y():
    assert preload_torque_nm(40., np.array([.17,0.]), np.zeros(2)) == pytest.approx(6.8)
    assert preload_torque_nm(40., np.array([-.17,0.]), np.zeros(2)) == pytest.approx(-6.8)
    targets = leg_crank_targets(60., 0., {'front':np.array([.17,0.]),'rear':np.array([-.17,0.])}, np.zeros(2), 40.)
    assert targets == pytest.approx({'front':66.8,'rear':-6.8})


def test_handover_splits_without_preload():
    targets = leg_crank_targets(60., math.pi/2, {'front':np.array([0.,-.17]),'rear':np.array([0.,.17])}, np.zeros(2), 40.)
    assert targets == {'front':30.,'rear':30.}


def test_power_budget_scales_positive_work_only():
    torques = {'a':100.,'b':100.,'eccentric':-100.,'still':20.}
    out = scale_to_power_budget(torques, {'a':4.,'b':4.,'eccentric':4.,'still':0.}, 250.,450.)
    assert out == {'a':56.25,'b':56.25,'eccentric':-100.,'still':20.}
    assert torques['a'] == 100.
