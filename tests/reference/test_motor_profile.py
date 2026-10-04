"""Declared, immutable motor data and binary pedelec permission."""
from dataclasses import FrozenInstanceError, replace
import math

import pytest

from bike_sim.physics.motor_profile import (
    BOSCH_CX_GEN4, PROFILES, MotorProfile, assist_gain, pedelec_cap,
)


def test_bosch_profile_values_are_the_recalled_unverified_set():
    p = PROFILES['bosch_cx_gen4']
    assert p is BOSCH_CX_GEN4
    assert (p.peak_torque_nm, p.rated_power_w, p.peak_power_w) == (85., 250., 600.)
    assert p.torque_tau_s == .04
    assert p.cutoff_mps == pytest.approx(25/3.6)
    assert p.taper_width_mps == pytest.approx(2/3.6)
    assert p.gate_min_crank_rad_s == pytest.approx(math.radians(5.))
    assert 'unverified' in p.provenance and 'no web lookup' in p.provenance


@pytest.mark.parametrize('mode,human,expected', [
    ('eco', 30., .6), ('tour', 30., 1.4), ('turbo', 30., 3.4), ('turbo', 0., 3.4),
    ('emtb', 0., 1.4), ('emtb', 20., 2.4), ('emtb', 40., 3.4), ('emtb', 80., 3.4),
    ('emtb', -5., 1.4),
])
def test_mode_gains(mode, human, expected):
    assert assist_gain(BOSCH_CX_GEN4, mode, human) == pytest.approx(expected)


def test_unknown_mode_is_rejected():
    with pytest.raises(ValueError):
        assist_gain(BOSCH_CX_GEN4, 'sport', 10.)


def test_profile_is_frozen_and_copies_the_gain_mapping():
    gains = dict(BOSCH_CX_GEN4.mode_gains)
    profile = replace(BOSCH_CX_GEN4, mode_gains=gains)
    gains['turbo'] = 99.
    assert assist_gain(profile, 'turbo', 10.) == 3.4
    with pytest.raises(TypeError):
        profile.mode_gains['turbo'] = 99.
    with pytest.raises(FrozenInstanceError):
        profile.peak_torque_nm = 99.


@pytest.mark.parametrize('changes', [
    {'name': ''}, {'provenance': '  '}, {'mode_gains': {'eco': .5}},
    {'taper_width_mps': 8.}, {'peak_torque_nm': 0.}, {'rated_power_w': 700.},
    {'torque_tau_s': math.nan}, {'gate_min_crank_rad_s': math.inf},
    {'mode_gains': {'eco': (.6, .9), 'tour': 1.4, 'emtb': (1.4, 3.4), 'turbo': 3.4}},
    {'mode_gains': {'eco': .6, 'tour': 1.4, 'emtb': (3.4, 1.4), 'turbo': 3.4}},
    {'mode_gains': {'eco': .6, 'tour': 1.4, 'emtb': 2., 'turbo': 3.4}},
    {'mode_gains': {'eco': math.nan, 'tour': 1.4, 'emtb': (1.4, 3.4), 'turbo': 3.4}},
])
def test_profile_rejects_invalid_data(changes):
    with pytest.raises(ValueError):
        replace(BOSCH_CX_GEN4, **changes)


def test_pedelec_cap_is_a_permission_not_a_throttle():
    gate = math.radians(5.)
    allowed = dict(human_nm=10., crank_rad_s=1., external_cap_nm=None, braking=False,
                   gate_min_crank_rad_s=gate)
    assert pedelec_cap(**allowed) == math.inf
    assert pedelec_cap(**{**allowed, 'external_cap_nm': 30.}) == 30.
    assert pedelec_cap(**{**allowed, 'external_cap_nm': 0.}) == 0.
    assert pedelec_cap(**{**allowed, 'braking': True}) == 0.
    assert pedelec_cap(**{**allowed, 'human_nm': 0.}) == 0.
    assert pedelec_cap(**{**allowed, 'human_nm': -3.}) == 0.
    assert pedelec_cap(**{**allowed, 'crank_rad_s': gate}) == 0.
    assert pedelec_cap(**{**allowed, 'crank_rad_s': -1.}) == 0.
    assert pedelec_cap(**{**allowed, 'crank_rad_s': gate*1.01}) == math.inf


@pytest.mark.parametrize('changes', [
    {'human_nm': math.nan}, {'crank_rad_s': math.inf}, {'gate_min_crank_rad_s': -.1},
    {'external_cap_nm': -1.}, {'external_cap_nm': math.inf}, {'braking': 1},
])
def test_pedelec_cap_rejects_invalid_inputs(changes):
    allowed = dict(human_nm=10., crank_rad_s=1., external_cap_nm=None, braking=False,
                   gate_min_crank_rad_s=.1)
    with pytest.raises(ValueError):
        pedelec_cap(**{**allowed, **changes})


def test_gain_rejects_nonfinite_human_torque():
    with pytest.raises(ValueError):
        assist_gain(BOSCH_CX_GEN4, 'emtb', math.nan)
