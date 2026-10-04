"""PedalingPolicy after the reposition removal: a stall is an outcome, not a manoeuvre."""
import math

import pytest

from bike_sim.physics.physical_config import PedalingConfig
from bike_sim.physics.pedaling import PedalingPolicy, PedalingState


def test_reposition_knobs_and_mode_are_gone():
    with pytest.raises(TypeError):
        PedalingConfig(reposition_on_stall=True)
    assert 'reposition' not in ' '.join(PedalingConfig.__dataclass_fields__)
    policy = PedalingPolicy(PedalingConfig(enabled=True))
    with pytest.raises(TypeError):
        policy.update(0., 0., 0., 30., .001, reposition=True)


def test_a_rocking_loaded_crank_simply_keeps_pedalling():
    policy = PedalingPolicy(PedalingConfig(enabled=True, effort_slew_nm_s=0.))
    modes = set()
    for step in range(3000):
        rate = 3.*math.sin(step*.02)      # rocks around a dead spot for 3 s
        state = policy.update(math.pi/2, rate, 0., 40., .001)
        modes.add(state.mode)
    assert modes == {'pedaling'}


def test_ride_control_has_no_reposition_field():
    from bike_sim.sim.ride.control import RideControl
    assert 'crank_reposition' not in RideControl.__dataclass_fields__


def test_rider_program_rejects_the_removed_reposition_command():
    from bike_sim.sim.research.rider_program import RiderProgram
    with pytest.raises(ValueError, match='unknown or malformed rider keyframe'):
        RiderProgram.from_dict({'keyframes': [{'time_s': 0., 'crank_reposition': True}]})


def test_coast_decision_filters_a_single_stroke_spike_with_ema():
    cfg = PedalingConfig(enabled=True, coast_above_rpm=120., resume_below_rpm=105.,
                         coast_cadence_tau_s=.35, effort_slew_nm_s=0.)
    policy = PedalingPolicy(cfg)
    rpm = 2*math.pi/60
    for _ in range(100):
        assert policy.update(0., 90.*rpm, 90., 30., .001).mode == 'pedaling'
    for _ in range(50):                                   # 50 ms spike to 160 rpm
        assert policy.update(0., 160.*rpm, 90., 30., .001).mode == 'pedaling'
    for _ in range(1500):                                 # sustained 160 rpm
        state = policy.update(0., 160.*rpm, 160., 30., .001)
    assert state.mode == 'coasting' and state.reason == 'cadence'
    for _ in range(1500):                                 # back below resume
        state = policy.update(0., 95.*rpm, 95., 30., .001)
    assert state.mode == 'pedaling'


def test_zero_tau_keeps_the_raw_hysteresis():
    cfg = PedalingConfig(enabled=True, coast_above_rpm=120., resume_below_rpm=105.,
                         coast_cadence_tau_s=0., effort_slew_nm_s=0.)
    policy = PedalingPolicy(cfg)
    rpm = 2*math.pi/60
    assert policy.update(0., 90.*rpm, 90., 30., .001).mode == 'pedaling'
    assert policy.update(0., 121.*rpm, 90., 30., .001).mode == 'coasting'


def test_coast_tau_must_be_nonnegative():
    with pytest.raises(ValueError):
        PedalingConfig(enabled=True, coast_cadence_tau_s=-.1)
