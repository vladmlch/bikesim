"""Stall reflex: detect loaded drive without forward progress, not a slow crank."""
from math import cos, pi, sin

import pytest

from bike_sim.physics.pedaling import PedalingPolicy, ProgressStallDetector
from bike_sim.physics.physical_config import PedalingConfig

DT = .00125
# Crank parked on top/bottom dead centre: phase % pi == pi/2.
DEAD_SPOT = 81*pi + .5*pi


def _rocking(t, amplitude=.26, peak_rate=4.6):
    """Phase and rate of a crank rocking about the dead spot, as in the 35 % wall stall."""
    omega = peak_rate/amplitude
    return DEAD_SPOT + amplitude*sin(omega*t), amplitude*omega*cos(omega*t)


def _run(policy, motion, duration_s, wheel_rpm=0.):
    states = []
    for i in range(round(duration_s/DT)):
        phase, rate = motion(i*DT)
        states.append((i*DT, policy.update(phase, rate, wheel_rpm, 20., DT)))
    return states


def test_rocking_dead_spot_stall_triggers_reposition():
    """Rocking at up to 4.6 rad/s with no net advance is a stall; the old |rate| test never fired."""
    policy = PedalingPolicy(PedalingConfig(reposition_on_stall=True))
    states = _run(policy, _rocking, 2.)
    reflex = [t for t, s in states if s.mode == 'reposition' and s.reason == 'stall_reflex']
    assert reflex, 'rocking stall never triggered the reposition reflex'
    assert reflex[0] <= PedalingConfig().reposition_stall_window_s + .05


def test_static_dead_spot_stall_still_triggers():
    """A crank frozen on the dead spot remains a stall under the progress test."""
    policy = PedalingPolicy(PedalingConfig(reposition_on_stall=True))
    states = _run(policy, lambda t: (DEAD_SPOT, 0.), 1.5)
    assert any(s.mode == 'reposition' for _, s in states)


def test_slow_forward_grind_is_not_a_stall():
    """Net crank advance of ~1 rad/s (under 10 rpm) is progress, so no reflex."""
    policy = PedalingPolicy(PedalingConfig(reposition_on_stall=True))
    states = _run(policy, lambda t: (DEAD_SPOT + t, 1.), 5., wheel_rpm=9.5)
    assert all(s.mode != 'reposition' for _, s in states)


def test_wheel_progress_alone_prevents_stall():
    """A rolling wheel with a parked crank (e.g. coasting on) is not a dead-spot stall."""
    detector = ProgressStallDetector(1., .5, .5)
    fired = [detector.update(0., 1., DT, loaded=True) for _ in range(round(3./DT))]
    assert not any(fired)


def test_rollback_is_not_progress():
    """Backward wheel and crank motion accumulate as no progress and fire after the window."""
    detector = ProgressStallDetector(1., .5, .5)
    fired = [detector.update(-.8, -2., DT, loaded=True) for _ in range(round(1.1/DT))]
    assert sum(fired) == 1


def test_unloaded_drive_restarts_the_window():
    """Dropping effort resets the detector, so a stall needs a full loaded window."""
    detector = ProgressStallDetector(1., .5, .5)
    for _ in range(round(.9/DT)):
        assert not detector.update(0., 0., DT, loaded=True)
    detector.update(0., 0., DT, loaded=False)
    fired = [detector.update(0., 0., DT, loaded=True) for _ in range(round(.9/DT))]
    assert not any(fired)


@pytest.mark.parametrize('field,value', [
    ('reposition_stall_window_s', 0.),
    ('reposition_stall_progress_rad', 0.),
    ('reposition_stall_progress_rad', pi),
])
def test_invalid_stall_thresholds_are_rejected(field, value):
    """Window and progress must be positive, and progress below half a crank turn."""
    with pytest.raises(ValueError):
        PedalingConfig(**{field: value})
