import numpy as np
import pytest

from bike_sim.sim.ride.pedal_recovery import PedalRecovery


ORIGIN = np.zeros(3)
ROTATION = np.eye(3)
HALF = np.array([.05, .04, .008])


def observe(planner, position, force):
    planner.observe(ORIGIN, ROTATION, HALF, np.array(position), np.array(force))


def test_downward_underside_contact_first_releases_the_foot_away_from_surface():
    planner = PedalRecovery(.025, .02, .01)
    observe(planner, [0., 0., -.04], [0., 0., -100.])
    assert planner.stage == 'release'
    goal = planner.goal(ORIGIN, ROTATION, HALF)
    assert goal[2] == pytest.approx(-.058)
    assert goal[2] < -.04


def test_recovery_goes_around_finite_platform_before_returning_above_it():
    planner = PedalRecovery(.025, .02, .01)
    observe(planner, [0., 0., -.04], [0., 0., -100.])
    observe(planner, [0., 0., -.06], [0., 0., 0.])
    assert planner.stage == 'escape'
    goal = planner.goal(ORIGIN, ROTATION, HALF)
    assert goal[0] > .10
    assert goal[2] < -.04
    observe(planner, [.12, 0., -.06], [0., 0., 0.])
    assert planner.stage == 'raise'
    goal = planner.goal(ORIGIN, ROTATION, HALF)
    assert goal[0] > .10
    assert goal[2] > .008
    observe(planner, [.12, 0., .025], [0., 0., 0.])
    assert planner.stage == 'return'
    goal = planner.goal(ORIGIN, ROTATION, HALF)
    assert goal[0] == 0.
    assert goal[2] > .008
    observe(planner, [0., 0., .025], [0., 0., 0.])
    assert planner.stage == 'none'
    assert planner.goal(ORIGIN, ROTATION, HALF) is None


def test_normal_upper_support_does_not_trigger_a_recovery():
    planner = PedalRecovery(.025, .02, .01)
    observe(planner, [0., 0., .005], [0., 0., 100.])
    assert planner.stage == 'none'


def test_escape_uses_the_nearest_side_and_reset_clears_the_trajectory():
    planner = PedalRecovery(.025, .02, .01)
    observe(planner, [-.02, 0., -.04], [0., 0., -100.])
    observe(planner, [-.02, 0., -.06], [0., 0., 0.])
    assert planner.goal(ORIGIN, ROTATION, HALF)[0] < -.1
    planner.reset()
    assert planner.goal(ORIGIN, ROTATION, HALF) is None
