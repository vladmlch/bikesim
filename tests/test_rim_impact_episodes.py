"""Separate raw threshold crossings from explicitly defined impact episodes."""
from copy import deepcopy
import pytest
from bike_sim.sim.ride.rim_events import count_completed_impact_episodes


def event(start, end, x=20.):
    return {'start_time_s':start,'end_time_s':end,'x_m':x}


def test_grouping_preserves_raw_events_and_uses_elapsed_time_not_step_count():
    events=[event(1.,1.001),event(1.0015,1.009),event(1.0095,1.01)]
    saved=deepcopy(events)
    assert count_completed_impact_episodes(events)==1
    assert count_completed_impact_episodes(events,gap_s=.0001)==3
    assert events==saved


def test_distinct_features_or_later_impacts_stay_separate():
    assert count_completed_impact_episodes([event(1.,1.001),event(1.0015,1.003,20.1)])==2
    assert count_completed_impact_episodes([event(1.,1.001),event(1.003,1.004)])==2
    assert count_completed_impact_episodes([{'x_m':20.},{'x_m':20.}])==2


def test_bad_intervals_cannot_be_hidden_in_an_episode():
    with pytest.raises(ValueError,match='overlap'):
        count_completed_impact_episodes([event(1.,1.01),event(1.009,1.02)])
