import pytest
from bike_sim.sim.ride.reference_monitor import InvalidReferenceRun, ReferenceMonitor


def test_research_stops_before_decimation_can_hide_bad_step():
    monitor = ReferenceMonitor(strict=True)
    with pytest.raises(InvalidReferenceRun, match='foot_front.normal'):
        monitor.accept(0.00125, ('foot_front.normal',))
    assert monitor.first_failure[0] == 0.00125


def test_viewer_warns_but_never_restores_valid_status():
    monitor = ReferenceMonitor(strict=False)
    with pytest.warns(RuntimeWarning, match='grip_left.pull'):
        monitor.accept(0.01, ('grip_left.pull',))
    monitor.accept(0.02, ())
    assert monitor.first_failure == (0.01, ('grip_left.pull',))


def test_non_strict_warns_once_and_keeps_the_first_failure():
    monitor = ReferenceMonitor(strict=False)
    with pytest.warns(RuntimeWarning, match='foot_rear.friction'):
        monitor.accept(0.01, ('foot_rear.friction',))
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter('error')
        monitor.accept(0.02, ('saddle.normal',))
        monitor.accept(0.03, ())
    assert monitor.first_failure == (0.01, ('foot_rear.friction',))


def test_strict_raises_even_after_a_stored_failure():
    monitor = ReferenceMonitor(strict=True)
    with pytest.raises(InvalidReferenceRun):
        monitor.accept(0.005, ('grip.pull',))
    assert monitor.first_failure == (0.005, ('grip.pull',))
