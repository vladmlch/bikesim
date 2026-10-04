"""Low speed latches after grace and dwell; braking never contributes dwell."""
import pytest

from bike_sim.sim.ride.balance_monitor import BalanceMonitor


def _run(speeds):
    monitor = BalanceMonitor(4/3.6,.5,3.)
    for i,speed in enumerate(speeds):
        monitor.update(i*.1,i*.1*speed,speed,riding=True)
    return monitor


def test_slow_start_within_grace_is_fine():
    assert _run([0.]*29).event is None


def test_sustained_low_speed_latches_after_half_second_from_first_low_sample():
    monitor = _run([5/3.6]*31+[3/3.6]*6)
    event = monitor.event
    assert event is not None
    assert event.time_s == pytest.approx(3.6)
    assert event.position_m == pytest.approx(3.)
    assert monitor.update(10.,50.,10/3.6,riding=True) is event


def test_brief_dip_recovers_and_resets_dwell():
    monitor = _run([5/3.6]*31+[3/3.6]*4+[5/3.6]*5)
    assert monitor.event is None
    assert monitor.low_speed_s == 0.


def test_grace_starts_only_when_riding():
    monitor = BalanceMonitor(4/3.6,.5,3.)
    for i in range(50):
        assert monitor.update(i*.1,0.,0.,riding=False) is None
    for i in range(29):
        assert monitor.update(5+i*.1,0.,0.,riding=True) is None


def test_braking_pauses_dwell_without_restarting_grace():
    monitor = BalanceMonitor(4/3.6,.5,3.)
    monitor.update(0.,0.,5/3.6,riding=True)
    monitor.update(3.1,0.,0.,riding=True)
    monitor.update(3.4,0.,0.,riding=True)
    assert monitor.low_speed_s == pytest.approx(.3)
    monitor.update(3.5,0.,0.,riding=False)
    assert monitor.low_speed_s == 0.
    monitor.update(8.,0.,0.,riding=False)
    assert monitor.grace_left_s == 0.
    assert monitor.update(8.1,0.,0.,riding=True) is None
    assert monitor.update(8.5,0.,0.,riding=True) is None
    assert monitor.update(8.6,0.,0.,riding=True) is monitor.event
    assert monitor.event.time_s == pytest.approx(8.6)


def test_reset_clears_event_and_restarts_first_ride_grace():
    monitor = _run([0.]*37)
    assert monitor.event is not None
    monitor.reset()
    assert monitor.event is None
    assert monitor.low_speed_s == 0.
    assert monitor.update(100.,0.,0.,riding=True) is None
    assert monitor.grace_left_s == 3.


def test_floor_boundary_and_time_validation():
    monitor = BalanceMonitor(1.,.5,0.)
    assert monitor.update(0.,0.,1.,riding=True) is None
    assert monitor.update(1.,0.,1.,riding=True) is None
    with pytest.raises(ValueError):
        monitor.update(.5,0.,0.,riding=True)


def test_summary_and_research_metrics_keep_latched_event_position():
    from types import SimpleNamespace
    from bike_sim.sim.ride.physical_session import physical_summary
    from bike_sim.sim.research.metrics import episode_metrics
    monitor = BalanceMonitor(1.,.5,0.)
    monitor.update(0.,10.,0.,riding=True)
    monitor.update(.5,12.,0.,riding=True)
    runtime = SimpleNamespace(balance_monitor=monitor,
        reference_monitor=SimpleNamespace(first_failure=None),energy={},
        model_status=SimpleNamespace(as_dict=lambda:{}),
        drive=SimpleNamespace(battery=SimpleNamespace(energy_j=0.)),
        history=SimpleNamespace(work_j={},airtime_s={},duration_s=1.))
    sim = SimpleNamespace(physical=runtime,time_s=1.,position_m=15.,steps=800,
        crash=None,equilibrium={})
    summary = physical_summary(sim,{},'duration_reached')
    assert summary['outcome']['balance_lost']
    assert summary['outcome']['balance_lost_at_m'] == 12.
    env = SimpleNamespace(sim=sim,tracker=SimpleNamespace(metrics={}),
        start_position_m=10.,reason='duration',torque_requested_nms=0.,
        torque_delivered_nms=0.,demand_integral_nms=None,run_metadata={},
        max_shock_stroke_m=0.,max_fork_travel_m=0.,numerically_valid=True,
        max_energy_residual_ratio=0.)
    metrics = episode_metrics(env)
    assert metrics['balance_lost']
    assert metrics['balance_lost_at_m'] == 12.
