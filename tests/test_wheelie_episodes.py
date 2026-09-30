from bike_sim.sim.ride.wheelie import WheelieTracker, WheelieTruth

DT = .000125


def _truth(t, front=100., rear=600., fc=0., rc=0., pitch=0., rate=0.):
    return WheelieTruth(time_s=t, front_load_n=front, rear_load_n=rear,
        front_clearance_m=fc, rear_clearance_m=rc, relative_pitch_rad=pitch,
        pitch_rate_up_rad_s=rate)


def test_episode_opens_and_closes_with_context():
    tr = WheelieTracker()
    t = 0.0
    ctx = {'delivered_motor_nm': 80., 'road_pitch_rad': 0.2}
    for _ in range(10):
        tr.update(_truth(t), DT); t += DT
    # candidate bout: front unloaded, lifted, pitched; 50 ms > 20 ms persistence
    for _ in range(400):
        tr.update(_truth(t, front=0., fc=.05, pitch=.1), DT, context=ctx); t += DT
    for _ in range(10):
        tr.update(_truth(t), DT); t += DT
    assert len(tr.episodes) == 1
    ep = tr.episodes[0]
    assert ep['confirmed'] is True
    assert ep['onset'] == ctx
    assert ep['max_front_clearance_m'] >= .05
    assert ep['end_s'] > ep['start_s']


def test_unconfirmed_candidate_is_recorded_as_unconfirmed():
    tr = WheelieTracker()
    t = 0.0
    for _ in range(40):  # 5 ms < 20 ms persistence
        tr.update(_truth(t, front=0., fc=.05, pitch=.1), DT); t += DT
    tr.update(_truth(t), DT)
    assert len(tr.episodes) == 1
    assert tr.episodes[0]['confirmed'] is False


def test_open_episode_is_reported_without_mutating_tracker():
    tr = WheelieTracker()
    t = 0.0
    for _ in range(400):
        tr.update(_truth(t, front=0., fc=.05, pitch=.1), DT); t += DT
    records = tr.metrics['wheelie_episode_records']
    assert len(records) == 1 and records[0]['confirmed'] is True
    assert tr.episodes == []
    tr.update(_truth(t, front=0., fc=.05, pitch=.1), DT)  # tracker still usable


def test_margin_metrics_tracked():
    tr = WheelieTracker()
    # front 100 of 700 total -> fraction 1/7
    tr.update(_truth(0., front=100., rear=600.), DT)
    tr.update(_truth(DT, front=50., rear=600.), DT)
    m = tr.metrics
    assert m['front_load_fraction_min'] <= 50/650 + 1e-9
    assert m['front_load_fraction_mean'] > 0.


def test_pitch_rate_up_tracked_and_reset():
    tr = WheelieTracker()
    tr.update(_truth(0., rate=1.5), DT)
    assert tr.metrics['max_pitch_rate_up_rad_s'] == 1.5
    tr.reset()
    m = tr.metrics
    assert m['max_pitch_rate_up_rad_s'] == 0.
    assert m['front_load_fraction_min'] is None and m['front_load_fraction_mean'] is None
    assert tr.episodes == []
