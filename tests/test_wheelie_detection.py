import pytest
from bike_sim.sim.ride.wheelie import classify_contact, WheelieTruth, WheelieTracker, quasistatic_front_load


@pytest.mark.parametrize('front,rear,fgap,rgap,pitch,expected', [
    (300., 600., 0., 0., .3, 'two_wheels'),
    (0., 600., 0., 0., .1, 'front_unloaded'),
    (0., 600., .05, 0., 0., 'front_lift'),
    (0., 600., .05, 0., .1, 'wheelie_candidate'),
    (300., 0., 0., .05, 0., 'rear_lift'),
    (0., 0., .1, .1, .3, 'flight'),
    (0., 0., 0., 0., .3, 'unsupported'),
])
def test_distinct_contact_states(front, rear, fgap, rgap, pitch, expected):
    assert classify_contact(front_load_n=front, rear_load_n=rear, front_clearance_m=fgap,
        rear_clearance_m=rgap, relative_pitch_rad=pitch) == expected


def test_wheelie_needs_persistence_and_does_not_continue_in_flight():
    tracker = WheelieTracker(persistence_s=.02)
    for i in range(4):
        state = tracker.update(WheelieTruth(time_s=i*.005, front_clearance_m=.05,
            rear_load_n=600., relative_pitch_rad=.1), .005)
    assert state == 'wheelie'
    assert tracker.metrics['wheelie_episodes'] == 1
    assert tracker.update(WheelieTruth(time_s=.02, front_clearance_m=.1,
        rear_clearance_m=.1, relative_pitch_rad=.2), .005) == 'flight'
    assert tracker.metrics['flight_time_s'] == pytest.approx(.005)
    with pytest.raises(ValueError, match='interval'):
        tracker.update(WheelieTruth(time_s=.02), .005)


def test_quasistatic_load_transfer_has_correct_tipping_threshold():
    assert quasistatic_front_load(100., 1.2, .6, .8, 0., 0.) == pytest.approx(490.5)
    threshold = 9.81*.6/.8
    assert quasistatic_front_load(100., 1.2, .6, .8, 0., threshold) == pytest.approx(0., abs=1e-10)
    assert quasistatic_front_load(100., 1.2, .6, .8, .2, 0.) < 490.5
