# Exercises the mapping helper only; no physics needed.
from bike_sim.sim.research.environment import crash_reason
from bike_sim.sim.ride.virtual_rider import CrashEvent


def test_pitch_over_backward_is_loop_out():
    # root_pitch qpos is negative when nose-up (see wheelie.py pitch_up = -qpos)
    ev = CrashEvent(cause='pitch_over', time_s=1., position_m=5., pitch_rad=-1.2)
    assert crash_reason(ev) == 'crash:loop_out'


def test_pitch_over_forward_is_endo():
    ev = CrashEvent(cause='pitch_over', time_s=1., position_m=5., pitch_rad=+1.2)
    assert crash_reason(ev) == 'crash:endo'


def test_contact_cause_passthrough():
    ev = CrashEvent(cause='rider_ground_contact', time_s=1., position_m=5., pitch_rad=0.)
    assert crash_reason(ev) == 'crash:rider_ground_contact'
