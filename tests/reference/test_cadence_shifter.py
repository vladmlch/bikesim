"""CadenceShifter decides on the crank cadence alone; the motor is never cut by a shift."""
import pytest

from bike_sim.physics.chain import DrivetrainSpecs
from bike_sim.physics.physical_config import ShiftingConfig
from bike_sim.physics.shifting import CadenceShifter

CASSETTE = (10, 12, 14, 16, 18, 21, 24, 28, 33, 39, 45, 51)


def _shifter(rear=28, **kw):
    cfg = ShiftingConfig(enabled=True, cassette=CASSETTE, target_cadence_min_rpm=75.,
                         target_cadence_max_rpm=110., cadence_smoothing_tau_s=0., **kw)
    return CadenceShifter(DrivetrainSpecs(front_teeth=22, rear_teeth=rear), cfg)


def test_low_crank_cadence_downshifts_to_the_next_bigger_cog():
    s = _shifter(28)
    assert s.update(60., 60., .01)
    assert s.rear_teeth == 33 and s.direction == 'down'


def test_a_wheel_implied_spike_alone_never_upshifts():
    # Regression: the old decision used max(crank, wheel-implied); a slipping or
    # bouncing wheel then upshifted a rider grinding at 85 rpm.
    s = _shifter(28)
    for _ in range(50):
        assert not s.update(85., 140., .01)
    assert s.rear_teeth == 28


def test_high_crank_cadence_upshifts_when_the_landing_stays_in_band():
    s = _shifter(28)
    assert s.update(115., 115., .01)
    assert s.rear_teeth == 24 and s.direction == 'up'


def test_upshifts_stop_at_the_smallest_cog():
    s = _shifter(51)
    # 112 rpm on 51T lands at 112*45/51 = 98.8 -> allowed; on 10T -> 12T impossible (smallest)
    assert s.update(112., 112., .01) and s.rear_teeth == 45
    s = _shifter(12)
    # 112 on 12T -> 10T lands at 93 (in band) -> allowed; then no smaller cog
    assert s.update(112., 112., .01) and s.rear_teeth == 10
    s.cooldown_s = 0.
    assert not s.update(130., 130., .01)


def test_cooldown_and_not_pedalling_block_shifts():
    s = _shifter(28)
    assert s.update(60., 60., .01)
    assert not s.update(60., 60., .01)          # cooldown
    s = _shifter(28)
    assert not s.update(60., 60., .01, pedaling=False)
    assert not s.update(60., 60., .01, braking=True)
    assert not s.update(60., -5., .01)          # rollback


def test_wheelspin_blocks_an_upshift_but_not_a_downshift():
    s = _shifter(28)
    assert not s.update(115., 115., .01, rear_slip_mps=.8)
    assert s.update(60., 60., .01, rear_slip_mps=.8)


def test_ema_filters_a_single_stroke_spike():
    cfg = ShiftingConfig(enabled=True, cassette=CASSETTE, target_cadence_min_rpm=75.,
                         target_cadence_max_rpm=110., cadence_smoothing_tau_s=.35)
    s = CadenceShifter(DrivetrainSpecs(front_teeth=22, rear_teeth=28), cfg)
    s.update(90., 90., .01)
    for _ in range(5):
        assert not s.update(130., 90., .01)


def test_upshift_that_lands_below_the_band_is_rejected():
    s = _shifter(28)
    assert not s.update(115., 70., .01)
    assert s.rear_teeth == 28


def test_downshift_that_lands_above_the_band_is_rejected():
    s = _shifter(28)
    assert not s.update(60., 110., .01)
    assert s.rear_teeth == 28
