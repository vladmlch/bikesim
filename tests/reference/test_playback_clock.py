"""Pure pacing regressions; no simulation, viewer or wall-clock sleeps."""
import math
import pytest
from bike_sim.sim.playback import PlaybackClock, speed_key


def test_budget_yield_preserves_uncompleted_debt():
    clock = PlaybackClock(.00125, scale=2)
    clock.rebase(0., step=0)
    assert clock.target_step(.01, current_step=0) == 16
    assert clock.target_step(.01, current_step=8) == 16
    clock.set_scale(4, now=.01, step=8)
    assert clock.target_step(.02, current_step=8) == 40


def test_catchup_is_capped_and_counted_from_completed_steps():
    clock = PlaybackClock(.00125, scale=8)
    clock.rebase(0., step=10)
    assert clock.target_step(10., current_step=10) == 330
    assert clock.target_step(10., current_step=20) == 330
    clock.rebase(10., step=0)
    assert clock.target_step(10., current_step=0) == 0


def test_fractional_remainder_and_regressing_wall_time():
    clock = PlaybackClock(.01)
    clock.rebase(0., step=0)
    assert clock.target_step(.007, current_step=0) == 0
    assert clock.target_step(.012, current_step=0) == 1
    assert clock.target_step(.009, current_step=1) == 1
    # The regression must not count .009 -> .012 twice.
    assert clock.target_step(.019, current_step=1) == 1
    assert clock.target_step(.022, current_step=1) == 2


def test_pause_resume_and_scale_rebase_discard_backlog():
    clock = PlaybackClock(.00125)
    clock.rebase(0., step=0)
    assert clock.target_step(1., current_step=0) == 40
    clock.set_paused(True, now=1., step=0)
    assert clock.target_step(100., current_step=0) == 0
    clock.set_paused(False, now=100., step=0)
    assert clock.target_step(100., current_step=0) == 0
    clock.set_scale(2, now=101., step=0)
    assert clock.target_step(101., current_step=0) == 0
    clock.rebase(102., step=20)
    with pytest.raises(ValueError, match='backwards'):
        clock.target_step(102., current_step=0)


@pytest.mark.parametrize('dt', [0., -1., math.nan, math.inf, True, '1'])
def test_invalid_timestep(dt):
    with pytest.raises(ValueError):
        PlaybackClock(dt)


@pytest.mark.parametrize('scale', [0, 3, 16, True, 2.0, '2'])
def test_invalid_scale(scale):
    with pytest.raises(ValueError):
        PlaybackClock(.00125, scale=scale)


def test_speed_keys_do_not_reuse_physical_controls():
    assert speed_key(295, 2) == 1
    assert speed_key(296, 2) == 4
    assert speed_key(297, 8) == 1
    assert speed_key(295, 1) == 1
    assert speed_key(296, 8) == 8
    for key in (32, 44, 46, ord('B'), ord('R'), ord('T'), ord('G')):
        assert speed_key(key, 2) is None
