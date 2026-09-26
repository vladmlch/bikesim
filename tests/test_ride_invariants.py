"""
Invariants of a headless ride over the default rough road (`road_worn`, 25 km/h).

Heightfield contact is chaotic, so nothing here pins a trace. The assertions are physical
bounds, plus loosely pinned characteristics whose measured values are recorded in the
docstrings (measured 2026-09-26 on MuJoCo 3.x, macOS arm64):

- start equilibrium: fork 73.14 mm (40.6 %), rear wheel 40.89 mm (22.7 %) -- the solved
  figures docs/RIDE.md section 9 reports, not the first-order 42.0 / 25.3 %;
- fork travel 24.9 .. 146.9 mm: works hard, never bottoms out, never tops out;
- shock stroke -4.14 .. 37.14 mm: the soft joint limit lets it pass top-out by ~4 mm;
- speed over the window 22.5 .. 27.1 km/h, mean 25.03;
- both wheels never airborne together, so the virtual rider never fires;
- each wheel carries load on 96-97 % of window steps.

One full traverse (~30 000 steps, ~3 s wall) is shared by the module; a second traverse
checks repeatability.
"""

import numpy as np
import pytest

from bike_sim.sim.ride.metrics import RAMP_EXCLUSION_M, summarize_ride
from bike_sim.sim.ride.recorder import RideRecorder
from bike_sim.sim.ride.termination import REASON_END_OF_TRACK
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset

TARGET_KMH = 25.0
FORK_TRAVEL_MM = 180.0
SHOCK_STROKE_MM = 65.0
FORK_OVERTRAVEL_TOLERANCE_MM = 5.0   # as tests/test_ride_track.py
SHOCK_TOPOUT_TOLERANCE_MM = 7.0      # as tests/test_ride_track.py


def _traverse():
    sim = RideSimulation(track=get_preset("road_worn"), target_speed_kmh=TARGET_KMH)
    rec = RideRecorder(sim)
    rec.record(sim)
    outcome = sim.run(on_step=rec.record)
    return sim, rec, outcome


@pytest.fixture(scope="module")
def ride():
    sim, rec, outcome = _traverse()
    channels = rec.columns()
    window = channels["x_m"] >= sim.start_x_m + RAMP_EXCLUSION_M
    return sim, rec, outcome, channels, window


def test_run_completes_without_crashing(ride):
    sim, _, outcome, _, _ = ride
    assert outcome.reason == REASON_END_OF_TRACK
    assert outcome.crash is None
    assert outcome.position_m >= sim.track.length_m


def test_start_equilibrium_matches_the_documented_solved_sag(ride):
    """40.6 % front / 22.7 % rear (solved), see docs/RIDE.md section 9."""
    sim, _, _, _, _ = ride
    eq = sim.equilibrium
    assert eq["fork_travel_mm"] == pytest.approx(73.14, abs=0.5)
    assert 100.0 * eq["fork_travel_mm"] / FORK_TRAVEL_MM == pytest.approx(40.6, abs=0.3)
    assert eq["rear_travel_mm"] == pytest.approx(40.89, abs=0.5)
    assert 100.0 * eq["rear_travel_mm"] / FORK_TRAVEL_MM == pytest.approx(22.7, abs=0.3)


def test_travel_stays_within_the_soft_joint_limits(ride):
    """Measured: fork 24.9 .. 146.9 mm, shock -4.14 .. 37.14 mm."""
    _, _, _, c, _ = ride
    fork, shock = c["fork_travel_mm"], c["shock_stroke_mm"]
    assert fork.min() >= -FORK_OVERTRAVEL_TOLERANCE_MM
    assert fork.max() <= FORK_TRAVEL_MM + FORK_OVERTRAVEL_TOLERANCE_MM
    assert shock.min() >= -SHOCK_TOPOUT_TOLERANCE_MM
    assert shock.max() <= SHOCK_STROKE_MM
    # Characteristic, loosely pinned: the road works the fork but does not bottom it out.
    assert 100.0 < fork.max() < FORK_TRAVEL_MM - 10.0
    assert shock.max() < 50.0


def test_speed_tracks_the_target_outside_the_ramp(ride):
    """Measured over the window: mean 25.03, min 22.5, max 27.1 km/h."""
    _, _, _, c, w = ride
    kmh = c["speed_mps"][w] * 3.6
    assert kmh.mean() == pytest.approx(TARGET_KMH, abs=1.0)
    assert kmh.min() > TARGET_KMH - 5.0
    assert kmh.max() < TARGET_KMH + 5.0


def test_a_road_never_launches_the_bike(ride):
    """Both wheels never leave the ground together, so the virtual rider injects nothing."""
    _, _, _, c, w = ride
    both_off = (c["front_contact"] < 0.5) & (c["rear_contact"] < 0.5)
    assert not both_off[w].any()
    assert c["rider_work_j"].max() == 0.0
    assert np.abs(c["rider_impulse_nms"]).max() == 0.0
    assert c["rider_moment_nm"].max() == 0.0 and c["rider_moment_nm"].min() == 0.0


def test_wheels_carry_load_most_of_the_time(ride):
    """Measured contact share over the window: front 96.4 %, rear 97.2 %."""
    _, _, _, c, w = ride
    assert c["front_contact"][w].mean() > 0.90
    assert c["rear_contact"][w].mean() > 0.90


def test_summary_is_consistent_with_the_channels(ride):
    sim, rec, outcome, c, w = ride
    s = summarize_ride(c, rec.sample_interval_s, sim.track, outcome, TARGET_KMH, FORK_TRAVEL_MM,
                       SHOCK_STROKE_MM, sim.applier.coil_shock.specs.bumper_engage_mm, sim.start_x_m)

    assert s.completed and s.crash is None
    assert s.fork.max_mm == pytest.approx(c["fork_travel_mm"][w].max())
    assert s.fork.bottom_outs == 0 and s.fork.top_outs == 0
    assert s.shock.bottom_outs == 0
    assert s.airborne_events == 0 and s.airborne_time_s == 0.0
    assert s.mean_speed_kmh == pytest.approx(TARGET_KMH, abs=1.0)
    assert s.n_potholes == 3 and s.n_bumps == 5
    assert len(s.potholes) == 3
    assert s.bar.rms_filtered_mps2 > 1.0
    assert s.bar.peak_raw_mps2 > s.bar.peak_filtered_mps2


def test_full_traverse_is_repeatable(ride):
    _, rec, outcome, _, _ = ride
    _, rec2, outcome2 = _traverse()

    assert outcome2.steps == outcome.steps
    assert np.array_equal(rec2.array(), rec.array())
