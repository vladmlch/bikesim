"""Full-traverse convergence checks for the pneumatic tyre fidelity tiers.

Fast and detailed runs use a common 1 ms telemetry interval. Measured on the development Mac
(M4 Max, MuJoCo 3.12.0, Python 3.12.13; rider seated; 25 km/h), the flat-track RMS differs
by 0.069 m/s² at the bar and 0.0099 m/s² at the saddle. On `single_edge`, the largest RMS
difference is 10.4 %; fork/shock maximum and p95 travel differ by at most 4.3 %, traverse
time by 0.09 %, and total rim strikes are equal (individual wheels can differ by one).
The assertions keep modest headroom around those measured values.
"""

import pytest

from bike_sim.physics.tyre import TyreConfig
from bike_sim.sim.ride.metrics import summarize_ride
from bike_sim.sim.ride.recorder import RideRecorder
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset

_FLAT_BAR_RMS_TOL_MPS2 = 0.10
_FLAT_SADDLE_RMS_TOL_MPS2 = 0.02
_ROUGH_RMS_REL_TOL = 0.15
_TRAVEL_REL_TOL = 0.06
_TRAVERSE_TIME_REL_TOL = 0.002


def _run_summary(track_name: str, tier: str):
    """Rides one preset and returns the same summary used by the CLI."""
    sim = RideSimulation(
        track=get_preset(track_name),
        target_speed_kmh=25.0,
        tyre=TyreConfig(model="pneumatic", tier=tier),
    )
    recorder = RideRecorder(sim, decimate=2 if tier == "fast" else 4)
    recorder.record(sim)
    outcome = sim.run(on_step=recorder.record)
    return summarize_ride(
        recorder.columns(),
        recorder.sample_interval_s,
        sim.track,
        outcome,
        target_speed_kmh=25.0,
        fork_travel_mm=sim.specs.fork_travel,
        shock_stroke_mm=sim.specs.shock_stroke,
        shock_bumper_engage_mm=sim.applier.coil_shock.specs.bumper_engage_mm,
        start_x_m=sim.start_x_m,
        rider_variant=sim.rider_variant,
        rider_mass_kg=sim.rider.mass_kg,
        tyre_rim_events=recorder.tyre_rim_events,
        tyre_rim_starts=recorder.tyre_rim_starts,
        tyre_times_s=recorder.tyre_times_s,
    )


def _relative_difference(value: float, reference: float) -> float:
    """Returns a scale-free difference, guarded for zero-valued reference metrics."""
    return abs(value - reference) / max(abs(reference), 1e-9)


@pytest.mark.parametrize("track_name", ["flat", "single_edge"])
def test_fast_tier_metrics_converge_to_detailed(track_name):
    fast = _run_summary(track_name, "fast")
    detailed = _run_summary(track_name, "detailed")

    assert fast.completed, fast.outcome
    assert detailed.completed, detailed.outcome
    if track_name == "flat":
        assert abs(fast.bar.rms_filtered_mps2 - detailed.bar.rms_filtered_mps2) <= _FLAT_BAR_RMS_TOL_MPS2
        assert abs(fast.saddle.rms_filtered_mps2 - detailed.saddle.rms_filtered_mps2) <= _FLAT_SADDLE_RMS_TOL_MPS2
    else:
        assert _relative_difference(
            fast.bar.rms_filtered_mps2, detailed.bar.rms_filtered_mps2
        ) <= _ROUGH_RMS_REL_TOL
        assert _relative_difference(
            fast.saddle.rms_filtered_mps2, detailed.saddle.rms_filtered_mps2
        ) <= _ROUGH_RMS_REL_TOL

    for fast_value, detailed_value in (
        (fast.fork.max_pct, detailed.fork.max_pct),
        (fast.fork.p95_mm, detailed.fork.p95_mm),
        (fast.shock.max_pct, detailed.shock.max_pct),
        (fast.shock.p95_mm, detailed.shock.p95_mm),
    ):
        assert _relative_difference(fast_value, detailed_value) <= _TRAVEL_REL_TOL
    assert _relative_difference(fast.sim_time_s, detailed.sim_time_s) <= _TRAVERSE_TIME_REL_TOL

    fast_rim_counts = tuple(fast.tyres[wheel].rim_strikes for wheel in ("front", "rear"))
    detailed_rim_counts = tuple(
        detailed.tyres[wheel].rim_strikes for wheel in ("front", "rear")
    )
    assert sum(fast_rim_counts) == sum(detailed_rim_counts)
    assert all(abs(a - b) <= 1 for a, b in zip(fast_rim_counts, detailed_rim_counts))
