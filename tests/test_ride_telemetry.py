"""
Tests for the ride-mode telemetry recorder and summary metrics.

Tests include:
- Recorder: one row per recorded step plus the initial row, decimation, channel order,
  CSV header and row count, read-back equality, bit-identical repeat of a short run,
  rear-wheel lookup agreeing with the analytical solver, accelerometers reading gravity
  at rest.
- Metrics (pure NumPy, synthetic inputs): event counting, low-pass leaving slow content
  and removing a 1 ms spike, travel and acceleration statistics, ramp-window exclusion,
  pothole effective drops, JSON round-trip and console table.
- End to end on the 40 m `single_edge` preset: summary fields consistent with the run.

The `single_edge` traverse is ~11 700 steps and takes a few seconds; it runs once per
module.
"""

import json

import numpy as np
import pytest

from bike_sim.sim.ride.metrics import (
    ACCEL_FILTER_HZ,
    GRAVITY_MPS2,
    RAMP_EXCLUSION_M,
    RideSummary,
    accel_stats,
    count_events,
    lowpass,
    pothole_reports,
    summarize_ride,
    travel_stats,
)
from bike_sim.sim.ride.recorder import CHANNELS, RideRecorder, read_csv
from bike_sim.sim.ride.termination import REASON_END_OF_TRACK, RunOutcome
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import Pothole, TrackSpec, get_preset


# --------------------------------------------------------------------------------------
# Fixtures
# --------------------------------------------------------------------------------------


@pytest.fixture(scope="module")
def edge_run():
    """One recorded traverse of `single_edge` at 25 km/h, shared by the module."""
    sim = RideSimulation(track=get_preset("single_edge"), target_speed_kmh=25.0)
    rec = RideRecorder(sim)
    rec.record(sim)
    outcome = sim.run(on_step=rec.record)
    return sim, rec, outcome


def _summary(sim, rec, outcome) -> RideSummary:
    return summarize_ride(
        rec.columns(),
        rec.sample_interval_s,
        sim.track,
        outcome,
        target_speed_kmh=25.0,
        fork_travel_mm=sim.specs.fork_travel,
        shock_stroke_mm=sim.specs.shock_stroke,
        shock_bumper_engage_mm=sim.applier.coil_shock.specs.bumper_engage_mm,
        start_x_m=sim.start_x_m,
        extras={"seed": 0},
    )


# --------------------------------------------------------------------------------------
# Recorder
# --------------------------------------------------------------------------------------


def test_recorder_stores_initial_row_plus_one_per_step(edge_run):
    sim, rec, outcome = edge_run

    assert outcome.reason == REASON_END_OF_TRACK
    assert rec.rows == outcome.steps + 1
    table = rec.array()
    assert table.shape == (rec.rows, len(CHANNELS))
    assert rec.column("time_s")[0] == 0.0
    assert np.all(np.diff(rec.column("time_s")) > 0)
    assert rec.column("x_m")[-1] >= sim.track.length_m


def test_recorder_channels_are_physically_sane(edge_run):
    sim, rec, _ = edge_run
    c = rec.columns()

    assert np.all(c["fork_travel_mm"] >= -1.0) and np.all(c["fork_travel_mm"] <= sim.specs.fork_travel + 6.0)
    assert set(np.unique(c["front_contact"])) <= {0.0, 1.0}
    assert np.all(c["front_load_n"] >= 0.0)
    # Proper acceleration at the start (at rest, upright) is gravity, pointing up.
    assert c["bar_acc_vert_mps2"][0] == pytest.approx(GRAVITY_MPS2, abs=0.2)
    assert c["saddle_acc_vert_mps2"][0] == pytest.approx(GRAVITY_MPS2, abs=0.2)
    assert abs(c["bar_acc_long_mps2"][0]) < 0.5
    # The 90 mm edge at 20 m is where the fork works hardest.
    x = c["x_m"]
    near_edge = (x > 19.5) & (x < 23.0)
    assert np.max(c["fork_travel_mm"][near_edge]) > np.max(c["fork_travel_mm"][x < 15.0]) + 10.0


def test_recorder_rear_wheel_lookup_matches_the_solver(edge_run):
    sim, rec, _ = edge_run
    for stroke in (0.0, 7.3, 15.25, 40.0, 64.9):
        expected = float(sim.solver.solve_state_from_shock_stroke(stroke)["wheel_travel"])
        assert rec.rear_wheel_mm(stroke) == pytest.approx(expected, abs=0.02)
    # Beyond the stroke range the lookup extrapolates instead of clamping.
    assert rec.rear_wheel_mm(-1.0) < rec.rear_wheel_mm(0.0)
    assert rec.rear_wheel_mm(70.0) > rec.rear_wheel_mm(65.0)


def test_recorder_csv_round_trip(edge_run, tmp_path):
    _, rec, _ = edge_run
    path = rec.write_csv(tmp_path / "telemetry.csv")
    text = path.read_text(encoding="utf-8").splitlines()

    assert text[0] == ",".join(CHANNELS)
    assert len(text) == rec.rows + 1
    back = read_csv(path)
    assert list(back) == list(CHANNELS)
    for name in ("x_m", "fork_travel_mm", "bar_acc_vert_mps2"):
        assert np.allclose(back[name], rec.column(name), rtol=1e-9, atol=1e-9)


def test_recorder_decimation():
    sim = RideSimulation(track=get_preset("flat"), target_speed_kmh=25.0)
    rec = RideRecorder(sim, decimate=10)
    rec.record(sim)
    for _ in range(99):
        sim.step()
        rec.record(sim)

    assert rec.rows == 10
    assert rec.sample_interval_s == pytest.approx(10 * sim.model.opt.timestep)
    assert np.allclose(np.diff(rec.column("time_s")), rec.sample_interval_s)


def test_recorder_rejects_bad_decimation():
    sim = RideSimulation(track=get_preset("flat"), target_speed_kmh=25.0)
    with pytest.raises(ValueError, match="decimate"):
        RideRecorder(sim, decimate=0)


def test_short_run_repeats_bit_identically(tmp_path):
    def run():
        sim = RideSimulation(track=get_preset("single_edge"), target_speed_kmh=25.0)
        rec = RideRecorder(sim)
        rec.record(sim)
        for _ in range(1500):
            sim.step()
            rec.record(sim)
        return rec

    a, b = run(), run()
    assert np.array_equal(a.array(), b.array())
    assert a.write_csv(tmp_path / "a.csv").read_bytes() == b.write_csv(tmp_path / "b.csv").read_bytes()


# --------------------------------------------------------------------------------------
# Metrics, synthetic
# --------------------------------------------------------------------------------------


def test_count_events_counts_runs_not_samples():
    assert count_events(np.array([0, 0, 1, 1, 1, 0, 1, 0, 0, 1], bool)) == 3
    assert count_events(np.array([1, 1, 0, 1], bool)) == 2
    assert count_events(np.zeros(5, bool)) == 0
    assert count_events(np.zeros(0, bool)) == 0


def test_lowpass_keeps_slow_content_and_spreads_a_millisecond_spike():
    """
    A low-pass preserves an impulse's area and spreads it over ~1/(2 fc): a 1.5 ms,
    200 m/s^2 transient (0.3 m/s of velocity change) becomes a ~5 ms pulse of roughly
    0.3 * 2 * 100 = 60 m/s^2. That is the honest, tyre-like number the summary reports.
    """
    dt = 0.0005
    t = np.arange(0.0, 2.0, dt)
    slow = 3.0 * np.sin(2.0 * np.pi * 5.0 * t)
    spiky = slow.copy()
    spiky[2000:2003] += 200.0  # 1.5 ms transient, impulse 0.3 m/s

    filtered = lowpass(spiky, dt, cutoff_hz=ACCEL_FILTER_HZ)
    pulse = filtered - slow

    assert np.max(np.abs(lowpass(slow, dt) - slow)) < 0.05
    assert 40.0 < np.max(pulse) < 80.0
    assert np.sum(pulse) * dt == pytest.approx(0.3, rel=0.05)  # impulse preserved
    assert np.all(np.abs(pulse[:1900]) < 0.5) and np.all(np.abs(pulse[2100:]) < 0.5)


def test_lowpass_passes_short_signals_through():
    short = np.arange(10.0)
    assert np.array_equal(lowpass(short, 0.0005), short)


def test_travel_stats_bottom_and_top_outs():
    travel = np.array([0.0, 0.5, 20.0, 176.0, 178.0, 30.0, 0.2, 40.0, 177.0])
    shaft = np.array([0.0, 0.1, 1.0, 3.0, -0.5, -2.0, 0.0, 0.5, 2.5])

    stats = travel_stats("fork", travel, shaft, travel_limit_mm=180.0, bottom_out_threshold_mm=175.0)

    assert stats.max_mm == 178.0
    assert stats.max_pct == pytest.approx(100.0 * 178.0 / 180.0)
    assert stats.bottom_outs == 2
    assert stats.top_outs == 2
    assert stats.max_compression_mps == 3.0
    assert stats.max_rebound_mps == 2.0
    assert stats.p95_mm == pytest.approx(np.percentile(travel, 95))


def test_accel_stats_removes_gravity_and_reports_raw_peak_separately():
    dt = 0.0005
    t = np.arange(0.0, 1.0, dt)
    clean = GRAVITY_MPS2 + 2.0 * np.sin(2.0 * np.pi * 3.0 * t)

    stats = accel_stats("bar", clean, dt)
    assert stats.rms_filtered_mps2 == pytest.approx(2.0 / np.sqrt(2.0), rel=0.02)
    assert stats.peak_filtered_mps2 == pytest.approx(2.0, rel=0.02)
    assert stats.peak_raw_mps2 == pytest.approx(2.0, rel=0.02)

    spiky = clean.copy()
    spiky[1000:1002] += 150.0  # 1 ms transient
    stats = accel_stats("bar", spiky, dt)
    assert stats.peak_raw_mps2 == pytest.approx(150.0, abs=2.1)
    assert stats.peak_filtered_mps2 < 0.4 * stats.peak_raw_mps2
    assert stats.filter_hz == ACCEL_FILTER_HZ
    assert "solver" in stats.raw_peak_note


def test_pothole_reports_show_geometric_limit():
    track = TrackSpec(
        name="t",
        length_m=50.0,
        obstacles=[
            Pothole(start_m=10.0, depth_m=0.120, hole_length_m=0.300),
            Pothole(start_m=20.0, depth_m=0.060, hole_length_m=1.000),
        ],
    )
    short, long_ = pothole_reports(track)

    assert short.declared_depth_mm == 120.0
    assert short.effective_drop_front_mm == pytest.approx(31.6, abs=0.5)
    assert short.effective_drop_rear_mm == pytest.approx(33.6, abs=0.5)
    assert long_.effective_drop_front_mm == pytest.approx(60.0, abs=0.1)


def test_summarize_ride_excludes_the_ramp_window():
    n = 4000
    dt = 0.0005
    x = np.linspace(0.0, 40.0, n)
    channels = {name: np.zeros(n) for name in CHANNELS}
    channels["x_m"] = x
    channels["speed_mps"] = np.where(x < 10.0, 3.0, 7.0)
    channels["fork_travel_mm"] = np.where(x < 10.0, 179.0, 60.0)  # bottom-out only in the ramp
    channels["front_contact"] = np.ones(n)
    channels["rear_contact"] = np.ones(n)
    channels["rear_contact"][2000:2010] = 0.0
    channels["front_contact"][2000:2010] = 0.0
    channels["bar_acc_vert_mps2"] = np.full(n, GRAVITY_MPS2)
    channels["saddle_acc_vert_mps2"] = np.full(n, GRAVITY_MPS2)
    outcome = RunOutcome(REASON_END_OF_TRACK, n - 1, (n - 1) * dt, 1.0, 40.0)
    track = TrackSpec(name="synthetic", length_m=40.0)

    s = summarize_ride(channels, dt, track, outcome, 25.0, 180.0, 65.0, 55.0, start_x_m=2.0)

    assert s.window_start_m == 2.0 + RAMP_EXCLUSION_M
    assert s.window_rows == int(np.sum(x >= 10.0))
    assert s.mean_speed_kmh == pytest.approx(7.0 * 3.6)
    assert s.fork.bottom_outs == 0
    assert s.fork.max_mm == 60.0
    assert s.airborne_events == 1
    assert s.airborne_time_s == pytest.approx(10 * dt)
    assert s.bar.rms_filtered_mps2 == pytest.approx(0.0, abs=1e-9)
    assert s.completed and s.crash is None


# --------------------------------------------------------------------------------------
# End to end
# --------------------------------------------------------------------------------------


def test_summary_of_the_single_edge_run(edge_run, tmp_path):
    sim, rec, outcome = edge_run
    s = _summary(sim, rec, outcome)

    assert s.track == "single_edge"
    assert s.completed and s.outcome == REASON_END_OF_TRACK
    assert s.n_obstacles == 1 and s.n_potholes == 0 and s.n_bumps == 0
    assert s.steps == outcome.steps
    assert abs(s.mean_speed_kmh - 25.0) < 1.5
    assert 0.0 < s.fork.max_mm <= sim.specs.fork_travel + 6.0
    assert s.fork.max_pct == pytest.approx(100.0 * s.fork.max_mm / sim.specs.fork_travel)
    assert s.bar.peak_raw_mps2 >= s.bar.peak_filtered_mps2 >= s.bar.rms_filtered_mps2 > 0.0
    assert s.extras == {"seed": 0}

    path = s.write_json(tmp_path / "summary.json")
    loaded = json.loads(path.read_text(encoding="utf-8"))
    assert loaded["fork"]["max_mm"] == pytest.approx(s.fork.max_mm)
    assert loaded["bar"]["filter_hz"] == ACCEL_FILTER_HZ

    table = s.format_table()
    assert "single_edge" in table and "fork" in table and "raw peak" in table
