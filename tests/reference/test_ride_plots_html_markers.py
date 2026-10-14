"""ride.html event markers are batched into one layout update and stay linear in ride length."""
from pathlib import Path
import time

import numpy as np
import pytest

pytest.importorskip("plotly")
from bike_sim.viz.ride_plots import plot_physical_ride_html, physical_ride_figures, _shift_events


def _channels(n_rows: int, n_shifts: int, n_obstacles: int):
    t = np.arange(n_rows) * 0.01
    ch = dict(time_s=t, x_m=t * 3.0, speed_kmh=np.full(n_rows, 10.0), grade_pct=np.zeros(n_rows),
              cadence_rpm=np.full(n_rows, 70.0), required_cadence_rpm=np.full(n_rows, 70.0),
              shift_torque_factor=np.ones(n_rows), gear_rear_teeth=np.full(n_rows, 30.0),
              gear_front_teeth=np.full(n_rows, 34.0), freehub_torque_nm=np.zeros(n_rows),
              motor_request_nm=np.zeros(n_rows), motor_torque_nm=np.zeros(n_rows),
              front_load_n=np.full(n_rows, 400.0), rear_load_n=np.full(n_rows, 500.0))
    count = np.zeros(n_rows)
    idx = np.linspace(1, n_rows - 1, n_shifts).astype(int)
    for k, i in enumerate(idx):
        count[i:] = k + 1
    ch["shift_count"] = count
    ch["last_shift_direction"] = np.array(["up"] * n_rows, dtype=object)
    ch["last_shift_from_teeth"] = np.full(n_rows, 30.0)
    ch["last_shift_to_teeth"] = np.full(n_rows, 28.0)
    labels = np.array([""] * n_rows, dtype=object)
    for k in range(n_obstacles):
        labels[(k + 1) * n_rows // (n_obstacles + 1)] = f"rock{k}@{float(k + 1):.1f}m"
    ch["obstacle"] = labels
    return ch


def test_markers_are_batched_and_match_the_events(tmp_path):
    """Every shift event becomes exactly one dotted line plus one label on the gear row (its
    own subplot axes), every obstacle one paper-relative marker on figure 2, and the HTML
    writer still produces the file."""
    ch = _channels(600, 7, 3)
    assert len(_shift_events(ch)) == 7
    fig1, fig2 = physical_ride_figures(ch)
    lines = [sh for sh in fig1.layout.shapes if sh.type == "line"]
    assert len(lines) == 7
    gear_x, gear_y = fig1.get_subplot(4, 1).xaxis.plotly_name, fig1.get_subplot(4, 1).yaxis.plotly_name
    assert {sh.xref for sh in lines} == {gear_x.replace("axis", "")}
    assert {sh.yref for sh in lines} == {gear_y.replace("axis", "") + " domain"}
    labels = [a for a in fig1.layout.annotations if a.text == "up:30->28"]
    assert len(labels) == 7 and all(a.textangle == -90 for a in labels)
    assert len(fig2.layout.shapes) == 3 and {sh.yref for sh in fig2.layout.shapes} == {"paper"}
    assert sorted(a.text for a in fig2.layout.annotations if "@" in a.text) == [
        "rock0@1.0m", "rock1@2.0m", "rock2@3.0m"]
    assert Path(plot_physical_ride_html(ch, tmp_path)).exists()


def test_shift_events_ignore_nan_teeth():
    """A NaN in the from/to teeth columns yields an unlabeled event instead of a ValueError."""
    ch = _channels(50, 2, 0)
    ch["last_shift_from_teeth"][:] = np.nan
    events = _shift_events(ch)
    assert len(events) == 2 and all(label == "" for _, label in events)


def test_html_generation_is_linear_in_ride_length(tmp_path):
    """Ten times the rows (and events) must not cost more than ~25x the time; the quadratic
    per-event path cost ~86x (0.43 s -> 37 s on real preview rows)."""
    small = _channels(1500, 25, 6)
    big = _channels(15000, 250, 6)
    t0 = time.perf_counter(); plot_physical_ride_html(small, tmp_path / "s"); t_small = time.perf_counter() - t0
    t0 = time.perf_counter(); plot_physical_ride_html(big, tmp_path / "b"); t_big = time.perf_counter() - t0
    assert t_big < 25 * max(t_small, 0.05), (t_small, t_big)
