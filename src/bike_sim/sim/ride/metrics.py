"""
Ride Summary Metrics.

Reduces a recorded ride to the handful of numbers a suspension comparison needs: how much
travel each end used, how often it hit its stops, what the rider's contact points felt,
how the run went. Pure NumPy/SciPy over the recorder's arrays; nothing here touches
MuJoCo.

**Measurement window.** The bike starts from rest and needs about eight metres to reach
its target speed, so every statistic is taken over ``x >= start + RAMP_EXCLUSION_M``
unless stated otherwise.

**Accelerations.** The rigid contact sphere meeting a sharp heightfield edge produces
solver transients of 12-16 system weights lasting 2-4 timesteps (1-2 ms). A real tyre
spreads that impact over 10-20 ms. The summary therefore reports RMS and peak of the
vertical acceleration after a 4th-order Butterworth low-pass at ``ACCEL_FILTER_HZ``,
zero-phase (``filtfilt``), which keeps every band the suspension works in (fork ~2-4 Hz,
wheel hop ~10-15 Hz) and removes the sub-10 ms transients -- *and* reports the raw peak
next to it, labelled, so nothing is hidden. Gravity is subtracted first: at rest the
channels read zero.
"""

from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Union

import numpy as np
from scipy.signal import butter, filtfilt

from bike_sim.sim.ride.termination import RunOutcome
from bike_sim.terrain.obstacles import BUMP_TYPES, POTHOLE_TYPES, Obstacle
from bike_sim.terrain.profile import TrackSpec
from bike_sim.terrain.wheelpath import FRONT_WHEEL_RADIUS_M, REAR_WHEEL_RADIUS_M, effective_drop_m

RAMP_EXCLUSION_M = 8.0
ACCEL_FILTER_HZ = 100.0
ACCEL_FILTER_ORDER = 4
GRAVITY_MPS2 = 9.81
FORK_BOTTOM_OUT_MARGIN_MM = 5.0
"""Fork travel within this many mm of full travel counts as a bottom-out."""
TOP_OUT_TOLERANCE_MM = 1.0
"""Travel within this many mm of zero counts as a top-out."""
KMH_PER_MPS = 3.6


@dataclass
class TravelStats:
    """Travel usage of one suspension end over the measurement window."""

    end: str
    travel_limit_mm: float
    max_mm: float
    max_pct: float
    p95_mm: float
    mean_mm: float
    bottom_out_threshold_mm: float
    bottom_outs: int
    top_outs: int
    max_compression_mps: float
    max_rebound_mps: float


@dataclass
class AccelStats:
    """Vertical acceleration at one rider contact point, gravity removed."""

    point: str
    filter_hz: float
    rms_filtered_mps2: float
    peak_filtered_mps2: float
    peak_raw_mps2: float
    raw_peak_note: str = "raw peak includes solver contact transients (1-2 ms); compare filtered values"


@dataclass
class RiderStats:
    """
    What the seated rider's body felt and how their weight sat on the bike.

    Loads are means over the measurement window, as fractions of the rider's weight, so
    they read against the static split the pose was built to (docs/RIDE.md section 7).
    """

    torso: AccelStats
    pelvis: AccelStats
    saddle_lift_events: int
    saddle_lift_time_s: float
    saddle_gap_max_mm: float
    mean_saddle_share: float
    mean_pedal_share: float
    mean_bar_share: float
    min_saddle_load_n: float
    max_saddle_load_n: float


@dataclass
class PotholeReport:
    """Declared versus effective size of one pothole on the track."""

    label: str
    kind: str
    start_m: float
    length_m: float
    declared_depth_mm: float
    effective_drop_front_mm: float
    effective_drop_rear_mm: float


@dataclass
class RideSummary:
    """Everything ``summary.json`` carries."""

    track: str
    track_length_m: float
    n_potholes: int
    n_bumps: int
    n_obstacles: int
    target_speed_kmh: float
    outcome: str
    completed: bool
    crash: Optional[str]
    steps: int
    sim_time_s: float
    wall_clock_s: float
    sample_interval_s: float
    window_start_m: float
    window_rows: int
    mean_speed_kmh: float
    airborne_events: int
    airborne_time_s: float
    fork: TravelStats
    shock: TravelStats
    bar: AccelStats
    saddle: AccelStats
    potholes: List[PotholeReport] = field(default_factory=list)
    extras: Dict[str, float] = field(default_factory=dict)
    rider_variant: str = "lumped"
    rider: Optional[RiderStats] = None

    def to_dict(self) -> Dict:
        """Returns a JSON-ready dictionary."""
        return asdict(self)

    def to_json(self) -> str:
        """Returns the summary as indented JSON text."""
        return json.dumps(self.to_dict(), indent=2)

    def write_json(self, path: Union[str, Path]) -> Path:
        """Writes ``summary.json``; returns the path."""
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(self.to_json() + "\n", encoding="utf-8")
        return destination

    def format_table(self) -> str:
        """Renders the summary as a fixed-width console table."""
        lines = [
            f"Ride summary -- {self.track} ({self.track_length_m:.0f} m, "
            f"{self.n_potholes} potholes, {self.n_bumps} bumps) at {self.target_speed_kmh:.0f} km/h",
            f"  outcome: {self.outcome}" + (f" ({self.crash})" if self.crash else ""),
            f"  {self.steps} steps, {self.sim_time_s:.2f} s sim, {self.wall_clock_s:.2f} s wall; "
            f"window from x = {self.window_start_m:.1f} m ({self.window_rows} rows)",
            f"  mean speed {self.mean_speed_kmh:.2f} km/h; airborne {self.airborne_events} x, "
            f"{self.airborne_time_s * 1000:.0f} ms total",
            "",
            f"  {'end':6s} {'max mm':>8s} {'max %':>6s} {'p95 mm':>8s} {'mean mm':>8s} "
            f"{'bottom':>7s} {'top':>5s} {'comp m/s':>9s} {'reb m/s':>8s}",
        ]
        for t in (self.fork, self.shock):
            lines.append(
                f"  {t.end:6s} {t.max_mm:8.1f} {t.max_pct:6.1f} {t.p95_mm:8.1f} {t.mean_mm:8.1f} "
                f"{t.bottom_outs:7d} {t.top_outs:5d} {t.max_compression_mps:9.3f} {t.max_rebound_mps:8.3f}"
            )
        lines += [
            "",
            f"  {'point':6s} {'RMS':>8s} {'peak':>8s} {'raw peak':>9s}   (m/s^2, vertical, g removed, "
            f"{self.bar.filter_hz:.0f} Hz low-pass; raw incl. solver transients)",
        ]
        for a in (self.bar, self.saddle):
            lines.append(
                f"  {a.point:6s} {a.rms_filtered_mps2:8.2f} {a.peak_filtered_mps2:8.2f} {a.peak_raw_mps2:9.1f}"
            )
        if self.rider is not None:
            r = self.rider
            for a in (r.torso, r.pelvis):
                lines.append(
                    f"  {a.point:6s} {a.rms_filtered_mps2:8.2f} {a.peak_filtered_mps2:8.2f} {a.peak_raw_mps2:9.1f}"
                )
            lines += [
                "",
                f"  seated rider: mean load split saddle {100 * r.mean_saddle_share:.1f} % / pedals "
                f"{100 * r.mean_pedal_share:.1f} % / bar {100 * r.mean_bar_share:.1f} % of rider weight; "
                f"saddle load {r.min_saddle_load_n:.0f}-{r.max_saddle_load_n:.0f} N; "
                f"left the saddle {r.saddle_lift_events} x, {r.saddle_lift_time_s * 1000:.0f} ms total, "
                f"max gap {r.saddle_gap_max_mm:.1f} mm",
            ]
        if self.potholes:
            lines += ["", f"  {'pothole':18s} {'x m':>7s} {'len m':>6s} {'depth mm':>9s} {'drop F mm':>10s} {'drop R mm':>10s}"]
            for p in self.potholes:
                lines.append(
                    f"  {p.label:18s} {p.start_m:7.2f} {p.length_m:6.2f} {p.declared_depth_mm:9.0f} "
                    f"{p.effective_drop_front_mm:10.0f} {p.effective_drop_rear_mm:10.0f}"
                )
        return "\n".join(lines)


# --------------------------------------------------------------------------------------
# Computation
# --------------------------------------------------------------------------------------


def count_events(flags: np.ndarray) -> int:
    """Counts rising edges of a boolean series: contiguous runs, not samples."""
    flags = np.asarray(flags, dtype=bool)
    if flags.size == 0:
        return 0
    starts = np.flatnonzero(np.diff(flags.astype(int)) == 1)
    return int(starts.size + (1 if flags[0] else 0))


def lowpass(signal: np.ndarray, sample_interval_s: float, cutoff_hz: float = ACCEL_FILTER_HZ,
            order: int = ACCEL_FILTER_ORDER) -> np.ndarray:
    """
    Zero-phase Butterworth low-pass.

    Args:
        signal: Samples.
        sample_interval_s: Time between samples.
        cutoff_hz: -3 dB frequency.
        order: Filter order.

    Returns:
        Filtered samples; the input unchanged when it is too short to filter or the
        cutoff is not below Nyquist.
    """
    signal = np.asarray(signal, dtype=float)
    nyquist = 0.5 / sample_interval_s
    padlen = 3 * (order + 1)  # filtfilt's default edge padding
    if signal.size <= padlen or cutoff_hz >= nyquist:
        return signal.copy()
    b, a = butter(order, cutoff_hz / nyquist, btype="low")
    return filtfilt(b, a, signal)


def travel_stats(end: str, travel_mm: np.ndarray, shaft_mps: np.ndarray, travel_limit_mm: float,
                 bottom_out_threshold_mm: float) -> TravelStats:
    """
    Summarises one end's travel channel.

    Args:
        end: "fork" or "shock".
        travel_mm: Travel samples over the window.
        shaft_mps: Shaft velocity samples (positive = compressing).
        travel_limit_mm: Full travel.
        bottom_out_threshold_mm: Travel at or beyond which a sample is a bottom-out.
    """
    if travel_mm.size == 0:
        return TravelStats(end, travel_limit_mm, 0.0, 0.0, 0.0, 0.0, bottom_out_threshold_mm, 0, 0, 0.0, 0.0)
    return TravelStats(
        end=end,
        travel_limit_mm=float(travel_limit_mm),
        max_mm=float(np.max(travel_mm)),
        max_pct=float(100.0 * np.max(travel_mm) / travel_limit_mm),
        p95_mm=float(np.percentile(travel_mm, 95)),
        mean_mm=float(np.mean(travel_mm)),
        bottom_out_threshold_mm=float(bottom_out_threshold_mm),
        bottom_outs=count_events(travel_mm >= bottom_out_threshold_mm),
        top_outs=count_events(travel_mm <= TOP_OUT_TOLERANCE_MM),
        max_compression_mps=float(max(0.0, np.max(shaft_mps))),
        max_rebound_mps=float(max(0.0, -np.min(shaft_mps))),
    )


def accel_stats(point: str, acc_vert_mps2: np.ndarray, sample_interval_s: float) -> AccelStats:
    """
    Summarises one accelerometer's vertical channel.

    Args:
        point: "bar" or "saddle".
        acc_vert_mps2: Proper vertical acceleration samples (world Z), +9.81 at rest.
        sample_interval_s: Time between samples.
    """
    if acc_vert_mps2.size == 0:
        return AccelStats(point, ACCEL_FILTER_HZ, 0.0, 0.0, 0.0)
    a = np.asarray(acc_vert_mps2, dtype=float) - GRAVITY_MPS2
    filtered = lowpass(a, sample_interval_s)
    return AccelStats(
        point=point,
        filter_hz=ACCEL_FILTER_HZ,
        rms_filtered_mps2=float(np.sqrt(np.mean(filtered**2))),
        peak_filtered_mps2=float(np.max(np.abs(filtered))),
        peak_raw_mps2=float(np.max(np.abs(a))),
    )


def rider_stats(w: Dict[str, np.ndarray], sample_interval_s: float, rider_weight_n: float) -> RiderStats:
    """
    Summarises the seated rider's channels over the measurement window.

    Args:
        w: Windowed channels keyed by name.
        sample_interval_s: Time between samples.
        rider_weight_n: The rider's weight, for the load shares.
    """
    lifted = w["saddle_gap_m"] > 0.0
    saddle = w["saddle_load_n"]
    pedals = w["pedal_load_front_n"] + w["pedal_load_rear_n"]
    bar = w["bar_hand_load_n"]
    n = max(1, saddle.size)
    return RiderStats(
        torso=accel_stats("torso", w["rider_torso_acc_vert_mps2"], sample_interval_s),
        pelvis=accel_stats("pelvis", w["rider_pelvis_acc_vert_mps2"], sample_interval_s),
        saddle_lift_events=count_events(lifted),
        saddle_lift_time_s=float(lifted.sum() * sample_interval_s),
        saddle_gap_max_mm=float(np.max(w["saddle_gap_m"]) * 1000.0) if saddle.size else 0.0,
        mean_saddle_share=float(saddle.sum() / n / rider_weight_n),
        mean_pedal_share=float(pedals.sum() / n / rider_weight_n),
        mean_bar_share=float(bar.sum() / n / rider_weight_n),
        min_saddle_load_n=float(np.min(saddle)) if saddle.size else 0.0,
        max_saddle_load_n=float(np.max(saddle)) if saddle.size else 0.0,
    )


def pothole_reports(track: TrackSpec) -> List[PotholeReport]:
    """Declared and effective sizes for every pothole-type obstacle on a track."""
    reports = []
    for o in track.sorted_obstacles:
        if isinstance(o, POTHOLE_TYPES):
            reports.append(
                PotholeReport(
                    label=o.label,
                    kind=type(o).__name__,
                    start_m=float(o.start_m),
                    length_m=float(o.length_m),
                    declared_depth_mm=float(o.depth_m * 1000.0),
                    effective_drop_front_mm=float(effective_drop_m(o, FRONT_WHEEL_RADIUS_M) * 1000.0),
                    effective_drop_rear_mm=float(effective_drop_m(o, REAR_WHEEL_RADIUS_M) * 1000.0),
                )
            )
    return reports


def count_kinds(obstacles: Sequence[Obstacle]):
    """Returns (potholes, bumps, annotated obstacles)."""
    potholes = sum(isinstance(o, POTHOLE_TYPES) for o in obstacles)
    bumps = sum(isinstance(o, BUMP_TYPES) for o in obstacles)
    annotated = sum(o.annotate for o in obstacles)
    return potholes, bumps, annotated


def summarize_ride(
    channels: Dict[str, np.ndarray],
    sample_interval_s: float,
    track: TrackSpec,
    outcome: RunOutcome,
    target_speed_kmh: float,
    fork_travel_mm: float,
    shock_stroke_mm: float,
    shock_bumper_engage_mm: float,
    start_x_m: float,
    extras: Optional[Dict[str, float]] = None,
    rider_variant: str = "lumped",
    rider_mass_kg: float = 80.0,
) -> RideSummary:
    """
    Computes the ride summary from recorded channels.

    Args:
        channels: Recorder channels keyed by name (``RideRecorder.columns()``).
        sample_interval_s: Time between recorded rows.
        track: The track ridden.
        outcome: How the run ended.
        target_speed_kmh: Cruise target.
        fork_travel_mm: Full fork travel.
        shock_stroke_mm: Full shock stroke.
        shock_bumper_engage_mm: Stroke at which the shock's bottom-out bumper engages.
        start_x_m: Chassis start position; the window opens ``RAMP_EXCLUSION_M`` later.
        extras: Free-form numbers to carry into the summary (e.g. seed, sag).
        rider_variant: Which rider rode: ``none``, ``lumped`` or ``seated``. The seated
            rider's channels are summarised only for ``seated``.
        rider_mass_kg: The rider's mass, for the seated load shares.

    Returns:
        The summary.
    """
    x = channels["x_m"]
    window_start = start_x_m + RAMP_EXCLUSION_M
    window = x >= window_start
    w = {name: values[window] for name, values in channels.items()}

    airborne = (w["front_contact"] < 0.5) & (w["rear_contact"] < 0.5) if window.any() else np.zeros(0, bool)
    n_potholes, n_bumps, n_annotated = count_kinds(track.obstacles)
    rider = None
    if rider_variant == "seated" and "saddle_load_n" in w:
        rider = rider_stats(w, sample_interval_s, rider_mass_kg * GRAVITY_MPS2)

    return RideSummary(
        track=track.name,
        track_length_m=float(track.length_m),
        n_potholes=n_potholes,
        n_bumps=n_bumps,
        n_obstacles=n_annotated,
        target_speed_kmh=float(target_speed_kmh),
        outcome=outcome.reason,
        completed=outcome.completed,
        crash=outcome.crash.describe() if outcome.crash is not None else None,
        steps=int(outcome.steps),
        sim_time_s=float(outcome.sim_time_s),
        wall_clock_s=float(outcome.wall_clock_s),
        sample_interval_s=float(sample_interval_s),
        window_start_m=float(window_start),
        window_rows=int(window.sum()),
        mean_speed_kmh=float(np.mean(w["speed_mps"]) * KMH_PER_MPS) if window.any() else 0.0,
        airborne_events=count_events(airborne),
        airborne_time_s=float(airborne.sum() * sample_interval_s),
        fork=travel_stats("fork", w["fork_travel_mm"], w["fork_shaft_mps"], fork_travel_mm,
                          fork_travel_mm - FORK_BOTTOM_OUT_MARGIN_MM),
        shock=travel_stats("shock", w["shock_stroke_mm"], w["shock_shaft_mps"], shock_stroke_mm,
                           shock_bumper_engage_mm),
        bar=accel_stats("bar", w["bar_acc_vert_mps2"], sample_interval_s),
        saddle=accel_stats("saddle", w["saddle_acc_vert_mps2"], sample_interval_s),
        potholes=pothole_reports(track),
        extras=dict(extras or {}),
        rider_variant=rider_variant,
        rider=rider,
    )


__all__ = [
    "RAMP_EXCLUSION_M",
    "ACCEL_FILTER_HZ",
    "FORK_BOTTOM_OUT_MARGIN_MM",
    "TOP_OUT_TOLERANCE_MM",
    "TravelStats",
    "AccelStats",
    "RiderStats",
    "PotholeReport",
    "RideSummary",
    "count_events",
    "lowpass",
    "travel_stats",
    "accel_stats",
    "rider_stats",
    "pothole_reports",
    "summarize_ride",
]
