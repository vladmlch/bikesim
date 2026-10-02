"""
Tests for the full `enduro_aggressive` traverse, run termination, and the interactive units.

Tests include:
- One 115 m `enduro_aggressive` run at 25 km/h: it terminates with the reason "end of track",
  never trips the crash detector, and stays inside the suspension's joint limits to within the
  measured soft-limit excursion.
- What the bike actually does at the kicker, which is **not** what docs/RIDE.md section 2
  predicts: it reaches the lip at 22.1 km/h -- inside the stated 20.9-30.6 km/h working
  band -- and still cases, the rear wheel dropping into the gap while the front lands on
  the slope. Recorded as a characterization test, not as a target.
- Run termination: every reason, including the two safety caps, with an injected clock.
- The contact debounce measured against this track's real airborne phases.
- The pitch sign convention, which the HUD and the next task's telemetry both report signed.
- The ride key map, the HUD line, the real-time pacer and the camera's ride-mode tracking.
"""

from math import degrees
from dataclasses import replace
from typing import Any, Callable, Dict, List, NamedTuple, Tuple

import mujoco
import numpy as np
import pytest

from bike_sim.sim.camera import CameraManager
from bike_sim.sim.input_handler import PlaygroundInputHandler
from bike_sim.sim.ride.contacts import CONTACT_DROPOUT_STEPS, TerrainContactQuery
from bike_sim.sim.ride.cruise import (
    KMH_PER_MPS,
    MAX_TARGET_SPEED_KMH,
    MIN_TARGET_SPEED_KMH,
)
from bike_sim.sim.ride.hud import (
    PITCH_LEVEL,
    PITCH_NOSE_DOWN,
    PITCH_NOSE_UP,
    RideHUD,
    nearest_obstacle,
    pitch_label,
)
from bike_sim.sim.ride.input import BRAKE_STRENGTH_STEP, TARGET_SPEED_STEP_KMH
from bike_sim.physics.tyre import TyreConfig
from bike_sim.sim.ride.termination import (
    REASON_CRASH,
    REASON_END_OF_TRACK,
    REASON_STEP_CAP,
    REASON_WALL_CLOCK_CAP,
    STEP_CAP_SAFETY_FACTOR,
    RunLimits,
    RunTerminator,
)
from bike_sim.sim.ride.session import DEFAULT_BRAKE_STRENGTH, RideSession
from bike_sim.sim.ride.viewer import RealTimePacer
from bike_sim.sim.ride.virtual_rider import (
    CAUSE_PITCH_OVER,
    CRASH_PITCH_LIMIT_DEG,
    PITCH_MOMENT_CEILING_NM,
    CrashEvent,
)
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset

TARGET_SPEED_KMH = 25.0
TARGET_SPEED_MPS = TARGET_SPEED_KMH / KMH_PER_MPS
RIDE_TIMESTEP_S = 0.0005

# Kicker geometry of the shipped preset: a 1.4 m ramp from 57.0 m, so the lip is at 58.4 m; a
# 2.5 m gap; then the 20 degree landing slope from 60.9 m to 65.9 m (docs/RIDE.md section 2).
KICKER_LIP_M = 58.4
GAP_END_M = 60.9
LANDING_END_M = 65.9

# The working band section 2 states for the kicker, in km/h. Below it the bike cases.
KICKER_BAND_FLOOR_KMH = 20.9
KICKER_BAND_CEILING_KMH = 30.6

# The suspension's own joint ranges, and the excursions past them that this run produces.
#
# MuJoCo's ride-mode joint limits are deliberately soft (`solreflimit="0.01 1"`): they are the
# final barrier, not the bottom-out mechanism (docs/RIDE.md section 10), so the coordinate
# passes the limit under a hard enough load. Measured over this traverse: the fork reaches
# 184.399 mm on 55 of 34 827 steps, peaking where the kicker's landing slope meets the flat at
# 65.9 m, and the shock never tops out -- the rebound ceiling keeps it inside its range
# (minimum +0.156 mm, on the kicker touchdown). The tolerances below are those
# measurements with a small margin, not a number chosen to make the assertion pass.
FORK_TRAVEL_LIMIT_MM = 180.0
SHOCK_STROKE_LIMIT_MM = 65.0
FORK_OVERTRAVEL_TOLERANCE_MM = 5.0
SHOCK_TOPOUT_TOLERANCE_MM = 7.0

# Windows over which the traverse is judged, in metres of track position.
#
# The bike leaves the solved equilibrium at rest and needs about 10 m to reach the cruise
# target, so no speed window may start before that. The runout is the one stretch of this track
# that is level, ridden at cruise, and not inside the acceleration ramp -- where the drive
# torque wheelies the front wheel for a genuine 71 ms -- so it is the only place a lost contact
# row can only be a collision artifact.
CRUISE_WINDOW_M: Tuple[float, float] = (10.0, 115.0)
RUNOUT_WINDOW_M: Tuple[float, float] = (96.0, 115.0)
DROP_WINDOW_M: Tuple[float, float] = (51.5, 55.0)
KICKER_WINDOW_M: Tuple[float, float] = (56.5, 66.5)
ROCK_GARDEN_WINDOWS: Tuple[Tuple[float, float], ...] = ((26.0, 34.0), (84.0, 95.0))


class Traverse(NamedTuple):
    """Per-step trace of the full `enduro_aggressive` traverse, one array per channel."""

    outcome: Any
    x_m: np.ndarray
    speed_mps: np.ndarray
    pitch_rad: np.ndarray
    pitch_rate_radps: np.ndarray
    fork_mm: np.ndarray
    shock_mm: np.ndarray
    gate_front: np.ndarray
    gate_rear: np.ndarray
    raw_front: np.ndarray
    raw_rear: np.ndarray
    moment_nm: np.ndarray
    rider_active: np.ndarray
    front_x_m: np.ndarray
    rear_x_m: np.ndarray

    def window(self, lo_m: float, hi_m: float) -> np.ndarray:
        """Returns the boolean mask of samples with track position in [lo_m, hi_m]."""
        return (self.x_m >= lo_m) & (self.x_m <= hi_m)

    @property
    def gate_airborne(self) -> np.ndarray:
        """Steps on which the debounced gate reports both wheels off the ground."""
        return ~self.gate_front & ~self.gate_rear

    @property
    def raw_airborne(self) -> np.ndarray:
        """Steps on which neither wheel owns a loaded contact row, debouncing aside."""
        return ~self.raw_front & ~self.raw_rear


def _runs(mask: np.ndarray) -> List[Tuple[int, int]]:
    """
    Finds the maximal runs of consecutive True values in a boolean mask.

    Args:
        mask: Boolean array to scan.

    Returns:
        List of (start index, length) pairs, in order.
    """
    out: List[Tuple[int, int]] = []
    i, n = 0, len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j < n and mask[j]:
                j += 1
            out.append((i, j - i))
            i = j
        else:
            i += 1
    return out


def _longest_run(mask: np.ndarray) -> int:
    """Returns the length of the longest True run in a mask, or 0 if there is none."""
    runs = _runs(mask)
    return max((length for _, length in runs), default=0)


def _in_any_window(x_m: np.ndarray, windows: Tuple[Tuple[float, float], ...]) -> np.ndarray:
    """Returns the mask of samples falling inside any of the given position windows."""
    mask = np.zeros_like(x_m, dtype=bool)
    for lo, hi in windows:
        mask |= (x_m >= lo) & (x_m <= hi)
    return mask


@pytest.fixture(scope="module")
def sim() -> RideSimulation:
    """
    Compiles the full 115 m default preset once: compilation and the sag solve are slow.

    The aggressive preset is ridden by the **lumped** rider on purpose: it has a 600 mm drop
    and a kicker, and everything this module pins about the flight -- the virtual rider's
    moment, the touchdown pitch -- is the standing rigid rider's behaviour (docs/RIDE.md
    section 7). The seated rider is a rough-road model and is exercised elsewhere.
    """
    return RideSimulation(
        track=get_preset("enduro_aggressive"), target_speed_kmh=TARGET_SPEED_KMH, rider="lumped"
    )


@pytest.fixture(scope="module")
def traverse(sim: RideSimulation) -> Traverse:
    """
    Rides the whole default preset from the start of the track, recording every step.

    A second contact query with the debounce disabled runs alongside the simulation's own, so
    the tests can compare the debounced gate the controllers see with the raw collision signal.

    `rider_active` is recorded rather than derived from the contact channels because the
    stabilizer is gated on the snapshot taken at the *end of the previous step* -- see
    `RideSimulation.step` -- so the moment recorded at index i belongs to the contacts recorded
    at index i - 1, and comparing the two at the same index is off by one timestep.
    """
    sim.reset()
    raw_query = TerrainContactQuery(sim.model, dropout_steps=0)
    front_geom = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_GEOM, "geom_front_contact")
    rear_geom = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_GEOM, "geom_rear_contact")
    pitch_dofadr = sim.stabilizer.pitch_dofadr
    channels: List[Tuple[float, ...]] = []

    def record(s: RideSimulation) -> None:
        raw = raw_query.query(s.model, s.data)
        channels.append(
            (
                s.position_m,
                s.speed_mps,
                s.pitch_rad,
                float(s.data.qvel[pitch_dofadr]),
                s.fork_travel_mm,
                s.shock_stroke_mm,
                float(s.contacts.front_in_contact),
                float(s.contacts.rear_in_contact),
                float(raw.front_in_contact),
                float(raw.rear_in_contact),
                s.stabilizer.moment_nm,
                float(s.stabilizer.active),
                float(s.data.geom_xpos[front_geom][0]),
                float(s.data.geom_xpos[rear_geom][0]),
            )
        )

    outcome = sim.run(on_step=record)
    trace = np.array(channels, dtype=float)
    return Traverse(
        outcome=outcome,
        x_m=trace[:, 0],
        speed_mps=trace[:, 1],
        pitch_rad=trace[:, 2],
        pitch_rate_radps=trace[:, 3],
        fork_mm=trace[:, 4],
        shock_mm=trace[:, 5],
        gate_front=trace[:, 6].astype(bool),
        gate_rear=trace[:, 7].astype(bool),
        raw_front=trace[:, 8].astype(bool),
        raw_rear=trace[:, 9].astype(bool),
        moment_nm=trace[:, 10],
        rider_active=trace[:, 11].astype(bool),
        front_x_m=trace[:, 12],
        rear_x_m=trace[:, 13],
    )


@pytest.fixture(scope="module")
def flat_sim() -> RideSimulation:
    """A featureless-track simulation for the interactive units, with the marker geoms present."""
    return RideSimulation(
        track=get_preset("flat"), target_speed_kmh=TARGET_SPEED_KMH, debug_markers=True, rider="lumped"
    )


@pytest.fixture
def session(flat_sim: RideSimulation) -> RideSession:
    """A fresh ride session over the shared flat simulation, at the default cruise target."""
    flat_sim.cruise.target_speed_kmh = TARGET_SPEED_KMH
    return RideSession(flat_sim, show_telemetry=False)


# --------------------------------------------------------------------------------------
# The full traverse
# --------------------------------------------------------------------------------------


def test_enduro_run_terminates_at_the_end_of_the_track(sim: RideSimulation, traverse: Traverse):
    """
    The bike rides all 115 m of the default preset and the run says so explicitly.

    Wall clock and step count are recorded rather than asserted: 34 827 steps of 17.414 s of
    simulated time in about 3.5 s of wall clock, i.e. roughly 5 times real time.
    """
    assert traverse.outcome.reason == REASON_END_OF_TRACK
    assert traverse.outcome.completed
    assert traverse.outcome.crash is None
    assert sim.crash is None
    assert traverse.outcome.position_m >= sim.track.length_m
    assert traverse.outcome.steps == len(traverse.x_m)


def test_traverse_is_worked_by_every_obstacle(traverse: Traverse):
    """
    Each major feature drives the fork well past its sag, so the bike really rode the track.

    This is also the check that `hfield_data` was filled before the run: on an unfilled field
    the road is dead flat and the fork never leaves the 73 mm sag band.
    """
    sag_mm = traverse.fork_mm[traverse.window(10.0, 11.0)].mean()
    assert 60.0 < sag_mm < 90.0
    for lo, hi, floor_mm in (
        (12.0, 20.1, 110.0),   # braking bumps
        (21.5, 23.5, 110.0),   # 90 mm square edge
        (42.0, 47.0, 150.0),   # G-out
        (51.5, 55.0, 150.0),   # 600 mm drop
        (56.5, 66.5, 150.0),   # kicker
        (84.0, 95.0, 150.0),   # second rock garden
    ):
        peak_mm = traverse.fork_mm[traverse.window(lo, hi)].max()
        assert peak_mm > floor_mm, f"fork peaked at only {peak_mm:.1f} mm over {lo}-{hi} m"


def test_suspension_travel_stays_within_the_joint_limits(traverse: Traverse):
    """
    Neither coordinate leaves its range by more than the soft limits' measured excursion.

    Both suspension joints carry `solreflimit="0.01 1"` in ride mode, so a limit is a stiff
    spring rather than a wall and a hard enough impact pushes through it. Measured on this
    traverse: fork 184.399 mm against a 180 mm range (55 of 34 827 steps past the limit, the
    peak at the foot of the kicker's landing slope) and shock never below zero at all -- the
    raised rebound ceiling holds it inside its range (minimum +0.156 mm, at the kicker
    touchdown). The fork never goes negative and the shock never exceeds 65 mm.
    """
    assert traverse.fork_mm.min() >= 0.0
    assert traverse.fork_mm.max() <= FORK_TRAVEL_LIMIT_MM + FORK_OVERTRAVEL_TOLERANCE_MM
    assert traverse.shock_mm.max() <= SHOCK_STROKE_LIMIT_MM
    assert traverse.shock_mm.min() >= -SHOCK_TOPOUT_TOLERANCE_MM


def test_mean_speed_tracks_the_target_past_the_acceleration_ramp(traverse: Traverse):
    """
    Mean speed from 10 m to the finish is within 5 % of the 25 km/h target.

    Measured 24.63 km/h, 98.5 % of target, against instantaneous extremes of 16.5 and
    35.3 km/h -- the low recovering from the kicker's case, the high on the 600 mm drop.
    """
    cruising = traverse.window(*CRUISE_WINDOW_M)
    assert float(traverse.speed_mps[cruising].mean()) == pytest.approx(TARGET_SPEED_MPS, rel=0.05)


def test_pitch_stays_inside_the_crash_limit(traverse: Traverse):
    """
    Attitude never reaches the 60 degree crash threshold anywhere on the track.

    Measured range -13.65 degrees nose-up, on the kicker's ramp, to +24.60 degrees nose-down,
    at the gap touchdown -- so the run clears the threshold by better than a factor of two.
    """
    assert np.abs(np.degrees(traverse.pitch_rad)).max() < CRASH_PITCH_LIMIT_DEG


def test_pitch_stabilizer_stays_inside_its_ceiling(traverse: Traverse):
    """No moment anywhere on the track exceeds the +/-80 N.m damage limit of section 7."""
    assert np.abs(traverse.moment_nm).max() <= PITCH_MOMENT_CEILING_NM


def test_pitch_stabilizer_is_silent_whenever_a_wheel_is_in_contact(traverse: Traverse):
    """
    The moment is exactly zero on every step the stabilizer's own gate calls grounded.

    The gate is also pinned to the snapshot it is documented to use: the rider's `active` flag
    at each step is the airborne state of the contacts queried at the *end of the previous*
    step, which is what makes the moment and the contact channels one timestep apart.
    """
    assert np.all(traverse.moment_nm[~traverse.rider_active] == 0.0)
    assert np.array_equal(traverse.rider_active[1:], traverse.gate_airborne[:-1])


def test_pitch_stabilizer_saturates_rather_than_regulating_over_the_kicker(traverse: Traverse):
    """
    Over the kicker the stabilizer runs against its ceiling one-sided, so its gains never act.

    This is the answer to the question the gains were left open for. Across the 320.5 ms flight
    off the ramp the moment sits at exactly -80 N.m for the first 400 of 641 steps, then fades
    once the pitch rate is arrested, and never changes sign: the loop is saturated, not
    oscillating, so `kd` (0.54 of critical) is not the knob and raising it would change nothing.
    What the ceiling buys is 20.5 N.m.s of angular impulse against a launch pitch rate of
    1.11 rad/s on a 38.6 kg.m^2 coordinate. The authority is the limit, as section 7 says it
    is; the gains are not.
    """
    start, length = _kicker_flight(traverse)
    moment = traverse.moment_nm[start : start + length]

    assert moment.size > 300
    saturated = np.abs(moment) >= PITCH_MOMENT_CEILING_NM - 1e-9
    assert saturated.mean() > 0.5
    assert not saturated[int(0.75 * moment.size) :].any()  # the tail is the release, not ringing
    nonzero = np.sign(moment[moment != 0.0])
    assert np.all(nonzero == nonzero[0]), "the moment reversed, so the loop is ringing after all"


def test_the_stabilizer_arrests_the_launch_rotation_before_touchdown(traverse: Traverse):
    """
    The ceiling now has enough flight time to stop the rotation the ramp imparts.

    Measured over the flight: the bike leaves the ramp at +3.01 degrees nose-down rotating
    nose-down at 1.109 rad/s; the pegged moment caps the dive at +10.15 degrees near 60.0 m,
    the rate crosses zero, and the bike touches down at +8.26 degrees rotating back at
    -0.41 rad/s. Arrested and half-recovered, not settled -- it never rings, and more
    authority than +/-80 N.m is still a spec change, not a gain adjustment.
    """
    start, length = _kicker_flight(traverse)
    end = start + length - 1

    assert traverse.pitch_rate_radps[start] > 0.8            # leaves the lip rotating nose-down
    assert traverse.pitch_rate_radps[start:end].min() < 0.0  # arrested and reversed in flight
    assert traverse.pitch_rad[end] < traverse.pitch_rad[start:end].max()  # already recovering
    assert traverse.pitch_rate_radps[end] < 0.0                          # still coming back


# --------------------------------------------------------------------------------------
# The kicker
# --------------------------------------------------------------------------------------


def _kicker_flight(traverse: Traverse) -> Tuple[int, int]:
    """
    Finds the flight the bike takes off the kicker's ramp.

    Args:
        traverse: The recorded traverse.

    Returns:
        Tuple of (first index, length) of the longest run with both wheels genuinely unloaded
        that begins on the kicker's ramp.

    Raises:
        AssertionError: If no such flight exists, which would mean the bike never left the ramp.
    """
    on_ramp = (traverse.x_m >= 56.5) & (traverse.x_m <= KICKER_LIP_M + 0.5)
    candidates = [(i, n) for i, n in _runs(traverse.raw_airborne) if on_ramp[i]]
    assert candidates, "the bike never left the ground on the kicker's ramp"
    return max(candidates, key=lambda run: run[1])


def test_kicker_launches_the_bike_off_the_lip(traverse: Traverse):
    """
    The front wheel leaves the ground at the lip and the whole bike flies for over 300 ms.

    Measured: the front wheel's own airborne phase starts at 58.327 m, 73 mm short of the
    58.4 m lip -- the last cell of the ramp already kicks it clear -- and both wheels are off
    the ground for 641 steps (320.5 ms), covering 2.09 m of track.
    """
    front_air = _runs(~traverse.raw_front)
    lip_takeoffs = [
        traverse.front_x_m[i]
        for i, n in front_air
        if n > 100 and abs(traverse.front_x_m[i] - KICKER_LIP_M) < 0.5
    ]
    assert lip_takeoffs, "the front wheel never left the ground at the lip"
    assert abs(lip_takeoffs[0] - KICKER_LIP_M) < 0.10

    start, length = _kicker_flight(traverse)
    assert length * RIDE_TIMESTEP_S > 0.100
    assert traverse.x_m[start + length - 1] - traverse.x_m[start] > 1.0


def test_kicker_cases_into_the_gap_at_the_default_speed(traverse: Traverse):
    """
    The bike still does not clear the gap at the default speed, but the case is now split
    across the axles: the front wheel reaches the slope while the rear drops in.

    Section 2's 20.9-30.6 km/h band is a point-mass ballistic bound, and the launch is now
    inside it: measured off the lip at 22.1 km/h, the firmer rebound having carried more
    speed through the ramp entry. The bike is not a point, though -- after the 320 ms flight
    the rear wheel touches down at 60.41 m, half a metre inside the gap, while the front is
    already past the far edge and lands on the slope at 62.05 m.

    The run survives the case: no crash.
    """
    start, length = _kicker_flight(traverse)
    launch_speed_kmh = traverse.speed_mps[start] * KMH_PER_MPS
    end = start + length - 1

    assert KICKER_BAND_FLOOR_KMH < launch_speed_kmh < KICKER_BAND_CEILING_KMH
    assert KICKER_LIP_M < traverse.rear_x_m[end] < GAP_END_M
    assert traverse.front_x_m[end] > GAP_END_M
    assert traverse.outcome.crash is None


def test_the_front_wheel_does_reach_the_landing_slope(traverse: Traverse):
    """
    Somewhere on the landing the front wheel is airborne and comes down on the slope.

    Kept separate from the casing test above on purpose: the bike does end up on the 20 degree
    slope between 60.9 and 65.9 m, and a reader of the casing result should not conclude that
    the landing is never used. Measured touchdown 62.045 m -- the kicker flight itself now
    carries the front wheel across the gap line, while the rear drops in behind it.
    """
    landings = [
        traverse.front_x_m[i + n - 1]
        for i, n in _runs(~traverse.raw_front)
        if n > 100 and GAP_END_M <= traverse.front_x_m[i + n - 1] <= LANDING_END_M
    ]
    assert landings, "the front wheel never touched down on the landing slope"


# --------------------------------------------------------------------------------------
# The contact debounce, measured against this track
# --------------------------------------------------------------------------------------


def test_contact_flicker_on_flat_road_is_far_inside_the_debounce_window(traverse: Traverse):
    """
    On level road at cruise the collision artifact is one step long, and fully suppressed.

    Measured over the runout, the one stretch of this track that is level, at cruise, and clear
    of the acceleration ramp: both wheels lose their contact row simultaneously on 1.65 % of its
    5 394 steps and never for more than a single step; the front alone on 12.9 % of steps for at
    most two; the rear alone on 14.9 % for at most one. The debounced gate reports both wheels
    in contact on every one of those steps. A separate run of the `flat` preset at 45 km/h, the
    top of the section 6 band, raises the both-wheels rate to 6.2 % but the longest run only to
    two steps -- the artifact's rate scales with speed and its duration does not, which is what
    makes a fixed window sound against a speed-dependent artifact.
    """
    runout = traverse.window(*RUNOUT_WINDOW_M)
    assert _longest_run(traverse.raw_airborne & runout) <= CONTACT_DROPOUT_STEPS // 3
    assert (traverse.raw_airborne & runout).sum() > 0, "no dropouts at all: the artifact is gone?"
    assert not np.any(traverse.gate_airborne & runout)
    assert np.all(traverse.gate_front[runout])
    assert np.all(traverse.gate_rear[runout])


def test_the_drop_and_kicker_flights_are_far_outside_the_debounce_window(traverse: Traverse):
    """
    The real flights are an order of magnitude longer than the window that suppresses flicker.

    Measured longest both-wheels-unloaded run: 262 steps (131 ms) at the 600 mm drop and 391
    steps (195.5 ms) at the kicker, against the 10-step window.
    """
    for lo, hi in (DROP_WINDOW_M, KICKER_WINDOW_M):
        longest = _longest_run(traverse.raw_airborne & traverse.window(lo, hi))
        assert longest >= 10 * CONTACT_DROPOUT_STEPS, f"only {longest} steps over {lo}-{hi} m"


def test_the_debounce_window_overlaps_genuine_airborne_phases_over_the_rock_gardens(
    traverse: Traverse,
):
    """
    On rough ground the two populations overlap, so no fixed window separates them.

    This is the honest answer to "the window must stay well above the flicker and well below
    the shortest genuine flight". On flat road it holds with a factor of five and a factor of
    twenty-six respectively. On this track as a whole it does not: the both-wheels-unloaded run
    lengths form a continuum from 1 to 391 steps, 1 864 of the 1 900 runs are 10 steps or
    shorter and are therefore suppressed outright (7.0 % of the run's steps), and the shortest
    run that survives the window is 11 steps -- one step of margin, not an order of magnitude.

    The window is still the right call, because the cost of the overlap is bounded and small: a
    both-wheels phase shorter than 5 ms needs a take-off vertical velocity under 0.03 m/s, so
    what is being suppressed is a wheel skimming rather than a jump, and the price is that the
    drive torque stays on and the rider stays quiet for at most 5 ms longer than the raw signal
    would say. What is *not* sound is the claim of a clean separation, so this test pins the
    overlap rather than the claim.
    """
    rough = _in_any_window(traverse.x_m, ROCK_GARDEN_WINDOWS)
    lengths = np.array([n for i, n in _runs(traverse.raw_airborne) if rough[i]])

    assert lengths.size > 100
    mid_band = (lengths > 2) & (lengths <= CONTACT_DROPOUT_STEPS)
    assert mid_band.sum() > 10, "no runs between the flicker and the window: they do separate"
    assert lengths.max() > CONTACT_DROPOUT_STEPS


# --------------------------------------------------------------------------------------
# The pitch sign convention
# --------------------------------------------------------------------------------------


def test_positive_root_pitch_is_nose_down(sim: RideSimulation):
    """
    Positive `qpos[root_pitch]` puts the front axle below the rear, and negative above it.

    Nothing pinned this before. `root_pitch` is a hinge about world +Y, so a positive rotation
    tips the bike's forward axis toward the ground -- but the stabilizer and the crash detector
    are both sign-symmetric, so neither would notice if the convention flipped, while the HUD
    and the next task's telemetry both report pitch as a signed quantity.
    """
    model = sim.model
    data = mujoco.MjData(model)
    pitch_adr = sim.root_pitch_qposadr
    z_adr = int(model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_z")])
    front_site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "site_PFA")
    rear_site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "site_PRA")

    def axle_height_difference(pitch_rad: float) -> float:
        mujoco.mj_resetData(model, data)
        data.qpos[z_adr] = 1.0
        data.qpos[pitch_adr] = pitch_rad
        mujoco.mj_forward(model, data)
        return float(data.site_xpos[front_site][2] - data.site_xpos[rear_site][2])

    level = axle_height_difference(0.0)
    assert axle_height_difference(+0.2) < level - 0.2
    assert axle_height_difference(-0.2) > level + 0.2


# --------------------------------------------------------------------------------------
# Run termination
# --------------------------------------------------------------------------------------


class _FakeClock:
    """A hand-advanced monotonic clock, so the wall-clock cap can be tested without waiting."""

    def __init__(self) -> None:
        self.now_s = 0.0

    def __call__(self) -> float:
        return self.now_s


def test_run_limits_derive_the_step_cap_from_the_slowest_band_speed():
    """The cap is the remaining distance at 15 km/h, times the stated safety factor."""
    track = get_preset("enduro_aggressive")
    limits = RunLimits.for_track(track, timestep_s=RIDE_TIMESTEP_S, start_x_m=2.0)

    slowest_mps = MIN_TARGET_SPEED_KMH / KMH_PER_MPS
    expected = STEP_CAP_SAFETY_FACTOR * (track.length_m - 2.0) / (slowest_mps * RIDE_TIMESTEP_S)

    assert limits.finish_x_m == track.length_m
    assert limits.max_steps == int(expected)


def test_run_limits_reject_a_run_that_cannot_move_forward():
    """A run starting at or past the end of the track is a mis-specified experiment."""
    track = get_preset("single_edge")
    with pytest.raises(ValueError, match="at or past the end"):
        RunLimits.for_track(track, timestep_s=RIDE_TIMESTEP_S, start_x_m=track.length_m)


def test_run_limits_accept_a_power_limited_climb_floor():
    """A slow-but-moving climb below the cruise band gets a longer budget."""
    track = get_preset("enduro_aggressive")
    flat = RunLimits.for_track(track, timestep_s=RIDE_TIMESTEP_S, start_x_m=2.0)
    climb = RunLimits.for_track(
        track, timestep_s=RIDE_TIMESTEP_S, start_x_m=2.0, min_speed_mps=2.5
    )
    expected = STEP_CAP_SAFETY_FACTOR * (track.length_m - 2.0) / (2.5 * RIDE_TIMESTEP_S)
    assert climb.max_steps == int(expected) > flat.max_steps
    with pytest.raises(ValueError, match="min_speed_mps"):
        RunLimits.for_track(track, timestep_s=RIDE_TIMESTEP_S, start_x_m=2.0, min_speed_mps=0.)


def test_default_limits_size_the_cap_from_the_climb_power_budget():
    """On a graded track an effort drive cannot hold 15 km/h, so the cap must not assume it."""
    from types import SimpleNamespace
    from bike_sim.physics.model_config import SimulationPhysicsConfig
    from bike_sim.physics.physical_config import PhysicalDriveConfig
    from bike_sim.terrain.research import build_research_track

    drive = PhysicalDriveConfig(human_torque_nm=20.)
    sim = SimpleNamespace(
        track=build_research_track("rough_uphill"),
        physics_config=SimulationPhysicsConfig(
            "physical", drive_mode="crank_effort", drive=drive),
        model=SimpleNamespace(
            body_mass=np.array([90., 15.]),
            opt=SimpleNamespace(timestep=.00125)),
        start_x_m=2.)
    limits = RideSimulation.default_limits(sim)

    # 500 W shaft + ~94 W sustained human against 22% on a 105 kg system.
    grade = .22
    floor = (drive.assist.max_power
             + drive.human_torque_nm*drive.pedaling.mash_cadence_rpm*np.pi/30.) \
        / (105.*9.81*(grade/np.sqrt(1.+grade*grade)))
    expected = STEP_CAP_SAFETY_FACTOR * 33. / (floor * .00125)
    assert limits.max_steps == int(expected)
    cruise = RunLimits.for_track(sim.track, timestep_s=.00125, start_x_m=2.)
    assert limits.max_steps > cruise.max_steps

    # A flat track keeps the cruise-band floor, and a coast drive adds no power.
    flat_track = SimpleNamespace(
        track=get_preset("flat"),
        physics_config=sim.physics_config,
        model=sim.model, start_x_m=2.)
    flat_limits = RideSimulation.default_limits(flat_track)
    flat_expected = STEP_CAP_SAFETY_FACTOR * (flat_track.track.length_m - 2.) \
        / (MIN_TARGET_SPEED_KMH / KMH_PER_MPS * .00125)
    assert flat_limits.max_steps == int(flat_expected)


@pytest.mark.parametrize("bad", [{"finish_x_m": 0.0}, {"max_steps": 0}, {"max_wall_clock_s": 0.0}])
def test_run_limits_reject_a_non_positive_bound(bad: Dict[str, float]):
    """Every bound must be positive; a zero bound would terminate every run immediately."""
    kwargs: Dict[str, Any] = {"finish_x_m": 100.0, "max_steps": 1000, "max_wall_clock_s": 10.0}
    kwargs.update(bad)
    with pytest.raises(ValueError):
        RunLimits(**kwargs)


def test_terminator_reports_the_end_of_the_track():
    """Reaching the finish position ends the run, and the reason names it."""
    terminator = RunTerminator(RunLimits(finish_x_m=40.0, max_steps=1000, max_wall_clock_s=10.0))
    assert terminator.reason(position_m=39.999, steps=10) is None
    assert terminator.reason(position_m=40.0, steps=10) == REASON_END_OF_TRACK


def test_terminator_reports_a_crash_even_on_the_finish_line():
    """
    A crash outranks every other reason.

    A bike that goes down as it crosses the line has crashed, and reporting that run as
    completed would hide exactly the failure the detector exists to catch.
    """
    terminator = RunTerminator(RunLimits(finish_x_m=40.0, max_steps=1000, max_wall_clock_s=10.0))
    crash = CrashEvent(cause=CAUSE_PITCH_OVER, time_s=1.0, position_m=40.0, pitch_rad=1.2)

    assert terminator.reason(position_m=40.0, steps=10, crash=crash) == REASON_CRASH
    outcome = terminator.outcome(REASON_CRASH, 10, 1.0, 40.0, crash)
    assert not outcome.completed
    assert outcome.crash is crash
    assert CAUSE_PITCH_OVER in outcome.describe()
    assert REASON_CRASH in outcome.describe()


def test_terminator_reports_the_step_cap():
    """A bike that is not making progress fails on the step cap rather than running forever."""
    terminator = RunTerminator(RunLimits(finish_x_m=40.0, max_steps=100, max_wall_clock_s=10.0))
    assert terminator.reason(position_m=5.0, steps=99) is None
    assert terminator.reason(position_m=5.0, steps=100) == REASON_STEP_CAP


def test_terminator_reports_the_wall_clock_cap():
    """Real time is capped independently of simulated progress."""
    clock = _FakeClock()
    terminator = RunTerminator(
        RunLimits(finish_x_m=40.0, max_steps=10**9, max_wall_clock_s=2.0), clock=clock
    )
    terminator.start()

    clock.now_s = 1.999
    assert terminator.reason(position_m=5.0, steps=1) is None
    clock.now_s = 2.0
    assert terminator.reason(position_m=5.0, steps=1) == REASON_WALL_CLOCK_CAP
    assert terminator.wall_clock_s == pytest.approx(2.0)


def test_run_stops_on_the_step_cap_and_says_so(flat_sim: RideSimulation):
    """`RideSimulation.run` honours a step cap and reports it as the reason."""
    flat_sim.reset()
    outcome = flat_sim.run(
        limits=RunLimits(finish_x_m=flat_sim.track.length_m, max_steps=200, max_wall_clock_s=60.0)
    )

    assert outcome.reason == REASON_STEP_CAP
    assert not outcome.completed
    assert outcome.steps == 200
    assert outcome.sim_time_s == pytest.approx(200 * RIDE_TIMESTEP_S)


def test_run_checks_termination_before_it_steps(flat_sim: RideSimulation):
    """A run whose limits are already met returns without taking a step."""
    flat_sim.reset()
    steps_before = flat_sim.steps
    outcome = flat_sim.run(
        limits=RunLimits(finish_x_m=1.0, max_steps=1000, max_wall_clock_s=60.0)
    )

    assert outcome.reason == REASON_END_OF_TRACK
    assert flat_sim.steps == steps_before


# --------------------------------------------------------------------------------------
# The real-time pacer
# --------------------------------------------------------------------------------------


def test_pacer_takes_one_step_per_timestep():
    """A frame exactly one timestep long advances the simulation exactly one step."""
    pacer = RealTimePacer(RIDE_TIMESTEP_S)
    assert pacer.steps_for(RIDE_TIMESTEP_S) == 1


def test_pacer_carries_the_sub_timestep_remainder():
    """Three half-steps become one step and then two, rather than none at all."""
    pacer = RealTimePacer(RIDE_TIMESTEP_S)
    assert [pacer.steps_for(RIDE_TIMESTEP_S * 0.75) for _ in range(4)] == [0, 1, 1, 1]


def test_pacer_caps_the_arrears_it_will_chase():
    """A stalled frame loses the excess simulated time instead of sprinting through it."""
    pacer = RealTimePacer(RIDE_TIMESTEP_S, max_catchup_s=0.05)
    assert pacer.steps_for(10.0) == int(0.05 / RIDE_TIMESTEP_S)
    assert pacer.steps_for(0.0) == 0


def test_pacer_ignores_a_clock_that_goes_backwards():
    """A negative elapsed time contributes nothing rather than rewinding the accumulator."""
    pacer = RealTimePacer(RIDE_TIMESTEP_S)
    pacer.steps_for(RIDE_TIMESTEP_S * 0.75)
    assert pacer.steps_for(-1.0) == 0
    assert pacer.steps_for(RIDE_TIMESTEP_S * 0.75) == 1


@pytest.mark.parametrize("kwargs", [{"timestep_s": 0.0}, {"max_catchup_s": 0.0001}])
def test_pacer_rejects_limits_that_would_freeze_the_run(kwargs: Dict[str, float]):
    """A non-positive timestep or a cap shorter than one step would return zero steps forever."""
    args: Dict[str, float] = {"timestep_s": RIDE_TIMESTEP_S, "max_catchup_s": 0.05}
    args.update(kwargs)
    with pytest.raises(ValueError):
        RealTimePacer(**args)


# --------------------------------------------------------------------------------------
# The camera
# --------------------------------------------------------------------------------------


class _StubCam:
    """The three camera fields `CameraManager.update_viewer` writes."""

    def __init__(self) -> None:
        self.lookat = np.zeros(3)
        self.azimuth = 0.0
        self.elevation = 0.0
        self.distance = 0.0


class _StubViewer:
    """A viewer handle stand-in exposing only `cam`."""

    def __init__(self) -> None:
        self.cam = _StubCam()


def test_camera_tracks_x_exactly_with_no_lag():
    """
    Longitudinal tracking is exact, so the bike cannot drift out of frame at riding speed.

    At the ride mode's 6.94 m/s a first-order filter at alpha = 0.08 would settle 1.4 m behind
    the bike; X is therefore assigned, not filtered, and only Z is smoothed.
    """
    camera = CameraManager(default_mode="2d")
    viewer = _StubViewer()
    offset = CameraManager.PRESETS["2d"]["lookat_offset"]

    camera.update_viewer(viewer, bike_x=0.0, bike_z=0.0)
    for bike_x in (1.0, 5.0, 40.0, 115.0):
        camera.update_viewer(viewer, bike_x=bike_x, bike_z=0.0)
        assert viewer.cam.lookat[0] == pytest.approx(bike_x + offset[0])


def test_camera_smooths_z_so_the_view_does_not_jolt_on_every_bump():
    """A vertical step is followed gradually, at the manager's smoothing rate."""
    alpha = 0.08
    camera = CameraManager(default_mode="2d", smoothing_alpha=alpha)
    viewer = _StubViewer()
    offset = CameraManager.PRESETS["2d"]["lookat_offset"]

    camera.update_viewer(viewer, bike_x=0.0, bike_z=0.0)
    start_z = float(viewer.cam.lookat[2])
    camera.update_viewer(viewer, bike_x=0.0, bike_z=1.0)

    target_z = 1.0 + offset[2]
    assert viewer.cam.lookat[2] == pytest.approx(start_z + alpha * (target_z - start_z))
    assert viewer.cam.lookat[2] < target_z


# --------------------------------------------------------------------------------------
# The HUD
# --------------------------------------------------------------------------------------


def test_hud_line_reports_every_declared_channel(flat_sim: RideSimulation):
    """
    The line carries position, both speeds, both travels in mm and per cent, both shaft
    velocities, the brake state, per-wheel contact, pitch with its direction, and wheel power.
    """
    flat_sim.cruise.target_speed_kmh = TARGET_SPEED_KMH
    hud = RideHUD(flat_sim.model)
    line = hud.line(flat_sim, braking=True, brake_strength=0.4)

    for token in ("RIDE", "flat", "m ", "km/h", "Fork:", "Shock:", "mm", "%", "m/s"):
        assert token in line
    assert "Brake:ON  40%" in line
    assert "Grip:" in line
    assert "deg" in line and pitch_label(flat_sim.pitch_rad) in line
    assert "Pwr:" in line and "W" in line
    assert f"{flat_sim.position_m:7.2f}" in line
    assert f"{TARGET_SPEED_KMH:4.1f}" in line


def test_hud_line_names_the_nearest_obstacle(sim: RideSimulation):
    """On a track with features the line reports the closest one and its signed distance."""
    hud = RideHUD(sim.model)
    line = hud.line(sim)
    label, distance_m = nearest_obstacle(sim.track, sim.position_m)

    assert label in line
    assert f"{distance_m:+.1f}m" in line


def test_hud_line_reports_the_brake_toggle_state(flat_sim: RideSimulation):
    """A released brake still shows the strength the toggle would apply, marked as off."""
    hud = RideHUD(flat_sim.model)
    assert "Brake:off( 60%)" in hud.line(flat_sim, braking=False, brake_strength=0.6)


def test_pneumatic_hud_shows_pressure_slip_and_a_rim_strike_flash():
    sim = RideSimulation(
        track=get_preset("flat"),
        tyre=TyreConfig(model="pneumatic"),
        rider="none",
    )
    hud = RideHUD(sim.model)
    line = hud.line(sim)
    assert "Tyre F/R 1.50/1.70bar" in line
    assert "κ" in line
    assert "RIM STRIKE" not in line

    sim.tyre_applier.front_outputs = replace(
        sim.tyre_applier.front_outputs, rim_strike_active=True
    )
    assert "RIM STRIKE" in hud.line(sim)


@pytest.mark.parametrize(
    "pitch_rad,expected",
    [(0.2, PITCH_NOSE_DOWN), (-0.2, PITCH_NOSE_UP), (0.0, PITCH_LEVEL)],
)
def test_hud_labels_the_pitch_direction(pitch_rad: float, expected: str):
    """A signed angle in degrees is ambiguous, so the HUD spells the direction out."""
    assert pitch_label(pitch_rad) == expected


def test_nearest_obstacle_names_the_closest_feature_with_a_signed_distance():
    """Distance is positive ahead of the bike and negative behind it."""
    track = get_preset("enduro_aggressive")

    label, distance_m = nearest_obstacle(track, 21.0)
    assert "SquareEdge" in label
    assert distance_m == pytest.approx(1.0)

    label, distance_m = nearest_obstacle(track, 23.0)
    assert "SquareEdge" in label
    assert distance_m == pytest.approx(-1.0)


def test_nearest_obstacle_is_none_on_a_featureless_track():
    """A track with no obstacles has no nearest one, rather than a made-up marker at zero."""
    assert nearest_obstacle(get_preset("flat"), 10.0) is None


def test_help_text_documents_the_new_ride_bindings():
    """The help screen names every binding that has no test-stand counterpart."""
    text = RideHUD.get_help_text()
    for token in ("W / S", "SPACE", ", / .", "Cruise Target", "Brake Strength", "TOGGLE",
                  "N / M", "; / '", "TYRES"):
        assert token in text


# --------------------------------------------------------------------------------------
# The key map
# --------------------------------------------------------------------------------------


class _StubPlayground:
    """The four bound methods `PlaygroundInputHandler` reads while building its map."""

    def __init__(self) -> None:
        self.toggle_rider: Callable[[], None] = lambda: None
        self.reset_state: Callable[[], None] = lambda: None
        self.toggle_debug_markers: Callable[[], None] = lambda: None
        self.print_help: Callable[[], None] = lambda: None


def test_ride_keys_are_the_playground_keys_with_the_stand_only_ones_replaced(
    session: RideSession,
):
    """
    Ride mode inherits the whole test-stand key map bar the bindings a rolling bike cannot use.

    The stand's two arrow keys command fork and rear travel directly, which a free-rolling bike
    has no equivalent of; the brake-strength pair takes their place. The stand's rider toggle
    goes too: the ride rider is a compile-time variant (`--rider`), not an in-place mass swap.
    Everything else -- both dampers, the air spring, the camera, the markers, the preset, the
    telemetry stream, reset and help -- keeps its stand binding, `W`/`S` and `Space` keep
    their keys while changing meaning, `E` cycles the assist mode, and `V` requests one
    crank-reposition maneuver (drive.motor_clutch only) -- both ride-mode additions.
    """
    playground_keys = set(PlaygroundInputHandler(_StubPlayground())._dispatch_map)
    arrow_keys = {264, 265}
    rider_toggle_keys = {66, 98}
    brake_strength_keys = {44, 46}
    tyre_pressure_keys = {78, 110, 77, 109, 59, 39}
    assist_cycle_keys = {69, 101}
    crank_reposition_keys = {86, 118}

    assert set(session.input.bound_keycodes) == (
        (playground_keys - arrow_keys - rider_toggle_keys)
        | brake_strength_keys
        | tyre_pressure_keys
        | assist_cycle_keys
        | crank_reposition_keys
    )


def test_tyre_pressure_keys_explain_sphere_mode(capsys: pytest.CaptureFixture, session: RideSession):
    assert session.sim.tyre_applier is None
    assert session.input.handle_key(ord("N")) is True
    assert "require --tyre-model pneumatic" in capsys.readouterr().out


def test_tyre_pressure_keys_change_and_clamp_both_wheels(capsys: pytest.CaptureFixture):
    sim = RideSimulation(
        track=get_preset("flat"),
        tyre=TyreConfig(model="pneumatic"),
        rider="none",
    )
    session = RideSession(sim, show_telemetry=False)
    front = sim.tyre_applier.front_tyre
    rear = sim.tyre_applier.rear_tyre

    session.handle_key(ord("N"))
    session.handle_key(ord("M"))
    session.handle_key(ord(";"))
    session.handle_key(ord("'"))
    assert (front.tyre.pressure_bar, rear.tyre.pressure_bar) == pytest.approx((1.50, 1.70))

    front.set_pressure(0.8)
    session.handle_key(ord("N"))
    assert front.tyre.pressure_bar == pytest.approx(0.8)
    session.handle_key(ord("M"))
    assert front.tyre.pressure_bar == pytest.approx(0.85)
    assert "0.85 bar" in capsys.readouterr().out


def test_unmapped_key_is_reported_as_unhandled(session: RideSession):
    """An unbound keycode changes nothing and says it was not handled."""
    assert session.input.handle_key(999) is False


def test_w_and_s_step_the_cruise_target(session: RideSession):
    """`W` and `S` move the target by exactly one km/h."""
    start_kmh = session.sim.cruise.target_speed_kmh

    session.handle_key(ord("W"))
    assert session.sim.cruise.target_speed_kmh == pytest.approx(start_kmh + TARGET_SPEED_STEP_KMH)
    session.handle_key(ord("s"))
    assert session.sim.cruise.target_speed_kmh == pytest.approx(start_kmh)


def test_cruise_target_keys_clamp_at_the_band_edges(session: RideSession):
    """
    Walking into the edge of the 15-45 km/h band clamps rather than raising.

    The controller's setter rejects an out-of-band target, because a mis-specified experiment
    should fail loudly -- but a key press that walks into the edge is not one, and an exception
    raised inside the viewer's key handler would take the run down with it.
    """
    for _ in range(40):
        session.handle_key(ord("W"))
    assert session.sim.cruise.target_speed_kmh == pytest.approx(MAX_TARGET_SPEED_KMH)

    for _ in range(60):
        session.handle_key(ord("S"))
    assert session.sim.cruise.target_speed_kmh == pytest.approx(MIN_TARGET_SPEED_KMH)


def test_space_toggles_the_brakes(session: RideSession):
    """Braking is a toggle, because a passive viewer delivers presses and not key state."""
    assert session.braking is False
    session.handle_key(32)
    assert session.braking is True
    session.handle_key(32)
    assert session.braking is False


def test_brake_strength_keys_step_and_clamp(session: RideSession):
    """`,` and `.` move the lever position by 10 % and stop at both ends of the range."""
    assert session.brake_strength == pytest.approx(DEFAULT_BRAKE_STRENGTH)

    session.handle_key(ord("."))
    assert session.brake_strength == pytest.approx(DEFAULT_BRAKE_STRENGTH + BRAKE_STRENGTH_STEP)
    for _ in range(20):
        session.handle_key(ord("."))
    assert session.brake_strength == pytest.approx(1.0)
    for _ in range(20):
        session.handle_key(ord(","))
    assert session.brake_strength == pytest.approx(0.0)


def test_damper_keys_reach_the_circuit_their_stand_binding_names(session: RideSession):
    """Each inherited damping key moves the same circuit it moves on the test stand."""
    fork = session.suspension_system.fork_damper
    shock = session.suspension_system.shock_damper

    for keycode, damper, attribute, delta in (
        (ord("J"), fork, "hsc_clicks", +1),
        (ord("H"), fork, "hsc_clicks", -1),
        (ord("L"), fork, "lsc_clicks", +1),
        (ord("K"), fork, "lsc_clicks", -1),
        (ord("U"), fork, "rebound_clicks", +1),
        (ord("Y"), fork, "rebound_clicks", -1),
        (ord("8"), shock, "hsc_clicks", +1),
        (ord("7"), shock, "hsc_clicks", -1),
        (ord("0"), shock, "lsc_clicks", +1),
        (ord("9"), shock, "lsc_clicks", -1),
        (ord("6"), shock, "rebound_clicks", +1),
        (ord("5"), shock, "rebound_clicks", -1),
        (ord("4"), shock, "hbo_clicks", +1),
        (ord("3"), shock, "hbo_clicks", -1),
    ):
        before = getattr(damper, attribute)
        session.handle_key(keycode)
        after = getattr(damper, attribute)
        # A circuit already at its end stop stays there; every other key moves by one click.
        assert after in (before + delta, before), f"key {chr(keycode)} moved {attribute} wrongly"
        assert after != before or before in (0, getattr(damper, f"max_{attribute[:3]}"))


def test_air_spring_keys_adjust_tokens_and_pressure(session: RideSession):
    """`[`/`]` and `-`/`=` keep their stand meaning on the fork air spring."""
    air = session.air_spring
    tokens_before, psi_before = air.num_tokens, air.gauge_pressure_psi

    session.handle_key(ord("]"))
    assert air.num_tokens == tokens_before + 1
    session.handle_key(ord("["))
    assert air.num_tokens == tokens_before

    session.handle_key(ord("="))
    assert air.gauge_pressure_psi == pytest.approx(psi_before + 2.0)
    session.handle_key(ord("-"))
    assert air.gauge_pressure_psi == pytest.approx(psi_before)


def test_camera_keys_select_and_cycle_the_presets(session: RideSession):
    """`1`, `2` and `C` drive the shared camera manager."""
    session.handle_key(ord("2"))
    assert session.camera.active_mode == "3d"
    session.handle_key(ord("1"))
    assert session.camera.active_mode == "2d"
    session.handle_key(ord("C"))
    assert session.camera.active_mode == "3d"


def test_t_toggles_the_telemetry_stream(session: RideSession):
    """`T` flips the HUD stream without touching the physics."""
    before = session.show_telemetry
    session.handle_key(ord("T"))
    assert session.show_telemetry is not before


def test_g_toggles_the_marker_livery(session: RideSession):
    """`G` reveals and hides the pivot markers that the viewer's model carries."""
    marker_ids = session.livery.marker_geom_ids
    assert session.livery.has_markers, "the session's model has no marker geoms to toggle"
    assert session.sim.model.geom_rgba[marker_ids[0], 3] == 0.0

    session.handle_key(ord("G"))
    assert session.debug_markers is True
    assert session.sim.model.geom_rgba[marker_ids[0], 3] == 1.0

    session.handle_key(ord("G"))
    assert session.debug_markers is False
    assert session.sim.model.geom_rgba[marker_ids[0], 3] == 0.0


def test_b_is_unbound_in_ride_mode(session: RideSession):
    """
    `B` does nothing on the track.

    The test stand toggles its lumped rider in place by rewriting the `frame` body's mass. The
    ride model's seated rider is its own bodies and joints, so a rider change is a recompile the
    passive viewer cannot follow; the variant is a `bike-ride --rider` choice and the key is
    deliberately left unbound rather than half-working for one variant.
    """
    frame_id = mujoco.mj_name2id(session.sim.model, mujoco.mjtObj.mjOBJ_BODY, "frame")
    mass_before = float(session.sim.model.body_mass[frame_id])
    steps_before = session.sim.steps

    assert session.input.handle_key(ord("B")) is False
    assert session.input.handle_key(ord("b")) is False
    assert float(session.sim.model.body_mass[frame_id]) == mass_before
    assert session.sim.steps == steps_before
    assert session.sim.rider_variant == "lumped"


# --------------------------------------------------------------------------------------
# The session
# --------------------------------------------------------------------------------------


def test_session_latches_its_outcome_and_stops_stepping(session: RideSession):
    """Once the run has terminated the session holds the state on screen instead of riding on."""
    session.limits = RunLimits(
        finish_x_m=session.sim.track.length_m, max_steps=session.sim.steps + 5, max_wall_clock_s=60.0
    )
    session.terminator = RunTerminator(session.limits)
    session.terminator.start()

    while session.step() is None:
        pass

    outcome, steps = session.outcome, session.sim.steps
    assert outcome.reason == REASON_STEP_CAP
    assert session.step() is outcome
    assert session.sim.steps == steps


def test_session_brake_toggle_applies_brake_torque_to_both_wheels(session: RideSession):
    """
    With the toggle engaged both wheels are given torque against their rotation.

    The demand the session applies is the brake-strength lever, so a 50 % toggle is 100 N.m of
    the 200 N.m per-wheel ceiling, tapered as either wheel slows to a stop.
    """
    for _ in range(600):
        session.step()
    assert session.sim.brakes.front_torque_nm == 0.0
    assert session.sim.brakes.rear_torque_nm == 0.0

    session.handle_key(32)
    session.step()

    assert session.braking is True
    assert session.sim.brakes.front_torque_nm < 0.0
    assert session.sim.brakes.rear_torque_nm < 0.0
    assert abs(session.sim.brakes.front_torque_nm) == pytest.approx(
        DEFAULT_BRAKE_STRENGTH * 200.0, rel=1e-6
    )


def test_session_reset_returns_the_bike_to_the_start_with_the_brakes_off(session: RideSession):
    """A restart re-solves the equilibrium, clears the outcome, and releases the brakes."""
    session.handle_key(32)
    for _ in range(200):
        session.step()
    assert session.sim.steps > 0

    session.handle_key(ord("R"))

    assert session.sim.steps == 0
    assert session.outcome is None
    assert session.braking is False
    assert session.sim.position_m == pytest.approx(session.sim.start_x_m, abs=0.05)
