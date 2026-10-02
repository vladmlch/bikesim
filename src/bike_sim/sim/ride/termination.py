"""
Ride-Mode Run Termination.

A run ends for exactly one of four reasons, and the reason is carried in the result rather
than inferred from it: the bike passed the end of the track, the crash detector tripped, or
one of two safety caps fired. A caller that only sees "the loop finished" cannot tell a
completed traverse from a bike that stopped moving 30 m in, so nothing here returns a bare
boolean.

The two caps are not redundant. The **step cap** bounds the simulated distance: it is derived
from the track length at the slowest speed section 6 admits, so a bike that stalls, reverses,
or is held on the brakes fails instead of running forever. The **wall-clock cap** bounds real
time, which the step cap cannot: a run that hits a solver slowdown, or an interactive session
left open, is not producing usable telemetry either.

Nothing in this module reads or writes MuJoCo state. It is given a position, a step count and
a latched crash, and it answers whether the run is over -- so a terminator can be driven from
a headless loop, from the interactive viewer, or from a test with a fake clock.
"""

import time
from dataclasses import dataclass
from typing import Callable, Optional

from bike_sim.sim.ride.cruise import KMH_PER_MPS, MIN_TARGET_SPEED_KMH
from bike_sim.sim.ride.virtual_rider import CrashEvent
from bike_sim.terrain import TrackSpec

REASON_END_OF_TRACK = "end of track"
REASON_CRASH = "crash"
REASON_STEP_CAP = "step cap"
REASON_WALL_CLOCK_CAP = "wall clock cap"

# Multiple of the steps a traverse needs at the slowest speed of the section 6 band. Two is
# enough to absorb the standing start (the bike leaves the solved equilibrium at rest and
# needs ~10 m to reach target) and any number of stumbles, and still fails a bike that has
# stopped rather than letting it hang.
STEP_CAP_SAFETY_FACTOR = 2.0

# Real time a single run may take. The measured 115 m `enduro_aggressive` traverse runs
# 17.4 s of simulated time in 3.5 s of wall clock, so 300 s is nearly two orders of magnitude
# of headroom: it fires for a wedged process, never for a slow one.
DEFAULT_MAX_WALL_CLOCK_S = 300.0


@dataclass(frozen=True)
class RunLimits:
    """
    The three bounds a run is judged against.

    Attributes:
        finish_x_m: Track position at which the run has been completed, in metres.
        max_steps: Simulation steps after which the run is abandoned.
        max_wall_clock_s: Real seconds after which the run is abandoned.
    """

    finish_x_m: float
    max_steps: int
    max_wall_clock_s: float = DEFAULT_MAX_WALL_CLOCK_S

    def __post_init__(self) -> None:
        """
        Raises:
            ValueError: If any bound is not positive, which would terminate every run
                immediately and report a cap as though the bike had failed.
        """
        if self.finish_x_m <= 0.0:
            raise ValueError(f"finish_x_m must be positive, got {self.finish_x_m}")
        if self.max_steps <= 0:
            raise ValueError(f"max_steps must be positive, got {self.max_steps}")
        if self.max_wall_clock_s <= 0.0:
            raise ValueError(f"max_wall_clock_s must be positive, got {self.max_wall_clock_s}")

    @classmethod
    def for_track(
        cls,
        track: TrackSpec,
        timestep_s: float,
        start_x_m: float,
        max_wall_clock_s: float = DEFAULT_MAX_WALL_CLOCK_S,
        min_speed_mps: Optional[float] = None,
    ) -> "RunLimits":
        """
        Derives the limits of a traverse of one track.

        The step cap is the distance still to cover, ridden at the slowest speed the section 6
        band allows, times `STEP_CAP_SAFETY_FACTOR`. Deriving it rather than fixing it keeps a
        short debugging track from inheriting the 115 m preset's budget.

        Args:
            track: Track to be ridden.
            timestep_s: Integration timestep of the compiled model, in seconds.
            start_x_m: Track position the run starts from, in metres.
            max_wall_clock_s: Real seconds the run may take.
            min_speed_mps: Slowest sustained speed the cap is sized for. Defaults to the
                minimum of the section 6 cruise band; callers with a steep grade should pass
                the power-limited climb speed instead.

        Returns:
            Limits for a traverse from `start_x_m` to the end of `track`.

        Raises:
            ValueError: If the timestep is not positive, or the run starts at or past the end
                of the track.
        """
        if timestep_s <= 0.0:
            raise ValueError(f"timestep_s must be positive, got {timestep_s}")
        distance_m = float(track.length_m) - float(start_x_m)
        if distance_m <= 0.0:
            raise ValueError(
                f"run starts at x = {start_x_m:.3f} m, at or past the end of track "
                f"'{track.name}' at {track.length_m:.3f} m"
            )

        if min_speed_mps is None:
            slowest_mps = MIN_TARGET_SPEED_KMH / KMH_PER_MPS
        else:
            slowest_mps = float(min_speed_mps)
            if slowest_mps <= 0.0:
                raise ValueError(f"min_speed_mps must be positive, got {min_speed_mps}")
        steps = STEP_CAP_SAFETY_FACTOR * distance_m / (slowest_mps * timestep_s)
        return cls(
            finish_x_m=float(track.length_m),
            max_steps=int(steps),
            max_wall_clock_s=float(max_wall_clock_s),
        )


@dataclass(frozen=True)
class RunOutcome:
    """
    How a run ended, and the state it ended in.

    Attributes:
        reason: One of the four `REASON_*` constants.
        steps: Simulation steps taken.
        sim_time_s: Simulated time at termination, in seconds.
        wall_clock_s: Real time the run took, in seconds.
        position_m: Track position at termination, in metres.
        crash: The latched crash event, or None if the run never crashed. Present whenever
            `reason` is `REASON_CRASH`, and None otherwise.
    """

    reason: str
    steps: int
    sim_time_s: float
    wall_clock_s: float
    position_m: float
    crash: Optional[CrashEvent] = None

    @property
    def completed(self) -> bool:
        """Whether the bike rode the whole track."""
        return self.reason == REASON_END_OF_TRACK

    def describe(self) -> str:
        """Returns a one-line human-readable summary of the run."""
        line = (
            f"{self.reason}: x = {self.position_m:.2f} m after {self.steps} steps, "
            f"t = {self.sim_time_s:.3f} s sim / {self.wall_clock_s:.2f} s wall"
        )
        if self.crash is not None:
            line = f"{line} -- {self.crash.describe()}"
        return line


class RunTerminator:
    """
    Judges, once per step, whether a run is over and why.

    The clock is injected so that the wall-clock cap can be exercised without waiting for it,
    and so that a caller measuring a run's cost uses the same clock the cap does.
    """

    def __init__(
        self,
        limits: RunLimits,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        """
        Args:
            limits: Bounds the run is judged against.
            clock: Monotonic wall-clock source returning seconds.
        """
        self.limits = limits
        self._clock = clock
        self._started_at = clock()

    def start(self) -> None:
        """Marks the current instant as the start of the run, for the wall-clock cap."""
        self._started_at = self._clock()

    @property
    def wall_clock_s(self) -> float:
        """Real seconds since `start`."""
        return float(self._clock() - self._started_at)

    def reason(
        self,
        position_m: float,
        steps: int,
        crash: Optional[CrashEvent] = None,
    ) -> Optional[str]:
        """
        Tests every termination condition against the current state.

        A crash is tested first: a bike that goes down on the finish line has crashed, and
        reporting that run as completed would hide the failure the detector exists to catch.

        Args:
            position_m: Current track position, in metres.
            steps: Simulation steps taken so far.
            crash: The crash detector's latched event, or None.

        Returns:
            One of the `REASON_*` constants, or None while the run should continue.
        """
        if crash is not None:
            return REASON_CRASH
        if position_m >= self.limits.finish_x_m:
            return REASON_END_OF_TRACK
        if steps >= self.limits.max_steps:
            return REASON_STEP_CAP
        if self.wall_clock_s >= self.limits.max_wall_clock_s:
            return REASON_WALL_CLOCK_CAP
        return None

    def outcome(
        self,
        reason: str,
        steps: int,
        sim_time_s: float,
        position_m: float,
        crash: Optional[CrashEvent] = None,
    ) -> RunOutcome:
        """
        Packages a termination reason with the state the run ended in.

        Args:
            reason: The reason `reason()` returned.
            steps: Simulation steps taken.
            sim_time_s: Simulated time at termination, in seconds.
            position_m: Track position at termination, in metres.
            crash: The latched crash event, or None.

        Returns:
            The completed run's outcome.
        """
        return RunOutcome(
            reason=reason,
            steps=int(steps),
            sim_time_s=float(sim_time_s),
            wall_clock_s=self.wall_clock_s,
            position_m=float(position_m),
            crash=crash,
        )


__all__ = [
    "RunLimits",
    "RunOutcome",
    "RunTerminator",
    "REASON_END_OF_TRACK",
    "REASON_CRASH",
    "REASON_STEP_CAP",
    "REASON_WALL_CLOCK_CAP",
    "STEP_CAP_SAFETY_FACTOR",
    "DEFAULT_MAX_WALL_CLOCK_S",
]
