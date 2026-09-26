"""
Procedural Rough-Road Generator.

Turns a statistical description of a worn road -- how many potholes and bumps per
hundred metres, how large, what shape, how rough the asphalt between them -- into an
ordinary list of catalogue obstacles. The result is a plain :class:`TrackSpec` layout,
so validation, markers, plots and the HUD need to know nothing about how it was made.

Everything is drawn from one ``numpy`` generator seeded by ``seed``; the same spec and
seed always produce the same road. Counts are deterministic (rate times usable length,
rounded), not Poisson, so two seeds of the same level are comparable in severity and
differ only in layout and individual sizes.

The generator works in metres, like the rest of the package. The track-file loader is
where millimetres are accepted.
"""

from dataclasses import dataclass, field
import math
from typing import Callable, Dict, List, Sequence, Tuple
import numpy as np

from bike_sim.terrain.obstacles import (
    BowlPothole,
    Bump,
    Obstacle,
    Pothole,
    RoadRoughness,
    SlopedPothole,
    TrapezoidBump,
)
from bike_sim.terrain.profile import TrackSpec

Range = Tuple[float, float]
Interval = Tuple[float, float]

POTHOLE_EDGES = ("sharp", "sloped", "bowl")
BUMP_SHAPES = ("cosine", "trapezoid")

DEFAULT_ROAD_LENGTH_M = 100.0
MIN_ROUGHNESS_SEGMENT_CORRELATIONS = 4.0
"""Free stretches shorter than this many correlation lengths get no roughness segment."""


@dataclass
class RoadGeneratorSpec:
    """
    Statistical description of a rough road, in metres.

    Ranges are ``(low, high)`` tuples drawn uniformly per obstacle; a fixed size is a
    range with equal ends. Shape weights are relative and need not sum to one.
    """

    seed: int = 0
    runup_m: float = 10.0
    runout_m: float = 5.0
    min_gap_m: float = 1.0

    potholes_per_100m: float = 4.0
    pothole_depth_m: Range = (0.040, 0.120)
    pothole_length_m: Range = (0.300, 0.800)
    pothole_edge: Dict[str, float] = field(default_factory=lambda: {"sharp": 1.0})
    pothole_edge_m: float = 0.100

    bumps_per_100m: float = 6.0
    bump_height_m: Range = (0.030, 0.080)
    bump_length_m: Range = (0.300, 0.600)
    bump_shape: Dict[str, float] = field(default_factory=lambda: {"cosine": 1.0})
    bump_ramp_fraction: float = 0.3

    roughness_m: float = 0.003
    roughness_correlation_m: float = 0.200

    def validate(self) -> None:
        """
        Checks the spec for values the generator cannot honour.

        Raises:
            ValueError: On negative rates, inverted ranges, unknown shape names or
                weights that are all zero.
        """
        if self.potholes_per_100m < 0.0 or self.bumps_per_100m < 0.0:
            raise ValueError("defect rates must be non-negative")
        for name in ("pothole_depth_m", "pothole_length_m", "bump_height_m", "bump_length_m"):
            lo, hi = getattr(self, name)
            if lo < 0.0 or hi < lo:
                raise ValueError(f"{name} range ({lo}, {hi}) is not 0 <= low <= high")
        _check_weights("pothole_edge", self.pothole_edge, POTHOLE_EDGES)
        _check_weights("bump_shape", self.bump_shape, BUMP_SHAPES)
        if self.roughness_m < 0.0:
            raise ValueError("roughness_m must be non-negative")
        if not 0.0 < self.bump_ramp_fraction <= 0.5:
            raise ValueError("bump_ramp_fraction must be in (0, 0.5]")


def _check_weights(name: str, weights: Dict[str, float], allowed: Sequence[str]) -> None:
    unknown = set(weights) - set(allowed)
    if unknown:
        raise ValueError(f"{name}: unknown shape(s) {sorted(unknown)}; allowed: {list(allowed)}")
    if not weights or sum(weights.values()) <= 0.0:
        raise ValueError(f"{name}: weights must include at least one positive entry")
    if any(w < 0.0 for w in weights.values()):
        raise ValueError(f"{name}: weights must be non-negative")


SIZE_ROUNDING_M = 0.001
"""Generated heights, depths and lengths are rounded to the millimetre."""

POSITION_ROUNDING_M = 0.01
"""Generated start positions are rounded to the centimetre."""


def _draw(rng: np.random.Generator, rng_range: Range) -> float:
    """Draws a size from a range, rounded so a dumped track file reads like one a person wrote."""
    lo, hi = rng_range
    value = float(lo) if hi <= lo else float(rng.uniform(lo, hi))
    return round(value / SIZE_ROUNDING_M) * SIZE_ROUNDING_M


def _choose(rng: np.random.Generator, weights: Dict[str, float]) -> str:
    names = sorted(weights)
    p = np.array([weights[n] for n in names], dtype=float)
    return names[int(rng.choice(len(names), p=p / p.sum()))]


def _free_intervals(span: Interval, occupied: Sequence[Interval], gap_m: float) -> List[Interval]:
    """Sub-intervals of ``span`` at least ``gap_m`` clear of every occupied interval."""
    lo, hi = span
    free: List[Interval] = []
    cursor = lo
    for start, end in sorted(occupied):
        right = min(start - gap_m, hi)
        if right > cursor:
            free.append((cursor, right))
        cursor = max(cursor, end + gap_m)
    if hi > cursor:
        free.append((cursor, hi))
    return free


def _place(
    rng: np.random.Generator,
    length_m: float,
    span: Interval,
    occupied: Sequence[Interval],
    gap_m: float,
    label: str,
) -> float:
    """Draws a start position for an obstacle of ``length_m``; the caller records it."""
    slots = [(a, b - length_m) for a, b in _free_intervals(span, occupied, gap_m) if b - a >= length_m]
    if not slots:
        raise ValueError(
            f"no room left for a {length_m:.2f} m {label}: lower the defect rates, "
            f"shorten the sizes or lengthen the track"
        )
    # Weight slots by how much choice they offer, but never zero: a slot the obstacle
    # fits exactly is still a slot.
    widths = np.array([b - a for a, b in slots]) + POSITION_ROUNDING_M
    a, b = slots[int(rng.choice(len(slots), p=widths / widths.sum()))]
    start = float(rng.uniform(a, b)) if b > a else float(a)
    # Round towards the inside of the slot so rounding can never breach the gap.
    return min(max(round(start / POSITION_ROUNDING_M) * POSITION_ROUNDING_M, a), b)


def _make_pothole(spec: RoadGeneratorSpec, rng: np.random.Generator) -> Tuple[str, float, float]:
    edge = _choose(rng, spec.pothole_edge)
    depth = _draw(rng, spec.pothole_depth_m)
    length = _draw(rng, spec.pothole_length_m)
    return edge, depth, length


def _make_bump(spec: RoadGeneratorSpec, rng: np.random.Generator) -> Tuple[str, float, float]:
    shape = _choose(rng, spec.bump_shape)
    height = _draw(rng, spec.bump_height_m)
    length = _draw(rng, spec.bump_length_m)
    return shape, height, length


def generate_road(
    spec: RoadGeneratorSpec,
    length_m: float,
    reserved: Sequence[Interval] = (),
) -> List[Obstacle]:
    """
    Generates the defects and roughness of a rough road.

    Args:
        spec: Statistical description of the road.
        length_m: Track length in metres.
        reserved: ``(start, end)`` intervals already occupied by hand-placed obstacles;
            generated defects keep ``spec.min_gap_m`` clear of them and roughness fills
            around them.

    Returns:
        Generated obstacles only (not the reserved ones), unsorted.

    Raises:
        ValueError: If the spec is invalid or the requested density does not fit.
    """
    spec.validate()
    span: Interval = (spec.runup_m, length_m - spec.runout_m)
    usable = span[1] - span[0]
    if usable <= 0.0:
        raise ValueError(
            f"track of {length_m} m leaves no room between runup {spec.runup_m} m and "
            f"runout {spec.runout_m} m"
        )

    rng = np.random.default_rng(spec.seed)
    occupied: List[Interval] = list(reserved)
    obstacles: List[Obstacle] = []

    n_potholes = _count(spec.potholes_per_100m, usable)
    n_bumps = _count(spec.bumps_per_100m, usable)

    # Sizes and shapes are drawn first, in a fixed order, so that changing the count of one
    # kind does not reshuffle the sizes of the other.
    potholes = [_make_pothole(spec, rng) for _ in range(n_potholes)]
    bumps = [_make_bump(spec, rng) for _ in range(n_bumps)]

    # Largest first: small defects fit into gaps that large ones cannot.
    order = sorted(
        [("pothole", i, potholes[i][2]) for i in range(n_potholes)]
        + [("bump", i, bumps[i][2]) for i in range(n_bumps)],
        key=lambda item: -item[2],
    )
    for kind, index, length in order:
        start = _place(rng, length, span, occupied, spec.min_gap_m, kind)
        if kind == "pothole":
            edge, depth, _ = potholes[index]
            obstacle = _build_pothole(edge, start, depth, length, spec.pothole_edge_m)
        else:
            shape, height, _ = bumps[index]
            obstacle = _build_bump(shape, start, height, length, spec.bump_ramp_fraction)
        # Reserve the obstacle's *own* extent: a trapezoid rebuilt from ramp and plateau
        # can differ from the drawn length by an ulp, and roughness is laid down flush.
        occupied.append((obstacle.start_m, obstacle.end_m))
        obstacles.append(obstacle)

    if spec.roughness_m > 0.0:
        min_len = MIN_ROUGHNESS_SEGMENT_CORRELATIONS * spec.roughness_correlation_m
        for a, b in _free_intervals(span, occupied, 0.0):
            if b - a >= min_len:
                # Trim float noise from the segment length without ever crossing into the
                # neighbouring defect (validate() rejects even a one-ulp overlap).
                seg_len = round(b - a, 6)
                while a + seg_len > b:
                    seg_len = round(seg_len - 1e-6, 6)
                obstacles.append(
                    RoadRoughness(
                        start_m=a,
                        section_length_m=seg_len,
                        amplitude_m=spec.roughness_m,
                        correlation_length_m=spec.roughness_correlation_m,
                        seed=int(rng.integers(0, 2**31 - 1)),
                    )
                )

    return obstacles


def _count(rate_per_100m: float, usable_m: float) -> int:
    """Expected defect count over the usable length, rounded half up (not banker's)."""
    return int(math.floor(rate_per_100m * usable_m / 100.0 + 0.5))


def _build_pothole(edge: str, start: float, depth: float, length: float, edge_m: float) -> Obstacle:
    if edge == "sharp":
        return Pothole(start_m=start, depth_m=depth, hole_length_m=length)
    if edge == "sloped":
        return SlopedPothole(start_m=start, depth_m=depth, hole_length_m=length, edge_m=edge_m)
    return BowlPothole(start_m=start, depth_m=depth, hole_length_m=length)


def _build_bump(shape: str, start: float, height: float, length: float, ramp_fraction: float) -> Obstacle:
    if shape == "cosine":
        return Bump(start_m=start, height_m=height, bump_length_m=length)
    ramp = length * ramp_fraction
    return TrapezoidBump(start_m=start, height_m=height, ramp_m=ramp, plateau_m=length - 2.0 * ramp)


def build_road(
    name: str,
    spec: RoadGeneratorSpec,
    length_m: float = DEFAULT_ROAD_LENGTH_M,
    hand_placed: Sequence[Obstacle] = (),
    description: str = "",
) -> TrackSpec:
    """
    Builds a validated track from hand-placed obstacles plus generated fill.

    Args:
        name: Track name.
        spec: Generator description.
        length_m: Track length in metres.
        hand_placed: Obstacles authored explicitly; kept verbatim.
        description: Free-text description for the track.

    Returns:
        A validated TrackSpec.
    """
    reserved = [(o.start_m, o.end_m) for o in hand_placed]
    generated = generate_road(spec, length_m, reserved)
    track = TrackSpec(
        name=name,
        length_m=length_m,
        description=description,
        obstacles=list(hand_placed) + generated,
    )
    track.validate()
    return track


# --------------------------------------------------------------------------------------
# Built-in road levels
# --------------------------------------------------------------------------------------


def road_smooth_spec() -> RoadGeneratorSpec:
    """Well-kept asphalt with the occasional shallow defect."""
    return RoadGeneratorSpec(
        seed=0,
        potholes_per_100m=1.0,
        pothole_depth_m=(0.030, 0.060),
        pothole_length_m=(0.300, 0.500),
        bumps_per_100m=2.0,
        bump_height_m=(0.020, 0.040),
        bump_length_m=(0.300, 0.500),
        roughness_m=0.002,
    )


def road_worn_spec() -> RoadGeneratorSpec:
    """Worn urban asphalt: the default rough road."""
    return RoadGeneratorSpec(seed=0)


def road_broken_spec() -> RoadGeneratorSpec:
    """Broken-up road with mixed defect shapes and coarse texture."""
    return RoadGeneratorSpec(
        seed=0,
        potholes_per_100m=8.0,
        pothole_depth_m=(0.060, 0.150),
        pothole_length_m=(0.400, 1.000),
        pothole_edge={"sharp": 0.6, "sloped": 0.3, "bowl": 0.1},
        bumps_per_100m=10.0,
        bump_height_m=(0.050, 0.100),
        bump_length_m=(0.300, 0.700),
        bump_shape={"cosine": 0.7, "trapezoid": 0.3},
        roughness_m=0.005,
        roughness_correlation_m=0.150,
    )


def road_smooth() -> TrackSpec:
    """Builds the ``road_smooth`` preset."""
    return build_road("road_smooth", road_smooth_spec(), description="Well-kept asphalt.")


def road_worn() -> TrackSpec:
    """Builds the ``road_worn`` preset."""
    return build_road("road_worn", road_worn_spec(), description="Worn urban asphalt.")


def road_broken() -> TrackSpec:
    """Builds the ``road_broken`` preset."""
    return build_road("road_broken", road_broken_spec(), description="Broken-up road.")


ROAD_LEVEL_SPECS: Dict[str, Callable[[], RoadGeneratorSpec]] = {
    "road_smooth": road_smooth_spec,
    "road_worn": road_worn_spec,
    "road_broken": road_broken_spec,
}


__all__ = [
    "Range",
    "Interval",
    "POTHOLE_EDGES",
    "BUMP_SHAPES",
    "DEFAULT_ROAD_LENGTH_M",
    "RoadGeneratorSpec",
    "generate_road",
    "build_road",
    "road_smooth_spec",
    "road_worn_spec",
    "road_broken_spec",
    "road_smooth",
    "road_worn",
    "road_broken",
    "ROAD_LEVEL_SPECS",
]
