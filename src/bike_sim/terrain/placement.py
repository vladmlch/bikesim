"""
Seeded Interval Placement.

Helpers shared by the procedural track generators (:mod:`bike_sim.terrain.generator`,
:mod:`bike_sim.terrain.road`, :mod:`bike_sim.terrain.alpine`): a rounding rule that keeps a
dumped TOML file round-trip exact, range validation, and clearance-aware random placement of
an interval inside a span. Callers pass their own ``numpy`` Generator; nothing here touches
global RNG state.
"""
from typing import List, Sequence, Tuple

from bike_sim.physics.checks import scalar

Interval = Tuple[float, float]

CLEARANCE_M = 1.
"""Minimum clear distance between placed features and sections, in metres."""
PLACEMENT_TRIES = 200


def rounded(value, digits=4):
    """Round so saved TOML files reproduce the track bit-for-bit."""
    return round(float(value), digits)


def check_range(value, name, *, minimum=0., integer=False):
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise ValueError(f'{name} must be a (low, high) pair')
    lo, hi = value
    if integer:
        if any(type(v) is not int for v in (lo, hi)):
            raise ValueError(f'{name} must hold integers')
    else:
        lo, hi = scalar(lo, name, minimum=minimum), scalar(hi, name, minimum=minimum)
    if lo > hi or lo < minimum:
        raise ValueError(f'{name} must satisfy {minimum} <= low <= high')
    return (lo, hi)


def is_free(start, end, taken):
    return all(end+CLEARANCE_M <= a or start >= b+CLEARANCE_M for a, b in taken)


def place(rng, length, lo, hi, taken, *, shrink=False):
    """Random start for an interval of `length` inside [lo, hi] clear of `taken`.

    With shrink, the interval is halved (down to 1 m) when it cannot be placed.
    Returns (start, length) or None.
    """
    while True:
        if length <= hi-lo:
            for _ in range(PLACEMENT_TRIES):
                start = rounded(rng.uniform(lo, hi-length), 2)
                if is_free(start, start+length, taken):
                    return start, length
        if not shrink or length/2. < 1.:
            return None
        length = rounded(length/2., 2)


def free_intervals(span: Interval, occupied: Sequence[Interval], gap_m: float) -> List[Interval]:
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


__all__ = ['CLEARANCE_M', 'PLACEMENT_TRIES', 'Interval', 'rounded', 'check_range', 'is_free', 'place',
           'free_intervals']
