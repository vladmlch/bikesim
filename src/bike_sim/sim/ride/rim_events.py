"""Observation-only grouping of raw rim crossings; never a contact-force filter."""
from collections.abc import Mapping, Sequence
from math import isfinite

RIM_EPISODE_GAP_S = 0.001
RIM_EPISODE_DISTANCE_M = 0.01


def count_completed_impact_episodes(
    events: Sequence[Mapping[str, float]], *,
    gap_s: float = RIM_EPISODE_GAP_S,
    distance_m: float = RIM_EPISODE_DISTANCE_M,
) -> int:
    """Group completed crossings at one road feature separated by <=1 ms.

    Coarse ray sampling can momentarily unload the entire rim between adjacent
    rays. Raw crossings and forces remain untouched. This separate, explicitly
    thresholded metric counts an impact episode; it is not calibrated damage.
    Legacy event records without timestamps are conservatively kept separate.
    """
    if not all(isfinite(v) and v >= 0 for v in (gap_s, distance_m)):
        raise ValueError('invalid rim episode thresholds')
    count = 0
    previous_end = previous_x = None
    for event in events:
        x = float(event['x_m'])
        start, end = event.get('start_time_s'), event.get('end_time_s')
        if not isfinite(x):
            raise ValueError('non-finite rim position')
        if start is None or end is None:
            count += 1
            previous_end = previous_x = None
            continue
        start, end = float(start), float(end)
        if not all(isfinite(v) for v in (start, end)) or start < 0 or end < start:
            raise ValueError('invalid rim event interval')
        if previous_end is not None and start < previous_end - 1e-12:
            raise ValueError('rim event intervals overlap or are unordered')
        connected = (previous_end is not None and start - previous_end <= gap_s + 1e-12
                     and abs(x - previous_x) <= distance_m)
        if not connected:
            count += 1
        previous_end, previous_x = end, x
    return count
