"""Hook for a reactive rider; an interface only, no behavior lives here.

A RiderBehavior sees what a body could plausibly feel (angular rate,
proper acceleration, saddle/bar/pedal loads) and may answer with a posture
goal. It never sees terrain, true pitch or wheel loads. No reflex or balance
model ships with this package (see the anti-goals in the refocus plan).
"""
from dataclasses import dataclass
from typing import Protocol, runtime_checkable
from bike_sim.physics.rider_posture import RiderPosture


@dataclass(frozen=True)
class RiderSignals:
    pitch_rate_up_rad_s: float = 0.
    specific_force_body_mps2: tuple[float, float, float] = (0., 0., 0.)
    saddle_load_n: float = 0.
    bar_load_n: float = 0.
    pedal_load_n: float = 0.


@runtime_checkable
class RiderBehavior(Protocol):
    def reset(self, seed: int) -> None: ...

    def act(self, time_s: float, signals: RiderSignals) -> RiderPosture | None: ...


def signals_from_sample(sample):
    """Sensory-like signals from the last solved interval; zeros before any exists."""
    if sample is None:
        return RiderSignals()
    sensors, rider = sample.channels['sensors'], sample.channels['rider']
    grip = rider.get('grip', {}).get('force_on_bike_n', (0., 0., 0.))
    return RiderSignals(
        pitch_rate_up_rad_s=-float(sensors['frame_gyro_body_rad_s'][1]),
        specific_force_body_mps2=tuple(float(v) for v in sensors['frame_specific_force_body_mps2']),
        saddle_load_n=float(rider.get('saddle', {}).get('normal_load_n', 0.)),
        bar_load_n=float(sum(v*v for v in grip)**.5),
        pedal_load_n=float(sum(rider.get(n, {}).get('normal_load_n', 0.) for n in ('front_pedal', 'rear_pedal'))))
