"""A latched low-speed event for the planar rider; never stops integration."""
from dataclasses import dataclass

from bike_sim.physics.checks import scalar


@dataclass(frozen=True)
class BalanceLostEvent:
    time_s: float
    position_m: float
    speed_mps: float


class BalanceMonitor:
    def __init__(self, floor_mps: float, dwell_s: float, grace_s: float):
        self.floor_mps = scalar(floor_mps,'balance floor',positive=True)
        self.dwell_s = scalar(dwell_s,'balance dwell',positive=True)
        self.grace_s = scalar(grace_s,'balance grace',minimum=0.)
        self.reset()

    def reset(self):
        self.event = None
        self.low_speed_s = 0.
        self.grace_left_s = self.grace_s
        self._started_s = None
        self._low_started_s = None
        self._last_time_s = None

    def update(self, time_s: float, position_m: float, speed_mps: float,
               *, riding: bool) -> BalanceLostEvent | None:
        time = scalar(time_s,'balance time',minimum=0.)
        position = scalar(position_m,'balance position')
        speed = scalar(speed_mps,'balance speed')
        if not isinstance(riding,bool):
            raise ValueError('balance riding state must be a bool')
        if self._last_time_s is not None and time < self._last_time_s:
            raise ValueError('balance monitor time must not rewind')
        self._last_time_s = time
        if self.event is not None:
            return self.event
        if riding and self._started_s is None:
            self._started_s = time
        self.grace_left_s = (self.grace_s if self._started_s is None else
            max(0.,self._started_s+self.grace_s-time))
        if not riding or self.grace_left_s > 1e-12 or speed >= self.floor_mps:
            self._low_started_s = None
            self.low_speed_s = 0.
            return None
        if self._low_started_s is None:
            self._low_started_s = time
        self.low_speed_s = time-self._low_started_s
        if self.low_speed_s >= self.dwell_s-1e-12:
            self.event = BalanceLostEvent(time,position,speed)
        return self.event

    def observation(self) -> dict:
        return {'balance_lost':self.event is not None,
                'low_speed_s':self.low_speed_s,'grace_left_s':self.grace_left_s,
                'balance_lost_at_m':None if self.event is None else self.event.position_m,
                'balance_lost_time_s':None if self.event is None else self.event.time_s}
