"""A low-speed monitor for the planar rider; never stops integration.

``event`` is the run's latched first-loss record for summaries and research
metrics; ``current_event`` is the active loss shown on the HUD and clears
whenever the dwell timer resets (speed back at the floor, braking, grace,
or the rider not riding).
"""
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
        self.current_event = None
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
        if riding and self._started_s is None:
            self._started_s = time
        self.grace_left_s = (self.grace_s if self._started_s is None else
            max(0.,self._started_s+self.grace_s-time))
        if not riding or self.grace_left_s > 1e-12 or speed >= self.floor_mps:
            self._low_started_s = None
            self.low_speed_s = 0.
            self.current_event = None
            return None
        if self._low_started_s is None:
            self._low_started_s = time
        self.low_speed_s = time-self._low_started_s
        if self.low_speed_s >= self.dwell_s-1e-12 and self.current_event is None:
            self.current_event = BalanceLostEvent(time,position,speed)
            if self.event is None:
                self.event = self.current_event
        return self.current_event

    def observation(self) -> dict:
        event = self.current_event
        return {'balance_lost':event is not None,
                'low_speed_s':self.low_speed_s,'grace_left_s':self.grace_left_s,
                'balance_lost_at_m':None if event is None else event.position_m,
                'balance_lost_time_s':None if event is None else event.time_s}

    def state_dict(self) -> dict:
        """Owned snapshot of every latched/running field for the bootstrap."""
        event, current = self.event, self.current_event
        return {'event': None if event is None else {
                    'time_s': event.time_s, 'position_m': event.position_m,
                    'speed_mps': event.speed_mps},
                'current_event': None if current is None else {
                    'time_s': current.time_s, 'position_m': current.position_m,
                    'speed_mps': current.speed_mps},
                'low_speed_s': self.low_speed_s,
                'grace_left_s': self.grace_left_s,
                'started_s': self._started_s,
                'low_started_s': self._low_started_s,
                'last_time_s': self._last_time_s}
