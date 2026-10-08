"""Rider-control period with a held muscle command between physics ticks."""
from math import isfinite


class ControlClock:
    def __init__(self, timestep_s: float, period_s: float):
        if not (isfinite(timestep_s) and isfinite(period_s)) or min(timestep_s, period_s) <= 0:
            raise ValueError('control clock requires positive finite durations')
        steps = round(period_s/timestep_s)
        if steps < 1 or abs(steps*timestep_s-period_s) > 1e-9:
            raise ValueError('control period must be an integer multiple of the timestep')
        self.timestep_s = float(timestep_s)
        self.period_s = float(period_s)
        self.steps_per_period = steps
        self._held = None

    def is_tick(self, step: int) -> bool:
        if type(step) is not int or step < 0:
            raise ValueError('control clock requires a nonnegative physics step')
        return step % self.steps_per_period == 0

    def hold(self, torques: dict[str, float]) -> None:
        self._held = {str(k): float(v) for k, v in torques.items()}

    def held(self) -> dict[str, float]:
        if self._held is None:
            raise RuntimeError('control clock holds no command yet')
        return dict(self._held)

    def reset(self) -> None:
        self._held = None

    def held_state(self) -> dict[str, float] | None:
        """The currently held command copy, or None before the first tick."""
        return None if self._held is None else dict(self._held)
