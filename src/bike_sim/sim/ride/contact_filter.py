"""Time-based contact debounce for controller gating, independent of wheel load."""

from math import isfinite


class GroundedFilter:
    """Hold a controller's grounded signal briefly after raw road contact ends."""

    def __init__(self, hold_s: float) -> None:
        if not isfinite(hold_s) or hold_s < 0.0:
            raise ValueError("hold_s must be finite and nonnegative")
        self.hold_s = float(hold_s)
        self.reset()

    def reset(self) -> None:
        """Clear all history before a new simulation run."""
        self.last_time: float | None = None
        self.last_loaded: float | None = None
        self.value = False

    def state_dict(self) -> dict:
        """Owned snapshot of the debounce state for the runtime bootstrap."""
        return {'last_time_s': self.last_time,
                'last_loaded_s': self.last_loaded,
                'value': bool(self.value)}

    def update(self, raw_grounded: bool, time_s: float) -> bool:
        """Return the controller gate at this timestamp without creating a load."""
        if not isfinite(time_s):
            raise ValueError("non-finite timestamp")
        if self.last_time is not None and time_s < self.last_time:
            raise ValueError("timestamp moved backwards; reset required")
        if time_s == self.last_time:
            return self.value
        self.last_time = time_s
        if raw_grounded:
            self.last_loaded = time_s
        self.value = bool(
            raw_grounded
            or (self.last_loaded is not None and time_s - self.last_loaded < self.hold_s)
        )
        return self.value
