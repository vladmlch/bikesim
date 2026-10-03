"""Per-physics-step rejection of invalid reference intervals.

Strict research raises on the first violated budget so the bad interval can
never be hidden by recording decimation or reported as a legal outcome. The
non-strict (viewer/diagnostic) mode warns once, keeps the first failure and
lets recording continue — it never restores a valid status.
"""
from dataclasses import dataclass
import warnings


class InvalidReferenceRun(RuntimeError):
    pass


@dataclass
class ReferenceMonitor:
    strict: bool
    first_failure: tuple[float, tuple[str, ...]] | None = None

    def accept(self, time_s: float, violations: tuple[str, ...]) -> None:
        if not violations:
            return
        first = self.first_failure is None
        if first:
            self.first_failure = (time_s, violations)
        message = f'invalid reference step at {time_s:.9f}: {violations}'
        if self.strict:
            raise InvalidReferenceRun(message)
        if first:
            warnings.warn(message, RuntimeWarning, stacklevel=2)
