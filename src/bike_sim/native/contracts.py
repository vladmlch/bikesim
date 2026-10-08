"""Owning, read-only value contracts crossing the native runtime boundary.

Design section 3.2: ``FrameSnapshot``, ``AdvanceResult``, ``SampleBatch``
and ``PhysicalViewState`` carry boxed copies — every array owns its storage
with the writeable flag cleared and every mapping is immutable. Nothing here
aliases a reusable native scratch buffer.
"""
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Mapping

import numpy as np


def _owned_array(values, name: str) -> np.ndarray:
    array = np.array(values, dtype=np.float64, copy=True)
    array.flags.writeable = False
    if not array.flags.owndata:
        raise ValueError(f'{name}: array must own its storage')
    return array


@dataclass(frozen=True)
class PhysicalViewState:
    """Read-only channels the presentation layer consumes (filled by A4/C1)."""

    channels: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, 'channels',
                           MappingProxyType(dict(self.channels)))


@dataclass(frozen=True)
class FrameSnapshot:
    """One committed frame boundary owned by the native runtime."""

    generation: int
    step: int
    time_s: float
    integration_state: np.ndarray
    latest_sample: Any = None
    view: PhysicalViewState | None = None
    outcome: str | None = None
    first_failure: Any = None

    def __post_init__(self) -> None:
        object.__setattr__(
            self, 'integration_state',
            _owned_array(self.integration_state, 'integration_state'))
        if self.view is not None and not isinstance(self.view, PhysicalViewState):
            raise ValueError('view must be a PhysicalViewState')


@dataclass(frozen=True)
class AdvanceResult:
    """The committed prefix of one advance() call (design section 3.2)."""

    step: int
    time_s: float
    reason: str
    outcome: str | None = None

    def __post_init__(self) -> None:
        if self.reason not in ('target', 'budget', 'outcome'):
            raise ValueError(f'unknown advance reason {self.reason!r}')


@dataclass(frozen=True)
class SampleBatch:
    """An acknowledged, boxed batch of completed sample rows.

    ``rows`` carries schema-2 ``PhysicalSample.as_dict()`` payloads;
    ``interval_ids`` is the parallel interval index. Both are owned tuples —
    removing pending native rows happens only after this object exists.
    """

    rows: tuple[Mapping[str, Any], ...] = ()
    interval_ids: tuple[int, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, 'rows', tuple(dict(r) for r in self.rows))
        object.__setattr__(self, 'interval_ids', tuple(int(i) for i in self.interval_ids))
        if len(self.rows) != len(self.interval_ids):
            raise ValueError('rows and interval_ids must have equal length')

    def as_dict_rows(self) -> list[dict]:
        """Materialize the schema-2 rows for tests and export."""
        return [dict(row) for row in self.rows]
