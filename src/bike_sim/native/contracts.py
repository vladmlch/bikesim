"""Owned, recursively read-only values crossing the native runtime boundary.

Numeric columns are boxed once per drain. Schema-2 dictionaries are materialized
only when requested; their native backing remains owned after acknowledgement,
reset and close. No published value aliases the engine or a scratch buffer.
"""
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from numbers import Integral
from types import MappingProxyType
from typing import Any

import numpy as np


def _owned_array(values, name: str) -> np.ndarray:
    array = np.array(values, dtype=np.float64, copy=True)
    if array.ndim != 1:
        raise ValueError(f'{name}: expected a vector')
    array.flags.writeable = False
    if not array.flags.owndata:
        raise ValueError(f'{name}: array must own its storage')
    return array


def _freeze(value):
    if isinstance(value, np.ndarray):
        result = np.array(value, copy=True)
        result.flags.writeable = False
        return result
    if isinstance(value, Mapping):
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_freeze(item) for item in value)
    if isinstance(value, np.generic):
        return value.item()
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    raise ValueError(f'unsupported native value: {type(value).__name__}')


def _plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, Mapping):
        return {key: _plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_plain(item) for item in value]
    return value


@dataclass(frozen=True)
class PhysicalViewState:
    """Published physical channels, independent of the render replica."""

    channels: Mapping[str, Any] = field(default_factory=dict)
    drive_mode: str = 'articulated_effort'
    sample: Any = None
    endpoint: Mapping[str, Any] = field(default_factory=dict)
    preview_row: Mapping[str, Any] = field(default_factory=dict)
    requested_scale: int = 1
    achieved_rtf: float | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, 'channels', _freeze(self.channels))
        object.__setattr__(self, 'endpoint', _freeze(self.endpoint))
        object.__setattr__(self, 'preview_row', _freeze(self.preview_row))


@dataclass(frozen=True)
class FrameSnapshot:
    """One committed frame boundary, including the latest closed interval."""

    generation: int
    step: int
    time_s: float
    integration_state: np.ndarray
    latest_sample: Any = None
    view: PhysicalViewState | None = None
    outcome: str | None = None
    first_failure: Any = None
    model_status: Mapping[str, Any] = field(default_factory=dict)
    model_fields: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, 'integration_state',
                           _owned_array(self.integration_state, 'integration_state'))
        object.__setattr__(self, 'first_failure', _freeze(self.first_failure))
        object.__setattr__(self, 'model_status', _freeze(self.model_status))
        object.__setattr__(self, 'model_fields', _freeze(self.model_fields))
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


@dataclass(frozen=True, init=False, eq=False)
class SampleBatch:
    """Acknowledged native columns with lazily boxed schema-2 rows.

    A native batch is retained by this value, not by its originating runtime.
    Column conversion and validation finish before the driver acknowledges
    the pending prefix. A later failed row export is retryable on this batch.
    """

    interval_ids: tuple[int, ...]
    columns: Mapping[str, np.ndarray]
    generation: int
    _rows: tuple[Mapping[str, Any], ...] | None
    _native_batch: Any

    def __init__(self, rows=(), interval_ids=(), *, columns=None, generation=0):
        owned_rows = tuple(_freeze(row) for row in rows)
        self._initialize(interval_ids, {} if columns is None else columns,
                         generation, owned_rows, None)

    def _initialize(self, interval_ids, columns, generation, rows, native_batch):
        ids = tuple(interval_ids)
        if any(isinstance(i, (bool, np.bool_)) or not isinstance(i, Integral)
               or i < 0 for i in ids):
            raise ValueError('interval_ids must contain nonnegative integers')
        ids = tuple(int(i) for i in ids)
        if any(right != left + 1 for left, right in zip(ids, ids[1:])):
            raise ValueError('batch intervals must be consecutive')
        if rows is not None and len(rows) != len(ids):
            raise ValueError('rows and interval_ids must have equal length')
        owned_columns = {name: _owned_array(values, name)
                         for name, values in columns.items()}
        if any(len(values) != len(ids) for values in owned_columns.values()):
            raise ValueError('columns and interval_ids must have equal length')
        object.__setattr__(self, 'interval_ids', ids)
        object.__setattr__(self, 'columns', MappingProxyType(owned_columns))
        object.__setattr__(self, 'generation', int(generation))
        object.__setattr__(self, '_rows', rows)
        object.__setattr__(self, '_native_batch', native_batch)

    @classmethod
    def from_native(cls, native_batch) -> SampleBatch:
        """Build all eager outputs without acknowledging the native prefix."""
        result = cls.__new__(cls)
        result._initialize(native_batch.interval_ids, native_batch.columns,
                           native_batch.generation, None, native_batch)
        return result

    def __len__(self) -> int:
        return len(self.interval_ids)

    @property
    def rows(self) -> tuple[Mapping[str, Any], ...]:
        if self._rows is None:
            rows = tuple(_freeze(row) for row in self._native_batch.as_dict_rows())
            if len(rows) != len(self.interval_ids) or any(
                    row['interval_id'] != interval_id
                    for row, interval_id in zip(rows, self.interval_ids)):
                raise ValueError('native rows do not match the acknowledged interval index')
            object.__setattr__(self, '_rows', rows)
        return self._rows

    def as_dict_rows(self) -> list[dict]:
        """Fresh mutable schema-2 dictionaries; edits cannot change this batch."""
        return [_plain(row) for row in self.rows]
