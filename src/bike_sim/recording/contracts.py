"""Immutable run-recording contract values (design R01, R08, R14).

Every value crossing into ``RunCapture`` is recursively frozen at
construction: mutable nested input cannot modify captured metadata after the
fact. Freezing snapshots containers -- a ``Mapping`` becomes a new
``MappingProxyType`` tree and a list becomes a tuple -- so later writes to
the caller's originals never reach recorded evidence.
"""
from __future__ import annotations

from collections.abc import Iterator, Mapping, Sequence
from dataclasses import dataclass, field
from datetime import datetime, timezone
from numbers import Integral, Real
from pathlib import Path
from types import MappingProxyType
from typing import Any, Protocol
from uuid import UUID

import numpy as np


def freeze(value):
    """Recursively immutable snapshot of a JSON-shaped value tree."""
    if type(value) in (float, int, str, bool, bytes, datetime, UUID) \
            or value is None:
        return value
    if isinstance(value, np.ndarray):
        if value.dtype.hasobject:
            raise ValueError('recording arrays must not contain Python objects')
        # Immutable bytes, not merely a reversible writeable=False flag. Even
        # setflags(write=True) cannot let a consumer mutate captured evidence.
        array = np.ascontiguousarray(value)
        return np.frombuffer(array.tobytes(), dtype=array.dtype).reshape(array.shape)
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Mapping):
        if any(not isinstance(key, str) for key in value):
            raise ValueError('recording object keys must be strings')
        return MappingProxyType({key: freeze(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(freeze(item) for item in value)
    raise ValueError(
        f'unsupported recording value: {type(value).__name__}')


def _integer(value, name: str, *, minimum: int = 0) -> int:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral):
        raise ValueError(f'{name} must be an integer')
    value = int(value)
    if value < minimum:
        raise ValueError(f'{name} must be >= {minimum}')
    return value


def _text(value, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f'{name} must be a non-empty string')
    return value


def _finite(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f'{name} must be a real number')
    result = float(value)
    if not np.isfinite(result):
        raise ValueError(f'{name} must be finite')
    return result


@dataclass(frozen=True)
class RunMetadata:
    """One CLI launch: a run UUID, UTC start and frozen provenance values."""

    run_id: UUID
    started_at: datetime
    backend: str
    mode: str
    values: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not isinstance(self.run_id, UUID):
            raise ValueError('run_id must be a UUID')
        if not isinstance(self.started_at, datetime):
            raise ValueError('started_at must be a datetime')
        if self.started_at.tzinfo is None or self.started_at.utcoffset() is None:
            raise ValueError('started_at must be timezone-aware (UTC)')
        object.__setattr__(self, 'started_at', self.started_at.astimezone(timezone.utc))
        _text(self.backend, 'backend')
        _text(self.mode, 'mode')
        object.__setattr__(self, 'values', freeze(self.values))


@dataclass(frozen=True)
class InputEvent:
    """One journaled source input on a physical episode timeline (R11)."""

    episode_id: int
    event_id: int
    kind: str
    received_wall_s: float
    processed_step: int
    applied_step: int | None
    values: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        _integer(self.episode_id, 'episode_id')
        _integer(self.event_id, 'event_id')
        _text(self.kind, 'kind')
        _finite(self.received_wall_s, 'received_wall_s')
        _integer(self.processed_step, 'processed_step')
        if self.applied_step is not None:
            applied = _integer(self.applied_step, 'applied_step')
            if applied < self.processed_step:
                raise ValueError(
                    'applied_step precedes the step the input was processed')
        object.__setattr__(self, 'values', freeze(self.values))


@dataclass(frozen=True)
class Asset:
    """A named bundled input: bytes with media type and content hash (R12)."""

    name: str
    media_type: str
    sha256: str
    content: bytes

    def __post_init__(self) -> None:
        _text(self.name, 'asset name')
        _text(self.media_type, 'media_type')
        if not isinstance(self.sha256, str) or len(self.sha256) != 64 \
                or any(c not in '0123456789abcdef' for c in self.sha256):
            raise ValueError('sha256 must be a 64-digit lowercase hex digest')
        if not isinstance(self.content, bytes):
            raise ValueError('asset content must be bytes')


@dataclass(frozen=True)
class TableBatch:
    """One typed table fragment: columns by name plus an explicit row count.

    Columns are owned read-only sequences: a passed ``ndarray`` is copied
    into a non-writeable array and any other sequence is copied into a
    tuple, so a producer cannot mutate a batch after handing it over.
    """

    table: str
    columns: Mapping[str, Sequence[Any] | np.ndarray]
    row_count: int

    def __post_init__(self) -> None:
        _text(self.table, 'table')
        _integer(self.row_count, 'row_count')
        frozen = {}
        for name, column in self.columns.items():
            if len(column) != self.row_count:
                raise ValueError(
                    f'{self.table}.{name}: column length {len(column)} '
                    f'does not match row_count {self.row_count}')
            if isinstance(column, np.ndarray):
                column = freeze(column)
            else:
                column = tuple(column)
            frozen[str(name)] = column
        object.__setattr__(self, 'columns', MappingProxyType(frozen))


@dataclass(frozen=True)
class PieceBatch:
    """Native-emitted table fragment: column pieces for the Arrow assembler.

    ``pieces`` maps emitted column name to the projector's piece dict; the
    mapping is proxied while the buffers inside stay owned by the binding.
    """

    table: str
    row_count: int
    pieces: Mapping[str, Any]

    def __post_init__(self):
        _text(self.table, 'table')
        _integer(self.row_count, 'row_count')
        object.__setattr__(self, 'pieces', MappingProxyType(dict(self.pieces)))


class PhysicalSource(Protocol):
    """Owned physical evidence: typed table batches on demand (R14)."""

    row_count: int

    def tables(self, *, batch_rows: int) -> Iterator[TableBatch]: ...


class ResearchSource(Protocol):
    """Owned control-rate history. No method may call a live environment."""

    def tables(self, *, batch_rows: int) -> Iterator[TableBatch]: ...


@dataclass(frozen=True)
class ResearchEvidence:
    history: ResearchSource
    initial_integration_state: np.ndarray
    final_integration_state: np.ndarray
    replay_contract_version: int
    complete_control_steps: int
    replay_eligible: bool

    def __post_init__(self) -> None:
        if not callable(getattr(self.history, 'tables', None)):
            raise ValueError('research history must implement tables')
        if self.replay_contract_version != 3:
            raise ValueError('new recordings require research replay contract 3')
        _integer(self.complete_control_steps, 'complete_control_steps')
        if type(self.replay_eligible) is not bool:
            raise ValueError('replay_eligible must be a bool')
        for name in ('initial_integration_state', 'final_integration_state'):
            array = np.asarray(getattr(self, name))
            if array.dtype != np.dtype('float64') or array.ndim != 1:
                raise ValueError(f'{name} must be a one-dimensional float64 array')
            object.__setattr__(self, name, freeze(array))
        if self.initial_integration_state.shape != self.final_integration_state.shape:
            raise ValueError('initial and final integration state shapes differ')


@dataclass(frozen=True)
class EpisodeEvidence:
    """One reset-separated physical timeline committed to a run (R01, R03)."""

    episode_id: int
    generation: int
    physical: PhysicalSource
    summary: Mapping[str, Any]
    assets: tuple[Asset, ...]
    research: ResearchEvidence | None = None

    def __post_init__(self) -> None:
        _integer(self.episode_id, 'episode_id')
        _integer(self.generation, 'generation')
        physical = self.physical
        rows = getattr(physical, 'row_count', None)
        _integer(rows, 'physical.row_count')
        if not callable(getattr(physical, 'tables', None)):
            raise ValueError('physical must implement tables(batch_rows=...)')
        object.__setattr__(self, 'summary', freeze(self.summary))
        assets = tuple(self.assets)
        for asset in assets:
            if not isinstance(asset, Asset):
                raise ValueError('assets must contain Asset values')
        object.__setattr__(self, 'assets', assets)
        if self.research is not None and not isinstance(self.research, ResearchEvidence):
            raise ValueError('research must be ResearchEvidence or None')
