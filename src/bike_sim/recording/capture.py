"""One launch's in-RAM recording owner (design R02, R03, R14).

``RunCapture`` retains completed ``EpisodeEvidence`` values and the ordered
input-event journal until launch finalization. ``event()`` and
``add_episode()`` never touch a storage destination; only ``finalize()``
runs the publication seam. A disabled capture validates identically but
retains nothing and has no persistent side effect.

State machine: ``collecting`` accepts evidence; ``publishing`` is the
in-flight publication; ``published`` returns the sealed path idempotently;
``publication_failed`` preserves the owned capture so ``finalize`` may be
retried while the process lives (R04, R16).
"""
from collections.abc import Iterator
from pathlib import Path

from bike_sim.recording.contracts import (
    EpisodeEvidence, InputEvent, RunMetadata,
)

COLLECTING = 'collecting'
PUBLISHING = 'publishing'
PUBLISHED = 'published'
PUBLICATION_FAILED = 'publication_failed'


def _default_publisher():
    """Resolve the storage writer lazily; absent until A4 lands storage.py."""
    from bike_sim.recording.storage import write_recording
    return write_recording


class RunCapture:
    """The single recording owner for one CLI launch (R14)."""

    def __init__(self, metadata: RunMetadata, *, enabled: bool,
                 _publisher=None):
        if not isinstance(metadata, RunMetadata):
            raise TypeError('metadata must be a RunMetadata')
        if not isinstance(enabled, bool):
            raise TypeError('enabled must be a bool')
        if _publisher is not None and not callable(_publisher):
            raise TypeError('publisher must be callable')
        self._metadata = metadata
        self._enabled = enabled
        self._publisher = _publisher
        self._state = COLLECTING
        self._episodes = {}
        self._episode_order = []
        self._events = []
        self._last_event_key = None
        self._published_path = None

    @property
    def metadata(self) -> RunMetadata:
        return self._metadata

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def state(self) -> str:
        return self._state

    def _require_collecting(self, what: str) -> None:
        if self._state != COLLECTING:
            raise RuntimeError(
                f'cannot record {what}: run capture is {self._state}')

    def event(self, event: InputEvent) -> None:
        """Validate and retain one journaled input event.

        Event identity ``(episode_id, event_id)`` must strictly increase in
        journal order: a reset-separated episode restarts ``event_id``, and
        events for a closed episode or out-of-order IDs are rejected.
        """
        if not isinstance(event, InputEvent):
            raise TypeError('event must be an InputEvent')
        key = (event.episode_id, event.event_id)
        if self._state != COLLECTING:
            self._require_collecting('event')
        if self._last_event_key is not None and key <= self._last_event_key:
            raise ValueError(
                f'event identity {key} does not follow '
                f'{self._last_event_key}')
        self._last_event_key = key
        if self._enabled:
            self._events.append(event)

    def add_episode(self, evidence: EpisodeEvidence) -> None:
        """Commit one sealed episode, atomically.

        Episode identifiers are never reused: re-adding the identical owned
        evidence is idempotent; a different evidence under a committed or
        regressed identifier is a conflicting repeat and is rejected whole.
        """
        if not isinstance(evidence, EpisodeEvidence):
            raise TypeError('evidence must be an EpisodeEvidence')
        self._require_collecting('episode')
        if not self._enabled:
            return
        existing = self._episodes.get(evidence.episode_id)
        if existing is not None:
            if existing is evidence or existing == evidence:
                return
            raise ValueError(
                f'conflicting evidence for episode {evidence.episode_id}')
        if self._episode_order and \
                evidence.episode_id <= self._episode_order[-1]:
            raise ValueError(
                f'episode identifier {evidence.episode_id} was already used')
        self._episodes[evidence.episode_id] = evidence
        self._episode_order.append(evidence.episode_id)

    def iter_episodes(self) -> Iterator[EpisodeEvidence]:
        """Committed episodes in identity order; read-only, non-consuming."""
        return (self._episodes[key] for key in self._episode_order)

    def iter_events(self) -> Iterator[InputEvent]:
        """Retained input events in journal order; read-only."""
        return iter(tuple(self._events))

    def finalize(self, out_root: Path) -> Path | None:
        """Publish the recording once at launch end.

        Disabled captures return ``None`` and create nothing. An enabled
        capture runs the injected publication seam (storage by default),
        caches the sealed path, and returns it on every later call. A
        publication failure keeps the owned evidence so the launch can retry.
        """
        out_root = Path(out_root)
        if not self._enabled:
            if self._state == COLLECTING:
                self._state = PUBLISHED
            return None
        if self._state == PUBLISHED:
            return self._published_path
        if self._state == PUBLISHING:
            raise RuntimeError('finalize() re-entered during publication')
        self._state = PUBLISHING
        try:
            publisher = self._publisher or _default_publisher()
            path = publisher(self, Path(out_root))
            if not isinstance(path, Path):
                path = Path(path)
            self._published_path = path
            self._state = PUBLISHED
            return path
        except BaseException:
            self._state = PUBLICATION_FAILED
            raise
