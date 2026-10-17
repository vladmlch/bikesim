"""T20 recovery ownership and cleanup controls; no physics or child process.

The publisher double models publish_close's replacement of per-period rows.
The real runtime recovery and reference-monitor paths must retain every
published period while propagating the original failure exactly once.
"""
from __future__ import annotations

import pickle

import pytest

from bike_sim.sim.ride import period_buffer
from bike_sim.sim.ride.async_close import AsyncWorkerLost, _close_queues
from bike_sim.sim.ride.physical_runtime import PhysicalRuntime
from bike_sim.sim.ride.reference_monitor import InvalidReferenceRun, ReferenceMonitor


def _runtime(monkeypatch, *, prefix=(), rejected=None, failure=None):
    runtime = PhysicalRuntime.__new__(PhysicalRuntime)
    runtime.completed_samples = tuple(prefix)
    runtime.interval_constraints = {item: f'constraint-{item}' for item in prefix}
    runtime.sample = prefix[-1] if prefix else None
    runtime.reference_monitor = ReferenceMonitor(strict=True)
    runtime._async_close = None
    runtime._async_ctx_dirty = False
    runtime._buffer = object()
    calls = []

    def publish(owner, raws):
        assert owner is runtime
        calls.append(raws)
        if failure is not None and raws is failure[0]:
            raise failure[1]
        owner.completed_samples = tuple(raws)
        owner.interval_constraints = {item: f'constraint-{item}' for item in raws}
        owner.sample = raws[-1]
        return [(0.002, ('forced-budget',))] if raws is rejected else []

    monkeypatch.setattr(period_buffer, 'publish_close', publish)
    return runtime, calls


def test_replay_retains_every_period_and_previously_published_prefix(monkeypatch):
    runtime, calls = _runtime(monkeypatch, prefix=(0,))
    periods = [[1, 2], [3], [4, 5]]
    buffer = runtime._buffer

    runtime._replay_async_periods(periods)

    assert calls == periods
    assert all(actual is expected for actual, expected in zip(calls, periods))
    assert runtime.completed_samples == (0, 1, 2, 3, 4, 5)
    assert runtime.interval_constraints == {i: f'constraint-{i}' for i in range(6)}
    assert runtime.sample == 5
    assert runtime._buffer is buffer


def test_replay_does_not_carry_constraints_from_an_old_publication_window(monkeypatch):
    runtime, _ = _runtime(monkeypatch)
    runtime.interval_constraints = {-1: 'old-window'}

    runtime._replay_async_periods([[1], [2]])

    assert runtime.completed_samples == (1, 2)
    assert runtime.interval_constraints == {1: 'constraint-1', 2: 'constraint-2'}


def test_empty_replay_leaves_publication_state_untouched(monkeypatch):
    runtime, calls = _runtime(monkeypatch)
    previous = {-1: 'old-window'}
    runtime.interval_constraints = previous

    runtime._replay_async_periods([])

    assert calls == []
    assert runtime.completed_samples == ()
    assert runtime.interval_constraints is previous


def test_strict_rejection_keeps_published_prefix_and_stops_before_later_periods(monkeypatch):
    rejected = [2, 3]
    runtime, calls = _runtime(monkeypatch, prefix=(0,), rejected=rejected)

    with pytest.raises(InvalidReferenceRun, match='forced-budget'):
        runtime._replay_async_periods([[1], rejected, [4]])

    assert calls == [[1], rejected]
    assert runtime.completed_samples == (0, 1, 2, 3)
    assert runtime.interval_constraints == {i: f'constraint-{i}' for i in range(4)}
    assert runtime.sample == 3
    assert runtime.reference_monitor.first_failure == (0.002, ('forced-budget',))


def test_failure_before_publication_restores_prefix_without_repeating_it(monkeypatch):
    failed = [2]
    original = RuntimeError('publication failed before rows were produced')
    runtime, calls = _runtime(monkeypatch, prefix=(0,), failure=(failed, original))

    with pytest.raises(RuntimeError) as caught:
        runtime._replay_async_periods([[1], failed, [3]])

    assert caught.value is original
    assert calls == [[1], failed]
    assert runtime.completed_samples == (0, 1)
    assert runtime.interval_constraints == {0: 'constraint-0', 1: 'constraint-1'}
    assert runtime.sample == 1


class _RecoveryRunner:
    def __init__(self, pending, *, failure_phase='submit', cleanup_error=None):
        self.pending = list(pending)
        self.failure_phase = failure_phase
        self.cleanup_error = cleanup_error
        self.close_calls = self.context_calls = self.submit_calls = 0

    def send_context(self, context):
        self.context_calls += 1
        if self.failure_phase == 'context':
            raise pickle.PicklingError('context serialization failed')

    def submit(self, raws):
        self.submit_calls += 1
        if self.failure_phase == 'registration':
            self.pending.append(raws)
        raise pickle.PicklingError('period serialization failed')

    def collect(self, *, block=False):
        raise AsyncWorkerLost('worker exited before acknowledging the batch')

    def detach_pending(self):
        pending, self.pending = self.pending, []
        return pending

    def close(self):
        self.close_calls += 1
        if self.cleanup_error is not None:
            raise self.cleanup_error


@pytest.mark.parametrize('failure_phase', ['context', 'submit', 'registration'])
def test_submission_recovery_publishes_pending_and_current_periods_once(monkeypatch, failure_phase):
    runtime, calls = _runtime(monkeypatch, prefix=(0,))
    pending, current = [[1], [2]], [3]
    runner = _RecoveryRunner(pending, failure_phase=failure_phase)
    runtime._async_close = runner
    runtime._async_ctx_dirty = True
    runtime._async_context = lambda: {'plain': 'context'}

    runtime._async_submit(current)

    assert calls == pending + [current]
    assert runtime.completed_samples == (0, 1, 2, 3)
    assert runtime.interval_constraints == {i: f'constraint-{i}' for i in range(4)}
    assert runner.context_calls == 1
    assert runner.submit_calls == (0 if failure_phase == 'context' else 1)
    assert runner.close_calls == 1
    assert runtime._async_close is None
    assert runtime._async_ctx_dirty is False


def test_transport_recovery_preserves_every_period_without_a_second_replay(monkeypatch):
    runtime, calls = _runtime(monkeypatch)
    periods = [[1], [2, 3]]
    runner = _RecoveryRunner(periods)
    runtime._async_close = runner

    assert runtime._drain_async(block=True) is True
    assert runtime._drain_async(block=True) is False

    assert calls == periods
    assert runtime.completed_samples == (1, 2, 3)
    assert runtime.interval_constraints == {i: f'constraint-{i}' for i in (1, 2, 3)}
    assert runner.close_calls == 1
    assert runtime._async_close is None


def test_transport_cleanup_error_still_preserves_recovered_rows(monkeypatch):
    runtime, calls = _runtime(monkeypatch)
    periods = [[1], [2]]
    original = OSError('worker cleanup failed')
    runtime._async_close = _RecoveryRunner(periods, cleanup_error=original)

    with pytest.raises(OSError) as caught:
        runtime._drain_async(block=True)

    assert caught.value is original
    assert calls == periods
    assert runtime.completed_samples == (1, 2)
    assert runtime.interval_constraints == {1: 'constraint-1', 2: 'constraint-2'}
    assert runtime._async_close is None


class _Queue:
    def __init__(self, name, events, errors=None):
        self.name, self.events, self.errors = name, events, errors or {}

    def _operation(self, name):
        self.events.append((self.name, name))
        if name in self.errors:
            raise self.errors[name]

    def cancel_join_thread(self):
        self._operation('cancel_join_thread')

    def close(self):
        self._operation('close')


QUEUE_OPERATIONS = [
    ('in', 'cancel_join_thread'), ('in', 'close'),
    ('out', 'cancel_join_thread'), ('out', 'close'),
]


def test_queue_cleanup_attempts_all_operations_on_success():
    events = []
    queues = [_Queue('in', events), _Queue('out', events)]
    assert _close_queues(iter(queues)) is None
    assert events == QUEUE_OPERATIONS
    assert _close_queues(()) is None


@pytest.mark.parametrize('failed_queue,failed_operation', QUEUE_OPERATIONS)
def test_queue_cleanup_attempts_remaining_operations_after_each_failure(failed_queue, failed_operation):
    events = []
    original = OSError('queue cleanup failed')
    queues = [_Queue(name, events, {failed_operation: original} if name == failed_queue else {})
              for name in ('in', 'out')]

    with pytest.raises(OSError) as caught:
        _close_queues(queues)

    assert caught.value is original
    assert events == QUEUE_OPERATIONS


def test_queue_cleanup_preserves_initial_interrupt_and_notes_later_failures():
    events = []
    original = KeyboardInterrupt('cleanup interrupted')
    queues = [
        _Queue('in', events, {'cancel_join_thread': original,
                              'close': OSError('input close failed')}),
        _Queue('out', events, {'close': RuntimeError('output close failed')}),
    ]

    with pytest.raises(KeyboardInterrupt) as caught:
        _close_queues(queues)

    assert caught.value is original
    assert events == QUEUE_OPERATIONS
    assert len(original.__notes__) == 2
    assert 'input close failed' in original.__notes__[0]
    assert 'output close failed' in original.__notes__[1]
