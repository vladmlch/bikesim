"""RunCapture contract tests: ownership, identity, ordering, finalization."""
from pathlib import Path

import pytest

from bike_sim.recording.capture import RunCapture
from bike_sim.recording.contracts import EpisodeEvidence, InputEvent, RunMetadata

from _recording_support import (
    TreePhysicalSource, episode, run_metadata, sample_row,
)


def _event(episode_id=0, event_id=0, *, kind='brake_strength',
           received=0.0, processed=0, applied=0, values=None):
    return InputEvent(
        episode_id=episode_id, event_id=event_id, kind=kind,
        received_wall_s=received, processed_step=processed,
        applied_step=applied,
        values={} if values is None else values)


def test_disabled_capture_has_no_persistent_side_effect(tmp_path):
    target = tmp_path / 'not-created'
    capture = RunCapture(run_metadata(), enabled=False)
    assert capture.finalize(target) is None
    assert not target.exists()


def test_metadata_must_be_a_run_metadata():
    with pytest.raises(TypeError):
        RunCapture({'backend': 'native'}, enabled=True)
    with pytest.raises(TypeError):
        RunCapture(run_metadata(), enabled='yes')


def test_disabled_capture_still_validates_but_retains_nothing(tmp_path):
    capture = RunCapture(run_metadata(), enabled=False)
    capture.event(_event())
    capture.add_episode(episode(TreePhysicalSource([sample_row(0)])))
    assert list(capture.iter_events()) == []
    assert list(capture.iter_episodes()) == []
    with pytest.raises(ValueError):
        capture.event(_event(0, 0))  # ordering rules apply when disabled too
    assert capture.finalize(tmp_path / 'run') is None
    assert not (tmp_path / 'run').exists()


def test_conflicting_episode_ids_reject_atomically():
    capture = RunCapture(run_metadata(), enabled=True)
    first = episode(TreePhysicalSource([sample_row(0)]), episode_id=0)
    capture.add_episode(first)
    conflicting = episode(
        TreePhysicalSource([sample_row(0), sample_row(1)]), episode_id=0)
    with pytest.raises(ValueError, match='conflict'):
        capture.add_episode(conflicting)
    # The failed addition changed nothing: the original evidence is intact.
    retained = list(capture.iter_episodes())
    assert retained == [first]
    assert retained[0].physical is first.physical


def test_identical_episode_readdition_is_idempotent():
    capture = RunCapture(run_metadata(), enabled=True)
    evidence = episode(TreePhysicalSource([sample_row(0)]), episode_id=0)
    capture.add_episode(evidence)
    capture.add_episode(evidence)
    assert list(capture.iter_episodes()) == [evidence]


def test_episode_identifiers_are_never_reused():
    capture = RunCapture(run_metadata(), enabled=True)
    capture.add_episode(episode(TreePhysicalSource([sample_row(0)]), episode_id=1))
    with pytest.raises(ValueError):
        capture.add_episode(
            episode(TreePhysicalSource([sample_row(0)]), episode_id=0))
    assert len(list(capture.iter_episodes())) == 1


def test_event_identity_and_order_are_validated():
    capture = RunCapture(run_metadata(), enabled=True)
    capture.event(_event(0, 0))
    capture.event(_event(0, 1))
    capture.event(_event(1, 0))  # a reset-separated episode restarts event ids
    capture.event(_event(1, 3))
    assert [e.event_id for e in capture.iter_events()] == [0, 1, 0, 3]
    with pytest.raises(ValueError):
        capture.event(_event(0, 5))   # an earlier episode is closed
    with pytest.raises(ValueError):
        capture.event(_event(1, 3))   # duplicate identity
    with pytest.raises(ValueError):
        capture.event(_event(1, 2))   # out-of-order event id


def test_event_field_validation():
    capture = RunCapture(run_metadata(), enabled=True)
    with pytest.raises(ValueError):
        capture.event(_event(0, 0, received=float('nan')))
    with pytest.raises(ValueError):
        capture.event(_event(0, 0, applied=-1))
    with pytest.raises(ValueError):
        capture.event(_event(0, 0, processed=4, applied=2))
    with pytest.raises(ValueError):
        _event(0, 0, kind='')
    with pytest.raises(ValueError):
        _event(-1, 0)
    capture.event(_event(0, 0, applied=None))  # requested but never applied


def test_zero_interval_episode_evidence_is_valid():
    capture = RunCapture(run_metadata(), enabled=True)
    empty = episode(TreePhysicalSource([]), episode_id=0)
    assert empty.physical.row_count == 0
    capture.add_episode(empty)
    assert list(capture.iter_episodes()) == [empty]


def test_episode_evidence_contract_is_validated():
    capture = RunCapture(run_metadata(), enabled=True)
    with pytest.raises(ValueError):
        episode(TreePhysicalSource([]), episode_id=-1)
    with pytest.raises(ValueError):
        episode(TreePhysicalSource([]), episode_id=True)
    with pytest.raises(ValueError):
        EpisodeEvidence(episode_id=0, generation=0,
                        physical=object(), summary={}, assets=())
    with pytest.raises(ValueError):
        EpisodeEvidence(episode_id=0, generation=0,
                        physical=TreePhysicalSource([]), summary={},
                        assets=(), research={'policy': 'anything'})


def test_nested_mutable_source_input_cannot_modify_captured_metadata():
    nested = {'terrain': {'grade': [0.1, 0.2]}, 'seed': 19}
    metadata = RunMetadata(
        run_id=run_metadata().run_id,
        started_at=run_metadata().started_at,
        backend='native', mode='ride', values=nested)
    nested['terrain']['grade'][0] = 9.9
    nested['seed'] = -1
    nested['new_key'] = True
    values = capture_values = RunCapture(metadata, enabled=True).metadata.values
    assert capture_values['terrain']['grade'] == (0.1, 0.2)
    assert capture_values['seed'] == 19
    assert 'new_key' not in capture_values
    with pytest.raises(TypeError):
        values['terrain'] = {}
    with pytest.raises(TypeError):
        values['terrain']['grade'] = 0.


def test_event_and_summary_values_are_frozen():
    values = {'request': {'brake': [0.5]}}
    event = _event(values=values)
    values['request']['brake'].append(1.0)
    assert event.values['request']['brake'] == (0.5,)
    with pytest.raises(TypeError):
        event.values['request']['brake'] = 0.


def _recording_publisher(target, calls):
    def publish(capture, out_root):
        calls.append(out_root)
        Path(out_root).mkdir(parents=True, exist_ok=True)
        return target
    return publish


def test_finalize_publishes_once_and_retries_failed_publication(tmp_path):
    calls = []
    capture = RunCapture(run_metadata(), enabled=True,
                         _publisher=_recording_publisher(tmp_path / 'run-a', calls))
    capture.add_episode(episode(TreePhysicalSource([sample_row(0)])))
    assert capture.state == 'collecting'
    result = capture.finalize(tmp_path)
    assert result == tmp_path / 'run-a'
    assert capture.state == 'published'
    # Idempotent: a repeated finalize returns the same path without re-publishing.
    assert capture.finalize(tmp_path) == tmp_path / 'run-a'
    assert calls == [tmp_path]


def test_failed_publication_keeps_capture_retryable(tmp_path):
    attempts = []

    def flaky(capture, out_root):
        attempts.append(out_root)
        if len(attempts) == 1:
            raise OSError('injected publication failure')
        Path(out_root).mkdir(parents=True, exist_ok=True)
        return out_root / 'sealed'

    capture = RunCapture(run_metadata(), enabled=True, _publisher=flaky)
    with pytest.raises(OSError, match='injected publication failure'):
        capture.finalize(tmp_path)
    assert capture.state == 'publication_failed'
    assert capture.finalize(tmp_path) == tmp_path / 'sealed'
    assert capture.state == 'published'


def test_finalize_without_a_publisher_fails_without_side_effect(tmp_path):
    capture = RunCapture(run_metadata(), enabled=True)
    target = tmp_path / 'run'
    with pytest.raises(Exception):
        capture.finalize(target)
    assert capture.state == 'publication_failed'
    assert not target.exists()


def test_appends_are_rejected_after_publication(tmp_path):
    capture = RunCapture(run_metadata(), enabled=True,
                         _publisher=_recording_publisher(tmp_path / 'run', []))
    capture.finalize(tmp_path)
    with pytest.raises(RuntimeError):
        capture.event(_event(0, 0))
    with pytest.raises(RuntimeError):
        capture.add_episode(episode(TreePhysicalSource([])))
    failed = RunCapture(run_metadata(), enabled=True)
    with pytest.raises(Exception):
        failed.finalize(tmp_path / 'x')
    with pytest.raises(RuntimeError):
        failed.event(_event(0, 0))
