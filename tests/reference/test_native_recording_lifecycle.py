"""A3 launch-level run-recording lifecycle contracts (R03/R11/R14/R16).

Fake-window viewer-pump tests (fake input + fake clock) sit beside real
native-driver adapter tests: episodes seal exactly once, the input journal
stays receipt-ordered, publication happens only at ``finalize``, and every
failure path keeps its owned evidence alive for retry or post-mortem.
"""
import signal
from types import SimpleNamespace

import pytest

from bike_sim.recording.capture import RunCapture
from bike_sim.recording.contracts import EpisodeEvidence
from bike_sim.recording.native import (DERIVED_TRANSITION_RULE,
                                       episode_interval_for_time,
                                       iter_derived_events)
from bike_sim.sim.ride.physical_frontend import (make_run_capture,
                                                 run_headless, run_metadata,
                                                 run_viewer)
from bike_sim.sim.ride.reference_monitor import InvalidReferenceRun

from _recording_support import TreePhysicalSource, sample_row
from native_loader import load_native
from test_frontend_windows import (Clock, Driver, FakeEpisode, FakePhysical,
                                   patch_window, row)

# The selected-artifact import doubles as the sanitizer-runtime check.
bike_native = load_native()


def _capture(published, *, backend='native', mode='viewer', publisher=None,
             strict=False):
    """An enabled capture whose publication seam is a spy.

    Metadata goes through the real ``run_metadata`` builder so the
    resolved check policy stays part of the launch record.
    """
    def spy(capture, out_root):
        published.append((capture, out_root))
        path = out_root/'recording.marker'
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('published')
        return path
    driver = SimpleNamespace(backend=backend, strict=strict)
    return RunCapture(run_metadata(driver, mode=mode), enabled=True,
                      _publisher=spy if publisher is None else publisher)


def _recording(*, mode='viewer', publisher=None):
    """An enabled capture on the real DuckDB publication seam.

    An optional ``publisher`` wraps the real writer (e.g. to inject a
    second Ctrl-C inside the shielded finalize).
    """
    driver = SimpleNamespace(backend='native', strict=False)
    return RunCapture(run_metadata(driver, mode=mode), enabled=True,
                      _publisher=publisher)


def _interrupting_publisher():
    """The real storage seam, with a second SIGINT delivered at its start."""
    from bike_sim.recording.storage import write_recording
    def publish(capture, out_root):
        signal.raise_signal(signal.SIGINT)
        return write_recording(capture, out_root)
    return publish


def kinds(events, episode_id=None):
    return [event.kind for event in events
            if episode_id is None or event.episode_id == episode_id]


class RecordingDriver(Driver):
    """Window-pump driver with seal/episodic fakes and injectable failures."""

    def __init__(self, clock, *, published=None, episode_rows=(),
                 first_failure=None, advance_failures=(), seal_failures=()):
        super().__init__(clock)
        self._published = published
        self.episode_rows = list(episode_rows) or [[]]
        self._first_failure = first_failure
        self.advance_calls = 0
        self._advance_failures = dict(advance_failures)
        self._seal_failures = list(seal_failures)
        self.seal_calls = []

    def advance(self, target, **kwargs):
        # Publication must never run inside the pump (R14/R16).
        assert not self._published, 'publisher ran during the viewer pump'
        self.advance_calls += 1
        error = self._advance_failures.get(self.advance_calls)
        if error is not None:
            raise error
        return super().advance(target, **kwargs)

    def summary(self, reason=None):
        self.events.append(('summary', self.generation))
        return {'outcome': {'reason': reason, 'time_s': self.time_s,
                            'position_m': 0.},
                'model_status': {'model_valid': True},
                'first_failure': self._first_failure}

    def seal_episode(self, episode_id, *, reason):
        self.seal_calls.append((self.generation, episode_id, reason))
        self.events.append(('seal', self.generation, episode_id))
        if self._seal_failures:
            raise self._seal_failures.pop(0)
        rows = self.episode_rows[min(self.generation-1,
                                     len(self.episode_rows)-1)]
        episode = FakeEpisode(self.generation, rows)
        return EpisodeEvidence(
            episode_id=episode_id, generation=self.generation,
            physical=FakePhysical(episode),
            summary={'outcome': {'reason': reason, 'time_s': self.time_s,
                                 'position_m': 0.},
                     'model_status': {'model_valid': True},
                     'first_failure': self._first_failure},
            assets=())


class StoredPhysical(TreePhysicalSource):
    """Storage-valid physical source that also feeds the event deriver.

    ``tables``/``catalog`` come from ``TreePhysicalSource`` over authored
    sample trees; ``episode`` exposes a ``FakeEpisode`` of channel rows so
    ``iter_derived_events`` still re-reads the sealed episode.
    """

    def __init__(self, rows, deriver_episode):
        super().__init__(rows)
        self._deriver_episode = deriver_episode

    @property
    def episode(self):
        return self._deriver_episode


class StoringDriver(RecordingDriver):
    """Window-pump driver whose sealed evidence is storage-valid.

    ``seal_hooks`` run inside ``seal_episode`` — a deterministic seam to
    deliver a second Ctrl-C mid-seal (``signal.raise_signal(SIGINT)``).
    """

    def __init__(self, clock, *, seal_hooks=(), **kwargs):
        super().__init__(clock, **kwargs)
        self._seal_hooks = list(seal_hooks)

    def seal_episode(self, episode_id, *, reason):
        if self._seal_hooks:
            self._seal_hooks.pop(0)()
        self.seal_calls.append((self.generation, episode_id, reason))
        self.events.append(('seal', self.generation, episode_id))
        if self._seal_failures:
            raise self._seal_failures.pop(0)
        rows = self.episode_rows[min(self.generation-1,
                                     len(self.episode_rows)-1)]
        stored = [sample_row(i, start=i*.001, dt=.001)
                  for i in range(len(rows))]
        return EpisodeEvidence(
            episode_id=episode_id, generation=self.generation,
            physical=StoredPhysical(
                stored, FakeEpisode(self.generation, rows)),
            summary={'outcome': {'reason': reason, 'time_s': self.time_s,
                                 'position_m': 0.},
                     'model_status': {'model_valid': True},
                     'first_failure': self._first_failure},
            assets=())


def _second_sigint():
    """A second Ctrl-C delivered inside the shielded critical section."""
    signal.raise_signal(signal.SIGINT)


def _event_kinds(path, episode_id=0):
    from bike_sim.recording.reader import Recording
    with Recording(path) as saved:
        return [entry['kind'] for entry in saved.rows(
            'SELECT kind FROM events WHERE episode_id=? ORDER BY event_id',
            [episode_id])]


def test_viewer_journals_two_episodes_and_publishes_once(tmp_path, monkeypatch):
    import bike_sim.sim.ride.physical_frontend as frontend
    clock = Clock()
    keys = {5: [32],          # brake on
            8: [44],          # strength down while braking -> applied record
            9: [46],          # strength up   while braking -> applied record
            15: [ord('R')],   # reset: seal episode 0, install episode 1
            20: [32],         # brake on in episode 1
            25: [44],         # strength down while OFF? still braking -> applied
            30: [ord('Q')]}   # quit
    patch_window(monkeypatch, frontend, clock, keys)
    published = []
    driver = RecordingDriver(clock, published=published)
    assert run_viewer(driver, out_root=tmp_path,
                      capture=_capture(published)) == 0
    assert driver.closed
    assert driver.events == [('seal', 1, 0), ('reset', 1), ('seal', 2, 1)]
    assert len(published) == 1
    capture, _root = published[0]
    episodes = list(capture.iter_episodes())
    assert [e.episode_id for e in episodes] == [0, 1]
    assert [e.generation for e in episodes] == [1, 2]
    events = list(capture.iter_events())
    ep0 = kinds(events, 0)
    assert ep0[:2] == ['episode_start', 'brake_toggle']
    # Requested inputs arrive in receipt order; each applied demand record
    # follows the request that changed it.
    assert ep0 == ['episode_start', 'brake_toggle', 'brake_demand',
                   'brake_strength', 'brake_demand',
                   'brake_strength', 'brake_demand', 'reset']
    assert kinds(events, 1) == ['episode_start', 'brake_toggle',
                              'brake_demand', 'brake_strength',
                              'brake_demand', 'quit']
    by_kind = {(e.episode_id, e.kind): e for e in events}
    toggle = by_kind[0, 'brake_toggle']
    applied = by_kind[0, 'brake_demand']
    # received (wall receipt) and processed (physics step) stay distinct.
    assert toggle.received_wall_s <= applied.received_wall_s
    assert toggle.values['record'] == 'requested'
    assert toggle.values['braking'] is True
    assert applied.values['record'] == 'applied'
    assert applied.values['front_brake_demand'] == pytest.approx(.5)
    assert applied.applied_step == applied.processed_step
    quit_event = by_kind[1, 'quit']
    assert quit_event.applied_step is None


def test_viewer_requests_are_journaled_before_state_changes(tmp_path, monkeypatch):
    import bike_sim.sim.ride.physical_frontend as frontend
    clock = Clock()
    patch_window(monkeypatch, frontend, clock,
                 {3: [46], 5: [32], 7: [32], 9: [ord('Q')]})
    published = []
    driver = RecordingDriver(clock, published=published)
    run_viewer(driver, out_root=tmp_path, capture=_capture(published))
    capture, _ = published[0]
    events = list(capture.iter_events())
    # Strength while brakes are off: requested only, no applied record.
    strength = [e for e in events if e.kind == 'brake_strength']
    assert len(strength) == 1
    assert strength[0].values == {'record': 'requested', 'strength': .6}
    toggles = [e for e in events if e.kind == 'brake_toggle']
    # A canceled toggle keeps both requests, each followed by its applied
    # record — the journal shows the cancel instead of a net no-op.
    assert [t.values['braking'] for t in toggles] == [True, False]
    demands = [e for e in events if e.kind == 'brake_demand']
    assert [d.values['front_brake_demand'] for d in demands] == [.6, 0.]


def test_viewer_playback_scale_is_requested_only(tmp_path, monkeypatch):
    import bike_sim.sim.ride.physical_frontend as frontend
    clock = Clock()
    patch_window(monkeypatch, frontend, clock,
                 {4: [296], 6: [295], 8: [ord('Q')]})
    published = []
    driver = RecordingDriver(clock, published=published)
    run_viewer(driver, out_root=tmp_path, capture=_capture(published))
    events = list(published[0][0].iter_events())
    scales = [e for e in events if e.kind == 'playback_scale']
    assert len(scales) == 2
    assert all(e.applied_step is None for e in scales)
    # Playback never feeds physics: wall pacing stays out of applied_step.
    assert scales[0].values['record'] == 'requested'


def test_viewer_duplicate_terminal_notice_keeps_one_episode(tmp_path, monkeypatch):
    import bike_sim.sim.ride.physical_frontend as frontend
    clock = Clock()

    class Ended(RecordingDriver):
        def advance(self, target, **kwargs):
            super().advance(target, **kwargs)
            if self.step >= 6:
                self.reason = 'end_of_track'

    patch_window(monkeypatch, frontend, clock, {60: [ord('Q')]})
    published = []
    driver = Ended(clock, published=published)
    assert run_viewer(driver, out_root=tmp_path,
                      capture=_capture(published)) == 0
    # Terminal announcement sealed once; the post-loop seal deduped.
    assert driver.events == [('seal', 1, 0)]
    episodes = list(published[0][0].iter_episodes())
    assert len(episodes) == 1
    assert episodes[0].summary['outcome']['reason'] == 'end_of_track'


def test_viewer_ctrl_c_adds_no_control_and_still_finalizes(tmp_path, monkeypatch):
    import bike_sim.sim.ride.physical_frontend as frontend
    clock = Clock()
    patch_window(monkeypatch, frontend, clock, {40: [ord('Q')]})
    published = []
    driver = RecordingDriver(clock, published=published,
                             advance_failures={3: KeyboardInterrupt()})
    with pytest.raises(KeyboardInterrupt):
        run_viewer(driver, out_root=tmp_path, capture=_capture(published))
    # Ctrl-C is an operator stop, not a simulation failure: the sealed
    # episode and the driver's failure metadata say so.
    assert driver.seal_calls == [(1, 0, 'operator_stop')]
    assert driver.metadata.get('failure') is None
    episodes = list(published[0][0].iter_episodes())
    assert len(episodes) == 1
    assert episodes[0].summary['outcome']['reason'] == 'operator_stop'
    # No hidden control action after the interrupt: the journal holds only
    # episode_start plus seal-time derived facts for the committed prefix.
    assert driver.advance_calls == 3
    events = list(published[0][0].iter_events())
    assert all(e.kind in ('episode_start', 'derived_transition',
                          'model_first_failure') for e in events)
    assert driver.closed


def test_viewer_simulation_error_seals_prefix_and_reraises(tmp_path, monkeypatch):
    import bike_sim.sim.ride.physical_frontend as frontend
    clock = Clock()
    patch_window(monkeypatch, frontend, clock, {40: [ord('Q')]})
    published = []
    driver = RecordingDriver(clock, published=published,
                             advance_failures={2: RuntimeError('engine broke')})
    with pytest.raises(RuntimeError, match='engine broke'):
        run_viewer(driver, out_root=tmp_path, capture=_capture(published))
    assert driver.metadata['failure'] == 'RuntimeError: engine broke'
    episodes = list(published[0][0].iter_episodes())
    assert len(episodes) == 1
    assert episodes[0].summary['outcome']['reason'] == 'simulation_error'
    assert driver.closed


def test_viewer_second_ctrl_c_during_cleanup_seal_still_publishes(
        tmp_path, monkeypatch):
    """R04 regression for the live defect: a first Ctrl-C interrupts the
    pump, a SECOND arrives inside the cleanup seal. The deferred
    interrupt is journaled and the owned prefix still publishes."""
    import bike_sim.sim.ride.physical_frontend as frontend
    clock = Clock()
    patch_window(monkeypatch, frontend, clock, {40: [ord('Q')]})
    driver = StoringDriver(
        clock, advance_failures={3: KeyboardInterrupt()},
        episode_rows=[[row(0, contact=True, brake=.4),
                       row(1, contact=False, brake=.4)]],
        seal_hooks=[_second_sigint])
    with pytest.raises(KeyboardInterrupt):
        run_viewer(driver, out_root=tmp_path, capture=_recording())
    assert driver.seal_calls == [(1, 0, 'operator_stop')]
    manifests = list(tmp_path.rglob('manifest.json'))
    assert len(manifests) == 1
    assert (manifests[0].parent/'run.duckdb').is_file()
    from bike_sim.recording.reader import Recording
    with Recording(manifests[0].parent) as saved:
        assert saved.episode_ids == (0,)
        assert saved.episode()['physical_rows'] == 2
    # episode_start + the seal-time derived transition + the deferred
    # interrupt event, all on the sealed episode — no dangling events.
    assert _event_kinds(manifests[0].parent) == [
        'episode_start', 'derived_transition', 'operator_stop_deferred']
    assert driver.closed


def test_viewer_second_ctrl_c_during_cleanup_finalize_still_publishes(
        tmp_path, monkeypatch):
    """The deferred interrupt can also land inside ``finalize`` itself:
    too late to journal, so it rides the propagating error's notes —
    but the recording is already committed."""
    import bike_sim.sim.ride.physical_frontend as frontend
    clock = Clock()
    patch_window(monkeypatch, frontend, clock, {40: [ord('Q')]})
    driver = StoringDriver(clock, advance_failures={3: KeyboardInterrupt()})
    with pytest.raises(KeyboardInterrupt) as excinfo:
        run_viewer(driver, out_root=tmp_path,
                   capture=_recording(publisher=_interrupting_publisher()))
    assert driver.seal_calls == [(1, 0, 'operator_stop')]
    assert any('operator interrupt deferred' in note
               for note in getattr(excinfo.value, '__notes__', ()))
    manifests = list(tmp_path.rglob('manifest.json'))
    assert len(manifests) == 1
    assert (manifests[0].parent/'run.duckdb').is_file()
    from bike_sim.recording.reader import Recording
    with Recording(manifests[0].parent) as saved:
        assert saved.episode_ids == (0,)
    assert 'operator_stop_deferred' not in _event_kinds(manifests[0].parent)
    assert driver.closed


def test_viewer_ctrl_c_during_normal_finalize_publishes_then_stops(
        tmp_path, monkeypatch):
    """A Ctrl-C deferred on the NORMAL path: publication completes, the
    journal gets the deferred event when it lands pre-finalize, then the
    interrupt is honored — the run exits as an operator stop."""
    import bike_sim.sim.ride.physical_frontend as frontend
    clock = Clock()
    patch_window(monkeypatch, frontend, clock, {3: [ord('Q')]})
    driver = StoringDriver(clock, seal_hooks=[_second_sigint],
                           episode_rows=[[row(0, contact=True, brake=.4)]])
    with pytest.raises(KeyboardInterrupt):
        run_viewer(driver, out_root=tmp_path, capture=_recording())
    assert driver.seal_calls == [(1, 0, 'viewer_closed')]
    manifests = list(tmp_path.rglob('manifest.json'))
    assert len(manifests) == 1
    from bike_sim.recording.reader import Recording
    with Recording(manifests[0].parent) as saved:
        assert saved.episode_ids == (0,)
        assert 'operator_stop_deferred' in _event_kinds(manifests[0].parent)
    assert driver.closed


def test_viewer_ctrl_c_during_normal_finalize_is_noted_on_stop(
        tmp_path, monkeypatch):
    """Deferred inside ``finalize`` on the normal path: nothing more can
    be journaled, so the re-raised interrupt carries the note."""
    import bike_sim.sim.ride.physical_frontend as frontend
    clock = Clock()
    patch_window(monkeypatch, frontend, clock, {3: [ord('Q')]})
    driver = StoringDriver(clock)
    with pytest.raises(KeyboardInterrupt) as excinfo:
        run_viewer(driver, out_root=tmp_path,
                   capture=_recording(publisher=_interrupting_publisher()))
    assert driver.seal_calls == [(1, 0, 'viewer_closed')]
    assert any('operator interrupt deferred' in note
               for note in getattr(excinfo.value, '__notes__', ()))
    manifests = list(tmp_path.rglob('manifest.json'))
    assert len(manifests) == 1
    assert driver.closed


def test_viewer_strict_seal_rejection_retries_once(tmp_path, monkeypatch):
    import bike_sim.sim.ride.physical_frontend as frontend
    clock = Clock()
    patch_window(monkeypatch, frontend, clock, {3: [ord('Q')]})
    published = []
    driver = RecordingDriver(
        clock, published=published,
        seal_failures=[InvalidReferenceRun('tail violations')])
    assert run_viewer(driver, out_root=tmp_path,
                      capture=_capture(published)) == 2
    assert driver.seal_calls == [(1, 0, 'viewer_closed'),
                                 (1, 0, 'invalid_controller')]
    assert driver.metadata['tail_failure'] == 'tail violations'
    episodes = list(published[0][0].iter_episodes())
    assert episodes[0].summary['outcome']['reason'] == 'invalid_controller'


def test_viewer_python_backend_record_rejected_before_window(tmp_path, monkeypatch):
    import bike_sim.sim.ride.physical_frontend as frontend
    clock = Clock()
    patch_window(monkeypatch, frontend, clock, {})
    driver = RecordingDriver(clock)
    driver.backend = 'python'
    with pytest.raises(ValueError, match='native backend'):
        run_viewer(driver, out_root=tmp_path, record=True)
    assert driver.closed


def test_derived_transitions_carry_rule_version_and_interval():
    episode = FakeEpisode(1, [
        row(0, contact=True, brake=0.),
        row(1, contact=True, brake=.4),
        row(2, contact=False, brake=.4),
    ])
    events = list(iter_derived_events(episode, 0, start_event_id=3,
                                      received_wall_s=1.5))
    assert [e.event_id for e in events] == [3, 4]
    first, second = events
    assert first.kind == 'derived_transition'
    assert first.values['classification'] == 'derived_transition'
    assert first.values['derivation_rule'] == DERIVED_TRANSITION_RULE
    assert first.values['derived_kind'] == 'brake_gate'
    assert first.values['interval_id'] == 1
    assert first.processed_step == first.applied_step == 1
    assert first.values['changes'] == {'front_brake_demand': .4,
                                       'rear_brake_demand': .4}
    assert second.values['derived_kind'] == 'tire_contact'
    assert second.values['changes'] == {'front.raw_contact': False,
                                        'rear.raw_contact': False}
    assert second.received_wall_s == 1.5


def test_derived_transitions_cover_shift_coast_and_limiter_gates():
    """Every tracked drive gate family emits a labeled derived_transition."""
    def drive_row(interval_id, **fields):
        return SimpleNamespace(
            interval_id=interval_id, time_s=interval_id*.001,
            end_time_s=(interval_id+1)*.001,
            channels={'drive': fields})
    episode = FakeEpisode(1, [
        drive_row(0, shift_active=False, shift_direction='none',
                  freehub_engaged=True, coasting_reason=None,
                  safety_limited=False, energy_limited=False,
                  motor_enabled=True, battery_empty=False),
        drive_row(1, shift_active=True, shift_direction='down',
                  freehub_engaged=False, coasting_reason='shift',
                  safety_limited=False, energy_limited=False,
                  motor_enabled=True, battery_empty=False),
        drive_row(2, shift_active=False, shift_direction='none',
                  freehub_engaged=False, coasting_reason='coast',
                  safety_limited=True, energy_limited=False,
                  motor_enabled=False, battery_empty=False),
        drive_row(3, shift_active=False, shift_direction='none',
                  freehub_engaged=True, coasting_reason=None,
                  safety_limited=False, energy_limited=True,
                  motor_enabled=False, battery_empty=True),
    ])
    events = list(iter_derived_events(episode, 0, start_event_id=0,
                                      received_wall_s=0.))
    by_kind = {}
    for event in events:
        by_kind.setdefault(event.values['derived_kind'], []).append(event)
    assert set(by_kind) == {'shift', 'freehub_coast', 'drive_limits'}
    # Interval 1: the shift gate flips both tracked fields in one event.
    assert [e.values['interval_id'] for e in by_kind['shift']] == [1, 2]
    assert by_kind['shift'][0].values['changes'] == {
        'shift_active': True, 'shift_direction': 'down'}
    assert by_kind['shift'][1].values['changes'] == {
        'shift_active': False, 'shift_direction': 'none'}
    coast = by_kind['freehub_coast']
    assert [e.values['interval_id'] for e in coast] == [1, 2, 3]
    assert coast[0].values['changes'] == {'coasting_reason': 'shift',
                                          'freehub_engaged': False}
    assert coast[1].values['changes'] == {'coasting_reason': 'coast'}
    assert coast[2].values['changes'] == {'coasting_reason': None,
                                          'freehub_engaged': True}
    limits = by_kind['drive_limits']
    assert [e.values['interval_id'] for e in limits] == [2, 3]
    assert limits[0].values['changes'] == {'safety_limited': True,
                                           'motor_enabled': False}
    assert limits[1].values['changes'] == {'safety_limited': False,
                                           'energy_limited': True,
                                           'battery_empty': True}
    assert all(e.values['classification'] == 'derived_transition'
               for e in events)
    assert all(e.values['derivation_rule'] == DERIVED_TRANSITION_RULE
               for e in events)
    assert all(e.processed_step == e.applied_step for e in events)


def test_derived_first_model_failure_stays_a_derived_transition():
    def violation_row(interval_id, first):
        return SimpleNamespace(
            interval_id=interval_id, time_s=interval_id*.001,
            end_time_s=(interval_id+1)*.001,
            channels={'model_status': {'model_valid': first is None,
                                       'first_model_violation': first}})
    episode = FakeEpisode(1, [violation_row(0, None),
                              violation_row(1, {'time_s': .001,
                                                'reasons': ['joint.limit']}),
                              violation_row(2, {'time_s': .001,
                                                'reasons': ['joint.limit']})])
    events = list(iter_derived_events(episode, 0, start_event_id=0,
                                      received_wall_s=0.))
    assert [e.kind for e in events] == ['derived_transition',
                                      'derived_transition']
    assert events[0].values['derived_kind'] == 'model_validity'
    assert events[0].values['changes'] == {'model_valid': False}
    # The sticky model-status violation is a channel-derived fact; the
    # monitor's measured first_failure stays a distinct event kind.
    assert events[1].values['derived_kind'] == 'model_first_failure'


def test_derived_missing_reason_instrumentation_emits_nothing_source():
    """Absent channel instrumentation is not a transition — and a crash
    cause that does appear is a derived fact, never a source event."""
    def crash_row(interval_id, *, cause_present, cause):
        channels = {}
        if cause_present:
            channels['contact_crash_cause'] = cause
        return SimpleNamespace(
            interval_id=interval_id, time_s=interval_id*.001,
            end_time_s=(interval_id+1)*.001, channels=channels)
    # The root never appears: no error, no inferred event.
    silent = FakeEpisode(1, [crash_row(i, cause_present=False, cause=None)
                             for i in range(4)])
    assert list(iter_derived_events(silent, 0, start_event_id=0,
                                    received_wall_s=0.)) == []
    # When the instrumentation does appear, the fact is labeled derived.
    episode = FakeEpisode(1, [crash_row(0, cause_present=True, cause=None),
                              crash_row(1, cause_present=True,
                                        cause={'kind': 'tire_blowout'})])
    events = list(iter_derived_events(episode, 0, start_event_id=0,
                                      received_wall_s=0.))
    assert [e.kind for e in events] == ['derived_transition']
    assert events[0].values['derived_kind'] == 'contact_crash'
    assert events[0].values['classification'] == 'derived_transition'


def test_episode_interval_for_time_maps_failure_timestamp():
    episode = FakeEpisode(1, [row(i) for i in range(5)])
    assert episode_interval_for_time(episode, 0.) == 0
    assert episode_interval_for_time(episode, .003) == 2
    assert episode_interval_for_time(episode, 99.) == 4
    assert episode_interval_for_time(FakeEpisode(1), .1) == 0


def test_capture_retries_publication_with_evidence_intact(tmp_path):
    calls = []
    def flaky(capture, out_root):
        calls.append(1)
        if len(calls) == 1:
            raise OSError('disk busy')
        return out_root/'recording.marker'
    capture = _capture([], publisher=flaky)
    driver = SimpleNamespace(backend='native')
    # Hand the capture straight to the journal/emission layer to keep this a
    # pure capture-contract check.
    from bike_sim.recording.capture import EpisodeJournal
    journal = EpisodeJournal(capture, 0)
    journal.emit('episode_start', received_wall_s=0., processed_step=0)
    with pytest.raises(OSError, match='disk busy'):
        capture.finalize(tmp_path)
    assert capture.state == 'publication_failed'
    path = capture.finalize(tmp_path)
    assert path == tmp_path/'recording.marker'
    assert capture.state == 'published'
    assert len(calls) == 2
    assert [e.kind for e in capture.iter_events()] == ['episode_start']


def test_headless_rejects_python_record_before_any_setup(tmp_path, monkeypatch):
    import bike_sim.sim.ride.physical_frontend as frontend
    def forbidden(*args, **kwargs):
        raise AssertionError('driver construction ran for python --record')
    monkeypatch.setattr(frontend, 'build_physical_driver', forbidden)
    args = SimpleNamespace(record=True, backend='python')
    with pytest.raises(ValueError, match='native backend'):
        frontend.run_headless(None, args, 0, None)


def _headless_driver(clock=None, cls=RecordingDriver, **driver_kwargs):
    """A duck-typed driver for the headless frontend contract."""
    clock = Clock() if clock is None else clock

    class Headless(cls):
        def __init__(self):
            super().__init__(clock, **driver_kwargs)
            self.recorder = SimpleNamespace(rows=0, columns=lambda: {})
        def max_steps(self):
            return 6
        def advance(self, target, **kwargs):
            self.advance_calls += 1
            error = self._advance_failures.get(self.advance_calls)
            if error is not None:
                raise error
            self.step = min(int(target), self.step+3)
            self.time_s = self.step*self.dt_s
            if self.step >= 6:
                self.reason = 'duration_reached'
    driver = Headless()
    driver.max_steps = 6
    return driver


def _headless_args(tmp_path, *, record=False, backend='native'):
    return SimpleNamespace(record=record, backend=backend, out=tmp_path,
                           decimate=1, duration=.01, no_plots=True,
                           resolved_check_policy='strict', headless=True)


def test_headless_records_seals_and_finalizes_once(tmp_path, monkeypatch):
    import bike_sim.sim.ride.physical_frontend as frontend
    driver = _headless_driver()
    monkeypatch.setattr(frontend, 'build_physical_driver',
                        lambda *args, **kwargs: driver)
    published = []
    args = _headless_args(tmp_path, record=True)
    code = frontend.run_headless(
        SimpleNamespace(name='fake'), args, 0, None,
        capture=_capture(published, mode='headless'))
    assert code == 0
    assert driver.events == [('seal', 1, 0)]
    assert len(published) == 1
    capture, _ = published[0]
    episodes = list(capture.iter_episodes())
    assert [e.episode_id for e in episodes] == [0]
    assert episodes[0].summary['outcome']['reason'] == 'duration_reached'
    assert kinds(capture.iter_events(), 0)[:1] == ['episode_start']


def test_headless_zero_step_failure_keeps_owned_episode(tmp_path, monkeypatch):
    """An initialization-time failure seals its committed prefix (R03/R16):
    owned evidence exists and publishes even with zero advanced steps."""
    import bike_sim.sim.ride.physical_frontend as frontend
    driver = _headless_driver()
    monkeypatch.setattr(frontend, 'build_physical_driver',
                        lambda *args, **kwargs: driver)
    def advance(target, **kwargs):
        driver.advance_calls += 1
        raise RuntimeError('initialization blew up')
    driver.advance = advance
    published = []
    args = _headless_args(tmp_path, record=True)
    code = frontend.run_headless(
        SimpleNamespace(name='fake'), args, 0, None,
        capture=_capture(published, mode='headless', strict=True))
    assert code == 1
    assert driver.metadata['failure'] == 'RuntimeError: initialization blew up'
    episodes = list(published[0][0].iter_episodes())
    assert len(episodes) == 1
    assert episodes[0].episode_id == 0
    assert episodes[0].summary['outcome']['reason'] == 'simulation_error'
    assert driver.closed


@pytest.mark.parametrize('record', [False, True])
def test_headless_trailer_names_only_existing_artifacts(
        tmp_path, monkeypatch, capsys, record):
    """Off-record the run dir is never created, so the trailer says so;
    on-record it names the actual sealed path the publisher returned."""
    import bike_sim.sim.ride.physical_frontend as frontend
    driver = _headless_driver()
    monkeypatch.setattr(frontend, 'build_physical_driver',
                        lambda *args, **kwargs: driver)
    published = []
    capture = _capture(published, mode='headless') if record else None
    code = frontend.run_headless(
        SimpleNamespace(name='fake'), _headless_args(tmp_path, record=record),
        0, None, capture=capture)
    assert code == 0
    line = capsys.readouterr().out.strip().splitlines()[-1]
    if record:
        marker = str(published[0][1]/'recording.marker')
        assert line.endswith(f'-> {marker}')
        assert 'no files' not in line
    else:
        assert 'recording off (no files)' in line
        assert '->' not in line


def test_headless_off_writes_no_artifact_files(native_module, tmp_path):
    """Without --record a real native headless launch writes NOTHING under
    --out: no telemetry.csv, intervals.jsonl or preview artifacts — not
    even empty ones."""
    from bike_sim.cli import ride
    from _native_runtime_support import PHYSICS, TRACK
    args = ride.parse_args([
        '--physics-config', str(PHYSICS), '--track', str(TRACK),
        '--backend', 'native', '--check-policy', 'diagnostic',
        '--headless', '--duration', '0.01', '--no-plots',
        '--out', str(tmp_path)])
    track = ride.resolve_track(args.track, seed=args.seed,
                               length_m=args.length)
    rider = ride.resolve_rider(args)
    code = run_headless(track, args, args.seed, rider)
    # The rough recipe still reports its sticky model violation; the check
    # outcome is honest even though no artifact was requested or written.
    assert code == 2
    assert list(tmp_path.rglob('*')) == []


def test_explicit_programmatic_export_still_writes_artifacts(
        environment_factory, native_module, tmp_path):
    """driver.export() is the explicit opt-in outside the frontend (R02):
    a legacy-retention caller still gets the telemetry/intervals/summary
    files it asked for."""
    from bike_sim.sim.ride.physical_driver import PhysicalRideDriver
    driver = PhysicalRideDriver(environment_factory().sim, backend='native',
                                duration_s=.02, record=None)
    try:
        driver.advance(16)
        driver.flush()
        out = tmp_path/'explicit'
        summary = driver.export(out, reason='viewer_closed')
        assert (out/'telemetry.csv').exists()
        assert (out/'intervals.jsonl').exists()
        assert (out/'summary.json').exists()
        assert summary['outcome']['reason'] == 'viewer_closed'
        # Legacy retention keeps the decimated projection — real rows,
        # not a header-only artifact.
        rows = (out/'telemetry.csv').read_text().strip().splitlines()
        assert len(rows) > 1
    finally:
        driver.close()


def test_headless_success_with_failed_publication_is_nonzero(tmp_path, monkeypatch):
    """A clean recorded run whose finalize fails still exits nonzero."""
    import bike_sim.sim.ride.physical_frontend as frontend
    driver = _headless_driver()
    monkeypatch.setattr(frontend, 'build_physical_driver',
                        lambda *args, **kwargs: driver)
    def boom(capture, out_root):
        raise OSError('publish failed')
    args = _headless_args(tmp_path, record=True)
    with pytest.raises(OSError, match='publish failed'):
        frontend.run_headless(
            SimpleNamespace(name='fake'), args, 0, None,
            capture=_capture([], mode='headless', publisher=boom))
    assert driver.metadata.get('failure') is None
    assert driver.closed


def test_headless_publish_failure_preserves_primary_and_evidence(tmp_path, monkeypatch):
    """A failing publisher propagates with the primary sim failure noted;
    the owned episode stays in the failed capture for retry (R04/R16)."""
    import bike_sim.sim.ride.physical_frontend as frontend
    driver = _headless_driver()
    monkeypatch.setattr(frontend, 'build_physical_driver',
                        lambda *args, **kwargs: driver)
    def advance(target, **kwargs):
        driver.advance_calls += 1
        raise RuntimeError('primary boom')
    driver.advance = advance
    def boom(capture, out_root):
        raise OSError('publish failed')
    capture = _capture([], mode='headless', publisher=boom)
    args = _headless_args(tmp_path, record=True)
    with pytest.raises(OSError, match='publish failed') as failure:
        frontend.run_headless(SimpleNamespace(name='fake'), args, 0, None,
                              capture=capture)
    assert driver.metadata['failure'] == 'RuntimeError: primary boom'
    assert any('primary boom' in note for note in failure.value.__notes__)
    assert capture.state == 'publication_failed'
    assert len(list(capture.iter_episodes())) == 1
    assert driver.closed


def test_headless_second_ctrl_c_during_seal_journals_and_still_publishes(
        tmp_path, monkeypatch):
    """Headless R04: a Ctrl-C deferred inside the shielded seal is
    journaled, the owned prefix publishes, then the interrupt is
    honored — the run exits as an operator stop."""
    import bike_sim.sim.ride.physical_frontend as frontend
    driver = _headless_driver(cls=StoringDriver,
                              episode_rows=[[row(0, contact=True, brake=.4),
                                             row(1, contact=False, brake=.4)]],
                              seal_hooks=[_second_sigint])
    monkeypatch.setattr(frontend, 'build_physical_driver',
                        lambda *args, **kwargs: driver)
    with pytest.raises(KeyboardInterrupt):
        frontend.run_headless(
            SimpleNamespace(name='fake'), _headless_args(tmp_path, record=True),
            0, None, capture=_recording(mode='headless'))
    assert driver.events == [('seal', 1, 0)]
    manifests = list(tmp_path.rglob('manifest.json'))
    assert len(manifests) == 1
    assert (manifests[0].parent/'run.duckdb').is_file()
    from bike_sim.recording.reader import Recording
    with Recording(manifests[0].parent) as saved:
        assert saved.episode_ids == (0,)
    assert _event_kinds(manifests[0].parent) == [
        'episode_start', 'derived_transition', 'operator_stop_deferred']
    assert driver.closed


def test_headless_second_ctrl_c_during_finalize_is_noted_and_publishes(
        tmp_path, monkeypatch):
    """A Ctrl-C deferred inside ``finalize`` lands too late to journal;
    the re-raised interrupt carries the note and the manifest exists."""
    import bike_sim.sim.ride.physical_frontend as frontend
    driver = _headless_driver(cls=StoringDriver)
    monkeypatch.setattr(frontend, 'build_physical_driver',
                        lambda *args, **kwargs: driver)
    with pytest.raises(KeyboardInterrupt) as excinfo:
        frontend.run_headless(
            SimpleNamespace(name='fake'), _headless_args(tmp_path, record=True),
            0, None,
            capture=_recording(mode='headless',
                               publisher=_interrupting_publisher()))
    assert any('operator interrupt deferred' in note
               for note in getattr(excinfo.value, '__notes__', ()))
    manifests = list(tmp_path.rglob('manifest.json'))
    assert len(manifests) == 1
    assert 'operator_stop_deferred' not in _event_kinds(manifests[0].parent)
    assert driver.closed


def test_make_run_capture_rejects_non_native_recording():
    driver = SimpleNamespace(backend='python', strict=False)
    with pytest.raises(ValueError, match='native backend'):
        make_run_capture(driver, enabled=True, mode='viewer')
    capture = make_run_capture(driver, enabled=False, mode='viewer')
    assert capture.enabled is False
    assert capture.metadata.values['check_policy'] == 'diagnostic'
    assert capture.metadata.backend == 'python'


# --- Real native driver adapter checks (separate from the fake pump) ---


def test_headless_native_run_records_one_episode(native_module, tmp_path):
    """A real native --record headless run seals once and publishes at end."""
    from bike_sim.cli import ride
    from _native_runtime_support import PHYSICS, TRACK
    args = ride.parse_args([
        '--physics-config', str(PHYSICS), '--track', str(TRACK),
        '--backend', 'native', '--record', '--check-policy', 'diagnostic',
        '--headless', '--duration', '0.01', '--no-plots',
        '--out', str(tmp_path)])
    track = ride.resolve_track(args.track, seed=args.seed,
                               length_m=args.length)
    rider = ride.resolve_rider(args)
    published = []
    code = run_headless(track, args, args.seed, rider,
                        capture=_capture(published, mode='headless'))
    # The rough recipe trips a real model-check violation; diagnostic mode
    # keeps the sticky first failure, and the exit code reports it.
    assert code == 2
    assert len(published) == 1
    capture, _root = published[0]
    episodes = list(capture.iter_episodes())
    assert len(episodes) == 1
    assert episodes[0].physical.row_count > 0
    assert episodes[0].summary['outcome']['reason'] == 'duration_reached'
    failure_time, violations = episodes[0].summary['first_failure']
    assert 'energy.constraint_work' in violations
    events = list(capture.iter_events())
    assert events[0].kind == 'episode_start'
    first_failures = [e for e in events if e.kind == 'model_first_failure']
    assert len(first_failures) == 1
    assert first_failures[0].values['classification'] == 'measured'
    assert first_failures[0].values['time_s'] == pytest.approx(failure_time)
    # Typed tables decode through the bound catalog for the real episode.
    tables = {batch.table for batch in episodes[0].physical.tables(batch_rows=64)}
    assert 'channel_tires' in tables and 'channel_control' in tables
    metadata = capture.metadata
    assert metadata.backend == 'native' and metadata.mode == 'headless'
    assert metadata.values['check_policy'] == 'diagnostic'


def test_viewer_native_pump_seals_reset_generations(
        environment_factory, native_module, tmp_path, monkeypatch):
    """A real native driver under the fake window seals each generation."""
    import bike_sim.sim.ride.physical_frontend as frontend
    from bike_sim.sim.ride.physical_driver import PhysicalRideDriver
    driver = PhysicalRideDriver(environment_factory().sim, backend='native',
                                duration_s=.02, record=True)
    clock = Clock()
    patch_window(monkeypatch, frontend, clock,
                 {3: [32], 6: [ord('R')], 12: [ord('Q')]})
    published = []
    try:
        code = run_viewer(driver, out_root=tmp_path,
                          capture=_capture(published))
        # The environment recipe reports a sticky model violation, so the
        # run exits with the check-policy code after publishing its
        # evidence.
        assert code == 2
        capture, _root = published[0]
        episodes = list(capture.iter_episodes())
        assert [e.episode_id for e in episodes] == [0, 1]
        assert [e.generation for e in episodes] == [1, 2]
        events = list(capture.iter_events())
        ep0, ep1 = kinds(events, 0), kinds(events, 1)
        assert ep0[:2] == ['episode_start', 'brake_toggle']
        assert 'reset' in ep0
        # Seal-time facts (the measured first failure and channel-derived
        # transitions) are journaled after the quit request.
        assert ep1[:2] == ['episode_start', 'quit']
        assert all(kind in ('episode_start', 'quit', 'model_first_failure',
                            'derived_transition') for kind in ep1)
        assert {e.values['record'] for e in events
                if e.kind == 'brake_demand'} == {'applied'}
        # Derived transitions re-read the real sealed channel rows; the
        # sticky model-status violation is the transition this recipe
        # actually produces.
        derived = [e for e in events if e.kind == 'derived_transition']
        assert derived
        assert all(e.values.get('derivation_rule') == DERIVED_TRANSITION_RULE
                   for e in derived)
        assert any(e.values.get('derived_kind') == 'model_first_failure'
                   for e in derived)
        assert any(e.values.get('derived_kind') == 'model_validity'
                   for e in derived)
    finally:
        driver.close()
