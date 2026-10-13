"""Final-only launch ownership shared by both research entrypoints.

No physics work is performed by sealing. A requested stop drains only an
already accepted external window; it never asks a policy for a new command.
"""
from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
import sys
import time
from uuid import uuid4

from bike_sim.recording.capture import EpisodeJournal, RunCapture
from bike_sim.recording.contracts import InputEvent, RunMetadata
from bike_sim.recording.interrupt import InterruptShield
from bike_sim.recording.native import iter_derived_events


def drain_active(env, session=None):
    from bike_sim.sim.playback import COMPUTE_SLICE_S
    if session is not None:
        session.stop()
        while session.pending:
            session.advance_pending(wall_budget_s=COMPUTE_SLICE_S)
    else:
        env.stop()
        while env.control_pending:
            env.advance_control(wall_budget_s=COMPUTE_SLICE_S)


def close_environment(env, primary=None):
    try:
        env.close(discard_pending=True)
    except BaseException as error:
        if primary is None:
            raise
        primary.add_note(f'research close failed: {type(error).__name__}: {error}')


class ResearchRun:
    def __init__(self, env, out_root, *, enabled, mode, capture=None, invocation=None):
        if enabled and env.capture_mode != 'full':
            raise ValueError('persistent research recording requires full capture before setup')
        self.env, self.session, self.out_root = env, None, Path(out_root)
        self.capture = capture if capture is not None else RunCapture(RunMetadata(
            uuid4(), datetime.now(timezone.utc), env.backend, mode,
            dict(invocation=list(sys.argv if invocation is None else invocation),
                 monotonic_origin_s=time.monotonic(), setup=env.metadata, execution=env.execution,
                 check_policy='strict' if env.config.stop_on_model_violation else 'diagnostic')),
            enabled=bool(enabled))
        if self.capture.enabled != bool(enabled):
            raise ValueError('injected capture and launch recording policy disagree')
        self.episode_id = 0
        self.journal = EpisodeJournal(self.capture, self.episode_id)
        self._evidence = None
        self._seal_events = None
        self.finalization_attempted = False
        self.path = None
        # The RAM owner is available for an explicit retry after publication
        # failure, even after the engine and its temporary model are closed.
        env.recording_run = self
        self.event('episode_started', values={'capture_mode': env.capture_mode,
                                             'check_policy': 'strict' if env.config.stop_on_model_violation else 'diagnostic'})

    def attach(self, session):
        self.session = session
        session._recording_event = self.event
        session._after_recording_reset = self._new_episode

    def event(self, kind, *, received_wall_s=None, applied=False, values=None):
        now = time.monotonic()
        return self.journal.emit(kind, received_wall_s=now if received_wall_s is None else received_wall_s,
            processed_step=int(self.env.sim.steps),
            applied_step=int(self.env.sim.steps) if applied else None,
            values=dict(time_s=float(self.env.sim.time_s), **(values or {})))

    def _new_episode(self):
        self.episode_id += 1
        self.journal = EpisodeJournal(self.capture, self.episode_id)
        self._evidence = self._seal_events = None
        self.event('episode_started', values={'capture_mode': self.env.capture_mode, 'reset': True})

    def reset(self, *, seed=None):
        if self.env.control_pending:
            raise RuntimeError('drain the accepted control window before resetting a recorded episode')
        self.seal(reason=self.env.reason or 'reset')
        if self.session is not None:
            # The hook runs after successful environment reset but before
            # policy.reset, so a policy setup error belongs to the new episode.
            return self.session.reset(seed=seed)
        result = self.env.reset(seed=seed)
        self._new_episode()
        return result

    def seal(self, *, reason):
        if not self.capture.enabled:
            return None
        if self._evidence is None:
            owner = self.session if self.session is not None else self.env
            self._evidence = owner.seal_episode(self.episode_id, reason=reason)
        self.capture.add_episode(self._evidence)
        if self._seal_events is None:
            now = time.monotonic()
            first = self.journal.next_id
            events = list(iter_derived_events(self._evidence.physical.episode,
                episode_id=self.episode_id, start_event_id=first, received_wall_s=now))
            failure = self._evidence.summary.get('first_failure')
            if failure is not None:
                events.append(InputEvent(self.episode_id, first + len(events), 'model_first_failure', now,
                    int(self.env.sim.steps), None, {'evidence': failure, 'classification': 'measured'}))
            events.append(InputEvent(self.episode_id, first + len(events), 'episode_ended', now,
                int(self.env.sim.steps), None,
                {'reason': reason, 'time_s': float(self.env.sim.time_s), 'processed_wall_s': now}))
            self._seal_events = tuple(events)
        for event in self._seal_events:
            if event.event_id >= self.journal.next_id:
                self.journal.emit_all((event,))
        return self._evidence

    def _journal_deferred(self, shield):
        """Journal a deferred operator interrupt while events are accepted.

        Runs after the episode commits and before publication, so the
        event stays inside a sealed episode's contiguous identity. A
        journal or capture rejection is best-effort — observability never
        breaks the shielded section. Returns True when journaled.
        """
        if not shield.deferred or not self.capture.enabled:
            return False
        try:
            self.event('operator_stop_deferred',
                       values={'deferred_during': 'publication'})
        except Exception:
            return False
        return True

    def finish(self, *, primary=None, stop=False):
        if self.finalization_attempted:
            # Do not turn a failed publication into an automatic second attempt.
            # An explicit caller may retry capture.finalize(out_root).
            return self.path
        self.finalization_attempted = True
        cleanup_primary = None
        if stop:
            try:
                drain_active(self.env, self.session)
            except BaseException as error:
                if primary is not None:
                    primary.add_note(f'active control drain failed: {type(error).__name__}: {error}')
                else:
                    cleanup_primary = error
        cause = primary if primary is not None else cleanup_primary
        # A frontend failure can happen between completed control windows,
        # before the environment has an error of its own. Keep the evidence
        # analyzable, but do not advertise such a launch as checked replay.
        if cause is not None and not isinstance(cause, KeyboardInterrupt):
            self.env.run_metadata['launch_failure'] = {
                'error_type': type(cause).__name__, 'error': str(cause)}
        # Seal+finalize is one shielded section (R04): a second Ctrl-C is
        # deferred behind the write instead of stranding journaled events
        # against an unsealed episode. A deferred interrupt is journaled
        # while events are still accepted, then re-raised where the
        # launch would otherwise continue — and rides the notes of an
        # in-flight error that is already propagating.
        shield = InterruptShield()
        journaled = False
        try:
            with shield:
                if cause is not None:
                    self.event('launch_interrupted' if isinstance(cause, KeyboardInterrupt) else 'launch_failed',
                               values={'error_type': type(cause).__name__, 'error': str(cause),
                                       'cleanup_notes': list(getattr(cause, '__notes__', ()))})
                reason = self.env.reason or ('operator_stop' if isinstance(cause, KeyboardInterrupt)
                                             else 'simulation_error' if cause is not None else 'viewer_closed')
                self.seal(reason=reason)
                journaled = self._journal_deferred(shield)
                self.path = self.capture.finalize(self.out_root)
        except BaseException as error:
            if shield.deferred and not journaled:
                error.add_note('operator interrupt deferred during recording publication')
            if cause is not None:
                cause.add_note(f'recording finalization failed: {type(error).__name__}: {error}')
                raise cause from error
            raise
        if shield.deferred:
            propagating = cleanup_primary if cleanup_primary is not None else cause
            if propagating is not None:
                # Already unwinding: the deferral rides the notes instead
                # of replacing the exception that owns the traceback.
                if not journaled:
                    propagating.add_note('operator interrupt deferred during recording publication')
            else:
                stop = KeyboardInterrupt()
                if not journaled:
                    stop.add_note('operator interrupt deferred during recording publication')
                raise stop
        if cleanup_primary is not None:
            raise cleanup_primary
        return self.path
