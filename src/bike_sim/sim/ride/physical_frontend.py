"""Physical CLI frontends: backend-neutral owner, playback clock and the
launch-level run-recording lifecycle (R03, R11, R14, R16).

Both frontends own one ``RunCapture`` across reset-separated episodes:
episodes are sealed into RAM as they close (``driver.seal_episode``),
input requests are journaled in receipt order with received/processed
timestamps, and publication happens exactly once at launch end through
``RunCapture.finalize`` — never from advance, reset or render. Preview
CSV and recording-derived plots are produced only when recording is on;
a non-recording launch leaves no persistent side effects.
"""
from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
from queue import Empty, SimpleQueue
import time
from uuid import uuid4

from bike_sim.recording.capture import EpisodeJournal, RunCapture
from bike_sim.recording.contracts import RunMetadata
from bike_sim.recording.native import iter_derived_events
from bike_sim.sim.playback import PlaybackClock, COMPUTE_SLICE_S, RENDER_INTERVAL_S, speed_key
from bike_sim.sim.ride.physical_driver import build_physical_driver
from bike_sim.sim.ride.physical_session import PhysicalLiveCsv, physical_run_dir_name
from bike_sim.sim.ride.physical_view import present_view
from bike_sim.sim.ride.presentation import AchievedRate, FramePresenter
from bike_sim.sim.ride.reference_monitor import InvalidReferenceRun


def physical_exit_code(reason, summary):
    if reason == 'operator_stop':
        return 130
    if reason == 'simulation_error':
        return 1
    if reason == 'invalid_controller' or not summary['model_status']['model_valid']:
        return 2
    return 0 if reason in ('duration_reached', 'end_of_track', 'viewer_closed') else 1


def run_metadata(driver, *, mode, values=None):
    """Launch-level metadata; the check policy resolved before stepping."""
    return RunMetadata(
        run_id=uuid4(), started_at=datetime.now(timezone.utc),
        backend=driver.backend, mode=mode,
        values={'check_policy': 'strict' if getattr(driver, 'strict', False)
                                else 'diagnostic',
                **dict(values or {})})


def make_run_capture(driver, *, enabled, mode, publisher=None, values=None):
    """Resolve the launch-level ``RunCapture`` before runtime stepping.

    An enabled capture requires the native backend: the ordinary Python
    path has no recording adapter. The CLI rejects that combination
    earlier; this guard keeps programmatic callers honest too.
    """
    if enabled and driver.backend != 'native':
        raise ValueError(
            'run recording requires the native backend; the ordinary '
            'physical Python backend has no recording adapter')
    return RunCapture(
        run_metadata(driver, mode=mode, values=values),
        enabled=bool(enabled), _publisher=publisher)


def _check_tail_failure(driver, reason):
    """Classify a trailing-flush rejection without losing committed rows."""
    try:
        driver.flush()
    except InvalidReferenceRun as error:
        driver.metadata['tail_failure'] = str(error)
        return 'simulation_error' if reason == 'simulation_error' \
            else 'invalid_controller'
    except (ValueError, RuntimeError, ArithmeticError) as error:
        driver.metadata['tail_failure'] = f'{type(error).__name__}: {error}'
        return 'simulation_error'
    return reason


def _failure_fields(failure):
    """(time_s, violations) from either wire shape of first_failure."""
    if isinstance(failure, dict):
        return failure.get('time_s'), failure.get('violations')
    if isinstance(failure, (tuple, list)) and len(failure) == 2:
        return failure[0], failure[1]
    return None, None


def _seal_episode(driver, capture, journal, *, reason, sealed, counter):
    """Seal the active generation once and commit its evidence (R03).

    Returns the episode summary, or ``None`` on a duplicate terminal
    notification for an already-sealed generation. A strict sealing
    rejection commits its owned prefix and sticky first failure, adjusts
    the reason and retries once. Source facts (the monitor's first
    failure) precede the derived transitions re-read from sealed channels.
    """
    from bike_sim.recording.native import episode_interval_for_time
    frame = driver.snapshot()
    if frame.generation in sealed:
        # Duplicate terminal notification: the episode stays singular.
        return None
    episode_id = counter[0]
    try:
        evidence = driver.seal_episode(episode_id, reason=reason)
    except InvalidReferenceRun as error:
        driver.metadata['tail_failure'] = str(error)
        if reason != 'simulation_error':
            reason = 'invalid_controller'
        evidence = driver.seal_episode(episode_id, reason=reason)
    if journal is not None and journal.episode_id == episode_id:
        failure = _failure_fields(evidence.summary.get('first_failure'))
        if failure[1]:
            step = (episode_interval_for_time(evidence.physical.episode,
                                              failure[0])
                    if failure[0] is not None else driver.step)
            journal.emit('model_first_failure',
                         received_wall_s=time.monotonic(),
                         processed_step=int(step),
                         applied_step=None,
                         values={'classification': 'measured',
                                 'time_s': failure[0],
                                 'violations': list(failure[1])})
        journal.emit_all(iter_derived_events(
            evidence.physical.episode, episode_id,
            start_event_id=journal.next_id,
            received_wall_s=time.monotonic()))
    capture.add_episode(evidence)
    sealed.add(frame.generation)
    counter[0] += 1
    return evidence.summary


def _new_journal(capture, episode_id, *, step, generation=None):
    journal = EpisodeJournal(capture, episode_id)
    journal.emit('episode_start', received_wall_s=time.monotonic(),
                 processed_step=int(step), applied_step=None,
                 values={'record': 'lifecycle',
                         'generation': (None if generation is None
                                        else int(generation))})
    return journal


def run_headless(track, args, seed, rider, *, capture=None):
    # Recording is native-only: reject before any simulation is built so a
    # Python-backend request can never silently route or step (R02).
    if getattr(args, 'record', False) \
            and getattr(args, 'backend', 'python') != 'native':
        raise ValueError(
            'run recording requires the native backend; the ordinary '
            'physical Python backend has no recording adapter')
    # resolved_check_policy already encodes the headless-strict default;
    # hand-built args without it keep the historical strict headless mode.
    strict = True if getattr(args, 'resolved_check_policy', None) is None \
        else None
    driver = build_physical_driver(track, args, rider, seed=seed,
                                   strict=strict)
    journal = None
    try:
        if capture is None:
            capture = make_run_capture(
                driver, enabled=bool(getattr(args, 'record', False)),
                mode='headless')
        elif capture.enabled and driver.backend != 'native':
            raise ValueError(
                'run recording requires the native backend; the ordinary '
                'physical Python backend has no recording adapter')
        recording = capture.enabled
        out = Path(args.out)/physical_run_dir_name(track.name, driver.metadata)
        journal = _new_journal(capture, 0, step=driver.step)
        reason = None
        driver.start()
        try:
            while driver.reason is None:
                driver.advance(driver.max_steps, wall_budget_s=COMPUTE_SLICE_S)
            reason = driver.reason
        except KeyboardInterrupt:
            # Ctrl-C stops the run; it must not become another control input.
            reason = 'operator_stop'
        except InvalidReferenceRun as error:
            reason = 'invalid_controller'
            driver.metadata['failure'] = str(error)
        except (ValueError, RuntimeError, ArithmeticError) as error:
            reason = 'simulation_error'
            driver.metadata['failure'] = f'{type(error).__name__}: {error}'
        if recording:
            sealed, counter = set(), [0]
            summary = _seal_episode(
                driver, capture, journal, reason=reason,
                sealed=sealed, counter=counter)
            if summary is None:
                summary = driver.summary(reason)
            try:
                capture.finalize(out)
            except BaseException as publish_error:
                if driver.metadata.get('failure'):
                    publish_error.add_note(
                        f'primary failure before publication: '
                        f'{driver.metadata["failure"]}')
                raise
        else:
            reason = _check_tail_failure(driver, reason)
            try:
                summary = driver.export(out, reason=reason)
            except BaseException as export_error:
                if driver.metadata.get('failure'):
                    export_error.add_note(
                        f'primary failure before export: '
                        f'{driver.metadata["failure"]}')
                raise
        if not args.no_plots and driver.recorder.rows:
            from bike_sim.viz.ride_plots import plot_physical_ride
            plot_physical_ride(driver.recorder.columns(), out)
        from bike_sim.sim.ride.console import outcome
        end = summary['outcome']
        outcome(f"[bike-ride] {reason}: {end['time_s']:.6f} s, {end['position_m']:.3f} m -> {out}")
        return physical_exit_code(reason, summary)
    finally:
        driver.close()


def run_viewer(driver, *, out_root='output/ride', time_scale=1, record=False,
               capture=None):
    """Own the driver, the input journal and the launch-level capture.

    The pump journals raw key requests in receipt order before mutating
    braking/strength/playback state, seals each generation exactly once
    (outgoing episodes close before reset installs the next), and calls
    ``capture.finalize`` once before teardown — publication never happens
    from a render or a mid-run export.
    """
    keys = SimpleQueue()
    out = Path(out_root)/physical_run_dir_name(driver.track.name, driver.metadata)
    braking, strength = False, .5
    announced, stop = False, False
    sealed, counter = set(), [0]
    summary = None
    journal = None
    preview = None

    def seal_episode(reason):
        nonlocal summary
        if capture is not None and capture.enabled:
            sealed_summary = _seal_episode(
                driver, capture, journal, reason=reason,
                sealed=sealed, counter=counter)
            if sealed_summary is not None:
                summary = sealed_summary
            return summary
        frame = driver.snapshot()
        if frame.generation in sealed:
            return summary
        adjusted = _check_tail_failure(driver, reason)
        summary = driver.summary(reason=adjusted)
        sealed.add(frame.generation)
        counter[0] += 1
        return summary

    def seal_outgoing(reason):
        """Seal then reset, in that order (R03): the outgoing episode's
        evidence commits before a new generation is installed."""
        seal_episode(reason)

    try:
        if capture is None:
            capture = make_run_capture(driver, enabled=bool(record),
                                       mode='viewer')
        elif capture.enabled and driver.backend != 'native':
            raise ValueError(
                'run recording requires the native backend; the ordinary '
                'physical Python backend has no recording adapter')
        recording = capture.enabled
        import mujoco.viewer
        from bike_sim.sim.ride.console import info, outcome, print_help
        clock = PlaybackClock(driver.dt_s, scale=time_scale)
        presenter = FramePresenter(driver.make_render_model())
        frame = driver.snapshot()
        presenter.replica.apply(frame)
        journal = _new_journal(capture, counter[0], step=frame.step,
                               generation=frame.generation)
        applied_demand = (0., 0.)
        print_help(cruise_target_keys=False)
        info('F6 slower; F7 faster; F8 1x. Playback does not change the integration timestep.')
        if recording:
            info(f'[bike-ride] backend={driver.backend}; recording and live csv: {out}')
            preview = PhysicalLiveCsv(out/'preview.csv')
        else:
            info(f'[bike-ride] backend={driver.backend}; recording off (no files)')
        try:
            with mujoco.viewer.launch_passive(presenter.replica.model, presenter.replica.data,
                    key_callback=lambda key: keys.put((key, time.monotonic())),
                    show_left_ui=False, show_right_ui=False) as viewer:
                now = last_sync = last_hud = time.monotonic()
                rate = AchievedRate(now, driver.time_s)
                clock.rebase(now, step=driver.step)
                driver.start()
                while viewer.is_running() and not stop:
                    while True:
                        try:
                            item = keys.get_nowait()
                        except Empty:
                            break
                        # The callback stamps receipt; raw test keys fall back
                        # to dequeue time. Either way the request is journaled
                        # BEFORE any local control state changes.
                        key, received = (item if isinstance(item, tuple)
                                         else (item, time.monotonic()))
                        processed = driver.step
                        scale = speed_key(key, clock.scale)
                        if scale is not None:
                            journal.emit('playback_scale',
                                         received_wall_s=received,
                                         processed_step=processed,
                                         applied_step=None,
                                         values={'record': 'requested',
                                                 'scale': scale,
                                                 'previous_scale': clock.scale})
                            clock.set_scale(scale, now=time.monotonic(), step=driver.step)
                            rate.rebase(time.monotonic(), driver.time_s)
                        elif key == 32:
                            journal.emit('brake_toggle',
                                         received_wall_s=received,
                                         processed_step=processed,
                                         applied_step=None,
                                         values={'record': 'requested',
                                                 'braking': not braking})
                            braking = not braking
                        elif key in (44, 46):
                            updated = max(0., min(1., strength + (.1 if key == 46 else -.1)))
                            journal.emit('brake_strength',
                                         received_wall_s=received,
                                         processed_step=processed,
                                         applied_step=None,
                                         values={'record': 'requested',
                                                 'strength': updated})
                            strength = updated
                        elif key in (ord('R'), ord('r')):
                            journal.emit('reset',
                                         received_wall_s=received,
                                         processed_step=processed,
                                         applied_step=None,
                                         values={'record': 'requested'})
                            seal_outgoing(driver.reason or 'operator_reset')
                            frame = driver.reset()
                            journal = _new_journal(capture, counter[0],
                                                   step=frame.step,
                                                   generation=frame.generation)
                            braking, announced = False, False
                            applied_demand = (0., 0.)
                            presenter.camera.reset_preset()
                            now = time.monotonic()
                            clock.rebase(now, step=frame.step)
                            rate.rebase(now, frame.time_s)
                        elif key in (ord('Q'), ord('q'), 256):
                            journal.emit('quit',
                                         received_wall_s=received,
                                         processed_step=processed,
                                         applied_step=None,
                                         values={'record': 'requested'})
                            stop = True
                        elif presenter.handle_key(key, viewer):
                            journal.emit('viewer_key',
                                         received_wall_s=received,
                                         processed_step=processed,
                                         applied_step=None,
                                         values={'record': 'requested',
                                                 'key': int(key)})
                        elif key in (ord('?'), ord('/')):
                            journal.emit('help',
                                         received_wall_s=received,
                                         processed_step=processed,
                                         applied_step=None,
                                         values={'record': 'requested'})
                            print_help(cruise_target_keys=False)
                        else:
                            journal.emit('unhandled_key',
                                         received_wall_s=received,
                                         processed_step=processed,
                                         applied_step=None,
                                         values={'record': 'requested',
                                                 'key': int(key)})
                            info('Physical parameters are fixed per run; edit the configuration before a new run.')
                        # The user brake demand actually staged for the next
                        # advance is a separate applied record; a rollback
                        # hold inside the engine never overwrites it.
                        demand = strength if braking else 0.
                        if (demand, demand) != applied_demand:
                            applied_demand = (demand, demand)
                            journal.emit('brake_demand',
                                         received_wall_s=received,
                                         processed_step=processed,
                                         applied_step=(None if driver.reason
                                                       is not None else processed),
                                         values={'record': 'applied',
                                                 'front_brake_demand': demand,
                                                 'rear_brake_demand': demand,
                                                 'braking': braking,
                                                 'strength': strength})
                    now = time.monotonic()
                    target = clock.target_step(now, current_step=driver.step)
                    if not stop and driver.reason is None and target > driver.step:
                        demand = strength if braking else 0.
                        driver.advance(target, front_brake_demand=demand, rear_brake_demand=demand,
                                       wall_budget_s=COMPUTE_SLICE_S)
                    now = time.monotonic()
                    factor = rate.update(now, driver.time_s)
                    if driver.reason is not None and not announced:
                        seal_episode(driver.reason)
                        outcome(f'[RUN ENDED] {driver.reason}; t={driver.time_s:.6f}s; steps={driver.step}')
                        clock.rebase(time.monotonic(), step=driver.step)
                        announced = True
                    if now - last_sync >= RENDER_INTERVAL_S:
                        frame = driver.snapshot()
                        view = present_view(frame.view, requested_scale=clock.scale, achieved_rtf=factor)
                        if preview is not None and preview.due(frame.time_s, frame.generation):
                            preview.write(frame.time_s, frame.generation, presenter.hud.preview_log_row(view))
                        presenter.sync(viewer, frame)
                        last_sync = now
                        if now-last_hud >= .08:
                            presenter.print_hud(frame, view)
                            last_hud = now
                    time.sleep(.001)
            reason = driver.reason or 'viewer_closed'
            seal_episode(reason)
            published = capture.finalize(out)
            if preview is not None:
                frame = driver.snapshot()
                preview.write(frame.time_s, frame.generation,
                              presenter.hud.preview_log_row(present_view(frame.view,
                                  requested_scale=clock.scale, achieved_rtf=rate.value)))
                preview.write_marker(f'run ended; {reason}; t={frame.time_s:.6f}s')
                preview.close()
        finally:
            if preview is not None:
                preview.close()
        if recording and published is not None:
            try:
                # The report is generated only after publication; the same
                # stream of recorded preview rows feeds it.
                from bike_sim.viz.ride_plots import load_ride_csv, plot_physical_ride_html
                info(f"[bike-ride] physical live html: "
                     f"{plot_physical_ride_html(load_ride_csv(out/'preview.csv'), out)}")
            except ImportError:
                info("[bike-ride] skipped ride.html (plotly not installed; "
                     "use `uv run --with plotly`)")
        return physical_exit_code(summary['outcome']['reason'], summary)
    except BaseException as error:
        driver.metadata['failure'] = f'{type(error).__name__}: {error}'
        try:
            seal_episode('simulation_error')
        except BaseException as cleanup:
            error.add_note(f'physical episode sealing failed: {type(cleanup).__name__}: {cleanup}')
        try:
            if capture is not None:
                capture.finalize(out)
        except BaseException as cleanup:
            error.add_note(f'run recording finalize failed: {type(cleanup).__name__}: {cleanup}')
        raise
    finally:
        driver.close()
