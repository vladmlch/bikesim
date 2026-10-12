input requests are journaled in receipt order with received/processed
    timestamps, and publication happens exactly once at launch end through
``RunCapture.finalize`` — never from advance, reset or render. Preview
CSV and recording-derived plots are produced only when recording is on;
a non-recording launch leaves no persistent side effects.
CSV writers and automatic plots are not part of this path. The final report
is generated from the closed database; a disabled capture writes nothing.
"""
from __future__ import annotations

from dataclasses import asdict
from datetime import datetime, timezone
import sys
from pathlib import Path
from queue import Empty, SimpleQueue
import time

from bike_sim.recording.native import iter_derived_events
from bike_sim.sim.playback import PlaybackClock, COMPUTE_SLICE_S, RENDER_INTERVAL_S, speed_key
from bike_sim.sim.ride.physical_driver import build_physical_driver
from bike_sim.sim.ride.physical_session import PhysicalLiveCsv, physical_run_dir_name
from bike_sim.sim.ride.physical_session import physical_run_dir_name
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.physical_view import present_view
from bike_sim.sim.ride.presentation import AchievedRate, FramePresenter
from bike_sim.sim.ride.reference_monitor import InvalidReferenceRun

    return RunMetadata(
        run_id=uuid4(), started_at=datetime.now(timezone.utc),
        backend=driver.backend, mode=mode,
        values={'check_policy': 'strict' if getattr(driver, 'strict', False)
        values={'monotonic_origin_s': time.monotonic(), 'invocation': list(sys.argv),
                'setup': driver.metadata, 'check_policy': 'strict' if getattr(driver, 'strict', False)
                                else 'diagnostic',
                **dict(values or {})})


    return journal


def _command_request(journal, driver, front=0., rear=0., *, received=None):
    journal.emit('command_requested', received_wall_s=time.monotonic() if received is None else received,
                 processed_step=driver.step, applied_step=None,
                 values={'record': 'requested', 'time_s': driver.time_s, 'step': driver.step,
                         'control': asdict(RideControl()), 'front_brake_demand': front,
                         'rear_brake_demand': rear})


def _close_preserving(driver, primary=None):
    active = sys.exception() or primary
    try:
        driver.close()
    except BaseException as error:
        if active is None:
            raise
        active.add_note(f'driver close also failed: {type(error).__name__}: {error}')


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
    enabled = bool(getattr(args, 'record', False)) if capture is None else capture.enabled
    if enabled and getattr(args, 'backend', 'python') != 'native':
        raise ValueError('ordinary run recording requires the native backend')
    if capture is not None:
        from copy import copy
        args = copy(args)
        args.record = enabled
    strict = True if getattr(args, 'resolved_check_policy', None) is None else None
    driver = build_physical_driver(track, args, rider, seed=seed, strict=strict)
    primary, journal, published = None, None, None
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
            capture = make_run_capture(driver, enabled=enabled, mode='headless')
        out = Path(args.out)/physical_run_dir_name(track.name, driver.metadata)
        journal = _new_journal(capture, 0, step=driver.step)
        _command_request(journal, driver)
        reason = None
        driver.start()
        try:
            driver.start()
            while driver.reason is None:
                driver.advance(driver.max_steps, wall_budget_s=COMPUTE_SLICE_S)
            reason = driver.reason
        except KeyboardInterrupt:
            # Ctrl-C stops the run; it must not become another control input.
            reason = 'operator_stop'
        except KeyboardInterrupt as error:
            primary, reason = error, 'operator_stop'
        except InvalidReferenceRun as error:
            reason = 'invalid_controller'
            primary, reason = error, 'invalid_controller'
            driver.metadata['failure'] = str(error)
        except (ValueError, RuntimeError, ArithmeticError) as error:
            reason = 'simulation_error'
        except Exception as error:
            primary, reason = error, 'simulation_error'
            driver.metadata['failure'] = f'{type(error).__name__}: {error}'
        if recording:
            sealed, counter = set(), [0]
            summary = _seal_episode(
                driver, capture, journal, reason=reason,
                sealed=sealed, counter=counter)
            if summary is None:
        if primary is not None:
            journal.emit('interrupt' if isinstance(primary, KeyboardInterrupt) else 'execution_failure',
                received_wall_s=time.monotonic(), processed_step=driver.step,
                values={'record': 'lifecycle', 'time_s': driver.time_s,
                        'error': f'{type(primary).__name__}: {primary}'})
        try:
            if capture.enabled:
                summary = _seal_episode(driver, capture, journal, reason=reason,
                                        sealed=set(), counter=[0])
                if summary is None:
                    summary = driver.summary(reason)
                published = capture.finalize(out)
            else:
                reason = _check_tail_failure(driver, reason)
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
        except BaseException as secondary:
            if primary is not None and secondary is not primary:
                primary.add_note(f'finalization also failed: {type(secondary).__name__}: {secondary}')
                for note in getattr(secondary, '__notes__', ()):
                    primary.add_note(note)
                raise primary from secondary
            raise
        from bike_sim.sim.ride.console import outcome
        end = summary['outcome']
        outcome(f"[bike-ride] {reason}: {end['time_s']:.6f} s, {end['position_m']:.3f} m -> {out}")
        reason = end['reason']
        destination = 'recording off' if published is None else str(published)
        outcome(f"[bike-ride] {reason}: {end['time_s']:.6f} s, {end['position_m']:.3f} m -> {destination}")
        return physical_exit_code(reason, summary)
    finally:
        driver.close()
        _close_preserving(driver, primary)


def run_viewer(driver, *, out_root='output/ride', time_scale=1, record=False,

    sealed, counter = set(), [0]
    summary = None
    journal = None
    preview = None
    publication_attempted = False

    def seal_episode(reason):
        nonlocal summary

        presenter.replica.apply(frame)
        journal = _new_journal(capture, counter[0], step=frame.step,
                               generation=frame.generation)
        applied_demand = (0., 0.)
        requested_demand = (0., 0.)
        _command_request(journal, driver)
        print_help(cruise_target_keys=False)
        info('F6 slower; F7 faster; F8 1x. Playback does not change the integration timestep.')
        if recording:
            info(f'[bike-ride] backend={driver.backend}; recording and live csv: {out}')
            preview = PhysicalLiveCsv(out/'preview.csv')
            info(f'[bike-ride] backend={driver.backend}; recording in RAM until close: {out}')
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
                    journal.emit('key_received', received_wall_s=received,
                                 processed_step=processed, values={'record': 'requested',
                                 'key': int(key), 'time_s': driver.time_s})
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
                        requested_demand = (0., 0.)
                        _command_request(journal, driver)
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
                    # These are requests, not assertions that physics applied
                    # a command. Cancelled toggles remain visible. Applied
                    # records come from the retained physical control channels.
                    demand = strength if braking else 0.
                    if (demand, demand) != requested_demand:
                        requested_demand = (demand, demand)
                        _command_request(journal, driver, demand, demand, received=received)
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
                    presenter.sync(viewer, frame)
                    last_sync = now
                    if now-last_hud >= .08:
                        presenter.print_hud(frame, view)
                        last_hud = now
                time.sleep(.001)
        reason = driver.reason or 'viewer_closed'
        seal_episode(reason)
        publication_attempted = True
        published = capture.finalize(out)
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
            info(f'[bike-ride] recording: {published}')
        return physical_exit_code(summary['outcome']['reason'], summary)
    except BaseException as error:
        driver.metadata['failure'] = f'{type(error).__name__}: {error}'
        if journal is not None:
            try:
                journal.emit('frontend_failure', received_wall_s=time.monotonic(),
                             processed_step=driver.step, values={'error': str(error),
                             'time_s': driver.time_s, 'record': 'lifecycle'})
            except BaseException as journal_error:
                error.add_note(f'failure journaling also failed: {journal_error}')
        try:
            seal_episode('simulation_error')
            seal_episode('operator_stop' if isinstance(error, KeyboardInterrupt) else 'simulation_error')
        except BaseException as cleanup:
            error.add_note(f'physical episode sealing failed: {type(cleanup).__name__}: {cleanup}')
        try:
            if capture is not None:
            if capture is not None and not publication_attempted:
                publication_attempted = True
                capture.finalize(out)
        except BaseException as cleanup:
            error.add_note(f'run recording finalize failed: {type(cleanup).__name__}: {cleanup}')
        raise
    finally:
        driver.close()
        _close_preserving(driver)
