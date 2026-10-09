"""Physical CLI frontends sharing one backend-neutral owner and playback clock."""
from __future__ import annotations

from pathlib import Path
from queue import Empty, SimpleQueue
import time

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


def run_headless(track, args, seed, rider):
    driver = build_physical_driver(track, args, rider, seed=seed, strict=True)
    try:
        out = Path(args.out)/physical_run_dir_name(track.name, driver.metadata)
        reason = None
        driver.start()
        try:
            while driver.reason is None:
                driver.advance(driver.max_steps, wall_budget_s=COMPUTE_SLICE_S)
            reason = driver.reason
        except KeyboardInterrupt:
            reason = 'operator_stop'
        except InvalidReferenceRun as error:
            reason = 'invalid_controller'
            driver.metadata['failure'] = str(error)
        except (ValueError, RuntimeError, ArithmeticError) as error:
            reason = 'simulation_error'
            driver.metadata['failure'] = f'{type(error).__name__}: {error}'
        try:
            driver.flush()
        except InvalidReferenceRun as error:
            if reason != 'simulation_error':
                reason = 'invalid_controller'
            driver.metadata['tail_failure'] = str(error)
        except (ValueError, RuntimeError, ArithmeticError) as error:
            if reason != 'simulation_error':
                reason = 'simulation_error'
            driver.metadata['tail_failure'] = f'{type(error).__name__}: {error}'
        summary = driver.export(out, reason=reason)
        if not args.no_plots and driver.recorder.rows:
            from bike_sim.viz.ride_plots import plot_physical_ride
            plot_physical_ride(driver.recorder.columns(), out)
        from bike_sim.sim.ride.console import outcome
        end = summary['outcome']
        outcome(f"[bike-ride] {reason}: {end['time_s']:.6f} s, {end['position_m']:.3f} m -> {out}")
        return physical_exit_code(reason, summary)
    finally:
        driver.close()


def run_viewer(driver, *, out_root='output/ride', time_scale=1):
    """Own the driver through final flush/export/close, including window errors."""
    keys = SimpleQueue()
    out = Path(out_root)/physical_run_dir_name(driver.track.name, driver.metadata)
    braking, strength = False, .5
    announced, stop = False, False
    exported = set()
    summary = None
    preview = None

    def export_generation(reason):
        nonlocal summary
        frame = driver.snapshot()
        if frame.generation not in exported:
            # Keep committed evidence even when a trailing check fails.
            try:
                driver.flush()
            except InvalidReferenceRun as error:
                driver.metadata['tail_failure'] = str(error)
                if reason != 'simulation_error':
                    reason = 'invalid_controller'
            except (ValueError, RuntimeError, ArithmeticError) as error:
                driver.metadata['tail_failure'] = f'{type(error).__name__}: {error}'
                reason = 'simulation_error'
            summary = driver.export(out/f'generation-{frame.generation:04d}', reason=reason)
            exported.add(frame.generation)
        return summary

    try:
        import mujoco.viewer
        from bike_sim.sim.ride.console import info, outcome, print_help
        clock = PlaybackClock(driver.dt_s, scale=time_scale)
        presenter = FramePresenter(driver.make_render_model())
        frame = driver.snapshot()
        presenter.replica.apply(frame)
        print_help()
        info('F6 slower; F7 faster; F8 1x. Playback does not change the integration timestep.')
        info(f'[bike-ride] backend={driver.backend}; physical live csv: {out / "preview.csv"}')
        with PhysicalLiveCsv(out/'preview.csv') as preview:
            with mujoco.viewer.launch_passive(presenter.replica.model, presenter.replica.data,
                    key_callback=keys.put, show_left_ui=False, show_right_ui=False) as viewer:
                now = last_sync = last_hud = time.monotonic()
                rate = AchievedRate(now, driver.time_s)
                clock.rebase(now, step=driver.step)
                driver.start()
                while viewer.is_running() and not stop:
                    while True:
                        try:
                            key = keys.get_nowait()
                        except Empty:
                            break
                        now = time.monotonic()
                        scale = speed_key(key, clock.scale)
                        if scale is not None:
                            clock.set_scale(scale, now=now, step=driver.step)
                            rate.rebase(now, driver.time_s)
                        elif key == 32:
                            braking = not braking
                        elif key in (44, 46):
                            strength = max(0., min(1., strength + (.1 if key == 46 else -.1)))
                        elif key in (ord('R'), ord('r')):
                            export_generation(driver.reason or 'operator_reset')
                            frame = driver.reset()
                            braking, announced = False, False
                            presenter.camera.reset_preset()
                            now = time.monotonic()
                            clock.rebase(now, step=frame.step)
                            rate.rebase(now, frame.time_s)
                        elif key in (ord('Q'), ord('q'), 256):
                            stop = True
                        elif presenter.handle_key(key, viewer):
                            pass
                        elif key in (ord('?'), ord('/')):
                            print_help()
                        else:
                            info('Physical parameters are fixed per run; edit the configuration before a new run.')
                    now = time.monotonic()
                    target = clock.target_step(now, current_step=driver.step)
                    if not stop and driver.reason is None and target > driver.step:
                        demand = strength if braking else 0.
                        driver.advance(target, front_brake_demand=demand, rear_brake_demand=demand,
                                       wall_budget_s=COMPUTE_SLICE_S)
                    now = time.monotonic()
                    factor = rate.update(now, driver.time_s)
                    if driver.reason is not None and not announced:
                        export_generation(driver.reason)
                        outcome(f'[RUN ENDED] {driver.reason}; t={driver.time_s:.6f}s; steps={driver.step}')
                        clock.rebase(time.monotonic(), step=driver.step)
                        announced = True
                    if now - last_sync >= RENDER_INTERVAL_S:
                        frame = driver.snapshot()
                        view = present_view(frame.view, requested_scale=clock.scale, achieved_rtf=factor)
                        if preview.due(frame.time_s, frame.generation):
                            preview.write(frame.time_s, frame.generation, presenter.hud.preview_log_row(view))
                        presenter.sync(viewer, frame)
                        last_sync = now
                        if now-last_hud >= .08:
                            presenter.print_hud(frame, view)
                            last_hud = now
                    time.sleep(.001)
            reason = driver.reason or 'viewer_closed'
            summary = export_generation(reason)
            frame = driver.snapshot()
            preview.write(frame.time_s, frame.generation,
                          presenter.hud.preview_log_row(present_view(frame.view,
                              requested_scale=clock.scale, achieved_rtf=rate.value)))
            preview.write_marker(f'run ended; {reason}; t={frame.time_s:.6f}s')
        return physical_exit_code(summary['outcome']['reason'], summary)
    except BaseException as error:
        driver.metadata['failure'] = f'{type(error).__name__}: {error}'
        try:
            export_generation('simulation_error')
        except BaseException as cleanup:
            error.add_note(f'physical final export failed: {type(cleanup).__name__}: {cleanup}')
        raise
    finally:
        driver.close()
