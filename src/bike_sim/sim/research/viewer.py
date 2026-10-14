"""Display the accounted research loop without changing its integration clock."""
from pathlib import Path
from queue import Empty, SimpleQueue
import time

from bike_sim.sim.research.configuration import build_environment
from bike_sim.sim.research.demand import DemandProgram
from bike_sim.sim.research.environment import ExperimentConfig
from bike_sim.sim.research.policy_session import PolicySession, load_policy, episode_exit_code
from bike_sim.sim.research.rider_program import RiderProgram
from bike_sim.sim.research.sensors import SensorConfig


DEFAULT_POLICY = 'bike_sim.sim.research.policies:passthrough_factory'


def make_ride_session(track, args, rider, *, capture_mode='legacy'):
    reference = args.policy or DEFAULT_POLICY
    experiment = ExperimentConfig(
        duration_s=90. if args.duration is None else args.duration,
        control_period_s=args.control_period, actuator_delay_s=args.actuator_delay,
        seed=0 if args.seed is None else args.seed, record_decimation=args.decimate,
        stop_on_model_violation=getattr(args, 'resolved_check_policy', None) == 'strict'
            if getattr(args, 'resolved_check_policy', None) is not None else args.headless)
    sensors = SensorConfig(sample_period_s=args.sensor_period, latency_s=args.sensor_delay)
    program = None if args.rider_program is None else RiderProgram.load(args.rider_program)
    demand = None if args.demand is None else DemandProgram.constant(args.demand)
    env = build_environment(track=track, rider=rider, physics_config=args.resolved_physics,
        experiment=experiment, sensors=sensors, road_resolution_m=args.road_resolution,
        demand=demand, rider_program=program, backend=getattr(args, 'backend', 'python'), capture_mode=capture_mode)
    from bike_sim.sim.research.recording_lifecycle import ResearchRun, close_environment
    launch = None
    try:
        if capture_mode != 'legacy':
            launch = ResearchRun(env, args.out, enabled=capture_mode == 'full',
                                 mode='research_headless' if args.headless else 'research_viewer')
        try:
            policy = load_policy(reference)
        except Exception as error:
            env.fail_policy(error)
            raise
        session = PolicySession(env, policy, reference=reference)
        if launch is not None:
            launch.attach(session)
        return session
    except BaseException as error:
        if launch is not None:
            try:
                launch.finish(primary=error, stop=True)
            except BaseException as cleanup:
                if cleanup is not error:
                    error.add_note(f'research setup finalization failed: {type(cleanup).__name__}: {cleanup}')
        close_environment(env, error)
        raise


def advance_control_ticks(session, count, *, brake_demand=0.):
    if type(count) is not int or count < 0:
        raise ValueError('control tick count must be a nonnegative integer')
    completed = 0
    for tick_index in range(count):
        if session.env.done:
            break
        session.advance(front_brake_demand=brake_demand, rear_brake_demand=brake_demand)
        completed += 1
    return completed


def advance_playback(session, target_step, *, wall_budget_s=.008):
    """One bounded slice, without resubmitting a partially consumed command."""
    if session.env.done or target_step <= session.env.sim.steps:
        return None
    if not session.pending:
        if session.paused:
            return None
        session.begin_advance()
    return session.advance_pending(wall_budget_s=wall_budget_s, target_step=target_step)


def stop_at_boundary(session):
    """Complete only an already active window; never ask the policy for another."""
    from bike_sim.sim.playback import COMPUTE_SLICE_S
    session.stop()
    while session.pending:
        session.advance_pending(wall_budget_s=COMPUTE_SLICE_S)


def run_research_viewer(session, output_dir, *, time_scale=1, record=None, capture=None):
    """Owned snapshots, bounded slices, boundary-safe pause/stop/save/reset."""
    from bike_sim.sim.research.recording_lifecycle import ResearchRun, close_environment
    env = session.env
    enabled = getattr(env, 'capture_mode', 'legacy') == 'full' if record is None else bool(record)
    launch = getattr(env, 'recording_run', None)
    if launch is None:
        launch = ResearchRun(env, output_dir, enabled=enabled, mode='research_viewer', capture=capture)
        launch.attach(session)
    keys = SimpleQueue()
    primary = None
    saved = False
    paused = braking = stop = reset_requested = False

    def save_episode():
        nonlocal saved
        if not saved:
            launch.seal(reason=env.reason or 'viewer_closed')
            if launch.capture.enabled:
                print(f'[bike-ride research] episode {launch.episode_id}: {env.reason}; retained in RAM', flush=True)
            saved = True

    try:
        import mujoco.viewer
        from bike_sim.sim.playback import PlaybackClock, COMPUTE_SLICE_S, RENDER_INTERVAL_S, speed_key
        from bike_sim.sim.ride.physical_view import environment_snapshot, present_view
        from bike_sim.sim.ride.presentation import AchievedRate, FramePresenter
        from copy import copy
        clock = PlaybackClock(env.dt_s, scale=time_scale)
        model = env.make_render_model() if env.backend == 'native' else copy(env.sim.model)
        presenter = FramePresenter(model)
        presenter.replica.apply(environment_snapshot(env))
        print('[bike-ride research] Space pause | R close/reset | B brakes | C/1/2 camera | '
              'T telemetry | G markers | Q stop | F6/F7/F8 speed', flush=True)
        with mujoco.viewer.launch_passive(model, presenter.replica.data, key_callback=lambda key: keys.put((key, time.monotonic())),
                show_left_ui=False, show_right_ui=False) as viewer:
            now = last_sync = last_hud = time.monotonic()
            clock.rebase(now, step=env.sim.steps)
            rate = AchievedRate(now, env.sim.time_s)
            while viewer.is_running():
                while True:
                    try:
                        received = keys.get_nowait()
                    except Empty:
                        break
                    now = time.monotonic()
                    key, received_wall_s = received if isinstance(received, tuple) else (received, now)
                    launch.event('key_received', received_wall_s=received_wall_s,
                                 values={'key': key, 'terminal': env.done})
                    scale = speed_key(key, clock.scale)
                    if scale is not None:
                        launch.event('time_scale_requested', values={'scale': scale})
                        clock.set_scale(scale, now=now, step=env.sim.steps)
                        launch.event('time_scale_applied', applied=True, values={'scale': scale})
                        rate.rebase(now, env.sim.time_s)
                    elif key == 32:
                        paused = not paused
                        session.pause() if paused else session.resume()
                        clock.set_paused(paused, now=now, step=env.sim.steps)
                        rate.rebase(now, env.sim.time_s)
                    elif key in (ord('R'), ord('r')):
                        launch.event('reset_requested', values={'terminal': env.done})
                        reset_requested = True
                        session.stop()
                    elif key in (ord('B'), ord('b')):
                        braking = not braking
                        session.set_brakes(front_brake_demand=.5 if braking else 0.,
                                           rear_brake_demand=.5 if braking else 0.)
                    elif key in (ord('Q'), ord('q'), 256):
                        stop = True
                        session.stop()
                    elif not presenter.handle_key(key, viewer):
                        print('[bike-ride research] Physical parameters are fixed per run.', flush=True)
                now = time.monotonic()
                target = clock.target_step(now, current_step=env.sim.steps)
                if session.pending and (stop or reset_requested or paused):
                    # Boundary-safe operator changes finish the same command;
                    # no new command or policy call is issued while draining.
                    session.advance_pending(wall_budget_s=COMPUTE_SLICE_S)
                elif not stop and not reset_requested:
                    advance_playback(session, target, wall_budget_s=COMPUTE_SLICE_S)
                if not session.pending and (env.done or reset_requested or stop):
                    save_episode()
                    if stop:
                        break
                    if reset_requested:
                        launch.reset()
                        saved = paused = braking = reset_requested = False
                        now = time.monotonic()
                        clock.set_paused(False, now=now, step=env.sim.steps)
                        clock.rebase(now, step=env.sim.steps)
                        rate.rebase(now, env.sim.time_s)
                        presenter.camera.reset_preset()
                now = time.monotonic()
                factor = rate.update(now, env.sim.time_s)
                if now-last_sync >= RENDER_INTERVAL_S:
                    frame = environment_snapshot(env)
                    view = present_view(frame.view, requested_scale=clock.scale,
                                        achieved_rtf=factor, track=env.sim.track)
                    presenter.sync(viewer, frame)
                    last_sync = now
                    if now-last_hud >= .5:
                        quality = 'invalid' if not env.model_valid or not env.numerically_valid else 'synthetic'
                        presenter.print_hud(frame, view,
                            prefix=f'[research {env.backend} {quality} {env.reason or ("paused" if paused else "running")}] ')
                        last_hud = now
                time.sleep(.001)
        output = launch.finish(stop=True)
        print(f'[bike-ride research] {env.reason}: ' +
              ('recording off' if output is None else str(output)), flush=True)
        return episode_exit_code(env)
    except BaseException as error:
        primary = error
        if not launch.finalization_attempted:
            launch.finish(primary=error, stop=True)
        raise
    finally:
        close_environment(env, primary)


def run_ride_research(track, args, rider):
    if not args.headless:
        from bike_sim.sim.playground import ensure_macos_mjpython
        ensure_macos_mjpython()
    from bike_sim.sim.research.recording_lifecycle import close_environment
    session = make_ride_session(track, args, rider,
                                capture_mode='full' if getattr(args, 'record', False) else 'off')
    launch = session.env.recording_run
    if not args.headless:
        return run_research_viewer(session, args.out, time_scale=getattr(args, 'time_scale', 1),
                                   record=getattr(args, 'record', False))
    primary = None
    try:
        while not session.env.done:
            session.advance()
        output = launch.finish()
        print(f'[bike-ride research] {session.env.reason}: '
              f'{session.env.sim.time_s:.3f}s x={session.env.sim.position_m:.3f}m; ' +
              ('recording off' if output is None else str(output)))
        return episode_exit_code(session.env)
    except BaseException as error:
        primary = error
        if not launch.finalization_attempted:
            launch.finish(primary=error, stop=True)
        raise
    finally:
        close_environment(session.env, primary)
