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


def make_ride_session(track, args, rider):
    reference = args.policy or DEFAULT_POLICY
    policy = load_policy(reference)
    experiment = ExperimentConfig(
        duration_s=90. if args.duration is None else args.duration,
        control_period_s=args.control_period, actuator_delay_s=args.actuator_delay,
        seed=0 if args.seed is None else args.seed, record_decimation=args.decimate,
        stop_on_model_violation=args.headless)
    sensors = SensorConfig(sample_period_s=args.sensor_period, latency_s=args.sensor_delay)
    program = None if args.rider_program is None else RiderProgram.load(args.rider_program)
    demand = None if args.demand is None else DemandProgram.constant(args.demand)
    env = build_environment(track=track, rider=rider, physics_config=args.resolved_physics,
        experiment=experiment, sensors=sensors, road_resolution_m=args.road_resolution,
        demand=demand, rider_program=program, backend=getattr(args, 'backend', 'python'))
    try:
        return PolicySession(env, policy, reference=reference)
    except BaseException as error:
        try:
            env.close(discard_pending=True)
        except BaseException as cleanup:
            error.add_note(f'research setup cleanup failed: {type(cleanup).__name__}: {cleanup}')
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


def run_research_viewer(session, output_dir, *, time_scale=1):
    """Owned snapshots, bounded slices, boundary-safe pause/stop/save/reset."""
    env = session.env
    keys = SimpleQueue()
    generation = 0
    saved = False
    paused = braking = stop = reset_requested = False

    def save_episode():
        nonlocal saved
        if not saved:
            destination = Path(output_dir)/f'episode-{generation:04d}'
            session.save(destination)
            print(f'[bike-ride research] {env.reason}: {destination}', flush=True)
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
        print('[bike-ride research] Space pause | R save/reset | B brakes | C/1/2 camera | '
              'T telemetry | G markers | Q stop | F6/F7/F8 speed', flush=True)
        with mujoco.viewer.launch_passive(model, presenter.replica.data, key_callback=keys.put,
                show_left_ui=False, show_right_ui=False) as viewer:
            now = last_sync = last_hud = time.monotonic()
            clock.rebase(now, step=env.sim.steps)
            rate = AchievedRate(now, env.sim.time_s)
            while viewer.is_running():
                while True:
                    try:
                        key = keys.get_nowait()
                    except Empty:
                        break
                    now = time.monotonic()
                    scale = speed_key(key, clock.scale)
                    if scale is not None:
                        clock.set_scale(scale, now=now, step=env.sim.steps)
                        rate.rebase(now, env.sim.time_s)
                    elif key == 32:
                        paused = not paused
                        session.pause() if paused else session.resume()
                        clock.set_paused(paused, now=now, step=env.sim.steps)
                        rate.rebase(now, env.sim.time_s)
                    elif key in (ord('R'), ord('r')):
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
                        generation += 1
                        session.reset()
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
        stop_at_boundary(session)
        save_episode()
        return episode_exit_code(env)
    except BaseException as error:
        try:
            stop_at_boundary(session)
            save_episode()
        except BaseException as cleanup:
            error.add_note(f'research finalization failed: {type(cleanup).__name__}: {cleanup}')
        raise
    finally:
        env.close(discard_pending=True)


def run_ride_research(track, args, rider):
    if not args.headless:
        from bike_sim.sim.playground import ensure_macos_mjpython
        ensure_macos_mjpython()
    output = Path(args.out)
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise FileExistsError('research output already exists; choose a new --out directory')
    session = make_ride_session(track, args, rider)
    try:
        if not args.headless:
            try:
                return run_research_viewer(session, output, time_scale=getattr(args, 'time_scale', 1))
            except Exception:
                if session.env.error is None:
                    raise
                print(f'[bike-ride research] {session.env.error}', flush=True)
                return episode_exit_code(session.env)
        try:
            while not session.env.done:
                session.advance()
        except KeyboardInterrupt:
            stop_at_boundary(session)
        except Exception:
            if session.env.error is None:
                raise
            print(f'[bike-ride research] {session.env.error}', flush=True)
        finally:
            session.save(output/'episode-0000')
        print(f'[bike-ride research] {session.env.reason}: '
              f'{session.env.sim.time_s:.3f}s x={session.env.sim.position_m:.3f}m -> {output}')
        return episode_exit_code(session.env)
    finally:
        session.env.close(discard_pending=True)
