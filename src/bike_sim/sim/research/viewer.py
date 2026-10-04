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
        demand=demand, rider_program=program)
    return PolicySession(env, policy, reference=reference)


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


def run_research_viewer(session, output_dir):
    """Space pauses, R saves/resets, B toggles brakes,
    C changes camera, Q ends."""
    import mujoco.viewer
    from bike_sim.sim.camera import CameraManager
    from bike_sim.sim.ride.viewer import RealTimePacer

    env = session.env
    sim = env.sim
    camera = CameraManager(default_mode='2d')
    pacer = RealTimePacer(env.config.control_period_s,
                          max_catchup_s=max(.05, env.config.control_period_s))
    keys = SimpleQueue()
    generation = 0
    saved = False
    paused = False
    braking = False
    stop = False

    def save_episode():
        nonlocal saved
        if not saved:
            destination = Path(output_dir) / f'episode-{generation:04d}'
            session.save(destination)
            print(f'[bike-ride research] {env.reason}: {destination}', flush=True)
            saved = True

    print('[bike-ride research] Space pause | R reset | B brakes | C camera | Q stop', flush=True)
    try:
        with mujoco.viewer.launch_passive(
                sim.model, sim.data, key_callback=keys.put,
                show_left_ui=False, show_right_ui=False) as viewer:
            previous = last_sync = last_hud = rate_wall = time.monotonic()
            rate_sim = sim.time_s
            while viewer.is_running() and not stop:
                while True:
                    try:
                        key = keys.get_nowait()
                    except Empty:
                        break
                    if key == 32:
                        paused = not paused
                        pacer.reset()
                    elif key == ord('R'):
                        session.stop()
                        save_episode()
                        generation += 1
                        session.reset()
                        saved = paused = braking = False
                        pacer.reset()
                        previous = rate_wall = time.monotonic()
                        rate_sim = sim.time_s
                    elif key == ord('B'):
                        braking = not braking
                    elif key == ord('C'):
                        camera.cycle_mode()
                    elif key in (ord('Q'), 256):
                        stop = True
                    else:
                        print('[bike-ride research] Change physical parameters in the config before a new run.', flush=True)
                now = time.monotonic()
                if not paused and not env.done and not stop:
                    advance_control_ticks(session, pacer.steps_for(now-previous),
                                          brake_demand=.5 if braking else 0.)
                previous = now
                if env.done:
                    save_episode()
                    pacer.reset()
                now = time.monotonic()
                if now-last_sync >= 1./60.:
                    with viewer.lock():
                        camera.update_viewer(viewer, bike_x=sim.position_m,
                            bike_z=float(sim.data.xpos[sim._frame_body_id, 2]))
                    viewer.sync()
                    last_sync = now
                if now-last_hud >= .5:
                    elapsed = now-rate_wall
                    factor = (sim.time_s-rate_sim)/elapsed if elapsed else 0.
                    quality = ('invalid' if not env.model_valid or not env.numerically_valid
                               else 'synthetic')
                    print(f'\rt={sim.time_s:.3f}s x={sim.position_m:.2f}m '
                          f'motor={env.observation.motor_torque_nm:.1f}Nm '
                          f'{env.tracker.state} {quality} rtf={factor:.2f} '
                          f'{env.reason or ("paused" if paused else "running")}    ',
                          end='', flush=True)
                    rate_wall, rate_sim, last_hud = now, sim.time_s, now
                time.sleep(.001)
    finally:
        session.stop()
        save_episode()
        print()
    return episode_exit_code(env)


def run_ride_research(track, args, rider):
    if not args.headless:
        from bike_sim.sim.playground import ensure_macos_mjpython
        ensure_macos_mjpython()
    output = Path(args.out)
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise FileExistsError('research output already exists; choose a new --out directory')
    session = make_ride_session(track, args, rider)
    if not args.headless:
        try:
            return run_research_viewer(session, output)
        except Exception:
            if session.env.error is None:
                raise
            print(f'[bike-ride research] {session.env.error}', flush=True)
            return episode_exit_code(session.env)
    try:
        while not session.env.done:
            session.advance()
    except KeyboardInterrupt:
        session.stop()
    except Exception:
        if session.env.error is None:
            raise
        print(f'[bike-ride research] {session.env.error}', flush=True)
    finally:
        session.save(output / 'episode-0000')
    print(f'[bike-ride research] {session.env.reason}: '
          f'{session.env.sim.time_s:.3f}s x={session.env.sim.position_m:.3f}m -> {output}')
    return episode_exit_code(session.env)
