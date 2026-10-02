"""Headless torque-control experiments; all dependencies are in the offline bundle."""
import argparse
from dataclasses import replace
from pathlib import Path
import json
import sys
import tomllib
from bike_sim.physics.chain import DrivetrainSpecs
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.physical_config import AssistConfig, PhysicalDriveConfig, TireBackendConfig, TireParameters
from bike_sim.physics.rider import RiderSpecs
from bike_sim.physics.rider_posture import RiderPosture
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.research.configuration import (
    DEFAULT_TIME_STEPS, build_environment, resolve_research_physics,
)
from bike_sim.sim.research.demand import DemandProgram
from bike_sim.sim.research.rider_random import RiderRandomSpec, sample_rider
from bike_sim.sim.research.environment import ExperimentConfig
from bike_sim.sim.research.rider_program import RiderProgram
from bike_sim.sim.research.policy_session import PolicySession, load_policy, episode_exit_code
from bike_sim.sim.research.sensors import SensorConfig
from bike_sim.terrain.generator import TerrainGenSpec, generate_track
from bike_sim.terrain.research import RESEARCH_SCENARIOS, build_research_track
from bike_sim.terrain.trackfile import load_track


TRANSMISSIONS = ('elastic_chain', 'ideal_mid_drive', 'geometric_ideal_mid_drive')


class ExplicitPhysicsValue(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        supplied = set(getattr(namespace, '_explicit_physics', ()))
        supplied.add(self.dest)
        setattr(namespace, '_explicit_physics', supplied)
        setattr(namespace, self.dest, values)


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--physics-config', type=Path)
    p.add_argument('--road-resolution', type=float, default=.005, help='compiled road spacing, m')
    p.add_argument('--sensor-period', type=float, default=.005, help='acquisition period, s')
    p.add_argument('--rider-program', type=Path)
    p.add_argument('--policy', help='motor policy factory, module:factory')
    p.add_argument('--diagnostic-model-limits', action='store_true',
                   help='continue out-of-scope diagnostic runs; validity remains false')
    p.add_argument('--scenario', choices=RESEARCH_SCENARIOS+('generated',), default='rough_uphill',
        help="'generated' draws a seeded procedural rough climb (see --gen-spec); the seed is --seed")
    p.add_argument('--gen-spec', type=Path, help='TOML of TerrainGenSpec ranges for --scenario generated')
    p.add_argument('--track-file', type=Path, help='TOML overrides --scenario; preserves its authored seed/geometry')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--rider', choices=('lumped', 'articulated_planar'), default='articulated_planar')
    p.add_argument('--rider-mass', type=float, default=80., help='kg')
    p.add_argument('--rider-height', type=float, default=1.8, help='m')
    p.add_argument('--rider-random', action='store_true',
        help='sample rider mass/height/posture/effort from --rider-seed (articulated rider; replaces --rider-mass/-height)')
    p.add_argument('--rider-seed', type=int, default=None, help='default: --seed')
    p.add_argument('--posture', choices=('neutral', 'forward', 'crouched', 'standing'), default='neutral')
    p.add_argument('--duration', type=float, default=3., help='simulation seconds (10-30 s episodes are supported; see --record-decimation)')
    p.add_argument('--dt', type=float, default=None, action=ExplicitPhysicsValue,
        help='physics timestep, seconds (default: 0.0005 for the ideal transmissions, validated by '
             'tools/validate_antiwheelie.py; 0.000125 for elastic_chain, whose low gears oscillate at 0.5 ms)')
    p.add_argument('--control-period', type=float, default=.01, help='seconds, integer multiple of --dt')
    p.add_argument('--actuator-delay', type=float, default=.005, help='seconds, integer multiple of --dt')
    p.add_argument('--sensor-delay', type=float, default=.01, help='seconds')
    p.add_argument('--ideal-sensors', action='store_true', help='zero noise and zero sensor latency')
    p.add_argument('--initial-brake-demand', type=float, default=1., action=ExplicitPhysicsValue, help='both brakes during static initialization only; released at t=0')
    p.add_argument('--initial-speed', type=float, default=2., action=ExplicitPhysicsValue, help='m/s (unlike legacy bike-ride km/h)')
    p.add_argument('--human-torque', type=float, default=0., action=ExplicitPhysicsValue, help='mean crank torque, N*m')
    p.add_argument('--motor-torque', type=float, default=80., help='external crank-side setpoint, N*m')
    p.add_argument('--demand', type=float, default=None,
        help='constant torque demand, N*m: advisory echo as env.demand_nm; the CLI loop passes it as the motor setpoint')
    p.add_argument('--demand-file', type=Path, help='DemandProgram TOML ([[keyframes]] time_s/torque_nm)')
    p.add_argument('--assist', action='store_true', help='use configured pedelec demand instead of external motor setpoint')
    p.add_argument('--motor-limit', type=float, default=None, help='immediate crank-side safety ceiling, N*m')
    p.add_argument('--motor-max-torque', type=float, default=80., action=ExplicitPhysicsValue, help='synthetic motor envelope, N*m')
    p.add_argument('--motor-max-power', type=float, default=500., action=ExplicitPhysicsValue, help='synthetic shaft power envelope, W')
    p.add_argument('--transmission', choices=TRANSMISSIONS, default='ideal_mid_drive', action=ExplicitPhysicsValue,
        help='ideal_mid_drive: one-way tendon, cheap, omits chain-growth/suspension coupling; '
             'geometric_ideal_mid_drive: experimental, tendon linearized from the chain geometry; '
             'elastic_chain: frozen reference for A/B comparison')
    p.add_argument('--chain-stiffness', type=float, default=None, action=ExplicitPhysicsValue, help='override chain spring rate, N/m')
    p.add_argument('--freehub-stiffness', type=float, default=None, action=ExplicitPhysicsValue, help='override freehub spring rate, N*m/rad')
    p.add_argument('--front-teeth', type=int, default=34, action=ExplicitPhysicsValue)
    p.add_argument('--rear-teeth', type=int, default=51, action=ExplicitPhysicsValue)
    p.add_argument('--record-decimation', type=int, default=80,
        help='keep every Nth physics interval in telemetry; for 10-30 s episodes use >= 80 (memory and file size scale with duration/dt/N)')
    p.add_argument('--energy-tolerance', type=float, default=.05, help='maximum normalized numerical energy residual')
    p.add_argument('--out', type=Path, default=Path('output/antiwheelie'))
    p.add_argument('--overwrite', action='store_true')
    return p


def build_track(args):
    """--track-file wins over --scenario so an authored/frozen track is never regenerated."""
    if args.track_file:
        return load_track(args.track_file)
    if args.gen_spec is not None and args.scenario != 'generated':
        raise ValueError('--gen-spec requires --scenario generated')
    if args.scenario == 'generated':
        spec = TerrainGenSpec.from_dict(tomllib.loads(args.gen_spec.read_text())) if args.gen_spec else TerrainGenSpec()
        return generate_track(spec, seed=args.seed, name=f'generated_{args.seed}')
    return build_research_track(args.scenario, seed=args.seed)


# Largest step where every tools/validate_antiwheelie.py case passes with energy ratio < 0.05
# (verification/dt_sweep_ideal). 0.001 is refused by SimulationPhysicsConfig's
# closure_time_constant_s >= 2*dt guard, so it is not a candidate without a model change.
IDEAL_DT_S = DEFAULT_TIME_STEPS['ideal_mid_drive']
CHAIN_DT_S = DEFAULT_TIME_STEPS['elastic_chain']


def resolve_dt(args):
    if args.dt is not None:
        return args.dt
    return CHAIN_DT_S if args.transmission == 'elastic_chain' else IDEAL_DT_S


def build_rider(args):
    """(RiderSpecs, RiderProgram|None). A sampled rider brings a program that owns posture and effort."""
    if args.rider_program is not None:
        if (args.rider_random or args.rider != 'articulated_planar' or args.posture != 'neutral'
                or 'human_torque' in getattr(args, '_explicit_physics', ())):
            raise ValueError('--rider-program owns articulated posture and effort; drop conflicting rider inputs')
        return (RiderSpecs(variant=args.rider, mass_kg=args.rider_mass, height_m=args.rider_height),
                RiderProgram.load(args.rider_program))
    if args.rider_random:
        if args.rider != 'articulated_planar' or args.posture != 'neutral' or args.human_torque != 0.:
            raise ValueError('--rider-random owns rider type, posture and effort; drop --rider/--posture/--human-torque')
        return sample_rider(RiderRandomSpec(), args.seed if args.rider_seed is None else args.rider_seed)
    return RiderSpecs(variant=args.rider, mass_kg=args.rider_mass, height_m=args.rider_height), None


def build_demand(args):
    if args.demand is not None and args.demand_file is not None:
        raise ValueError('choose one of --demand and --demand-file')
    if args.demand_file is not None:
        return DemandProgram.load(args.demand_file)
    return None if args.demand is None else DemandProgram.constant(args.demand)


def make_environment(args):
    track = build_track(args)
    if args.posture != 'neutral' and args.rider != 'articulated_planar':
        raise ValueError('dynamic posture requires --rider articulated_planar')
    tires = TireBackendConfig(backend='compliant_2d', surface_mode='track',
        front=TireParameters(mu=1.1), rear=TireParameters(mu=1.1))
    defaults = parser().parse_args([])
    drive = PhysicalDriveConfig(human_torque_nm=defaults.human_torque,
        gearing=DrivetrainSpecs(defaults.front_teeth, defaults.rear_teeth), transmission_model=defaults.transmission,
        assist=AssistConfig(max_torque=defaults.motor_max_torque, max_power=defaults.motor_max_power))
    cfg = SimulationPhysicsConfig('physical',
        drive_mode='articulated_effort' if args.rider == 'articulated_planar' else 'crank_effort',
        timestep_s=resolve_dt(defaults), initial_speed_mps=defaults.initial_speed, tires=tires, drive=drive,
        initial_front_brake=defaults.initial_brake_demand, initial_rear_brake=defaults.initial_brake_demand,
        # Let unilateral rider supports settle before an expensive static solve.
        # Frequent early refinement helps the lumped plant but wastes solves on
        # an articulated pose whose feet/saddle have not settled yet.
        equilibrium_refine_after_s=3. if args.rider == 'articulated_planar' else .1,
        equilibrium_refine_period_s=3. if args.rider == 'articulated_planar' else .5)
    cfg = resolve_research_physics(cfg, args)
    if cfg.drive.transmission_model != 'elastic_chain' and (
            args.chain_stiffness is not None or args.freehub_stiffness is not None):
        raise ValueError('--chain-stiffness/--freehub-stiffness apply only to --transmission elastic_chain')
    experiment = ExperimentConfig(control_period_s=args.control_period, actuator_delay_s=args.actuator_delay,
        duration_s=args.duration, seed=args.seed, record_decimation=args.record_decimation,
        maximum_energy_residual_ratio=args.energy_tolerance,
        stop_on_model_violation=not args.diagnostic_model_limits)
    sensors = SensorConfig.ideal() if args.ideal_sensors else SensorConfig(latency_s=args.sensor_delay)
    sensors = replace(sensors, sample_period_s=args.sensor_period)
    rider, program = build_rider(args)
    return build_environment(track=track, rider=rider, physics_config=cfg,
        experiment=experiment, sensors=sensors, road_resolution_m=args.road_resolution,
        demand=build_demand(args), rider_program=program)


def posture_at(name, time_s):
    if name == 'neutral':
        return None
    phase = max(0., min(1., (time_s-.5)/.5))
    blend = phase*phase*(3.-2.*phase)
    if name == 'forward':
        return RiderPosture(torso_lean_rad=.20*blend, pelvis_pitch_rad=.05*blend)
    if name == 'crouched':
        return RiderPosture(torso_lean_rad=.10*blend, pelvis_offset_m=(.02*blend, -.06*blend))
    return RiderPosture(torso_lean_rad=.08*blend, pelvis_offset_m=(0., .16*blend), use_saddle=blend < .1)


def command_for(env, args):
    """The CLI's fixed 'policy': setpoint (or demand passthrough), never an anti-wheelie law."""
    owned = env.rider_program is not None  # a sampled rider owns posture and effort
    motor = env.demand_nm if env.demand_nm is not None else args.motor_torque
    explicit_human = 'human_torque' in getattr(args, '_explicit_physics', ())
    human = args.human_torque if explicit_human or args.physics_config is None else None
    return RideControl(motor_torque_nm=None if args.assist else motor, motor_limit_nm=args.motor_limit,
        human_torque_nm=None if owned else human,
        posture=None if owned else posture_at(args.posture, env.sim.time_s))


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    env = None
    session = None
    try:
        # Validate a complete command before compiling the plant or touching outputs.
        RideControl(motor_torque_nm=None if args.assist else args.motor_torque,
                    motor_limit_nm=args.motor_limit, human_torque_nm=args.human_torque)
        if args.out.exists() and any(args.out.iterdir()) and not args.overwrite:
            raise FileExistsError(f'output exists; choose another --out or pass --overwrite: {args.out}')
        policy = None if args.policy is None else load_policy(args.policy)
        env = make_environment(args)
        if policy is not None:
            session = PolicySession(env, policy, reference=args.policy)
        while not env.done:
            if session is None:
                env.step(command_for(env, args))
            else:
                session.advance()
        if session is None:
            env.save(args.out, overwrite=args.overwrite)
        else:
            session.save(args.out, overwrite=args.overwrite)
        print(json.dumps(dict(output=str(args.out), outcome=env.reason, metrics=env.tracker.metrics), indent=2))
        return episode_exit_code(env)
    except Exception as exc:
        if env is not None and env.error is not None:
            if session is None:
                env.save(args.out, overwrite=args.overwrite)
            else:
                session.save(args.out, overwrite=args.overwrite)
        print(f'ERROR: {exc}', file=sys.stderr)
        return 3 if env is not None and env.error is not None else 2


if __name__ == '__main__':
    raise SystemExit(main())
