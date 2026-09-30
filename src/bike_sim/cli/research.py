"""Headless torque-control experiments; all dependencies are in the offline bundle."""
import argparse
from pathlib import Path
import json
import sys
from bike_sim.physics.chain import DrivetrainSpecs
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.physical_config import AssistConfig, PhysicalDriveConfig, TireBackendConfig, TireParameters
from bike_sim.physics.rider import RiderSpecs
from bike_sim.physics.rider_posture import RiderPosture
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.sim.research.environment import ExperimentConfig, ResearchEnvironment
from bike_sim.sim.research.sensors import SensorConfig
from bike_sim.terrain.research import RESEARCH_SCENARIOS, build_research_track
from bike_sim.terrain.trackfile import load_track


TRANSMISSIONS = ('elastic_chain', 'ideal_mid_drive', 'geometric_ideal_mid_drive')


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenario', choices=RESEARCH_SCENARIOS, default='rough_uphill')
    p.add_argument('--track-file', type=Path, help='TOML overrides --scenario; preserves its authored seed/geometry')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--rider', choices=('lumped', 'articulated_planar'), default='articulated_planar')
    p.add_argument('--rider-mass', type=float, default=80., help='kg')
    p.add_argument('--rider-height', type=float, default=1.8, help='m')
    p.add_argument('--posture', choices=('neutral', 'forward', 'crouched', 'standing'), default='neutral')
    p.add_argument('--duration', type=float, default=3., help='simulation seconds')
    p.add_argument('--dt', type=float, default=.000125, help='physics timestep, seconds')
    p.add_argument('--control-period', type=float, default=.01, help='seconds, integer multiple of --dt')
    p.add_argument('--actuator-delay', type=float, default=.005, help='seconds, integer multiple of --dt')
    p.add_argument('--sensor-delay', type=float, default=.01, help='seconds')
    p.add_argument('--ideal-sensors', action='store_true', help='zero noise and zero sensor latency')
    p.add_argument('--initial-brake-demand', type=float, default=1., help='both brakes during static initialization only; released at t=0')
    p.add_argument('--initial-speed', type=float, default=2., help='m/s (unlike legacy bike-ride km/h)')
    p.add_argument('--human-torque', type=float, default=0., help='mean crank torque, N*m')
    p.add_argument('--motor-torque', type=float, default=80., help='external crank-side setpoint, N*m')
    p.add_argument('--assist', action='store_true', help='use configured pedelec demand instead of external motor setpoint')
    p.add_argument('--motor-limit', type=float, default=None, help='immediate crank-side safety ceiling, N*m')
    p.add_argument('--motor-max-torque', type=float, default=80., help='synthetic motor envelope, N*m')
    p.add_argument('--motor-max-power', type=float, default=500., help='synthetic shaft power envelope, W')
    p.add_argument('--transmission', choices=TRANSMISSIONS, default='ideal_mid_drive',
        help='ideal_mid_drive: one-way tendon, cheap, omits chain-growth/suspension coupling; '
             'geometric_ideal_mid_drive: experimental, tendon linearized from the chain geometry; '
             'elastic_chain: frozen reference for A/B comparison')
    p.add_argument('--chain-stiffness', type=float, default=None, help='override chain spring rate, N/m')
    p.add_argument('--freehub-stiffness', type=float, default=None, help='override freehub spring rate, N*m/rad')
    p.add_argument('--front-teeth', type=int, default=34)
    p.add_argument('--rear-teeth', type=int, default=51)
    p.add_argument('--record-decimation', type=int, default=80)
    p.add_argument('--energy-tolerance', type=float, default=.05, help='maximum normalized numerical energy residual')
    p.add_argument('--out', type=Path, default=Path('output/antiwheelie'))
    p.add_argument('--overwrite', action='store_true')
    return p


def make_environment(args):
    track = load_track(args.track_file) if args.track_file else build_research_track(args.scenario, seed=args.seed)
    if args.posture != 'neutral' and args.rider != 'articulated_planar':
        raise ValueError('dynamic posture requires --rider articulated_planar')
    tires = TireBackendConfig(backend='compliant_2d', surface_mode='track',
        front=TireParameters(mu=1.1), rear=TireParameters(mu=1.1))
    drive_kwargs = {}
    if args.transmission != 'elastic_chain' and (args.chain_stiffness is not None or args.freehub_stiffness is not None):
        # Explicit over silent: the ideal modes have no chain/freehub spring to tune.
        raise ValueError('--chain-stiffness/--freehub-stiffness apply only to --transmission elastic_chain')
    if args.chain_stiffness is not None:
        drive_kwargs['chain_k_n_m'] = args.chain_stiffness
    if args.freehub_stiffness is not None:
        drive_kwargs['freehub_k_nm_rad'] = args.freehub_stiffness
    drive = PhysicalDriveConfig(human_torque_nm=args.human_torque,
        gearing=DrivetrainSpecs(args.front_teeth, args.rear_teeth), transmission_model=args.transmission,
        assist=AssistConfig(max_torque=args.motor_max_torque, max_power=args.motor_max_power),
        **drive_kwargs)
    cfg = SimulationPhysicsConfig('physical',
        drive_mode='articulated_effort' if args.rider == 'articulated_planar' else 'crank_effort',
        timestep_s=args.dt, initial_speed_mps=args.initial_speed, tires=tires, drive=drive,
        initial_front_brake=args.initial_brake_demand, initial_rear_brake=args.initial_brake_demand,
        # Let unilateral rider supports settle before an expensive static solve.
        # Frequent early refinement helps the lumped plant but wastes solves on
        # an articulated pose whose feet/saddle have not settled yet.
        equilibrium_refine_after_s=3. if args.rider == 'articulated_planar' else .1,
        equilibrium_refine_period_s=3. if args.rider == 'articulated_planar' else .5)
    experiment = ExperimentConfig(control_period_s=args.control_period, actuator_delay_s=args.actuator_delay,
        duration_s=args.duration, seed=args.seed, record_decimation=args.record_decimation,
        maximum_energy_residual_ratio=args.energy_tolerance)
    sensors = SensorConfig.ideal() if args.ideal_sensors else SensorConfig(latency_s=args.sensor_delay)
    sim = RideSimulation(track=track, rider=RiderSpecs(variant=args.rider, mass_kg=args.rider_mass, height_m=args.rider_height),
                         physics_config=cfg)
    return ResearchEnvironment(sim, experiment, sensors)


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


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    env = None
    try:
        # Validate a complete command before compiling the plant or touching outputs.
        RideControl(motor_torque_nm=None if args.assist else args.motor_torque,
                    motor_limit_nm=args.motor_limit, human_torque_nm=args.human_torque)
        if args.out.exists() and any(args.out.iterdir()) and not args.overwrite:
            raise FileExistsError(f'output exists; choose another --out or pass --overwrite: {args.out}')
        env = make_environment(args)
        while not env.done:
            command = RideControl(motor_torque_nm=None if args.assist else args.motor_torque,
                motor_limit_nm=args.motor_limit, human_torque_nm=args.human_torque,
                posture=posture_at(args.posture, env.sim.time_s))
            env.step(command)
        env.save(args.out, overwrite=args.overwrite)
        print(json.dumps(dict(output=str(args.out), outcome=env.reason, metrics=env.tracker.metrics), indent=2))
        return 0 if env.reason in ('duration', 'finish') else 1
    except (ValueError, RuntimeError, ArithmeticError, OSError) as exc:
        if env is not None and env.reason == 'simulation_error':
            env.save(args.out, overwrite=args.overwrite)
        print(f'ERROR: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
