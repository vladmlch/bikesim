"""Executable policy wiring example; the ramp is not an anti-wheelie controller."""
from pathlib import Path
from bike_sim.cli.research import parser, make_environment
from bike_sim.sim.research.sensors import SensorObservation
from bike_sim.sim.ride.control import RideControl


def policy(observation: SensorObservation) -> RideControl:
    """Replace this demonstration ramp with an estimator/policy using sensors only."""
    request = 80.0*min(observation.time_s, 1.0) if observation.valid else 0.0
    return RideControl(motor_torque_nm=request, motor_limit_nm=80.0, human_torque_nm=0.0)


def main() -> None:
    args = parser().parse_args()
    env = make_environment(args)
    while not env.done:
        transition = env.step(policy(env.observation))
        if not transition.numerically_valid:
            break
    env.save(args.out, overwrite=args.overwrite)
    print(f'{env.reason}: {Path(args.out).resolve()}')


if __name__ == '__main__':
    main()
