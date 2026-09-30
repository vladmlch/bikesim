"""Executable policy wiring example; the passthrough is not an anti-wheelie controller.

Policy signature: policy(observation, demand_nm) -> RideControl. `demand_nm` is the
scripted "the bike wants to go" torque (an advisory command echo, None when no
--demand/--demand-file is given), not a sensor. Try:
    python examples/research/controller_loop.py --scenario generated --seed 3 --demand 60 --duration 10
"""
from pathlib import Path
from bike_sim.cli.research import parser, make_environment
from bike_sim.sim.research.sensors import SensorObservation
from bike_sim.sim.ride.control import RideControl

MOTOR_CAP_NM = 80.0


def policy(observation: SensorObservation, demand_nm: float | None) -> RideControl:
    """Replace this passthrough with an estimator/policy using sensors only.

    motor_torque_nm is the request; motor_limit_nm is the immediate pedelec-style
    ceiling applied after the motor lag (lower it to cap assistance). human_torque_nm
    stays None so a rider program (--rider-random) can own the pedalling effort.
    """
    request = min(demand_nm, MOTOR_CAP_NM) if observation.valid and demand_nm is not None else 0.0
    return RideControl(motor_torque_nm=request, motor_limit_nm=MOTOR_CAP_NM)


def main() -> None:
    args = parser().parse_args()
    env = make_environment(args)
    while not env.done:
        transition = env.step(policy(env.observation, env.demand_nm))
        if not transition.numerically_valid:
            break
    env.save(args.out, overwrite=args.overwrite)
    print(f'{env.reason}: {Path(args.out).resolve()}')


if __name__ == '__main__':
    main()
