"""Demo plumbing policies for the batch runner: not anti-wheelie solutions.

They only show the `policy(observation, demand_nm) -> RideControl` wiring and
give the batch report something to compare against. None of them estimates or
prevents a wheelie; the real policy is the user's to write.
"""
from bike_sim.sim.ride.control import RideControl


def _request(demand_nm):
    # RideControl(motor_torque_nm=None) would silently select pedelec assistance.
    return 0. if demand_nm is None else float(demand_nm)


def passthrough(observation, demand_nm):
    return RideControl(motor_torque_nm=_request(demand_nm))


def zero(observation, demand_nm):
    return RideControl(motor_torque_nm=0.)


def fixed_limit_40(observation, demand_nm):
    return RideControl(motor_torque_nm=_request(demand_nm), motor_limit_nm=40.)


POLICIES = {'passthrough': passthrough, 'zero': zero, 'fixed_limit_40': fixed_limit_40}
