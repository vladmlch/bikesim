"""Grade feasibility and traction margin assessment."""
from bike_sim.physics.checks import scalar


def necessary_traction_margin(mu: float, rear_fraction: float, grade: float) -> float:
    """
    Calculate the necessary traction margin for steady-state 2-wheel climb.

    Returns the margin (mu * rear_fraction - grade). A negative value indicates
    that available traction at the rear wheel is insufficient to maintain steady climb
    at the given grade, even before drivetrain/rolling losses or dynamic disturbances.
    Note: this is a postprocessing/scenario feasibility check, not a motor torque command
    or sufficient condition for climb completion.
    """
    friction = scalar(mu, 'friction coefficient', minimum=0.)
    fraction = scalar(rear_fraction, 'rear normal-load fraction', minimum=0.)
    slope = scalar(grade, 'uphill grade', minimum=0.)
    if fraction > 1.:
        raise ValueError('rear normal-load fraction must not exceed one')
    return friction * fraction - slope
