"""Synthetic crank effort profile; it is not a speed controller."""
from math import cos
from bike_sim.physics.checks import scalar


def human_crank_torque(mean_nm,phase_rad,ripple=.35):
    mean_nm = scalar(mean_nm,'mean human torque')
    phase_rad = scalar(phase_rad,'crank phase')
    ripple = scalar(ripple,'pedal ripple',minimum=0)
    if ripple >= 1:
        raise ValueError('pedal ripple must be less than one')
    return scalar(mean_nm*(1-ripple*cos(2*phase_rad)),'human torque')
