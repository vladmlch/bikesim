"""Anatomical rider masses and geometry, independent of support load shares.

Fractions are the project's existing de Leva table, moved here as the canonical
anatomy source. Segment shapes, midpoint CoMs and contact padding are synthetic.
"""
from dataclasses import dataclass, replace
import numpy as np
from bike_sim.physics.checks import array, scalar

DE_LEVA_MASS_FRACTIONS = {
    'head': .0694, 'trunk_upper': .1596, 'trunk_middle': .1633,
    'trunk_lower': .1117, 'upper_arm': .0271, 'forearm': .0162,
    'hand': .0061, 'thigh': .1416, 'shank': .0433, 'foot': .0137,
}


def segment_masses(total_kg,helmet_kg):
    total = scalar(total_kg,'rider mass',positive=True)
    helmet = scalar(helmet_kg,'helmet mass',minimum=0)
    if helmet >= total:
        raise ValueError('helmet must be lighter than the rider')
    body,f = total-helmet,DE_LEVA_MASS_FRACTIONS
    result = {
        'pelvis':f['trunk_lower']*body,
        'torso':(f['trunk_upper']+f['trunk_middle'])*body,
        'head':f['head']*body+helmet,
        'upper_arm_pair':2*f['upper_arm']*body,
        'forearm_pair':2*(f['forearm']+f['hand'])*body,
    }
    for side in ('front','rear'):
        for segment in ('thigh','shank','foot'):
            result[f'{segment}_{side}'] = f[segment]*body
    if not np.isclose(sum(result.values()),total,rtol=0,atol=1e-10):
        raise ArithmeticError('anatomical mass budget does not close')
    return result


def segment_inertia(mass_kg,axis_vector_m,radius_m):
    mass = scalar(mass_kg,'segment mass',positive=True)
    axis = array(axis_vector_m,'segment axis',(3,))
    radius = scalar(radius_m,'segment radius',positive=True)
    length = float(np.linalg.norm(axis))
    if length <= 0:
        raise ValueError('segment length must be positive')
    unit = axis/length
    parallel = .5*mass*radius**2
    transverse = mass*(3*radius**2+length**2)/12
    return transverse*np.eye(3)+(parallel-transverse)*np.outer(unit,unit)


@dataclass(frozen=True)
class ArticulatedPose:
    saddle: object
    hip: np.ndarray
    shoulder: np.ndarray
    elbow: np.ndarray
    grip: np.ndarray
    head_center: np.ndarray
    knee_front: np.ndarray
    knee_rear: np.ndarray
    ankle_front: np.ndarray
    ankle_rear: np.ndarray
    pedal_front: np.ndarray
    pedal_rear: np.ndarray

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            if name != 'saddle':
                object.__setattr__(self,name,array(getattr(self,name),name,(3,),readonly=True))


def geometry_pose(rider,specs):
    """Use the existing fitting solver only for joint centres, never its masses."""
    neutral = replace(rider,variant='seated',legs='rigid',
                      saddle_share=.55,pedal_share=.33,bar_share=.12)
    pose = neutral.seated_pose(specs)
    return ArticulatedPose(**{name:getattr(pose,name) for name in ArticulatedPose.__dataclass_fields__})
