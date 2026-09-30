"""Explicit anatomical coordinate conventions and conservative soft joint edges.

No default anatomical range is claimed. A supplied profile must describe all
nine internal joints in radians and identify its provenance.
"""
from dataclasses import dataclass
from math import isfinite
from pathlib import Path
import json
import numpy as np

RIDER_JOINTS = ('rider_torso_hinge','rider_shoulder','rider_elbow') + tuple(
    f'rider_{joint}_{side}' for side in ('front','rear') for joint in ('hip','knee','ankle'))


@dataclass(frozen=True)
class JointEnvelope:
    neutral_anatomical_rad: float
    direction: int
    minimum_anatomical_rad: float
    maximum_anatomical_rad: float
    provenance: str

    def __post_init__(self):
        values=(self.neutral_anatomical_rad,self.minimum_anatomical_rad,self.maximum_anatomical_rad)
        if not all(isfinite(v) for v in values) or type(self.direction) is not int or self.direction not in (-1,1):
            raise ValueError('invalid joint coordinate convention')
        if not self.minimum_anatomical_rad < self.maximum_anatomical_rad:
            raise ValueError('empty joint range')
        if not isinstance(self.provenance,str) or not self.provenance.strip():
            raise ValueError('joint range provenance is required')


def joint_q_range(envelope: JointEnvelope) -> tuple[float,float]:
    a=(envelope.minimum_anatomical_rad-envelope.neutral_anatomical_rad)/envelope.direction
    b=(envelope.maximum_anatomical_rad-envelope.neutral_anatomical_rad)/envelope.direction
    return min(a,b),max(a,b)


def load_joint_envelopes(path: str) -> dict[str,JointEnvelope]:
    payload=json.loads(Path(path).read_text())
    if not isinstance(payload,dict):
        raise ValueError('joint profile must be an object')
    if 'joints' in payload:
        if set(payload)-{'joints','schema_version','provenance'} or payload.get('schema_version',1)!=1:
            raise ValueError('unknown joint profile schema')
        payload=payload['joints']
    if not isinstance(payload,dict) or set(payload)!=set(RIDER_JOINTS):
        raise ValueError('profile must cover exactly nine internal rider joints, never root joints')
    try:
        return {name:JointEnvelope(**payload[name]) for name in RIDER_JOINTS}
    except TypeError as exc:
        raise ValueError('invalid joint envelope fields') from exc


def soft_edge_response(q, lower, upper, stiffness_nm_rad, margin_rad):
    """Torque = -dU/dq; inward soft edges precede, not replace, solver limits."""
    q,lo,hi=np.broadcast_arrays(np.asarray(q,float),lower,upper)
    if not np.isfinite(np.r_[q.ravel(),lo.ravel(),hi.ravel(),stiffness_nm_rad,margin_rad]).all():
        raise ValueError('joint edge state must be finite')
    if np.any(hi<=lo) or stiffness_nm_rad<0 or margin_rad<0:
        raise ValueError('invalid soft edge parameters')
    margin=np.minimum(margin_rad,(hi-lo)/2.)
    left=np.maximum(lo+margin-q,0.)
    right=np.maximum(q-hi+margin,0.)
    return stiffness_nm_rad*(left-right), .5*stiffness_nm_rad*(left*left+right*right)
