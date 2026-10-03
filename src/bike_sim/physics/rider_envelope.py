"""Explicit anatomical coordinate conventions and conservative soft joint edges.

No default anatomical range is claimed. A supplied profile must describe all
nine internal joints in radians and identify its provenance.
"""
from dataclasses import dataclass
from math import atan2, cos, isfinite, sin
from pathlib import Path
import json
import numpy as np

from bike_sim.mujoco.reference_rider import reference_joint_names

RIDER_JOINTS = reference_joint_names()
LEGACY_RIDER_JOINTS = ('rider_torso_hinge','rider_shoulder','rider_elbow') + tuple(
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


def anatomical_angle(envelope: JointEnvelope, joint_q: float) -> float:
    """Map a joint coordinate back to the declared anatomical angle.

    This is the exact inverse of ``joint_q_range``: the direction convention
    is applied to q before adding the neutral, then the result is normalized
    to a principal angle so a wrapped coordinate cannot fake being in range.
    """
    if not isinstance(envelope,JointEnvelope) or not isfinite(joint_q):
        raise ValueError('anatomical angle needs an envelope and a finite joint coordinate')
    value=envelope.neutral_anatomical_rad+envelope.direction*joint_q
    return atan2(sin(value),cos(value))


def reference_rom_check(joint_angles_rad, envelopes) -> tuple[str, ...]:
    """Names of joints whose anatomical angle leaves its declared range.

    ``joint_angles_rad`` are joint coordinates in q convention (the built pose
    has all internal hinges at q=0, i.e. at each declared neutral), which are
    converted to anatomical angles before comparison. Checking the built
    pose's own anatomical angles — not a round-trip of the declared range —
    is what catches an inconsistent envelope or a mis-signed direction.
    """
    if set(joint_angles_rad)!=set(envelopes):
        raise ValueError('ROM check needs one angle per declared joint')
    violations=[]
    for name,envelope in envelopes.items():
        if not isinstance(envelope,JointEnvelope):
            raise ValueError('ROM check needs JointEnvelope entries')
        q=float(joint_angles_rad[name])
        if not isfinite(q):
            raise ValueError('ROM check needs finite joint angles')
        angle=anatomical_angle(envelope,q)
        if not envelope.minimum_anatomical_rad-1e-12<=angle<=envelope.maximum_anatomical_rad+1e-12:
            violations.append(name)
    return tuple(violations)


def load_joint_envelopes(path: str) -> dict[str,JointEnvelope]:
    payload=json.loads(Path(path).read_text())
    if not isinstance(payload,dict):
        raise ValueError('joint profile must be an object')
    names = RIDER_JOINTS
    if 'joints' in payload:
        version=payload.get('schema_version',1)
        if version==2:
            if set(payload)!={'joints','schema_version','provenance','topology'}:
                raise ValueError('unknown joint profile schema')
            if payload['topology']!='articulated_planar_two_arm':
                raise ValueError('joint profile names a different rider topology')
        elif version==1:
            if set(payload)!={'joints','schema_version','provenance'}:
                raise ValueError('unknown joint profile schema')
            names = LEGACY_RIDER_JOINTS
        else:
            raise ValueError('unknown joint profile schema')
        payload=payload['joints']
    if not isinstance(payload,dict) or set(payload)!=set(names):
        raise ValueError(f'profile must cover exactly {len(names)} internal rider joints, never root joints')
    try:
        return {name:JointEnvelope(**payload[name]) for name in names}
    except TypeError as exc:
        raise ValueError('invalid joint envelope fields') from exc


def soft_edge_response(q, lower, upper, stiffness_nm_rad, margin_rad):
    """Torque = -dU/dq; inward soft edges precede, not replace, solver limits."""
    if (isinstance(q,(float,int,np.floating,np.integer))
            and isinstance(lower,(float,int,np.floating,np.integer))
            and isinstance(upper,(float,int,np.floating,np.integer))
            and isinstance(stiffness_nm_rad,(float,int,np.floating,np.integer))
            and isinstance(margin_rad,(float,int,np.floating,np.integer))):
        # Scalar fast path: identical validation and arithmetic without the
        # broadcast/concatenation ritual (np.float64 is already a float
        # subclass; float() normalizes ints and other numpy scalars).
        q,lo,hi=float(q),float(lower),float(upper)
        k,m=float(stiffness_nm_rad),float(margin_rad)
        if not (isfinite(q) and isfinite(lo) and isfinite(hi)
                and isfinite(k) and isfinite(m)):
            raise ValueError('joint edge state must be finite')
        if hi<=lo or k<0 or m<0:
            raise ValueError('invalid soft edge parameters')
        margin=min(m,(hi-lo)/2.)
        left=max(lo+margin-q,0.)
        right=max(q-hi+margin,0.)
        return k*(left-right),.5*k*(left*left+right*right)
    q,lo,hi=np.broadcast_arrays(np.asarray(q,float),lower,upper)
    if not np.isfinite(np.r_[q.ravel(),lo.ravel(),hi.ravel(),stiffness_nm_rad,margin_rad]).all():
        raise ValueError('joint edge state must be finite')
    if np.any(hi<=lo) or stiffness_nm_rad<0 or margin_rad<0:
        raise ValueError('invalid soft edge parameters')
    margin=np.minimum(margin_rad,(hi-lo)/2.)
    left=np.maximum(lo+margin-q,0.)
    right=np.maximum(q-hi+margin,0.)
    return stiffness_nm_rad*(left-right), .5*stiffness_nm_rad*(left*left+right*right)
